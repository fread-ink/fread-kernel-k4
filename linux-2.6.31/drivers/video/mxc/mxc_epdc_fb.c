/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 * Copyright 2010-2012 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
/*
 * Based on STMP378X LCDIF
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */
#ifdef __ARM_NEON__
#include <arm_neon.h>
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/cpufreq.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/dmaengine.h>
#include <linux/pxp_dma.h>
#include <linux/mxcfb.h>
#include <linux/mxcfb_epdc_kernel.h>
#include <linux/gpio.h>
#include <linux/regulator/driver.h>
#include <linux/fsl_devices.h>
#include <linux/bitops.h>
#include <mach/boardid.h>

#include "epdc_regs.h"
#include "cmap_lab126.h"

#define NUM_SCREENS_MIN 2 
#define EPDC_NUM_LUTS 16
#define EPDC_MAX_NUM_UPDATES 16
#define EPDC_MAX_NUM_BUFFERS 2
#define INVALID_LUT -1

#define DEFAULT_TEMP_INDEX	0  /* Lab126: 8 -> 0 to support 25C-only waveforms */
#define DEFAULT_TEMP		20 /* room temp in deg Celsius */

#define INIT_UPDATE_MARKER	0x12345678
#define PAN_UPDATE_MARKER	0x12345679

#define POWER_STATE_OFF	0
#define POWER_STATE_ON	1
#define POWER_STATE_GOING_UP 2

#define MERGE_OK	0
#define MERGE_FAIL	1
#define MERGE_BLOCK	2

static unsigned long default_bpp = 16;
static int mxc_epdc_paused = 0;

extern int papyrus_temp;

struct update_marker_data {
	struct list_head full_list;
	struct list_head upd_list;
	u32 update_marker;
	struct completion update_completion;
	struct completion submit_completion;
	int lut_num;
	bool waiting;
	bool submitted;
	unsigned long long start_time;
};

struct update_desc_list {
	struct list_head list;
	struct mxcfb_update_data upd_data;/* Update parameters */
	u32 epdc_offs;		/* Added to buffer ptr to resolve alignment */
	struct list_head upd_marker_list; /* List of markers for this update */
	u32 update_order;	/* Numeric ordering value for update */
};

/* This structure represents a list node containing both
 * a memory region allocated as an output buffer for the PxP
 * update processing task, and the update description (mode, region, etc.) */
struct update_data_list {
	struct list_head list;
	dma_addr_t phys_addr;			/* Pointer to phys address of processed Y buf */
	void *virt_addr;
	struct update_desc_list *update_desc;
	int lut_num;				/* Assigned before update is processed into working buffer */
	int collision_mask;			/* Set when update results in collision */
						/* Represents other LUTs that we collide with */
};

struct mxc_epdc_fb_data {
	struct fb_info info;
	struct fb_var_screeninfo epdc_fb_var;
	u32 pseudo_palette[16];
	char *fb_panel_str;
	struct list_head list;
	struct mxc_epdc_fb_mode *cur_mode;
	struct mxc_epdc_fb_platform_data *pdata;
	int blank;
	u32 max_pix_size;
	ssize_t map_size;
	dma_addr_t phys_start;
	u32 fb_offset;
	int default_bpp;
	int native_width;
	int native_height;
	int num_screens;
	int epdc_irq;
	struct device *dev;
	wait_queue_head_t vsync_wait_q;
	u32 vsync_count;
	void *par;
	int power_state;
	int wait_for_powerdown;
	struct completion powerdown_compl;
	struct clk *epdc_clk_axi;
	struct clk *epdc_clk_pix;
	struct regulator *display_regulator;
	struct regulator *vcom_regulator;

	/* FB elements related to EPDC updates */
	bool in_init;
	bool hw_ready;
	bool waiting_for_idle;
	u32 auto_mode;
	u32 upd_scheme;
	struct list_head upd_pending_list;
	struct list_head upd_buf_queue;
	struct list_head upd_buf_free_list;
	struct list_head upd_buf_collision_list;
	struct update_data_list *cur_update;
	struct mutex queue_mutex; 		
	int trt_entries;
	int temp_index;
	u8 *temp_range_bounds;
	struct mxcfb_waveform_modes wv_modes;
	u32 *waveform_buffer_virt;
	u32 waveform_buffer_phys;
	u32 waveform_buffer_size;
	u32 *working_buffer_virt;
	u32 working_buffer_phys;
	u32 working_buffer_size;
	dma_addr_t *phys_addr_updbuf;
	void **virt_addr_updbuf;
	u32 upd_buffer_num;
	u32 max_num_buffers;
	dma_addr_t phys_addr_copybuf;		/* Phys address of copied update data */
	void *virt_addr_copybuf;	/* Used for PxP SW workaround */
	u32 order_cnt;
	struct list_head full_marker_list;
	u32 lut_update_order[EPDC_NUM_LUTS];
	u32 luts_complete_wb;
	struct completion updates_done;
	struct delayed_work epdc_done_work;
	struct workqueue_struct *epdc_submit_workqueue;
	struct work_struct epdc_submit_work;
	struct workqueue_struct *epdc_intr_workqueue; 
	struct work_struct epdc_intr_work;	
	bool waiting_for_wb;
	bool waiting_for_lut;
	bool waiting_for_lut15;
	struct completion update_res_free;
	struct completion lut15_free;
	struct completion eof_event;
	int eof_sync_period;
	struct mutex power_mutex;
	bool powering_down;
	bool updates_active;
	int pwrdown_delay;
	unsigned long tce_prevent;

	/* FB elements related to PxP DMA */
	struct completion pxp_tx_cmpl;
	struct pxp_channel *pxp_chan;
	struct pxp_config_data pxp_conf;
	struct dma_async_tx_descriptor *txd;
	dma_cookie_t cookie;
	struct scatterlist sg[2];
	struct mutex pxp_mutex; /* protects access to PxP */

	/* Lab126 */
	struct mxcfb_waveform_data_file *wv_file;
	char *wv_file_name;
	int wv_file_size;
	u32 waveform_type;
};

struct waveform_data_header {
	unsigned int wi0;
	unsigned int wi1;
	unsigned int wi2;
	unsigned int wi3;
	unsigned int wi4;
	unsigned int wi5;
	unsigned int wi6;
	unsigned int xwia:24;
	unsigned int cs1:8;
	unsigned int wmta:24;
	unsigned int fvsn:8;
	unsigned int luts:8;
	unsigned int mc:8;
	unsigned int trc:8;
	unsigned int reserved0_0:8;
	unsigned int eb:8;
	unsigned int sb:8;
	unsigned int reserved0_1:8;
	unsigned int reserved0_2:8;
	unsigned int reserved0_3:8;
	unsigned int reserved0_4:8;
	unsigned int reserved0_5:8;
	unsigned int cs2:8;
};

struct mxcfb_waveform_data_file {
	struct waveform_data_header wdh;
	u32 *data;	/* Temperature Range Table + Waveform Data */
};

void __iomem *epdc_base;

static int mxc_epdc_marker = 0; /* To be used only for X */

/* Lab126 */
static int dont_register_fb = 0;
/*
 * Enable this parameter to have a default panel
 * loaded during driver initialization
 */
static int default_panel_hw_init = 0;
static int default_update_mode = 0;
static char *wf_to_use = NULL;
static int use_builtin_cmap = 0; // False by default to not impact legacy products.
static int use_cmap = 0; // False by default to not impact legacy products.

static int mxc_epdc_debugging = 0;
atomic_t mxc_clear_queue = ATOMIC_INIT(0);

#ifdef MODULE
module_param_named(dont_register_fb, dont_register_fb, int, S_IRUGO);
MODULE_PARM_DESC(dont_register_fb, "non-zero to not register");
module_param_named(default_panel_hw_init, default_panel_hw_init, int, S_IRUGO);
MODULE_PARM_DESC(default_panel_hw_init, "Default hardware initialization");
module_param_named(default_update_mode, default_update_mode, int, S_IRUGO);
MODULE_PARM_DESC(default_update_mode, "Default update mode");
module_param_named(waveform_to_use, wf_to_use, charp, S_IRUGO);
MODULE_PARM_DESC(waveform_to_use, "/path/to/waveform_file or built-in");
module_param_named(use_builtin_cmap, use_builtin_cmap, int, S_IRUGO);
MODULE_PARM_DESC(use_builtin_cmap, "Use builtin ColorMap");
#endif

struct mxc_epdc_fb_data *g_fb_data = NULL;

/* forward declaration */
static bool is_free_list_full(struct mxc_epdc_fb_data *fb_data); /* Lab126 */
static int mxc_epdc_fb_get_temp_index(struct mxc_epdc_fb_data *fb_data, int temp); /* Lab126 */
static void mxc_epdc_fb_flush_updates(struct mxc_epdc_fb_data *fb_data);
static int mxc_epdc_fb_blank(int blank, struct fb_info *info);
static int mxc_epdc_fb_init_hw(struct fb_info *info);
static int pxp_process_update(struct mxc_epdc_fb_data *fb_data,
			      u32 src_width, u32 src_height,
			      struct mxcfb_rect *update_region);
static int pxp_complete_update(struct mxc_epdc_fb_data *fb_data, u32 *hist_stat);
static void draw_mode0(struct mxc_epdc_fb_data *fb_data);
static bool is_free_list_full(struct mxc_epdc_fb_data *fb_data);

#ifdef DEBUG
static void dump_pxp_config(struct mxc_epdc_fb_data *fb_data,
			    struct pxp_config_data *pxp_conf)
{
	dev_info(fb_data->dev, "S0 fmt 0x%x",
		pxp_conf->s0_param.pixel_fmt);
	dev_info(fb_data->dev, "S0 width 0x%x",
		pxp_conf->s0_param.width);
	dev_info(fb_data->dev, "S0 height 0x%x",
		pxp_conf->s0_param.height);
	dev_info(fb_data->dev, "S0 ckey 0x%x",
		pxp_conf->s0_param.color_key);
	dev_info(fb_data->dev, "S0 ckey en 0x%x",
		pxp_conf->s0_param.color_key_enable);

	dev_info(fb_data->dev, "OL0 combine en 0x%x",
		pxp_conf->ol_param[0].combine_enable);
	dev_info(fb_data->dev, "OL0 fmt 0x%x",
		pxp_conf->ol_param[0].pixel_fmt);
	dev_info(fb_data->dev, "OL0 width 0x%x",
		pxp_conf->ol_param[0].width);
	dev_info(fb_data->dev, "OL0 height 0x%x",
		pxp_conf->ol_param[0].height);
	dev_info(fb_data->dev, "OL0 ckey 0x%x",
		pxp_conf->ol_param[0].color_key);
	dev_info(fb_data->dev, "OL0 ckey en 0x%x",
		pxp_conf->ol_param[0].color_key_enable);
	dev_info(fb_data->dev, "OL0 alpha 0x%x",
		pxp_conf->ol_param[0].global_alpha);
	dev_info(fb_data->dev, "OL0 alpha en 0x%x",
		pxp_conf->ol_param[0].global_alpha_enable);
	dev_info(fb_data->dev, "OL0 local alpha en 0x%x",
		pxp_conf->ol_param[0].local_alpha_enable);

	dev_info(fb_data->dev, "Out fmt 0x%x",
		pxp_conf->out_param.pixel_fmt);
	dev_info(fb_data->dev, "Out width 0x%x",
		pxp_conf->out_param.width);
	dev_info(fb_data->dev, "Out height 0x%x",
		pxp_conf->out_param.height);

	dev_info(fb_data->dev,
		"drect left 0x%x right 0x%x width 0x%x height 0x%x",
		pxp_conf->proc_data.drect.left, pxp_conf->proc_data.drect.top,
		pxp_conf->proc_data.drect.width,
		pxp_conf->proc_data.drect.height);
	dev_info(fb_data->dev,
		"srect left 0x%x right 0x%x width 0x%x height 0x%x",
		pxp_conf->proc_data.srect.left, pxp_conf->proc_data.srect.top,
		pxp_conf->proc_data.srect.width,
		pxp_conf->proc_data.srect.height);
	dev_info(fb_data->dev, "Scaling en 0x%x", pxp_conf->proc_data.scaling);
	dev_info(fb_data->dev, "HFlip en 0x%x", pxp_conf->proc_data.hflip);
	dev_info(fb_data->dev, "VFlip en 0x%x", pxp_conf->proc_data.vflip);
	dev_info(fb_data->dev, "Rotation 0x%x", pxp_conf->proc_data.rotate);
	dev_info(fb_data->dev, "BG Color 0x%x", pxp_conf->proc_data.bgcolor);
}

static void dump_epdc_reg(void)
{
	printk(KERN_DEBUG "\n\n");
	printk(KERN_DEBUG "EPDC_CTRL 0x%x\n", __raw_readl(EPDC_CTRL));
	printk(KERN_DEBUG "EPDC_WVADDR 0x%x\n", __raw_readl(EPDC_WVADDR));
	printk(KERN_DEBUG "EPDC_WB_ADDR 0x%x\n", __raw_readl(EPDC_WB_ADDR));
	printk(KERN_DEBUG "EPDC_RES 0x%x\n", __raw_readl(EPDC_RES));
	printk(KERN_DEBUG "EPDC_FORMAT 0x%x\n", __raw_readl(EPDC_FORMAT));
	printk(KERN_DEBUG "EPDC_FIFOCTRL 0x%x\n", __raw_readl(EPDC_FIFOCTRL));
	printk(KERN_DEBUG "EPDC_UPD_ADDR 0x%x\n", __raw_readl(EPDC_UPD_ADDR));
	printk(KERN_DEBUG "EPDC_UPD_FIXED 0x%x\n", __raw_readl(EPDC_UPD_FIXED));
	printk(KERN_DEBUG "EPDC_UPD_CORD 0x%x\n", __raw_readl(EPDC_UPD_CORD));
	printk(KERN_DEBUG "EPDC_UPD_SIZE 0x%x\n", __raw_readl(EPDC_UPD_SIZE));
	printk(KERN_DEBUG "EPDC_UPD_CTRL 0x%x\n", __raw_readl(EPDC_UPD_CTRL));
	printk(KERN_DEBUG "EPDC_TEMP 0x%x\n", __raw_readl(EPDC_TEMP));
	printk(KERN_DEBUG "EPDC_TCE_CTRL 0x%x\n", __raw_readl(EPDC_TCE_CTRL));
	printk(KERN_DEBUG "EPDC_TCE_SDCFG 0x%x\n", __raw_readl(EPDC_TCE_SDCFG));
	printk(KERN_DEBUG "EPDC_TCE_GDCFG 0x%x\n", __raw_readl(EPDC_TCE_GDCFG));
	printk(KERN_DEBUG "EPDC_TCE_HSCAN1 0x%x\n", __raw_readl(EPDC_TCE_HSCAN1));
	printk(KERN_DEBUG "EPDC_TCE_HSCAN2 0x%x\n", __raw_readl(EPDC_TCE_HSCAN2));
	printk(KERN_DEBUG "EPDC_TCE_VSCAN 0x%x\n", __raw_readl(EPDC_TCE_VSCAN));
	printk(KERN_DEBUG "EPDC_TCE_OE 0x%x\n", __raw_readl(EPDC_TCE_OE));
	printk(KERN_DEBUG "EPDC_TCE_POLARITY 0x%x\n", __raw_readl(EPDC_TCE_POLARITY));
	printk(KERN_DEBUG "EPDC_TCE_TIMING1 0x%x\n", __raw_readl(EPDC_TCE_TIMING1));
	printk(KERN_DEBUG "EPDC_TCE_TIMING2 0x%x\n", __raw_readl(EPDC_TCE_TIMING2));
	printk(KERN_DEBUG "EPDC_TCE_TIMING3 0x%x\n", __raw_readl(EPDC_TCE_TIMING3));
	printk(KERN_DEBUG "EPDC_IRQ_MASK 0x%x\n", __raw_readl(EPDC_IRQ_MASK));
	printk(KERN_DEBUG "EPDC_IRQ 0x%x\n", __raw_readl(EPDC_IRQ));
	printk(KERN_DEBUG "EPDC_STATUS_LUTS 0x%x\n", __raw_readl(EPDC_STATUS_LUTS));
	printk(KERN_DEBUG "EPDC_STATUS_NEXTLUT 0x%x\n", __raw_readl(EPDC_STATUS_NEXTLUT));
	printk(KERN_DEBUG "EPDC_STATUS_COL 0x%x\n", __raw_readl(EPDC_STATUS_COL));
	printk(KERN_DEBUG "EPDC_STATUS 0x%x\n", __raw_readl(EPDC_STATUS));
	printk(KERN_DEBUG "EPDC_DEBUG 0x%x\n", __raw_readl(EPDC_DEBUG));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT0 0x%x\n", __raw_readl(EPDC_DEBUG_LUT0));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT1 0x%x\n", __raw_readl(EPDC_DEBUG_LUT1));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT2 0x%x\n", __raw_readl(EPDC_DEBUG_LUT2));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT3 0x%x\n", __raw_readl(EPDC_DEBUG_LUT3));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT4 0x%x\n", __raw_readl(EPDC_DEBUG_LUT4));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT5 0x%x\n", __raw_readl(EPDC_DEBUG_LUT5));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT6 0x%x\n", __raw_readl(EPDC_DEBUG_LUT6));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT7 0x%x\n", __raw_readl(EPDC_DEBUG_LUT7));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT8 0x%x\n", __raw_readl(EPDC_DEBUG_LUT8));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT9 0x%x\n", __raw_readl(EPDC_DEBUG_LUT9));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT10 0x%x\n", __raw_readl(EPDC_DEBUG_LUT10));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT11 0x%x\n", __raw_readl(EPDC_DEBUG_LUT11));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT12 0x%x\n", __raw_readl(EPDC_DEBUG_LUT12));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT13 0x%x\n", __raw_readl(EPDC_DEBUG_LUT13));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT14 0x%x\n", __raw_readl(EPDC_DEBUG_LUT14));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT15 0x%x\n", __raw_readl(EPDC_DEBUG_LUT15));
	printk(KERN_DEBUG "EPDC_GPIO 0x%x\n", __raw_readl(EPDC_GPIO));
	printk(KERN_DEBUG "EPDC_VERSION 0x%x\n", __raw_readl(EPDC_VERSION));
	printk(KERN_DEBUG "\n\n");
}

static void dump_update_data(struct device *dev,
			     struct update_data_list *upd_data_list)
{
	dev_info(dev,
		"X = %d, Y = %d, Width = %d, Height = %d, WaveMode = %d, "
		"LUT = %d, Coll Mask = 0x%x, order = %d\n",
		upd_data_list->update_desc->upd_data.update_region.left,
		upd_data_list->update_desc->upd_data.update_region.top,
		upd_data_list->update_desc->upd_data.update_region.width,
		upd_data_list->update_desc->upd_data.update_region.height,
		upd_data_list->update_desc->upd_data.waveform_mode,
		upd_data_list->lut_num,
		upd_data_list->collision_mask,
		upd_data_list->update_desc->update_order);
}

static void dump_collision_list(struct mxc_epdc_fb_data *fb_data)
{
	struct update_data_list *plist;

	dev_info(fb_data->dev, "Collision List:\n");
	if (list_empty(&fb_data->upd_buf_collision_list))
		dev_info(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_buf_collision_list, list) {
		dev_info(fb_data->dev, "Virt Addr = 0x%x, Phys Addr = 0x%x ",
			(u32)plist->virt_addr, plist->phys_addr);
		dump_update_data(fb_data->dev, plist);
	}
}

static void dump_free_list(struct mxc_epdc_fb_data *fb_data)
{
	struct update_data_list *plist;

	dev_info(fb_data->dev, "Free List:\n");
	if (list_empty(&fb_data->upd_buf_free_list))
		dev_info(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_buf_free_list, list)
		dev_info(fb_data->dev, "Virt Addr = 0x%x, Phys Addr = 0x%x ",
			(u32)plist->virt_addr, plist->phys_addr);
}

static void dump_queue(struct mxc_epdc_fb_data *fb_data)
{
	struct update_data_list *plist;

	dev_info(fb_data->dev, "Queue:\n");
	if (list_empty(&fb_data->upd_buf_queue))
		dev_info(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_buf_queue, list) {
		dev_info(fb_data->dev, "Virt Addr = 0x%x, Phys Addr = 0x%x ",
			(u32)plist->virt_addr, plist->phys_addr);
		dump_update_data(fb_data->dev, plist);
	}
}

static void dump_desc_data(struct device *dev,
			     struct update_desc_list *upd_desc_list)
{
	dev_info(dev,
		"X = %d, Y = %d, Width = %d, Height = %d, WaveMode = %d, "
		"order = %d\n",
		upd_desc_list->upd_data.update_region.left,
		upd_desc_list->upd_data.update_region.top,
		upd_desc_list->upd_data.update_region.width,
		upd_desc_list->upd_data.update_region.height,
		upd_desc_list->upd_data.waveform_mode,
		upd_desc_list->update_order);
}

static void dump_pending_list(struct mxc_epdc_fb_data *fb_data)
{
	struct update_desc_list *plist;

	dev_info(fb_data->dev, "Queue:\n");
	if (list_empty(&fb_data->upd_pending_list))
		dev_info(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_pending_list, list)
		dump_desc_data(fb_data->dev, plist);
}

static void dump_all_updates(struct mxc_epdc_fb_data *fb_data)
{
	dump_free_list(fb_data);
	dump_queue(fb_data);
	dump_collision_list(fb_data);
	dev_info(fb_data->dev, "Current update being processed:\n");
	if (fb_data->cur_update == NULL)
		dev_info(fb_data->dev, "No current update\n");
	else
		dump_update_data(fb_data->dev, fb_data->cur_update);
}
#else
static inline void dump_pxp_config(struct mxc_epdc_fb_data *fb_data,
				   struct pxp_config_data *pxp_conf) {}
static inline void dump_epdc_reg(void) {}
static inline void dump_update_data(struct device *dev,
			     struct update_data_list *upd_data_list) {}
static inline void dump_collision_list(struct mxc_epdc_fb_data *fb_data) {}
static inline void dump_free_list(struct mxc_epdc_fb_data *fb_data) {}
static inline void dump_queue(struct mxc_epdc_fb_data *fb_data) {}
static inline void dump_all_updates(struct mxc_epdc_fb_data *fb_data) {}

#endif

static long long timeofday_msec(void)
{
        struct timeval tv;

        do_gettimeofday(&tv);
        return (long long)tv.tv_sec*1000 + tv.tv_usec/1000;
}

/********************************************************
 * Start Low-Level EPDC Functions
 ********************************************************/

static inline void epdc_clear_interrupts(void) /* Lab126 */
{
	__raw_writel(0, EPDC_IRQ_CLEAR);
	__raw_writel(0, EPDC_IRQ_MASK);
	__raw_writel(0, EPDC_IRQ);
}

static inline void epdc_lut_complete_intr(u32 lut_num, bool enable)
{
	if (enable)
		__raw_writel(1 << lut_num, EPDC_IRQ_MASK_SET);
	else
		__raw_writel(1 << lut_num, EPDC_IRQ_MASK_CLEAR);
}

static inline void epdc_working_buf_intr(bool enable)
{
	if (enable)
		__raw_writel(EPDC_IRQ_WB_CMPLT_IRQ, EPDC_IRQ_MASK_SET);
	else
		__raw_writel(EPDC_IRQ_WB_CMPLT_IRQ, EPDC_IRQ_MASK_CLEAR);
}

static inline void epdc_clear_working_buf_irq(void)
{
	__raw_writel(EPDC_IRQ_WB_CMPLT_IRQ | EPDC_IRQ_LUT_COL_IRQ,
		     EPDC_IRQ_CLEAR);
}

static inline void epdc_eof_intr(bool enable)
{
	if (enable)
		__raw_writel(EPDC_IRQ_FRAME_END_IRQ, EPDC_IRQ_MASK_SET);
	else
		__raw_writel(EPDC_IRQ_FRAME_END_IRQ, EPDC_IRQ_MASK_CLEAR);
}

static inline void epdc_clear_eof_irq(void)
{
	__raw_writel(EPDC_IRQ_FRAME_END_IRQ, EPDC_IRQ_CLEAR);
}

static inline bool epdc_signal_eof(void)
{
	return (__raw_readl(EPDC_IRQ_MASK) & __raw_readl(EPDC_IRQ)
		& EPDC_IRQ_FRAME_END_IRQ) ? true : false;
}

static inline void epdc_set_temp(u32 temp)
{
	__raw_writel(temp, EPDC_TEMP);
}

static inline void epdc_set_screen_res(u32 width, u32 height)
{
	u32 val = (height << EPDC_RES_VERTICAL_OFFSET) | width;
	__raw_writel(val, EPDC_RES);
}

static inline void epdc_set_update_addr(u32 addr)
{
	__raw_writel(addr, EPDC_UPD_ADDR);
}

static inline void epdc_set_update_coord(u32 x, u32 y)
{
	u32 val = (y << EPDC_UPD_CORD_YCORD_OFFSET) | x;
	__raw_writel(val, EPDC_UPD_CORD);
}

static inline void epdc_set_update_dimensions(u32 width, u32 height)
{
	u32 val = (height << EPDC_UPD_SIZE_HEIGHT_OFFSET) | width;
	__raw_writel(val, EPDC_UPD_SIZE);
}

static void epdc_submit_update(u32 lut_num, u32 waveform_mode, u32 update_mode,
			       bool use_test_mode, u32 np_val)
{
	u32 reg_val = 0;

	if (use_test_mode) {
		reg_val |=
		    ((np_val << EPDC_UPD_FIXED_FIXNP_OFFSET) &
		     EPDC_UPD_FIXED_FIXNP_MASK) | EPDC_UPD_FIXED_FIXNP_EN;

		__raw_writel(reg_val, EPDC_UPD_FIXED);

		reg_val = EPDC_UPD_CTRL_USE_FIXED;
	} else {
		__raw_writel(reg_val, EPDC_UPD_FIXED);
	}

	reg_val |=
	    ((lut_num << EPDC_UPD_CTRL_LUT_SEL_OFFSET) &
	     EPDC_UPD_CTRL_LUT_SEL_MASK) |
	    ((waveform_mode << EPDC_UPD_CTRL_WAVEFORM_MODE_OFFSET) &
	     EPDC_UPD_CTRL_WAVEFORM_MODE_MASK) |
	    update_mode;

	__raw_writel(reg_val, EPDC_UPD_CTRL);
}

static inline bool epdc_is_lut_complete(u32 lut_num)
{
	u32 val = __raw_readl(EPDC_IRQ);
	bool is_compl = val & (1 << lut_num) ? true : false;

	return is_compl;
}

static inline void epdc_clear_lut_complete_irq(u32 lut_num)
{
	__raw_writel(1 << lut_num, EPDC_IRQ_CLEAR);
}

static inline bool epdc_is_lut_active(u32 lut_num)
{
	u32 val = __raw_readl(EPDC_STATUS_LUTS);
	bool is_active = val & (1 << lut_num) ? true : false;

	return is_active;
}

static inline bool epdc_any_luts_active(void)
{
	bool any_active = __raw_readl(EPDC_STATUS_LUTS) ? true : false;

	return any_active;
}

static inline bool epdc_any_luts_available(void)
{
	bool luts_available =
	    (__raw_readl(EPDC_STATUS_NEXTLUT) &
	     EPDC_STATUS_NEXTLUT_NEXT_LUT_VALID) ? true : false;
	return luts_available;
}

static inline int epdc_get_next_lut(void)
{
	u32 val =
	    __raw_readl(EPDC_STATUS_NEXTLUT) &
	    EPDC_STATUS_NEXTLUT_NEXT_LUT_MASK;
	return val;
}

static int epdc_choose_next_lut(int *next_lut)
{
	u32 luts_status = __raw_readl(EPDC_STATUS_LUTS);

	*next_lut = fls(luts_status & 0xFFFF);

	if (*next_lut > 15)
		*next_lut = ffz(luts_status & 0xFFFF);

	if (luts_status & 0x8000)
		return 1;
	else
		return 0;
}

static inline bool epdc_is_working_buffer_busy(void)
{
	u32 val = __raw_readl(EPDC_STATUS);
	bool is_busy = (val & EPDC_STATUS_WB_BUSY) ? true : false;

	return is_busy;
}

static inline bool epdc_is_working_buffer_complete(void)
{
	u32 val = __raw_readl(EPDC_IRQ);
	bool is_compl = (val & EPDC_IRQ_WB_CMPLT_IRQ) ? true : false;

	return is_compl;
}

static inline bool epdc_is_collision(void)
{
	u32 val = __raw_readl(EPDC_IRQ);
	return (val & EPDC_IRQ_LUT_COL_IRQ) ? true : false;
}

static inline int epdc_get_colliding_luts(void)
{
	u32 val = __raw_readl(EPDC_STATUS_COL);
	return val;
}

static void epdc_set_horizontal_timing(u32 horiz_start, u32 horiz_end,
				       u32 hsync_width, u32 hsync_line_length)
{
	u32 reg_val =
	    ((hsync_width << EPDC_TCE_HSCAN1_LINE_SYNC_WIDTH_OFFSET) &
	     EPDC_TCE_HSCAN1_LINE_SYNC_WIDTH_MASK)
	    | ((hsync_line_length << EPDC_TCE_HSCAN1_LINE_SYNC_OFFSET) &
	       EPDC_TCE_HSCAN1_LINE_SYNC_MASK);
	__raw_writel(reg_val, EPDC_TCE_HSCAN1);

	reg_val =
	    ((horiz_start << EPDC_TCE_HSCAN2_LINE_BEGIN_OFFSET) &
	     EPDC_TCE_HSCAN2_LINE_BEGIN_MASK)
	    | ((horiz_end << EPDC_TCE_HSCAN2_LINE_END_OFFSET) &
	       EPDC_TCE_HSCAN2_LINE_END_MASK);
	__raw_writel(reg_val, EPDC_TCE_HSCAN2);
}

static void epdc_set_vertical_timing(u32 vert_start, u32 vert_end,
				     u32 vsync_width)
{
	u32 reg_val =
	    ((vert_start << EPDC_TCE_VSCAN_FRAME_BEGIN_OFFSET) &
	     EPDC_TCE_VSCAN_FRAME_BEGIN_MASK)
	    | ((vert_end << EPDC_TCE_VSCAN_FRAME_END_OFFSET) &
	       EPDC_TCE_VSCAN_FRAME_END_MASK)
	    | ((vsync_width << EPDC_TCE_VSCAN_FRAME_SYNC_OFFSET) &
	       EPDC_TCE_VSCAN_FRAME_SYNC_MASK);
	__raw_writel(reg_val, EPDC_TCE_VSCAN);
}

void epdc_init_settings(struct mxc_epdc_fb_data *fb_data)
{
	struct mxc_epdc_fb_mode *epdc_mode = fb_data->cur_mode;
	struct fb_var_screeninfo *screeninfo = &fb_data->epdc_fb_var;
	u32 reg_val;
	int num_ce;

	/* Reset */
	__raw_writel(EPDC_CTRL_SFTRST, EPDC_CTRL_SET);
	while (!(__raw_readl(EPDC_CTRL) & EPDC_CTRL_CLKGATE))
		;
	__raw_writel(EPDC_CTRL_SFTRST, EPDC_CTRL_CLEAR);

	/* Enable clock gating (clear to enable) */
	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_CLEAR);
	while (__raw_readl(EPDC_CTRL) & (EPDC_CTRL_SFTRST | EPDC_CTRL_CLKGATE))
		;

	/* EPDC_CTRL */
	reg_val = __raw_readl(EPDC_CTRL);
	reg_val &= ~EPDC_CTRL_UPD_DATA_SWIZZLE_MASK;
	reg_val |= EPDC_CTRL_UPD_DATA_SWIZZLE_NO_SWAP;
	reg_val &= ~EPDC_CTRL_LUT_DATA_SWIZZLE_MASK;
	reg_val |= EPDC_CTRL_LUT_DATA_SWIZZLE_NO_SWAP;
	__raw_writel(reg_val, EPDC_CTRL_SET);

	/* EPDC_FORMAT - 2bit TFT and 4bit Buf pixel format */
	reg_val = EPDC_FORMAT_TFT_PIXEL_FORMAT_2BIT
	    | EPDC_FORMAT_BUF_PIXEL_FORMAT_P4N
	    | ((0x0 << EPDC_FORMAT_DEFAULT_TFT_PIXEL_OFFSET) &
	       EPDC_FORMAT_DEFAULT_TFT_PIXEL_MASK);
	__raw_writel(reg_val, EPDC_FORMAT);

	/* EPDC_FIFOCTRL (disabled) */
	reg_val =
	    ((100 << EPDC_FIFOCTRL_FIFO_INIT_LEVEL_OFFSET) &
	     EPDC_FIFOCTRL_FIFO_INIT_LEVEL_MASK)
	    | ((200 << EPDC_FIFOCTRL_FIFO_H_LEVEL_OFFSET) &
	       EPDC_FIFOCTRL_FIFO_H_LEVEL_MASK)
	    | ((100 << EPDC_FIFOCTRL_FIFO_L_LEVEL_OFFSET) &
	       EPDC_FIFOCTRL_FIFO_L_LEVEL_MASK);
	__raw_writel(reg_val, EPDC_FIFOCTRL);

	/* EPDC_TEMP - 8 for room temperature */
	/* epdc_set_temp(8); */
	/* Lab126: Allow for 25C-only waveforms */
	epdc_set_temp(mxc_epdc_fb_get_temp_index(fb_data, DEFAULT_TEMP));

	/* EPDC_RES */
	epdc_set_screen_res(epdc_mode->vmode->xres, epdc_mode->vmode->yres);

	/*
	 * EPDC_TCE_CTRL
	 * VSCAN_HOLDOFF = 4
	 * VCOM_MODE = MANUAL
	 * VCOM_VAL = 0
	 * DDR_MODE = DISABLED
	 * LVDS_MODE_CE = DISABLED
	 * LVDS_MODE = DISABLED
	 * DUAL_SCAN = DISABLED
	 * SDDO_WIDTH = 8bit
	 * PIXELS_PER_SDCLK = 4
	 */
	reg_val =
	    ((epdc_mode->vscan_holdoff << EPDC_TCE_CTRL_VSCAN_HOLDOFF_OFFSET) &
	     EPDC_TCE_CTRL_VSCAN_HOLDOFF_MASK)
	    | EPDC_TCE_CTRL_PIXELS_PER_SDCLK_4;
	__raw_writel(reg_val, EPDC_TCE_CTRL);

	/* EPDC_TCE_HSCAN */
	epdc_set_horizontal_timing(screeninfo->left_margin,
				   screeninfo->right_margin,
				   screeninfo->hsync_len,
				   screeninfo->hsync_len);

	/* EPDC_TCE_VSCAN */
	epdc_set_vertical_timing(screeninfo->upper_margin,
				 screeninfo->lower_margin,
				 screeninfo->vsync_len);

	/* EPDC_TCE_OE */
	reg_val =
	    ((epdc_mode->sdoed_width << EPDC_TCE_OE_SDOED_WIDTH_OFFSET) &
	     EPDC_TCE_OE_SDOED_WIDTH_MASK)
	    | ((epdc_mode->sdoed_delay << EPDC_TCE_OE_SDOED_DLY_OFFSET) &
	       EPDC_TCE_OE_SDOED_DLY_MASK)
	    | ((epdc_mode->sdoez_width << EPDC_TCE_OE_SDOEZ_WIDTH_OFFSET) &
	       EPDC_TCE_OE_SDOEZ_WIDTH_MASK)
	    | ((epdc_mode->sdoez_delay << EPDC_TCE_OE_SDOEZ_DLY_OFFSET) &
	       EPDC_TCE_OE_SDOEZ_DLY_MASK);
	__raw_writel(reg_val, EPDC_TCE_OE);

	/* EPDC_TCE_TIMING1 */
	__raw_writel(0x0, EPDC_TCE_TIMING1);

	/* EPDC_TCE_TIMING2 */
	reg_val =
	    ((epdc_mode->gdclk_hp_offs << EPDC_TCE_TIMING2_GDCLK_HP_OFFSET) &
	     EPDC_TCE_TIMING2_GDCLK_HP_MASK)
	    | ((epdc_mode->gdsp_offs << EPDC_TCE_TIMING2_GDSP_OFFSET_OFFSET) &
	       EPDC_TCE_TIMING2_GDSP_OFFSET_MASK);
	__raw_writel(reg_val, EPDC_TCE_TIMING2);

	/* EPDC_TCE_TIMING3 */
	reg_val =
	    ((epdc_mode->gdoe_offs << EPDC_TCE_TIMING3_GDOE_OFFSET_OFFSET) &
	     EPDC_TCE_TIMING3_GDOE_OFFSET_MASK)
	    | ((epdc_mode->gdclk_offs << EPDC_TCE_TIMING3_GDCLK_OFFSET_OFFSET) &
	       EPDC_TCE_TIMING3_GDCLK_OFFSET_MASK);
	__raw_writel(reg_val, EPDC_TCE_TIMING3);

	/*
	 * EPDC_TCE_SDCFG
	 * SDCLK_HOLD = 1
	 * SDSHR = 1
	 * NUM_CE = 1
	 * SDDO_REFORMAT = FLIP_PIXELS
	 * SDDO_INVERT = DISABLED
	 * PIXELS_PER_CE = display horizontal resolution
	 */
	num_ce = epdc_mode->num_ce;
	if (num_ce == 0)
		num_ce = 1;
	reg_val = EPDC_TCE_SDCFG_SDCLK_HOLD | EPDC_TCE_SDCFG_SDSHR
	    | ((num_ce << EPDC_TCE_SDCFG_NUM_CE_OFFSET) &
	       EPDC_TCE_SDCFG_NUM_CE_MASK)
	    | EPDC_TCE_SDCFG_SDDO_REFORMAT_FLIP_PIXELS
	    | ((epdc_mode->vmode->xres/num_ce << EPDC_TCE_SDCFG_PIXELS_PER_CE_OFFSET) &
	       EPDC_TCE_SDCFG_PIXELS_PER_CE_MASK);
	__raw_writel(reg_val, EPDC_TCE_SDCFG);

	/*
	 * EPDC_TCE_GDCFG
	 * GDRL = 1
	 * GDOE_MODE = 0;
	 * GDSP_MODE = 0;
	 */
	reg_val = EPDC_TCE_SDCFG_GDRL;
	__raw_writel(reg_val, EPDC_TCE_GDCFG);

	/*
	 * EPDC_TCE_POLARITY
	 * SDCE_POL = ACTIVE LOW
	 * SDLE_POL = ACTIVE HIGH
	 * SDOE_POL = ACTIVE HIGH
	 * GDOE_POL = ACTIVE HIGH
	 * GDSP_POL = ACTIVE LOW
	 */
	reg_val = EPDC_TCE_POLARITY_SDLE_POL_ACTIVE_HIGH
	    | EPDC_TCE_POLARITY_SDOE_POL_ACTIVE_HIGH
	    | EPDC_TCE_POLARITY_GDOE_POL_ACTIVE_HIGH;
	__raw_writel(reg_val, EPDC_TCE_POLARITY);

	/* EPDC_IRQ_MASK */
	__raw_writel(EPDC_IRQ_TCE_UNDERRUN_IRQ, EPDC_IRQ_MASK);

	/*
	 * EPDC_GPIO (LAB126)
	 * PWRCOM = 0
	 * PWRCTRL = 0
	 * BDR = 0
	 */
	reg_val = __raw_readl(EPDC_GPIO) & ~(EPDC_GPIO_PWRCOM
		 | EPDC_GPIO_PWRCTRL_MASK | EPDC_GPIO_BDR_MASK);
	__raw_writel(reg_val, EPDC_GPIO);
}

static void epdc_force_powerup(void)
{
	u32 reg_val; /* Lab126 */
	struct clk *epdc_axi_clk = clk_get(NULL, "epdc_axi");
	struct clk *epdc_pxp_clk = clk_get(NULL, "epdc_pix");

	clk_enable(epdc_axi_clk);
	clk_enable(epdc_pxp_clk);

	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_CLEAR);

	printk(KERN_WARNING "EPDC Force Powerup\n");

	reg_val = __raw_readl(EPDC_GPIO) | EPDC_GPIO_PWRCOM;
	__raw_writel(reg_val, EPDC_GPIO);
}

/* Lab126 */
static inline __u32 get_waveform_by_type(struct mxc_epdc_fb_data *fb_data, __u32 waveform)
{
	switch(waveform)
	{
		case(WAVEFORM_MODE_INIT):
			return fb_data->wv_modes.mode_init;
		case(WAVEFORM_MODE_AUTO):
			return WAVEFORM_MODE_AUTO;
		case(WAVEFORM_MODE_DU):
			return fb_data->wv_modes.mode_du;
		case(WAVEFORM_MODE_GC16):
			return fb_data->wv_modes.mode_gc16;
		case(WAVEFORM_MODE_GC16_FAST):
			return fb_data->wv_modes.mode_gc16_fast;
		case (WAVEFORM_MODE_A2):
			return fb_data->wv_modes.mode_a2;
		case(WAVEFORM_MODE_GL16):
			return fb_data->wv_modes.mode_gl16;
		case(WAVEFORM_MODE_GL16_FAST):
			return fb_data->wv_modes.mode_gl16_fast;
		case(WAVEFORM_MODE_DU4):
			return fb_data->wv_modes.mode_du4;
		case(WAVEFORM_MODE_REAGL):
			return fb_data->wv_modes.mode_reagl;
		case(WAVEFORM_MODE_REAGLD):
			return fb_data->wv_modes.mode_reagld;
		default:
			return WAVEFORM_MODE_AUTO;
	}
	return WAVEFORM_MODE_AUTO;
}

static inline int epdc_powerup_wait_for_enabled(struct mxc_epdc_fb_data *fb_data)
{
	unsigned long orig_jiffies = jiffies;
	unsigned long target_jiffies = orig_jiffies + msecs_to_jiffies(1000);
	int reg_state = 0;
	
	while(1)
	{
		reg_state = regulator_is_enabled(fb_data->display_regulator);
		if (reg_state > 0)
			break;
		if (reg_state < 0)
		{
			// Regulator is in shutdown state
			regulator_disable(fb_data->display_regulator);
			fb_data->power_state = POWER_STATE_OFF;
			return 0;
		}
		if (time_after(jiffies, target_jiffies))
		{
			dev_err(fb_data->dev, "Papyrus timeout.");
			regulator_disable(fb_data->display_regulator);
			fb_data->power_state = POWER_STATE_OFF;
			return 0;
		}
		schedule();
	}
	return 1;
}

static int epdc_powerup(struct mxc_epdc_fb_data *fb_data)
{
	int ret = 0;
	
	/*
	 * If power down request is pending, clear
	 * powering_down to cancel the request.
	 */
	if (fb_data->powering_down)
		fb_data->powering_down = false;

	if (fb_data->power_state == POWER_STATE_ON || fb_data->power_state == POWER_STATE_GOING_UP) {
		return 1;
	}
	
	fb_data->power_state = POWER_STATE_GOING_UP;

	dev_dbg(fb_data->dev, "EPDC Powerup\n");

	fb_data->updates_active = true;
	
	/* Enable pins used by EPDC */
	if (fb_data->pdata->enable_pins)
		fb_data->pdata->enable_pins();

	clk_enable(fb_data->epdc_clk_axi);
	clk_enable(fb_data->epdc_clk_pix);

	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_CLEAR);

	/* Enable power to the EPD panel */
	ret = regulator_enable(fb_data->display_regulator);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable DISPLAY regulator."
			"err = 0x%x\n", ret);
		fb_data->power_state = POWER_STATE_OFF;
		return 0;
	}
	return 1;
}

static int epdc_powerup_vcom(struct mxc_epdc_fb_data *fb_data)
{
	int ret = 0;
	u32 reg_val;

	if (fb_data->power_state != POWER_STATE_GOING_UP)
		return 0;

	ret = regulator_enable(fb_data->vcom_regulator);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable VCOM regulator."
			"err = 0x%x\n", ret);
		regulator_disable(fb_data->display_regulator);
		fb_data->power_state = POWER_STATE_OFF;
		return 0;
	}
	
	/* Lab126: Enable PWRCOM */
	reg_val = __raw_readl(EPDC_GPIO) | EPDC_GPIO_PWRCOM;
	__raw_writel(reg_val, EPDC_GPIO);
  
	fb_data->power_state = POWER_STATE_ON;
	return 1;
}

static void epdc_powerdown(struct mxc_epdc_fb_data *fb_data)
{
	u32 reg_val; /* Lab126 */

	/* If powering_down has been cleared, a powerup
	 * request is pre-empting this powerdown request.
	 * Also dont power down if someone is in the process of powering up
	 */
	if (!fb_data->powering_down
		|| (fb_data->power_state == POWER_STATE_OFF)
		|| (fb_data->power_state == POWER_STATE_GOING_UP)) {
		return;
	}

	dev_dbg(fb_data->dev, "EPDC Powerdown\n");
	
	/* Lab126: Disable PWRCOM */
	reg_val = __raw_readl(EPDC_GPIO) & ~EPDC_GPIO_PWRCOM;
	__raw_writel(reg_val, EPDC_GPIO);

	/* Disable power to the EPD panel */
	regulator_disable(fb_data->vcom_regulator);
	regulator_disable(fb_data->display_regulator);

	/* Disable clocks to EPDC */
	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_SET);
	clk_disable(fb_data->epdc_clk_pix);
	clk_disable(fb_data->epdc_clk_axi);

	/* Disable pins used by EPDC (to prevent leakage current) */
	if (fb_data->pdata->disable_pins)
		fb_data->pdata->disable_pins();

	fb_data->power_state = POWER_STATE_OFF;
	fb_data->powering_down = false;

	if (fb_data->wait_for_powerdown) {
		fb_data->wait_for_powerdown = false;
		complete(&fb_data->powerdown_compl);
	}

}

static void epdc_init_sequence(struct mxc_epdc_fb_data *fb_data)
{
	/* Initialize EPDC, passing pointer to EPDC registers */
	epdc_init_settings(fb_data);
	__raw_writel(fb_data->waveform_buffer_phys, EPDC_WVADDR);
	__raw_writel(fb_data->working_buffer_phys, EPDC_WB_ADDR);
	fb_data->in_init = true;
	mutex_lock(&fb_data->power_mutex);
	if (!epdc_powerup(fb_data) || !epdc_powerup_wait_for_enabled(fb_data))
	{
		printk(KERN_ERR "Papyrus error.\n");
		mutex_unlock(&fb_data->power_mutex);
		return;
	}
	epdc_powerup_vcom(fb_data);
	draw_mode0(fb_data);
	epdc_powerdown(fb_data);
	mutex_unlock(&fb_data->power_mutex);
	fb_data->updates_active = false;
}

static int mxc_epdc_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (offset < info->fix.smem_len) {
		/* mapping framebuffer memory */
		len = info->fix.smem_len - offset;
		vma->vm_pgoff = (info->fix.smem_start + offset) >> PAGE_SHIFT;
	} else
		return -EINVAL;

	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	vma->vm_flags |= VM_IO | VM_RESERVED;

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(info->device, "mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}

	return 0;
}

static inline u_int _chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int mxc_epdc_fb_setcolreg(u_int regno, u_int red, u_int green,
				 u_int blue, u_int transp, struct fb_info *info)
{
	unsigned int val;
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	/*if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;*/
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val = _chan_to_field(red, &info->var.red);
			val |= _chan_to_field(green, &info->var.green);
			val |= _chan_to_field(blue, &info->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}

static int mxc_epdc_fb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	int count, index, r;
	u16 *red, *green, *blue, *transp;
	u16 trans = 0xffff;
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	int i;

	dev_dbg(fb_data->dev, "setcmap\n");

	if (info->fix.visual == FB_VISUAL_STATIC_PSEUDOCOLOR) {
		/* Only support an 8-bit, 256 entry lookup */
		if (cmap->len != 256)
			return 1;

		mxc_epdc_fb_flush_updates(fb_data);

		mutex_lock(&fb_data->pxp_mutex);
		/*
		 * Store colormap in pxp_conf structure for later transmit
		 * to PxP during update process to convert gray pixels.
		 *
		 * Since red=blue=green for pseudocolor visuals, we can
		 * just use red values.
		 */
		for (i = 0; i < 256; i++)
			fb_data->pxp_conf.proc_data.lut_map[i] = cmap->red[i] & 0xFF;

		fb_data->pxp_conf.proc_data.lut_map_updated = true;

		use_cmap = 1;
		mutex_unlock(&fb_data->pxp_mutex);
	} else {
		red     = cmap->red;
		green   = cmap->green;
		blue    = cmap->blue;
		transp  = cmap->transp;
		index   = cmap->start;

		for (count = 0; count < cmap->len; count++) {
			if (transp)
				trans = *transp++;
			r = mxc_epdc_fb_setcolreg(index++, *red++, *green++, *blue++,
						trans, info);
			if (r != 0)
				return r;
		}
	}
	return 0;
}

static void adjust_coordinates(struct mxc_epdc_fb_data *fb_data,
	struct mxcfb_rect *update_region, struct mxcfb_rect *adj_update_region)
{
	struct fb_var_screeninfo *screeninfo = &fb_data->epdc_fb_var;
	u32 rotation = fb_data->epdc_fb_var.rotate;
	u32 temp;

	/* If adj_update_region == NULL, pass result back in update_region */
	/* If adj_update_region == valid, use it to pass back result */
	if (adj_update_region)
		switch (rotation) {
		case FB_ROTATE_UR:
			adj_update_region->top = update_region->top;
			adj_update_region->left = update_region->left;
			adj_update_region->width = update_region->width;
			adj_update_region->height = update_region->height;
			break;
		case FB_ROTATE_CW:
			adj_update_region->top = update_region->left;
			adj_update_region->left = screeninfo->yres -
				(update_region->top + update_region->height);
			adj_update_region->width = update_region->height;
			adj_update_region->height = update_region->width;
			break;
		case FB_ROTATE_UD:
			adj_update_region->width = update_region->width;
			adj_update_region->height = update_region->height;
			adj_update_region->top = screeninfo->yres -
				(update_region->top + update_region->height);
			adj_update_region->left = screeninfo->xres -
				(update_region->left + update_region->width);
			break;
		case FB_ROTATE_CCW:
			adj_update_region->left = update_region->top;
			adj_update_region->top = screeninfo->xres -
				(update_region->left + update_region->width);
			adj_update_region->width = update_region->height;
			adj_update_region->height = update_region->width;
			break;
		}
	else
		switch (rotation) {
		case FB_ROTATE_UR:
			/* No adjustment needed */
			break;
		case FB_ROTATE_CW:
			temp = update_region->top;
			update_region->top = update_region->left;
			update_region->left = screeninfo->yres -
				(temp + update_region->height);
			temp = update_region->width;
			update_region->width = update_region->height;
			update_region->height = temp;
			break;
		case FB_ROTATE_UD:
			update_region->top = screeninfo->yres -
				(update_region->top + update_region->height);
			update_region->left = screeninfo->xres -
				(update_region->left + update_region->width);
			break;
		case FB_ROTATE_CCW:
			temp = update_region->left;
			update_region->left = update_region->top;
			update_region->top = screeninfo->xres -
				(temp + update_region->width);
			temp = update_region->width;
			update_region->width = update_region->height;
			update_region->height = temp;
			break;
		}
}

/*
 * Set fixed framebuffer parameters based on variable settings.
 *
 * @param       info     framebuffer information pointer
 */
static int mxc_epdc_fb_set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	if (var->grayscale)
		fix->visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	else
		fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ypanstep = 1;

	return 0;
}

static struct fb_deferred_io mxc_epdc_fb_defio;

/*
 * This routine actually sets the video mode. It's in here where we
 * the hardware state info->par and fix which can be affected by the
 * change in par. For this driver it doesn't do much.
 *
 */
static int mxc_epdc_fb_set_par(struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	struct pxp_config_data *pxp_conf = &fb_data->pxp_conf;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;
	struct fb_var_screeninfo *screeninfo = &fb_data->info.var;
	struct mxc_epdc_fb_mode *epdc_modes = fb_data->pdata->epdc_mode;
	int i;
	int ret;

	mxc_epdc_fb_flush_updates(fb_data);
	
	mutex_lock(&fb_data->queue_mutex);
	fb_data->epdc_fb_var = *screeninfo;	
	mutex_unlock(&fb_data->queue_mutex);


	mutex_lock(&fb_data->pxp_mutex);

	/*
	 * Update PxP config data (used to process FB regions for updates)
	 * based on FB info and processing tasks required
	 */

	/* Initialize non-channel-specific PxP parameters */
	proc_data->drect.left = proc_data->srect.left = 0;
	proc_data->drect.top = proc_data->srect.top = 0;
	proc_data->drect.width = proc_data->srect.width = screeninfo->xres;
	proc_data->drect.height = proc_data->srect.height = screeninfo->yres;
	proc_data->scaling = 0;
	proc_data->hflip = 0;
	proc_data->vflip = 0;
	proc_data->rotate = screeninfo->rotate;
	proc_data->bgcolor = 0;
	proc_data->overlay_state = 0;
	proc_data->lut_transform = PXP_LUT_NONE;

	/*
	 * configure S0 channel parameters
	 * Parameters should match FB format/width/height
	 */
	if (screeninfo->grayscale)
		pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_GREY;
	else {
		switch (screeninfo->bits_per_pixel) {
		case 16:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
			break;
		case 24:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB24;
			break;
		case 32:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB32;
			break;
		default:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
			break;
		}
	}
	pxp_conf->s0_param.width = screeninfo->xres_virtual;
	pxp_conf->s0_param.height = screeninfo->yres;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;

	/*
	 * Initialize Output channel parameters
	 * Output is Y-only greyscale
	 * Output width/height will vary based on update region size
	 */
	pxp_conf->out_param.width = screeninfo->xres;
	pxp_conf->out_param.height = screeninfo->yres;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_GREY;

	mutex_unlock(&fb_data->pxp_mutex);

	/*
	 * If HW not yet initialized, check to see if we are being sent
	 * an initialization request.
	 */
	if (!fb_data->hw_ready) {
		struct fb_videomode mode;

		fb_var_to_videomode(&mode, screeninfo);

		/* Match videomode against epdc modes */
		for (i = 0; i < fb_data->pdata->num_modes; i++) {
			if (!fb_mode_is_equal(epdc_modes[i].vmode, &mode))
				continue;
			fb_data->cur_mode = &epdc_modes[i];
			break;
		}

		/* Found a match - Grab timing params */
		screeninfo->left_margin = mode.left_margin;
		screeninfo->right_margin = mode.right_margin;
		screeninfo->upper_margin = mode.upper_margin;
		screeninfo->lower_margin = mode.lower_margin;
		screeninfo->hsync_len = mode.hsync_len;
		screeninfo->vsync_len = mode.vsync_len;

		/* Initialize EPDC settings and init panel */
		ret =
		    mxc_epdc_fb_init_hw((struct fb_info *)fb_data);
		if (ret) {
			dev_err(fb_data->dev,
				"Failed to load panel waveform data\n");
			return ret;
		}
	}

	/*
	 * EOF sync delay (in us) should be equal to the vscan holdoff time
	 * VSCAN_HOLDOFF time = (VSCAN_HOLDOFF value + 1) * Vertical lines
	 * Add 25us for additional margin
	 */
	fb_data->eof_sync_period = (fb_data->cur_mode->vscan_holdoff + 1) *
		1000000/(fb_data->cur_mode->vmode->refresh *
		(fb_data->cur_mode->vmode->upper_margin +
		fb_data->cur_mode->vmode->yres +
		fb_data->cur_mode->vmode->lower_margin +
		fb_data->cur_mode->vmode->vsync_len)) + 25;

	mxc_epdc_fb_set_fix(info);

	return 0;
}

static int mxc_epdc_fb_check_var(struct fb_var_screeninfo *var,
				 struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
	    (var->bits_per_pixel != 16) && (var->bits_per_pixel != 8))
		var->bits_per_pixel = default_bpp;

	switch (var->bits_per_pixel) {
	case 8:
		if (var->grayscale != 0) {
			/*
			 * For 8-bit grayscale, R, G, and B offset are equal.
			 *
			 */
			var->red.length = 8;
			var->red.offset = 0;
			var->red.msb_right = 0;

			var->green.length = 8;
			var->green.offset = 0;
			var->green.msb_right = 0;

			var->blue.length = 8;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
		} else {
			var->red.length = 3;
			var->red.offset = 5;
			var->red.msb_right = 0;

			var->green.length = 3;
			var->green.offset = 2;
			var->green.msb_right = 0;

			var->blue.length = 2;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
		}
		break;
	case 16:
		var->red.length = 5;
		var->red.offset = 11;
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 5;
		var->green.msb_right = 0;

		var->blue.length = 5;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 24:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 32:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 24;
		var->transp.msb_right = 0;
		break;
	}

	switch (var->rotate) {
	case FB_ROTATE_UR:
	case FB_ROTATE_UD:
		var->xres = fb_data->native_width;
		var->yres = fb_data->native_height;
		break;
	case FB_ROTATE_CW:
	case FB_ROTATE_CCW:
		var->xres = fb_data->native_height;
		var->yres = fb_data->native_width;
		break;
	default:
		/* Invalid rotation value */
		var->rotate = 0;
		dev_dbg(fb_data->dev, "Invalid rotation request\n");
		return -EINVAL;
	}

	var->xres_virtual = ALIGN(var->xres, 32);
	var->yres_virtual = ALIGN(var->yres, 128) * fb_data->num_screens;

	var->height = -1;
	var->width = -1;

	/*
	 * Can't change the FB parameters until current updates have completed.
	 * This function returns when all active updates are done.
	 */
	mxc_epdc_fb_flush_updates(fb_data);

	return 0;
}

void mxc_epdc_fb_set_waveform_modes(struct mxcfb_waveform_modes *modes,
	struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;


	dev_dbg(fb_data->dev, "MXCFB : Setting waveform modes.\n\
			\tBefore :\n\
			\t\tDU : %d\n\
			\t\tGC : %d\n\
			\t\tGL : %d\n\
			\t\tGCF: %d\n\
			\t\tGLF: %d\n\
			\t\tA2 : %d\n\
			\t\tDU4: %d\n\
			\t\tREAGL: %d\n\
			\t\tREAGLD: %d\n",
			fb_data->wv_modes.mode_du,
			fb_data->wv_modes.mode_gc16,
			fb_data->wv_modes.mode_gl16,
			fb_data->wv_modes.mode_gc16_fast,
			fb_data->wv_modes.mode_gl16_fast,
			fb_data->wv_modes.mode_a2,
			fb_data->wv_modes.mode_du4,
			fb_data->wv_modes.mode_reagl,
			fb_data->wv_modes.mode_reagld
			);
			
	memcpy(&fb_data->wv_modes, modes,
		sizeof(struct mxcfb_waveform_modes)); /* Lab126 */
	
	dev_dbg(fb_data->dev, "MXCFB : Setting waveform modes.\n\
			\tAfter :\n\
			\t\tDU : %d\n\
			\t\tGC : %d\n\
			\t\tGL : %d\n\
			\t\tGCF: %d\n\
			\t\tGLF: %d\n\
			\t\tA2 : %d\n\
			\t\tDU4: %d\n\
			\t\tREAGL: %d\n\
			\t\tREAGLD: %d\n",
			fb_data->wv_modes.mode_du,
			fb_data->wv_modes.mode_gc16,
			fb_data->wv_modes.mode_gl16,
			fb_data->wv_modes.mode_gc16_fast,
			fb_data->wv_modes.mode_gl16_fast,
			fb_data->wv_modes.mode_a2,
			fb_data->wv_modes.mode_du4,
			fb_data->wv_modes.mode_reagl,
			fb_data->wv_modes.mode_reagld
			);
}
EXPORT_SYMBOL(mxc_epdc_fb_set_waveform_modes);

static int mxc_epdc_fb_get_temp_index(struct mxc_epdc_fb_data *fb_data, int temp)
{
	int i;
	int index = -1;

	if (fb_data->trt_entries == 0) {
		dev_err(fb_data->dev,
			"No TRT exists...using default temp index\n");
		return DEFAULT_TEMP_INDEX;
	}

	/* Search temperature ranges for a match */
	for (i = 0; i < fb_data->trt_entries - 1; i++) {
		if ((temp >= fb_data->temp_range_bounds[i])) {
			index = i;
			if ((temp < fb_data->temp_range_bounds[i+1])) {
				break;
      }
		}
	}

	if (index < 0) {
		dev_dbg(fb_data->dev,
			"No TRT index match...using default temp index\n");
		return DEFAULT_TEMP_INDEX;
	}

	dev_dbg(fb_data->dev, "Using temperature index %d\n", index);

	return index;
}

int mxc_epdc_fb_set_temperature(int temperature, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info : g_fb_data;
	/* int temp_index; Lab126 */

	/* Store temp index. Used later when configuring updates. */
	mutex_lock(&fb_data->queue_mutex);
	fb_data->temp_index = mxc_epdc_fb_get_temp_index(fb_data, temperature);
	mutex_unlock(&fb_data->queue_mutex);

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_temperature);

int mxc_epdc_fb_set_auto_update(u32 auto_mode, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	dev_dbg(fb_data->dev, "Setting auto update mode to %d\n", auto_mode);

	switch (auto_mode) {
	case AUTO_UPDATE_MODE_REGION_MODE:
	case AUTO_UPDATE_MODE_AUTOMATIC_MODE_FULL:
	case AUTO_UPDATE_MODE_AUTOMATIC_MODE_PART:
		fb_data->auto_mode = auto_mode;
		break;
		
	default:
		dev_err(fb_data->dev, "Invalid auto update mode parameter.\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_auto_update);

int mxc_epdc_fb_set_upd_scheme(u32 upd_scheme, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	dev_dbg(fb_data->dev, "Setting optimization level to %d\n", upd_scheme);

	/*
	 * Can't change the scheme while until current updates have completed.
	 * This function returns when all active updates are done.
	 */
	mxc_epdc_fb_flush_updates(fb_data);

	if ((upd_scheme == UPDATE_SCHEME_SNAPSHOT)
		|| (upd_scheme == UPDATE_SCHEME_QUEUE)
		|| (upd_scheme == UPDATE_SCHEME_QUEUE_AND_MERGE))
		fb_data->upd_scheme = upd_scheme;
	else {
		dev_err(fb_data->dev, "Invalid update scheme specified.\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_upd_scheme);

static void copy_before_process(struct mxc_epdc_fb_data *fb_data,
	struct update_data_list *upd_data_list)
{
	struct mxcfb_update_data *upd_data =
		&upd_data_list->update_desc->upd_data;
	int i;
	unsigned char *temp_buf_ptr = fb_data->virt_addr_copybuf;
	unsigned char *src_ptr;
	struct mxcfb_rect *src_upd_region;
	int temp_buf_stride;
	int src_stride;
	int bpp = fb_data->info.var.bits_per_pixel;
	int left_offs, right_offs;
	int x_trailing_bytes, y_trailing_bytes;
	int alt_buf_offset;

	/* Set source buf pointer based on input source, panning, etc. */
	if (upd_data->flags & EPDC_FLAG_USE_ALT_BUFFER) {
		src_upd_region = &upd_data->alt_buffer_data.alt_update_region;
		src_stride =
			upd_data->alt_buffer_data.width * bpp/8;
		alt_buf_offset = upd_data->alt_buffer_data.phys_addr -
			fb_data->info.fix.smem_start;
		src_ptr = fb_data->info.screen_base + alt_buf_offset
			+ src_upd_region->top * src_stride;
	} else {
		src_upd_region = &upd_data->update_region;
		src_stride = fb_data->info.var.xres_virtual * bpp/8;
		src_ptr = fb_data->info.screen_base + fb_data->fb_offset
			+ src_upd_region->top * src_stride;
	}

	temp_buf_stride = ALIGN(src_upd_region->width, 8) * bpp/8;
	left_offs = src_upd_region->left * bpp/8;
	right_offs = src_upd_region->width * bpp/8;
	x_trailing_bytes = (ALIGN(src_upd_region->width, 8)
		- src_upd_region->width) * bpp/8;

	for (i = 0; i < src_upd_region->height; i++) {
		/* Copy the full line */
		memcpy(temp_buf_ptr, src_ptr + left_offs,
			src_upd_region->width * bpp/8);

		/* Clear any unwanted pixels at the end of each line */
		if (src_upd_region->width & 0x7) {
			memset(temp_buf_ptr + right_offs, 0x0,
				x_trailing_bytes);
		}

		temp_buf_ptr += temp_buf_stride;
		src_ptr += src_stride;
	}

	/* Clear any unwanted pixels at the bottom of the end of each line */
	if (src_upd_region->height & 0x7) {
		y_trailing_bytes = (ALIGN(src_upd_region->height, 8)
			- src_upd_region->height) *
			ALIGN(src_upd_region->width, 8) * bpp/8;

		memset(temp_buf_ptr, 0x0, y_trailing_bytes);
	}
}

static inline int epdc_hist_analysis_is_gray(struct mxcfb_rect *rect,
		struct mxc_epdc_fb_data *fb_data)
{
	struct fb_info *info = &fb_data->info;
	u8 *fb = info->screen_base + fb_data->fb_offset;
	int i, y;
	u32 bpp     = info->var.bits_per_pixel;

	u32 xres_fb   = info->var.xres_virtual;
	u32 xres_buffer   = rect->width;

	u32 rowbytes_fb   = xres_fb * (bpp / 8);
	u32 rowbytes_buffer = xres_buffer * (bpp / 8);

	u32 ystart    = rect->top;
	u32 yend      = ystart + rect->height;
	u32 xstart    = rect->left  * (bpp / 8);

#if __ARM_NEON__
	uint32_t cmp[2];
	uint8x8_t one_vec = vdup_n_u8(1);
	
	for (y = ystart; y < yend; y++)
	{
		int x = 0;
		if (rowbytes_buffer > 8)
		{
			for (x = 0; (x + 8) <= rowbytes_buffer; x+=8)
			{
				uint8x8_t in_vec = vld1_u8(&fb[(rowbytes_fb * y) + xstart + x]);
				uint8x8_t end_vec = vshr_n_u8(vadd_u8(in_vec, one_vec),1); // Shift right by 1

				vst1_u32(cmp, end_vec);
				if (cmp[0] || cmp[1])
					return 1;
			}
		}

		for (; x < rowbytes_buffer; x++)
		{
			u8 val = fb[(rowbytes_fb * y) + xstart + x];
			if (val != 0 && val != 0xff)
				return 1;
		}
	}
	return 0;
#else
	for (i = 0, y = ystart; i < yend; i++, y++)
	{
		int x;
		for (x = 0; x < rowbytes_buffer; x++)
		{
			u8 val = fb[(rowbytes_fb * y) + xstart + x];
			if (val != 0 && val != 0xff)
				return 1;
		}
	}
	return 0;
#endif
}

static int epdc_process_update(struct update_data_list *upd_data_list,
				   struct mxc_epdc_fb_data *fb_data)
{
	struct mxcfb_rect *src_upd_region; /* Region of src buffer for update */
	struct mxcfb_rect pxp_upd_region;
	u32 src_width, src_height;
	u32 offset_from_4, bytes_per_pixel;
	u32 post_rotation_xcoord, post_rotation_ycoord, width_pxp_blocks;
	u32 pxp_input_offs, pxp_output_offs, pxp_output_shift;
	u32 hist_stat = 0;
	int width_unaligned, height_unaligned;
	bool input_unaligned = false;
	bool line_overflow = false;
	int pix_per_line_added;
	bool use_temp_buf = false;
	struct mxcfb_rect temp_buf_upd_region;
	struct update_desc_list *upd_desc_list = upd_data_list->update_desc;

	int ret;

	/*
	 * Gotta do a whole bunch of buffer ptr manipulation to
	 * work around HW restrictions for PxP & EPDC
	 */

	/* If needed start powering up the display */
	mutex_lock(&fb_data->power_mutex);
	if ((fb_data->power_state == POWER_STATE_OFF)
		|| fb_data->powering_down) {
		if (!epdc_powerup(fb_data))
		{
			mutex_unlock(&fb_data->power_mutex);
			return 1;
		}
	}
	
	/*
	 * Are we using FB or an alternate (overlay)
	 * buffer for source of update?
	 */
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER) {
		src_width = upd_desc_list->upd_data.alt_buffer_data.width;
		src_height = upd_desc_list->upd_data.alt_buffer_data.height;	
		src_upd_region = &upd_desc_list->upd_data.alt_buffer_data.alt_update_region;
	} else {
		src_width = fb_data->epdc_fb_var.xres_virtual;
		src_height = fb_data->epdc_fb_var.yres;
		src_upd_region = &upd_desc_list->upd_data.update_region;
	}

	bytes_per_pixel = fb_data->info.var.bits_per_pixel >> 3;

	/*
	 * SW workaround for PxP limitation
	 *
	 * There are 3 cases where we cannot process the update data
	 * directly from the input buffer:
	 *
	 * 1) PxP must process 8x8 pixel blocks, and all pixels in each block
	 * are considered for auto-waveform mode selection. If the
	 * update region is not 8x8 aligned, additional unwanted pixels
	 * will be considered in auto-waveform mode selection.
	 *
	 * 2) PxP input must be 32-bit aligned, so any update
	 * address not 32-bit aligned must be shifted to meet the
	 * 32-bit alignment.  The PxP will thus end up processing pixels
	 * outside of the update region to satisfy this alignment restriction,
	 * which can affect auto-waveform mode selection.
	 *
	 * 3) If input fails 32-bit alignment, and the resulting expansion
	 * of the processed region would add at least 8 pixels more per
	 * line than the original update line width, the EPDC would
	 * cause screen artifacts by incorrectly handling the 8+ pixels
	 * at the end of each line.
	 *
	 * Workaround is to copy from source buffer into a temporary
	 * buffer, which we pad with zeros to match the 8x8 alignment
	 * requirement. This temp buffer becomes the input to the PxP.
	 */
	width_unaligned = src_upd_region->width & 0x7;
	height_unaligned = src_upd_region->height & 0x7;

	offset_from_4 = src_upd_region->left & 0x3;
	input_unaligned = ((offset_from_4 * bytes_per_pixel % 4) != 0) ?
		true : false;

	pix_per_line_added = (offset_from_4 * bytes_per_pixel % 4) / bytes_per_pixel;
	if ((((fb_data->info.var.rotate == FB_ROTATE_UR) ||
					fb_data->info.var.rotate == FB_ROTATE_UD)) &&
			(ALIGN(src_upd_region->width, 8) <
			 ALIGN(src_upd_region->width + pix_per_line_added, 8)))
		line_overflow = true;

	/* Grab pxp_mutex here so that we protect access
	 * to copybuf in addition to the PxP structures */
	mutex_lock(&fb_data->pxp_mutex);


	if (!(src_upd_region->width == fb_data->info.var.xres &&  src_upd_region->height == fb_data->info.var.yres))
	{
		if (((width_unaligned || height_unaligned || input_unaligned) &&
					(upd_desc_list->upd_data.waveform_mode == WAVEFORM_MODE_AUTO))
				|| line_overflow) {
			if (!line_overflow)
			{
				int is_gray = epdc_hist_analysis_is_gray(src_upd_region, fb_data);
				if (is_gray)
					upd_desc_list->upd_data.waveform_mode = upd_desc_list->upd_data.hist_gray_waveform_mode;
				else
					upd_desc_list->upd_data.waveform_mode = upd_desc_list->upd_data.hist_bw_waveform_mode;
			}
			else
			{
				dev_dbg(fb_data->dev, "Copying update before processing.\n");

				/* Update to reflect what the new source buffer will be */
				src_width = ALIGN(src_upd_region->width, 8);
				src_height = ALIGN(src_upd_region->height, 8);

				copy_before_process(fb_data, upd_data_list);

				/*
				 * src_upd_region should now describe
				 * the new update buffer attributes.
				 */
				temp_buf_upd_region.left = 0;
				temp_buf_upd_region.top = 0;
				temp_buf_upd_region.width = src_upd_region->width;
				temp_buf_upd_region.height = src_upd_region->height;
				src_upd_region = &temp_buf_upd_region;

				use_temp_buf = true;
			}
		}
	}

	/*
	 * Compute buffer offset to account for
	 * PxP limitation (input must be 32-bit aligned)
	 */
	offset_from_4 = src_upd_region->left & 0x3;
	input_unaligned = ((offset_from_4 * bytes_per_pixel % 4) != 0) ?
				true : false;
	if (input_unaligned) {
		/* Leave a gap between PxP input addr and update region pixels */
		pxp_input_offs =
			(src_upd_region->top * src_width + src_upd_region->left)
			* bytes_per_pixel & 0xFFFFFFFC;
		/* Update region should change to reflect relative position to input ptr */
		pxp_upd_region.top = 0;
		pxp_upd_region.left = (offset_from_4 * bytes_per_pixel % 4) / bytes_per_pixel;
	} else {
		pxp_input_offs =
			(src_upd_region->top * src_width + src_upd_region->left)
			* bytes_per_pixel;
		/* Update region should change to reflect relative position to input ptr */
		pxp_upd_region.top = 0;
		pxp_upd_region.left = 0;
	}

	/* Update region dimensions to meet 8x8 pixel requirement */
	pxp_upd_region.width =
		ALIGN(src_upd_region->width + pxp_upd_region.left, 8);
	pxp_upd_region.height = ALIGN(src_upd_region->height, 8);

	switch (fb_data->epdc_fb_var.rotate) {
	case FB_ROTATE_UR:
	default:
		post_rotation_xcoord = pxp_upd_region.left;
		post_rotation_ycoord = pxp_upd_region.top;
		width_pxp_blocks = pxp_upd_region.width;
		break;
	case FB_ROTATE_CW:
		width_pxp_blocks = pxp_upd_region.height;
		post_rotation_xcoord = width_pxp_blocks - src_upd_region->height;
		post_rotation_ycoord = pxp_upd_region.left;
		break;
	case FB_ROTATE_UD:
		width_pxp_blocks = pxp_upd_region.width;
		post_rotation_xcoord = width_pxp_blocks - src_upd_region->width - pxp_upd_region.left;
		post_rotation_ycoord = pxp_upd_region.height - src_upd_region->height - pxp_upd_region.top;
		break;
	case FB_ROTATE_CCW:
		width_pxp_blocks = pxp_upd_region.height;
		post_rotation_xcoord = pxp_upd_region.top;
		post_rotation_ycoord = pxp_upd_region.width - src_upd_region->width - pxp_upd_region.left;
		break;
	}

	/* Update region start coord to force PxP to process full 8x8 regions */
	pxp_upd_region.top &= ~0x7;
	pxp_upd_region.left &= ~0x7;

	pxp_output_shift = ALIGN(post_rotation_xcoord, 8)
		- post_rotation_xcoord;

	pxp_output_offs = post_rotation_ycoord * width_pxp_blocks
		+ pxp_output_shift;

	upd_desc_list->epdc_offs = ALIGN(pxp_output_offs, 8);

	/* Source address either comes from alternate buffer
	   provided in update data, or from the framebuffer. */
	if (use_temp_buf)
		sg_dma_address(&fb_data->sg[0]) =
			fb_data->phys_addr_copybuf;
	else if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER)
		sg_dma_address(&fb_data->sg[0]) =
			upd_desc_list->upd_data.alt_buffer_data.phys_addr
				+ pxp_input_offs;
	else {
		sg_dma_address(&fb_data->sg[0]) =
			fb_data->info.fix.smem_start + fb_data->fb_offset
			+ pxp_input_offs;
		sg_set_page(&fb_data->sg[0],
			virt_to_page(fb_data->info.screen_base),
			fb_data->info.fix.smem_len,
			offset_in_page(fb_data->info.screen_base));
	}

	/* Update sg[1] to point to output of PxP proc task */
	sg_dma_address(&fb_data->sg[1]) = upd_data_list->phys_addr
						+ pxp_output_shift;
	sg_set_page(&fb_data->sg[1], virt_to_page(upd_data_list->virt_addr),
			fb_data->max_pix_size,
			offset_in_page(upd_data_list->virt_addr));

	/*
	 * Set PxP LUT transform type based on update flags.
	 */
	fb_data->pxp_conf.proc_data.lut_transform = 0;
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_ENABLE_INVERSION)
		fb_data->pxp_conf.proc_data.lut_transform |= PXP_LUT_INVERT;
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_FORCE_MONOCHROME)
		fb_data->pxp_conf.proc_data.lut_transform |=
			PXP_LUT_BLACK_WHITE;
	if ((upd_desc_list->upd_data.flags & EPDC_FLAG_USE_CMAP) || use_cmap) {
		fb_data->pxp_conf.proc_data.lut_transform |=
			PXP_LUT_USE_CMAP;
	}

	/*
	 * Toggle inversion processing if 8-bit
	 * inverted is the current pixel format.
	 */
	if (fb_data->epdc_fb_var.grayscale == GRAYSCALE_8BIT_INVERTED)
		fb_data->pxp_conf.proc_data.lut_transform ^= PXP_LUT_INVERT;

	/* This is a blocking call, so upon return PxP tx should be done */	
	ret = pxp_process_update(fb_data, src_width, src_height,
		&pxp_upd_region);
	if (ret) {
		dev_err(fb_data->dev, "Unable to submit PxP update task.\n");
		mutex_unlock(&fb_data->power_mutex);
		mutex_unlock(&fb_data->pxp_mutex);
		return ret;
	}

	/* This is a blocking call, so upon return PxP tx should be done */
	ret = pxp_complete_update(fb_data, &hist_stat);
	if (ret) {
		dev_err(fb_data->dev, "Unable to complete PxP update task: %d\n", ret);
		mutex_unlock(&fb_data->power_mutex);
		mutex_unlock(&fb_data->pxp_mutex);
		return ret;
	}
	mutex_unlock(&fb_data->pxp_mutex);
	
	/* Update waveform mode from PxP histogram results */
	if (upd_desc_list->upd_data.waveform_mode == WAVEFORM_MODE_AUTO)
	{
		if (hist_stat & 0x1)
			upd_desc_list->upd_data.waveform_mode = upd_desc_list->upd_data.hist_bw_waveform_mode;
		else if (hist_stat & 0x2)
			upd_desc_list->upd_data.waveform_mode = upd_desc_list->upd_data.hist_gray_waveform_mode;
		else if (hist_stat & 0x4)
			upd_desc_list->upd_data.waveform_mode = upd_desc_list->upd_data.hist_gray_waveform_mode;
		else if (hist_stat & 0x8)
			upd_desc_list->upd_data.waveform_mode = upd_desc_list->upd_data.hist_gray_waveform_mode;
		else
			upd_desc_list->upd_data.waveform_mode = upd_desc_list->upd_data.hist_gray_waveform_mode;
	
		dev_dbg(fb_data->dev, "hist_stat = 0x%x, new waveform = 0x%x\n",
				hist_stat, upd_desc_list->upd_data.waveform_mode);
	}
		
	if (mxc_epdc_debugging) {
		printk(KERN_INFO "mxc_epdc_fb: [%d] Waveform used is 0x%x\n", upd_desc_list->upd_data.update_marker, upd_desc_list->upd_data.waveform_mode);
	}
	
	if (fb_data->power_state == POWER_STATE_GOING_UP)
	{
		if (!epdc_powerup_wait_for_enabled(fb_data))
		{
			mutex_unlock(&fb_data->power_mutex);
			return 1;
		}
		epdc_powerup_vcom(fb_data);
	}
	mutex_unlock(&fb_data->power_mutex);

	return 0;
}

static void merge_hist_waveforms(struct mxcfb_update_data *a, struct mxcfb_update_data *b)
{
	// Do the Black and White waveforms first
	__u32 *awf = &a->hist_bw_waveform_mode;
	__u32 *bwf = &b->hist_bw_waveform_mode;

	if (*awf != *bwf)
	{
		if (!(*awf) || !(*bwf))
		{
			// If one is not set give priority to the one set
			*awf = (!(*awf))?(*bwf):(*awf);
		}
		else
		{
			// If there are not the same, one is DU.
			*awf = g_fb_data->wv_modes.mode_du;
		}
	}

	// Now do the gray waveforms
	awf = &a->hist_gray_waveform_mode;
	bwf = &b->hist_gray_waveform_mode;

	if (*awf != *bwf)
	{
		if (!(*awf) || !(*bwf))
		{
			// If one is not set give priority to the one set
			*awf = (!*(awf))?(*bwf):(*awf);
		}
		else
		{
			if (*awf == g_fb_data->wv_modes.mode_gc16 || 
					*bwf == g_fb_data->wv_modes.mode_gc16)
			{
				// Priority to GC16
				*awf = g_fb_data->wv_modes.mode_gc16;
			}
			else if (*awf == g_fb_data->wv_modes.mode_gl16 ||
							 *bwf == g_fb_data->wv_modes.mode_gl16)
			{
				// Next priority to GL16
				*awf = g_fb_data->wv_modes.mode_gl16;
			}
			else
			{
				// All other gray wvs are equal. Just use the same one
			}
		}
	}
}

static int epdc_submit_merge(struct update_desc_list *upd_desc_list,
				struct update_desc_list *update_to_merge)
{
	struct mxcfb_update_data *a, *b;
	struct mxcfb_rect *arect, *brect;
	struct mxcfb_rect combine;

	a = &upd_desc_list->upd_data;
	b = &update_to_merge->upd_data;
	arect = &upd_desc_list->upd_data.update_region;
	brect = &update_to_merge->upd_data.update_region;

	/*
	 * Updates with different flags must be executed sequentially.
	 * Halt the merge process to ensure this.
	 */
	if (a->flags != b->flags)
		return MERGE_BLOCK;

	if (arect->left > (brect->left + brect->width) || 
			brect->left > (arect->left + arect->width) ||
			arect->top > (brect->top + brect->height) ||
			brect->top > (arect->top + arect->height))
	{
		return MERGE_FAIL;
	}
	
	if (mxc_epdc_debugging)
	{
		printk(KERN_INFO "mxc_epdc_fb: Going to merge update %d and %d. Waveforms: %d and %d\n", 
				a->update_marker, b->update_marker, a->waveform_mode, b->waveform_mode);
	}

	/* Lab126: Merge code giving priority to highest fidelity waveforms */
	if (a->waveform_mode == b->waveform_mode)
	{
		// Merge prefered histogram waveforms in case of AUTO
	}
	else if (a->waveform_mode == g_fb_data->wv_modes.mode_gc16 ||
			b->waveform_mode == g_fb_data->wv_modes.mode_gc16)
	{
		// Priority to GC16 updates
		a->waveform_mode = g_fb_data->wv_modes.mode_gc16;
	}
	else if (a->waveform_mode == g_fb_data->wv_modes.mode_gl16 ||
			b->waveform_mode == g_fb_data->wv_modes.mode_gl16)
	{
		// Next priority to GL16 updates
		a->waveform_mode = g_fb_data->wv_modes.mode_gl16;
	}
	else if (a->waveform_mode == g_fb_data->wv_modes.mode_gc16_fast ||
			b->waveform_mode == g_fb_data->wv_modes.mode_gc16_fast)
	{
		// Next priority GC16 fast
		a->waveform_mode = g_fb_data->wv_modes.mode_gc16_fast;
	}
	else if (a->waveform_mode == g_fb_data->wv_modes.mode_gl16_fast ||
			b->waveform_mode == g_fb_data->wv_modes.mode_gl16_fast)
	{
		// Next priority GL16 fast
		a->waveform_mode = g_fb_data->wv_modes.mode_gl16_fast;
	}
	else if (a->waveform_mode == g_fb_data->wv_modes.mode_du4 ||
			b->waveform_mode == g_fb_data->wv_modes.mode_du4)
	{
		// Next priority DU4
		a->waveform_mode = g_fb_data->wv_modes.mode_du4;
	}
	else if (a->waveform_mode == g_fb_data->wv_modes.mode_du ||
			b->waveform_mode == g_fb_data->wv_modes.mode_du)
	{
		// Next priority DU
		a->waveform_mode = g_fb_data->wv_modes.mode_du;
	}
	else if (a->waveform_mode == g_fb_data->wv_modes.mode_a2 ||
			b->waveform_mode == g_fb_data->wv_modes.mode_a2)
	{
		// Next priority A2
		a->waveform_mode = g_fb_data->wv_modes.mode_a2;
	}
	else if (a->waveform_mode == WAVEFORM_MODE_AUTO)
	{
		a->waveform_mode = WAVEFORM_MODE_AUTO;
	}
	else
	{
		a->waveform_mode = b->waveform_mode;
	}

	if (mxc_epdc_debugging)
	{
		printk(KERN_INFO "mxc_epdc_fb: Merging update %d and %d. New waveform: %d\n", 
				a->update_marker, b->update_marker, a->waveform_mode);
	}
	// Lab126: We want to merge prefered hist waveforms in case of collision
	merge_hist_waveforms(a, b);
	
	if (a->update_mode == UPDATE_MODE_FULL ||
			b->update_mode == UPDATE_MODE_FULL)
	{
		// If any of the two updates are FULL, make it a FULL update
		a->update_mode = UPDATE_MODE_FULL;
	}
  
	combine.left = arect->left < brect->left ? arect->left : brect->left;
	combine.top = arect->top < brect->top ? arect->top : brect->top;
	combine.width = (arect->left + arect->width) >
			(brect->left + brect->width) ?
			(arect->left + arect->width - combine.left) :
			(brect->left + brect->width - combine.left);
	combine.height = (arect->top + arect->height) >
			(brect->top + brect->height) ?
			(arect->top + arect->height - combine.top) :
			(brect->top + brect->height - combine.top);

	*arect = combine;

	/* Merge markers */
	list_splice_tail(&update_to_merge->upd_marker_list,
		&upd_desc_list->upd_marker_list);

	/* Merged update should take on the earliest order */
	upd_desc_list->update_order =
		(upd_desc_list->update_order > update_to_merge->update_order) ?
		upd_desc_list->update_order : update_to_merge->update_order;

	return MERGE_OK;
}

static void epdc_submit_work_func(struct work_struct *work)
{
	int temp_index;
	struct update_data_list *next_update, *temp_update;
	struct update_desc_list *next_desc, *temp_desc;
	struct update_marker_data *next_marker, *temp_marker;
	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data, epdc_submit_work);
	struct update_data_list *upd_data_list = NULL;
	struct mxcfb_rect adj_update_region;
	bool end_merge = false;
	int ret;

	/* Protect access to buffer queues and to update HW */
	mutex_lock(&fb_data->queue_mutex);

	/*
	 * Are any of our collision updates able to go now?
	 * Go through all updates in the collision list and check to see
	 * if the collision mask has been fully cleared
	 */
	list_for_each_entry_safe(next_update, temp_update,
				&fb_data->upd_buf_collision_list, list) {

		if (next_update->collision_mask != 0)
			continue;

		dev_dbg(fb_data->dev, "A collision update is ready to go!\n");

		/* Force waveform mode to auto for resubmitted collisions */
		/* Lab126: Prefered waveforms remain unchanged */
		next_update->update_desc->upd_data.waveform_mode = WAVEFORM_MODE_AUTO;
		
		/*
		 * We have a collision cleared, so select it for resubmission.
		 * If an update is already selected, attempt to merge.
		 */
		if (!upd_data_list) {
			upd_data_list = next_update;
			list_del_init(&next_update->list);
			if (fb_data->upd_scheme ==
				UPDATE_SCHEME_QUEUE)
				/* If not merging, we have our update */
				break;
		} else {
			switch (epdc_submit_merge(upd_data_list->update_desc,
						next_update->update_desc)) {
			case MERGE_OK:
			       dev_dbg(fb_data->dev,
			               "Update merged [collision]\n");
				list_del_init(&next_update->update_desc->list);
				kfree(next_update->update_desc);
				next_update->update_desc = NULL;
			       list_del_init(&next_update->list);
			       /* Add to free buffer list */
			       list_add_tail(&next_update->list,
						&fb_data->upd_buf_free_list);
			       break;
	               case MERGE_FAIL:
			       dev_dbg(fb_data->dev,
			               "Update not merged [collision]\n");
			       break;
	               case MERGE_BLOCK:
			       dev_dbg(fb_data->dev,
			               "Merge blocked [collision]\n");
			       end_merge = true;
			       break;
	               }

	               if (end_merge) {
			       end_merge = false;
			       break;
	               }
	       }
	}

	/*
	 * Skip pending update list only if we found a collision
	 * update and we are not merging
	 */
	if (!((fb_data->upd_scheme == UPDATE_SCHEME_QUEUE) &&
		upd_data_list)) {
		/*
		 * If we didn't find a collision update ready to go,
		 * we try to grab one from the update queue
		 */
		 if (!upd_data_list &&
			list_empty(&fb_data->upd_buf_free_list)) {
				mutex_unlock(&fb_data->queue_mutex);
				return;
		}

		list_for_each_entry_safe(next_desc, temp_desc,
			&fb_data->upd_pending_list, list) {

			dev_dbg(fb_data->dev, "Found a pending update!\n");

			if (!upd_data_list) {
				if (list_empty(&fb_data->upd_buf_free_list))
					break;
				upd_data_list =
					list_entry(fb_data->upd_buf_free_list.next,
						struct update_data_list, list);
				list_del_init(&upd_data_list->list);
				upd_data_list->update_desc = next_desc;
				list_del_init(&next_desc->list);
				if (fb_data->upd_scheme ==
					UPDATE_SCHEME_QUEUE)
					/* If not merging, we have an update */
					break;
			 } else {
				switch (epdc_submit_merge(upd_data_list->update_desc,
						next_desc)) {
			       case MERGE_OK:
			               dev_dbg(fb_data->dev,
			                       "Update merged [queue]\n");
					list_del_init(&next_desc->list);
					kfree(next_desc);
			               break;
			       case MERGE_FAIL:
			               dev_dbg(fb_data->dev,
			                       "Update not merged [queue]\n");
			               break;
			       case MERGE_BLOCK:
			               dev_dbg(fb_data->dev,
			                       "Merge blocked [collision]\n");
			               end_merge = true;
			               break;
			       }

			       if (end_merge)
			               break;
	               }
		}
	}

	/* Is update list empty? */
	if (!upd_data_list)
	{
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}

	/* Select from PxP output buffers */
	upd_data_list->phys_addr =
		fb_data->phys_addr_updbuf[fb_data->upd_buffer_num];
	upd_data_list->virt_addr =
		fb_data->virt_addr_updbuf[fb_data->upd_buffer_num];
	fb_data->upd_buffer_num++;
	if (fb_data->upd_buffer_num > fb_data->max_num_buffers-1)
		fb_data->upd_buffer_num = 0;
	
	/* Release buffer queues */
	mutex_unlock(&fb_data->queue_mutex);

	/* Perform PXP processing - EPDC power will also be enabled */
	if (epdc_process_update(upd_data_list, fb_data)) {
		dev_dbg(fb_data->dev, "PXP processing error.\n");
		/* Protect access to buffer queues and to update HW */
		mutex_lock(&fb_data->queue_mutex);
		list_del_init(&upd_data_list->update_desc->list);
		kfree(upd_data_list->update_desc);
		upd_data_list->update_desc = NULL;
		/* Add to free buffer list */
		list_add_tail(&upd_data_list->list,
			&fb_data->upd_buf_free_list);
		/* Release buffer queues */
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}
	
	/* Protect access to buffer queues and to update HW */
	mutex_lock(&fb_data->queue_mutex);

	/* Get rotation-adjusted coordinates */
	adjust_coordinates(fb_data,
		&upd_data_list->update_desc->upd_data.update_region,
		&adj_update_region);

	/*
	 * Is the working buffer idle?
	 * If the working buffer is busy, we must wait for the resource
	 * to become free. The IST will signal this event.
	 */
	if (fb_data->cur_update != NULL) {
		dev_dbg(fb_data->dev, "working buf busy!\n");

		/* Initialize event signalling an update resource is free */
		init_completion(&fb_data->update_res_free);

		fb_data->waiting_for_wb = true;

		/* Leave spinlock while waiting for WB to complete */
		mutex_unlock(&fb_data->queue_mutex);
		wait_for_completion(&fb_data->update_res_free);
		mutex_lock(&fb_data->queue_mutex);
	}

	/*
	 * If there are no LUTs available,
	 * then we must wait for the resource to become free.
	 * The IST will signal this event.
	 */
	if (!epdc_any_luts_available()) {
		dev_dbg(fb_data->dev, "no luts available!\n");

		/* Initialize event signalling an update resource is free */
		init_completion(&fb_data->update_res_free);

		fb_data->waiting_for_lut = true;

		/* Leave spinlock while waiting for LUT to free up */
		mutex_unlock(&fb_data->queue_mutex);
		wait_for_completion(&fb_data->update_res_free);
		mutex_lock(&fb_data->queue_mutex);
	}

	ret = epdc_choose_next_lut(&upd_data_list->lut_num);
	/*
	 * If LUT15 is in use:
	 *   - Wait for LUT15 to complete is if TCE underrun prevent is enabled
	 *   - If we go ahead with update, sync update submission with EOF
	 */
	if (ret && fb_data->tce_prevent) {
		dev_dbg(fb_data->dev, "Waiting for LUT15\n");

		/* Initialize event signalling that lut15 is free */
		init_completion(&fb_data->lut15_free);

		fb_data->waiting_for_lut15 = true;

		/* Leave spinlock while waiting for LUT to free up */
		mutex_unlock(&fb_data->queue_mutex);
		wait_for_completion(&fb_data->lut15_free);
		mutex_lock(&fb_data->queue_mutex);

		epdc_choose_next_lut(&upd_data_list->lut_num);
	} else if (ret) {
		/* Synchronize update submission time to reduce
		   chances of TCE underrun */
		init_completion(&fb_data->eof_event);

		epdc_eof_intr(true);

		/* Leave spinlock while waiting for EOF event */
		mutex_unlock(&fb_data->queue_mutex);
		ret = wait_for_completion_timeout(&fb_data->eof_event, msecs_to_jiffies(1000));
		if (ret <= 0) {
			dev_err(fb_data->dev, "** Missed EOF event! **\n");
			epdc_eof_intr(false);
		}
		udelay(fb_data->eof_sync_period);
		mutex_lock(&fb_data->queue_mutex);

	}

	/* LUTs are available, so we get one here */
	fb_data->cur_update = upd_data_list;

	/* Reset mask for LUTS that have completed during WB processing */
	fb_data->luts_complete_wb = 0;

	list_for_each_entry_safe(next_marker, temp_marker,
		&upd_data_list->update_desc->upd_marker_list, upd_list)
		next_marker->lut_num = fb_data->cur_update->lut_num;

	/* Mark LUT with order */
	fb_data->lut_update_order[upd_data_list->lut_num] =
		upd_data_list->update_desc->update_order;

	/* Enable Collision and WB complete IRQs */
	epdc_working_buf_intr(true);
	epdc_lut_complete_intr(upd_data_list->lut_num, true);

	/* Program EPDC update to process buffer */
	if (upd_data_list->update_desc->upd_data.temp == TEMP_USE_AUTO) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
			papyrus_temp);
		epdc_set_temp(temp_index);
	} else if (upd_data_list->update_desc->upd_data.temp != TEMP_USE_AMBIENT) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
				upd_data_list->update_desc->upd_data.temp);
		epdc_set_temp(temp_index);
	}
	epdc_set_update_addr(upd_data_list->phys_addr
				 + upd_data_list->update_desc->epdc_offs);
	epdc_set_update_coord(adj_update_region.left, adj_update_region.top);
	epdc_set_update_dimensions(adj_update_region.width,
				   adj_update_region.height);
	
	if (!atomic_read(&mxc_clear_queue))
	{
		dev_dbg(fb_data->dev, "SUBMIT_WORK update:\n\
				\tUpdate region: [%d,%d,%d,%d]\n \
				\tWaveform : %d\n \
				\tUpdate mode : %d\n \
				\tTemperature : %d\n",
				upd_data_list->update_desc->upd_data.update_region.top,
				upd_data_list->update_desc->upd_data.update_region.left,
				upd_data_list->update_desc->upd_data.update_region.left +
				upd_data_list->update_desc->upd_data.update_region.width,
				upd_data_list->update_desc->upd_data.update_region.top +
				upd_data_list->update_desc->upd_data.update_region.height,
				upd_data_list->update_desc->upd_data.waveform_mode,
				upd_data_list->update_desc->upd_data.update_mode,
				upd_data_list->update_desc->upd_data.temp);
		
		epdc_submit_update(upd_data_list->lut_num,
			upd_data_list->update_desc->upd_data.waveform_mode,
			upd_data_list->update_desc->upd_data.update_mode,
			false, 0);
		
		if (list_empty(&upd_data_list->update_desc->upd_marker_list))
		{
			mutex_unlock(&fb_data->queue_mutex);
			return;
		}
		
		list_for_each_entry_safe(next_marker, temp_marker,
				&upd_data_list->update_desc->upd_marker_list, upd_list)
		{
			if (next_marker->update_marker != 0)
			{
				next_marker->submitted = true;
				complete(&next_marker->submit_completion);
			}
		}
	}
		
	/* Release buffer queues */
	mutex_unlock(&fb_data->queue_mutex);
}

static void mxc_epdc_dump_fb(struct mxcfb_update_data *upd_data, struct mxc_epdc_fb_data *fb_data)
{
	int j, k;
	u8* buf_ptr = (u8 *)(fb_data->info.screen_base + fb_data->fb_offset);
	u8 cur_byte;

	for (j = upd_data->update_region.top;
		j < upd_data->update_region.top + upd_data->update_region.height;
		j++) {
		for (k = upd_data->update_region.left;
			k < upd_data->update_region.left + upd_data->update_region.width;
			k++) {
				cur_byte = buf_ptr[(j * fb_data->info.var.xres_virtual + k) * fb_data->info.var.bits_per_pixel/8];
				printk("0x%x ",cur_byte);
		}
	}
}

int mxc_epdc_fb_send_update(struct mxcfb_update_data *upd_data,
				   struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	struct update_data_list *upd_data_list = NULL;
	struct mxcfb_rect *screen_upd_region; /* Region on screen to update */
	int temp_index;
	int ret;
	struct update_desc_list *upd_desc;
	struct update_marker_data *marker_data, *next_marker, *temp_marker;
	
	if (mxc_epdc_paused) {
		dev_err(fb_data->dev, "Updates paused ... not sending to epdc\n");
		return -EPERM;
	}

	/* Has EPDC HW been initialized? */
	if (!fb_data->hw_ready) {
		dev_err(fb_data->dev, "Display HW not properly initialized."
			"  Aborting update.\n");
		return -EPERM;
	}

	/* Check validity of update params */
	if ((upd_data->update_mode != UPDATE_MODE_PARTIAL) &&
		(upd_data->update_mode != UPDATE_MODE_FULL)) {
		dev_err(fb_data->dev,
			"Update mode 0x%x is invalid.  Aborting update.\n",
			upd_data->update_mode);
		return -EINVAL;
	}
	if ((upd_data->waveform_mode > 255) &&
		(upd_data->waveform_mode != WAVEFORM_MODE_AUTO)) {
		dev_err(fb_data->dev,
			"Update waveform mode 0x%x is invalid."
			"  Aborting update.\n",
			upd_data->waveform_mode);
		return -EINVAL;
	}
		
	if ((upd_data->update_region.left + upd_data->update_region.width > fb_data->epdc_fb_var.xres) ||
		(upd_data->update_region.top + upd_data->update_region.height > fb_data->epdc_fb_var.yres)) {

		dev_err(fb_data->dev,
			"Update region is outside bounds of framebuffer [l:%d,t:%d,w:%d,h:%d]. Aborting update.\n", upd_data->update_region.left, upd_data->update_region.top,
			upd_data->update_region.width, upd_data->update_region.height);
		return -EINVAL;
	}

	if (upd_data->flags & EPDC_FLAG_USE_ALT_BUFFER) {
		if ((upd_data->update_region.width !=
			upd_data->alt_buffer_data.alt_update_region.width) ||
			(upd_data->update_region.height !=
			upd_data->alt_buffer_data.alt_update_region.height)) {
			dev_err(fb_data->dev,
				"Alternate update region dimensions must "
				"match screen update region dimensions.\n");
			return -EINVAL;
		}
		/* Validate physical address parameter */
		if ((upd_data->alt_buffer_data.phys_addr <
			fb_data->info.fix.smem_start) ||
			(upd_data->alt_buffer_data.phys_addr >
			fb_data->info.fix.smem_start + fb_data->map_size)) {
			dev_err(fb_data->dev,
				"Invalid physical address for alternate "
				"buffer.  Aborting update...\n");
			return -EINVAL;
		}
	}

	if(mxc_epdc_debugging)
	{
		printk(KERN_INFO "mxc_epdc_fb: [%d] Requested waveforms: mode: 0x%x __ BW: 0x%x __ Gray : 0x%x\n",
				upd_data->update_marker,
				upd_data->waveform_mode,
				upd_data->hist_bw_waveform_mode,
				upd_data->hist_gray_waveform_mode);
	}
	
	/* Lab126: Convert to valid waveforms */
	upd_data->waveform_mode = get_waveform_by_type(fb_data, upd_data->waveform_mode);
	upd_data->hist_bw_waveform_mode = get_waveform_by_type(fb_data, upd_data->hist_bw_waveform_mode);
	upd_data->hist_gray_waveform_mode = get_waveform_by_type(fb_data, upd_data->hist_gray_waveform_mode);

	/* Lab126: Check validity of prefered waveforms and override if needed */
	if (upd_data->hist_bw_waveform_mode == WAVEFORM_MODE_INIT || upd_data->hist_bw_waveform_mode == WAVEFORM_MODE_AUTO)
		upd_data->hist_bw_waveform_mode = fb_data->wv_modes.mode_du;
	if (upd_data->hist_gray_waveform_mode == WAVEFORM_MODE_INIT || upd_data->hist_gray_waveform_mode == WAVEFORM_MODE_AUTO)
		upd_data->hist_gray_waveform_mode = fb_data->wv_modes.mode_gc16;
	
	if(mxc_epdc_debugging)
	{
		printk(KERN_INFO "mxc_epdc_fb: [%d] Converted waveforms: mode: 0x%x __ BW: 0x%x __ Gray : 0x%x\n", 
				upd_data->update_marker,
				upd_data->waveform_mode,
				upd_data->hist_bw_waveform_mode,
				upd_data->hist_gray_waveform_mode);
	}
	
	mutex_lock(&fb_data->queue_mutex);

	/*
	 * If we are waiting to go into suspend, or the FB is blanked,
	 * we do not accept new updates
	 */
	mutex_lock(&fb_data->power_mutex);
	if ((fb_data->waiting_for_idle) ||
		(fb_data->blank != FB_BLANK_UNBLANK)) {
		dev_err(fb_data->dev, "EPDC not active."
			"Update request abort.\n");
		mutex_unlock(&fb_data->power_mutex);
		mutex_unlock(&fb_data->queue_mutex);
		return -EPERM;
	}
	mutex_unlock(&fb_data->power_mutex);

	if (fb_data->upd_scheme == UPDATE_SCHEME_SNAPSHOT) {
		int count = 0;
		struct update_data_list *plist;
		
		/* Count buffers in free buffer list */
		list_for_each_entry(plist, &fb_data->upd_buf_free_list, list)
			count++;

		/* Use count to determine if we have enough
		 * free buffers to handle this update request */
		if (count + fb_data->max_num_buffers
				<= EPDC_MAX_NUM_UPDATES) {
		dev_err(fb_data->dev,
					"No free intermediate buffers available.\n");
			mutex_unlock(&fb_data->queue_mutex);
			return -ENOMEM;
		}

		/* Grab first available buffer and delete from the free list */
		upd_data_list =
			list_entry(fb_data->upd_buf_free_list.next,
					struct update_data_list, list);

		list_del_init(&upd_data_list->list);
	}
	/*
	 * Get available intermediate (PxP output) buffer to hold
	 * processed update region
	 */
	upd_desc = kzalloc(sizeof(struct update_desc_list), GFP_ATOMIC);
	if (!upd_desc) {
		dev_err(fb_data->dev,
			"Insufficient system memory for update! Aborting.\n");
		if (fb_data->upd_scheme == UPDATE_SCHEME_SNAPSHOT) {
			list_add(&upd_data_list->list,
				&fb_data->upd_buf_free_list);
		}
		mutex_unlock(&fb_data->queue_mutex);
		return -EPERM;
	}

	/* Initialize per-update marker list */
	INIT_LIST_HEAD(&upd_desc->upd_marker_list);
	upd_desc->upd_data = *upd_data;
	upd_desc->update_order = fb_data->order_cnt++;
	list_add_tail(&upd_desc->list, &fb_data->upd_pending_list);

	/* If marker specified, associate it with a completion */
	if (upd_data->update_marker != 0) {
		/* Allocate new update marker and set it up */
		marker_data = kzalloc(sizeof(struct update_marker_data),
				GFP_ATOMIC);
		if (!marker_data) {
			dev_err(fb_data->dev, "No memory for marker!\n");
			mutex_unlock(&fb_data->queue_mutex);
			return -ENOMEM;
		}
		list_add_tail(&marker_data->upd_list,
			&upd_desc->upd_marker_list);
		marker_data->update_marker = upd_data->update_marker;
		marker_data->lut_num = INVALID_LUT;
		marker_data->submitted = false;
		init_completion(&marker_data->update_completion);
		init_completion(&marker_data->submit_completion);
		/* Add marker to master marker list */
		list_add_tail(&marker_data->full_list,
			&fb_data->full_marker_list);

		if (mxc_epdc_debugging) {
			marker_data->start_time = timeofday_msec();
			printk(KERN_INFO "mxc_epdc_fb: [%d] update start marker=%d, start time=%lld\n",
					marker_data->update_marker, marker_data->update_marker, marker_data->start_time);
			printk(KERN_INFO "mxc_epdc_fb: [%d] waveform=0x%x mode=0x%x temp:%d update region top=%d, left=%d, width=%d, heigth=%d\n", marker_data->update_marker, upd_data->waveform_mode, upd_data->update_mode, papyrus_temp, upd_data->update_region.top, upd_data->update_region.left, upd_data->update_region.width, upd_data->update_region.height);
		}
	}

	if (fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT) {

		/* Signal workqueue to handle new update */
		queue_work(fb_data->epdc_submit_workqueue,
			&fb_data->epdc_submit_work);
		
		/* Queued update scheme processing */
		mutex_unlock(&fb_data->queue_mutex);

		return 0;
	}

	/* Snapshot update scheme processing */

	/* Set descriptor for current update, delete from pending list */
	upd_data_list->update_desc = upd_desc;
	list_del_init(&upd_desc->list);

	mutex_unlock(&fb_data->queue_mutex);

	/*
	 * Hold on to original screen update region, which we
	 * will ultimately use when telling EPDC where to update on panel
	 */
	screen_upd_region = &upd_desc->upd_data.update_region;

	/* Select from PxP output buffers */
	upd_data_list->phys_addr = 
		fb_data->phys_addr_updbuf[fb_data->upd_buffer_num];
	upd_data_list->virt_addr =
		fb_data->virt_addr_updbuf[fb_data->upd_buffer_num];
	fb_data->upd_buffer_num++;
	if (fb_data->upd_buffer_num > fb_data->max_num_buffers-1)
		fb_data->upd_buffer_num = 0;
	
	ret = epdc_process_update(upd_data_list, fb_data);
	if (ret) {
		return ret;
	}

	/* Pass selected waveform mode back to user */
	upd_data->waveform_mode = upd_desc->upd_data.waveform_mode;

	/* Get rotation-adjusted coordinates */
	adjust_coordinates(fb_data, &upd_desc->upd_data.update_region,
		NULL);

	/* Grab lock for queue manipulation and update submission */
	mutex_lock(&fb_data->queue_mutex);

	/*
	 * Is the working buffer idle?
	 * If either the working buffer is busy, or there are no LUTs available,
	 * then we return and let the ISR handle the update later
	 */
	if ((fb_data->cur_update != NULL) || !epdc_any_luts_available()) {
		/* Add processed Y buffer to update list */
		list_add_tail(&upd_data_list->list, &fb_data->upd_buf_queue);

		/* Return and allow the update to be submitted by the ISR. */
		mutex_unlock(&fb_data->queue_mutex);
		return 0;
	}

	/* LUTs are available, so we get one here */
	ret = epdc_choose_next_lut(&upd_data_list->lut_num);
	if (ret && fb_data->tce_prevent) {
		dev_dbg(fb_data->dev, "Must wait for LUT15\n");
		/* Add processed Y buffer to update list */
		list_add_tail(&upd_data_list->list, &fb_data->upd_buf_queue);

		/* Return and allow the update to be submitted by the ISR. */
		mutex_unlock(&fb_data->queue_mutex);

		return 0;
	}
	
	/* Save current update */
	fb_data->cur_update = upd_data_list;

	/* Reset mask for LUTS that have completed during WB processing */
	fb_data->luts_complete_wb = 0;

	/* Associate LUT with update marker */
	list_for_each_entry_safe(next_marker, temp_marker,
		&upd_data_list->update_desc->upd_marker_list, upd_list)
		next_marker->lut_num = upd_data_list->lut_num;

	/* Mark LUT as containing new update */
	fb_data->lut_update_order[upd_data_list->lut_num] =
		upd_desc->update_order;

	/* Clear status and Enable LUT complete and WB complete IRQs */
	epdc_working_buf_intr(true);
	epdc_lut_complete_intr(upd_data_list->lut_num, true);

	/* Program EPDC update to process buffer */
	epdc_set_update_addr(upd_data_list->phys_addr + upd_desc->epdc_offs);
	epdc_set_update_coord(screen_upd_region->left, screen_upd_region->top);
	epdc_set_update_dimensions(screen_upd_region->width,
		screen_upd_region->height);
	if (upd_desc->upd_data.temp == TEMP_USE_AUTO) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
			papyrus_temp);
		epdc_set_temp(temp_index);
	} else if (upd_desc->upd_data.temp != TEMP_USE_AMBIENT) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
			upd_desc->upd_data.temp);
		epdc_set_temp(temp_index);
	} else
		epdc_set_temp(fb_data->temp_index);

	dev_dbg(fb_data->dev, "SENDUPDATE update:\n\
			\tUpdate region: [%d,%d,%d,%d]\n \
			\tWaveform : %d\n \
			\tUpdate mode : %d\n \
			\tTemperature : %d\n",
			upd_desc->upd_data.update_region.top,
			upd_desc->upd_data.update_region.left,
			upd_desc->upd_data.update_region.left +
			upd_desc->upd_data.update_region.width,
			upd_desc->upd_data.update_region.top +
			upd_desc->upd_data.update_region.height,
			upd_desc->upd_data.waveform_mode,
			upd_desc->upd_data.update_mode,
			upd_desc->upd_data.temp);
	epdc_submit_update(upd_data_list->lut_num,
			   upd_desc->upd_data.waveform_mode,
			   upd_desc->upd_data.update_mode, false, 0);

	mutex_unlock(&fb_data->queue_mutex);
	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_send_update);

int mxc_epdc_fb_wait_update_complete(u32 update_marker, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	struct update_marker_data *next_marker;
	struct update_marker_data *temp;
	bool marker_found = false;
	int ret = 0;

	/* 0 is an invalid update_marker value */
	if (update_marker == 0)
		return -EINVAL;

	/*
	 * Wait for completion associated with update_marker requested.
	 * Note: If update completed already, marker will have been
	 * cleared and we will just return
	 */
	/* Grab queue lock to protect access to marker list */
	mutex_lock(&fb_data->queue_mutex);

	list_for_each_entry_safe(next_marker, temp,
		&fb_data->full_marker_list, full_list) {
		if (next_marker->update_marker == update_marker) {
			dev_dbg(fb_data->dev, "Waiting for marker %d\n",
				update_marker);
			next_marker->waiting = true;
			marker_found = true;
			break;
		}
	}

	mutex_unlock(&fb_data->queue_mutex);

       /*
        * If marker not found, it has either been signalled already
        * or the update request failed.  In either case, just return.
        */
       if (!marker_found)
               return ret;

       ret = wait_for_completion_timeout(&next_marker->update_completion,
                                               msecs_to_jiffies(5000));
       if (!ret) {
               dev_err(fb_data->dev,
                       "Timed out waiting for update completion %d\n", update_marker);
		list_del_init(&next_marker->full_list);
		ret = -ETIMEDOUT;
       }

       /* Free update marker object */
       kfree(next_marker);

       return ret;
}
EXPORT_SYMBOL(mxc_epdc_fb_wait_update_complete);

int mxc_epdc_fb_wait_update_submission(u32 update_marker, struct fb_info *info)
{ 
  struct mxc_epdc_fb_data *fb_data = info ?
    (struct mxc_epdc_fb_data *)info:g_fb_data;
  struct update_marker_data *next_marker;
  struct update_marker_data *temp; 
  bool marker_found = false;
  int ret = 0;
  
  /* 0 is an invalid update_marker value */
  if (update_marker == 0)
    return -EINVAL;

	/* 
	 * Wait for completion associated with update_marker requested.
	 * Note: If update completed already, marker will have been
	 * cleared and we will just return
	 */ 
	/* Grab queue lock to protect access to marker list */
	mutex_lock(&fb_data->queue_mutex);

	list_for_each_entry_safe(next_marker, temp,
			&fb_data->full_marker_list, full_list) {
		if (next_marker->update_marker == update_marker) {
			dev_dbg(fb_data->dev, "Waiting for marker %d\n",
					update_marker);
			marker_found = true;
			break;
		}
	}

	mutex_unlock(&fb_data->queue_mutex);

	/*
	 * If marker not found, it has either been signalled already,
	 * the update request failed or it has already been submitted.
	 * In either case, just return.
	 */
	if (!marker_found || next_marker->submitted)
	{
		return ret;
	}

	ret = wait_for_completion_timeout(&next_marker->submit_completion,
			msecs_to_jiffies(1000));
	if (!ret) {
		dev_err(fb_data->dev,
				"Timed out waiting for update submission %d\n", update_marker);
		ret = -ETIMEDOUT;
	}

	return ret;
} 
EXPORT_SYMBOL(mxc_epdc_fb_wait_update_submission);

int mxc_epdc_fb_set_pwrdown_delay(u32 pwrdown_delay,
					    struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	fb_data->pwrdown_delay = pwrdown_delay;

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_pwrdown_delay);

int mxc_epdc_get_pwrdown_delay(struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	return fb_data->pwrdown_delay;
}
EXPORT_SYMBOL(mxc_epdc_get_pwrdown_delay);

static void  mxc_epdc_fb_send_full_update(struct mxc_epdc_fb_data *fb_data);

static void ff_work_fn(struct work_struct *work)
{
	mxc_epdc_fb_send_full_update(g_fb_data);
}
	
DECLARE_DELAYED_WORK(ff_work, ff_work_fn);

static int mxc_epdc_fb_ioctl(struct fb_info *info, unsigned int cmd,
			     unsigned long arg)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	void __user *argp = (void __user *)arg;
	int ret = -EINVAL;

	switch (cmd) {
	case MXCFB_SET_WAVEFORM_MODES:
		{
			struct mxcfb_waveform_modes modes;
			if (!copy_from_user(&modes, argp, sizeof(modes))) {
				mxc_epdc_fb_set_waveform_modes(&modes, info);
				ret = 0;
			}
			break;
		}
	case MXCFB_SET_TEMPERATURE:
		{
			int temperature;
			if (!get_user(temperature, (int32_t __user *) arg))
				ret = mxc_epdc_fb_set_temperature(temperature,
					info);
			break;
		}
	case MXCFB_GET_TEMPERATURE:
		{
			ret = 0;
			if (put_user(papyrus_temp, (int __user *)argp))
				ret = -EFAULT;
			break;
		}
	case MXCFB_SET_AUTO_UPDATE_MODE:
		{
			u32 auto_mode = 0;
			if (!get_user(auto_mode, (__u32 __user *) arg))
				ret = mxc_epdc_fb_set_auto_update(auto_mode,
					info);
			break;
		}
	case MXCFB_SET_UPDATE_SCHEME:
		{
			u32 upd_scheme = 0;
			if (!get_user(upd_scheme, (__u32 __user *) arg))
				ret = mxc_epdc_fb_set_upd_scheme(upd_scheme,
					info);
			break;
		}
	case MXCFB_SEND_UPDATE:
		{
			struct mxcfb_update_data upd_data;

			if (!copy_from_user(&upd_data, argp, sizeof(upd_data))) {
				ret = mxc_epdc_fb_send_update(&upd_data, info);
				if (ret == 0 && copy_to_user(argp, &upd_data,
							sizeof(upd_data))) {
					ret = -EFAULT;
				}
			}
			else {
				ret = -EFAULT;
			}
			break;
		}
	case MXCFB_CLEAR_UPDATE_QUEUE:
		{
			atomic_set(&mxc_clear_queue, 1);
			flush_workqueue(fb_data->epdc_submit_workqueue);
			ret = 0;
			atomic_set(&mxc_clear_queue, 0);
			break;
		}
	case MXCFB_WAIT_FOR_UPDATE_COMPLETE:
		{
			u32 update_marker = 0;
			if (!get_user(update_marker, (__u32 __user *) arg))
				ret =
				    mxc_epdc_fb_wait_update_complete(update_marker,
					info);
			break;
		}
	case MXCFB_WAIT_FOR_UPDATE_SUBMISSION:
		{
			u32 update_marker = 0;
			if (!get_user(update_marker, (__u32 __user *) arg))
				ret =
				    mxc_epdc_fb_wait_update_submission(update_marker,
					info);
			break;
		}
	case MXCFB_SET_PWRDOWN_DELAY:
		{
			int delay = 0;
			if (!get_user(delay, (__u32 __user *) arg))
				ret =
				    mxc_epdc_fb_set_pwrdown_delay(delay, info);
			break;
		}

	case MXCFB_GET_PWRDOWN_DELAY:
		{
			int pwrdown_delay = mxc_epdc_get_pwrdown_delay(info);
			ret = 0;

			if (put_user(pwrdown_delay,
				(int __user *)argp))
				ret = -EFAULT;
			break;
		}
	case MXCFB_SET_PAUSE:
		{
			mxc_epdc_paused = 1;
			ret = 0;
			break;
		}
	case MXCFB_GET_PAUSE:
		{
			if (put_user(mxc_epdc_paused,
				(int __user *)argp))
				ret = -EFAULT;
			ret = 0;
			break;
		}
	case MXCFB_SET_RESUME:
		{
			mxc_epdc_paused = 0; 
			ret = 0;
			break;
		}
	case MXCFB_GET_WAVEFORM_TYPE:
		{
			struct mxc_epdc_fb_data *fb_data = info ? (struct mxc_epdc_fb_data *)info:g_fb_data;
			if (put_user(fb_data->waveform_type, (int __user *)argp))
				ret = -EFAULT;
			ret = 0;
			break;
		}
	default:
		break;
	}
	return ret;
}

static void  mxc_epdc_fb_send_full_update(struct mxc_epdc_fb_data *fb_data)
{
	struct mxcfb_update_data update;

	if (mxc_epdc_paused)
		return;

	mutex_lock(&fb_data->pxp_mutex);

	update.update_region.left = 0;
	update.update_region.width = fb_data->info.var.xres;
	update.update_region.top = 0;
	update.update_region.height = fb_data->info.var.yres;
	update.update_mode = UPDATE_MODE_PARTIAL;

	mxc_epdc_marker++;
	update.update_marker = mxc_epdc_marker;
	update.waveform_mode = WAVEFORM_MODE_AUTO;
	update.hist_bw_waveform_mode = 0;
	update.hist_gray_waveform_mode = 0;
	update.flags = 0;

	mutex_unlock(&fb_data->pxp_mutex);

	mxc_epdc_fb_send_update(&update, &fb_data->info);
}

static void mxc_epdc_fb_update_pages(struct mxc_epdc_fb_data *fb_data,
				     u16 y1, u16 y2)
{
	struct mxcfb_update_data update;

  
	cancel_rearming_delayed_work(&ff_work);

	/* Do partial screen update, Update full horizontal lines */
	update.update_region.left = 0;
	update.update_region.width = fb_data->epdc_fb_var.xres;
	update.update_region.top = y1;
	update.update_region.height = y2 - y1;
	update.waveform_mode = WAVEFORM_MODE_AUTO;
	update.update_mode = UPDATE_MODE_PARTIAL;
	update.update_marker = 0;
//	update.temp = TEMP_USE_AMBIENT; //papyrus_temp;
	update.temp = TEMP_USE_AUTO;
	update.flags = 0;
	update.hist_bw_waveform_mode = 0;
	update.hist_gray_waveform_mode = 0;
	
	/* Set the scheme to QUEUE_AND_MERGE */
	fb_data->upd_scheme = UPDATE_SCHEME_QUEUE_AND_MERGE;

	mxc_epdc_fb_send_update(&update, &fb_data->info);

	schedule_delayed_work(&ff_work, msecs_to_jiffies(1000));
}

/* this is called back from the deferred io workqueue */
static void mxc_epdc_fb_deferred_io(struct fb_info *info,
				    struct list_head *pagelist)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	struct page *page;
	unsigned long beg, end;
	int y1, y2, miny, maxy;

	/* Lab126 */
	if (fb_data->auto_mode == AUTO_UPDATE_MODE_REGION_MODE)
		return;

	miny = INT_MAX;
	maxy = 0;
	list_for_each_entry(page, pagelist, lru) {
		beg = page->index << PAGE_SHIFT;
		end = beg + PAGE_SIZE - 1;
		y1 = beg / info->fix.line_length;
		y2 = end / info->fix.line_length;
		if (y2 >= fb_data->epdc_fb_var.yres)
			y2 = fb_data->epdc_fb_var.yres - 1;
		if (miny > y1)
			miny = y1;
		if (maxy < y2)
			maxy = y2;
	}

	mxc_epdc_fb_update_pages(fb_data, miny, maxy);
}

void mxc_epdc_fb_flush_updates(struct mxc_epdc_fb_data *fb_data)
{
	int ret = 0;

	if (fb_data->in_init)
		return;

	/* Grab queue lock to prevent any new updates from being submitted */
	mutex_lock(&fb_data->queue_mutex);
	mutex_lock(&fb_data->power_mutex);
	
	if (!list_empty(&fb_data->upd_pending_list) ||
		!is_free_list_full(fb_data) ||
		(fb_data->updates_active == true)) {
		/* Initialize event signalling updates are done */
		init_completion(&fb_data->updates_done);
		fb_data->waiting_for_idle = true;
		mutex_unlock(&fb_data->power_mutex);
		mutex_unlock(&fb_data->queue_mutex);
		
		/* Wait for any currently active updates to complete */
		ret = wait_for_completion_timeout(&fb_data->updates_done,
				msecs_to_jiffies(5000));
		if (!ret)
			dev_err(fb_data->dev,
				"Flush updates timeout! ret = 0x%x\n", ret);

		mutex_lock(&fb_data->queue_mutex);
		mutex_lock(&fb_data->power_mutex);
		fb_data->waiting_for_idle = false;
	}

	mutex_unlock(&fb_data->power_mutex);
	mutex_unlock(&fb_data->queue_mutex);
}

static int mxc_epdc_fb_blank(int blank, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	int ret = 0;

	dev_dbg(fb_data->dev, "blank = %d\n", blank);
	
	if (fb_data->blank == blank)
		return 0;

	/* Lab126:  Eliminated FB_BLANK_UNBLANK. */
	switch (blank) {
	case FB_BLANK_POWERDOWN:
		mxc_epdc_fb_flush_updates(fb_data);
		/* Wait for powerdown */
		mutex_lock(&fb_data->power_mutex);
		if ((fb_data->power_state == POWER_STATE_ON) &&
				(fb_data->pwrdown_delay == FB_POWERDOWN_DISABLE)) {
			/* Powerdown disabled, so we disable EPDC manually */
			int count = 0;
			int sleep_ms = 10;

			mutex_unlock(&fb_data->power_mutex);

			/* If any active updates, wait for them to complete */
			while (fb_data->updates_active) {
				/* Timeout after 1 sec */
				if ((count * sleep_ms) > 1000)
					break;
				msleep(sleep_ms);
				count++;
			}

			mutex_lock(&fb_data->power_mutex);
			fb_data->powering_down = true;
			epdc_powerdown(fb_data);
			mutex_unlock(&fb_data->power_mutex);
		}
		else if (fb_data->power_state != POWER_STATE_OFF) {
			fb_data->wait_for_powerdown = true;
			init_completion(&fb_data->powerdown_compl);
			mutex_unlock(&fb_data->power_mutex);
			ret = wait_for_completion_timeout(&fb_data->powerdown_compl,
				msecs_to_jiffies(5000));
			if (!ret) {
				dev_err(fb_data->dev,
					"No powerdown received!\n");
				return -ETIMEDOUT;
			}
		}
		else
			mutex_unlock(&fb_data->power_mutex);
	case FB_BLANK_UNBLANK:
		fb_data->blank = blank;
		break;
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		mxc_epdc_fb_flush_updates(fb_data);
		break;
	}
	return 0;
}

static int mxc_epdc_fb_pan_display(struct fb_var_screeninfo *var,
				   struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	u_int y_bottom;

	dev_dbg(info->device, "%s: var->xoffset %d, info->var.xoffset %d\n",
		 __func__, var->xoffset, info->var.xoffset);
	/* check if var is valid; also, xpan is not supported */
	if (!var || (var->xoffset != info->var.xoffset) ||
	    (var->yoffset + var->yres > var->yres_virtual)) {
		dev_dbg(info->device, "x panning not supported\n");
		return -EINVAL;
	}

	if ((fb_data->epdc_fb_var.xoffset == var->xoffset) &&
		(fb_data->epdc_fb_var.yoffset == var->yoffset))
		return 0;	/* No change, do nothing */

	y_bottom = var->yoffset;

	if (!(var->vmode & FB_VMODE_YWRAP))
		y_bottom += var->yres;

	if (y_bottom > info->var.yres_virtual)
		return -EINVAL;

	fb_data->fb_offset = (var->yoffset * var->xres_virtual + var->xoffset)
		* (var->bits_per_pixel) / 8;

	fb_data->epdc_fb_var.xoffset = var->xoffset;
	fb_data->epdc_fb_var.yoffset = var->yoffset;

	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;

	return 0;
}

static struct fb_ops mxc_epdc_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mxc_epdc_fb_check_var,
	.fb_set_par = mxc_epdc_fb_set_par,
	.fb_setcmap = mxc_epdc_fb_setcmap,
	.fb_setcolreg = mxc_epdc_fb_setcolreg,
	.fb_pan_display = mxc_epdc_fb_pan_display,
	.fb_ioctl = mxc_epdc_fb_ioctl,
	.fb_mmap = mxc_epdc_fb_mmap,
	.fb_blank = mxc_epdc_fb_blank,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static struct fb_deferred_io mxc_epdc_fb_defio = {
	.delay = HZ/2,
	.deferred_io = mxc_epdc_fb_deferred_io,
};

static void epdc_done_work_func(struct work_struct *work)
{
	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data,
			epdc_done_work.work);
	mutex_lock(&fb_data->power_mutex);
	epdc_powerdown(fb_data);
	mutex_unlock(&fb_data->power_mutex);
}

static bool is_free_list_full(struct mxc_epdc_fb_data *fb_data)
{
	int count = 0;
	struct update_data_list *plist;

	/* Count buffers in free buffer list */
	list_for_each_entry(plist, &fb_data->upd_buf_free_list, list)
		count++;

	/* Check to see if all buffers are in this list */
	if (count == EPDC_MAX_NUM_UPDATES)
		return true;
	else
		return false;
}

static irqreturn_t mxc_epdc_irq_handler(int irq, void *dev_id)
{
	struct mxc_epdc_fb_data *fb_data = dev_id;
	u32 ints_fired;


	/*
	 * If we just completed one-time panel init, bypass
	 * queue handling, clear interrupt and return
	 */
	if (fb_data->in_init) {
		if (epdc_is_working_buffer_complete()) {
			epdc_working_buf_intr(false);
			epdc_clear_working_buf_irq();
			dev_dbg(fb_data->dev, "Cleared WB for init update\n");
		}

		if (epdc_is_lut_complete(0)) {
			epdc_lut_complete_intr(0, false);
			epdc_clear_lut_complete_irq(0);
			fb_data->in_init = false;
			dev_dbg(fb_data->dev, "Cleared LUT complete for init update\n");
		}

		return IRQ_HANDLED;
	}

	if (!(__raw_readl(EPDC_IRQ_MASK) & __raw_readl(EPDC_IRQ)))
		return IRQ_HANDLED;

	if (__raw_readl(EPDC_IRQ) & EPDC_IRQ_TCE_UNDERRUN_IRQ) {
		dev_err(fb_data->dev,
			"TCE underrun! Will continue to update panel\n");
		/* Clear TCE underrun IRQ */
		__raw_writel(EPDC_IRQ_TCE_UNDERRUN_IRQ, EPDC_IRQ_CLEAR);
	}

	/* Check if we are waiting on EOF to sync a new update submission */
	if (epdc_signal_eof()) {
		epdc_eof_intr(false);
		epdc_clear_eof_irq();
		complete(&fb_data->eof_event);
	}
		/* Clear the interrupt mask for any interrupts signalled */
	ints_fired = __raw_readl(EPDC_IRQ_MASK) & __raw_readl(EPDC_IRQ);
	__raw_writel(ints_fired, EPDC_IRQ_MASK_CLEAR);

	queue_work(fb_data->epdc_intr_workqueue,
		&fb_data->epdc_intr_work);

	return IRQ_HANDLED;
}

static void epdc_intr_work_func(struct work_struct *work)
{
	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data, epdc_intr_work);
	struct update_data_list *collision_update;
	struct mxcfb_rect *next_upd_region;
	struct update_marker_data *next_marker;
	struct update_marker_data *temp;
	int temp_index;
	u32 temp_mask;
	u32 lut;
	bool ignore_collision = false;
	int i;
	bool wb_lut_done = false;
	bool free_update = true;
	int next_lut;
	u32 epdc_irq_stat, epdc_luts_active, epdc_wb_busy, epdc_luts_avail;
	u32 epdc_collision, epdc_colliding_luts, epdc_next_lut_15;
	bool epdc_waiting_on_wb;

	/* Protect access to buffer queues and to update HW */
	mutex_lock(&fb_data->queue_mutex);

	/* Capture EPDC status one time up front to prevent race conditions */
	epdc_luts_active = epdc_any_luts_active();
	epdc_wb_busy = epdc_is_working_buffer_busy();
	epdc_luts_avail = epdc_any_luts_available();
	epdc_collision = epdc_is_collision();
	epdc_colliding_luts = epdc_get_colliding_luts();
	epdc_irq_stat = __raw_readl(EPDC_IRQ);
	epdc_waiting_on_wb = (fb_data->cur_update != NULL) ? true : false;

	/* Free any LUTs that have completed */
	for (i = 0; i < EPDC_NUM_LUTS; i++) {
		if (!(epdc_irq_stat & (1 << i)))
			continue;

		dev_dbg(fb_data->dev, "\nLUT %d completed\n", i);

		/* Disable IRQ for completed LUT */
		epdc_lut_complete_intr(i, false);

		/*
		 * Go through all updates in the collision list and
		 * unmask any updates that were colliding with
		 * the completed LUT.
		 */
		list_for_each_entry(collision_update,
				    &fb_data->upd_buf_collision_list, list) {
			collision_update->collision_mask =
			    collision_update->collision_mask & ~(1 << i);
		}

		epdc_clear_lut_complete_irq(i);

		fb_data->luts_complete_wb |= 1 << i;

		fb_data->lut_update_order[i] = 0;

		/* Signal completion if submit workqueue needs a LUT */
		if (fb_data->waiting_for_lut) {
			complete(&fb_data->update_res_free);
			fb_data->waiting_for_lut = false;
		}

		/* Signal completion if LUT15 free and is needed */
		if (fb_data->waiting_for_lut15 && (i == 15)) {
			complete(&fb_data->lut15_free);
			fb_data->waiting_for_lut15 = false;
		}

		/* Detect race condition where WB and its LUT complete
		   (i.e. full update completes) in one swoop */
		if (fb_data->cur_update &&
			(i == fb_data->cur_update->lut_num))
			wb_lut_done = true;

		/* Signal completion if anyone waiting on this LUT */
		if (!wb_lut_done)
			list_for_each_entry_safe(next_marker, temp,
				&fb_data->full_marker_list,
				full_list) {
				if (next_marker->lut_num != i)
					continue;

				/* Found marker to signal - remove from list */
				list_del_init(&next_marker->full_list);

				/* Signal completion of update */
				dev_dbg(fb_data->dev, "Signaling marker %d\n",
					next_marker->update_marker);

				if (mxc_epdc_debugging &&
					(next_marker->update_marker != 0) ) {
                                        long long end_time = timeofday_msec();
                                        printk(KERN_INFO "mxc_epdc_fb: [%d] update end marker=%u, end time=%lld, time taken=%lld ms\n",
                                                next_marker->update_marker, next_marker->update_marker, end_time, end_time - next_marker->start_time);
				}

				if (next_marker->waiting)
					complete(&next_marker->update_completion);
				else
					kfree(next_marker);
			}
	}

	/* Check to see if all updates have completed */
	if (list_empty(&fb_data->upd_pending_list) &&
		is_free_list_full(fb_data) &&
		(fb_data->cur_update == NULL) &&
		!epdc_luts_active) {

		mutex_lock(&fb_data->power_mutex);
		fb_data->updates_active = false;
		
		if (fb_data->pwrdown_delay != FB_POWERDOWN_DISABLE) {
			/*
			 * Set variable to prevent overlapping
			 * enable/disable requests
			 */
			fb_data->powering_down = true;

			/* Schedule task to disable EPDC HW until next update */
			schedule_delayed_work(&fb_data->epdc_done_work,
				msecs_to_jiffies(fb_data->pwrdown_delay));

			/* Reset counter to reduce chance of overflow */
			fb_data->order_cnt = 0;
		}

		if (fb_data->waiting_for_idle)
			complete(&fb_data->updates_done);
		mutex_unlock(&fb_data->power_mutex);
	}

	/* Is Working Buffer busy? */
	if (epdc_wb_busy) {
		/* Can't submit another update until WB is done */
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}

	/*
	 * Were we waiting on working buffer?
	 * If so, update queues and check for collisions
	 */
	if (epdc_waiting_on_wb) {
		dev_dbg(fb_data->dev, "\nWorking buffer completed\n");

		/* Signal completion if submit workqueue was waiting on WB */
		if (fb_data->waiting_for_wb) {
			complete(&fb_data->update_res_free);
			fb_data->waiting_for_wb = false;
		}

		/* Was there a collision? */
		if (epdc_collision) {
			/* Check list of colliding LUTs, and add to our collision mask */
			fb_data->cur_update->collision_mask =
				epdc_colliding_luts;

			/* Clear collisions that completed since WB began */
			fb_data->cur_update->collision_mask &=
				~fb_data->luts_complete_wb;

			dev_dbg(fb_data->dev, "\nCollision mask = 0x%x\n",
			       fb_data->cur_update->collision_mask);

			/*
			 * If we collide with newer updates, then
			 * we don't need to re-submit the update. The
			 * idea is that the newer updates should take
			 * precedence anyways, so we don't want to
			 * overwrite them.
			 */
			for (temp_mask = fb_data->cur_update->collision_mask, lut = 0;
				temp_mask != 0;
				lut++, temp_mask = temp_mask >> 1) {
				if (!(temp_mask & 0x1))
					continue;

				if (fb_data->lut_update_order[lut] >=
					fb_data->cur_update->update_desc->update_order) {
					dev_dbg(fb_data->dev,
						"Ignoring collision with"
						"newer update.\n");
					ignore_collision = true;
					break;
				}
			}

			if (!ignore_collision) {
				free_update = false;
				/*
				 * If update has markers, clear the LUTs to
				 * avoid signalling that they have completed.
				 */
				list_for_each_entry_safe(next_marker, temp,
					&fb_data->cur_update->update_desc->upd_marker_list,
					upd_list) {
					next_marker->lut_num = INVALID_LUT;
				}

				/* Move to collision list */
				list_add_tail(&fb_data->cur_update->list,
					 &fb_data->upd_buf_collision_list);
			}
		}

		if (free_update) {
			/* Handle condition where WB & LUT are both complete */
			if (wb_lut_done)
				list_for_each_entry_safe(next_marker, temp,
					&fb_data->cur_update->update_desc->upd_marker_list,
					upd_list) {

					/* Del from per-update & full list */
					list_del_init(&next_marker->upd_list);
					list_del_init(&next_marker->full_list);

					/* Signal completion of update */
					dev_dbg(fb_data->dev,
						"Signaling marker %d\n",
						next_marker->update_marker);

					if (next_marker->waiting)
						complete(&next_marker->update_completion);
					else
						kfree(next_marker);
				}

			/* Free marker list and update descriptor */
			kfree(fb_data->cur_update->update_desc);

			/* Add to free buffer list */
			list_add_tail(&fb_data->cur_update->list,
				 &fb_data->upd_buf_free_list);
		}

		/* Clear current update */
		fb_data->cur_update = NULL;

		/* Clear IRQ for working buffer */
		epdc_working_buf_intr(false);
		epdc_clear_working_buf_irq();
	}

	if (fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT) {
		/* Queued update scheme processing */

		/* Schedule task to submit collision and pending update */
		if (!fb_data->powering_down)
			queue_work(fb_data->epdc_submit_workqueue,
				&fb_data->epdc_submit_work);

		/* Release buffer queues */
		mutex_unlock(&fb_data->queue_mutex);

		return;
	}

	/* Snapshot update scheme processing */

	/* Check to see if any LUTs are free */
	if (!epdc_luts_avail) {
		dev_dbg(fb_data->dev, "No luts available.\n");
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}

	epdc_next_lut_15 = epdc_choose_next_lut(&next_lut);
	/* Check to see if there is a valid LUT to use */
	if (epdc_next_lut_15 && fb_data->tce_prevent) {
		dev_dbg(fb_data->dev, "Must wait for LUT15\n");
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}

	/*
	 * Are any of our collision updates able to go now?
	 * Go through all updates in the collision list and check to see
	 * if the collision mask has been fully cleared
	 */
	list_for_each_entry(collision_update,
			    &fb_data->upd_buf_collision_list, list) {

		if (collision_update->collision_mask != 0)
			continue;

		dev_dbg(fb_data->dev, "A collision update is ready to go!\n");
		/*
		 * We have a collision cleared, so select it
		 * and we will retry the update
		 */
		fb_data->cur_update = collision_update;
		list_del_init(&fb_data->cur_update->list);
		break;
	}

	/*
	 * If we didn't find a collision update ready to go,
	 * we try to grab one from the update queue
	 */
	if (fb_data->cur_update == NULL) {
		/* Is update list empty? */
		if (list_empty(&fb_data->upd_buf_queue)) {
			dev_dbg(fb_data->dev, "No pending updates.\n");

			/* No updates pending, so we are done */
			mutex_unlock(&fb_data->queue_mutex);
			return;
		} else {
			dev_dbg(fb_data->dev, "Found a pending update!\n");

			/* Process next item in update list */
			fb_data->cur_update =
			    list_entry(fb_data->upd_buf_queue.next,
				       struct update_data_list, list);
			list_del_init(&fb_data->cur_update->list);
		}
	}

	/* Use LUT selected above */
	fb_data->cur_update->lut_num = next_lut;

	/* Associate LUT with update markers */
	list_for_each_entry_safe(next_marker, temp,
		&fb_data->cur_update->update_desc->upd_marker_list, upd_list)
		next_marker->lut_num = fb_data->cur_update->lut_num;

	/* Mark LUT as containing new update */
	fb_data->lut_update_order[fb_data->cur_update->lut_num] =
		fb_data->cur_update->update_desc->update_order;

	/* Enable Collision and WB complete IRQs */
	epdc_working_buf_intr(true);
	epdc_lut_complete_intr(fb_data->cur_update->lut_num, true);

	/* Program EPDC update to process buffer */
	next_upd_region =
		&fb_data->cur_update->update_desc->upd_data.update_region;

	if (fb_data->cur_update->update_desc->upd_data.temp == TEMP_USE_AUTO) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
			papyrus_temp);
    epdc_set_temp(temp_index);
	} else if (fb_data->cur_update->update_desc->upd_data.temp
		!= TEMP_USE_AMBIENT) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
			fb_data->cur_update->update_desc->upd_data.temp);
		epdc_set_temp(temp_index);
	} else
		epdc_set_temp(fb_data->temp_index);
	epdc_set_update_addr(fb_data->cur_update->phys_addr +
				fb_data->cur_update->update_desc->epdc_offs);
	epdc_set_update_coord(next_upd_region->left, next_upd_region->top);
	epdc_set_update_dimensions(next_upd_region->width,
				   next_upd_region->height);

	dev_dbg(fb_data->dev, "IRQ Submitting update:\n\
			\tUpdate region: [%d,%d,%d,%d]\n \
			\tWaveform : %d\n \
			\tUpdate mode : %d\n \
			\tTemperature : %d\n",
			fb_data->cur_update->update_desc->upd_data.update_region.top,
			fb_data->cur_update->update_desc->upd_data.update_region.left,
			fb_data->cur_update->update_desc->upd_data.update_region.left +
			fb_data->cur_update->update_desc->upd_data.update_region.width,
			fb_data->cur_update->update_desc->upd_data.update_region.top +
			fb_data->cur_update->update_desc->upd_data.update_region.height,
			fb_data->cur_update->update_desc->upd_data.waveform_mode,
			fb_data->cur_update->update_desc->upd_data.update_mode,
			fb_data->cur_update->update_desc->upd_data.temp);
	epdc_submit_update(fb_data->cur_update->lut_num,
			   fb_data->cur_update->update_desc->upd_data.waveform_mode,
			   fb_data->cur_update->update_desc->upd_data.update_mode,
			   false, 0);

	/* Release buffer queues */
	mutex_unlock(&fb_data->queue_mutex);

	return;
}

static void draw_mode0(struct mxc_epdc_fb_data *fb_data)
{
	u32 *upd_buf_ptr;
	int i;
	struct fb_var_screeninfo *screeninfo = &fb_data->epdc_fb_var;
	u32 xres, yres;
	int temp_index;
  
	upd_buf_ptr = (u32 *)fb_data->info.screen_base;

	epdc_working_buf_intr(true);
	epdc_lut_complete_intr(0, true);

	/* Use unrotated (native) width/height */
	if ((screeninfo->rotate == FB_ROTATE_CW) ||
		(screeninfo->rotate == FB_ROTATE_CCW)) {
		xres = screeninfo->yres;
		yres = screeninfo->xres;
	} else {
		xres = screeninfo->xres;
		yres = screeninfo->yres;
	}

	/* Program EPDC update to process buffer */
	epdc_set_update_addr(fb_data->phys_start);
	epdc_set_update_coord(0, 0);
	epdc_set_update_dimensions(fb_data->info.var.xres,
			fb_data->info.var.yres);
	temp_index = mxc_epdc_fb_get_temp_index(fb_data,papyrus_temp);
	epdc_set_temp(temp_index);
	epdc_submit_update(0, fb_data->wv_modes.mode_init, UPDATE_MODE_FULL, true, 0x00);

	dev_dbg(fb_data->dev, "Mode0 update - Waiting for LUT to complete...\n");

	/* Will timeout after ~4-5 seconds */

	for (i = 0; i < 40; i++) {
		if (!epdc_is_lut_active(0)) {
			dev_dbg(fb_data->dev, "Mode0 init complete\n");
			return;
		}
		msleep(100);
	}

	dev_err(fb_data->dev, "Mode0 init failed!\n");

	return;
}

static int mxc_epdc_fb_init_hw(struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	struct mxcfb_update_data update;
	struct mxcfb_waveform_data_file *wv_file;
	int wv_data_offs;
	int wv_file_size; /* Lab126 */
	int ret;
	int i;

	if (fb_data->hw_ready) {
	    dev_dbg(fb_data->dev, "hw already inited\n");
	    return 0;
	}

	/*
	 * Lab126: Use the passed-in waveform file
	 */
	if (!fb_data->wv_file) {
	    printk(KERN_ERR "%s: No waveform file specified!\n", __FUNCTION__);
	    return -1;
	}

	wv_file = fb_data->wv_file;
	wv_file_size = fb_data->wv_file_size;
	
	printk("%s: %s\n", __FUNCTION__, fb_data->wv_file_name);

	/* Get size and allocate temperature range table */
	fb_data->trt_entries = wv_file->wdh.trc + 1;
	fb_data->temp_range_bounds = kzalloc(fb_data->trt_entries, GFP_KERNEL);

	for (i = 0; i < fb_data->trt_entries; i++)
		dev_dbg(fb_data->dev, "trt entry #%d = 0x%x\n", i, *((u8 *)&wv_file->data + i));

	/* Copy TRT data */
	memcpy(fb_data->temp_range_bounds, &wv_file->data, fb_data->trt_entries);

	/* Set default temperature index using TRT and room temp */
	fb_data->temp_index = mxc_epdc_fb_get_temp_index(fb_data, DEFAULT_TEMP);

	/* Get offset and size for waveform data */
	wv_data_offs = sizeof(wv_file->wdh) + fb_data->trt_entries + 1;
	fb_data->waveform_buffer_size = wv_file_size - wv_data_offs; /* Lab126 */

	/* Allocate memory for waveform data */
	fb_data->waveform_buffer_virt = dma_alloc_coherent(fb_data->dev,
						fb_data->waveform_buffer_size,
						&fb_data->waveform_buffer_phys,
						GFP_DMA);
	if (fb_data->waveform_buffer_virt == NULL) {
		dev_err(fb_data->dev, "Can't allocate mem for waveform!\n");
		ret = -ENOMEM;
	}
	
	memcpy(fb_data->waveform_buffer_virt, (u8 *)(wv_file) + wv_data_offs,
		fb_data->waveform_buffer_size); /* Lab126 */

	fb_data->waveform_type |= WAVEFORM_TYPE_4BIT;

	/* Enable clocks to access EPDC regs */
	clk_enable(fb_data->epdc_clk_axi);

	/* Enable pix clk for EPDC */
	clk_enable(fb_data->epdc_clk_pix);
	clk_set_rate(fb_data->epdc_clk_pix, fb_data->cur_mode->vmode->pixclock);

	epdc_init_sequence(fb_data);

	/* Disable clocks */
	clk_disable(fb_data->epdc_clk_axi);
	clk_disable(fb_data->epdc_clk_pix);

	fb_data->hw_ready = true;

	update.update_region.left = 0;
	update.update_region.width = info->var.xres;
	update.update_region.top = 0;
	update.update_region.height = info->var.yres;
	update.update_mode = UPDATE_MODE_FULL;
	update.waveform_mode = WAVEFORM_MODE_AUTO;
	update.update_marker = INIT_UPDATE_MARKER;
	update.hist_bw_waveform_mode = 0;
	update.hist_gray_waveform_mode = 0;

	//	update.temp = TEMP_USE_AMBIENT;
	update.temp = TEMP_USE_AUTO;
	update.flags = 0;

	mxc_epdc_fb_send_update(&update, info);

	/* Block on initial update */
	ret = mxc_epdc_fb_wait_update_complete(update.update_marker, info);
	if (ret < 0)
		dev_err(fb_data->dev,
			"Wait for update complete failed.  Error = 0x%x", ret);

	return 0;
}

static ssize_t store_update(struct device *device,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct mxcfb_update_data update;
	struct fb_info *info = dev_get_drvdata(device);
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	if (strncmp(buf, "au", 2) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_a2;
	else if (strncmp(buf, "du", 2) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_du;
	else if (strncmp(buf, "gc16", 4) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc16;
	else if (strncmp(buf, "gc16f", 5) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc16_fast;
	else if (strncmp(buf, "gc4", 3) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc4;
	else if (strncmp(buf, "gl16", 4) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gl16;
	else if (strncmp(buf, "gl16f", 5) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gl16_fast;
	else if (strncmp(buf, "auto", 4) == 0)
		update.waveform_mode = WAVEFORM_MODE_AUTO;
	else
		update.waveform_mode = WAVEFORM_MODE_AUTO;

	/* Now, request full screen update */
	update.update_region.left = 0;
	update.update_region.width = fb_data->epdc_fb_var.xres;
	update.update_region.top = 0;
	update.update_region.height = fb_data->epdc_fb_var.yres;
	update.update_mode = UPDATE_MODE_FULL;
//	update.temp = TEMP_USE_AMBIENT;
	update.hist_bw_waveform_mode = 0; 
	update.hist_gray_waveform_mode = 0;
	update.temp = TEMP_USE_AUTO;
	update.update_marker = 0;
	update.flags = 0;

	mxc_epdc_fb_send_update(&update, info);

	return count;
}

static struct device_attribute fb_attrs[] = {
	__ATTR(update, S_IRUGO|S_IWUSR, NULL, store_update),
};

static ssize_t mxc_epdc_pwrdown_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	return sprintf(buf, "%d\n", fb_data->pwrdown_delay);
}

static ssize_t mxc_epdc_pwrdown_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int value = 0;
	struct fb_info *info = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &value) <= 0) {
		printk(KERN_ERR "Error in epdc power state value\n");
		return -EINVAL;
	}

	mxc_epdc_fb_set_pwrdown_delay(value, info);

	return size;
}
static DEVICE_ATTR(mxc_epdc_pwrdown, 0666, mxc_epdc_pwrdown_show, mxc_epdc_pwrdown_store);

extern void gpio_epdc_pins_enable(int enable);

static ssize_t mxc_epdc_force_powerup_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	gpio_epdc_pins_enable(1);
	epdc_force_powerup();
	return size;
}
static DEVICE_ATTR(mxc_epdc_force_powerup, 0666, NULL, mxc_epdc_force_powerup_store);

static ssize_t mxc_epdc_powerup_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	return sprintf(buf, "%d\n", fb_data->power_state);
}

static ssize_t mxc_epdc_powerup_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int value = 0;
	struct fb_info *info = dev_get_drvdata(dev);
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	if (sscanf(buf, "%d", &value) <= 0) {
		printk(KERN_ERR "Error in epdc power state value\n");
		return -EINVAL;
	}

	if (value == 0) {
		mutex_lock(&fb_data->power_mutex);
		fb_data->powering_down = true;
		epdc_powerdown(fb_data);
		mutex_unlock(&fb_data->power_mutex);
	}
	else {
		mutex_lock(&fb_data->power_mutex);
		
		if (!epdc_powerup(fb_data) || !epdc_powerup_wait_for_enabled(fb_data))
		{
			mutex_unlock(&fb_data->power_mutex);
			return size;
		}
		epdc_powerup_vcom(fb_data);
		mutex_unlock(&fb_data->power_mutex);
	}

	return size;
}
static DEVICE_ATTR(mxc_epdc_powerup, 0666, mxc_epdc_powerup_show, mxc_epdc_powerup_store);

static ssize_t mxc_epdc_update_store(struct device *device,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mxcfb_update_data update;
	struct fb_info *info = dev_get_drvdata(device);
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	
	char wf_buf[64];
	int top,left,width,height,mode;
	int params = 0;

	if ((params = sscanf(buf, "%10s %d %d %d %d %d", wf_buf, &mode, &top, &left, &width, 
					&height)) <= 0)
		return -EINVAL;

	if (strncmp(wf_buf, "auto", 4) == 0)
		update.waveform_mode = WAVEFORM_MODE_AUTO;
	else if (strncmp(wf_buf, "au", 2) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_a2;
	else if (strncmp(wf_buf, "du4", 3) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_du4;
	else if (strncmp(wf_buf, "du", 2) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_du;
	else if (strncmp(wf_buf, "gc16f", 5) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc16_fast;
	else if (strncmp(wf_buf, "gc16", 4) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc16;
	else if (strncmp(wf_buf, "gc4", 3) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc4;
	else if (strncmp(wf_buf, "gl16f", 5) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gl16_fast;
	else if (strncmp(wf_buf, "gl16", 4) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gl16;
	else 
		update.waveform_mode = WAVEFORM_MODE_AUTO;

	update.update_mode = (params >= 2)? mode : UPDATE_MODE_FULL;
	update.update_region.top = (params >= 3) ? top : 0;
	update.update_region.left = (params >= 4) ? left : 0;
	update.update_region.width = (params >= 5) ? width : (info->var.xres - update.update_region.left);
	update.update_region.height = (params >= 6) ? height : (info->var.yres - update.update_region.top);
	update.hist_bw_waveform_mode = 0;
	update.hist_gray_waveform_mode = 0;
	update.temp = papyrus_temp;
	update.update_marker = 1;
	update.flags = 0;

	mxc_epdc_fb_send_update(&update, info);

	return count;
}

static ssize_t mxc_epdc_update_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "1\n");
}
static DEVICE_ATTR(mxc_epdc_update, 0666, mxc_epdc_update_show, mxc_epdc_update_store);

static ssize_t mxc_epdc_debug_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", mxc_epdc_debugging);
}

static ssize_t mxc_epdc_debug_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t size)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		printk(KERN_ERR "Error in epdc debug value\n");
		return -EINVAL;
	}

	mxc_epdc_debugging = value;
	return size;
}

static DEVICE_ATTR(mxc_epdc_debug, 0666, mxc_epdc_debug_show, mxc_epdc_debug_store);


#include "mxc_epdc_fb_lab126.c"

int __devinit mxc_epdc_fb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mxc_epdc_fb_data *fb_data;
	struct resource *res;
	struct fb_info *info;
	char *options, *opt;
	char *panel_str = NULL;
	char name[] = "mxcepdcfb";
	struct fb_videomode *vmode;
	int xres_virt, yres_virt, buf_size;
	int xres_virt_rot, yres_virt_rot, pix_size_rot;
	struct fb_var_screeninfo *var_info;
	struct fb_fix_screeninfo *fix_info;
	struct pxp_config_data *pxp_conf;
	struct pxp_proc_data *proc_data;
	struct scatterlist *sg;
	struct update_data_list *upd_list;
	struct update_data_list *plist, *temp_list;
	int i;
	unsigned long x_mem_size = 0;
#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	struct mxcfb_update_data update;
#endif
	
	fb_data = (struct mxc_epdc_fb_data *)framebuffer_alloc(
			sizeof(struct mxc_epdc_fb_data), &pdev->dev);
	if (fb_data == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	/* Get platform data and check validity */
	fb_data->pdata = pdev->dev.platform_data;
	if ((fb_data->pdata == NULL) || (fb_data->pdata->num_modes < 1)
		|| (fb_data->pdata->epdc_mode == NULL)
		|| (fb_data->pdata->epdc_mode->vmode == NULL)) {
		ret = -EINVAL;
		goto out_fbdata;
	}

	if (fb_get_options(name, &options)) {
		ret = -ENODEV;
		goto out_fbdata;
	}

	fb_data->tce_prevent = 0;

	if (options)
		while ((opt = strsep(&options, ",")) != NULL) {
			if (!*opt)
				continue;

			if (!strncmp(opt, "bpp=", 4))
				fb_data->default_bpp =
					simple_strtoul(opt + 4, NULL, 0);
			else if (!strncmp(opt, "x_mem=", 6))
				x_mem_size = memparse(opt + 6, NULL);
			else if (!strncmp(opt, "tce_prevent", 11))
				fb_data->tce_prevent = 1;
			else
				panel_str = opt;
		}

	fb_data->dev = &pdev->dev;

	if (!fb_data->default_bpp)
		fb_data->default_bpp = 16;

	/* Create early so that it exists even after probe failure */
	if (device_create_file(&pdev->dev, &dev_attr_mxc_epdc_force_powerup) < 0)
		dev_err(&pdev->dev, "Unable to create mxc_epdc_force_pwrdown file\n");

	/* Set default (first defined mode) before searching for a match */
	fb_data->cur_mode = &fb_data->pdata->epdc_mode[0];

	if (panel_str)
		for (i = 0; i < fb_data->pdata->num_modes; i++)
			if (!strcmp(fb_data->pdata->epdc_mode[i].vmode->name,
						panel_str)) {
				fb_data->cur_mode =
					&fb_data->pdata->epdc_mode[i];
				break;
			}

	vmode = fb_data->cur_mode->vmode;

	platform_set_drvdata(pdev, fb_data);
	info = &fb_data->info;

	/* Allocate color map for the FB */
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret)
		goto out_fbdata;

	dev_dbg(&pdev->dev, "resolution %dx%d, bpp %d\n",
		vmode->xres, vmode->yres, fb_data->default_bpp);

	/*
	 * GPU alignment restrictions dictate framebuffer parameters:
	 * - 32-byte alignment for buffer width
	 * - 128-byte alignment for buffer height
	 * => 4K buffer alignment for buffer start
	 */
	xres_virt = ALIGN(vmode->xres, 32);
	yres_virt = ALIGN(vmode->yres, 128);
	fb_data->max_pix_size = PAGE_ALIGN(xres_virt * yres_virt);
	
	xres_virt_rot = ALIGN(vmode->yres, 32);
	yres_virt_rot = ALIGN(vmode->xres, 128);
	pix_size_rot = PAGE_ALIGN(xres_virt_rot * yres_virt_rot);
	
	fb_data->max_pix_size = (fb_data->max_pix_size > pix_size_rot) ? 
		fb_data->max_pix_size : pix_size_rot;
	
	buf_size = fb_data->max_pix_size * fb_data->default_bpp/8;

	/* Compute the number of screens needed based on X memory requested */
	if (x_mem_size > 0) {
		fb_data->num_screens = DIV_ROUND_UP(x_mem_size, buf_size);
		if (fb_data->num_screens < NUM_SCREENS_MIN)
			fb_data->num_screens = NUM_SCREENS_MIN;
		else if (buf_size * fb_data->num_screens > SZ_16M)
			fb_data->num_screens = SZ_16M / buf_size;
	} else
		fb_data->num_screens = NUM_SCREENS_MIN;

	fb_data->map_size = buf_size * fb_data->num_screens;
	dev_dbg(&pdev->dev, "memory to allocate: %d\n", fb_data->map_size);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENODEV;
		goto out_cmap;
	}

	epdc_base = ioremap(res->start, SZ_4K);
	if (epdc_base == NULL) {
		ret = -ENOMEM;
		goto out_cmap;
	}


	/* Allocate FB memory */
	info->screen_base = dma_alloc_writecombine(&pdev->dev,
						  fb_data->map_size,
						  &fb_data->phys_start,
						  GFP_DMA);

	if (info->screen_base == NULL) {
		ret = -ENOMEM;
		goto out_mapregs;
	}
	dev_dbg(&pdev->dev, "allocated at %p:0x%x\n", info->screen_base,
		fb_data->phys_start);

	var_info = &info->var;
	var_info->activate = FB_ACTIVATE_TEST;
	var_info->bits_per_pixel = fb_data->default_bpp;
	var_info->xres = vmode->xres;
	var_info->yres = vmode->yres;
	var_info->xres_virtual = xres_virt;
	/* Additional screens allow for panning  and buffer flipping */
	var_info->yres_virtual = yres_virt * fb_data->num_screens;

	var_info->pixclock = vmode->pixclock;
	var_info->left_margin = vmode->left_margin;
	var_info->right_margin = vmode->right_margin;
	var_info->upper_margin = vmode->upper_margin;
	var_info->lower_margin = vmode->lower_margin;
	var_info->hsync_len = vmode->hsync_len;
	var_info->vsync_len = vmode->vsync_len;
	var_info->vmode = FB_VMODE_NONINTERLACED;

	switch (fb_data->default_bpp) {
	case 32:
	case 24:
		var_info->red.offset = 16;
		var_info->red.length = 8;
		var_info->green.offset = 8;
		var_info->green.length = 8;
		var_info->blue.offset = 0;
		var_info->blue.length = 8;
		break;

	case 16:
		var_info->red.offset = 11;
		var_info->red.length = 5;
		var_info->green.offset = 5;
		var_info->green.length = 6;
		var_info->blue.offset = 0;
		var_info->blue.length = 5;
		break;

	case 8:
		/*
		 * For 8-bit grayscale, R, G, and B offset are equal.
		 *
		 */
		var_info->grayscale = GRAYSCALE_8BIT_INVERTED; /* Lab126 */

		var_info->red.length = 8;
		var_info->red.offset = 0;
		var_info->red.msb_right = 0;
		var_info->green.length = 8;
		var_info->green.offset = 0;
		var_info->green.msb_right = 0;
		var_info->blue.length = 8;
		var_info->blue.offset = 0;
		var_info->blue.msb_right = 0;
		break;

	default:
		dev_err(&pdev->dev, "unsupported bitwidth %d\n",
			fb_data->default_bpp);
		ret = -EINVAL;
		goto out_dma_fb;
	}

	fix_info = &info->fix;

	strcpy(fix_info->id, "mxc_epdc_fb");
	fix_info->type = FB_TYPE_PACKED_PIXELS;
	fix_info->visual = FB_VISUAL_TRUECOLOR;
	fix_info->xpanstep = 0;
	fix_info->ypanstep = 0;
	fix_info->ywrapstep = 0;
	fix_info->accel = FB_ACCEL_NONE;
	fix_info->smem_start = fb_data->phys_start;
	fix_info->smem_len = fb_data->map_size;
	fix_info->ypanstep = 0;

	fb_data->native_width = vmode->xres;
	fb_data->native_height = vmode->yres;

	info->fbops = &mxc_epdc_fb_ops;
	info->var.activate = FB_ACTIVATE_NOW;
	info->pseudo_palette = fb_data->pseudo_palette;
	info->screen_size = info->fix.smem_len;
	fb_data->par = NULL;
	info->flags = FBINFO_FLAG_DEFAULT;

	mxc_epdc_fb_set_fix(info);

	fb_data->auto_mode = AUTO_UPDATE_MODE_REGION_MODE;
	fb_data->upd_scheme = UPDATE_SCHEME_QUEUE_AND_MERGE; 
  
	init_waitqueue_head(&fb_data->vsync_wait_q);
	fb_data->vsync_count = 0;

	fb_data->epdc_fb_var = *var_info;
	fb_data->fb_offset = 0;
	fb_data->eof_sync_period = 0;

	fb_data->max_num_buffers = EPDC_MAX_NUM_BUFFERS;
	/*
	 * Initialize lists for pending updates,
	 * active update requests, update collisions,
	 * and available update (PxP output) buffers
	 */
	INIT_LIST_HEAD(&fb_data->upd_pending_list);
	INIT_LIST_HEAD(&fb_data->upd_buf_queue);
	INIT_LIST_HEAD(&fb_data->upd_buf_free_list);
	INIT_LIST_HEAD(&fb_data->upd_buf_collision_list);

	/* Allocate update buffers and add them to the list */
	for (i = 0; i < EPDC_MAX_NUM_UPDATES; i++) {
		upd_list = kzalloc(sizeof(*upd_list), GFP_KERNEL);
		if (upd_list == NULL) {
			ret = -ENOMEM;
			goto out_upd_lists;
		}
		/* Add newly allocated buffer to free list */
		list_add(&upd_list->list, &fb_data->upd_buf_free_list);
	}

	fb_data->virt_addr_updbuf =
		kzalloc(sizeof(void *) * fb_data->max_num_buffers, GFP_KERNEL);
	fb_data->phys_addr_updbuf =
		kzalloc(sizeof(dma_addr_t) * fb_data->max_num_buffers,
				GFP_KERNEL);

	for (i = 0; i < fb_data->max_num_buffers; i++) {

		/*
		 * Allocate memory for PxP output buffer.
		 * Each update buffer is 1 byte per pixel, and can
		 * be as big as the full-screen frame buffer
		 */

		fb_data->virt_addr_updbuf[i] = 
		    dma_alloc_coherent(fb_data->info.device, fb_data->max_pix_size,
						&fb_data->phys_addr_updbuf[i], GFP_DMA);
		if (fb_data->virt_addr_updbuf[i] == NULL) {
			ret = -ENOMEM;
			goto out_upd_buffers;
		}

		dev_dbg(fb_data->info.device, "allocated %d bytes @ 0x%08X\n",
				fb_data->max_pix_size, fb_data->phys_addr_updbuf[i]);
	}

	/* Counter indicating which update buffer should be used next */
	fb_data->upd_buffer_num = 0;
	
	/*
	 * Allocate memory for PxP SW workaround buffer
	 * These buffers are used to hold copy of the update region,
	 * before sending it to PxP for processing.
	 */
	fb_data->virt_addr_copybuf =
	    dma_alloc_coherent(fb_data->info.device, fb_data->max_pix_size,
			       &fb_data->phys_addr_copybuf, GFP_DMA);
	if (fb_data->virt_addr_copybuf == NULL) {
		ret = -ENOMEM;
		goto out_upd_buffers;
 	}

	fb_data->working_buffer_size = vmode->yres * vmode->xres * 2;
	/* Allocate memory for EPDC working buffer */
	fb_data->working_buffer_virt =
	    dma_alloc_coherent(&pdev->dev, fb_data->working_buffer_size,
			       &fb_data->working_buffer_phys, GFP_DMA);
	if (fb_data->working_buffer_virt == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for working buf!\n");
		ret = -ENOMEM;
		goto out_copybuffer;
	}

	/* Initialize EPDC pins */
	if (fb_data->pdata->get_pins)
		fb_data->pdata->get_pins();

	fb_data->epdc_clk_axi = clk_get(fb_data->dev, "epdc_axi");
	if (IS_ERR(fb_data->epdc_clk_axi)) {
		dev_err(&pdev->dev, "Unable to get EPDC AXI clk."
			"err = 0x%x\n", (int)fb_data->epdc_clk_axi);
		ret = -ENODEV;
		goto out_copybuffer;
	}
	fb_data->epdc_clk_pix = clk_get(fb_data->dev, "epdc_pix");
	if (IS_ERR(fb_data->epdc_clk_pix)) {
		dev_err(&pdev->dev, "Unable to get EPDC pix clk."
			"err = 0x%x\n", (int)fb_data->epdc_clk_pix);
		ret = -ENODEV;
		goto out_copybuffer;
	}

	fb_data->in_init = false;

	fb_data->hw_ready = false;

	/*
	 * Set default waveform mode values.
	 * Should be overwritten via ioctl.
	 */
	fb_data->wv_modes.mode_init = WAVEFORM_MODE_INIT;
	fb_data->wv_modes.mode_du   = WAVEFORM_MODE_DU;
	fb_data->wv_modes.mode_gc4  = WAVEFORM_MODE_GC4;
	fb_data->wv_modes.mode_gc8  = WAVEFORM_MODE_GC16;
	fb_data->wv_modes.mode_gc16 = WAVEFORM_MODE_GC16;
	fb_data->wv_modes.mode_gc32 = WAVEFORM_MODE_GC16;
	fb_data->wv_modes.mode_gl16 = WAVEFORM_MODE_GL16;
	fb_data->wv_modes.mode_a2   = WAVEFORM_MODE_A2;
	fb_data->wv_modes.mode_gc16_fast = WAVEFORM_MODE_GC16_FAST;
	fb_data->wv_modes.mode_gl16_fast = WAVEFORM_MODE_GL16_FAST;
	fb_data->wv_modes.mode_du4 = WAVEFORM_MODE_DU4;
  
	/* Initialize marker list */
	INIT_LIST_HEAD(&fb_data->full_marker_list);

	/* Initialize all LUTs to inactive */
	for (i = 0; i < EPDC_NUM_LUTS; i++)
		fb_data->lut_update_order[i] = 0;

	INIT_DELAYED_WORK(&fb_data->epdc_done_work, epdc_done_work_func);
	fb_data->epdc_submit_workqueue =
		__create_workqueue(("EPDC Submit"), 1, 0, 1);
	INIT_WORK(&fb_data->epdc_submit_work, epdc_submit_work_func);
	fb_data->epdc_intr_workqueue =
		__create_workqueue(("EPDC Interrupt"), 1, 0, 1);
	INIT_WORK(&fb_data->epdc_intr_work, epdc_intr_work_func);


	/* Retrieve EPDC IRQ num */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot get IRQ resource\n");
		ret = -ENODEV;
		goto out_dma_work_buf;
	}
	fb_data->epdc_irq = res->start;

	/* Register IRQ handler */
	if (mx50_board_is(BOARD_ID_TEQUILA))
		ret = request_irq(fb_data->epdc_irq, mxc_epdc_irq_handler,
				IRQF_NODELAY, "fb_dma", fb_data);
	else
		ret = request_irq(fb_data->epdc_irq, mxc_epdc_irq_handler,
				0, "fb_dma", fb_data);

	if (ret) {
		dev_err(&pdev->dev, "request_irq (%d) failed with error %d\n",
			fb_data->epdc_irq, ret);
		ret = -ENODEV;
		goto out_dma_work_buf;
	}

/*
	INIT_DELAYED_WORK(&fb_data->epdc_done_work, epdc_done_work_func);
	fb_data->epdc_submit_workqueue = create_rt_workqueue("submit");
	INIT_WORK(&fb_data->epdc_submit_work, epdc_submit_work_func);
*/

	info->fbdefio = &mxc_epdc_fb_defio;

	/* get pmic regulators */
	fb_data->display_regulator = regulator_get(NULL, "DISPLAY");
	if (IS_ERR(fb_data->display_regulator)) {
		dev_err(&pdev->dev, "Unable to get display PMIC regulator."
			"err = 0x%x\n", (int)fb_data->display_regulator);
		ret = -ENODEV;
		goto out_irq;
	}
	fb_data->vcom_regulator = regulator_get(NULL, "VCOM");
	if (IS_ERR(fb_data->vcom_regulator)) {
		regulator_put(fb_data->display_regulator);
		dev_err(&pdev->dev, "Unable to get VCOM regulator."
			"err = 0x%x\n", (int)fb_data->vcom_regulator);
		ret = -ENODEV;
		goto out_irq;
	}

	if (device_create_file(info->dev, &fb_attrs[0]))
		dev_err(&pdev->dev, "Unable to create file from fb_attrs\n");

	/* Lab126 */
	if (device_create_file(&pdev->dev, &dev_attr_mxc_epdc_powerup) < 0)
		dev_err(&pdev->dev, "Unable to create mxc_epdc_powerup file\n");

	if (device_create_file(&pdev->dev, &dev_attr_mxc_epdc_debug) < 0)
		dev_err(&pdev->dev, "Unable to create mxc_epdc_debug file\n");

	if (device_create_file(&pdev->dev, &dev_attr_mxc_epdc_update) < 0)
		dev_err(&pdev->dev, "Unable to create mxc_epdc_update file\n");

	if (device_create_file(&pdev->dev, &dev_attr_mxc_epdc_pwrdown) < 0)
		dev_err(&pdev->dev, "Unable to create  mxc_epdc_pwrdown file\n");

	fb_data->cur_update = NULL;

	mutex_init(&fb_data->queue_mutex);

	mutex_init(&fb_data->pxp_mutex);

	mutex_init(&fb_data->power_mutex);

	/* PxP DMA interface */
	dmaengine_get();

	/*
	 * Fill out PxP config data structure based on FB info and
	 * processing tasks required
	 */
	pxp_conf = &fb_data->pxp_conf;
	proc_data = &pxp_conf->proc_data;

	/* Initialize non-channel-specific PxP parameters */
	proc_data->drect.left = proc_data->srect.left = 0;
	proc_data->drect.top = proc_data->srect.top = 0;
	proc_data->drect.width = proc_data->srect.width = fb_data->info.var.xres;
	proc_data->drect.height = proc_data->srect.height = fb_data->info.var.yres;
	proc_data->scaling = 0;
	proc_data->hflip = 0;
	proc_data->vflip = 0;
	proc_data->rotate = 0;
	proc_data->bgcolor = 0;
	proc_data->overlay_state = 0;
	proc_data->lut_transform = PXP_LUT_NONE;
	proc_data->lut_map = NULL;

	/*
	 * We initially configure PxP for RGB->YUV conversion,
	 * and only write out Y component of the result.
	 */

	/*
	 * Initialize S0 channel parameters
	 * Parameters should match FB format/width/height
	 */
	pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
	pxp_conf->s0_param.width = fb_data->info.var.xres_virtual;
	pxp_conf->s0_param.height = fb_data->info.var.yres;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;

	/*
	 * Initialize OL0 channel parameters
	 * No overlay will be used for PxP operation
	 */
	for (i = 0; i < 8; i++) {
		pxp_conf->ol_param[i].combine_enable = false;
		pxp_conf->ol_param[i].width = 0;
		pxp_conf->ol_param[i].height = 0;
		pxp_conf->ol_param[i].pixel_fmt = PXP_PIX_FMT_RGB565;
		pxp_conf->ol_param[i].color_key_enable = false;
		pxp_conf->ol_param[i].color_key = -1;
		pxp_conf->ol_param[i].global_alpha_enable = false;
		pxp_conf->ol_param[i].global_alpha = 0;
		pxp_conf->ol_param[i].local_alpha_enable = false;
	}

	/*
	 * Initialize Output channel parameters
	 * Output is Y-only greyscale
	 * Output width/height will vary based on update region size
	 */
	pxp_conf->out_param.width = fb_data->info.var.xres;
	pxp_conf->out_param.height = fb_data->info.var.yres;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_GREY;

	/* Initialize color map for conversion of 8-bit gray pixels */
	fb_data->pxp_conf.proc_data.lut_map = kmalloc(256, GFP_KERNEL);
	if (fb_data->pxp_conf.proc_data.lut_map == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for lut map!\n");
		ret = -ENOMEM;
		goto out_dmaengine;
	}

	if(use_builtin_cmap == 1) {
		dev_dbg(&pdev->dev, "use_builtin_cmap !!\n");
		mxc_epdc_fb_setcmap(&einkfb_8bpp_cmap, &fb_data->info);
		fb_copy_cmap(&einkfb_8bpp_cmap, &info->cmap);
	} else {
		unsigned short channel_map[256];
		struct fb_cmap linear_8bpp_cmap = {
			.len    = 256,
			.start  = 0,
			.red    = channel_map,
			.green  = channel_map,
			.blue   = channel_map,
			.transp = NULL
		};
		
		dev_dbg(&pdev->dev, "not use_builtin_cmap !!\n");	
		
		for (i = 0; i < 256; i++)
			channel_map[i] = (i << 8 | i);
		
		mxc_epdc_fb_setcmap(&linear_8bpp_cmap, &fb_data->info);
		fb_copy_cmap(&linear_8bpp_cmap, &info->cmap);
	}
	fb_data->pxp_conf.proc_data.lut_map_updated = true;
	/*
	 * Ensure this is set to NULL here...we will initialize pxp_chan
	 * later in our thread.
	 */
	fb_data->pxp_chan = NULL;

	/* Initialize Scatter-gather list containing 2 buffer addresses. */
	sg = fb_data->sg;
	sg_init_table(sg, 2);

	/*
	 * For use in PxP transfers:
	 * sg[0] holds the FB buffer pointer
	 * sg[1] holds the Output buffer pointer (configured before TX request)
	 */
	sg_dma_address(&sg[0]) = info->fix.smem_start;
	sg_set_page(&sg[0], virt_to_page(info->screen_base),
		    info->fix.smem_len, offset_in_page(info->screen_base));

	fb_data->order_cnt = 0;
	fb_data->waiting_for_wb = false;
	fb_data->waiting_for_lut = false;
	fb_data->waiting_for_lut15 = false;
	fb_data->waiting_for_idle = false;
	fb_data->blank = FB_BLANK_UNBLANK;
	fb_data->power_state = POWER_STATE_OFF;
	fb_data->powering_down = false;
	fb_data->wait_for_powerdown = false;
	fb_data->updates_active = false;
	fb_data->pwrdown_delay = 0;

	/* Lab126  and Tequila only */
	if (dont_register_fb) {
		goto dont_register;
	}
	/* Register FB */
	ret = register_framebuffer(info);
	if (ret) {
		dev_err(&pdev->dev,
			"register_framebuffer failed with error %d\n", ret);
		goto out_dmaengine;
	}
	
	dev_dbg(fb_data->dev, "Before waveform init:\n\
			\tBefore :\n\
			\t\tDU : %d\n\
			\t\tGC : %d\n\
			\t\tGC_FAST : %d\n\
			\t\tGL : %d\n\
			\t\tGL_FAST : %d\n\
			\t\tA2 : %d\n\
			\t\tDU4 : %d\n",
			fb_data->wv_modes.mode_du,
			fb_data->wv_modes.mode_gc16,
			fb_data->wv_modes.mode_gc16_fast,
			fb_data->wv_modes.mode_gl16,
			fb_data->wv_modes.mode_gl16_fast,
			fb_data->wv_modes.mode_a2,
			fb_data->wv_modes.mode_du4
			);
	
	mxc_epdc_waveform_init(fb_data);
	
	dev_dbg(fb_data->dev, "After waveform init:\n\
			\tBefore :\n\
			\t\tDU : %d\n\
			\t\tGC : %d\n\
			\t\tGC_FAST : %d\n\
			\t\tGL : %d\n\
			\t\tGL_FAST : %d\n\
			\t\tA2 : %d\n\
			\t\tDU4: %d\n",
			fb_data->wv_modes.mode_du,
			fb_data->wv_modes.mode_gc16,
			fb_data->wv_modes.mode_gc16_fast,
			fb_data->wv_modes.mode_gl16,
			fb_data->wv_modes.mode_gl16_fast,
			fb_data->wv_modes.mode_a2,
			fb_data->wv_modes.mode_du4
			);
  
dont_register: /* Lab126 */
	g_fb_data = fb_data;

	if (default_panel_hw_init && !fb_data->hw_ready)
	{
		ret = mxc_epdc_fb_init_hw((struct fb_info *)fb_data);
		if (ret) {
			dev_err(&pdev->dev, "Failed to initialize HW!\n");
		}
	}

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	/* If FB console included, update display to show logo */
	update.update_region.left = 0;
	update.update_region.width = info->var.xres;
	update.update_region.top = 0;
	update.update_region.height = info->var.yres;
	update.update_mode = UPDATE_MODE_PARTIAL;
	update.waveform_mode = WAVEFORM_MODE_AUTO;
	update.update_marker = INIT_UPDATE_MARKER;
	update.hist_bw_waveform_mode = 0;
	update.hist_gray_waveform_mode = 0;
	//	update.temp = TEMP_USE_AMBIENT;
	update.temp = TEMP_USE_AUTO;
	update.flags = 0;

	mxc_epdc_fb_send_update(&update, info);

	ret = mxc_epdc_fb_wait_update_complete(update.update_marker, info);
	if (ret < 0)
		dev_err(fb_data->dev,
			"Wait for update complete failed.  Error = 0x%x", ret);
#endif
	goto out;

out_dmaengine:
	dmaengine_put();
out_irq:
	free_irq(fb_data->epdc_irq, fb_data);
out_dma_work_buf:
	dma_free_writecombine(&pdev->dev, fb_data->working_buffer_size,
		fb_data->working_buffer_virt, fb_data->working_buffer_phys);
	if (fb_data->pdata->put_pins)
		fb_data->pdata->put_pins();
out_copybuffer:
	dma_free_writecombine(&pdev->dev, fb_data->max_pix_size,
			      fb_data->virt_addr_copybuf,
			      fb_data->phys_addr_copybuf);
out_upd_buffers:
	for (i = 0; i < fb_data->max_num_buffers; i++)
		if (fb_data->virt_addr_updbuf[i] != NULL)
			dma_free_writecombine(&pdev->dev, fb_data->max_pix_size,
					fb_data->virt_addr_updbuf[i],
					fb_data->phys_addr_updbuf[i]);
	if (fb_data->virt_addr_updbuf != NULL)
		kfree(fb_data->virt_addr_updbuf);
	if (fb_data->phys_addr_updbuf != NULL)
		kfree(fb_data->phys_addr_updbuf);
out_upd_lists:
	list_for_each_entry_safe(plist, temp_list, &fb_data->upd_buf_free_list,
			list) {
		list_del(&plist->list);
		kfree(plist);
	}
out_dma_fb:
	dma_free_writecombine(&pdev->dev, fb_data->map_size, info->screen_base,
			      fb_data->phys_start);

out_mapregs:
out_cmap:
	fb_dealloc_cmap(&info->cmap);
out_fbdata:
	kfree(fb_data);
out:
	return ret;
}

static int mxc_epdc_fb_remove(struct platform_device *pdev)
{
	struct update_data_list *plist, *temp_list;
	struct mxc_epdc_fb_data *fb_data = platform_get_drvdata(pdev);
	struct fb_info *info = &fb_data->info; /* Lab126 */
	int i;

	cancel_rearming_delayed_work(&ff_work);
	
	mxc_epdc_fb_blank(FB_BLANK_POWERDOWN, &fb_data->info);

	cancel_rearming_delayed_work(&fb_data->epdc_done_work);

	/* Lab126 */
	device_remove_file(info->dev, &fb_attrs[0]);
	device_remove_file(&pdev->dev, &dev_attr_mxc_epdc_powerup);
	device_remove_file(&pdev->dev, &dev_attr_mxc_epdc_debug);
	device_remove_file(&pdev->dev, &dev_attr_mxc_epdc_update);
	device_remove_file(&pdev->dev, &dev_attr_mxc_epdc_pwrdown);
	device_remove_file(&pdev->dev, &dev_attr_mxc_epdc_force_powerup);
	
	regulator_put(fb_data->display_regulator);
	regulator_put(fb_data->vcom_regulator);

	/* Lab126 */
	if (!(dont_register_fb)) {
		unregister_framebuffer(&fb_data->info);
		mxc_epdc_waveform_done();
	}
	free_irq(fb_data->epdc_irq, fb_data);

	dma_free_writecombine(&pdev->dev, fb_data->working_buffer_size,
				fb_data->working_buffer_virt,
				fb_data->working_buffer_phys);
	if (fb_data->waveform_buffer_virt != NULL)
		dma_free_writecombine(&pdev->dev, fb_data->waveform_buffer_size,
				fb_data->waveform_buffer_virt,
				fb_data->waveform_buffer_phys);
	if (fb_data->virt_addr_copybuf != NULL)
		dma_free_writecombine(&pdev->dev, fb_data->max_pix_size,
				fb_data->virt_addr_copybuf,
				fb_data->phys_addr_copybuf);

	for (i = 0; i < fb_data->max_num_buffers; i++)
		if (fb_data->virt_addr_updbuf[i] != NULL)
			dma_free_writecombine(&pdev->dev, fb_data->max_pix_size,
					fb_data->virt_addr_updbuf[i],
					fb_data->phys_addr_updbuf[i]);
	if (fb_data->virt_addr_updbuf != NULL)
		kfree(fb_data->virt_addr_updbuf);
	if (fb_data->phys_addr_updbuf != NULL)
		kfree(fb_data->phys_addr_updbuf);

	
	list_for_each_entry_safe(plist, temp_list, &fb_data->upd_buf_free_list,
			list) {
		list_del(&plist->list);
		kfree(plist);
	}

	dma_free_writecombine(&pdev->dev, fb_data->map_size, fb_data->info.screen_base,
			      fb_data->phys_start);

	if (fb_data->pdata->put_pins)
		fb_data->pdata->put_pins();

	/* Release PxP-related resources */
	if (fb_data->pxp_chan != NULL)
		dma_release_channel(&fb_data->pxp_chan->dma_chan);

	dmaengine_put();

	if (fb_data->pxp_conf.proc_data.lut_map != NULL)
		kfree(fb_data->pxp_conf.proc_data.lut_map);

	iounmap(epdc_base);

	fb_dealloc_cmap(&fb_data->info.cmap);

	framebuffer_release(&fb_data->info);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static void mxc_epdc_fb_shutdown(struct platform_device *pdev)
{
	struct mxc_epdc_fb_data *data = platform_get_drvdata(pdev);

	mxc_epdc_fb_blank(FB_BLANK_POWERDOWN, &data->info);
}

#ifdef CONFIG_PM
static int mxc_epdc_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mxc_epdc_fb_data *data = platform_get_drvdata(pdev);
	int ret;

	ret = mxc_epdc_fb_blank(FB_BLANK_POWERDOWN, &data->info);
	if (ret)
		goto out;

out:
	return ret;
}

static int mxc_epdc_fb_resume(struct platform_device *pdev)
{
	struct mxc_epdc_fb_data *data = platform_get_drvdata(pdev);

	mxc_epdc_fb_blank(FB_BLANK_UNBLANK, &data->info);
	return 0;
}
#else
#define mxc_epdc_fb_suspend	NULL
#define mxc_epdc_fb_resume	NULL
#endif

static struct platform_driver mxc_epdc_fb_driver = {
	.probe = mxc_epdc_fb_probe,
	.remove = mxc_epdc_fb_remove,
	.suspend = mxc_epdc_fb_suspend,
	.resume = mxc_epdc_fb_resume,
	.shutdown = mxc_epdc_fb_shutdown,
	.driver = {
		   .name = "mxc_epdc_fb",
		   .owner = THIS_MODULE,
		   },
};

/* Callback function triggered after PxP receives an EOF interrupt */
static void pxp_dma_done(void *arg)
{
	struct pxp_tx_desc *tx_desc = to_tx_desc(arg);
	struct dma_chan *chan = tx_desc->txd.chan;
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);
	struct mxc_epdc_fb_data *fb_data = pxp_chan->client;

	/* This call will signal wait_for_completion_timeout() in send_buffer_to_pxp */
	complete(&fb_data->pxp_tx_cmpl);
}

/* Function to request PXP DMA channel */
static int pxp_chan_init(struct mxc_epdc_fb_data *fb_data)
{
	dma_cap_mask_t mask;
	struct dma_chan *chan;

	/*
	 * Request a free channel
	 */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_PRIVATE, mask);
	chan = dma_request_channel(mask, NULL, NULL);
	if (!chan) {
		dev_err(fb_data->dev, "Unsuccessfully received channel!!!!\n");
		return -EBUSY;
	}

	dev_dbg(fb_data->dev, "Successfully received channel.\n");

	fb_data->pxp_chan = to_pxp_channel(chan);

	fb_data->pxp_chan->client = fb_data;

	init_completion(&fb_data->pxp_tx_cmpl);

	return 0;
}

/*
 * Function to call PxP DMA driver and send our latest FB update region
 * through the PxP and out to an intermediate buffer.
 * Note: This is a blocking call, so upon return the PxP tx should be complete.
 */
static int pxp_process_update(struct mxc_epdc_fb_data *fb_data,
			      u32 src_width, u32 src_height,
			      struct mxcfb_rect *update_region)
{
	dma_cookie_t cookie;
	struct scatterlist *sg = fb_data->sg;
	struct dma_chan *dma_chan;
	struct pxp_tx_desc *desc;
	struct dma_async_tx_descriptor *txd;
	struct pxp_config_data *pxp_conf = &fb_data->pxp_conf;
	struct pxp_proc_data *proc_data = &fb_data->pxp_conf.proc_data;
	int i, ret;
	int length;

	dev_dbg(fb_data->dev, "Starting PxP Send Buffer\n");

	/* First, check to see that we have acquired a PxP Channel object */
	if (fb_data->pxp_chan == NULL) {
		/*
		 * PxP Channel has not yet been created and initialized,
		 * so let's go ahead and try
		 */
		ret = pxp_chan_init(fb_data);
		if (ret) {
			/*
			 * PxP channel init failed, and we can't use the
			 * PxP until the PxP DMA driver has loaded, so we abort
			 */
			dev_err(fb_data->dev, "PxP chan init failed\n");
			return -ENODEV;
		}
	}

	/*
	 * Init completion, so that we
	 * can be properly informed of the completion
	 * of the PxP task when it is done.
	 */
	init_completion(&fb_data->pxp_tx_cmpl);

	dev_dbg(fb_data->dev, "sg[0] = 0x%x, sg[1] = 0x%x\n",
		sg_dma_address(&sg[0]), sg_dma_address(&sg[1]));

	dma_chan = &fb_data->pxp_chan->dma_chan;

	txd = dma_chan->device->device_prep_slave_sg(dma_chan, sg, 2,
						     DMA_TO_DEVICE,
						     DMA_PREP_INTERRUPT);
	if (!txd) {
		dev_err(fb_data->info.device,
			"Error preparing a DMA transaction descriptor.\n");
		return -EIO;
	}

	txd->callback_param = txd;
	txd->callback = pxp_dma_done;

	/*
	 * Configure PxP for processing of new update region
	 * The rest of our config params were set up in
	 * probe() and should not need to be changed.
	 */
	pxp_conf->s0_param.width = src_width;
	pxp_conf->s0_param.height = src_height;
	proc_data->srect.top = update_region->top;
	proc_data->srect.left = update_region->left;
	proc_data->srect.width = update_region->width;
	proc_data->srect.height = update_region->height;

	/*
	 * Because only YUV/YCbCr image can be scaled, configure
	 * drect equivalent to srect, as such do not perform scaling.
	 */
	proc_data->drect.top = 0;
	proc_data->drect.left = 0;
	proc_data->drect.width = proc_data->srect.width;
	proc_data->drect.height = proc_data->srect.height;

	/* PXP expects rotation in terms of degrees */
	proc_data->rotate = fb_data->epdc_fb_var.rotate * 90;
	if (proc_data->rotate > 270)
		proc_data->rotate = 0;

	pxp_conf->out_param.width = update_region->width;
	pxp_conf->out_param.height = update_region->height;

	desc = to_tx_desc(txd);
	length = desc->len;
	for (i = 0; i < length; i++) {
		if (i == 0) {/* S0 */
			memcpy(&desc->proc_data, proc_data, sizeof(struct pxp_proc_data));
			pxp_conf->s0_param.paddr = sg_dma_address(&sg[0]);
			memcpy(&desc->layer_param.s0_param, &pxp_conf->s0_param,
				sizeof(struct pxp_layer_param));
		} else if (i == 1) {
			pxp_conf->out_param.paddr = sg_dma_address(&sg[1]);
			memcpy(&desc->layer_param.out_param, &pxp_conf->out_param,
				sizeof(struct pxp_layer_param));
		}
		/* TODO: OverLay */

		desc = desc->next;
	}

	/* Submitting our TX starts the PxP processing task */
	cookie = txd->tx_submit(txd);
	dev_dbg(fb_data->info.device, "%d: Submit %p #%d\n", __LINE__, txd,
		cookie);
	if (cookie < 0) {
		dev_err(fb_data->info.device, "Error sending FB through PxP\n");
		return -EIO;
	}

	fb_data->txd = txd;

	/* trigger ePxP */
	dma_async_issue_pending(dma_chan);

	return 0;
}

static int pxp_complete_update(struct mxc_epdc_fb_data *fb_data, u32 *hist_stat)
{
	int ret;
	/*
	 * Wait for completion event, which will be set
	 * through our TX callback function.
	 */
	ret = wait_for_completion_timeout(&fb_data->pxp_tx_cmpl, HZ / 10);
	if (ret <= 0) {
		dev_info(fb_data->info.device,
			 "PxP operation failed due to %s\n",
			 ret < 0 ? "user interrupt" : "timeout");
		dma_release_channel(&fb_data->pxp_chan->dma_chan);
		fb_data->pxp_chan = NULL;
		return ret ? : -ETIMEDOUT;
	}

	if (((fb_data->pxp_conf.proc_data.lut_transform & EPDC_FLAG_USE_CMAP) || use_cmap) &&
		fb_data->pxp_conf.proc_data.lut_map_updated)
		fb_data->pxp_conf.proc_data.lut_map_updated = false;

	*hist_stat = to_tx_desc(fb_data->txd)->hist_status;

	dev_dbg(fb_data->dev, "TX completed\n");

	return 0;
}

static int __init mxc_epdc_fb_init(void)
{
	return platform_driver_register(&mxc_epdc_fb_driver);
}
module_init(mxc_epdc_fb_init);


static void __exit mxc_epdc_fb_exit(void)
{
	platform_driver_unregister(&mxc_epdc_fb_driver);
}
module_exit(mxc_epdc_fb_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC EPDC framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("fb");