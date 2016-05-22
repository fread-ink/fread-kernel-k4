/*
 * Copyright (C) 2010-2011 Amazon.com, Inc. All Rights Reserved.
 * Manish Lachwani (lachwani@amazon.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/papyrus.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/videodev2.h>
#include <linux/mxcfb.h>
#include <linux/fec.h>
#include <linux/bootmem.h>
#include <linux/input/tequila_keypad.h>
#include <linux/whitney_button.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/flash.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc.h>
#include <mach/mxc_dvfs.h>
#include <mach/boardid.h>
#include <asm/mach/keypad.h>
#include "iomux.h"
#include "mx50_pins.h"
#include "devices.h"
#include "crm_regs.h"
#include "usb.h"

extern void __init mx50_yoshime_io_init(void);
extern int __init mx50_yoshime_init_mc34708(void);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
static int num_cpu_wp = 2;

void mxc_kernel_uptime(void)
{
	struct timespec uptime;

	/* Record the kernel boot time, now that this is the last thing */
	do_posix_clock_monotonic_gettime(&uptime);
	monotonic_to_bootbased(&uptime);

	printk("%lu.%02lu seconds:\n", (unsigned long) uptime.tv_sec,
			(uptime.tv_nsec / (NSEC_PER_SEC / 100)));
}
EXPORT_SYMBOL(mxc_kernel_uptime);

/*
 * In case of a battery drain down, this saves the last good time.
 */
u32 saved_last_seconds = 0;
EXPORT_SYMBOL(saved_last_seconds);

/*
 * Get the last good saved time
 */
static int __init mxc_last_saved_time(void)
{
	unsigned int regA = 0, regB = 0;

	/* Get the last good saved seconds */
	pmic_read_reg(REG_MC34708_MEM_A, &regA, (0xff << 16));
	pmic_read_reg(REG_MC34708_MEM_B, &regB, 0x00ffffff);

	saved_last_seconds = (regA << 8)| regB;

	printk(KERN_INFO "kernel: I perf:kernel:kernel_loaded=");
	mxc_kernel_uptime();

	return 0;
}
late_initcall(mxc_last_saved_time);

static struct mxc_dvfs_platform_data dvfs_core_data = {
       .reg_id = "SW1A",
       .clk1_id = "cpu_clk",
       .clk2_id = "gpc_dvfs_clk",
       .gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
       .gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
       .ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
       .ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
       .ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
       .prediv_mask = 0x1F800,
       .prediv_offset = 11,
       .prediv_val = 3,
       .div3ck_mask = 0xE0000000,
       .div3ck_offset = 29,
       .div3ck_val = 2,
       .emac_val = 0x08,
       .upthr_val = 25,
       .dnthr_val = 9,
       .pncthr_val = 33,
       .upcnt_val = 10,
       .dncnt_val = 10,
       .delay_time = 30,
       .num_wp = 2,
};

/* working point(wp): 0 - 800MHz; 1 - 166.25MHz; */
static struct cpu_wp cpu_wp_auto[] = {
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1050000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 160000000,
	 .pdf = 4,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 4,
	 .cpu_voltage = 850000,},
};

static struct cpu_wp *mx50_yoshi_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

static void mx50_yoshi_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}

static struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 1,
};

static struct fec_platform_data fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

extern void mx50_yoshi_gpio_spi_chipselect_active(int cspi_mode, int status,
						    int chipselect);
extern void mx50_yoshi_gpio_spi_chipselect_inactive(int cspi_mode, int status,
						      int chipselect);
static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
};

static struct mxc_spi_master mxcspi2_data = {
	.maxchipselect = 4,
	.spi_version = 23,
};

static struct mxc_spi_master mxcspi3_data = {
	.maxchipselect = 4,
	.spi_version = 7,
	.chipselect_active = mx50_yoshi_gpio_spi_chipselect_active,
	.chipselect_inactive = mx50_yoshi_gpio_spi_chipselect_inactive,
};

static struct spi_board_info panel_flash_device = {
	.modalias = "panel_flash_spi",
	.max_speed_hz = 1000000,        /* max spi SCK clock speed in HZ */
	.bus_num = 2,
	.chip_select = 0,
};

static struct mxc_i2c_platform_data mxci2c_data = {
	.i2c_clk = 100000,
};

static struct mxc_srtc_platform_data srtc_data = {
	.srtc_sec_mode_addr = OCOTP_CTRL_BASE_ADDR + 0x80,
};

#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

static struct regulator_init_data papyrus_init_data[] = {
        {
                .constraints = {
                        .name = "DISPLAY",
                },
        }, {
                .constraints = {
                        .name = "GVDD",
                        .min_uV = V_to_uV(20),
                        .max_uV = V_to_uV(20),
                },
        }, {
                .constraints = {
                        .name = "GVEE",
                        .min_uV = V_to_uV(-22),
                        .max_uV = V_to_uV(-22),
                },
        }, {
                .constraints = {
                        .name = "VCOM",
                        .min_uV = mV_to_uV(0),
                        .max_uV = mV_to_uV(2749),
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
                },
        }, {
                .constraints = {
                        .name = "VNEG",
                        .min_uV = V_to_uV(-15),
                        .max_uV = V_to_uV(-15),
                },
        }, {
                .constraints = {
                        .name = "VPOS",
                        .min_uV = V_to_uV(15),
                        .max_uV = V_to_uV(15),
                        .min_uV = V_to_uV(15),
                        .max_uV = V_to_uV(15),
                },
        }, {
                .constraints = {
                        .name = "TMST",
                },
        },
};

static struct papyrus_platform_data papyrus_pdata = {
	.vneg_pwrup = 1,
	.gvee_pwrup = 1,
	.vpos_pwrup = 2,
	.gvdd_pwrup = 1,
	.gvdd_pwrdn = 1,
	.vpos_pwrdn = 2,
	.gvee_pwrdn = 1,
	.vneg_pwrdn = 1,
	.gpio_pmic_pwrgood = IOMUX_TO_GPIO(MX50_PIN_EPDC_PWRSTAT),
	.gpio_pmic_vcom_ctrl = IOMUX_TO_GPIO(MX50_PIN_EPDC_PWRCOM),
	.gpio_pmic_wakeup = IOMUX_TO_GPIO(MX50_PIN_EPDC_PWRCTRL0),
	.gpio_pmic_intr = IOMUX_TO_GPIO(MX50_PIN_EPDC_PWRCTRL1),
	.regulator_init = papyrus_init_data,
};

#ifdef CONFIG_MX50_YOSHI_SX5844

static void gpio_sx5844_get(void)
{
	/* Needs Support */
}

static void gpio_sx5844_put(void)
{
	/* Needs Support */
}

#define MX50_SX5844_STX0	135
#define MX50_SX5844_SRX0	136

static struct mxc_sx5844_platform_data sx5844_data = {
	.reg_dvdd_io = "GPO3",
	.reg_avdd = "VSD",
	.gpio_pin_get = gpio_sx5844_get,
	.gpio_pin_put = gpio_sx5844_put,
	.int1 = MX50_SX5844_STX0,
	.int2 = MX50_SX5844_SRX0,
};

#endif

static struct i2c_board_info mxc_i2c2_board_info[] = {
	{
		.type = "cyttsp",
		.addr = 0x24,
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] = {
	{
	 .type = "Yoshi_Battery",
	 .addr = 0x55,
	 },
	{
	 .type = "wm8960",	
	 .addr = 0x1a,
	 .irq = IOMUX_TO_IRQ(MX50_PIN_EIM_DA4),
	},
	{
	 I2C_BOARD_INFO("papyrus", 0x48),
	 .platform_data = &papyrus_pdata,
	},
	{
	 .type = "maxim_al32",
	 .addr = 0x35,
	 .irq = IOMUX_TO_IRQ(MX50_PIN_OWIRE),
	},
	{
	 .type = "summit_smb347",
	 .addr = 0x06,
	 .irq = IOMUX_TO_IRQ(MX50_PIN_OWIRE),
	}
};

static struct i2c_board_info mxc_i2c0_board_info[] = {
	{
	 .type = "MX50_Proximity",
	 .addr = 0x0D,
	},
	{
	 .type = "mma8653",
	 .addr = 0x1D,
	},
};

static struct fb_videomode e60_v110_mode = {
	.name = "E60_V110",
	.refresh = 50,
	.xres = 800,
	.yres = 600,
	.pixclock = 18604700,
	.left_margin = 8,
	.right_margin = 178,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode e60_v220_mode = {
	.name = "E60_V220",
	.refresh = 85,
	.xres = 800,
#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	.yres = 800,
#else
	.yres = 600,
#endif
	.pixclock = 32000000,
	.left_margin = 8,
	.right_margin = 166,
	.upper_margin = 4,
	.lower_margin = 26,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode e60_v220_210_mode = {
	.name = "E60_V220_210",
	.refresh = 85,
	.xres = 1024,
#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	.yres = 1024,
#else
	.yres = 758,
#endif
	.pixclock = 40000000,
	.left_margin = 8,
	.right_margin = 166,
	.upper_margin = 4,
	.lower_margin = 26,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode e97_v110_mode = {
	.name = "E97_V110",
	.refresh = 50,
	.xres = 1200,
	.yres = 825,
	.pixclock = 32000000,
	.left_margin = 12,
	.right_margin = 128,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct mxc_epdc_fb_mode panel_modes[] = {
	{
		&e60_v220_210_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		428,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e60_v220_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		428,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e60_v110_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		428,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e97_v110_mode,
		8,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		632,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		3,      /* num_ce */
	},
};

extern int gpio_epdc_pins_enable(int enable);

static void epdc_get_pins(void)
{
	return;
}

static void epdc_put_pins(void)
{
        return;
}

static void epdc_disable_pins(void)
{
	gpio_epdc_pins_enable(0);
        return;
}

static void epdc_enable_pins(void)
{
	gpio_epdc_pins_enable(1);
        return;
}

static struct mxc_epdc_fb_platform_data epdc_data = {
	.epdc_mode = panel_modes,
	.num_modes = ARRAY_SIZE(panel_modes),
	.get_pins = epdc_get_pins,
	.put_pins = epdc_put_pins,
	.enable_pins = epdc_enable_pins,
	.disable_pins = epdc_disable_pins,
};

static int sdhc_write_protect(struct device *dev)
{
	return 0;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{

    /* card detect for ESDHC4 */
    if (to_platform_device(dev)->id == 3) {
		return gpio_get_value(IOMUX_TO_GPIO(MX50_PIN_DISP_D15));
    }
    
    return 0;
}

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ | MMC_CAP_SD_HIGHSPEED,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

static struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
	        | MMC_VDD_31_32,
	.min_clk = 400000,
	.max_clk = 40000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
	.caps = MMC_CAP_8_BIT_DATA | MMC_CAP_MMC_HIGHSPEED,  
};

#ifdef CONFIG_MX50_YOSHI_SDCARD

static struct mxc_mmc_platform_data mmc4_data = {
        .ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
                | MMC_VDD_31_32,
        .caps = MMC_CAP_4_BIT_DATA,
        .min_clk = 150000,
        .max_clk = 20000000,
        .card_inserted_state = 1,
        .status = sdhc_get_card_det_status,
        .wp_status = sdhc_write_protect,
        .clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};
#endif

/* Check CPU register for wdog reset */
int mx50_srsr_wdog(void)
{
	/* Read SRSR */
	unsigned long reg = __raw_readl(MXC_SRC_BASE + 0x008);

	/* Bit 4 is wdog rst */
	return (reg & 0x10);
}
EXPORT_SYMBOL(mx50_srsr_wdog);

static void wvga_reset(void)
{
	/* ELCDIF D0 */
	mxc_free_iomux(MX50_PIN_DISP_D0, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D0, IOMUX_CONFIG_ALT0);
	/* ELCDIF D1 */
	mxc_free_iomux(MX50_PIN_DISP_D1, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D1, IOMUX_CONFIG_ALT0);
	/* ELCDIF D2 */
	mxc_free_iomux(MX50_PIN_DISP_D2, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D2, IOMUX_CONFIG_ALT0);
	/* ELCDIF D3 */
	mxc_free_iomux(MX50_PIN_DISP_D3, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D3, IOMUX_CONFIG_ALT0);
	/* ELCDIF D4 */
	mxc_free_iomux(MX50_PIN_DISP_D4, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D4, IOMUX_CONFIG_ALT0);
	/* ELCDIF D5 */
	mxc_free_iomux(MX50_PIN_DISP_D5, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D5, IOMUX_CONFIG_ALT0);
	/* ELCDIF D6 */
	mxc_free_iomux(MX50_PIN_DISP_D6, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D6, IOMUX_CONFIG_ALT0);
	/* ELCDIF D7 */
	mxc_free_iomux(MX50_PIN_DISP_D7, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D7, IOMUX_CONFIG_ALT0);
	return;
}

static struct mxc_audio_platform_data wm8960_data = {
        .ssi_num = 1,
        .src_port = 2,
        .ext_port = 3,
};

static struct platform_device mxc_wm8960_device = {
        .name = "imx-yoshime-wm8960",	
};

static struct mxc_lcd_platform_data lcd_wvga_data = {
	.reset = wvga_reset,
};

static struct platform_device lcd_wvga_device = {
	.name = "lcd_claa",
	.dev = {
		.platform_data = &lcd_wvga_data,
		},
};

static struct fb_videomode video_modes[] = {
	{
	 /* 800x480 @ 55 Hz , pixel clk @ 25MHz */
	 "CLAA-WVGA", 55, 800, 480, 40000, 40, 40, 5, 5, 20, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = V4L2_PIX_FMT_RGB565,
	 .mode_str = "CLAA-WVGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
};

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	mxc_set_cpu_type(MXC_CPU_MX50);

	get_cpu_wp = mx50_yoshi_get_cpu_wp;
	set_num_cpu_wp = mx50_yoshi_set_num_cpu_wp;

	mx50_init_boardid();
}

extern void pmic_power_off(void);
extern void gpio_watchdog_low(void);

#define PMIC_BUTTON_DEBOUNCE_VALUE	0x3
#define PMIC_BUTTON_DEBOUNCE_MASK	0x3

static void mxc_configure_pb_debounce(void)
{
	/* Configure debounce time for power button 1 */
	pmic_write_reg(REG_MC34708_POWER_CTL2, (PMIC_BUTTON_DEBOUNCE_VALUE << 4),
				(PMIC_BUTTON_DEBOUNCE_MASK << 4));

	/* Configure debounce time for power button 2 */
	pmic_write_reg(REG_MC34708_POWER_CTL2, (PMIC_BUTTON_DEBOUNCE_VALUE << 6),
				(PMIC_BUTTON_DEBOUNCE_MASK << 6));

	pmic_write_reg(REG_MC34708_POWER_CTL2, (0 << 1), (1 << 1));
	pmic_write_reg(REG_MC34708_POWER_CTL2, (0 << 2), (1 << 2));
}

static void mx50_yoshi_poweroff(void)
{
	mxc_configure_pb_debounce();

	/* Clear out the RTC time interrupt */
	pmic_write_reg(REG_MC34708_INT_STATUS1, 0, 0x2);

	/* Mask the RTC alarm interrupt */
	pmic_write_reg(REG_MC34708_INT_MASK1, 0x3, 0x3);

	/* Zero out the RTC alarm day */
	pmic_write_reg(REG_MC34708_RTC_ALARM, 0x0, 0xffffffff);

	/* Zero out the RTC alarm time */
	pmic_write_reg(REG_MC34708_RTC_DAY_ALARM, 0x0, 0xffffffff);

	/* Record it for boot string */
	pmic_write_reg(REG_MC34708_MEM_A, (1 << 2), (1 << 2));

	/* Pull the wdog pin low */
	gpio_watchdog_low();
}

extern int set_high_bus_freq(int high_freq);

static void mx50_yoshi_restart(char mode, const char *cmd)
{
	printk(KERN_EMERG "Restarting Yoshi\n");

	set_high_bus_freq(0);
	arm_machine_restart('h', cmd);
}

#define FAKE_MMC_DETECT_IRQ	0	

#if defined(CONFIG_MXC_AMD_GPU_LAB126) || defined(CONFIG_MXC_AMD_GPU_LAB126_MODULE)
static int z160_version = 1;
#endif

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	/* SD card detect irqs */
	mxcsdhc2_device.resource[2].start = IOMUX_TO_IRQ(MX50_PIN_SD2_CD);
	mxcsdhc2_device.resource[2].end = IOMUX_TO_IRQ(MX50_PIN_SD2_CD);
	mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ(MX50_PIN_EIM_CRE);
	mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ(MX50_PIN_EIM_CRE);
	mxcsdhc3_device.resource[2].start = FAKE_MMC_DETECT_IRQ; 
	mxcsdhc3_device.resource[2].end = FAKE_MMC_DETECT_IRQ; 
	mxcsdhc4_device.resource[2].start = IOMUX_TO_IRQ(MX50_PIN_DISP_D15);
	mxcsdhc4_device.resource[2].end = IOMUX_TO_IRQ(MX50_PIN_DISP_D15);

	mxc_cpu_common_init();
	mxc_register_gpios();
	mx50_yoshime_io_init();
	
	mxc_register_device(&busfreq_device, NULL);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mxc_register_device(&mxcspi2_device, &mxcspi2_data);
	mxc_register_device(&mxcspi3_device, &mxcspi3_data);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[2], &mxci2c_data);
 
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));

	mx50_yoshime_init_mc34708();

	mxc_register_device(&mxc_rtc_device, &srtc_data);
	mxc_register_device(&mxc_w1_master_device, &mxc_w1_data);
	mxc_register_device(&mxc_pxp_device, NULL);
	mxc_register_device(&mxc_pxp_client_device, NULL);

	mxc_register_device(&mxcsdhc2_device, &mmc2_data);
	mxc_register_device(&mxcsdhc3_device, &mmc3_data);

	spi_register_board_info(&panel_flash_device,
				sizeof(panel_flash_device));

	mxc_register_device(&epdc_device, &epdc_data);

	mx5_usb_dr_init();

	pm_power_off = mx50_yoshi_poweroff;
	arm_pm_restart = mx50_yoshi_restart;

#ifdef CONFIG_MX50_YOSHI_SDCARD
	mxc_register_device(&mxcsdhc4_device, &mmc4_data);
#endif
	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&mxc_fec_device, &fec_data);
	mxc_register_device(&mxc_yoshime_ripley_device, NULL);
	
	mxc_register_device(&lcd_wvga_device, &lcd_wvga_data);
	mxc_register_device(&elcdif_device, &fb_data[0]);

	mx5_usbh1_init();

	mxc_register_device(&mxc_wm8960_device, &wm8960_data);

#if defined(CONFIG_MXC_AMD_GPU_LAB126) || defined(CONFIG_MXC_AMD_GPU_LAB126_MODULE)
	mxc_register_device(&gpu_device, &z160_version);
#endif

	if (cpu_is_mx50_rev(CHIP_REV_1_1) >= 1)
		mxc_register_device(&mxc_zq_calib_device, NULL);
}

char *mxc_get_task_comm(char *buf, struct task_struct *tsk)
{
        return get_task_comm(buf, tsk);
}
EXPORT_SYMBOL(mxc_get_task_comm);

extern void __init mx50_init_idle_clocks(void);

static void __init mx50_yoshi_timer_init(void)
{
	struct clk *uart_clk;

	mx50_clocks_init(32768, 24000000, 22579200);
	mx50_init_idle_clocks();

	uart_clk = clk_get(NULL, "uart_clk.0");
	early_console_setup(MX53_BASE_ADDR(UART1_BASE_ADDR), uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx50_yoshi_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX50_YOSHI data structure.
 */
MACHINE_START(MX50_YOSHI, "Amazon.com MX50 YOSHIME Board")
	/* Maintainer: Manish Lachwani <lachwani@amazon.com> */
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
