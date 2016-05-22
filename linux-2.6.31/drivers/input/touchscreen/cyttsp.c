/* 
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * Copyright 2011 Amazon.com, Inc. All rights reserved.
 * Vidhyananth Venkatasamy (venkatas@lab126.com)
 *
 * Cypress TrueTouch(TM) Standard Product touchscreen driver.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/byteorder/generic.h>
#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include "cyttsp.h"

int i2c_probe_success = 0;
extern void gpio_touchcntrl_request_irq(int enable);
extern int gpio_touchcntrl_irq(void);
static struct cyttsp *cyttsp_tsc = NULL;
static struct cyttsp_i2c *cyttsp_i2c_clientst = NULL;

static struct cyttsp_platform_data cyttsp_i2c_platform_data = {
#ifdef CY_USE_MT
	.mt_sync = input_mt_sync,
#endif
	.maxx = 240,
	.maxy = 320,
	.flags = 0,
	.gen = CY_GEN3,
	.use_st = 0,
	.use_mt = 1,
	.use_trk_id = 0,
	.use_hndshk = 1,
	.use_timer = 0,
	.use_sleep = 1,
	.use_gestures = 0,
	.use_load_file = 1,
	.use_force_fw_update = 1,
	.use_virtual_keys = 0,
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CY_GEST_GRP_NONE | CY_ACT_DIST,
	/* change act_intrvl to customize the Active power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.name = CY_DRIVER_NAME,
};

static irqreturn_t cyttsp_bl_ready_irq(int irq, void *handle);
static int cyttsp_soft_reset(struct cyttsp *ts, bool *status);
static int cyttsp_set_operational_mode(struct cyttsp *ts);
static int cyttsp_exit_bl_mode(struct cyttsp *ts);

/* ************************************************************************
   *********************  CYTTSP - I2C CLIENT I/F *************************
  *************************************************************************/
struct cyttsp_i2c {
    struct cyttsp_bus_ops ops;
    struct i2c_client *client;
    void *ttsp_client;
};

static s32 ttsp_i2c_read_block_data(void *handle, u8 addr,
    u8 length, void *values)
{
	int retval = 0;
	struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);
	retval = i2c_master_send(ts->client, &addr, 1);
	if (retval < 0)
		return retval;
	retval = i2c_master_recv(ts->client, values, length);
	
	return (retval < 0) ? retval : 0;
}

static s32 ttsp_i2c_write_block_data(void *handle, u8 addr,
    u8 length, const void *values)
{
	u8 data[I2C_SMBUS_BLOCK_MAX+1];
	int num_bytes, count;
	int retval;
	struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);
	num_bytes = length;
	data[0] = addr;
	count = (num_bytes > I2C_SMBUS_BLOCK_MAX) ?
		I2C_SMBUS_BLOCK_MAX : num_bytes;
	memcpy(&data[1], values, count+1);
	num_bytes -= count;
	retval = i2c_master_send(ts->client, data, count+1);
	if (retval < 0)
		return retval;
	while (num_bytes > 0) {
		count = (num_bytes > I2C_SMBUS_BLOCK_MAX) ?
			I2C_SMBUS_BLOCK_MAX : num_bytes;
		memcpy(&data[0], values, count);
		num_bytes -= count;
		retval = i2c_master_send(ts->client, data, count);
		if (retval < 0)
			return retval;
	}
	
	return 0;
}

static s32 ttsp_i2c_tch_ext(void *handle, void *values)
{
	int retval = 0;
	struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);
	
	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)
	
	/* Add custom touch extension handling code here */
	/* set: retval < 0 for any returned system errors,
	    retval = 0 if normal touch handling is required,
	    retval > 0 if normal touch handling is *not* required */
	if (!ts || !values)
		retval = -EIO;
	
	return retval;
}

static int __devinit cyttsp_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
	struct cyttsp_i2c *ts;
	int retval;
	
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	
	/* allocate and clear memory */
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "%s: Error, kzalloc.\n", __func__);
		retval = -ENOMEM;
		goto error_alloc_data_failed;
	}
	
	/* register driver_data */
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->ops.write = ttsp_i2c_write_block_data;
	ts->ops.read = ttsp_i2c_read_block_data;
	ts->ops.ext = ttsp_i2c_tch_ext;
	
	cyttsp_i2c_clientst = ts;		
	
	DBG(printk(KERN_INFO "%s: Registration complete %s\n",
	    __func__, CY_DRIVER_NAME);)
	i2c_probe_success = 1;		
	return 0;

error_alloc_data_failed:
	return retval;
}

static int __devexit cyttsp_i2c_remove(struct i2c_client *client)
{
	struct cyttsp_i2c *ts;
	
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	ts = i2c_get_clientdata(client);
	kfree(ts);
	return 0;
}

static const struct i2c_device_id cyttsp_i2c_id[] = {
	{ CY_DRIVER_NAME, 0 },  { }
};

static struct i2c_driver cyttsp_i2c_driver = {
	.driver = {
		.name = CY_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = cyttsp_i2c_probe,
	.remove = __devexit_p(cyttsp_i2c_remove),
	.id_table = cyttsp_i2c_id,
};


/* ************************************************************************
   ************************  CYTTSP - CORE I/F ****************************
  *************************************************************************/
static void cyttsp_init_tch(struct cyttsp *ts)
{
	/* init the touch structures */
	ts->num_prv_st_tch = CY_NTCH;
	memset(ts->act_trk, CY_NTCH, sizeof(ts->act_trk));
	memset(ts->prv_mt_pos, CY_NTCH, sizeof(ts->prv_mt_pos));
	memset(ts->prv_mt_tch, CY_IGNR_TCH, sizeof(ts->prv_mt_tch));
	memset(ts->prv_st_tch, CY_IGNR_TCH, sizeof(ts->prv_st_tch));
}

static u8 ttsp_convert_gen2(u8 cur_tch, struct cyttsp_xydata *pxy_data)
{
	struct cyttsp_xydata_gen2 *pxy_data_gen2;
	pxy_data_gen2 = (struct cyttsp_xydata_gen2 *)(pxy_data);

	if (pxy_data_gen2->evnt_idx == CY_GEN2_NOTOUCH) {
		cur_tch = 0;
	} else if (cur_tch == CY_GEN2_GHOST) {
		cur_tch = 0;
	} else if (cur_tch == CY_GEN2_2TOUCH) {
		/* stuff artificial track ID1 and ID2 */
		pxy_data->touch12_id = 0x12;
		pxy_data->z1 = CY_MAXZ;
		pxy_data->z2 = CY_MAXZ;
		cur_tch--; /* 2 touches */
	} else if (cur_tch == CY_GEN2_1TOUCH) {
		/* stuff artificial track ID1 and ID2 */
		pxy_data->touch12_id = 0x12;
		pxy_data->z1 = CY_MAXZ;
		pxy_data->z2 = CY_NTCH;
		if (pxy_data_gen2->evnt_idx == CY_GEN2_TOUCH2) {
			/* push touch 2 data into touch1
			 * (first finger up; second finger down) */
			/* stuff artificial track ID1 for touch2 info */
			pxy_data->touch12_id = 0x20;
			/* stuff touch 1 with touch 2 coordinate data */
			pxy_data->x1 = pxy_data->x2;
			pxy_data->y1 = pxy_data->y2;
		}
	} else {
		cur_tch = 0;
	}
	return cur_tch;
}

static int ttsp_read_block_data(struct cyttsp *ts, u8 command,
	u8 length, void *buf)
{
	int rc;
	int tries;
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	if (!buf || !length) {
		printk(KERN_ERR "%s: Error, buf:%s len:%u\n",
				__func__, !buf ? "NULL" : "OK", length);
		return -EIO;
	}

	for (tries = 0, rc = -1; tries < CY_NUM_RETRY && (rc < 0); tries++) {
		rc = ts->bus_ops->read(ts->bus_ops, command, length, buf);
		if (rc)
			msleep(CY_DELAY_DFLT);
	}

	if (rc < 0)
		printk(KERN_ERR "%s: error %d\n", __func__, rc);
	DBG(print_data_block(__func__, command, length, buf);)
	return rc;
}

static int ttsp_write_block_data(struct cyttsp *ts, u8 command,
	u8 length, void *buf)
{
	int rc;
	int tries;
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	if (!buf || !length) {
		printk(KERN_ERR "%s: Error, buf:%s len:%u\n",
				__func__, !buf ? "NULL" : "OK", length);
		return -EIO;
	}

	for (tries = 0, rc = -1; tries < CY_NUM_RETRY && (rc < 0); tries++) {
		rc = ts->bus_ops->write(ts->bus_ops, command, length, buf);
		if (rc)
			msleep(CY_DELAY_DFLT);
	}

	if (rc < 0)
		printk(KERN_ERR "%s: error %d\n", __func__, rc);
	DBG(print_data_block(__func__, command, length, buf);)
	return rc;
}

static int ttsp_tch_ext(struct cyttsp *ts, void *buf)
{
	int rc;
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	if (!buf) {
		printk(KERN_ERR "%s: Error, buf:%s\n",
				__func__, !buf ? "NULL" : "OK");
		return -EIO;
	}
	rc = ts->bus_ops->ext(ts->bus_ops, buf);
	if (rc < 0)
		printk(KERN_ERR "%s: error %d\n", __func__, rc);
	return rc;
}

static int cyttsp_inlist(u16 prev_track[], u8 cur_trk_id, u8 *prev_loc,
	u8 num_touches)
{
	u8 id = 0;

	DBG(printk(KERN_INFO"%s: IN p[%d]=%d c=%d n=%d loc=%d\n",
		__func__, id, prev_track[id], cur_trk_id,
		num_touches, *prev_loc);)

	for (*prev_loc = CY_IGNR_TCH; id < num_touches; id++) {
		DBG(printk(KERN_INFO"%s: p[%d]=%d c=%d n=%d loc=%d\n",
			__func__, id, prev_track[id], cur_trk_id,
				num_touches, *prev_loc);)
		if (prev_track[id] == cur_trk_id) {
			*prev_loc = id;
			break;
		}
	}
	DBG(printk(KERN_INFO"%s: OUT p[%d]=%d c=%d n=%d loc=%d\n", __func__,
		id, prev_track[id], cur_trk_id, num_touches, *prev_loc);)

	return *prev_loc < CY_NUM_TRK_ID;
}

static int cyttsp_next_avail_inlist(u16 cur_trk[], u8 *new_loc,
	u8 num_touches)
{
	u8 id = 0;
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	for (*new_loc = CY_IGNR_TCH; id < num_touches; id++) {
		if (cur_trk[id] > CY_NUM_TRK_ID) {
			*new_loc = id;
			break;
		}
	}
	return *new_loc < CY_NUM_TRK_ID;
}

/* Timer function used as dummy interrupt driver */
static void cyttsp_timer(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	DBG(printk(KERN_INFO"%s: TTSP timer event!\n", __func__);)
	/* schedule motion signal handling */
	if (!work_pending(&ts->work))
		schedule_work(&ts->work);
	mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
	return;
}


/* ************************************************************************
 * ISR function. This function is general, initialized in drivers init
 * function and disables further IRQs until this IRQ is processed in worker.
 * *************************************************************************/
static irqreturn_t cyttsp_irq(int irq, void *handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	DBG(printk(KERN_INFO"%s: Got IRQ!\n", __func__);)
	if (!work_pending(&ts->work))
		schedule_work(&ts->work);
	return IRQ_HANDLED;
}

/* ************************************************************************
 * The cyttsp_xy_worker function reads the XY coordinates and sends them to
 * the input layer.  It is scheduled from the interrupt (or timer).
 * *************************************************************************/
static void handle_single_touch(struct cyttsp_xydata *xy,
	struct cyttsp_track_data *t, struct cyttsp *ts)
{
	u8 id;
	u8 use_trk_id = ts->platform_data->use_trk_id;

	DBG(printk(KERN_INFO"%s: ST STEP 0 - ST1 ID=%d  ST2 ID=%d\n",
		__func__, t->cur_st_tch[CY_ST_FNGR1_IDX],
		t->cur_st_tch[CY_ST_FNGR2_IDX]);)

	if (t->cur_st_tch[CY_ST_FNGR1_IDX] > CY_NUM_TRK_ID) {
		/* reassign finger 1 and 2 positions to new tracks */
		if (t->cur_tch > 0) {
			/* reassign st finger1 */
			if (use_trk_id) {
				id = CY_MT_TCH1_IDX;
				t->cur_st_tch[CY_ST_FNGR1_IDX] =
							t->cur_mt_tch[id];
			} else {
				id = GET_TOUCH1_ID(xy->touch12_id);
				t->cur_st_tch[CY_ST_FNGR1_IDX] = id;
			}
			t->st_x1 = t->cur_mt_pos[id][CY_XPOS];
			t->st_y1 = t->cur_mt_pos[id][CY_YPOS];
			t->st_z1 = t->cur_mt_z[id];

			DBG(printk(KERN_INFO"%s: ST STEP 1 - ST1 ID=%3d\n",
				__func__, t->cur_st_tch[CY_ST_FNGR1_IDX]);)

			if ((t->cur_tch > 1) &&
				(t->cur_st_tch[CY_ST_FNGR2_IDX] >
				CY_NUM_TRK_ID)) {
				/* reassign st finger2 */
				if (use_trk_id) {
					id = CY_MT_TCH2_IDX;
					t->cur_st_tch[CY_ST_FNGR2_IDX] =
						t->cur_mt_tch[id];
				} else {
					id = GET_TOUCH2_ID(xy->touch12_id);
					t->cur_st_tch[CY_ST_FNGR2_IDX] = id;
				}
				t->st_x2 = t->cur_mt_pos[id][CY_XPOS];
				t->st_y2 = t->cur_mt_pos[id][CY_YPOS];
				t->st_z2 = t->cur_mt_z[id];

				DBG(
				printk(KERN_INFO"%s: ST STEP 2 - ST2 ID=%3d\n",
				__func__, t->cur_st_tch[CY_ST_FNGR2_IDX]);)
			}
		}
	} else if (t->cur_st_tch[CY_ST_FNGR2_IDX] > CY_NUM_TRK_ID) {
		if (t->cur_tch > 1) {
			/* reassign st finger2 */
			if (use_trk_id) {
				/* reassign st finger2 */
				id = CY_MT_TCH2_IDX;
				t->cur_st_tch[CY_ST_FNGR2_IDX] =
							t->cur_mt_tch[id];
			} else {
				/* reassign st finger2 */
				id = GET_TOUCH2_ID(xy->touch12_id);
				t->cur_st_tch[CY_ST_FNGR2_IDX] = id;
			}
			t->st_x2 = t->cur_mt_pos[id][CY_XPOS];
			t->st_y2 = t->cur_mt_pos[id][CY_YPOS];
			t->st_z2 = t->cur_mt_z[id];

			DBG(printk(KERN_INFO"%s: ST STEP 3 - ST2 ID=%3d\n",
				   __func__, t->cur_st_tch[CY_ST_FNGR2_IDX]);)
		}
	}
	/* if the 1st touch is missing and there is a 2nd touch,
	 * then set the 1st touch to 2nd touch and terminate 2nd touch
	 */
	if ((t->cur_st_tch[CY_ST_FNGR1_IDX] > CY_NUM_TRK_ID) &&
			(t->cur_st_tch[CY_ST_FNGR2_IDX] < CY_NUM_TRK_ID)) {
		t->st_x1 = t->st_x2;
		t->st_y1 = t->st_y2;
		t->st_z1 = t->st_z2;
		t->cur_st_tch[CY_ST_FNGR1_IDX] = t->cur_st_tch[CY_ST_FNGR2_IDX];
		t->cur_st_tch[CY_ST_FNGR2_IDX] = CY_IGNR_TCH;
	}
	/* if the 2nd touch ends up equal to the 1st touch,
	 * then just report a single touch */
	if (t->cur_st_tch[CY_ST_FNGR1_IDX] == t->cur_st_tch[CY_ST_FNGR2_IDX])
		t->cur_st_tch[CY_ST_FNGR2_IDX] = CY_IGNR_TCH;

	/* set Single Touch current event signals */
	if (t->cur_st_tch[CY_ST_FNGR1_IDX] < CY_NUM_TRK_ID) {
		input_report_abs(ts->input, ABS_X, t->st_x1);
		input_report_abs(ts->input, ABS_Y, t->st_y1);
		input_report_abs(ts->input, ABS_PRESSURE, t->st_z1);
		input_report_key(ts->input, BTN_TOUCH, CY_TCH);
		input_report_abs(ts->input, ABS_TOOL_WIDTH, t->tool_width);

		DBG2(printk(KERN_INFO"%s:ST->F1:%3d X:%3d Y:%3d Z:%3d\n",
			   __func__, t->cur_st_tch[CY_ST_FNGR1_IDX],
			   t->st_x1, t->st_y1, t->st_z1);)

		if (t->cur_st_tch[CY_ST_FNGR2_IDX] < CY_NUM_TRK_ID) {
			input_report_key(ts->input, BTN_2, CY_TCH);
			input_report_abs(ts->input, ABS_HAT0X, t->st_x2);
			input_report_abs(ts->input, ABS_HAT0Y, t->st_y2);

			DBG2(printk(KERN_INFO
				"%s:ST->F2:%3d X:%3d Y:%3d Z:%3d\n",
				__func__, t->cur_st_tch[CY_ST_FNGR2_IDX],
				t->st_x2, t->st_y2, t->st_z2);)
		} else {
			input_report_key(ts->input, BTN_2, CY_NTCH);
		}
	} else {
		input_report_abs(ts->input, ABS_PRESSURE, CY_NTCH);
		input_report_key(ts->input, BTN_TOUCH, CY_NTCH);
		input_report_key(ts->input, BTN_2, CY_NTCH);
	}
	/* update platform data for the current single touch info */
	ts->prv_st_tch[CY_ST_FNGR1_IDX] = t->cur_st_tch[CY_ST_FNGR1_IDX];
	ts->prv_st_tch[CY_ST_FNGR2_IDX] = t->cur_st_tch[CY_ST_FNGR2_IDX];

}

static void handle_multi_touch(struct cyttsp_track_data *t, struct cyttsp *ts)
{

	u8 id;
	u8 i, loc;
	void (*mt_sync_func)(struct input_dev *) = ts->platform_data->mt_sync;

	if (!ts->platform_data->use_trk_id)
		goto no_track_id;

	/* terminate any previous touch where the track
	 * is missing from the current event */
	for (id = 0; id < CY_NUM_TRK_ID; id++) {
		if ((ts->act_trk[id] == CY_NTCH) || (t->cur_trk[id] != CY_NTCH))
			continue;

		input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, CY_NTCH);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, t->tool_width);
		input_report_abs(ts->input, ABS_MT_POSITION_X,
					ts->prv_mt_pos[id][CY_XPOS]);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,
					ts->prv_mt_pos[id][CY_YPOS]);
		if (mt_sync_func)
			mt_sync_func(ts->input);
		ts->act_trk[id] = CY_NTCH;
		ts->prv_mt_pos[id][CY_XPOS] = 0;
		ts->prv_mt_pos[id][CY_YPOS] = 0;
	}
	/* set Multi-Touch current event signals */
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		if (t->cur_mt_tch[id] >= CY_NUM_TRK_ID)
			continue;

		input_report_abs(ts->input, ABS_MT_TRACKING_ID,
						t->cur_mt_tch[id]);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
						t->cur_mt_z[id]);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
						t->tool_width);
		input_report_abs(ts->input, ABS_MT_POSITION_X,
						t->cur_mt_pos[id][CY_XPOS]);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,
						t->cur_mt_pos[id][CY_YPOS]);
		if (mt_sync_func)
			mt_sync_func(ts->input);

		ts->act_trk[id] = CY_TCH;
		ts->prv_mt_pos[id][CY_XPOS] = t->cur_mt_pos[id][CY_XPOS];
		ts->prv_mt_pos[id][CY_YPOS] = t->cur_mt_pos[id][CY_YPOS];
	}
	return;
no_track_id:

	/* set temporary track array elements to voids */
	memset(t->tmp_trk, CY_IGNR_TCH, sizeof(t->tmp_trk));
	memset(t->snd_trk, CY_IGNR_TCH, sizeof(t->snd_trk));

	/* get what is currently active */
	for (i = id = 0; id < CY_NUM_TRK_ID && i < CY_NUM_MT_TCH_ID; id++) {
		if (t->cur_trk[id] == CY_TCH) {
			/* only incr counter if track found */
			t->tmp_trk[i] = id;
			i++;
		}
	}
	DBG(printk(KERN_INFO"%s: T1: t0=%d, t1=%d, t2=%d, t3=%d\n", __func__,
					t->tmp_trk[0], t->tmp_trk[1],
					t->tmp_trk[2], t->tmp_trk[3]);)
	DBG(printk(KERN_INFO"%s: T1: p0=%d, p1=%d, p2=%d, p3=%d\n", __func__,
					ts->prv_mt_tch[0], ts->prv_mt_tch[1],
					ts->prv_mt_tch[2], ts->prv_mt_tch[3]);)

	/* pack in still active previous touches */
	for (id = t->prv_tch = 0; id < CY_NUM_MT_TCH_ID; id++) {
		if (t->tmp_trk[id] >= CY_NUM_TRK_ID)
			continue;

		if (cyttsp_inlist(ts->prv_mt_tch, t->tmp_trk[id], &loc,
							CY_NUM_MT_TCH_ID)) {
			loc &= CY_NUM_MT_TCH_ID - 1;
			t->snd_trk[loc] = t->tmp_trk[id];
			t->prv_tch++;
			DBG(printk(KERN_INFO"%s: in list s[%d]=%d "
					"t[%d]=%d, loc=%d p=%d\n", __func__,
					loc, t->snd_trk[loc],
					id, t->tmp_trk[id],
					loc, t->prv_tch);)
		} else {
			DBG(printk(KERN_INFO"%s: is not in list s[%d]=%d"
					" t[%d]=%d loc=%d\n", __func__,
					id, t->snd_trk[id],
					id, t->tmp_trk[id],
					loc);)
		}
	}
	DBG(printk(KERN_INFO"%s: S1: s0=%d, s1=%d, s2=%d, s3=%d p=%d\n",
		   __func__,
		   t->snd_trk[0], t->snd_trk[1], t->snd_trk[2],
		   t->snd_trk[3], t->prv_tch);)

	/* pack in new touches */
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		if (t->tmp_trk[id] >= CY_NUM_TRK_ID)
			continue;

		if (!cyttsp_inlist(t->snd_trk, t->tmp_trk[id], &loc,
							CY_NUM_MT_TCH_ID)) {

			DBG(
			printk(KERN_INFO"%s: not in list t[%d]=%d, loc=%d\n",
				   __func__,
				   id, t->tmp_trk[id], loc);)

			if (cyttsp_next_avail_inlist(t->snd_trk, &loc,
							CY_NUM_MT_TCH_ID)) {
				loc &= CY_NUM_MT_TCH_ID - 1;
				t->snd_trk[loc] = t->tmp_trk[id];
				DBG(printk(KERN_INFO "%s: put in list s[%d]=%d"
					" t[%d]=%d\n", __func__,
					loc,
					t->snd_trk[loc], id, t->tmp_trk[id]);
				    )
			}
		} else {
			DBG(printk(KERN_INFO"%s: is in list s[%d]=%d "
				"t[%d]=%d loc=%d\n", __func__,
				id, t->snd_trk[id], id, t->tmp_trk[id], loc);)
		}
	}
	DBG(printk(KERN_INFO"%s: S2: s0=%d, s1=%d, s2=%d, s3=%d\n", __func__,
			t->snd_trk[0], t->snd_trk[1],
			t->snd_trk[2], t->snd_trk[3]);)

	/* sync motion event signals for each current touch */
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		/* z will either be 0 (NOTOUCH) or
		 * some pressure (TOUCH)
		 */
		DBG(printk(KERN_INFO "%s: MT0 prev[%d]=%d "
				"temp[%d]=%d send[%d]=%d\n",
				__func__, id, ts->prv_mt_tch[id],
				id, t->tmp_trk[id], id, t->snd_trk[id]);)

		if (t->snd_trk[id] < CY_NUM_TRK_ID) {
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
					t->cur_mt_z[t->snd_trk[id]]);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
					t->tool_width);
			input_report_abs(ts->input, ABS_MT_POSITION_X,
					t->cur_mt_pos[t->snd_trk[id]][CY_XPOS]);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,
					t->cur_mt_pos[t->snd_trk[id]][CY_YPOS]);
			if (mt_sync_func)
				mt_sync_func(ts->input);

			DBG2(printk(KERN_INFO"%s: MT1 -> TID:"
				"%3d X:%3d  Y:%3d  Z:%3d\n", __func__,
				t->snd_trk[id],
				t->cur_mt_pos[t->snd_trk[id]][CY_XPOS],
				t->cur_mt_pos[t->snd_trk[id]][CY_YPOS],
				t->cur_mt_z[t->snd_trk[id]]);)

		} else if (ts->prv_mt_tch[id] < CY_NUM_TRK_ID) {
			/* void out this touch */
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
							CY_NTCH);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
							t->tool_width);
			input_report_abs(ts->input, ABS_MT_POSITION_X,
				ts->prv_mt_pos[ts->prv_mt_tch[id]][CY_XPOS]);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,
				ts->prv_mt_pos[ts->prv_mt_tch[id]][CY_YPOS]);

			if (mt_sync_func)
				mt_sync_func(ts->input);

			DBG2(printk(KERN_INFO"%s: "
				"MT2->TID:%2d X:%3d Y:%3d Z:%3d liftoff-sent\n",
				__func__, ts->prv_mt_tch[id],
				ts->prv_mt_pos[ts->prv_mt_tch[id]][CY_XPOS],
				ts->prv_mt_pos[ts->prv_mt_tch[id]][CY_YPOS],
				CY_NTCH);)
		} else {
			/* do not stuff any signals for this
			 * previously and currently void touches
			 */
			DBG(printk(KERN_INFO"%s: "
				"MT3->send[%d]=%d - No touch - NOT sent\n",
				__func__, id, t->snd_trk[id]);)
		}
	}

	/* save current posted tracks to
	 * previous track memory */
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		ts->prv_mt_tch[id] = t->snd_trk[id];
		if (t->snd_trk[id] < CY_NUM_TRK_ID) {
			ts->prv_mt_pos[t->snd_trk[id]][CY_XPOS] =
					t->cur_mt_pos[t->snd_trk[id]][CY_XPOS];
			ts->prv_mt_pos[t->snd_trk[id]][CY_YPOS] =
					t->cur_mt_pos[t->snd_trk[id]][CY_YPOS];
			DBG(printk(KERN_INFO"%s: "
				"MT4->TID:%2d X:%3d Y:%3d Z:%3d save for prv\n",
				__func__, t->snd_trk[id],
				ts->prv_mt_pos[t->snd_trk[id]][CY_XPOS],
				ts->prv_mt_pos[t->snd_trk[id]][CY_YPOS],
				CY_NTCH);)
		}
	}
	memset(ts->act_trk, CY_NTCH, sizeof(ts->act_trk));
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		if (t->snd_trk[id] < CY_NUM_TRK_ID)
			ts->act_trk[t->snd_trk[id]] = CY_TCH;
	}
}

static void cyttsp_xy_worker(struct work_struct *work)
{
	struct cyttsp *ts = container_of(work, struct cyttsp, work);
	struct cyttsp_xydata xy_data;
	u8 id, tilt, rev_x, rev_y;
	struct cyttsp_track_data trc;
	s32 retval;

	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)
	/* get event data from CYTTSP device */
	retval = ttsp_read_block_data(ts, CY_REG_BASE,
				      sizeof(xy_data), &xy_data);

	if (retval < 0) {
		printk(KERN_ERR "%s: Error, fail to read device on host interface bus\n",
			__func__);
		goto exit_xy_worker;
	}

	/* touch extension handling */
	retval = ttsp_tch_ext(ts, &xy_data);

	if (retval < 0) {
		printk(KERN_ERR "%s: Error, touch extension handling\n",
			__func__);
		goto exit_xy_worker;
	} else if (retval > 0) {
		DBG(printk(KERN_INFO "%s: Touch extension handled\n",
			__func__);)
		goto exit_xy_worker;
	}

	/* provide flow control handshake */
	if (ts->irq) {
		if (ts->platform_data->use_hndshk) {
			u8 cmd;
			cmd = xy_data.hst_mode & CY_HNDSHK_BIT ?
				xy_data.hst_mode & ~CY_HNDSHK_BIT :
				xy_data.hst_mode | CY_HNDSHK_BIT;
			retval = ttsp_write_block_data(ts, CY_REG_BASE,
						       sizeof(cmd), (u8 *)&cmd);
		}
	}
	trc.cur_tch = GET_NUM_TOUCHES(xy_data.tt_stat);

	if (ts->platform_data->power_state == CY_IDLE_STATE)
		goto exit_xy_worker;
	else if (GET_BOOTLOADERMODE(xy_data.tt_mode)) {
		/* TTSP device has reset back to bootloader mode */
		/* reset driver touch history */
		bool timeout;
		DBG3(printk(KERN_INFO
			"%s: Bootloader detected; reset driver\n",
			__func__);)
		cyttsp_init_tch(ts);
		free_irq(ts->irq, ts);
		cyttsp_soft_reset(ts, &timeout);
		cyttsp_exit_bl_mode(ts);
		cyttsp_set_operational_mode(ts);
		goto exit_xy_worker;
	} else if (IS_LARGE_AREA(xy_data.tt_stat) == 1) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		DBG(printk(KERN_INFO "%s: Large area detected\n",
			__func__);)
	} else if (trc.cur_tch > CY_NUM_MT_TCH_ID) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		DBG(printk(KERN_INFO "%s: Num touch error detected\n",
			__func__);)
	} else if (IS_BAD_PKT(xy_data.tt_mode)) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		DBG(printk(KERN_INFO "%s: Invalid buffer detected\n",
			__func__);)
	}

	/* set tool size */
	trc.tool_width = CY_SMALL_TOOL_WIDTH;

	if (ts->platform_data->gen == CY_GEN2) {
		/* translate Gen2 interface data into comparable Gen3 data */
		trc.cur_tch = ttsp_convert_gen2(trc.cur_tch, &xy_data);
	}

	/* clear current active track ID array and count previous touches */
	for (id = 0, trc.prv_tch = CY_NTCH; id < CY_NUM_TRK_ID; id++) {
		trc.cur_trk[id] = CY_NTCH;
		trc.prv_tch += ts->act_trk[id];
	}

	/* send no events if there were no previous touches */
	/* and no new touches */
	if ((trc.prv_tch == CY_NTCH) && ((trc.cur_tch == CY_NTCH) ||
				(trc.cur_tch > CY_NUM_MT_TCH_ID)))
		goto exit_xy_worker;

	DBG(printk(KERN_INFO "%s: prev=%d  curr=%d\n", __func__,
		   trc.prv_tch, trc.cur_tch);)

	/* clear current single-touch array */
	memset(trc.cur_st_tch, CY_IGNR_TCH, sizeof(trc.cur_st_tch));

	/* clear single touch positions */
	trc.st_x1 = trc.st_y1 = trc.st_z1 =
			trc.st_x2 = trc.st_y2 = trc.st_z2 = CY_NTCH;

	/* clear current multi-touch arrays */
	memset(trc.cur_mt_tch, CY_IGNR_TCH, sizeof(trc.cur_mt_tch));
	memset(trc.cur_mt_pos, CY_NTCH, sizeof(trc.cur_mt_pos));
	memset(trc.cur_mt_z, CY_NTCH, sizeof(trc.cur_mt_z));

	DBG(
	if (trc.cur_tch) {
		unsigned i;
		u8 *pdata = (u8 *)&xy_data;

		printk(KERN_INFO "%s: TTSP data_pack: ", __func__);
		for (i = 0; i < sizeof(struct cyttsp_xydata); i++)
			printk(KERN_INFO "[%d]=0x%x ", i, pdata[i]);
		printk(KERN_INFO "\n");
	})

	/* Determine if display is tilted */
	tilt = !!FLIP_DATA(ts->platform_data->flags);
	/* Check for switch in origin */
	rev_x = !!REVERSE_X(ts->platform_data->flags);
	rev_y = !!REVERSE_Y(ts->platform_data->flags);

	/* process the touches */
	switch (trc.cur_tch) {
	case 4:
		xy_data.x4 = be16_to_cpu(xy_data.x4);
		xy_data.y4 = be16_to_cpu(xy_data.y4);
		if (tilt)
			FLIP_XY(xy_data.x4, xy_data.y4);

		if (rev_x)
			xy_data.x4 = INVERT_X(xy_data.x4,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.y4 = INVERT_X(xy_data.y4,
					ts->platform_data->maxy);

		id = GET_TOUCH4_ID(xy_data.touch34_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH4_IDX][CY_XPOS] = xy_data.x4;
			trc.cur_mt_pos[CY_MT_TCH4_IDX][CY_YPOS] = xy_data.y4;
			trc.cur_mt_z[CY_MT_TCH4_IDX] = xy_data.z4;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.x4;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.y4;
			trc.cur_mt_z[id] = xy_data.z4;
		}
		trc.cur_mt_tch[CY_MT_TCH4_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		if (ts->prv_st_tch[CY_ST_FNGR1_IDX] <	CY_NUM_TRK_ID) {
			if (ts->prv_st_tch[CY_ST_FNGR1_IDX] == id) {
				trc.st_x1 = xy_data.x4;
				trc.st_y1 = xy_data.y4;
				trc.st_z1 = xy_data.z4;
				trc.cur_st_tch[CY_ST_FNGR1_IDX] = id;
			} else if (ts->prv_st_tch[CY_ST_FNGR2_IDX] == id) {
				trc.st_x2 = xy_data.x4;
				trc.st_y2 = xy_data.y4;
				trc.st_z2 = xy_data.z4;
				trc.cur_st_tch[CY_ST_FNGR2_IDX] = id;
			}
		}
		DBG(printk(KERN_INFO"%s: 4th XYZ:% 3d,% 3d,% 3d  ID:% 2d\n\n",
				__func__, xy_data.x4, xy_data.y4, xy_data.z4,
				(xy_data.touch34_id & 0x0F));)

		/* do not break */
	case 3:
		xy_data.x3 = be16_to_cpu(xy_data.x3);
		xy_data.y3 = be16_to_cpu(xy_data.y3);
		if (tilt)
			FLIP_XY(xy_data.x3, xy_data.y3);

		if (rev_x)
			xy_data.x3 = INVERT_X(xy_data.x3,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.y3 = INVERT_X(xy_data.y3,
					ts->platform_data->maxy);

		id = GET_TOUCH3_ID(xy_data.touch34_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH3_IDX][CY_XPOS] = xy_data.x3;
			trc.cur_mt_pos[CY_MT_TCH3_IDX][CY_YPOS] = xy_data.y3;
			trc.cur_mt_z[CY_MT_TCH3_IDX] = xy_data.z3;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.x3;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.y3;
			trc.cur_mt_z[id] = xy_data.z3;
		}
		trc.cur_mt_tch[CY_MT_TCH3_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		if (ts->prv_st_tch[CY_ST_FNGR1_IDX] < CY_NUM_TRK_ID) {
			if (ts->prv_st_tch[CY_ST_FNGR1_IDX] == id) {
				trc.st_x1 = xy_data.x3;
				trc.st_y1 = xy_data.y3;
				trc.st_z1 = xy_data.z3;
				trc.cur_st_tch[CY_ST_FNGR1_IDX] = id;
			} else if (ts->prv_st_tch[CY_ST_FNGR2_IDX] == id) {
				trc.st_x2 = xy_data.x3;
				trc.st_y2 = xy_data.y3;
				trc.st_z2 = xy_data.z3;
				trc.cur_st_tch[CY_ST_FNGR2_IDX] = id;
			}
		}
		DBG(printk(KERN_INFO"%s: 3rd XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
			__func__, xy_data.x3, xy_data.y3, xy_data.z3,
			((xy_data.touch34_id >> 4) & 0x0F));)

		/* do not break */
	case 2:
		xy_data.x2 = be16_to_cpu(xy_data.x2);
		xy_data.y2 = be16_to_cpu(xy_data.y2);
		if (tilt)
			FLIP_XY(xy_data.x2, xy_data.y2);

		if (rev_x)
			xy_data.x2 = INVERT_X(xy_data.x2,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.y2 = INVERT_X(xy_data.y2,
					ts->platform_data->maxy);
		id = GET_TOUCH2_ID(xy_data.touch12_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH2_IDX][CY_XPOS] = xy_data.x2;
			trc.cur_mt_pos[CY_MT_TCH2_IDX][CY_YPOS] = xy_data.y2;
			trc.cur_mt_z[CY_MT_TCH2_IDX] = xy_data.z2;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.x2;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.y2;
			trc.cur_mt_z[id] = xy_data.z2;
		}
		trc.cur_mt_tch[CY_MT_TCH2_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		if (ts->prv_st_tch[CY_ST_FNGR1_IDX] <	CY_NUM_TRK_ID) {
			if (ts->prv_st_tch[CY_ST_FNGR1_IDX] == id) {
				trc.st_x1 = xy_data.x2;
				trc.st_y1 = xy_data.y2;
				trc.st_z1 = xy_data.z2;
				trc.cur_st_tch[CY_ST_FNGR1_IDX] = id;
			} else if (ts->prv_st_tch[CY_ST_FNGR2_IDX] == id) {
				trc.st_x2 = xy_data.x2;
				trc.st_y2 = xy_data.y2;
				trc.st_z2 = xy_data.z2;
				trc.cur_st_tch[CY_ST_FNGR2_IDX] = id;
			}
		}
		DBG(printk(KERN_INFO"%s: 2nd XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.x2, xy_data.y2, xy_data.z2,
				(xy_data.touch12_id & 0x0F));)

		/* do not break */
	case 1:
		xy_data.x1 = be16_to_cpu(xy_data.x1);
		xy_data.y1 = be16_to_cpu(xy_data.y1);
		if (tilt)
			FLIP_XY(xy_data.x1, xy_data.y1);

		if (rev_x)
			xy_data.x1 = INVERT_X(xy_data.x1,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.y1 = INVERT_X(xy_data.y1,
					ts->platform_data->maxy);

		id = GET_TOUCH1_ID(xy_data.touch12_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH1_IDX][CY_XPOS] = xy_data.x1;
			trc.cur_mt_pos[CY_MT_TCH1_IDX][CY_YPOS] = xy_data.y1;
			trc.cur_mt_z[CY_MT_TCH1_IDX] = xy_data.z1;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.x1;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.y1;
			trc.cur_mt_z[id] = xy_data.z1;
		}
		trc.cur_mt_tch[CY_MT_TCH1_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		if (ts->prv_st_tch[CY_ST_FNGR1_IDX] <	CY_NUM_TRK_ID) {
			if (ts->prv_st_tch[CY_ST_FNGR1_IDX] == id) {
				trc.st_x1 = xy_data.x1;
				trc.st_y1 = xy_data.y1;
				trc.st_z1 = xy_data.z1;
				trc.cur_st_tch[CY_ST_FNGR1_IDX] = id;
			} else if (ts->prv_st_tch[CY_ST_FNGR2_IDX] == id) {
				trc.st_x2 = xy_data.x1;
				trc.st_y2 = xy_data.y1;
				trc.st_z2 = xy_data.z1;
				trc.cur_st_tch[CY_ST_FNGR2_IDX] = id;
			}
		}
		DBG(printk(KERN_INFO"%s: S1st XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.x1, xy_data.y1, xy_data.z1,
				((xy_data.touch12_id >> 4) & 0x0F));)

		break;
	case 0:
	default:
		break;
	}

	if (ts->platform_data->use_st)
		handle_single_touch(&xy_data, &trc, ts);

	if (ts->platform_data->use_mt)
		handle_multi_touch(&trc, ts);

	/* handle gestures */
	if (ts->platform_data->use_gestures && xy_data.gest_id) {
		input_report_key(ts->input, BTN_3, CY_TCH);
		input_report_abs(ts->input, ABS_HAT1X, xy_data.gest_id);
		input_report_abs(ts->input, ABS_HAT1Y, xy_data.gest_cnt);
	}

	/* signal the view motion event */
	input_sync(ts->input);

	/* update platform data for the current multi-touch information */
	memcpy(ts->act_trk, trc.cur_trk, CY_NUM_TRK_ID);

exit_xy_worker:
	DBG(printk(KERN_INFO"%s: finished.\n", __func__);)
	return;
}

/* ************************************************************************
 * Probe initialization functions
 * ************************************************************************ */

static int cyttsp_check_polling(struct cyttsp *ts)
{
	return ts->platform_data->use_timer;
}

/* Timeout timer */
static void cyttsp_to_timer(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	DBG(printk(KERN_INFO"%s: TTSP timeout timer event!\n", __func__);)
	ts->to_timeout = true;
	return;
}

static void cyttsp_setup_to_timer(struct cyttsp *ts)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	setup_timer(&ts->to_timer, cyttsp_to_timer, (unsigned long) ts);
}

static void cyttsp_kill_to_timer(struct cyttsp *ts)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	del_timer(&ts->to_timer);
}

static void cyttsp_start_to_timer(struct cyttsp *ts, int ms)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	ts->to_timeout = false;
	mod_timer(&ts->to_timer, jiffies + ms);
}

static bool cyttsp_timeout(struct cyttsp *ts)
{
	if (cyttsp_check_polling(ts))
		return false;
	else
		return ts->to_timeout;
}

static irqreturn_t cyttsp_bl_ready_irq(int irq, void *handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	DBG2(printk(KERN_INFO"%s: Got BL IRQ!\n", __func__);)
	ts->bl_ready = true;
	return IRQ_HANDLED;
}

static void cyttsp_set_bl_ready(struct cyttsp *ts, bool set)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	ts->bl_ready = set;
	DBG(printk(KERN_INFO"%s: bl_ready=%d\n", __func__, (int)ts->bl_ready);)
}

static bool cyttsp_check_bl_ready(struct cyttsp *ts)
{
	if (cyttsp_check_polling(ts))
		return true;
	else
		return ts->bl_ready;
}

static int cyttsp_load_bl_regs(struct cyttsp *ts)
{
	int retval;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	retval =  ttsp_read_block_data(ts, CY_REG_BASE,
				sizeof(ts->bl_data), &(ts->bl_data));

	if (retval < 0) {
		DBG2(printk(KERN_INFO "%s: Failed reading block data, err:%d\n",
			__func__, retval);)
		goto fail;
	}

	DBG({
	      int i;
	      u8 *bl_data = (u8 *)&(ts->bl_data);
	      for (i = 0; i < sizeof(struct cyttsp_bootloader_data); i++)
			printk(KERN_INFO "%s bl_data[%d]=0x%x\n",
				__func__, i, bl_data[i]);
	})
	DBG2(printk(KERN_INFO "%s: bl f=%02X s=%02X e=%02X\n",
		__func__,
		ts->bl_data.bl_file,
		ts->bl_data.bl_status,
		ts->bl_data.bl_error);)

	return 0;
fail:
	return retval;
}

static bool cyttsp_bl_app_valid(struct cyttsp *ts)
{
	int retval;

	ts->bl_data.bl_status = 0x00;

	retval = cyttsp_load_bl_regs(ts);

	if (retval < 0)
		return false;

	if (ts->bl_data.bl_status == 0x11) {
		printk(KERN_INFO "%s: App found; normal boot\n", __func__);
		return true;
	} else if (ts->bl_data.bl_status == 0x10) {
		printk(KERN_INFO "%s: NO APP; load firmware!!\n", __func__);
		return false;
	} else {
		if (ts->bl_data.bl_file == CY_OPERATE_MODE) {
			int invalid_op_mode_status;
			invalid_op_mode_status = ts->bl_data.bl_status & 0x3f;
			if (invalid_op_mode_status)
				return false;
			else {
				if (ts->platform_data->power_state ==
					CY_ACTIVE_STATE)
					printk(KERN_INFO "%s: Operational\n",
						__func__);
				else
					printk(KERN_INFO "%s: No bootloader\n",
						__func__);
			}
		}
		return true;
	}
}

static bool cyttsp_bl_status(struct cyttsp *ts)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	return ((ts->bl_data.bl_status == 0x10) ||
		(ts->bl_data.bl_status == 0x11));
}

static bool cyttsp_bl_err_status(struct cyttsp *ts)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	return (((ts->bl_data.bl_status == 0x10) &&
		(ts->bl_data.bl_error == 0x20)) ||
		((ts->bl_data.bl_status == 0x11) &&
		(ts->bl_data.bl_error == 0x20)));
}

static bool cyttsp_wait_bl_ready(struct cyttsp *ts,
	int pre_delay, int loop_delay, int max_try,
	bool (*done)(struct cyttsp *ts))
{
	int tries;
	bool rdy = false, tmo = false;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	DBG(printk(KERN_INFO"%s: pre-dly=%d loop-dly=%d, max-try=%d\n",
		__func__, pre_delay, loop_delay, max_try);)

	tries = 0;
	ts->bl_data.bl_file = 0;
	ts->bl_data.bl_status = 0;
	ts->bl_data.bl_error = 0;
	if (cyttsp_check_polling(ts)) {
		msleep(pre_delay);
		do {
			msleep(abs(loop_delay));
			cyttsp_load_bl_regs(ts);
		} while (!done(ts) &&
			tries++ < max_try);
		DBG(printk(KERN_INFO"%s: polling mode tries=%d\n",
			__func__, tries);)
	} else {
		cyttsp_start_to_timer(ts, abs(loop_delay) * max_try);
		while (!rdy && !tmo) {
			rdy = cyttsp_check_bl_ready(ts);
			tmo = cyttsp_timeout(ts);
			if (loop_delay < 0)
				udelay(abs(loop_delay));
			else
				msleep(abs(loop_delay));
			tries++;
		}
		DBG2(printk(KERN_INFO"%s: irq mode tries=%d rdy=%d tmo=%d\n",
			__func__, tries, (int)rdy, (int)tmo);)
		cyttsp_load_bl_regs(ts);
	}

	if (tries >= max_try || tmo)
		return true;	/* timeout */
	else
		return false;
}

static int cyttsp_exit_bl_mode(struct cyttsp *ts)
{
	int retval;
	int tries = 0;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	/* check if in bootloader mode;
	 * if in operational mode then just return without fail
	 */
	cyttsp_load_bl_regs(ts);
	if (!GET_BOOTLOADERMODE(ts->bl_data.bl_status))
		return 0;

	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(bl_cmd),
				       (void *)bl_cmd);
	if (retval < 0) {
		printk(KERN_ERR "%s: Failed writing block data, err:%d\n",
			__func__, retval);
		goto fail;
	}
	do {
		msleep(20);
		cyttsp_load_bl_regs(ts);
	} while (GET_BOOTLOADERMODE(ts->bl_data.bl_status) && tries++ < 10);

	DBG2(printk(KERN_INFO "%s: read tries=%d\n", __func__, tries);)

	DBG(printk(KERN_INFO"%s: Exit "
				"f=%02X s=%02X e=%02X\n",
				__func__,
				ts->bl_data.bl_file, ts->bl_data.bl_status,
				ts->bl_data.bl_error);)

	return 0;
fail:
	return retval;
}

static int cyttsp_set_sysinfo_mode(struct cyttsp *ts)
{
	int retval;
	int tries;
	u8 cmd = CY_SYSINFO_MODE;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	memset(&(ts->sysinfo_data), 0, sizeof(struct cyttsp_sysinfo_data));

	/* switch to sysinfo mode */
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	if (retval < 0) {
		printk(KERN_ERR "%s: Failed writing block data, err:%d\n",
			__func__, retval);
		goto exit_sysinfo_mode;
	}

	/* read sysinfo registers */
	tries = 0;
	do {
		msleep(20);
		retval = ttsp_read_block_data(ts, CY_REG_BASE,
			sizeof(ts->sysinfo_data), &(ts->sysinfo_data));
	} while (!(retval < 0) &&
		(ts->sysinfo_data.tts_verh == 0) &&
		(ts->sysinfo_data.tts_verl == 0) &&
		(tries++ < (500/20)));

	DBG2(printk(KERN_INFO "%s: read tries=%d\n", __func__, tries);)

	if (retval < 0) {
		printk(KERN_ERR "%s: Failed reading block data, err:%d\n",
			__func__, retval);
		return retval;
	}

exit_sysinfo_mode:
	DBG(printk(KERN_INFO"%s:SI2: hst_mode=0x%02X mfg_cmd=0x%02X "
		"mfg_stat=0x%02X\n", __func__, ts->sysinfo_data.hst_mode,
		ts->sysinfo_data.mfg_cmd,
		ts->sysinfo_data.mfg_stat);)

	DBG(printk(KERN_INFO"%s:SI2: bl_ver=0x%02X%02X\n",
		__func__, ts->sysinfo_data.bl_verh, ts->sysinfo_data.bl_verl);)

	DBG(printk(KERN_INFO"%s:SI2: sysinfo act_intrvl=0x%02X "
		"tch_tmout=0x%02X lp_intrvl=0x%02X\n",
		__func__, ts->sysinfo_data.act_intrvl,
		ts->sysinfo_data.tch_tmout,
		ts->sysinfo_data.lp_intrvl);)

	printk(KERN_INFO"%s:SI2:tts_ver=0x%02X%02X app_id=0x%02X%02X "
		"app_ver=0x%02X%02X c_id=0x%02X%02X%02X\n", __func__,
		ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
		ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
		ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
		ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1],
		ts->sysinfo_data.cid[2]);

	return retval;
}

static int cyttsp_set_sysinfo_regs(struct cyttsp *ts)
{
	int retval = 0;

	if (ts->platform_data->scn_typ != CY_SCN_TYP_DFLT) {
		u8 scn_typ = ts->platform_data->scn_typ;

		retval = ttsp_write_block_data(ts,
				CY_REG_SCN_TYP,
				sizeof(scn_typ), &scn_typ);
	}

	if (retval < 0)
		return retval;

	if ((ts->platform_data->act_intrvl != CY_ACT_INTRVL_DFLT) ||
		(ts->platform_data->tch_tmout != CY_TCH_TMOUT_DFLT) ||
		(ts->platform_data->lp_intrvl != CY_LP_INTRVL_DFLT)) {

		u8 intrvl_ray[3];

		intrvl_ray[0] = ts->platform_data->act_intrvl;
		intrvl_ray[1] = ts->platform_data->tch_tmout;
		intrvl_ray[2] = ts->platform_data->lp_intrvl;

		/* set intrvl registers */
		retval = ttsp_write_block_data(ts,
				CY_REG_ACT_INTRVL,
				sizeof(intrvl_ray), intrvl_ray);

		msleep(CY_DELAY_SYSINFO);
	}

	return retval;
}

static int cyttsp_set_operational_mode(struct cyttsp *ts)
{
	int retval;
	int tries;
	u8 cmd = CY_OPERATE_MODE;
	u8 gest_default;
	struct cyttsp_xydata xy_data;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	memset(&(xy_data), 0, sizeof(xy_data));

	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	if (retval < 0) {
		printk(KERN_ERR "%s: Failed writing block data, err:%d\n",
			__func__, retval);
		goto write_block_data_fail;
	}

	/* wait for TTSP Device to complete switch to Operational mode */
	msleep(20);

	tries = 0;
	gest_default =
		CY_GEST_GRP1 + CY_GEST_GRP2 +
		CY_GEST_GRP3 + CY_GEST_GRP4 +
		CY_ACT_DIST_DFLT;
	do {
		msleep(20);
		retval = ttsp_read_block_data(ts, CY_REG_BASE,
			sizeof(xy_data), &(xy_data));
	} while (!(retval < 0) &&
		(xy_data.gest_set != gest_default) &&
		(tries++ < (500/20)));

	DBG2(printk(KERN_INFO "%s: read tries=%d\n", __func__, tries);)

	/* Touch Timer or Interrupt setup */
	if (ts->platform_data->use_timer) {
		mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
	} else {
		DBG2(printk(KERN_INFO
			"%s: Setting up Interrupt. Device name=%s\n",
			__func__, ts->platform_data->name);)
		retval = request_irq(ts->irq, cyttsp_irq,
			IRQF_TRIGGER_FALLING, ts->platform_data->name, ts);

		if (retval < 0) {
			printk(KERN_ERR "%s: Error, could not request irq\n",
				__func__);
			goto write_block_data_fail;
		} else {
			DBG2(printk(KERN_INFO "%s: Interrupt=%d\n",
				__func__, ts->irq);)
		}
	}
	return 0;
write_block_data_fail:
	return retval;
}

static int cyttsp_soft_reset(struct cyttsp *ts, bool *status)
{
	int retval;
	u8 cmd = CY_SOFT_RESET_MODE;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	/* reset TTSP Device back to bootloader mode */
	if (!cyttsp_check_polling(ts)) {
		DBG(printk(KERN_INFO
			"%s: Setting up BL Ready Interrupt. Device name=%s\n",
			__func__, ts->platform_data->name);)
		retval = request_irq(ts->irq, cyttsp_bl_ready_irq,
			IRQF_TRIGGER_FALLING, ts->platform_data->name, ts);
		cyttsp_setup_to_timer(ts);
	}

	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	/* place this after write to reset;
	 * better to sacrifice one than fail
	 * on false early indication
	 */
	cyttsp_set_bl_ready(ts, false);
	/* wait for TTSP Device to complete reset back to bootloader */
	if (!retval)
		*status = cyttsp_wait_bl_ready(ts, 300, 10, 100,
			cyttsp_bl_status);

	if (!cyttsp_check_polling(ts)) {
		cyttsp_kill_to_timer(ts);
		free_irq(ts->irq, ts);
	}

	return retval;
}

static int cyttsp_gesture_setup(struct cyttsp *ts)
{
	int retval;
	u8 gesture_setup;

	DBG(printk(KERN_INFO"%s: Init gesture; active distance setup\n",
		__func__);)

	gesture_setup = ts->platform_data->gest_set;
	retval = ttsp_write_block_data(ts, CY_REG_GEST_SET,
		sizeof(gesture_setup), &gesture_setup);

	return retval;
}

#ifdef CY_INCLUDE_LOAD_RECS
#include "cyttsp_ldr.h"
#else
static int cyttsp_loader(struct cyttsp *ts)
{
	void *fptr = cyttsp_bl_err_status;	/* kill warning */

	if (ts) {
		DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
		DBG(printk(KERN_INFO"%s: Exit\n", __func__);)
	}

	if (!fptr)
		return -EIO;
	else
		return 0;
}
#endif

static int cyttsp_power_on(struct cyttsp *ts)
{
	int retval = 0;
	bool timeout;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	ts->platform_data->power_state = CY_IDLE_STATE;

	/* check if the TTSP device has a bootloader installed */
	retval = cyttsp_soft_reset(ts, &timeout);
	DBG2(printk(KERN_INFO"%s: after softreset r=%d\n", __func__, retval);)
	if (retval < 0 || timeout)
		goto bypass;

	if (ts->platform_data->use_load_file)
		retval = cyttsp_loader(ts);

	if (!cyttsp_bl_app_valid(ts)) {
		retval = 1;
		goto bypass;
	}

	retval = cyttsp_exit_bl_mode(ts);
	DBG2(printk(KERN_INFO"%s: after exit bl r=%d\n", __func__, retval);)

	if (retval < 0)
		goto bypass;

	/* switch to System Information mode to read */
	/* versions and set interval registers */
	retval = cyttsp_set_sysinfo_mode(ts);
	if (retval < 0)
		goto bypass;

	retval = cyttsp_set_sysinfo_regs(ts);
	if (retval < 0)
		goto bypass;

	/* switch back to Operational mode */
	DBG2(printk(KERN_INFO"%s: switch back to operational mode\n",
		__func__);)
	retval = cyttsp_set_operational_mode(ts);
	if (retval < 0)
		goto bypass;

	/* init gesture setup; required for active distance */
	cyttsp_gesture_setup(ts);

bypass:
	if (!retval)
		ts->platform_data->power_state = CY_ACTIVE_STATE;

	printk(KERN_INFO"%s: Power state is %s\n",
		__func__, (ts->platform_data->power_state == CY_ACTIVE_STATE) ?
		"ACTIVE" : "IDLE");
	return retval;
}

static int cyttsp_resume(struct cyttsp *ts)
{
	int retval = 0;
	struct cyttsp_xydata xydata;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	if (ts->platform_data->use_sleep && (ts->platform_data->power_state !=
		CY_ACTIVE_STATE)) {
		if (ts->platform_data->wakeup) {
			retval = ts->platform_data->wakeup();
			if (retval < 0)
				printk(KERN_ERR "%s: Error, wakeup failed!\n",
					__func__);
		} else {
			printk(KERN_ERR "%s: Error, wakeup not implemented "
				"(check board file).\n", __func__);
			retval = -ENOSYS;
		}
		if (!(retval < 0)) {
			retval = ttsp_read_block_data(ts, CY_REG_BASE,
				sizeof(xydata), &xydata);
			if (!(retval < 0) && !GET_HSTMODE(xydata.hst_mode))
				ts->platform_data->power_state =
					CY_ACTIVE_STATE;
		}
	}
	DBG(printk(KERN_INFO"%s: Wake Up %s\n", __func__,
		(retval < 0) ? "FAIL" : "PASS");)
	return retval;
}

static int cyttsp_suspend(struct cyttsp *ts)
{
	u8 sleep_mode = 0;
	int retval = 0;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	if (ts->platform_data->use_sleep &&
			(ts->platform_data->power_state == CY_ACTIVE_STATE)) {
		sleep_mode = CY_DEEP_SLEEP_MODE;
		retval = ttsp_write_block_data(ts,
			CY_REG_BASE, sizeof(sleep_mode), &sleep_mode);
		if (!(retval < 0))
			ts->platform_data->power_state = CY_SLEEP_STATE;
	}
	DBG(printk(KERN_INFO"%s: Sleep Power state is %s\n", __func__,
		(ts->platform_data->power_state == CY_ACTIVE_STATE) ?
		"ACTIVE" :
		((ts->platform_data->power_state == CY_SLEEP_STATE) ?
		"SLEEP" : "LOW POWER"));)
	return retval;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp_ts_early_suspend(struct early_suspend *h)
{
	struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	LOCK(ts->mutex);
	if (!ts->fw_loader_mode) {
		if (ts->platform_data->use_timer)
			del_timer(&ts->timer);
		else
			disable_irq_nosync(ts->irq);
		ts->suspended = 1;
		cancel_work_sync(&ts->work);
		cyttsp_suspend(ts);
	}
	UNLOCK(ts->mutex);
}

static void cyttsp_ts_late_resume(struct early_suspend *h)
{
	struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	LOCK(ts->mutex);
	if (!ts->fw_loader_mode && ts->suspended) {
		ts->suspended = 0;
		if (cyttsp_resume(ts) < 0)
			printk(KERN_ERR "%s: Error, cyttsp_resume.\n",
				__func__);
		if (ts->platform_data->use_timer)
			mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
		else
			enable_irq(ts->irq);
	}
	UNLOCK(ts->mutex);
}
#endif

#ifdef CONFIG_HAS_FWLOADER
static int cyttsp_wr_reg(struct cyttsp *ts, u8 reg_id, u8 reg_data)
{

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	return ttsp_write_block_data(ts,
		CY_REG_BASE + reg_id, sizeof(u8), &reg_data);
}

static int cyttsp_rd_reg(struct cyttsp *ts, u8 reg_id, u8 *reg_data)
{
	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
	return ttsp_read_block_data(ts,
		CY_REG_BASE + reg_id, sizeof(u8), reg_data);
}

static ssize_t firmware_write(struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t pos, size_t size)
{
	unsigned short val;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp *ts = platform_get_drvdata(dev);
	LOCK(ts->mutex);
	if (!ts->fw_loader_mode) {
		val = *(unsigned short *)buf;
		ts->reg_id = (val & 0xff00) >> 8;
		if (!(ts->reg_id & 0x80)) {
			/* write user specified operational register */
			cyttsp_wr_reg(ts, ts->reg_id, (u8)(val & 0xff));
			DBG2(printk(KERN_INFO "%s: write(r=0x%02X d=0x%02X)\n",
				__func__, ts->reg_id, (val & 0xff));)
		} else {
			/* save user specified operational read register */
			DBG2(printk(KERN_INFO "%s: read(r=0x%02X)\n",
				__func__, ts->reg_id);)
		}
	} else {
		int retval = 0;
		int tries = 0;
		DBG({
			char str[128];
			char *p = str;
			int i;
			for (i = 0; i < size; i++, p += 2)
				sprintf(p, "%02x", (unsigned char)buf[i]);
			printk(KERN_DEBUG "%s: size %d, pos %ld payload %s\n",
			       __func__, size, (long)pos, str);
		})
		do {
			retval = ttsp_write_block_data(ts,
				CY_REG_BASE, size, buf);
			if (retval < 0)
				msleep(500);
		} while ((retval < 0) && (tries++ < 10));
	}
	UNLOCK(ts->mutex);
	return size;
}

static ssize_t firmware_read(struct kobject *kobj,
	struct bin_attribute *ba,
	char *buf, loff_t pos, size_t size)
{
	int count = 0;
	u8 reg_data;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp *ts = platform_get_drvdata(dev);

	DBG2(printk(KERN_INFO"%s: Enter (mode=%d)\n",
		__func__, ts->fw_loader_mode);)

	LOCK(ts->mutex);
	if (!ts->fw_loader_mode) {
		/* read user specified operational register */
		cyttsp_rd_reg(ts, ts->reg_id & ~0x80, &reg_data);
		*(unsigned short *)buf = reg_data << 8;
		count = sizeof(unsigned short);
		DBG2(printk(KERN_INFO "%s: read(d=0x%02X)\n",
			__func__, reg_data);)
	} else {
		int retval = 0;
		int tries = 0;

		do {
			retval = cyttsp_load_bl_regs(ts);
			if (retval < 0)
				msleep(500);
		} while ((retval < 0) && (tries++ < 10));

		if (retval < 0) {
			printk(KERN_ERR "%s: error reading status\n", __func__);
			count = 0;
		} else {
			*(unsigned short *)buf = ts->bl_data.bl_status << 8 |
				ts->bl_data.bl_error;
			count = sizeof(unsigned short);
		}

		DBG2(printk(KERN_INFO
			"%s:bl_f=0x%02X bl_s=0x%02X bl_e=0x%02X\n",
			__func__,
			ts->bl_data.bl_file,
			ts->bl_data.bl_status,
			ts->bl_data.bl_error);)
	}
	UNLOCK(ts->mutex);
	return count;
}

static struct bin_attribute cyttsp_firmware = {
	.attr = {
		.name = "firmware",
		.mode = 0644,
	},
	.size = 128,
	.read = firmware_read,
	.write = firmware_write,
};

static ssize_t attr_fwloader_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = platform_get_drvdata(dev);
	return sprintf(buf, "0x%02X%02X 0x%02X%02X 0x%02X%02X 0x%02X%02X%02X\n",
		ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
		ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
		ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
		ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1],
		ts->sysinfo_data.cid[2]);
}

static ssize_t attr_fwloader_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	char *p;
	int ret;
	bool timeout;
	struct cyttsp *ts = platform_get_drvdata(dev);
	unsigned val = simple_strtoul(buf, &p, 10);

	ret = p - buf;
	if (*p && isspace(*p))
		ret++;
	printk(KERN_DEBUG "%s: %u\n", __func__, val);

	LOCK(ts->mutex)
	if (val == 3) {
		sysfs_remove_bin_file(&dev->kobj, &cyttsp_firmware);
		DBG2(printk(KERN_INFO "%s: FW loader closed for reg r/w\n",
			__func__);)
	} else if (val == 2) {
		if (sysfs_create_bin_file(&dev->kobj, &cyttsp_firmware))
			printk(KERN_ERR "%s: unable to create file\n",
				__func__);
		DBG2(printk(KERN_INFO "%s: FW loader opened for reg r/w\n",
			__func__);)
	} else if ((val == 1) && !ts->fw_loader_mode) {
		ts->fw_loader_mode = 1;
		if (ts->suspended) {
			cyttsp_resume(ts);
		} else {
			if (ts->platform_data->use_timer)
				del_timer(&ts->timer);
			else
				disable_irq_nosync(ts->irq);
			cancel_work_sync(&ts->work);
		}
		ts->suspended = 0;
		if (sysfs_create_bin_file(&dev->kobj, &cyttsp_firmware))
			printk(KERN_ERR "%s: unable to create file\n",
				__func__);
		if (ts->platform_data->power_state == CY_ACTIVE_STATE)
			free_irq(ts->irq, ts);
		DBG2(printk(KERN_INFO
			"%s: FW loader opened for start load: ps=%d mode=%d\n",
			__func__,
			ts->platform_data->power_state, ts->fw_loader_mode);)
		cyttsp_soft_reset(ts, &timeout);
		printk(KERN_INFO "%s: FW loader started.\n", __func__);
	} else if (!val && ts->fw_loader_mode) {
		sysfs_remove_bin_file(&dev->kobj, &cyttsp_firmware);
		cyttsp_soft_reset(ts, &timeout);
		cyttsp_exit_bl_mode(ts);
		cyttsp_set_sysinfo_mode(ts);	/* update sysinfo rev data */
		cyttsp_set_operational_mode(ts);
		ts->platform_data->power_state = CY_ACTIVE_STATE;
		ts->fw_loader_mode = 0;
		printk(KERN_INFO "%s: FW loader finished.\n", __func__);
		cyttsp_bl_app_valid(ts);
	}
	UNLOCK(ts->mutex);
	return  ret == size ? ret : -EINVAL;
}

static struct device_attribute fwloader =
	__ATTR(fwloader, 0644, attr_fwloader_show, attr_fwloader_store);

#endif 		

static int cyttsp_probe(struct platform_device* pdev)
{
	struct input_dev *input_device;
	struct cyttsp *ts;
	int retval = 0;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "%s: Error, kzalloc\n", __func__);
		retval = -ENOMEM;
		goto error_alloc_data_failed;
	}
	mutex_init(&ts->mutex);
	ts->pdev = &pdev->dev;

	ts->platform_data = &cyttsp_i2c_platform_data;

	if (cyttsp_i2c_clientst == NULL) {
		printk(KERN_ERR "%s: Error, CYTTSP I2C Client not initialized \n",__func__);
		goto error_init;
	}
	ts->bus_ops = &cyttsp_i2c_clientst->ops;       

	if (ts->platform_data->init)
		retval = ts->platform_data->init(1);
	if (retval) {
		printk(KERN_ERR "%s: platform init failed! \n", __func__);
		goto error_init;
	}

	if (ts->platform_data->use_timer)
		ts->irq = -1;
	else {
		gpio_touchcntrl_request_irq(1);
		ts->irq = gpio_touchcntrl_irq();
	}

	/* Create the input device and register it. */
	input_device = input_allocate_device();
	if (!input_device) {
		retval = -ENOMEM;
		printk(KERN_ERR "%s: Error, failed to allocate input device\n",
			__func__);
		goto error_input_allocate_device;
	}

	ts->input = input_device;
	input_device->name = ts->platform_data->name;
	input_device->phys = ts->phys;
	input_device->dev.parent = ts->pdev;

	/* Prepare our worker structure prior to setting up the timer/ISR */
	INIT_WORK(&ts->work, cyttsp_xy_worker);

	if (ts->platform_data->use_timer) {
		DBG(printk(KERN_INFO "%s: Setting up Timer\n", __func__);)
		setup_timer(&ts->timer, cyttsp_timer, (unsigned long) ts);
	}

	retval = cyttsp_power_on(ts);
	if (retval < 0) {
		printk(KERN_ERR "%s: Error, power on failed! \n", __func__);
		goto error_power_on;
	}

	cyttsp_init_tch(ts);

	set_bit(EV_SYN, input_device->evbit);
	set_bit(EV_KEY, input_device->evbit);
	set_bit(EV_ABS, input_device->evbit);
	set_bit(BTN_TOUCH, input_device->keybit);
	set_bit(BTN_2, input_device->keybit);
	if (ts->platform_data->use_gestures)
		set_bit(BTN_3, input_device->keybit);

	input_set_abs_params(input_device, ABS_X, 0, ts->platform_data->maxx,
			     0, 0);
	input_set_abs_params(input_device, ABS_Y, 0, ts->platform_data->maxy,
			     0, 0);
	input_set_abs_params(input_device, ABS_TOOL_WIDTH, 0,
			     CY_LARGE_TOOL_WIDTH, 0, 0);
	input_set_abs_params(input_device, ABS_PRESSURE, 0, CY_MAXZ, 0, 0);
	input_set_abs_params(input_device, ABS_HAT0X, 0,
			     ts->platform_data->maxx, 0, 0);
	input_set_abs_params(input_device, ABS_HAT0Y, 0,
			     ts->platform_data->maxy, 0, 0);
	if (ts->platform_data->use_gestures) {
		input_set_abs_params(input_device, ABS_HAT1X, 0, CY_MAXZ,
				     0, 0);
		input_set_abs_params(input_device, ABS_HAT1Y, 0, CY_MAXZ,
				     0, 0);
	}
	if (ts->platform_data->use_mt) {
		input_set_abs_params(input_device, ABS_MT_POSITION_X, 0,
				     ts->platform_data->maxx, 0, 0);
		input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0,
				     ts->platform_data->maxy, 0, 0);
		input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 0,
				     CY_MAXZ, 0, 0);
		input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR, 0,
				     CY_LARGE_TOOL_WIDTH, 0, 0);
		if (ts->platform_data->use_trk_id)
			input_set_abs_params(input_device, ABS_MT_TRACKING_ID,
					0, CY_NUM_TRK_ID, 0, 0);
	}

	if (ts->platform_data->use_virtual_keys)
		input_set_capability(input_device, EV_KEY, KEY_PROG1);

	retval = input_register_device(input_device);
	if (retval) {
		printk(KERN_ERR "%s: Error, failed to register input device\n",
			__func__);
		goto error_input_register_device;
	}
	DBG(printk(KERN_INFO "%s: Registered input device %s\n",
		   __func__, input_device->name);)

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = cyttsp_ts_early_suspend;
	ts->early_suspend.resume = cyttsp_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef CONFIG_HAS_FWLOADER
	retval = device_create_file(&pdev->dev, &fwloader);	
	if (retval) {
		printk(KERN_ERR "%s: Error, could not create attribute\n",
			__func__);
		goto device_create_error;
	}
#endif

	platform_set_drvdata(pdev, ts);
	cyttsp_tsc = ts;
	return 0;

#ifdef CONFIG_HAS_FWLOADER
device_create_error:
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	input_unregister_device(input_device);
error_input_register_device:
	if (ts->irq >= 0)
		free_irq(ts->irq, ts);
error_power_on:
	cyttsp_kill_to_timer(ts);
	input_free_device(input_device);
error_input_allocate_device:
	gpio_touchcntrl_request_irq(0);
	if (ts->platform_data->init)
		ts->platform_data->init(0);
error_init:
	kfree(ts);
error_alloc_data_failed:
	return retval;
}

static int cyttsp_remove(struct platform_device *pdev)
{
	struct cyttsp *ts;
	
	ts = (struct cyttsp *) platform_get_drvdata(pdev);
	if (ts == NULL)
		return -ENODEV;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	cancel_work_sync(&ts->work);
	if (ts->platform_data->use_timer)
		del_timer_sync(&ts->timer);
	else {
		free_irq(ts->irq, ts);
		gpio_touchcntrl_request_irq(0);
	}
	input_unregister_device(ts->input);
	input_free_device(ts->input);
	if (ts->platform_data->init)
		ts->platform_data->init(0);
	kfree(ts);

	return 0;
}

static void cyttsp_shutdown(struct platform_device *pdev)
{
	//TBD 
}

static struct platform_driver cyttsp_driver =
{
	.driver = {
	  .name = CY_DRIVER_NAME,
	},
	.probe = cyttsp_probe,
	.shutdown = cyttsp_shutdown,
	.remove = cyttsp_remove,
};

static void cyttsp_nop_release(struct device* dev)
{
	/* Do Nothing */
}

static struct platform_device cyttsp_device =
{
	.name = CY_DRIVER_NAME,
	.id   = 0,
	.dev  = {
		.release = cyttsp_nop_release,
	},
};

static int  __init cyttsp_init(void)
{
	int ret = 0;
	
	i2c_probe_success = 0;
	ret = i2c_add_driver(&cyttsp_i2c_driver);
	if (ret < 0)
	{
		DEBUG_ERR(CY_ERR_I2C_ADD);		
		return -ENODEV;
	}
	if (!i2c_probe_success)
	{
		i2c_del_driver(&cyttsp_i2c_driver);
		return -ENODEV;
	}
	
	DEBUG_INFO("Registering platform device\n");
	platform_device_register(&cyttsp_device);
	platform_driver_register(&cyttsp_driver);
	return ret;
}

static void __exit cyttsp_exit(void)
{
	DEBUG_INFO("Calling exit");
	if (i2c_probe_success)
	{
		i2c_probe_success = 0;
		platform_driver_unregister(&cyttsp_driver);
		platform_device_unregister(&cyttsp_device);
	}
	i2c_del_driver(&cyttsp_i2c_driver);
}

module_init(cyttsp_init);
module_exit(cyttsp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver core");
MODULE_AUTHOR("Vidhyananth Venkatasamy <venkatas@lab126.com>");
