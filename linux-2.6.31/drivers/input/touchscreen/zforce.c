/*
 * Copyright (c) 2010 Amazon Technologies, Inc. All Rights Reserved.
 * Nadim Awad (nawad@lab126.com)
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/poll.h>
#include <linux/byteorder/generic.h>
#include <mach/boardid.h>

#include "zforceint.h"

static int reltest = 0;
#ifdef MODULE
module_param_named(reltest, reltest, int, S_IRUGO);
MODULE_PARM_DESC(reltest, "LED reliability test");
#endif

ZForceDrvData     zforce_data;    // Main data structure
wait_queue_head_t zf_wq;          // Wait queue
wait_queue_head_t zf_user_wq;     // Wait queue for user requested info
wait_queue_head_t zf_init_wq;     // Wait queue for starting and stopping driver
DEFINE_MUTEX(contactframe_mutex); // Contact frame mutex for testing
DEFINE_MUTEX(ledlvl_mutex);       // LED levels mutex
DEFINE_MUTEX(lowlvl_mutex);       // Low Level Signal mutex
int initSuccess;                  // Initialize successfull

static struct i2c_client *zforce_i2c_client; // I2C client
int i2c_probe_success;            // I2C probed successfully

// Protos
static void zforce_stop_touch(void);
static void zforce_restart_touch(void);
ZFStatus processData(u8 request, ZFStatus *result);
static void sendTouchUpdate(TouchData *tData);
ZFStatus sendLedLevelRequest(void);
ZFStatus sendForceCalibration(void);
ZFStatus sendPulseSignalInfo(char time, char strength);

/**** MISC DEVICE ****/
static long
zforce_misc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
  void __user *argp = (void __user *)arg;

  DEBUG_INFO("IN IOCTL HANDLING\n");
  switch(cmd) {
    case ZF_GET_LED_LVL:
    {
      DEFINE_WAIT(wait);

      if(!zforce_data.isConnected)
        return -1;
      
      DEBUG_INFO("Got IOCTL GET_LED_LVL\n");
      sendLedLevelRequest();
      if(!wait_event_timeout(zf_user_wq, zforce_data.ledlvl_ready != 0, msecs_to_jiffies(2000)))
      {
        DEBUG_ERR("Timeout getting led levels\n");
        return -1;
      }
      mutex_lock(&ledlvl_mutex);
      zforce_data.ledlvl_ready = 0;
      if (copy_to_user(argp, &zforce_data.ledLvl, sizeof(LedLevelResponse)))
      {
        mutex_unlock(&ledlvl_mutex);
        return -EFAULT;
      }
      mutex_unlock(&ledlvl_mutex);
      break;
    }
    case ZF_FORCE_CALIBRATION:
    {
      if (!zforce_data.isConnected)
        return -1;

      sendForceCalibration();
      if (!wait_event_timeout(zf_user_wq, zforce_data.force_calib_sent != 0, msecs_to_jiffies(2000)))
      {
        DEBUG_ERR("Timeout sending force calibration\n");
        return -1;
      }
      break;
    }
    case ZF_GET_RAW_DIODE_LVLS:
    {
      if (!zforce_data.raw_diode_ready)
      {
        FixedPulseStrengthResponse rawData;
        memset(&rawData, 0, sizeof(FixedPulseStrengthResponse));
        if (copy_to_user(argp, &rawData, sizeof(FixedPulseStrengthResponse)))
        {
          zforce_data.raw_diode_ready = 0;
          return -EFAULT;
        }
        break;
      }
      
      mutex_lock(&ledlvl_mutex);
      zforce_data.raw_diode_ready = 0;
      if (copy_to_user(argp, &zforce_data.rawDiode, sizeof(FixedPulseStrengthResponse)))
      {
        mutex_unlock(&ledlvl_mutex);
        return -EFAULT;
      }
      mutex_unlock(&ledlvl_mutex);
      break;
    }
    case ZF_GET_LOW_SIG_INFO:
    {
      if(!zforce_data.isConnected)
        return -1;
      DEBUG_INFO("Got IOCTL GET_LOW_LVL_INFO\n");
      mutex_lock(&lowlvl_mutex);
      if(copy_to_user(argp, &zforce_data.lowSig, sizeof(LowSignalAlert)))
      {
        mutex_unlock(&ledlvl_mutex);
        return -EFAULT;
      }
      mutex_unlock(&lowlvl_mutex);
      break;
    }
    case ZF_SET_FIXED_PULSE:
    {
      PulseSignalInfo pulseSignalInfo;
      if (!zforce_data.isConnected)
        return -EFAULT;

      if(copy_from_user(&pulseSignalInfo, argp, sizeof(PulseSignalInfo)))
        return -EFAULT;
      sendPulseSignalInfo(pulseSignalInfo.time, pulseSignalInfo.strength);   

      break; 
    }
    case ZF_SET_STATE_UPDATE:
    {
      if (zforce_data.state == ZF_STATE_UPDATE)
      {
        DEBUG_ERR(ZF_ERR_IN_UPDATE);
        return -EINVAL;
      }
      zforce_set_ready_for_update();
      break;
    }
    case ZF_SET_STATE_RUN:
    {
      if (zforce_data.state == ZF_STATE_RUN)
      {
        DEBUG_ERR(ZF_ERR_NOT_STOPPED);
        return -EINVAL;
      }
      zforce_restart_touch();
      break; 
    }
    default:
      DEBUG_ERR("Unknown ioctl provided\n");
      return -EINVAL;
      break;
  }
  return 0;
}

static ssize_t zforce_misc_write(struct file *file, const char __user *buf,
                                 size_t count, loff_t *pos)
{
  return 0;
}

static ssize_t zforce_misc_read(struct file *file, char __user *buf,
                                size_t count, loff_t *pos)
{
  return 0;
}


static const struct file_operations zforce_misc_fops =
{
  .owner = THIS_MODULE,
  .read  = zforce_misc_read,
  .write = zforce_misc_write,
  .unlocked_ioctl = zforce_misc_unlocked_ioctl,
};

static struct miscdevice zforce_misc_device =
{
  .minor = ZF_DEV_MINOR,
  .name  = ZF_DRIVER_NAME,
  .fops  = &zforce_misc_fops,
};

/**** END MISC DEVICE ****/

/* Externs */
extern int  gpio_neonode_irq(void);
extern void gpio_neonode_request_irq(int enable);
extern void gpio_neonode_bsl_init(void);
extern void gpio_neonode_set_bsl_reset(int val);
extern void gpio_neonode_set_bsl_test(int val);
extern bool gpio_neonode_datain(void);

/* Sysfs */
static ssize_t
zforce_connected_show(struct device* dev,
                      struct device_attribute* attr,
                      char* buf)
{
  return sprintf(buf, "%d\n", zforce_data.isConnected);
}
static DEVICE_ATTR(connected, 0444, zforce_connected_show, NULL);

static ssize_t
zforce_version_show(struct device* dev,
                    struct device_attribute* attr,
                    char* buf)
{
  return sprintf(buf, "%d.%db%dr%d\n", zforce_data.vData.major,
                                       zforce_data.vData.minor,
                                       zforce_data.vData.build,
                                       zforce_data.vData.revision);
}
static DEVICE_ATTR(version, 0444, zforce_version_show, NULL);

/* Reset and BSLReset hooks */
static ssize_t
zforce_bslreset_store(struct device* dev,
                      struct device_attribute* attr,
                      const char* buf,
                      size_t size)
{
  ZFState state;
  
  if (sscanf(buf, "%d", (int*)&state) <= 0)
    return -EINVAL;

  switch(state)
  {
    case(ZF_STATE_UPDATE):
      {
        if (zforce_data.state == ZF_STATE_UPDATE)
        {
          DEBUG_ERR(ZF_ERR_IN_UPDATE);
          break;
        }
        zforce_set_ready_for_update();
        break;
      }
    case(ZF_STATE_RUN):
      {
        if (zforce_data.state == ZF_STATE_RUN)
        {
          DEBUG_ERR(ZF_ERR_NOT_STOPPED);
          break;
        }
        zforce_restart_touch();
        break;
      }
    default:
      {
        DEBUG_ERR(ZF_ERR_UNKNOWN_STATE);
        break;
      }
  } 
  return size;
}

static ssize_t
zforce_bslreset_show(struct device* dev,
                     struct device_attribute* attr,
                     char* buf)
{
  return sprintf(buf, "%d\n", (int)zforce_data.state);
}
static DEVICE_ATTR(bslreset, 0644, zforce_bslreset_show, zforce_bslreset_store);

static ssize_t
zforce_mode_show(struct device* dev, struct device_attribute* attr, char* buf)
{
  return sprintf(buf, "%d\n", zforce_data.mode);
} 

// Switch between I2C and Test mode of operation
static ssize_t
zforce_mode_store(struct device* dev,
                  struct device_attribute* attr,
                  const char* buf,
                  size_t size)
{
  ZFMode mode;
  
  if (sscanf(buf, "%d", (int*)&mode) <= 0)
    return -EINVAL;
  if (mode == zforce_data.mode)
    return -EINVAL;
  if (mode > MODE_TEST)
  {
    DEBUG_ERR("Invalid mode of operation\n");
    return -EINVAL;
  }
  
  zforce_data.tData.count      = 0;
  zforce_data.tState.count     = 0;
  zforce_data.mode             = MODE_I2C;
  
  if (mode == MODE_TEST)
  {
    zforce_stop_touch();
    zforce_data.mode = mode;
  }
  else if (mode == MODE_I2C)
  {
    zforce_restart_touch();
  }
  
  return size;
} 
static DEVICE_ATTR(mode, 0644, zforce_mode_show, zforce_mode_store);

static ssize_t
zforce_test_show(struct device* dev,
                 struct device_attribute* attr,
                 char* buf)
{
  return sprintf(buf, "Num active contacts : %d\n", zforce_data.tData.count);
}

// Virtual contact creation
static ssize_t
zforce_test_store(struct device* dev,
                  struct device_attribute* attr,
                  const char*  buf,
                  size_t size)
{
  ZFTestEventType type;
  CoordinateData  contact;
  int coord_id;


  coord_id = MAX_CONTACTS;
  contact.X = -1;
  contact.Y = -1;

  if (zforce_data.mode != MODE_TEST)
  {
    DEBUG_ERR("Set the operational mode to Diagnostics in the mode /sys file\n");
    return -EINVAL;
  }

  if (sscanf(buf, "%d %d %hu %hu", (int*)&type,
                                    &coord_id,
                                    (unsigned short*)&contact.X,
                                    (unsigned short*)&contact.Y) <= 0)
    return -EINVAL;

  mutex_lock(&contactframe_mutex);
  
  switch(type)
  {
    case(TEST_CONTACT_SET):
    {
      if (coord_id >= MAX_CONTACTS)
      {
        DEBUG_ERR("Provided id is too high. Max value is %d\n", MAX_CONTACTS);
        mutex_unlock(&contactframe_mutex);
        return -EINVAL;
      }
      if (contact.X >= X_RESOLUTION || contact.X < 0 || 
          contact.Y >= Y_RESOLUTION || contact.Y < 0)
      {
        DEBUG_ERR("X and Y need to be between 0 and %d excluded\n", X_RESOLUTION);
        mutex_unlock(&contactframe_mutex);
        return -EINVAL;
      }
      contact.touch_state = TOUCH_MOVE;
      contact.id          = coord_id+1;
      zforce_data.tState.coordinateData[coord_id] = contact;
      DEBUG_INFO("Set Contact : [%d,%d,%d]\n", coord_id, contact.X, contact.Y);
      break;
    }
    case(TEST_CONTACT_REMOVE):
    {
      if (coord_id >= MAX_CONTACTS)
      {
        DEBUG_ERR("Provided id is too high. Max value is %d\n", MAX_CONTACTS);
        mutex_unlock(&contactframe_mutex);
        return -EINVAL;
      }
      zforce_data.tState.coordinateData[coord_id].touch_state = TOUCH_UP;
      DEBUG_INFO("Remove Contact : [%d]\n", coord_id);
      break;
    }
    case(TEST_CONTACT_CLEAR):
    {
      int i;
      for (i = 0; i < MAX_CONTACTS; i++)
        zforce_data.tState.coordinateData[i].touch_state = TOUCH_UP;
      zforce_data.tState.count = 0;
      DEBUG_INFO("Clear all contacts\n");
      break;
    }
    default: 
    {
      mutex_unlock(&contactframe_mutex);
      return -EINVAL;
    }
  }

  mutex_unlock(&contactframe_mutex);
  sendTouchUpdate(&zforce_data.tData);
  
  return size;
}
static DEVICE_ATTR(test, 0644, zforce_test_show, zforce_test_store);

/**** END Sysfs ****/

static void zforce_touch_work_func(struct work_struct *work)
{
  ZFStatus status;
 
  if (!reltest)
    processData(TYPE_TOUCH_DATA_RES, &status);
  else
    processData(TYPE_RAW_DIODE_DATA_RES, &status);
  enable_irq(zforce_data.irq);
}
DECLARE_WORK(zforce_touch_work, zforce_touch_work_func);

/* IRQ handler */
static irqreturn_t zforce_irq_handler(int irq, void *devid)
{
  DEBUG_INFO("******* Interrupt *******");
  disable_irq_nosync(zforce_data.irq);
  schedule_work(&zforce_touch_work);
  return IRQ_HANDLED;
}

static int zforce_touch_poll(void)
{
  int status = 0, retry = 20;
  do {
    status = gpio_neonode_datain();
    retry--;
    mdelay(20);
  } while (status == 1 && retry > 0);
  return (status == 0 ? 0 : -ETIMEDOUT);
}

static int register_input_device(struct platform_device *pdev)
{
  int i   = 0;
  int ret = 0;
  struct input_dev *tdev;

  tdev = input_allocate_device();
  if (!tdev)
  {
    DEBUG_ERR(ZF_ERR_NO_MEM_INPUT);
    return -ENOMEM;
  }

  tdev->name = ZF_INPUT_NAME;
  tdev->dev.parent = &pdev->dev;
  tdev->phys = "zforce/input0";

  __set_bit(EV_KEY, tdev->evbit);
  __set_bit(EV_ABS, tdev->evbit);
  __set_bit(BTN_TOUCH, tdev->keybit);
  __set_bit(BTN_TOOL_FINGER, tdev->keybit);
  __set_bit(BTN_TOOL_DOUBLETAP, tdev->keybit);
  __set_bit(BTN_TOOL_TRIPLETAP, tdev->keybit);
  __set_bit(ABS_X, tdev->keybit);
  __set_bit(ABS_Y, tdev->keybit);

#ifdef SLOTS_SUPPORT
  // Register input multi-touch slots
  input_mt_create_slots(tdev, MAX_CONTACTS);
  input_set_abs_params(tdev, ABS_MT_SLOT      ,  0, MAX_CONTACTS - 1,  0, 0);
#endif
  input_set_abs_params(tdev, ABS_MT_POSITION_X,  0, X_RESOLUTION    , 0, 0);
  input_set_abs_params(tdev, ABS_MT_POSITION_Y,  0, Y_RESOLUTION    , 0, 0);
  input_set_abs_params(tdev, ABS_X,              0, X_RESOLUTION    , 0, 0);
  input_set_abs_params(tdev, ABS_Y,              0, Y_RESOLUTION    , 0, 0);
  input_set_abs_params(tdev, ABS_MT_TRACKING_ID, 0, 255             , 0, 0);

  ret = input_register_device(tdev);
  if (ret < 0)
  {
    input_free_device(tdev);
    DEBUG_ERR(ZF_ERR_INPUT_MT);
    return ret;
  }
 
  // Make sure we clear the MT_SLOT state
  for(i = 0; i < MAX_CONTACTS; i++)
  {
    input_report_abs(tdev, ABS_MT_SLOT, i);
    input_report_abs(tdev, ABS_MT_TRACKING_ID, -1);
  }
  
  zforce_data.tdev = tdev;
  return 0;
}

static void remove_input_device(void)
{
  if (zforce_data.tdev)
  {
#ifdef SLOTS_SUPPORT
    input_mt_destroy_slots(zforce_data.tdev);
#endif
    input_unregister_device(zforce_data.tdev);
    input_free_device(zforce_data.tdev);
    zforce_data.tdev = NULL;
  }
}

/* BSL Update */
void bslReset(bool invokeBSL)
{
  DEBUG_INFO("Calling bslReset : %d\n", (int)invokeBSL);
  gpio_neonode_bsl_init();

  if (invokeBSL)
  {
    DEBUG_INFO("Triggering BSL\n");
    gpio_neonode_set_bsl_reset(0); /* RST  pin: GND */
    gpio_neonode_set_bsl_test(0);  /* TEST pin: GND */ msleep(250);
    gpio_neonode_set_bsl_test(1);  /* TEST pin: VCC */ msleep(10);
    gpio_neonode_set_bsl_test(0);  /* TEST pin: GND */ msleep(10);
    gpio_neonode_set_bsl_test(1);  /* TEST pin: VCC */ msleep(10);
    gpio_neonode_set_bsl_reset(1); /* RST  pin: VCC */ msleep(10);
    gpio_neonode_set_bsl_test(0);  /* TEST pin: GND */
  }
  else
  {
    DEBUG_INFO("Triggering RESET Only\n");
    gpio_neonode_set_bsl_reset(0); /* RST  pin: GND */
    msleep(10);
    gpio_neonode_set_bsl_reset(1); /* RST  pin: VCC */
  }
  msleep(250);
  DEBUG_INFO("RESET DONE\n");
}

/* I2C Setup */
static unsigned short normal_i2c[] = { ZFORCE_I2C_ADDRESS, I2C_CLIENT_END };
I2C_CLIENT_INSMOD;

struct zforce_info {
  struct i2c_client *client;
};

static int zforce_i2c_probe(struct i2c_client *client,
                       const struct i2c_device_id *id)
{
  struct zforce_info *info;
  
  DEBUG_INFO("Probing i2c\n");

  /* Reset the chip */
  bslReset(false);
  
  info = kzalloc(sizeof(*info), GFP_KERNEL);
  if (!info)
  {
    return -ENOMEM;
  }

  client->addr = ZFORCE_I2C_ADDRESS;
  i2c_set_clientdata(client, info);
  info->client = client;
  zforce_i2c_client = info->client;
  zforce_i2c_client->addr = ZFORCE_I2C_ADDRESS;
  DEBUG_INFO("Probing i2c done\n");
  i2c_probe_success = 1;
  return 0;
}

static int zforce_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
  DEBUG_INFO("I2C suspend\n");
  zforce_stop_touch();
  return 0;
}

static int zforce_i2c_resume(struct i2c_client *client)
{
  DEBUG_INFO("I2C resume\n");
  zforce_restart_touch();
  return 0;
}

static int zforce_i2c_remove(struct i2c_client *client)
{
  DEBUG_INFO("I2C Removing zforce\n");

  zforce_stop_touch();
  remove_input_device();
  
  DEBUG_INFO("I2C Done remove zforce\n");
  return 0;
}

static const struct i2c_device_id zforce_i2c_id[] =
{
  { ZF_DRIVER_NAME, 0 },
  { },
};

static struct i2c_driver zforce_i2c_driver =
{
  .driver = {
    .name = ZF_DRIVER_NAME,
  },
  .probe = zforce_i2c_probe,
  .remove = zforce_i2c_remove,
  .id_table = zforce_i2c_id,
  .suspend = zforce_i2c_suspend,
  .resume = zforce_i2c_resume,
};

static inline unsigned int zforce_read(u8* data, int len)
{
  struct i2c_msg xfer[1];
  int ret;

  xfer[0].addr = zforce_i2c_client->addr;
  xfer[0].flags = I2C_M_RD;
  xfer[0].len = len;
  xfer[0].buf = data;

  ret = i2c_transfer(zforce_i2c_client->adapter, xfer, 1);
  if (ret != 1 || ret < 0) {
    DEBUG_ERR(ZF_ERR_IO_READ);
    return 0;
  }

  return len;
}

static inline unsigned int zforce_write(u8*data, int len)
{
  struct i2c_msg xfer[1];
  int ret;

  xfer[0].addr = zforce_i2c_client->addr;
  xfer[0].flags = 0;
  xfer[0].len = len;
  xfer[0].buf = data;

  ret = i2c_transfer(zforce_i2c_client->adapter, xfer, 1);
  if (ret != 1 || ret < 0) {
    DEBUG_ERR(ZF_ERR_IO_WRITE);
    return 0;
  }

  return len;
}

int writeCommand(u8* command, u32 size)
{
  u32 i;
  int ret;
 
  for (i = 0; i < size; i++)
  {
    ret = i2c_smbus_write_byte(zforce_i2c_client, command[i]);
    if (ret < 0)
      return ret;
  }
  return i;
}

s32 readByte(void)
{
  s32 result;

  result = i2c_smbus_read_byte(zforce_i2c_client);
  if (result < 0)
  {
    DEBUG_ERR(ZF_ERR_I2C_READ_RETRY);
    return -EIO;
  }

  result = (u8)(result & 0xFF);
  return result;
}
/******* End I2C Setup *******/

#ifdef SLOTS_SUPPORT

static void send_user_event(u8 prev, u8 cur)
{
  struct input_dev *tdev = zforce_data.tdev;
  // No more touches. Tell user-space. 
  if (cur == 0 && prev > 0)
  {
    char *envp[] = {"ZForce=notouch", NULL};
    kobject_uevent_env(&tdev->dev.kobj, KOBJ_CHANGE, envp); 
  }
  else if (cur > 0 && prev == 0)
  {
    char *envp[] = {"ZForce=touch", NULL};
    kobject_uevent_env(&tdev->dev.kobj, KOBJ_CHANGE, envp); 
  }
}

static void sendTouchUpdate(TouchData *tData)
{
  u8 i;
  struct input_dev *tdev = zforce_data.tdev;
  u8 contacts_left = 0;
  TouchData *state = &zforce_data.tState;

  mutex_lock(&contactframe_mutex);
  
  DEBUG_INFO("Tdata->count : %d\n", tData->count);
  for(i = 0; i < tData->count; i++)
  {
    CoordinateData *coord = &(tData->coordinateData[i]);
    state->coordinateData[coord->id - 1] = *coord;
 
    DEBUG_INFO("TData : [%d] X:%d Y:%d S:%d\n", coord->id, coord->X, coord->Y, coord->touch_state);
  }
  
  DEBUG_INFO("Sending update\n");
  for(i = 0; i < MAX_CONTACTS; i++)
  {
    CoordinateData *coord = &state->coordinateData[i];
    int tState = coord->touch_state;

    DEBUG_INFO("[%d] X:%d Y:%d S:%d\n", coord->id, coord->X, coord->Y, coord->touch_state);
    
    // An update is pending for this touch. Report slot number
    if (tState != TOUCH_CLEAR)
      input_report_abs(tdev, ABS_MT_SLOT, coord->id - 1);

    if (tState == TOUCH_DOWN || tState == TOUCH_MOVE)
    {
      // Specify ID of touch the update applies to
      input_report_abs(tdev, ABS_MT_TRACKING_ID, coord->id - 1);
      
      if(mx50_board_is(BOARD_ID_YOSHI))
      {
        // Flip the coordinates on Yoshi
        input_report_abs(tdev, ABS_MT_POSITION_X, X_RESOLUTION - coord->X);
        input_report_abs(tdev, ABS_MT_POSITION_Y, Y_RESOLUTION - coord->Y);
      }
      else
      {
        input_report_abs(tdev, ABS_MT_POSITION_X, coord->X);
        input_report_abs(tdev, ABS_MT_POSITION_Y, coord->Y);
      }
      contacts_left++;
    }
    else if (tState == TOUCH_UP)
    {
      // Report touch up
      coord->touch_state = TOUCH_CLEAR;
      input_report_abs(tdev, ABS_MT_TRACKING_ID, -1);
      DEBUG_INFO("[ID:%d], GOT UP \n", coord->id);
    }
    else if (tState == TOUCH_CLEAR)
    {
      // Do nothing
    }
  }

  // Clear reading frame
  tData->count = 0;

  // Notify user space that there is new activity
  send_user_event(state->count, contacts_left);
  
  state->count = contacts_left;

  mutex_unlock(&contactframe_mutex);
  
  input_report_key(tdev, BTN_TOUCH, contacts_left > 0);
  input_report_key(tdev, BTN_TOOL_FINGER, contacts_left >= 1);
  input_report_key(tdev, BTN_TOOL_DOUBLETAP, contacts_left >= 2);
  input_report_key(tdev, BTN_TOOL_TRIPLETAP, contacts_left == 3);

  input_sync(tdev);
}

#else
/* Propagate touch events */
static void sendTouchUpdate(TouchData *tData)
{
  u8 i;
  struct input_dev *tdev = zforce_data.tdev;
  u8 contacts_left = 0;
  TouchData *state = &zforce_data.tState;
  
  mutex_lock(&contactframe_mutex);
  
  DEBUG_INFO("Tdata->count : %d\n", tData->count);
  for(i = 0; i < tData->count; i++)
  {
    CoordinateData *coord = &(tData->coordinateData[i]);
    state->coordinateData[coord->id - 1] = *coord;
  
    DEBUG_INFO("TData : [%d] X:%d Y:%d S:%d\n", coord->id, coord->X, coord->Y, coord->touch_state);
  }
  
  DEBUG_INFO("Sending update\n");
  for(i = 0; i < MAX_CONTACTS; i++)
  {
    CoordinateData *coord = &state->coordinateData[i];
    int tState = coord->touch_state;

    DEBUG_INFO("[%d] X:%d Y:%d S:%d\n", coord->id, coord->X, coord->Y, coord->touch_state);
    if (tState == TOUCH_DOWN || tState == TOUCH_MOVE)
    {
      if(mx50_board_is(BOARD_ID_YOSHI))
      {
        input_report_abs(tdev, ABS_MT_POSITION_X, X_RESOLUTION - coord->X);
        input_report_abs(tdev, ABS_MT_POSITION_Y, Y_RESOLUTION - coord->Y);
      }
      else
      {
        input_report_abs(tdev, ABS_MT_POSITION_X, coord->X);
        input_report_abs(tdev, ABS_MT_POSITION_Y, coord->Y);
      }
      input_report_abs(tdev, ABS_MT_TRACKING_ID, coord->id);
      input_mt_sync(tdev);
      contacts_left++;
    }
    else if (tState == TOUCH_UP)
    {
      coord->touch_state = TOUCH_CLEAR;
      DEBUG_INFO("[ID:%d], GOT UP \n", coord->id);
    }
    else if (tState == TOUCH_CLEAR)
    {
      // Do nothing
    }
  }

  // Clear reading frame
  tData->count = 0;
  state->count = contacts_left;
  mutex_unlock(&contactframe_mutex);

  input_report_key(tdev, BTN_TOUCH, contacts_left > 0);
  input_report_key(tdev, BTN_TOOL_FINGER, contacts_left >= 1);
  input_report_key(tdev, BTN_TOOL_DOUBLETAP, contacts_left >= 2);
  input_report_key(tdev, BTN_TOOL_TRIPLETAP, contacts_left == 3);

  input_sync(tdev);
}
#endif // SLOTS_SUPPORT

u32 readTouchData(unsigned char* data, TouchData *tData)
{
  u32 totalRead  = 0;
  u8  numTouches = 0;
  
  DEBUG_INFO("====== New Touch Data ======\n");
  numTouches = data[totalRead++];
  if (numTouches >= MAX_CONTACTS)
    numTouches = MAX_CONTACTS;
  tData->count = numTouches;

  
  DEBUG_INFO("Num touches: %d\n", numTouches);
  for (numTouches = 0; numTouches < tData->count; numTouches++)
  {
    CoordinateData* coord = &(tData->coordinateData[numTouches]);

    memcpy(&coord->X, &data[totalRead  ], sizeof(u16));
    memcpy(&coord->Y, &data[totalRead+2], sizeof(u16));
    coord->touch_state =  data[totalRead+4]        >> 6;
    coord->id          = (data[totalRead+4] & 0x3C) >> 2;
    coord->probability =  data[totalRead+6];
    
    totalRead += sizeof(CoordinateData);
    DEBUG_INFO("X:%d __ Y:%d __ State:%d __ ID:%d\n",
                coord->X, coord->Y, coord->touch_state, coord->id);
  }
  DEBUG_INFO("====== End Touch Data =====\n");
  return totalRead;
}

u32 readLowSignalAlert(unsigned char* data, LowSignalAlert* lsalert)
{
  u32 totalRead = 0;
  u8  count     = 0;
  
  lsalert->xCount = data[totalRead++];
  if (lsalert->xCount > MAX_X_LED_COUNT)
  {
    DEBUG_ERR(ZF_ERR_X_LED);
    return totalRead;
  }
  lsalert->yCount = data[totalRead++];
  if (lsalert->yCount > MAX_Y_LED_COUNT)
  {
    DEBUG_ERR(ZF_ERR_Y_LED);
    return totalRead;
  }
  for (count = 0; count < lsalert->xCount; count++)
  {
    lsalert->xLEDS[count].PDSignal1Low = (data[totalRead]   & 0xF0) >> 4;
    lsalert->xLEDS[count].PDSignal2Low = (data[totalRead++] & 0x0F);
  }
  for (count = 0; count < lsalert->yCount; count++)
  {
    lsalert->yLEDS[count].PDSignal1Low = (data[totalRead]   & 0xF0) >> 4;
    lsalert->yLEDS[count].PDSignal2Low = (data[totalRead++] & 0x0F);
  }
  return totalRead;
}

u32 readLedLevelResponse(unsigned char* data, LedLevelResponse* lvl)
{
  u32 totalRead = 0;
  u8  count     = 0;

  lvl->xCount = data[totalRead++];
  lvl->yCount = data[totalRead++];
  for (count = 0; count < lvl->xCount; count++)
  {
    lvl->xLEDS[count].LedStrength1 = (data[totalRead]   & 0xF0) >> 4;
    lvl->xLEDS[count].LedStrength2 = (data[totalRead++] & 0x0F);
    lvl->xLEDS[count].PDSignal1    = (data[totalRead++]       );
    lvl->xLEDS[count].PDSignal2    = (data[totalRead++]       );
  }
  for (count = 0; count < lvl->yCount; count++)
  {
    lvl->yLEDS[count].LedStrength1 = (data[totalRead]   & 0xF0) >> 4;
    lvl->yLEDS[count].LedStrength2 = (data[totalRead++] & 0x0F);
    lvl->yLEDS[count].PDSignal1    = (data[totalRead++]       );
    lvl->yLEDS[count].PDSignal2    = (data[totalRead++]       );
  }
  return totalRead;
}

u32 readRawDiodeDataResponse(unsigned char *data, RawDiodeData* rawData)
{
  u32 totalRead = 0;
  u8  count     = 0;

  rawData->NumberOfPulses = data[totalRead++];
  rawData->xCount = data[totalRead++];
  rawData->yCount = data[totalRead++];
  for (count = 0; count < rawData->xCount*2; count++)
    rawData->xValues[count] = data[totalRead++];
  for (count = 0; count < rawData->yCount*2; count++)
    rawData->yValues[count] = data[totalRead++];
  return totalRead;
}

u32 readFixedPulseResponse(unsigned char *data, FixedPulseStrengthResponse* rawData)
{
  u32 totalRead = 0;
  u8  count     = 0;

  rawData->xCount = data[totalRead++];
  rawData->yCount = data[totalRead++];
  for (count = 0; count < rawData->xCount; count++)
    rawData->xValues[count] = data[totalRead++];
  for (count = 0; count < rawData->yCount; count++)
    rawData->yValues[count] = data[totalRead++];
  return totalRead;
}

/* Process device response */
ZFStatus processData(u8 request, ZFStatus *result)
{
  u8       *data     = NULL;
  u8        type     = INVALID_COMMAND_RES;
  u8        dataSize = 0;
  int       dataRead = 0;
  int       retry    = 0; 
  u8        frameHeader[2];
  
  *result = ZF_NO_RESPONSE;

  DEBUG_INFO("Request is : 0x%x\n", request);
  do
  {
    retry++;
    if (retry > 2)
    {
      DEBUG_ERR("Max retries reached. Error getting frame start\n");
      return ZF_ERROR;  
    }
    
    if(!zforce_read(frameHeader, 2))
    {
      DEBUG_ERR("Error reading frame header\n");
      DEBUG_ERR(ZF_ERR_IO_READ);
      DEBUG_ERR("Retrying...\n");
      msleep(10);
      continue;
    }
  }
  while(frameHeader[0] != FRAME_START && retry <= 2);

  if (frameHeader[0] != FRAME_START)
  {
    DEBUG_ERR("Error reading Start frame: 0x%x 0x%x\n", frameHeader[0], frameHeader[1]);
    DEBUG_ERR(ZF_ERR_IO_READ);
    return ZF_ERROR;
  }

  dataSize = frameHeader[1];
  DEBUG_INFO("Data size : %d\n", dataSize);
  
  data = kzalloc(dataSize * sizeof(u8), GFP_KERNEL);
  if (!data)
  {
    DEBUG_ERR(ZF_ERR_ALLOC);
    return ZF_ERROR;
  }

  if (!zforce_read(data, (int)dataSize))
  {

    DEBUG_ERR(ZF_ERR_IO_READ);
    kfree(data);
    return ZF_ERROR;
  }

  while(dataRead < dataSize)
  {
    type = data[dataRead++];
    DEBUG_INFO("Request: 0x%02x __ Type received: 0x%02x\n", request, type);
    switch(type)
    {
      case(TYPE_DEACTIVATE_RES):
        {
          DeactivateResult res;
          DEBUG_INFO("Got DeActivate response\n");
          memcpy(&res, &data[dataRead], sizeof(DeactivateResult));
          dataRead += sizeof(DeactivateResult);
          if (request == TYPE_DEACTIVATE_RES)
            *result = (res.result == SUCCESS_RES) ? ZF_OK : ZF_ERROR;
          break;
        }
      case(TYPE_ACTIVATE_RES):
        {
          ActivateResult res;
          DEBUG_INFO("Got Activate response\n");
          memcpy(&res, &data[dataRead], sizeof(ActivateResult));
          dataRead += sizeof(ActivateResult);
          if (type == TYPE_ACTIVATE_RES)
            *result = (res.result == SUCCESS_RES) ? ZF_OK : ZF_ERROR;
          
          break;
        }
      case(TYPE_SET_RESOLUTION_RES):
        {
          SetResolutionResult res;
          DEBUG_INFO("Got SetResolution response\n");
          memcpy(&res, &data[dataRead], sizeof(SetResolutionResult));
          dataRead += sizeof(SetResolutionResult);
          if (request == TYPE_SET_RESOLUTION_RES)
            *result = (res.result == SUCCESS_RES) ? ZF_OK : ZF_ERROR;
          break;
       }
      case(TYPE_SET_CONFIGURATION_RES):
        {
          SetConfigurationResult res;
          DEBUG_INFO("Got SetConfiguration response\n");
          memcpy(&res, &data[dataRead], sizeof(SetConfigurationResult));
          dataRead += sizeof(SetConfigurationResult);
          if (request == TYPE_SET_CONFIGURATION_RES)
            *result = (res.result == SUCCESS_RES) ? ZF_OK : ZF_ERROR;
          break;
        }
      case(TYPE_TOUCH_DATA_RES):
        {
          TouchData *tData = &zforce_data.tData;
          dataRead += readTouchData(&data[dataRead], tData);
          if (request == TYPE_TOUCH_DATA_RES)
            *result = ZF_OK;
          sendTouchUpdate(tData);
          break;
        }
      case(TYPE_SCANNING_FREQ_RES):
        {
          SetScanningFrequencyResult res;
          DEBUG_INFO("Got ScanningFreq response\n");
          memcpy(&res, &data[dataRead], sizeof(SetScanningFrequencyResult));
          dataRead += sizeof(SetScanningFrequencyResult);
          if (request == TYPE_SCANNING_FREQ_RES)
            *result = (res.result == SUCCESS_RES) ? ZF_OK : ZF_ERROR;
          break;
        }
      case(TYPE_VERSION_RES):
        {
          VersionData *vData = &zforce_data.vData;
          memcpy(vData, &data[dataRead], sizeof(VersionData));
          dataRead += sizeof(VersionData);
          if (request == TYPE_VERSION_RES)
            *result = ZF_OK;
          DEBUG_INFO("Version: Ma: %d __ Mi: %d __ B: %d __ R: %d\n",
              vData->major, vData->minor, vData->build, vData->revision);
          break;
        }
      case(TYPE_LED_LEVEL_RES):
        {
          LedLevelResponse *ledLvl = &zforce_data.ledLvl;
          DEBUG_INFO("Got LedLevel response\n");
          mutex_lock(&ledlvl_mutex);
          zforce_data.ledlvl_ready = 1;
          dataRead += readLedLevelResponse(&data[dataRead], ledLvl);
          if (request == TYPE_LED_LEVEL_RES)
            *result = ZF_OK;
          wake_up(&zf_user_wq);
          mutex_unlock(&ledlvl_mutex);
          break;
        }
      case(TYPE_LOW_SIGNAL_ALERT):
        {
          LowSignalAlert *lowSig = &zforce_data.lowSig;
          char *envp[] = {"ZForce=low_signal", NULL};
          DEBUG_INFO("Reading Low Signal Alert\n");
          
          dataRead += readLowSignalAlert(&data[dataRead], lowSig);
          kobject_uevent_env(&zforce_data.tdev->dev.kobj, KOBJ_CHANGE, envp);
          if (request == TYPE_LOW_SIGNAL_ALERT)
            *result = ZF_OK;
          break;
        }
      case(TYPE_REL_FRAME_RESP_NUM_RES):
        {
          SetRelFrameRespNumResult res;
          DEBUG_INFO("Got reliability set frame number response\n");
          memcpy(&res, &data[dataRead], sizeof(SetRelFrameRespNumResult));
          dataRead += sizeof(SetRelFrameRespNumResult);
          if (request == TYPE_REL_FRAME_RESP_NUM_RES)
            *result = (res.result == SUCCESS_RES) ? ZF_OK : ZF_ERROR;
          break;
        }
      case(TYPE_FIXED_PULSE_STR_RES):
        {
          FixedPulseStrengthResponse *res = &zforce_data.rawDiode;
          DEBUG_INFO("Got Fixed pulse strength result\n");
          mutex_lock(&ledlvl_mutex);
          dataRead += readFixedPulseResponse(&data[dataRead], res);
          zforce_data.raw_diode_ready = 1;
          mutex_unlock(&ledlvl_mutex);
          *result = ZF_OK;
          break;
        }
      default:
        {
          DEBUG_INFO("Unknown type received... flushing : 0x%02X\n", type);
          dataRead = dataSize;
          break;
        } 
    }
  }
  DEBUG_INFO("In process Data. Result is : %d\n", *result);
  DEBUG_INFO("Status is : %d\n", zforce_data.state);
  kfree(data);
  return ZF_OK; 
}

ZFStatus sendActivate(void)
{
  ZFStatus result = ZF_OK;
  u8 command[1];
  
  DEBUG_INFO("Sending Activate");
  command[0] = ACTIVATE_CMD;

  if (writeCommand(command, 1) < 0)
  {
    DEBUG_ERR(ZF_ERR_IO_WRITE);
    return ZF_ERROR;
  }
 
  if (zforce_touch_poll())
    return ZF_ERROR;

  if (processData(ACTIVATE_CMD, &result) != ZF_OK)
    return result;

  DEBUG_INFO("Activate SUCCESS\n"); 
  return result;
}

ZFStatus sendDeactivate(void)
{
  ZFStatus result = ZF_OK;
  u8 command[1];

  DEBUG_INFO("Sending deactivate\n");
  command[0] = DEACTIVATE_CMD;
 
  if (writeCommand(command, 1) < 0)
  {
    DEBUG_ERR(ZF_ERR_IO_WRITE);
    return ZF_ERROR;
  }
  
  if (zforce_touch_poll())
    return ZF_ERROR;

  if (processData(DEACTIVATE_CMD, &result) != ZF_OK)
    return result;
  
  DEBUG_INFO("Deactivate SUCCESS\n");
  return result;
}

ZFStatus sendSetResolution(u16 X, u16 Y)
{
  ZFStatus result = ZF_OK;
  u8 command[5];
 
  DEBUG_INFO("Setting Resolution\n");
  command[0] = SET_RESOLUTION_CMD;
  memcpy(&command[1], &X, sizeof(X));
  memcpy(&command[3], &Y, sizeof(Y));

  if (!zforce_write(command, 5))
  {
    DEBUG_ERR(ZF_ERR_SET_RESOLUTION);
    return ZF_ERROR;
  }
  
  if (zforce_touch_poll())
    return ZF_ERROR;

  if (processData(SET_RESOLUTION_CMD, &result) != ZF_OK)
    return result;

  DEBUG_INFO("SetResolution SUCCESS\n");
  return result;
}

ZFStatus sendVersionRequest(void)
{
  ZFStatus result = ZF_OK;
  u8 command[1];
  
  DEBUG_INFO("Requesting Version\n");
  command[0] = GET_VERSION_CMD;
  
  if (writeCommand(command, 1) < 0)
  {
    DEBUG_ERR(ZF_ERR_IO_WRITE);
    return ZF_ERROR;
  }

  if (zforce_touch_poll())
    return ZF_ERROR;

  if (processData(GET_VERSION_CMD, &result) != ZF_OK)
    return result;
  
  DEBUG_INFO("Request Version SUCCESS\n");
  return result;
}

ZFStatus sendSetConfiguration(u32 flag)
{
  ZFStatus result = ZF_OK;
  ConfigurationData config;
  u8 command[5];
  
  DEBUG_INFO("Requesting SetConfiguration\n");
  config.flags = flag;
  command[0] = SET_CONFIGURATION_CMD;
  memcpy(&command[1], &config, sizeof(ConfigurationData));
  
  if(!zforce_write(command, 5))
  {
    DEBUG_ERR(ZF_ERR_SET_CONFIGURATION);
    return ZF_ERROR;
  }
  
  if (zforce_touch_poll())
    return ZF_ERROR;

  if (processData(SET_CONFIGURATION_CMD, &result) != ZF_OK)
    return result;
  
  DEBUG_INFO("Request SetConfiguration SUCCESS\n");
  return result;
}

ZFStatus sendSetScanningFrequency(u8 idle, u8 full)
{
  ZFStatus result = ZF_OK;
  u8 command[3];

  command[0] = SCANNING_FREQ_CMD;
  command[1] = idle;
  command[2] = full;
 
  DEBUG_INFO("Setting Scanning Frequency\n");
 
  if (!zforce_write(command, 3))
  {
    DEBUG_ERR(ZF_ERR_SET_SCAN_FREQ);
    return ZF_ERROR;
  }
 
  if (zforce_touch_poll())
    return ZF_ERROR;

  if (processData(SCANNING_FREQ_CMD, &result) != ZF_OK)
    return result;
  
  DEBUG_INFO("Set Scanning Frequency SUCCESS\n");
  return result;
}

ZFStatus sendLedLevelRequest(void)
{
  ZFStatus result = ZF_OK;
  u8 command[1];
 
  DEBUG_INFO("Sending LedLevelRequest\n");
  command[0] = LED_LEVEL_CMD;
 
  if (writeCommand(command, 1) < 0)
  {
    DEBUG_ERR(ZF_ERR_IO_WRITE);
    return ZF_ERROR;
  }
 
  DEBUG_INFO("LedLevelRequest SUCCESS\n");
  return result;
}

ZFStatus sendTouchDataRequest(void)
{
  u8 command[1];
 
  DEBUG_INFO("Sending initial Touch data request\n");
  command[0] = TOUCH_DATA_CMD;
 
  if (writeCommand(command, 1) < 0)
  {
    DEBUG_ERR(ZF_ERR_IO_WRITE);
    return ZF_IO_ERROR;
  }

  return ZF_OK;
}

ZFStatus sendSetFrameResponseNumber(u16 time)
{         
  ZFStatus result;
  u8 command[3];
        
  DEBUG_INFO("Setting Frame Response Number\n");
  command[0] = SET_REL_FRAME_RESP_NUM_CMD;
  memcpy(&command[1], &time, sizeof(time));
 
  DEBUG_INFO("Set Frame Response Number");
  if (!zforce_write(command, 3))
  {
    DEBUG_ERR(ZF_ERR_SET_FRAME_RESP);
    return ZF_ERROR;
  }

  if (zforce_touch_poll())
    return ZF_ERROR;

  if (processData(SET_REL_FRAME_RESP_NUM_CMD, &result) != ZF_OK)
    return ZF_ERROR;
  
  DEBUG_INFO("SetFrameResponseNumber SUCCESS\n");
  return ZF_OK;
}

ZFStatus sendPulseSignalInfo(char time, char strength)
{
  PulseSignalInfo sig;
  u8 command[2];
 
  command[0] = FIXED_PULSE_STR_CMD;
  
  sig.strength = strength;
  sig.time     = time;
  sig.reserved = 0;

  printk(KERN_INFO "SIG : Strength : %d __ Time : %d\n", sig.strength, sig.time);
  memcpy(&command[1], &sig, sizeof(PulseSignalInfo));
 
  if (!zforce_write(command, 2))
  {
    DEBUG_ERR(ZF_ERR_SET_PULSE_SIG_INFO);
    return ZF_ERROR;
  }

  return ZF_OK;
}

ZFStatus sendForceCalibration(void)
{
  u8 command[1];

  DEBUG_INFO("Sending Force Calibration\n");
  command[0] = FORCE_CALIBRATION_CMD;

  if (writeCommand(command, 1) < 0)
  {
    DEBUG_ERR(ZF_ERR_IO_WRITE);
    return ZF_IO_ERROR;
  }

  zforce_data.force_calib_sent = 1;
  wake_up(&zf_user_wq);
  
  DEBUG_INFO("Force Calibration SUCCESS\n");
  return ZF_OK;
}

ZFStatus initialize(void)
{
  ZFStatus status = ZF_OK;

  DEBUG_INFO("Initialize...");
   
  status = sendActivate();
  if (status != ZF_OK)
    return status;
  if (reltest)
  { 
    DEBUG_INFO("Set Frame Response Number");
    status = sendSetFrameResponseNumber(REL_RESPONSE_TIME);
    if (status != ZF_OK)
      return status;

    DEBUG_INFO("Set Signal Strength and Time");
    sendPulseSignalInfo(zforce_data.relTime, zforce_data.relStrength);
  }
  else
  {
    status = sendSetResolution(X_RESOLUTION, Y_RESOLUTION);
    if (status != ZF_OK)
      return status;
    status = sendVersionRequest();
    if (status != ZF_OK)
      return status;
    status = sendSetScanningFrequency(ZF_IDLE_FREQ, ZF_FULL_FREQ);
    if (status != ZF_OK)
      return status;
    status = sendSetConfiguration(ZFORCE_CONFIG_FLAG);
    if (status != ZF_OK)
      return status;
  }
  status = sendTouchDataRequest();
  return status; 
}

ZFStatus stop(void)
{
  int ret = 0;
 
  if (zforce_data.mode == MODE_I2C && sendDeactivate() != ZF_OK)
  {
    DEBUG_ERR("Could not send deactivate\n");
    ret = ZF_ERROR;
  }
  ret = ZF_OK;
  return ret;
}

static void zforce_work_handler(struct work_struct *unused)
{
  ZFStatus  status        = ZF_OK;
  int       ret           = 0;
  zforce_data.state       = ZF_STATE_RUN;

  if ((status = initialize()) != ZF_OK)
  {
    if (status == ZF_TIMEOUT)
      DEBUG_ERR(ZF_ERR_TIMEOUT);
    DEBUG_ERR(ZF_ERR_INIT);

    goto out_free_irq;
  }

  if (!zforce_data.irq)
  {
    /* Set the IRQ */
    gpio_neonode_request_irq(1); // Enable IRQ

    zforce_data.irq = gpio_neonode_irq();
    set_irq_type(zforce_data.irq, IRQF_TRIGGER_FALLING);
    ret = request_irq(zforce_data.irq, zforce_irq_handler, 0, "ZForce", NULL);
    if (ret != 0) {
      DEBUG_ERR(ZF_ERR_IRQ, zforce_data.irq);
      goto out_free_irq;
    }
  }
  
  zforce_data.isConnected = 1;
  
  /* Let user space know we are here */
  kobject_uevent(&zforce_data.tdev->dev.kobj, KOBJ_ADD);
  
  initSuccess = 1;
  wake_up(&zf_init_wq);
  return;

out_free_irq:
  gpio_neonode_request_irq(0);
  zforce_data.irq       = 0;
  zforce_data.state     = ZF_STATE_STOP;
  initSuccess           = 0;
  wake_up(&zf_init_wq);
}
DECLARE_WORK(zforce_wq, zforce_work_handler);


static void zforce_init_data(void)
{
  int i;

  zforce_data.pdev        = NULL;
  zforce_data.tData.count = 0;
  zforce_data.tState.count= 0;
  zforce_data.state       = ZF_STATE_STOP;
  zforce_data.irq         = 0;
  zforce_data.mode        = MODE_I2C;
  zforce_data.ledlvl_ready= 0;
  zforce_data.isConnected = 0;
  zforce_data.probed      = 0;
  zforce_data.work_queue  = NULL;
  zforce_data.relTime     = 0;
  zforce_data.relStrength = 3;
  
  /* Initialize touch state to empty */
  for(i = 0; i < MAX_CONTACTS; i++)
    zforce_data.tState.coordinateData[i].touch_state = TOUCH_CLEAR;
}

static int zforce_probe(struct platform_device* pdev)
{
  ZFStatus  status       = ZF_OK;
  initSuccess            = -1;

  zforce_init_data();
 
  zforce_data.work_queue = create_workqueue("zforce_wq");
  init_waitqueue_head(&zf_wq);
  init_waitqueue_head(&zf_user_wq);
  init_waitqueue_head(&zf_init_wq);
  
  if (register_input_device(pdev) < 0)
  {
    status = ZF_ERROR;
    goto out_free_irq;
  }
  
  /* Create sys entry : connected */
  if(device_create_file(&pdev->dev, &dev_attr_connected) < 0)
  { 
    status = ZF_ERROR;
    DEBUG_ERR(ZF_ERR_CONNECT_FILE);
    goto out_remove_input_dev;
  }

  if (device_create_file(&pdev->dev, &dev_attr_version) < 0)
  {
    status = ZF_ERROR;
    DEBUG_ERR(ZF_ERR_VERSION_FILE);
    goto out_remove_connect_file;
  }
  
  if (device_create_file(&pdev->dev, &dev_attr_bslreset) < 0)
  {
    status = ZF_ERROR;
    DEBUG_ERR(ZF_ERR_BSLRESET_FILE);
    goto out_remove_version_file;
  }
  
  /* Create sys entry : mode */
  if (device_create_file(&pdev->dev, &dev_attr_mode) < 0)
  {
    status = ZF_ERROR;
    DEBUG_ERR(ZF_ERR_MODE_FILE);
    goto out_remove_bslreset_file;
  }
  
  /* Create sys entry : test */
  if (device_create_file(&pdev->dev, &dev_attr_test) < 0)
  { 
    status = ZF_ERROR;
    DEBUG_ERR(ZF_ERR_TEST_FILE);
    goto out_remove_mode_file;
  }

  /* Create misc device */
  if (misc_register(&zforce_misc_device))
  {
    status = ZF_ERROR;
    DEBUG_ERR("zforce: Couldn't register device 10, %d.\n", ZF_DEV_MINOR);
    goto out_remove_test_file;
  }
  
  mutex_init(&contactframe_mutex);
  mutex_init(&ledlvl_mutex);
  
  zforce_data.pdev = pdev;

  zforce_data.probed = 1;

  queue_work(zforce_data.work_queue, &zforce_wq);
  
  wait_event_timeout(zf_init_wq, initSuccess >= 0, msecs_to_jiffies(2000));

  return (int)status;

out_remove_test_file:
  device_remove_file(&pdev->dev, &dev_attr_test);
out_remove_mode_file:
  device_remove_file(&pdev->dev, &dev_attr_mode);
out_remove_bslreset_file:
  device_remove_file(&pdev->dev, &dev_attr_bslreset);
out_remove_version_file:
  device_remove_file(&pdev->dev, &dev_attr_version);
out_remove_connect_file:
  device_remove_file(&pdev->dev, &dev_attr_connected);
out_remove_input_dev:
  remove_input_device();
out_free_irq:
  gpio_neonode_request_irq(0);
  free_irq(zforce_data.irq, NULL);
  zforce_data.irq = 0;
  zforce_data.state = ZF_STATE_STOP;
  return -(int)status;
}

static void zforce_stop_touch(void)
{
  int i;

  DEBUG_INFO("Stopping thread\n");
  zforce_data.state = ZF_STATE_STOP; 

  if (zforce_data.irq)
  {
    gpio_neonode_request_irq(0);
    free_irq(zforce_data.irq, NULL);
    zforce_data.irq = 0;
  }

  stop();
  
  if (zforce_data.work_queue)
  {
    destroy_workqueue(zforce_data.work_queue);
    zforce_data.work_queue = NULL;
  }
  
  zforce_data.tData.count = 0;
  zforce_data.tState.count= 0;
  zforce_data.state       = ZF_STATE_STOP;
  zforce_data.mode        = MODE_I2C;
  zforce_data.ledlvl_ready= 0;
  zforce_data.isConnected = 0;
  
  initSuccess             = -1;
  
  /* Initialize touch state to empty */
  for(i = 0; i < MAX_CONTACTS; i++)
    zforce_data.tState.coordinateData[i].touch_state = TOUCH_CLEAR;
  
  DEBUG_INFO("Stopping thread done\n");
}

static void zforce_restart_touch(void)
{
  DEBUG_INFO("Starting thread\n");
  zforce_data.state = ZF_STATE_RUN;

  // Reset the chip
  bslReset(false);
  
  if(zforce_data.work_queue == NULL)
    zforce_data.work_queue = create_workqueue("zforce_wq");
  queue_work(zforce_data.work_queue, &zforce_wq);
  wait_event_timeout(zf_init_wq, initSuccess >= 0, msecs_to_jiffies(2000));
  DEBUG_INFO("Starting thread done\n");
}

static void zforce_set_ready_for_update(void)
{
  zforce_stop_touch();
  bslReset(true);
  zforce_data.state = ZF_STATE_UPDATE;
}


static int zforce_remove(struct platform_device *pdev)
{
  if (zforce_data.probed)
  {
    zforce_data.probed = 0;

    kobject_uevent(&zforce_data.tdev->dev.kobj, KOBJ_REMOVE);
    if (misc_deregister(&zforce_misc_device) < 0)
      DEBUG_ERR("Error removing misc device /dev/zforce\n");
    device_remove_file(&pdev->dev, &dev_attr_connected);
    device_remove_file(&pdev->dev, &dev_attr_version);
    device_remove_file(&pdev->dev, &dev_attr_bslreset);
    device_remove_file(&pdev->dev, &dev_attr_test);
    device_remove_file(&pdev->dev, &dev_attr_mode);
  }
  bslReset(false);
  return 0;
}

static void zforce_shutdown(struct platform_device *pdev)
{
  // Just reset the chip
  bslReset(false);
}

static struct platform_driver zforce_driver =
{
  .driver = {
    .name = ZF_DRIVER_NAME,
  },
  .probe = zforce_probe,
  .shutdown = zforce_shutdown,
  .remove = zforce_remove,
};

static void zforce_nop_release(struct device* dev)
{
  /* Do Nothing */
}

static struct platform_device zforce_device =
{
  .name = ZF_DRIVER_NAME,
  .id   = 0,
  .dev  = {
    .release = zforce_nop_release,
  },
};

static int  __init zforce_init(void)
{
  int ret = 0;

  i2c_probe_success = 0;
  ret = i2c_add_driver(&zforce_i2c_driver);
  if (ret < 0)
  {
    DEBUG_ERR(ZF_ERR_I2C_ADD);
    return -ENODEV;
  }
  if (!i2c_probe_success)
  {
    i2c_del_driver(&zforce_i2c_driver);
    return -ENODEV;
  }
  
  DEBUG_INFO("Registering platform device\n");
  platform_device_register(&zforce_device);
  platform_driver_register(&zforce_driver);
  return ret;
}

static void __exit zforce_exit(void)
{
  DEBUG_INFO("Calling exit");
  if (i2c_probe_success)
  {
    i2c_probe_success = 0;
    platform_driver_unregister(&zforce_driver);
    platform_device_unregister(&zforce_device);
  }
  i2c_del_driver(&zforce_i2c_driver);
}

module_init(zforce_init);
module_exit(zforce_exit);

MODULE_DESCRIPTION("Neonode ZForce driver");
MODULE_AUTHOR("Nadim Awad <nawad@lab126.com>");
MODULE_LICENSE("GPL");
