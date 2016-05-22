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

/* Received data from the zForce follows the following structure:             */
/* [FRAME_START][Data Size][DATA (DataSize * Byte)]                           */
/* With DATA:                                                                 */
/* [Data Type (BYTE)][Type Data][Data] ... [Data Type n][Type Data n][Data]   */

#ifndef __ZFORCEINT_H__
#define __ZFORCEINT_H__

#include <linux/zforce.h>

/* Macros */

// Set this to 1 for DEBUG output
#define _ZFDEBUG 0 
#if _ZFDEBUG > 0
#    define DEBUG_INFO(fmt, args...) printk(KERN_INFO "[ZForce] " fmt, ## args)
#else
#    define DEBUG_INFO(fmt, args...)
#endif
#    define DEBUG_ERR(fmt, args...)  printk(KERN_ERR "[ZForce] ERROR: " fmt, ## args)

// Comment to enable type A instead of type B multi-touch protocol
#define SLOTS_SUPPORT

/* Driver */
#define ZF_INPUT_NAME               "zforce"
#define ZF_DRIVER_NAME              "zforce"
#define ZF_THREAD_NICE_LEVEL        (-5) // Thread level
#define ZF_DEF_TIMEOUT              1000 // Timeout in milliseconds
#define ZF_DEV_MINOR                160  // Device minor
#define ZF_REPEAT_TOUCH_TIMEOUT     60   // If no event is received, resend previous every 60ms
#define ZF_MAX_READ_RETRY           3    // I2C read retries if communication failed

/* Device specific configuration */
#define MAX_CONTACTS                2    // Maximum number of contacts the sensor can report 
#define ZFORCE_I2C_ADDRESS          0x50 // I2C address of the device
#define ZFORCE_CONFIG_FLAG          0x1  // Enable dual touch

/* Scanning Frequencies */
#define ZF_FULL_FREQ                60   // Full scanning frequency
#define ZF_IDLE_FREQ                30   // Idle scanning frequency
#define REL_RESPONSE_TIME_SEC       10   // Gest reliablity test response every 10 seconds
#define REL_RESPONSE_TIME           ZF_FULL_FREQ*REL_RESPONSE_TIME_SEC

/* Errors Messages */
#define ZF_ERR_NO_MEM_INPUT         "Could not allocate input device\n"
#define ZF_ERR_INPUT_MT             "Could not register zforce input device\n"
#define ZF_ERR_CONNECT_FILE         "Could not create connect /sys\n"
#define ZF_ERR_VERSION_FILE         "Could not create version /sys\n"
#define ZF_ERR_BSLRESET_FILE        "Could not create bslreset /sys\n"
#define ZF_ERR_MODE_FILE            "Could not create mode /sys\n"
#define ZF_ERR_TEST_FILE            "Could not create test /sys\n"

#define ZF_ERR_IRQ                  "Could not get IRQ : %d\n"
#define ZF_ERR_INIT                 "Could not initialize device\n"
#define ZF_ERR_TOUCH_DATA           "Could not retrieve touch data\n"
#define ZF_ERR_TIMEOUT              "Timeout waiting for device interrupt\n"
#define ZF_ERR_I2C_ADD              "Could not add I2C driver\n"
#define ZF_ERR_I2C_READ_RETRY       "I2C read retry\n"
#define ZF_ERR_IO_READ              "I/O. Read error\n"
#define ZF_ERR_IO_WRITE             "I/O. Write error\n"
#define ZF_ERR_ALLOC                "Could not allocate data buffer\n"
#define ZF_ERR_SYNC                 "Frame Start not received. Out of sync!\n"
#define ZF_ERR_NOT_COMMAND          "Not requested command response rcved\n"

#define ZF_ERR_SET_RESOLUTION       "Could not set resolution\n"
#define ZF_ERR_SET_CONFIGURATION    "Could not set configuration\n"
#define ZF_ERR_SET_SCAN_FREQ        "Could not set scanning frequency\n"
#define ZF_ERR_GET_LED_LVL          "Could not get LED levels\n"
#define ZF_ERR_GET_VERSION          "Could not retrieve version info\n"
#define ZF_ERR_GET_TOUCH_DATA       "Could not retrieve touch data\n"
#define ZF_ERR_X_LED                "Too many X LEDs in LowSignalAlert\n"
#define ZF_ERR_Y_LED                "Too many Y LEDs in LowSignalAlert\n"
#define ZF_ERR_SET_PULSE_SIG_INFO   "Could not send pulse signal info\n"
#define ZF_ERR_SET_FRAME_RESP       "Could not set frame response number\n"

#define ZF_ERR_NOT_RUNNING          "ZForce is not running\n"
#define ZF_ERR_NOT_STOPPED          "ZForce is not stopped\n"
#define ZF_ERR_IN_UPDATE            "ZForce is already in update mode\n"
#define ZF_ERR_UNKNOWN_STATE        "Unknown state change requested\n"

/* Commands */
#define DEACTIVATE_CMD              0x00 // Deactivate zForce
#define ACTIVATE_CMD                0x01 // Activate zForce
#define SET_RESOLUTION_CMD          0x02 // Set screen resolution
#define SET_CONFIGURATION_CMD       0x03 // Custom configuration / modes
#define TOUCH_DATA_CMD              0x04 // Request touch information
#define SCANNING_FREQ_CMD           0x08 // Set scanning frequency
#define GET_VERSION_CMD             0x0A // Request firmware version
#define FIXED_PULSE_STR_CMD         0x0F // Fixed Pulse Strength command
#define FORCE_CALIBRATION_CMD       0x1A // Force LED calibration
#define LED_LEVEL_CMD               0x1C // Request LED and photodiode information
#define SET_REL_FRAME_RESP_NUM_CMD  0x69 // Reliability set response time

/* Data Type corresponding to send commands responses */
#define FRAME_START                 0xEE // Beginning of data frame returned
#define SUCCESS_RES                 0x00 // Command succeeded
#define TYPE_DEACTIVATE_RES         0x00 // Dectivate command result
#define TYPE_ACTIVATE_RES           0x01 // Activate command result
#define TYPE_SET_RESOLUTION_RES     0x02 // SetResolution command result
#define TYPE_SET_CONFIGURATION_RES  0x03 // SetConfiguration command result
#define TYPE_TOUCH_DATA_RES         0x04 // Touch Data type
#define TYPE_RAW_DIODE_DATA_RES     0x05 // Reliability raw diode data response
#define TYPE_SCANNING_FREQ_RES      0x08 // Scanning frequency result
#define TYPE_VERSION_RES            0x0A // Version command result
#define TYPE_LOW_SIGNAL_ALERT       0x0D // Low signal alert
#define TYPE_FIXED_PULSE_STR_RES    0x0F // Fixed pulse strength 
#define TYPE_LED_LEVEL_RES          0x1C // Led level response
#define TYPE_REL_FRAME_RESP_NUM_RES 0x69 // Reliablity output frequency response
#define INVALID_COMMAND_RES         0xFE // Invalid command received

/* Touch States */
#define TOUCH_DOWN                  0    // CoordinateData touch down state 
#define TOUCH_MOVE                  1    // CoordinateData touch move state
#define TOUCH_UP                    2    // CoordinateData touch up state
#define TOUCH_CLEAR                 3    // Touch is clear

/* Sys reset commands */
typedef enum ZFState_e
{
  ZF_STATE_RUN,           // Neonode is running
  ZF_STATE_UPDATE,        // Neonode is ready for update
  ZF_STATE_STOP,          // Neonode is stopped
  ZF_STATE_CONTINUE       // Userspace command received
} ZFState;

/* Diagnostics event type on sys file diag */
typedef enum ZFTestEventType_e
{
  TEST_CONTACT_SET,       // Create a virtual contact
  TEST_CONTACT_REMOVE,    // Remove a contact
  TEST_CONTACT_CLEAR      // Clear all contacts
} ZFTestEventType;

typedef enum ZFDiagRequest_e
{
  DIAG_LED_LEVELS         // Request LED levels from device
} ZFDiagRequest;

/* Mode of operation on sys file mode */
typedef enum ZFMode_e
{
  MODE_I2C,               // Switch to I2C mode
  MODE_TEST               // Switch to diagnostics mode
} ZFMode;

/* Status */
typedef enum ZFStatus_e
{
  ZF_OK,                  // Operation OK
  ZF_ERROR,               // General error
  ZF_TIMEOUT,             // Timeout waiting for interrupt
  ZF_IO_ERROR,            // Error on communication
  ZF_NO_RESPONSE,         // No response to request recieved
  ZF_FORCE_EXIT,          // Driver forced an exit
  ZF_CONTINUE
} ZFStatus;

#pragma pack(1)
typedef struct DeactivateResult_s
{
  u8 result;
} DeactivateResult;

typedef struct ActivateResult_s
{
  u8 result;
} ActivateResult;

typedef struct SetResolutionResult_s
{
  u8 result;
} SetResolutionResult;

typedef struct SetConfigurationResult_s
{
  u8 result;
} SetConfigurationResult;

typedef struct SetScanningFrequencyResult_s
{
  u8 result;
} SetScanningFrequencyResult;

typedef struct SetRelFrameRespNumResult_s
{
  u8 result;
} SetRelFrameRespNumResult;

typedef struct ConfigurationData_s
{
  u32 flags;
} ConfigurationData;

typedef struct CoordinateData_s
{
  u16 X;
  u16 Y;
  u8  touch_state:2;
  u8  id         :4;
  u8  reserved1  :5;
  u8  reserved2  :5;
  u8  probability;
} CoordinateData;

typedef struct TouchData_s
{
  u8 count;
  CoordinateData coordinateData[MAX_CONTACTS];
} TouchData;

typedef struct AmbientLightData_s
{
  u8 ambient_level;
} AmbientLightData;

typedef struct VersionData_s
{
  u16 major;
  u16 minor;
  u16 build;
  u16 revision;
} VersionData;

typedef struct LowSignalInfo
{
  u8 PDSignal1Low:4;
  u8 PDSignal2Low:4;
} LowSignalInfo;

typedef struct LowSignalAlert_s
{
  u8 xCount;
  u8 yCount;
  LowSignalInfo xLEDS[MAX_X_LED_COUNT];
  LowSignalInfo yLEDS[MAX_Y_LED_COUNT];
} LowSignalAlert;

typedef struct ZFCommand_s
{
  unsigned char    cmd;
  void            *data;
  struct list_head pend;
} ZFCommand;

typedef struct ZForceDrvData_s
{
  struct platform_device *pdev;               /* Platform device */
  ZFState                 state;              /* Nenode state */
  ZFMode                  mode;               /* Neonode mode of operation */
  struct input_dev       *tdev;               /* Device */
  struct workqueue_struct *work_queue;        /* Own work queue */
  int                     irq;                /* IRQ number */
  u32                     probed;             /* I2C device has been probed */
  u32                     isConnected;        /* Device connected */
  TouchData               tData;              /* Touch frame */
  TouchData               tState;             /* State frame */
  VersionData             vData;              /* Version Data */
  LedLevelResponse        ledLvl;             /* LED levels */
  bool                    ledlvl_ready;       /* 0: Data not ready / 1: Data ready */
  bool                    force_calib_sent;   /* force calibration command sent flag */
  FixedPulseStrengthResponse rawDiode;
  bool                    raw_diode_ready;    /* 0: Data not ready / 1: Data ready */
  int                     relTime;            /* LED pulse time */
  int                     relStrength;        /* LED pulse strength */
  LowSignalAlert          lowSig;             /* Low signal alert data */
} ZForceDrvData;

/* Prototypes */
static void zforce_set_ready_for_update(void);
static void zforce_stop_thread(void);
static void zforce_restart_thread(void);
#endif // __ZFORCEINT_H__
