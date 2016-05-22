/*
 * mwan.c  --  Yoshi WAN hardware control driver
 *
 * Copyright 2005-2010 Lab126, Inc.  All rights reserved.
 *
 */


#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <mach/mwan.h>
#include <linux/device.h>
#include <linux/pmic_external.h>
#include <mach/pmic_power.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <linux/wait.h>
#include <linux/platform_device.h>

#define DEBUG 1

#ifdef DEBUG
#define log_debug(format, arg...) printk("mwan: D %s:" format, __func__, ## arg)
#else
#define log_debug(format, arg...)
#endif

#define log_info(format, arg...) printk("mwan: I %s:" format, __func__, ## arg)
#define log_err(format, arg...) printk("mwan: E %s:" format, __func__, ## arg)

#define VERSION			"2.0.0"

#define PROC_WAN		"wan"
#define PROC_WAN_POWER		"power"
#define PROC_WAN_TYPE		"type"
#define PROC_WAN_USB		"usb"

static struct proc_dir_entry *proc_wan_parent;
static struct proc_dir_entry *proc_wan_power;
static struct proc_dir_entry *proc_wan_type;
static struct proc_dir_entry *proc_wan_usb;

static wan_status_t wan_status = WAN_OFF;
static int modem_type = MODEM_TYPE_UNKNOWN;

static int wan_rf_enable_state = 0;
static int wan_usb_enable_state = 0;
static int wan_on_off_state = 0;

#define WAN_STRING_CLASS	"wan"
#define WAN_STRING_DEV		"mwan"

static struct file_operations mwan_ops = {
	.owner = THIS_MODULE,
	.ioctl = NULL,		// (add ioctl support, if desired)
};


static struct class *wan_class = NULL;
static struct device *wan_dev = NULL;
static int wan_major = 0;

// standard network deregistration time definition:
//   2s -- maximum time required between the start of power down and that of IMSI detach
//   5s -- maximum time required for the IMSI detach
//   5s -- maximum time required between the IMSI detach and power down finish (time
//         required to stop tasks, etc.)
//   3s -- recommended safety margin
#define NETWORK_DEREG_TIME	((12 + 3) * 1000)

// for the Elmo module, we use an optimized path for performing deregistration
#define NETWORK_DEREG_TIME_OPT	(3 * 1000)

// minimum allowed delay between TPH notifications
// set this lower but don't remove completely just in case we get spurious GPIO transitions on boot
// 600 ms for PWR ON/OFF and then 5 sec boot until tasks are all initialized
#define WAKE_EVENT_INTERVAL	6

/*
 * Small snippet taken out of the implementation of msecs_to_jiffies
 * to optimize a little bit.
 * HZ is equal to or smaller than 1000, and 1000 is a nice
 * round multiple of HZ, divide with the factor between them,
 * but round upwards:
 */
#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
#define MS_TO_JIFFIES(X)	((unsigned long)((X) + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ))
#else
#define MS_TO_JIFFIES(X)	msecs_to_jiffies(X)
#endif

#define MSEC_2_JIFFIES(X)	\
(__builtin_constant_p(X)	\
? MS_TO_JIFFIES(X):msecs_to_jiffies(X))



static unsigned long modem_off_jiffies = INITIAL_JIFFIES;	// setting this to initial jiffies to avoid getting zero value in there

#define ELMO_SUPERCAP_CHARGE_TIME_MS		20
#define ELMO_PWR_ON_OFF_HOLD_TIME_MS		600
#define ELMO_SUPERCAP_DISCHARGE_TIME_MSEC	1000	// WAN power-down supercap discharge delay for QSC-based modems
#define ELMO_BOOT_TIME_SEC			5
#define GPIO_DBOUNCE_TIME_MSEC			30

extern int gpio_wan_fw_ready_irq(void);
extern int gpio_wan_fw_ready(void);
extern int gpio_wan_usb_wake(void);
extern int gpio_wan_mhi_irq(void);
static wait_queue_head_t elmo_wait_q;
static int elmo_fw_ready_condition;

static inline int
get_wan_on_off(
	void)
{
	return wan_on_off_state;
}


static void
set_wan_on_off(
	int enable)
{
	unsigned long current_jiffies;
	unsigned long time_delta;		// to get rid of the compile warning of C90 style
	unsigned long wait_jiffies;


	extern void gpio_wan_power(int);

	enable = enable != 0;	// (ensure that "enable" is a boolean)

	if (!enable) {
		modem_off_jiffies = jiffies;
	} else {
		current_jiffies = jiffies;
		time_delta = current_jiffies - modem_off_jiffies;
		// allow for supercap discharge so that the modem actually powers off
		if(time_delta < MSEC_2_JIFFIES(ELMO_SUPERCAP_DISCHARGE_TIME_MSEC)) {			
			// only wait for the remaining time
			wait_jiffies = (MSEC_2_JIFFIES(ELMO_SUPERCAP_DISCHARGE_TIME_MSEC) - time_delta);
			log_info("wpd:wait=%lu jiffies:modem power on delay\n", wait_jiffies);
			while(wait_jiffies) {
				set_current_state(TASK_UNINTERRUPTIBLE);
				wait_jiffies = schedule_timeout(wait_jiffies);
			}
		}
	}

	log_debug("pow:enable=%d:setting WAN hardware power state\n", enable);

	gpio_wan_power(enable);

	wan_on_off_state = enable;
}


static inline int
get_wan_rf_enable(
	void)
{
	return wan_rf_enable_state;
}


static void
set_wan_rf_enable(
	int enable)
{
	extern void gpio_wan_rf_enable(int);

	if (enable != get_wan_rf_enable()) {
		log_debug("swe:enable=%d:setting WAN RF enable state\n", enable);

		gpio_wan_rf_enable(enable);

		wan_rf_enable_state = enable;
	}
}

static inline int
get_wan_usb_enable(
	void)
{
	return wan_usb_enable_state;
}


static void
set_wan_usb_enable(
	int enable)
{
	extern void gpio_wan_usb_enable(int);

	if (enable != get_wan_usb_enable()) {
		log_debug("swu:enable=%d:setting WAN USB enable state\n", enable);

		gpio_wan_usb_enable(enable);

		wan_usb_enable_state = enable;
	}
}


static int
set_wan_power(
	wan_status_t new_status)
{
	wan_status_t check_status = new_status == WAN_OFF_KILL ? WAN_OFF : new_status;
	int err = 0;

	if (check_status == wan_status) {
		return 0;
	}

	switch (new_status) {

		case WAN_ON :
			// bring up WAN_ON_OFF
			elmo_fw_ready_condition = 0;
			set_wan_on_off(1);

			// pause following power-on before bringing up the RF_ENABLE line
			msleep(ELMO_SUPERCAP_CHARGE_TIME_MS);

			// pulse WAN_RF_ENABLE to power on
			set_wan_rf_enable(1);
			msleep(ELMO_PWR_ON_OFF_HOLD_TIME_MS);
			set_wan_rf_enable(0);
			log_debug("elmo_cond:elmo_fw_ready_condition=%d:Waiting for fw ready event\n",elmo_fw_ready_condition);
			// wait for firmware ready
			if( (err = wait_event_interruptible_timeout(
				elmo_wait_q,
				elmo_fw_ready_condition,
				ELMO_BOOT_TIME_SEC*HZ)) <= 0 ) {
				log_err("fw_ready_err:err=%d:elmo fw ready %s\n",err,(err)?("received signal"):("timed out"));
				err = -EIO;
			} else {
				log_debug("elmo_cond:elmo_fw_ready_condition=%d:FW ready\n",elmo_fw_ready_condition);
			}
			/* enable usb wake irq */
			enable_irq(gpio_wan_mhi_irq());
			break;
		default :
			log_err("req_err:request=%d:unknown power request\n", new_status);

			// (fall through)

		case WAN_OFF :
		case WAN_OFF_KILL :
			disable_irq(gpio_wan_mhi_irq());
			// pulse WAN_RF_ENABLE to power off
			set_wan_rf_enable(1);
			msleep(ELMO_PWR_ON_OFF_HOLD_TIME_MS);
			set_wan_rf_enable(0);

			if (new_status != WAN_OFF_KILL) {
				// wait the necessary deregistration interval
				msleep(NETWORK_DEREG_TIME_OPT);
			}

			// bring down WAN_ON_OFF
			set_wan_on_off(0);
			new_status = WAN_OFF;
			break;

	}

	wan_status = new_status;

	wan_set_power_status(wan_status);

	return err;
}


static void
init_modem_type(
	int type)
{
	if (modem_type != type) {
		log_info("smt:type=%d:setting modem type\n", type);

		modem_type = type;
	}
}


int
wan_get_modem_type(
	void)
{
	return modem_type;
}

EXPORT_SYMBOL(wan_get_modem_type);


static int
proc_power_read(
	char *page,
	char **start,
	off_t off,
	int count,
	int *eof,
	void *data)
{
	*eof = 1;

	return sprintf(page, "%d\n", wan_status == WAN_ON ? 1 : 0);
}


static int
proc_power_write(
	struct file *file,
	const char __user *buf,
	unsigned long count,
	void *data)
{
	char lbuf[16];
	unsigned char op;

	memset(lbuf, 0, sizeof(lbuf));

	if (copy_from_user(lbuf, buf, 1)) {
		return -EFAULT;
	}

	op = lbuf[0];
	if (op >= '0' && op <= '9') {
		wan_status_t new_wan_status = (wan_status_t)(op - '0'), prev_wan_status = wan_status;

		switch (new_wan_status) {

			case WAN_OFF :
			case WAN_ON :
				// perform normal on/off power handling
				if ((new_wan_status == WAN_ON && prev_wan_status == WAN_OFF) ||
				    (new_wan_status == WAN_OFF && prev_wan_status == WAN_ON)) {
					if( set_wan_power(new_wan_status) != 0 ) {
						return -EIO;
					}
				}
				break;


			case WAN_OFF_KILL :
				set_wan_power(new_wan_status);
				break;

			default :
				log_err("req_err:request=%d:unknown power request\n", new_wan_status);
				break;

		}

	} else {
		log_err("req_err:request='%c':unknown power request\n", op);
	}

	return count;
}


static int
proc_type_read(
	char *page,
	char **start,
	off_t off,
	int count,
	int *eof,
	void *data)
{
	*eof = 1;

	return sprintf(page, "%d\n", modem_type);
}


static int
proc_type_write(
	struct file *file,
	const char __user *buf,
	unsigned long count,
	void *data)
{
	char lbuf[16], ch;

	memset(lbuf, 0, sizeof(lbuf));

	if (copy_from_user(lbuf, buf, 1)) {
		return -EFAULT;
	}

	ch = lbuf[0];
	if (ch >= '0' && ch <= '9') {
		init_modem_type(ch - '0');

	} else {
		log_err("type_err:type=%c:invalid type\n", ch);

	}

	return count;
}


static int
proc_usb_read(
	char *page,
	char **start,
	off_t off,
	int count,
	int *eof,
	void *data)
{
	*eof = 1;

	return sprintf(page, "%d\n", get_wan_usb_enable());
}


static int
proc_usb_write(
	struct file *file,
	const char __user *buf,
	unsigned long count,
	void *data)
{
	char lbuf[16];

	memset(lbuf, 0, sizeof(lbuf));

	if (copy_from_user(lbuf, buf, 1)) {
		return -EFAULT;
	}

	set_wan_usb_enable(lbuf[0] != '0');

	return count;
}


static wan_usb_wake_callback_t usb_wake_callback = NULL;

void
wan_set_usb_wake_callback(
	wan_usb_wake_callback_t wake_fn)
{
	usb_wake_callback = wake_fn;
}
EXPORT_SYMBOL(wan_set_usb_wake_callback);

static void
wan_tph_notify(
	void)
{
	static unsigned long last_tph_sec = 0;
	unsigned long current_sec = CURRENT_TIME_SEC.tv_sec;

	/* limit back-to-back pmic events */
	if( current_sec - last_tph_sec < WAKE_EVENT_INTERVAL ) {
		return;
	}
	last_tph_sec = current_sec;
	kobject_uevent(&wan_dev->kobj, KOBJ_CHANGE);
	log_info("tph::tph event occurred; notifying system of TPH\n");
}

static void do_elmo_fw_ready(struct work_struct *dummy)
{
	/* FW ready line is high until power off */
	if ( gpio_wan_fw_ready() ) {
		log_debug("elmo_fw::elmo fw ready\n");
		elmo_fw_ready_condition = 1;
		wake_up(&elmo_wait_q);
	}
	enable_irq(gpio_wan_fw_ready_irq());
}
static DECLARE_DELAYED_WORK(elmo_fw_ready_work, do_elmo_fw_ready);

static irqreturn_t elmo_fw_ready_handler(int irq, void *devid)
{
	/* debounce time for 30ms */
	disable_irq_nosync(irq);
	schedule_delayed_work(&elmo_fw_ready_work, MSEC_2_JIFFIES(GPIO_DBOUNCE_TIME_MSEC));
	return IRQ_HANDLED;
}

static void do_elmo_usb_wake(struct work_struct *dummy)
{
	if ( gpio_wan_usb_wake() ) {
		log_debug("elmo_usb::wakeup USB\n");
		if( usb_wake_callback ) {
			(*usb_wake_callback)();
		}
	}
	enable_irq(gpio_wan_mhi_irq());

}

static DECLARE_DELAYED_WORK(elmo_usb_wake_work, do_elmo_usb_wake);
static irqreturn_t elmo_usb_wake_handler(int irq, void  *devid)
{
	/* debounce time for 30ms */
	disable_irq_nosync(irq);
	schedule_delayed_work(&elmo_usb_wake_work, MSEC_2_JIFFIES(GPIO_DBOUNCE_TIME_MSEC));
	return IRQ_HANDLED;
}

static int elmo_suspend(struct platform_device *pdev, pm_message_t state)
{
	log_debug("elmo_suspend::disable irqs\n");
	disable_irq(gpio_wan_fw_ready_irq());
	disable_irq(gpio_wan_mhi_irq());
	return 0;
}

static int elmo_resume(struct platform_device *pdev)
{
	log_debug("elmo_resume::enable irqs\n");
	enable_irq(gpio_wan_fw_ready_irq());
	enable_irq(gpio_wan_mhi_irq());
	return 0;
}

/* dummy function to prevent error messages */
static void elmo_device_release (struct device *dev)
{
	return;
}

static int __devinit elmo_probe(struct platform_device *pdev)
{
	int ret = -1;
	int error = 0;

	log_debug("elmo_probe::Probing elmo device\n");
	/* set up the PMIC event configuration for the "ON2B" line */
	error = pmic_power_set_conf_button(BT_ON2B, 0, 0);
	if (error < 0) {
		log_err("pmic_err:err=%d:could not set bt_on2b\n",error);
		goto exit0;
	}
	error = pmic_power_event_sub(PWR_IT_ONOFD2I, wan_tph_notify);
	if (error < 0) {
		log_err("pmic_err:err=%d:could not subscribe WAN TPH event\n",error);
		goto exit0;
	}

	init_waitqueue_head(&elmo_wait_q);
	error = request_irq(gpio_wan_fw_ready_irq(),elmo_fw_ready_handler,IRQF_TRIGGER_RISING,"ELMO_fw_ready",NULL);
	if (error < 0) {
		log_err("elmo_fw_irq_err:irq=%d:Could not request IRQ\n", gpio_wan_fw_ready_irq());
		goto exit1;
	}

	error = request_irq(gpio_wan_mhi_irq(),elmo_usb_wake_handler,IRQF_TRIGGER_RISING,"ELMO_usb_wake",NULL);
	
		if (error < 0) {
		log_err("elmo_usb_irq_err:irq=%d:Could not request IRQ\n", gpio_wan_mhi_irq());
		goto exit_fw_irq;
	}
	/* disable usb wake irq until WAN is powered up */
	disable_irq(gpio_wan_mhi_irq());

	wan_major = register_chrdev(0, WAN_STRING_DEV, &mwan_ops);
	if (wan_major < 0) {
		ret = wan_major;
		log_err("dev_err:device=" WAN_STRING_DEV ",err=%d:could not register device\n", ret);
		goto exit_usb_irq;
	}

	wan_class = class_create(THIS_MODULE, WAN_STRING_CLASS);
	if (IS_ERR(wan_class)) {
		ret = PTR_ERR(wan_class);
		log_err("class_err:err=%d:could not create class\n", ret);
		goto exit2;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31))
	wan_dev = device_create(wan_class, NULL, MKDEV(wan_major, 0), NULL, WAN_STRING_DEV);
#else
	wan_dev = device_create_drvdata(wan_class, NULL, MKDEV(wan_major, 0), NULL, WAN_STRING_DEV);
#endif
	if (IS_ERR(wan_dev)) {
		ret = PTR_ERR(wan_dev);
		log_err("dev_err:err=%d:could not create class device\n", ret);
		goto exit3;
	}

	wan_set_power_status(WAN_OFF);

	ret = 0;
	goto exit0;

exit3:
	class_destroy(wan_class);
	wan_class = NULL;

exit2:
	unregister_chrdev(wan_major, WAN_STRING_DEV);

exit_usb_irq:
	free_irq(gpio_wan_mhi_irq(),NULL);

exit_fw_irq:
	free_irq(gpio_wan_fw_ready_irq(),NULL);
exit1:
	/* clear the PMIC event handler */
	pmic_power_event_unsub(PWR_IT_ONOFD2I, wan_tph_notify);

exit0:
	return ret;

}

static int __devexit elmo_remove(struct platform_device *pdev)
{
	extern void gpio_wan_exit(void *);

	log_debug("elmo_remove::removing elmo device\n");
	wan_set_power_status(WAN_INVALID);

	/* clear the PMIC event handler */
	pmic_power_event_unsub(PWR_IT_ONOFD2I, wan_tph_notify);

	if (wan_dev != NULL) {
		device_destroy(wan_class, MKDEV(wan_major, 0));
		wan_dev = NULL;
		class_destroy(wan_class);
		unregister_chrdev(wan_major, WAN_STRING_DEV);
	}
	free_irq(gpio_wan_fw_ready_irq(),NULL);
	free_irq(gpio_wan_mhi_irq(),NULL);

	return 0;
}

static struct platform_driver elmo_driver = {
        .driver = {
                   .name = "mwan",
                   .owner  = THIS_MODULE,
                   },
        .suspend = elmo_suspend,
        .resume = elmo_resume,
	.probe = elmo_probe,
	.remove = elmo_remove,
};

static struct platform_device elmo_device = {
	.name = "mwan",
	.id = -1,
	.dev = {
		.release = elmo_device_release,
	       },
};

static int
wan_init(
	void)
{
	if ( platform_device_register(&elmo_device) != 0 ) {
		log_err("device_reg::can not register elmo device\n");
		return -1;
	}
	if ( platform_driver_register(&elmo_driver) != 0 ) {
		log_err("driver_reg::can not register elmo driver\n");
		goto exit1;
	}

	return 0;

exit1:
	platform_device_unregister(&elmo_device);
	return -1;
}


static void
wan_exit(
	void)
{
        platform_driver_unregister(&elmo_driver);
	platform_device_unregister(&elmo_device);
}


static int __init
mwan_init(
	void)
{
	int ret = 0;

	log_info("init:yoshi WAN hardware driver " VERSION "\n");

	// create the "/proc/wan" parent directory
	proc_wan_parent = create_proc_entry(PROC_WAN, S_IFDIR | S_IRUGO | S_IXUGO, NULL);
	if (proc_wan_parent != NULL) {

		// create the "/proc/wan/power" entry
		proc_wan_power = create_proc_entry(PROC_WAN_POWER, S_IWUSR | S_IRUGO, proc_wan_parent);
		if (proc_wan_power != NULL) {
			proc_wan_power->data = NULL;
			proc_wan_power->read_proc = proc_power_read;
			proc_wan_power->write_proc = proc_power_write;
		}

		// create the "/proc/wan/type" entry
		proc_wan_type = create_proc_entry(PROC_WAN_TYPE, S_IWUSR | S_IRUGO, proc_wan_parent);
		if (proc_wan_type != NULL) {
			proc_wan_type->data = NULL;
			proc_wan_type->read_proc = proc_type_read;
			proc_wan_type->write_proc = proc_type_write;
		}

		// create the "/proc/wan/usb" entry
		proc_wan_usb = create_proc_entry(PROC_WAN_USB, S_IWUSR | S_IRUGO, proc_wan_parent);
		if (proc_wan_usb != NULL) {
			proc_wan_usb->data = NULL;
			proc_wan_usb->read_proc = proc_usb_read;
			proc_wan_usb->write_proc = proc_usb_write;
		}

	} else {
		ret = -1;

	}

	if (ret == 0) {
		ret = wan_init();
	}

	return ret;
}


static void __exit
mwan_exit(
	void)
{
	if (proc_wan_parent != NULL) {
		remove_proc_entry(PROC_WAN_USB, proc_wan_parent);
		remove_proc_entry(PROC_WAN_TYPE, proc_wan_parent);
		remove_proc_entry(PROC_WAN_POWER, proc_wan_parent);
		remove_proc_entry(PROC_WAN, NULL);

		proc_wan_usb = proc_wan_type = proc_wan_power = proc_wan_parent = NULL;
	}

	wan_exit();
}


module_init(mwan_init);
module_exit(mwan_exit);

MODULE_DESCRIPTION("Yoshi WAN hardware driver");
MODULE_AUTHOR("Lab126");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION);

