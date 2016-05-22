/*
 * mx50_proximity -- MX50 Promixity Sensor for Microchip PIC12LF1822
 *
 * Copyright 2009-2011 Amazon Technologies Inc., All rights reserved
 * Manish Lachwani (lachwani@lab126.com)
 *
 */
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/sysdev.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/io.h>
#include <asm/irq.h>

extern int gpio_proximity_int(void);
extern int gpio_proximity_detected(void);

#define DRIVER_NAME			"MX50_Proximity"
#define PROXIMITY_THRESHOLD		10 /* 10 ms */
#define MX50_PROXIMITY_I2C_ADDR		0x0D

/* Register Set for the PIC12LF1822 */
#define PIC12LF1822_FWVER		0x40
#define PIC12LF1822_PROXL		0x41
#define PIC12LF1822_PROXH		0x42
#define PIC12LF1822_BCKGNDAVGL		0x43
#define PIC12LF1822_BCKGNDAVGH		0x44
#define PIC12LF1822_PROXSTATE		0x45
#define PIC12LF1822_PROXTHRES		0x46
#define PIC12LF1822_PROXNODETECT	0x47
#define PIC12LF1822_WDTCON		0x48
#define PIC12LF1822_PROXSCANRATE	0x49
#define PIC12LF1822_PROXAVGRATE		0x4A
#define PIC12LF1822_PROXTIMEOUTL	0x4B
#define PIC12LF1822_PROXTIMEOUTH	0x4C
#define PIC12LF1822_PROXDBNCDET		0x4D
#define PIC12LF1822_PROXDBNCREL		0x4E
#define PIC12LF1822_PROXAVGEN		0x4F

#define PROXIMITY_PORT_MINOR		161	/* /dev/proximitysensor */

#define PROXIMITY_MAX_REGISTERS		0x4F

static struct miscdevice proximity_misc_device = {
	PROXIMITY_PORT_MINOR,
	"proximitysensor",
	NULL,
};

struct mx50_proximity_info {
	struct i2c_client *client;
};

static struct i2c_client *mx50_proximity_i2c_client;

static int mx50_proximity_read_i2c(u8 *id, u8 reg_num)
{
	s32 error;

	error = i2c_smbus_read_byte_data(mx50_proximity_i2c_client, reg_num);
	if (error < 0) {
		return -EIO;
	}

	*id = (error & 0xFF);
	return 0;
}

static int mx50_proximity_write_i2c(u8 reg_num, u8 data)
{
	return i2c_smbus_write_byte_data(mx50_proximity_i2c_client, reg_num, data);
}

static u8 proximity_reg_number = 0;

static void dump_regs(void)
{
	u8 i = 0;
	u8 value = 0;

	for (i = PIC12LF1822_FWVER; i <= PROXIMITY_MAX_REGISTERS; i++) {
		mx50_proximity_read_i2c(&value, i);	
		printk("Register 0x%x = Value:0x%x\n", i, value);
	}
}

static ssize_t proximity_reg_store(struct sys_device *dev, struct sysdev_attribute *attr, 
				const char *buf, size_t size)
{
	u16 value = 0;

	if (sscanf(buf, "%hx", &value) <= 0) {
		printk(KERN_ERR "Could not store the codec register value\n");
		return -EINVAL;
	}

	proximity_reg_number = value;
	return size;
}

static ssize_t proximity_reg_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	char *curr = buf;

	curr += sprintf(curr, "Proximity Register Number: %d\n", proximity_reg_number);
	curr += sprintf(curr, "\n");

	return curr - buf;
}

static SYSDEV_ATTR(proximity_reg, 0644, proximity_reg_show, proximity_reg_store);

static struct sysdev_class proximity_reg_sysclass = {
	.name	= "proximity_reg",
};

static struct sys_device device_proximity_reg = {
	.id	= 0,
	.cls	= &proximity_reg_sysclass,
};

static ssize_t proximity_register_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	char *curr = buf;
	u8 value = 0;

	if (proximity_reg_number > PROXIMITY_MAX_REGISTERS) {
		curr += sprintf(curr, "Proximity Registers\n");
		curr += sprintf(curr, "\n");
		dump_regs();
	}
	else {
		mx50_proximity_read_i2c(&value, proximity_reg_number);
		curr += sprintf(curr, "Proximity Register %d\n", proximity_reg_number);
		curr += sprintf(curr, "\n");
		curr += sprintf(curr, " Value: 0x%x\n", value);
		curr += sprintf(curr, "\n");
	}

	return curr - buf;
}

static ssize_t proximity_register_store(struct sys_device *dev, struct sysdev_attribute *attr, 
					const char *buf, size_t size)
{
	u8 value = 0;

	if (proximity_reg_number > PROXIMITY_MAX_REGISTERS) {
		printk(KERN_ERR "No codec register %d\n", proximity_reg_number);
		return size;
	}

	if (sscanf(buf, "%hhx", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}
	
	mx50_proximity_write_i2c(proximity_reg_number, value);
	return size;
}

static SYSDEV_ATTR(proximity_register, 0644, proximity_register_show, proximity_register_store);

static struct sysdev_class proximity_register_sysclass = {
	.name	= "proximity_register",
};

static struct sys_device device_proximity_register = {
	.id	= 0,
	.cls	= &proximity_register_sysclass,
};

static ssize_t prox_fwver_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	u8 value;

	mx50_proximity_read_i2c(&value, PIC12LF1822_FWVER);
	return sprintf(buf, "%d\n", value);
}

static SYSDEV_ATTR(fwver, 0644, prox_fwver_show, NULL); 	 

static ssize_t prox_prox_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	u8 value_LSB, value_MSB;
	u16 value;

	mx50_proximity_read_i2c(&value_LSB, PIC12LF1822_PROXL);
	mx50_proximity_read_i2c(&value_MSB, PIC12LF1822_PROXH);
	value = (u16)(value_MSB<<8|value_LSB);
	return sprintf(buf, "%d\n", value);
}

static SYSDEV_ATTR(prox, 0644, prox_prox_show, NULL); 	 

static ssize_t prox_bckgndavg_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	u8 value_LSB, value_MSB;
	u16 value;

	mx50_proximity_read_i2c(&value_LSB, PIC12LF1822_BCKGNDAVGL);
	mx50_proximity_read_i2c(&value_MSB, PIC12LF1822_BCKGNDAVGH);
	value = (u16)(value_MSB<<8|value_LSB);
	return sprintf(buf, "%d\n", value);
}

static ssize_t prox_bckgndavg_store(struct sys_device *dev, struct sysdev_attribute *attr, 
					const char *buf, size_t size)
{
	u16 value = 0;
	u8 tmp_LSB = 0;
	u8 tmp_MSB = 0;

	if (sscanf(buf, "%hx", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}
		
	tmp_LSB = (u8)value;
	tmp_MSB = (u8)(value>>8);

	mx50_proximity_write_i2c(PIC12LF1822_BCKGNDAVGL, tmp_LSB);
	mx50_proximity_write_i2c(PIC12LF1822_BCKGNDAVGH, tmp_MSB);
	return size;
}

static SYSDEV_ATTR(bckgndavg, 0644, prox_bckgndavg_show, prox_bckgndavg_store); 	 

static ssize_t prox_proxstate_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	u8 value;

	mx50_proximity_read_i2c(&value, PIC12LF1822_PROXSTATE);
	return sprintf(buf, "%d\n", value);
}

static ssize_t prox_proxstate_store(struct sys_device *dev, struct sysdev_attribute *attr, 
					const char *buf, size_t size)
{
	u8 value = 0;

	if (sscanf(buf, "%hhx", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}

	mx50_proximity_write_i2c(PIC12LF1822_PROXSTATE, value);
	return size;
}

static SYSDEV_ATTR(proxstate, 0644, prox_proxstate_show, prox_proxstate_store); 	 


static ssize_t prox_proxthres_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{	
	u8 value;

	mx50_proximity_read_i2c(&value, PIC12LF1822_PROXTHRES);
	return sprintf(buf, "%d\n", value);
}

static ssize_t prox_proxthres_store(struct sys_device *dev, struct sysdev_attribute *attr, 
					const char *buf, size_t size)
{
	u8 value = 0;
	
	if (sscanf(buf, "%hhx", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}

	mx50_proximity_write_i2c(PIC12LF1822_PROXTHRES, value);
	return size;
}

static SYSDEV_ATTR(proxthres, 0644, prox_proxthres_show, prox_proxthres_store); 

static ssize_t prox_proxnodetect_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	u8 value;

	mx50_proximity_read_i2c(&value, PIC12LF1822_PROXNODETECT);
	return sprintf(buf, "%d\n", value);
}

static ssize_t prox_proxnodetect_store(struct sys_device *dev, struct sysdev_attribute *attr, 
					const char *buf, size_t size)
{
	u8 value = 0;

	if (sscanf(buf, "%hhx", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}

	mx50_proximity_write_i2c(PIC12LF1822_PROXNODETECT, value);
	return size;
}

static SYSDEV_ATTR(proxnodetect, 0644, prox_proxnodetect_show, prox_proxnodetect_store); 

static ssize_t prox_wdtcon_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	u8 value;

	mx50_proximity_read_i2c(&value, PIC12LF1822_WDTCON);
	return sprintf(buf, "%d\n", value);
}

static ssize_t prox_wdtcon_store(struct sys_device *dev, struct sysdev_attribute *attr, 
					const char *buf, size_t size)
{
	u8 value = 0;

	if (sscanf(buf, "%hhx", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}

	mx50_proximity_write_i2c(PIC12LF1822_WDTCON, value);
	return size;
}

static SYSDEV_ATTR(wdtcon, 0644, prox_wdtcon_show, prox_wdtcon_store); 

static ssize_t prox_proxscanrate_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	u8 value;

	mx50_proximity_read_i2c(&value, PIC12LF1822_PROXSCANRATE);
	return sprintf(buf, "%d\n", value);
}

static ssize_t prox_proxscanrate_store(struct sys_device *dev, struct sysdev_attribute *attr, 
					const char *buf, size_t size)
{
	u8 value = 0;

	if (sscanf(buf, "%hhx", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}

	mx50_proximity_write_i2c(PIC12LF1822_PROXSCANRATE, value);
	return size;
}

static SYSDEV_ATTR(proxscanrate, 0644, prox_proxscanrate_show, prox_proxscanrate_store); 

static ssize_t prox_proxavgrate_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	u8 value;

	mx50_proximity_read_i2c(&value, PIC12LF1822_PROXAVGRATE);
	return sprintf(buf, "%d\n", value);
}

static ssize_t prox_proxavgrate_store(struct sys_device *dev, struct sysdev_attribute *attr, 
					const char *buf, size_t size)
{
	u8 value = 0;

	if (sscanf(buf, "%hhx", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}
	
	mx50_proximity_write_i2c(PIC12LF1822_PROXAVGRATE, value);
	return size;
}

static SYSDEV_ATTR(proxavgrate, 0644, prox_proxavgrate_show, prox_proxavgrate_store); 

static ssize_t prox_proxtimeout_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	u8 value_LSB, value_MSB;
	u16 value;

	mx50_proximity_read_i2c(&value_LSB, PIC12LF1822_PROXTIMEOUTL);
	mx50_proximity_read_i2c(&value_MSB, PIC12LF1822_PROXTIMEOUTH);

	value = (u16)(value_MSB<<8|value_LSB);
	return sprintf(buf, "%d\n", value);
}

static ssize_t prox_proxtimeout_store(struct sys_device *dev, struct sysdev_attribute *attr, 
					const char *buf, size_t size)
{
	u16 value = 0;
	u8 tmp_LSB = 0;
	u8 tmp_MSB = 0;

	if (sscanf(buf, "%hx", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}

	tmp_LSB = (u8)value;
	tmp_MSB = (u8)(value>>8);

	mx50_proximity_write_i2c(PIC12LF1822_PROXTIMEOUTL, tmp_LSB);
	mx50_proximity_write_i2c(PIC12LF1822_PROXTIMEOUTH, tmp_MSB);
	return size;
}

static SYSDEV_ATTR(proxtimeout, 0644, prox_proxtimeout_show, prox_proxtimeout_store); 

static ssize_t prox_proxdbncdet_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	u8 value;

	mx50_proximity_read_i2c(&value, PIC12LF1822_PROXDBNCDET);
	return sprintf(buf, "%d\n", value);
}

static ssize_t prox_proxdbncdet_store(struct sys_device *dev, struct sysdev_attribute *attr, 
					const char *buf, size_t size)
{
	u8 value = 0;

	if (sscanf(buf, "%hhx", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}

	mx50_proximity_write_i2c(PIC12LF1822_PROXDBNCDET, value);
	return size;
}

static SYSDEV_ATTR(proxdbncdet, 0644, prox_proxdbncdet_show, prox_proxdbncdet_store); 

static ssize_t prox_proxdbncrel_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	u8 value;

	mx50_proximity_read_i2c(&value, PIC12LF1822_PROXDBNCREL);
	return sprintf(buf, "%d\n", value);
}

static ssize_t prox_proxdbncrel_store(struct sys_device *dev, struct sysdev_attribute *attr, 
					const char *buf, size_t size)
{
	u8 value = 0;

	if (sscanf(buf, "%hhx", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}

	mx50_proximity_write_i2c(PIC12LF1822_PROXDBNCREL, value);
	return size;
}

static SYSDEV_ATTR(proxdbncrel, 0644, prox_proxdbncrel_show, prox_proxdbncrel_store); 

static ssize_t prox_proxavgen_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	u8 value;

	mx50_proximity_read_i2c(&value, PIC12LF1822_PROXAVGEN);
	return sprintf(buf, "%d\n", value);
}

static ssize_t prox_proxavgen_store(struct sys_device *dev, struct sysdev_attribute *attr, 
					const char *buf, size_t size)
{
	u8 value = 0;

	if (sscanf(buf, "%hhx", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}

	mx50_proximity_write_i2c(PIC12LF1822_PROXAVGEN, value);
	return size;
}

static SYSDEV_ATTR(proxavgen, 0644, prox_proxavgen_show, prox_proxavgen_store); 

static struct sysdev_class proximity_sysclass = {
	.name	= "proximity",
};

static struct sys_device device_proximity = {
	.id	= 0,
	.cls	= &proximity_sysclass,
};

extern void gpio_proximity_reset(void);

static int mx50_proximity_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mx50_proximity_info *info;
	u8 touch_status = 0;
	int error = 0;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}

	/* Reset Proximity */
	gpio_proximity_reset();

	client->addr = MX50_PROXIMITY_I2C_ADDR;
	i2c_set_clientdata(client, info);
	info->client = client;
	mx50_proximity_i2c_client = info->client;
	mx50_proximity_i2c_client->addr = MX50_PROXIMITY_I2C_ADDR;

	if (mx50_proximity_read_i2c(&touch_status, PIC12LF1822_FWVER) < 0)
		return -ENODEV;

	dump_regs();

	printk(KERN_INFO "Proximity Sensor Found, Firmware Rev: 0x%x\n", touch_status);
	
	/* /sys files */
	error = sysdev_class_register(&proximity_reg_sysclass);
	if (error) goto exit;
	error = sysdev_register(&device_proximity_reg);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity_reg, &attr_proximity_reg);
	if (error) goto exit;

	error = sysdev_class_register(&proximity_register_sysclass);
	if (error) goto exit;
	error = sysdev_register(&device_proximity_register);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity_register, &attr_proximity_register);
	if (error) goto exit;

	error = sysdev_class_register(&proximity_sysclass);
	if (error) goto exit;
	error = sysdev_register(&device_proximity);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_fwver);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_prox);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_bckgndavg);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_proxstate);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_proxthres);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_proxnodetect);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_wdtcon);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_proxscanrate);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_proxavgrate);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_proxtimeout);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_proxdbncdet);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_proxdbncrel);
	if (error) goto exit;
	error = sysdev_create_file(&device_proximity, &attr_proxavgen);
	if (error) goto exit;
	return 0;
exit:
	return -error;
}	

static int mx50_proximity_remove(struct i2c_client *client)
{
	struct mx50_proximity_info *info = i2c_get_clientdata(client);

	i2c_set_clientdata(client, info);

	sysdev_remove_file(&device_proximity_reg, &attr_proximity_reg);
	sysdev_remove_file(&device_proximity_register, &attr_proximity_register);
	sysdev_remove_file(&device_proximity, &attr_fwver);
	sysdev_remove_file(&device_proximity, &attr_prox);
	sysdev_remove_file(&device_proximity, &attr_bckgndavg);
	sysdev_remove_file(&device_proximity, &attr_proxstate);
	sysdev_remove_file(&device_proximity, &attr_proxthres);
	sysdev_remove_file(&device_proximity, &attr_proxnodetect);
	sysdev_remove_file(&device_proximity, &attr_wdtcon);
	sysdev_remove_file(&device_proximity, &attr_proxscanrate);
	sysdev_remove_file(&device_proximity, &attr_proxavgrate);
	sysdev_remove_file(&device_proximity, &attr_proxtimeout);
	sysdev_remove_file(&device_proximity, &attr_proxdbncdet);
	sysdev_remove_file(&device_proximity, &attr_proxdbncrel);
	sysdev_remove_file(&device_proximity, &attr_proxavgen);
	
	return 0;
}

static const struct i2c_device_id mx50_proximity_id[] =  {
	{ "MX50_Proximity", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mx50_proximity_id);

static struct i2c_driver proximity_i2c_driver = {
	.driver = {
			.name = DRIVER_NAME,
		},
	.probe = mx50_proximity_probe,
	.remove = mx50_proximity_remove,
	.id_table = mx50_proximity_id,
};

static void do_proximity_work(struct work_struct *dummy)
{
	int irq = gpio_proximity_int();

	printk(KERN_INFO "Received Proximity Sensor Interrupt\n");

	if (!gpio_proximity_detected()) {
		kobject_uevent(&proximity_misc_device.this_device->kobj, KOBJ_ONLINE);
		set_irq_type(irq, IRQF_TRIGGER_RISING);
	}
	else {
		kobject_uevent(&proximity_misc_device.this_device->kobj, KOBJ_OFFLINE);
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
	}

	enable_irq(irq);
}

static DECLARE_DELAYED_WORK(proximity_work, do_proximity_work);

static irqreturn_t proximity_irq(int irq, void *devid)
{
	disable_irq_nosync(irq);

	/* Debounce for 10 ms */
	schedule_delayed_work(&proximity_work, msecs_to_jiffies(PROXIMITY_THRESHOLD));
	
	return IRQ_HANDLED;
}

static int __init proximity_port_init(void)
{
	int err = 0, irq = gpio_proximity_int();

	printk(KERN_INFO "Initializing MX50 Yoshi Proximity Sensor\n");
	
	if (!gpio_proximity_detected())
		set_irq_type(irq, IRQF_TRIGGER_RISING);
	else
		set_irq_type(irq, IRQF_TRIGGER_FALLING);

	err = request_irq(irq, proximity_irq, 0, "MX50_Proximity", NULL);

	if (err != 0) {
		printk(KERN_ERR "IRQF_DISABLED: Could not get IRQ %d\n", irq);
		return err;
	}

	if (misc_register(&proximity_misc_device)) {
		printk (KERN_ERR "proximity_misc_device: Couldn't register device 10, "
				"%d.\n", PROXIMITY_PORT_MINOR);
		return -EBUSY;
	}

	err = i2c_add_driver(&proximity_i2c_driver);
	
	if (err) {
		printk(KERN_ERR "Yoshi Proximity: Could not add driver\n");
		free_irq(gpio_proximity_int(), NULL);
	}

	if (!gpio_proximity_detected())
		kobject_uevent(&proximity_misc_device.this_device->kobj, KOBJ_ONLINE);
	else
		kobject_uevent(&proximity_misc_device.this_device->kobj, KOBJ_OFFLINE);	

	return err;
}

static void proximity_port_cleanup(void)
{
	misc_deregister(&proximity_misc_device);
	free_irq(gpio_proximity_int(), NULL);
	i2c_del_driver(&proximity_i2c_driver);
}

static void __exit proximity_port_remove(void)
{
	proximity_port_cleanup();
}

module_init(proximity_port_init);
module_exit(proximity_port_remove);

MODULE_AUTHOR("Manish Lachwani");
MODULE_DESCRIPTION("MX50 Proximity Sensor");
MODULE_LICENSE("GPL");
