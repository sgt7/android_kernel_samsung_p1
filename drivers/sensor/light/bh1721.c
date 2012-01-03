
/*
 *  bh1721fvc.c - Ambient Light Sensor IC
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Donggeun Kim <dg77.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/bh1721.h>

#define ON              1
#define OFF				0

struct class *lightsensor_class;
struct device *switch_cmd_dev;
static bool light_enable = OFF;


#define NUM_OF_BYTES_WRITE	1
#define NUM_OF_BYTES_READ	2

#define MAX_LEVEL 	8
#define MAX_LUX		65528

const unsigned char POWER_DOWN = 0x00;
const unsigned char POWER_ON = 0x01;
const unsigned char AUTO_RESOLUTION_1 = 0x10;
const unsigned char AUTO_RESOLUTION_2 = 0x20;
const unsigned char H_RESOLUTION_1 = 0x12;
const unsigned char H_RESOLUTION_2 = 0x22;
const unsigned char L_RESOLUTION_1 = 0x13;
const unsigned char L_RESOLUTION_2 = 0x23;
const unsigned char L_RESOLUTION_3 = 0x16;
const unsigned char L_RESOLUTION_4 = 0x26;

enum {
	LIGHT_ENABLED = BIT(0),
	PROXIMITY_ENABLED = BIT(1),
};

typedef enum 
{
				DOWNWARD = 0,
				UPSIDE,
				EQUAL
}Direction;

struct bh1721_data;

/* driver data */
struct bh1721_data {
	struct input_dev *light_input_dev;
	struct bh1721_platform_data *pdata;
	void (*reset) (void);
	struct i2c_client *i2c_client;
	struct work_struct work_light;
	struct hrtimer timer;
	ktime_t light_poll_delay;
	u8 power_state;
	struct mutex power_lock;
	struct workqueue_struct *wq;
	unsigned char illuminance_data[2];
};

static unsigned int prev_level = 0;
static unsigned int prev_lux = 0;
static unsigned int LUX_TO_LEVEL[MAX_LEVEL][3] = 
{ 
										{0, 10, 5},			// {Downward Threshold, Upside Threshold, Representative Value for Step}
										{5, 48, 22},
										{34, 188, 75},
										{116, 520, 220},
										{340, 1880, 800},
										{1160, 4500, 2800},
										{3400, 6000, 4800},
										{5000, MAX_LUX, 6000} 
};
										
static int bh1721_write_command(struct i2c_client *client, const char *command)
{
	return i2c_master_send(client, command, NUM_OF_BYTES_WRITE);
}

static int bh1721_read_value(struct i2c_client *client, char *buf)
{
	return i2c_master_recv(client, buf, NUM_OF_BYTES_READ);
}

static unsigned int get_next_level(unsigned int cur_lux)
{
	int i = 0;
	Direction dir;
	unsigned int rep_lux = 0;
	
	// get direction
	if(cur_lux > prev_lux)
		dir = UPSIDE;
	else if(cur_lux < prev_lux)
		dir = DOWNWARD;
	else
		dir = EQUAL;
		
	// Check how many steps are changed.
	if(dir == UPSIDE)
	{
		for(i = prev_level; i < MAX_LEVEL -1; i++)
			if(cur_lux >= LUX_TO_LEVEL[i][UPSIDE])
				continue;
			else
				break;
		
		rep_lux = LUX_TO_LEVEL[i][2];
		prev_lux = cur_lux;
		prev_level = i;
	}
	else if(dir == DOWNWARD)
	{
		for(i = prev_level; i > 0; i--)
		if(cur_lux < LUX_TO_LEVEL[i][DOWNWARD])
			continue;
		else
			break;
		
		rep_lux = LUX_TO_LEVEL[i][2];
		prev_lux = cur_lux;
		prev_level = i;
	}
	else
	{
		rep_lux = LUX_TO_LEVEL[prev_level][2];
	}
	
	return rep_lux;
}



static void bh1721_light_enable(struct bh1721_data *bh1721)
{
	printk("[Light Sensor] starting poll timer, delay %lldns\n",
		    ktime_to_ns(bh1721->light_poll_delay));
	hrtimer_start(&bh1721->timer, bh1721->light_poll_delay, HRTIMER_MODE_REL);

	if((bh1721_write_command(bh1721->i2c_client, &POWER_ON))>0)
		printk("[Light Sensor] Power ON");
}

static void bh1721_light_disable(struct bh1721_data *bh1721)
{
	printk("[Light Sensor] cancelling poll timer\n");
	hrtimer_cancel(&bh1721->timer);
	cancel_work_sync(&bh1721->work_light);

	if((bh1721_write_command(bh1721->i2c_client, &POWER_DOWN))>0)
		printk("[Light Sensor] Power off");
}

static ssize_t poll_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bh1721_data *bh1721 = dev_get_drvdata(dev);
	return sprintf(buf, "%lld\n", ktime_to_ns(bh1721->light_poll_delay));
}


static ssize_t poll_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct bh1721_data *bh1721 = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	printk("[Light Sensor] new delay = %lldns, old delay = %lldns\n",
		    new_delay, ktime_to_ns(bh1721->light_poll_delay));
	mutex_lock(&bh1721->power_lock);
	if (new_delay != ktime_to_ns(bh1721->light_poll_delay)) {
		bh1721->light_poll_delay = ns_to_ktime(new_delay);
		if (bh1721->power_state & LIGHT_ENABLED) {
			bh1721_light_disable(bh1721);
			bh1721_light_enable(bh1721);
		}
	}
	mutex_unlock(&bh1721->power_lock);

	return size;
}

static ssize_t light_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct bh1721_data *bh1721 = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", (bh1721->power_state & LIGHT_ENABLED) ? 1 : 0);
}


static ssize_t light_enable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct bh1721_data *bh1721 = dev_get_drvdata(dev);
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	mutex_lock(&bh1721->power_lock);
	printk("[Light Sensor] new_value = %d, old state = %d\n", new_value, (bh1721->power_state & LIGHT_ENABLED) ? 1 : 0);
	if (new_value && !(bh1721->power_state & LIGHT_ENABLED)) {
		bh1721->power_state |= LIGHT_ENABLED;
		bh1721_light_enable(bh1721);
	} else if (!new_value && (bh1721->power_state & LIGHT_ENABLED)) {
		bh1721_light_disable(bh1721);
		bh1721->power_state &= ~LIGHT_ENABLED;
	}
	mutex_unlock(&bh1721->power_lock);
	return size;
}

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   poll_delay_show, poll_delay_store);

static struct device_attribute dev_attr_light_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	       light_enable_show, light_enable_store);


/*For Factory Test Mode*/
static ssize_t lightsensor_file_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	struct bh1721_data *bh1721 = dev_get_drvdata(dev);
	unsigned int result;
	/* 
	 * POWER ON command is possible to omit.
	 */
	printk("[%s] lightsensor_file_state_show \n", __func__);	

	bh1721_write_command(bh1721->i2c_client, &POWER_ON);
	bh1721_write_command(bh1721->i2c_client, &H_RESOLUTION_2);

	/* Maximum measurement time */
	msleep(180);
	
	bh1721_read_value(bh1721->i2c_client, bh1721->illuminance_data);
	bh1721_write_command(bh1721->i2c_client, &POWER_DOWN);

	result = bh1721->illuminance_data[0] << 8 | bh1721->illuminance_data[1];
	result = (result*10)/12;
	
	/* apply hysteresis */
	printk("[%s] original lux = %u\n", __func__, result);	
	result = get_next_level(result);	
	printk("[%s] representative lux = %u\n", __func__, result);
	
	return sprintf(buf, "%d\n", result);
}
static DEVICE_ATTR(lightsensor_file_state, S_IRUGO | S_IWUSR | S_IWGRP, lightsensor_file_state_show,NULL);

static ssize_t bh1721_show_illuminance(struct device *dev, struct device_attribute *attr, char *buf)
{	
	struct bh1721_data *bh1721 = dev_get_drvdata(dev);
	unsigned int result;

	/* 
	 * POWER ON command is possible to omit.
	 */
	if((bh1721_write_command(bh1721->i2c_client, &POWER_ON))>0)
		light_enable = ON;
	bh1721_write_command(bh1721->i2c_client, &H_RESOLUTION_2);

	/* Maximum measurement time */
	msleep(180);
	bh1721_read_value(bh1721->i2c_client, bh1721->illuminance_data);
	if((bh1721_write_command(bh1721->i2c_client, &POWER_DOWN))>0)
		light_enable = OFF;
	result = bh1721->illuminance_data[0] << 8 | bh1721->illuminance_data[1];
	result = (result*10)/12;
	return sprintf(buf, "%d\n", result);
}

static DEVICE_ATTR(illuminance, S_IRUGO | S_IWUSR | S_IWGRP, bh1721_show_illuminance, NULL);

static ssize_t lightsensor_file_illuminance_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{	
	unsigned int result;
	struct bh1721_data *bh1721 = dev_get_drvdata(dev);

	/* 
	 * POWER ON command is possible to omit.
	 */
	if((bh1721_write_command(bh1721->i2c_client, &POWER_ON))>0)
		light_enable = ON;
	bh1721_write_command(bh1721->i2c_client, &H_RESOLUTION_2);

	/* Maximum measurement time */
	msleep(180);
	bh1721_read_value(bh1721->i2c_client, bh1721->illuminance_data);
	if((bh1721_write_command(bh1721->i2c_client, &POWER_DOWN))>0)
		light_enable = OFF;
	result = bh1721->illuminance_data[0] << 8 | bh1721->illuminance_data[1];
	result = (result*10)/12;
	return sprintf(buf, "%d\n", result);
}

static DEVICE_ATTR(lightsensor_file_illuminance, S_IRUGO | S_IWUSR | S_IWGRP, lightsensor_file_illuminance_show,NULL);

/* for light sensor on/off control from platform */
static ssize_t lightsensor_file_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{		
	return sprintf(buf, "%d\n", light_enable);
}
static ssize_t lightsensor_file_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct bh1721_data *bh1721 = dev_get_drvdata(dev);
	int mode,ret;	
	ret=0;
	sscanf(buf,"%d",&mode);	
	if(mode ==0)
	{
		bh1721_write_command(bh1721->i2c_client, &POWER_DOWN);
		light_enable = OFF;
	}
	else if(mode ==1)		
	{	
		bh1721_write_command(bh1721->i2c_client, &POWER_ON);
		light_enable = ON;
	}
	return count;		
}
static DEVICE_ATTR(lightsensor_file_cmd, S_IRUGO | S_IWUSR | S_IWGRP, lightsensor_file_cmd_show, lightsensor_file_cmd_store);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_light_enable.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_illuminance.attr,
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};

static unsigned int lightsensor_get_adcvalue(struct bh1721_data *bh1721)
{
	unsigned int result;
	bh1721_write_command(bh1721->i2c_client, &H_RESOLUTION_2);

	/* Maximum measurement time */
	msleep(180);
	bh1721_read_value(bh1721->i2c_client, bh1721->illuminance_data);
	
	result = bh1721->illuminance_data[0] << 8 | bh1721->illuminance_data[1];
	result = (result*10)/12;

	result = get_next_level(result);	
	return result;
}

static void bh1721_work_func_light(struct work_struct *work)
{
	struct bh1721_data *bh1721 = container_of(work, struct bh1721_data, work_light);
	unsigned int adc = lightsensor_get_adcvalue(bh1721);
	input_report_abs(bh1721->light_input_dev, ABS_MISC, adc);
	input_sync(bh1721->light_input_dev);
}

/* This function is for light sensor.  It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */
static enum hrtimer_restart bh1721_timer_func(struct hrtimer *timer)
{
	struct bh1721_data *bh1721 = container_of(timer, struct bh1721_data, timer);
	queue_work(bh1721->wq, &bh1721->work_light);
	hrtimer_forward_now(&bh1721->timer, bh1721->light_poll_delay);
	return HRTIMER_RESTART;
}

static int bh1721_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret = -ENODEV;
	struct input_dev *input_dev;
	struct bh1721_data *bh1721;
	struct bh1721_platform_data *pdata = client->dev.platform_data;

	if (!pdata) {
		pr_err("%s: missing pdata!\n", __func__);
		return ret;
	}
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c functionality check failed!\n", __func__);
		return ret;
	}

	bh1721 = kzalloc(sizeof(struct bh1721_data), GFP_KERNEL);
	if (!bh1721) {
		pr_err("%s: failed to alloc memory for module data\n",
		       __func__);
		return -ENOMEM;
	}

	
	bh1721->pdata = pdata;
	bh1721->reset = pdata->reset;
	bh1721->i2c_client = client;
	i2c_set_clientdata(client, bh1721);
	
	if (bh1721->reset)
		bh1721->reset();

	mutex_init(&bh1721->power_lock);

	/* hrtimer settings.  we poll for light values using a timer. */
	hrtimer_init(&bh1721->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bh1721->light_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
	bh1721->timer.function = bh1721_timer_func;

	/* the timer just fires off a work queue request.  we need a thread
	   to read the i2c (can be slow and blocking). */
	bh1721->wq = create_singlethread_workqueue("bh1721_wq");
	if (!bh1721->wq) {
		ret = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}
	/* this is the thread function we run on the work queue */
	INIT_WORK(&bh1721->work_light, bh1721_work_func_light);

	/* allocate lightsensor-level input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		ret = -ENOMEM;
		goto err_input_allocate_device_light;
	}
	input_set_drvdata(input_dev, bh1721);
	input_dev->name = "lightsensor-level";
	input_set_capability(input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(input_dev, ABS_MISC, 0, 1, 0, 0);

	ret = input_register_device(input_dev);
	if (ret < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_light;
	}
	bh1721->light_input_dev = input_dev;

	ret = sysfs_create_group(&input_dev->dev.kobj,&light_attribute_group);
	if (ret) {
		printk("Creating bh1721 attribute group failed");
		goto error_device;
	}

	/* set sysfs for light sensor test mode*/
	lightsensor_class = class_create(THIS_MODULE, "lightsensor");
	if (IS_ERR(lightsensor_class))
	{
		printk("Failed to create class(lightsensor)!\n");
		goto error_device;
	}
	switch_cmd_dev = device_create(lightsensor_class, NULL, 0, NULL, "switch_cmd");
	if (IS_ERR(switch_cmd_dev))
	{
		printk("Failed to create device(switch_cmd_dev)!\n");
		goto DESTROY_CLASS;
	}
	if (device_create_file(switch_cmd_dev, &dev_attr_lightsensor_file_state) < 0)
	{
		printk("Failed to create device file(%s)!\n", dev_attr_lightsensor_file_state.attr.name);
		device_remove_file(switch_cmd_dev, &dev_attr_lightsensor_file_state);
		goto DESTROY_DEVICE;
	}
	if (device_create_file(switch_cmd_dev, &dev_attr_lightsensor_file_illuminance) < 0)
	{
		printk("Failed to create device file(%s)!\n", dev_attr_lightsensor_file_illuminance.attr.name);
		device_remove_file(switch_cmd_dev, &dev_attr_lightsensor_file_illuminance);
		goto DESTROY_DEVICE;
	}
	if (device_create_file(switch_cmd_dev, &dev_attr_lightsensor_file_cmd) < 0)
	{
		printk("Failed to create device file(%s)!\n", dev_attr_lightsensor_file_cmd.attr.name);
		goto DESTROY_DEVICE;
	}
	dev_set_drvdata(switch_cmd_dev, bh1721);

	printk("[%s]: Light Sensor probe complete.", __func__);
	
	goto done;

	/* error, unwind it all */
DESTROY_DEVICE:
	device_destroy(lightsensor_class,0);
DESTROY_CLASS:
	class_destroy(lightsensor_class);	
error_device:
	sysfs_remove_group(&client->dev.kobj, &light_attribute_group);
err_input_register_device_light:
	input_unregister_device(bh1721->light_input_dev);
err_input_allocate_device_light:
	destroy_workqueue(bh1721->wq);
err_create_workqueue:
	mutex_destroy(&bh1721->power_lock);
	kfree(bh1721);
done:
	return ret;
}

static int bh1721_suspend(struct device *dev)
{
	/* We disable power only if proximity is disabled.  If proximity
	   is enabled, we leave power on because proximity is allowed
	   to wake up device.  We remove power without changing
	   bh1721->power_state because we use that state in resume
	*/
	struct i2c_client *client = to_i2c_client(dev);
	struct bh1721_data *bh1721 = i2c_get_clientdata(client);
	if (bh1721->power_state & LIGHT_ENABLED) bh1721_light_disable(bh1721);
	
	return 0;
}

static int bh1721_resume(struct device *dev)
{	
	/* Turn power back on if we were before suspend. */
	struct i2c_client *client = to_i2c_client(dev);
	struct bh1721_data *bh1721 = i2c_get_clientdata(client);

	if (bh1721->reset)
		bh1721->reset();
	if (bh1721->power_state & LIGHT_ENABLED)
		bh1721_light_enable(bh1721);
	return 0;

}

static int bh1721_i2c_remove(struct i2c_client *client)
{
	struct bh1721_data *bh1721 = i2c_get_clientdata(client);
	sysfs_remove_group(&bh1721->light_input_dev->dev.kobj, &light_attribute_group);
	input_unregister_device(bh1721->light_input_dev);
	device_remove_file(switch_cmd_dev, &dev_attr_lightsensor_file_cmd);
	device_remove_file(switch_cmd_dev, &dev_attr_lightsensor_file_illuminance);
	device_remove_file(switch_cmd_dev, &dev_attr_lightsensor_file_state);
	device_destroy(lightsensor_class,0);
	class_destroy(lightsensor_class);
	
	if (bh1721->power_state) 
	{
		bh1721->power_state = 0;
		if (bh1721->power_state & LIGHT_ENABLED)
			bh1721_light_disable(bh1721);
	}
	destroy_workqueue(bh1721->wq);
	mutex_destroy(&bh1721->power_lock);
	kfree(bh1721);
	return 0;
}

static const struct i2c_device_id bh1721_device_id[] = 
{
	{"bh1721", 0},
	{}
}
MODULE_DEVICE_TABLE(i2c, bh1721_device_id);

static const struct dev_pm_ops bh1721_pm_ops = 
{	
	.suspend = bh1721_suspend,
	.resume = bh1721_resume
};


static struct i2c_driver bh1721_i2c_driver = {
	.driver = {
		.name = "bh1721",
		.owner = THIS_MODULE,
		.pm = &bh1721_pm_ops
	},
	.probe		= bh1721_i2c_probe,
	.remove		= bh1721_i2c_remove,
	.id_table	= bh1721_device_id,
};

static int __init bh1721_init(void)
{
	return i2c_add_driver(&bh1721_i2c_driver);
}

static void __exit bh1721_exit(void)
{
	i2c_del_driver(&bh1721_i2c_driver);
}
module_init(bh1721_init);
module_exit(bh1721_exit);

MODULE_AUTHOR("mjchen@sta.samsung.com");
MODULE_DESCRIPTION("Optical Sensor driver for bh1721");
MODULE_LICENSE("GPL");

