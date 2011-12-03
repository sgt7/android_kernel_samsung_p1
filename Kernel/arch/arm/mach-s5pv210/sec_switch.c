/*
 * UART/USB path switching driver for Samsung Electronics devices.
 *
 *  Copyright (C) 2010 Samsung Electronics
 *  Ikkeun Kim <iks.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <mach/param.h>
#include <linux/fsa9480.h>
#include <linux/sec_battery.h>
#include <asm/mach/arch.h>
#include <linux/regulator/consumer.h>
#include <mach/gpio.h>
#include <mach/gpio-p1.h>
#include <mach/sec_switch.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <linux/moduleparam.h>

struct sec_switch_struct {
	struct sec_switch_platform_data *pdata;
	int switch_sel;
	int uart_owner;
};

struct sec_switch_wq {
	struct delayed_work work_q;
	struct sec_switch_struct *sdata;
	struct list_head entry;
};

#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
extern void samsung_enable_function(int mode);
#endif
extern int askon_status;

extern struct device *switch_dev;
static int switchsel;
// Get SWITCH_SEL param value from kernel CMDLINE parameter.
__module_param_call("", switchsel, param_set_int, param_get_int, &switchsel, 0, 0444);
MODULE_PARM_DESC(switchsel, "Switch select parameter value.");


static void usb_switch_mode(struct sec_switch_struct *secsw, int mode)
{
	if(mode == SWITCH_PDA)
	{
		if(secsw->pdata && secsw->pdata->set_regulator)
			secsw->pdata->set_regulator(AP_VBUS_ON);
		mdelay(10);

		fsa9480_manual_switching(AUTO_SWITCH);
	}
	else  // SWITCH_MODEM
	{
		if(secsw->pdata && secsw->pdata->set_regulator)
			secsw->pdata->set_regulator(CP_VBUS_ON);
		mdelay(10);

		fsa9480_manual_switching(SWITCH_V_Audio_Port);
	}
}

/* for sysfs control (/sys/class/sec/switch/usb_sel) */
static ssize_t usb_sel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sec_switch_struct *secsw = dev_get_drvdata(dev);
	int usb_path = secsw->switch_sel & (int)(USB_SEL_MASK);

	return sprintf(buf, "USB Switch : %s\n", usb_path==SWITCH_PDA?"PDA":"MODEM");
}

static ssize_t usb_sel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_switch_struct *secsw = dev_get_drvdata(dev);

	printk("\n");

	if (sec_get_param_value)
		sec_get_param_value(__SWITCH_SEL, &(secsw->switch_sel));

	if(strncmp(buf, "PDA", 3) == 0 || strncmp(buf, "pda", 3) == 0) {
		usb_switch_mode(secsw, SWITCH_PDA);
//		usb_switching_value_update(SWITCH_PDA);
		secsw->switch_sel |= USB_SEL_MASK;
	}

	if(strncmp(buf, "MODEM", 5) == 0 || strncmp(buf, "modem", 5) == 0) {
		usb_switch_mode(secsw, SWITCH_MODEM);
//		usb_switching_value_update(SWITCH_MODEM);	
		secsw->switch_sel &= ~USB_SEL_MASK;
	}

//	switching_value_update();

	if (sec_set_param_value)
		sec_set_param_value(__SWITCH_SEL, &(secsw->switch_sel));

	// update shared variable.
	if(secsw->pdata && secsw->pdata->set_switch_status)
		secsw->pdata->set_switch_status(secsw->switch_sel);

	return size;
}

static DEVICE_ATTR(usb_sel, S_IRUGO |S_IWGRP | S_IWUSR, usb_sel_show, usb_sel_store);


static ssize_t uart_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sec_switch_struct *secsw = dev_get_drvdata(dev);

	if (secsw->uart_owner)
		return sprintf(buf, "[UART Switch] Current UART owner = PDA \n");
	else			
		return sprintf(buf, "[UART Switch] Current UART owner = MODEM \n");
}

static ssize_t uart_switch_store(struct device *dev, struct device_attribute *attr,	const char *buf, size_t size)
{	
	struct sec_switch_struct *secsw = dev_get_drvdata(dev);
	
	if (sec_get_param_value)
		sec_get_param_value(__SWITCH_SEL, &(secsw->switch_sel));

	if (strncmp(buf, "PDA", 3) == 0 || strncmp(buf, "pda", 3) == 0) {
		gpio_set_value(GPIO_UART_SEL, 1);
//		uart_switching_value_update(SWITCH_PDA);
		secsw->uart_owner = 1;
		secsw->switch_sel |= UART_SEL_MASK;
		printk("[UART Switch] Path : PDA\n");	
	}	

	if (strncmp(buf, "MODEM", 5) == 0 || strncmp(buf, "modem", 5) == 0) {
		gpio_set_value(GPIO_UART_SEL, 0);
//		uart_switching_value_update(SWITCH_MODEM);
		secsw->uart_owner = 0;
		secsw->switch_sel &= ~UART_SEL_MASK;
		printk("[UART Switch] Path : MODEM\n");	
	}

//	switching_value_update();

	if (sec_set_param_value)
		sec_set_param_value(__SWITCH_SEL, &(secsw->switch_sel));

	// update shared variable.
	if(secsw->pdata && secsw->pdata->set_switch_status)
		secsw->pdata->set_switch_status(secsw->switch_sel);

	return size;
}

static DEVICE_ATTR(uart_sel, S_IRUGO | S_IWGRP | S_IWUSR, uart_switch_show, uart_switch_store);


// for sysfs control (/sys/class/sec/switch/usb_state)
static ssize_t usb_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sec_switch_struct *secsw = dev_get_drvdata(dev);
	int cable_state = CABLE_TYPE_NONE;

	if(secsw->pdata && secsw->pdata->get_cable_status)
		cable_state = secsw->pdata->get_cable_status();

	return sprintf(buf, "%s\n", (cable_state==CABLE_TYPE_USB)?"USB_STATE_CONFIGURED":"USB_STATE_NOTCONFIGURED");
} 

static ssize_t usb_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("\n");
	return size;
}

static DEVICE_ATTR(usb_state, S_IRUGO | S_IWGRP | S_IWUSR, usb_state_show, usb_state_store);


// for sysfs control (/sys/class/sec/switch/disable_vbus)
static ssize_t disable_vbus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("\n");
	return 0;
} 

static ssize_t disable_vbus_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_switch_struct *secsw = dev_get_drvdata(dev);
	printk("%s\n", __func__);
	if(secsw->pdata && secsw->pdata->set_regulator)
		secsw->pdata->set_regulator(AP_VBUS_OFF);

	return size;
}

static DEVICE_ATTR(disable_vbus, S_IRUGO | S_IWGRP | S_IWUSR, disable_vbus_show, disable_vbus_store);


static void sec_switch_init_work(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct sec_switch_wq *wq = container_of(dw, struct sec_switch_wq, work_q);
	struct sec_switch_struct *secsw = wq->sdata;
	int usb_sel = 0;
	int uart_sel = 0;
	int ret = 0;
	int samsung_kies_sel,ums_sel,mtp_sel,vtp_sel,askon_sel;

//	printk("%s : called!!\n", __func__);
         if (sec_get_param_value &&
            secsw->pdata &&
            secsw->pdata->set_regulator &&
            secsw->pdata->get_phy_init_status &&
            secsw->pdata->get_phy_init_status()) {
                sec_get_param_value(__SWITCH_SEL, &secsw->switch_sel);
                cancel_delayed_work(&wq->work_q);
         }
	 //else if (!regulator_get(NULL, "vbus_ap")  || !(secsw->pdata->get_phy_init_status())) {
         else  {
		schedule_delayed_work(&wq->work_q, msecs_to_jiffies(1000));
		return ;
	       }

	if(secsw->pdata && secsw->pdata->get_regulator) {
		ret = secsw->pdata->get_regulator();
		if(ret != 0) {
			pr_err("%s : failed to get regulators\n", __func__);
			return ;
		}
	}

	// init shared variable.
	if(secsw->pdata && secsw->pdata->set_switch_status)
		secsw->pdata->set_switch_status(secsw->switch_sel);

	usb_sel = secsw->switch_sel & (int)(USB_SEL_MASK);
	uart_sel = (secsw->switch_sel & (int)(UART_SEL_MASK)) >> 1;

	printk("%s : initial usb_sel(%d), uart_sel(%d)\n", __func__, usb_sel, uart_sel);

	// init UART/USB path.
	if(usb_sel) {
		usb_switch_mode(secsw, SWITCH_PDA);
	}
	else {
		usb_switch_mode(secsw, SWITCH_MODEM);
	}

	if(uart_sel) {
		gpio_set_value(GPIO_UART_SEL, 1);
		secsw->uart_owner = 1;
	}
	else {
		gpio_set_value(GPIO_UART_SEL, 0);
		secsw->uart_owner = 0;
	}

}

static int sec_switch_probe(struct platform_device *pdev)
{
	struct sec_switch_struct *secsw;
	struct sec_switch_platform_data *pdata = pdev->dev.platform_data;
	struct sec_switch_wq *wq;


	if (!pdata) {
		pr_err("%s : pdata is NULL.\n", __func__);
		return -ENODEV;
	}

	secsw = kzalloc(sizeof(struct sec_switch_struct), GFP_KERNEL);
	if (!secsw) {
		pr_err("%s : failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	printk("%s : *** switch_sel (0x%x)\n", __func__, switchsel);

	secsw->pdata = pdata;
	secsw->switch_sel = 1;

	dev_set_drvdata(switch_dev, secsw);

	// create sysfs files.
	if (device_create_file(switch_dev, &dev_attr_uart_sel) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_uart_sel.attr.name);

	if (device_create_file(switch_dev, &dev_attr_usb_sel) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_usb_sel.attr.name);

	if (device_create_file(switch_dev, &dev_attr_usb_state) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_usb_state.attr.name);

	if (device_create_file(switch_dev, &dev_attr_disable_vbus) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_disable_vbus.attr.name);

	// run work queue
	wq = kmalloc(sizeof(struct sec_switch_wq), GFP_ATOMIC);
	if (wq) {
		wq->sdata = secsw;
		INIT_DELAYED_WORK(&wq->work_q, sec_switch_init_work);
		schedule_delayed_work(&wq->work_q, msecs_to_jiffies(100));
	}
	else
		return -ENOMEM;

	return 0;
}

static int sec_switch_remove(struct platform_device *pdev)
{
	struct sec_switch_struct *secsw = dev_get_drvdata(&pdev->dev);
	
	kfree(secsw);

	return 0;
}

static struct platform_driver sec_switch_driver = {
	.probe = sec_switch_probe,
	.remove = sec_switch_remove,
	.driver = {
			.name = "sec_switch",
			.owner = THIS_MODULE,
	},
};

static int __init sec_switch_init(void)
{
	return platform_driver_register(&sec_switch_driver);
}

static void __exit sec_switch_exit(void)
{
	platform_driver_unregister(&sec_switch_driver);
}

module_init(sec_switch_init);
module_exit(sec_switch_exit);

MODULE_AUTHOR("Ikkeun Kim <iks.kim@samsung.com>");
MODULE_DESCRIPTION("Samsung Electronics Corp Switch driver");
MODULE_LICENSE("GPL");
