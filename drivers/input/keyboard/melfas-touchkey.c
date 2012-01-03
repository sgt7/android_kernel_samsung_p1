/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <asm/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#if defined(CONFIG_MACH_S5PC110_ARIES)
#include <mach/gpio-aries.h>
#elif defined(CONFIG_MACH_P1)
#include <mach/gpio-p1.h>
#elif defined(CONFIG_MACH_S5PC110_CRESPO)
#include <mach/gpio-crespo.h>
#endif

/*
Melfas touchkey register
*/
#define KEYCODE_REG 0x00
#define FIRMWARE_VERSION 0x01
#define TOUCHKEY_MODULE_VERSION 0x02
#define TOUCHKEY_ADDRESS	0x20

#define UPDOWN_EVENT_BIT 0x08
#define KEYCODE_BIT 0x07

/* keycode value */
#define RESET_KEY 0x01
#define SWTICH_KEY 0x02
#define OK_KEY 0x03
#define END_KEY 0x04

#define I2C_M_WR 0 /* for i2c */

#define IRQ_TOUCH_INT (IRQ_EINT_GROUP22_BASE + 1) 

#define DEVICE_NAME "melfas-touchkey"

static int touchkey_keycode[] = {NULL, KEY_BACK, KEY_MENU, KEY_ENTER, KEY_END};
static struct input_dev *touchkey_dev;

struct workqueue_struct *touchkey_wq;



static const struct i2c_device_id melfas_touchkey_id[] = {
       { "melfas_touchkey" , 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, melfas_touchkey_id);


static void init_hw(void);
static int i2c_touchkey_probe(struct i2c_client *client,const struct i2c_device_id *id);

struct i2c_driver touchkey_i2c_driver = {
	.driver = {
		.name = "melfas_touchkey",
	},
	.id_table = melfas_touchkey_id,
	.probe =i2c_touchkey_probe , 
};

struct i2c_touchkey_driver {
        struct i2c_client *client;
        struct input_dev *input_dev;
        struct work_struct work;
        struct early_suspend    early_suspend;
};
struct i2c_touchkey_driver *touchkey_driver = NULL;


static char i2c_touchkey_read(u8 reg, u8 *val, unsigned int len)
{
	int 	 err;
	struct 	 i2c_msg msg[1];
	
	unsigned char data[1];
	if( (touchkey_driver->client == NULL) || (!touchkey_driver->client->adapter) )
	{
		return -ENODEV;
	}
	
	msg->addr 	= touchkey_driver->client->addr;
	msg->flags = I2C_M_RD;
	msg->len   = len;
	msg->buf   = val;
	err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);

	if (err >= 0) 
	{
		return 0;
	}
	printk("%s %d i2c transfer error\n", __func__, __LINE__);/* add by inter.park */

	return err;

}
#if 0
static char i2c_touchkey_write( u8 reg, u8 *val )
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	if( (touchkey_driver->client == NULL) || (!touchkey_driver->client->adapter) ){
		return -ENODEV;
	}
	
	data[0] = reg;
	data[1] = *val;

	msg->addr = touchkey_driver->client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;
	
	err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);

	if (err >= 0) return 0;

	printk("%s %d i2c transfer error\n", __func__, __LINE__);
	return err;
}
#endif


void  touchkey_work_func(struct work_struct * p)
{
	u8 data;

	if(i2c_touchkey_read(KEYCODE_REG, &data, 1) < 0)
	{
		printk("%s %d i2c transfer error\n", __func__, __LINE__);
		return;
	}

	if(data & UPDOWN_EVENT_BIT)
	{
		input_report_key(touchkey_driver->input_dev, touchkey_keycode[data & KEYCODE_BIT], 0);
		input_sync(touchkey_driver->input_dev);
	}
	else
	{
		input_report_key(touchkey_driver->input_dev, touchkey_keycode[data & KEYCODE_BIT],1);
		input_sync(touchkey_driver->input_dev);
	}

//	enable_irq(IRQ_TOUCH_INT);
	return ;
}

static irqreturn_t touchkey_interrupt(int irq, void *dummy)
{

	u8 data;

//	disable_irq(IRQ_TOUCH_INT);

	queue_work(touchkey_wq, &touchkey_driver->work);
	
	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_touchkey_early_suspend(struct early_suspend *h)
{
	int err ;
	disable_irq(IRQ_TOUCH_INT);

	/*reset the gpio's*/
	err=gpio_request(_3_GPIO_TOUCH_EN,"_3_GPIO_TOUCH_EN");
	if (err)
	{
		printk("_3_GPIO_TOUCH_EN GPIO Failed\n");
		return ;
	}
	gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
	gpio_free(_3_GPIO_TOUCH_EN);

	err=gpio_request(_3_GPIO_TOUCH_CE,"_3_GPIO_TOUCH_CE");
	if (err)
	{
		printk("_3_GPIO_TOUCH_CE GPIO failed\n");
		return ;
	}
	gpio_direction_output(_3_GPIO_TOUCH_CE, 0);
	gpio_free(_3_GPIO_TOUCH_CE);
}

static void melfas_touchkey_early_resume(struct early_suspend *h)
{
	init_hw();
	enable_irq(IRQ_TOUCH_INT);
}
#endif	// End of CONFIG_HAS_EARLYSUSPEND

extern int mcsdl_download_binary_data(void);
static int i2c_touchkey_probe(struct i2c_client *client,const struct i2c_device_id *id)
{

//	struct i2c_touchkey_driver * state;
	struct device *dev = &client->dev;
	struct input_dev *input_dev;
	int err = 0;
	

	printk("melfas i2c_touchkey_probe\n");
	
	touchkey_driver = kzalloc(sizeof(struct i2c_touchkey_driver),GFP_KERNEL);
	if (touchkey_driver == NULL) {
                dev_err(dev, "failed to create our state\n");
                return -ENOMEM;
        }
	
	touchkey_driver->client = client;
	
	touchkey_driver->client->irq=IRQ_TOUCH_INT ; 
	strlcpy(touchkey_driver->client->name, "melfas-touchkey", I2C_NAME_SIZE);

       // i2c_set_clientdata(client, state);

	input_dev = input_allocate_device();
	
	if (!input_dev)
		return -ENOMEM;

	touchkey_driver->input_dev = input_dev;
	
	input_dev->name = DEVICE_NAME;
	input_dev->phys = "melfas-touchkey/input0";
	input_dev->id.bustype = BUS_HOST;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(touchkey_keycode[1], input_dev->keybit);
	set_bit(touchkey_keycode[2], input_dev->keybit);
	set_bit(touchkey_keycode[3], input_dev->keybit);
	set_bit(touchkey_keycode[4], input_dev->keybit);


	err = input_register_device(input_dev);
	if (err) {
		input_free_device(input_dev);
		return err;
	}	

	touchkey_wq = create_singlethread_workqueue("melfas_touchkey_wq");
	if (!touchkey_wq)
		return -ENOMEM;

	INIT_WORK(&touchkey_driver->work, touchkey_work_func);
	
	if (request_irq(IRQ_TOUCH_INT, touchkey_interrupt, IRQF_DISABLED, DEVICE_NAME, touchkey_driver)) {
                printk(KERN_ERR "%s Can't allocate irq ..\n", __FUNCTION__);
                return -EBUSY;
        }


	#ifdef CONFIG_HAS_EARLYSUSPEND
	touchkey_driver->early_suspend.suspend = melfas_touchkey_early_suspend;
	touchkey_driver->early_suspend.resume = melfas_touchkey_early_resume;
	register_early_suspend(&touchkey_driver->early_suspend);
	#endif /* CONFIG_HAS_EARLYSUSPEND */ 

	u8 data;

        if(i2c_touchkey_read(KEYCODE_REG, &data, 1) < 0)
        {
                printk("%s %d i2c transfer error\n", __func__, __LINE__);
                return;
        }

	
	return 0;
}

static void init_hw(void)
{
	int err ;

	err=gpio_request(_3_GPIO_TOUCH_EN,"_3_GPIO_TOUCH_EN");
	if (err)
	{
		printk("_3_GPIO_TOUCH_EN GPIO Failed\n");
		return ;
	}

	gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
	mdelay(1);
	gpio_free(_3_GPIO_TOUCH_EN);

	err=gpio_request(_3_GPIO_TOUCH_CE,"_3_GPIO_TOUCH_CE");
	if (err)
	{
		printk("_3_GPIO_TOUCH_CE GPIO failed\n");
		return ;
	}
	gpio_direction_output(_3_GPIO_TOUCH_CE, 1);
//	msleep(50);
	gpio_free(_3_GPIO_TOUCH_CE);

	set_irq_type(IRQ_TOUCH_INT, IRQ_TYPE_EDGE_FALLING);
	s3c_gpio_cfgpin(S5PV210_GPJ4(1), S3C_GPIO_SFN(0xf));
}


int touchkey_update_open (struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t touchkey_update_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

extern int mcsdl_download_binary_file(unsigned char *pData, unsigned short nBinary_length);
ssize_t touchkey_update_write (struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	unsigned char * pdata;

	disable_irq(IRQ_TOUCH_INT);
	printk("count = %d\n",count);
	pdata = kzalloc(count, GFP_KERNEL);
	if(pdata ==NULL)
	{
		printk("memory allocate fail \n");
		return 0;
	}
	if (copy_from_user(pdata, buf, count))
	{
		printk("copy fail \n");
		kfree(pdata);
		return 0;
	}
	
	mcsdl_download_binary_file((unsigned char *)pdata, (unsigned short)count);
	kfree(pdata);

	init_hw();
	enable_irq(IRQ_TOUCH_INT);
	return count;
}

int touchkey_update_release (struct inode *inode, struct file *filp)
{
	return 0;
}


struct file_operations touchkey_update_fops =
{
	.owner   = THIS_MODULE,
	.read    = touchkey_update_read,
	.write   = touchkey_update_write,
	.open    = touchkey_update_open,
	.release = touchkey_update_release,
};

static struct miscdevice touchkey_update_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "melfas_touchkey",
	.fops = &touchkey_update_fops,
};

static int HWREV=0xA;
static int __init touchkey_init(void)
{
	int ret = 0;
	int retry=10;

	if(HWREV >= 0xA)
	{
		touchkey_keycode[2] = KEY_ENTER;
	}

	printk("melfas touchkey_init\n");
	
	ret = misc_register(&touchkey_update_device);
	if (ret) {
		printk("%s misc_register fail\n",__FUNCTION__);
	}
	
	init_hw();

#if 0
	if (request_irq(IRQ_TOUCH_INT, touchkey_interrupt, 0, DEVICE_NAME, NULL)) {
                printk(KERN_ERR "%s Can't allocate irq ..\n", __FUNCTION__);
                return -EBUSY;
        }
#endif

	
	while(retry--)
	{
		if(mcsdl_download_binary_data())
		{
			break;
		}
		printk("%s F/W download fail\n",__FUNCTION__);
	}
	ret = i2c_add_driver(&touchkey_i2c_driver);
	
	if(ret)
	{
		printk("melfas touch keypad registration failed, module not inserted.ret= %d\n",ret);
	}
	return ret;
}

static void __exit touchkey_exit(void)
{
	i2c_del_driver(&touchkey_i2c_driver);
	misc_deregister(&touchkey_update_device);
	if (touchkey_wq)
		destroy_workqueue(touchkey_wq);

}

late_initcall(touchkey_init);
module_exit(touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("@@@");
MODULE_DESCRIPTION("melfas touch keypad");
