/* 

 * drivers/i2c/chips/akm8973.c - akm8973 compass driver

 *

 * Copyright (C) 2008-2009 HTC Corporation.

 * Author: viral wang <viralwang@gmail.com>

 *

 * This software is licensed under the terms of the GNU General Public

 * License version 2, as published by the Free Software Foundation, and

 * may be copied, distributed, and modified under those terms.

 *

 * This program is distributed in the hope that it will be useful,

 * but WITHOUT ANY WARRANTY; without even the implied warranty of

 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the

 * GNU General Public License for more details.

 *

 */

 
/*

 * Revised by AKM 2010/03/25

 * 

 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <mach/gpio-p1.h>

#include "ak8973b.h"

#define DEBUG 0

#define E_COMPASS_ADDRESS	0x1c	/* CAD0 : 0, CAD1 : 0 */
#define I2C_DF_NOTIFY       0x01
#define IRQ_COMPASS_INT IRQ_EINT(2) /* EINT(2) */

static struct i2c_client *this_client;

struct ak8973b_data {
	struct i2c_client		*client;
	struct early_suspend	early_suspend;
};

static DECLARE_WAIT_QUEUE_HEAD(open_wq);
static char ak_e2prom_data[3];

// this proc file system's path is "/proc/driver/ak8973"
// usage :	(at the path) type "cat bma020" , it will show short information for current accelation
// 			use it for simple working test only
#define AK8973_PROC_FS

#ifdef AK8973_PROC_FS
#include <linux/proc_fs.h>
#define DRIVER_PROC_ENTRY		"driver/ak8973"

static int AKECS_SetMeasure (void);
static int AKECS_GetData (short *rbuf);

static int ak_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *p = page;
	int len;

	short mag_sensor[4] ={0, 0, 0, 0};
	s3c_gpio_cfgpin(GPIO_MSENSE_IRQ, GPIO_INPUT);
	s3c_gpio_setpull(GPIO_MSENSE_IRQ, S3C_GPIO_PULL_NONE);
	AKECS_SetMeasure();
	while(!gpio_get_value(GPIO_MSENSE_IRQ));
	AKECS_GetData(mag_sensor);

	p += sprintf(p,"[AK8973]\nX: %d, Y: %d, Z: %d\n", mag_sensor[1], mag_sensor[2], mag_sensor[3] );
	len = (p - page) - off;
	if (len < 0) {
		len = 0;
	}
	printk("ak_proc_read: success full\n");
	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}
#endif	//BMA020_PROC_FS

static int AKI2C_RxData(char *rxData, int length)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	if (i2c_transfer(this_client->adapter, msgs, 2) < 0) {
		printk(KERN_ERR "AKI2C_RxData: transfer error \n");
		return -EIO;
	} else
		return 0;
}

static int AKI2C_TxData(char *txData, int length)
{
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	if (i2c_transfer(this_client->adapter, msg, 1) < 0) {
		printk(KERN_ERR "AKI2C_TxData: transfer error addr \n");
		return -EIO;
	} else
		return 0;
}

static int AKECS_Init(void)
{
	return 0;
}

/*----------------------------------------------------------------------------*/
//Description : Resetting AKECS 
//WHO : AKEK GWLEE
//DATE : 2008 12 08
/*----------------------------------------------------------------------------*/
static void AKECS_Reset (void)
{
      
	gpio_set_value(GPIO_MSENSE_nRST, GPIO_LEVEL_LOW);
	udelay(120);
	gpio_set_value(GPIO_MSENSE_nRST, GPIO_LEVEL_HIGH);
	gprintk("AKECS RESET COMPLETE\n");
}

/*----------------------------------------------------------------------------*/
//Description : Setting measurement to AKECS 
//WHO : AKEK GWLEE
//DATE : 2008 12 08
/*----------------------------------------------------------------------------*/
static int AKECS_SetMeasure (void)
{
	char buffer[2];

	gprintk("MEASURE MODE\n");
	/* Set measure mode */
	buffer[0] = AKECS_REG_MS1;
	buffer[1] = AKECS_MODE_MEASURE;

	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

/*----------------------------------------------------------------------------*/
//Description : Setting EEPROM to AKECS 
//WHO : AKEK GWLEE
//DATE : 2008 12 08
/*----------------------------------------------------------------------------*/
static int AKECS_SetE2PRead ( void )
{
	char buffer[2];

	/* Set measure mode */
	buffer[0] = AKECS_REG_MS1;
	buffer[1] = AKECS_MODE_E2P_READ;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}


/*----------------------------------------------------------------------------*/
//Description : Power Down to AKECS 
//WHO : AKEK GWLEE
//DATE : 2008 12 08
/*----------------------------------------------------------------------------*/
static int AKECS_PowerDown (void)
{
	char buffer[2];
	int ret;

	/* Set powerdown mode */
	buffer[0] = AKECS_REG_MS1;
	buffer[1] = AKECS_MODE_POWERDOWN;
	/* Set data */
	ret = AKI2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;

	/* Dummy read for clearing INT pin */
	buffer[0] = AKECS_REG_TMPS;
	/* Read data */
	ret = AKI2C_RxData(buffer, 1);
	if (ret < 0)
		return ret;

 return ret;
}

/*----------------------------------------------------------------------------*/
//Description : Get EEPROM Data to AKECS 
//WHO : AKEK GWLEE
//DATE : 2008 12 08
/*----------------------------------------------------------------------------*/
static int AKECS_GetEEPROMData (void)
{ 
	int ret;
	char buffer[RBUFF_SIZE + 1];

	ret = AKECS_SetE2PRead();
	if (ret < 0) return ret;

	memset(buffer, 0, RBUFF_SIZE + 1);
	buffer[0] = AKECS_EEP_EHXGA;
	ret =  AKI2C_RxData(buffer, 3);

	if (ret < 0) return ret;

	ak_e2prom_data[0]= buffer[0];
	ak_e2prom_data[1]= buffer[1];
	ak_e2prom_data[2]= buffer[2];
	gprintk("AKE2PROM_Data -->%d , %d, %d ----\n", ak_e2prom_data[0],ak_e2prom_data[1],ak_e2prom_data[2]);
	
	return ret;
}

static int AKECS_SetMode(char mode)
{
	int ret = 0;	
	
	switch (mode) {
		case AKECS_MODE_MEASURE:
			ret = AKECS_SetMeasure();
			break;
		case AKECS_MODE_E2P_READ:
			ret = AKECS_SetE2PRead();
			break;
		case AKECS_MODE_POWERDOWN:
			ret = AKECS_PowerDown();
			break;
		default:
			return -EINVAL;
	}

	/* wait at least 300us after changing mode */
	msleep(1);
	return ret;
}

static int AKECS_TransRBuff(char *rbuf, int size)
{
	if(size < RBUFF_SIZE + 1)
		return -EINVAL;

	// read C0 - C4
	rbuf[0] = AKECS_REG_ST;
	return AKI2C_RxData(rbuf, RBUFF_SIZE + 1);

}

/*----------------------------------------------------------------------------*/
//Description : Get EEPROM Data to AKECS 
//WHO : AKEK GWLEE
//DATE : 2008 12 08
/*----------------------------------------------------------------------------*/
static int AKECS_GetData (short *rbuf)
{ 
	int ret;
  	char buffer[RBUFF_SIZE + 1];

	memset(buffer, 0, RBUFF_SIZE + 1);
	buffer[0] = AKECS_REG_TMPS;

	ret =  AKI2C_RxData(buffer, 4);  //temp, x, y, z
	if(ret<0) {
		printk(KERN_ERR "AKECS_GetData  failed--------------\n");
		return ret;
		}
	else	{
			
			rbuf[0] = 35 + (120 -buffer[0])*10/16;
			rbuf[1] = buffer[1];
			rbuf[2] = buffer[2];
			rbuf[3] = buffer[3];
			printk("AKECS_Get_MAG: temp = %d, x = %d, y = %d, z = %d\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
		}
  return ret;
}

static int akmd_open(struct inode *inode, struct file *file)
{
	gprintk(KERN_INFO "[AK8973B] %s\n", __FUNCTION__);
	return nonseekable_open(inode, file);
}

static int akmd_release(struct inode *inode, struct file *file)
{
	gprintk(KERN_INFO "[AK8973B] %s\n", __FUNCTION__);
	return 0;
}

static int akmd_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int err = 0;
	int i;
	void __user *argp = (void __user *)arg;

	char msg[RBUFF_SIZE + 1], rwbuf[16];//, numfrq[2];
	int ret = -1;
	short mode; /* step_count,*/

	/* check cmd */
	if(_IOC_TYPE(cmd) != AKMIO)
	{
#if DEBUG       
		printk("[AK8973] cmd magic type error\n");
#endif
		return -ENOTTY;
	}
	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err)
	{
#if DEBUG
		printk("[AK8973] cmd access_ok error\n");
#endif
		return -EFAULT;
	}	

	switch (cmd) {
		case ECS_IOCTL_READ:
		case ECS_IOCTL_WRITE:
			if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
				return -EFAULT;
			break;
		case ECS_IOCTL_SET_MODE:
			if (copy_from_user(&mode, argp, sizeof(mode)))
				return -EFAULT;
			break;
		default:
			break;
	}
	switch (cmd) {
		case ECS_IOCTL_RESET:
			gprintk("[AK8973B] ECS_IOCTL_RESET %x\n", cmd);
			AKECS_Reset();
			break;
		case ECS_IOCTL_READ:
			gprintk("[AK8973B] ECS_IOCTL_READ %x\n", cmd);
			gprintk(" len %02x:", rwbuf[0]);
			gprintk(" addr %02x:", rwbuf[1]);
			gprintk("\n");
			if (rwbuf[0] < 1)
				return -EINVAL;
			ret = AKI2C_RxData(&rwbuf[1], rwbuf[0]);
			//for(i=0; i<rwbuf[0]; i++){
			//	printk(" %02x", rwbuf[i+1]);
			//}
			gprintk(" ret = %d\n", ret);
			if (ret < 0)
				return ret;
			break;
		case ECS_IOCTL_WRITE:
			gprintk("[AK8973B] ECS_IOCTL_WRITE %x\n", cmd);
			gprintk(" len %02x:", rwbuf[0]);
			for(i=0; i<rwbuf[0]; i++){
				gprintk(" %02x", rwbuf[i+1]);
			}
			gprintk("\n");
			if (rwbuf[0] < 2)
				return -EINVAL;
			ret = AKI2C_TxData(&rwbuf[1], rwbuf[0]);
			gprintk(" ret = %d\n", ret);
			if (ret < 0)
				return ret;
			break;
		case ECS_IOCTL_SET_MODE:
			gprintk("[AK8973B] ECS_IOCTL_SET_MODE %x mode=%x\n", cmd, mode);
			ret = AKECS_SetMode((char)mode);
			gprintk(" ret = %d\n", ret);
			if (ret < 0)
				return ret;
			break;
		case ECS_IOCTL_GETDATA:
			gprintk("[AK8973B] ECS_IOCTL_GETDATA %x\n", cmd);
			ret = AKECS_TransRBuff(msg, RBUFF_SIZE+1);
			gprintk(" ret = %d\n", ret);
			if (ret < 0)
				return ret;
			for(i=0; i<ret; i++){
				gprintk(" %02x", msg[i]);
			}
			gprintk("\n");
			break;
		default:
			gprintk("Unknown cmd %x\n", cmd);
			return -ENOTTY;
	}

	switch (cmd) {
		case ECS_IOCTL_READ:
			if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
				return -EFAULT;
			break;
		case ECS_IOCTL_GETDATA:
			if (copy_to_user(argp, &msg, sizeof(msg)))
				return -EFAULT;
			break;
		default:
			break;
	}

	return 0;
}

static void ak8973b_init_hw(void)
{
#if 0	
	s3c_gpio_cfgpin(GPIO_MSENSE_INT, S3C_GPIO_SFN(GPIO_MSENSE_INT_AF));
	s3c_gpio_setpull(GPIO_MSENSE_INT, S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_COMPASS_INT, IRQ_TYPE_EDGE_RISING);
#endif

	if(gpio_is_valid(GPIO_MSENSE_nRST)){
		if(gpio_request(GPIO_MSENSE_nRST, "GPB"))
		{
			printk(KERN_ERR "Failed to request GPIO_MSENSE_nRST!\n");
		}
		gpio_direction_output(GPIO_MSENSE_nRST, GPIO_LEVEL_LOW);
	}

//	s3c_gpio_setpull(GPIO_MSENSE_nRST, S3C_GPIO_PULL_NONE);
/*VNVS: 14-MAY'10 Pulling the RST Pin HIGH instead of keeping in floating state(Pulling NONE)*/
	s3c_gpio_setpull(GPIO_MSENSE_nRST, S3C_GPIO_PULL_NONE);

	gprintk("gpio setting complete!\n");
}

static struct file_operations akmd_fops = {
	.owner = THIS_MODULE,
	.open = akmd_open,
	.release = akmd_release,
	.ioctl = akmd_ioctl,
};

static struct miscdevice akmd_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm8973",
	.fops = &akmd_fops,
};

static int ak8973_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	struct ak8973b_data *akm;
	struct device *dev = &client->dev;

	gprintk("start\n");
	printk("[%s] ak8973 started...\n",__func__);

	akm = kzalloc(sizeof(struct ak8973b_data), GFP_KERNEL);
	if (!akm) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

    this_client = client;
    i2c_set_clientdata(client, akm);

	if(this_client == NULL)
	{
		gprintk("i2c_client is NULL\n");
		return -ENODEV;
	}

	err = misc_register(&akmd_device);
	if (err) {
		printk(KERN_ERR "akm8973_probe: akmd_device register failed\n");
		goto exit_misc_device_register_failed;
	}

#ifdef AK8973_PROC_FS
	create_proc_read_entry(DRIVER_PROC_ENTRY, 0, 0, ak_proc_read, NULL);
#endif	//BMA020_PROC_FS


	AKECS_GetEEPROMData();

	return 0;

exit_misc_device_register_failed:
exit_input_dev_alloc_failed:
	kfree(akm);
exit_alloc_data_failed:
	return err;
}

static int __exit ak8973_remove(struct i2c_client *client)
{
	// int err;
	struct ak8973b_data *akm = i2c_get_clientdata(client);

	printk("[%s] ak8973 removed...\n",__func__);
	
	misc_deregister(&akmd_device);
	
	kfree(akm);
	kfree(client); 
	akm = NULL;	
	this_client = NULL;
	
	gpio_free(GPIO_MSENSE_nRST);
	
	gprintk("end\n");
	return 0;
}

static int ak8973_suspend( struct platform_device* pdev, pm_message_t state )
{
	printk("############## %s \n",__func__);	
	printk("[%s] Set Mode AKECS_MODE_POWERDOWN\n", __func__);
	AKECS_SetMode(AKECS_MODE_POWERDOWN);			
	return 0;
}


static int ak8973_resume( struct platform_device* pdev )
{
	printk("@@@@ %s \n",__func__); 
	
	printk("[%s] Set Mode AKECS_MODE_MEASURE\n", __func__);
	AKECS_SetMode(AKECS_MODE_MEASURE);	

	wake_up(&open_wq);
	return 0;
}

static const struct i2c_device_id ak8973_id[] = {
	{"ak8973", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ak8973_id);

static struct i2c_driver ak8973b_i2c_driver = {
	.driver = {
		   .name = "ak8973",
		   },
	.probe = ak8973_probe,
	.remove = __exit_p(ak8973_remove),
	.suspend = ak8973_suspend,
	.resume = ak8973_resume,
	.id_table = ak8973_id,
};

static int __init ak8973b_init(void)
{
	gprintk("__start\n");
	ak8973b_init_hw();
	AKECS_Reset(); /*module reset */
	udelay(200);

	return i2c_add_driver(&ak8973b_i2c_driver);
}

static void __exit ak8973b_exit(void)
{
	gprintk("__exit\n");
	i2c_del_driver(&ak8973b_i2c_driver);
}

module_init(ak8973b_init);
module_exit(ak8973b_exit);

MODULE_AUTHOR("Asahi Kasei Microdevices");
MODULE_DESCRIPTION("AK8973B compass driver");
MODULE_LICENSE("GPL");

