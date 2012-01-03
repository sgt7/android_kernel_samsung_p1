
/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/workqueue.h>

//#include <asm/hardware.h>
//#include <asm/arch/gpio.h>
//add by inter.park
#include <mach/hardware.h>
#include <linux/gpio.h>

#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include "bma020_i2c.h"


#define I2C_M_WR				0x00
#define I2C_DF_NOTIFY			0x01


static struct i2c_client *g_client;	

void i2c_acc_bma020_delay(unsigned int msec)
{
	mdelay(msec);
}

char i2c_acc_bma020_read(u8 reg, u8 *val, unsigned int len )
{
	int 	 err;
	struct 	 i2c_msg msg[1];
		
	unsigned char data[1];
	if( (g_client == NULL) || (!g_client->adapter) )
	{
		return -ENODEV;
	}
	
	msg->addr 	= g_client->addr;
	msg->flags 	= I2C_M_WR;
	msg->len 	= 1;
	msg->buf 	= data;
	*data       = reg;

	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0) 
	{
		msg->flags = I2C_M_RD;
		msg->len   = len;
		msg->buf   = val;
		err = i2c_transfer(g_client->adapter, msg, 1);
	}

	if (err >= 0) 
	{
		return 0;
	}
	printk("%s %d i2c transfer error\n", __func__, __LINE__);/* add by inter.park */

	return err;

}
char i2c_acc_bma020_write( u8 reg, u8 *val )
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	if( (g_client == NULL) || (!g_client->adapter) ){
		return -ENODEV;
	}
	
	data[0] = reg;
	data[1] = *val;

	msg->addr = g_client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;
	
	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0) return 0;

	printk("%s %d i2c transfer error\n", __func__, __LINE__);/* add by inter.park */
	return err;
}

static int __devinit bma020_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
        printk("%s called \n",__func__);
		g_client = client;
        return 0;
}

static int __devexit bma020_remove(struct i2c_client *client)
{	
	g_client = NULL;
	return 0;
}

static const struct i2c_device_id bma020_ids[] = {	
	{ "bma020", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma020_ids);

struct i2c_driver acc_bma020_i2c_driver =
{
	.driver	= {
		.name	= "bma020",
	},
	.probe		= bma020_probe,
	.remove		= __devexit_p(bma020_remove),
	.id_table	= bma020_ids,

};

int i2c_acc_bma020_init(void)
{
	int ret;
        printk("%s called \n",__func__);
	if ( (ret = i2c_add_driver(&acc_bma020_i2c_driver)) ) 
	{
		printk("Driver registration failed, module not inserted.\n");
		return ret;
	}

	return 0;
}

void i2c_acc_bma020_exit(void)
{
	printk("[BMA020] i2c_exit\n");
	i2c_del_driver(&acc_bma020_i2c_driver); 
}

