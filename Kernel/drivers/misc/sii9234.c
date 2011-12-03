#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <plat/pm.h>
#include <asm/irq.h>
#include <linux/delay.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/gpio-p1.h>


//#include "sii9234.h"
#include "MHD_SiI9234.h"
#include "sii9234_tpi_regs.h"


#include <linux/syscalls.h> //denis
#include <linux/fcntl.h> //denis
#include <asm/uaccess.h> //denis



/* Slave address */

#define SII9234_SLAVE_ADDR	0x72
#define SUBJECT "MHL_DRIVER"

#define SII_DEV_DBG(format,...)\
	printk ("[ "SUBJECT " (%s,%d) ] " format "\n", __func__, __LINE__, ## __VA_ARGS__);


struct i2c_driver SII9234_i2c_driver;
struct i2c_client *SII9234_i2c_client = NULL;

struct i2c_driver SII9234A_i2c_driver;
struct i2c_client *SII9234A_i2c_client = NULL;

struct i2c_driver SII9234B_i2c_driver;
struct i2c_client *SII9234B_i2c_client = NULL;

struct i2c_driver SII9234C_i2c_driver;
struct i2c_client *SII9234C_i2c_client = NULL;

static struct i2c_device_id SII9234_id[] = {
	{"SII9234", 0},
	{}
};

static struct i2c_device_id SII9234A_id[] = {
	{"SII9234A", 0},
	{}
};

static struct i2c_device_id SII9234B_id[] = {
	{"SII9234B", 0},
	{}
};

static struct i2c_device_id SII9234C_id[] = {
	{"SII9234C", 0},
	{}
};

int MHL_i2c_init = 0;


struct SII9234_state {
	struct i2c_client *client;
};

static struct timer_list MHL_reg_check;

static u8 SII9234_i2c_read(struct i2c_client *client, u8 reg)
{
	u8 ret;

	if(!MHL_i2c_init)
	{
		SII_DEV_DBG("I2C not ready");
		return 0;
	}
	
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
	{
		SII_DEV_DBG("i2c read fail");
		return -EIO;
	}
	return ret;

}


static int SII9234_i2c_write(struct i2c_client *client, u8 reg, u8 data)
{
	if(!MHL_i2c_init)
	{
		SII_DEV_DBG("I2C not ready");
		return 0;
	}
	return i2c_smbus_write_byte_data(client, reg, data);
}

void SII9234_HW_Reset(void)
{
	SII_DEV_DBG("");
	gpio_direction_output(GPIO_HDMI_EN1, GPIO_LEVEL_HIGH);
	msleep(10);
	gpio_direction_output(GPIO_MHL_RST, GPIO_LEVEL_HIGH);
	msleep(5);
	gpio_direction_output(GPIO_MHL_RST, GPIO_LEVEL_LOW);
	msleep(10);
	gpio_direction_output(GPIO_MHL_RST, GPIO_LEVEL_HIGH);  
	msleep(30);
	
}

void SII9234_HW_Off(void)
{
	SII_DEV_DBG("");
	//s3c_gpio_cfgpin(GPIO_HDMI_EN1, S3C_GPIO_OUTPUT);
	//s3c_gpio_setpin(GPIO_HDMI_EN1, 0);
	
	gpio_direction_output(GPIO_HDMI_EN1, GPIO_LEVEL_LOW);
	msleep(10);
}

void SII9234_GPIO_INIT(void)
{
	SII_DEV_DBG("");
	if (gpio_request(GPIO_HDMI_EN1, "HDMI_EN1"))
			printk(KERN_ERR "Filed to request GPIO_HDMI_EN1!\n");
	if (gpio_request(GPIO_MHL_RST, "MHL_RST"))
			printk(KERN_ERR "Failed to request GPIO_MHL_RST!\n");
}
int SII9234_HW_IsOn(void)
{
	int IsOn = gpio_get_value(GPIO_HDMI_EN1);
	if(IsOn)
		return true;
	else
		return false;	
}

#include "MHD_SiI9234.c"

void sii_9234_monitor(unsigned long arg)
{
	SII_DEV_DBG("");
	//sii9234_polling();
	ReadIndexedRegister(INDEXED_PAGE_0, 0x81);
	//printk("SII9234_i2c_read  INDEXED_PAGE_0: 0x%02x\n", data);

	//MHL_reg_check.expires = get_jiffies_64() + (HZ*3);
	//add_timer(&MHL_reg_check);
}

static void check_HDMI_signal(unsigned long arg)
{
	SII_DEV_DBG("");
	//u8 data;

	//MHL_HW_Reset();
  	//sii9234_initial_registers_set();
	//startTPI();
  	//mhl_output_enable();
	sii9234_tpi_init();
	
	MHL_reg_check.function = sii_9234_monitor;
	MHL_reg_check.expires = get_jiffies_64() + (HZ*3);
	add_timer(&MHL_reg_check);
	
	//data=ReadIndexedRegister(INDEXED_PAGE_0, 0x81);
	//printk("SII9234_i2c_read  INDEXED_PAGE_0: 0x%02x\n", data);
}




//static DECLARE_DELAYED_WORK(init_sii9234, sii92324_init_sequance);

static int SII9234_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	SII_DEV_DBG("");
	//int retval;

	struct SII9234_state *state;

	state = kzalloc(sizeof(struct SII9234_state), GFP_KERNEL);
	if (state == NULL) {		
		printk("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	printk("SII9234 attach success!!!\n");

	SII9234_i2c_client = client;

	MHL_i2c_init = 1;
	//schedule_delayed_work(&init_sii9234,5000);
	
	//init_timer(&MHL_reg_check);
	//MHL_reg_check.function = check_HDMI_signal;
	//MHL_reg_check.expires = get_jiffies_64() + (HZ*10);
	//add_timer(&MHL_reg_check);
	
	//MHL_HW_Reset();
  	//sii9234_initial_registers_set();
  	//startTPI();
  	//mhl_output_enable();
	
	return 0;

}



static int __devexit SII9234_remove(struct i2c_client *client)
{
	struct SII9234_state *state = i2c_get_clientdata(client);
	kfree(state);

	return 0;
}

static int SII9234A_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	SII_DEV_DBG("");

	struct SII9234_state *state;

	state = kzalloc(sizeof(struct SII9234_state), GFP_KERNEL);
	if (state == NULL) {		
		printk("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	printk("SII9234A attach success!!!\n");

	SII9234A_i2c_client = client;

	return 0;

}



static int __devexit SII9234A_remove(struct i2c_client *client)
{
	struct SII9234_state *state = i2c_get_clientdata(client);
	kfree(state);
	return 0;
}

static int SII9234B_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	SII_DEV_DBG("");

	struct SII9234_state *state;

	state = kzalloc(sizeof(struct SII9234_state), GFP_KERNEL);
	if (state == NULL) {		
		printk("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	printk("SII9234B attach success!!!\n");

	SII9234B_i2c_client = client;

	
	return 0;

}



static int __devexit SII9234B_remove(struct i2c_client *client)
{
	struct SII9234_state *state = i2c_get_clientdata(client);
	kfree(state);
	return 0;
}

static int SII9234C_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	SII_DEV_DBG("");

	struct SII9234_state *state;

	state = kzalloc(sizeof(struct SII9234_state), GFP_KERNEL);
	if (state == NULL) {		
		printk("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	printk("SII9234C attach success!!!\n");

	SII9234C_i2c_client = client;

	
	return 0;

}



static int __devexit SII9234C_remove(struct i2c_client *client)
{
	struct SII9234_state *state = i2c_get_clientdata(client);
	kfree(state);
	return 0;
}


struct i2c_driver SII9234_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SII9234",
	},
	.id_table	= SII9234_id,
	.probe	= SII9234_i2c_probe,
	.remove	= __devexit_p(SII9234_remove),
	.command = NULL,
};

struct i2c_driver SII9234A_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SII9234A",
	},
	.id_table	= SII9234A_id,
	.probe	= SII9234A_i2c_probe,
	.remove	= __devexit_p(SII9234A_remove),
	.command = NULL,
};

struct i2c_driver SII9234B_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SII9234B",
	},
	.id_table	= SII9234B_id,
	.probe	= SII9234B_i2c_probe,
	.remove	= __devexit_p(SII9234B_remove),
	.command = NULL,
};

struct i2c_driver SII9234C_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SII9234C",
	},
	.id_table	= SII9234C_id,
	.probe	= SII9234C_i2c_probe,
	.remove	= __devexit_p(SII9234C_remove),
	.command = NULL,
};


