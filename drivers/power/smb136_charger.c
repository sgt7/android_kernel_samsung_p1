/*
 *  smb136_charger.c
 * 
 *  Copyright (C) 2011 Samsung Electronics
 *  Ikkeun Kim <iks.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/sec_battery.h>
#include <linux/smb136_charger.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-p1.h>

 
/* Slave address */
#define SMB136_SLAVE_ADDR		0x9A

/* SMB136 Registers. */
#define SMB_ChargeCurrent		0x00
#define SMB_InputCurrentLimit	0x01
#define SMB_FloatVoltage		0x02
#define SMB_ControlA			0x03
#define SMB_ControlB			0x04
#define SMB_PinControl			0x05
#define SMB_OTGControl			0x06
#define SMB_Fault				0x07
#define SMB_Temperature		0x08
#define SMB_SafetyTimer			0x09
#define SMB_VSYS				0x0A
#define SMB_I2CAddr			0x0B

#define SMB_IRQreset			0x30
#define SMB_CommandA			0x31
#define SMB_StatusA			0x32
#define SMB_StatusB			0x33
#define SMB_StatusC			0x34
#define SMB_StatusD			0x35
#define SMB_StatusE			0x36
#define SMB_StatusF			0x37
#define SMB_StatusG			0x38
#define SMB_StatusH			0x39
#define SMB_DeviceID			0x3B
#define SMB_CommandB			0x3C

/* SMB_StatusC register bit. */
#define SMB_USB				1
#define SMB_CHARGER			0
#define Compelete				1
#define Busy					0
#define InputCurrent275			0xE
#define InputCurrent500			0xF
#define InputCurrent700			0x0
#define InputCurrent800			0x1
#define InputCurrent900			0x2
#define InputCurrent1000		0x3
#define InputCurrent1100		0x4
#define InputCurrent1200		0x5
#define InputCurrent1300		0x6
#define InputCurrent1400		0x7


struct smb136_chg_data {
	struct i2c_client *client;
	struct smb136_charger_data *pdata;
	struct charger_device *chgdev;
};

static struct smb136_chg_data *smb136_chg;  // for local use
static int charger_i2c_init = 0;


static int smb136_i2c_read(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret = 0;

	if(!client)
		return -ENODEV;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		return -EIO;

	*data = ret & 0xff;
	return 0;
}

static int smb136_i2c_write(struct i2c_client *client, u8 reg, u8 data)
{
	if(!client)
		return -ENODEV;

	return i2c_smbus_write_byte_data(client, reg, data);
}

int smb136_get_charging_status(void)
{
	struct smb136_chg_data *chg = smb136_chg;
	enum charging_status_type_t status = CHARGING_STATUS_NONE;
	u8 data1 = 0;
	u8 data2 = 0;

	smb136_i2c_read(chg->client, 0x36, &data1);
	smb136_i2c_read(chg->client, 0x39, &data2);
	printk("%s : 0x36h(0x%02x), 0x39h(0x%02x)\n", __func__, data1, data2);

	if(data2 & 0x01)
		status = CHARGING_STATUS_ACTIVE;
	else {
		if ((data1 & 0x08) == 0x08)  // if error bit check, ignore the status of charger-ic
			status = CHARGING_STATUS_ERROR;
		else if((data1 & 0xc0) == 0xc0)  // At least one charge cycle terminated, Charge current < Termination Current
			status = CHARGING_STATUS_FULL;
	}

	return (int)status;
}

void smb136_test_read(void)
{
	struct smb136_chg_data * chg = smb136_chg;
	u8 data = 0;
	u32 addr = 0;

	if(!charger_i2c_init) {
		printk("%s : smb136 charger IC i2c is not initialized!!\n", __func__);
		return ;
	}

	for(addr=0;addr<0x0c;addr++)
	{
		smb136_i2c_read(chg->client, addr, &data);		
		printk("SMB136 addr : 0x%02x data : 0x%02x\n", addr,data);
	}

	for(addr=0x31;addr<0x3D;addr++)
	{
		smb136_i2c_read(chg->client, addr, &data);		
		printk("SMB136 addr : 0x%02x data : 0x%02x\n", addr,data);
	}
}

int smb136_charging(int en, int cable_status)
{
	struct smb136_chg_data *chg = smb136_chg;
	u8 data = 0;

	if(!charger_i2c_init) {
		printk("%s : smb136 charger IC i2c is not initialized!!\n", __func__);
		return -1;
	}

	printk("%s : enable(%d), cable_status(%d)\n",__func__,en, cable_status);

	if(en) {  // enable
		if(cable_status==CABLE_TYPE_AC)
		{
			//1. HC mode
			data = 0x8c;	

			smb136_i2c_write(chg->client, SMB_CommandA, data);
			udelay(10);

			// 2. Change USB5/1/HC Control from Pin to I2C
			smb136_i2c_write(chg->client, SMB_PinControl, 0x8);
			udelay(10);

			smb136_i2c_write(chg->client, SMB_CommandA, 0x8c);
			udelay(10);

			//3. Set charge current to 1500mA
			data = 0xf4;
			
			smb136_i2c_write(chg->client, SMB_ChargeCurrent, data);
			udelay(10);
		}
		else if(cable_status==CABLE_TYPE_USB || cable_status==CABLE_TYPE_IMPROPER_AC)
		{
			// 1. USBIN 500mA mode 
			data = 0x88;	

			smb136_i2c_write(chg->client, SMB_CommandA, data);
			udelay(10);

			// 2. Change USB5/1/HC Control from Pin to I2C
			smb136_i2c_write(chg->client, SMB_PinControl, 0x8);
			udelay(10);

			smb136_i2c_write(chg->client, SMB_CommandA, 0x88);
			udelay(10);

			// 3. Set charge current to 500mA
			data = 0x14;
			
			smb136_i2c_write(chg->client, SMB_ChargeCurrent, data);
			udelay(10);
		}

		// 3. Disable Automatic Input Current Limit
		data = 0xe6;
		smb136_i2c_write(chg->client, SMB_InputCurrentLimit, data);
		udelay(10);

		//4. Automatic Recharge Disabed 
		data = 0x8c;
		smb136_i2c_write(chg->client, SMB_ControlA, data);
		udelay(10);

		//5. Safty timer Disabled
		data = 0x28;
		smb136_i2c_write(chg->client, SMB_ControlB, data);
		udelay(10);

		//6. Disable USB D+/D- Detection
		data = 0x28;
		smb136_i2c_write(chg->client, SMB_OTGControl, data);
		udelay(10);

		//7. Set Output Polarity for STAT
		data = 0xCA;
		smb136_i2c_write(chg->client, SMB_FloatVoltage, data);
		udelay(10);

		//8. Re-load Enable
		data = 0x4b;
		smb136_i2c_write(chg->client, SMB_SafetyTimer, data);
		udelay(10);
	}
	else {
		// do nothing...
	}

	return 0;
}

static irqreturn_t smb136_irq_thread(int irq, void *data)
{
	struct smb136_chg_data *chg = data;

	printk("%s\n", __func__);

	smb136_test_read();
	
	if(gpio_get_value(GPIO_TA_nCHG)==1 && smb136_get_charging_status()==CHARGING_STATUS_FULL)
	{
		chg->chgdev->set_charging_status(CHARGING_STATUS_FULL);
	}

	return IRQ_HANDLED;
}

static int smb136_irq_init(struct smb136_chg_data *chg)
{
	struct i2c_client *client = chg->client;
	int ret;

	if (client->irq) {
		ret = request_threaded_irq(client->irq, NULL,
			smb136_irq_thread, IRQ_TYPE_EDGE_RISING,
			"SMB136 charger", chg);
		if (ret) {
			dev_err(&client->dev, "failed to reqeust IRQ\n");
			return ret;
		}

		ret = enable_irq_wake(client->irq);
		if (ret < 0)
			dev_err(&client->dev,
				"failed to enable wakeup src %d\n", ret);
	}

	return 0;
}

static int smb136_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct smb136_chg_data *chg;
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	pr_info("%s : SMB136 Charger Driver Loading\n", __func__);

	chg = kzalloc(sizeof(struct smb136_chg_data), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->client = client;
	chg->pdata = client->dev.platform_data;
	chg->chgdev = chg->pdata->chgdev;

	i2c_set_clientdata(client, chg);
	smb136_chg = chg;  // set local

	printk("Smb136 charger attach success!!!\n");

	// Check whether battery already full charged or not.
	if(smb136_get_charging_status()==CHARGING_STATUS_FULL)
		chg->chgdev->set_charging_status(CHARGING_STATUS_FULL);


	if (!chg->pdata) {
		pr_err("%s : No platform data supplied\n", __func__);
		ret = -EINVAL;
		goto err_pdata;
	}

	chg->chgdev->charging_control = smb136_charging;
	chg->chgdev->get_connection_status = NULL;
	chg->chgdev->get_charging_status = smb136_get_charging_status;
	if(chg->pdata && chg->pdata->charger_dev_register)
		chg->pdata->charger_dev_register(chg->chgdev);

	charger_i2c_init = 1;

	ret = smb136_irq_init(chg);
	if (ret)
		goto err_pdata;

	smb136_test_read();
	
	return 0;

err_pdata:
	kfree(chg);
	return ret;
}

static int __devexit smb136_remove(struct i2c_client *client)
{
	struct smb136_chg_data *chg = i2c_get_clientdata(client);

	if(chg->pdata && chg->pdata->charger_dev_unregister)
		chg->pdata->charger_dev_unregister(chg->chgdev);

	kfree(chg);
	return 0;
}

static const struct i2c_device_id smb136_id[] = {
	{ "smb136-charger", 0 },
	{ }
};


static struct i2c_driver smb136_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "smb136-charger",
	},
	.id_table	= smb136_id,
	.probe	= smb136_i2c_probe,
	.remove	= __devexit_p(smb136_remove),
	.command = NULL,
};


MODULE_DEVICE_TABLE(i2c, smb136_id);

static int __init smb136_init(void)
{
	return i2c_add_driver(&smb136_i2c_driver);
}

static void __exit smb136_exit(void)
{
	i2c_del_driver(&smb136_i2c_driver);
}

module_init(smb136_init);
module_exit(smb136_exit);

MODULE_AUTHOR("Ikkeun Kim <iks.kim@samsung.com>");
MODULE_DESCRIPTION("smb136 charger driver");
MODULE_LICENSE("GPL");
