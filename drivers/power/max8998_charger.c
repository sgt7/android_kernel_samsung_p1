/*
 * linux/drivers/power/max8998_charger.c
 *
 * Charger driver of MAX8998 PMIC
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/sec_battery.h>
#include <linux/mfd/max8998.h>
#include <linux/mfd/max8998-private.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>


struct max8998_chg_data {
	struct device		*dev;
	struct max8998_dev	*iodev;
	struct max8998_charger_data *pdata;
	struct charger_device *chgdev;
};

static struct max8998_chg_data *max8998_chg;  // for local use


static int max8998_check_vdcin(void)
{
	struct max8998_chg_data *chg = max8998_chg;
	u8 data = 0;
	int ret;

	ret = max8998_read_reg(chg->iodev, MAX8998_REG_STATUS2, &data);

	if (ret < 0) {
		pr_err("max8998_read_reg error\n");
		return ret;
	}

	return data & MAX8998_MASK_VDCIN;
}

static int max8998_charging_control(int en, int cable_status)
{
	struct max8998_chg_data *chg = max8998_chg;
	int ret = 0;

	if (!en) {
		/* disable charging */
		ret = max8998_update_reg(chg->iodev, MAX8998_REG_CHGR2,
			(1 << MAX8998_SHIFT_CHGEN), MAX8998_MASK_CHGEN);
		if (ret < 0)
			goto err;

		pr_debug("%s : charging disabled", __func__);
	} else {
		/* enable charging */
		if (cable_status == CABLE_TYPE_AC) {
			/* ac */
			ret = max8998_update_reg(chg->iodev, MAX8998_REG_CHGR1,
				(2 << MAX8998_SHIFT_TOPOFF), MAX8998_MASK_TOPOFF);
			if (ret < 0)
				goto err;

			ret = max8998_update_reg(chg->iodev, MAX8998_REG_CHGR1,
				(5 << MAX8998_SHIFT_ICHG), MAX8998_MASK_ICHG);
			if (ret < 0)
				goto err;

			ret = max8998_update_reg(chg->iodev, MAX8998_REG_CHGR2,
				(2 << MAX8998_SHIFT_ESAFEOUT), MAX8998_MASK_ESAFEOUT);
			if (ret < 0)
				goto err;

			pr_debug("%s : TA charging enabled", __func__);

		} else {
			/* usb */
			ret = max8998_update_reg(chg->iodev, MAX8998_REG_CHGR1,
				(6 << MAX8998_SHIFT_TOPOFF), MAX8998_MASK_TOPOFF);
			if (ret < 0)
				goto err;

			ret = max8998_update_reg(chg->iodev, MAX8998_REG_CHGR1,
				(2 << MAX8998_SHIFT_ICHG), MAX8998_MASK_ICHG);
			if (ret < 0)
				goto err;

			ret = max8998_update_reg(chg->iodev, MAX8998_REG_CHGR2,
				(3 << MAX8998_SHIFT_ESAFEOUT), MAX8998_MASK_ESAFEOUT);
			if (ret < 0)
				goto err;

			pr_debug("%s : USB charging enabled", __func__);
		}

		ret = max8998_update_reg(chg->iodev, MAX8998_REG_CHGR2,
			(0 << MAX8998_SHIFT_CHGEN), MAX8998_MASK_CHGEN);
		if (ret < 0)
			goto err;
	}

	return 0;

err:
	pr_err("max8998_read_reg error\n");
	return ret;
}

static __devinit int max8998_charger_probe(struct platform_device *pdev)
{
	struct max8998_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct max8998_platform_data *pdata = dev_get_platdata(iodev->dev);
	struct max8998_chg_data *chg;
	int ret = 0;

	pr_info("%s : MAX8998 Charger Driver Loading\n", __func__);

	chg = kzalloc(sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->iodev = iodev;
	chg->pdata = pdata->charger;
	chg->chgdev = chg->pdata->chgdev;

	max8998_chg = chg;  // set local

	if (!chg->pdata) {
		pr_err("%s : No platform data supplied\n", __func__);
		ret = -EINVAL;
		goto err_pdata;
	}

	ret = max8998_update_reg(iodev, MAX8998_REG_CHGR1, /* disable */
		(0x3 << MAX8998_SHIFT_RSTR), MAX8998_MASK_RSTR);
	if (ret < 0)
		goto err_kfree;

	ret = max8998_update_reg(iodev, MAX8998_REG_CHGR2, /* 6 Hr */
		(0x2 << MAX8998_SHIFT_FT), MAX8998_MASK_FT);
	if (ret < 0)
		goto err_kfree;

	ret = max8998_update_reg(iodev, MAX8998_REG_CHGR2, /* 4.2V */
		(0x0 << MAX8998_SHIFT_BATTSL), MAX8998_MASK_BATTSL);
	if (ret < 0)
		goto err_kfree;

	ret = max8998_update_reg(iodev, MAX8998_REG_CHGR2, /* 105c */
		(0x0 << MAX8998_SHIFT_TMP), MAX8998_MASK_TMP);
	if (ret < 0)
		goto err_kfree;

#if 0  // interrupt setting
	pr_info("%s : pmic interrupt registered\n", __func__);
	ret = max8998_write_reg(iodev, MAX8998_REG_IRQM1,
		~(MAX8998_MASK_DCINR | MAX8998_MASK_DCINF));
	if (ret < 0)
		goto err_kfree;

	ret = max8998_write_reg(iodev, MAX8998_REG_IRQM2, 0xFF);
	if (ret < 0)
		goto err_kfree;

	ret = max8998_write_reg(iodev, MAX8998_REG_IRQM3, ~0x4);
	if (ret < 0)
		goto err_kfree;

	ret = max8998_write_reg(iodev, MAX8998_REG_IRQM4, 0xFF);
	if (ret < 0)
		goto err_kfree;
#endif

	chg->chgdev->charging_control = max8998_charging_control;
	chg->chgdev->get_connection_status = max8998_check_vdcin;
	chg->chgdev->get_charging_status = NULL;

	if(chg->pdata && chg->pdata->charger_dev_register)
		chg->pdata->charger_dev_register(chg->chgdev);

	return 0;

err_kfree:
err_pdata:
	kfree(chg);
	return ret;
}

static int __devexit max8998_charger_remove(struct platform_device *pdev)
{
	struct max8998_chg_data *chg = platform_get_drvdata(pdev);

	if(chg->pdata && chg->pdata->charger_dev_unregister)
		chg->pdata->charger_dev_unregister(chg->chgdev);

	kfree(chg);

	return 0;
}

static struct platform_driver max8998_charger_driver = {
	.driver = {
		.name = "max8998-charger",
		.owner = THIS_MODULE,
	},
	.probe = max8998_charger_probe,
	.remove = __devexit_p(max8998_charger_remove),
};

static int __init max8998_charger_init(void)
{
	return platform_driver_register(&max8998_charger_driver);
}

static void __exit max8998_charger_exit(void)
{
	platform_driver_register(&max8998_charger_driver);
}

module_init(max8998_charger_init);
module_exit(max8998_charger_exit);

MODULE_AUTHOR("Ikkeun Kim <iks.kim@samsung.com>");
MODULE_DESCRIPTION("max8998 charger driver");
MODULE_LICENSE("GPL");
