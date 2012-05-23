/*
 *  nt39411b Backlight Driver based on SWI Driver.
 *
 *  Copyright (c) 2009 Samsung Electronics
 *  InKi Dae <inki.dae@samsung.com>
 *
 *  Based on Sharp's Corgi Backlight Driver
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/swi.h>
#include <linux/earlysuspend.h>

#define nt39411b_MAX_INTENSITY		255
#define nt39411b_DEFAULT_INTENSITY	255
#define MAX_LEVEL			16

//brightness tuning
#define MAX_BRIGHTNESS_LEVEL 255
#define LOW_BRIGHTNESS_LEVEL 30
#define MAX_BACKLIGHT_VALUE 15
#define LOW_BACKLIGHT_VALUE 1
#define DIM_BACKLIGHT_VALUE 1

#define TUNE_DIV		8
static unsigned int tuning_table[33] = {0, 1,  1,  1,  2,  2, 
										   3,  3,  4,  5,  6, 
										   7,  8,  9,  9,  9,  
										   9, 10, 10, 10, 11, 
										  11, 11, 12, 12, 12, 
										  13, 13, 13, 14, 14, 
										  14, 15, 15};

#define A1_A3_CURRENT_SELECT		18
#define A1_A3_ONOFF_SELECT		20
#define	A1_A2_ON			7
#define A1_A3_ALL_ON			8
#define B1_B2_CURRENT_SELECT		17
#define B1_B2_ONOFF_SELECT		19
#define B1_B2_ALL_ON			4

#ifdef CONFIG_FB_S3C_MDNIE
extern void init_mdnie_class(void);
#endif

struct early_suspend	bl_early_suspend;
struct swi_device *bl_swi_dev;

/* when nt39411b is turned on, it becomes max(100) soon. */
static int current_gamma_level = MAX_LEVEL;
static unsigned int gamma_table[17] = {0, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};

static int nt39411b_suspended;
static int current_intensity = 0, first = 1;
static DEFINE_SPINLOCK(nt39411bl_lock);

static void nt39411b_apply(struct swi_device *swi_dev, int level)
{
	int i;

	/* A1-A3 current select */
	for (i = 0; i < A1_A3_CURRENT_SELECT; i++)
		swi_dev->transfer(swi_dev, SWI_CHANGE);
	udelay(500);
	for (i = 0; i < gamma_table[level]; i++)
		swi_dev->transfer(swi_dev, SWI_CHANGE);

	udelay(500);

	/* B1-B2 current select */
	for (i = 0; i < B1_B2_CURRENT_SELECT; i++)
		swi_dev->transfer(swi_dev, SWI_CHANGE);
	udelay(500);
	for (i = 0; i < gamma_table[level]; i++)
		swi_dev->transfer(swi_dev, SWI_CHANGE);

	udelay(500);

	if (first) {
		/* A1-A3 on/off select */
		for (i = 0; i < A1_A3_ONOFF_SELECT; i++)
			swi_dev->transfer(swi_dev, SWI_CHANGE);
		udelay(500);
		/* A1-A3 all on */
		for (i = 0; i < A1_A2_ON; i++)
			swi_dev->transfer(swi_dev, SWI_CHANGE);

		udelay(500);
		/* B1-B2 on/off select */
		for (i = 0; i < B1_B2_ONOFF_SELECT; i++)
			swi_dev->transfer(swi_dev, SWI_CHANGE);

		udelay(500);
		/* B1-B2 all on */
		for (i = 0; i < B1_B2_ALL_ON; i++)
			swi_dev->transfer(swi_dev, SWI_CHANGE);

		first = 0;
	}

	current_gamma_level = level;

	dev_dbg(&swi_dev->dev, "%s:current_gamma_level=%d\n", __func__, current_gamma_level);
}

static void nt39411b_apply_brightness(struct swi_device *swi_dev, int level)
{
	nt39411b_apply(swi_dev, level);

	dev_dbg(&swi_dev->dev, "%s : apply_level=%d\n", __func__, level);
	printk("%s : apply_level=%d\n", __func__, level);
}


static void nt39411b_backlight_ctl(struct swi_device *swi_dev, int intensity)
{
	int tune_level;

	if(intensity > MAX_BRIGHTNESS_LEVEL)
		intensity = MAX_BRIGHTNESS_LEVEL;
	
	// brightness tuning
	tune_level = tuning_table[ (intensity+1)/TUNE_DIV ];

	if(intensity > 0 && tune_level == 0)
		tune_level = 1;
	
#if 0
	intensity += 1;
	intensity /= MAX_LEVEL;

	/* gamma range is 1 to 16. */
	if (intensity < 1)
		intensity = 1;
	if (intensity > 16)
		intensity = 16;
#endif

	nt39411b_apply_brightness(swi_dev, tune_level);
}


static void nt39411b_send_intensity(struct backlight_device *bd)
{
	unsigned long flags;
	int intensity = bd->props.brightness;
	struct swi_device *swi_dev = NULL;

	swi_dev = dev_get_drvdata(&bd->dev);
	if (swi_dev == NULL) {
		printk(KERN_ERR "failed to get swi device.\n");
		return;
	}

	if (bd->props.power != FB_BLANK_UNBLANK ||
		bd->props.fb_blank != FB_BLANK_UNBLANK ||
		nt39411b_suspended) {
		/* enter into shutdown mode. */
		swi_dev->transfer(swi_dev, SWI_LOW);
		udelay(500);
		intensity = 0;
	}

	spin_lock_irqsave(&nt39411bl_lock, flags);

	nt39411b_backlight_ctl(swi_dev, intensity);

	spin_unlock_irqrestore(&nt39411bl_lock, flags);

	current_intensity = intensity;
}

#ifdef CONFIG_PM
static int nt39411b_suspend(struct swi_device *swi_dev, pm_message_t state)
{
	struct backlight_device *bd = swi_get_drvdata(swi_dev);

	nt39411b_suspended = 1;
	nt39411b_send_intensity(bd);
	return 0;
}

static int nt39411b_resume(struct swi_device *swi_dev)
{
	struct backlight_device *bd = swi_get_drvdata(swi_dev);

	/* A1-A3 and B1-B2 on */
	first = 1;

	bd->props.brightness = nt39411b_DEFAULT_INTENSITY;
	nt39411b_suspended = 0;

	nt39411b_send_intensity(bd);
	return 0;
}
#else
#define nt39411b_suspend	NULL
#define nt39411b_resume		NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static int nt39411b_early_suspend(struct early_suspend *h)
{
	struct backlight_device *bd = swi_get_drvdata(bl_swi_dev);

	nt39411b_suspended = 1;
	nt39411b_send_intensity(bd);
	return 0;
}

static int nt39411b_early_resume(struct early_suspend *h)
{
	struct backlight_device *bd = swi_get_drvdata(bl_swi_dev);

	/* A1-A3 and B1-B2 on */
	first = 1;

	//bd->props.brightness = nt39411b_DEFAULT_INTENSITY;
	nt39411b_suspended = 0;

	nt39411b_send_intensity(bd);
	return 0;
}
#endif

static int nt39411b_set_intensity(struct backlight_device *bd)
{
	printk("BD->PROPS.BRIGHTNESS = %d\n", bd->props.brightness);

	nt39411b_send_intensity(bd);
	return 0;
}

static int nt39411b_get_intensity(struct backlight_device *bd)
{
	return current_intensity;
}

static struct backlight_ops nt39411b_ops = {
	.get_brightness = nt39411b_get_intensity,
	.update_status  = nt39411b_set_intensity,
};

//for measuring luminance
void nt39411b_set_brightness(int brightness)
{
	unsigned long flags;

	printk("%s: value=%d\n", __func__, brightness);
	
	spin_lock_irqsave(&nt39411bl_lock, flags);

	nt39411b_apply_brightness(bl_swi_dev, brightness);

	spin_unlock_irqrestore(&nt39411bl_lock, flags);
}
EXPORT_SYMBOL(nt39411b_set_brightness);

static int nt39411b_probe(struct swi_device *swi_dev)
{
	struct backlight_device *bd;

	printk("NT39411B Probe START!!!\n");

	bd = backlight_device_register("s5p_bl", &swi_dev->dev, swi_dev, &nt39411b_ops);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	swi_set_drvdata(swi_dev, bd);

	bd->props.max_brightness = nt39411b_MAX_INTENSITY;
	bd->props.brightness = nt39411b_DEFAULT_INTENSITY;

	if (swi_dev->bi == NULL) {
		dev_err(&swi_dev->dev, "failed to get init function pointer.\n");
		return -1;
	}

	swi_dev->bi->init();

	nt39411b_set_intensity(bd);

	dev_info(&swi_dev->dev, "nt39411b backlight driver is enabled.\n");

	bl_swi_dev = swi_dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	bl_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	bl_early_suspend.suspend = nt39411b_early_suspend;
	bl_early_suspend.resume = nt39411b_early_resume;
	register_early_suspend(&bl_early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#ifdef CONFIG_FB_S3C_MDNIE
//	init_mdnie_class();  //set mDNIe UI mode, Outdoormode
#endif

	printk("NT39411B Probe END!!!\n");

	return 0;
}

static int nt39411b_remove(struct swi_device *swi_dev)
{
	struct backlight_device *bd = swi_get_drvdata(swi_dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&bl_early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	bd->props.brightness = 0;
	bd->props.power = 0;
	nt39411b_send_intensity(bd);

	backlight_device_unregister(bd);

	return 0;
}

static struct swi_driver nt39411b_driver = {
	.driver		= {
		.name	= "nt39411b_bl",
		.owner	= THIS_MODULE,
	},
	.probe		= nt39411b_probe,
	.remove		= nt39411b_remove,
#if !(defined CONFIG_HAS_EARLYSUSPEND)
	.suspend	= nt39411b_suspend,
	.resume		= nt39411b_resume,
#endif
};

static int __init nt39411b_init(void)
{
	return swi_register_driver(&nt39411b_driver);
}

static void __exit nt39411b_exit(void)
{
	swi_unregister_driver(&nt39411b_driver);
}

module_init(nt39411b_init);
module_exit(nt39411b_exit);
