/*
 *  mdnie_pwm Backlight Driver based on SWI Driver.
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
#include <linux/earlysuspend.h>

#include <mach/hardware.h>
#include <mach/gpio.h>

#include <plat/gpio-cfg.h>
//#include <plat/regs-gpio.h>


#define MDNIE_PWM_MAX_INTENSITY		255
#define MDNIE_PWM_DEFAULT_INTENSITY	255
#define MAX_LEVEL			100

// brightness tuning
#define MAX_BRIGHTNESS_LEVEL 255
#define LOW_BRIGHTNESS_LEVEL 30
#define DIM_BACKLIGHT_LEVEL 20	
#define MAX_BACKLIGHT_VALUE 100 //88
#define LOW_BACKLIGHT_VALUE 8	//8
#define DIM_BACKLIGHT_VALUE 5	//5

#ifdef CONFIG_FB_S3C_MDNIE
extern void init_mdnie_class(void);
#endif

extern void s5p_mdine_pwm_brightness(int value);
extern void s5p_mdine_pwm_enable(int on);


struct early_suspend	st_early_suspend;
struct platform_device *bl_pdev;

static int current_gamma_level = MAX_LEVEL;

static int mdnie_pwm_suspended;
static int current_intensity = 0;
static DEFINE_SPINLOCK(mdnie_pwm_lock);

static void mdnie_pwm_apply(struct platform_device *pdev, int level)
{

	s5p_mdine_pwm_brightness(level);

	current_gamma_level = level;

	dev_dbg(&pdev->dev, "%s:current_gamma_level=%d\n", __func__, current_gamma_level);
}

static void mdnie_pwm_apply_brightness(struct platform_device *pdev, int level)
{
	mdnie_pwm_apply(pdev, level);

	dev_dbg(&pdev->dev, "%s : apply_level=%d\n", __func__, level);
	printk("%s : apply_level=%d\n", __func__, level);
}


static void mdnie_pwm_backlight_ctl(struct platform_device *pdev, int intensity)
{
	int tune_level;
	
	// brightness tuning
	if(intensity >= LOW_BRIGHTNESS_LEVEL)
		tune_level = (intensity - LOW_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE-LOW_BACKLIGHT_VALUE) / (MAX_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE;
	else if(intensity >= DIM_BACKLIGHT_LEVEL)
		tune_level = (intensity - DIM_BACKLIGHT_LEVEL) * (LOW_BACKLIGHT_VALUE-DIM_BACKLIGHT_VALUE) / (LOW_BRIGHTNESS_LEVEL-DIM_BACKLIGHT_LEVEL) + DIM_BACKLIGHT_VALUE;
	else if(intensity > 0)
		tune_level = (intensity) * (DIM_BACKLIGHT_VALUE) / (DIM_BACKLIGHT_LEVEL);
	else
		tune_level = intensity;

	mdnie_pwm_apply_brightness(pdev, tune_level);
}


static void mdnie_pwm_send_intensity(struct backlight_device *bd)
{
	unsigned long flags;
	int intensity = bd->props.brightness;
	struct platform_device *pdev = NULL;

	pdev = dev_get_drvdata(&bd->dev);
	if (pdev == NULL) 
		{
		printk(KERN_ERR "%s:failed to get platform device.\n", __func__);
		return;
		}

	if (bd->props.power != FB_BLANK_UNBLANK ||
		bd->props.fb_blank != FB_BLANK_UNBLANK ||
		mdnie_pwm_suspended) 
		{
		if(!current_intensity)
			return;
		//udelay(500);
		msleep(1);
		intensity = 0;
		}

	spin_lock_irqsave(&mdnie_pwm_lock, flags);

	mdnie_pwm_backlight_ctl(pdev, intensity);

	spin_unlock_irqrestore(&mdnie_pwm_lock, flags);

	current_intensity = intensity;
}

static void mdnie_pwm_gpio_init()
{
	s3c_gpio_cfgpin(GPIO_LCD_CABC_PWM_R05, S3C_GPIO_SFN(3));	//mdnie pwm
    s3c_gpio_setpull(GPIO_LCD_CABC_PWM_R05, S3C_GPIO_PULL_NONE);
}

#ifdef CONFIG_PM
static int mdnie_pwm_suspend(struct platform_device *swi_dev, pm_message_t state)
{
	struct backlight_device *bd = platform_get_drvdata(swi_dev);

	mdnie_pwm_suspended = 1;
	mdnie_pwm_send_intensity(bd);
	
	return 0;
}

static int mdnie_pwm_resume(struct platform_device *swi_dev)
{	
	struct backlight_device *bd = platform_get_drvdata(swi_dev);

	bd->props.brightness = MDNIE_PWM_DEFAULT_INTENSITY;
	mdnie_pwm_suspended = 0;
	mdnie_pwm_send_intensity(bd);

	return 0;
}
#else
#define mdnie_pwm_suspend		NULL
#define mdnie_pwm_resume		NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mdnie_pwm_early_suspend(struct early_suspend *h)
{	
	struct backlight_device *bd = platform_get_drvdata(bl_pdev);

	mdnie_pwm_suspended = 1;
	mdnie_pwm_send_intensity(bd);

	// set gpio '0' 
	gpio_set_value(GPIO_LCD_CABC_PWM_R05, 0);
	s3c_gpio_cfgpin(GPIO_LCD_CABC_PWM_R05, S3C_GPIO_OUTPUT);

}

static void mdnie_pwm_early_resume(struct early_suspend *h)
{
	struct backlight_device *bd = platform_get_drvdata(bl_pdev);

	//bd->props.brightness = MDNIE_PWM_DEFAULT_INTENSITY;
	mdnie_pwm_suspended = 0;

	mdnie_pwm_gpio_init();
		
	mdnie_pwm_send_intensity(bd);
}
#endif

static int mdnie_pwm_set_intensity(struct backlight_device *bd)
{
	printk("BD->PROPS.BRIGHTNESS = %d\n", bd->props.brightness);

	mdnie_pwm_send_intensity(bd);

	return 0;
}


static int mdnie_pwm_get_intensity(struct backlight_device *bd)
{
	return current_intensity;
}


static struct backlight_ops mdnie_pwm_ops = {
	.get_brightness = mdnie_pwm_get_intensity,
	.update_status  = mdnie_pwm_set_intensity,
};

//for measuring luminance
void mdnie_pwm_set_brightness(int brightness)
{
	unsigned long flags;

	printk("%s: value=%d\n", __func__, brightness);

	spin_lock_irqsave(&mdnie_pwm_lock, flags);

	mdnie_pwm_apply_brightness(bl_pdev, brightness);

	spin_unlock_irqrestore(&mdnie_pwm_lock, flags);
}
EXPORT_SYMBOL(mdnie_pwm_set_brightness);

static int mdnie_pwm_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;

	printk("MDNIE_PWM Probe START!!!\n");

	bd = backlight_device_register("s5p_bl", &pdev->dev, pdev, &mdnie_pwm_ops);

	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);

	bd->props.max_brightness = MDNIE_PWM_MAX_INTENSITY;
	bd->props.brightness = MDNIE_PWM_DEFAULT_INTENSITY;

	mdnie_pwm_gpio_init();

//	s5p_mdine_pwm_enable(1);

//	mdnie_pwm_set_intensity(bd);

	dev_info(&pdev->dev, "mdnie_pwm backlight driver is enabled.\n");

	bl_pdev = pdev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	st_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	st_early_suspend.suspend = mdnie_pwm_early_suspend;
	st_early_suspend.resume = mdnie_pwm_early_resume;
	register_early_suspend(&st_early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#ifdef CONFIG_FB_S3C_MDNIE
//	init_mdnie_class();  //set mDNIe UI mode, Outdoormode
#endif

	printk("MDNIE_PWM Probe END!!!\n");
	return 0;

}

static int mdnie_pwm_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&st_early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	bd->props.brightness = 0;
	bd->props.power = 0;
	mdnie_pwm_send_intensity(bd);

	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver mdnie_pwm_driver = {
	.driver		= {
		.name	= "mdnie_pwm_bl",
		.owner	= THIS_MODULE,
	},
	.probe		= mdnie_pwm_probe,
	.remove		= mdnie_pwm_remove,
#if !(defined CONFIG_HAS_EARLYSUSPEND)
	.suspend	= mdnie_pwm_suspend,
	.resume		= mdnie_pwm_resume,
#endif
};

static int __init mdnie_pwm_init(void)
{	
	return platform_driver_register(&mdnie_pwm_driver);
}

static void __exit mdnie_pwm_exit(void)
{
	platform_driver_unregister(&mdnie_pwm_driver);
}

module_init(mdnie_pwm_init);
module_exit(mdnie_pwm_exit);
