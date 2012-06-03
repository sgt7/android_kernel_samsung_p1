/*
 *  cmc623_pwm Backlight Driver based on SWI Driver.
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
#include <plat/fb.h>
//#include <plat/regs-gpio.h>
#include "../samsung/s3cfb.h"


#define CMC623_PWM_MAX_INTENSITY		255
#define CMC623_PWM_DEFAULT_INTENSITY	255
#define MAX_LEVEL			1600

// brightness tuning
#define MAX_BRIGHTNESS_LEVEL 255
#define MID_BRIGHTNESS_LEVEL 140
#define LOW_BRIGHTNESS_LEVEL 30
#define DIM_BACKLIGHT_LEVEL 20	

#define MAX_BACKLIGHT_VALUE_VA 1280 //80
#define MID_BACKLIGHT_VALUE_VA 512 //32
#define LOW_BACKLIGHT_VALUE_VA 16	//1
#define DIM_BACKLIGHT_VALUE_VA 16	//1
#define MAX_BACKLIGHT_VALUE_VA50 1168 //73
#define MID_BACKLIGHT_VALUE_VA50 464 //29
#define LOW_BACKLIGHT_VALUE_VA50 16	//1
#define DIM_BACKLIGHT_VALUE_VA50 16	//1
#define MAX_BACKLIGHT_VALUE_PLS 1312 //82
#define MID_BACKLIGHT_VALUE_PLS 528 //33
#define LOW_BACKLIGHT_VALUE_PLS 16	//1
#define DIM_BACKLIGHT_VALUE_PLS 16	//1
#define MAX_BACKLIGHT_VALUE_TN	1280 //80
#define MID_BACKLIGHT_VALUE_TN	 512 //32
#define LOW_BACKLIGHT_VALUE_TN	 16	//1
#define DIM_BACKLIGHT_VALUE_TN	 16	//1
#define MAX_BACKLIGHT_VALUE_FFS 1504 //94
#define MID_BACKLIGHT_VALUE_FFS 608 //38
#define LOW_BACKLIGHT_VALUE_FFS 16	//1
#define DIM_BACKLIGHT_VALUE_FFS 16	//1
#define MAX_BACKLIGHT_VALUE_LCDPLS	1360 //85
#define MID_BACKLIGHT_VALUE_LCDPLS	 544 //34
//#define LOW_BACKLIGHT_VALUE_LCDPLS	 64	//1
//#define DIM_BACKLIGHT_VALUE_LCDPLS	 64	//1
#define LOW_BACKLIGHT_VALUE_LCDPLS	 16	//1
#define DIM_BACKLIGHT_VALUE_LCDPLS	 16	//1
#define MAX_BACKLIGHT_VALUE_T7	1280 //80
#define MID_BACKLIGHT_VALUE_T7	 512 //32
#define LOW_BACKLIGHT_VALUE_T7	 16	//1
#define DIM_BACKLIGHT_VALUE_T7	 16	//1
#define MAX_BACKLIGHT_VALUE_T8	1280 //80
#define MID_BACKLIGHT_VALUE_T8	 512 //32
#define LOW_BACKLIGHT_VALUE_T8	 16	//1
#define DIM_BACKLIGHT_VALUE_T8	 16	//1
typedef enum
{
	LCD_TYPE_VA,
	LCD_TYPE_PLS,
	LCD_TYPE_VA50,
	LCD_TYPE_TN,
	LCD_TYPE_FFS,	
	LCD_TYPE_LCDPLS,	
	LCD_TYPE_T7,
	LCD_TYPE_T8,
	LCD_TYPE_MAX,
}Lcd_Type;
extern Lcd_Type lcd_type;

#ifdef CONFIG_FB_S3C_MDNIE
extern void init_mdnie_class(void);
#endif

extern void tune_cmc623_pwm_brightness(int value);
//extern void s5p_mdine_pwm_enable(int on);
extern void qt602240_inform_first_brightness(void);

static struct early_suspend	st_early_suspend;
static struct platform_device *bl_pdev;

static int current_gamma_level = MAX_LEVEL;

static int first_brightness_setting = 1;
static int cmc623_pwm_suspended;
static int current_intensity = 0;
//static DEFINE_SPINLOCK(cmc623_pwm_lock);
static DEFINE_MUTEX(cmc623_pwm_mutex);

static void cmc623_pwm_apply(struct platform_device *pdev, int level)
{

	tune_cmc623_pwm_brightness(level);

	current_gamma_level = level;

	dev_dbg(&pdev->dev, "%s:current_gamma_level=%d\n", __func__, current_gamma_level);
}

static void cmc623_pwm_apply_brightness(struct platform_device *pdev, int level)
{
	cmc623_pwm_apply(pdev, level);

	dev_dbg(&pdev->dev, "%s : apply_level=%d\n", __func__, level);
//	printk("%s : apply_level=%d\n", __func__, level);
}

static int cmc623_pwm_get_tune(int level)
{
	int tune_value;

	switch(lcd_type)
		{
		case LCD_TYPE_PLS:
			if(level >= MID_BRIGHTNESS_LEVEL)
				tune_value = (level - MID_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE_PLS-MID_BACKLIGHT_VALUE_PLS) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + MID_BACKLIGHT_VALUE_PLS;
			else if(level >= LOW_BRIGHTNESS_LEVEL)
				tune_value = (level - LOW_BRIGHTNESS_LEVEL) * (MID_BACKLIGHT_VALUE_PLS-LOW_BACKLIGHT_VALUE_PLS) / (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE_PLS;
			else if(level >= DIM_BACKLIGHT_LEVEL)
				tune_value = (level - DIM_BACKLIGHT_LEVEL) * (LOW_BACKLIGHT_VALUE_PLS-DIM_BACKLIGHT_VALUE_PLS) / (LOW_BRIGHTNESS_LEVEL-DIM_BACKLIGHT_LEVEL) + DIM_BACKLIGHT_VALUE_PLS;
			else if(level > 0)
				tune_value = (level) * (DIM_BACKLIGHT_VALUE_PLS) / (DIM_BACKLIGHT_LEVEL);
			else
				tune_value = level;
			break;
		case LCD_TYPE_LCDPLS:
			if(level >= MID_BRIGHTNESS_LEVEL)
				tune_value = (level - MID_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE_LCDPLS-MID_BACKLIGHT_VALUE_LCDPLS) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + MID_BACKLIGHT_VALUE_LCDPLS;
			else if(level >= LOW_BRIGHTNESS_LEVEL)
				tune_value = (level - LOW_BRIGHTNESS_LEVEL) * (MID_BACKLIGHT_VALUE_LCDPLS-LOW_BACKLIGHT_VALUE_LCDPLS) / (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE_LCDPLS;
			else if(level >= DIM_BACKLIGHT_LEVEL)
				tune_value = (level - DIM_BACKLIGHT_LEVEL) * (LOW_BACKLIGHT_VALUE_LCDPLS-DIM_BACKLIGHT_VALUE_LCDPLS) / (LOW_BRIGHTNESS_LEVEL-DIM_BACKLIGHT_LEVEL) + DIM_BACKLIGHT_VALUE_LCDPLS;
			else if(level > 0)
				tune_value = (level) * (DIM_BACKLIGHT_VALUE_LCDPLS) / (DIM_BACKLIGHT_LEVEL);
//				tune_value = DIM_BACKLIGHT_VALUE_LCDPLS;
			else
				tune_value = level;
			break;
		case LCD_TYPE_VA50:
			if(level >= MID_BRIGHTNESS_LEVEL)
				tune_value = (level - MID_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE_VA50-MID_BACKLIGHT_VALUE_VA50) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + MID_BACKLIGHT_VALUE_VA50;
			else if(level >= LOW_BRIGHTNESS_LEVEL)
				tune_value = (level - LOW_BRIGHTNESS_LEVEL) * (MID_BACKLIGHT_VALUE_VA50-LOW_BACKLIGHT_VALUE_VA50) / (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE_VA50;
			else if(level >= DIM_BACKLIGHT_LEVEL)
				tune_value = (level - DIM_BACKLIGHT_LEVEL) * (LOW_BACKLIGHT_VALUE_VA50-DIM_BACKLIGHT_VALUE_VA50) / (LOW_BRIGHTNESS_LEVEL-DIM_BACKLIGHT_LEVEL) + DIM_BACKLIGHT_VALUE_VA50;
			else if(level > 0)
				tune_value = (level) * (DIM_BACKLIGHT_VALUE_VA50) / (DIM_BACKLIGHT_LEVEL);
			else
				tune_value = level;
			break;
		case LCD_TYPE_FFS:
			if(level >= MID_BRIGHTNESS_LEVEL)
				tune_value = (level - MID_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE_FFS-MID_BACKLIGHT_VALUE_FFS) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + MID_BACKLIGHT_VALUE_FFS;
			else if(level >= LOW_BRIGHTNESS_LEVEL)
				tune_value = (level - LOW_BRIGHTNESS_LEVEL) * (MID_BACKLIGHT_VALUE_FFS-LOW_BACKLIGHT_VALUE_FFS) / (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE_FFS;
			else if(level >= DIM_BACKLIGHT_LEVEL)
				tune_value = (level - DIM_BACKLIGHT_LEVEL) * (LOW_BACKLIGHT_VALUE_FFS-DIM_BACKLIGHT_VALUE_FFS) / (LOW_BRIGHTNESS_LEVEL-DIM_BACKLIGHT_LEVEL) + DIM_BACKLIGHT_VALUE_FFS;
			else if(level > 0)
				tune_value = (level) * (DIM_BACKLIGHT_VALUE_FFS) / (DIM_BACKLIGHT_LEVEL);
			else
				tune_value = level;
			break;
		case LCD_TYPE_TN:
			if(level >= MID_BRIGHTNESS_LEVEL)
				tune_value = (level - MID_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE_TN-MID_BACKLIGHT_VALUE_TN) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + MID_BACKLIGHT_VALUE_TN;
			else if(level >= LOW_BRIGHTNESS_LEVEL)
				tune_value = (level - LOW_BRIGHTNESS_LEVEL) * (MID_BACKLIGHT_VALUE_TN-LOW_BACKLIGHT_VALUE_TN) / (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE_TN;
			else if(level >= DIM_BACKLIGHT_LEVEL)
				tune_value = (level - DIM_BACKLIGHT_LEVEL) * (LOW_BACKLIGHT_VALUE_TN-DIM_BACKLIGHT_VALUE_TN) / (LOW_BRIGHTNESS_LEVEL-DIM_BACKLIGHT_LEVEL) + DIM_BACKLIGHT_VALUE_TN;
			else if(level > 0)
				tune_value = (level) * (DIM_BACKLIGHT_VALUE_TN) / (DIM_BACKLIGHT_LEVEL);
			else
				tune_value = level;
			break;
		case LCD_TYPE_T7:
			if(level >= MID_BRIGHTNESS_LEVEL)
				tune_value = (level - MID_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE_T7-MID_BACKLIGHT_VALUE_T7) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + MID_BACKLIGHT_VALUE_T7;
			else if(level >= LOW_BRIGHTNESS_LEVEL)
				tune_value = (level - LOW_BRIGHTNESS_LEVEL) * (MID_BACKLIGHT_VALUE_T7-LOW_BACKLIGHT_VALUE_T7) / (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE_T7;
			else if(level >= DIM_BACKLIGHT_LEVEL)
				tune_value = (level - DIM_BACKLIGHT_LEVEL) * (LOW_BACKLIGHT_VALUE_T7-DIM_BACKLIGHT_VALUE_T7) / (LOW_BRIGHTNESS_LEVEL-DIM_BACKLIGHT_LEVEL) + DIM_BACKLIGHT_VALUE_T7;
			else if(level > 0)
				tune_value = (level) * (DIM_BACKLIGHT_VALUE_T7) / (DIM_BACKLIGHT_LEVEL);
			else
				tune_value = level;
			break;
		case LCD_TYPE_T8:
			if(level >= MID_BRIGHTNESS_LEVEL)
				tune_value = (level - MID_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE_T8-MID_BACKLIGHT_VALUE_T8) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + MID_BACKLIGHT_VALUE_T8;
			else if(level >= LOW_BRIGHTNESS_LEVEL)
				tune_value = (level - LOW_BRIGHTNESS_LEVEL) * (MID_BACKLIGHT_VALUE_T8-LOW_BACKLIGHT_VALUE_T8) / (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE_T8;
			else if(level >= DIM_BACKLIGHT_LEVEL)
				tune_value = (level - DIM_BACKLIGHT_LEVEL) * (LOW_BACKLIGHT_VALUE_T8-DIM_BACKLIGHT_VALUE_T8) / (LOW_BRIGHTNESS_LEVEL-DIM_BACKLIGHT_LEVEL) + DIM_BACKLIGHT_VALUE_T8;
			else if(level > 0)
				tune_value = (level) * (DIM_BACKLIGHT_VALUE_T8) / (DIM_BACKLIGHT_LEVEL);
			else
				tune_value = level;
			break;
		case LCD_TYPE_VA:
		default:
			if(level >= MID_BRIGHTNESS_LEVEL)
				tune_value = (level - MID_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE_VA-MID_BACKLIGHT_VALUE_VA) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + MID_BACKLIGHT_VALUE_VA;
			else if(level >= LOW_BRIGHTNESS_LEVEL)
				tune_value = (level - LOW_BRIGHTNESS_LEVEL) * (MID_BACKLIGHT_VALUE_VA-LOW_BACKLIGHT_VALUE_VA) / (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE_VA;
			else if(level >= DIM_BACKLIGHT_LEVEL)
				tune_value = (level - DIM_BACKLIGHT_LEVEL) * (LOW_BACKLIGHT_VALUE_VA-DIM_BACKLIGHT_VALUE_VA) / (LOW_BRIGHTNESS_LEVEL-DIM_BACKLIGHT_LEVEL) + DIM_BACKLIGHT_VALUE_VA;
			else if(level > 0)
				tune_value = (level) * (DIM_BACKLIGHT_VALUE_VA) / (DIM_BACKLIGHT_LEVEL);
			else
				tune_value = level;
			break;
		}

	return tune_value;
}


static void cmc623_pwm_backlight_ctl(struct platform_device *pdev, int intensity)
{
	int tune_level;
	
	// brightness tuning
	tune_level = cmc623_pwm_get_tune(intensity);

	printk("[cmc]%d(%d)\n", intensity, tune_level);

	cmc623_pwm_apply_brightness(pdev, tune_level);
}


static void cmc623_pwm_send_intensity(struct backlight_device *bd)
{
	//unsigned long flags;
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
		cmc623_pwm_suspended)
		{
		printk("[cmc]i:%d(c:%d)\n", intensity, current_intensity);
		if(!current_intensity)
			return;
		msleep(1);
		intensity = 0;
	}

	//spin_lock_irqsave(&cmc623_pwm_lock, flags);
	//spin_lock(&cmc623_pwm_lock);
	mutex_lock(&cmc623_pwm_mutex);

	cmc623_pwm_backlight_ctl(pdev, intensity);

	//spin_unlock_irqrestore(&cmc623_pwm_lock, flags);
	//spin_unlock(&cmc623_pwm_lock);
	mutex_unlock(&cmc623_pwm_mutex);

	current_intensity = intensity;
}

static void cmc623_pwm_gpio_init()
{
//	s3c_gpio_cfgpin(GPIO_LCD_CABC_PWM_R05, S3C_GPIO_SFN(3));	//mdnie pwm
//    s3c_gpio_setpull(GPIO_LCD_CABC_PWM_R05, S3C_GPIO_PULL_NONE);
}

#ifdef CONFIG_PM
static int cmc623_pwm_suspend(struct platform_device *swi_dev, pm_message_t state)
{
	struct backlight_device *bd = platform_get_drvdata(swi_dev);

	cmc623_pwm_suspended = 1;
	cmc623_pwm_send_intensity(bd);
	
	return 0;
}

static int cmc623_pwm_resume(struct platform_device *swi_dev)
{	
	struct backlight_device *bd = platform_get_drvdata(swi_dev);

	bd->props.brightness = CMC623_PWM_DEFAULT_INTENSITY;
	cmc623_pwm_suspended = 0;
	cmc623_pwm_send_intensity(bd);

	return 0;
}
#else
#define cmc623_pwm_suspend		NULL
#define cmc623_pwm_resume		NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cmc623_pwm_early_suspend(struct early_suspend *h)
{	
	struct backlight_device *bd = platform_get_drvdata(bl_pdev);

	cmc623_pwm_suspended = 1;
	cmc623_pwm_send_intensity(bd);

	return;
}

static void cmc623_pwm_early_resume(struct early_suspend *h)
{
	struct backlight_device *bd = platform_get_drvdata(bl_pdev);

	//bd->props.brightness = cmc623_pwm_DEFAULT_INTENSITY;
	cmc623_pwm_suspended = 0;

	cmc623_pwm_gpio_init();
		
	cmc623_pwm_send_intensity(bd);
	
	return;
}
#endif

static int cmc623_pwm_check_fb(struct backlight_device *bd, struct fb_info *fi)
{
	
	if(bd->dev.parent->parent)
		{
		struct s3cfb_global *fbdev = platform_get_drvdata(to_platform_device(bd->dev.parent->parent));
		struct s3c_platform_fb *pdata = to_platform_device(bd->dev.parent->parent)->dev.platform_data;
		if(fbdev->fb[pdata->default_win] == fi)
			{
			printk(KERN_INFO "%s fb matched\n", __func__);
			return 1;
			}
		}

	//printk(KERN_DEBUG "%s fb unmatched\n", __func__);

	return 0;
}

static int cmc623_pwm_set_intensity(struct backlight_device *bd)
{
	//printk("BD->PROPS.BRIGHTNESS = %d\n", bd->props.brightness);

	cmc623_pwm_send_intensity(bd);

	if (first_brightness_setting) {
		first_brightness_setting = 0;
		/* to touch driver */
		qt602240_inform_first_brightness();
	}
	
	return 0;
}


static int cmc623_pwm_get_intensity(struct backlight_device *bd)
{
	return current_intensity;
}


static struct backlight_ops cmc623_pwm_ops = {
	.get_brightness = cmc623_pwm_get_intensity,
	.update_status  = cmc623_pwm_set_intensity,
	.check_fb = cmc623_pwm_check_fb,
};

//for measuring luminance
void cmc623_pwm_set_brightness(int brightness)
{
	//unsigned long flags;

	printk("%s: value=%d\n", __func__, brightness);

	//spin_lock_irqsave(&cmc623_pwm_lock, flags);
	//spin_lock(&cmc623_pwm_lock);
	mutex_lock(&cmc623_pwm_mutex);

	cmc623_pwm_apply_brightness(bl_pdev, brightness);

	//spin_unlock_irqrestore(&cmc623_pwm_lock, flags);
	//spin_unlock(&cmc623_pwm_lock);
	mutex_unlock(&cmc623_pwm_mutex);
}
EXPORT_SYMBOL(cmc623_pwm_set_brightness);

static int cmc623_pwm_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;

	printk("cmc623_pwm Probe START!!!\n");

	bd = backlight_device_register("s5p_bl", &pdev->dev, pdev, &cmc623_pwm_ops, NULL);

	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);

	bd->props.max_brightness = CMC623_PWM_MAX_INTENSITY;
	bd->props.brightness = CMC623_PWM_DEFAULT_INTENSITY;

	cmc623_pwm_gpio_init();

//	s5p_mdine_pwm_enable(1);

//	cmc623_pwm_set_intensity(bd);

	dev_info(&pdev->dev, "cmc623_pwm backlight driver is enabled.\n");

	bl_pdev = pdev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	st_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	st_early_suspend.suspend = cmc623_pwm_early_suspend;
	st_early_suspend.resume = cmc623_pwm_early_resume;
	register_early_suspend(&st_early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#ifdef CONFIG_FB_S3C_MDNIE
//	init_mdnie_class();  //set mDNIe UI mode, Outdoormode
#endif

	printk("cmc623_pwm Probe END!!!\n");
	return 0;

}

static int cmc623_pwm_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&st_early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	bd->props.brightness = 0;
	bd->props.power = 0;
	cmc623_pwm_send_intensity(bd);

	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver cmc623_pwm_driver = {
	.driver		= {
		.name	= "cmc623_pwm_bl",
		.owner	= THIS_MODULE,
	},
	.probe		= cmc623_pwm_probe,
	.remove		= cmc623_pwm_remove,
#if !(defined CONFIG_HAS_EARLYSUSPEND)
	.suspend	= cmc623_pwm_suspend,
	.resume		= cmc623_pwm_resume,
#endif
};

static int __init cmc623_pwm_init(void)
{	
	return platform_driver_register(&cmc623_pwm_driver);
}

static void __exit cmc623_pwm_exit(void)
{
	platform_driver_unregister(&cmc623_pwm_driver);
}

module_init(cmc623_pwm_init);
module_exit(cmc623_pwm_exit);
