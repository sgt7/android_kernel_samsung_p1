/*
 * LMS700JF TFT-LCD Panel Driver for the Samsung Universal board
 *
 * Derived from drivers/video/omap/lcd-apollon.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/lcd.h>
#include <linux/backlight.h>
#include <linux/regulator/consumer.h>

#include <plat/gpio-cfg.h>
//#include <plat/regs-lcd.h>

#include <mach/gpio-p1.h>

#include "s3cfb.h"

#define SLEEPMSEC		0x1000
#define ENDDEF			0x2000
#define	DEFMASK			0xFF00
#define COMMAND_ONLY		0xFE
#define DATA_ONLY		0xFF

#define DIM_BL	20
#define MIN_BL	30
#define MAX_BL	255

#define MAX_GAMMA_VALUE	24	// we have 25 levels. -> 16 levels -> 24 levels
#define CRITICAL_BATTERY_LEVEL 5

#define GAMMASET_CONTROL //for 1.9/2.2 gamma control from platform
#define ACL_ENABLE

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define ADC_LOWER_LIMIT_PLS_TYPE	3850
#define ADC_LOWER_LIMIT_T8_TYPE	3400
#define ADC_LOWER_LIMIT_LCDPLS_TYPE	2900
#define ADC_LOWER_LIMIT_VA50_TYPE	2230
#define ADC_LOWER_LIMIT_FFS_TYPE	1600
#define ADC_LOWER_LIMIT_TN_TYPE		900
#define ADC_LOWER_LIMIT_T7_TYPE	350
#define ADC_LOWER_LIMIT_VA_TYPE		0

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

Lcd_Type lcd_type = LCD_TYPE_VA;
static Lcd_Type lcd_vendor_by_adc = LCD_TYPE_MAX;
extern int s3c_adc_get_adc_data(int channel);
#define SEC_LCD_ADC_CHANNEL 2

/*********** for debug **********************************************************/
#if 0 
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

extern unsigned int HWREV;

//extern unsigned int get_battery_level(void);
//extern unsigned int is_charging_enabled(void);
extern void cmc623_cabc_enable(int enable);
extern void cmc623_autobrightness_enable(int enable);

struct s5p_lcd{
	struct platform_device *pdev;
	struct lcd_device *lcd_dev;
};

#if 0
#ifdef GAMMASET_CONTROL
struct class *gammaset_class;
struct device *switch_gammaset_dev;
#endif
#endif

#ifdef ACL_ENABLE
int cabc_enable = 0;
int cur_acl = 0;

struct class *cabc_class;
struct device *switch_cabcset_dev;
#endif

#ifdef CONFIG_FB_S3C_MDNIE
extern void init_mdnie_class(void);
#endif

static DEFINE_MUTEX(lms700_power_lock);

static struct regulator *regulator_lvds33;

int autobrightness_enable = 0;
static struct s5p_lcd lcd;

struct lms600_state_type{
	unsigned int powered_up;
};

static struct lms600_state_type lms700_state = { 
	.powered_up = TRUE,
};


#if 0
int IsLDIEnabled(void)
{
	return ldi_enable;
}
EXPORT_SYMBOL(IsLDIEnabled);


static void SetLDIEnabledFlag(int OnOff)
{
	ldi_enable = OnOff;
}
#endif

#if 0
void tl2796_ldi_init(void)
{
	s6e63m0_panel_send_sequence(s6e63m0_SEQ_SETTING);
	s6e63m0_panel_send_sequence(s6e63m0_SEQ_STANDBY_OFF);

	SetLDIEnabledFlag(1);
	printk(KERN_DEBUG "LDI enable ok\n");
	dev_dbg(lcd.lcd_dev,"%s::%d -> ldi initialized\n",__func__,__LINE__);	
}
#endif

void lms700_powerup(void)
{
	int ret;
	
	printk(KERN_INFO "%s(%d)\n", __func__, lms700_state.powered_up);

	mutex_lock(&lms700_power_lock);

	if(!lms700_state.powered_up)
		{
		// ldo enable
		ret = regulator_enable(regulator_lvds33);
		if(ret<0)
			printk(KERN_ERR "%s: is_enabled() failed for regulator_lvds33: %d\n", __func__, ret);

	//#if !defined(CONFIG_TARGET_LOCALE_LTN)  // CYS_ 2010.07.02 for Camera Preview
	//	if(HWREV <= 10)		// MIDAS[2010.09.14] MLCD_ON control below rev0.4 (EUR) or rev0.8(KOR)
	//		gpio_set_value(GPIO_MLCD_ON, 1);
	//#endif

		gpio_set_value(GPIO_LCD_LDO_EN, 1);		//backlight & LCD

#if defined(CONFIG_TARGET_LOCALE_EUR) || defined (CONFIG_TARGET_LOCALE_HKTW) || defined (CONFIG_TARGET_LOCALE_HKTW_FET) || defined (CONFIG_TARGET_LOCALE_VZW) || defined (CONFIG_TARGET_LOCALE_USAGSM)
		if(HWREV >= 13)		// above rev0.7 (EUR)
#elif defined(CONFIG_TARGET_LOCALE_KOR)
		if(HWREV >= 13)		// above rev1.1 (KOR)
#else
		if(1)
#endif
			{
			msleep(150);		// waiting LCD turned on
			gpio_set_value(GPIO_LVDS_SHDN, 1);
			msleep(1);
			}
		else
			{
			msleep(500);		// waiting LCD turned on
			}

		lms700_state.powered_up = TRUE;
		}

	mutex_unlock(&lms700_power_lock);
	
	dev_dbg(&lcd.lcd_dev->dev,"%s::%d\n",__func__,__LINE__);	
}
EXPORT_SYMBOL(lms700_powerup);

void lms700_powerdown(void)
{
	int ret;
	
	printk(KERN_INFO "%s(%d)\n", __func__, lms700_state.powered_up);

	mutex_lock(&lms700_power_lock);

	if(lms700_state.powered_up)
		{
		// Disable LDOs
#if defined (CONFIG_TARGET_LOCALE_EUR) || defined (CONFIG_TARGET_LOCALE_HKTW) || defined (CONFIG_TARGET_LOCALE_HKTW_FET) || defined (CONFIG_TARGET_LOCALE_VZW) || defined (CONFIG_TARGET_LOCALE_USAGSM)
		if(HWREV >= 13)		// above rev0.7 (EUR)
#elif defined(CONFIG_TARGET_LOCALE_KOR)
		if(HWREV >= 13)		// above rev1.1 (KOR)
#endif
			{
			gpio_set_value(GPIO_LVDS_SHDN, 0);
			}
		msleep(20);
		
		gpio_set_value(GPIO_LCD_LDO_EN, 0);
		msleep(150);

	//#if !defined(CONFIG_TARGET_LOCALE_LTN) // CYS_ 2010.07.02 for Camera Preview
	//	if(HWREV <= 10)		// MIDAS[2010.09.14] MLCD_ON control below rev0.4 (EUR) or rev0.8(KOR)
	//		gpio_set_value(GPIO_MLCD_ON, 0);
	//#endif

		ret = regulator_disable(regulator_lvds33);
		if(ret<0)
			printk(KERN_ERR "%s: is_disabled() failed for regulator_lvds33: %d\n", __func__, ret);

		lms700_state.powered_up = FALSE;
		}

	mutex_unlock(&lms700_power_lock);
	
	dev_dbg(&lcd.lcd_dev->dev,"%s::%d\n",__func__,__LINE__);	
}
EXPORT_SYMBOL(lms700_powerdown);

#if 0
void tl2796_ldi_enable(void)
{
}

void tl2796_ldi_disable(void)
{
	s6e63m0_panel_send_sequence(s6e63m0_SEQ_STANDBY_ON);
	s6e63m0_panel_send_sequence(s6e63m0_SEQ_DISPLAY_OFF);

	SetLDIEnabledFlag(0);
	printk(KERN_DEBUG "LDI disable ok\n");
	dev_dbg(&lcd.lcd_dev->dev,"%s::%d -> ldi disabled\n",__func__,__LINE__);	
}

void s3cfb_set_lcd_info(struct s3cfb_global *ctrl)
{
	s6e63m0.init_ldi = NULL;
	ctrl->lcd = &s6e63m0;
}
#endif

//mkh:lcd operations and functions
int s5p_lcd_set_power(struct lcd_device *ld, int power)
{
	// Originally, an argument power means LCD panel power status (0: full on, 1..3: controller power on, flat panel power off, 4: full off)
	// In this point of view, this function is wrong obviously.
	// But we still use this implementation for compatibility 
	printk("s5p_lcd_set_power is called: %d", power);
	if(power)
	{
		lms700_powerup();
		//s6e63m0_panel_send_sequence(s6e63m0_SEQ_DISPLAY_ON);
	}
	else
	{
		lms700_powerdown();
		//s6e63m0_panel_send_sequence(s6e63m0_SEQ_DISPLAY_OFF);
	}

	return 0;
}
EXPORT_SYMBOL(s5p_lcd_set_power);

static int s5p_lcd_check_fb(struct lcd_device *lcddev, struct fb_info *fi)
{
	// it doesn't use fb notifier
	return 0;	
}

static struct lcd_ops s5p_lcd_ops = {
	.set_power = s5p_lcd_set_power,
	.check_fb = s5p_lcd_check_fb,
};


#ifdef ACL_ENABLE 
static ssize_t cabcset_file_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "%s \n", __func__);

	return sprintf(buf,"%u\n", cabc_enable);
}
static ssize_t cabcset_file_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *endp;
	int enable = simple_strtoul(buf, &endp, 0);

	printk(KERN_NOTICE "%s:%d\n", __func__, enable);

	//printk(KERN_INFO "[acl set] in aclset_file_cmd_store, input value = %d \n", value);

	cmc623_cabc_enable(enable);
	cabc_enable = enable;

	
#if 0
	if(IsLDIEnabled()==0)
	{
		printk(KERN_DEBUG "[acl set] return because LDI is disabled, input value = %d \n",value);
		return size;
	}

	if(value==1 && acl_enable == 0)
	{		
		acl_enable = value;
		
		s6e63m0_panel_send_sequence(acl_cutoff_init);
		msleep(20);
	
		if (current_gamma_value ==1)
		{
			s6e63m0_panel_send_sequence(ACL_cutoff_set[0]); //set 0% ACL
			cur_acl = 0;
			//printk(" ACL_cutoff_set Percentage : 0!!\n");
		}
		else if(current_gamma_value ==2)
		{
			s6e63m0_panel_send_sequence(ACL_cutoff_set[1]); //set 12% ACL
			cur_acl = 12;
			//printk(" ACL_cutoff_set Percentage : 12!!\n");
		}
		else if(current_gamma_value ==3)
		{
			s6e63m0_panel_send_sequence(ACL_cutoff_set[2]); //set 22% ACL
			cur_acl = 22;
			//printk(" ACL_cutoff_set Percentage : 22!!\n");
		}
		else if(current_gamma_value ==4)
		{
			s6e63m0_panel_send_sequence(ACL_cutoff_set[3]); //set 30% ACL
			cur_acl = 30;
			//printk(" ACL_cutoff_set Percentage : 30!!\n");
		}
		else if(current_gamma_value ==5)
		{
			s6e63m0_panel_send_sequence(ACL_cutoff_set[4]); //set 35% ACL
			cur_acl = 35;
			//printk(" ACL_cutoff_set Percentage : 35!!\n");
		}
		else
		{
			s6e63m0_panel_send_sequence(ACL_cutoff_set[5]); //set 40% ACL
			cur_acl = 40;
			//printk(" ACL_cutoff_set Percentage : 40!!\n");
		}
	}
	else if(value==0 && acl_enable == 1)
	{
		acl_enable = value;
		
		//ACL Off
		s6e63m0_panel_send_sequence(ACL_cutoff_set[0]); //ACL OFF
		//printk(" ACL_cutoff_set Percentage : 0!!\n");
		cur_acl  = 0;
	}
	else
		printk("\naclset_file_cmd_store value is same : value(%d)\n",value);

#endif

	return size;
}

static DEVICE_ATTR(cabcset_file_cmd,0664, cabcset_file_cmd_show, cabcset_file_cmd_store);
#endif


static void lms700_shutdown(struct platform_device *dev)
{
	//printk("%s\n", __func__);
	
	// Disable LDOs
#if defined(CONFIG_MACH_P1) && defined(CONFIG_TARGET_LOCALE_EUR) || defined(CONFIG_TARGET_LOCALE_HKTW) || defined (CONFIG_TARGET_LOCALE_HKTW_FET) || defined(CONFIG_TARGET_LOCALE_VZW) || defined (CONFIG_TARGET_LOCALE_USAGSM)
	if(HWREV >= 13)		// above rev0.7
#endif
		{
		gpio_set_value(GPIO_LVDS_SHDN, 0);
		}
	gpio_set_value(GPIO_LCD_LDO_EN, 0);
}

static ssize_t lightsensor_file_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "%s \n", __func__);

	return sprintf(buf,"%u\n", autobrightness_enable);
}

static ssize_t lightsensor_file_state_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	char *endp;
	int enable = simple_strtoul(buf, &endp, 0);
	printk(KERN_NOTICE "%s:%d\n", __func__, enable);
	
	autobrightness_enable = enable;
	cmc623_autobrightness_enable(enable);

	return size;
}

static DEVICE_ATTR(lightsensor_file_state,0664, lightsensor_file_state_show, lightsensor_file_state_store);

static const char lcdtype_name[][64] = {
		"SMD_LMS700JF03",
		"HYD_HV070WS1",
		"SMD_LMS700JF06",
		"HAN_HSD070PFW1",
		"BOE_HV070WSA",
		"SEC_LTN070NL01",
		"XXX_T7",
		"XXX_T8"
		};

static ssize_t lcdtype_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "%s\n", __func__);

	switch(lcd_vendor_by_adc) {
	case LCD_TYPE_PLS:
		pr_info("type(PLS): %s\n",lcdtype_name[1]);
		return sprintf(buf,lcdtype_name[1]);
		break;
	case LCD_TYPE_VA50:
		pr_info("type(VA50): %s\n",lcdtype_name[2]);
		return sprintf(buf,lcdtype_name[2]);
		break;
	case LCD_TYPE_TN:
		pr_info("type(TN): %s\n",lcdtype_name[3]);
		return sprintf(buf,lcdtype_name[3]);
		break;
	case LCD_TYPE_FFS:
		pr_info("type(FFS): %s\n",lcdtype_name[4]);
		return sprintf(buf,lcdtype_name[4]);
		break;
	case LCD_TYPE_LCDPLS:
		pr_info("type(LCDPLS): %s\n",lcdtype_name[5]);
		return sprintf(buf,lcdtype_name[5]);
		break;
	case LCD_TYPE_T7:
		pr_info("type(T7): %s\n",lcdtype_name[6]);
		return sprintf(buf,lcdtype_name[6]);
		break;
	case LCD_TYPE_T8:
		pr_info("type(T8): %s\n",lcdtype_name[7]);
		return sprintf(buf,lcdtype_name[7]);
		break;
	case LCD_TYPE_VA:
	default:
		pr_info("type(VA70): %s\n",lcdtype_name[0]);
		return sprintf(buf,lcdtype_name[0]);
		break;
	}

}

static ssize_t lcdtype_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	printk(KERN_NOTICE "%s:%s\n", __func__, buf);

	return size;
}

static DEVICE_ATTR(lcdtype,0644, lcdtype_show, lcdtype_store);

static ssize_t lcdid_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int lcd_adc = 0;
	printk(KERN_INFO "%s \n", __func__);

	lcd_adc = s3c_adc_get_adc_data(SEC_LCD_ADC_CHANNEL);
	pr_info("lcd_adc : %d\n",lcd_adc);

	return sprintf(buf,"%u\n", lcd_adc);
}

static ssize_t lcdid_adc_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	printk(KERN_NOTICE "%s:%s\n", __func__, buf);

	return size;
}

static DEVICE_ATTR(lcdid_adc,0644, lcdid_adc_show, lcdid_adc_store);

extern struct class *sec_class;
struct device *sec_lcd_dev;

static int __init lms700_probe(struct platform_device *pdev)
{
	int ret=0;
	int lcd_adc = 0;

	lcd.pdev = pdev;
	lcd.lcd_dev = lcd_device_register("s5p_lcd",&pdev->dev,&lcd,&s5p_lcd_ops);	//remove lcd_ops on gingerbread
	platform_set_drvdata(pdev, &lcd);

	if (IS_ERR_OR_NULL(regulator_lvds33)) {
	        regulator_lvds33 = regulator_get(NULL, "vcc_lcd");
	        if (IS_ERR_OR_NULL(regulator_lvds33)) {
	               pr_err("failed to get LVDS 3.3V regulator");
	               return -EINVAL;
	        }
	}
	if(regulator_enable(regulator_lvds33))	// default enable
		printk(KERN_ERR "%s: is_enabled() failed for regulator_lvds33\n", __func__);

//        SetLDIEnabledFlag(1);

#if 0
#ifdef GAMMASET_CONTROL //for 1.9/2.2 gamma control from platform
	gammaset_class = class_create(THIS_MODULE, "gammaset");
	if (IS_ERR(gammaset_class))
		pr_err("Failed to create class(gammaset_class)!\n");

	switch_gammaset_dev = device_create(gammaset_class, NULL, 0, NULL, "switch_gammaset");
	if (IS_ERR(switch_gammaset_dev))
		pr_err("Failed to create device(switch_gammaset_dev)!\n");

	if (device_create_file(switch_gammaset_dev, &dev_attr_gammaset_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gammaset_file_cmd.attr.name);
#endif	
#endif

	cabc_class = class_create(THIS_MODULE, "cabcset");
	if (IS_ERR(cabc_class))
		pr_err("Failed to create class(acl_class)!\n");

	switch_cabcset_dev = device_create(cabc_class, NULL, 0, NULL, "switch_cabcset");
	if (IS_ERR(switch_cabcset_dev))
		pr_err("Failed to create device(switch_cabcset_dev)!\n");

	if (device_create_file(switch_cabcset_dev, &dev_attr_cabcset_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_cabcset_file_cmd.attr.name);

	if (device_create_file(switch_cabcset_dev, &dev_attr_lightsensor_file_state) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_lightsensor_file_state.attr.name);

	sec_lcd_dev = device_create(sec_class, NULL, 0, NULL, "sec_lcd");
	if (IS_ERR(sec_lcd_dev))
		pr_err("Failed to create device(sec_lcd_dev)!\n");

	if (device_create_file(sec_lcd_dev, &dev_attr_lcdtype) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_lcdtype.attr.name);

	if (device_create_file(sec_lcd_dev, &dev_attr_lcdid_adc) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_lcdid_adc.attr.name);


#ifdef CONFIG_FB_S3C_MDNIE
	init_mdnie_class();  //set mDNIe UI mode, Outdoormode
#endif

	if (ret < 0){
		pr_err("%s::%d-> lms700 probe failed Err=%d\n",__func__,__LINE__,ret);
		return ret;
	}
	pr_info("%s::%d->lms700 probed successfuly\n",__func__,__LINE__);

	pr_info("HWREV : %d\n",HWREV);
	//check lcd type
//#if defined(CONFIG_TARGET_LOCALE_KOR)
//	if(HWREV >= 14)		// above rev1.2 (KOR)
//	{
//		lcd_type = LCD_TYPE_PLS;
//		lcd_adc = s3c_adc_get_adc_data(SEC_LCD_ADC_CHANNEL);
//		pr_info("lcd_adc : %d\n",lcd_adc);
//	}
//	else
//	{
//		lcd_type = LCD_TYPE_VA;
//		lcd_adc = 0;
//	}
//#elif defined(CONFIG_TARGET_LOCALE_VZW) 
//	if(HWREV >= 7)		
//	{
//		lcd_type = LCD_TYPE_PLS;
//		lcd_adc = s3c_adc_get_adc_data(SEC_LCD_ADC_CHANNEL);
//		pr_info("lcd_adc : %d\n",lcd_adc);
//	}
//	else
//	{
//		lcd_type = LCD_TYPE_VA;
//		lcd_adc = 0;
//	}
#if defined(CONFIG_TARGET_LOCALE_KOR)
	if(HWREV >= 14)		// above rev1.2 (KOR)
#elif defined(CONFIG_TARGET_LOCALE_VZW) 
	if(HWREV >= 7)		
#else
// EUR and HKTW and HKTW_FET and USAGSM and etc
	if(HWREV >= 16)		// above rev1.0 (EUR)
#endif
	{
		lcd_adc = s3c_adc_get_adc_data(SEC_LCD_ADC_CHANNEL);
		pr_info("lcd_adc : %d\n",lcd_adc);
		if(lcd_adc > ADC_LOWER_LIMIT_PLS_TYPE)
		{
			lcd_type = LCD_TYPE_PLS;
		}
		else if(lcd_adc > ADC_LOWER_LIMIT_T8_TYPE)
		{
			lcd_type = LCD_TYPE_T8;
		}
		else if(lcd_adc > ADC_LOWER_LIMIT_LCDPLS_TYPE)
		{
			lcd_type = LCD_TYPE_LCDPLS;
		}
		else if(lcd_adc > ADC_LOWER_LIMIT_VA50_TYPE)
		{
			lcd_type = LCD_TYPE_VA50;
		}
		else if(lcd_adc > ADC_LOWER_LIMIT_FFS_TYPE)
		{
			lcd_type = LCD_TYPE_FFS;
		}
		else if(lcd_adc > ADC_LOWER_LIMIT_TN_TYPE)
		{
			lcd_type = LCD_TYPE_TN;
		}
		else if(lcd_adc > ADC_LOWER_LIMIT_T7_TYPE)
		{
			lcd_type = LCD_TYPE_T7;
		}
		else
		{
			lcd_type = LCD_TYPE_VA;
		}		
	}
	else
	{
		lcd_type = LCD_TYPE_VA;
		lcd_adc = 0;
	}
//#endif
	//lcd_type = LCD_TYPE_VA;

	switch(lcd_type)
	{
	case LCD_TYPE_PLS:
		pr_info("LCD_TYPE_PLS\n");
		break;
	case LCD_TYPE_LCDPLS:
		pr_info("LCD_TYPE_LCDPLS\n");
		pr_notice("Tuning for this type LCD is not completed yet.\n");
		break;
	case LCD_TYPE_VA50:
		pr_info("LCD_TYPE_VA_50\n");
		break;
	case LCD_TYPE_TN:
		pr_info("LCD_TYPE_TN\n");
		pr_notice("Tuning for this type LCD is not completed yet.\n");
		break;
	case LCD_TYPE_FFS:
		pr_info("LCD_TYPE_FFS\n");
		break;
	case LCD_TYPE_T7:
		pr_info("LCD_TYPE_T7\n");
		pr_notice("Tuning for this type LCD is not completed yet.\n");
		break;
	case LCD_TYPE_T8:
		pr_info("LCD_TYPE_T8\n");
		pr_notice("Tuning for this type LCD is not completed yet.\n");
		break;
	case LCD_TYPE_VA:
	default:
		pr_info("LCD_TYPE_VA\n");
		break;
	}

	if(lcd_adc > ADC_LOWER_LIMIT_PLS_TYPE)
	{
		lcd_vendor_by_adc = LCD_TYPE_PLS;
	}
	else if(lcd_adc > ADC_LOWER_LIMIT_T8_TYPE)
	{
		lcd_vendor_by_adc = LCD_TYPE_T8;
	}
	else if(lcd_adc > ADC_LOWER_LIMIT_LCDPLS_TYPE)
	{
		lcd_vendor_by_adc = LCD_TYPE_LCDPLS;
	}
	else if(lcd_adc > ADC_LOWER_LIMIT_VA50_TYPE)
	{
		lcd_vendor_by_adc = LCD_TYPE_VA50;
	}
	else if(lcd_adc > ADC_LOWER_LIMIT_FFS_TYPE)
	{
		lcd_vendor_by_adc = LCD_TYPE_FFS;
	}
	else if(lcd_adc > ADC_LOWER_LIMIT_TN_TYPE)
	{
		lcd_vendor_by_adc = LCD_TYPE_TN;
	}
	else if(lcd_adc > ADC_LOWER_LIMIT_T7_TYPE)
	{
		lcd_vendor_by_adc = LCD_TYPE_T7;
	}
	else
	{
		lcd_vendor_by_adc = LCD_TYPE_VA;
	}		

	switch(lcd_vendor_by_adc)
	{
	case LCD_TYPE_PLS:
		pr_info("LCD vendor: HYDIS PLS\n");
		break;
	case LCD_TYPE_LCDPLS:
		pr_info("LCD vendor: LCD PLS\n");
		break;
	case LCD_TYPE_VA50:
		pr_info("LCD vendor: SMD VA50%%\n");
		break;
	case LCD_TYPE_TN:
		pr_info("LCD vendor: HANNSTAR TN\n");
		break;
	case LCD_TYPE_FFS:
		pr_info("LCD vendor: BOE FFS\n");
		break;
	case LCD_TYPE_VA:
		pr_info("LCD vendor: SMD VA70%%\n");
		break;
	case LCD_TYPE_T7:
		pr_info("LCD vendor: XXX T7\n");
		break;
	case LCD_TYPE_T8:
		pr_info("LCD vendor: XXX T8\n");
		break;
	default:
		pr_info("LCD vendor: none\n");
		break;
	}
	
	return ret;
}

#if CONFIG_PM 
int lms700_suspend(struct platform_device *pdev, pm_message_t state)
{
	pr_info("%s::%d->lms700 suspend\n",__func__,0);
	lms700_powerdown();

	return 0;
}

int lms700_resume(struct platform_device *pdev, pm_message_t state)
{
	pr_info("%s::%d ->lms700 resume\n",__func__,0);
	lms700_powerup();

	return 0;
}
#endif

static struct platform_driver lms700_driver = {
	.driver = {
		.name	= "lms700",
		.owner	= THIS_MODULE,
	},
	.probe		= lms700_probe,
	.remove		= __exit_p(lms700_remove),
	.shutdown	= lms700_shutdown,
//#ifdef CONFIG_PM
//	.suspend	= lms700_suspend,
//	.resume		= lms700_resume,
//#else
	.suspend	= NULL,
	.resume		= NULL,
//#endif
};

static int __init lms700_init(void)
{
	return  platform_driver_register(&lms700_driver);
}

static void __exit lms700_exit(void)
{
	platform_driver_unregister(&lms700_driver);
}


module_init(lms700_init);
module_exit(lms700_exit);


MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("LMS700 LCD driver");
MODULE_LICENSE("GPL");

