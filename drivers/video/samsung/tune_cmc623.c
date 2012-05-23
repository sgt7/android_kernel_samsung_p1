/*
 * cmc623_tune.c
 *
 * Parameter read & save driver on param partition.
 *
 * COPYRIGHT(C) Samsung Electronics Co.Ltd. 2006-2010 All Right Reserved.  
 *
 */
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/i2c.h>
#include "s3cfb.h"
#include "tune_cmc623.h"
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <mach/gpio-p1.h>


#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#if 0
#define dprintk(x...) printk(x)
#else
#define dprintk(x...) (0)
#endif

#if defined(CONFIG_TARGET_PCLK_44_46)
#define SETTING_PCLK_47M
#elif defined(CONFIG_TARGET_PCLK_47_6)
#define SETTING_PCLK_53M
#else
#define SETTING_PCLK_55M
#endif

//#define CMC623_TUNING

//#define MDNIE_TUNING_TUNEFILE_PATH	"/sdcard/sd/p1/mdnie_tune"
//#define CMC623_PATH_TUNING_DATA       "/sdcard/cmc623_tune"
#define CMC623_PATH_TUNING_DATA3  "/sdcard/external_sd/p1/mdnie_tune"
#define CMC623_PATH_TUNING_DATA2  "/sdcard/external_sd/p1/1"
#define CMC623_PATH_TUNING_DATA   "/sdcard/external_sd/p1/cmc623_tune"

#define CMC623_ADDR		     0x70  /* slave addr for i2c */
#define CMC623_DEVICE_ADDR   (0x38 << 1)
#define CMC623_MAX_SETTINGS	 100
#define CMC623_I2C_SPEED_KHZ 400

#define klogi(fmt, arg...)  printk(KERN_INFO "%s: " fmt "\n" , __func__, ## arg)
#define kloge(fmt, arg...)  printk(KERN_ERR "%s(%d): " fmt "\n" , __func__, __LINE__, ## arg)

#define END_SEQ		0xffff


#ifdef CMC623_TUNING
static Cmc623RegisterSet Cmc623_TuneSeq[CMC623_MAX_SETTINGS];
#endif

#define DELIMITER 0xff

static const u8 all_regs_bank0[] = {
	0xb4, 0xb3, 0x10, 0x24, 0x0b, 0x12, 0x13, 0x14, 0x15, 
	0x16, 0x17, 0x18, 0x19, 0x0f, 0x0d, 0x0e, 0x22, 0x23, 0x49, 0x4a, 
	0x4b, 0x4d, 0xc8, 0xc9, 0x42, 0x6e, 0x6f, 0x70, 0x71, 
	0x76, 0x77, 0x78, 0x79, 0x7a, 0x28, 0x09, 0x26,
	DELIMITER,
	0x01,
	0x2c, 0x2d, 0x2e, 0x2f, 0x3a, 0x3b, 0x3c, 0x3f, 0x42, 
	DELIMITER,
	0x72, 0x73, 0x74, 0x75, 0x7c, 
	DELIMITER,
	0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3,
	DELIMITER,
};

static const u8 all_regs_bank1[] = {
	0x09, 0x0a, 0x0b, 0x0c, 0x01, 0x06, 0x07, 0x65, 0x68, 0x6c,
	0x6d, 0x6e, 
	DELIMITER,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
	0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31,
	0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
	DELIMITER,

};

static struct i2c_client *g_client;
#define I2C_M_WR 0 /* for i2c */
#define I2c_M_RD 1 /* for i2c */

/* Each client has this additional data */
struct cmc623_data {
	struct i2c_client *client;
};

static struct s3cfb_lcd_timing 	s3cfb_lcd_timing_data = {0,};

static unsigned short stageClkA = 0;
static unsigned short stageClkB = 0;
static unsigned short refreshTime = 0;

static void set_cmc623_val_for_pclk(int pclk)
{
	if(pclk > 53000000)
		{
		stageClkA = 0x1a08;
		stageClkB = 0x0809;
		refreshTime = 0x0074;

		}
	else if(pclk > 47000000)
		{
		stageClkA = 0x1a09;
		stageClkB = 0x090a;
		refreshTime = 0x0067;
		}
	else	// 41Mhz
		{
		stageClkA = 0x1a0a;
		stageClkB = 0x0a0b;
		refreshTime = 0x0062;
		}
}
	
static struct cmc623_data * p_cmc623_data = NULL;
struct workqueue_struct *ove_wq=NULL;
struct work_struct work_ove;

struct cmc623_state_type{
	unsigned int cabc_enabled;
	unsigned int brightness;
	unsigned int suspended;
	int white;
	int black;
	int saturation;
	int power_lut_num;
};

static struct cmc623_state_type cmc623_state = { 
	.cabc_enabled = FALSE,
	.brightness = 32,
	.suspended = FALSE,
	.white = 0,
	.black = 0,
	.saturation = 0,
	.power_lut_num = 0,
};

typedef struct {
	u16 addr;
	u16 data;
} mDNIe_data_type;

typedef enum
{
	mDNIe_UI_MODE,
	mDNIe_VIDEO_MODE,
	mDNIe_VIDEO_WARM_MODE,
	mDNIe_VIDEO_COLD_MODE,
	mDNIe_CAMERA_MODE,
	mDNIe_NAVI,
	mDNIe_DMB_MODE,
	mDNIe_VT_MODE,
}Lcd_mDNIe_UI;

typedef enum
{
	mode_type_CABC_none,
	mode_type_CABC_on,
	mode_type_CABC_off,
}mDNIe_mode_CABC_type;

static Lcd_CMC623_UI_mode current_cmc623_UI = CMC623_UI_MODE; // mDNIe Set Status Checking Value.
static int current_cmc623_OutDoor_OnOff = FALSE;
static int current_cmc623_CABC_OnOff = FALSE;

static int setting_first = FALSE;
static int setting_boot = FALSE;
static int cmc623_bypass_mode = FALSE;
static int current_autobrightness_enable = FALSE;
static int cmc623_current_region_enable = FALSE;

static DEFINE_MUTEX(cmc623_state_transaction_lock);

mDNIe_mode_CABC_type cmc623_cabc_mode[]=
{
	mode_type_CABC_none,		// UI
	mode_type_CABC_on,		// Video
	mode_type_CABC_on,		// Video warm
	mode_type_CABC_on,		// Video cold
	mode_type_CABC_off, 	// Camera
	mode_type_CABC_none,		// Navi
};

#include "tune_cmc623_value.h"	

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

#define NUM_ITEM_POWER_LUT	9
#define NUM_POWER_LUT	2

static int current_power_lut_num = 0;

static unsigned char cmc623_Power_LUT[NUM_POWER_LUT][NUM_ITEM_POWER_LUT]={
	{ 0x46, 0x4b, 0x42, 0x50, 0x46, 0x43, 0x3e, 0x3b, 0x43 },
	{ 0x35, 0x3a, 0x31, 0x3f, 0x35, 0x32, 0x2d, 0x2a, 0x32 },//video
};

static bool cmc623_I2cWrite16(unsigned char Addr, unsigned long Data);
static void cmc623_cabc_pwm_brightness_reg(int value);
static void cmc623_manual_pwm_brightness_reg(int value);
static void cmc623_manual_pwm_brightness_reg_nosync(int value);

static unsigned long last_cmc623_Bank = 0xffff;
static unsigned long last_cmc623_Algorithm = 0xffff;

static void cmc623_reg_unmask(void)
{
	if(!p_cmc623_data)
	{
	printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
	return;
	}

	cmc623_I2cWrite16(0x28, 0x0000);
}

static void cmc623_Color_White_Change(int value, int finalize)
{
//	mDNIe_data_type *mode = cmc623_white_values[((value+2)*LCD_TYPE_MAX*2)+(2*lcd_type)+cmc623_state.cabc_enabled];
	mDNIe_data_type *mode = cmc623_white_val_values[(2*lcd_type)+cmc623_state.cabc_enabled];
	mode += (value+4);
	cmc623_state.white = value;

#ifdef CMC623_TUNING
	printk(KERN_ERR "%s ignore for tuning mode\n", __func__);
	return;
#endif

	cmc623_I2cWrite16(0x0000, 0x0000);		//bank
	cmc623_I2cWrite16(mode->addr, mode->data);
	
	if(finalize == TRUE)
	{
		cmc623_reg_unmask();
	}
}

static void cmc623_Color_Black_Change(int value, int finalize)
{
	mDNIe_data_type *mode = cmc623_black_values[((value+4)*LCD_TYPE_MAX*2)+(2*lcd_type)+cmc623_state.cabc_enabled];
	cmc623_state.black = value;

#ifdef CMC623_TUNING
	printk(KERN_ERR "%s ignore for tuning mode\n", __func__);
	return;
#endif

	while ( mode->addr != END_SEQ)
	{
		cmc623_I2cWrite16(mode->addr, mode->data);
		mode++;
	}
	
	if(finalize == TRUE)
	{
		cmc623_reg_unmask();
	}
}

static void cmc623_Color_Saturation_Change(int value, int finalize)
{
//	mDNIe_data_type *mode = cmc623_saturation_values[((value+2)*LCD_TYPE_MAX*2)+(2*lcd_type)+cmc623_state.cabc_enabled];
	mDNIe_data_type *mode = cmc623_saturation_val_values[(2*lcd_type)+cmc623_state.cabc_enabled];
	mode += (value+4);
	cmc623_state.saturation = value;

#ifdef CMC623_TUNING
	printk(KERN_ERR "%s ignore for tuning mode\n", __func__);
	return;
#endif

	cmc623_I2cWrite16(0x0000, 0x0000);		//bank
	cmc623_I2cWrite16(mode->addr, mode->data);
	
	if(finalize == TRUE)
	{
		cmc623_reg_unmask();
	}
}

static int cmc623_OutDoor_Enable(int enable);

static mDNIe_data_type* current_cmc623_mode = 0;
static void cmc623_Mode_Change_Compare(mDNIe_data_type *mode, int cabc_enable);
static void cmc623_Mode_Change(mDNIe_data_type *mode, int cabc_enable)
{
	int check;

//	if(mDNIe_Tuning_Mode == TRUE)
//	{
//		printk("mDNIe_Mode_Change [mDNIe_Tuning_Mode = TRUE, API is Return] \n");
//		return;
//	}
//	else
//	{
//		s3c_mdnie_mask();

#ifdef CMC623_TUNING
	printk(KERN_ERR "%s ignore for tuning mode\n", __func__);
	return;
#endif

	if(!p_cmc623_data)
		{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
		}

	if(cmc623_bypass_mode)
		{
		printk(KERN_WARNING "%s ignore for bypass mode\n", __func__);
		return;
		}
	
//	if(current_cmc623_mode != 0)
//		cmc623_Mode_Change_Compare(mode,cabc_enable);

	current_cmc623_mode = mode;
	while ( mode->addr != END_SEQ)
	{
		cmc623_I2cWrite16(mode->addr, mode->data);
		dprintk(KERN_INFO "[cmc623] a(0x%x),d(0x%x)\n",mode->addr, mode->data);	
		mode++;
	}
	
	// brightness setting 
	check = (setting_first || cabc_enable != cmc623_state.cabc_enabled);
	if(check || current_power_lut_num != cmc623_state.power_lut_num)
	{
		if(cabc_enable)
		{
			//CABC brightness setting
			cmc623_cabc_pwm_brightness_reg(cmc623_state.brightness);

			cmc623_state.cabc_enabled = TRUE;
		}
		else
		{
			//Manual brightness setting
			if(setting_first&&!setting_boot)
				cmc623_manual_pwm_brightness_reg_nosync(cmc623_state.brightness);
			else
				cmc623_manual_pwm_brightness_reg(cmc623_state.brightness);

			cmc623_state.cabc_enabled = FALSE;
		}
		cmc623_state.power_lut_num = current_power_lut_num;
	}
	if(check)
	{
		cmc623_Color_White_Change(cmc623_state.white,FALSE);
		cmc623_Color_Saturation_Change(cmc623_state.saturation,FALSE);
		cmc623_Color_Black_Change(cmc623_state.black,FALSE);	
	}
	
	if(!cmc623_OutDoor_Enable(current_cmc623_OutDoor_OnOff))
	{
		cmc623_reg_unmask();
	}
}

static void cmc623_Mode_Change_Compare(mDNIe_data_type *mode, int cabc_enable)
{
	int c = 0;
#ifdef CMC623_TUNING
	printk(KERN_ERR "%s ignore for tuning mode\n", __func__);
	return;
#endif

	if(!p_cmc623_data)
		{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
		}

	if(cmc623_bypass_mode)
		{
		printk(KERN_WARNING "%s ignore for bypass mode\n", __func__);
		return;
		}

	if(current_cmc623_mode == 0)
	{
		current_cmc623_mode = mode;
		while ( mode->addr != END_SEQ)
		{
			cmc623_I2cWrite16(mode->addr, mode->data);
			dprintk(KERN_INFO "[cmc623] a(0x%x),d(0x%x)\n",mode->addr, mode->data);	
			mode++;
		}
	}
	else
	{
		mDNIe_data_type *cmode = current_cmc623_mode;
		current_cmc623_mode = mode;
		while ( mode->addr != END_SEQ)
		{
			//printk("[cmc62345678] a(0x%x),a(0x%x)\n",mode->addr, cmode->addr);
			if(mode->addr == 0x0000)
			{
				//cmc623_I2cWrite16(mode->addr, mode->data);
				dprintk(KERN_INFO "[cmc623] a(0x%x),d(0x%x)\n",mode->addr, mode->data);	
				//printk("[cmc6233333] a(0x%x),d(0x%x)\n",mode->addr, mode->data);	
				mode++;
				
				if(cmode->addr == 0x0000)
				{
					cmode++;
				}
				else
				{
					while(cmode->addr != 0x0000)
					{
						cmode++;
					}
					cmode++;
				}
			}
			else if(mode->addr == cmode->addr)
			{
				//printk("mode->addr == cmode->addr %x %x\n",mode->data,cmode->data);
				if(mode->data != cmode->data)
				{
					//cmc623_I2cWrite16(mode->addr, mode->data);
					dprintk(KERN_INFO "[cmc623] a(0x%x),d(0x%x)\n",mode->addr, mode->data);	
					printk("[cmc6233333] a(0x%x),d(0x%x)\n",mode->addr, mode->data);	
				}
				
				mode++;
				cmode++;
			}
			else
			{
				if(mode->addr < cmode->addr)
				{
					//cmc623_I2cWrite16(mode->addr, mode->data);
					dprintk(KERN_INFO "[cmc623] a(0x%x),d(0x%x)\n",mode->addr, mode->data);
					printk("[cmc6233333] a(0x%x),d(0x%x)\n",mode->addr, mode->data);	
					
					mode++;
				}
				else
				{
					cmode++;
				}
			}
			
			c++;
			if(c>60)
				break;
		}
	}
	
	// brightness setting
	if(setting_first || cabc_enable != cmc623_state.cabc_enabled)
	{
		if(cabc_enable)
		{
			//CABC brightness setting
			cmc623_cabc_pwm_brightness_reg(cmc623_state.brightness);

			cmc623_state.cabc_enabled = TRUE;
		}
		else
		{
			//Manual brightness setting
			if(setting_first)
				cmc623_manual_pwm_brightness_reg_nosync(cmc623_state.brightness);
			else
				cmc623_manual_pwm_brightness_reg(cmc623_state.brightness);

			cmc623_state.cabc_enabled = FALSE;
		}
		cmc623_Color_White_Change(cmc623_state.white,FALSE);
		cmc623_Color_Saturation_Change(cmc623_state.saturation,FALSE);
		cmc623_Color_Black_Change(cmc623_state.black,FALSE);	
		}
	
	if(!cmc623_OutDoor_Enable(current_cmc623_OutDoor_OnOff))
	{
		cmc623_reg_unmask();
	}
}

static void cmc623_Set_Mode(Lcd_CMC623_UI_mode mode, int cmc623_CABC_OnOff)
{
	int cabc_enable=0;
	current_cmc623_UI = mode;
	
	if(cmc623_CABC_OnOff)
	{
		cabc_enable = 1;
		
		switch(mode)
		{
			case CMC623_UI_MODE:
				current_power_lut_num = 0;
				cmc623_Mode_Change(cmc623_values[CMC_UI_CABC*LCD_TYPE_MAX+lcd_type], TRUE);
			break;

			case CMC623_VIDEO_MODE:
				current_power_lut_num = 1;
				cmc623_Mode_Change(cmc623_values[CMC_Video_CABC*LCD_TYPE_MAX+lcd_type], TRUE);
			break;

			case CMC623_VIDEO_WARM_MODE:
				current_power_lut_num = 1;
				cmc623_Mode_Change(cmc623_values[CMC_Video_CABC*LCD_TYPE_MAX+lcd_type], TRUE);
			break;

			case CMC623_VIDEO_COLD_MODE:
				current_power_lut_num = 1;
				cmc623_Mode_Change(cmc623_values[CMC_Video_CABC*LCD_TYPE_MAX+lcd_type], TRUE);
			break;
			
			case CMC623_CAMERA_MODE:
				current_power_lut_num = 0;
				cabc_enable = 0;
				cmc623_Mode_Change(cmc623_values[CMC_Camera*LCD_TYPE_MAX+lcd_type], FALSE);
			break;

			case CMC623_NAVI:
				current_power_lut_num = 0;
				cmc623_Mode_Change(cmc623_values[CMC_UI_CABC*LCD_TYPE_MAX+lcd_type], TRUE);
			break;

			case CMC623_DMB_MODE:
				current_power_lut_num = 0;
				cmc623_Mode_Change(cmc623_values[CMC_DMB_CABC*LCD_TYPE_MAX+lcd_type], TRUE);
			break;

			case CMC623_VT_MODE:
				current_power_lut_num = 0;
				cmc623_Mode_Change(cmc623_values[CMC_VT_CABC*LCD_TYPE_MAX+lcd_type], TRUE);
			break;

			case CMC623_GALLERY_MODE:
				current_power_lut_num = 0;
				cmc623_Mode_Change(cmc623_values[CMC_GALLERY_CABC*LCD_TYPE_MAX+lcd_type], TRUE);
			break;
		}

		//current_cmc623_UI = mode;
		current_cmc623_CABC_OnOff = TRUE;
	}
	else
	{
		cabc_enable = 0;
		
		switch(mode)
		{
			case CMC623_UI_MODE:
				current_power_lut_num = 0;
				cmc623_Mode_Change(cmc623_values[CMC_UI*LCD_TYPE_MAX+lcd_type], FALSE);
			break;

			case CMC623_VIDEO_MODE:
				current_power_lut_num = 1;
				cabc_enable = 0;
				cmc623_Mode_Change(cmc623_values[CMC_Video*LCD_TYPE_MAX+lcd_type], FALSE);
			break;

			case CMC623_VIDEO_WARM_MODE:
				current_power_lut_num = 1;
				cabc_enable = 0;
				cmc623_Mode_Change(cmc623_values[CMC_Video_CABC*LCD_TYPE_MAX+lcd_type], FALSE);
			break;

			case CMC623_VIDEO_COLD_MODE:
				current_power_lut_num = 1;
				cabc_enable = 0;
				cmc623_Mode_Change(cmc623_values[CMC_Video_CABC*LCD_TYPE_MAX+lcd_type], FALSE);
			break;
			
			case CMC623_CAMERA_MODE:
				current_power_lut_num = 0;
				cmc623_Mode_Change(cmc623_values[CMC_Camera*LCD_TYPE_MAX+lcd_type], FALSE);
			break;

			case CMC623_NAVI:
				current_power_lut_num = 0;
				cmc623_Mode_Change(cmc623_values[CMC_UI*LCD_TYPE_MAX+lcd_type], FALSE);
			break;

			case CMC623_DMB_MODE:
				current_power_lut_num = 0;
				cmc623_Mode_Change(cmc623_values[CMC_DMB*LCD_TYPE_MAX+lcd_type], FALSE);
			break;

			case CMC623_VT_MODE:
				current_power_lut_num = 0;
				cmc623_Mode_Change(cmc623_values[CMC_VT*LCD_TYPE_MAX+lcd_type], FALSE);
			break;

			case CMC623_GALLERY_MODE:
				current_power_lut_num = 0;
				cmc623_Mode_Change(cmc623_values[CMC_GALLERY*LCD_TYPE_MAX+lcd_type], FALSE);
			break;
		}
		
		//current_cmc623_UI = mode;
		current_cmc623_CABC_OnOff = FALSE;
	}	
	printk("[cmc623] cmc623_Set_Mode: current_cmc623_UI(%d), current_cmc623_CABC_OnOff(%d)->(%d) / OVE : %d \n",current_cmc623_UI, current_cmc623_CABC_OnOff, cabc_enable,current_cmc623_OutDoor_OnOff);	
}
//EXPORT_SYMBOL(cmc623_Set_Mode);

void cmc623_Set_Mode_Ext(Lcd_CMC623_UI_mode mode, u8 mDNIe_Outdoor_OnOff)
{
	mutex_lock(&cmc623_state_transaction_lock);
	if(mDNIe_Outdoor_OnOff)
	{
		cmc623_Set_Mode(mode, current_cmc623_CABC_OnOff);
		//current_cmc623_OutDoor_OnOff = TRUE;
	}
	else
	{
		cmc623_Set_Mode(mode, current_cmc623_CABC_OnOff);
		//current_cmc623_OutDoor_OnOff = FALSE;
	}
	mutex_unlock(&cmc623_state_transaction_lock);
	
	dprintk("[cmc623] cmc623_Set_Mode_Ext: current_cmc623_UI(%d), current_cmc623_OutDoor_OnOff(%d)  \n",current_cmc623_UI, current_cmc623_OutDoor_OnOff);	
}
EXPORT_SYMBOL(cmc623_Set_Mode_Ext);

#if 0
static int cmc623_I2cWrite(struct i2c_client *client, u8 reg, u8 *data, u8 length)
{
	int ret, i;
	u8 buf[length+1];
	struct i2c_msg msg[1];

	buf[0] = reg;
	for(i=0; i<length; i++)
    {   
		buf[i+1] = *(data++);
    }    

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = length+1;
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret != 1) 
    {   
        return -EIO;
    }

	return 0;
}
#endif

static bool cmc623_I2cWrite16( unsigned char Addr, unsigned long Data)
{
    int err;
    struct i2c_msg msg[1];
    unsigned char data[3];

	if(!p_cmc623_data)
		{
	    printk(KERN_ERR "p_cmc623_data is NULL\n");
        return -ENODEV;
		}
	g_client = p_cmc623_data->client;		

    if( (g_client == NULL))  
    {
        printk("cmc623_I2cWrite16 g_client is NULL\n");
        return -ENODEV;
    }

    if (!g_client->adapter) 
    {
        printk("cmc623_I2cWrite16 g_client->adapter is NULL\n");
        return -ENODEV;
    }

	if(TRUE == cmc623_state.suspended)
		{
        printk("cmc623 don't need writing while LCD off(a:%x,d:%x)\n", Addr, Data);
        return 0;
		}

	if(Addr == 0x0000)
	{
		if(Data == last_cmc623_Bank)
		{
			return 0;
		}
		last_cmc623_Bank = Data;
	}
	else if(Addr == 0x0001)
	{
		last_cmc623_Algorithm = Data;
	}
    
    data[0] = Addr;
    data[1] = ((Data >>8)&0xFF);
    data[2] = (Data)&0xFF;
    msg->addr = g_client->addr;
    msg->flags = I2C_M_WR;
    msg->len = 3;
    msg->buf = data;
    
    err = i2c_transfer(g_client->adapter, msg, 1);

    if (err >= 0) 
    {
        //printk("%s %d i2c transfer OK\n", __func__, __LINE__);/* add by inter.park */
        return 0;
    }

    printk("%s i2c transfer error:%d(a:%x)\n", __func__, err, Addr);/* add by inter.park */
    return err;    
}

static int cmc623_I2cRead16(u8 reg, u16 *val)
{
    int      err;
    struct   i2c_msg msg[2];
	u8 regaddr = reg;
	u8 data[2];

	if(!p_cmc623_data)
		{
	    printk(KERN_ERR "%s p_cmc623_data is NULL\n", __func__);
        return -ENODEV;
		}
	g_client = p_cmc623_data->client;		

    if( (g_client == NULL))  
    {
        printk("%s g_client is NULL\n", __func__);
        return -ENODEV;
    }

    if (!g_client->adapter) 
    {
        printk("%s g_client->adapter is NULL\n", __func__);
        return -ENODEV;
    }
	
	if(regaddr == 0x0001)
	{
		*val = last_cmc623_Algorithm;
		return 0;
	}
    
    msg[0].addr   = g_client->addr;
    msg[0].flags  = I2C_M_WR;
    msg[0].len    = 1;
    msg[0].buf    = &regaddr;
    msg[1].addr   = g_client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len   = 2;
    msg[1].buf   = &data[0];
    err = i2c_transfer(g_client->adapter, &msg[0], 2);

    if (err >= 0) 
    {
    	*val = (data[0]<<8) | data[1];
        return 0;
    }
    printk(KERN_ERR "%s %d i2c transfer error: %d\n", __func__, __LINE__, err);/* add by inter.park */

    return err;
}

void cmc623_Set_Region(int enable, int hStart, int hEnd, int vStart, int vEnd)
{
	u16 data=0;

	mutex_lock(&cmc623_state_transaction_lock);
	cmc623_I2cRead16(0x0001, &data);
	data &= 0x00ff;

	cmc623_I2cWrite16(0x0000,0x0000);
	if(enable)
	{
		cmc623_I2cWrite16(0x0001,0x0300 | data);
	
		cmc623_I2cWrite16(0x0002,hStart);
		cmc623_I2cWrite16(0x0003,hEnd);
		cmc623_I2cWrite16(0x0004,vStart);
		cmc623_I2cWrite16(0x0005,vEnd);
	}
	else
	{
		cmc623_I2cWrite16(0x0001,0x0000 | data);
	}
	cmc623_I2cWrite16(0x0028,0x0000);
	mutex_unlock(&cmc623_state_transaction_lock);
	
	cmc623_current_region_enable = enable;
}
EXPORT_SYMBOL(cmc623_Set_Region);

void cmc623_autobrightness_enable(int enable)
{
	mutex_lock(&cmc623_state_transaction_lock);
	if(current_cmc623_OutDoor_OnOff == TRUE && enable == FALSE)
	{//outdoor mode off
		current_cmc623_OutDoor_OnOff = FALSE;
		cmc623_Set_Mode(current_cmc623_UI, current_cmc623_CABC_OnOff);
	}

	current_autobrightness_enable = enable;
	mutex_unlock(&cmc623_state_transaction_lock);
}
EXPORT_SYMBOL(cmc623_autobrightness_enable);

static void cmc623_cabc_enable_flag(int enable, int flag_first, int flag_boot)
{
	if(!p_cmc623_data)
	{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
	}

	printk(KERN_INFO "%s(%d)\n", __func__, enable);

	setting_first = flag_first;
	setting_boot = flag_boot;
	
	cmc623_Set_Mode(current_cmc623_UI, enable);
	
	setting_first = FALSE;
	setting_boot = FALSE;
}

void cmc623_cabc_enable(int enable)
{
	if(!p_cmc623_data)
	{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
	}

	printk(KERN_INFO "%s(%d)\n", __func__, enable);

	mutex_lock(&cmc623_state_transaction_lock);
	cmc623_Set_Mode(current_cmc623_UI, enable);
	mutex_unlock(&cmc623_state_transaction_lock);
}
EXPORT_SYMBOL(cmc623_cabc_enable);

#ifdef CMC623_TUNING

//static bool cmc623_tune(unsigned long num)
bool cmc623_tune(unsigned long num) // P1_LSJ : DE08
{
	unsigned int i;
    
	//  num = NV_ARRAY_SIZE(Cmc623_TuneSeq);
	// printk("========== Start of tuning CMC623 Jun  ==========\n");
        
	for (i=0; i<num; i++) 
    {
		// printk("[%2d] Writing => reg: 0x%2x, data: 0x%4lx\n", i+1, Cmc623_TuneSeq[i].RegAddr, Cmc623_TuneSeq[i].Data);

		if (0 > cmc623_I2cWrite16(Cmc623_TuneSeq[i].RegAddr, Cmc623_TuneSeq[i].Data)) 

        // cmc623_I2cWrite(struct i2c_client *client, u8 reg, u8 *data, u8 length)
        //if (!cmc623_I2cWrite16(Cmc623_TuneSeq[i].RegAddr, Cmc623_TuneSeq[i].Data)) 
        {
			//printk("I2cWrite16 failed\n");
			return 0;
		}
        else 
        {
			//printk("I2cWrite16 succeed\n");
        }

        if ( Cmc623_TuneSeq[i].RegAddr == CMC623_REG_SWRESET && Cmc623_TuneSeq[i].Data == 0xffff ) 
        {
			// NvOdmOsWaitUS(3000);  // 3ms  
			// msleep(3);
            mdelay(3);
		}
	}
	// printk("==========  End of tuning CMC623 Jun  ==========\n");
	return 1;
}

//static int parse_text(char * src, int len, unsigned short * output)
static int parse_text(char * src, int len)
{
	int i,count, ret;
	int index=0;
	char * str_line[CMC623_MAX_SETTINGS];
	char * sstart;
	char * c;
	unsigned int data1, data2;

	c = src;
	count = 0;
	sstart = c;
    
	for(i=0; i<len; i++,c++) 
    {
		char a = *c;
		if(a=='\r' || a=='\n') 
        {
			if(c > sstart) 
            {
				str_line[count] = sstart;
				count++;
			}
			*c='\0';
			sstart = c+1;
		}
	}
    
	if(c > sstart) 
    {
		str_line[count] = sstart;
		count++;
	}

	// printk("----------------------------- Total number of lines:%d\n", count);

	for(i=0; i<count; i++) 
    {
		// printk("line:%d, [start]%s[end]\n", i, str_line[i]);
		ret = sscanf(str_line[i], "0x%x,0x%x\n", &data1, &data2);
		// printk("Result => [0x%2x 0x%4x] %s\n", data1, data2, (ret==2)?"Ok":"Not available");
		if(ret == 2) 
        {   
			Cmc623_TuneSeq[index].RegAddr = (unsigned char)data1;
			Cmc623_TuneSeq[index++].Data  = (unsigned long)data2;
		}
	}
	return index;
}

static int cmc623_load_data(void)
{
	struct file *filp;
	char	*dp;
	long	l, i ;
	loff_t  pos;
	int     ret, num;
	mm_segment_t fs;

	klogi("cmc623_load_data start!");

	fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(CMC623_PATH_TUNING_DATA, O_RDONLY, 0);
	if(IS_ERR(filp)) 
    {
		kloge("file open error:%d", (s32)filp);

		return -1;

		/*		
        filp = filp_open(CMC623_PATH_TUNING_DATA2, O_RDONLY, 0);
        if(IS_ERR(filp)) 
        {
            kloge("file open error2");

            filp = filp_open(CMC623_PATH_TUNING_DATA3, O_RDONLY, 0);
            if(IS_ERR(filp)) 
            {
                kloge("file open error3");
                return -1;
            }
        }
        */
	}

	l = filp->f_path.dentry->d_inode->i_size;
	klogi("Size of the file : %ld(bytes)", l);

	//dp = kmalloc(l, GFP_KERNEL);
	dp = kmalloc(l+10, GFP_KERNEL);		// add cushion
	if(dp == NULL) 
    {
		kloge("Out of Memory!");
		filp_close(filp, current->files);
		return -1;
	}
	pos = 0;
	memset(dp, 0, l);
    kloge("== Before vfs_read ======");
	ret = vfs_read(filp, (char __user *)dp, l, &pos);   // P1_LSJ : DE08 : ¿©±â¼­ Á×À½ 
    kloge("== After vfs_read ======");

	if(ret != l) 
    {
		kloge("<CMC623> Failed to read file (ret = %d)", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return -1;
	}

	filp_close(filp, current->files);

	set_fs(fs);

	for(i=0; i<l; i++)
    {   
		// printk("%x ", dp[i]);
    }
	// printk("\n");

	num = parse_text(dp, l);

	if(!num) 
    {
		kloge("Nothing to parse!");
		kfree(dp);
		return -1;
	}
		
	// printk("------ Jun Total number of parsed lines: %d\n", num);
	cmc623_tune(num);

	kfree(dp);
	return num;
}

int CMC623_tuning_load_from_file(void)
{
	return cmc623_load_data();
}
EXPORT_SYMBOL(CMC623_tuning_load_from_file);

#endif	//CMC623_TUNING


static int cmc623_tuning_set (void)  // P1_LSJ DE19
{
    int ret = 0;

    // printk("**************************************\n");
    // printk("**** < cmc623_tuning_set >       *****\n");
    // printk("**************************************\n");

	cmc623_I2cWrite16(0x00, 0x0000);    //BANK 0
	cmc623_I2cWrite16(0x01, 0x0070);    //algorithm selection
	cmc623_I2cWrite16(0xb4, 0x41C0);    //PWM ratio
	cmc623_I2cWrite16(0xb3, 0xffff);    //up/down step
	cmc623_I2cWrite16(0x10, 0x001A);    // PCLK Polarity Sel
	cmc623_I2cWrite16(0x24, 0x0001);    // Polarity Sel    
	cmc623_I2cWrite16(0x0b, 0x0184);    // Clock Gating
	cmc623_I2cWrite16(0x12, 0x0000);    // Pad strength start
	cmc623_I2cWrite16(0x13, 0x0000);
	cmc623_I2cWrite16(0x14, 0x0000);
	cmc623_I2cWrite16(0x15, 0x0000);
	cmc623_I2cWrite16(0x16, 0x0000);
	cmc623_I2cWrite16(0x17, 0x0000);
	cmc623_I2cWrite16(0x18, 0x0000);
	cmc623_I2cWrite16(0x19, 0x0000);     // Pad strength end
	cmc623_I2cWrite16(0x0f, 0x0010);     // PWM clock ratio
	cmc623_I2cWrite16(0x0d, 0x1a0a);     // A-Stage clk
	cmc623_I2cWrite16(0x0e, 0x0a0b);     // B-stage clk    
	cmc623_I2cWrite16(0x22, 0x0400);     // H_Size
	cmc623_I2cWrite16(0x23, 0x0258);     // V_Size
	cmc623_I2cWrite16(0x2c, 0x0fff);    //DNR bypass
	cmc623_I2cWrite16(0x2e, 0x0000);    //DNR bypass
	cmc623_I2cWrite16(0x3a, 0x000c);    //HDTR on DE, 
	cmc623_I2cWrite16(0x3b, 0x03ff);    //DE strength
	cmc623_I2cWrite16(0x42, 0x0001);    //max_Diff
	cmc623_I2cWrite16(0x49, 0x0400);    //PCC
	cmc623_I2cWrite16(0x4a, 0x7272);
	cmc623_I2cWrite16(0x4b, 0x8d8d);
	cmc623_I2cWrite16(0x4d, 0x01c0);
	cmc623_I2cWrite16(0xC8, 0x0000);    //scr
	cmc623_I2cWrite16(0xC9, 0x1000);
	cmc623_I2cWrite16(0xCA, 0xFFFF);
	cmc623_I2cWrite16(0xCB, 0xFFFF);
	cmc623_I2cWrite16(0xCC, 0x0000);
	cmc623_I2cWrite16(0xCD, 0xFFFF);
	cmc623_I2cWrite16(0xCE, 0x1000);
	cmc623_I2cWrite16(0xCF, 0xF0F0);
	cmc623_I2cWrite16(0xD0, 0x00FF);
	cmc623_I2cWrite16(0xD1, 0x00FF);
	cmc623_I2cWrite16(0xD2, 0x00FF);
	cmc623_I2cWrite16(0xD3, 0x00FF);
	cmc623_I2cWrite16(0x00, 0x0001);    //BANK 1
	cmc623_I2cWrite16(0x09, 0x0400);    // H_Size
	cmc623_I2cWrite16(0x0a, 0x0258);    // V_Size
	cmc623_I2cWrite16(0x0b, 0x0400);    // H_Size
	cmc623_I2cWrite16(0x0c, 0x0258);    // V_Size
	cmc623_I2cWrite16(0x01, 0x0500);    // BF_Line
	cmc623_I2cWrite16(0x06, 0x0062);    // Refresh time
	cmc623_I2cWrite16(0x07, 0x2225);    // eDRAM
	cmc623_I2cWrite16(0x65, 0x0008);    // V_Sync cal.
	cmc623_I2cWrite16(0x68, 0x0080);    // TCON Polarity
	cmc623_I2cWrite16(0x6c, 0x0414);    // VLW,HLW
	cmc623_I2cWrite16(0x6d, 0x0506);    // VBP,VFP
	cmc623_I2cWrite16(0x6e, 0x1e32);    // HBP,HFP
	cmc623_I2cWrite16(0x20, 0x0000);		//GAMMA	11
	cmc623_I2cWrite16(0x21, 0x2000);
	cmc623_I2cWrite16(0x22, 0x2000);
	cmc623_I2cWrite16(0x23, 0x2000);
	cmc623_I2cWrite16(0x24, 0x2000);
	cmc623_I2cWrite16(0x25, 0x2000);
	cmc623_I2cWrite16(0x26, 0x2000);
	cmc623_I2cWrite16(0x27, 0x2000);
	cmc623_I2cWrite16(0x28, 0x2000);
	cmc623_I2cWrite16(0x29, 0x1f01);
	cmc623_I2cWrite16(0x2A, 0x1f01);
	cmc623_I2cWrite16(0x2B, 0x1f01);
	cmc623_I2cWrite16(0x2C, 0x1f01);
	cmc623_I2cWrite16(0x2D, 0x1f01);
	cmc623_I2cWrite16(0x2E, 0x1f01);
	cmc623_I2cWrite16(0x2F, 0x1f01);
	cmc623_I2cWrite16(0x30, 0x1f01);
	cmc623_I2cWrite16(0x31, 0x1f01);
	cmc623_I2cWrite16(0x32, 0x1f01);
	cmc623_I2cWrite16(0x33, 0x1f01);
	cmc623_I2cWrite16(0x34, 0xa20b);
	cmc623_I2cWrite16(0x35, 0xa20b);
	cmc623_I2cWrite16(0x36, 0x2000);
	cmc623_I2cWrite16(0x37, 0x1f08);
	cmc623_I2cWrite16(0x38, 0xFF00); 
	//cmc623_I2cWrite16(0x20, 0x0001);    // GAMMA 11
	cmc623_I2cWrite16(0x20, 0x0000);    // GAMMA disable
	cmc623_I2cWrite16(0x00, 0x0000);
	cmc623_I2cWrite16(0x28, 0x0000);
	cmc623_I2cWrite16(0x09, 0x0000);
	cmc623_I2cWrite16(0x09, 0xffff);

	//bypass
	cmc623_I2cWrite16(0x00, 0x0000);    //BANK 0
	cmc623_I2cWrite16(0x01, 0x0020);    //algorithm selection
	cmc623_I2cWrite16(0x3a, 0x0000);    
	cmc623_I2cWrite16(0x28, 0x0000);



	//delay 5ms
	mdelay(80);	//msleep(5);

	cmc623_I2cWrite16(0x26, 0x0001);

    // printk("**** < end cmc623_tuning_set >       *****\n");

	return ret;    
}

static int cmc623_initial_set (void)  // P1_LSJ DE19
{
    int ret = 0;
	uint16_t vlw_hlw, vbp_vfp, hbp_hfp;

    // printk("**************************************\n");
    // printk("**** < cmc623_initial_set >       *****\n");
    // printk("**************************************\n");

	if(s3cfb_lcd_timing_data.h_fp == 0)
		{
	    printk("####### ERROR: s3cfb_lcd_timing_data is invalid #######\n");
		}

	vlw_hlw = ((s3cfb_lcd_timing_data.v_sw&0xff)<<8)|(s3cfb_lcd_timing_data.h_sw&0xff);
	vbp_vfp = ((s3cfb_lcd_timing_data.v_bp&0xff)<<8)|(s3cfb_lcd_timing_data.v_fp&0xff);
	hbp_hfp = ((s3cfb_lcd_timing_data.h_bp&0xff)<<8)|(s3cfb_lcd_timing_data.h_fp&0xff);

	cmc623_I2cWrite16(0x00, 0x0000);    //BANK 0
	cmc623_I2cWrite16(0x01, 0x0020);    //algorithm selection
	cmc623_I2cWrite16(0xb4, 0xC000);    //PWM ratio
	cmc623_I2cWrite16(0xb3, 0xffff);    //up/down step
	cmc623_I2cWrite16(0x10, 0x001A);    // PCLK Polarity Sel
	cmc623_I2cWrite16(0x24, 0x0001);    // Polarity Sel    
	cmc623_I2cWrite16(0x0b, 0x0184);    // Clock Gating
//	cmc623_I2cWrite16(0x12, 0x0000);    // Pad strength start
//	cmc623_I2cWrite16(0x13, 0x0000);
//	cmc623_I2cWrite16(0x14, 0x0000);
//	cmc623_I2cWrite16(0x15, 0x0000);
//	cmc623_I2cWrite16(0x16, 0x0000);
//	cmc623_I2cWrite16(0x17, 0x0000);
//	cmc623_I2cWrite16(0x18, 0x0000);
//	cmc623_I2cWrite16(0x19, 0x0000);     // Pad strength end
	cmc623_I2cWrite16(0x0f, 0x0010);     // PWM clock ratio
	cmc623_I2cWrite16(0x0d, stageClkA);     // A-Stage clk
	cmc623_I2cWrite16(0x0e, stageClkB);     // B-stage clk
	cmc623_I2cWrite16(0x22, 0x0400);     // H_Size
	cmc623_I2cWrite16(0x23, 0x0258);     // V_Size
	cmc623_I2cWrite16(0x2c, 0x0fff);	//DNR bypass
	cmc623_I2cWrite16(0x2d, 0x1900);	//DNR bypass
	cmc623_I2cWrite16(0x2e, 0x0000);	//DNR bypass
	cmc623_I2cWrite16(0x2f, 0x00ff);	//DNR bypass
	cmc623_I2cWrite16(0x3a, 0x0000);    //HDTR on DE, 
	cmc623_I2cWrite16(0x00, 0x0001);    //BANK 1
	cmc623_I2cWrite16(0x09, 0x0400);    // H_Size
	cmc623_I2cWrite16(0x0a, 0x0258);    // V_Size
	cmc623_I2cWrite16(0x0b, 0x0400);    // H_Size
	cmc623_I2cWrite16(0x0c, 0x0258);    // V_Size
	cmc623_I2cWrite16(0x01, 0x0500);    // BF_Line
	cmc623_I2cWrite16(0x06, refreshTime);    // Refresh time
	cmc623_I2cWrite16(0x07, 0x2225);    // eDRAM
//	cmc623_I2cWrite16(0x65, 0x0008);    // V_Sync cal.
	cmc623_I2cWrite16(0x68, 0x0080);    // TCON Polarity
	cmc623_I2cWrite16(0x6c, vlw_hlw);    // VLW,HLW
	cmc623_I2cWrite16(0x6d, vbp_vfp);    // VBP,VFP
	cmc623_I2cWrite16(0x6e, hbp_hfp);    // HBP,HFP
//	cmc623_I2cWrite16(0x20, 0x0000);	//GAMMA	11
//	cmc623_I2cWrite16(0x21, 0x2000);
//	cmc623_I2cWrite16(0x22, 0x2000);
//	cmc623_I2cWrite16(0x23, 0x2000);
//	cmc623_I2cWrite16(0x24, 0x2000);
//	cmc623_I2cWrite16(0x25, 0x2000);
//	cmc623_I2cWrite16(0x26, 0x2000);
//	cmc623_I2cWrite16(0x27, 0x2000);
//	cmc623_I2cWrite16(0x28, 0x2000);
//	cmc623_I2cWrite16(0x29, 0x2000);
//	cmc623_I2cWrite16(0x2A, 0x2000);
//	cmc623_I2cWrite16(0x2B, 0x2000);
//	cmc623_I2cWrite16(0x2C, 0x2000);
//	cmc623_I2cWrite16(0x2D, 0x2000);
//	cmc623_I2cWrite16(0x2E, 0x2000);
//	cmc623_I2cWrite16(0x2F, 0x2000);
//	cmc623_I2cWrite16(0x30, 0x2000);
//	cmc623_I2cWrite16(0x31, 0x2000);
//	cmc623_I2cWrite16(0x32, 0x2000);
//	cmc623_I2cWrite16(0x33, 0x2000);
//	cmc623_I2cWrite16(0x34, 0x2000);
//	cmc623_I2cWrite16(0x35, 0x2000);
//	cmc623_I2cWrite16(0x36, 0x2000);
//	cmc623_I2cWrite16(0x37, 0x2000);
//	cmc623_I2cWrite16(0x38, 0xFF00); 
//	cmc623_I2cWrite16(0x20, 0x0001);    // GAMMA 11
	cmc623_I2cWrite16(0x00, 0x0000);

	cmc623_I2cWrite16(0x28, 0x0000);
	cmc623_I2cWrite16(0x09, 0x0000);
	cmc623_I2cWrite16(0x09, 0xffff);

	//delay 5ms
	msleep(5);

	cmc623_I2cWrite16(0x26, 0x0001);

    // printk("**** < end cmc623_initial_set >       *****\n");

	return ret;    
}

static void cmc623_set_tuning (void)
{

    // printk("**************************************\n");
    // printk("**** < cmc623_set_tuning >       *****\n");
    // printk("**************************************\n");

#if 0
	//VA tuning
	//start
	cmc623_I2cWrite16(0x0000,0x0000);	//BANK 0
	cmc623_I2cWrite16(0x0001,0x0060);	//SCR LABC
	cmc623_I2cWrite16(0x002c,0x0fff);	//DNR bypass	bypass,dir_th   0x003C
	cmc623_I2cWrite16(0x002d,0x1900);	//DNR bypass	dir_num,decont7 0x0a08
	cmc623_I2cWrite16(0x002e,0x0000);	//DNR bypass	decont5,mask_th 0x1010
	cmc623_I2cWrite16(0x002f,0x00ff);	//DNR bypass	block_th        0x0400
	cmc623_I2cWrite16(0x003A,0x000D);	//HDTR DE CS
	cmc623_I2cWrite16(0x003B,0x03ff);	//DE SHARPNESS
	cmc623_I2cWrite16(0x003C,0x0000);	//NOISE LEVEL
	cmc623_I2cWrite16(0x003F,0x0070);	//CS GAIN
	cmc623_I2cWrite16(0x0042,0x0000);	//DE TH (MAX DIFF) 3F
	//cmc623_I2cWrite16(0x00b4,0x4640);	//Count PWM
	cmc623_I2cWrite16(0x00c8,0x0030);	//kb	0x0000	SCR
	cmc623_I2cWrite16(0x00c9,0x0000);	//gc	0x0000
	cmc623_I2cWrite16(0x00ca,0xffff);	//rm	0xffff
	cmc623_I2cWrite16(0x00cb,0xffff);	//yw	0xffff
	cmc623_I2cWrite16(0x00cc,0x0000);	//kb	0x0000
	cmc623_I2cWrite16(0x00cd,0xffff);	//gc	0xffff
	cmc623_I2cWrite16(0x00ce,0x0000);	//rm	0x0000
	cmc623_I2cWrite16(0x00cf,0xfffa);	//yw	0xffff
	cmc623_I2cWrite16(0x00d0,0x00ff);	//kb	0x00ff
	cmc623_I2cWrite16(0x00d1,0x00ff);	//gc	0x00ff
	cmc623_I2cWrite16(0x00d2,0x00ff);	//rm	0x00ff
	cmc623_I2cWrite16(0x00d3,0x00ff);	//yw	0x00ff
	cmc623_I2cWrite16(0x0000,0x0001);	//BANK 1
	cmc623_I2cWrite16(0x0020,0x0000);	//GAMMA 9
	cmc623_I2cWrite16(0x0021,0x0f00);
	cmc623_I2cWrite16(0x0022,0x0f00);
	cmc623_I2cWrite16(0x0023,0x0f00);
	cmc623_I2cWrite16(0x0024,0x0f00);
	cmc623_I2cWrite16(0x0025,0x0f00);
	cmc623_I2cWrite16(0x0026,0x0f00);
	cmc623_I2cWrite16(0x0027,0x0f00);
	cmc623_I2cWrite16(0x0028,0x0f00);
	cmc623_I2cWrite16(0x0029,0x0f00);
	cmc623_I2cWrite16(0x002A,0x0f00);
	cmc623_I2cWrite16(0x002B,0xa319);
	cmc623_I2cWrite16(0x002C,0xa319);
	cmc623_I2cWrite16(0x002D,0xa319);
	cmc623_I2cWrite16(0x002E,0xa319);
	cmc623_I2cWrite16(0x002F,0xa319);
	cmc623_I2cWrite16(0x0030,0xa319);
	cmc623_I2cWrite16(0x0031,0xa319);
	cmc623_I2cWrite16(0x0032,0xa319);
	cmc623_I2cWrite16(0x0033,0xa319);
	cmc623_I2cWrite16(0x0034,0xa319);
	cmc623_I2cWrite16(0x0035,0xa319);
	cmc623_I2cWrite16(0x0036,0xa319);
	cmc623_I2cWrite16(0x0037,0xa318);
	cmc623_I2cWrite16(0x0038,0xFF00);
	cmc623_I2cWrite16(0x0020,0x0001);
	cmc623_I2cWrite16(0x0000,0x0000);	//BANK 0
	cmc623_I2cWrite16(0x0028,0x0000);	//Register Mask
	//end
	
#elif 0
	//PLS tuning
	//start
	cmc623_I2cWrite16(0x0000,0x0000);	//BANK 0
	cmc623_I2cWrite16(0x0001,0x0060);	//SCR LABC
	cmc623_I2cWrite16(0x002c,0x0fff);	//DNR bypass	bypass,dir_th   0x003C
	cmc623_I2cWrite16(0x002d,0x1900);	//DNR bypass	dir_num,decont7 0x0a08
	cmc623_I2cWrite16(0x002e,0x0000);	//DNR bypass	decont5,mask_th 0x1010
	cmc623_I2cWrite16(0x002f,0x00ff);	//DNR bypass	block_th        0x0400
	cmc623_I2cWrite16(0x003A,0x000D);	//HDTR DE CS
	cmc623_I2cWrite16(0x003B,0x03ff);	//DE SHARPNESS
	cmc623_I2cWrite16(0x003C,0x0000);	//NOISE LEVEL
	cmc623_I2cWrite16(0x003F,0x0100);	//CS GAIN
	cmc623_I2cWrite16(0x0042,0x0000);	//DE TH (MAX DIFF) 3F
	//0x00b4,0x4640,	//Count PWM
	cmc623_I2cWrite16(0x00c8,0x0030);	//kb	0x0000	SCR
	cmc623_I2cWrite16(0x00c9,0x0000);	//gc	0x0000
	cmc623_I2cWrite16(0x00ca,0xffff);	//rm	0xffff
	cmc623_I2cWrite16(0x00cb,0xffff);	//yw	0xffff
	cmc623_I2cWrite16(0x00cc,0x0000);	//kb	0x0000
	cmc623_I2cWrite16(0x00cd,0xffff);	//gc	0xffff
	cmc623_I2cWrite16(0x00ce,0x0000);	//rm	0x0000
	cmc623_I2cWrite16(0x00cf,0xffe8);	//yw	0xffff
	cmc623_I2cWrite16(0x00d0,0x00ff);	//kb	0x00ff
	cmc623_I2cWrite16(0x00d1,0x00ff);	//gc	0x00ff
	cmc623_I2cWrite16(0x00d2,0x00ff);	//rm	0x00ff
	cmc623_I2cWrite16(0x00d3,0x00ff);	//yw	0x00ff
	cmc623_I2cWrite16(0x0000,0x0001);	//BANK 1
	cmc623_I2cWrite16(0x0020,0x0000);	//GAMMA 9
	cmc623_I2cWrite16(0x0021,0x0f00);
	cmc623_I2cWrite16(0x0022,0x0f00);
	cmc623_I2cWrite16(0x0023,0x0f00);
	cmc623_I2cWrite16(0x0024,0x0f00);
	cmc623_I2cWrite16(0x0025,0x0f00);
	cmc623_I2cWrite16(0x0026,0x0f00);
	cmc623_I2cWrite16(0x0027,0x0f00);
	cmc623_I2cWrite16(0x0028,0x0f00);
	cmc623_I2cWrite16(0x0029,0x0f00);
	cmc623_I2cWrite16(0x002A,0x0f00);
	cmc623_I2cWrite16(0x002B,0xa319);
	cmc623_I2cWrite16(0x002C,0xa319);
	cmc623_I2cWrite16(0x002D,0xa319);
	cmc623_I2cWrite16(0x002E,0xa319);
	cmc623_I2cWrite16(0x002F,0xa319);
	cmc623_I2cWrite16(0x0030,0xa319);
	cmc623_I2cWrite16(0x0031,0xa319);
	cmc623_I2cWrite16(0x0032,0xa319);
	cmc623_I2cWrite16(0x0033,0xa319);
	cmc623_I2cWrite16(0x0034,0xa319);
	cmc623_I2cWrite16(0x0035,0xa319);
	cmc623_I2cWrite16(0x0036,0xa319);
	cmc623_I2cWrite16(0x0037,0xa318);
	cmc623_I2cWrite16(0x0038,0xFF00);
	cmc623_I2cWrite16(0x0020,0x0001);
	cmc623_I2cWrite16(0x0000,0x0000);	//BANK 0
	cmc623_I2cWrite16(0x0028,0x0000);	//Register Mask
	//end

#endif
}

// value: 0 ~ 100
static void cmc623_cabc_pwm_brightness_reg(int value)
{
	u32 reg;
	unsigned char * p_plut;
	u16 min_duty;

	if(!p_cmc623_data)
		{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
		}

	p_plut = cmc623_Power_LUT[current_power_lut_num];

	min_duty = p_plut[7] * value / 100;
	if(min_duty < 4)
	{
		if(value == 0)
			reg = 0x4000;
		else
			reg = 0x4000 | ((max(1,(value*p_plut[3]/100)))<<4);		
	}
	else
	{
		cmc623_I2cWrite16(0x76,(p_plut[0] * value / 100) << 8 | (p_plut[1] * value / 100));	//PowerLUT
		cmc623_I2cWrite16(0x77,(p_plut[2] * value / 100) << 8 | (p_plut[3] * value / 100));	//PowerLUT
		cmc623_I2cWrite16(0x78,(p_plut[4] * value / 100) << 8 | (p_plut[5] * value / 100));	//PowerLUT
		cmc623_I2cWrite16(0x79,(p_plut[6] * value / 100) << 8 | (p_plut[7] * value / 100));	//PowerLUT
		cmc623_I2cWrite16(0x7a,(p_plut[8] * value / 100) << 8);	//PowerLUT

		reg = 0x5000 | (value<<4);
	}
	
	if(setting_first)
	{
		reg |= 0x8000;
	}

	cmc623_I2cWrite16(0xB4, reg);			//pwn duty
}

// value: 0 ~ 100
static void cmc623_cabc_pwm_brightness(int value)
{
	u32 reg;

	if(!p_cmc623_data)
		{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
		}

	cmc623_I2cWrite16(0x00,0x0000);	//BANK 0

	cmc623_cabc_pwm_brightness_reg(value);

	cmc623_I2cWrite16(0x28,0x0000);
}

// value: 0 ~ 100
// This should be used only for special purpose as resume
static void cmc623_manual_pwm_brightness_reg_nosync(int value)
{
	u32 reg;

	if(!p_cmc623_data)
		{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
		}

	reg = 0xC000 | (value);

	cmc623_I2cWrite16(0xB4, reg);			//pwn duty
}

// value: 0 ~ 100
static void cmc623_manual_pwm_brightness_reg(int value)
{
	u32 reg;

	if(!p_cmc623_data)
		{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
		}

	reg = 0x4000 | (value<<4);

	cmc623_I2cWrite16(0xB4, reg);			//pwn duty
}

// value: 0 ~ 100
static void cmc623_manual_pwm_brightness(int value)
{
	u32 reg;

	if(!p_cmc623_data)
		{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
		}

	reg = 0x4000 | (value<<4);

	cmc623_I2cWrite16(0x00, 0x0000);		//bank0
	cmc623_I2cWrite16(0xB4, reg);			//pwn duty
	cmc623_I2cWrite16(0x28, 0x0000);

}

u16 ove_target_value=0;
static void ove_workqueue_func(void* data)
{
	int i = 0;
	for(i=0; i<=8; ++i)
	{
		mutex_lock(&cmc623_state_transaction_lock);
		if(cmc623_state.suspended == TRUE)
			{
			mutex_unlock(&cmc623_state_transaction_lock);
			return;
			}

		cmc623_I2cWrite16(0x0054, (((ove_target_value >> 8) * i / 8) << 8) | ((ove_target_value & 0x00ff) * i / 8));
		cmc623_reg_unmask();
		mutex_unlock(&cmc623_state_transaction_lock);
		
		msleep(15);
	}
}

static int cmc623_OutDoor_Enable(int enable)
{
	u16 i2cdata=0;
	int i=0;

	if(enable)
	{
		switch(current_cmc623_UI)
		{
			case CMC623_UI_MODE:
			ove_target_value = OVE_values[6 + cmc623_state.cabc_enabled];
			break;

			case CMC623_VIDEO_MODE:
			ove_target_value = OVE_values[2 + cmc623_state.cabc_enabled];
			break;
			
			case CMC623_CAMERA_MODE:
			ove_target_value = OVE_values[4 + cmc623_state.cabc_enabled];
			break;

			case CMC623_DMB_MODE:
			ove_target_value = OVE_values[10 + cmc623_state.cabc_enabled];
			break;

			case CMC623_VT_MODE:
			ove_target_value = OVE_values[8 + cmc623_state.cabc_enabled];
			break;

			case CMC623_GALLERY_MODE:
			ove_target_value = OVE_values[12 + cmc623_state.cabc_enabled];
			break;
			
			default:
			ove_target_value = OVE_values[0];
			break;
		}
		
		if(ove_target_value == 0x00)
		{
			return 0;
		}

		cmc623_I2cWrite16(0x0000, 0x0000);
		cmc623_I2cRead16(0x0001, &i2cdata);
		i2cdata |= 0x0002;
		cmc623_I2cWrite16(0x0001, i2cdata);

		if(current_cmc623_OutDoor_OnOff != enable)
		{
			queue_work(ove_wq, &work_ove);
		}
		else
		{
			cmc623_I2cWrite16(0x0054, ove_target_value);
			cmc623_reg_unmask();
		}
	}
	else 
	{//outdoor mode off
		cmc623_I2cWrite16(0x0000, 0x0000);
		cmc623_I2cRead16(0x0001, &i2cdata);
		i2cdata &= 0xfffc;
		cmc623_I2cWrite16(0x0001, i2cdata);

		cmc623_reg_unmask();
	}
	current_cmc623_OutDoor_OnOff = enable;

	return 1;
}
//EXPORT_SYMBOL(cmc623_OutDoor_Enable);

// value: 0 ~ 1600
void tune_cmc623_pwm_brightness(int value)
{
	u32 data;

	if(!p_cmc623_data)
		{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		}

	if(value<0)
		data = 0;
	else if(value>1600)
		data = 1600;
	else
		data = value;

	mutex_lock(&cmc623_state_transaction_lock);
		
	if(data == 1280 && current_autobrightness_enable)
	{//outdoor mode on
		cmc623_OutDoor_Enable(TRUE);
		//current_cmc623_OutDoor_OnOff = TRUE;
		//cmc623_Set_Mode(current_cmc623_UI, current_cmc623_CABC_OnOff);
	}
	else if (current_cmc623_OutDoor_OnOff == TRUE && data < 1280)
	{//outdoor mode off
		cmc623_OutDoor_Enable(FALSE);
		//current_cmc623_OutDoor_OnOff = FALSE;
		//cmc623_Set_Mode(current_cmc623_UI, current_cmc623_CABC_OnOff);
	}

	data >>= 4;

	// data must not be zero unless value is zero
	if(value>0 && data==0)
		data = 1;

	cmc623_state.brightness = data;
	
	if(cmc623_state.cabc_enabled == TRUE)
	{
		cmc623_cabc_pwm_brightness(data);
	}
	else
	{
		cmc623_manual_pwm_brightness(data);
	}
	mutex_unlock(&cmc623_state_transaction_lock);
}
EXPORT_SYMBOL(tune_cmc623_pwm_brightness);

static int tune_cmc623_hw_rst()
{
    if (gpio_is_valid(GPIO_CMC_BYPASS)) 
    {
        gpio_direction_output(GPIO_CMC_BYPASS, GPIO_LEVEL_LOW);
    }
    s3c_gpio_setpull(GPIO_CMC_BYPASS, S3C_GPIO_PULL_NONE);
    gpio_set_value(GPIO_CMC_BYPASS, GPIO_LEVEL_HIGH);
    msleep(2);

    if (gpio_is_valid(GPIO_CMC_RST)) 
    {
        gpio_direction_output(GPIO_CMC_RST, GPIO_LEVEL_HIGH);
    }
    s3c_gpio_setpull(GPIO_CMC_RST, S3C_GPIO_PULL_NONE);
    gpio_set_value(GPIO_CMC_RST, GPIO_LEVEL_LOW);
    msleep(4);
    gpio_set_value(GPIO_CMC_RST, GPIO_LEVEL_HIGH);
    msleep(5);

	return 0;
}

int tune_cmc623_suspend()
{
	printk(KERN_INFO "%s called\n", __func__);

	if(!p_cmc623_data)
		{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return 0;
		}

	// 1.2V/1.8V/3.3V may be on

	mutex_lock(&cmc623_state_transaction_lock);
	// CMC623[0x07] := 0x0004
    cmc623_I2cWrite16(0x07, 0x0004);

	cmc623_state.suspended = TRUE;
	mutex_unlock(&cmc623_state_transaction_lock);

	// SLEEPB(L6) <= LOW
	gpio_set_value(GPIO_CMC_SLEEP, GPIO_LEVEL_LOW);

	// BYPASSB(A2) <= LOW
	gpio_set_value(GPIO_CMC_BYPASS, GPIO_LEVEL_LOW);

	// wait 1ms
	msleep(1);

	// FAILSAFEB(E6) <= LOW
	gpio_set_value(GPIO_CMC_SHDN, GPIO_LEVEL_LOW);

	// 1.2V/3.3V off 
	gpio_set_value(GPIO_CMC_EN, GPIO_LEVEL_LOW);

	// FAILSAFEB(E6) <= HIGH
	gpio_set_value(GPIO_CMC_SHDN, GPIO_LEVEL_HIGH);

	// 1.8V off (optional, but all io lines shoud be all low or all high or all high-z while sleep for preventing leakage current)

	// Reset chip
	//gpio_set_value(GPIO_CMC_RST, GPIO_LEVEL_LOW);
    msleep(100);

	return 0;
}
EXPORT_SYMBOL(tune_cmc623_suspend);

int tune_cmc623_pre_resume()
{
	printk(KERN_INFO "%s called\n", __func__);
		
	if(!p_cmc623_data)
		{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return 0;
		}

	// 1.2V/1.8V/3.3V may be off
	// FAILSAFEB(E6) may be high

	// FAILSAFEB(E6) <= LOW
	// RESETB(K6) <= HIGH
	gpio_set_value(GPIO_CMC_RST, GPIO_LEVEL_HIGH);
	gpio_set_value(GPIO_CMC_SHDN, GPIO_LEVEL_LOW);
	msleep(1);

	// 1.2V/1.8V/3.3V on
	gpio_set_value(GPIO_CMC_EN, GPIO_LEVEL_HIGH);
	msleep(1);

	return 0;
}
EXPORT_SYMBOL(tune_cmc623_pre_resume);

//CAUTION : pre_resume function must be called before using this function
int tune_cmc623_resume()
{
	printk(KERN_INFO "%s called\n", __func__);
		
	if(!p_cmc623_data)
		{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return 0;
		}	

	// FAILSAFEB(E6) <= HIGH            
	gpio_set_value(GPIO_CMC_SHDN, GPIO_LEVEL_HIGH);
	msleep(1);

	// BYPASSB(A2) <= HIGH
	gpio_set_value(GPIO_CMC_BYPASS, GPIO_LEVEL_HIGH);
	msleep(1);

	// SLEEPB(L6) <= HIGH
	gpio_set_value(GPIO_CMC_SLEEP, GPIO_LEVEL_HIGH);
	msleep(1);

	// RESETB(K6) <= LOW
	gpio_set_value(GPIO_CMC_RST, GPIO_LEVEL_LOW);

	// wait 4ms or above
	msleep(5);

	// RESETB(K6) <= HIGH
	gpio_set_value(GPIO_CMC_RST, GPIO_LEVEL_HIGH);

	// wait 0.3ms or above
	msleep(1);	//udelay(300);

	mutex_lock(&cmc623_state_transaction_lock);
	cmc623_state.suspended = FALSE;

	// set registers using I2C
	cmc623_initial_set();

#ifdef CMC623_TUNING
	cmc623_set_tuning();	//for test
#endif 

	// restore mode & cabc status
	//setting_first = TRUE;
	cmc623_state.brightness = 0;
	cmc623_cabc_enable_flag(cmc623_state.cabc_enabled, TRUE, FALSE);
	//setting_first = FALSE;
	mutex_unlock(&cmc623_state_transaction_lock);

	msleep(10);

	return 0;
}
EXPORT_SYMBOL(tune_cmc623_resume);

void tune_cmc623_set_lcddata(const struct s3cfb_lcd * pdata)
{
	s3cfb_lcd_timing_data = pdata->timing;
}
EXPORT_SYMBOL(tune_cmc623_set_lcddata);

void tune_cmc623_set_lcd_pclk(int pclk)
{
	set_cmc623_val_for_pclk(pclk);
}
EXPORT_SYMBOL(tune_cmc623_set_lcd_pclk);

static ssize_t tune_cmc623_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;

#ifdef CMC623_TUNING
	klogi("");
	ret = cmc623_load_data();
#endif

	if(ret<0)
    {   
        return sprintf(buf, "FAIL\n");
    }
    else
    {   
        return sprintf(buf, "OK\n");
    }
}

static ssize_t tune_cmc623_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(tune, S_IRUGO | S_IWUSR, tune_cmc623_show, tune_cmc623_store);

static ssize_t set_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "<addr> <data> ex)0x00, 0x0000\n");
}

static ssize_t set_reg_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	int ret;
	u32 data1, data2;
	
	// printk("[cmc623] %s : %s\n", __func__, buf);
	ret = sscanf(buf, "0x%x 0x%x\n", &data1, &data2);
	if(ret == 2)
		{
		// printk("addr:0x%04x, data:0x%04x\n", data1, data2);
		cmc623_I2cWrite16(data1, data2);
		}
	else
		{
		printk("parse error num:%d, data:0x%04x, data:0x%04x\n", ret, data1, data2);
		}

	return size;
}

static DEVICE_ATTR(set_reg, 0664, set_reg_show, set_reg_store);

static u32 read_reg_address=0;

static ssize_t read_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	u16 data2;
	
	// printk("[cmc623] %s\n", __func__);
	// printk("addr:0x%04x\n", read_reg_address);
	if(read_reg_address >= 0x100)
		ret = cmc623_I2cWrite16(0x00, 0x0001);
	else if(read_reg_address > 0x0)
		ret = cmc623_I2cWrite16(0x00, 0x0000);
	ret = cmc623_I2cRead16(read_reg_address, &data2);
	// printk("data:0x%04x\n", data2);

    return sprintf(buf, "addr:0x%04x, data:0x%04x\n", read_reg_address, data2);
}

static ssize_t read_reg_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	int ret;
	u32 data1;
	u16 data2;
	
	// printk("[cmc623] %s : %s\n", __func__, buf);
	ret = sscanf(buf, "0x%x\n", &data1);
	if(ret == 1)
		{
		read_reg_address = data1;
		// printk("addr:0x%04x\n", data1);
		if(read_reg_address >= 0x100)
			ret = cmc623_I2cWrite16(0x00, 0x0001);
		else if(read_reg_address > 0x0)
			ret = cmc623_I2cWrite16(0x00, 0x0000);
		ret = cmc623_I2cRead16(read_reg_address, &data2);
		// printk("data:0x%04x\n", data2);
		}
	else
		{
		printk("parse error num:%d, data:0x%04x\n", ret, data1);
		}

	return size;
}

static DEVICE_ATTR(read_reg, 0664, read_reg_show, read_reg_store);

static ssize_t show_regs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "write 1 for reading all regs\n");
}

static ssize_t show_regs_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	int i;
	int ret;
	u32 data1;
	u16 data2;
	
	// printk("[cmc623] %s : %s\n", __func__, buf);
	ret = sscanf(buf, "%d\n", &data1);
	if(ret == 1 && data1 == 1)
		{
		ret = cmc623_I2cWrite16(0x00, 0x0000);
		// printk("BANK0\n");
		for(i=0;i<ARRAY_SIZE(all_regs_bank0);i++)
			{
			if(all_regs_bank0[i] == DELIMITER)
				{
				// printk("------------------------\n");
				}
			else
				{
				ret = cmc623_I2cRead16(all_regs_bank0[i], &data2);
				// printk("addr:0x%04x, data:0x%04x\n", all_regs_bank0[i], data2);
				}
			}
		ret = cmc623_I2cWrite16(0x00, 0x0001);
		// printk("BANK1\n");
		for(i=0;i<ARRAY_SIZE(all_regs_bank1);i++)
			{
			if(all_regs_bank1[i] == DELIMITER)
				{
				// printk("------------------------\n");
				}
			else
				{
				ret = cmc623_I2cRead16(all_regs_bank1[i], &data2);
				// printk("addr:0x%04x, data:0x%04x\n", all_regs_bank1[i], data2);
				}
			}
		}
	else
		{
		printk("parse error num:%d, data:0x%04x\n", ret, data1);
		}

	printk("end %s\n", __func__);

	return size;
}

static DEVICE_ATTR(show_regs, 0664, show_regs_show, show_regs_store);

static ssize_t set_bypass_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	if(cmc623_bypass_mode)
    {   
        return sprintf(buf, "bypass\n");
    }
    else
    {   
        return sprintf(buf, "normal\n");
    }
}

static ssize_t set_bypass_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	//int i;
	int ret;
	u32 data1;
	//u16 data2;
	mDNIe_data_type *mode = cmc623_values[CMC_Bypass*LCD_TYPE_MAX+lcd_type];
	
	// printk("[cmc623] %s : %s\n", __func__, buf);
	ret = sscanf(buf, "%d\n", &data1);

	mutex_lock(&cmc623_state_transaction_lock);
	if(data1)
		{
		cmc623_bypass_mode = TRUE;
		while ( mode->addr != END_SEQ)
			{
			cmc623_I2cWrite16(mode->addr, mode->data);
			printk(KERN_INFO "[cmc623] a(0x%x),d(0x%x)\n",mode->addr, mode->data);	
			mode++;
			}
		
		// brightness setting
			{
			//Manual brightness setting
			cmc623_manual_pwm_brightness_reg(cmc623_state.brightness);

			cmc623_state.cabc_enabled = FALSE;
			}
		
		cmc623_reg_unmask();
		
		}
	else
		{
		cmc623_bypass_mode = FALSE;

		//setting_first = TRUE;
		cmc623_cabc_enable_flag(cmc623_state.cabc_enabled, TRUE, FALSE);
		//setting_first = FALSE;
		}
	mutex_unlock(&cmc623_state_transaction_lock);
	
	// printk("end %s\n", __func__);

	return size;
}

static DEVICE_ATTR(set_bypass, 0664, set_bypass_show, set_bypass_store);

static ssize_t color_white_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n",cmc623_state.white);
}

static ssize_t color_white_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	int white;

	sscanf(buf, "%d", &white);
	printk(KERN_NOTICE "%s:%d\n", __func__, white);

	mutex_lock(&cmc623_state_transaction_lock);
	cmc623_state.white = white;
	cmc623_Color_White_Change(cmc623_state.white,true);
	mutex_unlock(&cmc623_state_transaction_lock);

	return size;
}

static DEVICE_ATTR(color_white, 0664, color_white_show, color_white_store);

static ssize_t color_black_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if defined(CONFIG_TARGET_LOCALE_NTT)
	return sprintf(buf,"%d\n",-cmc623_state.black);
#else
	return sprintf(buf,"%d\n",cmc623_state.black);
#endif
}

static ssize_t color_black_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	int black;

	sscanf(buf, "%d", &black);
	printk(KERN_NOTICE "%s:%d\n", __func__, black);
	
	mutex_lock(&cmc623_state_transaction_lock);
	cmc623_state.black = black;

#if defined(CONFIG_TARGET_LOCALE_NTT)
	cmc623_state.black = -(cmc623_state.black);
#endif
	
	cmc623_Color_Black_Change(cmc623_state.black,true);
	mutex_unlock(&cmc623_state_transaction_lock);

	return size;
}

static DEVICE_ATTR(color_black, 0664, color_black_show, color_black_store);

static ssize_t color_saturation_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n",cmc623_state.saturation);
}

static ssize_t color_saturation_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	int saturation;

	sscanf(buf, "%d", &saturation);
	printk(KERN_NOTICE "%s:%d\n", __func__, saturation);
	
	mutex_lock(&cmc623_state_transaction_lock);
	cmc623_state.saturation = saturation;
	cmc623_Color_Saturation_Change(cmc623_state.saturation,true);
	mutex_unlock(&cmc623_state_transaction_lock);

	return size;
}

static DEVICE_ATTR(color_saturation, 0664, color_saturation_show, color_saturation_store);

static int cmc623_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cmc623_data *data;
//	struct i2c_client *c;
	//int ret = 0;

	// printk("==============================\n");
	// printk("cmc623 attach START!!!        \n");
	// printk("==============================\n");

	data = kzalloc(sizeof(struct cmc623_data), GFP_KERNEL);
	if (!data)
    {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
    }    

	data->client = client;
	i2c_set_clientdata(client, data);

	dev_info(&client->dev, "cmc623 i2c probe success!!!\n");

	p_cmc623_data = data;

	#if defined(SETTING_PCLK_55M)
		set_cmc623_val_for_pclk(55000000);
	#elif defined(SETTING_PCLK_53M)
		set_cmc623_val_for_pclk(53000000);
	#else
		set_cmc623_val_for_pclk(47000000);
	#endif

//	tune_cmc623_hw_rst();
	//cmc623_tuning_set();
//	cmc623_initial_set();
#ifdef CMC623_TUNING
	cmc623_set_tuning();	//for test
#endif
	cmc623_cabc_enable_flag(cmc623_state.cabc_enabled, TRUE, TRUE);
	
	return 0;
}

static int __devexit cmc623_i2c_remove(struct i2c_client *client)
{
	struct cmc623_data *data = i2c_get_clientdata(client);

	p_cmc623_data = NULL;

	i2c_set_clientdata(client, NULL);

	kfree(data);

	dev_info(&client->dev, "cmc623 i2c remove success!!!\n");

	return 0;
}

static const struct i2c_device_id sec_tune_cmc623_ids[] = {
	{ "sec_tune_cmc623_i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sec_tune_cmc623_ids);

struct i2c_driver sec_tune_cmc623_i2c_driver =  
{
	.driver	= {
		.name	= "sec_tune_cmc623_i2c",
        .owner = THIS_MODULE,
	},
	.probe 		= cmc623_i2c_probe,
	.remove 	= __devexit_p(cmc623_i2c_remove),
	.id_table	= sec_tune_cmc623_ids,
};

extern struct class *sec_class;
struct device *tune_cmc623_dev;

static int __devinit cmc623_probe(struct platform_device *pdev)
{
	int ret=0;

    // printk("**** < cmc623_probe >      ***********\n");

	tune_cmc623_dev = device_create(sec_class, NULL, 0, NULL, "sec_tune_cmc623");

	if (IS_ERR(tune_cmc623_dev)) 
    {
		printk("Failed to create device!");
		ret = -1;
	}
	if (device_create_file(tune_cmc623_dev, &dev_attr_tune) < 0) {
		printk("Failed to create device file!(%s)!\n", dev_attr_tune.attr.name);
		ret = -1;
	}
	if (device_create_file(tune_cmc623_dev, &dev_attr_set_reg) < 0) {
		printk("Failed to create device file!(%s)!\n", dev_attr_set_reg.attr.name);
		ret = -1;
	}
	if (device_create_file(tune_cmc623_dev, &dev_attr_read_reg) < 0) {
		printk("Failed to create device file!(%s)!\n", dev_attr_read_reg.attr.name);
		ret = -1;
	}
	if (device_create_file(tune_cmc623_dev, &dev_attr_show_regs) < 0) {
		printk("Failed to create device file!(%s)!\n", dev_attr_show_regs.attr.name);
		ret = -1;
	}
	if (device_create_file(tune_cmc623_dev, &dev_attr_set_bypass) < 0) {
		printk("Failed to create device file!(%s)!\n", dev_attr_set_bypass.attr.name);
		ret = -1;
	}
	if (device_create_file(tune_cmc623_dev, &dev_attr_color_white) < 0) {
		printk("Failed to create device file!(%s)!\n", dev_attr_color_white.attr.name);
		ret = -1;
	}
	if (device_create_file(tune_cmc623_dev, &dev_attr_color_black) < 0) {
		printk("Failed to create device file!(%s)!\n", dev_attr_color_black.attr.name);
		ret = -1;
	}
	if (device_create_file(tune_cmc623_dev, &dev_attr_color_saturation) < 0) {
		printk("Failed to create device file!(%s)!\n", dev_attr_color_saturation.attr.name);
		ret = -1;
	}
	
    // printk("<sec_tune_cmc623_i2c_driver Add START> \n");
    ret = i2c_add_driver(&sec_tune_cmc623_i2c_driver);    // P1_LSJ : DE07 
    // printk("cmc623_init Return value  (%d)\n", ret);
	
	ove_wq = create_singlethread_workqueue("ove_wq");
	INIT_WORK(&work_ove, ove_workqueue_func);

//	ret = cmc623_gamma_set();                             // P1_LSJ : DE19
//    printk("cmc623_gamma_set Return value  (%d)\n", ret);
    // printk("<sec_tune_cmc623_i2c_driver Add END>   \n");

	return ret;
}

static int __devexit cmc623_remove(struct platform_device *pdev)
{
    if (ove_wq)
		destroy_workqueue(ove_wq);

	i2c_del_driver(&sec_tune_cmc623_i2c_driver);

	return 0;
}

struct platform_driver sec_tune_cmc623 =  {
	.driver = {
		.name = "sec_tune_cmc623",
        .owner  = THIS_MODULE,
	},
    .probe  = cmc623_probe,
    .remove = cmc623_remove,
};

static int __init cmc623_init(void)
{

    // printk("**************************************\n");
    // printk("**** < cmc623_init  >               **\n");
    // printk("**************************************\n");

	return platform_driver_register(&sec_tune_cmc623);
}

static void __exit cmc623_exit(void)
{
	platform_driver_unregister(&sec_tune_cmc623);
}

// module_init(cmc623_init);
module_init(cmc623_init);
module_exit(cmc623_exit);

/* Module information */
MODULE_AUTHOR("Samsung");
MODULE_DESCRIPTION("Tuning CMC623 image converter");
MODULE_LICENSE("GPL");
