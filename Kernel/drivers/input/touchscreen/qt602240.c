/*
 * qt602240_ts.c - AT42QT602240 Touchscreen driver
 *
 * Copyright (C) 2009 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
 #include "qt602240.h"
 #include <linux/bln.h>
/******************************************************************************
*
*
*       QT602240 Object table init
*
*
* *****************************************************************************/
//General Object
static gen_powerconfig_t7_config_t power_config = {0};                 //Power config settings.
static gen_acquisitionconfig_t8_config_t acquisition_config = {0};     // Acquisition config.

//Touch Object
static touch_multitouchscreen_t9_config_t touchscreen_config = {0};    //Multitouch screen config.
static touch_keyarray_t15_config_t keyarray_config = {0};              //Key array config.

//Signal Processing Objects
static proci_gripfacesuppression_t20_config_t gripfacesuppression_config = {0};    //Grip / face suppression config.
static procg_noisesuppression_t22_config_t noise_suppression_config = {0};         //Noise suppression config.

//Support Objects
static spt_cteconfig_t28_config_t cte_config = {0};                //Capacitive touch engine config.
static gen_commandprocessor_t6_config_t    command_config = {0};

static report_finger_info_t fingerInfo[MAX_USING_FINGER_NUM];

static bool set_mode_for_ta = false;		// true: TA or USB, false: normal
static int set_mode_for_amoled = 0;		//0: TFt-LCD, 1: AMOLED
static int gFirmware_Update_State = FW_UPDATE_READY;
#if defined (CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
static bool gbfilter =false;
#endif

extern struct class *sec_class;

//unsigned char maxim_chg_status(void);	// 1: TA or UST, 0: normal

struct qt602240_data * p_qt602240_data;
#ifdef CONFIG_HAS_EARLYSUSPEND
static void qt602240_early_suspend(struct early_suspend *);
static void qt602240_late_resume(struct early_suspend *);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

static int qt602240_write_object(struct qt602240_data *data, u8 type, u8 offset, u8 val);
		
static bool cal_check_flag = false;
static unsigned int qt_time_point=0;
static unsigned int qt_time_diff=0;
static unsigned int qt_timer_state =0;
#if defined (LED_SWITCH)
static unsigned int p_time_point=0;
static unsigned int p_time_diff=0;
static int led_sw= 1;
#endif

#if defined (KEY_LED_CONTROL)
void led_control(int data)
{
    int i;

    data = data -1;

    local_irq_disable();
    gpio_direction_output(KEYLED_EN, 0);
    udelay(2);

    for(i = 0; i < data; i++)
    {
        gpio_direction_output(KEYLED_EN, 1);
        udelay(2);
        gpio_direction_output(KEYLED_EN, 0);
        udelay(2);
    }
    gpio_direction_output(KEYLED_EN, 1);
    local_irq_enable();

    //wait for the mode change
    mdelay(1);
}

void init_led(void)
{
    if(gpio_is_valid(KEYLED_EN))
    {
        gpio_request(KEYLED_EN, "KEYLED_EN");
        s3c_gpio_cfgpin(KEYLED_EN, GPIO_OUTPUT);
        s3c_gpio_setpull(KEYLED_EN, S3C_GPIO_PULL_NONE);
    }
}

static void touch_led_on(int val)
{
    static int preset = 0;
    int set = 2;
//    printk(KERN_DEBUG "[TSP] keyled : %d \n", val );

    if(val < 42)
        set = 1;

    if(val > 0)
    {
        if(set !=preset)
        {
            //KEYLED is only working in low current mode.
            led_control(16);
            led_control(KEYLED_ADDRESS_MAX);      // [Address ] Max current setting
            led_control(KEYLED_DATA_LOW);           // [Data] Low current mode
            led_control(KEYLED_ADDRESS_LOW);      // [Address] Low current mode
            if(set == 1)
            {
                led_control(2);                                     // [Data] 0.5mA
                preset = 1;
                //printk(KERN_DEBUG "[TSP] keyled : 0.5mA\n");
            }
            else
            {
                led_control(4);                                     // [Data] 2mA
                preset = 2;
                //printk(KERN_DEBUG "[TSP] keyled : 2mA\n");
            }
        }
    }
    else
    {
        gpio_direction_output(KEYLED_EN, 0);
        //printk(KERN_DEBUG "[TSP] keyled : off\n");
        preset = 0;
    }
}

#if 0
void touch_led_ctl(int type)
{
    int data =0;

    switch(type)
    {
        case 0:
            data = 1;
            break;

        case 1:
        case 2:
        case 3:
        case 4:
            /*
            Data      D4      D3      D2      D1
            1           on      on      on      on
            2           on      on      on      off
            3           on      on      off     on
            4           on      on      off     off
            ....
            16          off     off     off     off
            */
            data = ((0x1)<<(type-1)) + 1;
            break;

        default:
            printk(KERN_DEBUG "[TSP] %s Unknown data\n", __func__);
            break;
    }

    if(bLedOn)
    {
        // [Address ] Led on/off
        led_control(KEYLED_ADDRESS_ONOFF);
        led_control(data);
    }
}
#endif

static ssize_t key_led_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
    int i = 0;
    if(sscanf(buf,"%d",&i) !=1 )
    {
        printk(KERN_ERR"[TSP] keyled write error\n");
    }

#if defined (LED_SWITCH)
    if(led_sw == 1)
#endif
        touch_led_on(i);

    return size;
}
static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR, NULL, key_led_store);

#endif      //KEY_LED_CONTROL

#ifdef CONFIG_GENERIC_BLN
static void p1_touchkey_bln_enable(void)
{
   init_led();
   touch_led_on(255);
}

static void p1_touchkey_bln_disable(void)
{
  touch_led_on(false);
}

static struct bln_implementation p1_touchkey_bln = {
  .enable = p1_touchkey_bln_enable,
  .disable = p1_touchkey_bln_disable,
};
#endif

static int press_time(void)
{
    p_time_diff = jiffies_to_msecs(jiffies) - p_time_point;
    if(p_time_diff >2000)
    	return 1;
    else
    	return 0;
}

#if defined(DRIVER_FILTER)
#if defined (CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
static void equalize_coordinate(bool detect, u8 id, u16 *px, u16 *py)
{
    static int tcount[MAX_USING_FINGER_NUM] = { 0, };
    static u16 pre_x[MAX_USING_FINGER_NUM][4] = {{0}, };
    static u16 pre_y[MAX_USING_FINGER_NUM][4] = {{0}, };
    int coff[4] = {0,};
    int distance = 0;

    if(detect)
    {
        tcount[id] = 0;
    }

    pre_x[id][tcount[id]%4] = *px;
    pre_y[id][tcount[id]%4] = *py;

    if(gbfilter)
    {
         if(tcount[id] >3)
        {
            *px = (u16)((*px + pre_x[id][(tcount[id]-1)%4] + pre_x[id][(tcount[id]-2)%4] + pre_x[id][(tcount[id]-3)%4] )/4);
            *py = (u16)((*py+ pre_y[id][(tcount[id]-1)%4] + pre_y[id][(tcount[id]-2)%4]+ pre_y[id][(tcount[id]-3)%4])/4);
        }
        else switch(tcount[id])
        {
            case 2:
            {
                *px = (u16)((*px + pre_x[id][(tcount[id]-1)%4])>>1);
                *py = (u16)((*py + pre_y[id][(tcount[id]-1)%4])>>1);
                break;
            }

            case 3:
            {
                *px = (u16)((*px + pre_x[id][(tcount[id]-1)%4] + pre_x[id][(tcount[id]-2)%4])/3);
                *py = (u16)((*py + pre_y[id][(tcount[id]-1)%4] + pre_y[id][(tcount[id]-2)%4])/3);
                break;
            }

            default:
                break;
        }

    }
    else if(tcount[id] >3)
    {
        {
            distance = abs(pre_x[id][(tcount[id]-1)%4] - *px) + abs(pre_y[id][(tcount[id]-1)%4] - *py);

            coff[0] = (u8)(2 + distance/5);
            if(coff[0] < 8)
            {
                coff[0] = max(2, coff[0]);
                coff[1] = min((8 - coff[0]), (coff[0]>>1)+1);
                coff[2] = min((8 - coff[0] - coff[1]), (coff[1]>>1)+1);
                coff[3] = 8 - coff[0] - coff[1] - coff[2];

    //            printk(KERN_DEBUG "[TSP] %d, %d, %d, %d \n", coff[0], coff[1], coff[2], coff[3]);

                *px = (u16)((*px*(coff[0]) + pre_x[id][(tcount[id]-1)%4]*(coff[1])
                    + pre_x[id][(tcount[id]-2)%4]*(coff[2]) + pre_x[id][(tcount[id]-3)%4]*(coff[3]))/8);
                *py = (u16)((*py*(coff[0]) + pre_y[id][(tcount[id]-1)%4]*(coff[1])
                    + pre_y[id][(tcount[id]-2)%4]*(coff[2]) + pre_y[id][(tcount[id]-3)%4]*(coff[3]))/8);
            }
            else
            {
                *px = (u16)((*px*4 + pre_x[id][(tcount[id]-1)%4])/5);
                *py = (u16)((*py*4 + pre_y[id][(tcount[id]-1)%4])/5);
            }
        }
     }
    tcount[id]++;
}

#else   //CONFIG_TARGET_LOCALE_KOR
static void equalize_coordinate(bool detect, u8 id, u16 *px, u16 *py)
{
    static int tcount[MAX_USING_FINGER_NUM] = { 0, };
    static u16 pre_x[MAX_USING_FINGER_NUM][4] = {{0}, };
    static u16 pre_y[MAX_USING_FINGER_NUM][4] = {{0}, };
    int coff[4] = {0,};
    int distance = 0;

    if(detect)
    {
        tcount[id] = 0;
    }

    pre_x[id][tcount[id]%4] = *px;
    pre_y[id][tcount[id]%4] = *py;

    if(tcount[id] >3)
    {
        distance = abs(pre_x[id][(tcount[id]-1)%4] - *px) + abs(pre_y[id][(tcount[id]-1)%4] - *py);

        coff[0] = (u8)(4 + distance/5);
        if(coff[0] < 8)
        {
            coff[0] = max(4, coff[0]);
            coff[1] = min((10 - coff[0]), (coff[0]>>1)+1);
            coff[2] = min((10 - coff[0] - coff[1]), (coff[1]>>1)+1);
            coff[3] = 10 - coff[0] - coff[1] - coff[2];

//            printk(KERN_DEBUG "[TSP] %d, %d, %d, %d \n", coff[0], coff[1], coff[2], coff[3]);

            *px = (u16)((*px*(coff[0]) + pre_x[id][(tcount[id]-1)%4]*(coff[1])
                + pre_x[id][(tcount[id]-2)%4]*(coff[2]) + pre_x[id][(tcount[id]-3)%4]*(coff[3]))/10);
            *py = (u16)((*py*(coff[0]) + pre_y[id][(tcount[id]-1)%4]*(coff[1])
                + pre_y[id][(tcount[id]-2)%4]*(coff[2]) + pre_y[id][(tcount[id]-3)%4]*(coff[3]))/10);
        }
        else
        {
            *px = (u16)((*px*4 + pre_x[id][(tcount[id]-1)%4])/5);
            *py = (u16)((*py*4 + pre_y[id][(tcount[id]-1)%4])/5);
        }
    }
#if 0
    else switch(tcount[id])
    {
        case 2:
        {
            *px = (u16)((*px + pre_x[id][(tcount[id]-1)%4])>>1);
            *py = (u16)((*py + pre_y[id][(tcount[id]-1)%4])>>1);
            break;
        }

        case 3:
        {
            *px = (u16)((*px + pre_x[id][(tcount[id]-1)%4] + pre_x[id][(tcount[id]-2)%4])/3);
            *py = (u16)((*py + pre_y[id][(tcount[id]-1)%4] + pre_y[id][(tcount[id]-2)%4])/3);
            break;
        }

        default:
            break;
    }
#endif

    tcount[id]++;
}
#endif
#endif  //DRIVER_FILTER

static void release_all_fingers(struct input_dev *input_dev)
{
	int i;

	printk(KERN_DEBUG "[TSP] %s\n", __func__);

	for ( i= 0; i<MAX_USING_FINGER_NUM; ++i )
	{
	    if ( fingerInfo[i].pressure == -1 )
	        continue;

	    fingerInfo[i].pressure = 0;
#ifdef CONFIG_TOUCHSCREEN_QT602240_ROT90
	    input_report_abs(input_dev, ABS_MT_POSITION_X, fingerInfo[i].y);
	    input_report_abs(input_dev, ABS_MT_POSITION_Y, QT602240_MAX_XC - 1 - fingerInfo[i].x);
#else
	    input_report_abs(input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
	    input_report_abs(input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);
#endif
	    input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, fingerInfo[i].pressure);
	    input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].id);
	    input_report_abs(input_dev, ABS_PRESSURE, fingerInfo[i].pressure);
	    input_report_key(input_dev, BTN_TOUCH, 0);
	    input_mt_sync(input_dev);

	    if ( fingerInfo[i].pressure == 0 )
	        fingerInfo[i].pressure= -1;
	}

	input_sync(input_dev);
	s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
}

static void release_all_keys(struct input_dev *input_dev)
{
	int i;

	for(i = 0; i <NUMOFKEYS; i++ )
	{
		if(tsp_keystatus[i])
		{
		input_report_key(input_dev, tsp_keycodes[i], 0);
		input_sync(input_dev);
		printk(KERN_DEBUG "[TSP] %s key is pseudo release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
		tsp_keystatus[i] = KEY_RELEASE;
		}
	}
}

static struct qt602240_object * qt602240_get_object(struct qt602240_data *data, u8 type)
{
	struct qt602240_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++)
	{
		object = data->object_table + i;
		if (object->type == type)
		{
		    return object;
		}
	}

	dev_err(&data->client->dev, "invalid type\n");
	return NULL;
}

static int qt602240_reg_write(struct qt602240_data *qt602240data,  u8 type, u8 *values)
{
    int i;
    unsigned char data[I2C_MAX_SEND_LENGTH];
    struct i2c_msg wmsg;
    struct qt602240_object *object;

    object = qt602240_get_object(qt602240data, type);
    if (!object)
        printk(KERN_ERR"[TSP] error : 0x%x\n", object->type);

    if(object->size > ( I2C_MAX_SEND_LENGTH - 2 ))
        printk(KERN_ERR"[TSP][ERROR] %s() data length error\n", __FUNCTION__);

    wmsg.addr = qt602240data->client->addr;
    wmsg.flags = I2C_M_WR;
    wmsg.len = (object->size + 3);
    wmsg.buf = data;

    data[0] = object->start_address & 0x00ff;
    data[1] = object->start_address >> 8;

    for (i = 0; i < object->size + 1; i++)
    {
        data[i+2] = *(values+i);
    }

    return (i2c_transfer(qt602240data->client->adapter, &wmsg, 1));
}

static int QT602240_Command_Config_Init(struct qt602240_data *data)
{
    command_config.reset = 0x0;
    command_config.backupnv = 0x0;
    command_config.calibrate = 0x0;
    command_config.reportall = 0x0;
    command_config.reserve= 0x0;
    command_config.diagnostic = 0x0;

    return (qt602240_reg_write(data, GEN_COMMANDPROCESSOR_T6, (void *) &command_config));
}

static int QT602240_Powr_Config_Init(struct qt602240_data *data)
{
    power_config.idleacqint = 32;//0x20    //32ms in idle status
    power_config.actvacqint = 0xff;           // free run in active status
    power_config.actv2idleto = 0x32;          //10s
    return (qt602240_reg_write(data, GEN_POWERCONFIG_T7, (void *) &power_config));
}

static int QT602240_Acquisition_Config_Init(struct qt602240_data *data)
{
//    int version = data->info->version;

    acquisition_config.chrgtime = 8;                  //2us       Charge-transfer dwell time
    acquisition_config.reserved = 0x00;
    acquisition_config.tchdrift = 1;               // 1s              Touch drift time
    acquisition_config.driftst = 1;                // 1 cycle        Drift suspend time
    acquisition_config.tchautocal = 0x00;          // infinite        Touch Automatic Calibration
    acquisition_config.sync = 0x00;                  // disabled
    acquisition_config.atchcalst = 0x09;//0x05;  // 1800ms      Anti-touch calibration time
    acquisition_config.atchcalsthr = 0x0f;//0x14//                  Anti-touch Calibration suspend time

    return (qt602240_reg_write(data, GEN_ACQUISITIONCONFIG_T8, (void *) &acquisition_config));
}

static int QT602240_Multitouch_Threshold_Reset(struct qt602240_data *data)
{
        return qt602240_write_object(data, QT602240_TOUCH_MULTI,
                QT602240_TOUCH_TCHTHR, touchscreen_config.tchthr);
}

static void QT602240_Multitouch_Threshold_High(struct qt602240_data *data)
{
	int ret;
	u8 tchthr = 60;

	printk(KERN_DEBUG "[TSP] %s\n", __func__);
	
	ret = qt602240_write_object(data, QT602240_TOUCH_MULTI,
	                QT602240_TOUCH_TCHTHR, tchthr);
	if (ret < 0)
	{
		printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
	}
}

static int QT602240_Multitouch_Config_Init(struct qt602240_data *data)
{
//    int version = data->info->version;

    //0x80 :Scan en
    //0x8 : Disable vector change, 0x2: Enable reporting, 0x1 : Enable the multi-touch
    touchscreen_config.ctrl = 0x8f;         //Disable amplitude change : 0x1 << 3
    touchscreen_config.xorigin = 0x00;
    touchscreen_config.yorigin = 0x00;

    touchscreen_config.xsize = 0x12;
    touchscreen_config.ysize = 0x0b;

    touchscreen_config.akscfg = 1;
    touchscreen_config.blen = 0x00;         // Gain of the analog circuits in front of the ADC [7:4]
    touchscreen_config.tchthr = 28;//0x27;       // touch Threshold value
    touchscreen_config.orient = 0x04;       // 0x4 : Invert Y, 0x2 : Invert X, 0x1 : Switch

    touchscreen_config.mrgtimeout = 0x00;
    touchscreen_config.movhysti = 16;   //0x1;    // Move hysteresis, initial
    touchscreen_config.movhystn = 10;  //0x1     // Move hysteresis, next
#if defined(CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
    touchscreen_config.movfilter = 0x0c;              // Filter Limit[6:4] , Adapt threshold [3:0]
#else
    touchscreen_config.movfilter = 0x0b;              // Filter Limit[6:4] , Adapt threshold [3:0]
#endif
    touchscreen_config.numtouch= 0x0a;
    touchscreen_config.tchdi = 0x02;
    touchscreen_config.mrghyst = 0x5;               // Merge hysteresis
    touchscreen_config.mrgthr = 0x5;//0xa            // Merge threshold
    touchscreen_config.amphyst = 0x0a;              // Amplitude hysteresis
    touchscreen_config.xrange1= 0xff;   //2byte
    touchscreen_config.xrange2= 0x03;

    touchscreen_config.yrange1 = 0x57;  //2byte
    touchscreen_config.yrange2 = 0x02;
    touchscreen_config.xloclip = 0x00;
    touchscreen_config.xhiclip = 0x00;
    touchscreen_config.yloclip = 0x00;
    touchscreen_config.yhiclip = 0x00;
    touchscreen_config.xedgectrl = 0x00;
    touchscreen_config.xedgedist = 0x00;
    touchscreen_config.yedgectrl = 0x00;
    touchscreen_config.yedgedist = 0x00;
#ifdef CONFIG_TARGET_LOCALE_USAGSM
    touchscreen_config.jumplimit = 18;
#else
    touchscreen_config.jumplimit = 10;            // ??*8
#endif

    return (qt602240_reg_write(data, TOUCH_MULTITOUCHSCREEN_T9, (void *) &touchscreen_config));
}

static int QT602240_KeyArrary_Config_Init(struct qt602240_data *data)
{

    keyarray_config.ctrl = 0x3;
    keyarray_config.xorigin = 0x00;
    keyarray_config.yorigin = 0x0b;
    keyarray_config.xsize = 0x04;
    keyarray_config.ysize = 0x01;
    keyarray_config.akscfg = 1;
    keyarray_config.blen = 0x00;
    keyarray_config.tchthr = 25;        //25
    keyarray_config.tchdi = 3;      //2;

    keyarray_config.reserved[0] = 0;
    keyarray_config.reserved[1] = 0;

    return (qt602240_reg_write(data, TOUCH_KEYARRAY_T15, (void *) &keyarray_config));
}

static int QT602240_Grip_Face_Suppression_Config_Init(struct qt602240_data *data)
{
//    int version = data->info->version;
    gripfacesuppression_config.ctrl = 0x13;//0x4 : enable Grip suppression, 0x2: Enable report, 0x1 : Enable
    gripfacesuppression_config.xlogrip = 5;
    gripfacesuppression_config.xhigrip = 5;
    gripfacesuppression_config.ylogrip = 5;
    gripfacesuppression_config.yhigrip = 5;
    gripfacesuppression_config.maxtchs = 0x00;
    gripfacesuppression_config.reserved = 0x00;
    gripfacesuppression_config.szthr1 = 0x19;
    gripfacesuppression_config.szthr2 = 0x0f;
    gripfacesuppression_config.shpthr1 = 0x02;
    gripfacesuppression_config.shpthr2 = 0x0a;
    gripfacesuppression_config.supextto = 0x32;

    /* Write grip suppression config to chip. */
    return (qt602240_reg_write(data, PROCI_GRIPFACESUPPRESSION_T20, (void *) &gripfacesuppression_config));
}

static int QT602240_Noise_Config_Init(struct qt602240_data *data)
{

//    int version = data->info->version;
    //0x8 : Enable Median filter, 0x4 : Enable Frequency hopping, 0x1 : Enable
#if defined(CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
    noise_suppression_config.ctrl = 0x0d;    //Median filter off, report enable
#else
    noise_suppression_config.ctrl = 0x0d;    //Median filter off, report enable
#endif
    noise_suppression_config.reserved = 0;
    noise_suppression_config.reserved1 = 0;
    noise_suppression_config.gcaful1 = 0;        // Upper limit for the GCAF
    noise_suppression_config.gcaful2 = 0;
    noise_suppression_config.gcafll1 = 0;        // Lower limit for the GCAF
    noise_suppression_config.gcafll2 = 0;
    noise_suppression_config.actvgcafvalid = 3;//0x0f;   //Minium number of samples in active mode
    noise_suppression_config.noisethr = 20;       //0x0f;  // Threshold for the noise signal
    noise_suppression_config.freqhopscale = 0x00;//0x1e;
#if defined(CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
    noise_suppression_config.freq[0] = 11;
    noise_suppression_config.freq[1] = 15;
    noise_suppression_config.freq[2] = 36;
    noise_suppression_config.freq[3] = 45;
    noise_suppression_config.freq[4] = 55;
#else
    noise_suppression_config.freq[0] = 10;
    noise_suppression_config.freq[1] = 15;
    noise_suppression_config.freq[2] = 20;
    noise_suppression_config.freq[3] = 25;
    noise_suppression_config.freq[4] = 30;
#endif
    noise_suppression_config.idlegcafvalid = 3;

    /* Write Noise suppression config to chip. */
    return (qt602240_reg_write(data, PROCG_NOISESUPPRESSION_T22, (void *) &noise_suppression_config));
}

static int QT602240_CTE_Config_Init(struct qt602240_data *data)
{
//    int version = data->info->version;

     /* Set CTE config */
    cte_config.ctrl = 0x00;     //reserved
    cte_config.cmd = 0x00;
    cte_config.mode = 0x02;
    cte_config.idlegcafdepth = 0x8;//0x20      //Size of sampling window in idle acquisition mode
#if defined(CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
    cte_config.actvgcafdepth = 32;//0x63   //Size of sampling window in active acquisition mode
#else
    cte_config.actvgcafdepth = 32;//0x63   //Size of sampling window in active acquisition mode
#endif
    cte_config.voltage = 0x0a;                    // 0.01 * 10 + 2.7

     /* Write CTE config to chip. */
    return (qt602240_reg_write(data, SPT_CTECONFIG_T28, (void *) &cte_config));
}

static int qt602240_object_readable(unsigned int type)
{
	switch (type) {
	case QT602240_GEN_MESSAGE:
	case QT602240_GEN_COMMAND:
	case QT602240_GEN_POWER:
	case QT602240_GEN_ACQUIRE:
	case QT602240_TOUCH_MULTI:
	case QT602240_TOUCH_KEYARRAY:
	case QT602240_TOUCH_PROXIMITY:
	case QT602240_PROCI_GRIPFACE:
	case QT602240_PROCG_NOISE:
	case QT602240_PROCI_ONETOUCH:
	case QT602240_PROCI_TWOTOUCH:
	case QT602240_SPT_GPIOPWM:
	case QT602240_SPT_SELFTEST:
	case QT602240_SPT_CTECONFIG:
		return 1;
	default:
		return 0;
	}
}

static int qt602240_check_bootloader(struct i2c_client *client,
		unsigned int state)
{
	u8 val;
	static int error_count = 0;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1)
	{
		dev_err(&client->dev, "i2c recv failed\n");
		return -EIO;
	}

	switch (state) {
	case QT602240_WAITING_BOOTLOAD_CMD:
	case QT602240_WAITING_FRAME_DATA:
		val &= ~QT602240_BOOT_STATUS_MASK;
		break;
	case QT602240_FRAME_CRC_PASS:
		if (val == QT602240_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state)
	{
	    if(error_count>=10)
	    {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		return -EINVAL;
           }
           else
           {
		error_count++;
		printk(KERN_ERR"[TSP] state : 0x%x, return val : 0x%d , error_count : %d\n", state, val, error_count );
		return 1;
	     }
	}

	return 0;
}

static int qt602240_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = QT602240_UNLOCK_CMD_LSB;
	buf[1] = QT602240_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "i2c send failed\n");
		return -EIO;
	}

	return 0;
}

static int qt602240_fw_write(struct i2c_client *client, const u8 *data,
		unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "i2c send failed\n");
		return -EIO;
	}

	return 0;
}

static int qt602240_read_reg(struct i2c_client *client, u16 reg)
{
	u8 buf[2];
	u8 val;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "i2c send failed\n");
		return -EIO;
	}

	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "i2c recv failed\n");
		return -EIO;
	}

	return val;
}

static int qt602240_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = val;

	if (i2c_master_send(client, buf, 3) != 3) {
		dev_err(&client->dev, "i2c send failed\n");
		return -EIO;
	}

	return 0;
}

static int qt602240_read_object_table(struct i2c_client *client, u16 reg,
		u8 *object_buf)
{
	u8 buf[2];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "i2c send failed\n");
		return -EIO;
	}

	if (i2c_master_recv(client, object_buf, 6) != 6) {
		dev_err(&client->dev, "i2c recv failed\n");
		return -EIO;
	}

	return 0;
}

static int qt602240_read_message(struct qt602240_data *data)
{
	struct i2c_client *client = data->client;
	struct qt602240_object *object;
	u16 reg;
	u8 buf[2];

	object = qt602240_get_object(data, QT602240_GEN_MESSAGE);
	if (!object)
		return -EINVAL;

	reg = object->start_address;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "i2c send failed\n");
		return -EIO;
	}

	if (i2c_master_recv(client, (u8 *)data->object_message, 9) != 9) {
		dev_err(&client->dev, "i2c recv failed\n");
		return -EIO;
	}

	return 0;
}

static int qt602240_read_diagnostic(u16 read_addr, u8 *buffer, u8 size)
{
	struct i2c_client *client = p_qt602240_data->client;
	struct qt602240_object *object;
	u16 reg;
	u8 buf[2];

	object = qt602240_get_object(p_qt602240_data, QT602240_DEBUG_DIAGNOSTIC);
	if (!object)
		return -EINVAL;

	reg = object->start_address + read_addr;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "i2c send failed\n");
		return -EIO;
	}

	if (i2c_master_recv(client, buffer, size) != size) {
		dev_err(&client->dev, "i2c recv failed\n");
		return -EIO;
	}

	return 0;
}


static int qt602240_read_object(struct qt602240_data *data, u8 type, u8 offset)
{
	struct qt602240_object *object;
	u16 reg;

	object = qt602240_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;

	return qt602240_read_reg(data->client, reg + offset);
}

static int qt602240_write_object(struct qt602240_data *data, u8 type,
		u8 offset, u8 val)
{
	struct qt602240_object *object;
	u16 reg;

	object = qt602240_get_object(data, type);
	if (!object)
	{
	    printk(KERN_ERR"[TSP] error : 0x%x\n", object->type);
		return -EINVAL;
	}

	reg = object->start_address;
	return qt602240_write_reg(data->client, reg + offset, val);
}

static void calibrate_chip(struct qt602240_data *data)
{
    uint8_t atchcalst, atchcalsthr;
    int error;

    if(!cal_check_flag)
    {
        printk(KERN_DEBUG "[TSP] Calibrating...\n");
        /* change calibration suspend settings to zero until calibration confirmed good */
        /* store normal settings */
        atchcalst = acquisition_config.atchcalst;
        atchcalsthr = acquisition_config.atchcalsthr;

        /* resume calibration must be performed with zero settings */
        acquisition_config.atchcalst = 0;
        acquisition_config.atchcalsthr = 0;

        error = QT602240_Acquisition_Config_Init(data);
        if(error<0)
        {
            printk(KERN_ERR "[TSP] fail to initialize the Acqusition config\n");
        }

        /* restore settings to the local structure so that when we confirm the
        * cal is good we can correct them in the chip */
        /* this must be done before returning */
        acquisition_config.atchcalst = atchcalst;
        acquisition_config.atchcalsthr = atchcalsthr;
    }

    /* send calibration command to the chip */
    error = qt602240_write_object(data, QT602240_GEN_COMMAND,
                QT602240_COMMAND_CALIBRATE, 1);
    if (error < 0)
    {
        printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
    }
    else
    {
        /* set flag to show we must still confirm if calibration was good or bad */
        cal_check_flag = true;
        release_all_fingers(data->input_dev);
    }

}

static int check_abs_time(void)
{
    qt_time_diff = jiffies_to_msecs(jiffies) - qt_time_point;
    if(qt_time_diff >0)
        return 1;
    else
        return 0;
}

static void check_chip_calibration(struct qt602240_data *data)
{
    u8 data_buffer[100] = { 0 };
    u16 check_tsp = 0;
    int try_ctr = 0;
    int tch_ch = 0, atch_ch = 0;
    int i, error, x_line_limit;
    static int anti_cnt = 0;
    static int bad_cnt = 0;

    /* we have had the first touchscreen or face suppression message
    * after a calibration - check the sensor state and try to confirm if
    * cal was good or bad */

    /* get touch flags from the chip using the diagnostic object */
    /* write command to command processor to get touch flags - 0xF3 Command required to do this */
    error = qt602240_write_object(data, QT602240_GEN_COMMAND,
        	QT602240_COMMAND_DIAGNOSTIC, 0xF3);
    if (error < 0)
    {
        printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
    }

    qt602240_read_diagnostic(0, data_buffer, 2 );

    while(!((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00)))
    {
        /* wait for data to be valid  */
        if(try_ctr > 10) //0318 hugh 100-> 10
        {
            /* Failed! */
            printk(KERN_ERR "[TSP] Diagnostic Data did not update!!\n");
            qt_timer_state = 0;//0430 hugh

            /* soft reset */
            qt602240_write_object(data, QT602240_GEN_COMMAND,
                	QT602240_COMMAND_RESET, 1);

            /* wait for soft reset */
            msleep(150);
            calibrate_chip(data);
            break;
        }
        msleep(2); //0318 hugh  3-> 2
        try_ctr++; /* timeout counter */

        qt602240_read_diagnostic(0, data_buffer, 2 );
//        printk("[TSP] Waiting for diagnostic data to update, try %d\n", try_ctr);
    }

    /* data is ready - read the detection flags */
    qt602240_read_diagnostic(0, data_buffer, 82);

    /* data array is 20 x 16 bits for each set of flags, 2 byte header, 40 bytes for touch flags 40 bytes for antitouch flags*/

    /* count up the channels/bits if we recived the data properly */
    if((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00))
    {
        x_line_limit = touchscreen_config.xsize;

        if(x_line_limit > 20)
        {
            /* hard limit at 20 so we don't over-index the array */
            x_line_limit = 20;
        }

        /* double the limit as the array is in bytes not words */
        x_line_limit = x_line_limit << 1;

        for(i = 0; i < x_line_limit; i++)
        {
            check_tsp = data_buffer[2+i];
            while(check_tsp)
            {
                if(check_tsp & 0x1)
                {
                    tch_ch++;
                }
                check_tsp = check_tsp >>1;
            }

            check_tsp = data_buffer[42+i];
            while(check_tsp)
            {
                if(check_tsp & 0x1)
                {
                    atch_ch++;
                }
                check_tsp = check_tsp >>1;
            }
        }

        /* print how many channels we counted */
            printk(KERN_DEBUG "[TSP] Flags Counted channels: t:%d a:%d \n", tch_ch, atch_ch);

        /* send page up command so we can detect when data updates next time,
			 * page byte will sit at 1 until we next send F3 command */
        error = qt602240_write_object(data, QT602240_GEN_COMMAND,
        	QT602240_COMMAND_DIAGNOSTIC, 0x01);
        if (error < 0)
        {
            printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
        }

        /* process counters and decide if we must re-calibrate or if cal was good */
        if((tch_ch>0) && (atch_ch == 0))  //jwlee change.
        {
	anti_cnt = 0;
                bad_cnt = 0;        
            /* cal was good - don't need to check any more */
            //hugh 0312
            if(!check_abs_time())
                qt_time_diff=1001;

            if(qt_timer_state == 1)
            {
                if(qt_time_diff > 1000)
                {
                    printk(KERN_DEBUG "[TSP] calibration was good\n");
                    qt_timer_state =0;
                    cal_check_flag = false;

                    /* Write normal acquisition config back to the chip. */
                    error = QT602240_Acquisition_Config_Init(data);
                    if(error<0)
                    {
                        printk(KERN_ERR "[TSP] fail to initialize the Acqusition config\n");
                    }
                }

            }
            else
            {
            	qt_timer_state=1;
            	qt_time_point = jiffies_to_msecs(jiffies);
            	cal_check_flag=true;
            }

        }
        else if(atch_ch >= 3)		//jwlee add 0325
        {
                anti_cnt = 0;        

	if (bad_cnt > 5) {
		bad_cnt = 0;
            printk(KERN_DEBUG "[TSP] calibration was bad.....resetting\n");
	            /* soft reset */
	            qt602240_write_object(data, QT602240_GEN_COMMAND,
	                	QT602240_COMMAND_RESET, 1);

		/* releasing finger  is needed when reset */
	            release_all_fingers(data->input_dev);  

	            /* wait for soft reset */
	            msleep(150);
	}
	else {
		bad_cnt++;
            printk(KERN_DEBUG "[TSP] calibration was bad\n");
	            /* cal was bad - must recalibrate and check afterwards */
	            calibrate_chip(data);
	            qt_timer_state=0;
	}
        }
        else if(atch_ch >= 1)
        {

            if(anti_cnt > 5)
            {
              printk(KERN_DEBUG "[TSP] anti_cnt:%d\n", anti_cnt);
	anti_cnt = 0;
                calibrate_chip(data);
                qt_timer_state=0;
            }
            else
                anti_cnt++;        
        }
        else
        {
    //        printk(KERN_DEBUG "[TSP] calibration was not decided yet\n");
            /* we cannot confirm if good or bad - we must wait for next touch  message to confirm */
            /* Reset the 100ms timer */
            qt_timer_state=0;//0430 hugh 1 --> 0
            qt_time_point = jiffies_to_msecs(jiffies);
	anti_cnt = 0;
        }
    }
}


static void qt602240_input_read(struct qt602240_data *data)
{
	struct qt602240_message *message = data->object_message;
	struct qt602240_object *object;
	struct input_dev *input_dev = data->input_dev;
	bool touch_message_flag;
	u8 reportid = 0xff;
	u8 touch_status = 0;
	u8 id, size;
	int i;
	int bChangeUpDn= 0;
	int x = 0, y = 0;
	static int nPrevID= -1;

	touch_message_flag = false;
	if (qt602240_read_message(data)) {
		printk(KERN_ERR "[TSP] Couldn't read message\n");
		goto soft_reset;
	}

	reportid = message->reportid;

	if (( reportid >= REPORTID_TSP_MIN ) && ( reportid <= REPORTID_TSP_MAX )) {
		id = message->reportid-2;

		/* whether reportid is thing of QT602240_TOUCH_MULTI */
		object = qt602240_get_object(data, QT602240_TOUCH_MULTI);
		if (object < 0) {
			printk(KERN_ERR "[TSP] Couldn't get the object\n");
			goto soft_reset;
		}

		touch_status = message->message[0];                             //Message[0] : Touch status

		x = (message->message[1] << 2) |                                 //Message[1] : x position MSByte
			((message->message[3] & ~0x3f) >> 6);                   //Message[3] : x position LSBits , bit4 ~ 7

		y = (message->message[2] << 2) |                                 //Message[2] : y position MSByte
			((message->message[3] & ~0xf3) >> 2);                  //Message[3] : y position LSBits , bit0~3

		size = message->message[4];

		if ( touch_status & 0x20 ) {                                         // Release : 0x20
			s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_7);
			fingerInfo[id].pressure= 0;
			bChangeUpDn= 1;
		} else if ((touch_status & 0xf0 ) == 0xc0) {                                  // Detect & Press  : 0x80 | 0x40
			touch_message_flag = true;
			s5pv210_lock_dvfs_high_level(DVFS_LOCK_TOKEN_7, L2); // 1000MHz
			fingerInfo[id].pressure= 40;
			fingerInfo[id].x= (int16_t)x;
			fingerInfo[id].y= (int16_t)y;
			bChangeUpDn= 1;
#if defined(DRIVER_FILTER)
			equalize_coordinate(1, id, &fingerInfo[id].x, &fingerInfo[id].y);
#endif
		} else if ((touch_status & 0xf0 ) == 0x90 ) {	                      // Detect & Move : 0x80 | 0x10
			fingerInfo[id].x= (int16_t)x;
			fingerInfo[id].y= (int16_t)y;
#if defined(DRIVER_FILTER)
			equalize_coordinate(0, id, &fingerInfo[id].x, &fingerInfo[id].y);
#endif
		}
		else
			printk(KERN_DEBUG "[TSP] Unknown state(%x)! \n", touch_status);

		fingerInfo[id].id = id;
		fingerInfo[id].size = size;

		if (nPrevID >= id || bChangeUpDn ) {

			for (i= 0; i<MAX_USING_FINGER_NUM; ++i ) {
				if (fingerInfo[i].pressure == -1 )
					continue;
#ifdef CONFIG_TOUCHSCREEN_QT602240_ROT90
				input_report_abs(input_dev, ABS_MT_POSITION_X, fingerInfo[i].y);
				input_report_abs(input_dev, ABS_MT_POSITION_Y, QT602240_MAX_XC - 1 - fingerInfo[i].x);
#else
				input_report_abs(input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
				input_report_abs(input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);
#endif
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, fingerInfo[i].pressure);
				input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].size);
				input_report_abs(input_dev, ABS_MT_TRACKING_ID, fingerInfo[i].id);
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, fingerInfo[i].pressure);

				if (fingerInfo[i].pressure == 0 ) {
					input_report_key(input_dev, BTN_TOUCH, 0);
					fingerInfo[i].pressure= -1;
				}
				else
					input_report_key(input_dev, BTN_TOUCH, 1);

				input_mt_sync(input_dev);
			}
			input_sync(input_dev);
		}

		nPrevID= id;

		}
	else if (( reportid >= REPORTID_TSPKEY_MIN )&& ( reportid <= REPORTID_TSPKEY_MAX )) {
		/* whether reportid is thing of QT602240_TOUCH_KEYARRAY */
		object = qt602240_get_object(data, QT602240_TOUCH_KEYARRAY);
		if (object < 0) {
			printk(KERN_ERR "[TSP] Couldn't get the object\n");
			goto soft_reset;
		}

		for (i = 0; i <NUMOFKEYS; i++ ) {
			if (tsp_keystatus[i]) {
				input_report_key(input_dev, tsp_keycodes[i], 0);
				input_sync(input_dev);
				tsp_keystatus[i] = KEY_RELEASE;
#if defined (LED_SWITCH)
				if(i == 0) {
					if(p_time_point > 0) {
						if(press_time()){
							if (led_sw){
								touch_led_on(false);
								led_sw= 0;
							} else {
								init_led();
								touch_led_on(255);
								led_sw= 1;
							}
						}
					}
					p_time_point= 0;
				}
#endif
			} else if (message->message[1] & (0x1<<i) ) {
				if(message->message[0] & 0x80) {                                  // detect
					input_report_key(input_dev, tsp_keycodes[i], 1);
					input_sync(input_dev);
					tsp_keystatus[i] = KEY_PRESS;
#if defined (LED_SWITCH)
					if(i == 0) {
						p_time_point = jiffies_to_msecs(jiffies);
					}
#endif
				}
			}
		}
	}
	else if ( reportid == REPORTID_PALM ) {      	//20102017 julia
		/* whether reportid is thing of QT602240_PROCI_GRIPFACE */
		object = qt602240_get_object(data, QT602240_PROCI_GRIPFACE);
		if (object < 0) {
			printk(KERN_ERR "[TSP] Couldn't get the object : palm touch.\n");
			goto soft_reset;
		}

		if (((message->message[0])&0x01) == 0x00) {
			printk(KERN_DEBUG "[TSP] Palm Touch!released.\n");
			release_all_fingers(input_dev);
		} else {
			printk(KERN_DEBUG "[TSP] Palm Touch!suppressed\n");
			touch_message_flag = true;
		}
	}
	else if (reportid == REPORTID_COMMANDPROECESSOR) {
		/* Check the calibration and overflow */
		object = qt602240_get_object(data, QT602240_GEN_COMMAND);
		if (object < 0) {
			printk(KERN_ERR "[TSP] Couldn't get the object : QT602240_GEN_COMMAND \n");
			goto soft_reset;
		} else
			printk(KERN_DEBUG "[TSP] msg:0x%x\n", message->message[0]);

		if (((message->message[0])&0x80) == 0x80 ||((message->message[0])&0x10) == 0x10)
			release_all_fingers(input_dev);

		if (((message->message[0])&0x40) == 0x40) {
			printk(KERN_ERR "[TSP] Overflow!!0: 0x%x, 1: 0x%x, 2: 0x%x, 3: 0x%x, 4: 0x%x, 5: 0x%x, 6: 0x%x, checksum: 0x%x\n",
			message->message[0], message->message[1], message->message[2],
			message->message[3], message->message[4], message->message[5],
			message->message[6], message->checksum);
			goto soft_reset;
		}
	}

	if(cal_check_flag && touch_message_flag)
		check_chip_calibration(data);
	return ;

soft_reset:
	release_all_fingers(input_dev);
	release_all_keys(data->input_dev);

	/* try to soft reset */
	qt602240_write_object(data, QT602240_GEN_COMMAND,
		QT602240_COMMAND_RESET, 1);

	/* wait for soft reset */
	msleep(150);

	/* calibrate again */
	calibrate_chip(data);

}

static irqreturn_t qt602240_interrupt(int irq, void *dev_id)
{
    struct qt602240_data *data = dev_id;

    qt602240_input_read(data);

    return IRQ_HANDLED;
}

static void qt602240_reg_init(struct qt602240_data *data)
{
    int ret;

    ret = QT602240_Command_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the Command config\n");

    ret = QT602240_Powr_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the Power config\n");

    ret = QT602240_Acquisition_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the Acqusition config\n");

    ret = QT602240_Multitouch_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the Multi-touch config\n");

    ret = QT602240_KeyArrary_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the KeyArrary config\n");

    ret = QT602240_Grip_Face_Suppression_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the suppression config\n");

    ret = QT602240_Noise_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the Noise config\n");

    ret = QT602240_CTE_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the CTE config\n");

}

static void qt602240_check_matrix_size(struct qt602240_data *data)
{
    u8 mode;
    mode = touchscreen_config.xsize - 16;
    if( mode != cte_config.mode)
        qt602240_write_object(data, QT602240_SPT_CTECONFIG,
                QT602240_CTE_MODE, mode);
}

static int qt602240_info_get(struct qt602240_data *data)
{
	struct i2c_client *client = data->client;
	struct qt602240_info *info = data->info;
	int val;

	val = qt602240_read_reg(client, QT602240_FAMILY_ID);
	if (val < 0)
		return -EIO;
	info->family_id = val;

	val = qt602240_read_reg(client, QT602240_VARIANT_ID);
	if (val < 0)
		return -EIO;
	info->variant_id = val;

	val = qt602240_read_reg(client, QT602240_VERSION);
	if (val < 0)
		return -EIO;
	info->version = val;

	val = qt602240_read_reg(client, QT602240_BUILD);
	if (val < 0)
		return -EIO;
	info->build = val;

	val = qt602240_read_reg(client, QT602240_MATRIX_X_SIZE);
	if (val < 0)
		return -EIO;
	info->matrix_xsize = val;

	val = qt602240_read_reg(client, QT602240_MATRIX_Y_SIZE);
	if (val < 0)
		return -EIO;
	info->matrix_ysize = val;

	val = qt602240_read_reg(client, QT602240_OBJECT_NUM);
	if (val < 0)
		return -EIO;
	info->object_num = val;

	return 0;
}

static int qt602240_initialize(struct qt602240_data *data)
{
	struct i2c_client *client = data->client;
	struct qt602240_info *info;
	int i;
	int ret;
	u16 reg;
	u8 buf[QT602240_OBJECT_SIZE];
	u8 reportid = 0;

	info = data->info = kzalloc(sizeof(struct qt602240_info), GFP_KERNEL);

	if (!data->info) {
		dev_err(&data->client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	ret = qt602240_info_get(data);
	if (ret < 0)
	{
	    dev_err( &data->client->dev, "[TSP] Failed to get TSP info. \n");
	    return ret;
	}
	else
	{
            printk(KERN_DEBUG "[TSP] F/W version    : %d\n", data->info->version );
            printk(KERN_DEBUG "[TSP] Family  ID     : %d\n", data->info->family_id);
            printk(KERN_DEBUG "[TSP] Variant ID     : %d\n", data->info->variant_id);
            printk(KERN_DEBUG "[TSP] Build number   : %d\n", data->info->build);
            printk(KERN_DEBUG "[TSP] X size         : %d\n", data->info->matrix_xsize);
            printk(KERN_DEBUG "[TSP] Y size         : %d\n", data->info->matrix_ysize);
        }

	data->object_table =
		kzalloc(sizeof(struct qt602240_object) * data->info->object_num,
				GFP_KERNEL);
	data->object_message =
		kzalloc(sizeof(struct qt602240_message), GFP_KERNEL);
	if (!data->object_table || !data->object_message) {
		dev_err(&data->client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* get object table information */
	for (i = 0; i < data->info->object_num; i++) {
		struct qt602240_object *object = data->object_table + i;

		reg = QT602240_OBJECT_START + QT602240_OBJECT_SIZE * i;
		ret = qt602240_read_object_table(client, reg, buf);
		if (ret)
			return ret;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3];
		object->instances = buf[4];
		object->num_report_ids = buf[5];

		if (object->num_report_ids) {
			reportid += object->num_report_ids *
				(object->instances + 1);
			object->max_reportid = reportid;
		}
	}

        /* Init qt602240 reg*/
        qt602240_reg_init(data);

	/* check X/Y matrix size */
	qt602240_check_matrix_size(data);

	/* read dummy message to make high CHG pin */
	do {
		ret = qt602240_read_object(data, QT602240_GEN_MESSAGE, 0);
		if (ret < 0)
			return ret;
	} while (ret != 0xff);


	/* backup to memory */
	qt602240_write_object(data, QT602240_GEN_COMMAND,
			QT602240_COMMAND_BACKUPNV, QT602240_BACKUP_VALUE);

	msleep(100);
	/* soft reset */
	qt602240_write_object(data, QT602240_GEN_COMMAND,
			QT602240_COMMAND_RESET, 1);

        msleep(150);

	printk(KERN_DEBUG "[TSP] Family ID: %d Variant ID: %d Version: %d Build: %d\n",
			info->family_id, info->variant_id, info->version, info->build);

	printk(KERN_DEBUG "[TSP] Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize, info->object_num);

	calibrate_chip(data);

	QT602240_Multitouch_Threshold_High(data);

	#ifdef CONFIG_GENERIC_BLN
		register_bln_implementation(&p1_touchkey_bln);
	#endif

	return 0;
}

static ssize_t qt602240_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct qt602240_data *data = dev_get_drvdata(dev);
	struct qt602240_info *info = data->info;
	int count = 0;

	count += sprintf(buf + count, "Family ID:\t0x%x (%d)\n",
			info->family_id, info->family_id);
	count += sprintf(buf + count, "Variant ID:\t0x%x (%d)\n",
			info->variant_id, info->variant_id);
	count += sprintf(buf + count, "Version:\t0x%x (%d)\n",
			info->version, info->version);
	count += sprintf(buf + count, "Build:\t\t0x%x (%d)\n",
			info->build, info->build);
	count += sprintf(buf + count, "Matrix X Size:\t0x%x (%d)\n",
			info->matrix_xsize, info->matrix_xsize);
	count += sprintf(buf + count, "Matrix Y Size:\t0x%x (%d)\n",
			info->matrix_ysize, info->matrix_ysize);
	count += sprintf(buf + count, "Object Num:\t0x%x (%d)\n",
			info->object_num, info->object_num);

	return count;
}

static ssize_t qt602240_object_table_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct qt602240_data *data = dev_get_drvdata(dev);
	struct qt602240_object *object;
	int count = 0;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		count += sprintf(buf + count,
				"Object Table Element %d\n", i + 1);
		count += sprintf(buf + count, "  type:\t\t\t0x%x (%d)\n",
				object->type, object->type);
		count += sprintf(buf + count, "  start_address:\t0x%x (%d)\n",
				object->start_address, object->start_address);
		count += sprintf(buf + count, "  size:\t\t\t0x%x (%d)\n",
				object->size, object->size);
		count += sprintf(buf + count, "  instances:\t\t0x%x (%d)\n",
				object->instances, object->instances);
		count += sprintf(buf + count, "  num_report_ids:\t0x%x (%d)\n",
				object->num_report_ids, object->num_report_ids);
		count += sprintf(buf + count, "  max_reportid:\t\t0x%x (%d)\n",
				object->max_reportid, object->max_reportid);
		count += sprintf(buf + count, "\n");
	}

	return count;
}

static ssize_t qt602240_object_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct qt602240_data *data = dev_get_drvdata(dev);
	struct qt602240_object *object;
	int val;
	int count = 0;
	int i, j;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		count += sprintf(buf + count,
				"Object Table Element %d(Type %d)\n",
				i + 1, object->type);

		if (!qt602240_object_readable(object->type)) {
			count += sprintf(buf + count, "\n");
			continue;
		}

		for (j = 0; j < object->size + 1; j++) {
			val = qt602240_read_object(data, object->type, j);
			if (val < 0)
				return count;

			count += sprintf(buf + count,
					"  Byte %d: 0x%x (%d)\n", j, val, val);
		}

		count += sprintf(buf + count, "\n");
	}

	return count;
}

static ssize_t qt602240_object_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct qt602240_data *data = dev_get_drvdata(dev);
	struct qt602240_object *object;
	unsigned int type;
	unsigned int offset;
	unsigned int val;
	int ret;

	if (sscanf(buf, "%u %u %u", &type, &offset, &val) != 3) {
		dev_err(dev, "Invalid values\n");
		return -EINVAL;
	}

	object = qt602240_get_object(data, type);
	if (!object || (offset > object->size)) {
		dev_err(dev, "Invalid values\n");
		return -EINVAL;
	}

	ret = qt602240_write_object(data, type, offset, val);
	if (ret < 0)
		return ret;

	return count;
}

static int qt602240_load_fw(struct device *dev, unsigned int version)
{
    struct qt602240_data *data = dev_get_drvdata(dev);
    const struct firmware *fw = NULL;
    unsigned int frame_size;
    unsigned int pos = 0;
    int ret;

    fw = kzalloc(sizeof(struct firmware), GFP_KERNEL);
    if (!fw)
    {
        dev_err(dev, "memory all error\n");
        ret = -ENOMEM;
        goto err_fw;
    }

    if(version == QT602240_VER_22)
    {
        ret = request_firmware(&fw, QT602240_FW_NAME, dev);
        if (ret < 0)
        {
            dev_err(dev, "Unable to open firmware %s\n", QT602240_FW_NAME);
            ret = -EIO;
            goto err_fw;
        }
    }

    /* change to the bootloader mode */
    qt602240_write_object(data, QT602240_GEN_COMMAND,
    		QT602240_COMMAND_RESET, QT602240_BOOT_VALUE);
    msleep(150);

    /* change to slave address of bootloader */
    if (data->client->addr == QT602240_APP_LOW)
    	data->client->addr = QT602240_BOOT_LOW;
    else
    	data->client->addr = QT602240_BOOT_HIGH;

    ret = qt602240_check_bootloader(data->client,
    		QT602240_WAITING_BOOTLOAD_CMD);
    if (ret < 0)
    	goto err_fw;

    /* unlock bootloader */
    qt602240_unlock_bootloader(data->client);
    msleep(100);

    while (pos < fw->size )
    {
    	ret = qt602240_check_bootloader(data->client,
    			QT602240_WAITING_FRAME_DATA);
    	if (ret < 0)
    		goto err_fw;

    	frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

    	/* We should add 2 at frame size as the firmware data is not
    	 * included the CRC bytes.
    	 */
    	frame_size += 2;

    	printk(KERN_DEBUG "[TSP] frame_size : 0x%x\n", frame_size);

    	/* write one frame to device */
    	qt602240_fw_write(data->client, fw->data + pos, frame_size);

    	ret = qt602240_check_bootloader(data->client,
    			QT602240_FRAME_CRC_PASS);
    	if (ret < 0)
    		goto err_fw;
    	else if( ret == 0)
    	    pos += frame_size;

    	dev_info(dev, "Updated %zd bytes / %zd bytes\n", pos, fw->size);
    }

    err_fw:
    /* change to slave address of application */
    if (data->client->addr == QT602240_BOOT_LOW)
    	data->client->addr = QT602240_APP_LOW;
    else
    	data->client->addr = QT602240_APP_HIGH;

    kfree(fw);

    return ret;
}

static ssize_t qt602240_update_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gFirmware_Update_State);
}

static ssize_t qt602240_update_fw_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int version;
    int ret;

    if (sscanf(buf, "%u", &version) != 1) {
    	dev_err(dev, "Invalid values\n");
    	return -EINVAL;
    }

    printk(KERN_DEBUG "[TSP] F/W  downloading...\n");
    gFirmware_Update_State = FW_UPDATE_DOWNLOADING;

    ret = qt602240_load_fw(dev, version);
    if (ret < 0)
    {
        gFirmware_Update_State = FW_UPDATE_FAIL;
    	dev_err(dev, "The firmware update fail\n");
    }
    else
    {
        gFirmware_Update_State = FW_UPDATE_DONE;
    	dev_info(dev, "The firmware update success\n");
    }

    return count;
}

void qt602240_set_amoled_display(int mode)
{
	set_mode_for_amoled = mode;
}
EXPORT_SYMBOL(qt602240_set_amoled_display);

// mode 1 = Charger connected
// mode 0 = Charger disconnected
void qt602240_inform_charger_connection(int mode)
{
    if(mode)
    {
        set_mode_for_ta = true;
    }
    else
    {
        set_mode_for_ta = false;
    }

    if(p_qt602240_data != NULL)
    {
        if (!work_pending(&p_qt602240_data->ta_work))
        {
            schedule_work(&p_qt602240_data->ta_work);
        }
    }

}
EXPORT_SYMBOL(qt602240_inform_charger_connection);

static void qt602240_ta_worker(struct work_struct *work)
{
    struct qt602240_data *data = container_of(work,
    	struct qt602240_data, ta_work);
    int error;

    if(set_mode_for_ta)
    {
        printk(KERN_DEBUG "[TSP] TA is connected.\n");
        touchscreen_config.tchthr = 28;
    }
    else
    {
        printk(KERN_DEBUG "[TSP] TA is disconnected.\n");
        touchscreen_config.tchthr = 25;
    }

    error = qt602240_write_object(data, QT602240_TOUCH_MULTI,
                QT602240_TOUCH_TCHTHR, touchscreen_config.tchthr);
    if (error < 0)
    {
        printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
    }

    calibrate_chip(data);

}

void qt602240_inform_first_brightness(void)
{
    int ret;

    printk(KERN_DEBUG "[TSP] %s\n", __func__);

    if(p_qt602240_data != NULL)
    {
        ret = QT602240_Multitouch_Threshold_Reset(p_qt602240_data);
	    if (ret < 0)
	    {
	        printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
	    }
    }
    else
        printk(KERN_ERR "[TSP] %s qt602240_data is not initialized.\n", __func__);

}
EXPORT_SYMBOL(qt602240_inform_first_brightness);

static DEVICE_ATTR(info, 0444, qt602240_info_show, NULL);
static DEVICE_ATTR(object_table, 0444, qt602240_object_table_show, NULL);
static DEVICE_ATTR(object, 0664, qt602240_object_show, qt602240_object_store);
static DEVICE_ATTR(update_fw, 0664, NULL, qt602240_update_fw_store);
static DEVICE_ATTR(update_status, 0664, qt602240_update_status_show, NULL);

static struct attribute *qt602240_attrs[] = {
	&dev_attr_info.attr,
	&dev_attr_object_table.attr,
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_update_status.attr,
//	&dev_attr_config_mode.attr,
	NULL
};

static const struct attribute_group qt602240_attr_group = {
	.attrs = qt602240_attrs,
};

#ifdef ENABLE_NOISE_TEST_MODE
struct device *qt602240_noise_test;
//botton_right, botton_left, center, top_right, top_left
unsigned char test_node[5] = {201, 193, 113, 21, 13};

static void diagnostic_chip(u8 mode)
{
    int error;
    error = qt602240_write_object(p_qt602240_data, QT602240_GEN_COMMAND,
        	QT602240_COMMAND_DIAGNOSTIC, mode);
    if (error < 0)
    {
        printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
    }
}

static void read_dbg_data(uint8_t dbg_mode , uint8_t node, uint16_t * dbg_data)
{
    u8 read_page, read_point;
    u8 data_buffer[2] = { 0 };
    int i;

    read_page = node / 64;
    node %= 64;
    read_point = (node *2) + 2;

    //Page Num Clear
    diagnostic_chip(QT_CTE_MODE);
    msleep(20);

    diagnostic_chip(dbg_mode);
    msleep(20);

    for(i = 0; i < 5; i++)
    {
        qt602240_read_diagnostic(0, data_buffer, 1);
        if(data_buffer[0] == dbg_mode)
        {
            break;
        }
        msleep(20);
    }
    printk(KERN_DEBUG "[TSP] page clear \n");

    for(i = 1; i <= read_page; i++)
    {
        diagnostic_chip(QT_PAGE_UP);
        msleep(20);
        qt602240_read_diagnostic(1, data_buffer, 1);
        printk(KERN_DEBUG "[TSP] page buffer : %d, i : %d \n", data_buffer[0], i);
        if(data_buffer[0] != i)
        {
            if(data_buffer[0] >= 0x4)
                break;
            i--;
        }
    }

    qt602240_read_diagnostic(read_point, data_buffer, 2);
    *dbg_data= ((uint16_t)data_buffer[1]<<8)+ (uint16_t)data_buffer[0];

}

static ssize_t set_refer0_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_refrence=0;
    read_dbg_data(QT_REFERENCE_MODE, test_node[0],&qt_refrence);
    return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_refer1_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_refrence=0;
    read_dbg_data(QT_REFERENCE_MODE, test_node[1],&qt_refrence);
    return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_refer2_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_refrence=0;
    read_dbg_data(QT_REFERENCE_MODE, test_node[2],&qt_refrence);
    return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_refer3_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_refrence=0;
    read_dbg_data(QT_REFERENCE_MODE, test_node[3],&qt_refrence);
    return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_refer4_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_refrence=0;
    read_dbg_data(QT_REFERENCE_MODE, test_node[4],&qt_refrence);
    return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_delta0_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_delta=0;
    read_dbg_data(QT_DELTA_MODE, test_node[0],&qt_delta);
    if(qt_delta < 32767)
        return sprintf(buf, "%u\n", qt_delta);
    else
        qt_delta = 65535 - qt_delta;

    return sprintf(buf, "-%u\n", qt_delta);
}

static ssize_t set_delta1_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_delta=0;
    read_dbg_data(QT_DELTA_MODE, test_node[1],&qt_delta);
    if(qt_delta < 32767)
        return sprintf(buf, "%u\n", qt_delta);
    else
        qt_delta = 65535 - qt_delta;

    return sprintf(buf, "-%u\n", qt_delta);
}

static ssize_t set_delta2_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_delta=0;
    read_dbg_data(QT_DELTA_MODE, test_node[2],&qt_delta);
    if(qt_delta < 32767)
        return sprintf(buf, "%u\n", qt_delta);
    else
        qt_delta = 65535 - qt_delta;

    return sprintf(buf, "-%u\n", qt_delta);
}

static ssize_t set_delta3_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_delta=0;
    read_dbg_data(QT_DELTA_MODE, test_node[3],&qt_delta);
    if(qt_delta < 32767)
        return sprintf(buf, "%u\n", qt_delta);
    else
        qt_delta = 65535 - qt_delta;

    return sprintf(buf, "-%u\n", qt_delta);
}

static ssize_t set_delta4_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_delta=0;
    read_dbg_data(QT_DELTA_MODE, test_node[4],&qt_delta);
    if(qt_delta < 32767)
        return sprintf(buf, "%u\n", qt_delta);
    else
        qt_delta = 65535 - qt_delta;

    return sprintf(buf, "-%u\n", qt_delta);
}

static ssize_t set_threshold_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", touchscreen_config.tchthr);
}

static DEVICE_ATTR(set_refer0, 0664, set_refer0_mode_show, NULL);
static DEVICE_ATTR(set_delta0, 0664, set_delta0_mode_show, NULL);
static DEVICE_ATTR(set_refer1, 0664, set_refer1_mode_show, NULL);
static DEVICE_ATTR(set_delta1, 0664, set_delta1_mode_show, NULL);
static DEVICE_ATTR(set_refer2, 0664, set_refer2_mode_show, NULL);
static DEVICE_ATTR(set_delta2, 0664, set_delta2_mode_show, NULL);
static DEVICE_ATTR(set_refer3, 0664, set_refer3_mode_show, NULL);
static DEVICE_ATTR(set_delta3, 0664, set_delta3_mode_show, NULL);
static DEVICE_ATTR(set_refer4, 0664, set_refer4_mode_show, NULL);
static DEVICE_ATTR(set_delta4, 0664, set_delta4_mode_show, NULL);
static DEVICE_ATTR(set_threshould, 0664, set_threshold_mode_show, NULL);
#endif /* ENABLE_NOISE_TEST_MODE */

extern int mdnie_cmc623_tuning_load_from_file(u32* mdnie, u32* cmc623);

static ssize_t firmware1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if !defined(MDNIE_TUNING)
	u8 version = p_qt602240_data->info->version;
	u8 build = p_qt602240_data->info->build;
	printk(KERN_DEBUG "[TSP] Firmware %x %x\n", version, build);

	return sprintf(buf, "%x.%x\n", version>>4, version&0xf);
#else
	int ret;
	u32 a, b;

	ret = mdnie_cmc623_tuning_load_from_file(&a, &b);
	printk(KERN_DEBUG "[TSP] ret = %d, a = %d, b = %d\n", ret, a, b);

	if(ret<0)
		return sprintf(buf, "FAIL\n");
	else
		return sprintf(buf, "OK (%d+%d data)\n", a, b);
#endif
}

static ssize_t firmware1_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static ssize_t key_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", touchscreen_config.tchthr);
}

static ssize_t key_threshold_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
    int i, ret =0;
    if(sscanf(buf,"%d",&i)==1)
    {
        touchscreen_config.tchthr = i;
        printk(KERN_DEBUG "[TSP] threshold is changed to %d\n",i);
    }
    else
        printk(KERN_DEBUG "[TSP] threshold write error\n");

    ret = qt602240_write_object(p_qt602240_data, QT602240_TOUCH_MULTI,
	            QT602240_TOUCH_TCHTHR, i);
    if (ret < 0)
    {
    	printk(KERN_DEBUG "[TSP] error %s: write_object\n", __func__);
    }

    return size;
}

static DEVICE_ATTR(firmware1, S_IRUGO | S_IWUSR, firmware1_show, firmware1_store);
static DEVICE_ATTR(key_threshold, S_IRUGO | S_IWUSR, key_threshold_show, key_threshold_store);

#if defined (CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
static ssize_t tsp_filter_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
    int i, ret =0;
    if(sscanf(buf,"%d",&i)==1)
    {
        if( i == 0x1)
        {
            gbfilter = true;
        }
        else
        {
            gbfilter = false;
        }
    }
    else
        printk(KERN_DEBUG "[TSP] threshold write error\n");

    return size;
}
//static DEVICE_ATTR(tsp_filter, S_IRUGO | S_IWUSR, NULL, tsp_filter_store);
static DEVICE_ATTR(tsp_filter, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, NULL, tsp_filter_store);
#endif

static int __devinit qt602240_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
    struct qt602240_data *data;
    struct input_dev *input_dev;
    struct device *ts_dev;
#if defined (KEY_LED_CONTROL)
    struct class *leds_class;
    struct device *led_dev;
#endif      //KEY_LED_CONTROL

    int ret;
    int i;

    data = kzalloc(sizeof(struct qt602240_data), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (!data || !input_dev) {
    	dev_err(&client->dev, "Failed to allocate memory\n");
    	ret = -ENOMEM;
    	goto err_free_mem;
    }

    client->irq = IRQ_TOUCH_INT;

    INIT_WORK(&data->ta_work, qt602240_ta_worker);

    data->client = client;
    data->input_dev = input_dev;
    data->irq = client->irq;

    input_dev->name = "AT42QT602240 Touchscreen";
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &client->dev;

    set_bit(EV_SYN, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(BTN_TOUCH, input_dev->keybit);
    set_bit(EV_ABS, input_dev->evbit);

#ifdef CONFIG_TOUCHSCREEN_QT602240_ROT90
    input_set_abs_params(input_dev, ABS_MT_POSITION_X,
            0, QT602240_MAX_YC, 0, 0);

    input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
            0, QT602240_MAX_XC, 0, 0);
#else
    input_set_abs_params(input_dev, ABS_MT_POSITION_X,
            0, QT602240_MAX_XC, 0, 0);

    input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
            0, QT602240_MAX_YC, 0, 0);
#endif
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
            0, QT602240_MAX_PRESSURE, 0, 0);

    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,
            0, QT602240_MAX_SIZE, 0, 0);

    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
            0, QT602240_MAX_ID, 0, 0);

    input_set_abs_params(input_dev, ABS_PRESSURE,
            0, QT602240_MAX_PRESSURE, 0, 0);

    input_dev->keycode = (void*)tsp_keycodes;

    for(i = 0; i < NUMOFKEYS; i++)
    {
        input_set_capability(input_dev, EV_KEY, ((int*)input_dev->keycode)[i]);
        tsp_keystatus[i] = KEY_RELEASE;
    }

    input_set_drvdata(input_dev, data);

    p_qt602240_data = data;

    /* for multi-touch */
    for (i=0; i<MAX_USING_FINGER_NUM ; i++)		// touchscreen_config.numtouch is 10
    {
        fingerInfo[i].pressure = -1;
    }

    ret = input_register_device(input_dev);
    if (ret < 0)
    {
    	goto err_free_irq;
    }

    ret = qt602240_initialize(data);
    if (ret < 0)
    {
	goto err_free_object;
    }

    ret = request_threaded_irq(data->irq, NULL, qt602240_interrupt,
            IRQF_ONESHOT|IRQF_TRIGGER_LOW, client->dev.driver->name, data);

    if (ret < 0)
    {
    	dev_err(&client->dev, "Failed to register interrupt\n");
    	goto err_free_object;
    }

    ret = sysfs_create_group(&client->dev.kobj, &qt602240_attr_group);
    if (ret)
    {
    	goto err_free_irq;
    }

    i2c_set_clientdata(client, data);

    ts_dev = device_create(sec_class, NULL, 0, NULL, "ts");
    if (IS_ERR(ts_dev))
    {
    	pr_err("Failed to create device(ts)!\n");
    }

    if (device_create_file(ts_dev, &dev_attr_firmware1) < 0)
    {
    	pr_err("Failed to create device file(%s)!\n", dev_attr_firmware1.attr.name);
    }

    if (device_create_file(ts_dev, &dev_attr_key_threshold) < 0)
    {
    	pr_err("Failed to create device file(%s)!\n", dev_attr_key_threshold.attr.name);
    }

#if defined (CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
    if (device_create_file(ts_dev, &dev_attr_tsp_filter) < 0)
    {
        pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_filter.attr.name);
    }
#endif

#ifdef ENABLE_NOISE_TEST_MODE
    qt602240_noise_test = device_create(sec_class, NULL, 0, NULL, "qt602240_noise_test");

    if (IS_ERR(qt602240_noise_test))
    {
    	printk(KERN_ERR "Failed to create device(qt602240_noise_test)!\n");
    }

    if (device_create_file(qt602240_noise_test, &dev_attr_set_refer0)< 0)
    {
    	printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer0.attr.name);
    }

    if (device_create_file(qt602240_noise_test, &dev_attr_set_delta0) < 0)
    {
    	printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta0.attr.name);
    }

    if (device_create_file(qt602240_noise_test, &dev_attr_set_refer1)< 0)
    {
    	printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer1.attr.name);
    }

    if (device_create_file(qt602240_noise_test, &dev_attr_set_delta1) < 0)
    {
    	printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta1.attr.name);
    }

    if (device_create_file(qt602240_noise_test, &dev_attr_set_refer2)< 0)
    {
    	printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer2.attr.name);
    }

    if (device_create_file(qt602240_noise_test, &dev_attr_set_delta2) < 0)
    {
    	printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta2.attr.name);
    }

    if (device_create_file(qt602240_noise_test, &dev_attr_set_refer3)< 0)
    {
    	printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer3.attr.name);
    }

    if (device_create_file(qt602240_noise_test, &dev_attr_set_delta3) < 0)
    {
    	printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta3.attr.name);
    }

    if (device_create_file(qt602240_noise_test, &dev_attr_set_refer4)< 0)
    {
    	printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer4.attr.name);
    }

    if (device_create_file(qt602240_noise_test, &dev_attr_set_delta4) < 0)
    {
    	printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta4.attr.name);
    }

    if (device_create_file(qt602240_noise_test, &dev_attr_set_threshould) < 0)
    {
    	printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_threshould.attr.name);
    }

#endif

#if defined (KEY_LED_CONTROL)
    init_led();

#if defined(KEY_LED_SELF)
    touch_led_on(255);
#endif
#if defined(LED_SWITCH)
    led_sw= 1;
#endif
	
    leds_class = class_create(THIS_MODULE, "leds");
    if (IS_ERR(leds_class))
    {
        return PTR_ERR(leds_class);
    }

    led_dev = device_create(leds_class, NULL, 0, NULL, "button-backlight");

    if (device_create_file(led_dev, &dev_attr_brightness) < 0)
    {
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }
#endif		//KEY_LED_CONTROL

#ifdef CONFIG_HAS_EARLYSUSPEND
    data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    data->early_suspend.suspend = qt602240_early_suspend;
    data->early_suspend.resume = qt602240_late_resume;
    register_early_suspend(&data->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

    return 0;

    err_free_irq:
    free_irq(client->irq, data);
    err_free_object:
    kfree(data->object_message);
    kfree(data->object_table);
    kfree(data->info);
    err_free_mem:
    input_free_device(input_dev);
    kfree(data);
    return ret;
}

static int __devexit qt602240_remove(struct i2c_client *client)
{
	struct qt602240_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
	kfree(data->object_message);
	kfree(data->object_table);
	kfree(data->info);
	kfree(data);

	sysfs_remove_group(&client->dev.kobj, &qt602240_attr_group);
	i2c_set_clientdata(client, NULL);

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void qt602240_early_suspend(struct early_suspend *early_sus)
{
	struct qt602240_data *data = container_of(early_sus,
	    	struct qt602240_data, early_suspend);
#else
static int qt602240_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct qt602240_data *data = i2c_get_clientdata(client);
#endif
    disable_irq_nosync(data->irq);

    /* touch disable */
    qt602240_write_object(data, QT602240_TOUCH_MULTI,
            QT602240_TOUCH_CTRL, 0);

    qt602240_write_object(data, QT602240_GEN_POWER,
            QT602240_POWER_IDLEACQINT, 0);

    qt602240_write_object(data, QT602240_GEN_POWER,
            QT602240_POWER_ACTVACQINT, 0);

    /* for multi-touch */
    release_all_fingers(data->input_dev);
    release_all_keys(data->input_dev);

#if defined (KEY_LED_CONTROL)
    touch_led_on(false);
#endif      //KEY_LED_CONTROL

    qt_timer_state = 0;

}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void qt602240_late_resume(struct early_suspend *early_sus)
{
	struct qt602240_data *data = container_of(early_sus,
	    	struct qt602240_data, early_suspend);
#else
static int qt602240_resume(struct i2c_client *client)
{
    struct qt602240_data *data = i2c_get_clientdata(client);
#endif
	int ret = 0;

    printk(KERN_DEBUG "[TSP] %s\n", __func__);

    while(1)
    {
        /* soft reset */
        ret = qt602240_write_object(data, QT602240_GEN_COMMAND,
            	QT602240_COMMAND_RESET, 1);

        /* wait for soft reset */
        msleep(150);
        if (ret < 0)
        {
            printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
        }
        else
        {
            break;
        }
    }

    if (!gpio_get_value(GPIO_TOUCH_INT)) {
    	printk(KERN_INFO "[TSP] touch_int is low\n");
        /* read dummy message to make high CHG pin */
	do {
		ret = qt602240_read_object(data, QT602240_GEN_MESSAGE, 0);
		if (ret < 0) {
			printk(KERN_ERR "[TSP] %s read fail:%d\n", __func__, ret);
			break; //return ;
		}
	} while (ret != 0xff);
    }
    calibrate_chip(data);

#if defined (LED_SWITCH)
    if (led_sw == 1){
#endif
#if defined (KEY_LED_CONTROL)
        init_led();
#if defined(KEY_LED_SELF)
    touch_led_on(255);
#endif
#endif      //KEY_LED_CONTROL
#if defined (LED_SWITCH)
    }
#endif
    enable_irq(data->irq);

}

static const struct i2c_device_id qt602240_id[] = {
	{ "qt602240_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, qt602240_id);

static struct i2c_driver qt602240_driver = {
	.driver = {
		.name = "qt602240_ts",
	},
	.probe		= qt602240_probe,
	.remove		= __devexit_p(qt602240_remove),
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend	= qt602240_suspend,
	.resume		= qt602240_resume,
#endif
	.id_table	= qt602240_id,
};

static int __init qt602240_init(void)
{
	return i2c_add_driver(&qt602240_driver);
}

static void __exit qt602240_exit(void)
{
	i2c_del_driver(&qt602240_driver);
}

module_init(qt602240_init);
module_exit(qt602240_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("AT42QT602240 Touchscreen driver");
MODULE_LICENSE("GPL");
