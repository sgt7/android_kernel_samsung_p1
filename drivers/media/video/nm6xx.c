/*
 * Driver for NM6XX ( Analog TV Tunner ) from Newport Media
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-i2c-drv.h>
#include <media/nm6xx_platform.h>

#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif

#include <linux/rtc.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-p1.h>
#include <mach/regs-clock.h>
#include <plat/regs-fimc.h> 

#include "nm6xx.h"

#define NM6XX_DRIVER_NAME	"NM6XX"

#define FORMAT_FLAGS_COMPRESSED		    0x3
#define SENSOR_JPEG_SNAPSHOT_MEMSIZE	0x33F000     //3403776 //2216 * 1536

//#define NM6XX_DEBUG
//#define NM6XX_INFO
//#define CONFIG_LOAD_FILE	//For tunning binary

#ifdef NM6XX_DEBUG
#define nm6xx_msg	dev_err
#else
#define nm6xx_msg 	dev_dbg
#endif

#ifdef NM6XX_INFO
#define nm6xx_info	dev_err
#else
#define nm6xx_info	dev_dbg
#endif

/* protect s_ctrl calls */
static DEFINE_MUTEX(sensor_s_ctrl);

/* stop_af_operation is used to cancel the operation while doing, or even before it has started */
static volatile int stop_af_operation;

/* Here we store the status of AF; 0 --> AF not started, 1 --> AF started , 2 --> AF operation finished */
static int af_operation_status;

/*
 * Whenever there is an AF cancell request the timer is started. The cancel operation
 * is valid for 100ms after that it is expired
 */
static struct timer_list af_cancel_timer;
static void af_cancel_handler(unsigned long data);

/* Save the focus mode value, it can be marco or auto */
static int af_mode;

#define CDBG(format, arg...) if (cdbg == 1) { printk("<NM6XX> %s " format, __func__, ## arg); }

static int cdbg = 0;
static int gLowLight = 0;
static int gCurrentScene = SCENE_MODE_NONE;

/* Default resolution & pixelformat. plz ref nm6xx_platform.h */
#define DEFAULT_PIX_FMT		V4L2_PIX_FMT_UYVY	/* YUV422 */
#define DEFUALT_MCLK		24000000
#define POLL_TIME_MS		10

enum nm6xx_oprmode {
	NM6XX_OPRMODE_VIDEO = 0,
	NM6XX_OPRMODE_IMAGE = 1,
};

enum nm6xx_frame_size {
	NM6XX_PREVIEW_QCIF = 0,	/* 176x144 */
	NM6XX_PREVIEW_QVGA,		/* 320x240 */ //johnny.kim
	NM6XX_PREVIEW_D1,		/* 720x480 */
	NM6XX_PREVIEW_SVGA,		/* 800x600 */
	NM6XX_PREVIEW_WSVGA,		/* 1024x600*/
	NM6XX_CAPTURE_SVGA,		/* SVGA  - 800x600 */
	NM6XX_CAPTURE_WSVGA,		/* SVGA  - 1024x600 */
	NM6XX_CAPTURE_W1MP,		/* WUXGA  - 1600x960 */
	NM6XX_CAPTURE_2MP,		/* UXGA  - 1600x1200 */
	NM6XX_CAPTURE_W2MP,		/* WQXGA  - 2048x1232 */
	NM6XX_CAPTURE_3MP,		/* QXGA  - 2048x1536 */
};

struct nm6xx_enum_framesize {
	/* mode is 0 for preview, 1 for capture */
	enum nm6xx_oprmode mode;
	unsigned int index;
	unsigned int width;
	unsigned int height;	
};

static struct nm6xx_enum_framesize nm6xx_framesize_list[] = {
	{ NM6XX_OPRMODE_VIDEO, NM6XX_PREVIEW_QCIF,	176,  144 },
	{ NM6XX_OPRMODE_VIDEO, NM6XX_PREVIEW_QVGA,	320,  240 },
};

struct nm6xx_version {
	unsigned int major;
	unsigned int minor;
};

struct nm6xx_date_info {
	unsigned int year;
	unsigned int month;
	unsigned int date;
};

enum nm6xx_runmode {
	NM6XX_RUNMODE_NOTREADY,
	NM6XX_RUNMODE_IDLE, 
	NM6XX_RUNMODE_RUNNING, 
};

struct nm6xx_firmware {
	unsigned int addr;
	unsigned int size;
};

/* Camera functional setting values configured by user concept */
struct nm6xx_userset {
	signed int exposure_bias;	/* V4L2_CID_EXPOSURE */
	unsigned int auto_wb;		/* V4L2_CID_AUTO_WHITE_BALANCE */
	unsigned int manual_wb;		/* V4L2_CID_WHITE_BALANCE_PRESET */
	unsigned int effect;		/* Color FX (AKA Color tone) */
	unsigned int contrast;		/* V4L2_CID_CONTRAST */
	unsigned int saturation;	/* V4L2_CID_SATURATION */
	unsigned int sharpness;		/* V4L2_CID_SHARPNESS */
};

struct nm6xx_jpeg_param {
	unsigned int enable;
	unsigned int quality;
	unsigned int main_size;  /* Main JPEG file size */
	unsigned int thumb_size; /* Thumbnail file size */
	unsigned int main_offset;
	unsigned int thumb_offset;
	unsigned int postview_offset;
} ; 

struct nm6xx_position {
	int x;
	int y;
} ; 

struct nm6xx_state {
	struct nm6xx_platform_data *pdata;
	struct v4l2_subdev sd;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	struct nm6xx_userset userset;
	struct nm6xx_jpeg_param jpeg;
	struct nm6xx_version fw;
	struct nm6xx_version prm;
	struct nm6xx_date_info dateinfo;
	struct nm6xx_firmware fw_info;
	struct nm6xx_position position;
	struct v4l2_streamparm strm;
	enum nm6xx_runmode runmode;
	enum nm6xx_oprmode oprmode;
	int framesize_index;
	int sensor_version;
	int freq;	/* MCLK in Hz */
	int fps;
	int preview_size;
};

const static struct v4l2_fmtdesc capture_fmts[] = {
	{
		.index          = 0,
		.type           = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.flags          = FORMAT_FLAGS_COMPRESSED,
		.description    = "JPEG + Postview",
		.pixelformat    = V4L2_PIX_FMT_JPEG,
	},
};

extern int nm6xx_cam_stdby(bool en);

#ifdef CONFIG_LOAD_FILE
static int nm6xx_regs_table_write(struct i2c_client *client, char *name);
#endif

static inline struct nm6xx_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct nm6xx_state, sd);
}

/**
 * nm6xx_i2c_read: Read 2 bytes from sensor 
 */
static inline int nm6xx_i2c_read(struct i2c_client *client, 
	unsigned short subaddr, unsigned short *data)
{
	unsigned char buf[2];
	int err = 0;
	struct i2c_msg msg = {client->addr, 0, 2, buf};

	buf[0] = subaddr>> 8;
	buf[1] = subaddr & 0xff;
	
	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
		nm6xx_msg(&client->dev, "%s: register read fail\n", __func__);	

	msg.flags = I2C_M_RD;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
		nm6xx_msg(&client->dev, "%s: register read fail\n", __func__);	

	/*
	 * [Arun c]Data comes in Little Endian in parallel mode; So there
	 * is no need for byte swapping here
	 */
	*data = *(unsigned short *)(&buf);
		
	return err;
}

/** 
 * nm6xx_i2c_read_multi: Read (I2C) multiple bytes to the camera sensor 
 * @client: pointer to i2c_client
 * @cmd: command register
 * @w_data: data to be written
 * @w_len: length of data to be written
 * @r_data: buffer where data is read
 * @r_len: number of bytes to read
 *
 * Returns 0 on success, <0 on error
 */
static inline int nm6xx_i2c_read_multi(struct i2c_client *client,  
	unsigned short subaddr, unsigned long *data)
{
	unsigned char buf[4];
	int err = 0;
	struct i2c_msg msg = {client->addr, 0, 2, buf};

	buf[0] = subaddr>> 8;
	buf[1] = subaddr & 0xff;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
		nm6xx_msg(&client->dev, "%s: register read fail\n", __func__);	

	msg.flags = I2C_M_RD;
	msg.len = 4;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
		nm6xx_msg(&client->dev, "%s: register read fail\n", __func__);	

	/*
	 * [Arun c]Data comes in Little Endian in parallel mode; So there
	 * is no need for byte swapping here
	 */
	*data = *(unsigned long *)(&buf);

	return err;
}

/** 
 * nm6xx_i2c_write_multi: Write (I2C) multiple bytes to the camera sensor 
 * @client: pointer to i2c_client
 * @cmd: command register
 * @w_data: data to be written
 * @w_len: length of data to be written
 *
 * Returns 0 on success, <0 on error
 */
static inline int nm6xx_i2c_write_multi(struct i2c_client *client, unsigned short addr, unsigned int w_data, unsigned int w_len)
{
	int retry_count = 5;
	unsigned char buf[w_len+2];
	struct i2c_msg msg = {client->addr, 0, w_len+2, buf};
	int ret;

	buf[0] = addr >> 8;
	buf[1] = addr & 0xff;	

	/* 
	 * [Arun c]Data should be written in Little Endian in parallel mode; So there
	 * is no need for byte swapping here
	 */
	if(w_len == 1)
		buf[2] = (unsigned char)w_data;
	else if(w_len == 2)
		*((unsigned short *)&buf[2]) = (unsigned short)w_data;
	else
		*((unsigned int *)&buf[2]) = w_data;

#ifdef NM6XX_DEBUG
	{
		int j;
		printk("W: ");
		for(j = 0; j <= w_len+1; j++){
			printk("0x%02x ", buf[j]);
		}
		printk("\n");
	}
#endif

	while(retry_count--){
		ret  = i2c_transfer(client->adapter, &msg, 1);
		if (likely(ret == 1))
			break;
		msleep(POLL_TIME_MS);
		}

	return (ret == 1) ? 0 : -EIO;
}

static int nm6xx_write_regs(struct v4l2_subdev *sd, nm6xx_short_t regs[], 
				int size, char *name)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i, err;

#ifdef CONFIG_LOAD_FILE
	nm6xx_regs_table_write(client, name);
#else
	for (i = 0; i < size; i++) {
		err = nm6xx_i2c_write_multi(client, regs[i].subaddr, regs[i].value, regs[i].len);
		if (unlikely(err < 0)) {
			v4l_info(client, "%s: register set failed\n",  __func__);
			return err;
		}
	}
#endif
	return 0;
}

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

static char *nm6xx_regs_table = NULL;

static int nm6xx_regs_table_size;

int nm6xx_regs_table_init(void)
{
	printk("[BestIQ] + nm6xx_regs_table_init\n");
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int i;
	int ret;
	mm_segment_t fs = get_fs();

	printk("%s %d\n", __func__, __LINE__);

	set_fs(get_ds());
#if 0
	filp = filp_open("/data/camera/nm6xx.h", O_RDONLY, 0);
#else
	filp = filp_open("/mnt/internal_sd/external_sd/nm6xx.h", O_RDONLY, 0);
#endif

	if (IS_ERR(filp)) {
		printk("file open error\n");
		return PTR_ERR(filp);
	}
	
	l = filp->f_path.dentry->d_inode->i_size;	
	printk("l = %ld\n", l);
	dp = kmalloc(l, GFP_KERNEL);
//	dp = vmalloc(l);	
	if (dp == NULL) {
		printk("Out of Memory\n");
		filp_close(filp, current->files);
	}
	
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	
	if (ret != l) {
		printk("Failed to read file ret = %d\n", ret);
//		kfree(dp);
		vfree(dp);
		filp_close(filp, current->files);
		return -EINVAL;
	}

	filp_close(filp, current->files);
		
	set_fs(fs);
	
	nm6xx_regs_table = dp;
		
	nm6xx_regs_table_size = l;
	
	*((nm6xx_regs_table + nm6xx_regs_table_size) - 1) = '\0';
	
	printk("nm6xx_regs_table 0x%08x, %ld\n", dp, l);
	printk("[BestIQ] - nm6xx_reg_table_init\n");

	return 0;
}

void nm6xx_regs_table_exit(void)
{
	printk("[BestIQ] + nm6xx_regs_table_exit\n");
	printk("%s %d\n", __func__, __LINE__);
	if (nm6xx_regs_table) {
		kfree(nm6xx_regs_table);
		nm6xx_regs_table = NULL;
	}
	printk("[BestIQ] - nm6xx_regs_table_exit\n");
}

static int nm6xx_regs_table_write(struct i2c_client *client, char *name)
{
	printk("[BestIQ] + nm6xx_regs_table_write\n");
	char *start, *end, *reg, *data;	
	unsigned short addr;
	unsigned int len, value;	
	char reg_buf[7], data_buf[7], len_buf[2];
	
	*(reg_buf + 6) = '\0';
	*(data_buf + 6) = '\0';
	*(len_buf + 1) = '\0';	

//	printk("[BestIQ] + nm6xx_regs_table_write ------- start\n");
	start = strstr(nm6xx_regs_table, name);
	end = strstr(start, "};");
	
	while (1) {	
		/* Find Address */	
		reg = strstr(start,"{0x");		
		if (reg)
			start = (reg + 19);  //{0x000b, 0x0004, 1},	
		if ((reg == NULL) || (reg > end))
			break;
		/* Write Value to Address */	
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 1), 6);	
			memcpy(data_buf, (reg + 9), 6);	
			memcpy(len_buf, (reg + 17), 1);			
			addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
			value = (unsigned int)simple_strtoul(data_buf, NULL, 16); 
			len = (unsigned int)simple_strtoul(len_buf, NULL, 10); 			
//			printk("addr 0x%04x, value 0x%04x, len %d\n", addr, value, len);
			
			if (addr == 0xdddd)
			{
/*				if (value == 0x0010)
				mdelay(10);
				else if (value == 0x0020)
				mdelay(20);
				else if (value == 0x0030)
				mdelay(30);
				else if (value == 0x0040)
				mdelay(40);
				else if (value == 0x0050)
				mdelay(50);
				else if (value == 0x0100)
				mdelay(100);*/
				mdelay(value);
				printk("delay 0x%04x, value 0x%04x, , len 0x%01x\n", addr, value, len);
			}	
			else
				nm6xx_i2c_write_multi(client, addr, value, len);
		}
	}
	printk("[BestIQ] - nm6xx_regs_table_write\n");
	return 0;
}

static short nm6xx_regs_max_value(char *name)
{
	printk("[BestIQ] + nm6xx_regs_max_value\n");
	char *start, *reg, *data;	
	unsigned short value;
	char data_buf[7];

	*(data_buf + 6) = '\0';

	start = strstr(nm6xx_regs_table, name);

	/* Find Address */	
	reg = strstr(start," 0x");		
	if (reg == NULL)
		return 0;
	/* Write Value to Address */	
	if (reg != NULL) {
		memcpy(data_buf, (reg + 1), 6);	
		value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 
	}
	printk("[BestIQ] - nm6xx_regs_max_value\n");
	return value;
}

#endif

static int nm6xx_set_preview_stop(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct nm6xx_state *state = to_state(sd);

	if(NM6XX_RUNMODE_RUNNING == state->runmode){
		state->runmode = NM6XX_RUNMODE_IDLE;
	}

	dev_err(&client->dev, "%s: change preview mode~~~~~~~~~~~~~~\n", __func__);		
//bestiq	nm6xx_i2c_write_multi(client, 0x0011, 0x0011, 1);
	return 0;
}

static int nm6xx_set_dzoom(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	nm6xx_msg(&client->dev, "[BestIQ] - nm6xx_set_dzoom~~~~~~ %d\n", ctrl->value);

	switch (ctrl->value) {
		case 0:
	case 20:
	case 30:
	default:
		err = nm6xx_write_regs(sd, nm6xx_Zoom_00, sizeof(nm6xx_Zoom_00) / sizeof(nm6xx_Zoom_00[0]),
				"nm6xx_Zoom_00");
		break;
		case 1:
		err = nm6xx_write_regs(sd, nm6xx_Zoom_01, sizeof(nm6xx_Zoom_01) / sizeof(nm6xx_Zoom_01[0]),
				"nm6xx_Zoom_01");
		break;
		case 2:
		err = nm6xx_write_regs(sd, nm6xx_Zoom_02, sizeof(nm6xx_Zoom_02) / sizeof(nm6xx_Zoom_02[0]),
				"nm6xx_Zoom_02");
		break;
		case 3:
		err = nm6xx_write_regs(sd, nm6xx_Zoom_03, sizeof(nm6xx_Zoom_03) / sizeof(nm6xx_Zoom_03[0]),
				"nm6xx_Zoom_03");
		break;
		case 4:
		err = nm6xx_write_regs(sd, nm6xx_Zoom_04, sizeof(nm6xx_Zoom_04) / sizeof(nm6xx_Zoom_04[0]),
				"nm6xx_Zoom_04");
		break;
		case 5:
		err = nm6xx_write_regs(sd, nm6xx_Zoom_05, sizeof(nm6xx_Zoom_05) / sizeof(nm6xx_Zoom_05[0]),
				"nm6xx_Zoom_05");
		break;
		case 6:
		err = nm6xx_write_regs(sd, nm6xx_Zoom_06, sizeof(nm6xx_Zoom_06) / sizeof(nm6xx_Zoom_06[0]),
				"nm6xx_Zoom_06");
		break;
		case 7:
		err = nm6xx_write_regs(sd, nm6xx_Zoom_07, sizeof(nm6xx_Zoom_07) / sizeof(nm6xx_Zoom_07[0]),
				"nm6xx_Zoom_07");
		break;
	}

		if(err < 0){
		dev_err(&client->dev, "%s: i2c_write failed\n", __func__);
		return -EIO;
	}
	return 0;
}

static int nm6xx_set_preview_size(struct v4l2_subdev *sd)
{
	int err = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct nm6xx_state *state = to_state(sd);
	int index = state->framesize_index;

	dev_err(&client->dev, "[zzangdol] %s: index = %d\n", __func__, index);

	switch(index){
	case NM6XX_PREVIEW_QCIF:
		err = nm6xx_i2c_write_multi(client, 0x0022, 0x00B0, 2);// HSIZE_MONI - 176
		err = nm6xx_i2c_write_multi(client, 0x0028, 0x0090, 2);// VSIZE_MONI - 144		
		break;
	case NM6XX_PREVIEW_D1:
		err = nm6xx_i2c_write_multi(client, 0x0022, 0x02D0, 2);// HSIZE_MONI - 720
		err = nm6xx_i2c_write_multi(client, 0x0028, 0x01E0, 2);// VSIZE_MONI - 480	
		break;
	case NM6XX_PREVIEW_SVGA:
		err = nm6xx_i2c_write_multi(client, 0x0022, 0x0320, 2);// HSIZE_MONI - 800
		err = nm6xx_i2c_write_multi(client, 0x0028, 0x0258, 2);// VSIZE_MONI - 600
		break;
	case NM6XX_PREVIEW_WSVGA:
		err = nm6xx_i2c_write_multi(client, 0x0022, 0x0400, 2);// HSIZE_MONI - 1024
		err = nm6xx_i2c_write_multi(client, 0x0028, 0x0258, 2);// VSIZE_MONI - 600
		break;
	default:
		/* When running in image capture mode, the call comes here.
 		 * Set the default video resolution - CE147_PREVIEW_VGA
 		 */ 
		nm6xx_msg(&client->dev, "Setting preview resoution as VGA for image capture mode\n");
		break;
	}

	state->preview_size = index; 

	err = nm6xx_i2c_write_multi(client, 0x0012, 0x0001, 1); //Moni_Refresh
	err = nm6xx_i2c_write_multi(client, 0x00FC, 0x0002, 1); //Clear Interrupt Status (CM_Change)
	msleep(5);
	
	nm6xx_msg(&client->dev, "Using HD preview\n");

	return err;	
}

static int nm6xx_set_preview_start(struct v4l2_subdev *sd)
{
	int err, timeout_cnt;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct nm6xx_state *state = to_state(sd);
	unsigned short read_value;

	/* Reset the AF check variables for the next sequence */
	stop_af_operation = 0;
	af_operation_status = 0;

	if (!state->pix.width || !state->pix.height || !state->fps)
		return -EINVAL;


	err = nm6xx_write_regs(sd, nm6xx_cap_to_prev, sizeof(nm6xx_cap_to_prev) / sizeof(nm6xx_cap_to_prev[0]),
			"nm6xx_cap_to_prev");

	nm6xx_i2c_read(client, 0x0004, &read_value);
	if((read_value & 0x03) != 0) {
		nm6xx_i2c_write_multi(client, 0x0011, 0x0000, 1);	//MODE_SEL  0x00: Monitor mode

		/* Wait for Mode Transition (CM) */
		timeout_cnt = 0;
		do {
			timeout_cnt++;
			nm6xx_i2c_read(client, 0x00F8, &read_value);
			msleep(1);
			if (timeout_cnt > 1000) {
				dev_err(&client->dev, "%s: Entering moniter mode timed out \n", __func__);	
				break;
			}
		}while(!(read_value&0x02));

		timeout_cnt = 0;
		do {
			timeout_cnt++;
			nm6xx_i2c_write_multi(client, 0x00FC, 0x0002, 1);		
			msleep(1);			
			nm6xx_i2c_read(client, 0x00F8, &read_value);
			if (timeout_cnt > 1000) {
				dev_err(&client->dev, "%s: Entering moniter mode timed out \n", __func__);	
				break;
			}
		} while(read_value&0x02);	

	}
		
	err = nm6xx_set_preview_size(sd);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: Could not set preview size\n", __func__);
	       return -EIO;
	}

	state->runmode = NM6XX_RUNMODE_RUNNING;

	msleep(200);
	
	dev_err(&client->dev, "%s: init setting~~~~~~~~~~~~~~\n", __func__);		
	return 0;
}

static int nm6xx_set_capture_size(struct v4l2_subdev *sd)
{
	int err = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct nm6xx_state *state = to_state(sd);

	int index = state->framesize_index;
	printk("nm6xx_set_capture_size ---------index : %d\n", index);	

	switch(index){
	case NM6XX_CAPTURE_SVGA: /* 800x600 */
		err = nm6xx_i2c_write_multi(client, 0x0386, 0x0140, 2);// HSIZE_TN - 320
		err = nm6xx_i2c_write_multi(client, 0x0024, 0x0320, 2); //HSIZE_CAP 800
		err = nm6xx_i2c_write_multi(client, 0x002A, 0x0258, 2); //VSIZE_CAP 600
		break;
	case NM6XX_CAPTURE_WSVGA: /* 1024x600 */
		err = nm6xx_i2c_write_multi(client, 0x0386, 0x0190, 2);// HSIZE_TN - 400
		err = nm6xx_i2c_write_multi(client, 0x0024, 0x0400, 2); //HSIZE_CAP 1024
		err = nm6xx_i2c_write_multi(client, 0x002A, 0x0258, 2); //VSIZE_CAP 600
		break;		
	case NM6XX_CAPTURE_W1MP: /* 1600x960 */
		err = nm6xx_i2c_write_multi(client, 0x0386, 0x0190, 2);// HSIZE_TN - 400
		err = nm6xx_i2c_write_multi(client, 0x0024, 0x0640, 2); //HSIZE_CAP 1600
		err = nm6xx_i2c_write_multi(client, 0x002A, 0x03C0, 2); //VSIZE_CAP 960
		break;
	case NM6XX_CAPTURE_2MP: /* 1600x1200 */
		err = nm6xx_i2c_write_multi(client, 0x0386, 0x0140, 2);// HSIZE_TN - 320
		err = nm6xx_i2c_write_multi(client, 0x0024, 0x0640, 2); //HSIZE_CAP 1600
		err = nm6xx_i2c_write_multi(client, 0x002A, 0x04B0, 2); //VSIZE_CAP 1200
		break;
	case NM6XX_CAPTURE_W2MP: /* 2048x1232 */
		err = nm6xx_i2c_write_multi(client, 0x0386, 0x0190, 2);// HSIZE_TN - 400
		err = nm6xx_i2c_write_multi(client, 0x0024, 0x0800, 2); //HSIZE_CAP 2048
		err = nm6xx_i2c_write_multi(client, 0x002A, 0x04D0, 2); //VSIZE_CAP 1232
		break;
	case NM6XX_CAPTURE_3MP: /* 2048x1536 */
		err = nm6xx_i2c_write_multi(client, 0x0386, 0x0140, 2);// HSIZE_TN - 320
		err = nm6xx_i2c_write_multi(client, 0x0024, 0x0800, 2); //HSIZE_CAP 2048
		err = nm6xx_i2c_write_multi(client, 0x002A, 0x0600, 2); //VSIZE_CAP 1536
		break;
	default:
		/* The framesize index was not set properly. 
 		 * Check s_fmt call - it must be for video mode. */
		return -EINVAL;
	}

	/* Set capture image size */
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for capture_resolution\n", __func__);
		return -EIO; 
	}

	printk("nm6xx_set_capture_size: %d\n", index);
	return 0;	
}

static int nm6xx_set_jpeg_quality(struct v4l2_subdev *sd)
{
	struct nm6xx_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	int err;

	if(state->jpeg.quality < 0)
		state->jpeg.quality = 0;
	if(state->jpeg.quality > 100)
		state->jpeg.quality = 100;

	switch(state->jpeg.quality)
	{
		case 100: //Super fine
			err = nm6xx_i2c_write_multi(client, 0x0204, 0x0002, 1);
		break;

		case 70: // Fine
			err = nm6xx_i2c_write_multi(client, 0x0204, 0x0001, 1);
		break;

		case 40: // Normal
			err = nm6xx_i2c_write_multi(client, 0x0204, 0x0000, 1);
		break;

		default:
			err = nm6xx_i2c_write_multi(client, 0x0204, 0x0002, 1);

		break;
	}

//	nm6xx_msg(&client->dev, "Quality = %d \n", state->jpeg.quality);
	printk("Quality = %d \n", state->jpeg.quality);	

	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for jpeg_comp_level\n", __func__);
		return -EIO;
	}

	nm6xx_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}

static int nm6xx_get_snapshot_data(struct v4l2_subdev *sd)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct nm6xx_state *state = to_state(sd);

	unsigned long jpeg_framesize;

	if(state->jpeg.enable){
		/* Get main JPEG size */
		err = nm6xx_i2c_read_multi(client, 0x1624, &jpeg_framesize);
		printk("%s: JPEG main filesize = 0x%lx bytes\n", __func__, jpeg_framesize );				
		if(err < 0){
			dev_err(&client->dev, "%s: failed: i2c_read for jpeg_framesize\n", __func__);
			return -EIO;
		}			
		state->jpeg.main_size = jpeg_framesize;

		printk("%s: JPEG main filesize = %d bytes\n", __func__, state->jpeg.main_size );

		state->jpeg.main_offset = 0;
		state->jpeg.thumb_offset = 0x271000;
		state->jpeg.postview_offset = 0x280A00;		
	}

	nm6xx_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}

static int nm6xx_get_LowLightCondition(struct v4l2_subdev *sd, int *Result)
{
	int err = 0;
	unsigned short read_value = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);	

	printk("nm6xx_get_LowLightCondition : start \n");

	err = nm6xx_i2c_read(client, 0x027A, &read_value); // AGC_SCL_NOW
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_read for low_light Condition\n", __func__);
		return -EIO; 
	}

	printk("nm6xx_get_LowLightCondition : Read(0x%X) \n", read_value);
#ifdef CONFIG_LOAD_FILE
	unsigned short max_value = 0;
	max_value = nm6xx_regs_max_value("MAX_VALUE");
	printk("%s   max_value = %x \n", __func__, max_value);
	if(read_value >= max_value)//if(read_value >= 0x0B33)
	{
		*Result = 1; //gLowLight
	}
#else
	if(read_value >= 0x0A20)//if(read_value >= 0x0B33)
	{
		*Result = 1; //gLowLight
	}
#endif
	printk("nm6xx_get_LowLightCondition : end \n");

	return err;
}

static int nm6xx_LowLightCondition_Off(struct v4l2_subdev *sd)
{
	int err = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);	

	printk("nm6xx_LowLightCondition_Off : start \n");


	//write : Outdoor_off
	err = nm6xx_write_regs(sd, \
			nm6xx_Outdoor_Off, \
			sizeof(nm6xx_Outdoor_Off) / sizeof(nm6xx_Outdoor_Off[0]), \
			"nm6xx_Outdoor_Off");	
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for low_light Condition\n", __func__);
		return -EIO; 
	}

	if(gLowLight == 1)
	{
		gLowLight = 0;

		if(gCurrentScene == SCENE_MODE_NIGHTSHOT)
		{
			printk("SCENE_MODE_NIGHTSHOT --- nm6xx_Night_Mode_Off: start \n");		
			err = nm6xx_write_regs(sd, \
					nm6xx_Night_Mode_Off, \
					sizeof(nm6xx_Night_Mode_Off) / sizeof(nm6xx_Night_Mode_Off[0]), \
					"nm6xx_Night_Mode_Off");	
		}
		else
		{
			printk("Not Night mode --- nm6xx_Low_Cap_Off: start \n");			
			err = nm6xx_write_regs(sd, \
					nm6xx_Low_Cap_Off, \
					sizeof(nm6xx_Low_Cap_Off) / sizeof(nm6xx_Low_Cap_Off[0]), \
					"nm6xx_Low_Cap_Off");			
		}
	}
	printk("nm6xx_LowLightCondition_Off : end \n");

	return err;
}

static int nm6xx_set_capture_start(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err, timeout_cnt;
	unsigned short read_value;
	struct i2c_client *client = v4l2_get_subdevdata(sd);


	/*
	 *  1. capture number
	 */
	nm6xx_i2c_write_multi(client, 0x00FC, 0x0002, 1);	//interupt clear

	err = nm6xx_write_regs(sd, nm6xx_prev_to_cap, sizeof(nm6xx_prev_to_cap) / sizeof(nm6xx_prev_to_cap[0]),
			"nm6xx_prev_to_cap");
	/* Outdoor setting */
	nm6xx_i2c_read(client, 0x6C21, &read_value);
	dev_err(&client->dev, "%s: i2c_read ---  OUTDOOR_F == 0x%x \n", __func__, read_value);	
	if(read_value == 0x01)
	{
		nm6xx_i2c_write_multi(client, 0x0014, 0x0003, 1);/*CAPNUM is setted 3. default value is 2. */
		err = nm6xx_write_regs(sd, \
				nm6xx_Outdoor_On, \
				sizeof(nm6xx_Outdoor_On) / sizeof(nm6xx_Outdoor_On[0]), \
				"nm6xx_Outdoor_On");		
	}

	err = nm6xx_get_LowLightCondition(sd, &gLowLight);

	if(gLowLight)
	{
		nm6xx_i2c_write_multi(client, 0x0014, 0x0003, 1);/*CAPNUM is setted 3. default value is 2. */
		if(gCurrentScene == SCENE_MODE_NIGHTSHOT)
		{
			err = nm6xx_write_regs(sd, \
					nm6xx_Night_Mode_On, \
					sizeof(nm6xx_Night_Mode_On) / sizeof(nm6xx_Night_Mode_On[0]), \
					"nm6xx_Night_Mode_On");	
		}
		else
		{
			err = nm6xx_write_regs(sd, \
					nm6xx_Low_Cap_On, \
					sizeof(nm6xx_Low_Cap_On) / sizeof(nm6xx_Low_Cap_On[0]), \
					"nm6xx_Low_Cap_On");
		}
	}
	else
	{
		nm6xx_i2c_write_multi(client, 0x0014, 0x0002, 1);/*CAPNUM is setted 2. default value is 2. */		
	}

	/*
	 *  2. Set image size
	 */
	err = nm6xx_set_capture_size(sd);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for capture_resolution\n", __func__);
		return -EIO; 
	}

	nm6xx_i2c_write_multi(client, 0x0011, 0x0002, 1); //capture_command

	msleep(30);
	/* Wait for Mode Transition (CM) */
	timeout_cnt = 0;
	do {
		timeout_cnt++;
		nm6xx_i2c_read(client, 0x00F8, &read_value);
		msleep(1);
		if (timeout_cnt > 1000) {
			dev_err(&client->dev, "%s: Entering capture mode timed out\n", __func__);	
			break;
		}
	}while(!(read_value&0x02));

	timeout_cnt = 0;
	do {
		timeout_cnt++;
		nm6xx_i2c_write_multi(client, 0x00FC, 0x0002, 1);		
		msleep(1);			
		nm6xx_i2c_read(client, 0x00F8, &read_value);
		if (timeout_cnt > 1000) {
			dev_err(&client->dev, "%s: Entering capture mode timed out\n", __func__);	
			break;
		}
	}while(read_value&0x02);	

	//capture frame out....
	dev_err(&client->dev, "%s: Capture frame out~~~~ \n", __func__);	
	msleep(50);

	timeout_cnt = 0;
	do {
		timeout_cnt++;
		nm6xx_i2c_read(client, 0x00F8, &read_value);
		msleep(1);		
		if (timeout_cnt > 1000) {
			dev_err(&client->dev, "%s: JPEG capture timed out\n", __func__);	
			break;
		}
	}while(!(read_value&0x08));	

	timeout_cnt = 0;
	do {
		timeout_cnt++;
		nm6xx_i2c_write_multi(client, 0x00FC, 0x0008, 1);	
		msleep(1);
		nm6xx_i2c_read(client, 0x00F8, &read_value);
		if (timeout_cnt > 1000) {
			dev_err(&client->dev, "%s: JPEG capture timed out\n", __func__);	
			break;
		}
	}while(read_value&0x08);	

	nm6xx_i2c_read(client, 0x0200, &read_value);
	dev_err(&client->dev, "%s: JPEG STS --- read_value == 0x%x \n", __func__, read_value);			


	/*
	 * 8. Get JPEG Main Data
	 */ 
	err = nm6xx_get_snapshot_data(sd);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: get_snapshot_data\n", __func__);
		return err;
	}

	err = nm6xx_LowLightCondition_Off(sd);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: nm6xx_LowLightCondition_Off\n", __func__);
		return err;
	}
	return 0;
}

static int nm6xx_change_scene_mode(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	nm6xx_msg(&client->dev, "%s: start   CurrentScene : %d, Scene : %d\n", __func__, gCurrentScene, ctrl->value);
	
	switch(ctrl->value)
	{
		case SCENE_MODE_NONE:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Default, \
				sizeof(nm6xx_Scene_Default) / sizeof(nm6xx_Scene_Default[0]), \
				"nm6xx_Scene_Default");			
		break;

		case SCENE_MODE_PORTRAIT:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Portrait, \
				sizeof(nm6xx_Scene_Portrait) / sizeof(nm6xx_Scene_Portrait[0]), \
				"nm6xx_Scene_Portrait");			
		break;

		case SCENE_MODE_NIGHTSHOT:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Nightshot, \
				sizeof(nm6xx_Scene_Nightshot) / sizeof(nm6xx_Scene_Nightshot[0]), \
				"nm6xx_Scene_Nightshot");			
		break;

		case SCENE_MODE_BACK_LIGHT:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Backlight, \
				sizeof(nm6xx_Scene_Backlight) / sizeof(nm6xx_Scene_Backlight[0]), \
				"nm6xx_Scene_Backlight");			
		break;

		case SCENE_MODE_LANDSCAPE:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Landscape, \
				sizeof(nm6xx_Scene_Landscape) / sizeof(nm6xx_Scene_Landscape[0]), \
				"nm6xx_Scene_Landscape");			
		break;

		case SCENE_MODE_SPORTS:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Sports, \
				sizeof(nm6xx_Scene_Sports) / sizeof(nm6xx_Scene_Sports[0]), \
				"nm6xx_Scene_Sports");			
		break;

		case SCENE_MODE_PARTY_INDOOR:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Party_Indoor, \
				sizeof(nm6xx_Scene_Party_Indoor) / sizeof(nm6xx_Scene_Party_Indoor[0]), \
				"nm6xx_Scene_Party_Indoor");			
		break;

		case SCENE_MODE_BEACH_SNOW:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Beach_Snow, \
				sizeof(nm6xx_Scene_Beach_Snow) / sizeof(nm6xx_Scene_Beach_Snow[0]), \
				"nm6xx_Scene_Beach_Snow");			
		break;

		case SCENE_MODE_SUNSET:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Sunset, \
				sizeof(nm6xx_Scene_Sunset) / sizeof(nm6xx_Scene_Sunset[0]), \
				"nm6xx_Scene_Sunset");			
		break;

		case SCENE_MODE_DUST_DAWN:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Duskdawn, \
				sizeof(nm6xx_Scene_Duskdawn) / sizeof(nm6xx_Scene_Duskdawn[0]), \
				"nm6xx_Scene_Duskdawn");			
		break;

		case SCENE_MODE_FALL_COLOR:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Fall_Color, \
				sizeof(nm6xx_Scene_Fall_Color) / sizeof(nm6xx_Scene_Fall_Color[0]), \
				"nm6xx_Scene_Fall_Color");			
		break;

		case SCENE_MODE_FIREWORKS:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Fireworks, \
				sizeof(nm6xx_Scene_Fireworks) / sizeof(nm6xx_Scene_Fireworks[0]), \
				"nm6xx_Scene_Fireworks");			
		break;		

		case SCENE_MODE_TEXT:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Text, \
				sizeof(nm6xx_Scene_Text) / sizeof(nm6xx_Scene_Text[0]), \
				"nm6xx_Scene_Text");			
		break;	

		case SCENE_MODE_CANDLE_LIGHT:
			err = nm6xx_write_regs(sd, \
				nm6xx_Scene_Candle_Light, \
				sizeof(nm6xx_Scene_Candle_Light) / sizeof(nm6xx_Scene_Candle_Light[0]), \
				"nm6xx_Scene_Candle_Light");			
		break;			
		default:
		dev_err(&client->dev, "%s: unsupported scene mode\n", __func__);
		break;
	}

		if(err < 0){
		dev_err(&client->dev, "%s: i2c_write failed\n", __func__);
		return -EIO;
	}
	
	gCurrentScene = ctrl->value;	
	nm6xx_msg(&client->dev, "%s: done   CurrentScene : %d, Scene : %d\n", __func__, gCurrentScene, ctrl->value);
	return 0;
}

static int nm6xx_set_effect(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	nm6xx_msg(&client->dev, "%s: setting value =%d\n", __func__, ctrl->value);

	switch(ctrl->value) {
		case IMAGE_EFFECT_NONE:
	case IMAGE_EFFECT_AQUA:
	case IMAGE_EFFECT_ANTIQUE:
	case IMAGE_EFFECT_SHARPEN:
	default:
		err = nm6xx_write_regs(sd, nm6xx_Effect_Normal, sizeof(nm6xx_Effect_Normal) / sizeof(nm6xx_Effect_Normal[0]),
			"nm6xx_Effect_Normal");
		break;
		case IMAGE_EFFECT_BNW:
		err = nm6xx_write_regs(sd, nm6xx_Effect_Black_White, sizeof(nm6xx_Effect_Black_White) / sizeof(nm6xx_Effect_Black_White[0]),
			"nm6xx_Effect_Black_White");
		break;
		case IMAGE_EFFECT_SEPIA:
		err = nm6xx_write_regs(sd, nm6xx_Effect_Sepia, sizeof(nm6xx_Effect_Sepia) / sizeof(nm6xx_Effect_Sepia[0]),
			"nm6xx_Effect_Sepia");
		break;
		case IMAGE_EFFECT_NEGATIVE:
		err = nm6xx_write_regs(sd, nm6xx_Effect_Negative, sizeof(nm6xx_Effect_Negative) / sizeof(nm6xx_Effect_Negative[0]),
			"nm6xx_Effect_Negative");
		break;
	}

		if(err < 0){
		dev_err(&client->dev, "%s: i2c_write failed\n", __func__);
		return -EIO;
	}
	return 0;
}

static int nm6xx_set_saturation(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	switch(ctrl->value)
	{
		case SATURATION_MINUS_2:
			err = nm6xx_write_regs(sd, \
				nm6xx_Saturation_Minus_2, \
				sizeof(nm6xx_Saturation_Minus_2) / sizeof(nm6xx_Saturation_Minus_2[0]), \
				"nm6xx_Saturation_Minus_2");	
		break;

		case SATURATION_MINUS_1:
			err = nm6xx_write_regs(sd, \
				nm6xx_Saturation_Minus_1, \
				sizeof(nm6xx_Saturation_Minus_1) / sizeof(nm6xx_Saturation_Minus_1[0]), \
				"nm6xx_Saturation_Minus_1");	
		break;

		case SATURATION_DEFAULT:
		default:
			err = nm6xx_write_regs(sd, \
				nm6xx_Saturation_Default, \
				sizeof(nm6xx_Saturation_Default) / sizeof(nm6xx_Saturation_Default[0]), \
				"nm6xx_Saturation_Default");	
		break;

		case SATURATION_PLUS_1:
			err = nm6xx_write_regs(sd, \
				nm6xx_Saturation_Plus_1, \
				sizeof(nm6xx_Saturation_Plus_1) / sizeof(nm6xx_Saturation_Plus_1[0]), \
				"nm6xx_Saturation_Plus_1");	
		break;

		case SATURATION_PLUS_2:
			err = nm6xx_write_regs(sd, \
				nm6xx_Saturation_Plus_2, \
				sizeof(nm6xx_Saturation_Plus_2) / sizeof(nm6xx_Saturation_Plus_2[0]), \
				"nm6xx_Saturation_Plus_2");	
		break;
	}

	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_saturation\n", __func__);
		return -EIO;
	}
	
	nm6xx_msg(&client->dev, "%s: done, saturation: %d\n", __func__, ctrl->value);

	return 0;
}

static int nm6xx_set_contrast(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	switch(ctrl->value)
	{
		case CONTRAST_MINUS_2:
			err = nm6xx_write_regs(sd, \
				nm6xx_Contrast_Minus_2, \
				sizeof(nm6xx_Contrast_Minus_2) / sizeof(nm6xx_Contrast_Minus_2[0]), \
				"nm6xx_Contrast_Minus_2");	
		break;

		case CONTRAST_MINUS_1:
			err = nm6xx_write_regs(sd, \
				nm6xx_Contrast_Minus_1, \
				sizeof(nm6xx_Contrast_Minus_1) / sizeof(nm6xx_Contrast_Minus_1[0]), \
				"nm6xx_Contrast_Minus_1");	
		break;

		case CONTRAST_DEFAULT:
		default:
			err = nm6xx_write_regs(sd, \
				nm6xx_Contrast_Default, \
				sizeof(nm6xx_Contrast_Default) / sizeof(nm6xx_Contrast_Default[0]), \
				"nm6xx_Contrast_Default");	
		break;

		case CONTRAST_PLUS_1:
			err = nm6xx_write_regs(sd, \
				nm6xx_Contrast_Plus_1, \
				sizeof(nm6xx_Contrast_Plus_1) / sizeof(nm6xx_Contrast_Plus_1[0]), \
				"nm6xx_Contrast_Plus_1");	
		break;

		case CONTRAST_PLUS_2:
			err = nm6xx_write_regs(sd, \
				nm6xx_Contrast_Plus_2, \
				sizeof(nm6xx_Contrast_Plus_2) / sizeof(nm6xx_Contrast_Plus_2[0]), \
				"nm6xx_Contrast_Plus_2");	
		break;
	}

	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_contrast\n", __func__);
		return -EIO;
	}
	
	nm6xx_msg(&client->dev, "%s: done, contrast: %d\n", __func__, ctrl->value);

	return 0;
}

static int nm6xx_set_sharpness(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	switch(ctrl->value)
	{
		case SHARPNESS_MINUS_2:
			err = nm6xx_write_regs(sd, \
				nm6xx_Sharpness_Minus_2, \
				sizeof(nm6xx_Sharpness_Minus_2) / sizeof(nm6xx_Sharpness_Minus_2[0]), \
				"nm6xx_Sharpness_Minus_2");	
		break;

		case SHARPNESS_MINUS_1:
			err = nm6xx_write_regs(sd, \
				nm6xx_Sharpness_Minus_1, \
				sizeof(nm6xx_Sharpness_Minus_1) / sizeof(nm6xx_Sharpness_Minus_1[0]), \
				"nm6xx_Sharpness_Minus_1");	
		break;

		case SHARPNESS_DEFAULT:
		default:
			err = nm6xx_write_regs(sd, \
				nm6xx_Sharpness_Default, \
				sizeof(nm6xx_Sharpness_Default) / sizeof(nm6xx_Sharpness_Default[0]), \
				"nm6xx_Sharpness_Default");	
		break;

		case SHARPNESS_PLUS_1:
			err = nm6xx_write_regs(sd, \
				nm6xx_Sharpness_Plus_1, \
				sizeof(nm6xx_Sharpness_Plus_1) / sizeof(nm6xx_Sharpness_Plus_1[0]), \
				"nm6xx_Sharpness_Plus_1");	
		break;

		case SHARPNESS_PLUS_2:
			err = nm6xx_write_regs(sd, \
				nm6xx_Sharpness_Plus_2, \
				sizeof(nm6xx_Sharpness_Plus_2) / sizeof(nm6xx_Sharpness_Plus_2[0]), \
				"nm6xx_Sharpness_Plus_2");	
		break;
	}

	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_saturation\n", __func__);
		return -EIO;
	}
	
	nm6xx_msg(&client->dev, "%s: done, sharpness: %d\n", __func__, ctrl->value);

	return 0;
}

/*Camcorder fix fps*/
static int nm6xx_set_sensor_mode(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	printk("[kidggang]:func(%s):line(%d):ctrl->value(%d)\n",__func__,__LINE__,ctrl->value);
	if (ctrl->value) {
		err = nm6xx_i2c_write_multi(client, 0x0104, 0x000D, 1);
		err = nm6xx_i2c_write_multi(client, 0x0012, 0x0001, 1); //Moni_Refresh
		
		if (err < 0) {
			dev_err(&client->dev, "%s: failed: i2c_write for set_sensor_mode\n", __func__);
			return -EIO;
		}
	}
	return 0;
}

static int nm6xx_set_focus_mode(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;

	switch(ctrl->value) {
		case FOCUS_MODE_MACRO:
		nm6xx_msg(&client->dev, "%s: FOCUS_MODE_MACRO \n", __func__);	
		err = nm6xx_write_regs(sd, nm6xx_AF_Macro_mode, NM6XX_AF_MACRO_MODE_REGS, "nm6xx_AF_Macro_mode");
		if (err < 0) {
			dev_err(&client->dev, "%s: i2c_write failed\n", __func__);
			return -EIO;
		}
		af_mode = FOCUS_MODE_MACRO;
		break;

		case FOCUS_MODE_INFINITY:
			nm6xx_msg(&client->dev, "%s: FOCUS_MODE_INFINITY \n", __func__);
			err = nm6xx_write_regs(sd, nm6xx_AF_Macro_mode, \
								   NM6XX_AF_MACRO_MODE_REGS, "isx005_AF_Infinity_mode");
			if (err < 0) 
			{
				dev_err(&client->dev, "%s: [INFINITY]i2c_write failed\n", __func__);
				return -EIO;
			}
			af_mode = FOCUS_MODE_INFINITY;
			break;

		case FOCUS_MODE_AUTO:
		default:
		nm6xx_msg(&client->dev, "%s: FOCUS_MODE_AUTO \n", __func__);	
		err = nm6xx_write_regs(sd, nm6xx_AF_Normal_mode, NM6XX_AF_NORMAL_MODE_REGS, "nm6xx_AF_Normal_mode");
		if (err < 0) {
			dev_err(&client->dev, "%s: i2c_write failed\n", __func__);
			return -EIO;
		}
		af_mode = FOCUS_MODE_AUTO;
		break;
	}

	return 0;
}

static int nm6xx_set_white_balance(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	switch(ctrl->value)
	{
		case WHITE_BALANCE_AUTO:
			err = nm6xx_write_regs(sd, \
				nm6xx_WB_Auto, \
				sizeof(nm6xx_WB_Auto) / sizeof(nm6xx_WB_Auto[0]), \
				"nm6xx_WB_Auto");
		break;

		case WHITE_BALANCE_SUNNY:
		case WHITE_BALANCE_DAYLIGHT:
			err = nm6xx_write_regs(sd, \
				nm6xx_WB_Sunny, \
				sizeof(nm6xx_WB_Sunny) / sizeof(nm6xx_WB_Sunny[0]), \
				"nm6xx_WB_Sunny");
		break;

		case WHITE_BALANCE_CLOUDY_DAYLIGHT:
			err = nm6xx_write_regs(sd, \
				nm6xx_WB_Cloudy, \
				sizeof(nm6xx_WB_Cloudy) / sizeof(nm6xx_WB_Cloudy[0]), \
				"nm6xx_WB_Cloudy");
		break;

		case WHITE_BALANCE_TUNGSTEN:
		case WHITE_BALANCE_INCANDESCENT:
			err = nm6xx_write_regs(sd, \
				nm6xx_WB_Tungsten, \
				sizeof(nm6xx_WB_Tungsten) / sizeof(nm6xx_WB_Tungsten[0]), \
				"nm6xx_WB_Tungsten");
		break;

		case WHITE_BALANCE_FLUORESCENT:
			err = nm6xx_write_regs(sd, \
				nm6xx_WB_Fluorescent, \
				sizeof(nm6xx_WB_Fluorescent) / sizeof(nm6xx_WB_Fluorescent[0]), \
				"nm6xx_WB_Fluorescent");
		break;

		default:
			dev_err(&client->dev, "%s: failed: to set_white_balance, enum: %d\n", __func__, ctrl->value);
			return -EINVAL;
		break;
	}

	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for white_balance\n", __func__);
		return -EIO;
	}
	
	nm6xx_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}

static int nm6xx_set_ev(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	switch(ctrl->value)
	{
		case EV_MINUS_4:
			err = nm6xx_write_regs(sd, \
				nm6xx_EV_Minus_4, \
				sizeof(nm6xx_EV_Minus_4) / sizeof(nm6xx_EV_Minus_4[0]), \
				"nm6xx_EV_Minus_4");
		break;

		case EV_MINUS_3:
			err = nm6xx_write_regs(sd, \
				nm6xx_EV_Minus_3, \
				sizeof(nm6xx_EV_Minus_3) / sizeof(nm6xx_EV_Minus_3[0]), \
				"nm6xx_EV_Minus_3");
		break;

		case EV_MINUS_2:
			err = nm6xx_write_regs(sd, \
				nm6xx_EV_Minus_2, \
				sizeof(nm6xx_EV_Minus_2) / sizeof(nm6xx_EV_Minus_2[0]), \
				"nm6xx_EV_Minus_2");
		break;

		case EV_MINUS_1:
			err = nm6xx_write_regs(sd, \
				nm6xx_EV_Minus_1, \
				sizeof(nm6xx_EV_Minus_1) / sizeof(nm6xx_EV_Minus_1[0]), \
				"nm6xx_EV_Minus_1");
		break;

		case EV_DEFAULT:
			err = nm6xx_write_regs(sd, \
				nm6xx_EV_Default, \
				sizeof(nm6xx_EV_Default) / sizeof(nm6xx_EV_Default[0]), \
				"nm6xx_EV_Default");
		break;

		case EV_PLUS_1:
			err = nm6xx_write_regs(sd, \
				nm6xx_EV_Plus_1, \
				sizeof(nm6xx_EV_Plus_1) / sizeof(nm6xx_EV_Plus_1[0]), \
				"nm6xx_EV_Default");
		break;

		case EV_PLUS_2:
			err = nm6xx_write_regs(sd, \
				nm6xx_EV_Plus_2, \
				sizeof(nm6xx_EV_Plus_2) / sizeof(nm6xx_EV_Plus_2[0]), \
				"nm6xx_EV_Plus_2");
		break;
		
		case EV_PLUS_3:
			err = nm6xx_write_regs(sd, \
				nm6xx_EV_Plus_3, \
				sizeof(nm6xx_EV_Plus_3) / sizeof(nm6xx_EV_Plus_3[0]), \
				"nm6xx_EV_Plus_3");
		break;

		case EV_PLUS_4:
			err = nm6xx_write_regs(sd, \
				nm6xx_EV_Plus_4, \
				sizeof(nm6xx_EV_Plus_4) / sizeof(nm6xx_EV_Plus_4[0]), \
				"nm6xx_EV_Plus_4");
		break;			

		default:
			dev_err(&client->dev, "%s: failed: to set_ev, enum: %d\n", __func__, ctrl->value);
			return -EINVAL;
		break;
	}

	//printk("nm6xx_set_ev: set_ev:, data: 0x%02x\n", nm6xx_buf_set_ev[1]);
	
		if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_ev\n", __func__);
		return -EIO;
	}
	
	nm6xx_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}

static int nm6xx_set_metering(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	switch(ctrl->value)
	{
		case METERING_MATRIX:
			err = nm6xx_write_regs(sd, \
				nm6xx_Metering_Matrix, \
				sizeof(nm6xx_Metering_Matrix) / sizeof(nm6xx_Metering_Matrix[0]), \
				"nm6xx_Metering_Matrix");
		break;

		case METERING_CENTER:
			err = nm6xx_write_regs(sd, \
				nm6xx_Metering_Center, \
				sizeof(nm6xx_Metering_Center) / sizeof(nm6xx_Metering_Center[0]), \
				"nm6xx_Metering_Center");
		break;

		case METERING_SPOT:
			err = nm6xx_write_regs(sd, \
				nm6xx_Metering_Spot, \
				sizeof(nm6xx_Metering_Spot) / sizeof(nm6xx_Metering_Spot[0]), \
				"nm6xx_Metering_Spot");
		break;

		default:
			dev_err(&client->dev, "%s: failed: to set_photometry, enum: %d\n", __func__, ctrl->value);
			return -EINVAL;
		break;
	}
	
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_photometry\n", __func__);
		return -EIO;
	}
	
	nm6xx_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}

static int nm6xx_set_iso(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	switch(ctrl->value)
	{
		case ISO_50:
		case ISO_800:
		case ISO_1600:
		case ISO_SPORTS:
		case ISO_NIGHT:
		case ISO_AUTO:
		default:
			err = nm6xx_write_regs(sd, \
				nm6xx_ISO_Auto, \
				sizeof(nm6xx_ISO_Auto) / sizeof(nm6xx_ISO_Auto[0]), \
				"nm6xx_ISO_Auto");
		break;

		case ISO_100:
			err = nm6xx_write_regs(sd, \
				nm6xx_ISO_100, \
				sizeof(nm6xx_ISO_100) / sizeof(nm6xx_ISO_100[0]), \
				"nm6xx_ISO_100");
		break;

		case ISO_200:
			err = nm6xx_write_regs(sd, \
				nm6xx_ISO_200, \
				sizeof(nm6xx_ISO_200) / sizeof(nm6xx_ISO_200[0]), \
				"nm6xx_ISO_200");
		break;

		case ISO_400:
			err = nm6xx_write_regs(sd, \
				nm6xx_ISO_400, \
				sizeof(nm6xx_ISO_400) / sizeof(nm6xx_ISO_400[0]), \
				"nm6xx_ISO_400");
		break;

	}
	
	if(err < 0){
		dev_err(&client->dev, "%s: i2c_write failed\n", __func__);
		return -EIO;
	}
	return 0;
}
	
static void af_cancel_handler(unsigned long data)
{
	stop_af_operation = 0;
}

static DEFINE_MUTEX(af_cancel_op);
/* GAUDI Project([arun.c@samsung.com]) 2010.05.19. [Implemented AF cancel] */
static int nm6xx_set_auto_focus(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned short read_value;
	int timeout_cnt;

	/*
	 * ctrl -> value can be 0, 1, 2
	 *
	 * 1 --> start SINGLE AF operation
	 * 0 --> stop SINGLE AF operation or cancel it
	 * 2 --> Check the status of AF cancel operation
	 */
	if (ctrl->value == 1) {
		af_operation_status = 0;
		if (stop_af_operation) {
			del_timer(&af_cancel_timer);
			stop_af_operation = 0;
			return 0;
		}

		/* Enter moniter mode if it is some different mode */
		nm6xx_i2c_read(client, 0x0011, &read_value);
		if ((read_value & 0xFF) != 0x00) {
			nm6xx_i2c_write_multi(client, 0x0011, 0x0000, 1);
			nm6xx_i2c_write_multi(client, 0x0012, 0x0001, 1); //Moni_Refresh
			/* Wait for Mode Transition (CM) */
			timeout_cnt = 0;
			do {
				timeout_cnt++;
				nm6xx_i2c_read(client, 0x00F8, &read_value);
				msleep(1);
				if (timeout_cnt > 1000) {
					dev_err(&client->dev, "%s: Entering moniter mode timed out \n", __func__);	
					break;
				}
			} while (!(read_value & 0x02));

			timeout_cnt = 0;
			do {
				timeout_cnt++;
				nm6xx_i2c_write_multi(client, 0x00FC, 0x0002, 1);		
				msleep(1);			
				nm6xx_i2c_read(client, 0x00F8, &read_value);
				if (timeout_cnt > 1000) {
					dev_err(&client->dev, "%s: Entering moniter mode timed out \n", __func__);	
					break;
				}
			} while (read_value & 0x02);	

			/* delay for AE and AWB to settle down */
			msleep(300);
		}

		if (af_mode == FOCUS_MODE_MACRO) {
			err = nm6xx_write_regs(sd, nm6xx_AF_Return_Macro_pos, NM6XX_AF_RETURN_MACRO_POS_REGS, "nm6xx_AF_Return_Macro_pos");
			if (err < 0) {
				nm6xx_msg(&client->dev, "%s: register write fail \n", __func__);	
				return -EIO;
			}
		} else {
			err = nm6xx_write_regs(sd, nm6xx_AF_Return_Inf_pos, NM6XX_AF_RETURN_INF_POS_REGS, "nm6xx_AF_Return_Inf_pos");
			if (err < 0) {
				nm6xx_msg(&client->dev, "%s: register write fail \n", __func__);	
				return -EIO;
			}
		}

		/* Go to Half release mode */
		nm6xx_i2c_read(client, 0x0011, &read_value);
		if ((read_value & 0xFF) != 0x01) {
			err = nm6xx_write_regs(sd, nm6xx_half_release, sizeof(nm6xx_half_release) / sizeof(nm6xx_half_release[0]),
				       	"nm6xx_half_release");

		/* Wait for Mode Transition (CM) */
			timeout_cnt = 0;
		do {
				timeout_cnt++;
			nm6xx_i2c_read(client, 0x00F8, &read_value);
			msleep(1);
				if (timeout_cnt > 1000) {
					dev_err(&client->dev, "%s: Entering Half release mode timed out \n", __func__);	
					break;
				}
		} while (!(read_value & 0x02));

			timeout_cnt = 0;
		do {
				timeout_cnt++;
			nm6xx_i2c_write_multi(client, 0x00FC, 0x0002, 1);		
			msleep(1);			
			nm6xx_i2c_read(client, 0x00F8, &read_value);
				if (timeout_cnt > 1000) {
					dev_err(&client->dev, "%s: Entering Half release mode timed out \n", __func__);	
					break;
				}
		} while (read_value & 0x02);	
		} 

		err = nm6xx_write_regs(sd, nm6xx_Single_AF_Start, NM6XX_SINGLE_AF_START_REGS, "nm6xx_Single_AF_Start");
		if (err < 0) {
			nm6xx_msg(&client->dev, "%s: register write fail \n", __func__);	
			return -EIO;
		}

		af_operation_status = 1;
	} else if (ctrl->value == 0) {
		mutex_lock(&af_cancel_op);
		stop_af_operation = 1;
		mod_timer(&af_cancel_timer, get_jiffies_64() + msecs_to_jiffies(150));
		if (af_operation_status == 2) {
			stop_af_operation = 0;
			af_operation_status = 0;

			/* Return to moniter mode */
			nm6xx_i2c_read(client, 0x0011, &read_value);
			if ((read_value & 0xFF) != 0x00) {
				nm6xx_i2c_write_multi(client, 0x0011, 0x0000, 1);
				nm6xx_i2c_write_multi(client, 0x0012, 0x0001, 1); //Moni_Refresh

				/* Wait for Mode Transition (CM) */
				timeout_cnt = 0;
				do {
					timeout_cnt++;
					nm6xx_i2c_read(client, 0x00F8, &read_value);
					msleep(1);
					if (timeout_cnt > 1000) {
						dev_err(&client->dev, "%s: Entering moniter mode timed out \n", __func__);	
						break;
					}
				} while (!(read_value & 0x02));

				timeout_cnt = 0;
				do {
					timeout_cnt++;
					nm6xx_i2c_write_multi(client, 0x00FC, 0x0002, 1);		
					msleep(1);			
					nm6xx_i2c_read(client, 0x00F8, &read_value);
					if (timeout_cnt > 1000) {
						dev_err(&client->dev, "%s: Entering moniter mode timed out \n", __func__);	
						break;
					}
				} while (read_value & 0x02);	
			}

			if (af_mode == FOCUS_MODE_MACRO) {
				err = nm6xx_write_regs(sd, nm6xx_AF_Return_Macro_pos, NM6XX_AF_RETURN_MACRO_POS_REGS, "nm6xx_AF_Return_Macro_pos");
				if (err < 0) {
					nm6xx_msg(&client->dev, "%s: register write fail \n", __func__);	
					mutex_unlock(&af_cancel_op);
					return -EIO;
				}
			} else {
				err = nm6xx_write_regs(sd, nm6xx_AF_Return_Inf_pos, NM6XX_AF_RETURN_INF_POS_REGS, "nm6xx_AF_Return_Inf_pos");
				if (err < 0) {
					nm6xx_msg(&client->dev, "%s: register write fail \n", __func__);	
					mutex_unlock(&af_cancel_op);
					return -EIO;
				}
			}
		}
		mutex_unlock(&af_cancel_op);
	} else {
		nm6xx_msg(&client->dev, "%s:get AF cancel status start \n", __func__);	
		while (stop_af_operation)
			msleep(5);
		nm6xx_msg(&client->dev, "%s:get AF cancel status end \n", __func__);	
	}

	return 0;
}

static int nm6xx_get_auto_focus_status(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err, count = 0;
	unsigned short read_value;	
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int timeout_cnt;

	nm6xx_msg(&client->dev, "%s: Check AF Result~~~~~~~ \n", __func__);		

	/* If af operation is not started do not perform status check */
	if (!af_operation_status) {
		ctrl->value = 0x02;
		return 0;
	}

	/* Check the AF result by polling at 0x6d76 for 0x08*/
	do {
		/* count is used to prevent endless looping here */
		count++;

		/* Check for AF cancel if af is cancelled stop the operation and return */
		if (stop_af_operation) {
			nm6xx_msg(&client->dev, "AF is cancelled while doing\n");
			del_timer(&af_cancel_timer);
			nm6xx_i2c_write_multi(client, 0x4885, 0x0001, 1);

			/* Return to Macro or infinite position */
			if (af_mode == FOCUS_MODE_MACRO) {
				err = nm6xx_write_regs(sd, nm6xx_AF_Return_Macro_pos, NM6XX_AF_RETURN_MACRO_POS_REGS, "nm6xx_AF_Return_Macro_pos");
				if (err < 0) {
					nm6xx_msg(&client->dev, "%s: register read fail \n", __func__);	
					return -EIO;
				}
			} else {
				err = nm6xx_write_regs(sd, nm6xx_AF_Return_Inf_pos, NM6XX_AF_RETURN_INF_POS_REGS, "nm6xx_AF_Return_Inf_pos");
				if (err < 0) {
					nm6xx_msg(&client->dev, "%s: register read fail \n", __func__);	
					return -EIO;
				}
			}

			/* Return to moniter mode */
			nm6xx_i2c_read(client, 0x0011, &read_value);
			if ((read_value & 0xFF) != 0x00) {
				nm6xx_i2c_write_multi(client, 0x0011, 0x0000, 1);
				nm6xx_i2c_write_multi(client, 0x0012, 0x0001, 1); //Moni_Refresh

				/* Wait for Mode Transition (CM) */
				timeout_cnt = 0;
				do {
					timeout_cnt++;
					nm6xx_i2c_read(client, 0x00F8, &read_value);
					msleep(1);
					if (timeout_cnt > 1000) {
						dev_err(&client->dev, "%s: Entering moniter mode timed out \n", __func__);	
						break;
					}
				} while (!(read_value & 0x02));

				timeout_cnt = 0;
				do {
					timeout_cnt++;
					nm6xx_i2c_write_multi(client, 0x00FC, 0x0002, 1);		
					msleep(1);			
					nm6xx_i2c_read(client, 0x00F8, &read_value);
					if (timeout_cnt > 1000) {
						dev_err(&client->dev, "%s: Entering moniter mode timed out \n", __func__);	
						break;
					}
				} while (read_value & 0x02);	
			}

			/* Clear AF_LOCK_STS */
			nm6xx_i2c_write_multi(client, 0x00FC, 0x0010, 1);

			/*
			 * Inform user that AF is cancelled
			 * 0 --> AF failure
			 * 1 --> AF success
			 * 2 --> AF cancel
			 */
			if (count < 100 && count > 15)
				msleep(100);			
			else if (count >= 100)
				msleep(300);			

			ctrl->value = 0x02;
			stop_af_operation = 0;
			nm6xx_msg(&client->dev, "AF cancel finished\n");
			return 0;
		}
		nm6xx_i2c_read(client, 0x6D76, &read_value);
		nm6xx_msg(&client->dev, "%s: i2c_read --- read_value == 0x%x \n", __func__, read_value);		
		msleep(1);
		if (count > 1000) {
			ctrl->value = 0x00;	/* 0x00 --> AF failed*/ 
			break;
		}
	}while(!(read_value&0x08));	

	mutex_lock(&af_cancel_op);
	/* Clear AF_LOCK_STS */
	nm6xx_i2c_write_multi(client, 0x00FC, 0x0010, 1);

	/* Read AF result */
	nm6xx_i2c_read(client, 0x6D77, &read_value);
	dev_err(&client->dev, "%s: i2c_read --- read_value == 0x%x \n", __func__, read_value);		

	if ((read_value & 0xFF) == 0x01) {
		nm6xx_msg(&client->dev, "%s: AF is success~~~~~~~ \n", __func__);
		ctrl->value = 0x01;	/* 0x01 --> AF sucess */ 
	} else if ((read_value & 0xFF) == 0x00) {
		nm6xx_msg(&client->dev, "%s: AF is Failure~~~~~~~ \n", __func__);
		ctrl->value = 0x00;	/* 0x00 --> AF failed*/ 
	}

	/* We finished turn off the single AF now */
	nm6xx_msg(&client->dev, "%s: single AF Off command Setting~~~~ \n", __func__);	

		err = nm6xx_write_regs(sd, nm6xx_Single_AF_Off, NM6XX_SINGLE_AF_OFF_REGS, "nm6xx_Single_AF_Off");
	if (err < 0) {
		nm6xx_msg(&client->dev, "%s: register write fail \n", __func__);	
		mutex_unlock(&af_cancel_op);
		return -EIO;
	}

	if (stop_af_operation) {
		/* Return to moniter mode */
		nm6xx_i2c_read(client, 0x0011, &read_value);
		if ((read_value & 0xFF) != 0x00) {
		nm6xx_i2c_write_multi(client, 0x0011, 0x0000, 1);
		nm6xx_i2c_write_multi(client, 0x0012, 0x0001, 1); //Moni_Refresh

		/* Wait for Mode Transition (CM) */
			timeout_cnt = 0;
		do {
				timeout_cnt++;
			nm6xx_i2c_read(client, 0x00F8, &read_value);
			msleep(1);
				if (timeout_cnt > 1000) {
					dev_err(&client->dev, "%s: Entering moniter mode timed out \n", __func__);	
					break;
				}
		} while (!(read_value & 0x02));

			timeout_cnt = 0;
		do {
				timeout_cnt++;
			nm6xx_i2c_write_multi(client, 0x00FC, 0x0002, 1);		
			msleep(1);			
			nm6xx_i2c_read(client, 0x00F8, &read_value);
				if (timeout_cnt > 1000) {
					dev_err(&client->dev, "%s: Entering moniter mode timed out \n", __func__);	
					break;
				}
		} while (read_value & 0x02);	
		}

		/* Return to Macro or infinite position */
		if (af_mode == FOCUS_MODE_MACRO) {
			err = nm6xx_write_regs(sd, nm6xx_AF_Return_Macro_pos, NM6XX_AF_RETURN_MACRO_POS_REGS, "nm6xx_AF_Return_Macro_pos");
			if (err < 0) {
				nm6xx_msg(&client->dev, "%s: register write fail \n", __func__);	
				mutex_unlock(&af_cancel_op);
			return -EIO;
			}
		} else {
			err = nm6xx_write_regs(sd, nm6xx_AF_Return_Inf_pos, NM6XX_AF_RETURN_INF_POS_REGS, "nm6xx_AF_Return_Inf_pos");
			if (err < 0) {
				nm6xx_msg(&client->dev, "%s: register write fail \n", __func__);	
				mutex_unlock(&af_cancel_op);
				return -EIO;
			}	
		}	
	}

	stop_af_operation = 0;
	af_operation_status = 2;
	nm6xx_msg(&client->dev, "%s: single AF check finished~~~~ \n", __func__);	
	mutex_unlock(&af_cancel_op);
	return 0;
}

static int nm6xx_aeawb_unlock(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	unsigned short read_value;	
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int timeout_cnt;

	nm6xx_msg(&client->dev, "%s: unlocking AE&AWB\n", __func__);	

	mutex_lock(&af_cancel_op);
	if (af_operation_status == 2) {
		af_operation_status = 0;

		/* Enter moniter mode if it is some different mode */
		nm6xx_i2c_read(client, 0x0011, &read_value);
		if ((read_value & 0xFF) != 0x00) {
		nm6xx_i2c_write_multi(client, 0x0011, 0x0000, 1);
		nm6xx_i2c_write_multi(client, 0x0012, 0x0001, 1); //Moni_Refresh
		/* Wait for Mode Transition (CM) */
			timeout_cnt = 0;
		do {
				timeout_cnt++;
			nm6xx_i2c_read(client, 0x00F8, &read_value);
			msleep(1);
				if (timeout_cnt > 1000) {
					dev_err(&client->dev, "%s: Entering moniter mode timed out \n", __func__);	
					break;
				}
		} while (!(read_value & 0x02));

			timeout_cnt = 0;
		do {
				timeout_cnt++;
			nm6xx_i2c_write_multi(client, 0x00FC, 0x0002, 1);		
			msleep(1);			
			nm6xx_i2c_read(client, 0x00F8, &read_value);
				if (timeout_cnt > 1000) {
					dev_err(&client->dev, "%s: Entering moniter mode timed out \n", __func__);	
					break;
				}
		} while (read_value & 0x02);	
		}
	}
	mutex_unlock(&af_cancel_op);
	return 0;
}

static int nm6xx_get_iso(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned short tmp;
	unsigned int iso_table[19] = {25, 32, 40, 50, 64,
							 80, 100, 125, 160, 200,
							 250, 320, 400, 500, 640,
							 800, 1000, 1250, 1600};

	err = nm6xx_i2c_read(client, 0x00F0, &tmp);
	ctrl->value = iso_table[tmp-1];

	return err;
}

static int nm6xx_get_shutterspeed(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned short tmp;

	err = nm6xx_i2c_read(client, 0x00F2, &tmp);
	ctrl->value |= tmp;
	err = nm6xx_i2c_read(client, 0x00F4, &tmp);
	ctrl->value |= (tmp << 16);

	return err;
}

static void nm6xx_init_parameters(struct v4l2_subdev *sd)
{
	struct nm6xx_state *state = to_state(sd);

	/* Set initial values for the sensor stream parameters */
	state->strm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	state->strm.parm.capture.timeperframe.numerator = 1;
	state->strm.parm.capture.capturemode = 0;

	state->framesize_index = NM6XX_PREVIEW_QVGA;//bestiq //johnny.kim
	state->fps = 30; /* Default value */
	
	state->jpeg.enable = 0;
	state->jpeg.quality = 100;
	state->jpeg.main_offset = 0;
	state->jpeg.main_size = 0;
	state->jpeg.thumb_offset = 0;
	state->jpeg.thumb_size = 0;
	state->jpeg.postview_offset = 0;
}

#if 0
/* Sample code */
static const char *nm6xx_querymenu_wb_preset[] = {
	"WB Tungsten", "WB Fluorescent", "WB sunny", "WB cloudy", NULL
};
#endif

static struct v4l2_queryctrl nm6xx_controls[] = {
#if 0
	/* Sample code */
	{
		.id = V4L2_CID_WHITE_BALANCE_PRESET,
		.type = V4L2_CTRL_TYPE_MENU,
		.name = "White balance preset",
		.minimum = 0,
		.maximum = ARRAY_SIZE(nm6xx_querymenu_wb_preset) - 2,
		.step = 1,
		.default_value = 0,
	},
#endif
};

const char **nm6xx_ctrl_get_menu(u32 id)
{
	switch (id) {
#if 0
	/* Sample code */
	case V4L2_CID_WHITE_BALANCE_PRESET:
		return nm6xx_querymenu_wb_preset;
#endif
	default:
		return v4l2_ctrl_get_menu(id);
	}
}

static inline struct v4l2_queryctrl const *nm6xx_find_qctrl(int id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(nm6xx_controls); i++)
		if (nm6xx_controls[i].id == id)
			return &nm6xx_controls[i];

	return NULL;
}

static int nm6xx_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(nm6xx_controls); i++) {
		if (nm6xx_controls[i].id == qc->id) {
			memcpy(qc, &nm6xx_controls[i], sizeof(struct v4l2_queryctrl));
			return 0;
		}
	}

	return -EINVAL;
}

static int nm6xx_querymenu(struct v4l2_subdev *sd, struct v4l2_querymenu *qm)
{
	struct v4l2_queryctrl qctrl;

	qctrl.id = qm->id;
	nm6xx_queryctrl(sd, &qctrl);

	return v4l2_ctrl_query_menu(qm, &qctrl, nm6xx_ctrl_get_menu(qm->id));
}

/*
 * Clock configuration
 * Configure expected MCLK from host and return EINVAL if not supported clock
 * frequency is expected
 * 	freq : in Hz
 * 	flag : not supported for now
 */
static int nm6xx_s_crystal_freq(struct v4l2_subdev *sd, u32 freq, u32 flags)
{
	int err = -EINVAL;

	return err;
}

static int nm6xx_g_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;

	return err;
}

static int nm6xx_get_framesize_index(struct v4l2_subdev *sd);
static int nm6xx_set_framesize_index(struct v4l2_subdev *sd, unsigned int index);
/* Information received: 
 * width, height
 * pixel_format -> to be handled in the upper layer 
 *
 * */
static int nm6xx_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;
	struct nm6xx_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int framesize_index = -1;

	if(fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_JPEG && fmt->fmt.pix.colorspace != V4L2_COLORSPACE_JPEG){
		dev_err(&client->dev, "%s: mismatch in pixelformat and colorspace\n", __func__);
		return -EINVAL;
	}

	if(fmt->fmt.pix.width == 800 && fmt->fmt.pix.height == 448) {
		state->pix.width = 1280;
		state->pix.height = 720;
	} else {
		state->pix.width = fmt->fmt.pix.width;
		state->pix.height = fmt->fmt.pix.height;
	}
	
	state->pix.pixelformat = fmt->fmt.pix.pixelformat;

	if(fmt->fmt.pix.colorspace == V4L2_COLORSPACE_JPEG)
		state->oprmode = NM6XX_OPRMODE_IMAGE;
	else
		state->oprmode = NM6XX_OPRMODE_VIDEO; 


	framesize_index = nm6xx_get_framesize_index(sd);

	nm6xx_msg(&client->dev, "%s:framesize_index = %d\n", __func__, framesize_index);
	
	err = nm6xx_set_framesize_index(sd, framesize_index);
	if(err < 0){
		dev_err(&client->dev, "%s: set_framesize_index failed\n", __func__);
		return -EINVAL;
	}

	if(state->pix.pixelformat == V4L2_PIX_FMT_JPEG){
		state->jpeg.enable = 1;
	} else {
		state->jpeg.enable = 0;
	}

	return 0;
}

static int nm6xx_enum_framesizes(struct v4l2_subdev *sd, \
					struct v4l2_frmsizeenum *fsize)
{
	struct nm6xx_state *state = to_state(sd);
	int num_entries = sizeof(nm6xx_framesize_list)/sizeof(struct nm6xx_enum_framesize);	
	struct nm6xx_enum_framesize *elem;	
	int index = 0;
	int i = 0;

	/* The camera interface should read this value, this is the resolution
 	 * at which the sensor would provide framedata to the camera i/f
 	 *
 	 * In case of image capture, this returns the default camera resolution (SVGA)
 	 */
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	if(state->pix.pixelformat == V4L2_PIX_FMT_JPEG){
		index = NM6XX_PREVIEW_SVGA;
	} else {
		index = state->framesize_index;
	}

	for(i = 0; i < num_entries; i++){
		elem = &nm6xx_framesize_list[i];
		if(elem->index == index){
			fsize->discrete.width = nm6xx_framesize_list[index].width;
			fsize->discrete.height = nm6xx_framesize_list[index].height;
			return 0;
		}
	}

	return -EINVAL;
}

static int nm6xx_enum_frameintervals(struct v4l2_subdev *sd, 
					struct v4l2_frmivalenum *fival)
{
	int err = 0;

	return err;
}

static int nm6xx_enum_fmt(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmtdesc)
{
	int num_entries;

	num_entries = sizeof(capture_fmts)/sizeof(struct v4l2_fmtdesc);

	if(fmtdesc->index >= num_entries)
		return -EINVAL;

	memset(fmtdesc, 0, sizeof(*fmtdesc));
	memcpy(fmtdesc, &capture_fmts[fmtdesc->index], sizeof(*fmtdesc));

	return 0;
}

static int nm6xx_try_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int num_entries;
	int i;

	num_entries = sizeof(capture_fmts)/sizeof(struct v4l2_fmtdesc);

	for(i = 0; i < num_entries; i++){
		if(capture_fmts[i].pixelformat == fmt->fmt.pix.pixelformat)
			return 0;
	} 

	return -EINVAL;
}

/** Gets current FPS value */
static int nm6xx_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct nm6xx_state *state = to_state(sd);
	int err = 0;

	state->strm.parm.capture.timeperframe.numerator = 1;
	state->strm.parm.capture.timeperframe.denominator = state->fps;

	memcpy(param, &state->strm, sizeof(param));

	return err;
}

/** Sets the FPS value */
static int nm6xx_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	int err = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct nm6xx_state *state = to_state(sd);

	if(param->parm.capture.timeperframe.numerator != state->strm.parm.capture.timeperframe.numerator ||
			param->parm.capture.timeperframe.denominator != state->strm.parm.capture.timeperframe.denominator){
		
		int fps = 0;
		int fps_max = 30;

		if(param->parm.capture.timeperframe.numerator && param->parm.capture.timeperframe.denominator)
			fps = (int)(param->parm.capture.timeperframe.denominator/param->parm.capture.timeperframe.numerator);
		else 
			fps = 0;

		if(fps <= 0 || fps > fps_max){
			dev_err(&client->dev, "%s: Framerate %d not supported, setting it to %d fps.\n",__func__, fps, fps_max);
			fps = fps_max;
		}

		param->parm.capture.timeperframe.numerator = 1;
		param->parm.capture.timeperframe.denominator = fps;
	
		state->fps = fps;
	}

	/* Don't set the fps value, just update it in the state 
	 * We will set the resolution and fps in the start operation (preview/capture) call */
	
	return err;
}

/* This function is called from the g_ctrl api
 *
 * This function should be called only after the s_fmt call,
 * which sets the required width/height value.
 *
 * It checks a list of available frame sizes and returns the 
 * most appropriate index of the frame size.
 *
 * Note: The index is not the index of the entry in the list. It is
 * the value of the member 'index' of the particular entry. This is
 * done to add additional layer of error checking.
 *
 * The list is stored in an increasing order (as far as possible).
 * Hene the first entry (searching from the beginning) where both the 
 * width and height is more than the required value is returned.
 * In case of no match, we return the last entry (which is supposed
 * to be the largest resolution supported.)
 *
 * It returns the index (enum nm6xx_frame_size) of the framesize entry.
 */
static int nm6xx_get_framesize_index(struct v4l2_subdev *sd)
{
	int i = 0;
	struct nm6xx_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct nm6xx_enum_framesize *frmsize;

	nm6xx_msg(&client->dev, "%s: Requested Res: %dx%d\n", __func__, state->pix.width, state->pix.height);

	/* Check for video/image mode */
	for(i = 0; i < (sizeof(nm6xx_framesize_list)/sizeof(struct nm6xx_enum_framesize)); i++)
	{
		frmsize = &nm6xx_framesize_list[i];

		if(frmsize->mode != state->oprmode)
			continue;

		if(state->oprmode == NM6XX_OPRMODE_IMAGE){
			/* In case of image capture mode, if the given image resolution is not supported,
 			 * return the next higher image resolution. */
			if(frmsize->width >= state->pix.width && frmsize->height >= state->pix.height)
				return frmsize->index;
		} else {
			/* In case of video mode, if the given video resolution is not matching, use
 			 * the default rate (currently NM6XX_PREVIEW_WVGA).
 			 */		 
			if(frmsize->width == state->pix.width && frmsize->height == state->pix.height)
#ifdef CONFIG_VIDEO_NM6XX
				return NM6XX_PREVIEW_QVGA;//frmsize->index; //johnny.kim
#else
				return frmsize->index;
#endif
		}

	} 
	
	/* If it fails, return the default value. */
#ifdef CONFIG_VIDEO_NM6XX
	return (state->oprmode == NM6XX_OPRMODE_IMAGE) ? NM6XX_CAPTURE_3MP : NM6XX_PREVIEW_QVGA;//NM6XX_PREVIEW_SVGA; //johnny.kim
#else
	return (state->oprmode == NM6XX_OPRMODE_IMAGE) ? NM6XX_CAPTURE_3MP : NM6XX_PREVIEW_SVGA;
#endif
}


/* This function is called from the s_ctrl api
 * Given the index, it checks if it is a valid index.
 * On success, it returns 0.
 * On Failure, it returns -EINVAL
 */
static int nm6xx_set_framesize_index(struct v4l2_subdev *sd, unsigned int index)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct nm6xx_state *state = to_state(sd);
	int i = 0;

	for(i = 0; i < (sizeof(nm6xx_framesize_list)/sizeof(struct nm6xx_enum_framesize)); i++)
	{
		if(nm6xx_framesize_list[i].index == index && nm6xx_framesize_list[i].mode == state->oprmode){
			state->framesize_index = nm6xx_framesize_list[i].index;	
			state->pix.width = nm6xx_framesize_list[i].width;
			state->pix.height = nm6xx_framesize_list[i].height;
			nm6xx_info(&client->dev, "%s: Camera Res: %dx%d\n", __func__, state->pix.width, state->pix.height);
			return 0;
		} 
	} 
	
	return -EINVAL;
}

static int nm6xx_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct nm6xx_state *state = to_state(sd);
	struct nm6xx_userset userset = state->userset;
	int err = -ENOIOCTLCMD;

	CDBG("nm6xx_g_ctrl id = 0x%x, %d\n", ctrl->id, ctrl->id & 0xFF);
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ctrl->value = userset.exposure_bias;
		err = 0;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ctrl->value = userset.auto_wb;
		err = 0;
		break;
	case V4L2_CID_WHITE_BALANCE_PRESET:
		ctrl->value = userset.manual_wb;
		err = 0;
		break;
	case V4L2_CID_COLORFX:
		ctrl->value = userset.effect;
		err = 0;
		break;
	case V4L2_CID_CONTRAST:
		ctrl->value = userset.contrast;
		err = 0;
		break;
	case V4L2_CID_SATURATION:
		ctrl->value = userset.saturation;
		err = 0;
		break;
	case V4L2_CID_SHARPNESS:
		ctrl->value = userset.sharpness;
		err = 0;
		break;

	case V4L2_CID_CAM_JPEG_MAIN_SIZE:
		ctrl->value = state->jpeg.main_size;
		err = 0;
		break;
	
	case V4L2_CID_CAM_JPEG_MAIN_OFFSET:
		ctrl->value = state->jpeg.main_offset;
		err = 0;
		break;

	case V4L2_CID_CAM_JPEG_THUMB_SIZE:
		ctrl->value = state->jpeg.thumb_size;
		err = 0;
		break;
	case V4L2_CID_CAM_JPEG_THUMB_OFFSET:
		ctrl->value = state->jpeg.thumb_offset;
		err = 0;
		break;

	case V4L2_CID_CAM_JPEG_POSTVIEW_OFFSET:
		ctrl->value = state->jpeg.postview_offset;
		err = 0;
		break; 
	
	case V4L2_CID_CAM_JPEG_MEMSIZE:
		ctrl->value = SENSOR_JPEG_SNAPSHOT_MEMSIZE;
		err = 0;
		break;

	//need to be modified
	case V4L2_CID_CAM_JPEG_QUALITY:
		ctrl->value = state->jpeg.quality;
		err = 0;
		break;
	case V4L2_CID_CAMERA_OBJ_TRACKING_STATUS:
		err = 0;
		break;
	case V4L2_CID_CAMERA_SMART_AUTO_STATUS:
		err = 0;
		break;

	case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT:
		err = nm6xx_get_auto_focus_status(sd, ctrl);
		break;

	case V4L2_CID_CAM_DATE_INFO_YEAR:
		ctrl->value = 2010;//state->dateinfo.year;//bestiq 
		err = 0;
		break; 
	case V4L2_CID_CAM_DATE_INFO_MONTH:
		ctrl->value = 2;//state->dateinfo.month;
		err = 0;
		break; 
	case V4L2_CID_CAM_DATE_INFO_DATE:
		ctrl->value = 25;//state->dateinfo.date;
		err = 0;
		break; 
	case V4L2_CID_CAM_SENSOR_VER:
		ctrl->value = state->sensor_version;
		err = 0;
		break; 
	case V4L2_CID_CAM_FW_MINOR_VER:
		ctrl->value = state->fw.minor;
		err = 0;
		break; 
	case V4L2_CID_CAM_FW_MAJOR_VER:
		ctrl->value = state->fw.major;
		err = 0;
		break; 
	case V4L2_CID_CAM_PRM_MINOR_VER:
		ctrl->value = state->prm.minor;
		err = 0;
		break; 
	case V4L2_CID_CAM_PRM_MAJOR_VER:
		ctrl->value = state->prm.major;
		err = 0;
		break;

	case V4L2_CID_CAMERA_GET_FLASH_ONOFF:
		err = 0;
		break;

	case V4L2_CID_CAMERA_GET_ISO:
		err = 0;
		break;

	case V4L2_CID_CAMERA_GET_SHT_TIME:
		err = 0;
		break;

	case V4L2_CID_ESD_INT: //johnny.kim
		err = 0;
		break;		

	case V4L2_CID_CAM_SENSOR_TYPE:
		err = 0;
		break;		
		
	default:
		dev_err(&client->dev, "%s: no such ctrl\n", __func__);
		break;
	}
	
	return err;
}

static int nm6xx_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct nm6xx_state *state = to_state(sd);
	int err = 0;

	nm6xx_info(&client->dev, "%s: V4l2 control ID =%d\n", __func__, ctrl->id - V4L2_CID_PRIVATE_BASE);

#ifdef CONFIG_VIDEO_NM6XX //johnny.kim
	ctrl->value = 0;
	printk("nm6xx_s_ctrl id = 0x%x, %d\n", ctrl->id, ctrl->id & 0xFF);
	return 0;
#else
	mutex_lock(&sensor_s_ctrl);

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_AEAWB_LOCK_UNLOCK:
		err = nm6xx_aeawb_unlock(sd, ctrl);
		break;
		
	case V4L2_CID_CAMERA_FLASH_MODE:
		err = 0;
		break;
	case V4L2_CID_CAMERA_BRIGHTNESS:
		err = nm6xx_set_ev(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_WHITE_BALANCE:
		err = nm6xx_set_white_balance(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_EFFECT:
		err = nm6xx_set_effect(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_ISO:
		err = nm6xx_set_iso(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_METERING:
		err = nm6xx_set_metering(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_CONTRAST:
		err = nm6xx_set_contrast(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SATURATION:
		err = nm6xx_set_saturation(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SHARPNESS:
		err = nm6xx_set_sharpness(sd, ctrl);
		break;
/*Camcorder fix fps*/
	case V4L2_CID_CAMERA_SENSOR_MODE:
		err = nm6xx_set_sensor_mode(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_WDR:
		err = 0;
		break;

	case V4L2_CID_CAMERA_ANTI_SHAKE:
		err = 0;
		break;

	case V4L2_CID_CAMERA_FACE_DETECTION:
		err = 0;
		break;

	case V4L2_CID_CAMERA_SMART_AUTO:
		err = 0;
		break;

	case V4L2_CID_CAMERA_FOCUS_MODE:
		err = nm6xx_set_focus_mode(sd, ctrl);
		break;
		
	case V4L2_CID_CAMERA_VINTAGE_MODE:
		err = 0;
		break;
		
	case V4L2_CID_CAMERA_BEAUTY_SHOT:
		err = 0;
		break;

	case V4L2_CID_CAMERA_FACEDETECT_LOCKUNLOCK:
		err = 0;
		break;		

	//need to be modified
	case V4L2_CID_CAM_JPEG_QUALITY:
		if(ctrl->value < 0 || ctrl->value > 100){
			err = -EINVAL;
		} else {
			state->jpeg.quality = ctrl->value;
			err = nm6xx_set_jpeg_quality(sd);
		}
		break;

	case V4L2_CID_CAMERA_SCENE_MODE:
		err = nm6xx_change_scene_mode(sd, ctrl);
		printk("nm6xx_change_scene_mode = %d \n", ctrl->value);	
//		err = 0;
		break;

	case V4L2_CID_CAMERA_GPS_LATITUDE:
		dev_err(&client->dev, "%s: V4L2_CID_CAMERA_GPS_LATITUDE: not implemented\n", __func__);
		break;

	case V4L2_CID_CAMERA_GPS_LONGITUDE:
		dev_err(&client->dev, "%s: V4L2_CID_CAMERA_GPS_LONGITUDE: not implemented\n", __func__);
		break;

	case V4L2_CID_CAMERA_GPS_TIMESTAMP:
		dev_err(&client->dev, "%s: V4L2_CID_CAMERA_GPS_TIMESTAMP: not implemented\n", __func__);
		break;

	case V4L2_CID_CAMERA_GPS_ALTITUDE:
		dev_err(&client->dev, "%s: V4L2_CID_CAMERA_GPS_ALTITUDE: not implemented\n", __func__);
		break;

	case V4L2_CID_CAMERA_ZOOM:
		err = nm6xx_set_dzoom(sd, ctrl);
//		err = 0;
		break;

	case V4L2_CID_CAMERA_TOUCH_AF_START_STOP:
		err = 0;
		break;
		
	case V4L2_CID_CAMERA_CAF_START_STOP:
		err = 0;
		break;	

	case V4L2_CID_CAMERA_OBJECT_POSITION_X:
		state->position.x = ctrl->value;
		err = 0;
		break;

	case V4L2_CID_CAMERA_OBJECT_POSITION_Y:
		state->position.y = ctrl->value;
		err = 0;
		break;

	case V4L2_CID_CAMERA_OBJ_TRACKING_START_STOP:
		err = 0;
		break;

	case V4L2_CID_CAMERA_SET_AUTO_FOCUS:
		err = nm6xx_set_auto_focus(sd, ctrl);
		break;		

	case V4L2_CID_CAMERA_FRAME_RATE:
		state->fps = ctrl->value;
		err = 0;		
		break;
		
	case V4L2_CID_CAMERA_ANTI_BANDING:
		err = 0;		
		break;

	case V4L2_CID_CAM_CAPTURE:
		err = nm6xx_set_capture_start(sd, ctrl);
		break;
	
	/* Used to start / stop preview operation. 
 	 * This call can be modified to START/STOP operation, which can be used in image capture also */
	case V4L2_CID_CAM_PREVIEW_ONOFF:
		if(ctrl->value)
			err = nm6xx_set_preview_start(sd);
		else
			err = nm6xx_set_preview_stop(sd);
		break;

	case V4L2_CID_CAM_UPDATE_FW:
		err = 0;
		break;

	case V4L2_CID_CAM_SET_FW_ADDR:
		err = 0;
		break;

	case V4L2_CID_CAM_SET_FW_SIZE:
		err = 0;
		break;

	case V4L2_CID_CAM_FW_VER:
		err = 0;
		break;

	case V4L2_CID_CAMERA_GET_ISO:
		err = nm6xx_get_iso(sd, ctrl); 	
		break;
	
	case V4L2_CID_CAMERA_GET_SHT_TIME:
		err = nm6xx_get_shutterspeed(sd, ctrl);		
		break;	

	default:
		dev_err(&client->dev, "%s: no such control\n", __func__);
		break;
	}

	if (err < 0)
		dev_err(&client->dev, "%s: vidioc_s_ctrl failed %d\n", __func__, err);

	mutex_unlock(&sensor_s_ctrl);
	return err;
#endif//johnny
}

static int nm6xx_calibration(struct v4l2_subdev *sd)
{	
	unsigned long OTP00, OTP10;
	unsigned long OTP0, OTP1, OTP2; //OTPX0, OTPX1, OTPX2
	unsigned char valid_OPT = 3;
	unsigned char ret0, ret1;	
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	
	printk("nm6xx_calibration : start \n");
	
	ret0 = nm6xx_i2c_read_multi(client, 0x0238, &OTP10);
	ret1 = nm6xx_i2c_read_multi(client, 0x022C, &OTP00);	

	if(ret0 & ret1)
	{
		dev_err(&client->dev, "%s [cam] OPT10=0x%lx, OPT00=0x%lx \n", __func__, OTP10,OTP00);	

		if(((OTP10 & 0x10) >> 4) == 1) 
		{// CASE1 : READ OPT1 DATA
		    OTP0 = OTP10;
		    
		    nm6xx_i2c_read_multi(client, 0x023C, &OTP1);
		    nm6xx_i2c_read_multi(client, 0x0240, &OTP2);
		    valid_OPT = 1;
		}
		else if(((OTP00 & 0x10) >> 4) == 1)
		{// CASE2 : READ OPT0 DATA 
		    OTP0 = OTP00;
		    nm6xx_i2c_read_multi(client, 0x0230, &OTP1);
		    nm6xx_i2c_read_multi(client, 0x0234, &OTP2);
		    valid_OPT = 0;
		}
		else  // if((((OTP10 & 0x10) >> 4) == 0) && (((OTP00 & 0x10) >> 4) == 0 ) )
		{// CASE3 : Default cal. 
		    // Module was not calibrated in module vendor.
		    valid_OPT = 2;
		    return 0;
		}
	}
	else
	{
	    dev_err(&client->dev,"%s [cam] CALIBRATION : READ FAIL \n", __func__); 
	    return 0;
	}

	dev_err(&client->dev,"%s [cam] valid_OPT = %d \n", __func__, valid_OPT); 
	dev_err(&client->dev,"%s [cam] OTP2=0x%lx, OTP1=0x%lx, OTP0=0x%lx \n", __func__, OTP2, OTP1, OTP0); 


	// Shading Cal. 적용
	if (valid_OPT == 1 || valid_OPT ==0)
	{ //CASE 1 || CASE 2:
	//Shading Index : OPTx0 [14-13]

	    unsigned char shading_index;
	    int err = 0;
	    
	    shading_index = (OTP0 & 0x6000) >> 13;
	    dev_err(&client->dev,"%s [cam] Shading Cal. : shading_index = %d  \n", __func__, shading_index); 

	    if (shading_index == 1) // 01
		{
			err = nm6xx_write_regs(sd, \
					nm6xx_shading_2, \
					sizeof(nm6xx_shading_2) / sizeof(nm6xx_shading_2[0]), \
					"nm6xx_shading_2");
			if (err < 0)
			{
				nm6xx_msg(&client->dev, "%s: register write fail \n", __func__);	
				return -EIO;
			}		
		}
	    else if (shading_index== 2) // 10
		{
			err = nm6xx_write_regs(sd, \
					nm6xx_shading_3, \
					sizeof(nm6xx_shading_3) / sizeof(nm6xx_shading_3[0]), \
					"nm6xx_shading_3");
			if (err < 0)
			{
				nm6xx_msg(&client->dev, "%s: register write fail \n", __func__);	
				return -EIO;
			}		
		}
	    else //  if (shading_index == 0), 00 or 11
		{
			err = nm6xx_write_regs(sd, \
					nm6xx_shading_1, \
					sizeof(nm6xx_shading_1) / sizeof(nm6xx_shading_1[0]), \
					"nm6xx_shading_1");
			if (err < 0)
			{
				nm6xx_msg(&client->dev, "%s: register write fail \n", __func__);	
				return -EIO;
			}		
		}
	}


	// AWB Cal. 적용

	{
		unsigned short NORMR, NORMB; //14bit
		unsigned short AWBPRER, AWBPREB; //10bit

		NORMR = ((OTP1 & 0x3F) << 8) | ((OTP0 & 0xFF000000) >> 24);
		dev_err(&client->dev,"%s [cam] NORMR = 0x%x \n", __func__, NORMR); 
		if(NORMR <= 0x3FFF)
			nm6xx_i2c_write_multi(client, 0x4A04, NORMR, 2);


		NORMB =  ((OTP1 & 0xFFFC0) >> 6);
		dev_err(&client->dev,"%s [cam] NORMB = 0x%x \n", __func__, NORMB); 
		if(NORMB <= 0x3FFF)
			nm6xx_i2c_write_multi(client, 0x4A06, NORMB, 2);


		AWBPRER = ((OTP1 & 0x3FF00000) >> 20);
		dev_err(&client->dev,"%s [cam] AWBPRER = 0x%x \n", __func__, AWBPRER); 
		if(AWBPRER <= 0x3FF)
			nm6xx_i2c_write_multi(client, 0x4A08, AWBPRER, 2);


		AWBPREB = ((OTP2 & 0xFF) << 2) | ((OTP1 & 0xC0000000) >> 30);
		dev_err(&client->dev,"%s [cam] AWBPREB = 0x%x \n", __func__, AWBPREB); 
		if(AWBPREB <= 0x3FF)
			nm6xx_i2c_write_multi(client, 0x4A0A, AWBPREB, 2);

	}

	dev_err(&client->dev,"%s [cam] CALIBRATION : END \n", __func__); 
	return 1;		
}

static int  nm6xx_set_default_calibration(struct v4l2_subdev *sd)
{
	int err;

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_err(&client->dev,"%s [cam] nm6xx_set_default_calibration  \n", __func__); 

	err = nm6xx_write_regs(sd, \
			nm6xx_default_calibration, \
			sizeof(nm6xx_default_calibration) / sizeof(nm6xx_default_calibration[0]), \
			"nm6xx_default_calibration");
	if (err < 0)
	{
		nm6xx_msg(&client->dev, "%s: register write fail \n", __func__);	
		return -EIO;
	}    
	return 1;
}

static int nm6xx_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL, i, count=0;
	unsigned short read_value_1, read_value_2;

	dev_err(&client->dev, "%s: init setting~~~~~~~~~~~~~~\n", __func__);
	printk("==================  nm6xx_init -----------------------------\n");
#ifdef CONFIG_VIDEO_NM6XX //johnny.kim
	nm6xx_init_parameters(sd);
	msleep(20);	
	return 0;
#else
	nm6xx_init_parameters(sd);

	msleep(10);	
#ifdef CONFIG_LOAD_FILE
	printk("[BestIQ] + nm6xx_init\n");
	err = nm6xx_regs_table_init();
	if (err) {
		nm6xx_msg(&client->dev, "%s: config file read fail\n", __func__);
		return -EIO;
	}
	msleep(100);
	
	nm6xx_write_regs(sd, nm6xx_init_reg, NM6XX_INIT_REGS, "nm6xx_init_reg");
	msleep(20);	

	if (nm6xx_calibration(sd) == FALSE)
	{
	    nm6xx_set_default_calibration(sd);
	}

	dev_err(&client->dev, "%s: nm6xx_init_image_tuning_setting~~~~~~~~~~~~~~\n", __func__);		
	nm6xx_write_regs(sd, nm6xx_init_image_tuning_setting, NM6XX_INIT_IMAGETUNING_SETTING_REGS, "nm6xx_init_image_tuning_setting");
	printk("[BestIQ] - nm6xx_init\n");

	nm6xx_cam_stdby(TRUE);//bestiq  standby pin
#else

	for(i = 0; i <NM6XX_INIT_REGS; i++)
	{
		err = nm6xx_i2c_write_multi(client, nm6xx_init_reg[i].subaddr ,  nm6xx_init_reg[i].value,  nm6xx_init_reg[i].len);
		if (err < 0)
		{
			nm6xx_msg(&client->dev, "%s: register read fail \n", __func__);	
			return -EIO;
		}		
	}

	msleep(5);

	if (nm6xx_calibration(sd) == FALSE)
	{
	    nm6xx_set_default_calibration(sd);
	}
	
	dev_err(&client->dev, "%s: nm6xx_init_image_tuning_setting~~~~~~~~~~~~~~\n", __func__);		
	for(i = 0; i <NM6XX_INIT_IMAGETUNING_SETTING_REGS; i++)
	{
		err = nm6xx_i2c_write_multi(client, nm6xx_init_image_tuning_setting[i].subaddr ,  nm6xx_init_image_tuning_setting[i].value ,  nm6xx_init_image_tuning_setting[i].len);
		if (err < 0)
		{
			nm6xx_msg(&client->dev, "%s: register read fail \n", __func__);	
			return -EIO;
		}		
	}	

	nm6xx_cam_stdby(TRUE);//bestiq  standby pin
#endif	
//Can use AF Command
	do
	{
		count++;
		nm6xx_i2c_read(client, 0x000A, &read_value_1);
		dev_err(&client->dev, "%s: i2c_read --- read_value_1 == 0x%x \n", __func__, read_value_1);

 		nm6xx_i2c_read(client, 0x6D76, &read_value_2);
		dev_err(&client->dev, "%s: i2c_read --- read_value_2 == 0x%x \n", __func__, read_value_2);
		msleep(10);
		/*
		 * Arun c
		 * When the esd error occures during init the while loop never returns
		 * so keep a count of the loops
		 */
		if (count > 100)
			break;
	}while((!(read_value_1&0x02))&&(!(read_value_2&0x03)));		

	
	return err;
#endif
}

/*
 * s_config subdev ops
 * With camera device, we need to re-initialize every single opening time therefor,
 * it is not necessary to be initialized on probe time. except for version checking
 * NOTE: version checking is optional
 */
static int nm6xx_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct nm6xx_state *state = to_state(sd);
	struct nm6xx_platform_data *pdata;

	pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -ENODEV;
	}

	/*
	 * Assign default format and resolution
	 * Use configured default information in platform data
	 * or without them, use default information in driver
	 */
	if (!(pdata->default_width && pdata->default_height)) {
		/* TODO: assign driver default resolution */
	} else {
		state->pix.width = pdata->default_width;
		state->pix.height = pdata->default_height;
	}

	if (!pdata->pixelformat)
		state->pix.pixelformat = DEFAULT_PIX_FMT;
	else
		state->pix.pixelformat = pdata->pixelformat;

	if (!pdata->freq)
		state->freq = DEFUALT_MCLK;	/* 24MHz default */
	else
		state->freq = pdata->freq;

	return 0;
}

static const struct v4l2_subdev_core_ops nm6xx_core_ops = {
	.init = nm6xx_init,	/* initializing API */
	.s_config = nm6xx_s_config,	/* Fetch platform data */
	.queryctrl = nm6xx_queryctrl,
	.querymenu = nm6xx_querymenu,
	.g_ctrl = nm6xx_g_ctrl,
	.s_ctrl = nm6xx_s_ctrl,
};

static const struct v4l2_subdev_video_ops nm6xx_video_ops = {
	.s_crystal_freq = nm6xx_s_crystal_freq,
	.g_fmt = nm6xx_g_fmt,
	.s_fmt = nm6xx_s_fmt,
	.enum_framesizes = nm6xx_enum_framesizes,
	.enum_frameintervals = nm6xx_enum_frameintervals,
	.enum_fmt = nm6xx_enum_fmt,
	.try_fmt = nm6xx_try_fmt,
	.g_parm = nm6xx_g_parm,
	.s_parm = nm6xx_s_parm,
};

static const struct v4l2_subdev_ops nm6xx_ops = {
	.core = &nm6xx_core_ops,
	.video = &nm6xx_video_ops,
};

/*
 * nm6xx_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
static int nm6xx_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct nm6xx_state *state;
	struct v4l2_subdev *sd;

	CDBG("probing.................................................... \n");
	
	state = kzalloc(sizeof(struct nm6xx_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	state->runmode = NM6XX_RUNMODE_NOTREADY;

	sd = &state->sd;
	strcpy(sd->name, NM6XX_DRIVER_NAME);

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &nm6xx_ops);

	init_timer(&af_cancel_timer);
	af_cancel_timer.expires  = (get_jiffies_64() + msecs_to_jiffies(100));
	af_cancel_timer.function = af_cancel_handler;

	nm6xx_info(&client->dev, "Analog TV NM6XX loaded.\n");

	return 0;
}

static int nm6xx_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	CDBG("nm6xx_remove.................................................... \n");

//	nm6xx_i2c_write_multi(client, 0x0008, 0x0001, 1);	//set standby in the register
//bestiq	msleep(200);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));

	del_timer(&af_cancel_timer);
	nm6xx_info(&client->dev, "Unloaded camera sensor NM6XX.\n");

	return 0;
}

static const struct i2c_device_id nm6xx_id[] = {
	{ NM6XX_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nm6xx_id);

static struct v4l2_i2c_driver_data v4l2_i2c_data = {
	.name = NM6XX_DRIVER_NAME,
	.probe = nm6xx_probe,
	.remove = nm6xx_remove,
	.id_table = nm6xx_id,
};

MODULE_DESCRIPTION("NEC NM6XX-SONY 3MP camera driver");
MODULE_AUTHOR("Tushar Behera <tushar.b@samsung.com>");
MODULE_LICENSE("GPL");
