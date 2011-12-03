/* linux/arch/arm/mach-s5pv210/mach-p1.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/gpio.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/max8998.h>
#include <linux/smb136_charger.h>
#include <linux/sec_battery.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/usb/ch9.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/clk.h>
#include <linux/usb/ch9.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/skbuff.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/gpio.h>
#include <mach/gpio-p1.h>
#include <mach/mach-p1.h>
#include <mach/sec_switch.h>
#include <mach/adc.h>
#include <mach/param.h>
#include <mach/system.h>

#ifdef CONFIG_SEC_HEADSET
#include <mach/sec_jack.h>
#endif

#include <linux/usb/gadget.h>
#include <linux/fsa9480.h>
#if defined(CONFIG_PN544)
#include <linux/pn544.h>
#endif
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/wlan_plat.h>
#include <linux/mfd/wm8994/wm8994_pdata.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#include <plat/media.h>
#include <mach/media.h>
#endif

#ifdef CONFIG_S5PV210_POWER_DOMAIN
#include <mach/power-domain.h>
#endif

#ifdef CONFIG_VIDEO_ISX005
#include <media/isx005_platform.h>
#endif
#ifdef CONFIG_VIDEO_S5K6AAFX
#include <media/s5k6aafx_platform.h>
#endif
#ifdef CONFIG_VIDEO_S5K5CCGX
#include <media/s5k5ccgx_platform.h>
#endif
#ifdef CONFIG_VIDEO_NM6XX 
#include <media/nm6xx_platform.h>
#endif

#include <plat/regs-serial.h>
#include <plat/s5pv210.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/fb.h>
#include <plat/mfc.h>
#include <plat/iic.h>
#include <plat/pm.h>

#include <plat/sdhci.h>
#include <plat/fimc.h>
#include <plat/jpeg.h>
#include <plat/clock.h>
#include <plat/regs-otg.h>
#include <linux/bh1721.h>
#include <linux/i2c/l3g4200d.h>
#include <../../../drivers/sensor/accel/bma020.h>
#include <../../../drivers/video/samsung/s3cfb.h>
#ifdef CONFIG_SAMSUNG_JACK
#include <linux/sec_jack.h>
#endif
#include <linux/input/mxt224.h>
#include <linux/max17042_battery.h>
#include <linux/mfd/max8998.h>
#include <linux/switch.h>
#include <linux/cpufreq.h>

#if defined(CONFIG_KEYBOARD_GPIO)
#include <linux/gpio_keys.h>
#else
#if defined(CONFIG_INPUT_GPIO)
#include <linux/gpio_event.h>
#endif
#endif

#if defined(CONFIG_DVFS_LIMIT)
#include <mach/cpu-freq-v210.h>
#endif

#include "herring.h"

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

struct device *switch_dev;
EXPORT_SYMBOL(switch_dev);

void (*sec_set_param_value)(int idx, void *value);
EXPORT_SYMBOL(sec_set_param_value);

void (*sec_get_param_value)(int idx, void *value);
EXPORT_SYMBOL(sec_get_param_value);

#define KERNEL_REBOOT_MASK      0xFFFFFFFF
#define REBOOT_MODE_FAST_BOOT		7

#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];
EXPORT_SYMBOL(wlan_static_skb);

struct sec_battery_callbacks *callbacks;
struct max17042_callbacks *max17042_cb;
static enum cable_type_t set_cable_status;
static enum charging_status_type_t charging_status;
static int fsa9480_init_flag = 0;
static int sec_switch_status = 0;
static int sec_switch_inited = 0;
static bool fsa9480_jig_status = 0;
static bool ap_vbus_disabled = 0;

void sec_switch_set_regulator(int mode);
void otg_phy_init(void);

struct wifi_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static int crespo_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	int mode = REBOOT_MODE_NONE;
	unsigned int temp;

	if ((code == SYS_RESTART) && _cmd) {
		if (!strcmp((char *)_cmd, "arm11_fota"))
			mode = REBOOT_MODE_ARM11_FOTA;
		else if (!strcmp((char *)_cmd, "arm9_fota"))
			mode = REBOOT_MODE_ARM9_FOTA;
		else if (!strcmp((char *)_cmd, "recovery"))
			mode = REBOOT_MODE_RECOVERY;
		else if (!strcmp((char *)_cmd, "download")) 
			mode = REBOOT_MODE_DOWNLOAD;
		else if (!strcmp((char *)_cmd, "factory_reboot")) 
			mode = REBOOT_MODE_NONE;
	}
	
	if(code != SYS_POWER_OFF) {
		if(sec_set_param_value)	{
			sec_set_param_value(__REBOOT_MODE, &mode);
		}
	}

	return NOTIFY_DONE;
}

static struct notifier_block crespo_reboot_notifier = {
	.notifier_call = crespo_notifier_call,
};

#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
static void gps_gpio_init(void)
{
	struct device *gps_dev;

	gps_dev = device_create(sec_class, NULL, 0, NULL, "gps");
	if (IS_ERR(gps_dev)) {
		pr_err("Failed to create device(gps)!\n");
		goto err;
	}

	gpio_request(GPIO_GPS_nRST, "GPS_nRST");	/* XMMC3CLK */
	s3c_gpio_setpull(GPIO_GPS_nRST, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_GPS_nRST, S3C_GPIO_OUTPUT);
	gpio_direction_output(GPIO_GPS_nRST, 1);

	gpio_request(GPIO_GPS_PWR_EN, "GPS_PWR_EN");	/* XMMC3CLK */
	s3c_gpio_setpull(GPIO_GPS_PWR_EN, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_GPS_PWR_EN, S3C_GPIO_OUTPUT);
	gpio_direction_output(GPIO_GPS_PWR_EN, 0);

	s3c_gpio_setpull(GPIO_GPS_RXD, S3C_GPIO_PULL_UP);
	gpio_export(GPIO_GPS_nRST, 1);
	gpio_export(GPIO_GPS_PWR_EN, 1);

	gpio_export_link(gps_dev, "GPS_nRST", GPIO_GPS_nRST);
	gpio_export_link(gps_dev, "GPS_PWR_EN", GPIO_GPS_PWR_EN);

 err:
	return;
}
#endif

static void uart_switch_init(void)
{
	int ret;
	struct device *uartswitch_dev;

	uartswitch_dev = device_create(sec_class, NULL, 0, NULL, "uart_switch");
	if (IS_ERR(uartswitch_dev)) {
		pr_err("Failed to create device(uart_switch)!\n");
		return;
	}

	ret = gpio_request(GPIO_UART_SEL, "UART_SEL");
	if (ret < 0) {
		pr_err("Failed to request GPIO_UART_SEL!\n");
		return;
	}
	s3c_gpio_setpull(GPIO_UART_SEL, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_UART_SEL, S3C_GPIO_OUTPUT);
//	gpio_direction_output(GPIO_UART_SEL, 1);

	gpio_export(GPIO_UART_SEL, 1);

	gpio_export_link(uartswitch_dev, "UART_SEL", GPIO_UART_SEL);
}

static void herring_switch_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");

	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");

	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");

	if (IS_ERR(switch_dev))
		pr_err("Failed to create device(switch)!\n");
};

/* << additional feature - end */
#define SOC_DUALCAM_POWERCTRL

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define S5PV210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define S5PV210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define S5PV210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg crespo_uartcfgs[] __initdata = {
	{
		.hwport		= 0,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
#ifdef CONFIG_MACH_HERRING
		.wake_peer	= herring_bt_uart_wake_peer,
#endif
	},
	{
		.hwport		= 1,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
#ifndef CONFIG_FIQ_DEBUGGER
	{
		.hwport		= 2,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
#endif
	{
		.hwport		= 3,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
};

static struct s3cfb_lcd s6e63m0 = {
	.width = 480,
	.height = 800,
	.p_width = 52,
	.p_height = 86,
	.bpp = 24,
	.freq = 60,

	.timing = {
		.h_fp = 16,
		.h_bp = 16,
		.h_sw = 2,
		.v_fp = 28,
		.v_fpe = 1,
		.v_bp = 1,
		.v_bpe = 1,
		.v_sw = 2,
	},
	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 1,
	},
};

static struct s3cfb_lcd lvds = {
        .width = 1024,
        .height = 600,
		.p_width = 154,
		.p_height = 90,
		.bpp = 24,
        .freq = 60,

        .timing = {
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
                .h_fp = 142,    //50,	//179,	//.h_fp = 79,
                .h_bp = 210,    //30,	//225,	//.h_bp = 200,
#elif defined(CONFIG_MACH_P1_CDMA)
                .h_fp = 100,    //50,	//179,	//.h_fp = 79,
                .h_bp = 80,    //30,	//225,	//.h_bp = 200,
#endif
                .h_sw = 50,     //20,	//40,
                .v_fp = 10,     //6,	//10,
                .v_fpe = 1,
                .v_bp = 11,     //5,	//11,
                .v_bpe = 1,
                .v_sw = 10,     // 4,	//10,

        },

        .polarity = {
                .rise_vclk = 0,
                .inv_hsync = 1,
                .inv_vsync = 1,
                .inv_vden = 0,
        },
};

#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC0 		(8192 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC1 		(9900 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC2 		(8192 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC0 		(36864 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC1 		(36864 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMD 		(4800 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_JPEG 		(14100 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_PMEM 		(8192 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_PMEM_GPU1 	(4200 * SZ_1K)
#define  S5PV210_ANDROID_PMEM_MEMSIZE_PMEM_ADSP 	(1500 * SZ_1K)
#define  S5PV210_VIDEO_SAMSUNG_MEMSIZE_TEXSTREAM 	(4800 * SZ_1K)

static struct s5p_media_device crespo_media_devs[] = {
	[0] = {
		.id = S5P_MDEV_MFC,
		.name = "mfc",
		.bank = 0,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC0,
		.paddr = 0,
	},
	[1] = {
		.id = S5P_MDEV_MFC,
		.name = "mfc",
		.bank = 1,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_MFC1,
		.paddr = 0,
	},
	[2] = {
		.id = S5P_MDEV_FIMC0,
		.name = "fimc0",
		.bank = 1,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC0,
		.paddr = 0,
	},
	[3] = {
		.id = S5P_MDEV_FIMC1,
		.name = "fimc1",
		.bank = 1,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC1,
		.paddr = 0,
	},
	[4] = {
		.id = S5P_MDEV_FIMC2,
		.name = "fimc2",
		.bank = 1,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMC2,
		.paddr = 0,
	},
	[5] = {
		.id = S5P_MDEV_JPEG,
		.name = "jpeg",
		.bank = 0,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_JPEG,
		.paddr = 0,
	},
	[6] = {
		.id = S5P_MDEV_FIMD,
		.name = "fimd",
		.bank = 1,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_FIMD,
		.paddr = 0,
	},
	[7] = {
		.id = S5P_MDEV_PMEM,
		.name = "pmem",
		.bank = 0,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_PMEM,
		.paddr = 0,
	},
	[8] = {
		.id = S5P_MDEV_PMEM_GPU1,
		.name = "pmem_gpu1",
		.bank = 0,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_PMEM_GPU1,
		.paddr = 0,
	},
	[9] = {
		.id = S5P_MDEV_PMEM_ADSP,
		.name = "pmem_adsp",
		.bank = 0,
		.memsize = S5PV210_ANDROID_PMEM_MEMSIZE_PMEM_ADSP,
		.paddr = 0,
	},
	[10] = {
		.id = S5P_MDEV_TEXSTREAM,
		.name = "texstream",
		.bank = 0,
		.memsize = S5PV210_VIDEO_SAMSUNG_MEMSIZE_TEXSTREAM,
		.paddr = 0,
	},
	[11] = {
		.id = S3C_MDEV_WIFI,
		.name = "wifi",
		.bank = 0,
		.memsize = 256 * SZ_1K,
		.paddr = 0,
	},
};

/* MAX8998 LDO */
static struct regulator_consumer_supply ldo3_consumer[] = {
	REGULATOR_SUPPLY("pd_io", "s3c-usbgadget"),
	{	.supply	= "tv_pll", },
};

static struct regulator_consumer_supply ldo4_consumer[] = {
                {       .supply = "v_adc", },
};

static struct regulator_consumer_supply ldo7_consumer[] = {
	{	.supply	= "vcc_vtf", },
};

static struct regulator_consumer_supply ldo8_consumer[] = {
	REGULATOR_SUPPLY("pd_core", "s3c-usbgadget"),
	{	.supply	= "tv_osc", },
};

static struct regulator_consumer_supply ldo11_consumer[] = {
	{	.supply	= "cam_io", },
};

static struct regulator_consumer_supply ldo12_consumer[] = {
	{	.supply	= "cam_cif", },
};

static struct regulator_consumer_supply ldo13_consumer[] = {
	{	.supply	= "cam_analog", },
};

static struct regulator_consumer_supply ldo14_consumer[] = {
	{	.supply	= "cam_3m", },
};

static struct regulator_consumer_supply ldo15_consumer[] = {
	{	.supply	= "cam_af", },
};

static struct regulator_consumer_supply ldo16_consumer[] = {
	{	.supply	= "vcc_motor", },
};

static struct regulator_consumer_supply ldo17_consumer[] = {
	{	.supply	= "vcc_lcd", },
};

static struct regulator_consumer_supply buck1_consumer[] = {
	{	.supply	= "vddarm", },
};

static struct regulator_consumer_supply buck2_consumer[] = {
	{	.supply	= "vddint", },
};

static struct regulator_consumer_supply buck3_consumer[] = {
        {       .supply = "vcc_ram", },
};

static struct regulator_consumer_supply safeout1_consumer[] = {
	{	.supply	= "vbus_ap", },
};

static struct regulator_consumer_supply safeout2_consumer[] = {
	{	.supply	= "vbus_cp", },
};

static struct regulator_init_data crespo_ldo2_data = {
	.constraints	= {
		.name		= "VALIVE_1.2V",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled = 1,
		},
	},
};

static struct regulator_init_data crespo_ldo3_data = {
	.constraints	= {
		.name		= "VUSB_1.1V",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo3_consumer),
	.consumer_supplies	= ldo3_consumer,
};

static struct regulator_init_data crespo_ldo4_data = {
	.constraints	= {
		.name		= "VADC_3.3V",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem      = {
			.uV     = 3000000,
			.mode   = REGULATOR_MODE_NORMAL,
 			.disabled = 1,
 		},
 	},
        .num_consumer_supplies  = ARRAY_SIZE(ldo4_consumer),
        .consumer_supplies      = ldo4_consumer,
 };

static struct regulator_init_data crespo_ldo7_data = {
	.constraints	= {
		.name		= "VTF_2.8V",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo7_consumer),
	.consumer_supplies	= ldo7_consumer,
};

static struct regulator_init_data crespo_ldo8_data = {
	.constraints	= {
		.name		= "VUSB_3.3V",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo8_consumer),
	.consumer_supplies	= ldo8_consumer,
};

static struct regulator_init_data crespo_ldo9_data = {
	.constraints	= {
		.name		= "VCC_2.8V_PDA",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.always_on	= 1,
	},
};

static struct regulator_init_data crespo_ldo11_data = {
	.constraints	= {
		.name		= "CAM_IO_2.8V",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo11_consumer),
	.consumer_supplies	= ldo11_consumer,
};

static struct regulator_init_data crespo_ldo12_data = {
	.constraints	= {
		.name		= "CAM_CIF_1.5V",
		.min_uV		= 1500000,
		.max_uV		= 1500000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo12_consumer),
	.consumer_supplies	= ldo12_consumer,
};

static struct regulator_init_data crespo_ldo13_data = {
	.constraints	= {
		.name		= "CAM_A_2.8V",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo13_consumer),
	.consumer_supplies	= ldo13_consumer,
};

static struct regulator_init_data crespo_ldo14_data = {
	.constraints	= {
		.name		= "CAM_3M_1.2V",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo14_consumer),
	.consumer_supplies	= ldo14_consumer,
};

static struct regulator_init_data crespo_ldo15_data = {
	.constraints	= {
		.name		= "CAM_AF_3.0V",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo15_consumer),
	.consumer_supplies	= ldo15_consumer,
};

static struct regulator_init_data crespo_ldo16_data = {
	.constraints	= {
		.name		= "MOTOR_3.0V",
		.min_uV		= 3400000,
		.max_uV		= 3400000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo16_consumer),
	.consumer_supplies	= ldo16_consumer,
};

static struct regulator_init_data crespo_ldo17_data = {
	.constraints	= {
		.name		= "LVDS_VDD3.3V",
		.min_uV		= 2600000,
		.max_uV		= 2600000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
                        .uV     = 2600000,
                        .mode   = REGULATOR_MODE_NORMAL,
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo17_consumer),
	.consumer_supplies	= ldo17_consumer,
};

static struct regulator_init_data crespo_buck1_data = {
	.constraints	= {
		.name		= "VDD_ARM",
		.min_uV		= 750000,
		.max_uV		= 1500000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.uV	= 1250000,
			.mode	= REGULATOR_MODE_NORMAL,
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(buck1_consumer),
	.consumer_supplies	= buck1_consumer,
};


static struct regulator_init_data crespo_buck2_data = {
	.constraints	= {
		.name		= "VDD_INT",
		.min_uV		= 750000,
		.max_uV		= 1500000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.uV	= 1100000,
			.mode	= REGULATOR_MODE_NORMAL,
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(buck2_consumer),
	.consumer_supplies	= buck2_consumer,
};

static struct regulator_init_data crespo_buck3_data = {
	.constraints	= {
		.name		= "VCC_1.8V",
		.min_uV		= 1700000,
		.max_uV		= 1700000,
		.apply_uV	= 1,
		.boot_on        = 1,
		.always_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
		                  REGULATOR_CHANGE_STATUS,
		.state_mem      = {
                        .uV     = 1700000,
			.mode   = REGULATOR_MODE_NORMAL,
			.disabled = 1,
                },
	},
        .num_consumer_supplies  = ARRAY_SIZE(buck3_consumer),
        .consumer_supplies      = buck3_consumer,
};

static struct regulator_init_data crespo_safeout1_data = {
	.constraints	= {
		.name		= "USB_VBUS_AP",
		.min_uV		= 5000000,
		.max_uV		= 5000000,
		.apply_uV	= 1,
		.valid_ops_mask	=  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout1_consumer),
	.consumer_supplies	= safeout1_consumer,
};

static struct regulator_init_data crespo_safeout2_data = {
	.constraints	= {
		.name		= "USB_VBUS_CP",
		.min_uV		= 5000000,
		.max_uV		= 5000000,
		.apply_uV	= 1,
		.valid_ops_mask	=  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout2_consumer),
	.consumer_supplies	= safeout2_consumer,
};

static struct max8998_regulator_data crespo_regulators[] = {
	{ MAX8998_LDO2,  &crespo_ldo2_data },
	{ MAX8998_LDO3,  &crespo_ldo3_data },
	{ MAX8998_LDO4,  &crespo_ldo4_data },
	{ MAX8998_LDO7,  &crespo_ldo7_data },
	{ MAX8998_LDO8,  &crespo_ldo8_data },
	{ MAX8998_LDO9,  &crespo_ldo9_data },
	{ MAX8998_LDO11, &crespo_ldo11_data },
	{ MAX8998_LDO12, &crespo_ldo12_data },
	{ MAX8998_LDO13, &crespo_ldo13_data },
	{ MAX8998_LDO14, &crespo_ldo14_data },
	{ MAX8998_LDO15, &crespo_ldo15_data },
	{ MAX8998_LDO16, &crespo_ldo16_data },
	{ MAX8998_LDO17, &crespo_ldo17_data },
	{ MAX8998_BUCK1, &crespo_buck1_data },
	{ MAX8998_BUCK2, &crespo_buck2_data },
	{ MAX8998_BUCK3, &crespo_buck3_data },
	{ MAX8998_ESAFEOUT1, &crespo_safeout1_data },
	{ MAX8998_ESAFEOUT2, &crespo_safeout2_data },
};

static struct sec_battery_adc_table_data temper_table[] =  {
	/* ADC, Temperature (C/10) */
	{ 1830, 		-200},
	{ 1819, 		-190},
	{ 1809, 		-180},
	{ 1798, 		-170},
	{ 1788, 		-160},
	{ 1778, 		-150},
	{ 1767, 		-140},
	{ 1756, 		-130},
	{ 1745, 		-120},
	{ 1734, 		-110},
	{ 1723, 		-100},
	{ 1710, 		-90 },
	{ 1697, 		-80 },
	{ 1685, 		-70 },
	{ 1672, 		-60 },
	{ 1660, 		-50 },
	{ 1638, 		-40 },
	{ 1616, 		-30 },
	{ 1594, 		-20 },
	{ 1572, 		-10 },
	{ 1550, 		0	},
	{ 1532, 		10	},
	{ 1514, 		20	},
	{ 1494, 		30	},
	{ 1478, 		40	},
	{ 1461, 		50	},
	{ 1437, 		60	},
	{ 1414, 		70	},
	{ 1390, 		80	},
	{ 1367, 		90	},
	{ 1344, 		100 },
	{ 1322, 		110 },
	{ 1300, 		120 },
	{ 1279, 		130 },
	{ 1257, 		140 },
	{ 1236, 		150 },
	{ 1208, 		160 },
	{ 1181, 		170 },
	{ 1153, 		180 },
	{ 1126, 		190 },
	{ 1099, 		200 },
	{ 1076, 		210 },
	{ 1054, 		220 },
	{ 1031, 		230 },
	{ 1009,		240 },
	{ 987,		250 },
	{ 965,		260 },
	{ 943,		270 },
	{ 921,		280 },
	{ 899,		290 },
	{ 877,		300 },
	{ 852,		310 },
	{ 828,		320 },
	{ 803,		330 },
	{ 779,		340 },
	{ 755,		350 },
	{ 734,		360 },
	{ 713,		370 },
	{ 693,		380 },
	{ 672,		390 },
	{ 652,		400 },
	{ 634,		410 },
	{ 617,		420 },
	{ 600,		430 },
	{ 583,		440 },
	{ 566,		450 },
	{ 547,		460 },
	{ 528,		470 },
	{ 509,		480 },
	{ 490,		490 },
	{ 471,		500 },
	{ 457,		510 },
	{ 443,		520 },
	{ 429,		530 },
	{ 415,		540 },
	{ 402,		550 },
	{ 389,		560 },
	{ 376,		570 },
	{ 364,		580 },
	{ 351,		590 },
	{ 339,		600 },
	{ 328,		610 },
	{ 317,		620 },
	{ 306,		630 },
	{ 295,		640 },
	{ 284,		650 },
	{ 273,		660 },
	{ 263,		670 },
	{ 252,		680 },
	{ 242,		690 },
	{ 232,		700 },
};


static void sec_battery_register_callbacks(
		struct sec_battery_callbacks *ptr)
{
	callbacks = ptr;
	/* if there was a cable status change before the charger was
	ready, send this now */
	if ((set_cable_status != 0) && callbacks && callbacks->set_cable)
		callbacks->set_cable(callbacks, set_cable_status);

	if ((charging_status != 0) && callbacks && callbacks->set_status)
		callbacks->set_status(callbacks, charging_status);
}

static int max17042_callbacks(int request_mode, int arg1, int arg2)
{
	int ret = 0;

	if(!max17042_cb) {
		printk("%s: callbacks are not registerd!!\n", __func__);
		return -1;
	}

	printk(
"%s: request_mode(%d), arg1(%d), arg2(%d)\n", __func__, request_mode, arg1, arg2);

	switch(request_mode) {
	case REQ_FULL_CHARGE_COMPENSATION:
		if(max17042_cb->full_charge_comp)
			max17042_cb->full_charge_comp(max17042_cb, arg1, arg2);
		break;

	case REQ_VF_FULLCAP_RANGE_CHECK:
		if(max17042_cb->vf_fullcap_check)
			max17042_cb->vf_fullcap_check(max17042_cb);
		break;

	case REQ_CAP_CORRUPTION_CHECK:
		if(max17042_cb->corruption_check)
			ret = max17042_cb->corruption_check(max17042_cb);
		break;

	case REQ_LOW_BATTERY_COMPENSATION:
		if(max17042_cb->low_batt_comp)
			max17042_cb->low_batt_comp(max17042_cb, arg1);
		break;

	case REQ_ADJUST_CAPACITY_RESTART:
		if(max17042_cb->adjust_capacity)
			ret = max17042_cb->adjust_capacity(max17042_cb);
		break;

	case REQ_TEST_MODE_INTERFACE:
		if(max17042_cb->test_mode_request)
			ret = max17042_cb->test_mode_request(max17042_cb,
				(max17042_test_mode_type_t)arg1, arg2);
		break;

	default:
		break;
	}

	return ret;

}

static bool sec_battery_get_jig_status(void)
{
	return fsa9480_jig_status;
}

static struct sec_battery_platform_data sec_battery_pdata = {
	.register_callbacks = &sec_battery_register_callbacks,
	.adc_table		= temper_table,
	.adc_array_size	= ARRAY_SIZE(temper_table),
	.fuelgauge_cb		= &max17042_callbacks,
	.get_jig_status		= &sec_battery_get_jig_status,
};

struct platform_device sec_device_battery = {
	.name	= "sec_battery",
	.id	= -1,
	.dev	= {
		.platform_data	= &sec_battery_pdata,
	}
};

static void sec_bat_set_charging_status(int status)
{
	charging_status = status;  // ERROR : -1, NONE : 0, ACTIVE : 1, FULL : 2

	if (callbacks && callbacks->set_status)
		callbacks->set_status(callbacks, charging_status);
}

static struct charger_device max8998_chgdev = {
	.set_charging_status = sec_bat_set_charging_status,
};

static int max8998_charger_register(struct charger_device *chgdev)
{
	sec_battery_pdata.pmic_charger = chgdev;
	return 0;
}

static void max8998_charger_unregister(struct charger_device *chgdev)
{
	sec_battery_pdata.pmic_charger = NULL;
}

static struct max8998_charger_data max8998_charger = {
	.charger_dev_register = max8998_charger_register,
	.charger_dev_unregister = max8998_charger_unregister,
	.chgdev = &max8998_chgdev,
};

static struct max8998_platform_data max8998_pdata = {
	.num_regulators = ARRAY_SIZE(crespo_regulators),
	.regulators     = crespo_regulators,
	.charger        = &max8998_charger,
	.irq_base	= IRQ_MAX8998_BASE,
};

struct platform_device sec_device_dpram = {
	.name	= "dpram-device",
	.id	= -1,
};

static void lvds_cfg_gpio(struct platform_device *pdev)
{
        int i,err;

        for (i = 0; i < 8; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF0(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF0(i), S3C_GPIO_PULL_NONE);
        }

        for (i = 0; i < 8; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF1(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF1(i), S3C_GPIO_PULL_NONE);
        }

        for (i = 0; i < 8; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF2(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF2(i), S3C_GPIO_PULL_NONE);
        }

        for (i = 0; i < 4; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF3(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF3(i), S3C_GPIO_PULL_NONE);
        }

        /* mDNIe SEL: why we shall write 0x2 ? */
#ifndef CONFIG_FB_S3C_MDNIE
        writel(0x2, S5P_MDNIE_SEL);
#else
        writel(0x1, S5P_MDNIE_SEL);
#endif

        /* set drive strength to max */
//        writel(0xffffffff, S5P_VA_GPIO + 0x12c);
//        writel(0xffffffff, S5P_VA_GPIO + 0x14c);
//        writel(0xffffffff, S5P_VA_GPIO + 0x16c);
//        writel(0x000000ff, S5P_VA_GPIO + 0x18c);
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
        writel(0x5555557f, S5P_VA_GPIO + 0x12c);
#elif defined(CONFIG_MACH_P1_CDMA)
        writel(0x555555bf, S5P_VA_GPIO + 0x12c);
#endif
        writel(0x55555555, S5P_VA_GPIO + 0x14c);
        writel(0x55555555, S5P_VA_GPIO + 0x16c);
        writel(0x00000055, S5P_VA_GPIO + 0x18c);

}

static int lvds_reset_lcd(struct platform_device *pdev)
{
        int err=0;

	return err;
}

static int lvds_backlight_on(struct platform_device *pdev)
{
        int err=0;

        return err;
}

static struct s3c_platform_fb lvds_data __initdata = {
	.hw_ver = 0x62,
	.clk_name = "sclk_fimd",		//"lcd",
	.nr_wins = 5,
	.default_win = CONFIG_FB_S3C_DEFAULT_WINDOW,
	.swap = FB_SWAP_HWORD | FB_SWAP_WORD,

	.lcd = &lvds,
	.cfg_gpio = lvds_cfg_gpio,
	.backlight_on = lvds_backlight_on,
	.reset_lcd = lvds_reset_lcd,
};

#if defined(CONFIG_FB_S3C_CMC623)
static struct platform_device cmc623_pwm_backlight = {
	.name   = "cmc623_pwm_bl",
	.id		= -1,
	.dev	= {
		.parent		= &s3c_device_fb.dev,
	},
};
#endif

static struct platform_device sec_device_lms700 = {
	.name   = "lms700",
	.id		= -1,
};

static void tl2796_cfg_gpio(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF0(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF0(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF1(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF1(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF2(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF2(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 4; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF3(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF3(i), S3C_GPIO_PULL_NONE);
	}

	/* mDNIe SEL: why we shall write 0x2 ? */
#ifdef CONFIG_FB_S3C_MDNIE
	writel(0x1, S5P_MDNIE_SEL);
#else
	writel(0x2, S5P_MDNIE_SEL);
#endif

	/* DISPLAY_CS */
	s3c_gpio_cfgpin(S5PV210_MP01(1), S3C_GPIO_SFN(1));
	/* DISPLAY_CLK */
	s3c_gpio_cfgpin(S5PV210_MP04(1), S3C_GPIO_SFN(1));
	/* DISPLAY_SO */
	s3c_gpio_cfgpin(S5PV210_MP04(2), S3C_GPIO_SFN(1));
	/* DISPLAY_SI */
	s3c_gpio_cfgpin(S5PV210_MP04(3), S3C_GPIO_SFN(1));

	/* DISPLAY_CS */
	s3c_gpio_setpull(S5PV210_MP01(1), S3C_GPIO_PULL_NONE);
	/* DISPLAY_CLK */
	s3c_gpio_setpull(S5PV210_MP04(1), S3C_GPIO_PULL_NONE);
	/* DISPLAY_SO */
	s3c_gpio_setpull(S5PV210_MP04(2), S3C_GPIO_PULL_NONE);
	/* DISPLAY_SI */
	s3c_gpio_setpull(S5PV210_MP04(3), S3C_GPIO_PULL_NONE);
}

void lcd_cfg_gpio_early_suspend(void)
{
	int i;

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF0(i), S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(S5PV210_GPF0(i), S3C_GPIO_PULL_NONE);
		gpio_set_value(S5PV210_GPF0(i), 0);
	}

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF1(i), S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(S5PV210_GPF1(i), S3C_GPIO_PULL_NONE);
		gpio_set_value(S5PV210_GPF1(i), 0);
	}

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF2(i), S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(S5PV210_GPF2(i), S3C_GPIO_PULL_NONE);
		gpio_set_value(S5PV210_GPF2(i), 0);
	}

	for (i = 0; i < 4; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF3(i), S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(S5PV210_GPF3(i), S3C_GPIO_PULL_NONE);
		gpio_set_value(S5PV210_GPF3(i), 0);
	}
	/* drive strength to min */
	writel(0x00000000, S5P_VA_GPIO + 0x12c); /* GPF0DRV */
	writel(0x00000000, S5P_VA_GPIO + 0x14c); /* GPF1DRV */
	writel(0x00000000, S5P_VA_GPIO + 0x16c); /* GPF2DRV */
	writel(0x00000000, S5P_VA_GPIO + 0x18c); /* GPF3DRV */

	/* OLED_DET */
	s3c_gpio_cfgpin(GPIO_OLED_DET, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_OLED_DET, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_OLED_DET, 0);

	/* LCD_RST */
	s3c_gpio_cfgpin(GPIO_MLCD_RST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_MLCD_RST, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_MLCD_RST, 0);

	/* DISPLAY_CS */
	s3c_gpio_cfgpin(GPIO_DISPLAY_CS, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_DISPLAY_CS, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_DISPLAY_CS, 0);

	/* DISPLAY_CLK */
	s3c_gpio_cfgpin(GPIO_DISPLAY_CLK, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_DISPLAY_CLK, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_DISPLAY_CLK, 0);

	/* DISPLAY_SO */
	/*
	s3c_gpio_cfgpin(S5PV210_MP04(2), S3C_GPIO_INPUT);
	s3c_gpio_setpull(S5PV210_MP04(2), S3C_GPIO_PULL_DOWN);
	*/

	/* DISPLAY_SI */
	s3c_gpio_cfgpin(GPIO_DISPLAY_SI, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_DISPLAY_SI, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_DISPLAY_SI, 0);

	/* OLED_ID */
	s3c_gpio_cfgpin(GPIO_OLED_ID, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_OLED_ID, S3C_GPIO_PULL_DOWN);
	/* gpio_set_value(GPIO_OLED_ID, 0); */

	/* DIC_ID */
	s3c_gpio_cfgpin(GPIO_DIC_ID, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_DIC_ID, S3C_GPIO_PULL_DOWN);
	/* gpio_set_value(GPIO_DIC_ID, 0); */
}
EXPORT_SYMBOL(lcd_cfg_gpio_early_suspend);

void lcd_cfg_gpio_late_resume(void)
{
	/* OLED_DET */
	s3c_gpio_cfgpin(GPIO_OLED_DET, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_OLED_DET, S3C_GPIO_PULL_NONE);
	/* OLED_ID */
	s3c_gpio_cfgpin(GPIO_OLED_ID, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_OLED_ID, S3C_GPIO_PULL_NONE);
	/* gpio_set_value(GPIO_OLED_ID, 0); */
	/* DIC_ID */
	s3c_gpio_cfgpin(GPIO_DIC_ID, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_DIC_ID, S3C_GPIO_PULL_NONE);
	/* gpio_set_value(GPIO_DIC_ID, 0); */
}
EXPORT_SYMBOL(lcd_cfg_gpio_late_resume);

static int tl2796_reset_lcd(struct platform_device *pdev)
{
	int err;

	err = gpio_request(S5PV210_MP05(5), "MLCD_RST");
	if (err) {
		printk(KERN_ERR "failed to request MP0(5) for "
				"lcd reset control\n");
		return err;
	}

	gpio_direction_output(S5PV210_MP05(5), 1);
	msleep(10);

	gpio_set_value(S5PV210_MP05(5), 0);
	msleep(10);

	gpio_set_value(S5PV210_MP05(5), 1);
	msleep(10);

	gpio_free(S5PV210_MP05(5));

	return 0;
}

static int tl2796_backlight_on(struct platform_device *pdev)
{
	return 0;
}

static struct s3c_platform_fb tl2796_data __initdata = {
	.hw_ver		= 0x62,
	.clk_name	= "sclk_fimd",
	.nr_wins	= 5,
	.default_win	= CONFIG_FB_S3C_DEFAULT_WINDOW,
	.swap		= FB_SWAP_HWORD | FB_SWAP_WORD,

	.lcd = &s6e63m0,
	.cfg_gpio	= tl2796_cfg_gpio,
	.backlight_on	= tl2796_backlight_on,
	.reset_lcd	= tl2796_reset_lcd,
};

#define LCD_BUS_NUM     3
#define DISPLAY_CS      S5PV210_MP01(1)
#define SUB_DISPLAY_CS  S5PV210_MP01(2)
#define DISPLAY_CLK     S5PV210_MP04(1)
#define DISPLAY_SI      S5PV210_MP04(3)

static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias	= "tl2796",
		.platform_data	= &herring_panel_data,
		.max_speed_hz	= 1200000,
		.bus_num	= LCD_BUS_NUM,
		.chip_select	= 0,
		.mode		= SPI_MODE_3,
		.controller_data = (void *)DISPLAY_CS,
	},
};

static struct spi_gpio_platform_data tl2796_spi_gpio_data = {
	.sck	= DISPLAY_CLK,
	.mosi	= DISPLAY_SI,
	.miso	= -1,
	.num_chipselect = 2,
};

static struct platform_device s3c_device_spi_gpio = {
	.name	= "spi_gpio",
	.id	= LCD_BUS_NUM,
	.dev	= {
		.parent		= &s3c_device_fb.dev,
		.platform_data	= &tl2796_spi_gpio_data,
	},
};

#ifdef CONFIG_30PIN_CONN
struct platform_device sec_device_connector = {
		.name	= "acc_con",
		.id 	= -1,
};
#endif

static  struct  i2c_gpio_platform_data  i2c4_platdata = {
	.sda_pin		= GPIO_AP_SDA_18V,
	.scl_pin		= GPIO_AP_SCL_18V,
	.udelay			= 2,    /* 250KHz */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c4 = {
	.name			= "i2c-gpio",
	.id			= 4,
	.dev.platform_data	= &i2c4_platdata,
};

static  struct  i2c_gpio_platform_data  i2c5_platdata = {
	.sda_pin		= GPIO_AP_SDA_28V,
	.scl_pin		= GPIO_AP_SCL_28V,
	.udelay			= 2,    /* 250KHz */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c5 = {
	.name			= "i2c-gpio",
	.id			= 5,
	.dev.platform_data	= &i2c5_platdata,
};

static struct i2c_gpio_platform_data i2c6_platdata = {
	.sda_pin                = GPIO_AP_PMIC_SDA,
	.scl_pin                = GPIO_AP_PMIC_SCL,
	.udelay                 = 2,    /* 250KHz */
	.sda_is_open_drain      = 0,
	.scl_is_open_drain      = 0,
	.scl_is_output_only     = 0,
};

static struct platform_device s3c_device_i2c6 = {
	.name			= "i2c-gpio",
	.id			= 6,
	.dev.platform_data      = &i2c6_platdata,
};

static  struct  i2c_gpio_platform_data  i2c7_platdata = {
	.sda_pin                = GPIO_USB_SW_SDA,
	.scl_pin                = GPIO_USB_SW_SCL,
	.udelay                 = 2,    /* 250KHz */
	.sda_is_open_drain      = 0,
	.scl_is_open_drain      = 0,
	.scl_is_output_only     = 0,
};

static struct platform_device s3c_device_i2c7 = {
	.name			= "i2c-gpio",
	.id			= 7,
	.dev.platform_data      = &i2c7_platdata,
};

static  struct  i2c_gpio_platform_data  i2c8_platdata = {
	.sda_pin                = GYRO_SDA_28V,
	.scl_pin                = GYRO_SCL_28V,
	.udelay                 = 2,    /* 250KHz */
	.sda_is_open_drain      = 0,
	.scl_is_open_drain      = 0,
	.scl_is_output_only     = 0,
};

static struct platform_device s3c_device_i2c8 = {
	.name			= "i2c-gpio",
	.id			= 8,
	.dev.platform_data      = &i2c8_platdata,
};


static  struct  i2c_gpio_platform_data  i2c9_platdata = {
	.sda_pin                = GPIO_FUEL_AP_SDA,
	.scl_pin                = GPIO_FUEL_AP_SCL,
	.udelay                 = 2,    /* 250KHz */
	.sda_is_open_drain      = 0,
	.scl_is_open_drain      = 0,
	.scl_is_output_only     = 0,
};

static struct platform_device s3c_device_i2c9 = {
	.name			= "i2c-gpio",
	.id			= 9,
	.dev.platform_data	= &i2c9_platdata,
};

static  struct  i2c_gpio_platform_data  i2c10_platdata = {
	.sda_pin                = GPIO_AP_SDA_2_8V,
	.scl_pin                = GPIO_AP_SCL_2_8V,
	.udelay                 = 2,    /* 250KHz */
	.sda_is_open_drain      = 0,
	.scl_is_open_drain      = 0,
	.scl_is_output_only     = 0,
};

static struct platform_device s3c_device_i2c10 = {
	.name			= "i2c-gpio",
	.id			= 10,
	.dev.platform_data	= &i2c10_platdata,
};

static	struct	i2c_gpio_platform_data	i2c11_platdata = {
	.sda_pin		= GPIO_CHARGER_SDA_2_8V,
	.scl_pin		= GPIO_CHARGER_SCL_2_8V,
	.udelay			= 2,	/* 250KHz */		
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c11 = {
	.name				= "i2c-gpio",
	.id					= 11,
	.dev.platform_data	= &i2c11_platdata,
};
	

#if 0
static  struct  i2c_gpio_platform_data  i2c11_platdata = {
	.sda_pin                = GPIO_ALS_SDA_28V,
	.scl_pin                = GPIO_ALS_SCL_28V,
	.udelay                 = 2,    /* 250KHz */
	.sda_is_open_drain      = 0,
	.scl_is_open_drain      = 0,
	.scl_is_output_only     = 0,
};

static struct platform_device s3c_device_i2c11 = {
	.name			= "i2c-gpio",
	.id			= 11,
	.dev.platform_data	= &i2c11_platdata,
};
#endif

#if 0
static  struct  i2c_gpio_platform_data  i2c12_platdata = {
	.sda_pin                = GPIO_AP_SDA_2_8V,
	.scl_pin                = GPIO_AP_SCL_2_8V,
	.udelay                 = 0,    /* 250KHz */
	.sda_is_open_drain      = 0,
	.scl_is_open_drain      = 0,
	.scl_is_output_only     = 0,
};

static struct platform_device s3c_device_i2c12 = {
	.name			= "i2c-gpio",
	.id			= 12,
	.dev.platform_data	= &i2c12_platdata,
};
#endif

#if defined(CONFIG_FB_S3C_CMC623)
static struct platform_device sec_device_tune_cmc623 = { // P1_LSJ : DE06
		.name			= "sec_tune_cmc623",
		.id 			= -1,
};
#endif

static  struct  i2c_gpio_platform_data  i2c13_platdata = {
	.sda_pin                = GPIO_CMC_SDA_18V,
	.scl_pin                = GPIO_CMC_SCL_18V,
	.udelay                 = 1,    /* 500KHz */
	.sda_is_open_drain      = 0,
	.scl_is_open_drain      = 0,
	.scl_is_output_only     = 0,
};

static struct platform_device s3c_device_i2c13 = {
	.name			= "i2c-gpio",
	.id			= 13,
	.dev.platform_data	= &i2c13_platdata,
};

#if defined(CONFIG_PN544)
static struct i2c_gpio_platform_data i2c14_platdata = {
	.sda_pin		= NFC_SDA_18V,
	.scl_pin		= NFC_SCL_18V,
	.udelay			= 2,
	.sda_is_open_drain      = 0,
	.scl_is_open_drain      = 0,
	.scl_is_output_only     = 0,
};

static struct platform_device s3c_device_i2c14 = {
	.name			= "i2c-gpio",
	.id			= 14,
	.dev.platform_data	= &i2c14_platdata,
};
#endif

#if defined(CONFIG_MACH_P1_LTN) && defined(CONFIG_VIDEO_NM6XX)
static	struct	i2c_gpio_platform_data	i2c15_platdata = {
	.sda_pin		= GPIO_ISDBT_SDA,
	.scl_pin		= GPIO_ISDBT_SCL,
	.udelay			= 2/*5*/, /* 250KHz */ 
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c15 = {
	.name				= "i2c-gpio",
	.id					= 15,
	.dev.platform_data	= &i2c15_platdata,
};
#endif

#if defined(CONFIG_KEYBOARD_GPIO)
static struct gpio_keys_button button_data[] = {
    { KEY_POWER, S5PV210_GPH2(6), 1, "Power", EV_KEY, 1, 5},
    { KEY_VOLUMEUP, S5PV210_GPH3(0), 1, "Volume Up", EV_KEY, 1, 5},
    { KEY_VOLUMEDOWN, S5PV210_GPH3(1), 1, "Volume Down", EV_KEY, 1, 5},
};

static struct gpio_keys_platform_data gpio_keys_data = {
    .buttons 	= button_data,
    .nbuttons	= ARRAY_SIZE(button_data),
    .rep		= 0,
};

static struct platform_device gpio_keys_device = {
    .name           = "gpio-keys",
    .id             = -1,
    .dev            = {
        .platform_data  = &gpio_keys_data,
    },
};
#else   //CONFIG_KEYBOARD_GPIO
#if defined(CONFIG_INPUT_GPIO)
static struct gpio_event_direct_entry p1_keypad_key_map[] = {
	{
		.gpio	= S5PV210_GPH2(6),
		.code	= KEY_POWER,
	},
	{
		.gpio	= S5PV210_GPH3(1),
		.code	= KEY_VOLUMEDOWN,
	},
	{
		.gpio	= S5PV210_GPH3(0),
		.code	= KEY_VOLUMEUP,
	}
};

static struct gpio_event_input_info p1_keypad_key_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.type = EV_KEY,
	.keymap = p1_keypad_key_map,
	.keymap_size = ARRAY_SIZE(p1_keypad_key_map)
};

static struct gpio_event_info *p1_input_info[] = {
	&p1_keypad_key_info.info,
};


static struct gpio_event_platform_data p1_input_data = {
	.names = {
		"p1-keypad",
		NULL,
	},
	.info = p1_input_info,
	.info_count = ARRAY_SIZE(p1_input_info),
};

static struct platform_device p1_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &p1_input_data,
	},
};
#endif  //CONFIG_INPUT_GPIO
#endif

#ifdef CONFIG_S5P_ADC
static struct s3c_adc_mach_info s3c_adc_platform __initdata = {
	/* s5pc110 support 12-bit resolution */
	.delay  = 10000,
	.presc  = 65,
	.resolution = 12,
};
#endif

#ifdef CONFIG_VIDEO_ISX005
static DEFINE_MUTEX(isx005_lock);
static struct regulator *cam_io_regulator;
static struct regulator *cam_a_regulator;
static struct regulator *cam_3m_regulator;
static struct regulator *cam_af_regulator;

static bool isx005_powered_on=0;
static int isx005_regulator_init(void)
{
	if (IS_ERR_OR_NULL(cam_io_regulator)) {
		cam_io_regulator = regulator_get(NULL, "cam_io");	//LDO11
		if (IS_ERR_OR_NULL(cam_io_regulator)) {
			pr_err("failed to get cam_io regulator");
			return -EINVAL;
		}
	}
	if (IS_ERR_OR_NULL(cam_a_regulator)) {
		cam_a_regulator = regulator_get(NULL, "cam_analog");	//LDO13
		if (IS_ERR_OR_NULL(cam_a_regulator)) {
			pr_err("failed to get cam_a regulator");
			return -EINVAL;
		}
	}
	if (IS_ERR_OR_NULL(cam_3m_regulator)) {
		cam_3m_regulator = regulator_get(NULL, "cam_3m");	//LDO14
		if (IS_ERR_OR_NULL(cam_3m_regulator)) {
			pr_err("failed to get cam_3m regulator");
			return -EINVAL;
		}
	}
	if (IS_ERR_OR_NULL(cam_af_regulator)) {
		cam_af_regulator = regulator_get(NULL, "cam_af");	//LDO15
		if (IS_ERR_OR_NULL(cam_af_regulator)) {
			pr_err("failed to get cam_af regulator");
			return -EINVAL;
		}
	}

	pr_debug("cam_io_regulator = %p\n", cam_io_regulator);
	pr_debug("cam_a_regulator = %p\n", cam_a_regulator);
	pr_debug("cam_3m_regulator = %p\n", cam_3m_regulator);
	pr_debug("cam_af_regulator = %p\n", cam_af_regulator);

	return 0;
}

static void isx005_gpio_init(void)
{
	/* CAM_MEGA_nRST - GPJ1(5)*/
	if (gpio_request(GPIO_CAM_MEGA_nRST, "GPJ1") < 0){
		
		pr_err("failed gpio_request GPJ1(GPIO_CAM_MEGA_nRST) for camera control\n");
	}
	/* CAM_MEGA_EN - GPJ1(2) */
	if (gpio_request(GPIO_CAM_MEGA_EN, "GPJ1") < 0){
		
		pr_err("failed gpio_request GPJ1(GPIO_CAM_MEGA_EN) for camera control(%d)\n",__LINE__);
	}
}

static int isx005_ldo_en(bool en)
{
	int err = 0;
	int result;

	if (IS_ERR_OR_NULL(cam_io_regulator) ||		//LDO11
		IS_ERR_OR_NULL(cam_a_regulator) ||		//LDO13
		IS_ERR_OR_NULL(cam_3m_regulator) ||		//LDO14
		IS_ERR_OR_NULL(cam_af_regulator)) {		//LDO15
		pr_err("Camera regulators not initialized\n");
		return -EINVAL;
	}

	if (!en)
		goto off;

	/* Turn CAM_3M_1.2V on */
	err = regulator_enable(cam_3m_regulator);
	if (err) {
		pr_err("Failed to enable regulator cam_3m_regulator\n");
		goto off;
	}
	udelay(50);

	/* Turn CAM_IO_2.8V on */
	err = regulator_enable(cam_io_regulator);
	if (err) {
		pr_err("Failed to enable regulator cam_io_regulator\n");
		goto off;
	}
	udelay(50);
	
	/* Turn CAM_A_2.8V on */
	err = regulator_enable(cam_a_regulator);
	if (err) {
		pr_err("Failed to enable regulator cam_a_regulator\n");
		goto off;
	}
	udelay(50);
	
	/* Turn CAM_AF_3.0V on */
	err = regulator_enable(cam_af_regulator);
	if (err) {
		pr_err("Failed to enable regulator cam_af_regulator\n");
		goto off;
	}
	
	return 0;

off:
	result = err;

	err = regulator_disable(cam_a_regulator);
	if (err) {
		pr_err("Failed to disable regulator cam_a_regulator\n");
		result = err;
	}
	
	err = regulator_disable(cam_af_regulator);
	if (err) {
		pr_err("Failed to disable regulator cam_af_regulator\n");
		result = err;
	}
	
	err = regulator_disable(cam_io_regulator);
	if (err) {
		pr_err("Failed to disable regulator cam_io_regulator\n");
		result = err;
	}
	udelay(50);
	err = regulator_disable(cam_3m_regulator);
	if (err) {
		pr_err("Failed to disable regulator cam_3m_regulator\n");
		result = err;
	}

	return result;
}

static int isx005_power_on(void)
{
	/* LDO on */
	int err;

	s5pv210_lock_dvfs_high_level(DVFS_LOCK_TOKEN_2, L6);
	
	/* can't do this earlier because regulators aren't available in
	 * early boot
	 */
	if (isx005_regulator_init()) {
		pr_err("Failed to initialize camera regulators\n");
		return -EINVAL;
	}

	/* CAM_MEGA_nRST - GPJ1(5)*/
	if (gpio_request(GPIO_CAM_MEGA_nRST, "GPJ1") < 0){
		
		pr_err("failed gpio_request GPJ1(GPIO_CAM_MEGA_nRST) for camera control\n");
	}

	err = isx005_ldo_en(true);
	if (err)
		return err;
	msleep(2);

	/* MCLK on - default is input, to save power when camera not on */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(GPIO_CAM_MCLK_AF));
	msleep(2);

	gpio_direction_output(GPIO_CAM_MEGA_nRST, 0);
	msleep(1);
	gpio_direction_output(GPIO_CAM_MEGA_nRST, 1);
	msleep(1);
	/* CAM_MEGA_nRST - GPJ1(5) LOW */
	gpio_set_value(GPIO_CAM_MEGA_nRST, 1);
	mdelay(1);

	gpio_free(GPIO_CAM_MEGA_nRST);
	return 0;
}

static int isx005_power_off(void)
{
	int err = 0;

	s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_2);
	
	isx005_gpio_init();

	/* CAM_MEGA_EN - GPJ1(2) LOW */
	gpio_direction_output(GPIO_CAM_MEGA_EN, 1);
	gpio_set_value(GPIO_CAM_MEGA_EN, 0);
	mdelay(3);


	/* CAM_MEGA_nRST - GPJ1(5) LOW */
	gpio_direction_output(GPIO_CAM_MEGA_nRST, 1);
	gpio_set_value(GPIO_CAM_MEGA_nRST, 0);
	mdelay(1);

	/*  Mclk disable - set to input function to save power */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, 0);
	mdelay(1);

	err = isx005_ldo_en(false);
	if (err)
		return err;
	mdelay(1);

	gpio_free(GPIO_CAM_MEGA_nRST);
	gpio_free(GPIO_CAM_MEGA_EN);
	msleep(10);

	return 0;
}

static int isx005_power_en(int onoff)
{
	int err = 0;
	mutex_lock(&isx005_lock);
	/* we can be asked to turn off even if we never were turned
	 * on if something odd happens and we are closed
	 * by camera framework before we even completely opened.
	 */
	if (onoff != isx005_powered_on) {
		if (onoff)
			err = isx005_power_on();
		else{
			err = isx005_power_off();
			s3c_i2c0_force_stop();
		}
		if (!err)
			isx005_powered_on = onoff;
	}
	mutex_unlock(&isx005_lock);

	return err;
}

int isx005_power_reset(void)
{
	isx005_power_en(0);
	isx005_power_en(1);

	return 0;
}

int isx005_cam_stdby(bool en)
{
	/* CAM_MEGA_EN - GPJ1(2) */
	if (gpio_request(GPIO_CAM_MEGA_EN, "GPJ1") < 0){
		pr_err("failed gpio_request GPJ1(GPIO_CAM_MEGA_EN) for camera control(%d)\n",__LINE__);
	}
	mdelay(1);

	gpio_direction_output(GPIO_CAM_MEGA_EN, 0);
	msleep(1);
	gpio_direction_output(GPIO_CAM_MEGA_EN, 1);
	msleep(1);

	if(en)
	{
		gpio_set_value(GPIO_CAM_MEGA_EN, 1);
	} 
	else 
	{
		gpio_set_value(GPIO_CAM_MEGA_EN, 0);
	}
	msleep(1);

	gpio_free(GPIO_CAM_MEGA_EN);

	return 0;
}

static struct isx005_platform_data isx005_plat = {
	.default_width = 800,
	.default_height = 600,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 0,
};

static struct i2c_board_info  isx005_i2c_info = {
	I2C_BOARD_INFO("ISX005", 0x1A ),
	.platform_data = &isx005_plat,
};

static struct s3c_platform_camera isx005 = {
	.id = CAMERA_PAR_A,
	.type = CAM_TYPE_ITU,
	.fmt = ITU_601_YCBCR422_8BIT,
	.order422 = CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum = 0,
	.info = &isx005_i2c_info,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.srclk_name = "xusbxti",
	.clk_name = "sclk_cam",
	.clk_rate = 24000000,
	.line_length = 1536,
	.width = 800,
	.height = 600,
	.window = {
		.left 	= 0,
		.top = 0,
		.width = 800,
		.height = 600,
	},

	/* Polarity */
	.inv_pclk = 0,
	.inv_vsync = 1,
	.inv_href = 0,
	.inv_hsync = 0,
	.initialized = 0,
	.cam_power = isx005_power_en,
};
#endif /* CONFIG_VIDEO_ISX005 */

#ifdef CONFIG_VIDEO_NM6XX 

static int nm6xx_power_en(int onoff)
{
	printk("==============  NM6XX Tuner Sensor ============== \n");

	return 0;
}

static struct nm6xx_platform_data nm6xx_plat = {
	.default_width = 320,
	.default_height = 240,
	.pixelformat = V4L2_PIX_FMT_YUYV,
	.freq = 13500000,
	.is_mipi = 0,
};

static struct i2c_board_info  nm6xx_i2c_info = {
//	I2C_BOARD_INFO("ISX005", 0x78 >> 1),
// I2C_BOARD_INFO("ISX005", 0x1A >> 1),
  I2C_BOARD_INFO("NM6XX", 0x1A ),
	.platform_data = &nm6xx_plat,
};

static struct s3c_platform_camera nm6xx = {
	.id = CAMERA_PAR_A,
	.type = CAM_TYPE_ITU,
	.fmt = ITU_601_YCBCR422_8BIT,//ITU_656_YCBCR422_8BIT,
	.order422 = CAM_ORDER422_8BIT_YCBYCR,//CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum = 0,
	.info = &nm6xx_i2c_info,
	.pixelformat = V4L2_PIX_FMT_YUYV,
	.srclk_name = "xusbxti",
	.clk_name = "sclk_cam",
	.clk_rate = 6750000,
	.line_length = 320,
	.width = 320,
	.height = 240,
	.window = {
		.left 	= 0,
		.top = 0,
		.width = 320,
		.height = 240,
	},

	/* Polarity */
	.inv_pclk = 0,
	.inv_vsync = 1,
	.inv_href = 0,
	.inv_hsync = 0,
	.initialized = 0,
	.cam_power = nm6xx_power_en,
};

static struct s3c_platform_camera dummy = {
	.id = CAMERA_PAR_A,
	.type = CAM_TYPE_ITU,
	.fmt = ITU_601_YCBCR422_8BIT,//ITU_656_YCBCR422_8BIT,
	.order422 = CAM_ORDER422_8BIT_YCBYCR,//CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum = 0,
	.info = NULL,
	.pixelformat = V4L2_PIX_FMT_YUYV,
	.srclk_name = "xusbxti",
	.clk_name = "sclk_cam",
	.clk_rate = 6750000,
	.line_length = 320,
	.width = 320,
	.height = 240,
	.window = {
		.left 	= 0,
		.top = 0,
		.width = 320,
		.height = 240,
	},

	/* Polarity */
	.inv_pclk = 0,
	.inv_vsync = 1,
	.inv_href = 0,
	.inv_hsync = 0,
	.initialized = 0,
	.cam_power = NULL,
};
#endif

#ifdef CONFIG_VIDEO_S5K6AAFX
/* External camera module setting */
static DEFINE_MUTEX(s5k6aafx_lock);
static struct regulator *cam_io_regulator;
static struct regulator *cam_a_regulator;
static struct regulator *cam_cif_regulator;
static bool s5k6aafx_powered_on=0;
static int s5k6aafx_regulator_init(void)
{
	if (IS_ERR_OR_NULL(cam_io_regulator)) {
		cam_io_regulator = regulator_get(NULL, "cam_io");	//LDO11
		if (IS_ERR_OR_NULL(cam_io_regulator)) {
			pr_err("failed to get cam_io regulator");
			return -EINVAL;
		}
	}
	if (IS_ERR_OR_NULL(cam_cif_regulator)) {
		cam_cif_regulator = regulator_get(NULL, "cam_cif");	//LDO12
		if (IS_ERR_OR_NULL(cam_cif_regulator)) {
			pr_err("failed to get cam_cif regulator");
			return -EINVAL;
		}
	}
	if (IS_ERR_OR_NULL(cam_a_regulator)) {
		cam_a_regulator = regulator_get(NULL, "cam_analog");	//LDO13
		if (IS_ERR_OR_NULL(cam_a_regulator)) {
			pr_err("failed to get cam_a regulator");
			return -EINVAL;
		}
	}

	pr_debug("cam_io_regulator = %p\n", cam_io_regulator);
	pr_debug("cam_cif_regulator = %p\n", cam_cif_regulator);
	pr_debug("cam_a_regulator = %p\n", cam_a_regulator);

	return 0;
}

static void s5k6aafx_gpio_init(void)
{
	int err;

	/* CAM_VGA_nSTBY - GPB(0) */
	err = gpio_request(GPIO_CAM_VGA_nSTBY, "GPB0");
	if (err) {
		pr_err("Failed to request GPB0(GPIO_CAM_VGA_nSTBY) for camera control\n");
	}
	/* CAM_VGA_nRST - GPB(2) */
	err = gpio_request(GPIO_CAM_VGA_nRST, "GPB2");
	if (err) {
		pr_err("Failed to request GPB2(GPIO_CAM_VGA_nRST) for camera control\n");
	}
}

static int s5k6aafx_ldo_en(bool en)
{
	int err = 0;
	int result;

	if (IS_ERR_OR_NULL(cam_io_regulator) ||		//LDO11
		IS_ERR_OR_NULL(cam_cif_regulator) ||	//LDO12
		IS_ERR_OR_NULL(cam_a_regulator)){	//LDO13
		pr_err("Camera regulators not initialized\n");
		return -EINVAL;
	}

	if (!en)
		goto off;
	
	/* Turn CAM_A_2.8V on */
	err = regulator_enable(cam_io_regulator);
	if (err) {
		pr_err("Failed to enable regulator cam_io_regulator\n");
		goto off;
	}
	udelay(50);
	
	/* Turn CAM_CIF_1.8V on */
	err = regulator_enable(cam_cif_regulator);
	if (err) {
		pr_err("Failed to enable regulator cam_cif_regulator\n");
		goto off;
	}
	udelay(50);
	
	/* Turn CAM_A_2.8V on */
	err = regulator_enable(cam_a_regulator);
	if (err) {
		pr_err("Failed to enable regulator cam_a_regulator\n");
		goto off;
	}
	udelay(50);
	
	return 0;

off:
	result = err;

	err = regulator_disable(cam_io_regulator);
	if (err) {
		pr_err("Failed to disable regulator cam_io_regulator\n");
		result = err;
	}
	udelay(50);

	err = regulator_disable(cam_cif_regulator);
	if (err) {
		pr_err("Failed to disable regulator cam_cif_regulator\n");
		result = err;
	}
	udelay(50);

	err = regulator_disable(cam_a_regulator);
	if (err) {
		pr_err("Failed to disable regulator cam_a_regulator\n");
		result = err;
	}
	udelay(50);

	return result;

}


static int s5k6aafx_power_on(void)
{
	/* LDO on */
	int err = 0;

	s5pv210_lock_dvfs_high_level(DVFS_LOCK_TOKEN_2, L6);
	
	/* can't do this earlier because regulators aren't available in
	 * early boot
	 */
	if (s5k6aafx_regulator_init()) {
		pr_err("Failed to initialize camera regulators\n");
		return -EINVAL;
	}

	s5k6aafx_gpio_init();

	err = s5k6aafx_ldo_en(true);
	if (err)
		return err;

	/* CAM_VGA_nSTBY HIGH */
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 0);
	gpio_set_value(GPIO_CAM_VGA_nSTBY, 1);
	udelay(500);

	/* MCLK on - default is input, to save power when camera not on */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(GPIO_CAM_MCLK_AF));
	udelay(200);

	/* CAM_VGA_nRST HIGH */
	gpio_direction_output(GPIO_CAM_VGA_nRST, 0);
	gpio_set_value(GPIO_CAM_VGA_nRST, 1);		
	udelay(500);
	
	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);

	return 0;
}

static int s5k6aafx_power_off(void)
{
	int err = 0;

	s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_2);
	
	s5k6aafx_gpio_init();

	/* CAM_VGA_nRST LOW */
	gpio_direction_output(GPIO_CAM_VGA_nRST, 1);
	gpio_set_value(GPIO_CAM_VGA_nRST, 0);
	udelay(200);

	/*  Mclk disable - set to input function to save power */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, 0);
	udelay(50);

	/* CAM_VGA_nSTBY LOW */
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 1);
	gpio_set_value(GPIO_CAM_VGA_nSTBY, 0);
	udelay(500);

	err = s5k6aafx_ldo_en(false);
	if (err)
		return err;

	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);	

	return 0;
}

static int s5k6aafx_power_en(int onoff)
{
	int err = 0;
	mutex_lock(&s5k6aafx_lock);
	/* we can be asked to turn off even if we never were turned
	 * on if something odd happens and we are closed
	 * by camera framework before we even completely opened.
	 */
	if (onoff != s5k6aafx_powered_on) {
		if (onoff)
			err = s5k6aafx_power_on();
		else{
			err = s5k6aafx_power_off();
			s3c_i2c0_force_stop();
		}
		if (!err)
			s5k6aafx_powered_on = onoff;
	}
	mutex_unlock(&s5k6aafx_lock);
	return err;
}

int s5k6aafx_power_reset(void)
{
	s5k6aafx_power_en(0);
	s5k6aafx_power_en(1);

	return 0;
}

static struct s5k6aafx_platform_data s5k6aafx_plat = {
	.default_width = 800,
	.default_height = 600,
	.pixelformat = V4L2_PIX_FMT_UYVY,
};

static struct i2c_board_info s5k6aafx_i2c_info = {
	I2C_BOARD_INFO("S5K6AAFX", 0x78 >> 1),
	.platform_data = &s5k6aafx_plat,
};

static struct s3c_platform_camera s5k6aafx = {
	.id		= CAMERA_PAR_A,
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.i2c_busnum	= 0,
	.info		= &s5k6aafx_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti",
	.clk_name	= "sclk_cam",
	.clk_rate	= 24000000,
	.line_length	= 800,
	.width		= 800,
	.height		= 600,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 800,
		.height	= 600,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync 	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,

	.initialized 	= 0,
	.cam_power	= s5k6aafx_power_en,
};
#endif	//CONFIG_VIDEO_S5K6AAFX

#ifdef CONFIG_VIDEO_S5K5CCGX
static DEFINE_MUTEX(s5k5ccgx_lock);
/*static struct regulator *s5k5ccgx_cam_io_regulator;
static struct regulator *s5k5ccgx_cam_a_regulator;
static struct regulator *s5k5ccgx_cam_3m_regulator;
static struct regulator *s5k5ccgx_cam_af_regulator;*/

static bool s5k5ccgx_powered_on=0;
static int s5k5ccgx_regulator_init(void)
{
	if (IS_ERR_OR_NULL(cam_io_regulator)) {
		cam_io_regulator = regulator_get(NULL, "cam_io");	//LDO11
		if (IS_ERR_OR_NULL(cam_io_regulator)) {
			pr_err("failed to get cam_io regulator");
			return -EINVAL;
		}
	}
	if (IS_ERR_OR_NULL(cam_a_regulator)) {
		cam_a_regulator = regulator_get(NULL, "cam_analog");	//LDO13
		if (IS_ERR_OR_NULL(cam_a_regulator)) {
			pr_err("failed to get cam_a regulator");
			return -EINVAL;
		}
	}
	if (IS_ERR_OR_NULL(cam_3m_regulator)) {
		cam_3m_regulator = regulator_get(NULL, "cam_3m");	//LDO14
		if (IS_ERR_OR_NULL(cam_3m_regulator)) {
			pr_err("failed to get cam_3m regulator");
			return -EINVAL;
		}
	}
	if (IS_ERR_OR_NULL(cam_af_regulator)) {
		cam_af_regulator = regulator_get(NULL, "cam_af");	//LDO15
		if (IS_ERR_OR_NULL(cam_af_regulator)) {
			pr_err("failed to get cam_af regulator");
			return -EINVAL;
		}
	}
	if (IS_ERR_OR_NULL(cam_cif_regulator)) {
		cam_cif_regulator = regulator_get(NULL, "cam_cif");	//LDO12
		if (IS_ERR_OR_NULL(cam_cif_regulator)) {
			pr_err("failed to get cam_cif regulator");
			return -EINVAL;
		}
	}

	pr_debug("cam_io_regulator = %p\n", cam_io_regulator);
	pr_debug("cam_a_regulator = %p\n", cam_a_regulator);
	pr_debug("cam_3m_regulator = %p\n", cam_3m_regulator);
	pr_debug("cam_af_regulator = %p\n", cam_af_regulator);
	pr_debug("cam_cif_regulator = %p\n", cam_cif_regulator);
	return 0;
}

static void s5k5ccgx_gpio_init(void)
{
	// CAM_MEGA_nRST - GPJ1(5)
	if (gpio_request(GPIO_CAM_MEGA_nRST, "GPJ1") < 0){
		pr_err("failed gpio_request GPJ1(GPIO_CAM_MEGA_nRST) for camera control\n");
	}
	// CAM_MEGA_EN - GPJ1(2)
	if (gpio_request(GPIO_CAM_MEGA_EN, "GPJ1") < 0){
		pr_err("failed gpio_request GPJ1(GPIO_CAM_MEGA_EN) for camera control(%d)\n",__LINE__);
	}
#ifdef SOC_DUALCAM_POWERCTRL
	// CAM_VGA_nSTBY - GPB(0)/
	if(gpio_request(GPIO_CAM_VGA_nSTBY, "GPB0") < 0){
		pr_err("Failed to request GPB0(GPIO_CAM_VGA_nSTBY) for camera control\n");
	}
	// CAM_VGA_nRST - GPB(2)
	if(gpio_request(GPIO_CAM_VGA_nRST, "GPB2") < 0){
		pr_err("Failed to request GPB2(GPIO_CAM_VGA_nRST) for camera control\n");
	}
#endif
}

static int s5k5ccgx_ldo_en(bool en)
{
	int err = 0;
	int result;

	if (IS_ERR_OR_NULL(cam_io_regulator) ||		//LDO11
		IS_ERR_OR_NULL(cam_cif_regulator) ||	//LDO12
		IS_ERR_OR_NULL(cam_a_regulator) ||	//LDO13
		IS_ERR_OR_NULL(cam_3m_regulator) ||	//LDO14
		IS_ERR_OR_NULL(cam_af_regulator)) {	//LDO15
		pr_err("Camera regulators not initialized\n");
		return -EINVAL;
	}

	if (!en)
		goto off;

	/* Turn CAM_A_2.8V on */
	err = regulator_enable(cam_a_regulator);
	if (err) {
		pr_err("Failed to enable regulator s5k5ccgx_cam_a_regulator\n");
		goto off;
	}

#ifdef SOC_DUALCAM_POWERCTRL
	/* Turn CAM_CIF_1.8V on */
	err = regulator_enable(cam_cif_regulator);
	if (err) {
		pr_err("Failed to enable regulator cam_cif_regulator\n");
		goto off;
	}
	udelay(50);
#endif

	/* Turn CAM_3M_1.2V on */
	err = regulator_enable(cam_3m_regulator);
	if (err) {
		pr_err("Failed to enable regulator s5k5ccgx_cam_3m_regulator\n");
		goto off;
	}

	/* Turn CAM_AF_3.0V on */
	err = regulator_enable(cam_af_regulator);
	if (err) {
		pr_err("Failed to enable regulator s5k5ccgx_cam_af_regulator\n");
		goto off;
	}
	
	/* Turn CAM_IO_2.8V on */
	err = regulator_enable(cam_io_regulator);
	if (err) {
		pr_err("Failed to enable regulator s5k5ccgx_cam_io_regulator\n");
		goto off;
	}

	return 0;

off:
	result = err;

	err = regulator_disable(cam_io_regulator);
	if (err) {
		pr_err("Failed to disable regulator s5k5ccgx_cam_io_regulator\n");
		result = err;
	}
	
	err = regulator_disable(cam_af_regulator);
	if (err) {
		pr_err("Failed to disable regulator s5k5ccgx_cam_af_regulator\n");
		result = err;
	}

	err = regulator_disable(cam_3m_regulator);
	if (err) {
		pr_err("Failed to disable regulator s5k5ccgx_cam_3m_regulator\n");
		result = err;
	}

	err = regulator_disable(cam_a_regulator);
	if (err) {
		pr_err("Failed to disable regulator s5k5ccgx_cam_a_regulator\n");
		result = err;
	}

	return result;
}

static int s5k5ccgx_power_on(void)
{
	/* LDO on */
	int err;

	s5pv210_lock_dvfs_high_level(DVFS_LOCK_TOKEN_2, L6);
	
	/* can't do this earlier because regulators aren't available in
	 * early boot
	 */
	if (s5k5ccgx_regulator_init()) {
		pr_err("Failed to initialize camera regulators\n");
		return -EINVAL;
	}

	s5k5ccgx_gpio_init();

	err = s5k5ccgx_ldo_en(true);
	if (err){
		return err;
	}

#ifdef SOC_DUALCAM_POWERCTRL
	udelay(60);
	/* CAM_VGA_nSTBY HIGH */
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 0);
	gpio_set_value(GPIO_CAM_VGA_nSTBY, 1);
	mdelay(5);	
#endif
	/* MCLK on - default is input, to save power when camera not on */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(GPIO_CAM_MCLK_AF));

#ifdef SOC_DUALCAM_POWERCTRL
	/* CAM_VGA_nRST HIGH */
	gpio_direction_output(GPIO_CAM_VGA_nRST, 0);
	gpio_set_value(GPIO_CAM_VGA_nRST, 1);		
	mdelay(7);	

	/* CAM_VGA_nSTBY HIGH */
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 1);
	gpio_set_value(GPIO_CAM_VGA_nSTBY, 0);
	udelay(20);
#endif

	/* CAM_MEGA_EN - GPJ1(2) HIGH */
	gpio_direction_output(GPIO_CAM_MEGA_EN, 0);
	gpio_set_value(GPIO_CAM_MEGA_EN, 1);
	mdelay(1);


	/* CAM_MEGA_nRST - GPJ1(5) HIGH */
	gpio_direction_output(GPIO_CAM_MEGA_nRST, 0);
	gpio_set_value(GPIO_CAM_MEGA_nRST, 1);
	msleep(10);

	gpio_free(GPIO_CAM_MEGA_EN);
	gpio_free(GPIO_CAM_MEGA_nRST);
#ifdef SOC_DUALCAM_POWERCTRL 
	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);	
#endif

	return 0;
}

static int s5k5ccgx_power_off(void)
{
	int err = 0;

	s5pv210_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_2);

	// CAM_MEGA_nRST - GPJ1(5)
	if (gpio_request(GPIO_CAM_MEGA_nRST, "GPJ1") < 0){
		pr_err("failed gpio_request(GPJ1) for camera control\n");
	}
	// CAM_MEGA_EN - GPJ1(2)
	if (gpio_request(GPIO_CAM_MEGA_EN, "GPJ1") < 0){
		pr_err("failed gpio_request(GPJ1) for camera control\n");
	}
#ifdef SOC_DUALCAM_POWERCTRL
	// CAM_VGA_nRST - GPB(2)
	if(gpio_request(GPIO_CAM_VGA_nRST, "GPB2") < 0){
		pr_err("Failed to request GPB2 for camera control\n");
	}
#endif

	/* CAM_MEGA_nRST - GPJ1(5) LOW */
	gpio_direction_output(GPIO_CAM_MEGA_nRST, 1);
	gpio_set_value(GPIO_CAM_MEGA_nRST, 0);
	mdelay(1);

	/*  Mclk disable - set to input function to save power */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, 0);

	/* CAM_MEGA_EN - GPJ1(2) LOW */
	gpio_direction_output(GPIO_CAM_MEGA_EN, 1);
	gpio_set_value(GPIO_CAM_MEGA_EN, 0);

#ifdef SOC_DUALCAM_POWERCTRL 
	gpio_direction_output(GPIO_CAM_VGA_nRST, 1);
	gpio_set_value(GPIO_CAM_VGA_nRST, 0);
#endif

	err = s5k5ccgx_ldo_en(false);
	if (err)
	{
		return err;
	}

	gpio_free(GPIO_CAM_MEGA_EN);
	gpio_free(GPIO_CAM_MEGA_nRST);
#ifdef SOC_DUALCAM_POWERCTRL 
	gpio_free(GPIO_CAM_VGA_nRST);
#endif

	return 0;
}

static int s5k5ccgx_power_en(int onoff)
{
	int err = 0;
	mutex_lock(&s5k5ccgx_lock);
	/* we can be asked to turn off even if we never were turned
	 * on if something odd happens and we are closed
	 * by camera framework before we even completely opened.
	 */
	if (onoff != s5k5ccgx_powered_on) {
		if (onoff)
			err = s5k5ccgx_power_on();
		else{
			err = s5k5ccgx_power_off();
			s3c_i2c0_force_stop();
		}
		if (!err)
			s5k5ccgx_powered_on = onoff;
	}
	mutex_unlock(&s5k5ccgx_lock);
	return err;
}

static int s5k5ccgx_reset(struct v4l2_subdev *sd)
{
	s5k5ccgx_power_en(0);
	mdelay(5);
	s5k5ccgx_power_en(1);
	mdelay(5);
	return 0;
}

int s5k5ccgx_cam_stdby(bool en)
{
	printk(KERN_ERR"<MACHINE> stdby(%d)\n", en);

	mdelay(1);

	gpio_direction_output(GPIO_CAM_MEGA_EN, 0);
	msleep(1);
	gpio_direction_output(GPIO_CAM_MEGA_EN, 1);
	msleep(1);

	if(en)
	{
		gpio_set_value(GPIO_CAM_MEGA_EN, 1);
	} 
	else 
	{
		gpio_set_value(GPIO_CAM_MEGA_EN, 0);
	}
	msleep(1);

	return 0;
}

static struct s5k5ccgx_platform_data s5k5ccgx_plat = {
	.default_width = 800,
	.default_height = 600,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 0,
};

static struct i2c_board_info  s5k5ccgx_i2c_info = {
I2C_BOARD_INFO("S5K5CCGX", 0x78>>1 ),
	.platform_data = &s5k5ccgx_plat,
};

static struct s3c_platform_camera s5k5ccgx = {
	.id = CAMERA_PAR_A,
	.type = CAM_TYPE_ITU,
	.fmt = ITU_601_YCBCR422_8BIT,
	.order422 = CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum = 0,
	.info = &s5k5ccgx_i2c_info,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.srclk_name = "xusbxti",
	.clk_name = "sclk_cam",
	.clk_rate = 24000000,
	.line_length = 1536,
	.width = 800,
	.height = 600,
	.window = {
		.left 	= 0,
		.top = 0,
		.width = 800,
		.height = 600,
	},

	/* Polarity */
	.inv_pclk = 0,
	.inv_vsync = 1,
	.inv_href = 0,
	.inv_hsync = 0,
	.initialized = 0,
	.cam_power = s5k5ccgx_power_en,
};
#endif	//CONFIG_VIDEO_S5K5CCGX

/* Interface setting */
static struct s3c_platform_fimc fimc_plat_lsi = {
	.srclk_name	= "mout_mpll",
	.clk_name	= "sclk_fimc",
	.lclk_name	= "sclk_fimc_lclk",
	.clk_rate	= 166750000,
	.default_cam	= CAMERA_PAR_A,
	.camera		= {
#ifdef CONFIG_VIDEO_ISX005
		&isx005,
#endif
#ifdef CONFIG_VIDEO_S5K6AAFX
		&s5k6aafx,
#endif
#ifdef CONFIG_VIDEO_NM6XX 
    &dummy, 
    &nm6xx,
#endif

#ifdef CONFIG_VIDEO_S5K5CCGX
		&s5k5ccgx,
#endif
	},
	.hw_ver		= 0x43,
};

#ifdef CONFIG_VIDEO_JPEG_V2
static struct s3c_platform_jpeg jpeg_plat __initdata = {
	.max_main_width		= 1280,
	.max_main_height	= 960,
	.max_thumb_width	= 400,
	.max_thumb_height	= 240,
};
#endif


static struct i2c_board_info i2c_devs4[] __initdata = {
	{
		I2C_BOARD_INFO("wm8994", (0x34>>1)),
	},
#ifdef CONFIG_MHL_SII9234
	{
		I2C_BOARD_INFO("SII9234", 0x72>>1),
	},
	{
		I2C_BOARD_INFO("SII9234A", 0x7A>>1),
	},
	{
		I2C_BOARD_INFO("SII9234B", 0x92>>1),
	},
	{
		I2C_BOARD_INFO("SII9234C", 0xC8>>1),
	},
#endif	
};

static struct platform_device bma020_accel = {
       .name  = "bma020-accelerometer",
       .id    = -1,
};

static struct l3g4200d_platform_data l3g4200d_p1p2_platform_data = {
};

static struct i2c_board_info i2c_devs5[] __initdata = {
	{
		I2C_BOARD_INFO("bma020", 0x38),
	},
	{
		I2C_BOARD_INFO("l3g4200d", 0x68),
		.platform_data = &l3g4200d_p1p2_platform_data,
		.irq = -1,
	},
};
/* I2C1 */
static struct i2c_board_info i2c_devs1[] __initdata = {
    {
        I2C_BOARD_INFO("s5p_ddc", (0x74>>1)),
    },
};

/* I2C2 */
static struct i2c_board_info i2c_devs2[] __initdata = {
	{
		I2C_BOARD_INFO("qt602240_ts", 0x4a),
	},
};


static void l3g4200d_irq_init(void)
{
	i2c_devs5[1].irq = IRQ_EINT(29);
}

static void fsa9480_usb_cb(bool attached)
{
	struct usb_gadget *gadget = platform_get_drvdata(&s3c_device_usbgadget);

	if (gadget) {
		if (attached)
			usb_gadget_vbus_connect(gadget);
		else {
			gadget->speed = USB_SPEED_HIGH;
			usb_gadget_vbus_disconnect(gadget);
		}
	}

	set_cable_status = attached ? CABLE_TYPE_USB : CABLE_TYPE_NONE;
	if (callbacks && callbacks->set_cable)
		callbacks->set_cable(callbacks, set_cable_status);

	if(!attached)	ap_vbus_disabled = 0;  // reset flag
}

static void fsa9480_charger_cb(bool attached)
{
	set_cable_status = attached ? CABLE_TYPE_AC : CABLE_TYPE_NONE;
	if (callbacks && callbacks->set_cable)
		callbacks->set_cable(callbacks, set_cable_status);

	if(!attached)	ap_vbus_disabled = 0;  // reset flag
}

static void fsa9480_jig_cb(bool attached)
{
	printk("%s : attached (%d)\n", __func__, (int)attached);
	fsa9480_jig_status = attached;
}

static struct switch_dev switch_dock = {
	.name = "dock",
};

static void fsa9480_deskdock_cb(bool attached)
{
	if (attached)
		switch_set_state(&switch_dock, 1);
	else
		switch_set_state(&switch_dock, 0);
}

static void fsa9480_cardock_cb(bool attached)
{
	if (attached)
		switch_set_state(&switch_dock, 2);
	else
		switch_set_state(&switch_dock, 0);
}

static void fsa9480_reset_cb(void)
{
	int ret;

	/* for CarDock, DeskDock */
	ret = switch_dev_register(&switch_dock);
	if (ret < 0)
		pr_err("Failed to register dock switch. %d\n", ret);
}

static void fsa9480_set_init_flag(void)
{
	fsa9480_init_flag = 1;
}

static void fsa9480_usb_switch(void)
{
	// check if sec_switch init finished.
	if(!sec_switch_inited)
		return;

	if(sec_switch_status & (int)(USB_SEL_MASK)) {
		sec_switch_set_regulator(AP_VBUS_ON);
	}
	else {
		sec_switch_set_regulator(CP_VBUS_ON);
	}
}

static struct fsa9480_platform_data fsa9480_pdata = {
	.usb_cb = fsa9480_usb_cb,
	.charger_cb = fsa9480_charger_cb,
	.jig_cb = fsa9480_jig_cb,
	.deskdock_cb = fsa9480_deskdock_cb,
	.cardock_cb = fsa9480_cardock_cb,
	.reset_cb = fsa9480_reset_cb,
	.set_init_flag = fsa9480_set_init_flag,
	.set_usb_switch = fsa9480_usb_switch,
};

static struct i2c_board_info i2c_devs7[] __initdata = {
	{
		I2C_BOARD_INFO("fsa9480", 0x4A >> 1),
		.platform_data = &fsa9480_pdata,
		.irq = IRQ_EINT(23),
	},
};

static struct charger_device smb136_chgdev = {
	.set_charging_status = sec_bat_set_charging_status,
};

static int smb136_charger_register(struct charger_device *chgdev)
{
	sec_battery_pdata.external_charger = chgdev;
	return 0;
}

static void smb136_charger_unregister(struct charger_device *chgdev)
{
	sec_battery_pdata.external_charger = NULL;
}

static struct smb136_charger_data smb136_charger = {
	.charger_dev_register = smb136_charger_register,
	.charger_dev_unregister = smb136_charger_unregister,
	.chgdev = &smb136_chgdev,
};

static struct i2c_board_info i2c_devs11[] __initdata = {
	{
		I2C_BOARD_INFO("smb136-charger",  0x9A >> 1),
		.platform_data = &smb136_charger,
		.irq = IRQ_EINT(9),
	},
};

static void __init smb136_gpio_init(void)
{
	s3c_gpio_cfgpin(GPIO_TA_nCHG, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_TA_nCHG, S3C_GPIO_PULL_NONE);
}

static struct i2c_board_info i2c_devs6[] __initdata = {
#ifdef CONFIG_REGULATOR_MAX8998
	{
		/* The address is 0xCC used since SRAD = 0 */
		I2C_BOARD_INFO("max8998", (0xCC >> 1)),
		.platform_data	= &max8998_pdata,
		.irq		= IRQ_EINT7,
	}, {
		I2C_BOARD_INFO("rtc_max8998", (0x0D >> 1)),
	},
#endif
};

#if defined(CONFIG_PN544)
static struct pn544_i2c_platform_data pn544_pdata = {
	.irq_gpio = NFC_IRQ,
	.ven_gpio = NFC_EN,
	.firm_gpio = NFC_FIRM,
};

static struct i2c_board_info i2c_devs14[] __initdata = {
	{
		I2C_BOARD_INFO("pn544", 0x2b),
		.irq = IRQ_EINT(12),
		.platform_data = &pn544_pdata,
	},
};
#endif

static int max17042_power_supply_register(struct device *parent,
	struct power_supply *psy)
{
	sec_battery_pdata.psy_fuelgauge = psy;
	return 0;
}

static void max17042_power_supply_unregister(struct power_supply *psy)
{
	sec_battery_pdata.psy_fuelgauge = NULL;
}

static void max17042_force_status_update(void)
{
	if (callbacks && callbacks->force_update)
		callbacks->force_update(callbacks);
}

static void max17042_register_callbacks(
		struct max17042_callbacks *ptr)
{
	max17042_cb = ptr;
}

static struct max17042_platform_data max17042_pdata = {
	.register_callbacks = max17042_register_callbacks,
	.power_supply_register = max17042_power_supply_register,
	.power_supply_unregister = max17042_power_supply_unregister,
	.force_update_status = max17042_force_status_update,
};

static struct i2c_board_info i2c_devs9[] __initdata = {
	{
		I2C_BOARD_INFO("max17042", (0x6D >> 1)),
		.platform_data = &max17042_pdata,
		.irq = IRQ_EINT(14),
	},
};

static void __init max17042_gpio_init(void)
{
	s3c_gpio_cfgpin(GPIO_FUEL_ARLT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_FUEL_ARLT, S3C_GPIO_PULL_NONE);
}

static void ambient_light_sensor_reset(void)
{
	int gpio;

	/* BH1721FVC */
	gpio = S5PV210_GPG2(2);		/* XMMC2CDn */
	s3c_gpio_cfgpin(gpio, S3C_GPIO_OUTPUT);
	gpio_request(gpio, "ALS_nRST");
	gpio_direction_output(gpio, 0);

	gpio_set_value(gpio, 0);
	/* More than 1us */
	udelay(2);
	gpio_set_value(gpio, 1);
}

static struct bh1721_platform_data bh1721_p1p2_platform_data = {
	.reset = ambient_light_sensor_reset,
};

static struct i2c_board_info i2c_devs10[] __initdata = {
	{
		I2C_BOARD_INFO("ak8973", 0x1c),
	},
	{
		I2C_BOARD_INFO("bh1721", 0x23),
		.platform_data	= &bh1721_p1p2_platform_data,
	},
};

static struct i2c_board_info i2c_devs13[] __initdata = {
	{
		I2C_BOARD_INFO("sec_tune_cmc623_i2c", 0x38),
	},
};

#if defined(CONFIG_VIDEO_NM6XX)
	static struct i2c_board_info i2c_devs15[] __initdata = {
		{
			I2C_BOARD_INFO("nmi625", 0x61),
		},
	};
#endif

static struct resource ram_console_resource[] = {
	{
		.flags = IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources = ARRAY_SIZE(ram_console_resource),
	.resource = ram_console_resource,
};

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data pmem_pdata = {
	.name = "pmem",
	.no_allocator = 1,
	.cached = 1,
	.start = 0,
	.size = 0,
};

static struct android_pmem_platform_data pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.no_allocator = 1,
	.cached = 1,
	.buffered = 1,
	.start = 0,
	.size = 0,
};

static struct android_pmem_platform_data pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.no_allocator = 1,
	.cached = 1,
	.buffered = 1,
	.start = 0,
	.size = 0,
};

static struct platform_device pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &pmem_pdata },
};

static struct platform_device pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &pmem_gpu1_pdata },
};

static struct platform_device pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &pmem_adsp_pdata },
};

static void __init android_pmem_set_platdata(void)
{
	pmem_pdata.start = (u32)s5p_get_media_memory_bank(S5P_MDEV_PMEM, 0);
	pmem_pdata.size = (u32)s5p_get_media_memsize_bank(S5P_MDEV_PMEM, 0);

	pmem_gpu1_pdata.start =
		(u32)s5p_get_media_memory_bank(S5P_MDEV_PMEM_GPU1, 0);
	pmem_gpu1_pdata.size =
		(u32)s5p_get_media_memsize_bank(S5P_MDEV_PMEM_GPU1, 0);

	pmem_adsp_pdata.start =
		(u32)s5p_get_media_memory_bank(S5P_MDEV_PMEM_ADSP, 0);
	pmem_adsp_pdata.size =
		(u32)s5p_get_media_memsize_bank(S5P_MDEV_PMEM_ADSP, 0);
}
#endif


static struct regulator *reg_safeout1;
static struct regulator *reg_safeout2;

int sec_switch_get_regulator(void)
{
	printk("%s\n", __func__);
		
	// get regulators.
	if (IS_ERR_OR_NULL(reg_safeout1)) {
		reg_safeout1 = regulator_get(NULL, "vbus_ap");
		if (IS_ERR_OR_NULL(reg_safeout1)) {
			   pr_err("failed to get safeout1 regulator");
			   return -1;
		}
	}

	if (IS_ERR_OR_NULL(reg_safeout2)) {
		reg_safeout2 = regulator_get(NULL, "vbus_cp");
		if (IS_ERR_OR_NULL(reg_safeout2)) {
			   pr_err("failed to get safeout2 regulator");
			   return -1;
		}
	}

//	printk("reg_safeout1 = %p\n", reg_safeout1);
//	printk("reg_safeout2 = %p\n", reg_safeout2);

	return 0;
}

void sec_switch_set_regulator(int mode)
{
	struct usb_gadget *gadget = platform_get_drvdata(&s3c_device_usbgadget);

	printk("%s (mode : %d)\n", __func__, mode);

	if (IS_ERR_OR_NULL(reg_safeout1) ||
		IS_ERR_OR_NULL(reg_safeout2)) {
		pr_err("safeout regulators not initialized yet!!\n");
		return -EINVAL;
	}

	// note : safeout1/safeout2 register setting is not matched regulator's use_count.
	//            so, set/reset use_count is needed to control safeout regulator correctly...
	if(mode == CP_VBUS_ON) {
		if(!regulator_is_enabled(reg_safeout2)) {
			regulator_set_use_count(reg_safeout2, 0);
			regulator_enable(reg_safeout2);
		}
		
		if(regulator_is_enabled(reg_safeout1)) {
			regulator_set_use_count(reg_safeout1, 1);
			regulator_disable(reg_safeout1);
		}
	}
	else if(mode == AP_VBUS_ON) {
		/* if(!regulator_is_enabled(reg_safeout1)) */ {
			regulator_set_use_count(reg_safeout1, 0);
			regulator_enable(reg_safeout1);
		}
		
		if(regulator_is_enabled(reg_safeout2)) {
			regulator_set_use_count(reg_safeout2, 1);
			regulator_disable(reg_safeout2);
		}
	}
	else {  // AP_VBUS_OFF
		printk("%s : AP VBUS OFF\n", __func__);

		gadget->speed = USB_SPEED_UNKNOWN;
		usb_gadget_vbus_disconnect(gadget);
#if 0 // regulator should be enabled for tethering
		mdelay(10);
		if(regulator_is_enabled(reg_safeout1)) {
			regulator_set_use_count(reg_safeout1, 1);
			regulator_disable(reg_safeout1);
		}
		mdelay(10);
		usb_gadget_vbus_connect(gadget);
#endif
		ap_vbus_disabled = 1;  // set flag
	}
}

int sec_switch_get_cable_status(void)
{
	return (ap_vbus_disabled ? CABLE_TYPE_NONE : set_cable_status);
}

int sec_switch_get_phy_init_status(void)
{
	return fsa9480_init_flag;
}

void sec_switch_set_switch_status(int val)
{
	printk("%s (switch_status : %d)\n", __func__, val);
	if(!sec_switch_inited)
		sec_switch_inited = 1;

	sec_switch_status = val;
}

static struct sec_switch_platform_data sec_switch_pdata = {
	.get_regulator = sec_switch_get_regulator,
	.set_regulator = sec_switch_set_regulator,
	.get_cable_status = sec_switch_get_cable_status,
	.get_phy_init_status = sec_switch_get_phy_init_status,
	.set_switch_status = sec_switch_set_switch_status,
};

struct platform_device sec_device_switch = {
	.name	= "sec_switch",
	.id	= 1,
	.dev	= {
		.platform_data	= &sec_switch_pdata,
	}
};

static struct platform_device sec_device_rfkill = {
	.name	= "bt_rfkill",
	.id	= -1,
};

static struct platform_device sec_device_btsleep = {
	.name	= "bt_sleep",
	.id	= -1,
};

#ifdef CONFIG_SAMSUNG_JACK
static struct sec_jack_zone sec_jack_zones[] = {
	{
		/* adc == 0, unstable zone, default to 3pole if it stays
		 * in this range for a half second (20ms delays, 25 samples)
		 */
		.adc_high = 0,
		.delay_ms = 20,
		.check_count = 25,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 0 < adc <= 1000, unstable zone, default to 3pole if it stays
		 * in this range for a second (10ms delays, 100 samples)
		 */
		.adc_high = 1000,
		.delay_ms = 10,
		.check_count = 100,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 1000 < adc <= 2000, unstable zone, default to 4pole if it
		 * stays in this range for a second (10ms delays, 100 samples)
		 */
		.adc_high = 2000,
		.delay_ms = 10,
		.check_count = 100,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* 2000 < adc <= 3700, 4 pole zone, default to 4pole if it
		 * stays in this range for 200ms (20ms delays, 10 samples)
		 */
		.adc_high = 3700,
		.delay_ms = 20,
		.check_count = 10,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* adc > 3700, unstable zone, default to 3pole if it stays
		 * in this range for a second (10ms delays, 100 samples)
		 */
		.adc_high = 0x7fffffff,
		.delay_ms = 10,
		.check_count = 100,
		.jack_type = SEC_HEADSET_3POLE,
	},
};

static int sec_jack_get_adc_value(void)
{
	return s3c_adc_get_adc_data(3);
}

static void sec_jack_set_micbias_state(bool on)
{
	gpio_set_value(GPIO_EAR_MICBIAS_EN, on);
}

struct sec_jack_platform_data sec_jack_pdata = {
	.set_micbias_state = sec_jack_set_micbias_state,
	.get_adc_value = sec_jack_get_adc_value,
	.zones = sec_jack_zones,
	.num_zones = ARRAY_SIZE(sec_jack_zones),
	.det_gpio = GPIO_DET_35,
	.send_end_gpio = GPIO_EAR_SEND_END,
};

static struct platform_device sec_device_jack = {
	.name			= "sec_jack",
	.id			= 1, /* will be used also for gpio_event id */
	.dev.platform_data	= &sec_jack_pdata,
};
#endif

#ifdef CONFIG_SEC_HEADSET
static struct sec_jack_port sec_jack_port_info[] = {
		{
		{ // HEADSET detect info
			.eint		=IRQ_EINT8, 
			.gpio		= GPIO_DET_35,	 
			.gpio_af	= GPIO_DET_35_AF , 
			.low_active 	= 1
		},
		{ // SEND/END info
			.eint		= IRQ_EINT12,
			.gpio		= GPIO_EAR_SEND_END, 
			.gpio_af	= GPIO_EAR_SEND_END_AF, 
			.low_active = 1
		}			
		}
};

static struct sec_jack_platform_data sec_jack_pdata = {
		.port			= sec_jack_port_info,
		.nheadsets		= ARRAY_SIZE(sec_jack_port_info)
};

static struct platform_device sec_device_jack= {
		.name			= "sec_jack",
		.id 			= -1,
		.dev			= {
				.platform_data	= &sec_jack_pdata,
		}
};
#endif

 /* touch screen device init */
static void __init qt_touch_init(void)
{
    int gpio;

    gpio = S5PV210_GPH2(1);
    gpio_request(gpio, "TOUCH_EN");
    s3c_gpio_cfgpin(gpio, S3C_GPIO_OUTPUT);
    gpio_direction_output(gpio, 1);

    gpio = GPIO_TOUCH_INT;
    gpio_request(gpio, "TOUCH_INT");
    s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(0xf));
    s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
}

#if defined(CONFIG_MACH_P1_LTN)
#if defined(CONFIG_VIDEO_NM6XX)
static void __init nmi_i2s_cfg_gpio_init(void)
{
	s3c_gpio_cfgpin(GPIO_I2S_SCLK_18V, S3C_GPIO_SFN(0x4));
	s3c_gpio_cfgpin(GPIO_I2S_MCLK_18V, S3C_GPIO_SFN(0x4));
	s3c_gpio_cfgpin(GPIO_I2S_LRCLK_18V, S3C_GPIO_SFN(0x4));
	s3c_gpio_cfgpin(GPIO_I2S_DATA_18V, S3C_GPIO_SFN(0x4));

	s3c_gpio_setpull(GPIO_I2S_SCLK_18V, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_I2S_MCLK_18V, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_I2S_LRCLK_18V, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_I2S_DATA_18V, S3C_GPIO_PULL_NONE);
}
#else
static void __init nmi_pwr_disable(void)
{	
	int err = 0;

	if (HWREV == 13)    // Disable the ISDBT PWR : Only Latin HW 0.3
	{		
		err = gpio_request(GPIO_ISDBT_PWR_EN, "ISDBT_EN");
		if (err) 
		{
			printk(KERN_ERR "failed to request GPIO_ISDBT_PWR_EN for TV control\n");
		}

		gpio_request(GPIO_ISDBT_PWR_EN,"ISDBT_EN");
		udelay(50);
		gpio_direction_output(GPIO_ISDBT_PWR_EN, 0);
		gpio_free(GPIO_ISDBT_PWR_EN);	
	}	
}
#endif
#endif



#define S3C_GPIO_SETPIN_ZERO         0
#define S3C_GPIO_SETPIN_ONE          1
#define S3C_GPIO_SETPIN_NONE	     2

struct gpio_init_data {
	uint num;
	uint cfg;
	uint val;
	uint pud;
	uint drv;
};

static struct gpio_init_data herring_init_gpios[] = {
	{
		.num	= S5PV210_GPB(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPB(1),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,

	}, {
		.num	= S5PV210_GPB(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPB(3),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPB(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPB(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPB(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPB(7),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPC0(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPC0(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPC0(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPC0(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPC0(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},
#if defined(CONFIG_MACH_P1_LTN)
 	{
		.num	= S5PV210_GPC1(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPC1(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPC1(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPC1(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPC1(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},
	/*{
		.num	= S5PV210_GPD0(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, */
#else
	/* {
		.num	= S5PV210_GPC1(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPC1(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPC1(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPC1(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPC1(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPD0(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, */
#endif
	 {
		.num	= S5PV210_GPD0(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPD0(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},
#if defined(CONFIG_MACH_P1_LTN)
	{
		.num	= S5PV210_GPD0(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},
#else	
// worked here, 2010. 11. 03  13:14 john
	/*{
		.num	= S5PV210_GPD0(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},*/
#endif
	{
		.num	= S5PV210_GPD1(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPD1(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPD1(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPD1(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPD1(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPD1(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPE0(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPE0(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPE0(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPE0(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPE0(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPE0(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPE0(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPE0(7),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},


	{
		.num	= S5PV210_GPE1(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPE1(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPE1(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPE1(3),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPE1(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPF3(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPF3(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPG0(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG0(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG0(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG0(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG0(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG0(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG0(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPG1(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG1(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, 
#if defined(CONFIG_MACH_P1_LTN) 
	{
		.num	= S5PV210_GPG1(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, 
#else
	{
		.num	= S5PV210_GPG1(2),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, 
#endif
	{
		.num	= S5PV210_GPG1(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG1(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG1(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG1(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPG2(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG2(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG2(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG2(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG2(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG2(5),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG2(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPG3(0),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG3(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG3(2),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG3(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG3(4),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG3(5),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPG3(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPH0(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH0(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH0(2),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH0(3),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH0(4),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH0(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, { 
		.num	= S5PV210_GPH0(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH0(7),
		.cfg	= S3C_GPIO_SFN(0xF),
		.val	= S3C_GPIO_SETPIN_NONE,
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
		.pud	= S3C_GPIO_PULL_NONE,
#elif defined(CONFIG_MACH_P1_CDMA)
		.pud	= S3C_GPIO_PULL_UP,
#endif
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPH1(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH1(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH1(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH1(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, { 
		.num	= S5PV210_GPH1(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH1(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH1(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH1(7),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPH2(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH2(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH2(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH2(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH2(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH2(5),
		.cfg	= S3C_GPIO_SFN(0xF),
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH2(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH2(7),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPH3(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH3(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH3(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH3(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH3(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH3(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPH3(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
		.num	= S5PV210_GPH3(7),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
#elif defined(CONFIG_MACH_P1_CDMA)
		.num    = S5PV210_GPH3(7),
		.cfg    = S3C_GPIO_OUTPUT,
		.val    = S3C_GPIO_SETPIN_ZERO,
		.pud    = S3C_GPIO_PULL_DOWN,
		.drv    = S3C_GPIO_DRVSTR_4X,
#endif
	}, {
		.num	= S5PV210_GPI(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPI(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPI(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPI(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPI(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPI(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPI(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ0(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ0(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ0(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ0(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ0(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ0(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ0(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ0(7),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ1(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ1(1),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ1(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ1(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ1(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ1(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPJ2(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ2(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ2(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ2(3),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ2(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ2(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
//	}, {
//		.num	= S5PV210_GPJ2(6),
//		.cfg	= S3C_GPIO_INPUT,
//		.val	= S3C_GPIO_SETPIN_NONE,
//		.pud	= S3C_GPIO_PULL_DOWN,
//		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ2(7),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPJ3(0),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ3(1),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ3(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, { 
		.num	= S5PV210_GPJ3(3),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ3(4),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ3(5),  // TA_EN
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ3(6),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ3(7),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_GPJ4(0),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ4(1),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ4(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ4(3),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_GPJ4(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},
#if !defined(CONFIG_MACH_P1_LTN)
	{
		.num	= S5PV210_MP01(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, 
#endif
	{
		.num	= S5PV210_MP01(2),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, 
#if !defined(CONFIG_MACH_P1_LTN) 
        {
		.num	= S5PV210_MP01(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, 
#endif
	{
		.num	= S5PV210_MP01(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP01(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP01(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP01(7),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_MP02(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP02(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP02(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP02(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, 
	{
		.num	= S5PV210_MP03(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP03(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP03(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP03(7),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_MP04(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP04(2),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP04(3),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, 
#if defined(CONFIG_PN544) && !defined(CONFIG_MACH_P1_LTN) 
	{ /* NFC_SCL_18V - has external pull up resistor */
		.num	= S5PV210_MP04(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, 
#endif
#if defined(CONFIG_PN544)
        { /* NFC_SDA_18V - has external pull up resistor */
		.num	= S5PV210_MP04(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, 
#endif
	{
		.num	= S5PV210_MP04(6),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= S3C_GPIO_SETPIN_ZERO,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP04(7),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},

	{
		.num	= S5PV210_MP05(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP05(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP05(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP05(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_NONE,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, {
		.num	= S5PV210_MP05(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	}, 
#if !defined(CONFIG_MACH_P1_LTN)
       {
		.num	= S5PV210_MP05(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= S3C_GPIO_SETPIN_NONE,
		.pud	= S3C_GPIO_PULL_DOWN,
		.drv	= S3C_GPIO_DRVSTR_1X,
	},
#endif
};

void s3c_config_gpio_table(void)
{
	u32 i, gpio;

	for (i = 0; i < ARRAY_SIZE(herring_init_gpios); i++) {
		gpio = herring_init_gpios[i].num;
		if (gpio <= S5PV210_GPJ4(4)) {
			s3c_gpio_cfgpin(gpio, herring_init_gpios[i].cfg);
			s3c_gpio_setpull(gpio, herring_init_gpios[i].pud);

			if (herring_init_gpios[i].val != S3C_GPIO_SETPIN_NONE)
				gpio_set_value(gpio, herring_init_gpios[i].val);

			s3c_gpio_set_drvstrength(gpio, herring_init_gpios[i].drv);
		}
	}
}

#define S5PV210_PS_HOLD_CONTROL_REG (S3C_VA_SYS+0xE81C)
static void p1_power_off(void)
{
	while (1) {
		/* Check reboot charging */
		if (set_cable_status) {
			/* watchdog reset */
			pr_info("%s: charger connected, rebooting\n", __func__);
			writel(3, S5P_INFORM6);
			arch_reset('r', NULL);
			pr_crit("%s: waiting for reset!\n", __func__);
			while (1);
		}

		/* wait for power button release */
		if (gpio_get_value(GPIO_nPOWER)) {
			pr_info("%s: set PS_HOLD low\n", __func__);

			/* PS_HOLD high  PS_HOLD_CONTROL, R/W, 0xE010_E81C */
			writel(readl(S5PV210_PS_HOLD_CONTROL_REG) & 0xFFFFFEFF,
			       S5PV210_PS_HOLD_CONTROL_REG);

			pr_crit("%s: should not reach here!\n", __func__);
		}

		/* if power button is not released, wait and check TA again */
		pr_info("%s: PowerButton is not released.\n", __func__);
		mdelay(1000);
	}
}

/* this table only for B4 board */
static unsigned int herring_sleep_gpio_table[][3] = {
	{ S5PV210_GPA0(0), S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPA0(1), S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPA0(2), S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPA0(3), S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPA0(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPA0(5), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPA0(6), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPA0(7), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},

	{ S5PV210_GPA1(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPA1(1), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPA1(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPA1(3), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},

	{ S5PV210_GPB(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPB(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPB(2),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPB(3),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPB(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPB(5),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPB(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPB(7),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},

	{ S5PV210_GPC0(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPC0(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPC0(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPC0(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPC0(4), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},

	{ S5PV210_GPC1(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPC1(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPC1(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPC1(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPC1(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_GPD0(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPD0(1), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPD0(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPD0(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_GPD1(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPD1(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPD1(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPD1(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPD1(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPD1(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_GPE0(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPE0(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPE0(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPE0(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPE0(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPE0(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPE0(6), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPE0(7), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_GPE1(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPE1(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPE1(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPE1(3), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPE1(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_GPF0(0), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF0(1), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF0(2), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF0(3), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF0(4), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF0(5), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF0(6), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF0(7), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},

	{ S5PV210_GPF1(0), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF1(1), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF1(2), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF1(3), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF1(4), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF1(5), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF1(6), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF1(7), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},

	{ S5PV210_GPF2(0), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF2(1), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF2(2), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF2(3), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF2(4), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF2(5), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF2(6), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF2(7), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},

	{ S5PV210_GPF3(0), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF3(1), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF3(2), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF3(3), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF3(4), S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPF3(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_GPG0(0), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPG0(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, // NAND_CMD
	{ S5PV210_GPG0(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPG0(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, // NAND_D(0)
	{ S5PV210_GPG0(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, // NAND_D(1)
	{ S5PV210_GPG0(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, // NAND_D(2)
	{ S5PV210_GPG0(6), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, // NAND_D(3)

	{ S5PV210_GPG1(0), S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPG1(1), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPG1(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPG1(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, // NAND_D(4)
	{ S5PV210_GPG1(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, // NAND_D(5)
	{ S5PV210_GPG1(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, // NAND_D(6)
	{ S5PV210_GPG1(6), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, // NAND_D(7)

	{ S5PV210_GPG2(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPG2(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPG2(2), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPG2(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPG2(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPG2(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPG2(6), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_GPG3(0), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPG3(1), S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPG3(2), S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPG3(3), S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPG3(4), S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPG3(5), S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPG3(6), S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE},

	/* Alive part ending and off part start*/
	{ S5PV210_GPI(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPI(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPI(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPI(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPI(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPI(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPI(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_GPJ0(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPJ0(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPJ0(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ0(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ0(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ0(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPJ0(6), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ0(7), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},

	{ S5PV210_GPJ1(0), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ1(1), S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE}, // GPIO_MASSMEMORY_EN
	{ S5PV210_GPJ1(2), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ1(3), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ1(4), S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ1(5), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},

	{ S5PV210_GPJ2(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPJ2(1), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, // GYRO_CS (NC)
	{ S5PV210_GPJ2(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPJ2(3), S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ2(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPJ2(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPJ2(6), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ2(7), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},

	{ S5PV210_GPJ3(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPJ3(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPJ3(2), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ3(3), S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ3(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ3(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ3(6), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPJ3(7), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_GPJ4(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ4(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_GPJ4(2), S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ4(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_GPJ4(4), S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},

	/* memory part */
	{ S5PV210_MP01(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP01(1), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP01(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP01(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP01(4), S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP01(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP01(6), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP01(7), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_MP02(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP02(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP02(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP02(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_MP03(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP03(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP03(2), S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP03(3), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP03(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP03(5), S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP03(6), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP03(7), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_MP04(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP04(1), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP04(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP04(3), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP04(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP04(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP04(6), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP04(7), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_MP05(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP05(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP05(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP05(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP05(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP05(5), S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ S5PV210_MP05(6), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP05(7), S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},

	{ S5PV210_MP06(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP06(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP06(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP06(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP06(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP06(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP06(6), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP06(7), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ S5PV210_MP07(0), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP07(1), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP07(2), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP07(3), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP07(4), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP07(5), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP07(6), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ S5PV210_MP07(7), S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	/* Memory part ending and off part ending */
};


static unsigned int p1_sleep_gpio_table[][3] = {

	{S5PV210_GPA0(0),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(2),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(3),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(5),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPA1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(3),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPB(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(1),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(2),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPB(3),  // GPIO_BT_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPC0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPC1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD0(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_CODEC_LDO_EN
	{S5PV210_GPF3(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPF3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPG1(2),  // GPIO_WLAN_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
	{S5PV210_GPG1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},

	{S5PV210_GPI(0), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPI(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(3), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPI(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

//	{S5PV210_GPJ0(0),  // GPIO_WLAN_BT_EN
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ2(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ3(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(5),  // TA_EN
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ4(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_MICBIAS_EN
	{S5PV210_GPJ4(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPJ4(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	/* memory part */
	{S5PV210_MP01(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(1), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(4), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},
	{S5PV210_MP01(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP02(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP03(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(2), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_MP03(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
 	{S5PV210_MP03(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP04(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(7), // MHL_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP05(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(7), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, //UART_SEL prev

	{S5PV210_MP06(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP07(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	/* Memory part ending and off part ending */
};

static unsigned int p1_r05_sleep_gpio_table[][3] = {

	{S5PV210_GPA0(0),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(2),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(3),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(5),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPA1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(3),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPB(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(1),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(2),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPB(3),  // GPIO_BT_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPC0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPC1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD0(0),  // KEY_LED_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(3),  // LCD_CABC_PWM
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_CODEC_LDO_EN
	{S5PV210_GPF3(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPF3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPG1(2),  // GPIO_WLAN_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
	{S5PV210_GPG1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(2),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},

	{S5PV210_GPI(0), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPI(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(3), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPI(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

//	{S5PV210_GPJ0(0),  // GPIO_WLAN_BT_EN
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(5),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(1),  // VIBTONE_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(1),  // GYRO_CS (NC)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(4),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(5),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ3(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(5),  // TA_EN
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ4(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(1),  // CURR_ADJ
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_MICBIAS_EN
	{S5PV210_GPJ4(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPJ4(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	/* memory part */
	{S5PV210_MP01(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(4), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},
	{S5PV210_MP01(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP02(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP03(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(2), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_MP03(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
 	{S5PV210_MP03(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP04(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(7), // MHL_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},

	{S5PV210_MP05(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(7), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, //UART_SEL prev

	{S5PV210_MP06(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP07(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	/* Memory part ending and off part ending */
};

static unsigned int p1_r08_sleep_gpio_table[][3] = {
	{S5PV210_GPJ1(4),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(3),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(5),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
};

static unsigned int p1_r09_sleep_gpio_table[][3] = {

	{S5PV210_GPA0(0),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(2),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(3),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(5),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPA1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(3),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPB(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(1),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(2),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPB(3),  // GPIO_BT_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPC0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPC1(0),  // CMC_SLEEP
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(1),  // CMC_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(2),  // CMC_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(3),  // CMC_SHDN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(4),  // CMC_BYPASS
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD0(0),  // KEY_LED_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(3),  // LCD_CABC_PWM
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_CODEC_LDO_EN
	{S5PV210_GPF3(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPF3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPG1(2),  // GPIO_WLAN_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
	{S5PV210_GPG1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(2),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(3),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},

	{S5PV210_GPI(0), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPI(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(3), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPI(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

//	{S5PV210_GPJ0(0),  // GPIO_WLAN_BT_EN
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(5),  // TOUCH_INT
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(1),  // MESSMEMORY_EN
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPJ1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(4),  // OVF_FLAG
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ2(0),  // CHARGER_SDA_2.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(1),  // CHARGER_SCL_2.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(2),  // HDMI_EN1
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(4),  // MESSMEMORY_EN2
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(5),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ3(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(5),  // TA_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ4(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(1),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_MICBIAS_EN
	{S5PV210_GPJ4(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPJ4(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	/* memory part */
	{S5PV210_MP01(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(4), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},
	{S5PV210_MP01(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP02(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP03(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(2), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_MP03(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
 	{S5PV210_MP03(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP04(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(3),  // CMC_SCL_1.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(5),  // CMC_SDA_1.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(6),  // GPS_CNBTL
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(7), // MHL_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},

	{S5PV210_MP05(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(7), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, //UART_SEL prev

	{S5PV210_MP06(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP07(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	/* Memory part ending and off part ending */
};

static unsigned int p1_r11_sleep_gpio_table[][3] = {
	{S5PV210_GPJ1(4),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
};

static unsigned int p1_r12_sleep_gpio_table[][3] = {

	{S5PV210_GPA0(0),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(2),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(3),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(4),
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
#endif
	{S5PV210_GPA0(5),
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
#endif
	{S5PV210_GPA0(6), 
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPA0(7),
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
#endif

	{S5PV210_GPA1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPA1(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(2), 
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPA1(3),
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
#endif

	{S5PV210_GPB(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(1),
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
#endif
	{S5PV210_GPB(2),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPB(3),  // GPIO_BT_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#if defined(CONFIG_MACH_P1_LTN)
	{S5PV210_GPB(4),  // HWREV_MODE3
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(5),  // HWREV_MODE2
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(6),  // HWREV_MODE1
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(7),  // HWREV_MODE0
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPB(4),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPB(5),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPB(6),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPB(7),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPC0(0), 
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPC0(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPC0(2), 
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPC0(3), 
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPC0(4), 
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
#endif

#if defined(CONFIG_MACH_P1_LTN) 
	{S5PV210_GPC1(0),  // I2S_SCLK_1.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPC1(1),  // I2S_MCLK_1.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPC1(2),  // I2S_LRCLK_1.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPC1(3),  // I2S_DATA_1.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPC1(4),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#else
	{S5PV210_GPC1(0),  // CMC_SLEEP
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(1),  // CMC_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(2),  // CMC_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(3),  // CMC_SHDN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(4),  // CMC_BYPASS
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPD0(0),  // KEY_LED_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

#if defined(CONFIG_MACH_P1_LTN) 
	{S5PV210_GPD0(2),   // HWREV_MODE4
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPD0(2),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPD0(3),  // LCD_CABC_PWM
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPD1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPD1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPD1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPE0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(4),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPF0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_CODEC_LDO_EN
	{S5PV210_GPF3(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPF3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(1),  // NAND_CMD
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
#if defined(CONFIG_MACH_P1_LTN)
	{S5PV210_GPG0(2),   // TOUCH_INT
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPG0(2),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
#endif
	{S5PV210_GPG0(3),  // NAND_D(0)
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPG0(4),  // NAND_D(1)
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPG0(5),  // NAND_D(2)
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPG0(6),  // NAND_D(3)
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif

	{S5PV210_GPG1(0),  // GPS_nRST
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
#endif
	{S5PV210_GPG1(1),  // GPS_PWR_EN
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
#if defined(CONFIG_MACH_P1_LTN) && defined(CONFIG_VIDEO_NM6XX)
	{S5PV210_GPG1(2), // ISDBT_RSTn 
			  S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
#else
	{S5PV210_GPG1(2),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
#endif
	{S5PV210_GPG1(3),  // NAND_D(4)
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPG1(4),  // NAND_D(5)
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPG1(5),  // NAND_D(6)
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPG1(6),  // NAND_D(7)
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif

	{S5PV210_GPG2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(1),  // T_FLASH_CLK
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(3),  // T_FLASH_D(0)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG2(4),  // T_FLASH_D(1)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG2(5),  // T_FLASH_D(2)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG2(6),  // T_FLASH_D(3)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPG3(0),  // WLAN_SDIO_CLK
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(1),  // WLAN_SDIO_CMD
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPG3(2),  // WLAN_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(3),  // WLAN_SDIO_D(0)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(4),  // WLAN_SDIO_D(1)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(5),  // WLAN_SDIO_D(2)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(6),  // WLAN_SDIO_D(3)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},

	{S5PV210_GPI(0), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPI(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(3), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(5),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPI(6),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

#if defined(CONFIG_MACH_P1_LTN) 
#if defined(CONFIG_VIDEO_NM6XX)
	{S5PV210_GPJ0(0),  // ISDBT_SCL
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(1),  // ISDBT_SDA
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(2), // ISDBT_CLK
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPJ0(3), // ISDBT_SYNC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPJ0(4), // ISDBT_VALID
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN},  
	{S5PV210_GPJ0(5),  // ISDBT_DATA
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN},  
	{S5PV210_GPJ0(6), // ISDBT_ERR
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
#else
	{S5PV210_GPJ0(0),  // ISDBT_SCL
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPJ0(1),  // ISDBT_SDA
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPJ0(2), // ISDBT_CLK
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPJ0(3), // ISDBT_SYNC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPJ0(4), // ISDBT_VALID
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},  
	{S5PV210_GPJ0(5),  // ISDBT_DATA
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},  
	{S5PV210_GPJ0(6), // ISDBT_ERR
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
#else
//	{S5PV210_GPJ0(0),  // GPIO_WLAN_BT_EN
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(5),  // TOUCH_INT
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
	{S5PV210_GPJ0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPJ0(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#if defined(CONFIG_MACH_P1_LTN)
//	{S5PV210_GPJ1(0),   // GPIO_WLAN_BT_EN
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPJ1(0),  // NC
#if defined(CONFIG_MACH_P1_GSM)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
#endif
	{S5PV210_GPJ1(1),  // MESSMEMORY_EN
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPJ1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#if defined(CONFIG_MACH_P1_LTN) && defined (CONFIG_VIDEO_NM6XX)
	{S5PV210_GPJ1(3), // ISDBT_PWR_EN
			 S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
#else
	{S5PV210_GPJ1(3),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
#if defined(CONFIG_MACH_P1_CDMA)
	{S5PV210_GPJ1(4),
                        S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
#endif
	{S5PV210_GPJ1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ2(0),  // CHARGER_SDA_2.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(1),  // CHARGER_SCL_2.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(2),  // HDMI_EN1
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(4),  // BT_WAKE
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(5),  // WLAN_WAKE
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ3(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#if defined(CONFIG_MACH_P1_LTN)
#if defined(CONFIG_VIDEO_NM6XX)
	{S5PV210_GPJ3(2),  // ATV_RSTn (Latin Rev0.3)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPJ3(2),  // ATV_RSTn (Latin Rev0.3)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
#else
	{S5PV210_GPJ3(2),  // NC(Rev0.6), CAM_LDO_EN(Rev0.7)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPJ3(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(5),  // TA_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ4(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(1),  // MASSMEMORY_EN2
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_MICBIAS_EN
	{S5PV210_GPJ4(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPJ4(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	/* memory part */
#if defined(CONFIG_MACH_P1_LTN)
	{S5PV210_MP01(0),  // CMC_SHDN
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
	{S5PV210_MP01(1),  // CMC_SLEEP
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_MP01(0),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP01(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_MP01(2),  // RESET_REQ_N
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#if defined(CONFIG_MACH_P1_LTN)
	{S5PV210_MP01(3),  // CMC_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_MP01(3),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_MP01(4),  // AP_NANDCS
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(5),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
	{S5PV210_MP01(6),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP01(7),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP02(0),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP02(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP02(2),  // VCC_1.8V_PDA
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(3),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP03(0),  // LVDS_SHDN
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(2),  // NC (*)
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_MP03(3),  // PDA_ACTIVE
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(4),  // VCC_1.8V_PDA
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(5),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
 	{S5PV210_MP03(6),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(7),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP04(0),  // GPS_CNTL
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
#endif
	{S5PV210_MP04(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP04(2),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP04(3),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#if defined(CONFIG_MACH_P1_LTN)
	{S5PV210_MP04(4),   // CMC_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_MP04(4),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_MP04(5),  // CMC_SDA_1.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(6),  // CMC_SCL_1.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(7), // MHL_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},

	{S5PV210_MP05(0),  // LCD_ID
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP05(2),  // AP_SCL
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(3),  // AP_SDA
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(4),  // NC
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#elif defined(CONFIG_MACH_P1_CDMA)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
#if defined(CONFIG_MACH_P1_LTN)
	{S5PV210_MP05(6),  // CMC_BYPASS
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_MP05(6),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_MP05(7),  //UART_SEL
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},

	{S5PV210_MP06(0),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(1),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(2),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(3),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(4),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(5),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(6),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(7),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP07(0),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(1),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(2),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(3),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(4),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(5),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(6),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(7),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	/* Memory part ending and off part ending */
};

static unsigned int p1_r15_sleep_gpio_table[][3] = {
	{S5PV210_GPD0(1),  // VIBTONE_PWM
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(3),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPJ1(3),  // VIBTONE_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(2),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP05(0),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP05(1),  // EAR_MICBIAS_EN
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(5),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
};

static unsigned int p1_r16_sleep_gpio_table[][3] = {
	{S5PV210_MP04(2),  // FLASH_EN1
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(3),  // FLASH_EN2
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
};

static unsigned int p1_r18_sleep_gpio_table[][3] = {
	{S5PV210_MP01(5),  // EAR_MICBIAS_EN
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
};

static unsigned int p1_lcd_amoled_sleep_gpio_table[][3] = {
	{S5PV210_GPJ1(3),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_GPJ2(6),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_MP05(5),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
};

static unsigned int p1_lcd_tft_sleep_gpio_table[][3] = {
	{S5PV210_GPJ1(3),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
	{S5PV210_GPJ2(6),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
	{S5PV210_MP05(5),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
};

#if defined(CONFIG_KEYBOARD_P1)
static unsigned int p1_keyboard_sleep_gpio_table[][3] = {
	{S5PV210_GPJ1(4),  // ACCESSORY_EN
		S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_UP},
};
#endif





void s3c_config_sleep_gpio_table(int array_size, unsigned int (*gpio_table)[3])
{
	u32 i, gpio;

	for (i = 0; i < array_size; i++) {
		gpio = gpio_table[i][0];
		s3c_gpio_slp_cfgpin(gpio, gpio_table[i][1]);
		s3c_gpio_slp_setpull_updown(gpio, gpio_table[i][2]);
	}
}

#define s3c_gpio_setpin gpio_set_value

void s3c_config_sleep_gpio(void)
{
	// Setting the alive mode registers
#if defined(CONFIG_MACH_P1_CDMA)
	s3c_gpio_cfgpin(GPIO_AP_PS_HOLD,S3C_GPIO_INPUT); // Not used in Froyo also but confingured as similar
	s3c_gpio_setpull(GPIO_AP_PS_HOLD,S3C_GPIO_PULL_DOWN);	
#endif
	s3c_gpio_cfgpin(GPIO_ACC_INT, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_ACC_INT, S3C_GPIO_PULL_DOWN);
//	s3c_gpio_setpin(GPIO_ACC_INT, 0);

	s3c_gpio_cfgpin(GPIO_BUCK_1_EN_A, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_BUCK_1_EN_A, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpin(GPIO_BUCK_1_EN_A, 0);

	s3c_gpio_cfgpin(GPIO_BUCK_1_EN_B, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_BUCK_1_EN_B, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpin(GPIO_BUCK_1_EN_B, 0);

	s3c_gpio_cfgpin(GPIO_BUCK_2_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_BUCK_2_EN, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpin(GPIO_BUCK_2_EN, 0);

//	s3c_gpio_cfgpin(GPIO_ACCESSORY_INT, S3C_GPIO_INPUT);
//	s3c_gpio_setpull(GPIO_ACCESSORY_INT, S3C_GPIO_PULL_NONE);
#if defined(CONFIG_MACH_P1_CDMA)
	s3c_gpio_cfgpin(GPIO_ACCESSORY_INT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_ACCESSORY_INT, S3C_GPIO_PULL_NONE);
#endif
//	s3c_gpio_setpin(GPIO_ACCESSORY_INT, 0);

	if(HWREV >= 0x4) {  // NC
		if(HWREV == 14 || HWREV == 15) {  // RF_TOUCH_INT (P1000 Rev0.8, Rev0.9)
			s3c_gpio_cfgpin(GPIO_GPH06, S3C_GPIO_INPUT);
			s3c_gpio_setpull(GPIO_GPH06, S3C_GPIO_PULL_NONE);
			//s3c_gpio_setpin(GPIO_GPH06, 0);
		}
		else {  // NC
			s3c_gpio_cfgpin(GPIO_GPH06, S3C_GPIO_INPUT);
			s3c_gpio_setpull(GPIO_GPH06, S3C_GPIO_PULL_DOWN);
			//s3c_gpio_setpin(GPIO_GPH06, 0);
		}
	}
	else {  // DET_3.5
		//s3c_gpio_cfgpin(GPIO_DET_35, S3C_GPIO_INPUT);
		//s3c_gpio_setpull(GPIO_DET_35, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_DET_35, 0);
	}

	//s3c_gpio_cfgpin(GPIO_AP_PMIC_IRQ, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_AP_PMIC_IRQ, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_AP_PMIC_IRQ, 0);

	if(HWREV >= 0x4) {  // DET_3.5
		//s3c_gpio_cfgpin(GPIO_DET_35_R04, S3C_GPIO_INPUT);
		//s3c_gpio_setpull(GPIO_DET_35_R04, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_DET_35_R04, 0);
	}
	else {  // NC
		s3c_gpio_cfgpin(GPIO_GPH10, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH10, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH10, 0);
	}

	//s3c_gpio_cfgpin(GPIO_TA_nCHG, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_TA_nCHG, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_TA_nCHG, 0);

	s3c_gpio_cfgpin(GPIO_MHL_INT, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_MHL_INT, S3C_GPIO_PULL_DOWN);
//	s3c_gpio_setpin(GPIO_MHL_INT, 1);

	//s3c_gpio_cfgpin(GPIO_nINT_ONEDRAM_AP, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_nINT_ONEDRAM_AP, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_nINT_ONEDRAM_AP, 0);

	if(HWREV >= 0x4) {  // SEND_END
		//s3c_gpio_cfgpin(GPIO_EAR_SEND_END_R04, S3C_GPIO_INPUT);
		//s3c_gpio_setpull(GPIO_EAR_SEND_END_R04, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_EAR_SEND_END_R04, 0);
	}
	else {  // NC
		s3c_gpio_cfgpin(GPIO_GPH14, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH14, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH14, 0);
	}

	s3c_gpio_cfgpin(GPIO_HDMI_HPD, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_HDMI_HPD, S3C_GPIO_PULL_DOWN);
//	s3c_gpio_setpin(GPIO_HDMI_HPD,0);

	//s3c_gpio_cfgpin(GPIO_FUEL_ARLT, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_FUEL_ARLT, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_FUEL_ARLT, 0);

	//s3c_gpio_cfgpin(GPIO_PHONE_ACTIVE, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_PHONE_ACTIVE, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_PHONE_ACTIVE, 0);

	if(HWREV >= 12) {  // REMOTE_SENSE_IRQ (GT-P1000 Rev0.6)
		s3c_gpio_cfgpin(GPIO_REMOTE_SENSE_IRQ, S3C_GPIO_INPUT);
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
		s3c_gpio_setpull(GPIO_REMOTE_SENSE_IRQ, S3C_GPIO_PULL_NONE);
#elif defined(CONFIG_MACH_P1_CDMA)
		s3c_gpio_setpull(GPIO_REMOTE_SENSE_IRQ, S3C_GPIO_PULL_DOWN);
#endif
		//s3c_gpio_setpin(GPIO_REMOTE_SENSE_IRQ, 0);
	}
	else {
		s3c_gpio_cfgpin(GPIO_GPH20, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH20, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH20, 0);
	}

	if(HWREV >= 12) {  // TOUCH_EN (GT-P1000 Rev0.6)
		s3c_gpio_cfgpin(GPIO_TOUCH_EN_REV06, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_TOUCH_EN_REV06, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_TOUCH_EN_REV06, 0);
	}
	else {  // NC
		s3c_gpio_cfgpin(GPIO_GPH21, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH21, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH21, 0);
	}

	if(HWREV >= 12) {  // GYRO_INT (GT-P1000 Rev0.6)
		s3c_gpio_cfgpin(GPIO_GYRO_INT, S3C_GPIO_INPUT);
		s3c_gpio_setpull(GPIO_GYRO_INT, S3C_GPIO_PULL_DOWN);
		//s3c_gpio_setpin(GPIO_GYRO_INT, 0);
	}
	else {  // NC
		s3c_gpio_cfgpin(GPIO_GPH22, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH22, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH22, 0);
	}

	if(HWREV >= 0x5) {  // NC
		if(HWREV == 15) {  // WAKEUP_KEY(P1000 Rev0.9)
			s3c_gpio_cfgpin(GPIO_GPH23, S3C_GPIO_INPUT);
			s3c_gpio_setpull(GPIO_GPH23, S3C_GPIO_PULL_NONE);
			//s3c_gpio_setpin(GPIO_GPH23, 0);
		}
		else {  // NC
			s3c_gpio_cfgpin(GPIO_GPH23, S3C_GPIO_INPUT);
			s3c_gpio_setpull(GPIO_GPH23, S3C_GPIO_PULL_DOWN);
			//s3c_gpio_setpin(GPIO_GPH23, 0);
		}
	}
	else {  // TOUCH_KEY_INT
		s3c_gpio_cfgpin(GPIO_TOUCH_KEY_INT, S3C_GPIO_INPUT);
		s3c_gpio_setpull(GPIO_TOUCH_KEY_INT, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_GPH23, 0);
	}

	//s3c_gpio_cfgpin(GPIO_WLAN_HOST_WAKE, S3C_GPIO_OUTPUT);
	//s3c_gpio_setpull(GPIO_WLAN_HOST_WAKE, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_WLAN_HOST_WAKE, 0);

	//s3c_gpio_cfgpin(GPIO_BT_HOST_WAKE, S3C_GPIO_OUTPUT);
	//s3c_gpio_setpull(GPIO_BT_HOST_WAKE, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_BT_HOST_WAKE, 0);

	//s3c_gpio_cfgpin(GPIO_nPOWER, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_nPOWER, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_nPOWER, 0);

	//s3c_gpio_cfgpin(GPIO_JACK_nINT, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_JACK_nINT, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_JACK_nINT, 0);

#if 0 // keypad
	s3c_gpio_cfgpin(GPIO_KBR0, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_KBR0, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_KBR0, 0);

	s3c_gpio_cfgpin(GPIO_KBR1, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_KBR1, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_KBR1, 0);
#endif

	s3c_gpio_cfgpin(GPIO_MSENSE_IRQ, S3C_GPIO_INPUT);
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
	s3c_gpio_setpull(GPIO_MSENSE_IRQ, S3C_GPIO_PULL_NONE);
#elif defined(CONFIG_MACH_P1_CDMA)
	s3c_gpio_setpull(GPIO_MSENSE_IRQ, S3C_GPIO_PULL_UP);
#endif
	//s3c_gpio_setpin(GPIO_MSENSE_IRQ, 0);

	if(HWREV >= 0x6) {  // SIM_DETECT
		//s3c_gpio_cfgpin(GPIO_SIM_nDETECT, S3C_GPIO_INPUT);
		//s3c_gpio_setpull(GPIO_SIM_nDETECT, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_SIM_nDETECT, 0);
	}
	else {  // NC
		s3c_gpio_cfgpin(GPIO_GPH33, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH33, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH33, 0);
	}

	//s3c_gpio_cfgpin(GPIO_T_FLASH_DETEC, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_T_FLASH_DETECT, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_T_FLASH_DETECT, 0);

	if(HWREV >= 11) {   // DOCK_INT (GT-P1000 Rev0.5)
		//s3c_gpio_cfgpin(GPIO_DOCK_INT, S3C_GPIO_INPUT);
		//s3c_gpio_setpull(GPIO_DOCK_INT, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_DOCK_INT, 0);
	}
	else {  // NC
		s3c_gpio_cfgpin(GPIO_GPH35, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH35, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH35, 0);
	}

	if(HWREV >= 0x4) {  // NC
		s3c_gpio_cfgpin(GPIO_GPH36, S3C_GPIO_INPUT);
		s3c_gpio_setpull(GPIO_GPH36, S3C_GPIO_PULL_DOWN);
		//s3c_gpio_setpin(GPIO_GPH36, 0);
	}
	else {  // SEND_END
		//s3c_gpio_cfgpin(GPIO_EAR_SEND_END, S3C_GPIO_INPUT);
		//s3c_gpio_setpull(GPIO_EAR_SEND_END, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_EAR_SEND_END, 0);
	}

	s3c_gpio_cfgpin(GPIO_CP_RST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_CP_RST, S3C_GPIO_PULL_UP);
	//s3c_gpio_setpin(GPIO_CP_RST, 1);

	if(HWREV >= 12) {  // Above P1000 Rev0.6 (1.2)
		s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r12_sleep_gpio_table),
			p1_r12_sleep_gpio_table);

		if(HWREV >= 15) {  // Above P1000 Rev0.9
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r15_sleep_gpio_table),
				p1_r15_sleep_gpio_table);
		}
		
		if(HWREV >= 16) {  // Above P1000 Rev1.0
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r16_sleep_gpio_table),
				p1_r16_sleep_gpio_table);
		}

		if(HWREV >= 18) {  // Above P1000 Rev1.2
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r18_sleep_gpio_table),
				p1_r18_sleep_gpio_table);
		}
#if defined(CONFIG_KEYBOARD_P1)
extern bool keyboard_enable;
            if(keyboard_enable)
            {
                    s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_keyboard_sleep_gpio_table),
				p1_keyboard_sleep_gpio_table);
            }
#endif
	}
	else if(HWREV >= 8) {  // Above P1000 Rev0.2 (0.8)
		s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r09_sleep_gpio_table),
			p1_r09_sleep_gpio_table);

		if(HWREV == 11) {  // Rev0.5 : only 1 gpio status is different from Rev0.3 (0.9)
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r11_sleep_gpio_table),
				p1_r11_sleep_gpio_table);
		}

		if(HWREV == 8) {  // Rev0.2 : only 3 gpio status is different from Rev0.3 (0.9)
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r08_sleep_gpio_table),
				p1_r08_sleep_gpio_table);
		}
	}
	else if(HWREV >= 5) {  // Above Rev0.5
		s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r05_sleep_gpio_table),
			p1_r05_sleep_gpio_table);
	}
	else {  // Under Rev0.4
		s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_sleep_gpio_table),
			p1_sleep_gpio_table);
	}

#if 0
	if (get_machine_type() == MACHINE_P1_AMOLED) {
		if(HWREV < 0x5)
			{
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_lcd_amoled_sleep_gpio_table),
				p1_lcd_amoled_sleep_gpio_table);
			}
		else
			{
			//PWM
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_lcd_tft_sleep_gpio_table),
				p1_lcd_tft_sleep_gpio_table);
			}
	}
	else if (get_machine_type() == MACHINE_P1_TFT) {
#endif		
		s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_lcd_tft_sleep_gpio_table),
			p1_lcd_tft_sleep_gpio_table);
//	}

#if defined(CONFIG_MACH_P1_LTN) && defined(CONFIG_VIDEO_NM6XX)
	if(HWREV >= 16) {  
		s3c_gpio_cfgpin(GPIO_ATV_RSTn_REV10, S3C_GPIO_INPUT);
		s3c_gpio_setpull(GPIO_ATV_RSTn_REV10, S3C_GPIO_PULL_DOWN);

		s3c_gpio_cfgpin(GPIO_ISDBT_PWR_EN_REV10, S3C_GPIO_INPUT);
		s3c_gpio_setpull(GPIO_ISDBT_PWR_EN_REV10, S3C_GPIO_PULL_DOWN);
	}

	if(HWREV == 16) {  

		s3c_gpio_cfgpin(GPIO_TV_CLK_EN, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_TV_CLK_EN, S3C_GPIO_PULL_NONE);
	}
#endif
}

EXPORT_SYMBOL(s3c_config_sleep_gpio);

#if 0
void s3c_config_sleep_gpio(void)
{
	/* setting the alive mode registers */
	s3c_gpio_cfgpin(S5PV210_GPH0(1), S3C_GPIO_INPUT);
	s3c_gpio_setpull(S5PV210_GPH0(1), S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(S5PV210_GPH0(3), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(S5PV210_GPH0(3), S3C_GPIO_PULL_NONE);
	gpio_set_value(S5PV210_GPH0(3), 0);

	s3c_gpio_cfgpin(S5PV210_GPH0(4), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(S5PV210_GPH0(4), S3C_GPIO_PULL_NONE);
	gpio_set_value(S5PV210_GPH0(4), 0);

	s3c_gpio_cfgpin(S5PV210_GPH0(5), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(S5PV210_GPH0(5), S3C_GPIO_PULL_NONE);
	gpio_set_value(S5PV210_GPH0(5), 0);

	s3c_gpio_cfgpin(S5PV210_GPH1(0), S3C_GPIO_INPUT);
	s3c_gpio_setpull(S5PV210_GPH1(0), S3C_GPIO_PULL_DOWN);

	s3c_gpio_cfgpin(S5PV210_GPH1(1), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(S5PV210_GPH1(1), S3C_GPIO_PULL_NONE);
	gpio_set_value(S5PV210_GPH1(1), 0);

	s3c_gpio_cfgpin(S5PV210_GPH1(2), S3C_GPIO_INPUT);
	s3c_gpio_setpull(S5PV210_GPH1(2), S3C_GPIO_PULL_DOWN);

	s3c_gpio_cfgpin(S5PV210_GPH1(4), S3C_GPIO_INPUT);
	s3c_gpio_setpull(S5PV210_GPH1(4), S3C_GPIO_PULL_DOWN);

	s3c_gpio_cfgpin(S5PV210_GPH1(5), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(S5PV210_GPH1(5), S3C_GPIO_PULL_NONE);
	gpio_set_value(S5PV210_GPH1(5), 0);

	s3c_gpio_cfgpin(S5PV210_GPH1(6), S3C_GPIO_INPUT);
	s3c_gpio_setpull(S5PV210_GPH1(6), S3C_GPIO_PULL_DOWN);

	s3c_gpio_cfgpin(S5PV210_GPH1(7), S3C_GPIO_INPUT);
	s3c_gpio_setpull(S5PV210_GPH1(7), S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(S5PV210_GPH2(0), S3C_GPIO_INPUT);
	s3c_gpio_setpull(S5PV210_GPH2(0), S3C_GPIO_PULL_DOWN);

	s3c_gpio_cfgpin(S5PV210_GPH2(3), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(S5PV210_GPH2(3), S3C_GPIO_PULL_NONE);
	gpio_set_value(S5PV210_GPH2(3), 0);

	s3c_gpio_cfgpin(S5PV210_GPH3(0), S3C_GPIO_INPUT);
	s3c_gpio_setpull(S5PV210_GPH3(0), S3C_GPIO_PULL_UP);

	s3c_gpio_cfgpin(S5PV210_GPH3(3), S3C_GPIO_INPUT);
	s3c_gpio_setpull(S5PV210_GPH3(3), S3C_GPIO_PULL_DOWN);

	s3c_gpio_cfgpin(S5PV210_GPH3(4), S3C_GPIO_INPUT);
	s3c_gpio_setpull(S5PV210_GPH3(4), S3C_GPIO_PULL_DOWN);

	// GYRO_INT (GT-P1000)
	s3c_gpio_cfgpin(GPIO_GYRO_INT, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_GYRO_INT, S3C_GPIO_PULL_DOWN);

}
EXPORT_SYMBOL(s3c_config_sleep_gpio);
#endif

#if 0 // for wifi
static unsigned int wlan_sdio_on_table[][4] = {
	{GPIO_WLAN_SDIO_CLK, GPIO_WLAN_SDIO_CLK_AF, GPIO_LEVEL_NONE,
		S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_CMD, GPIO_WLAN_SDIO_CMD_AF, GPIO_LEVEL_NONE,
		S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D0, GPIO_WLAN_SDIO_D0_AF, GPIO_LEVEL_NONE,
		S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D1, GPIO_WLAN_SDIO_D1_AF, GPIO_LEVEL_NONE,
		S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D2, GPIO_WLAN_SDIO_D2_AF, GPIO_LEVEL_NONE,
		S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D3, GPIO_WLAN_SDIO_D3_AF, GPIO_LEVEL_NONE,
		S3C_GPIO_PULL_NONE},
};

static unsigned int wlan_sdio_off_table[][4] = {
	{GPIO_WLAN_SDIO_CLK, 1, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_CMD, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D0, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D1, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D2, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D3, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
};

static int wlan_power_en(int onoff)
{
	if (onoff) {
		s3c_gpio_cfgpin(GPIO_WLAN_HOST_WAKE,
				S3C_GPIO_SFN(GPIO_WLAN_HOST_WAKE_AF));
		s3c_gpio_setpull(GPIO_WLAN_HOST_WAKE, S3C_GPIO_PULL_DOWN);

		s3c_gpio_cfgpin(GPIO_WLAN_WAKE,
				S3C_GPIO_SFN(GPIO_WLAN_WAKE_AF));
		s3c_gpio_setpull(GPIO_WLAN_WAKE, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_WLAN_WAKE, GPIO_LEVEL_LOW);

		s3c_gpio_cfgpin(GPIO_WLAN_nRST,
				S3C_GPIO_SFN(GPIO_WLAN_nRST_AF));
		s3c_gpio_setpull(GPIO_WLAN_nRST, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_WLAN_nRST, GPIO_LEVEL_HIGH);
		s3c_gpio_slp_cfgpin(GPIO_WLAN_nRST, S3C_GPIO_SLP_OUT1);
		s3c_gpio_slp_setpull_updown(GPIO_WLAN_nRST, S3C_GPIO_PULL_NONE);

		s3c_gpio_cfgpin(GPIO_WLAN_BT_EN, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_WLAN_BT_EN, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_WLAN_BT_EN, GPIO_LEVEL_HIGH);
		s3c_gpio_slp_cfgpin(GPIO_WLAN_BT_EN, S3C_GPIO_SLP_OUT1);
		s3c_gpio_slp_setpull_updown(GPIO_WLAN_BT_EN,
					S3C_GPIO_PULL_NONE);

		msleep(80);
	} else {
		gpio_set_value(GPIO_WLAN_nRST, GPIO_LEVEL_LOW);
		s3c_gpio_slp_cfgpin(GPIO_WLAN_nRST, S3C_GPIO_SLP_OUT0);
		s3c_gpio_slp_setpull_updown(GPIO_WLAN_nRST, S3C_GPIO_PULL_NONE);

		if (gpio_get_value(GPIO_BT_nRST) == 0) {
			gpio_set_value(GPIO_WLAN_BT_EN, GPIO_LEVEL_LOW);
			s3c_gpio_slp_cfgpin(GPIO_WLAN_BT_EN, S3C_GPIO_SLP_OUT0);
			s3c_gpio_slp_setpull_updown(GPIO_WLAN_BT_EN,
						S3C_GPIO_PULL_NONE);
		}
	}
	return 0;
}

static int wlan_reset_en(int onoff)
{
	gpio_set_value(GPIO_WLAN_nRST,
			onoff ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
	return 0;
}

static int wlan_carddetect_en(int onoff)
{
	u32 i;
	u32 sdio;

	if (onoff) {
		for (i = 0; i < ARRAY_SIZE(wlan_sdio_on_table); i++) {
			sdio = wlan_sdio_on_table[i][0];
			s3c_gpio_cfgpin(sdio,
					S3C_GPIO_SFN(wlan_sdio_on_table[i][1]));
			s3c_gpio_setpull(sdio, wlan_sdio_on_table[i][3]);
			if (wlan_sdio_on_table[i][2] != GPIO_LEVEL_NONE)
				gpio_set_value(sdio, wlan_sdio_on_table[i][2]);
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(wlan_sdio_off_table); i++) {
			sdio = wlan_sdio_off_table[i][0];
			s3c_gpio_cfgpin(sdio,
				S3C_GPIO_SFN(wlan_sdio_off_table[i][1]));
			s3c_gpio_setpull(sdio, wlan_sdio_off_table[i][3]);
			if (wlan_sdio_off_table[i][2] != GPIO_LEVEL_NONE)
				gpio_set_value(sdio, wlan_sdio_off_table[i][2]);
		}
	}
	udelay(5);

	sdhci_s3c_force_presence_change(&s3c_device_hsmmc3);
	return 0;
}
#endif
static struct resource wifi_resources[] = {
	[0] = {
		.name	= "bcm4329_wlan_irq",
		.start	= IRQ_EINT(20),
		.end	= IRQ_EINT(20),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct wifi_mem_prealloc wifi_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

static void *crespo_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;

	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wifi_mem_array[section].size < size)
		return NULL;

	return wifi_mem_array[section].mem_ptr;
}

int __init crespo_init_wifi_mem(void)
{
	int i;
	int j;

	for (i = 0 ; i < WLAN_SKB_BUF_NUM ; i++) {
		wlan_static_skb[i] = dev_alloc_skb(
				((i < (WLAN_SKB_BUF_NUM / 2)) ? 4096 : 8192));

		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wifi_mem_array[i].mem_ptr =
				kmalloc(wifi_mem_array[i].size, GFP_KERNEL);

		if (!wifi_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wifi_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}

#if 0 // for wifi
static struct wifi_platform_data wifi_pdata = {
	.set_power		= wlan_power_en,
	.set_reset		= wlan_reset_en,
	.set_carddetect		= wlan_carddetect_en,
	.mem_prealloc		= crespo_mem_prealloc,
};

static struct platform_device sec_device_wifi = {
	.name			= "bcm4329_wlan",
	.id			= 1,
	.num_resources		= ARRAY_SIZE(wifi_resources),
	.resource		= wifi_resources,
	.dev			= {
		.platform_data = &wifi_pdata,
	},
};
#endif

static struct platform_device watchdog_device = {
	.name = "watchdog",
	.id = -1,
};


#if defined(CONFIG_KEYBOARD_P1)
static struct platform_device p1_keyboard = {
        .name  = "p1_keyboard",
        .id    = -1,
};
#endif

static struct platform_device *crespo_devices[] __initdata = {
	&watchdog_device,
#ifdef CONFIG_FIQ_DEBUGGER
	&s5pv210_device_fiqdbg_uart2,
#endif
	&s5pc110_device_onenand,
#ifdef CONFIG_RTC_DRV_S3C
	&s5p_device_rtc,
#endif

	&s5pv210_device_iis0,
	&s3c_device_wdt,

#ifdef CONFIG_FB_S3C
	&s3c_device_fb,
#endif

#if defined(CONFIG_KEYBOARD_P1)
	&p1_keyboard,
#endif

#ifdef CONFIG_VIDEO_MFC50
	&s3c_device_mfc,
#endif
#ifdef	CONFIG_S5P_ADC
	&s3c_device_adc,
#endif
#ifdef CONFIG_VIDEO_FIMC
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
#endif

#ifdef CONFIG_VIDEO_JPEG_V2
	&s3c_device_jpeg,
#endif

	&s3c_device_g3d,
	&s3c_device_lcd,

#ifdef CONFIG_FB_S3C_TL2796
	&s3c_device_spi_gpio,
#endif
#if defined(CONFIG_SAMSUNG_JACK) || defined(CONFIG_SEC_HEADSET)
	&sec_device_jack,
#endif
	&s3c_device_i2c0,
#if defined(CONFIG_S3C_DEV_I2C1)
	&s3c_device_i2c1,
#endif

#if defined(CONFIG_S3C_DEV_I2C2)
	&s3c_device_i2c2,
#endif
	&s3c_device_i2c4,
	&s3c_device_i2c5,  /* accel sensor & gyro sensor*/
	&s3c_device_i2c6,
	&s3c_device_i2c7,
	&s3c_device_i2c8,  
	&s3c_device_i2c9,  /* max1704x:fuel_guage */
	&s3c_device_i2c11,  /* smb136:charger-ic */
//	&s3c_device_i2c12, 
	&s3c_device_i2c13, /*cmc623 mdnie */
#if defined(CONFIG_PN544)	
	&s3c_device_i2c14, /* nfc sensor */
#endif
#if defined(CONFIG_MACH_P1_LTN) && defined(CONFIG_VIDEO_NM6XX)
	&s3c_device_i2c15, /* nmi625  */
#endif	
	&sec_device_switch,  // samsung switch driver

#ifdef CONFIG_USB_GADGET
	&s3c_device_usbgadget,
#endif
#ifdef CONFIG_USB_ANDROID
	&s3c_device_android_usb,
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	&s3c_device_usb_mass_storage,
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	&s3c_device_rndis,
#endif
#endif

#ifdef CONFIG_S3C_DEV_HSMMC
	&s3c_device_hsmmc0,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	&s3c_device_hsmmc1,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	&s3c_device_hsmmc3,
#endif

#if defined(CONFIG_FB_S3C_CMC623)
	&sec_device_tune_cmc623,
#endif

	&sec_device_battery,
	&s3c_device_i2c10, /* magnetic sensor & lightsensor */

#ifdef CONFIG_S5PV210_POWER_DOMAIN
	&s5pv210_pd_audio,
	&s5pv210_pd_cam,
	&s5pv210_pd_tv,
	&s5pv210_pd_lcd,
	&s5pv210_pd_g3d,
	&s5pv210_pd_mfc,
#endif

#ifdef CONFIG_ANDROID_PMEM
	&pmem_device,
	&pmem_gpu1_device,
	&pmem_adsp_device,
#endif

#ifdef CONFIG_HAVE_PWM
	&s3c_device_timer[0],
	&s3c_device_timer[1],
	&s3c_device_timer[2],
	&s3c_device_timer[3],
#endif
	&sec_device_rfkill,
	&sec_device_btsleep,
	&ram_console_device,
#if 0
	&sec_device_wifi,
#endif
#if defined(CONFIG_KEYBOARD_GPIO)
    &gpio_keys_device,
#else
#if defined(CONFIG_INPUT_GPIO)
    &p1_input_device,
#endif
#endif

#ifdef CONFIG_SND_S5P_RP
	&s5p_device_rp,
#endif
#if defined(CONFIG_VIDEO_TSI)
	&s3c_device_tsi,
#endif	
#if defined(CONFIG_MACH_P1_CDMA)
	&sec_device_dpram,
#endif
};

unsigned int HWREV;
EXPORT_SYMBOL(HWREV);

static void __init p1_map_io(void)
{
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s5pv210_gpiolib_init();
	s3c24xx_init_uarts(crespo_uartcfgs, ARRAY_SIZE(crespo_uartcfgs));
	s5p_reserve_bootmem(crespo_media_devs, ARRAY_SIZE(crespo_media_devs));
#ifdef CONFIG_MTD_ONENAND
	s5pc110_device_onenand.name = "s5pc110-onenand";
#endif
}

unsigned int pm_debug_scratchpad;

static unsigned int ram_console_start;
static unsigned int ram_console_size;

static void __init p1_fixup(struct machine_desc *desc,
		struct tag *tags, char **cmdline,
		struct meminfo *mi)
{
	mi->bank[0].start = 0x30000000;
	mi->bank[0].size = 80 * SZ_1M;
	mi->bank[0].node = 0;

	mi->bank[1].start = 0x40000000;
	mi->bank[1].size = 256 * SZ_1M;
	mi->bank[1].node = 1;

	mi->bank[2].start = 0x50000000;
	/* 1M for ram_console buffer */
	mi->bank[2].size = 255 * SZ_1M;
	mi->bank[2].node = 2;
	mi->nr_banks = 3;

	ram_console_start = mi->bank[2].start + mi->bank[2].size;
	ram_console_size = SZ_1M - SZ_4K;

	pm_debug_scratchpad = ram_console_start + ram_console_size;
}

/* this function are used to detect s5pc110 chip version temporally */
int s5pc110_version ;

void _hw_version_check(void)
{
	void __iomem *phy_address ;
	int temp;

	phy_address = ioremap(0x40, 1);

	temp = __raw_readl(phy_address);

	if (temp == 0xE59F010C)
		s5pc110_version = 0;
	else
		s5pc110_version = 1;

	printk(KERN_INFO "S5PC110 Hardware version : EVT%d\n",
				s5pc110_version);

	iounmap(phy_address);
}

/*
 * Temporally used
 * return value 0 -> EVT 0
 * value 1 -> evt 1
 */

int hw_version_check(void)
{
	return s5pc110_version ;
}
EXPORT_SYMBOL(hw_version_check);

static void herring_init_gpio(void)
{
	s3c_config_gpio_table();
	s3c_config_sleep_gpio_table(ARRAY_SIZE(herring_sleep_gpio_table),
			herring_sleep_gpio_table);
}

static void __init fsa9480_gpio_init(void)
{
	s3c_gpio_cfgpin(GPIO_USB_SEL, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_USB_SEL, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_UART_SEL, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_UART_SEL, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(GPIO_JACK_nINT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_JACK_nINT, S3C_GPIO_PULL_NONE);
}

static void __init setup_ram_console_mem(void)
{
	ram_console_resource[0].start = ram_console_start;
	ram_console_resource[0].end = ram_console_start + ram_console_size - 1;
}

static unsigned int p1_get_hwrev(void)
{
	unsigned int model_rev = 0;
	unsigned int hw_rev = 0;
	unsigned char model_str[12];
	
	// Read HWREV_MODE gpio status
	s3c_gpio_cfgpin(GPIO_HWREV_MODE0, S3C_GPIO_INPUT);
	s3c_gpio_setpull( GPIO_HWREV_MODE0, S3C_GPIO_PULL_NONE); 
	s3c_gpio_cfgpin(GPIO_HWREV_MODE1, S3C_GPIO_INPUT);
	s3c_gpio_setpull( GPIO_HWREV_MODE1, S3C_GPIO_PULL_NONE);  
	s3c_gpio_cfgpin(GPIO_HWREV_MODE2, S3C_GPIO_INPUT);
	s3c_gpio_setpull( GPIO_HWREV_MODE2, S3C_GPIO_PULL_NONE); 
	s3c_gpio_cfgpin(GPIO_HWREV_MODE3, S3C_GPIO_INPUT);
	s3c_gpio_setpull( GPIO_HWREV_MODE3, S3C_GPIO_PULL_NONE); 
	s3c_gpio_cfgpin(GPIO_HWREV_MODE4, S3C_GPIO_INPUT);
	s3c_gpio_setpull( GPIO_HWREV_MODE4, S3C_GPIO_PULL_NONE); 
	s3c_gpio_cfgpin(GPIO_HWREV_MODE5, S3C_GPIO_INPUT);
	s3c_gpio_setpull( GPIO_HWREV_MODE5, S3C_GPIO_PULL_NONE); 

	hw_rev = gpio_get_value(GPIO_HWREV_MODE0);
	hw_rev = hw_rev | (gpio_get_value(GPIO_HWREV_MODE1) <<1);
	hw_rev = hw_rev | (gpio_get_value(GPIO_HWREV_MODE2) <<2);
	hw_rev = hw_rev | (gpio_get_value(GPIO_HWREV_MODE3) <<3);

	model_rev = (gpio_get_value(GPIO_HWREV_MODE4) << 1) | gpio_get_value(GPIO_HWREV_MODE5);
	switch(model_rev)
	{
		case 0 :
			sprintf(model_str, "P1_AMOLED");
			break;
		case 1:
			sprintf(model_str, "P2");
			break;
		case 2:
#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
			sprintf(model_str, "GT-P1000");
#elif defined(CONFIG_MACH_P1_CDMA)
			sprintf(model_str, "SPH-P100");
#endif
			break;
		case 3:
			sprintf(model_str, "P1");
			break;
		default:
			sprintf(model_str, "Unknown");
			break;
	}

	if(model_rev == 0x2)  // GT-P1000
		hw_rev += 6;  // Rev0.6
	
	printk(KERN_NOTICE "%s: HWREV (0x%x), Model (%s)\n", __func__, hw_rev, model_str);
	
	return hw_rev;
}

static void __init p1_machine_init(void)
{
	setup_ram_console_mem();
	s3c_usb_set_serial();
	platform_add_devices(crespo_devices, ARRAY_SIZE(crespo_devices));

	/* Find out S5PC110 chip version */
	_hw_version_check();

	pm_power_off = p1_power_off ;

	HWREV = p1_get_hwrev();
	printk(KERN_INFO "HWREV is 0x%x\n", HWREV);
	
	if(HWREV < P1_HWREV_REV06)
	{
		printk(KERN_ERR "This board is not supported: HWREV=0x%x\n", HWREV);
	}
	
	/*initialise the gpio's*/
	herring_init_gpio();

#ifdef CONFIG_ANDROID_PMEM
	android_pmem_set_platdata();
#endif
#ifdef CONFIG_SAMSUNG_JACK
	/* headset/earjack detection */
	if (system_rev >= 0x09)
		gpio_request(GPIO_EAR_MICBIAS_EN, "ear_micbias_enable");
#endif

	gpio_request(GPIO_TOUCH_EN, "touch en");

	/* i2c */
	s3c_i2c0_set_platdata(NULL);
#ifdef CONFIG_S3C_DEV_I2C1
	s3c_i2c1_set_platdata(NULL);
#endif

#ifdef CONFIG_S3C_DEV_I2C2
	s3c_i2c2_set_platdata(NULL);
#endif

	l3g4200d_irq_init();

	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));
	i2c_register_board_info(2, i2c_devs2, ARRAY_SIZE(i2c_devs2));
	i2c_register_board_info(4, i2c_devs4, ARRAY_SIZE(i2c_devs4));

	/* accel and gyro sensor */
	if(HWREV <  P1_HWREV_REV09)
		i2c_register_board_info(5, i2c_devs5, ARRAY_SIZE(i2c_devs5));
	else
	{
		i2c_devs5[ARRAY_SIZE(i2c_devs5)-1].addr += 1;						// From HW rev 0.9, slave addres is changed from 0x68 to 0x69
		i2c_register_board_info(5, i2c_devs5, ARRAY_SIZE(i2c_devs5));
	}
	/* magnetic and light sensor */
	i2c_register_board_info(10, i2c_devs10, ARRAY_SIZE(i2c_devs10));

	i2c_register_board_info(6, i2c_devs6, ARRAY_SIZE(i2c_devs6));
	/* FSA9480 */
	fsa9480_gpio_init();
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));

	/* max17042 */
	max17042_gpio_init();
	i2c_register_board_info(9, i2c_devs9, ARRAY_SIZE(i2c_devs9));

	/* smb136 */
	smb136_gpio_init();
	i2c_register_board_info(11, i2c_devs11, ARRAY_SIZE(i2c_devs11));
	
	/* cmc623 */
	i2c_register_board_info(13, i2c_devs13, ARRAY_SIZE(i2c_devs13));

#if defined(CONFIG_PN544)
	/* nfc sensor */
	//i2c_register_board_info(14, i2c_devs14, ARRAY_SIZE(i2c_devs14));
#endif
#if defined(CONFIG_MACH_P1_LTN) && defined(CONFIG_VIDEO_NM6XX)
	i2c_register_board_info(15, i2c_devs15, ARRAY_SIZE(i2c_devs15));
#endif

#ifdef CONFIG_FB_S3C_LVDS
#if defined(CONFIG_FB_S3C_CMC623)
		platform_device_register(&cmc623_pwm_backlight);
#endif
		platform_device_register(&sec_device_lms700);
		s3cfb_set_platdata(&lvds_data);
#endif

#ifdef CONFIG_FB_S3C_TL2796
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
	s3cfb_set_platdata(&tl2796_data);
#endif

#if defined(CONFIG_S5P_ADC)
	s3c_adc_set_platdata(&s3c_adc_platform);
#endif

#if defined(CONFIG_PM)
	s3c_pm_init();
#endif

#ifdef CONFIG_VIDEO_FIMC
	/* fimc */
	s3c_fimc0_set_platdata(&fimc_plat_lsi);
	s3c_fimc1_set_platdata(&fimc_plat_lsi);
	s3c_fimc2_set_platdata(&fimc_plat_lsi);
#endif

#ifdef CONFIG_VIDEO_JPEG_V2
	s3c_jpeg_set_platdata(&jpeg_plat);
#endif

#ifdef CONFIG_VIDEO_MFC50
	/* mfc */
	s3c_mfc_set_platdata(NULL);
#endif

#ifdef CONFIG_S3C_DEV_HSMMC
	s5pv210_default_sdhci0();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	s5pv210_default_sdhci1();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s5pv210_default_sdhci2();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s5pv210_default_sdhci3();
#endif
#ifdef CONFIG_S5PV210_SETUP_SDHCI
	s3c_sdhci_set_platdata();
#endif

	regulator_has_full_constraints();

	register_reboot_notifier(&crespo_reboot_notifier);

	herring_switch_init();

#if defined(CONFIG_MACH_P1_GSM) || defined(CONFIG_MACH_P1_LTN)
	gps_gpio_init();
#endif

	uart_switch_init();

	crespo_init_wifi_mem();
	
	qt_touch_init();

#if defined(CONFIG_MACH_P1_LTN)
#if defined(CONFIG_VIDEO_NM6XX)
	nmi_i2s_cfg_gpio_init();
#else
	nmi_pwr_disable();  // Disable the ISDBT PWR : Only Latin HW 0.3
#endif
#endif

#ifdef CONFIG_VIDEO_TV20
	platform_device_register(&s5p_device_tvout);
	platform_device_register(&s5p_device_cec);
	platform_device_register(&s5p_device_hpd);
#endif
#ifdef CONFIG_30PIN_CONN
	platform_device_register(&sec_device_connector);
#endif
}

#ifdef CONFIG_USB_SUPPORT
/* Initializes OTG Phy. */
void otg_phy_init(void)
{
	/* USB PHY0 Enable */
	writel(readl(S5P_USB_PHY_CONTROL) | (0x1<<0),
			S5P_USB_PHY_CONTROL);
	writel((readl(S3C_USBOTG_PHYPWR) & ~(0x3<<3) & ~(0x1<<0)) | (0x1<<5),
			S3C_USBOTG_PHYPWR);
	writel((readl(S3C_USBOTG_PHYCLK) & ~(0x5<<2)) | (0x3<<0),
			S3C_USBOTG_PHYCLK);
	writel((readl(S3C_USBOTG_RSTCON) & ~(0x3<<1)) | (0x1<<0),
			S3C_USBOTG_RSTCON);
	msleep(1);
	writel(readl(S3C_USBOTG_RSTCON) & ~(0x7<<0),
			S3C_USBOTG_RSTCON);
	msleep(1);

	/* rising/falling time */
	writel(readl(S3C_USBOTG_PHYTUNE) | (0x1<<20),
			S3C_USBOTG_PHYTUNE);

	/* set DC level as 6 (6%) */
	writel((readl(S3C_USBOTG_PHYTUNE) & ~(0xf)) | (0x1<<2) | (0x1<<1),
			S3C_USBOTG_PHYTUNE);
}
EXPORT_SYMBOL(otg_phy_init);

/* USB Control request data struct must be located here for DMA transfer */
struct usb_ctrlrequest usb_ctrl __attribute__((aligned(64)));

/* OTG PHY Power Off */
void otg_phy_off(void)
{
	writel(readl(S3C_USBOTG_PHYPWR) | (0x3<<3),
			S3C_USBOTG_PHYPWR);
	writel(readl(S5P_USB_PHY_CONTROL) & ~(1<<0),
			S5P_USB_PHY_CONTROL);
}
EXPORT_SYMBOL(otg_phy_off);

void usb_host_phy_init(void)
{
	struct clk *otg_clk;

	otg_clk = clk_get(NULL, "otg");
	clk_enable(otg_clk);

	if (readl(S5P_USB_PHY_CONTROL) & (0x1<<1))
		return;

	__raw_writel(__raw_readl(S5P_USB_PHY_CONTROL) | (0x1<<1),
			S5P_USB_PHY_CONTROL);
	__raw_writel((__raw_readl(S3C_USBOTG_PHYPWR)
			& ~(0x1<<7) & ~(0x1<<6)) | (0x1<<8) | (0x1<<5),
			S3C_USBOTG_PHYPWR);
	__raw_writel((__raw_readl(S3C_USBOTG_PHYCLK) & ~(0x1<<7)) | (0x3<<0),
			S3C_USBOTG_PHYCLK);
	__raw_writel((__raw_readl(S3C_USBOTG_RSTCON)) | (0x1<<4) | (0x1<<3),
			S3C_USBOTG_RSTCON);
	__raw_writel(__raw_readl(S3C_USBOTG_RSTCON) & ~(0x1<<4) & ~(0x1<<3),
			S3C_USBOTG_RSTCON);
}
EXPORT_SYMBOL(usb_host_phy_init);

void usb_host_phy_off(void)
{
	__raw_writel(__raw_readl(S3C_USBOTG_PHYPWR) | (0x1<<7)|(0x1<<6),
			S3C_USBOTG_PHYPWR);
	__raw_writel(__raw_readl(S5P_USB_PHY_CONTROL) & ~(1<<1),
			S5P_USB_PHY_CONTROL);
}
EXPORT_SYMBOL(usb_host_phy_off);
#endif

MACHINE_START(SMDKC110, "SMDKC110")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.fixup		= p1_fixup,
	.init_irq	= s5pv210_init_irq,
	.map_io		= p1_map_io,
	.init_machine	= p1_machine_init,
#if	defined(CONFIG_S5P_HIGH_RES_TIMERS)
	.timer		= &s5p_systimer,
#else
	.timer		= &s3c24xx_timer,
#endif
MACHINE_END

#if defined(CONFIG_MACH_P1_GSM)
MACHINE_START(P1, "GT-P1000")
#elif defined(CONFIG_MACH_P1_LTN)
#if defined(CONFIG_VIDEO_NM6XX)
MACHINE_START(P1, "GT-P1000")
#else
MACHINE_START(P1, "GT-P1000")
#endif
#elif defined(CONFIG_MACH_P1_CDMA)
MACHINE_START(P1, "SPH-P100")
#endif
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.fixup		= p1_fixup,
	.init_irq	= s5pv210_init_irq,
	.map_io		= p1_map_io,
	.init_machine	= p1_machine_init,
	.timer		= &s5p_systimer,
MACHINE_END

void s3c_setup_uart_cfg_gpio(unsigned char port)
{
	switch (port) {
	case 0:
		s3c_gpio_cfgpin(GPIO_BT_RXD, S3C_GPIO_SFN(GPIO_BT_RXD_AF));
		s3c_gpio_setpull(GPIO_BT_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_BT_TXD, S3C_GPIO_SFN(GPIO_BT_TXD_AF));
		s3c_gpio_setpull(GPIO_BT_TXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_BT_CTS, S3C_GPIO_SFN(GPIO_BT_CTS_AF));
		s3c_gpio_setpull(GPIO_BT_CTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_BT_RTS, S3C_GPIO_SFN(GPIO_BT_RTS_AF));
		s3c_gpio_setpull(GPIO_BT_RTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_RXD, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_TXD, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_TXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_CTS, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_CTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_RTS, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_RTS, S3C_GPIO_PULL_NONE);
		break;
	case 1:
		s3c_gpio_cfgpin(GPIO_GPS_RXD, S3C_GPIO_SFN(GPIO_GPS_RXD_AF));
		s3c_gpio_setpull(GPIO_GPS_RXD, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(GPIO_GPS_TXD, S3C_GPIO_SFN(GPIO_GPS_TXD_AF));
		s3c_gpio_setpull(GPIO_GPS_TXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_GPS_CTS, S3C_GPIO_SFN(GPIO_GPS_CTS_AF));
		s3c_gpio_setpull(GPIO_GPS_CTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_GPS_RTS, S3C_GPIO_SFN(GPIO_GPS_RTS_AF));
		s3c_gpio_setpull(GPIO_GPS_RTS, S3C_GPIO_PULL_NONE);
		break;
	case 2:
		s3c_gpio_cfgpin(GPIO_AP_RXD, S3C_GPIO_SFN(GPIO_AP_RXD_AF));
		s3c_gpio_setpull(GPIO_AP_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_AP_TXD, S3C_GPIO_SFN(GPIO_AP_TXD_AF));
		s3c_gpio_setpull(GPIO_AP_TXD, S3C_GPIO_PULL_NONE);
		break;
	case 3:
		s3c_gpio_cfgpin(GPIO_FLM_RXD, S3C_GPIO_SFN(GPIO_FLM_RXD_AF));
		s3c_gpio_setpull(GPIO_FLM_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_FLM_TXD, S3C_GPIO_SFN(GPIO_FLM_TXD_AF));
		s3c_gpio_setpull(GPIO_FLM_TXD, S3C_GPIO_PULL_NONE);
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(s3c_setup_uart_cfg_gpio);

#if 1
void s3c_config_gpio_alive_table(int array_size, unsigned int (*gpio_table)[4])
{
	u32 i, gpio;

	for (i = 0; i < array_size; i++) {
		gpio = gpio_table[i][0];
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(gpio_table[i][1]));
		s3c_gpio_setpull(gpio, gpio_table[i][3]);
		if (gpio_table[i][2] != GPIO_LEVEL_NONE)
			gpio_set_value(gpio, gpio_table[i][2]);
	}
}



static unsigned int wlan_gpio_table[][4] = {	
	{GPIO_WLAN_nRST, GPIO_WLAN_nRST_AF, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_HOST_WAKE, GPIO_WLAN_HOST_WAKE_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_DOWN},
	{GPIO_WLAN_WAKE, GPIO_WLAN_WAKE_AF, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
};

static unsigned int wlan_sdio_on_table[][4] = {
        {GPIO_WLAN_SDIO_CLK, GPIO_WLAN_SDIO_CLK_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {GPIO_WLAN_SDIO_CMD, GPIO_WLAN_SDIO_CMD_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {GPIO_WLAN_SDIO_D0, GPIO_WLAN_SDIO_D0_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {GPIO_WLAN_SDIO_D1, GPIO_WLAN_SDIO_D1_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {GPIO_WLAN_SDIO_D2, GPIO_WLAN_SDIO_D2_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {GPIO_WLAN_SDIO_D3, GPIO_WLAN_SDIO_D3_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
};

static unsigned int wlan_sdio_off_table[][4] = {
        {GPIO_WLAN_SDIO_CLK, 1, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
        {GPIO_WLAN_SDIO_CMD, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {GPIO_WLAN_SDIO_D0, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {GPIO_WLAN_SDIO_D1, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {GPIO_WLAN_SDIO_D2, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {GPIO_WLAN_SDIO_D3, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
};

void wlan_setup_power(int on, int flag)
{
	printk(/*KERN_INFO*/ "%s %s", __func__, on ? "on" : "down");
	if (flag != 1) {	
		printk(/*KERN_DEBUG*/ " --reset(flag=%d)\n", flag);
		if (on)
			gpio_set_value(GPIO_WLAN_nRST, GPIO_LEVEL_HIGH);
		else
			gpio_set_value(GPIO_WLAN_nRST, GPIO_LEVEL_LOW);			
		return;
	}	
	printk(/*KERN_INFO*/ " --enter\n");
		
	if (on) {		
		s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_gpio_table), wlan_gpio_table);
		s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_sdio_on_table), wlan_sdio_on_table);
		
		/* PROTECT this check under spinlock.. No other thread should be touching
		 * GPIO_BT_REG_ON at this time.. If BT is operational, don't touch it. */
//		spin_lock_irqsave(&wlan_reglock, wlan_reglock_flags);	
		
		s3c_gpio_cfgpin(GPIO_WLAN_BT_EN, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_WLAN_BT_EN, S3C_GPIO_PULL_NONE);

		gpio_set_value(GPIO_WLAN_BT_EN, GPIO_LEVEL_HIGH);
		s3c_gpio_slp_cfgpin(GPIO_WLAN_BT_EN, S3C_GPIO_SLP_OUT1);

		msleep(100);

		gpio_set_value(GPIO_WLAN_nRST, GPIO_LEVEL_HIGH);
		s3c_gpio_slp_cfgpin(GPIO_WLAN_nRST, S3C_GPIO_SLP_OUT1);
		
		printk(KERN_DEBUG "WLAN: GPIO_WLAN_BT_EN = %d, GPIO_WLAN_nRST = %d\n", 
			   gpio_get_value(GPIO_WLAN_BT_EN), gpio_get_value(GPIO_WLAN_nRST));
		
//		spin_unlock_irqrestore(&wlan_reglock, wlan_reglock_flags);
	}
	else {
		/* PROTECT this check under spinlock.. No other thread should be touching
		 * GPIO_BT_REG_ON at this time.. If BT is operational, don't touch it. */
//		spin_lock_irqsave(&wlan_reglock, wlan_reglock_flags);	
		/* need delay between v_bat & reg_on for 2 cycle @ 38.4MHz */
		udelay(5);
		
		if (gpio_get_value(GPIO_BT_nRST) == 0) {
			gpio_set_value(GPIO_WLAN_BT_EN, GPIO_LEVEL_LOW);	
			s3c_gpio_slp_cfgpin(GPIO_WLAN_BT_EN, S3C_GPIO_SLP_OUT0);
		}
		
		gpio_set_value(GPIO_WLAN_nRST, GPIO_LEVEL_LOW);
		s3c_gpio_slp_cfgpin(GPIO_WLAN_nRST, S3C_GPIO_SLP_OUT0);
		
		printk(KERN_DEBUG "WLAN: GPIO_WLAN_BT_EN = %d, GPIO_WLAN_nRST = %d\n", 
			   gpio_get_value(GPIO_WLAN_BT_EN), gpio_get_value(GPIO_WLAN_nRST));
		
//		spin_unlock_irqrestore(&wlan_reglock, wlan_reglock_flags);

		
		s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_sdio_off_table), wlan_sdio_off_table);	
	}
	msleep(100);

	/* mmc_rescan*/
	sdhci_s3c_force_presence_change(&s3c_device_hsmmc3);

}
EXPORT_SYMBOL(wlan_setup_power);
#endif
