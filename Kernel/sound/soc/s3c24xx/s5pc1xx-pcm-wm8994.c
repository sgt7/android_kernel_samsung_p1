/*
 * s5pc110_wm8994.c
 *
 * Copyright (C) 2010, Samsung Elect. Ltd. - SLSI
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/map-base.h>
#include <mach/regs-clock.h>
#include <linux/delay.h>

#include "../codecs/wm8994.h"
#include "s3c-dma.h"
#include "s3c-pcmdev.h"

#define PLAY_51       0
#define PLAY_STEREO   1
#define PLAY_OFF      2

#define REC_MIC    0
#define REC_LINE   1
#define REC_OFF    2

#define SRC_CLK	s3c_i2s_get_clockrate()


/* #define CONFIG_SND_DEBUG */
#ifdef CONFIG_SND_DEBUG
#define debug_msg(x...) printk(x)
#else
#define debug_msg(x...)
#endif

static int smdkc110_set_gpio(int id)
{
	switch (id) {

	case 0:
		s3c_gpio_cfgpin(S5PV210_GPC1(0), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S5PV210_GPC1(1), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S5PV210_GPC1(2), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S5PV210_GPC1(3), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S5PV210_GPC1(4), S3C_GPIO_SFN(3));

		s3c_gpio_setpull(S5PV210_GPC1(0), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S5PV210_GPC1(1), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S5PV210_GPC1(2), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S5PV210_GPC1(3), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S5PV210_GPC1(4), S3C_GPIO_PULL_NONE);

		break;
	case 1:
		s3c_gpio_cfgpin(S5PV210_GPC0(0), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S5PV210_GPC0(1), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S5PV210_GPC0(2), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S5PV210_GPC0(3), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S5PV210_GPC0(4), S3C_GPIO_SFN(3));

		s3c_gpio_setpull(S5PV210_GPC0(0), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S5PV210_GPC0(1), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S5PV210_GPC0(2), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S5PV210_GPC0(3), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S5PV210_GPC0(4), S3C_GPIO_PULL_NONE);
		break;
	case 2:
		s3c_gpio_cfgpin(S5PV210_GPI(0), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S5PV210_GPI(1), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S5PV210_GPI(2), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S5PV210_GPI(3), S3C_GPIO_SFN(3));
		s3c_gpio_cfgpin(S5PV210_GPI(4), S3C_GPIO_SFN(3));

		s3c_gpio_setpull(S5PV210_GPI(0), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S5PV210_GPI(1), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S5PV210_GPI(2), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S5PV210_GPI(3), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(S5PV210_GPI(4), S3C_GPIO_PULL_NONE);

		break;
	default:
		debug_msg("Not a valid PCM IP Number. - %d\n", id);
		return -EINVAL;
	}
	return 0;
}

static int smdkc110_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int  ret;

	debug_msg("+++++++++%s\n", __func__);
	/*TODO Check below function in detail    */

	/* Set the AP DAI pcm mode configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A);
	if (ret < 0)
		return ret;

	/* Clock Settings */
	/*clock settings need not to be done here.It will be done in
	SOC pcm driver file.*/
	/* Mask output clock of AUDIO1*/
	writel(readl(S5P_CLK_SRC_MASK0) | S5P_CLKSRC_MASK0_AUDIO1,
						S5P_CLK_SRC_MASK0);



	debug_msg("----------%s\n", __func__);
	return 0;
}

/*
 * WM8580 DAI operations.
 */
static struct snd_soc_ops smdkc110_ops = {
	.hw_params = smdkc110_hw_params,
};


static int smdkc110_wm8994_init(struct snd_soc_codec *codec)
{
	debug_msg("---------%s\n", __func__);
	/*Nothing to configure for codec here.Codec control configuration
	will be done in codec driver.*/
	return 0;
}

static struct snd_soc_dai_link smdkc110_dai[] = {
{
	.name = "PCM-C110",
	.stream_name = "PCM FOR VT",
	.cpu_dai = &s3c_pcmdev_dai,
	.codec_dai = &wm8994_pcm_dai,
	.init = smdkc110_wm8994_init,
	.ops = &smdkc110_ops,
},
};

static struct snd_soc_card smdkc110 = {
	.name = "smdkc110-pcm",
	.platform = &s3c24xx_pcm_soc_platform,
	.dai_link = smdkc110_dai,
	.num_links = ARRAY_SIZE(smdkc110_dai),
};

static struct wm8994_setup_data smdkc110_wm8994_setup = {
	.i2c_address = 0x34>>1,
	.i2c_bus = 4,
};


static struct snd_soc_device smdkc110_snd_devdata = {
	.card = &smdkc110,
	.codec_dev = &soc_codec_dev_pcm_wm8994,
	.codec_data = &smdkc110_wm8994_setup,
};

static struct platform_device *smdkc110_snd_device;
static int __init smdkc110_audio_init(void)
{
	int ret;

	debug_msg("+++++++%s\n", __func__);

	if (smdkc110_set_gpio(PCM_ID)) {
		debug_msg("Failed to set GPIO in %s @%d\n",
				 __func__, __LINE__);
		 return -EINVAL;
	}

	smdkc110_snd_device = platform_device_alloc("soc-audio", 1);
	if (!smdkc110_snd_device) {
		debug_msg("Failed to alloc platform device in %s @%d\n",
				 __func__, __LINE__);
		return -ENOMEM;
	}

	platform_set_drvdata(smdkc110_snd_device, &smdkc110_snd_devdata);
	smdkc110_snd_devdata.dev = &smdkc110_snd_device->dev;
	ret = platform_device_add(smdkc110_snd_device);

	if (ret) {
		debug_msg("###going to put platform device ..%s\n", __func__);
		platform_device_put(smdkc110_snd_device);
	}

	debug_msg("----------%s\n", __func__);

	return ret;
}


static void __exit smdkc110_audio_exit(void)
{
	debug_msg("%s\n", __func__);

	platform_device_unregister(smdkc110_snd_device);
}

module_init(smdkc110_audio_init);
module_exit(smdkc110_audio_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC SMDKC110 ");
MODULE_LICENSE("GPL");
