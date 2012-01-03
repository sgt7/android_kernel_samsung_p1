/*
 * s5pc1xx_wm8994.c
 *
 * Copyright (C) 2010, Samsung Elect. Ltd. - 
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <mach/regs-clock.h>
#include "../codecs/wm8994.h"
#include "s3c-dma.h"
#include "s5pc1xx-i2s2.h"
#include "../codecs/nmi625-snd.h"

#include <linux/io.h>

#define I2S_NUM 0

extern struct snd_soc_dai i2s_sec_fifo_dai;
extern struct snd_soc_dai i2s2_dai;
extern struct snd_soc_platform s5pc110_soc_platform;
extern const struct snd_kcontrol_new s5p_idma_control;

//static int set_epll_rate(unsigned long rate); //not-used


#define SRC_CLK	(i2s->clk_rate) 

#define CONFIG_SND_DEBUG
#ifdef CONFIG_SND_DEBUG
#define debug_msg(x...) printk(x)
#else
#define debug_msg(x...)
#endif

static int lowpower = 0;

/*  BLC(bits-per-channel) --> BFS(bit clock shud be >= FS*(Bit-per-channel)*2)  */
/*  BFS --> RFS(must be a multiple of BFS)                                  */
/*  RFS & SRC_CLK --> Prescalar Value(SRC_CLK / RFS_VAL / fs - 1)           */
static int smdkc110_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int bfs, rfs, ret;
	u32 ap_codec_clk;
	int psr;

	debug_msg("%s\n", __FUNCTION__);

	/* Choose BFS and RFS values combination that is supported by
	 * both the WM8994 codec as well as the S5P AP
	 *
	 */
	switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S8:
			bfs = 16;
			rfs = 256;		/* Can take any RFS value for AP */
			break;
		case SNDRV_PCM_FORMAT_S16_LE:
			bfs = 32;
			rfs = 256;		/* Can take any RFS value for AP */
			break;
		case SNDRV_PCM_FORMAT_S20_3LE:
		case SNDRV_PCM_FORMAT_S24_LE:
			bfs = 48;
			rfs = 512;		/* B'coz 48-BFS needs atleast 512-RFS acc to *S5P6440* UserManual */
			break;
		case SNDRV_PCM_FORMAT_S32_LE:	/* Impossible, as the AP doesn't support 64fs or more BFS */
		default:
			return -EINVAL;
	}


#if 1	//namkh, test
#if 1
	/*
	 * MSB (left) Justified
	 */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_LEFT_J);
#else
	/*
	 * IIS-bus Format
	 */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
#endif
#else
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
#endif

	if (ret < 0)

		return ret;

#if 0 //used for AP master clock setting
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C_CDCLKSRC_INT, params_rate(params), SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C_CLKSRC_CLKAUDIO, params_rate(params), SND_SOC_CLOCK_OUT);

	if (ret < 0)
	{
		printk("smdkc110_atv_hw_params : AP sys clock setting error!\n");
		return ret;
	}
#endif

	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C_CDCLKSRC_EXT, params_rate(params), SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C_CLKSRC_I2SEXT, params_rate(params), SND_SOC_CLOCK_IN);
	if (ret < 0)
	{
		printk("smdkc110_atv_hw_params : AP sys clock setting error!\n");
		return ret;
	}

	switch (params_rate(params)) {
		case 8000:
		case 16000:
		case 32000:
		case 48000:
		case 64000:
		case 96000:
			ap_codec_clk = 49152000;
			break;
		case 11025:
		case 22050:
		case 44100:
		case 88200:
		default:
			ap_codec_clk = 67738000;
			break;
	}


	/* Calculate Prescalare/PLL values for supported Rates */	
	psr = ap_codec_clk / rfs / params_rate(params);
	ret = ap_codec_clk / rfs - psr * params_rate(params);

	if(ret >= params_rate(params)/2)	// round off
		psr += 1;

	psr -= 1;
	printk("ap_codec_clk=%d PSR=%d RFS=%d BFS=%d\n", ap_codec_clk, psr, rfs, bfs);

	/* Set the AP Prescalar/Pll */
	//	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_DIV_PRESCALER, psr);

	//	if (ret < 0)
	{
		//		printk("smdkc110_atv_hw_params : AP prescalar setting error!\n");
		//		return ret;
	}

	/* Set the AP RFS */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_DIV_MCLK, rfs);
	if (ret < 0)
	{
		printk("smdkc110_atv_hw_params : AP RFS setting error!\n");
		return ret;
	}

	/* Set the AP BFS */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_DIV_BCLK, bfs);
	if (ret < 0)
	{
		printk("smdkc110_atv_hw_params : AP BCLK setting error!\n");
		return ret;
	}

	return 0;
}


static int smdkc110_atv_init(struct snd_soc_codec *codec)
{
	debug_msg("%s\n", __FUNCTION__);

#if 0
	/* Add smdkc110 specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8580_smdkc110_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&wm8580_smdkc110_controls[i], codec, NULL));
		if (err < 0)
			return err;
	}

	/* Add smdkc110 specific widgets */
	snd_soc_dapm_new_controls(codec, wm8580_dapm_widgets,
			ARRAY_SIZE(wm8580_dapm_widgets));

	/* Set up smdkc110 specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* No jack detect - mark all jacks as enabled */
	for (i = 0; i < ARRAY_SIZE(wm8580_dapm_widgets); i++)
		snd_soc_dapm_enable_pin(codec, wm8580_dapm_widgets[i].name);

	/* Setup Default Route */
	smdkc110_play_opt = PLAY_STEREO;
	smdkc110_rec_opt = REC_LINE;
	smdkc110_ext_control(codec);
#endif
	return 0;
}


/* machine stream operations */
static struct snd_soc_ops smdkc110_ops = {
	.hw_params = smdkc110_hw_params,
};

/* digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link smdkc1xx_dai = {
	.name = "ATV",
	.stream_name = "I2S2 ATV Playback",
	.cpu_dai = &i2s2_dai,
	.codec_dai = &nmi625_dai,
	.init = smdkc110_atv_init,
	.ops = &smdkc110_ops,
};

static struct snd_soc_card smdkc110 = {
	.name = "smdkc110-atv",
	.platform = &s5pc110_soc_platform, //sayanta check
	.dai_link = &smdkc1xx_dai,
	.num_links = 1,//ARRAY_SIZE(smdkc1xx_dai),
};

static struct wm8994_setup_data smdkc110_wm8994_setup = {
#if 0
	/*
		The I2C address of the WM89940 is 0x34. To the I2C driver 
		the address is a 7-bit number hence the right shift .
	*/
	.i2c_address = 0x34,
	.i2c_bus = 4,
#endif //TODO check what address need to give for ATV / and how the interface is??
};

/* audio subsystem */
static struct snd_soc_device smdkc1xx_snd_devdata = {
	.card = &smdkc110,
	.codec_dev = &soc_codec_dev_nmi625,
};

static struct platform_device *smdkc1xx_snd_device; 
static int __init smdkc110_audio_init(void)
{
	int ret;

	debug_msg("%s\n", __FUNCTION__);
	//TODO set GPIOs for I2S2 here
	
	smdkc1xx_snd_device = platform_device_alloc("soc-audio", 2);
	if (!smdkc1xx_snd_device)
		return -ENOMEM;

	platform_set_drvdata(smdkc1xx_snd_device, &smdkc1xx_snd_devdata);
	smdkc1xx_snd_devdata.dev = &smdkc1xx_snd_device->dev;
	ret = platform_device_add(smdkc1xx_snd_device);

	if (ret)
		platform_device_put(smdkc1xx_snd_device);

	return ret;
}

static void __exit smdkc110_audio_exit(void)
{
	debug_msg("%s\n", __FUNCTION__);

	platform_device_unregister(smdkc1xx_snd_device);
}

module_init(smdkc110_audio_init);
module_exit(smdkc110_audio_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC SMDKC110 ATV");
MODULE_LICENSE("GPL");
