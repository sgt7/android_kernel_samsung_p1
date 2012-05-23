/*
 * wm8994_aries.c  --  WM8994 ALSA Soc Audio driver related Aries
 *
 *  Copyright (C) 2010 Samsung Electronics.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/gpio.h> 
#include <plat/gpio-cfg.h> 
#include <plat/map-base.h>
#include <mach/regs-clock.h> 
#include <mach/gpio-p1.h> 
#include "wm8994.h"
#include "wm8994_def.h"

#define SUBJECT "wm8994_p1.c"

#ifdef CONFIG_SND_VOODOO
#include "wm8994_voodoo.h"
#endif

//------------------------------------------------
// Definitions of tunning volumes for wm8994
//------------------------------------------------
// DAC
#define TUNING_DAC1L_VOL 0xC0 // 610h
#define TUNING_DAC1R_VOL 0xC0 // 611h
#define TUNING_DAC2L_VOL 0xC0 // 612h
#define TUNING_DAC2R_VOL 0xC0 // 613h

// Speaker
#define TUNING_SPKL_VOL 0x3E // 26h
#define TUNING_SPKR_VOL 0x3E // 27h
#define TUNING_CLASSD_VOL 0x6 // 25h

#define TUNING_SPKMIXL_ATTEN 0x0 // 22h
#define TUNING_SPKMIXR_ATTEN 0x0 // 23h

#ifdef CONFIG_TARGET_LOCALE_VZW
#define TUNING_MP3_SPKR_VOL 0x3D // 27h
#define TUNING_MP3_SPKL_VOL 0x3D // 26h
#else
#define TUNING_MP3_SPKR_VOL 0x3E // 27h
#define TUNING_MP3_SPKL_VOL 0x3E // 26h
#endif

#define TUNING_MP3_CLASSD_VOL 0x5 // 25h

#define TUNING_FMRADIO_SPKL_VOL 0x3E // 26h
#define TUNING_FMRADIO_CLASSD_VOL 0x5 // 25h

#ifdef CONFIG_TARGET_LOCALE_KOR
#define TUNING_CALL_CLASSD_VOL	 0x2D // 25h
#define TUNING_CALL_SPKL_VOL 0x3E // 26h
#define TUNING_CALL_SPKR_VOL 0x3E // 27h
#else
#define TUNING_CALL_CLASSD_VOL 0x24 // 25h
#define TUNING_CALL_SPKL_VOL 0x3E // 26h
#define TUNING_CALL_SPKR_VOL 0x3E // 27h
#endif


// Headset
#define TUNING_EAR_OUTMIX5_VOL 0x0 // 31h
#define TUNING_EAR_OUTMIX6_VOL 0x0 // 32h

#define TUNING_RING_OUTPUTL_VOL 0x36 // 1Ch
#define TUNING_RING_OUTPUTR_VOL 0x36 // 1Dh
#define TUNING_RING_OPGAL_VOL 0x39 // 20h
#define TUNING_RING_OPGAR_VOL 0x39 // 21h

#define TUNING_MP3_OUTPUTL_VOL 0x36 // 1Ch
#define TUNING_MP3_OUTPUTR_VOL 0x36 // 1Dh
#define TUNING_MP3_OPGAL_VOL 0x39 // 20h
#define TUNING_MP3_OPGAR_VOL 0x39 // 21h

// Dual
#define TUNING_MP3_DUAL_OUTPUTL_VOL 0x1E // 1Ch
#define TUNING_MP3_DUAL_OUTPUTR_VOL 0x1E // 1Dh

// Extra_Dock_speaker
#define TUNING_MP3_EXTRA_DOCK_SPK_OPGAL_VOL 0x3D // 20h
#define TUNING_MP3_EXTRA_DOCK_SPK_OPGAR_VOL 0x3D // 21h

#define TUNING_EXTRA_DOCK_SPK_OUTMIX5_VOL 0x2 // 31h
#define TUNING_EXTRA_DOCK_SPK_OUTMIX6_VOL 0x2 // 32h
#define TUNING_MP3_EXTRA_DOCK_SPK_VOL 0x0 //1Eh


#define TUNING_CALL_OUTPUTL_VOL 0x39 // 1Ch
#define TUNING_CALL_OUTPUTR_VOL 0x39 // 1Dh
#define TUNING_CALL_OPGAL_VOL 0x39 // 20h
#define TUNING_CALL_OPGAR_VOL 0x39 // 21h

#define TUNING_FMRADIO_OUTPUTL_VOL 0x3D // 1Ch
#define TUNING_FMRADIO_OUTPUTR_VOL 0x3D // 1Dh
#define TUNING_FMRADIO_OPGAL_VOL 0x39 // 20h
#define TUNING_FMRADIO_OPGAR_VOL 0x39 // 21h

#define TUNING_HPOUTMIX_VOL 0x0

// Dual
#define TUNING_DUAL_OUTPUTL_VOL 0x20 // 1Ch
#define TUNING_DUAL_OUTPUTR_VOL 0x20 // 1Dh

//------------------------------------------------
// Recording
// Main MIC
#ifdef CONFIG_TARGET_LOCALE_KOR
#define TUNING_RECORD_MAIN_INPUTLINE_VOL 0x16 // 18h
#else
#define TUNING_RECORD_MAIN_INPUTLINE_VOL 0x17 // 18h
#endif
#define TUNING_RECORD_MAIN_AIF1ADCL_VOL 0xC0 // 400h
#define TUNING_RECORD_MAIN_AIF1ADCR_VOL 0xC0 // 401h

#define TUNING_RECOGNITION_MAIN_INPUTLINE_VOL 0x0F // 18h 
#define TUNING_RECOGNITION_MAIN_AIF1ADCL_VOL 0xC0 // 400h
#define TUNING_RECOGNITION_MAIN_AIF1ADCR_VOL 0xC0 // 401h

#ifdef FEATURE_VSUITE_RECOGNITION
#define TUNING_VSUITE_RECOGNITION_MAIN_INPUTLINE_VOL 0x0F // 18h 
#define TUNING_VSUITE_RECOGNITION_MAIN_AIF1ADCL_VOL 0xC0 // 400h
#define TUNING_VSUITE_RECOGNITION_MAIN_AIF1ADCR_VOL 0xC0 // 401h
#endif

// Ear MIC
#define TUNING_RECORD_SUB_INPUTMIX_VOL 0x15 // 1Ah
#define TUNING_RECORD_SUB_AIF1ADCL_VOL 0xC0 // 400h
#define TUNING_RECORD_SUB_AIF1ADCR_VOL 0xC0 // 401h

#define TUNING_RECOGNITION_SUB_INPUTMIX_VOL 0x0A // 1Ah
#define TUNING_RECOGNITION_SUB_AIF1ADCL_VOL 0xC0 // 400h
#define TUNING_RECOGNITION_SUB_AIF1ADCR_VOL 0xC0 // 401h

#ifdef FEATURE_VSUITE_RECOGNITION
#define TUNING_VSUITE_RECOGNITION_SUB_INPUTMIX_VOL 0x12 // 1Ah
#define TUNING_VSUITE_RECOGNITION_SUB_AIF1ADCL_VOL 0xC0 // 400h
#define TUNING_VSUITE_RECOGNITION_SUB_AIF1ADCR_VOL 0xC0 // 401h
#endif

//BT MIC
#define TUNING_RECORD_BT_AIF2DACR_VOL 0xC0 //501h
#define TUNING_RECORD_BT_AIF2DACL_VOL 0xC0 //502h
#define TUNING_RECORD_BT_AIF1ADC1L_VOL 0xC0 //400h
#define TUNING_RECORD_BT_AIF1ADC1R_VOL 0xC0 //401h

#define TUNING_RECOGNITION_BT_AIF2DACL_VOL 0xC0 //501h
#define TUNING_RECOGNITION_BT_AIF2DACR_VOL 0xC0 //502h
#define TUNING_RECOGNITION_BT_AIF1ADC1L_VOL 0xB8 //400h
#define TUNING_RECOGNITION_BT_AIF1ADC1R_VOL 0xB8 //401h

#ifdef FEATURE_VSUITE_RECOGNITION
#define TUNING_VSUITE_RECOGNITION_BT_AIF2DACL_VOL 0xC0 //501h
#define TUNING_VSUITE_RECOGNITION_BT_AIF2DACR_VOL 0xC0 //502h
#define TUNING_VSUITE_RECOGNITION_BT_AIF1ADC1L_VOL 0xB8 //400h
#define TUNING_VSUITE_RECOGNITION_BT_AIF1ADC1R_VOL 0xB8 //401h
#endif

// Call Main MIC
#define TUNING_CALL_RCV_INPUTMIX_VOL 0x16 // 18h
#define TUNING_CALL_RCV_MIXER_VOL WM8994_IN1L_MIXINL_VOL // 29h 30dB
#ifdef CONFIG_TARGET_LOCALE_KOR
#define TUNING_CALL_SPK_INPUTMIX_VOL 0x11 // 18h
#else
#define TUNING_CALL_SPK_INPUTMIX_VOL 0x17 // 18h
#endif

#define TUNING_CALL_SPK_MIXER_VOL WM8994_IN1L_MIXINL_VOL // 29h 30dB


// Call Ear MIC
#define TUNING_CALL_EAR_INPUTMIX_VOL 0x1F // 1Ah

// Receiver
#define TUNING_RCV_OUTMIX5_VOL 0x0 // 31h
#define TUNING_RCV_OUTMIX6_VOL 0x0 // 32h
#define TUNING_RCV_OPGAL_VOL 0x3D // 20h
#define TUNING_RCV_OPGAR_VOL 0x3D // 21h
#define TUNING_HPOUT2_VOL 0x0 // 1Fh

//Input
#define TUNING_DAC1L_RADIO_VOL 0xA8 // 402h
#define TUNING_DAC1R_RADIO_VOL	 0xA8 // 403h

//Dual
#define TUNING_DUAL_DAC1L_RADIO_VOL 0x70 // 402h
#define TUNING_DUAL_DAC1R_RADIO_VOL 0x70 // 403h

// FM Radio Input
#define TUNING_FMRADIO_EAR_INPUTMIXL_VOL 0x0B // 19h
#define TUNING_FMRADIO_EAR_INPUTMIXR_VOL 0x0B // 1Bh

#define TUNING_FMRADIO_SPK_INPUTMIXL_VOL 0x0F // 19h
#define TUNING_FMRADIO_SPK_INPUTMIXR_VOL 0x0F // 1Bh


// ear loopback
#define TUNING_LOOPBACK_EAR_INPUTMIX_VOL 0x1D // 1Ah

#define TUNING_LOOPBACK_OUTPUTL_VOL 0x37 // 1Ch
#define TUNING_LOOPBACK_OUTPUTR_VOL 0x37 // 1Dh
#define TUNING_LOOPBACK_OPGAL_VOL 0x39 // 20h
#define TUNING_LOOPBACK_OPGAR_VOL 0x39 // 21h


int audio_init(void)
{
	//CODEC LDO SETTING
	if (gpio_is_valid(GPIO_CODEC_LDO_EN))
	{
		if (gpio_request(GPIO_CODEC_LDO_EN, "CODEC_LDO_EN"))
		{
			DEBUG_LOG_ERR("Failed to request CODEC_LDO_EN!");
		}
		
		gpio_direction_output(GPIO_CODEC_LDO_EN, 0);
	}

	s3c_gpio_setpull(GPIO_CODEC_LDO_EN, S3C_GPIO_PULL_NONE);

	s3c_gpio_slp_cfgpin(GPIO_CODEC_LDO_EN, S3C_GPIO_SLP_PREV);
	s3c_gpio_slp_setpull_updown(GPIO_CODEC_LDO_EN, S3C_GPIO_PULL_NONE);


	if (gpio_is_valid(GPIO_MICBIAS_EN))
	{
		if (gpio_request(GPIO_MICBIAS_EN, "GPJ4"))
		{
			DEBUG_LOG_ERR("Failed to request GPIO_MICBIAS_EN!");
		}

		gpio_direction_output(GPIO_MICBIAS_EN, 0);
	}
	
	s3c_gpio_setpull(GPIO_MICBIAS_EN, S3C_GPIO_PULL_NONE);

	s3c_gpio_slp_cfgpin(GPIO_MICBIAS_EN, S3C_GPIO_SLP_PREV);
	s3c_gpio_slp_setpull_updown(GPIO_MICBIAS_EN, S3C_GPIO_PULL_NONE);

	DEBUG_LOG("MICBIAS Init");

	return 0;
}


int codec_power_status = 0;

int audio_power(int en)
{
	u32 val;


	if(en)
	{
		// Forbid to turn off MCLK in sleep mode.
		val = __raw_readl(S5P_SLEEP_CFG);
		val |= (S5P_SLEEP_CFG_USBOSC_EN);
		__raw_writel(val , S5P_SLEEP_CFG);

		// Turn on LDO for codec.
		gpio_set_value(GPIO_CODEC_LDO_EN, 1);

		msleep(10);	// Wait for warming up.

		// Turn on master clock.
#if (defined CONFIG_SND_S5P_WM8994_MASTER)
		__raw_writel(__raw_readl(S5P_OTHERS) | (3<<8) , S5P_OTHERS);
		//__raw_writel(__raw_readl(S5P_CLK_OUT) | (0x1) , S5P_CLK_OUT);	
#endif
	}
	else
	{
		// Turn off LDO for codec.
		gpio_set_value(GPIO_CODEC_LDO_EN, 0);

		msleep(125);	// Wait to turn off codec entirely.

		// Turn off master clock.
#if (defined CONFIG_SND_S5P_WM8994_MASTER)
		__raw_writel(__raw_readl(S5P_OTHERS) & (~(0x3<<8)) , S5P_OTHERS);
		__raw_writel(__raw_readl(S5P_CLK_OUT) & (0xFFFFFFFE) , S5P_CLK_OUT);
#endif

		// Allow to turn off MCLK in sleep mode.
		val = __raw_readl(S5P_SLEEP_CFG);
		val &= ~(S5P_SLEEP_CFG_USBOSC_EN);
		__raw_writel(val , S5P_SLEEP_CFG);
	}

	codec_power_status = en;
	DEBUG_LOG("AUDIO POWER COMPLETED : %d", en);

	return 0;
}

int get_audio_power_status(void)
{
	return codec_power_status;
}


void audio_ctrl_mic_bias_gpio(int enable)
{
	//DEBUG_LOG("audio_ctrl_mic_bias_gpio = [%d]", enable);

	if(enable)
	{
		DEBUG_LOG("MICBIAS On");
		gpio_set_value(GPIO_MICBIAS_EN, 1);
	}
	else
	{
		DEBUG_LOG("MICBIAS Off");
		gpio_set_value(GPIO_MICBIAS_EN, 0);
	}
}

void audio_ctrl_earmic_bias_gpio(int enable)
{
	if(enable)
	{
		DEBUG_LOG("EAR_MICBIAS On");
		gpio_set_value(GPIO_EAR_MICBIAS0_EN, 1);
		gpio_set_value(GPIO_EAR_MICBIAS_EN, 1);
	}
	else
	{
		DEBUG_LOG("EAR_MICBIAS Off");
		gpio_set_value(GPIO_EAR_MICBIAS0_EN, 0);
		gpio_set_value(GPIO_EAR_MICBIAS_EN, 0);
	}
}


/*Audio Routing routines for the universal board..wm8994 codec*/
void wm8994_disable_playback_path(struct snd_soc_codec *codec, enum playback_path path)
{
	u16 val;

	DEBUG_LOG("Path = [%d]", path);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);	
	
	switch(path)
	{
		case RCV:		
			//Disable the HPOUT2
			val &= ~(WM8994_HPOUT2_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);
			
			// Disable left MIXOUT
			val = wm8994_read(codec,WM8994_OUTPUT_MIXER_1);
			val &= ~(WM8994_DAC1L_TO_MIXOUTL_MASK);
			wm8994_write(codec,WM8994_OUTPUT_MIXER_1,val);

			// Disable right MIXOUT
			val = wm8994_read(codec,WM8994_OUTPUT_MIXER_2);
			val &= ~(WM8994_DAC1R_TO_MIXOUTR_MASK);
			wm8994_write(codec,WM8994_OUTPUT_MIXER_2,val);

			// Disable HPOUT Mixer
			val = wm8994_read(codec,WM8994_HPOUT2_MIXER);
			val &= ~(WM8994_MIXOUTLVOL_TO_HPOUT2_MASK | WM8994_MIXOUTRVOL_TO_HPOUT2_MASK);
			wm8994_write(codec,WM8994_HPOUT2_MIXER,val);

			// Disable mixout volume control
			val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_3);
			val &= ~(WM8994_MIXOUTLVOL_ENA_MASK | WM8994_MIXOUTRVOL_ENA_MASK | WM8994_MIXOUTL_ENA_MASK | WM8994_MIXOUTR_ENA_MASK);
			wm8994_write(codec,WM8994_POWER_MANAGEMENT_3,val);			
		break;

		case SPK:
			//Disable the SPKOUTL & SPKOUTR
			val &= ~(WM8994_SPKOUTL_ENA_MASK|WM8994_SPKOUTR_ENA_MASK); 
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

			// Disable SPKLVOL
			val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_3);
			val &= ~(WM8994_SPKLVOL_ENA_MASK | WM8994_SPKRVOL_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

			// Disable SPKOUT mixer
			val = wm8994_read(codec,WM8994_SPKOUT_MIXERS);
			val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
			wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);
			
			//Mute Speaker mixer
			val = wm8994_read(codec,WM8994_SPEAKER_MIXER);
			val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK | WM8994_DAC1R_TO_SPKMIXR_MASK);
			wm8994_write(codec, WM8994_SPEAKER_MIXER ,val);
		break;

		case HP:
		case HP_NO_MIC:
			//Disable the HPOUT1L & HPOUT1R
			val &= ~(WM8994_HPOUT1L_ENA_MASK |WM8994_HPOUT1R_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);
			
			// Disable DAC1L to HPOUT1L path
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
			val &= ~(WM8994_DAC1L_TO_HPOUT1L_MASK | WM8994_DAC1L_TO_MIXOUTL_MASK);
			wm8994_write(codec,WM8994_OUTPUT_MIXER_1, val);
			
			// Disable DAC1R to HPOUT1R path
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
			val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK | WM8994_DAC1R_TO_MIXOUTR_MASK);
			wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);
			
			//Disable Charge Pump	
			val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
			val &= ~WM8994_CP_ENA_MASK ;
			val |=  WM8994_CP_ENA_DEFAULT ; // this is from wolfson	
			wm8994_write(codec, WM8994_CHARGE_PUMP_1, val);
			
			// Intermediate HP settings
			val = wm8994_read(codec, WM8994_ANALOGUE_HP_1); 	
			val &= ~(WM8994_HPOUT1R_DLY_MASK |WM8994_HPOUT1R_OUTP_MASK |WM8994_HPOUT1R_RMV_SHORT_MASK |
				WM8994_HPOUT1L_DLY_MASK |WM8994_HPOUT1L_OUTP_MASK | WM8994_HPOUT1L_RMV_SHORT_MASK);
			wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);
		break;

		case BT :
			; //todo
		break;

		case SPK_HP :
			//Disable the HPOUT1L & HPOUT1R & SPKOUTL & SPKOUTR
			val &= ~(WM8994_HPOUT1L_ENA_MASK |WM8994_HPOUT1R_ENA_MASK |WM8994_SPKOUTL_ENA_MASK | WM8994_SPKOUTR_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

			// ------------------ Ear path setting ------------------
			// Disable DAC1L to HPOUT1L path
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
			val &= ~(WM8994_DAC1L_TO_HPOUT1L_MASK | WM8994_DAC1L_TO_MIXOUTL_MASK);
			wm8994_write(codec,WM8994_OUTPUT_MIXER_1, val);
			
			// Disable DAC1R to HPOUT1R path
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
			val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK | WM8994_DAC1R_TO_MIXOUTR_MASK);
			wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);
			
			//Disable Charge Pump	
			val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
			val &= ~WM8994_CP_ENA_MASK ;
			val |=  WM8994_CP_ENA_DEFAULT ; // this is from wolfson	
			wm8994_write(codec, WM8994_CHARGE_PUMP_1, val);
			
			// Intermediate HP settings
			val = wm8994_read(codec, WM8994_ANALOGUE_HP_1); 	
			val &= ~(WM8994_HPOUT1R_DLY_MASK |WM8994_HPOUT1R_OUTP_MASK |WM8994_HPOUT1R_RMV_SHORT_MASK |
				WM8994_HPOUT1L_DLY_MASK |WM8994_HPOUT1L_OUTP_MASK | WM8994_HPOUT1L_RMV_SHORT_MASK);
			wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

			// ------------------ Spk path setting ------------------		
			// Disable SPKLVOL
			val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_3);
			val &= ~(WM8994_SPKLVOL_ENA_MASK | WM8994_SPKRVOL_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

			// Disable SPKOUT mixer
			val = wm8994_read(codec,WM8994_SPKOUT_MIXERS);
			val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
			wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);
			
			//Mute Speaker mixer
			val = wm8994_read(codec,WM8994_SPEAKER_MIXER);
			val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK | WM8994_DAC1R_TO_SPKMIXR_MASK);
			wm8994_write(codec, WM8994_SPEAKER_MIXER ,val);
		break;			

		default:
			DEBUG_LOG_ERR("Path[%d] is not invaild!", path);
			return;
		break;
	}
}

void wm8994_disable_rec_path(struct snd_soc_codec *codec, enum mic_path rec_path) 
{
	struct wm8994_priv *wm8994 = codec->drvdata;

	u16 val;

	wm8994->rec_path = MIC_OFF;

	if(!(wm8994->codec_state & CALL_ACTIVE))	// Normal case
	{
		audio_ctrl_mic_bias_gpio(0);	// Disable MIC bias
	}

	switch(rec_path)
	{
		case MAIN:
			DEBUG_LOG("Disabling MAIN Mic Path..");

			if(!(wm8994->codec_state & CALL_ACTIVE))	// Normal case
			{
				val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_2  );
				val &= ~( WM8994_IN1L_ENA_MASK | WM8994_MIXINL_ENA_MASK);
				wm8994_write(codec,WM8994_POWER_MANAGEMENT_2, val);

				if(wm8994->testmode_config_flag == SEC_NORMAL)
				{	
					// Mute IN1L PGA, update volume
					val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);	
					val &= ~(WM8994_IN1L_MUTE_MASK | WM8994_IN1L_VOL_MASK);
					val |= (WM8994_IN1L_VU |WM8994_IN1L_MUTE); //volume
					wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

					//Mute the PGA
					val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
					val&= ~(WM8994_IN1L_TO_MIXINL_MASK | WM8994_IN1L_MIXINL_VOL_MASK | WM8994_MIXOUTL_MIXINL_VOL_MASK);
					wm8994_write(codec, WM8994_INPUT_MIXER_3, val); 
				}

				//Disconnect IN1LN ans IN1LP to the inputs
				val = wm8994_read(codec,WM8994_INPUT_MIXER_2 ); 
				val &= (WM8994_IN1LN_TO_IN1L_MASK | WM8994_IN1LP_TO_IN1L_MASK);
				wm8994_write(codec, WM8994_INPUT_MIXER_2, val);

				//Digital Paths 	
				val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
				val &= ~(WM8994_ADCL_ENA_MASK |WM8994_AIF1ADC1L_ENA_MASK);
				wm8994_write(codec,WM8994_POWER_MANAGEMENT_4, val);
			}
			else
			{
				//Digital Paths 	
				val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
				val &= ~(WM8994_AIF1ADC1L_ENA_MASK);
				wm8994_write(codec,WM8994_POWER_MANAGEMENT_4, val);
			}

			//Disable timeslots
			val = wm8994_read(codec,WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
			val &= ~(WM8994_ADC1L_TO_AIF1ADC1L);
			wm8994_write(codec,WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);	
		break;

		case SUB:
			DEBUG_LOG("Disbaling SUB Mic path..");

			if(!(wm8994->codec_state & CALL_ACTIVE))	// Normal case
			{
				val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_2);
				val &= ~(WM8994_IN1R_ENA_MASK |WM8994_MIXINR_ENA_MASK);
				wm8994_write(codec,WM8994_POWER_MANAGEMENT_2,val);

				if(wm8994->testmode_config_flag == SEC_NORMAL)
				{	
					// Disable volume,unmute Right Line	
					val = wm8994_read(codec,WM8994_RIGHT_LINE_INPUT_1_2_VOLUME);	
					val &= ~(WM8994_IN1R_MUTE_MASK | WM8994_IN1R_VOL_MASK);	// Unmute IN1R
					val |= (WM8994_IN1R_VU | WM8994_IN1R_MUTE);
					wm8994_write(codec,WM8994_RIGHT_LINE_INPUT_1_2_VOLUME, val);

					// Mute right pga, set volume 
					val = wm8994_read(codec,WM8994_INPUT_MIXER_4);
					val&= ~(WM8994_IN1R_TO_MIXINR_MASK | WM8994_IN1R_MIXINR_VOL_MASK | WM8994_MIXOUTR_MIXINR_VOL_MASK);
					wm8994_write(codec,WM8994_INPUT_MIXER_4, val);
				}

				//Disconnect in1rn to inr1 and in1rp to inrp
				val = wm8994_read(codec,WM8994_INPUT_MIXER_2);
				val &= ~(WM8994_IN1RN_TO_IN1R_MASK | WM8994_IN1RP_TO_IN1R_MASK);
				wm8994_write(codec,WM8994_INPUT_MIXER_2, val);

				//Digital Paths 
				//Disable right ADC and time slot
				val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
				val &= ~(WM8994_ADCR_ENA_MASK |WM8994_AIF1ADC1R_ENA_MASK);
				wm8994_write(codec,WM8994_POWER_MANAGEMENT_4, val);			
			}
			else
			{
				//Digital Paths 
				//Disable right ADC and time slot
				val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
				val &= ~(WM8994_AIF1ADC1R_ENA_MASK);
				wm8994_write(codec,WM8994_POWER_MANAGEMENT_4, val);

				val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);	//605H : 0x0010
				val &= ~(WM8994_AIF1ADCL_SRC_MASK);
				wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);
			}

			//ADC Right mixer routing
			val = wm8994_read(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
			val &= ~(WM8994_ADC1R_TO_AIF1ADC1R_MASK);
			wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);
		break;

		case BT_REC:
			if(!(wm8994->codec_state & CALL_ACTIVE))	// Normal case
			{
				//R1542(606h) - 0x0000
				val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
				val &= ~(WM8994_AIF2DACL_TO_AIF1ADC1L_MASK | WM8994_ADC1L_TO_AIF1ADC1L_MASK);
				wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);

				//R1543(607h) - 0x0000
				val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
				val &= ~(WM8994_AIF2DACR_TO_AIF1ADC1R_MASK | WM8994_ADC1R_TO_AIF1ADC1R_MASK);
				wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);

				//R1312(520h) - 0x0200
				val = wm8994_read(codec, WM8994_AIF2_DAC_FILTERS_1);
				val &= ~(WM8994_AIF2DAC_MUTE_MASK);
				val |= (WM8994_AIF2DAC_MUTE);
				wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, val); 
			}
		break;

		default:
                    if((wm8994->codec_state & CALL_ACTIVE) &&  (wm8994->call_record_ch == CH_OFF))
                    {
                            if(wm8994->call_record_path == CALL_RECORDING_MAIN)
                            {
                                    DEBUG_LOG("Disabling MAIN Mic Path in call");
                                    //Digital Paths 	
                                    val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
                                    val &= ~(WM8994_AIF1ADC1L_ENA_MASK);
                                    wm8994_write(codec,WM8994_POWER_MANAGEMENT_4, val);

                                    //Disable timeslots
                                    val = wm8994_read(codec,WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
                                    val &= ~(WM8994_ADC1L_TO_AIF1ADC1L);
                                    wm8994_write(codec,WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);	
                            }
                            else if(wm8994->call_record_path == CALL_RECORDING_SUB)
                            {
                                    DEBUG_LOG("Disbaling SUB Mic path in call");
                                    //Digital Paths 
                                    //Disable right ADC and time slot
                                    val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
                                    val &= ~(WM8994_AIF1ADC1R_ENA_MASK);
                                    wm8994_write(codec,WM8994_POWER_MANAGEMENT_4, val);

                                    val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);	//605H : 0x0010
                                    val &= ~(WM8994_AIF1ADCL_SRC_MASK);
                                    wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

                                    //ADC Right mixer routing
                                    val = wm8994_read(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
                                    val &= ~(WM8994_ADC1R_TO_AIF1ADC1R_MASK);
                                    wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);
                            }

                            wm8994->call_record_path = CALL_RECORDING_OFF;
                    }
                    else
                    {
                            DEBUG_LOG_ERR("Path[%d] is not invaild!", rec_path);
                    }
		break;
	}
}

void wm8994_disable_fmradio_path(struct snd_soc_codec *codec, enum fmradio_path path)
{
	struct wm8994_priv *wm8994 = codec->drvdata;

	u16 val;

	DEBUG_LOG("Turn off fmradio_path = [%d]", path);

	switch(path)
	{
		case FMR_OFF :
			wm8994->fmradio_path = FMR_OFF;

			//---------------------------------------------------
			// Disable speaker setting for FM radio
			if(wm8994->codec_state & CALL_ACTIVE)
			{

			//disable the SPKOUTL
			val =wm8994_read(codec,WM8994_POWER_MANAGEMENT_1);
			val &= ~(WM8994_SPKOUTL_ENA_MASK); 
			wm8994_write(codec,WM8994_POWER_MANAGEMENT_1 ,val);

			// Disable SPK Volume.
			val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
			val &= ~(WM8994_SPKRVOL_ENA_MASK | WM8994_SPKLVOL_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

			if(wm8994->testmode_config_flag == SEC_NORMAL)
			{	
				// Mute the SPKMIXVOLUME
				val = wm8994_read(codec, WM8994_SPKMIXL_ATTENUATION);
				val &= ~(WM8994_SPKMIXL_VOL_MASK);
				wm8994_write(codec, WM8994_SPKMIXL_ATTENUATION, val);
					
				val = wm8994_read(codec, WM8994_SPKMIXR_ATTENUATION);
				val &= ~(WM8994_SPKMIXR_VOL_MASK);
				wm8994_write(codec, WM8994_SPKMIXR_ATTENUATION, val);
			
				val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
				val &= ~(WM8994_SPKOUTL_MUTE_N_MASK | WM8994_SPKOUTL_VOL_MASK);
				wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);
			
				val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_RIGHT);
				val &= ~(WM8994_SPKOUTR_MUTE_N_MASK | WM8994_SPKOUTR_VOL_MASK);
				val |= (WM8994_SPKOUT_VU);
				wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);
			
				val = wm8994_read(codec, WM8994_CLASSD);
				val &= ~(WM8994_SPKOUTL_BOOST_MASK);
				wm8994_write(codec, WM8994_CLASSD, val);
			}
			
			/*Output MIxer-Output PGA*/
			val = wm8994_read(codec,WM8994_SPKOUT_MIXERS );
			val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | 
				WM8994_SPKMIXR_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
			wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);
							
			// Output mixer setting
			val = wm8994_read(codec, WM8994_SPEAKER_MIXER);
			val &= ~(WM8994_MIXINL_TO_SPKMIXL_MASK | WM8994_MIXINR_TO_SPKMIXR_MASK);
			wm8994_write(codec, WM8994_SPEAKER_MIXER, val);

			//---------------------------------------------------
			// Disable earpath setting for FM radio

			//Disable end point for preventing pop up noise.
			val =wm8994_read(codec,WM8994_POWER_MANAGEMENT_1);
			val &= ~(WM8994_HPOUT1L_ENA_MASK |WM8994_HPOUT1R_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

			// Disable MIXOUT
			val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
			val &= ~(WM8994_MIXOUTLVOL_ENA_MASK | WM8994_MIXOUTRVOL_ENA_MASK | WM8994_MIXOUTL_ENA_MASK | WM8994_MIXOUTR_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

			if(wm8994->testmode_config_flag == SEC_NORMAL)
			{
				// Output setting
				val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME);
				val &= ~(WM8994_HPOUT1L_MUTE_N_MASK | WM8994_HPOUT1L_VOL_MASK);
				wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);
			
				val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME);
				val &= ~(WM8994_HPOUT1R_MUTE_N_MASK | WM8994_HPOUT1R_VOL_MASK);
				val |= (WM8994_HPOUT1_VU);
				wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);
			}

			//Disable Charge Pump	
			val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
			val &= ~WM8994_CP_ENA_MASK ;
			val |= WM8994_CP_ENA_DEFAULT ; // this is from wolfson		
			wm8994_write(codec, WM8994_CHARGE_PUMP_1, val);
			
			// Intermediate HP settings
			val = wm8994_read(codec, WM8994_ANALOGUE_HP_1); 	
			val &= ~(WM8994_HPOUT1R_DLY_MASK |WM8994_HPOUT1R_OUTP_MASK |WM8994_HPOUT1R_RMV_SHORT_MASK |
				WM8994_HPOUT1L_DLY_MASK |WM8994_HPOUT1L_OUTP_MASK | WM8994_HPOUT1L_RMV_SHORT_MASK);
			wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);
												
			// Disable Output mixer setting
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
			val &= ~(WM8994_DAC1L_TO_HPOUT1L_MASK | WM8994_MIXINL_TO_MIXOUTL_MASK);
			wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val); 
			
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
			val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK | WM8994_MIXINR_TO_MIXOUTR_MASK);
			wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val); 

			//---------------------------------------------------
			// Disable common setting for FM radio
			
			// Disable IN2 and MIXIN
			val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
			val &= ~(WM8994_TSHUT_ENA_MASK | WM8994_TSHUT_OPDIS_MASK | WM8994_OPCLK_ENA_MASK | 
					WM8994_MIXINL_ENA_MASK | WM8994_MIXINR_ENA_MASK | WM8994_IN2L_ENA_MASK | WM8994_IN2R_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);
			
			// Disable Input mixer setting
			val = wm8994_read(codec, WM8994_INPUT_MIXER_2);
			val &= ~(WM8994_IN2LN_TO_IN2L_MASK | WM8994_IN2RN_TO_IN2R_MASK);
			wm8994_write(codec, WM8994_INPUT_MIXER_2, val); 	
			
			if(wm8994->testmode_config_flag == SEC_NORMAL)
			{
				// Disable IN2L to MIXINL
				val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
				val &= ~(WM8994_IN2L_TO_MIXINL_MASK);
				wm8994_write(codec, WM8994_INPUT_MIXER_3, val);
				
				//Disable IN2R to MIXINR
				val = wm8994_read(codec, WM8994_INPUT_MIXER_4);
				val &= ~(WM8994_IN2R_TO_MIXINR_MASK);
				wm8994_write(codec, WM8994_INPUT_MIXER_4, val); 	
			}
							
			// Mute IN2L PGA volume
			val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_3_4_VOLUME);
			val &= ~(WM8994_IN2L_VU_MASK | WM8994_IN2L_MUTE_MASK | WM8994_IN2L_VOL_MASK);
			val |= (WM8994_IN2L_MUTE);
			wm8994_write(codec, WM8994_LEFT_LINE_INPUT_3_4_VOLUME, val); 
			
			val = wm8994_read(codec, WM8994_RIGHT_LINE_INPUT_3_4_VOLUME);
			val &= ~(WM8994_IN2R_VU_MASK | WM8994_IN2R_MUTE_MASK | WM8994_IN2R_VOL_MASK);
			val |= (WM8994_IN2R_VU |WM8994_IN2R_MUTE);	
			wm8994_write(codec, WM8994_RIGHT_LINE_INPUT_3_4_VOLUME, val); 

			//---------------------------------------------------
			// Disable path setting for mixing
			val = wm8994_read(codec,WM8994_SPEAKER_MIXER);
			val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK);
			wm8994_write(codec, WM8994_SPEAKER_MIXER, val);			

			// Disable DAC1L to HPOUT1L path
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
			val &= ~(WM8994_DAC1L_TO_HPOUT1L_MASK | WM8994_DAC1L_TO_MIXOUTL_MASK);
			wm8994_write(codec,WM8994_OUTPUT_MIXER_1, val);
			
			// Disable DAC1R to HPOUT1R path
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
			val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK | WM8994_DAC1R_TO_MIXOUTR_MASK);
			wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);				
			}
			break;
		
		case FMR_SPK :
			//disable the SPKOUTL
			val =wm8994_read(codec,WM8994_POWER_MANAGEMENT_1);
			val &= ~(WM8994_SPKOUTL_ENA_MASK); 
//			wm8994_write(codec,WM8994_POWER_MANAGEMENT_1 ,val);

			// Disable SPK Volume.
			val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
			val &= ~(WM8994_SPKRVOL_ENA_MASK | WM8994_SPKLVOL_ENA_MASK);
//			wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

			if(wm8994->testmode_config_flag == SEC_NORMAL)
			{	
				// Mute the SPKMIXVOLUME
				val = wm8994_read(codec, WM8994_SPKMIXL_ATTENUATION);
				val &= ~(WM8994_SPKMIXL_VOL_MASK);
				wm8994_write(codec, WM8994_SPKMIXL_ATTENUATION, val);
					
				val = wm8994_read(codec, WM8994_SPKMIXR_ATTENUATION);
				val &= ~(WM8994_SPKMIXR_VOL_MASK);
				wm8994_write(codec, WM8994_SPKMIXR_ATTENUATION, val);
			
				val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
				val &= ~(WM8994_SPKOUTL_MUTE_N_MASK | WM8994_SPKOUTL_VOL_MASK);
				wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);
			
				val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_RIGHT);
				val &= ~(WM8994_SPKOUTR_MUTE_N_MASK | WM8994_SPKOUTR_VOL_MASK);
				val |= (WM8994_SPKOUT_VU);
				wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);
			
				val = wm8994_read(codec, WM8994_CLASSD);
				val &= ~(WM8994_SPKOUTL_BOOST_MASK);
				wm8994_write(codec, WM8994_CLASSD, val);
			}
			
			/*Output MIxer-Output PGA*/
			val = wm8994_read(codec,WM8994_SPKOUT_MIXERS );
			val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | 
				WM8994_SPKMIXR_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
			wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);
							
			// Output mixer setting
			val = wm8994_read(codec, WM8994_SPEAKER_MIXER);
			val &= ~(WM8994_MIXINL_TO_SPKMIXL_MASK | WM8994_MIXINR_TO_SPKMIXR_MASK);
			wm8994_write(codec, WM8994_SPEAKER_MIXER, val);
			break;

		case FMR_HP :
			//Disable end point for preventing pop up noise.
			val =wm8994_read(codec,WM8994_POWER_MANAGEMENT_1);
			val &= ~(WM8994_HPOUT1L_ENA_MASK |WM8994_HPOUT1R_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

			// Disable MIXOUT
			val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
			val &= ~(WM8994_MIXOUTLVOL_ENA_MASK | WM8994_MIXOUTRVOL_ENA_MASK | WM8994_MIXOUTL_ENA_MASK | WM8994_MIXOUTR_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

			if(wm8994->testmode_config_flag == SEC_NORMAL)
			{
				// Output setting
				val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME);
				val &= ~(WM8994_HPOUT1L_MUTE_N_MASK | WM8994_HPOUT1L_VOL_MASK);
				wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);
			
				val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME);
				val &= ~(WM8994_HPOUT1R_MUTE_N_MASK | WM8994_HPOUT1R_VOL_MASK);
				val |= (WM8994_HPOUT1_VU);
				wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);
			}

			//Disable Charge Pump	
			val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
			val &= ~WM8994_CP_ENA_MASK ;
			val |= WM8994_CP_ENA_DEFAULT ; // this is from wolfson		
			wm8994_write(codec, WM8994_CHARGE_PUMP_1, val);
			
			// Intermediate HP settings
			val = wm8994_read(codec, WM8994_ANALOGUE_HP_1); 	
			val &= ~(WM8994_HPOUT1R_DLY_MASK |WM8994_HPOUT1R_OUTP_MASK |WM8994_HPOUT1R_RMV_SHORT_MASK |
				WM8994_HPOUT1L_DLY_MASK |WM8994_HPOUT1L_OUTP_MASK | WM8994_HPOUT1L_RMV_SHORT_MASK);
			wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);
												
			// Disable Output mixer setting
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
			val &= ~(WM8994_DAC1L_TO_HPOUT1L_MASK | WM8994_MIXINL_TO_MIXOUTL_MASK);
			wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val); 
			
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
			val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK | WM8994_MIXINR_TO_MIXOUTR_MASK);
			wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val); 
			break;

		case FMR_SPK_MIX :
			//Mute the DAC path
			val = wm8994_read(codec,WM8994_SPEAKER_MIXER);
			val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK);
			wm8994_write(codec, WM8994_SPEAKER_MIXER, val);			
			break;

		case FMR_HP_MIX :					
			// Disable DAC1L to HPOUT1L path
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
			val &= ~(WM8994_DAC1L_TO_HPOUT1L_MASK | WM8994_DAC1L_TO_MIXOUTL_MASK);
			wm8994_write(codec,WM8994_OUTPUT_MIXER_1, val);
			
			// Disable DAC1R to HPOUT1R path
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
			val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK | WM8994_DAC1R_TO_MIXOUTR_MASK);
			wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);	
			break;

		case FMR_DUAL_MIX :		
			//Mute the DAC path
			val = wm8994_read(codec,WM8994_SPEAKER_MIXER);
			val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK);
			wm8994_write(codec, WM8994_SPEAKER_MIXER, val);
			
			// Disable DAC1L to HPOUT1L path
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
			val &= ~(WM8994_DAC1L_TO_HPOUT1L_MASK | WM8994_DAC1L_TO_MIXOUTL_MASK);
			wm8994_write(codec,WM8994_OUTPUT_MIXER_1, val);
			
			// Disable DAC1R to HPOUT1R path
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
			val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK | WM8994_DAC1R_TO_MIXOUTR_MASK);
			wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);	
			break;

		default:
			DEBUG_LOG_ERR("fmradio path[%d] is not invaild!", path);
			return;
			break;
	}
}

void wm8994_record_headset_mic(struct snd_soc_codec *codec) 
{
        struct wm8994_priv *wm8994 = codec->drvdata;

        u16 val;


        DEBUG_LOG("Recording through Headset Mic");


        audio_ctrl_mic_bias_gpio(0);	
        audio_ctrl_earmic_bias_gpio(1);


        if(!(wm8994->codec_state & CALL_ACTIVE))	// Normal case
        {
                // Disable FM radio path
                val = wm8994_read(codec, WM8994_SPEAKER_MIXER);
                val &= ~WM8994_MIXINL_TO_SPKMIXL_MASK;
                wm8994_write(codec, WM8994_SPEAKER_MIXER, val);

                val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
                val &= ~WM8994_MIXINL_TO_MIXOUTL_MASK;
                wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val); 

                val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
                val &= ~WM8994_MIXINR_TO_MIXOUTR_MASK;
                wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

                val = wm8994_read(codec, WM8994_DAC2_LEFT_MIXER_ROUTING);	//604H : 0x0010
                val &= ~(WM8994_ADC1_TO_DAC2L_MASK);
                wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, val);

                val = wm8994_read(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING);	//605H : 0x0010
                val &= ~(WM8994_ADC1_TO_DAC2R_MASK);
                wm8994_write(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING, val);

                // Mixing left channel output to right channel.
                val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);	//605H : 0x0010
                val &= ~(WM8994_AIF1ADCL_SRC_MASK | WM8994_AIF1ADCR_SRC_MASK);
                val |= (WM8994_AIF1ADCL_SRC | WM8994_AIF1ADCR_SRC);
                wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

                wm8994_write(codec,WM8994_ANTIPOP_2,0x68);	//Ear  mic volume issue fix

                //Enable mic bias, vmid, bias generator
                val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1 );
                val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK);
                val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL);
                wm8994_write(codec,WM8994_POWER_MANAGEMENT_1,val);

                //Enable Right Input Mixer,Enable IN1R PGA - Temporary inserted for blocking MIC and FM radio mixing - DW Shim 2010.03.04
                //val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_2 );
                //val &= ~(WM8994_IN1R_ENA_MASK |WM8994_MIXINR_ENA_MASK  );
                //val |= (WM8994_MIXINR_ENA | WM8994_IN1R_ENA );
                val = (WM8994_MIXINR_ENA | WM8994_IN1R_ENA );
                wm8994_write(codec,WM8994_POWER_MANAGEMENT_2,val);

                if(wm8994->testmode_config_flag == SEC_NORMAL)
                {	
                        // Enable volume,unmute Right Line	
                        val = wm8994_read(codec,WM8994_RIGHT_LINE_INPUT_1_2_VOLUME);	
                        val &= ~(WM8994_IN1R_MUTE_MASK | WM8994_IN1R_VOL_MASK);	// Unmute IN1R
                        if(wm8994->recognition_active == REC_ON)
                                val |= (WM8994_IN1R_VU | TUNING_RECOGNITION_SUB_INPUTMIX_VOL);
#ifdef FEATURE_VSUITE_RECOGNITION
                        else if(wm8994->vsuite_recognition_active == REC_ON)
                                val |= (WM8994_IN1R_VU | TUNING_VSUITE_RECOGNITION_SUB_INPUTMIX_VOL);
#endif			
                        else
                                val |= (WM8994_IN1R_VU | TUNING_RECORD_SUB_INPUTMIX_VOL);
                        wm8994_write(codec,WM8994_RIGHT_LINE_INPUT_1_2_VOLUME,val);

                        // unmute right pga, set volume 
                        val = wm8994_read(codec,WM8994_INPUT_MIXER_4 );
                        val&= ~(WM8994_IN1R_TO_MIXINR_MASK | WM8994_IN1R_MIXINR_VOL_MASK | WM8994_MIXOUTR_MIXINR_VOL_MASK);
                        val |= (WM8994_IN1R_TO_MIXINR | WM8994_IN1R_MIXINR_VOL);//30db
                        wm8994_write(codec,WM8994_INPUT_MIXER_4 ,val);
                }

                val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_VOLUME);
                val &= ~(WM8994_AIF1ADC1L_VOL_MASK);
                if(wm8994->recognition_active == REC_ON)		
                        val |= (TUNING_RECOGNITION_SUB_AIF1ADCL_VOL); 
#ifdef FEATURE_VSUITE_RECOGNITION
                else if(wm8994->vsuite_recognition_active == REC_ON)
                        val |= (TUNING_VSUITE_RECOGNITION_SUB_AIF1ADCL_VOL); 
#endif	
                else
                        val |= (TUNING_RECORD_SUB_AIF1ADCL_VOL); // 0db
                wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_VOLUME, val);

                val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_VOLUME);
                val &= ~(WM8994_AIF1ADC1R_VOL_MASK);
                if(wm8994->recognition_active == REC_ON)		
                        val |= (WM8994_AIF1ADC1_VU | TUNING_RECOGNITION_SUB_AIF1ADCR_VOL);
#ifdef FEATURE_VSUITE_RECOGNITION
                else if(wm8994->vsuite_recognition_active == REC_ON)
                        val |= (WM8994_AIF1ADC1_VU | TUNING_VSUITE_RECOGNITION_SUB_AIF1ADCR_VOL);
#endif			
                else
                        val |= (WM8994_AIF1ADC1_VU | TUNING_RECORD_SUB_AIF1ADCR_VOL); // 0db
                wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_VOLUME, val);

                //connect in1rn to inr1 and in1rp to inrp - Temporary inserted for blocking MIC and FM radio mixing - DW Shim 2010.03.04
                //val = wm8994_read(codec,WM8994_INPUT_MIXER_2);
                //val &= ~( WM8994_IN1RN_TO_IN1R_MASK | WM8994_IN1RP_TO_IN1R_MASK);
                //val |= (WM8994_IN1RN_TO_IN1R | WM8994_IN1RP_TO_IN1R)  ;	
                val = (WM8994_IN1RN_TO_IN1R | WM8994_IN1RP_TO_IN1R)  ; 
                wm8994_write(codec,WM8994_INPUT_MIXER_2,val);
        }
        else
        {
                // Mixing left channel output to right channel.
                val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);	//605H : 0x0010
                val &= ~(WM8994_AIF1ADCL_SRC_MASK | WM8994_AIF1ADCR_SRC_MASK);
                val |= (WM8994_AIF1ADCL_SRC | WM8994_AIF1ADCR_SRC);
                wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);
        }

        //Digital Paths	
        //Enable right ADC and time slot
        val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
        val &= ~(WM8994_ADCR_ENA_MASK |WM8994_AIF1ADC1R_ENA_MASK );
        val |= (WM8994_AIF1ADC1R_ENA | WM8994_ADCR_ENA  );
        wm8994_write(codec,WM8994_POWER_MANAGEMENT_4 ,val);

        //ADC Right mixer routing
        val = wm8994_read(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
        val &= ~( WM8994_ADC1R_TO_AIF1ADC1R_MASK);
        val |= WM8994_ADC1R_TO_AIF1ADC1R;
        wm8994_write(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING,val);

        val = wm8994_read(codec, WM8994_AIF1_MASTER_SLAVE);
        val |= (WM8994_AIF1_MSTR | WM8994_AIF1_CLK_FRC | WM8994_AIF1_LRCLK_FRC);	// Master mode
        wm8994_write(codec, WM8994_AIF1_MASTER_SLAVE, val);

        wm8994_write( codec, WM8994_GPIO_1, 0xA101 );   // GPIO1 is Input Enable
}

void wm8994_record_main_mic(struct snd_soc_codec *codec) 
{
	struct wm8994_priv *wm8994 = codec->drvdata;

	u16 val;

	DEBUG_LOG("Recording through Main Mic");
	audio_ctrl_mic_bias_gpio(1);

	if(!(wm8994->codec_state & CALL_ACTIVE))	// Normal case
	{
		// Disable FM radio path
		val = wm8994_read(codec, WM8994_SPEAKER_MIXER);
		val &= ~WM8994_MIXINL_TO_SPKMIXL_MASK;
		wm8994_write(codec, WM8994_SPEAKER_MIXER, val);

		val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
		val &= ~WM8994_MIXINL_TO_MIXOUTL_MASK;
		wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val); 

		val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
		val &= ~WM8994_MIXINR_TO_MIXOUTR_MASK;
		wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val); 

		val = wm8994_read(codec, WM8994_DAC2_LEFT_MIXER_ROUTING);	//604H : 0x0010
		val &= ~(WM8994_ADC1_TO_DAC2L_MASK);
		wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, val);

		val = wm8994_read(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING);	//605H : 0x0010
		val &= ~(WM8994_ADC1_TO_DAC2R_MASK);
		wm8994_write(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING, val);

		// Mixing left channel output to right channel.
		val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);	//605H : 0x0010
		val &= ~(WM8994_AIF1ADCL_SRC_MASK | WM8994_AIF1ADCR_SRC_MASK);
		wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

		wm8994_write(codec, WM8994_ANTIPOP_2, 0x68); //Main mic volume issue fix: requested H/W

		//Enable micbias,vmid,mic1
		val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1);
		val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK);
		val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL);  
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

		//Enable left input mixer and IN1L PGA	- Temporary inserted for blocking MIC and FM radio mixing - DW Shim 2010.03.04
		//val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_2  );
		//val &= ~( WM8994_IN1L_ENA_MASK | WM8994_MIXINL_ENA_MASK );
		//val |= (WM8994_MIXINL_ENA |WM8994_IN1L_ENA );
		val = (WM8994_MIXINL_ENA |WM8994_IN1L_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

		//Enable HPF Filter for google voice search
		if(wm8994->recognition_active == REC_ON
#ifdef FEATURE_VSUITE_RECOGNITION
			|| wm8994->vsuite_recognition_active == REC_ON
#endif				
		){
			val = wm8994_read(codec, WM8994_AIF1_ADC1_FILTERS);
			val &= ~(WM8994_AIF1ADC1L_HPF_MASK | WM8994_AIF1ADC1R_HPF_MASK);
			val |= (WM8994_AIF1ADC1L_HPF | WM8994_AIF1ADC1R_HPF);
#ifdef CONFIG_TARGET_LOCALE_USAGSM
			wm8994_write(codec, WM8994_AIF1_ADC1_FILTERS ,0x3800);
#else
			wm8994_write(codec, WM8994_AIF1_ADC1_FILTERS ,val);
#endif
		}

		if(wm8994->testmode_config_flag == SEC_NORMAL)
		{
			// Unmute IN1L PGA, update volume
			val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);	
			val &= ~(WM8994_IN1L_MUTE_MASK | WM8994_IN1L_VOL_MASK);
			
			if(wm8994->recognition_active == REC_ON)
				val |= (WM8994_IN1L_VU |TUNING_RECOGNITION_MAIN_INPUTLINE_VOL); //volume
#ifdef FEATURE_VSUITE_RECOGNITION
			else if(wm8994->vsuite_recognition_active == REC_ON)
				val |= (WM8994_IN1L_VU |TUNING_VSUITE_RECOGNITION_MAIN_INPUTLINE_VOL); //volume
#endif
			else
				val |= (WM8994_IN1L_VU |TUNING_RECORD_MAIN_INPUTLINE_VOL); //volume
				
			wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

			//Unmute the PGA
			val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
			val&= ~(WM8994_IN1L_TO_MIXINL_MASK | WM8994_IN1L_MIXINL_VOL_MASK | WM8994_MIXOUTL_MIXINL_VOL_MASK);
			val |= (WM8994_IN1L_TO_MIXINL | WM8994_IN1L_MIXINL_VOL); //30db
				
			wm8994_write(codec, WM8994_INPUT_MIXER_3, val); 
		}

		//Connect IN1LN ans IN1LP to the inputs - Temporary inserted for blocking MIC and FM radio mixing - DW Shim 2010.03.04
		//val = wm8994_read(codec,WM8994_INPUT_MIXER_2);	
		//val &= (WM8994_IN1LN_TO_IN1L_MASK | WM8994_IN1LP_TO_IN1L_MASK);
		//val |= (WM8994_IN1LP_TO_IN1L | WM8994_IN1LN_TO_IN1L);
		val = (WM8994_IN1LP_TO_IN1L | WM8994_IN1LN_TO_IN1L);
		wm8994_write(codec, WM8994_INPUT_MIXER_2, val);

		//Digital Paths
		val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_VOLUME);
		val &= ~(WM8994_AIF1ADC1L_VOL_MASK);
		
		if(wm8994->recognition_active == REC_ON)		
			val |= (TUNING_RECOGNITION_MAIN_AIF1ADCL_VOL);
#ifdef FEATURE_VSUITE_RECOGNITION
		else if(wm8994->vsuite_recognition_active == REC_ON)
			val |= (TUNING_VSUITE_RECOGNITION_MAIN_AIF1ADCL_VOL);
#endif		
		else
			val |= (TUNING_RECORD_MAIN_AIF1ADCL_VOL); // 0db
			
		wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_VOLUME, val);

		val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_VOLUME);
		val &= ~(WM8994_AIF1ADC1R_VOL_MASK);
		
		if(wm8994->recognition_active == REC_ON)		
			val |= (WM8994_AIF1ADC1_VU | TUNING_RECOGNITION_MAIN_AIF1ADCR_VOL);
#ifdef FEATURE_VSUITE_RECOGNITION
		else if(wm8994->vsuite_recognition_active == REC_ON)
			val |= (WM8994_AIF1ADC1_VU | TUNING_VSUITE_RECOGNITION_MAIN_AIF1ADCR_VOL);
#endif	
		else
			val |= (WM8994_AIF1ADC1_VU | TUNING_RECORD_MAIN_AIF1ADCR_VOL); // 0db
			
		wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_VOLUME, val);
	}

	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
	val &= ~(WM8994_ADCL_ENA_MASK |WM8994_AIF1ADC1L_ENA_MASK);
	val |= (WM8994_AIF1ADC1L_ENA | WM8994_ADCL_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

	//Enable timeslots
	val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
	val |=WM8994_ADC1L_TO_AIF1ADC1L;  
	wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);

	val = wm8994_read(codec, WM8994_AIF1_MASTER_SLAVE);
	val |= (WM8994_AIF1_MSTR | WM8994_AIF1_CLK_FRC | WM8994_AIF1_LRCLK_FRC);	// Master mode
	wm8994_write(codec, WM8994_AIF1_MASTER_SLAVE, val);

	wm8994_write(codec, WM8994_GPIO_1, 0xA101);   // GPIO1 is Input Enable

	// for stable pcm input when start google voice recognition
	if(wm8994->recognition_active == REC_ON) 
	{
		msleep(300);
	}
#ifdef CONFIG_SND_VOODOO_RECORD_PRESETS
	voodoo_hook_record_main_mic();
#endif
}

void wm8994_record_bluetooth(struct snd_soc_codec *codec)
{
	u16 val;

	struct wm8994_priv *wm8994 = codec->drvdata;

	DEBUG_LOG("BT Record Path for Voice Command");

	if(!(wm8994->codec_state & CALL_ACTIVE))	// Normal case
	{
		wm8994_set_voicecall_common_setting(codec);
	
		val = wm8994_read(codec, WM8994_DAC2_LEFT_MIXER_ROUTING);	//604H : 0x0010
		val &= ~(WM8994_ADC1_TO_DAC2L_MASK);
		wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, val);

		val = wm8994_read(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING);	//605H : 0x0010
		val &= ~(WM8994_ADC1_TO_DAC2R_MASK);
		wm8994_write(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING, val);

		//R1(1h) - 0x0003 -normal vmid
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
		val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK);
		val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL);	
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

		/*Digital Path Enables and Unmutes*/	
		//R2(2h) - 0x0000
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, 0x0000);

		//R3(3h) - 0x0000
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, 0x0000);

		//R4(4h) - 0x0300
		//AIF1ADC1(Left/Right) Output
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
		val &= ~(WM8994_AIF1ADC1L_ENA_MASK |WM8994_AIF1ADC1R_ENA_MASK);
		val |= ( WM8994_AIF1ADC1L_ENA | WM8994_AIF1ADC1R_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_4 , val);

		//R5(5h) - 0x3000
		//AIF2DAC(Left/Right) Input
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);
		val &= ~(WM8994_AIF2DACL_ENA_MASK | WM8994_AIF2DACR_ENA_MASK);
		val |= (WM8994_AIF2DACL_ENA | WM8994_AIF2DACR_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

		//R6(6h) - 0x0002
		//AIF2_DACDAT_SRC(GPIO8/DACDAT3)
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_6);
		val &= ~(WM8994_AIF3_ADCDAT_SRC_MASK |WM8994_AIF2_DACDAT_SRC_MASK);
		val |= (0x1 << WM8994_AIF3_ADCDAT_SRC_SHIFT | WM8994_AIF2_DACDAT_SRC);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_6, val);

		/*Un-Mute*/
		//R1312(520h) - 0x0000
		//AIF2DAC_MUTE(Un-mute)
		val = wm8994_read(codec, WM8994_AIF2_DAC_FILTERS_1);
		val &= ~(WM8994_AIF2DAC_MUTE_MASK);
		wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, val); 

		/*Mixer Routing*/
		//R1542(606h) - 0x0002
		//AIF2DACL_TO_AIF1ADC1L
		val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
		val &= ~(WM8994_AIF2DACL_TO_AIF1ADC1L_MASK);
		val |= (WM8994_AIF2DACL_TO_AIF1ADC1L);
		wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);

		//R1543(607h) - 0x0002
		//AIF2DACR_TO_AIF1ADC1R
		val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
		val &= ~(WM8994_AIF2DACR_TO_AIF1ADC1R_MASK);
		val |= (WM8994_AIF2DACR_TO_AIF1ADC1R);
		wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);

		wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0019);

		wm8994_write(codec, WM8994_OVERSAMPLING, 0X0000);

		if(wm8994->testmode_config_flag == SEC_NORMAL)
		{
			//Digital Paths
			//Enable HPF Filter for google voice search
			if(wm8994->recognition_active == REC_ON
#ifdef FEATURE_VSUITE_RECOGNITION
				|| wm8994->vsuite_recognition_active == REC_ON
#endif				
			){
				val = wm8994_read(codec,WM8994_AIF1_ADC1_FILTERS );
				val &= ~(WM8994_AIF1ADC1L_HPF_MASK | WM8994_AIF1ADC1R_HPF_MASK);
				//val |= (WM8994_AIF1ADC1L_HPF | WM8994_AIF1ADC1R_HPF); //HPF off	
				wm8994_write(codec,WM8994_AIF1_ADC1_FILTERS ,val);
			}

			//AIF1_ADC1 volume
			val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_VOLUME);
			val &= ~(WM8994_AIF1ADC1L_VOL_MASK);
			if(wm8994->recognition_active == REC_ON)		
				val |= (TUNING_RECOGNITION_BT_AIF1ADC1L_VOL);
#ifdef FEATURE_VSUITE_RECOGNITION
			else if(wm8994->vsuite_recognition_active == REC_ON)
				val |= (TUNING_VSUITE_RECOGNITION_BT_AIF1ADC1L_VOL);
#endif				
			else
				val |= (TUNING_RECORD_BT_AIF1ADC1L_VOL);
			wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_VOLUME, val);

			val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_VOLUME);
			val &= ~(WM8994_AIF1ADC1R_VOL_MASK);
			if(wm8994->recognition_active == REC_ON)		
				val |= (WM8994_AIF1ADC1_VU | TUNING_RECOGNITION_BT_AIF1ADC1R_VOL);
#ifdef FEATURE_VSUITE_RECOGNITION
			else if(wm8994->vsuite_recognition_active == REC_ON)
				val |= (WM8994_AIF1ADC1_VU | TUNING_VSUITE_RECOGNITION_BT_AIF1ADC1R_VOL);	
#endif				
			else
				val |= (WM8994_AIF1ADC1_VU | TUNING_RECORD_BT_AIF1ADC1R_VOL);
			wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_VOLUME, val);
			
			//AIF2DAC volume
			val = wm8994_read(codec, WM8994_AIF2_DAC_LEFT_VOLUME);
			val &= ~(WM8994_AIF2DACL_VOL_MASK);
			if(wm8994->recognition_active == REC_ON)		
				val |= (TUNING_RECOGNITION_BT_AIF2DACL_VOL);
#ifdef FEATURE_VSUITE_RECOGNITION
			else if(wm8994->vsuite_recognition_active == REC_ON)
				val |= (TUNING_VSUITE_RECOGNITION_BT_AIF2DACL_VOL);
#endif				
			else
				val |= (TUNING_RECORD_BT_AIF2DACL_VOL);
			wm8994_write(codec, WM8994_AIF2_DAC_LEFT_VOLUME, val);

			val = wm8994_read(codec, WM8994_AIF2_DAC_RIGHT_VOLUME);
			val &= ~(WM8994_AIF2DACR_VOL_MASK);
			if(wm8994->recognition_active == REC_ON)		
				val |= (WM8994_AIF2DAC_VU | TUNING_RECOGNITION_BT_AIF2DACR_VOL);
#ifdef FEATURE_VSUITE_RECOGNITION
			else if(wm8994->vsuite_recognition_active == REC_ON)
				val |= (WM8994_AIF2DAC_VU | TUNING_VSUITE_RECOGNITION_BT_AIF2DACR_VOL);
#endif				
			else
				val |= (WM8994_AIF2DAC_VU | TUNING_RECORD_BT_AIF2DACR_VOL); 
			wm8994_write(codec, WM8994_AIF2_DAC_RIGHT_VOLUME, val);
		}
		else
		{
			//volume update (for HwCodec Tuning)
			val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_VOLUME);
			val |= WM8994_AIF1ADC1_VU;
			wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_VOLUME, val);

			val = wm8994_read(codec, WM8994_AIF2_DAC_RIGHT_VOLUME);
			val |= WM8994_AIF2DAC_VU;
			wm8994_write(codec, WM8994_AIF2_DAC_RIGHT_VOLUME, val);
		}
		
		/*GPIO Configuration*/		
		wm8994_write(codec, WM8994_GPIO_8, WM8994_GP8_DIR | WM8994_GP8_DB);
		wm8994_write(codec, WM8994_GPIO_9, WM8994_GP9_DB);
		wm8994_write(codec, WM8994_GPIO_10, WM8994_GP10_DB);
		wm8994_write(codec, WM8994_GPIO_11, WM8994_GP11_DB); 
	}
}

void wm8994_set_playback_receiver(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;

	u16 val;

	DEBUG_LOG("");

	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		val = wm8994_read(codec, WM8994_OUTPUT_MIXER_5);
		val &= ~(WM8994_DACL_MIXOUTL_VOL_MASK);
		val |= TUNING_RCV_OUTMIX5_VOL << WM8994_DACL_MIXOUTL_VOL_SHIFT;
		wm8994_write(codec,WM8994_OUTPUT_MIXER_5, val );

		val = wm8994_read(codec, WM8994_OUTPUT_MIXER_6);
		val &= ~(WM8994_DACR_MIXOUTR_VOL_MASK);
		val |= TUNING_RCV_OUTMIX6_VOL << WM8994_DACR_MIXOUTR_VOL_SHIFT;
		wm8994_write(codec,WM8994_OUTPUT_MIXER_6, val );

		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK | WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUTL_MUTE_N | TUNING_RCV_OPGAL_VOL);
		wm8994_write(codec,WM8994_LEFT_OPGA_VOLUME, val );

		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK | WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU | WM8994_MIXOUTR_MUTE_N | TUNING_RCV_OPGAR_VOL);
		wm8994_write(codec,WM8994_RIGHT_OPGA_VOLUME, val );

		val = wm8994_read(codec, WM8994_HPOUT2_VOLUME);
		val &= ~(WM8994_HPOUT2_MUTE_MASK | WM8994_HPOUT2_VOL_MASK);
		val |= TUNING_HPOUT2_VOL << WM8994_HPOUT2_VOL_SHIFT;
		wm8994_write(codec,WM8994_HPOUT2_VOLUME, val );

		//Unmute DAC1 left
		val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );
		val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL); 
		wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);

		//Unmute and volume ctrl RightDAC
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
		val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
		val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL);
		wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);

		val = wm8994_read(codec, WM8994_AIF1_DAC1_LEFT_VOLUME);
		val &= ~(WM8994_AIF1DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_LEFT_VOLUME, val);

		val = wm8994_read(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME);
		val &= ~(WM8994_AIF1DAC1R_VOL_MASK);
		val |= (WM8994_AIF1DAC1_VU |TUNING_DAC1R_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME, val);
	}

	val = wm8994_read(codec,WM8994_OUTPUT_MIXER_1);
	val &= ~(WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |= (WM8994_DAC1L_TO_MIXOUTL);
	wm8994_write(codec,WM8994_OUTPUT_MIXER_1,val);

	val = wm8994_read(codec,WM8994_OUTPUT_MIXER_2);
	val &= ~(WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= (WM8994_DAC1R_TO_MIXOUTR);
	wm8994_write(codec,WM8994_OUTPUT_MIXER_2,val);

	val = wm8994_read(codec,WM8994_HPOUT2_MIXER);
	val &= ~(WM8994_MIXOUTLVOL_TO_HPOUT2_MASK | WM8994_MIXOUTRVOL_TO_HPOUT2_MASK);
	val |= (WM8994_MIXOUTRVOL_TO_HPOUT2 | WM8994_MIXOUTLVOL_TO_HPOUT2);
	wm8994_write(codec,WM8994_HPOUT2_MIXER,val);

	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_5);
	val &= ~(WM8994_DAC1R_ENA_MASK | WM8994_DAC1L_ENA_MASK | WM8994_AIF1DAC1R_ENA_MASK | WM8994_AIF1DAC1L_ENA_MASK);
	val |= (WM8994_AIF1DAC1L_ENA | WM8994_AIF1DAC1R_ENA | WM8994_DAC1L_ENA | WM8994_DAC1R_ENA);
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_5,val);

	val = wm8994_read(codec,WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK |WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE | WM8994_AIF1DAC1_MONO);
	wm8994_write(codec,WM8994_AIF1_DAC1_FILTERS_1,val);

	val = wm8994_read(codec,WM8994_DAC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= (WM8994_AIF1DAC1L_TO_DAC1L);
	wm8994_write(codec,WM8994_DAC1_LEFT_MIXER_ROUTING,val);

	val = wm8994_read(codec,WM8994_DAC1_RIGHT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
	val |= (WM8994_AIF1DAC1R_TO_DAC1R);
	wm8994_write(codec,WM8994_DAC1_RIGHT_MIXER_ROUTING,val);

	val = wm8994_read(codec, WM8994_CLOCKING_1);
	val &= ~(WM8994_DSP_FS1CLK_ENA_MASK | WM8994_DSP_FSINTCLK_ENA_MASK);
	val |= (WM8994_DSP_FS1CLK_ENA | WM8994_DSP_FSINTCLK_ENA);
	wm8994_write(codec, WM8994_CLOCKING_1, val);

	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_MIXOUTLVOL_ENA_MASK | WM8994_MIXOUTRVOL_ENA_MASK | WM8994_MIXOUTL_ENA_MASK | WM8994_MIXOUTR_ENA_MASK);
	val |= (WM8994_MIXOUTL_ENA | WM8994_MIXOUTR_ENA | WM8994_MIXOUTRVOL_ENA | WM8994_MIXOUTLVOL_ENA);
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_3,val);

	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1 );
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK | WM8994_HPOUT2_ENA_MASK );
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL | WM8994_HPOUT2_ENA );
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_1,val);
}


void wm8994_set_playback_headset(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;

	u16 val;

	u16 TestReturn1=0;
	u16 TestReturn2=0;
	u16 TestLow1=0;
	u16 TestHigh1=0;
	u8 TestLow=0;
	u8 TestHigh=0;

	DEBUG_LOG("");

	//Configuring the Digital Paths

	// Enable the Timeslot0 to DAC1L
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING  ); 	
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= WM8994_AIF1DAC1L_TO_DAC1L;
	wm8994_write(codec,WM8994_DAC1_LEFT_MIXER_ROUTING ,val);

	//Enable the Timeslot0 to DAC1R
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING  ); 	
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
	val |= WM8994_AIF1DAC1R_TO_DAC1R;
	wm8994_write(codec,WM8994_DAC1_RIGHT_MIXER_ROUTING ,val);

	val = wm8994_read(codec, 0x102 ); 	
	val &= ~(0x0003);
	val = 0x0003;
	wm8994_write(codec,0x102,val);

	val = wm8994_read(codec, 0x56 ); 	
	val &= ~(0x0003);
	val = 0x0003;
	wm8994_write(codec,0x56,val);

	val = wm8994_read(codec, 0x102 ); 	
	val &= ~(0x0000);
	val = 0x0000;
	wm8994_write(codec,0x102,val);

	val = wm8994_read(codec, WM8994_CLASS_W_1  ); 	
	val &= ~(0x0005);
	val |= 0x0005;
	wm8994_write(codec,WM8994_CLASS_W_1,val);

	// Headset Control
	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		val = wm8994_read(codec,WM8994_LEFT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK | WM8994_HPOUT1L_VOL_MASK);
		if(wm8994->ringtone_active)
		val |= (WM8994_HPOUT1L_MUTE_N | TUNING_RING_OUTPUTL_VOL);
		else
		val |= (WM8994_HPOUT1L_MUTE_N | TUNING_MP3_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);


		val = wm8994_read(codec,WM8994_RIGHT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK | WM8994_HPOUT1R_VOL_MASK);
		if(wm8994->ringtone_active)
		val |= (WM8994_HPOUT1_VU | WM8994_HPOUT1R_MUTE_N | TUNING_RING_OUTPUTR_VOL);
		else
		val |= (WM8994_HPOUT1_VU | WM8994_HPOUT1R_MUTE_N | TUNING_MP3_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);


		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK | WM8994_MIXOUTL_VOL_MASK);
		if(wm8994->ringtone_active)
		val |= (WM8994_MIXOUTL_MUTE_N | TUNING_RING_OPGAL_VOL);
		else
		val |= (WM8994_MIXOUTL_MUTE_N | TUNING_MP3_OPGAL_VOL);
		wm8994_write(codec,WM8994_LEFT_OPGA_VOLUME, val );

		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK | WM8994_MIXOUTR_VOL_MASK);
		if(wm8994->ringtone_active)
		val |= (WM8994_MIXOUT_VU | WM8994_MIXOUTR_MUTE_N | TUNING_RING_OPGAR_VOL);
		else
		val |= (WM8994_MIXOUT_VU | WM8994_MIXOUTR_MUTE_N | TUNING_MP3_OPGAR_VOL);
		wm8994_write(codec,WM8994_RIGHT_OPGA_VOLUME, val);

		val = wm8994_read(codec, WM8994_AIF1_DAC1_LEFT_VOLUME);
		val &= ~(WM8994_AIF1DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_LEFT_VOLUME, val);

		val = wm8994_read(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME);
		val &= ~(WM8994_AIF1DAC1R_VOL_MASK);
		val |= (WM8994_AIF1DAC1_VU |TUNING_DAC1R_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME, val);
	}

	val = wm8994_read(codec, WM8994_DC_SERVO_2  ); 	
	val &= ~(0x03E0);
	val = 0x03E0;
	wm8994_write(codec,WM8994_DC_SERVO_2,val);

	//Enable vmid,bias, hp left and right
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1 );
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK |WM8994_HPOUT1L_ENA_MASK |WM8994_HPOUT1R_ENA_MASK | WM8994_SPKOUTL_ENA_MASK | WM8994_SPKOUTR_ENA_MASK );
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL | WM8994_HPOUT1R_ENA |WM8994_HPOUT1L_ENA);  
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_1,val);

	val = wm8994_read(codec, WM8994_ANALOGUE_HP_1  ); 	
	val &= ~(0x0022);
	val = 0x0022;
	wm8994_write(codec,WM8994_ANALOGUE_HP_1,val);

	//Enable Charge Pump	
	val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
	val &= ~WM8994_CP_ENA_MASK ;
	val |= WM8994_CP_ENA | WM8994_CP_ENA_DEFAULT ; // this is from wolfson  	
	wm8994_write(codec, WM8994_CHARGE_PUMP_1, val);

	msleep(5);

	//Enable Dac1 and DAC2 and the Timeslot0 for AIF1	
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5 ); 	
	val &= ~(WM8994_DAC1R_ENA_MASK | WM8994_DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK |  WM8994_AIF1DAC1L_ENA_MASK );
	val |= (WM8994_AIF1DAC1L_ENA | WM8994_AIF1DAC1R_ENA  | WM8994_DAC1L_ENA |WM8994_DAC1R_ENA );
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5 ,val);

	// Enable DAC1L to HPOUT1L path
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
	val &=  ~(WM8994_DAC1L_TO_HPOUT1L_MASK | WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |= WM8994_DAC1L_TO_MIXOUTL ;  	
	wm8994_write(codec,WM8994_OUTPUT_MIXER_1, val);	// Enable MIXOUT

	// Enable DAC1R to HPOUT1R path
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
	val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK | WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= WM8994_DAC1R_TO_MIXOUTR;
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_MIXOUTLVOL_ENA_MASK | WM8994_MIXOUTRVOL_ENA_MASK | WM8994_MIXOUTL_ENA_MASK | WM8994_MIXOUTR_ENA_MASK);
	val |= (WM8994_MIXOUTLVOL_ENA | WM8994_MIXOUTRVOL_ENA | WM8994_MIXOUTL_ENA | WM8994_MIXOUTR_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, 0x0030);

	val = wm8994_read(codec, WM8994_DC_SERVO_1  ); 	
	val &= ~(0x0303);
	val = 0x0303;
	wm8994_write(codec,WM8994_DC_SERVO_1,val);

	msleep(160);	// 160ms delay

	TestReturn1=wm8994_read(codec,WM8994_DC_SERVO_4);

	TestLow=(signed char)(TestReturn1 & 0xff);
	TestHigh=(signed char)((TestReturn1>>8) & 0xff);

	TestLow1=((signed short)(TestLow-5))&0x00ff;
	TestHigh1=(((signed short)(TestHigh-5)<<8)&0xff00);
	TestReturn2=TestLow1|TestHigh1;
	wm8994_write(codec,WM8994_DC_SERVO_4, TestReturn2);

	val = wm8994_read(codec, WM8994_DC_SERVO_1  ); 	
	val &= ~(0x000F);
	val = 0x000F;
	wm8994_write(codec,WM8994_DC_SERVO_1,val);

	msleep(20);

	// Intermediate HP settings
	val = wm8994_read(codec, WM8994_ANALOGUE_HP_1); 	
	val &= ~(WM8994_HPOUT1R_DLY_MASK |WM8994_HPOUT1R_OUTP_MASK |WM8994_HPOUT1R_RMV_SHORT_MASK |
		WM8994_HPOUT1L_DLY_MASK |WM8994_HPOUT1L_OUTP_MASK | WM8994_HPOUT1L_RMV_SHORT_MASK);
	val = (WM8994_HPOUT1L_RMV_SHORT | WM8994_HPOUT1L_OUTP|WM8994_HPOUT1L_DLY |WM8994_HPOUT1R_RMV_SHORT | 
		WM8994_HPOUT1R_OUTP | WM8994_HPOUT1R_DLY);
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	//Unmute DAC1 left
	val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );
	val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL); 
	wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);

	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
	val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume	
	wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);	

	// Unmute the AF1DAC1	
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1 ); 	
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK | WM8994_AIF1DAC1_MONO_MASK);
	val |= WM8994_AIF1DAC1_UNMUTE;
	wm8994_write(codec,WM8994_AIF1_DAC1_FILTERS_1 ,val);	
}

void wm8994_set_playback_speaker(struct snd_soc_codec *codec)
{
#ifdef CONFIG_TARGET_LOCALE_KOR
	struct wm8994_priv *wm8994 = codec->drvdata;
#endif

	u16 val;

	DEBUG_LOG("");

	//Disable end point for preventing pop up noise.
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_SPKOUTL_ENA_MASK | WM8994_SPKOUTR_ENA_MASK | WM8994_HPOUT1L_ENA_MASK | WM8994_HPOUT1R_ENA_MASK);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~( WM8994_SPKLVOL_ENA_MASK | WM8994_SPKRVOL_ENA_MASK  );
	val |= ( WM8994_SPKLVOL_ENA | WM8994_SPKRVOL_ENA );
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

	// Speaker Volume Control
	// Unmute the SPKMIXVOLUME
	val = wm8994_read(codec, WM8994_SPKMIXL_ATTENUATION);
	val &= ~(WM8994_SPKMIXL_VOL_MASK);
	val |= TUNING_SPKMIXL_ATTEN;	
	wm8994_write(codec, WM8994_SPKMIXL_ATTENUATION, val);

	val = wm8994_read(codec,WM8994_SPKMIXR_ATTENUATION);
	val &= ~(WM8994_SPKMIXR_VOL_MASK);
	val |= TUNING_SPKMIXR_ATTEN;	
	wm8994_write(codec, WM8994_SPKMIXR_ATTENUATION, val);

	val = wm8994_read(codec,WM8994_SPEAKER_VOLUME_LEFT );
	val &= ~(WM8994_SPKOUT_VU_MASK | WM8994_SPKOUTL_MUTE_N_MASK | WM8994_SPKOUTL_VOL_MASK);
	val |=(WM8994_SPKOUT_VU  | WM8994_SPKOUTL_MUTE_N | TUNING_MP3_SPKL_VOL );
	wm8994_write(codec,WM8994_SPEAKER_VOLUME_LEFT ,val);

	val = wm8994_read(codec,WM8994_SPEAKER_VOLUME_RIGHT );
	val &= ~(WM8994_SPKOUT_VU_MASK | WM8994_SPKOUTR_MUTE_N_MASK | WM8994_SPKOUTR_VOL_MASK);
	val |=(WM8994_SPKOUT_VU  | WM8994_SPKOUTR_MUTE_N | TUNING_MP3_SPKR_VOL );
	wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT ,val);

	/* Tunning Mobile Parameteric Equalizer 2010.05.13 */
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_GAINS_1, 0x3319); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_GAINS_2, 0x6300);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_1_A, 0x0F71); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_1_B, 0x0403);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_1_PG, 0x023C); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_2_A, 0x1EB5);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_2_B, 0xF145); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_2_C, 0x0B75);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_2_PG, 0x01C5); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_3_A, 0x1C58);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_3_B, 0xF373); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_3_C, 0x0A54);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_3_PG, 0x0558); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_4_A, 0x168E);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_4_B, 0xF829); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_4_C, 0x07AD);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_4_PG, 0x1103); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_5_A, 0xFA14);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_5_B, 0x0400); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_5_PG, 0x2850);

#ifdef CONFIG_TARGET_LOCALE_VZW
	wm8994_write(codec, WM8994_CLASSD, 0x0164);	
#else
	val = wm8994_read(codec,WM8994_CLASSD );
	val &= ~(0x3F);
	val |= 0x24;
	wm8994_write(codec,WM8994_CLASSD ,val); 

#endif

	//Unmute DAC1 left
	val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );
	val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
	val |= (WM8994_DAC1_VU | TUNING_DAC1L_VOL); //0 db volume	
	wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);

	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
	val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume	
	wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);

	val = wm8994_read(codec, WM8994_AIF1_DAC1_LEFT_VOLUME);
	val &= ~(WM8994_AIF1DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL);
	wm8994_write(codec, WM8994_AIF1_DAC1_LEFT_VOLUME, val);

	val = wm8994_read(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME);
	val &= ~(WM8994_AIF1DAC1R_VOL_MASK);
	val |= (WM8994_AIF1DAC1_VU |TUNING_DAC1R_VOL);
	wm8994_write(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME, val);

	val = wm8994_read(codec,WM8994_SPKOUT_MIXERS);
	val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | 
	WM8994_SPKMIXR_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
	val |= (WM8994_SPKMIXL_TO_SPKOUTL | WM8994_SPKMIXR_TO_SPKOUTR );
	wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);

	//Unmute the DAC path
	val = wm8994_read(codec,WM8994_SPEAKER_MIXER);
	val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK | WM8994_DAC1R_TO_SPKMIXR_MASK);
	val |= (WM8994_DAC1L_TO_SPKMIXL | WM8994_DAC1R_TO_SPKMIXR ) ;
	wm8994_write(codec, WM8994_SPEAKER_MIXER, val);

	// Eable DAC1 Left and timeslot left
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_5); 
	val &= ~( WM8994_DAC1L_ENA_MASK | WM8994_AIF1DAC1L_ENA_MASK | WM8994_DAC1R_ENA_MASK | WM8994_AIF1DAC1R_ENA_MASK  );
	val |= (WM8994_AIF1DAC1L_ENA | WM8994_DAC1L_ENA | WM8994_AIF1DAC1R_ENA | WM8994_DAC1R_ENA );
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);   

	//Unmute
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK | WM8994_AIF1DAC1_MONO_MASK);
	val |= WM8994_AIF1DAC1_UNMUTE;
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);

	//enable timeslot0 to left dac
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= WM8994_AIF1DAC1L_TO_DAC1L;
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);

	val = wm8994_read(codec,WM8994_DAC1_RIGHT_MIXER_ROUTING );
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK );
	val |= WM8994_AIF1DAC1R_TO_DAC1R;
	wm8994_write(codec,WM8994_DAC1_RIGHT_MIXER_ROUTING ,val);

#ifdef CONFIG_SND_VOODOO
	voodoo_hook_playback_speaker();
#endif

	//Enbale bias,vmid and Left speaker
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK |WM8994_SPKOUTL_ENA_MASK | WM8994_SPKOUTR_ENA_MASK );
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL |	WM8994_SPKOUTL_ENA | WM8994_SPKOUTR_ENA  );  
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

#ifdef CONFIG_TARGET_LOCALE_KOR
	if(wm8994->testmode_config_flag == SEC_TEST_15MODE)
	{
		//Speaker Volume Left
		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
		val &= ~(WM8994_SPKOUTL_MUTE_N_MASK | WM8994_SPKOUTL_VOL_MASK);
		val |= 0x0000;
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);

		// Speaker Volume Right
		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_RIGHT);
		val &= ~(WM8994_SPKOUTR_MUTE_N_MASK | WM8994_SPKOUTR_VOL_MASK);
		val |= (WM8994_SPKOUT_VU | WM8994_SPKOUTR_MUTE_N | TUNING_MP3_SPKR_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);

		// Speaker Output Mixer
 		val = wm8994_read(codec, WM8994_SPKOUT_MIXERS );
		val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTL_MASK 
			    | WM8994_SPKMIXL_TO_SPKOUTR_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
 		val |= WM8994_SPKMIXR_TO_SPKOUTR;
 		wm8994_write(codec, WM8994_SPKOUT_MIXERS, val );
	}
#endif	
}


void wm8994_set_playback_speaker_headset(struct snd_soc_codec *codec)
{

	struct wm8994_priv *wm8994 = codec->drvdata;

	u16 val;

	u16 nReadServo4Val = 0;
	u16 ncompensationResult = 0;
	u16 nCompensationResultLow=0;
	u16 nCompensationResultHigh=0;
	u8  nServo4Low = 0;
	u8  nServo4High = 0;

	//------------------  Common Settings ------------------
	// Enable the Timeslot0 to DAC1L
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING	);	   
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= WM8994_AIF1DAC1L_TO_DAC1L;
	wm8994_write(codec,WM8994_DAC1_LEFT_MIXER_ROUTING ,val);

	//Enable the Timeslot0 to DAC1R
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING  );    
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
	val |= WM8994_AIF1DAC1R_TO_DAC1R;
	wm8994_write(codec,WM8994_DAC1_RIGHT_MIXER_ROUTING ,val);


	/* Tunning Mobile Parameteric Equalizer 2010.05.13 */
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_GAINS_1, 0x3319); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_GAINS_2, 0x6300);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_1_A, 0x0F71); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_1_B, 0x0403);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_1_PG, 0x023C); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_2_A, 0x1EB5);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_2_B, 0xF145); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_2_C, 0x0B75);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_2_PG, 0x01C5); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_3_A, 0x1C58);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_3_B, 0xF373); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_3_C, 0x0A54);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_3_PG, 0x0558); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_4_A, 0x168E);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_4_B, 0xF829); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_4_C, 0x07AD);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_4_PG, 0x1103); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_5_A, 0xFA14);
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_5_B, 0x0400); 
	wm8994_write( codec,WM8994_AIF1_DAC1_EQ_BAND_5_PG, 0x2850);


	val = wm8994_read(codec, WM8994_AIF1_DAC1_LEFT_VOLUME);
	val &= ~(WM8994_AIF1DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL);
	wm8994_write(codec, WM8994_AIF1_DAC1_LEFT_VOLUME, val);

	val = wm8994_read(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME);
	val &= ~(WM8994_AIF1DAC1R_VOL_MASK);
	val |= (WM8994_AIF1DAC1_VU | TUNING_DAC1R_VOL);
	wm8994_write(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME, val);

	//------------------  Speaker Path Settings ------------------

	// Speaker Volume Control
	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		val = wm8994_read(codec,WM8994_SPEAKER_VOLUME_LEFT);
		val &= ~(WM8994_SPKOUTL_MUTE_N_MASK | WM8994_SPKOUTL_VOL_MASK);		   
		val |= (WM8994_SPKOUTL_MUTE_N | TUNING_MP3_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);

		val = wm8994_read(codec,WM8994_SPEAKER_VOLUME_RIGHT);
		val &= ~(WM8994_SPKOUTR_MUTE_N_MASK | WM8994_SPKOUTR_VOL_MASK);
		val |= (WM8994_SPKOUT_VU | WM8994_SPKOUTL_MUTE_N | TUNING_MP3_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);

		val = wm8994_read(codec,WM8994_CLASSD );
		val &= ~(0x3F);
		val |= 0x24;
		wm8994_write(codec,WM8994_CLASSD ,val);
	}


	val = wm8994_read(codec,WM8994_SPKOUT_MIXERS );
	val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | 
	WM8994_SPKMIXR_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
	val |= WM8994_SPKMIXL_TO_SPKOUTL;
	val |= WM8994_SPKMIXR_TO_SPKOUTR;
	wm8994_write(codec,WM8994_SPKOUT_MIXERS,val );

	//Unmute the DAC path
	val = wm8994_read(codec,WM8994_SPEAKER_MIXER );
	val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK | WM8994_DAC1R_TO_SPKMIXR_MASK);
	val |= ( WM8994_DAC1L_TO_SPKMIXL | WM8994_DAC1R_TO_SPKMIXR );
	wm8994_write(codec,WM8994_SPEAKER_MIXER ,val);

	//------------------  Ear Path Settings ------------------
	//Configuring the Digital Paths
	val = wm8994_read(codec, 0x102);
	val &= ~(0x0003);
	val = 0x0003;
	wm8994_write(codec,0x102,val);

	val = wm8994_read(codec, 0x56  );   
	val &= ~(0x0003);
	val = 0x0003;
	wm8994_write(codec,0x56,val);

	val = wm8994_read(codec, 0x102  );  
	val &= ~(0x0000);
	val = 0x0000;
	wm8994_write(codec,0x102,val);

	val = wm8994_read(codec, WM8994_CLASS_W_1  );   
	val &= ~(0x0005);
	val = 0x0005;
	wm8994_write(codec,WM8994_CLASS_W_1,val);

	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		val = wm8994_read(codec,WM8994_LEFT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK | WM8994_HPOUT1L_VOL_MASK);
		val |= (WM8994_HPOUT1L_MUTE_N | TUNING_MP3_DUAL_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec,WM8994_RIGHT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK | WM8994_HPOUT1R_VOL_MASK);
		val |= (WM8994_HPOUT1_VU | WM8994_HPOUT1R_MUTE_N | TUNING_MP3_DUAL_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);
	}

	//* DC Servo Series Count
	val = 0x03E0;
	wm8994_write(codec, WM8994_DC_SERVO_2, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK |
		WM8994_HPOUT1L_ENA_MASK | WM8994_HPOUT1R_ENA_MASK | WM8994_SPKOUTL_ENA_MASK | WM8994_SPKOUTR_ENA_MASK );
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL |
	WM8994_HPOUT1R_ENA	| WM8994_HPOUT1L_ENA | WM8994_SPKOUTL_ENA | WM8994_SPKOUTR_ENA );
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_1,val);

	val = (WM8994_HPOUT1L_DLY | WM8994_HPOUT1R_DLY);
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	//Enable Charge Pump    
	val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
	val &= ~WM8994_CP_ENA_MASK ;
	val |= WM8994_CP_ENA | WM8994_CP_ENA_DEFAULT ; // this is from wolfson	   
	wm8994_write(codec, WM8994_CHARGE_PUMP_1, val);

	msleep(5);

	//Enable DAC1 and DAC2 and the Timeslot0 for AIF1   
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_5 );    
	val &= ~(WM8994_DAC1R_ENA_MASK | WM8994_DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK |  WM8994_AIF1DAC1L_ENA_MASK );
	val |= (WM8994_AIF1DAC1L_ENA | WM8994_AIF1DAC1R_ENA	| WM8994_DAC1L_ENA |WM8994_DAC1R_ENA );
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_5,val);	

	// Enbale DAC1L to HPOUT1L path
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
	val &=  ~(WM8994_DAC1L_TO_HPOUT1L_MASK | WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |=  WM8994_DAC1L_TO_MIXOUTL;
	wm8994_write(codec,WM8994_OUTPUT_MIXER_1, val);

	// Enbale DAC1R to HPOUT1R path
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
	val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK | WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= WM8994_DAC1R_TO_MIXOUTR;
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

	//Enbale bias,vmid, hp left and right and Left speaker
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_MIXOUTLVOL_ENA_MASK | WM8994_MIXOUTRVOL_ENA_MASK | WM8994_MIXOUTL_ENA_MASK | WM8994_MIXOUTR_ENA_MASK | WM8994_SPKLVOL_ENA_MASK);
	val |= (WM8994_MIXOUTLVOL_ENA | WM8994_MIXOUTRVOL_ENA | WM8994_MIXOUTL_ENA | WM8994_MIXOUTR_ENA | WM8994_SPKLVOL_ENA | WM8994_SPKRVOL_ENA );
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3 ,val);

	//* DC Servo 
	val = (WM8994_DCS_TRIG_SERIES_1 | WM8994_DCS_TRIG_SERIES_0 | WM8994_DCS_ENA_CHAN_1 | WM8994_DCS_ENA_CHAN_0);
	wm8994_write(codec,WM8994_DC_SERVO_1, 0x0303 );

	msleep(160);

	nReadServo4Val=wm8994_read(codec,WM8994_DC_SERVO_4);
	nServo4Low=(signed char)(nReadServo4Val & 0xff);
	nServo4High=(signed char)((nReadServo4Val>>8) & 0xff);

	nCompensationResultLow=((signed short)nServo4Low -5)&0x00ff;
	nCompensationResultHigh=((signed short)(nServo4High -5)<<8)&0xff00;
	ncompensationResult=nCompensationResultLow|nCompensationResultHigh;
	wm8994_write(codec,WM8994_DC_SERVO_4, ncompensationResult);

	val = (WM8994_DCS_TRIG_DAC_WR_1 | WM8994_DCS_TRIG_DAC_WR_0 | WM8994_DCS_ENA_CHAN_1 | WM8994_DCS_ENA_CHAN_0);
	wm8994_write(codec,WM8994_DC_SERVO_1, val );

	msleep(15); 

	// Intermediate HP settings
	val = wm8994_read(codec, WM8994_ANALOGUE_HP_1);	   
	val &= ~(WM8994_HPOUT1R_DLY_MASK |WM8994_HPOUT1R_OUTP_MASK |WM8994_HPOUT1R_RMV_SHORT_MASK |
	WM8994_HPOUT1L_DLY_MASK |WM8994_HPOUT1L_OUTP_MASK | WM8994_HPOUT1L_RMV_SHORT_MASK);
	val |= (WM8994_HPOUT1L_RMV_SHORT | WM8994_HPOUT1L_OUTP|WM8994_HPOUT1L_DLY |WM8994_HPOUT1R_RMV_SHORT | 
	WM8994_HPOUT1R_OUTP | WM8994_HPOUT1R_DLY);
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		//Unmute DAC1 left
		val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );
		val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL); 
		wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);

		//Unmute and volume ctrl RightDAC
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
		val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
		val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume  
		wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);
	}

	//------------------  Common Settings ------------------
	// Unmute the AIF1DAC1  
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1 );  
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK | WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE | WM8994_AIF1DAC1_MONO);
	wm8994_write(codec,WM8994_AIF1_DAC1_FILTERS_1 ,val);


	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		// Unmute the SPKMIXVOLUME
		val = wm8994_read(codec, WM8994_SPKMIXL_ATTENUATION);
		val &= ~(WM8994_SPKMIXL_VOL_MASK);
		val |= TUNING_SPKMIXL_ATTEN;    
		wm8994_write(codec, WM8994_SPKMIXL_ATTENUATION, val);

		val = wm8994_read(codec,WM8994_SPKMIXR_ATTENUATION);
		val &= ~(WM8994_SPKMIXR_VOL_MASK);
		val |= TUNING_SPKMIXR_ATTEN;
		wm8994_write(codec, WM8994_SPKMIXR_ATTENUATION, val);
	}
}


void wm8994_set_playback_bluetooth(struct snd_soc_codec *codec)
{
	u16 val;

	DEBUG_LOG("BT Playback Path for Voice Command");


	wm8994_set_voicecall_common_setting(codec);

	//R1(1h) - 0x0003 -normal vmid
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK);
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL);	
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);
	
	/*Digital Path Enables and Unmutes*/	
	//R2(2h) - 0x0000
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, 0x0000);

	//R3(3h) - 0x0000
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, 0x0000);

	//R4(4h) - 0x3000
	//AIF2ADC(Left/Right) Output
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4 );
	val &= ~(WM8994_AIF2ADCL_ENA_MASK |WM8994_AIF2ADCR_ENA_MASK);
	val |= ( WM8994_AIF2ADCL_ENA | WM8994_AIF2ADCR_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4 , val);

	//R5(5h) - 0x0300
	//AIF1DAC1(Left/Right) Input
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);	//05 : 3303
	val &= ~(WM8994_AIF2DACL_ENA_MASK | WM8994_AIF2DACR_ENA_MASK | WM8994_AIF1DAC1L_ENA_MASK | WM8994_AIF1DAC1R_ENA_MASK
		 | WM8994_DAC1L_ENA_MASK | WM8994_DAC1R_ENA_MASK);
	val |= (WM8994_AIF2DACL_ENA | WM8994_AIF2DACR_ENA | WM8994_AIF1DAC1L_ENA | WM8994_AIF1DAC1R_ENA
		 | WM8994_DAC1L_ENA | WM8994_DAC1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);
	
	//R6(6h) - 0x0008
	//AIF3_ADCDAT_SRC(AIF2ADCDAT2)
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_6);
	val &= ~(WM8994_AIF3_ADCDAT_SRC_MASK);
	val |= (0x0001 << WM8994_AIF3_ADCDAT_SRC_SHIFT);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_6, val);

	/*Un-Mute*/
	//R1056(420h) - 0x0000
	//AIF1DAC1_MUTE(Un-mute)
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK | WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE);
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);  

	/*Mixer Routing*/
	//R1540(604h) - 0x0004
	//AIF2DACL_TO_DAC2L
	val = wm8994_read(codec, WM8994_DAC2_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC2L_MASK);
	val |= (WM8994_AIF1DAC1L_TO_DAC2L);
	wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, val);
	
	//R1541(605h) - 0x0004
	//AIF2DACR_TO_DAC2R
	val = wm8994_read(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1R_TO_DAC2R_MASK);
	val |= (WM8994_AIF1DAC1R_TO_DAC2R);
	wm8994_write(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING, val);

	/*Volume*/
	wm8994_write(codec, WM8994_DAC2_LEFT_VOLUME, 0x00C0);
	wm8994_write(codec, WM8994_DAC2_RIGHT_VOLUME, 0x01C0); 

	wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0019);

	wm8994_write(codec, WM8994_OVERSAMPLING, 0X0000);

	/*GPIO Configuration*/		
	wm8994_write(codec, WM8994_GPIO_8, WM8994_GP8_DIR | WM8994_GP8_DB);
	wm8994_write(codec, WM8994_GPIO_9, WM8994_GP9_DB);
	wm8994_write(codec, WM8994_GPIO_10, WM8994_GP10_DB);
	wm8994_write(codec, WM8994_GPIO_11, WM8994_GP11_DB); 
}

void wm8994_set_playback_extra_dock_speaker(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;

	u16 val;

	DEBUG_LOG("");

	wm8994_write(codec, WM8994_ANTIPOP_2, 0x0048);
       
	//OUTPUT mute
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_LINEOUT2N_ENA_MASK | WM8994_LINEOUT2P_ENA_MASK);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);


	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		val = wm8994_read(codec, WM8994_OUTPUT_MIXER_5);
		val &= ~(WM8994_DACL_MIXOUTL_VOL_MASK);
		val |= TUNING_EXTRA_DOCK_SPK_OUTMIX5_VOL << WM8994_DACL_MIXOUTL_VOL_SHIFT;
		wm8994_write(codec,WM8994_OUTPUT_MIXER_5, val );

		val = wm8994_read(codec, WM8994_OUTPUT_MIXER_6);
		val &= ~(WM8994_DACR_MIXOUTR_VOL_MASK);
		val |= TUNING_EXTRA_DOCK_SPK_OUTMIX6_VOL << WM8994_DACR_MIXOUTR_VOL_SHIFT;
		wm8994_write(codec,WM8994_OUTPUT_MIXER_6, val );

		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK | WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUTL_MUTE_N | TUNING_MP3_EXTRA_DOCK_SPK_OPGAL_VOL);
		wm8994_write(codec,WM8994_LEFT_OPGA_VOLUME, val );

		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK | WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU | WM8994_MIXOUTR_MUTE_N | TUNING_MP3_EXTRA_DOCK_SPK_OPGAR_VOL);
		wm8994_write(codec,WM8994_RIGHT_OPGA_VOLUME, val );

		val = wm8994_read(codec,WM8994_LINE_OUTPUTS_VOLUME);
		val &= ~(WM8994_LINEOUT2_VOL_MASK);
		val |= (TUNING_MP3_EXTRA_DOCK_SPK_VOL << WM8994_LINEOUT2_VOL_SHIFT);		
		wm8994_write(codec,WM8994_LINE_OUTPUTS_VOLUME,val);	

		//Unmute DAC1 left
		val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );
		val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL);
		wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);

		//Unmute and volume ctrl RightDAC
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
		val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
		val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL);
		wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);

		val = wm8994_read(codec, WM8994_AIF1_DAC1_LEFT_VOLUME);
		val &= ~(WM8994_AIF1DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_LEFT_VOLUME, val);

		val = wm8994_read(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME);
		val &= ~(WM8994_AIF1DAC1R_VOL_MASK);
		val |= (WM8994_AIF1DAC1_VU |TUNING_DAC1R_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME, val);
	}

	val = wm8994_read(codec,WM8994_OUTPUT_MIXER_1);
	val &= ~(WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |= (WM8994_DAC1L_TO_MIXOUTL);
	wm8994_write(codec,WM8994_OUTPUT_MIXER_1,val);

	val = wm8994_read(codec,WM8994_OUTPUT_MIXER_2);
	val &= ~(WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= (WM8994_DAC1R_TO_MIXOUTR);
	wm8994_write(codec,WM8994_OUTPUT_MIXER_2,val);


	val = wm8994_read(codec,WM8994_LINE_MIXER_2);
	val &= ~(WM8994_MIXOUTR_TO_LINEOUT2N_MASK | WM8994_MIXOUTL_TO_LINEOUT2N_MASK | WM8994_LINEOUT2_MODE_MASK | WM8994_MIXOUTR_TO_LINEOUT2P_MASK);
	val |= (WM8994_MIXOUTL_TO_LINEOUT2N | WM8994_LINEOUT2_MODE | WM8994_MIXOUTR_TO_LINEOUT2P);
	wm8994_write(codec,WM8994_LINE_MIXER_2,val);

	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_5);
	val &= ~(WM8994_DAC1R_ENA_MASK | WM8994_DAC1L_ENA_MASK | WM8994_AIF1DAC1R_ENA_MASK | WM8994_AIF1DAC1L_ENA_MASK);
	val |= (WM8994_AIF1DAC1L_ENA | WM8994_AIF1DAC1R_ENA | WM8994_DAC1L_ENA | WM8994_DAC1R_ENA);
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_5,val);

	val = wm8994_read(codec,WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK |WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE);
	wm8994_write(codec,WM8994_AIF1_DAC1_FILTERS_1,val);

	val = wm8994_read(codec,WM8994_LINE_OUTPUTS_VOLUME);
	val &= ~(WM8994_LINEOUT2N_MUTE_MASK | WM8994_LINEOUT2P_MUTE_MASK);
	wm8994_write(codec,WM8994_LINE_OUTPUTS_VOLUME,val);

	val = wm8994_read(codec,WM8994_DAC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= (WM8994_AIF1DAC1L_TO_DAC1L);
	wm8994_write(codec,WM8994_DAC1_LEFT_MIXER_ROUTING,val);

	val = wm8994_read(codec,WM8994_DAC1_RIGHT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
	val |= (WM8994_AIF1DAC1R_TO_DAC1R);
	wm8994_write(codec,WM8994_DAC1_RIGHT_MIXER_ROUTING,val);

	val = wm8994_read(codec, WM8994_CLOCKING_1);
	val &= ~(WM8994_DSP_FS1CLK_ENA_MASK | WM8994_DSP_FSINTCLK_ENA_MASK);
	val |= (WM8994_DSP_FS1CLK_ENA | WM8994_DSP_FSINTCLK_ENA);
	wm8994_write(codec, WM8994_CLOCKING_1, val);

	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_MIXOUTLVOL_ENA_MASK | WM8994_MIXOUTRVOL_ENA_MASK | WM8994_MIXOUTL_ENA_MASK | WM8994_MIXOUTR_ENA_MASK);
	val |= (WM8994_MIXOUTL_ENA | WM8994_MIXOUTR_ENA | WM8994_MIXOUTRVOL_ENA | WM8994_MIXOUTLVOL_ENA);
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_3,val);

	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1 );
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK | WM8994_HPOUT2_ENA_MASK  | WM8994_HPOUT1L_ENA_MASK | WM8994_HPOUT1R_ENA_MASK 
		| WM8994_SPKOUTL_ENA_MASK | WM8994_SPKOUTR_ENA_MASK);
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL );
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_1,val);
	
	/* To enhance THD, Ch Seperation Characteristic (2010.08.19, KDS)*/
	val = wm8994_read(codec,WM8994_ADDITIONAL_CONTROL );
	val &= ~(WM8994_LINEOUT1_FB_MASK | WM8994_LINEOUT2_FB_MASK );
	/* To tune THD, Power of each frequency (2010.09.15, KDS)*/
	// val |= WM8994_LINEOUT2_FB ;
	wm8994_write(codec,WM8994_ADDITIONAL_CONTROL,val);
	
	/* To tune THD, Power of each frequency (2010.09.15, KDS)*/
	val = wm8994_read(codec,WM8994_ANTIPOP_1);
	val &= ~(WM8994_LINEOUT_VMID_BUF_ENA_MASK | WM8994_HPOUT2_IN_ENA_MASK | WM8994_LINEOUT1_DISCH_MASK | WM8994_LINEOUT2_DISCH_MASK);
	val |= WM8994_LINEOUT_VMID_BUF_ENA ;
	wm8994_write(codec,WM8994_ANTIPOP_1,val);

	msleep(230);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_LINEOUT2N_ENA_MASK | WM8994_LINEOUT2P_ENA_MASK);
	val |= (WM8994_LINEOUT2N_ENA | WM8994_LINEOUT2P_ENA);    
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

}


void wm8994_set_playback_hdmi_tvout(struct snd_soc_codec *codec)
{
	int val;
	
	DEBUG_LOG("");

	//Disable all output
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_HPOUT2_ENA_MASK | WM8994_HPOUT1L_ENA_MASK | WM8994_HPOUT1R_ENA_MASK
		| WM8994_SPKOUTL_ENA_MASK | WM8994_SPKOUTR_ENA_MASK);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_LINEOUT2N_ENA_MASK | WM8994_LINEOUT2P_ENA_MASK);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);
	
	wm8994_write(codec, WM8994_AIF1_BCLK, 0x70);
}


void wm8994_set_playback_speaker_hdmitvout(struct snd_soc_codec *codec)
{
	DEBUG_LOG("");
	
	wm8994_write(codec, WM8994_AIF1_BCLK, 0x70);
	
	wm8994_set_playback_speaker(codec);
}


void wm8994_set_playback_speakerheadset_hdmitvout(struct snd_soc_codec *codec)
{
	DEBUG_LOG("");
	
	wm8994_write(codec, WM8994_AIF1_BCLK, 0x70);	

	wm8994_set_playback_speaker_headset(codec);
}


void wm8994_set_voicecall_common_setting(struct snd_soc_codec *codec)
{
	int val;

	/*GPIO Configuration*/
	wm8994_write(codec, WM8994_GPIO_1, 0xA101);
	wm8994_write(codec, WM8994_GPIO_2, 0x8100);
#ifdef CONFIG_TARGET_LOCALE_VZW
    wm8994_write(codec, 0x0702, 0x8100);    // GPIO 3. Speech PCM Clock
    wm8994_write(codec, 0x0703, 0x8100);    // GPIO 4. Speech PCM Sync
    wm8994_write(codec, 0x0704, 0x8100);    // GPIO 5. Speech PCM Data Out
#else
	wm8994_write(codec, WM8994_GPIO_3, 0x0100);
	wm8994_write(codec, WM8994_GPIO_4, 0x0100);
	wm8994_write(codec, WM8994_GPIO_5, 0x8100);
#endif
	wm8994_write(codec, WM8994_GPIO_6, 0xA101);
#ifdef CONFIG_TARGET_LOCALE_VZW
        wm8994_write(codec, 0x0706, 0x0100);    // GPIO 7. Speech PCM Data Input
#else
	wm8994_write(codec, WM8994_GPIO_7, 0x0100);
#endif
	wm8994_write(codec, WM8994_GPIO_8, 0xA101);
	wm8994_write(codec, WM8994_GPIO_9, 0xA101);
	wm8994_write(codec, WM8994_GPIO_10, 0xA101);
	wm8994_write(codec, WM8994_GPIO_11, 0xA101);

	/*FLL2 Setting*/
	wm8994_write(codec, WM8994_FLL2_CONTROL_2, 0x2F00);
	wm8994_write(codec, WM8994_FLL2_CONTROL_3, 0x3126);
	wm8994_write(codec, WM8994_FLL2_CONTROL_4, 0x0100);
	wm8994_write(codec, WM8994_FLL2_CONTROL_5, 0x0C88);
//	wm8994_write(codec, WM8994_FLL2_CONTROL_5, 0x0C89);	// CP 24MHz
	wm8994_write(codec, WM8994_FLL2_CONTROL_1, WM8994_FLL2_FRACN_ENA | WM8994_FLL2_ENA);

	val = wm8994_read(codec, WM8994_AIF2_CLOCKING_1);
	if(!(val & WM8994_AIF2CLK_ENA))
		wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0018);

	wm8994_write(codec, WM8994_AIF2_RATE, 0x3 << WM8994_AIF2CLK_RATE_SHIFT);
	
	// AIF2 Interface - PCM Stereo mode
	wm8994_write(codec, WM8994_AIF2_CONTROL_1,	//Left Justified, BCLK invert, LRCLK Invert
		WM8994_AIF2ADCR_SRC | WM8994_AIF2_BCLK_INV |0x18);

	wm8994_write(codec, WM8994_AIF2_BCLK, 0x70);
	wm8994_write(codec, WM8994_AIF2_CONTROL_2, 0x0000);
#ifdef CONFIG_TARGET_LOCALE_VZW
        wm8994_write(codec, WM8994_AIF2_MASTER_SLAVE, 0x0000);
#else
	wm8994_write(codec, WM8994_AIF2_MASTER_SLAVE, WM8994_AIF2_MSTR | WM8994_AIF2_CLK_FRC | WM8994_AIF2_LRCLK_FRC);	//Master
#endif
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);	//05 : 3303
	val &= ~(WM8994_AIF2DACL_ENA_MASK | WM8994_AIF2DACR_ENA_MASK | WM8994_AIF1DAC1L_ENA_MASK | WM8994_AIF1DAC1R_ENA_MASK
		 | WM8994_DAC1L_ENA_MASK | WM8994_DAC1R_ENA_MASK);
	val |= (WM8994_AIF2DACL_ENA | WM8994_AIF2DACR_ENA | WM8994_AIF1DAC1L_ENA | WM8994_AIF1DAC1R_ENA
		 | WM8994_DAC1L_ENA | WM8994_DAC1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	/*Clocking*/	
	val = wm8994_read(codec, WM8994_CLOCKING_1);
	val |= (WM8994_DSP_FS2CLK_ENA);
	wm8994_write(codec, WM8994_CLOCKING_1, val);

	wm8994_write(codec, WM8994_POWER_MANAGEMENT_6, 0x0);

//	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, 0x0);

	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_3 );
	val &= ~( WM8994_SPKLVOL_ENA_MASK | WM8994_SPKRVOL_ENA_MASK  );
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_3 ,val);


	val = wm8994_read(codec,WM8994_SPEAKER_MIXER );
	val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK | WM8994_DAC1R_TO_SPKMIXR_MASK);
	wm8994_write(codec,WM8994_SPEAKER_MIXER ,val);

	
	// AIF1 & AIF2 Output is connected to DAC1	
	val = wm8994_read(codec,WM8994_DAC1_LEFT_MIXER_ROUTING);	
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK | WM8994_AIF2DACL_TO_DAC1L_MASK);	
	val |= (WM8994_AIF1DAC1L_TO_DAC1L | WM8994_AIF2DACL_TO_DAC1L);
	wm8994_write(codec,WM8994_DAC1_LEFT_MIXER_ROUTING, val );  

	val = wm8994_read(codec,WM8994_DAC1_RIGHT_MIXER_ROUTING);	
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK | WM8994_AIF2DACR_TO_DAC1R_MASK);	
	val |= (WM8994_AIF1DAC1R_TO_DAC1R | WM8994_AIF2DACR_TO_DAC1R);
	wm8994_write(codec,WM8994_DAC1_RIGHT_MIXER_ROUTING, val);
	
	val = wm8994_read(codec, WM8994_AIF1_DAC1_LEFT_VOLUME);
	val &= ~(WM8994_AIF1DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL);
	wm8994_write(codec, WM8994_AIF1_DAC1_LEFT_VOLUME, val);
	
	val = wm8994_read(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME);
	val &= ~(WM8994_AIF1DAC1R_VOL_MASK);
	val |= (WM8994_AIF1DAC1_VU |TUNING_DAC1R_VOL);
	wm8994_write(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME, val);

	wm8994_write(codec, 0x6, 0x0);
}

void wm8994_set_voicecall_receiver(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;

	int val;
	
	DEBUG_LOG("");

	audio_ctrl_mic_bias_gpio(1);

	wm8994_set_voicecall_common_setting(codec);

	wm8994_write(codec, WM8994_CHARGE_PUMP_1, WM8994_CP_ENA_DEFAULT);	// Turn off charge pump.
	
	// Analogue Input Configuration -Main MIC
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, 
		WM8994_TSHUT_ENA | WM8994_TSHUT_OPDIS | WM8994_MIXINL_ENA | WM8994_IN1L_ENA);
	
	wm8994_write(codec, WM8994_INPUT_MIXER_2, WM8994_IN1LP_TO_IN1L | WM8994_IN1LN_TO_IN1L); 	// differential(3) or single ended(1)
		
	/* Digital Path Enables and Unmutes*/	
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, WM8994_AIF2ADCL_ENA | WM8994_ADCL_ENA);	
	
	wm8994_write(codec,WM8994_AIF1_DAC1_FILTERS_1, 0x0000 );  	//AIF1DAC1 Unmute, Mono Mix diable, Fast Ramp
	wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, 0x0000 );	//AIF2DAC Unmute, Mono Mix diable, Fast Ramp

	wm8994_write(codec, WM8994_DAC2_MIXER_VOLUMES, 0x000C);	
	wm8994_write(codec, WM8994_DAC2_LEFT_VOLUME, 0x00C0 );	
	wm8994_write(codec, WM8994_DAC2_RIGHT_VOLUME, 0x01C0 );	

	// Tx -> AIF2 Path
	wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, WM8994_ADC1_TO_DAC2L);	
	wm8994_write(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING, WM8994_ADC1_TO_DAC2R);

	wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0019);

	if((wm8994->testmode_config_flag == SEC_NORMAL)
		|| (wm8994->testmode_config_flag == SEC_TEST_PBA_LOOPBACK)
	)
	{	
		// Unmute IN1L PGA, update volume
		val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);	
		val &= ~(WM8994_IN1L_MUTE_MASK | WM8994_IN1L_VOL_MASK);
		val |= (WM8994_IN1L_VU |TUNING_CALL_RCV_INPUTMIX_VOL); //volume
		wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);
	
		//Unmute the PGA
		val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
		val&= ~(WM8994_IN1L_TO_MIXINL_MASK | WM8994_IN1L_MIXINL_VOL_MASK | WM8994_MIXOUTL_MIXINL_VOL_MASK);
		val |= (WM8994_IN1L_TO_MIXINL |TUNING_CALL_RCV_MIXER_VOL);//30db
		wm8994_write(codec, WM8994_INPUT_MIXER_3, val); 
	
		// Volume Control - Output
		val = wm8994_read(codec, WM8994_OUTPUT_MIXER_5);
		val &= ~(WM8994_DACL_MIXOUTL_VOL_MASK);
		val |= TUNING_RCV_OUTMIX5_VOL << WM8994_DACL_MIXOUTL_VOL_SHIFT;
		wm8994_write(codec,WM8994_OUTPUT_MIXER_5, val );
		
		val = wm8994_read(codec, WM8994_OUTPUT_MIXER_6);
		val &= ~(WM8994_DACR_MIXOUTR_VOL_MASK);
		val |= TUNING_RCV_OUTMIX6_VOL << WM8994_DACR_MIXOUTR_VOL_SHIFT;
		wm8994_write(codec,WM8994_OUTPUT_MIXER_6, val );
	
		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK | WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUTL_MUTE_N | TUNING_RCV_OPGAL_VOL);
		wm8994_write(codec,WM8994_LEFT_OPGA_VOLUME, val );
	
		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK | WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU | WM8994_MIXOUTR_MUTE_N | TUNING_RCV_OPGAR_VOL);
		wm8994_write(codec,WM8994_RIGHT_OPGA_VOLUME, val );
		
		val = wm8994_read(codec, WM8994_HPOUT2_VOLUME);
		val &= ~(WM8994_HPOUT2_MUTE_MASK | WM8994_HPOUT2_VOL_MASK);
		val |= TUNING_HPOUT2_VOL << WM8994_HPOUT2_VOL_SHIFT;
		wm8994_write(codec,WM8994_HPOUT2_VOLUME, val );
	
		//Unmute DAC1 left
		val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );
		val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL); 
		wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);
		
		//Unmute and volume ctrl RightDAC
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
		val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
		val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume	
		wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);
	}

	// Output Mixing
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, WM8994_DAC1L_TO_MIXOUTL);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, WM8994_DAC1R_TO_MIXOUTR );

	// Analogue Output Configuration
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, 
		WM8994_MIXOUTLVOL_ENA | WM8994_MIXOUTRVOL_ENA |WM8994_MIXOUTL_ENA | WM8994_MIXOUTR_ENA);
	wm8994_write(codec, WM8994_HPOUT2_MIXER, WM8994_MIXOUTLVOL_TO_HPOUT2 |WM8994_MIXOUTRVOL_TO_HPOUT2); 
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, 
		WM8994_HPOUT2_ENA | WM8994_VMID_SEL_NORMAL | WM8994_BIAS_ENA);

	//	wm8994_write(codec, WM8994_INPUT_MIXER_4, 0x0000 ); 
}


void wm8994_set_voicecall_headphone(struct snd_soc_codec *codec)
{
#ifdef CONFIG_TARGET_LOCALE_KOR

	struct wm8994_priv *wm8994 = codec->drvdata;

	u16 TestReturn1=0;
	u16 TestReturn2=0;
	u16 TestLow1=0;
	u16 TestHigh1=0;
	u8 TestLow=0;
	u8 TestHigh=0;

	DEBUG_LOG("");

	audio_ctrl_mic_bias_gpio(1);
	wm8994_write(codec,WM8994_SOFTWARE_RESET, 0x0000 );
	msleep(50);

	wm8994_write(codec,0x39, 0x0068 );	         //WM8994_ANTIPOP_2


	//Output Enable (HPOUT1, HPOUT2, SPKOUT)( )
	wm8994_write(codec,0x01, 0x07 );             //WM8994_POWER_MANAGEMENT_1 (for fast start up)
	msleep(50);                                  // 50msec delay is recommeded by wolfson
	wm8994_write(codec,0x01, 0x03 );             //WM8994_POWER_MANAGEMENT_1 (for normal operation)


	/* Added for Solving Robotic Sound(recommeded by Wolfson) */
	wm8994_write(codec,0x0102, 0x0003 );
	wm8994_write(codec,0x0817, 0x0000 );
	wm8994_write(codec,0x0102, 0x0000 );

	//INPUT
	wm8994_write(codec,0x15, 0x0040 );	         //WM8994_INPUT_MIXER_1

	/*GPIO Configuration*/	
	wm8994_write(codec, 0x700, 0xA101 );  // GPIO1 is Input Enable
	wm8994_write(codec,0x701, 0x8100 );	         //WM8994_GPIO_2
	wm8994_write(codec,0x702, 0x8100 );	         //WM8994_GPIO_3
	wm8994_write(codec,0x703, 0x8100 );          //WM8994_GPIO_4
	wm8994_write(codec,0x704, 0x8100 );	         //WM8994_GPIO_5
	wm8994_write(codec,0x706, 0x0100 );	         //WM8994_GPIO_7

	// FLL1 = 11.2896 MHz
	wm8994_write(codec, 0x224, 0x0C88 ); // 0x0C88 -> 0x0C80 FLL1_CLK_REV_DIV = 1 = MCLK / FLL1, FLL1 = MCLK1
	wm8994_write(codec, 0x221, 0x0700 ); // FLL1_OUT_DIV = 8 = Fvco / Fout
	wm8994_write(codec, 0x222, 0x86C2 ); // FLL1_N = 34498
	wm8994_write(codec, 0x223, 0x00E0 ); // FLL1_N = 7, FLL1_GAIN = X1	
	wm8994_write(codec, 0x220, 0x0005 ); // FLL1 Enable, FLL1 Fractional Mode
	// FLL2 = 12.288MHz , fs = 8kHz
	wm8994_write(codec, 0x244, 0x0C83 ); // 0x0C88 -> 0x0C80 FLL2_CLK_REV_DIV = 1 = MCLK / FLL2, FLL2 = MCLK1
	wm8994_write(codec, 0x241, 0x0700 ); // FLL2_OUTDIV = 23 = Fvco / Fout
	wm8994_write(codec, 0x243, 0x0600 ); // FLL2_N[14:5] = 8, FLL2_GAIN[3:0] = X1
	wm8994_write(codec, 0x240, 0x0001 ); // FLL2 Enable, FLL2 Fractional Mode
	msleep(2);

	wm8994_write(codec,0x04, 0x3003 );	         //WM8994_POWER_MANAGEMENT_4
	wm8994_write(codec,0x02, 0x6240 );           //WM8994_POWER_MANAGEMENT_2

	/* ==================== Clocking and AIF Configuration ==================== */
	wm8994_write(codec, 0x620, 0x0000); // ADC oversampling disabled, DAC oversampling disabled
	wm8994_write(codec, 0x210, 0x0073); // AIF1 Sample Rate = 44.1kHz AIF1CLK_RATE=256 => AIF1CLK = 11.2896 MHz
	wm8994_write(codec, 0x300, 0x4010);
	wm8994_write(codec, 0x302, 0x7000);
	wm8994_write(codec, 0x211, 0x0009); // AIF2 Sample Rate = 8kHz AIF2CLK_RATE=256 => AIF2CLK = 8k*256 = 2.048MHz
	wm8994_write(codec, 0x310, 0x4118); // DSP A mode, 16bit, BCLK2 invert
	wm8994_write(codec, 0x311, 0x0000); // AIF2_LOOPBACK
	wm8994_write(codec, 0x208, 0x000F); // 0x000E -> 0x0007 // DSP1, DSP2 processing Enable, SYSCLK = AIF1CLK = 11.2896 MHz   
	wm8994_write(codec, 0x200, 0x0011); // AIF1 Enable, AIF1CLK = FLL1 
	wm8994_write(codec, 0x204, 0x0019 ); // AIF2CLK_SRC = FLL2, AIF2CLK_INV[2] = 0,  AIF2CLK_DIV[1] = 0, AIF2CLK_ENA[0] = 1

	wm8994_write(codec,0x520, 0x0000 );	         //WM8994_AIF2_DAC_FILTERS_1
	wm8994_write(codec,0x420, 0x0000 );	         //WM8994_AIF1_DAC1_FILTERS_1

	wm8994_write(codec,0x601, 0x0005);           //WM8994_DAC1_LEFT_MIXER_ROUTING
	wm8994_write(codec,0x602, 0x0005);           //WM8994_DAC1_RIGHT_MIXER_ROUTING
	wm8994_write(codec,0x603, 0x000C );	         //WM8994_DAC2_MIXER_VOLUMES
	wm8994_write(codec,0x604, 0x0010 );          //WM8994_DAC2_LEFT_MIXER_ROUTING

	wm8994_write(codec,0X621, 0x01C0 );          //WM8994_SIDETONE  
	wm8994_write(codec,0X312, 0x0000 );          //WM8994_AIF2_MASTER_SLAVE
	wm8994_write(codec,0x301, 0x4000 );          //WM8994_AIF1_CONTROL_2

	wm8994_write(codec,0x18, 0x0111 );           //WM8994_LEFT_LINE_INPUT_1_2_VOLUME
	wm8994_write(codec,0x28, 0x0030 );	         //WM8994_INPUT_MIXER_2
	wm8994_write(codec,0x29, 0x0030 );	         //WM8994_INPUT_MIXER_3
	wm8994_write(codec,0x15, 0x0000 );	         //WM8994_INPUT_MIXER_1

	wm8994_write(codec,0x500, 0x01C0 );	         //WM8994_AIF2_ADC_LEFT_VOLUME

	wm8994_write(codec,0x102, 0x0003 );	         //0x610
	wm8994_write(codec,0x56, 0x0003 );	         //0x610
	wm8994_write(codec,0x102, 0x0000 );	         //0x612
	wm8994_write(codec,0x5D, 0x0002 );	         //0x610
	wm8994_write(codec,0x55, 0x03E0);	         //WM8994_DC_SERVO_2
	wm8994_write(codec,0x01, 0x0303 );	         //WM8994_POWER_MANAGEMENT_1
	wm8994_write(codec,0x60, 0x0022 );	         //WM8994_ANALOGUE_HP_1
	wm8994_write(codec,0x4C, 0x9F25 );	         //0x610

	//delay 5ms
	msleep(5);

	wm8994_write(codec,0x05, 0x3303 );	         //WM8994_POWER_MANAGEMENT_5	

	wm8994_write(codec,0x03, 0x0030 );	         //WM8994_POWER_MANAGEMENT_3


	wm8994_write(codec,0x2D, 0x0001 );	         //WM8994_OUTPUT_MIXER_1
	wm8994_write(codec,0x2E, 0x0001 );	         //WM8994_OUTPUT_MIXER_2


	wm8994_write(codec,0x54, 0x0303);	         //WM8994_DC_SERVO_1

	//delay 160ms
	msleep(160);
	TestReturn1=wm8994_read(codec,WM8994_DC_SERVO_4);

	TestLow=(signed char)(TestReturn1 & 0xff);
	TestHigh=(signed char)((TestReturn1>>8) & 0xff);

	TestLow1=((signed short)(TestLow-3))&0x00ff;
	TestHigh1=(((signed short)(TestHigh-3)<<8)&0xff00);
	TestReturn2=TestLow1|TestHigh1;
	wm8994_write(codec,WM8994_DC_SERVO_4, TestReturn2);

	wm8994_write(codec,0x54, 0x000F );	         //WM8994_DC_SERVO_1
	//delay 20ms
	msleep(20);
	wm8994_write(codec,0x60, 0x00EE );	         //WM8994_ANALOGUE_HP_1

	//Unmute DAC1 left and volume ctrl LeftDAC
	wm8994_write(codec,0x610, 0x00C0 );	         //WM8994_DAC1_LEFT_VOLUME
	wm8994_write(codec,0x611, 0x01C0 );	         //WM8994_DAC1_RIGHT_VOLUME
	wm8994_write(codec,0x612, 0x01C0 );	         //WM8994_DAC2_LEFT_VOLUME


	if(wm8994->testmode_config_flag != SEC_NORMAL)
	{
		wm8994_write(codec,0x20, 0x0071 );	         //WM8994_DAC1_LEFT_VOLUME
		wm8994_write(codec,0x21, 0x0171 );	         //WM8994_DAC1_LEFT_VOLUME
	}

	//HPOUT Volume
	wm8994_write(codec,0x1C, 0x006d);
	wm8994_write(codec,0x1D, 0x016d);

	wm8994_write(codec,0x2A, 0x0030);
	wm8994_write(codec,0x1A, 0x010E);


#else


	struct wm8994_priv *wm8994 = codec->drvdata;

	int val;

	u16 TestReturn1=0;
	u16 TestReturn2=0;
	u16 TestLow1=0;
	u16 TestHigh1=0;
	u8 TestLow=0;
	u8 TestHigh=0;

	DEBUG_LOG("");

	audio_ctrl_mic_bias_gpio(1);
	
	wm8994_set_voicecall_common_setting(codec);
	
	/*Digital Path Enables and Unmutes*/	
	wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, WM8994_ADC1_TO_DAC2L);
	wm8994_write(codec,WM8994_DAC2_MIXER_VOLUMES, 0x000C );  
	wm8994_write(codec,WM8994_SIDETONE, 0x01C0);

	/*Analogue Input Configuration*/
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_2, 0x6240 );

	if((wm8994->testmode_config_flag == SEC_NORMAL)
		|| (wm8994->testmode_config_flag == SEC_TEST_PBA_LOOPBACK)
	)
	{
		wm8994_write(codec,WM8994_LEFT_LINE_INPUT_1_2_VOLUME,0x0111);
		wm8994_write(codec,WM8994_INPUT_MIXER_3 ,0x0030);
	}	

	wm8994_write(codec,WM8994_INPUT_MIXER_2, 0x0030 );	 

	/* Unmute*/
	if((wm8994->testmode_config_flag == SEC_NORMAL)
		|| (wm8994->testmode_config_flag == SEC_TEST_PBA_LOOPBACK)
	)
	{
		wm8994_write(codec,WM8994_LEFT_OPGA_VOLUME, 0x0071 );
		wm8994_write(codec,WM8994_RIGHT_OPGA_VOLUME, 0x0171 );
	}
		
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_4, 0x3003 );   
	
	val = wm8994_read(codec, 0x102  ); 	
	val &= ~(0x0003);
	val = 0x0003;
	wm8994_write(codec,0x102,val);
	
	val = wm8994_read(codec, 0x56  ); 	
	val &= ~(0x0003);
	val = 0x0003;
	wm8994_write(codec,0x56,val);
	
	val = wm8994_read(codec, 0x102  ); 	
	val &= ~(0x0000);
	val = 0x0000;
	wm8994_write(codec,0x102,val);

	val = wm8994_read(codec, WM8994_CLASS_W_1  ); 	
	val &= ~(0x0005);
	val |= 0x0005;
	wm8994_write(codec,WM8994_CLASS_W_1,val);

	if((wm8994->testmode_config_flag == SEC_NORMAL)
		|| (wm8994->testmode_config_flag == SEC_TEST_PBA_LOOPBACK)
	)
	{
		val = wm8994_read(codec,WM8994_LEFT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK | WM8994_HPOUT1L_VOL_MASK);
		val |= (WM8994_HPOUT1L_MUTE_N | TUNING_CALL_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec,WM8994_RIGHT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK | WM8994_HPOUT1R_VOL_MASK);
		val |= (WM8994_HPOUT1_VU | WM8994_HPOUT1R_MUTE_N | TUNING_CALL_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);
	}

	val = wm8994_read(codec, WM8994_DC_SERVO_2  ); 	
	val &= ~(0x03E0);
	val = 0x03E0;
	wm8994_write(codec,WM8994_DC_SERVO_2,val);
	
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_1, 0x0303 );  
		
	wm8994_write(codec,WM8994_ANALOGUE_HP_1, 0x0022 );  
	wm8994_write(codec,WM8994_CHARGE_PUMP_1, 0x9F25 );  

	msleep(5);

	/*Analogue Output Configuration*/	
	wm8994_write(codec,WM8994_OUTPUT_MIXER_1, 0x0001 );   
	wm8994_write(codec,WM8994_OUTPUT_MIXER_2, 0x0001 );   

	wm8994_write(codec,WM8994_POWER_MANAGEMENT_3, 0x0030 );   

	wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0019);
	
	wm8994_write(codec,WM8994_DC_SERVO_1, 0x303);  

	msleep(160);	// 160ms delay

	TestReturn1=wm8994_read(codec,WM8994_DC_SERVO_4);
	
	TestLow=(signed char)(TestReturn1 & 0xff);
	TestHigh=(signed char)((TestReturn1>>8) & 0xff);

	TestLow1=((signed short)TestLow-5)&0x00ff;
	TestHigh1=(((signed short)(TestHigh-5)<<8)&0xff00);
	TestReturn2=TestLow1|TestHigh1;
	wm8994_write(codec,WM8994_DC_SERVO_4, TestReturn2);

	val = wm8994_read(codec, WM8994_DC_SERVO_1  ); 	
	val &= ~(0x000F);
	val = 0x000F;
	wm8994_write(codec,WM8994_DC_SERVO_1,val);
	
	msleep(15);
	
	wm8994_write(codec,WM8994_ANALOGUE_HP_1, 0x00EE );  

	//Unmute DAC1 left
	val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );
	val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL); 
	wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);

	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
	val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume	
	wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);

	wm8994_write(codec,WM8994_DAC2_LEFT_VOLUME, 0x01C0 );

	wm8994_write(codec,WM8994_AIF1_DAC1_FILTERS_1, 0x0000 );  
	wm8994_write(codec,WM8994_AIF2_DAC_FILTERS_1, 0x0000 );  

#endif	

    wm8994_call_recording_change_path(codec);

}

void wm8994_set_voicecall_headset(struct snd_soc_codec *codec)
{
#ifdef CONFIG_TARGET_LOCALE_KOR

	struct wm8994_priv *wm8994 = codec->drvdata;

	u16 TestReturn1=0;
	u16 TestReturn2=0;
	u16 TestLow1=0;
	u16 TestHigh1=0;
	u8 TestLow=0;
	u8 TestHigh=0;

	DEBUG_LOG("");

	audio_ctrl_mic_bias_gpio(0);	
	audio_ctrl_earmic_bias_gpio(1);

	wm8994_write(codec,WM8994_SOFTWARE_RESET, 0x0000 );
	msleep(50);

	wm8994_write(codec,0x39, 0x0068 );	         //WM8994_ANTIPOP_2


	//Output Enable (HPOUT1, HPOUT2, SPKOUT)( )
	wm8994_write(codec,0x01, 0x07 );             //WM8994_POWER_MANAGEMENT_1 (for fast start up)
	msleep(50);                                  // 50msec delay is recommeded by wolfson
	wm8994_write(codec,0x01, 0x03 );             //WM8994_POWER_MANAGEMENT_1 (for normal operation)


	/* Added for Solving Robotic Sound(recommeded by Wolfson) */
	wm8994_write(codec,0x0102, 0x0003 );
	wm8994_write(codec,0x0817, 0x0000 );
	wm8994_write(codec,0x0102, 0x0000 );

	//INPUT
	wm8994_write(codec,0x15, 0x0040 );	         //WM8994_INPUT_MIXER_1

	/*GPIO Configuration*/	
	wm8994_write(codec, 0x700, 0xA101 );  // GPIO1 is Input Enable
	wm8994_write(codec,0x701, 0x8100 );	         //WM8994_GPIO_2
	wm8994_write(codec,0x702, 0x8100 );	         //WM8994_GPIO_3
	wm8994_write(codec,0x703, 0x8100 );          //WM8994_GPIO_4
	wm8994_write(codec,0x704, 0x8100 );	         //WM8994_GPIO_5
	wm8994_write(codec,0x706, 0x0100 );	         //WM8994_GPIO_7

	// FLL1 = 11.2896 MHz
	wm8994_write(codec, 0x224, 0x0C88 ); // 0x0C88 -> 0x0C80 FLL1_CLK_REV_DIV = 1 = MCLK / FLL1, FLL1 = MCLK1
	wm8994_write(codec, 0x221, 0x0700 ); // FLL1_OUT_DIV = 8 = Fvco / Fout
	wm8994_write(codec, 0x222, 0x86C2 ); // FLL1_N = 34498
	wm8994_write(codec, 0x223, 0x00E0 ); // FLL1_N = 7, FLL1_GAIN = X1	
	wm8994_write(codec, 0x220, 0x0005 ); // FLL1 Enable, FLL1 Fractional Mode
	// FLL2 = 12.288MHz , fs = 8kHz
	wm8994_write(codec, 0x244, 0x0C83 ); // 0x0C88 -> 0x0C80 FLL2_CLK_REV_DIV = 1 = MCLK / FLL2, FLL2 = MCLK1
	wm8994_write(codec, 0x241, 0x0700 ); // FLL2_OUTDIV = 23 = Fvco / Fout
	wm8994_write(codec, 0x243, 0x0600 ); // FLL2_N[14:5] = 8, FLL2_GAIN[3:0] = X1
	wm8994_write(codec, 0x240, 0x0001 ); // FLL2 Enable, FLL2 Fractional Mode
	msleep(2);

	wm8994_write(codec,0x04, 0x3003 );	         //WM8994_POWER_MANAGEMENT_4
	wm8994_write(codec,0x02, 0x6110 );           //WM8994_POWER_MANAGEMENT_2

	/* ==================== Clocking and AIF Configuration ==================== */
	wm8994_write(codec, 0x620, 0x0000); // ADC oversampling disabled, DAC oversampling disabled
	wm8994_write(codec, 0x210, 0x0073); // AIF1 Sample Rate = 44.1kHz AIF1CLK_RATE=256 => AIF1CLK = 11.2896 MHz
	wm8994_write(codec, 0x300, 0x4010);
	wm8994_write(codec, 0x302, 0x7000);
	wm8994_write(codec, 0x211, 0x0009); // AIF2 Sample Rate = 8kHz AIF2CLK_RATE=256 => AIF2CLK = 8k*256 = 2.048MHz
	wm8994_write(codec, 0x310, 0x4118); // DSP A mode, 16bit, BCLK2 invert
	wm8994_write(codec, 0x311, 0x0000); // AIF2_LOOPBACK
	wm8994_write(codec, 0x208, 0x000F); // 0x000E -> 0x0007 // DSP1, DSP2 processing Enable, SYSCLK = AIF1CLK = 11.2896 MHz   
	wm8994_write(codec, 0x200, 0x0011); // AIF1 Enable, AIF1CLK = FLL1 
	wm8994_write(codec, 0x204, 0x0019 ); // AIF2CLK_SRC = FLL2, AIF2CLK_INV[2] = 0,  AIF2CLK_DIV[1] = 0, AIF2CLK_ENA[0] = 1


	wm8994_write(codec,0x520, 0x0000 );	         //WM8994_AIF2_DAC_FILTERS_1
	wm8994_write(codec,0x420, 0x0000 );	         //WM8994_AIF1_DAC1_FILTERS_1

	wm8994_write(codec,0x601, 0x0005);           //WM8994_DAC1_LEFT_MIXER_ROUTING
	wm8994_write(codec,0x602, 0x0005);           //WM8994_DAC1_RIGHT_MIXER_ROUTING
	wm8994_write(codec,0x603, 0x018C );	         //WM8994_DAC2_MIXER_VOLUMES
	wm8994_write(codec,0x604, 0x0030 );          //WM8994_DAC2_LEFT_MIXER_ROUTING

	wm8994_write(codec,0X621, 0x01C0 );          //WM8994_SIDETONE  

	wm8994_write(codec,0X312, 0x0000 );          //WM8994_AIF2_MASTER_SLAVE

	wm8994_write(codec,0x1A, 0x010E );           //WM8994_RIGHT_LINE_INPUT_1_2_VOLUME
	wm8994_write(codec,0x28, 0x0001 );	         //WM8994_INPUT_MIXER_2
	wm8994_write(codec,0x2A, 0x0030 );           //WM8994_INPUT_MIXER_4
	wm8994_write(codec,0x15, 0x0000 );	         //WM8994_INPUT_MIXER_1


	wm8994_write(codec,0x500, 0x01C0 );	         //WM8994_AIF2_ADC_LEFT_VOLUME

	wm8994_write(codec,0x102, 0x0003 );	         //0x610
	wm8994_write(codec,0x56, 0x0003 );	         //0x610
	wm8994_write(codec,0x102, 0x0000 );	         //0x612
	wm8994_write(codec,0x5D, 0x0002 );	         //0x610
	wm8994_write(codec,0x55, 0x03E0);	         //WM8994_DC_SERVO_2
	wm8994_write(codec,0x01, 0x0303 );	         //WM8994_POWER_MANAGEMENT_1
	wm8994_write(codec,0x60, 0x0022 );	         //WM8994_ANALOGUE_HP_1
	wm8994_write(codec,0x4C, 0x9F25 );	         //0x610

	//delay 5ms
	msleep(5);

	wm8994_write(codec,0x05, 0x3303 );	         //WM8994_POWER_MANAGEMENT_5	

	wm8994_write(codec,0x03, 0x0030 );	         //WM8994_POWER_MANAGEMENT_3


	wm8994_write(codec,0x2D, 0x0001 );	         //WM8994_OUTPUT_MIXER_1
	wm8994_write(codec,0x2E, 0x0001 );	         //WM8994_OUTPUT_MIXER_2


	wm8994_write(codec,0x54, 0x0303);	         //WM8994_DC_SERVO_1

	//delay 160ms
	msleep(160);

	TestReturn1=wm8994_read(codec,WM8994_DC_SERVO_4);

	TestLow=(signed char)(TestReturn1 & 0xff);
	TestHigh=(signed char)((TestReturn1>>8) & 0xff);

	TestLow1=((signed short)(TestLow-3))&0x00ff;
	TestHigh1=(((signed short)(TestHigh-3)<<8)&0xff00);
	TestReturn2=TestLow1|TestHigh1;
	wm8994_write(codec,WM8994_DC_SERVO_4, TestReturn2);

	wm8994_write(codec,0x54, 0x000F );	         //WM8994_DC_SERVO_1
	//delay 20ms
	msleep(20);
	wm8994_write(codec,0x60, 0x00EE );	         //WM8994_ANALOGUE_HP_1

	//Unmute DAC1 left and volume ctrl LeftDAC
	wm8994_write(codec,0x610, 0x00C0 );	         //WM8994_DAC1_LEFT_VOLUME
	wm8994_write(codec,0x611, 0x01C0 );	         //WM8994_DAC1_RIGHT_VOLUME
	wm8994_write(codec,0x612, 0x01C0 );	         //WM8994_DAC2_LEFT_VOLUME

	//HPOUT Volume
	wm8994_write(codec,0x1C, 0x006d);
	wm8994_write(codec,0x1D, 0x016d);

	if(wm8994->testmode_config_flag == SEC_TEST_PBA_LOOPBACK)
	{
		wm8994_write(codec,0x1C, 0x007E);
		wm8994_write(codec,0x1D, 0x017E);
		wm8994_write(codec,0x1A, 0x0104 );           //WM8994_RIGHT_LINE_INPUT_1_2_VOLUME
	}
	else if(wm8994->testmode_config_flag != SEC_NORMAL)
	{
		wm8994_write(codec,0x20, 0x0071 );	         //WM8994_DAC1_LEFT_VOLUME
		wm8994_write(codec,0x21, 0x0171 );	         //WM8994_DAC1_LEFT_VOLUME
	}


#else


	struct wm8994_priv *wm8994 = codec->drvdata;

	int val;

	u16 TestReturn1=0;
	u16 TestReturn2=0;
	u16 TestLow1=0;
	u16 TestHigh1=0;
	u8 TestLow=0;
	u8 TestHigh=0;

	DEBUG_LOG("");

	audio_ctrl_mic_bias_gpio(0);	
	audio_ctrl_earmic_bias_gpio(1);
	
	wm8994_set_voicecall_common_setting(codec);
	
	/*Digital Path Enables and Unmutes*/	
	wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, WM8994_ADC2_TO_DAC2L);
	wm8994_write(codec,WM8994_DAC2_MIXER_VOLUMES, 0x0180 );  
	wm8994_write(codec,WM8994_SIDETONE, 0x01C0);

	/*Analogue Input Configuration*/
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_2);	
	val &= ~(WM8994_TSHUT_ENA_MASK|WM8994_TSHUT_OPDIS_MASK|WM8994_MIXINR_ENA_MASK|WM8994_IN1R_ENA_MASK);	
	val |= (WM8994_TSHUT_ENA|WM8994_TSHUT_OPDIS|WM8994_MIXINR_ENA|WM8994_IN1R_ENA);
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_2, 0x6110 );

	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		val = wm8994_read(codec,WM8994_RIGHT_LINE_INPUT_1_2_VOLUME);	
		val &= ~(WM8994_IN1R_MUTE_MASK | WM8994_IN1R_VOL_MASK);	// Unmute IN1R
		val |= (WM8994_IN1R_VU | TUNING_CALL_EAR_INPUTMIX_VOL);
		wm8994_write(codec,WM8994_RIGHT_LINE_INPUT_1_2_VOLUME,val);
	
		// unmute right pga, set volume 
		val = wm8994_read(codec,WM8994_INPUT_MIXER_4 );
		val&= ~(WM8994_IN1R_TO_MIXINR_MASK | WM8994_IN1R_MIXINR_VOL_MASK | WM8994_MIXOUTR_MIXINR_VOL_MASK);
		val |= (WM8994_IN1R_TO_MIXINR);//0db
		wm8994_write(codec,WM8994_INPUT_MIXER_4 ,val);
	}	

	val = wm8994_read(codec,WM8994_INPUT_MIXER_2 );
	val&= ~(WM8994_IN1RP_TO_IN1R_MASK |  WM8994_IN1RN_TO_IN1R_MASK);
	val |= (WM8994_IN1RP_TO_IN1R|WM8994_IN1RN_TO_IN1R);//0db
	wm8994_write(codec,WM8994_INPUT_MIXER_2, 0x0003 );	 

	/* Unmute*/
	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK | WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUTL_MUTE_N | TUNING_CALL_OPGAL_VOL);
		wm8994_write(codec,WM8994_LEFT_OPGA_VOLUME, val );
	
		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK | WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU | WM8994_MIXOUTR_MUTE_N | TUNING_CALL_OPGAR_VOL);
		wm8994_write(codec,WM8994_RIGHT_OPGA_VOLUME, val );
	}
		
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_4, 0x2001 );   
	
	val = wm8994_read(codec, 0x102  ); 	
	val &= ~(0x0003);
	val = 0x0003;
	wm8994_write(codec,0x102,val);
	
	val = wm8994_read(codec, 0x56  ); 	
	val &= ~(0x0003);
	val = 0x0003;
	wm8994_write(codec,0x56,val);
	
	val = wm8994_read(codec, 0x102  ); 	
	val &= ~(0x0000);
	val = 0x0000;
	wm8994_write(codec,0x102,val);

	val = wm8994_read(codec, WM8994_CLASS_W_1  ); 	
	val &= ~(0x0005);
	val |= 0x0005;
	wm8994_write(codec,WM8994_CLASS_W_1,val);

	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		val = wm8994_read(codec,WM8994_LEFT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK | WM8994_HPOUT1L_VOL_MASK);
		val |= (WM8994_HPOUT1L_MUTE_N | TUNING_CALL_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec,WM8994_RIGHT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK | WM8994_HPOUT1R_VOL_MASK);
		val |= (WM8994_HPOUT1_VU | WM8994_HPOUT1R_MUTE_N | TUNING_CALL_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);
	}

	val = wm8994_read(codec, WM8994_DC_SERVO_2  ); 	
	val &= ~(0x03E0);
	val = 0x03E0;
	wm8994_write(codec,WM8994_DC_SERVO_2,val);
	
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_1, 0x0303 );  
		
	wm8994_write(codec,WM8994_ANALOGUE_HP_1, 0x0022 );  
	wm8994_write(codec,WM8994_CHARGE_PUMP_1, 0x9F25 );  

	msleep(5);

	/*Analogue Output Configuration*/	
	wm8994_write(codec,WM8994_OUTPUT_MIXER_1, 0x0001 );   
	wm8994_write(codec,WM8994_OUTPUT_MIXER_2, 0x0001 );   

	wm8994_write(codec,WM8994_POWER_MANAGEMENT_3, 0x0030 );   

	wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0019);
	
	wm8994_write(codec,WM8994_DC_SERVO_1, 0x303);  

	msleep(160);	// 160ms delay

	TestReturn1=wm8994_read(codec,WM8994_DC_SERVO_4);
	
	TestLow=(signed char)(TestReturn1 & 0xff);
	TestHigh=(signed char)((TestReturn1>>8) & 0xff);

	TestLow1=((signed short)TestLow-5)&0x00ff;
	TestHigh1=(((signed short)(TestHigh-5)<<8)&0xff00);
	TestReturn2=TestLow1|TestHigh1;
	wm8994_write(codec,WM8994_DC_SERVO_4, TestReturn2);

	val = wm8994_read(codec, WM8994_DC_SERVO_1  ); 	
	val &= ~(0x000F);
	val = 0x000F;
	wm8994_write(codec,WM8994_DC_SERVO_1,val);
	
	msleep(15);
	
	wm8994_write(codec,WM8994_ANALOGUE_HP_1, 0x00EE );  

	//Unmute DAC1 left
	val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );
	val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL);
	wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);

	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
	val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume	
	wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);

	wm8994_write(codec,WM8994_DAC2_LEFT_VOLUME, 0x01C0 );

	wm8994_write(codec,WM8994_AIF1_DAC1_FILTERS_1, 0x0000 );  
	wm8994_write(codec,WM8994_AIF2_DAC_FILTERS_1, 0x0000 );  

	if(wm8994->testmode_config_flag == SEC_TEST_PBA_LOOPBACK)
	{
		DEBUG_LOG("SEC_TEST_PBA_LOOPBACK");
		
		val = wm8994_read(codec,WM8994_RIGHT_LINE_INPUT_1_2_VOLUME);	
		val &= ~(WM8994_IN1R_MUTE_MASK | WM8994_IN1R_VOL_MASK);	// Unmute IN1R
		val |= (WM8994_IN1R_VU | TUNING_LOOPBACK_EAR_INPUTMIX_VOL);
		wm8994_write(codec,WM8994_RIGHT_LINE_INPUT_1_2_VOLUME,val);

		// unmute right pga, set volume 
		val = wm8994_read(codec,WM8994_INPUT_MIXER_4 );
		val&= ~(WM8994_IN1R_TO_MIXINR_MASK | WM8994_IN1R_MIXINR_VOL_MASK | WM8994_MIXOUTR_MIXINR_VOL_MASK);
		val |= (WM8994_IN1R_TO_MIXINR);//0db
		wm8994_write(codec,WM8994_INPUT_MIXER_4 ,val);

		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK | WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUTL_MUTE_N | TUNING_LOOPBACK_OPGAL_VOL);
		wm8994_write(codec,WM8994_LEFT_OPGA_VOLUME, val );

		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK | WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU | WM8994_MIXOUTR_MUTE_N | TUNING_LOOPBACK_OPGAR_VOL);
		wm8994_write(codec,WM8994_RIGHT_OPGA_VOLUME, val );

		val = wm8994_read(codec,WM8994_LEFT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK | WM8994_HPOUT1L_VOL_MASK);
		val |= (WM8994_HPOUT1L_MUTE_N | TUNING_LOOPBACK_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec,WM8994_RIGHT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK | WM8994_HPOUT1R_VOL_MASK);
		val |= (WM8994_HPOUT1_VU | WM8994_HPOUT1R_MUTE_N | TUNING_LOOPBACK_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);
	}

#endif
    
    wm8994_call_recording_change_path(codec);


}

void wm8994_set_voicecall_speaker(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;

	int val;

	DEBUG_LOG("");

	audio_ctrl_mic_bias_gpio(1);

#ifdef CONFIG_TARGET_LOCALE_KOR


	wm8994_write(codec,WM8994_SOFTWARE_RESET, 0x0000 );
	msleep(50);

	wm8994_write(codec,0x39, 0x0068 );	         //WM8994_ANTIPOP_2


	//Output Enable (HPOUT1, HPOUT2, SPKOUT)( )
	wm8994_write(codec,0x01, 0x07 );             //WM8994_POWER_MANAGEMENT_1 (for fast start up)
	msleep(50);                                  // 50msec delay is recommeded by wolfson
	wm8994_write(codec,0x01, 0x03 );             //WM8994_POWER_MANAGEMENT_1 (for normal operation)


	/* Added for Solving Robotic Sound(recommeded by Wolfson) */
	wm8994_write(codec,0x0102, 0x0003 );
	wm8994_write(codec,0x0817, 0x0000 );
	wm8994_write(codec,0x0102, 0x0000 );


	//INPUT
	wm8994_write(codec,0x15, 0x0040 );	         //WM8994_INPUT_MIXER_1

	/*GPIO Configuration*/	
	wm8994_write(codec, 0x700, 0xA101 );  // GPIO1 is Input Enable
	wm8994_write(codec,0x701, 0x8100 );	         //WM8994_GPIO_2
	wm8994_write(codec,0x702, 0x8100 );	         //WM8994_GPIO_3
	wm8994_write(codec,0x703, 0x8100 );          //WM8994_GPIO_4
	wm8994_write(codec,0x704, 0x8100 );	         //WM8994_GPIO_5
	wm8994_write(codec,0x706, 0x0100 );	         //WM8994_GPIO_7

	// FLL1 = 11.2896 MHz
	wm8994_write(codec, 0x224, 0x0C88 ); // 0x0C88 -> 0x0C80 FLL1_CLK_REV_DIV = 1 = MCLK / FLL1, FLL1 = MCLK1
	wm8994_write(codec, 0x221, 0x0700 ); // FLL1_OUT_DIV = 8 = Fvco / Fout
	wm8994_write(codec, 0x222, 0x86C2 ); // FLL1_N = 34498
	wm8994_write(codec, 0x223, 0x00E0 ); // FLL1_N = 7, FLL1_GAIN = X1	
	wm8994_write(codec, 0x220, 0x0005 ); // FLL1 Enable, FLL1 Fractional Mode
	// FLL2 = 12.288MHz , fs = 8kHz
	wm8994_write(codec, 0x244, 0x0C83 ); // 0x0C88 -> 0x0C80 FLL2_CLK_REV_DIV = 1 = MCLK / FLL2, FLL2 = MCLK1
	wm8994_write(codec, 0x241, 0x0700 ); // FLL2_OUTDIV = 23 = Fvco / Fout
	wm8994_write(codec, 0x243, 0x0600 ); // FLL2_N[14:5] = 8, FLL2_GAIN[3:0] = X1
	wm8994_write(codec, 0x240, 0x0001 ); // FLL2 Enable, FLL2 Fractional Mode
	msleep(2);

	wm8994_write(codec, 0x04, 0x2002 ); // AIF2ADCL_ENA, ADCL_ENA
	wm8994_write(codec, 0x05, 0x3303); // 303 ); // AIF2DACL_ENA, AIF2DACR_ENA, AIF1DAC1L_ENA, AIF1DAC1R_ENA, DAC1L_EN, DAC1R_EN


	/* ==================== Clocking and AIF Configuration ==================== */
	wm8994_write(codec, 0x620, 0x0000); // ADC oversampling disabled, DAC oversampling disabled
	wm8994_write(codec, 0x210, 0x0073); // AIF1 Sample Rate = 44.1kHz AIF1CLK_RATE=256 => AIF1CLK = 11.2896 MHz
	wm8994_write(codec, 0x300, 0x4010);
	wm8994_write(codec, 0x302, 0x7000);
	wm8994_write(codec, 0x211, 0x0009); // AIF2 Sample Rate = 8kHz AIF2CLK_RATE=256 => AIF2CLK = 8k*256 = 2.048MHz
	wm8994_write(codec, 0x310, 0x4118); // DSP A mode, 16bit, BCLK2 invert
	wm8994_write(codec, 0x311, 0x0000); // AIF2_LOOPBACK
	wm8994_write(codec, 0x208, 0x000F); // 0x000E -> 0x0007 // DSP1, DSP2 processing Enable, SYSCLK = AIF1CLK = 11.2896 MHz   
	wm8994_write(codec, 0x200, 0x0011); // AIF1 Enable, AIF1CLK = FLL1 
	wm8994_write(codec, 0x204, 0x0019 ); // AIF2CLK_SRC = FLL2, AIF2CLK_INV[2] = 0,  AIF2CLK_DIV[1] = 0, AIF2CLK_ENA[0] = 1

	wm8994_write(codec,0x520, 0x0000 );	         //WM8994_AIF2_DAC_FILTERS_1
	wm8994_write(codec,0x420, 0x0000 );	         //WM8994_AIF1_DAC1_FILTERS_1

	wm8994_write(codec,0x601, 0x0005);           //WM8994_DAC1_LEFT_MIXER_ROUTING

	wm8994_write(codec,0x603, 0x000C );	         //WM8994_DAC2_MIXER_VOLUMES
	wm8994_write(codec,0x604, 0x0010 );          //WM8994_DAC2_LEFT_MIXER_ROUTING
	wm8994_write(codec,0x621, 0x01C0 );          //WM8994_SIDETONE

	wm8994_write(codec,0x312, 0x0000 );          //WM8994_AIF2_MASTER_SLAVE


	// Analogue Input Configuration
	// Input Enable (IN1L, IN1R, IN2L, IN2R, MIXINL, MIXINR)
	wm8994_write(codec,0x02, 0x6240 );           //WM8994_POWER_MANAGEMENT_2
	wm8994_write(codec,0x18, 0x010B );           //WM8994_LEFT_LINE_INPUT_1_2_VOLUME
	wm8994_write(codec,0x28, 0x0030 );	         //WM8994_INPUT_MIXER_2
	wm8994_write(codec,0x29, 0x0030 );	         //WM8994_INPUT_MIXER_3
	wm8994_write(codec,0x15, 0x0000 );	         //WM8994_INPUT_MIXER_1


	wm8994_write(codec,0x500, 0x01C0 );	         //WM8994_AIF2_ADC_LEFT_VOLUME	

	wm8994_write(codec,0x36, 0x0002 );	         //WM8994_SPEAKER_MIXER

	// Unmute the SPKMIXVOLUME
	wm8994_write(codec,0x22, 0x0000 );	         //WM8994_SPKMIXL_ATTENUATION
	wm8994_write(codec,0x25, 0x0176 );	         //WM8994_CLASSD

	wm8994_write(codec,0x03, 0x0300 );	         //WM8994_POWER_MANAGEMENT_3	

	wm8994_write(codec,0x01, 0x3003 );	         //WM8994_POWER_MANAGEMENT_1



	wm8994_write(codec,0x620, 0x0000 );	         //WM8994_OVERSAMPLING

	//Unmute DAC1 left and volume ctrl LeftDAC
	wm8994_write(codec,0x610, 0x01C0 );	         //WM8994_DAC1_LEFT_VOLUME
	wm8994_write(codec,0x612, 0x01C0 );	         //WM8994_DAC2_LEFT_VOLUME

	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		// Volume Control - Input
		val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
		val&= ~(WM8994_IN1L_TO_MIXINL_MASK | WM8994_IN1L_MIXINL_VOL_MASK | WM8994_MIXOUTL_MIXINL_VOL_MASK);
		val |= (WM8994_IN1L_TO_MIXINL | TUNING_CALL_SPK_MIXER_VOL);//0db
		wm8994_write(codec, WM8994_INPUT_MIXER_3, val); 

		val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME );	
		val &= ~(WM8994_IN1L_MUTE_MASK | WM8994_IN1L_VOL_MASK);	// Unmute IN1L
		val |= (WM8994_IN1L_VU | TUNING_CALL_SPK_INPUTMIX_VOL);
		wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME , val);

		// Volume Control - Output
		// Unmute the SPKMIXVOLUME
		val = wm8994_read(codec, WM8994_SPKMIXL_ATTENUATION);
		val &= ~(WM8994_SPKMIXL_VOL_MASK);
		val |= TUNING_SPKMIXL_ATTEN;	
		wm8994_write(codec, WM8994_SPKMIXL_ATTENUATION, val);

		val = wm8994_read(codec,WM8994_SPKMIXR_ATTENUATION);
		val &= ~(WM8994_SPKMIXR_VOL_MASK);
		val |= TUNING_SPKMIXR_ATTEN;	
		wm8994_write(codec, WM8994_SPKMIXR_ATTENUATION, val);

		val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_3 );
		val &= ~( WM8994_SPKLVOL_ENA_MASK | WM8994_SPKRVOL_ENA_MASK  );
		val |= WM8994_SPKLVOL_ENA | WM8994_SPKRVOL_ENA;
		wm8994_write(codec,WM8994_POWER_MANAGEMENT_3 ,val);

		wm8994_write(codec,0x26, 0x007D);	//WM8994_SPEAKER_VOLUME_LEFT
		wm8994_write(codec,0x27, 0x017D);	//WM8994_SPEAKER_VOLUME_RIGHT

		val = wm8994_read(codec,WM8994_SPKOUT_MIXERS );
		val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTL_MASK 
			|WM8994_SPKMIXL_TO_SPKOUTR_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
		val |= WM8994_SPKMIXL_TO_SPKOUTL |WM8994_SPKMIXL_TO_SPKOUTR;
		wm8994_write(codec,WM8994_SPKOUT_MIXERS,val );
	}	

	if(wm8994->testmode_config_flag == SEC_TEST_15MODE)
	{
		val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME );	
		val &= ~(WM8994_IN1L_MUTE_MASK | WM8994_IN1L_VOL_MASK);	// Unmute IN1L
		val |= (WM8994_IN1L_VU | 0x14);
		wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME , val);


		val = wm8994_read(codec,WM8994_SPEAKER_VOLUME_RIGHT);
		val &= ~(WM8994_SPKOUTR_MUTE_N_MASK | WM8994_SPKOUTR_VOL_MASK);
		val |= 0x0000;
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);

		//Speaker Volume Left
		val = wm8994_read(codec,WM8994_SPEAKER_VOLUME_LEFT);
		val &= ~(WM8994_SPKOUTL_MUTE_N_MASK | WM8994_SPKOUTL_VOL_MASK);
		val |= (WM8994_SPKOUT_VU | WM8994_SPKOUTL_MUTE_N | TUNING_CALL_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);

		val = wm8994_read(codec,WM8994_CLASSD );
		val &= ~(0x3F);
		val = 0x0036;
		wm8994_write(codec,WM8994_CLASSD ,val);

		val = wm8994_read(codec,WM8994_SPKOUT_MIXERS );
		val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTL_MASK 
			|WM8994_SPKMIXL_TO_SPKOUTR_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
		val |= WM8994_SPKMIXL_TO_SPKOUTL;
		wm8994_write(codec,WM8994_SPKOUT_MIXERS,val );
	}
	else if(wm8994->testmode_config_flag == SEC_TEST_HQRL_LOOPBACK)
	{
		val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME );	
		val &= ~(WM8994_IN1L_MUTE_MASK | WM8994_IN1L_VOL_MASK);	// Unmute IN1L
		val |= (WM8994_IN1L_VU | 0x14);
		wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME , val);

		//Speaker Volume Left
		val = wm8994_read(codec,WM8994_SPEAKER_VOLUME_LEFT);
		val &= ~(WM8994_SPKOUTL_MUTE_N_MASK | WM8994_SPKOUTL_VOL_MASK);
		val |= (WM8994_SPKOUTL_MUTE_N | TUNING_CALL_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);

		val = wm8994_read(codec,WM8994_SPEAKER_VOLUME_RIGHT);
		val &= ~(WM8994_SPKOUTR_MUTE_N_MASK | WM8994_SPKOUTR_VOL_MASK);
		val |= (WM8994_SPKOUT_VU | WM8994_SPKOUTR_MUTE_N | TUNING_CALL_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);

		val = wm8994_read(codec,WM8994_CLASSD );
		val &= ~(0x3F);
		val = 0x0036;
		wm8994_write(codec,WM8994_CLASSD ,val);

		val = wm8994_read(codec,WM8994_SPKOUT_MIXERS );
		val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTL_MASK 
			|WM8994_SPKMIXL_TO_SPKOUTR_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
		val |= (WM8994_SPKMIXL_TO_SPKOUTL | WM8994_SPKMIXR_TO_SPKOUTL 
			|WM8994_SPKMIXL_TO_SPKOUTR | WM8994_SPKMIXR_TO_SPKOUTR);
		wm8994_write(codec,WM8994_SPKOUT_MIXERS,val );		
	}


#else


	wm8994_set_voicecall_common_setting(codec);

	wm8994_write(codec,0x601, 0x0005 );   
	wm8994_write(codec,0x602, 0x0005 );   
	wm8994_write(codec,0x603, 0x000C );   
	// Tx -> AIF2 Path
	wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, WM8994_ADC1_TO_DAC2L);	

	/*Analogue Input Configuration*/
	wm8994_write(codec,0x02, 0x6240 );/*wm8994_write(codec,WM8994_POWER_MANAGEMENT_2, WM8994_TSHUT_ENA|WM8994_TSHUT_OPDIS|WM8994_MIXINL_ENA|WM8994_IN1L_ENA );*/    
	wm8994_write(codec, WM8994_INPUT_MIXER_2, WM8994_IN1LP_TO_IN1L | WM8994_IN1LN_TO_IN1L); 	// differential(3) or single ended(1)

	if((wm8994->testmode_config_flag == SEC_NORMAL)
		|| (wm8994->testmode_config_flag == SEC_TEST_PBA_LOOPBACK)
	)
	{
		// Volume Control - Input
		val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
		val&= ~(WM8994_IN1L_TO_MIXINL_MASK | WM8994_IN1L_MIXINL_VOL_MASK | WM8994_MIXOUTL_MIXINL_VOL_MASK);
		val |= (WM8994_IN1L_TO_MIXINL | TUNING_CALL_SPK_MIXER_VOL);//0db
		wm8994_write(codec, WM8994_INPUT_MIXER_3, val); 

		val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME );	
		val &= ~(WM8994_IN1L_MUTE_MASK | WM8994_IN1L_VOL_MASK);	// Unmute IN1L
		val |= (WM8994_IN1L_VU | TUNING_CALL_SPK_INPUTMIX_VOL);
		wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME , val);
	}

	/*Analogue Output Configuration*/	
	wm8994_write(codec,0x03, 0x0300 );

	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, WM8994_AIF2ADCL_ENA | WM8994_ADCL_ENA);	

	wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0019);

	if((wm8994->testmode_config_flag == SEC_NORMAL)
		|| (wm8994->testmode_config_flag == SEC_TEST_PBA_LOOPBACK)
	)
	{
		// Volume Control - Output
		// Unmute the SPKMIXVOLUME
		val = wm8994_read(codec, WM8994_SPKMIXL_ATTENUATION);
		val &= ~(WM8994_SPKMIXL_VOL_MASK);
		val |= TUNING_SPKMIXL_ATTEN;	
		wm8994_write(codec, WM8994_SPKMIXL_ATTENUATION, val);

		val = wm8994_read(codec,WM8994_SPKMIXR_ATTENUATION);
		val &= ~(WM8994_SPKMIXR_VOL_MASK);
		val |= TUNING_SPKMIXR_ATTEN;	
		wm8994_write(codec, WM8994_SPKMIXR_ATTENUATION, val);


		val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_3 );
		val &= ~( WM8994_SPKLVOL_ENA_MASK | WM8994_SPKRVOL_ENA_MASK  );
		val |= WM8994_SPKLVOL_ENA | WM8994_SPKRVOL_ENA;
		wm8994_write(codec,WM8994_POWER_MANAGEMENT_3 ,val);

		val = wm8994_read(codec,WM8994_SPEAKER_VOLUME_LEFT);
		val &= ~(WM8994_SPKOUTL_MUTE_N_MASK | WM8994_SPKOUTL_VOL_MASK);
		val |= (WM8994_SPKOUTL_MUTE_N | TUNING_CALL_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);

		val = wm8994_read(codec,WM8994_SPEAKER_VOLUME_RIGHT);
		val &= ~(WM8994_SPKOUTR_MUTE_N_MASK | WM8994_SPKOUTR_VOL_MASK);
		val |= (WM8994_SPKOUT_VU | WM8994_SPKOUTR_MUTE_N | TUNING_CALL_SPKR_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);

		wm8994_write(codec, WM8994_CLASSD, TUNING_CALL_CLASSD_VOL);
	}

	//Unmute the DAC path
	val = wm8994_read(codec,WM8994_SPEAKER_MIXER );
	val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK | WM8994_DAC1R_TO_SPKMIXR_MASK);
	val |= ( WM8994_DAC1L_TO_SPKMIXL | WM8994_DAC1R_TO_SPKMIXR );
	wm8994_write(codec,WM8994_SPEAKER_MIXER ,val);

	val = wm8994_read(codec,WM8994_SPKOUT_MIXERS );
	val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | 
		WM8994_SPKMIXR_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
	val |= WM8994_SPKMIXL_TO_SPKOUTL;
	val |= WM8994_SPKMIXR_TO_SPKOUTR;
	wm8994_write(codec,WM8994_SPKOUT_MIXERS,val );

	wm8994_write(codec,0x36, 0x0003 );    
	/* Digital Path Enables and Unmutes*/	

	wm8994_write(codec,WM8994_SIDETONE, 0x01C0 );   


	wm8994_write(codec, WM8994_ANALOGUE_HP_1,0x0000); 
	wm8994_write(codec, WM8994_DC_SERVO_1,0x0000); 

	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1 );
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK |WM8994_SPKOUTL_ENA_MASK | WM8994_SPKOUTR_ENA_MASK);
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL | WM8994_SPKOUTL_ENA | WM8994_SPKOUTR_ENA);  
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_1,val);

	if((wm8994->testmode_config_flag == SEC_NORMAL)
		|| (wm8994->testmode_config_flag == SEC_TEST_PBA_LOOPBACK)
	)
	{
		//Unmute DAC1 left
		val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );
		val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL);
		wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);

		//Unmute and volume ctrl RightDAC
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
		val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
		val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume	
		wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);
	}		
 
	wm8994_write(codec,WM8994_DAC2_LEFT_VOLUME, 0x01C0 );   
	wm8994_write(codec,WM8994_AIF2_DAC_FILTERS_1, WM8994_AIF1DAC1_UNMUTE );   
	wm8994_write(codec,WM8994_AIF1_DAC1_FILTERS_1, WM8994_AIF1DAC2_UNMUTE );   

#endif

    wm8994_call_recording_change_path(codec);

    
}

void wm8994_set_voicecall_bluetooth(struct snd_soc_codec *codec)
{
#ifdef CONFIG_TARGET_LOCALE_KOR
	u16 val;

	wm8994_write(codec,0x39,0x0060);
	wm8994_write(codec,0x01,0x0003);
	msleep(50);

	//Enable Dac1 and DAC2 and the Timeslot0 for AIF1
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, 0x3000); // AIF2ADCL_ENA, ADCL_ENA
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5 ,0x330C);
	

/* ==================== Digital Path Configuration ==================== */
	wm8994_write(codec, 0x620, 0x0000 );
	wm8994_write(codec, 0x621, 0x01C0 ); // Digital Sidetone HPF, fcut-off = 370Hz
	
	wm8994_write(codec, 0x208, 0x000E ); // 0x000E -> 0x0007 for noise suppression // DSP1, DSP2 processing Enable, SYSCLK = AIF1CLK = 11.2896 MHz
	wm8994_write(codec, 0x210, 0x0073 ); // AIF1 Sample Rate = 44.1kHz AIF1CLK_RATE=256 => AIF1CLK = 11.2896 MHz
	wm8994_write(codec, 0x211, 0x0009 ); // AIF2 Sample Rate = 8kHz AIF2CLK_RATE=256 => AIF2CLK = 8k*256 = 2.048MHz
	wm8994_write(codec, 0x310, 0x4118 ); // DSP A mode, 16bit, BCLK2 invert

	wm8994_write(codec, 0x311, 0x0400); // AIF2_LOOPBACK

	// DIGITAL Master/Slave Mode
	wm8994_write(codec, 0x312, 0x0000 ); // SLAVE mode

	// FLL1 = 11.2896 MHz
	wm8994_write(codec, 0x220, 0x0005 ); // FLL1 Enable, FLL1 Fractional Mode
	wm8994_write(codec, 0x221, 0x0700 ); // FLL1_OUT_DIV = 8 = Fvco / Fout
	wm8994_write(codec, 0x222, 0x86C2 ); // FLL1_N = 34498
	wm8994_write(codec, 0x224, 0x0C88 ); // 0x0C88 -> 0x0C80 FLL1_CLK_REV_DIV = 1 = MCLK / FLL1, FLL1 = MCLK1
	wm8994_write(codec, 0x223, 0x00E0 ); // FLL1_N = 7, FLL1_GAIN = X1
	wm8994_write(codec, 0x200, 0x0011 ); // AIF1 Enable, AIF1CLK = FLL1 

	// FLL2 = 2.048MHz , fs = 8kHz
	wm8994_write(codec, 0x240, 0x0005 ); // FLL2 Enable, FLL2 Fractional Mode
	wm8994_write(codec, 0x241, 0x0700 ); // FLL2_OUTDIV = 23 = Fvco / Fout
//	wm8994_write(codec, 0x242, 0x3126 ); // FLL2_K = 12582
	wm8994_write(codec, 0x244, 0x0C83 ); // 0x0C88 -> 0x0C80 FLL2_CLK_REV_DIV = 1 = MCLK / FLL2, FLL2 = MCLK1
	wm8994_write(codec, 0x204, 0x0019 ); // AIF2CLK_SRC = FLL2, AIF2CLK_INV[2] = 0,  AIF2CLK_DIV[1] = 0, AIF2CLK_ENA[0] = 1
	wm8994_write(codec, 0x243, 0x0600 ); // FLL2_N[14:5] = 8, FLL2_GAIN[3:0] = X1

	wm8994_write(codec, 0x244, 0x0C83 ); // 0x0C88 -> 0x0C80 FLL2_CLK_REV_DIV = 1 = MCLK / FLL2, FLL2 = BCLK2
	wm8994_write(codec, 0x241, 0x0700 ); // FLL2_OUTDIV = 23 = Fvco / Fout
	wm8994_write(codec, 0x243, 0x0600 ); // FLL2_N[14:5] = 8, FLL2_GAIN[3:0] = X1
	wm8994_write(codec, 0x240, 0x0001 ); // FLL2 Enable, FLL2 Fractional Mode
	wm8994_write(codec, 0x208, 0x000E ); // 0x000E -> 0x0007 // DSP1, DSP2 processing Enable, SYSCLK = AIF1CLK = 11.2896 MHz
	wm8994_write(codec, 0x204, 0x0019 ); // AIF2CLK_SRC = FLL2, AIF2CLK_INV[2] = 0,  AIF2CLK_DIV[1] = 0, AIF2CLK_ENA[0] = 1
	wm8994_write(codec, 0x211, 0x0009 ); // AIF2 Sample Rate = 8kHz AIF2CLK_RATE=256 => AIF2CLK = 8k*256 = 2.048MHz
	wm8994_write(codec, 0x310, 0x4118 ); // DSP A mode, 16bit, BCLK2 invert

	wm8994_write(codec,0x620, 0x0000); // ADC oversampling disabled, DAC oversampling disabled


	// GPIO
	wm8994_write(codec, 0x700, 0xA101 );  // GPIO1 is Input Enable
	wm8994_write(codec, 0x701, 0x8100 ); // GPIO 2 - Set to MCLK2 input
	wm8994_write(codec, 0x702, 0x8100 ); // GPIO 3 - Set to BCLK2 input
	wm8994_write(codec, 0x703, 0x8100 ); // GPIO 4 - Set to DACLRCLK2 input
	wm8994_write(codec, 0x704, 0x8100 ); // GPIO 5 - Set to DACDAT2 input
	wm8994_write(codec, 0x705, 0xA101 ); // GPIO 6 is Input Enable
	wm8994_write(codec, 0x706, 0x0100 ); // GPIO 7 - Set to ADCDAT2 output

	wm8994_write(codec, 0x603, 0x000C ); // ADC2_DAC2_VOL2[8:5] = -36dB(0000) ADC1_DAC2_VOL[3:0] = 0dB(1100), 
	wm8994_write(codec, 0x604, 0x0010 ); // ADC1_TO_DAC2L[4] = Enable(1)


	//-------------------------------------- Rx Path
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, 0x0000 ); // unmute DAC1L AND DAC1R
	wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, 0x0000 ); // unmute AIF2DAC, 
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, 0x0005 ); // AIF2DAC2L_TO DAC1L, AIF1DAC1L_TO_DAC1L
	wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, 0x0005 ); // AIF2DAC2R_TO_DAC1R, AIF1DAC1R_TO_DAC1R
//	wm8994_write(codec, 0x603, 0x018C ); // ADC2_DAC2_VOL2[8:5] = -36dB(0000) ADC1_DAC2_VOL[3:0] = 0dB(1100), 
	wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, 0x0005 ); // ADC1_TO_DAC2L[4] = Enable(1)
	wm8994_write(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING, 0x0005 ); // ADC1_TO_DAC2L[4] = Enable(1)

	wm8994_write(codec, WM8994_DAC2_LEFT_VOLUME, 0x1C0 );
	wm8994_write(codec, WM8994_DAC2_RIGHT_VOLUME, 0x1C0 ); 

/* ==================== Bias Configuration ==================== */
	//Enbale bias,vmid and Left speaker
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1 );
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK | WM8994_SPKOUTL_ENA_MASK | WM8994_SPKOUTR_ENA_MASK | WM8994_HPOUT1L_ENA_MASK |WM8994_HPOUT1R_ENA_MASK);
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL);  
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_1,val);
	
/* ==================== Digital Paths Configuration ==================== */


	// Unmute the AF1DAC1
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1 ); 	
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK);
	val |= WM8994_AIF1DAC1_UNMUTE;
	wm8994_write(codec,WM8994_AIF1_DAC1_FILTERS_1 ,val);
	// Enable the Timeslot0 to DAC1L
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING  ); 	
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= WM8994_AIF1DAC1L_TO_DAC1L;
	wm8994_write(codec,WM8994_DAC1_LEFT_MIXER_ROUTING ,val);
	//Enable the Timeslot0 to DAC1R
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING  ); 	
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
	val |= WM8994_AIF1DAC1R_TO_DAC1R;
	wm8994_write(codec,WM8994_DAC1_RIGHT_MIXER_ROUTING ,val);

	// Enable the Timeslot0 to DAC2L
	val = wm8994_read(codec, WM8994_DAC2_LEFT_MIXER_ROUTING  ); 	
	val &= ~(WM8994_AIF1DAC1L_TO_DAC2L_MASK);
	val |= WM8994_AIF1DAC1L_TO_DAC2L;
	wm8994_write(codec,WM8994_DAC2_LEFT_MIXER_ROUTING ,val);
	//Enable the Timeslot0 to DAC2R
	val = wm8994_read(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING  ); 	
	val &= ~(WM8994_AIF1DAC1R_TO_DAC2R_MASK);
	val |= WM8994_AIF1DAC1R_TO_DAC2R;
	wm8994_write(codec,WM8994_DAC2_RIGHT_MIXER_ROUTING ,val);


	//Unmute LeftDAC1
	val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME );
	val &= ~(WM8994_DAC1L_MUTE_MASK);
	wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME,val);
	//Unmute RightDAC1
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
	val &= ~(WM8994_DAC1R_MUTE_MASK);
	wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);

	//Unmute LeftDAC1
	val = wm8994_read(codec, WM8994_DAC2_LEFT_VOLUME );
	val &= ~(WM8994_DAC2L_MUTE_MASK);
	wm8994_write(codec,WM8994_DAC2_LEFT_VOLUME,val);
	//Unmute RightDAC1
	val = wm8994_read(codec, WM8994_DAC2_RIGHT_VOLUME ); 
	val &= ~(WM8994_DAC2R_MUTE_MASK);
	wm8994_write(codec,WM8994_DAC2_RIGHT_VOLUME,val);
	

/* ==================== Output Path Configuration ==================== */
/* //kevin : sleep current in call
	//SPKMIXL, SPKLVOL PGA enable
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_3 );
	val &= ~( WM8994_SPKLVOL_ENA_MASK | WM8994_SPKRVOL_ENA_MASK);
	val |= (WM8994_SPKLVOL_ENA | WM8994_SPKRVOL_ENA);
	wm8994_write(codec,WM8994_POWER_MANAGEMENT_3 , val);
	//SPKMIXL, R
	val = wm8994_read(codec, WM8994_SPEAKER_MIXER);
	val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK | WM8994_DAC1R_TO_SPKMIXR_MASK | WM8994_DAC2L_TO_SPKMIXL_MASK | WM8994_DAC2R_TO_SPKMIXR_MASK);
	val |= (WM8994_DAC2L_TO_SPKMIXL | WM8994_DAC2R_TO_SPKMIXR);
	wm8994_write(codec, WM8994_SPEAKER_MIXER, val);
	// SPKLVOL, SPKRVOL
	val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
	val &= ~(WM8994_SPKOUTL_MUTE_N_MASK);
	val |= WM8994_SPKOUTL_MUTE_N;
	wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);
	val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_RIGHT);
	val &= ~(WM8994_SPKOUTR_MUTE_N_MASK);
	val |= WM8994_SPKOUTR_MUTE_N;
	wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);
	// SPKOUTLBOOST
	val = wm8994_read(codec, WM8994_SPKOUT_MIXERS);
	val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTL_MASK);
	val |= (WM8994_SPKMIXL_TO_SPKOUTL | WM8994_SPKMIXR_TO_SPKOUTL);
	wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);
*/
/* ==================== Set default Gain ==================== */
	//-------------------------------------- Digital
	// AIF1_DAC1_VOL 1st step
	val = wm8994_read(codec, WM8994_AIF1_DAC1_LEFT_VOLUME ); 
	val &= ~(WM8994_AIF1DAC1_VU_MASK | WM8994_AIF1DAC1L_VOL_MASK);
	val |= (WM8994_AIF1DAC1_VU | 0xC0); //0 dB volume 	
	wm8994_write(codec,WM8994_AIF1_DAC1_LEFT_VOLUME, val);
	
	val = wm8994_read(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME ); 
	val &= ~(WM8994_AIF1DAC1_VU_MASK | WM8994_AIF1DAC1R_VOL_MASK);
	val |= (WM8994_AIF1DAC1_VU | 0xC0); //0 dB volume 	
	wm8994_write(codec,WM8994_AIF1_DAC1_RIGHT_VOLUME, val);


	// DAC1_VOL 2nd step
	val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME ); 
	val &= ~(WM8994_DAC1_VU_MASK | WM8994_DAC1L_VOL_MASK);
	val |= (WM8994_DAC1_VU | 0xC0); //0 dB volume 	
	wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME,val);

	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
	val &= ~(WM8994_DAC1_VU_MASK | WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU | 0xC0); //0 dB volume 	
	wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);


/*//kevin : sleep current in call

	//-------------------------------------- Analog Output
	// SPKMIXL, R VOL unmute
	val = wm8994_read(codec,WM8994_SPKMIXL_ATTENUATION);
	val &= ~(WM8994_DAC1L_SPKMIXL_VOL_MASK | WM8994_SPKMIXL_VOL_MASK);
	val |= 0x0; 
	wm8994_write(codec,WM8994_SPKMIXL_ATTENUATION  ,val);

	val = wm8994_read(codec,WM8994_SPKMIXR_ATTENUATION);
	val &= ~(WM8994_DAC1R_SPKMIXR_VOL_MASK | WM8994_SPKMIXR_VOL_MASK);
	val |= 0x0; 
	wm8994_write(codec,WM8994_SPKMIXR_ATTENUATION  ,val);

	// SPKLVOL, SPKRVOL
	val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
	val &= ~(WM8994_SPKOUT_VU_MASK | WM8994_SPKOUTL_VOL_MASK);
	val |= (WM8994_SPKOUT_VU | 0x3F);

	wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);
	val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_RIGHT);
	val &= ~(WM8994_SPKOUT_VU_MASK | WM8994_SPKOUTR_VOL_MASK);
	val |= (WM8994_SPKOUT_VU | 0x3F);
	wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);

	// SPKOUTLBOOST
	val = wm8994_read(codec, WM8994_CLASSD);
	val &= ~(WM8994_SPKOUTL_BOOST_MASK);
	val |= 0x0010; // 3 dB
	wm8994_write(codec, WM8994_CLASSD, val);
*/

	wm8994_write(codec, 0x06, 0x000C );  
	wm8994_write(codec, 0x700, 0xA101 ); 
	wm8994_write(codec, 0x702, 0x8100 ); 
	wm8994_write(codec, 0x703, 0x8100 ); 
	wm8994_write(codec, 0x704, 0x8100 ); 
	wm8994_write(codec, 0x705, 0xA101 ); 
	wm8994_write(codec, 0x706, 0x0100 ); 
	wm8994_write(codec, 0x707, 0x8100 ); 
	wm8994_write(codec, 0x708, 0x0100 ); 
	wm8994_write(codec, 0x709, 0x0100 ); 
	wm8994_write(codec, 0x70A, 0x0100 ); 

#else


	int val;
	
	DEBUG_LOG("");

	wm8994_set_voicecall_common_setting(codec);

	/*GPIO Configuration*/		
	wm8994_write(codec, WM8994_GPIO_8, WM8994_GP8_DIR | WM8994_GP8_DB);
	wm8994_write(codec, WM8994_GPIO_9, WM8994_GP9_DB);
	wm8994_write(codec, WM8994_GPIO_10, WM8994_GP10_DB);
	wm8994_write(codec, WM8994_GPIO_11, WM8994_GP11_DB);

	/*Digital Path Enables and Unmutes*/		 
	val  = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_SPKOUTL_ENA_MASK | WM8994_SPKOUTR_ENA_MASK | WM8994_HPOUT2_ENA_MASK | WM8994_HPOUT1L_ENA_MASK | WM8994_HPOUT1R_ENA_MASK);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, WM8994_AIF2ADCL_ENA | WM8994_ADCL_ENA);

	// If Input MIC is enabled, bluetooth Rx is muted.
	wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, WM8994_IN1L_MUTE);
	wm8994_write(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME, WM8994_IN1R_MUTE);
	wm8994_write(codec, WM8994_INPUT_MIXER_2, 0x00);
	wm8994_write(codec, WM8994_INPUT_MIXER_3, 0x00);
	wm8994_write(codec, WM8994_INPUT_MIXER_4, 0x00);
	
	//for BT DTMF Play
	//Rx Path: AIF2ADCDAT2 select
	//CP(CALL) Path:GPIO5/DACDAT2 select
	//AP(DTMF) Path: DACDAT1 select
	//Tx Path: GPIO8/DACDAT3 select

	wm8994_write(codec, WM8994_POWER_MANAGEMENT_6, 0x000C);
	
	// AIF1 & AIF2 Output is connected to DAC1	
	wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, WM8994_AIF2DACL_TO_DAC2L | WM8994_AIF1DAC1L_TO_DAC2L);
	wm8994_write(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING, WM8994_AIF2DACR_TO_DAC2R | WM8994_AIF1DAC1R_TO_DAC2R);

	wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0019);

	wm8994_write(codec, WM8994_DAC2_MIXER_VOLUMES, 0x000C); 

	wm8994_write(codec, WM8994_DAC2_LEFT_VOLUME, 0x00C0);
	wm8994_write(codec, WM8994_DAC2_RIGHT_VOLUME, 0x01C0); 

	wm8994_write(codec, WM8994_OVERSAMPLING, 0X0000);

	//Unmute DAC1 left
	val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );
	val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL);
	wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);
	
	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
	val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume	
	wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);
	
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, 0x0000);  
	wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, 0x0000); 
	
#endif
}


void wm8994_set_voicecall_record(struct snd_soc_codec *codec, int channel)
{
        struct wm8994_priv *wm8994 = codec->drvdata;
        u16 val;

        DEBUG_LOG("channel = [%d]", channel);


        if(wm8994 ->call_record_path == CALL_RECORDING_MAIN )
        {
                switch(channel)
                {
                        case CH_UDLINK:  //TxRx
                                val = wm8994_read(codec,WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING );
                                val &= ~( WM8994_ADC1L_TO_AIF1ADC1L_MASK | WM8994_AIF2DACL_TO_AIF1ADC1L_MASK);	
                                val |= (WM8994_ADC1L_TO_AIF1ADC1L | WM8994_AIF2DACL_TO_AIF1ADC1L );  
                                wm8994_write(codec,WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING ,val);

                                val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
                                val &= ~( WM8994_AIF1ADC1L_ENA_MASK | WM8994_AIF1ADC1R_ENA_MASK);
                                val |= ( WM8994_AIF1ADC1L_ENA|WM8994_AIF1ADC1R_ENA);
                                wm8994_write(codec,WM8994_POWER_MANAGEMENT_4,val);	

                                wm8994_write(codec,0x300,0x4010);	

                                wm8994_write(codec,0x400,0x00C0);	// AIF1 INPUT VOLUME : 0x00C0:0db
                        break;

                        case CH_UPLINK:  //Tx only
                                val = wm8994_read(codec,WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING );
                                val &= ~( WM8994_ADC1L_TO_AIF1ADC1L_MASK | WM8994_AIF2DACL_TO_AIF1ADC1L_MASK);	
                                val |= (WM8994_ADC1L_TO_AIF1ADC1L);  
                                wm8994_write(codec,WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING ,val);

                                val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
                                val &= ~( WM8994_AIF1ADC1L_ENA_MASK | WM8994_AIF1ADC1R_ENA_MASK);
                                val |= ( WM8994_AIF1ADC1L_ENA|WM8994_AIF1ADC1R_ENA);
                                wm8994_write(codec,WM8994_POWER_MANAGEMENT_4,val);	

                                wm8994_write(codec,0x300,0x4010);	

                                wm8994_write(codec,0x400,0x00C0);	// AIF1 INPUT VOLUME : 0x00C0:0db	
                        break;

                        case CH_DOWNLINK:  //Rx only
                                val = wm8994_read(codec,WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING );
                                val &= ~( WM8994_ADC1L_TO_AIF1ADC1L_MASK | WM8994_AIF2DACL_TO_AIF1ADC1L_MASK);	
                                val |= (WM8994_AIF2DACL_TO_AIF1ADC1L );  
                                wm8994_write(codec,WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING ,val);

                                val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
                                val &= ~( WM8994_AIF1ADC1L_ENA_MASK | WM8994_AIF1ADC1R_ENA_MASK);
                                val |= ( WM8994_AIF1ADC1L_ENA|WM8994_AIF1ADC1R_ENA);
                                wm8994_write(codec,WM8994_POWER_MANAGEMENT_4,val);	

                                wm8994_write(codec,0x300,0x4010);	

                                wm8994_write(codec,0x400,0x00C0);	// AIF1 INPUT VOLUME : 0x00C0:0db	
                        break;

                        default:
                                DEBUG_LOG("Recording through Main Mic in call");
                                audio_ctrl_mic_bias_gpio(1);
            
                                val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
                                val &= ~(WM8994_ADCL_ENA_MASK |WM8994_AIF1ADC1L_ENA_MASK);
                                val |= (WM8994_AIF1ADC1L_ENA | WM8994_ADCL_ENA);
                                wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

                                //Enable timeslots
                                val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
                                val |=WM8994_ADC1L_TO_AIF1ADC1L;  
                                wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);

                                val = wm8994_read(codec, WM8994_AIF1_MASTER_SLAVE);
                                val |= (WM8994_AIF1_MSTR | WM8994_AIF1_CLK_FRC | WM8994_AIF1_LRCLK_FRC);	// Master mode
                                wm8994_write(codec, WM8994_AIF1_MASTER_SLAVE, val);

                                wm8994_write(codec, WM8994_GPIO_1, 0xA101);   // GPIO1 is Input Enable

                                // for stable pcm input when start google voice recognition
                                if(wm8994->recognition_active == REC_ON) 
                                {
                                        msleep(300);
                                }
                        break;
                }
        }
        else if(wm8994 ->call_record_path == CALL_RECORDING_SUB)
        {
                switch(channel)
                {
                        case CH_UDLINK:  //TxRx
                                val = wm8994_read(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
                                val &= ~( WM8994_ADC1R_TO_AIF1ADC1R_MASK | WM8994_AIF2DACR_TO_AIF1ADC1R_MASK);
                                val |= (WM8994_ADC1R_TO_AIF1ADC1R |WM8994_AIF2DACR_TO_AIF1ADC1R);
                                wm8994_write(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING,val);

                                val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
                                val &= ~( WM8994_AIF1ADC1L_ENA_MASK | WM8994_AIF1ADC1R_ENA_MASK);
                                val |= ( WM8994_AIF1ADC1L_ENA|WM8994_AIF1ADC1R_ENA);
                                wm8994_write(codec,WM8994_POWER_MANAGEMENT_4,val);	

                                wm8994_write(codec,0x300,0xC010);	

                                wm8994_write(codec,0x401,0x00C0);	// AIF1 INPUT VOLUME : 0x00C0:0db	
                        break;

                        case CH_UPLINK:  //Tx only
                                val = wm8994_read(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
                                val &= ~( WM8994_ADC1R_TO_AIF1ADC1R_MASK | WM8994_AIF2DACR_TO_AIF1ADC1R_MASK);
                                val |= (WM8994_ADC1R_TO_AIF1ADC1R );
                                wm8994_write(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING,val);

                                val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
                                val &= ~( WM8994_AIF1ADC1L_ENA_MASK | WM8994_AIF1ADC1R_ENA_MASK);
                                val |= ( WM8994_AIF1ADC1L_ENA|WM8994_AIF1ADC1R_ENA);
                                wm8994_write(codec,WM8994_POWER_MANAGEMENT_4,val);	

                                wm8994_write(codec,0x300,0xC010);	

                                wm8994_write(codec,0x401,0x00C0);	// AIF1 INPUT VOLUME : 0x00C0:0db	
                        break;		

                        case CH_DOWNLINK:  //Rx only
                                val = wm8994_read(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
                                val &= ~( WM8994_ADC1R_TO_AIF1ADC1R_MASK | WM8994_AIF2DACR_TO_AIF1ADC1R_MASK);
                                val |= (WM8994_AIF2DACR_TO_AIF1ADC1R);
                                wm8994_write(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING,val);

                                val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
                                val &= ~( WM8994_AIF1ADC1L_ENA_MASK | WM8994_AIF1ADC1R_ENA_MASK);
                                val |= ( WM8994_AIF1ADC1L_ENA|WM8994_AIF1ADC1R_ENA);
                                wm8994_write(codec,WM8994_POWER_MANAGEMENT_4,val);	

                                wm8994_write(codec,0x300,0xC010);	

                                wm8994_write(codec,0x401,0x00C0);	// AIF1 INPUT VOLUME : 0x00C0:0db	               
                        break;

                        default:
                                DEBUG_LOG("Recording through Headset Mic in call");

                                audio_ctrl_mic_bias_gpio(0);	
                                audio_ctrl_earmic_bias_gpio(1);

                                // Mixing left channel output to right channel.
                                val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);	//605H : 0x0010
                                val &= ~(WM8994_AIF1ADCL_SRC_MASK | WM8994_AIF1ADCR_SRC_MASK);
                                val |= (WM8994_AIF1ADCL_SRC | WM8994_AIF1ADCR_SRC);
                                wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

                                //Digital Paths	
                                //Enable right ADC and time slot
                                val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_4);
                                val &= ~(WM8994_ADCR_ENA_MASK |WM8994_AIF1ADC1R_ENA_MASK );
                                val |= (WM8994_AIF1ADC1R_ENA | WM8994_ADCR_ENA  );
                                wm8994_write(codec,WM8994_POWER_MANAGEMENT_4 ,val);

                                //ADC Right mixer routing
                                val = wm8994_read(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
                                val &= ~( WM8994_ADC1R_TO_AIF1ADC1R_MASK);
                                val |= WM8994_ADC1R_TO_AIF1ADC1R;
                                wm8994_write(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING,val);

                                val = wm8994_read(codec, WM8994_AIF1_MASTER_SLAVE);
                                val |= (WM8994_AIF1_MSTR | WM8994_AIF1_CLK_FRC | WM8994_AIF1_LRCLK_FRC);	// Master mode
                                wm8994_write(codec, WM8994_AIF1_MASTER_SLAVE, val);

                                wm8994_write( codec, WM8994_GPIO_1, 0xA101 );   // GPIO1 is Input Enable
                        break;
                }	
        }
}

void wm8994_call_recording_change_path(struct snd_soc_codec *codec)
{
        struct wm8994_priv *wm8994 = codec->drvdata;

        if(wm8994->rec_path == MAIN)
        {
                wm8994->call_record_path = CALL_RECORDING_MAIN;
                DEBUG_LOG("Changing rec_path[%d] -> call_record_path [%d]", wm8994 ->rec_path, wm8994 ->call_record_path);           
                
                wm8994->rec_path = MIC_OFF;
        }
        else if(wm8994->rec_path == SUB)
        {
                wm8994->call_record_path = CALL_RECORDING_SUB;
                DEBUG_LOG("Changing rec_path[%d] -> call_record_path [%d]", wm8994 ->rec_path, wm8994 ->call_record_path);
                
                wm8994->rec_path = MIC_OFF;
        }


        if(wm8994 ->call_record_path != CALL_RECORDING_OFF)
        {
                DEBUG_LOG("wm8994 ->call_record_path = [%d]", wm8994 ->call_record_path);    
                wm8994_set_voicecall_record(codec, (int)wm8994 ->call_record_ch);
        }
}

void wm8994_set_voicecall_record_off(struct snd_soc_codec *codec)
{
	DEBUG_LOG("");

	//codec off
	wm8994_write(codec,WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING ,0x0000);
	wm8994_write(codec,WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING,0x0000);
}

void wm8994_set_fmradio_common(struct snd_soc_codec *codec, int onoff)
{	
	struct wm8994_priv *wm8994 = codec->drvdata;
	
	u16 val;

	DEBUG_LOG("onoff = [%d]", onoff);

	wm8994_write(codec, 0x39, 0x8);	//Cross Talk (H/W requested)

	if(onoff)
	{
		val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_3_4_VOLUME); 	
		val &= ~(WM8994_IN2L_VU_MASK | WM8994_IN2L_MUTE_MASK | WM8994_IN2L_VOL_MASK);
		if(wm8994->fmradio_path == FMR_HP)
			val |= (TUNING_FMRADIO_EAR_INPUTMIXL_VOL);
		else
			val |= (TUNING_FMRADIO_SPK_INPUTMIXL_VOL);
		wm8994_write(codec, WM8994_LEFT_LINE_INPUT_3_4_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_LINE_INPUT_3_4_VOLUME); 	
		val &= ~(WM8994_IN2R_VU_MASK | WM8994_IN2R_MUTE_MASK | WM8994_IN2R_VOL_MASK);
		if(wm8994->fmradio_path == FMR_HP)
			val |= (WM8994_IN2R_VU | TUNING_FMRADIO_EAR_INPUTMIXR_VOL);
		else
			val |= (WM8994_IN2R_VU | TUNING_FMRADIO_SPK_INPUTMIXR_VOL);
		wm8994_write(codec, WM8994_RIGHT_LINE_INPUT_3_4_VOLUME, val);

		// Input mixer setting - Temporary inserted for blocking MIC and FM radio mixing - DW Shim 2010.02.25
	//	val = wm8994_read(codec, WM8994_INPUT_MIXER_2);
	//	val &= ~(WM8994_IN2LN_TO_IN2L_MASK | WM8994_IN2RN_TO_IN2R_MASK);
	//	val |= (WM8994_IN2LN_TO_IN2L | WM8994_IN2RN_TO_IN2R);
		val = (WM8994_IN2LN_TO_IN2L | WM8994_IN2RN_TO_IN2R);
		wm8994_write(codec, WM8994_INPUT_MIXER_2, val); 	

		if(wm8994->testmode_config_flag == SEC_NORMAL)
		{
			// IN2L to MIXINL
			val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
			val &= ~(WM8994_IN2L_TO_MIXINL_MASK);
			val |= WM8994_IN2L_TO_MIXINL;
			wm8994_write(codec, WM8994_INPUT_MIXER_3, val);

			//IN2R to MIXINR
			val = wm8994_read(codec, WM8994_INPUT_MIXER_4);
			val &= ~(WM8994_IN2R_TO_MIXINR_MASK);
			val |= WM8994_IN2R_TO_MIXINR;
			wm8994_write(codec, WM8994_INPUT_MIXER_4, val);	
		}
		
		//DRC for Noise-gate (AIF2)
		wm8994_write(codec, WM8994_AIF2_ADC_FILTERS, 0xF800);
		wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, 0x0036);
		wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_2, 0x0010);
		wm8994_write(codec, WM8994_AIF2_DRC_2, 0x0840);
		wm8994_write(codec, WM8994_AIF2_DRC_3, 0x2400);
		wm8994_write(codec, WM8994_AIF2_DRC_4, 0x0000);
		wm8994_write(codec, WM8994_AIF2_DRC_5, 0x0000);
		wm8994_write(codec, WM8994_AIF2_DRC_1, 0x019C);
	}
	else
	{		
		val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_3_4_VOLUME);	
		val &= ~(WM8994_IN2L_VU_MASK | WM8994_IN2L_MUTE_MASK | WM8994_IN2L_VOL_MASK);
		val |= (WM8994_IN2L_MUTE);
		wm8994_write(codec, WM8994_LEFT_LINE_INPUT_3_4_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_LINE_INPUT_3_4_VOLUME);
		val &= ~(WM8994_IN2R_VU_MASK | WM8994_IN2R_MUTE_MASK | WM8994_IN2R_VOL_MASK);
		val |= (WM8994_IN2R_VU | WM8994_IN2R_MUTE);
		wm8994_write(codec, WM8994_RIGHT_LINE_INPUT_3_4_VOLUME, val);

		val = wm8994_read(codec, WM8994_INPUT_MIXER_2);
		val &= ~(WM8994_IN2LN_TO_IN2L | WM8994_IN2RN_TO_IN2R);
		wm8994_write(codec, WM8994_INPUT_MIXER_2, val);
		
		if(wm8994->testmode_config_flag == SEC_NORMAL)
		{
			// IN2L to MIXINL
			val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
			val &= ~(WM8994_IN2L_TO_MIXINL_MASK);
			wm8994_write(codec, WM8994_INPUT_MIXER_3, val);

			//IN2R to MIXINR
			val = wm8994_read(codec, WM8994_INPUT_MIXER_4);
			val &= ~(WM8994_IN2R_TO_MIXINR_MASK);
			wm8994_write(codec, WM8994_INPUT_MIXER_4, val);	
		}
	}		
}

void wm8994_set_fmradio_headset(struct snd_soc_codec *codec)
{	
	struct wm8994_priv *wm8994 = codec->drvdata;

	u16 val;
	
	u16 nReadServo4Val = 0;
	u16 ncompensationResult = 0;
	u16 nCompensationResultLow=0;
	u16 nCompensationResultHigh=0;
	u8  nServo4Low = 0;
	u8  nServo4High = 0;
	
	DEBUG_LOG("Routing ear path : FM Radio -> EAR Out");

	wm8994->fmradio_path = FMR_HP;

	wm8994_disable_fmradio_path(codec, FMR_SPK);

	//DAC1 Setting
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);	//601H : 0x05
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK | WM8994_AIF1DAC2L_TO_DAC1L_MASK | WM8994_AIF2DACL_TO_DAC1L_MASK);
	val |= (WM8994_AIF2DACL_TO_DAC1L | WM8994_AIF1DAC1L_TO_DAC1L);
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);

	val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING);	//602H : 0x05
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK | WM8994_AIF1DAC2R_TO_DAC1R_MASK | WM8994_AIF2DACR_TO_DAC1R_MASK);
	val |= (WM8994_AIF1DAC1R_TO_DAC1R | WM8994_AIF2DACR_TO_DAC1R);
	wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, val);

	//* Headset
	wm8994_write(codec, 0x102, 0x0003);
	wm8994_write(codec, 0x56, 0x0003);
	wm8994_write(codec, 0x102, 0x0000);

	wm8994_write(codec, WM8994_GPIO_3, 0x0100);
	wm8994_write(codec, WM8994_GPIO_4, 0x0100);
	wm8994_write(codec, WM8994_GPIO_5, 0x8100);
	wm8994_write(codec, WM8994_GPIO_6, 0xA101);
	wm8994_write(codec, WM8994_GPIO_7, 0x0100);

	val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME); 	
	val = 0x0000;
	wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

	val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME); 	
	val = 0x0100;
	wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);
	
	// Disable reg sync to MCLK
	val = wm8994_read(codec, WM8994_AIF1_CLOCKING_1); 	
	val &= ~(WM8994_AIF1CLK_ENA_MASK);
	val |= WM8994_AIF1CLK_ENA;
	wm8994_write(codec, WM8994_AIF1_CLOCKING_1, val);


	// Analogue Path Config
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2  ); 	
	val &= ~(WM8994_MIXINL_ENA_MASK | WM8994_MIXINR_ENA_MASK | WM8994_IN2L_ENA_MASK | WM8994_IN2R_ENA_MASK);
	val |= (WM8994_MIXINL_ENA | WM8994_MIXINR_ENA| WM8994_IN2L_ENA| WM8994_IN2R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_2 , val );

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1); 	
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_NORMAL);
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1 , 0x0003);
		
	//* Unmutes
	// Output setting
	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME); 	
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK | WM8994_HPOUT1L_VOL_MASK);
		val |= (WM8994_HPOUT1L_MUTE_N | TUNING_FMRADIO_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME); 	
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK | WM8994_HPOUT1R_VOL_MASK);
		val |= (WM8994_HPOUT1_VU | WM8994_HPOUT1R_MUTE_N | TUNING_FMRADIO_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK | WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUTL_MUTE_N | TUNING_FMRADIO_OPGAL_VOL);
		wm8994_write(codec,WM8994_LEFT_OPGA_VOLUME, val );

		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK | WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU | WM8994_MIXOUTR_MUTE_N | TUNING_FMRADIO_OPGAR_VOL);
		wm8994_write(codec,WM8994_RIGHT_OPGA_VOLUME, val );

		val = wm8994_read(codec, WM8994_AIF1_DAC1_LEFT_VOLUME);
		val &= ~(WM8994_AIF1DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_RADIO_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_LEFT_VOLUME, val);

		val = wm8994_read(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME);
		val &= ~(WM8994_AIF1DAC1R_VOL_MASK);
		val |= (WM8994_AIF1DAC1_VU |TUNING_DAC1R_RADIO_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME, val);
	}

	wm8994_set_fmradio_common(codec, 1);
	
	//FLL2 Setting
	val = wm8994_read(codec, WM8994_AIF2_CLOCKING_1);	//204H : 0x0011
	val &= ~(WM8994_AIF2CLK_ENA_MASK | WM8994_AIF2CLK_SRC_MASK);
	val |= (WM8994_AIF2CLK_ENA | 0x2 << WM8994_AIF2CLK_SRC_SHIFT);
	wm8994_write(codec, WM8994_AIF2_CLOCKING_1, val);

	val = wm8994_read(codec, WM8994_CLOCKING_1);	//208H : 0xF -> 0xE
	val &= ~(WM8994_SYSCLK_SRC_MASK | WM8994_DSP_FSINTCLK_ENA_MASK | WM8994_DSP_FS2CLK_ENA_MASK | WM8994_DSP_FS1CLK_ENA_MASK);
	val |= (WM8994_DSP_FS1CLK_ENA | WM8994_DSP_FS2CLK_ENA | WM8994_DSP_FSINTCLK_ENA);
	wm8994_write(codec, WM8994_CLOCKING_1, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);	//04H
	val &= ~(WM8994_AIF2ADCL_ENA_MASK | WM8994_AIF2ADCR_ENA_MASK | WM8994_ADCL_ENA_MASK | WM8994_ADCR_ENA_MASK);
	val |= (WM8994_AIF2ADCL_ENA | WM8994_AIF2ADCR_ENA | WM8994_ADCL_ENA | WM8994_ADCR_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

	//DAC2 Setting
	wm8994_write(codec, WM8994_DAC2_MIXER_VOLUMES, 0x018C);	//603H : 0x018C

	val = wm8994_read(codec, WM8994_DAC2_LEFT_MIXER_ROUTING);	//604H : 0x0010
	val &= ~(WM8994_ADC1_TO_DAC2L_MASK);
	val |= (WM8994_ADC1_TO_DAC2L);
	wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, val);

	val = wm8994_read(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING);	//605H : 0x0010
	val &= ~(WM8994_ADC2_TO_DAC2R_MASK);
	val |= (WM8994_ADC2_TO_DAC2R);			// Changed value to support stereo
	wm8994_write(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING, val);

	val = wm8994_read(codec, WM8994_DAC2_LEFT_VOLUME);	//612 : 1C0
	val &= ~(WM8994_DAC2L_MUTE_MASK | WM8994_DAC2L_VOL_MASK);
	val |= (TUNING_DAC2L_VOL); //0 db volume	
	wm8994_write(codec,WM8994_DAC2_LEFT_VOLUME,val);
	
	val = wm8994_read(codec, WM8994_DAC2_RIGHT_VOLUME);	//613 : 1C0
	val &= ~(WM8994_DAC2R_MUTE_MASK | WM8994_DAC2R_VOL_MASK);
	val |= (WM8994_DAC2_VU | TUNING_DAC2R_VOL); //0 db volume	
	wm8994_write(codec, WM8994_DAC2_RIGHT_VOLUME, val);

	wm8994_write(codec, WM8994_AIF1_DAC1_EQ_GAINS_1, 0x0000);	//480 : 0
	 
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);	//03 : F
	val &= ~(WM8994_MIXOUTLVOL_ENA_MASK | WM8994_MIXOUTRVOL_ENA_MASK | WM8994_MIXOUTL_ENA_MASK | WM8994_MIXOUTR_ENA_MASK);
	val |= (WM8994_MIXOUTLVOL_ENA | WM8994_MIXOUTRVOL_ENA | WM8994_MIXOUTL_ENA | WM8994_MIXOUTR_ENA);	
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

	wm8994_write(codec, WM8994_OVERSAMPLING, 0x0000);
	
//AIF2 Master/Slave , LOOPBACK, AIF2DAC Unmute
	val = wm8994_read(codec, WM8994_AIF2_MASTER_SLAVE);	//312 : 7000
	val &= ~(WM8994_AIF2_LRCLK_FRC_MASK | WM8994_AIF2_CLK_FRC_MASK | WM8994_AIF2_MSTR_MASK);
	val |= (WM8994_AIF2_LRCLK_FRC | WM8994_AIF2_CLK_FRC | WM8994_AIF2_MSTR);
	wm8994_write(codec, WM8994_AIF2_MASTER_SLAVE, val);

	val = wm8994_read(codec, WM8994_AIF2_CONTROL_2);	//311 : 4001
	val &= ~(WM8994_AIF2_LOOPBACK_MASK);
	val |= (WM8994_AIF2_LOOPBACK);
	wm8994_write(codec, WM8994_AIF2_CONTROL_2, val);

	wm8994_write(codec, WM8994_SIDETONE, 0x01c0);

	//* DC Servo Series Count
	val = 0x03E0;
	wm8994_write(codec, WM8994_DC_SERVO_2, val);

	//* HP first and second stage
	//Enable vmid,bias, hp left and right
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1 );
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK |WM8994_HPOUT1L_ENA_MASK |WM8994_HPOUT1R_ENA_MASK);
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL |WM8994_HPOUT1R_ENA	 |WM8994_HPOUT1L_ENA);  
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	val = (WM8994_HPOUT1L_DLY | WM8994_HPOUT1R_DLY);
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	//Enable Charge Pump	
	val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
	val &= ~WM8994_CP_ENA_MASK ;
	val |= WM8994_CP_ENA | WM8994_CP_ENA_DEFAULT ; // this is from wolfson  	
	wm8994_write(codec, WM8994_CHARGE_PUMP_1, 0x9F25);

	msleep(5);

	//Digital  Mixer setting
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);	//05 : 3303
	val &= ~(WM8994_AIF2DACL_ENA_MASK | WM8994_AIF2DACR_ENA_MASK | WM8994_AIF1DAC1L_ENA_MASK | WM8994_AIF1DAC1R_ENA_MASK
		 | WM8994_DAC1L_ENA_MASK | WM8994_DAC1R_ENA_MASK);
	val |= (WM8994_AIF2DACL_ENA | WM8994_AIF2DACR_ENA | WM8994_AIF1DAC1L_ENA | WM8994_AIF1DAC1R_ENA
		 | WM8994_DAC1L_ENA | WM8994_DAC1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);	//2D : 1
	val &= ~(WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |= (WM8994_DAC1L_TO_MIXOUTL);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val);

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);	//2E : 1
	val &= ~(WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= (WM8994_DAC1R_TO_MIXOUTR);	
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

	//* DC Servo 
	val = (WM8994_DCS_TRIG_SERIES_1 | WM8994_DCS_TRIG_SERIES_0 | WM8994_DCS_ENA_CHAN_1 | WM8994_DCS_ENA_CHAN_0);
	wm8994_write(codec,WM8994_DC_SERVO_1, 0x0303 );

	msleep(160);

	nReadServo4Val=wm8994_read(codec,WM8994_DC_SERVO_4);
	nServo4Low=(signed char)(nReadServo4Val & 0xff);
	nServo4High=(signed char)((nReadServo4Val>>8) & 0xff);

	nCompensationResultLow=((signed short)nServo4Low -5)&0x00ff;
	nCompensationResultHigh=((signed short)(nServo4High -5)<<8)&0xff00;
	ncompensationResult=nCompensationResultLow|nCompensationResultHigh;
	wm8994_write(codec,WM8994_DC_SERVO_4, ncompensationResult);

	val = (WM8994_DCS_TRIG_DAC_WR_1 | WM8994_DCS_TRIG_DAC_WR_0 | WM8994_DCS_ENA_CHAN_1 | WM8994_DCS_ENA_CHAN_0);
	wm8994_write(codec,WM8994_DC_SERVO_1, val );

	msleep(20);

	//* Headphone Output
		// Intermediate HP settings
	val = wm8994_read(codec, WM8994_ANALOGUE_HP_1); 	
	val &= ~(WM8994_HPOUT1R_DLY_MASK |WM8994_HPOUT1R_OUTP_MASK |WM8994_HPOUT1R_RMV_SHORT_MASK |
		WM8994_HPOUT1L_DLY_MASK |WM8994_HPOUT1L_OUTP_MASK | WM8994_HPOUT1L_RMV_SHORT_MASK);
	val |= (WM8994_HPOUT1L_RMV_SHORT | WM8994_HPOUT1L_OUTP|WM8994_HPOUT1L_DLY |WM8994_HPOUT1R_RMV_SHORT | 
		WM8994_HPOUT1R_OUTP | WM8994_HPOUT1R_DLY);
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	//DAC1, DAC2 Volume Setting
	//Unmute DAC1 left
	val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );	//610H : 1C0
	val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL); 
	wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);

	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME);	//611 : 1C0
	val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume	
	wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);

	//DAC1 Unmute
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, 0x0000);

	val = wm8994_read(codec, WM8994_AIF2_DAC_FILTERS_1);	//520 : 0
	val &= ~(WM8994_AIF2DAC_MUTE_MASK);
	wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, val);

}
	
void wm8994_set_fmradio_speaker(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;

	u16 val;

	DEBUG_LOG("Routing spk path : FM Radio -> SPK Out");

	wm8994_disable_fmradio_path(codec, FMR_HP);

	wm8994->fmradio_path = FMR_SPK;

	//Disable end point for preventing pop up noise.
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_SPKOUTL_ENA_MASK);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);	//03 : 0100
	val &= ~(WM8994_SPKRVOL_ENA_MASK | WM8994_SPKLVOL_ENA_MASK);
	val |= (WM8994_SPKLVOL_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{	
		// Unmute the SPKMIXVOLUME
		val = wm8994_read(codec, WM8994_SPKMIXL_ATTENUATION);
		val &= ~(WM8994_SPKMIXL_VOL_MASK);
		val |= TUNING_SPKMIXL_ATTEN;	
		wm8994_write(codec, WM8994_SPKMIXL_ATTENUATION, val);
			
		val = wm8994_read(codec, WM8994_SPKMIXR_ATTENUATION);
		val &= ~(WM8994_SPKMIXR_VOL_MASK);
		wm8994_write(codec, WM8994_SPKMIXR_ATTENUATION, val);
	
		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
		val &= ~(WM8994_SPKOUTL_MUTE_N_MASK | WM8994_SPKOUTL_VOL_MASK);
		val |= (WM8994_SPKOUTL_MUTE_N | TUNING_FMRADIO_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);
	
		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_RIGHT);
		val &= ~(WM8994_SPKOUTR_MUTE_N_MASK | WM8994_SPKOUTR_VOL_MASK);
		val |= (WM8994_SPKOUT_VU);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);
	
		val = wm8994_read(codec, WM8994_CLASSD);
		val &= ~(WM8994_SPKOUTL_BOOST_MASK);
		val |= TUNING_FMRADIO_CLASSD_VOL << WM8994_SPKOUTL_BOOST_SHIFT;
		wm8994_write(codec, WM8994_CLASSD, val);
		
		val = wm8994_read(codec, WM8994_AIF1_DAC1_LEFT_VOLUME);
		val &= ~(WM8994_AIF1DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_LEFT_VOLUME, val);

		val = wm8994_read(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME);
		val &= ~(WM8994_AIF1DAC1R_VOL_MASK);
		val |= (WM8994_AIF1DAC1_VU |TUNING_DAC1R_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME, val);		
	}

	wm8994_set_fmradio_common(codec, 1);

	wm8994_write(codec, WM8994_GPIO_3, 0x0100);	//702
	wm8994_write(codec, WM8994_GPIO_4, 0x0100);	//703
	wm8994_write(codec, WM8994_GPIO_5, 0x8100);	//704
	wm8994_write(codec, WM8994_GPIO_6, 0xA101);	//705
	wm8994_write(codec, WM8994_GPIO_7, 0x0100);	//706

	/*Output MIxer-Output PGA*/
	val = wm8994_read(codec,WM8994_SPKOUT_MIXERS );
	val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | 
		WM8994_SPKMIXR_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
	val |= (WM8994_SPKMIXL_TO_SPKOUTL);
	wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);

	// Output mixer setting
	val = wm8994_read(codec, WM8994_SPEAKER_MIXER);
	val &= ~(WM8994_MIXINL_TO_SPKMIXL_MASK | WM8994_MIXINR_TO_SPKMIXR_MASK | WM8994_DAC1L_TO_SPKMIXL_MASK | WM8994_DAC1R_TO_SPKMIXR_MASK);
	val |= (WM8994_DAC1L_TO_SPKMIXL);
	wm8994_write(codec, WM8994_SPEAKER_MIXER, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);	//05 : 3303
	val &= ~(WM8994_AIF2DACL_ENA_MASK | WM8994_AIF2DACR_ENA_MASK | WM8994_AIF1DAC1L_ENA_MASK | WM8994_AIF1DAC1R_ENA_MASK
		 | WM8994_DAC1L_ENA_MASK | WM8994_DAC1R_ENA_MASK);
	val |= (WM8994_AIF2DACL_ENA | WM8994_AIF2DACR_ENA | WM8994_AIF1DAC1L_ENA | WM8994_AIF1DAC1R_ENA
		 | WM8994_DAC1L_ENA | WM8994_DAC1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	// Enable IN2 and MIXIN - Temporary inserted for blocking MIC and FM radio mixing - DW Shim 2010.03.04
//	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
//	val &= ~(WM8994_TSHUT_ENA_MASK | WM8994_TSHUT_OPDIS_MASK | WM8994_OPCLK_ENA_MASK | 
//			WM8994_MIXINL_ENA_MASK | WM8994_MIXINR_ENA_MASK | WM8994_IN2L_ENA_MASK | WM8994_IN2R_ENA_MASK);
//	val |= (WM8994_TSHUT_ENA | WM8994_TSHUT_OPDIS | WM8994_OPCLK_ENA | WM8994_MIXINL_ENA | 
//			WM8994_MIXINR_ENA | WM8994_IN2L_ENA | WM8994_IN2R_ENA);
	val = (WM8994_TSHUT_ENA | WM8994_TSHUT_OPDIS | WM8994_OPCLK_ENA | WM8994_MIXINL_ENA | 
			WM8994_MIXINR_ENA | WM8994_IN2L_ENA | WM8994_IN2R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

	//AIF2 clock source <- FLL1
	val = wm8994_read(codec, WM8994_AIF2_CLOCKING_1);	//204H : 0x0011
	val &= ~(WM8994_AIF2CLK_ENA_MASK | WM8994_AIF2CLK_SRC_MASK);
	val |= (WM8994_AIF2CLK_ENA | 0x2 << WM8994_AIF2CLK_SRC_SHIFT);
	wm8994_write(codec, WM8994_AIF2_CLOCKING_1, val);

	val = wm8994_read(codec, WM8994_CLOCKING_1);	//208H : 0xF -> 0xE
	val &= ~(WM8994_SYSCLK_SRC_MASK | WM8994_DSP_FSINTCLK_ENA_MASK | WM8994_DSP_FS2CLK_ENA_MASK | WM8994_DSP_FS1CLK_ENA_MASK);
	val |= (WM8994_DSP_FS1CLK_ENA | WM8994_DSP_FS2CLK_ENA | WM8994_DSP_FSINTCLK_ENA);
	wm8994_write(codec, WM8994_CLOCKING_1, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);	//04H
	val &= ~(WM8994_AIF2ADCL_ENA_MASK | WM8994_AIF2ADCR_ENA_MASK | WM8994_ADCL_ENA_MASK | WM8994_ADCR_ENA_MASK);
	val |= (WM8994_AIF2ADCL_ENA | WM8994_AIF2ADCR_ENA | WM8994_ADCL_ENA | WM8994_ADCR_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

	//DAC2 Setting
	//ADCR_TO_DAC2 vol, ADCL_TO_DAC2 vol
	wm8994_write(codec, WM8994_DAC2_MIXER_VOLUMES, 0x018C); //603H : 0x018C
	
	//ADCL_TO_DAC2L
	val = wm8994_read(codec, WM8994_DAC2_LEFT_MIXER_ROUTING);	//604H : 0x0010
	val &= ~(WM8994_ADC1_TO_DAC2L_MASK);
	val |= (WM8994_ADC1_TO_DAC2L);
	wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, val);

	//ADCR_TO_DAC2R
	val = wm8994_read(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING);	//605H : 0x0010
	val &= ~(WM8994_ADC2_TO_DAC2R_MASK);
	val |= (WM8994_ADC2_TO_DAC2R);					// Changed value to support stereo
	wm8994_write(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING, val);

	//DAC block volume
	//DAC1, DAC2 Volume Setting
	//Unmute DAC1 left
	val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );	//610H : 1C0
	val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL); 
	wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);

	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME);	//611 : 1C0
	val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume	
	wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);

	val = wm8994_read(codec, WM8994_DAC2_LEFT_VOLUME);	//612 : 1C0
	val &= ~(WM8994_DAC2L_MUTE_MASK | WM8994_DAC2L_VOL_MASK);
	val |= (TUNING_DAC2L_VOL); //0 db volume	
	wm8994_write(codec,WM8994_DAC2_LEFT_VOLUME,val);
	
	val = wm8994_read(codec, WM8994_DAC2_RIGHT_VOLUME);	//613 : 1C0
	val &= ~(WM8994_DAC2R_MUTE_MASK | WM8994_DAC2R_VOL_MASK);
	val |= (WM8994_DAC2_VU | TUNING_DAC2R_VOL); //0 db volume	
	wm8994_write(codec, WM8994_DAC2_RIGHT_VOLUME, val);

	wm8994_write(codec, WM8994_AIF1_DAC1_EQ_GAINS_1, 0x0000);	//480 : 0

//	wm8994_write(codec, 0x01, 0x1003);
	
	wm8994_write(codec, WM8994_OVERSAMPLING, 0x0000);

	//AIF2 Master/Slave , LOOPBACK, AIF2DAC Unmute
	val = wm8994_read(codec, WM8994_AIF2_MASTER_SLAVE); //312 : 7000
	val &= ~(WM8994_AIF2_LRCLK_FRC_MASK | WM8994_AIF2_CLK_FRC_MASK | WM8994_AIF2_MSTR_MASK);
	val |= (WM8994_AIF2_LRCLK_FRC | WM8994_AIF2_CLK_FRC | WM8994_AIF2_MSTR);
	wm8994_write(codec, WM8994_AIF2_MASTER_SLAVE, val);

	val = wm8994_read(codec, WM8994_AIF2_CONTROL_2);	//311 : 4001
	val &= ~(WM8994_AIF2_LOOPBACK_MASK);
	val |= (WM8994_AIF2_LOOPBACK);
	wm8994_write(codec, WM8994_AIF2_CONTROL_2, val);

	val = wm8994_read(codec, WM8994_AIF2_DAC_FILTERS_1);	//520 : 0
	val &= ~(WM8994_AIF2DAC_MUTE_MASK);
	wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, val);
	
	//Enbale bias,vmid and Left speaker
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK |WM8994_SPKOUTL_ENA_MASK);
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL | WM8994_SPKOUTL_ENA);  
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	//DAC Routing
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);	//601H : 0x05
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK | WM8994_AIF1DAC2L_TO_DAC1L_MASK | WM8994_AIF2DACL_TO_DAC1L_MASK);
	val |= (WM8994_AIF2DACL_TO_DAC1L | WM8994_AIF1DAC1L_TO_DAC1L);
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);

	val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING);	//602H : 0x05
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK | WM8994_AIF1DAC2R_TO_DAC1R_MASK | WM8994_AIF2DACR_TO_DAC1R_MASK);
	val |= (WM8994_AIF1DAC1R_TO_DAC1R | WM8994_AIF2DACR_TO_DAC1R);
	wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, val);	

	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, 0x0000);

}

void wm8994_set_fmradio_headset_mix(struct snd_soc_codec *codec)
{	
	struct wm8994_priv *wm8994 = codec->drvdata;

	int val;

	DEBUG_LOG("");
	
	if(wm8994->fmradio_path == FMR_SPK)
		wm8994_set_playback_headset(codec);
	else
	{
		// Unmute the AF1DAC1	
		val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1 );	
		val &= ~(WM8994_AIF1DAC1_MUTE_MASK | WM8994_AIF1DAC1_MONO_MASK);
		val |= WM8994_AIF1DAC1_UNMUTE;
		wm8994_write(codec,WM8994_AIF1_DAC1_FILTERS_1 ,val);
		
		// Enable the Timeslot0 to DAC1L
		val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING  ); 	
		val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
		val |= WM8994_AIF1DAC1L_TO_DAC1L;
		wm8994_write(codec,WM8994_DAC1_LEFT_MIXER_ROUTING ,val);
				
		//Enable the Timeslot0 to DAC1R
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING  );	
		val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
		val |= WM8994_AIF1DAC1R_TO_DAC1R;
		wm8994_write(codec,WM8994_DAC1_RIGHT_MIXER_ROUTING ,val);
		
		// Enable DAC1L to HPOUT1L path
		val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
		val &=	~(WM8994_DAC1L_TO_HPOUT1L_MASK | WM8994_DAC1L_TO_MIXOUTL_MASK);
		val |= WM8994_DAC1L_TO_MIXOUTL ;	
		wm8994_write(codec,WM8994_OUTPUT_MIXER_1, val);
		
		// Enable DAC1R to HPOUT1R path
		val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
		val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK | WM8994_DAC1R_TO_MIXOUTR_MASK);
		val |= WM8994_DAC1R_TO_MIXOUTR;
		wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);
				
		//Enable Dac1 and DAC2 and the Timeslot0 for AIF1	
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5 );	
		val &= ~(WM8994_DAC1R_ENA_MASK | WM8994_DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK |	WM8994_AIF1DAC1L_ENA_MASK );
		val |= (WM8994_AIF1DAC1L_ENA | WM8994_AIF1DAC1R_ENA  | WM8994_DAC1L_ENA |WM8994_DAC1R_ENA );
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_5 ,val);
				
		//Unmute DAC1 left
		val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );
		val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL); 
		wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);
		
		//Unmute and volume ctrl RightDAC
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
		val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
		val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume	
		wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);
		
		val = wm8994_read(codec, WM8994_AIF1_DAC1_LEFT_VOLUME);
		val &= ~(WM8994_AIF1DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_RADIO_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_LEFT_VOLUME, val);

		val = wm8994_read(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME);
		val &= ~(WM8994_AIF1DAC1R_VOL_MASK);
		val |= (WM8994_AIF1DAC1_VU |TUNING_DAC1R_RADIO_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME, val);
	}
}

void wm8994_set_fmradio_speaker_mix(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;

	int val;
	
	DEBUG_LOG("");

	if(wm8994->fmradio_path == FMR_HP)
		wm8994_set_playback_speaker(codec);
	else
	{
		//Unmute DAC1 left
		val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );
		val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL);
		wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);
	
		//Unmute and volume ctrl RightDAC
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
		val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
		val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume	
		wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);

		//Unmute the DAC path
		val = wm8994_read(codec,WM8994_SPEAKER_MIXER);
		val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK);
		val |= WM8994_DAC1L_TO_SPKMIXL;
		wm8994_write(codec, WM8994_SPEAKER_MIXER, val);

		// Eable DAC1 Left and timeslot left
		val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_5);	
		val &= ~( WM8994_DAC1L_ENA_MASK | WM8994_AIF1DAC1R_ENA_MASK | WM8994_AIF1DAC1L_ENA_MASK);
		val |= (WM8994_AIF1DAC1L_ENA | WM8994_AIF1DAC1R_ENA | WM8994_DAC1L_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);   

		//Unmute
		val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
		val &= ~(WM8994_AIF1DAC1_MUTE_MASK | WM8994_AIF1DAC1_MONO_MASK);
		val |= (WM8994_AIF1DAC1_UNMUTE | WM8994_AIF1DAC1_MONO);
		wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);

		//enable timeslot0 to left dac
		val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);
		val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
		val |= WM8994_AIF1DAC1L_TO_DAC1L;
		wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);
	}
	wm8994->fmradio_path = FMR_SPK_MIX;
}

void wm8994_set_fmradio_speaker_headset_mix(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;

	u16 val;
	
	u16 nReadServo4Val = 0;
	u16 ncompensationResult = 0;
	u16 nServo4Low = 0;
	u16 nServo4High = 0;
	u8  nCompensationResultLow=0;
	u8 nCompensationResultHigh=0;

	DEBUG_LOG("");

	if(wm8994->fmradio_path == FMR_HP)
		wm8994_disable_fmradio_path(codec, FMR_HP);
	else
		wm8994_disable_fmradio_path(codec, FMR_SPK);
	
	wm8994->fmradio_path = FMR_DUAL_MIX;

	wm8994_write(codec, WM8994_GPIO_3, 0x0100);
	wm8994_write(codec, WM8994_GPIO_4, 0x0100);
	wm8994_write(codec, WM8994_GPIO_5, 0x8100);
	wm8994_write(codec, WM8994_GPIO_6, 0xA101);
	wm8994_write(codec, WM8994_GPIO_7, 0x0100);

	// Disable reg sync to MCLK
	val = wm8994_read(codec, WM8994_AIF1_CLOCKING_1); 	
	val &= ~(WM8994_AIF1CLK_ENA_MASK);
	val |= WM8994_AIF1CLK_ENA;
	wm8994_write(codec, WM8994_AIF1_CLOCKING_1, val);


	// Analogue Path Config
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2  ); 	
	val &= ~(WM8994_MIXINL_ENA_MASK | WM8994_MIXINR_ENA_MASK | WM8994_IN2L_ENA_MASK | WM8994_IN2R_ENA_MASK);
	val |= (WM8994_MIXINL_ENA | WM8994_MIXINR_ENA| WM8994_IN2L_ENA| WM8994_IN2R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_2 , val );

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1); 	
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_NORMAL);
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1 , 0x0003);
		
	val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME); 	
	val = 0x0000;
	wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

	val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME); 	
	val = 0x0100;
	wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);

//	wm8994_set_fmradio_common(codec, 1);

	//FLL2 Setting
	val = wm8994_read(codec, WM8994_AIF2_CLOCKING_1);	//204H : 0x0011
	val &= ~(WM8994_AIF2CLK_ENA_MASK | WM8994_AIF2CLK_SRC_MASK);
	val |= (WM8994_AIF2CLK_ENA | 0x2 << WM8994_AIF2CLK_SRC_SHIFT);
	wm8994_write(codec, WM8994_AIF2_CLOCKING_1, val);

	val = wm8994_read(codec, WM8994_CLOCKING_1);	//208H : 0xF -> 0xE
	val &= ~(WM8994_SYSCLK_SRC_MASK | WM8994_DSP_FSINTCLK_ENA_MASK | WM8994_DSP_FS2CLK_ENA_MASK | WM8994_DSP_FS1CLK_ENA_MASK);
	val |= (WM8994_DSP_FS1CLK_ENA | WM8994_DSP_FS2CLK_ENA | WM8994_DSP_FSINTCLK_ENA);
	wm8994_write(codec, WM8994_CLOCKING_1, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);	//04H
	val &= ~(WM8994_AIF2ADCL_ENA_MASK | WM8994_AIF2ADCR_ENA_MASK | WM8994_ADCL_ENA_MASK | WM8994_ADCR_ENA_MASK);
	val |= (WM8994_AIF2ADCL_ENA | WM8994_AIF2ADCR_ENA | WM8994_ADCL_ENA | WM8994_ADCR_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

	//DAC2 Setting
	wm8994_write(codec, WM8994_DAC2_MIXER_VOLUMES, 0x018C);	//603H : 0x018C

	val = wm8994_read(codec, WM8994_DAC2_LEFT_MIXER_ROUTING);	//604H : 0x0010
	val &= ~(WM8994_ADC1_TO_DAC2L_MASK);
	val |= (WM8994_ADC1_TO_DAC2L);
	wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, val);

	val = wm8994_read(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING);	//605H : 0x0010
	val &= ~(WM8994_ADC2_TO_DAC2R_MASK);
	val |= (WM8994_ADC2_TO_DAC2R);					// Changed value to support stereo
	wm8994_write(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING, val);
	
	//DRC for Noise-gate (AIF2)
	wm8994_write(codec, 0x541, 0x0850);
	wm8994_write(codec, 0x542, 0x0800);
	wm8994_write(codec, 0x543, 0x0001);
	wm8994_write(codec, 0x544, 0x0008);
	wm8994_write(codec, 0x540, 0x01BC);

	//DAC1 Setting
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);	//601H : 0x05
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK | WM8994_AIF1DAC2L_TO_DAC1L_MASK | WM8994_AIF2DACL_TO_DAC1L_MASK);
	val |= (WM8994_AIF2DACL_TO_DAC1L | WM8994_AIF1DAC1L_TO_DAC1L);
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);


	val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING);	//602H : 0x05
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK | WM8994_AIF1DAC2R_TO_DAC1R_MASK | WM8994_AIF2DACR_TO_DAC1R_MASK);
	val |= (WM8994_AIF1DAC1R_TO_DAC1R | WM8994_AIF2DACR_TO_DAC1R);
	wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, val);

	//DAC1, DAC2 Volume Setting
	//Unmute DAC1 left
	val = wm8994_read(codec,WM8994_DAC1_LEFT_VOLUME );	//610H : 1C0
	val &= ~(WM8994_DAC1L_MUTE_MASK | WM8994_DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL); 
	wm8994_write(codec,WM8994_DAC1_LEFT_VOLUME ,val);

	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME);	//611 : 1C0
	val &= ~(WM8994_DAC1R_MUTE_MASK | WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU | TUNING_DAC1R_VOL); //0 db volume	
	wm8994_write(codec,WM8994_DAC1_RIGHT_VOLUME,val);


	val = wm8994_read(codec, WM8994_DAC2_LEFT_VOLUME);	//612 : 1C0
	val &= ~(WM8994_DAC2L_MUTE_MASK | WM8994_DAC2L_VOL_MASK);
	val |= (TUNING_DAC2L_VOL); //0 db volume	
	wm8994_write(codec,WM8994_DAC2_LEFT_VOLUME,val);
	
	val = wm8994_read(codec, WM8994_DAC2_RIGHT_VOLUME);	//613 : 1C0
	val &= ~(WM8994_DAC2R_MUTE_MASK | WM8994_DAC2R_VOL_MASK);
	val |= (WM8994_DAC2_VU | TUNING_DAC2R_VOL); //0 db volume	
	wm8994_write(codec, WM8994_DAC2_RIGHT_VOLUME, val);

	wm8994_write(codec, WM8994_AIF1_DAC1_EQ_GAINS_1, 0x0000);	//480 : 0
	 
	//Digital  Mixer setting
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);	//05 : 3303
	val &= ~(WM8994_AIF2DACL_ENA_MASK | WM8994_AIF2DACR_ENA_MASK | WM8994_AIF1DAC1L_ENA_MASK | WM8994_AIF1DAC1R_ENA_MASK
		 | WM8994_DAC1L_ENA_MASK | WM8994_DAC1R_ENA_MASK);
	val |= (WM8994_AIF2DACL_ENA | WM8994_AIF2DACR_ENA | WM8994_AIF1DAC1L_ENA | WM8994_AIF1DAC1R_ENA
		 | WM8994_DAC1L_ENA | WM8994_DAC1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);	//03 : F
	val &= ~(WM8994_MIXOUTLVOL_ENA_MASK | WM8994_MIXOUTRVOL_ENA_MASK | WM8994_MIXOUTL_ENA_MASK | WM8994_MIXOUTR_ENA_MASK
		|WM8994_SPKLVOL_ENA_MASK | WM8994_SPKRVOL_ENA_MASK);
	val |= (WM8994_MIXOUTLVOL_ENA | WM8994_MIXOUTRVOL_ENA | WM8994_MIXOUTL_ENA | WM8994_MIXOUTR_ENA | WM8994_SPKLVOL_ENA);	
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);	//2D : 1
	val &= ~(WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |= (WM8994_DAC1L_TO_MIXOUTL);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val);

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);	//2E : 1
	val &= ~(WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= (WM8994_DAC1R_TO_MIXOUTR);	
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

	val = wm8994_read(codec, WM8994_SPKOUT_MIXERS );
	val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK | 
		WM8994_SPKMIXR_TO_SPKOUTL_MASK | WM8994_SPKMIXR_TO_SPKOUTR_MASK);
	val |= (WM8994_SPKMIXL_TO_SPKOUTL);
	wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);

	//Unmute the DAC path
	val = wm8994_read(codec,WM8994_SPEAKER_MIXER);
	val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK);
	val |= WM8994_DAC1L_TO_SPKMIXL;
	wm8994_write(codec, WM8994_SPEAKER_MIXER, val);	

	//AIF1 FLL Setting	
	wm8994_write(codec, WM8994_OVERSAMPLING, 0x0000);
	
	//AIF2 Master/Slave , LOOPBACK, AIF2DAC Unmute
	val = wm8994_read(codec, WM8994_AIF2_MASTER_SLAVE);	//312 : 7000
	val &= ~(WM8994_AIF2_LRCLK_FRC_MASK | WM8994_AIF2_CLK_FRC_MASK | WM8994_AIF2_MSTR_MASK);
	val |= (WM8994_AIF2_LRCLK_FRC | WM8994_AIF2_CLK_FRC | WM8994_AIF2_MSTR);
	wm8994_write(codec, WM8994_AIF2_MASTER_SLAVE, val);

	val = wm8994_read(codec, WM8994_AIF2_CONTROL_2);	//311 : 4001
	val &= ~(WM8994_AIF2_LOOPBACK_MASK);
	val |= (WM8994_AIF2_LOOPBACK);
	wm8994_write(codec, WM8994_AIF2_CONTROL_2, val);

	//* Unmutes
	// Output setting
	if(wm8994->testmode_config_flag == SEC_NORMAL)
	{
		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK | WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUTL_MUTE_N | TUNING_FMRADIO_OPGAL_VOL);
		wm8994_write(codec,WM8994_LEFT_OPGA_VOLUME, val );
	
		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK | WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU | WM8994_MIXOUTR_MUTE_N | TUNING_FMRADIO_OPGAR_VOL);
		wm8994_write(codec,WM8994_RIGHT_OPGA_VOLUME, val );
		
		val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK | WM8994_HPOUT1L_VOL_MASK);
		val |= (WM8994_HPOUT1L_MUTE_N | TUNING_FMRADIO_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK | WM8994_HPOUT1R_VOL_MASK);
		val |= (WM8994_HPOUT1_VU | WM8994_HPOUT1R_MUTE_N | TUNING_FMRADIO_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_AIF1_DAC1_LEFT_VOLUME);
		val &= ~(WM8994_AIF1DAC1L_VOL_MASK);
		val |= (TUNING_DUAL_DAC1L_RADIO_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_LEFT_VOLUME, val);

		val = wm8994_read(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME);
		val &= ~(WM8994_AIF1DAC1R_VOL_MASK);
		val |= (WM8994_AIF1DAC1_VU |TUNING_DUAL_DAC1R_RADIO_VOL);
		wm8994_write(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME, val);

		val = wm8994_read(codec, WM8994_CLASSD);
		val &= ~(WM8994_SPKOUTL_BOOST_MASK);
		val |= TUNING_FMRADIO_CLASSD_VOL << WM8994_SPKOUTL_BOOST_SHIFT;
		wm8994_write(codec, WM8994_CLASSD, val);		

		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
		val &= ~(WM8994_SPKOUTL_MUTE_N_MASK | WM8994_SPKOUTL_VOL_MASK);
		val |= (WM8994_SPKOUT_VU | WM8994_SPKOUTL_MUTE_N | TUNING_FMRADIO_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);		
	}	
	//* Headset
	wm8994_write(codec, 0x102, 0x0003);
	wm8994_write(codec, 0x56, 0x0003);
	wm8994_write(codec, 0x102, 0x0000);
	wm8994_write(codec, 0x5D, 0x0002);


	//* DC Servo Series Count
	val = 0x03E0;
	wm8994_write(codec, WM8994_DC_SERVO_2, val);

	//* HP first and second stage
	//Enable vmid,bias, hp left and right
	val = wm8994_read(codec,WM8994_POWER_MANAGEMENT_1 );
	val &= ~(WM8994_BIAS_ENA_MASK | WM8994_VMID_SEL_MASK |WM8994_HPOUT1L_ENA_MASK | WM8994_HPOUT1R_ENA_MASK | WM8994_SPKOUTR_ENA_MASK | WM8994_SPKOUTL_ENA_MASK);
	val |= (WM8994_BIAS_ENA | WM8994_VMID_SEL_NORMAL | WM8994_HPOUT1R_ENA | WM8994_HPOUT1L_ENA | WM8994_SPKOUTL_ENA);  
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	val = (WM8994_HPOUT1L_DLY | WM8994_HPOUT1R_DLY);
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);


	//Enable Charge Pump	
	val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
	val &= ~WM8994_CP_ENA_MASK ;
	val |= WM8994_CP_ENA | WM8994_CP_ENA_DEFAULT ; // this is from wolfson  	
	wm8994_write(codec, WM8994_CHARGE_PUMP_1, 0x9F25);

	msleep(5);

	//* DC Servo 
	val = (WM8994_DCS_TRIG_SERIES_1 | WM8994_DCS_TRIG_SERIES_0 | WM8994_DCS_ENA_CHAN_1 | WM8994_DCS_ENA_CHAN_0);
	wm8994_write(codec,WM8994_DC_SERVO_1, 0x0303 );

	msleep(160);

	nReadServo4Val=wm8994_read(codec,WM8994_DC_SERVO_4);
	nServo4Low=(u8)(nReadServo4Val & 0xff);
	nServo4High=(u8)((nReadServo4Val>>8) & 0xff);

	nCompensationResultLow=((u16)nServo4Low-5)&0x00ff;
	nCompensationResultHigh=(((u16)nServo4High-5)<<8)&0xff00;
	ncompensationResult=nCompensationResultLow|nCompensationResultHigh;
	wm8994_write(codec,WM8994_DC_SERVO_4, ncompensationResult);

	val = (WM8994_DCS_TRIG_DAC_WR_1 | WM8994_DCS_TRIG_DAC_WR_0 | WM8994_DCS_ENA_CHAN_1 | WM8994_DCS_ENA_CHAN_0);
	wm8994_write(codec,WM8994_DC_SERVO_1, val );

	msleep(20);

	//* Headphone Output
		// Intermediate HP settings
	val = wm8994_read(codec, WM8994_ANALOGUE_HP_1); 	
	val &= ~(WM8994_HPOUT1R_DLY_MASK |WM8994_HPOUT1R_OUTP_MASK |WM8994_HPOUT1R_RMV_SHORT_MASK |
		WM8994_HPOUT1L_DLY_MASK |WM8994_HPOUT1L_OUTP_MASK | WM8994_HPOUT1L_RMV_SHORT_MASK);
	val |= (WM8994_HPOUT1L_RMV_SHORT | WM8994_HPOUT1L_OUTP|WM8994_HPOUT1L_DLY |WM8994_HPOUT1R_RMV_SHORT | 
		WM8994_HPOUT1R_OUTP | WM8994_HPOUT1R_DLY);
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	//DAC1 Unmute
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK | WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE | WM8994_AIF1DAC1_MONO);
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);	
	
	val = wm8994_read(codec, WM8994_AIF2_DAC_FILTERS_1);	//520 : 0
	val &= ~(WM8994_AIF2DAC_MUTE_MASK);
	wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, val);

}

#ifdef CONFIG_TARGET_LOCALE_KOR
void wm8994_set_voipcall_receiver(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;


	DEBUG_LOG("");
/*
    // 1st Step : VMID enable
 	wm8994_write(codec, 0x39, 0x006C );      // VMID_RAMP=11, VMID_BUF_ENA=1, STARTUP_BIAS_ENA=1
	wm8994_write(codec, 0x1, 0x0007 );

	msleep(50);

	wm8994_write(codec, 0x1, 0x0003 );
	wm8994_write(codec, 0x102, 0x0003 ); 
	wm8994_write(codec, 0x817, 0x0000 ); 
	wm8994_write(codec, 0x102, 0x0000 ); 


    // 2nd Step : GPIO enable
	wm8994_write(codec, 0x700, 0xa101 );
 

    // 3rd Step : AIFn, FLLn Enable         // 24Mhz --> 11.2896Mhz 
	wm8994_write(codec, 0x220, 0x0005 );   // FLL1 Cntr1, FLL1 Enable
	wm8994_write(codec, 0x221, 0x0700 );   // FLL1 Cntr2, FLL1 Setting
	wm8994_write(codec, 0x222, 0x871C );   // FLL1 Cntr3, K Value
	wm8994_write(codec, 0x224, 0x0C88 );   // FLL1 Cntr4, N Value
	wm8994_write(codec, 0x223, 0x00E0 );   // FLL1 Cntr4, N Value
	wm8994_write(codec, 0x200, 0x0010 );   // Enable AIF1 Clock, AIF1 Clock
	wm8994_write(codec, 0x208, 0x000A ); 
	wm8994_write(codec, 0x210, 0x0073 ); 
	wm8994_write(codec, 0x620, 0x0000 ); 
	wm8994_write(codec, 0x302, 0x7000 ); 
	wm8994_write(codec, 0x300, 0x4010 ); 
	wm8994_write(codec, 0x301, 0x0000 ); 
	wm8994_write(codec, 0x200, 0x0011 );   // Enable AIF1 Clock, AIF1 Clock
 
    // 4th Step : Routing
    // Digital Routing
	wm8994_write(codec, 0x601, 0x0001 );
	wm8994_write(codec, 0x602, 0x0001 );
	wm8994_write(codec, 0x606, 0x0002 );
	wm8994_write(codec, 0x607, 0x0002 );

    // Analogue Input
	wm8994_write(codec, 0x2, 0x6240 ); 
	wm8994_write(codec, 0x3, 0x0300 ); 
	wm8994_write(codec, 0x4, 0x0003 ); 
	wm8994_write(codec, 0x5, 0x0303 ); 
	wm8994_write(codec, 0x28, 0x0030 ); 
	wm8994_write(codec, 0x29, 0x0030 ); 
	wm8994_write(codec, 0x18, 0x010d ); 

    // Analogue Output
	wm8994_write(codec, 0x22, 0x0000 ); 
	wm8994_write(codec, 0x23, 0x0000 ); 
	wm8994_write(codec, 0x26, 0x0179 ); 
	wm8994_write(codec, 0x27, 0x0179 ); 
	wm8994_write(codec, 0x25, 0x017F ); 
	wm8994_write(codec, 0x24, 0x0011 ); 
	wm8994_write(codec, 0x36, 0x0003 ); 
	wm8994_write(codec, 0x621, 0x01c0 ); 
	wm8994_write(codec, 0x1, 0x3003 ); 

    // Unmute
	wm8994_write(codec, 0x4, 0x0303 );
	wm8994_write(codec, 0x610, 0x00c0 ); 
	wm8994_write(codec, 0x611, 0x00c0 ); 
	wm8994_write(codec, 0x420, 0x0000 );

*/
}

void wm8994_set_voipcall_speaker(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;


	DEBUG_LOG("");

    // 1st Step : VMID enable
 	wm8994_write(codec, 0x39, 0x006C );      // VMID_RAMP=11, VMID_BUF_ENA=1, STARTUP_BIAS_ENA=1
	wm8994_write(codec, 0x1, 0x0007 );

	msleep(50);

	wm8994_write(codec, 0x1, 0x0003 );
	wm8994_write(codec, 0x102, 0x0003 ); 
	wm8994_write(codec, 0x817, 0x0000 ); 
	wm8994_write(codec, 0x102, 0x0000 ); 


    // 2nd Step : GPIO enable
	wm8994_write(codec, 0x700, 0xa101 );
 

    // 3rd Step : AIFn, FLLn Enable         // 24Mhz --> 11.2896Mhz 
	wm8994_write(codec, 0x220, 0x0005 );   // FLL1 Cntr1, FLL1 Enable
	wm8994_write(codec, 0x221, 0x0700 );   // FLL1 Cntr2, FLL1 Setting
	wm8994_write(codec, 0x222, 0x871C );   // FLL1 Cntr3, K Value
	wm8994_write(codec, 0x224, 0x0C88 );   // FLL1 Cntr4, N Value
	wm8994_write(codec, 0x223, 0x00E0 );   // FLL1 Cntr4, N Value
	wm8994_write(codec, 0x200, 0x0010 );   // Enable AIF1 Clock, AIF1 Clock
	wm8994_write(codec, 0x208, 0x000A ); 
	wm8994_write(codec, 0x210, 0x0073 ); 
	wm8994_write(codec, 0x620, 0x0000 ); 
	wm8994_write(codec, 0x302, 0x7000 ); 
	wm8994_write(codec, 0x300, 0x4010 ); 
	wm8994_write(codec, 0x301, 0x0000 ); 
	wm8994_write(codec, 0x200, 0x0011 );   // Enable AIF1 Clock, AIF1 Clock
 
    // 4th Step : Routing
    // Digital Routing
	wm8994_write(codec, 0x601, 0x0001 );
	wm8994_write(codec, 0x602, 0x0001 );
	wm8994_write(codec, 0x606, 0x0002 );
	wm8994_write(codec, 0x607, 0x0002 );

    // Analogue Input
	wm8994_write(codec, 0x2, 0x6240 ); 
	wm8994_write(codec, 0x3, 0x0300 ); 
	wm8994_write(codec, 0x4, 0x0003 ); 
	wm8994_write(codec, 0x5, 0x0303 ); 
	wm8994_write(codec, 0x28, 0x0030 ); 
	wm8994_write(codec, 0x29, 0x0030 ); 
	wm8994_write(codec, 0x18, 0x010d ); 

    // Analogue Output
	wm8994_write(codec, 0x22, 0x0000 ); 
	wm8994_write(codec, 0x23, 0x0000 ); 
	wm8994_write(codec, 0x26, 0x0179 ); 
	wm8994_write(codec, 0x27, 0x0179 ); 
	wm8994_write(codec, 0x25, 0x017F ); 
	wm8994_write(codec, 0x24, 0x0011 ); 
	wm8994_write(codec, 0x36, 0x0003 ); 
	wm8994_write(codec, 0x621, 0x01c0 ); 
	wm8994_write(codec, 0x1, 0x3003 ); 

    // Unmute
	wm8994_write(codec, 0x4, 0x0303 );
	wm8994_write(codec, 0x610, 0x00c0 ); 
	wm8994_write(codec, 0x611, 0x00c0 ); 
	wm8994_write(codec, 0x420, 0x0000 );

}

void wm8994_set_voipcall_headphone(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;


	DEBUG_LOG("");
/*
    // 1st Step : VMID enable
 	wm8994_write(codec, 0x39, 0x006C );      // VMID_RAMP=11, VMID_BUF_ENA=1, STARTUP_BIAS_ENA=1
	wm8994_write(codec, 0x1, 0x0007 );

	msleep(50);

	wm8994_write(codec, 0x1, 0x0003 );
	wm8994_write(codec, 0x102, 0x0003 ); 
	wm8994_write(codec, 0x817, 0x0000 ); 
	wm8994_write(codec, 0x102, 0x0000 ); 


    // 2nd Step : GPIO enable
	wm8994_write(codec, 0x700, 0xa101 );
 

    // 3rd Step : AIFn, FLLn Enable         // 24Mhz --> 11.2896Mhz 
	wm8994_write(codec, 0x220, 0x0005 );   // FLL1 Cntr1, FLL1 Enable
	wm8994_write(codec, 0x221, 0x0700 );   // FLL1 Cntr2, FLL1 Setting
	wm8994_write(codec, 0x222, 0x871C );   // FLL1 Cntr3, K Value
	wm8994_write(codec, 0x224, 0x0C88 );   // FLL1 Cntr4, N Value
	wm8994_write(codec, 0x223, 0x00E0 );   // FLL1 Cntr4, N Value
	wm8994_write(codec, 0x200, 0x0010 );   // Enable AIF1 Clock, AIF1 Clock
	wm8994_write(codec, 0x208, 0x000A ); 
	wm8994_write(codec, 0x210, 0x0073 ); 
	wm8994_write(codec, 0x620, 0x0000 ); 
	wm8994_write(codec, 0x302, 0x7000 ); 
	wm8994_write(codec, 0x300, 0x4010 ); 
	wm8994_write(codec, 0x301, 0x0000 ); 
	wm8994_write(codec, 0x200, 0x0011 );   // Enable AIF1 Clock, AIF1 Clock
 
    // 4th Step : Routing
    // Digital Routing
	wm8994_write(codec, 0x601, 0x0001 );
	wm8994_write(codec, 0x602, 0x0001 );
	wm8994_write(codec, 0x606, 0x0002 );
	wm8994_write(codec, 0x607, 0x0002 );

    // Analogue Input
	wm8994_write(codec, 0x2, 0x6240 ); 
	wm8994_write(codec, 0x3, 0x0300 ); 
	wm8994_write(codec, 0x4, 0x0003 ); 
	wm8994_write(codec, 0x5, 0x0303 ); 
	wm8994_write(codec, 0x28, 0x0030 ); 
	wm8994_write(codec, 0x29, 0x0030 ); 
	wm8994_write(codec, 0x18, 0x010d ); 

    // Analogue Output
	wm8994_write(codec, 0x22, 0x0000 ); 
	wm8994_write(codec, 0x23, 0x0000 ); 
	wm8994_write(codec, 0x26, 0x0179 ); 
	wm8994_write(codec, 0x27, 0x0179 ); 
	wm8994_write(codec, 0x25, 0x017F ); 
	wm8994_write(codec, 0x24, 0x0011 ); 
	wm8994_write(codec, 0x36, 0x0003 ); 
	wm8994_write(codec, 0x621, 0x01c0 ); 
	wm8994_write(codec, 0x1, 0x3003 ); 

    // Unmute
	wm8994_write(codec, 0x4, 0x0303 );
	wm8994_write(codec, 0x610, 0x00c0 ); 
	wm8994_write(codec, 0x611, 0x00c0 ); 
	wm8994_write(codec, 0x420, 0x0000 );
*/
}

void wm8994_set_voipcall_headset(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;


	DEBUG_LOG("");
/*
    // 1st Step : VMID enable
 	wm8994_write(codec, 0x39, 0x006C );      // VMID_RAMP=11, VMID_BUF_ENA=1, STARTUP_BIAS_ENA=1
	wm8994_write(codec, 0x1, 0x0007 );

	msleep(50);

	wm8994_write(codec, 0x1, 0x0003 );
	wm8994_write(codec, 0x102, 0x0003 ); 
	wm8994_write(codec, 0x817, 0x0000 ); 
	wm8994_write(codec, 0x102, 0x0000 ); 


    // 2nd Step : GPIO enable
	wm8994_write(codec, 0x700, 0xa101 );
 

    // 3rd Step : AIFn, FLLn Enable         // 24Mhz --> 11.2896Mhz 
	wm8994_write(codec, 0x220, 0x0005 );   // FLL1 Cntr1, FLL1 Enable
	wm8994_write(codec, 0x221, 0x0700 );   // FLL1 Cntr2, FLL1 Setting
	wm8994_write(codec, 0x222, 0x871C );   // FLL1 Cntr3, K Value
	wm8994_write(codec, 0x224, 0x0C88 );   // FLL1 Cntr4, N Value
	wm8994_write(codec, 0x223, 0x00E0 );   // FLL1 Cntr4, N Value
	wm8994_write(codec, 0x200, 0x0010 );   // Enable AIF1 Clock, AIF1 Clock
	wm8994_write(codec, 0x208, 0x000A ); 
	wm8994_write(codec, 0x210, 0x0073 ); 
	wm8994_write(codec, 0x620, 0x0000 ); 
	wm8994_write(codec, 0x302, 0x7000 ); 
	wm8994_write(codec, 0x300, 0x4010 ); 
	wm8994_write(codec, 0x301, 0x0000 ); 
	wm8994_write(codec, 0x200, 0x0011 );   // Enable AIF1 Clock, AIF1 Clock
 
    // 4th Step : Routing
    // Digital Routing
	wm8994_write(codec, 0x601, 0x0001 );
	wm8994_write(codec, 0x602, 0x0001 );
	wm8994_write(codec, 0x606, 0x0002 );
	wm8994_write(codec, 0x607, 0x0002 );

    // Analogue Input
	wm8994_write(codec, 0x2, 0x6240 ); 
	wm8994_write(codec, 0x3, 0x0300 ); 
	wm8994_write(codec, 0x4, 0x0003 ); 
	wm8994_write(codec, 0x5, 0x0303 ); 
	wm8994_write(codec, 0x28, 0x0030 ); 
	wm8994_write(codec, 0x29, 0x0030 ); 
	wm8994_write(codec, 0x18, 0x010d ); 

    // Analogue Output
	wm8994_write(codec, 0x22, 0x0000 ); 
	wm8994_write(codec, 0x23, 0x0000 ); 
	wm8994_write(codec, 0x26, 0x0179 ); 
	wm8994_write(codec, 0x27, 0x0179 ); 
	wm8994_write(codec, 0x25, 0x017F ); 
	wm8994_write(codec, 0x24, 0x0011 ); 
	wm8994_write(codec, 0x36, 0x0003 ); 
	wm8994_write(codec, 0x621, 0x01c0 ); 
	wm8994_write(codec, 0x1, 0x3003 ); 

    // Unmute
	wm8994_write(codec, 0x4, 0x0303 );
	wm8994_write(codec, 0x610, 0x00c0 ); 
	wm8994_write(codec, 0x611, 0x00c0 ); 
	wm8994_write(codec, 0x420, 0x0000 );
*/
}


void wm8994_set_voipcall_bluetooth(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->drvdata;


	DEBUG_LOG("");
/*
    // 1st Step : VMID enable
 	wm8994_write(codec, 0x39, 0x006C );      // VMID_RAMP=11, VMID_BUF_ENA=1, STARTUP_BIAS_ENA=1
	wm8994_write(codec, 0x1, 0x0007 );

	msleep(50);

	wm8994_write(codec, 0x1, 0x0003 );
	wm8994_write(codec, 0x102, 0x0003 ); 
	wm8994_write(codec, 0x817, 0x0000 ); 
	wm8994_write(codec, 0x102, 0x0000 ); 


    // 2nd Step : GPIO enable
	wm8994_write(codec, 0x700, 0xa101 );
 

    // 3rd Step : AIFn, FLLn Enable         // 24Mhz --> 11.2896Mhz 
	wm8994_write(codec, 0x220, 0x0005 );   // FLL1 Cntr1, FLL1 Enable
	wm8994_write(codec, 0x221, 0x0700 );   // FLL1 Cntr2, FLL1 Setting
	wm8994_write(codec, 0x222, 0x871C );   // FLL1 Cntr3, K Value
	wm8994_write(codec, 0x224, 0x0C88 );   // FLL1 Cntr4, N Value
	wm8994_write(codec, 0x223, 0x00E0 );   // FLL1 Cntr4, N Value
	wm8994_write(codec, 0x200, 0x0010 );   // Enable AIF1 Clock, AIF1 Clock
	wm8994_write(codec, 0x208, 0x000A ); 
	wm8994_write(codec, 0x210, 0x0073 ); 
	wm8994_write(codec, 0x620, 0x0000 ); 
	wm8994_write(codec, 0x302, 0x7000 ); 
	wm8994_write(codec, 0x300, 0x4010 ); 
	wm8994_write(codec, 0x301, 0x0000 ); 
	wm8994_write(codec, 0x200, 0x0011 );   // Enable AIF1 Clock, AIF1 Clock
 
    // 4th Step : Routing
    // Digital Routing
	wm8994_write(codec, 0x601, 0x0001 );
	wm8994_write(codec, 0x602, 0x0001 );
	wm8994_write(codec, 0x606, 0x0002 );
	wm8994_write(codec, 0x607, 0x0002 );

    // Analogue Input
	wm8994_write(codec, 0x2, 0x6240 ); 
	wm8994_write(codec, 0x3, 0x0300 ); 
	wm8994_write(codec, 0x4, 0x0003 ); 
	wm8994_write(codec, 0x5, 0x0303 ); 
	wm8994_write(codec, 0x28, 0x0030 ); 
	wm8994_write(codec, 0x29, 0x0030 ); 
	wm8994_write(codec, 0x18, 0x010d ); 

    // Analogue Output
	wm8994_write(codec, 0x22, 0x0000 ); 
	wm8994_write(codec, 0x23, 0x0000 ); 
	wm8994_write(codec, 0x26, 0x0179 ); 
	wm8994_write(codec, 0x27, 0x0179 ); 
	wm8994_write(codec, 0x25, 0x017F ); 
	wm8994_write(codec, 0x24, 0x0011 ); 
	wm8994_write(codec, 0x36, 0x0003 ); 
	wm8994_write(codec, 0x621, 0x01c0 ); 
	wm8994_write(codec, 0x1, 0x3003 ); 

    // Unmute
	wm8994_write(codec, 0x4, 0x0303 );
	wm8994_write(codec, 0x610, 0x00c0 ); 
	wm8994_write(codec, 0x611, 0x00c0 ); 
	wm8994_write(codec, 0x420, 0x0000 );
*/
}
#endif


#if defined WM8994_REGISTER_DUMP
void wm8994_register_dump(struct snd_soc_codec *codec)
{
	int wm8994_register;

	for(wm8994_register = 0; wm8994_register <= 0x6; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));

	DEBUG_LOG_ERR("Register= [%X], Value = [%X]", 0x15, wm8994_read(codec, 0x15));

	for(wm8994_register = 0x18; wm8994_register <= 0x3C; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));

	DEBUG_LOG_ERR("Register= [%X], Value = [%X]", 0x4C, wm8994_read(codec, 0x4C));

	for(wm8994_register = 0x51; wm8994_register <= 0x5C; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));

	DEBUG_LOG_ERR("Register= [%X], Value = [%X]", 0x60, wm8994_read(codec, 0x60));
	DEBUG_LOG_ERR("Register= [%X], Value = [%X]", 0x101, wm8994_read(codec, 0x101));
	DEBUG_LOG_ERR("Register= [%X], Value = [%X]", 0x110, wm8994_read(codec, 0x110));
	DEBUG_LOG_ERR("Register= [%X], Value = [%X]", 0x111, wm8994_read(codec, 0x111));

	for(wm8994_register = 0x200; wm8994_register <= 0x212; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));
	
	for(wm8994_register = 0x220; wm8994_register <= 0x224; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));
	
	for(wm8994_register = 0x240; wm8994_register <= 0x244; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));

	for(wm8994_register = 0x300; wm8994_register <= 0x317; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));

	for(wm8994_register = 0x400; wm8994_register <= 0x411; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));

	for(wm8994_register = 0x420; wm8994_register <= 0x423; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));

	for(wm8994_register = 0x440; wm8994_register <= 0x444; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));

	for(wm8994_register = 0x450; wm8994_register <= 0x454; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));

	for(wm8994_register = 0x480; wm8994_register <= 0x493; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));

	for(wm8994_register = 0x4A0; wm8994_register <= 0x4B3; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));
	
	for(wm8994_register = 0x500; wm8994_register <= 0x503; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));
	
	DEBUG_LOG_ERR("Register= [%X], Value = [%X]", 0x510, wm8994_read(codec, 0x510));
	DEBUG_LOG_ERR("Register= [%X], Value = [%X]", 0x520, wm8994_read(codec, 0x520));
	DEBUG_LOG_ERR("Register= [%X], Value = [%X]", 0x521, wm8994_read(codec, 0x521));

	for(wm8994_register = 0x540; wm8994_register <= 0x544; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));
	
	for(wm8994_register = 0x580; wm8994_register <= 0x593; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));

	for(wm8994_register = 0x600; wm8994_register <= 0x614; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));
	
	DEBUG_LOG_ERR("Register= [%X], Value = [%X]", 0x620, wm8994_read(codec, 0x620));
	DEBUG_LOG_ERR("Register= [%X], Value = [%X]", 0x621, wm8994_read(codec, 0x621));

	for(wm8994_register = 0x700; wm8994_register <= 0x70A; wm8994_register++)
		DEBUG_LOG_ERR("Register= [%X], Value = [%X]", wm8994_register, wm8994_read(codec, wm8994_register));
		
}
#endif
