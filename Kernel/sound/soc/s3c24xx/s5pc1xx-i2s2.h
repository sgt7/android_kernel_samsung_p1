/* sound/soc/s3c24xx/s5pc1xx-i2s.h
 *
 * ALSA SoC Audio Layer - s5pc1xx I2S driver
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SND_SOC_S3C24XX_s5pc1xx_I2S_H
#define __SND_SOC_S3C24XX_s5pc1xx_I2S_H


#include "s3c-i2s2-v2.h"

struct clk;

#if 0 //sayanta commented
#define s5pc1xx_DIV_BCLK	S3C_I2SV2_DIV_BCLK
#define s5pc1xx_DIV_RCLK	S3C_I2SV2_DIV_RCLK
#define s5pc1xx_DIV_PRESCALER	S3C_I2SV2_DIV_PRESCALER

#define s5pc1xx_CLKSRC_PCLK	(0)
#define s5pc1xx_CLKSRC_MUX	(1)
#define s5pc1xx_CLKSRC_CDCLK    (2)
#define s5pc1xx_CLKSRC_I2SEXT   (3)//sayanta
#endif
extern struct snd_soc_dai s5pc1xx_i2s_dai[];
static struct s3c_i2s2v2_info *i2s;

extern struct clk *s5pc1xx_i2s_get_clock(struct snd_soc_dai *dai);

#define EXTCLK		"mout_audss"
#define EXTPRNT		"mout_epll"

#endif /* __SND_SOC_S3C24XX_s5pc1xx_I2S_H */
