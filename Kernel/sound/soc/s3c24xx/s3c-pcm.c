/* sound/soc/s3c24xx/s3c-pcm.c
 *
 * ALSA SoC Audio Layer - S3C PCM-Controller driver
 *
 * Copyright (c) 2009 Samsung Electronics Co. Ltd
 * Author: Jaswinder Singh <jassi.brar@samsung.com>
 * based upon I2S drivers by Ben Dooks.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <plat/audio.h>
//#include <plat/dma.h>

#include <mach/dma.h>
#include <mach/regs-clock.h>

#include "s3c-dma.h"
#include "s3c-pcm.h"
#include "s3c-pcmdev.h"

static unsigned char vtcallActive = 0;
static u32 clk_en_dis_play  = 0;
static u32 clk_en_dis_rec  = 0;
static unsigned char* curr_pos;
static u32 remaining_len;

struct s3c_pcmdev_info {
	void __iomem	*regs;
	struct clk	*pcm_clk;
	struct clk	*clk_src;
	u32		clk_rate;
	u32		pcmctl;
	u32		pcmclkctl;
	u32		pcmirqctl;
};

static struct s3c_pcmdev_info s3c_pcmdev;
static struct s3c2410_dma_client s3c_pcm_dma_client_out = {
	.name		= "PCM Stereo out"
};

static struct s3c2410_dma_client s3c_pcm_dma_client_in = {
	.name		= "PCM Stereo in"
};

static struct s3c_dma_params s3c_pcm_stereo_out[] = {
	[0] = {
		.client		= &s3c_pcm_dma_client_out,
		.dma_size	= 4,
		.channel	= DMACH_PCMDEV_OUT,
		.dma_addr	= S3C_PA_PCM + S3C_PCM_TXFIFO,
	},
	[1] = {
		.client		= &s3c_pcm_dma_client_out,
		.dma_size	= 4,
		.channel	= DMACH_PCMDEV_OUT,
		.dma_addr	= S3C_PA_PCM + S3C_PCM_TXFIFO,
	},
};

static struct s3c_dma_params s3c_pcm_stereo_in[] = {
	[0] = {
		.client		= &s3c_pcm_dma_client_in,
		.dma_size	= 4,
		.channel	= DMACH_PCMDEV_IN,
		.dma_addr	= S3C_PA_PCM + S3C_PCM_RXFIFO,
	},
	[1] = {
		.client		= &s3c_pcm_dma_client_in,
		.dma_size	= 4,
		.channel	= DMACH_PCMDEV_IN,
		.dma_addr	= S3C_PA_PCM + S3C_PCM_RXFIFO,
	},
};

static struct s3c_pcm_info s3c_pcm[2];

static inline struct s3c_pcm_info *to_info(struct snd_soc_dai *cpu_dai)
{
	return cpu_dai->private_data;
}

static void s3c_pcm_snd_txctrl(struct s3c_pcm_info *pcm, int on)
{
	void __iomem *regs = pcm->regs;
	u32 ctl, clkctl;
	u32 value;


	value = readl(regs + S3C_PCM_CTL);

	if (on) {
		value &= ~(S3C_PCMCTL_TXFIFO_DIPSTICK_MASK |
			S3C_PCMCTL_TX_DMA_EN | S3C_PCMCTL_TXFIFO_EN);
		value |= (0x8<<S3C_PCMCTL_TXFIFO_DIPSTICK_SHIFT) |
			S3C_PCMCTL_TX_DMA_EN | S3C_PCMCTL_TXFIFO_EN;

		writel(value, regs + S3C_PCM_CTL);
		pr_debug("%s: PCM_CTL=0X%x,PCM_CLKCTL=0X%x\n", __func__,
			readl(regs +S3C_PCM_CTL),
			readl(regs + S3C_PCM_CLKCTL));
	} else {
		value &= ~(S3C_PCMCTL_TX_DMA_EN | S3C_PCMCTL_TXFIFO_EN);
		writel(value, regs + S3C_PCM_CTL);
		value = readl(regs +S3C_PCM_IRQ_CTL);
		value &= ~(0x7FFF);
		pr_debug("%s: PCM_CTL=0X%x,PCM_CLKCTL=0X%x\n", __func__,
			readl(regs +S3C_PCM_CTL),
			readl(regs + S3C_PCM_CLKCTL));
	}
	pr_debug("%s: PCM_IRQ_STAT=0X%x,PCM_FIFO_STAT=0X%x,IRQ_CTL=0x%x\n",
		__func__, readl(regs +S3C_PCM_IRQ_STAT),
		readl(regs + S3C_PCM_FIFO_STAT),
		readl(regs +S3C_PCM_IRQ_CTL));
}

static void s3c_pcm_snd_rxctrl(struct s3c_pcm_info *pcm, int on)
{
	void __iomem *regs = pcm->regs;
	u32 ctl, clkctl;
	u32 value;

	if (on) {
		value = readl(regs + S3C_PCM_CTL);
		value &= ~(S3C_PCMCTL_RXFIFO_DIPSTICK_MASK |
			S3C_PCMCTL_RX_DMA_EN | S3C_PCMCTL_RXFIFO_EN);
		value |= (0x8 << S3C_PCMCTL_RXFIFO_DIPSTICK_SHIFT) |
			S3C_PCMCTL_RX_DMA_EN | S3C_PCMCTL_RXFIFO_EN;
		writel(value, regs + S3C_PCM_CTL);
		pr_debug("%s: PCM_CTL=0X%x,PCM_CLKCTL=0X%x\n", __func__,
			readl(regs +S3C_PCM_CTL),
			readl(regs + S3C_PCM_CLKCTL));
	} else {
		value = readl(regs + S3C_PCM_CTL);
		value &= ~(S3C_PCMCTL_RX_DMA_EN | S3C_PCMCTL_RXFIFO_EN);
		writel(value, regs + S3C_PCM_CTL);
		pr_debug("%s: PCM_CTL=0X%x,PCM_CLKCTL=0X%x\n", __func__,
			readl(regs +S3C_PCM_CTL),
			readl(regs + S3C_PCM_CLKCTL));
	}
	pr_debug("%s: PCM_IRQ_STAT=0X%x,PCM_FIFO_STAT=0X%x,IRQ_CTL=0x%x\n",
		__func__, readl(regs + S3C_PCM_IRQ_STAT),
		readl(regs + S3C_PCM_FIFO_STAT),
		readl(regs +S3C_PCM_IRQ_CTL));
        pr_debug("%s: S5P_CLK_SRC6=0x%x,S5P_CLKGATE_IP3=0x%x\n", __func__,
		readl(S5P_CLK_SRC6),readl(S5P_CLKGATE_IP3));
}

static int s3c_pcm_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct s3c_pcm_info *pcm = to_info(rtd->dai->cpu_dai);
	unsigned long flags;

	pr_debug("Entered %s\n", __func__);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock_irqsave(&pcm->lock, flags);

		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			s3c_pcm_snd_rxctrl(pcm, 1);
		else
			s3c_pcm_snd_txctrl(pcm, 1);

		spin_unlock_irqrestore(&pcm->lock, flags);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irqsave(&pcm->lock, flags);

		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			s3c_pcm_snd_rxctrl(pcm, 0);
		else
			s3c_pcm_snd_txctrl(pcm, 0);

		spin_unlock_irqrestore(&pcm->lock, flags);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int s3c_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *socdai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai = rtd->dai;
	struct s3c_pcm_info *pcm = to_info(dai->cpu_dai);
	struct s3c_dma_params *dma_data;
	void __iomem *regs = pcm->regs;
	struct clk *clk;
	int sclk_div, sync_div;
	unsigned long flags;
	u32 clkctl;
	u32 bfs, rfs, clk_div;
	u32 value, clk_rate;
	u32 audioclk,tmp;

	pr_debug("Entered %s\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = pcm->dma_playback;
	else
		dma_data = pcm->dma_capture;

	if (params_format(params) == SNDRV_PCM_FORMAT_U8)
		dma_data->dma_size = 2;

	snd_soc_dai_set_dma_data(dai->cpu_dai, substream, dma_data);

	spin_lock_irqsave(&pcm->lock, flags);
	switch (params_format(params)) {
        case SNDRV_PCM_FORMAT_S8:
                bfs = 16;
                rfs = 256;              /* Can take any RFS value for AP */
                break;
        case SNDRV_PCM_FORMAT_S16_LE:
                bfs = 32;
                rfs = 256;              /* Can take any RFS value for AP */
                break;
        case SNDRV_PCM_FORMAT_S20_3LE:
        case SNDRV_PCM_FORMAT_S24_LE:
		/* B'coz 48-BFS needs atleast 512-RFS
		 * acc to *S5P6440* UserManual
		 */
		bfs = 48;
		rfs = 512;
		break;
        case SNDRV_PCM_FORMAT_S32_LE:
		/* Impossible, as the AP doesn't support 64fs or more BFS */
	default:
		pr_err("%s: Default format not supported\n", __func__);
                return -EINVAL;
        }

	/* Enable the interrupts */
	value = readl(regs + S3C_PCM_IRQ_CTL);
	value |= S3C_PCMIRQSTAT_TXFIFO_ALMOST_EMPTY;
	value |= S3C_PCMIRQ_EN_IRQ_TO_ARM;

	clk = clk_get(NULL, RATESRCCLK);
	if (IS_ERR(clk)) {
		pr_err("%s: failed to get %s\n", __func__, RATESRCCLK);
		return -EBUSY;
	}
	clk_disable(clk);
	clk_div = readl(S5P_CLK_DIV6);
	clk_div &= ~(0xF<<4);
 
	switch (params_rate(params)) {
	case 8000:
		clk_set_rate(clk, 49152000);
		clk_div |= (3<<4);
		writel(clk_div, S5P_CLK_DIV6);
                audioclk = (49152000/4);
                break;

        case 16000:
		clk_set_rate(clk, 49152000);
		clk_div |= (1<<4);
		writel(clk_div, S5P_CLK_DIV6);
                audioclk = (49152000/2);
                break;

        case 32000:
		clk_set_rate(clk, 49152000);
                audioclk = 49152000;
		clk_div |= (0<<4);
                writel(clk_div, S5P_CLK_DIV6);
                break;
 
	case 64000:
		clk_set_rate(clk, 49152000);
                audioclk = 49152000;
		clk_div |= (0<<4);
                writel(clk_div, S5P_CLK_DIV6);
                break;

       	case 48000:
		clk_set_rate(clk, 73728000);
                audioclk = 73728000;
		clk_div |= (0<<4);
                writel(clk_div, S5P_CLK_DIV6);
                break;

       	case 96000:
                clk_set_rate(clk, 73728000);
		audioclk = 73728000;
		clk_div |= (0<<4);
                writel(clk_div, S5P_CLK_DIV6);
                break;

       	case 11025:
		clk_set_rate(clk, 67738000);
                audioclk = 67738000/4;
		clk_div |= (3<<4);
                writel(clk_div, S5P_CLK_DIV6);
		break;

       	case 22050:
		clk_set_rate(clk, 67738000);
                audioclk = 67738000/2;
		clk_div |= (1<<4);
                writel(clk_div, S5P_CLK_DIV6);
		break;

       	case 44100:
		clk_set_rate(clk, 67738000);
                audioclk = 67738000;
		clk_div |= (0<<4);
                writel(clk_div, S5P_CLK_DIV6);
		break;

      	case 88200:
                clk_set_rate(clk, 67738000);
		audioclk = 67738000;
		clk_div |= (0<<4);
                writel(clk_div, S5P_CLK_DIV6);
               	break;
       	default:
       		pr_err("%s: Required Rate =%d..not supported\n", __func__,
			params_rate(params));
		return -EINVAL;
        }

	clk_rate = (params_rate(params)*bfs);
	tmp = ((audioclk/(clk_rate))/2)-1;
	pr_debug("%s: clk_rate=%d,sclk_div=%d,clk_div6=0x%x,smaple_rate=%d\n",
		__func__, clk_rate, tmp, readl(S5P_CLK_DIV6),
		params_rate(params));
	sclk_div = tmp;

	sync_div = bfs-1;

	value = readl(regs + S3C_PCM_CLKCTL);
	value &= ~(S3C_PCMCLKCTL_SCLK_DIV | S3C_PCMCLKCTL_SYNC_DIV);
        value |= (sclk_div << 9);
        value |= (sync_div << 0);

        writel(value, regs + S3C_PCM_CLKCTL);

	clk_enable(clk);
        s3c_pcmdev.clk_rate = clk_get_rate(s3c_pcmdev.clk_src);
        pr_debug("%s: Setting FOUTepll to %dHz\n", __func__, 	s3c_pcmdev.clk_rate);

        s3c_pcmdev.clk_rate = clk_get_rate(s3c_pcmdev.pcm_clk);
        clk_put(clk);

	spin_unlock_irqrestore(&pcm->lock, flags);

	pr_debug("PCMSOURCE_CLK-%lu SCLK=%ufs SCLK_DIV=%d SYNC_DIV=%d\n",
				clk_get_rate(clk), pcm->sclk_per_fs,
				sclk_div, sync_div);

	return 0;
}

static bool value_saved = false;
static int s3c_pcm_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *socdai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai = rtd->dai;
	struct s3c_pcm_info *pcm = to_info(dai->cpu_dai);
	void __iomem *regs = pcm->regs;

	pr_debug("%s:\n", __func__);

	if (vtcallActive == 1) {
		pr_debug("%s : clock is already setup!!", __func__);
		return 0;
	}

	if (!clk_en_dis_rec && !clk_en_dis_play) {
		clk_enable(s3c_pcmdev.clk_src);
		clk_enable(s3c_pcmdev.pcm_clk);
		if (value_saved == true) {
			writel(s3c_pcmdev.pcmctl,
				regs + S3C_PCM_CTL);
			writel(s3c_pcmdev.pcmclkctl,
				regs + S3C_PCM_CLKCTL);
			writel(s3c_pcmdev.pcmirqctl,
				regs + S3C_PCM_IRQ_CTL);
			value_saved = false;
		}
		pr_info("%s: enabled pcm clocks finally\n", __func__);
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		clk_en_dis_play =1;
	else
		clk_en_dis_rec = 1;

	writel(readl(regs + S3C_PCM_CLKCTL) | S3C_PCMCLKCTL_SERCLK_EN,
		regs + S3C_PCM_CLKCTL);
	writel(readl(regs +S3C_PCM_CTL) | S3C_PCMCTL_ENABLE,
		regs + S3C_PCM_CTL);
	pr_debug("%s: PCM_CTL=0X%x,PCM_CLKCTL=0X%x\n", __func__,
		readl(regs +S3C_PCM_CTL),
		readl(regs + S3C_PCM_CLKCTL));
	return 0;
}

static void s3c_pcm_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *socdai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai = rtd->dai;
	struct s3c_pcm_info *pcm = to_info(dai->cpu_dai);
	void __iomem *regs = pcm->regs;
	u32 value;

	if (vtcallActive == 1) {
		pr_debug("%s : vtcall is using!!", __func__);
		return;
	}

	value = readl(regs + S3C_PCM_CTL);
	value &= ~(S3C_PCMCTL_ENABLE);
        writel(value, regs + S3C_PCM_CTL);
        writel(readl(regs + S3C_PCM_CLKCTL) &
		~S3C_PCMCLKCTL_SERCLK_EN, regs + S3C_PCM_CLKCTL);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
                clk_en_dis_play = 0;
        else
                clk_en_dis_rec =  0;

	if ((!clk_en_dis_play) && (!clk_en_dis_rec)) {
		if(value_saved == true)
			return;
		s3c_pcmdev.pcmctl= readl(regs + S3C_PCM_CTL);
	        s3c_pcmdev.pcmclkctl = readl(regs + S3C_PCM_CLKCTL);
        	s3c_pcmdev.pcmirqctl = readl(regs + S3C_PCM_IRQ_CTL);
		value_saved = true;
		clk_disable(s3c_pcmdev.pcm_clk);
		clk_disable(s3c_pcmdev.clk_src);
		pr_info("%s: disabled pcm clocks finally\n", __func__);
	}
}

static int s3c_pcm_prepare(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{

	pr_debug("%s:\n", __func__);
	return 0;
}

static int s3c_pcm_set_fmt(struct snd_soc_dai *cpu_dai,
			       unsigned int fmt)
{
	struct s3c_pcm_info *pcm = to_info(cpu_dai);
	void __iomem *regs = pcm->regs;
	unsigned long flags;
	int ret = 0;
	u32 ctl;
	u32 value;

	pr_debug("Entered %s\n", __func__);

	spin_lock_irqsave(&pcm->lock, flags);

	value = readl(regs + S3C_PCM_CTL);
	value &= ~(S3C_PCMCTL_TX_MSB_POS_MASK | S3C_PCMCTL_RX_MSB_POS_MASK);
 
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
        case SND_SOC_DAIFMT_I2S: /* Data starts (MSB) 1 clock after PCMSYNC signal */
		value |= S3C_PCMCTL_TX_MSB_POS1 | S3C_PCMCTL_RX_MSB_POS1;
                break;
	case SND_SOC_DAIFMT_DSP_B:
		value |= S3C_PCMCTL_TX_MSB_POS0 | S3C_PCMCTL_RX_MSB_POS0;
                break;
	case SND_SOC_DAIFMT_DSP_A:
                value |= S3C_PCMCTL_TX_MSB_POS1 | S3C_PCMCTL_RX_MSB_POS1;
                break;
        default:
		pr_info("%s: Invalid DAI format specified - 0x%x\n",
			__func__, fmt & SND_SOC_DAIFMT_FORMAT_MASK);
                return -EINVAL;
        }

	writel(value, regs + S3C_PCM_CTL);
	pr_debug("%s: PCM_CTL=0X%x,PCM_CLKCTL=0X%x\n", __func__,
		readl(regs +S3C_PCM_CTL),
		readl(regs + S3C_PCM_CLKCTL));
exit:
	spin_unlock_irqrestore(&pcm->lock, flags);

	return ret;
}

static int s3c_pcm_set_clkdiv(struct snd_soc_dai *cpu_dai,
						int div_id, int div)
{
	struct s3c_pcm_info *pcm = to_info(cpu_dai);
	return 0;
}

static int s3c_pcm_set_sysclk(struct snd_soc_dai *cpu_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct s3c_pcm_info *pcm = to_info(cpu_dai);
	void __iomem *regs = pcm->regs;
	u32 clkctl = readl(regs + S3C_PCM_CLKCTL);
	u32 value;	
	pr_debug("%s:\n", __func__);
	s3c_pcmdev.clk_rate = clk_get_rate(s3c_pcmdev.pcm_clk);
	value = readl(regs + S3C_PCM_CLKCTL);
	value |= ~(S3C_PCMCLKCTL_SERCLK_SEL); /* ..using  SCLK_* clock source */
	writel(value, regs + S3C_PCM_CLKCTL);
	return 0;
}

static struct snd_soc_dai_ops s3c_pcm_dai_ops = {
	.set_sysclk	= s3c_pcm_set_sysclk,
	.set_clkdiv	= s3c_pcm_set_clkdiv,
	.trigger	= s3c_pcm_trigger,
	.hw_params	= s3c_pcm_hw_params,
	.prepare	= s3c_pcm_prepare,
	.set_fmt	= s3c_pcm_set_fmt,
	.startup	= s3c_pcm_startup,
	.shutdown	= s3c_pcm_shutdown,
};

#define S3C_PCM_RATES  SNDRV_PCM_RATE_8000_96000

#define S3C_PCM_DECLARE(n)			\
{								\
	.name		 = "samsung-pcm",			\
	.id		 = (n),				\
	.symmetric_rates = 1,					\
	.ops = &s3c_pcm_dai_ops,				\
	.playback = {						\
		.channels_min	= 1,				\
		.channels_max	= 2,				\
		.rates		= S3C_PCM_RATES,		\
		.formats	= SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S8 |		\
				SNDRV_PCM_FMTBIT_U16_LE,	\
	},							\
	.capture = {						\
		.channels_min	= 1,				\
		.channels_max	= 2,				\
		.rates		= S3C_PCM_RATES,		\
		.formats	= SNDRV_PCM_FMTBIT_S8 |		\
				SNDRV_PCM_FMTBIT_S16_LE		\
	},							\
}

struct snd_soc_dai s3c_pcm_dai[] = {
	S3C_PCM_DECLARE(0),
	S3C_PCM_DECLARE(1),
};
EXPORT_SYMBOL_GPL(s3c_pcm_dai);

/* TODO: Added to use in interrupt mode.
 * interrupt mode implementation has to be done
 */
static irqreturn_t s3c_pcmdev_irq(int irqno, void *dev_id)
{
	u32 irq_status;
	static u32 count;

	irq_status = readl(s3c_pcmdev.regs + S3C_PCM_IRQ_STAT);
	pr_debug("%s: irq count %d, status = 0x%x..fifo_stat =0x%x\n", 
		__func__, count++, irq_status,
		readl(s3c_pcmdev.regs + S3C_PCM_FIFO_STAT));

	pr_debug("%s: IRQ..check point..curr_pos=0x%x..\n", __func__,
		(u32)curr_pos);
	if (remaining_len) {
		writel(0x1,s3c_pcmdev.regs + S3C_PCM_CLRINT);
	}
	else
		writel(0x1,s3c_pcmdev.regs + S3C_PCM_CLRINT);

	return IRQ_HANDLED;
}
int s3c_pcmdev_clock_control(int enable)
{
	vtcallActive = enable;
	return 0;
}
static __devinit int s3c_pcm_dev_probe(struct platform_device *pdev)
{
	struct s3c_pcm_info *pcm;
	struct snd_soc_dai *dai;
	struct resource *mem_res, *dmatx_res, *dmarx_res;
	struct s3c_audio_pdata *pcm_pdata;
	struct clk *cm, *cf;
	int ret;

	/* Check for valid device index */
	if ((pdev->id < 0) || pdev->id >= ARRAY_SIZE(s3c_pcm)) {
		dev_err(&pdev->dev, "id %d out of range\n", pdev->id);
		return -EINVAL;
	}

	pcm_pdata = pdev->dev.platform_data;

	if (pcm_pdata && pcm_pdata->cfg_gpio && pcm_pdata->cfg_gpio(pdev)) {
		dev_err(&pdev->dev, "Unable to configure gpio\n");
		return -EINVAL;
	}

	pcm = &s3c_pcm[pdev->id];
	pcm->dev = &pdev->dev;

	spin_lock_init(&pcm->lock);

	dai = &s3c_pcm_dai[pdev->id];
	dai->dev = &pdev->dev;

	pcm->regs = ioremap(S3C_PA_PCM, 0x100);
	if (pcm->regs == NULL) {
		dev_err(&pdev->dev, "cannot ioremap registers\n");
		ret = -ENXIO;
		goto err3;
	}

	s3c_pcmdev.regs = pcm->regs;

	ret = request_irq(IRQ_S3C_PCM, s3c_pcmdev_irq, 0, "s3c-pcmdev", pdev);
	if (ret < 0) {
		pr_err("%s: fail to claim pcmdev irq , ret = %d\n",
			__func__, ret);
		iounmap(s3c_pcmdev.regs);
		return -ENODEV;
	}
	/* Default is 128fs */
	pcm->sclk_per_fs = 128;

	s3c_pcmdev.pcm_clk = clk_get(&pdev->dev, "pcm");
	if (IS_ERR(s3c_pcmdev.pcm_clk)) {
		dev_err(&pdev->dev, "failed to get pcm_clock\n");
		ret = -ENOENT;
		goto err4;
	}

	s3c_pcmdev.clk_rate = clk_get_rate(s3c_pcmdev.pcm_clk);
	s3c_pcmdev.clk_src = clk_get(&pdev->dev, "sclk_audio");
	if (IS_ERR(s3c_pcmdev.clk_src)) {
		dev_err(&pdev->dev, "failed to get sclk_audio\n");
		ret = PTR_ERR(s3c_pcmdev.clk_src);
		goto err1;
	}

	cm = clk_get(NULL, EXTPRNT);
	if (IS_ERR(cm)) {
		pr_err("%s: failed to get %s\n", __func__, EXTPRNT);
	}

	if (clk_set_parent(s3c_pcmdev.clk_src, cm)) {
		pr_err("%s: failed to set mOUTepll as parent of scklkaudio1\n",
			__func__);
	}

	cf = clk_get(NULL, "fout_epll");
	if (IS_ERR(cf)) {
		pr_err("%s: failed to get fout_epll\n", __func__);
	}

	if (clk_set_parent(cm, cf)){
		pr_err("%s: failed to set FOUTepll as parent of MOUTepll\n",
			__func__);
	}
	s3c_pcmdev.clk_rate = clk_get_rate(s3c_pcmdev.clk_src);
	/* record our pcm structure for later use in the callbacks */
	dai->private_data = pcm;


	//clk_enable(pcm->cclk);
	//clk_enable(pcm->pclk);

	clk_put(cf);
	clk_put(cm);
	s3c_pcm_snd_txctrl(pcm, 0);
	s3c_pcm_snd_rxctrl(pcm, 0);

	ret = snd_soc_register_dai(dai);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to get pcm_clock\n");
		goto err5;
	}

	pcm->dma_capture = &s3c_pcm_stereo_in[pdev->id];
	pcm->dma_playback = &s3c_pcm_stereo_out[pdev->id];

	return 0;

err5:
	clk_disable(pcm->pclk);
	clk_put(s3c_pcmdev.pcm_clk);
err4:
	iounmap(pcm->regs);
err3:
	release_mem_region(mem_res->start, resource_size(mem_res));
err2:
	clk_disable(s3c_pcmdev.clk_src);
	clk_put(s3c_pcmdev.clk_src);
err1:
	return ret;
}

static __devexit int s3c_pcm_dev_remove(struct platform_device *pdev)
{
	struct s3c_pcm_info *pcm = &s3c_pcm[pdev->id];
	struct resource *mem_res;

	iounmap(pcm->regs);

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem_res->start, resource_size(mem_res));

	clk_disable(s3c_pcmdev.clk_src);
	clk_disable(s3c_pcmdev.pcm_clk);
	clk_put(s3c_pcmdev.pcm_clk);
	clk_put(s3c_pcmdev.clk_src);

	return 0;
}

#ifdef CONFIG_PM
static int s3c_pcm_dev_suspend(struct platform_device *dev, pm_message_t state)
{
	pr_debug("%s:\n", __func__);

	s3c_pcmdev.pcmctl= readl(s3c_pcmdev.regs + S3C_PCM_CTL);
	s3c_pcmdev.pcmclkctl = readl(s3c_pcmdev.regs + S3C_PCM_CLKCTL);
	s3c_pcmdev.pcmirqctl = readl(s3c_pcmdev.regs + S3C_PCM_IRQ_CTL);

	if ((clk_en_dis_play)||(clk_en_dis_rec)) {
		clk_disable(s3c_pcmdev.pcm_clk);
        	clk_disable(s3c_pcmdev.clk_src);
	}
	return 0;
}

static int s3c_pcm_dev_resume(struct platform_device *dev)
{
	pr_debug("%s:\n", __func__);

	if ((clk_en_dis_play)||(clk_en_dis_rec)) {
		clk_enable(s3c_pcmdev.pcm_clk);
        	clk_enable(s3c_pcmdev.clk_src);
	}

	writel(s3c_pcmdev.pcmctl, s3c_pcmdev.regs + S3C_PCM_CTL);
	writel(s3c_pcmdev.pcmclkctl, s3c_pcmdev.regs + S3C_PCM_CLKCTL);
	writel(s3c_pcmdev.pcmirqctl, s3c_pcmdev.regs + S3C_PCM_IRQ_CTL);

	return 0;
}
#else
#define s3c_pcmdev_suspend NULL
#define s3c_pcmdev_resume NULL
#endif

static struct platform_driver s3c_pcm_driver = {
	.probe  = s3c_pcm_dev_probe,
	.remove = s3c_pcm_dev_remove,
	.driver = {
		.name = "samsung-pcm",
		.owner = THIS_MODULE,
	},
	.suspend = s3c_pcm_dev_suspend,
	.resume = s3c_pcm_dev_resume,
};

static int __init s3c_pcm_init(void)
{
	return platform_driver_register(&s3c_pcm_driver);
}
module_init(s3c_pcm_init);

static void __exit s3c_pcm_exit(void)
{
	platform_driver_unregister(&s3c_pcm_driver);
}
module_exit(s3c_pcm_exit);

/* Module information */
MODULE_AUTHOR("Jaswinder Singh, <jassi.brar@samsung.com>");
MODULE_DESCRIPTION("S3C PCM Controller Driver");
MODULE_LICENSE("GPL");
