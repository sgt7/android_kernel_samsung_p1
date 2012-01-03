/*
 * s5p-i2s_sec.c  --  Secondary Fifo driver for I2S_v5
 *
 * (c) 2009 Samsung Electronics Co. Ltd
 *   - Jaswinder Singh Brar <jassi.brar@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/io.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <plat/regs-iis.h>

#include <mach/dma.h>

#include "s3c-dma.h"
#include "s3c-idma.h"

static void __iomem *s5p_i2s0_regs;

static struct s3c2410_dma_client s5p_dma_client_outs = {
	.name		= "I2S_Sec PCM Stereo out"
};

static struct s3c_dma_params s5p_i2s_sec_pcm_out = {
	.channel	= DMACH_I2S0S_TX,
	.client		= &s5p_dma_client_outs,
	.dma_size	= 4,
};

static void s5p_snd_txctrl(int on)
{
	u32 iiscon, iismod;
	u32 iisfic; //GNUX@2010.09.28 : avoid tick noise

	iiscon = readl(s5p_i2s0_regs + S3C2412_IISCON);
	iismod = readl(s5p_i2s0_regs + S3C2412_IISMOD);

	if (on) {
#if 1 //GNUX@2010.09.28 : avoid tick noise
		// Flush I2S FIFO
		iisfic = readl(s5p_i2s0_regs + S5P_IISFICS);
		iisfic |= S3C2412_IISFIC_TXFLUSH;
		writel(iisfic, s5p_i2s0_regs + S5P_IISFICS);

		do {
			cpu_relax();
		} while ((__raw_readl(s5p_i2s0_regs + S5P_IISFICS) >> 8) & 0x7f);

		iisfic = readl(s5p_i2s0_regs + S5P_IISFICS);
		iisfic &= ~S3C2412_IISFIC_TXFLUSH;
		writel(iisfic, s5p_i2s0_regs + S5P_IISFICS);
#endif

		iiscon |= S3C2412_IISCON_IIS_ACTIVE;
		iiscon &= ~S3C2412_IISCON_TXCH_PAUSE;
		iiscon &= ~S5P_IISCON_TXSDMAPAUSE;
		iiscon |= S5P_IISCON_TXSDMACTIVE;

		switch (iismod & S3C2412_IISMOD_MODE_MASK) {
		case S3C2412_IISMOD_MODE_TXONLY:
		case S3C2412_IISMOD_MODE_TXRX:
			/* do nothing, we are in the right mode */
			break;

		case S3C2412_IISMOD_MODE_RXONLY:
			iismod &= ~S3C2412_IISMOD_MODE_MASK;
			iismod |= S3C2412_IISMOD_MODE_TXRX;
			break;

		default:
			printk(KERN_ERR "TXEN: Invalid MODE %x in IISMOD\n",
				iismod & S3C2412_IISMOD_MODE_MASK);
			break;
		}

		writel(iiscon, s5p_i2s0_regs + S3C2412_IISCON);
		writel(iismod, s5p_i2s0_regs + S3C2412_IISMOD);
	} else {
		iiscon |= S5P_IISCON_TXSDMAPAUSE;
		iiscon &= ~S5P_IISCON_TXSDMACTIVE;

		/* return if primary is active */
		if (iiscon & S3C2412_IISCON_TXDMA_ACTIVE) {
			writel(iiscon, s5p_i2s0_regs + S3C2412_IISCON);
			return;
		}

		iiscon |= S3C2412_IISCON_TXCH_PAUSE;

		switch (iismod & S3C2412_IISMOD_MODE_MASK) {
		case S3C2412_IISMOD_MODE_TXRX:
			iismod &= ~S3C2412_IISMOD_MODE_MASK;
			iismod |= S3C2412_IISMOD_MODE_RXONLY;
			break;

		case S3C2412_IISMOD_MODE_TXONLY:
			iismod &= ~S3C2412_IISMOD_MODE_MASK;
			iiscon &= ~S3C2412_IISCON_IIS_ACTIVE;
			break;

		default:
			printk(KERN_ERR "TXDIS: Invalid MODE %x in IISMOD\n",
				iismod & S3C2412_IISMOD_MODE_MASK);
			break;
		}

		writel(iismod, s5p_i2s0_regs + S3C2412_IISMOD);
		writel(iiscon, s5p_i2s0_regs + S3C2412_IISCON);
	}
}

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)

/*
 * Wait for the LR signal to allow synchronisation to the L/R clock
 * from the codec. May only be needed for slave mode.
 */
static int s5p_snd_lrsync(void)
{
	u32 iiscon;
	unsigned long loops = msecs_to_loops(1);

	pr_debug("Entered %s\n", __func__);

	while (--loops) {
		iiscon = readl(s5p_i2s0_regs + S3C2412_IISCON);
		if (iiscon & S3C2412_IISCON_LRINDEX)
			break;

		cpu_relax();
	}

	if (!loops) {
		pr_debug("%s: timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}

int s5p_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	u32 iismod;

	snd_soc_dai_set_dma_data(rtd->dai->cpu_dai, substream,
					&s5p_i2s_sec_pcm_out);

	iismod = readl(s5p_i2s0_regs + S3C2412_IISMOD);

	/* Copy the same bps as Primary */
	iismod &= ~S5P_IISMOD_BLCSMASK;
	iismod |= ((iismod & S5P_IISMOD_BLCPMASK) << 2);

	writel(iismod, s5p_i2s0_regs + S3C2412_IISMOD);

	return 0;
}
EXPORT_SYMBOL_GPL(s5p_i2s_hw_params);

int s5p_i2s_startup(struct snd_soc_dai *dai)
{
	u32 iiscon, iisfic;
	u32 iismod, iisahb;

	iiscon = readl(s5p_i2s0_regs + S3C2412_IISCON);
	iismod = readl(s5p_i2s0_regs + S3C2412_IISMOD);
	iisahb = readl(s5p_i2s0_regs + S5P_IISAHB);

	iisahb |= (S5P_IISAHB_DMARLD | S5P_IISAHB_DISRLDINT);
	iismod |= S5P_IISMOD_TXSLP;

	writel(iisahb, s5p_i2s0_regs + S5P_IISAHB);
	writel(iismod, s5p_i2s0_regs + S3C2412_IISMOD);

	/* FIFOs must be flushed before enabling PSR
	*  and other MOD bits, so we do it here. */
	if (iiscon & S5P_IISCON_TXSDMACTIVE)
		return 0;

	iisfic = readl(s5p_i2s0_regs + S5P_IISFICS);
	iisfic |= S3C2412_IISFIC_TXFLUSH;
	writel(iisfic, s5p_i2s0_regs + S5P_IISFICS);

	do {
		cpu_relax();
	} while ((__raw_readl(s5p_i2s0_regs + S5P_IISFICS) >> 8) & 0x7f);

	iisfic = readl(s5p_i2s0_regs + S5P_IISFICS);
	iisfic &= ~S3C2412_IISFIC_TXFLUSH;
	writel(iisfic, s5p_i2s0_regs + S5P_IISFICS);

	return 0;
}
EXPORT_SYMBOL_GPL(s5p_i2s_startup);

int s5p_i2s_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* We don't configure clocks from this Sec i/f.
		 * So, we simply wait enough time for LRSYNC to
		 * get synced and not check return 'error'
		 */
		s5p_snd_lrsync();
		s5p_snd_txctrl(1);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		s5p_snd_txctrl(0);
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(s5p_i2s_trigger);

void s5p_i2s_sec_init(void *regs, dma_addr_t phys_base)
{
	u32 val;
#ifdef CONFIG_ARCH_S5PV210
/* S5PC110 or S5PV210 */
#define S3C_VA_AUDSS	S3C_ADDR(0x01600000)	/* Audio SubSystem */
#include <mach/regs-audss.h>
	/* We use I2SCLK for rate generation, so set EPLLout as
	 * the parent of I2SCLK.
	 */
	val = readl(S5P_CLKSRC_AUDSS);
	val &= ~(0x3<<2);
	val |= (1<<0);
	writel(val, S5P_CLKSRC_AUDSS);

	val = readl(S5P_CLKGATE_AUDSS);
	val |= (0x7f<<0);
	writel(val, S5P_CLKGATE_AUDSS);
#else
	#error INITIALIZE HERE!
#endif

	s5p_i2s0_regs = regs;
	s5p_i2s_sec_pcm_out.dma_addr = phys_base + S5P_IISTXDS;

	s5p_snd_txctrl(0);
	s5p_idma_init(regs);
}

/* Module information */
MODULE_AUTHOR("Jaswinder Singh <jassi.brar@samsung.com>");
MODULE_DESCRIPTION("S5P I2S-SecFifo SoC Interface");
MODULE_LICENSE("GPL");
