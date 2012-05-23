/* sound/soc/s3c24xx/s5pc1xx-i2s2.c
 *
 * ALSA SoC Audio Layer - s5pc1xx I2S2 driver
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include <sound/soc.h>

#include <mach/dma.h>
#include <mach/map.h>
#include <mach/irqs.h> 
#include <mach/regs-clock.h> 
#include <mach/regs-audss.h> 
#include "s3c-dma.h"
#include "s5pc1xx-i2s2.h"
/* The value should be set to maximum of the total number
 * of I2Sv3 controllers that any supported SoC has.
 */
#define MAX_I2SV3	3

extern struct snd_soc_dai i2s_sec_fifo_dai;
extern void s5p_i2s_sec_init(void *, dma_addr_t);

static struct s3c2410_dma_client s5pc1xx_dma_client_out = {
	.name		= "I2S2 PCM Stereo out"
};

static struct s3c2410_dma_client s5pc1xx_dma_client_in = {
	.name		= "I2S2 PCM Stereo in"
};

static struct s3c_dma_params s5pc1xx_i2s_pcm_stereo_out[MAX_I2SV3];
static struct s3c_dma_params s5pc1xx_i2s_pcm_stereo_in[MAX_I2SV3];
static struct s3c_i2s2v2_info s5pc1xx_i2s[MAX_I2SV3];
static struct s3c_i2s2v2_info *i2s;
struct snd_soc_dai s5pc1xx_i2s2_dai[MAX_I2SV3];

EXPORT_SYMBOL_GPL(s5pc1xx_i2s2_dai);

static inline struct s3c_i2s2v2_info *to_info(struct snd_soc_dai *cpu_dai)
{
	return cpu_dai->private_data;
}

static irqreturn_t s3c_iis_irq(int irqno, void *dev_id)
{
	u32 iiscon;
	//u32	val;
	//TODO
	//dump_i2s();
#if 1
	if(readl(i2s->regs + S3C_IISCON) & (1<<26))
	{
		//printk("\n rx overflow int ..@=%d\n",__LINE__);
		writel(readl(i2s->regs + S3C_IISCON) | (1<<26),i2s->regs + S3C_IISCON); //clear rxfifo overflow interrupt
//		writel(readl(i2s->regs + S3C_IISFIC) | (1<<7) , i2s->regs + S3C_IISFIC); //flush rx 
	}
	//printk("++++IISCON = 0x%08x\n, IISFIC 0x%x", readl(i2s->regs + S3C_IISCON),readl(i2s->regs + S3C_IISFIC));
#endif

	iiscon  = readl(i2s->regs + S3C_IISCON);
	if(iiscon & S3C_IISCON_FTXURSTATUS) {
		// iiscon &= ~S3C_IISCON_FTXURINTEN;	
		iiscon |= S3C_IISCON_FTXURSTATUS;	
		writel(iiscon, i2s->regs + S3C_IISCON);
		//printk("TX_P underrun interrupt IISCON = 0x%08x\n", readl(i2s->regs + S3C_IISCON));
	}

// latinlab system : js0809.kim ATV lockup issue
	  	if(iiscon & S3C_IISCON_FRXOFSTAT) {
		iiscon |= S3C_IISCON_FRXOFSTAT;
		writel(iiscon, i2s->regs + S3C_IISCON);
	}
// latinlab system : js0809.kim ATV lockup issue

	return IRQ_HANDLED;
}

#if 0 // CYS_ I2S
struct clk *s5pc1xx_i2s_get_clock(struct snd_soc_dai *dai)
{

	return i2s->i2sclk;
}
EXPORT_SYMBOL_GPL(s5pc1xx_i2s_get_clock);
#endif

#define s5pc1xx_I2S_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 | \
	SNDRV_PCM_RATE_KNOT)

#define s5pc1xx_I2S_FMTS \
	(SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE |\
	 SNDRV_PCM_FMTBIT_S24_LE)

static int s5pc1xx_iis_dev_probe(struct platform_device *pdev,struct snd_soc_dai *dai)
{
	struct s3c_audio_pdata *i2s_pdata;
	//struct resource *res;//getting resources was failing
	int ret = 0; 
	struct clk *cm , *cn , *cf; 
	if (pdev->id >= MAX_I2SV3) {
		dev_err(&pdev->dev, "id %d out of range\n", pdev->id);
		return -EINVAL;
	}

	i2s = &s5pc1xx_i2s[pdev->id];
	i2s->dma_capture = &s5pc1xx_i2s_pcm_stereo_in[pdev->id];
	i2s->dma_playback = &s5pc1xx_i2s_pcm_stereo_out[pdev->id];
#if 0
	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get I2S-TX dma resource\n");
		return -ENXIO;
	}
	//i2s->dma_playback->channel = res->start;
#endif //sayanta commmented.. it is unable to get resource
	i2s->dma_playback->channel = DMACH_I2S2_TX; 
#if 0
	res = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get I2S-RX dma resource\n");
		return -ENXIO;
	}
	//i2s->dma_capture->channel = res->start;
#endif //sayanta commmented.. it is unable to get resource
	i2s->dma_capture->channel = DMACH_I2S2_RX; 

#if 0
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get I2S SFR address\n");
		return -ENXIO;
	}
	if (!request_mem_region(res->start, resource_size(res),
				"s5pc1xx-i2s")) {
		dev_err(&pdev->dev, "Unable to request SFR region\n");
		return -EBUSY;
	}
#endif ////sayanta commmented..it is unable to get resource


	i2s->dma_capture->client = &s5pc1xx_dma_client_in;
	i2s->dma_capture->dma_size = 4;
	i2s->dma_playback->client = &s5pc1xx_dma_client_out;
	i2s->dma_playback->dma_size = 4;

	i2s_pdata = pdev->dev.platform_data;

	i2s->regs = ioremap(S3C_IIS_PABASE, 0x100); 
	if (i2s->regs == NULL)
		return -ENXIO;

	ret = request_irq(S3C_IISIRQ, s3c_iis_irq, 0, "s3c-i2s2", pdev);
	if (ret < 0) {
		printk("fail to claim i2s2 irq , ret = %d\n", ret);
		iounmap(i2s->regs);
		return -ENODEV;
	}

	i2s->dma_capture->dma_addr = S3C_IIS_PABASE + S3C_IISRXD;
	i2s->dma_playback->dma_addr = S3C_IIS_PABASE + S3C_IISTXD;
	/* Configure the I2S pins if MUX'ed */
	/*	if (i2s_pdata && i2s_pdata->cfg_gpio && i2s_pdata->cfg_gpio(pdev)) {  
		dev_err(&pdev->dev, "Unable to configure gpio\n");
		return -EINVAL;
		}*/ //sayanta commented..to avoid compilation error ..need to check usage

	i2s->iis1_ip_clk = clk_get(&pdev->dev, CLK_I2S1);
	if (IS_ERR(i2s->iis1_ip_clk)) {
		printk("failed to get clk(%s)\n", CLK_I2S1);
		goto err;
	}
	printk("Got Clock -> %s\n", CLK_I2S1);

	i2s->i2sclk = clk_get(&pdev->dev, "sclk_audio");
	if (IS_ERR(i2s->i2sclk)) {
		printk("failed to get clk sclk_audio\n");
		goto lb3;
	}

	cm = clk_get(NULL, "mout_epll");
	if (IS_ERR(cm)) {
		printk("failed to get mout_epll\n");
		goto lb2;
	}
	printk("Got Audio Bus Source Clock -> mout_epll\n");

	if(clk_set_parent(i2s->i2sclk, cm)){
		printk("failed to set mOUTepll as parent of scklkaudio1\n");
		goto lb2;
	}

	cf = clk_get(NULL, "fout_epll");
	if (IS_ERR(cf)) {
		printk("failed to get fout_epll\n");
		goto lb1;
	}

	if(clk_set_parent(cm, cf)){
		printk("failed to set FOUTepll as parent of MOUTepll\n");
		goto lb1;
	}
#if 0	// clk is not generated 
	clk_enable(i2s->i2sclk);
	clk_enable(i2s->iis1_ip_clk);
#endif
	clk_put(cf);
	clk_put(cm);
	/* To avoid switching between sources(LP vs NM mode),
	 * we use EXTPRNT as parent clock of i2sclkd2.
	 */
#if 0
	i2s->i2sclk = clk_get(&pdev->dev, EXTCLK);
	if (IS_ERR(i2s->i2sclk)) {
		printk("failed to get clk(%s)\n", EXTCLK);
		goto lb3;
	}
	printk("Got Audio I2s-Clock -> %s\n", EXTCLK);

	i2s->i2sbusclk = clk_get(NULL, "dout_audio_bus_clk_i2s");//getting AUDIO BUS CLK
	if (IS_ERR(i2s->i2sbusclk)) {
		printk("failed to get audss_hclk\n");
		goto lb2;
	}
	printk("Got audss_hclk..Audio-Bus-clk\n");

	cm = clk_get(NULL, EPLLCLK);
	if (IS_ERR(cm)) {
		printk("failed to get fout_epll\n");
		goto lb2_busclk;
	}

	cn = clk_get(NULL,EXTPRNT);
	if (IS_ERR(cn)) {
		printk("failed to get i2smain_clk\n");
		goto lb1;
	}

	if(clk_set_parent(i2s->i2sclk, cn)){
		printk("failed to set i2s-main clock as parent of audss_hclk\n");
		clk_put(cn);
		goto lb1;
	}

	if(clk_set_parent(cn, cm)){
		printk("failed to set fout-epll as parent of i2s-main clock \n");
		clk_put(cn);
		goto lb1;
	}

	clk_put(cn);
	clk_put(cm);
	clk_put(i2s->i2sbusclk);
#endif	
	ret = s3c_i2s2v2_probe(pdev, dai, i2s, S3C_IIS_PABASE);
	if (ret)
		goto err;

	return 0;
#ifdef USE_CLKAUDIO
lb1:
	clk_put(cm);

	//lb2_busclk:
	//        clk_put(i2s->i2sbusclk);
lb2:
	clk_put(i2s->i2sclk);
#endif
lb3:
	clk_put(i2s->iis1_ip_clk);//sayanta check

	//err_i2sv2: //used in secondary DAI initialization
	/* Not implemented for I2Sv2 core yet */
err:
	return ret;
}

static void s5pc1xx_iis_dev_remove(struct platform_device *pdev,struct snd_soc_dai *dai)
{
	dev_err(&pdev->dev, "Device removal not yet supported\n");
}
static struct snd_soc_dai_ops s5pc1xx_i2s_dai_ops ;
struct snd_soc_dai i2s2_dai = {
	.name = "s3c-i2s2",
	.id = 2,
	.probe = s5pc1xx_iis_dev_probe,
	.remove = s5pc1xx_iis_dev_remove,
	.playback = {
		.channels_min = 2,
		.channels_max = PLBK_CHAN,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &s5pc1xx_i2s_dai_ops,
};
EXPORT_SYMBOL_GPL(i2s2_dai);


static int __init s5pc1xx_i2s2_init(void)
{
	s3c_i2s2v2_register_dai(&i2s2_dai);
	return snd_soc_register_dai(&i2s2_dai);
}
module_init(s5pc1xx_i2s2_init);

static void __exit s5pc1xx_i2s2_exit(void)
{
	snd_soc_unregister_dai(&i2s2_dai);
}
module_exit(s5pc1xx_i2s2_exit);

/* Module information */
MODULE_AUTHOR("Ben Dooks, <ben@simtec.co.uk>");
MODULE_DESCRIPTION("s5pc1xx I2S2 SoC Interface");
MODULE_LICENSE("GPL");
