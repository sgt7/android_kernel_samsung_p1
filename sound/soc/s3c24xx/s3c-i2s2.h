/*
 * s5p-i2s.h  --  ALSA Soc Audio Layer
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef S5P_I2S2_H_
#define S5P_I2S2_H_

#define USE_CLKAUDIO	1 /* use it for LPMP3 mode */

/* Clock dividers */
#define S3C_DIV_MCLK	0
#define S3C_DIV_BCLK	1
#define S3C_DIV_PRESCALER	2

#define S3C_IISCON		(0x00)
#define S3C_IISMOD		(0x04)
#define S3C_IISFIC		(0x08)
#define S3C_IISPSR		(0x0C)
#define S3C_IISTXD		(0x10)
#define S3C_IISRXD		(0x14)


#define S3C_IISCON_I2SACTIVE	(0x1<<0)
#define S3C_IISCON_RXDMACTIVE	(0x1<<1)
#define S3C_IISCON_TXDMACTIVE	(0x1<<2)
#define S3C_IISCON_RXCHPAUSE	(0x1<<3)
#define S3C_IISCON_TXCHPAUSE	(0x1<<4)
#define S3C_IISCON_RXDMAPAUSE	(0x1<<5)
#define S3C_IISCON_TXDMAPAUSE	(0x1<<6)
#define S3C_IISCON_FRXFULL		(0x1<<7)

#define S3C_IISCON_FTX0FULL		(0x1<<8)

#define S3C_IISCON_FRXEMPT		(0x1<<9)
#define S3C_IISCON_FTX0EMPT		(0x1<<10)
#define S3C_IISCON_LRI			(0x1<<11)

#define S3C_IISCON_FTX1FULL		(0x1<<12)
#define S3C_IISCON_FTX2FULL		(0x1<<13)
#define S3C_IISCON_FTX1EMPT		(0x1<<14)
#define S3C_IISCON_FTX2EMPT		(0x1<<15)

#define S3C_IISCON_FTXURINTEN	(0x1<<16)
#define S3C_IISCON_FTXURSTATUS	(0x1<<17)
#define S3C_IISCON_FRXOFINTEN	(0x1<<18)
#define S3C_IISCON_FRXOFSTAT	(0x1<<19)


#define S3C_IISMOD_BFSMASK	(3<<1)
#define S3C_IISMOD_32FS		(0<<1)
#define S3C_IISMOD_48FS		(1<<1)
#define S3C_IISMOD_16FS		(2<<1)
#define S3C_IISMOD_24FS		(3<<1)

#define S3C_IISMOD_RFSMASK		(3<<3)
#define S3C_IISMOD_256FS		(0<<3)
#define S3C_IISMOD_512FS		(1<<3)
#define S3C_IISMOD_384FS		(2<<3)
#define S3C_IISMOD_768FS		(3<<3)

#define S3C_IISMOD_SDFMASK		(3<<5)
#define S3C_IISMOD_IIS			(0<<5)
#define S3C_IISMOD_MSB			(1<<5)
#define S3C_IISMOD_LSB			(2<<5)

#define S3C_IISMOD_LRP			(1<<7)

#define S3C_IISMOD_TXRMASK		(3<<8)
#define S3C_IISMOD_TX			(0<<8)
#define S3C_IISMOD_RX			(1<<8)
#define S3C_IISMOD_TXRX			(2<<8)

#define S3C_IISMOD_IMSMASK		(3<<10)
#define S3C_IISMOD_MSTPCLK		(0<<10)
#define S3C_IISMOD_MSTCLKAUDIO	(1<<10)
#define S3C_IISMOD_SLVPCLK		(2<<10)
#define S3C_IISMOD_SLVI2SCLK	(3<<10)

#define S3C_IISMOD_CDCLKCON		(1<<12)

#define S3C_IISMOD_BLCMASK		(3<<13)
#define S3C_IISMOD_16BIT		(0<<13)
#define S3C_IISMOD_8BIT			(1<<13)
#define S3C_IISMOD_24BIT		(2<<13)


#define S3C_IISFIC_FRXCNTMSK	(0x7f<<0)
#define S3C_IISFIC_RFLUSH		(1<<7)
#define S3C_IISFIC_FTX0CNTMSK	(0x7f<<8)
#define S3C_IISFIC_TFLUSH		(1<<15)

#define S3C_IISPSR_PSVALA		(0x3f<<8)
#define S3C_IISPSR_PSRAEN		(1<<15)


/* clock sources */
#define S3C_CLKSRC_PCLK		S3C_IISMOD_MSTPCLK
#define S3C_CLKSRC_CLKAUDIO	S3C_IISMOD_MSTCLKAUDIO
#define S3C_CLKSRC_SLVPCLK	S3C_IISMOD_SLVPCLK
#define S3C_CLKSRC_I2SEXT	S3C_IISMOD_SLVI2SCLK
#define S3C_CDCLKSRC_INT	(4<<10)
#define S3C_CDCLKSRC_EXT	(5<<10)

//#define IRQ_S3C_IISV32		IRQ_I2S1
#define IRQ_S3C_IISV32		IRQ_I2S2

/* below definitions have moved to "arch/arm/mach-s5pc100/include/mach/map.h"
-#define S3C_PA_IIS_V32         S5PC1XX_PA_IIS_V32
-#define S3C_PA_IIS_V50         S5PC1XX_PA_IIS_V50
*/

#define S3C_DMACH_I2S_OUT	DMACH_I2S2_TX
#define S3C_DMACH_I2S_IN	DMACH_I2S2_RX
#define S3C_IIS_PABASE		S5PV210_PA_IIS2
#define S3C_IISIRQ			IRQ_S3C_IISV32
#define CLK_I2S1			"i2s_v32"//CLK_GATE_IP3 register for I2S1

#ifdef CONFIG_SND_WM8580_MASTER /* ?? */
#define EXTPRNT "iis_cdclk0"
#else
#if defined(CONFIG_SND_SMDKC100_WM8580)
#define EXTPRNT "fout_epll"
#elif defined(CONFIG_SND_SMDKC110_WM8580)
#define EXTPRNT "i2smain_clk"
#elif defined(CONFIG_SND_UNIVERSAL_WM8994)|| defined(CONFIG_SND_UNIVERSAL_WM8994_MODULE)
#define EXTPRNT "mout_audss"
#endif
#endif

#if defined(CONFIG_SND_SMDKC100_WM8580)
#define EXTCLK			"i2sclkd2"
#define EPLLCLK 		EXTPRNT
#else
#define EPLLCLK 		"fout_epll"
#endif

#define PLBK_CHAN		6
#define S3C_DESC		"S3C AP I2S-V5.0 Interface"

/* dma_state */
#define S3C_I2SDMA_START   1
#define S3C_I2SDMA_STOP    2
#define S3C_I2SDMA_FLUSH   3
#define S3C_I2SDMA_SUSPEND 4
#define S3C_I2SDMA_RESUME  5

struct s5p_i2s_pdata {
	int lp_mode;
	u32 *p_rate;
	unsigned  dma_prd;
	void *dma_token;
	dma_addr_t dma_saddr;
	unsigned int dma_size;
	int dma_state;
	void (*dma_cb)(void *dt, int bytes_xfer);
	void (*dma_setcallbk)(void (*dcb)(void *, int), unsigned chunksize);
	void (*dma_getpos)(dma_addr_t *src, dma_addr_t *dst);
	int (*dma_enqueue)(void *id);
	void (*dma_ctrl)(int cmd);
	struct snd_soc_dai i2s_dai;
	void (*set_mode)(int lp_mode);
	spinlock_t lock;
};


#endif /*S5P_I2S2_H_*/
