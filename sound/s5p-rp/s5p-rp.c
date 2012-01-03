/* sound/s5p-rp/s5p-rp.c
 *
 * SRP Audio driver for Samsung s5pc110
 *
 * Copyright (c) 2010 Samsung Electronics
 * http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/irqs.h>

#include "s5p-rp_reg.h"
#include "s5p-rp_fw.h"
#include "s5p-rp_ioctl.h"

#define _USE_AUTO_PAUSE_		/* Pause RP, when two IBUF are empty */
#define _USE_FW_ENDIAN_CONVERT_		/* Endian conversion should be enabled */
#define _USE_RESAMPLER_			/* RP's resampler should be enabled */
#define	_USE_PCM_DUMP_			/* Dump PCM data for Android's buffer
					   snoop fearute */
#define	_PCM_DUMP_IDLE_		2 
#undef	_USE_CRC_CHK_			/* MP3 CRC checking */
#undef	_USE_POSTPROCESS_SKIP_TEST_	/* MP3 decoding test without audio output */

#define _IMEM_MAX_		(64*1024)	/* 64KBytes */
#define _DMEM_MAX_		(96*1024)	/* 96KBytes */
#define _IBUF_SIZE_		(18*1024)	/* 18KBytes */
#define _OBUF_SIZE_		 (9*1024)	/*  9KBytes */
#define _WBUF_SIZE_		(_IBUF_SIZE_ * 2)	/* in DRAM */

#ifdef _USE_FW_ENDIAN_CONVERT_
#define ENDIAN_CHK_CONV(VAL)		\
	(((VAL >> 24) & 0x000000FF) |	\
	 ((VAL >>  8) & 0x0000FF00) |	\
	 ((VAL <<  8) & 0x00FF0000) |	\
	 ((VAL << 24) & 0xFF000000))
#else
#define ENDIAN_CHK_CONV(VAL)	(VAL)
#endif

#define RP_DEV_MINOR		(250)
#define RP_CTRL_DEV_MINOR	(251)

#ifdef CONFIG_SND_S5P_RP_DEBUG
#define s5pdbg(x...) printk(KERN_INFO "S5P_RP: " x)
#else
#define s5pdbg(x...)
#endif

#define EFFECT_DATA_BEGIN		(0x077F0)
#define EFFECT_DATA_END			(0x0DE38)

DECLARE_WAIT_QUEUE_HEAD(WaitQueue_Write);
DECLARE_WAIT_QUEUE_HEAD(WaitQueue_EOS);

struct s5p_rp_info {
	void __iomem  *imem;
	void __iomem  *dmem;
	void __iomem  *ibuf0;
	void __iomem  *ibuf1;
	void __iomem  *obuf0;
	void __iomem  *obuf1;
	void __iomem  *commbox;

	void __iomem  *clkgate;

	int ibuf_next;				/* IBUF index to be filled at next write */
	int ibuf_empty[2];			/* Empty flag of IBUF0/1 */
	unsigned long ibuf_size;		/* IBUF size byte */
	unsigned long obuf_frame_size;		/* OBUF frame size */
	unsigned long frame_size;		/* Decoded one frame size */
	unsigned long frame_count;		/* Decoded frame counter */
	unsigned long frame_count_base;
	unsigned long channel;			/* Mono = 1, Stereo = 2 */

	int block_mode;				/* Block Mode */
	int decoding_started;			/* Decoding started flag */
	int wait_for_eos;			/* Wait for End-Of-Stream */
	int stop_after_eos;			/* State for Stop-after-EOS */
	unsigned long error_info;		/* Error Information */
	unsigned long gain;			/* Gain */
	unsigned long gain_subl;		/* Gain sub left */
	unsigned long gain_subr;		/* Gain sub right */
	int dram_in_use;			/* DRAM is accessed by SRP */

	int early_suspend_entered;		/* Early suspend state */
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int fw_set;				/* Firmware Set index */
	unsigned long *fw_text;			/* Firmware TEXT */
	unsigned long *fw_data;			/* Firmware DATA */
	unsigned long fw_xchg_addr;		/* Firmware TEXT exchanginng addr */
	int alt_fw_loaded;			/* Alt-Firmware State */

	unsigned char *wbuf;			/* WBUF in DRAM */
	unsigned long wbuf_pa;			/* Physical address */
	unsigned long wbuf_pos;			/* Write pointer */
	unsigned long wbuf_fill_size;		/* Total size by user write() */

	unsigned long effect_enabled;		/* Effect enable switch */
	unsigned long effect_def;		/* Effect definition */
	unsigned long effect_eq_user;		/* Effect EQ user */
	unsigned long effect_speaker;		/* Effect Speaker mode */

	unsigned char *pcm_va;			/* PCM dump virtual address */
	unsigned long pcm_pa;			/* PCM dump physical address */

	unsigned long pcm_dump_enabled;		/* PCM dump switch */
	unsigned long pcm_buf_offset;		/* RP internal decoded MP3 buffer, DMEM offset */
	unsigned long pcm_dump_idle;		/* PCM dump count from RP */
};

static struct s5p_rp_info s5p_rp;

static DEFINE_MUTEX(rp_mutex);

#ifdef CONFIG_SND_S5P_RP_DEBUG
struct timeval time_irq, time_write;
#endif

/* s5p_rp_is_running, s5p_rp_is_opened should be accessed from i2s driver only. */
volatile int s5p_rp_is_opened = 0;
volatile int s5p_rp_is_running = 0;
EXPORT_SYMBOL(s5p_rp_is_opened);
EXPORT_SYMBOL(s5p_rp_is_running);

static int s5p_rp_dump_cnt = 0;
static int eos_pause_suspend = 0;
static int s5p_rp_auto_pause = 0;
static int s5p_rp_flushing = 0;

/* These functions are provided by i2s driver. */
extern void s5p_i2s_idma_enable(unsigned long frame_size_bytes);
extern void s5p_i2s_idma_pause(void);
extern void s5p_i2s_idma_continue(void);
extern void s5p_i2s_idma_stop(void);
extern void s5p_i2s_do_resume_for_rp(void);
extern void s5p_i2s_do_suspend_for_rp(void);

#ifdef _LP_ULP_MIXED_AUDIO_
/* These functions are provided by pcm(dma) driver. */
extern void s5p_pcm_change_nr_to_lp(void);
extern void s5p_pcm_change_lp_to_nr(void);

/* These functions are provided by i2s driver. */
extern void s5p_i2s_change_nr_to_lp_pause(void);
extern void s5p_i2s_change_nr_to_lp_continue(void);
extern void s5p_i2s_change_lp_to_nr_pause(void);
extern void s5p_i2s_change_lp_to_nr_continue(void);
/*extern void s5p_i2s_dump_regs(void);*/
#endif

static void s5p_rp_set_effect_apply(void);
static void s5p_rp_set_gain_apply(void);

int s5p_rp_get_op_level(void)
{
	int op_lvl = 0;

	if (s5p_rp_is_running) {
		if (s5p_rp.dram_in_use)
			op_lvl = 1;
		else
			op_lvl = 0;
	} else {
		op_lvl = 1;
	}

	return op_lvl;
}
EXPORT_SYMBOL(s5p_rp_get_op_level);

#ifdef _USE_PCM_DUMP_
static void s5p_rp_set_pcm_dump(int on)
{
	unsigned long sw_def = readl(s5p_rp.commbox + RP_SW_DEF);
	
	if (s5p_rp.pcm_dump_enabled != on) {
		if (on) { 
			sw_def |= 0x20;
		} else {
			sw_def &= ~0x20;
		}
		s5p_rp.pcm_dump_enabled = on;
		s5p_rp.dram_in_use = on;
		writel(sw_def, s5p_rp.commbox + RP_SW_DEF);

		if (!s5p_rp.pcm_dump_enabled)	/* Clear dump buffer */
			memset(s5p_rp.pcm_va, 0, 4096);
	}
}
#endif

static void s5p_rp_commbox_init(void)
{
	s5pdbg("Commbox initialized\n");

	/* RP_MISC should be set first */
	writel(0x0000000B, s5p_rp.commbox + RP_MISC);		/* RP decoding, IBUF write Big endian */
	writel(0x00000000, s5p_rp.commbox + RP_INTERRUPT);	/* Disable interrupt & wake up */

	writel(0x00000000, s5p_rp.commbox + RP_FRAME_INDEX);	/* Clear Decoded Frame No. = 0 */
	writel(0x00000000, s5p_rp.commbox + RP_FRAME_SIZE);	/* Clear FRAME Size = 0 */

	writel(0x00000000, s5p_rp.commbox + RP_EFFECT_DEF);	/* All Effect Off */
	writel(0x00000000, s5p_rp.commbox + RP_EQ_USER_DEF);	/* Clear EQ user */

#ifdef _USE_POSTPROCESS_SKIP_TEST_
	/* Enable ARM postprocessing mode for MP3 decoding test */
	writel(0x00000001, s5p_rp.commbox + RP_SW_DEF);
#endif

	writel(RP_IBUF0_ADDR, s5p_rp.commbox + RP_IN_BUFF0);	/* Input Buffer0 address */
	writel(RP_IBUF1_ADDR, s5p_rp.commbox + RP_IN_BUFF1);	/* Input Buffer1 address */
	writel(0x00001200, s5p_rp.commbox + RP_IN_BUFF_SIZE);	/* Input Buffer Size = 4.5KB (per one buf)*/

	writel(0x00000001, s5p_rp.commbox + RP_BOOT);		/* Instruction in Internal IMEM */

#ifdef _USE_CRC_CHK_
	writel(0x00000020, s5p_rp.commbox + RP_PCM_TYPE);	/* PCM signed 16bits and CRC ON */
#else
	writel(0x00000000, s5p_rp.commbox + RP_PCM_TYPE);	/* PCM signed 16bits and CRC OFF */
#endif
	writel(0x00000000, s5p_rp.commbox + RP_READ_BITSTREAM_SIZE);

	s5p_rp_set_effect_apply();				/* Set Sound Alive */
	s5p_rp_set_gain_apply();
}

static void s5p_rp_commbox_deinit(void)
{
	s5pdbg("Commbox deinitialized\n");

	/* Reset value */
	writel(0x00000000, s5p_rp.commbox + RP_MISC);		/* ARM decoding */
	writel(0x00000001, s5p_rp.commbox + RP_PENDING);	/* RP pending  */
	writel(0x00000000, s5p_rp.commbox + RP_INTERRUPT);	/* Disable interrupt & wake up */
}

static void s5p_rp_fw_download(int data_only)
{
	unsigned long n;
	unsigned long *pval;

#ifdef CONFIG_SND_S5P_RP_DEBUG
	struct timeval begin, end;

	do_gettimeofday(&begin);
#endif

	if (!data_only) {
		for (n = 0, pval = s5p_rp.fw_text; n < _IMEM_MAX_; n += 4, pval++) {
			writel(ENDIAN_CHK_CONV(*pval), s5p_rp.imem + n);
		}
	}

	for (n = 0, pval = s5p_rp.fw_data; n < _DMEM_MAX_; n += 4, pval++) {
		writel(ENDIAN_CHK_CONV(*pval), s5p_rp.dmem + n);
	}

#ifdef CONFIG_SND_S5P_RP_DEBUG
	do_gettimeofday(&end);

	s5pdbg("Firmware Download Time : %lu.%06lu seconds.\n",
		end.tv_sec - begin.tv_sec, end.tv_usec - begin.tv_usec);
#endif
}

static void s5p_rp_fw_effect_download(void)
{
	unsigned long n;
	unsigned long *pval;

#ifdef CONFIG_SND_S5P_RP_DEBUG
	struct timeval begin, end;

	do_gettimeofday(&begin);
#endif

	/* TEXT */
	s5pdbg("Effect TEXT 0x%08lX ~ 0x%08X\n", s5p_rp.fw_xchg_addr, 0xFFFF);
	for (n = s5p_rp.fw_xchg_addr, pval = s5p_rp.fw_text + (s5p_rp.fw_xchg_addr >> 2);
		n < _IMEM_MAX_; n += 4, pval++) {
		writel(ENDIAN_CHK_CONV(*pval), s5p_rp.imem + n);
	}
	/* DATA */
	s5pdbg("Effect DATA 0x%08X ~ 0x%08X\n", EFFECT_DATA_BEGIN, EFFECT_DATA_END);
	for (n = EFFECT_DATA_BEGIN, pval = s5p_rp.fw_data + (EFFECT_DATA_BEGIN >> 2);
		n <= EFFECT_DATA_END; n += 4, pval++) {
		writel(ENDIAN_CHK_CONV(*pval), s5p_rp.dmem + n);
	}

#ifdef CONFIG_SND_S5P_RP_DEBUG
	do_gettimeofday(&end);

	s5pdbg("Effect Code Download Time : %lu.%06lu seconds.\n",
		end.tv_sec - begin.tv_sec, end.tv_usec - begin.tv_usec);
#endif
}

static void s5p_rp_set_default_fw(int data_only)
{
	/* Initialize Commbox & default parameters */
	s5p_rp_commbox_init();

	/* Download default Firmware */
	s5p_rp_fw_download(data_only);
}

#ifdef _LP_ULP_MIXED_AUDIO_
static void s5p_rp_clear_imem(void)
{
	int n;

	for (n = 0; n < _IMEM_MAX_; n += 4) {
		writel(0, s5p_rp.imem + n);	/* Now imem is free for LP Audio */
	}
}
#endif

static void s5p_rp_flush_ibuf(void)
{
	int n;

	for (n = 0; n < s5p_rp.ibuf_size; n += 4) {
		writel(0, s5p_rp.ibuf0 + n);
		writel(0, s5p_rp.ibuf1 + n);
	}

	s5p_rp.ibuf_next = 0;			/* Next IBUF to be filled */
	s5p_rp.ibuf_empty[0] = 1;		/* IBUF0 is empty */
	s5p_rp.ibuf_empty[1] = 1;		/* IBUF1 is empty */
	s5p_rp.wbuf_pos = 0;
	s5p_rp.wbuf_fill_size = 0;
}

static void s5p_rp_flush_obuf(void)
{
	int n;

	for (n = 0; n < _OBUF_SIZE_; n += 4) {
		writel(0, s5p_rp.obuf0 + n);
		writel(0, s5p_rp.obuf1 + n);
	}
}

static void s5p_rp_reset_frame_counter(void)
{
	s5p_rp.frame_count = 0;
	s5p_rp.frame_count_base = 0;
}

static unsigned long s5p_rp_get_frame_counter(void)
{
	unsigned long val;

	val = readl(s5p_rp.commbox + RP_FRAME_INDEX);				/* Decoded Frame No. from RP */
	s5p_rp.frame_count = s5p_rp.frame_count_base + val;			/* Accumulate Count */

	return s5p_rp.frame_count;
}

static void s5p_rp_reset(void)
{
	unsigned long val, sw_def;

	wake_up_interruptible(&WaitQueue_Write);				/* Wake up blocked writes */

	writel(0x00000001, s5p_rp.commbox + RP_PENDING);			/* RP pending  */
	writel(0x00000000, s5p_rp.commbox + RP_FRAME_INDEX);			/* Clear Decoded Frame No. = 0 */
	writel(0x00000000, s5p_rp.commbox + RP_READ_BITSTREAM_SIZE);
	writel(0x7FFFFFFF, s5p_rp.commbox + RP_BITSTREAM_SIZE);			/* Set Total Bitstream Size to Maximum */
	writel(s5p_rp.pcm_pa, s5p_rp.commbox + RP_PCM_ADDR0);

	val = readl(s5p_rp.commbox + RP_SW_DEF);
	val &= ~0x08;
	writel(val, s5p_rp.commbox + RP_SW_DEF);				/* Clear Pause State */

	s5p_rp_set_effect_apply();						/* Set Sound Alive */
	writel(0x00000000, s5p_rp.commbox + RP_RESET);				/* RP Reset  */
	writel(0x00000001, s5p_rp.commbox + RP_INTERRUPT);			/* Interrupt & wake up */

	sw_def = readl(s5p_rp.commbox + RP_SW_DEF);
	if (s5p_rp.pcm_dump_enabled) {
		sw_def |= 0x20;
		s5p_rp.dram_in_use = 1;
 	} else {
		sw_def &= ~0x20;
		s5p_rp.dram_in_use = 0;
	}
	writel(sw_def, s5p_rp.commbox + RP_SW_DEF);

	s5p_rp.error_info = 0;							/* Clear Error Info */
	s5p_rp.frame_count_base = s5p_rp.frame_count;				/* Store Total Count */
	s5p_rp.obuf_frame_size = 0;
	s5p_rp.channel = 0;
	s5p_rp.wait_for_eos = 0;						/* Clear Wait Info */
	s5p_rp.stop_after_eos = 0;
	s5p_rp.decoding_started = 0;						/* Clear Decoding Start Flag */
	if (!s5p_rp_flushing)
		s5p_rp_is_running = 0;

	s5p_rp_auto_pause = 0;
}

static void s5p_rp_pause(void)
{
	unsigned long val;

	s5p_i2s_idma_pause();							/* Pause iDMA */
	val = readl(s5p_rp.commbox + RP_SW_DEF);
	val |= 0x08;
	writel(val, s5p_rp.commbox + RP_SW_DEF);				/* Request Pause  */
}

static void s5p_rp_continue(void)
{
	unsigned long val;

	val = readl(s5p_rp.commbox + RP_SW_DEF);
	val &= ~0x08;
	writel(val, s5p_rp.commbox + RP_SW_DEF);				/* Request Continue */
	writel(0x00000000, s5p_rp.commbox + RP_PENDING);			/* Set RP running */
	s5p_i2s_idma_continue();						/* Continue iDMA */
}

static void s5p_rp_stop(void)
{
	s5p_i2s_idma_stop();							/* iDMA Stop */
	writel(0x00000001, s5p_rp.commbox + RP_PENDING);			/* RP pending  */
	s5p_rp_flush_obuf();							/* Flush OBUF */
}

static int s5p_rp_choose_fw_set(void)
{
	int sound_alive, eq_mode, fw_set_new;
	int ret_val = 0;

	sound_alive = (s5p_rp.effect_def >> 5) & 0x07;
	eq_mode = (s5p_rp.effect_def >> 1) & 0x0F;

	switch (sound_alive) {
	case SA_SUROUND:			/* m-Theater */
		fw_set_new = RP_FW_CODE1;
		break;
	case SA_3D:				/* 3D */
		fw_set_new = RP_FW_CODE20;
		break;
	case SA_REVERB:				/* Reverberation */
		fw_set_new = RP_FW_CODE21;
		break;
	case SA_MC:				/* Music Clarify */
		fw_set_new = RP_FW_CODE22;
		break;
	case SA_BE:				/* Bass Enhancement */
		fw_set_new = RP_FW_CODE30;
		break;
	case SA_SE:				/* Sound Externalization */
		fw_set_new = RP_FW_CODE31;
		break;
	case SA_BYPASS:				/* Sound Alive Off */
	default:
		if (eq_mode)			/* Use 3D set for EQ */
			fw_set_new = RP_FW_CODE20;
		else				/* No change, if EQ is bypass */
			fw_set_new = s5p_rp.fw_set;
		break;
	}

	if( s5p_rp.fw_set != fw_set_new)
		ret_val = 1;
	
	s5p_rp.fw_set = fw_set_new;
	s5p_rp.fw_text = rp_fw_text[s5p_rp.fw_set];
	s5p_rp.fw_data = rp_fw_data[s5p_rp.fw_set];

	s5pdbg("Firmware - %s\n", rp_fw_name[s5p_rp.fw_set]);

	return ret_val;
}

static void s5p_rp_set_effect_apply(void)
{
	unsigned long sw_def = readl(s5p_rp.commbox + RP_SW_DEF);

	writel(s5p_rp.effect_def | s5p_rp.effect_speaker,
		s5p_rp.commbox + RP_EFFECT_DEF);

	writel(s5p_rp.effect_eq_user, s5p_rp.commbox + RP_EQ_USER_DEF);

	sw_def &= ~0x06;
	sw_def |= s5p_rp.effect_enabled ? 0x02 : 0x00;
	writel(sw_def, s5p_rp.commbox + RP_SW_DEF);
}

static void s5p_rp_effect_trigger(void)
{
	unsigned long sw_def = readl(s5p_rp.commbox + RP_SW_DEF);

	if (s5p_rp.effect_enabled)
		sw_def |= 0x06;
	else
		sw_def &= ~0x06;

	writel(s5p_rp.effect_def | s5p_rp.effect_speaker,
		s5p_rp.commbox + RP_EFFECT_DEF);
	writel(s5p_rp.effect_eq_user, s5p_rp.commbox + RP_EQ_USER_DEF);
	writel(sw_def, s5p_rp.commbox + RP_SW_DEF);
}

static void s5p_rp_set_gain_apply(void)
{
	unsigned long gain;

	gain = (((s5p_rp.gain * s5p_rp.gain_subl) / 100) & 0x01FFFC00) >> 10;
	gain |= (((s5p_rp.gain * s5p_rp.gain_subr) / 100) & 0x01FFFC00) << 6;

	s5pdbg("========== SRP GAIN register [%08lX] ============\n", gain);

	writel(gain, s5p_rp.commbox + RP_GAIN_FACTOR);
}

#if 1	/* Test only */
static void s5p_rp_set_effect_mode_test(int mode)
{
	unsigned long effect;

	if (mode >= 0 && mode <= RP_FW_CODE_MAX) {
		effect = (mode << 5) | (1 << 1);	/* eq:rock, effect wkearphone mode wit effect */
		s5p_rp.effect_enabled = effect ? 0x01 : 0x00;
		s5p_rp.effect_def = effect;

		s5p_rp_choose_fw_set();					/* Choose firmware set */
		if (s5p_rp_is_running) {
			s5p_rp_effect_trigger();			/* Apply will be done in ISR */
		} else if (s5p_rp_is_opened) {
			s5p_rp_fw_effect_download();			/* Download immediately */
			s5p_rp_set_effect_apply();			/* Apply immediately */
		}
	}
}
#else
static void s5p_rp_set_effect_mode_test(int mode)
{
	printk("S5P_RP: Do not use '%s', which is only for test.\n", __FUNCTION__);
}
#endif

#ifdef _USE_POSTPROCESS_SKIP_TEST_
struct timeval time_open, time_release;
#endif
static int s5p_rp_open(struct inode *inode, struct file *file)
{
	mutex_lock(&rp_mutex);
	if (s5p_rp_is_opened) {
		s5pdbg("s5p_rp_open() - SRP is already opened.\n");
		mutex_unlock(&rp_mutex);
		return -1;			/* Fail */
	}

#ifdef _USE_POSTPROCESS_SKIP_TEST_
	do_gettimeofday(&time_open);
#endif

	s5p_rp_is_opened = 1;			/* 1 */
	mutex_unlock(&rp_mutex);

	s5p_i2s_do_resume_for_rp();

#ifdef _LP_ULP_MIXED_AUDIO_
	s5pdbg("%s: Change LP to ULP(+Normal) Audio...\n", __FUNCTION__);

	s5p_i2s_change_lp_to_nr_pause();	/* 2 */
	s5p_pcm_change_lp_to_nr();		/* 3 */
	s5p_i2s_change_lp_to_nr_continue();	/* 4 */
	s5p_rp_clear_imem();			/* 5 */
/*	s5p_i2s_dump_regs();*/
#endif

	if (!(file->f_flags & O_NONBLOCK)) {					/* If Block Mode is used */
		s5pdbg("s5p_rp_open() - Block Mode\n");
		s5p_rp.block_mode = 1;
	} else {
		s5pdbg("s5p_rp_open() - NonBlock Mode\n");
		s5p_rp.block_mode = 0;
	}
	s5p_rp.frame_size = 0;							/* Clear Frame-Size Info */

	s5p_rp_reset_frame_counter();
	s5p_rp_set_default_fw(0);

	return 0;
}

static int s5p_rp_release(struct inode *inode, struct file *file)
{
	s5pdbg("s5p_rp_release()\n");

	s5p_rp_commbox_deinit();		/* Reset commbox */

#ifdef _LP_ULP_MIXED_AUDIO_
	s5pdbg("%s: Change ULP(+Normal) to LP Audio...\n", __FUNCTION__);

	s5p_rp_clear_imem();			/* 1 */
	s5p_rp_is_opened = 0;			/* 2 */
	s5p_i2s_change_nr_to_lp_pause();	/* 3 */
	s5p_pcm_change_nr_to_lp();		/* 4 */
	s5p_i2s_change_nr_to_lp_continue();	/* 5 */
/*	s5p_i2s_dump_regs();*/
#else

	if (s5p_rp_is_running) {
		s5p_rp_is_running = 0;
		s5p_i2s_idma_pause();
	}

	s5p_rp_is_opened = 0;

	if (s5p_rp_dump_cnt > 0) {
		s5p_rp_set_pcm_dump(0);
		s5p_rp_dump_cnt = 0;
	}

	s5p_i2s_do_suspend_for_rp();		/* I2S suspend */
#endif

#ifdef _USE_POSTPROCESS_SKIP_TEST_
	do_gettimeofday(&time_release);
	printk("S5P_RP: Usage period : %lu.%06lu seconds.\n",
		time_release.tv_sec - time_open.tv_sec, time_release.tv_usec - time_open.tv_usec);
#endif

	return 0;
}

static ssize_t s5p_rp_read(struct file *file, char * buffer, size_t size, loff_t * pos)
{
	s5pdbg("s5p_rp_read()\n");

	mutex_lock(&rp_mutex);

	mutex_unlock(&rp_mutex);

	return -1;
}

static ssize_t s5p_rp_write(struct file *file, const char *buffer,
				size_t size, loff_t * pos)
{
	unsigned long frame_idx;

	if (!size && (s5p_rp.wbuf_fill_size >= s5p_rp.ibuf_size * 2)) {
		mutex_lock(&rp_mutex);
		if (!s5p_rp_is_running || s5p_rp_auto_pause == 1) {
			s5pdbg("Resume RP after Waiting EOS\n");
			s5p_rp_flush_obuf();
			s5p_rp_continue();			
			s5p_rp_is_running = 1;
			s5p_rp_auto_pause = 0;
		}
		mutex_unlock(&rp_mutex);

		return size;

	} else if (!size) {
		printk(KERN_INFO "size=%d, decode_started=%d, wbuf_pos=%lu\n",
		size, s5p_rp.decoding_started, s5p_rp.wbuf_pos);
	}

	mutex_lock(&rp_mutex);
	if (s5p_rp.decoding_started && (!s5p_rp_is_running || s5p_rp_auto_pause == 1)) {
		s5pdbg("Resume RP\n");
		s5p_rp_flush_obuf();			/* Flush OBUF */
		s5p_rp_continue();
		s5p_rp_is_running = 1;
		s5p_rp_auto_pause = 0;
	}
	mutex_unlock(&rp_mutex);

	if (copy_from_user(&s5p_rp.wbuf[s5p_rp.wbuf_pos], buffer, size)) {
		return -EFAULT;
	}
	s5p_rp.wbuf_pos += size;
	s5p_rp.wbuf_fill_size += size;

	if (!s5p_rp.decoding_started)
		s5p_rp_is_running = 1;

	if (s5p_rp.wbuf_pos < s5p_rp.ibuf_size) {	/* too small */
		frame_idx = readl(s5p_rp.commbox + RP_FRAME_INDEX);
		while(!s5p_rp.early_suspend_entered && s5p_rp_is_running) {
				if (readl(s5p_rp.commbox + RP_READ_BITSTREAM_SIZE)
				+ (s5p_rp.ibuf_size * 2) + 4096
				>= s5p_rp.wbuf_fill_size)
					break;
				if (readl(s5p_rp.commbox + RP_FRAME_INDEX)
					> frame_idx + 2)	/* long sleep? */
					break;
				msleep_interruptible(2);
		}

		return size;
	}

	if (!s5p_rp.ibuf_empty[s5p_rp.ibuf_next]) {	/* IBUF not available */
		if (file->f_flags & O_NONBLOCK) {
			return -1;	/* return Error at NonBlock mode */
		}

		/* Sleep until IBUF empty interrupt */
		s5pdbg("s5p_rp_write() enter to sleep until IBUF empty INT\n");
		interruptible_sleep_on(&WaitQueue_Write);
		s5pdbg("s5p_rp_write() wake up\n");
	}

	mutex_lock(&rp_mutex);
	if (s5p_rp.ibuf_next == 0) {
		memcpy(s5p_rp.ibuf0, s5p_rp.wbuf, s5p_rp.ibuf_size);
		s5pdbg("Fill IBUF0\n");
		s5p_rp.ibuf_empty[0] = 0;
		s5p_rp.ibuf_next = 1;
	} else {
		memcpy(s5p_rp.ibuf1, s5p_rp.wbuf, s5p_rp.ibuf_size);
		s5pdbg("Fill IBUF1\n");
		s5p_rp.ibuf_empty[1] = 0;
		s5p_rp.ibuf_next = 0;
	}

	memcpy(s5p_rp.wbuf, &s5p_rp.wbuf[s5p_rp.ibuf_size], s5p_rp.ibuf_size);
	s5p_rp.wbuf_pos -= s5p_rp.ibuf_size;

	if (!s5p_rp.ibuf_empty[0] && !s5p_rp.ibuf_empty[1]) {
		if (!s5p_rp.decoding_started) {
			s5pdbg("Start RP decoding!!\n");
			writel(0x00000000, s5p_rp.commbox + RP_PENDING);
			s5p_rp_is_running = 1;
		}
	}

#ifdef CONFIG_SND_S5P_RP_DEBUG
	do_gettimeofday(&time_write);
	s5pdbg("IRQ to write-func Time : %lu.%06lu seconds.\n",
		time_write.tv_sec - time_irq.tv_sec,
		time_write.tv_usec - time_irq.tv_usec);
#endif
	mutex_unlock(&rp_mutex);

	if (s5p_rp.error_info) {	/* RP Decoding Error occurred? */
		return -1;
	}

	return size;
}

static void s5p_rp_write_last(void)
{
	s5pdbg("Send remained data, %lu Bytes (Total: %08lX)\n",
		s5p_rp.wbuf_pos ? s5p_rp.wbuf_pos : s5p_rp.ibuf_size,
		s5p_rp.wbuf_fill_size);
	memset(&s5p_rp.wbuf[s5p_rp.wbuf_pos], 0xFF,
		s5p_rp.ibuf_size - s5p_rp.wbuf_pos);
	if (s5p_rp.ibuf_next == 0) {
		memcpy(s5p_rp.ibuf0, s5p_rp.wbuf, s5p_rp.ibuf_size);
		s5pdbg("Fill IBUF0 (final)\n");
		s5p_rp.ibuf_empty[0] = 0;
		s5p_rp.ibuf_next = 1;
	} else {
		memcpy(s5p_rp.ibuf1, s5p_rp.wbuf, s5p_rp.ibuf_size);
		s5pdbg("Fill IBUF1 (final)\n");
		s5p_rp.ibuf_empty[1] = 0;
		s5p_rp.ibuf_next = 0;
	}
	s5p_rp.wbuf_pos = 0;
	writel(s5p_rp.wbuf_fill_size, s5p_rp.commbox + RP_BITSTREAM_SIZE);
}

static int s5p_rp_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
#ifdef CONFIG_SND_S5P_RP_DEBUG
	struct timeval begin, end;
#endif
	unsigned long val;
	int ret_val = 0;

	mutex_lock(&rp_mutex);

	s5pdbg("s5p_rp_ioctl(cmd:: %08X)\n", cmd);

	switch (cmd) {
	case S5P_RP_INIT:							/* Init */
		val = arg;
		if ((val >= 4*1024) && (val <= _IBUF_SIZE_)) {
			s5pdbg("Init, IBUF size [%ld], OBUF size [%d]\n", val, _OBUF_SIZE_);
			s5p_rp.ibuf_size = val;					/* IBUF size */
			writel(s5p_rp.ibuf_size, s5p_rp.commbox + RP_IN_BUFF_SIZE);	/* Input Buffer Size (per IBUF) */
			s5p_rp_flush_ibuf();					/* Flush IBUF */
			s5p_rp_reset();						/* RP Reset */
		} else {
			s5pdbg("Init error, IBUF size [%ld]\n", val);
			ret_val = -1;
		}
		break;

	case S5P_RP_DEINIT:							/* Deinit */
		s5pdbg("Deinit\n");
		writel(0x00000001, s5p_rp.commbox + RP_PENDING);		/* RP pending  */
		s5p_rp_is_running = 0;
		break;

	case S5P_RP_PAUSE:							/* Pause */
		s5pdbg("Pause\n");
		s5p_rp_pause();							/* RP Pause */
		s5p_rp_is_running = 0;						/* To stop i2s */
		break;

	case S5P_RP_STOP:							/* Stop */
		s5pdbg("Stop\n");
		s5p_rp_stop();							/* RP Stop */
		s5p_rp_is_running = 0;						/* To stop i2s */
		break;

	case S5P_RP_FLUSH:							/* Flush */
		s5pdbg("Flush\n");
		s5p_rp_stop();							/* RP Stop */
		//s5p_rp_is_running = 0;	/* To stop i2s */
		s5p_rp_flush_ibuf();						/* Flush IBUF */
		s5p_rp_set_default_fw(1);					/* Reload DATA */
		writel(s5p_rp.ibuf_size, s5p_rp.commbox + RP_IN_BUFF_SIZE);	/* Input Buffer Size (per IBUF) */
		s5p_rp_flushing = 1;
		s5p_rp_reset();							/* RP Reset */
		s5p_rp_flushing = 0;
		break;

	case S5P_RP_SEND_EOS:
		s5pdbg("Send EOS\n");
		if (s5p_rp.wbuf_fill_size  == 0) {
			s5p_rp.stop_after_eos = 1;
		} else if (s5p_rp.wbuf_fill_size < s5p_rp.ibuf_size * 2) {
			s5p_rp_write_last();
			if (s5p_rp.ibuf_empty[s5p_rp.ibuf_next]) {
				s5p_rp_write_last();
			}
			s5pdbg("Start RP decoding!!\n");
			writel(0x00000000, s5p_rp.commbox + RP_PENDING);
			s5p_rp_is_running = 1;
			s5p_rp.wait_for_eos = 1;
		} else if (s5p_rp.ibuf_empty[s5p_rp.ibuf_next]) {
			s5p_rp_write_last();		/* Last data */
			s5p_rp.wait_for_eos = 1;
		} else {
			s5p_rp.wait_for_eos = 1;
		}
		break;

	case S5P_RP_STOP_EOS_STATE:
		val = s5p_rp.stop_after_eos;
		s5pdbg("RP Stop [%s]\n", val == 1 ? "ON" : "OFF");
		ret_val = copy_to_user((unsigned long *)arg,
			&val, sizeof(unsigned long));
		break;

	case S5P_RP_PENDING_STATE:
		val = readl(s5p_rp.commbox + RP_PENDING);			/* RP Pending State*/
		s5pdbg("RP Pending [%s]\n", val == 1 ? "ON" : "OFF");
		ret_val = copy_to_user((unsigned long *)arg, &val, sizeof(unsigned long));
		break;

	case S5P_RP_ERROR_STATE:						/* Get Error State */
		s5pdbg("Error Info [%08lX]\n", s5p_rp.error_info);
		ret_val = copy_to_user((unsigned long *)arg, &s5p_rp.error_info, sizeof(unsigned long));
		s5p_rp.error_info = 0;						/* Clear Error */
		break;

	case S5P_RP_DECODED_FRAME_NO:						/* Get Params - Decoded Frame No. */
		val = s5p_rp_get_frame_counter();
		s5pdbg("Decoded Frame No [%ld]\n", val);
		ret_val = copy_to_user((unsigned long *)arg, &val, sizeof(unsigned long));
		break;

	case S5P_RP_DECODED_ONE_FRAME_SIZE:
		if (s5p_rp.frame_size) {
			s5pdbg("One Frame Size [%lu]\n", s5p_rp.frame_size);
			ret_val = copy_to_user((unsigned long *)arg, &s5p_rp.frame_size, sizeof(unsigned long));
		} else {
			s5pdbg("Frame not decoded yet...\n");
			ret_val = -1;
		}
		break;

	case S5P_RP_DECODED_FRAME_SIZE:			
		if (s5p_rp.frame_size >= 0) {
			val = s5p_rp_get_frame_counter() * s5p_rp.frame_size;
			ret_val = copy_to_user((unsigned long *)arg, 
				&val, sizeof(unsigned long));
		} else {
			s5pdbg("Frame not decoded yet...\n");
		}
		break;

	case S5P_RP_DECODED_PCM_SIZE:
		if (s5p_rp.frame_size) {
			val = s5p_rp_get_frame_counter() * s5p_rp.frame_size;
			val *= s5p_rp.channel << 1;		/* 16bit pcm */
			s5pdbg("Decoded PCM Data [%ld] Bytes\n", val);
		} else {
			val = 0;
			s5pdbg("Frame is not decoded yet...\n");
		}
		ret_val = copy_to_user((unsigned long *)arg,
			&val, sizeof(unsigned long));
		break;

	case S5P_RP_CHANNEL_COUNT:
		if (s5p_rp.channel) {
			s5pdbg("Channel Count [%ld]\n", s5p_rp.channel);
			ret_val = copy_to_user((unsigned long *)arg,
				&s5p_rp.channel, sizeof(unsigned long));
		}
		break;

	default:
		ret_val = -ENOIOCTLCMD;
		break;
	}

	mutex_unlock(&rp_mutex);

	return ret_val;
}

#ifdef CONFIG_SND_S5P_RP_DEBUG
static unsigned long elapsed_usec_old;
#endif
static irqreturn_t s5p_rp_irq(int irqno, void *dev_id)
{
	int wakeup_req = 0;
	int wakeupEOS_req = 0;
	int pendingoff_req = 0;
	unsigned long irq_code = readl(s5p_rp.commbox + RP_INTERRUPT_CODE);
	unsigned long irq_info = readl(s5p_rp.commbox + RP_INFORMATION);
	unsigned long err_info = readl(s5p_rp.commbox + RP_ERROR_CODE);
	unsigned long sw_def;
#ifdef CONFIG_SND_S5P_RP_DEBUG
	unsigned long elapsed_usec;
#endif

	s5pdbg("IRQ Occured, Code [%08lX] , Info [%08lX]\n", irq_code, irq_info);

	irq_code &= RP_INTR_CODE_MASK;
	irq_info &= RP_INTR_INFO_MASK;

	if (irq_code & RP_INTR_CODE_EFFECTADDR) {				/* RP iDMA-Tx start signal */
		s5pdbg("Effect code addr [0x%08lX]\n", irq_info);
		s5p_rp.fw_xchg_addr = irq_info;
		pendingoff_req = 1;
	}

	if (irq_code & RP_INTR_CODE_PLAYERRDONE) {				/* Stream end with Error code */
		s5pdbg("Stream end with Error [%08lX]......\n", err_info);
		s5p_rp.error_info = err_info;
		wakeup_req = 1;
	}

	if (irq_code & RP_INTR_CODE_PLAYSTART) {				/* RP iDMA-Tx start signal */
		s5p_rp.obuf_frame_size = readl(s5p_rp.commbox + RP_FRAME_SIZE);	/* OBUF Frame Size (Word)  */
		s5pdbg("Out Frame Size [%lu]\n", s5p_rp.obuf_frame_size);
		s5p_i2s_idma_enable(s5p_rp.obuf_frame_size << 2);				/* Start iDMA-Tx */
		s5p_rp.decoding_started = 1;					/* Clear Decoding Started */
		pendingoff_req = 1;
	}

	if (irq_code & RP_INTR_CODE_REQUARTCODE) {
		printk("S5P_RP: UART Code received [0x%08lx]\n", irq_info);
		pendingoff_req = 1;
	}

	if (irq_code & RP_INTR_CODE_REQUEST) {					/* Request? */
		switch (irq_info) {
		case RP_INTR_INFO_DATAEMPTY:					/* Input buffer empty */
			s5p_rp.ibuf_empty[(irq_code & 0x40) ? 1 : 0] = 1;	/* Set empty flag */

			if (s5p_rp.decoding_started) {				/* Should be after Decoding */
				if (s5p_rp.ibuf_empty[0] && s5p_rp.ibuf_empty[1]) {	/* All IBUF empty? */
					if (s5p_rp.wait_for_eos) {
						s5pdbg("Stop at EOS (buffer empty)\n");
						s5p_rp_is_running = 0;
						s5p_rp.stop_after_eos = 1;
						writel(0x00000000, s5p_rp.commbox + RP_INTERRUPT_CODE);
						return IRQ_HANDLED;	/* test */
#ifdef _USE_AUTO_PAUSE_
					} else if (s5p_rp_is_running == 1) {
						s5pdbg("Auto-Pause\n");
						s5p_rp_pause();			/* Request Pause  */
						s5p_rp_auto_pause = 1;
						/* Leave running state for i2s */
#endif
					}
				} else {			
					pendingoff_req = 1;
					if (s5p_rp.wait_for_eos && s5p_rp.wbuf_pos)
						s5p_rp_write_last();

				}
			}
#ifdef CONFIG_SND_S5P_RP_DEBUG
			do_gettimeofday(&time_irq);
			elapsed_usec = time_irq.tv_sec * 1000000 + time_irq.tv_usec;
			s5pdbg("IRQ: IBUF empty -------------------------------- Interval [%lu.%06lu]\n",
				(elapsed_usec - elapsed_usec_old) / 1000000, (elapsed_usec - elapsed_usec_old) % 1000000);
			elapsed_usec_old = elapsed_usec;
#endif
			if (s5p_rp.block_mode && !s5p_rp.stop_after_eos)
				wakeup_req = 1;
			break;
		case RP_INTR_INFO_OBUF1SYNC:					/* Do postprocessing to OBUF0 */
		case RP_INTR_INFO_OBUF2SYNC:					/* Do postprocessing to OBUF1 */
			s5pdbg("Request Post-processing for OBUF%d\n", (irq_info == 0x04) ? 1 : 0);
			sw_def = readl(s5p_rp.commbox + RP_SW_DEF);
			if ((sw_def & 0x06) == 0x06) {				/* FW patch needed? */
				s5p_rp_fw_effect_download();
				s5p_rp_set_effect_apply();			/* Set Sound Alive */
				pendingoff_req = 1;
				s5pdbg("Effect changed\n");
			} else {
				/* for MP3 decoding test only */
				s5pdbg("Request Post-processing for OBUF%d\n", 
					(irq_info == 0x04) ? 1 : 0);
				pendingoff_req = 1;
			}
			break;
		case RP_INTR_INFO_I2SPAUSE:					/* I2S Pause with Error code */
			s5pdbg("Request I2S Pause - Error [%08lX]......\n", err_info);
			s5p_i2s_idma_pause();
			s5p_rp.error_info = err_info;
			break;
		case RP_INTR_INFO_I2SCONTINUE:					/* I2S Continue */
			s5pdbg("Request I2S Continue\n");
			s5p_i2s_idma_continue();
			break;
		default:
			break;
		}
	}

	if (irq_code & RP_INTR_CODE_PLAYDONE) {
		s5pdbg("Stop at EOS\n");
		s5p_i2s_idma_pause();
		s5p_rp_is_running = 0;
		s5p_rp.stop_after_eos = 1;
		writel(0x00000000, s5p_rp.commbox + RP_INTERRUPT_CODE);
		return IRQ_HANDLED;
	}

	if (irq_code & RP_INTR_CODE_INFORMATION) {
		s5p_rp.channel = irq_info & 0x03;				/* Mono / Stereo */
		s5p_rp.frame_size = (irq_info & 0x0C) == 0x04 ?
					576 : 1152;				/* Frame siez 576 or 1152 */
		s5pdbg("Channel = %lu, Frame size = %lu\n",
			s5p_rp.channel, s5p_rp.frame_size);
		pendingoff_req = 1;
	}

	writel(0x00000000, s5p_rp.commbox + RP_INTERRUPT_CODE);			/* Clear Interrupt Code */
	writel(0x00000000, s5p_rp.commbox + RP_INFORMATION);			/* Clear Interrupt Info */
	writel(0x00000000, s5p_rp.commbox + RP_INTERRUPT);			/* Clear IRQ */

	if (pendingoff_req)
		writel(0x00000000, s5p_rp.commbox + RP_PENDING);		/* Set RP running  */

	if (wakeup_req)
		wake_up_interruptible(&WaitQueue_Write);			/* Wake up blocked writes */

	if (wakeupEOS_req)
		wake_up_interruptible(&WaitQueue_EOS);				/* Wake up blocked WaitEOS */

	return IRQ_HANDLED;
}

static int s5p_rp_ctrl_open(struct inode *inode, struct file *file)
{
	s5pdbg("s5p_rp_ctrl_open()\n");

	return 0;
}

static int s5p_rp_ctrl_release(struct inode *inode, struct file *file)
{
	s5pdbg("s5p_rp_ctrl_release()\n");

	return 0;
}

static ssize_t s5p_rp_ctrl_read(struct file *file, char * buffer, size_t size, loff_t * pos)
{
	s5pdbg("s5p_rp_ctrl_read()\n");

	return -1;
}

static ssize_t s5p_rp_ctrl_write(struct file *file, const char *buffer, size_t size, loff_t * pos)
{
	s5pdbg("s5p_rp_ctrl_write()\n");

	return -1;
}

static int s5p_rp_ctrl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret_val = 0;
	unsigned long val;
	unsigned long alt_fw_set;
	unsigned long *alt_fw_text;
	unsigned long *alt_fw_data;

	s5pdbg("s5p_rp_ctrl_ioctl(cmd:: %08X)\n", cmd);

	switch (cmd) {
	case S5P_RP_CTRL_SET_GAIN:					/* Set Output Gain */
		s5pdbg("CTRL: Gain\n");
		s5p_rp.gain = arg;
		if (s5p_rp_is_opened)		/* Volume control */
			s5p_rp_set_gain_apply();
		break;

	case S5P_RP_CTRL_SET_GAIN_SUB_LR:
		s5p_rp.gain_subl = arg >> 16;
		if (s5p_rp.gain_subl > 100)
			s5p_rp.gain_subl = 100;

		s5p_rp.gain_subr = arg & 0xFFFF;
		if (s5p_rp.gain_subr > 100)
			s5p_rp.gain_subr = 100;

		s5pdbg("CTRL: Gain sub [L:%03ld, R:%03ld]\n",
			s5p_rp.gain_subl, s5p_rp.gain_subr);
		if (s5p_rp_is_opened)		/* Change gain immediately */
			s5p_rp_set_gain_apply();
		break;

	case S5P_RP_CTRL_SET_EFFECT:					/* Test only */
		s5pdbg("CTRL: Effect Test\n");
		s5p_rp_set_effect_mode_test((int)arg);
		break;

	case S5P_RP_CTRL_GET_PCM_1KFRAME:				/* Get PCM data 1Kframes from OBUF */
		s5pdbg("CTRL: Get PCM 1K Frame\n");
		ret_val = copy_to_user((unsigned long *)arg, s5p_rp.pcm_va, 4096);	/* Only 1k frames */
		break;

#ifdef _USE_PCM_DUMP_
	case S5P_RP_CTRL_PCM_DUMP_OP:
		if (arg == 1 && s5p_rp.early_suspend_entered == 0) {
			s5p_rp_dump_cnt++;
			if (s5p_rp_dump_cnt == 1)
				s5p_rp_set_pcm_dump(arg);
		} else {	
			s5p_rp_dump_cnt--;
			if (s5p_rp_dump_cnt <= 0) {
				s5p_rp_dump_cnt = 0;
				s5p_rp_set_pcm_dump(0);
			}
			
		}
		break;
#endif

	case S5P_RP_CTRL_EFFECT_ENABLE:
		arg &= 0x01;
		s5pdbg("CTRL: Effect switch %s\n", arg ? "ON" : "OFF");
		if(s5p_rp_choose_fw_set() || (s5p_rp.effect_enabled != arg)){
			s5p_rp.effect_enabled = arg;
			if (s5p_rp_is_running) {
				s5p_rp_effect_trigger();			/* Apply will be done in ISR */
			} else if (s5p_rp_is_opened) {
				s5p_rp_fw_effect_download();			/* Download immediately */
				s5p_rp_set_effect_apply();			/* Apply immediately */
			}
		} else
			s5p_rp.effect_enabled = arg;
		
		break;

	case S5P_RP_CTRL_EFFECT_DEF:
		s5pdbg("CTRL: Effect define\n");
		s5p_rp.effect_def = arg & 0xFFFFFFFE;
		if (s5p_rp_is_running ) {
			writel(s5p_rp.effect_def | s5p_rp.effect_speaker,
		                s5p_rp.commbox + RP_EFFECT_DEF);
		} else if (s5p_rp_is_opened) {
			s5p_rp_fw_effect_download();			/* Download immediately */
			s5p_rp_set_effect_apply();			/* Apply immediately */
		}
		break;

	case S5P_RP_CTRL_EFFECT_EQ_USR:
		s5pdbg("CTRL: Effect EQ user\n");
		s5p_rp.effect_eq_user = arg;
		if(s5p_rp_is_running)
			writel(s5p_rp.effect_eq_user, s5p_rp.commbox + RP_EQ_USER_DEF);
		break;

	case S5P_RP_CTRL_EFFECT_SPEAKER:
		s5pdbg("CTRL: Effect Speaker mode %s\n",
			arg & 0x01 ? "ON" : "OFF");
		if ((arg & 0x01) != s5p_rp.effect_speaker) {
			s5p_rp.effect_speaker = arg & 0x01;
			if (s5p_rp_is_running)
				s5p_rp_effect_trigger();
			else if (s5p_rp_is_opened)
				s5p_rp_set_effect_apply();
		}
		break;

	case S5P_RP_CTRL_IS_OPENED:					/* Get RP Open State */
		val = (unsigned long)s5p_rp_is_opened;			/* RP Open State*/
		s5pdbg("CTRL: RP is [%s]\n", val == 1 ? "Opened" : "Not Opened");
		ret_val = copy_to_user((unsigned long *)arg, &val, 
					sizeof(unsigned long));
		break;

	case S5P_RP_CTRL_IS_RUNNING:					/* Get RP Running State */
		val = (unsigned long)s5p_rp_is_running;			/* RP Running State*/
		s5pdbg("CTRL: RP is [%s]\n", val == 1 ? "Running" : "Pending");
		ret_val = copy_to_user((unsigned long *)arg, &val, 
					sizeof(unsigned long));
		break;

	case S5P_RP_CTRL_IS_PCM_DUMP:
		val = (unsigned long)s5p_rp.pcm_dump_enabled;
		ret_val = copy_to_user((unsigned long *)arg, &val, sizeof(unsigned long));

		break;

	/* Alt-Firmware Functions */
	case S5P_RP_CTRL_ALTFW_STATE:
		val = s5p_rp.alt_fw_loaded;				/* Alt-Firmware State */
/*		s5pdbg("CTRL: Alt-Firmware %sLoaded\n", val == 1 ? "" : "Not ");*/
		ret_val = copy_to_user((unsigned long *)arg, &val, 
					sizeof(unsigned long));
		break;

	case S5P_RP_CTRL_ALTFW_LOAD:					/* Alt-Firmware Loading */
		s5p_rp.alt_fw_loaded = 1;
		alt_fw_text = (unsigned long *)arg;
		alt_fw_data = (unsigned long *)(arg + _IMEM_MAX_);
		alt_fw_set = *((unsigned long *)(arg + _IMEM_MAX_ + _DMEM_MAX_));
		s5pdbg("CTRL: Alt-Firmware Loading: %s\n", rp_fw_name[alt_fw_set]);
		ret_val = copy_from_user(rp_fw_text[alt_fw_set], 
					alt_fw_text, _IMEM_MAX_);
		ret_val = copy_from_user(rp_fw_data[alt_fw_set], 
					alt_fw_data, _DMEM_MAX_);
		break;

	default:
		ret_val = -ENOIOCTLCMD;
		break;
	}

	return ret_val;
}

static struct file_operations s5p_rp_fops = {
	.owner		= THIS_MODULE,
	.read		= s5p_rp_read,
	.write		= s5p_rp_write,
	.ioctl		= s5p_rp_ioctl,
	.open		= s5p_rp_open,
	.release	= s5p_rp_release,

};

static struct miscdevice s5p_rp_miscdev = {
	.minor		= RP_DEV_MINOR,
	.name		= "s5p-rp",
	.fops		= &s5p_rp_fops,
};

static struct file_operations s5p_rp_ctrl_fops = {
	.owner		= THIS_MODULE,
	.read		= s5p_rp_ctrl_read,
	.write		= s5p_rp_ctrl_write,
	.ioctl		= s5p_rp_ctrl_ioctl,
	.open		= s5p_rp_ctrl_open,
	.release	= s5p_rp_ctrl_release,

};

static struct miscdevice s5p_rp_ctrl_miscdev = {
	.minor		= RP_CTRL_DEV_MINOR,
	.name		= "s5p-rp_ctrl",
	.fops		= &s5p_rp_ctrl_fops,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
void s5p_rp_early_suspend(struct early_suspend *h)
{
	printk(KERN_INFO "S5P_RP: early suspend\n");
	s5p_rp.early_suspend_entered = 1;
	if (s5p_rp_is_running) {
		if (s5p_rp_dump_cnt > 0)
        	        s5p_rp_set_pcm_dump(0);
	}
}

void s5p_rp_late_resume(struct early_suspend *h)
{
	printk(KERN_INFO "S5P_RP: late resume\n");

	s5p_rp.early_suspend_entered = 0;

	if (s5p_rp_is_running) {
                if (s5p_rp_dump_cnt > 0)
                        s5p_rp_set_pcm_dump(1);
        }
}
#endif

/*
 * The functions for inserting/removing us as a module.
 */

static int __init s5p_rp_probe(struct platform_device *pdev)
{
	int ret;

	s5p_rp.imem    = ioremap(RP_IMEM_ADDR,    _IMEM_MAX_);	/* IMEM */
	s5p_rp.dmem    = ioremap(RP_DMEM_RP_ADDR, _DMEM_MAX_);	/* DMEM */

	s5p_rp.ibuf0   = ioremap(RP_IBUF0_ADDR,   _IBUF_SIZE_);	/* IBUF0 */
	s5p_rp.ibuf1   = ioremap(RP_IBUF1_ADDR,   _IBUF_SIZE_);	/* IBUF1 */

	s5p_rp.obuf0   = ioremap(RP_OBUF0_ADDR,   _OBUF_SIZE_);	/* OBUF0 */
	s5p_rp.obuf1   = ioremap(RP_OBUF1_ADDR,   _OBUF_SIZE_);	/* OBUF1 */
	s5p_rp.commbox = ioremap(RP_COMMBOX_ADDR, 0x01000);	/* COMMBOX */

	s5p_rp.clkgate = ioremap(RP_ASSCLK_GATE_ADDR, 0x04);	/* CLK GATE */

	if (s5p_rp.imem == NULL || s5p_rp.dmem == NULL ||
		s5p_rp.ibuf0 == NULL || s5p_rp.ibuf1 == NULL ||
		s5p_rp.obuf0 == NULL || s5p_rp.obuf1 == NULL ||
		s5p_rp.commbox == NULL || s5p_rp.clkgate == NULL) {
		return -ENXIO;
	}

	s5p_rp.wbuf = dma_alloc_writecombine(0, _WBUF_SIZE_,
			   (dma_addr_t *)&s5p_rp.wbuf_pa, GFP_KERNEL);
	s5p_rp.pcm_va = dma_alloc_writecombine(0, 8192,
			   (dma_addr_t *)&s5p_rp.pcm_pa, GFP_KERNEL);

	ret = request_irq(IRQ_ASS, s5p_rp_irq, 0, "s5p-rp", pdev);
	if (ret < 0) {
		printk(KERN_ERR "S5P_RP: Fail to claim RP(AUDIO_SS) irq , ret = %d\n", ret);

		iounmap(s5p_rp.imem);
		iounmap(s5p_rp.dmem);
		iounmap(s5p_rp.ibuf0);
		iounmap(s5p_rp.ibuf1);
		iounmap(s5p_rp.obuf0);
		iounmap(s5p_rp.obuf1);
		iounmap(s5p_rp.commbox);

		iounmap(s5p_rp.clkgate);

		return -ENODEV;
	}

	s5p_rp.early_suspend_entered = 0;
#ifdef CONFIG_HAS_EARLYSUSPEND
	s5p_rp.early_suspend.suspend = s5p_rp_early_suspend;
	s5p_rp.early_suspend.resume = s5p_rp_late_resume;
	register_early_suspend(&s5p_rp.early_suspend);
#endif
	/* Information for I2S driver */
	s5p_rp_is_opened = 0;
	s5p_rp_is_running = 0;

	/* Set Default Gain to 1.0 */
	s5p_rp.gain = 1<<24;			/* Gain factor = 1.0 * (1<<24) */

	/* Clear address of RP internal PCM buffer */
	s5p_rp.pcm_buf_offset = 0;

	/* Initialize pcm dump feature */
	s5p_rp.pcm_dump_enabled = 0;
	
	s5p_rp.dram_in_use = 0;

	/* Set Sound Alive Off */
	s5p_rp.effect_def = 0;
	s5p_rp.effect_eq_user = 0;
	s5p_rp.effect_speaker = 1;
	s5p_rp.fw_xchg_addr = 0;
	s5p_rp_choose_fw_set();			/* Default firmware set */
	s5p_rp_effect_trigger();

	/* Clear alternate FW Text/Data tables and prepare buffer for FW */
	s5p_rp.alt_fw_loaded = 0;

	ret = misc_register(&s5p_rp_miscdev);
	if (ret) {
		printk(KERN_ERR "S5P_RP: Cannot register miscdev on minor=%d (%d)\n",
			RP_DEV_MINOR, ret);
		goto err;
	}

	ret = misc_register(&s5p_rp_ctrl_miscdev);
	if (ret) {
		printk(KERN_ERR "S5P_RP: Cannot register miscdev on minor=%d (%d)\n",
			RP_CTRL_DEV_MINOR, ret);
		goto err;
	}

	printk(KERN_INFO "S5P_RP: Driver successfully probed\n");

	return 0;

err:
	free_irq(IRQ_ASS, pdev);

	iounmap(s5p_rp.imem);
	iounmap(s5p_rp.dmem);
	iounmap(s5p_rp.ibuf0);
	iounmap(s5p_rp.ibuf1);
	iounmap(s5p_rp.obuf0);
	iounmap(s5p_rp.obuf1);
	iounmap(s5p_rp.commbox);

	iounmap(s5p_rp.clkgate);

	return ret;
}


static int s5p_rp_remove(struct platform_device *pdev)
{
	s5pdbg("s5p_rp_remove() called !\n");

	free_irq(IRQ_ASS, pdev);

	dma_free_writecombine(0, _WBUF_SIZE_, s5p_rp.wbuf, s5p_rp.wbuf_pa);
	iounmap(s5p_rp.imem);
	iounmap(s5p_rp.dmem);
	iounmap(s5p_rp.ibuf0);
	iounmap(s5p_rp.ibuf1);
	iounmap(s5p_rp.obuf0);
	iounmap(s5p_rp.obuf1);
	iounmap(s5p_rp.commbox);

	iounmap(s5p_rp.clkgate);

	return 0;
}

#ifdef CONFIG_PM
static int s5p_rp_suspend(struct platform_device *pdev, pm_message_t state)
{
	s5pdbg("RP Driver suspend\n");
	if (s5p_rp_is_opened) {			/* RP opened? */
		if (s5p_rp.wait_for_eos == 1 && s5p_rp_is_running == 0)
			eos_pause_suspend = 1;
	}
	return 0;
}

static int s5p_rp_resume(struct platform_device *pdev)
{
	s5pdbg("RP Driver resume\n");

	if (s5p_rp_is_opened) {			/* RP opened? */
			 
		s5p_i2s_do_resume_for_rp();	/* I2S resume */
		s5p_rp_set_default_fw(0);

		s5p_rp_flush_ibuf();							/* Flush IBUF */
		s5pdbg("Init, IBUF size [%ld], OBUF size [%d]\n", s5p_rp.ibuf_size, _OBUF_SIZE_);
		writel(s5p_rp.ibuf_size, s5p_rp.commbox + RP_IN_BUFF_SIZE);		/* Input Buffer Size (per IBUF) */
		s5p_rp_reset();							/* RP Reset */

		if (eos_pause_suspend == 1) {
			s5p_rp.stop_after_eos = 1;
			eos_pause_suspend = 0;
		}
	}

	return 0;
}
#else
#define s5p_rp_suspend NULL
#define s5p_rp_resume  NULL
#endif

static struct platform_driver s5p_rp_driver = {
	.probe		= s5p_rp_probe,
	.remove		= s5p_rp_remove,
	.suspend	= s5p_rp_suspend,
	.resume		= s5p_rp_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s5p-rp",
	},
};

static char banner[] __initdata = KERN_INFO "S5PC11X RP driver, (c) 2010 Samsung Electronics\n";

int __init s5p_rp_init(void)
{
	printk(banner);

	return platform_driver_register(&s5p_rp_driver);
}

void __exit s5p_rp_exit(void)
{
	platform_driver_unregister(&s5p_rp_driver);
}

module_init(s5p_rp_init);
module_exit(s5p_rp_exit);

MODULE_AUTHOR("Yeongman Seo <yman.seo@samsung.com>");
MODULE_DESCRIPTION("S5PC11X RP driver");
MODULE_LICENSE("GPL");
