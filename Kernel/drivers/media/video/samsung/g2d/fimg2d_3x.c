/* linux/drivers/media/video/samsung/g2d/fimg2d_3x.c
 *
 * Copyright  2010 Samsung Electronics Co, Ltd. All Rights Reserved.
 *		      http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This file implements sec-g2d driver.
 */

#include <linux/init.h>

#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <asm/div64.h>
#include <linux/tty.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/signal.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/kmod.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/semaphore.h>
#include <linux/regulator/consumer.h>
#include <linux/io.h>

#include <asm/page.h>
#include <asm/irq.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>

#include <mach/hardware.h>
#include <mach/map.h>
#include <mach/power-domain.h>
#include <mach/media.h>
#include <plat/media.h>
#include <plat/cpu.h>
#include "fimg2d_regs.h"
#include "fimg2d_3x.h"

static int               g_g2d_irq_num = NO_IRQ;
static struct resource  *g_g2d_mem;
static void   __iomem   *g_g2d_base;

static wait_queue_head_t g_g2d_waitq;
static int               g_in_use;
static int               g_num_of_g2d_object;

static struct regulator *g_g2d_pd_regulator;
static struct clk       *g_g2d_clk;
static int               g_flag_clk_enable;
static struct timer_list g_g2d_domain_timer;

static DEFINE_MUTEX(g_g2d_rot_mutex);

static u32 g_g2d_reserved_phys_addr;
static u32 g_g2d_reserved_size;

static u32 g_g2d_src_phys_addr;
static u32 g_g2d_src_virt_addr;
static u32 g_g2d_src_size;

static u32 g_g2d_dst_phys_addr;
static u32 g_g2d_dst_virt_addr;
static u32 g_g2d_dst_size;

static u32 sec_g2d_check_fifo_stat_wait(void)
{
	int cnt = 50;
	/*
	 * 1 = The graphics engine finishes the execution of command.
	 * 0 = in the middle of rendering process.
	 */
	while ((!(__raw_readl(g_g2d_base + FIFO_STAT_REG) & 0x1)) &&
	       (cnt > 0)) {
		cnt--;
		msleep_interruptible(2);
	}

	if (cnt <= 0) {
		__raw_writel(1, g_g2d_base + FIFO_STAT_REG);
		return -1;
	}

	return 0;
}

static inline int sec_g2d_color_mode_and_stride(u32 color_space,
						u32 *color_mode,
						u32 *stride)
{
	switch (color_space) {
	/* 565 */
	case G2D_RGB_565:
		*color_mode = G2D_CHL_ORDER_XRGB | G2D_FMT_RGB_565;
		*stride     = 2;
		break;
	/* 8888 */
	case G2D_RGBA_8888:
		*color_mode = G2D_CHL_ORDER_RGBX | G2D_FMT_ARGB_8888;
		*stride     = 4;
		break;
	case G2D_ARGB_8888:
		*color_mode = G2D_CHL_ORDER_XRGB | G2D_FMT_ARGB_8888;
		*stride     = 4;
		break;
	case G2D_BGRA_8888:
		*color_mode = G2D_CHL_ORDER_BGRX | G2D_FMT_ARGB_8888;
		*stride     = 4;
		break;
	case G2D_ABGR_8888:
		*color_mode = G2D_CHL_ORDER_XBGR | G2D_FMT_ARGB_8888;
		*stride     = 4;
		break;
	case G2D_RGBX_8888:
		*color_mode = G2D_CHL_ORDER_RGBX | G2D_FMT_XRGB_8888;
		*stride     = 4;
		break;
	case G2D_XRGB_8888:
		*color_mode = G2D_CHL_ORDER_XRGB | G2D_FMT_XRGB_8888;
		*stride     = 4;
		break;
	case G2D_BGRX_8888:
		*color_mode = G2D_CHL_ORDER_BGRX | G2D_FMT_XRGB_8888;
		*stride     = 4;
		break;
	case G2D_XBGR_8888:
		*color_mode = G2D_CHL_ORDER_XBGR | G2D_FMT_XRGB_8888;
		*stride     = 4;
		break;
	/* 5551 */
	case G2D_RGBA_5551:
		*color_mode = G2D_CHL_ORDER_RGBX | G2D_FMT_ARGB_1555;
		*stride     = 2;
		break;
	case G2D_ARGB_1555:
		*color_mode = G2D_CHL_ORDER_XRGB | G2D_FMT_ARGB_1555;
		*stride     = 2;
		break;
	case G2D_BGRA_5551:
		*color_mode = G2D_CHL_ORDER_BGRX | G2D_FMT_ARGB_1555;
		*stride     = 2;
		break;
	case G2D_ABGR_1555:
		*color_mode = G2D_CHL_ORDER_XBGR | G2D_FMT_ARGB_1555;
		*stride     = 2;
		break;
	case G2D_RGBX_5551:
		*color_mode = G2D_CHL_ORDER_RGBX | G2D_FMT_XRGB_1555;
		*stride     = 2;
		break;
	case G2D_XRGB_1555:
		*color_mode = G2D_CHL_ORDER_XRGB | G2D_FMT_XRGB_1555;
		*stride     = 2;
		break;
	case G2D_BGRX_5551:
		*color_mode = G2D_CHL_ORDER_BGRX | G2D_FMT_XRGB_1555;
		*stride     = 2;
		break;
	case G2D_XBGR_1555:
		*color_mode = G2D_CHL_ORDER_XBGR | G2D_FMT_XRGB_1555;
		*stride     = 2;
		break;
	/* 4444 */
	case G2D_RGBA_4444:
		*color_mode = G2D_CHL_ORDER_RGBX | G2D_FMT_ARGB_4444;
		*stride     = 2;
		break;
	case G2D_ARGB_4444:
		*color_mode = G2D_CHL_ORDER_XRGB | G2D_FMT_ARGB_4444;
		*stride     = 2;
		break;
	case G2D_BGRA_4444:
		*color_mode = G2D_CHL_ORDER_BGRX | G2D_FMT_ARGB_4444;
		*stride     = 2;
		break;
	case G2D_ABGR_4444:
		*color_mode = G2D_CHL_ORDER_XBGR | G2D_FMT_ARGB_4444;
		*stride     = 2;
		break;
	case G2D_RGBX_4444:
		*color_mode = G2D_CHL_ORDER_RGBX | G2D_FMT_XRGB_4444;
		*stride     = 2;
		break;
	case G2D_XRGB_4444:
		*color_mode = G2D_CHL_ORDER_XRGB | G2D_FMT_XRGB_4444;
		*stride     = 2;
		break;
	case G2D_BGRX_4444:
		*color_mode = G2D_CHL_ORDER_BGRX | G2D_FMT_XRGB_4444;
		*stride     = 2;
		break;
	case G2D_XBGR_4444:
		*color_mode = G2D_CHL_ORDER_XBGR | G2D_FMT_XRGB_4444;
		*stride     = 2;
		break;
	/* PACKED_888 */
	case G2D_PACKED_RGB_888:
		*color_mode = G2D_CHL_ORDER_XRGB | G2D_FMT_PACKED_RGB_888;
		*stride     = 3;
		break;
	case G2D_PACKED_BGR_888:
		*color_mode = G2D_CHL_ORDER_XBGR | G2D_FMT_PACKED_RGB_888;
		*stride     = 3;
		break;
	default:
		pr_err("g2d: %s::unmatched color_space(%d)\n",
				__func__, color_space);
		return -1;
	}
	return 0;
}

static void sec_g2d_rot_config(unsigned int rotate_value,
			u32 *rot, u32 *src_dir,
			u32 *dst_dir)
{
	switch (rotate_value) {
	/* rotation = 1, src_y_dir == dst_y_dir, src_x_dir == dst_x_dir */
	case G2D_ROT_90:
		*rot = 1;
		*src_dir = 0;
		*dst_dir = 0;
		break;

	/* rotation = 1, src_y_dir != dst_y_dir, src_x_dir != dst_x_dir */
	case G2D_ROT_270:
		*rot = 1;
		*src_dir = 0;
		*dst_dir = 0x3;
		break;

	/* rotation = 0, src_y_dir != dst_y_dir, src_x_dir != dst_x_dir */
	case G2D_ROT_180:
		*rot = 0;
		*src_dir = 0;
		*dst_dir = 0x3;
		break;

	/* rotation = 0, src_y_dir != dst_y_dir */
	case G2D_ROT_X_FLIP:
		*rot = 0;
		*src_dir = 0;
		*dst_dir = 0x2;
		break;

	/* rotation = 0, src_x_dir != dst_y_dir */
	case G2D_ROT_Y_FLIP:
		*rot = 0;
		*src_dir = 0;
		*dst_dir = 0x1;
		break;

	/* rotation = 0 */
	default:
		*rot = 0;
		*src_dir = 0;
		*dst_dir = 0;
		break;
	}
}

static u32 sec_g2d_set_src_img(struct g2d_rect *src_rect,
			struct g2d_rect *dst_rect,
			struct g2d_flag *flag)
{
	u32 data    = 0;
	u32 data2   = 0;
	u32 blt_cmd = 0;

	/* set  source to one color */
	if (src_rect == NULL) {
		/* select source */
		__raw_writel(G2D_SRC_SELECT_R_USE_FG_COLOR,
					g_g2d_base + SRC_SELECT_REG);

		/* this assume RGBA_8888 byte order */
		switch (dst_rect->color_format) {
		case G2D_RGB_565:
			data  = ((flag->color_val & 0xF8000000) >> 16); /* R */
			data |= ((flag->color_val & 0x00FC0000) >> 13); /* G */
			data |= ((flag->color_val & 0x0000F800) >> 11); /* B */
			break;

		case G2D_ARGB_8888:
		case G2D_XRGB_8888:
			data  = ((flag->color_val & 0xFF000000) >> 8);  /* R */
			data |= ((flag->color_val & 0x00FF0000) >> 8);  /* G */
			data |= ((flag->color_val & 0x0000FF00) >> 8);  /* B */
			data |= ((flag->color_val & 0x000000FF) << 24); /* A */
			break;

		case G2D_BGRA_8888:
		case G2D_BGRX_8888:
			data  = ((flag->color_val & 0xFF000000) >> 16); /* R */
			data |= ((flag->color_val & 0x00FF0000));       /* G */
			data |= ((flag->color_val & 0x0000FF00) << 16); /* B */
			data |= ((flag->color_val & 0x000000FF));       /* A */
			break;

		case G2D_ABGR_8888:
		case G2D_XBGR_8888:
			data  = ((flag->color_val & 0xFF000000) >> 24); /* R */
			data |= ((flag->color_val & 0x00FF0000) >> 8);  /* G */
			data |= ((flag->color_val & 0x0000FF00) << 8);  /* B */
			data |= ((flag->color_val & 0x000000FF) << 24); /* A */
			break;

		case G2D_RGBA_8888:
		case G2D_RGBX_8888:
		default:
			data = flag->color_val;
			break;
	}

	/* foreground color */
	__raw_writel(data, g_g2d_base + FG_COLOR_REG);

	} else {
		/* select source */
		__raw_writel(G2D_SRC_SELECT_R_NORMAL,
					g_g2d_base + SRC_SELECT_REG);

		/* set base address of source image */
		__raw_writel(src_rect->phys_addr,
					g_g2d_base + SRC_BASE_ADDR_REG);

		sec_g2d_color_mode_and_stride(src_rect->color_format,
					&data,
					&data2);

		/* set color mode */
		__raw_writel(data, g_g2d_base + SRC_COLOR_MODE_REG);

		/* set stride */
		__raw_writel(src_rect->full_w * data2,
					g_g2d_base + SRC_STRIDE_REG);

		/* set coordinate of source image */
		data = (src_rect->y << 16) | (src_rect->x);
		__raw_writel(data, g_g2d_base + SRC_LEFT_TOP_REG);

		data =  ((src_rect->y + src_rect->h) << 16) |
				(src_rect->x + src_rect->w);
		__raw_writel(data, g_g2d_base + SRC_RIGHT_BOTTOM_REG);

	}

	return blt_cmd;
}

static u32 sec_g2d_set_dst_img(struct g2d_rect *rect)
{
	u32 data    = 0;
	u32 data2   = 0;
	u32 blt_cmd = 0;

	/* select destination */
	__raw_writel(G2D_DST_SELECT_R_NORMAL,
					g_g2d_base + DST_SELECT_REG);

	/* set base address of destination image */
	__raw_writel(rect->phys_addr,
					g_g2d_base + DST_BASE_ADDR_REG);

	sec_g2d_color_mode_and_stride(rect->color_format,
				&data,
				&data2);

	/* set color mode */
	__raw_writel(data, g_g2d_base + DST_COLOR_MODE_REG);

	/* set stride */
	__raw_writel(rect->full_w * data2, g_g2d_base + DST_STRIDE_REG);

	/* set coordinate of destination image */
	data = (rect->y << 16) | (rect->x);
	__raw_writel(data, g_g2d_base + DST_LEFT_TOP_REG);

	data =  ((rect->y + rect->h) << 16) | (rect->x + rect->w);
	__raw_writel(data, g_g2d_base + DST_RIGHT_BOTTOM_REG);

	return blt_cmd;
}

static u32 sec_g2d_set_rotation(struct g2d_flag *flag)
{
	u32 blt_cmd = 0;
	u32 rot = 0, src_dir = 0, dst_dir = 0;

	sec_g2d_rot_config(flag->rotate_val, &rot, &src_dir, &dst_dir);

	__raw_writel(rot,     g_g2d_base + ROTATE_REG);
	__raw_writel(src_dir, g_g2d_base + SRC_MSK_DIRECT_REG);
	__raw_writel(dst_dir, g_g2d_base + DST_PAT_DIRECT_REG);

	return blt_cmd;
}

static u32 sec_g2d_set_clip_win(struct g2d_rect *rect)
{
	u32 blt_cmd = 0;

	if (rect->w != 0 && rect->h != 0) {
		blt_cmd |= G2D_BLT_CMD_R_CW_ENABLE;
		__raw_writel((rect->y << 16) | (rect->x),
					g_g2d_base + CW_LEFT_TOP_REG);
		__raw_writel(((rect->y + rect->h) << 16) | (rect->x + rect->w),
					g_g2d_base + CW_RIGHT_BOTTOM_REG);
	}

	return blt_cmd;
}

static u32 sec_g2d_set_color_key(struct g2d_flag *flag)
{
	u32 blt_cmd = 0;

	/* Transparent Selection */
	switch (flag->blue_screen_mode) {
	case G2D_BLUE_SCREEN_TRANSPARENT:
		__raw_writel(flag->color_key_val, g_g2d_base + BS_COLOR_REG);

		blt_cmd |= G2D_BLT_CMD_R_TRANSPARENT_MODE_TRANS;
		break;
	case G2D_BLUE_SCREEN_WITH_COLOR:
		__raw_writel(flag->color_key_val, g_g2d_base + BS_COLOR_REG);
		__raw_writel(flag->color_val,     g_g2d_base + BG_COLOR_REG);

		blt_cmd |= G2D_BLT_CMD_R_TRANSPARENT_MODE_BLUESCR;
		break;
	case G2D_BLUE_SCREEN_NONE:
	default:
		blt_cmd |= G2D_BLT_CMD_R_TRANSPARENT_MODE_OPAQUE;
		break;
	}

	blt_cmd |= G2D_BLT_CMD_R_COLOR_KEY_DISABLE;

	return blt_cmd;
}

static u32 sec_g2d_set_pattern(struct g2d_rect *rect, struct g2d_flag *flag)
{
	u32 data    = 0;
	u32 data2   = 0;
	u32 blt_cmd = 0;

	if (rect == NULL)
		data = G2D_THIRD_OP_REG_NONE;
	else {
		/*  Third Operand Selection */
		switch (flag->third_op_mode) {
		case G2D_THIRD_OP_PATTERN:
			/* set base address of pattern image */
			__raw_writel(rect->phys_addr,
					g_g2d_base + PAT_BASE_ADDR_REG);

			/* set size of pattern image */
			data =   ((rect->y + rect->h) << 16)
			       |  (rect->x + rect->w);
			__raw_writel(data, g_g2d_base + PAT_SIZE_REG);

			sec_g2d_color_mode_and_stride(rect->color_format,
						&data,
						&data2);

			/* set color mode */
			__raw_writel(data, g_g2d_base + PAT_COLOR_MODE_REG);

			/* set stride */
			__raw_writel(rect->full_w * data2,
						g_g2d_base + PAT_STRIDE_REG);

			data =   (rect->y << 16) | rect->x;
			__raw_writel(data, g_g2d_base + PAT_OFFSET_REG);

			data = G2D_THIRD_OP_REG_PATTERN;
			break;
		case G2D_THIRD_OP_FG:
			data = G2D_THIRD_OP_REG_FG_COLOR;
			break;
		case G2D_THIRD_OP_BG:
			data = G2D_THIRD_OP_REG_BG_COLOR;
			break;
		case G2D_THIRD_OP_NONE:
		default:
			data = G2D_THIRD_OP_REG_NONE;
			break;
		}
	}
	__raw_writel(data, g_g2d_base + THIRD_OPERAND_REG);

	if (flag->third_op_mode == G2D_THIRD_OP_NONE) {
		data = ((G2D_ROP_REG_SRC << 8) | G2D_ROP_REG_SRC);
	} else {
		switch (flag->rop_mode) {
		case G2D_ROP_DST:
			data = ((G2D_ROP_REG_DST << 8) |
					G2D_ROP_REG_DST);
			break;
		case G2D_ROP_SRC_AND_DST:
			data = ((G2D_ROP_REG_SRC_AND_DST << 8) |
					G2D_ROP_REG_SRC_AND_DST);
			break;
		case G2D_ROP_SRC_OR_DST:
			data = ((G2D_ROP_REG_SRC_OR_DST << 8) |
					G2D_ROP_REG_SRC_OR_DST);
			break;
		case G2D_ROP_3RD_OPRND:
			data = ((G2D_ROP_REG_3RD_OPRND << 8) |
					G2D_ROP_REG_3RD_OPRND);
			break;
		case G2D_ROP_SRC_AND_3RD_OPRND:
			data = ((G2D_ROP_REG_SRC_AND_3RD_OPRND << 8) |
					G2D_ROP_REG_SRC_AND_3RD_OPRND);
			break;
		case G2D_ROP_SRC_OR_3RD_OPRND:
			data = ((G2D_ROP_REG_SRC_OR_3RD_OPRND << 8) |
					G2D_ROP_REG_SRC_OR_3RD_OPRND);
			break;
		case G2D_ROP_SRC_XOR_3RD_OPRND:
			data = ((G2D_ROP_REG_SRC_XOR_3RD_OPRND << 8) |
					G2D_ROP_REG_SRC_XOR_3RD_OPRND);
			break;
		case G2D_ROP_DST_OR_3RD:
			data = ((G2D_ROP_REG_DST_OR_3RD_OPRND << 8) |
					G2D_ROP_REG_DST_OR_3RD_OPRND);
			break;
		case G2D_ROP_SRC:
		default:
			data = ((G2D_ROP_REG_SRC << 8) |
					G2D_ROP_REG_SRC);
			break;
		}
	}
	__raw_writel(data, g_g2d_base + ROP4_REG);

	/* Mask Operation */
	if (rect && flag->mask_mode == TRUE) {
		sec_g2d_color_mode_and_stride(rect->color_format,
					&data,
					&data2);

		__raw_writel(rect->phys_addr,
					g_g2d_base + MASK_BASE_ADDR_REG);
		__raw_writel(rect->full_w * data2,
					g_g2d_base + MASK_STRIDE_REG);

		blt_cmd |= G2D_BLT_CMD_R_MASK_ENABLE;
	}

	return blt_cmd;
}

static u32 sec_g2d_set_alpha(struct g2d_rect *src_rect,
			struct g2d_rect *dst_rect,
			struct g2d_flag *flag)
{
	u32 blt_cmd = 0;

	/* Alpha Value */
	if (flag->alpha_val <= G2D_ALPHA_VALUE_MAX) /* < 255 */ {
		blt_cmd |= G2D_BLT_CMD_R_ALPHA_BLEND_ALPHA_BLEND;
		__raw_writel((flag->alpha_val & 0xff), g_g2d_base + ALPHA_REG);
	} else /*if (alphaValue == G2D_ALPHA_BLENDING_OPAQUE) */ {
		blt_cmd |= G2D_BLT_CMD_R_ALPHA_BLEND_NONE;
	}

	return blt_cmd;
}

static void sec_g2d_set_bitblt_cmd(struct g2d_rect *src_rect,
				struct g2d_rect *dst_rect,
				u32 blt_cmd)
{
	if (src_rect) {
		if ((src_rect->w != dst_rect->w) ||
			(src_rect->h != dst_rect->h)) {
			blt_cmd |= G2D_BLT_CMD_R_STRETCH_ENABLE;
		}
	}

	__raw_writel(blt_cmd, g_g2d_base + BITBLT_COMMAND_REG);
}

static void sec_g2d_init_regs(struct g2d_params *params)
{
	u32 blt_cmd = 0;

	struct g2d_rect *src_rect = params->src_rect;
	struct g2d_rect *dst_rect = params->dst_rect;
	struct g2d_flag *flag     = params->flag;

	/* source image */
	blt_cmd |= sec_g2d_set_src_img(src_rect, dst_rect, flag);

	/* destination image */
	blt_cmd |= sec_g2d_set_dst_img(dst_rect);

	/* rotation */
	blt_cmd |= sec_g2d_set_rotation(flag);

	/* color key */
	blt_cmd |= sec_g2d_set_color_key(flag);

	/* pattern */
	blt_cmd |= sec_g2d_set_pattern(src_rect, flag);

	/* rop & alpha blending */
	blt_cmd |= sec_g2d_set_alpha(src_rect, dst_rect, flag);

	/* command */
	sec_g2d_set_bitblt_cmd(src_rect, dst_rect, blt_cmd);
}

static void sec_g2d_rotate_with_bitblt(struct g2d_params *params)
{
	/* enable interrupt */
	__raw_writel(G2D_INTEN_R_CF_ENABLE, g_g2d_base + INTEN_REG);

	__raw_writel(0x7, g_g2d_base + CACHECTL_REG);

	__raw_writel(G2D_BITBLT_R_START, g_g2d_base + BITBLT_START_REG);
}

static irqreturn_t sec_g2d_irq(int irq, void *dev_id)
{
	__raw_writel(G2D_INTC_PEND_R_INTP_CMD_FIN, g_g2d_base + INTC_PEND_REG);

	g_in_use = 0;

	wake_up_interruptible(&g_g2d_waitq);

	return IRQ_HANDLED;
}

static void sec_g2d_clk_enable(void)
{
	if (g_flag_clk_enable == 0) {
		regulator_enable(g_g2d_pd_regulator);
		clk_enable(g_g2d_clk);
		g_flag_clk_enable = 1;
	}
}

static void sec_g2d_clk_disable(void)
{
	if (g_flag_clk_enable == 1) {
		if (g_in_use == 0) {
			clk_disable(g_g2d_clk);
			regulator_disable(g_g2d_pd_regulator);
			g_flag_clk_enable = 0;
		} else
			mod_timer(&g_g2d_domain_timer, jiffies + HZ);
	}
}

static void sec_g2d_domain_timer(unsigned long arg)
{
	sec_g2d_clk_disable();
}

static int sec_g2d_open(struct inode *inode, struct file *file)
{
	g_num_of_g2d_object++;

	pr_debug("g2d: open ok!\n");

	return 0;
}

static int sec_g2d_release(struct inode *inode, struct file *file)
{
	g_num_of_g2d_object--;

	if (g_num_of_g2d_object == 0)
		g_in_use = 0;

	pr_debug("g2d: release ok!\n");

	return 0;
}

static int sec_g2d_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long pageFrameNo = 0;
	unsigned long size = vma->vm_end - vma->vm_start;

	if (g_g2d_reserved_size < size) {
		pr_err("g2d: the size (%ld) mapping is too big!\n", size);
		return -EINVAL;
	}

	pageFrameNo = __phys_to_pfn(g_g2d_src_phys_addr);

	if (remap_pfn_range(vma, vma->vm_start, pageFrameNo,
				g_g2d_reserved_size, vma->vm_page_prot)) {
		pr_err("g2d:remap_pfn_range fail\n");
		return -EINVAL;
	}

	return 0;
}

static int sec_g2d_ioctl(struct inode *inode,
			struct file *file,
			unsigned int cmd,
			unsigned long arg)
{
	int                  ret    = 0;
	struct g2d_params   *params = NULL;
	struct g2d_dma_info  dma_info;
	void                *vaddr;

	switch (cmd) {
	case G2D_GET_MEMORY:
		ret = copy_to_user((unsigned int *)arg, &g_g2d_reserved_phys_addr,
						sizeof(unsigned int));
		break;
	case G2D_GET_MEMORY_SIZE:
		ret = copy_to_user((unsigned int *)arg, &g_g2d_reserved_size,
						sizeof(unsigned int));
		break;
	case G2D_DMA_CACHE_INVAL:
		ret = copy_from_user(&dma_info, (struct g2d_dma_info *)arg,
						sizeof(dma_info));
		vaddr = phys_to_virt(dma_info.addr);
		dmac_map_area(vaddr, (unsigned long)vaddr + dma_info.size,
					DMA_FROM_DEVICE);
		break;
	case G2D_DMA_CACHE_CLEAN:
		ret = copy_from_user(&dma_info, (struct g2d_dma_info *)arg,
						sizeof(dma_info));
		vaddr = phys_to_virt(dma_info.addr);
		dmac_map_area(vaddr, (unsigned long)vaddr + dma_info.size,
						DMA_TO_DEVICE);
		break;
	case G2D_DMA_CACHE_FLUSH:
		ret = copy_from_user(&dma_info, (struct g2d_dma_info *)arg,
						sizeof(dma_info));
		vaddr = phys_to_virt(dma_info.addr);
		dmac_flush_range(vaddr, vaddr + dma_info.size);
		break;
	case G2D_SET_MEMORY:
		ret = copy_from_user(&dma_info, (struct g2d_dma_info *)arg,
						sizeof(dma_info));
		vaddr = phys_to_virt(dma_info.addr);
		memset(vaddr, 0x00000000, dma_info.size);
		break;
	default:
		ret = -1;
		break;
	}

	if (ret == 0)
		return 0;

	mutex_lock(&g_g2d_rot_mutex);

	sec_g2d_clk_enable();

	if (file->f_flags & O_NONBLOCK) {
		if (g_in_use == 1) {
			if (wait_event_interruptible_timeout(g_g2d_waitq,
					(g_in_use == 0),
					msecs_to_jiffies(G2D_TIMEOUT)) == 0) {
				__raw_writel(G2D_SWRESET_R_RESET, g_g2d_base + SOFT_RESET_REG);
				pr_err("g2d:%s: waiting for interrupt is timeout\n", __func__);
			}
		}
	}

	g_in_use = 1;

	params = (struct g2d_params *)arg;

	if (cmd == G2D_BLIT) {
		/* initialize */
		sec_g2d_init_regs(params);

		/* bitblit */
		sec_g2d_rotate_with_bitblt(params);
	} else {
		pr_err("g2d: unmatched command (%d)\n", cmd);
		goto sec_g2d_ioctl_done;
	}

	if (!(file->f_flags & O_NONBLOCK)) {
		if (g_in_use == 1) {
			if (wait_event_interruptible_timeout(g_g2d_waitq,
					(g_in_use == 0),
					msecs_to_jiffies(G2D_TIMEOUT)) == 0) {
				__raw_writel(G2D_SWRESET_R_RESET, g_g2d_base + SOFT_RESET_REG);
				pr_err("g2d:%s:	waiting for interrupt is timeout\n", __func__);
			}

			g_in_use = 0;
		}
	}

	ret = 0;

sec_g2d_ioctl_done:

	sec_g2d_clk_disable();

	mutex_unlock(&g_g2d_rot_mutex);

	if (ret != 0) {
		if (params->src_rect) {
			pr_err("src : %d, %d, %d, %d / %d, %d / %d / 0x%x)\n",
				params->src_rect->x,
				params->src_rect->y,
				params->src_rect->w,
				params->src_rect->h,
				params->src_rect->full_w,
				params->src_rect->full_h,
				params->src_rect->color_format,
				params->src_rect->phys_addr);
		}
		if (params->dst_rect) {
			pr_err("dst : %d, %d, %d, %d / %d, %d / %d / 0x%x)\n",
				params->dst_rect->x,
				params->dst_rect->y,
				params->dst_rect->w,
				params->dst_rect->h,
				params->dst_rect->full_w,
				params->dst_rect->full_h,
				params->dst_rect->color_format,
				params->dst_rect->phys_addr);
		}

		if (params->flag) {
			pr_err("alpha_value : %d\n",
			params->flag->alpha_val);
		}
	}

	return ret;
}

static u32 sec_g2d_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	if (g_in_use == 0) {
		mask = POLLOUT | POLLWRNORM;
	} else {
		poll_wait(file, &g_g2d_waitq, wait);

		if (g_in_use == 0)
			mask = POLLOUT | POLLWRNORM;
	}

	return mask;
}

static const struct file_operations sec_g2d_fops = {
	.owner		= THIS_MODULE,
	.open		= sec_g2d_open,
	.release	= sec_g2d_release,
	.mmap		= sec_g2d_mmap,
	.ioctl		= sec_g2d_ioctl,
	.poll		= sec_g2d_poll,
};

static struct miscdevice sec_g2d_dev = {
	.minor		= G2D_MINOR,
	.name		= "sec-g2d",
	.fops		= &sec_g2d_fops,
};

static int sec_g2d_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;

	pr_debug("g2d: start probe : name=%s num=%d res[0].start=0x%x res[1].start=0x%x\n",
			pdev->name, pdev->num_resources,
			pdev->resource[0].start, pdev->resource[1].start);

	/* get the memory region */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		pr_err("g2d: failed to get memory region resouce\n");
		ret = -ENOENT;
		goto probe_out;
	}

	g_g2d_mem = request_mem_region(res->start,
					res->end - res->start + 1,
					pdev->name);
	if (g_g2d_mem == NULL) {
		pr_err("g2d: failed to reserve memory region\n");
		ret = -ENOENT;
		goto err_mem_req;
	}

	/* ioremap */
	g_g2d_base = ioremap(g_g2d_mem->start, g_g2d_mem->end - res->start + 1);
	if (g_g2d_base == NULL) {
		pr_err("g2d: failed ioremap\n");
		ret = -ENOENT;
		goto err_mem_map;
	}

	/* irq */
	g_g2d_irq_num = platform_get_irq(pdev, 0);
	if (g_g2d_irq_num <= 0) {
		pr_err("g2d: failed to get irq resouce\n");
		ret = -ENOENT;
		goto err_irq_req;
	}

	ret = request_irq(g_g2d_irq_num, sec_g2d_irq,
				IRQF_DISABLED, pdev->name, NULL);
	if (ret) {
		pr_err("g2d: request_irq(g2d) failed.\n");
		goto err_irq_req;
	}

	/* Get g2d power domain regulator */
	g_g2d_pd_regulator = regulator_get(&pdev->dev, "pd");
	if (IS_ERR(g_g2d_pd_regulator)) {
		printk(KERN_ERR "g2d: failed to get resource %s\n", "g2d");
		ret = -ENOENT;
		goto err_regulator_get;
	}

	g_g2d_clk = clk_get(&pdev->dev, "g2d");
	if (g_g2d_clk == NULL) {
		pr_err("g2d: failed to find g2d clock source\n");
		ret = -ENOENT;
		goto err_clk_get;
	}

	/* blocking I/O */
	init_waitqueue_head(&g_g2d_waitq);

	/* atomic init */
	g_in_use = 0;

	/* misc register */
	ret = misc_register(&sec_g2d_dev);
	if (ret) {
		pr_err("g2d: cannot register miscdev on minor=%d (%d)\n",
			G2D_MINOR, ret);
		goto err_misc_reg;
	}

	g_g2d_reserved_phys_addr = s5p_get_media_memory_bank(S5P_MDEV_G2D, 0);
	if (g_g2d_reserved_phys_addr == 0) {
		pr_err("g2d: failed to s3c_get_media_memory_bank !!!\n");
		ret = -ENOENT;
		goto err_req_fw;
	}

	g_g2d_reserved_size = s5p_get_media_memsize_bank(S5P_MDEV_G2D, 0);

	g_g2d_src_phys_addr = g_g2d_reserved_phys_addr;
	g_g2d_src_virt_addr = (u32)phys_to_virt(g_g2d_src_phys_addr);
	g_g2d_src_size      = PAGE_ALIGN(g_g2d_reserved_size >> 1);

	g_g2d_dst_phys_addr = g_g2d_src_phys_addr + g_g2d_src_size;
	g_g2d_dst_virt_addr = g_g2d_src_virt_addr + g_g2d_src_size;
	g_g2d_dst_size      = PAGE_ALIGN(g_g2d_reserved_size - g_g2d_src_size);

	/* init domain timer */
	init_timer(&g_g2d_domain_timer);
	g_g2d_domain_timer.function = sec_g2d_domain_timer;

	pr_debug("g2d: sec_g2d_probe ok!\n");

	return 0;

err_req_fw:
	misc_deregister(&sec_g2d_dev);
err_misc_reg:
	clk_put(g_g2d_clk);
	g_g2d_clk = NULL;
err_clk_get:
	regulator_put(g_g2d_pd_regulator);
err_regulator_get:
	free_irq(g_g2d_irq_num, NULL);
err_irq_req:
	iounmap(g_g2d_base);
err_mem_map:
	release_resource(g_g2d_mem);
	kfree(g_g2d_mem);
err_mem_req:
probe_out:
	pr_err("g2d: sec_g2d_probe fail!\n");
	return ret;
}

static int sec_g2d_remove(struct platform_device *dev)
{
	pr_debug("g2d: sec_g2d_remove called !\n");

	del_timer(&g_g2d_domain_timer);

	iounmap(g_g2d_base);

	if (g_g2d_mem != NULL) {
		release_resource(g_g2d_mem);
		kfree(g_g2d_mem);
		g_g2d_mem = NULL;
	}

	free_irq(g_g2d_irq_num, NULL);

	clk_put(g_g2d_clk);
	regulator_put(g_g2d_pd_regulator);

	misc_deregister(&sec_g2d_dev);

	pr_debug("g2d: sec_g2d_remove ok!\n");

	return 0;
}

static struct platform_driver sec_g2d_driver = {
	.probe      = sec_g2d_probe,
	.remove     = sec_g2d_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-g2d",
	},
};

static int __init sec_g2d_init(void)
{
	if (platform_driver_register(&sec_g2d_driver) != 0) {
		pr_err("g2d: platform device register Failed\n");
		return -1;
	}

	pr_debug("g2d: init ok!\n");

	return 0;
}

void sec_g2d_exit(void)
{
	platform_driver_unregister(&sec_g2d_driver);

	pr_debug("g2d: exit ok!\n");
}

module_init(sec_g2d_init);
module_exit(sec_g2d_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("SEC G2D Device Driver");
MODULE_LICENSE("GPL");
