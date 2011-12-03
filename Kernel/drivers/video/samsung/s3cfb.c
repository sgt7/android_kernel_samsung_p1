/* linux/drivers/video/samsung/s3cfb.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 *
 * Core file for Samsung Display Controller (FIMD) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/cpufreq.h>
#include <plat/clock.h>
#include <plat/cpu-freq.h>
#include <plat/media.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif
#if defined(CONFIG_MACH_P1) || defined(CONFIG_MACH_HERRING)
#include <linux/delay.h>
#include <mach/gpio.h>
#include <mach/gpio-p1.h>
#include <mach/regs-clock.h>
#endif
#include "s3cfb.h"
#ifdef CONFIG_FB_S3C_MDNIE
#include "s3cfb_mdnie.h"
#endif
#ifdef CONFIG_FB_S3C_CMC623
#include "tune_cmc623.h"
#endif

#if defined(CONFIG_MACH_S5PC110_P1) && defined(CONFIG_TARGET_LOCALE_KOR)
#define BOOTLOADER_INIT_LCD
#define DISPLAY_BOOT_PROGRESS
#define __COPY_LOGO_FROM_BOOTLOADER__

static int show_progress = 1;
#endif

#if defined(CONFIG_MACH_P1) || defined(CONFIG_MACH_HERRING)
extern unsigned int HWREV;
#endif

#if defined(CONFIG_MACH_S5PC110_P1) && defined(CONFIG_TARGET_LOCALE_KOR)
static int current_band_for_pclk = 0;
#endif
/*
 *  Mark for GetLog (tkhwang)
 */

struct struct_frame_buf_mark {
	u32 special_mark_1;
	u32 special_mark_2;
	u32 special_mark_3;
	u32 special_mark_4;
	void *p_fb;
	u32 resX;
	u32 resY;
	u32 bpp;    //color depth : 16 or 24
	u32 frames; // frame buffer count : 2
};

static struct struct_frame_buf_mark  frame_buf_mark = {
	.special_mark_1 = (('*' << 24) | ('^' << 16) | ('^' << 8) | ('*' << 0)),
	.special_mark_2 = (('I' << 24) | ('n' << 16) | ('f' << 8) | ('o' << 0)),
	.special_mark_3 = (('H' << 24) | ('e' << 16) | ('r' << 8) | ('e' << 0)),
	.special_mark_4 = (('f' << 24) | ('b' << 16) | ('u' << 8) | ('f' << 0)),
	.p_fb   = 0,
	.resX   = 1024,
	.resY   = 600,
	.bpp    = 32,
	.frames = 2
};

struct s3c_platform_fb *to_fb_plat(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	return (struct s3c_platform_fb *)pdev->dev.platform_data;
}

#ifdef DISPLAY_BOOT_PROGRESS
static int progress_flag = 0;
static int progress_pos;
static struct timer_list progress_timer;

#define PROGRESS_BAR_LEFT_POS	80 //116
#define PROGRESS_BAR_RIGHT_POS	530 //490
#define PROGRESS_BAR_START_X	328
#define PROGRESS_BAR_WIDTH		4
#define PROGRESS_BAR_HEIGHT		8

static unsigned char anycall_progress_bar_left[] =
{	
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0xf3, 0xc5, 0x00, 0x00, 0xf3, 0xc5, 0x00, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0xf3, 0xc5, 0x00, 0x00, 0xf3, 0xc5, 0x00, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0xf3, 0xc5, 0x00, 0x00, 0xf3, 0xc5, 0x00, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0xf3, 0xc5, 0x00, 0x00, 0xf3, 0xc5, 0x00, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00
};

static unsigned char anycall_progress_bar_right[] =
{	
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0xf3, 0xc5, 0x00, 0x00, 0xf3, 0xc5, 0x00, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 
	0xf3, 0xc5, 0x00, 0x00, 0xf3, 0xc5, 0x00, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 
	0xf3, 0xc5, 0x00, 0x00, 0xf3, 0xc5, 0x00, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 
	0xf3, 0xc5, 0x00, 0x00, 0xf3, 0xc5, 0x00, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00
};

static unsigned char anycall_progress_bar_center[] =
{
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0xf3, 0xc5, 0x00, 0x00, 0xf3, 0xc5, 0x00, 0x00,
	0xf3, 0xc5, 0x00, 0x00, 0xf3, 0xc5, 0x00, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00
};

static unsigned char anycall_progress_bar[] = 
{
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00,
	0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00, 0x33, 0x33, 0x33, 0x00
};

static void s3cfb_start_progress(struct fb_info *fb);
static void s3cfb_stop_progress(void);
static void progress_timer_handler(unsigned long data);
#endif

static unsigned int bootloaderfb;
module_param_named(bootloaderfb, bootloaderfb, uint, 0444);
MODULE_PARM_DESC(bootloaderfb, "Address of booting logo image in Bootloader");

#ifndef CONFIG_FRAMEBUFFER_CONSOLE

#if defined(CONFIG_FB_S3C_LVDS)

#define LOGO_MEM_SIZE       		1024*600*4
#define LOGO_MEM_BASE		        0x4EC00000 

#define BOOT_FB_WINDOW	0

static int s3cfb_draw_logo(struct fb_info *fb)
{
     memcpy(fb->screen_base, \
            phys_to_virt(LOGO_MEM_BASE), LOGO_MEM_SIZE);
 
        return 0;
}

#else //defined(CONFIG_FB_S3C_LVDS) 
static int s3cfb_draw_logo(struct fb_info *fb)
{
#ifdef CONFIG_FB_S3C_SPLASH_SCREEN
	struct fb_fix_screeninfo *fix = &fb->fix;
	struct fb_var_screeninfo *var = &fb->var;

	u32 height = var->yres / 3;
	u32 line = fix->line_length;
	u32 i, j;

	for (i = 0; i < height; i++) {
		int offset = i * line;
		for (j = 0; j < var->xres; j++) {
			fb->screen_base[offset++] = 0;
			fb->screen_base[offset++] = 0;
			fb->screen_base[offset++] = 0xff;
			fb->screen_base[offset++] = 0;
		}
	}

	for (i = height; i < height * 2; i++) {
		int offset = i * line;
		for (j = 0; j < var->xres; j++) {
			fb->screen_base[offset++] = 0;
			fb->screen_base[offset++] = 0xff;
			fb->screen_base[offset++] = 0;
			fb->screen_base[offset++] = 0;
		}
	}

	for (i = height * 2; i < height * 3; i++) {
		int offset = i * line;
		for (j = 0; j < var->xres; j++) {
			fb->screen_base[offset++] = 0xff;
			fb->screen_base[offset++] = 0;
			fb->screen_base[offset++] = 0;
			fb->screen_base[offset++] = 0;
		}
	}
#endif
	if (bootloaderfb) {
		u8 *logo_virt_buf;
		logo_virt_buf = ioremap_nocache(bootloaderfb,
				fb->var.yres * fb->fix.line_length);

		memcpy(fb->screen_base, logo_virt_buf,
				fb->var.yres * fb->fix.line_length);
		iounmap(logo_virt_buf);
	}
	return 0;
}
#endif //defined(CONFIG_FB_S3C_LVDS) 

#endif
static irqreturn_t s3cfb_irq_frame(int irq, void *data)
{
	struct s3cfb_global *fbdev = (struct s3cfb_global *)data;

	s3cfb_clear_interrupt(fbdev);

	complete_all(&fbdev->fb_complete);

	return IRQ_HANDLED;
}
static void s3cfb_set_window(struct s3cfb_global *ctrl, int id, int enable)
{
	struct s3cfb_window *win = ctrl->fb[id]->par;

	if (enable) {
		s3cfb_window_on(ctrl, id);
		win->enabled = 1;
	} else {
		s3cfb_window_off(ctrl, id);
		win->enabled = 0;
	}
}
static int s3cfb_init_global(struct s3cfb_global *ctrl)
{
	ctrl->output = OUTPUT_RGB;
	ctrl->rgb_mode = MODE_RGB_P;

	init_completion(&ctrl->fb_complete);
	mutex_init(&ctrl->lock);

	s3cfb_set_output(ctrl);
	s3cfb_set_display_mode(ctrl);
	s3cfb_set_polarity(ctrl);
	s3cfb_set_timing(ctrl);
	s3cfb_set_lcd_size(ctrl);

	return 0;
}
static int s3cfb_map_video_memory(struct fb_info *fb)
{
	struct fb_fix_screeninfo *fix = &fb->fix;
	struct s3cfb_window *win = fb->par;
	struct s3cfb_global *fbdev =
		platform_get_drvdata(to_platform_device(fb->device));
	struct s3c_platform_fb *pdata = to_fb_plat(fbdev->dev);

	if (win->owner == DMA_MEM_OTHER) {
		fix->smem_start = win->other_mem_addr;
		fix->smem_len = win->other_mem_size;
		return 0;
	}

	if (fb->screen_base)
		return 0;

//#if defined(CONFIG_FB_S3C_LVDS)
//	fix->smem_start = pdata->pmem_start;
//	fb->screen_base = ioremap_wc(fix->smem_start, pdata->pmem_size);
//#else
	if (pdata && pdata->pmem_start && (pdata->pmem_size >= fix->smem_len)) {
		fix->smem_start = pdata->pmem_start;
		fb->screen_base = ioremap_wc(fix->smem_start, pdata->pmem_size);
	} else
		fb->screen_base = dma_alloc_writecombine(fbdev->dev,
						 PAGE_ALIGN(fix->smem_len),
						 (unsigned int *)
						 &fix->smem_start, GFP_KERNEL);
//#endif

	if (!fb->screen_base)
		return -ENOMEM;

	dev_info(fbdev->dev, "[fb%d] dma: 0x%08x, cpu: 0x%08x, "
			 "size: 0x%08x\n", win->id,
			 (unsigned int)fix->smem_start,
			 (unsigned int)fb->screen_base, fix->smem_len);

	memset(fb->screen_base, 0, fix->smem_len);
	win->owner = DMA_MEM_FIMD;

	return 0;
}

static int s3cfb_map_default_video_memory(struct fb_info *fb)
{
#if defined(CONFIG_FB_S3C_VIRTUAL)
	struct fb_fix_screeninfo *fix = &fb->fix;
	struct s3cfb_window *win = fb->par;
	struct s3c_platform_fb *pdata = to_fb_plat(fbdev->dev);

	if (win->owner == DMA_MEM_OTHER)
		return 0;

	fix->smem_start = pdata->pmem_start;
	fb->screen_base = ioremap_wc(fix->smem_start, pdata->pmem_size);

	if (!fb->screen_base)
		return -ENOMEM;
	else
		dev_info(fbdev->dev, "[fb%d] dma: 0x%08x, cpu: 0x%08x, "
			"size: 0x%08x\n", win->id,
			(unsigned int)fix->smem_start,
			(unsigned int)fb->screen_base, fix->smem_len);

	memset(fb->screen_base, 0, fix->smem_len);
	win->owner = DMA_MEM_FIMD;
#else
	s3cfb_map_video_memory(fb);
#endif
	return 0;
}

static int s3cfb_unmap_video_memory(struct fb_info *fb)
{
	struct s3cfb_global *fbdev =
		platform_get_drvdata(to_platform_device(fb->device));
	struct s3c_platform_fb *pdata = to_fb_plat(fbdev->dev);
	struct fb_fix_screeninfo *fix = &fb->fix;
	struct s3cfb_window *win = fb->par;

	if (fix->smem_start) {
		if (win->owner == DMA_MEM_FIMD) {
//#if defined(CONFIG_FB_S3C_LVDS)
//			iounmap(fb->screen_base);
//#else		
			if (pdata && pdata->pmem_start &&
					(pdata->pmem_size >= fix->smem_len))
				iounmap(fb->screen_base);
			else
				dma_free_writecombine(fbdev->dev, fix->smem_len,
					      fb->screen_base, fix->smem_start);
//#endif						  
		}
		fix->smem_start = 0;
		fix->smem_len = 0;
		dev_info(fbdev->dev,
			"[fb%d] video memory released\n", win->id);
	}

	return 0;
}
static int s3cfb_unmap_default_video_memory(struct fb_info *fb)
{
	struct s3cfb_global *fbdev =
		platform_get_drvdata(to_platform_device(fb->device));
	struct s3c_platform_fb *pdata = to_fb_plat(fbdev->dev);
	struct fb_fix_screeninfo *fix = &fb->fix;
	struct s3cfb_window *win = fb->par;

	if (fix->smem_start) {
#if defined(CONFIG_FB_S3C_VIRTUAL)
		iounmap(fb->screen_base);
#else
		if (pdata && pdata->pmem_start &&
				(pdata->pmem_size >= fix->smem_len))
			iounmap(fb->screen_base);
		else
			dma_free_writecombine(fbdev->dev, fix->smem_len,
				      fb->screen_base, fix->smem_start);
#endif
		fix->smem_start = 0;
		fix->smem_len = 0;
		dev_info(fbdev->dev,
			"[fb%d] video memory released\n", win->id);
	}

	return 0;
}

static void s3cfb_set_bitfield(struct fb_var_screeninfo *var)
{
	switch (var->bits_per_pixel) {
	case 16:
		if (var->transp.length == 1) {
			var->red.offset = 10;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 5;
			var->blue.offset = 0;
			var->blue.length = 5;
			var->transp.offset = 15;
		} else if (var->transp.length == 4) {
			var->red.offset = 8;
			var->red.length = 4;
			var->green.offset = 4;
			var->green.length = 4;
			var->blue.offset = 0;
			var->blue.length = 4;
			var->transp.offset = 12;
		} else {
			var->red.offset = 11;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 0;
			var->blue.length = 5;
			var->transp.offset = 0;
		}
		break;

	case 24:
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;

	case 32:
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 24;
		var->transp.length = 8;

		break;
	}
}

static void s3cfb_set_alpha_info(struct fb_var_screeninfo *var,
				struct s3cfb_window *win)
{
	if (var->transp.length > 0)
		win->alpha.mode = PIXEL_BLENDING;
	else {
		win->alpha.mode = PLANE_BLENDING;
		win->alpha.channel = 0;
		win->alpha.value = S3CFB_AVALUE(0xf, 0xf, 0xf);
	}
}
static int s3cfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	struct s3cfb_window *win = fb->par;
	struct s3cfb_global *fbdev =
		platform_get_drvdata(to_platform_device(fb->device));
	struct s3cfb_lcd *lcd = fbdev->lcd;

	dev_dbg(fbdev->dev, "[fb%d] check_var\n", win->id);

	if (var->bits_per_pixel != 16 && var->bits_per_pixel != 24 &&
	    var->bits_per_pixel != 32) {
		dev_err(fbdev->dev, "invalid bits per pixel\n");
		return -EINVAL;
	}

	if (var->xres > lcd->width)
		var->xres = lcd->width;

	if (var->yres > lcd->height)
		var->yres = lcd->height;

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
#if 0 /*  FIMD is apply the LSI patch CL:182711 GaLAB */
	if (var->yres_virtual > var->yres * CONFIG_FB_S3C_NR_BUFFERS)
		var->yres_virtual = var->yres * CONFIG_FB_S3C_NR_BUFFERS;
#endif
	var->xoffset = 0;

	if (var->yoffset + var->yres > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;

	if (win->x + var->xres > lcd->width)
		win->x = lcd->width - var->xres;

	if (win->y + var->yres > lcd->height)
		win->y = lcd->height - var->yres;

	s3cfb_set_bitfield(var);
	s3cfb_set_alpha_info(var, win);

	return 0;
}

static void s3cfb_set_win_params(struct s3cfb_global *ctrl, int id)
{
	s3cfb_set_window_control(ctrl, id);
	s3cfb_set_window_position(ctrl, id);
	s3cfb_set_window_size(ctrl, id);
	s3cfb_set_buffer_address(ctrl, id);
	s3cfb_set_buffer_size(ctrl, id);

	if (id > 0) {
		s3cfb_set_alpha_blending(ctrl, id);
		s3cfb_set_chroma_key(ctrl, id);
	}
}
static int s3cfb_set_par(struct fb_info *fb)
{
	struct s3cfb_global *fbdev =
		platform_get_drvdata(to_platform_device(fb->device));
	struct s3c_platform_fb *pdata = to_fb_plat(fbdev->dev);
	struct s3cfb_window *win = fb->par;

	dev_dbg(fbdev->dev, "[fb%d] set_par\n", win->id);

	/* modify the fix info */
	if (win->id != pdata->default_win) {
		fb->fix.line_length = fb->var.xres_virtual *
						fb->var.bits_per_pixel / 8;
		fb->fix.smem_len = fb->fix.line_length * fb->var.yres_virtual;

		s3cfb_map_video_memory(fb);
	}

	s3cfb_set_win_params(fbdev, win->id);

	return 0;
}

static int s3cfb_blank(int blank_mode, struct fb_info *fb)
{
	struct s3cfb_window *win = fb->par;
	struct s3cfb_global *fbdev =
		platform_get_drvdata(to_platform_device(fb->device));

	dev_dbg(fbdev->dev, "change blank mode\n");

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		if (fb->fix.smem_start) {
			s3cfb_win_map_off(fbdev, win->id);
			s3cfb_set_window(fbdev, win->id, 1);
		} else
			dev_info(fbdev->dev,
				 "[fb%d] no allocated memory for unblank\n",
				 win->id);
		break;

	case FB_BLANK_NORMAL:
		s3cfb_win_map_on(fbdev, win->id, 0);
		s3cfb_set_window(fbdev, win->id, 1);

		break;

	case FB_BLANK_POWERDOWN:
		s3cfb_set_window(fbdev, win->id, 0);
		s3cfb_win_map_off(fbdev, win->id);

		break;

	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	default:
		dev_dbg(fbdev->dev, "unsupported blank mode\n");
		return -EINVAL;
	}

	return 0;
}
static int s3cfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	struct fb_fix_screeninfo *fix = &fb->fix;
	struct s3cfb_window *win = fb->par;
	struct s3cfb_global *fbdev =
		platform_get_drvdata(to_platform_device(fb->device));

	if (var->yoffset + var->yres > var->yres_virtual) {
		dev_err(fbdev->dev, "invalid yoffset value\n");
		return -EINVAL;
	}

	if (win->owner == DMA_MEM_OTHER)
		fix->smem_start = win->other_mem_addr;

	fb->var.yoffset = var->yoffset;

	dev_dbg(fbdev->dev,
		"[fb%d] yoffset for pan display: %d\n",
		win->id, var->yoffset);

	s3cfb_set_buffer_address(fbdev, win->id);

	return 0;
}

static unsigned int __chan_to_field(unsigned int chan,
					   struct fb_bitfield bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf.length;

	return chan << bf.offset;
}

static int s3cfb_setcolreg(unsigned int regno, unsigned int red,
			   unsigned int green, unsigned int blue,
			   unsigned int transp, struct fb_info *fb)
{
	unsigned int *pal = (unsigned int *)fb->pseudo_palette;
	unsigned int val = 0;

	if (regno < 16) {
		val |= __chan_to_field(red, fb->var.red);
		val |= __chan_to_field(green, fb->var.green);
		val |= __chan_to_field(blue, fb->var.blue);
		val |= __chan_to_field(transp, fb->var.transp);

		pal[regno] = val;
	}

	return 0;
}

static int s3cfb_open(struct fb_info *fb, int user)
{
	struct s3cfb_global *fbdev =
		platform_get_drvdata(to_platform_device(fb->device));
	struct s3c_platform_fb *pdata = to_fb_plat(fbdev->dev);
	struct s3cfb_window *win = fb->par;
	int ret = 0;

	mutex_lock(&fbdev->lock);

	if (win->in_use && win->id != pdata->default_win)
		ret = -EBUSY;
	else
		win->in_use++;

	mutex_unlock(&fbdev->lock);

	return ret;
}
static int s3cfb_release_window(struct fb_info *fb)
{
	struct s3cfb_global *fbdev =
		platform_get_drvdata(to_platform_device(fb->device));
	struct s3c_platform_fb *pdata = to_fb_plat(fbdev->dev);
	struct s3cfb_window *win = fb->par;

	if (win->id != pdata->default_win) {
		s3cfb_set_window(fbdev, win->id, 0);
		s3cfb_unmap_video_memory(fb);
		s3cfb_set_buffer_address(fbdev, win->id);
	}

	win->x = 0;
	win->y = 0;

	return 0;
}
static int s3cfb_release(struct fb_info *fb, int user)
{
	struct s3cfb_global *fbdev =
		platform_get_drvdata(to_platform_device(fb->device));
	struct s3cfb_window *win = fb->par;

	s3cfb_release_window(fb);

	mutex_lock(&fbdev->lock);

	if (!WARN_ON(!win->in_use))
		win->in_use--;

	mutex_unlock(&fbdev->lock);

	return 0;
}

static int s3cfb_wait_for_vsync(struct s3cfb_global *ctrl)
{
	int ret;

	dev_dbg(ctrl->dev, "waiting for VSYNC interrupt\n");

	ret = wait_for_completion_interruptible_timeout(
		&ctrl->fb_complete, msecs_to_jiffies(100));
	if (ret == 0)
		return -ETIMEDOUT;
	if (ret < 0)
		return ret;

	dev_dbg(ctrl->dev, "got a VSYNC interrupt\n");

	return ret;
}
static int s3cfb_ioctl(struct fb_info *fb, unsigned int cmd, unsigned long arg)
{
	struct s3cfb_global *fbdev =
		platform_get_drvdata(to_platform_device(fb->device));
	struct fb_var_screeninfo *var = &fb->var;
	struct s3cfb_window *win = fb->par;
	struct s3cfb_lcd *lcd = fbdev->lcd;
	struct fb_fix_screeninfo *fix = &fb->fix;
	struct s3cfb_next_info next_fb_info;

	int ret = 0;

	union {
		struct s3cfb_user_window user_window;
		struct s3cfb_user_plane_alpha user_alpha;
		struct s3cfb_user_chroma user_chroma;
		int vsync;
	} p;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		s3cfb_wait_for_vsync(fbdev);
		break;

	case S3CFB_WIN_POSITION:
		if (copy_from_user(&p.user_window,
				   (struct s3cfb_user_window __user *)arg,
				   sizeof(p.user_window)))
			ret = -EFAULT;
		else {
			if (p.user_window.x < 0)
				p.user_window.x = 0;

			if (p.user_window.y < 0)
				p.user_window.y = 0;

			if (p.user_window.x + var->xres > lcd->width)
				win->x = lcd->width - var->xres;
			else
				win->x = p.user_window.x;

			if (p.user_window.y + var->yres > lcd->height)
				win->y = lcd->height - var->yres;
			else
				win->y = p.user_window.y;

			s3cfb_set_window_position(fbdev, win->id);
		}
		break;

	case S3CFB_WIN_SET_PLANE_ALPHA:
		if (copy_from_user(&p.user_alpha,
				   (struct s3cfb_user_plane_alpha __user *)arg,
				   sizeof(p.user_alpha)))
			ret = -EFAULT;
		else {
			win->alpha.mode = PLANE_BLENDING;
			win->alpha.channel = p.user_alpha.channel;
			win->alpha.value =
			    S3CFB_AVALUE(p.user_alpha.red,
					 p.user_alpha.green, p.user_alpha.blue);

			s3cfb_set_alpha_blending(fbdev, win->id);
		}
		break;

	case S3CFB_WIN_SET_CHROMA:
		if (copy_from_user(&p.user_chroma,
				   (struct s3cfb_user_chroma __user *)arg,
				   sizeof(p.user_chroma)))
			ret = -EFAULT;
		else {
			win->chroma.enabled = p.user_chroma.enabled;
			win->chroma.key = S3CFB_CHROMA(p.user_chroma.red,
						       p.user_chroma.green,
						       p.user_chroma.blue);

			s3cfb_set_chroma_key(fbdev, win->id);
		}
		break;

	case S3CFB_SET_VSYNC_INT:
		if (get_user(p.vsync, (int __user *)arg))
			ret = -EFAULT;
		else {
			if (p.vsync)
				s3cfb_set_global_interrupt(fbdev, 1);

			s3cfb_set_vsync_interrupt(fbdev, p.vsync);
		}
		break;

	case S3CFB_GET_CURR_FB_INFO:
		next_fb_info.phy_start_addr = fix->smem_start;
		next_fb_info.xres = var->xres;
		next_fb_info.yres = var->yres;
		next_fb_info.xres_virtual = var->xres_virtual;
		next_fb_info.yres_virtual = var->yres_virtual;
		next_fb_info.xoffset = var->xoffset;
		next_fb_info.yoffset = var->yoffset;
		next_fb_info.lcd_offset_x = 0;
		next_fb_info.lcd_offset_y = 0;

		if (copy_to_user((void *)arg,
				 (struct s3cfb_next_info *) &next_fb_info,
				 sizeof(struct s3cfb_next_info)))
			return -EFAULT;
		break;
	}

	return ret;
}

struct fb_ops s3cfb_ops = {
	.owner = THIS_MODULE,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_check_var = s3cfb_check_var,
	.fb_set_par = s3cfb_set_par,
	.fb_blank = s3cfb_blank,
	.fb_pan_display = s3cfb_pan_display,
	.fb_setcolreg = s3cfb_setcolreg,
	.fb_ioctl = s3cfb_ioctl,
	.fb_open = s3cfb_open,
	.fb_release = s3cfb_release,
};

static void s3cfb_init_fbinfo(struct s3cfb_global *ctrl, int id)
{
	struct fb_info *fb = ctrl->fb[id];
	struct fb_fix_screeninfo *fix = &fb->fix;
	struct fb_var_screeninfo *var = &fb->var;
	struct s3cfb_window *win = fb->par;
	struct s3cfb_alpha *alpha = &win->alpha;
	struct s3cfb_lcd *lcd = ctrl->lcd;
	struct s3cfb_lcd_timing *timing = &lcd->timing;

	memset(win, 0, sizeof(*win));
	platform_set_drvdata(to_platform_device(ctrl->dev), ctrl);
	strcpy(fix->id, S3CFB_NAME);

	win->id = id;
	win->path = DATA_PATH_DMA;
	win->dma_burst = 16;
	alpha->mode = PLANE_BLENDING;

	fb->fbops = &s3cfb_ops;
	fb->flags = FBINFO_FLAG_DEFAULT;
	fb->pseudo_palette = &win->pseudo_pal;
#if (CONFIG_FB_S3C_NR_BUFFERS != 1)
	fix->xpanstep = 2;
	fix->ypanstep = 1;
#else
	fix->xpanstep = 0;
	fix->ypanstep = 0;
#endif
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	fix->visual = FB_VISUAL_TRUECOLOR;
	var->xres = lcd->width;
	var->yres = lcd->height;
#if defined(CONFIG_FB_S3C_VIRTUAL)
	var->xres_virtual = CONFIG_FB_S3C_X_VRES;
	var->yres_virtual = CONFIG_FB_S3C_Y_VRES * CONFIG_FB_S3C_NR_BUFFERS;
#else
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres * CONFIG_FB_S3C_NR_BUFFERS;
#endif
	var->bits_per_pixel = 32;
	var->xoffset = 0;
	var->yoffset = 0;
	var->width = lcd->p_width;
	var->height = lcd->p_height;
	var->transp.length = 0;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;
	fix->smem_len = fix->line_length * var->yres_virtual;

	var->nonstd = 0;
	var->activate = FB_ACTIVATE_NOW;
	var->vmode = FB_VMODE_NONINTERLACED;
	var->hsync_len = timing->h_sw;
	var->vsync_len = timing->v_sw;
	var->left_margin = timing->h_fp;
	var->right_margin = timing->h_bp;
	var->upper_margin = timing->v_fp;
	var->lower_margin = timing->v_bp;

	var->pixclock = lcd->freq * (var->left_margin + var->right_margin +
				var->hsync_len + var->xres) *
				(var->upper_margin + var->lower_margin +
				var->vsync_len + var->yres);

	dev_dbg(ctrl->dev, "pixclock: %d\n", var->pixclock);

	s3cfb_set_bitfield(var);
	s3cfb_set_alpha_info(var, win);

}

static int s3cfb_alloc_framebuffer(struct s3cfb_global *ctrl)
{
	struct s3c_platform_fb *pdata = to_fb_plat(ctrl->dev);
	int ret, i;

	ctrl->fb = kmalloc(pdata->nr_wins *
			sizeof(*(ctrl->fb)), GFP_KERNEL);
	if (!ctrl->fb) {
		dev_err(ctrl->dev, "not enough memory\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	for (i = 0; i < pdata->nr_wins; i++) {
		ctrl->fb[i] = framebuffer_alloc(sizeof(*ctrl->fb),
						 ctrl->dev);
		if (!ctrl->fb[i]) {
			dev_err(ctrl->dev, "not enough memory\n");
			ret = -ENOMEM;
			goto err_alloc_fb;
		}

		s3cfb_init_fbinfo(ctrl, i);

		if (i == pdata->default_win) {
			if (s3cfb_map_video_memory(ctrl->fb[i])) {
				dev_err(ctrl->dev,
					"failed to map video memory "
					"for default window (%d)\n", i);
				ret = -ENOMEM;
				goto err_map_video_mem;
			}
		}
	}

	return 0;

err_alloc_fb:
	while (--i >= 0) {
		if (i == pdata->default_win)
			s3cfb_unmap_default_video_memory(ctrl->fb[i]);

err_map_video_mem:
		framebuffer_release(ctrl->fb[i]);
	}
	kfree(ctrl->fb);

err_alloc:
	return ret;
}

static int s3cfb_register_framebuffer(struct s3cfb_global *ctrl)
{
	struct s3c_platform_fb *pdata = to_fb_plat(ctrl->dev);
	int ret, i, j;

	for (i = pdata->default_win;
		i < pdata->nr_wins + pdata->default_win; i++) {
			j = i % pdata->nr_wins;
			ret = register_framebuffer(ctrl->fb[j]);
			if (ret) {
				dev_err(ctrl->dev, "failed to register "
						"framebuffer device\n");
				return -EINVAL;
				goto err_register_fb;
			}
#ifndef CONFIG_FRAMEBUFFER_CONSOLE
			if (j == pdata->default_win) {
				s3cfb_check_var(&ctrl->fb[j]->var, ctrl->fb[j]);
				s3cfb_set_par(ctrl->fb[j]);
				s3cfb_draw_logo(ctrl->fb[j]);
			}
#endif
	}

	return 0;
err_register_fb:
	while (--i >= pdata->default_win) {
		j = i % pdata->nr_wins;
		unregister_framebuffer(ctrl->fb[j]);
	}
	return ret;
}
static int s3cfb_sysfs_show_win_power(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s3c_platform_fb *pdata = to_fb_plat(dev);
	struct platform_device *pdev = to_platform_device(dev);
	struct s3cfb_global *fbdev = platform_get_drvdata(pdev);
	struct s3cfb_window *win;
	char temp[16];
	int i;

	for (i = 0; i < pdata->nr_wins; i++) {
		win = fbdev->fb[i]->par;
		sprintf(temp, "[fb%d] %s\n", i, win->enabled ? "on" : "off");
		strcat(buf, temp);
	}

	return strlen(buf);
}

static int s3cfb_sysfs_store_win_power(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct s3c_platform_fb *pdata = to_fb_plat(dev);
	struct platform_device *pdev = to_platform_device(dev);
	struct s3cfb_global *fbdev = platform_get_drvdata(pdev);
	int id, to;
	int ret = 0;

	ret = strict_strtoul(buf, 10, (unsigned long *)&id);
	if (ret < 0)
		return -EINVAL;

	to = (id % 10);
	if (to != 0 && to != 1)
		return -EINVAL;

	id = (id / 10);
	if (id < 0 || id >= pdata->nr_wins)
		return -EINVAL;

	s3cfb_set_window(fbdev, id, to);

	return len;
}

static DEVICE_ATTR(win_power, S_IRUGO | S_IWUSR,
		   s3cfb_sysfs_show_win_power, s3cfb_sysfs_store_win_power);


#if defined(CONFIG_MACH_S5PC110_P1)
static void s3cfb_update_framebuffer(struct fb_info *fb,
									int x, int y, void *buffer, 
									int src_width, int src_height)
{
	struct s3c_platform_fb *pdata = to_fb_plat(fbdev->dev);
	struct fb_fix_screeninfo *fix = &fb->fix;
	struct fb_var_screeninfo *var = &fb->var;
	int row;
	int bytes_per_pixel = (var->bits_per_pixel / 8 );
	
	unsigned char *pSrc = buffer;
	unsigned char *pDst = fbdev->fb[pdata->default_win]->screen_base;

	if (x+src_width > var->xres || y+src_height > var->yres)
	{
		// err("invalid destination coordinate or source size (%d, %d) (%d %d) \n", x, y, src_width, src_height);
		return;
	}	

	pDst += y * fix->line_length + x * bytes_per_pixel;	
    
	for (row = 0; row < src_width ; row++)	
	{		
		memcpy(pDst, pSrc, src_height * bytes_per_pixel);
		pSrc += src_height * bytes_per_pixel;
		pDst += fix->line_length;
	}
}
#endif //defined(CONFIG_MACH_S5PC110_P1)

#ifdef DISPLAY_BOOT_PROGRESS
static void s3cfb_start_progress(struct fb_info *fb)
{	
	// int y_pos;
	init_timer(&progress_timer);

	progress_timer.expires  = (get_jiffies_64() + (HZ/20));
	progress_timer.data     = (long)fb;
	progress_timer.function = progress_timer_handler;
	progress_pos = PROGRESS_BAR_LEFT_POS;
	
    #if 0
	// draw progress background.
	for (y_pos = PROGRESS_BAR_LEFT_POS ; y_pos <= PROGRESS_BAR_RIGHT_POS ; y_pos += PROGRESS_BAR_HEIGHT)
	{
		s3cfb_update_framebuffer(fb,
			PROGRESS_BAR_START_X,
			y_pos,
			(void*)anycall_progress_bar,					
			PROGRESS_BAR_WIDTH,
			PROGRESS_BAR_HEIGHT);
	}
	s3cfb_update_framebuffer(fb,PROGRESS_BAR_START_X,PROGRESS_BAR_LEFT_POS,	
                                (void*)anycall_progress_bar_left,PROGRESS_BAR_HEIGHT, PROGRESS_BAR_WIDTH);
	
	progress_pos += PROGRESS_BAR_HEIGHT;	
	
	s3cfb_update_framebuffer(fb, PROGRESS_BAR_START_X, progress_pos, 
                                (void*)anycall_progress_bar_right,  PROGRESS_BAR_HEIGHT, PROGRESS_BAR_WIDTH);
    #endif
	
	add_timer(&progress_timer);

	progress_flag = 1;

}

static void s3cfb_stop_progress(void)
{	
	if (progress_flag == 0)		
		return;	
	del_timer(&progress_timer);	
	progress_flag = 0;
}

static void progress_timer_handler(unsigned long data)
{	
	int i;	
	for(i = 0; i < 4; i++)	
	{		
		s3cfb_update_framebuffer((struct fb_info *)data,
			PROGRESS_BAR_START_X,
			progress_pos++,
			(void*)anycall_progress_bar_center,					
			1,
			PROGRESS_BAR_HEIGHT);	
	}	
	
#if 0
	s3cfb_update_framebuffer((struct fb_info *)data,		
		PROGRESS_BAR_START_X,
		progress_pos,
		(void*)anycall_progress_bar_right,		
		PROGRESS_BAR_WIDTH,
		PROGRESS_BAR_HEIGHT);    
#endif
	
	if (progress_pos + PROGRESS_BAR_HEIGHT >= PROGRESS_BAR_RIGHT_POS )    
	{        
		s3cfb_stop_progress();    
	}    
	else    
	{        
		progress_timer.expires = (get_jiffies_64() + (HZ/20));
		progress_timer.function = progress_timer_handler;
		add_timer(&progress_timer);
	}
}
#endif

#if defined(CONFIG_MACH_S5PC110_P1) && defined(CONFIG_TARGET_LOCALE_KOR)
static int s3cfb_sysfs_show_pclk_mode(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", current_band_for_pclk);
}

static int s3cfb_sysfs_store_pclk_mode(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct s3c_platform_fb *pdata = to_fb_plat(dev);
	struct s3cfb_global fb_struct;
	int id;
	id = simple_strtoul(buf, NULL, 10);
	printk("%s, id=%d\n", __func__, id);

	current_band_for_pclk  = id;

	return len;
	
}
static DEVICE_ATTR(pclk_mode, 0666,
		   s3cfb_sysfs_show_pclk_mode, s3cfb_sysfs_store_pclk_mode);

static struct s3cfb_lcd lvds_44_46Mhz = {
        .timing = {
                .h_fp = 50,
                .h_bp = 60,
                .h_sw = 35,
                .v_fp = 10,
                .v_fpe = 1,
                .v_bp = 11,
                .v_bpe = 1,
                .v_sw = 10,
        },
};

static struct s3cfb_lcd lvds_51_3Mhz = {
        .timing = {
                .h_fp = 120,
                .h_bp = 170,
                .h_sw = 40,
                .v_fp = 10,
                .v_fpe = 1,
                .v_bp = 11,
                .v_bpe = 1,
                .v_sw = 10,
        },
};

static void s3cfb_change_pclk_mode(int id)
{
	struct s3cfb_global fb_struct;

	switch(id)
		{
		case 2:
			printk("band(%d), set PCLK 51.3Mhz\n", id);
			fbdev->lcd->timing = lvds_51_3Mhz.timing;
			clk_set_rate(fbdev->clock, 52000000);
			//msleep(10);
		#ifdef CONFIG_FB_S3C_CMC623
			tune_cmc623_set_lcddata(fbdev->lcd);
			tune_cmc623_set_lcd_pclk(51300000);
		#endif
			break;

		default:
			printk("band(%d), set PCLK 44.46Mhz\n", id);
			fbdev->lcd->timing = lvds_44_46Mhz.timing;
			clk_set_rate(fbdev->clock, 45000000);
			//msleep(10);
		#ifdef CONFIG_FB_S3C_CMC623
			tune_cmc623_set_lcddata(fbdev->lcd);
			tune_cmc623_set_lcd_pclk(44460000);
		#endif
			break;
		}
}
#endif

static int __devinit s3cfb_probe(struct platform_device *pdev)
{
	struct s3c_platform_fb *pdata;
	struct s3cfb_global *fbdev;
	struct resource *res;
	int i, j, ret = 0;

	fbdev = kzalloc(sizeof(struct s3cfb_global), GFP_KERNEL);
	if (!fbdev) {
		dev_err(fbdev->dev, "failed to allocate for "
			"global fb structure\n");
		ret = -ENOMEM;
		goto err_global;
	}
	fbdev->dev = &pdev->dev;

	fbdev->regulator = regulator_get(&pdev->dev, "pd");
	if (!fbdev->regulator) {
		dev_err(fbdev->dev, "failed to get regulator\n");
		ret = -EINVAL;
		goto err_regulator;
	}
	ret = regulator_enable(fbdev->regulator);
	if (ret < 0) {
		dev_err(fbdev->dev, "failed to enable regulator\n");
		ret = -EINVAL;
		goto err_regulator;
	}
	pdata = to_fb_plat(&pdev->dev);
	if (!pdata) {
		dev_err(fbdev->dev, "failed to get platform data\n");
		ret = -EINVAL;
		goto err_pdata;
	}

	fbdev->lcd = (struct s3cfb_lcd *)pdata->lcd;

#ifdef CONFIG_FB_S3C_CMC623
	tune_cmc623_set_lcddata(fbdev->lcd);
#endif

	if (pdata->cfg_gpio)
		pdata->cfg_gpio(pdev);

	if (pdata->clk_on)
		pdata->clk_on(pdev, &fbdev->clock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(fbdev->dev, "failed to get io memory region\n");
		ret = -EINVAL;
		goto err_io;
	}

	res = request_mem_region(res->start,
				 res->end - res->start + 1, pdev->name);
	if (!res) {
		dev_err(fbdev->dev, "failed to request io memory region\n");
		ret = -EINVAL;
		goto err_io;
	}

	fbdev->regs = ioremap(res->start, res->end - res->start + 1);
	if (!fbdev->regs) {
		dev_err(fbdev->dev, "failed to remap io region\n");
		ret = -EINVAL;
		goto err_mem;
	}

	s3cfb_set_vsync_interrupt(fbdev, 1);
	s3cfb_set_global_interrupt(fbdev, 1);
	s3cfb_init_global(fbdev);

#ifdef CONFIG_FB_S3C_MDNIE
	s3c_mdnie_setup();
#endif

	if (s3cfb_alloc_framebuffer(fbdev)) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	if (s3cfb_register_framebuffer(fbdev)) {
		ret = -EINVAL;
		goto err_register;
	}

	frame_buf_mark.p_fb = (void*)fbdev->fb[pdata->default_win]->fix.smem_start;
	frame_buf_mark.bpp = fbdev->fb[pdata->default_win]->var.bits_per_pixel;

	s3cfb_set_clock(fbdev);
#ifdef CONFIG_FB_S3C_MDNIE
	mDNIe_Mode_Set();
#endif 
	s3cfb_set_window(fbdev, pdata->default_win, 1);

	if(pdata->default_win != BOOT_FB_WINDOW) {
		dev_warn(fbdev->dev, "closing bootloader FIMD window 0 \n",BOOT_FB_WINDOW);
		s3cfb_set_window(fbdev,BOOT_FB_WINDOW, 0);
	}


#if defined(CONFIG_MACH_S5PC110_P1) && defined(CONFIG_FB_S3C_MDNIE) && defined(__NO_BOOT_INIT_MDNIE__)
	// it doesn't need this if bootloader enable mdnie
	s3cfb_ielcd_enable(fbdev, 1);
	s3c_mdnie_init_global(fbdev);
	s3c_mdnie_start(fbdev);
#endif 

#ifndef BOOTLOADER_INIT_LCD
	s3cfb_display_on(fbdev);
#endif

	fbdev->irq = platform_get_irq(pdev, 0);
	if (request_irq(fbdev->irq, s3cfb_irq_frame, IRQF_SHARED,
			pdev->name, fbdev)) {
		dev_err(fbdev->dev, "request_irq failed\n");
		ret = -EINVAL;
		goto err_irq;
	}

#ifdef CONFIG_FB_S3C_LCD_INIT
#if defined(CONFIG_FB_S3C_TL2796)
	if (pdata->backlight_on)
		pdata->backlight_on(pdev);
#endif
#ifndef BOOTLOADER_INIT_LCD
	if (!bootloaderfb && pdata->reset_lcd)
		pdata->reset_lcd(pdev);
#endif
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	fbdev->early_suspend.suspend = s3cfb_early_suspend;
	fbdev->early_suspend.resume = s3cfb_late_resume;
	fbdev->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&fbdev->early_suspend);
#endif

	ret = device_create_file(&(pdev->dev), &dev_attr_win_power);
	if (ret < 0)
		dev_err(fbdev->dev, "failed to add sysfs entries\n");
#if defined(CONFIG_MACH_S5PC110_P1) && defined(CONFIG_TARGET_LOCALE_KOR)
    ret = device_create_file(&(pdev->dev), &dev_attr_pclk_mode);
    if (ret < 0)
        dev_err(fbdev->dev,"s3cfb: failed to add entries\n");
#endif

	dev_info(fbdev->dev, "registered successfully\n");

#ifdef DISPLAY_BOOT_PROGRESS
	if(!get_boot_charger_info() && show_progress == 1)
	{
		s3cfb_start_progress(fbdev->fb[pdata->default_win]);
	}
#endif

	return 0;

err_irq:
	s3cfb_display_off(fbdev);
	s3cfb_set_window(fbdev, pdata->default_win, 0);
	for (i = pdata->default_win;
			i < pdata->nr_wins + pdata->default_win; i++) {
		j = i % pdata->nr_wins;
		unregister_framebuffer(fbdev->fb[j]);
	}
err_register:
	for (i = 0; i < pdata->nr_wins; i++) {
		if (i == pdata->default_win)
			s3cfb_unmap_default_video_memory(fbdev->fb[i]);
		framebuffer_release(fbdev->fb[i]);
	}
	kfree(fbdev->fb);

err_alloc:
	iounmap(fbdev->regs);

err_mem:
	release_mem_region(res->start,
				 res->end - res->start + 1);

err_io:
	pdata->clk_off(pdev, &fbdev->clock);

err_pdata:
	regulator_disable(fbdev->regulator);

err_regulator:
	kfree(fbdev);

err_global:
	return ret;
}

static int __devexit s3cfb_remove(struct platform_device *pdev)
{
	struct s3c_platform_fb *pdata = to_fb_plat(&pdev->dev);
	struct s3cfb_global *fbdev = platform_get_drvdata(pdev);
	struct s3cfb_window *win;
	struct resource *res;
	struct fb_info *fb;
	int i;

	device_remove_file(&(pdev->dev), &dev_attr_win_power);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&fbdev->early_suspend);
#endif

	free_irq(fbdev->irq, fbdev);
	iounmap(fbdev->regs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start,
			res->end - res->start + 1);

	pdata->clk_off(pdev, &fbdev->clock);

	for (i = 0; i < pdata->nr_wins; i++) {
		fb = fbdev->fb[i];

		if (fb) {
			win = fb->par;
			if (win->id == pdata->default_win)
				s3cfb_unmap_default_video_memory(fb);
			else
				s3cfb_unmap_video_memory(fb);

			s3cfb_set_buffer_address(fbdev, i);
			framebuffer_release(fb);
		}
	}

	regulator_disable(fbdev->regulator);

	kfree(fbdev->fb);
	kfree(fbdev);

	return 0;
}

void s3cfb_early_suspend(struct early_suspend *h)
{
	struct s3cfb_global *fbdev =
		container_of(h, struct s3cfb_global, early_suspend);

	pr_debug("s3cfb_early_suspend is called\n");

#if defined (CONFIG_FB_S3C_LVDS)
	lms700_powerdown();
#endif

	#ifdef CONFIG_FB_S3C_CMC623
	tune_cmc623_suspend();
	#endif

	#ifdef CONFIG_FB_S3C_MDNIE
	writel(0,fbdev->regs + 0x27c);
	msleep(20);  // mdelay->msleep
	writel(0x2, S5P_MDNIE_SEL);
	s3c_mdnie_stop();
	#endif

	s3cfb_display_off(fbdev);
	#ifdef CONFIG_FB_S3C_MDNIE
	s3c_mdnie_off();
	#endif 
	clk_disable(fbdev->clock);
#if defined(CONFIG_FB_S3C_TL2796)
	lcd_cfg_gpio_early_suspend();
#endif
	regulator_disable(fbdev->regulator);

	return ;
}

void s3cfb_late_resume(struct early_suspend *h)
{
	struct s3cfb_global *fbdev =
		container_of(h, struct s3cfb_global, early_suspend);
	struct s3c_platform_fb *pdata = to_fb_plat(fbdev->dev);
	struct platform_device *pdev = to_platform_device(fbdev->dev);
	struct fb_info *fb;
	struct s3cfb_window *win;
	int i, j, ret;

	pr_info("s3cfb_late_resume is called\n");

	ret = regulator_enable(fbdev->regulator);
	if (ret < 0)
		dev_err(fbdev->dev, "failed to enable regulator\n");

#if defined(CONFIG_FB_S3C_TL2796)
	lcd_cfg_gpio_late_resume();
#endif
	dev_dbg(fbdev->dev, "wake up from suspend\n");
	if (pdata->cfg_gpio)
		pdata->cfg_gpio(pdev);

	clk_enable(fbdev->clock);

#if defined(CONFIG_MACH_S5PC110_P1) && defined(CONFIG_TARGET_LOCALE_KOR)
	s3cfb_change_pclk_mode(current_band_for_pclk);
#endif

#ifdef CONFIG_FB_S3C_MDNIE
#if !(defined(CONFIG_MACH_S5PC110_P1) && defined(CONFIG_TARGET_LOCALE_KOR))
	writel(0x1, S5P_MDNIE_SEL);
	writel(3,fbdev->regs + 0x27c);
#endif
#endif
	s3cfb_init_global(fbdev);
	s3cfb_set_clock(fbdev);

#ifdef CONFIG_FB_S3C_MDNIE
#if !(defined(CONFIG_MACH_S5PC110_P1) && defined(CONFIG_TARGET_LOCALE_KOR))
	s3c_mdnie_init_global(fbdev);
	s3c_mdnie_start(fbdev);
#endif 
#endif 

	#ifdef CONFIG_FB_S3C_CMC623
	tune_cmc623_pre_resume();
	#endif

	s3cfb_display_on(fbdev);

	#ifdef CONFIG_FB_S3C_CMC623
	tune_cmc623_resume();
	#endif

#if defined (CONFIG_FB_S3C_LVDS)
	lms700_powerup();
#endif

	for (i = pdata->default_win;
		i < pdata->nr_wins + pdata->default_win; i++) {
			j = i % pdata->nr_wins;
			fb = fbdev->fb[j];
			win = fb->par;
			if ((win->path == DATA_PATH_DMA) && (win->enabled)) {
				s3cfb_set_par(fb);
				s3cfb_set_window(fbdev, win->id, 1);
			}
	}

	s3cfb_set_vsync_interrupt(fbdev, 1);
	s3cfb_set_global_interrupt(fbdev, 1);

	if (pdata->backlight_on)
		pdata->backlight_on(pdev);

	if (pdata->reset_lcd)
		pdata->reset_lcd(pdev);

	pr_info("s3cfb_late_resume is complete\n");
	return ;
}

static struct platform_driver s3cfb_driver = {
	.probe = s3cfb_probe,
	.remove = __devexit_p(s3cfb_remove),
	.driver = {
		   .name = S3CFB_NAME,
		   .owner = THIS_MODULE,
	},
};

static int __init s3cfb_register(void)
{
	platform_driver_register(&s3cfb_driver);

	return 0;
}
static void __exit s3cfb_unregister(void)
{
	platform_driver_unregister(&s3cfb_driver);
}

#ifdef DISPLAY_BOOT_PROGRESS
static void __init bootmode(char **p)
{
	show_progress = simple_strtoul(*p, p, 10);
}
__early_param("bootmode=", bootmode);
#endif

module_init(s3cfb_register);
module_exit(s3cfb_unregister);

MODULE_AUTHOR("Jonghun, Han <jonghun.han@samsung.com>");
MODULE_AUTHOR("Jinsung, Yang <jsgood.yang@samsung.com>");
MODULE_DESCRIPTION("Samsung Display Controller (FIMD) driver");
MODULE_LICENSE("GPL");
