/* linux/drivers/media/video/samsung/g2d/fimg2d_3x.h
 *
 * Copyright  2010 Samsung Electronics Co, Ltd. All Rights Reserved.
 *            http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _SEC_G2D_DRIVER_H_
#define _SEC_G2D_DRIVER_H_

#define G2D_MINOR  240

#define G2D_IOCTL_MAGIC 'G'

#define G2D_BLIT             _IO(G2D_IOCTL_MAGIC, 0)
#define G2D_GET_VERSION      _IO(G2D_IOCTL_MAGIC, 1)
#define G2D_GET_MEMORY       _IOR(G2D_IOCTL_MAGIC, 2, unsigned int)
#define G2D_GET_MEMORY_SIZE  _IOR(G2D_IOCTL_MAGIC, 3, unsigned int)
#define G2D_DMA_CACHE_INVAL  _IOWR(G2D_IOCTL_MAGIC, 4, struct g2d_dma_info)
#define G2D_DMA_CACHE_CLEAN  _IOWR(G2D_IOCTL_MAGIC, 5, struct g2d_dma_info)
#define G2D_DMA_CACHE_FLUSH  _IOWR(G2D_IOCTL_MAGIC, 6, struct g2d_dma_info)
#define G2D_SET_MEMORY       _IOWR(G2D_IOCTL_MAGIC, 7, struct g2d_dma_info)

#define G2D_SFR_SIZE        (0x1000)

#define TRUE                (1)
#define FALSE               (0)

#define G2D_MAX_WIDTH       (2048)
#define G2D_MAX_HEIGHT      (2048)

#define G2D_ALPHA_VALUE_MAX (255)

enum G2D_ROT_DEG {
	G2D_ROT_0 = 0,
	G2D_ROT_90,
	G2D_ROT_180,
	G2D_ROT_270,
	G2D_ROT_X_FLIP,
	G2D_ROT_Y_FLIP
};

enum G2D_ALPHA_BLENDING_MODE {
	G2D_ALPHA_BLENDING_MIN    = 0,   /* wholly transparent */
	G2D_ALPHA_BLENDING_MAX    = 255, /* 255 */
	G2D_ALPHA_BLENDING_OPAQUE = 256, /* opaque */
};

enum G2D_COLORKEY_MODE {
	G2D_COLORKEY_NONE = 0,
	G2D_COLORKEY_SRC_ON,
	G2D_COLORKEY_DST_ON,
	G2D_COLORKEY_SRC_DST_ON,
};

enum G2D_BLUE_SCREEN_MODE {
	G2D_BLUE_SCREEN_NONE = 0,
	G2D_BLUE_SCREEN_TRANSPARENT,
	G2D_BLUE_SCREEN_WITH_COLOR,
};

enum G2D_ROP_TYPE {
	G2D_ROP_SRC = 0,
	G2D_ROP_DST,
	G2D_ROP_SRC_AND_DST,
	G2D_ROP_SRC_OR_DST,
	G2D_ROP_3RD_OPRND,
	G2D_ROP_SRC_AND_3RD_OPRND,
	G2D_ROP_SRC_OR_3RD_OPRND,
	G2D_ROP_SRC_XOR_3RD_OPRND,
	G2D_ROP_DST_OR_3RD,
};

enum G2D_THIRD_OP_MODE {
	G2D_THIRD_OP_NONE = 0,
	G2D_THIRD_OP_PATTERN,
	G2D_THIRD_OP_FG,
	G2D_THIRD_OP_BG
};

enum G2D_COLOR {
	G2D_BLACK = 0,
	G2D_RED,
	G2D_GREEN,
	G2D_BLUE,
	G2D_WHITE,
	G2D_YELLOW,
	G2D_CYAN,
	G2D_MAGENTA
};

enum G2D_COLOR_SPACE {
	G2D_RGB_565 = 0,

	G2D_RGBA_8888,
	G2D_ARGB_8888,
	G2D_BGRA_8888,
	G2D_ABGR_8888,

	G2D_RGBX_8888,
	G2D_XRGB_8888,
	G2D_BGRX_8888,
	G2D_XBGR_8888,

	G2D_RGBA_5551,
	G2D_ARGB_1555,
	G2D_BGRA_5551,
	G2D_ABGR_1555,

	G2D_RGBX_5551,
	G2D_XRGB_1555,
	G2D_BGRX_5551,
	G2D_XBGR_1555,

	G2D_RGBA_4444,
	G2D_ARGB_4444,
	G2D_BGRA_4444,
	G2D_ABGR_4444,

	G2D_RGBX_4444,
	G2D_XRGB_4444,
	G2D_BGRX_4444,
	G2D_XBGR_4444,

	G2D_PACKED_RGB_888,
	G2D_PACKED_BGR_888,

	G2D_MAX_COLOR_SPACE
};

struct g2d_rect {
	unsigned int   x;
	unsigned int   y;
	unsigned int   w;
	unsigned int   h;
	unsigned int   full_w;
	unsigned int   full_h;
	int            color_format;
	unsigned int   phys_addr;
	unsigned char *virt_addr;
};

struct g2d_flag {
	unsigned int   rotate_val;
	unsigned int   alpha_val;

	/* true : enable, false : disable */
	unsigned int   blue_screen_mode;

	/* screen color value */
	unsigned int   color_key_val;

	/* one color // RGBA_8888 byte order */
	unsigned int   color_val;

	unsigned int   third_op_mode;
	unsigned int   rop_mode;
	unsigned int   mask_mode;
};

struct g2d_params {
	struct g2d_rect *src_rect;
	struct g2d_rect *dst_rect;
	struct g2d_flag *flag;
};

struct g2d_dma_info {
	unsigned long addr;
	unsigned int  size;
};

/**** function declearation***************************/
static void sec_g2d_init_regs(struct g2d_params *params);
static void sec_g2d_rotate_with_bitblt(struct g2d_params *params);
static u32  sec_g2d_check_fifo_stat_wait(void);
static u32  sec_g2d_set_src_img(struct g2d_rect *src_rect,
				struct g2d_rect *dst_rect,
				struct g2d_flag *flag);
static u32  sec_g2d_set_dst_img(struct g2d_rect *rect);
static u32  sec_g2d_set_rotation(struct g2d_flag *flag);
static u32  sec_g2d_set_clip_win(struct g2d_rect *rect);
static u32  sec_g2d_set_color_key(struct g2d_flag *flag);
static u32  sec_g2d_set_pattern(struct g2d_rect *rect,
				struct g2d_flag *flag);
static u32  sec_g2d_set_alpha(struct g2d_rect *src_rect,
			struct g2d_rect *dst_rect,
			struct g2d_flag *flag);
static void sec_g2d_set_bitblt_cmd(struct g2d_rect *src_rect,
				struct g2d_rect *dst_rect,
				u32 blt_cmd);
static int  sec_g2d_open(struct inode *inode, struct file *file);
static int  sec_g2d_release(struct inode *inode, struct file *file);
static int  sec_g2d_mmap(struct file *filp, struct vm_area_struct *vma);
static int  sec_g2d_ioctl(struct inode *inode,
			struct file *file,
			unsigned int cmd, unsigned long arg);
static u32 sec_g2d_poll(struct file *file, poll_table *wait);

#endif /*_SEC_G2D_DRIVER_H_*/

