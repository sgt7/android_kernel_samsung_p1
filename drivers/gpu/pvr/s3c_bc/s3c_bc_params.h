/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 * 		Samsung Electronics System LSI. modify
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
******************************************************************************/

#include <plat/media.h>
#include <mach/media.h>

#define S3C_BC_DEVICE_NAME					"s3c_bc"
#define S3C_BC_DEVICE_ID					0
#define S3C_BC_DEVICE_BUFFER_COUNT			2								/* TODO: Modify this accordingly. */
#define S3C_BC_DEVICE_PHYS_PAGE_SIZE		0x1000							/* 4KB */
#define S3C_BC_DEVICE_PHYS_ADDR_START		((unsigned int)s5p_get_media_memory_bank(S5P_MDEV_TEXSTREAM,0)) 
