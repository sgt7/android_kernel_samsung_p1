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

#ifndef __S3C_BC_LINUX_H__
#define __S3C_BC_LINUX_H__

#include <linux/ioctl.h>



typedef struct S3C_BC_ioctl_package_TAG
{
	int inputparam;
	int outputparam;
} S3C_BC_ioctl_package, *PS3C_BC_ioctl_package;

typedef struct S3C_BC_Buffer_info_TAG
{
	int 			ui32BufferCount;
	int			pixelformat;
	int			ui32ByteStride;
	int			ui32Width;
	int			ui32Height;
}S3C_BC_Buffer_info_t, *PS3C_BC_Buffer_info_t;


#define S3C_BC_IOC_GID      'g'

#define S3C_BC_IOWR(INDEX)  _IOWR(S3C_BC_IOC_GID, INDEX, S3C_BC_ioctl_package)

#define S3C_BC_ioctl_get_physical_base_address		S3C_BC_IOWR(0)
#define S3C_BC_ioctl_get_buffer_info			S3C_BC_IOWR(1)

#endif 

