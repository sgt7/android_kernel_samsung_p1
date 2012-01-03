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

#ifndef __S3C_BC_H__
#define __S3C_BC_H__

#include "img_defs.h"
#include "servicesext.h"
#include "kernelbuffer.h"
#include "s3c_bc_params.h"

#if defined(__cplusplus)
extern "C" {
#endif

extern IMG_IMPORT IMG_BOOL PVRGetBufferClassJTable(PVRSRV_BC_BUFFER2SRV_KMJTABLE *psJTable);

typedef struct S3C_BC_BUFFER_TAG
{
	IMG_SYS_PHYADDR			sSysAddr;

	PVRSRV_SYNC_DATA		*psSyncData;
} S3C_BC_BUFFER;

typedef struct S3C_BC_DEVINFO_TAG
{
	unsigned long           ulDeviceID;

	S3C_BC_BUFFER			sSystemBuffer[S3C_BC_DEVICE_BUFFER_COUNT];

	PVRSRV_BC_BUFFER2SRV_KMJTABLE sPVRJTable;

	PVRSRV_BC_SRV2BUFFER_KMJTABLE sBCJTable;

	unsigned long           ulRefCount;

	BUFFER_INFO             sBufferInfo;

}  S3C_BC_DEVINFO;

typedef enum _S3C_BC_ERROR_
{
	S3C_BC_OK                             =  0,
	S3C_BC_ERROR_GENERIC                  =  1,
	S3C_BC_ERROR_OUT_OF_MEMORY            =  2,
	S3C_BC_ERROR_TOO_FEW_BUFFERS          =  3,
	S3C_BC_ERROR_INVALID_PARAMS           =  4,
	S3C_BC_ERROR_INIT_FAILURE             =  5,
	S3C_BC_ERROR_CANT_REGISTER_CALLBACK   =  6,
	S3C_BC_ERROR_INVALID_DEVICE           =  7,
	S3C_BC_ERROR_DEVICE_REGISTER_FAILED   =  8,
	S3C_BC_ERROR_NO_PRIMARY               =  9
} S3C_BC_ERROR;

S3C_BC_ERROR S3C_BC_Register(void);
S3C_BC_ERROR S3C_BC_Unregister(void);

void *BCAllocKernelMem(unsigned long ulSize);
void BCFreeKernelMem(void *pvMem);

S3C_BC_ERROR BCGetLibFuncAddr (char *szFunctionName, PFN_BC_GET_PVRJTABLE *ppfnFuncTable);
S3C_BC_ERROR S3C_BC_Buffer_Info(int devid, BUFFER_INFO* buf );

#if defined(__cplusplus)
}
#endif

#endif

