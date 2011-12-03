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

#include <linux/string.h>
#include <linux/fb.h>
#include "s3c_bc.h"
#include "img_defs.h"

IMG_UINT32 g_screen_w = 0, g_screen_h = 0;
IMG_UINT32 g_screen_stride, g_buffer_size = 0;


static void *gpvAnchor = NULL;
static PFN_BC_GET_PVRJTABLE pfnGetPVRJTable = IMG_NULL;

static S3C_BC_DEVINFO * GetAnchorPtr(void)
{
	return (S3C_BC_DEVINFO *)gpvAnchor;
}

static void SetAnchorPtr(S3C_BC_DEVINFO *psDevInfo)
{
	gpvAnchor = (void *)psDevInfo;
}

static PVRSRV_ERROR OpenBCDevice(IMG_UINT32 ui32DeviceID, IMG_HANDLE *phDevice)
{
	S3C_BC_DEVINFO *psDevInfo;

	psDevInfo = GetAnchorPtr();


	*phDevice = (IMG_HANDLE)psDevInfo;

	return (PVRSRV_OK);
}

static PVRSRV_ERROR CloseBCDevice(IMG_UINT32 ui32DeviceID, IMG_HANDLE hDevice)
{	
	return (PVRSRV_OK);
}

static PVRSRV_ERROR GetBCBuffer(IMG_HANDLE          hDevice,
                                IMG_UINT32          ui32BufferNumber,
                                PVRSRV_SYNC_DATA   *psSyncData,
                                IMG_HANDLE         *phBuffer)
{
	S3C_BC_DEVINFO	*psDevInfo;

	if(!hDevice || !phBuffer)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (S3C_BC_DEVINFO*)hDevice;

	if( ui32BufferNumber < S3C_BC_DEVICE_BUFFER_COUNT )
	{
		psDevInfo->sSystemBuffer[ui32BufferNumber].psSyncData = psSyncData;
		*phBuffer = (IMG_HANDLE)&psDevInfo->sSystemBuffer[ui32BufferNumber];
	}
	else
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	return (PVRSRV_OK);
}

static PVRSRV_ERROR GetBCInfo(IMG_HANDLE hDevice, BUFFER_INFO *psBCInfo)
{
	S3C_BC_DEVINFO	*psDevInfo;

	if(!hDevice || !psBCInfo)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (S3C_BC_DEVINFO*)hDevice;

	*psBCInfo = psDevInfo->sBufferInfo;

	return (PVRSRV_OK);
}

static PVRSRV_ERROR GetBCBufferAddr(IMG_HANDLE      hDevice,
                                    IMG_HANDLE      hBuffer,
                                    IMG_SYS_PHYADDR **ppsSysAddr,
                                    IMG_UINT32      *pui32ByteSize,
                                    IMG_VOID        **ppvCpuVAddr,
                                    IMG_HANDLE      *phOSMapInfo,
                                    IMG_BOOL        *pbIsContiguous,
                                    IMG_UINT32      *pui32TilingStride)
{
	S3C_BC_BUFFER *psBuffer;

	PVR_UNREFERENCED_PARAMETER(pui32TilingStride);
	
	if(!hDevice || !hBuffer || !ppsSysAddr || !pui32ByteSize)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psBuffer = (S3C_BC_BUFFER *) hBuffer;

	*ppsSysAddr = &psBuffer->sSysAddr;
	*pui32ByteSize = g_buffer_size;
	*ppvCpuVAddr = 0;
	*phOSMapInfo = 0;
	*pbIsContiguous = IMG_TRUE;

	return (PVRSRV_OK);
}

S3C_BC_ERROR S3C_BC_Register(void)
{
	S3C_BC_DEVINFO	*psDevInfo;
	int i;
	unsigned long addr;

	IMG_UINT32 pixelformat;

	struct fb_info *fbinfo = registered_fb[0];

	g_screen_w = (int)fbinfo->var.xres;
	g_screen_h = (int)fbinfo->var.yres;

	if ( g_screen_w < g_screen_h )
	{
		IMG_UINT32 t = g_screen_w;
		g_screen_w = g_screen_h;
		g_screen_h = t;
	}

	if (16 == (int)fbinfo->var.bits_per_pixel)
	{
		pixelformat = PVRSRV_PIXEL_FORMAT_RGB565;
		g_screen_stride = g_screen_w*2;
	}
	else
	{
		pixelformat = PVRSRV_PIXEL_FORMAT_ARGB8888;
		g_screen_stride = g_screen_w*4;
	}

	g_buffer_size = ((g_screen_h * g_screen_stride + S3C_BC_DEVICE_PHYS_PAGE_SIZE - 1) & ~(S3C_BC_DEVICE_PHYS_PAGE_SIZE-1));

	psDevInfo = GetAnchorPtr();

	if (!psDevInfo)
	{
		psDevInfo = (S3C_BC_DEVINFO *)BCAllocKernelMem(sizeof(S3C_BC_DEVINFO));

		if(!psDevInfo)
		{
			return (S3C_BC_ERROR_OUT_OF_MEMORY);/* failure */
		}

		SetAnchorPtr((void*)psDevInfo);

		psDevInfo->ulRefCount = 0;
		
		psDevInfo->ulDeviceID = S3C_BC_DEVICE_ID;
	
		if(BCGetLibFuncAddr ("PVRGetBufferClassJTable", &pfnGetPVRJTable) != S3C_BC_OK)
		{
			return (S3C_BC_ERROR_INIT_FAILURE);
		}

		if(!(*pfnGetPVRJTable)(&psDevInfo->sPVRJTable))
		{
			return (S3C_BC_ERROR_INIT_FAILURE);
		}


		psDevInfo->sBufferInfo.pixelformat        = pixelformat;
		psDevInfo->sBufferInfo.ui32Width          = g_screen_w;
		psDevInfo->sBufferInfo.ui32Height         = g_screen_h;
		psDevInfo->sBufferInfo.ui32ByteStride     = g_screen_stride;
		psDevInfo->sBufferInfo.ui32BufferDeviceID = S3C_BC_DEVICE_ID;
		psDevInfo->sBufferInfo.ui32Flags          = 0;
		psDevInfo->sBufferInfo.ui32BufferCount    = S3C_BC_DEVICE_BUFFER_COUNT;

		addr = S3C_BC_DEVICE_PHYS_ADDR_START;
		for(i = 0; i < S3C_BC_DEVICE_BUFFER_COUNT; i++, addr += g_buffer_size)
		{
			psDevInfo->sSystemBuffer[i].sSysAddr.uiAddr = (IMG_UINTPTR_T)addr;
		}

		psDevInfo->sBCJTable.ui32TableSize    = sizeof(PVRSRV_BC_SRV2BUFFER_KMJTABLE);
		psDevInfo->sBCJTable.pfnOpenBCDevice  = OpenBCDevice;
		psDevInfo->sBCJTable.pfnCloseBCDevice = CloseBCDevice;
		psDevInfo->sBCJTable.pfnGetBCBuffer   = GetBCBuffer;
		psDevInfo->sBCJTable.pfnGetBCInfo     = GetBCInfo;
		psDevInfo->sBCJTable.pfnGetBufferAddr = GetBCBufferAddr;

		if(psDevInfo->sPVRJTable.pfnPVRSRVRegisterBCDevice (&psDevInfo->sBCJTable,
										(IMG_UINT32*)&psDevInfo->ulDeviceID ) != PVRSRV_OK)
		{
			return (S3C_BC_ERROR_DEVICE_REGISTER_FAILED);
		}
	}

	psDevInfo->ulRefCount++;

	return (S3C_BC_OK);
}

S3C_BC_ERROR S3C_BC_Unregister(void)
{
	S3C_BC_DEVINFO *psDevInfo;

	psDevInfo = GetAnchorPtr();

	if (!psDevInfo)
	{
		return (S3C_BC_ERROR_GENERIC);
	}

	psDevInfo->ulRefCount--;

	if (psDevInfo->ulRefCount == 0)
	{
		PVRSRV_BC_BUFFER2SRV_KMJTABLE	*psJTable = &psDevInfo->sPVRJTable;

		if (psJTable->pfnPVRSRVRemoveBCDevice(psDevInfo->ulDeviceID) != PVRSRV_OK)
		{
			return (S3C_BC_ERROR_GENERIC);
		}

		BCFreeKernelMem(psDevInfo);

		SetAnchorPtr(0);
	}

	return (S3C_BC_OK);
}

S3C_BC_ERROR S3C_BC_Buffer_Info(int devid, BUFFER_INFO* buf )
{
	S3C_BC_DEVINFO *psDevInfo;

	psDevInfo = GetAnchorPtr();
	*buf = psDevInfo->sBufferInfo;
	return 0;
}
