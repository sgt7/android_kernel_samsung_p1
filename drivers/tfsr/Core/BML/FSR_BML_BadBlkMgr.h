/**
 *   @mainpage   Flex Sector Remapper : LinuStoreIII_1.2.0_b038-FSR_1.2.1p1_b139_RC
 *
 *   @section Intro Intro
 *       Flash Translation Layer for Flex-OneNAND and OneNAND
 *   
 *     @MULTI_BEGIN@ @COPYRIGHT_DEFAULT
 *     @section Copyright COPYRIGHT_DEFAULT
 *            COPYRIGHT. SAMSUNG ELECTRONICS CO., LTD.
 *                                    ALL RIGHTS RESERVED
 *     Permission is hereby granted to licensees of Samsung Electronics Co., Ltd. products
 *     to use this computer program only in accordance 
 *     with the terms of the SAMSUNG FLASH MEMORY DRIVER SOFTWARE LICENSE AGREEMENT.
 *     @MULTI_END@
 *
 *     @MULTI_BEGIN@ @COPYRIGHT_GPL
 *     @section Copyright COPYRIGHT_GPL
 *            COPYRIGHT. SAMSUNG ELECTRONICS CO., LTD.
 *                                    ALL RIGHTS RESERVED
 *     This program is free software; you can redistribute it and/or modify it
 *     under the terms of the GNU General Public License version 2 
 *     as published by the Free Software Foundation.
 *     @MULTI_END@
 *
 *     @section Description
 *
 */

/**
 * @file      FSR_BML_BadBlkMgr.h
 * @brief     This file contains the definition and prototypes of exported
 * @n         functions for Bad Block Manager 
 * @author    MinYoung Kim
 * @date      15-JAN-2007
 * @remark
 * REVISION HISTORY
 * @n  15-JAN-2007 [MinYoung Kim] : first writing
 *
 */

#ifndef     _FSR_BML_BADBLKMGR_H_
#define     _FSR_BML_BADBLKMGR_H_

/*****************************************************************************/
/* property of data                                                          */
/*****************************************************************************/
#define     BML_META_DATA       (1)
#define     BML_USER_DATA       (2)

/*****************************************************************************/
/* exported function prototype of Bad Block Manager                          */
/*****************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if !defined(FSR_NBL2)
INT32  FSR_BBM_Format               (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev,
                                     FSRPartI    *pstPart);
INT32  FSR_BBM_Repartition          (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev,
                                     FSRPartI    *pstPart);
INT32  FSR_BBM_UpdatePIExt          (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev,
                                     FSRPIExt    *pstPExt);
VOID   FSR_BBM_PrintBMI             (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev);
VOID   FSR_BBM_GetBMI               (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev,
                                     UINT32       nDieIdx,
                                     BmlBMF      *pstBMF,
                                     UINT32      *pnNumOfBMF,
                                     UINT16      *pstRCB,
                                     UINT32      *pnNumOfRCB,
                                     UINT32       nFlag);
#endif /* FSR_NBL2 */

INT32  FSR_BBM_HandleBadBlk         (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev,
                                     UINT32       nSbn,
                                     UINT32       nPgOff,
                                     UINT32       nNumOfBadBlks,
                                     UINT32       nFlag);
VOID   FSR_BBM_WaitUntilPowerDn     (VOID);
VOID   FSR_BBM_SetWaitTimeForErError(UINT32       nNMSec);
INT32  FSR_BBM_RefreshByErase       (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev,
                                     UINT32       nDieIdx,
                                     UINT32       nFlag);
INT32  FSR_BBM_ChkRefreshBlk        (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev,
                                     UINT32       nDieIdx);
INT32  FSR_BBM_UpdateERL            (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev,
                                     UINT32       nDieIdx,
                                     UINT32       nOrgSbn,
                                     UINT32       nFlag);
INT32  FSR_BBM_FixSLCBoundary       (BmlVolCxt   *pstVol,
                                     UINT32       nPDev,
                                     UINT32       nDieIdx);
INT32  FSR_BBM_LockTight            (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev,
                                     UINT32       nDieIdx);
INT32  FSR_BBM_BackupPrevData       (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev,
                                     UINT32       nDieIdx,
                                     UINT8       *pMBuf,
                                     FSRSpareBuf *pSBuf,
                                     UINT32       nFlag);
INT32  FSR_BBM_RestorePrevData      (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev,
                                     UINT32       nDieIdx,
                                     UINT8       *pMBuf,
                                     FSRSpareBuf *pSBuf,
                                     UINT32       nFlag);
INT32  FSR_BBM_EraseREFBlk          (BmlVolCxt   *pstVol,
                                     BmlDevCxt   *pstDev,
                                     UINT32       nDieIdx);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _FSR_BML_BADBLKMGR_H_ */
