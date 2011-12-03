/* sound/s5p-rp/s5p-rp_fw.h
 *
 * SRP Audio Firmware for Samsung s5pc110
 *
 * Copyright (c) 2010 Samsung Electronics
 * 	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _RP_FW_H_
#define _RP_FW_H_

#if 0
/* Total 7 Firmwares are required */
#define RP_FW_CODE0		0
#define RP_FW_CODE1		1
#define RP_FW_CODE20		2
#define RP_FW_CODE21		3
#define RP_FW_CODE22		4
#define RP_FW_CODE30		5
#define RP_FW_CODE31		6
#define RP_FW_CODE_MAX		7
#else
/* Total 6 Firmwares are required */
#define RP_FW_CODE0		0
#define RP_FW_CODE1		0
#define RP_FW_CODE20		1
#define RP_FW_CODE21		2
#define RP_FW_CODE22		3
#define RP_FW_CODE30		4
#define RP_FW_CODE31		5
#define RP_FW_CODE_MAX		6
#endif

extern char rp_fw_name[][10];
extern unsigned long rp_fw_text[][64*1024/4];
extern unsigned long rp_fw_data[][96*1024/4];

#endif
