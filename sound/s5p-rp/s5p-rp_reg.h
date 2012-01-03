/* sound/s5p-rp/s5p-rp_reg.h
 *
 * Audio RP Registers for Samsung s5pc110
 *
 * Copyright (c) 2010 Samsung Electronics
 * http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _RP_REG_H_
#define _RP_REG_H_


#define RP_SRAM_BASE			(0xC0000000)
#define RP_BUF_BASE			(0xEEA00000)
#define RP_COMMBOX_BASE			(0xEEE20000)
#define RP_ASSCLK_BASE			(0xEEE10000)

/*
 * SRAM & Commbox base address
 */

#define RP_IMEM_ADDR			(RP_SRAM_BASE + 0x00000000)
#define RP_DMEM_ARM_ADDR		(RP_SRAM_BASE + 0x00010000)
#define RP_DMEM_RP_ADDR			(RP_SRAM_BASE + 0x00100000)

#define RP_IBUF0_ADDR			(RP_BUF_BASE  + 0x00000000)
#define RP_IBUF1_ADDR			(RP_BUF_BASE  + 0x00100000)
#define RP_OBUF0_ADDR			(RP_BUF_BASE  + 0x00200000)
#define RP_OBUF1_ADDR			(RP_BUF_BASE  + 0x00300000)

#define RP_COMMBOX_ADDR			(RP_COMMBOX_BASE + 0x00000)

#define RP_ASSCLK_SRC_ADDR		(RP_ASSCLK_BASE + 0x00000)
#define RP_ASSCLK_DIV_ADDR		(RP_ASSCLK_BASE + 0x00004)
#define RP_ASSCLK_GATE_ADDR		(RP_ASSCLK_BASE + 0x00008)


/*
 * Commbox Offset
 */

#define RP_INTERRUPT			(0x0000)
#define RP_INTERRUPT_CODE		(0x0004)
#define RP_INFORMATION			(0x0008)
#define RP_FRAME_INDEX			(0x000C)
#define RP_ERROR_CODE			(0x0010)
#define RP_INST_START			(0x0014)
#define RP_EFFECT_DEF			(0x0018)
#define RP_RESET			(0x0100)
#define RP_PENDING			(0x0104)
#define RP_FRAME_SIZE			(0x0108)
#define RP_SW_DEF			(0x010C)
#define RP_GAIN_FACTOR			(0x0110)
#define RP_PCM_ADDR0			(0x0114)
#define RP_PCM_ADDR1			(0x0118)
#define RP_IN_BUFF0			(0x011C)
#define RP_IN_BUFF1			(0x0120)
#define RP_IN_BUFF_SIZE			(0x0124)
#define RP_BITSTREAM_SIZE		(0x0128)
#define RP_BOOT				(0x012C)
#define RP_READ_BITSTREAM_SIZE		(0x0130)
#define RP_EQ_USER_DEF			(0x0134)
#define RP_PCM_TYPE			(0x0138)
#define RP_PAD_PDN_CTRL			(0x0204)
#define RP_MISC				(0x0208)


/*
 * Interrupt Code & Information
 */

#define RP_INTR_CODE_MASK		(0x00FF)
#define RP_INTR_CODE_PLAYDONE		(0x01 << 0)
#define RP_INTR_CODE_PLAYERRDONE	(0x01 << 1)
#define RP_INTR_CODE_REQUEST		(0x01 << 2)
#define RP_INTR_CODE_INFORMATION	(0x01 << 3)
#define RP_INTR_CODE_PLAYSTART		(0x01 << 4)
#define RP_INTR_CODE_EFFECTADDR		(0x01 << 5)
#define RP_INTR_CODE_IBUFSYNC		(0x01 << 6)
#define RP_INTR_CODE_REQUARTCODE	(0x01 << 7)
#define RP_INTR_CODE_POLLINGWAIT	(0x01 << 9)

#define RP_INTR_INFO_MASK		(0xFFFF)
#define RP_INTR_INFO_DATAEMPTY		(0x01 << 0)
#define RP_INTR_INFO_OBUF1SYNC		(0x01 << 1)
#define RP_INTR_INFO_OBUF2SYNC		(0x01 << 2)
#define RP_INTR_INFO_I2SPAUSE		(0x01 << 3)
#define RP_INTR_INFO_I2SCONTINUE	(0x01 << 4)

#endif
