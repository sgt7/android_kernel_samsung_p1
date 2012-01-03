/* sound/s5p-rp/s5p-rp_fw.c
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

/* S5PC110 SRP Firmeware */

/* Name */
char rp_fw_name[][10] = {
/*	"Code 0  ",*/
	"Code 1  ",
	"Code 2-0",
	"Code 2-1",
	"Code 2-2",
	"Code 3-0",
	"Code 3-1",
};

/* TEXT 64KBytes */
unsigned long rp_fw_text[][64*1024/4] = {
/*	{
	#include "s5p-rp_fw_code0_text"
	},*/
	{
	#include "s5p-rp_fw_code1_text"
	},
	{
	#include "s5p-rp_fw_code20_text"
	},
	{
	#include "s5p-rp_fw_code21_text"
	},
	{
	#include "s5p-rp_fw_code22_text"
	},
	{
	#include "s5p-rp_fw_code30_text"
	},
	{
	#include "s5p-rp_fw_code31_text"
	},
};

/* DATA 96KBytes */
unsigned long rp_fw_data[][96*1024/4] = {
/*	{
	#include "s5p-rp_fw_code0_data"
	},*/
	{
	#include "s5p-rp_fw_code1_data"
	},
	{
	#include "s5p-rp_fw_code20_data"
	},
	{
	#include "s5p-rp_fw_code21_data"
	},
	{
	#include "s5p-rp_fw_code22_data"
	},
	{
	#include "s5p-rp_fw_code30_data"
	},
	{
	#include "s5p-rp_fw_code31_data"
	},
};

