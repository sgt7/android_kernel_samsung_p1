/*
 * Copyright (C) 2010 Samsung Electronics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_SEC_SWITCH_H
#define __ASM_ARCH_SEC_SWITCH_H

struct sec_switch_platform_data {
	int (*get_regulator) (void);
	void (*set_regulator) (int mode);
	int (*get_cable_status) (void);
	int (*get_phy_init_status) (void);
	void (*set_switch_status) (int val);
};

#define SWITCH_MODEM	0
#define SWITCH_PDA	1

enum {
	AP_VBUS_ON = 0,
	CP_VBUS_ON,
	AP_VBUS_OFF,
};

#endif
