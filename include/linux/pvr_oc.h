/*
* Author: Jason Toney <jt1134@gmail.com>
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
*/


#ifndef _LINUX_PVR_OC_H
#define _LINUX_PVR_OC_H

extern unsigned int pvr_clk_val;

int pvr_oc_init(void);
void pvr_oc_exit(void);

#endif
