/* linux/arch/arm/mach-s5pv210/include/mach/memory.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5PV210 - Memory definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#if defined(CONFIG_MACH_SMDKV210)
#define PHYS_OFFSET		UL(0x20000000)
#else
#define PHYS_OFFSET		UL(0x30000000)
#define END_MEM			UL(0x58000000)
#endif

#define CONSISTENT_DMA_SIZE	(SZ_8M + SZ_4M + SZ_2M)

/* Maximum of 256MiB in one bank */
#define MAX_PHYSMEM_BITS	32
#define SECTION_SIZE_BITS	28
#define NODE_MEM_SIZE_BITS   28

#if defined(CONFIG_KEXEC_HARDBOOT) && !defined(CONFIG_MACH_SMDKV210)
#define KEXEC_HB_PAGE_ADDR 0x57fff000
#endif

#endif /* __ASM_ARCH_MEMORY_H */
