/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ASM_D2S_FAULT_H
#define __ASM_D2S_FAULT_H

#include <linux/types.h>

/*
 * d2s bring-up: snapshot of the most recent kernel fault, captured in
 * die_kernel_fault() and read back when the init process dies, so the
 * panic site can print the registers of the fault that killed it.
 */
struct d2s_fault_info {
	unsigned long addr;
	unsigned long esr;
	unsigned long pc;
	unsigned long lr;
	unsigned long x0, x1, x2, x3;
	unsigned long x4, x5, x6, x7;
};

bool d2s_fault_get_recent(struct d2s_fault_info *info);

#endif
