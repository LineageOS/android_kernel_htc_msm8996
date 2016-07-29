/* arch/arm/mach-msm/htc_mnemosyne.h
 * Copyright (C) 2013 HTC Corporation.
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
#ifndef __MACH_MNEMOSYNE_H
#define __MACH_MNEMOSYNE_H

#if defined(CONFIG_64BIT)
#define MNEMOSYNE_ELEMENT_TYPE			uint64_t	
#define MNEMOSYNE_ELEMENT_SIZE_BIT_SHIFT	3		
#else
#define MNEMOSYNE_ELEMENT_TYPE			uint32_t	
#define MNEMOSYNE_ELEMENT_SIZE_BIT_SHIFT	2		
#endif
#define MNEMOSYNE_ELEMENT_SIZE			(1<<MNEMOSYNE_ELEMENT_SIZE_BIT_SHIFT)		

#ifdef __ASSEMBLY__
#include <linux/linkage.h>
#include <linux/threads.h>

#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

#define DECLARE_MNEMOSYNE_START()
#define DECLARE_MNEMOSYNE(name)			ASM_ENUM	mnemosyne_##name
#define DECLARE_MNEMOSYNE_ARRAY(name, number)	ASM_ENUM_ARRAY	mnemosyne_##name, number

.SET LAST_ENUM_VALUE, 0

.MACRO ASM_ENUM_ARRAY name, number
.EQUIV \name, LAST_ENUM_VALUE * MNEMOSYNE_ELEMENT_SIZE
.SET LAST_ENUM_VALUE, LAST_ENUM_VALUE + \number
.ENDM

.MACRO ASM_ENUM name
ASM_ENUM_ARRAY \name, 1
.ENDM

#define DECLARE_MNEMOSYNE_END()


	.macro MPIDR2INDEX dst, tmp
	mrs	\tmp, mpidr_el1

	and	\dst, \tmp, #0xff		

	and	\tmp, \tmp, #0xff00		
	add	\dst, \dst, \tmp, lsr #6	
	.endm

	.macro VIRT2PHYS dst, virt, anchor, tmp
	ldr	\dst, =\virt
	adr	\tmp, \anchor
	add	\dst, \dst, \tmp
	ldr	\tmp, =\anchor
	sub	\dst, \dst, \tmp
	.endm

#else
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#define DECLARE_MNEMOSYNE_START()		struct mnemosyne_data {
#define DECLARE_MNEMOSYNE(name)			MNEMOSYNE_ELEMENT_TYPE name;
#define DECLARE_MNEMOSYNE_ARRAY(name, number)	MNEMOSYNE_ELEMENT_TYPE name[number];
#define DECLARE_MNEMOSYNE_END()			};
#endif 

#include <htc_mnemosyne/htc_mnemosyne_footprint.inc>

#ifndef __ASSEMBLY__
#define MNEMOSYNE_SET(f, v)		do {								\
						if (mnemosyne_get_base()) {mnemosyne_get_base()->f = (MNEMOSYNE_ELEMENT_TYPE )(v);}	\
					} while(0);

#define MNEMOSYNE_SET_I(f, i, v)	do {									\
						if (mnemosyne_get_base()) {mnemosyne_get_base()->f[i] = (MNEMOSYNE_ELEMENT_TYPE)(v);}	\
					} while(0);

#define MNEMOSYNE_GET(f)		((mnemosyne_get_base())?mnemosyne_get_base()->f:0)

#define MNEMOSYNE_GET_ADDR(f)		((mnemosyne_get_base())?&mnemosyne_get_base()->f:NULL)

#define MNEMOSYNE_GET_I(f, i)		((mnemosyne_get_base())?mnemosyne_get_base()->f[i]:0)

#define MNEMOSYNE_GET_ADDR_I(f, i)	((mnemosyne_get_base())?&mnemosyne_get_base()->f[i]:NULL)

struct mnemosyne_platform_data {
	u64 phys;
	u64 size;
};

struct mnemosyne_data *mnemosyne_get_base(void);
int mnemosyne_is_ready(void);
int mnemosyne_early_init(void);
#endif 
#endif
