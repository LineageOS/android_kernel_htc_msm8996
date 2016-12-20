/*
 * Copyright (c) 2013-2014 TRUSTONIC LIMITED
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef MCIFC_H_
#define MCIFC_H_

#include "platform.h"


#define MC_FC_INVALID	((u32)0)  

#if defined(CONFIG_ARM64) && !defined(MC_ARMV7_FC)

#define MC_FC_STD64_BASE	((u32)0xFF000000)
#define MC_FC_STD64(x)	((u32)(MC_FC_STD64_BASE + (x)))

#define MC_FC_INIT	MC_FC_STD64(1)  
#define MC_FC_INFO	MC_FC_STD64(2)  
#define MC_FC_MEM_TRACE	MC_FC_STD64(10)  
#define MC_FC_SWAP_CPU	MC_FC_STD64(54)  

#else

#define MC_FC_INIT	((u32)(-1))  
#define MC_FC_INFO	((u32)(-2))  
#define MC_FC_MEM_TRACE	((u32)(-31))  
#define MC_FC_SWAP_CPU	((u32)(0x84000005))  

#endif


#define MC_SMC_N_YIELD			3
#define MC_SMC_N_SIQ			4

#define MC_STATUS_NOT_INITIALIZED	0
#define MC_STATUS_BAD_INIT		1
#define MC_STATUS_INITIALIZED		2
#define MC_STATUS_HALT			3

#define MC_EXT_INFO_ID_MCI_VERSION	0
#define MC_EXT_INFO_ID_FLAGS		1
#define MC_EXT_INFO_ID_HALT_CODE	2
#define MC_EXT_INFO_ID_HALT_IP		3
#define MC_EXT_INFO_ID_FAULT_CNT	4
#define MC_EXT_INFO_ID_FAULT_CAUSE	5
#define MC_EXT_INFO_ID_FAULT_META	6
#define MC_EXT_INFO_ID_FAULT_THREAD	7
#define MC_EXT_INFO_ID_FAULT_IP		8
#define MC_EXT_INFO_ID_FAULT_SP		9
#define MC_EXT_INFO_ID_FAULT_ARCH_DFSR	10
#define MC_EXT_INFO_ID_FAULT_ARCH_ADFSR	11
#define MC_EXT_INFO_ID_FAULT_ARCH_DFAR	12
#define MC_EXT_INFO_ID_FAULT_ARCH_IFSR	13
#define MC_EXT_INFO_ID_FAULT_ARCH_AIFSR	14
#define MC_EXT_INFO_ID_FAULT_ARCH_IFAR	15
#define MC_EXT_INFO_ID_MC_CONFIGURED	16
#define MC_EXT_INFO_ID_MC_SCHED_STATUS	17
#define MC_EXT_INFO_ID_MC_STATUS	18
#define MC_EXT_INFO_ID_MC_EXC_PARTNER	19
#define MC_EXT_INFO_ID_MC_EXC_IPCPEER	20
#define MC_EXT_INFO_ID_MC_EXC_IPCMSG	21
#define MC_EXT_INFO_ID_MC_EXC_IPCDATA	22
#define MC_EXT_INFO_ID_MC_EXC_UUID	23
#define MC_EXT_INFO_ID_MC_EXC_UUID1	24
#define MC_EXT_INFO_ID_MC_EXC_UUID2	25
#define MC_EXT_INFO_ID_MC_EXC_UUID3	26


#define MC_FC_RET_OK				0
#define MC_FC_RET_ERR_INVALID			1
#define MC_FC_RET_ERR_ALREADY_INITIALIZED	5

#define MC_FC_INIT_FLAG_LPAE			BIT(0)

#endif 

