/*
 * HND Run Time Environment debug info area
 *
 * Copyright (C) 1999-2016, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id: hnd_debug.h 561108 2015-06-03 09:20:16Z $
 */

#ifndef	_HND_DEBUG_H
#define	_HND_DEBUG_H

#define HND_DEBUG_PTR_PTR_MAGIC 0x50504244  	

#define HND_RAMSIZE_PTR_MAGIC	0x534d4152	

#ifndef _LANGUAGE_ASSEMBLY



#ifdef FWID
extern uint32 gFWID;
#endif

#define _HD_EVLOG_P	uint32
#define _HD_CONS_P	uint32
#define _HD_TRAP_P	uint32

typedef struct hnd_debug_ptr {
	uint32	magic;

	uint32	hnd_debug_addr;

	
	uint32	ram_base_addr;

} hnd_debug_ptr_t;

typedef struct hnd_ramsize_ptr {
	uint32	magic;			

	
	uint32	ram_size;
} hnd_ramsize_ptr_t;

typedef struct hnd_debug {
	uint32	magic;
#define HND_DEBUG_MAGIC 0x47424544	

	uint32	version;		
#define HND_DEBUG_VERSION 1

	uint32	fwid;			
	char	epivers[32];

	_HD_TRAP_P trap_ptr;		
	_HD_CONS_P console;		

	uint32	ram_base;
	uint32	ram_size;

	uint32	rom_base;
	uint32	rom_size;

	_HD_EVLOG_P event_log_top;

} hnd_debug_t;


typedef struct             {    
	uint32 tv_sec;	
	uint32 tv_usec;	
} timeval_t;


typedef struct prstatus {
	  int32 si_signo; 	
	  int32 si_code; 	
	  int32 si_errno; 	
	  uint16 pr_cursig; 	
	  uint16 unused;
	  uint32 pr_sigpend;	
	  uint32 pr_sighold;	
	  uint32 pr_pid;
	  uint32 pr_ppid;
	  uint32 pr_pgrp;
	  uint32 pr_sid;
	  timeval_t pr_utime;	
	  timeval_t pr_stime;	
	  timeval_t pr_cutime;	
	  timeval_t pr_cstime;	
	  uint32 uregs[18];
	  int32 pr_fpvalid;	
} prstatus_t;

#ifdef __GNUC__
extern hnd_debug_t *get_hnd_debug_info(void);
#endif 

#define DUMP_INFO_PTR_PTR_0   0x74
#define DUMP_INFO_PTR_PTR_1   0x78
#define DUMP_INFO_PTR_PTR_2   0xf0
#define DUMP_INFO_PTR_PTR_3   0xf8
#define DUMP_INFO_PTR_PTR_4   0x874
#define DUMP_INFO_PTR_PTR_5   0x878
#define DUMP_INFO_PTR_PTR_END 0xffffffff
#define DUMP_INFO_PTR_PTR_LIST	DUMP_INFO_PTR_PTR_0, \
		DUMP_INFO_PTR_PTR_1,					\
		DUMP_INFO_PTR_PTR_2,					\
		DUMP_INFO_PTR_PTR_3,					\
		DUMP_INFO_PTR_PTR_4,					\
		DUMP_INFO_PTR_PTR_5,					\
		DUMP_INFO_PTR_PTR_END

#define RAMSIZE_PTR_PTR_0	0x6c
#define RAMSIZE_PTR_PTR_END	0xffffffff
#define RAMSIZE_PTR_PTR_LIST	RAMSIZE_PTR_PTR_0, \
				RAMSIZE_PTR_PTR_END

#endif 

#endif 
