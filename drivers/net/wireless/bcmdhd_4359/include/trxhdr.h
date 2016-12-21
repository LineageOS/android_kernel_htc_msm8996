/*
 * TRX image file header format.
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
 * $Id: trxhdr.h 520026 2014-12-10 01:29:40Z $
 */

#ifndef _TRX_HDR_H
#define _TRX_HDR_H

#include <typedefs.h>

#define TRX_MAGIC	0x30524448	
#define TRX_MAX_LEN	0x3B0000	
#define TRX_NO_HEADER	1		
#define TRX_GZ_FILES	0x2     
#define TRX_EMBED_UCODE	0x8	
#define TRX_ROMSIM_IMAGE	0x10	
#define TRX_UNCOMP_IMAGE	0x20	
#define TRX_BOOTLOADER		0x40	

#define TRX_V1		1
#define TRX_V1_MAX_OFFSETS	3		

#ifndef BCMTRXV2
#define TRX_VERSION	TRX_V1		
#define TRX_MAX_OFFSET TRX_V1_MAX_OFFSETS
#endif

struct trx_header {
	uint32 magic;		
	uint32 len;		
	uint32 crc32;		
	uint32 flag_version;	
#ifndef BCMTRXV2
	uint32 offsets[TRX_MAX_OFFSET];	
#else
	uint32 offsets[1];	
#endif
};

#ifdef BCMTRXV2
#define TRX_VERSION		TRX_V2		
#define TRX_MAX_OFFSET  TRX_V2_MAX_OFFSETS

#define TRX_V2		2
#define TRX_V2_MAX_OFFSETS	5
#define SIZEOF_TRXHDR_V1	(sizeof(struct trx_header)+(TRX_V1_MAX_OFFSETS-1)*sizeof(uint32))
#define SIZEOF_TRXHDR_V2	(sizeof(struct trx_header)+(TRX_V2_MAX_OFFSETS-1)*sizeof(uint32))
#define TRX_VER(trx)		((trx)->flag_version>>16)
#define ISTRX_V1(trx)		(TRX_VER(trx) == TRX_V1)
#define ISTRX_V2(trx)		(TRX_VER(trx) == TRX_V2)
#define SIZEOF_TRX(trx)	    (ISTRX_V2(trx) ? SIZEOF_TRXHDR_V2: SIZEOF_TRXHDR_V1)
#else
#define SIZEOF_TRX(trx)	    (sizeof(struct trx_header))
#endif 

typedef struct trx_header TRXHDR, *PTRXHDR;

#endif 
