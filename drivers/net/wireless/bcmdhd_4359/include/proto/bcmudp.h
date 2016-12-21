/*
 * Fundamental constants relating to UDP Protocol
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
 * $Id: bcmudp.h 597933 2015-11-06 18:52:06Z $
 */

#ifndef _bcmudp_h_
#define _bcmudp_h_

#ifndef _TYPEDEFS_H_
#include <typedefs.h>
#endif

#include <packed_section_start.h>


#define UDP_DEST_PORT_OFFSET	2	
#define UDP_LEN_OFFSET		4	
#define UDP_CHKSUM_OFFSET	6	

#define UDP_HDR_LEN	8	
#define UDP_PORT_LEN	2	

BWL_PRE_PACKED_STRUCT struct bcmudp_hdr
{
	uint16	src_port;	
	uint16	dst_port;	
	uint16	len;		
	uint16	chksum;		
} BWL_POST_PACKED_STRUCT;

#include <packed_section_end.h>

#endif	
