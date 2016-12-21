/*
 * Fundamental constants relating to 802.3
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
 * $Id: 802.3.h 518342 2014-12-01 23:21:41Z $
 */

#ifndef _802_3_h_
#define _802_3_h_

#include <packed_section_start.h>

#define SNAP_HDR_LEN	6	
#define DOT3_OUI_LEN	3	

BWL_PRE_PACKED_STRUCT struct dot3_mac_llc_snap_header {
	uint8	ether_dhost[ETHER_ADDR_LEN];	
	uint8	ether_shost[ETHER_ADDR_LEN];	
	uint16	length;				
	uint8	dsap;				
	uint8	ssap;				
	uint8	ctl;				
	uint8	oui[DOT3_OUI_LEN];		
	uint16	type;				
} BWL_POST_PACKED_STRUCT;

#include <packed_section_end.h>

#endif	
