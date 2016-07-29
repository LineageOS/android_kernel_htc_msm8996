/*
 * Fundamental constants relating to DHCP Protocol
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
 * $Id: bcmdhcp.h 597933 2015-11-06 18:52:06Z $
 */

#ifndef _bcmdhcp_h_
#define _bcmdhcp_h_

#define DHCP_TYPE_OFFSET	0	
#define DHCP_TID_OFFSET		4	
#define DHCP_FLAGS_OFFSET	10	
#define DHCP_CIADDR_OFFSET	12	
#define DHCP_YIADDR_OFFSET	16	
#define DHCP_GIADDR_OFFSET	24	
#define DHCP_CHADDR_OFFSET	28	
#define DHCP_OPT_OFFSET		236	

#define DHCP_OPT_MSGTYPE	53	
#define DHCP_OPT_MSGTYPE_REQ	3
#define DHCP_OPT_MSGTYPE_ACK	5	

#define DHCP_OPT_CODE_OFFSET	0	
#define DHCP_OPT_LEN_OFFSET	1	
#define DHCP_OPT_DATA_OFFSET	2	

#define DHCP_OPT_CODE_CLIENTID	61	

#define DHCP_TYPE_REQUEST	1	
#define DHCP_TYPE_REPLY		2	

#define DHCP_PORT_SERVER	67	
#define DHCP_PORT_CLIENT	68	

#define DHCP_FLAG_BCAST	0x8000	

#define DHCP_FLAGS_LEN	2	

#define DHCP6_TYPE_SOLICIT	1	
#define DHCP6_TYPE_ADVERTISE	2	
#define DHCP6_TYPE_REQUEST	3	
#define DHCP6_TYPE_CONFIRM	4	
#define DHCP6_TYPE_RENEW	5	
#define DHCP6_TYPE_REBIND	6	
#define DHCP6_TYPE_REPLY	7	
#define DHCP6_TYPE_RELEASE	8	
#define DHCP6_TYPE_DECLINE	9	
#define DHCP6_TYPE_RECONFIGURE	10	
#define DHCP6_TYPE_INFOREQ	11	
#define DHCP6_TYPE_RELAYFWD	12	
#define DHCP6_TYPE_RELAYREPLY	13	

#define DHCP6_TYPE_OFFSET	0	

#define	DHCP6_MSG_OPT_OFFSET	4	
#define	DHCP6_RELAY_OPT_OFFSET	34	

#define	DHCP6_OPT_CODE_OFFSET	0	
#define	DHCP6_OPT_LEN_OFFSET	2	
#define	DHCP6_OPT_DATA_OFFSET	4	

#define	DHCP6_OPT_CODE_CLIENTID	1	
#define	DHCP6_OPT_CODE_SERVERID	2	

#define DHCP6_PORT_SERVER	547	
#define DHCP6_PORT_CLIENT	546	

#endif	
