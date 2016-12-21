/*
 * Common stats definitions for clients of dongle
 * ports
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
 * $Id: dngl_stats.h 523030 2014-12-25 17:28:07Z $
 */

#ifndef _dngl_stats_h_
#define _dngl_stats_h_

#include <proto/ethernet.h>
#include <proto/802.11.h>

typedef struct {
	unsigned long	rx_packets;		
	unsigned long	tx_packets;		
	unsigned long	rx_bytes;		
	unsigned long	tx_bytes;		
	unsigned long	rx_errors;		
	unsigned long	tx_errors;		
	unsigned long	rx_dropped;		
	unsigned long	tx_dropped;		
	unsigned long   multicast;      
} dngl_stats_t;

typedef int32 wifi_radio;
typedef int32 wifi_channel;
typedef int32 wifi_rssi;
typedef struct { uint16 version; uint16 length; } ver_len;

typedef enum wifi_channel_width {
	WIFI_CHAN_WIDTH_20	  = 0,
	WIFI_CHAN_WIDTH_40	  = 1,
	WIFI_CHAN_WIDTH_80	  = 2,
	WIFI_CHAN_WIDTH_160   = 3,
	WIFI_CHAN_WIDTH_80P80 = 4,
	WIFI_CHAN_WIDTH_5	  = 5,
	WIFI_CHAN_WIDTH_10	  = 6,
	WIFI_CHAN_WIDTH_INVALID = -1
} wifi_channel_width_t;

typedef enum {
	WIFI_DISCONNECTED = 0,
	WIFI_AUTHENTICATING = 1,
	WIFI_ASSOCIATING = 2,
	WIFI_ASSOCIATED = 3,
	WIFI_EAPOL_STARTED = 4,   
	WIFI_EAPOL_COMPLETED = 5, 
} wifi_connection_state;

typedef enum {
	WIFI_ROAMING_IDLE = 0,
	WIFI_ROAMING_ACTIVE = 1
} wifi_roam_state;

typedef enum {
	WIFI_INTERFACE_STA = 0,
	WIFI_INTERFACE_SOFTAP = 1,
	WIFI_INTERFACE_IBSS = 2,
	WIFI_INTERFACE_P2P_CLIENT = 3,
	WIFI_INTERFACE_P2P_GO = 4,
	WIFI_INTERFACE_NAN = 5,
	WIFI_INTERFACE_MESH = 6
} wifi_interface_mode;

#define WIFI_CAPABILITY_QOS          0x00000001     
#define WIFI_CAPABILITY_PROTECTED    0x00000002     
#define WIFI_CAPABILITY_INTERWORKING 0x00000004     
#define WIFI_CAPABILITY_HS20         0x00000008     
#define WIFI_CAPABILITY_SSID_UTF8    0x00000010     
#define WIFI_CAPABILITY_COUNTRY      0x00000020     

typedef struct {
	wifi_interface_mode mode;     
	uint8 mac_addr[6];               
	wifi_connection_state state;  
	wifi_roam_state roaming;      
	uint32 capabilities;             
	uint8 ssid[DOT11_MAX_SSID_LEN+1]; 
	uint8 bssid[ETHER_ADDR_LEN];     
	uint8 ap_country_str[3];         
	uint8 country_str[3];            
} wifi_interface_info;

typedef wifi_interface_info *wifi_interface_handle;

typedef struct {
	wifi_channel_width_t width;   
	wifi_channel center_freq;   
	wifi_channel center_freq0;  
	wifi_channel center_freq1;  
} wifi_channel_info;

typedef struct {
	uint32 preamble;   
	uint32 nss;   	
	uint32 bw;   	
	uint32 rateMcsIdx; 
			
	uint32 reserved;   
	uint32 bitrate;    
} wifi_rate;

typedef struct {
	wifi_channel_info channel;  
	uint32 on_time;         	
	uint32 cca_busy_time;          
} wifi_channel_stat;

typedef struct {
	struct {
		uint16 version;
		uint16 length;
	};
	wifi_radio radio;               
	uint32 on_time;                    
	uint32 tx_time;                    
	uint32 rx_time;                    
	uint32 on_time_scan;               
	uint32 on_time_nbd;                
	uint32 on_time_gscan;              
	uint32 on_time_roam_scan;          
	uint32 on_time_pno_scan;           
	uint32 on_time_hs20;               
	uint32 num_channels;               
	wifi_channel_stat channels[1];   
} wifi_radio_stat;

typedef struct {
	struct {
		uint16 version;
		uint16 length;
	};
	uint32 tx_mpdu;        
	uint32 rx_mpdu;        
	uint32 mpdu_lost;      
	uint32 retries;        
	uint32 retries_short;  
	uint32 retries_long;   
	wifi_rate rate;     
} wifi_rate_stat;

typedef enum {
	WIFI_AC_VO  = 0,
	WIFI_AC_VI  = 1,
	WIFI_AC_BE  = 2,
	WIFI_AC_BK  = 3,
	WIFI_AC_MAX = 4
} wifi_traffic_ac;

typedef enum
{
	WIFI_PEER_STA,
	WIFI_PEER_AP,
	WIFI_PEER_P2P_GO,
	WIFI_PEER_P2P_CLIENT,
	WIFI_PEER_NAN,
	WIFI_PEER_TDLS,
	WIFI_PEER_INVALID
} wifi_peer_type;

typedef struct {
	wifi_peer_type type;           
	uint8 peer_mac_address[6];        
	uint32 capabilities;              
	uint32 num_rate;                  
	wifi_rate_stat rate_stats[1];   
} wifi_peer_info;

typedef struct {
	wifi_traffic_ac ac;             
	uint32 tx_mpdu;                    
	uint32 rx_mpdu;                    
	uint32 tx_mcast;                   
	uint32 rx_mcast;                   
	uint32 rx_ampdu;                   
	uint32 tx_ampdu;                   
	uint32 mpdu_lost;                  
	uint32 retries;                    
	uint32 retries_short;              
	uint32 retries_long;               
	uint32 contention_time_min;        
	uint32 contention_time_max;        
	uint32 contention_time_avg;        
	uint32 contention_num_samples;     
} wifi_wmm_ac_stat;

typedef struct {
	wifi_interface_handle iface;          
	wifi_interface_info info;             
	uint32 beacon_rx;                     
	uint32 mgmt_rx;                       
	uint32 mgmt_action_rx;                
	uint32 mgmt_action_tx;                
	wifi_rssi rssi_mgmt;                  
	wifi_rssi rssi_data;                  
	wifi_rssi rssi_ack;                   
	wifi_wmm_ac_stat ac[WIFI_AC_MAX];     
	uint32 num_peers;                        
	wifi_peer_info peer_info[1];           
} wifi_iface_stat;

#endif 
