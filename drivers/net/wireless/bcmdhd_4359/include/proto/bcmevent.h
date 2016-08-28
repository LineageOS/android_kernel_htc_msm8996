/*
 * Broadcom Event  protocol definitions
 *
 * Dependencies: proto/bcmeth.h
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
 * $Id: bcmevent.h 617327 2016-02-05 01:47:54Z $
 *
 */


#ifndef _BCMEVENT_H_
#define _BCMEVENT_H_

#ifndef _TYPEDEFS_H_
#include <typedefs.h>
#endif
#include <proto/bcmeth.h>

#include <packed_section_start.h>

#define BCM_EVENT_MSG_VERSION		2	
#define BCM_MSG_IFNAME_MAX		16	

#define WLC_EVENT_MSG_LINK		0x01	
#define WLC_EVENT_MSG_FLUSHTXQ		0x02	
#define WLC_EVENT_MSG_GROUP		0x04	
#define WLC_EVENT_MSG_UNKBSS		0x08	
#define WLC_EVENT_MSG_UNKIF		0x10	


typedef BWL_PRE_PACKED_STRUCT struct
{
	uint16	version;
	uint16	flags;			
	uint32	event_type;		
	uint32	status;			
	uint32	reason;			
	uint32	auth_type;		
	uint32	datalen;		
	struct ether_addr	addr;	
	char	ifname[BCM_MSG_IFNAME_MAX]; 
} BWL_POST_PACKED_STRUCT wl_event_msg_v1_t;

typedef BWL_PRE_PACKED_STRUCT struct
{
	uint16	version;
	uint16	flags;			
	uint32	event_type;		
	uint32	status;			
	uint32	reason;			
	uint32	auth_type;		
	uint32	datalen;		
	struct ether_addr	addr;	
	char	ifname[BCM_MSG_IFNAME_MAX]; 
	uint8	ifidx;			
	uint8	bsscfgidx;		
} BWL_POST_PACKED_STRUCT wl_event_msg_t;
typedef union bcm_event_msg_u {
        wl_event_msg_t          event;
#ifdef HEALTH_CHECK
        bcm_dngl_event_msg_t    dngl_event;
#endif 

        
} bcm_event_msg_u_t;
typedef BWL_PRE_PACKED_STRUCT struct bcm_event {
	struct ether_header eth;
	bcmeth_hdr_t		bcm_hdr;
	wl_event_msg_t		event;
	
} BWL_POST_PACKED_STRUCT bcm_event_t;

#define BCM_MSG_LEN	(sizeof(bcm_event_t) - sizeof(bcmeth_hdr_t) - sizeof(struct ether_header))

#define WLC_E_SET_SSID		0	
#define WLC_E_JOIN		1	
#define WLC_E_START		2	
#define WLC_E_AUTH		3	
#define WLC_E_AUTH_IND		4	
#define WLC_E_DEAUTH		5	
#define WLC_E_DEAUTH_IND	6	
#define WLC_E_ASSOC		7	
#define WLC_E_ASSOC_IND		8	
#define WLC_E_REASSOC		9	
#define WLC_E_REASSOC_IND	10	
#define WLC_E_DISASSOC		11	
#define WLC_E_DISASSOC_IND	12	
#define WLC_E_QUIET_START	13	
#define WLC_E_QUIET_END		14	
#define WLC_E_BEACON_RX		15	
#define WLC_E_LINK		16	
#define WLC_E_MIC_ERROR		17	
#define WLC_E_NDIS_LINK		18	
#define WLC_E_ROAM		19	
#define WLC_E_TXFAIL		20	
#define WLC_E_PMKID_CACHE	21	
#define WLC_E_RETROGRADE_TSF	22	
#define WLC_E_PRUNE		23	
#define WLC_E_AUTOAUTH		24	
#define WLC_E_EAPOL_MSG		25	
#define WLC_E_SCAN_COMPLETE	26	
#define WLC_E_ADDTS_IND		27	
#define WLC_E_DELTS_IND		28	
#define WLC_E_BCNSENT_IND	29	
#define WLC_E_BCNRX_MSG		30	
#define WLC_E_BCNLOST_MSG	31	
#define WLC_E_ROAM_PREP		32	
#define WLC_E_PFN_NET_FOUND	33	
#define WLC_E_PFN_NET_LOST	34	
#define WLC_E_RESET_COMPLETE	35
#define WLC_E_JOIN_START	36
#define WLC_E_ROAM_START	37	
#define WLC_E_ASSOC_START	38
#define WLC_E_IBSS_ASSOC	39
#define WLC_E_RADIO		40
#define WLC_E_PSM_WATCHDOG	41	
#if defined(BCMCCX) && defined(CCX_SDK)
#define WLC_E_CCX_ASSOC_START	42	
#define WLC_E_CCX_ASSOC_ABORT	43	
#endif 
#define WLC_E_PROBREQ_MSG       44      
#define WLC_E_SCAN_CONFIRM_IND  45
#define WLC_E_PSK_SUP		46	
#define WLC_E_COUNTRY_CODE_CHANGED	47
#define	WLC_E_EXCEEDED_MEDIUM_TIME	48	
#define WLC_E_ICV_ERROR		49	
#define WLC_E_UNICAST_DECODE_ERROR	50	
#define WLC_E_MULTICAST_DECODE_ERROR	51	
#define WLC_E_TRACE		52
#ifdef WLBTAMP
#define WLC_E_BTA_HCI_EVENT	53	
#endif
#define WLC_E_IF		54	
#define WLC_E_P2P_DISC_LISTEN_COMPLETE	55	
#define WLC_E_RSSI		56	
#define WLC_E_PFN_BEST_BATCHING	57	
#define WLC_E_EXTLOG_MSG	58
#define WLC_E_ACTION_FRAME      59	
#define WLC_E_ACTION_FRAME_COMPLETE	60	
#define WLC_E_PRE_ASSOC_IND	61	
#define WLC_E_PRE_REASSOC_IND	62	
#define WLC_E_CHANNEL_ADOPTED	63
#define WLC_E_AP_STARTED	64	
#define WLC_E_DFS_AP_STOP	65	
#define WLC_E_DFS_AP_RESUME	66	
#define WLC_E_WAI_STA_EVENT	67	
#define WLC_E_WAI_MSG 		68	
#define WLC_E_ESCAN_RESULT 	69	
#define WLC_E_ACTION_FRAME_OFF_CHAN_COMPLETE 	70	
#define WLC_E_PROBRESP_MSG	71	
#define WLC_E_P2P_PROBREQ_MSG	72	
#define WLC_E_DCS_REQUEST	73
#define WLC_E_FIFO_CREDIT_MAP	74	
#define WLC_E_ACTION_FRAME_RX	75	
#define WLC_E_WAKE_EVENT	76	
#define WLC_E_RM_COMPLETE	77	
#define WLC_E_HTSFSYNC		78	
#define WLC_E_OVERLAY_REQ	79	
#define WLC_E_CSA_COMPLETE_IND		80	
#define WLC_E_EXCESS_PM_WAKE_EVENT	81	
#define WLC_E_PFN_SCAN_NONE		82	
#define WLC_E_PFN_BSSID_NET_FOUND	82
#define WLC_E_PFN_SCAN_ALLGONE		83	
#define WLC_E_PFN_BSSID_NET_LOST	83
#define WLC_E_GTK_PLUMBED		84
#define WLC_E_ASSOC_IND_NDIS		85	
#define WLC_E_REASSOC_IND_NDIS		86	
#define WLC_E_ASSOC_REQ_IE		87
#define WLC_E_ASSOC_RESP_IE		88
#define WLC_E_ASSOC_RECREATED		89	
#define WLC_E_ACTION_FRAME_RX_NDIS	90	
#define WLC_E_AUTH_REQ			91	
#define WLC_E_TDLS_PEER_EVENT		92	
#define WLC_E_SPEEDY_RECREATE_FAIL	93	
#define WLC_E_NATIVE			94	
#define WLC_E_PKTDELAY_IND		95	
#define WLC_E_PSTA_PRIMARY_INTF_IND	99	
#define WLC_E_NAN			100     
#define WLC_E_BEACON_FRAME_RX		101
#define WLC_E_SERVICE_FOUND		102	
#define WLC_E_GAS_FRAGMENT_RX		103	
#define WLC_E_GAS_COMPLETE		104	
#define WLC_E_P2PO_ADD_DEVICE		105	
#define WLC_E_P2PO_DEL_DEVICE		106	
#define WLC_E_WNM_STA_SLEEP		107	
#define WLC_E_TXFAIL_THRESH		108	
#define WLC_E_PROXD			109	
#define WLC_E_IBSS_COALESCE		110	
#define WLC_E_AIBSS_TXFAIL		110	
#define WLC_E_BSS_LOAD			114	
#define WLC_E_MIMO_PWR_SAVE		115	
#define WLC_E_LEAKY_AP_STATS	116 
#define WLC_E_ALLOW_CREDIT_BORROW 117	
#define WLC_E_MSCH			120	
#define WLC_E_CSA_START_IND		121
#define WLC_E_CSA_DONE_IND		122
#define WLC_E_CSA_FAILURE_IND		123
#define WLC_E_CCA_CHAN_QUAL		124	
#define WLC_E_BSSID		125	
#define WLC_E_TX_STAT_ERROR		126	
#define WLC_E_BCMC_CREDIT_SUPPORT	127	
#define WLC_E_PEER_TIMEOUT	128 
#define WLC_E_BT_WIFI_HANDOVER_REQ	130	
#define WLC_E_SPW_TXINHIBIT		131     
#define WLC_E_FBT_AUTH_REQ_IND		132	
#define WLC_E_RSSI_LQM			133	
#define WLC_E_PFN_GSCAN_FULL_RESULT		134 
#define WLC_E_PFN_SWC		135 
#define WLC_E_AUTHORIZED	136	
#define WLC_E_PROBREQ_MSG_RX	137 
#define WLC_E_PFN_SCAN_COMPLETE	138	
#define WLC_E_RMC_EVENT		139	
#define WLC_E_DPSTA_INTF_IND	140	
#define WLC_E_RRM		141	
#define WLC_E_PFN_SSID_EXT	142	
#define WLC_E_ROAM_EXP_EVENT	143	
#define WLC_E_ULP			146	
#define WLC_E_MACDBG			147	
#define WLC_E_RESERVED			148	
#define WLC_E_PRE_ASSOC_RSEP_IND	149	
#define WLC_E_ID_AUTH			150	
#define WLC_E_TKO			151     
#define WLC_E_SDB_TRANSITION            152     
#define WLC_E_NATOE_NFCT		153     
#define WLC_E_LAST			154	
#if (WLC_E_LAST > 154)
#error "WLC_E_LAST: Invalid value for last event; must be <= 153."
#endif 

extern const char *bcmevent_get_name(uint event_type);
extern void wl_event_to_host_order(wl_event_msg_t * evt);
extern void wl_event_to_network_order(wl_event_msg_t * evt);
#ifndef SEC_ENHANCEMENT
extern int is_wlc_event_frame(void *pktdata, wl_event_msg_t *event,
        uint pktlen);
#else
extern int is_wlc_event_frame(void *pktdata, uint pktlen, uint16 exp_usr_subtype,
        bcm_event_msg_u_t *out_event);
#endif

extern int is_wlc_event_frame_tmp(void *pktdata, uint pktlen, uint16 exp_usr_subtype,
        bcm_event_msg_u_t *out_event);
void wl_event_to_host_order(wl_event_msg_t * evt);
void wl_event_to_network_order(wl_event_msg_t * evt);


#define WLC_E_STATUS_SUCCESS		0	
#define WLC_E_STATUS_FAIL		1	
#define WLC_E_STATUS_TIMEOUT		2	
#define WLC_E_STATUS_NO_NETWORKS	3	
#define WLC_E_STATUS_ABORT		4	
#define WLC_E_STATUS_NO_ACK		5	
#define WLC_E_STATUS_UNSOLICITED	6	
#define WLC_E_STATUS_ATTEMPT		7	
#define WLC_E_STATUS_PARTIAL		8	
#define WLC_E_STATUS_NEWSCAN		9	
#define WLC_E_STATUS_NEWASSOC		10	
#define WLC_E_STATUS_11HQUIET		11	
#define WLC_E_STATUS_SUPPRESS		12	
#define WLC_E_STATUS_NOCHANS		13	
#ifdef BCMCCX
#define WLC_E_STATUS_CCXFASTRM		14	
#endif 
#define WLC_E_STATUS_CS_ABORT		15	
#define WLC_E_STATUS_ERROR		16	
#define WLC_E_STATUS_INVALID 0xff  

#define WLC_E_STATUS_SDB_START          1
#define WLC_E_STATUS_SDB_COMPLETE       2

#define WLC_E_REASON_HOST_DIRECT	0
#define WLC_E_REASON_INFRA_ASSOC	1
#define WLC_E_REASON_INFRA_ROAM		2
#define WLC_E_REASON_INFRA_DISASSOC	3
#define WLC_E_REASON_NO_MODE_CHANGE_NEEDED	4

#define WL_MAX_BSSCFG     4
#define WL_EVENT_SDB_TRANSITION_VER     1
typedef struct wl_event_sdb_data {
	uint8 wlunit;           
	uint8 is_iftype;        
	uint16 chanspec;        
	char ssidbuf[(4 * 32) + 1];	
} wl_event_sdb_data_t;

typedef struct wl_event_sdb_trans {
	uint8 version;          
	uint8 rsdb_mode;
	uint8 enable_bsscfg;
	uint8 reserved;
	struct wl_event_sdb_data values[WL_MAX_BSSCFG];
} wl_event_sdb_trans_t;

#define WLC_E_REASON_INITIAL_ASSOC	0	
#define WLC_E_REASON_LOW_RSSI		1	
#define WLC_E_REASON_DEAUTH		2	
#define WLC_E_REASON_DISASSOC		3	
#define WLC_E_REASON_BCNS_LOST		4	

#define WLC_E_REASON_FAST_ROAM_FAILED	5	
#define WLC_E_REASON_DIRECTED_ROAM	6	
#define WLC_E_REASON_TSPEC_REJECTED	7	
#define WLC_E_REASON_BETTER_AP		8	
#define WLC_E_REASON_MINTXRATE		9	
#define WLC_E_REASON_TXFAIL		10	
#define WLC_E_REASON_REQUESTED_ROAM	11
#define WLC_E_REASON_BSSTRANS_REQ	11	
#define WLC_E_REASON_LOW_RSSI_CU		12 
#define WLC_E_REASON_RADAR_DETECTED	13	

#define WLC_E_PRUNE_ENCR_MISMATCH	1	
#define WLC_E_PRUNE_BCAST_BSSID		2	
#define WLC_E_PRUNE_MAC_DENY		3	
#define WLC_E_PRUNE_MAC_NA		4	
#define WLC_E_PRUNE_REG_PASSV		5	
#define WLC_E_PRUNE_SPCT_MGMT		6	
#define WLC_E_PRUNE_RADAR		7	
#define WLC_E_RSN_MISMATCH		8	
#define WLC_E_PRUNE_NO_COMMON_RATES	9	
#define WLC_E_PRUNE_BASIC_RATES		10	
#ifdef BCMCCX
#define WLC_E_PRUNE_CCXFAST_PREVAP	11	
#endif 
#define WLC_E_PRUNE_CIPHER_NA		12	
#define WLC_E_PRUNE_KNOWN_STA		13	
#ifdef BCMCCX
#define WLC_E_PRUNE_CCXFAST_DROAM	14	
#endif 
#define WLC_E_PRUNE_WDS_PEER		15	
#define WLC_E_PRUNE_QBSS_LOAD		16	
#define WLC_E_PRUNE_HOME_AP		17	
#ifdef BCMCCX
#define WLC_E_PRUNE_AP_BLOCKED		18	
#define WLC_E_PRUNE_NO_DIAG_SUPPORT	19	
#endif 
#define WLC_E_PRUNE_AUTH_RESP_MAC	20	

#define WLC_E_SUP_OTHER			0	
#define WLC_E_SUP_DECRYPT_KEY_DATA	1	
#define WLC_E_SUP_BAD_UCAST_WEP128	2	
#define WLC_E_SUP_BAD_UCAST_WEP40	3	
#define WLC_E_SUP_UNSUP_KEY_LEN		4	
#define WLC_E_SUP_PW_KEY_CIPHER		5	
#define WLC_E_SUP_MSG3_TOO_MANY_IE	6	
#define WLC_E_SUP_MSG3_IE_MISMATCH	7	
#define WLC_E_SUP_NO_INSTALL_FLAG	8	
#define WLC_E_SUP_MSG3_NO_GTK		9	
#define WLC_E_SUP_GRP_KEY_CIPHER	10	
#define WLC_E_SUP_GRP_MSG1_NO_GTK	11	
#define WLC_E_SUP_GTK_DECRYPT_FAIL	12	
#define WLC_E_SUP_SEND_FAIL		13	
#define WLC_E_SUP_DEAUTH		14	
#define WLC_E_SUP_WPA_PSK_TMO		15	

#define WLC_E_MACDBG_LIST_PSM		0	
#define WLC_E_MACDBG_LIST_PSMX		1	
#define WLC_E_MACDBG_REGALL		2	

typedef BWL_PRE_PACKED_STRUCT struct wl_event_rx_frame_data {
	uint16	version;
	uint16	channel;	
	int32	rssi;
	uint32	mactime;
	uint32	rate;
} BWL_POST_PACKED_STRUCT wl_event_rx_frame_data_t;

#define BCM_RX_FRAME_DATA_VERSION 1

typedef struct wl_event_data_if {
	uint8 ifidx;		
	uint8 opcode;		
	uint8 reserved;		
	uint8 bssidx;		
	uint8 role;		
} wl_event_data_if_t;

typedef struct wl_event_data_natoe {
	uint32 natoe_active;
	uint32 sta_ip;
	uint16 start_port;
	uint16 end_port;
} wl_event_data_natoe_t;

#define WLC_E_IF_ADD		1	
#define WLC_E_IF_DEL		2	
#define WLC_E_IF_CHANGE		3	

#define WLC_E_IF_ROLE_STA		0	
#define WLC_E_IF_ROLE_AP		1	
#define WLC_E_IF_ROLE_WDS		2	
#define WLC_E_IF_ROLE_P2P_GO		3	
#define WLC_E_IF_ROLE_P2P_CLIENT	4	
#ifdef WLBTAMP
#define WLC_E_IF_ROLE_BTA_CREATOR	5	
#define WLC_E_IF_ROLE_BTA_ACCEPTOR	6	
#endif
#define WLC_E_IF_ROLE_IBSS              8       
#ifdef WL_NAN
#define WLC_E_IF_ROLE_NAN               9       
#endif

typedef struct wl_event_data_rssi {
	int32 rssi;
	int32 snr;
	int32 noise;
} wl_event_data_rssi_t;

#define WLC_E_IF_FLAGS_BSSCFG_NOIF	0x1	

#define WLC_E_LINK_BCN_LOSS     1   
#define WLC_E_LINK_DISASSOC     2   
#define WLC_E_LINK_ASSOC_REC    3   
#define WLC_E_LINK_BSSCFG_DIS   4   


typedef BWL_PRE_PACKED_STRUCT struct ndis_link_parms {
	struct ether_addr peer_mac; 
	uint16 chanspec;            
	uint32 link_speed;          
	uint32 max_link_speed;      
	int32  rssi;                
} BWL_POST_PACKED_STRUCT ndis_link_parms_t;

#define WLC_E_OVL_DOWNLOAD		0	
#define WLC_E_OVL_UPDATE_IND	1	

#define WLC_E_TDLS_PEER_DISCOVERED		0	
#define WLC_E_TDLS_PEER_CONNECTED		1
#define WLC_E_TDLS_PEER_DISCONNECTED	2

#define WLC_E_REASON_RMC_NONE		0
#define WLC_E_REASON_RMC_AR_LOST		1
#define WLC_E_REASON_RMC_AR_NO_ACK		2

#ifdef WLTDLS
#define TDLS_AF_CATEGORY		12
#define TDLS_VENDOR_SPECIFIC		127
#define TDLS_ACTION_SETUP_REQ		0
#define TDLS_ACTION_SETUP_RESP		1
#define TDLS_ACTION_SETUP_CONFIRM	2
#define TDLS_ACTION_TEARDOWN		3
#define WLAN_TDLS_SET_PROBE_WFD_IE	11
#define WLAN_TDLS_SET_SETUP_WFD_IE	12
#define WLAN_TDLS_SET_WFD_ENABLED	13
#define WLAN_TDLS_SET_WFD_DISABLED	14
#endif


typedef BWL_PRE_PACKED_STRUCT struct wl_event_gas {
	uint16	channel;		
	uint8	dialog_token;	
	uint8	fragment_id;	
	uint16	status_code;	
	uint16 	data_len;		
	uint8	data[1];		
} BWL_POST_PACKED_STRUCT wl_event_gas_t;

typedef BWL_PRE_PACKED_STRUCT struct wl_sd_tlv {
	uint16	length;			
	uint8	protocol;		
	uint8	transaction_id;		
	uint8	status_code;		
	uint8	data[1];		
} BWL_POST_PACKED_STRUCT wl_sd_tlv_t;

typedef BWL_PRE_PACKED_STRUCT struct wl_event_sd {
	uint16	channel;		
	uint8	count;			
	wl_sd_tlv_t	tlv[1];		
} BWL_POST_PACKED_STRUCT wl_event_sd_t;


#define WLC_E_PROXD_FOUND		1	
#define WLC_E_PROXD_GONE		2	
#define WLC_E_PROXD_START		3	
#define WLC_E_PROXD_STOP		4	
#define WLC_E_PROXD_COMPLETED		5	
#define WLC_E_PROXD_ERROR		6	
#define WLC_E_PROXD_COLLECT_START	7	
#define WLC_E_PROXD_COLLECT_STOP	8	
#define WLC_E_PROXD_COLLECT_COMPLETED	9	
#define WLC_E_PROXD_COLLECT_ERROR	10	
#define WLC_E_PROXD_NAN_EVENT		11	
#define WLC_E_PROXD_TS_RESULTS          12      

typedef struct ftm_sample {
	uint32 value;	
	int8 rssi;	
} ftm_sample_t;

typedef struct ts_sample {
	uint32 t1;
	uint32 t2;
	uint32 t3;
	uint32 t4;
} ts_sample_t;

typedef BWL_PRE_PACKED_STRUCT struct proxd_event_data {
	uint16 ver;			
	uint16 mode;			
	uint16 method;			
	uint8  err_code;		
	uint8  TOF_type;		
	uint8  OFDM_frame_type;		
	uint8  bandwidth;		
	struct ether_addr peer_mac;	
	uint32 distance;		
	uint32 meanrtt;			
	uint32 modertt;			
	uint32 medianrtt;		
	uint32 sdrtt;			
	int32  gdcalcresult;		
					
	int16  avg_rssi;		
	int16  validfrmcnt;		
	int32  peer_router_info;	
					
	int32 var1;			
	int32 var2;			
	int32 var3;			
					
	uint16 ftm_unit;		
	uint16 ftm_cnt;			
	ftm_sample_t ftm_buff[1];	
} BWL_POST_PACKED_STRUCT wl_proxd_event_data_t;

typedef BWL_PRE_PACKED_STRUCT struct proxd_event_ts_results {
	uint16 ver;                     
	uint16 mode;                    
	uint16 method;                  
	uint8  err_code;                
	uint8  TOF_type;                
	uint16  ts_cnt;                 
	ts_sample_t ts_buff[1];         
} BWL_POST_PACKED_STRUCT wl_proxd_event_ts_results_t;


#define INTFER_EVENT_VERSION		1
#define INTFER_STREAM_TYPE_NONTCP	1
#define INTFER_STREAM_TYPE_TCP		2
#define WLINTFER_STATS_NSMPLS		4
typedef struct wl_intfer_event {
	uint16 version;			
	uint16 status;			
	uint8 txfail_histo[WLINTFER_STATS_NSMPLS]; 
} wl_intfer_event_t;

#define RRM_EVENT_VERSION		0
typedef struct wl_rrm_event {
	int16 version;
	int16 len;
	int16 cat;		
	int16 subevent;
	char payload[1]; 
} wl_rrm_event_t;


typedef struct wl_psta_primary_intf_event {
	struct ether_addr prim_ea;	
} wl_psta_primary_intf_event_t;

typedef enum {
	WL_INTF_PSTA = 1,
	WL_INTF_DWDS = 2
} wl_dpsta_intf_type;

typedef struct wl_dpsta_intf_event {
	wl_dpsta_intf_type intf_type;    
} wl_dpsta_intf_event_t;

#define NAN_EVENT_BUFFER_SIZE 512 
typedef enum nan_app_events {
	WL_NAN_EVENT_START = 1,     
	WL_NAN_EVENT_JOIN = 2,      
	WL_NAN_EVENT_ROLE = 3,      
	WL_NAN_EVENT_SCAN_COMPLETE = 4,
	WL_NAN_EVENT_DISCOVERY_RESULT = 5,
	WL_NAN_EVENT_REPLIED = 6,
	WL_NAN_EVENT_TERMINATED = 7,	
	WL_NAN_EVENT_RECEIVE = 8,
	WL_NAN_EVENT_STATUS_CHG = 9,  
	WL_NAN_EVENT_MERGE = 10,      
	WL_NAN_EVENT_STOP = 11,       
	WL_NAN_EVENT_P2P = 12,       
	WL_NAN_EVENT_WINDOW_BEGIN_P2P = 13, 
	WL_NAN_EVENT_WINDOW_BEGIN_MESH = 14,
	WL_NAN_EVENT_WINDOW_BEGIN_IBSS = 15,
	WL_NAN_EVENT_WINDOW_BEGIN_RANGING = 16,
	WL_NAN_EVENT_POST_DISC = 17, 
	WL_NAN_EVENT_DATA_IF_ADD = 18, 
	WL_NAN_EVENT_DATA_PEER_ADD = 19, 
	WL_NAN_EVENT_INVALID	
} nan_app_events_e;

#define IS_NAN_EVT_ON(var, evt) ((var & (1 << (evt-1))) != 0)

#define WL_ULP_EVENT_VERSION		1
#define WL_ULP_DISABLE_CONSOLE		1	

typedef struct wl_ulp_event {
	uint16 version;
	uint16 ulp_dongle_action;
} wl_ulp_event_t;

typedef BWL_PRE_PACKED_STRUCT struct wl_event_tko {
	uint8 index;		
	uint8 pad[3];		
} BWL_POST_PACKED_STRUCT wl_event_tko_t;

#include <packed_section_end.h>

#endif 
