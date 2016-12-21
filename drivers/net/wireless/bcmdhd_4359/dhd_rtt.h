/*
 * Broadcom Dongle Host Driver (DHD), RTT
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
 * $Id: dhd_rtt.h 599270 2015-11-13 05:22:03Z $
 */
#ifndef __DHD_RTT_H__
#define __DHD_RTT_H__

#include "dngl_stats.h"

#define RTT_MAX_TARGET_CNT	50
#define RTT_MAX_FRAME_CNT	25
#define RTT_MAX_RETRY_CNT	10
#define DEFAULT_FTM_CNT		6
#define DEFAULT_RETRY_CNT	6
#define TARGET_INFO_SIZE(count) (sizeof(rtt_target_info_t) * count)

#define TARGET_TYPE(target) (target->type)

#ifndef BIT
#define BIT(x) (1 << (x))
#endif

#define WL_MAXRATE	108	
#define WL_RATE_1M	2	
#define WL_RATE_2M	4	
#define WL_RATE_5M5	11	
#define WL_RATE_11M	22	
#define WL_RATE_6M	12	
#define WL_RATE_9M	18	
#define WL_RATE_12M	24	
#define WL_RATE_18M	36	
#define WL_RATE_24M	48	
#define WL_RATE_36M	72	
#define WL_RATE_48M	96	
#define WL_RATE_54M	108	


enum rtt_role {
	RTT_INITIATOR = 0,
	RTT_TARGET = 1
};

enum rtt_status {
	RTT_STOPPED = 0,
	RTT_STARTED = 1,
	RTT_ENABLED = 2
};

typedef int64_t wifi_timestamp; 
typedef int64_t wifi_timespan;
typedef int32 wifi_rssi;

typedef enum {
	RTT_INVALID,
	RTT_ONE_WAY,
	RTT_TWO_WAY,
	RTT_AUTO
} rtt_type_t;

typedef enum {
	RTT_PEER_STA,
	RTT_PEER_AP,
	RTT_PEER_P2P,
	RTT_PEER_NAN,
	RTT_PEER_INVALID
} rtt_peer_type_t;

typedef enum rtt_reason {
	RTT_REASON_SUCCESS,
	RTT_REASON_FAILURE,
	RTT_REASON_FAIL_NO_RSP,
	RTT_REASON_FAIL_INVALID_TS, 
	RTT_REASON_FAIL_PROTOCOL, 
	RTT_REASON_FAIL_REJECTED,
	RTT_REASON_FAIL_NOT_SCHEDULED_YET,
	RTT_REASON_FAIL_SCHEDULE, 
	RTT_REASON_FAIL_TM_TIMEOUT,
	RTT_REASON_FAIL_AP_ON_DIFF_CHANNEL,
	RTT_REASON_FAIL_NO_CAPABILITY,
	RTT_REASON_FAIL_BUSY_TRY_LATER,
	RTT_REASON_ABORTED
} rtt_reason_t;

enum {
	RTT_CAP_ONE_WAY	 = BIT(0),
	
	RTT_CAP_FTM_WAY  = BIT(1)
};

enum {
	RTT_FEATURE_LCI = BIT(0),
	RTT_FEATURE_LCR = BIT(1),
	RTT_FEATURE_PREAMBLE = BIT(2),
	RTT_FEATURE_BW = BIT(3)
};

enum {
	RTT_PREAMBLE_LEGACY = BIT(0),
	RTT_PREAMBLE_HT = BIT(1),
	RTT_PREAMBLE_VHT = BIT(2)
};

enum {
	RTT_BW_5 = BIT(0),
	RTT_BW_10 = BIT(1),
	RTT_BW_20 = BIT(2),
	RTT_BW_40 = BIT(3),
	RTT_BW_80 = BIT(4),
	RTT_BW_160 = BIT(5)
};

#define FTM_MAX_NUM_BURST_EXP	14
#define HAS_11MC_CAP(cap) (cap & RTT_CAP_FTM_WAY)
#define HAS_ONEWAY_CAP(cap) (cap & RTT_CAP_ONE_WAY)
#define HAS_RTT_CAP(cap) (HAS_ONEWAY_CAP(cap) || HAS_11MC_CAP(cap))

typedef struct wifi_channel_info {
	wifi_channel_width_t width;
	wifi_channel center_freq; 
	wifi_channel center_freq0; 
	wifi_channel center_freq1; 
} wifi_channel_info_t;

typedef struct wifi_rate {
	uint32 preamble		:3; 
	uint32 nss		:2; 
	uint32 bw		:3; 
	uint32 rateMcsIdx :8;
	uint32 reserved :16; 
	uint32 bitrate;	
} wifi_rate_t;

typedef struct rtt_target_info {
	struct ether_addr addr;
	rtt_type_t type; 
	rtt_peer_type_t peer; 
	wifi_channel_info_t channel; 
	chanspec_t chanspec; 
	bool	disable; 
	uint32	burst_period;
	uint16 num_burst;
	uint32 num_frames_per_burst;
	uint32 num_retries_per_ftm; 
	
	uint32 num_retries_per_ftmr;
	uint8  LCI_request;
	uint8  LCR_request;
	uint32 burst_duration;
	uint8  preamble; 
	uint8  bw;  
} rtt_target_info_t;

typedef struct rtt_report {
	struct ether_addr addr;
	unsigned int burst_num; 
	unsigned int ftm_num; 
	unsigned int success_num; 
	uint8  num_per_burst_peer; 
	rtt_reason_t status; 
	
	uint8 retry_after_duration;
	rtt_type_t type; 
	wifi_rssi  rssi; 
	wifi_rssi  rssi_spread; 
	wifi_rate_t tx_rate;
	wifi_rate_t rx_rate;
	wifi_timespan rtt;	
	wifi_timespan rtt_sd;	
	wifi_timespan rtt_spread; 
	int distance; 
	int distance_sd; 
	int distance_spread; 
	wifi_timestamp ts; 
	int burst_duration; 
	int negotiated_burst_num; 
	bcm_tlv_t *LCI; 
	bcm_tlv_t *LCR; 
} rtt_report_t;

#define RTT_REPORT_SIZE (sizeof(rtt_report_t))

typedef struct rtt_results_header {
	struct ether_addr peer_mac;
	uint32 result_cnt;
	uint32 result_tot_len; 
	struct list_head list;
	struct list_head result_list;
} rtt_results_header_t;

typedef struct rtt_result {
	struct list_head list;
	struct rtt_report report;
	int32 report_len; 
} rtt_result_t;

typedef struct rtt_capabilities {
	uint8 rtt_one_sided_supported;  
	uint8 rtt_ftm_supported;        
	uint8 lci_support;		
	uint8 lcr_support;		
	uint8 preamble_support;         
	uint8 bw_support;               
} rtt_capabilities_t;

typedef struct rtt_config_params {
	int8 rtt_target_cnt;
	rtt_target_info_t *target_info;
} rtt_config_params_t;

typedef void (*dhd_rtt_compl_noti_fn)(void *ctx, void *rtt_data);
int
dhd_dev_rtt_set_cfg(struct net_device *dev, void *buf);

int
dhd_dev_rtt_cancel_cfg(struct net_device *dev, struct ether_addr *mac_list, int mac_cnt);

int
dhd_dev_rtt_register_noti_callback(struct net_device *dev, void *ctx,
	dhd_rtt_compl_noti_fn noti_fn);

int
dhd_dev_rtt_unregister_noti_callback(struct net_device *dev, dhd_rtt_compl_noti_fn noti_fn);

int
dhd_dev_rtt_capability(struct net_device *dev, rtt_capabilities_t *capa);

chanspec_t
dhd_rtt_convert_to_chspec(wifi_channel_info_t channel);

int
dhd_rtt_idx_to_burst_duration(uint idx);

int
dhd_rtt_set_cfg(dhd_pub_t *dhd, rtt_config_params_t *params);

int
dhd_rtt_stop(dhd_pub_t *dhd, struct ether_addr *mac_list, int mac_cnt);


int
dhd_rtt_register_noti_callback(dhd_pub_t *dhd, void *ctx, dhd_rtt_compl_noti_fn noti_fn);

int
dhd_rtt_unregister_noti_callback(dhd_pub_t *dhd, dhd_rtt_compl_noti_fn noti_fn);

int
dhd_rtt_event_handler(dhd_pub_t *dhd, wl_event_msg_t *event, void *event_data);

int
dhd_rtt_capability(dhd_pub_t *dhd, rtt_capabilities_t *capa);

int
dhd_rtt_init(dhd_pub_t *dhd);

int
dhd_rtt_deinit(dhd_pub_t *dhd);
#endif 
