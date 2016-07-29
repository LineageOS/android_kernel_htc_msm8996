#ifndef __NET_CFG80211_H
#define __NET_CFG80211_H
/*
 * 802.11 device and configuration interface
 *
 * Copyright 2006-2010	Johannes Berg <johannes@sipsolutions.net>
 * Copyright 2013-2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/netdevice.h>
#include <linux/debugfs.h>
#include <linux/list.h>
#include <linux/bug.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/nl80211.h>
#include <linux/if_ether.h>
#include <linux/ieee80211.h>
#include <linux/net.h>
#include <net/regulatory.h>




struct wiphy;

#define SUPPORT_WDEV_CFG80211_VENDOR_EVENT_ALLOC 1
#define CFG80211_DEL_STA_V2 1


enum ieee80211_band {
	IEEE80211_BAND_2GHZ = NL80211_BAND_2GHZ,
	IEEE80211_BAND_5GHZ = NL80211_BAND_5GHZ,
	IEEE80211_BAND_60GHZ = NL80211_BAND_60GHZ,

	
	IEEE80211_NUM_BANDS
};

enum ieee80211_channel_flags {
	IEEE80211_CHAN_DISABLED		= 1<<0,
	IEEE80211_CHAN_NO_IR		= 1<<1,
	
	IEEE80211_CHAN_RADAR		= 1<<3,
	IEEE80211_CHAN_NO_HT40PLUS	= 1<<4,
	IEEE80211_CHAN_NO_HT40MINUS	= 1<<5,
	IEEE80211_CHAN_NO_OFDM		= 1<<6,
	IEEE80211_CHAN_NO_80MHZ		= 1<<7,
	IEEE80211_CHAN_NO_160MHZ	= 1<<8,
	IEEE80211_CHAN_INDOOR_ONLY	= 1<<9,
	IEEE80211_CHAN_GO_CONCURRENT	= 1<<10,
	IEEE80211_CHAN_NO_20MHZ		= 1<<11,
	IEEE80211_CHAN_NO_10MHZ		= 1<<12,
};

#define IEEE80211_CHAN_NO_HT40 \
	(IEEE80211_CHAN_NO_HT40PLUS | IEEE80211_CHAN_NO_HT40MINUS)

#define IEEE80211_DFS_MIN_CAC_TIME_MS		60000
#define IEEE80211_DFS_MIN_NOP_TIME_MS		(30 * 60 * 1000)

struct ieee80211_channel {
	enum ieee80211_band band;
	u16 center_freq;
	u16 hw_value;
	u32 flags;
	int max_antenna_gain;
	int max_power;
	int max_reg_power;
	bool beacon_found;
	u32 orig_flags;
	int orig_mag, orig_mpwr;
	enum nl80211_dfs_state dfs_state;
	unsigned long dfs_state_entered;
	unsigned int dfs_cac_ms;
};

enum ieee80211_rate_flags {
	IEEE80211_RATE_SHORT_PREAMBLE	= 1<<0,
	IEEE80211_RATE_MANDATORY_A	= 1<<1,
	IEEE80211_RATE_MANDATORY_B	= 1<<2,
	IEEE80211_RATE_MANDATORY_G	= 1<<3,
	IEEE80211_RATE_ERP_G		= 1<<4,
	IEEE80211_RATE_SUPPORTS_5MHZ	= 1<<5,
	IEEE80211_RATE_SUPPORTS_10MHZ	= 1<<6,
};

struct ieee80211_rate {
	u32 flags;
	u16 bitrate;
	u16 hw_value, hw_value_short;
};

struct ieee80211_sta_ht_cap {
	u16 cap; 
	bool ht_supported;
	u8 ampdu_factor;
	u8 ampdu_density;
	struct ieee80211_mcs_info mcs;
};

struct ieee80211_sta_vht_cap {
	bool vht_supported;
	u32 cap; 
	struct ieee80211_vht_mcs_info vht_mcs;
};

struct ieee80211_supported_band {
	struct ieee80211_channel *channels;
	struct ieee80211_rate *bitrates;
	enum ieee80211_band band;
	int n_channels;
	int n_bitrates;
	struct ieee80211_sta_ht_cap ht_cap;
	struct ieee80211_sta_vht_cap vht_cap;
};



struct vif_params {
       int use_4addr;
       u8 macaddr[ETH_ALEN];
};

struct key_params {
	const u8 *key;
	const u8 *seq;
	int key_len;
	int seq_len;
	u32 cipher;
};

struct cfg80211_chan_def {
	struct ieee80211_channel *chan;
	enum nl80211_chan_width width;
	u32 center_freq1;
	u32 center_freq2;
};

static inline enum nl80211_channel_type
cfg80211_get_chandef_type(const struct cfg80211_chan_def *chandef)
{
	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_20_NOHT:
		return NL80211_CHAN_NO_HT;
	case NL80211_CHAN_WIDTH_20:
		return NL80211_CHAN_HT20;
	case NL80211_CHAN_WIDTH_40:
		if (chandef->center_freq1 > chandef->chan->center_freq)
			return NL80211_CHAN_HT40PLUS;
		return NL80211_CHAN_HT40MINUS;
	default:
		WARN_ON(1);
		return NL80211_CHAN_NO_HT;
	}
}

void cfg80211_chandef_create(struct cfg80211_chan_def *chandef,
			     struct ieee80211_channel *channel,
			     enum nl80211_channel_type chantype);

static inline bool
cfg80211_chandef_identical(const struct cfg80211_chan_def *chandef1,
			   const struct cfg80211_chan_def *chandef2)
{
	return (chandef1->chan == chandef2->chan &&
		chandef1->width == chandef2->width &&
		chandef1->center_freq1 == chandef2->center_freq1 &&
		chandef1->center_freq2 == chandef2->center_freq2);
}

const struct cfg80211_chan_def *
cfg80211_chandef_compatible(const struct cfg80211_chan_def *chandef1,
			    const struct cfg80211_chan_def *chandef2);

bool cfg80211_chandef_valid(const struct cfg80211_chan_def *chandef);

bool cfg80211_chandef_usable(struct wiphy *wiphy,
			     const struct cfg80211_chan_def *chandef,
			     u32 prohibited_flags);

int cfg80211_chandef_dfs_required(struct wiphy *wiphy,
				  const struct cfg80211_chan_def *chandef,
				  enum nl80211_iftype iftype);

static inline enum ieee80211_rate_flags
ieee80211_chandef_rate_flags(struct cfg80211_chan_def *chandef)
{
	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_5:
		return IEEE80211_RATE_SUPPORTS_5MHZ;
	case NL80211_CHAN_WIDTH_10:
		return IEEE80211_RATE_SUPPORTS_10MHZ;
	default:
		break;
	}
	return 0;
}

static inline int
ieee80211_chandef_max_power(struct cfg80211_chan_def *chandef)
{
	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_5:
		return min(chandef->chan->max_reg_power - 6,
			   chandef->chan->max_power);
	case NL80211_CHAN_WIDTH_10:
		return min(chandef->chan->max_reg_power - 3,
			   chandef->chan->max_power);
	default:
		break;
	}
	return chandef->chan->max_power;
}

enum survey_info_flags {
	SURVEY_INFO_NOISE_DBM = 1<<0,
	SURVEY_INFO_IN_USE = 1<<1,
	SURVEY_INFO_CHANNEL_TIME = 1<<2,
	SURVEY_INFO_CHANNEL_TIME_BUSY = 1<<3,
	SURVEY_INFO_CHANNEL_TIME_EXT_BUSY = 1<<4,
	SURVEY_INFO_CHANNEL_TIME_RX = 1<<5,
	SURVEY_INFO_CHANNEL_TIME_TX = 1<<6,
};

struct survey_info {
	struct ieee80211_channel *channel;
	u64 channel_time;
	u64 channel_time_busy;
	u64 channel_time_ext_busy;
	u64 channel_time_rx;
	u64 channel_time_tx;
	u32 filled;
	s8 noise;
};

struct cfg80211_crypto_settings {
	u32 wpa_versions;
	u32 cipher_group;
	int n_ciphers_pairwise;
	u32 ciphers_pairwise[NL80211_MAX_NR_CIPHER_SUITES];
	int n_akm_suites;
	u32 akm_suites[NL80211_MAX_NR_AKM_SUITES];
	bool control_port;
	__be16 control_port_ethertype;
	bool control_port_no_encrypt;
};

struct cfg80211_beacon_data {
	const u8 *head, *tail;
	const u8 *beacon_ies;
	const u8 *proberesp_ies;
	const u8 *assocresp_ies;
	const u8 *probe_resp;

	size_t head_len, tail_len;
	size_t beacon_ies_len;
	size_t proberesp_ies_len;
	size_t assocresp_ies_len;
	size_t probe_resp_len;
};

struct mac_address {
	u8 addr[ETH_ALEN];
};

struct cfg80211_acl_data {
	enum nl80211_acl_policy acl_policy;
	int n_acl_entries;

	
	struct mac_address mac_addrs[];
};

struct cfg80211_ap_settings {
	struct cfg80211_chan_def chandef;

	struct cfg80211_beacon_data beacon;

	int beacon_interval, dtim_period;
	const u8 *ssid;
	size_t ssid_len;
	enum nl80211_hidden_ssid hidden_ssid;
	struct cfg80211_crypto_settings crypto;
	bool privacy;
	enum nl80211_auth_type auth_type;
	enum nl80211_smps_mode smps_mode;
	int inactivity_timeout;
	u8 p2p_ctwindow;
	bool p2p_opp_ps;
	const struct cfg80211_acl_data *acl;
};

struct cfg80211_csa_settings {
	struct cfg80211_chan_def chandef;
	struct cfg80211_beacon_data beacon_csa;
	const u16 *counter_offsets_beacon;
	const u16 *counter_offsets_presp;
	unsigned int n_counter_offsets_beacon;
	unsigned int n_counter_offsets_presp;
	struct cfg80211_beacon_data beacon_after;
	bool radar_required;
	bool block_tx;
	u8 count;
};

enum station_parameters_apply_mask {
	STATION_PARAM_APPLY_UAPSD = BIT(0),
	STATION_PARAM_APPLY_CAPABILITY = BIT(1),
	STATION_PARAM_APPLY_PLINK_STATE = BIT(2),
};

struct station_parameters {
	const u8 *supported_rates;
	struct net_device *vlan;
	u32 sta_flags_mask, sta_flags_set;
	u32 sta_modify_mask;
	int listen_interval;
	u16 aid;
	u8 supported_rates_len;
	u8 plink_action;
	u8 plink_state;
	const struct ieee80211_ht_cap *ht_capa;
	const struct ieee80211_vht_cap *vht_capa;
	u8 uapsd_queues;
	u8 max_sp;
	enum nl80211_mesh_power_mode local_pm;
	u16 capability;
	const u8 *ext_capab;
	u8 ext_capab_len;
	const u8 *supported_channels;
	u8 supported_channels_len;
	const u8 *supported_oper_classes;
	u8 supported_oper_classes_len;
	u8 opmode_notif;
	bool opmode_notif_used;
};

struct station_del_parameters {
	const u8 *mac;
	u8 subtype;
	u16 reason_code;
};

enum cfg80211_station_type {
	CFG80211_STA_AP_CLIENT,
	CFG80211_STA_AP_MLME_CLIENT,
	CFG80211_STA_AP_STA,
	CFG80211_STA_IBSS,
	CFG80211_STA_TDLS_PEER_SETUP,
	CFG80211_STA_TDLS_PEER_ACTIVE,
	CFG80211_STA_MESH_PEER_KERNEL,
	CFG80211_STA_MESH_PEER_USER,
};

int cfg80211_check_station_change(struct wiphy *wiphy,
				  struct station_parameters *params,
				  enum cfg80211_station_type statype);

enum station_info_flags {
	STATION_INFO_INACTIVE_TIME		= BIT(0),
	STATION_INFO_RX_BYTES			= BIT(1),
	STATION_INFO_TX_BYTES			= BIT(2),
	STATION_INFO_LLID			= BIT(3),
	STATION_INFO_PLID			= BIT(4),
	STATION_INFO_PLINK_STATE		= BIT(5),
	STATION_INFO_SIGNAL			= BIT(6),
	STATION_INFO_TX_BITRATE			= BIT(7),
	STATION_INFO_RX_PACKETS			= BIT(8),
	STATION_INFO_TX_PACKETS			= BIT(9),
	STATION_INFO_TX_RETRIES			= BIT(10),
	STATION_INFO_TX_FAILED			= BIT(11),
	STATION_INFO_RX_DROP_MISC		= BIT(12),
	STATION_INFO_SIGNAL_AVG			= BIT(13),
	STATION_INFO_RX_BITRATE			= BIT(14),
	STATION_INFO_BSS_PARAM			= BIT(15),
	STATION_INFO_CONNECTED_TIME		= BIT(16),
	STATION_INFO_ASSOC_REQ_IES		= BIT(17),
	STATION_INFO_STA_FLAGS			= BIT(18),
	STATION_INFO_BEACON_LOSS_COUNT		= BIT(19),
	STATION_INFO_T_OFFSET			= BIT(20),
	STATION_INFO_LOCAL_PM			= BIT(21),
	STATION_INFO_PEER_PM			= BIT(22),
	STATION_INFO_NONPEER_PM			= BIT(23),
	STATION_INFO_RX_BYTES64			= BIT(24),
	STATION_INFO_TX_BYTES64			= BIT(25),
	STATION_INFO_CHAIN_SIGNAL		= BIT(26),
	STATION_INFO_CHAIN_SIGNAL_AVG		= BIT(27),
	STATION_INFO_EXPECTED_THROUGHPUT	= BIT(28),
};

enum rate_info_flags {
	RATE_INFO_FLAGS_MCS			= BIT(0),
	RATE_INFO_FLAGS_VHT_MCS			= BIT(1),
	RATE_INFO_FLAGS_40_MHZ_WIDTH		= BIT(2),
	RATE_INFO_FLAGS_80_MHZ_WIDTH		= BIT(3),
	RATE_INFO_FLAGS_80P80_MHZ_WIDTH		= BIT(4),
	RATE_INFO_FLAGS_160_MHZ_WIDTH		= BIT(5),
	RATE_INFO_FLAGS_SHORT_GI		= BIT(6),
	RATE_INFO_FLAGS_60G			= BIT(7),
};

struct rate_info {
	u8 flags;
	u8 mcs;
	u16 legacy;
	u8 nss;
};

enum bss_param_flags {
	BSS_PARAM_FLAGS_CTS_PROT	= 1<<0,
	BSS_PARAM_FLAGS_SHORT_PREAMBLE	= 1<<1,
	BSS_PARAM_FLAGS_SHORT_SLOT_TIME	= 1<<2,
};

struct sta_bss_parameters {
	u8 flags;
	u8 dtim_period;
	u16 beacon_interval;
};

#define IEEE80211_MAX_CHAINS	4

struct station_info {
	u32 filled;
	u32 connected_time;
	u32 inactive_time;
	u64 rx_bytes;
	u64 tx_bytes;
	u16 llid;
	u16 plid;
	u8 plink_state;
	s8 signal;
	s8 signal_avg;

	u8 chains;
	s8 chain_signal[IEEE80211_MAX_CHAINS];
	s8 chain_signal_avg[IEEE80211_MAX_CHAINS];

	struct rate_info txrate;
	struct rate_info rxrate;
	u32 rx_packets;
	u32 tx_packets;
	u32 tx_retries;
	u32 tx_failed;
	u32 rx_dropped_misc;
	struct sta_bss_parameters bss_param;
	struct nl80211_sta_flag_update sta_flags;

	int generation;

	const u8 *assoc_req_ies;
	size_t assoc_req_ies_len;

	u32 beacon_loss_count;
	s64 t_offset;
	enum nl80211_mesh_power_mode local_pm;
	enum nl80211_mesh_power_mode peer_pm;
	enum nl80211_mesh_power_mode nonpeer_pm;

	u32 expected_throughput;

};

int cfg80211_get_station(struct net_device *dev, const u8 *mac_addr,
			 struct station_info *sinfo);

enum monitor_flags {
	MONITOR_FLAG_FCSFAIL		= 1<<NL80211_MNTR_FLAG_FCSFAIL,
	MONITOR_FLAG_PLCPFAIL		= 1<<NL80211_MNTR_FLAG_PLCPFAIL,
	MONITOR_FLAG_CONTROL		= 1<<NL80211_MNTR_FLAG_CONTROL,
	MONITOR_FLAG_OTHER_BSS		= 1<<NL80211_MNTR_FLAG_OTHER_BSS,
	MONITOR_FLAG_COOK_FRAMES	= 1<<NL80211_MNTR_FLAG_COOK_FRAMES,
	MONITOR_FLAG_ACTIVE		= 1<<NL80211_MNTR_FLAG_ACTIVE,
};

enum mpath_info_flags {
	MPATH_INFO_FRAME_QLEN		= BIT(0),
	MPATH_INFO_SN			= BIT(1),
	MPATH_INFO_METRIC		= BIT(2),
	MPATH_INFO_EXPTIME		= BIT(3),
	MPATH_INFO_DISCOVERY_TIMEOUT	= BIT(4),
	MPATH_INFO_DISCOVERY_RETRIES	= BIT(5),
	MPATH_INFO_FLAGS		= BIT(6),
};

struct mpath_info {
	u32 filled;
	u32 frame_qlen;
	u32 sn;
	u32 metric;
	u32 exptime;
	u32 discovery_timeout;
	u8 discovery_retries;
	u8 flags;

	int generation;
};

struct bss_parameters {
	int use_cts_prot;
	int use_short_preamble;
	int use_short_slot_time;
	const u8 *basic_rates;
	u8 basic_rates_len;
	int ap_isolate;
	int ht_opmode;
	s8 p2p_ctwindow, p2p_opp_ps;
};

struct mesh_config {
	u16 dot11MeshRetryTimeout;
	u16 dot11MeshConfirmTimeout;
	u16 dot11MeshHoldingTimeout;
	u16 dot11MeshMaxPeerLinks;
	u8 dot11MeshMaxRetries;
	u8 dot11MeshTTL;
	u8 element_ttl;
	bool auto_open_plinks;
	u32 dot11MeshNbrOffsetMaxNeighbor;
	u8 dot11MeshHWMPmaxPREQretries;
	u32 path_refresh_time;
	u16 min_discovery_timeout;
	u32 dot11MeshHWMPactivePathTimeout;
	u16 dot11MeshHWMPpreqMinInterval;
	u16 dot11MeshHWMPperrMinInterval;
	u16 dot11MeshHWMPnetDiameterTraversalTime;
	u8 dot11MeshHWMPRootMode;
	u16 dot11MeshHWMPRannInterval;
	bool dot11MeshGateAnnouncementProtocol;
	bool dot11MeshForwarding;
	s32 rssi_threshold;
	u16 ht_opmode;
	u32 dot11MeshHWMPactivePathToRootTimeout;
	u16 dot11MeshHWMProotInterval;
	u16 dot11MeshHWMPconfirmationInterval;
	enum nl80211_mesh_power_mode power_mode;
	u16 dot11MeshAwakeWindowDuration;
	u32 plink_timeout;
};

struct mesh_setup {
	struct cfg80211_chan_def chandef;
	const u8 *mesh_id;
	u8 mesh_id_len;
	u8 sync_method;
	u8 path_sel_proto;
	u8 path_metric;
	u8 auth_id;
	const u8 *ie;
	u8 ie_len;
	bool is_authenticated;
	bool is_secure;
	bool user_mpm;
	u8 dtim_period;
	u16 beacon_interval;
	int mcast_rate[IEEE80211_NUM_BANDS];
	u32 basic_rates;
};

struct ieee80211_txq_params {
	enum nl80211_ac ac;
	u16 txop;
	u16 cwmin;
	u16 cwmax;
	u8 aifs;
};


struct cfg80211_ssid {
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	u8 ssid_len;
};

struct cfg80211_scan_request {
	struct cfg80211_ssid *ssids;
	int n_ssids;
	u32 n_channels;
	enum nl80211_bss_scan_width scan_width;
	const u8 *ie;
	size_t ie_len;
	u32 flags;

	u32 rates[IEEE80211_NUM_BANDS];

	struct wireless_dev *wdev;

	
	struct wiphy *wiphy;
	unsigned long scan_start;
	bool aborted, notified;
	bool no_cck;

	
	struct ieee80211_channel *channels[0];
};

struct cfg80211_match_set {
	struct cfg80211_ssid ssid;
	s32 rssi_thold;
};

struct cfg80211_sched_scan_request {
	struct cfg80211_ssid *ssids;
	int n_ssids;
	u32 n_channels;
	enum nl80211_bss_scan_width scan_width;
	u32 interval;
	const u8 *ie;
	size_t ie_len;
	u32 flags;
	struct cfg80211_match_set *match_sets;
	int n_match_sets;
	s32 min_rssi_thold;

	
	struct wiphy *wiphy;
	struct net_device *dev;
	unsigned long scan_start;

	
	struct ieee80211_channel *channels[0];
};

enum cfg80211_signal_type {
	CFG80211_SIGNAL_TYPE_NONE,
	CFG80211_SIGNAL_TYPE_MBM,
	CFG80211_SIGNAL_TYPE_UNSPEC,
};

struct cfg80211_bss_ies {
	u64 tsf;
	struct rcu_head rcu_head;
	int len;
	bool from_beacon;
	u8 data[];
};

struct cfg80211_bss {
	struct ieee80211_channel *channel;
	enum nl80211_bss_scan_width scan_width;

	const struct cfg80211_bss_ies __rcu *ies;
	const struct cfg80211_bss_ies __rcu *beacon_ies;
	const struct cfg80211_bss_ies __rcu *proberesp_ies;

	struct cfg80211_bss *hidden_beacon_bss;

	s32 signal;

	u16 beacon_interval;
	u16 capability;

	u8 bssid[ETH_ALEN];

	u8 priv[0] __aligned(sizeof(void *));
};

const u8 *ieee80211_bss_get_ie(struct cfg80211_bss *bss, u8 ie);


struct cfg80211_auth_request {
	struct cfg80211_bss *bss;
	const u8 *ie;
	size_t ie_len;
	enum nl80211_auth_type auth_type;
	const u8 *key;
	u8 key_len, key_idx;
	const u8 *sae_data;
	size_t sae_data_len;
};

enum cfg80211_assoc_req_flags {
	ASSOC_REQ_DISABLE_HT		= BIT(0),
	ASSOC_REQ_DISABLE_VHT		= BIT(1),
	ASSOC_REQ_USE_RRM		= BIT(2),
};

struct cfg80211_assoc_request {
	struct cfg80211_bss *bss;
	const u8 *ie, *prev_bssid;
	size_t ie_len;
	struct cfg80211_crypto_settings crypto;
	bool use_mfp;
	u32 flags;
	struct ieee80211_ht_cap ht_capa;
	struct ieee80211_ht_cap ht_capa_mask;
	struct ieee80211_vht_cap vht_capa, vht_capa_mask;
};

struct cfg80211_deauth_request {
	const u8 *bssid;
	const u8 *ie;
	size_t ie_len;
	u16 reason_code;
	bool local_state_change;
};

struct cfg80211_disassoc_request {
	struct cfg80211_bss *bss;
	const u8 *ie;
	size_t ie_len;
	u16 reason_code;
	bool local_state_change;
};

struct cfg80211_ibss_params {
	const u8 *ssid;
	const u8 *bssid;
	struct cfg80211_chan_def chandef;
	const u8 *ie;
	u8 ssid_len, ie_len;
	u16 beacon_interval;
	u32 basic_rates;
	bool channel_fixed;
	bool privacy;
	bool control_port;
	bool userspace_handles_dfs;
	int mcast_rate[IEEE80211_NUM_BANDS];
	struct ieee80211_ht_cap ht_capa;
	struct ieee80211_ht_cap ht_capa_mask;
};

struct cfg80211_connect_params {
	struct ieee80211_channel *channel;
	struct ieee80211_channel *channel_hint;
	const u8 *bssid;
	const u8 *bssid_hint;
	const u8 *ssid;
	size_t ssid_len;
	enum nl80211_auth_type auth_type;
	const u8 *ie;
	size_t ie_len;
	bool privacy;
	enum nl80211_mfp mfp;
	struct cfg80211_crypto_settings crypto;
	const u8 *key;
	u8 key_len, key_idx;
	u32 flags;
	int bg_scan_period;
	struct ieee80211_ht_cap ht_capa;
	struct ieee80211_ht_cap ht_capa_mask;
	struct ieee80211_vht_cap vht_capa;
	struct ieee80211_vht_cap vht_capa_mask;
};

enum wiphy_params_flags {
	WIPHY_PARAM_RETRY_SHORT		= 1 << 0,
	WIPHY_PARAM_RETRY_LONG		= 1 << 1,
	WIPHY_PARAM_FRAG_THRESHOLD	= 1 << 2,
	WIPHY_PARAM_RTS_THRESHOLD	= 1 << 3,
	WIPHY_PARAM_COVERAGE_CLASS	= 1 << 4,
	WIPHY_PARAM_DYN_ACK		= 1 << 5,
};

struct cfg80211_bitrate_mask {
	struct {
		u32 legacy;
		u8 ht_mcs[IEEE80211_HT_MCS_MASK_LEN];
		u16 vht_mcs[NL80211_VHT_NSS_MAX];
		enum nl80211_txrate_gi gi;
	} control[IEEE80211_NUM_BANDS];
};
struct cfg80211_pmksa {
	const u8 *bssid;
	const u8 *pmkid;
};

struct cfg80211_pkt_pattern {
	const u8 *mask, *pattern;
	int pattern_len;
	int pkt_offset;
};

struct cfg80211_wowlan_tcp {
	struct socket *sock;
	__be32 src, dst;
	u16 src_port, dst_port;
	u8 dst_mac[ETH_ALEN];
	int payload_len;
	const u8 *payload;
	struct nl80211_wowlan_tcp_data_seq payload_seq;
	u32 data_interval;
	u32 wake_len;
	const u8 *wake_data, *wake_mask;
	u32 tokens_size;
	
	struct nl80211_wowlan_tcp_data_token payload_tok;
};

struct cfg80211_wowlan {
	bool any, disconnect, magic_pkt, gtk_rekey_failure,
	     eap_identity_req, four_way_handshake,
	     rfkill_release;
	struct cfg80211_pkt_pattern *patterns;
	struct cfg80211_wowlan_tcp *tcp;
	int n_patterns;
};

struct cfg80211_coalesce_rules {
	int delay;
	enum nl80211_coalesce_condition condition;
	struct cfg80211_pkt_pattern *patterns;
	int n_patterns;
};

struct cfg80211_coalesce {
	struct cfg80211_coalesce_rules *rules;
	int n_rules;
};

struct cfg80211_wowlan_wakeup {
	bool disconnect, magic_pkt, gtk_rekey_failure,
	     eap_identity_req, four_way_handshake,
	     rfkill_release, packet_80211,
	     tcp_match, tcp_connlost, tcp_nomoretokens;
	s32 pattern_idx;
	u32 packet_present_len, packet_len;
	const void *packet;
};

struct cfg80211_gtk_rekey_data {
	const u8 *kek, *kck, *replay_ctr;
};

struct cfg80211_update_ft_ies_params {
	u16 md;
	const u8 *ie;
	size_t ie_len;
};

struct cfg80211_mgmt_tx_params {
	struct ieee80211_channel *chan;
	bool offchan;
	unsigned int wait;
	const u8 *buf;
	size_t len;
	bool no_cck;
	bool dont_wait_for_ack;
	int n_csa_offsets;
	const u16 *csa_offsets;
};

struct cfg80211_dscp_exception {
	u8 dscp;
	u8 up;
};

struct cfg80211_dscp_range {
	u8 low;
	u8 high;
};

#define IEEE80211_QOS_MAP_MAX_EX	21
#define IEEE80211_QOS_MAP_LEN_MIN	16
#define IEEE80211_QOS_MAP_LEN_MAX \
	(IEEE80211_QOS_MAP_LEN_MIN + 2 * IEEE80211_QOS_MAP_MAX_EX)

struct cfg80211_qos_map {
	u8 num_des;
	struct cfg80211_dscp_exception dscp_exception[IEEE80211_QOS_MAP_MAX_EX];
	struct cfg80211_dscp_range up[8];
};

struct cfg80211_ops {
	int	(*suspend)(struct wiphy *wiphy, struct cfg80211_wowlan *wow);
	int	(*resume)(struct wiphy *wiphy);
	void	(*set_wakeup)(struct wiphy *wiphy, bool enabled);

	struct wireless_dev * (*add_virtual_intf)(struct wiphy *wiphy,
						  const char *name,
						  enum nl80211_iftype type,
						  u32 *flags,
						  struct vif_params *params);
	int	(*del_virtual_intf)(struct wiphy *wiphy,
				    struct wireless_dev *wdev);
	int	(*change_virtual_intf)(struct wiphy *wiphy,
				       struct net_device *dev,
				       enum nl80211_iftype type, u32 *flags,
				       struct vif_params *params);

	int	(*add_key)(struct wiphy *wiphy, struct net_device *netdev,
			   u8 key_index, bool pairwise, const u8 *mac_addr,
			   struct key_params *params);
	int	(*get_key)(struct wiphy *wiphy, struct net_device *netdev,
			   u8 key_index, bool pairwise, const u8 *mac_addr,
			   void *cookie,
			   void (*callback)(void *cookie, struct key_params*));
	int	(*del_key)(struct wiphy *wiphy, struct net_device *netdev,
			   u8 key_index, bool pairwise, const u8 *mac_addr);
	int	(*set_default_key)(struct wiphy *wiphy,
				   struct net_device *netdev,
				   u8 key_index, bool unicast, bool multicast);
	int	(*set_default_mgmt_key)(struct wiphy *wiphy,
					struct net_device *netdev,
					u8 key_index);

	int	(*start_ap)(struct wiphy *wiphy, struct net_device *dev,
			    struct cfg80211_ap_settings *settings);
	int	(*change_beacon)(struct wiphy *wiphy, struct net_device *dev,
				 struct cfg80211_beacon_data *info);
	int	(*stop_ap)(struct wiphy *wiphy, struct net_device *dev);


	int	(*add_station)(struct wiphy *wiphy, struct net_device *dev,
			       const u8 *mac,
			       struct station_parameters *params);
	int	(*del_station)(struct wiphy *wiphy, struct net_device *dev,
			       const u8 *mac);
	int	(*change_station)(struct wiphy *wiphy, struct net_device *dev,
				  const u8 *mac,
				  struct station_parameters *params);
	int	(*get_station)(struct wiphy *wiphy, struct net_device *dev,
			       const u8 *mac, struct station_info *sinfo);
	int	(*dump_station)(struct wiphy *wiphy, struct net_device *dev,
				int idx, u8 *mac, struct station_info *sinfo);

	int	(*add_mpath)(struct wiphy *wiphy, struct net_device *dev,
			       const u8 *dst, const u8 *next_hop);
	int	(*del_mpath)(struct wiphy *wiphy, struct net_device *dev,
			       const u8 *dst);
	int	(*change_mpath)(struct wiphy *wiphy, struct net_device *dev,
				  const u8 *dst, const u8 *next_hop);
	int	(*get_mpath)(struct wiphy *wiphy, struct net_device *dev,
			     u8 *dst, u8 *next_hop, struct mpath_info *pinfo);
	int	(*dump_mpath)(struct wiphy *wiphy, struct net_device *dev,
			      int idx, u8 *dst, u8 *next_hop,
			      struct mpath_info *pinfo);
	int	(*get_mesh_config)(struct wiphy *wiphy,
				struct net_device *dev,
				struct mesh_config *conf);
	int	(*update_mesh_config)(struct wiphy *wiphy,
				      struct net_device *dev, u32 mask,
				      const struct mesh_config *nconf);
	int	(*join_mesh)(struct wiphy *wiphy, struct net_device *dev,
			     const struct mesh_config *conf,
			     const struct mesh_setup *setup);
	int	(*leave_mesh)(struct wiphy *wiphy, struct net_device *dev);

	int	(*change_bss)(struct wiphy *wiphy, struct net_device *dev,
			      struct bss_parameters *params);

	int	(*set_txq_params)(struct wiphy *wiphy, struct net_device *dev,
				  struct ieee80211_txq_params *params);

	int	(*libertas_set_mesh_channel)(struct wiphy *wiphy,
					     struct net_device *dev,
					     struct ieee80211_channel *chan);

	int	(*set_monitor_channel)(struct wiphy *wiphy,
				       struct cfg80211_chan_def *chandef);

	int	(*scan)(struct wiphy *wiphy,
			struct cfg80211_scan_request *request);

	int	(*auth)(struct wiphy *wiphy, struct net_device *dev,
			struct cfg80211_auth_request *req);
	int	(*assoc)(struct wiphy *wiphy, struct net_device *dev,
			 struct cfg80211_assoc_request *req);
	int	(*deauth)(struct wiphy *wiphy, struct net_device *dev,
			  struct cfg80211_deauth_request *req);
	int	(*disassoc)(struct wiphy *wiphy, struct net_device *dev,
			    struct cfg80211_disassoc_request *req);

	int	(*connect)(struct wiphy *wiphy, struct net_device *dev,
			   struct cfg80211_connect_params *sme);
	int	(*disconnect)(struct wiphy *wiphy, struct net_device *dev,
			      u16 reason_code);

	int	(*join_ibss)(struct wiphy *wiphy, struct net_device *dev,
			     struct cfg80211_ibss_params *params);
	int	(*leave_ibss)(struct wiphy *wiphy, struct net_device *dev);

	int	(*set_mcast_rate)(struct wiphy *wiphy, struct net_device *dev,
				  int rate[IEEE80211_NUM_BANDS]);

	int	(*set_wiphy_params)(struct wiphy *wiphy, u32 changed);

	int	(*set_tx_power)(struct wiphy *wiphy, struct wireless_dev *wdev,
				enum nl80211_tx_power_setting type, int mbm);
	int	(*get_tx_power)(struct wiphy *wiphy, struct wireless_dev *wdev,
				int *dbm);

	int	(*set_wds_peer)(struct wiphy *wiphy, struct net_device *dev,
				const u8 *addr);

	void	(*rfkill_poll)(struct wiphy *wiphy);

#ifdef CONFIG_NL80211_TESTMODE
	int	(*testmode_cmd)(struct wiphy *wiphy, struct wireless_dev *wdev,
				void *data, int len);
	int	(*testmode_dump)(struct wiphy *wiphy, struct sk_buff *skb,
				 struct netlink_callback *cb,
				 void *data, int len);
#endif

	int	(*set_bitrate_mask)(struct wiphy *wiphy,
				    struct net_device *dev,
				    const u8 *peer,
				    const struct cfg80211_bitrate_mask *mask);

	int	(*dump_survey)(struct wiphy *wiphy, struct net_device *netdev,
			int idx, struct survey_info *info);

	int	(*set_pmksa)(struct wiphy *wiphy, struct net_device *netdev,
			     struct cfg80211_pmksa *pmksa);
	int	(*del_pmksa)(struct wiphy *wiphy, struct net_device *netdev,
			     struct cfg80211_pmksa *pmksa);
	int	(*flush_pmksa)(struct wiphy *wiphy, struct net_device *netdev);

	int	(*remain_on_channel)(struct wiphy *wiphy,
				     struct wireless_dev *wdev,
				     struct ieee80211_channel *chan,
				     unsigned int duration,
				     u64 *cookie);
	int	(*cancel_remain_on_channel)(struct wiphy *wiphy,
					    struct wireless_dev *wdev,
					    u64 cookie);

	int	(*mgmt_tx)(struct wiphy *wiphy, struct wireless_dev *wdev,
			   struct cfg80211_mgmt_tx_params *params,
			   u64 *cookie);
	int	(*mgmt_tx_cancel_wait)(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       u64 cookie);

	int	(*set_power_mgmt)(struct wiphy *wiphy, struct net_device *dev,
				  bool enabled, int timeout);

	int	(*set_cqm_rssi_config)(struct wiphy *wiphy,
				       struct net_device *dev,
				       s32 rssi_thold, u32 rssi_hyst);

	int	(*set_cqm_txe_config)(struct wiphy *wiphy,
				      struct net_device *dev,
				      u32 rate, u32 pkts, u32 intvl);

	void	(*mgmt_frame_register)(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       u16 frame_type, bool reg);

	int	(*set_antenna)(struct wiphy *wiphy, u32 tx_ant, u32 rx_ant);
	int	(*get_antenna)(struct wiphy *wiphy, u32 *tx_ant, u32 *rx_ant);

	int	(*sched_scan_start)(struct wiphy *wiphy,
				struct net_device *dev,
				struct cfg80211_sched_scan_request *request);
	int	(*sched_scan_stop)(struct wiphy *wiphy, struct net_device *dev);

	int	(*set_rekey_data)(struct wiphy *wiphy, struct net_device *dev,
				  struct cfg80211_gtk_rekey_data *data);

	int	(*tdls_mgmt)(struct wiphy *wiphy, struct net_device *dev,
			     const u8 *peer, u8 action_code,  u8 dialog_token,
			     u16 status_code, u32 peer_capability,
			     bool initiator, const u8 *buf, size_t len);
	int	(*tdls_oper)(struct wiphy *wiphy, struct net_device *dev,
			     const u8 *peer, enum nl80211_tdls_operation oper);

	int	(*probe_client)(struct wiphy *wiphy, struct net_device *dev,
				const u8 *peer, u64 *cookie);

	int	(*set_noack_map)(struct wiphy *wiphy,
				  struct net_device *dev,
				  u16 noack_map);

	int	(*get_channel)(struct wiphy *wiphy,
			       struct wireless_dev *wdev,
			       struct cfg80211_chan_def *chandef);

	int	(*start_p2p_device)(struct wiphy *wiphy,
				    struct wireless_dev *wdev);
	void	(*stop_p2p_device)(struct wiphy *wiphy,
				   struct wireless_dev *wdev);

	int	(*set_mac_acl)(struct wiphy *wiphy, struct net_device *dev,
			       const struct cfg80211_acl_data *params);

	int	(*start_radar_detection)(struct wiphy *wiphy,
					 struct net_device *dev,
					 struct cfg80211_chan_def *chandef,
					 u32 cac_time_ms);
	int	(*update_ft_ies)(struct wiphy *wiphy, struct net_device *dev,
				 struct cfg80211_update_ft_ies_params *ftie);
	int	(*crit_proto_start)(struct wiphy *wiphy,
				    struct wireless_dev *wdev,
				    enum nl80211_crit_proto_id protocol,
				    u16 duration);
	void	(*crit_proto_stop)(struct wiphy *wiphy,
				   struct wireless_dev *wdev);
	int	(*set_coalesce)(struct wiphy *wiphy,
				struct cfg80211_coalesce *coalesce);

	int	(*channel_switch)(struct wiphy *wiphy,
				  struct net_device *dev,
				  struct cfg80211_csa_settings *params);

	int     (*set_qos_map)(struct wiphy *wiphy,
			       struct net_device *dev,
			       struct cfg80211_qos_map *qos_map);

	int	(*set_ap_chanwidth)(struct wiphy *wiphy, struct net_device *dev,
				    struct cfg80211_chan_def *chandef);

	int	(*add_tx_ts)(struct wiphy *wiphy, struct net_device *dev,
			     u8 tsid, const u8 *peer, u8 user_prio,
			     u16 admitted_time);
	int	(*del_tx_ts)(struct wiphy *wiphy, struct net_device *dev,
			     u8 tsid, const u8 *peer);
};


enum wiphy_flags {
	WIPHY_FLAG_SUPPORTS_WMM_ADMISSION	= BIT(0),
	
	
	WIPHY_FLAG_NETNS_OK			= BIT(3),
	WIPHY_FLAG_PS_ON_BY_DEFAULT		= BIT(4),
	WIPHY_FLAG_4ADDR_AP			= BIT(5),
	WIPHY_FLAG_4ADDR_STATION		= BIT(6),
	WIPHY_FLAG_CONTROL_PORT_PROTOCOL	= BIT(7),
	WIPHY_FLAG_IBSS_RSN			= BIT(8),
	WIPHY_FLAG_MESH_AUTH			= BIT(10),
	WIPHY_FLAG_SUPPORTS_SCHED_SCAN		= BIT(11),
	
	WIPHY_FLAG_SUPPORTS_FW_ROAM		= BIT(13),
	WIPHY_FLAG_AP_UAPSD			= BIT(14),
	WIPHY_FLAG_SUPPORTS_TDLS		= BIT(15),
	WIPHY_FLAG_TDLS_EXTERNAL_SETUP		= BIT(16),
	WIPHY_FLAG_HAVE_AP_SME			= BIT(17),
	WIPHY_FLAG_REPORTS_OBSS			= BIT(18),
	WIPHY_FLAG_AP_PROBE_RESP_OFFLOAD	= BIT(19),
	WIPHY_FLAG_OFFCHAN_TX			= BIT(20),
	WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL	= BIT(21),
	WIPHY_FLAG_SUPPORTS_5_10_MHZ		= BIT(22),
	WIPHY_FLAG_HAS_CHANNEL_SWITCH		= BIT(23),
	WIPHY_FLAG_DFS_OFFLOAD                  = BIT(24)
};

struct ieee80211_iface_limit {
	u16 max;
	u16 types;
};

struct ieee80211_iface_combination {
	const struct ieee80211_iface_limit *limits;
	u32 num_different_channels;
	u16 max_interfaces;
	u8 n_limits;
	bool beacon_int_infra_match;
	u8 radar_detect_widths;
	u8 radar_detect_regions;
};

struct ieee80211_txrx_stypes {
	u16 tx, rx;
};

enum wiphy_wowlan_support_flags {
	WIPHY_WOWLAN_ANY		= BIT(0),
	WIPHY_WOWLAN_MAGIC_PKT		= BIT(1),
	WIPHY_WOWLAN_DISCONNECT		= BIT(2),
	WIPHY_WOWLAN_SUPPORTS_GTK_REKEY	= BIT(3),
	WIPHY_WOWLAN_GTK_REKEY_FAILURE	= BIT(4),
	WIPHY_WOWLAN_EAP_IDENTITY_REQ	= BIT(5),
	WIPHY_WOWLAN_4WAY_HANDSHAKE	= BIT(6),
	WIPHY_WOWLAN_RFKILL_RELEASE	= BIT(7),
};

struct wiphy_wowlan_tcp_support {
	const struct nl80211_wowlan_tcp_data_token_feature *tok;
	u32 data_payload_max;
	u32 data_interval_max;
	u32 wake_payload_max;
	bool seq;
};

struct wiphy_wowlan_support {
	u32 flags;
	int n_patterns;
	int pattern_max_len;
	int pattern_min_len;
	int max_pkt_offset;
	const struct wiphy_wowlan_tcp_support *tcp;
};

struct wiphy_coalesce_support {
	int n_rules;
	int max_delay;
	int n_patterns;
	int pattern_max_len;
	int pattern_min_len;
	int max_pkt_offset;
};

enum wiphy_vendor_command_flags {
	WIPHY_VENDOR_CMD_NEED_WDEV = BIT(0),
	WIPHY_VENDOR_CMD_NEED_NETDEV = BIT(1),
	WIPHY_VENDOR_CMD_NEED_RUNNING = BIT(2),
};

struct wiphy_vendor_command {
	struct nl80211_vendor_cmd_info info;
	u32 flags;
	int (*doit)(struct wiphy *wiphy, struct wireless_dev *wdev,
		    const void *data, int data_len);
};

struct wiphy {
	

	
	u8 perm_addr[ETH_ALEN];
	u8 addr_mask[ETH_ALEN];

	struct mac_address *addresses;

	const struct ieee80211_txrx_stypes *mgmt_stypes;

	const struct ieee80211_iface_combination *iface_combinations;
	int n_iface_combinations;
	u16 software_iftypes;

	u16 n_addresses;

	
	u16 interface_modes;

	u16 max_acl_mac_addrs;

	u32 flags, regulatory_flags, features;

	u32 ap_sme_capa;

	enum cfg80211_signal_type signal_type;

	int bss_priv_size;
	u8 max_scan_ssids;
	u8 max_sched_scan_ssids;
	u8 max_match_sets;
	u16 max_scan_ie_len;
	u16 max_sched_scan_ie_len;

	int n_cipher_suites;
	const u32 *cipher_suites;

	u8 retry_short;
	u8 retry_long;
	u32 frag_threshold;
	u32 rts_threshold;
	u8 coverage_class;

	char fw_version[ETHTOOL_FWVERS_LEN];
	u32 hw_version;

#ifdef CONFIG_PM
	const struct wiphy_wowlan_support *wowlan;
	struct cfg80211_wowlan *wowlan_config;
#endif

	u16 max_remain_on_channel_duration;

	u8 max_num_pmkids;

	u32 available_antennas_tx;
	u32 available_antennas_rx;

	u32 probe_resp_offload;

	const u8 *extended_capabilities, *extended_capabilities_mask;
	u8 extended_capabilities_len;

	const void *privid;

	struct ieee80211_supported_band *bands[IEEE80211_NUM_BANDS];

	
	void (*reg_notifier)(struct wiphy *wiphy,
			     struct regulatory_request *request);

	

	const struct ieee80211_regdomain __rcu *regd;

	struct device dev;

	
	bool registered;

	
	struct dentry *debugfsdir;

	const struct ieee80211_ht_cap *ht_capa_mod_mask;
	const struct ieee80211_vht_cap *vht_capa_mod_mask;

#ifdef CONFIG_NET_NS
	
	struct net *_net;
#endif

#ifdef CONFIG_CFG80211_WEXT
	const struct iw_handler_def *wext;
#endif

	const struct wiphy_coalesce_support *coalesce;

	const struct wiphy_vendor_command *vendor_commands;
	const struct nl80211_vendor_cmd_info *vendor_events;
	int n_vendor_commands, n_vendor_events;

	u16 max_ap_assoc_sta;

	u8 max_num_csa_counters;
	u8 max_adj_channel_rssi_comp;

	char priv[0] __aligned(NETDEV_ALIGN);
};

static inline struct net *wiphy_net(struct wiphy *wiphy)
{
	return read_pnet(&wiphy->_net);
}

static inline void wiphy_net_set(struct wiphy *wiphy, struct net *net)
{
	write_pnet(&wiphy->_net, net);
}

static inline void *wiphy_priv(struct wiphy *wiphy)
{
	BUG_ON(!wiphy);
	return &wiphy->priv;
}

static inline struct wiphy *priv_to_wiphy(void *priv)
{
	BUG_ON(!priv);
	return container_of(priv, struct wiphy, priv);
}

static inline void set_wiphy_dev(struct wiphy *wiphy, struct device *dev)
{
	wiphy->dev.parent = dev;
}

static inline struct device *wiphy_dev(struct wiphy *wiphy)
{
	return wiphy->dev.parent;
}

static inline const char *wiphy_name(const struct wiphy *wiphy)
{
	return dev_name(&wiphy->dev);
}

struct wiphy *wiphy_new(const struct cfg80211_ops *ops, int sizeof_priv);

int wiphy_register(struct wiphy *wiphy);

void wiphy_unregister(struct wiphy *wiphy);

void wiphy_free(struct wiphy *wiphy);

struct cfg80211_conn;
struct cfg80211_internal_bss;
struct cfg80211_cached_keys;

struct wireless_dev {
	struct wiphy *wiphy;
	enum nl80211_iftype iftype;

	
	struct list_head list;
	struct net_device *netdev;

	u32 identifier;

	struct list_head mgmt_registrations;
	spinlock_t mgmt_registrations_lock;

	struct mutex mtx;

	bool use_4addr, p2p_started;

	u8 address[ETH_ALEN] __aligned(sizeof(u16));

	
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	u8 ssid_len, mesh_id_len, mesh_id_up_len;
	struct cfg80211_conn *conn;
	struct cfg80211_cached_keys *connect_keys;

	struct list_head event_list;
	spinlock_t event_lock;

	struct cfg80211_internal_bss *current_bss; 
	struct cfg80211_chan_def preset_chandef;
	struct cfg80211_chan_def chandef;

	bool ibss_fixed;
	bool ibss_dfs_possible;

	bool ps;
	int ps_timeout;

	int beacon_interval;

	u32 ap_unexpected_nlportid;

	bool cac_started;
	unsigned long cac_start_time;
	unsigned int cac_time_ms;

	u32 owner_nlportid;

#ifdef CONFIG_CFG80211_WEXT
	
	struct {
		struct cfg80211_ibss_params ibss;
		struct cfg80211_connect_params connect;
		struct cfg80211_cached_keys *keys;
		const u8 *ie;
		size_t ie_len;
		u8 bssid[ETH_ALEN], prev_bssid[ETH_ALEN];
		u8 ssid[IEEE80211_MAX_SSID_LEN];
		s8 default_key, default_mgmt_key;
		bool prev_bssid_valid;
	} wext;
#endif
};

static inline u8 *wdev_address(struct wireless_dev *wdev)
{
	if (wdev->netdev)
		return wdev->netdev->dev_addr;
	return wdev->address;
}

static inline void *wdev_priv(struct wireless_dev *wdev)
{
	BUG_ON(!wdev);
	return wiphy_priv(wdev->wiphy);
}


int ieee80211_channel_to_frequency(int chan, enum ieee80211_band band);

int ieee80211_frequency_to_channel(int freq);

struct ieee80211_channel *__ieee80211_get_channel(struct wiphy *wiphy,
						  int freq);
static inline struct ieee80211_channel *
ieee80211_get_channel(struct wiphy *wiphy, int freq)
{
	return __ieee80211_get_channel(wiphy, freq);
}

struct ieee80211_rate *
ieee80211_get_response_rate(struct ieee80211_supported_band *sband,
			    u32 basic_rates, int bitrate);

u32 ieee80211_mandatory_rates(struct ieee80211_supported_band *sband,
			      enum nl80211_bss_scan_width scan_width);


struct radiotap_align_size {
	uint8_t align:4, size:4;
};

struct ieee80211_radiotap_namespace {
	const struct radiotap_align_size *align_size;
	int n_bits;
	uint32_t oui;
	uint8_t subns;
};

struct ieee80211_radiotap_vendor_namespaces {
	const struct ieee80211_radiotap_namespace *ns;
	int n_ns;
};


struct ieee80211_radiotap_iterator {
	struct ieee80211_radiotap_header *_rtheader;
	const struct ieee80211_radiotap_vendor_namespaces *_vns;
	const struct ieee80211_radiotap_namespace *current_namespace;

	unsigned char *_arg, *_next_ns_data;
	__le32 *_next_bitmap;

	unsigned char *this_arg;
	int this_arg_index;
	int this_arg_size;

	int is_radiotap_ns;

	int _max_length;
	int _arg_index;
	uint32_t _bitmap_shifter;
	int _reset_on_ext;
};

int
ieee80211_radiotap_iterator_init(struct ieee80211_radiotap_iterator *iterator,
				 struct ieee80211_radiotap_header *radiotap_header,
				 int max_length,
				 const struct ieee80211_radiotap_vendor_namespaces *vns);

int
ieee80211_radiotap_iterator_next(struct ieee80211_radiotap_iterator *iterator);


extern const unsigned char rfc1042_header[6];
extern const unsigned char bridge_tunnel_header[6];

unsigned int ieee80211_get_hdrlen_from_skb(const struct sk_buff *skb);

unsigned int __attribute_const__ ieee80211_hdrlen(__le16 fc);

unsigned int ieee80211_get_mesh_hdrlen(struct ieee80211s_hdr *meshhdr);


int ieee80211_data_to_8023(struct sk_buff *skb, const u8 *addr,
			   enum nl80211_iftype iftype);

int ieee80211_data_from_8023(struct sk_buff *skb, const u8 *addr,
			     enum nl80211_iftype iftype, const u8 *bssid,
			     bool qos);

void ieee80211_amsdu_to_8023s(struct sk_buff *skb, struct sk_buff_head *list,
			      const u8 *addr, enum nl80211_iftype iftype,
			      const unsigned int extra_headroom,
			      bool has_80211_header);

unsigned int cfg80211_classify8021d(struct sk_buff *skb,
				    struct cfg80211_qos_map *qos_map);

const u8 *cfg80211_find_ie(u8 eid, const u8 *ies, int len);

const u8 *cfg80211_find_vendor_ie(unsigned int oui, u8 oui_type,
				  const u8 *ies, int len);


int regulatory_hint(struct wiphy *wiphy, const char *alpha2);

int regulatory_hint_user(const char *alpha2,
			 enum nl80211_user_reg_hint_type user_reg_hint_type);

void wiphy_apply_custom_regulatory(struct wiphy *wiphy,
				   const struct ieee80211_regdomain *regd);

const struct ieee80211_reg_rule *freq_reg_info(struct wiphy *wiphy,
					       u32 center_freq);

const char *reg_initiator_name(enum nl80211_reg_initiator initiator);


void cfg80211_scan_done(struct cfg80211_scan_request *request, bool aborted);

void cfg80211_sched_scan_results(struct wiphy *wiphy);

void cfg80211_sched_scan_stopped(struct wiphy *wiphy);

void cfg80211_sched_scan_stopped_rtnl(struct wiphy *wiphy);

struct cfg80211_bss * __must_check
cfg80211_inform_bss_width_frame(struct wiphy *wiphy,
				struct ieee80211_channel *rx_channel,
				enum nl80211_bss_scan_width scan_width,
				struct ieee80211_mgmt *mgmt, size_t len,
				s32 signal, gfp_t gfp);

static inline struct cfg80211_bss * __must_check
cfg80211_inform_bss_frame(struct wiphy *wiphy,
			  struct ieee80211_channel *rx_channel,
			  struct ieee80211_mgmt *mgmt, size_t len,
			  s32 signal, gfp_t gfp)
{
	return cfg80211_inform_bss_width_frame(wiphy, rx_channel,
					       NL80211_BSS_CHAN_WIDTH_20,
					       mgmt, len, signal, gfp);
}

enum cfg80211_bss_frame_type {
	CFG80211_BSS_FTYPE_UNKNOWN,
	CFG80211_BSS_FTYPE_BEACON,
	CFG80211_BSS_FTYPE_PRESP,
};

struct cfg80211_bss * __must_check
cfg80211_inform_bss_width(struct wiphy *wiphy,
			  struct ieee80211_channel *rx_channel,
			  enum nl80211_bss_scan_width scan_width,
			  enum cfg80211_bss_frame_type ftype,
			  const u8 *bssid, u64 tsf, u16 capability,
			  u16 beacon_interval, const u8 *ie, size_t ielen,
			  s32 signal, gfp_t gfp);

static inline struct cfg80211_bss * __must_check
cfg80211_inform_bss(struct wiphy *wiphy,
		    struct ieee80211_channel *rx_channel,
		    enum cfg80211_bss_frame_type ftype,
		    const u8 *bssid, u64 tsf, u16 capability,
		    u16 beacon_interval, const u8 *ie, size_t ielen,
		    s32 signal, gfp_t gfp)
{
	return cfg80211_inform_bss_width(wiphy, rx_channel,
					 NL80211_BSS_CHAN_WIDTH_20, ftype,
					 bssid, tsf, capability,
					 beacon_interval, ie, ielen, signal,
					 gfp);
}

struct cfg80211_bss *cfg80211_get_bss(struct wiphy *wiphy,
				      struct ieee80211_channel *channel,
				      const u8 *bssid,
				      const u8 *ssid, size_t ssid_len,
				      u16 capa_mask, u16 capa_val);
static inline struct cfg80211_bss *
cfg80211_get_ibss(struct wiphy *wiphy,
		  struct ieee80211_channel *channel,
		  const u8 *ssid, size_t ssid_len)
{
	return cfg80211_get_bss(wiphy, channel, NULL, ssid, ssid_len,
				WLAN_CAPABILITY_IBSS, WLAN_CAPABILITY_IBSS);
}

void cfg80211_ref_bss(struct wiphy *wiphy, struct cfg80211_bss *bss);

void cfg80211_put_bss(struct wiphy *wiphy, struct cfg80211_bss *bss);

void cfg80211_unlink_bss(struct wiphy *wiphy, struct cfg80211_bss *bss);

static inline enum nl80211_bss_scan_width
cfg80211_chandef_to_scan_width(const struct cfg80211_chan_def *chandef)
{
	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_5:
		return NL80211_BSS_CHAN_WIDTH_5;
	case NL80211_CHAN_WIDTH_10:
		return NL80211_BSS_CHAN_WIDTH_10;
	default:
		return NL80211_BSS_CHAN_WIDTH_20;
	}
}

void cfg80211_rx_mlme_mgmt(struct net_device *dev, const u8 *buf, size_t len);

void cfg80211_auth_timeout(struct net_device *dev, const u8 *addr);

void cfg80211_rx_assoc_resp(struct net_device *dev,
			    struct cfg80211_bss *bss,
			    const u8 *buf, size_t len,
			    int uapsd_queues);

void cfg80211_assoc_timeout(struct net_device *dev, struct cfg80211_bss *bss);

void cfg80211_tx_mlme_mgmt(struct net_device *dev, const u8 *buf, size_t len);

void cfg80211_rx_unprot_mlme_mgmt(struct net_device *dev,
				  const u8 *buf, size_t len);

void cfg80211_michael_mic_failure(struct net_device *dev, const u8 *addr,
				  enum nl80211_key_type key_type, int key_id,
				  const u8 *tsc, gfp_t gfp);

void cfg80211_ibss_joined(struct net_device *dev, const u8 *bssid,
			  struct ieee80211_channel *channel, gfp_t gfp);

void cfg80211_notify_new_peer_candidate(struct net_device *dev,
		const u8 *macaddr, const u8 *ie, u8 ie_len, gfp_t gfp);


void wiphy_rfkill_set_hw_state(struct wiphy *wiphy, bool blocked);

void wiphy_rfkill_start_polling(struct wiphy *wiphy);

void wiphy_rfkill_stop_polling(struct wiphy *wiphy);


struct sk_buff *__cfg80211_alloc_reply_skb(struct wiphy *wiphy,
					   enum nl80211_commands cmd,
					   enum nl80211_attrs attr,
					   int approxlen);

struct sk_buff *__cfg80211_alloc_event_skb(struct wiphy *wiphy,
					   struct wireless_dev *wdev,
					   enum nl80211_commands cmd,
					   enum nl80211_attrs attr,
					   int vendor_event_idx,
					   int approxlen, gfp_t gfp);

void __cfg80211_send_event_skb(struct sk_buff *skb, gfp_t gfp);

static inline struct sk_buff *
cfg80211_vendor_cmd_alloc_reply_skb(struct wiphy *wiphy, int approxlen)
{
	return __cfg80211_alloc_reply_skb(wiphy, NL80211_CMD_VENDOR,
					  NL80211_ATTR_VENDOR_DATA, approxlen);
}

int cfg80211_vendor_cmd_reply(struct sk_buff *skb);

static inline struct sk_buff *
cfg80211_vendor_event_alloc(struct wiphy *wiphy, struct wireless_dev *wdev,
			     int approxlen, int event_idx, gfp_t gfp)
{
	return __cfg80211_alloc_event_skb(wiphy, wdev, NL80211_CMD_VENDOR,
					  NL80211_ATTR_VENDOR_DATA,
					  event_idx, approxlen, gfp);
}

static inline void cfg80211_vendor_event(struct sk_buff *skb, gfp_t gfp)
{
	__cfg80211_send_event_skb(skb, gfp);
}

#ifdef CONFIG_NL80211_TESTMODE

static inline struct sk_buff *
cfg80211_testmode_alloc_reply_skb(struct wiphy *wiphy, int approxlen)
{
	return __cfg80211_alloc_reply_skb(wiphy, NL80211_CMD_TESTMODE,
					  NL80211_ATTR_TESTDATA, approxlen);
}

static inline int cfg80211_testmode_reply(struct sk_buff *skb)
{
	return cfg80211_vendor_cmd_reply(skb);
}

static inline struct sk_buff *
cfg80211_testmode_alloc_event_skb(struct wiphy *wiphy, int approxlen, gfp_t gfp)
{
	return __cfg80211_alloc_event_skb(wiphy, NULL, NL80211_CMD_TESTMODE,
					  NL80211_ATTR_TESTDATA, -1,
					  approxlen, gfp);
}

static inline void cfg80211_testmode_event(struct sk_buff *skb, gfp_t gfp)
{
	__cfg80211_send_event_skb(skb, gfp);
}

#define CFG80211_TESTMODE_CMD(cmd)	.testmode_cmd = (cmd),
#define CFG80211_TESTMODE_DUMP(cmd)	.testmode_dump = (cmd),
#else
#define CFG80211_TESTMODE_CMD(cmd)
#define CFG80211_TESTMODE_DUMP(cmd)
#endif

void cfg80211_connect_result(struct net_device *dev, const u8 *bssid,
			     const u8 *req_ie, size_t req_ie_len,
			     const u8 *resp_ie, size_t resp_ie_len,
			     u16 status, gfp_t gfp);

void cfg80211_roamed(struct net_device *dev,
		     struct ieee80211_channel *channel,
		     const u8 *bssid,
		     const u8 *req_ie, size_t req_ie_len,
		     const u8 *resp_ie, size_t resp_ie_len, gfp_t gfp);

void cfg80211_roamed_bss(struct net_device *dev, struct cfg80211_bss *bss,
			 const u8 *req_ie, size_t req_ie_len,
			 const u8 *resp_ie, size_t resp_ie_len, gfp_t gfp);

void cfg80211_disconnected(struct net_device *dev, u16 reason,
			   const u8 *ie, size_t ie_len, gfp_t gfp);

void cfg80211_ready_on_channel(struct wireless_dev *wdev, u64 cookie,
			       struct ieee80211_channel *chan,
			       unsigned int duration, gfp_t gfp);

void cfg80211_remain_on_channel_expired(struct wireless_dev *wdev, u64 cookie,
					struct ieee80211_channel *chan,
					gfp_t gfp);


void cfg80211_new_sta(struct net_device *dev, const u8 *mac_addr,
		      struct station_info *sinfo, gfp_t gfp);

void cfg80211_del_sta(struct net_device *dev, const u8 *mac_addr, gfp_t gfp);

void cfg80211_conn_failed(struct net_device *dev, const u8 *mac_addr,
			  enum nl80211_connect_failed_reason reason,
			  gfp_t gfp);

bool cfg80211_rx_mgmt(struct wireless_dev *wdev, int freq, int sig_dbm,
		      const u8 *buf, size_t len, u32 flags);

void cfg80211_mgmt_tx_status(struct wireless_dev *wdev, u64 cookie,
			     const u8 *buf, size_t len, bool ack, gfp_t gfp);


void cfg80211_cqm_rssi_notify(struct net_device *dev,
			      enum nl80211_cqm_rssi_threshold_event rssi_event,
			      gfp_t gfp);

void cfg80211_radar_event(struct wiphy *wiphy,
			  struct cfg80211_chan_def *chandef, gfp_t gfp);

void cfg80211_cac_event(struct net_device *netdev,
			const struct cfg80211_chan_def *chandef,
			enum nl80211_radar_event event, gfp_t gfp);


void cfg80211_cqm_pktloss_notify(struct net_device *dev,
				 const u8 *peer, u32 num_packets, gfp_t gfp);

void cfg80211_cqm_txe_notify(struct net_device *dev, const u8 *peer,
			     u32 num_packets, u32 rate, u32 intvl, gfp_t gfp);

void cfg80211_gtk_rekey_notify(struct net_device *dev, const u8 *bssid,
			       const u8 *replay_ctr, gfp_t gfp);

void cfg80211_pmksa_candidate_notify(struct net_device *dev, int index,
				     const u8 *bssid, bool preauth, gfp_t gfp);

bool cfg80211_rx_spurious_frame(struct net_device *dev,
				const u8 *addr, gfp_t gfp);

bool cfg80211_rx_unexpected_4addr_frame(struct net_device *dev,
					const u8 *addr, gfp_t gfp);

void cfg80211_probe_status(struct net_device *dev, const u8 *addr,
			   u64 cookie, bool acked, gfp_t gfp);

void cfg80211_report_obss_beacon(struct wiphy *wiphy,
				 const u8 *frame, size_t len,
				 int freq, int sig_dbm);

bool cfg80211_reg_can_beacon(struct wiphy *wiphy,
			     struct cfg80211_chan_def *chandef,
			     enum nl80211_iftype iftype);

void cfg80211_ch_switch_notify(struct net_device *dev,
			       struct cfg80211_chan_def *chandef);

bool ieee80211_operating_class_to_band(u8 operating_class,
				       enum ieee80211_band *band);

void cfg80211_tdls_oper_request(struct net_device *dev, const u8 *peer,
				enum nl80211_tdls_operation oper,
				u16 reason_code, gfp_t gfp);

u32 cfg80211_calculate_bitrate(struct rate_info *rate);

void cfg80211_unregister_wdev(struct wireless_dev *wdev);

struct cfg80211_ft_event_params {
	const u8 *ies;
	size_t ies_len;
	const u8 *target_ap;
	const u8 *ric_ies;
	size_t ric_ies_len;
};

void cfg80211_ft_event(struct net_device *netdev,
		       struct cfg80211_ft_event_params *ft_event);

int cfg80211_get_p2p_attr(const u8 *ies, unsigned int len,
			  enum ieee80211_p2p_attr_id attr,
			  u8 *buf, unsigned int bufsize);

void cfg80211_report_wowlan_wakeup(struct wireless_dev *wdev,
				   struct cfg80211_wowlan_wakeup *wakeup,
				   gfp_t gfp);

void cfg80211_crit_proto_stopped(struct wireless_dev *wdev, gfp_t gfp);

unsigned int ieee80211_get_num_supported_channels(struct wiphy *wiphy);

int cfg80211_check_combinations(struct wiphy *wiphy,
				const int num_different_channels,
				const u8 radar_detect,
				const int iftype_num[NUM_NL80211_IFTYPES]);

int cfg80211_iter_combinations(struct wiphy *wiphy,
			       const int num_different_channels,
			       const u8 radar_detect,
			       const int iftype_num[NUM_NL80211_IFTYPES],
			       void (*iter)(const struct ieee80211_iface_combination *c,
					    void *data),
			       void *data);

void cfg80211_stop_iface(struct wiphy *wiphy, struct wireless_dev *wdev,
			 gfp_t gfp);

void cfg80211_shutdown_all_interfaces(struct wiphy *wiphy);


void cfg80211_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info);

void cfg80211_ap_stopped(struct net_device *netdev, gfp_t gfp);
bool cfg80211_is_gratuitous_arp_unsolicited_na(struct sk_buff *skb);



#define wiphy_printk(level, wiphy, format, args...)		\
	dev_printk(level, &(wiphy)->dev, format, ##args)
#define wiphy_emerg(wiphy, format, args...)			\
	dev_emerg(&(wiphy)->dev, format, ##args)
#define wiphy_alert(wiphy, format, args...)			\
	dev_alert(&(wiphy)->dev, format, ##args)
#define wiphy_crit(wiphy, format, args...)			\
	dev_crit(&(wiphy)->dev, format, ##args)
#define wiphy_err(wiphy, format, args...)			\
	dev_err(&(wiphy)->dev, format, ##args)
#define wiphy_warn(wiphy, format, args...)			\
	dev_warn(&(wiphy)->dev, format, ##args)
#define wiphy_notice(wiphy, format, args...)			\
	dev_notice(&(wiphy)->dev, format, ##args)
#define wiphy_info(wiphy, format, args...)			\
	dev_info(&(wiphy)->dev, format, ##args)

#define wiphy_debug(wiphy, format, args...)			\
	wiphy_printk(KERN_DEBUG, wiphy, format, ##args)

#define wiphy_dbg(wiphy, format, args...)			\
	dev_dbg(&(wiphy)->dev, format, ##args)

#if defined(VERBOSE_DEBUG)
#define wiphy_vdbg	wiphy_dbg
#else
#define wiphy_vdbg(wiphy, format, args...)				\
({									\
	if (0)								\
		wiphy_printk(KERN_DEBUG, wiphy, format, ##args);	\
	0;								\
})
#endif

#define wiphy_WARN(wiphy, format, args...)			\
	WARN(1, "wiphy: %s\n" format, wiphy_name(wiphy), ##args);

#endif 
