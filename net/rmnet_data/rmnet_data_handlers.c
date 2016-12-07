/*
 * Copyright (c) 2013-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * RMNET Data ingress/egress handler
 *
 */

#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/rmnet_data.h>
#include <linux/net_map.h>
#include <linux/netdev_features.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include "rmnet_data_private.h"
#include "rmnet_data_config.h"
#include "rmnet_data_vnd.h"
#include "rmnet_map.h"
#include "rmnet_data_stats.h"
#include "rmnet_data_trace.h"

RMNET_LOG_MODULE(RMNET_DATA_LOGMASK_HANDLER);


void rmnet_egress_handler(struct sk_buff *skb,
			  struct rmnet_logical_ep_conf_s *ep);

#ifdef CONFIG_RMNET_DATA_DEBUG_PKT
unsigned int dump_pkt_rx;
module_param(dump_pkt_rx, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dump_pkt_rx, "Dump packets entering ingress handler");

unsigned int dump_pkt_tx;
module_param(dump_pkt_tx, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dump_pkt_tx, "Dump packets exiting egress handler");
#endif 

long gro_flush_time __read_mostly = 10000L;
module_param(gro_flush_time, long, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(gro_flush_time, "Flush GRO when spaced more than this");

#define RMNET_DATA_IP_VERSION_4 0x40
#define RMNET_DATA_IP_VERSION_6 0x60

#define RMNET_DATA_GRO_RCV_FAIL 0
#define RMNET_DATA_GRO_RCV_PASS 1


static inline void __rmnet_data_set_skb_proto(struct sk_buff *skb)
{
	switch (skb->data[0] & 0xF0) {
	case RMNET_DATA_IP_VERSION_4:
		skb->protocol = htons(ETH_P_IP);
		break;
	case RMNET_DATA_IP_VERSION_6:
		skb->protocol = htons(ETH_P_IPV6);
		break;
	default:
		skb->protocol = htons(ETH_P_MAP);
		break;
	}
}

#ifdef CONFIG_RMNET_DATA_DEBUG_PKT
void rmnet_print_packet(const struct sk_buff *skb, const char *dev, char dir)
{
	char buffer[200];
	unsigned int len, printlen;
	int i, buffloc = 0;

	switch (dir) {
	case 'r':
		printlen = dump_pkt_rx;
		break;

	case 't':
		printlen = dump_pkt_tx;
		break;

	default:
		printlen = 0;
		break;
	}

	if (!printlen)
		return;

	pr_err("[%s][%c] - PKT skb->len=%d skb->head=%pK skb->data=%pK\n",
	       dev, dir, skb->len, (void *)skb->head, (void *)skb->data);
	pr_err("[%s][%c] - PKT skb->tail=%pK skb->end=%pK\n",
	       dev, dir, skb_tail_pointer(skb), skb_end_pointer(skb));

	if (skb->len > 0)
		len = skb->len;
	else
		len = ((unsigned int)(uintptr_t)skb->end) -
		      ((unsigned int)(uintptr_t)skb->data);

	pr_err("[%s][%c] - PKT len: %d, printing first %d bytes\n",
		dev, dir, len, printlen);

	memset(buffer, 0, sizeof(buffer));
	for (i = 0; (i < printlen) && (i < len); i++) {
		if ((i%16) == 0) {
			pr_err("[%s][%c] - PKT%s\n", dev, dir, buffer);
			memset(buffer, 0, sizeof(buffer));
			buffloc = 0;
			buffloc += snprintf(&buffer[buffloc],
					sizeof(buffer)-buffloc, "%04X:",
					i);
		}

		buffloc += snprintf(&buffer[buffloc], sizeof(buffer)-buffloc,
					" %02x", skb->data[i]);

	}
	pr_err("[%s][%c] - PKT%s\n", dev, dir, buffer);
}
#else
void rmnet_print_packet(const struct sk_buff *skb, const char *dev, char dir)
{
	return;
}
#endif 


static rx_handler_result_t rmnet_bridge_handler(struct sk_buff *skb,
					struct rmnet_logical_ep_conf_s *ep)
{
	if (!ep->egress_dev) {
		LOGD("Missing egress device for packet arriving on %s",
		     skb->dev->name);
		rmnet_kfree_skb(skb, RMNET_STATS_SKBFREE_BRDG_NO_EGRESS);
	} else {
		rmnet_egress_handler(skb, ep);
	}

	return RX_HANDLER_CONSUMED;
}

#ifdef NET_SKBUFF_DATA_USES_OFFSET
static void rmnet_reset_mac_header(struct sk_buff *skb)
{
	skb->mac_header = 0;
	skb->mac_len = 0;
}
#else
static void rmnet_reset_mac_header(struct sk_buff *skb)
{
	skb->mac_header = skb->network_header;
	skb->mac_len = 0;
}
#endif 

static int rmnet_check_skb_can_gro(struct sk_buff *skb)
{
	switch (skb->data[0] & 0xF0) {
	case RMNET_DATA_IP_VERSION_4:
		if (ip_hdr(skb)->protocol == IPPROTO_TCP)
			return RMNET_DATA_GRO_RCV_PASS;
		break;
	case RMNET_DATA_IP_VERSION_6:
		if (ipv6_hdr(skb)->nexthdr == IPPROTO_TCP)
			return RMNET_DATA_GRO_RCV_PASS;
		
	}

	return RMNET_DATA_GRO_RCV_FAIL;
}

static void rmnet_optional_gro_flush(struct napi_struct *napi,
				     struct rmnet_logical_ep_conf_s *ep)
{
	struct timespec curr_time, diff;

	if (!gro_flush_time)
		return;

	if (unlikely(ep->flush_time.tv_sec == 0)) {
		getnstimeofday(&ep->flush_time);
	} else {
		getnstimeofday(&(curr_time));
		diff = timespec_sub(curr_time, ep->flush_time);
		if ((diff.tv_sec > 0) || (diff.tv_nsec > gro_flush_time)) {
			napi_gro_flush(napi, false);
			getnstimeofday(&ep->flush_time);
		}
	}
}

static rx_handler_result_t __rmnet_deliver_skb(struct sk_buff *skb,
					 struct rmnet_logical_ep_conf_s *ep)
{
	struct napi_struct *napi = NULL;
	gro_result_t gro_res;

	trace___rmnet_deliver_skb(skb);
	switch (ep->rmnet_mode) {
	case RMNET_EPMODE_NONE:
		return RX_HANDLER_PASS;

	case RMNET_EPMODE_BRIDGE:
		return rmnet_bridge_handler(skb, ep);

	case RMNET_EPMODE_VND:
		skb_reset_transport_header(skb);
		skb_reset_network_header(skb);
		switch (rmnet_vnd_rx_fixup(skb, skb->dev)) {
		case RX_HANDLER_CONSUMED:
			return RX_HANDLER_CONSUMED;

		case RX_HANDLER_PASS:
			skb->pkt_type = PACKET_HOST;
			rmnet_reset_mac_header(skb);
			if (rmnet_check_skb_can_gro(skb) &&
			    (skb->dev->features & NETIF_F_GRO)) {
				napi = get_current_napi_context();
				if (napi != NULL) {
					gro_res = napi_gro_receive(napi, skb);
					trace_rmnet_gro_downlink(gro_res);
					rmnet_optional_gro_flush(napi, ep);
				} else {
					WARN_ONCE(1, "current napi is NULL\n");
					netif_receive_skb(skb);
				}
			} else {
				netif_receive_skb(skb);
			}
			return RX_HANDLER_CONSUMED;
		}
		return RX_HANDLER_PASS;

	default:
		LOGD("Unkown ep mode %d", ep->rmnet_mode);
		rmnet_kfree_skb(skb, RMNET_STATS_SKBFREE_DELIVER_NO_EP);
		return RX_HANDLER_CONSUMED;
	}
}

static rx_handler_result_t rmnet_ingress_deliver_packet(struct sk_buff *skb,
					    struct rmnet_phys_ep_conf_s *config)
{
	if (!config) {
		LOGD("%s", "NULL physical EP provided");
		kfree_skb(skb);
		return RX_HANDLER_CONSUMED;
	}

	if (!(config->local_ep.refcount)) {
		LOGD("Packet on %s has no local endpoint configuration",
			skb->dev->name);
		rmnet_kfree_skb(skb, RMNET_STATS_SKBFREE_IPINGRESS_NO_EP);
		return RX_HANDLER_CONSUMED;
	}

	skb->dev = config->local_ep.egress_dev;

	return __rmnet_deliver_skb(skb, &(config->local_ep));
}


static rx_handler_result_t _rmnet_map_ingress_handler(struct sk_buff *skb,
					    struct rmnet_phys_ep_conf_s *config)
{
	struct rmnet_logical_ep_conf_s *ep;
	uint8_t mux_id;
	uint16_t len;
	int ckresult;

	if (RMNET_MAP_GET_CD_BIT(skb)) {
		if (config->ingress_data_format
		    & RMNET_INGRESS_FORMAT_MAP_COMMANDS)
			return rmnet_map_command(skb, config);

		LOGM("MAP command packet on %s; %s", skb->dev->name,
		     "Not configured for MAP commands");
		rmnet_kfree_skb(skb,
				RMNET_STATS_SKBFREE_INGRESS_NOT_EXPECT_MAPC);
		return RX_HANDLER_CONSUMED;
	}

	mux_id = RMNET_MAP_GET_MUX_ID(skb);
	len = RMNET_MAP_GET_LENGTH(skb)
			- RMNET_MAP_GET_PAD(skb)
			- config->tail_spacing;

	if (mux_id >= RMNET_DATA_MAX_LOGICAL_EP) {
		LOGD("Got packet on %s with bad mux id %d",
			skb->dev->name, mux_id);
		rmnet_kfree_skb(skb, RMNET_STATS_SKBFREE_MAPINGRESS_BAD_MUX);
			return RX_HANDLER_CONSUMED;
	}

	ep = &(config->muxed_ep[mux_id]);

	if (!ep->refcount) {
		LOGD("Packet on %s:%d; has no logical endpoint config",
		     skb->dev->name, mux_id);

		rmnet_kfree_skb(skb, RMNET_STATS_SKBFREE_MAPINGRESS_MUX_NO_EP);
		return RX_HANDLER_CONSUMED;
	}

	if (config->ingress_data_format & RMNET_INGRESS_FORMAT_DEMUXING)
		skb->dev = ep->egress_dev;

	if ((config->ingress_data_format & RMNET_INGRESS_FORMAT_MAP_CKSUMV3) ||
	    (config->ingress_data_format & RMNET_INGRESS_FORMAT_MAP_CKSUMV4)) {
		ckresult = rmnet_map_checksum_downlink_packet(skb);
		trace_rmnet_map_checksum_downlink_packet(skb, ckresult);
		rmnet_stats_dl_checksum(ckresult);
		if (likely((ckresult == RMNET_MAP_CHECKSUM_OK)
			    || (ckresult == RMNET_MAP_CHECKSUM_SKIPPED)))
			skb->ip_summed |= CHECKSUM_UNNECESSARY;
		else if (ckresult != RMNET_MAP_CHECKSUM_ERR_UNKNOWN_IP_VERSION
			&& ckresult != RMNET_MAP_CHECKSUM_ERR_UNKNOWN_TRANSPORT
			&& ckresult != RMNET_MAP_CHECKSUM_VALID_FLAG_NOT_SET
			&& ckresult != RMNET_MAP_CHECKSUM_FRAGMENTED_PACKET) {
			rmnet_kfree_skb(skb,
				RMNET_STATS_SKBFREE_INGRESS_BAD_MAP_CKSUM);
			return RX_HANDLER_CONSUMED;
		}
	}

	
	skb_pull(skb, sizeof(struct rmnet_map_header_s));
	skb_trim(skb, len);
	__rmnet_data_set_skb_proto(skb);
	return __rmnet_deliver_skb(skb, ep);
}

static rx_handler_result_t rmnet_map_ingress_handler(struct sk_buff *skb,
					    struct rmnet_phys_ep_conf_s *config)
{
	struct sk_buff *skbn;
	int rc, co = 0;

	if (config->ingress_data_format & RMNET_INGRESS_FORMAT_DEAGGREGATION) {
		trace_rmnet_start_deaggregation(skb);
		while ((skbn = rmnet_map_deaggregate(skb, config)) != 0) {
			_rmnet_map_ingress_handler(skbn, config);
			co++;
		}
		trace_rmnet_end_deaggregation(skb, co);
		LOGD("De-aggregated %d packets", co);
		rmnet_stats_deagg_pkts(co);
		rmnet_kfree_skb(skb, RMNET_STATS_SKBFREE_MAPINGRESS_AGGBUF);
		rc = RX_HANDLER_CONSUMED;
	} else {
		rc = _rmnet_map_ingress_handler(skb, config);
	}

	return rc;
}

static int rmnet_map_egress_handler(struct sk_buff *skb,
				    struct rmnet_phys_ep_conf_s *config,
				    struct rmnet_logical_ep_conf_s *ep,
				    struct net_device *orig_dev)
{
	int required_headroom, additional_header_length, ckresult;
	struct rmnet_map_header_s *map_header;

	additional_header_length = 0;

	required_headroom = sizeof(struct rmnet_map_header_s);
	if ((config->egress_data_format & RMNET_EGRESS_FORMAT_MAP_CKSUMV3) ||
	    (config->egress_data_format & RMNET_EGRESS_FORMAT_MAP_CKSUMV4)) {
		required_headroom +=
			sizeof(struct rmnet_map_ul_checksum_header_s);
		additional_header_length +=
			sizeof(struct rmnet_map_ul_checksum_header_s);
	}

	LOGD("headroom of %d bytes", required_headroom);

	if (skb_headroom(skb) < required_headroom) {
		if (pskb_expand_head(skb, required_headroom, 0, GFP_KERNEL)) {
			LOGD("Failed to add headroom of %d bytes",
			     required_headroom);
			return 1;
		}
	}

	if ((config->egress_data_format & RMNET_EGRESS_FORMAT_MAP_CKSUMV3) ||
	    (config->egress_data_format & RMNET_EGRESS_FORMAT_MAP_CKSUMV4)) {
		ckresult = rmnet_map_checksum_uplink_packet
				(skb, orig_dev, config->egress_data_format);
		trace_rmnet_map_checksum_uplink_packet(orig_dev, ckresult);
		rmnet_stats_ul_checksum(ckresult);
	}

	if ((!(config->egress_data_format &
	    RMNET_EGRESS_FORMAT_AGGREGATION)) ||
	    ((orig_dev->features & NETIF_F_GSO) && skb_is_nonlinear(skb)))
		map_header = rmnet_map_add_map_header
		(skb, additional_header_length, RMNET_MAP_NO_PAD_BYTES);
	else
		map_header = rmnet_map_add_map_header
		(skb, additional_header_length, RMNET_MAP_ADD_PAD_BYTES);

	if (!map_header) {
		LOGD("%s", "Failed to add MAP header to egress packet");
		return 1;
	}

	if (config->egress_data_format & RMNET_EGRESS_FORMAT_MUXING) {
		if (ep->mux_id == 0xff)
			map_header->mux_id = 0;
		else
			map_header->mux_id = ep->mux_id;
	}

	skb->protocol = htons(ETH_P_MAP);

	if (config->egress_data_format & RMNET_EGRESS_FORMAT_AGGREGATION) {
		rmnet_map_aggregate(skb, config);
		return RMNET_MAP_CONSUMED;
	}

	return RMNET_MAP_SUCCESS;
}

rx_handler_result_t rmnet_ingress_handler(struct sk_buff *skb)
{
	struct rmnet_phys_ep_conf_s *config;
	struct net_device *dev;
	int rc;

	if (!skb)
		BUG();

	dev = skb->dev;
	trace_rmnet_ingress_handler(skb);
	rmnet_print_packet(skb, dev->name, 'r');

	config = (struct rmnet_phys_ep_conf_s *)
		rcu_dereference(skb->dev->rx_handler_data);

	if (!config) {
		LOGD("%s is not associated with rmnet_data", skb->dev->name);
		kfree_skb(skb);
		return RX_HANDLER_CONSUMED;
	}

	if (config->ingress_data_format & RMNET_INGRESS_FIX_ETHERNET) {
		skb_push(skb, RMNET_ETHERNET_HEADER_LENGTH);
		__rmnet_data_set_skb_proto(skb);
	}

	if (config->ingress_data_format & RMNET_INGRESS_FORMAT_MAP) {
			rc = rmnet_map_ingress_handler(skb, config);
	} else {
		switch (ntohs(skb->protocol)) {
		case ETH_P_MAP:
			if (config->local_ep.rmnet_mode ==
				RMNET_EPMODE_BRIDGE) {
				rc = rmnet_ingress_deliver_packet(skb, config);
			} else {
				LOGD("MAP packet on %s; MAP not set",
					dev->name);
				rmnet_kfree_skb(skb,
				   RMNET_STATS_SKBFREE_INGRESS_NOT_EXPECT_MAPD);
				rc = RX_HANDLER_CONSUMED;
			}
			break;

		case ETH_P_ARP:
		case ETH_P_IP:
		case ETH_P_IPV6:
			rc = rmnet_ingress_deliver_packet(skb, config);
			break;

		default:
			LOGD("Unknown skb->proto 0x%04X\n",
				ntohs(skb->protocol) & 0xFFFF);
			rc = RX_HANDLER_PASS;
		}
	}

	return rc;
}

rx_handler_result_t rmnet_rx_handler(struct sk_buff **pskb)
{
	return rmnet_ingress_handler(*pskb);
}

void rmnet_egress_handler(struct sk_buff *skb,
			  struct rmnet_logical_ep_conf_s *ep)
{
	struct rmnet_phys_ep_conf_s *config;
	struct net_device *orig_dev;
	int rc;
	orig_dev = skb->dev;
	skb->dev = ep->egress_dev;

	config = (struct rmnet_phys_ep_conf_s *)
		rcu_dereference(skb->dev->rx_handler_data);

	if (!config) {
		LOGD("%s is not associated with rmnet_data", skb->dev->name);
		kfree_skb(skb);
		return;
	}

	LOGD("Packet going out on %s with egress format 0x%08X",
	     skb->dev->name, config->egress_data_format);

	if (config->egress_data_format & RMNET_EGRESS_FORMAT_MAP) {
		switch (rmnet_map_egress_handler(skb, config, ep, orig_dev)) {
		case RMNET_MAP_CONSUMED:
			LOGD("%s", "MAP process consumed packet");
			return;

		case RMNET_MAP_SUCCESS:
			break;

		default:
			LOGD("MAP egress failed on packet on %s",
			     skb->dev->name);
			rmnet_kfree_skb(skb, RMNET_STATS_SKBFREE_EGR_MAPFAIL);
			return;
		}
	}

	if (ep->rmnet_mode == RMNET_EPMODE_VND)
		rmnet_vnd_tx_fixup(skb, orig_dev);

	rmnet_print_packet(skb, skb->dev->name, 't');
	trace_rmnet_egress_handler(skb);
	rc = dev_queue_xmit(skb);
	if (rc != 0) {
		LOGD("Failed to queue packet for transmission on [%s]",
		      skb->dev->name);
	}
	rmnet_stats_queue_xmit(rc, RMNET_STATS_QUEUE_XMIT_EGRESS);
}
