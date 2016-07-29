/*
 * u_ether.h -- interface to USB gadget "ethernet link" utilities
 *
 * Copyright (C) 2003-2005,2008 David Brownell
 * Copyright (C) 2003-2004 Robert Schwebel, Benedikt Spranger
 * Copyright (C) 2008 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __U_ETHER_H
#define __U_ETHER_H

#include <linux/err.h>
#include <linux/if_ether.h>
#include <linux/usb/composite.h>
#include <linux/usb/cdc.h>
#include <linux/netdevice.h>

#include "gadget_chips.h"

#define QMULT_DEFAULT 5

#define USB_ETHERNET_MODULE_PARAMETERS() \
	static unsigned qmult = QMULT_DEFAULT;				\
	module_param(qmult, uint, S_IRUGO|S_IWUSR);			\
	MODULE_PARM_DESC(qmult, "queue length multiplier at high/super speed");\
									\
	static char *dev_addr;						\
	module_param(dev_addr, charp, S_IRUGO);				\
	MODULE_PARM_DESC(dev_addr, "Device Ethernet Address");		\
									\
	static char *host_addr;						\
	module_param(host_addr, charp, S_IRUGO);			\
	MODULE_PARM_DESC(host_addr, "Host Ethernet Address")

struct eth_dev;

struct gether {
	struct usb_function		func;

	
	struct eth_dev			*ioport;

	
	struct usb_ep			*in_ep;
	struct usb_ep			*out_ep;

	bool				is_zlp_ok;

	u16				cdc_filter;

	
	u32				header_len;
	
	bool				is_fixed;
	u32				fixed_out_len;
	u32				fixed_in_len;
	unsigned			ul_max_pkts_per_xfer;
	uint32_t			dl_max_pkts_per_xfer;
	uint32_t			dl_max_xfer_size;
	bool				multi_pkt_xfer;
	bool				rx_trigger_enabled;
	bool				rx_triggered;
	struct sk_buff			*(*wrap)(struct gether *port,
						struct sk_buff *skb);
	int				(*unwrap)(struct gether *port,
						struct sk_buff *skb,
						struct sk_buff_head *list);

	
	void				(*open)(struct gether *);
	void				(*close)(struct gether *);
	struct rndis_packet_msg_type	*header;
};

#define	DEFAULT_FILTER	(USB_CDC_PACKET_TYPE_BROADCAST \
			|USB_CDC_PACKET_TYPE_ALL_MULTICAST \
			|USB_CDC_PACKET_TYPE_PROMISCUOUS \
			|USB_CDC_PACKET_TYPE_DIRECTED)

struct eth_dev *gether_setup_name(struct usb_gadget *g,
		const char *dev_addr, const char *host_addr,
		u8 ethaddr[ETH_ALEN], unsigned qmult, const char *netname);

static inline struct eth_dev *gether_setup(struct usb_gadget *g,
		const char *dev_addr, const char *host_addr,
		u8 ethaddr[ETH_ALEN], unsigned qmult)
{
	return gether_setup_name(g, dev_addr, host_addr, ethaddr, qmult, "usb");
}

struct net_device *gether_setup_name_default(const char *netname);

int gether_register_netdev(struct net_device *net);
void gether_update_dl_max_pkts_per_xfer(struct gether *link, uint32_t n);
void gether_update_dl_max_xfer_size(struct gether *link, uint32_t s);
void gether_enable_sg(struct gether *link, bool);

static inline struct net_device *gether_setup_default(void)
{
	return gether_setup_name_default("usb");
}

void gether_set_gadget(struct net_device *net, struct usb_gadget *g);

int gether_set_dev_addr(struct net_device *net, const char *dev_addr);

int gether_get_dev_addr(struct net_device *net, char *dev_addr, int len);

int gether_set_host_addr(struct net_device *net, const char *host_addr);

int gether_get_host_addr(struct net_device *net, char *host_addr, int len);

int gether_get_host_addr_cdc(struct net_device *net, char *host_addr, int len);

void gether_get_host_addr_u8(struct net_device *net, u8 host_mac[ETH_ALEN]);

void gether_set_qmult(struct net_device *net, unsigned qmult);

unsigned gether_get_qmult(struct net_device *net);

int gether_get_ifname(struct net_device *net, char *name, int len);

void gether_cleanup(struct eth_dev *dev);

int gether_change_mtu(int new_mtu);

struct net_device *gether_connect(struct gether *);
void gether_disconnect(struct gether *);
int gether_up(struct gether *);

static inline bool can_support_ecm(struct usb_gadget *gadget)
{
	if (!gadget_supports_altsettings(gadget))
		return false;

	return true;
}

int rndis_rx_trigger(bool);

#endif 
