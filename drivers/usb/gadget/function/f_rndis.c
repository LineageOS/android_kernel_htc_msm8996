/*
 * f_rndis.c -- RNDIS link function driver
 *
 * Copyright (C) 2003-2005,2008 David Brownell
 * Copyright (C) 2003-2004 Robert Schwebel, Benedikt Spranger
 * Copyright (C) 2008 Nokia Corporation
 * Copyright (C) 2009 Samsung Electronics
 *                    Author: Michal Nazarewicz (mina86@mina86.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/etherdevice.h>

#include <linux/atomic.h>

#include "u_ether.h"
#include "u_ether_configfs.h"
#include "u_rndis.h"
#include "rndis.h"
#include "../configfs.h"


static unsigned int rndis_dl_max_pkt_per_xfer = 3;
module_param(rndis_dl_max_pkt_per_xfer, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(rndis_dl_max_pkt_per_xfer,
	"Maximum packets per transfer for DL aggregation");

static unsigned int rndis_ul_max_pkt_per_xfer = 3;
module_param(rndis_ul_max_pkt_per_xfer, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(rndis_ul_max_pkt_per_xfer,
       "Maximum packets per transfer for UL aggregation");

static unsigned int rx_trigger_enabled;
module_param(rx_trigger_enabled, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(rx_trigger_enabled, "rx trigger_enable");

struct f_rndis {
	struct gether			port;
	u8				ctrl_id, data_id;
	u8				ethaddr[ETH_ALEN];
	u32				vendorID;
	const char			*manufacturer;
	int				config;

	struct usb_ep			*notify;
	struct usb_request		*notify_req;
	atomic_t			notify_count;
};

static struct f_rndis *__rndis;

int
rndis_rx_trigger(bool write)
{
	struct f_rndis *rndis = __rndis;

	if (!rx_trigger_enabled || !rndis) {
		pr_err("can't set rx trigger\n");
		return -EINVAL;
	}
	if (!write)
		return rndis->port.rx_triggered;

	if (rndis->port.rx_triggered)
		return 0;


	rndis->port.rx_triggered = true;
	gether_up(&rndis->port);

	return 0;
}

static inline struct f_rndis *func_to_rndis(struct usb_function *f)
{
	return container_of(f, struct f_rndis, port.func);
}

static unsigned int bitrate(struct usb_gadget *g)
{
	if (gadget_is_superspeed(g) && g->speed == USB_SPEED_SUPER)
		return 13 * 1024 * 8 * 1000 * 8;
	else if (gadget_is_dualspeed(g) && g->speed == USB_SPEED_HIGH)
		return 13 * 512 * 8 * 1000 * 8;
	else
		return 19 * 64 * 1 * 1000 * 8;
}



#define RNDIS_STATUS_INTERVAL_MS	32
#define STATUS_BYTECOUNT		8	



static struct usb_interface_descriptor rndis_control_intf = {
	.bLength =		sizeof rndis_control_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	
	
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_COMM,
	.bInterfaceSubClass =   USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol =   USB_CDC_ACM_PROTO_VENDOR,
	
};

static struct usb_cdc_header_desc header_desc = {
	.bLength =		sizeof header_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,

	.bcdCDC =		cpu_to_le16(0x0110),
};

static struct usb_cdc_call_mgmt_descriptor call_mgmt_descriptor = {
	.bLength =		sizeof call_mgmt_descriptor,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_CALL_MANAGEMENT_TYPE,

	.bmCapabilities =	0x00,
	.bDataInterface =	0x01,
};

static struct usb_cdc_acm_descriptor rndis_acm_descriptor = {
	.bLength =		sizeof rndis_acm_descriptor,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_ACM_TYPE,

	.bmCapabilities =	0x00,
};

static struct usb_cdc_union_desc rndis_union_desc = {
	.bLength =		sizeof(rndis_union_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	
	
};


static struct usb_interface_descriptor rndis_data_intf = {
	.bLength =		sizeof rndis_data_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	
};


static struct usb_interface_assoc_descriptor
rndis_iad_descriptor = {
	.bLength =		sizeof rndis_iad_descriptor,
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface =	0, 
	.bInterfaceCount = 	2,	
	.bFunctionClass =	USB_CLASS_COMM,
	.bFunctionSubClass =	USB_CDC_SUBCLASS_ETHERNET,
	.bFunctionProtocol =	USB_CDC_PROTO_NONE,
	
};


static struct usb_endpoint_descriptor fs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(STATUS_BYTECOUNT),
	.bInterval =		RNDIS_STATUS_INTERVAL_MS,
};

static struct usb_endpoint_descriptor fs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *eth_fs_function[] = {
	(struct usb_descriptor_header *) &rndis_iad_descriptor,

	
	(struct usb_descriptor_header *) &rndis_control_intf,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &call_mgmt_descriptor,
	(struct usb_descriptor_header *) &rndis_acm_descriptor,
	(struct usb_descriptor_header *) &rndis_union_desc,
	(struct usb_descriptor_header *) &fs_notify_desc,

	
	(struct usb_descriptor_header *) &rndis_data_intf,
	(struct usb_descriptor_header *) &fs_in_desc,
	(struct usb_descriptor_header *) &fs_out_desc,
	NULL,
};


static struct usb_endpoint_descriptor hs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(STATUS_BYTECOUNT),
	.bInterval =		USB_MS_TO_HS_INTERVAL(RNDIS_STATUS_INTERVAL_MS)
};

static struct usb_endpoint_descriptor hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *eth_hs_function[] = {
	(struct usb_descriptor_header *) &rndis_iad_descriptor,

	
	(struct usb_descriptor_header *) &rndis_control_intf,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &call_mgmt_descriptor,
	(struct usb_descriptor_header *) &rndis_acm_descriptor,
	(struct usb_descriptor_header *) &rndis_union_desc,
	(struct usb_descriptor_header *) &hs_notify_desc,

	
	(struct usb_descriptor_header *) &rndis_data_intf,
	(struct usb_descriptor_header *) &hs_in_desc,
	(struct usb_descriptor_header *) &hs_out_desc,
	NULL,
};


static struct usb_endpoint_descriptor ss_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(STATUS_BYTECOUNT),
	.bInterval =		USB_MS_TO_HS_INTERVAL(RNDIS_STATUS_INTERVAL_MS)
};

static struct usb_ss_ep_comp_descriptor ss_intr_comp_desc = {
	.bLength =		sizeof ss_intr_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	
	
	
	.wBytesPerInterval =	cpu_to_le16(STATUS_BYTECOUNT),
};

static struct usb_endpoint_descriptor ss_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor ss_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor ss_bulk_comp_desc = {
	.bLength =		sizeof ss_bulk_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	
	
	
};

static struct usb_descriptor_header *eth_ss_function[] = {
	(struct usb_descriptor_header *) &rndis_iad_descriptor,

	
	(struct usb_descriptor_header *) &rndis_control_intf,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &call_mgmt_descriptor,
	(struct usb_descriptor_header *) &rndis_acm_descriptor,
	(struct usb_descriptor_header *) &rndis_union_desc,
	(struct usb_descriptor_header *) &ss_notify_desc,
	(struct usb_descriptor_header *) &ss_intr_comp_desc,

	
	(struct usb_descriptor_header *) &rndis_data_intf,
	(struct usb_descriptor_header *) &ss_in_desc,
	(struct usb_descriptor_header *) &ss_bulk_comp_desc,
	(struct usb_descriptor_header *) &ss_out_desc,
	(struct usb_descriptor_header *) &ss_bulk_comp_desc,
	NULL,
};


static struct usb_string rndis_string_defs[] = {
	[0].s = "RNDIS Communications Control",
	[1].s = "RNDIS Ethernet Data",
	[2].s = "RNDIS",
	{  } 
};

static struct usb_gadget_strings rndis_string_table = {
	.language =		0x0409,	
	.strings =		rndis_string_defs,
};

static struct usb_gadget_strings *rndis_strings[] = {
	&rndis_string_table,
	NULL,
};


static struct sk_buff *rndis_add_header(struct gether *port,
					struct sk_buff *skb)
{
	struct sk_buff *skb2;
	struct rndis_packet_msg_type *header = NULL;
	struct f_rndis *rndis = func_to_rndis(&port->func);
	struct usb_composite_dev *cdev = port->func.config->cdev;

	if (rndis->port.multi_pkt_xfer || cdev->gadget->sg_supported) {
		if (port->header) {
			header = port->header;
			header->MessageType = cpu_to_le32(RNDIS_MSG_PACKET);
			header->MessageLength = cpu_to_le32(skb->len +
							sizeof(*header));
			header->DataOffset = cpu_to_le32(36);
			header->DataLength = cpu_to_le32(skb->len);
			pr_debug("MessageLength:%d DataLength:%d\n",
						header->MessageLength,
						header->DataLength);
			return skb;
		} else {
			dev_kfree_skb_any(skb);
			pr_err("RNDIS header is NULL.\n");
			return NULL;
		}
	} else {
		skb2 = skb_realloc_headroom(skb,
				sizeof(struct rndis_packet_msg_type));
		if (skb2)
			rndis_add_hdr(skb2);

		dev_kfree_skb_any(skb);
		return skb2;
	}
}

static void rndis_response_available(void *_rndis)
{
	struct f_rndis			*rndis = _rndis;
	struct usb_request		*req = rndis->notify_req;
	struct usb_composite_dev	*cdev = rndis->port.func.config->cdev;
	__le32				*data = req->buf;
	int				status;

	if (atomic_inc_return(&rndis->notify_count) != 1)
		return;

	if (!rndis->notify->driver_data)
		return;
	data[0] = cpu_to_le32(1);
	data[1] = cpu_to_le32(0);

	status = usb_ep_queue(rndis->notify, req, GFP_ATOMIC);
	if (status) {
		atomic_dec(&rndis->notify_count);
		DBG(cdev, "notify/0 --> %d\n", status);
	}
}

static void rndis_response_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_rndis			*rndis = req->context;
	struct usb_composite_dev	*cdev;
	int				status = req->status;

	if (!rndis->port.func.config || !rndis->port.func.config->cdev)
		return;
	else
		cdev = rndis->port.func.config->cdev;

	switch (status) {
	case -ECONNRESET:
	case -ESHUTDOWN:
		
		atomic_set(&rndis->notify_count, 0);
		break;
	default:
		DBG(cdev, "RNDIS %s response error %d, %d/%d\n",
			ep->name, status,
			req->actual, req->length);
		
	case 0:
		if (ep != rndis->notify)
			break;

		if (atomic_dec_and_test(&rndis->notify_count))
			break;
		status = usb_ep_queue(rndis->notify, req, GFP_ATOMIC);
		if (status) {
			atomic_dec(&rndis->notify_count);
			DBG(cdev, "notify/1 --> %d\n", status);
		}
		break;
	}
}

static void rndis_command_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_rndis			*rndis = req->context;
	struct usb_composite_dev	*cdev;
	int				status;
	rndis_init_msg_type		*buf;

	if (!rndis->port.func.config || !rndis->port.func.config->cdev)
		return;
	else
		cdev = rndis->port.func.config->cdev;

	
	status = rndis_msg_parser(rndis->config, (u8 *) req->buf);
	if (status < 0)
		pr_err("RNDIS command error %d, %d/%d\n",
			status, req->actual, req->length);

	buf = (rndis_init_msg_type *)req->buf;

	if (buf->MessageType == RNDIS_MSG_INIT) {
		if (cdev->gadget->sg_supported) {
			rndis->port.dl_max_xfer_size = buf->MaxTransferSize;
			gether_update_dl_max_xfer_size(&rndis->port,
					rndis->port.dl_max_xfer_size);

			rndis->port.dl_max_pkts_per_xfer = 3;

			gether_update_dl_max_pkts_per_xfer(&rndis->port,
					 rndis->port.dl_max_pkts_per_xfer);

			return;
		}

		if (buf->MaxTransferSize > 2048)
			rndis->port.multi_pkt_xfer = 1;
		else
			rndis->port.multi_pkt_xfer = 0;
		pr_info("%s: MaxTransferSize: %d : Multi_pkt_txr: %s\n",
				__func__, buf->MaxTransferSize,
				rndis->port.multi_pkt_xfer ? "enabled" :
							    "disabled");
		if (rndis_dl_max_pkt_per_xfer <= 1)
			rndis->port.multi_pkt_xfer = 0;
	}
}

static int
rndis_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_rndis		*rndis = func_to_rndis(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_SEND_ENCAPSULATED_COMMAND:
		if (w_value || w_index != rndis->ctrl_id)
			goto invalid;
		
		value = w_length;
		req->complete = rndis_command_complete;
		req->context = rndis;
		
		break;

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_GET_ENCAPSULATED_RESPONSE:
		if (w_value || w_index != rndis->ctrl_id)
			goto invalid;
		else {
			u8 *buf;
			u32 n;

			
			buf = rndis_get_next_response(rndis->config, &n);
			if (buf) {
				memcpy(req->buf, buf, n);
				req->complete = rndis_response_complete;
				req->context = rndis;
				rndis_free_response(rndis->config, buf);
				value = n;
			}
			
		}
		break;

	default:
invalid:
		VDBG(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	
	if (value >= 0) {
		DBG(cdev, "rndis req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = (value < w_length);
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "rndis response on err %d\n", value);
	}

	
	return value;
}


static int rndis_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_rndis		*rndis = func_to_rndis(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	

	if (intf == rndis->ctrl_id) {
		if (rndis->notify->driver_data) {
			VDBG(cdev, "reset rndis control %d\n", intf);
			usb_ep_disable(rndis->notify);
		}
		if (!rndis->notify->desc) {
			VDBG(cdev, "init rndis ctrl %d\n", intf);
			if (config_ep_by_speed(cdev->gadget, f, rndis->notify))
				goto fail;
		}
		usb_ep_enable(rndis->notify);
		rndis->notify->driver_data = rndis;

	} else if (intf == rndis->data_id) {
		struct net_device	*net;

		rndis->port.rx_triggered = false;

		if (rndis->port.in_ep->driver_data) {
			DBG(cdev, "reset rndis\n");
			gether_disconnect(&rndis->port);
		}

		if (!rndis->port.in_ep->desc || !rndis->port.out_ep->desc) {
			DBG(cdev, "init rndis\n");
			if (config_ep_by_speed(cdev->gadget, f,
					       rndis->port.in_ep) ||
			    config_ep_by_speed(cdev->gadget, f,
					       rndis->port.out_ep)) {
				rndis->port.in_ep->desc = NULL;
				rndis->port.out_ep->desc = NULL;
				goto fail;
			}
		}

		
		rndis->port.is_zlp_ok = false;

		rndis->port.cdc_filter = 0;

		DBG(cdev, "RNDIS RX/TX early activation ...\n");
		gether_enable_sg(&rndis->port, true);
		net = gether_connect(&rndis->port);
		if (IS_ERR(net))
			return PTR_ERR(net);

		rndis_set_param_dev(rndis->config, net,
				&rndis->port.cdc_filter);
	} else
		goto fail;

	return 0;
fail:
	return -EINVAL;
}

static void rndis_disable(struct usb_function *f)
{
	struct f_rndis		*rndis = func_to_rndis(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	if (!rndis->notify->driver_data)
		return;

	DBG(cdev, "rndis deactivated\n");

	rndis_uninit(rndis->config);
	gether_disconnect(&rndis->port);

	usb_ep_disable(rndis->notify);
	rndis->notify->driver_data = NULL;
}



static void rndis_open(struct gether *geth)
{
	struct f_rndis		*rndis = func_to_rndis(&geth->func);
	struct usb_composite_dev *cdev = geth->func.config->cdev;

	DBG(cdev, "%s\n", __func__);

	rndis_set_param_medium(rndis->config, RNDIS_MEDIUM_802_3,
				bitrate(cdev->gadget) / 100);
	rndis_signal_connect(rndis->config);
}

static void rndis_close(struct gether *geth)
{
	struct f_rndis		*rndis = func_to_rndis(&geth->func);

	DBG(geth->func.config->cdev, "%s\n", __func__);

	rndis_set_param_medium(rndis->config, RNDIS_MEDIUM_802_3, 0);
	rndis_signal_disconnect(rndis->config);
}


static inline bool can_support_rndis(struct usb_configuration *c)
{
	
	return true;
}


static int
rndis_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_rndis		*rndis = func_to_rndis(f);
	struct usb_string	*us;
	int			status;
	struct usb_ep		*ep;

	struct f_rndis_opts *rndis_opts;

	if (!can_support_rndis(c))
		return -EINVAL;

	rndis_opts = container_of(f->fi, struct f_rndis_opts, func_inst);

	if (cdev->use_os_string) {
		f->os_desc_table = kzalloc(sizeof(*f->os_desc_table),
					   GFP_KERNEL);
		if (!f->os_desc_table)
			return -ENOMEM;
		f->os_desc_n = 1;
		f->os_desc_table[0].os_desc = &rndis_opts->rndis_os_desc;
	}

	if (f->fi && !rndis_opts->bound) {
		gether_set_gadget(rndis_opts->net, cdev->gadget);
		status = gether_register_netdev(rndis_opts->net);
		if (status)
			goto fail;
		rndis_opts->bound = true;
	}

	us = usb_gstrings_attach(cdev, rndis_strings,
				 ARRAY_SIZE(rndis_string_defs));
	if (IS_ERR(us)) {
		status = PTR_ERR(us);
		goto fail;
	}
	rndis_control_intf.iInterface = us[0].id;
	rndis_data_intf.iInterface = us[1].id;
	rndis_iad_descriptor.iFunction = us[2].id;

	
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	rndis->ctrl_id = status;
	rndis_iad_descriptor.bFirstInterface = status;

	rndis_control_intf.bInterfaceNumber = status;
	rndis_union_desc.bMasterInterface0 = status;

	if (cdev->use_os_string)
		f->os_desc_table[0].if_id =
			rndis_iad_descriptor.bFirstInterface;

	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	rndis->data_id = status;

	rndis_data_intf.bInterfaceNumber = status;
	rndis_union_desc.bSlaveInterface0 = status;

	status = -ENODEV;

	
	ep = usb_ep_autoconfig(cdev->gadget, &fs_in_desc);
	if (!ep)
		goto fail;
	rndis->port.in_ep = ep;
	ep->driver_data = cdev;	

	ep = usb_ep_autoconfig(cdev->gadget, &fs_out_desc);
	if (!ep)
		goto fail;
	rndis->port.out_ep = ep;
	ep->driver_data = cdev;	

	ep = usb_ep_autoconfig(cdev->gadget, &fs_notify_desc);
	if (!ep)
		goto fail;
	rndis->notify = ep;
	ep->driver_data = cdev;	

	status = -ENOMEM;

	
	rndis->notify_req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!rndis->notify_req)
		goto fail;
	rndis->notify_req->buf = kmalloc(STATUS_BYTECOUNT +
			cdev->gadget->extra_buf_alloc, GFP_KERNEL);
	if (!rndis->notify_req->buf)
		goto fail;
	rndis->notify_req->length = STATUS_BYTECOUNT;
	rndis->notify_req->context = rndis;
	rndis->notify_req->complete = rndis_response_complete;

	hs_in_desc.bEndpointAddress = fs_in_desc.bEndpointAddress;
	hs_out_desc.bEndpointAddress = fs_out_desc.bEndpointAddress;
	hs_notify_desc.bEndpointAddress = fs_notify_desc.bEndpointAddress;

	ss_in_desc.bEndpointAddress = fs_in_desc.bEndpointAddress;
	ss_out_desc.bEndpointAddress = fs_out_desc.bEndpointAddress;
	ss_notify_desc.bEndpointAddress = fs_notify_desc.bEndpointAddress;

	status = usb_assign_descriptors(f, eth_fs_function, eth_hs_function,
			eth_ss_function);
	if (status)
		goto fail;

	rndis->port.open = rndis_open;
	rndis->port.close = rndis_close;

	rndis_set_param_medium(rndis->config, RNDIS_MEDIUM_802_3, 0);
	rndis_set_host_mac(rndis->config, rndis->ethaddr);
	rndis_set_max_pkt_xfer(rndis->config, rndis_ul_max_pkt_per_xfer);

	if (rndis->manufacturer && rndis->vendorID &&
			rndis_set_param_vendor(rndis->config, rndis->vendorID,
					       rndis->manufacturer)) {
		status = -EINVAL;
		goto fail_free_descs;
	}


	DBG(cdev, "RNDIS: %s speed IN/%s OUT/%s NOTIFY/%s\n",
			gadget_is_superspeed(c->cdev->gadget) ? "super" :
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			rndis->port.in_ep->name, rndis->port.out_ep->name,
			rndis->notify->name);
	return 0;

fail_free_descs:
	usb_free_all_descriptors(f);
fail:
	kfree(f->os_desc_table);
	f->os_desc_n = 0;

	if (rndis->notify_req) {
		kfree(rndis->notify_req->buf);
		usb_ep_free_request(rndis->notify, rndis->notify_req);
	}

	
	if (rndis->notify)
		rndis->notify->driver_data = NULL;
	if (rndis->port.out_ep)
		rndis->port.out_ep->driver_data = NULL;
	if (rndis->port.in_ep)
		rndis->port.in_ep->driver_data = NULL;

	ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);

	return status;
}

static void
rndis_old_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_rndis	*rndis = func_to_rndis(f);

	rndis_deregister(rndis->config);

	usb_free_all_descriptors(f);

	kfree(rndis->notify_req->buf);
	usb_ep_free_request(rndis->notify, rndis->notify_req);

	kfree(rndis);
	__rndis = NULL;
}

int
rndis_bind_config_vendor(struct usb_configuration *c, u8 ethaddr[ETH_ALEN],
		u32 vendorID, const char *manufacturer, struct eth_dev *dev)
{
	struct f_rndis	*rndis;
	int		status;

	
	status = -ENOMEM;
	rndis = kzalloc(sizeof *rndis, GFP_KERNEL);
	if (!rndis)
		goto fail;

	__rndis = rndis;

	memcpy(rndis->ethaddr, ethaddr, ETH_ALEN);
	rndis->vendorID = vendorID;
	rndis->manufacturer = manufacturer;

	rndis->port.ioport = dev;
	
	rndis->port.cdc_filter = 0;

	
	rndis->port.header_len = sizeof(struct rndis_packet_msg_type);
	rndis->port.wrap = rndis_add_header;
	rndis->port.unwrap = rndis_rm_hdr;
	rndis->port.ul_max_pkts_per_xfer = rndis_ul_max_pkt_per_xfer;
	rndis->port.dl_max_pkts_per_xfer = rndis_dl_max_pkt_per_xfer;
	rndis->port.rx_trigger_enabled = rx_trigger_enabled;

	rndis->port.func.name = "rndis";
	
	rndis->port.func.bind = rndis_bind;
	rndis->port.func.unbind = rndis_old_unbind;
	rndis->port.func.set_alt = rndis_set_alt;
	rndis->port.func.setup = rndis_setup;
	rndis->port.func.disable = rndis_disable;

	status = rndis_register(rndis_response_available, rndis, NULL);
	if (status < 0) {
		kfree(rndis);
		return status;
	}
	rndis->config = status;

	status = usb_add_function(c, &rndis->port.func);
	if (status)
		kfree(rndis);
fail:
	return status;
}

void rndis_borrow_net(struct usb_function_instance *f, struct net_device *net)
{
	struct f_rndis_opts *opts;

	opts = container_of(f, struct f_rndis_opts, func_inst);
	if (opts->bound)
		gether_cleanup(netdev_priv(opts->net));
	else
		free_netdev(opts->net);
	opts->borrowed_net = opts->bound = true;
	opts->net = net;
}
EXPORT_SYMBOL_GPL(rndis_borrow_net);

static inline struct f_rndis_opts *to_f_rndis_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_rndis_opts,
			    func_inst.group);
}

USB_ETHERNET_CONFIGFS_ITEM(rndis);

USB_ETHERNET_CONFIGFS_ITEM_ATTR_DEV_ADDR(rndis);

USB_ETHERNET_CONFIGFS_ITEM_ATTR_HOST_ADDR(rndis);

USB_ETHERNET_CONFIGFS_ITEM_ATTR_QMULT(rndis);

USB_ETHERNET_CONFIGFS_ITEM_ATTR_IFNAME(rndis);

static struct configfs_attribute *rndis_attrs[] = {
	&f_rndis_opts_dev_addr.attr,
	&f_rndis_opts_host_addr.attr,
	&f_rndis_opts_qmult.attr,
	&f_rndis_opts_ifname.attr,
	NULL,
};

static struct config_item_type rndis_func_type = {
	.ct_item_ops	= &rndis_item_ops,
	.ct_attrs	= rndis_attrs,
	.ct_owner	= THIS_MODULE,
};

static void rndis_free_inst(struct usb_function_instance *f)
{
	struct f_rndis_opts *opts;

	opts = container_of(f, struct f_rndis_opts, func_inst);
	if (!opts->borrowed_net) {
		if (opts->bound)
			gether_cleanup(netdev_priv(opts->net));
		else
			free_netdev(opts->net);
	}

	kfree(opts->rndis_os_desc.group.default_groups); 
	kfree(opts);
}

static struct usb_function_instance *rndis_alloc_inst(void)
{
	struct f_rndis_opts *opts;
	struct usb_os_desc *descs[1];
	char *names[1];

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	opts->rndis_os_desc.ext_compat_id = opts->rndis_ext_compat_id;

	mutex_init(&opts->lock);
	opts->func_inst.free_func_inst = rndis_free_inst;
	opts->net = gether_setup_default();
	if (IS_ERR(opts->net)) {
		struct net_device *net = opts->net;
		kfree(opts);
		return ERR_CAST(net);
	}
	INIT_LIST_HEAD(&opts->rndis_os_desc.ext_prop);

	descs[0] = &opts->rndis_os_desc;
	names[0] = "rndis";
	usb_os_desc_prepare_interf_dir(&opts->func_inst.group, 1, descs,
				       names, THIS_MODULE);
	config_group_init_type_name(&opts->func_inst.group, "",
				    &rndis_func_type);

	return &opts->func_inst;
}

static void rndis_free(struct usb_function *f)
{
	struct f_rndis *rndis;
	struct f_rndis_opts *opts;

	rndis = func_to_rndis(f);
	rndis_deregister(rndis->config);
	opts = container_of(f->fi, struct f_rndis_opts, func_inst);
	kfree(rndis);
	mutex_lock(&opts->lock);
	opts->refcnt--;
	mutex_unlock(&opts->lock);
}

static void rndis_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_rndis		*rndis = func_to_rndis(f);

	kfree(f->os_desc_table);
	f->os_desc_n = 0;
	usb_free_all_descriptors(f);

	kfree(rndis->notify_req->buf);
	usb_ep_free_request(rndis->notify, rndis->notify_req);
}

static struct usb_function *rndis_alloc(struct usb_function_instance *fi)
{
	struct f_rndis	*rndis;
	struct f_rndis_opts *opts;
	int status;

	
	rndis = kzalloc(sizeof(*rndis), GFP_KERNEL);
	if (!rndis)
		return ERR_PTR(-ENOMEM);

	opts = container_of(fi, struct f_rndis_opts, func_inst);
	mutex_lock(&opts->lock);
	opts->refcnt++;

	gether_get_host_addr_u8(opts->net, rndis->ethaddr);
	rndis->vendorID = opts->vendor_id;
	rndis->manufacturer = opts->manufacturer;

	rndis->port.ioport = netdev_priv(opts->net);
	mutex_unlock(&opts->lock);
	
	rndis->port.cdc_filter = 0;

	
	rndis->port.header_len = sizeof(struct rndis_packet_msg_type);
	rndis->port.wrap = rndis_add_header;
	rndis->port.unwrap = rndis_rm_hdr;
	rndis->port.ul_max_pkts_per_xfer = rndis_ul_max_pkt_per_xfer;
	rndis->port.dl_max_pkts_per_xfer = rndis_dl_max_pkt_per_xfer;

	rndis->port.func.name = "rndis";
	
	rndis->port.func.bind = rndis_bind;
	rndis->port.func.unbind = rndis_unbind;
	rndis->port.func.set_alt = rndis_set_alt;
	rndis->port.func.setup = rndis_setup;
	rndis->port.func.disable = rndis_disable;
	rndis->port.func.free_func = rndis_free;

	status = rndis_register(rndis_response_available, rndis, NULL);
	if (status < 0) {
		kfree(rndis);
		return ERR_PTR(status);
	}
	rndis->config = status;

	return &rndis->port.func;
}

DECLARE_USB_FUNCTION(rndis, rndis_alloc_inst, rndis_alloc);

static int __init rndis_mod_init(void)
{
	int ret;

	ret = rndis_init();
	if (ret)
		return ret;

	return usb_function_register(&rndisusb_func);
}
module_init(rndis_mod_init);

static void __exit rndis_mod_exit(void)
{
	usb_function_unregister(&rndisusb_func);
	rndis_exit();
}
module_exit(rndis_mod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Brownell");
