/*
 * f_ncm.c -- USB CDC Network (NCM) link function driver
 *
 * Copyright (C) 2010 Nokia Corporation
 * Contact: Yauheni Kaliuta <yauheni.kaliuta@nokia.com>
 *
 * The driver borrows from f_ecm.c which is:
 *
 * Copyright (C) 2003-2005,2008 David Brownell
 * Copyright (C) 2008 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/crc32.h>

#include <linux/usb/cdc.h>

#include "u_ether.h"
#include "u_ether_configfs.h"
#include "u_ncm.h"



#define NCM_NDP_HDR_CRC_MASK	0x01000000
#define NCM_NDP_HDR_CRC		0x01000000
#define NCM_NDP_HDR_NOCRC	0x00000000

enum ncm_notify_state_ncm {
	NCM2_NOTIFY_NONE,		
	NCM2_NOTIFY_CONNECT,		
	NCM2_NOTIFY_SPEED,		
};

struct f_ncm {
	struct gether			port;
	u8				ctrl_id, data_id;

	char				ethaddr[14];

	struct usb_ep			*notify;
	struct usb_request		*notify_req;
	u8				notify_state;
	bool				is_open;

	struct ndp_parser_opts_ncm	*parser_opts;
	bool				is_crc;
	u32				ndp_sign;

	int				iCurMaxDataSize;

	spinlock_t			lock;
	struct net_device               *netdev;
};

static inline struct f_ncm *func_to_ncm(struct usb_function *f)
{
	return container_of(f, struct f_ncm, port.func);
}

static inline unsigned ncm_bitrate(struct usb_gadget *g)
{
	if (gadget_is_dualspeed(g) && g->speed == USB_SPEED_HIGH)
		return 13 * 512 * 8 * 1000 * 8;
	else
		return 19 *  64 * 1 * 1000 * 8;
}


#define NTB_DEFAULT_IN_SIZE_NCM	16384
#define NTB_OUT_SIZE_NCM	16384

#define MAX_TX_NONFIXED		(512 * 3)

#define FORMATS_SUPPORTED_NCM	(USB_CDC_NCM_NTB16_SUPPORTED |	\
				 USB_CDC_NCM_NTB32_SUPPORTED)

static struct usb_cdc_ncm_ntb_parameters ntb_parameters_ncm = {
	.wLength = sizeof ntb_parameters_ncm,
	.bmNtbFormatsSupported = cpu_to_le16(FORMATS_SUPPORTED_NCM),
	.dwNtbInMaxSize = cpu_to_le32(NTB_DEFAULT_IN_SIZE_NCM),
	.wNdpInDivisor = cpu_to_le16(4),
	.wNdpInPayloadRemainder = cpu_to_le16(0),
	.wNdpInAlignment = cpu_to_le16(4),

	.dwNtbOutMaxSize = cpu_to_le32(NTB_OUT_SIZE_NCM),
	.wNdpOutDivisor = cpu_to_le16(4),
	.wNdpOutPayloadRemainder = cpu_to_le16(0),
	.wNdpOutAlignment = cpu_to_le16(4),
};


#define LOG2_STATUS_INTERVAL_MSEC	5	
#define NCM_STATUS_BYTECOUNT		16	

static struct usb_interface_assoc_descriptor ncm_iad_desc = {
	.bLength =		sizeof ncm_iad_desc,
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,

	
	.bInterfaceCount =	2,	
	.bFunctionClass =	USB_CLASS_COMM,
	.bFunctionSubClass =	USB_CDC_SUBCLASS_NCM,
	.bFunctionProtocol =	USB_CDC_PROTO_NONE,
	
};


static struct usb_interface_descriptor ncm_control_intf = {
	.bLength =		sizeof ncm_control_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_COMM,
	.bInterfaceSubClass =	USB_CDC_SUBCLASS_NCM,
	.bInterfaceProtocol =	USB_CDC_PROTO_NONE,
	
};

static struct usb_cdc_header_desc ncm_header_desc = {
	.bLength =		sizeof ncm_header_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,

	.bcdCDC =		cpu_to_le16(0x0110),
};

static struct usb_cdc_union_desc ncm_union_desc = {
	.bLength =		sizeof(ncm_union_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	
	
};

static struct usb_cdc_ether_desc ecm_desc = {
	.bLength =		sizeof ecm_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_ETHERNET_TYPE,

	
	
	.bmEthernetStatistics =	cpu_to_le32(0), 
	.wMaxSegmentSize =	cpu_to_le16(ETH_FRAME_LEN),
	.wNumberMCFilters =	cpu_to_le16(0),
	.bNumberPowerFilters =	0,
};

#define NCAPS	(USB_CDC_NCM_NCAP_ETH_FILTER | USB_CDC_NCM_NCAP_CRC_MODE | USB_CDC_NCM_NCAP_MAX_DATAGRAM_SIZE)

static struct usb_cdc_ncm_desc ncm_desc = {
	.bLength =		sizeof ncm_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_NCM_TYPE,

	.bcdNcmVersion =	cpu_to_le16(0x0100),
	
	.bmNetworkCapabilities = NCAPS,
};


static struct usb_interface_descriptor ncm_data_nop_intf = {
	.bLength =		sizeof ncm_data_nop_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	.bInterfaceNumber =	1,
	.bAlternateSetting =	0,
	.bNumEndpoints =	0,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	USB_CDC_NCM_PROTO_NTB,
	
};


static struct usb_interface_descriptor ncm_data_intf = {
	.bLength =		sizeof ncm_data_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	.bInterfaceNumber =	1,
	.bAlternateSetting =	1,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	USB_CDC_NCM_PROTO_NTB,
	
};


static struct usb_endpoint_descriptor fs_ncm_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(NCM_STATUS_BYTECOUNT),
	.bInterval =		1 << LOG2_STATUS_INTERVAL_MSEC,
};

static struct usb_endpoint_descriptor fs_ncm_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_ncm_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *ncm_fs_function[] = {
	(struct usb_descriptor_header *) &ncm_iad_desc,
	
	(struct usb_descriptor_header *) &ncm_control_intf,
	(struct usb_descriptor_header *) &ncm_header_desc,
	(struct usb_descriptor_header *) &ncm_union_desc,
	(struct usb_descriptor_header *) &ecm_desc,
	(struct usb_descriptor_header *) &ncm_desc,
	(struct usb_descriptor_header *) &fs_ncm_notify_desc,
	
	(struct usb_descriptor_header *) &ncm_data_nop_intf,
	(struct usb_descriptor_header *) &ncm_data_intf,
	(struct usb_descriptor_header *) &fs_ncm_in_desc,
	(struct usb_descriptor_header *) &fs_ncm_out_desc,
	NULL,
};


static struct usb_endpoint_descriptor hs_ncm_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(NCM_STATUS_BYTECOUNT),
	.bInterval =		LOG2_STATUS_INTERVAL_MSEC + 4,
};
static struct usb_endpoint_descriptor hs_ncm_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_ncm_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *ncm_hs_function[] = {
	(struct usb_descriptor_header *) &ncm_iad_desc,
	
	(struct usb_descriptor_header *) &ncm_control_intf,
	(struct usb_descriptor_header *) &ncm_header_desc,
	(struct usb_descriptor_header *) &ncm_union_desc,
	(struct usb_descriptor_header *) &ecm_desc,
	(struct usb_descriptor_header *) &ncm_desc,
	(struct usb_descriptor_header *) &hs_ncm_notify_desc,
	
	(struct usb_descriptor_header *) &ncm_data_nop_intf,
	(struct usb_descriptor_header *) &ncm_data_intf,
	(struct usb_descriptor_header *) &hs_ncm_in_desc,
	(struct usb_descriptor_header *) &hs_ncm_out_desc,
	NULL,
};

static struct usb_endpoint_descriptor ncm_ss_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(NCM_STATUS_BYTECOUNT),
	.bInterval =		LOG2_STATUS_INTERVAL_MSEC + 4,
};

static struct usb_ss_ep_comp_descriptor ncm_ss_notify_comp_desc = {
	.bLength =		sizeof(ncm_ss_notify_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,
	
	
	
	.wBytesPerInterval =	cpu_to_le16(NCM_STATUS_BYTECOUNT),
};

static struct usb_endpoint_descriptor ncm_ss_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor ncm_ss_in_comp_desc = {
	.bLength =		sizeof(ncm_ss_in_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,
	
	
	
};

static struct usb_endpoint_descriptor ncm_ss_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor ncm_ss_out_comp_desc = {
	.bLength =		sizeof(ncm_ss_out_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,
	
	
	
};

static struct usb_descriptor_header *ncm_ss_function[] = {
	(struct usb_descriptor_header *) &ncm_iad_desc,
	
	(struct usb_descriptor_header *) &ncm_control_intf,
	(struct usb_descriptor_header *) &ncm_header_desc,
	(struct usb_descriptor_header *) &ncm_union_desc,
	(struct usb_descriptor_header *) &ecm_desc,
	(struct usb_descriptor_header *) &ncm_desc,
	(struct usb_descriptor_header *) &ncm_ss_notify_desc,
	(struct usb_descriptor_header *) &ncm_ss_notify_comp_desc,
	
	(struct usb_descriptor_header *) &ncm_data_nop_intf,
	(struct usb_descriptor_header *) &ncm_data_intf,
	(struct usb_descriptor_header *) &ncm_ss_in_desc,
	(struct usb_descriptor_header *) &ncm_ss_in_comp_desc,
	(struct usb_descriptor_header *) &ncm_ss_out_desc,
	(struct usb_descriptor_header *) &ncm_ss_out_comp_desc,
	NULL,
};


#define STRING_CTRL_IDX	0
#define STRING_MAC_IDX	1
#define STRING_DATA_IDX_NCM	2
#define STRING_IAD_IDX	3

static struct usb_string ncm_string_defs[] = {
	[STRING_CTRL_IDX].s = "CDC Network Control Model (NCM)",
	[STRING_MAC_IDX].s = "",
	[STRING_DATA_IDX_NCM].s = "CDC Network Data",
	[STRING_IAD_IDX].s = "CDC NCM",
	{  } 
};

static struct usb_gadget_strings ncm_string_table = {
	.language =		0x0409,	
	.strings =		ncm_string_defs,
};

static struct usb_gadget_strings *ncm_strings[] = {
	&ncm_string_table,
	NULL,
};


struct ndp_parser_opts_ncm {
	u32		nth_sign;
	u32		ndp_sign;
	unsigned	nth_size;
	unsigned	ndp_size;
	unsigned	dpe_size;
	unsigned	ndplen_align;
	
	unsigned	dgram_item_len; 
	unsigned	block_length;
	unsigned	ndp_index;
	unsigned	reserved1;
	unsigned	reserved2;
	unsigned	next_ndp_index;
};

#define INIT_NDP16_OPTS {					\
		.nth_sign = USB_CDC_NCM_NTH16_SIGN,		\
		.ndp_sign = USB_CDC_NCM_NDP16_NOCRC_SIGN,	\
		.nth_size = sizeof(struct usb_cdc_ncm_nth16),	\
		.ndp_size = sizeof(struct usb_cdc_ncm_ndp16),	\
		.dpe_size = sizeof(struct usb_cdc_ncm_dpe16),	\
		.ndplen_align = 4,				\
		.dgram_item_len = 1,				\
		.block_length = 1,				\
		.ndp_index = 1,					\
		.reserved1 = 0,					\
		.reserved2 = 0,					\
		.next_ndp_index = 1,				\
	}


#define INIT_NDP32_OPTS {					\
		.nth_sign = USB_CDC_NCM_NTH32_SIGN,		\
		.ndp_sign = USB_CDC_NCM_NDP32_NOCRC_SIGN,	\
		.nth_size = sizeof(struct usb_cdc_ncm_nth32),	\
		.ndp_size = sizeof(struct usb_cdc_ncm_ndp32),	\
		.dpe_size = sizeof(struct usb_cdc_ncm_dpe32),	\
		.ndplen_align = 8,				\
		.dgram_item_len = 2,				\
		.block_length = 2,				\
		.ndp_index = 2,					\
		.reserved1 = 1,					\
		.reserved2 = 2,					\
		.next_ndp_index = 2,				\
	}

static struct ndp_parser_opts_ncm ndp16_opts_ncm = INIT_NDP16_OPTS;
static struct ndp_parser_opts_ncm ndp32_opts_ncm = INIT_NDP32_OPTS;

static inline void put_ncm(__le16 **p, unsigned size, unsigned val)
{
	switch (size) {
	case 1:
		put_unaligned_le16((u16)val, *p);
		break;
	case 2:
		put_unaligned_le32((u32)val, *p);

		break;
	default:
		BUG();
	}

	*p += size;
}

static inline unsigned get_ncm(__le16 **p, unsigned size)
{
	unsigned tmp;

	switch (size) {
	case 1:
		tmp = get_unaligned_le16(*p);
		break;
	case 2:
		tmp = get_unaligned_le32(*p);
		break;
	default:
		BUG();
	}

	*p += size;
	return tmp;
}


static inline void ncm_reset_values(struct f_ncm *ncm)
{
	ncm->parser_opts = &ndp16_opts_ncm;
	ncm->ndp_sign = ncm->parser_opts->ndp_sign;
	ncm->is_crc = false;
	ncm->port.cdc_filter = DEFAULT_FILTER;

	
	ncm->port.header_len = 0;

	ncm->port.fixed_out_len = le32_to_cpu(ntb_parameters_ncm.dwNtbOutMaxSize);
	ncm->port.fixed_in_len = NTB_DEFAULT_IN_SIZE_NCM;
}

static void ncm_do_notify(struct f_ncm *ncm)
{
	struct usb_request		*req = ncm->notify_req;
	struct usb_cdc_notification	*event;
	struct usb_composite_dev	*cdev = ncm->port.func.config->cdev;
	__le32				*data;
	int				status;

	
	if (!req)
		return;

	event = req->buf;
	switch (ncm->notify_state) {
	case NCM2_NOTIFY_NONE:
		return;

	case NCM2_NOTIFY_CONNECT:
		event->bNotificationType = USB_CDC_NOTIFY_NETWORK_CONNECTION;
		if (ncm->is_open)
			event->wValue = cpu_to_le16(1);
		else
			event->wValue = cpu_to_le16(0);
		event->wLength = 0;
		req->length = sizeof *event;

		DBG(cdev, "notify connect %s\n",
				ncm->is_open ? "true" : "false");
		ncm->notify_state = NCM2_NOTIFY_NONE;
		break;

	case NCM2_NOTIFY_SPEED:
		event->bNotificationType = USB_CDC_NOTIFY_SPEED_CHANGE;
		event->wValue = cpu_to_le16(0);
		event->wLength = cpu_to_le16(8);
		req->length = NCM_STATUS_BYTECOUNT;

		
		data = req->buf + sizeof *event;
		data[0] = cpu_to_le32(ncm_bitrate(cdev->gadget));
		data[1] = data[0];

		DBG(cdev, "notify speed %d\n", ncm_bitrate(cdev->gadget));
		ncm->notify_state = NCM2_NOTIFY_CONNECT;
		break;
	}
	event->bmRequestType = 0xA1;
	event->wIndex = cpu_to_le16(ncm->ctrl_id);

	ncm->notify_req = NULL;
	spin_unlock(&ncm->lock);
	status = usb_ep_queue(ncm->notify, req, GFP_ATOMIC);
	spin_lock(&ncm->lock);
	if (status < 0) {
		ncm->notify_req = req;
		DBG(cdev, "notify --> %d\n", status);
	}
}

static void ncm_notify(struct f_ncm *ncm)
{
	ncm->notify_state = NCM2_NOTIFY_SPEED;
	ncm_do_notify(ncm);
}

static void ncm_notify_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_ncm			*ncm = req->context;

	spin_lock(&ncm->lock);
	switch (req->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ESHUTDOWN:
		ncm->notify_state = NCM2_NOTIFY_NONE;
		break;
	default:
		break;
	}
	ncm->notify_req = req;
	ncm_do_notify(ncm);
	spin_unlock(&ncm->lock);
}

static void ncm_ep0out_complete(struct usb_ep *ep, struct usb_request *req)
{
	
	unsigned		in_size;
	struct usb_function	*f = req->context;
	struct f_ncm		*ncm = func_to_ncm(f);

	req->context = NULL;
	if (req->status || req->actual != req->length) {
		goto invalid;
	}

	in_size = get_unaligned_le32(req->buf);
	if (in_size < USB_CDC_NCM_NTB_MIN_IN_SIZE ||
	    in_size > le32_to_cpu(ntb_parameters_ncm.dwNtbInMaxSize)) {
		goto invalid;
	}

	ncm->port.fixed_in_len = in_size;
	return;

invalid:
	usb_ep_set_halt(ep);
	return;
}

static void ncm_ep0out_complete2(struct usb_ep *ep, struct usb_request *req)
{
	u16			in_size;
	struct usb_function	*f = req->context;
	struct f_ncm		*ncm = func_to_ncm(f);
	struct usb_composite_dev	*cdev = ep->driver_data;

	req->context = NULL;
	if (req->status || req->actual != req->length) {
		DBG(cdev, "Bad control-OUT transfer\n");
		goto invalid;
	}

	in_size = get_unaligned_le16(req->buf);

	DBG(cdev, "Set USB_CDC_SET_MAX_DATAGRAM_SIZE %d\n", in_size);
	gether_change_mtu(in_size - ETH_HLEN);
	ncm->iCurMaxDataSize = in_size;
	return;

invalid:
	usb_ep_set_halt(ep);
	return;
}

static int ncm_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_ncm		*ncm = func_to_ncm(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_SET_ETHERNET_PACKET_FILTER:
		if (w_length != 0 || w_index != ncm->ctrl_id)
			goto invalid;
		DBG(cdev, "packet filter %02x\n", w_value);
		ncm->port.cdc_filter = w_value;
		value = 0;
		break;

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_GET_NTB_PARAMETERS:

		if (w_length == 0 || w_value != 0 || w_index != ncm->ctrl_id)
			goto invalid;
		value = w_length > sizeof ntb_parameters_ncm ?
			sizeof ntb_parameters_ncm : w_length;
		memcpy(req->buf, &ntb_parameters_ncm, value);
		DBG(cdev, "Host asked NTB parameters\n");
		break;

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_GET_NTB_INPUT_SIZE:

		if (w_length < 4 || w_value != 0 || w_index != ncm->ctrl_id)
			goto invalid;
		put_unaligned_le32(ncm->port.fixed_in_len, req->buf);
		value = 4;
		DBG(cdev, "Host asked INPUT SIZE, sending %d\n",
		     ncm->port.fixed_in_len);
		break;

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_SET_NTB_INPUT_SIZE:
	{
		if (w_length != 4 || w_value != 0 || w_index != ncm->ctrl_id)
			goto invalid;
		req->complete = ncm_ep0out_complete;
		req->length = w_length;
		req->context = f;

		value = req->length;
		break;
	}

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_GET_NTB_FORMAT:
	{
		uint16_t format;

		if (w_length < 2 || w_value != 0 || w_index != ncm->ctrl_id)
			goto invalid;
		format = (ncm->parser_opts == &ndp16_opts_ncm) ? 0x0000 : 0x0001;
		put_unaligned_le16(format, req->buf);
		value = 2;
		DBG(cdev, "Host asked NTB FORMAT, sending %d\n", format);
		break;
	}

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_SET_NTB_FORMAT:
	{
		if (w_length != 0 || w_index != ncm->ctrl_id)
			goto invalid;
		switch (w_value) {
		case 0x0000:
			ncm->parser_opts = &ndp16_opts_ncm;
			DBG(cdev, "NCM16 selected\n");
			break;
		case 0x0001:
			ncm->parser_opts = &ndp32_opts_ncm;
			DBG(cdev, "NCM32 selected\n");
			break;
		default:
			goto invalid;
		}
		value = 0;
		break;
	}
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_GET_CRC_MODE:
	{
		uint16_t is_crc;

		if (w_length < 2 || w_value != 0 || w_index != ncm->ctrl_id)
			goto invalid;
		is_crc = ncm->is_crc ? 0x0001 : 0x0000;
		put_unaligned_le16(is_crc, req->buf);
		value = 2;
		DBG(cdev, "Host asked CRC MODE, sending %d\n", is_crc);
		break;
	}

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_SET_CRC_MODE:
	{
		int ndp_hdr_crc = 0;

		if (w_length != 0 || w_index != ncm->ctrl_id)
			goto invalid;
		switch (w_value) {
		case 0x0000:
			ncm->is_crc = false;
			ndp_hdr_crc = NCM_NDP_HDR_NOCRC;
			DBG(cdev, "non-CRC mode selected\n");
			break;
		case 0x0001:
			ncm->is_crc = true;
			ndp_hdr_crc = NCM_NDP_HDR_CRC;
			DBG(cdev, "CRC mode selected\n");
			break;
		default:
			goto invalid;
		}
		ncm->parser_opts->ndp_sign &= ~NCM_NDP_HDR_CRC_MASK;
		ncm->parser_opts->ndp_sign |= ndp_hdr_crc;
		value = 0;
		break;
	}

	
	
	
	
	

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_GET_MAX_DATAGRAM_SIZE:
	{
		if (w_length < 2 || w_value != 0 || w_index != ncm->ctrl_id)
			goto invalid;
		if ((ncm->iCurMaxDataSize == 0) || (ncm->iCurMaxDataSize > ETH_FRAME_LEN_MAX))
			ncm->iCurMaxDataSize = ETH_FRAME_LEN_MAX;
		put_unaligned_le16(ncm->iCurMaxDataSize, req->buf);
		value = 2;
		DBG(cdev, "Host asked MAX_DATAGRAME_SIZE, sending %d\n", ncm->iCurMaxDataSize);
		break;
	}

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_SET_MAX_DATAGRAM_SIZE:
	{
		if (w_length != 2 || w_value != 0 || w_index != ncm->ctrl_id)
			goto invalid;
		DBG(cdev, "Host set MAX_DATAGRAM_SIZE\n");
		req->complete = ncm_ep0out_complete2;
		req->length = w_length;
		req->context = f;

		value = req->length;
		break;
	}

	default:
invalid:
		DBG(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	
	if (value >= 0) {
		DBG(cdev, "ncm req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = 0;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			DBG(cdev, "ncm req %02x.%02x response err %d\n",
					ctrl->bRequestType, ctrl->bRequest,
					value);
	}

	
	return value;
}


static int ncm_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_ncm		*ncm = func_to_ncm(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	
	if (intf == ncm->ctrl_id) {
		if (alt != 0)
			goto fail;

		if (ncm->notify->driver_data) {
			DBG(cdev, "reset ncm control %d\n", intf);
			usb_ep_disable(ncm->notify);
		}

		if (!(ncm->notify->desc)) {
			DBG(cdev, "init ncm ctrl %d\n", intf);
			if (config_ep_by_speed(cdev->gadget, f, ncm->notify))
				goto fail;
		}
		usb_ep_enable(ncm->notify);
		ncm->notify->driver_data = ncm;

	
	} else if (intf == ncm->data_id) {
		if (alt > 1)
			goto fail;

		if (ncm->port.in_ep->driver_data) {
			DBG(cdev, "reset ncm\n");
			ncm->netdev = NULL;
			gether_disconnect(&ncm->port);
			ncm_reset_values(ncm);
		}

		if (alt == 1) {
			struct net_device	*net;

			if (!ncm->port.in_ep->desc ||
			    !ncm->port.out_ep->desc) {
				DBG(cdev, "init ncm\n");
				if (config_ep_by_speed(cdev->gadget, f,
						       ncm->port.in_ep) ||
				    config_ep_by_speed(cdev->gadget, f,
						       ncm->port.out_ep)) {
					ncm->port.in_ep->desc = NULL;
					ncm->port.out_ep->desc = NULL;
					goto fail;
				}
			}

			
			ncm->port.is_zlp_ok = !(
				gadget_is_musbhdrc(cdev->gadget)
				);
			ncm->port.cdc_filter = DEFAULT_FILTER;
			DBG(cdev, "activate ncm\n");
			net = gether_connect(&ncm->port);
			if (IS_ERR(net))
				return PTR_ERR(net);
			ncm->netdev = net;
		}

		spin_lock(&ncm->lock);
		ncm_notify(ncm);
		spin_unlock(&ncm->lock);
	} else
		goto fail;

	return 0;
fail:
	return -EINVAL;
}

static int ncm_get_alt(struct usb_function *f, unsigned intf)
{
	struct f_ncm		*ncm = func_to_ncm(f);

	if (intf == ncm->ctrl_id)
		return 0;
	return ncm->port.in_ep->driver_data ? 1 : 0;
}

static struct sk_buff *ncm_wrap_ntb(struct gether *port,
				    struct sk_buff *skb)
{
	struct f_ncm	*ncm = func_to_ncm(&port->func);
	struct sk_buff	*skb2;
	int		ncb_len = 0;
	__le16		*tmp;
	int		div;
	int		rem;
	int		pad;
	int		ndp_align;
	int		ndp_pad;

	unsigned	max_size = ncm->port.fixed_in_len;
	const struct ndp_parser_opts_ncm *opts = ncm->parser_opts;
	unsigned	crc_len = ncm->is_crc ? sizeof(uint32_t) : 0;

	div = le16_to_cpu(ntb_parameters_ncm.wNdpInDivisor);
	rem = le16_to_cpu(ntb_parameters_ncm.wNdpInPayloadRemainder);
	ndp_align = le16_to_cpu(ntb_parameters_ncm.wNdpInAlignment);

	ncb_len += opts->nth_size;
	ndp_pad = ALIGN(ncb_len, ndp_align) - ncb_len;
	ncb_len += ndp_pad;
	ncb_len += opts->ndp_size;
	ncb_len += 2 * 2 * opts->dgram_item_len; 
	ncb_len += 2 * 2 * opts->dgram_item_len; 
	pad = ALIGN(ncb_len, div) + rem - ncb_len;
	ncb_len += pad;

	if (ncb_len + skb->len + crc_len > max_size) {
		dev_kfree_skb_any(skb);
		return NULL;
	}

	skb2 = skb_copy_expand(skb, ncb_len,
				max_size - skb->len - ncb_len - crc_len,
				GFP_ATOMIC);
	dev_kfree_skb_any(skb);
	if (!skb2)
		return NULL;

	skb = skb2;
	tmp = (void *) skb_push(skb, ncb_len);
	memset(tmp, 0, ncb_len);
	put_unaligned_le32(opts->nth_sign, tmp); 
	tmp += 2;
	
	put_unaligned_le16(opts->nth_size, tmp++);
	tmp++; 
	put_ncm(&tmp, opts->block_length, skb->len); 
	
	
	put_ncm(&tmp, opts->ndp_index, opts->nth_size + ndp_pad);
	tmp = (void *)tmp + ndp_pad;
	
	put_unaligned_le32(opts->ndp_sign, tmp); 
	tmp += 2;
	
	put_unaligned_le16(ncb_len - opts->nth_size - pad, tmp++);

	tmp += opts->reserved1;
	tmp += opts->next_ndp_index; 
	tmp += opts->reserved2;

	if (ncm->is_crc) {
		uint32_t crc;

		crc = ~crc32_le(~0,
				skb->data + ncb_len,
				skb->len - ncb_len);
		put_unaligned_le32(crc, skb->data + skb->len);
		skb_put(skb, crc_len);
	}

	
	put_ncm(&tmp, opts->dgram_item_len, ncb_len);
	
	put_ncm(&tmp, opts->dgram_item_len, skb->len - ncb_len);
	

	if (skb->len > MAX_TX_NONFIXED && (max_size <= (ETH_FRAME_LEN - ETH_HLEN)))
	{
		memset(skb_put(skb, max_size - skb->len),
				0, max_size - skb->len);
	}

	return skb;
}

static int ncm_unwrap_ntb(struct gether *port,
			  struct sk_buff *skb,
			  struct sk_buff_head *list)
{
	struct f_ncm	*ncm = func_to_ncm(&port->func);
	__le16		*tmp = (void *) skb->data;
	unsigned	index, index2;
	int		ndp_index;
	unsigned	dg_len, dg_len2;
	unsigned	ndp_len;
	struct sk_buff	*skb2;
	int		ret = -EINVAL;
	unsigned	max_size = le32_to_cpu(ntb_parameters_ncm.dwNtbOutMaxSize);
	const struct ndp_parser_opts_ncm *opts = ncm->parser_opts;
	unsigned	crc_len = ncm->is_crc ? sizeof(uint32_t) : 0;
	int		dgram_counter;

	
	if (get_unaligned_le32(tmp) != opts->nth_sign) {
		print_hex_dump(KERN_INFO, "HEAD:", DUMP_PREFIX_ADDRESS, 32, 1,
			       skb->data, 32, false);

		goto err;
	}
	tmp += 2;
	
	if (get_unaligned_le16(tmp++) != opts->nth_size) {
		goto err;
	}
	tmp++; 

	
	if (get_ncm(&tmp, opts->block_length) > max_size) {
		goto err;
	}

	ndp_index = get_ncm(&tmp, opts->ndp_index);

	
	do {
		
		if (((ndp_index % 4) != 0) &&
				(ndp_index < opts->nth_size)) {
			goto err;
		}

		
		tmp = (void *)(skb->data + ndp_index);
		if (get_unaligned_le32(tmp) != opts->ndp_sign) {
			INFO(port->func.config->cdev, "Wrong NDP SIGN\n");
			goto err;
		}
		tmp += 2;

		ndp_len = get_unaligned_le16(tmp++);
		if ((ndp_len < opts->ndp_size
				+ 2 * 2 * (opts->dgram_item_len * 2))
				|| (ndp_len % opts->ndplen_align != 0)) {
			goto err;
		}
		tmp += opts->reserved1;
		
		ndp_index = get_ncm(&tmp, opts->next_ndp_index);
		tmp += opts->reserved2;

		ndp_len -= opts->ndp_size;
		index2 = get_ncm(&tmp, opts->dgram_item_len);
		dg_len2 = get_ncm(&tmp, opts->dgram_item_len);
		dgram_counter = 0;

		do {
			index = index2;
			dg_len = dg_len2;
			if (dg_len < 14 + crc_len) { 
				goto err;
			}
			if (ncm->is_crc) {
				uint32_t crc, crc2;

				crc = get_unaligned_le32(skb->data +
							 index + dg_len -
							 crc_len);
				crc2 = ~crc32_le(~0,
						 skb->data + index,
						 dg_len - crc_len);
				if (crc != crc2) {
					goto err;
				}
			}

			index2 = get_ncm(&tmp, opts->dgram_item_len);
			dg_len2 = get_ncm(&tmp, opts->dgram_item_len);

			skb2 = netdev_alloc_skb_ip_align(ncm->netdev,
							dg_len - crc_len);
			if (skb2 == NULL)
				goto err;
			memcpy(skb_put(skb2, dg_len - crc_len),
			       skb->data + index, dg_len - crc_len);

			skb_queue_tail(list, skb2);

			ndp_len -= 2 * (opts->dgram_item_len * 2);

			dgram_counter++;

			if (index2 == 0 || dg_len2 == 0)
				break;
		} while (ndp_len > 2 * (opts->dgram_item_len * 2));
	} while (ndp_index);

	dev_kfree_skb_any(skb);

	VDBG(port->func.config->cdev,
	     "Parsed NTB with %d frames\n", dgram_counter);
	return 0;
err:
	skb_queue_purge(list);
	dev_kfree_skb_any(skb);
	return ret;
}

static void ncm_disable(struct usb_function *f)
{
	struct f_ncm		*ncm = func_to_ncm(f);

	if (ncm->port.in_ep->driver_data)
		gether_disconnect(&ncm->port);

	if (ncm->notify->driver_data) {
		usb_ep_disable(ncm->notify);
		ncm->notify->driver_data = NULL;
		ncm->notify->desc = NULL;
	}
}



static void ncm_open(struct gether *geth)
{
	struct f_ncm		*ncm = func_to_ncm(&geth->func);

	DBG(ncm->port.func.config->cdev, "%s\n", __func__);

	spin_lock(&ncm->lock);
	ncm->is_open = true;
	ncm_notify(ncm);
	spin_unlock(&ncm->lock);
}

static void ncm_close(struct gether *geth)
{
	struct f_ncm		*ncm = func_to_ncm(&geth->func);

	DBG(ncm->port.func.config->cdev, "%s\n", __func__);

	spin_lock(&ncm->lock);
	ncm->is_open = false;
	ncm_notify(ncm);
	spin_unlock(&ncm->lock);
}



static int ncm_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_ncm		*ncm = func_to_ncm(f);
	struct usb_string	*us;
	int			status;
	struct usb_ep		*ep;
	struct f_ncm_opts	*ncm_opts;

	if (!can_support_ecm(cdev->gadget))
		return -EINVAL;

	ncm_opts = container_of(f->fi, struct f_ncm_opts, func_inst);
	if (!ncm_opts->bound) {
		mutex_lock(&ncm_opts->lock);
		gether_set_gadget(ncm_opts->net, cdev->gadget);
		status = gether_register_netdev(ncm_opts->net);
		mutex_unlock(&ncm_opts->lock);
		if (status)
			return status;
		ncm_opts->bound = true;
	}
	us = usb_gstrings_attach(cdev, ncm_strings,
				 ARRAY_SIZE(ncm_string_defs));
	if (IS_ERR(us))
		return PTR_ERR(us);
	ncm_control_intf.iInterface = us[STRING_CTRL_IDX].id;
	ncm_data_nop_intf.iInterface = us[STRING_DATA_IDX_NCM].id;
	ncm_data_intf.iInterface = us[STRING_DATA_IDX_NCM].id;
	ecm_desc.iMACAddress = us[STRING_MAC_IDX].id;
	ecm_desc.wMaxSegmentSize = cpu_to_le16(ETH_FRAME_LEN_MAX);
	ncm_iad_desc.iFunction = us[STRING_IAD_IDX].id;

	
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	ncm->ctrl_id = status;
	ncm_iad_desc.bFirstInterface = status;

	ncm_control_intf.bInterfaceNumber = status;
	ncm_union_desc.bMasterInterface0 = status;

	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	ncm->data_id = status;

	ncm_data_nop_intf.bInterfaceNumber = status;
	ncm_data_intf.bInterfaceNumber = status;
	ncm_union_desc.bSlaveInterface0 = status;

	status = -ENODEV;

	
	ep = usb_ep_autoconfig(cdev->gadget, &fs_ncm_in_desc);
	if (!ep)
		goto fail;
	ncm->port.in_ep = ep;
	ncm->port.in_ep->is_ncm = true;
	ep->driver_data = cdev;	

	ep = usb_ep_autoconfig(cdev->gadget, &fs_ncm_out_desc);
	if (!ep)
		goto fail;
	ncm->port.out_ep = ep;
	ncm->port.out_ep->is_ncm = true;
	ep->driver_data = cdev;	

	ep = usb_ep_autoconfig(cdev->gadget, &fs_ncm_notify_desc);
	if (!ep)
		goto fail;
	ncm->notify = ep;
	ep->driver_data = cdev;	

	status = -ENOMEM;

	
	ncm->notify_req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!ncm->notify_req)
		goto fail;
	ncm->notify_req->buf = kmalloc(NCM_STATUS_BYTECOUNT
			+ (cdev->gadget->extra_buf_alloc), GFP_KERNEL);
	if (!ncm->notify_req->buf)
		goto fail;
	ncm->notify_req->context = ncm;
	ncm->notify_req->complete = ncm_notify_complete;

	hs_ncm_in_desc.bEndpointAddress = fs_ncm_in_desc.bEndpointAddress;
	hs_ncm_out_desc.bEndpointAddress = fs_ncm_out_desc.bEndpointAddress;
	hs_ncm_notify_desc.bEndpointAddress =
		fs_ncm_notify_desc.bEndpointAddress;

	if (gadget_is_superspeed(c->cdev->gadget)) {
		ncm_ss_in_desc.bEndpointAddress =
					fs_ncm_in_desc.bEndpointAddress;
		ncm_ss_out_desc.bEndpointAddress =
					fs_ncm_out_desc.bEndpointAddress;
		ncm_ss_notify_desc.bEndpointAddress =
					fs_ncm_notify_desc.bEndpointAddress;
	}

	status = usb_assign_descriptors(f, ncm_fs_function, ncm_hs_function,
					ncm_ss_function);

	ncm->port.open = ncm_open;
	ncm->port.close = ncm_close;

	DBG(cdev, "CDC Network: %s speed IN/%s OUT/%s NOTIFY/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			ncm->port.in_ep->name, ncm->port.out_ep->name,
			ncm->notify->name);
	return 0;

fail:
	if (ncm->notify_req) {
		kfree(ncm->notify_req->buf);
		usb_ep_free_request(ncm->notify, ncm->notify_req);
	}

	
	if (ncm->notify)
		ncm->notify->driver_data = NULL;
	if (ncm->port.out_ep)
		ncm->port.out_ep->driver_data = NULL;
	if (ncm->port.in_ep)
		ncm->port.in_ep->driver_data = NULL;

	ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);

	return status;
}

static inline struct f_ncm_opts *to_f_ncm_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_ncm_opts,
			    func_inst.group);
}

USB_ETHERNET_CONFIGFS_ITEM(ncm);

USB_ETHERNET_CONFIGFS_ITEM_ATTR_DEV_ADDR(ncm);

USB_ETHERNET_CONFIGFS_ITEM_ATTR_HOST_ADDR(ncm);

USB_ETHERNET_CONFIGFS_ITEM_ATTR_QMULT(ncm);

USB_ETHERNET_CONFIGFS_ITEM_ATTR_IFNAME(ncm);

static struct configfs_attribute *ncm_attrs[] = {
	&f_ncm_opts_dev_addr.attr,
	&f_ncm_opts_host_addr.attr,
	&f_ncm_opts_qmult.attr,
	&f_ncm_opts_ifname.attr,
	NULL,
};

static struct config_item_type ncm_func_type = {
	.ct_item_ops	= &ncm_item_ops,
	.ct_attrs	= ncm_attrs,
	.ct_owner	= THIS_MODULE,
};

static void ncm_free_inst(struct usb_function_instance *f)
{
	struct f_ncm_opts *opts;

	opts = container_of(f, struct f_ncm_opts, func_inst);
	if (opts->bound)
		gether_cleanup(netdev_priv(opts->net));
	else
		free_netdev(opts->net);
	kfree(opts);
}

static struct usb_function_instance *ncm_alloc_inst(void)
{
	struct f_ncm_opts *opts;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	mutex_init(&opts->lock);
	opts->func_inst.free_func_inst = ncm_free_inst;
	opts->net = gether_setup_default();
	if (IS_ERR(opts->net)) {
		struct net_device *net = opts->net;
		kfree(opts);
		return ERR_CAST(net);
	}

	config_group_init_type_name(&opts->func_inst.group, "", &ncm_func_type);

	return &opts->func_inst;
}

static void ncm_free(struct usb_function *f)
{
	struct f_ncm *ncm;
	struct f_ncm_opts *opts;

	ncm = func_to_ncm(f);
	opts = container_of(f->fi, struct f_ncm_opts, func_inst);
	kfree(ncm);
	mutex_lock(&opts->lock);
	opts->refcnt--;
	mutex_unlock(&opts->lock);
}

static void ncm_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_ncm *ncm = func_to_ncm(f);

	DBG(c->cdev, "ncm unbind\n");

	
	
	usb_free_all_descriptors(f);

	kfree(ncm->notify_req->buf);
	usb_ep_free_request(ncm->notify, ncm->notify_req);

	ncm->port.in_ep->is_ncm = false;
	ncm->port.out_ep->is_ncm = false;
}

static struct usb_function *ncm_alloc(struct usb_function_instance *fi)
{
	struct f_ncm		*ncm;
	struct f_ncm_opts	*opts;
	int status;

	
	ncm = kzalloc(sizeof(*ncm), GFP_KERNEL);
	if (!ncm)
		return ERR_PTR(-ENOMEM);

	opts = container_of(fi, struct f_ncm_opts, func_inst);
	mutex_lock(&opts->lock);
	opts->refcnt++;

	
	status = gether_get_host_addr_cdc(opts->net, ncm->ethaddr,
				      sizeof(ncm->ethaddr));
	if (status < 12) { 
		kfree(ncm);
		mutex_unlock(&opts->lock);
		return ERR_PTR(-EINVAL);
	}
	ncm_string_defs[STRING_MAC_IDX].s = ncm->ethaddr;

	spin_lock_init(&ncm->lock);
	ncm_reset_values(ncm);
	ncm->port.ioport = netdev_priv(opts->net);
	mutex_unlock(&opts->lock);
	ncm->port.is_fixed = true;

	ncm->port.func.name = "cdc_network";
	
	ncm->port.func.bind = ncm_bind;
	ncm->port.func.unbind = ncm_unbind;
	ncm->port.func.set_alt = ncm_set_alt;
	ncm->port.func.get_alt = ncm_get_alt;
	ncm->port.func.setup = ncm_setup;
	ncm->port.func.disable = ncm_disable;
	ncm->port.func.free_func = ncm_free;

	ncm->port.wrap = ncm_wrap_ntb;
	ncm->port.unwrap = ncm_unwrap_ntb;

	return &ncm->port.func;
}

DECLARE_USB_FUNCTION_INIT(ncm, ncm_alloc_inst, ncm_alloc);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yauheni Kaliuta");
