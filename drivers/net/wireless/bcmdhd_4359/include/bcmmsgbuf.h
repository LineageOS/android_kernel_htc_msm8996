/*
 * MSGBUF network driver ioctl/indication encoding
 * Broadcom 802.11abg Networking Device Driver
 *
 * Definitions subject to change without notice.
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
 * $Id: bcmmsgbuf.h 541060 2015-03-13 23:28:01Z $
 */
#ifndef _bcmmsgbuf_h_
#define	_bcmmsgbuf_h_

#include <proto/ethernet.h>
#include <wlioctl.h>
#include <bcmpcie.h>

#define MSGBUF_MAX_MSG_SIZE   ETHER_MAX_LEN

#define D2H_EPOCH_MODULO		253 
#define D2H_EPOCH_INIT_VAL		(D2H_EPOCH_MODULO + 1)

#define H2D_EPOCH_MODULO		253 
#define H2D_EPOCH_INIT_VAL		(H2D_EPOCH_MODULO + 1)

#define H2DRING_TXPOST_ITEMSIZE		48
#define H2DRING_RXPOST_ITEMSIZE		32
#define H2DRING_CTRL_SUB_ITEMSIZE	40
#define D2HRING_TXCMPLT_ITEMSIZE	16
#define D2HRING_RXCMPLT_ITEMSIZE	32
#define D2HRING_CTRL_CMPLT_ITEMSIZE	24

#define H2DRING_TXPOST_MAX_ITEM			512
#define H2DRING_RXPOST_MAX_ITEM			512
#define H2DRING_CTRL_SUB_MAX_ITEM		64
#define D2HRING_TXCMPLT_MAX_ITEM		1024
#define D2HRING_RXCMPLT_MAX_ITEM		512

#define D2HRING_CTRL_CMPLT_MAX_ITEM		64

enum {
	DNGL_TO_HOST_MSGBUF,
	HOST_TO_DNGL_MSGBUF
};

enum {
	HOST_TO_DNGL_TXP_DATA,
	HOST_TO_DNGL_RXP_DATA,
	HOST_TO_DNGL_CTRL,
	DNGL_TO_HOST_DATA,
	DNGL_TO_HOST_CTRL
};

#define MESSAGE_PAYLOAD(a) (a & MSG_TYPE_INTERNAL_USE_START) ? TRUE : FALSE

#ifdef PCIE_API_REV1

#define BCMMSGBUF_DUMMY_REF(a, b)	do {BCM_REFERENCE((a));BCM_REFERENCE((b));}  while (0)

#define BCMMSGBUF_API_IFIDX(a)		0
#define BCMMSGBUF_API_SEQNUM(a)		0
#define BCMMSGBUF_IOCTL_XTID(a)		0
#define BCMMSGBUF_IOCTL_PKTID(a)	((a)->cmd_id)

#define BCMMSGBUF_SET_API_IFIDX(a, b)	BCMMSGBUF_DUMMY_REF(a, b)
#define BCMMSGBUF_SET_API_SEQNUM(a, b)	BCMMSGBUF_DUMMY_REF(a, b)
#define BCMMSGBUF_IOCTL_SET_PKTID(a, b)	(BCMMSGBUF_IOCTL_PKTID(a) = (b))
#define BCMMSGBUF_IOCTL_SET_XTID(a, b)	BCMMSGBUF_DUMMY_REF(a, b)

#else 

#define BCMMSGBUF_API_IFIDX(a)		((a)->if_id)
#define BCMMSGBUF_IOCTL_PKTID(a)	((a)->pkt_id)
#define BCMMSGBUF_API_SEQNUM(a)		((a)->u.seq.seq_no)
#define BCMMSGBUF_IOCTL_XTID(a)		((a)->xt_id)

#define BCMMSGBUF_SET_API_IFIDX(a, b)	(BCMMSGBUF_API_IFIDX((a)) = (b))
#define BCMMSGBUF_SET_API_SEQNUM(a, b)	(BCMMSGBUF_API_SEQNUM((a)) = (b))
#define BCMMSGBUF_IOCTL_SET_PKTID(a, b)	(BCMMSGBUF_IOCTL_PKTID((a)) = (b))
#define BCMMSGBUF_IOCTL_SET_XTID(a, b)	(BCMMSGBUF_IOCTL_XTID((a)) = (b))

#endif 


union addr64 {
	struct {
		uint32 low;
		uint32 high;
	};
	struct {
		uint32 low_addr;
		uint32 high_addr;
	};
	uint64 u64;
} DECLSPEC_ALIGN(8);

typedef union addr64 bcm_addr64_t;

typedef struct cmn_msg_hdr {
	
	uint8 msg_type;
	
	uint8 if_id;
	
	uint8 flags;
	
	uint8 epoch;
	
	uint32 request_id;
} cmn_msg_hdr_t;

typedef enum bcmpcie_msgtype {
	MSG_TYPE_GEN_STATUS 		= 0x1,
	MSG_TYPE_RING_STATUS		= 0x2,
	MSG_TYPE_FLOW_RING_CREATE	= 0x3,
	MSG_TYPE_FLOW_RING_CREATE_CMPLT	= 0x4,
	MSG_TYPE_FLOW_RING_DELETE	= 0x5,
	MSG_TYPE_FLOW_RING_DELETE_CMPLT	= 0x6,
	MSG_TYPE_FLOW_RING_FLUSH	= 0x7,
	MSG_TYPE_FLOW_RING_FLUSH_CMPLT	= 0x8,
	MSG_TYPE_IOCTLPTR_REQ		= 0x9,
	MSG_TYPE_IOCTLPTR_REQ_ACK	= 0xA,
	MSG_TYPE_IOCTLRESP_BUF_POST	= 0xB,
	MSG_TYPE_IOCTL_CMPLT		= 0xC,
	MSG_TYPE_EVENT_BUF_POST		= 0xD,
	MSG_TYPE_WL_EVENT		= 0xE,
	MSG_TYPE_TX_POST		= 0xF,
	MSG_TYPE_TX_STATUS		= 0x10,
	MSG_TYPE_RXBUF_POST		= 0x11,
	MSG_TYPE_RX_CMPLT		= 0x12,
	MSG_TYPE_LPBK_DMAXFER 		= 0x13,
	MSG_TYPE_LPBK_DMAXFER_CMPLT	= 0x14,
	MSG_TYPE_FLOW_RING_RESUME	 = 0x15,
	MSG_TYPE_FLOW_RING_RESUME_CMPLT	= 0x16,
	MSG_TYPE_FLOW_RING_SUSPEND	= 0x17,
	MSG_TYPE_FLOW_RING_SUSPEND_CMPLT	= 0x18,
	MSG_TYPE_INFO_BUF_POST		= 0x19,
	MSG_TYPE_INFO_BUF_CMPLT		= 0x1A,
	MSG_TYPE_H2D_RING_CREATE	= 0x1B,
	MSG_TYPE_D2H_RING_CREATE	= 0x1C,
	MSG_TYPE_H2D_RING_CREATE_CMPLT	= 0x1D,
	MSG_TYPE_D2H_RING_CREATE_CMPLT	= 0x1E,
	MSG_TYPE_H2D_RING_CONFIG	= 0x1F,
	MSG_TYPE_D2H_RING_CONFIG	= 0x20,
	MSG_TYPE_H2D_RING_CONFIG_CMPLT	= 0x21,
	MSG_TYPE_D2H_RING_CONFIG_CMPLT	= 0x22,
	MSG_TYPE_H2D_MAILBOX_DATA	= 0x23,
	MSG_TYPE_D2H_MAILBOX_DATA	= 0x24,

	MSG_TYPE_API_MAX_RSVD		= 0x3F
} bcmpcie_msg_type_t;

typedef enum bcmpcie_msgtype_int {
	MSG_TYPE_INTERNAL_USE_START	= 0x40,
	MSG_TYPE_EVENT_PYLD		= 0x41,
	MSG_TYPE_IOCT_PYLD		= 0x42,
	MSG_TYPE_RX_PYLD		= 0x43,
	MSG_TYPE_HOST_FETCH		= 0x44,
	MSG_TYPE_LPBK_DMAXFER_PYLD	= 0x45,
	MSG_TYPE_TXMETADATA_PYLD	= 0x46,
	MSG_TYPE_INDX_UPDATE		= 0x47
} bcmpcie_msgtype_int_t;

typedef enum bcmpcie_msgtype_u {
	MSG_TYPE_TX_BATCH_POST		= 0x80,
	MSG_TYPE_IOCTL_REQ		= 0x81,
	MSG_TYPE_HOST_EVNT		= 0x82, 
	MSG_TYPE_LOOPBACK		= 0x83
} bcmpcie_msgtype_u_t;

typedef struct bcmpcie_soft_doorbell {
	uint32	value;  /* host defined value to be written, eg HW threadid */
	bcm_addr64_t haddr; 
	uint16	items;  
	uint16	msecs;  
} bcmpcie_soft_doorbell_t;


#define BCMPCIE_CMNHDR_IFIDX_PHYINTF_SHFT	5
#define BCMPCIE_CMNHDR_IFIDX_PHYINTF_MAX	0x7
#define BCMPCIE_CMNHDR_IFIDX_PHYINTF_MASK	\
	(BCMPCIE_CMNHDR_IFIDX_PHYINTF_MAX << BCMPCIE_CMNHDR_IFIDX_PHYINTF_SHFT)
#define BCMPCIE_CMNHDR_IFIDX_VIRTINTF_SHFT	0
#define BCMPCIE_CMNHDR_IFIDX_VIRTINTF_MAX	0x1F
#define BCMPCIE_CMNHDR_IFIDX_VIRTINTF_MASK	\
	(BCMPCIE_CMNHDR_IFIDX_PHYINTF_MAX << BCMPCIE_CMNHDR_IFIDX_PHYINTF_SHFT)

#define BCMPCIE_CMNHDR_FLAGS_DMA_R_IDX		0x1
#define BCMPCIE_CMNHDR_FLAGS_DMA_R_IDX_INTR	0x2
#define BCMPCIE_CMNHDR_FLAGS_PHASE_BIT		0x80


typedef struct ioctl_req_msg {
	
	cmn_msg_hdr_t 	cmn_hdr;
	
	uint32		cmd;
	
	uint16		trans_id;
	
	uint16		input_buf_len;
	
	uint16		output_buf_len;
	
	uint16		rsvd[3];
	
	bcm_addr64_t	host_input_buf_addr;
	
	uint32		rsvd1[2];
} ioctl_req_msg_t;

typedef struct ioctl_resp_evt_buf_post_msg {
	
	cmn_msg_hdr_t	cmn_hdr;
	
	uint16		host_buf_len;
	
	uint16		reserved[3];
	
	bcm_addr64_t	host_buf_addr;
	uint32		rsvd[4];
} ioctl_resp_evt_buf_post_msg_t;


typedef struct pcie_dma_xfer_params {
	
	cmn_msg_hdr_t	cmn_hdr;

	
	bcm_addr64_t	host_input_buf_addr;

	
	bcm_addr64_t	host_ouput_buf_addr;

	
	uint32		xfer_len;
	
	uint32		srcdelay;
	
	uint32		destdelay;
	uint32		rsvd;
} pcie_dma_xfer_params_t;

typedef struct tx_flowring_create_request {
	cmn_msg_hdr_t   msg;
	uint8	da[ETHER_ADDR_LEN];
	uint8	sa[ETHER_ADDR_LEN];
	uint8	tid;
	uint8 	if_flags;
	uint16	flow_ring_id;
	uint8 	tc;
	uint8	priority;
	uint16 	int_vector;
	uint16	max_items;
	uint16	len_item;
	bcm_addr64_t flow_ring_ptr;
} tx_flowring_create_request_t;

typedef struct tx_flowring_delete_request {
	cmn_msg_hdr_t   msg;
	uint16	flow_ring_id;
	uint16 	reason;
	uint32	rsvd[7];
} tx_flowring_delete_request_t;

typedef struct tx_flowring_flush_request {
	cmn_msg_hdr_t   msg;
	uint16	flow_ring_id;
	uint16 	reason;
	uint32	rsvd[7];
} tx_flowring_flush_request_t;

typedef enum ring_config_subtype {
	
	D2H_RING_CONFIG_SUBTYPE_SOFT_DOORBELL = 1, 
	D2H_RING_CONFIG_SUBTYPE_MSI_DOORBELL = 2   
} ring_config_subtype_t;

typedef struct ring_config_req {
	cmn_msg_hdr_t	msg;
	uint16	subtype;
	uint16	ring_id;
	uint32	rsvd;
	union {
		uint32  data[6];
		
		bcmpcie_soft_doorbell_t soft_doorbell;
	};
} ring_config_req_t;

typedef union ctrl_submit_item {
	ioctl_req_msg_t			ioctl_req;
	ioctl_resp_evt_buf_post_msg_t	resp_buf_post;
	pcie_dma_xfer_params_t		dma_xfer;
	tx_flowring_create_request_t	flow_create;
	tx_flowring_delete_request_t	flow_delete;
	tx_flowring_flush_request_t	flow_flush;
	ring_config_req_t		ring_config_req;
	unsigned char			check[H2DRING_CTRL_SUB_ITEMSIZE];
} ctrl_submit_item_t;

typedef struct compl_msg_hdr {
	
	int16	status;
	
	uint16	flow_ring_id;
} compl_msg_hdr_t;

typedef uint32 dma_done_t;

#define	BCMPCIE_SUCCESS			0
#define BCMPCIE_NOTFOUND		1
#define BCMPCIE_NOMEM			2
#define BCMPCIE_BADOPTION		3
#define BCMPCIE_RING_IN_USE		4
#define BCMPCIE_RING_ID_INVALID		5
#define BCMPCIE_PKT_FLUSH		6
#define BCMPCIE_NO_EVENT_BUF		7
#define BCMPCIE_NO_RX_BUF		8
#define BCMPCIE_NO_IOCTLRESP_BUF	9
#define BCMPCIE_MAX_IOCTLRESP_BUF	10
#define BCMPCIE_MAX_EVENT_BUF		11

typedef struct ioctl_compl_resp_msg {
	
	cmn_msg_hdr_t		cmn_hdr;
	
	compl_msg_hdr_t		compl_hdr;
	
	uint16			resp_len;
	
	uint16			trans_id;
	
	uint32			cmd;
	
	dma_done_t		marker;
} ioctl_comp_resp_msg_t;

typedef struct ioctl_req_ack_msg {
	
	cmn_msg_hdr_t		cmn_hdr;
	
	compl_msg_hdr_t 	compl_hdr;
	
	uint32			cmd;
	uint32			rsvd;
	
	dma_done_t		marker;
} ioctl_req_ack_msg_t;

typedef struct wlevent_req_msg {
	
	cmn_msg_hdr_t		cmn_hdr;
	
	compl_msg_hdr_t		compl_hdr;
	
	uint16			event_data_len;
	
	uint16			seqnum;
	
	uint32			rsvd;
	
	dma_done_t		marker;
} wlevent_req_msg_t;

typedef struct pcie_dmaxfer_cmplt {
	
	cmn_msg_hdr_t		cmn_hdr;
	
	compl_msg_hdr_t		compl_hdr;
	uint32			rsvd[2];
	
	dma_done_t		marker;
} pcie_dmaxfer_cmplt_t;

typedef struct pcie_gen_status {
	
	cmn_msg_hdr_t		cmn_hdr;
	
	compl_msg_hdr_t		compl_hdr;
	uint32			rsvd[2];
	
	dma_done_t		marker;
} pcie_gen_status_t;

typedef struct pcie_ring_status {
	
	cmn_msg_hdr_t		cmn_hdr;
	
	compl_msg_hdr_t		compl_hdr;
	
	uint16			write_idx;
	uint16			rsvd[3];
	
	dma_done_t		marker;
} pcie_ring_status_t;

typedef struct tx_flowring_create_response {
	cmn_msg_hdr_t		msg;
	compl_msg_hdr_t 	cmplt;
	uint32			rsvd[2];
	
	dma_done_t		marker;
} tx_flowring_create_response_t;

typedef struct tx_flowring_delete_response {
	cmn_msg_hdr_t		msg;
	compl_msg_hdr_t 	cmplt;
	uint32			rsvd[2];
	
	dma_done_t		marker;
} tx_flowring_delete_response_t;

typedef struct tx_flowring_flush_response {
	cmn_msg_hdr_t		msg;
	compl_msg_hdr_t 	cmplt;
	uint32			rsvd[2];
	
	dma_done_t		marker;
} tx_flowring_flush_response_t;

typedef struct ctrl_compl_msg {
	
	cmn_msg_hdr_t       cmn_hdr;
	
	compl_msg_hdr_t     compl_hdr;
	uint32          rsvd[2];
	
	dma_done_t      marker;
} ctrl_compl_msg_t;

typedef struct ring_config_resp {
	
	cmn_msg_hdr_t       cmn_hdr;
	
	compl_msg_hdr_t     compl_hdr;
	uint32          rsvd[2];
	
	dma_done_t      marker;
} ring_config_resp_t;

typedef union ctrl_completion_item {
	ioctl_comp_resp_msg_t		ioctl_resp;
	wlevent_req_msg_t		event;
	ioctl_req_ack_msg_t		ioct_ack;
	pcie_dmaxfer_cmplt_t		pcie_xfer_cmplt;
	pcie_gen_status_t		pcie_gen_status;
	pcie_ring_status_t		pcie_ring_status;
	tx_flowring_create_response_t	txfl_create_resp;
	tx_flowring_delete_response_t	txfl_delete_resp;
	tx_flowring_flush_response_t	txfl_flush_resp;
	ctrl_compl_msg_t		ctrl_compl;
	ring_config_resp_t		ring_config_resp;
	unsigned char		check[D2HRING_CTRL_CMPLT_ITEMSIZE];
} ctrl_completion_item_t;

typedef struct host_rxbuf_post {
	
	cmn_msg_hdr_t   cmn_hdr;
	
	uint16		metadata_buf_len;
	
	uint16		data_buf_len;
	
	uint32		rsvd;
	
	bcm_addr64_t	metadata_buf_addr;
	
	bcm_addr64_t	data_buf_addr;
} host_rxbuf_post_t;

typedef union rxbuf_submit_item {
	host_rxbuf_post_t	rxpost;
	unsigned char		check[H2DRING_RXPOST_ITEMSIZE];
} rxbuf_submit_item_t;


typedef struct host_rxbuf_cmpl {
	
	cmn_msg_hdr_t	cmn_hdr;
	
	compl_msg_hdr_t	compl_hdr;
	
	uint16		metadata_len;
	
	uint16		data_len;
	
	uint16		data_offset;
	
	uint16		flags;
	
	uint32		rx_status_0;
	uint32		rx_status_1;
	
	dma_done_t	marker;
} host_rxbuf_cmpl_t;

typedef union rxbuf_complete_item {
	host_rxbuf_cmpl_t	rxcmpl;
	unsigned char		check[D2HRING_RXCMPLT_ITEMSIZE];
} rxbuf_complete_item_t;


typedef struct host_txbuf_post {
	
	cmn_msg_hdr_t   cmn_hdr;
	
	uint8		txhdr[ETHER_HDR_LEN];
	
	uint8		flags;
	
	uint8		seg_cnt;

	
	bcm_addr64_t	metadata_buf_addr;
	
	bcm_addr64_t	data_buf_addr;
	
	uint16		metadata_buf_len;
	
	uint16		data_len;
	
	dma_done_t	marker;
} host_txbuf_post_t;

#define BCMPCIE_PKT_FLAGS_FRAME_802_3	0x01
#define BCMPCIE_PKT_FLAGS_FRAME_802_11	0x02

#define BCMPCIE_PKT_FLAGS_FRAME_EXEMPT_MASK	0x03	
#define BCMPCIE_PKT_FLAGS_FRAME_EXEMPT_SHIFT	0x02	


#define BCMPCIE_PKT_FLAGS_PRIO_SHIFT		5
#define BCMPCIE_PKT_FLAGS_PRIO_MASK		(7 << BCMPCIE_PKT_FLAGS_PRIO_SHIFT)

#define BCMPCIE_TXPOST_FLAGS_FRAME_802_3	BCMPCIE_PKT_FLAGS_FRAME_802_3
#define BCMPCIE_TXPOST_FLAGS_FRAME_802_11	BCMPCIE_PKT_FLAGS_FRAME_802_11
#define BCMPCIE_TXPOST_FLAGS_PRIO_SHIFT		BCMPCIE_PKT_FLAGS_PRIO_SHIFT
#define BCMPCIE_TXPOST_FLAGS_PRIO_MASK		BCMPCIE_PKT_FLAGS_PRIO_MASK

typedef union txbuf_submit_item {
	host_txbuf_post_t	txpost;
	unsigned char		check[H2DRING_TXPOST_ITEMSIZE];
} txbuf_submit_item_t;

typedef struct host_txbuf_cmpl {
	
	cmn_msg_hdr_t	cmn_hdr;
	
	compl_msg_hdr_t	compl_hdr;
	union {
		struct {
			
			uint16	metadata_len;
			
			uint16	tx_status;
		};
		
		dma_done_t	marker;
	};
} host_txbuf_cmpl_t;

typedef union txbuf_complete_item {
	host_txbuf_cmpl_t	txcmpl;
	unsigned char		check[D2HRING_TXCMPLT_ITEMSIZE];
} txbuf_complete_item_t;

#define BCMPCIE_D2H_METADATA_HDRLEN	4
#define BCMPCIE_D2H_METADATA_MINLEN	(BCMPCIE_D2H_METADATA_HDRLEN + 4)

typedef struct ret_buf_ptr {
	uint32 low_addr;
	uint32 high_addr;
} ret_buf_t;


#ifdef PCIE_API_REV1

typedef struct ioctl_hdr {
	uint16 		cmd;
	uint16		retbuf_len;
	uint32		cmd_id;
} ioctl_hdr_t;

typedef struct ioctlptr_hdr {
	uint16 		cmd;
	uint16		retbuf_len;
	uint16 		buflen;
	uint16		rsvd;
	uint32		cmd_id;
} ioctlptr_hdr_t;

#else 

typedef struct ioctl_req_hdr {
	uint32		pkt_id;	
	uint32 		cmd;	
	uint16		retbuf_len;
	uint16 		buflen;
	uint16		xt_id;	
	uint16		rsvd[1];
} ioctl_req_hdr_t;

#endif 


typedef struct ioct_reqst_hdr {
	cmn_msg_hdr_t msg;
#ifdef PCIE_API_REV1
	ioctl_hdr_t ioct_hdr;
#else
	ioctl_req_hdr_t ioct_hdr;
#endif
	ret_buf_t ret_buf;
} ioct_reqst_hdr_t;

typedef struct ioctptr_reqst_hdr {
	cmn_msg_hdr_t msg;
#ifdef PCIE_API_REV1
	ioctlptr_hdr_t ioct_hdr;
#else
	ioctl_req_hdr_t ioct_hdr;
#endif
	ret_buf_t ret_buf;
	ret_buf_t ioct_buf;
} ioctptr_reqst_hdr_t;

typedef struct ioct_resp_hdr {
	cmn_msg_hdr_t   msg;
#ifdef PCIE_API_REV1
	uint32	cmd_id;
#else
	uint32	pkt_id;
#endif
	uint32	status;
	uint32	ret_len;
	uint32  inline_data;
#ifdef PCIE_API_REV1
#else
	uint16	xt_id;	
	uint16	rsvd[1];
#endif
} ioct_resp_hdr_t;

typedef struct msgbuf_ioctl_resp {
	ioct_resp_hdr_t	ioct_hdr;
	ret_buf_t	ret_buf;	
} msgbuf_ioct_resp_t;

typedef struct wl_event_hdr {
	cmn_msg_hdr_t   msg;
	uint16 event;
	uint8 flags;
	uint8 rsvd;
	uint16 retbuf_len;
	uint16 rsvd1;
	uint32 rxbufid;
} wl_event_hdr_t;

#define TXDESCR_FLOWID_PCIELPBK_1	0xFF
#define TXDESCR_FLOWID_PCIELPBK_2	0xFE

typedef struct txbatch_lenptr_tup {
	uint32 pktid;
	uint16 pktlen;
	uint16 rsvd;
	ret_buf_t	ret_buf;	
} txbatch_lenptr_tup_t;

typedef struct txbatch_cmn_msghdr {
	cmn_msg_hdr_t   msg;
	uint8 priority;
	uint8 hdrlen;
	uint8 pktcnt;
	uint8 flowid;
	uint8 txhdr[ETHER_HDR_LEN];
	uint16 rsvd;
} txbatch_cmn_msghdr_t;

typedef struct txbatch_msghdr {
	txbatch_cmn_msghdr_t txcmn;
	txbatch_lenptr_tup_t tx_tup[0]; 
} txbatch_msghdr_t;

typedef struct tx_lenptr_tup {
	uint16 pktlen;
	uint16 rsvd;
	ret_buf_t	ret_buf;	
} tx_lenptr_tup_t;

typedef struct txdescr_cmn_msghdr {
	cmn_msg_hdr_t   msg;
	uint8 priority;
	uint8 hdrlen;
	uint8 descrcnt;
	uint8 flowid;
	uint32 pktid;
} txdescr_cmn_msghdr_t;

typedef struct txdescr_msghdr {
	txdescr_cmn_msghdr_t txcmn;
	uint8 txhdr[ETHER_HDR_LEN];
	uint16 rsvd;
	tx_lenptr_tup_t tx_tup[0];	
} txdescr_msghdr_t;

typedef struct txstatus_hdr {
	cmn_msg_hdr_t   msg;
	uint32 pktid;
} txstatus_hdr_t;

typedef struct rx_lenptr_tup {
	uint32 rxbufid;
	uint16 len;
	uint16 rsvd2;
	ret_buf_t	ret_buf;	
} rx_lenptr_tup_t;

typedef struct rxdesc_msghdr {
	cmn_msg_hdr_t   msg;
	uint16 rsvd0;
	uint8 rsvd1;
	uint8 descnt;
	rx_lenptr_tup_t rx_tup[0];
} rxdesc_msghdr_t;

typedef struct rxcmplt_tup {
	uint16 retbuf_len;
	uint16 data_offset;
	uint32 rxstatus0;
	uint32 rxstatus1;
	uint32 rxbufid;
} rxcmplt_tup_t;

typedef struct rxcmplt_hdr {
	cmn_msg_hdr_t   msg;
	uint16 rsvd0;
	uint16 rxcmpltcnt;
	rxcmplt_tup_t rx_tup[0];
} rxcmplt_hdr_t;

typedef struct hostevent_hdr {
	cmn_msg_hdr_t   msg;
	uint32 evnt_pyld;
} hostevent_hdr_t;

typedef struct dma_xfer_params {
	uint32 src_physaddr_hi;
	uint32 src_physaddr_lo;
	uint32 dest_physaddr_hi;
	uint32 dest_physaddr_lo;
	uint32 len;
	uint32 srcdelay;
	uint32 destdelay;
} dma_xfer_params_t;

enum {
	HOST_EVENT_CONS_CMD = 1
};

#define MSGBUF_IOC_ACTION_MASK 0x1

#define MAX_SUSPEND_REQ 15

typedef struct tx_idle_flowring_suspend_request {
	cmn_msg_hdr_t	msg;
	uint16	ring_id[MAX_SUSPEND_REQ];      
	uint16	num;    
} tx_idle_flowring_suspend_request_t;

typedef struct tx_idle_flowring_suspend_response {
	cmn_msg_hdr_t	msg;
	compl_msg_hdr_t	cmplt;
	uint32			rsvd[2];
	dma_done_t		marker;
} tx_idle_flowring_suspend_response_t;

typedef struct tx_idle_flowring_resume_request {
	cmn_msg_hdr_t	msg;
	uint16	flow_ring_id;
	uint16	reason;
	uint32	rsvd[7];
} tx_idle_flowring_resume_request_t;

typedef struct tx_idle_flowring_resume_response {
	cmn_msg_hdr_t	msg;
	compl_msg_hdr_t	cmplt;
	uint32			rsvd[2];
	dma_done_t		marker;
} tx_idle_flowring_resume_response_t;

#endif 
