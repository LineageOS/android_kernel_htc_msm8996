/**
 * @file definition of host message ring functionality
 * Provides type definitions and function prototypes used to link the
 * DHD OS, bus, and protocol modules.
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
 * $Id: dhd_msgbuf.c 619124 2016-02-15 09:26:47Z $
 */


#include <typedefs.h>
#include <osl.h>

#include <bcmutils.h>
#include <bcmmsgbuf.h>
#include <bcmendian.h>

#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_proto.h>

#include <dhd_bus.h>

#include <dhd_dbg.h>
#include <siutils.h>


#include <dhd_flowring.h>

#include <pcie_core.h>
#include <bcmpcie.h>
#include <dhd_pcie.h>

#if defined(DHD_LB)
#include <linux/cpu.h>
#if defined(DHD_LB_RXC) || defined(DHD_LB_TXC)
#include <bcm_ring.h>
#endif
#define DHD_LB_WORKQ_SZ                            (8192)
#define DHD_LB_WORKQ_SYNC           (16)
#define DHD_LB_WORK_SCHED           (DHD_LB_WORKQ_SYNC * 2)
#endif 


/**
 * Host configures a soft doorbell for d2h rings, by specifying a 32bit host
 * address where a value must be written. Host may also interrupt coalescing
 * on this soft doorbell.
 * Use Case: Hosts with network processors, may register with the dongle the
 * network processor's thread wakeup register and a value corresponding to the
 * core/thread context. Dongle will issue a write transaction <address,value>
 * to the PCIE RC which will need to be routed to the mapped register space, by
 * the host.
 */

#if defined(IOCTLRESP_USE_CONSTMEM) && defined(DHD_USE_STATIC_CTRLBUF)
#error "DHD_USE_STATIC_CTRLBUF is NOT working with DHD_USE_OSLPKT_FOR_RESPBUF"
#endif 

#define RETRIES 2		

#define DEFAULT_RX_BUFFERS_TO_POST	256
#define RXBUFPOST_THRESHOLD			32
#define RX_BUF_BURST				32 

#define DHD_STOP_QUEUE_THRESHOLD	200
#define DHD_START_QUEUE_THRESHOLD	100

#define RX_DMA_OFFSET		8 
#define IOCT_RETBUF_SIZE	(RX_DMA_OFFSET + WLC_IOCTL_MAXLEN)
#define FLOWRING_SIZE		(H2DRING_TXPOST_MAX_ITEM * H2DRING_TXPOST_ITEMSIZE)

#define MSGBUF_IOCTL_ACK_PENDING	(1<<0)
#define MSGBUF_IOCTL_RESP_PENDING	(1<<1)

#define DMA_ALIGN_LEN		4

#define DMA_D2H_SCRATCH_BUF_LEN	8
#define DMA_XFER_LEN_LIMIT	0x400000

#ifdef BCM_HOST_BUF
#ifndef DMA_HOST_BUFFER_LEN
#define DMA_HOST_BUFFER_LEN	0x200000
#endif
#endif 

#define DHD_FLOWRING_IOCTL_BUFPOST_PKTSZ		8192

#define DHD_FLOWRING_MAX_EVENTBUF_POST			8
#define DHD_FLOWRING_MAX_IOCTLRESPBUF_POST		8

#define DHD_PROT_FUNCS	37

#define DHD_BUS_TPUT_BUF_LEN 2048

#define TXP_FLUSH_NITEMS

#define TXP_FLUSH_MAX_ITEMS_FLUSH_CNT	48

#define RING_NAME_MAX_LENGTH		24


struct msgbuf_ring; 

#define PCIE_D2H_SYNC

#if defined(PCIE_D2H_SYNC)
#define PCIE_D2H_SYNC_WAIT_TRIES    (512UL)
#define PCIE_D2H_SYNC_NUM_OF_STEPS	(3UL)
#define PCIE_D2H_SYNC_DELAY			(50UL)	

typedef uint8 (* d2h_sync_cb_t)(dhd_pub_t *dhd, struct msgbuf_ring *ring,
                                volatile cmn_msg_hdr_t *msg, int msglen);
#endif 



#define DHD_FLOWRING_START_FLOWID   BCMPCIE_H2D_COMMON_MSGRINGS

#define DHD_IS_FLOWRING(ringid) \
	((ringid) >= BCMPCIE_COMMON_MSGRINGS)

#define DHD_FLOWID_TO_RINGID(flowid) \
	(BCMPCIE_COMMON_MSGRINGS + ((flowid) - BCMPCIE_H2D_COMMON_MSGRINGS))

#define DHD_RINGID_TO_FLOWID(ringid) \
	(BCMPCIE_H2D_COMMON_MSGRINGS + ((ringid) - BCMPCIE_COMMON_MSGRINGS))

#define DHD_H2D_RING_OFFSET(ringid) \
	((DHD_IS_FLOWRING(ringid)) ? DHD_RINGID_TO_FLOWID(ringid) : (ringid))

#define DHD_D2H_RING_OFFSET(ringid) \
	((ringid) - BCMPCIE_H2D_COMMON_MSGRINGS)

#define DHD_D2H_RINGID(offset) \
	((offset) + BCMPCIE_H2D_COMMON_MSGRINGS)


#define DHD_DMAH_NULL      ((void*)NULL)

#if defined(L1_CACHE_BYTES)
#define DHD_DMA_PAD        (L1_CACHE_BYTES)
#else
#define DHD_DMA_PAD        (128)
#endif

typedef struct dhd_dmaxfer {
	dhd_dma_buf_t srcmem;
	dhd_dma_buf_t dstmem;
	uint32        srcdelay;
	uint32        destdelay;
	uint32        len;
	bool          in_progress;
} dhd_dmaxfer_t;

typedef struct msgbuf_ring {
	bool           inited;
	uint16         idx;       
	uint16         rd;        
	uint16         wr;        
	uint16         max_items; 
	uint16         item_len;  
	sh_addr_t      base_addr; 
	dhd_dma_buf_t  dma_buf;   
	uint32         seqnum;    
#ifdef TXP_FLUSH_NITEMS
	void           *start_addr;
	
	uint16         pend_items_count;
#endif 
	uchar		name[RING_NAME_MAX_LENGTH];
} msgbuf_ring_t;

#define DHD_RING_BGN_VA(ring)           ((ring)->dma_buf.va)
#define DHD_RING_END_VA(ring) \
	((uint8 *)(DHD_RING_BGN_VA((ring))) + \
	 (((ring)->max_items - 1) * (ring)->item_len))



typedef struct dhd_prot {
	osl_t *osh;		
	uint16 rxbufpost;
	uint16 max_rxbufpost;
	uint16 max_eventbufpost;
	uint16 max_ioctlrespbufpost;
	uint16 cur_event_bufs_posted;
	uint16 cur_ioctlresp_bufs_posted;

	
	uint16 active_tx_count; 
	uint16 max_tx_count;
	uint16 txp_threshold;  

	
	msgbuf_ring_t h2dring_ctrl_subn; 
	msgbuf_ring_t h2dring_rxp_subn; 
	msgbuf_ring_t d2hring_ctrl_cpln; 
	msgbuf_ring_t d2hring_tx_cpln; 
	msgbuf_ring_t d2hring_rx_cpln; 

	msgbuf_ring_t *h2d_flowrings_pool; 
	dhd_dma_buf_t flowrings_dma_buf; 
	uint16        h2d_rings_total; 

	uint32		rx_dataoffset;

	dhd_mb_ring_t	mb_ring_fn;	

	
	uint8 ioctl_state;
	int16 ioctl_status;		
	uint16 ioctl_resplen;
	dhd_ioctl_recieved_status_t ioctl_received;
	dhd_dma_buf_t	retbuf;		
	dhd_dma_buf_t	ioctbuf;	

	dhd_dma_buf_t	d2h_dma_scratch_buf;	

	
	uint32          rw_index_sz; 
	dhd_dma_buf_t   h2d_dma_indx_wr_buf;	
	dhd_dma_buf_t	h2d_dma_indx_rd_buf;	
	dhd_dma_buf_t	d2h_dma_indx_wr_buf;	
	dhd_dma_buf_t	d2h_dma_indx_rd_buf;	

	dhd_dma_buf_t	host_bus_throughput_buf; 

	dhd_dma_buf_t   *flowring_buf;    
	uint32			flowring_num;

#if defined(PCIE_D2H_SYNC)
	d2h_sync_cb_t d2h_sync_cb; 
	ulong d2h_sync_wait_max; 
	ulong d2h_sync_wait_tot; 
#endif  

	dhd_dmaxfer_t	dmaxfer; 

	uint16		ioctl_seq_no;
	uint16		data_seq_no;
	uint16		ioctl_trans_id;
	void		*pktid_map_handle; 
	bool		metadata_dbg;
	void		*pktid_map_handle_ioctl;

	
	uint16		rx_metadata_offset;
	uint16		tx_metadata_offset;


#if defined(DHD_D2H_SOFT_DOORBELL_SUPPORT)
	
	bcmpcie_soft_doorbell_t soft_doorbell[BCMPCIE_D2H_COMMON_MSGRINGS];
#endif 
#if defined(DHD_LB)
#if defined(DHD_LB_TXC)
	uint32 tx_compl_prod_sync ____cacheline_aligned;
	bcm_workq_t tx_compl_prod, tx_compl_cons;
#endif 
#if defined(DHD_LB_RXC)
	uint32 rx_compl_prod_sync ____cacheline_aligned;
	bcm_workq_t rx_compl_prod, rx_compl_cons;
#endif 
#endif 
#ifdef DHD_TRACE_WAKE_LOCK
	ulong		wake_lock_dbg_time;
#endif
} dhd_prot_t;

static INLINE void dhd_base_addr_htolpa(sh_addr_t *base_addr, dmaaddr_t pa);

static int  dhd_dma_buf_audit(dhd_pub_t *dhd, dhd_dma_buf_t *dma_buf);
static int  dhd_dma_buf_alloc(dhd_pub_t *dhd, dhd_dma_buf_t *dma_buf, uint32 buf_len);
static void dhd_dma_buf_reset(dhd_pub_t *dhd, dhd_dma_buf_t *dma_buf);
static void dhd_dma_buf_free(dhd_pub_t *dhd, dhd_dma_buf_t *dma_buf);

static int dhd_prot_ring_attach(dhd_pub_t *dhd, msgbuf_ring_t *ring,
	const char *name, uint16 max_items, uint16 len_item, uint16 ringid);
static void dhd_prot_ring_init(dhd_pub_t *dhd, msgbuf_ring_t *ring);
static void dhd_prot_ring_reset(dhd_pub_t *dhd, msgbuf_ring_t *ring);
static void dhd_prot_ring_detach(dhd_pub_t *dhd, msgbuf_ring_t *ring);

static int  dhd_prot_flowrings_pool_attach(dhd_pub_t *dhd);
static void dhd_prot_flowrings_pool_reset(dhd_pub_t *dhd);
static void dhd_prot_flowrings_pool_detach(dhd_pub_t *dhd);

static msgbuf_ring_t *dhd_prot_flowrings_pool_fetch(dhd_pub_t *dhd,
	uint16 flowid);

static void* dhd_prot_alloc_ring_space(dhd_pub_t *dhd, msgbuf_ring_t *ring,
	uint16 nitems, uint16 *alloced, bool exactly_nitems);
static void* dhd_prot_get_ring_space(msgbuf_ring_t *ring, uint16 nitems,
	uint16 *alloced, bool exactly_nitems);

static uint8* dhd_prot_get_read_addr(dhd_pub_t *dhd, msgbuf_ring_t *ring,
	uint32 *available_len);

static void dhd_prot_ring_write_complete(dhd_pub_t *dhd, msgbuf_ring_t *ring,
	void *p, uint16 len);
static void dhd_prot_upd_read_idx(dhd_pub_t *dhd, msgbuf_ring_t *ring);

static INLINE int dhd_prot_dma_indx_alloc(dhd_pub_t *dhd, uint8 type,
	dhd_dma_buf_t *dma_buf, uint32 bufsz);

static void dhd_prot_dma_indx_set(dhd_pub_t *dhd, uint16 new_index, uint8 type,
	uint16 ringid);
static uint16 dhd_prot_dma_indx_get(dhd_pub_t *dhd, uint8 type, uint16 ringid);

static INLINE void *dhd_prot_packet_get(dhd_pub_t *dhd, uint32 pktid, uint8 pkttype,
	bool free_pktid);
static INLINE void dhd_prot_packet_free(dhd_pub_t *dhd, void *pkt, uint8 pkttype, bool send);

static int dhd_msgbuf_query_ioctl(dhd_pub_t *dhd, int ifidx, uint cmd,
	void *buf, uint len, uint8 action);
static int dhd_msgbuf_set_ioctl(dhd_pub_t *dhd, int ifidx, uint cmd,
	void *buf, uint len, uint8 action);
static int dhd_msgbuf_wait_ioctl_cmplt(dhd_pub_t *dhd, uint32 len, void *buf);
static int dhd_fillup_ioct_reqst(dhd_pub_t *dhd, uint16 len, uint cmd,
	void *buf, int ifidx);

static uint16 dhd_msgbuf_rxbuf_post_ctrlpath(dhd_pub_t *dhd, bool event_buf, uint32 max_to_post);
static void dhd_msgbuf_rxbuf_post_ioctlresp_bufs(dhd_pub_t *pub);
static void dhd_msgbuf_rxbuf_post_event_bufs(dhd_pub_t *pub);
static void dhd_msgbuf_rxbuf_post(dhd_pub_t *dhd, bool use_rsv_pktid);
static int dhd_prot_rxbuf_post(dhd_pub_t *dhd, uint16 count, bool use_rsv_pktid);

static void dhd_prot_return_rxbuf(dhd_pub_t *dhd, uint32 pktid, uint32 rxcnt);

static int dhd_prot_process_msgtype(dhd_pub_t *dhd, msgbuf_ring_t *ring, uint8 *buf, uint32 len);

static void dhd_prot_noop(dhd_pub_t *dhd, void *msg);
static void dhd_prot_txstatus_process(dhd_pub_t *dhd, void *msg);
static void dhd_prot_ioctcmplt_process(dhd_pub_t *dhd, void *msg);
static void dhd_prot_ioctack_process(dhd_pub_t *dhd, void *msg);
static void dhd_prot_ringstatus_process(dhd_pub_t *dhd, void *msg);
static void dhd_prot_genstatus_process(dhd_pub_t *dhd, void *msg);
static void dhd_prot_rxcmplt_process(dhd_pub_t *dhd, void *msg);
static void dhd_prot_event_process(dhd_pub_t *dhd, void *msg);

static void dmaxfer_free_dmaaddr(dhd_pub_t *dhd, dhd_dmaxfer_t *dma);
static int dmaxfer_prepare_dmaaddr(dhd_pub_t *dhd, uint len, uint srcdelay,
	uint destdelay, dhd_dmaxfer_t *dma);
static void dhd_msgbuf_dmaxfer_process(dhd_pub_t *dhd, void *msg);

static void dhd_prot_flow_ring_create_response_process(dhd_pub_t *dhd, void *msg);
static void dhd_prot_flow_ring_delete_response_process(dhd_pub_t *dhd, void *msg);
static void dhd_prot_flow_ring_flush_response_process(dhd_pub_t *dhd, void *msg);

static void dhd_msgbuf_ring_config_d2h_soft_doorbell(dhd_pub_t *dhd);
static void dhd_prot_d2h_ring_config_cmplt_process(dhd_pub_t *dhd, void *msg);

typedef void (*dhd_msgbuf_func_t)(dhd_pub_t *dhd, void *msg);

#define MSG_TYPE_INVALID 0

static dhd_msgbuf_func_t table_lookup[DHD_PROT_FUNCS] = {
	dhd_prot_noop, 
	dhd_prot_genstatus_process, 
	dhd_prot_ringstatus_process, 
	NULL,
	dhd_prot_flow_ring_create_response_process, 
	NULL,
	dhd_prot_flow_ring_delete_response_process, 
	NULL,
	dhd_prot_flow_ring_flush_response_process, 
	NULL,
	dhd_prot_ioctack_process, 
	NULL,
	dhd_prot_ioctcmplt_process, 
	NULL,
	dhd_prot_event_process, 
	NULL,
	dhd_prot_txstatus_process, 
	NULL,
	dhd_prot_rxcmplt_process, 
	NULL,
	dhd_msgbuf_dmaxfer_process, 
	NULL, 
	NULL, 
	NULL, 
	NULL, 
	NULL, 
	NULL, 
	NULL, 
	NULL, 
	NULL, 
	NULL, 
	NULL, 
	NULL, 
	NULL, 
	dhd_prot_d2h_ring_config_cmplt_process, 
	NULL, 
	NULL, 
};


#ifdef DHD_RX_CHAINING

#define PKT_CTF_CHAINABLE(dhd, ifidx, evh, prio, h_sa, h_da, h_prio) \
	(!ETHER_ISNULLDEST(((struct ether_header *)(evh))->ether_dhost) && \
	 !ETHER_ISMULTI(((struct ether_header *)(evh))->ether_dhost) && \
	 !eacmp((h_da), ((struct ether_header *)(evh))->ether_dhost) && \
	 !eacmp((h_sa), ((struct ether_header *)(evh))->ether_shost) && \
	 ((h_prio) == (prio)) && (dhd_ctf_hotbrc_check((dhd), (evh), (ifidx))) && \
	 ((((struct ether_header *)(evh))->ether_type == HTON16(ETHER_TYPE_IP)) || \
	 (((struct ether_header *)(evh))->ether_type == HTON16(ETHER_TYPE_IPV6))) && \
	 dhd_l2_filter_chainable((dhd), (evh), (ifidx)))

static INLINE void BCMFASTPATH dhd_rxchain_reset(rxchain_info_t *rxchain);
static void BCMFASTPATH dhd_rxchain_frame(dhd_pub_t *dhd, void *pkt, uint ifidx);
static void BCMFASTPATH dhd_rxchain_commit(dhd_pub_t *dhd);

#define DHD_PKT_CTF_MAX_CHAIN_LEN	64

#endif 

static void dhd_prot_h2d_sync_init(dhd_pub_t *dhd);

#if defined(PCIE_D2H_SYNC) 

static void dhd_prot_d2h_sync_livelock(dhd_pub_t *dhd, msgbuf_ring_t *ring,
                                       uint32 tries, uchar *msg, int msglen);
static uint8 dhd_prot_d2h_sync_seqnum(dhd_pub_t *dhd, msgbuf_ring_t *ring,
                                      volatile cmn_msg_hdr_t *msg, int msglen);
static uint8 dhd_prot_d2h_sync_xorcsum(dhd_pub_t *dhd, msgbuf_ring_t *ring,
                                       volatile cmn_msg_hdr_t *msg, int msglen);
static uint8 dhd_prot_d2h_sync_none(dhd_pub_t *dhd, msgbuf_ring_t *ring,
                                    volatile cmn_msg_hdr_t *msg, int msglen);
static void dhd_prot_d2h_sync_init(dhd_pub_t *dhd);

static void
dhd_prot_d2h_sync_livelock(dhd_pub_t *dhd, msgbuf_ring_t *ring, uint32 tries,
                           uchar *msg, int msglen)
{
	uint32 seqnum = ring->seqnum;

	DHD_ERROR(("LIVELOCK DHD<%p> seqnum<%u:%u> tries<%u> max<%lu> tot<%lu>"
		"dma_buf va<%p> msg<%p>\n",
		dhd, seqnum, seqnum% D2H_EPOCH_MODULO, tries,
		dhd->prot->d2h_sync_wait_max, dhd->prot->d2h_sync_wait_tot,
		ring->dma_buf.va, msg));
	prhex("D2H MsgBuf Failure", (uchar *)msg, msglen);

#if defined(SUPPORT_LINKDOWN_RECOVERY) && defined(CONFIG_ARCH_MSM)
	dhd->bus->islinkdown = TRUE;
	dhd_os_check_hang(dhd, 0, -ETIMEDOUT);
#endif 
}

static uint8 BCMFASTPATH
dhd_prot_d2h_sync_seqnum(dhd_pub_t *dhd, msgbuf_ring_t *ring,
                         volatile cmn_msg_hdr_t *msg, int msglen)
{
	uint32 tries;
	uint32 ring_seqnum = ring->seqnum % D2H_EPOCH_MODULO;
	int num_words = msglen / sizeof(uint32); 
	volatile uint32 *marker = (uint32 *)msg + (num_words - 1); 
	dhd_prot_t *prot = dhd->prot;
	uint32 step = 0;
	uint32 delay = PCIE_D2H_SYNC_DELAY;
	uint32 total_tries = 0;

	ASSERT(msglen == ring->item_len);

	BCM_REFERENCE(delay);
	for (step = 1; step <= PCIE_D2H_SYNC_NUM_OF_STEPS; step++) {
		for (tries = 1; tries <= PCIE_D2H_SYNC_WAIT_TRIES; tries++) {
			uint32 msg_seqnum = *marker;
			if (ltoh32(msg_seqnum) == ring_seqnum) { 
				ring->seqnum++; 
				goto dma_completed;
			}

			total_tries = ((step-1) * PCIE_D2H_SYNC_NUM_OF_STEPS) + tries;

			if (total_tries > prot->d2h_sync_wait_max)
				prot->d2h_sync_wait_max = total_tries;

			OSL_CACHE_INV(msg, msglen); 
			OSL_CPU_RELAX(); 
#if defined(CONFIG_ARCH_MSM8996) || defined(CONFIG_SOC_EXYNOS8890)
			
			OSL_DELAY(delay * step);
#endif 
		} 
	} 

	dhd_prot_d2h_sync_livelock(dhd, ring, total_tries, (uchar *)msg, msglen);

	ring->seqnum++; 
	return MSG_TYPE_INVALID; 

dma_completed:

	prot->d2h_sync_wait_tot += total_tries;
	return msg->msg_type;
}

static uint8 BCMFASTPATH
dhd_prot_d2h_sync_xorcsum(dhd_pub_t *dhd, msgbuf_ring_t *ring,
                          volatile cmn_msg_hdr_t *msg, int msglen)
{
	uint32 tries;
	uint32 prot_checksum = 0; 
	int num_words = msglen / sizeof(uint32); 
	uint8 ring_seqnum = ring->seqnum % D2H_EPOCH_MODULO;
	dhd_prot_t *prot = dhd->prot;
	uint32 step = 0;
	uint32 delay = PCIE_D2H_SYNC_DELAY;
	uint32 total_tries = 0;

	ASSERT(msglen == ring->item_len);

	BCM_REFERENCE(delay);

	for (step = 1; step <= PCIE_D2H_SYNC_NUM_OF_STEPS; step++) {
		for (tries = 1; tries <= PCIE_D2H_SYNC_WAIT_TRIES; tries++) {
			prot_checksum = bcm_compute_xor32((volatile uint32 *)msg, num_words);
			if (prot_checksum == 0U) { 
				if (msg->epoch == ring_seqnum) {
					ring->seqnum++; 
					goto dma_completed;
				}
			}

			total_tries = ((step-1) * PCIE_D2H_SYNC_NUM_OF_STEPS) + tries;

			if (total_tries > prot->d2h_sync_wait_max)
				prot->d2h_sync_wait_max = total_tries;

			OSL_CACHE_INV(msg, msglen); 
			OSL_CPU_RELAX(); 
#if defined(CONFIG_ARCH_MSM8996) || defined(CONFIG_SOC_EXYNOS8890)
			
			OSL_DELAY(delay * step);
#endif 

		} 
	} 

	dhd_prot_d2h_sync_livelock(dhd, ring, total_tries, (uchar *)msg, msglen);

	ring->seqnum++; 
	return MSG_TYPE_INVALID; 

dma_completed:

	prot->d2h_sync_wait_tot += total_tries;
	return msg->msg_type;
}

static uint8 BCMFASTPATH
dhd_prot_d2h_sync_none(dhd_pub_t *dhd, msgbuf_ring_t *ring,
                       volatile cmn_msg_hdr_t *msg, int msglen)
{
	return msg->msg_type;
}

static void
dhd_prot_d2h_sync_init(dhd_pub_t *dhd)
{
	dhd_prot_t *prot = dhd->prot;
	prot->d2h_sync_wait_max = 0UL;
	prot->d2h_sync_wait_tot = 0UL;

	prot->d2hring_ctrl_cpln.seqnum = D2H_EPOCH_INIT_VAL;
	prot->d2hring_tx_cpln.seqnum = D2H_EPOCH_INIT_VAL;
	prot->d2hring_rx_cpln.seqnum = D2H_EPOCH_INIT_VAL;

	if (dhd->d2h_sync_mode & PCIE_SHARED_D2H_SYNC_SEQNUM) {
		prot->d2h_sync_cb = dhd_prot_d2h_sync_seqnum;
	} else if (dhd->d2h_sync_mode & PCIE_SHARED_D2H_SYNC_XORCSUM) {
		prot->d2h_sync_cb = dhd_prot_d2h_sync_xorcsum;
	} else {
		prot->d2h_sync_cb = dhd_prot_d2h_sync_none;
	}
}

#endif 

int INLINE
dhd_wakeup_ioctl_event(dhd_pub_t *dhd, dhd_ioctl_recieved_status_t reason)
{
	
	OSL_SMP_WMB();
	dhd->prot->ioctl_received = reason;
	
	OSL_SMP_WMB();
	dhd_os_ioctl_resp_wake(dhd);
	return 0;
}

static void
dhd_prot_h2d_sync_init(dhd_pub_t *dhd)
{
	dhd_prot_t *prot = dhd->prot;
	prot->h2dring_rxp_subn.seqnum = H2D_EPOCH_INIT_VAL;
	prot->h2dring_ctrl_subn.seqnum = H2D_EPOCH_INIT_VAL;
}




static INLINE void
dhd_base_addr_htolpa(sh_addr_t *base_addr, dmaaddr_t pa)
{
	base_addr->low_addr = htol32(PHYSADDRLO(pa));
	base_addr->high_addr = htol32(PHYSADDRHI(pa));
}


static int
dhd_dma_buf_audit(dhd_pub_t *dhd, dhd_dma_buf_t *dma_buf)
{
	uint32 base, end; 

	ASSERT(dma_buf);
	base = PHYSADDRLO(dma_buf->pa);
	ASSERT(base);
	ASSERT(ISALIGNED(base, DMA_ALIGN_LEN));
	ASSERT(dma_buf->len != 0);

	
	end = (base + dma_buf->len); 

	if ((end & 0xFFFFFFFF) < (base & 0xFFFFFFFF)) { 
		DHD_ERROR(("%s: dma_buf %x len %d spans dongle 32bit ptr arithmetic\n",
			__FUNCTION__, base, dma_buf->len));
		return BCME_ERROR;
	}

	return BCME_OK;
}

static int
dhd_dma_buf_alloc(dhd_pub_t *dhd, dhd_dma_buf_t *dma_buf, uint32 buf_len)
{
	uint32 dma_pad = 0;
	osl_t *osh = dhd->osh;

	ASSERT(dma_buf != NULL);
	ASSERT(dma_buf->va == NULL);
	ASSERT(dma_buf->len == 0);

	dma_pad = (buf_len % DHD_DMA_PAD) ? DHD_DMA_PAD : 0;
	dma_buf->va = DMA_ALLOC_CONSISTENT(osh, buf_len + dma_pad,
		DMA_ALIGN_LEN, &dma_buf->_alloced, &dma_buf->pa, &dma_buf->dmah);

	if (dma_buf->va == NULL) {
		DHD_ERROR(("%s: buf_len %d, no memory available\n",
			__FUNCTION__, buf_len));
		return BCME_NOMEM;
	}

	dma_buf->len = buf_len; 

	if (dhd_dma_buf_audit(dhd, dma_buf) != BCME_OK) { 
		dhd_dma_buf_free(dhd, dma_buf);
		return BCME_ERROR;
	}

	dhd_dma_buf_reset(dhd, dma_buf); 

	return BCME_OK;
}

static void
dhd_dma_buf_reset(dhd_pub_t *dhd, dhd_dma_buf_t *dma_buf)
{
	if ((dma_buf == NULL) || (dma_buf->va == NULL)) {
		return;
	}

	(void)dhd_dma_buf_audit(dhd, dma_buf);

	
	memset((void*)dma_buf->va, 0, dma_buf->len);
	OSL_CACHE_FLUSH((void *)dma_buf->va, dma_buf->len);
}

static void
dhd_dma_buf_free(dhd_pub_t *dhd, dhd_dma_buf_t *dma_buf)
{
	osl_t *osh = dhd->osh;

	ASSERT(dma_buf);

	if (dma_buf->va == NULL) {
		return; 
	}

	
	(void)dhd_dma_buf_audit(dhd, dma_buf);

	
	DMA_FREE_CONSISTENT(osh, dma_buf->va, dma_buf->_alloced,
		dma_buf->pa, dma_buf->dmah);

	memset(dma_buf, 0, sizeof(dhd_dma_buf_t));
}

void
dhd_dma_buf_init(dhd_pub_t *dhd, void *dhd_dma_buf,
	void *va, uint32 len, dmaaddr_t pa, void *dmah, void *secdma)
{
	dhd_dma_buf_t *dma_buf;
	ASSERT(dhd_dma_buf);
	dma_buf = (dhd_dma_buf_t *)dhd_dma_buf;
	dma_buf->va = va;
	dma_buf->len = len;
	dma_buf->pa = pa;
	dma_buf->dmah = dmah;
	dma_buf->secdma = secdma;

	
	(void)dhd_dma_buf_audit(dhd, dma_buf);
}


#define DHD_PCIE_PKTID
#define MAX_PKTID_ITEMS     (3072) 



#if defined(PROP_TXSTATUS) && !defined(DHD_PCIE_PKTID)
#error "PKTIDMAP must be supported with PROP_TXSTATUS/WLFC"
#endif

typedef enum dhd_pkttype {
	PKTTYPE_DATA_TX = 0,
	PKTTYPE_DATA_RX,
	PKTTYPE_IOCTL_RX,
	PKTTYPE_EVENT_RX,
	
	PKTTYPE_NO_CHECK
} dhd_pkttype_t;

#define DHD_PKTID_INVALID               (0U)
#define DHD_IOCTL_REQ_PKTID             (0xFFFE)
#define DHD_FAKE_PKTID                  (0xFACE)

#define DHD_PKTID_FREE_LOCKER           (FALSE)
#define DHD_PKTID_RSV_LOCKER            (TRUE)

typedef void * dhd_pktid_map_handle_t; 

static dhd_pktid_map_handle_t *dhd_pktid_map_init(dhd_pub_t *dhd, uint32 num_items, uint32 index);

static void dhd_pktid_map_fini(dhd_pub_t *dhd, dhd_pktid_map_handle_t *map);

#define PKTID_MAP_HANDLE	(0)
#define PKTID_MAP_HANDLE_IOCTL	(1)

#define DHD_NATIVE_TO_PKTID_INIT(dhd, items, index) dhd_pktid_map_init((dhd), (items), (index))
#define DHD_NATIVE_TO_PKTID_FINI(dhd, map)   dhd_pktid_map_fini((dhd), (map))

#if defined(DHD_PCIE_PKTID)


static INLINE uint32 dhd_pktid_map_avail_cnt(dhd_pktid_map_handle_t *handle);

static INLINE uint32 dhd_pktid_map_reserve(dhd_pub_t *dhd, dhd_pktid_map_handle_t *handle,
	void *pkt);
static INLINE void dhd_pktid_map_save(dhd_pub_t *dhd, dhd_pktid_map_handle_t *handle,
	void *pkt, uint32 nkey, dmaaddr_t pa, uint32 len, uint8 dma,
	void *dmah, void *secdma, dhd_pkttype_t pkttype);
static uint32 dhd_pktid_map_alloc(dhd_pub_t *dhd, dhd_pktid_map_handle_t *map,
	void *pkt, dmaaddr_t pa, uint32 len, uint8 dma,
	void *dmah, void *secdma, dhd_pkttype_t pkttype);

static void *dhd_pktid_map_free(dhd_pub_t *dhd, dhd_pktid_map_handle_t *map,
	uint32 id, dmaaddr_t *pa, uint32 *len, void **dmah,
	void **secdma, dhd_pkttype_t pkttype, bool rsv_locker);

#ifndef DHD_PKTID_AUDIT_ENABLED
#define DHD_PKTID_AUDIT_ENABLED 1
#endif 


#if defined(DHD_PKTID_AUDIT_ENABLED)
#define USE_DHD_PKTID_AUDIT_LOCK 1

#define DHD_PKTID_AUDIT_RING

#if defined(DHD_PKTID_AUDIT_MAP) && defined(DHD_PKTID_AUDIT_RING)
#error "May only enabled audit of MAP or RING, at a time."
#endif 

#define DHD_DUPLICATE_ALLOC     1
#define DHD_DUPLICATE_FREE      2
#define DHD_TEST_IS_ALLOC       3
#define DHD_TEST_IS_FREE        4

#ifdef USE_DHD_PKTID_AUDIT_LOCK
#define DHD_PKTID_AUDIT_LOCK_INIT(osh)          dhd_os_spin_lock_init(osh)
#define DHD_PKTID_AUDIT_LOCK_DEINIT(osh, lock)  dhd_os_spin_lock_deinit(osh, lock)
#define DHD_PKTID_AUDIT_LOCK(lock)              dhd_os_spin_lock(lock)
#define DHD_PKTID_AUDIT_UNLOCK(lock, flags)     dhd_os_spin_unlock(lock, flags)
#else
#define DHD_PKTID_AUDIT_LOCK_INIT(osh)          (void *)(1)
#define DHD_PKTID_AUDIT_LOCK_DEINIT(osh, lock)  do {  } while (0)
#define DHD_PKTID_AUDIT_LOCK(lock)              0
#define DHD_PKTID_AUDIT_UNLOCK(lock, flags)     do {  } while (0)
#endif 

#endif 


#ifdef USE_DHD_PKTID_LOCK
#define DHD_PKTID_LOCK_INIT(osh)                dhd_os_spin_lock_init(osh)
#define DHD_PKTID_LOCK_DEINIT(osh, lock)        dhd_os_spin_lock_deinit(osh, lock)
#define DHD_PKTID_LOCK(lock)                    dhd_os_spin_lock(lock)
#define DHD_PKTID_UNLOCK(lock, flags)           dhd_os_spin_unlock(lock, flags)
#else
#define DHD_PKTID_LOCK_INIT(osh)                (void *)(1)
#define DHD_PKTID_LOCK_DEINIT(osh, lock)	\
	do { \
		BCM_REFERENCE(osh); \
		BCM_REFERENCE(lock); \
	} while (0)
#define DHD_PKTID_LOCK(lock)                    0
#define DHD_PKTID_UNLOCK(lock, flags)           \
	do { \
		BCM_REFERENCE(lock); \
		BCM_REFERENCE(flags); \
	} while (0)
#endif 


typedef enum dhd_locker_state {
	LOCKER_IS_FREE,
	LOCKER_IS_BUSY,
	LOCKER_IS_RSVD
} dhd_locker_state_t;

typedef struct dhd_pktid_item {
	dhd_locker_state_t state;  
	uint8         dir;      
	dhd_pkttype_t pkttype;  
	uint16        len;      
	void          *pkt;     
	dmaaddr_t     pa;       
	void          *dmah;    
	void          *secdma;
} dhd_pktid_item_t;

typedef struct dhd_pktid_map {
	uint32      items;    
	uint32      avail;    
	int         failures; 
	
	void        *pktid_lock; 

#if defined(DHD_PKTID_AUDIT_ENABLED)
	void	    *pktid_audit_lock;
	struct bcm_mwbmap *pktid_audit; 
#endif 

	uint32      keys[MAX_PKTID_ITEMS + 1]; 
	dhd_pktid_item_t lockers[0];           
} dhd_pktid_map_t;


#define DHD_PKTID_ITEM_SZ               (sizeof(dhd_pktid_item_t))
#define DHD_PKIDMAP_ITEMS(items)        (items)
#define DHD_PKTID_MAP_SZ(items)         (sizeof(dhd_pktid_map_t) + \
					(DHD_PKTID_ITEM_SZ * ((items) + 1)))

#define DHD_NATIVE_TO_PKTID_FINI_IOCTL(dhd, map)  dhd_pktid_map_fini_ioctl((dhd), (map))

#define DHD_NATIVE_TO_PKTID_RSV(dhd, map, pkt)    dhd_pktid_map_reserve((dhd), (map), (pkt))

#define DHD_NATIVE_TO_PKTID_SAVE(dhd, map, pkt, nkey, pa, len, dir, dmah, secdma, pkttype) \
	dhd_pktid_map_save((dhd), (map), (void *)(pkt), (nkey), (pa), (uint32)(len), \
	                   (uint8)(dir), (void *)(dmah), (void *)(secdma), \
			   (dhd_pkttype_t)(pkttype))

#define DHD_NATIVE_TO_PKTID(dhd, map, pkt, pa, len, dir, dmah, secdma, pkttype) \
	dhd_pktid_map_alloc((dhd), (map), (void *)(pkt), (pa), (uint32)(len), \
	                    (uint8)(dir), (void *)(dmah), (void *)(secdma), \
			    (dhd_pkttype_t)(pkttype))

#define DHD_PKTID_TO_NATIVE(dhd, map, pktid, pa, len, dmah, secdma, pkttype) \
	dhd_pktid_map_free((dhd), (map), (uint32)(pktid), \
	(dmaaddr_t *)&(pa), (uint32 *)&(len), (void **)&(dmah), \
	(void **) &secdma, (dhd_pkttype_t)(pkttype), DHD_PKTID_FREE_LOCKER)

#define DHD_PKTID_TO_NATIVE_RSV(dhd, map, pktid, pa, len, dmah, secdma, pkttype) \
	dhd_pktid_map_free((dhd), (map), (uint32)(pktid), \
	(dmaaddr_t *)&(pa), (uint32 *)&(len), (void **)&(dmah), \
	(void **) &secdma, (dhd_pkttype_t)(pkttype), DHD_PKTID_RSV_LOCKER)

#define DHD_PKTID_AVAIL(map)                 dhd_pktid_map_avail_cnt(map)

#if defined(DHD_PKTID_AUDIT_ENABLED)

static int dhd_pktid_audit(dhd_pub_t *dhd, dhd_pktid_map_t *pktid_map, uint32 pktid,
	const int test_for, const char *errmsg);

#ifdef DHD_DEBUG_PAGEALLOC
extern void dhd_pktid_audit_fail_cb(dhd_pub_t *dhdp);
#endif 

static int
dhd_pktid_audit(dhd_pub_t *dhd, dhd_pktid_map_t *pktid_map, uint32 pktid,
	const int test_for, const char *errmsg)
{
#define DHD_PKT_AUDIT_STR "ERROR: %16s Host PktId Audit: "

	const uint32 max_pktid_items = (MAX_PKTID_ITEMS);
	struct bcm_mwbmap *handle;
	uint32	flags;
	bool ignore_audit;

	if (pktid_map == (dhd_pktid_map_t *)NULL) {
		DHD_ERROR((DHD_PKT_AUDIT_STR "Pkt id map NULL\n", errmsg));
		return BCME_OK;
	}

	flags = DHD_PKTID_AUDIT_LOCK(pktid_map->pktid_audit_lock);

	handle = pktid_map->pktid_audit;
	if (handle == (struct bcm_mwbmap *)NULL) {
		DHD_ERROR((DHD_PKT_AUDIT_STR "Handle NULL\n", errmsg));
		DHD_PKTID_AUDIT_UNLOCK(pktid_map->pktid_audit_lock, flags);
		return BCME_OK;
	}

	
	ignore_audit = (pktid == DHD_IOCTL_REQ_PKTID) | (pktid == DHD_FAKE_PKTID);
	if (ignore_audit) {
		DHD_PKTID_AUDIT_UNLOCK(pktid_map->pktid_audit_lock, flags);
		return BCME_OK;
	}

	if ((pktid == DHD_PKTID_INVALID) || (pktid > max_pktid_items)) {
		DHD_ERROR((DHD_PKT_AUDIT_STR "PktId<%d> invalid\n", errmsg, pktid));
		
		goto error;
	}

	
	switch (test_for) {
		case DHD_DUPLICATE_ALLOC:
			if (!bcm_mwbmap_isfree(handle, pktid)) {
				DHD_ERROR((DHD_PKT_AUDIT_STR "PktId<%d> alloc duplicate\n",
				           errmsg, pktid));
				goto error;
			}
			bcm_mwbmap_force(handle, pktid);
			break;

		case DHD_DUPLICATE_FREE:
			if (bcm_mwbmap_isfree(handle, pktid)) {
				DHD_ERROR((DHD_PKT_AUDIT_STR "PktId<%d> free duplicate\n",
				           errmsg, pktid));
				goto error;
			}
			bcm_mwbmap_free(handle, pktid);
			break;

		case DHD_TEST_IS_ALLOC:
			if (bcm_mwbmap_isfree(handle, pktid)) {
				DHD_ERROR((DHD_PKT_AUDIT_STR "PktId<%d> is not allocated\n",
				           errmsg, pktid));
				goto error;
			}
			break;

		case DHD_TEST_IS_FREE:
			if (!bcm_mwbmap_isfree(handle, pktid)) {
				DHD_ERROR((DHD_PKT_AUDIT_STR "PktId<%d> is not free",
				           errmsg, pktid));
				goto error;
			}
			break;

		default:
			goto error;
	}

	DHD_PKTID_AUDIT_UNLOCK(pktid_map->pktid_audit_lock, flags);
	return BCME_OK;

error:

	DHD_PKTID_AUDIT_UNLOCK(pktid_map->pktid_audit_lock, flags);
	
#ifdef DHD_DEBUG_PAGEALLOC
	dhd_pktid_audit_fail_cb(dhd);
#else
	ASSERT(0);
#endif 
	return BCME_ERROR;
}

#define DHD_PKTID_AUDIT(dhdp, map, pktid, test_for) \
	dhd_pktid_audit((dhdp), (dhd_pktid_map_t *)(map), (pktid), (test_for), __FUNCTION__)

#endif 





static dhd_pktid_map_handle_t *
dhd_pktid_map_init(dhd_pub_t *dhd, uint32 num_items, uint32 index)
{
	void *osh;
	uint32 nkey;
	dhd_pktid_map_t *map;
	uint32 dhd_pktid_map_sz;
	uint32 map_items;
#ifdef DHD_USE_STATIC_PKTIDMAP
	uint32 section;
#endif 
	osh = dhd->osh;

	ASSERT((num_items >= 1) && (num_items <= MAX_PKTID_ITEMS));
	dhd_pktid_map_sz = DHD_PKTID_MAP_SZ(num_items);

#ifdef DHD_USE_STATIC_PKTIDMAP
	if (index == PKTID_MAP_HANDLE) {
		section = DHD_PREALLOC_PKTID_MAP;
	} else {
		section = DHD_PREALLOC_PKTID_MAP_IOCTL;
	}

	map = (dhd_pktid_map_t *)DHD_OS_PREALLOC(dhd, section, dhd_pktid_map_sz);
#else
	map = (dhd_pktid_map_t *)MALLOC(osh, dhd_pktid_map_sz);
#endif 

	if (map == NULL) {
		DHD_ERROR(("%s:%d: MALLOC failed for size %d\n",
			__FUNCTION__, __LINE__, dhd_pktid_map_sz));
		goto error;
	}

	bzero(map, dhd_pktid_map_sz);

	
	map->pktid_lock = DHD_PKTID_LOCK_INIT(osh);
	if (map->pktid_lock == NULL) {
		DHD_ERROR(("%s:%d: Lock init failed \r\n", __FUNCTION__, __LINE__));
		goto error;
	}

	map->items = num_items;
	map->avail = num_items;

	map_items = DHD_PKIDMAP_ITEMS(map->items);

#if defined(DHD_PKTID_AUDIT_ENABLED)
	
	map->pktid_audit = bcm_mwbmap_init(osh, map_items + 1);
	if (map->pktid_audit == (struct bcm_mwbmap *)NULL) {
		DHD_ERROR(("%s:%d: pktid_audit init failed\r\n", __FUNCTION__, __LINE__));
		goto error;
	} else {
		DHD_ERROR(("%s:%d: pktid_audit init succeeded %d\n",
			__FUNCTION__, __LINE__, map_items + 1));
	}

	map->pktid_audit_lock = DHD_PKTID_AUDIT_LOCK_INIT(osh);

#endif 

	for (nkey = 1; nkey <= map_items; nkey++) { 
		map->keys[nkey] = nkey; 
		map->lockers[nkey].state = LOCKER_IS_FREE;
		map->lockers[nkey].pkt   = NULL; 
		map->lockers[nkey].len   = 0;
	}

	
	map->lockers[DHD_PKTID_INVALID].state = LOCKER_IS_BUSY;
	map->lockers[DHD_PKTID_INVALID].pkt   = NULL; 
	map->lockers[DHD_PKTID_INVALID].len   = 0;

#if defined(DHD_PKTID_AUDIT_ENABLED)
	
	bcm_mwbmap_force(map->pktid_audit, DHD_PKTID_INVALID);
#endif 

	return (dhd_pktid_map_handle_t *)map; 

error:

	if (map) {

#if defined(DHD_PKTID_AUDIT_ENABLED)
		if (map->pktid_audit != (struct bcm_mwbmap *)NULL) {
			bcm_mwbmap_fini(osh, map->pktid_audit); 
			map->pktid_audit = (struct bcm_mwbmap *)NULL;
			if (map->pktid_audit_lock)
				DHD_PKTID_AUDIT_LOCK_DEINIT(osh, map->pktid_audit_lock);
		}
#endif 

		if (map->pktid_lock)
			DHD_PKTID_LOCK_DEINIT(osh, map->pktid_lock);

		MFREE(osh, map, dhd_pktid_map_sz);
	}

	return (dhd_pktid_map_handle_t *)NULL;
}


static void
dhd_pktid_map_fini(dhd_pub_t *dhd, dhd_pktid_map_handle_t *handle)
{
	void *osh;
	uint32 nkey;
	dhd_pktid_map_t *map;
	uint32 dhd_pktid_map_sz;
	dhd_pktid_item_t *locker;
	uint32 map_items;
	uint32 flags;

	if (handle == NULL) {
		return;
	}

	map = (dhd_pktid_map_t *)handle;
	flags =  DHD_PKTID_LOCK(map->pktid_lock);
	osh = dhd->osh;

	dhd_pktid_map_sz = DHD_PKTID_MAP_SZ(map->items);

	nkey = 1; 
	locker = &map->lockers[nkey];

	map_items = DHD_PKIDMAP_ITEMS(map->items);

	for (; nkey <= map_items; nkey++, locker++) {

		if (locker->state == LOCKER_IS_BUSY) { 

			locker->state = LOCKER_IS_FREE; 

#if defined(DHD_PKTID_AUDIT_ENABLED)
			DHD_PKTID_AUDIT(dhd, map, nkey, DHD_DUPLICATE_FREE); 
#endif 

			{   
				DMA_UNMAP(osh, locker->pa, locker->len,
					locker->dir, 0, DHD_DMAH_NULL);
				dhd_prot_packet_free(dhd, (ulong*)locker->pkt,
					locker->pkttype, TRUE);
			}
		}
#if defined(DHD_PKTID_AUDIT_ENABLED)
		else {
			DHD_PKTID_AUDIT(dhd, map, nkey, DHD_TEST_IS_FREE);
		}
#endif 

		locker->pkt = NULL; 
		locker->len = 0;
	}

#if defined(DHD_PKTID_AUDIT_ENABLED)
	if (map->pktid_audit != (struct bcm_mwbmap *)NULL) {
		bcm_mwbmap_fini(osh, map->pktid_audit); 
		map->pktid_audit = (struct bcm_mwbmap *)NULL;
		if (map->pktid_audit_lock) {
			DHD_PKTID_AUDIT_LOCK_DEINIT(osh, map->pktid_audit_lock);
		}
	}
#endif 

	DHD_PKTID_UNLOCK(map->pktid_lock, flags);
	DHD_PKTID_LOCK_DEINIT(osh, map->pktid_lock);

#ifdef DHD_USE_STATIC_PKTIDMAP
	DHD_OS_PREFREE(dhd, handle, dhd_pktid_map_sz);
#else
	MFREE(osh, handle, dhd_pktid_map_sz);
#endif 
}

#ifdef IOCTLRESP_USE_CONSTMEM
static void
dhd_pktid_map_fini_ioctl(dhd_pub_t *dhd, dhd_pktid_map_handle_t *handle)
{
	uint32 nkey;
	dhd_pktid_map_t *map;
	uint32 dhd_pktid_map_sz;
	dhd_pktid_item_t *locker;
	uint32 map_items;
	uint32 flags;
	osl_t *osh = dhd->osh;

	if (handle == NULL) {
		return;
	}

	map = (dhd_pktid_map_t *)handle;
	flags = DHD_PKTID_LOCK(map->pktid_lock);

	dhd_pktid_map_sz = DHD_PKTID_MAP_SZ(map->items);

	nkey = 1; 
	locker = &map->lockers[nkey];

	map_items = DHD_PKIDMAP_ITEMS(map->items);

	for (; nkey <= map_items; nkey++, locker++) {

		if (locker->state == LOCKER_IS_BUSY) { 

			locker->state = LOCKER_IS_FREE; 

#if defined(DHD_PKTID_AUDIT_ENABLED)
			DHD_PKTID_AUDIT(dhd, map, nkey, DHD_DUPLICATE_FREE); 
#endif 

			{
				dhd_dma_buf_t retbuf;
				retbuf.va = locker->pkt;
				retbuf.len = locker->len;
				retbuf.pa = locker->pa;
				retbuf.dmah = locker->dmah;
				retbuf.secdma = locker->secdma;

				
				DHD_PKTID_UNLOCK(map->pktid_lock, flags);
				free_ioctl_return_buffer(dhd, &retbuf);
				flags = DHD_PKTID_LOCK(map->pktid_lock);
			}
		}
#if defined(DHD_PKTID_AUDIT_ENABLED)
		else {
			DHD_PKTID_AUDIT(dhd, map, nkey, DHD_TEST_IS_FREE);
		}
#endif 

		locker->pkt = NULL; 
		locker->len = 0;
	}

#if defined(DHD_PKTID_AUDIT_ENABLED)
	if (map->pktid_audit != (struct bcm_mwbmap *)NULL) {
		bcm_mwbmap_fini(osh, map->pktid_audit); 
		map->pktid_audit = (struct bcm_mwbmap *)NULL;
		if (map->pktid_audit_lock) {
			DHD_PKTID_AUDIT_LOCK_DEINIT(osh, map->pktid_audit_lock);
		}
	}
#endif 

	DHD_PKTID_UNLOCK(map->pktid_lock, flags);
	DHD_PKTID_LOCK_DEINIT(osh, map->pktid_lock);

#ifdef DHD_USE_STATIC_PKTIDMAP
	DHD_OS_PREFREE(dhd, handle, dhd_pktid_map_sz);
#else
	MFREE(osh, handle, dhd_pktid_map_sz);
#endif 
}
#endif 

static INLINE uint32 BCMFASTPATH
dhd_pktid_map_avail_cnt(dhd_pktid_map_handle_t *handle)
{
	dhd_pktid_map_t *map;
	uint32	flags;
	uint32	avail;

	ASSERT(handle != NULL);
	map = (dhd_pktid_map_t *)handle;

	flags = DHD_PKTID_LOCK(map->pktid_lock);
	avail = map->avail;
	DHD_PKTID_UNLOCK(map->pktid_lock, flags);

	return avail;
}


static INLINE uint32
__dhd_pktid_map_reserve(dhd_pub_t *dhd, dhd_pktid_map_handle_t *handle, void *pkt)
{
	uint32 nkey;
	dhd_pktid_map_t *map;
	dhd_pktid_item_t *locker;

	ASSERT(handle != NULL);
	map = (dhd_pktid_map_t *)handle;

	if (map->avail <= 0) { 
		map->failures++;
		DHD_ERROR(("%s:%d: failed, no free keys\n", __FUNCTION__, __LINE__));
		return DHD_PKTID_INVALID; 
	}

	ASSERT(map->avail <= map->items);
	nkey = map->keys[map->avail]; 
#ifdef CUSTOMER_HW_ONE
	if (nkey > DHD_PKIDMAP_ITEMS(map->items)) {
		DHD_ERROR(("%s:%d: invalid nkey (%d)\n", __FUNCTION__, __LINE__, nkey));
		return DHD_PKTID_INVALID; 
	}
#endif 
	locker = &map->lockers[nkey]; 
	map->avail--;
	locker->pkt = pkt; 
	locker->len = 0;
	locker->state = LOCKER_IS_BUSY; 

#if defined(DHD_PKTID_AUDIT_MAP)
	DHD_PKTID_AUDIT(dhd, map, nkey, DHD_DUPLICATE_ALLOC); 
#endif 

	ASSERT(nkey != DHD_PKTID_INVALID);
	return nkey; 
}


static INLINE uint32
dhd_pktid_map_reserve(dhd_pub_t *dhd, dhd_pktid_map_handle_t *handle, void *pkt)
{
	dhd_pktid_map_t *map;
	uint32 flags;
	uint32 ret;

	ASSERT(handle != NULL);
	map = (dhd_pktid_map_t *)handle;
	flags = DHD_PKTID_LOCK(map->pktid_lock);
	ret = __dhd_pktid_map_reserve(dhd, handle, pkt);
	DHD_PKTID_UNLOCK(map->pktid_lock, flags);

	return ret;
}

static INLINE void
__dhd_pktid_map_save(dhd_pub_t *dhd, dhd_pktid_map_handle_t *handle, void *pkt,
	uint32 nkey, dmaaddr_t pa, uint32 len, uint8 dir, void *dmah, void *secdma,
	dhd_pkttype_t pkttype)
{
	dhd_pktid_map_t *map;
	dhd_pktid_item_t *locker;

	ASSERT(handle != NULL);
	map = (dhd_pktid_map_t *)handle;

	ASSERT((nkey != DHD_PKTID_INVALID) && (nkey <= DHD_PKIDMAP_ITEMS(map->items)));

	locker = &map->lockers[nkey];

	ASSERT(((locker->state == LOCKER_IS_BUSY) && (locker->pkt == pkt)) ||
		((locker->state == LOCKER_IS_RSVD) && (locker->pkt == NULL)));

#if defined(DHD_PKTID_AUDIT_MAP)
	DHD_PKTID_AUDIT(dhd, map, nkey, DHD_TEST_IS_ALLOC); 
#endif 

	
	locker->dir = dir;
	locker->pa = pa;
	locker->len = (uint16)len; 
	locker->dmah = dmah; 
	locker->secdma = secdma;
	locker->pkttype = pkttype;
	locker->pkt = pkt;
	locker->state = LOCKER_IS_BUSY; 
}

static INLINE void
dhd_pktid_map_save(dhd_pub_t *dhd, dhd_pktid_map_handle_t *handle, void *pkt,
	uint32 nkey, dmaaddr_t pa, uint32 len, uint8 dir, void *dmah, void *secdma,
	dhd_pkttype_t pkttype)
{
	dhd_pktid_map_t *map;
	uint32 flags;

	ASSERT(handle != NULL);
	map = (dhd_pktid_map_t *)handle;
	flags = DHD_PKTID_LOCK(map->pktid_lock);
	__dhd_pktid_map_save(dhd, handle, pkt, nkey, pa, len,
		dir, dmah, secdma, pkttype);
	DHD_PKTID_UNLOCK(map->pktid_lock, flags);
}

static uint32 BCMFASTPATH
dhd_pktid_map_alloc(dhd_pub_t *dhd, dhd_pktid_map_handle_t *handle, void *pkt,
	dmaaddr_t pa, uint32 len, uint8 dir, void *dmah, void *secdma,
	dhd_pkttype_t pkttype)
{
	uint32 nkey;
	uint32 flags;
	dhd_pktid_map_t *map;

	ASSERT(handle != NULL);
	map = (dhd_pktid_map_t *)handle;

	flags = DHD_PKTID_LOCK(map->pktid_lock);

	nkey = __dhd_pktid_map_reserve(dhd, handle, pkt);
	if (nkey != DHD_PKTID_INVALID) {
		__dhd_pktid_map_save(dhd, handle, pkt, nkey, pa,
			len, dir, dmah, secdma, pkttype);
#if defined(DHD_PKTID_AUDIT_MAP)
		DHD_PKTID_AUDIT(dhd, map, nkey, DHD_TEST_IS_ALLOC); 
#endif 
	}

	DHD_PKTID_UNLOCK(map->pktid_lock, flags);

	return nkey;
}

static void * BCMFASTPATH
dhd_pktid_map_free(dhd_pub_t *dhd, dhd_pktid_map_handle_t *handle, uint32 nkey,
	dmaaddr_t *pa, uint32 *len, void **dmah, void **secdma,
	dhd_pkttype_t pkttype, bool rsv_locker)
{
	dhd_pktid_map_t *map;
	dhd_pktid_item_t *locker;
	void * pkt;
	uint32 flags;

	ASSERT(handle != NULL);

	map = (dhd_pktid_map_t *)handle;

	flags = DHD_PKTID_LOCK(map->pktid_lock);

	ASSERT((nkey != DHD_PKTID_INVALID) && (nkey <= DHD_PKIDMAP_ITEMS(map->items)));
#ifdef CUSTOMER_HW_ONE
	if ((nkey == DHD_PKTID_INVALID) || (nkey > DHD_PKIDMAP_ITEMS(map->items))) {
		DHD_ERROR(("%s:%d: Error! freeing out of range pktid<%u>\n",
			__FUNCTION__, __LINE__, nkey));
		DHD_PKTID_UNLOCK(map->pktid_lock, flags);
		return NULL;
	}
#endif 

	locker = &map->lockers[nkey];

#if defined(DHD_PKTID_AUDIT_MAP)
	DHD_PKTID_AUDIT(dhd, map, nkey, DHD_DUPLICATE_FREE); 
#endif 

	if (locker->state == LOCKER_IS_FREE) { 
		DHD_ERROR(("%s:%d: Error! freeing invalid pktid<%u>\n",
			__FUNCTION__, __LINE__, nkey));
		ASSERT(locker->state != LOCKER_IS_FREE);

		DHD_PKTID_UNLOCK(map->pktid_lock, flags);
		return NULL;
	}

	if ((pkttype != PKTTYPE_NO_CHECK) && (locker->pkttype != pkttype)) {

		DHD_PKTID_UNLOCK(map->pktid_lock, flags);

		DHD_ERROR(("%s:%d: Error! Invalid Buffer Free for pktid<%u> \n",
			__FUNCTION__, __LINE__, nkey));
		ASSERT(locker->pkttype == pkttype);

		return NULL;
	}

	if (rsv_locker == DHD_PKTID_FREE_LOCKER) {
		map->avail++;
		map->keys[map->avail] = nkey; 
		locker->state = LOCKER_IS_FREE; 
	} else {
		
		locker->state = LOCKER_IS_RSVD;
	}

#if defined(DHD_PKTID_AUDIT_MAP)
	DHD_PKTID_AUDIT(dhd, map, nkey, DHD_TEST_IS_FREE);
#endif 

	*pa = locker->pa; 
	*len = (uint32)locker->len;
	*dmah = locker->dmah;
	*secdma = locker->secdma;

	pkt = locker->pkt;
	locker->pkt = NULL; 
	locker->len = 0;

	DHD_PKTID_UNLOCK(map->pktid_lock, flags);
	return pkt;
}

#else 


typedef struct pktlist {
	PKT_LIST *tx_pkt_list;		
	PKT_LIST *rx_pkt_list;		
	PKT_LIST *ctrl_pkt_list;	
} pktlists_t;

#define DHD_PKTID32(pktptr32)	((uint32)(pktptr32))
#define DHD_PKTPTR32(pktid32)	((void *)(pktid32))


static INLINE uint32 dhd_native_to_pktid(dhd_pktid_map_handle_t *map, void *pktptr32,
	dmaaddr_t pa, uint32 dma_len, void *dmah, void *secdma,
	dhd_pkttype_t pkttype);
static INLINE void * dhd_pktid_to_native(dhd_pktid_map_handle_t *map, uint32 pktid32,
	dmaaddr_t *pa, uint32 *dma_len, void **dmah, void **secdma,
	dhd_pkttype_t pkttype);

static dhd_pktid_map_handle_t *
dhd_pktid_map_init(dhd_pub_t *dhd, uint32 num_items, uint32 index)
{
	osl_t *osh = dhd->osh;
	pktlists_t *handle = NULL;

	if ((handle = (pktlists_t *) MALLOCZ(osh, sizeof(pktlists_t))) == NULL) {
		DHD_ERROR(("%s:%d: MALLOC failed for lists allocation, size=%d\n",
		           __FUNCTION__, __LINE__, sizeof(pktlists_t)));
		goto error_done;
	}

	if ((handle->tx_pkt_list = (PKT_LIST *) MALLOC(osh, sizeof(PKT_LIST))) == NULL) {
		DHD_ERROR(("%s:%d: MALLOC failed for list allocation, size=%d\n",
		           __FUNCTION__, __LINE__, sizeof(PKT_LIST)));
		goto error;
	}

	if ((handle->rx_pkt_list = (PKT_LIST *) MALLOC(osh, sizeof(PKT_LIST))) == NULL) {
		DHD_ERROR(("%s:%d: MALLOC failed for list allocation, size=%d\n",
		           __FUNCTION__, __LINE__, sizeof(PKT_LIST)));
		goto error;
	}

	if ((handle->ctrl_pkt_list = (PKT_LIST *) MALLOC(osh, sizeof(PKT_LIST))) == NULL) {
		DHD_ERROR(("%s:%d: MALLOC failed for list allocation, size=%d\n",
		           __FUNCTION__, __LINE__, sizeof(PKT_LIST)));
		goto error;
	}

	PKTLIST_INIT(handle->tx_pkt_list);
	PKTLIST_INIT(handle->rx_pkt_list);
	PKTLIST_INIT(handle->ctrl_pkt_list);

	return (dhd_pktid_map_handle_t *) handle;

error:
	if (handle->ctrl_pkt_list) {
		MFREE(osh, handle->ctrl_pkt_list, sizeof(PKT_LIST));
	}

	if (handle->rx_pkt_list) {
		MFREE(osh, handle->rx_pkt_list, sizeof(PKT_LIST));
	}

	if (handle->tx_pkt_list) {
		MFREE(osh, handle->tx_pkt_list, sizeof(PKT_LIST));
	}

	if (handle) {
		MFREE(osh, handle, sizeof(pktlists_t));
	}

error_done:
	return (dhd_pktid_map_handle_t *)NULL;
}

static void
dhd_pktid_map_fini(dhd_pub_t *dhd, dhd_pktid_map_handle_t *map)
{
	osl_t *osh = dhd->osh;
	pktlists_t *handle = (pktlists_t *) map;

	ASSERT(handle != NULL);
	if (handle == (pktlists_t *)NULL) {
		return;
	}

	if (handle->ctrl_pkt_list) {
		PKTLIST_FINI(handle->ctrl_pkt_list);
		MFREE(osh, handle->ctrl_pkt_list, sizeof(PKT_LIST));
	}

	if (handle->rx_pkt_list) {
		PKTLIST_FINI(handle->rx_pkt_list);
		MFREE(osh, handle->rx_pkt_list, sizeof(PKT_LIST));
	}

	if (handle->tx_pkt_list) {
		PKTLIST_FINI(handle->tx_pkt_list);
		MFREE(osh, handle->tx_pkt_list, sizeof(PKT_LIST));
	}

	if (handle) {
		MFREE(osh, handle, sizeof(pktlists_t));
	}
}

static INLINE uint32
dhd_native_to_pktid(dhd_pktid_map_handle_t *map, void *pktptr32,
	dmaaddr_t pa, uint32 dma_len, void *dmah, void *secdma,
	dhd_pkttype_t pkttype)
{
	pktlists_t *handle = (pktlists_t *) map;
	ASSERT(pktptr32 != NULL);
	DHD_PKT_SET_DMA_LEN(pktptr32, dma_len);
	DHD_PKT_SET_DMAH(pktptr32, dmah);
	DHD_PKT_SET_PA(pktptr32, pa);
	DHD_PKT_SET_SECDMA(pktptr32, secdma);

	if (pkttype == PKTTYPE_DATA_TX) {
		PKTLIST_ENQ(handle->tx_pkt_list,  pktptr32);
	} else if (pkttype == PKTTYPE_DATA_RX) {
		PKTLIST_ENQ(handle->rx_pkt_list,  pktptr32);
	} else {
		PKTLIST_ENQ(handle->ctrl_pkt_list,  pktptr32);
	}

	return DHD_PKTID32(pktptr32);
}

static INLINE void *
dhd_pktid_to_native(dhd_pktid_map_handle_t *map, uint32 pktid32,
	dmaaddr_t *pa, uint32 *dma_len, void **dmah, void **secdma,
	dhd_pkttype_t pkttype)
{
	pktlists_t *handle = (pktlists_t *) map;
	void *pktptr32;

	ASSERT(pktid32 != 0U);
	pktptr32 = DHD_PKTPTR32(pktid32);
	*dma_len = DHD_PKT_GET_DMA_LEN(pktptr32);
	*dmah = DHD_PKT_GET_DMAH(pktptr32);
	*pa = DHD_PKT_GET_PA(pktptr32);
	*secdma = DHD_PKT_GET_SECDMA(pktptr32);

	if (pkttype == PKTTYPE_DATA_TX) {
		PKTLIST_UNLINK(handle->tx_pkt_list,  pktptr32);
	} else if (pkttype == PKTTYPE_DATA_RX) {
		PKTLIST_UNLINK(handle->rx_pkt_list,  pktptr32);
	} else {
		PKTLIST_UNLINK(handle->ctrl_pkt_list,  pktptr32);
	}

	return pktptr32;
}

#define DHD_NATIVE_TO_PKTID_RSV(dhd, map, pkt)  DHD_PKTID32(pkt)

#define DHD_NATIVE_TO_PKTID_SAVE(dhd, map, pkt, nkey, pa, len, dma_dir, dmah, secdma, pkttype) \
	({ BCM_REFERENCE(dhd); BCM_REFERENCE(nkey); BCM_REFERENCE(dma_dir); \
	   dhd_native_to_pktid((dhd_pktid_map_handle_t *) map, (pkt), (pa), (len), \
			   (dmah), (secdma), (dhd_pkttype_t)(pkttype)); \
	})

#define DHD_NATIVE_TO_PKTID(dhd, map, pkt, pa, len, dma_dir, dmah, secdma, pkttype) \
	({ BCM_REFERENCE(dhd); BCM_REFERENCE(dma_dir); \
	   dhd_native_to_pktid((dhd_pktid_map_handle_t *) map, (pkt), (pa), (len), \
			   (dmah), (secdma), (dhd_pkttype_t)(pkttype)); \
	})

#define DHD_PKTID_TO_NATIVE(dhd, map, pktid, pa, len, dmah, secdma, pkttype) \
	({ BCM_REFERENCE(dhd); BCM_REFERENCE(pkttype);	\
		dhd_pktid_to_native((dhd_pktid_map_handle_t *) map, (uint32)(pktid), \
				(dmaaddr_t *)&(pa), (uint32 *)&(len), (void **)&(dmah), \
				(void **)&secdma, (dhd_pkttype_t)(pkttype)); \
	})

#define DHD_PKTID_AVAIL(map)  (~0)

#endif 



int
dhd_prot_attach(dhd_pub_t *dhd)
{
	osl_t *osh = dhd->osh;
	dhd_prot_t *prot;

	
	if (!(prot = (dhd_prot_t *)DHD_OS_PREALLOC(dhd, DHD_PREALLOC_PROT,
		sizeof(dhd_prot_t)))) {
		DHD_ERROR(("%s: kmalloc failed\n", __FUNCTION__));
		goto fail;
	}
	memset(prot, 0, sizeof(*prot));

	prot->osh = osh;
	dhd->prot = prot;

	
	dhd->dma_d2h_ring_upd_support = FALSE;
	dhd->dma_h2d_ring_upd_support = FALSE;

	

	
	if (dhd_prot_ring_attach(dhd, &prot->h2dring_ctrl_subn, "h2dctrl",
	        H2DRING_CTRL_SUB_MAX_ITEM, H2DRING_CTRL_SUB_ITEMSIZE,
	        BCMPCIE_H2D_MSGRING_CONTROL_SUBMIT) != BCME_OK) {
		DHD_ERROR(("%s: dhd_prot_ring_attach H2D Ctrl Submission failed\n",
			__FUNCTION__));
		goto fail;
	}

	
	if (dhd_prot_ring_attach(dhd, &prot->h2dring_rxp_subn, "h2drxp",
	        H2DRING_RXPOST_MAX_ITEM, H2DRING_RXPOST_ITEMSIZE,
	        BCMPCIE_H2D_MSGRING_RXPOST_SUBMIT) != BCME_OK) {
		DHD_ERROR(("%s: dhd_prot_ring_attach H2D RxPost failed\n",
			__FUNCTION__));
		goto fail;
	}

	
	if (dhd_prot_ring_attach(dhd, &prot->d2hring_ctrl_cpln, "d2hctrl",
	        D2HRING_CTRL_CMPLT_MAX_ITEM, D2HRING_CTRL_CMPLT_ITEMSIZE,
	        BCMPCIE_D2H_MSGRING_CONTROL_COMPLETE) != BCME_OK) {
		DHD_ERROR(("%s: dhd_prot_ring_attach D2H Ctrl Completion failed\n",
			__FUNCTION__));
		goto fail;
	}

	
	if (dhd_prot_ring_attach(dhd, &prot->d2hring_tx_cpln, "d2htxcpl",
	        D2HRING_TXCMPLT_MAX_ITEM, D2HRING_TXCMPLT_ITEMSIZE,
	        BCMPCIE_D2H_MSGRING_TX_COMPLETE) != BCME_OK) {
		DHD_ERROR(("%s: dhd_prot_ring_attach D2H Tx Completion failed\n",
			__FUNCTION__));
		goto fail;

	}

	
	if (dhd_prot_ring_attach(dhd, &prot->d2hring_rx_cpln, "d2hrxcpl",
	        D2HRING_RXCMPLT_MAX_ITEM, D2HRING_RXCMPLT_ITEMSIZE,
	        BCMPCIE_D2H_MSGRING_RX_COMPLETE) != BCME_OK) {
		DHD_ERROR(("%s: dhd_prot_ring_attach D2H Rx Completion failed\n",
			__FUNCTION__));
		goto fail;

	}

	
	if (dhd_dma_buf_alloc(dhd, &prot->retbuf, IOCT_RETBUF_SIZE)) {
		goto fail;
	}

	
	if (dhd_dma_buf_alloc(dhd, &prot->ioctbuf, IOCT_RETBUF_SIZE)) {
		goto fail;
	}

	
#ifdef BCM_HOST_BUF
	if (dhd_dma_buf_alloc(dhd, &prot->d2h_dma_scratch_buf,
		ROUNDUP(DMA_D2H_SCRATCH_BUF_LEN, 16) + DMA_HOST_BUFFER_LEN)) {
#else
	if (dhd_dma_buf_alloc(dhd, &prot->d2h_dma_scratch_buf, DMA_D2H_SCRATCH_BUF_LEN)) {

#endif 
		goto fail;
	}

	
	if (dhd_dma_buf_alloc(dhd, &prot->host_bus_throughput_buf, DHD_BUS_TPUT_BUF_LEN)) {
		goto fail;
	}

#ifdef DHD_RX_CHAINING
	dhd_rxchain_reset(&prot->rxchain);
#endif

#if defined(DHD_LB)

	   
#if defined(DHD_LB_TXC)
	{
		void *buffer;
		buffer = MALLOC(dhd->osh, sizeof(void*) * DHD_LB_WORKQ_SZ);
		bcm_workq_init(&prot->tx_compl_prod, &prot->tx_compl_cons,
			buffer, DHD_LB_WORKQ_SZ);
		prot->tx_compl_prod_sync = 0;
		DHD_INFO(("%s: created tx_compl_workq <%p,%d>\n",
			__FUNCTION__, buffer, DHD_LB_WORKQ_SZ));
	}
#endif 

#if defined(DHD_LB_RXC)
	{
		void *buffer;
		buffer = MALLOC(dhd->osh, sizeof(uint32) * DHD_LB_WORKQ_SZ);
		bcm_workq_init(&prot->rx_compl_prod, &prot->rx_compl_cons,
			buffer, DHD_LB_WORKQ_SZ);
		prot->rx_compl_prod_sync = 0;
		DHD_INFO(("%s: created rx_compl_workq <%p,%d>\n",
			__FUNCTION__, buffer, DHD_LB_WORKQ_SZ));
	}
#endif 

#endif 

	return BCME_OK;

fail:

#ifndef CONFIG_DHD_USE_STATIC_BUF
	if (prot != NULL) {
		dhd_prot_detach(dhd);
	}
#endif 

	return BCME_NOMEM;
} 


int
dhd_prot_init(dhd_pub_t *dhd)
{
	sh_addr_t base_addr;
	dhd_prot_t *prot = dhd->prot;

	
	if (prot->pktid_map_handle != NULL) {
		DHD_ERROR(("%s: pktid_map_handle already set!\n", __FUNCTION__));
		ASSERT(0);
		return BCME_ERROR;
	}

#ifdef IOCTLRESP_USE_CONSTMEM
	if (prot->pktid_map_handle_ioctl != NULL) {
		DHD_ERROR(("%s: pktid_map_handle_ioctl already set!\n", __FUNCTION__));
		ASSERT(0);
		return BCME_ERROR;
	}
#endif 

	prot->pktid_map_handle = DHD_NATIVE_TO_PKTID_INIT(dhd, MAX_PKTID_ITEMS, PKTID_MAP_HANDLE);
	if (prot->pktid_map_handle == NULL) {
		DHD_ERROR(("%s: Unable to map packet id's\n", __FUNCTION__));
		ASSERT(0);
		return BCME_NOMEM;
	}

#ifdef IOCTLRESP_USE_CONSTMEM
	prot->pktid_map_handle_ioctl = DHD_NATIVE_TO_PKTID_INIT(dhd,
		DHD_FLOWRING_MAX_IOCTLRESPBUF_POST, PKTID_MAP_HANDLE_IOCTL);
	if (prot->pktid_map_handle_ioctl == NULL) {
		DHD_ERROR(("%s: Unable to map ioctl response buffers\n", __FUNCTION__));
		ASSERT(0);
		return BCME_NOMEM;
	}
#endif 

	
	prot->max_tx_count = H2DRING_TXPOST_MAX_ITEM;

	DHD_INFO(("%s:%d: MAX_TX_COUNT = %d\n", __FUNCTION__, __LINE__, prot->max_tx_count));

	
	dhd_bus_cmn_readshared(dhd->bus, &prot->max_rxbufpost, MAX_HOST_RXBUFS, 0);
	if (prot->max_rxbufpost == 0) {
		
		
		prot->max_rxbufpost = DEFAULT_RX_BUFFERS_TO_POST;
	}
	DHD_INFO(("%s:%d: MAX_RXBUFPOST = %d\n", __FUNCTION__, __LINE__, prot->max_rxbufpost));

	
	prot->max_eventbufpost = DHD_FLOWRING_MAX_EVENTBUF_POST;
	prot->max_ioctlrespbufpost = DHD_FLOWRING_MAX_IOCTLRESPBUF_POST;

	prot->cur_ioctlresp_bufs_posted = 0;
	prot->active_tx_count = 0;
	prot->data_seq_no = 0;
	prot->ioctl_seq_no = 0;
	prot->rxbufpost = 0;
	prot->cur_event_bufs_posted = 0;
	prot->ioctl_state = 0;
	prot->ioctl_received = 0;
	prot->ioctl_received = IOCTL_WAIT;

	prot->dmaxfer.srcmem.va = NULL;
	prot->dmaxfer.dstmem.va = NULL;
	prot->dmaxfer.in_progress = FALSE;

	prot->metadata_dbg = FALSE;
	prot->rx_metadata_offset = 0;
	prot->tx_metadata_offset = 0;
	prot->txp_threshold = TXP_FLUSH_MAX_ITEMS_FLUSH_CNT;

	prot->ioctl_trans_id = 0;
#ifdef DHD_TRACE_WAKE_LOCK
	prot->wake_lock_dbg_time = 0;
#endif

	
	
	prot->mb_ring_fn = dhd_bus_get_mbintr_fn(dhd->bus);

	

	dhd_prot_ring_init(dhd, &prot->h2dring_ctrl_subn);
	dhd_prot_ring_init(dhd, &prot->h2dring_rxp_subn);
	dhd_prot_ring_init(dhd, &prot->d2hring_ctrl_cpln);
	dhd_prot_ring_init(dhd, &prot->d2hring_tx_cpln);
	dhd_prot_ring_init(dhd, &prot->d2hring_rx_cpln);

#if defined(PCIE_D2H_SYNC)
	dhd_prot_d2h_sync_init(dhd);
#endif 

	dhd_prot_h2d_sync_init(dhd);

	
	dhd_base_addr_htolpa(&base_addr, prot->d2h_dma_scratch_buf.pa);
	dhd_bus_cmn_writeshared(dhd->bus, &base_addr, sizeof(base_addr),
		D2H_DMA_SCRATCH_BUF, 0);
	dhd_bus_cmn_writeshared(dhd->bus, &prot->d2h_dma_scratch_buf.len,
		sizeof(prot->d2h_dma_scratch_buf.len), D2H_DMA_SCRATCH_BUF_LEN, 0);

	if (DMA_INDX_ENAB(dhd->dma_d2h_ring_upd_support)) {
		dhd_base_addr_htolpa(&base_addr, prot->d2h_dma_indx_wr_buf.pa);
		dhd_bus_cmn_writeshared(dhd->bus, &base_addr, sizeof(base_addr),
			D2H_DMA_INDX_WR_BUF, 0);
		dhd_base_addr_htolpa(&base_addr, prot->h2d_dma_indx_rd_buf.pa);
		dhd_bus_cmn_writeshared(dhd->bus, &base_addr, sizeof(base_addr),
			H2D_DMA_INDX_RD_BUF, 0);
	}

	if (DMA_INDX_ENAB(dhd->dma_h2d_ring_upd_support)) {
		dhd_base_addr_htolpa(&base_addr, prot->h2d_dma_indx_wr_buf.pa);
		dhd_bus_cmn_writeshared(dhd->bus, &base_addr, sizeof(base_addr),
			H2D_DMA_INDX_WR_BUF, 0);
		dhd_base_addr_htolpa(&base_addr, prot->d2h_dma_indx_rd_buf.pa);
		dhd_bus_cmn_writeshared(dhd->bus, &base_addr, sizeof(base_addr),
			D2H_DMA_INDX_RD_BUF, 0);
	}


	
	if (dhd_prot_flowrings_pool_attach(dhd) != BCME_OK) {
		return BCME_ERROR;
	}

	

	
	dhd_msgbuf_ring_config_d2h_soft_doorbell(dhd);

	
	dhd_msgbuf_rxbuf_post(dhd, FALSE); 
	dhd_msgbuf_rxbuf_post_ioctlresp_bufs(dhd);
	dhd_msgbuf_rxbuf_post_event_bufs(dhd);

	return BCME_OK;
} 


void
dhd_prot_detach(dhd_pub_t *dhd)
{
	dhd_prot_t *prot = dhd->prot;

	
	if (prot) {

		

		dhd_dma_buf_free(dhd, &prot->d2h_dma_scratch_buf);
		dhd_dma_buf_free(dhd, &prot->retbuf); 
		dhd_dma_buf_free(dhd, &prot->ioctbuf);
		dhd_dma_buf_free(dhd, &prot->host_bus_throughput_buf);

		
		dhd_dma_buf_free(dhd, &prot->h2d_dma_indx_wr_buf);
		dhd_dma_buf_free(dhd, &prot->h2d_dma_indx_rd_buf);
		dhd_dma_buf_free(dhd, &prot->d2h_dma_indx_wr_buf);
		dhd_dma_buf_free(dhd, &prot->d2h_dma_indx_rd_buf);

		
		dhd_prot_ring_detach(dhd, &prot->h2dring_ctrl_subn);
		dhd_prot_ring_detach(dhd, &prot->h2dring_rxp_subn);
		dhd_prot_ring_detach(dhd, &prot->d2hring_ctrl_cpln);
		dhd_prot_ring_detach(dhd, &prot->d2hring_tx_cpln);
		dhd_prot_ring_detach(dhd, &prot->d2hring_rx_cpln);

		
		dhd_prot_flowrings_pool_detach(dhd);

		DHD_NATIVE_TO_PKTID_FINI(dhd, dhd->prot->pktid_map_handle);

#ifndef CONFIG_DHD_USE_STATIC_BUF
		MFREE(dhd->osh, dhd->prot, sizeof(dhd_prot_t));
#endif 

#if defined(DHD_LB)
#if defined(DHD_LB_TXC)
		if (prot->tx_compl_prod.buffer) {
			MFREE(dhd->osh, prot->tx_compl_prod.buffer,
				sizeof(void*) * DHD_LB_WORKQ_SZ);
		}
#endif 
#if defined(DHD_LB_RXC)
		if (prot->rx_compl_prod.buffer) {
			MFREE(dhd->osh, prot->rx_compl_prod.buffer,
				sizeof(void*) * DHD_LB_WORKQ_SZ);
		}
#endif 
#endif 

		dhd->prot = NULL;
	}
} 


void
dhd_prot_reset(dhd_pub_t *dhd)
{
	struct dhd_prot *prot = dhd->prot;

	DHD_TRACE(("%s\n", __FUNCTION__));

	if (prot == NULL) {
		return;
	}

	dhd_prot_flowrings_pool_reset(dhd);

	dhd_prot_ring_reset(dhd, &prot->h2dring_ctrl_subn);
	dhd_prot_ring_reset(dhd, &prot->h2dring_rxp_subn);
	dhd_prot_ring_reset(dhd, &prot->d2hring_ctrl_cpln);
	dhd_prot_ring_reset(dhd, &prot->d2hring_tx_cpln);
	dhd_prot_ring_reset(dhd, &prot->d2hring_rx_cpln);

	dhd_dma_buf_reset(dhd, &prot->retbuf);
	dhd_dma_buf_reset(dhd, &prot->ioctbuf);
	dhd_dma_buf_reset(dhd, &prot->d2h_dma_scratch_buf);
	dhd_dma_buf_reset(dhd, &prot->h2d_dma_indx_rd_buf);
	dhd_dma_buf_reset(dhd, &prot->h2d_dma_indx_wr_buf);
	dhd_dma_buf_reset(dhd, &prot->d2h_dma_indx_rd_buf);
	dhd_dma_buf_reset(dhd, &prot->d2h_dma_indx_wr_buf);


	prot->rx_metadata_offset = 0;
	prot->tx_metadata_offset = 0;

	prot->rxbufpost = 0;
	prot->cur_event_bufs_posted = 0;
	prot->cur_ioctlresp_bufs_posted = 0;
#ifdef TX_STUCK_DETECTION
	printf("%s active_tx_count %d\n", __FUNCTION__, prot->active_tx_count);
#endif 
	prot->active_tx_count = 0;
	prot->data_seq_no = 0;
	prot->ioctl_seq_no = 0;
	prot->ioctl_state = 0;
	prot->ioctl_received = IOCTL_WAIT;
	prot->ioctl_trans_id = 0;
#ifdef DHD_TRACE_WAKE_LOCK
	prot->wake_lock_dbg_time = 0;
#endif

	if (dhd->flow_rings_inited) {
		dhd_flow_rings_deinit(dhd);
	}

	if (prot->pktid_map_handle) {
		DHD_NATIVE_TO_PKTID_FINI(dhd, prot->pktid_map_handle);
		prot->pktid_map_handle = NULL;
	}

#ifdef IOCTLRESP_USE_CONSTMEM
	if (prot->pktid_map_handle_ioctl) {
		DHD_NATIVE_TO_PKTID_FINI_IOCTL(dhd, prot->pktid_map_handle_ioctl);
		prot->pktid_map_handle_ioctl = NULL;
	}
#endif 

#if defined(CONFIG_DHD_USE_STATIC_BUF)
	PKTFREE_ALL_STATIC(dhd->osh);
#endif 
} 

#ifdef DHD_TRACE_WAKE_LOCK
int
dhd_prot_wake_lock_dbg_print(dhd_pub_t *dhd)
{
	dhd_prot_t *prot = dhd->prot;
	if (prot->wake_lock_dbg_time &&
		(OSL_SYSUPTIME() - prot->wake_lock_dbg_time > TIMEOUT_WAKE_LOCK_DBG_PRINT)) {
		prot->wake_lock_dbg_time = OSL_SYSUPTIME();
		return 1;
	}
	if(!prot->wake_lock_dbg_time)
		prot->wake_lock_dbg_time = OSL_SYSUPTIME();
	return 0;
}
#endif 

void
dhd_prot_rx_dataoffset(dhd_pub_t *dhd, uint32 rx_offset)
{
	dhd_prot_t *prot = dhd->prot;
	prot->rx_dataoffset = rx_offset;
}

int
dhd_sync_with_dongle(dhd_pub_t *dhd)
{
	int ret = 0;
	wlc_rev_info_t revinfo;


	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	dhd_os_set_ioctl_resp_timeout(IOCTL_RESP_TIMEOUT);



#ifdef DHD_FW_COREDUMP
	
	
	dhd_get_memdump_info(dhd);
#endif 

	
	memset(&revinfo, 0, sizeof(revinfo));
	ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_REVINFO, &revinfo, sizeof(revinfo), FALSE, 0);
	if (ret < 0) {
		DHD_ERROR(("%s: GET revinfo FAILED\n", __FUNCTION__));
		goto done;
	}
	DHD_ERROR(("%s: GET_REVINFO device 0x%x, vendor 0x%x, chipnum 0x%x\n", __FUNCTION__,
		revinfo.deviceid, revinfo.vendorid, revinfo.chipnum));

	dhd_process_cid_mac(dhd, TRUE);

	ret = dhd_preinit_ioctls(dhd);

	if (!ret) {
		dhd_process_cid_mac(dhd, FALSE);
	}

	
	dhd->iswl = TRUE;
done:
	return ret;
} 

#if defined(DHD_LB)


extern void dhd_lb_tx_compl_dispatch(dhd_pub_t *dhdp);
extern void dhd_lb_rx_compl_dispatch(dhd_pub_t *dhdp);
extern void dhd_lb_rx_napi_dispatch(dhd_pub_t *dhdp);

extern void dhd_lb_rx_pkt_enqueue(dhd_pub_t *dhdp, void *pkt, int ifidx);

static INLINE void
dhd_lb_dispatch(dhd_pub_t *dhdp, uint16 ring_idx)
{
	switch (ring_idx) {

#if defined(DHD_LB_TXC)
		case BCMPCIE_D2H_MSGRING_TX_COMPLETE:
			bcm_workq_prod_sync(&dhdp->prot->tx_compl_prod); 
			dhd_lb_tx_compl_dispatch(dhdp); 
			break;
#endif 

		case BCMPCIE_D2H_MSGRING_RX_COMPLETE:
		{
#if defined(DHD_LB_RXC)
			dhd_prot_t *prot = dhdp->prot;
			
			if (prot->rxbufpost <= (prot->max_rxbufpost - RXBUFPOST_THRESHOLD)) {
				
				bcm_workq_prod_sync(&dhdp->prot->rx_compl_prod);
				dhd_lb_rx_compl_dispatch(dhdp); 
			}
#endif 
#if defined(DHD_LB_RXP)
			dhd_lb_rx_napi_dispatch(dhdp); 
#endif 
			break;
		}
		default:
			break;
	}
}


#if defined(DHD_LB_TXC)
void
dhd_lb_tx_compl_handler(unsigned long data)
{
	int elem_ix;
	void *pkt, **elem;
	dmaaddr_t pa;
	uint32 pa_len;
	dhd_pub_t *dhd = (dhd_pub_t *)data;
	dhd_prot_t *prot = dhd->prot;
	bcm_workq_t *workq = &prot->tx_compl_cons;
	uint32 count = 0;

	DHD_LB_STATS_TXC_PERCPU_CNT_INCR(dhd);

	while (1) {
		elem_ix = bcm_ring_cons(WORKQ_RING(workq), DHD_LB_WORKQ_SZ);

		if (elem_ix == BCM_RING_EMPTY) {
			break;
		}

		elem = WORKQ_ELEMENT(void *, workq, elem_ix);
		pkt = *elem;

		DHD_INFO(("%s: tx_compl_cons pkt<%p>\n", __FUNCTION__, pkt));

		OSL_PREFETCH(PKTTAG(pkt));
		OSL_PREFETCH(pkt);

		pa = DHD_PKTTAG_PA((dhd_pkttag_fr_t *)PKTTAG(pkt));
		pa_len = DHD_PKTTAG_PA_LEN((dhd_pkttag_fr_t *)PKTTAG(pkt));

		DMA_UNMAP(dhd->osh, pa, pa_len, DMA_RX, 0, 0);

#if defined(BCMPCIE)
		dhd_txcomplete(dhd, pkt, true);
#endif 

		PKTFREE(dhd->osh, pkt, TRUE);
		count++;
	}

	
	bcm_workq_cons_sync(workq);
	DHD_LB_STATS_UPDATE_TXC_HISTO(dhd, count);
}
#endif 

#if defined(DHD_LB_RXC)
void
dhd_lb_rx_compl_handler(unsigned long data)
{
	dhd_pub_t *dhd = (dhd_pub_t *)data;
	bcm_workq_t *workq = &dhd->prot->rx_compl_cons;

	DHD_LB_STATS_RXC_PERCPU_CNT_INCR(dhd);

	dhd_msgbuf_rxbuf_post(dhd, TRUE); 
	bcm_workq_cons_sync(workq);
}
#endif 

#endif 

#define DHD_DBG_SHOW_METADATA	0

#if DHD_DBG_SHOW_METADATA
static void BCMFASTPATH
dhd_prot_print_metadata(dhd_pub_t *dhd, void *ptr, int len)
{
	uint8 tlv_t;
	uint8 tlv_l;
	uint8 *tlv_v = (uint8 *)ptr;

	if (len <= BCMPCIE_D2H_METADATA_HDRLEN)
		return;

	len -= BCMPCIE_D2H_METADATA_HDRLEN;
	tlv_v += BCMPCIE_D2H_METADATA_HDRLEN;

	while (len > TLV_HDR_LEN) {
		tlv_t = tlv_v[TLV_TAG_OFF];
		tlv_l = tlv_v[TLV_LEN_OFF];

		len -= TLV_HDR_LEN;
		tlv_v += TLV_HDR_LEN;
		if (len < tlv_l)
			break;
		if ((tlv_t == 0) || (tlv_t == WLFC_CTL_TYPE_FILLER))
			break;

		switch (tlv_t) {
		case WLFC_CTL_TYPE_TXSTATUS: {
			uint32 txs;
			memcpy(&txs, tlv_v, sizeof(uint32));
			if (tlv_l < (sizeof(wl_txstatus_additional_info_t) + sizeof(uint32))) {
				printf("METADATA TX_STATUS: %08x\n", txs);
			} else {
				wl_txstatus_additional_info_t tx_add_info;
				memcpy(&tx_add_info, tlv_v + sizeof(uint32),
					sizeof(wl_txstatus_additional_info_t));
				printf("METADATA TX_STATUS: %08x WLFCTS[%04x | %08x - %08x - %08x]"
					" rate = %08x tries = %d - %d\n", txs,
					tx_add_info.seq, tx_add_info.entry_ts,
					tx_add_info.enq_ts, tx_add_info.last_ts,
					tx_add_info.rspec, tx_add_info.rts_cnt,
					tx_add_info.tx_cnt);
			}
			} break;

		case WLFC_CTL_TYPE_RSSI: {
			if (tlv_l == 1)
				printf("METADATA RX_RSSI: rssi = %d\n", *tlv_v);
			else
				printf("METADATA RX_RSSI[%04x]: rssi = %d snr = %d\n",
					(*(tlv_v + 3) << 8) | *(tlv_v + 2),
					(int8)(*tlv_v), *(tlv_v + 1));
			} break;

		case WLFC_CTL_TYPE_FIFO_CREDITBACK:
			bcm_print_bytes("METADATA FIFO_CREDITBACK", tlv_v, tlv_l);
			break;

		case WLFC_CTL_TYPE_TX_ENTRY_STAMP:
			bcm_print_bytes("METADATA TX_ENTRY", tlv_v, tlv_l);
			break;

		case WLFC_CTL_TYPE_RX_STAMP: {
			struct {
				uint32 rspec;
				uint32 bus_time;
				uint32 wlan_time;
			} rx_tmstamp;
			memcpy(&rx_tmstamp, tlv_v, sizeof(rx_tmstamp));
			printf("METADATA RX TIMESTMAP: WLFCTS[%08x - %08x] rate = %08x\n",
				rx_tmstamp.wlan_time, rx_tmstamp.bus_time, rx_tmstamp.rspec);
			} break;

		case WLFC_CTL_TYPE_TRANS_ID:
			bcm_print_bytes("METADATA TRANS_ID", tlv_v, tlv_l);
			break;

		case WLFC_CTL_TYPE_COMP_TXSTATUS:
			bcm_print_bytes("METADATA COMP_TXSTATUS", tlv_v, tlv_l);
			break;

		default:
			bcm_print_bytes("METADATA UNKNOWN", tlv_v, tlv_l);
			break;
		}

		len -= tlv_l;
		tlv_v += tlv_l;
	}
}
#endif 

static INLINE void BCMFASTPATH
dhd_prot_packet_free(dhd_pub_t *dhd, void *pkt, uint8 pkttype, bool send)
{
	if (pkt) {
		if (pkttype == PKTTYPE_IOCTL_RX ||
			pkttype == PKTTYPE_EVENT_RX) {
#ifdef DHD_USE_STATIC_CTRLBUF
			PKTFREE_STATIC(dhd->osh, pkt, send);
#else
			PKTFREE(dhd->osh, pkt, send);
#endif 
		} else {
			PKTFREE(dhd->osh, pkt, send);
		}
	}
}

static INLINE void * BCMFASTPATH
dhd_prot_packet_get(dhd_pub_t *dhd, uint32 pktid, uint8 pkttype, bool free_pktid)
{
	void *PKTBUF;
	dmaaddr_t pa;
	uint32 len;
	void *dmah;
	void *secdma;

#ifdef DHD_PCIE_PKTID
	if (free_pktid) {
		PKTBUF = DHD_PKTID_TO_NATIVE(dhd, dhd->prot->pktid_map_handle,
			pktid, pa, len, dmah, secdma, pkttype);
	} else {
		PKTBUF = DHD_PKTID_TO_NATIVE_RSV(dhd, dhd->prot->pktid_map_handle,
			pktid, pa, len, dmah, secdma, pkttype);
	}
#else
	PKTBUF = DHD_PKTID_TO_NATIVE(dhd, dhd->prot->pktid_map_handle, pktid, pa,
		len, dmah, secdma, pkttype);
#endif 

	if (PKTBUF) {
		{
			if (SECURE_DMA_ENAB(dhd->osh)) {
				SECURE_DMA_UNMAP(dhd->osh, pa, (uint) len, DMA_RX, 0, dmah,
					secdma, 0);
			} else {
				DMA_UNMAP(dhd->osh, pa, (uint) len, DMA_RX, 0, dmah);
			}
		}
	}

	return PKTBUF;
}

#ifdef IOCTLRESP_USE_CONSTMEM
static INLINE void BCMFASTPATH
dhd_prot_ioctl_ret_buffer_get(dhd_pub_t *dhd, uint32 pktid, dhd_dma_buf_t *retbuf)
{
	memset(retbuf, 0, sizeof(dhd_dma_buf_t));
	retbuf->va = DHD_PKTID_TO_NATIVE(dhd, dhd->prot->pktid_map_handle_ioctl, pktid,
		retbuf->pa, retbuf->len, retbuf->dmah, retbuf->secdma, PKTTYPE_IOCTL_RX);

	return;
}
#endif 

static void BCMFASTPATH
dhd_msgbuf_rxbuf_post(dhd_pub_t *dhd, bool use_rsv_pktid)
{
	dhd_prot_t *prot = dhd->prot;
	int16 fillbufs;
	uint16 cnt = 256;
	int retcount = 0;

	fillbufs = prot->max_rxbufpost - prot->rxbufpost;
	while (fillbufs >= RX_BUF_BURST) {
		cnt--;
		if (cnt == 0) {
			
			DHD_ERROR(("h2d rx post ring not available to post host buffers \n"));
			DHD_ERROR(("Current posted host buf count %d \n", prot->rxbufpost));
			break;
		}

		
		fillbufs = MIN(fillbufs, RX_BUF_BURST);

		
		retcount = dhd_prot_rxbuf_post(dhd, fillbufs, use_rsv_pktid);

		if (retcount >= 0) {
			prot->rxbufpost += (uint16)retcount;
#ifdef DHD_LB_RXC
			
			DHD_LB_STATS_UPDATE_RXC_HISTO(dhd, retcount);
#endif 
			
			fillbufs = prot->max_rxbufpost - prot->rxbufpost;
		} else {
			
			fillbufs = 0;
		}
	}
}

static int BCMFASTPATH
dhd_prot_rxbuf_post(dhd_pub_t *dhd, uint16 count, bool use_rsv_pktid)
{
	void *p;
	uint16 pktsz = DHD_FLOWRING_RX_BUFPOST_PKTSZ;
	uint8 *rxbuf_post_tmp;
	host_rxbuf_post_t *rxbuf_post;
	void *msg_start;
	dmaaddr_t pa;
	uint32 pktlen;
	uint8 i = 0;
	uint16 alloced = 0;
	unsigned long flags;
	uint32 pktid;
	dhd_prot_t *prot = dhd->prot;
	msgbuf_ring_t *ring = &prot->h2dring_rxp_subn;

	DHD_GENERAL_LOCK(dhd, flags);

	
	msg_start = (void *)
		dhd_prot_alloc_ring_space(dhd, ring, count, &alloced, TRUE);

	DHD_GENERAL_UNLOCK(dhd, flags);

	if (msg_start == NULL) {
		DHD_INFO(("%s:%d: Rxbufpost Msgbuf Not available\n", __FUNCTION__, __LINE__));
		return -1;
	}
	
	ASSERT(alloced > 0);

	rxbuf_post_tmp = (uint8*)msg_start;

	
	for (i = 0; i < alloced; i++) {
		rxbuf_post = (host_rxbuf_post_t *)rxbuf_post_tmp;
		
		if ((p = PKTGET(dhd->osh, pktsz, FALSE)) == NULL) {
			DHD_ERROR(("%s:%d: PKTGET for rxbuf failed\n", __FUNCTION__, __LINE__));
			dhd->rx_pktgetfail++;
			break;
		}

		pktlen = PKTLEN(dhd->osh, p);
		if (SECURE_DMA_ENAB(dhd->osh)) {
			DHD_GENERAL_LOCK(dhd, flags);
			pa = SECURE_DMA_MAP(dhd->osh, PKTDATA(dhd->osh, p), pktlen,
				DMA_RX, p, 0, ring->dma_buf.secdma, 0);
			DHD_GENERAL_UNLOCK(dhd, flags);
		} else {
			pa = DMA_MAP(dhd->osh, PKTDATA(dhd->osh, p), pktlen, DMA_RX, p, 0);
		}

		if (PHYSADDRISZERO(pa)) {
			PKTFREE(dhd->osh, p, FALSE);
			DHD_ERROR(("Invalid phyaddr 0\n"));
			ASSERT(0);
			break;
		}

		PKTPULL(dhd->osh, p, prot->rx_metadata_offset);
		pktlen = PKTLEN(dhd->osh, p);

		
		rxbuf_post->cmn_hdr.msg_type = MSG_TYPE_RXBUF_POST;
		rxbuf_post->cmn_hdr.if_id = 0;
		rxbuf_post->cmn_hdr.epoch = ring->seqnum % H2D_EPOCH_MODULO;
		ring->seqnum++;

#if defined(DHD_LB_RXC)
		if (use_rsv_pktid == TRUE) {
			bcm_workq_t *workq = &prot->rx_compl_cons;
			int elem_ix = bcm_ring_cons(WORKQ_RING(workq), DHD_LB_WORKQ_SZ);
			if (elem_ix == BCM_RING_EMPTY) {
				DHD_ERROR(("%s rx_compl_cons ring is empty\n", __FUNCTION__));
				pktid = DHD_PKTID_INVALID;
				goto alloc_pkt_id;
			} else {
				uint32 *elem = WORKQ_ELEMENT(uint32, workq, elem_ix);
				pktid = *elem;
			}

			
			if (pktid != DHD_PKTID_INVALID) {
				rxbuf_post->cmn_hdr.request_id = htol32(pktid);
				DHD_NATIVE_TO_PKTID_SAVE(dhd, dhd->prot->pktid_map_handle, p, pktid,
					pa, pktlen, DMA_RX, NULL, ring->dma_buf.secdma,
					PKTTYPE_DATA_RX);
			}
		} else
#endif 
		{
#if defined(DHD_LB_RXC)
alloc_pkt_id:
#endif
#if defined(DHD_PCIE_PKTID)
		
		DHD_GENERAL_LOCK(dhd, flags);
#endif
		pktid = DHD_NATIVE_TO_PKTID(dhd, dhd->prot->pktid_map_handle, p, pa,
			pktlen, DMA_RX, NULL, ring->dma_buf.secdma, PKTTYPE_DATA_RX);

#if defined(DHD_PCIE_PKTID)
		
		DHD_GENERAL_UNLOCK(dhd, flags);

		if (pktid == DHD_PKTID_INVALID) {

			if (SECURE_DMA_ENAB(dhd->osh)) {
				DHD_GENERAL_LOCK(dhd, flags);
				SECURE_DMA_UNMAP(dhd->osh, pa, pktlen, DMA_RX, 0, DHD_DMAH_NULL,
				    ring->dma_buf.secdma, 0);
				DHD_GENERAL_UNLOCK(dhd, flags);
			} else {
				DMA_UNMAP(dhd->osh, pa, pktlen, DMA_RX, 0, DHD_DMAH_NULL);
			}

			PKTFREE(dhd->osh, p, FALSE);
			DHD_ERROR(("Pktid pool depleted.\n"));
			break;
		}
#endif 
		}

		rxbuf_post->data_buf_len = htol16((uint16)pktlen);
		rxbuf_post->data_buf_addr.high_addr = htol32(PHYSADDRHI(pa));
		rxbuf_post->data_buf_addr.low_addr =
			htol32(PHYSADDRLO(pa) + prot->rx_metadata_offset);

		if (prot->rx_metadata_offset) {
			rxbuf_post->metadata_buf_len = prot->rx_metadata_offset;
			rxbuf_post->metadata_buf_addr.high_addr = htol32(PHYSADDRHI(pa));
			rxbuf_post->metadata_buf_addr.low_addr  = htol32(PHYSADDRLO(pa));
		} else {
			rxbuf_post->metadata_buf_len = 0;
			rxbuf_post->metadata_buf_addr.high_addr = 0;
			rxbuf_post->metadata_buf_addr.low_addr  = 0;
		}

#if defined(DHD_PKTID_AUDIT_RING)
		DHD_PKTID_AUDIT(dhd, prot->pktid_map_handle, pktid, DHD_DUPLICATE_ALLOC);
#endif 

		rxbuf_post->cmn_hdr.request_id = htol32(pktid);

		
		rxbuf_post_tmp = rxbuf_post_tmp + ring->item_len;
	}

	if (i < alloced) {
		if (ring->wr < (alloced - i)) {
			ring->wr = ring->max_items - (alloced - i);
		} else {
			ring->wr -= (alloced - i);
		}

		alloced = i;
	}

	
	if (alloced > 0) {
		dhd_prot_ring_write_complete(dhd, ring, msg_start, alloced);
	}

	return alloced;
} 

#ifdef IOCTLRESP_USE_CONSTMEM
static int
alloc_ioctl_return_buffer(dhd_pub_t *dhd, dhd_dma_buf_t *retbuf)
{
	int err;
	memset(retbuf, 0, sizeof(dhd_dma_buf_t));

	if ((err = dhd_dma_buf_alloc(dhd, retbuf, IOCT_RETBUF_SIZE)) != BCME_OK) {
		DHD_ERROR(("%s: dhd_dma_buf_alloc err %d\n", __FUNCTION__, err));
		ASSERT(0);
		return BCME_NOMEM;
	}

	return BCME_OK;
}

static void
free_ioctl_return_buffer(dhd_pub_t *dhd, dhd_dma_buf_t *retbuf)
{
	
	if (retbuf->va) {
		uint32 dma_pad;
		dma_pad = (IOCT_RETBUF_SIZE % DHD_DMA_PAD) ? DHD_DMA_PAD : 0;
		retbuf->len = IOCT_RETBUF_SIZE;
		retbuf->_alloced = retbuf->len + dma_pad;
		/* JIRA:SWWLAN-70021 The pa value would be overwritten by the dongle.
		 * Need to reassign before free to pass the check in dhd_dma_buf_audit().
		 */
		retbuf->pa = DMA_MAP(dhd->osh, retbuf->va, retbuf->len, DMA_RX, NULL, NULL);
	}

	dhd_dma_buf_free(dhd, retbuf);
	return;
}
#endif 

static int
dhd_prot_rxbufpost_ctrl(dhd_pub_t *dhd, bool event_buf)
{
	void *p;
	uint16 pktsz;
	ioctl_resp_evt_buf_post_msg_t *rxbuf_post;
	dmaaddr_t pa;
	uint32 pktlen;
	dhd_prot_t *prot = dhd->prot;
	uint16 alloced = 0;
	unsigned long flags;
	dhd_dma_buf_t retbuf;
	void *dmah = NULL;
	uint32 pktid;
	void *map_handle;
	msgbuf_ring_t *ring = &prot->h2dring_ctrl_subn;

	if (dhd->busstate == DHD_BUS_DOWN) {
		DHD_ERROR(("%s: bus is already down.\n", __FUNCTION__));
		return -1;
	}

	memset(&retbuf, 0, sizeof(dhd_dma_buf_t));

	if (event_buf) {
		
		pktsz = DHD_FLOWRING_RX_BUFPOST_PKTSZ;
	} else {
		
		pktsz = DHD_FLOWRING_IOCTL_BUFPOST_PKTSZ;
	}

#ifdef IOCTLRESP_USE_CONSTMEM
	if (!event_buf) {
		if (alloc_ioctl_return_buffer(dhd, &retbuf) != BCME_OK) {
			DHD_ERROR(("Could not allocate IOCTL response buffer\n"));
			return -1;
		}
		ASSERT(retbuf.len == IOCT_RETBUF_SIZE);
		p = retbuf.va;
		pktlen = retbuf.len;
		pa = retbuf.pa;
		dmah = retbuf.dmah;
	} else
#endif 
	{
#ifdef DHD_USE_STATIC_CTRLBUF
		p = PKTGET_STATIC(dhd->osh, pktsz, FALSE);
#else
		p = PKTGET(dhd->osh, pktsz, FALSE);
#endif 
		if (p == NULL) {
			DHD_ERROR(("%s:%d: PKTGET for %s buf failed\n",
				__FUNCTION__, __LINE__, event_buf ?
				"EVENT" : "IOCTL RESP"));
			dhd->rx_pktgetfail++;
			return -1;
		}

		pktlen = PKTLEN(dhd->osh, p);

		if (SECURE_DMA_ENAB(dhd->osh)) {
			DHD_GENERAL_LOCK(dhd, flags);
			pa = SECURE_DMA_MAP(dhd->osh, PKTDATA(dhd->osh, p), pktlen,
				DMA_RX, p, 0, ring->dma_buf.secdma, 0);
			DHD_GENERAL_UNLOCK(dhd, flags);
		} else {
			pa = DMA_MAP(dhd->osh, PKTDATA(dhd->osh, p), pktlen, DMA_RX, p, 0);
		}

		if (PHYSADDRISZERO(pa)) {
			DHD_ERROR(("Invalid physaddr 0\n"));
			ASSERT(0);
			goto free_pkt_return;
		}
	}

	DHD_GENERAL_LOCK(dhd, flags);

	rxbuf_post = (ioctl_resp_evt_buf_post_msg_t *)
		dhd_prot_alloc_ring_space(dhd, ring, 1, &alloced, FALSE);

	if (rxbuf_post == NULL) {
		DHD_GENERAL_UNLOCK(dhd, flags);
		DHD_ERROR(("%s:%d: Ctrl submit Msgbuf Not available to post buffer \n",
			__FUNCTION__, __LINE__));

#ifdef IOCTLRESP_USE_CONSTMEM
		if (event_buf)
#endif 
		{
			if (SECURE_DMA_ENAB(dhd->osh)) {
				DHD_GENERAL_LOCK(dhd, flags);
				SECURE_DMA_UNMAP(dhd->osh, pa, pktlen, DMA_RX, 0, DHD_DMAH_NULL,
					ring->dma_buf.secdma, 0);
				DHD_GENERAL_UNLOCK(dhd, flags);
			} else {
				DMA_UNMAP(dhd->osh, pa, pktlen, DMA_RX, 0, DHD_DMAH_NULL);
			}
		}
		goto free_pkt_return;
	}

	
	if (event_buf) {
		rxbuf_post->cmn_hdr.msg_type = MSG_TYPE_EVENT_BUF_POST;
	} else {
		rxbuf_post->cmn_hdr.msg_type = MSG_TYPE_IOCTLRESP_BUF_POST;
	}

#ifdef IOCTLRESP_USE_CONSTMEM
	if (!event_buf) {
		map_handle = dhd->prot->pktid_map_handle_ioctl;
		pktid =	DHD_NATIVE_TO_PKTID(dhd, map_handle, p, pa, pktlen,
			DMA_RX, dmah, ring->dma_buf.secdma, PKTTYPE_IOCTL_RX);
	} else
#endif 
	{
		map_handle = dhd->prot->pktid_map_handle;
		pktid =	DHD_NATIVE_TO_PKTID(dhd, map_handle,
			p, pa, pktlen, DMA_RX, dmah, ring->dma_buf.secdma,
			event_buf ? PKTTYPE_EVENT_RX : PKTTYPE_IOCTL_RX);
	}

	if (pktid == DHD_PKTID_INVALID) {
		if (ring->wr == 0) {
			ring->wr = ring->max_items - 1;
		} else {
			ring->wr--;
		}
		DHD_GENERAL_UNLOCK(dhd, flags);
		DMA_UNMAP(dhd->osh, pa, pktlen, DMA_RX, 0, DHD_DMAH_NULL);
		goto free_pkt_return;
	}

#if defined(DHD_PKTID_AUDIT_RING)
	DHD_PKTID_AUDIT(dhd, map_handle, pktid, DHD_DUPLICATE_ALLOC);
#endif 

	rxbuf_post->cmn_hdr.request_id = htol32(pktid);
	rxbuf_post->cmn_hdr.if_id = 0;
	rxbuf_post->cmn_hdr.epoch = ring->seqnum % H2D_EPOCH_MODULO;
	ring->seqnum++;

#if defined(DHD_PCIE_PKTID)
	if (rxbuf_post->cmn_hdr.request_id == DHD_PKTID_INVALID) {
		if (ring->wr == 0) {
			ring->wr = ring->max_items - 1;
		} else {
			ring->wr--;
		}
		DHD_GENERAL_UNLOCK(dhd, flags);
#ifdef IOCTLRESP_USE_CONSTMEM
		if (event_buf)
#endif 
		{
			if (SECURE_DMA_ENAB(dhd->osh)) {
				DHD_GENERAL_LOCK(dhd, flags);
				SECURE_DMA_UNMAP(dhd->osh, pa, pktlen, DMA_RX, 0, DHD_DMAH_NULL,
					ring->dma_buf.secdma, 0);
				DHD_GENERAL_UNLOCK(dhd, flags);
			} else {
				DMA_UNMAP(dhd->osh, pa, pktlen, DMA_RX, 0, DHD_DMAH_NULL);
			}
		}
		goto free_pkt_return;
	}
#endif 

	rxbuf_post->cmn_hdr.flags = 0;
#ifndef IOCTLRESP_USE_CONSTMEM
	rxbuf_post->host_buf_len = htol16((uint16)PKTLEN(dhd->osh, p));
#else
	rxbuf_post->host_buf_len = htol16((uint16)pktlen);
#endif 
	rxbuf_post->host_buf_addr.high_addr = htol32(PHYSADDRHI(pa));
	rxbuf_post->host_buf_addr.low_addr  = htol32(PHYSADDRLO(pa));

	
	dhd_prot_ring_write_complete(dhd, ring, rxbuf_post, 1);
	DHD_GENERAL_UNLOCK(dhd, flags);

	return 1;

free_pkt_return:
#ifdef IOCTLRESP_USE_CONSTMEM
	if (!event_buf) {
		free_ioctl_return_buffer(dhd, &retbuf);
	} else
#endif 
	{
		dhd_prot_packet_free(dhd, p,
			event_buf ? PKTTYPE_EVENT_RX : PKTTYPE_IOCTL_RX,
			FALSE);
	}

	return -1;
} 

static uint16
dhd_msgbuf_rxbuf_post_ctrlpath(dhd_pub_t *dhd, bool event_buf, uint32 max_to_post)
{
	uint32 i = 0;
	int32 ret_val;

	DHD_INFO(("max to post %d, event %d \n", max_to_post, event_buf));

	if (dhd->busstate == DHD_BUS_DOWN) {
		DHD_ERROR(("%s: bus is already down.\n", __FUNCTION__));
		return 0;
	}

	while (i < max_to_post) {
		ret_val  = dhd_prot_rxbufpost_ctrl(dhd, event_buf);
		if (ret_val < 0) {
			break;
		}
		i++;
	}
	DHD_INFO(("posted %d buffers to event_pool/ioctl_resp_pool %d\n", i, event_buf));
	return (uint16)i;
}

static void
dhd_msgbuf_rxbuf_post_ioctlresp_bufs(dhd_pub_t *dhd)
{
	dhd_prot_t *prot = dhd->prot;
	int max_to_post;

	DHD_INFO(("ioctl resp buf post\n"));
	max_to_post = prot->max_ioctlrespbufpost - prot->cur_ioctlresp_bufs_posted;
	if (max_to_post <= 0) {
		DHD_INFO(("%s: Cannot post more than max IOCTL resp buffers\n",
			__FUNCTION__));
		return;
	}
	prot->cur_ioctlresp_bufs_posted += dhd_msgbuf_rxbuf_post_ctrlpath(dhd,
		FALSE, max_to_post);
}

static void
dhd_msgbuf_rxbuf_post_event_bufs(dhd_pub_t *dhd)
{
	dhd_prot_t *prot = dhd->prot;
	int max_to_post;

	max_to_post = prot->max_eventbufpost - prot->cur_event_bufs_posted;
	if (max_to_post <= 0) {
		DHD_INFO(("%s: Cannot post more than max event buffers\n",
			__FUNCTION__));
		return;
	}
	prot->cur_event_bufs_posted += dhd_msgbuf_rxbuf_post_ctrlpath(dhd,
		TRUE, max_to_post);
}

bool BCMFASTPATH
dhd_prot_process_msgbuf_rxcpl(dhd_pub_t *dhd, uint bound)
{
	bool more = TRUE;
	uint n = 0;
	msgbuf_ring_t *ring = &dhd->prot->d2hring_rx_cpln;

	
	while (!dhd_is_device_removed(dhd)) {
		uint8 *msg_addr;
		uint32 msg_len;

		if (dhd->hang_was_sent) {
			more = FALSE;
			break;
		}

		
		msg_addr = dhd_prot_get_read_addr(dhd, ring, &msg_len);
		if (msg_addr == NULL) {
			more = FALSE;
			break;
		}

		
		OSL_PREFETCH(msg_addr);

		if (dhd_prot_process_msgtype(dhd, ring, msg_addr, msg_len) != BCME_OK) {
			DHD_ERROR(("%s: process %s msg addr %p len %d\n",
				__FUNCTION__, ring->name, msg_addr, msg_len));
		}

		
		dhd_prot_upd_read_idx(dhd, ring);

		
		n += msg_len / ring->item_len;
		if (n >= bound) {
			break;
		}
	}

	return more;
}

void
dhd_prot_update_txflowring(dhd_pub_t *dhd, uint16 flowid, void *msgring)
{
	msgbuf_ring_t *ring = (msgbuf_ring_t *)msgring;

	
	if (DMA_INDX_ENAB(dhd->dma_d2h_ring_upd_support)) {
		ring->rd = dhd_prot_dma_indx_get(dhd, H2D_DMA_INDX_RD_UPD, ring->idx);
	}

	DHD_TRACE(("ringid %d flowid %d write %d read %d \n\n",
		ring->idx, flowid, ring->wr, ring->rd));

	
	dhd_bus_schedule_queue(dhd->bus, flowid, TRUE); 
}

bool BCMFASTPATH
dhd_prot_process_msgbuf_txcpl(dhd_pub_t *dhd, uint bound)
{
	bool more = TRUE;
	uint n = 0;
	msgbuf_ring_t *ring = &dhd->prot->d2hring_tx_cpln;

	
	while (!dhd_is_device_removed(dhd)) {
		uint8 *msg_addr;
		uint32 msg_len;

		if (dhd->hang_was_sent) {
			more = FALSE;
			break;
		}

		
		msg_addr = dhd_prot_get_read_addr(dhd, ring, &msg_len);
		if (msg_addr == NULL) {
			more = FALSE;
			break;
		}

		
		OSL_PREFETCH(msg_addr);

		if (dhd_prot_process_msgtype(dhd, ring, msg_addr, msg_len) != BCME_OK) {
			DHD_ERROR(("%s: process %s msg addr %p len %d\n",
				__FUNCTION__, ring->name, msg_addr, msg_len));
		}

		
		dhd_prot_upd_read_idx(dhd, ring);

		
		n += msg_len / ring->item_len;
		if (n >= bound) {
			break;
		}
	}

	return more;
}

int BCMFASTPATH
dhd_prot_process_ctrlbuf(dhd_pub_t *dhd)
{
	dhd_prot_t *prot = dhd->prot;
	msgbuf_ring_t *ring = &prot->d2hring_ctrl_cpln;

	
	while (!dhd_is_device_removed(dhd)) {
		uint8 *msg_addr;
		uint32 msg_len;

		if (dhd->hang_was_sent) {
			break;
		}

		
		msg_addr = dhd_prot_get_read_addr(dhd, ring, &msg_len);
		if (msg_addr == NULL) {
			break;
		}

		
		OSL_PREFETCH(msg_addr);
		if (dhd_prot_process_msgtype(dhd, ring, msg_addr, msg_len) != BCME_OK) {
			DHD_ERROR(("%s: process %s msg addr %p len %d\n",
				__FUNCTION__, ring->name, msg_addr, msg_len));
		}

		
		dhd_prot_upd_read_idx(dhd, ring);
	}

	return 0;
}

static int BCMFASTPATH
dhd_prot_process_msgtype(dhd_pub_t *dhd, msgbuf_ring_t *ring, uint8 *buf, uint32 len)
{
	int buf_len = len;
	uint16 item_len;
	uint8 msg_type;
	cmn_msg_hdr_t *msg = NULL;
	int ret = BCME_OK;

	ASSERT(ring);
	item_len = ring->item_len;
	if (item_len == 0) {
		DHD_ERROR(("%s: ringidx %d item_len %d buf_len %d\n",
			__FUNCTION__, ring->idx, item_len, buf_len));
		return BCME_ERROR;
	}

	while (buf_len > 0) {
		if (dhd->hang_was_sent) {
			ret = BCME_ERROR;
			goto done;
		}

		msg = (cmn_msg_hdr_t *)buf;

#if defined(PCIE_D2H_SYNC)
		
		msg_type = dhd->prot->d2h_sync_cb(dhd, ring, msg, item_len);
#else
		msg_type = msg->msg_type;
#endif 

		
		OSL_PREFETCH(buf + item_len);

		DHD_INFO(("msg_type %d item_len %d buf_len %d\n",
			msg_type, item_len, buf_len));

		if (msg_type == MSG_TYPE_LOOPBACK) {
			bcm_print_bytes("LPBK RESP: ", (uint8 *)msg, item_len);
			DHD_ERROR((" MSG_TYPE_LOOPBACK, len %d\n", item_len));
		}

		ASSERT(msg_type < DHD_PROT_FUNCS);
		if (msg_type >= DHD_PROT_FUNCS) {
			DHD_ERROR(("%s: msg_type %d item_len %d buf_len %d\n",
				__FUNCTION__, msg_type, item_len, buf_len));
			ret = BCME_ERROR;
			goto done;
		}

		if (table_lookup[msg_type]) {
			table_lookup[msg_type](dhd, buf);
		}

		if (buf_len < item_len) {
			ret = BCME_ERROR;
			goto done;
		}
		buf_len = buf_len - item_len;
		buf = buf + item_len;
	}

done:

#ifdef DHD_RX_CHAINING
	dhd_rxchain_commit(dhd);
#endif
#if defined(DHD_LB)
	dhd_lb_dispatch(dhd, ring->idx);
#endif
	return ret;
} 

static void
dhd_prot_noop(dhd_pub_t *dhd, void *msg)
{
	return;
}

static void
dhd_prot_ringstatus_process(dhd_pub_t *dhd, void *msg)
{
	pcie_ring_status_t *ring_status = (pcie_ring_status_t *)msg;
	DHD_ERROR(("ring status: request_id %d, status 0x%04x, flow ring %d, write_idx %d \n",
		ring_status->cmn_hdr.request_id, ring_status->compl_hdr.status,
		ring_status->compl_hdr.flow_ring_id, ring_status->write_idx));
	
	return;
}

static void
dhd_prot_genstatus_process(dhd_pub_t *dhd, void *msg)
{
	pcie_gen_status_t *gen_status = (pcie_gen_status_t *)msg;
	DHD_ERROR(("ERROR: gen status: request_id %d, STATUS 0x%04x, flow ring %d \n",
		gen_status->cmn_hdr.request_id, gen_status->compl_hdr.status,
		gen_status->compl_hdr.flow_ring_id));

	
	return;
}

static void
dhd_prot_ioctack_process(dhd_pub_t *dhd, void *msg)
{
	uint32 pktid;
	ioctl_req_ack_msg_t *ioct_ack = (ioctl_req_ack_msg_t *)msg;
	unsigned long flags;

	pktid = ltoh32(ioct_ack->cmn_hdr.request_id);

#if defined(DHD_PKTID_AUDIT_RING)
	
	if (pktid != DHD_IOCTL_REQ_PKTID) {
		DHD_PKTID_AUDIT(dhd, dhd->prot->pktid_map_handle, pktid,
			DHD_TEST_IS_ALLOC);
	}
#endif 

	DHD_GENERAL_LOCK(dhd, flags);
	if ((dhd->prot->ioctl_state & MSGBUF_IOCTL_ACK_PENDING) &&
		(dhd->prot->ioctl_state & MSGBUF_IOCTL_RESP_PENDING)) {
		dhd->prot->ioctl_state &= ~MSGBUF_IOCTL_ACK_PENDING;
	} else {
		DHD_ERROR(("%s: received ioctl ACK with state %02x\n",
			__FUNCTION__, dhd->prot->ioctl_state));
	}
	DHD_GENERAL_UNLOCK(dhd, flags);

	DHD_CTL(("ioctl req ack: request_id %d, status 0x%04x, flow ring %d \n",
		ioct_ack->cmn_hdr.request_id, ioct_ack->compl_hdr.status,
		ioct_ack->compl_hdr.flow_ring_id));
	if (ioct_ack->compl_hdr.status != 0)  {
		DHD_ERROR(("got an error status for the ioctl request...need to handle that\n"));
	}
}

static void
dhd_prot_ioctcmplt_process(dhd_pub_t *dhd, void *msg)
{
	dhd_prot_t *prot = dhd->prot;
	uint32 pkt_id, xt_id;
	ioctl_comp_resp_msg_t *ioct_resp = (ioctl_comp_resp_msg_t *)msg;
	void *pkt;
	unsigned long flags;
	dhd_dma_buf_t retbuf;

	memset(&retbuf, 0, sizeof(dhd_dma_buf_t));

	pkt_id = ltoh32(ioct_resp->cmn_hdr.request_id);

#if defined(DHD_PKTID_AUDIT_RING)
#ifndef IOCTLRESP_USE_CONSTMEM
	DHD_PKTID_AUDIT(dhd, prot->pktid_map_handle, pkt_id, DHD_DUPLICATE_FREE);
#else
	DHD_PKTID_AUDIT(dhd, prot->pktid_map_handle_ioctl, pkt_id, DHD_DUPLICATE_FREE);
#endif 
#endif 

	DHD_GENERAL_LOCK(dhd, flags);
	if ((prot->ioctl_state & MSGBUF_IOCTL_ACK_PENDING) ||
		!(prot->ioctl_state & MSGBUF_IOCTL_RESP_PENDING)) {
		
		prot->ioctl_state = 0;
		DHD_GENERAL_UNLOCK(dhd, flags);
		DHD_ERROR(("%s: received ioctl response with state %02x\n",
			__FUNCTION__, dhd->prot->ioctl_state));
		return;
	}
#ifndef IOCTLRESP_USE_CONSTMEM
	pkt = dhd_prot_packet_get(dhd, pkt_id, PKTTYPE_IOCTL_RX, TRUE);
#else
	dhd_prot_ioctl_ret_buffer_get(dhd, pkt_id, &retbuf);
	pkt = retbuf.va;
#endif 
	if (!pkt) {
		prot->ioctl_state = 0;
		DHD_GENERAL_UNLOCK(dhd, flags);
		DHD_ERROR(("%s: received ioctl response with NULL pkt\n", __FUNCTION__));
		return;
	}
	DHD_GENERAL_UNLOCK(dhd, flags);

	prot->ioctl_resplen = ltoh16(ioct_resp->resp_len);
	prot->ioctl_status = ltoh16(ioct_resp->compl_hdr.status);
	xt_id = ltoh16(ioct_resp->trans_id);
	if (xt_id != prot->ioctl_trans_id) {
		ASSERT(0);
		goto exit;
	}

	DHD_CTL(("IOCTL_COMPLETE: req_id %x transid %d status %x resplen %d\n",
		pkt_id, xt_id, prot->ioctl_status, prot->ioctl_resplen));

	if (prot->ioctl_resplen > 0) {
#ifndef IOCTLRESP_USE_CONSTMEM
		bcopy(PKTDATA(dhd->osh, pkt), prot->retbuf.va, prot->ioctl_resplen);
#else
		bcopy(pkt, prot->retbuf.va, prot->ioctl_resplen);
#endif 
	}

	
	dhd_wakeup_ioctl_event(dhd, IOCTL_RETURN_ON_SUCCESS);

exit:
#ifndef IOCTLRESP_USE_CONSTMEM
	dhd_prot_packet_free(dhd, pkt,
		PKTTYPE_IOCTL_RX, FALSE);
#else
	free_ioctl_return_buffer(dhd, &retbuf);
#endif 
}

static void BCMFASTPATH
dhd_prot_txstatus_process(dhd_pub_t *dhd, void *msg)
{
	dhd_prot_t *prot = dhd->prot;
	host_txbuf_cmpl_t * txstatus;
	unsigned long flags;
	uint32 pktid;
	void *pkt = NULL;
	ulong pa;
	uint32 len;
	void *dmah;
	void *secdma;

	
	DHD_GENERAL_LOCK(dhd, flags);

	txstatus = (host_txbuf_cmpl_t *)msg;
	pktid = ltoh32(txstatus->cmn_hdr.request_id);

#if defined(DHD_PKTID_AUDIT_RING)
	DHD_PKTID_AUDIT(dhd, dhd->prot->pktid_map_handle, pktid,
		DHD_DUPLICATE_FREE);
#endif 

	DHD_INFO(("txstatus for pktid 0x%04x\n", pktid));
	if (prot->active_tx_count) {
		prot->active_tx_count--;

		
		if (prot->active_tx_count == 0)
			 DHD_OS_WAKE_UNLOCK(dhd);

	} else {
		DHD_ERROR(("Extra packets are freed\n"));
	}

	
#if defined(TX_STUCK_DETECTION) && defined(CUSTOMER_HW_ONE)
	dhd->new_tx_completed_count++;
#endif 
	

	ASSERT(pktid != 0);

#if defined(DHD_LB_TXC) && !defined(BCM_SECURE_DMA)
	{
		int elem_ix;
		void **elem;
		bcm_workq_t *workq;

		pkt = DHD_PKTID_TO_NATIVE(dhd, dhd->prot->pktid_map_handle,
			pktid, pa, len, dmah, secdma, PKTTYPE_DATA_TX);

		workq = &prot->tx_compl_prod;
		OSL_PREFETCH(PKTTAG(pkt));

		
		elem_ix = bcm_ring_prod(WORKQ_RING(workq), DHD_LB_WORKQ_SZ);

		DHD_PKTTAG_SET_PA((dhd_pkttag_fr_t *)PKTTAG(pkt), pa);
		DHD_PKTTAG_SET_PA_LEN((dhd_pkttag_fr_t *)PKTTAG(pkt), len);

		if (elem_ix == BCM_RING_FULL) {
			DHD_ERROR(("tx_compl_prod BCM_RING_FULL\n"));
			goto workq_ring_full;
		}

		elem = WORKQ_ELEMENT(void *, &prot->tx_compl_prod, elem_ix);
		*elem = pkt;

		smp_wmb();

		
		if (++prot->tx_compl_prod_sync >= DHD_LB_WORKQ_SYNC) {
			bcm_workq_prod_sync(workq);
			prot->tx_compl_prod_sync = 0;
		}

		DHD_INFO(("%s: tx_compl_prod pkt<%p> sync<%d>\n",
		__FUNCTION__, pkt, prot->tx_compl_prod_sync));

		DHD_GENERAL_UNLOCK(dhd, flags);
		return;
	   }

workq_ring_full:

#endif 

	if (pkt == NULL) {
		pkt = DHD_PKTID_TO_NATIVE(dhd, dhd->prot->pktid_map_handle,
			pktid, pa, len, dmah, secdma, PKTTYPE_DATA_TX);
	}

	if (pkt) {
		if (SECURE_DMA_ENAB(dhd->osh)) {
			int offset = 0;
			BCM_REFERENCE(offset);

			if (dhd->prot->tx_metadata_offset)
				offset = dhd->prot->tx_metadata_offset + ETHER_HDR_LEN;
			SECURE_DMA_UNMAP(dhd->osh, (uint) pa,
				(uint) dhd->prot->tx_metadata_offset, DMA_RX, 0, dmah,
				secdma, offset);
		} else {
			DMA_UNMAP(dhd->osh, pa, (uint) len, DMA_RX, 0, dmah);
		}
#if defined(BCMPCIE)
		dhd_txcomplete(dhd, pkt, true);
#endif 

#if DHD_DBG_SHOW_METADATA
		if (dhd->prot->metadata_dbg &&
		    dhd->prot->tx_metadata_offset && txstatus->metadata_len) {
			uchar *ptr;
			PKTPULL(dhd->osh, pkt, ETHER_HDR_LEN);
			ptr = PKTDATA(dhd->osh, pkt)  - (dhd->prot->tx_metadata_offset);
			bcm_print_bytes("txmetadata", ptr, txstatus->metadata_len);
			dhd_prot_print_metadata(dhd, ptr, txstatus->metadata_len);
		}
#endif 
		PKTFREE(dhd->osh, pkt, TRUE);
		DHD_FLOWRING_TXSTATUS_CNT_UPDATE(dhd->bus, txstatus->compl_hdr.flow_ring_id,
		txstatus->tx_status);
	}

	DHD_GENERAL_UNLOCK(dhd, flags);

	return;
} 

static void
dhd_prot_event_process(dhd_pub_t *dhd, void *msg)
{
	wlevent_req_msg_t *evnt;
	uint32 bufid;
	uint16 buflen;
	int ifidx = 0;
	void* pkt;
	unsigned long flags;
	dhd_prot_t *prot = dhd->prot;

	
	evnt = (wlevent_req_msg_t *)msg;
	bufid = ltoh32(evnt->cmn_hdr.request_id);

#if defined(DHD_PKTID_AUDIT_RING)
	DHD_PKTID_AUDIT(dhd, dhd->prot->pktid_map_handle, bufid,
		DHD_DUPLICATE_FREE);
#endif 

	buflen = ltoh16(evnt->event_data_len);

	ifidx = BCMMSGBUF_API_IFIDX(&evnt->cmn_hdr);

	
	if (prot->cur_event_bufs_posted) {
		prot->cur_event_bufs_posted--;
	}
	dhd_msgbuf_rxbuf_post_event_bufs(dhd);

	
	DHD_GENERAL_LOCK(dhd, flags);
	pkt = dhd_prot_packet_get(dhd, bufid, PKTTYPE_EVENT_RX, TRUE);
	DHD_GENERAL_UNLOCK(dhd, flags);

	if (!pkt) {
		DHD_ERROR(("%s: pkt is NULL\n", __FUNCTION__));
		return;
	}

	
	if (dhd->prot->rx_dataoffset) {
		PKTPULL(dhd->osh, pkt, dhd->prot->rx_dataoffset);
	}

	PKTSETLEN(dhd->osh, pkt, buflen);

	dhd_bus_rx_frame(dhd->bus, pkt, ifidx, 1);
}

static void BCMFASTPATH
dhd_prot_rxcmplt_process(dhd_pub_t *dhd, void *msg)
{
	host_rxbuf_cmpl_t *rxcmplt_h;
	uint16 data_offset;             
	void *pkt;
	unsigned long flags;
	uint ifidx;
	uint32 pktid;
#if defined(DHD_LB_RXC)
	const bool free_pktid = FALSE;
#else
	const bool free_pktid = TRUE;
#endif 

	
	rxcmplt_h = (host_rxbuf_cmpl_t *)msg;

	
	data_offset = ltoh16(rxcmplt_h->data_offset);

	pktid = ltoh32(rxcmplt_h->cmn_hdr.request_id);

#if defined(DHD_PKTID_AUDIT_RING)
	DHD_PKTID_AUDIT(dhd, dhd->prot->pktid_map_handle, pktid,
		DHD_DUPLICATE_FREE);
#endif 

	DHD_GENERAL_LOCK(dhd, flags);
	pkt = dhd_prot_packet_get(dhd, pktid, PKTTYPE_DATA_RX, free_pktid);
	DHD_GENERAL_UNLOCK(dhd, flags);

	if (!pkt) {
		DHD_ERROR(("%s: pkt is NULL\n", __FUNCTION__));
		return;
	}

	
	dhd_prot_return_rxbuf(dhd, pktid, 1);

	DHD_INFO(("id 0x%04x, offset %d, len %d, idx %d, phase 0x%02x, pktdata %p, metalen %d\n",
		ltoh32(rxcmplt_h->cmn_hdr.request_id), data_offset, ltoh16(rxcmplt_h->data_len),
		rxcmplt_h->cmn_hdr.if_id, rxcmplt_h->cmn_hdr.flags, PKTDATA(dhd->osh, pkt),
		ltoh16(rxcmplt_h->metadata_len)));
#if DHD_DBG_SHOW_METADATA
	if (dhd->prot->metadata_dbg &&
	    dhd->prot->rx_metadata_offset && rxcmplt_h->metadata_len) {
		uchar *ptr;
		ptr = PKTDATA(dhd->osh, pkt) - (dhd->prot->rx_metadata_offset);
		
		bcm_print_bytes("rxmetadata", ptr, rxcmplt_h->metadata_len);
		dhd_prot_print_metadata(dhd, ptr, rxcmplt_h->metadata_len);
	}
#endif 

	if (rxcmplt_h->flags & BCMPCIE_PKT_FLAGS_FRAME_802_11) {
		DHD_INFO(("D11 frame rxed \n"));
	}

	
	if (data_offset) {
		
		PKTPULL(dhd->osh, pkt, data_offset); 
	} else {
		
		if (dhd->prot->rx_dataoffset) {
			PKTPULL(dhd->osh, pkt, dhd->prot->rx_dataoffset);
		}
	}
	
	PKTSETLEN(dhd->osh, pkt, ltoh16(rxcmplt_h->data_len));

	ifidx = rxcmplt_h->cmn_hdr.if_id;

#if defined(DHD_LB_RXP)
	dhd_lb_rx_pkt_enqueue(dhd, pkt, ifidx);
#else  
#ifdef DHD_RX_CHAINING
	
	dhd_rxchain_frame(dhd, pkt, ifidx);
#else 
	
	dhd_bus_rx_frame(dhd->bus, pkt, ifidx, 1);
#endif 
#endif 
} 

void dhd_prot_stop(dhd_pub_t *dhd)
{
	ASSERT(dhd);
	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

}

void BCMFASTPATH
dhd_prot_hdrpush(dhd_pub_t *dhd, int ifidx, void *PKTBUF)
{
	return;
}

uint
dhd_prot_hdrlen(dhd_pub_t *dhd, void *PKTBUF)
{
	return 0;
}


#define PKTBUF pktbuf

int BCMFASTPATH
dhd_prot_txdata(dhd_pub_t *dhd, void *PKTBUF, uint8 ifidx)
{
	unsigned long flags;
	dhd_prot_t *prot = dhd->prot;
	host_txbuf_post_t *txdesc = NULL;
	dmaaddr_t pa, meta_pa;
	uint8 *pktdata;
	uint32 pktlen;
	uint32 pktid;
	uint8	prio;
	uint16 flowid = 0;
	uint16 alloced = 0;
	uint16	headroom;
	msgbuf_ring_t *ring;
	flow_ring_table_t *flow_ring_table;
	flow_ring_node_t *flow_ring_node;

	if (dhd->flow_ring_table == NULL) {
		return BCME_NORESOURCE;
	}

	flowid = DHD_PKT_GET_FLOWID(PKTBUF);

	flow_ring_table = (flow_ring_table_t *)dhd->flow_ring_table;
	flow_ring_node = (flow_ring_node_t *)&flow_ring_table[flowid];

	ring = (msgbuf_ring_t *)flow_ring_node->prot_info;


	DHD_GENERAL_LOCK(dhd, flags);

	
	pktid = DHD_NATIVE_TO_PKTID_RSV(dhd, dhd->prot->pktid_map_handle, PKTBUF);
#if defined(DHD_PCIE_PKTID)
	if (pktid == DHD_PKTID_INVALID) {
		DHD_ERROR(("Pktid pool depleted.\n"));
		goto err_no_res_pktfree;
	}
#endif 

	
	txdesc = (host_txbuf_post_t *)
		dhd_prot_alloc_ring_space(dhd, ring, 1, &alloced, FALSE);
	if (txdesc == NULL) {
#if defined(DHD_PCIE_PKTID)
		void *dmah;
		void *secdma;
		
		DHD_PKTID_TO_NATIVE(dhd, dhd->prot->pktid_map_handle, pktid,
			pa, pktlen, dmah, secdma, PKTTYPE_NO_CHECK);
#endif 
		DHD_INFO(("%s:%d: HTOD Msgbuf Not available TxCount = %d\n",
			__FUNCTION__, __LINE__, prot->active_tx_count));
		goto err_no_res_pktfree;
	}

	
	pktdata = PKTDATA(dhd->osh, PKTBUF);
	pktlen  = PKTLEN(dhd->osh, PKTBUF);

	
	bcopy(pktdata, txdesc->txhdr, ETHER_HDR_LEN);

	
	pktdata = PKTPULL(dhd->osh, PKTBUF, ETHER_HDR_LEN);
	pktlen -= ETHER_HDR_LEN;

	
	if (SECURE_DMA_ENAB(dhd->osh)) {
		int offset = 0;
		BCM_REFERENCE(offset);

		if (prot->tx_metadata_offset) {
			offset = prot->tx_metadata_offset + ETHER_HDR_LEN;
		}

		pa = SECURE_DMA_MAP(dhd->osh, PKTDATA(dhd->osh, PKTBUF), pktlen,
			DMA_TX, PKTBUF, 0, ring->dma_buf.secdma, offset);
	} else {
		pa = DMA_MAP(dhd->osh, PKTDATA(dhd->osh, PKTBUF), pktlen, DMA_TX, PKTBUF, 0);
	}

	if (PHYSADDRISZERO(pa)) {
		DHD_ERROR(("Something really bad, unless 0 is a valid phyaddr\n"));
		ASSERT(0);
	}

	
	DHD_NATIVE_TO_PKTID_SAVE(dhd, dhd->prot->pktid_map_handle, PKTBUF, pktid,
	    pa, pktlen, DMA_TX, NULL, ring->dma_buf.secdma, PKTTYPE_DATA_TX);

#ifdef TXP_FLUSH_NITEMS
	if (ring->pend_items_count == 0) {
		ring->start_addr = (void *)txdesc;
	}
	ring->pend_items_count++;
#endif

	

	
	txdesc->cmn_hdr.msg_type = MSG_TYPE_TX_POST;
	txdesc->cmn_hdr.if_id = ifidx;

	txdesc->flags = BCMPCIE_PKT_FLAGS_FRAME_802_3;
	prio = (uint8)PKTPRIO(PKTBUF);


	txdesc->flags |= (prio & 0x7) << BCMPCIE_PKT_FLAGS_PRIO_SHIFT;
	txdesc->seg_cnt = 1;

	txdesc->data_len = htol16((uint16) pktlen);
	txdesc->data_buf_addr.high_addr = htol32(PHYSADDRHI(pa));
	txdesc->data_buf_addr.low_addr  = htol32(PHYSADDRLO(pa));

	
	PKTPUSH(dhd->osh, PKTBUF, ETHER_HDR_LEN);

	
	headroom = (uint16)PKTHEADROOM(dhd->osh, PKTBUF);
	if (prot->tx_metadata_offset && (headroom < prot->tx_metadata_offset)) {
		DHD_ERROR(("No headroom for Metadata tx %d %d\n",
		prot->tx_metadata_offset, headroom));
	}

	if (prot->tx_metadata_offset && (headroom >= prot->tx_metadata_offset)) {
		DHD_TRACE(("Metadata in tx %d\n", prot->tx_metadata_offset));

		
		PKTPUSH(dhd->osh, PKTBUF, prot->tx_metadata_offset);

		if (SECURE_DMA_ENAB(dhd->osh)) {
			meta_pa = SECURE_DMA_MAP_TXMETA(dhd->osh, PKTDATA(dhd->osh, PKTBUF),
				prot->tx_metadata_offset + ETHER_HDR_LEN, DMA_RX, PKTBUF,
				0, ring->dma_buf.secdma);
		} else {
			meta_pa = DMA_MAP(dhd->osh, PKTDATA(dhd->osh, PKTBUF),
				prot->tx_metadata_offset, DMA_RX, PKTBUF, 0);
		}

		if (PHYSADDRISZERO(meta_pa)) {
			DHD_ERROR(("Something really bad, unless 0 is a valid phyaddr\n"));
			ASSERT(0);
		}

		
		PKTPULL(dhd->osh, PKTBUF, prot->tx_metadata_offset);

		txdesc->metadata_buf_len = prot->tx_metadata_offset;
		txdesc->metadata_buf_addr.high_addr = htol32(PHYSADDRHI(meta_pa));
		txdesc->metadata_buf_addr.low_addr = htol32(PHYSADDRLO(meta_pa));
	} else {
		txdesc->metadata_buf_len = htol16(0);
		txdesc->metadata_buf_addr.high_addr = 0;
		txdesc->metadata_buf_addr.low_addr = 0;
	}

#if defined(DHD_PKTID_AUDIT_RING)
	DHD_PKTID_AUDIT(dhd, prot->pktid_map_handle, pktid,
		DHD_DUPLICATE_ALLOC);
#endif 

	txdesc->cmn_hdr.request_id = htol32(pktid);

	DHD_TRACE(("txpost: data_len %d, pktid 0x%04x\n", txdesc->data_len,
		txdesc->cmn_hdr.request_id));

	
#ifdef TXP_FLUSH_NITEMS
	
	
	if ((ring->pend_items_count == prot->txp_threshold) ||
		((uint8 *) txdesc == (uint8 *) DHD_RING_END_VA(ring))) {
		dhd_prot_txdata_write_flush(dhd, flowid, TRUE);
	}
#else
	
	dhd_prot_ring_write_complete(dhd, ring, txdesc, 1);
#endif

	prot->active_tx_count++;

	if (prot->active_tx_count == 1)
		DHD_OS_WAKE_LOCK(dhd);

	DHD_GENERAL_UNLOCK(dhd, flags);

	return BCME_OK;

err_no_res_pktfree:



	DHD_GENERAL_UNLOCK(dhd, flags);
	return BCME_NORESOURCE;
} 

void BCMFASTPATH
dhd_prot_txdata_write_flush(dhd_pub_t *dhd, uint16 flowid, bool in_lock)
{
#ifdef TXP_FLUSH_NITEMS
	unsigned long flags = 0;
	flow_ring_table_t *flow_ring_table;
	flow_ring_node_t *flow_ring_node;
	msgbuf_ring_t *ring;

	if (dhd->flow_ring_table == NULL) {
		return;
	}

	if (!in_lock) {
		DHD_GENERAL_LOCK(dhd, flags);
	}

	flow_ring_table = (flow_ring_table_t *)dhd->flow_ring_table;
	flow_ring_node = (flow_ring_node_t *)&flow_ring_table[flowid];
	ring = (msgbuf_ring_t *)flow_ring_node->prot_info;

	if (ring->pend_items_count) {
		
		dhd_prot_ring_write_complete(dhd, ring, ring->start_addr,
			ring->pend_items_count);
		ring->pend_items_count = 0;
		ring->start_addr = NULL;
	}

	if (!in_lock) {
		DHD_GENERAL_UNLOCK(dhd, flags);
	}
#endif 
}

#undef PKTBUF	

int BCMFASTPATH
dhd_prot_hdrpull(dhd_pub_t *dhd, int *ifidx, void *pkt, uchar *buf, uint *len)
{
	return 0;
}

static void BCMFASTPATH
dhd_prot_return_rxbuf(dhd_pub_t *dhd, uint32 pktid, uint32 rxcnt)
{
	dhd_prot_t *prot = dhd->prot;
#if defined(DHD_LB_RXC)
	int elem_ix;
	uint32 *elem;
	bcm_workq_t *workq;

	workq = &prot->rx_compl_prod;

	
	elem_ix = bcm_ring_prod(WORKQ_RING(workq), DHD_LB_WORKQ_SZ);
	if (elem_ix == BCM_RING_FULL) {
		DHD_ERROR(("%s LB RxCompl workQ is full\n", __FUNCTION__));
		ASSERT(0);
		return;
	}

	elem = WORKQ_ELEMENT(uint32, workq, elem_ix);
	*elem = pktid;

	smp_wmb();

	
	if (++prot->rx_compl_prod_sync >= DHD_LB_WORKQ_SYNC) {
		bcm_workq_prod_sync(workq);
		prot->rx_compl_prod_sync = 0;
	}

	DHD_INFO(("%s: rx_compl_prod pktid<%u> sync<%d>\n",
		__FUNCTION__, pktid, prot->rx_compl_prod_sync));

#endif 


	if (prot->rxbufpost >= rxcnt) {
		prot->rxbufpost -= rxcnt;
	} else {
		
		prot->rxbufpost = 0;
	}

#if !defined(DHD_LB_RXC)
	if (prot->rxbufpost <= (prot->max_rxbufpost - RXBUFPOST_THRESHOLD)) {
		dhd_msgbuf_rxbuf_post(dhd, FALSE); 
	}
#endif 
}

static void
dhd_prot_wlioctl_intercept(dhd_pub_t *dhd, wl_ioctl_t * ioc, void * buf)
{
	dhd_prot_t *prot = dhd->prot;

	if (ioc->cmd == WLC_SET_VAR && buf != NULL && !strcmp(buf, "pcie_bus_tput")) {
		int slen = 0;
		pcie_bus_tput_params_t *tput_params;

		slen = strlen("pcie_bus_tput") + 1;
		tput_params = (pcie_bus_tput_params_t*)((char *)buf + slen);
		bcopy(&prot->host_bus_throughput_buf.pa, &tput_params->host_buf_addr,
			sizeof(tput_params->host_buf_addr));
		tput_params->host_buf_len = DHD_BUS_TPUT_BUF_LEN;
	}
}


int dhd_prot_ioctl(dhd_pub_t *dhd, int ifidx, wl_ioctl_t * ioc, void * buf, int len)
{
	int ret = -1;
	uint8 action;

	if ((dhd->busstate == DHD_BUS_DOWN) || dhd->hang_was_sent) {
		DHD_ERROR(("%s : bus is down. we have nothing to do\n", __FUNCTION__));
		goto done;
	}

	if (dhd->busstate == DHD_BUS_SUSPEND) {
		DHD_ERROR(("%s : bus is suspended\n", __FUNCTION__));
		goto done;
	}

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));


	ASSERT(len <= WLC_IOCTL_MAXLEN);

	if (len > WLC_IOCTL_MAXLEN) {
		goto done;
	}

	action = ioc->set;

	dhd_prot_wlioctl_intercept(dhd, ioc, buf);

	if (action & WL_IOCTL_ACTION_SET) {
		ret = dhd_msgbuf_set_ioctl(dhd, ifidx, ioc->cmd, buf, len, action);
	} else {
		ret = dhd_msgbuf_query_ioctl(dhd, ifidx, ioc->cmd, buf, len, action);
		if (ret > 0) {
			ioc->used = ret;
		}
	}

	
	if (ret >= 0) {
		ret = 0;
	} else {
#ifndef DETAIL_DEBUG_LOG_FOR_IOCTL
		DHD_ERROR(("%s: status ret value is %d \n", __FUNCTION__, ret));
#endif 
		dhd->dongle_error = ret;
	}

	if (!ret && ioc->cmd == WLC_SET_VAR && buf != NULL) {
		
		if (!strcmp(buf, "wme_dp")) {
			int slen, val = 0;

			slen = strlen("wme_dp") + 1;
			if (len >= (int)(slen + sizeof(int))) {
				bcopy(((char *)buf + slen), &val, sizeof(int));
			}
			dhd->wme_dp = (uint8) ltoh32(val);
		}

	}

done:
	return ret;

} 


int
dhdmsgbuf_lpbk_req(dhd_pub_t *dhd, uint len)
{
	unsigned long flags;
	dhd_prot_t *prot = dhd->prot;
	uint16 alloced = 0;

	ioct_reqst_hdr_t *ioct_rqst;

	uint16 hdrlen = sizeof(ioct_reqst_hdr_t);
	uint16 msglen = len + hdrlen;
	msgbuf_ring_t *ring = &prot->h2dring_ctrl_subn;

	msglen = ALIGN_SIZE(msglen, DMA_ALIGN_LEN);
	msglen = LIMIT_TO_MAX(msglen, MSGBUF_MAX_MSG_SIZE);

	DHD_GENERAL_LOCK(dhd, flags);

	ioct_rqst = (ioct_reqst_hdr_t *)
		dhd_prot_alloc_ring_space(dhd, ring, 1, &alloced, FALSE);

	if (ioct_rqst == NULL) {
		DHD_GENERAL_UNLOCK(dhd, flags);
		return 0;
	}

	{
		uint8 *ptr;
		uint16 i;

		ptr = (uint8 *)ioct_rqst;
		for (i = 0; i < msglen; i++) {
			ptr[i] = i % 256;
		}
	}

	
	ioct_rqst->msg.epoch = ring->seqnum % H2D_EPOCH_MODULO;
	ring->seqnum++;

	ioct_rqst->msg.msg_type = MSG_TYPE_LOOPBACK;
	ioct_rqst->msg.if_id = 0;

	bcm_print_bytes("LPBK REQ: ", (uint8 *)ioct_rqst, msglen);

	
	dhd_prot_ring_write_complete(dhd, ring, ioct_rqst, 1);
	DHD_GENERAL_UNLOCK(dhd, flags);

	return 0;
}

void dmaxfer_free_dmaaddr(dhd_pub_t *dhd, dhd_dmaxfer_t *dmaxfer)
{
	if (dmaxfer == NULL) {
		return;
	}

	dhd_dma_buf_free(dhd, &dmaxfer->srcmem);
	dhd_dma_buf_free(dhd, &dmaxfer->dstmem);
}

int dmaxfer_prepare_dmaaddr(dhd_pub_t *dhd, uint len,
	uint srcdelay, uint destdelay, dhd_dmaxfer_t *dmaxfer)
{
	uint i;
	if (!dmaxfer) {
		return BCME_ERROR;
	}

	
	dmaxfer_free_dmaaddr(dhd, dmaxfer);

	if (dhd_dma_buf_alloc(dhd, &dmaxfer->srcmem, len)) {
		return BCME_NOMEM;
	}

	if (dhd_dma_buf_alloc(dhd, &dmaxfer->dstmem, len + 8)) {
		dhd_dma_buf_free(dhd, &dmaxfer->srcmem);
		return BCME_NOMEM;
	}

	dmaxfer->len = len;

	
	for (i = 0; i < dmaxfer->len; i++) {
		((uint8*)dmaxfer->srcmem.va)[i] = i % 256;
	}
	OSL_CACHE_FLUSH(dmaxfer->srcmem.va, dmaxfer->len);

	dmaxfer->srcdelay = srcdelay;
	dmaxfer->destdelay = destdelay;

	return BCME_OK;
} 

static void
dhd_msgbuf_dmaxfer_process(dhd_pub_t *dhd, void *msg)
{
	dhd_prot_t *prot = dhd->prot;

	OSL_CACHE_INV(prot->dmaxfer.dstmem.va, prot->dmaxfer.len);
	if (prot->dmaxfer.srcmem.va && prot->dmaxfer.dstmem.va) {
		if (memcmp(prot->dmaxfer.srcmem.va,
		        prot->dmaxfer.dstmem.va, prot->dmaxfer.len)) {
			bcm_print_bytes("XFER SRC: ",
			    prot->dmaxfer.srcmem.va, prot->dmaxfer.len);
			bcm_print_bytes("XFER DST: ",
			    prot->dmaxfer.dstmem.va, prot->dmaxfer.len);
		} else {
			DHD_INFO(("DMA successful\n"));
		}
	}
	dmaxfer_free_dmaaddr(dhd, &prot->dmaxfer);
	dhd->prot->dmaxfer.in_progress = FALSE;
}

int
dhdmsgbuf_dmaxfer_req(dhd_pub_t *dhd, uint len, uint srcdelay, uint destdelay)
{
	unsigned long flags;
	int ret = BCME_OK;
	dhd_prot_t *prot = dhd->prot;
	pcie_dma_xfer_params_t *dmap;
	uint32 xferlen = LIMIT_TO_MAX(len, DMA_XFER_LEN_LIMIT);
	uint16 alloced = 0;
	msgbuf_ring_t *ring = &prot->h2dring_ctrl_subn;

	if (prot->dmaxfer.in_progress) {
		DHD_ERROR(("DMA is in progress...\n"));
		return ret;
	}

	prot->dmaxfer.in_progress = TRUE;
	if ((ret = dmaxfer_prepare_dmaaddr(dhd, xferlen, srcdelay, destdelay,
	        &prot->dmaxfer)) != BCME_OK) {
		prot->dmaxfer.in_progress = FALSE;
		return ret;
	}

	DHD_GENERAL_LOCK(dhd, flags);

	dmap = (pcie_dma_xfer_params_t *)
		dhd_prot_alloc_ring_space(dhd, ring, 1, &alloced, FALSE);

	if (dmap == NULL) {
		dmaxfer_free_dmaaddr(dhd, &prot->dmaxfer);
		prot->dmaxfer.in_progress = FALSE;
		DHD_GENERAL_UNLOCK(dhd, flags);
		return BCME_NOMEM;
	}

	
	dmap->cmn_hdr.msg_type = MSG_TYPE_LPBK_DMAXFER;
	dmap->cmn_hdr.request_id = htol32(DHD_FAKE_PKTID);
	dmap->cmn_hdr.epoch = ring->seqnum % H2D_EPOCH_MODULO;
	ring->seqnum++;

	dmap->host_input_buf_addr.high = htol32(PHYSADDRHI(prot->dmaxfer.srcmem.pa));
	dmap->host_input_buf_addr.low = htol32(PHYSADDRLO(prot->dmaxfer.srcmem.pa));
	dmap->host_ouput_buf_addr.high = htol32(PHYSADDRHI(prot->dmaxfer.dstmem.pa));
	dmap->host_ouput_buf_addr.low = htol32(PHYSADDRLO(prot->dmaxfer.dstmem.pa));
	dmap->xfer_len = htol32(prot->dmaxfer.len);
	dmap->srcdelay = htol32(prot->dmaxfer.srcdelay);
	dmap->destdelay = htol32(prot->dmaxfer.destdelay);

	
	dhd_prot_ring_write_complete(dhd, ring, dmap, 1);
	DHD_GENERAL_UNLOCK(dhd, flags);

	DHD_ERROR(("DMA Started...\n"));

	return BCME_OK;
} 

static int
dhd_msgbuf_query_ioctl(dhd_pub_t *dhd, int ifidx, uint cmd, void *buf, uint len, uint8 action)
{
	int ret = 0;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	
	if (cmd == WLC_GET_VAR && buf)
	{
		if (!strcmp((char *)buf, "bcmerrorstr"))
		{
			strncpy((char *)buf, bcmerrorstr(dhd->dongle_error), BCME_STRLEN);
			goto done;
		}
		else if (!strcmp((char *)buf, "bcmerror"))
		{
			*(int *)buf = dhd->dongle_error;
			goto done;
		}
	}

	ret = dhd_fillup_ioct_reqst(dhd, (uint16)len, cmd, buf, ifidx);

	DHD_CTL(("query_ioctl: ACTION %d ifdix %d cmd %d len %d \n",
		action, ifidx, cmd, len));

	
	ret = dhd_msgbuf_wait_ioctl_cmplt(dhd, len, buf);

done:
	return ret;
}

static int
dhd_msgbuf_wait_ioctl_cmplt(dhd_pub_t *dhd, uint32 len, void *buf)
{
	dhd_prot_t *prot = dhd->prot;
	int timeleft;
	unsigned long flags;
	int ret = 0;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	if (dhd->dongle_reset) {
		ret = -EIO;
		goto out;
	}

	if (prot->cur_ioctlresp_bufs_posted) {
		prot->cur_ioctlresp_bufs_posted--;
	}

	dhd_msgbuf_rxbuf_post_ioctlresp_bufs(dhd);

	timeleft = dhd_os_ioctl_resp_wait(dhd, &prot->ioctl_received);
	if (timeleft == 0) {
		dhd->rxcnt_timeout++;
		dhd->rx_ctlerrs++;

#if defined(DHD_FW_COREDUMP)
		
		
		if (dhd->memdump_enabled && !dhd->dongle_trap_occured) {
			
			DHD_OS_WAKE_LOCK(dhd);
			dhd->memdump_type = DUMP_TYPE_RESUMED_ON_TIMEOUT;
			dhd_bus_mem_dump(dhd);
			DHD_OS_WAKE_UNLOCK(dhd);
		}
#endif 
		if (dhd->rxcnt_timeout >= MAX_CNTL_RX_TIMEOUT) {
#ifdef SUPPORT_LINKDOWN_RECOVERY
#ifdef CONFIG_ARCH_MSM
			dhd->bus->islinkdown = 1;
#endif 
#endif 
			DHD_ERROR(("%s: timeout > MAX_CNTL_RX_TIMEOUT\n", __FUNCTION__));
		}
		ret = -ETIMEDOUT;
		goto out;
	} else {
		if (prot->ioctl_received != IOCTL_RETURN_ON_SUCCESS) {
			DHD_ERROR(("%s: IOCTL failure due to ioctl_received = %d\n",
				__FUNCTION__, prot->ioctl_received));
			ret = -ECONNABORTED;
			goto out;
		}
		dhd->rxcnt_timeout = 0;
		dhd->rx_ctlpkts++;
		DHD_CTL(("%s: ioctl resp resumed, got %d\n",
			__FUNCTION__, prot->ioctl_resplen));
	}

	if (dhd->dongle_trap_occured) {
#ifdef SUPPORT_LINKDOWN_RECOVERY
#ifdef CONFIG_ARCH_MSM
		dhd->bus->islinkdown = 1;
#endif 
#endif 
		DHD_ERROR(("%s: TRAP occurred!!\n", __FUNCTION__));
		ret = -EREMOTEIO;
		goto out;
	}

	if (dhd->prot->ioctl_resplen > len) {
		dhd->prot->ioctl_resplen = (uint16)len;
	}
	if (buf) {
		bcopy(dhd->prot->retbuf.va, buf, dhd->prot->ioctl_resplen);
	}

	ret = (int)(dhd->prot->ioctl_status);
out:
	DHD_GENERAL_LOCK(dhd, flags);
	dhd->prot->ioctl_state &= ~MSGBUF_IOCTL_RESP_PENDING;
	dhd->prot->ioctl_resplen = 0;
	dhd->prot->ioctl_received = IOCTL_WAIT;
	DHD_GENERAL_UNLOCK(dhd, flags);

	return ret;
} 

static int
dhd_msgbuf_set_ioctl(dhd_pub_t *dhd, int ifidx, uint cmd, void *buf, uint len, uint8 action)
{
	int ret = 0;

	DHD_TRACE(("%s: Enter \n", __FUNCTION__));

	if (dhd->busstate == DHD_BUS_DOWN) {
		DHD_ERROR(("%s : bus is down. we have nothing to do\n", __FUNCTION__));
		return -EIO;
	}

	
	if (dhd->hang_was_sent) {
		DHD_ERROR(("%s: HANG was sent up earlier. Not talking to the chip\n",
			__FUNCTION__));
		return -EIO;
	}

	
	ret = dhd_fillup_ioct_reqst(dhd, (uint16)len, cmd, buf, ifidx);

	DHD_CTL(("ACTION %d ifdix %d cmd %d len %d \n",
		action, ifidx, cmd, len));

	ret = dhd_msgbuf_wait_ioctl_cmplt(dhd, len, buf);

	return ret;
}

int dhd_prot_ctl_complete(dhd_pub_t *dhd)
{
	return 0;
}

int dhd_prot_iovar_op(dhd_pub_t *dhd, const char *name,
	void *params, int plen, void *arg, int len, bool set)
{
	return BCME_UNSUPPORTED;
}

void dhd_prot_dump(dhd_pub_t *dhd, struct bcmstrbuf *b)
{

#if defined(PCIE_D2H_SYNC)
	if (dhd->d2h_sync_mode & PCIE_SHARED_D2H_SYNC_SEQNUM)
		bcm_bprintf(b, "\nd2h_sync: SEQNUM:");
	else if (dhd->d2h_sync_mode & PCIE_SHARED_D2H_SYNC_XORCSUM)
		bcm_bprintf(b, "\nd2h_sync: XORCSUM:");
	else
		bcm_bprintf(b, "\nd2h_sync: NONE:");
	bcm_bprintf(b, " d2h_sync_wait max<%lu> tot<%lu>\n",
		dhd->prot->d2h_sync_wait_max, dhd->prot->d2h_sync_wait_tot);
#endif  

	bcm_bprintf(b, "\nDongle DMA Indices: h2d %d  d2h %d index size %d bytes\n",
		DMA_INDX_ENAB(dhd->dma_h2d_ring_upd_support),
		DMA_INDX_ENAB(dhd->dma_d2h_ring_upd_support),
		dhd->prot->rw_index_sz);
}

void dhd_prot_dstats(dhd_pub_t *dhd)
{
	return;
}

int dhd_process_pkt_reorder_info(dhd_pub_t *dhd, uchar *reorder_info_buf,
	uint reorder_info_len, void **pkt, uint32 *free_buf_count)
{
	return 0;
}

int
dhd_post_dummy_msg(dhd_pub_t *dhd)
{
	unsigned long flags;
	hostevent_hdr_t *hevent = NULL;
	uint16 alloced = 0;

	dhd_prot_t *prot = dhd->prot;
	msgbuf_ring_t *ring = &prot->h2dring_ctrl_subn;

	DHD_GENERAL_LOCK(dhd, flags);

	hevent = (hostevent_hdr_t *)
		dhd_prot_alloc_ring_space(dhd, ring, 1, &alloced, FALSE);

	if (hevent == NULL) {
		DHD_GENERAL_UNLOCK(dhd, flags);
		return -1;
	}

	
	hevent->msg.epoch = ring->seqnum % H2D_EPOCH_MODULO;
	ring->seqnum++;
	hevent->msg.msg_type = MSG_TYPE_HOST_EVNT;
	hevent->msg.if_id = 0;

	
	hevent->evnt_pyld = htol32(HOST_EVENT_CONS_CMD);

	dhd_prot_ring_write_complete(dhd, ring, hevent, 1);
	DHD_GENERAL_UNLOCK(dhd, flags);

	return 0;
}

static void * BCMFASTPATH
dhd_prot_alloc_ring_space(dhd_pub_t *dhd, msgbuf_ring_t *ring,
	uint16 nitems, uint16 * alloced, bool exactly_nitems)
{
	void * ret_buf;

	
	ret_buf = dhd_prot_get_ring_space(ring, nitems, alloced, exactly_nitems);

	if (ret_buf == NULL) {
		
		if (DMA_INDX_ENAB(dhd->dma_d2h_ring_upd_support)) {
			ring->rd = dhd_prot_dma_indx_get(dhd, H2D_DMA_INDX_RD_UPD, ring->idx);
		} else {
			dhd_bus_cmn_readshared(dhd->bus, &(ring->rd), RING_RD_UPD, ring->idx);
		}

		
		ret_buf = dhd_prot_get_ring_space(ring, nitems, alloced, exactly_nitems);

		if (ret_buf == NULL) {
			DHD_INFO(("%s: Ring space not available  \n", ring->name));
			return NULL;
		}
	}

	
	return ret_buf;
}

static int
dhd_fillup_ioct_reqst(dhd_pub_t *dhd, uint16 len, uint cmd, void* buf, int ifidx)
{
	dhd_prot_t *prot = dhd->prot;
	ioctl_req_msg_t *ioct_rqst;
	void * ioct_buf;	
	uint16  rqstlen, resplen;
	unsigned long flags;
	uint16 alloced = 0;
	msgbuf_ring_t *ring = &prot->h2dring_ctrl_subn;

	rqstlen = len;
	resplen = len;

	
	
	
	
	rqstlen = MIN(rqstlen, MSGBUF_MAX_MSG_SIZE);

	DHD_GENERAL_LOCK(dhd, flags);

	if (prot->ioctl_state) {
		DHD_CTL(("pending ioctl %02x\n", prot->ioctl_state));
		DHD_GENERAL_UNLOCK(dhd, flags);
		return BCME_BUSY;
	} else {
		prot->ioctl_state = MSGBUF_IOCTL_ACK_PENDING | MSGBUF_IOCTL_RESP_PENDING;
	}

	
	ioct_rqst = (ioctl_req_msg_t*)
		dhd_prot_alloc_ring_space(dhd, ring, 1, &alloced, FALSE);
	if (ioct_rqst == NULL) {
		DHD_ERROR(("couldn't allocate space on msgring to send ioctl request\n"));
		prot->ioctl_state = 0;
		prot->ioctl_received = IOCTL_WAIT;
		DHD_GENERAL_UNLOCK(dhd, flags);
		return -1;
	}

	
	ioct_rqst->cmn_hdr.msg_type = MSG_TYPE_IOCTLPTR_REQ;
	ioct_rqst->cmn_hdr.if_id = (uint8)ifidx;
	ioct_rqst->cmn_hdr.flags = 0;
	ioct_rqst->cmn_hdr.request_id = htol32(DHD_IOCTL_REQ_PKTID);
	ioct_rqst->cmn_hdr.epoch = ring->seqnum % H2D_EPOCH_MODULO;
	ring->seqnum++;

	ioct_rqst->cmd = htol32(cmd);
	ioct_rqst->output_buf_len = htol16(resplen);
	prot->ioctl_trans_id++;
	ioct_rqst->trans_id = prot->ioctl_trans_id;

	
	ioct_rqst->input_buf_len = htol16(rqstlen);
	ioct_rqst->host_input_buf_addr.high = htol32(PHYSADDRHI(prot->ioctbuf.pa));
	ioct_rqst->host_input_buf_addr.low = htol32(PHYSADDRLO(prot->ioctbuf.pa));
	
	ioct_buf = (void *) prot->ioctbuf.va;

	if (buf) {
		memcpy(ioct_buf, buf, len);
	}

	OSL_CACHE_FLUSH((void *) prot->ioctbuf.va, len);

	if (!ISALIGNED(ioct_buf, DMA_ALIGN_LEN)) {
		DHD_ERROR(("host ioct address unaligned !!!!! \n"));
	}

	DHD_CTL(("submitted IOCTL request request_id %d, cmd %d, output_buf_len %d, tx_id %d\n",
		ioct_rqst->cmn_hdr.request_id, cmd, ioct_rqst->output_buf_len,
		ioct_rqst->trans_id));

	
	dhd_prot_ring_write_complete(dhd, ring, ioct_rqst, 1);
	DHD_GENERAL_UNLOCK(dhd, flags);

	return 0;
} 


static int
dhd_prot_ring_attach(dhd_pub_t *dhd, msgbuf_ring_t *ring, const char *name,
	uint16 max_items, uint16 item_len, uint16 ringid)
{
	int dma_buf_alloced = BCME_NOMEM;
	uint32 dma_buf_len = max_items * item_len;
	dhd_prot_t *prot = dhd->prot;

	ASSERT(ring);
	ASSERT(name);
	ASSERT((max_items < 0xFFFF) && (item_len < 0xFFFF) && (ringid < 0xFFFF));

	
	strncpy(ring->name, name, RING_NAME_MAX_LENGTH);
	ring->name[RING_NAME_MAX_LENGTH - 1] = '\0';

	ring->idx = ringid;

	ring->max_items = max_items;
	ring->item_len = item_len;

	
	if (DHD_IS_FLOWRING(ringid) && (prot->flowrings_dma_buf.va)) {
		
		uint16 flowid;
		uint32 base_offset;

		dhd_dma_buf_t *dma_buf = &ring->dma_buf;
		dhd_dma_buf_t *rsv_buf = &prot->flowrings_dma_buf;

		flowid = DHD_RINGID_TO_FLOWID(ringid);
		base_offset = (flowid - BCMPCIE_H2D_COMMON_MSGRINGS) * dma_buf_len;

		ASSERT(base_offset + dma_buf_len <= rsv_buf->len);

		dma_buf->len = dma_buf_len;
		dma_buf->va = (void *)((uintptr)rsv_buf->va + base_offset);
		PHYSADDRHISET(dma_buf->pa, PHYSADDRHI(rsv_buf->pa));
		PHYSADDRLOSET(dma_buf->pa, PHYSADDRLO(rsv_buf->pa) + base_offset);

		
		ASSERT(PHYSADDRLO(dma_buf->pa) >= PHYSADDRLO(rsv_buf->pa));

		dma_buf->dmah   = rsv_buf->dmah;
		dma_buf->secdma = rsv_buf->secdma;

		(void)dhd_dma_buf_audit(dhd, &ring->dma_buf);
	} else {
		
		dma_buf_alloced = dhd_dma_buf_alloc(dhd, &ring->dma_buf, dma_buf_len);
		if (dma_buf_alloced != BCME_OK) {
			return BCME_NOMEM;
		}
	}

	
	dhd_base_addr_htolpa(&ring->base_addr, ring->dma_buf.pa);

#ifdef BCM_SECURE_DMA
	if (SECURE_DMA_ENAB(prot->osh)) {
		ring->dma_buf.secdma = MALLOCZ(prot->osh, sizeof(sec_cma_info_t));
		if (ring->dma_buf.secdma == NULL) {
			goto free_dma_buf;
		}
	}
#endif 

	DHD_INFO(("RING_ATTACH : %s Max item %d len item %d total size %d "
		"ring start %p buf phys addr  %x:%x \n",
		ring->name, ring->max_items, ring->item_len,
		dma_buf_len, ring->dma_buf.va, ltoh32(ring->base_addr.high_addr),
		ltoh32(ring->base_addr.low_addr)));

	return BCME_OK;

#ifdef BCM_SECURE_DMA
free_dma_buf:
	if (dma_buf_alloced == BCME_OK) {
		dhd_dma_buf_free(dhd, &ring->dma_buf);
	}
#endif 

	return BCME_NOMEM;

} 


static void
dhd_prot_ring_init(dhd_pub_t *dhd, msgbuf_ring_t *ring)
{
	ring->wr = 0;
	ring->rd = 0;

	
	dhd_bus_cmn_writeshared(dhd->bus, &ring->base_addr,
		sizeof(sh_addr_t), RING_BUF_ADDR, ring->idx);
	dhd_bus_cmn_writeshared(dhd->bus, &ring->max_items,
		sizeof(uint16), RING_MAX_ITEMS, ring->idx);
	dhd_bus_cmn_writeshared(dhd->bus, &ring->item_len,
		sizeof(uint16), RING_ITEM_LEN, ring->idx);

	dhd_bus_cmn_writeshared(dhd->bus, &(ring->wr),
		sizeof(uint16), RING_WR_UPD, ring->idx);
	dhd_bus_cmn_writeshared(dhd->bus, &(ring->rd),
		sizeof(uint16), RING_RD_UPD, ring->idx);

	
	ring->inited = TRUE;

} 


static void
dhd_prot_ring_reset(dhd_pub_t *dhd, msgbuf_ring_t *ring)
{
	DHD_TRACE(("%s\n", __FUNCTION__));

	dhd_dma_buf_reset(dhd, &ring->dma_buf);

	ring->rd = ring->wr = 0;
}


static void
dhd_prot_ring_detach(dhd_pub_t *dhd, msgbuf_ring_t *ring)
{
	dhd_prot_t *prot = dhd->prot;
	ASSERT(ring);

	ring->inited = FALSE;
	

#ifdef BCM_SECURE_DMA
	if (SECURE_DMA_ENAB(prot->osh)) {
		if (ring->dma_buf.secdma) {
			SECURE_DMA_UNMAP_ALL(prot->osh, ring->dma_buf.secdma);
			MFREE(prot->osh, ring->dma_buf.secdma, sizeof(sec_cma_info_t));
			ring->dma_buf.secdma = NULL;
		}
	}
#endif 

	if (DHD_IS_FLOWRING(ring->idx) && (prot->flowrings_dma_buf.va)) {
		(void)dhd_dma_buf_audit(dhd, &ring->dma_buf);
		memset(&ring->dma_buf, 0, sizeof(dhd_dma_buf_t));
	} else {
		dhd_dma_buf_free(dhd, &ring->dma_buf);
	}

} 



#define DHD_FLOWRINGS_POOL_TOTAL(h2d_rings_total) \
	((h2d_rings_total) - BCMPCIE_H2D_COMMON_MSGRINGS)

#define DHD_FLOWRINGS_POOL_OFFSET(flowid) \
	((flowid) - BCMPCIE_H2D_COMMON_MSGRINGS)

#define DHD_RING_IN_FLOWRINGS_POOL(prot, flowid) \
	(msgbuf_ring_t*)((prot)->h2d_flowrings_pool) + DHD_FLOWRINGS_POOL_OFFSET(flowid)

#define FOREACH_RING_IN_FLOWRINGS_POOL(prot, ring, flowid) \
	for ((flowid) = DHD_FLOWRING_START_FLOWID, \
		 (ring) = DHD_RING_IN_FLOWRINGS_POOL(prot, flowid); \
		 (flowid) < (prot)->h2d_rings_total; \
		 (flowid)++, (ring)++)

static int
dhd_prot_flowrings_pool_attach(dhd_pub_t *dhd)
{
	uint16 flowid;
	msgbuf_ring_t *ring;
	uint16 h2d_flowrings_total; 
	dhd_prot_t *prot = dhd->prot;
	char ring_name[RING_NAME_MAX_LENGTH];

	if (prot->h2d_flowrings_pool != NULL) {
		return BCME_OK; 
	}

	ASSERT(prot->h2d_rings_total == 0);

	
	prot->h2d_rings_total = (uint16)dhd_bus_max_h2d_queues(dhd->bus);

	if (prot->h2d_rings_total < BCMPCIE_H2D_COMMON_MSGRINGS) {
		DHD_ERROR(("%s: h2d_rings_total advertized as %u\n",
			__FUNCTION__, prot->h2d_rings_total));
		return BCME_ERROR;
	}

	
	h2d_flowrings_total = DHD_FLOWRINGS_POOL_TOTAL(prot->h2d_rings_total);

	DHD_ERROR(("Attach flowrings pool for %d rings\n", h2d_flowrings_total));

	
	prot->h2d_flowrings_pool = (msgbuf_ring_t *)MALLOCZ(prot->osh,
		(h2d_flowrings_total * sizeof(msgbuf_ring_t)));

	if (prot->h2d_flowrings_pool == NULL) {
		DHD_ERROR(("%s: flowrings pool for %d flowrings, alloc failure\n",
			__FUNCTION__, h2d_flowrings_total));
		goto fail;
	}

	
	FOREACH_RING_IN_FLOWRINGS_POOL(prot, ring, flowid) {
		snprintf(ring_name, sizeof(ring_name), "h2dflr_%03u", flowid);
		ring_name[RING_NAME_MAX_LENGTH - 1] = '\0';
		if (dhd_prot_ring_attach(dhd, ring, ring_name,
		        H2DRING_TXPOST_MAX_ITEM, H2DRING_TXPOST_ITEMSIZE,
		        DHD_FLOWID_TO_RINGID(flowid)) != BCME_OK) {
			goto attach_fail;
		}
	}

	return BCME_OK;

attach_fail:
	dhd_prot_flowrings_pool_detach(dhd); 

fail:
	prot->h2d_rings_total = 0;
	return BCME_NOMEM;

} 


static void
dhd_prot_flowrings_pool_reset(dhd_pub_t *dhd)
{
	uint16 flowid;
	msgbuf_ring_t *ring;
	dhd_prot_t *prot = dhd->prot;

	if (prot->h2d_flowrings_pool == NULL) {
		ASSERT(prot->h2d_rings_total == 0);
		return;
	}

	
	FOREACH_RING_IN_FLOWRINGS_POOL(prot, ring, flowid) {
		dhd_prot_ring_reset(dhd, ring);
		ring->inited = FALSE;
	}

	
}


static void
dhd_prot_flowrings_pool_detach(dhd_pub_t *dhd)
{
	int flowid;
	msgbuf_ring_t *ring;
	int h2d_flowrings_total; 
	dhd_prot_t *prot = dhd->prot;

	if (prot->h2d_flowrings_pool == NULL) {
		ASSERT(prot->h2d_rings_total == 0);
		return;
	}

	
	FOREACH_RING_IN_FLOWRINGS_POOL(prot, ring, flowid) {
		dhd_prot_ring_detach(dhd, ring);
	}

	h2d_flowrings_total = DHD_FLOWRINGS_POOL_TOTAL(prot->h2d_rings_total);

	MFREE(prot->osh, prot->h2d_flowrings_pool,
		(h2d_flowrings_total * sizeof(msgbuf_ring_t)));

	prot->h2d_flowrings_pool = (msgbuf_ring_t*)NULL;
	prot->h2d_rings_total = 0;

} 


static msgbuf_ring_t *
dhd_prot_flowrings_pool_fetch(dhd_pub_t *dhd, uint16 flowid)
{
	msgbuf_ring_t *ring;
	dhd_prot_t *prot = dhd->prot;

	ASSERT(flowid >= DHD_FLOWRING_START_FLOWID);
	ASSERT(flowid < prot->h2d_rings_total);
	ASSERT(prot->h2d_flowrings_pool != NULL);

	ring = DHD_RING_IN_FLOWRINGS_POOL(prot, flowid);

	

	ring->wr = 0;
	ring->rd = 0;
	ring->inited = TRUE;

	return ring;
}


void
dhd_prot_flowrings_pool_release(dhd_pub_t *dhd, uint16 flowid, void *flow_ring)
{
	msgbuf_ring_t *ring;
	dhd_prot_t *prot = dhd->prot;

	ASSERT(flowid >= DHD_FLOWRING_START_FLOWID);
	ASSERT(flowid < prot->h2d_rings_total);
	ASSERT(prot->h2d_flowrings_pool != NULL);

	ring = DHD_RING_IN_FLOWRINGS_POOL(prot, flowid);

	ASSERT(ring == (msgbuf_ring_t*)flow_ring);
	

	(void)dhd_dma_buf_audit(dhd, &ring->dma_buf);

	ring->wr = 0;
	ring->rd = 0;
	ring->inited = FALSE;
}


static void *BCMFASTPATH
dhd_prot_get_ring_space(msgbuf_ring_t *ring, uint16 nitems, uint16 * alloced,
	bool exactly_nitems)
{
	void *ret_ptr = NULL;
	uint16 ring_avail_cnt;

	ASSERT(nitems <= ring->max_items);

	ring_avail_cnt = CHECK_WRITE_SPACE(ring->rd, ring->wr, ring->max_items);

	if ((ring_avail_cnt == 0) ||
	       (exactly_nitems && (ring_avail_cnt < nitems) &&
	       ((ring->max_items - ring->wr) >= nitems))) {
		DHD_INFO(("Space not available: ring %s items %d write %d read %d\n",
			ring->name, nitems, ring->wr, ring->rd));
		return NULL;
	}
	*alloced = MIN(nitems, ring_avail_cnt);

	
	ret_ptr = (char *)DHD_RING_BGN_VA(ring) + (ring->wr * ring->item_len);

	
	if ((ring->wr + *alloced) == ring->max_items) {
		ring->wr = 0;
	} else if ((ring->wr + *alloced) < ring->max_items) {
		ring->wr += *alloced;
	} else {
		
		ASSERT(0);
		return NULL;
	}

	return ret_ptr;
} 


static void BCMFASTPATH
dhd_prot_ring_write_complete(dhd_pub_t *dhd, msgbuf_ring_t * ring, void* p,
	uint16 nitems)
{
	dhd_prot_t *prot = dhd->prot;

	
	OSL_CACHE_FLUSH(p, ring->item_len * nitems);

	if (DMA_INDX_ENAB(dhd->dma_h2d_ring_upd_support)) {
		dhd_prot_dma_indx_set(dhd, ring->wr,
			H2D_DMA_INDX_WR_UPD, ring->idx);
	} else {
		dhd_bus_cmn_writeshared(dhd->bus, &(ring->wr),
			sizeof(uint16), RING_WR_UPD, ring->idx);
	}

	
	prot->mb_ring_fn(dhd->bus, ring->wr);
}


static void
dhd_prot_upd_read_idx(dhd_pub_t *dhd, msgbuf_ring_t * ring)
{
	
	if (DMA_INDX_ENAB(dhd->dma_h2d_ring_upd_support)) {
		dhd_prot_dma_indx_set(dhd, ring->rd,
			D2H_DMA_INDX_RD_UPD, ring->idx);
	} else {
		dhd_bus_cmn_writeshared(dhd->bus, &(ring->rd),
			sizeof(uint16), RING_RD_UPD, ring->idx);
	}
}


static void
dhd_prot_dma_indx_set(dhd_pub_t *dhd, uint16 new_index, uint8 type, uint16 ringid)
{
	uint8 *ptr;
	uint16 offset;
	dhd_prot_t *prot = dhd->prot;

	switch (type) {
		case H2D_DMA_INDX_WR_UPD:
			ptr = (uint8 *)(prot->h2d_dma_indx_wr_buf.va);
			offset = DHD_H2D_RING_OFFSET(ringid);
			break;

		case D2H_DMA_INDX_RD_UPD:
			ptr = (uint8 *)(prot->d2h_dma_indx_rd_buf.va);
			offset = DHD_D2H_RING_OFFSET(ringid);
			break;

		default:
			DHD_ERROR(("%s: Invalid option for DMAing read/write index\n",
				__FUNCTION__));
			return;
	}

	ASSERT(prot->rw_index_sz != 0);
	ptr += offset * prot->rw_index_sz;

	*(uint16*)ptr = htol16(new_index);

	OSL_CACHE_FLUSH((void *)ptr, prot->rw_index_sz);

	DHD_TRACE(("%s: data %d type %d ringid %d ptr 0x%p offset %d\n",
		__FUNCTION__, new_index, type, ringid, ptr, offset));

} 


static uint16
dhd_prot_dma_indx_get(dhd_pub_t *dhd, uint8 type, uint16 ringid)
{
	uint8 *ptr;
	uint16 data;
	uint16 offset;
	dhd_prot_t *prot = dhd->prot;

	switch (type) {
		case H2D_DMA_INDX_WR_UPD:
			ptr = (uint8 *)(prot->h2d_dma_indx_wr_buf.va);
			offset = DHD_H2D_RING_OFFSET(ringid);
			break;

		case H2D_DMA_INDX_RD_UPD:
			ptr = (uint8 *)(prot->h2d_dma_indx_rd_buf.va);
			offset = DHD_H2D_RING_OFFSET(ringid);
			break;

		case D2H_DMA_INDX_WR_UPD:
			ptr = (uint8 *)(prot->d2h_dma_indx_wr_buf.va);
			offset = DHD_D2H_RING_OFFSET(ringid);
			break;

		case D2H_DMA_INDX_RD_UPD:
			ptr = (uint8 *)(prot->d2h_dma_indx_rd_buf.va);
			offset = DHD_D2H_RING_OFFSET(ringid);
			break;

		default:
			DHD_ERROR(("%s: Invalid option for DMAing read/write index\n",
				__FUNCTION__));
			return 0;
	}

	ASSERT(prot->rw_index_sz != 0);
	ptr += offset * prot->rw_index_sz;

	OSL_CACHE_INV((void *)ptr, prot->rw_index_sz);

	data = LTOH16(*((uint16*)ptr));

	DHD_TRACE(("%s: data %d type %d ringid %d ptr 0x%p offset %d\n",
		__FUNCTION__, data, type, ringid, ptr, offset));

	return (data);

} 


static INLINE int
dhd_prot_dma_indx_alloc(dhd_pub_t *dhd, uint8 type,
	dhd_dma_buf_t *dma_buf, uint32 bufsz)
{
	int rc;

	if ((dma_buf->len == bufsz) || (dma_buf->va != NULL))
		return BCME_OK;

	rc = dhd_dma_buf_alloc(dhd, dma_buf, bufsz);

	return rc;
}

int
dhd_prot_dma_indx_init(dhd_pub_t *dhd, uint32 rw_index_sz, uint8 type, uint32 length)
{
	uint32 bufsz;
	dhd_prot_t *prot = dhd->prot;
	dhd_dma_buf_t *dma_buf;

	if (prot == NULL) {
		DHD_ERROR(("prot is not inited\n"));
		return BCME_ERROR;
	}

	
	ASSERT(rw_index_sz != 0);
	prot->rw_index_sz = rw_index_sz;

	bufsz = rw_index_sz * length;

	switch (type) {
		case H2D_DMA_INDX_WR_BUF:
			dma_buf = &prot->h2d_dma_indx_wr_buf;
			if (dhd_prot_dma_indx_alloc(dhd, type, dma_buf, bufsz)) {
				goto ret_no_mem;
			}
			DHD_ERROR(("H2D DMA WR INDX : array size %d = %d * %d\n",
				dma_buf->len, rw_index_sz, length));
			break;

		case H2D_DMA_INDX_RD_BUF:
			dma_buf = &prot->h2d_dma_indx_rd_buf;
			if (dhd_prot_dma_indx_alloc(dhd, type, dma_buf, bufsz)) {
				goto ret_no_mem;
			}
			DHD_ERROR(("H2D DMA RD INDX : array size %d = %d * %d\n",
				dma_buf->len, rw_index_sz, length));
			break;

		case D2H_DMA_INDX_WR_BUF:
			dma_buf = &prot->d2h_dma_indx_wr_buf;
			if (dhd_prot_dma_indx_alloc(dhd, type, dma_buf, bufsz)) {
				goto ret_no_mem;
			}
			DHD_ERROR(("D2H DMA WR INDX : array size %d = %d * %d\n",
				dma_buf->len, rw_index_sz, length));
			break;

		case D2H_DMA_INDX_RD_BUF:
			dma_buf = &prot->d2h_dma_indx_rd_buf;
			if (dhd_prot_dma_indx_alloc(dhd, type, dma_buf, bufsz)) {
				goto ret_no_mem;
			}
			DHD_ERROR(("D2H DMA RD INDX : array size %d = %d * %d\n",
				dma_buf->len, rw_index_sz, length));
			break;

		default:
			DHD_ERROR(("%s: Unexpected option\n", __FUNCTION__));
			return BCME_BADOPTION;
	}

	return BCME_OK;

ret_no_mem:
	DHD_ERROR(("%s: dhd_prot_dma_indx_alloc type %d buf_sz %d failure\n",
		__FUNCTION__, type, bufsz));
	return BCME_NOMEM;

} 


static uint8*
dhd_prot_get_read_addr(dhd_pub_t *dhd, msgbuf_ring_t *ring, uint32 *available_len)
{
	uint16 wr;
	uint16 rd;
	uint16 depth;
	uint16 items;
	void  *read_addr = NULL; 
	uint16 d2h_wr = 0;

	DHD_TRACE(("%s: d2h_dma_indx_rd_buf %p, d2h_dma_indx_wr_buf %p\n",
		__FUNCTION__, (uint32 *)(dhd->prot->d2h_dma_indx_rd_buf.va),
		(uint32 *)(dhd->prot->d2h_dma_indx_wr_buf.va)));

	
	if (DMA_INDX_ENAB(dhd->dma_d2h_ring_upd_support)) {
		
		d2h_wr = dhd_prot_dma_indx_get(dhd, D2H_DMA_INDX_WR_UPD, ring->idx);
		ring->wr = d2h_wr;
	} else {
		dhd_bus_cmn_readshared(dhd->bus, &(ring->wr), RING_WR_UPD, ring->idx);
	}

	wr = ring->wr;
	rd = ring->rd;
	depth = ring->max_items;

	
	items = READ_AVAIL_SPACE(wr, rd, depth);
	if (items == 0) {
		return NULL;
	}

	ASSERT(items < ring->max_items);

	if (items >= ring->max_items) {
		DHD_ERROR(("\r\n======================= \r\n"));
		DHD_ERROR(("%s(): ring %p, ring->name %s, ring->max_items %d, items %d \r\n",
			__FUNCTION__, ring, ring->name, ring->max_items, items));
		DHD_ERROR(("wr: %d,  rd: %d,  depth: %d  \r\n", wr, rd, depth));
		DHD_ERROR(("dhd->busstate %d bus->suspended %d bus->wait_for_d3_ack %d \r\n",
			dhd->busstate, dhd->bus->suspended, dhd->bus->wait_for_d3_ack));
		DHD_ERROR(("\r\n======================= \r\n"));

		*available_len = 0;
		return NULL;
	}

	
	read_addr = (char*)ring->dma_buf.va + (rd * ring->item_len);

	
	if ((ring->rd + items) >= ring->max_items) {
		ring->rd = 0;
	} else {
		ring->rd += items;
	}

	ASSERT(ring->rd < ring->max_items);

	
	*available_len = (uint32)(items * ring->item_len);

	OSL_CACHE_INV(read_addr, *available_len);

	
	return read_addr;

} 

int
dhd_prot_flow_ring_create(dhd_pub_t *dhd, flow_ring_node_t *flow_ring_node)
{
	tx_flowring_create_request_t *flow_create_rqst;
	msgbuf_ring_t *flow_ring;
	dhd_prot_t *prot = dhd->prot;
	unsigned long flags;
	uint16 alloced = 0;
	msgbuf_ring_t *ctrl_ring = &prot->h2dring_ctrl_subn;

	
	flow_ring = dhd_prot_flowrings_pool_fetch(dhd, flow_ring_node->flowid);
	if (flow_ring == NULL) {
		DHD_ERROR(("%s: dhd_prot_flowrings_pool_fetch TX Flowid %d failed\n",
			__FUNCTION__, flow_ring_node->flowid));
		return BCME_NOMEM;
	}

	DHD_GENERAL_LOCK(dhd, flags);

	
	flow_create_rqst = (tx_flowring_create_request_t *)
		dhd_prot_alloc_ring_space(dhd, ctrl_ring, 1, &alloced, FALSE);

	if (flow_create_rqst == NULL) {
		dhd_prot_flowrings_pool_release(dhd, flow_ring_node->flowid, flow_ring);
		DHD_ERROR(("%s: Flow Create Req flowid %d - failure ring space\n",
			__FUNCTION__, flow_ring_node->flowid));
		DHD_GENERAL_UNLOCK(dhd, flags);
		return BCME_NOMEM;
	}

	flow_ring_node->prot_info = (void *)flow_ring;

	
	flow_create_rqst->msg.msg_type = MSG_TYPE_FLOW_RING_CREATE;
	flow_create_rqst->msg.if_id = (uint8)flow_ring_node->flow_info.ifindex;
	flow_create_rqst->msg.request_id = htol32(0); 

	flow_create_rqst->msg.epoch = ctrl_ring->seqnum % H2D_EPOCH_MODULO;
	ctrl_ring->seqnum++;

	
	flow_create_rqst->tid = flow_ring_node->flow_info.tid;
	flow_create_rqst->flow_ring_id = htol16((uint16)flow_ring_node->flowid);
	memcpy(flow_create_rqst->sa, flow_ring_node->flow_info.sa, sizeof(flow_create_rqst->sa));
	memcpy(flow_create_rqst->da, flow_ring_node->flow_info.da, sizeof(flow_create_rqst->da));
	
	flow_create_rqst->flow_ring_ptr.low_addr = flow_ring->base_addr.low_addr;
	flow_create_rqst->flow_ring_ptr.high_addr = flow_ring->base_addr.high_addr;
	flow_create_rqst->max_items = htol16(H2DRING_TXPOST_MAX_ITEM);
	flow_create_rqst->len_item = htol16(H2DRING_TXPOST_ITEMSIZE);
	DHD_ERROR(("%s: Send Flow Create Req flow ID %d for peer " MACDBG
		" prio %d ifindex %d\n", __FUNCTION__, flow_ring_node->flowid,
		MAC2STRDBG(flow_ring_node->flow_info.da), flow_ring_node->flow_info.tid,
		flow_ring_node->flow_info.ifindex));

	
	if (DMA_INDX_ENAB(dhd->dma_h2d_ring_upd_support)) {
		dhd_prot_dma_indx_set(dhd, flow_ring->wr,
			H2D_DMA_INDX_WR_UPD, flow_ring->idx);
	} else {
		dhd_bus_cmn_writeshared(dhd->bus, &(flow_ring->wr),
			sizeof(uint16), RING_WR_UPD, flow_ring->idx);
	}

	
	dhd_prot_ring_write_complete(dhd, ctrl_ring, flow_create_rqst, 1);

	DHD_GENERAL_UNLOCK(dhd, flags);

	return BCME_OK;
} 

static void
dhd_prot_flow_ring_create_response_process(dhd_pub_t *dhd, void *msg)
{
	tx_flowring_create_response_t *flow_create_resp = (tx_flowring_create_response_t *)msg;

	DHD_ERROR(("%s: Flow Create Response status = %d Flow %d\n", __FUNCTION__,
		ltoh16(flow_create_resp->cmplt.status),
		ltoh16(flow_create_resp->cmplt.flow_ring_id)));

	dhd_bus_flow_ring_create_response(dhd->bus,
		ltoh16(flow_create_resp->cmplt.flow_ring_id),
		ltoh16(flow_create_resp->cmplt.status));
}

void dhd_prot_clean_flow_ring(dhd_pub_t *dhd, void *msgbuf_flow_info)
{
	msgbuf_ring_t *flow_ring = (msgbuf_ring_t *)msgbuf_flow_info;
	dhd_prot_ring_detach(dhd, flow_ring);
	DHD_INFO(("%s Cleaning up Flow \n", __FUNCTION__));
}

void dhd_prot_print_flow_ring(dhd_pub_t *dhd, void *msgbuf_flow_info,
	struct bcmstrbuf *strbuf, const char * fmt)
{
	const char *default_fmt = "RD %d WR %d\n";
	msgbuf_ring_t *flow_ring = (msgbuf_ring_t *)msgbuf_flow_info;
	uint16 rd, wr;

	if (fmt == NULL) {
		fmt = default_fmt;
	}
	dhd_bus_cmn_readshared(dhd->bus, &rd, RING_RD_UPD, flow_ring->idx);
	dhd_bus_cmn_readshared(dhd->bus, &wr, RING_WR_UPD, flow_ring->idx);
	bcm_bprintf(strbuf, fmt, rd, wr);
}

void dhd_prot_print_info(dhd_pub_t *dhd, struct bcmstrbuf *strbuf)
{
	dhd_prot_t *prot = dhd->prot;
	bcm_bprintf(strbuf, "CtrlPost: ");
	dhd_prot_print_flow_ring(dhd, &prot->h2dring_ctrl_subn, strbuf, NULL);
	bcm_bprintf(strbuf, "CtrlCpl: ");
	dhd_prot_print_flow_ring(dhd, &prot->d2hring_ctrl_cpln, strbuf, NULL);

	bcm_bprintf(strbuf, "RxPost: ");
	bcm_bprintf(strbuf, "RBP %d ", prot->rxbufpost);
	dhd_prot_print_flow_ring(dhd, &prot->h2dring_rxp_subn, strbuf, NULL);
	bcm_bprintf(strbuf, "RxCpl: ");
	dhd_prot_print_flow_ring(dhd, &prot->d2hring_rx_cpln, strbuf, NULL);

	bcm_bprintf(strbuf, "TxCpl: ");
	dhd_prot_print_flow_ring(dhd, &prot->d2hring_tx_cpln, strbuf, NULL);
	bcm_bprintf(strbuf, "active_tx_count %d	 pktidmap_avail %d\n",
		dhd->prot->active_tx_count,
		DHD_PKTID_AVAIL(dhd->prot->pktid_map_handle));
}

int
dhd_prot_flow_ring_delete(dhd_pub_t *dhd, flow_ring_node_t *flow_ring_node)
{
	tx_flowring_delete_request_t *flow_delete_rqst;
	dhd_prot_t *prot = dhd->prot;
	unsigned long flags;
	uint16 alloced = 0;
	msgbuf_ring_t *ring = &prot->h2dring_ctrl_subn;

	DHD_GENERAL_LOCK(dhd, flags);

	
	flow_delete_rqst = (tx_flowring_delete_request_t *)
		dhd_prot_alloc_ring_space(dhd, ring, 1, &alloced, FALSE);

	if (flow_delete_rqst == NULL) {
		DHD_GENERAL_UNLOCK(dhd, flags);
		DHD_ERROR(("%s: Flow Delete Req - failure ring space\n", __FUNCTION__));
		return BCME_NOMEM;
	}

	
	flow_delete_rqst->msg.msg_type = MSG_TYPE_FLOW_RING_DELETE;
	flow_delete_rqst->msg.if_id = (uint8)flow_ring_node->flow_info.ifindex;
	flow_delete_rqst->msg.request_id = htol32(0); 

	flow_delete_rqst->msg.epoch = ring->seqnum % H2D_EPOCH_MODULO;
	ring->seqnum++;

	
	flow_delete_rqst->flow_ring_id = htol16((uint16)flow_ring_node->flowid);
	flow_delete_rqst->reason = htol16(BCME_OK);

	DHD_ERROR(("%s: Send Flow Delete Req RING ID %d for peer " MACDBG
		" prio %d ifindex %d\n", __FUNCTION__, flow_ring_node->flowid,
		MAC2STRDBG(flow_ring_node->flow_info.da), flow_ring_node->flow_info.tid,
		flow_ring_node->flow_info.ifindex));

	
	dhd_prot_ring_write_complete(dhd, ring, flow_delete_rqst, 1);
	DHD_GENERAL_UNLOCK(dhd, flags);

	return BCME_OK;
}

static void
dhd_prot_flow_ring_delete_response_process(dhd_pub_t *dhd, void *msg)
{
	tx_flowring_delete_response_t *flow_delete_resp = (tx_flowring_delete_response_t *)msg;

	DHD_INFO(("%s: Flow Delete Response status = %d \n", __FUNCTION__,
		flow_delete_resp->cmplt.status));

	dhd_bus_flow_ring_delete_response(dhd->bus, flow_delete_resp->cmplt.flow_ring_id,
		flow_delete_resp->cmplt.status);
}

int
dhd_prot_flow_ring_flush(dhd_pub_t *dhd, flow_ring_node_t *flow_ring_node)
{
	tx_flowring_flush_request_t *flow_flush_rqst;
	dhd_prot_t *prot = dhd->prot;
	unsigned long flags;
	uint16 alloced = 0;
	msgbuf_ring_t *ring = &prot->h2dring_ctrl_subn;

	DHD_GENERAL_LOCK(dhd, flags);

	
	flow_flush_rqst = (tx_flowring_flush_request_t *)
		dhd_prot_alloc_ring_space(dhd, ring, 1, &alloced, FALSE);
	if (flow_flush_rqst == NULL) {
		DHD_GENERAL_UNLOCK(dhd, flags);
		DHD_ERROR(("%s: Flow Flush Req - failure ring space\n", __FUNCTION__));
		return BCME_NOMEM;
	}

	
	flow_flush_rqst->msg.msg_type = MSG_TYPE_FLOW_RING_FLUSH;
	flow_flush_rqst->msg.if_id = (uint8)flow_ring_node->flow_info.ifindex;
	flow_flush_rqst->msg.request_id = htol32(0); 

	flow_flush_rqst->msg.epoch = ring->seqnum % H2D_EPOCH_MODULO;
	ring->seqnum++;

	flow_flush_rqst->flow_ring_id = htol16((uint16)flow_ring_node->flowid);
	flow_flush_rqst->reason = htol16(BCME_OK);

	DHD_INFO(("%s: Send Flow Flush Req\n", __FUNCTION__));

	
	dhd_prot_ring_write_complete(dhd, ring, flow_flush_rqst, 1);
	DHD_GENERAL_UNLOCK(dhd, flags);

	return BCME_OK;
} 

static void
dhd_prot_flow_ring_flush_response_process(dhd_pub_t *dhd, void *msg)
{
	tx_flowring_flush_response_t *flow_flush_resp = (tx_flowring_flush_response_t *)msg;

	DHD_INFO(("%s: Flow Flush Response status = %d\n", __FUNCTION__,
		flow_flush_resp->cmplt.status));

	dhd_bus_flow_ring_flush_response(dhd->bus, flow_flush_resp->cmplt.flow_ring_id,
		flow_flush_resp->cmplt.status);
}

void
dhd_msgbuf_ring_config_d2h_soft_doorbell(dhd_pub_t *dhd)
{
#if defined(DHD_D2H_SOFT_DOORBELL_SUPPORT)
	uint16 ring_idx;
	uint8 *msg_next;
	void *msg_start;
	uint16 alloced = 0;
	unsigned long flags;
	dhd_prot_t *prot = dhd->prot;
	ring_config_req_t *ring_config_req;
	bcmpcie_soft_doorbell_t *soft_doorbell;
	msgbuf_ring_t *ctrl_ring = &prot->h2dring_ctrl_subn;
	const uint16 d2h_rings = BCMPCIE_D2H_COMMON_MSGRINGS;

	
	DHD_GENERAL_LOCK(dhd, flags);
	msg_start = dhd_prot_alloc_ring_space(dhd, ctrl_ring, d2h_rings, &alloced, TRUE);

	if (msg_start == NULL) {
		DHD_ERROR(("%s Msgbuf no space for %d D2H ring config soft doorbells\n",
			__FUNCTION__, d2h_rings));
		DHD_GENERAL_UNLOCK(dhd, flags);
		return;
	}

	msg_next = (uint8*)msg_start;

	for (ring_idx = 0; ring_idx < d2h_rings; ring_idx++) {

		
		ring_config_req = (ring_config_req_t *)msg_next;

		
		ring_config_req->msg.msg_type = MSG_TYPE_D2H_RING_CONFIG;
		ring_config_req->msg.if_id = 0;
		ring_config_req->msg.flags = 0;

		ring_config_req->msg.epoch = ctrl_ring->seqnum % H2D_EPOCH_MODULO;
		ctrl_ring->seqnum++;

		ring_config_req->msg.request_id = htol32(DHD_FAKE_PKTID); 

		
		ring_config_req->subtype = htol16(D2H_RING_CONFIG_SUBTYPE_SOFT_DOORBELL);
		ring_config_req->ring_id = htol16(DHD_D2H_RINGID(ring_idx));

		
		soft_doorbell = &prot->soft_doorbell[ring_idx];

		ring_config_req->soft_doorbell.value = htol32(soft_doorbell->value);
		ring_config_req->soft_doorbell.haddr.high =
			htol32(soft_doorbell->haddr.high);
		ring_config_req->soft_doorbell.haddr.low =
			htol32(soft_doorbell->haddr.low);
		ring_config_req->soft_doorbell.items = htol16(soft_doorbell->items);
		ring_config_req->soft_doorbell.msecs = htol16(soft_doorbell->msecs);

		DHD_INFO(("%s: Soft doorbell haddr 0x%08x 0x%08x value 0x%08x\n",
			__FUNCTION__, ring_config_req->soft_doorbell.haddr.high,
			ring_config_req->soft_doorbell.haddr.low,
			ring_config_req->soft_doorbell.value));

		msg_next = msg_next + ctrl_ring->item_len;
	}

	
	dhd_prot_ring_write_complete(dhd, ctrl_ring, msg_start, d2h_rings);
	DHD_GENERAL_UNLOCK(dhd, flags);
#endif 
}

static void
dhd_prot_d2h_ring_config_cmplt_process(dhd_pub_t *dhd, void *msg)
{
	DHD_INFO(("%s: Ring Config Response - status %d ringid %d\n",
		__FUNCTION__, ltoh16(((ring_config_resp_t *)msg)->compl_hdr.status),
		ltoh16(((ring_config_resp_t *)msg)->compl_hdr.flow_ring_id)));
}

int
dhd_prot_ringupd_dump(dhd_pub_t *dhd, struct bcmstrbuf *b)
{
	uint32 *ptr;
	uint32 value;
	uint32 i;
	uint32 max_h2d_queues = dhd_bus_max_h2d_queues(dhd->bus);

	OSL_CACHE_INV((void *)dhd->prot->d2h_dma_indx_wr_buf.va,
		dhd->prot->d2h_dma_indx_wr_buf.len);

	ptr = (uint32 *)(dhd->prot->d2h_dma_indx_wr_buf.va);

	bcm_bprintf(b, "\n max_tx_queues %d\n", max_h2d_queues);

	bcm_bprintf(b, "\nRPTR block H2D common rings, 0x%04x\n", ptr);
	value = ltoh32(*ptr);
	bcm_bprintf(b, "\tH2D CTRL: value 0x%04x\n", value);
	ptr++;
	value = ltoh32(*ptr);
	bcm_bprintf(b, "\tH2D RXPOST: value 0x%04x\n", value);

	ptr++;
	bcm_bprintf(b, "RPTR block Flow rings , 0x%04x\n", ptr);
	for (i = BCMPCIE_H2D_COMMON_MSGRINGS; i < max_h2d_queues; i++) {
		value = ltoh32(*ptr);
		bcm_bprintf(b, "\tflowring ID %d: value 0x%04x\n", i, value);
		ptr++;
	}

	OSL_CACHE_INV((void *)dhd->prot->h2d_dma_indx_rd_buf.va,
		dhd->prot->h2d_dma_indx_rd_buf.len);

	ptr = (uint32 *)(dhd->prot->h2d_dma_indx_rd_buf.va);

	bcm_bprintf(b, "\nWPTR block D2H common rings, 0x%04x\n", ptr);
	value = ltoh32(*ptr);
	bcm_bprintf(b, "\tD2H CTRLCPLT: value 0x%04x\n", value);
	ptr++;
	value = ltoh32(*ptr);
	bcm_bprintf(b, "\tD2H TXCPLT: value 0x%04x\n", value);
	ptr++;
	value = ltoh32(*ptr);
	bcm_bprintf(b, "\tD2H RXCPLT: value 0x%04x\n", value);

	return 0;
}

uint32
dhd_prot_metadata_dbg_set(dhd_pub_t *dhd, bool val)
{
	dhd_prot_t *prot = dhd->prot;
#if DHD_DBG_SHOW_METADATA
	prot->metadata_dbg = val;
#endif
	return (uint32)prot->metadata_dbg;
}

uint32
dhd_prot_metadata_dbg_get(dhd_pub_t *dhd)
{
	dhd_prot_t *prot = dhd->prot;
	return (uint32)prot->metadata_dbg;
}

uint32
dhd_prot_metadatalen_set(dhd_pub_t *dhd, uint32 val, bool rx)
{
	dhd_prot_t *prot = dhd->prot;
	if (rx)
		prot->rx_metadata_offset = (uint16)val;
	else
		prot->tx_metadata_offset = (uint16)val;
	return dhd_prot_metadatalen_get(dhd, rx);
}

uint32
dhd_prot_metadatalen_get(dhd_pub_t *dhd, bool rx)
{
	dhd_prot_t *prot = dhd->prot;
	if (rx)
		return prot->rx_metadata_offset;
	else
		return prot->tx_metadata_offset;
}

uint32
dhd_prot_txp_threshold(dhd_pub_t *dhd, bool set, uint32 val)
{
	dhd_prot_t *prot = dhd->prot;
	if (set)
		prot->txp_threshold = (uint16)val;
	val = prot->txp_threshold;
	return val;
}

#ifdef DHD_RX_CHAINING

static INLINE void BCMFASTPATH
dhd_rxchain_reset(rxchain_info_t *rxchain)
{
	rxchain->pkt_count = 0;
}

static void BCMFASTPATH
dhd_rxchain_frame(dhd_pub_t *dhd, void *pkt, uint ifidx)
{
	uint8 *eh;
	uint8 prio;
	dhd_prot_t *prot = dhd->prot;
	rxchain_info_t *rxchain = &prot->rxchain;

	ASSERT(!PKTISCHAINED(pkt));
	ASSERT(PKTCLINK(pkt) == NULL);
	ASSERT(PKTCGETATTR(pkt) == 0);

	eh = PKTDATA(dhd->osh, pkt);
	prio = IP_TOS46(eh + ETHER_HDR_LEN) >> IPV4_TOS_PREC_SHIFT;

	if (rxchain->pkt_count && !(PKT_CTF_CHAINABLE(dhd, ifidx, eh, prio, rxchain->h_sa,
		rxchain->h_da, rxchain->h_prio))) {
		
		dhd_rxchain_commit(dhd);
	}

	
	
	if (rxchain->pkt_count == 0) {
		
		rxchain->pkthead = rxchain->pkttail = pkt;

		
		rxchain->h_da = ((struct ether_header *)eh)->ether_dhost;
		rxchain->h_sa = ((struct ether_header *)eh)->ether_shost;
		rxchain->h_prio = prio;
		rxchain->ifidx = ifidx;
		rxchain->pkt_count++;
	} else {
		
		PKTSETCLINK(rxchain->pkttail, pkt);
		rxchain->pkttail = pkt;
		rxchain->pkt_count++;
	}

	if ((!ETHER_ISMULTI(rxchain->h_da)) &&
		((((struct ether_header *)eh)->ether_type == HTON16(ETHER_TYPE_IP)) ||
		(((struct ether_header *)eh)->ether_type == HTON16(ETHER_TYPE_IPV6)))) {
		PKTSETCHAINED(dhd->osh, pkt);
		PKTCINCRCNT(rxchain->pkthead);
		PKTCADDLEN(rxchain->pkthead, PKTLEN(dhd->osh, pkt));
	} else {
		dhd_rxchain_commit(dhd);
		return;
	}

	
	if (rxchain->pkt_count >= DHD_PKT_CTF_MAX_CHAIN_LEN) {
		dhd_rxchain_commit(dhd);
	}
}

static void BCMFASTPATH
dhd_rxchain_commit(dhd_pub_t *dhd)
{
	dhd_prot_t *prot = dhd->prot;
	rxchain_info_t *rxchain = &prot->rxchain;

	if (rxchain->pkt_count == 0)
		return;

	
	dhd_bus_rx_frame(dhd->bus, rxchain->pkthead, rxchain->ifidx, rxchain->pkt_count);

	
	dhd_rxchain_reset(rxchain);
}

#endif 
