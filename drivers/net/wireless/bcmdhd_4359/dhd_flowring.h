/**
 * @file Header file describing the flow rings DHD interfaces.
 *
 * Flow rings are transmit traffic (=propagating towards antenna) related entities.
 *
 * Provides type definitions and function prototypes used to create, delete and manage flow rings at
 * high level.
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
 * $Id: dhd_flowrings.h  jaganlv $
 */



#ifndef _dhd_flowrings_h_
#define _dhd_flowrings_h_

#define FLOW_RING_QUEUE_THRESHOLD       (2048)

#define FLOW_RING_COMMON                BCMPCIE_H2D_COMMON_MSGRINGS

#define FLOWID_INVALID                  (ID16_INVALID)
#define FLOWID_RESERVED                 (FLOW_RING_COMMON)

#define FLOW_RING_STATUS_OPEN           0
#define FLOW_RING_STATUS_PENDING        1
#define FLOW_RING_STATUS_CLOSED         2
#define FLOW_RING_STATUS_DELETE_PENDING 3
#define FLOW_RING_STATUS_FLUSH_PENDING  4
#define FLOW_RING_STATUS_STA_FREEING    5

#define DHD_FLOWRING_RX_BUFPOST_PKTSZ	2048

#define DHD_FLOW_PRIO_AC_MAP		0
#define DHD_FLOW_PRIO_TID_MAP		1

typedef struct dhd_pkttag_fr {
	uint16  flowid;
	uint16  ifid;
	int     dataoff;
	dmaaddr_t physaddr;
	uint32 pa_len;

} dhd_pkttag_fr_t;

#define DHD_PKTTAG_SET_FLOWID(tag, flow)    ((tag)->flowid = (uint16)(flow))
#define DHD_PKTTAG_SET_IFID(tag, idx)       ((tag)->ifid = (uint16)(idx))
#define DHD_PKTTAG_SET_DATAOFF(tag, offset) ((tag)->dataoff = (int)(offset))
#define DHD_PKTTAG_SET_PA(tag, pa)          ((tag)->physaddr = (pa))
#define DHD_PKTTAG_SET_PA_LEN(tag, palen)   ((tag)->pa_len = (palen))

#define DHD_PKTTAG_FLOWID(tag)              ((tag)->flowid)
#define DHD_PKTTAG_IFID(tag)                ((tag)->ifid)
#define DHD_PKTTAG_DATAOFF(tag)             ((tag)->dataoff)
#define DHD_PKTTAG_PA(tag)                  ((tag)->physaddr)
#define DHD_PKTTAG_PA_LEN(tag)              ((tag)->pa_len)

#define DHD_FLOWRING_HASH_SIZE    256
#define	DHD_FLOWRING_HASHINDEX(ea, prio) \
	       ((((uint8 *)(ea))[3] ^ ((uint8 *)(ea))[4] ^ ((uint8 *)(ea))[5] ^ ((uint8)(prio))) \
		% DHD_FLOWRING_HASH_SIZE)

#define DHD_IF_ROLE(pub, idx)		(((if_flow_lkup_t *)(pub)->if_flow_lkup)[idx].role)
#define DHD_IF_ROLE_AP(pub, idx)	(DHD_IF_ROLE(pub, idx) == WLC_E_IF_ROLE_AP)
#define DHD_IF_ROLE_STA(pub, idx)	(DHD_IF_ROLE(pub, idx) == WLC_E_IF_ROLE_STA)
#define DHD_IF_ROLE_P2PGO(pub, idx)	(DHD_IF_ROLE(pub, idx) == WLC_E_IF_ROLE_P2P_GO)
#define DHD_FLOW_RING(dhdp, flowid) \
	(flow_ring_node_t *)&(((flow_ring_node_t *)((dhdp)->flow_ring_table))[flowid])

struct flow_queue;

typedef int (*flow_queue_cb_t)(struct flow_queue * queue, void * pkt);

typedef struct flow_queue {
	dll_t  list;                
	void * head;                
	void * tail;                
	uint16 len;                 
	uint16 max;                 
	uint32 threshold;           
	void * clen_ptr;            
	uint32 failures;            
	flow_queue_cb_t cb;         
	uint32 l2threshold;         
	void * l2clen_ptr;          
} flow_queue_t;

#define DHD_FLOW_QUEUE_LEN(queue)       ((int)(queue)->len)
#define DHD_FLOW_QUEUE_MAX(queue)       ((int)(queue)->max)
#define DHD_FLOW_QUEUE_THRESHOLD(queue) ((int)(queue)->threshold)
#define DHD_FLOW_QUEUE_L2THRESHOLD(queue) ((int)(queue)->l2threshold)
#define DHD_FLOW_QUEUE_EMPTY(queue)     ((queue)->len == 0)
#define DHD_FLOW_QUEUE_FAILURES(queue)  ((queue)->failures)

#define DHD_FLOW_QUEUE_AVAIL(queue)     ((int)((queue)->max - (queue)->len))
#define DHD_FLOW_QUEUE_FULL(queue)      ((queue)->len >= (queue)->max)

#define DHD_FLOW_QUEUE_OVFL(queue, budget)  \
	(((queue)->len) > budget)

#define DHD_FLOW_QUEUE_SET_MAX(queue, budget) \
	((queue)->max) = ((budget) - 1)

#define DHD_FLOW_QUEUE_SET_THRESHOLD(queue, cumm_threshold) \
	((queue)->threshold) = ((cumm_threshold) - 1)

#define DHD_FLOW_QUEUE_CLEN_PTR(queue)  ((queue)->clen_ptr)

#define DHD_FLOW_QUEUE_SET_CLEN(queue, parent_clen_ptr)  \
	((queue)->clen_ptr) = (void *)(parent_clen_ptr)

#define DHD_FLOW_QUEUE_SET_L2THRESHOLD(queue, l2cumm_threshold) \
	((queue)->l2threshold) = ((l2cumm_threshold) - 1)

#define DHD_FLOW_QUEUE_L2CLEN_PTR(queue)  ((queue)->l2clen_ptr)

#define DHD_FLOW_QUEUE_SET_L2CLEN(queue, grandparent_clen_ptr)  \
	((queue)->l2clen_ptr) = (void *)(grandparent_clen_ptr)

#define DHD_FLOWRING_MAXSTATUS_MSGS	5
#define DHD_FLOWRING_TXSTATUS_CNT_UPDATE(bus, flowid, txstatus)
typedef struct flow_info {
	uint8		tid;
	uint8		ifindex;
	char		sa[ETHER_ADDR_LEN];
	char		da[ETHER_ADDR_LEN];
} flow_info_t;

typedef struct flow_ring_node {
	dll_t		list;  
	flow_queue_t	queue; 
	bool		active;
	uint8		status;
	uint16		flowid;
	flow_info_t	flow_info;
	void		*prot_info;
	void		*lock; 
} flow_ring_node_t;

typedef flow_ring_node_t flow_ring_table_t;

typedef struct flow_hash_info {
	uint16			flowid;
	flow_info_t		flow_info;
	struct flow_hash_info	*next;
} flow_hash_info_t;

typedef struct if_flow_lkup {
	bool		status;
	uint8		role; 
	flow_hash_info_t *fl_hash[DHD_FLOWRING_HASH_SIZE]; 
} if_flow_lkup_t;

static INLINE flow_ring_node_t *
dhd_constlist_to_flowring(dll_t *item)
{
	return ((flow_ring_node_t *)item);
}


extern flow_ring_node_t * dhd_flow_ring_node(dhd_pub_t *dhdp, uint16 flowid);
extern flow_queue_t * dhd_flow_queue(dhd_pub_t *dhdp, uint16 flowid);

extern void dhd_flow_queue_init(dhd_pub_t *dhdp, flow_queue_t *queue, int max);
extern void dhd_flow_queue_register(flow_queue_t *queue, flow_queue_cb_t cb);
extern int  dhd_flow_queue_enqueue(dhd_pub_t *dhdp, flow_queue_t *queue, void *pkt);
extern void * dhd_flow_queue_dequeue(dhd_pub_t *dhdp, flow_queue_t *queue);
extern int  dhd_flow_queue_reinsert(dhd_pub_t *dhdp, flow_queue_t *queue, void *pkt);

extern void dhd_flow_ring_config_thresholds(dhd_pub_t *dhdp, uint16 flowid,
                          int queue_budget, int cumm_threshold, void *cumm_ctr,
                          int l2cumm_threshold, void *l2cumm_ctr);
extern int  dhd_flow_rings_init(dhd_pub_t *dhdp, uint32 num_flow_rings);

extern void dhd_flow_rings_deinit(dhd_pub_t *dhdp);

extern int dhd_flowid_update(dhd_pub_t *dhdp, uint8 ifindex, uint8 prio,
                void *pktbuf);

extern void dhd_flowid_free(dhd_pub_t *dhdp, uint8 ifindex, uint16 flowid);

extern void dhd_flow_rings_delete(dhd_pub_t *dhdp, uint8 ifindex);

#ifdef CUSTOMER_HW_ONE
extern void dhd_flow_rings_pending_cleanup(dhd_pub_t *dhdp);
#endif

extern void dhd_flow_rings_delete_for_peer(dhd_pub_t *dhdp, uint8 ifindex,
                char *addr);

extern void dhd_update_interface_flow_info(dhd_pub_t *dhdp, uint8 ifindex,
                uint8 op, uint8 role);

extern int dhd_update_interface_link_status(dhd_pub_t *dhdp, uint8 ifindex,
                uint8 status);
extern int dhd_flow_prio_map(dhd_pub_t *dhd, uint8 *map, bool set);
extern int dhd_update_flow_prio_map(dhd_pub_t *dhdp, uint8 map);

extern uint8 dhd_flow_rings_ifindex2role(dhd_pub_t *dhdp, uint8 ifindex);
#endif 
