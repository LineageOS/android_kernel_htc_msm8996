/*
 * Header file describing the internal (inter-module) DHD interfaces.
 *
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
 * $Id: dhd_bus.h 608409 2015-12-25 10:41:39Z $
 */

#ifndef _dhd_bus_h_
#define _dhd_bus_h_


extern int dhd_bus_register(void);
extern void dhd_bus_unregister(void);

extern int dhd_bus_download_firmware(struct dhd_bus *bus, osl_t *osh, char *fw_path, char *nv_path);

extern void dhd_bus_stop(struct dhd_bus *bus, bool enforce_mutex);

extern int dhd_bus_init(dhd_pub_t *dhdp, bool enforce_mutex);

extern void dhd_bus_getidletime(dhd_pub_t *dhdp, int *idletime);

extern void dhd_bus_setidletime(dhd_pub_t *dhdp, int idle_time);

#ifdef BCMPCIE
extern int dhd_bus_txdata(struct dhd_bus *bus, void *txp, uint8 ifidx);
#else
extern int dhd_bus_txdata(struct dhd_bus *bus, void *txp);
#endif


extern int dhd_bus_txctl(struct dhd_bus *bus, uchar *msg, uint msglen);
extern int dhd_bus_rxctl(struct dhd_bus *bus, uchar *msg, uint msglen);

extern bool dhd_bus_watchdog(dhd_pub_t *dhd);

extern int dhd_bus_oob_intr_register(dhd_pub_t *dhdp);
extern void dhd_bus_oob_intr_unregister(dhd_pub_t *dhdp);
extern void dhd_bus_oob_intr_set(dhd_pub_t *dhdp, bool enable);
extern void dhd_bus_dev_pm_stay_awake(dhd_pub_t *dhdpub);
extern void dhd_bus_dev_pm_relax(dhd_pub_t *dhdpub);
extern bool dhd_bus_dev_pm_enabled(dhd_pub_t *dhdpub);

#if defined(DHD_DEBUG)
extern int dhd_bus_console_in(dhd_pub_t *dhd, uchar *msg, uint msglen);
#endif 

extern bool dhd_bus_dpc(struct dhd_bus *bus);
extern void dhd_bus_isr(bool * InterruptRecognized, bool * QueueMiniportHandleInterrupt, void *arg);


extern int dhd_bus_iovar_op(dhd_pub_t *dhdp, const char *name,
                            void *params, int plen, void *arg, int len, bool set);

extern void dhd_bus_dump(dhd_pub_t *dhdp, struct bcmstrbuf *strbuf);

extern void dhd_bus_clearcounts(dhd_pub_t *dhdp);

extern uint dhd_bus_chip(struct dhd_bus *bus);

extern uint dhd_bus_chiprev(struct dhd_bus *bus);

extern void dhd_bus_set_nvram_params(struct dhd_bus * bus, const char *nvram_params);

extern void *dhd_bus_pub(struct dhd_bus *bus);
extern void *dhd_bus_txq(struct dhd_bus *bus);
extern void *dhd_bus_sih(struct dhd_bus *bus);
extern uint dhd_bus_hdrlen(struct dhd_bus *bus);
#ifdef BCMSDIO
extern void dhd_bus_set_dotxinrx(struct dhd_bus *bus, bool val);
extern uint8 dhd_bus_is_ioready(struct dhd_bus *bus);
#else
#define dhd_bus_set_dotxinrx(a, b) do {} while (0)
#endif

#define DHD_SET_BUS_STATE_DOWN(_bus)  do { \
	(_bus)->dhd->busstate = DHD_BUS_DOWN; \
} while (0)

extern int dhd_bus_reg_sdio_notify(void* semaphore);
extern void dhd_bus_unreg_sdio_notify(void);
extern void dhd_txglom_enable(dhd_pub_t *dhdp, bool enable);
extern int dhd_bus_get_ids(struct dhd_bus *bus, uint32 *bus_type, uint32 *bus_num,
	uint32 *slot_num);

#ifdef BCMPCIE
enum {
	
	D2H_DMA_SCRATCH_BUF,
	D2H_DMA_SCRATCH_BUF_LEN,

	
	H2D_DMA_INDX_WR_BUF, 
	H2D_DMA_INDX_RD_BUF, 
	D2H_DMA_INDX_WR_BUF, 
	D2H_DMA_INDX_RD_BUF, 

	
	H2D_DMA_INDX_WR_UPD, 
	H2D_DMA_INDX_RD_UPD, 
	D2H_DMA_INDX_WR_UPD, 
	D2H_DMA_INDX_RD_UPD, 

	
	H2D_MB_DATA,
	D2H_MB_DATA,

	
	RING_BUF_ADDR,       
	RING_ITEM_LEN,       
	RING_MAX_ITEMS,      

	
	RING_RD_UPD,         
	RING_WR_UPD,         

	TOTAL_LFRAG_PACKET_CNT,
	MAX_HOST_RXBUFS
};

typedef void (*dhd_mb_ring_t) (struct dhd_bus *, uint32);
extern void dhd_bus_cmn_writeshared(struct dhd_bus *bus, void * data, uint32 len, uint8 type,
	uint16 ringid);
extern void dhd_bus_ringbell(struct dhd_bus *bus, uint32 value);
extern void dhd_bus_cmn_readshared(struct dhd_bus *bus, void* data, uint8 type, uint16 ringid);
extern uint32 dhd_bus_get_sharedflags(struct dhd_bus *bus);
extern void dhd_bus_rx_frame(struct dhd_bus *bus, void* pkt, int ifidx, uint pkt_count);
extern void dhd_bus_start_queue(struct dhd_bus *bus);
extern void dhd_bus_stop_queue(struct dhd_bus *bus);

extern dhd_mb_ring_t dhd_bus_get_mbintr_fn(struct dhd_bus *bus);
extern void dhd_bus_write_flow_ring_states(struct dhd_bus *bus,
	void * data, uint16 flowid);
extern void dhd_bus_read_flow_ring_states(struct dhd_bus *bus,
	void * data, uint8 flowid);
extern int dhd_bus_flow_ring_create_request(struct dhd_bus *bus, void *flow_ring_node);
extern void dhd_bus_clean_flow_ring(struct dhd_bus *bus, void *flow_ring_node);
extern void dhd_bus_flow_ring_create_response(struct dhd_bus *bus, uint16 flow_id, int32 status);
extern int dhd_bus_flow_ring_delete_request(struct dhd_bus *bus, void *flow_ring_node);
extern void dhd_bus_flow_ring_delete_response(struct dhd_bus *bus, uint16 flowid, uint32 status);
extern int dhd_bus_flow_ring_flush_request(struct dhd_bus *bus, void *flow_ring_node);
extern void dhd_bus_flow_ring_flush_response(struct dhd_bus *bus, uint16 flowid, uint32 status);
extern uint32 dhd_bus_max_h2d_queues(struct dhd_bus *bus);
extern int dhd_bus_schedule_queue(struct dhd_bus *bus, uint16 flow_id, bool txs);


extern int dhdpcie_bus_clock_start(struct dhd_bus *bus);
extern int dhdpcie_bus_clock_stop(struct dhd_bus *bus);
extern int dhdpcie_bus_enable_device(struct dhd_bus *bus);
extern int dhdpcie_bus_disable_device(struct dhd_bus *bus);
extern int dhdpcie_bus_alloc_resource(struct dhd_bus *bus);
extern void dhdpcie_bus_free_resource(struct dhd_bus *bus);
extern bool dhdpcie_bus_dongle_attach(struct dhd_bus *bus);
extern int dhd_bus_release_dongle(struct dhd_bus *bus);
extern int dhd_bus_request_irq(struct dhd_bus *bus);


#ifdef DHD_FW_COREDUMP
extern int dhd_bus_mem_dump(dhd_pub_t *dhd);
#endif 

#if defined(DHD_PCIE_RUNTIMEPM) && defined(CUSTOMER_HW_ONE)
extern void dhd_mfg_setidletime(dhd_pub_t *dhdp, int idle_time);
#endif 

#endif 
#endif 
