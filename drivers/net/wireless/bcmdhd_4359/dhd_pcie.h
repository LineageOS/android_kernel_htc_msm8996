/*
 * Linux DHD Bus Module for PCIE
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
 * $Id: dhd_pcie.h 619124 2016-02-15 09:26:47Z $
 */


#ifndef dhd_pcie_h
#define dhd_pcie_h

#include <bcmpcie.h>
#include <hnd_cons.h>
#ifdef SUPPORT_LINKDOWN_RECOVERY
#ifdef CONFIG_ARCH_MSM
#if defined(CONFIG_ARCH_MSM8994) || defined(CONFIG_ARCH_MSM8996)
#include <linux/msm_pcie.h>
#else
#include <mach/msm_pcie.h>
#endif 
#endif 
#endif 

#ifdef DHD_PCIE_RUNTIMEPM
#include <linux/mutex.h>
#include <linux/wait.h>

#define DEFAULT_DHD_RUNTIME_MS 100
#ifndef CUSTOM_DHD_RUNTIME_MS
#define CUSTOM_DHD_RUNTIME_MS DEFAULT_DHD_RUNTIME_MS
#endif 


#ifndef MAX_IDLE_COUNT
#define MAX_IDLE_COUNT 16
#endif 

#ifndef MAX_RESUME_WAIT
#define MAX_RESUME_WAIT 100
#endif 
#endif 


#define PCMSGBUF_HDRLEN 0
#define DONGLE_REG_MAP_SIZE (32 * 1024)
#define DONGLE_TCM_MAP_SIZE (4096 * 1024)
#define DONGLE_MIN_MEMSIZE (128 *1024)
#ifdef DHD_DEBUG
#define DHD_PCIE_SUCCESS 0
#define DHD_PCIE_FAILURE 1
#endif 
#define	REMAP_ENAB(bus)			((bus)->remap)
#define	REMAP_ISADDR(bus, a)		(((a) >= ((bus)->orig_ramsize)) && ((a) < ((bus)->ramsize)))

#define MAX_DHD_TX_FLOWS	320

#define CONSOLE_LINE_MAX	192
#define CONSOLE_BUFFER_MAX	(8 * 1024)

#ifndef MAX_CNTL_D3ACK_TIMEOUT
#define MAX_CNTL_D3ACK_TIMEOUT 2
#endif 

#ifdef DHD_DEBUG

typedef struct dhd_console {
	 uint		count;	
	 uint		log_addr;		 
	 hnd_log_t	 log;			 
	 uint		 bufsize;		 
	 uint8		 *buf;			 
	 uint		 last;			 
} dhd_console_t;
#endif 
typedef struct ring_sh_info {
	uint32 ring_mem_addr;
	uint32 ring_state_w;
	uint32 ring_state_r;
} ring_sh_info_t;

typedef struct dhd_bus {
	dhd_pub_t	*dhd;
	struct pci_dev  *dev;		
	dll_t       const_flowring; 

	si_t		*sih;			
	char		*vars;			
	uint		varsz;			
	uint32		sbaddr;			
	sbpcieregs_t	*reg;			

	uint		armrev;			
	uint		ramrev;			
	uint32		ramsize;		
	uint32		orig_ramsize;		
	uint32		srmemsize;		

	uint32		bus;			
	uint32		intstatus;		
	bool		dpc_sched;		
	bool		fcstate;		

	uint16		cl_devid;		
	char		*fw_path;		
	char		*nv_path;		
#ifdef CACHE_FW_IMAGES
	int			processed_nvram_params_len;	
#endif


	struct pktq	txq;			

	bool		intr;			
	bool		ipend;			
	bool		intdis;			
	uint		intrcount;		
	uint		lastintrs;		

#ifdef DHD_DEBUG
	dhd_console_t	console;		
	uint		console_addr;		
#endif 

	bool		alp_only;		

	bool		remap;		
	uint32		resetinstr;
	uint32		dongle_ram_base;

	ulong		shared_addr;
	pciedev_shared_t	*pcie_sh;
	bool bus_flowctrl;
	uint32		dma_rxoffset;
	volatile char	*regs;		
	volatile char	*tcm;		
	uint32		tcm_size;
#ifdef CUSTOMER_HW_ONE
	phys_addr_t bar0_addr;
	phys_addr_t bar1_addr;
#endif 
	osl_t		*osh;
	uint32		nvram_csm;	
	uint16		pollrate;
	uint16  polltick;

	uint32  *pcie_mb_intr_addr;
	void    *pcie_mb_intr_osh;
	bool	sleep_allowed;

	
	ring_sh_info_t	ring_sh[BCMPCIE_COMMON_MSGRINGS + MAX_DHD_TX_FLOWS];
	uint8	h2d_ring_count;
	uint8	d2h_ring_count;
	uint32  ringmem_ptr;
	uint32  ring_state_ptr;

	uint32 d2h_dma_scratch_buffer_mem_addr;

	uint32 h2d_mb_data_ptr_addr;
	uint32 d2h_mb_data_ptr_addr;
	

	uint32 def_intmask;
	bool	ltrsleep_on_unload;
	uint	wait_for_d3_ack;
	uint32 max_sub_queues;
	uint32	rw_index_sz;
	bool	db1_for_mb;
	bool	suspended;

	dhd_timeout_t doorbell_timer;
	bool	device_wake_state;
#ifdef PCIE_OOB
	bool	oob_enabled;
#endif 
#ifdef SUPPORT_LINKDOWN_RECOVERY
#ifdef CONFIG_ARCH_MSM
	struct msm_pcie_register_event pcie_event;
	uint8 islinkdown;
#endif 
#endif 
#ifdef DHD_PCIE_RUNTIMEPM
	int32 idlecount;                
	int32 idletime;                 
	int32 bus_wake;                 
	bool runtime_resume_done;       
	struct mutex pm_lock;            
	wait_queue_head_t rpm_queue;    
#endif 
	uint32 d3_inform_cnt;
	uint32 d0_inform_cnt;
	uint32 d0_inform_in_use_cnt;
	uint8 force_suspend;
	uint32 oob_irq_num;
	uint32 pci_d3hot_done;
#ifdef CUSTOMER_HW_ONE
	struct workqueue_struct *cleanup_wq;
	struct work_struct cleanup_flow_work;
#endif

} dhd_bus_t;


extern uint32* dhdpcie_bus_reg_map(osl_t *osh, ulong addr, int size);
extern int dhdpcie_bus_register(void);
extern void dhdpcie_bus_unregister(void);
extern bool dhdpcie_chipmatch(uint16 vendor, uint16 device);

extern struct dhd_bus* dhdpcie_bus_attach(osl_t *osh,
	volatile char *regs, volatile char *tcm, uint32 tcm_size, void *pci_dev);
extern uint32 dhdpcie_bus_cfg_read_dword(struct dhd_bus *bus, uint32 addr, uint32 size);
extern void dhdpcie_bus_cfg_write_dword(struct dhd_bus *bus, uint32 addr, uint32 size, uint32 data);
extern void dhdpcie_bus_intr_enable(struct dhd_bus *bus);
extern void dhdpcie_bus_intr_disable(struct dhd_bus *bus);
extern void dhdpcie_bus_release(struct dhd_bus *bus);
extern int32 dhdpcie_bus_isr(struct dhd_bus *bus);
extern void dhdpcie_free_irq(dhd_bus_t *bus);
extern void dhdpcie_bus_ringbell_fast(struct dhd_bus *bus, uint32 value);
extern int dhdpcie_bus_suspend(struct  dhd_bus *bus, bool state);
extern int dhdpcie_pci_suspend_resume(struct  dhd_bus *bus, bool state);
extern bool dhdpcie_tcm_valid(dhd_bus_t *bus);
#ifndef BCMPCIE_OOB_HOST_WAKE
extern void dhdpcie_pme_active(osl_t *osh, bool enable);
#endif 
extern bool dhdpcie_pme_cap(osl_t *osh);
extern int dhdpcie_start_host_pcieclock(dhd_bus_t *bus);
extern int dhdpcie_stop_host_pcieclock(dhd_bus_t *bus);
extern int dhdpcie_disable_device(dhd_bus_t *bus);
extern int dhdpcie_enable_device(dhd_bus_t *bus);
extern int dhdpcie_alloc_resource(dhd_bus_t *bus);
extern void dhdpcie_free_resource(dhd_bus_t *bus);
extern int dhdpcie_bus_request_irq(struct dhd_bus *bus);
#ifdef BCMPCIE_OOB_HOST_WAKE
extern int dhdpcie_oob_intr_register(dhd_bus_t *bus);
extern void dhdpcie_oob_intr_unregister(dhd_bus_t *bus);
extern void dhdpcie_oob_intr_set(dhd_bus_t *bus, bool enable);
#endif 
#ifdef PCIE_OOB
extern void dhd_oob_set_bt_reg_on(struct dhd_bus *bus, bool val);
extern int dhd_oob_get_bt_reg_on(struct dhd_bus *bus);
#endif 

extern int dhd_buzzz_dump_dngl(dhd_bus_t *bus);

#endif 
