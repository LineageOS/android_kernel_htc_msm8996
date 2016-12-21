/*
 * BCMSDH Function Driver for the native SDIO/MMC driver in the Linux Kernel
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
 * <<Broadcom-WL-IPTag/Proprietary,Open:>>
 *
 * $Id: bcmsdh_sdmmc.h 514727 2014-11-12 03:02:48Z $
 */

#ifndef __BCMSDH_SDMMC_H__
#define __BCMSDH_SDMMC_H__

#define sd_err(x)
#define sd_trace(x)
#define sd_info(x)
#define sd_debug(x)
#define sd_data(x)
#define sd_ctrl(x)


#define sd_sync_dma(sd, read, nbytes)
#define sd_init_dma(sd)
#define sd_ack_intr(sd)
#define sd_wakeup(sd);

#define sd_log(x)

#define SDIOH_ASSERT(exp) \
	do { if (!(exp)) \
		printf("!!!ASSERT fail: file %s lines %d", __FILE__, __LINE__); \
	} while (0)

#define BLOCK_SIZE_4318 64
#define BLOCK_SIZE_4328 512

#define SUCCESS	0
#define ERROR	1

#define SDIOH_MODE_SD4		2
#define CLIENT_INTR			0x100	
#define SDIOH_SDMMC_MAX_SG_ENTRIES	32

struct sdioh_info {
	osl_t		*osh;			
	void		*bcmsdh;		
	bool		client_intr_enabled;	
	bool		intr_handler_valid;	
	sdioh_cb_fn_t	intr_handler;		
	void		*intr_handler_arg;	
	uint16		intmask;		

	int		intrcount;		
	bool		sd_use_dma;		
	bool		sd_blockmode;		
						
	bool		use_client_ints;	
	int		sd_mode;		
	int		client_block_size[SDIOD_MAX_IOFUNCS];		
	uint8		num_funcs;		
	uint32		com_cis_ptr;
	uint32		func_cis_ptr[SDIOD_MAX_IOFUNCS];
	bool		use_rxchain;
	struct scatterlist	sg_list[SDIOH_SDMMC_MAX_SG_ENTRIES];
	struct sdio_func	fake_func0;
	struct sdio_func	*func[SDIOD_MAX_IOFUNCS];

};


extern uint sd_msglevel;

extern bool check_client_intr(sdioh_info_t *sd);

extern void sdioh_sdmmc_devintr_on(sdioh_info_t *sd);
extern void sdioh_sdmmc_devintr_off(sdioh_info_t *sd);



extern uint32 *sdioh_sdmmc_reg_map(osl_t *osh, int32 addr, int size);
extern void sdioh_sdmmc_reg_unmap(osl_t *osh, int32 addr, int size);

extern int sdioh_sdmmc_register_irq(sdioh_info_t *sd, uint irq);
extern void sdioh_sdmmc_free_irq(uint irq, sdioh_info_t *sd);

extern sdioh_info_t *sdioh_attach(osl_t *osh, struct sdio_func *func);
extern SDIOH_API_RC sdioh_detach(osl_t *osh, sdioh_info_t *sd);
#endif 
