/*
 * SDIO access interface for drivers - linux specific (pci only)
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
 * $Id: bcmsdh_linux.c 514727 2014-11-12 03:02:48Z $
 */


#define __UNDEF_NO_VERSION__

#include <typedefs.h>
#include <linuxver.h>
#include <linux/pci.h>
#include <linux/completion.h>

#include <osl.h>
#include <pcicfg.h>
#include <bcmdefs.h>
#include <bcmdevs.h>
#include <linux/irq.h>
extern void dhdsdio_isr(void * args);
#include <bcmutils.h>
#include <dngl_stats.h>
#include <dhd.h>
#if defined(CONFIG_ARCH_ODIN)
#include <linux/platform_data/gpio-odin.h>
#endif 
#include <dhd_linux.h>

static bcmsdh_driver_t drvinfo = {NULL, NULL, NULL, NULL};

typedef enum {
	DHD_INTR_INVALID = 0,
	DHD_INTR_INBAND,
	DHD_INTR_HWOOB,
	DHD_INTR_SWOOB
} DHD_HOST_INTR_TYPE;

typedef struct bcmsdh_os_info {
	DHD_HOST_INTR_TYPE	intr_type;
	int			oob_irq_num;	
	unsigned long		oob_irq_flags;	
	bool			oob_irq_registered;
	bool			oob_irq_enabled;
	bool			oob_irq_wake_enabled;
	spinlock_t		oob_irq_spinlock;
	bcmsdh_cb_fn_t		oob_irq_handler;
	void			*oob_irq_handler_context;
	void			*context;	
	void			*sdioh;		
	void			*dev;		
	bool			dev_wake_enabled;
} bcmsdh_os_info_t;

#define SDLX_MSG(x)

bool
bcmsdh_chipmatch(uint16 vendor, uint16 device)
{
	

#ifdef BCMSDIOH_STD
	
	if (vendor == VENDOR_SI_IMAGE) {
		return (TRUE);
	}
	
	if (device == BCM27XX_SDIOH_ID && vendor == VENDOR_BROADCOM) {
		return (TRUE);
	}
	
	if (device == SDIOH_FPGA_ID && vendor == VENDOR_BROADCOM) {
		return (TRUE);
	}
	
	if (device == PCIXX21_SDIOH_ID && vendor == VENDOR_TI) {
		return (TRUE);
	}
	if (device == PCIXX21_SDIOH0_ID && vendor == VENDOR_TI) {
		return (TRUE);
	}
	
	if (device == R5C822_SDIOH_ID && vendor == VENDOR_RICOH) {
		return (TRUE);
	}
	
	if (device == JMICRON_SDIOH_ID && vendor == VENDOR_JMICRON) {
		return (TRUE);
	}

#endif 
#ifdef BCMSDIOH_SPI
	
	if (device == SPIH_FPGA_ID && vendor == VENDOR_BROADCOM) {
		printf("Found PCI SPI Host Controller\n");
		return (TRUE);
	}

#endif 

	return (FALSE);
}

void* bcmsdh_probe(osl_t *osh, void *dev, void *sdioh, void *adapter_info, uint bus_type,
	uint bus_num, uint slot_num)
{
	ulong regs;
	bcmsdh_info_t *bcmsdh;
	uint32 vendevid;
	bcmsdh_os_info_t *bcmsdh_osinfo = NULL;

	bcmsdh = bcmsdh_attach(osh, sdioh, &regs);
	if (bcmsdh == NULL) {
		SDLX_MSG(("%s: bcmsdh_attach failed\n", __FUNCTION__));
		goto err;
	}
	bcmsdh_osinfo = MALLOC(osh, sizeof(bcmsdh_os_info_t));
	if (bcmsdh_osinfo == NULL) {
		SDLX_MSG(("%s: failed to allocate bcmsdh_os_info_t\n", __FUNCTION__));
		goto err;
	}
	bzero((char *)bcmsdh_osinfo, sizeof(bcmsdh_os_info_t));
	bcmsdh->os_cxt = bcmsdh_osinfo;
	bcmsdh_osinfo->sdioh = sdioh;
	bcmsdh_osinfo->dev = dev;
	osl_set_bus_handle(osh, bcmsdh);

#if !defined(CONFIG_HAS_WAKELOCK) && (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36))
	if (dev && device_init_wakeup(dev, true) == 0)
		bcmsdh_osinfo->dev_wake_enabled = TRUE;
#endif 

#if defined(OOB_INTR_ONLY)
	spin_lock_init(&bcmsdh_osinfo->oob_irq_spinlock);
	
	bcmsdh_osinfo->oob_irq_num = wifi_platform_get_irq_number(adapter_info,
		&bcmsdh_osinfo->oob_irq_flags);
	if  (bcmsdh_osinfo->oob_irq_num < 0) {
		SDLX_MSG(("%s: Host OOB irq is not defined\n", __FUNCTION__));
		goto err;
	}
#endif 

	
	vendevid = bcmsdh_query_device(bcmsdh);
	
	bcmsdh_osinfo->context = drvinfo.probe((vendevid >> 16), (vendevid & 0xFFFF), bus_num,
		slot_num, 0, bus_type, (void *)regs, osh, bcmsdh);
	if (bcmsdh_osinfo->context == NULL) {
		SDLX_MSG(("%s: device attach failed\n", __FUNCTION__));
		goto err;
	}

	return bcmsdh;

	
err:
	if (bcmsdh != NULL)
		bcmsdh_detach(osh, bcmsdh);
	if (bcmsdh_osinfo != NULL)
		MFREE(osh, bcmsdh_osinfo, sizeof(bcmsdh_os_info_t));
	return NULL;
}

int bcmsdh_remove(bcmsdh_info_t *bcmsdh)
{
	bcmsdh_os_info_t *bcmsdh_osinfo = bcmsdh->os_cxt;

#if !defined(CONFIG_HAS_WAKELOCK) && (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36))
	if (bcmsdh_osinfo->dev)
		device_init_wakeup(bcmsdh_osinfo->dev, false);
	bcmsdh_osinfo->dev_wake_enabled = FALSE;
#endif 

	drvinfo.remove(bcmsdh_osinfo->context);
	MFREE(bcmsdh->osh, bcmsdh->os_cxt, sizeof(bcmsdh_os_info_t));
	bcmsdh_detach(bcmsdh->osh, bcmsdh);

	return 0;
}

int bcmsdh_suspend(bcmsdh_info_t *bcmsdh)
{
	bcmsdh_os_info_t *bcmsdh_osinfo = bcmsdh->os_cxt;

	if (drvinfo.suspend && drvinfo.suspend(bcmsdh_osinfo->context))
		return -EBUSY;
	return 0;
}

int bcmsdh_resume(bcmsdh_info_t *bcmsdh)
{
	bcmsdh_os_info_t *bcmsdh_osinfo = bcmsdh->os_cxt;

	if (drvinfo.resume)
		return drvinfo.resume(bcmsdh_osinfo->context);
	return 0;
}

extern int bcmsdh_register_client_driver(void);
extern void bcmsdh_unregister_client_driver(void);
extern int sdio_func_reg_notify(void* semaphore);
extern void sdio_func_unreg_notify(void);

#if defined(BCMLXSDMMC)
int bcmsdh_reg_sdio_notify(void* semaphore)
{
	return sdio_func_reg_notify(semaphore);
}

void bcmsdh_unreg_sdio_notify(void)
{
	sdio_func_unreg_notify();
}
#endif 

int
bcmsdh_register(bcmsdh_driver_t *driver)
{
	int error = 0;

	drvinfo = *driver;
	SDLX_MSG(("%s: register client driver\n", __FUNCTION__));
	error = bcmsdh_register_client_driver();
	if (error)
		SDLX_MSG(("%s: failed %d\n", __FUNCTION__, error));

	return error;
}

void
bcmsdh_unregister(void)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0))
		if (bcmsdh_pci_driver.node.next == NULL)
			return;
#endif

	bcmsdh_unregister_client_driver();
}

void bcmsdh_dev_pm_stay_awake(bcmsdh_info_t *bcmsdh)
{
#if !defined(CONFIG_HAS_WAKELOCK) && (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36))
	bcmsdh_os_info_t *bcmsdh_osinfo = bcmsdh->os_cxt;
	pm_stay_awake(bcmsdh_osinfo->dev);
#endif 
}

void bcmsdh_dev_relax(bcmsdh_info_t *bcmsdh)
{
#if !defined(CONFIG_HAS_WAKELOCK) && (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36))
	bcmsdh_os_info_t *bcmsdh_osinfo = bcmsdh->os_cxt;
	pm_relax(bcmsdh_osinfo->dev);
#endif 
}

bool bcmsdh_dev_pm_enabled(bcmsdh_info_t *bcmsdh)
{
	bcmsdh_os_info_t *bcmsdh_osinfo = bcmsdh->os_cxt;

	return bcmsdh_osinfo->dev_wake_enabled;
}

#if defined(OOB_INTR_ONLY)
void bcmsdh_oob_intr_set(bcmsdh_info_t *bcmsdh, bool enable)
{
	unsigned long flags;
	bcmsdh_os_info_t *bcmsdh_osinfo;

	if (!bcmsdh)
		return;

	bcmsdh_osinfo = bcmsdh->os_cxt;
	spin_lock_irqsave(&bcmsdh_osinfo->oob_irq_spinlock, flags);
	if (bcmsdh_osinfo->oob_irq_enabled != enable) {
		if (enable)
			enable_irq(bcmsdh_osinfo->oob_irq_num);
		else
			disable_irq_nosync(bcmsdh_osinfo->oob_irq_num);
		bcmsdh_osinfo->oob_irq_enabled = enable;
	}
	spin_unlock_irqrestore(&bcmsdh_osinfo->oob_irq_spinlock, flags);
}

static irqreturn_t wlan_oob_irq(int irq, void *dev_id)
{
	bcmsdh_info_t *bcmsdh = (bcmsdh_info_t *)dev_id;
	bcmsdh_os_info_t *bcmsdh_osinfo = bcmsdh->os_cxt;

	bcmsdh_oob_intr_set(bcmsdh, FALSE);
	bcmsdh_osinfo->oob_irq_handler(bcmsdh_osinfo->oob_irq_handler_context);

	return IRQ_HANDLED;
}

int bcmsdh_oob_intr_register(bcmsdh_info_t *bcmsdh, bcmsdh_cb_fn_t oob_irq_handler,
	void* oob_irq_handler_context)
{
	int err = 0;
	bcmsdh_os_info_t *bcmsdh_osinfo = bcmsdh->os_cxt;

	SDLX_MSG(("%s: Enter\n", __FUNCTION__));
	if (bcmsdh_osinfo->oob_irq_registered) {
		SDLX_MSG(("%s: irq is already registered\n", __FUNCTION__));
		return -EBUSY;
	}
	SDLX_MSG(("%s OOB irq=%d flags=%X \n", __FUNCTION__,
		(int)bcmsdh_osinfo->oob_irq_num, (int)bcmsdh_osinfo->oob_irq_flags));
	bcmsdh_osinfo->oob_irq_handler = oob_irq_handler;
	bcmsdh_osinfo->oob_irq_handler_context = oob_irq_handler_context;
#if defined(CONFIG_ARCH_ODIN)
	err = odin_gpio_sms_request_irq(bcmsdh_osinfo->oob_irq_num, wlan_oob_irq,
		bcmsdh_osinfo->oob_irq_flags, "bcmsdh_sdmmc", bcmsdh);
#else
	err = request_irq(bcmsdh_osinfo->oob_irq_num, wlan_oob_irq,
		bcmsdh_osinfo->oob_irq_flags, "bcmsdh_sdmmc", bcmsdh);
#endif 
	if (err) {
		SDLX_MSG(("%s: request_irq failed with %d\n", __FUNCTION__, err));
		return err;
	}

		err = enable_irq_wake(bcmsdh_osinfo->oob_irq_num);
		if (!err)
			bcmsdh_osinfo->oob_irq_wake_enabled = TRUE;
	bcmsdh_osinfo->oob_irq_enabled = TRUE;
	bcmsdh_osinfo->oob_irq_registered = TRUE;
	return err;
}

void bcmsdh_oob_intr_unregister(bcmsdh_info_t *bcmsdh)
{
	int err = 0;
	bcmsdh_os_info_t *bcmsdh_osinfo = bcmsdh->os_cxt;

	SDLX_MSG(("%s: Enter\n", __FUNCTION__));
	if (!bcmsdh_osinfo->oob_irq_registered) {
		SDLX_MSG(("%s: irq is not registered\n", __FUNCTION__));
		return;
	}
	if (bcmsdh_osinfo->oob_irq_wake_enabled) {
			err = disable_irq_wake(bcmsdh_osinfo->oob_irq_num);
			if (!err)
				bcmsdh_osinfo->oob_irq_wake_enabled = FALSE;
	}
	if (bcmsdh_osinfo->oob_irq_enabled) {
		disable_irq(bcmsdh_osinfo->oob_irq_num);
		bcmsdh_osinfo->oob_irq_enabled = FALSE;
	}
	free_irq(bcmsdh_osinfo->oob_irq_num, bcmsdh);
	bcmsdh_osinfo->oob_irq_registered = FALSE;
}
#endif 


extern uint sd_msglevel;	
module_param(sd_msglevel, uint, 0);

extern uint sd_power;	
module_param(sd_power, uint, 0);

extern uint sd_clock;	
module_param(sd_clock, uint, 0);

extern uint sd_divisor;	
module_param(sd_divisor, uint, 0);

extern uint sd_sdmode;	
module_param(sd_sdmode, uint, 0);

extern uint sd_hiok;	
module_param(sd_hiok, uint, 0);

extern uint sd_f2_blocksize;
module_param(sd_f2_blocksize, int, 0);

#ifdef BCMSDIOH_STD
extern int sd_uhsimode;
module_param(sd_uhsimode, int, 0);
extern uint sd_tuning_period;
module_param(sd_tuning_period, uint, 0);
extern int sd_delay_value;
module_param(sd_delay_value, uint, 0);

extern char dhd_sdiod_uhsi_ds_override[2];
module_param_string(dhd_sdiod_uhsi_ds_override, dhd_sdiod_uhsi_ds_override, 2, 0);

#endif

#ifdef BCMSDH_MODULE
EXPORT_SYMBOL(bcmsdh_attach);
EXPORT_SYMBOL(bcmsdh_detach);
EXPORT_SYMBOL(bcmsdh_intr_query);
EXPORT_SYMBOL(bcmsdh_intr_enable);
EXPORT_SYMBOL(bcmsdh_intr_disable);
EXPORT_SYMBOL(bcmsdh_intr_reg);
EXPORT_SYMBOL(bcmsdh_intr_dereg);

#if defined(DHD_DEBUG)
EXPORT_SYMBOL(bcmsdh_intr_pending);
#endif

EXPORT_SYMBOL(bcmsdh_devremove_reg);
EXPORT_SYMBOL(bcmsdh_cfg_read);
EXPORT_SYMBOL(bcmsdh_cfg_write);
EXPORT_SYMBOL(bcmsdh_cis_read);
EXPORT_SYMBOL(bcmsdh_reg_read);
EXPORT_SYMBOL(bcmsdh_reg_write);
EXPORT_SYMBOL(bcmsdh_regfail);
EXPORT_SYMBOL(bcmsdh_send_buf);
EXPORT_SYMBOL(bcmsdh_recv_buf);

EXPORT_SYMBOL(bcmsdh_rwdata);
EXPORT_SYMBOL(bcmsdh_abort);
EXPORT_SYMBOL(bcmsdh_query_device);
EXPORT_SYMBOL(bcmsdh_query_iofnum);
EXPORT_SYMBOL(bcmsdh_iovar_op);
EXPORT_SYMBOL(bcmsdh_register);
EXPORT_SYMBOL(bcmsdh_unregister);
EXPORT_SYMBOL(bcmsdh_chipmatch);
EXPORT_SYMBOL(bcmsdh_reset);
EXPORT_SYMBOL(bcmsdh_waitlockfree);

EXPORT_SYMBOL(bcmsdh_get_dstatus);
EXPORT_SYMBOL(bcmsdh_cfg_read_word);
EXPORT_SYMBOL(bcmsdh_cfg_write_word);
EXPORT_SYMBOL(bcmsdh_cur_sbwad);
EXPORT_SYMBOL(bcmsdh_chipinfo);

#endif 
