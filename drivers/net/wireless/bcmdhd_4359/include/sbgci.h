/*
 * SiliconBackplane GCI core hardware definitions
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
 * $Id: sbgci.h 514727 2014-11-12 03:02:48Z $
 */

#ifndef _SBGCI_H
#define _SBGCI_H

#if !defined(_LANGUAGE_ASSEMBLY) && !defined(__ASSEMBLY__)

#ifndef PAD
#define	_PADLINE(line)	pad ## line
#define	_XSTR(line)	_PADLINE(line)
#define	PAD		_XSTR(__LINE__)
#endif	

typedef volatile struct {
	uint32	gci_corecaps0;		
	uint32	gci_corecaps1;		
	uint32	gci_corecaps2;		
	uint32	gci_corectrl;		
	uint32	gci_corestat;		
	uint32	gci_intstat;		
	uint32	gci_intmask;		
	uint32	gci_wakemask;		
	uint32	gci_levelintstat;	
	uint32	gci_eventintstat;	
	uint32	gci_wakelevelintstat;	
	uint32	gci_wakeeventintstat;	
	uint32	semaphoreintstatus;	
	uint32	semaphoreintmask;	
	uint32	semaphorerequest;	
	uint32	semaphorereserve;	
	uint32	gci_indirect_addr;	
	uint32	gci_gpioctl;		
	uint32	gci_gpiostatus;		
	uint32	gci_gpiomask;		
	uint32	eventsummary;		
	uint32	gci_miscctl;		
	uint32	gci_gpiointmask;	
	uint32	gci_gpiowakemask;	
	uint32	gci_input[32];		
	uint32	gci_event[32];		
	uint32	gci_output[4];		
	uint32	gci_control_0;		
	uint32	gci_control_1;		
	uint32	gci_intpolreg;		
	uint32	gci_levelintmask;	
	uint32	gci_eventintmask;	
	uint32	wakelevelintmask;	
	uint32	wakeeventintmask;	
	uint32	hwmask;			
	uint32	PAD;
	uint32	gci_inbandeventintmask;	
	uint32	PAD;
	uint32	gci_inbandeventstatus;	
	uint32	gci_seciauxtx;		
	uint32	gci_seciauxrx;		
	uint32	gci_secitx_datatag;	
	uint32	gci_secirx_datatag;	
	uint32	gci_secitx_datamask;	
	uint32	gci_seciusef0tx_reg;	
	uint32	gci_secif0tx_offset;	
	uint32	gci_secif0rx_offset;	
	uint32	gci_secif1tx_offset;	
	uint32	gci_rxfifo_common_ctrl;	
	uint32	gci_rxfifoctrl;		
	uint32	PAD;
	uint32	gci_seciuartescval;	
	uint32	gic_seciuartautobaudctr;	
	uint32	gci_secififolevel;	
	uint32	gci_seciuartdata;	
	uint32	gci_secibauddiv;	
	uint32	gci_secifcr;		
	uint32	gci_secilcr;		
	uint32	gci_secimcr;		
	uint32	gci_secilsr;		
	uint32	gci_secimsr;		
	uint32	gci_baudadj;		
	uint32	gci_inbandintmask;	
	uint32  gci_chipctrl;		
	uint32  gci_chipsts; 		
	uint32	gci_gpioout; 		
	uint32	gci_gpioout_read; 	
	uint32	gci_mpwaketx; 		
	uint32	gci_mpwakedetect; 	
	uint32	gci_seciin_ctrl; 	
	uint32	gci_seciout_ctrl; 	
	uint32	gci_seciin_auxfifo_en; 	
	uint32	gci_seciout_txen_txbr; 	
	uint32	gci_seciin_rxbrstatus; 	
	uint32	gci_seciin_rxerrstatus; 
	uint32	gci_seciin_fcstatus; 	
	uint32	gci_seciout_txstatus; 	
	uint32	gci_seciout_txbrstatus; 
	uint32	PAD[49];
	uint32	gci_chipid;		
	uint32	PAD[3];
	uint32	otpstatus;		
	uint32	otpcontrol;		
	uint32	otpprog;		
	uint32	otplayout;		
	uint32	otplayoutextension;	
	uint32	otpcontrol1;		
} gciregs_t;

#endif 


#endif	
