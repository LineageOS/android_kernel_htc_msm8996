/*
 * BCM43XX PCIE core hardware definitions.
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
 * $Id: pcie_core.h 514727 2014-11-12 03:02:48Z $
 */
#ifndef	_PCIE_CORE_H
#define	_PCIE_CORE_H

#include <sbhnddma.h>
#include <siutils.h>

#ifndef PAD
#define	_PADLINE(line)	pad ## line
#define	_XSTR(line)	_PADLINE(line)
#define	PAD		_XSTR(__LINE__)
#endif

#define  PCIE_CORE_CONFIG_OFFSET	0x0
#define  PCIE_FUNC0_CONFIG_OFFSET	0x400
#define  PCIE_FUNC1_CONFIG_OFFSET	0x500
#define  PCIE_FUNC2_CONFIG_OFFSET	0x600
#define  PCIE_FUNC3_CONFIG_OFFSET	0x700
#define  PCIE_SPROM_SHADOW_OFFSET	0x800
#define  PCIE_SBCONFIG_OFFSET		0xE00


#define PCIEDEV_MAX_DMAS			4

#define PCIE_DEV_BAR0_SIZE		0x4000
#define PCIE_BAR0_WINMAPCORE_OFFSET	0x0
#define PCIE_BAR0_EXTSPROM_OFFSET	0x1000
#define PCIE_BAR0_PCIECORE_OFFSET	0x2000
#define PCIE_BAR0_CCCOREREG_OFFSET	0x3000

#define PCIE_CONFIGREGS 	1		
#define PCIE_PCIEREGS 		2		

typedef struct pcie_devdmaregs {
	dma64regs_t	tx;
	uint32		PAD[2];
	dma64regs_t	rx;
	uint32		PAD[2];
} pcie_devdmaregs_t;

#define PCIE_DB_HOST2DEV_0		0x1
#define PCIE_DB_HOST2DEV_1		0x2
#define PCIE_DB_DEV2HOST_0		0x3
#define PCIE_DB_DEV2HOST_1		0x4

typedef struct pcie_doorbell {
	uint32		host2dev_0;
	uint32		host2dev_1;
	uint32		dev2host_0;
	uint32		dev2host_1;
} pcie_doorbell_t;

typedef struct sbpcieregs {
	uint32 control;		
	uint32 iocstatus;	
	uint32 PAD[1];
	uint32 biststatus;	
	uint32 gpiosel;		
	uint32 gpioouten;	
	uint32 PAD[2];
	uint32 intstatus;	
	uint32 intmask;		
	uint32 sbtopcimailbox;	
	uint32 obffcontrol;	
	uint32 obffintstatus;	
	uint32 obffdatastatus;	
	uint32 PAD[2];
	uint32 errlog;		
	uint32 errlogaddr;	
	uint32 mailboxint;	
	uint32 mailboxintmsk; 
	uint32 ltrspacing;	
	uint32 ltrhysteresiscnt;	
	uint32 PAD[42];

	uint32 sbtopcie0;	
	uint32 sbtopcie1;	
	uint32 sbtopcie2;	
	uint32 PAD[5];

	
	uint32 configaddr;	
	uint32 configdata;	
	union {
		struct {
			
			uint32 mdiocontrol;	
			uint32 mdiodata;	
			
			uint32 pcieindaddr; 
			uint32 pcieinddata;	
			uint32 clkreqenctrl;	
			uint32 PAD[177];
		} pcie1;
		struct {
			
			uint32 mdiocontrol;	
			uint32 mdiowrdata;	
			uint32 mdiorddata;	
			uint32	PAD[3]; 	
			
			pcie_doorbell_t	   dbls[PCIEDEV_MAX_DMAS]; 
			uint32	dataintf;	
			uint32  PAD[1];		
			uint32	d2h_intrlazy_0; 
			uint32	h2d_intrlazy_0; 
			uint32  h2d_intstat_0;  
			uint32  h2d_intmask_0;	
			uint32  d2h_intstat_0;  
			uint32  d2h_intmask_0;  
			uint32	ltr_state;	
			uint32	pwr_int_status;	
			uint32	pwr_int_mask;	
			uint32  PAD[13]; 	
			uint32  clk_ctl_st;	
			uint32  PAD[7]; 	
			pcie_devdmaregs_t  h2d0_dmaregs; 
			pcie_devdmaregs_t  d2h0_dmaregs; 
			pcie_devdmaregs_t  h2d1_dmaregs; 
			pcie_devdmaregs_t  d2h1_dmaregs; 
			pcie_devdmaregs_t  h2d2_dmaregs; 
			pcie_devdmaregs_t  d2h2_dmaregs; 
			pcie_devdmaregs_t  h2d3_dmaregs; 
			pcie_devdmaregs_t  d2h3_dmaregs; 
		} pcie2;
	} u;
	uint32 pciecfg[4][64];	
	uint16 sprom[64];	
} sbpcieregs_t;

#define PCIE_RST_OE	0x01	
#define PCIE_RST	0x02	
#define PCIE_SPERST	0x04	
#define PCIE_DISABLE_L1CLK_GATING	0x10
#define PCIE_DLYPERST	0x100	
#define PCIE_DISSPROMLD	0x200	
#define PCIE_WakeModeL2	0x1000	
#define PCIE_PipeIddqDisable0	0x8000	
#define PCIE_PipeIddqDisable1	0x10000	

#define	PCIE_CFGADDR	0x120	
#define	PCIE_CFGDATA	0x124	

#define PCIE_INTA	0x01	
#define PCIE_INTB	0x02	
#define PCIE_INTFATAL	0x04	
#define PCIE_INTNFATAL	0x08	
#define PCIE_INTCORR	0x10	
#define PCIE_INTPME	0x20	
#define PCIE_PERST	0x40	

#define PCIE_INT_MB_FN0_0 0x0100 
#define PCIE_INT_MB_FN0_1 0x0200 
#define PCIE_INT_MB_FN1_0 0x0400 
#define PCIE_INT_MB_FN1_1 0x0800 
#define PCIE_INT_MB_FN2_0 0x1000 
#define PCIE_INT_MB_FN2_1 0x2000 
#define PCIE_INT_MB_FN3_0 0x4000 
#define PCIE_INT_MB_FN3_1 0x8000 

#define PCIE_MB_TOSB_FN0_0   	0x0001 
#define PCIE_MB_TOSB_FN0_1   	0x0002
#define PCIE_MB_TOSB_FN1_0   	0x0004
#define PCIE_MB_TOSB_FN1_1   	0x0008
#define PCIE_MB_TOSB_FN2_0   	0x0010
#define PCIE_MB_TOSB_FN2_1   	0x0020
#define PCIE_MB_TOSB_FN3_0   	0x0040
#define PCIE_MB_TOSB_FN3_1   	0x0080
#define PCIE_MB_TOPCIE_FN0_0 	0x0100 
#define PCIE_MB_TOPCIE_FN0_1 	0x0200
#define PCIE_MB_TOPCIE_FN1_0 	0x0400
#define PCIE_MB_TOPCIE_FN1_1 	0x0800
#define PCIE_MB_TOPCIE_FN2_0 	0x1000
#define PCIE_MB_TOPCIE_FN2_1 	0x2000
#define PCIE_MB_TOPCIE_FN3_0 	0x4000
#define PCIE_MB_TOPCIE_FN3_1 	0x8000
#define	PCIE_MB_TOPCIE_D2H0_DB0	0x10000
#define	PCIE_MB_TOPCIE_D2H0_DB1	0x20000
#define	PCIE_MB_TOPCIE_D2H1_DB0	0x40000
#define	PCIE_MB_TOPCIE_D2H1_DB1	0x80000
#define	PCIE_MB_TOPCIE_D2H2_DB0	0x100000
#define	PCIE_MB_TOPCIE_D2H2_DB1	0x200000
#define	PCIE_MB_TOPCIE_D2H3_DB0	0x400000
#define	PCIE_MB_TOPCIE_D2H3_DB1	0x800000

#define PCIE_MB_D2H_MB_MASK		\
	(PCIE_MB_TOPCIE_D2H0_DB0 | PCIE_MB_TOPCIE_D2H0_DB1 |	\
	PCIE_MB_TOPCIE_D2H1_DB0  | PCIE_MB_TOPCIE_D2H1_DB1 |	\
	PCIE_MB_TOPCIE_D2H2_DB0  | PCIE_MB_TOPCIE_D2H2_DB1 |	\
	PCIE_MB_TOPCIE_D2H3_DB0  | PCIE_MB_TOPCIE_D2H3_DB1)

#define SBTOPCIE0_MASK	0xfc000000
#define SBTOPCIE1_MASK	0xfc000000
#define SBTOPCIE2_MASK	0xc0000000

#define SBTOPCIE_MEM	0
#define SBTOPCIE_IO	1
#define SBTOPCIE_CFG0	2
#define SBTOPCIE_CFG1	3

#define SBTOPCIE_PF		4

#define SBTOPCIE_WR_BURST	8

#define CONFIGADDR_FUNC_MASK	0x7000
#define CONFIGADDR_FUNC_SHF	12
#define CONFIGADDR_REG_MASK	0x0FFF
#define CONFIGADDR_REG_SHF	0

#define PCIE_CONFIG_INDADDR(f, r)	((((f) & CONFIGADDR_FUNC_MASK) << CONFIGADDR_FUNC_SHF) | \
			                 (((r) & CONFIGADDR_REG_MASK) << CONFIGADDR_REG_SHF))

#define PCIEADDR_PROT_MASK	0x300
#define PCIEADDR_PROT_SHF	8
#define PCIEADDR_PL_TLP		0
#define PCIEADDR_PL_DLLP	1
#define PCIEADDR_PL_PLP		2

#define	PCIE_PLP_MODEREG		0x200 
#define	PCIE_PLP_STATUSREG		0x204 
#define PCIE_PLP_LTSSMCTRLREG		0x208 
#define PCIE_PLP_LTLINKNUMREG		0x20c 
#define PCIE_PLP_LTLANENUMREG		0x210 
#define PCIE_PLP_LTNFTSREG		0x214 
#define PCIE_PLP_ATTNREG		0x218 
#define PCIE_PLP_ATTNMASKREG		0x21C 
#define PCIE_PLP_RXERRCTR		0x220 
#define PCIE_PLP_RXFRMERRCTR		0x224 
#define PCIE_PLP_RXERRTHRESHREG		0x228 
#define PCIE_PLP_TESTCTRLREG		0x22C 
#define PCIE_PLP_SERDESCTRLOVRDREG	0x230 
#define PCIE_PLP_TIMINGOVRDREG		0x234 
#define PCIE_PLP_RXTXSMDIAGREG		0x238 
#define PCIE_PLP_LTSSMDIAGREG		0x23C 

#define PCIE_DLLP_LCREG			0x100 
#define PCIE_DLLP_LSREG			0x104 
#define PCIE_DLLP_LAREG			0x108 
#define PCIE_DLLP_LAMASKREG		0x10C 
#define PCIE_DLLP_NEXTTXSEQNUMREG	0x110 
#define PCIE_DLLP_ACKEDTXSEQNUMREG	0x114 
#define PCIE_DLLP_PURGEDTXSEQNUMREG	0x118 
#define PCIE_DLLP_RXSEQNUMREG		0x11C 
#define PCIE_DLLP_LRREG			0x120 
#define PCIE_DLLP_LACKTOREG		0x124 
#define PCIE_DLLP_PMTHRESHREG		0x128 
#define PCIE_DLLP_RTRYWPREG		0x12C 
#define PCIE_DLLP_RTRYRPREG		0x130 
#define PCIE_DLLP_RTRYPPREG		0x134 
#define PCIE_DLLP_RTRRWREG		0x138 
#define PCIE_DLLP_ECTHRESHREG		0x13C 
#define PCIE_DLLP_TLPERRCTRREG		0x140 
#define PCIE_DLLP_ERRCTRREG		0x144 
#define PCIE_DLLP_NAKRXCTRREG		0x148 
#define PCIE_DLLP_TESTREG		0x14C 
#define PCIE_DLLP_PKTBIST		0x150 
#define PCIE_DLLP_PCIE11		0x154 

#define PCIE_DLLP_LSREG_LINKUP		(1 << 16)

#define PCIE_TLP_CONFIGREG		0x000 
#define PCIE_TLP_WORKAROUNDSREG		0x004 
#define PCIE_TLP_WRDMAUPPER		0x010 
#define PCIE_TLP_WRDMALOWER		0x014 
#define PCIE_TLP_WRDMAREQ_LBEREG	0x018 
#define PCIE_TLP_RDDMAUPPER		0x01C 
#define PCIE_TLP_RDDMALOWER		0x020 
#define PCIE_TLP_RDDMALENREG		0x024 
#define PCIE_TLP_MSIDMAUPPER		0x028 
#define PCIE_TLP_MSIDMALOWER		0x02C 
#define PCIE_TLP_MSIDMALENREG		0x030 
#define PCIE_TLP_SLVREQLENREG		0x034 
#define PCIE_TLP_FCINPUTSREQ		0x038 
#define PCIE_TLP_TXSMGRSREQ		0x03C 
#define PCIE_TLP_ADRACKCNTARBLEN	0x040 
#define PCIE_TLP_DMACPLHDR0		0x044 
#define PCIE_TLP_DMACPLHDR1		0x048 
#define PCIE_TLP_DMACPLHDR2		0x04C 
#define PCIE_TLP_DMACPLMISC0		0x050 
#define PCIE_TLP_DMACPLMISC1		0x054 
#define PCIE_TLP_DMACPLMISC2		0x058 
#define PCIE_TLP_SPTCTRLLEN		0x05C 
#define PCIE_TLP_SPTCTRLMSIC0		0x060 
#define PCIE_TLP_SPTCTRLMSIC1		0x064 
#define PCIE_TLP_BUSDEVFUNC		0x068 
#define PCIE_TLP_RESETCTR		0x06C 
#define PCIE_TLP_RTRYBUF		0x070 
#define PCIE_TLP_TGTDEBUG1		0x074 
#define PCIE_TLP_TGTDEBUG2		0x078 
#define PCIE_TLP_TGTDEBUG3		0x07C 
#define PCIE_TLP_TGTDEBUG4		0x080 

#define PCIE2_MDIO_CONTROL    0x128
#define PCIE2_MDIO_WR_DATA    0x12C
#define PCIE2_MDIO_RD_DATA    0x130


#define MDIOCTL_DIVISOR_MASK		0x7f	
#define MDIOCTL_DIVISOR_VAL		0x2
#define MDIOCTL_PREAM_EN		0x80	
#define MDIOCTL_ACCESS_DONE		0x100   

#define MDIODATA_MASK			0x0000ffff	
#define MDIODATA_TA			0x00020000	
#define MDIODATA_REGADDR_SHF_OLD	18		
#define MDIODATA_REGADDR_MASK_OLD	0x003c0000	
#define MDIODATA_DEVADDR_SHF_OLD	22		
#define MDIODATA_DEVADDR_MASK_OLD	0x0fc00000	
#define MDIODATA_REGADDR_SHF		18		
#define MDIODATA_REGADDR_MASK		0x007c0000	
#define MDIODATA_DEVADDR_SHF		23		
#define MDIODATA_DEVADDR_MASK		0x0f800000	
#define MDIODATA_WRITE			0x10000000	
#define MDIODATA_READ			0x20000000	
#define MDIODATA_START			0x40000000	

#define MDIODATA_DEV_ADDR		0x0		
#define	MDIODATA_BLK_ADDR		0x1F		

#define MDIOCTL2_DIVISOR_MASK		0x7f	
#define MDIOCTL2_DIVISOR_VAL		0x2
#define MDIOCTL2_REGADDR_SHF		8		
#define MDIOCTL2_REGADDR_MASK		0x00FFFF00	
#define MDIOCTL2_DEVADDR_SHF		24		
#define MDIOCTL2_DEVADDR_MASK		0x0f000000	
#define MDIOCTL2_SLAVE_BYPASS		0x10000000	
#define MDIOCTL2_READ			0x20000000	

#define MDIODATA2_DONE			0x80000000	
#define MDIODATA2_MASK			0x7FFFFFFF	
#define MDIODATA2_DEVADDR_SHF		4		


#define MDIO_DEV_IEEE0		0x000
#define MDIO_DEV_IEEE1		0x001
#define MDIO_DEV_BLK0		0x800
#define MDIO_DEV_BLK1		0x801
#define MDIO_DEV_BLK2		0x802
#define MDIO_DEV_BLK3		0x803
#define MDIO_DEV_BLK4		0x804
#define MDIO_DEV_TXPLL		0x808	
#define MDIO_DEV_TXCTRL0	0x820
#define MDIO_DEV_SERDESID	0x831
#define MDIO_DEV_RXCTRL0	0x840


#define BLK1_PWR_MGMT0		0x16
#define BLK1_PWR_MGMT1		0x17
#define BLK1_PWR_MGMT2		0x18
#define BLK1_PWR_MGMT3		0x19
#define BLK1_PWR_MGMT4		0x1A

#define MDIODATA_DEV_PLL       		0x1d	
#define MDIODATA_DEV_TX        		0x1e	
#define MDIODATA_DEV_RX        		0x1f	
	
#define SERDES_RX_CTRL			1	
#define SERDES_RX_TIMER1		2	
#define SERDES_RX_CDR			6	
#define SERDES_RX_CDRBW			7	

	
#define SERDES_RX_CTRL_FORCE		0x80	
#define SERDES_RX_CTRL_POLARITY		0x40	

	
#define SERDES_PLL_CTRL                 1       
#define PLL_CTRL_FREQDET_EN             0x4000  

#define PCIE_L0THRESHOLDTIME_MASK       0xFF00	
#define PCIE_L1THRESHOLDTIME_MASK       0xFF00	
#define PCIE_L1THRESHOLDTIME_SHIFT      8	
#define PCIE_L1THRESHOLD_WARVAL         0x72	
#define PCIE_ASPMTIMER_EXTEND		0x01000000	

#define SRSH_ASPM_OFFSET		4	
#define SRSH_ASPM_ENB			0x18	
#define SRSH_ASPM_L1_ENB		0x10	
#define SRSH_ASPM_L0s_ENB		0x8	
#define SRSH_PCIE_MISC_CONFIG		5	
#define SRSH_L23READY_EXIT_NOPERST	0x8000	
#define SRSH_CLKREQ_OFFSET_REV5		20	
#define SRSH_CLKREQ_OFFSET_REV8		52	
#define SRSH_CLKREQ_ENB			0x0800	
#define SRSH_BD_OFFSET                  6       
#define SRSH_AUTOINIT_OFFSET            18      

#define PCIE_CAP_LINKCTRL_OFFSET	16	
#define PCIE_CAP_LCREG_ASPML0s		0x01	
#define PCIE_CAP_LCREG_ASPML1		0x02	
#define PCIE_CLKREQ_ENAB		0x100	
#define PCIE_LINKSPEED_MASK       	0xF0000	
#define PCIE_LINKSPEED_SHIFT      	16	

#define PCIE_CAP_DEVCTRL_OFFSET		8	
#define PCIE_CAP_DEVCTRL_MRRS_MASK	0x7000	
#define PCIE_CAP_DEVCTRL_MRRS_SHIFT	12	
#define PCIE_CAP_DEVCTRL_MRRS_128B	0	
#define PCIE_CAP_DEVCTRL_MRRS_256B	1	
#define PCIE_CAP_DEVCTRL_MRRS_512B	2	
#define PCIE_CAP_DEVCTRL_MRRS_1024B	3	
#define PCIE_CAP_DEVCTRL_MPS_MASK	0x00e0	
#define PCIE_CAP_DEVCTRL_MPS_SHIFT	5	
#define PCIE_CAP_DEVCTRL_MPS_128B	0	
#define PCIE_CAP_DEVCTRL_MPS_256B	1	
#define PCIE_CAP_DEVCTRL_MPS_512B	2	
#define PCIE_CAP_DEVCTRL_MPS_1024B	3	

#define PCIE_ASPM_ENAB			3	
#define PCIE_ASPM_L1_ENAB		2	
#define PCIE_ASPM_L0s_ENAB		1	
#define PCIE_ASPM_DISAB			0	

#define PCIE_ASPM_L11_ENAB		8	
#define PCIE_ASPM_L12_ENAB		4	

#define PCIE_CAP_DEVCTRL2_OFFSET	0x28	
#define PCIE_CAP_DEVCTRL2_LTR_ENAB_MASK	0x400	
#define PCIE_CAP_DEVCTRL2_OBFF_ENAB_SHIFT 13	
#define PCIE_CAP_DEVCTRL2_OBFF_ENAB_MASK 0x6000	

#define PCIE_LTR0_REG_OFFSET	0x844	
#define PCIE_LTR1_REG_OFFSET	0x848	
#define PCIE_LTR2_REG_OFFSET	0x84c	
#define PCIE_LTR0_REG_DEFAULT_60	0x883c883c	
#define PCIE_LTR0_REG_DEFAULT_150	0x88968896	
#define PCIE_LTR1_REG_DEFAULT		0x88648864	
#define PCIE_LTR2_REG_DEFAULT		0x90039003	

#define PCIE_PLP_POLARITYINV_STAT	0x10


#define BRCMCAP_PCIEREV_CT_MASK			0xF00
#define BRCMCAP_PCIEREV_CT_SHIFT		8
#define BRCMCAP_PCIEREV_REVID_MASK		0xFF
#define BRCMCAP_PCIEREV_REVID_SHIFT		0

#define PCIE_REVREG_CT_PCIE1		0
#define PCIE_REVREG_CT_PCIE2		1

#define PCIE2R0_BRCMCAP_REVID_OFFSET		4
#define PCIE2R0_BRCMCAP_BAR0_WIN0_WRAP_OFFSET	8
#define PCIE2R0_BRCMCAP_BAR0_WIN2_OFFSET	12
#define PCIE2R0_BRCMCAP_BAR0_WIN2_WRAP_OFFSET	16
#define PCIE2R0_BRCMCAP_BAR0_WIN_OFFSET		20
#define PCIE2R0_BRCMCAP_BAR1_WIN_OFFSET		24
#define PCIE2R0_BRCMCAP_SPROM_CTRL_OFFSET	28
#define PCIE2R0_BRCMCAP_BAR2_WIN_OFFSET		32
#define PCIE2R0_BRCMCAP_INTSTATUS_OFFSET	36
#define PCIE2R0_BRCMCAP_INTMASK_OFFSET		40
#define PCIE2R0_BRCMCAP_PCIE2SB_MB_OFFSET	44
#define PCIE2R0_BRCMCAP_BPADDR_OFFSET		48
#define PCIE2R0_BRCMCAP_BPDATA_OFFSET		52
#define PCIE2R0_BRCMCAP_CLKCTLSTS_OFFSET	56

#define PCIECFGREG_STATUS_CMD		0x4
#define PCIECFGREG_PM_CSR		0x4C
#define PCIECFGREG_MSI_CAP		0x58
#define PCIECFGREG_MSI_ADDR_L		0x5C
#define PCIECFGREG_MSI_ADDR_H		0x60
#define PCIECFGREG_MSI_DATA		0x64
#define PCIECFGREG_LINK_STATUS_CTRL	0xBC
#define PCIECFGREG_LINK_STATUS_CTRL2	0xDC
#define PCIECFGREG_RBAR_CTRL		0x228
#define PCIECFGREG_PML1_SUB_CTRL1	0x248
#define PCIECFGREG_REG_BAR2_CONFIG	0x4E0
#define PCIECFGREG_REG_BAR3_CONFIG	0x4F4
#define PCIECFGREG_PDL_CTRL1		0x1004
#define PCIECFGREG_PDL_IDDQ		0x1814
#define PCIECFGREG_REG_PHY_CTL7		0x181c

#define PCI_PM_L1_2_ENA_MASK		0x00000001	
#define PCI_PM_L1_1_ENA_MASK		0x00000002	
#define ASPM_L1_2_ENA_MASK		0x00000004	
#define ASPM_L1_1_ENA_MASK		0x00000008	

#define I_MB    0x3
#define I_BIT0  0x1
#define I_BIT1  0x2

#define PCIIntstatus	0x090
#define PCIIntmask	0x094
#define PCISBMbx	0x98

#define PCIH2D_MailBox  0x140
#define PCIH2D_DB1 0x144
#define PCID2H_MailBox  0x148
#define PCIMailBoxInt	0x48
#define PCIMailBoxMask	0x4C

#define I_F0_B0         (0x1 << 8) 
#define I_F0_B1         (0x1 << 9) 

#define PCIECFGREG_DEVCONTROL	0xB4
#define PCIECFGREG_DEVCONTROL_MRRS_SHFT	12
#define PCIECFGREG_DEVCONTROL_MRRS_MASK	(0x7 << PCIECFGREG_DEVCONTROL_MRRS_SHFT)

#define SROM_OFFSET_BAR1_CTRL  52

#define BAR1_ENC_SIZE_MASK	0x000e
#define BAR1_ENC_SIZE_SHIFT	1

#define BAR1_ENC_SIZE_1M	0
#define BAR1_ENC_SIZE_2M	1
#define BAR1_ENC_SIZE_4M	2

#define PCIEGEN2_CAP_DEVSTSCTRL2_OFFSET		0xD4
#define PCIEGEN2_CAP_DEVSTSCTRL2_LTRENAB	0x400

#define LTR_ACTIVE				2
#define LTR_ACTIVE_IDLE				1
#define LTR_SLEEP				0
#define LTR_FINAL_MASK				0x300
#define LTR_FINAL_SHIFT				8

#define PCIEGEN2_PWRINT_D0_STATE_SHIFT		0
#define PCIEGEN2_PWRINT_D1_STATE_SHIFT		1
#define PCIEGEN2_PWRINT_D2_STATE_SHIFT		2
#define PCIEGEN2_PWRINT_D3_STATE_SHIFT		3
#define PCIEGEN2_PWRINT_L0_LINK_SHIFT		4
#define PCIEGEN2_PWRINT_L0s_LINK_SHIFT		5
#define PCIEGEN2_PWRINT_L1_LINK_SHIFT		6
#define PCIEGEN2_PWRINT_L2_L3_LINK_SHIFT	7
#define PCIEGEN2_PWRINT_OBFF_CHANGE_SHIFT	8

#define PCIEGEN2_PWRINT_D0_STATE_MASK		(1 << PCIEGEN2_PWRINT_D0_STATE_SHIFT)
#define PCIEGEN2_PWRINT_D1_STATE_MASK		(1 << PCIEGEN2_PWRINT_D1_STATE_SHIFT)
#define PCIEGEN2_PWRINT_D2_STATE_MASK		(1 << PCIEGEN2_PWRINT_D2_STATE_SHIFT)
#define PCIEGEN2_PWRINT_D3_STATE_MASK		(1 << PCIEGEN2_PWRINT_D3_STATE_SHIFT)
#define PCIEGEN2_PWRINT_L0_LINK_MASK		(1 << PCIEGEN2_PWRINT_L0_LINK_SHIFT)
#define PCIEGEN2_PWRINT_L0s_LINK_MASK		(1 << PCIEGEN2_PWRINT_L0s_LINK_SHIFT)
#define PCIEGEN2_PWRINT_L1_LINK_MASK		(1 << PCIEGEN2_PWRINT_L1_LINK_SHIFT)
#define PCIEGEN2_PWRINT_L2_L3_LINK_MASK		(1 << PCIEGEN2_PWRINT_L2_L3_LINK_SHIFT)
#define PCIEGEN2_PWRINT_OBFF_CHANGE_MASK	(1 << PCIEGEN2_PWRINT_OBFF_CHANGE_SHIFT)

#define SBTOPCIE_MB_FUNC0_SHIFT 8
#define SBTOPCIE_MB_FUNC1_SHIFT 10
#define SBTOPCIE_MB_FUNC2_SHIFT 12
#define SBTOPCIE_MB_FUNC3_SHIFT 14

#define PCIEGEN2_IOC_D0_STATE_SHIFT		8
#define PCIEGEN2_IOC_D1_STATE_SHIFT		9
#define PCIEGEN2_IOC_D2_STATE_SHIFT		10
#define PCIEGEN2_IOC_D3_STATE_SHIFT		11
#define PCIEGEN2_IOC_L0_LINK_SHIFT		12
#define PCIEGEN2_IOC_L1_LINK_SHIFT		13
#define PCIEGEN2_IOC_L1L2_LINK_SHIFT		14
#define PCIEGEN2_IOC_L2_L3_LINK_SHIFT		15

#define PCIEGEN2_IOC_D0_STATE_MASK		(1 << PCIEGEN2_IOC_D0_STATE_SHIFT)
#define PCIEGEN2_IOC_D1_STATE_MASK		(1 << PCIEGEN2_IOC_D1_STATE_SHIFT)
#define PCIEGEN2_IOC_D2_STATE_MASK		(1 << PCIEGEN2_IOC_D2_STATE_SHIFT)
#define PCIEGEN2_IOC_D3_STATE_MASK		(1 << PCIEGEN2_IOC_D3_STATE_SHIFT)
#define PCIEGEN2_IOC_L0_LINK_MASK		(1 << PCIEGEN2_IOC_L0_LINK_SHIFT)
#define PCIEGEN2_IOC_L1_LINK_MASK		(1 << PCIEGEN2_IOC_L1_LINK_SHIFT)
#define PCIEGEN2_IOC_L1L2_LINK_MASK		(1 << PCIEGEN2_IOC_L1L2_LINK_SHIFT)
#define PCIEGEN2_IOC_L2_L3_LINK_MASK		(1 << PCIEGEN2_IOC_L2_L3_LINK_SHIFT)

#define PCIE_STAT_CTRL_RESET		0x1
#define PCIE_STAT_CTRL_ENABLE		0x2
#define PCIE_STAT_CTRL_INTENABLE	0x4
#define PCIE_STAT_CTRL_INTSTATUS	0x8

#ifdef BCMDRIVER
void pcie_watchdog_reset(osl_t *osh, si_t *sih, sbpcieregs_t *sbpcieregs);
void pcie_serdes_iddqdisable(osl_t *osh, si_t *sih, sbpcieregs_t *sbpcieregs);
#endif 

#endif	
