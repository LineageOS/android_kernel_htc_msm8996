/*
 *  linux/drivers/mmc/core/mmc.c
 *
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  Copyright (C) 2005-2007 Pierre Ossman, All Rights Reserved.
 *  MMCv4 support Copyright (C) 2006 Philip Langdale, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/pm_runtime.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/reboot.h>
#include <trace/events/mmc.h>

#include "core.h"
#include "bus.h"
#include "mmc_ops.h"
#include "sd_ops.h"

static const unsigned int tran_exp[] = {
	10000,		100000,		1000000,	10000000,
	0,		0,		0,		0
};

static const unsigned char tran_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

static const unsigned int tacc_exp[] = {
	1,	10,	100,	1000,	10000,	100000,	1000000, 10000000,
};

static const unsigned int tacc_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

#define UNSTUFF_BITS(resp,start,size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})

static int mmc_decode_cid(struct mmc_card *card)
{
	u32 *resp = card->raw_cid;

	switch (card->csd.mmca_vsn) {
	case 0: 
	case 1: 
		card->cid.manfid	= UNSTUFF_BITS(resp, 104, 24);
		card->cid.prod_name[0]	= UNSTUFF_BITS(resp, 96, 8);
		card->cid.prod_name[1]	= UNSTUFF_BITS(resp, 88, 8);
		card->cid.prod_name[2]	= UNSTUFF_BITS(resp, 80, 8);
		card->cid.prod_name[3]	= UNSTUFF_BITS(resp, 72, 8);
		card->cid.prod_name[4]	= UNSTUFF_BITS(resp, 64, 8);
		card->cid.prod_name[5]	= UNSTUFF_BITS(resp, 56, 8);
		card->cid.prod_name[6]	= UNSTUFF_BITS(resp, 48, 8);
		card->cid.hwrev		= UNSTUFF_BITS(resp, 44, 4);
		card->cid.fwrev		= UNSTUFF_BITS(resp, 40, 4);
		card->cid.serial	= UNSTUFF_BITS(resp, 16, 24);
		card->cid.month		= UNSTUFF_BITS(resp, 12, 4);
		card->cid.year		= UNSTUFF_BITS(resp, 8, 4) + 1997;
		break;

	case 2: 
	case 3: 
	case 4: 
		card->cid.manfid	= UNSTUFF_BITS(resp, 120, 8);
		card->cid.oemid		= UNSTUFF_BITS(resp, 104, 16);
		card->cid.prod_name[0]	= UNSTUFF_BITS(resp, 96, 8);
		card->cid.prod_name[1]	= UNSTUFF_BITS(resp, 88, 8);
		card->cid.prod_name[2]	= UNSTUFF_BITS(resp, 80, 8);
		card->cid.prod_name[3]	= UNSTUFF_BITS(resp, 72, 8);
		card->cid.prod_name[4]	= UNSTUFF_BITS(resp, 64, 8);
		card->cid.prod_name[5]	= UNSTUFF_BITS(resp, 56, 8);
		card->cid.prv		= UNSTUFF_BITS(resp, 48, 8);
		card->cid.serial	= UNSTUFF_BITS(resp, 16, 32);
		card->cid.month		= UNSTUFF_BITS(resp, 12, 4);
		card->cid.year		= UNSTUFF_BITS(resp, 8, 4) + 1997;

		
		if (card->cid.manfid == CID_MANFID_TOSHIBA ||
		    card->cid.manfid == CID_MANFID_MICRON  ||
		    card->cid.manfid == CID_MANFID_SAMSUNG ||
		    card->cid.manfid == CID_MANFID_HYNIX)
			card->cid.fwrev = UNSTUFF_BITS(resp, 48, 8);

		break;

	default:
		pr_err("%s: card has unknown MMCA version %d\n",
			mmc_hostname(card->host), card->csd.mmca_vsn);
		return -EINVAL;
	}

	return 0;
}

static void mmc_set_erase_size(struct mmc_card *card)
{
	if (card->ext_csd.erase_group_def & 1)
		card->erase_size = card->ext_csd.hc_erase_size;
	else
		card->erase_size = card->csd.erase_size;

	mmc_init_erase(card);
}

static const struct mmc_fixup mmc_fixups[] = {

	
	MMC_FIXUP_EXT_CSD_REV("MMC16G", CID_MANFID_KINGSTON, CID_OEMID_ANY,
		add_quirk, MMC_QUIRK_BROKEN_HPI, MMC_V4_41),

	
	MMC_FIXUP("MMC16G", CID_MANFID_KINGSTON, CID_OEMID_ANY,
		add_quirk_mmc, MMC_QUIRK_CACHE_DISABLE),

	END_FIXUP
};

static int mmc_decode_csd(struct mmc_card *card)
{
	struct mmc_csd *csd = &card->csd;
	unsigned int e, m, a, b;
	u32 *resp = card->raw_csd;

	csd->structure = UNSTUFF_BITS(resp, 126, 2);
	if (csd->structure == 0) {
		pr_err("%s: unrecognised CSD structure version %d\n",
			mmc_hostname(card->host), csd->structure);
		return -EINVAL;
	}

	csd->mmca_vsn	 = UNSTUFF_BITS(resp, 122, 4);
	m = UNSTUFF_BITS(resp, 115, 4);
	e = UNSTUFF_BITS(resp, 112, 3);
	csd->tacc_ns	 = (tacc_exp[e] * tacc_mant[m] + 9) / 10;
	csd->tacc_clks	 = UNSTUFF_BITS(resp, 104, 8) * 100;

	m = UNSTUFF_BITS(resp, 99, 4);
	e = UNSTUFF_BITS(resp, 96, 3);
	csd->max_dtr	  = tran_exp[e] * tran_mant[m];
	csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

	e = UNSTUFF_BITS(resp, 47, 3);
	m = UNSTUFF_BITS(resp, 62, 12);
	csd->capacity	  = (1 + m) << (e + 2);

	csd->read_blkbits = UNSTUFF_BITS(resp, 80, 4);
	csd->read_partial = UNSTUFF_BITS(resp, 79, 1);
	csd->write_misalign = UNSTUFF_BITS(resp, 78, 1);
	csd->read_misalign = UNSTUFF_BITS(resp, 77, 1);
	csd->dsr_imp = UNSTUFF_BITS(resp, 76, 1);
	csd->r2w_factor = UNSTUFF_BITS(resp, 26, 3);
	csd->write_blkbits = UNSTUFF_BITS(resp, 22, 4);
	csd->write_partial = UNSTUFF_BITS(resp, 21, 1);

	if (csd->write_blkbits >= 9) {
		a = UNSTUFF_BITS(resp, 42, 5);
		b = UNSTUFF_BITS(resp, 37, 5);
		csd->erase_size = (a + 1) * (b + 1);
		csd->erase_size <<= csd->write_blkbits - 9;
	}

	return 0;
}

int mmc_get_ext_csd(struct mmc_card *card, u8 **new_ext_csd)
{
	int err;
	u8 *ext_csd;

	BUG_ON(!card);
	BUG_ON(!new_ext_csd);

	*new_ext_csd = NULL;

	if (!mmc_can_ext_csd(card))
		return 0;

	ext_csd = kmalloc(512, GFP_KERNEL);
	if (!ext_csd) {
		pr_err("%s: could not allocate a buffer to "
			"receive the ext_csd.\n", mmc_hostname(card->host));
		return -ENOMEM;
	}

	err = mmc_send_ext_csd(card, ext_csd);
	if (err) {
		kfree(ext_csd);
		*new_ext_csd = NULL;

		if ((err != -EINVAL)
		 && (err != -ENOSYS)
		 && (err != -EFAULT))
			return err;

		if (card->csd.capacity == (4096 * 512)) {
			pr_err("%s: unable to read EXT_CSD "
				"on a possible high capacity card. "
				"Card will be ignored.\n",
				mmc_hostname(card->host));
		} else {
			pr_warn("%s: unable to read EXT_CSD, performance might suffer\n",
				mmc_hostname(card->host));
			err = 0;
		}
	} else
		*new_ext_csd = ext_csd;

	return err;
}

static void mmc_select_card_type(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	u8 card_type = card->ext_csd.raw_card_type;
	u32 caps = host->caps, caps2 = host->caps2;
	unsigned int hs_max_dtr = 0, hs200_max_dtr = 0;
	unsigned int avail_type = 0;

	if (caps & MMC_CAP_MMC_HIGHSPEED &&
	    card_type & EXT_CSD_CARD_TYPE_HS_26) {
		hs_max_dtr = MMC_HIGH_26_MAX_DTR;
		avail_type |= EXT_CSD_CARD_TYPE_HS_26;
	}

	if (caps & MMC_CAP_MMC_HIGHSPEED &&
	    card_type & EXT_CSD_CARD_TYPE_HS_52) {
		hs_max_dtr = MMC_HIGH_52_MAX_DTR;
		avail_type |= EXT_CSD_CARD_TYPE_HS_52;
	}

	if (caps & MMC_CAP_1_8V_DDR &&
	    card_type & EXT_CSD_CARD_TYPE_DDR_1_8V) {
		hs_max_dtr = MMC_HIGH_DDR_MAX_DTR;
		avail_type |= EXT_CSD_CARD_TYPE_DDR_1_8V;
	}

	if (caps & MMC_CAP_1_2V_DDR &&
	    card_type & EXT_CSD_CARD_TYPE_DDR_1_2V) {
		hs_max_dtr = MMC_HIGH_DDR_MAX_DTR;
		avail_type |= EXT_CSD_CARD_TYPE_DDR_1_2V;
	}

	if (caps2 & MMC_CAP2_HS200_1_8V_SDR &&
	    card_type & EXT_CSD_CARD_TYPE_HS200_1_8V) {
		hs200_max_dtr = MMC_HS200_MAX_DTR;
		avail_type |= EXT_CSD_CARD_TYPE_HS200_1_8V;
	}

	if (caps2 & MMC_CAP2_HS200_1_2V_SDR &&
	    card_type & EXT_CSD_CARD_TYPE_HS200_1_2V) {
		hs200_max_dtr = MMC_HS200_MAX_DTR;
		avail_type |= EXT_CSD_CARD_TYPE_HS200_1_2V;
	}

	if (caps2 & MMC_CAP2_HS400_1_8V &&
	    card_type & EXT_CSD_CARD_TYPE_HS400_1_8V) {
		hs200_max_dtr = MMC_HS200_MAX_DTR;
		avail_type |= EXT_CSD_CARD_TYPE_HS400_1_8V;
	}

	if (caps2 & MMC_CAP2_HS400_1_2V &&
	    card_type & EXT_CSD_CARD_TYPE_HS400_1_2V) {
		hs200_max_dtr = MMC_HS200_MAX_DTR;
		avail_type |= EXT_CSD_CARD_TYPE_HS400_1_2V;
	}

	card->ext_csd.hs_max_dtr = hs_max_dtr;
	card->ext_csd.hs200_max_dtr = hs200_max_dtr;
	card->mmc_avail_type = avail_type;
}

static void mmc_manage_enhanced_area(struct mmc_card *card, u8 *ext_csd)
{
	u8 hc_erase_grp_sz, hc_wp_grp_sz;

	card->ext_csd.enhanced_area_offset = -EINVAL;
	card->ext_csd.enhanced_area_size = -EINVAL;

	if ((ext_csd[EXT_CSD_PARTITION_SUPPORT] & 0x2) &&
	    (ext_csd[EXT_CSD_PARTITION_ATTRIBUTE] & 0x1)) {
		if (card->ext_csd.partition_setting_completed) {
			hc_erase_grp_sz =
				ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE];
			hc_wp_grp_sz =
				ext_csd[EXT_CSD_HC_WP_GRP_SIZE];

			card->ext_csd.enhanced_area_offset =
				(ext_csd[139] << 24) + (ext_csd[138] << 16) +
				(ext_csd[137] << 8) + ext_csd[136];
			if (mmc_card_blockaddr(card))
				card->ext_csd.enhanced_area_offset <<= 9;
			card->ext_csd.enhanced_area_size =
				(ext_csd[142] << 16) + (ext_csd[141] << 8) +
				ext_csd[140];
			card->ext_csd.enhanced_area_size *=
				(size_t)(hc_erase_grp_sz * hc_wp_grp_sz);
			card->ext_csd.enhanced_area_size <<= 9;
		} else {
			pr_warn("%s: defines enhanced area without partition setting complete\n",
				mmc_hostname(card->host));
		}
	}
}

static void mmc_manage_gp_partitions(struct mmc_card *card, u8 *ext_csd)
{
	int idx;
	u8 hc_erase_grp_sz, hc_wp_grp_sz;
	unsigned int part_size;

	if (ext_csd[EXT_CSD_PARTITION_SUPPORT] &
	    EXT_CSD_PART_SUPPORT_PART_EN) {
		hc_erase_grp_sz =
			ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE];
		hc_wp_grp_sz =
			ext_csd[EXT_CSD_HC_WP_GRP_SIZE];

		for (idx = 0; idx < MMC_NUM_GP_PARTITION; idx++) {
			if (!ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3] &&
			    !ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3 + 1] &&
			    !ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3 + 2])
				continue;
			if (card->ext_csd.partition_setting_completed == 0) {
				pr_warn("%s: has partition size defined without partition complete\n",
					mmc_hostname(card->host));
				break;
			}
			part_size =
				(ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3 + 2]
				<< 16) +
				(ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3 + 1]
				<< 8) +
				ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3];
			part_size *= (size_t)(hc_erase_grp_sz *
				hc_wp_grp_sz);
			mmc_part_add(card, part_size << 19,
				EXT_CSD_PART_CONFIG_ACC_GP0 + idx,
				"gp%d", idx, false,
				MMC_BLK_DATA_AREA_GP);
		}
	}
}

static void htc_mmc_show_cid_ext_csd(struct mmc_card *card, u8 *ext_csd)
{
	char *buf;
	int i, j;
	ssize_t n = 0;
	char *check_fw;

	if (mmc_card_mmc(card)) {
		pr_info("%s: cid %08x%08x%08x%08x\n",
			mmc_hostname(card->host),
			card->raw_cid[0], card->raw_cid[1],
			card->raw_cid[2], card->raw_cid[3]);
		pr_info("%s: csd %08x%08x%08x%08x\n",
			mmc_hostname(card->host),
			card->raw_csd[0], card->raw_csd[1],
			card->raw_csd[2], card->raw_csd[3]);

		buf = kmalloc(512, GFP_KERNEL);
		if (buf) {
			for (i = 0; i < 32; i++) {
				for (j = 511 - (16 * i); j >= 496 - (16 * i); j--)
					n += sprintf(buf + n, "%02x", ext_csd[j]);
				n += sprintf(buf + n, "\n");
				pr_info("%s: ext_csd_%03d %s",
					mmc_hostname(card->host), 511 - (16 * i), buf);
				n = 0;
			}
		}
		if (buf)
			kfree(buf);

		
		if (card->cid.manfid == CID_MANFID_SANDISK ||
		    card->cid.manfid == CID_MANFID_SANDISK_2) {
			if (card->ext_csd.rev == 6)
				card->cid.fwrev =
				ext_csd[EXT_CSD_VENDOR_SPECIFIC_FIELDS_73] & 0x3F;
			else if (card->ext_csd.rev > 6) { 
				card->cid.fwrev =
				ext_csd[EXT_CSD_VENDOR_SPECIFIC_FIELDS_258] - 0x30 ;

				
				check_fw = kmalloc(16, GFP_KERNEL);
				if(check_fw) {
					sprintf(check_fw, "%d%d%d%d%d%d",
					ext_csd[254]-0x30, ext_csd[255]-0x30,
					ext_csd[256]-0x30, ext_csd[257]-0x30,
					ext_csd[258]-0x30, ext_csd[259]-0x30);
				}
				if(!strcmp(check_fw, "110224") ||
					!strcmp(check_fw, "101149")) {
					card->ext_csd.cmdq_support = 0;
					card->ext_csd.cmdq_depth = 0;
					card->ext_csd.barrier_support = 0;
					card->ext_csd.cache_flush_policy = 0;
				}
				pr_info("%s: SanDisk: %s\n", mmc_hostname(card->host),
					check_fw);
				if(check_fw)
					kfree(check_fw);
			}

		} else if (card->cid.manfid == CID_MANFID_MICRON) {
			card->cid.fwrev = ext_csd[EXT_CSD_VENDOR_SPECIFIC_FIELDS_258];
		}
	}
}

#define MAX_CMDQ_DEPTH	16
static int mmc_read_ext_csd(struct mmc_card *card, u8 *ext_csd)
{
	int err = 0, idx;
	unsigned int part_size;
	u8 q_depth;

	BUG_ON(!card);

	if (!ext_csd)
		return 0;

	
	card->ext_csd.raw_ext_csd_structure = ext_csd[EXT_CSD_STRUCTURE];
	if (card->csd.structure == 3) {
		if (card->ext_csd.raw_ext_csd_structure > 2) {
			pr_err("%s: unrecognised EXT_CSD structure "
				"version %d\n", mmc_hostname(card->host),
					card->ext_csd.raw_ext_csd_structure);
			err = -EINVAL;
			goto out;
		}
	}

	card->ext_csd.rev = ext_csd[EXT_CSD_REV];

	card->ext_csd.raw_sectors[0] = ext_csd[EXT_CSD_SEC_CNT + 0];
	card->ext_csd.raw_sectors[1] = ext_csd[EXT_CSD_SEC_CNT + 1];
	card->ext_csd.raw_sectors[2] = ext_csd[EXT_CSD_SEC_CNT + 2];
	card->ext_csd.raw_sectors[3] = ext_csd[EXT_CSD_SEC_CNT + 3];
	if (card->ext_csd.rev >= 2) {
		card->ext_csd.sectors =
			ext_csd[EXT_CSD_SEC_CNT + 0] << 0 |
			ext_csd[EXT_CSD_SEC_CNT + 1] << 8 |
			ext_csd[EXT_CSD_SEC_CNT + 2] << 16 |
			ext_csd[EXT_CSD_SEC_CNT + 3] << 24;

		
		if (card->ext_csd.sectors > (2u * 1024 * 1024 * 1024) / 512)
			mmc_card_set_blockaddr(card);
	}

	card->ext_csd.raw_card_type = ext_csd[EXT_CSD_CARD_TYPE];
	mmc_select_card_type(card);

	card->ext_csd.raw_drive_strength = ext_csd[EXT_CSD_DRIVE_STRENGTH];

	card->ext_csd.raw_s_a_timeout = ext_csd[EXT_CSD_S_A_TIMEOUT];
	card->ext_csd.raw_erase_timeout_mult =
		ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT];
	card->ext_csd.raw_hc_erase_grp_size =
		ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE];
	if (card->ext_csd.rev >= 3) {
		u8 sa_shift = ext_csd[EXT_CSD_S_A_TIMEOUT];
		card->ext_csd.part_config = ext_csd[EXT_CSD_PART_CONFIG];

		
		card->ext_csd.part_time = 10 * ext_csd[EXT_CSD_PART_SWITCH_TIME];

		
		if (sa_shift > 0 && sa_shift <= 0x17)
			card->ext_csd.sa_timeout =
					1 << ext_csd[EXT_CSD_S_A_TIMEOUT];
		card->ext_csd.erase_group_def =
			ext_csd[EXT_CSD_ERASE_GROUP_DEF];
		card->ext_csd.hc_erase_timeout = 300 *
			ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT];
		card->ext_csd.hc_erase_size =
			ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE] << 10;

		card->ext_csd.rel_sectors = ext_csd[EXT_CSD_REL_WR_SEC_C];

		if (ext_csd[EXT_CSD_BOOT_MULT] && mmc_boot_partition_access(card->host)) {
			for (idx = 0; idx < MMC_NUM_BOOT_PARTITION; idx++) {
				part_size = ext_csd[EXT_CSD_BOOT_MULT] << 17;
				mmc_part_add(card, part_size,
					EXT_CSD_PART_CONFIG_ACC_BOOT0 + idx,
					"boot%d", idx, true,
					MMC_BLK_DATA_AREA_BOOT);
			}
		}
	}

	card->ext_csd.raw_hc_erase_gap_size =
		ext_csd[EXT_CSD_HC_WP_GRP_SIZE];
	card->ext_csd.raw_sec_trim_mult =
		ext_csd[EXT_CSD_SEC_TRIM_MULT];
	card->ext_csd.raw_sec_erase_mult =
		ext_csd[EXT_CSD_SEC_ERASE_MULT];
	card->ext_csd.raw_sec_feature_support =
		ext_csd[EXT_CSD_SEC_FEATURE_SUPPORT];
	card->ext_csd.raw_trim_mult =
		ext_csd[EXT_CSD_TRIM_MULT];
	card->ext_csd.raw_partition_support = ext_csd[EXT_CSD_PARTITION_SUPPORT];
	if (card->ext_csd.rev >= 4) {
		if (ext_csd[EXT_CSD_PARTITION_SETTING_COMPLETED] &
		    EXT_CSD_PART_SETTING_COMPLETED)
			card->ext_csd.partition_setting_completed = 1;
		else
			card->ext_csd.partition_setting_completed = 0;

		mmc_manage_enhanced_area(card, ext_csd);

		mmc_manage_gp_partitions(card, ext_csd);

		card->ext_csd.sec_trim_mult =
			ext_csd[EXT_CSD_SEC_TRIM_MULT];
		card->ext_csd.sec_erase_mult =
			ext_csd[EXT_CSD_SEC_ERASE_MULT];
		card->ext_csd.sec_feature_support =
			ext_csd[EXT_CSD_SEC_FEATURE_SUPPORT];
		card->ext_csd.trim_timeout = 300 *
			ext_csd[EXT_CSD_TRIM_MULT];

		card->ext_csd.boot_ro_lock = ext_csd[EXT_CSD_BOOT_WP];
		card->ext_csd.boot_ro_lockable = true;

		
		card->ext_csd.raw_pwr_cl_52_195 =
			ext_csd[EXT_CSD_PWR_CL_52_195];
		card->ext_csd.raw_pwr_cl_26_195 =
			ext_csd[EXT_CSD_PWR_CL_26_195];
		card->ext_csd.raw_pwr_cl_52_360 =
			ext_csd[EXT_CSD_PWR_CL_52_360];
		card->ext_csd.raw_pwr_cl_26_360 =
			ext_csd[EXT_CSD_PWR_CL_26_360];
		card->ext_csd.raw_pwr_cl_200_195 =
			ext_csd[EXT_CSD_PWR_CL_200_195];
		card->ext_csd.raw_pwr_cl_200_360 =
			ext_csd[EXT_CSD_PWR_CL_200_360];
		card->ext_csd.raw_pwr_cl_ddr_52_195 =
			ext_csd[EXT_CSD_PWR_CL_DDR_52_195];
		card->ext_csd.raw_pwr_cl_ddr_52_360 =
			ext_csd[EXT_CSD_PWR_CL_DDR_52_360];
		card->ext_csd.raw_pwr_cl_ddr_200_360 =
			ext_csd[EXT_CSD_PWR_CL_DDR_200_360];
	}

	
	if ((ext_csd[EXT_CSD_HPI_FEATURES] & 0x1) &&
		!(card->quirks & MMC_QUIRK_BROKEN_HPI)) {
		card->ext_csd.hpi = 1;
		if (ext_csd[EXT_CSD_HPI_FEATURES] & 0x2)
			card->ext_csd.hpi_cmd = MMC_STOP_TRANSMISSION;
		else
			card->ext_csd.hpi_cmd = MMC_SEND_STATUS;
		card->ext_csd.out_of_int_time =
			ext_csd[EXT_CSD_OUT_OF_INTERRUPT_TIME] * 10;
		pr_info("%s: Out-of-interrupt timeout is %d[ms]\n",
				mmc_hostname(card->host),
				card->ext_csd.out_of_int_time);
	}

	if (card->ext_csd.rev >= 5) {
		
		if (card->cid.year < 2010)
			card->cid.year += 16;

		
		if ((ext_csd[EXT_CSD_BKOPS_SUPPORT] & 0x1) &&
				card->ext_csd.hpi) {
			card->ext_csd.bkops = 1;
			card->ext_csd.bkops_en = ext_csd[EXT_CSD_BKOPS_EN];
			card->ext_csd.raw_bkops_status =
				ext_csd[EXT_CSD_BKOPS_STATUS];
			pr_info("%s: BKOPS_EN equals 0x%x\n",
					mmc_hostname(card->host),
					card->ext_csd.bkops_en);

		}

		card->ext_csd.rel_param = ext_csd[EXT_CSD_WR_REL_PARAM];
		card->ext_csd.rst_n_function = ext_csd[EXT_CSD_RST_N_FUNCTION];

		card->ext_csd.raw_rpmb_size_mult = ext_csd[EXT_CSD_RPMB_MULT];
		if (ext_csd[EXT_CSD_RPMB_MULT] && mmc_host_cmd23(card->host)) {
			mmc_part_add(card, ext_csd[EXT_CSD_RPMB_MULT] << 17,
				EXT_CSD_PART_CONFIG_ACC_RPMB,
				"rpmb", 0, false,
				MMC_BLK_DATA_AREA_RPMB);
		}
	}

	card->ext_csd.raw_erased_mem_count = ext_csd[EXT_CSD_ERASED_MEM_CONT];
	if (ext_csd[EXT_CSD_ERASED_MEM_CONT])
		card->erased_byte = 0xFF;
	else
		card->erased_byte = 0x0;

	
	if (card->ext_csd.rev >= 6) {
		card->ext_csd.feature_support |= MMC_DISCARD_FEATURE;

		card->ext_csd.generic_cmd6_time = 10 *
			ext_csd[EXT_CSD_GENERIC_CMD6_TIME];
		card->ext_csd.power_off_longtime = 10 *
			ext_csd[EXT_CSD_POWER_OFF_LONG_TIME];

		card->ext_csd.cache_size =
			ext_csd[EXT_CSD_CACHE_SIZE + 0] << 0 |
			ext_csd[EXT_CSD_CACHE_SIZE + 1] << 8 |
			ext_csd[EXT_CSD_CACHE_SIZE + 2] << 16 |
			ext_csd[EXT_CSD_CACHE_SIZE + 3] << 24;

		if (ext_csd[EXT_CSD_DATA_SECTOR_SIZE] == 1)
			card->ext_csd.data_sector_size = 4096;
		else
			card->ext_csd.data_sector_size = 512;

		if ((ext_csd[EXT_CSD_DATA_TAG_SUPPORT] & 1) &&
		    (ext_csd[EXT_CSD_TAG_UNIT_SIZE] <= 8)) {
			card->ext_csd.data_tag_unit_size =
			((unsigned int) 1 << ext_csd[EXT_CSD_TAG_UNIT_SIZE]) *
			(card->ext_csd.data_sector_size);
		} else {
			card->ext_csd.data_tag_unit_size = 0;
		}

		card->ext_csd.max_packed_writes =
			ext_csd[EXT_CSD_MAX_PACKED_WRITES];
		card->ext_csd.max_packed_reads =
			ext_csd[EXT_CSD_MAX_PACKED_READS];
	} else {
		card->ext_csd.data_sector_size = 512;
	}

	if (card->ext_csd.rev >= 7) {
		card->ext_csd.strobe_support = ext_csd[EXT_CSD_STROBE_SUPPORT];
		card->ext_csd.cmdq_support = ext_csd[EXT_CSD_CMDQ_SUPPORT];
		card->ext_csd.fw_version = ext_csd[EXT_CSD_FW_VERSION];
		pr_info("%s: eMMC FW version: 0x%02x\n",
			mmc_hostname(card->host),
			card->ext_csd.fw_version);
		if (card->ext_csd.cmdq_support) {
			q_depth = ext_csd[EXT_CSD_CMDQ_DEPTH] + 1;
			if (q_depth > MAX_CMDQ_DEPTH) {
				pr_info("%s: CMDQ limit depth: %u -> %u\n",
					mmc_hostname(card->host), q_depth, MAX_CMDQ_DEPTH);
				card->ext_csd.cmdq_depth = MAX_CMDQ_DEPTH;
			} else {
				card->ext_csd.cmdq_depth = q_depth;
			}
			pr_info("%s: CMDQ supported: depth: %d\n",
				mmc_hostname(card->host),
				card->ext_csd.cmdq_depth);
		}
		card->ext_csd.barrier_support =
			ext_csd[EXT_CSD_BARRIER_SUPPORT];
		card->ext_csd.cache_flush_policy =
			ext_csd[EXT_CSD_CACHE_FLUSH_POLICY];
		pr_info("%s: cache barrier support %d flush policy %d\n",
				mmc_hostname(card->host),
				card->ext_csd.barrier_support,
				card->ext_csd.cache_flush_policy);
		card->ext_csd.enhanced_rpmb_supported =
			(card->ext_csd.rel_param &
			 EXT_CSD_WR_REL_PARAM_EN_RPMB_REL_WR);
	} else {
		card->ext_csd.cmdq_support = 0;
		card->ext_csd.cmdq_depth = 0;
		card->ext_csd.barrier_support = 0;
		card->ext_csd.cache_flush_policy = 0;
	}

	card->ext_csd.ffu_mode_op = ext_csd[EXT_CSD_FFU_FEATURES];
	printk("%s: ext_csd[EXT_CSD_FFU_FEATURES]=%x \n", __func__, ext_csd[EXT_CSD_FFU_FEATURES]);

	htc_mmc_show_cid_ext_csd(card, ext_csd);

out:
	return err;
}

static inline void mmc_free_ext_csd(u8 *ext_csd)
{
	kfree(ext_csd);
}


static int mmc_compare_ext_csds(struct mmc_card *card, unsigned bus_width)
{
	u8 *bw_ext_csd;
	int err;

	if (bus_width == MMC_BUS_WIDTH_1)
		return 0;

	err = mmc_get_ext_csd(card, &bw_ext_csd);

	if (err || bw_ext_csd == NULL) {
		err = -EINVAL;
		goto out;
	}

	
	err = !((card->ext_csd.raw_partition_support ==
			bw_ext_csd[EXT_CSD_PARTITION_SUPPORT]) &&
		(card->ext_csd.raw_erased_mem_count ==
			bw_ext_csd[EXT_CSD_ERASED_MEM_CONT]) &&
		(card->ext_csd.rev ==
			bw_ext_csd[EXT_CSD_REV]) &&
		(card->ext_csd.raw_ext_csd_structure ==
			bw_ext_csd[EXT_CSD_STRUCTURE]) &&
		(card->ext_csd.raw_card_type ==
			bw_ext_csd[EXT_CSD_CARD_TYPE]) &&
		(card->ext_csd.raw_s_a_timeout ==
			bw_ext_csd[EXT_CSD_S_A_TIMEOUT]) &&
		(card->ext_csd.raw_hc_erase_gap_size ==
			bw_ext_csd[EXT_CSD_HC_WP_GRP_SIZE]) &&
		(card->ext_csd.raw_erase_timeout_mult ==
			bw_ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT]) &&
		(card->ext_csd.raw_hc_erase_grp_size ==
			bw_ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE]) &&
		(card->ext_csd.raw_sec_trim_mult ==
			bw_ext_csd[EXT_CSD_SEC_TRIM_MULT]) &&
		(card->ext_csd.raw_sec_erase_mult ==
			bw_ext_csd[EXT_CSD_SEC_ERASE_MULT]) &&
		(card->ext_csd.raw_sec_feature_support ==
			bw_ext_csd[EXT_CSD_SEC_FEATURE_SUPPORT]) &&
		(card->ext_csd.raw_trim_mult ==
			bw_ext_csd[EXT_CSD_TRIM_MULT]) &&
		(card->ext_csd.raw_sectors[0] ==
			bw_ext_csd[EXT_CSD_SEC_CNT + 0]) &&
		(card->ext_csd.raw_sectors[1] ==
			bw_ext_csd[EXT_CSD_SEC_CNT + 1]) &&
		(card->ext_csd.raw_sectors[2] ==
			bw_ext_csd[EXT_CSD_SEC_CNT + 2]) &&
		(card->ext_csd.raw_sectors[3] ==
			bw_ext_csd[EXT_CSD_SEC_CNT + 3]) &&
		(card->ext_csd.raw_pwr_cl_52_195 ==
			bw_ext_csd[EXT_CSD_PWR_CL_52_195]) &&
		(card->ext_csd.raw_pwr_cl_26_195 ==
			bw_ext_csd[EXT_CSD_PWR_CL_26_195]) &&
		(card->ext_csd.raw_pwr_cl_52_360 ==
			bw_ext_csd[EXT_CSD_PWR_CL_52_360]) &&
		(card->ext_csd.raw_pwr_cl_26_360 ==
			bw_ext_csd[EXT_CSD_PWR_CL_26_360]) &&
		(card->ext_csd.raw_pwr_cl_200_195 ==
			bw_ext_csd[EXT_CSD_PWR_CL_200_195]) &&
		(card->ext_csd.raw_pwr_cl_200_360 ==
			bw_ext_csd[EXT_CSD_PWR_CL_200_360]) &&
		(card->ext_csd.raw_pwr_cl_ddr_52_195 ==
			bw_ext_csd[EXT_CSD_PWR_CL_DDR_52_195]) &&
		(card->ext_csd.raw_pwr_cl_ddr_52_360 ==
			bw_ext_csd[EXT_CSD_PWR_CL_DDR_52_360]) &&
		(card->ext_csd.raw_pwr_cl_ddr_200_360 ==
			bw_ext_csd[EXT_CSD_PWR_CL_DDR_200_360]));

	if (err)
		err = -EINVAL;

out:
	mmc_free_ext_csd(bw_ext_csd);
	return err;
}

MMC_DEV_ATTR(cid, "%08x%08x%08x%08x\n", card->raw_cid[0], card->raw_cid[1],
	card->raw_cid[2], card->raw_cid[3]);
MMC_DEV_ATTR(csd, "%08x%08x%08x%08x\n", card->raw_csd[0], card->raw_csd[1],
	card->raw_csd[2], card->raw_csd[3]);
MMC_DEV_ATTR(date, "%02d/%04d\n", card->cid.month, card->cid.year);
MMC_DEV_ATTR(erase_size, "%u\n", card->erase_size << 9);
MMC_DEV_ATTR(preferred_erase_size, "%u\n", card->pref_erase << 9);
MMC_DEV_ATTR(fwrev, "0x%x\n", card->cid.fwrev);
MMC_DEV_ATTR(hwrev, "0x%x\n", card->cid.hwrev);
MMC_DEV_ATTR(manfid, "0x%06x\n", card->cid.manfid);
MMC_DEV_ATTR(name, "%s\n", card->cid.prod_name);
MMC_DEV_ATTR(oemid, "0x%04x\n", card->cid.oemid);
MMC_DEV_ATTR(prv, "0x%x\n", card->cid.prv);
MMC_DEV_ATTR(serial, "0x%08x\n", card->cid.serial);
MMC_DEV_ATTR(enhanced_area_offset, "%llu\n",
		card->ext_csd.enhanced_area_offset);
MMC_DEV_ATTR(enhanced_area_size, "%u\n", card->ext_csd.enhanced_area_size);
MMC_DEV_ATTR(raw_rpmb_size_mult, "%#x\n", card->ext_csd.raw_rpmb_size_mult);
MMC_DEV_ATTR(enhanced_rpmb_supported, "%#x\n",
		card->ext_csd.enhanced_rpmb_supported);
MMC_DEV_ATTR(rel_sectors, "%#x\n", card->ext_csd.rel_sectors);

static ssize_t htc_mmc_show_manf_name (struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mmc_card *card = mmc_dev_to_card(dev);
	int count = 0;

	switch (card->cid.manfid) {
	case CID_MANFID_SANDISK:
	case CID_MANFID_SANDISK_2:
		count = sprintf(buf, "Sandisk\n");
		break;
	case CID_MANFID_TOSHIBA:
		count = sprintf(buf, "Toshiba\n");
		break;
	case CID_MANFID_MICRON:
	case CID_MANFID_NUMONYX_MICRON:
		count = sprintf(buf, "Micron\n");
		break;
	case CID_MANFID_SAMSUNG:
		count = sprintf(buf, "Samsung\n");
		break;
	case CID_MANFID_HYNIX:
		count = sprintf(buf, "Hynix\n");
		break;
	case CID_MANFID_KINGSTON:
		count = sprintf(buf, "Kingston\n");
		break;
	default:
		count = sprintf(buf, "Unknown\n");
	}

	return count;
}
DEVICE_ATTR(manf_name, S_IRUGO, htc_mmc_show_manf_name, NULL);

static struct attribute *mmc_std_attrs[] = {
	&dev_attr_cid.attr,
	&dev_attr_csd.attr,
	&dev_attr_date.attr,
	&dev_attr_erase_size.attr,
	&dev_attr_preferred_erase_size.attr,
	&dev_attr_fwrev.attr,
	&dev_attr_hwrev.attr,
	&dev_attr_manfid.attr,
	&dev_attr_name.attr,
	&dev_attr_oemid.attr,
	&dev_attr_prv.attr,
	&dev_attr_serial.attr,
	&dev_attr_enhanced_area_offset.attr,
	&dev_attr_enhanced_area_size.attr,
	&dev_attr_raw_rpmb_size_mult.attr,
	&dev_attr_enhanced_rpmb_supported.attr,
	&dev_attr_rel_sectors.attr,
	&dev_attr_manf_name.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mmc_std);

static struct device_type mmc_type = {
	.groups = mmc_std_groups,
};

static int __mmc_select_powerclass(struct mmc_card *card,
				   unsigned int bus_width)
{
	struct mmc_host *host = card->host;
	struct mmc_ext_csd *ext_csd = &card->ext_csd;
	unsigned int pwrclass_val = 0;
	int err = 0;

	
	if (card->csd.mmca_vsn < CSD_SPEC_VER_4)
		return 0;

	
	if (bus_width == EXT_CSD_BUS_WIDTH_1)
		return 0;

	switch (1 << host->ios.vdd) {
	case MMC_VDD_165_195:
		if (host->ios.clock <= MMC_HIGH_26_MAX_DTR)
			pwrclass_val = ext_csd->raw_pwr_cl_26_195;
		else if (host->ios.clock <= MMC_HIGH_52_MAX_DTR)
			pwrclass_val = (bus_width <= EXT_CSD_BUS_WIDTH_8) ?
				ext_csd->raw_pwr_cl_52_195 :
				ext_csd->raw_pwr_cl_ddr_52_195;
		else if (host->ios.clock <= MMC_HS200_MAX_DTR)
			pwrclass_val = ext_csd->raw_pwr_cl_200_195;
		break;
	case MMC_VDD_27_28:
	case MMC_VDD_28_29:
	case MMC_VDD_29_30:
	case MMC_VDD_30_31:
	case MMC_VDD_31_32:
	case MMC_VDD_32_33:
	case MMC_VDD_33_34:
	case MMC_VDD_34_35:
	case MMC_VDD_35_36:
		if (host->ios.clock <= MMC_HIGH_26_MAX_DTR)
			pwrclass_val = ext_csd->raw_pwr_cl_26_360;
		else if (host->ios.clock <= MMC_HIGH_52_MAX_DTR)
			pwrclass_val = (bus_width <= EXT_CSD_BUS_WIDTH_8) ?
				ext_csd->raw_pwr_cl_52_360 :
				ext_csd->raw_pwr_cl_ddr_52_360;
		else if (host->ios.clock <= MMC_HS200_MAX_DTR)
			pwrclass_val = (bus_width == EXT_CSD_DDR_BUS_WIDTH_8) ?
				ext_csd->raw_pwr_cl_ddr_200_360 :
				ext_csd->raw_pwr_cl_200_360;
		break;
	default:
		pr_warn("%s: Voltage range not supported for power class\n",
			mmc_hostname(host));
		return -EINVAL;
	}

	if (bus_width & (EXT_CSD_BUS_WIDTH_8 | EXT_CSD_DDR_BUS_WIDTH_8))
		pwrclass_val = (pwrclass_val & EXT_CSD_PWR_CL_8BIT_MASK) >>
				EXT_CSD_PWR_CL_8BIT_SHIFT;
	else
		pwrclass_val = (pwrclass_val & EXT_CSD_PWR_CL_4BIT_MASK) >>
				EXT_CSD_PWR_CL_4BIT_SHIFT;

	
	if (pwrclass_val > 0) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_POWER_CLASS,
				 pwrclass_val,
				 card->ext_csd.generic_cmd6_time);
	}

	return err;
}

static int mmc_select_powerclass(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	u32 bus_width, ext_csd_bits;
	int err, ddr;

	
	if (!mmc_can_ext_csd(card))
		return 0;

	bus_width = host->ios.bus_width;
	
	if (bus_width == MMC_BUS_WIDTH_1)
		return 0;

	ddr = card->mmc_avail_type & EXT_CSD_CARD_TYPE_DDR_52;
	if (ddr)
		ext_csd_bits = (bus_width == MMC_BUS_WIDTH_8) ?
			EXT_CSD_DDR_BUS_WIDTH_8 : EXT_CSD_DDR_BUS_WIDTH_4;
	else
		ext_csd_bits = (bus_width == MMC_BUS_WIDTH_8) ?
			EXT_CSD_BUS_WIDTH_8 :  EXT_CSD_BUS_WIDTH_4;

	err = __mmc_select_powerclass(card, ext_csd_bits);
	if (err)
		pr_warn("%s: power class selection to bus width %d ddr %d failed\n",
			mmc_hostname(host), 1 << bus_width, ddr);

	return err;
}

static void mmc_set_bus_speed(struct mmc_card *card)
{
	unsigned int max_dtr = (unsigned int)-1;

	if ((mmc_card_hs200(card) || mmc_card_hs400(card)) &&
	     max_dtr > card->ext_csd.hs200_max_dtr)
		max_dtr = card->ext_csd.hs200_max_dtr;
	else if (mmc_card_hs(card) && max_dtr > card->ext_csd.hs_max_dtr)
		max_dtr = card->ext_csd.hs_max_dtr;
	else if (max_dtr > card->csd.max_dtr)
		max_dtr = card->csd.max_dtr;

	mmc_set_clock(card->host, max_dtr);
}

static int mmc_select_bus_width(struct mmc_card *card)
{
	static const unsigned ext_csd_bits[] = {
		EXT_CSD_BUS_WIDTH_8,
		EXT_CSD_BUS_WIDTH_4,
	};
	static const unsigned bus_widths[] = {
		MMC_BUS_WIDTH_8,
		MMC_BUS_WIDTH_4,
	};
	struct mmc_host *host = card->host;
	unsigned idx, bus_width = 0;
	int err = 0;

	if (!mmc_can_ext_csd(card) &&
	    !(host->caps & (MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA)))
		return 0;

	idx = (host->caps & MMC_CAP_8_BIT_DATA) ? 0 : 1;

	for (; idx < ARRAY_SIZE(bus_widths); idx++) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_BUS_WIDTH,
				 ext_csd_bits[idx],
				 card->ext_csd.generic_cmd6_time);
		if (err)
			continue;

		bus_width = bus_widths[idx];
		mmc_set_bus_width(host, bus_width);

		if (!(host->caps & MMC_CAP_BUS_WIDTH_TEST))
			err = mmc_compare_ext_csds(card, bus_width);
		else
			err = mmc_bus_test(card, bus_width);

		if (!err) {
			err = bus_width;
			break;
		} else {
			pr_warn("%s: switch to bus width %d failed\n",
				mmc_hostname(host), ext_csd_bits[idx]);
		}
	}

	return err;
}

static int mmc_select_hs(struct mmc_card *card)
{
	int err;

	err = __mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			   EXT_CSD_HS_TIMING, EXT_CSD_TIMING_HS,
			   card->ext_csd.generic_cmd6_time,
			   true, true, true);
	if (!err)
		mmc_set_timing(card->host, MMC_TIMING_MMC_HS);

	return err;
}

static int mmc_select_hs_ddr(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	u32 bus_width, ext_csd_bits;
	int err = 0;

	if (!(card->mmc_avail_type & EXT_CSD_CARD_TYPE_DDR_52))
		return 0;

	bus_width = host->ios.bus_width;
	if (bus_width == MMC_BUS_WIDTH_1)
		return 0;

	ext_csd_bits = (bus_width == MMC_BUS_WIDTH_8) ?
		EXT_CSD_DDR_BUS_WIDTH_8 : EXT_CSD_DDR_BUS_WIDTH_4;

	err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_BUS_WIDTH,
			ext_csd_bits,
			card->ext_csd.generic_cmd6_time);
	if (err) {
		pr_warn("%s: switch to bus width %d ddr failed\n",
			mmc_hostname(host), 1 << bus_width);
		return err;
	}

	err = -EINVAL;
	if (card->mmc_avail_type & EXT_CSD_CARD_TYPE_DDR_1_2V)
		err = __mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_120);

	if (err && (card->mmc_avail_type & EXT_CSD_CARD_TYPE_DDR_1_8V))
		err = __mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_180);

	
	if (err)
		err = __mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_330);

	if (!err)
		mmc_set_timing(host, MMC_TIMING_MMC_DDR52);

	return err;
}

static int mmc_select_hs400(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	int err = 0;
	u8 val;

	if (card->ext_csd.strobe_support) {
		if (!(card->mmc_avail_type & EXT_CSD_CARD_TYPE_HS400 &&
		    host->caps & MMC_CAP_8_BIT_DATA))
			return 0;

		if (card->mmc_avail_type & EXT_CSD_CARD_TYPE_HS200_1_2V)
			err = __mmc_set_signal_voltage(host,
					MMC_SIGNAL_VOLTAGE_120);

		if (err && card->mmc_avail_type & EXT_CSD_CARD_TYPE_HS200_1_8V)
			err = __mmc_set_signal_voltage(host,
					MMC_SIGNAL_VOLTAGE_180);
		if (err)
			return err;
	} else {
		if (!(card->mmc_avail_type & EXT_CSD_CARD_TYPE_HS400 &&
		    host->ios.bus_width == MMC_BUS_WIDTH_8))
			return 0;
	}

	mmc_set_timing(card->host, MMC_TIMING_MMC_HS);
	mmc_set_bus_speed(card);

	err = __mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			   EXT_CSD_HS_TIMING, EXT_CSD_TIMING_HS,
			   card->ext_csd.generic_cmd6_time,
			   true, true, true);
	if (err) {
		pr_warn("%s: switch to high-speed from hs200 failed, err:%d\n",
			mmc_hostname(host), err);
		return err;
	}

	val = EXT_CSD_DDR_BUS_WIDTH_8;
	if (card->ext_csd.strobe_support) {
		err = mmc_select_bus_width(card);
		if (IS_ERR_VALUE(err))
			return err;
		val |= EXT_CSD_BUS_WIDTH_STROBE;
	}
	err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			 EXT_CSD_BUS_WIDTH,
			 val,
			 card->ext_csd.generic_cmd6_time);
	if (err) {
		pr_warn("%s: switch to bus width for hs400 failed, err:%d\n",
			mmc_hostname(host), err);
		return err;
	}

	err = __mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			   EXT_CSD_HS_TIMING, EXT_CSD_TIMING_HS400,
			   card->ext_csd.generic_cmd6_time,
			   true, true, true);
	if (err) {
		pr_warn("%s: switch to hs400 failed, err:%d\n",
			 mmc_hostname(host), err);
		return err;
	}

	mmc_set_timing(host, MMC_TIMING_MMC_HS400);
	mmc_set_bus_speed(card);

	if (card->ext_csd.strobe_support && host->ops->enhanced_strobe) {
		mmc_host_clk_hold(host);
		err = host->ops->enhanced_strobe(host);
		mmc_host_clk_release(host);
	} else if ((host->caps2 & MMC_CAP2_HS400_POST_TUNING) &&
			host->ops->execute_tuning) {
		mmc_host_clk_hold(host);
		err = host->ops->execute_tuning(host,
				MMC_SEND_TUNING_BLOCK_HS200);
		mmc_host_clk_release(host);

		if (err)
			pr_warn("%s: tuning execution failed\n",
				mmc_hostname(host));
	}

	return err;
}

static int mmc_select_hs200(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	int err = -EINVAL;

	if (card->mmc_avail_type & EXT_CSD_CARD_TYPE_HS200_1_2V)
		err = __mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_120);

	if (err && card->mmc_avail_type & EXT_CSD_CARD_TYPE_HS200_1_8V)
		err = __mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_180);

	
	if (err)
		goto err;

	err = mmc_select_bus_width(card);
	if (!IS_ERR_VALUE(err)) {
		err = __mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				   EXT_CSD_HS_TIMING, EXT_CSD_TIMING_HS200,
				   card->ext_csd.generic_cmd6_time,
				   true, true, true);
		if (!err)
			mmc_set_timing(host, MMC_TIMING_MMC_HS200);
	}
err:
	return err;
}

static int mmc_reboot_notify(struct notifier_block *notify_block,
		unsigned long event, void *unused)
{
	struct mmc_card *card = container_of(
			notify_block, struct mmc_card, reboot_notify);

	card->pon_type = (event != SYS_RESTART) ? MMC_LONG_PON : MMC_SHRT_PON;

	return NOTIFY_OK;
}

static int mmc_select_timing(struct mmc_card *card)
{
	int err = 0;

	if (!mmc_can_ext_csd(card))
		goto bus_speed;

	
	if (card->ext_csd.strobe_support &&
	    card->mmc_avail_type & EXT_CSD_CARD_TYPE_HS400 &&
	    card->host->caps & MMC_CAP_8_BIT_DATA)
		err = mmc_select_hs400(card);
	else if (card->mmc_avail_type & EXT_CSD_CARD_TYPE_HS200)
		err = mmc_select_hs200(card);
	else if (card->mmc_avail_type & EXT_CSD_CARD_TYPE_HS)
		err = mmc_select_hs(card);

	if (err && err != -EBADMSG)
		return err;

	if (err) {
		pr_warn("%s: switch to %s failed\n",
			mmc_card_hs(card) ? "high-speed" :
			(mmc_card_hs200(card) ? "hs200" : ""),
			mmc_hostname(card->host));
		err = 0;
	}

bus_speed:
	mmc_set_bus_speed(card);
	return err;
}

const u8 tuning_blk_pattern_4bit[MMC_TUNING_BLK_PATTERN_4BIT_SIZE] = {
	0xff, 0x0f, 0xff, 0x00, 0xff, 0xcc, 0xc3, 0xcc,
	0xc3, 0x3c, 0xcc, 0xff, 0xfe, 0xff, 0xfe, 0xef,
	0xff, 0xdf, 0xff, 0xdd, 0xff, 0xfb, 0xff, 0xfb,
	0xbf, 0xff, 0x7f, 0xff, 0x77, 0xf7, 0xbd, 0xef,
	0xff, 0xf0, 0xff, 0xf0, 0x0f, 0xfc, 0xcc, 0x3c,
	0xcc, 0x33, 0xcc, 0xcf, 0xff, 0xef, 0xff, 0xee,
	0xff, 0xfd, 0xff, 0xfd, 0xdf, 0xff, 0xbf, 0xff,
	0xbb, 0xff, 0xf7, 0xff, 0xf7, 0x7f, 0x7b, 0xde,
};
EXPORT_SYMBOL(tuning_blk_pattern_4bit);

const u8 tuning_blk_pattern_8bit[MMC_TUNING_BLK_PATTERN_8BIT_SIZE] = {
	0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00,
	0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc, 0xcc,
	0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff, 0xff,
	0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee, 0xff,
	0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd, 0xdd,
	0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff, 0xbb,
	0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff, 0xff,
	0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee, 0xff,
	0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00,
	0x00, 0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc,
	0xcc, 0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff,
	0xff, 0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee,
	0xff, 0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd,
	0xdd, 0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff,
	0xbb, 0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff,
	0xff, 0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee,
};
EXPORT_SYMBOL(tuning_blk_pattern_8bit);

static int mmc_hs200_tuning(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	int err = 0;

	if (card->mmc_avail_type & EXT_CSD_CARD_TYPE_HS400 &&
	    host->ios.bus_width == MMC_BUS_WIDTH_8)
		mmc_set_timing(host, MMC_TIMING_MMC_HS400);

	if (host->ops->execute_tuning) {
		mmc_host_clk_hold(host);
		err = host->ops->execute_tuning(host,
				MMC_SEND_TUNING_BLOCK_HS200);
		mmc_host_clk_release(host);

		if (err)
			pr_warn("%s: tuning execution failed\n",
				mmc_hostname(host));
	}

	return err;
}

static int mmc_select_cmdq(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	int ret = 0;

	if (!host->cmdq_ops) {
		pr_err("%s: host controller doesn't support CMDQ\n",
		       mmc_hostname(host));
		return 0;
	}

	ret = mmc_set_blocklen(card, MMC_CARD_CMDQ_BLK_SIZE);
	if (ret)
		goto out;

	ret = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_CMDQ, 1,
			 card->ext_csd.generic_cmd6_time);
	if (ret)
		goto out;

	mmc_card_set_cmdq(card);
	mmc_host_clk_hold(card->host);
	ret = host->cmdq_ops->enable(card->host);
	if (ret) {
		mmc_host_clk_release(card->host);
		pr_err("%s: failed (%d) enabling CMDQ on host\n",
			mmc_hostname(host), ret);
		mmc_card_clr_cmdq(card);
		ret = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_CMDQ, 0,
				 card->ext_csd.generic_cmd6_time);
		goto out;
	}

	mmc_host_clk_release(card->host);
	pr_info_once("%s: CMDQ enabled on card\n", mmc_hostname(host));
out:
	return ret;
}

static int mmc_select_hs_ddr52(struct mmc_host *host)
{
	int err;

	mmc_select_hs(host->card);
	mmc_set_clock(host, MMC_HIGH_52_MAX_DTR);
	err = mmc_select_bus_width(host->card);
	if (err < 0) {
		pr_err("%s: %s: select_bus_width failed(%d)\n",
			mmc_hostname(host), __func__, err);
		return err;
	}

	err = mmc_select_hs_ddr(host->card);

	return err;
}

static int mmc_scale_low(struct mmc_host *host, unsigned long freq)
{
	int err = 0;

	mmc_set_timing(host, MMC_TIMING_LEGACY);
	mmc_set_clock(host, MMC_HIGH_26_MAX_DTR);

	if (host->clk_scaling.lower_bus_speed_mode &
	    MMC_SCALING_LOWER_DDR52_MODE) {
		err = mmc_select_hs_ddr52(host);
		if (err)
			pr_err("%s: %s: failed to switch to DDR52: err: %d\n",
			       mmc_hostname(host), __func__, err);
		else
			return err;
	}

	err = mmc_select_hs(host->card);
	if (err) {
		pr_err("%s: %s: scaling low: failed (%d)\n",
		       mmc_hostname(host), __func__, err);
		return err;
	}

	err = mmc_select_bus_width(host->card);
	if (err < 0) {
		pr_err("%s: %s: select_bus_width failed(%d)\n",
			mmc_hostname(host), __func__, err);
		return err;
	}

	mmc_set_clock(host, freq);

	return 0;
}

static int mmc_scale_high(struct mmc_host *host)
{
	int err = 0;

	if (!host->card->ext_csd.strobe_support) {
		if (!(host->card->mmc_avail_type & EXT_CSD_CARD_TYPE_HS200)) {
			pr_err("%s: %s: card does not support HS200\n",
				mmc_hostname(host), __func__);
			WARN_ON(1);
			return -EPERM;
		}

		err = mmc_select_hs200(host->card);
		if (err) {
			pr_err("%s: %s: selecting HS200 failed (%d)\n",
				mmc_hostname(host), __func__, err);
			return err;
		}

		mmc_set_bus_speed(host->card);

		err = mmc_hs200_tuning(host->card);
		if (err) {
			pr_err("%s: %s: hs200 tuning failed (%d)\n",
				mmc_hostname(host), __func__, err);
			return err;
		}

		if (!(host->card->mmc_avail_type & EXT_CSD_CARD_TYPE_HS400)) {
			pr_debug("%s: card does not support HS400\n",
				mmc_hostname(host));
			return 0;
		}
	}

	err = mmc_select_hs400(host->card);
	if (err) {
		pr_err("%s: %s: select hs400 failed (%d)\n",
			mmc_hostname(host), __func__, err);
		return err;
	}

	return 0;
}

static int mmc_set_clock_bus_speed(struct mmc_card *card, unsigned long freq)
{
	int err = 0;

	if (freq == MMC_HS200_MAX_DTR)
		err = mmc_scale_high(card->host);
	else
		err = mmc_scale_low(card->host, freq);

	return err;
}

static inline unsigned long mmc_ddr_freq_accommodation(unsigned long freq)
{
	if (freq == MMC_HIGH_DDR_MAX_DTR)
		return freq;

	return freq/2;
}

static int mmc_change_bus_speed(struct mmc_host *host, unsigned long *freq)
{
	int err = 0;
	struct mmc_card *card;
	unsigned long actual_freq;

	card = host->card;

	if (!card || !freq) {
		err = -EINVAL;
		goto out;
	}
	actual_freq = *freq;

	WARN_ON(!host->claimed);

	if (mmc_card_hs400(card) ||
		(*freq == MMC_HS200_MAX_DTR)) {
		err = mmc_set_clock_bus_speed(card, *freq);
		if (err) {
			pr_err("%s: %s: failed (%d)to set bus and clock speed (freq=%lu)\n",
				mmc_hostname(host), __func__, err, *freq);
			goto out;
		}
	} else if (mmc_card_hs200(host->card)) {
		mmc_set_clock(host, *freq);
		err = mmc_hs200_tuning(host->card);
		if (err) {
			pr_warn("%s: %s: tuning execution failed %d\n",
				mmc_hostname(card->host),
				__func__, err);
			mmc_set_clock(host, host->clk_scaling.curr_freq);
		}
	} else {
		if (mmc_card_ddr52(host->card))
			actual_freq = mmc_ddr_freq_accommodation(*freq);
		mmc_set_clock(host, actual_freq);
	}

out:
	return err;
}

static int mmc_init_card(struct mmc_host *host, u32 ocr,
	struct mmc_card *oldcard)
{
	struct mmc_card *card;
	int err;
	u32 cid[4];
	u32 rocr;
	u8 *ext_csd = NULL;

	BUG_ON(!host);
	WARN_ON(!host->claimed);

	
	if (!mmc_host_is_spi(host))
		mmc_set_bus_mode(host, MMC_BUSMODE_OPENDRAIN);

reinit:
	mmc_go_idle(host);

	
	err = mmc_send_op_cond(host, ocr | (1 << 30), &rocr);
	if (err) {
		pr_err("%s: %s: mmc_send_op_cond() fails %d\n",
				mmc_hostname(host), __func__, err);
		goto err;
	}

	if (mmc_host_is_spi(host)) {
		err = mmc_spi_set_crc(host, use_spi_crc);
		if (err) {
			pr_err("%s: %s: mmc_spi_set_crc() fails %d\n",
					mmc_hostname(host), __func__, err);
			goto err;
		}
	}

	if (mmc_host_is_spi(host))
		err = mmc_send_cid(host, cid);
	else
		err = mmc_all_send_cid(host, cid);
	if (err) {
		pr_err("%s: %s: mmc_send_cid() fails %d\n",
				mmc_hostname(host), __func__, err);
		goto err;
	}

	if (oldcard) {
		if (memcmp(cid, oldcard->raw_cid, sizeof(cid)) != 0) {
			err = -ENOENT;
			pr_err("%s: %s: CID memcmp failed %d\n",
					mmc_hostname(host), __func__, err);
			goto err;
		}

		card = oldcard;
	} else {
		card = mmc_alloc_card(host, &mmc_type);
		if (IS_ERR(card)) {
			err = PTR_ERR(card);
			pr_err("%s: %s: no memory to allocate for card %d\n",
					mmc_hostname(host), __func__, err);
			goto err;
		}

		card->ocr = ocr;
		card->type = MMC_TYPE_MMC;
		card->rca = 1;
		memcpy(card->raw_cid, cid, sizeof(card->raw_cid));
		host->card = card;
		card->reboot_notify.notifier_call = mmc_reboot_notify;
	}

	if (!mmc_host_is_spi(host)) {
		err = mmc_set_relative_addr(card);
		if (err) {
			pr_err("%s: %s: mmc_set_relative_addr() fails %d\n",
					mmc_hostname(host), __func__, err);
			goto free_card;
		}

		mmc_set_bus_mode(host, MMC_BUSMODE_PUSHPULL);
	}

	if (!oldcard) {
		err = mmc_send_csd(card, card->raw_csd);
		if (err) {
			pr_err("%s: %s: mmc_send_csd() fails %d\n",
					mmc_hostname(host), __func__, err);
			goto free_card;
		}

		err = mmc_decode_csd(card);
		if (err) {
			pr_err("%s: %s: mmc_decode_csd() fails %d\n",
					mmc_hostname(host), __func__, err);
			goto free_card;
		}
		err = mmc_decode_cid(card);
		if (err) {
			pr_err("%s: %s: mmc_decode_cid() fails %d\n",
					mmc_hostname(host), __func__, err);
			goto free_card;
		}
	}

	if (card->csd.dsr_imp && host->dsr_req)
		mmc_set_dsr(host);

	if (!mmc_host_is_spi(host)) {
		err = mmc_select_card(card);
		if (err) {
			pr_err("%s: %s: mmc_select_card() fails %d\n",
					mmc_hostname(host), __func__, err);
			goto free_card;
		}
	}

	if (!oldcard) {

		err = mmc_get_ext_csd(card, &ext_csd);
		if (err) {
			pr_err("%s: %s: mmc_get_ext_csd() fails %d\n",
					mmc_hostname(host), __func__, err);
			goto free_card;
		}
		card->cached_ext_csd = ext_csd;
		err = mmc_read_ext_csd(card, ext_csd);
		if (err) {
			pr_err("%s: %s: mmc_read_ext_csd() fails %d\n",
					mmc_hostname(host), __func__, err);
			goto free_card;
		}

		if (!(mmc_card_blockaddr(card)) && (rocr & (1<<30)))
			mmc_card_set_blockaddr(card);

		
		mmc_set_erase_size(card);

		if (card->ext_csd.sectors && (rocr & MMC_CARD_SECTOR_ADDR))
			mmc_card_set_blockaddr(card);
	}

	if (card->ext_csd.partition_setting_completed ||
	    (card->ext_csd.rev >= 3 && (host->caps2 & MMC_CAP2_HC_ERASE_SZ))) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_ERASE_GROUP_DEF, 1,
				 card->ext_csd.generic_cmd6_time);

		if (err && err != -EBADMSG) {
			pr_err("%s: %s: mmc_switch() for ERASE_GRP_DEF fails %d\n",
					mmc_hostname(host), __func__, err);
			goto free_card;
		}

		if (err) {
			err = 0;
			card->ext_csd.enhanced_area_offset = -EINVAL;
			card->ext_csd.enhanced_area_size = -EINVAL;
		} else {
			card->ext_csd.erase_group_def = 1;
			mmc_set_erase_size(card);
		}
	}

	if (card->ext_csd.part_config & EXT_CSD_PART_CONFIG_ACC_MASK) {
		card->ext_csd.part_config &= ~EXT_CSD_PART_CONFIG_ACC_MASK;
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_PART_CONFIG,
				 card->ext_csd.part_config,
				 card->ext_csd.part_time);
		if (err && err != -EBADMSG) {
			pr_err("%s: %s: mmc_switch() for PART_CONFIG fails %d\n",
					mmc_hostname(host), __func__, err);
			goto free_card;
		}
		card->part_curr = card->ext_csd.part_config &
				  EXT_CSD_PART_CONFIG_ACC_MASK;
	}

	if (card->ext_csd.rev >= 6) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_POWER_OFF_NOTIFICATION,
				 EXT_CSD_POWER_ON,
				 card->ext_csd.generic_cmd6_time);
		if (err && err != -EBADMSG) {
			pr_err("%s: %s: mmc_switch() for POWER_ON PON fails %d\n",
					mmc_hostname(host), __func__, err);
			goto free_card;
		}

		if (!err)
			card->ext_csd.power_off_notification = EXT_CSD_POWER_ON;
	}

	err = mmc_select_timing(card);
	if (err) {
		pr_err("%s: %s: mmc_select_timing() fails %d\n",
					mmc_hostname(host), __func__, err);
		goto free_card;
	}

	if (mmc_card_hs200(card)) {
		err = mmc_hs200_tuning(card);
		if (err)
			goto err;

		err = mmc_select_hs400(card);
		if (err)
			goto err;
	} else if (mmc_card_hs(card)) {
		
		err = mmc_select_bus_width(card);
		if (!IS_ERR_VALUE(err)) {
			err = mmc_select_hs_ddr(card);
			if (err)
				goto err;
		}
	}

	card->clk_scaling_lowest = host->f_min;
	if ((card->mmc_avail_type | EXT_CSD_CARD_TYPE_HS400) ||
			(card->mmc_avail_type | EXT_CSD_CARD_TYPE_HS200))
		card->clk_scaling_highest = card->ext_csd.hs200_max_dtr;
	else if ((card->mmc_avail_type | EXT_CSD_CARD_TYPE_HS) ||
			(card->mmc_avail_type | EXT_CSD_CARD_TYPE_DDR_52))
		card->clk_scaling_highest = card->ext_csd.hs_max_dtr;
	else
		card->clk_scaling_highest = card->csd.max_dtr;

	mmc_select_powerclass(card);

	if (card->ext_csd.hpi) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_HPI_MGMT, 1,
				card->ext_csd.generic_cmd6_time);
		if (err && err != -EBADMSG) {
			pr_err("%s: %s: mmc_switch() for HPI_MGMT fails %d\n",
					mmc_hostname(host), __func__, err);
			goto free_card;
		}
		if (err) {
			pr_warn("%s: Enabling HPI failed\n",
				mmc_hostname(card->host));
			err = 0;
		} else
			card->ext_csd.hpi_en = 1;
	}

	if (card->ext_csd.cache_size > 0) {
		if (card->ext_csd.hpi_en &&
			(!(card->quirks & MMC_QUIRK_CACHE_DISABLE))) {
			err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					EXT_CSD_CACHE_CTRL, 1,
					card->ext_csd.generic_cmd6_time);
			if (err && err != -EBADMSG) {
				pr_err("%s: %s: fail on CACHE_CTRL ON %d\n",
					mmc_hostname(host), __func__, err);
				goto free_card;
			}

			if (err) {
				pr_warn("%s: Cache is supported, but failed to turn on (%d)\n",
					mmc_hostname(card->host), err);
				card->ext_csd.cache_ctrl = 0;
				err = 0;
			} else {
				card->ext_csd.cache_ctrl = 1;
			}
			
			if (card->ext_csd.cache_ctrl &&
					card->ext_csd.barrier_support) {
				err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					EXT_CSD_BARRIER_CTRL, 1,
					card->ext_csd.generic_cmd6_time);
				if (err && err != -EBADMSG) {
					pr_err("%s: %s: mmc_switch() for BARRIER_CTRL fails %d\n",
						mmc_hostname(host), __func__,
						err);
					goto free_card;
				}
				if (err) {
					pr_warn("%s: Barrier is supported but failed to turn on (%d)\n",
						mmc_hostname(card->host), err);
					card->ext_csd.barrier_en = 0;
					err = 0;
				} else {
					card->ext_csd.barrier_en = 1;
				}
			}
		} else {
			err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					EXT_CSD_CACHE_CTRL, 0,
					card->ext_csd.generic_cmd6_time);
			if (err) {
				pr_err("%s: %s: fail on CACHE_CTRL OFF %d\n",
					mmc_hostname(host), __func__, err);
				goto free_card;
			}
		}
	}

	if (card->ext_csd.max_packed_writes >= 3 &&
	    card->ext_csd.max_packed_reads >= 5 &&
	    host->caps2 & MMC_CAP2_PACKED_CMD) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_EXP_EVENTS_CTRL,
				EXT_CSD_PACKED_EVENT_EN,
				card->ext_csd.generic_cmd6_time);
		if (err && err != -EBADMSG) {
			pr_err("%s: %s: mmc_switch() for EXP_EVENTS_CTRL fails %d\n",
					mmc_hostname(host), __func__, err);
			goto free_card;
		}
		if (err) {
			pr_warn("%s: Enabling packed event failed\n",
				mmc_hostname(card->host));
			card->ext_csd.packed_event_en = 0;
			err = 0;
		} else {
			card->ext_csd.packed_event_en = 1;
		}

	}

	if (!oldcard) {
		if ((host->caps2 & MMC_CAP2_PACKED_CMD) &&
		    (card->ext_csd.max_packed_writes > 0)) {
			card->wr_pack_stats.packing_events = kzalloc(
				(card->ext_csd.max_packed_writes + 1) *
				sizeof(*card->wr_pack_stats.packing_events),
				GFP_KERNEL);
			if (!card->wr_pack_stats.packing_events) {
				pr_err("%s: %s: no memory for packing events\n",
						mmc_hostname(host), __func__);
				goto free_card;
			}
		}
	}

	if (mmc_card_support_auto_bkops(card)) {
		(void)mmc_set_auto_bkops(card, true);
	}

	if (card->ext_csd.cmdq_support && (card->host->caps2 &
					   MMC_CAP2_CMD_QUEUE)) {
		err = mmc_select_cmdq(card);
		if (err) {
			pr_err("%s: selecting CMDQ mode: failed: %d\n",
					   mmc_hostname(card->host), err);
			card->ext_csd.cmdq_support = 0;
			oldcard = card;
			goto reinit;
		}
	}

	return 0;

free_card:
	if (!oldcard) {
		host->card = NULL;
		mmc_remove_card(card);
	}
err:
	return err;
}

static int mmc_can_sleepawake(struct mmc_host *host)
{
	return host && (host->caps2 & MMC_CAP2_SLEEP_AWAKE) && host->card &&
		(host->card->ext_csd.rev >= 3);
}

static int mmc_sleepawake(struct mmc_host *host, bool sleep)
{
	struct mmc_command cmd = {0};
	struct mmc_card *card = host->card;
	unsigned int timeout_ms;
	int err;

	if (!card) {
		pr_err("%s: %s: invalid card\n", mmc_hostname(host), __func__);
		return -EINVAL;
	}

	timeout_ms = DIV_ROUND_UP(card->ext_csd.sa_timeout, 10000);
	if (card->ext_csd.rev >= 3 &&
		card->part_curr == EXT_CSD_PART_CONFIG_ACC_RPMB) {
		u8 part_config = card->ext_csd.part_config;

		part_config &= ~EXT_CSD_PART_CONFIG_ACC_MASK;
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_PART_CONFIG,
				 part_config,
				 card->ext_csd.part_time);
		if (err) {
			pr_err("%s: %s: failed to switch to default part config %x\n",
				mmc_hostname(host), __func__, part_config);
			return err;
		}
		card->ext_csd.part_config = part_config;
		card->part_curr = card->ext_csd.part_config &
				  EXT_CSD_PART_CONFIG_ACC_MASK;
	}

	if (sleep) {
		err = mmc_deselect_cards(host);
		if (err)
			return err;
	}

	cmd.opcode = MMC_SLEEP_AWAKE;
	cmd.arg = card->rca << 16;
	if (sleep)
		cmd.arg |= 1 << 15;

	if (host->max_busy_timeout && (timeout_ms > host->max_busy_timeout)) {
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	} else {
		cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
		cmd.busy_timeout = timeout_ms;
	}

	err = mmc_wait_for_cmd(host, &cmd, 0);
	if (err)
		return err;

	if (!cmd.busy_timeout || !(host->caps & MMC_CAP_WAIT_WHILE_BUSY))
		mmc_delay(timeout_ms);

	if (!sleep)
		err = mmc_select_card(card);

	return err;
}

static int mmc_can_poweroff_notify(const struct mmc_card *card)
{
	return card &&
		mmc_card_mmc(card) &&
		(card->ext_csd.power_off_notification == EXT_CSD_POWER_ON);
}

static int mmc_poweroff_notify(struct mmc_card *card, unsigned int notify_type)
{
	unsigned int timeout = card->ext_csd.generic_cmd6_time;
	int err;

	
	if (notify_type == EXT_CSD_POWER_OFF_LONG)
		timeout = card->ext_csd.power_off_longtime;

	err = __mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_POWER_OFF_NOTIFICATION,
			notify_type, timeout, true, false, false);
	if (err)
		pr_err("%s: Power Off Notification timed out, %u\n",
		       mmc_hostname(card->host), timeout);

	
	card->ext_csd.power_off_notification = EXT_CSD_NO_POWER_NOTIFICATION;

	return err;
}

int mmc_send_pon(struct mmc_card *card)
{
	int err = 0;
	struct mmc_host *host = card->host;

	if (!mmc_can_poweroff_notify(card))
		goto out;

	mmc_get_card(card);
	if (card->pon_type & MMC_LONG_PON)
		err = mmc_poweroff_notify(host->card, EXT_CSD_POWER_OFF_LONG);
	else if (card->pon_type & MMC_SHRT_PON)
		err = mmc_poweroff_notify(host->card, EXT_CSD_POWER_OFF_SHORT);
	if (err)
		pr_warn("%s: error %d sending PON type %u",
			mmc_hostname(host), err, card->pon_type);
	mmc_put_card(card);
out:
	return err;
}

static void mmc_remove(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(!host->card);

	unregister_reboot_notifier(&host->card->reboot_notify);

	mmc_exit_clk_scaling(host);
	mmc_remove_card(host->card);

	mmc_claim_host(host);
	host->card = NULL;
	mmc_release_host(host);
}

static int mmc_alive(struct mmc_host *host)
{
	return mmc_send_status(host->card, NULL);
}

static void mmc_detect(struct mmc_host *host)
{
	int err;

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_get_card(host->card);

	err = _mmc_detect_card_removed(host);

	mmc_put_card(host->card);

	if (err) {
		mmc_remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
	}
}

static int mmc_cache_card_ext_csd(struct mmc_host *host)
{
	int err;
	u8 *ext_csd;
	struct mmc_card *card = host->card;

	err = mmc_get_ext_csd(card, &ext_csd);
	if (err || !ext_csd) {
		pr_err("%s: %s: mmc_get_ext_csd failed (%d)\n",
			mmc_hostname(host), __func__, err);
		return err;
	}

	
	card->ext_csd.raw_ext_csd_cmdq = ext_csd[EXT_CSD_CMDQ];
	card->ext_csd.raw_ext_csd_cache_ctrl = ext_csd[EXT_CSD_CACHE_CTRL];
	card->ext_csd.raw_ext_csd_bus_width = ext_csd[EXT_CSD_BUS_WIDTH];
	card->ext_csd.raw_ext_csd_hs_timing = ext_csd[EXT_CSD_HS_TIMING];

	mmc_free_ext_csd(ext_csd);

	return 0;
}

static int mmc_test_awake_ext_csd(struct mmc_host *host)
{
	int err;
	u8 *ext_csd;
	struct mmc_card *card = host->card;

	err = mmc_get_ext_csd(card, &ext_csd);
	if (err) {
		pr_err("%s: %s: mmc_get_ext_csd failed (%d)\n",
			mmc_hostname(host), __func__, err);
		return err;
	}

	
	pr_debug("%s: %s: type(cached:current) cmdq(%d:%d) cache_ctrl(%d:%d) bus_width (%d:%d) timing(%d:%d)\n",
		mmc_hostname(host), __func__,
		card->ext_csd.raw_ext_csd_cmdq,
		ext_csd[EXT_CSD_CMDQ],
		card->ext_csd.raw_ext_csd_cache_ctrl,
		ext_csd[EXT_CSD_CACHE_CTRL],
		card->ext_csd.raw_ext_csd_bus_width,
		ext_csd[EXT_CSD_BUS_WIDTH],
		card->ext_csd.raw_ext_csd_hs_timing,
		ext_csd[EXT_CSD_HS_TIMING]);

	err = !((card->ext_csd.raw_ext_csd_cmdq ==
			ext_csd[EXT_CSD_CMDQ]) &&
		(card->ext_csd.raw_ext_csd_cache_ctrl ==
			ext_csd[EXT_CSD_CACHE_CTRL]) &&
		(card->ext_csd.raw_ext_csd_bus_width ==
			ext_csd[EXT_CSD_BUS_WIDTH]) &&
		(card->ext_csd.raw_ext_csd_hs_timing ==
			ext_csd[EXT_CSD_HS_TIMING]));

	mmc_free_ext_csd(ext_csd);

	return err;
}

static int _mmc_suspend(struct mmc_host *host, bool is_suspend)
{
	int err = 0;

	BUG_ON(!host);
	BUG_ON(!host->card);

	err = mmc_suspend_clk_scaling(host);
	if (err) {
		pr_err("%s: %s: fail to suspend clock scaling (%d)\n",
			mmc_hostname(host), __func__, err);
		goto out;
	}

	mmc_claim_host(host);

	if (mmc_card_suspended(host->card)) {
		pr_info("%s: %s: device has been suspended\n",
			mmc_hostname(host), __func__);
		goto out;
	}

	if (host->card->cmdq_init) {
		BUG_ON(host->cmdq_ctx.active_reqs);

		err = mmc_cmdq_halt(host, true);
		if (err) {
			pr_err("%s: halt: failed: %d\n", __func__, err);
			goto out;
		}
		mmc_host_clk_hold(host);
		host->cmdq_ops->disable(host, true);
		mmc_host_clk_release(host);
	}

	if (mmc_card_doing_bkops(host->card)) {
		err = mmc_stop_bkops(host->card);
		if (err) {
			pr_err("%s: %s: fail to stop bkops (%d)\n",
				mmc_hostname(host), __func__, err);
			goto out;
		}
	}

	if (mmc_card_doing_auto_bkops(host->card)) {
		err = mmc_set_auto_bkops(host->card, false);
		if (err) {
			pr_err("%s: %s: fail to cancel auto bkops (%d)\n",
				mmc_hostname(host), __func__, err);
			goto out;
		}
	}

	err = mmc_flush_cache(host->card);
	if (err) {
		pr_err("%s: %s: fail to flush cache (%d)\n",
			mmc_hostname(host), __func__, err);
		goto out;
	}

	if (mmc_can_sleepawake(host)) {
		memcpy(&host->cached_ios, &host->ios, sizeof(host->cached_ios));
		mmc_cache_card_ext_csd(host);
		err = mmc_sleepawake(host, true);
		if (err) {
			pr_err("%s: %s: fail to sleep awake (%d)\n",
				mmc_hostname(host), __func__, err);
		}
	} else if (!mmc_host_is_spi(host)) {
		err = mmc_deselect_cards(host);
		if (err) {
			pr_err("%s: %s: fail to deselect card (%d)\n",
				mmc_hostname(host), __func__, err);
		}
	}

	if (!err) {
		mmc_power_off(host);
		mmc_card_set_suspended(host->card);
	}
out:
	
	if (!is_suspend && host->card->cmdq_init)
		wake_up(&host->cmdq_ctx.wait);

	mmc_release_host(host);
	return err;
}

static int mmc_partial_init(struct mmc_host *host)
{
	int err = 0;
	struct mmc_card *card = host->card;

	pr_debug("%s: %s: starting partial init\n",
		mmc_hostname(host), __func__);

	mmc_set_bus_width(host, host->cached_ios.bus_width);
	mmc_set_timing(host, host->cached_ios.timing);
	mmc_set_clock(host, host->cached_ios.clock);
	mmc_set_bus_mode(host, host->cached_ios.bus_mode);

	mmc_host_clk_hold(host);

	if (mmc_card_hs200(card) || mmc_card_hs400(card)) {
		if (card->ext_csd.strobe_support && host->ops->enhanced_strobe)
			err = host->ops->enhanced_strobe(host);
		else
			err = host->ops->execute_tuning(host,
				MMC_SEND_TUNING_BLOCK_HS200);
		if (err)
			pr_warn("%s: %s: tuning execution failed (%d)\n",
				mmc_hostname(host), __func__, err);
	}

	err = mmc_test_awake_ext_csd(host);
	if (err) {
		pr_debug("%s: %s: fail on ext_csd read (%d)\n",
			mmc_hostname(host), __func__, err);
		goto out;
	}
	pr_debug("%s: %s: reading and comparing ext_csd successful\n",
		mmc_hostname(host), __func__);

	
	if (!mmc_card_doing_auto_bkops(host->card)) {
		err = mmc_set_auto_bkops(host->card, true);
		if (err) {
			pr_err("%s: %s: fail to enable auto bkops (%d)\n",
				mmc_hostname(host), __func__, err);
		}
	}

	if (card->ext_csd.cmdq_support && (card->host->caps2 &
					   MMC_CAP2_CMD_QUEUE)) {
		err = mmc_select_cmdq(card);
		if (err) {
			pr_warn("%s: %s: enabling CMDQ mode failed (%d)\n",
					mmc_hostname(card->host),
					__func__, err);
		}
	}
out:
	mmc_host_clk_release(host);

	pr_debug("%s: %s: done partial init (%d)\n",
		mmc_hostname(host), __func__, err);

	return err;
}

static int mmc_suspend(struct mmc_host *host)
{
	int err;
	ktime_t start = ktime_get();

	err = _mmc_suspend(host, true);
	if (!err) {
		pm_runtime_disable(&host->card->dev);
		pm_runtime_set_suspended(&host->card->dev);
	} else {
		pr_err("%s: %s: failed, err:%d\n", mmc_hostname(host), __func__, err);
	}

	trace_mmc_suspend(mmc_hostname(host), err,
			ktime_to_us(ktime_sub(ktime_get(), start)));
	return err;
}

static int _mmc_resume(struct mmc_host *host)
{
	int err = -ENOSYS;
	int retries;

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);

	if (!mmc_card_suspended(host->card)) {
		mmc_release_host(host);
		goto out;
	}

	mmc_power_up(host, host->card->ocr);
	retries = 3;
	while (retries) {
		if (mmc_can_sleepawake(host)) {
			err = mmc_sleepawake(host, false);
			if (!err)
				err = mmc_partial_init(host);
			if (err)
				pr_err("%s: %s: awake failed (%d), fallback to full init\n",
					mmc_hostname(host), __func__, err);
		}

		if (err)
			err = mmc_init_card(host, host->card->ocr, host->card);

		if (err) {
			pr_err("%s: MMC card re-init failed rc = %d (retries = %d)\n",
			       mmc_hostname(host), err, retries);
			retries--;
			mmc_power_off(host);
			usleep_range(5000, 5500);
			mmc_power_up(host, host->card->ocr);
			mmc_select_voltage(host, host->card->ocr);
			continue;
		}
		break;
	}
	if (!err && mmc_card_cmdq(host->card)) {
		err = mmc_cmdq_halt(host, false);
		if (err)
			pr_err("%s: un-halt: failed: %d\n", __func__, err);
	}
	mmc_card_clr_suspended(host->card);

	mmc_release_host(host);

	err = mmc_resume_clk_scaling(host);
	if (err)
		pr_err("%s: %s: fail to resume clock scaling (%d)\n",
			mmc_hostname(host), __func__, err);

out:
	return err;
}

static int mmc_resume(struct mmc_host *host)
{
	int err = 0;
	ktime_t start = ktime_get();

	if (!(host->caps & MMC_CAP_RUNTIME_RESUME)) {
		err = _mmc_resume(host);
		pm_runtime_set_active(&host->card->dev);
		pm_runtime_mark_last_busy(&host->card->dev);
	}
	pm_runtime_enable(&host->card->dev);

	trace_mmc_resume(mmc_hostname(host), err,
			ktime_to_us(ktime_sub(ktime_get(), start)));

	return err;
}

static int mmc_runtime_suspend(struct mmc_host *host)
{
	int err;
	ktime_t start = ktime_get();

	if (!(host->caps & MMC_CAP_AGGRESSIVE_PM))
		return 0;

	err = _mmc_suspend(host, false);
	if (err)
		pr_err("%s: error %d doing aggessive suspend\n",
			mmc_hostname(host), err);

	trace_mmc_runtime_suspend(mmc_hostname(host), err,
			ktime_to_us(ktime_sub(ktime_get(), start)));
	return err;
}

static int mmc_runtime_resume(struct mmc_host *host)
{
	int err;
	ktime_t start = ktime_get();

	if (!(host->caps & (MMC_CAP_AGGRESSIVE_PM | MMC_CAP_RUNTIME_RESUME)))
		return 0;

	err = _mmc_resume(host);
	if (err)
		pr_err("%s: error %d doing aggessive resume\n",
			mmc_hostname(host), err);

	trace_mmc_runtime_resume(mmc_hostname(host), err,
			ktime_to_us(ktime_sub(ktime_get(), start)));

	return err;
}

static int mmc_power_restore(struct mmc_host *host)
{
	int ret;

	

	ret = mmc_suspend_clk_scaling(host);
	if (ret) {
		pr_err("%s: %s: fail to suspend clock scaling (%d)\n",
			mmc_hostname(host), __func__, ret);
		return ret;
	}

	ret = mmc_init_card(host, host->card->ocr, host->card);

	ret = mmc_resume_clk_scaling(host);
	if (ret)
		pr_err("%s: %s: fail to resume clock scaling (%d)\n",
			mmc_hostname(host), __func__, ret);

	return ret;
}

#define NO_AUTOSUSPEND	(-1)
static int mmc_runtime_idle(struct mmc_host *host)
{
	int err = 0;
	bool halt_cmdq;

	BUG_ON(!host->card);

	mmc_claim_host(host);

	halt_cmdq = mmc_card_cmdq(host->card) &&
			(host->card->bkops.needs_check ||
			 host->card->bkops.needs_manual);

	if (host->cmdq_ctx.active_reqs)
		goto no_suspend;

	if (halt_cmdq) {
		err = mmc_cmdq_halt(host, true);
		if (err) {
			pr_err("%s: %s failed to halt cmdq (%d)\n",
					mmc_hostname(host), __func__, err);
			goto out;
		}
	}

	if (host->card->bkops.needs_manual)
		host->card->bkops.needs_check = false;

	if (host->card->bkops.needs_check) {
		mmc_check_bkops(host->card);
		host->card->bkops.needs_check = false;

	}

	if (host->card->bkops.needs_manual)
		mmc_start_manual_bkops(host->card);

	if (halt_cmdq) {
		err = mmc_cmdq_halt(host, false);
		if (err)
			pr_err("%s: %s failed to unhalt cmdq (%d)\n",
					mmc_hostname(host), __func__, err);
	}

out:
	pm_schedule_suspend(&host->card->dev, MMC_AUTOSUSPEND_DELAY_MS);
no_suspend:
	mmc_release_host(host);
	pm_runtime_mark_last_busy(&host->card->dev);
	
	return (err) ? err : NO_AUTOSUSPEND;
}

static const struct mmc_bus_ops mmc_ops = {
	.remove = mmc_remove,
	.detect = mmc_detect,
	.suspend = mmc_suspend,
	.resume = mmc_resume,
	.runtime_suspend = mmc_runtime_suspend,
	.runtime_resume = mmc_runtime_resume,
	.runtime_idle = mmc_runtime_idle,
	.power_restore = mmc_power_restore,
	.alive = mmc_alive,
	.change_bus_speed = mmc_change_bus_speed,
};

int mmc_attach_mmc(struct mmc_host *host)
{
	int err;
	u32 ocr, rocr;

	BUG_ON(!host);
	WARN_ON(!host->claimed);

	
	if (!mmc_host_is_spi(host))
		mmc_set_bus_mode(host, MMC_BUSMODE_OPENDRAIN);

	err = mmc_send_op_cond(host, 0, &ocr);
	if (err)
		return err;

	mmc_attach_bus(host, &mmc_ops);
	if (host->ocr_avail_mmc)
		host->ocr_avail = host->ocr_avail_mmc;

	if (mmc_host_is_spi(host)) {
		err = mmc_spi_read_ocr(host, 1, &ocr);
		if (err)
			goto err;
	}

	rocr = mmc_select_voltage(host, ocr);

	if (!rocr) {
		err = -EINVAL;
		goto err;
	}

	err = mmc_init_card(host, rocr, NULL);
	if (err)
		goto err;

	mmc_release_host(host);
	err = mmc_add_card(host->card);
	mmc_claim_host(host);
	if (err)
		goto remove_card;

	err = mmc_init_clk_scaling(host);
	if (err)
		goto remove_card;

	register_reboot_notifier(&host->card->reboot_notify);

	return 0;

remove_card:
	mmc_release_host(host);
	mmc_remove_card(host->card);
	mmc_claim_host(host);
	host->card = NULL;
err:
	mmc_detach_bus(host);

	pr_err("%s: error %d whilst initialising MMC card\n",
		mmc_hostname(host), err);

	return err;
}
