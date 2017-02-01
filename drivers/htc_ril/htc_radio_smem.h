#ifndef _HTC_RADIO_SMEM_H
#define _HTC_RADIO_SMEM_H

#include <linux/types.h>

//hTC radio smem driver version
#define HTC_RADIO_SMEM_VERSION	0x20140901
//smlog magic number
#define HTC_SMLOG_MAGIC_NUMBER  0x534D4C4F

//APP run mode
#define APP_IN_BOOTLOADER       (1 << 0)
#define APP_IN_TESTBOOTLOADER   (1 << 1)
#define APP_IN_DIAG             (1 << 2)
#define APP_IN_RECOVERY         (1 << 3)
#define APP_IN_MFGKERNEL        (1 << 4)
#define APP_IN_HLOS             (1 << 5)
#define APP_IN_FTM              (1 << 6)

struct htc_smem_type
{
	uint32_t	version;
	uint32_t	struct_size;

	uint32_t	htc_smem_app_run_mode;
	uint32_t	htc_smem_ce_radio_dbg_flag;
	uint32_t	htc_smem_ce_radio_dbg_flag_ext1;
	uint32_t	htc_smem_ce_radio_dbg_flag_ext2;

	uint32_t	htc_smlog_magic;
	uint32_t	htc_smlog_base;
	uint32_t	htc_smlog_size;

	uint8_t		htc_rom_ver[16];
	uint32_t	htc_smem_is_nv_backup;
	uint8_t         RCMS_name[64];
	uint32_t        htc_smem_ce_radio_dbg_ril_fatal;
	uint8_t         SKU_Name[128];
	/* totally used 252 byte */
	uint8_t		reserved[1796];
	/* totally 2048 bytes */
};

struct htc_secure_smem_type
{
  /* ========= belows are App write ==================== */
  uint32_t        version;
  uint32_t        struct_size;
  uint32_t        secure_flag;
  uint32_t        htc_smem_pid;
  uint8_t         htc_smem_cid[8];
  uint8_t         htc_smem_imei[16];
  uint8_t         htc_smem_imei2[16];
  uint8_t         htc_smem_meid[16];
//SKU
  uint8_t         htc_smem_skuid[48];

  /* totally used 104 byte */
  uint8_t         reserved[8072];
  /* totally 8192 bytes */
};

#endif /* end of _HTC_RADIO_SMEM_H */
