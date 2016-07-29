/* Copyright (c) 2014-2016 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "SMBCHG: %s: " fmt, __func__

#include <linux/spmi.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/spmi.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/leds.h>
#include <linux/rtc.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/batterydata-lib.h>
#include <linux/of_batterydata.h>
#include <linux/msm_bcl.h>
#include <linux/ktime.h>
#ifdef CONFIG_HTC_BATT
#include <linux/power/htc_battery.h>
#include <linux/htc_flags.h>
#endif
#include "pmic-voter.h"

#define _SMB_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))
struct smbchg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct parallel_usb_cfg {
	struct power_supply		*psy;
	int				min_current_thr_ma;
	int				min_9v_current_thr_ma;
	int				allowed_lowering_ma;
	int				current_max_ma;
	bool				avail;
	struct mutex			lock;
	int				initial_aicl_ma;
	ktime_t				last_disabled;
	bool				enabled_once;
};

struct ilim_entry {
	int vmin_uv;
	int vmax_uv;
	int icl_pt_ma;
	int icl_lv_ma;
	int icl_hv_ma;
};

struct ilim_map {
	int			num;
	struct ilim_entry	*entries;
};

struct smbchg_version_tables {
	const int			*dc_ilim_ma_table;
	int				dc_ilim_ma_len;
	const int			*usb_ilim_ma_table;
	int				usb_ilim_ma_len;
	const int			*iterm_ma_table;
	int				iterm_ma_len;
	const int			*fcc_comp_table;
	int				fcc_comp_len;
	const int			*aicl_rerun_period_table;
	int				aicl_rerun_period_len;
	int				rchg_thr_mv;
};

struct smbchg_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	int				schg_version;

	
	u16				chgr_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				dc_chgpth_base;
	u16				otg_base;
	u16				misc_base;

	int				fake_battery_soc;
	u8				revision[4];

	
	int				iterm_ma;
	int				usb_max_current_ma;
	int				typec_current_ma;
	int				dc_max_current_ma;
	int				dc_target_current_ma;
	int				cfg_fastchg_current_ma;
	int				fastchg_current_ma;
	int				vfloat_mv;
	int				fastchg_current_comp;
	int				float_voltage_comp;
	int				resume_delta_mv;
	int				safety_time;
	int				prechg_safety_time;
	int				bmd_pin_src;
	int				jeita_temp_hard_limit;
	int				aicl_rerun_period_s;
	bool				use_vfloat_adjustments;
	bool				iterm_disabled;
	bool				bmd_algo_disabled;
	bool				soft_vfloat_comp_disabled;
	bool				chg_enabled;
	bool				charge_unknown_battery;
	bool				chg_inhibit_en;
	bool				chg_inhibit_source_fg;
	bool				low_volt_dcin;
	bool				cfg_chg_led_support;
	bool				cfg_chg_led_sw_ctrl;
	bool				vbat_above_headroom;
	bool				force_aicl_rerun;
	bool				hvdcp3_supported;
	bool				restricted_charging;
	bool				skip_usb_suspend_for_fake_battery;
	bool				hvdcp_not_supported;
	bool				otg_pinctrl;
	u8				original_usbin_allowance;
	struct parallel_usb_cfg		parallel;
	struct delayed_work		parallel_en_work;
	struct dentry			*debug_root;
	struct smbchg_version_tables	tables;

	
	struct ilim_map			wipower_default;
	struct ilim_map			wipower_pt;
	struct ilim_map			wipower_div2;
	struct qpnp_vadc_chip		*vadc_dev;
	bool				wipower_dyn_icl_avail;
	struct ilim_entry		current_ilim;
	struct mutex			wipower_config;
	bool				wipower_configured;
	struct qpnp_adc_tm_btm_param	param;

	
	int				rpara_uohm;
	int				rslow_uohm;
	int				vled_max_uv;

	
	int				max_vbat_sample;
	int				n_vbat_samples;

	
	int				wake_reasons;
	int				previous_soc;
	int				usb_online;
	bool				dc_present;
	bool				usb_present;
	bool				batt_present;
	int				otg_retries;
	ktime_t				otg_enable_time;
	bool				aicl_deglitch_short;
	bool				safety_timer_en;
	bool				aicl_complete;
	bool				usb_ov_det;
	bool				otg_pulse_skip_dis;
	const char			*battery_type;
	enum power_supply_type		usb_supply_type;
	bool				very_weak_charger;
	bool				parallel_charger_detected;
	bool				chg_otg_enabled;
	bool				flash_triggered;
	bool				flash_active;
	bool				icl_disabled;
	u32				wa_flags;
	int				usb_icl_delta;
	bool				typec_dfp;

	
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;

	
	int				batt_hot_irq;
	int				batt_warm_irq;
	int				batt_cool_irq;
	int				batt_cold_irq;
	int				batt_missing_irq;
	int				vbat_low_irq;
	int				chg_hot_irq;
	int				chg_term_irq;
	int				taper_irq;
	bool				taper_irq_enabled;
	struct mutex			taper_irq_lock;
	int				recharge_irq;
	int				fastchg_irq;
	int				wdog_timeout_irq;
	int				power_ok_irq;
	int				dcin_uv_irq;
	int				usbin_uv_irq;
	int				usbin_ov_irq;
	int				src_detect_irq;
	int				otg_fail_irq;
	int				otg_oc_irq;
	int				aicl_done_irq;
	int				usbid_change_irq;
	int				chg_error_irq;
	bool				enable_aicl_wake;

	
	struct power_supply		*usb_psy;
	struct power_supply		batt_psy;
	struct power_supply		dc_psy;
	struct power_supply		*bms_psy;
	struct power_supply		*typec_psy;
	int				dc_psy_type;
	const char			*bms_psy_name;
	const char			*battery_psy_name;
	bool				psy_registered;

	struct smbchg_regulator		otg_vreg;
	struct smbchg_regulator		ext_otg_vreg;
	struct work_struct		usb_set_online_work;
	struct delayed_work		vfloat_adjust_work;
	struct delayed_work		hvdcp_det_work;
	spinlock_t			sec_access_lock;
	struct mutex			therm_lvl_lock;
	struct mutex			usb_set_online_lock;
	struct mutex			pm_lock;
	
	unsigned long			first_aicl_seconds;
	int				aicl_irq_count;
	struct mutex			usb_status_lock;
	bool				hvdcp_3_det_ignore_uv;
	struct completion		src_det_lowered;
	struct completion		src_det_raised;
	struct completion		usbin_uv_lowered;
	struct completion		usbin_uv_raised;
	int				pulse_cnt;
	struct led_classdev		led_cdev;
	bool				skip_usb_notification;
#ifdef CONFIG_HTC_BATT
	struct work_struct		usb_aicl_limit_current;
	struct delayed_work		usb_limit_max_current;
	struct delayed_work		rerun_apsd_work;
	struct delayed_work		iusb_5v_2a_detect_work;
	struct delayed_work		downgrade_iusb_work;
#ifdef CONFIG_HTC_BATT_WA_PCN0017
	struct delayed_work 	chk_cable_workable_work;
#endif 
	struct delayed_work		force_hvdcp_work;
	struct work_struct		hvdcp_redet_work;
        u32                             htc_wa_flags;
#endif 

#ifdef CONFIG_HTC_CHARGER
	struct power_supply		*htcchg_psy;
	bool				htcchg_ext_mode;
	bool				htcchg_error_flag;
#endif 
	u32				vchg_adc_channel;
	struct qpnp_vadc_chip		*vchg_vadc_dev;

	
	struct votable			*fcc_votable;
	struct votable			*usb_icl_votable;
	struct votable			*dc_icl_votable;
	struct votable			*usb_suspend_votable;
	struct votable			*dc_suspend_votable;
	struct votable			*battchg_suspend_votable;
	struct votable			*hw_aicl_rerun_disable_votable;
	struct votable			*hw_aicl_rerun_enable_indirect_votable;
	struct votable			*aicl_deglitch_short_votable;
};

#ifdef CONFIG_HTC_BATT
#define TEMP_MONITOR_DISABLE_BIT             0
#define JEITA_TEMP_HARD_LIMIT_DISABLE_BIT    BIT(5)
#define HOT_COLD_SL_COMP_BITS                0
#define HARD_TEMP_MASK                       SMB_MASK(6, 5)
#define SOFT_TEMP_MASK                       SMB_MASK(3, 0)
static struct smbchg_chip *the_chip;
static bool g_is_batt_full_eoc_stop = false;
#ifdef CONFIG_HTC_BATT_WA_PCN0017
static bool g_is_cable_workable_detect = false;
#endif 
#endif

#ifdef CONFIG_HTC_BATT
#define USB_MA_0       (0)
#define USB_MA_2       (2)
#define USB_MA_100     (100)
#define USB_MA_150     (150)
#define USB_MA_500     (500)
#define USB_MA_900     (900)
#define USB_MA_1000     (1000)
#define USB_MA_1100    (1100)
#define USB_MA_1200    (1200)
#define USB_MA_1400 (1400)
#define USB_MA_1450 (1450)
#define USB_MA_1500	(1500)
#define USB_MA_1600	(1600)
#define USB_MA_2000	(2000)
#endif 

enum qpnp_schg {
	QPNP_SCHG,
	QPNP_SCHG_LITE,
};

static char *version_str[] = {
	[QPNP_SCHG]		= "SCHG",
	[QPNP_SCHG_LITE]	= "SCHG_LITE",
};

enum pmic_subtype {
	PMI8994		= 10,
	PMI8950		= 17,
	PMI8996		= 19,
	PMI8937		= 55,
};

enum smbchg_wa {
	SMBCHG_AICL_DEGLITCH_WA = BIT(0),
	SMBCHG_HVDCP_9V_EN_WA	= BIT(1),
	SMBCHG_USB100_WA = BIT(2),
	SMBCHG_BATT_OV_WA = BIT(3),
	SMBCHG_CC_ESR_WA = BIT(4),
	SMBCHG_FLASH_ICL_DISABLE_WA = BIT(5),
        SMBCHG_RESTART_WA = BIT(6),
        SMBCHG_FLASH_BUCK_SWITCH_FREQ_WA = BIT(7),
};

#ifdef CONFIG_HTC_BATT
enum htc_smbchg_wa {
	HTC_DISABLE_QC2_QC3_WA = BIT(0),
	HTC_FORCE_FSW_1MHZ_WA = BIT(1),
};
#endif

enum print_reason {
	PR_REGISTER	= BIT(0),
	PR_INTERRUPT	= BIT(1),
	PR_STATUS	= BIT(2),
	PR_DUMP		= BIT(3),
	PR_PM		= BIT(4),
	PR_MISC		= BIT(5),
	PR_WIPOWER	= BIT(6),
	PR_TYPEC	= BIT(7),
};

enum wake_reason {
	PM_PARALLEL_CHECK = BIT(0),
	PM_REASON_VFLOAT_ADJUST = BIT(1),
	PM_ESR_PULSE = BIT(2),
	PM_PARALLEL_TAPER = BIT(3),
};

enum fcc_voters {
	ESR_PULSE_FCC_VOTER,
	BATT_TYPE_FCC_VOTER,
	RESTRICTED_CHG_FCC_VOTER,
	NUM_FCC_VOTER,
};

enum icl_voters {
	PSY_ICL_VOTER,
	THERMAL_ICL_VOTER,
	HVDCP_ICL_VOTER,
	USER_ICL_VOTER,
	WEAK_CHARGER_ICL_VOTER,
	SW_AICL_ICL_VOTER,
	CHG_SUSPEND_WORKAROUND_ICL_VOTER,
#ifdef CONFIG_HTC_CHARGER
	HTCCHG_ICL_VOTER,
#endif 
	NUM_ICL_VOTER,
};

enum enable_voters {
	
	USER_EN_VOTER,
	POWER_SUPPLY_EN_VOTER,
	USB_EN_VOTER,
	WIRELESS_EN_VOTER,
	THERMAL_EN_VOTER,
	OTG_EN_VOTER,
	WEAK_CHARGER_EN_VOTER,
	FAKE_BATTERY_EN_VOTER,
	NUM_EN_VOTERS,
};

enum battchg_enable_voters {
	
	BATTCHG_USER_EN_VOTER,
	
	BATTCHG_UNKNOWN_BATTERY_EN_VOTER,
	NUM_BATTCHG_EN_VOTERS,
};

enum hw_aicl_rerun_enable_indirect_voters {
	
	DEFAULT_CONFIG_HW_AICL_VOTER,
	
	VARB_WORKAROUND_VOTER,
	
	SHUTDOWN_WORKAROUND_VOTER,
	NUM_HW_AICL_RERUN_ENABLE_INDIRECT_VOTERS,
};

enum hw_aicl_rerun_disable_voters {
	
	HW_AICL_RERUN_ENABLE_INDIRECT_VOTER,
	
	WEAK_CHARGER_HW_AICL_VOTER,
	NUM_HW_AICL_DISABLE_VOTERS,
};

enum aicl_short_deglitch_voters {
	
	VARB_WORKAROUND_SHORT_DEGLITCH_VOTER,
	
	HVDCP_SHORT_DEGLITCH_VOTER,
	NUM_HW_SHORT_DEGLITCH_VOTERS,
};
static int smbchg_debug_mask;
module_param_named(
	debug_mask, smbchg_debug_mask, int, S_IRUSR | S_IWUSR
);

static int smbchg_parallel_en = 1;
module_param_named(
	parallel_en, smbchg_parallel_en, int, S_IRUSR | S_IWUSR
);

static int smbchg_main_chg_fcc_percent = 50;
module_param_named(
	main_chg_fcc_percent, smbchg_main_chg_fcc_percent,
	int, S_IRUSR | S_IWUSR
);

static int smbchg_main_chg_icl_percent = 60;
module_param_named(
	main_chg_icl_percent, smbchg_main_chg_icl_percent,
	int, S_IRUSR | S_IWUSR
);

#ifdef CONFIG_HTC_BATT
static int smbchg_default_hvdcp_icl_ma = 1600;
#else
static int smbchg_default_hvdcp_icl_ma = 1800;
#endif 
module_param_named(
	default_hvdcp_icl_ma, smbchg_default_hvdcp_icl_ma,
	int, S_IRUSR | S_IWUSR
);

#ifdef CONFIG_HTC_BATT
static int smbchg_default_hvdcp3_icl_ma = 2500;
#else
static int smbchg_default_hvdcp3_icl_ma = 3000;
#endif 
module_param_named(
	default_hvdcp3_icl_ma, smbchg_default_hvdcp3_icl_ma,
	int, S_IRUSR | S_IWUSR
);

#ifdef CONFIG_HTC_BATT
#ifdef CONFIG_HTC_BATT_WA_PCN0021
static int smbchg_default_dcp_icl_ma = 1000;
#else
static int smbchg_default_dcp_icl_ma = 1500;
#endif 
#else
static int smbchg_default_dcp_icl_ma = 1800;
#endif 
module_param_named(
	default_dcp_icl_ma, smbchg_default_dcp_icl_ma,
	int, S_IRUSR | S_IWUSR
);

static int wipower_dyn_icl_en;
module_param_named(
	dynamic_icl_wipower_en, wipower_dyn_icl_en,
	int, S_IRUSR | S_IWUSR
);

static int wipower_dcin_interval = ADC_MEAS1_INTERVAL_2P0MS;
module_param_named(
	wipower_dcin_interval, wipower_dcin_interval,
	int, S_IRUSR | S_IWUSR
);

#define WIPOWER_DEFAULT_HYSTERISIS_UV	250000
static int wipower_dcin_hyst_uv = WIPOWER_DEFAULT_HYSTERISIS_UV;
module_param_named(
	wipower_dcin_hyst_uv, wipower_dcin_hyst_uv,
	int, S_IRUSR | S_IWUSR
);

#define pr_smb(reason, fmt, ...)				\
	do {							\
		if (smbchg_debug_mask & (reason))		\
			pr_info(fmt, ##__VA_ARGS__);		\
		else						\
			pr_debug(fmt, ##__VA_ARGS__);		\
	} while (0)

#define pr_smb_rt(reason, fmt, ...)					\
	do {								\
		if (smbchg_debug_mask & (reason))			\
			pr_info_ratelimited(fmt, ##__VA_ARGS__);	\
		else							\
			pr_debug_ratelimited(fmt, ##__VA_ARGS__);	\
	} while (0)

#ifdef CONFIG_HTC_BATT
static inline int ABS(int x) { return x >= 0 ? x : -x; }
static void dump_regs(struct smbchg_chip *chip);
int pmi8994_get_usbin_voltage_now(void);
static void restore_from_hvdcp_detection(struct smbchg_chip *chip);


#define AICL_5V_2A_DETECT_DELAY_MS	1000
#define AICL_DOWNGRADE_IUSB_DELAY_MS	1000


static bool g_is_limit_IUSB = false;
static bool g_is_hvdcp_detect_done = false;
static bool g_is_charger_ability_detected = false;
static int g_count_same_dischg = 0;
static bool g_is_5v_2a_detected = false;
static bool g_rerun_apsd_ignore_uv = false;
#endif 

#ifdef CONFIG_HTC_CHARGER
static const struct qpnp_vadc_map_pt usb_adcmap_btm_threshold[] = {
        {-200, 1668},
        {-190, 1659},
        {-180, 1651},
        {-170, 1641},
        {-160, 1632},
        {-150, 1622},
        {-140, 1611},
        {-130, 1600},
        {-120, 1589},
        {-110, 1577},
        {-100, 1565},
        {-90, 1552},
        {-80, 1539},
        {-70, 1525},
        {-60, 1511},
        {-50, 1496},
        {-40, 1481},
        {-30, 1466},
        {-20, 1449},
        {-10, 1433},
        {0, 1416},
        {10, 1398},
        {20, 1381},
        {30, 1362},
        {40, 1344},
        {50, 1325},
        {60, 1305},
        {70, 1286},
        {80, 1266},
        {90, 1245},
        {100, 1225},
        {110, 1204},
        {120, 1183},
        {130, 1161},
        {140, 1140},
        {150, 1118},
        {160, 1096},
        {170, 1075},
        {180, 1053},
        {190, 1031},
        {200, 1009},
        {210, 987},
        {220, 965},
        {230, 943},
        {240, 922},
        {250, 900},
        {260, 879},
        {270, 857},
        {280, 836},
        {290, 815},
        {300, 795},
        {310, 774},
        {320, 754},
        {330, 734},
        {340, 715},
        {350, 695},
        {360, 677},
        {370, 658},
        {380, 640},
        {390, 622},
        {400, 604},
        {410, 587},
        {420, 570},
        {430, 554},
        {440, 537},
        {450, 522},
        {460, 506},
        {470, 491},
        {480, 477},
        {490, 462},
        {500, 449},
        {510, 435},
        {520, 422},
        {530, 409},
        {540, 397},
        {550, 385},
        {560, 373},
        {570, 361},
        {580, 350},
        {590, 339},
        {600, 329},
        {610, 319},
        {620, 309},
        {630, 300},
        {640, 290},
        {650, 281},
        {660, 273},
        {670, 264},
        {680, 256},
        {690, 248},
        {700, 241},
        {710, 233},
        {720, 226},
        {730, 219},
        {740, 212},
        {750, 206},
        {760, 200},
        {770, 193},
        {780, 188},
        {790, 182},
        {800, 176},
        {810, 171},
        {820, 166},
        {830, 161},
        {840, 156},
        {850, 151},
        {860, 147},
        {870, 142},
        {880, 138},
        {890, 134},
        {900, 130},
        {910, 126},
        {920, 123},
        {930, 119},
        {940, 116},
        {950, 112},
        {960, 109},
        {970, 106},
        {980, 103},
        {990, 100},
        {1000, 97}
};
#endif 

#ifdef CONFIG_HTC_CHARGER
static int pmi8994_set_htc_chg_charging_enabled(int value);
static int pmi8994_set_htc_chg_flash_active(int value);
#endif 

static int smbchg_read(struct smbchg_chip *chip, u8 *val,
			u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "spmi read failed addr=0x%02x sid=0x%02x rc=%d\n",
				addr, spmi->sid, rc);
		return rc;
	}
	return 0;
}

static int smbchg_write(struct smbchg_chip *chip, u8 *val,
			u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "write failed addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return rc;
	}

	return 0;
}

static int smbchg_masked_write_raw(struct smbchg_chip *chip, u16 base, u8 mask,
									u8 val)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi read failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	reg &= ~mask;
	reg |= val & mask;

	pr_smb(PR_REGISTER, "addr = 0x%x writing 0x%x\n", base, reg);

	rc = smbchg_write(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi write failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	return 0;
}

static int smbchg_masked_write(struct smbchg_chip *chip, u16 base, u8 mask,
								u8 val)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&chip->sec_access_lock, flags);
	rc = smbchg_masked_write_raw(chip, base, mask, val);
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);

	return rc;
}

#define SEC_ACCESS_OFFSET	0xD0
#define SEC_ACCESS_VALUE	0xA5
#define PERIPHERAL_MASK		0xFF
static int smbchg_sec_masked_write(struct smbchg_chip *chip, u16 base, u8 mask,
									u8 val)
{
	unsigned long flags;
	int rc;
	u16 peripheral_base = base & (~PERIPHERAL_MASK);

	spin_lock_irqsave(&chip->sec_access_lock, flags);

	rc = smbchg_masked_write_raw(chip, peripheral_base + SEC_ACCESS_OFFSET,
				SEC_ACCESS_VALUE, SEC_ACCESS_VALUE);
	if (rc) {
		dev_err(chip->dev, "Unable to unlock sec_access: %d", rc);
		goto out;
	}

	rc = smbchg_masked_write_raw(chip, base, mask, val);

out:
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);
	return rc;
}

static void smbchg_stay_awake(struct smbchg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons | reason;
	if (reasons != 0 && chip->wake_reasons == 0) {
		pr_smb(PR_PM, "staying awake: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_stay_awake(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
}

static void smbchg_relax(struct smbchg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons & (~reason);
	if (reasons == 0 && chip->wake_reasons != 0) {
		pr_smb(PR_PM, "relaxing: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_relax(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
};

enum pwr_path_type {
	UNKNOWN = 0,
	PWR_PATH_BATTERY = 1,
	PWR_PATH_USB = 2,
	PWR_PATH_DC = 3,
};

#define PWR_PATH		0x08
#define PWR_PATH_MASK		0x03
static enum pwr_path_type smbchg_get_pwr_path(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + PWR_PATH, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read PWR_PATH rc = %d\n", rc);
		return PWR_PATH_BATTERY;
	}

	return reg & PWR_PATH_MASK;
}

#define RID_STS				0xB
#define RID_MASK			0xF
#define IDEV_STS			0x8
#define RT_STS				0x10
#define USBID_MSB			0xE
#define USBIN_UV_BIT			BIT(0)
#define USBIN_OV_BIT			BIT(1)
#define USBIN_SRC_DET_BIT		BIT(2)
#define FMB_STS_MASK			SMB_MASK(3, 0)
#define USBID_GND_THRESHOLD		0x495
static bool is_otg_present_schg(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;
	u8 usbid_reg[2];
	u16 usbid_val;

	msleep(20);

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read IDEV_STS rc = %d\n", rc);
		return false;
	}

	if ((reg & FMB_STS_MASK) != 0) {
		pr_smb(PR_STATUS, "IDEV_STS = %02x, not ground\n", reg);
		return false;
	}

	rc = smbchg_read(chip, usbid_reg, chip->usb_chgpth_base + USBID_MSB, 2);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read USBID rc = %d\n", rc);
		return false;
	}
	usbid_val = (usbid_reg[0] << 8) | usbid_reg[1];

	if (usbid_val > USBID_GND_THRESHOLD) {
		pr_smb(PR_STATUS, "USBID = 0x%04x, too high to be ground\n",
				usbid_val);
		return false;
	}

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RID_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read usb rid status rc = %d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "RID_STS = %02x\n", reg);

	return (reg & RID_MASK) == 0;
}

#define RID_GND_DET_STS			BIT(2)
static bool is_otg_present_schg_lite(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->otg_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read otg RT status rc = %d\n", rc);
		return false;
	}

	return !!(reg & RID_GND_DET_STS);
}

static bool is_otg_present(struct smbchg_chip *chip)
{
	if (chip->schg_version == QPNP_SCHG_LITE)
		return is_otg_present_schg_lite(chip);

	return is_otg_present_schg(chip);
}

#define USBIN_9V			BIT(5)
#define USBIN_UNREG			BIT(4)
#define USBIN_LV			BIT(3)
#define DCIN_9V				BIT(2)
#define DCIN_UNREG			BIT(1)
#define DCIN_LV				BIT(0)
#define INPUT_STS			0x0D
#define DCIN_UV_BIT			BIT(0)
#define DCIN_OV_BIT			BIT(1)
static bool is_dc_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->dc_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read dc status rc = %d\n", rc);
		return false;
	}

	if ((reg & DCIN_UV_BIT) || (reg & DCIN_OV_BIT))
		return false;

	return true;
}

static bool is_usb_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	if (!(reg & USBIN_SRC_DET_BIT) || (reg & USBIN_OV_BIT))
		return false;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + INPUT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return false;
	}

	return !!(reg & (USBIN_9V | USBIN_UNREG | USBIN_LV));
}

static char *usb_type_str[] = {
	"SDP",		
	"OTHER",	
	"DCP",		
	"CDP",		
	"NONE",		
};

#define N_TYPE_BITS		4
#define TYPE_BITS_OFFSET	4

static int get_type(u8 type_reg)
{
	unsigned long type = type_reg;
	type >>= TYPE_BITS_OFFSET;
	return find_first_bit(&type, N_TYPE_BITS);
}

static inline char *get_usb_type_name(int type)
{
	return usb_type_str[type];
}

static enum power_supply_type usb_type_enum[] = {
	POWER_SUPPLY_TYPE_USB,		
	POWER_SUPPLY_TYPE_USB_DCP,	
	POWER_SUPPLY_TYPE_USB_DCP,	
	POWER_SUPPLY_TYPE_USB_CDP,	
	POWER_SUPPLY_TYPE_UNKNOWN,	
};

static inline enum power_supply_type get_usb_supply_type(int type)
{
	return usb_type_enum[type];
}

static void read_usb_type(struct smbchg_chip *chip, char **usb_type_name,
				enum power_supply_type *usb_supply_type)
{
	int rc, type;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		*usb_type_name = "Other";
		*usb_supply_type = POWER_SUPPLY_TYPE_UNKNOWN;
		return;
	}
	type = get_type(reg);
	*usb_type_name = get_usb_type_name(type);
	*usb_supply_type = get_usb_supply_type(type);
}

#define CHGR_STS			0x0E
#define BATT_LESS_THAN_2V		BIT(4)
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			SMB_MASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3
#define CHG_INHIBIT_BIT			BIT(1)
#define BAT_TCC_REACHED_BIT		BIT(7)
static int get_prop_batt_status(struct smbchg_chip *chip)
{
	int rc, status = POWER_SUPPLY_STATUS_DISCHARGING;
	u8 reg = 0, chg_type;
	bool charger_present, chg_inhibit;
#ifdef CONFIG_HTC_BATT
	int level = htc_battery_level_adjust();
#endif

#ifdef CONFIG_HTC_BATT
	charger_present = is_usb_present(chip) | is_dc_present(chip) |
			  chip->hvdcp_3_det_ignore_uv | g_rerun_apsd_ignore_uv;
#else
	charger_present = is_usb_present(chip) | is_dc_present(chip) |
			  chip->hvdcp_3_det_ignore_uv;
#endif
	if (!charger_present)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	rc = smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

#ifdef CONFIG_HTC_BATT
	if ((reg & BAT_TCC_REACHED_BIT) && (level == 100))
		return POWER_SUPPLY_STATUS_FULL;

	chg_inhibit = reg & CHG_INHIBIT_BIT;
	if ((chg_inhibit) && (level == 100))
		return POWER_SUPPLY_STATUS_FULL;
#else
	if (reg & BAT_TCC_REACHED_BIT)
		return POWER_SUPPLY_STATUS_FULL;

	chg_inhibit = reg & CHG_INHIBIT_BIT;
	if (chg_inhibit)
		return POWER_SUPPLY_STATUS_FULL;
#endif 

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & CHG_HOLD_OFF_BIT) {
#ifdef CONFIG_HTC_BATT
		if (level == 100)
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
#else
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
#endif 
		goto out;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;

#ifdef CONFIG_HTC_BATT
	if (chg_type == BATT_NOT_CHG_VAL && !(chip->hvdcp_3_det_ignore_uv || g_rerun_apsd_ignore_uv))
		if (chip->usb_supply_type > POWER_SUPPLY_TYPE_UNKNOWN) {
			if (level == 100)
				status = POWER_SUPPLY_STATUS_FULL;
			else{
#ifdef CONFIG_HTC_CHARGER
				if(htc_battery_get_discharging_reason())
					status = POWER_SUPPLY_STATUS_DISCHARGING;
				else
#endif 
					status = POWER_SUPPLY_STATUS_CHARGING;
			}
		} else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	else {
		if (level == 100)
			status = POWER_SUPPLY_STATUS_FULL;
		else{
#ifdef CONFIG_HTC_CHARGER
			if(htc_battery_get_discharging_reason())
				status = POWER_SUPPLY_STATUS_DISCHARGING;
                        else
#endif 
				status = POWER_SUPPLY_STATUS_CHARGING;
		}
	}
#else
	if (chg_type == BATT_NOT_CHG_VAL && !chip->hvdcp_3_det_ignore_uv)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;
#endif
out:
	pr_smb_rt(PR_MISC, "CHGR_STS = 0x%02x\n", reg);
	return status;
}

#define BAT_PRES_STATUS			0x08
#define BAT_PRES_BIT			BIT(7)
static int get_prop_batt_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->bat_if_base + BAT_PRES_STATUS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	return !!(reg & BAT_PRES_BIT);
}

static int get_prop_charge_type(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, chg_type;

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if (chg_type == BATT_TAPER_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (chg_type == BATT_FAST_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (chg_type == BATT_PRE_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int set_property_on_fg(struct smbchg_chip *chip,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = chip->bms_psy->set_property(chip->bms_psy, prop, &ret);
	if (rc)
		pr_smb(PR_STATUS,
			"bms psy does not allow updating prop %d rc = %d\n",
			prop, rc);

	return rc;
}

static int get_property_from_fg(struct smbchg_chip *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	rc = chip->bms_psy->get_property(chip->bms_psy, prop, &ret);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

#define DEFAULT_BATT_CAPACITY	50
static int get_prop_batt_capacity(struct smbchg_chip *chip)
{
	int capacity, rc;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CAPACITY, &capacity);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get capacity rc = %d\n", rc);
		capacity = DEFAULT_BATT_CAPACITY;
	}

#ifdef CONFIG_HTC_BATT
	htc_battery_info_update(POWER_SUPPLY_PROP_CAPACITY, capacity);
	capacity = htc_battery_level_adjust();
#endif

	return capacity;
}

#define DEFAULT_BATT_TEMP		200
static int get_prop_batt_temp(struct smbchg_chip *chip)
{
	int temp, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_TEMP, &temp);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get temperature rc = %d\n", rc);
		temp = DEFAULT_BATT_TEMP;
	}

#ifdef CONFIG_HTC_BATT
	if (get_kernel_flag() & KERNEL_FLAG_TEST_PWR_SUPPLY)
		temp = POWER_MONITOR_BATT_TEMP;
	if (temp > 650 && (get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON))
		temp = 650;
#endif

	return temp;
}

#define DEFAULT_BATT_CURRENT_NOW	0
static int get_prop_batt_current_now(struct smbchg_chip *chip)
{
	int ua, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CURRENT_NOW, &ua);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get current rc = %d\n", rc);
		ua = DEFAULT_BATT_CURRENT_NOW;
	}
	return ua;
}

#define DEFAULT_BATT_VOLTAGE_NOW	0
static int get_prop_batt_voltage_now(struct smbchg_chip *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_NOW;
	}
	return uv;
}

#define DEFAULT_BATT_VOLTAGE_MAX_DESIGN	4200000
static int get_prop_batt_voltage_max_design(struct smbchg_chip *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_MAX_DESIGN;
	}
	return uv;
}

#ifdef CONFIG_HTC_BATT
static int get_prop_batt_soc(struct smbchg_chip *chip)
{
	int batt_soc, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CAPACITY_RAW, &batt_soc);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get batt_soc rc = %d\n", rc);
	}

	return batt_soc;
}

static int get_prop_system_soc(struct smbchg_chip *chip)
{
	int system_soc, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_SYSTEM_SOC, &system_soc);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get system_soc rc = %d\n", rc);
	}

	return system_soc;
}

#endif 

static int get_prop_batt_health(struct smbchg_chip *chip)
{
	if (chip->batt_hot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		return POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		return POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		return POWER_SUPPLY_HEALTH_COOL;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

static void get_property_from_typec(struct smbchg_chip *chip,
				enum power_supply_property property,
				union power_supply_propval *prop)
{
	int rc;

	rc = chip->typec_psy->get_property(chip->typec_psy, property, prop);
	if (rc)
		pr_smb(PR_TYPEC,
			"typec psy doesn't support reading prop %d rc = %d\n",
			property, rc);
}

static void update_typec_status(struct smbchg_chip *chip)
{
	union power_supply_propval type = {0, };
	union power_supply_propval capability = {0, };
	int rc;

	get_property_from_typec(chip, POWER_SUPPLY_PROP_TYPE, &type);
	if (type.intval != POWER_SUPPLY_TYPE_UNKNOWN) {
		get_property_from_typec(chip,
				POWER_SUPPLY_PROP_CURRENT_CAPABILITY,
				&capability);
		chip->typec_current_ma = capability.intval;

		if (!chip->skip_usb_notification) {
			rc = chip->usb_psy->set_property(chip->usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
				&capability);
			if (rc)
				pr_err("typec failed to set current max rc=%d\n",
					rc);
			pr_smb(PR_TYPEC, "SMB Type-C mode = %d, current=%d\n",
					type.intval, capability.intval);
		}
	} else {
		pr_smb(PR_TYPEC,
			"typec detection not completed continuing with USB update\n");
	}
}

static int find_closest_in_array(const int *arr, int len, int val)
{
	int i, closest = 0;

	if (len == 0)
		return closest;
	for (i = 0; i < len; i++)
		if (abs(val - arr[i]) < abs(val - arr[closest]))
			closest = i;

	return closest;
}

static int find_smaller_in_array(const int *table, int val, int len)
{
	int i;

	for (i = len - 1; i >= 0; i--) {
		if (val >= table[i])
			break;
	}

	return i;
}

static const int iterm_ma_table_8994[] = {
	300,
	50,
	100,
	150,
	200,
	250,
	500,
	600
};

static const int iterm_ma_table_8996[] = {
	300,
	50,
	100,
	150,
	200,
	250,
	400,
	500
};

static const int usb_ilim_ma_table_8994[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
	2050,
	2100,
	2300,
	2400,
	2500,
	3000
};

static const int usb_ilim_ma_table_8996[] = {
	300,
	400,
	500,
	600,
	700,
	800,
	900,
	1000,
	1100,
	1200,
	1300,
	1400,
	1450,
	1500,
	1550,
	1600,
	1700,
	1800,
	1900,
	1950,
	2000,
	2050,
	2100,
	2200,
	2300,
	2400,
	2500,
	2600,
	2700,
	2800,
	2900,
	3000
};

static int dc_ilim_ma_table_8994[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
};

static int dc_ilim_ma_table_8996[] = {
	300,
	400,
	500,
	600,
	700,
	800,
	900,
	1000,
	1100,
	1200,
	1300,
	1400,
	1450,
	1500,
	1550,
	1600,
	1700,
	1800,
	1900,
	1950,
	2000,
	2050,
	2100,
	2200,
	2300,
	2400,
};

static const int fcc_comp_table_8994[] = {
	250,
	700,
	900,
	1200,
};

static const int fcc_comp_table_8996[] = {
	250,
	1100,
	1200,
	1500,
};

static const int aicl_rerun_period[] = {
	45,
	90,
	180,
	360,
};

static const int aicl_rerun_period_schg_lite[] = {
	3,	
	6,	
	11,	
	23,	
	45,
	90,
	180,
	360,
};

static void use_pmi8994_tables(struct smbchg_chip *chip)
{
	chip->tables.usb_ilim_ma_table = usb_ilim_ma_table_8994;
	chip->tables.usb_ilim_ma_len = ARRAY_SIZE(usb_ilim_ma_table_8994);
	chip->tables.dc_ilim_ma_table = dc_ilim_ma_table_8994;
	chip->tables.dc_ilim_ma_len = ARRAY_SIZE(dc_ilim_ma_table_8994);
	chip->tables.iterm_ma_table = iterm_ma_table_8994;
	chip->tables.iterm_ma_len = ARRAY_SIZE(iterm_ma_table_8994);
	chip->tables.fcc_comp_table = fcc_comp_table_8994;
	chip->tables.fcc_comp_len = ARRAY_SIZE(fcc_comp_table_8994);
	chip->tables.rchg_thr_mv = 200;
	chip->tables.aicl_rerun_period_table = aicl_rerun_period;
	chip->tables.aicl_rerun_period_len = ARRAY_SIZE(aicl_rerun_period);
}

static void use_pmi8996_tables(struct smbchg_chip *chip)
{
	chip->tables.usb_ilim_ma_table = usb_ilim_ma_table_8996;
	chip->tables.usb_ilim_ma_len = ARRAY_SIZE(usb_ilim_ma_table_8996);
	chip->tables.dc_ilim_ma_table = dc_ilim_ma_table_8996;
	chip->tables.dc_ilim_ma_len = ARRAY_SIZE(dc_ilim_ma_table_8996);
	chip->tables.iterm_ma_table = iterm_ma_table_8996;
	chip->tables.iterm_ma_len = ARRAY_SIZE(iterm_ma_table_8996);
	chip->tables.fcc_comp_table = fcc_comp_table_8996;
	chip->tables.fcc_comp_len = ARRAY_SIZE(fcc_comp_table_8996);
	chip->tables.rchg_thr_mv = 150;
	chip->tables.aicl_rerun_period_table = aicl_rerun_period;
	chip->tables.aicl_rerun_period_len = ARRAY_SIZE(aicl_rerun_period);
}

#define CMD_CHG_REG	0x42
#define EN_BAT_CHG_BIT		BIT(1)
static int smbchg_charging_en(struct smbchg_chip *chip, bool en)
{
	
	return smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			EN_BAT_CHG_BIT, en ? 0 : EN_BAT_CHG_BIT);
}

#define CMD_IL			0x40
#define USBIN_SUSPEND_BIT	BIT(4)
#define CURRENT_100_MA		100
#define CURRENT_150_MA		150
#define CURRENT_500_MA		500
#define CURRENT_900_MA		900
#define CURRENT_1500_MA		1500
#define SUSPEND_CURRENT_MA	2
#define ICL_OVERRIDE_BIT	BIT(2)
static int smbchg_usb_suspend(struct smbchg_chip *chip, bool suspend)
{
	int rc;

	if (!suspend)
		g_count_same_dischg = 0;

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USBIN_SUSPEND_BIT, suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set usb suspend rc = %d\n", rc);
	return rc;
}

#define DCIN_SUSPEND_BIT	BIT(3)
static int smbchg_dc_suspend(struct smbchg_chip *chip, bool suspend)
{
	int rc = 0;

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			DCIN_SUSPEND_BIT, suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set dc suspend rc = %d\n", rc);
	return rc;
}

#define IL_CFG			0xF2
#define DCIN_INPUT_MASK	SMB_MASK(4, 0)
static int smbchg_set_dc_current_max(struct smbchg_chip *chip, int current_ma)
{
	int i;
	u8 dc_cur_val;

	i = find_smaller_in_array(chip->tables.dc_ilim_ma_table,
			current_ma, chip->tables.dc_ilim_ma_len);

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %dma current_table\n",
				current_ma);
		return -EINVAL;
	}

	chip->dc_max_current_ma = chip->tables.dc_ilim_ma_table[i];
	dc_cur_val = i & DCIN_INPUT_MASK;

	pr_smb(PR_STATUS, "dc current set to %d mA\n",
			chip->dc_max_current_ma);
	return smbchg_sec_masked_write(chip, chip->dc_chgpth_base + IL_CFG,
				DCIN_INPUT_MASK, dc_cur_val);
}

#define AICL_WL_SEL_CFG			0xF5
#define AICL_WL_SEL_MASK		SMB_MASK(1, 0)
#define AICL_WL_SEL_SCHG_LITE_MASK	SMB_MASK(2, 0)
static int smbchg_set_aicl_rerun_period_s(struct smbchg_chip *chip,
								int period_s)
{
	int i;
	u8 reg, mask;

	i = find_smaller_in_array(chip->tables.aicl_rerun_period_table,
			period_s, chip->tables.aicl_rerun_period_len);

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %ds in aicl rerun period\n",
				period_s);
		return -EINVAL;
	}

	if (chip->schg_version == QPNP_SCHG_LITE)
		mask = AICL_WL_SEL_SCHG_LITE_MASK;
	else
		mask = AICL_WL_SEL_MASK;

	reg = i & mask;

	pr_smb(PR_STATUS, "aicl rerun period set to %ds\n",
			chip->tables.aicl_rerun_period_table[i]);
	return smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + AICL_WL_SEL_CFG,
			mask, reg);
}

static struct power_supply *get_parallel_psy(struct smbchg_chip *chip)
{
	if (!chip->parallel.avail)
		return NULL;
	if (chip->parallel.psy)
		return chip->parallel.psy;
	chip->parallel.psy = power_supply_get_by_name("usb-parallel");
	if (!chip->parallel.psy)
		pr_smb(PR_STATUS, "parallel charger not found\n");
	return chip->parallel.psy;
}

static void smbchg_usb_update_online_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				usb_set_online_work);
#ifndef CONFIG_HTC_BATT
	bool user_enabled = !get_client_vote(chip->usb_suspend_votable,
						USER_EN_VOTER);
#endif
	int online;

#ifdef CONFIG_HTC_BATT
	online = chip->usb_present;
#else
	online = user_enabled && chip->usb_present && !chip->very_weak_charger;
#endif 

	mutex_lock(&chip->usb_set_online_lock);
	if (chip->usb_online != online) {
		pr_smb(PR_MISC, "setting usb psy online = %d\n", online);
		power_supply_set_online(chip->usb_psy, online);
		chip->usb_online = online;
	}
	mutex_unlock(&chip->usb_set_online_lock);
}

#define CHGPTH_CFG		0xF4
#define CFG_USB_2_3_SEL_BIT	BIT(7)
#define CFG_USB_2		0
#define CFG_USB_3		BIT(7)
#define USBIN_INPUT_MASK	SMB_MASK(4, 0)
#define USBIN_MODE_CHG_BIT	BIT(0)
#define USBIN_LIMITED_MODE	0
#define USBIN_HC_MODE		BIT(0)
#define USB51_MODE_BIT		BIT(1)
#define USB51_100MA		0
#define USB51_500MA		BIT(1)
static int smbchg_set_high_usb_chg_current(struct smbchg_chip *chip,
							int current_ma)
{
	int i, rc;
	u8 usb_cur_val;

	if (current_ma == CURRENT_100_MA) {
		rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
		if (rc < 0) {
			pr_err("Couldn't set CFG_USB_2 rc=%d\n", rc);
			return rc;
		}

		rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USBIN_MODE_CHG_BIT | USB51_MODE_BIT | ICL_OVERRIDE_BIT,
			USBIN_LIMITED_MODE | USB51_100MA | ICL_OVERRIDE_BIT);
		if (rc < 0) {
			pr_err("Couldn't set ICL_OVERRIDE rc=%d\n", rc);
			return rc;
		}

		pr_smb(PR_STATUS,
			"Forcing 100mA current limit\n");
		chip->usb_max_current_ma = CURRENT_100_MA;
		return rc;
	}

	i = find_smaller_in_array(chip->tables.usb_ilim_ma_table,
			current_ma, chip->tables.usb_ilim_ma_len);
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_150_MA);

		rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
		rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_150_MA, rc);
		else
			chip->usb_max_current_ma = 150;
		return rc;
	}

	usb_cur_val = i & USBIN_INPUT_MASK;
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG,
				USBIN_INPUT_MASK, usb_cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to config c rc = %d\n", rc);
		return rc;
	}

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
				USBIN_MODE_CHG_BIT, USBIN_HC_MODE);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't write cfg 5 rc = %d\n", rc);
	chip->usb_max_current_ma = chip->tables.usb_ilim_ma_table[i];
	return rc;
}

static int smbchg_set_usb_current_max(struct smbchg_chip *chip,
							int current_ma)
{
	int rc = 0;
#ifdef CONFIG_HTC_BATT
	int pd_current;
#endif

	if (!chip->batt_present && current_ma < chip->usb_max_current_ma) {
		pr_info_ratelimited("Ignoring usb current->%d, battery is absent\n",
				current_ma);
		return 0;
	}

#ifdef CONFIG_HTC_BATT
	if(g_is_limit_IUSB && (current_ma >= USB_MA_1000) &&
		(chip->usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP) &&
		!htc_battery_get_pd_type(&pd_current)){
		pr_smb(PR_STATUS, "Charger is bad, force limit current %d -> 1000 mA\n",current_ma);
		current_ma = USB_MA_1000;
	}
#endif 

	pr_smb(PR_STATUS, "USB current_ma = %d\n", current_ma);

	if (current_ma <= SUSPEND_CURRENT_MA) {
		
		rc = vote(chip->usb_suspend_votable, USB_EN_VOTER, true, 0);
		chip->usb_max_current_ma = 0;
		goto out;
	} else {
		rc = vote(chip->usb_suspend_votable, USB_EN_VOTER, false, 0);
	}

	switch (chip->usb_supply_type) {
	case POWER_SUPPLY_TYPE_USB:
#ifdef CONFIG_HTC_BATT
		if(htc_battery_get_pd_type(&pd_current)){
			if (current_ma <= CURRENT_1500_MA) {
				
				rc = smbchg_masked_write(chip,
						chip->usb_chgpth_base + CMD_IL,
						ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
				if (rc < 0)
					pr_err("Couldn't set override rc = %d\n", rc);
			}

			rc = smbchg_set_high_usb_chg_current(chip, current_ma);
			if (rc < 0)
				pr_err("Couldn't set %dmA rc = %d\n", current_ma, rc);
			break;
		}
#endif 
		if ((current_ma < CURRENT_150_MA) &&
				(chip->wa_flags & SMBCHG_USB100_WA))
			current_ma = CURRENT_150_MA;

		if (current_ma < CURRENT_150_MA) {
			
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 100;
		}
		
		if (current_ma == CURRENT_150_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 150;
		}
		if (current_ma == CURRENT_500_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_500MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 500;
		}
		if (current_ma == CURRENT_900_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_500MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 900;
		}
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
#ifdef CONFIG_HTC_BATT
		if (current_ma <= CURRENT_1500_MA) {
#else
		if (current_ma < CURRENT_1500_MA) {
#endif 
			
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
			if (rc < 0)
				pr_err("Couldn't set override rc = %d\n", rc);
		}
		
	default:
		rc = smbchg_set_high_usb_chg_current(chip, current_ma);
		if (rc < 0)
			pr_err("Couldn't set %dmA rc = %d\n", current_ma, rc);
		break;
	}

out:
	pr_smb(PR_STATUS, "usb type = %d current set to %d mA\n",
			chip->usb_supply_type, chip->usb_max_current_ma);
	return rc;
}

#define USBIN_HVDCP_STS				0x0C
#define USBIN_HVDCP_SEL_BIT			BIT(4)
#define USBIN_HVDCP_SEL_9V_BIT			BIT(1)
#define SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT	BIT(2)
#define SCHG_LITE_USBIN_HVDCP_SEL_BIT		BIT(0)
static int smbchg_get_min_parallel_current_ma(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, hvdcp_sel, hvdcp_sel_9v;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return 0;
	}
	if (chip->schg_version == QPNP_SCHG_LITE) {
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT;
	} else {
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = USBIN_HVDCP_SEL_9V_BIT;
	}

	if ((reg & hvdcp_sel) && (reg & hvdcp_sel_9v))
		return chip->parallel.min_9v_current_thr_ma;
	return chip->parallel.min_current_thr_ma;
}

#ifdef CONFIG_HTC_BATT
#define HVDCP_DETECT_FAILED_RETRY_MS 30000
#endif
static bool is_hvdcp_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, hvdcp_sel;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		pr_err("Couldn't read hvdcp status rc = %d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "HVDCP_STS = 0x%02x\n", reg);
	if (chip->schg_version == QPNP_SCHG_LITE)
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
	else
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;

	if ((reg & hvdcp_sel) && is_usb_present(chip))
		return true;

#ifdef CONFIG_HTC_BATT
	if ((chip->usb_supply_type == POWER_SUPPLY_TYPE_USB_HVDCP)
		&& is_usb_present(chip)) {
		if (delayed_work_pending(&chip->rerun_apsd_work))
			cancel_delayed_work(&chip->rerun_apsd_work);
		schedule_delayed_work(&chip->rerun_apsd_work,
				msecs_to_jiffies(HVDCP_DETECT_FAILED_RETRY_MS));
		pr_smb(PR_STATUS, "Schedule RERUN APSD for hvdcp_present detect failed.\n");
	}
#endif

	return false;
}

#define FCC_CFG			0xF2
#define FCC_500MA_VAL		0x4
#define FCC_MASK		SMB_MASK(4, 0)
static int smbchg_set_fastchg_current_raw(struct smbchg_chip *chip,
							int current_ma)
{
	int i, rc;
	u8 cur_val;

	
	i = find_smaller_in_array(chip->tables.usb_ilim_ma_table,
			current_ma, chip->tables.usb_ilim_ma_len);
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_500_MA);

		rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
					FCC_MASK,
					FCC_500MA_VAL);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_500_MA, rc);
		else
			chip->fastchg_current_ma = 500;
		return rc;
	}

	if (chip->tables.usb_ilim_ma_table[i] == chip->fastchg_current_ma) {
		pr_smb(PR_STATUS, "skipping fastchg current request: %d\n",
				chip->fastchg_current_ma);
		return 0;
	}

	cur_val = i & FCC_MASK;
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
				FCC_MASK, cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to fcc cfg rc = %d\n", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "fastcharge current requested %d, set to %d\n",
			current_ma, chip->tables.usb_ilim_ma_table[cur_val]);

	chip->fastchg_current_ma = chip->tables.usb_ilim_ma_table[cur_val];
	return rc;
}

#define ICL_STS_1_REG			0x7
#define ICL_STS_2_REG			0x9
#define ICL_STS_MASK			0x1F
#define AICL_SUSP_BIT			BIT(6)
#define AICL_STS_BIT			BIT(5)
#define USBIN_SUSPEND_STS_BIT		BIT(3)
#define USBIN_ACTIVE_PWR_SRC_BIT	BIT(1)
#define DCIN_ACTIVE_PWR_SRC_BIT		BIT(0)
#define PARALLEL_REENABLE_TIMER_MS	1000
#define PARALLEL_CHG_THRESHOLD_CURRENT	1800
static bool smbchg_is_usbin_active_pwr_src(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_2_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Could not read usb icl sts 2: %d\n", rc);
		return false;
	}

	return !(reg & USBIN_SUSPEND_STS_BIT)
		&& (reg & USBIN_ACTIVE_PWR_SRC_BIT);
}

static int smbchg_parallel_usb_charging_en(struct smbchg_chip *chip, bool en)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };

	if (!parallel_psy || !chip->parallel_charger_detected)
		return 0;

	pval.intval = en;
	return parallel_psy->set_property(parallel_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
}

#define ESR_PULSE_CURRENT_DELTA_MA	200
static int smbchg_sw_esr_pulse_en(struct smbchg_chip *chip, bool en)
{
	int rc, fg_current_now, icl_ma;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CURRENT_NOW,
						&fg_current_now);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support OCV\n");
		return 0;
	}

	icl_ma = max(chip->iterm_ma + ESR_PULSE_CURRENT_DELTA_MA,
				fg_current_now - ESR_PULSE_CURRENT_DELTA_MA);
	rc = vote(chip->fcc_votable, ESR_PULSE_FCC_VOTER, en, icl_ma);
	if (rc < 0) {
		pr_err("Couldn't Vote FCC en = %d rc = %d\n", en, rc);
		return rc;
	}
	rc = smbchg_parallel_usb_charging_en(chip, !en);
	return rc;
}

#define USB_AICL_CFG				0xF3
#define AICL_EN_BIT				BIT(2)
static void smbchg_rerun_aicl(struct smbchg_chip *chip)
{
#ifdef CONFIG_HTC_BATT
	int pd_current;
	if (htc_battery_get_pd_type(&pd_current)){
		set_aicl_enable(false);
		pr_info("PD charger donot rerun AICL.\n");
		return;
	}
#endif 
	pr_smb(PR_STATUS, "Rerunning AICL...\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	
	msleep(50);
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
}

static void taper_irq_en(struct smbchg_chip *chip, bool en)
{
	mutex_lock(&chip->taper_irq_lock);
	if (en != chip->taper_irq_enabled) {
		if (en) {
			enable_irq(chip->taper_irq);
			enable_irq_wake(chip->taper_irq);
		} else {
			disable_irq_wake(chip->taper_irq);
			disable_irq_nosync(chip->taper_irq);
		}
		chip->taper_irq_enabled = en;
	}
	mutex_unlock(&chip->taper_irq_lock);
}

static int smbchg_get_aicl_level_ma(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Could not read usb icl sts 1: %d\n", rc);
		return 0;
	}
	if (reg & AICL_SUSP_BIT) {
		pr_warn("AICL suspended: %02x\n", reg);
		return 0;
	}
	reg &= ICL_STS_MASK;
	if (reg >= chip->tables.usb_ilim_ma_len) {
		pr_warn("invalid AICL value: %02x\n", reg);
		return 0;
	}
	return chip->tables.usb_ilim_ma_table[reg];
}

static void smbchg_parallel_usb_disable(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;
	pr_smb(PR_STATUS, "disabling parallel charger\n");
	chip->parallel.last_disabled = ktime_get_boottime();
	taper_irq_en(chip, false);
	chip->parallel.initial_aicl_ma = 0;
	chip->parallel.current_max_ma = 0;
	power_supply_set_current_limit(parallel_psy,
				SUSPEND_CURRENT_MA * 1000);
	power_supply_set_present(parallel_psy, false);
	smbchg_set_fastchg_current_raw(chip,
			get_effective_result_locked(chip->fcc_votable));
	smbchg_set_usb_current_max(chip,
			get_effective_result_locked(chip->usb_icl_votable));
	smbchg_rerun_aicl(chip);
}

#define PARALLEL_TAPER_MAX_TRIES		3
#define PARALLEL_FCC_PERCENT_REDUCTION		75
#define MINIMUM_PARALLEL_FCC_MA			500
#define CHG_ERROR_BIT		BIT(0)
#define BAT_TAPER_MODE_BIT	BIT(6)
static void smbchg_parallel_usb_taper(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int parallel_fcc_ma, tries = 0;
	u8 reg = 0;

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;

	smbchg_stay_awake(chip, PM_PARALLEL_TAPER);
try_again:
	mutex_lock(&chip->parallel.lock);
	if (chip->parallel.current_max_ma == 0) {
		pr_smb(PR_STATUS, "Not parallel charging, skipping\n");
		goto done;
	}
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	tries += 1;
	parallel_fcc_ma = pval.intval / 1000;
	pr_smb(PR_STATUS, "try #%d parallel charger fcc = %d\n",
			tries, parallel_fcc_ma);
	if (parallel_fcc_ma < MINIMUM_PARALLEL_FCC_MA
				|| tries > PARALLEL_TAPER_MAX_TRIES) {
		smbchg_parallel_usb_disable(chip);
		goto done;
	}
	pval.intval = ((parallel_fcc_ma
			* PARALLEL_FCC_PERCENT_REDUCTION) / 100);
	pr_smb(PR_STATUS, "reducing FCC of parallel charger to %d\n",
		pval.intval);
	
	pval.intval *= 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	mutex_unlock(&chip->parallel.lock);
	msleep(100);

	mutex_lock(&chip->parallel.lock);
	if (chip->parallel.current_max_ma == 0) {
		pr_smb(PR_STATUS, "Not parallel charging, skipping\n");
		goto done;
	}
	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (reg & BAT_TAPER_MODE_BIT) {
		mutex_unlock(&chip->parallel.lock);
		goto try_again;
	}
	taper_irq_en(chip, true);
done:
	mutex_unlock(&chip->parallel.lock);
	smbchg_relax(chip, PM_PARALLEL_TAPER);
}

static void smbchg_parallel_usb_enable(struct smbchg_chip *chip,
		int total_current_ma)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int new_parallel_cl_ma, set_parallel_cl_ma, new_pmi_cl_ma, rc;
	int current_table_index, target_icl_ma;
	int fcc_ma, main_fastchg_current_ma;
	int target_parallel_fcc_ma, supplied_parallel_fcc_ma;
	int parallel_chg_fcc_percent;

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;

	pr_smb(PR_STATUS, "Attempting to enable parallel charger\n");

	rc = power_supply_set_voltage_limit(parallel_psy, chip->vfloat_mv + 50);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set Vflt on parallel psy rc: %d\n", rc);
		return;
	}
	
	target_icl_ma = get_effective_result_locked(chip->usb_icl_votable);
	new_parallel_cl_ma = total_current_ma
			* (100 - smbchg_main_chg_icl_percent) / 100;
	taper_irq_en(chip, true);
	power_supply_set_present(parallel_psy, true);
	power_supply_set_current_limit(parallel_psy,
				new_parallel_cl_ma * 1000);
	
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
	set_parallel_cl_ma = pval.intval / 1000;
	chip->parallel.current_max_ma = new_parallel_cl_ma;
	pr_smb(PR_MISC, "Requested ICL = %d from parallel, got %d\n",
		new_parallel_cl_ma, set_parallel_cl_ma);
	new_pmi_cl_ma = max(0, target_icl_ma - set_parallel_cl_ma);
	pr_smb(PR_STATUS, "New Total USB current = %d[%d, %d]\n",
		total_current_ma, new_pmi_cl_ma,
		set_parallel_cl_ma);
	smbchg_set_usb_current_max(chip, new_pmi_cl_ma);

	
	fcc_ma = get_effective_result_locked(chip->fcc_votable);
	parallel_chg_fcc_percent =
		100 - smbchg_main_chg_fcc_percent;
	target_parallel_fcc_ma =
		(fcc_ma * parallel_chg_fcc_percent) / 100;
	pval.intval = target_parallel_fcc_ma * 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	supplied_parallel_fcc_ma = pval.intval / 1000;
	pr_smb(PR_MISC, "Requested FCC = %d from parallel, got %d\n",
		target_parallel_fcc_ma, supplied_parallel_fcc_ma);

	
	current_table_index = find_smaller_in_array(
			chip->tables.usb_ilim_ma_table,
			fcc_ma - supplied_parallel_fcc_ma,
			chip->tables.usb_ilim_ma_len);
	main_fastchg_current_ma =
		chip->tables.usb_ilim_ma_table[current_table_index];
	smbchg_set_fastchg_current_raw(chip, main_fastchg_current_ma);
	pr_smb(PR_STATUS, "FCC = %d[%d, %d]\n", fcc_ma, main_fastchg_current_ma,
					supplied_parallel_fcc_ma);

	chip->parallel.enabled_once = true;

	return;
}

static bool smbchg_is_parallel_usb_ok(struct smbchg_chip *chip,
		int *ret_total_current_ma)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int min_current_thr_ma, rc, type;
	int total_current_ma, current_limit_ma, parallel_cl_ma;
	ktime_t kt_since_last_disable;
	u8 reg;
	int fcc_ma = get_effective_result_locked(chip->fcc_votable);
	int fcc_voter_id = get_effective_client_id_locked(chip->fcc_votable);
	int usb_icl_ma = get_effective_result_locked(chip->usb_icl_votable);

	if (!parallel_psy || !smbchg_parallel_en
			|| !chip->parallel_charger_detected) {
		pr_smb(PR_STATUS, "Parallel charging not enabled\n");
		return false;
	}

	kt_since_last_disable = ktime_sub(ktime_get_boottime(),
					chip->parallel.last_disabled);
	if (chip->parallel.current_max_ma == 0
		&& chip->parallel.enabled_once
		&& ktime_to_ms(kt_since_last_disable)
			< PARALLEL_REENABLE_TIMER_MS) {
		pr_smb(PR_STATUS, "Only been %lld since disable, skipping\n",
				ktime_to_ms(kt_since_last_disable));
		return false;
	}

	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST
			&& (get_prop_batt_present(chip)
				|| chip->parallel.current_max_ma == 0)) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return false;
	}

	if (get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_smb(PR_STATUS, "JEITA active, skipping\n");
		return false;
	}

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		return false;
	}

	type = get_type(reg);
	if (get_usb_supply_type(type) == POWER_SUPPLY_TYPE_USB_CDP) {
		pr_smb(PR_STATUS, "CDP adapter, skipping\n");
		return false;
	}

	if (get_usb_supply_type(type) == POWER_SUPPLY_TYPE_USB) {
		pr_smb(PR_STATUS, "SDP adapter, skipping\n");
		return false;
	}

	if (!smbchg_is_usbin_active_pwr_src(chip)) {
		pr_smb(PR_STATUS, "USB not active power source: %02x\n", reg);
		return false;
	}

	min_current_thr_ma = smbchg_get_min_parallel_current_ma(chip);
	if (min_current_thr_ma <= 0) {
		pr_smb(PR_STATUS, "parallel charger unavailable for thr: %d\n",
				min_current_thr_ma);
		return false;
	}

	if (usb_icl_ma < min_current_thr_ma) {
		pr_smb(PR_STATUS, "Weak USB chg skip enable: %d < %d\n",
			usb_icl_ma, min_current_thr_ma);
		return false;
	}

	if (fcc_voter_id != ESR_PULSE_FCC_VOTER
			&& fcc_ma < PARALLEL_CHG_THRESHOLD_CURRENT) {
		pr_smb(PR_STATUS, "FCC %d lower than %d\n",
			fcc_ma,
			PARALLEL_CHG_THRESHOLD_CURRENT);
		return false;
	}

	current_limit_ma = smbchg_get_aicl_level_ma(chip);
	if (current_limit_ma <= 0)
		return false;

	if (chip->parallel.initial_aicl_ma == 0) {
		if (current_limit_ma < min_current_thr_ma) {
			pr_smb(PR_STATUS, "Initial AICL very low: %d < %d\n",
				current_limit_ma, min_current_thr_ma);
			return false;
		}
		chip->parallel.initial_aicl_ma = current_limit_ma;
	}

	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
	parallel_cl_ma = pval.intval / 1000;
	if (parallel_cl_ma <= SUSPEND_CURRENT_MA)
		parallel_cl_ma = 0;

	total_current_ma = min(current_limit_ma + parallel_cl_ma, usb_icl_ma);

	if (total_current_ma < chip->parallel.initial_aicl_ma
			- chip->parallel.allowed_lowering_ma) {
		pr_smb(PR_STATUS,
			"Total current reduced a lot: %d (%d + %d) < %d - %d\n",
			total_current_ma,
			current_limit_ma, parallel_cl_ma,
			chip->parallel.initial_aicl_ma,
			chip->parallel.allowed_lowering_ma);
		return false;
	}

	*ret_total_current_ma = total_current_ma;
	return true;
}

#define PARALLEL_CHARGER_EN_DELAY_MS	500
static void smbchg_parallel_usb_en_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				parallel_en_work.work);
	int previous_aicl_ma, total_current_ma, aicl_ma;
	bool in_progress;

	
	previous_aicl_ma = smbchg_get_aicl_level_ma(chip);
	msleep(PARALLEL_CHARGER_EN_DELAY_MS);
	aicl_ma = smbchg_get_aicl_level_ma(chip);
	if (previous_aicl_ma == aicl_ma) {
		pr_smb(PR_STATUS, "AICL at %d\n", aicl_ma);
	} else {
		pr_smb(PR_STATUS,
			"AICL changed [%d -> %d], recheck %d ms\n",
			previous_aicl_ma, aicl_ma,
			PARALLEL_CHARGER_EN_DELAY_MS);
		goto recheck;
	}

	mutex_lock(&chip->parallel.lock);
	in_progress = (chip->parallel.current_max_ma != 0);
	if (smbchg_is_parallel_usb_ok(chip, &total_current_ma)) {
		smbchg_parallel_usb_enable(chip, total_current_ma);
	} else {
		if (in_progress) {
			pr_smb(PR_STATUS, "parallel charging unavailable\n");
			smbchg_parallel_usb_disable(chip);
		}
	}
	mutex_unlock(&chip->parallel.lock);
	smbchg_relax(chip, PM_PARALLEL_CHECK);
	return;

recheck:
	schedule_delayed_work(&chip->parallel_en_work, 0);
}

static void smbchg_parallel_usb_check_ok(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;

	smbchg_stay_awake(chip, PM_PARALLEL_CHECK);
	schedule_delayed_work(&chip->parallel_en_work, 0);
}

static int charging_suspend_vote_cb(struct device *dev, int suspend,
						int client, int last_suspend,
						int last_client)
{
	int rc;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_charging_en(chip, !suspend);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't configure batt chg: 0x%x rc = %d\n",
			!suspend, rc);
	}

	return rc;
}

static int usb_suspend_vote_cb(struct device *dev, int suspend,
						int client, int last_suspend,
						int last_client)
{
	int rc;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_usb_suspend(chip, suspend);
	if (rc < 0)
		return rc;

	if (client == THERMAL_EN_VOTER || client == POWER_SUPPLY_EN_VOTER ||
				client == USER_EN_VOTER ||
				client == FAKE_BATTERY_EN_VOTER)
		smbchg_parallel_usb_check_ok(chip);

	return rc;
}

static int dc_suspend_vote_cb(struct device *dev, int suspend,
						int client, int last_suspend,
						int last_client)
{
	int rc;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_dc_suspend(chip, suspend);
	if (rc < 0)
		return rc;

	if (chip->dc_psy_type != -EINVAL && chip->psy_registered)
		power_supply_changed(&chip->dc_psy);

	return rc;
}

static int set_fastchg_current_vote_cb(struct device *dev,
						int fcc_ma,
						int client,
						int last_fcc_ma,
						int last_client)
{
	struct smbchg_chip *chip = dev_get_drvdata(dev);
	int rc;

	if (chip->parallel.current_max_ma == 0) {
		rc = smbchg_set_fastchg_current_raw(chip, fcc_ma);
		if (rc < 0) {
			pr_err("Can't set FCC fcc_ma=%d rc=%d\n", fcc_ma, rc);
			return rc;
		}
	}
	smbchg_parallel_usb_check_ok(chip);
	return 0;
}

static int smbchg_set_fastchg_current_user(struct smbchg_chip *chip,
							int current_ma)
{
	int rc = 0;

	pr_smb(PR_STATUS, "User setting FCC to %d\n", current_ma);

	rc = vote(chip->fcc_votable, BATT_TYPE_FCC_VOTER, true, current_ma);
	if (rc < 0)
		pr_err("Couldn't vote en rc %d\n", rc);
	return rc;
}

static struct ilim_entry *smbchg_wipower_find_entry(struct smbchg_chip *chip,
				struct ilim_map *map, int uv)
{
	int i;
	struct ilim_entry *ret = &(chip->wipower_default.entries[0]);

	for (i = 0; i < map->num; i++) {
		if (is_between(map->entries[i].vmin_uv, map->entries[i].vmax_uv,
			uv))
			ret = &map->entries[i];
	}
	return ret;
}

#define ZIN_ICL_PT	0xFC
#define ZIN_ICL_LV	0xFD
#define ZIN_ICL_HV	0xFE
#define ZIN_ICL_MASK	SMB_MASK(4, 0)
static int smbchg_dcin_ilim_config(struct smbchg_chip *chip, int offset, int ma)
{
	int i, rc;

	i = find_smaller_in_array(chip->tables.dc_ilim_ma_table,
			ma, chip->tables.dc_ilim_ma_len);

	if (i < 0)
		i = 0;

	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + offset,
			ZIN_ICL_MASK, i);
	if (rc)
		dev_err(chip->dev, "Couldn't write bat if offset %d value = %d rc = %d\n",
				offset, i, rc);
	return rc;
}

static int smbchg_wipower_ilim_config(struct smbchg_chip *chip,
						struct ilim_entry *ilim)
{
	int rc = 0;

	if (chip->current_ilim.icl_pt_ma != ilim->icl_pt_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_PT, ilim->icl_pt_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_PT, ilim->icl_pt_ma, rc);
		else
			chip->current_ilim.icl_pt_ma =  ilim->icl_pt_ma;
	}

	if (chip->current_ilim.icl_lv_ma !=  ilim->icl_lv_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_LV, ilim->icl_lv_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_LV, ilim->icl_lv_ma, rc);
		else
			chip->current_ilim.icl_lv_ma =  ilim->icl_lv_ma;
	}

	if (chip->current_ilim.icl_hv_ma !=  ilim->icl_hv_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_HV, ilim->icl_hv_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_HV, ilim->icl_hv_ma, rc);
		else
			chip->current_ilim.icl_hv_ma =  ilim->icl_hv_ma;
	}
	return rc;
}

static void btm_notify_dcin(enum qpnp_tm_state state, void *ctx);
static int smbchg_wipower_dcin_btm_configure(struct smbchg_chip *chip,
		struct ilim_entry *ilim)
{
	int rc;

	if (ilim->vmin_uv == chip->current_ilim.vmin_uv
			&& ilim->vmax_uv == chip->current_ilim.vmax_uv)
		return 0;

	chip->param.channel = DCIN;
	chip->param.btm_ctx = chip;
	if (wipower_dcin_interval < ADC_MEAS1_INTERVAL_0MS)
		wipower_dcin_interval = ADC_MEAS1_INTERVAL_0MS;

	if (wipower_dcin_interval > ADC_MEAS1_INTERVAL_16S)
		wipower_dcin_interval = ADC_MEAS1_INTERVAL_16S;

	chip->param.timer_interval = wipower_dcin_interval;
	chip->param.threshold_notification = &btm_notify_dcin;
	chip->param.high_thr = ilim->vmax_uv + wipower_dcin_hyst_uv;
	chip->param.low_thr = ilim->vmin_uv - wipower_dcin_hyst_uv;
	chip->param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	rc = qpnp_vadc_channel_monitor(chip->vadc_dev, &chip->param);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure btm for dcin rc = %d\n",
				rc);
	} else {
		chip->current_ilim.vmin_uv = ilim->vmin_uv;
		chip->current_ilim.vmax_uv = ilim->vmax_uv;
		pr_smb(PR_STATUS, "btm ilim = (%duV %duV %dmA %dmA %dmA)\n",
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
	}
	return rc;
}

static int smbchg_wipower_icl_configure(struct smbchg_chip *chip,
						int dcin_uv, bool div2)
{
	int rc = 0;
	struct ilim_map *map = div2 ? &chip->wipower_div2 : &chip->wipower_pt;
	struct ilim_entry *ilim = smbchg_wipower_find_entry(chip, map, dcin_uv);

	rc = smbchg_wipower_ilim_config(chip, ilim);
	if (rc) {
		dev_err(chip->dev, "failed to config ilim rc = %d, dcin_uv = %d , div2 = %d, ilim = (%duV %duV %dmA %dmA %dmA)\n",
			rc, dcin_uv, div2,
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
		return rc;
	}

	rc = smbchg_wipower_dcin_btm_configure(chip, ilim);
	if (rc) {
		dev_err(chip->dev, "failed to config btm rc = %d, dcin_uv = %d , div2 = %d, ilim = (%duV %duV %dmA %dmA %dmA)\n",
			rc, dcin_uv, div2,
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
		return rc;
	}
	chip->wipower_configured = true;
	return 0;
}

static void smbchg_wipower_icl_deconfigure(struct smbchg_chip *chip)
{
	int rc;
	struct ilim_entry *ilim = &(chip->wipower_default.entries[0]);

	if (!chip->wipower_configured)
		return;

	rc = smbchg_wipower_ilim_config(chip, ilim);
	if (rc)
		dev_err(chip->dev, "Couldn't config default ilim rc = %d\n",
				rc);

	rc = qpnp_vadc_end_channel_monitor(chip->vadc_dev);
	if (rc)
		dev_err(chip->dev, "Couldn't de configure btm for dcin rc = %d\n",
				rc);

	chip->wipower_configured = false;
	chip->current_ilim.vmin_uv = 0;
	chip->current_ilim.vmax_uv = 0;
	chip->current_ilim.icl_pt_ma = ilim->icl_pt_ma;
	chip->current_ilim.icl_lv_ma = ilim->icl_lv_ma;
	chip->current_ilim.icl_hv_ma = ilim->icl_hv_ma;
	pr_smb(PR_WIPOWER, "De config btm\n");
}

#define FV_STS		0x0C
#define DIV2_ACTIVE	BIT(7)
static void __smbchg_wipower_check(struct smbchg_chip *chip)
{
	int chg_type;
	bool usb_present, dc_present;
	int rc;
	int dcin_uv;
	bool div2;
	struct qpnp_vadc_result adc_result;
	u8 reg;

	if (!wipower_dyn_icl_en) {
		smbchg_wipower_icl_deconfigure(chip);
		return;
	}

	chg_type = get_prop_charge_type(chip);
	usb_present = is_usb_present(chip);
	dc_present = is_dc_present(chip);
	if (chg_type != POWER_SUPPLY_CHARGE_TYPE_NONE
			 && !usb_present
			&& dc_present
			&& chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER) {
		rc = qpnp_vadc_read(chip->vadc_dev, DCIN, &adc_result);
		if (rc) {
			pr_smb(PR_STATUS, "error DCIN read rc = %d\n", rc);
			return;
		}
		dcin_uv = adc_result.physical;

		
		rc = smbchg_read(chip, &reg, chip->chgr_base + FV_STS, 1);
		if (rc) {
			pr_smb(PR_STATUS, "error DCIN read rc = %d\n", rc);
			return;
		}
		div2 = !!(reg & DIV2_ACTIVE);

		pr_smb(PR_WIPOWER,
			"config ICL chg_type = %d usb = %d dc = %d dcin_uv(adc_code) = %d (0x%x) div2 = %d\n",
			chg_type, usb_present, dc_present, dcin_uv,
			adc_result.adc_code, div2);
		smbchg_wipower_icl_configure(chip, dcin_uv, div2);
	} else {
		pr_smb(PR_WIPOWER,
			"deconfig ICL chg_type = %d usb = %d dc = %d\n",
			chg_type, usb_present, dc_present);
		smbchg_wipower_icl_deconfigure(chip);
	}
}

static void smbchg_wipower_check(struct smbchg_chip *chip)
{
	if (!chip->wipower_dyn_icl_avail)
		return;

	mutex_lock(&chip->wipower_config);
	__smbchg_wipower_check(chip);
	mutex_unlock(&chip->wipower_config);
}

static void btm_notify_dcin(enum qpnp_tm_state state, void *ctx)
{
	struct smbchg_chip *chip = ctx;

	mutex_lock(&chip->wipower_config);
	pr_smb(PR_WIPOWER, "%s state\n",
			state  == ADC_TM_LOW_STATE ? "low" : "high");
	chip->current_ilim.vmin_uv = 0;
	chip->current_ilim.vmax_uv = 0;
	__smbchg_wipower_check(chip);
	mutex_unlock(&chip->wipower_config);
}

static int force_dcin_icl_write(void *data, u64 val)
{
	struct smbchg_chip *chip = data;

	smbchg_wipower_check(chip);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_dcin_icl_ops, NULL,
		force_dcin_icl_write, "0x%02llx\n");

static int set_dc_current_limit_vote_cb(struct device *dev,
						int icl_ma,
						int client,
						int last_icl_ma,
						int last_client)
{
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	return smbchg_set_dc_current_max(chip, icl_ma);
}

static int set_usb_current_limit_vote_cb(struct device *dev,
						int icl_ma,
						int client,
						int last_icl_ma,
						int last_client)
{
	struct smbchg_chip *chip = dev_get_drvdata(dev);
	int rc, aicl_ma, effective_id;

	effective_id = get_effective_client_id_locked(chip->usb_icl_votable);

	
	if (effective_id == HVDCP_ICL_VOTER)
		smbchg_parallel_usb_disable(chip);

	if (chip->parallel.current_max_ma == 0) {
		rc = smbchg_set_usb_current_max(chip, icl_ma);
		if (rc) {
			pr_err("Failed to set usb current max: %d\n", rc);
			return rc;
		}
	}

#ifdef CONFIG_HTC_BATT
		
		if (effective_id == HVDCP_ICL_VOTER){
			if (chip->usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP){
				pr_smb(PR_STATUS, "AC type but HVDCP limit ICL set, re-detect HVDCP\n");
				schedule_work(&chip->hvdcp_redet_work);
				return 0;
			}
		}
#endif 

	
	if (effective_id == HVDCP_ICL_VOTER)
		return 0;

	aicl_ma = smbchg_get_aicl_level_ma(chip);
	if (icl_ma > aicl_ma)
		smbchg_rerun_aicl(chip);
	smbchg_parallel_usb_check_ok(chip);
	return 0;
}

static int smbchg_system_temp_level_set(struct smbchg_chip *chip,
								int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;
	int thermal_icl_ma;

	if (!chip->thermal_mitigation) {
		dev_err(chip->dev, "Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		dev_err(chip->dev, "Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->thermal_levels) {
		dev_err(chip->dev, "Unsupported level selected %d forcing %d\n",
				lvl_sel, chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->therm_lvl_lock);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;
	if (chip->therm_lvl_sel == (chip->thermal_levels - 1)) {
		rc = vote(chip->dc_suspend_votable, THERMAL_EN_VOTER, true, 0);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = vote(chip->usb_suspend_votable, THERMAL_EN_VOTER, true, 0);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
		goto out;
	}

	if (chip->therm_lvl_sel == 0) {
		rc = vote(chip->usb_icl_votable, THERMAL_ICL_VOTER, false, 0);
		if (rc < 0)
			pr_err("Couldn't disable USB thermal ICL vote rc=%d\n",
				rc);

		rc = vote(chip->dc_icl_votable, THERMAL_ICL_VOTER, false, 0);
		if (rc < 0)
			pr_err("Couldn't disable DC thermal ICL vote rc=%d\n",
				rc);
	} else {
		thermal_icl_ma =
			(int)chip->thermal_mitigation[chip->therm_lvl_sel];
		rc = vote(chip->usb_icl_votable, THERMAL_ICL_VOTER, true,
					thermal_icl_ma);
		if (rc < 0)
			pr_err("Couldn't vote for USB thermal ICL rc=%d\n", rc);

		rc = vote(chip->dc_icl_votable, THERMAL_ICL_VOTER, true,
					thermal_icl_ma);
		if (rc < 0)
			pr_err("Couldn't vote for DC thermal ICL rc=%d\n", rc);
	}

	if (prev_therm_lvl == chip->thermal_levels - 1) {
		rc = vote(chip->dc_suspend_votable, THERMAL_EN_VOTER, false, 0);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = vote(chip->usb_suspend_votable, THERMAL_EN_VOTER,
								false, 0);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
	}
out:
	mutex_unlock(&chip->therm_lvl_lock);
	return rc;
}

static int smbchg_ibat_ocp_threshold_ua = 4500000;
module_param(smbchg_ibat_ocp_threshold_ua, int, 0644);

#define UCONV			1000000LL
#define MCONV			1000LL
#define FLASH_V_THRESHOLD	3000000
#define FLASH_VDIP_MARGIN	100000
#define VPH_FLASH_VDIP		(FLASH_V_THRESHOLD + FLASH_VDIP_MARGIN)
#define BUCK_EFFICIENCY		800LL
static int smbchg_calc_max_flash_current(struct smbchg_chip *chip)
{
	int ocv_uv, esr_uohm, rbatt_uohm, ibat_now, rc;
	int64_t ibat_flash_ua, avail_flash_ua, avail_flash_power_fw;
	int64_t ibat_safe_ua, vin_flash_uv, vph_flash_uv;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_OCV, &ocv_uv);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support OCV\n");
		return 0;
	}

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_RESISTANCE,
			&esr_uohm);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support resistance\n");
		return 0;
	}

	rc = msm_bcl_read(BCL_PARAM_CURRENT, &ibat_now);
	if (rc) {
		pr_smb(PR_STATUS, "BCL current read failed: %d\n", rc);
		return 0;
	}

	rbatt_uohm = esr_uohm + chip->rpara_uohm + chip->rslow_uohm;
	ibat_safe_ua = div_s64((ocv_uv - VPH_FLASH_VDIP) * UCONV,
				rbatt_uohm);

	if (ibat_safe_ua <= smbchg_ibat_ocp_threshold_ua) {
		ibat_flash_ua = ibat_safe_ua - ibat_now;
		vph_flash_uv = VPH_FLASH_VDIP;
	} else {
		ibat_flash_ua = smbchg_ibat_ocp_threshold_ua - ibat_now;
		vph_flash_uv = ocv_uv - div64_s64((int64_t)rbatt_uohm
				* smbchg_ibat_ocp_threshold_ua, UCONV);
	}
	
	vin_flash_uv = max((chip->vled_max_uv + 500000LL),
				div64_s64((vph_flash_uv * 1200), 1000));
	
	avail_flash_power_fw = BUCK_EFFICIENCY * vph_flash_uv * ibat_flash_ua;
	avail_flash_ua = div64_s64(avail_flash_power_fw, vin_flash_uv * MCONV);
	pr_smb(PR_MISC,
		"avail_iflash=%lld, ocv=%d, ibat=%d, rbatt=%d\n",
		avail_flash_ua, ocv_uv, ibat_now, rbatt_uohm);
	return (int)avail_flash_ua;
}

#define FCC_CMP_CFG	0xF3
#define FCC_COMP_MASK	SMB_MASK(1, 0)
static int smbchg_fastchg_current_comp_set(struct smbchg_chip *chip,
					int comp_current)
{
	int rc;
	u8 i;

	for (i = 0; i < chip->tables.fcc_comp_len; i++)
		if (comp_current == chip->tables.fcc_comp_table[i])
			break;

	if (i >= chip->tables.fcc_comp_len)
		return -EINVAL;

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CMP_CFG,
			FCC_COMP_MASK, i);

	if (rc)
		dev_err(chip->dev, "Couldn't set fastchg current comp rc = %d\n",
			rc);

	return rc;
}

#define CFG_TCC_REG			0xF9
#define CHG_ITERM_MASK			SMB_MASK(2, 0)
static int smbchg_iterm_set(struct smbchg_chip *chip, int iterm_ma)
{
	int rc;
	u8 reg;

	reg = find_closest_in_array(
			chip->tables.iterm_ma_table,
			chip->tables.iterm_ma_len,
			iterm_ma);

	rc = smbchg_sec_masked_write(chip,
			chip->chgr_base + CFG_TCC_REG,
			CHG_ITERM_MASK, reg);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set iterm rc = %d\n", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "set tcc (%d) to 0x%02x\n",
			iterm_ma, reg);
	chip->iterm_ma = iterm_ma;

	return 0;
}

#define FV_CMP_CFG	0xF5
#define FV_COMP_MASK	SMB_MASK(5, 0)
static int smbchg_float_voltage_comp_set(struct smbchg_chip *chip, int code)
{
	int rc;
	u8 val;

	val = code & FV_COMP_MASK;
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FV_CMP_CFG,
			FV_COMP_MASK, val);

	if (rc)
		dev_err(chip->dev, "Couldn't set float voltage comp rc = %d\n",
			rc);

	return rc;
}

#define VFLOAT_CFG_REG			0xF4
#define MIN_FLOAT_MV			3600
#define MAX_FLOAT_MV			4500
#define VFLOAT_MASK			SMB_MASK(5, 0)

#define MID_RANGE_FLOAT_MV_MIN		3600
#define MID_RANGE_FLOAT_MIN_VAL		0x05
#define MID_RANGE_FLOAT_STEP_MV		20

#define HIGH_RANGE_FLOAT_MIN_MV		4340
#define HIGH_RANGE_FLOAT_MIN_VAL	0x2A
#define HIGH_RANGE_FLOAT_STEP_MV	10

#define VHIGH_RANGE_FLOAT_MIN_MV	4360
#define VHIGH_RANGE_FLOAT_MIN_VAL	0x2C
#define VHIGH_RANGE_FLOAT_STEP_MV	20
static int smbchg_float_voltage_set(struct smbchg_chip *chip, int vfloat_mv)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	int rc, delta;
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	if (vfloat_mv <= HIGH_RANGE_FLOAT_MIN_MV) {
		
		delta = vfloat_mv - MID_RANGE_FLOAT_MV_MIN;
		temp = MID_RANGE_FLOAT_MIN_VAL + delta
				/ MID_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % MID_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_mv <= VHIGH_RANGE_FLOAT_MIN_MV) {
		
		delta = vfloat_mv - HIGH_RANGE_FLOAT_MIN_MV;
		temp = HIGH_RANGE_FLOAT_MIN_VAL + delta
				/ HIGH_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % HIGH_RANGE_FLOAT_STEP_MV;
	} else {
		
		delta = vfloat_mv - VHIGH_RANGE_FLOAT_MIN_MV;
		temp = VHIGH_RANGE_FLOAT_MIN_VAL + delta
				/ VHIGH_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % VHIGH_RANGE_FLOAT_STEP_MV;
	}

	if (parallel_psy) {
		rc = power_supply_set_voltage_limit(parallel_psy,
				vfloat_mv + 50);
		if (rc)
			dev_err(chip->dev, "Couldn't set float voltage on parallel psy rc: %d\n",
				rc);
	}

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG,
			VFLOAT_MASK, temp);

	if (rc)
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
	else
		chip->vfloat_mv = vfloat_mv;

	return rc;
}

static int smbchg_float_voltage_get(struct smbchg_chip *chip)
{
	return chip->vfloat_mv;
}

#define SFT_CFG				0xFD
#define SFT_EN_MASK			SMB_MASK(5, 4)
#define SFT_TO_MASK			SMB_MASK(3, 2)
#define PRECHG_SFT_TO_MASK		SMB_MASK(1, 0)
#define SFT_TIMER_DISABLE_BIT		BIT(5)
#define PRECHG_SFT_TIMER_DISABLE_BIT	BIT(4)
#define SAFETY_TIME_MINUTES_SHIFT	2
static int smbchg_safety_timer_enable(struct smbchg_chip *chip, bool enable)
{
	int rc;
	u8 reg;

	if (enable == chip->safety_timer_en)
		return 0;

	if (enable)
		reg = 0;
	else
		reg = SFT_TIMER_DISABLE_BIT | PRECHG_SFT_TIMER_DISABLE_BIT;

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + SFT_CFG,
			SFT_EN_MASK, reg);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s safety timer rc = %d\n",
			enable ? "enable" : "disable", rc);
		return rc;
	}
	chip->safety_timer_en = enable;
	return 0;
}

enum skip_reason {
	REASON_OTG_ENABLED	= BIT(0),
	REASON_FLASH_ENABLED	= BIT(1)
};

#define BAT_IF_TRIM7_REG	0xF7
#define CFG_750KHZ_BIT		BIT(1)
#define MISC_CFG_NTC_VOUT_REG	0xF3
#define CFG_NTC_VOUT_FSW_BIT	BIT(0)
static int smbchg_switch_buck_frequency(struct smbchg_chip *chip,
				bool flash_active)
{
	int rc;

	if (!(chip->wa_flags & SMBCHG_FLASH_BUCK_SWITCH_FREQ_WA))
		return 0;

	if (chip->flash_active == flash_active) {
		pr_smb(PR_STATUS, "Fsw not changed, flash_active: %d\n",
			flash_active);
		return 0;
	}

#ifdef CONFIG_HTC_BATT
	if(chip->htc_wa_flags & HTC_FORCE_FSW_1MHZ_WA) {
		pr_smb(PR_STATUS, "Fsw already forced to 1MHz\n");
		chip->flash_active = flash_active;
		return 0;
	}
#endif

	rc = smbchg_sec_masked_write(chip, chip->misc_base +
				MISC_CFG_NTC_VOUT_REG, CFG_NTC_VOUT_FSW_BIT,
				flash_active ? CFG_NTC_VOUT_FSW_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set switching frequency multiplier rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + BAT_IF_TRIM7_REG,
			CFG_750KHZ_BIT, flash_active ? 0 : CFG_750KHZ_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Cannot set switching freq: %d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Fsw @ %sHz\n", flash_active ? "1M" : "750K");
	chip->flash_active = flash_active;
	return 0;
}

#define OTG_TRIM6		0xF6
#define TR_ENB_SKIP_BIT		BIT(2)
#define OTG_EN_BIT		BIT(0)
static int smbchg_otg_pulse_skip_disable(struct smbchg_chip *chip,
				enum skip_reason reason, bool disable)
{
	int rc;
	bool disabled;

	disabled = !!chip->otg_pulse_skip_dis;
	pr_smb(PR_STATUS, "%s pulse skip, reason %d\n",
			disable ? "disabling" : "enabling", reason);
	if (disable)
		chip->otg_pulse_skip_dis |= reason;
	else
		chip->otg_pulse_skip_dis &= ~reason;
	if (disabled == !!chip->otg_pulse_skip_dis)
		return 0;
	disabled = !!chip->otg_pulse_skip_dis;

	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_TRIM6,
			TR_ENB_SKIP_BIT, disabled ? TR_ENB_SKIP_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s otg pulse skip rc = %d\n",
			disabled ? "disable" : "enable", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "%s pulse skip\n", disabled ? "disabled" : "enabled");
	return 0;
}

#define LOW_PWR_OPTIONS_REG	0xFF
#define FORCE_TLIM_BIT		BIT(4)
static int smbchg_force_tlim_en(struct smbchg_chip *chip, bool enable)
{
	int rc;

	rc = smbchg_sec_masked_write(chip, chip->otg_base + LOW_PWR_OPTIONS_REG,
			FORCE_TLIM_BIT, enable ? FORCE_TLIM_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s otg force tlim rc = %d\n",
			enable ? "enable" : "disable", rc);
		return rc;
	}
	return rc;
}

static void smbchg_vfloat_adjust_check(struct smbchg_chip *chip)
{
	if (!chip->use_vfloat_adjustments)
		return;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	pr_smb(PR_STATUS, "Starting vfloat adjustments\n");
	schedule_delayed_work(&chip->vfloat_adjust_work, 0);
}

#define FV_STS_REG			0xC
#define AICL_INPUT_STS_BIT		BIT(6)
static bool smbchg_is_input_current_limited(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->chgr_base + FV_STS_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read FV_STS rc=%d\n", rc);
		return false;
	}

	return !!(reg & AICL_INPUT_STS_BIT);
}

#define SW_ESR_PULSE_MS			1500
static void smbchg_cc_esr_wa_check(struct smbchg_chip *chip)
{
	int rc, esr_count;

	if (!(chip->wa_flags & SMBCHG_CC_ESR_WA))
		return;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "No inputs present, skipping\n");
		return;
	}

	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return;
	}

	if (!smbchg_is_input_current_limited(chip)) {
		pr_smb(PR_STATUS, "Not input current limited, skipping\n");
		return;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_ESR_COUNT, &esr_count);
	if (rc) {
		pr_smb(PR_STATUS,
			"could not read ESR counter rc = %d\n", rc);
		return;
	}

	if (esr_count != 0) {
		pr_smb(PR_STATUS, "ESR count is not zero, skipping\n");
		return;
	}

	pr_smb(PR_STATUS, "Lowering charge current for ESR pulse\n");
	smbchg_stay_awake(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, true);
	msleep(SW_ESR_PULSE_MS);
	pr_smb(PR_STATUS, "Raising charge current for ESR pulse\n");
	smbchg_relax(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, false);
}

static void smbchg_soc_changed(struct smbchg_chip *chip)
{
	smbchg_cc_esr_wa_check(chip);
}

#define DC_AICL_CFG			0xF3
#define MISC_TRIM_OPT_15_8		0xF5
#define USB_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_LONG		0
#define DC_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_LONG		0
#define AICL_RERUN_MASK			(BIT(5) | BIT(4))
#define AICL_RERUN_ON			(BIT(5) | BIT(4))
#define AICL_RERUN_OFF			0

static int smbchg_hw_aicl_rerun_enable_indirect_cb(struct device *dev,
						int enable,
						int client, int last_enable,
						int last_client)
{
	int rc = 0;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = vote(chip->hw_aicl_rerun_disable_votable,
		HW_AICL_RERUN_ENABLE_INDIRECT_VOTER, !enable, 0);
	if (rc < 0) {
		pr_err("Couldn't vote for hw rerun rc= %d\n", rc);
		return rc;
	}

	return rc;
}

static int smbchg_hw_aicl_rerun_disable_cb(struct device *dev, int disable,
						int client, int last_disable,
						int last_client)
{
	int rc = 0;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_sec_masked_write(chip,
		chip->misc_base + MISC_TRIM_OPT_15_8,
		AICL_RERUN_MASK, disable ? AICL_RERUN_OFF : AICL_RERUN_ON);
	if (rc < 0)
		pr_err("Couldn't write to MISC_TRIM_OPTIONS_15_8 rc=%d\n", rc);

	return rc;
}

static int smbchg_aicl_deglitch_config_cb(struct device *dev, int shorter,
						int client, int last_result,
						int last_client)
{
	int rc = 0;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_sec_masked_write(chip,
		chip->usb_chgpth_base + USB_AICL_CFG,
		USB_AICL_DEGLITCH_MASK,
		shorter ? USB_AICL_DEGLITCH_SHORT : USB_AICL_DEGLITCH_LONG);
	if (rc < 0) {
		pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
		return rc;
	}
	rc = smbchg_sec_masked_write(chip,
		chip->dc_chgpth_base + DC_AICL_CFG,
		DC_AICL_DEGLITCH_MASK,
		shorter ? DC_AICL_DEGLITCH_SHORT : DC_AICL_DEGLITCH_LONG);
	if (rc < 0) {
		pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
		return rc;
	}
	return rc;
}

static void smbchg_aicl_deglitch_wa_en(struct smbchg_chip *chip, bool en)
{
	int rc;

	rc = vote(chip->aicl_deglitch_short_votable,
		VARB_WORKAROUND_VOTER, en, 0);
	if (rc < 0) {
		pr_err("Couldn't vote %s deglitch rc=%d\n",
				en ? "short" : "long", rc);
		return;
	}
	pr_smb(PR_STATUS, "AICL deglitch set to %s\n", en ? "short" : "long");

#ifdef CONFIG_HTC_BATT
			
			
	pr_smb(PR_STATUS, "Not disable aicl rerun.\n");
#else
	rc = vote(chip->hw_aicl_rerun_enable_indirect_votable,
			VARB_WORKAROUND_VOTER, en, 0);
	if (rc < 0) {
		pr_err("Couldn't vote hw aicl rerun rc= %d\n", rc);
		return;
	}
#endif
	chip->aicl_deglitch_short = en;
}

static void smbchg_aicl_deglitch_wa_check(struct smbchg_chip *chip)
{
	union power_supply_propval prop = {0,};
	int rc;
	bool low_volt_chgr = true;

	if (!(chip->wa_flags & SMBCHG_AICL_DEGLITCH_WA))
		return;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "Charger removed\n");
		smbchg_aicl_deglitch_wa_en(chip, false);
		return;
	}

	if (!chip->bms_psy)
		return;

	if (is_usb_present(chip)) {
		if (is_hvdcp_present(chip))
			low_volt_chgr = false;
	} else if (is_dc_present(chip)) {
		if (chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
			low_volt_chgr = false;
		else
			low_volt_chgr = chip->low_volt_dcin;
	}

	if (!low_volt_chgr) {
		pr_smb(PR_STATUS, "High volt charger! Don't set deglitch\n");
		smbchg_aicl_deglitch_wa_en(chip, false);
		return;
	}

	if (!chip->vbat_above_headroom) {
		rc = chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MIN, &prop);
		if (rc < 0) {
			pr_err("could not read voltage_min, rc=%d\n", rc);
			return;
		}
		chip->vbat_above_headroom = !prop.intval;
	}
	smbchg_aicl_deglitch_wa_en(chip, chip->vbat_above_headroom);
}

#define MISC_TEST_REG		0xE2
#define BB_LOOP_DISABLE_ICL	BIT(2)
static int smbchg_icl_loop_disable_check(struct smbchg_chip *chip)
{
	bool icl_disabled = !chip->chg_otg_enabled && chip->flash_triggered;
	int rc = 0;

	if ((chip->wa_flags & SMBCHG_FLASH_ICL_DISABLE_WA)
			&& icl_disabled != chip->icl_disabled) {
		rc = smbchg_sec_masked_write(chip,
				chip->misc_base + MISC_TEST_REG,
				BB_LOOP_DISABLE_ICL,
				icl_disabled ? BB_LOOP_DISABLE_ICL : 0);
		chip->icl_disabled = icl_disabled;
	}

	return rc;
}

#define UNKNOWN_BATT_TYPE	"Unknown Battery"
#define LOADING_BATT_TYPE	"Loading Battery Data"
static int smbchg_config_chg_battery_type(struct smbchg_chip *chip)
{
	int rc = 0, max_voltage_uv = 0, fastchg_ma = 0, ret = 0, iterm_ua = 0;
	struct device_node *batt_node, *profile_node;
	struct device_node *node = chip->spmi->dev.of_node;
	union power_supply_propval prop = {0,};

	rc = chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
	if (rc) {
		pr_smb(PR_STATUS, "Unable to read battery-type rc=%d\n", rc);
		return 0;
	}
	if (!strcmp(prop.strval, UNKNOWN_BATT_TYPE) ||
		!strcmp(prop.strval, LOADING_BATT_TYPE)) {
		pr_smb(PR_MISC, "Battery-type not identified\n");
		return 0;
	}
	
	if (chip->battery_type && !strcmp(prop.strval, chip->battery_type))
		return 0;

	batt_node = of_parse_phandle(node, "qcom,battery-data", 0);
	if (!batt_node) {
		pr_smb(PR_MISC, "No batterydata available\n");
		return 0;
	}

	profile_node = of_batterydata_get_best_profile(batt_node,
							"bms", NULL);
	if (!profile_node) {
		pr_err("couldn't find profile handle\n");
		return -EINVAL;
	}
	chip->battery_type = prop.strval;

	
	rc = of_property_read_u32(profile_node, "qcom,max-voltage-uv",
						&max_voltage_uv);
	if (rc) {
		pr_warn("couldn't find battery max voltage rc=%d\n", rc);
		ret = rc;
	} else {
		if (chip->vfloat_mv != (max_voltage_uv / 1000)) {
			pr_info("Vfloat changed from %dmV to %dmV for battery-type %s\n",
				chip->vfloat_mv, (max_voltage_uv / 1000),
				chip->battery_type);
			rc = smbchg_float_voltage_set(chip,
						(max_voltage_uv / 1000));
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
				return rc;
			}
		}
	}

	
	rc = of_property_read_u32(profile_node, "qcom,chg-term-ua",
						&iterm_ua);
	if (rc && rc != -EINVAL) {
		pr_warn("couldn't read battery term current=%d\n", rc);
		ret = rc;
	} else if (!rc) {
		if (chip->iterm_ma != (iterm_ua / 1000)
				&& !chip->iterm_disabled) {
			pr_info("Term current changed from %dmA to %dmA for battery-type %s\n",
				chip->iterm_ma, (iterm_ua / 1000),
				chip->battery_type);
			rc = smbchg_iterm_set(chip,
						(iterm_ua / 1000));
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't set iterm rc = %d\n", rc);
				return rc;
			}
		}
		chip->iterm_ma = iterm_ua / 1000;
	}

	if (!of_find_property(chip->spmi->dev.of_node,
				"qcom,fastchg-current-ma", NULL)) {
		rc = of_property_read_u32(profile_node,
				"qcom,fastchg-current-ma", &fastchg_ma);
		if (rc) {
			ret = rc;
		} else {
			pr_smb(PR_MISC,
				"fastchg-ma changed from to %dma for battery-type %s\n",
				fastchg_ma, chip->battery_type);
			rc = vote(chip->fcc_votable, BATT_TYPE_FCC_VOTER, true,
							fastchg_ma);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't vote for fastchg current rc=%d\n",
					rc);
				return rc;
			}
		}
	}

	return ret;
}

#define MAX_INV_BATT_ID		7700
#define MIN_INV_BATT_ID		7300
static void check_battery_type(struct smbchg_chip *chip)
{
	union power_supply_propval prop = {0,};
	bool en;

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
		en = (strcmp(prop.strval, UNKNOWN_BATT_TYPE) != 0
				|| chip->charge_unknown_battery)
			&& (strcmp(prop.strval, LOADING_BATT_TYPE) != 0);
		vote(chip->battchg_suspend_votable,
				BATTCHG_UNKNOWN_BATTERY_EN_VOTER, !en, 0);

		if (!chip->skip_usb_suspend_for_fake_battery) {
			chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_RESISTANCE_ID, &prop);
			
			en = (prop.intval <= MAX_INV_BATT_ID &&
				prop.intval >= MIN_INV_BATT_ID) ? 1 : 0;
			vote(chip->usb_suspend_votable, FAKE_BATTERY_EN_VOTER,
				en, 0);
		}
	}
}

static void smbchg_external_power_changed(struct power_supply *psy)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0, soc;
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";

	if (chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);

	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->bms_psy) {
		check_battery_type(chip);
		soc = get_prop_batt_capacity(chip);
		if (chip->previous_soc != soc) {
			chip->previous_soc = soc;
			smbchg_soc_changed(chip);
		}

		rc = smbchg_config_chg_battery_type(chip);
		if (rc)
			pr_smb(PR_MISC,
				"Couldn't update charger configuration rc=%d\n",
									rc);
	}

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
	if (rc == 0)
		vote(chip->usb_suspend_votable, POWER_SUPPLY_EN_VOTER,
				!prop.intval, 0);

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc == 0)
		current_limit = prop.intval / 1000;

	read_usb_type(chip, &usb_type_name, &usb_supply_type);

	if (usb_supply_type != POWER_SUPPLY_TYPE_USB)
		goto  skip_current_for_non_sdp;

	pr_smb(PR_MISC, "usb type = %s current_limit = %d\n",
			usb_type_name, current_limit);

	rc = vote(chip->usb_icl_votable, PSY_ICL_VOTER, true,
				current_limit);
	if (rc < 0)
		pr_err("Couldn't update USB PSY ICL vote rc=%d\n", rc);

skip_current_for_non_sdp:
	smbchg_vfloat_adjust_check(chip);

	power_supply_changed(&chip->batt_psy);
}

static int smbchg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	chip->otg_retries = 0;
	chip->chg_otg_enabled = true;
	smbchg_icl_loop_disable_check(chip);
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, true);

	
	if (chip->otg_pinctrl)
		return rc;

	
	msleep(20);
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			OTG_EN_BIT, OTG_EN_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d\n", rc);
	else
		chip->otg_enable_time = ktime_get();
	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	if (!chip->otg_pinctrl) {
		rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
				OTG_EN_BIT, 0);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n",
					rc);
	}

	chip->chg_otg_enabled = false;
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, false);
	smbchg_icl_loop_disable_check(chip);
	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_read(chip, &reg, chip->bat_if_base + CMD_CHG_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return (reg & OTG_EN_BIT) ? 1 : 0;
}

struct regulator_ops smbchg_otg_reg_ops = {
	.enable		= smbchg_otg_regulator_enable,
	.disable	= smbchg_otg_regulator_disable,
	.is_enabled	= smbchg_otg_regulator_is_enable,
};

#define USBIN_CHGR_CFG			0xF1
#define ADAPTER_ALLOWANCE_MASK		0x7
#define USBIN_ADAPTER_9V		0x3
#define USBIN_ADAPTER_5V_9V_CONT	0x2
#define USBIN_ADAPTER_5V_UNREGULATED_9V	0x5
#define HVDCP_EN_BIT			BIT(3)
static int smbchg_external_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = vote(chip->usb_suspend_votable, OTG_EN_VOTER, true, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't suspend charger rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_read(chip, &chip->original_usbin_allowance,
			chip->usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable HVDCP rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + USBIN_CHGR_CFG,
				0xFF, USBIN_ADAPTER_9V);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_external_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = vote(chip->usb_suspend_votable, OTG_EN_VOTER, false, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't unsuspend charger rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable HVDCP rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + USBIN_CHGR_CFG,
				0xFF, chip->original_usbin_allowance);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_external_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	return get_client_vote(chip->usb_suspend_votable, OTG_EN_VOTER);
}

struct regulator_ops smbchg_external_otg_reg_ops = {
	.enable		= smbchg_external_otg_regulator_enable,
	.disable	= smbchg_external_otg_regulator_disable,
	.is_enabled	= smbchg_external_otg_regulator_is_enable,
};

static int smbchg_regulator_init(struct smbchg_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	struct device_node *regulator_node;

	regulator_node = of_get_child_by_name(chip->dev->of_node,
			"qcom,smbcharger-boost-otg");

	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smbchg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = devm_regulator_register(chip->dev,
						&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	if (rc)
		return rc;

	regulator_node = of_get_child_by_name(chip->dev->of_node,
			"qcom,smbcharger-external-otg");
	if (!regulator_node) {
		dev_dbg(chip->dev, "external-otg node absent\n");
		return 0;
	}
	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		if (of_get_property(chip->dev->of_node,
					"otg-parent-supply", NULL))
			init_data->supply_regulator = "otg-parent";
		chip->ext_otg_vreg.rdesc.owner = THIS_MODULE;
		chip->ext_otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->ext_otg_vreg.rdesc.ops = &smbchg_external_otg_reg_ops;
		chip->ext_otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->ext_otg_vreg.rdev = devm_regulator_register(chip->dev,
						&chip->ext_otg_vreg.rdesc,
						&cfg);
		if (IS_ERR(chip->ext_otg_vreg.rdev)) {
			rc = PTR_ERR(chip->ext_otg_vreg.rdev);
			chip->ext_otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"external OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

#define CMD_CHG_LED_REG		0x43
#define CHG_LED_CTRL_BIT		BIT(0)
#define LED_SW_CTRL_BIT		0x1
#define LED_CHG_CTRL_BIT		0x0
#define CHG_LED_ON		0x03
#define CHG_LED_OFF		0x00
#define LED_BLINKING_PATTERN1		0x01
#define LED_BLINKING_PATTERN2		0x02
#define LED_BLINKING_CFG_MASK		SMB_MASK(2, 1)
#define CHG_LED_SHIFT		1
static int smbchg_chg_led_controls(struct smbchg_chip *chip)
{
	u8 reg, mask;
	int rc;

	if (chip->cfg_chg_led_sw_ctrl) {
		
		mask = CHG_LED_CTRL_BIT | LED_BLINKING_CFG_MASK;
		reg = LED_SW_CTRL_BIT;
	} else {
		mask = CHG_LED_CTRL_BIT;
		reg = LED_CHG_CTRL_BIT;
	}

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_LED_REG,
			mask, reg);
	if (rc < 0)
		dev_err(chip->dev,
				"Couldn't write LED_CTRL_BIT rc=%d\n", rc);
	return rc;
}

static void smbchg_chg_led_brightness_set(struct led_classdev *cdev,
		enum led_brightness value)
{
	struct smbchg_chip *chip = container_of(cdev,
			struct smbchg_chip, led_cdev);
	u8 reg;
	int rc;

	reg = (value > LED_OFF) ? CHG_LED_ON << CHG_LED_SHIFT :
		CHG_LED_OFF << CHG_LED_SHIFT;

	if (value > LED_OFF)
		power_supply_set_hi_power_state(chip->bms_psy, 1);
	else
		power_supply_set_hi_power_state(chip->bms_psy, 0);

	pr_smb(PR_STATUS,
			"set the charger led brightness to value=%d\n",
			value);
	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + CMD_CHG_LED_REG,
			LED_BLINKING_CFG_MASK, reg);
	if (rc)
		dev_err(chip->dev, "Couldn't write CHG_LED rc=%d\n",
				rc);
}

static enum
led_brightness smbchg_chg_led_brightness_get(struct led_classdev *cdev)
{
	struct smbchg_chip *chip = container_of(cdev,
			struct smbchg_chip, led_cdev);
	u8 reg_val, chg_led_sts;
	int rc;

	rc = smbchg_read(chip, &reg_val, chip->bat_if_base + CMD_CHG_LED_REG,
			1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read CHG_LED_REG sts rc=%d\n",
				rc);
		return rc;
	}

	chg_led_sts = (reg_val & LED_BLINKING_CFG_MASK) >> CHG_LED_SHIFT;

	pr_smb(PR_STATUS, "chg_led_sts = %02x\n", chg_led_sts);

	return (chg_led_sts == CHG_LED_OFF) ? LED_OFF : LED_FULL;
}

static void smbchg_chg_led_blink_set(struct smbchg_chip *chip,
		unsigned long blinking)
{
	u8 reg;
	int rc;

	if (blinking == 0) {
		reg = CHG_LED_OFF << CHG_LED_SHIFT;
		power_supply_set_hi_power_state(chip->bms_psy, 0);
	} else {
		power_supply_set_hi_power_state(chip->bms_psy, 1);
		if (blinking == 1)
			reg = LED_BLINKING_PATTERN1 << CHG_LED_SHIFT;
		else if (blinking == 2)
			reg = LED_BLINKING_PATTERN2 << CHG_LED_SHIFT;
		else
			reg = LED_BLINKING_PATTERN1 << CHG_LED_SHIFT;
	}

	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + CMD_CHG_LED_REG,
			LED_BLINKING_CFG_MASK, reg);
	if (rc)
		dev_err(chip->dev, "Couldn't write CHG_LED rc=%d\n",
				rc);
}

static ssize_t smbchg_chg_led_blink_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct smbchg_chip *chip = container_of(cdev, struct smbchg_chip,
			led_cdev);
	unsigned long blinking;
	ssize_t rc = -EINVAL;

	rc = kstrtoul(buf, 10, &blinking);
	if (rc)
		return rc;

	smbchg_chg_led_blink_set(chip, blinking);

	return len;
}

static DEVICE_ATTR(blink, 0664, NULL, smbchg_chg_led_blink_store);

static struct attribute *led_blink_attributes[] = {
	&dev_attr_blink.attr,
	NULL,
};

static struct attribute_group smbchg_led_attr_group = {
	.attrs = led_blink_attributes
};

static int smbchg_register_chg_led(struct smbchg_chip *chip)
{
	int rc;

	chip->led_cdev.name = "red";
	chip->led_cdev.brightness_set = smbchg_chg_led_brightness_set;
	chip->led_cdev.brightness_get = smbchg_chg_led_brightness_get;

	rc = led_classdev_register(chip->dev, &chip->led_cdev);
	if (rc) {
		dev_err(chip->dev, "unable to register charger led, rc=%d\n",
				rc);
		return rc;
	}

	rc = sysfs_create_group(&chip->led_cdev.dev->kobj,
			&smbchg_led_attr_group);
	if (rc) {
		dev_err(chip->dev, "led sysfs rc: %d\n", rc);
		return rc;
	}

	return rc;
}

static int vf_adjust_low_threshold = 5;
module_param(vf_adjust_low_threshold, int, 0644);

static int vf_adjust_high_threshold = 7;
module_param(vf_adjust_high_threshold, int, 0644);

static int vf_adjust_n_samples = 10;
module_param(vf_adjust_n_samples, int, 0644);

static int vf_adjust_max_delta_mv = 40;
module_param(vf_adjust_max_delta_mv, int, 0644);

static int vf_adjust_trim_steps_per_adjust = 1;
module_param(vf_adjust_trim_steps_per_adjust, int, 0644);

#define CENTER_TRIM_CODE		7
#define MAX_LIN_CODE			14
#define MAX_TRIM_CODE			15
#define SCALE_SHIFT			4
#define VF_TRIM_OFFSET_MASK		SMB_MASK(3, 0)
#define VF_STEP_SIZE_MV			10
#define SCALE_LSB_MV			17
static int smbchg_trim_add_steps(int prev_trim, int delta_steps)
{
	int scale_steps;
	int linear_offset, linear_scale;
	int offset_code = prev_trim & VF_TRIM_OFFSET_MASK;
	int scale_code = (prev_trim & ~VF_TRIM_OFFSET_MASK) >> SCALE_SHIFT;

	if (abs(delta_steps) > 1) {
		pr_smb(PR_STATUS,
			"Cant trim multiple steps delta_steps = %d\n",
			delta_steps);
		return prev_trim;
	}
	if (offset_code <= CENTER_TRIM_CODE)
		linear_offset = offset_code + CENTER_TRIM_CODE;
	else if (offset_code > CENTER_TRIM_CODE)
		linear_offset = MAX_TRIM_CODE - offset_code;

	if (scale_code <= CENTER_TRIM_CODE)
		linear_scale = scale_code + CENTER_TRIM_CODE;
	else if (scale_code > CENTER_TRIM_CODE)
		linear_scale = scale_code - (CENTER_TRIM_CODE + 1);

	
	if (linear_offset + delta_steps >= 0
			&& linear_offset + delta_steps <= MAX_LIN_CODE) {
		linear_offset += delta_steps;

		if (linear_offset > CENTER_TRIM_CODE)
			offset_code = linear_offset - CENTER_TRIM_CODE;
		else
			offset_code = MAX_TRIM_CODE - linear_offset;

		return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
	}

	
	scale_steps = delta_steps > 0 ? 1 : -1;

	if (linear_scale + scale_steps < 0
			|| linear_scale + scale_steps > MAX_LIN_CODE) {
		pr_smb(PR_STATUS,
			"Cant trim scale_steps = %d delta_steps = %d\n",
			scale_steps, delta_steps);
		return prev_trim;
	}

	linear_scale += scale_steps;

	if (linear_scale > CENTER_TRIM_CODE)
		scale_code = linear_scale - CENTER_TRIM_CODE;
	else
		scale_code = linear_scale + (CENTER_TRIM_CODE + 1);
	prev_trim = (prev_trim & VF_TRIM_OFFSET_MASK)
		| scale_code << SCALE_SHIFT;

	delta_steps = -1 * delta_steps;

	linear_offset = clamp(linear_offset + delta_steps, 0, MAX_LIN_CODE);
	if (linear_offset > CENTER_TRIM_CODE)
		offset_code = linear_offset - CENTER_TRIM_CODE;
	else
		offset_code = MAX_TRIM_CODE - linear_offset;

	return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
}

#define TRIM_14		0xFE
#define VF_TRIM_MASK	0xFF
static int smbchg_adjust_vfloat_mv_trim(struct smbchg_chip *chip,
						int delta_mv)
{
	int sign, delta_steps, rc = 0;
	u8 prev_trim, new_trim;
	int i;

	sign = delta_mv > 0 ? 1 : -1;
	delta_steps = (delta_mv + sign * VF_STEP_SIZE_MV / 2)
			/ VF_STEP_SIZE_MV;

	rc = smbchg_read(chip, &prev_trim, chip->misc_base + TRIM_14, 1);
	if (rc) {
		dev_err(chip->dev, "Unable to read trim 14: %d\n", rc);
		return rc;
	}

	for (i = 1; i <= abs(delta_steps)
			&& i <= vf_adjust_trim_steps_per_adjust; i++) {
		new_trim = (u8)smbchg_trim_add_steps(prev_trim,
				delta_steps > 0 ? 1 : -1);
		if (new_trim == prev_trim) {
			pr_smb(PR_STATUS,
				"VFloat trim unchanged from %02x\n", prev_trim);
			
			return -EINVAL;
		}

		rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_14,
				VF_TRIM_MASK, new_trim);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't change vfloat trim rc=%d\n", rc);
		}
		pr_smb(PR_STATUS,
			"VFlt trim %02x to %02x, delta steps: %d\n",
			prev_trim, new_trim, delta_steps);
		prev_trim = new_trim;
	}

	return rc;
}

#define VFLOAT_RESAMPLE_DELAY_MS	10000
static void smbchg_vfloat_adjust_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				vfloat_adjust_work.work);
	int vbat_uv, vbat_mv, ibat_ua, rc, delta_vfloat_mv;
	bool taper, enable;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	taper = (get_prop_charge_type(chip)
		== POWER_SUPPLY_CHARGE_TYPE_TAPER);
	enable = taper && (chip->parallel.current_max_ma == 0);

	if (!enable) {
		pr_smb(PR_MISC,
			"Stopping vfloat adj taper=%d parallel_ma = %d\n",
			taper, chip->parallel.current_max_ma);
		goto stop;
	}

	if (get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_smb(PR_STATUS, "JEITA active, skipping\n");
		goto stop;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_uv);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy does not support voltage rc = %d\n", rc);
		goto stop;
	}
	vbat_mv = vbat_uv / 1000;

	if ((vbat_mv - chip->vfloat_mv) < -1 * vf_adjust_max_delta_mv) {
		pr_smb(PR_STATUS, "Skip vbat out of range: %d\n", vbat_mv);
		goto reschedule;
	}

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ibat_ua);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy does not support current_now rc = %d\n", rc);
		goto stop;
	}

	if (ibat_ua / 1000 > -chip->iterm_ma) {
		pr_smb(PR_STATUS, "Skip ibat too high: %d\n", ibat_ua);
		goto reschedule;
	}

	pr_smb(PR_STATUS, "sample number = %d vbat_mv = %d ibat_ua = %d\n",
		chip->n_vbat_samples,
		vbat_mv,
		ibat_ua);

	chip->max_vbat_sample = max(chip->max_vbat_sample, vbat_mv);
	chip->n_vbat_samples += 1;
	if (chip->n_vbat_samples < vf_adjust_n_samples) {
		pr_smb(PR_STATUS, "Skip %d samples; max = %d\n",
			chip->n_vbat_samples, chip->max_vbat_sample);
		goto reschedule;
	}
	
	delta_vfloat_mv = chip->vfloat_mv - chip->max_vbat_sample;
	pr_smb(PR_STATUS, "delta_vfloat_mv = %d, samples = %d, mvbat = %d\n",
		delta_vfloat_mv, chip->n_vbat_samples, chip->max_vbat_sample);
	if (delta_vfloat_mv > vf_adjust_high_threshold
			|| delta_vfloat_mv < -1 * vf_adjust_low_threshold) {
		rc = smbchg_adjust_vfloat_mv_trim(chip, delta_vfloat_mv);
		if (rc) {
			pr_smb(PR_STATUS,
				"Stopping vfloat adj after trim adj rc = %d\n",
				 rc);
			goto stop;
		}
		chip->max_vbat_sample = 0;
		chip->n_vbat_samples = 0;
		goto reschedule;
	}

stop:
	chip->max_vbat_sample = 0;
	chip->n_vbat_samples = 0;
	smbchg_relax(chip, PM_REASON_VFLOAT_ADJUST);
	return;

reschedule:
	schedule_delayed_work(&chip->vfloat_adjust_work,
			msecs_to_jiffies(VFLOAT_RESAMPLE_DELAY_MS));
	return;
}

static int smbchg_charging_status_change(struct smbchg_chip *chip)
{
	smbchg_vfloat_adjust_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
	return 0;
}

#define BB_CLMP_SEL		0xF8
#define BB_CLMP_MASK		SMB_MASK(1, 0)
#define BB_CLMP_VFIX_3338MV	0x1
#define BB_CLMP_VFIX_3512MV	0x2
static int smbchg_set_optimal_charging_mode(struct smbchg_chip *chip, int type)
{
	int rc;
	bool hvdcp2 = (type == POWER_SUPPLY_TYPE_USB_HVDCP
			&& smbchg_is_usbin_active_pwr_src(chip));

	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + BAT_IF_TRIM7_REG,
			CFG_750KHZ_BIT, hvdcp2 ? 0 : CFG_750KHZ_BIT);
	if (rc) {
		dev_err(chip->dev, "Cannot set switching freq: %d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + BB_CLMP_SEL,
			BB_CLMP_MASK,
			hvdcp2 ? BB_CLMP_VFIX_3338MV : BB_CLMP_VFIX_3512MV);
	if (rc) {
		dev_err(chip->dev, "Cannot set switching freq: %d\n", rc);
		return rc;
	}

	return 0;
}

#define DEFAULT_SDP_MA		100
#define DEFAULT_CDP_MA		1500
static int smbchg_change_usb_supply_type(struct smbchg_chip *chip,
						enum power_supply_type type)
{
	int rc, current_limit_ma;
#ifdef CONFIG_HTC_BATT
	int pd_current;
#endif 
#ifdef CONFIG_HTC_BATT_WA_PCN0021
	struct timespec xtime;
#endif 

	if (type != POWER_SUPPLY_TYPE_UNKNOWN)
		chip->usb_supply_type = type;

#ifdef CONFIG_HTC_BATT_WA_PCN0021
	xtime = ktime_to_timespec(ktime_get());
	if (xtime.tv_sec > 120)
		smbchg_default_dcp_icl_ma = 1500;
#endif 

	if (chip->typec_psy && (type != POWER_SUPPLY_TYPE_USB))
		current_limit_ma = chip->typec_current_ma;
	else if (type == POWER_SUPPLY_TYPE_USB)
		current_limit_ma = DEFAULT_SDP_MA;
	else if (type == POWER_SUPPLY_TYPE_USB)
		current_limit_ma = DEFAULT_SDP_MA;
	else if (type == POWER_SUPPLY_TYPE_USB_CDP)
		current_limit_ma = DEFAULT_CDP_MA;
	else if (type == POWER_SUPPLY_TYPE_USB_HVDCP)
		current_limit_ma = smbchg_default_hvdcp_icl_ma;
	else if (type == POWER_SUPPLY_TYPE_USB_HVDCP_3)
		current_limit_ma = smbchg_default_hvdcp3_icl_ma;
#ifdef CONFIG_HTC_BATT
	else if (htc_battery_get_pd_type(&pd_current))
		current_limit_ma = pd_current;
#endif 
	else
		current_limit_ma = smbchg_default_dcp_icl_ma;

	pr_smb(PR_STATUS, "Type %d: setting mA = %d\n",
		type, current_limit_ma);
	rc = vote(chip->usb_icl_votable, PSY_ICL_VOTER, true,
				current_limit_ma);
	if (rc < 0) {
		pr_err("Couldn't vote for new USB ICL rc=%d\n", rc);
		goto out;
	}

	if (!chip->skip_usb_notification)
		power_supply_set_supply_type(chip->usb_psy, type);

	
	if (type == POWER_SUPPLY_TYPE_UNKNOWN)
		chip->usb_supply_type = type;

	
	rc = smbchg_set_optimal_charging_mode(chip, type);
	if (rc < 0)
		pr_err("Couldn't set charger optimal mode rc=%d\n", rc);

out:
	return rc;
}

#ifdef CONFIG_HTC_BATT_WA_PCN0021
#define MISC_TRIM_OPTIONS_7_0		0xF6
#define MISC_INPUT_MISSING_POLLER_EN_BIT	BIT(3)
void pmi8996_set_dcp_default(void)
{
	int aicl_result, rc;

	if(!the_chip) {
		pr_err("called before init\n");
		return;
	}

	aicl_result = smbchg_get_aicl_level_ma(the_chip);

	if ((smbchg_default_dcp_icl_ma > 1000) ||
		(aicl_result < 1000)){
		smbchg_default_dcp_icl_ma = 1500;
		return;
	}

	g_is_charger_ability_detected = false;
	smbchg_default_dcp_icl_ma = 1500;
	vote(the_chip->usb_icl_votable, PSY_ICL_VOTER, true,
				smbchg_default_dcp_icl_ma);
	pr_smb(PR_STATUS, "set to default dcp current.\n");

	msleep(300);
	if (g_is_hvdcp_detect_done){
		
		set_aicl_enable(false);
		
		rc = smbchg_sec_masked_write(the_chip,
			the_chip->misc_base + MISC_TRIM_OPTIONS_7_0,
			MISC_INPUT_MISSING_POLLER_EN_BIT, 0);
		if (rc < 0)
			pr_err("Couldn't disable input missing poller rc=%d\n", rc);
		if (delayed_work_pending(&the_chip->iusb_5v_2a_detect_work))
			cancel_delayed_work(&the_chip->iusb_5v_2a_detect_work);
		schedule_delayed_work(&the_chip->iusb_5v_2a_detect_work,
				msecs_to_jiffies(AICL_5V_2A_DETECT_DELAY_MS));
	}
}
#endif 

#define HVDCP_ADAPTER_SEL_MASK	SMB_MASK(5, 4)
#define HVDCP_5V		0x00
#define HVDCP_9V		0x10
#define USB_CMD_HVDCP_1		0x42
#define FORCE_HVDCP_2p0		BIT(3)

static int force_9v_hvdcp(struct smbchg_chip *chip)
{
	int rc;

	
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc) {
		pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);
		return rc;
	}

	
	rc = smbchg_masked_write(chip,
			chip->usb_chgpth_base + USB_CMD_HVDCP_1,
			FORCE_HVDCP_2p0, FORCE_HVDCP_2p0);
	rc |= smbchg_masked_write(chip,
			chip->usb_chgpth_base + USB_CMD_HVDCP_1,
			FORCE_HVDCP_2p0, 0);
	if (rc < 0) {
		pr_err("Couldn't force QC2.0 rc=%d\n", rc);
		return rc;
	}

	
	msleep(500);

	
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc)
		pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);

	return rc;
}

#ifdef CONFIG_HTC_BATT_WA_PCN0017
void smbchg_set_hvdcp_enable(struct smbchg_chip *chip, bool en)
{
	int rc = 0;

	if (en){
		if (!(get_kernel_flag() & KERNEL_FLAG_DISABLE_SAFETY_TIMER)
			&& (chip->htc_wa_flags & HTC_DISABLE_QC2_QC3_WA)) {
			
			rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, 0);
			if (rc < 0) {
				dev_err(chip->dev, "Couldn't set usb_chgpth cfg disable qc20 rc=%d\n", rc);
			}
		} else {
			
			rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
			if (rc < 0) {
				dev_err(chip->dev, "Couldn't set usb_chgpth cfg disable qc20 rc=%d\n", rc);
			}
		}
	} else {
		
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, 0);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set usb_chgpth cfg disable qc20 rc=%d\n", rc);
		}
	}
}

extern int workable_charging_cable(void);
#define CHK_CABLE_WORKABLE_DELAY_MS	100
#define CHK_CABLE_WORKABLE_CNT		10
#define CHK_CABLE_HVDCP_NOTIFY_MS	2500
static void smbchg_chk_cable_workable_work(struct work_struct *work)
{
	int 		cnt = 0;
	bool	hvdcp_en = true;
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";

	if(!the_chip) {
		pr_err("called before init\n");
		return;
	}

	cancel_delayed_work_sync(&the_chip->hvdcp_det_work);

	cnt = 0;
	while (cnt++ < 60){
		read_usb_type(the_chip, &usb_type_name, &usb_supply_type);
		if(usb_supply_type != POWER_SUPPLY_TYPE_USB_DCP)
                        return;
		switch(workable_charging_cable()){
			case 2:
				msleep(CHK_CABLE_WORKABLE_DELAY_MS * 10);
				hvdcp_en = false;
				break;
			case 1:
				msleep(CHK_CABLE_WORKABLE_DELAY_MS);
				hvdcp_en = true;
				if (cnt < 50)
					cnt = 50;
				break;
			default:
				hvdcp_en = false;
				cnt = 70;
				break;
		}
	}

	pr_smb(PR_STATUS, "set hvdcp %d\n", hvdcp_en);
	smbchg_set_hvdcp_enable(the_chip, hvdcp_en);
	if (hvdcp_en){
		set_aicl_enable(false);
		pmi8994_rerun_apsd();
		set_aicl_enable(true);
	}

	read_usb_type(the_chip, &usb_type_name, &usb_supply_type);
	if (!the_chip->hvdcp_not_supported &&
			(usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)) {
		schedule_delayed_work(&the_chip->hvdcp_det_work,
					msecs_to_jiffies(CHK_CABLE_HVDCP_NOTIFY_MS));
	}
}
#endif 

#define TRIM_OPTIONS_7_0		0xF6
#define INPUT_MISSING_POLLER_EN_BIT	BIT(3)
static void smbchg_hvdcp_det_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				hvdcp_det_work.work);
	int rc = 0;

	if (is_hvdcp_present(chip)) {
		if (!chip->hvdcp3_supported &&
			(chip->wa_flags & SMBCHG_HVDCP_9V_EN_WA)) {
			
			rc = force_9v_hvdcp(chip);
			if (rc)
				pr_err("could not force 9V HVDCP continuing rc=%d\n",
						rc);
		}
		pr_smb(PR_STATUS, "setting usb psy type = %d\n",
				POWER_SUPPLY_TYPE_USB_HVDCP);
		smbchg_change_usb_supply_type(chip,
				POWER_SUPPLY_TYPE_USB_HVDCP);
		if (chip->psy_registered)
			power_supply_changed(&chip->batt_psy);
		smbchg_aicl_deglitch_wa_check(chip);
#ifdef CONFIG_HTC_BATT
		g_is_hvdcp_detect_done = true;
	} else {
		if (!g_is_charger_ability_detected){
			
			set_aicl_enable(false);
			
			rc = smbchg_sec_masked_write(chip,
				chip->misc_base + TRIM_OPTIONS_7_0,
				INPUT_MISSING_POLLER_EN_BIT, 0);
			if (rc < 0)
				pr_err("Couldn't disable input missing poller rc=%d\n", rc);
			if (delayed_work_pending(&chip->iusb_5v_2a_detect_work))
				cancel_delayed_work(&chip->iusb_5v_2a_detect_work);
			schedule_delayed_work(&chip->iusb_5v_2a_detect_work,
					msecs_to_jiffies(AICL_5V_2A_DETECT_DELAY_MS));
		}
		g_is_hvdcp_detect_done = true;
#endif 
	}
}

static int set_usb_psy_dp_dm(struct smbchg_chip *chip, int state)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (!rc && !(reg & USBIN_UV_BIT) && !(reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_MISC, "overwriting state = %d with %d\n",
				state, POWER_SUPPLY_DP_DM_DPF_DMF);
		state = POWER_SUPPLY_DP_DM_DPF_DMF;
	}
	pr_smb(PR_MISC, "setting usb psy dp dm = %d\n", state);
	return power_supply_set_dp_dm(chip->usb_psy, state);
}

#define APSD_CFG		0xF5
#define AUTO_SRC_DETECT_EN_BIT	BIT(0)
#define APSD_TIMEOUT_MS		1500
static void restore_from_hvdcp_detection(struct smbchg_chip *chip)
{
	int rc;
#ifdef CONFIG_HTC_BATT
        if (!(get_kernel_flag() & KERNEL_FLAG_DISABLE_SAFETY_TIMER)
                && (chip->htc_wa_flags & HTC_DISABLE_QC2_QC3_WA)) {
		return;
	}
#endif
	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc < 0)
		pr_err("Couldn't configure HVDCP 9V rc=%d\n", rc);

	
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable HVDCP rc=%d\n", rc);

	
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, AUTO_SRC_DETECT_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable APSD rc=%d\n", rc);

	
	rc = smbchg_sec_masked_write(chip,
		chip->usb_chgpth_base + USBIN_CHGR_CFG,
		ADAPTER_ALLOWANCE_MASK, USBIN_ADAPTER_5V_UNREGULATED_9V);
	if (rc < 0)
		pr_err("Couldn't write usb allowance rc=%d\n", rc);

	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable AICL rc=%d\n", rc);

	chip->hvdcp_3_det_ignore_uv = false;
	chip->pulse_cnt = 0;
}

#ifdef CONFIG_HTC_BATT
#define INTERVAL_1_MINUTE_MS	60000
static void smbchg_usb_limit_max_current_work(struct work_struct *work)
{
	if(!the_chip) {
		pr_err("called before init\n");
		return;
	}

	pr_smb(PR_STATUS, "USB max current changed to %d\n",
		USB_MA_1500);
	vote(the_chip->usb_icl_votable, PSY_ICL_VOTER, true,
		USB_MA_1500);
}

static void smbchg_rerun_apsd_worker(struct work_struct *work)
{
	if(!the_chip) {
		pr_err("called before init\n");
		return;
	}

	if (the_chip->usb_supply_type != POWER_SUPPLY_TYPE_USB_HVDCP) {
		pr_smb(PR_STATUS, "Abort RERUN APSD. usb type: %d\n", the_chip->usb_supply_type);
		return;
	}

	pr_smb(PR_STATUS, "RERUN APSD\n");
	pmi8994_rerun_apsd();
	return;
}

#define FV_AICL_STS_BIT BIT(6)
static bool is_smbchg_hard_limit(struct smbchg_chip *chip)
{
	int rc = 0;
	u8 reg = 0;

	rc = smbchg_read(chip, &reg,
				chip->chgr_base + FV_STS, 1);
	if (rc)
		pr_err("Failed to read FV state rc=%d\n", rc);

	pr_smb(PR_MISC, "FV status = %02x, Hard Limit = %d\n", reg, ((reg & FV_AICL_STS_BIT)!=0));
	if ((reg & FV_AICL_STS_BIT) == 0)
		return false;
	else
		return true;
}

#define SMBCHG_VIN_MIN_MV 4400
#define SMBCHG_5V2A_VBATT_MV 180
static void smbchg_downgrade_iusb_work(struct work_struct *work)
{
	int current_aicl;
	int downgrade_index, target_index;
	int vbus = 0, rc = 0;
	bool hard_limit = false;
	struct smbchg_chip *chip = the_chip;

	if(!the_chip) {
		pr_err("called before init\n");
		return;
	} else {
		current_aicl = smbchg_get_aicl_level_ma(chip);
		hard_limit = is_smbchg_hard_limit(chip);
		vbus = pmi8994_get_usbin_voltage_now();
	}

	
	downgrade_index = find_smaller_in_array(
			chip->tables.usb_ilim_ma_table,
			current_aicl,
			chip->tables.usb_ilim_ma_len) - 1;
	target_index = find_smaller_in_array(
			chip->tables.usb_ilim_ma_table,
			USB_MA_1500,
			chip->tables.usb_ilim_ma_len);

	if (downgrade_index >= target_index){
		pr_smb(PR_STATUS, "Set iusb=%dmA, target=%dmA, "
		"current_aicl=%dmA, vbus=%duV, hard_limit=%d\n",
			chip->tables.usb_ilim_ma_table[downgrade_index],
			chip->tables.usb_ilim_ma_table[target_index],
			current_aicl, vbus, hard_limit);
		vote(chip->usb_icl_votable, PSY_ICL_VOTER, true,
			chip->tables.usb_ilim_ma_table[downgrade_index]);
		schedule_delayed_work(&chip->downgrade_iusb_work,
				msecs_to_jiffies(AICL_DOWNGRADE_IUSB_DELAY_MS));
		return;
	} else {
		pr_smb(PR_STATUS, "Downgrade to 1.5A done, "
			"current_aicl=%dmA, vbus=%duV, hard_limit=%d\n",
			current_aicl, vbus, hard_limit);
		
		rc = smbchg_sec_masked_write(chip,
			chip->misc_base + TRIM_OPTIONS_7_0,
			INPUT_MISSING_POLLER_EN_BIT, INPUT_MISSING_POLLER_EN_BIT);
		if (rc < 0)
			pr_err("Couldn't enable input missing poller rc=%d\n", rc);
		
		set_aicl_enable(true);
	}
}

static void smbchg_iusb_5v_2a_detect_work(struct work_struct *work)
{
	int upgrade_index, target_index;
	int rc = 0, current_aicl = 0, vbat_mv = 0, vbus = 0;
	bool hard_limit = false;
	struct smbchg_chip *chip = the_chip;

	if(!the_chip) {
		pr_err("called before init\n");
		return;
	} else {
		current_aicl = smbchg_get_aicl_level_ma(chip);
		vbat_mv = get_prop_batt_voltage_now(chip)/1000;
		vbus = pmi8994_get_usbin_voltage_now()/1000;
		hard_limit = is_smbchg_hard_limit(chip);
	}

	if (g_is_charger_ability_detected)
		return;

	
	upgrade_index = find_smaller_in_array(
			chip->tables.usb_ilim_ma_table,
			current_aicl,
			chip->tables.usb_ilim_ma_len) + 1;
	target_index = find_smaller_in_array(
			chip->tables.usb_ilim_ma_table,
			USB_MA_2000,
			chip->tables.usb_ilim_ma_len);

	if (!hard_limit || vbus < (SMBCHG_VIN_MIN_MV + 50) ||
			vbus < (vbat_mv + SMBCHG_5V2A_VBATT_MV)) {
		g_is_5v_2a_detected = true;
		g_is_charger_ability_detected = true;
		if (delayed_work_pending(&chip->downgrade_iusb_work))
			cancel_delayed_work_sync(&chip->downgrade_iusb_work);
		schedule_delayed_work(&chip->downgrade_iusb_work, 0);
		return;
	}

	if (upgrade_index <= target_index){
		pr_smb(PR_STATUS, "Set iusb=%dmA, target=%dmA, "
			"current_aicl=%dmA, vbatt=%dmV, vbus=%duV, hard_limit=%d\n",
			chip->tables.usb_ilim_ma_table[upgrade_index],
			chip->tables.usb_ilim_ma_table[target_index],
			current_aicl, vbat_mv, vbus, hard_limit);
		vote(chip->usb_icl_votable, PSY_ICL_VOTER, true,
			chip->tables.usb_ilim_ma_table[upgrade_index]);
		schedule_delayed_work(&chip->iusb_5v_2a_detect_work,
				msecs_to_jiffies(AICL_5V_2A_DETECT_DELAY_MS));
		return;
	} else {
		
		g_is_5v_2a_detected = true;
		g_is_charger_ability_detected = true;
		pr_smb(PR_STATUS, "Upgrade to 2A done, "
			"current_aicl=%dmA, vbatt=%dmV, vbus=%duV, hard_limit=%d\n\n",
		current_aicl, vbat_mv, vbus, hard_limit);
		
		rc = smbchg_sec_masked_write(chip,
			chip->misc_base + TRIM_OPTIONS_7_0,
			INPUT_MISSING_POLLER_EN_BIT, INPUT_MISSING_POLLER_EN_BIT);
		if (rc < 0)
			pr_err("Couldn't enable input missing poller rc=%d\n", rc);
		
		set_aicl_enable(true);
	}
}
#endif 

#define RESTRICTED_CHG_FCC_PERCENT	50
static int smbchg_restricted_charging(struct smbchg_chip *chip, bool enable)
{
	int current_table_index, fastchg_current;
	int rc = 0;

	current_table_index = find_smaller_in_array(
			chip->tables.usb_ilim_ma_table,
			chip->cfg_fastchg_current_ma
				* RESTRICTED_CHG_FCC_PERCENT / 100,
			chip->tables.usb_ilim_ma_len);
	fastchg_current =
		chip->tables.usb_ilim_ma_table[current_table_index];
	rc = vote(chip->fcc_votable, RESTRICTED_CHG_FCC_VOTER, enable,
			fastchg_current);

	pr_smb(PR_STATUS, "restricted_charging set to %d\n", enable);
	chip->restricted_charging = enable;

	return rc;
}

static void handle_usb_removal(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	int rc;
#ifdef CONFIG_HTC_BATT_WA_PCN0017
	int vbus_mv;
#endif 

	if (!strcmp(htc_get_bootmode(),"offmode_charging") ||
		(chip->usb_supply_type != POWER_SUPPLY_TYPE_USB)){
		if (g_rerun_apsd_ignore_uv){
#ifdef CONFIG_HTC_BATT_WA_PCN0017
			vbus_mv = pmi8994_get_usbin_voltage_now()/1000;
			pr_smb(PR_STATUS, "vbus = %dmv\n", vbus_mv);
			if (vbus_mv >= 4250){
				pr_smb(PR_STATUS, "Ignore removal for rerun APSD!!\n");
				g_is_charger_ability_detected = false;
				return;
			}
#else
			pr_smb(PR_STATUS, "Ignore removal for rerun APSD!!\n");
			return;
#endif 
		}
	}

	pr_smb(PR_STATUS, "triggered\n");
	smbchg_aicl_deglitch_wa_check(chip);
	
	if (chip->usb_ov_det)
		chip->usb_ov_det = false;
	
	if (chip->typec_psy)
		chip->typec_current_ma = 0;
	smbchg_change_usb_supply_type(chip, POWER_SUPPLY_TYPE_UNKNOWN);
	if (!chip->skip_usb_notification) {
		pr_smb(PR_MISC, "setting usb psy present = %d\n",
				chip->usb_present);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
	}
	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DPR_DMR);
	schedule_work(&chip->usb_set_online_work);
	pr_smb(PR_MISC, "setting usb psy health UNKNOWN\n");
	rc = power_supply_set_health_state(chip->usb_psy,
			POWER_SUPPLY_HEALTH_UNKNOWN);
	if (rc < 0)
		pr_smb(PR_STATUS,
			"usb psy does not allow updating prop %d rc = %d\n",
			POWER_SUPPLY_HEALTH_UNKNOWN, rc);

	if (parallel_psy && chip->parallel_charger_detected)
		power_supply_set_present(parallel_psy, false);
	if (chip->parallel.avail && chip->aicl_done_irq
			&& chip->enable_aicl_wake) {
		disable_irq_wake(chip->aicl_done_irq);
		chip->enable_aicl_wake = false;
	}
	chip->parallel.enabled_once = false;
	chip->vbat_above_headroom = false;
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			ICL_OVERRIDE_BIT, 0);
	if (rc < 0)
		pr_err("Couldn't set override rc = %d\n", rc);

	vote(chip->usb_icl_votable, WEAK_CHARGER_ICL_VOTER, false, 0);
	chip->usb_icl_delta = 0;
	vote(chip->usb_icl_votable, SW_AICL_ICL_VOTER, false, 0);
	vote(chip->aicl_deglitch_short_votable,
		HVDCP_SHORT_DEGLITCH_VOTER, false, 0);
#ifdef CONFIG_HTC_BATT
	restore_from_hvdcp_detection(chip);
	g_is_charger_ability_detected = false;
	g_is_hvdcp_detect_done = false;
	g_is_5v_2a_detected = false;
	
	if (delayed_work_pending(&chip->usb_limit_max_current))
		cancel_delayed_work_sync(&chip->usb_limit_max_current);
	if (delayed_work_pending(&chip->iusb_5v_2a_detect_work))
		cancel_delayed_work_sync(&chip->iusb_5v_2a_detect_work);
	if (delayed_work_pending(&chip->downgrade_iusb_work))
		cancel_delayed_work_sync(&chip->downgrade_iusb_work);
	
	set_aicl_enable(true);
	
	rc = smbchg_sec_masked_write(chip,
		chip->misc_base + TRIM_OPTIONS_7_0,
		INPUT_MISSING_POLLER_EN_BIT, INPUT_MISSING_POLLER_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable input missing poller rc=%d\n", rc);
#else
	if (!chip->hvdcp_not_supported)
		restore_from_hvdcp_detection(chip);
#endif 

#ifdef CONFIG_HTC_BATT_WA_PCN0017
	if (!g_rerun_apsd_ignore_uv){
		smbchg_set_hvdcp_enable(chip, false);
		g_is_cable_workable_detect = false;
		g_is_limit_IUSB = false;
	}
#endif 

#ifdef CONFIG_HTC_CHARGER
	chip->htcchg_error_flag = false;
	pmi8994_set_htc_chg_charging_enabled(chip->chg_enabled && (!chip->htcchg_error_flag));
#endif 
}

static bool is_src_detect_high(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return reg &= USBIN_SRC_DET_BIT;
}

static bool is_usbin_uv_high(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return reg &= USBIN_UV_BIT;
}

#define HVDCP_NOTIFY_MS		2500
#ifdef CONFIG_HTC_BATT
#define AICL_INIT_BIT	BIT(7)
#endif 
static void handle_usb_insertion(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	enum power_supply_type usb_supply_type;
	int rc;
	char *usb_type_name = "null";
#ifdef CONFIG_HTC_BATT
	int pd_current;

		smbchg_sec_masked_write(chip, chip->misc_base + MISC_TRIM_OPT_15_8,
				AICL_INIT_BIT, 0);
#endif 
	pr_smb(PR_STATUS, "triggered\n");
	
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	pr_smb(PR_STATUS,
		"inserted type = %d (%s)", usb_supply_type, usb_type_name);

#ifdef CONFIG_HTC_BATT_WA_PCN0017
	if (g_is_cable_workable_detect == false){
		smbchg_set_hvdcp_enable(chip, false);
		if (usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP){
			g_is_cable_workable_detect = true;
			schedule_delayed_work(&chip->chk_cable_workable_work,
						msecs_to_jiffies(CHK_CABLE_WORKABLE_DELAY_MS));
		}
	}
	g_is_charger_ability_detected = false;
#endif 

	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->typec_psy)
		update_typec_status(chip);
	smbchg_change_usb_supply_type(chip, usb_supply_type);
	if (!chip->skip_usb_notification) {
		pr_smb(PR_MISC, "setting usb psy present = %d\n",
				chip->usb_present);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
	}

	
	if (!chip->usb_ov_det) {
		pr_smb(PR_MISC, "setting usb psy health %s\n",
				chip->very_weak_charger
				? "UNSPEC_FAILURE" : "GOOD");
		rc = power_supply_set_health_state(chip->usb_psy,
				chip->very_weak_charger
				? POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
				: POWER_SUPPLY_HEALTH_GOOD);
		if (rc < 0)
			pr_smb(PR_STATUS,
				"usb psy does not allow updating prop %d rc = %d\n",
				POWER_SUPPLY_HEALTH_GOOD, rc);
	}
	schedule_work(&chip->usb_set_online_work);

#ifdef CONFIG_HTC_BATT
	if(htc_battery_get_pd_type(&pd_current)){
		g_is_charger_ability_detected = true;
		set_aicl_enable(false);
		return;
	}
#endif 

#ifndef CONFIG_HTC_BATT_WA_PCN0017
	if (!chip->hvdcp_not_supported &&
			(usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)) {
		cancel_delayed_work_sync(&chip->hvdcp_det_work);
		schedule_delayed_work(&chip->hvdcp_det_work,
					msecs_to_jiffies(HVDCP_NOTIFY_MS));
	}
#endif 

	if (parallel_psy) {
		rc = power_supply_set_present(parallel_psy, true);
		chip->parallel_charger_detected = rc ? false : true;
		if (rc)
			pr_debug("parallel-charger absent rc=%d\n", rc);
	}

	if (chip->parallel.avail && chip->aicl_done_irq
			&& !chip->enable_aicl_wake) {
		rc = enable_irq_wake(chip->aicl_done_irq);
		chip->enable_aicl_wake = true;
	}
}

void update_usb_status(struct smbchg_chip *chip, bool usb_present, bool force)
{
	mutex_lock(&chip->usb_status_lock);
	if (force) {
		chip->usb_present = usb_present;
		chip->usb_present ? handle_usb_insertion(chip)
			: handle_usb_removal(chip);
		goto unlock;
	}
	if (!chip->usb_present && usb_present) {
		chip->usb_present = usb_present;
		handle_usb_insertion(chip);
	} else if (chip->usb_present && !usb_present) {
		chip->usb_present = usb_present;
		handle_usb_removal(chip);
	}

	
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
unlock:
	mutex_unlock(&chip->usb_status_lock);
}

extern int oc_enable;
static int otg_oc_reset(struct smbchg_chip *chip)
{
	int rc;

#ifdef CONFIG_HTC_BATT
	int vbus_mv = 0;

	vbus_mv = pmi8994_get_usbin_voltage_now()/1000;
	if (vbus_mv < 3500 && !oc_enable) {
		pr_info("%s: Skip oc_reset due to abnormal vbus voltage (%d mv, oc %s)\n", __func__, vbus_mv, oc_enable?"enable":"disable");
		return 0;
	}
#endif 
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
						OTG_EN_BIT, 0);
	if (rc)
		pr_err("Failed to disable OTG rc=%d\n", rc);

	msleep(20);


	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
						OTG_EN_BIT, OTG_EN_BIT);
	if (rc)
		pr_err("Failed to re-enable OTG rc=%d\n", rc);

	return rc;
}

static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

#define AICL_IRQ_LIMIT_SECONDS	60
#define AICL_IRQ_LIMIT_COUNT	25
static void increment_aicl_count(struct smbchg_chip *chip)
{
	bool bad_charger = false;
	int max_aicl_count, rc;
	u8 reg;
	long elapsed_seconds;
	unsigned long now_seconds;

	pr_smb(PR_INTERRUPT, "aicl count c:%d dgltch:%d first:%ld\n",
			chip->aicl_irq_count, chip->aicl_deglitch_short,
			chip->first_aicl_seconds);

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (!rc)
		chip->aicl_complete = reg & AICL_STS_BIT;
	else
		chip->aicl_complete = false;

	if (chip->aicl_deglitch_short || chip->force_aicl_rerun) {
		if (!chip->aicl_irq_count)
			get_current_time(&chip->first_aicl_seconds);
		get_current_time(&now_seconds);
		elapsed_seconds = now_seconds
				- chip->first_aicl_seconds;

		if (elapsed_seconds > AICL_IRQ_LIMIT_SECONDS) {
			pr_smb(PR_INTERRUPT,
				"resetting: elp:%ld first:%ld now:%ld c=%d\n",
				elapsed_seconds, chip->first_aicl_seconds,
				now_seconds, chip->aicl_irq_count);
			chip->aicl_irq_count = 1;
			get_current_time(&chip->first_aicl_seconds);
			return;
		}
		max_aicl_count = AICL_IRQ_LIMIT_COUNT
			* (chip->parallel.avail ? 2 : 1);
		chip->aicl_irq_count++;

		if (chip->aicl_irq_count > max_aicl_count) {
			pr_smb(PR_INTERRUPT, "elp:%ld first:%ld now:%ld c=%d\n",
				elapsed_seconds, chip->first_aicl_seconds,
				now_seconds, chip->aicl_irq_count);
#ifdef CONFIG_HTC_BATT
			
			rc = vote(chip->usb_suspend_votable, USB_EN_VOTER, false, 0);
			smbchg_set_usb_current_max(chip, 150);

			pr_smb(PR_INTERRUPT, "Bad cable, set current ma: %d\n", USB_MA_150);

			
			if (delayed_work_pending(&chip->usb_limit_max_current))
				cancel_delayed_work(&chip->usb_limit_max_current);
			schedule_delayed_work(&chip->usb_limit_max_current,
				msecs_to_jiffies(INTERVAL_1_MINUTE_MS));
#else
			pr_smb(PR_INTERRUPT, "Disable AICL rerun\n");
			chip->very_weak_charger = true;
			bad_charger = true;

			
			rc = vote(chip->hw_aicl_rerun_disable_votable,
				WEAK_CHARGER_HW_AICL_VOTER, true, 0);
			if (rc < 0) {
				pr_err("Couldn't disable hw aicl rerun rc=%d\n",
					rc);
				return;
			}

			
			rc = vote(chip->usb_icl_votable, WEAK_CHARGER_ICL_VOTER,
					true, CURRENT_100_MA);
			if (rc < 0) {
				pr_err("Can't vote %d current limit rc=%d\n",
					CURRENT_100_MA, rc);
			}
#endif 
			chip->aicl_irq_count = 0;
		} else if ((get_prop_charge_type(chip) ==
				POWER_SUPPLY_CHARGE_TYPE_FAST) &&
					(reg & AICL_SUSP_BIT)) {
			chip->very_weak_charger = true;
			bad_charger = true;
		}
		if (bad_charger) {
			pr_smb(PR_MISC,
				"setting usb psy health UNSPEC_FAILURE\n");
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
			if (rc)
				pr_err("Couldn't set health on usb psy rc:%d\n",
					rc);
			schedule_work(&chip->usb_set_online_work);
		}
	}
}

static int wait_for_usbin_uv(struct smbchg_chip *chip, bool high)
{
	int rc;
	int tries = 3;
	struct completion *completion = &chip->usbin_uv_lowered;
	bool usbin_uv;

	if (high)
		completion = &chip->usbin_uv_raised;

	while (tries--) {
		rc = wait_for_completion_interruptible_timeout(
				completion,
				msecs_to_jiffies(APSD_TIMEOUT_MS));
		if (rc >= 0)
			break;
	}

	usbin_uv = is_usbin_uv_high(chip);

	if (high == usbin_uv)
		return 0;

	pr_err("usbin uv didnt go to a %s state, still at %s, tries = %d, rc = %d\n",
			high ? "risen" : "lowered",
			usbin_uv ? "high" : "low",
			tries, rc);
	return -EINVAL;
}

static int wait_for_src_detect(struct smbchg_chip *chip, bool high)
{
	int rc;
	int tries = 3;
	struct completion *completion = &chip->src_det_lowered;
	bool src_detect;

	if (high)
		completion = &chip->src_det_raised;

	while (tries--) {
		rc = wait_for_completion_interruptible_timeout(
				completion,
				msecs_to_jiffies(APSD_TIMEOUT_MS));
		if (rc >= 0)
			break;
	}

	src_detect = is_src_detect_high(chip);

	if (high == src_detect)
		return 0;

	pr_err("src detect didnt go to a %s state, still at %s, tries = %d, rc = %d\n",
			high ? "risen" : "lowered",
			src_detect ? "high" : "low",
			tries, rc);
	return -EINVAL;
}

static int fake_insertion_removal(struct smbchg_chip *chip, bool insertion)
{
	int rc;
	bool src_detect;
	bool usbin_uv;

	if (insertion) {
		reinit_completion(&chip->src_det_raised);
		reinit_completion(&chip->usbin_uv_lowered);
	} else {
		reinit_completion(&chip->src_det_lowered);
		reinit_completion(&chip->usbin_uv_raised);
	}

	
	usbin_uv = is_usbin_uv_high(chip);
	if (usbin_uv != insertion) {
		pr_err("Skip faking, usbin uv is already %d\n", usbin_uv);
		return -EINVAL;
	}

	
	src_detect = is_src_detect_high(chip);
	if (src_detect == insertion) {
		pr_err("Skip faking, src detect is already %d\n", src_detect);
		return -EINVAL;
	}

	pr_smb(PR_MISC, "Allow only %s charger\n",
			insertion ? "5-9V" : "9V only");
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USBIN_CHGR_CFG,
			ADAPTER_ALLOWANCE_MASK,
			insertion ?
			USBIN_ADAPTER_5V_9V_CONT : USBIN_ADAPTER_9V);
	if (rc < 0) {
		pr_err("Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on %s usbin uv\n",
			insertion ? "falling" : "rising");
	rc = wait_for_usbin_uv(chip, !insertion);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on %s src det\n",
			insertion ? "rising" : "falling");
	rc = wait_for_src_detect(chip, insertion);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static int smbchg_prepare_for_pulsing(struct smbchg_chip *chip)
{
	int rc = 0;
	u8 reg;

	
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		goto out;
	}

	
	msleep(500);
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		goto out;
	}

	
	pr_smb(PR_MISC, "Disable HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable HVDCP rc=%d\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "HVDCP voting for 300mA ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, true, 300);
	if (rc < 0) {
		pr_err("Couldn't vote for 300mA HVDCP ICL rc=%d\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "Disable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);

	chip->hvdcp_3_det_ignore_uv = true;
	
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0) {
		pr_err("Couldn't fake removal HVDCP Removed rc=%d\n", rc);
		goto handle_removal;
	}

	
	pr_smb(PR_MISC, "Disabling APSD\n");
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable APSD rc=%d\n", rc);
		goto out;
	}

	
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0) {
		pr_err("Couldn't fake insertion rc=%d\n", rc);
		goto handle_removal;
	}
	chip->hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_MISC, "Enable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DP0P6_DMF);
	msleep(HVDCP_NOTIFY_MS);
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 2s sleep\n");
		rc = -EINVAL;
		goto out;
	}

	smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if ((reg >> TYPE_BITS_OFFSET) != 0) {
		pr_smb(PR_MISC, "type bits set after 2s sleep - abort\n");
		rc = -EINVAL;
		goto out;
	}

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DP0P6_DM3P3);
	
	msleep(60);

	return 0;
out:
	chip->hvdcp_3_det_ignore_uv = false;
	restore_from_hvdcp_detection(chip);
	return rc;
handle_removal:
	chip->hvdcp_3_det_ignore_uv = false;
	update_usb_status(chip, 0, 0);
	return rc;
}

static int smbchg_unprepare_for_pulsing(struct smbchg_chip *chip)
{
	int rc = 0;

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DPF_DMF);

	
	pr_smb(PR_MISC, "Switch to 9V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 9V rc=%d\n", rc);
		return rc;
	}

	
	pr_smb(PR_MISC, "Enable HVDCP\n");
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0) {
		pr_err("Couldn't enable HVDCP rc=%d\n", rc);
		return rc;
	}

	
	pr_smb(PR_MISC, "Enabling APSD\n");
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, AUTO_SRC_DETECT_EN_BIT);
	if (rc < 0) {
		pr_err("Couldn't enable APSD rc=%d\n", rc);
		return rc;
	}

	
	pr_smb(PR_MISC, "Disable AICL\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable AICL rc=%d\n", rc);
		return rc;
	}

	
	chip->hvdcp_3_det_ignore_uv = true;
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0) {
		pr_err("Couldn't fake removal rc=%d\n", rc);
		goto out;
	}

	chip->parallel.enabled_once = false;

	
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0) {
		pr_err("Couldn't fake insertion rc=%d\n", rc);
		goto out;
	}
	chip->hvdcp_3_det_ignore_uv = false;

	
	pr_smb(PR_MISC, "Enable AICL\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't enable AICL rc=%d\n", rc);
		return rc;
	}

out:

	rc = vote(chip->aicl_deglitch_short_votable,
		HVDCP_SHORT_DEGLITCH_VOTER, true, 0);
	if (rc < 0)
		pr_err("Couldn't reduce aicl deglitch rc=%d\n", rc);

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	chip->hvdcp_3_det_ignore_uv = false;
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "HVDCP removed\n");
		update_usb_status(chip, 0, 0);
	}
	return rc;
}

#define USB_CMD_APSD		0x41
#define APSD_RERUN		BIT(0)
static int rerun_apsd(struct smbchg_chip *chip)
{
	int rc;

	reinit_completion(&chip->src_det_raised);
	reinit_completion(&chip->usbin_uv_lowered);
	reinit_completion(&chip->src_det_lowered);
	reinit_completion(&chip->usbin_uv_raised);

	
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + USB_CMD_APSD,
					APSD_RERUN, APSD_RERUN);
	if (rc) {
		pr_err("Couldn't re-run APSD rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on rising usbin uv\n");
	rc = wait_for_usbin_uv(chip, true);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on falling src det\n");
	rc = wait_for_src_detect(chip, false);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on falling usbin uv\n");
	rc = wait_for_usbin_uv(chip, false);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on rising src det\n");
	rc = wait_for_src_detect(chip, true);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

#ifdef CONFIG_HTC_BATT
	g_is_charger_ability_detected = false;
	g_is_hvdcp_detect_done = false;
#endif 

	return rc;
}

#define SCHG_LITE_USBIN_HVDCP_5_9V		0x8
#define SCHG_LITE_USBIN_HVDCP_5_9V_SEL_MASK	0x38
#define SCHG_LITE_USBIN_HVDCP_SEL_IDLE		BIT(3)
static bool is_hvdcp_5v_cont_mode(struct smbchg_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = smbchg_read(chip, &reg,
		chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc) {
		pr_err("Unable to read HVDCP status rc=%d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "HVDCP status = %x\n", reg);

	if (reg & SCHG_LITE_USBIN_HVDCP_SEL_IDLE) {
		rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + INPUT_STS, 1);
		if (rc) {
			pr_err("Unable to read INPUT status rc=%d\n", rc);
			return false;
		}
		pr_smb(PR_STATUS, "INPUT status = %x\n", reg);
		if ((reg & SCHG_LITE_USBIN_HVDCP_5_9V_SEL_MASK) ==
					SCHG_LITE_USBIN_HVDCP_5_9V)
			return true;
	}
	return false;
}

static int smbchg_prepare_for_pulsing_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	
	if (is_hvdcp_5v_cont_mode(chip)) {
		pr_smb(PR_MISC, "HVDCP by default is in 5V continuous mode\n");
		return 0;
	}

	
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		goto out;
	}

	
	msleep(500);
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		goto out;
	}

	pr_smb(PR_MISC, "HVDCP voting for 300mA ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, true, 300);
	if (rc < 0) {
		pr_err("Couldn't vote for 300mA HVDCP ICL rc=%d\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "Disable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);

	chip->hvdcp_3_det_ignore_uv = true;

	
	rc = rerun_apsd(chip);
	if (rc) {
		pr_err("APSD rerun failed\n");
		goto out;
	}

	chip->hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_MISC, "Enable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	msleep(HVDCP_NOTIFY_MS);
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 2s sleep\n");
		rc = -EINVAL;
		goto out;
	}

	
	if (!is_hvdcp_5v_cont_mode(chip)) {
		pr_err("HVDCP could not be set in 5V continuous mode\n");
		goto out;
	}

	return 0;
out:
	chip->hvdcp_3_det_ignore_uv = false;
	restore_from_hvdcp_detection(chip);
	return rc;
}

static int smbchg_unprepare_for_pulsing_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Forcing 9V HVDCP 2.0\n");
	rc = force_9v_hvdcp(chip);
	if (rc) {
		pr_err("Failed to force 9V HVDCP=%d\n",	rc);
		return rc;
	}

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	return rc;
}

#define CMD_HVDCP_2		0x43
#define SINGLE_INCREMENT	BIT(0)
#define SINGLE_DECREMENT	BIT(1)
static int smbchg_dp_pulse_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Increment DP\n");
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_HVDCP_2,
				SINGLE_INCREMENT, SINGLE_INCREMENT);
	if (rc)
		pr_err("Single-increment failed rc=%d\n", rc);

	return rc;
}

static int smbchg_dm_pulse_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Decrement DM\n");
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_HVDCP_2,
				SINGLE_DECREMENT, SINGLE_DECREMENT);
	if (rc)
		pr_err("Single-decrement failed rc=%d\n", rc);

	return rc;
}

static int smbchg_hvdcp3_confirmed(struct smbchg_chip *chip)
{
	int rc = 0;

	chip->parallel.enabled_once = false;

#ifdef CONFIG_HTC_BATT
	smbchg_sec_masked_write(chip, chip->misc_base + MISC_TRIM_OPT_15_8,
			AICL_INIT_BIT, AICL_INIT_BIT);
#endif 
	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	smbchg_change_usb_supply_type(chip, POWER_SUPPLY_TYPE_USB_HVDCP_3);

	return rc;
}

static int smbchg_dp_dm(struct smbchg_chip *chip, int val)
{
	int rc = 0;
	int target_icl_vote_ma;

	switch (val) {
	case POWER_SUPPLY_DP_DM_PREPARE:
#ifdef CONFIG_HTC_BATT
		if (!is_hvdcp_present(chip)
			&& (POWER_SUPPLY_TYPE_USB_HVDCP !=  chip->usb_supply_type)
			&& (POWER_SUPPLY_TYPE_USB_HVDCP_3 !=  chip->usb_supply_type)) {
#else
		if (!is_hvdcp_present(chip)) {
#endif 
			pr_err("No pulsing unless HVDCP\n");
			return -ENODEV;
		}
		if (chip->schg_version == QPNP_SCHG_LITE)
			rc = smbchg_prepare_for_pulsing_lite(chip);
		else
			rc = smbchg_prepare_for_pulsing(chip);
#ifdef CONFIG_HTC_BATT
		chip->pulse_cnt = 0;
#endif 
		break;
	case POWER_SUPPLY_DP_DM_UNPREPARE:
		if (chip->schg_version == QPNP_SCHG_LITE)
			rc = smbchg_unprepare_for_pulsing_lite(chip);
		else
			rc = smbchg_unprepare_for_pulsing(chip);
		break;
	case POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3:
		rc = smbchg_hvdcp3_confirmed(chip);
		break;
	case POWER_SUPPLY_DP_DM_DP_PULSE:
		if (chip->schg_version == QPNP_SCHG)
			rc = set_usb_psy_dp_dm(chip,
					POWER_SUPPLY_DP_DM_DP_PULSE);
		else
			rc = smbchg_dp_pulse_lite(chip);
		if (!rc)
			chip->pulse_cnt++;
		pr_smb(PR_MISC, "pulse_cnt = %d\n", chip->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_DM_PULSE:
		if (chip->schg_version == QPNP_SCHG)
			rc = set_usb_psy_dp_dm(chip,
					POWER_SUPPLY_DP_DM_DM_PULSE);
		else
			rc = smbchg_dm_pulse_lite(chip);
		if (!rc && chip->pulse_cnt)
			chip->pulse_cnt--;
		pr_smb(PR_MISC, "pulse_cnt = %d\n", chip->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_HVDCP3_SUPPORTED:
		chip->hvdcp3_supported = true;
		pr_smb(PR_MISC, "HVDCP3 supported\n");
		break;
	case POWER_SUPPLY_DP_DM_ICL_DOWN:
		chip->usb_icl_delta -= 100;
		target_icl_vote_ma = get_client_vote(chip->usb_icl_votable,
						PSY_ICL_VOTER);
		vote(chip->usb_icl_votable, SW_AICL_ICL_VOTER, true,
				target_icl_vote_ma + chip->usb_icl_delta);
		break;
	case POWER_SUPPLY_DP_DM_ICL_UP:
		chip->usb_icl_delta += 100;
		target_icl_vote_ma = get_client_vote(chip->usb_icl_votable,
						PSY_ICL_VOTER);
		vote(chip->usb_icl_votable, SW_AICL_ICL_VOTER, true,
				target_icl_vote_ma + chip->usb_icl_delta);
		break;
	default:
		break;
	}

	return rc;
}

#ifdef CONFIG_HTC_CHARGER
static int smbchg_get_htcchg_extension(struct smbchg_chip *chip)
{
	return chip->htcchg_ext_mode;
}

static int smbchg_set_htcchg_extension(struct smbchg_chip *chip, int val)
{
	int rc = 0;
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";

	switch (val) {
	case POWER_SUPPLY_HTCCHG_EXT_START:
		pr_smb(PR_STATUS, "POWER_SUPPLY_HTCCHG_EXT_START.\n");
		chip->htcchg_ext_mode = true;

		pr_smb(PR_MISC, "Disable AICL\n");
		smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
				AICL_EN_BIT, 0);

		break;

	case POWER_SUPPLY_HTCCHG_EXT_PREPARE:
		break;

	case POWER_SUPPLY_HTCCHG_EXT_UNPREPARE:
		pr_smb(PR_STATUS, "POWER_SUPPLY_HTCCHG_EXT_UNPREPARE.\n");
		if(chip->htcchg_ext_mode != true)
			break;
		if (chip->schg_version == QPNP_SCHG_LITE)
			rc = smbchg_unprepare_for_pulsing_lite(chip);
		else
			rc = smbchg_unprepare_for_pulsing(chip);
		break;

	case POWER_SUPPLY_HTCCHG_EXT_STOP:
		pr_smb(PR_STATUS, "POWER_SUPPLY_HTCCHG_EXT_STOP\n");
		if(chip->htcchg_ext_mode != true)
			break;
		chip->htcchg_ext_mode = false;
		chip->pulse_cnt = 0;
		read_usb_type(chip, &usb_type_name, &usb_supply_type);
		pr_smb(PR_STATUS, "USB type = %d (%s)\n", usb_supply_type, usb_type_name);
		if (is_hvdcp_present(chip)) {
			pr_smb(PR_STATUS, "HVDCP Present]]\n");
			usb_supply_type = POWER_SUPPLY_TYPE_USB_HVDCP;
		}
		if(is_usb_present(chip) || is_hvdcp_present(chip)) {
			if (!chip->skip_usb_notification) {
				power_supply_set_supply_type(chip->usb_psy,
						usb_supply_type);
			}
			if (chip->psy_registered) {
				power_supply_changed(&chip->batt_psy);
			}
		}
		break;

	case POWER_SUPPLY_HTCCHG_EXT_DP_PULSE:
		if(chip->htcchg_ext_mode != true)
			break;
		if (chip->schg_version == QPNP_SCHG)
			rc = set_usb_psy_dp_dm(chip,
					POWER_SUPPLY_DP_DM_DP_PULSE);
		else
			rc = smbchg_dp_pulse_lite(chip);
		if (!rc)
			chip->pulse_cnt++;
		pr_smb(PR_MISC, "POWER_SUPPLY_HTCCHG_EXT_DP_PULSE pulse_cnt = %d\n", chip->pulse_cnt);
		break;

	case POWER_SUPPLY_HTCCHG_EXT_DM_PULSE:
		if(chip->htcchg_ext_mode != true)
			break;
		if (chip->schg_version == QPNP_SCHG)
			rc = set_usb_psy_dp_dm(chip,
					POWER_SUPPLY_DP_DM_DM_PULSE);
		else
			rc = smbchg_dm_pulse_lite(chip);
		if (!rc && chip->pulse_cnt)
			chip->pulse_cnt--;
		pr_smb(PR_MISC, "POWER_SUPPLY_HTCCHG_EXT_DM_PULSE pulse_cnt = %d\n", chip->pulse_cnt);
		break;

	case POWER_SUPPLY_HTCCHG_EXT_ICL_UNSET:
		pr_smb(PR_STATUS, "Retracting vote for ICL\n");
		rc = vote(chip->usb_icl_votable, HTCCHG_ICL_VOTER,
			false, 0);
		pr_smb(PR_STATUS, "Effective ICL = %d mA\n", get_effective_result_locked(chip->usb_icl_votable));
		break;

	case POWER_SUPPLY_HTCCHG_EXT_ICL_SET_LOW:
		
		pr_smb(PR_STATUS, "Vote ICL for 100 mA\n");
		rc = vote(chip->usb_icl_votable, HTCCHG_ICL_VOTER, true, 100);
		pr_smb(PR_STATUS, "Effective ICL = %d mA\n", get_effective_result_locked(chip->usb_icl_votable));
		break;

	case POWER_SUPPLY_HTCCHG_EXT_ICL_SET_HIGH:
		
		pr_smb(PR_STATUS, "Vote ICL for 2500 mA\n");
		rc = vote(chip->usb_icl_votable, HTCCHG_ICL_VOTER, true, 2500);
		pr_smb(PR_STATUS, "Effective ICL = %d mA\n", get_effective_result_locked(chip->usb_icl_votable));
		break;

	default:
		break;
	}

	return rc;
}
#endif 

static void update_typec_capability_status(struct smbchg_chip *chip,
					const union power_supply_propval *val)
{
	int rc;

	pr_smb(PR_TYPEC, "typec capability = %dma\n", val->intval);

	if (!chip->skip_usb_notification) {
		rc = chip->usb_psy->set_property(chip->usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, val);
		if (rc)
			pr_err("typec failed to set current max rc=%d\n", rc);
	}

	pr_debug("changing ICL from %dma to %dma\n", chip->typec_current_ma,
			val->intval);
	chip->typec_current_ma = val->intval;
	smbchg_change_usb_supply_type(chip, chip->usb_supply_type);
}

static void update_typec_otg_status(struct smbchg_chip *chip, int mode,
					bool force)
{
	pr_smb(PR_TYPEC, "typec mode = %d\n", mode);

	if (mode == POWER_SUPPLY_TYPE_DFP) {
		chip->typec_dfp = true;
		power_supply_set_usb_otg(chip->usb_psy, chip->typec_dfp);
		
		set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
				get_prop_batt_status(chip));
	} else if (force || chip->typec_dfp) {
		chip->typec_dfp = false;
		power_supply_set_usb_otg(chip->usb_psy, chip->typec_dfp);
		
		set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
				get_prop_batt_status(chip));
	}
}

#define CHARGE_OUTPUT_VTG_RATIO		840
static int smbchg_get_iusb(struct smbchg_chip *chip)
{
	int rc, iusb_ua = -EINVAL;
	struct qpnp_vadc_result adc_result;

	if (!is_usb_present(chip) && !is_dc_present(chip))
		return 0;

	if (chip->vchg_vadc_dev && chip->vchg_adc_channel != -EINVAL) {
		rc = qpnp_vadc_read(chip->vchg_vadc_dev,
				chip->vchg_adc_channel, &adc_result);
		if (rc) {
			pr_smb(PR_STATUS,
				"error in VCHG (channel-%d) read rc = %d\n",
						chip->vchg_adc_channel, rc);
			return 0;
		}
		iusb_ua = div_s64(adc_result.physical * 1000,
						CHARGE_OUTPUT_VTG_RATIO);
	}

	return iusb_ua;
}

static enum power_supply_property smbchg_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_FLASH_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_INPUT_CURRENT_NOW,
	POWER_SUPPLY_PROP_FLASH_ACTIVE,
	POWER_SUPPLY_PROP_FLASH_TRIGGER,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_RERUN_AICL,
	POWER_SUPPLY_PROP_RESTRICTED_CHARGING,
#ifdef CONFIG_HTC_CHARGER
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
#endif 
};

static int smbchg_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		vote(chip->battchg_suspend_votable, BATTCHG_USER_EN_VOTER,
				!val->intval, 0);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = vote(chip->usb_suspend_votable, USER_EN_VOTER,
				!val->intval, 0);
		rc = vote(chip->dc_suspend_votable, USER_EN_VOTER,
				!val->intval, 0);
		chip->chg_enabled = val->intval;
		schedule_work(&chip->usb_set_online_work);
#ifdef CONFIG_HTC_CHARGER
		pmi8994_set_htc_chg_charging_enabled(chip->chg_enabled && (!chip->htcchg_error_flag));
#endif 
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		smbchg_system_temp_level_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = smbchg_set_fastchg_current_user(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = smbchg_float_voltage_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		rc = smbchg_safety_timer_enable(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		rc = smbchg_switch_buck_frequency(chip, val->intval);
		if (rc) {
			pr_err("Couldn't switch buck frequency, rc=%d\n", rc);
			BUG_ON(1);
		}

		rc = smbchg_otg_pulse_skip_disable(chip,
				REASON_FLASH_ENABLED, val->intval);
#ifdef CONFIG_HTC_CHARGER
		pr_smb(PR_STATUS, "POWER_SUPPLY_PROP_FLASH_ACTIVE: %d\n", val->intval);
		pmi8994_set_htc_chg_flash_active(val->intval);
#endif 
		break;
	case POWER_SUPPLY_PROP_FLASH_TRIGGER:
		chip->flash_triggered = !!val->intval;
		smbchg_icl_loop_disable_check(chip);
		break;
	case POWER_SUPPLY_PROP_FORCE_TLIM:
		rc = smbchg_force_tlim_en(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		rc = smbchg_dp_dm(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		smbchg_rerun_aicl(chip);
		break;
	case POWER_SUPPLY_PROP_RESTRICTED_CHARGING:
		rc = smbchg_restricted_charging(chip, val->intval);
		break;
#ifdef CONFIG_HTC_CHARGER
	case POWER_SUPPLY_PROP_HTCCHG_EXT:
		rc = smbchg_set_htcchg_extension(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_HTCCHG_ICL:
		if(val->intval <= 0)
			rc = vote(chip->usb_icl_votable, HTCCHG_ICL_VOTER, false, 0);
		else
			rc = vote(chip->usb_icl_votable, HTCCHG_ICL_VOTER, true, val->intval);
		pr_smb(PR_STATUS, "Effective ICL = %d mA\n", get_effective_result_locked(chip->usb_icl_votable));
		break;
	case POWER_SUPPLY_PROP_HTCCHG_FCC:
		if(val->intval <= 0)
			smbchg_set_fastchg_current_user(chip, 0);
		else
			smbchg_set_fastchg_current_user(chip, val->intval);
		pr_smb(PR_STATUS, "Effective FCC = %d mA\n", get_effective_result_locked(chip->fcc_votable));
		break;
#endif 
	case POWER_SUPPLY_PROP_CURRENT_CAPABILITY:
		if (chip->typec_psy)
			update_typec_capability_status(chip, val);
		break;
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		if (chip->typec_psy)
			update_typec_otg_status(chip, val->intval, false);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smbchg_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
	case POWER_SUPPLY_PROP_DP_DM:
	case POWER_SUPPLY_PROP_RERUN_AICL:
	case POWER_SUPPLY_PROP_RESTRICTED_CHARGING:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int smbchg_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval =
			!get_effective_result(chip->battchg_suspend_votable);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->chg_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = smbchg_float_voltage_get(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_FLASH_CURRENT_MAX:
		val->intval = smbchg_calc_max_flash_current(chip);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->fastchg_current_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = smbchg_get_aicl_level_ma(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		val->intval = (int)chip->aicl_complete;
		break;
	case POWER_SUPPLY_PROP_RESTRICTED_CHARGING:
		val->intval = (int)chip->restricted_charging;
		break;
	
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_batt_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_prop_batt_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = get_prop_batt_voltage_max_design(chip);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		val->intval = chip->safety_timer_en;
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		val->intval = chip->otg_pulse_skip_dis;
		break;
	case POWER_SUPPLY_PROP_FLASH_TRIGGER:
		val->intval = chip->flash_triggered;
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		val->intval = chip->pulse_cnt;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		val->intval = smbchg_is_input_current_limited(chip);
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		val->intval = 0;
		break;
#ifdef CONFIG_HTC_CHARGER
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = get_effective_result_locked(chip->usb_icl_votable);
		break;
	case POWER_SUPPLY_PROP_HTCCHG_EXT:
		val->intval = smbchg_get_htcchg_extension(chip);
		break;
#endif 
	case POWER_SUPPLY_PROP_INPUT_CURRENT_NOW:
		val->intval = smbchg_get_iusb(chip);
		break;
	default:
		return -EINVAL;
	}
#ifdef CONFIG_HTC_BATT
	if (prop != POWER_SUPPLY_PROP_CAPACITY)
		htc_battery_info_update(prop, val->intval);
#endif 
	return 0;
}

static char *smbchg_dc_supplicants[] = {
	"bms",
};

static enum power_supply_property smbchg_dc_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int smbchg_dc_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, dc_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = vote(chip->dc_suspend_votable, POWER_SUPPLY_EN_VOTER,
					!val->intval, 0);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = vote(chip->dc_icl_votable, USER_ICL_VOTER, true,
				val->intval / 1000);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smbchg_dc_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, dc_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = is_dc_present(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !get_effective_result(chip->dc_suspend_votable);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		
		val->intval = (smbchg_get_pwr_path(chip) == PWR_PATH_DC)
				&& (get_prop_batt_status(chip)
					== POWER_SUPPLY_STATUS_CHARGING);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->dc_max_current_ma * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smbchg_dc_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

#define HOT_BAT_HARD_BIT	BIT(0)
#define HOT_BAT_SOFT_BIT	BIT(1)
#define COLD_BAT_HARD_BIT	BIT(2)
#define COLD_BAT_SOFT_BIT	BIT(3)
#define BAT_OV_BIT		BIT(4)
#define BAT_LOW_BIT		BIT(5)
#define BAT_MISSING_BIT		BIT(6)
#define BAT_TERM_MISSING_BIT	BIT(7)
static irqreturn_t batt_hot_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_hot = !!(reg & HOT_BAT_HARD_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_cold_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_cold = !!(reg & COLD_BAT_HARD_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_BATT
#define TEMP_NORMAL_TO_WARM	470
#define TEMP_WARM_TO_NORMAL	450
#endif 
static irqreturn_t batt_warm_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_warm = !!(reg & HOT_BAT_SOFT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
#ifdef CONFIG_HTC_BATT
	if (chip->batt_warm)
		set_property_on_fg(chip, POWER_SUPPLY_PROP_WARM_TEMP, TEMP_WARM_TO_NORMAL);
	else
		set_property_on_fg(chip, POWER_SUPPLY_PROP_WARM_TEMP, TEMP_NORMAL_TO_WARM);
#endif 
	return IRQ_HANDLED;
}

static irqreturn_t batt_cool_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_cool = !!(reg & COLD_BAT_SOFT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_pres_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_present = !(reg & BAT_MISSING_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t vbat_low_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("vbat low\n");
	return IRQ_HANDLED;
}

#define CHG_COMP_SFT_BIT	BIT(3)
static irqreturn_t chg_error_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc = 0;
	u8 reg;
#ifdef CONFIG_HTC_BATT
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";

	read_usb_type(chip, &usb_type_name, &usb_supply_type);
#endif 
	pr_smb(PR_INTERRUPT, "chg-error triggered\n");

	rc = smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
	} else {
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
		if (reg & CHG_COMP_SFT_BIT)
			set_property_on_fg(chip,
					POWER_SUPPLY_PROP_SAFETY_TIMER_EXPIRED,
					1);
	}

	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
#ifdef CONFIG_HTC_BATT
	if (reg & CHG_COMP_SFT_BIT) {
		pr_info("Safety timer (%dmins) timeout with chg_src = %s\n",
			chip->safety_time, usb_type_name);
	} else {
		dump_regs(chip);
	}
#endif 

#ifdef CONFIG_HTC_CHARGER
	chip->htcchg_error_flag = true;
	pmi8994_set_htc_chg_charging_enabled(0);
#endif 

	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_BATT
static void smbchg_usb_limit_current_WA_work(struct work_struct *work)
{
	int rc = 0;
	if(!the_chip) {
		pr_err("called before init\n");
		return;
	}
	
	smbchg_sec_masked_write(the_chip, the_chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	pr_smb(PR_INTERRUPT, "AICL disabled\n");

	rc = vote(the_chip->usb_icl_votable, PSY_ICL_VOTER, true, USB_MA_1000);
	if (rc < 0)
		pr_err("Couldn't set usb current rc = %d\n", rc);
	else
		pr_smb(PR_INTERRUPT, "USB max current changed to 1A\n");


	
	msleep(50);
	smbchg_sec_masked_write(the_chip, the_chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	pr_smb(PR_INTERRUPT, "AICL re-enabled\n");
}
#endif 

static irqreturn_t fastchg_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;

	pr_smb(PR_INTERRUPT, "p2f triggered\n");
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t chg_hot_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("chg hot\n");
	smbchg_wipower_check(_chip);
	return IRQ_HANDLED;
}

static irqreturn_t chg_term_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;

	pr_smb(PR_INTERRUPT, "tcc triggered\n");
	set_property_on_fg(chip, POWER_SUPPLY_PROP_CHARGE_DONE, 1);

	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
#ifdef CONFIG_HTC_BATT
	g_is_batt_full_eoc_stop = true;
#endif

	return IRQ_HANDLED;
}

static irqreturn_t taper_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	taper_irq_en(chip, false);
	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_taper(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t recharge_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

static irqreturn_t wdog_timeout_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);
	pr_warn_ratelimited("wdog timeout rt_stat = 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

 #ifdef CONFIG_HTC_BATT
 #define POWER_OK_BIT   BIT(0)
 #define SEC_TO_MS   1000
 #define MS_TO_US   1000
 #define OCP_RELEASE_TIME_UPPER_BOUND_MS	180000
 #endif 
static irqreturn_t power_ok_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;
#ifdef CONFIG_HTC_BATT
	int power_ok;
	long diff;
	unsigned long dischg_time_ms;
	unsigned long cur_jiffies;
	static int first = 1;
	static int prev_power_ok = -1;
	static unsigned long prev_dischg_time_ms;
	static struct timeval dischg_last_time;
	struct timeval rtc_now;
#endif 

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);

#ifdef CONFIG_HTC_BATT
	if (reg & POWER_OK_BIT)
		power_ok = 1;
	else
		power_ok = 0;

	cur_jiffies = jiffies;
	do_gettimeofday(&rtc_now);

	if ((power_ok == 0) && (prev_power_ok == 1)) {
		
		dischg_last_time = rtc_now;
		dischg_time_ms = 0;
		first = 0;
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x.\n", reg);
	} else if ((power_ok == 1) && (prev_power_ok == 0) && (first == 0)){
		
		dischg_time_ms = ((rtc_now.tv_sec - dischg_last_time.tv_sec) * SEC_TO_MS) +
						((rtc_now.tv_usec - dischg_last_time.tv_usec) / MS_TO_US);

		
		
		diff = ABS(dischg_time_ms - prev_dischg_time_ms);

		if ((diff < 1200) && (dischg_time_ms <= OCP_RELEASE_TIME_UPPER_BOUND_MS)) {
			if(++g_count_same_dischg == 3) {
				g_is_limit_IUSB = true;
				g_count_same_dischg = 0;
				pr_smb(PR_INTERRUPT, "Start to limit USB current\n");
				
				schedule_work(&chip->usb_aicl_limit_current);
			}
		} else {
			g_is_limit_IUSB = false;
			g_count_same_dischg = 0;
		}
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x. "
				"prev_dischg_time_ms(%lu) dischg_time_ms(%lu) "
				"diff(%ld) count_same_dischg(%d) limit_iusb(%d)\n",
				reg,
				prev_dischg_time_ms,
				dischg_time_ms,
				diff,
				g_count_same_dischg,
				g_is_limit_IUSB);

		prev_dischg_time_ms = dischg_time_ms;
	} else
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x. prev_power_ok(%d) "
				"power_ok(%d) \n", reg, prev_power_ok, power_ok);

	prev_power_ok = power_ok;
#else
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
#endif 

	return IRQ_HANDLED;
}

#define DCIN_UNSUSPEND_DELAY_MS		1000
static irqreturn_t dcin_uv_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool dc_present = is_dc_present(chip);

	pr_smb(PR_STATUS, "chip->dc_present = %d dc_present = %d\n",
			chip->dc_present, dc_present);

	if (chip->dc_present != dc_present) {
		
		chip->dc_present = dc_present;
		if (chip->dc_psy_type != -EINVAL && chip->psy_registered)
			power_supply_changed(&chip->dc_psy);
		smbchg_charging_status_change(chip);
		smbchg_aicl_deglitch_wa_check(chip);
		chip->vbat_above_headroom = false;
	}

	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t usbin_ov_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc;
	u8 reg;
	bool usb_present;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		goto out;
	}

	
	if (reg & USBIN_OV_BIT) {
		chip->usb_ov_det = true;
		if (chip->usb_psy) {
#ifdef CONFIG_HTC_BATT
			pr_smb(PR_STATUS, "setting usb psy health OV\n");
#else
			pr_smb(PR_MISC, "setting usb psy health OV\n");
#endif 
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_OVERVOLTAGE);
			if (rc)
				pr_smb(PR_STATUS,
					"usb psy does not allow updating prop %d rc = %d\n",
					POWER_SUPPLY_HEALTH_OVERVOLTAGE, rc);
		}
	} else {
		chip->usb_ov_det = false;
		
		usb_present = is_usb_present(chip);
		update_usb_status(chip, usb_present, true);
	}
out:
	return IRQ_HANDLED;
}

#define ICL_MODE_MASK		SMB_MASK(5, 4)
#define ICL_MODE_HIGH_CURRENT	0
static irqreturn_t usbin_uv_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int aicl_level = smbchg_get_aicl_level_ma(chip);
	int rc;
#ifndef CONFIG_HTC_BATT
	bool unused;
#else
	int vbus = pmi8994_get_usbin_voltage_now();
#endif 
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc) {
		pr_err("could not read rt sts: %d", rc);
		goto out;
	}

#ifdef CONFIG_HTC_BATT
	pr_smb(PR_STATUS,
		"%s chip->usb_present = %d rt_sts = 0x%02x hvdcp_3_det_ignore_uv = %d"
		" rerun_apsd_ignore_uv = %d aicl = %d, vbus = %d\n",
		(chip->hvdcp_3_det_ignore_uv || g_rerun_apsd_ignore_uv) ? "Ignoring":"",
		chip->usb_present, reg, chip->hvdcp_3_det_ignore_uv, g_rerun_apsd_ignore_uv,
		aicl_level, vbus);
#else
	pr_smb(PR_STATUS,
		"%s chip->usb_present = %d rt_sts = 0x%02x hvdcp_3_det_ignore_uv = %d aicl = %d\n",
		chip->hvdcp_3_det_ignore_uv ? "Ignoring":"",
		chip->usb_present, reg, chip->hvdcp_3_det_ignore_uv,
		aicl_level);
#endif

	if (!(reg & USBIN_UV_BIT) && !(reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_MISC, "setting usb psy dp=f dm=f\n");
		power_supply_set_dp_dm(chip->usb_psy,
				POWER_SUPPLY_DP_DM_DPF_DMF);
	}

	if (reg & USBIN_UV_BIT)
		complete_all(&chip->usbin_uv_raised);
	else
		complete_all(&chip->usbin_uv_lowered);

#ifdef CONFIG_HTC_CHARGER
	if (chip->hvdcp_3_det_ignore_uv || g_rerun_apsd_ignore_uv)
		goto out;
#else
	if (chip->hvdcp_3_det_ignore_uv)
		goto out;
#endif 

#ifdef CONFIG_HTC_CHARGER
	if(chip->htcchg_ext_mode) {
		pr_smb(PR_STATUS, "Ignore USBIN_UV when htcchg enabled.\n");
		goto out;
	}
#endif 

	if ((reg & USBIN_UV_BIT) && (reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_STATUS, "Very weak charger detected\n");
#ifdef CONFIG_HTC_BATT
		if ((chip->usb_supply_type == POWER_SUPPLY_TYPE_USB_HVDCP) ||
			(chip->usb_supply_type == POWER_SUPPLY_TYPE_USB_HVDCP_3)){
			cancel_delayed_work_sync(&chip->force_hvdcp_work);
			schedule_delayed_work(&chip->force_hvdcp_work,
						msecs_to_jiffies(1000));
			goto out;
		}
#endif 
		chip->very_weak_charger = true;
		rc = smbchg_read(chip, &reg,
				chip->usb_chgpth_base + ICL_STS_2_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Could not read usb icl sts 2: %d\n",
					rc);
			goto out;
		}
		if ((reg & ICL_MODE_MASK) != ICL_MODE_HIGH_CURRENT) {
#ifdef CONFIG_HTC_BATT
			pr_smb(PR_STATUS, "Not let SDP into suspend. reg=0x%X\n", reg);
#else
			rc = vote(chip->usb_suspend_votable,
					WEAK_CHARGER_EN_VOTER, true, 0);
			if (rc < 0)
				pr_err("could not disable charger: %d", rc);
#endif 
		} else if (aicl_level == chip->tables.usb_ilim_ma_table[0]) {
#ifdef CONFIG_HTC_BATT
		
		
		pr_smb(PR_STATUS, "Not disable aicl rerun.\n");
#else
			rc = vote(chip->hw_aicl_rerun_disable_votable,
				WEAK_CHARGER_HW_AICL_VOTER, true, 0);
			if (rc < 0)
				pr_err("Couldn't disable hw aicl rerun rc=%d\n",
						rc);
#endif 
		}
		pr_smb(PR_MISC, "setting usb psy health UNSPEC_FAILURE\n");
		rc = power_supply_set_health_state(chip->usb_psy,
				POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
		if (rc)
			pr_err("Couldn't set health on usb psy rc:%d\n", rc);
		schedule_work(&chip->usb_set_online_work);
	}

	smbchg_wipower_check(chip);
out:
	return IRQ_HANDLED;
}

static irqreturn_t src_detect_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip);
	bool src_detect = is_src_detect_high(chip);
	int rc;

	pr_smb(PR_STATUS,
		"%s chip->usb_present = %d usb_present = %d src_detect = %d hvdcp_3_det_ignore_uv=%d\n",
		chip->hvdcp_3_det_ignore_uv ? "Ignoring":"",
		chip->usb_present, usb_present, src_detect,
		chip->hvdcp_3_det_ignore_uv);

	if (src_detect)
		complete_all(&chip->src_det_raised);
	else
		complete_all(&chip->src_det_lowered);

	if (chip->hvdcp_3_det_ignore_uv)
		goto out;

	chip->very_weak_charger = false;
	rc = vote(chip->hw_aicl_rerun_disable_votable,
		WEAK_CHARGER_HW_AICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't enable hw aicl rerun rc=%d\n", rc);

	rc = vote(chip->usb_suspend_votable, WEAK_CHARGER_EN_VOTER, false, 0);
	if (rc < 0)
		pr_err("could not enable charger: %d\n", rc);

	if (src_detect) {
		update_usb_status(chip, usb_present, 0);
	} else {
		update_usb_status(chip, 0, false);
		chip->aicl_irq_count = 0;
#ifdef CONFIG_HTC_BATT
		g_is_batt_full_eoc_stop = false;
#endif 
	}
out:
	return IRQ_HANDLED;
}


#ifdef CONFIG_HTC_BATT
#define NUM_OTG_RETRIES			1
#else
#define NUM_OTG_RETRIES			5
#endif 

#define OTG_OC_RETRY_DELAY_US		50000
static irqreturn_t otg_oc_handler(int irq, void *_chip)
{
	int rc;
	struct smbchg_chip *chip = _chip;
	s64 elapsed_us = ktime_us_delta(ktime_get(), chip->otg_enable_time);

	pr_smb(PR_INTERRUPT, "triggered\n");

	if (chip->schg_version == QPNP_SCHG_LITE) {
		pr_warn("OTG OC triggered - OTG disabled\n");
		return IRQ_HANDLED;
	}

	if (elapsed_us > OTG_OC_RETRY_DELAY_US)
		chip->otg_retries = 0;

	if (chip->otg_retries < NUM_OTG_RETRIES) {
		chip->otg_retries += 1;
		pr_smb(PR_STATUS,
			"Retrying OTG enable. Try #%d, elapsed_us %lld\n",
						chip->otg_retries, elapsed_us);
		rc = otg_oc_reset(chip);
		if (rc)
			pr_err("Failed to reset OTG OC state rc=%d\n", rc);
		chip->otg_enable_time = ktime_get();
	}
	return IRQ_HANDLED;
}

static irqreturn_t otg_fail_handler(int irq, void *_chip)
{
	pr_smb(PR_INTERRUPT, "triggered\n");
	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_BATT
#define SKIP_HARD_LIMIT_CHECK_LEVEL	70
void check_charger_ability(int aicl_level)
{
	union power_supply_propval prop = {0,};
	int rc, usb_supply_type;
	u8 reg;
	int level;

	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	level = get_prop_batt_capacity(the_chip);
	rc = the_chip->usb_psy->get_property(the_chip->usb_psy,
				POWER_SUPPLY_PROP_TYPE, &prop);
	usb_supply_type = prop.intval;
	pr_smb(PR_STATUS, "CHG_TYPE is %d, AICL is %d, level is %d\n",
		usb_supply_type, aicl_level, level);

	if (usb_supply_type != POWER_SUPPLY_TYPE_USB_DCP)
		return;

	
	rc = smbchg_read(the_chip, &reg,
				the_chip->chgr_base + FV_STS, 1);
	if (rc)
		pr_err("Failed to read FV state rc=%d\n", rc);
	pr_smb(PR_STATUS, "FV status = %02x, Hard Limit = %d\n", reg, ((reg & FV_AICL_STS_BIT)!=0));
	if (((reg & FV_AICL_STS_BIT) == 0) && (level > SKIP_HARD_LIMIT_CHECK_LEVEL)){
		if (aicl_level > USB_MA_2000){
			rc = vote(the_chip->usb_icl_votable, PSY_ICL_VOTER,
				true, USB_MA_2000);
			if (rc < 0) {
				pr_err("Couldn't vote for ICL rc=%d\n", rc);
				return;
			}
			pr_smb(PR_STATUS, "No Hard Limit, set AICL to 2000mA\n");
		} else if (aicl_level > USB_MA_1500){
			rc = vote(the_chip->usb_icl_votable, PSY_ICL_VOTER,
				true, USB_MA_1500);
			if (rc < 0) {
				pr_err("Couldn't vote for ICL rc=%d\n", rc);
				return;
			}
			pr_smb(PR_STATUS, "No Hard Limit, set AICL to 1500mA\n");
		} else {
			rc = vote(the_chip->usb_icl_votable, PSY_ICL_VOTER,
				true, USB_MA_1000);
			if (rc < 0) {
				pr_err("Couldn't vote for ICL rc=%d\n", rc);
				return;
			}
			pr_smb(PR_STATUS, "No Hard Limit, set as 1A adaptor\n");
			g_is_charger_ability_detected = true;
		}
		smbchg_rerun_aicl(the_chip);
		return;
	}

	
	if (aicl_level <= USB_MA_1400){
		rc = vote(the_chip->usb_icl_votable, PSY_ICL_VOTER,
			true, USB_MA_1000);
		if (rc < 0) {
			pr_err("Couldn't vote for ICL rc=%d\n", rc);
			return;
		}
		pr_smb(PR_STATUS, "1A adaptor detected\n");
		g_is_charger_ability_detected = true;
		smbchg_rerun_aicl(the_chip);
		return;
	}

	return;
}
#endif 
static irqreturn_t aicl_done_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip);
	int aicl_level = smbchg_get_aicl_level_ma(chip);

#ifdef CONFIG_HTC_BATT
	pr_smb(PR_INTERRUPT, "triggered, aicl: %d, vbus: %d\n", aicl_level, pmi8994_get_usbin_voltage_now());
	if (!g_is_charger_ability_detected)
		check_charger_ability(aicl_level);
#else
	pr_smb(PR_INTERRUPT, "triggered, aicl: %d\n", aicl_level);
#endif 

	increment_aicl_count(chip);

	if (usb_present)
		smbchg_parallel_usb_check_ok(chip);

	if (chip->aicl_complete)
		power_supply_changed(&chip->batt_psy);

	return IRQ_HANDLED;
}

static irqreturn_t usbid_change_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool otg_present;

	pr_smb(PR_INTERRUPT, "triggered\n");

	otg_present = is_otg_present(chip);
	if (chip->usb_psy) {
		pr_smb(PR_MISC, "setting usb psy OTG = %d\n",
				otg_present ? 1 : 0);
		power_supply_set_usb_otg(chip->usb_psy, otg_present ? 1 : 0);
	}
	if (otg_present)
		pr_smb(PR_STATUS, "OTG detected\n");

	
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));

	return IRQ_HANDLED;
}

static int determine_initial_status(struct smbchg_chip *chip)
{
	union power_supply_propval type = {0, };


	batt_pres_handler(0, chip);
	batt_hot_handler(0, chip);
	batt_warm_handler(0, chip);
	batt_cool_handler(0, chip);
	batt_cold_handler(0, chip);
	if (chip->typec_psy) {
		get_property_from_typec(chip, POWER_SUPPLY_PROP_TYPE, &type);
		update_typec_otg_status(chip, type.intval, true);
	} else {
		usbid_change_handler(0, chip);
	}
	src_detect_handler(0, chip);

	chip->usb_present = is_usb_present(chip);
	chip->dc_present = is_dc_present(chip);

	if (chip->usb_present) {
		pr_smb(PR_MISC, "setting usb psy dp=f dm=f\n");
		power_supply_set_dp_dm(chip->usb_psy,
				POWER_SUPPLY_DP_DM_DPF_DMF);
		handle_usb_insertion(chip);
	} else {
		handle_usb_removal(chip);
	}

	return 0;
}

static int prechg_time[] = {
	24,
	48,
	96,
	192,
};
static int chg_time[] = {
	192,
	384,
	768,
	1536,
};

enum bpd_type {
	BPD_TYPE_BAT_NONE,
	BPD_TYPE_BAT_ID,
	BPD_TYPE_BAT_THM,
	BPD_TYPE_BAT_THM_BAT_ID,
	BPD_TYPE_DEFAULT,
};

static const char * const bpd_label[] = {
	[BPD_TYPE_BAT_NONE]		= "bpd_none",
	[BPD_TYPE_BAT_ID]		= "bpd_id",
	[BPD_TYPE_BAT_THM]		= "bpd_thm",
	[BPD_TYPE_BAT_THM_BAT_ID]	= "bpd_thm_id",
};

static inline int get_bpd(const char *name)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(bpd_label); i++) {
		if (strcmp(bpd_label[i], name) == 0)
			return i;
	}
	return -EINVAL;
}

#define REVISION1_REG			0x0
#define DIG_MINOR			0
#define DIG_MAJOR			1
#define ANA_MINOR			2
#define ANA_MAJOR			3
#define CHGR_CFG1			0xFB
#define RECHG_THRESHOLD_SRC_BIT		BIT(1)
#define TERM_I_SRC_BIT			BIT(2)
#define TERM_SRC_FG			BIT(2)
#define CHG_INHIB_CFG_REG		0xF7
#define CHG_INHIBIT_50MV_VAL		0x00
#define CHG_INHIBIT_100MV_VAL		0x01
#define CHG_INHIBIT_200MV_VAL		0x02
#define CHG_INHIBIT_300MV_VAL		0x03
#define CHG_INHIBIT_MASK		0x03
#define USE_REGISTER_FOR_CURRENT	BIT(2)
#define CHGR_CFG2			0xFC
#define CHG_EN_SRC_BIT			BIT(7)
#define CHG_EN_POLARITY_BIT		BIT(6)
#define P2F_CHG_TRAN			BIT(5)
#define CHG_BAT_OV_ECC			BIT(4)
#define I_TERM_BIT			BIT(3)
#define AUTO_RECHG_BIT			BIT(2)
#define CHARGER_INHIBIT_BIT		BIT(0)
#define USB51_COMMAND_POL		BIT(2)
#define USB51AC_CTRL			BIT(1)
#define TR_8OR32B			0xFE
#define BUCK_8_16_FREQ_BIT		BIT(0)
#define BM_CFG				0xF3
#define BATT_MISSING_ALGO_BIT		BIT(2)
#define BMD_PIN_SRC_MASK		SMB_MASK(1, 0)
#define PIN_SRC_SHIFT			0
#define CHGR_CFG			0xFF
#define RCHG_LVL_BIT			BIT(0)
#define VCHG_EN_BIT			BIT(1)
#define VCHG_INPUT_CURRENT_BIT		BIT(3)
#define CFG_AFVC			0xF6
#define VFLOAT_COMP_ENABLE_MASK		SMB_MASK(2, 0)
#define TR_RID_REG			0xFA
#define FG_INPUT_FET_DELAY_BIT		BIT(3)
#define CHGR_CCMP_CFG			0xFA
#define JEITA_TEMP_HARD_LIMIT_BIT	BIT(5)
#define HVDCP_ADAPTER_SEL_MASK		SMB_MASK(5, 4)
#define HVDCP_ADAPTER_SEL_9V_BIT	BIT(4)
#define HVDCP_AUTH_ALG_EN_BIT		BIT(6)
#define CMD_APSD			0x41
#define APSD_RERUN_BIT			BIT(0)
#define OTG_CFG				0xF1
#define HICCUP_ENABLED_BIT		BIT(6)
#define OTG_PIN_POLARITY_BIT		BIT(4)
#define OTG_PIN_ACTIVE_LOW		BIT(4)
#define OTG_EN_CTRL_MASK		SMB_MASK(3, 2)
#define OTG_PIN_CTRL_RID_DIS		0x04
#define OTG_CMD_CTRL_RID_EN		0x08
#define AICL_ADC_BIT			BIT(6)
static void batt_ov_wa_check(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,
			CHG_BAT_OV_ECC, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return;
	}

	rc = smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read Battery RT status rc = %d\n", rc);
		return;
	}

	if (reg & BAT_OV_BIT) {
		rc = smbchg_charging_en(chip, false);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't disable charging: rc = %d\n", rc);
			return;
		}

		
		msleep(200);

		rc = smbchg_charging_en(chip, true);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't enable charging: rc = %d\n", rc);
			return;
		}
	}
}

#ifdef CONFIG_HTC_BATT
#define CHG_CURRENT_COMP_EN_MASK	SMB_MASK(1, 0)
#define DC_AICL_CFG2 0xF4
#define DC_USB_AICL_THRESHOLD_5V BIT(0)
#endif 
static int smbchg_hw_init(struct smbchg_chip *chip)
{
	int rc, i;
	u8 reg, mask;

	rc = smbchg_read(chip, chip->revision,
			chip->misc_base + REVISION1_REG, 4);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read revision rc=%d\n",
				rc);
		return rc;
	}
	pr_smb(PR_STATUS, "Charger Revision DIG: %d.%d; ANA: %d.%d\n",
			chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
			chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR]);

	
	if (!chip->hvdcp_not_supported) {
		rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
		if (rc < 0) {
			pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n",
					rc);
			return rc;
		}
	}

	if (chip->aicl_rerun_period_s > 0) {
		rc = smbchg_set_aicl_rerun_period_s(chip,
				chip->aicl_rerun_period_s);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set AICL rerun timer rc=%d\n",
					rc);
			return rc;
		}
	}

	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + TR_RID_REG,
			FG_INPUT_FET_DELAY_BIT, FG_INPUT_FET_DELAY_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable fg input fet delay rc=%d\n",
				rc);
		return rc;
	}

#ifdef CONFIG_HTC_BATT
	rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_OPTIONS_7_0,
			INPUT_MISSING_POLLER_EN_BIT, INPUT_MISSING_POLLER_EN_BIT);
#else
	rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_OPTIONS_7_0,
			INPUT_MISSING_POLLER_EN_BIT, 0);
#endif 
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable input missing poller rc=%d\n",
				rc);
		return rc;
	}

#ifdef CONFIG_HTC_BATT
	
	rc = smbchg_sec_masked_write(chip,
		chip->dc_chgpth_base + DC_AICL_CFG2, DC_USB_AICL_THRESHOLD_5V, 0);
	if (rc) {
		pr_err("Couldn't write to DC_AICL_CFG2 rc=%d\n", rc);
		return rc;
	}
#endif

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USE_REGISTER_FOR_CURRENT, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set input limit cmd rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,
			CHG_EN_SRC_BIT | CHG_EN_POLARITY_BIT | P2F_CHG_TRAN
			| I_TERM_BIT | AUTO_RECHG_BIT | CHARGER_INHIBIT_BIT,
			CHG_EN_POLARITY_BIT
			| (chip->chg_inhibit_en ? CHARGER_INHIBIT_BIT : 0)
			| (chip->iterm_disabled ? I_TERM_BIT : 0));
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_charging_en(chip, true);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable battery charging=%d\n", rc);
		return rc;
	}


	if (chip->chg_inhibit_source_fg)
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,
			TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT,
			TERM_SRC_FG | RECHG_THRESHOLD_SRC_BIT);
	else
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,
			TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
		USB51_COMMAND_POL | USB51AC_CTRL, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set usb_chgpth cfg rc=%d\n", rc);
		return rc;
	}

#ifdef CONFIG_HTC_BATT_WA_PCN0017
	smbchg_set_hvdcp_enable(chip, false);
#else
#ifdef CONFIG_HTC_BATT
	if (!(get_kernel_flag() & KERNEL_FLAG_DISABLE_SAFETY_TIMER)
		&& (chip->htc_wa_flags & HTC_DISABLE_QC2_QC3_WA)) {
		
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, 0);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set usb_chgpth cfg disable qc20 rc=%d\n", rc);
		}
	} else {
		
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, HVDCP_EN_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set usb_chgpth cfg disable qc20 rc=%d\n", rc);
		}
	}
#endif
#endif 

	check_battery_type(chip);

	
	if (chip->vfloat_mv != -EINVAL) {
		rc = smbchg_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set vfloat to %d\n", chip->vfloat_mv);
	}

	
	if (chip->fastchg_current_comp != -EINVAL) {
		rc = smbchg_fastchg_current_comp_set(chip,
			chip->fastchg_current_comp);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set fastchg current comp rc = %d\n",
				rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set fastchg current comp to %d\n",
			chip->fastchg_current_comp);
	}
#ifdef CONFIG_HTC_BATT
	
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CCMP_CFG,
			CHG_CURRENT_COMP_EN_MASK, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable chg_current_comp rc=%d\n", rc);
		return rc;
	}
#endif 

	
	if (chip->float_voltage_comp != -EINVAL) {
		rc = smbchg_float_voltage_comp_set(chip,
			chip->float_voltage_comp);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set float voltage comp rc = %d\n",
				rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set float voltage comp to %d\n",
			chip->float_voltage_comp);
	}

	
	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled) {
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			smbchg_iterm_set(chip, chip->iterm_ma);
		}
	}

	
	if (chip->safety_time != -EINVAL) {
		reg = (chip->safety_time > 0 ? 0 : SFT_TIMER_DISABLE_BIT) |
			(chip->prechg_safety_time > 0
			? 0 : PRECHG_SFT_TIMER_DISABLE_BIT);

		for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
			if (chip->safety_time <= chg_time[i]) {
				reg |= i << SAFETY_TIME_MINUTES_SHIFT;
				break;
			}
		}
		for (i = 0; i < ARRAY_SIZE(prechg_time); i++) {
			if (chip->prechg_safety_time <= prechg_time[i]) {
				reg |= i;
				break;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + SFT_CFG,
				SFT_EN_MASK | SFT_TO_MASK |
				(chip->prechg_safety_time > 0
				? PRECHG_SFT_TO_MASK : 0), reg);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set safety timer rc = %d\n",
				rc);
			return rc;
		}
		chip->safety_timer_en = true;
	} else {
		rc = smbchg_read(chip, &reg, chip->chgr_base + SFT_CFG, 1);
		if (rc < 0)
			dev_err(chip->dev, "Unable to read SFT_CFG rc = %d\n",
				rc);
		else if (!(reg & SFT_EN_MASK))
			chip->safety_timer_en = true;
	}

	
	if (chip->jeita_temp_hard_limit >= 0) {
		rc = smbchg_sec_masked_write(chip,
			chip->chgr_base + CHGR_CCMP_CFG,
			JEITA_TEMP_HARD_LIMIT_BIT,
			chip->jeita_temp_hard_limit
			? 0 : JEITA_TEMP_HARD_LIMIT_BIT);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set jeita temp hard limit rc = %d\n",
				rc);
			return rc;
		}
	}

	
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + TR_8OR32B,
			BUCK_8_16_FREQ_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set buck frequency rc = %d\n", rc);
		return rc;
	}

	
	mask =  BATT_MISSING_ALGO_BIT;
	reg = chip->bmd_algo_disabled ? BATT_MISSING_ALGO_BIT : 0;
	if (chip->bmd_pin_src < BPD_TYPE_DEFAULT) {
		mask |= BMD_PIN_SRC_MASK;
		reg |= chip->bmd_pin_src << PIN_SRC_SHIFT;
	}
	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + BM_CFG, mask, reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set batt_missing config = %d\n",
									rc);
		return rc;
	}

	if (chip->vchg_adc_channel != -EINVAL) {
		
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG,
				VCHG_INPUT_CURRENT_BIT | VCHG_EN_BIT,
				VCHG_INPUT_CURRENT_BIT | VCHG_EN_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set recharge rc = %d\n",
					rc);
			return rc;
		}
	}

	smbchg_charging_status_change(chip);

	vote(chip->usb_suspend_votable, USER_EN_VOTER, !chip->chg_enabled, 0);
	vote(chip->dc_suspend_votable, USER_EN_VOTER, !chip->chg_enabled, 0);
	
	if (chip->resume_delta_mv != -EINVAL) {

		if (!chip->chg_inhibit_source_fg) {
			if (chip->resume_delta_mv < 100)
				reg = CHG_INHIBIT_50MV_VAL;
			else if (chip->resume_delta_mv < 200)
				reg = CHG_INHIBIT_100MV_VAL;
			else if (chip->resume_delta_mv < 300)
				reg = CHG_INHIBIT_200MV_VAL;
			else
				reg = CHG_INHIBIT_300MV_VAL;

			rc = smbchg_sec_masked_write(chip,
					chip->chgr_base + CHG_INHIB_CFG_REG,
					CHG_INHIBIT_MASK, reg);
			if (rc < 0) {
				dev_err(chip->dev, "Couldn't set inhibit val rc = %d\n",
						rc);
				return rc;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + CHGR_CFG,
				RCHG_LVL_BIT,
				(chip->resume_delta_mv
				 < chip->tables.rchg_thr_mv)
				? 0 : RCHG_LVL_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set recharge rc = %d\n",
					rc);
			return rc;
		}
	}

#ifdef CONFIG_HTC_BATT
	
	if (get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON) {
		reg = TEMP_MONITOR_DISABLE_BIT |
			JEITA_TEMP_HARD_LIMIT_DISABLE_BIT | HOT_COLD_SL_COMP_BITS;
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CCMP_CFG,
			HARD_TEMP_MASK | SOFT_TEMP_MASK, reg);
				if (rc < 0) {
			dev_err(chip->dev, "Couldn't disable hw soft/hard temp monitor = %d\n",
				rc);
			return rc;
		}
	}
#endif

	
	if (chip->dc_psy_type != -EINVAL) {
		rc = vote(chip->dc_icl_votable, PSY_ICL_VOTER, true,
					chip->dc_target_current_ma);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't vote for initial DC ICL rc=%d\n", rc);
			return rc;
		}
	}


	if (chip->soft_vfloat_comp_disabled) {
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CFG_AFVC,
				VFLOAT_COMP_ENABLE_MASK, 0);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't disable soft vfloat rc = %d\n",
					rc);
			return rc;
		}
	}

#ifdef CONFIG_HTC_BATT
#define BAT_IF_TRIM7_REG    0xF7
#define CFG_750KHZ_BIT      BIT(1)
#define MISC_CFG_NTC_VOUT_REG   0xF3
#define CFG_NTC_VOUT_FSW_BIT    BIT(0)

	chip->htc_wa_flags |= HTC_FORCE_FSW_1MHZ_WA;
	rc = smbchg_sec_masked_write(chip, chip->misc_base + MISC_CFG_NTC_VOUT_REG, CFG_NTC_VOUT_FSW_BIT, CFG_NTC_VOUT_FSW_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set switching frequency multiplier rc=%d\n", rc);
		return rc;
	}
	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + BAT_IF_TRIM7_REG, CFG_750KHZ_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Cannot set switching freq: %d\n", rc);
		return rc;
	}
#endif

	rc = vote(chip->fcc_votable, BATT_TYPE_FCC_VOTER, true,
			chip->cfg_fastchg_current_ma);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't vote fastchg ma rc = %d\n", rc);
		return rc;
	}

	rc = smbchg_read(chip, &chip->original_usbin_allowance,
			chip->usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);

	if (chip->wipower_dyn_icl_avail) {
		rc = smbchg_wipower_ilim_config(chip,
				&(chip->wipower_default.entries[0]));
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set default wipower ilim = %d\n",
				rc);
			return rc;
		}
	}
	
	rc = smbchg_dc_suspend(chip, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't unsuspend dc path= %d\n", rc);
		return rc;
	}

	if (chip->force_aicl_rerun) {
		
		rc = vote(chip->hw_aicl_rerun_enable_indirect_votable,
			DEFAULT_CONFIG_HW_AICL_VOTER, true, 0);
		if (rc < 0) {
			pr_err("Couldn't vote enable hw aicl rerun rc=%d\n",
				rc);
			return rc;
		}
	}

	if (chip->schg_version == QPNP_SCHG_LITE) {
		
		rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_CFG,
					HICCUP_ENABLED_BIT, HICCUP_ENABLED_BIT);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set OTG OC config rc = %d\n",
				rc);
	}

	if (chip->otg_pinctrl) {
		
		rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_CFG,
				OTG_PIN_POLARITY_BIT | OTG_EN_CTRL_MASK,
				OTG_PIN_ACTIVE_LOW | OTG_PIN_CTRL_RID_DIS);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set OTG EN config rc = %d\n",
				rc);
			return rc;
		}
	}

	if (chip->wa_flags & SMBCHG_BATT_OV_WA)
		batt_ov_wa_check(chip);

	
	rc = smbchg_sec_masked_write(chip,
		chip->misc_base + MISC_TRIM_OPT_15_8, AICL_ADC_BIT, 0);
	if (rc)
		pr_err("Couldn't write to MISC_TRIM_OPTIONS_15_8 rc=%d\n",
			rc);

	return rc;
}

static struct of_device_id smbchg_match_table[] = {
	{
		.compatible     = "qcom,qpnp-smbcharger",
	},
	{ },
};

#define DC_MA_MIN 300
#define DC_MA_MAX 2000
#define OF_PROP_READ(chip, prop, dt_property, retval, optional)		\
do {									\
	if (retval)							\
		break;							\
	if (optional)							\
		prop = -EINVAL;						\
									\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
					"qcom," dt_property	,	\
					&prop);				\
									\
	if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
	else if (retval)						\
		dev_err(chip->dev, "Error reading " #dt_property	\
				" property rc = %d\n", rc);		\
} while (0)

#define ILIM_ENTRIES		3
#define VOLTAGE_RANGE_ENTRIES	2
#define RANGE_ENTRY		(ILIM_ENTRIES + VOLTAGE_RANGE_ENTRIES)
static int smb_parse_wipower_map_dt(struct smbchg_chip *chip,
		struct ilim_map *map, char *property)
{
	struct device_node *node = chip->dev->of_node;
	int total_elements, size;
	struct property *prop;
	const __be32 *data;
	int num, i;

	prop = of_find_property(node, property, &size);
	if (!prop) {
		dev_err(chip->dev, "%s missing\n", property);
		return -EINVAL;
	}

	total_elements = size / sizeof(int);
	if (total_elements % RANGE_ENTRY) {
		dev_err(chip->dev, "%s table not in multiple of %d, total elements = %d\n",
				property, RANGE_ENTRY, total_elements);
		return -EINVAL;
	}

	data = prop->value;
	num = total_elements / RANGE_ENTRY;
	map->entries = devm_kzalloc(chip->dev,
			num * sizeof(struct ilim_entry), GFP_KERNEL);
	if (!map->entries) {
		dev_err(chip->dev, "kzalloc failed for default ilim\n");
		return -ENOMEM;
	}
	for (i = 0; i < num; i++) {
		map->entries[i].vmin_uv =  be32_to_cpup(data++);
		map->entries[i].vmax_uv =  be32_to_cpup(data++);
		map->entries[i].icl_pt_ma =  be32_to_cpup(data++);
		map->entries[i].icl_lv_ma =  be32_to_cpup(data++);
		map->entries[i].icl_hv_ma =  be32_to_cpup(data++);
	}
	map->num = num;
	return 0;
}

static int smb_parse_wipower_dt(struct smbchg_chip *chip)
{
	int rc = 0;

	chip->wipower_dyn_icl_avail = false;

	if (!chip->vadc_dev)
		goto err;

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_default,
					"qcom,wipower-default-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-pt-ilim-map rc = %d\n",
				rc);
		goto err;
	}

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_pt,
					"qcom,wipower-pt-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-pt-ilim-map rc = %d\n",
				rc);
		goto err;
	}

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_div2,
					"qcom,wipower-div2-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-div2-ilim-map rc = %d\n",
				rc);
		goto err;
	}
	chip->wipower_dyn_icl_avail = true;
	return 0;
err:
	chip->wipower_default.num = 0;
	chip->wipower_pt.num = 0;
	chip->wipower_default.num = 0;
	if (chip->wipower_default.entries)
		devm_kfree(chip->dev, chip->wipower_default.entries);
	if (chip->wipower_pt.entries)
		devm_kfree(chip->dev, chip->wipower_pt.entries);
	if (chip->wipower_div2.entries)
		devm_kfree(chip->dev, chip->wipower_div2.entries);
	chip->wipower_default.entries = NULL;
	chip->wipower_pt.entries = NULL;
	chip->wipower_div2.entries = NULL;
	chip->vadc_dev = NULL;
	return rc;
}

#define DEFAULT_VLED_MAX_UV		3500000
#define DEFAULT_FCC_MA			2000
static int smb_parse_dt(struct smbchg_chip *chip)
{
	int rc = 0, ocp_thresh = -EINVAL;
	struct device_node *node = chip->dev->of_node;
	const char *dc_psy_type, *bpd;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	
	OF_PROP_READ(chip, ocp_thresh,
			"ibat-ocp-threshold-ua", rc, 1);
	if (ocp_thresh >= 0)
		smbchg_ibat_ocp_threshold_ua = ocp_thresh;
	OF_PROP_READ(chip, chip->iterm_ma, "iterm-ma", rc, 1);
	OF_PROP_READ(chip, chip->cfg_fastchg_current_ma,
			"fastchg-current-ma", rc, 1);
	if (chip->cfg_fastchg_current_ma == -EINVAL)
		chip->cfg_fastchg_current_ma = DEFAULT_FCC_MA;
	OF_PROP_READ(chip, chip->vfloat_mv, "float-voltage-mv", rc, 1);
	OF_PROP_READ(chip, chip->safety_time, "charging-timeout-mins", rc, 1);
	OF_PROP_READ(chip, chip->vled_max_uv, "vled-max-uv", rc, 1);
	if (chip->vled_max_uv < 0)
		chip->vled_max_uv = DEFAULT_VLED_MAX_UV;
	OF_PROP_READ(chip, chip->rpara_uohm, "rparasitic-uohm", rc, 1);
	if (chip->rpara_uohm < 0)
		chip->rpara_uohm = 0;
	OF_PROP_READ(chip, chip->prechg_safety_time, "precharging-timeout-mins",
			rc, 1);
	OF_PROP_READ(chip, chip->fastchg_current_comp, "fastchg-current-comp",
			rc, 1);
	OF_PROP_READ(chip, chip->float_voltage_comp, "float-voltage-comp",
			rc, 1);
	if (chip->safety_time != -EINVAL &&
		(chip->safety_time > chg_time[ARRAY_SIZE(chg_time) - 1])) {
		dev_err(chip->dev, "Bad charging-timeout-mins %d\n",
						chip->safety_time);
		return -EINVAL;
	}
	if (chip->prechg_safety_time != -EINVAL &&
		(chip->prechg_safety_time >
		 prechg_time[ARRAY_SIZE(prechg_time) - 1])) {
		dev_err(chip->dev, "Bad precharging-timeout-mins %d\n",
						chip->prechg_safety_time);
		return -EINVAL;
	}
	OF_PROP_READ(chip, chip->resume_delta_mv, "resume-delta-mv", rc, 1);
	OF_PROP_READ(chip, chip->parallel.min_current_thr_ma,
			"parallel-usb-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->parallel.min_9v_current_thr_ma,
			"parallel-usb-9v-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->parallel.allowed_lowering_ma,
			"parallel-allowed-lowering-ma", rc, 1);
	if (chip->parallel.min_current_thr_ma != -EINVAL
			&& chip->parallel.min_9v_current_thr_ma != -EINVAL)
		chip->parallel.avail = true;
	of_property_read_u32(chip->spmi->dev.of_node,
					"qcom,parallel-main-chg-fcc-percent",
					&smbchg_main_chg_fcc_percent);
	of_property_read_u32(chip->spmi->dev.of_node,
					"qcom,parallel-main-chg-icl-percent",
					&smbchg_main_chg_icl_percent);
	pr_smb(PR_STATUS, "parallel usb thr: %d, 9v thr: %d\n",
			chip->parallel.min_current_thr_ma,
			chip->parallel.min_9v_current_thr_ma);
	OF_PROP_READ(chip, chip->jeita_temp_hard_limit,
			"jeita-temp-hard-limit", rc, 1);
	OF_PROP_READ(chip, chip->aicl_rerun_period_s,
			"aicl-rerun-period-s", rc, 1);
	OF_PROP_READ(chip, chip->vchg_adc_channel,
			"vchg-adc-channel-id", rc, 1);

	
	chip->use_vfloat_adjustments = of_property_read_bool(node,
						"qcom,autoadjust-vfloat");
	chip->bmd_algo_disabled = of_property_read_bool(node,
						"qcom,bmd-algo-disabled");
	chip->iterm_disabled = of_property_read_bool(node,
						"qcom,iterm-disabled");
	chip->soft_vfloat_comp_disabled = of_property_read_bool(node,
					"qcom,soft-vfloat-comp-disabled");
	chip->chg_enabled = !(of_property_read_bool(node,
						"qcom,charging-disabled"));
	chip->charge_unknown_battery = of_property_read_bool(node,
						"qcom,charge-unknown-battery");
	chip->chg_inhibit_en = of_property_read_bool(node,
					"qcom,chg-inhibit-en");
	chip->chg_inhibit_source_fg = of_property_read_bool(node,
						"qcom,chg-inhibit-fg");
	chip->low_volt_dcin = of_property_read_bool(node,
					"qcom,low-volt-dcin");
	chip->force_aicl_rerun = of_property_read_bool(node,
					"qcom,force-aicl-rerun");
	chip->skip_usb_suspend_for_fake_battery = of_property_read_bool(node,
				"qcom,skip-usb-suspend-for-fake-battery");

	
	rc = of_property_read_string(chip->spmi->dev.of_node,
		"qcom,bmd-pin-src", &bpd);
	if (rc) {
		
		chip->bmd_pin_src = BPD_TYPE_DEFAULT;
		rc = 0;
	} else {
		chip->bmd_pin_src = get_bpd(bpd);
		if (chip->bmd_pin_src < 0) {
			dev_err(chip->dev,
				"failed to determine bpd schema %d\n", rc);
			return rc;
		}
	}

	
	rc = of_property_read_string(node, "qcom,dc-psy-type", &dc_psy_type);
	if (rc) {
		chip->dc_psy_type = -EINVAL;
		rc = 0;
	} else {
		if (strcmp(dc_psy_type, "Mains") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_MAINS;
		else if (strcmp(dc_psy_type, "Wireless") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIRELESS;
		else if (strcmp(dc_psy_type, "Wipower") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIPOWER;
	}
	if (chip->dc_psy_type != -EINVAL) {
		OF_PROP_READ(chip, chip->dc_target_current_ma,
				"dc-psy-ma", rc, 0);
		if (rc)
			return rc;
		if (chip->dc_target_current_ma < DC_MA_MIN
				|| chip->dc_target_current_ma > DC_MA_MAX) {
			dev_err(chip->dev, "Bad dc mA %d\n",
					chip->dc_target_current_ma);
			return -EINVAL;
		}
	}

	if (chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
		smb_parse_wipower_dt(chip);

	
	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	
	rc = of_property_read_string(node, "qcom,battery-psy-name",
						&chip->battery_psy_name);
	if (rc)
		chip->battery_psy_name = "battery";

	
	chip->cfg_chg_led_sw_ctrl =
		of_property_read_bool(node, "qcom,chg-led-sw-controls");
	chip->cfg_chg_led_support =
		of_property_read_bool(node, "qcom,chg-led-support");

	if (of_find_property(node, "qcom,thermal-mitigation",
					&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
			chip->thermal_levels,
			GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			dev_err(chip->dev, "thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	chip->skip_usb_notification
		= of_property_read_bool(node,
				"qcom,skip-usb-notification");

	chip->otg_pinctrl = of_property_read_bool(node, "qcom,otg-pinctrl");

	return 0;
}

#define SUBTYPE_REG			0x5
#define SMBCHG_CHGR_SUBTYPE		0x1
#define SMBCHG_OTG_SUBTYPE		0x8
#define SMBCHG_BAT_IF_SUBTYPE		0x3
#define SMBCHG_USB_CHGPTH_SUBTYPE	0x4
#define SMBCHG_DC_CHGPTH_SUBTYPE	0x5
#define SMBCHG_MISC_SUBTYPE		0x7
#define SMBCHG_LITE_CHGR_SUBTYPE	0x51
#define SMBCHG_LITE_OTG_SUBTYPE		0x58
#define SMBCHG_LITE_BAT_IF_SUBTYPE	0x53
#define SMBCHG_LITE_USB_CHGPTH_SUBTYPE	0x54
#define SMBCHG_LITE_DC_CHGPTH_SUBTYPE	0x55
#define SMBCHG_LITE_MISC_SUBTYPE	0x57
#define REQUEST_IRQ(chip, resource, irq_num, irq_name, irq_handler, flags, rc)\
do {									\
	irq_num = spmi_get_irq_byname(chip->spmi,			\
					resource, irq_name);		\
	if (irq_num < 0) {						\
		dev_err(chip->dev, "Unable to get " irq_name " irq\n");	\
		return -ENXIO;						\
	}								\
	rc = devm_request_threaded_irq(chip->dev,			\
			irq_num, NULL, irq_handler, flags, irq_name,	\
			chip);						\
	if (rc < 0) {							\
		dev_err(chip->dev, "Unable to request " irq_name " irq: %d\n",\
				rc);					\
		return -ENXIO;						\
	}								\
} while (0)

static int smbchg_request_irqs(struct smbchg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
							| IRQF_ONESHOT;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
		case SMBCHG_LITE_CHGR_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->chg_error_irq,
				"chg-error", chg_error_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->taper_irq,
				"chg-taper-thr", taper_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			disable_irq_nosync(chip->taper_irq);
			REQUEST_IRQ(chip, spmi_resource, chip->chg_term_irq,
				"chg-tcc-thr", chg_term_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource, chip->recharge_irq,
				"chg-rechg-thr", recharge_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->fastchg_irq,
				"chg-p2f-thr", fastchg_handler, flags, rc);
			enable_irq_wake(chip->chg_term_irq);
			enable_irq_wake(chip->chg_error_irq);
			enable_irq_wake(chip->fastchg_irq);
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
		case SMBCHG_LITE_BAT_IF_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->batt_hot_irq,
				"batt-hot", batt_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_warm_irq,
				"batt-warm", batt_warm_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_cool_irq,
				"batt-cool", batt_cool_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_cold_irq,
				"batt-cold", batt_cold_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_missing_irq,
				"batt-missing", batt_pres_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->vbat_low_irq,
				"batt-low", vbat_low_handler, flags, rc);
			enable_irq_wake(chip->batt_hot_irq);
			enable_irq_wake(chip->batt_warm_irq);
			enable_irq_wake(chip->batt_cool_irq);
			enable_irq_wake(chip->batt_cold_irq);
			enable_irq_wake(chip->batt_missing_irq);
			enable_irq_wake(chip->vbat_low_irq);
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_USB_CHGPTH_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->usbin_uv_irq,
				"usbin-uv", usbin_uv_handler,
				flags | IRQF_EARLY_RESUME, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->usbin_ov_irq,
				"usbin-ov", usbin_ov_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->src_detect_irq,
				"usbin-src-det",
				src_detect_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->aicl_done_irq,
				"aicl-done",
				aicl_done_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
				rc);
			if (chip->schg_version != QPNP_SCHG_LITE) {
				REQUEST_IRQ(chip, spmi_resource,
					chip->otg_fail_irq, "otg-fail",
					otg_fail_handler, flags, rc);
				REQUEST_IRQ(chip, spmi_resource,
					chip->otg_oc_irq, "otg-oc",
					otg_oc_handler,
					(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
					rc);
				REQUEST_IRQ(chip, spmi_resource,
					chip->usbid_change_irq, "usbid-change",
					usbid_change_handler,
					(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
					rc);
				enable_irq_wake(chip->otg_oc_irq);
				enable_irq_wake(chip->usbid_change_irq);
				enable_irq_wake(chip->otg_fail_irq);
			}
			enable_irq_wake(chip->usbin_uv_irq);
			enable_irq_wake(chip->usbin_ov_irq);
			enable_irq_wake(chip->src_detect_irq);
			if (chip->parallel.avail && chip->usb_present) {
				rc = enable_irq_wake(chip->aicl_done_irq);
				chip->enable_aicl_wake = true;
			}
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_DC_CHGPTH_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->dcin_uv_irq,
				"dcin-uv", dcin_uv_handler, flags, rc);
			enable_irq_wake(chip->dcin_uv_irq);
			break;
		case SMBCHG_MISC_SUBTYPE:
		case SMBCHG_LITE_MISC_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->power_ok_irq,
				"power-ok", power_ok_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->chg_hot_irq,
				"temp-shutdown", chg_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->wdog_timeout_irq,
				"wdog-timeout",
				wdog_timeout_handler, flags, rc);
			enable_irq_wake(chip->chg_hot_irq);
			enable_irq_wake(chip->wdog_timeout_irq);
			break;
		case SMBCHG_OTG_SUBTYPE:
			break;
		case SMBCHG_LITE_OTG_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource,
				chip->usbid_change_irq, "usbid-change",
				usbid_change_handler,
				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
				rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->otg_oc_irq, "otg-oc",
				otg_oc_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->otg_fail_irq, "otg-fail",
				otg_fail_handler, flags, rc);
			enable_irq_wake(chip->usbid_change_irq);
			enable_irq_wake(chip->otg_oc_irq);
			enable_irq_wake(chip->otg_fail_irq);
			break;
		}
	}

	return rc;
}

#define REQUIRE_BASE(chip, base, rc)					\
do {									\
	if (!rc && !chip->base) {					\
		dev_err(chip->dev, "Missing " #base "\n");		\
		rc = -EINVAL;						\
	}								\
} while (0)

static int smbchg_parse_peripherals(struct smbchg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
		case SMBCHG_LITE_CHGR_SUBTYPE:
			chip->chgr_base = resource->start;
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
		case SMBCHG_LITE_BAT_IF_SUBTYPE:
			chip->bat_if_base = resource->start;
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_USB_CHGPTH_SUBTYPE:
			chip->usb_chgpth_base = resource->start;
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_DC_CHGPTH_SUBTYPE:
			chip->dc_chgpth_base = resource->start;
			break;
		case SMBCHG_MISC_SUBTYPE:
		case SMBCHG_LITE_MISC_SUBTYPE:
			chip->misc_base = resource->start;
			break;
		case SMBCHG_OTG_SUBTYPE:
		case SMBCHG_LITE_OTG_SUBTYPE:
			chip->otg_base = resource->start;
			break;
		}
	}

	REQUIRE_BASE(chip, chgr_base, rc);
	REQUIRE_BASE(chip, bat_if_base, rc);
	REQUIRE_BASE(chip, usb_chgpth_base, rc);
	REQUIRE_BASE(chip, dc_chgpth_base, rc);
	REQUIRE_BASE(chip, misc_base, rc);

	return rc;
}

static inline void dump_reg(struct smbchg_chip *chip, u16 addr,
		const char *name)
{
	u8 reg;

	smbchg_read(chip, &reg, addr, 1);
	pr_smb(PR_DUMP, "%s - %04X = %02X\n", name, addr, reg);
}

static void dump_regs(struct smbchg_chip *chip)
{
	u16 addr;

#ifdef CONFIG_HTC_BATT
	
	for (addr = 0x4; addr <= 0xE; addr++)
			dump_reg(chip, chip->chgr_base + addr, "CHGR");
	for (addr = 0x10; addr <= 0x1B; addr++)
			dump_reg(chip, chip->chgr_base + addr, "CHGR");
	for (addr = 0xD9; addr <= 0xDF; addr++)
			dump_reg(chip, chip->chgr_base + addr, "CHGR");
	for (addr = 0xE0; addr <= 0xE7; addr++)
			dump_reg(chip, chip->chgr_base + addr, "CHGR");
	for (addr = 0xF0; addr <= 0xFF; addr++)
			dump_reg(chip, chip->chgr_base + addr, "CHGR");
	
	for (addr = 0x4; addr <= 0x5; addr++)
			dump_reg(chip, chip->otg_base + addr, "OTG");
	for (addr = 0x10; addr <= 0x1B; addr++)
			dump_reg(chip, chip->otg_base + addr, "OTG");
	for (addr = 0xD9; addr <= 0xDF; addr++)
			dump_reg(chip, chip->otg_base + addr, "OTG");
	for (addr = 0xE0; addr <= 0xEF; addr++)
			dump_reg(chip, chip->otg_base + addr, "OTG");
	for (addr = 0xF0; addr <= 0xFF; addr++)
			dump_reg(chip, chip->otg_base + addr, "OTG");
	
	for (addr = 0x4; addr <= 0x8; addr++)
			dump_reg(chip, chip->bat_if_base + addr, "BAT_IF");
	for (addr = 0x10; addr <= 0x1B; addr++)
			dump_reg(chip, chip->bat_if_base + addr, "BAT_IF");
	for (addr = 0x40; addr <= 0x43; addr++)
			dump_reg(chip, chip->bat_if_base + addr, "BAT_IF");
	for (addr = 0xD9; addr <= 0xDF; addr++)
			dump_reg(chip, chip->bat_if_base + addr, "BAT_IF");
	for (addr = 0xE0; addr <= 0xE1; addr++)
			dump_reg(chip, chip->bat_if_base + addr, "BAT_IF");
	for (addr = 0xF0; addr <= 0xFF; addr++)
			dump_reg(chip, chip->bat_if_base + addr, "BAT_IF");
	
	for (addr = 0x4; addr <= 0xF; addr++)
			dump_reg(chip, chip->usb_chgpth_base + addr, "USB");
	for (addr = 0x10; addr <= 0x1B; addr++)
			dump_reg(chip, chip->usb_chgpth_base + addr, "USB");
	for (addr = 0x40; addr <= 0x43; addr++)
			dump_reg(chip, chip->usb_chgpth_base + addr, "USB");
	for (addr = 0xD9; addr <= 0xDF; addr++)
			dump_reg(chip, chip->usb_chgpth_base + addr, "USB");
	for (addr = 0xE0; addr <= 0xE5; addr++)
			dump_reg(chip, chip->usb_chgpth_base + addr, "USB");
	for (addr = 0xF0; addr <= 0xFF; addr++)
			dump_reg(chip, chip->usb_chgpth_base + addr, "USB");
	
	for (addr = 0x4; addr <= 0x5; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC");
	for (addr = 0x10; addr <= 0x1B; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC");
	for (addr = 0xD9; addr <= 0xDF; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC");
	for (addr = 0xE0; addr <= 0xE2; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC");
	
	for (addr = 0x0; addr <= 0x8; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC");
	for (addr = 0x10; addr <= 0x1B; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC");
	for (addr = 0x40; addr <= 0x42; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC");
	for (addr = 0xD9; addr <= 0xDF; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC");
	for (addr = 0xE0; addr <= 0xEF; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC");

#else
	
	for (addr = 0xB; addr <= 0x10; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR Status");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR Config");
	
	dump_reg(chip, chip->bat_if_base + RT_STS, "BAT_IF Status");
	dump_reg(chip, chip->bat_if_base + CMD_CHG_REG, "BAT_IF Command");
	for (addr = 0xF0; addr <= 0xFB; addr++)
		dump_reg(chip, chip->bat_if_base + addr, "BAT_IF Config");
	
	for (addr = 0x7; addr <= 0x10; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB Status");
	dump_reg(chip, chip->usb_chgpth_base + CMD_IL, "USB Command");
	for (addr = 0xF0; addr <= 0xF5; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB Config");
	
	dump_reg(chip, chip->dc_chgpth_base + RT_STS, "DC Status");
	for (addr = 0xF0; addr <= 0xF6; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC Config");
	
	dump_reg(chip, chip->misc_base + IDEV_STS, "MISC Status");
	dump_reg(chip, chip->misc_base + RT_STS, "MISC Status");
	for (addr = 0xF0; addr <= 0xF3; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC CFG");
#endif 
}

static int create_debugfs_entries(struct smbchg_chip *chip)
{
	struct dentry *ent;

	chip->debug_root = debugfs_create_dir("qpnp-smbcharger", NULL);
	if (!chip->debug_root) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		return -EINVAL;
	}

	ent = debugfs_create_file("force_dcin_icl_check",
				  S_IFREG | S_IWUSR | S_IRUGO,
				  chip->debug_root, chip,
				  &force_dcin_icl_ops);
	if (!ent) {
		dev_err(chip->dev,
			"Couldn't create force dcin icl check file\n");
		return -EINVAL;
	}
	return 0;
}

static int smbchg_check_chg_version(struct smbchg_chip *chip)
{
	struct pmic_revid_data *pmic_rev_id;
	struct device_node *revid_dev_node;
	int rc;

	revid_dev_node = of_parse_phandle(chip->spmi->dev.of_node,
					"qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_err("Missing qcom,pmic-revid property - driver failed\n");
		return -EINVAL;
	}

	pmic_rev_id = get_revid_data(revid_dev_node);
	if (IS_ERR(pmic_rev_id)) {
		rc = PTR_ERR(revid_dev_node);
		if (rc != -EPROBE_DEFER)
			pr_err("Unable to get pmic_revid rc=%d\n", rc);
		return rc;
	}

	switch (pmic_rev_id->pmic_subtype) {
	case PMI8994:
		chip->wa_flags |= SMBCHG_AICL_DEGLITCH_WA
				| SMBCHG_BATT_OV_WA
				| SMBCHG_CC_ESR_WA
				| SMBCHG_RESTART_WA;
		use_pmi8994_tables(chip);
		chip->schg_version = QPNP_SCHG;
		break;
	case PMI8950:
	case PMI8937:
		chip->wa_flags |= SMBCHG_BATT_OV_WA;
		if (pmic_rev_id->rev4 < 2)  {
			chip->wa_flags |= SMBCHG_AICL_DEGLITCH_WA;
		} else	{ 
			chip->wa_flags |= SMBCHG_HVDCP_9V_EN_WA
					| SMBCHG_USB100_WA;
		}
		use_pmi8994_tables(chip);
		chip->tables.aicl_rerun_period_table =
				aicl_rerun_period_schg_lite;
		chip->tables.aicl_rerun_period_len =
			ARRAY_SIZE(aicl_rerun_period_schg_lite);

		chip->schg_version = QPNP_SCHG_LITE;
		if (pmic_rev_id->pmic_subtype == PMI8937)
			chip->hvdcp_not_supported = true;
		break;
	case PMI8996:
		chip->wa_flags |= SMBCHG_CC_ESR_WA
				| SMBCHG_FLASH_ICL_DISABLE_WA
				| SMBCHG_RESTART_WA
                                | SMBCHG_FLASH_BUCK_SWITCH_FREQ_WA;

#ifdef CONFIG_HTC_BATT
		pr_info("PMI8996 version:%d\n", pmic_rev_id->rev3);
		if (pmic_rev_id->rev3 == 0)  {
			chip->htc_wa_flags |= HTC_DISABLE_QC2_QC3_WA;
		}
#endif
		use_pmi8996_tables(chip);
		chip->schg_version = QPNP_SCHG;
		break;
	default:
		pr_err("PMIC subtype %d not supported, WA flags not set\n",
				pmic_rev_id->pmic_subtype);
	}

	pr_smb(PR_STATUS, "pmic=%s, wa_flags=0x%x, hvdcp_supported=%s\n",
			pmic_rev_id->pmic_name, chip->wa_flags,
			chip->hvdcp_not_supported ? "false" : "true");

	return 0;
}

#ifdef CONFIG_HTC_CHARGER
static int set_property_on_htcchg(struct smbchg_chip *chip,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->htcchg_psy) {
		chip->htcchg_psy = power_supply_get_by_name("htcchg");
	}

	if (!chip->htcchg_psy) {
		return -EINVAL;
	}

	ret.intval = val;
	rc = chip->htcchg_psy->set_property(chip->htcchg_psy, prop, &ret);
	if (rc)
		pr_smb(PR_STATUS, "htcchg psy does not allow updating prop %d rc = %d\n", prop, rc);
	return rc;

}

static int get_property_from_htcchg(struct smbchg_chip *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->htcchg_psy) {
		chip->htcchg_psy = power_supply_get_by_name("htcchg");
	}

	if (!chip->htcchg_psy) {
		return -EINVAL;
	}

	rc = chip->htcchg_psy->get_property(chip->htcchg_psy, prop, &ret);
	if (rc) {
		pr_smb(PR_STATUS, "htcchg psy doesn't support reading prop %d rc = %d\n", prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

int request_charger_status(enum htc_charger_request mode, void *ret_buf)
{
	int ret = -1;
	switch(mode)
	{
		case CHARGER_ABILITY_DETECT_DONE:
			pr_info("Hvdcp detect is done.\n");
			ret = g_is_charger_ability_detected ? 1 : 0;
			break;
		case CHARGER_5V_2A_DETECT_DONE:
			ret = g_is_5v_2a_detected ? 1 : 0;
			break;
		default:
			pr_info("The reqeust is not supported.\n");
			break;
	}
	return ret;
}

void set_aicl_enable(bool bEnable)
{
        if(!the_chip) {
                pr_err("called before init\n");
                return;
	}

	if(bEnable){
                pr_info("Enable AICL...\n");
                smbchg_sec_masked_write(the_chip, the_chip->usb_chgpth_base + USB_AICL_CFG,
                         AICL_EN_BIT, AICL_EN_BIT);
	}else{
		pr_info("Disable AICL...\n");
		smbchg_sec_masked_write(the_chip, the_chip->usb_chgpth_base + USB_AICL_CFG,
                        AICL_EN_BIT, 0);
	}
}


static int pmi8994_get_htc_chg_ext_mode(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	return the_chip->htcchg_ext_mode;
}

static int pmi8994_get_htc_chg_en(void)
{
	int rc = 0;
	int value = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = get_property_from_htcchg(the_chip, POWER_SUPPLY_PROP_CHARGE_ENABLED, &value);
	if (rc) {
		value = 0;
	}
	return value;
}

static int pmi8994_get_htc_chg_usb_in_isen_ma(void)
{
	int rc = 0;
	int value = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = get_property_from_htcchg(the_chip, POWER_SUPPLY_PROP_CHARGE_NOW, &value);
	if (rc) {
		value = 0;
	}
	return value;
}


static int pmi8994_get_htc_chg_usb_in_isen_adc(void)
{
	int rc = 0;
	int value = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = get_property_from_htcchg(the_chip, POWER_SUPPLY_PROP_CHARGE_NOW_RAW, &value);
	if (rc) {
		value = 0;
	}
	return value;
}

static int pmi8994_get_htc_chg_vbusdet_adc(void)
{
	int rc = 0;
	int value = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = get_property_from_htcchg(the_chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
	if (rc) {
		value = 0;
	}
	return value;
}

int pmi8994_get_htc_chg_usb_pwr_temp_adc(void)
{
	int rc = 0;
	int value = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = get_property_from_htcchg(the_chip, POWER_SUPPLY_PROP_TEMP, &value);
	if (rc) {
		value = 0;
	}
	return value;
}

static int pmi8994_set_htc_chg_charging_enabled(int value)
{
	int rc = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	pr_smb(PR_STATUS, "pmi8994_set_htc_chg_charging_enabled: %d\n", value);
	rc = set_property_on_htcchg(the_chip, POWER_SUPPLY_PROP_CHARGING_ENABLED, value);
	return rc;
}

static int pmi8994_set_htc_chg_flash_active(int value)
{
	int rc = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	pr_smb(PR_STATUS, "pmi8994_set_htc_chg_flash_active: %d\n", value);
	rc = set_property_on_htcchg(the_chip, POWER_SUPPLY_PROP_FLASH_ACTIVE, value);
	return rc;
}

#endif 

#ifdef CONFIG_HTC_BATT
void pmi8994_set_batt_health_good(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	the_chip->batt_warm = 0;
	set_property_on_fg(the_chip, POWER_SUPPLY_PROP_HEALTH, POWER_SUPPLY_HEALTH_GOOD);
}

void pmi8994_rerun_apsd(void)
{
	int rc;
	int vbus_mv;

	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	g_rerun_apsd_ignore_uv = true;

	
	if (workable_charging_cable()) {
		pr_smb(PR_MISC, "Set ADAPTOR range to 5V to 9V\n");
		smbchg_sec_masked_write(the_chip,
				the_chip->usb_chgpth_base + USBIN_CHGR_CFG,
				ADAPTER_ALLOWANCE_MASK, USBIN_ADAPTER_5V_9V_CONT);
	}

	
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(the_chip, false);
	if (rc < 0) {
		pr_err("Couldn't fake removal HVDCP Removed rc=%d\n", rc);
	}
	
	pr_smb(PR_MISC, "Disabling APSD\n");
	rc = smbchg_sec_masked_write(the_chip,
				the_chip->usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable APSD rc=%d\n", rc);
	}
	msleep(100);
	
	rc = smbchg_sec_masked_write(the_chip,
				the_chip->usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, AUTO_SRC_DETECT_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable APSD rc=%d\n", rc);
	
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(the_chip, true);
	if (rc < 0) {
		pr_err("Couldn't fake insertion rc=%d\n", rc);
	}

	g_rerun_apsd_ignore_uv = false;

	vbus_mv = pmi8994_get_usbin_voltage_now()/1000;
	if (vbus_mv < 4250){
		pr_smb(PR_STATUS, "Cable out during rerun APSD!!\n");
		update_usb_status(the_chip, false, true);
		return;
	}

#ifdef CONFIG_HTC_BATT_WA_PCN0017
	g_count_same_dischg = 0;
#endif 
}

void pmi8994_boot_update_usb_status(void)
{
	bool usb_present;
	u8 reg;

	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	smbchg_read(the_chip, &reg, the_chip->misc_base + IDEV_STS, 1);
	usb_present = ((reg & 0x10) == 0)? false: true;
	if (usb_present){
		update_usb_status(the_chip, usb_present, true);
		pr_smb(PR_STATUS, "Update usb type!");
	}

	return;
}

static void smbchg_force_hvdcp_worker(struct work_struct *work)
{
	int rc;
	int vbus_mv;

	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	if (!((the_chip->usb_supply_type == POWER_SUPPLY_TYPE_USB_HVDCP) ||
		(the_chip->usb_supply_type == POWER_SUPPLY_TYPE_USB_HVDCP_3))){
		return;
	}
	pr_smb(PR_STATUS, "D+/D- weak cable detected, force change to  QC2.0\n");

	
	rc = force_9v_hvdcp(the_chip);
	if (rc) {
		pr_err("Force 9V failed.\n");
		return;
	}

	vbus_mv = pmi8994_get_usbin_voltage_now()/1000;
	if (vbus_mv < 4250){
		pr_smb(PR_STATUS, "Cable out during force QC2.0!!\n");
		return;
	}

	
	rc = smbchg_change_usb_supply_type(the_chip, POWER_SUPPLY_TYPE_USB_HVDCP);
	if (rc) {
		pr_err("Change type to hvdcp failed.\n");
		return;
	}

	power_supply_changed(&the_chip->batt_psy);

	return;
}

static void smbchg_hvdcp_redet_worker(struct work_struct *work)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}
	restore_from_hvdcp_detection(the_chip);
}

void impedance_set_iusb_max (int current_ua, bool mode)
{
        int rc;
        rc = vote(the_chip->usb_icl_votable, USER_ICL_VOTER,
                        mode, current_ua / 1000);
        pr_smb(PR_STATUS, "set iusb max = %d\n",current_ua / 1000);
        if (rc < 0) {
                pr_err("Can't vote %d current limit rc=%d\n",
                        current_ua, rc);
        }
}

void pmi8994_set_iusb_max (int current_ua)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	rc = vote(the_chip->usb_icl_votable, PSY_ICL_VOTER,
			true, current_ua / 1000);
	pr_smb(PR_STATUS, "set iusb max = %d\n",current_ua / 1000);
	if (rc < 0) {
		pr_err("Can't vote %d current limit rc=%d\n",
			current_ua, rc);
	}
}

int pmi8994_get_usbin_voltage_now(void)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(the_chip->vadc_dev)) {
		the_chip->vadc_dev = qpnp_get_vadc(the_chip->dev, "usbin");
		if (IS_ERR(the_chip->vadc_dev))
			return PTR_ERR(the_chip->vadc_dev);
	}

	rc = qpnp_vadc_read(the_chip->vadc_dev, USBIN, &results);
	if (rc) {
		pr_err("Unable to read usbin rc=%d\n", rc);
		return 0;
	} else {
		return results.physical;
	}
}
int pmi8994_is_batt_full_eoc_stop(int *result)
{
	*result = g_is_batt_full_eoc_stop;
	return 0;
}

int pmi8994_set_float_voltage_comp (int vfloat_comp)
{
	int rc;

	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	pr_smb(PR_STATUS, "set vfloat comp = %d\n",vfloat_comp);
	rc = smbchg_float_voltage_comp_set(the_chip, vfloat_comp);
	if (rc) {
		pr_err("Unable to set float_voltage_comp rc=%d\n", rc);
		return -EINVAL;
	}

	return 0;
}

bool is_otg_enabled(void)
{
	u8 otg_status = 0;
	smbchg_read(the_chip, &otg_status, the_chip->bat_if_base + CMD_CHG_REG, 1);
	if ((otg_status & OTG_EN_BIT) != 0)
		return true;
	else
		return false;
}

#define CHG_SFT_RT_STS		BIT(3)
#define PMIC_REVID_REVISION3	0x102
#define PMIC_REVID_REVISION4	0x103
int charger_dump_all(void)
{
	u8 chgr_sts = 0, chgr_rt_sts = 0, bat_if_rt_sts = 0;
	u8 chgpth_rt_sts = 0, iusb_reg = 0, aicl_reg = 0, misc_idev_sts = 0, aicl_rerun_reg = 0;
	u8 misc_rt_sts = 0, chgr_cfg2 = 0, bat_if_cmd = 0, chgpth_cmd = 0, chgpth_cfg = 0;
	u8 chgpth_input_sts = 0, chgpth_aicl_cfg = 0;
	u8 pmic_revid_rev3 = 0, pmic_revid_rev4 = 0;
	int cc_uah = 0, rc = 0;
	int warm_temp = 0, cool_temp = 0;

	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	smbchg_read(the_chip, &chgr_sts, the_chip->chgr_base + CHGR_STS, 1);
	smbchg_read(the_chip, &chgr_rt_sts, the_chip->chgr_base + RT_STS, 1);
	smbchg_read(the_chip, &chgr_cfg2, the_chip->chgr_base + CHGR_CFG2, 1);
	smbchg_read(the_chip, &bat_if_rt_sts, the_chip->bat_if_base + RT_STS, 1);
	smbchg_read(the_chip, &bat_if_cmd, the_chip->bat_if_base + CMD_CHG_REG, 1);
	smbchg_read(the_chip, &chgpth_input_sts, the_chip->usb_chgpth_base + INPUT_STS, 1);
	smbchg_read(the_chip, &chgpth_rt_sts, the_chip->usb_chgpth_base + RT_STS, 1);
	smbchg_read(the_chip, &chgpth_cmd, the_chip->usb_chgpth_base + CMD_IL, 1);
	smbchg_read(the_chip, &chgpth_aicl_cfg, the_chip->usb_chgpth_base + USB_AICL_CFG, 1);
	smbchg_read(the_chip, &chgpth_cfg, the_chip->usb_chgpth_base + CHGPTH_CFG, 1);
	smbchg_read(the_chip, &misc_rt_sts, the_chip->misc_base + RT_STS, 1);
	smbchg_read(the_chip, &misc_idev_sts, the_chip->misc_base + IDEV_STS, 1);
	smbchg_read(the_chip, &iusb_reg, the_chip->usb_chgpth_base + IL_CFG, 1);
	smbchg_read(the_chip, &aicl_reg, the_chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	smbchg_read(the_chip, &aicl_rerun_reg, the_chip->misc_base + MISC_TRIM_OPT_15_8, 1);
	smbchg_read(the_chip, &pmic_revid_rev3, PMIC_REVID_REVISION3, 1);
	smbchg_read(the_chip, &pmic_revid_rev4, PMIC_REVID_REVISION4, 1);
	rc = get_property_from_fg(the_chip, POWER_SUPPLY_PROP_CHARGE_NOW_RAW, &cc_uah);
	if (rc) {
		
		cc_uah = 0;
	}
	rc = get_property_from_fg(the_chip, POWER_SUPPLY_PROP_WARM_TEMP, &warm_temp);
	rc = get_property_from_fg(the_chip, POWER_SUPPLY_PROP_COOL_TEMP, &cool_temp);

	if (chgr_rt_sts & CHG_SFT_RT_STS)
		pr_smb(PR_STATUS, "Charging Fail. Safety timeout.\n");

	if (chgpth_rt_sts & USBIN_OV_BIT)
		pr_smb(PR_STATUS, "Charging Fail. Over voltage.\n");

	pr_smb(PR_STATUS,"[BATT][SMBCHG] 0x100E=%02x,0x1010=%02x,0x10FC=%02x,"
		"0x1210=%02x,0x1242=%02x,0x130D=%02x,0x1310=%02x,0x1340=%02x,0x13F3=%02x,"
		"0x13F4=%02x,0x1610=%02x,0x13F2=%02x,0x1307=%02x,0x1608=%02x,0x16F5=%02x,cc=%duAh,"
		"warm_temp=%d,cool_temp=%d,pmic=rev%d.%d\n",
		chgr_sts,chgr_rt_sts,chgr_cfg2,bat_if_rt_sts,bat_if_cmd,chgpth_input_sts,
		chgpth_rt_sts,chgpth_cmd,chgpth_aicl_cfg,chgpth_cfg,misc_rt_sts,iusb_reg,aicl_reg,
		misc_idev_sts,aicl_rerun_reg,cc_uah,warm_temp,cool_temp,
		pmic_revid_rev4,pmic_revid_rev3);

	if (smbchg_debug_mask & PR_DUMP)
		dump_regs(the_chip);

	return 0;
}

int pmi8996_get_chgr_sts(void)
{
	u8 chgr_sts = 0;
	smbchg_read(the_chip, &chgr_sts, the_chip->chgr_base + CHGR_STS, 1);
	return chgr_sts;
}
#endif

#ifdef CONFIG_HTC_CHARGER
static int pm8996_adc_map_temp_voltage(int input, int *output)
{
        int i = 0;
        const struct qpnp_vadc_map_pt *pts;
        int tablesize;

        pts  = usb_adcmap_btm_threshold;
        if (pts == NULL)
                return -EINVAL;

        tablesize = ARRAY_SIZE(usb_adcmap_btm_threshold);

        while (i < tablesize) {
                if ((pts[i].y < input)) {
                        break;
                } else {
                        i++;
                }
        }

        if (i == 0) {
                *output = pts[0].x;
        } else if (i == tablesize) {
                *output = pts[tablesize-1].x;
        } else {
                
                
                *output = (( ((pts[i].x - pts[i-1].x)*
                        (input - pts[i-1].y))/
                        (pts[i].y - pts[i-1].y))+
                        pts[i-1].x);
        }

        return 0;
}

int pm8996_get_usb_temp(void)
{
	int usb_pwr_temp_uv, usb_pwr_temp;

	usb_pwr_temp_uv = pmi8994_get_htc_chg_usb_pwr_temp_adc();
	pm8996_adc_map_temp_voltage(usb_pwr_temp_uv/1000,&usb_pwr_temp);

	return usb_pwr_temp;
}

#endif 

#ifdef CONFIG_HTC_BATT
int pmi8994_charger_get_attr_text(char *buf, int size)
{
	int len = 0;
	u16 addr;
	u8 reg;
	int vbat_mv, ibat_ma, soc, batt_temp, vbus_uv;
	int health, present, charger_type, usb_online, dc_online;
	u8 chgr_sts = 0, chgr_rt_sts = 0, bat_if_rt_sts = 0;
	u8 chgpth_rt_sts = 0, iusb_reg = 0, aicl_reg = 0;
	int batt_soc, system_soc, aicl_result, cc_uah, rc;
	int warm_temp = 0, cool_temp = 0;
	u8 pmic_revid_rev3 = 0, pmic_revid_rev4 = 0;
#ifdef CONFIG_HTC_CHARGER
	int htc_chg_mode, htc_chg_en, usbin_isen_ma, usbin_isen_uv, vbusdet_uv, usb_pwr_temp_uv,usb_pwr_temp;
#endif 

	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	vbat_mv = get_prop_batt_voltage_now(the_chip)/1000;
	ibat_ma = get_prop_batt_current_now(the_chip)/1000;
	soc = get_prop_batt_capacity(the_chip);
	vbus_uv = pmi8994_get_usbin_voltage_now();
	batt_temp = get_prop_batt_temp(the_chip);
	health = get_prop_batt_health(the_chip);
	present = get_prop_batt_present(the_chip);
	charger_type = get_prop_charge_type(the_chip);
	usb_online = is_usb_present(the_chip);
	dc_online = is_dc_present(the_chip);
	batt_soc = get_prop_batt_soc(the_chip);
	system_soc = get_prop_system_soc(the_chip);
	aicl_result = smbchg_get_aicl_level_ma(the_chip);

	smbchg_read(the_chip, &chgr_sts, the_chip->chgr_base + CHGR_STS, 1);
	smbchg_read(the_chip, &chgr_rt_sts, the_chip->chgr_base + RT_STS, 1);
	smbchg_read(the_chip, &bat_if_rt_sts, the_chip->bat_if_base + RT_STS, 1);
	smbchg_read(the_chip, &chgpth_rt_sts,the_chip->usb_chgpth_base + RT_STS, 1);

	smbchg_read(the_chip, &iusb_reg, the_chip->usb_chgpth_base + IL_CFG, 1);
	smbchg_read(the_chip, &aicl_reg, the_chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	rc = get_property_from_fg(the_chip, POWER_SUPPLY_PROP_CHARGE_NOW_RAW, &cc_uah);
	if (rc)
		cc_uah = 0;
	rc = get_property_from_fg(the_chip, POWER_SUPPLY_PROP_WARM_TEMP, &warm_temp);
	rc = get_property_from_fg(the_chip, POWER_SUPPLY_PROP_COOL_TEMP, &cool_temp);
	smbchg_read(the_chip, &pmic_revid_rev3, PMIC_REVID_REVISION3, 1);
	smbchg_read(the_chip, &pmic_revid_rev4, PMIC_REVID_REVISION4, 1);

	len += scnprintf(buf + len, size -len,
			"SOC(%%): %d;\n"
			"VBAT(mV): %d;\n"
			"IBAT(mA): %d;\n"
			"VBUS(uV): %d;\n"
			"BATT_TEMP: %d;\n"
			"HEALTH: %d;\n"
			"BATT_PRESENT(bool): %d;\n"
			"CHARGE_TYPE: %d;\n"
			"USB_ONLINE: %d;\n"
			"DC_ONLINE: %d;\n"
			"CHGR_STS: 0x%02x;\n"
			"CHGR_RT_STS: 0x%02x;\n"
			"IF_RT_STS: 0x%02x;\n"
			"USB_RT_STS: 0x%02x;\n"
			"USB_IL_CFG: 0x%02x;\n"
			"USB_ICL_STS_1_REG: 0x%02x;\n"
			"BATT_SOC: %d\n"
			"SYSTEM_SOC: %d\n"
			"CC_uAh: %d\n"
			"AICL_RESLULT: %d\n"
			"warm_temp: %d\n"
			"cool_temp: %d\n"
			"PMIC: rev%d.%d\n",
			soc, vbat_mv, ibat_ma, vbus_uv, batt_temp, health, present,
			charger_type, usb_online, dc_online,
			chgr_sts, chgr_rt_sts, bat_if_rt_sts, chgpth_rt_sts,
			iusb_reg, aicl_reg, batt_soc, system_soc, cc_uah, aicl_result,
			warm_temp, cool_temp, pmic_revid_rev4, pmic_revid_rev3);

#ifdef CONFIG_HTC_CHARGER
	htc_chg_mode = pmi8994_get_htc_chg_ext_mode();
	htc_chg_en = pmi8994_get_htc_chg_en();
	usbin_isen_ma = pmi8994_get_htc_chg_usb_in_isen_ma();
	usbin_isen_uv = pmi8994_get_htc_chg_usb_in_isen_adc();
	vbusdet_uv = 	pmi8994_get_htc_chg_vbusdet_adc();
	usb_pwr_temp_uv = pmi8994_get_htc_chg_usb_pwr_temp_adc();
	pm8996_adc_map_temp_voltage(usb_pwr_temp_uv/1000,&usb_pwr_temp);

	len += scnprintf(buf + len, size -len,
			"HTC_CHG_MODE: %d;\n"
			"HTC_CHG_EN: %d;\n"
			"USB_IN_ISEN(mA): %d;\n"
			"USB_IN_ISEN(uV): %d;\n"
			"VBUSDET(uV): %d;\n"
			"USB_PWR_TEMP(uV): %d;\n"
			"USB_PWR_TEMP(degree): %d\n",
			htc_chg_mode, htc_chg_en, usbin_isen_ma, usbin_isen_uv, vbusdet_uv, usb_pwr_temp_uv,usb_pwr_temp);
#endif 

	
	len += scnprintf(buf + len, size -len, "CHGR_STS ------\n");
	for (addr = 0xB; addr <= 0x10; addr++){
		smbchg_read(the_chip, &reg, the_chip->chgr_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "0x%04x: 0x%02x\n", the_chip->chgr_base + addr, reg);
	}
	len += scnprintf(buf + len, size -len, "CHGR_CF ------\n");
	for (addr = 0xF0; addr <= 0xFF; addr++){
		smbchg_read(the_chip, &reg, the_chip->chgr_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "0x%04x: 0x%02x\n", the_chip->chgr_base + addr, reg);
	}
	
	smbchg_read(the_chip, &reg, the_chip->bat_if_base + RT_STS, 1);
	len += scnprintf(buf + len, size -len, "BAT_IF_STS(0x%04x): 0x%02x\n", the_chip->bat_if_base + RT_STS, reg);
	smbchg_read(the_chip, &reg, the_chip->bat_if_base + CMD_CHG_REG, 1);
	len += scnprintf(buf + len, size -len, "BAT_IF_Command(0x%04x): 0x%02x\n", the_chip->bat_if_base + CMD_CHG_REG, reg);

	len += scnprintf(buf + len, size -len, "BAT_IF_CF ------\n");
	for (addr = 0xF0; addr <= 0xFB; addr++){
		smbchg_read(the_chip, &reg, the_chip->bat_if_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "0x%04x: 0x%02x\n", the_chip->bat_if_base + addr, reg);
	}
	
	len += scnprintf(buf + len, size -len, "USB_STS ------\n");
	for (addr = 0x7; addr <= 0x10; addr++){
		smbchg_read(the_chip, &reg, the_chip->usb_chgpth_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "0x%04x: 0x%02x\n", the_chip->usb_chgpth_base + addr, reg);
	}
	smbchg_read(the_chip, &reg, the_chip->usb_chgpth_base + CMD_IL, 1);
	len += scnprintf(buf + len, size -len, "USB_Command(0x%04x): 0x%02x\n", the_chip->usb_chgpth_base + CMD_IL, reg);
	len += scnprintf(buf + len, size -len, "USB_CF ------\n");
	for (addr = 0xF0; addr <= 0xF5; addr++){
		smbchg_read(the_chip, &reg, the_chip->usb_chgpth_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "0x%04x: 0x%02x\n", the_chip->usb_chgpth_base + addr, reg);
	}
	
	smbchg_read(the_chip, &reg, the_chip->dc_chgpth_base + RT_STS, 1);
	len += scnprintf(buf + len, size -len, "DC_Status(0x%04x): 0x%02x\n", the_chip->dc_chgpth_base + RT_STS, reg);
	len += scnprintf(buf + len, size -len, "DC_CF ------\n");
	for (addr = 0xF0; addr <= 0xF6; addr++){
		smbchg_read(the_chip, &reg, the_chip->dc_chgpth_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "0x%04x: 0x%02x\n", the_chip->dc_chgpth_base + addr, reg);
	}
	
	smbchg_read(the_chip, &reg, the_chip->misc_base + IDEV_STS, 1);
	len += scnprintf(buf + len, size -len, "MISC_STS(0x%04x): 0x%02x\n", the_chip->misc_base + IDEV_STS, reg);
	smbchg_read(the_chip, &reg, the_chip->misc_base + RT_STS, 1);
	len += scnprintf(buf + len, size -len, "MISC_RT_STS(0x%04x): 0x%02x\n", the_chip->misc_base + RT_STS, reg);
	len += scnprintf(buf + len, size -len, "MISC_CF ------\n");
	for (addr = 0xF0; addr <= 0xFF; addr++){
		smbchg_read(the_chip, &reg, the_chip->misc_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "0x%04x: 0x%02x\n", the_chip->misc_base + addr, reg);
	}
	return len;
}

#endif 

static void rerun_hvdcp_det_if_necessary(struct smbchg_chip *chip)
{
	enum power_supply_type usb_supply_type;
	char *usb_type_name;
	int rc;

	if (!(chip->wa_flags & SMBCHG_RESTART_WA))
		return;

	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	if (usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP
		&& !is_hvdcp_present(chip)) {
		pr_smb(PR_STATUS, "DCP found rerunning APSD\n");
		rc = vote(chip->usb_icl_votable,
				CHG_SUSPEND_WORKAROUND_ICL_VOTER, true, 300);
		if (rc < 0)
			pr_err("Couldn't vote for 300mA for suspend wa, going ahead rc=%d\n",
					rc);

		pr_smb(PR_STATUS, "Faking Removal\n");
		fake_insertion_removal(chip, false);
		msleep(500);
		pr_smb(PR_STATUS, "Faking Insertion\n");
		fake_insertion_removal(chip, true);

		read_usb_type(chip, &usb_type_name, &usb_supply_type);
		if (usb_supply_type != POWER_SUPPLY_TYPE_USB_DCP) {
			msleep(500);
			pr_smb(PR_STATUS, "Fake Removal again as type!=DCP\n");
			fake_insertion_removal(chip, false);
			msleep(500);
			pr_smb(PR_STATUS, "Fake Insert again as type!=DCP\n");
			fake_insertion_removal(chip, true);
		}

		rc = vote(chip->usb_icl_votable,
				CHG_SUSPEND_WORKAROUND_ICL_VOTER, false, 0);
		if (rc < 0)
			pr_err("Couldn't vote for 0 for suspend wa, going ahead rc=%d\n",
					rc);
	}
}

static int smbchg_probe(struct spmi_device *spmi)
{
	int rc;
	struct smbchg_chip *chip;
	struct power_supply *usb_psy, *typec_psy = NULL;
	struct qpnp_vadc_chip *vadc_dev, *vchg_vadc_dev;
	const char *typec_psy_name;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_smb(PR_STATUS, "USB supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	if (of_property_read_bool(spmi->dev.of_node, "qcom,external-typec")) {
		
		rc = of_property_read_string(spmi->dev.of_node,
				"qcom,typec-psy-name", &typec_psy_name);
		if (rc) {
			pr_err("failed to get prop typec-psy-name rc=%d\n",
				rc);
			return rc;
		}

		typec_psy = power_supply_get_by_name(typec_psy_name);
		if (!typec_psy) {
			pr_smb(PR_STATUS,
				"Type-C supply not found, deferring probe\n");
			return -EPROBE_DEFER;
		}
	}

	if (of_find_property(spmi->dev.of_node, "qcom,usbin-vadc", NULL)) {
		vadc_dev = qpnp_get_vadc(&spmi->dev, "usbin");
		if (IS_ERR(vadc_dev)) {
			rc = PTR_ERR(vadc_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(&spmi->dev, "Couldn't get vadc rc=%d\n",
						rc);
			return rc;
		}
	}

	if (of_find_property(spmi->dev.of_node, "qcom,vchg_sns-vadc", NULL)) {
		vchg_vadc_dev = qpnp_get_vadc(&spmi->dev, "vchg_sns");
		if (IS_ERR(vchg_vadc_dev)) {
			rc = PTR_ERR(vchg_vadc_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(&spmi->dev, "Couldn't get vadc 'vchg' rc=%d\n",
						rc);
			return rc;
		}
	}

	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->fcc_votable = create_votable(&spmi->dev,
			"SMBCHG: fcc",
			VOTE_MIN, NUM_FCC_VOTER, 2000,
			set_fastchg_current_vote_cb);
	if (IS_ERR(chip->fcc_votable))
		return PTR_ERR(chip->fcc_votable);

	chip->usb_icl_votable = create_votable(&spmi->dev,
			"SMBCHG: usb_icl",
			VOTE_MIN, NUM_ICL_VOTER, 3000,
			set_usb_current_limit_vote_cb);
	if (IS_ERR(chip->usb_icl_votable))
		return PTR_ERR(chip->usb_icl_votable);

	chip->dc_icl_votable = create_votable(&spmi->dev,
			"SMBCHG: dcl_icl",
			VOTE_MIN, NUM_ICL_VOTER, 3000,
			set_dc_current_limit_vote_cb);
	if (IS_ERR(chip->dc_icl_votable))
		return PTR_ERR(chip->dc_icl_votable);

	chip->usb_suspend_votable = create_votable(&spmi->dev,
			"SMBCHG: usb_suspend",
			VOTE_SET_ANY, NUM_EN_VOTERS, 0,
			usb_suspend_vote_cb);
	if (IS_ERR(chip->usb_suspend_votable))
		return PTR_ERR(chip->usb_suspend_votable);

	chip->dc_suspend_votable = create_votable(&spmi->dev,
			"SMBCHG: dc_suspend",
			VOTE_SET_ANY, NUM_EN_VOTERS, 0,
			dc_suspend_vote_cb);
	if (IS_ERR(chip->dc_suspend_votable))
		return PTR_ERR(chip->dc_suspend_votable);

	chip->battchg_suspend_votable = create_votable(&spmi->dev,
			"SMBCHG: battchg_suspend",
			VOTE_SET_ANY, NUM_BATTCHG_EN_VOTERS, 0,
			charging_suspend_vote_cb);
	if (IS_ERR(chip->battchg_suspend_votable))
		return PTR_ERR(chip->battchg_suspend_votable);

	chip->hw_aicl_rerun_disable_votable = create_votable(&spmi->dev,
			"SMBCHG: hwaicl_disable",
			VOTE_SET_ANY, NUM_HW_AICL_DISABLE_VOTERS, 0,
			smbchg_hw_aicl_rerun_disable_cb);
	if (IS_ERR(chip->hw_aicl_rerun_disable_votable))
		return PTR_ERR(chip->hw_aicl_rerun_disable_votable);

	chip->hw_aicl_rerun_enable_indirect_votable = create_votable(&spmi->dev,
			"SMBCHG: hwaicl_enable_indirect",
			VOTE_SET_ANY, NUM_HW_AICL_RERUN_ENABLE_INDIRECT_VOTERS,
			0, smbchg_hw_aicl_rerun_enable_indirect_cb);
	if (IS_ERR(chip->hw_aicl_rerun_enable_indirect_votable))
		return PTR_ERR(chip->hw_aicl_rerun_enable_indirect_votable);

	chip->aicl_deglitch_short_votable = create_votable(&spmi->dev,
			"SMBCHG: hwaicl_short_deglitch",
			VOTE_SET_ANY, NUM_HW_SHORT_DEGLITCH_VOTERS, 0,
			smbchg_aicl_deglitch_config_cb);
	if (IS_ERR(chip->aicl_deglitch_short_votable))
		return PTR_ERR(chip->aicl_deglitch_short_votable);

	INIT_WORK(&chip->usb_set_online_work, smbchg_usb_update_online_work);
	INIT_DELAYED_WORK(&chip->parallel_en_work,
			smbchg_parallel_usb_en_work);
	INIT_DELAYED_WORK(&chip->vfloat_adjust_work, smbchg_vfloat_adjust_work);
	INIT_DELAYED_WORK(&chip->hvdcp_det_work, smbchg_hvdcp_det_work);
#ifdef CONFIG_HTC_BATT
	INIT_WORK(&chip->usb_aicl_limit_current, smbchg_usb_limit_current_WA_work);
	INIT_DELAYED_WORK(&chip->usb_limit_max_current, smbchg_usb_limit_max_current_work);
	INIT_DELAYED_WORK(&chip->rerun_apsd_work, smbchg_rerun_apsd_worker);
	INIT_DELAYED_WORK(&chip->iusb_5v_2a_detect_work, smbchg_iusb_5v_2a_detect_work);
	INIT_DELAYED_WORK(&chip->downgrade_iusb_work, smbchg_downgrade_iusb_work);
#ifdef CONFIG_HTC_BATT_WA_PCN0017
	INIT_DELAYED_WORK(&chip->chk_cable_workable_work, smbchg_chk_cable_workable_work);
#endif 
	INIT_DELAYED_WORK(&chip->force_hvdcp_work, smbchg_force_hvdcp_worker);
	INIT_WORK(&chip->hvdcp_redet_work, smbchg_hvdcp_redet_worker);
#endif 
	init_completion(&chip->src_det_lowered);
	init_completion(&chip->src_det_raised);
	init_completion(&chip->usbin_uv_lowered);
	init_completion(&chip->usbin_uv_raised);
	chip->vadc_dev = vadc_dev;
	chip->vchg_vadc_dev = vchg_vadc_dev;
	chip->spmi = spmi;
	chip->dev = &spmi->dev;
	chip->usb_psy = usb_psy;
	chip->typec_psy = typec_psy;
	chip->fake_battery_soc = -EINVAL;
	chip->usb_online = -EINVAL;
	dev_set_drvdata(&spmi->dev, chip);

	spin_lock_init(&chip->sec_access_lock);
	mutex_init(&chip->therm_lvl_lock);
	mutex_init(&chip->usb_set_online_lock);
	mutex_init(&chip->parallel.lock);
	mutex_init(&chip->taper_irq_lock);
	mutex_init(&chip->pm_lock);
	mutex_init(&chip->wipower_config);
	mutex_init(&chip->usb_status_lock);
	device_init_wakeup(chip->dev, true);

	rc = smbchg_parse_peripherals(chip);
	if (rc) {
		dev_err(chip->dev, "Error parsing DT peripherals: %d\n", rc);
		return rc;
	}

	rc = smbchg_check_chg_version(chip);
	if (rc) {
		pr_err("Unable to check schg version rc=%d\n", rc);
		return rc;
	}

	rc = smb_parse_dt(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to parse DT nodes: %d\n", rc);
		return rc;
	}

	rc = smbchg_regulator_init(chip);
	if (rc) {
		dev_err(&spmi->dev,
			"Couldn't initialize regulator rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_hw_init(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to intialize hardware rc = %d\n", rc);
		goto out;
	}

	rc = determine_initial_status(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to determine init status rc = %d\n", rc);
		goto out;
	}

	chip->previous_soc = -EINVAL;
	chip->batt_psy.name		= chip->battery_psy_name;
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smbchg_battery_get_property;
	chip->batt_psy.set_property	= smbchg_battery_set_property;
	chip->batt_psy.properties	= smbchg_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smbchg_battery_properties);
	chip->batt_psy.external_power_changed = smbchg_external_power_changed;
	chip->batt_psy.property_is_writeable = smbchg_battery_is_writeable;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto out;
	}

#ifdef CONFIG_HTC_BATT
	htc_battery_create_attrs(chip->batt_psy.dev);
#endif

	if (chip->dc_psy_type != -EINVAL) {
		chip->dc_psy.name		= "dc";
		chip->dc_psy.type		= chip->dc_psy_type;
		chip->dc_psy.get_property	= smbchg_dc_get_property;
		chip->dc_psy.set_property	= smbchg_dc_set_property;
		chip->dc_psy.property_is_writeable = smbchg_dc_is_writeable;
		chip->dc_psy.properties		= smbchg_dc_properties;
		chip->dc_psy.num_properties = ARRAY_SIZE(smbchg_dc_properties);
		chip->dc_psy.supplied_to = smbchg_dc_supplicants;
		chip->dc_psy.num_supplicants
			= ARRAY_SIZE(smbchg_dc_supplicants);
		rc = power_supply_register(chip->dev, &chip->dc_psy);
		if (rc < 0) {
			dev_err(&spmi->dev,
				"Unable to register dc_psy rc = %d\n", rc);
			goto unregister_batt_psy;
		}
	}
	chip->psy_registered = true;

	if (chip->cfg_chg_led_support &&
			chip->schg_version == QPNP_SCHG_LITE) {
		rc = smbchg_register_chg_led(chip);
		if (rc) {
			dev_err(chip->dev,
					"Unable to register charger led: %d\n",
					rc);
			goto unregister_dc_psy;
		}

		rc = smbchg_chg_led_controls(chip);
		if (rc) {
			dev_err(chip->dev,
					"Failed to set charger led controld bit: %d\n",
					rc);
			goto unregister_led_class;
		}
	}

	rc = smbchg_request_irqs(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to request irqs rc = %d\n", rc);
		goto unregister_led_class;
	}

	if (!chip->skip_usb_notification) {
		pr_smb(PR_MISC, "setting usb psy present = %d\n",
			chip->usb_present);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
	}

#ifdef CONFIG_HTC_BATT
	if (get_kernel_flag() & KERNEL_FLAG_ENABLE_BMS_CHARGER_LOG)
		smbchg_debug_mask = 0xFF;
	else
		smbchg_debug_mask = 0x06;
	printk("smbchg_debug_mask=0x%X\n",smbchg_debug_mask);
#endif 

	rerun_hvdcp_det_if_necessary(chip);

	dump_regs(chip);
	create_debugfs_entries(chip);
	dev_info(chip->dev,
		"SMBCHG successfully probe Charger version=%s Revision DIG:%d.%d ANA:%d.%d batt=%d dc=%d usb=%d\n",
			version_str[chip->schg_version],
			chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
			chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR],
			get_prop_batt_present(chip),
			chip->dc_present, chip->usb_present);

#ifdef CONFIG_HTC_BATT
	htc_battery_probe_process(CHARGER_PROBE_DONE);
	the_chip = chip;
	pmi8994_boot_update_usb_status();
#endif

	return 0;

unregister_led_class:
	if (chip->cfg_chg_led_support && chip->schg_version == QPNP_SCHG_LITE)
		led_classdev_unregister(&chip->led_cdev);
unregister_dc_psy:
	power_supply_unregister(&chip->dc_psy);
unregister_batt_psy:
	power_supply_unregister(&chip->batt_psy);
out:
	handle_usb_removal(chip);
	return rc;
}

static int smbchg_remove(struct spmi_device *spmi)
{
	struct smbchg_chip *chip = dev_get_drvdata(&spmi->dev);

	debugfs_remove_recursive(chip->debug_root);

	if (chip->dc_psy_type != -EINVAL)
		power_supply_unregister(&chip->dc_psy);

	power_supply_unregister(&chip->batt_psy);

	return 0;
}

static void smbchg_shutdown(struct spmi_device *spmi)
{
	struct smbchg_chip *chip = dev_get_drvdata(&spmi->dev);
	int i, rc;

	if (!(chip->wa_flags & SMBCHG_RESTART_WA))
		return;

	if (!is_hvdcp_present(chip))
		return;

	pr_smb(PR_MISC, "Disable Parallel\n");
	mutex_lock(&chip->parallel.lock);
	smbchg_parallel_en = 0;
	smbchg_parallel_usb_disable(chip);
	mutex_unlock(&chip->parallel.lock);

	pr_smb(PR_MISC, "Disable all interrupts\n");
	disable_irq(chip->aicl_done_irq);
	disable_irq(chip->batt_cold_irq);
	disable_irq(chip->batt_cool_irq);
	disable_irq(chip->batt_hot_irq);
	disable_irq(chip->batt_missing_irq);
	disable_irq(chip->batt_warm_irq);
	disable_irq(chip->chg_error_irq);
	disable_irq(chip->chg_hot_irq);
	disable_irq(chip->chg_term_irq);
	disable_irq(chip->dcin_uv_irq);
	disable_irq(chip->fastchg_irq);
	disable_irq(chip->otg_fail_irq);
	disable_irq(chip->otg_oc_irq);
	disable_irq(chip->power_ok_irq);
	disable_irq(chip->recharge_irq);
	disable_irq(chip->src_detect_irq);
	disable_irq(chip->taper_irq);
	disable_irq(chip->usbid_change_irq);
	disable_irq(chip->usbin_ov_irq);
	disable_irq(chip->usbin_uv_irq);
	disable_irq(chip->vbat_low_irq);
	disable_irq(chip->wdog_timeout_irq);

	
	for (i = 0; i < NUM_HW_SHORT_DEGLITCH_VOTERS; i++)
		vote(chip->aicl_deglitch_short_votable, i, false, 0);

	
	rc = vote(chip->hw_aicl_rerun_enable_indirect_votable,
			SHUTDOWN_WORKAROUND_VOTER, true, 0);
	if (rc < 0)
		pr_err("Couldn't vote to enable indirect AICL rerun\n");
	rc = vote(chip->hw_aicl_rerun_disable_votable,
		WEAK_CHARGER_HW_AICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't vote to enable AICL rerun\n");

	
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		return;
	}

	pr_smb(PR_MISC, "Wait 500mS to lower to 5V\n");
	
	msleep(500);
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		return;
	}

	
	pr_smb(PR_MISC, "Disable HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, 0);
	if (rc < 0)
		pr_err("Couldn't disable HVDCP rc=%d\n", rc);

	chip->hvdcp_3_det_ignore_uv = true;
	
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0)
		pr_err("Couldn't fake removal HVDCP Removed rc=%d\n", rc);

	
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0)
		pr_err("Couldn't fake insertion rc=%d\n", rc);

	pr_smb(PR_MISC, "Wait 1S to settle\n");
	msleep(1000);
	chip->hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_STATUS, "wrote power off configurations\n");
}

static const struct dev_pm_ops smbchg_pm_ops = {
};

MODULE_DEVICE_TABLE(spmi, smbchg_id);

static struct spmi_driver smbchg_driver = {
	.driver		= {
		.name		= "qpnp-smbcharger",
		.owner		= THIS_MODULE,
		.of_match_table	= smbchg_match_table,
		.pm		= &smbchg_pm_ops,
	},
	.probe		= smbchg_probe,
	.remove		= smbchg_remove,
	.shutdown	= smbchg_shutdown,
};

static int __init smbchg_init(void)
{
	return spmi_driver_register(&smbchg_driver);
}

static void __exit smbchg_exit(void)
{
	return spmi_driver_unregister(&smbchg_driver);
}

module_init(smbchg_init);
module_exit(smbchg_exit);

MODULE_DESCRIPTION("QPNP SMB Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:qpnp-smbcharger");
