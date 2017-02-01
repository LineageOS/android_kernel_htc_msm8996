#define CONFIG_HTC_BATT_PCN0001 //Remap
#define CONFIG_HTC_BATT_PCN0002 //Level consistent
#define CONFIG_HTC_BATT_PCN0003 //Battery ID judgement
#define CONFIG_HTC_BATT_PCN0004 //HTC Attribute
#define CONFIG_HTC_BATT_PCN0005 //overloading
#define CONFIG_HTC_BATT_PCN0006 //Debug flag
#define CONFIG_HTC_BATT_PCN0007 //Safety timer
#define CONFIG_HTC_BATT_PCN0008 //Charging cycle calculation
#define CONFIG_HTC_BATT_PCN0009 //SSD Battery View
#define CONFIG_HTC_BATT_PCN0010 //Limit IUSB in identical discharge time.
#define CONFIG_HTC_BATT_PCN0011 //Prepare/Complete function
#define CONFIG_HTC_BATT_PCN0012 //LED control
#define CONFIG_HTC_BATT_PCN0013 //Smart charging
#define CONFIG_HTC_BATT_PCN0014 //Driver ready notification
#define CONFIG_HTC_BATT_PCN0015 //5V/2A Charger implement
//#define CONFIG_HTC_BATT_PCN0016 //Cool charger: implement in msm_defconfig
#define CONFIG_HTC_BATT_PCN0017 //Unknown charger detection
#define CONFIG_HTC_BATT_PCN0018 //Bad cable detection
#define CONFIG_HTC_BATT_PCN0019 //QC2.0/QC3.0 UI notification
#define CONFIG_HTC_BATT_PCN0020 //PD charger: dependency with usb driver's code, ***must check***
#define CONFIG_HTC_BATT_PCN0021 //Power team monitor battery charge/discharge
#define CONFIG_HTC_BATT_PCN0022 //USB overheat feature
#define CONFIG_HTC_BATT_WA_PCN0001 //[QCTBUG] release wake lock
#define CONFIG_HTC_BATT_WA_PCN0002 //[QCTBUG] PMIC BUG
#define CONFIG_HTC_BATT_WA_PCN0003 //Unexpected battery warm
//#define CONFIG_HTC_BATT_WA_PCN0004 //Fingerprint disable charging, use CONFIG_FPC_HTC_DISABLE_CHARGING
#define CONFIG_HTC_BATT_WA_PCN0005 //USB2.0 charger detect slow
#define CONFIG_HTC_BATT_WA_PCN0006 //Hard limit to prevent AICL over spec.
#define CONFIG_HTC_BATT_WA_PCN0007 //[QCTBUG] boot time long issue
#define CONFIG_HTC_BATT_WA_PCN0008 //Charger suspend issue
#define CONFIG_HTC_BATT_WA_PCN0009 //[QCTBUG] OV keep when unplug
#define CONFIG_HTC_BATT_WA_PCN0010 //Avoid reverse boost after DCP unplugged
#define CONFIG_HTC_BATT_WA_PCN0011 //Rerun APSD conditions
#define CONFIG_HTC_BATT_WA_PCN0012 //OTG charge fail
#define CONFIG_HTC_BATT_WA_PCN0013 //QC3.0 cannot be detected with high impedance cable
#define CONFIG_HTC_BATT_WA_PCN0014 //High board impedance cause small charge current
#define CONFIG_HTC_BATT_WA_PCN0015 //Adjust cc to cv mode threshold
#define CONFIG_HTC_BATT_WA_PCN0016 //FG SRAM dump
#define CONFIG_HTC_BATT_WA_PCN0017 //Default disable hvdcp for non-standard cable
#define CONFIG_HTC_BATT_WA_PCN0019 //Skip IACS ready timeout when FG reset is on going
#define CONFIG_HTC_BATT_WA_PCN0021 //Set default AICL 1A, and set AICL back to 1.5A after kernel time 2min for car charger

#include <linux/rtc.h>
#ifdef CONFIG_HTC_BATT_PCN0011
#include <linux/alarmtimer.h>
#endif //CONFIG_HTC_BATT_PCN0011
#include <linux/wakelock.h>
#include <linux/power_supply.h>

#define BATT_LOG(x...) do { \
printk(KERN_INFO "[BATT] " x); \
} while (0)

#define BATT_ERR(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_ERR "[BATT] err:" x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#define BATT_EMBEDDED(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_ERR "[BATT] " x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#ifdef CONFIG_HTC_BATT_PCN0006
#define POWER_MONITOR_BATT_CAPACITY	77
#define POWER_MONITOR_BATT_TEMP	330
#endif //CONFIG_HTC_BATT_PCN0006

#ifdef CONFIG_HTC_BATT_PCN0002
/* stored consistent parameters */
#define STORE_MAGIC_NUM          0xDDAACC00
#define STORE_MAGIC_OFFSET       3104    /*0xC20*/
#define STORE_SOC_OFFSET         3108    /*0xC24*/
#define STORE_CURRTIME_OFFSET    3120    /*0xC30*/
#define STORE_TEMP_OFFSET		3140    /*0xC44*/
#endif //CONFIG_HTC_BATT_PCN0002

#ifdef CONFIG_HTC_BATT_PCN0008
/* for batt cycle info */
#define HTC_BATT_TOTAL_LEVELRAW		3144
#define HTC_BATT_OVERHEAT_MSEC		3148
#define HTC_BATT_FIRST_USE_TIME		3152
#define HTC_BATT_CYCLE_CHECKSUM		3156
#endif //CONFIG_HTC_BATT_PCN0008

/* for htc_extension */
#define HTC_EXT_UNKNOWN_USB_CHARGER		(1<<0)
#define HTC_EXT_CHG_UNDER_RATING		(1<<1)
#define HTC_EXT_CHG_SAFTY_TIMEOUT		(1<<2)
#define HTC_EXT_CHG_FULL_EOC_STOP		(1<<3)
#ifdef CONFIG_HTC_BATT_PCN0018
#define HTC_EXT_BAD_CABLE_USED			(1<<4)
#endif //CONFIG_HTC_BATT_PCN0018
#ifdef CONFIG_HTC_BATT_PCN0019
#define HTC_EXT_QUICK_CHARGER_USED		(1<<5)
#endif //CONFIG_HTC_BATT_PCN0019
#define HTC_EXT_USB_OVERHEAT				(1<<6)

#define BATT_TIMER_UPDATE_TIME				(60)
#ifdef CONFIG_HTC_BATT_PCN0011
#define BATT_SUSPEND_CHECK_TIME				(3600)
#define BATT_SUSPEND_HIGHFREQ_CHECK_TIME	(300)
#define BATT_TIMER_CHECK_TIME				(360)
#define CHECH_TIME_TOLERANCE_MS	(1000)

/* for suspend high frequency (5min) */
#define SUSPEND_HIGHFREQ_CHECK_BIT_TALK		(1)
#define SUSPEND_HIGHFREQ_CHECK_BIT_SEARCH	(1<<1)
#define SUSPEND_HIGHFREQ_CHECK_BIT_MUSIC	(1<<3)
#endif //CONFIG_HTC_BATT_PCN0011

struct battery_info_reply {
	u32 batt_vol;
	u32 batt_id;
	s32 batt_temp;
	s32 batt_current;
	u32 charging_source;
	u32 level;
	u32 level_raw;
	u32 full_level;
	u32 status;
	u32 chg_src;
	u32 chg_en;
	u32 chg_batt_en;
	u32 full_level_dis_batt_chg;
	u32 overload;
	u32 over_vchg;
	u32 health;
	bool is_full;
#ifdef CONFIG_HTC_BATT_PCN0016
	bool is_htcchg_ext_mode;
#endif //CONFIG_HTC_BATT_PCN0016
};

struct battery_info_previous {
	s32 batt_temp;
	u32 charging_source;
	u32 level;
	u32 level_raw;
};

#ifdef CONFIG_HTC_BATT_PCN0002
struct htc_battery_store {
	u32 batt_stored_magic_num;
	u32 batt_stored_soc;
	u32 batt_stored_temperature;
	unsigned long batt_stored_update_time;
	u32 consistent_flag;
};
#endif //CONFIG_HTC_BATT_PCN0002

struct htc_charger {
	int (*dump_all)(void);
	int (*get_vbus)(void);
#ifdef CONFIG_HTC_BATT_PCN0009
	int (*get_attr_text)(char *buf, int size);
#endif //CONFIG_HTC_BATT_PCN0009
	int (*is_battery_full_eoc_stop)(int *result);
};

struct htc_gauge {
	int (*get_attr_text)(char *buf, int size);
};

struct htc_battery_info {
	struct battery_info_reply rep;
	struct battery_info_previous prev;
#ifdef CONFIG_HTC_BATT_PCN0002
	struct htc_battery_store store;
#endif //CONFIG_HTC_BATT_PCN0002
	struct htc_charger *icharger;
	struct htc_gauge *igauge;
	struct power_supply		*batt_psy;
	struct power_supply		*bms_psy;
	struct power_supply		*usb_psy;
	int critical_low_voltage_mv;
	int smooth_chg_full_delay_min;
	int decreased_batt_level_check;
	int batt_full_voltage_mv;
	int batt_full_current_ma;
	int overload_curr_thr_ma;
	struct wake_lock charger_exist_lock;
	struct wake_lock check_overheat_lock;
	struct delayed_work chg_full_check_work;
#ifdef CONFIG_HTC_BATT_PCN0022
	struct delayed_work is_usb_overheat_work;
#endif //CONFIG_HTC_BATT_PCN0022
#ifdef CONFIG_HTC_BATT_PCN0017
	struct delayed_work chk_unknown_chg_work;
#endif //CONFIG_HTC_BATT_PCN0017
#ifdef CONFIG_HTC_BATT_PCN0018
	struct delayed_work cable_impedance_work;
#endif //CONFIG_HTC_BATT_PCN0018
	struct delayed_work htc_usb_overheat_work;
	int state;
	int vbus;
	int k_debug_flag;
	int current_limit_reason;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	struct workqueue_struct *batt_fb_wq;
	struct delayed_work work_fb;
#endif
	unsigned int htc_extension;	/* for htc in-house sw */
};

struct htc_battery_timer {
	unsigned long batt_system_jiffies;
	unsigned long total_time_ms;	/* since last do batt_work */
#ifdef CONFIG_HTC_BATT_PCN0011
	unsigned long batt_suspend_ms;
	struct alarm batt_check_wakeup_alarm;
#endif //CONFIG_HTC_BATT_PCN0011
	struct work_struct batt_work;
	struct timer_list batt_timer;
	struct workqueue_struct *batt_wq;
	struct wake_lock battery_lock;
	unsigned int time_out;
};

struct htc_battery_platform_data {
	struct htc_charger icharger;
};

#ifdef CONFIG_HTC_BATT_PCN0020
struct htc_pd_data {
	int	pd_list[10][2];
};
#endif //CONFIG_HTC_BATT_PCN0020

#ifdef CONFIG_HTC_BATT_PCN0021
struct htc_charging_statistics {
        unsigned long begin_chg_time;
        unsigned long end_chg_time;
        int begin_chg_batt_level;
        int end_chg_batt_level;
};

struct htc_statistics_category {
        unsigned long chg_time_sum;
        unsigned long dischg_time_sum;
        int sample_count;
};
#endif //CONFIG_HTC_BATT_PCN0021

enum charger_control_flag {
	STOP_CHARGER = 0,
	ENABLE_CHARGER,
	ENABLE_LIMIT_CHARGER,
	DISABLE_LIMIT_CHARGER,
	DISABLE_PWRSRC,
	ENABLE_PWRSRC,
#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004
	DISABLE_PWRSRC_FINGERPRINT,
	ENABLE_PWRSRC_FINGERPRINT,
#endif //CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004
	END_CHARGER
};

/*
 * MFG ftm mode charger control
 *
 * FTM_ENABLE_CHARGER: default, ftm control disabled
 * FTM_STOP_CHARGER: ftm control to disable charging
 * FTM_FAST_CHARGE: ftm control to force fast charge
 * FTM_SLOW_CHARGE: ftm control to force slow charge
 * FTM_END_CHARGER: do nothing, value for flag out of bound check
 */
enum ftm_charger_control_flag {
	FTM_ENABLE_CHARGER = 0,
	FTM_STOP_CHARGER,
	FTM_FAST_CHARGE,
	FTM_SLOW_CHARGE,
	FTM_END_CHARGER
};

enum user_set_input_current {
	SLOW_CHARGE_CURR = 500000,
	FAST_CHARGE_CURR = 900000,
	WALL_CHARGE_CURR = 1500000,
	HVDCP_CHARGE_CURR = 1600000,
	HVDCP_3_CHARGE_CURR = 2500000,
};

enum htc_batt_probe {
	CHARGER_PROBE_DONE,
	GAUGE_PROBE_DONE,
	HTC_BATT_PROBE_DONE,
	BATT_PROBE_MAX,
};
enum htc_charger_request {
#ifdef CONFIG_HTC_BATT_PCN0018
	CHARGER_ABILITY_DETECT_DONE,
#endif //CONFIG_HTC_BATT_PCN0018
#ifdef CONFIG_HTC_BATT_PCN0015
	CHARGER_5V_2A_DETECT_DONE,
#endif //CONFIG_HTC_BATT_PCN0015
};


enum htc_chr_type_data {
	DATA_NO_CHARGING = 0,
	DATA_UNKNOWN_CHARGER,
	DATA_UNKNOWN_TYPE,
	DATA_USB,
	DATA_USB_CDP,
	DATA_AC,
	DATA_QC2,
	DATA_QC3,
	DATA_PD_5V,
	DATA_PD_9V,
	DATA_PD_12V,
	DATA_TYPE_C
};

#ifdef CONFIG_HTC_BATT_PCN0004
int htc_battery_create_attrs(struct device *dev);
#endif //CONFIG_HTC_BATT_PCN0004
void htc_battery_info_update(enum power_supply_property prop, int intval);
#ifdef CONFIG_HTC_BATT_PCN0014
void htc_battery_probe_process(enum htc_batt_probe probe_type);
#endif //CONFIG_HTC_BATT_PCN0014
#ifdef CONFIG_HTC_BATT_PCN0001
int htc_battery_level_adjust(void);
#endif //CONFIG_HTC_BATT_PCN0001
#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004
int htc_battery_charger_switch_internal(int enable);
#endif //CONFIG_FPC_HTC_DISABLE_CHARGING //HTC_BATT_WA_PCN0004
#ifdef CONFIG_HTC_BATT_PCN0020
int htc_battery_pd_charger_support(int size, struct htc_pd_data pd_data, int *max_mA);
bool htc_battery_get_pd_type(int *curr);
#endif //CONFIG_HTC_BATT_PCN0020
bool htc_battery_get_discharging_reason(void);

/* Implement on QCT driver */
#ifdef CONFIG_HTC_BATT_PCN0018
int request_charger_status(enum htc_charger_request mode, void *ret_buf);
void impedance_set_iusb_max (int current_ua, bool mode);
#endif //CONFIG_HTC_BATT_PCN0018
#if defined(CONFIG_HTC_BATT_PCN0018)||defined(CONFIG_HTC_BATT_PCN0015)||defined(CONFIG_HTC_BATT_PCN0020)
void set_aicl_enable(bool bEnable);
#endif //CONFIG_HTC_BATT_PCN0018/PCN0015/PCN0020
int charger_dump_all(void);
int pmi8994_get_usbin_voltage_now(void);
#ifdef CONFIG_HTC_BATT_PCN0009
int pmi8994_charger_get_attr_text(char *buf, int size);
#endif //CONFIG_HTC_BATT_PCN0009
int pmi8994_is_batt_full_eoc_stop(int *result);
int pmi8994_set_float_voltage_comp (int vfloat_comp);
void pmi8994_set_iusb_max (int current_ua);
void pmi8994_set_batt_health_good(void);
void pmi8994_rerun_apsd(void);
bool is_otg_enabled(void);
int pmi8996_get_chgr_sts(void);
#ifdef CONFIG_HTC_BATT_WA_PCN0016
void force_dump_fg_sram(void);
#endif //CONFIG_HTC_BATT_WA_PCN0016
#ifdef CONFIG_HTC_BATT_WA_PCN0021
void pmi8996_set_dcp_default(void);
#endif //CONFIG_HTC_BATT_WA_PCN0021

#ifdef CONFIG_HTC_BATT_PCN0022
int pm8996_get_usb_temp(void);
#endif // CONFIG_HTC_BATT_PCN0022
#ifdef CONFIG_HTC_BATT_PCN0002
bool get_ima_error_status(void);
#endif //CONFIG_HTC_BATT_PCN0002

bool usb_otg_pulse_skip_control(bool disable);
