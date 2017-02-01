#define pr_fmt(fmt) "HTCCHG: %s: " fmt, __func__

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
#include <linux/platform_device.h>
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
#include <linux/htc_flags.h>
#include "pmic-voter.h"
#ifdef CONFIG_HTC_BATT
#include <linux/power/htc_battery.h>
#endif
#ifdef CONFIG_HTC_BATT_8960
#include <linux/power/htc_battery_8960.h>
#include <linux/power/htc_battery_core.h>
#endif

#define CHG_WORKER_CHECK_PERIOD_MS	30000
#define CHG_READY_CHECK_PERIOD_MS	1000
#define CHG_IUSB_POLL_PERIOD_MS		3000
#define CHG_WATCHDOG_PERIOD_MS		180000

enum print_reason {
	PR_REGISTER	= BIT(0),
	PR_INTERRUPT	= BIT(1),
	PR_STATUS	= BIT(2),
	PR_DUMP		= BIT(3),
};

/* Mask/Bit helpers */
#define _HTCCHG_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define HTCCHG_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_HTCCHG_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))

enum {
	CHARGE_DISABLE = 0,
	CHARGE_ENABLE = 1,
};

struct htcchg_parameter {
	int		v;
	char*	name;
};

#define HTCCHG_CONFIG(Name)	struct htcchg_parameter Name;
struct htcchg_config {
	#include <linux/power/htc_charger_cfg.h>
};
#undef HTCCHG_CONFIG

#define HTCCHG_CONFIG(Name)	#Name ,
static char* config_name[] = {
	#include <linux/power/htc_charger_cfg.h>
};
#undef HTCCHG_CONFIG

struct htcchg_chip {
	struct device			*dev;
	/* configuration parameters */
	bool			htcchg_ext_mode;
	int				charge_enabled;
	int				charge_disallowed;
	int				flash_actived;
	int				path_status;
	int				watchdog_timer;
	int				temp_level;

	/* gpios */
	int				chg_enable_gpio;
	int				chg_ready_gpio;
	int				adc_sw_sel_gpio;
	struct pinctrl			*pinctrl;
	struct pinctrl_state		*gpio_state_init;

	/* vadcs */
	struct qpnp_vadc_chip		*vadc_usb_in_isen;
	unsigned int				vadc_usb_in_isen_channel;
	struct qpnp_vadc_chip		*vadc_vbusdet;
	unsigned int				vadc_vbusdet_channel;
	struct qpnp_vadc_chip		*vadc_usb_pwr_temp;
	unsigned int				vadc_usb_pwr_temp_channel;

	unsigned int				iusb_rsen;
	unsigned int				iusb_multiplier;

	/* psy */
	struct power_supply		htcchg_psy;
	bool						psy_registered;
	struct power_supply		*bms_psy;
	struct power_supply		*battery_psy;
	struct power_supply		*usb_psy;
	const char				*bms_psy_name;
	const char				*battery_psy_name;
	const char				*usb_psy_name;

	/* workers */
	struct delayed_work		chg_check_work;

	/* mutex */
	struct mutex				access_lock;

	/* Daemon configuration */
	struct htcchg_config		config;

};

static int htcchg_debug_mask;
module_param_named(
	debug_mask, htcchg_debug_mask, int, S_IRUSR | S_IWUSR
);

#define pr_htcchg(reason, fmt, ...)				\
	do {							\
		if (htcchg_debug_mask & (reason))		\
			pr_info(fmt, ##__VA_ARGS__);		\
		else						\
			pr_debug(fmt, ##__VA_ARGS__);		\
	} while (0)

#define pr_htcchg_rt(reason, fmt, ...)					\
	do {								\
		if (htcchg_debug_mask & (reason))			\
			pr_info_ratelimited(fmt, ##__VA_ARGS__);	\
		else							\
			pr_debug_ratelimited(fmt, ##__VA_ARGS__);	\
	} while (0)

static bool enable_gpio_requested = false;
static bool ready_gpio_requested = false;
static bool adc_sw_sel_gpio_requested = false;

static int htcchg_set_property_on_battery(struct htcchg_chip *chip,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->battery_psy&& chip->battery_psy_name)
		chip->battery_psy =
			power_supply_get_by_name((char *)chip->battery_psy_name);
	if (!chip->battery_psy) {
		pr_htcchg(PR_STATUS, "no battery psy found\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = chip->battery_psy->set_property(chip->battery_psy, prop, &ret);
	if (rc)
		pr_htcchg(PR_STATUS,
			"battery psy does not allow updating prop %d rc = %d\n",
			prop, rc);
	return rc;
}

static int htcchg_get_property_from_usb(struct htcchg_chip *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->usb_psy && chip->usb_psy_name)
		chip->usb_psy =
			power_supply_get_by_name((char *)chip->usb_psy_name);
	if (!chip->usb_psy) {
		pr_htcchg(PR_STATUS, "no usb psy found\n");
		return -EINVAL;
	}

	rc = chip->usb_psy->get_property(chip->usb_psy, prop, &ret);
	if (rc) {
		pr_htcchg(PR_STATUS,
			"usb psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

static int htcchg_get_property_from_fg(struct htcchg_chip *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_htcchg(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	rc = chip->bms_psy->get_property(chip->bms_psy, prop, &ret);
	if (rc) {
		pr_htcchg(PR_STATUS,
			"bms psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

static int htcchg_get_soc_from_fg(struct htcchg_chip *chip)
{
#if defined(CONFIG_HTC_BATT) || defined(CONFIG_HTC_BATT_8960)
	int soc, rc;

	if(chip) {
		rc = htcchg_get_property_from_fg(chip, POWER_SUPPLY_PROP_SYSTEM_SOC, &soc);
		if (rc == 0) {
			return soc;
		}
	}
#endif //CONFIG_HTC_BATT || CONFIG_HTC_BATT_8960
	return -EINVAL;
}

static int htcchg_get_usb_in_isen_adc(struct htcchg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if(chip->vadc_usb_in_isen_channel == 0)
		return -EINVAL;

	if (IS_ERR(chip->vadc_usb_in_isen)) {
		chip->vadc_usb_in_isen = qpnp_get_vadc(chip->dev, "usb_in_isen");
	}

	if (IS_ERR(chip->vadc_usb_in_isen)) {
		dev_err(chip->dev, "Failed to get vadc_usb_in_isen %ld\n", PTR_ERR(chip->vadc_usb_in_isen));
		return -EINVAL;
	}

	if((chip->adc_sw_sel_gpio > 0) && (adc_sw_sel_gpio_requested == false)) {
		adc_sw_sel_gpio_requested = true;
		rc = gpio_request(chip->adc_sw_sel_gpio, "adc_sw_sel_gpio");
		if (rc < 0) {
			dev_err(chip->dev, "gpio_request adc_sw_sel_gpio fail!\n");
		} else {
			pr_htcchg(PR_STATUS, "gpio_request adc_sw_sel_gpio %d success.\n", chip->adc_sw_sel_gpio);
		}
	}

	if(chip->adc_sw_sel_gpio > 0)
		gpio_direction_output(chip->adc_sw_sel_gpio, 1);

	if(chip->vadc_usb_in_isen) {
		rc = qpnp_vadc_read(chip->vadc_usb_in_isen, chip->vadc_usb_in_isen_channel, &results);
		if (rc) {
			pr_info("Unable to read USB_IN_ISEN rc=%d\n", rc);
			return -EINVAL;
		} else {
			return (int)results.physical;
		}
	}

	return -EINVAL;
}

static int htcchg_get_usb_in_isen_current_ma(struct htcchg_chip *chip)
{
	int vsen = 0;

	if(chip->iusb_rsen)
	{
		vsen = htcchg_get_usb_in_isen_adc(chip);
		if(vsen >= 0)
			return (vsen * chip->iusb_multiplier / chip->iusb_rsen) / 1000;
		else
			return -1;
	} else {
		pr_info("iusb_rsen is not valid.\n");
		return -1;
	}
	return -EINVAL;
}

static int htcchg_get_vbusdet_adc(struct htcchg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if(chip->vadc_vbusdet_channel == 0)
		return -EINVAL;

	if (IS_ERR(chip->vadc_vbusdet)) {
		chip->vadc_vbusdet = qpnp_get_vadc(chip->dev, "vbusdet");
	}

	if (IS_ERR(chip->vadc_vbusdet)) {
		dev_err(chip->dev, "Failed to get vadc_vbusdet %ld\n", PTR_ERR(chip->vadc_vbusdet));
		return -EINVAL;
	}

	if((chip->adc_sw_sel_gpio > 0) && (adc_sw_sel_gpio_requested == false)) {
		adc_sw_sel_gpio_requested = true;
		rc = gpio_request(chip->adc_sw_sel_gpio, "adc_sw_sel_gpio");
		if (rc < 0) {
			dev_err(chip->dev, "gpio_request adc_sw_sel_gpio fail!\n");
		} else {
			pr_htcchg(PR_STATUS, "gpio_request adc_sw_sel_gpio %d success.\n", chip->adc_sw_sel_gpio);
		}
	}

	if(chip->adc_sw_sel_gpio > 0)
		gpio_direction_output(chip->adc_sw_sel_gpio, 0);

	if(chip->vadc_vbusdet) {
		rc = qpnp_vadc_read(chip->vadc_vbusdet, chip->vadc_vbusdet_channel, &results);
		if (rc) {
			pr_info("Unable to read VBUS_DET rc=%d\n", rc);
			return -EINVAL;
		} else {
			return (int)results.physical;
		}
	}
	return -EINVAL;
}

static int htcchg_get_usb_pwr_temp_adc(struct htcchg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if(chip->vadc_usb_pwr_temp_channel == 0)
		return -EINVAL;

	if (IS_ERR(chip->vadc_usb_pwr_temp)) {
		chip->vadc_usb_pwr_temp = qpnp_get_vadc(chip->dev, "usb_pwr_temp");
	}

	if (IS_ERR(chip->vadc_usb_pwr_temp)) {
		dev_err(chip->dev, "Failed to get vadc_usb_pwr_temp %ld\n", PTR_ERR(chip->vadc_usb_pwr_temp));
		return -EINVAL;
	}

	if(chip->vadc_usb_pwr_temp) {
		rc = qpnp_vadc_read(chip->vadc_usb_pwr_temp, chip->vadc_usb_pwr_temp_channel, &results);
		if (rc) {
			pr_info("Unable to read USB_PWR_TEMP rc=%d\n", rc);
			return -EINVAL;
		} else {
			return (int)results.physical;
		}
	}
	return -EINVAL;
}

static ssize_t htcchg_get_iusb_ma(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	struct power_supply * psy = (struct power_supply *)dev_get_drvdata(dev);
	struct htcchg_chip *chip = container_of(psy,
				struct htcchg_chip, htcchg_psy);

	if(chip) {
		return snprintf(buf, 10, "%d\n", htcchg_get_usb_in_isen_current_ma(chip));
	} else {
		return snprintf(buf, 10, "0\n");
	}
}

static ssize_t htcchg_get_soc(struct device *dev,
                struct device_attribute *attr,
                char *buf)

{
	struct power_supply * psy = (struct power_supply *)dev_get_drvdata(dev);
	struct htcchg_chip *chip = container_of(psy,
				struct htcchg_chip, htcchg_psy);

	if(chip) {
		return snprintf(buf, 10, "%d\n", htcchg_get_soc_from_fg(chip));
	} else {
		return snprintf(buf, 10, "0\n");
	}
}

static int htcchg_en_control(struct htcchg_chip *chip, int enable)
{
	int vbat = 0;
	int rc = 0;

	if(enable == CHARGE_ENABLE) {
		pr_htcchg(PR_STATUS, "CHARGE_ENABLE\n");
	} else if(enable == CHARGE_DISABLE) {
		pr_htcchg(PR_STATUS, "CHARGE_DISABLE\n");
	} else {
		pr_htcchg(PR_STATUS, "Unknown control %d.\n", enable);
		return -EINVAL;
	}

	if(chip == NULL)
		return -EINVAL;

	if(chip->htcchg_ext_mode == 0) {
		pr_htcchg(PR_STATUS, "HTCCHG_EXT_MODE: %d.\n", chip->htcchg_ext_mode);
		return 0;
	}

	chip->watchdog_timer = CHG_WATCHDOG_PERIOD_MS;

	if((chip->chg_enable_gpio > 0) && (enable_gpio_requested == false)) {
		enable_gpio_requested = true;
		rc = gpio_request(chip->chg_enable_gpio, "chg_enable_gpio");
		if (rc < 0) {
			dev_err(chip->dev, "gpio_request chg_enable_gpio fail!\n");
		} else {
			pr_htcchg(PR_STATUS, "gpio_request chg_enable_gpio %d success.\n", chip->chg_enable_gpio);
		}
	}

	if((chip->chg_ready_gpio > 0) && (ready_gpio_requested == false)) {
		ready_gpio_requested = true;
		rc = gpio_request(chip->chg_ready_gpio, "chg_ready_gpio");
		if (rc < 0) {
			dev_err(chip->dev, "gpio_request chg_ready_gpio fail!\n");
		} else {
			pr_htcchg(PR_STATUS, "gpio_request chg_ready_gpio %d success.\n", chip->chg_ready_gpio);
		}
	}

	htcchg_get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat);
	vbat = vbat / 1000;

	if( enable == CHARGE_ENABLE)
	{
		if(chip->config.vbat_mv_max.v > 0) {
			if(vbat > chip->config.vbat_mv_max.v) {
				pr_htcchg(PR_STATUS, "refuse enable request: vbat %d > %d\n", vbat, chip->config.vbat_mv_max.v);
				return -EINVAL;
			}
		}
	}

	chip->charge_enabled = enable;
	if(chip->charge_disallowed) {
		pr_htcchg(PR_STATUS, "Charge Disallowed.\n");
		enable = 0;
	}

	if(chip->flash_actived) {
		pr_htcchg(PR_STATUS, "Flash actived.\n");
		enable = 0;
	}

	if(enable != chip->path_status)
	{
		pr_htcchg(PR_STATUS, "Decision: %d\n", enable);

		mutex_lock(&chip->access_lock);
		chip->path_status = enable;
		if(chip->chg_ready_gpio> 0) {
			if( enable == CHARGE_ENABLE)
				gpio_direction_output(chip->chg_ready_gpio, 0);
			else
				gpio_direction_input(chip->chg_ready_gpio);
		}
		if(chip->chg_enable_gpio > 0) {
			if( enable == CHARGE_ENABLE)
				gpio_direction_output(chip->chg_enable_gpio, 1);
			else
				gpio_direction_output(chip->chg_enable_gpio, 0);
		}
		mutex_unlock(&chip->access_lock);
		if (delayed_work_pending(&chip->chg_check_work))
			cancel_delayed_work(&chip->chg_check_work);

		if(enable) {
			/* Start worker to check safety */
			schedule_delayed_work(&chip->chg_check_work, msecs_to_jiffies(CHG_READY_CHECK_PERIOD_MS));
			chip->watchdog_timer -= CHG_READY_CHECK_PERIOD_MS;
		}
	}
	return 0;
}

static ssize_t htcchg_control(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	struct power_supply * psy = (struct power_supply *)dev_get_drvdata(dev);
	struct htcchg_chip *chip = container_of(psy,
				struct htcchg_chip, htcchg_psy);
	int val = 0;
	int rc = 0;

	rc = kstrtoint(buf, 10, &val);
	if (rc)
		return count;

	switch (val) {
	case POWER_SUPPLY_HTCCHG_EXT_START:
		pr_htcchg(PR_STATUS, "POWER_SUPPLY_HTCCHG_EXT_START.\n");
		chip->htcchg_ext_mode = true;
		break;

	case POWER_SUPPLY_HTCCHG_EXT_STOP:
		pr_htcchg(PR_STATUS, "POWER_SUPPLY_HTCCHG_EXT_STOP\n");
		chip->htcchg_ext_mode = false;
		break;
	default:
		break;
	}

	htcchg_set_property_on_battery(chip, POWER_SUPPLY_PROP_HTCCHG_EXT, val);
	return count;
}

static ssize_t htcchg_icl(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	struct power_supply * psy = (struct power_supply *)dev_get_drvdata(dev);
	struct htcchg_chip *chip = container_of(psy,
				struct htcchg_chip, htcchg_psy);
	int val = 0;
	int rc = 0;

	rc = kstrtoint(buf, 10, &val);
	if (rc)
		return count;

	htcchg_set_property_on_battery(chip, POWER_SUPPLY_PROP_HTCCHG_ICL, val);
	return count;
}

static ssize_t htcchg_fcc(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	struct power_supply * psy = (struct power_supply *)dev_get_drvdata(dev);
	struct htcchg_chip *chip = container_of(psy,
				struct htcchg_chip, htcchg_psy);
	int val = 0;
	int rc = 0;

	rc = kstrtoint(buf, 10, &val);
	if (rc)
		return count;

	htcchg_set_property_on_battery(chip, POWER_SUPPLY_PROP_HTCCHG_FCC, val);
	return count;
}

static ssize_t htcchg_cfg_able(struct device *dev,
                struct device_attribute *attr,
                char *buf)

{
	struct power_supply * psy = (struct power_supply *)dev_get_drvdata(dev);
	struct htcchg_chip *chip = container_of(psy,
				struct htcchg_chip, htcchg_psy);
	int value = 0;

	if(chip) {
	if (get_kernel_flag() & KERNEL_FLAG_ENABLE_BMS_CHARGER_LOG)
		value = 1;
	else
		value = 0;

	}
	return snprintf(buf, 10, "%d\n", value);
}

static ssize_t htcchg_get_config(struct device *dev,
                struct device_attribute *attr,
                char *buf)

{
	struct power_supply * psy = (struct power_supply *)dev_get_drvdata(dev);
	struct htcchg_chip *chip = container_of(psy,
				struct htcchg_chip, htcchg_psy);
	int value = 0;

	if(chip) {
		int num_parameters = sizeof(struct htcchg_config) / sizeof(struct htcchg_parameter);
		struct htcchg_parameter* p;
		int i;

		p = (struct htcchg_parameter*)&(chip->config);

		for(i=0; i<num_parameters; i++) {
			if(0 == strcmp(attr->attr.name, p->name)) {
				value = p->v;
				break;
			}
			p++;
		}
	}
	return snprintf(buf, 10, "%d\n", value);
}

#define HTCCHG_CONFIG(Name)	__ATTR(Name, S_IRUGO, htcchg_get_config, NULL),
static struct device_attribute htcchg_attrs[] = {
	__ATTR(iusb, S_IRUGO, htcchg_get_iusb_ma, NULL),
	__ATTR(soc, S_IRUGO, htcchg_get_soc, NULL),
	__ATTR(cfg_able, S_IRUGO, htcchg_cfg_able, NULL),
	__ATTR(control, S_IWUSR | S_IWGRP, NULL, htcchg_control),
	__ATTR(icl, S_IWUSR | S_IWGRP, NULL, htcchg_icl),
	__ATTR(fcc, S_IWUSR | S_IWGRP, NULL, htcchg_fcc),

	#include <linux/power/htc_charger_cfg.h>
};
#undef HTCCHG_CONFIG

static int htcchg_create_attrs(struct device *dev)
{
	int i = 0, rc = 0;

	for (i = 0; i < ARRAY_SIZE(htcchg_attrs); i++) {
		rc = device_create_file(dev, &htcchg_attrs[i]);
		if (rc)
			goto htc_attrs_failed;
	}
	goto succeed;

htc_attrs_failed:
	while (i--)
		device_remove_file(dev, &htcchg_attrs[i]);
succeed:
	return rc;
}

static int htcchg_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct htcchg_chip *chip = container_of(psy,
				struct htcchg_chip, htcchg_psy);
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->charge_disallowed ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		val->intval = chip->flash_actived;
		break;
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		val->intval = chip->path_status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = htcchg_get_usb_in_isen_current_ma(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW_RAW:
		val->intval = htcchg_get_usb_in_isen_adc(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = htcchg_get_vbusdet_adc(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = htcchg_get_usb_pwr_temp_adc(chip);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->temp_level;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int htcchg_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	int usb_type=0;
	struct htcchg_chip *chip = container_of(psy,
				struct htcchg_chip, htcchg_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		pr_htcchg(PR_STATUS, "POWER_SUPPLY_PROP_CHARGING_ENABLED set: %d.\n", val->intval);
		chip->charge_disallowed = val->intval ? 0 : 1;
		rc = htcchg_en_control(chip, chip->charge_enabled);
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		pr_htcchg(PR_STATUS, "POWER_SUPPLY_PROP_FLASH_ACTIVE set: %d.\n", val->intval);
		chip->flash_actived = val->intval;
		rc = htcchg_en_control(chip, chip->charge_enabled);
		break;
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		pr_htcchg(PR_STATUS, "POWER_SUPPLY_PROP_CHARGE_ENABLED set: %d.\n", val->intval);
		rc = htcchg_en_control(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		pr_htcchg(PR_STATUS, "POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL set: %d.\n", val->intval);
		chip->temp_level = val->intval;
		if(0 == htcchg_get_property_from_usb(chip, POWER_SUPPLY_PROP_TYPE, &usb_type)) {
			pr_htcchg(PR_STATUS, "USB Type: %d\n", usb_type);
			if(usb_type == POWER_SUPPLY_TYPE_USB_HVDCP_3) {
				power_supply_changed(&chip->htcchg_psy);
			}
		}
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int htcchg_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static enum power_supply_property htcchg_properties[] = {
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL
};

static void htcchg_check_worker(struct work_struct *work)
{
	struct htcchg_chip *chip = container_of((struct delayed_work*)work, struct htcchg_chip, chg_check_work);
	int iusb = 0;

	if(chip) {
		pr_info("CHG_EN = %d\n", chip->charge_enabled);
		if( chip->charge_enabled == CHARGE_ENABLE) {
			iusb = htcchg_get_usb_in_isen_current_ma(chip);
			pr_info("iusb =%d mA\n", iusb);
			if(iusb <= 200) {
				pr_info("Detect cable out, turn off charger path.\n");
				htcchg_en_control(chip, CHARGE_DISABLE);
			} else if(chip->watchdog_timer <= 0) {
				pr_err("Watchdog timeout, turn off charger path.\n");
				htcchg_en_control(chip, CHARGE_DISABLE);
			} else {
				if (chip->chg_ready_gpio > 0 && gpio_is_valid(chip->chg_ready_gpio)) {
					if( iusb < 1800) {
						/* Keep track until iUSB stable */
						gpio_direction_output(chip->chg_ready_gpio, 0);
						schedule_delayed_work(&chip->chg_check_work, msecs_to_jiffies(CHG_READY_CHECK_PERIOD_MS));
						chip->watchdog_timer -= CHG_READY_CHECK_PERIOD_MS;
					} else {
						gpio_direction_input(chip->chg_ready_gpio);
						schedule_delayed_work(&chip->chg_check_work, msecs_to_jiffies(CHG_WORKER_CHECK_PERIOD_MS));
						chip->watchdog_timer -= CHG_WORKER_CHECK_PERIOD_MS;
					}
				} else {
					schedule_delayed_work(&chip->chg_check_work, msecs_to_jiffies(CHG_IUSB_POLL_PERIOD_MS));
					chip->watchdog_timer -= CHG_IUSB_POLL_PERIOD_MS;
				}
			}
		}
	}
}

static int htcchg_parse_dt(struct htcchg_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc = 0;
	int num_parameters = sizeof(struct htcchg_config) / sizeof(struct htcchg_parameter);
	struct htcchg_parameter* p;
	int i;
	u32 value;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}
	/* read the bms power supply name */
	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = "bms";

	/* read the battery power supply name */
	rc = of_property_read_string(node, "qcom,battery-psy-name",
						&chip->battery_psy_name);
	if (rc)
		chip->battery_psy_name = "battery";

	/* read the usb power supply name */
	rc = of_property_read_string(node, "qcom,usb-psy-name",
						&chip->usb_psy_name);
	if (rc)
		chip->usb_psy_name = "usb";

	chip->chg_enable_gpio = of_get_named_gpio(node, "qcom,charger-enable-gpio", 0);
	if (!gpio_is_valid(chip->chg_enable_gpio)) {
		dev_err(chip->dev, "qcom,charger-enable-gpio not valid: %d\n", chip->chg_enable_gpio);
		chip->chg_enable_gpio = 0;
	} else {
		dev_info(chip->dev, "qcom,charger-enable-gpio: %d\n", chip->chg_enable_gpio);
	}

	chip->chg_ready_gpio = of_get_named_gpio(node, "qcom,charger-ready-gpio", 0);
	if (!gpio_is_valid(chip->chg_ready_gpio)) {
		dev_err(chip->dev, "qcom,charger-ready-gpio not valid: %d\n", chip->chg_ready_gpio);
		chip->chg_ready_gpio = 0;
	} else {
		dev_info(chip->dev, "qcom,charger-ready-gpio: %d\n", chip->chg_ready_gpio);
	}

	chip->adc_sw_sel_gpio = of_get_named_gpio(node, "qcom,adc-sw-sel-gpio", 0);
	if (!gpio_is_valid(chip->adc_sw_sel_gpio)) {
		dev_err(chip->dev, "qcom,adc-sw-sel-gpio not valid: %d\n", chip->adc_sw_sel_gpio);
		chip->adc_sw_sel_gpio = 0;
	} else {
		dev_info(chip->dev, "qcom,adc-sw-sel-gpio: %d\n", chip->adc_sw_sel_gpio);
	}

	chip->vadc_usb_in_isen = qpnp_get_vadc(chip->dev, "usb_in_isen");
	if(0 > of_property_read_u32(node, "qcom,usb_in_isen-channel", &chip->vadc_usb_in_isen_channel)) {
		dev_err(chip->dev, "Failed to get vadc_usb_in_isen_channel\n");
		chip->vadc_usb_in_isen_channel = 0;
	} else {
		dev_info(chip->dev, "vadc_usb_in_isen_channel: %d\n", chip->vadc_usb_in_isen_channel);
	}

	chip->vadc_vbusdet = qpnp_get_vadc(chip->dev, "vbusdet");
	if(0 > of_property_read_u32(node, "qcom,vbusdet-channel", &chip->vadc_vbusdet_channel)) {
		dev_err(chip->dev, "Failed to get vadc_vbusdet_channel\n");
		chip->vadc_vbusdet_channel = 0;
	} else {
		dev_info(chip->dev, "vadc_vbusdet_channel: %d\n", chip->vadc_vbusdet_channel);
	}

	chip->vadc_usb_pwr_temp = qpnp_get_vadc(chip->dev, "usb_pwr_temp");
	if(0 > of_property_read_u32(node, "qcom,usb_pwr_temp-channel", &chip->vadc_usb_pwr_temp_channel)) {
		dev_err(chip->dev, "Failed to get vadc_usb_pwr_temp_channel\n");
		chip->vadc_usb_pwr_temp_channel = 0;
	} else {
		dev_info(chip->dev, "vadc_usb_pwr_temp_channel: %d\n", chip->vadc_usb_pwr_temp_channel);
	}

	if(0 > of_property_read_u32(node, "iusb_rsen", &chip->iusb_rsen)) {
		dev_err(chip->dev, "Failed to get iusb_rsen\n");
	} else {
		dev_info(chip->dev, "iusb_rsen: %d\n", chip->iusb_rsen);
	}

	if(0 > of_property_read_u32(node, "iusb_multiplier", &chip->iusb_multiplier)) {
		dev_err(chip->dev, "Failed to get iusb_multiplier\n");
	} else {
		dev_info(chip->dev, "iusb_multiplier: %d\n", chip->iusb_multiplier);
	}

	p = (struct htcchg_parameter*)&(chip->config);
	dev_info(chip->dev, "== HTCCHG Parameters ==\n");

	for(i=0; i<num_parameters; i++) {
		p->name = config_name[i];
		if(0 > of_property_read_u32(node, p->name, &value)) {
			dev_err(chip->dev, "Failed to get %s\n", p->name);
			p->v = -1;
		} else {
			p->v = value;
			dev_info(chip->dev, "%s: %d\n",p->name, p->v);
		}
		p++;
	}
	return 0;
}


int htcchg_pinctrl_control(struct htcchg_chip *chip)
{
       int ret = 0;

       /* Get pinctrl if target uses pinctrl */
       chip->pinctrl = devm_pinctrl_get(chip->dev);
       if (IS_ERR_OR_NULL(chip->pinctrl)) {
               pr_err("Target does not use pinctrl\n");
               ret = PTR_ERR(chip->pinctrl);
               chip->pinctrl = NULL;
               return ret;
       }

       chip->gpio_state_init = pinctrl_lookup_state(chip->pinctrl, "htcchg_gpio_init");
       if (IS_ERR_OR_NULL(chip->gpio_state_init)) {
               pr_err("Cannot get pintctrl state\n");
               ret = PTR_ERR(chip->gpio_state_init);
               chip->pinctrl = NULL;
               return ret;
       }

       ret = pinctrl_select_state(chip->pinctrl, chip->gpio_state_init);
       if (ret) {
               pr_err("Cannot init gpio\n");
               return ret;
       }
       return 0;
}

static int htcchg_probe(struct platform_device *pdev)
{
	int rc;
	struct htcchg_chip *chip;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, chip);

	rc = htcchg_parse_dt(chip);
	if (rc < 0) {
		dev_err(&pdev->dev, "Unable to parse DT nodes: %d\n", rc);
		return rc;
	}

	mutex_init(&chip->access_lock);
	INIT_DELAYED_WORK(&chip->chg_check_work, htcchg_check_worker);

	chip->htcchg_psy.name		= "htcchg";
	chip->htcchg_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->htcchg_psy.get_property	= htcchg_get_property;
	chip->htcchg_psy.set_property	= htcchg_set_property;
	chip->htcchg_psy.properties	= htcchg_properties;
	chip->htcchg_psy.num_properties	= ARRAY_SIZE(htcchg_properties);
	chip->htcchg_psy.property_is_writeable = htcchg_is_writeable;
	chip->htcchg_psy.external_power_changed = NULL;

	rc = power_supply_register(chip->dev, &chip->htcchg_psy);
	if (rc < 0) {
		dev_err(chip->dev,
			"Unable to register htcchg_psy rc = %d\n", rc);
		goto out;
	}

	htcchg_create_attrs(chip->htcchg_psy.dev);

	chip->psy_registered = true;

	dev_info(chip->dev, "HTC Charger successfully probed.\n");

	if (get_kernel_flag() & KERNEL_FLAG_ENABLE_BMS_CHARGER_LOG)
		htcchg_debug_mask = 0xFF;
	else
		htcchg_debug_mask = 0x06;
	dev_info(chip->dev, "htcchg_debug_mask=0x%X\n", htcchg_debug_mask);

	rc = htcchg_pinctrl_control(chip);
	if (rc) {
		dev_err(chip->dev,"pinctrl error, rc=%d\n", rc);
	}

	return 0;

out:
	return rc;
}

static int htcchg_remove(struct platform_device *pdev)
{
	struct htcchg_chip *chip = dev_get_drvdata(&pdev->dev);

	power_supply_unregister(&chip->htcchg_psy);

	return 0;
}

static struct of_device_id htcchg_match_table[] = {
	{
		.compatible     = "htc,htc-charger",
	},
	{ },
};

static struct platform_driver htcchg_driver = {
	.probe		= htcchg_probe,
	.remove		= htcchg_remove,
	.driver		= {
		.name		= "htc-charger",
		.owner		= THIS_MODULE,
		.of_match_table = htcchg_match_table,
	},
};

static int __init htc_htcchg_init(void)
{
	return platform_driver_register(&htcchg_driver);
}

static void __exit htc_htcchg_exit(void)
{
	return platform_driver_unregister(&htcchg_driver);
}

module_init(htc_htcchg_init);
module_exit(htc_htcchg_exit);

MODULE_DESCRIPTION("HTC Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:htc-charger");
