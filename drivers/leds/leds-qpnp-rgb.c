
/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/pwm.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define LED_TRIGGER_DEFAULT		"none"

#define RGB_LED_SRC_SEL(base)		(base + 0x45)
#define RGB_LED_EN_CTL(base)		(base + 0x46)
#define RGB_LED_ATC_CTL(base)		(base + 0x47)

#define RGB_MAX_LEVEL			LED_FULL
#define RGB_LED_ENABLE_RED		0x80
#define RGB_LED_ENABLE_GREEN		0x40
#define RGB_LED_ENABLE_BLUE		0x20
#define RGB_LED_SOURCE_VPH_PWR		0x01
#define RGB_LED_ENABLE_MASK		0xE0
#define RGB_LED_SRC_MASK		0x03
#define QPNP_LED_PWM_FLAGS	(PM_PWM_LUT_LOOP | PM_PWM_LUT_RAMP_UP)
#define QPNP_LUT_RAMP_STEP_DEFAULT	255
#define	PWM_LUT_MAX_SIZE		63
#define	PWM_GPLED_LUT_MAX_SIZE		31
#define RGB_LED_DISABLE			0x00

#define MPP_MAX_LEVEL			LED_FULL
#define LED_MPP_MODE_CTRL(base)		(base + 0x40)
#define LED_MPP_VIN_CTRL(base)		(base + 0x41)
#define LED_MPP_EN_CTRL(base)		(base + 0x46)
#define LED_MPP_SINK_CTRL(base)		(base + 0x4C)

#define LED_MPP_CURRENT_MIN		5
#define LED_MPP_CURRENT_MAX		40
#define LED_MPP_VIN_CTRL_DEFAULT	0
#define LED_MPP_CURRENT_PER_SETTING	5
#define LED_MPP_SOURCE_SEL_DEFAULT	LED_MPP_MODE_ENABLE

#define LED_MPP_SINK_MASK		0x07
#define LED_MPP_MODE_MASK		0x7F
#define LED_MPP_VIN_MASK		0x03
#define LED_MPP_EN_MASK			0x80
#define LED_MPP_SRC_MASK		0x0F
#define LED_MPP_MODE_CTRL_MASK		0x70

#define LED_MPP_MODE_SINK		(0x06 << 4)
#define LED_MPP_MODE_ENABLE		0x01
#define LED_MPP_MODE_OUTPUT		0x10
#define LED_MPP_MODE_DISABLE		0x00
#define LED_MPP_EN_ENABLE		0x80
#define LED_MPP_EN_DISABLE		0x00

#define MPP_SOURCE_DTEST1		0x08

#define LED_DBG(fmt, ...) \
		({ if (0) printk(KERN_DEBUG "[LED]" fmt, ##__VA_ARGS__); })
#define LED_INFO(fmt, ...) \
		printk(KERN_INFO "[LED]" fmt, ##__VA_ARGS__)
#define LED_ERR(fmt, ...) \
		printk(KERN_ERR "[LED][ERR]" fmt, ##__VA_ARGS__)

#define VIRTUAL_RAMP_SETP_TIME	25
#define VIRTUAL_LUT_START		0
#define VIRTUAL_LUT_LEN			10
#define AMBER_SHORT_LUT_START	VIRTUAL_LUT_START + VIRTUAL_LUT_LEN
#define GREEN_SHORT_LUT_START	AMBER_SHORT_LUT_START + SHORT_LUT_LEN
#define BLUE_SHORT_LUT_START		GREEN_SHORT_LUT_START + SHORT_LUT_LEN
#define SHORT_LUT_LEN			2
#define AMBER_LONG_LUT_START	BLUE_SHORT_LUT_START + SHORT_LUT_LEN
#define GREEN_LONG_LUT_START	AMBER_LONG_LUT_START + LONG_LUT_LEN
#define BLUE_LONG_LUT_START		GREEN_LONG_LUT_START + LONG_LUT_LEN
#define LONG_LUT_LEN			2

#define MAIN_TOUCH_SOLUTION 2
#define SEC_TOUCH_SOLUTION 1

static int amber_short_lut [2] = {0};
static int green_short_lut [2] = {0};
static int blue_short_lut [2] = {0};
static int amber_long_lut [2] = {0};
static int green_long_lut [2] = {0};
static int blue_long_lut [2] = {0};
static int lut_on [] = {0, 0, 10, 30, 45, 60, 75, 90, 100};
static int lut_off [] = {100, 100, 90, 75, 60, 45, 30, 10, 0};
static uint16_t g_led_touch_solution = MAIN_TOUCH_SOLUTION;
static u8 color_table[20] = {0};
static int use_color_table = 0;
static u8 current_table[20] = {0};
static int use_current_table = 0;
static uint32_t ModeRGB;
static u8 indicator_pwm_ratio = 255;
static u8 virtual_key_led_ignore_flag = 0;
#define Mode_Mask (0xff << 24)
#define Red_Mask (0xff << 16)
#define Green_Mask (0xff << 8)
#define Blue_Mask 0xff

module_param(indicator_pwm_ratio, byte, S_IRUSR | S_IWUSR);
module_param(virtual_key_led_ignore_flag, byte, S_IRUSR | S_IWUSR);


enum qpnp_leds {
	QPNP_ID_WLED = 0,
	QPNP_ID_FLASH1_LED0,
	QPNP_ID_FLASH1_LED1,
	QPNP_ID_RGB_RED,
	QPNP_ID_RGB_GREEN,
	QPNP_ID_RGB_BLUE,
	QPNP_ID_LED_MPP,
	QPNP_ID_KPDBL,
	QPNP_ID_LED_GPIO,
	QPNP_ID_MAX,
};

struct wake_lock pmic_led_rgb_wake_lock[QPNP_ID_MAX];

enum rgb_mode {
	RGB_MODE_PWM = 0,
	RGB_MODE_LPG,
	RGB_MANUAL_MODE,
};
enum led_mode {
	PWM_MODE = 0,
	LPG_MODE,
	MANUAL_MODE,
};

enum led_status {
	OFF = 0,
	ON,
	BLINK,
};

enum pm8xxx_blink_type {
	BLINK_STOP = -1,
	BLINK_UNCHANGE = 0,
	BLINK_64MS_PER_2SEC,
	BLINK_64MS_ON_310MS_PER_2SEC,
	BLINK_64MS_ON_2SEC_PER_2SEC,
	BLINK_1SEC_PER_2SEC,
};

static u8 rgb_pwm_debug_regs[] = {
	0x45, 0x46, 0x47,
};

static u8 mpp_debug_regs[] = {
	0x40, 0x41, 0x42, 0x45, 0x46, 0x4c,
};

struct pwm_config_data {
	struct lut_params	lut_params;
	struct pwm_device	*pwm_dev;
	u32			pwm_period_us;
	u32                     pwm_duty_us;
	struct pwm_duty_cycles	*duty_cycles;
	int	*old_duty_pcts;
	u8	mode;
	u8	default_mode;
	bool	pwm_enabled;
	bool use_blink;
	bool blinking;
	int 	pwm_coefficient;
};

struct mpp_config_data {
	struct pwm_config_data	*pwm_cfg;
	u8	current_setting;
	u8	source_sel;
	u8	mode_ctrl;
	u8	vin_ctrl;
	u8	min_brightness;
	u8 pwm_mode;
	u32	max_uV;
	u32	min_uV;
	struct regulator *mpp_reg;
	bool	enable;
	u8 blink_mode;
};

struct rgb_config_data {
	struct pwm_config_data	*pwm_cfg;
	u8	enable;
};

struct qpnp_led_data {
	struct led_classdev	cdev;
	struct spmi_device	*spmi_dev;
	struct delayed_work	dwork;
	struct workqueue_struct *workqueue;
	struct work_struct	work;
	int			id;
	u16			base;
	u8			reg;
	u8			num_leds;
	struct mutex		lock;
	struct rgb_config_data	*rgb_cfg;
	struct mpp_config_data	*mpp_cfg;
	int			max_current;
	bool			default_on;
	bool                    in_order_command_processing;
	struct delayed_work	blink_delayed_work;
	struct delayed_work	gpled_blink_delayed_work;
	struct delayed_work 	fade_delayed_work;
	int			turn_off_delay_ms;
	int 		base_pwm;
	uint8_t last_brightness;
	uint8_t current_setting;
	
	struct work_struct 		led_off_work;
	int status;
	int mode;
	struct work_struct 		led_blink_work;
	struct work_struct		led_multicolor_work;
#ifdef CONFIG_FB
	struct notifier_block fb_notifier;
#endif
};
static struct workqueue_struct *g_led_work_queue;
static struct workqueue_struct *g_gpled_work_queue;
static struct workqueue_struct *g_led_on_work_queue;
static struct qpnp_led_data *g_led_red = NULL, *g_led_green = NULL, *g_led_blue = NULL, *g_led_virtual = NULL;


static int current_blink = 0;
static int table_level_num = 0;
static DEFINE_MUTEX(flash_lock);

static int
qpnp_led_masked_write(struct qpnp_led_data *led, u16 addr, u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = spmi_ext_register_readl(led->spmi_dev->ctrl, led->spmi_dev->sid,
		addr, &reg, 1);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Unable to read from addr=%x, rc(%d)\n", addr, rc);
	}

	reg &= ~mask;
	reg |= val;

	rc = spmi_ext_register_writel(led->spmi_dev->ctrl, led->spmi_dev->sid,
		addr, &reg, 1);
	if (rc)
		dev_err(&led->spmi_dev->dev,
			"Unable to write to addr=%x, rc(%d)\n", addr, rc);
	return rc;
}

static void qpnp_dump_regs(struct qpnp_led_data *led, u8 regs[], u8 array_size)
{
	int i;
	u8 val;

	pr_debug("===== %s LED register dump start =====\n", led->cdev.name);
	for (i = 0; i < array_size; i++) {
		spmi_ext_register_readl(led->spmi_dev->ctrl,
					led->spmi_dev->sid,
					led->base + regs[i],
					&val, sizeof(val));
		pr_debug("%s: 0x%x = 0x%x\n", led->cdev.name,
					led->base + regs[i], val);
	}
	pr_debug("===== %s LED register dump end =====\n", led->cdev.name);
}

static void virtual_key_lut_table_set(int *virtual_key_lut_table, int array_len, int base_pwm, uint8_t target_pwm, uint8_t last_pwm)
{
	int i;
	uint8_t pwm_diff;

	virtual_key_lut_table[0] = base_pwm + (int)last_pwm * (100 - base_pwm) / 255;
	virtual_key_lut_table[array_len - 1] = base_pwm + (int)target_pwm * (100 - base_pwm) / 255;

	if (target_pwm > last_pwm) {
		pwm_diff = target_pwm - last_pwm;
		for(i = 1;i < array_len - 1;i++)
			virtual_key_lut_table[i] = virtual_key_lut_table[0] + (pwm_diff * (100 - base_pwm) / 255) * i / (array_len - 1);
	} else {
		pwm_diff = last_pwm - target_pwm;
		for(i = 1;i < array_len - 1;i++)
			virtual_key_lut_table[i] = virtual_key_lut_table[0] - (pwm_diff * (100 - base_pwm) / 255) * i / (array_len - 1);
	}
}

static int qpnp_mpp_set(struct qpnp_led_data *led)
{
	int rc;
	u8 val;
	int duty_us, duty_ns, period_us;
	int virtual_key_lut_table[VIRTUAL_LUT_LEN] = {0};

	LED_INFO("%s, name:%s, brightness = %d status: %d\n", __func__, led->cdev.name, led->cdev.brightness, led->status);

	if(virtual_key_led_ignore_flag)
		return 0;

	if(use_color_table && led->cdev.brightness < table_level_num){
		if(use_current_table) {
			if(led->current_setting != current_table[led->cdev.brightness] && led->cdev.brightness != 0) {
				led->current_setting = current_table[led->cdev.brightness];
				val = led->current_setting;
				if (val < LED_MPP_CURRENT_MIN)
					val = LED_MPP_CURRENT_MIN;
				else if (val > LED_MPP_CURRENT_MAX)
					val = LED_MPP_CURRENT_MAX;
				else {
					val /= LED_MPP_CURRENT_MIN;
					val *= LED_MPP_CURRENT_MIN;
				}

				val = (val / LED_MPP_CURRENT_MIN) - 1;

				rc = qpnp_led_masked_write(led,
						LED_MPP_SINK_CTRL(led->base),
						LED_MPP_SINK_MASK, val);
				if (rc) {
					LED_ERR("Failed to write sink control reg\n");
					return rc;
				}
			}
		}
		LED_INFO("color_table[%d] = %d, current_table[%d] = %d\n", led->cdev.brightness, color_table[led->cdev.brightness], led->cdev.brightness, current_table[led->cdev.brightness]);
		led->cdev.brightness = color_table[led->cdev.brightness];
	}

	if (led->cdev.brightness == led->last_brightness) {
		LED_INFO("%s, brightness no change, return\n", __func__);
		return 0;
	}

	if (led->cdev.brightness) {
		if (led->mpp_cfg->mpp_reg && !led->mpp_cfg->enable) {
			rc = regulator_set_voltage(led->mpp_cfg->mpp_reg,
					led->mpp_cfg->min_uV,
					led->mpp_cfg->max_uV);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Regulator voltage set failed rc=%d\n",
									rc);
				return rc;
			}

			rc = regulator_enable(led->mpp_cfg->mpp_reg);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Regulator enable failed(%d)\n", rc);
				goto err_reg_enable;
			}
		}

		led->mpp_cfg->enable = true;

		if (led->cdev.brightness < led->mpp_cfg->min_brightness) {
			dev_warn(&led->spmi_dev->dev,
				"brightness is less than supported..." \
				"set to minimum supported\n");
			led->cdev.brightness = led->mpp_cfg->min_brightness;
		}

		if (led->mpp_cfg->pwm_mode != MANUAL_MODE) {
			if (!led->mpp_cfg->pwm_cfg->blinking) {
				led->mpp_cfg->pwm_cfg->mode =
					led->mpp_cfg->pwm_cfg->default_mode;
				led->mpp_cfg->pwm_mode =
					led->mpp_cfg->pwm_cfg->default_mode;
			}
		}
		if (led->mpp_cfg->pwm_mode == PWM_MODE) {
			
			period_us = led->mpp_cfg->pwm_cfg->pwm_period_us;
			if (period_us > INT_MAX / NSEC_PER_USEC) {
				duty_us = (period_us * led->cdev.brightness) /
					LED_FULL;
				rc = pwm_config_us(
					led->mpp_cfg->pwm_cfg->pwm_dev,
					duty_us,
					period_us);
			} else {
				duty_ns = ((period_us * NSEC_PER_USEC) /
					LED_FULL) * led->cdev.brightness;
				rc = pwm_config(
					led->mpp_cfg->pwm_cfg->pwm_dev,
					duty_ns,
					period_us * NSEC_PER_USEC);
			}
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev, "Failed to " \
					"configure pwm for new values\n");
				goto err_mpp_reg_write;
			}
		}

		if (led->mpp_cfg->pwm_mode == LPG_MODE) {
		led->mpp_cfg->pwm_cfg->lut_params.flags = PM_PWM_LUT_RAMP_UP;
		led->mpp_cfg->pwm_cfg->lut_params.start_idx = VIRTUAL_LUT_START;
		led->mpp_cfg->pwm_cfg->lut_params.idx_len = VIRTUAL_LUT_LEN;
		led->mpp_cfg->pwm_cfg->lut_params.ramp_step_ms = VIRTUAL_RAMP_SETP_TIME;
		led->mpp_cfg->pwm_cfg->lut_params.lut_pause_hi = 0;
		led->mpp_cfg->pwm_cfg->lut_params.lut_pause_lo = 0;
		virtual_key_lut_table_set(virtual_key_lut_table, VIRTUAL_LUT_LEN, led->base_pwm, led->cdev.brightness, led->last_brightness);
		led->last_brightness = led->cdev.brightness;
		rc = pwm_lut_config(led->mpp_cfg->pwm_cfg->pwm_dev,
					PM_PWM_PERIOD_MIN,
					virtual_key_lut_table,
					led->mpp_cfg->pwm_cfg->lut_params);
		}

		if (led->mpp_cfg->pwm_mode != MANUAL_MODE)
			pwm_enable(led->mpp_cfg->pwm_cfg->pwm_dev);
		else {
			if (led->cdev.brightness < LED_MPP_CURRENT_MIN)
				led->cdev.brightness = LED_MPP_CURRENT_MIN;
			else {
				led->cdev.brightness /= LED_MPP_CURRENT_MIN;
				led->cdev.brightness *= LED_MPP_CURRENT_MIN;
			}

			val = (led->cdev.brightness / LED_MPP_CURRENT_MIN) - 1;

			rc = qpnp_led_masked_write(led,
					LED_MPP_SINK_CTRL(led->base),
					LED_MPP_SINK_MASK, val);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Failed to write sink control reg\n");
				goto err_mpp_reg_write;
			}
		}

		val = (led->mpp_cfg->source_sel & LED_MPP_SRC_MASK) |
			(led->mpp_cfg->mode_ctrl & LED_MPP_MODE_CTRL_MASK);

		rc = qpnp_led_masked_write(led,
			LED_MPP_MODE_CTRL(led->base), LED_MPP_MODE_MASK,
			val);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
					"Failed to write led mode reg\n");
			goto err_mpp_reg_write;
		}

		rc = qpnp_led_masked_write(led,
				LED_MPP_EN_CTRL(led->base), LED_MPP_EN_MASK,
				LED_MPP_EN_ENABLE);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
					"Failed to write led enable " \
					"reg\n");
			goto err_mpp_reg_write;
		}
	} else {
		if (led->mpp_cfg->pwm_mode == LPG_MODE) {
			led->mpp_cfg->pwm_cfg->lut_params.flags = PM_PWM_LUT_RAMP_UP;
			led->mpp_cfg->pwm_cfg->lut_params.start_idx = VIRTUAL_LUT_START;
			led->mpp_cfg->pwm_cfg->lut_params.idx_len = VIRTUAL_LUT_LEN;
			led->mpp_cfg->pwm_cfg->lut_params.ramp_step_ms = VIRTUAL_RAMP_SETP_TIME;
			led->mpp_cfg->pwm_cfg->lut_params.lut_pause_hi = 0;
			led->mpp_cfg->pwm_cfg->lut_params.lut_pause_lo = 0;
			virtual_key_lut_table_set(virtual_key_lut_table, VIRTUAL_LUT_LEN, led->base_pwm, led->cdev.brightness, led->last_brightness);
			led->last_brightness = led->cdev.brightness;
			rc = pwm_lut_config(led->mpp_cfg->pwm_cfg->pwm_dev,
					PM_PWM_PERIOD_MIN,
					virtual_key_lut_table,
					led->mpp_cfg->pwm_cfg->lut_params);
			pwm_enable(led->mpp_cfg->pwm_cfg->pwm_dev);
			queue_delayed_work(g_led_work_queue, &led->fade_delayed_work,
				msecs_to_jiffies(led->mpp_cfg->pwm_cfg->lut_params.ramp_step_ms * led->mpp_cfg->pwm_cfg->lut_params.idx_len));
		} else {
			if (led->mpp_cfg->pwm_mode != MANUAL_MODE) {
				led->mpp_cfg->pwm_cfg->mode =
					led->mpp_cfg->pwm_cfg->default_mode;
				led->mpp_cfg->pwm_mode =
					led->mpp_cfg->pwm_cfg->default_mode;
				pwm_disable(led->mpp_cfg->pwm_cfg->pwm_dev);
			}
			rc = qpnp_led_masked_write(led,
						LED_MPP_MODE_CTRL(led->base),
						LED_MPP_MODE_MASK,
						LED_MPP_MODE_DISABLE);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
						"Failed to write led mode reg\n");
				goto err_mpp_reg_write;
			}

			rc = qpnp_led_masked_write(led,
						LED_MPP_EN_CTRL(led->base),
						LED_MPP_EN_MASK,
						LED_MPP_EN_DISABLE);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
						"Failed to write led enable reg\n");
				goto err_mpp_reg_write;
			}

			if (led->mpp_cfg->mpp_reg && led->mpp_cfg->enable) {
				rc = regulator_disable(led->mpp_cfg->mpp_reg);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"MPP regulator disable failed(%d)\n",
						rc);
					return rc;
				}

				rc = regulator_set_voltage(led->mpp_cfg->mpp_reg,
							0, led->mpp_cfg->max_uV);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"MPP regulator voltage set failed(%d)\n",
						rc);
					return rc;
				}
			}
			led->mpp_cfg->enable = false;
		}
	}

	if (led->mpp_cfg->pwm_mode != MANUAL_MODE)
		led->mpp_cfg->pwm_cfg->blinking = false;
	qpnp_dump_regs(led, mpp_debug_regs, ARRAY_SIZE(mpp_debug_regs));

	return 0;

err_mpp_reg_write:
	if (led->mpp_cfg->mpp_reg)
		regulator_disable(led->mpp_cfg->mpp_reg);
err_reg_enable:
	if (led->mpp_cfg->mpp_reg)
		regulator_set_voltage(led->mpp_cfg->mpp_reg, 0,
							led->mpp_cfg->max_uV);
	led->mpp_cfg->enable = false;

	return rc;
}

static void led_fade_do_work(struct work_struct *work)
{
        struct qpnp_led_data *led;
        int rc;

        led = container_of(work, struct qpnp_led_data, fade_delayed_work.work);

		if (led->id == QPNP_ID_LED_MPP) {
			if (!led->mpp_cfg->pwm_cfg)
				return;
			if(led->cdev.brightness)
				return;
			if (led->mpp_cfg->pwm_mode != MANUAL_MODE) {
				led->mpp_cfg->pwm_cfg->mode =
					led->mpp_cfg->pwm_cfg->default_mode;
				led->mpp_cfg->pwm_mode =
					led->mpp_cfg->pwm_cfg->default_mode;
				pwm_disable(led->mpp_cfg->pwm_cfg->pwm_dev);
			}
			rc = qpnp_led_masked_write(led,
						LED_MPP_MODE_CTRL(led->base),
						LED_MPP_MODE_MASK,
						LED_MPP_MODE_DISABLE);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
						"Failed to write led mode reg\n");
			}

			rc = qpnp_led_masked_write(led,
						LED_MPP_EN_CTRL(led->base),
						LED_MPP_EN_MASK,
						LED_MPP_EN_DISABLE);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
						"Failed to write led enable reg\n");
			}

			if (led->mpp_cfg->mpp_reg && led->mpp_cfg->enable) {
				rc = regulator_disable(led->mpp_cfg->mpp_reg);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"MPP regulator disable failed(%d)\n",
						rc);
					return;
				}

				rc = regulator_set_voltage(led->mpp_cfg->mpp_reg,
							0, led->mpp_cfg->max_uV);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"MPP regulator voltage set failed(%d)\n",
						rc);
					return;
				}
			}
			led->mpp_cfg->enable = false;
		} else {
			pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
			rc = qpnp_led_masked_write(led,RGB_LED_EN_CTL(led->base), led->rgb_cfg->enable, RGB_LED_DISABLE);
			if (rc) {
				dev_err(&led->spmi_dev->dev, "Failed to write led enable reg\n");
			}
		}
}
static void led_blink_do_work(struct work_struct *work)
{
	struct qpnp_led_data *led;
	int rc,val;
	led = container_of(work, struct qpnp_led_data, blink_delayed_work.work);
	switch(led->id) {
		case QPNP_ID_LED_MPP:
			rc = pwm_config_us(led->mpp_cfg->pwm_cfg->pwm_dev, led->mpp_cfg->pwm_cfg->pwm_duty_us, led->mpp_cfg->pwm_cfg->pwm_period_us);
			rc = pwm_enable(led->mpp_cfg->pwm_cfg->pwm_dev);
			
			mdelay(10);
			rc = pwm_enable(led->mpp_cfg->pwm_cfg->pwm_dev);
			val = (led->mpp_cfg->source_sel & LED_MPP_SRC_MASK) |
				(led->mpp_cfg->mode_ctrl & LED_MPP_MODE_CTRL_MASK);

			rc = qpnp_led_masked_write(led,
				LED_MPP_MODE_CTRL(led->base), LED_MPP_MODE_MASK,
				val);
			rc = qpnp_led_masked_write(led,
					LED_MPP_EN_CTRL(led->base), LED_MPP_EN_MASK,
					LED_MPP_EN_ENABLE);
		break;
		case QPNP_ID_RGB_RED:
		case QPNP_ID_RGB_GREEN:
		case QPNP_ID_RGB_BLUE:
			rc = pwm_config_us(led->rgb_cfg->pwm_cfg->pwm_dev, led->rgb_cfg->pwm_cfg->pwm_duty_us, led->rgb_cfg->pwm_cfg->pwm_period_us);
			rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
			
			mdelay(10);
			rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
			led->status = BLINK;
		break;
		default:
		break;
	}
}
static int qpnp_rgb_set(struct qpnp_led_data *led)
{
	int rc;
	LED_INFO("%s, name:%s, brightness = %d status: %d\n", __func__, led->cdev.name, led->cdev.brightness, led->status);

	if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_LPG) {
		cancel_delayed_work(&led->fade_delayed_work);
	}
	if (led->cdev.brightness) {
		if (led->status != ON) {
			if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_PWM) {
				rc = pwm_config_us(led->rgb_cfg->pwm_cfg->pwm_dev, 640 * led->rgb_cfg->pwm_cfg->pwm_coefficient / 255, 640);
				if (rc < 0) {
					dev_err(&led->spmi_dev->dev, "Failed to " \
						"configure pwm for new values\n");
					return rc;
				}

			} else if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_LPG) {
							rc = pwm_lut_config(led->rgb_cfg->pwm_cfg->pwm_dev,
					PM_PWM_PERIOD_MIN,
									lut_on,
									led->rgb_cfg->pwm_cfg->lut_params);
			}

			rc = qpnp_led_masked_write(led,	RGB_LED_EN_CTL(led->base),
				led->rgb_cfg->enable, led->rgb_cfg->enable);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Failed to write led enable reg\n");
				return rc;
			}
			rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
			
			mdelay(10);
			rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
			led->status = ON;
			if (led->cdev.brightness == 9) {
				msleep(60);
				led->rgb_cfg->pwm_cfg->mode = led->rgb_cfg->pwm_cfg->default_mode;
				pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
				rc = qpnp_led_masked_write(led, RGB_LED_EN_CTL(led->base), led->rgb_cfg->enable, RGB_LED_DISABLE);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
							"Failed to write led enable reg\n");
					return rc;
				}
				led->status = OFF;
			}
			if (led->cdev.brightness == 10) {
				msleep(490);
				led->rgb_cfg->pwm_cfg->mode = led->rgb_cfg->pwm_cfg->default_mode;
				pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
				rc = qpnp_led_masked_write(led, RGB_LED_EN_CTL(led->base), led->rgb_cfg->enable, RGB_LED_DISABLE);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
							"Failed to write led enable reg\n");
					return rc;
				}
				led->status = OFF;
			}
		}
	} else {
                if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_LPG) {
			wake_lock_timeout(&pmic_led_rgb_wake_lock[led->id], HZ*2);
                        rc = pwm_lut_config(led->rgb_cfg->pwm_cfg->pwm_dev,
				PM_PWM_PERIOD_MIN,
				lut_off,
                                led->rgb_cfg->pwm_cfg->lut_params);
                        if (rc < 0) {
                                dev_err(&led->spmi_dev->dev, "Failed to " \
                                        "configure pwm LUT\n");
                                return rc;
		}
		rc = qpnp_led_masked_write(led,
			RGB_LED_EN_CTL(led->base),
			led->rgb_cfg->enable, led->rgb_cfg->enable);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"Failed to write led enable reg\n");
			return rc;
		}
		rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
			queue_delayed_work(g_led_work_queue, &led->fade_delayed_work,
				msecs_to_jiffies(led->rgb_cfg->pwm_cfg->lut_params.ramp_step_ms * led->rgb_cfg->pwm_cfg->lut_params.idx_len));
	} else {
		led->rgb_cfg->pwm_cfg->mode =
			led->rgb_cfg->pwm_cfg->default_mode;
		pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
		rc = qpnp_led_masked_write(led,
			RGB_LED_EN_CTL(led->base),
			led->rgb_cfg->enable, RGB_LED_DISABLE);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"Failed to write led enable reg\n");
			return rc;
			}
		}
		led->status = OFF;
	}

	led->rgb_cfg->pwm_cfg->blinking = false;
	qpnp_dump_regs(led, rgb_pwm_debug_regs, ARRAY_SIZE(rgb_pwm_debug_regs));
	return 0;
}

static void qpnp_led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct qpnp_led_data *led;
	bool ret;
	int retry_count = 0;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);
	if (value < LED_OFF) {
		dev_err(&led->spmi_dev->dev, "Invalid brightness value\n");
		return;
	}

	if (value > led->cdev.max_brightness)
		value = led->cdev.max_brightness;

	led->cdev.brightness = value;
	do {
		ret = queue_work(g_led_on_work_queue, &led->work);
		if (!ret) {
			retry_count++;
			msleep(1);
			LED_INFO("%s qpnp_led_set work already on queue, requeue!! ret:%d retry:%d\n", __func__, ret, retry_count);
		}
	} while ((!ret) && (retry_count < 3));
}

static void __qpnp_led_work(struct qpnp_led_data *led,
				enum led_brightness value)
{
	int rc;

		mutex_lock(&led->lock);

	switch (led->id) {
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		rc = qpnp_rgb_set(led);
		if (rc < 0)
			dev_err(&led->spmi_dev->dev,
				"RGB set brightness failed (%d)\n", rc);
		break;
	case QPNP_ID_LED_MPP:
		rc = qpnp_mpp_set(led);
		if (rc < 0)
			dev_err(&led->spmi_dev->dev,
					"MPP set brightness failed (%d)\n", rc);
		break;
	default:
		dev_err(&led->spmi_dev->dev, "Invalid LED(%d)\n", led->id);
		break;
	}
		mutex_unlock(&led->lock);

}

static void qpnp_led_work(struct work_struct *work)
{
	struct qpnp_led_data *led = container_of(work,
					struct qpnp_led_data, work);

	__qpnp_led_work(led, led->cdev.brightness);
	return;
}

static int qpnp_led_set_max_brightness(struct qpnp_led_data *led)
{
	switch (led->id) {
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		led->cdev.max_brightness = RGB_MAX_LEVEL;
		break;
	case QPNP_ID_LED_MPP:
		if (led->mpp_cfg->pwm_mode == MANUAL_MODE)
			led->cdev.max_brightness = led->max_current;
		else
			led->cdev.max_brightness = MPP_MAX_LEVEL;
		break;
	default:
		dev_err(&led->spmi_dev->dev, "Invalid LED(%d)\n", led->id);
		return -EINVAL;
	}

	return 0;
}

static enum led_brightness qpnp_led_get(struct led_classdev *led_cdev)
{
	struct qpnp_led_data *led;

	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	return led->cdev.brightness;
}

static void qpnp_led_turn_off_delayed(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct qpnp_led_data *led
		= container_of(dwork, struct qpnp_led_data, dwork);

	led->cdev.brightness = LED_OFF;
	qpnp_led_set(&led->cdev, led->cdev.brightness);
}

static void qpnp_led_turn_off(struct qpnp_led_data *led)
{
	INIT_DELAYED_WORK(&led->dwork, qpnp_led_turn_off_delayed);
	schedule_delayed_work(&led->dwork,
		msecs_to_jiffies(led->turn_off_delay_ms));
}

static int qpnp_pwm_init(struct pwm_config_data *pwm_cfg,
					struct spmi_device *spmi_dev,
					const char *name)
{
	int rc, start_idx, idx_len, lut_max_size;

	if (pwm_cfg->pwm_dev) {
		if (pwm_cfg->mode == LPG_MODE || pwm_cfg->use_blink) {
			start_idx =
			pwm_cfg->duty_cycles->start_idx;
			idx_len =
			pwm_cfg->duty_cycles->num_duty_pcts;

			lut_max_size = PWM_LUT_MAX_SIZE;

			if (idx_len >= lut_max_size && start_idx) {
				dev_err(&spmi_dev->dev,
					"Wrong LUT size or index\n");
				return -EINVAL;
			}

			if ((start_idx + idx_len) > lut_max_size) {
				dev_err(&spmi_dev->dev,
					"Exceed LUT limit\n");
				return -EINVAL;
			}
			rc = pwm_lut_config(pwm_cfg->pwm_dev,
				pwm_cfg->pwm_period_us,
				pwm_cfg->duty_cycles->duty_pcts,
				pwm_cfg->lut_params);
			if (rc < 0) {
				dev_err(&spmi_dev->dev, "Failed to " \
					"configure pwm LUT\n");
				return rc;
			}
		}
	} else {
		dev_err(&spmi_dev->dev,
			"Invalid PWM device\n");
		return -EINVAL;
	}
	return 0;
}

static ssize_t pwm_us_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u32 pwm_us;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	u32 previous_pwm_us;
	struct pwm_config_data *pwm_cfg;

	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	ret = kstrtou32(buf, 10, &pwm_us);
	if (ret)
		return ret;

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for pwm_us\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	previous_pwm_us = pwm_cfg->pwm_period_us;

	pwm_cfg->pwm_period_us = pwm_us;
	pwm_free(pwm_cfg->pwm_dev);
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret) {
		pwm_cfg->pwm_period_us = previous_pwm_us;
		pwm_free(pwm_cfg->pwm_dev);
		qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
		qpnp_led_set(&led->cdev, led->cdev.brightness);
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm with new pwm_us value\n");
		return ret;
	}
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;
}

static ssize_t pause_lo_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u32 pause_lo;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	u32 previous_pause_lo;
	struct pwm_config_data *pwm_cfg;

	ret = kstrtou32(buf, 10, &pause_lo);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for pause lo\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	previous_pause_lo = pwm_cfg->lut_params.lut_pause_lo;

	pwm_free(pwm_cfg->pwm_dev);
	pwm_cfg->lut_params.lut_pause_lo = pause_lo;
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret) {
		pwm_cfg->lut_params.lut_pause_lo = previous_pause_lo;
		pwm_free(pwm_cfg->pwm_dev);
		qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
		qpnp_led_set(&led->cdev, led->cdev.brightness);
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm with new pause lo value\n");
		return ret;
	}
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;
}

static ssize_t pause_hi_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u32 pause_hi;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	u32 previous_pause_hi;
	struct pwm_config_data *pwm_cfg;

	ret = kstrtou32(buf, 10, &pause_hi);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for pause hi\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	previous_pause_hi = pwm_cfg->lut_params.lut_pause_hi;

	pwm_free(pwm_cfg->pwm_dev);
	pwm_cfg->lut_params.lut_pause_hi = pause_hi;
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret) {
		pwm_cfg->lut_params.lut_pause_hi = previous_pause_hi;
		pwm_free(pwm_cfg->pwm_dev);
		qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
		qpnp_led_set(&led->cdev, led->cdev.brightness);
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm with new pause hi value\n");
		return ret;
	}
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;
}

static ssize_t start_idx_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u32 start_idx;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	u32 previous_start_idx;
	struct pwm_config_data *pwm_cfg;

	ret = kstrtou32(buf, 10, &start_idx);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for start idx\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	previous_start_idx = pwm_cfg->duty_cycles->start_idx;
	pwm_cfg->duty_cycles->start_idx = start_idx;
	pwm_cfg->lut_params.start_idx = pwm_cfg->duty_cycles->start_idx;
	pwm_free(pwm_cfg->pwm_dev);
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret) {
		pwm_cfg->duty_cycles->start_idx = previous_start_idx;
		pwm_cfg->lut_params.start_idx = pwm_cfg->duty_cycles->start_idx;
		pwm_free(pwm_cfg->pwm_dev);
		qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
		qpnp_led_set(&led->cdev, led->cdev.brightness);
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm with new start idx value\n");
		return ret;
	}
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;
}

static ssize_t ramp_step_ms_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u32 ramp_step_ms;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	u32 previous_ramp_step_ms;
	struct pwm_config_data *pwm_cfg;

	ret = kstrtou32(buf, 10, &ramp_step_ms);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for ramp step\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	previous_ramp_step_ms = pwm_cfg->lut_params.ramp_step_ms;

	pwm_free(pwm_cfg->pwm_dev);
	pwm_cfg->lut_params.ramp_step_ms = ramp_step_ms;
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret) {
		pwm_cfg->lut_params.ramp_step_ms = previous_ramp_step_ms;
		pwm_free(pwm_cfg->pwm_dev);
		qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
		qpnp_led_set(&led->cdev, led->cdev.brightness);
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm with new ramp step value\n");
		return ret;
	}
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;
}

static ssize_t lut_flags_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u32 lut_flags;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	u32 previous_lut_flags;
	struct pwm_config_data *pwm_cfg;

	ret = kstrtou32(buf, 10, &lut_flags);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for lut flags\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	previous_lut_flags = pwm_cfg->lut_params.flags;

	pwm_free(pwm_cfg->pwm_dev);
	pwm_cfg->lut_params.flags = lut_flags;
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret) {
		pwm_cfg->lut_params.flags = previous_lut_flags;
		pwm_free(pwm_cfg->pwm_dev);
		qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
		qpnp_led_set(&led->cdev, led->cdev.brightness);
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm with new lut flags value\n");
		return ret;
	}
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;
}

static ssize_t duty_pcts_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	int num_duty_pcts = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	char *buffer;
	ssize_t ret;
	int i = 0;
	int max_duty_pcts;
	struct pwm_config_data *pwm_cfg;
	u32 previous_num_duty_pcts;
	int value;
	int *previous_duty_pcts;

	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		max_duty_pcts = PWM_LUT_MAX_SIZE;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		max_duty_pcts = PWM_LUT_MAX_SIZE;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for duty pcts\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	buffer = (char *)buf;

	for (i = 0; i < max_duty_pcts; i++) {
		if (buffer == NULL)
			break;
		ret = sscanf((const char *)buffer, "%u,%s", &value, buffer);
		pwm_cfg->old_duty_pcts[i] = value;
		num_duty_pcts++;
		if (ret <= 1)
			break;
	}

	if (num_duty_pcts >= max_duty_pcts) {
		dev_err(&led->spmi_dev->dev,
			"Number of duty pcts given exceeds max (%d)\n",
			max_duty_pcts);
		return -EINVAL;
	}

	previous_num_duty_pcts = pwm_cfg->duty_cycles->num_duty_pcts;
	previous_duty_pcts = pwm_cfg->duty_cycles->duty_pcts;

	pwm_cfg->duty_cycles->num_duty_pcts = num_duty_pcts;
	pwm_cfg->duty_cycles->duty_pcts = pwm_cfg->old_duty_pcts;
	pwm_cfg->old_duty_pcts = previous_duty_pcts;
	pwm_cfg->lut_params.idx_len = pwm_cfg->duty_cycles->num_duty_pcts;

	pwm_free(pwm_cfg->pwm_dev);
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret)
		goto restore;

	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;

restore:
	dev_err(&led->spmi_dev->dev,
		"Failed to initialize pwm with new duty pcts value\n");
	pwm_cfg->duty_cycles->num_duty_pcts = previous_num_duty_pcts;
	pwm_cfg->old_duty_pcts = pwm_cfg->duty_cycles->duty_pcts;
	pwm_cfg->duty_cycles->duty_pcts = previous_duty_pcts;
	pwm_cfg->lut_params.idx_len = pwm_cfg->duty_cycles->num_duty_pcts;
	pwm_free(pwm_cfg->pwm_dev);
	qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return ret;
}

static void mpp_blink(struct qpnp_led_data *led,
					struct pwm_config_data *pwm_cfg)
{
	if (led->cdev.brightness) {
		pwm_cfg->blinking = true;
		if (led->id == QPNP_ID_LED_MPP)
			led->mpp_cfg->pwm_mode = LPG_MODE;
		pwm_cfg->mode = LPG_MODE;
	} else {
		pwm_cfg->blinking = false;
		pwm_cfg->mode = pwm_cfg->default_mode;
		if (led->id == QPNP_ID_LED_MPP)
			led->mpp_cfg->pwm_mode = pwm_cfg->default_mode;
	}
	
	
	
	qpnp_led_set(&led->cdev, led->cdev.brightness);
}

static ssize_t blink_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	unsigned long blinking;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &blinking);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);
	led->cdev.brightness = blinking ? led->cdev.max_brightness : 0;

	switch (led->id) {
	case QPNP_ID_LED_MPP:
			mpp_blink(led, led->mpp_cfg->pwm_cfg);
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
			mpp_blink(led, led->rgb_cfg->pwm_cfg);
		break;
	default:
		dev_err(&led->spmi_dev->dev, "Invalid LED id type for blink\n");
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(pwm_us, 0664, NULL, pwm_us_store);
static DEVICE_ATTR(pause_lo, 0664, NULL, pause_lo_store);
static DEVICE_ATTR(pause_hi, 0664, NULL, pause_hi_store);
static DEVICE_ATTR(start_idx, 0664, NULL, start_idx_store);
static DEVICE_ATTR(ramp_step_ms, 0664, NULL, ramp_step_ms_store);
static DEVICE_ATTR(lut_flags, 0664, NULL, lut_flags_store);
static DEVICE_ATTR(duty_pcts, 0664, NULL, duty_pcts_store);
static DEVICE_ATTR(qcom_blink, 0664, NULL, blink_store);

static struct attribute *led_attrs[] = {
	NULL
};

static const struct attribute_group led_attr_group = {
	.attrs = led_attrs,
};

static struct attribute *pwm_attrs[] = {
	&dev_attr_pwm_us.attr,
	NULL
};

static struct attribute *lpg_attrs[] = {
	&dev_attr_pause_lo.attr,
	&dev_attr_pause_hi.attr,
	&dev_attr_start_idx.attr,
	&dev_attr_ramp_step_ms.attr,
	&dev_attr_lut_flags.attr,
	&dev_attr_duty_pcts.attr,
	NULL
};

static struct attribute *blink_attrs[] = {
	&dev_attr_qcom_blink.attr,
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};

static const struct attribute_group lpg_attr_group = {
	.attrs = lpg_attrs,
};

static const struct attribute_group blink_attr_group = {
	.attrs = blink_attrs,
};

static int qpnp_rgb_init(struct qpnp_led_data *led)
{
	int rc;

	rc = qpnp_led_masked_write(led, RGB_LED_SRC_SEL(led->base),
		RGB_LED_SRC_MASK, RGB_LED_SOURCE_VPH_PWR);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Failed to write led source select register\n");
		return rc;
	}

	rc = qpnp_pwm_init(led->rgb_cfg->pwm_cfg, led->spmi_dev,
				led->cdev.name);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm\n");
		return rc;
	}
	
	rc = qpnp_led_masked_write(led, RGB_LED_ATC_CTL(led->base),
		led->rgb_cfg->enable, led->rgb_cfg->enable);

	return 0;
}

static int qpnp_mpp_init(struct qpnp_led_data *led)
{
	int rc;
	u8 val;


	if (led->max_current < LED_MPP_CURRENT_MIN ||
		led->max_current > LED_MPP_CURRENT_MAX) {
		dev_err(&led->spmi_dev->dev,
			"max current for mpp is not valid\n");
		return -EINVAL;
	}

	val = (led->mpp_cfg->current_setting / LED_MPP_CURRENT_PER_SETTING) - 1;

	if (val < 0)
		val = 0;

	rc = qpnp_led_masked_write(led, LED_MPP_VIN_CTRL(led->base),
		LED_MPP_VIN_MASK, led->mpp_cfg->vin_ctrl);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Failed to write led vin control reg\n");
		return rc;
	}

	rc = qpnp_led_masked_write(led, LED_MPP_SINK_CTRL(led->base),
		LED_MPP_SINK_MASK, val);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Failed to write sink control reg\n");
		return rc;
	}

	if (led->mpp_cfg->pwm_mode != MANUAL_MODE) {
		rc = qpnp_pwm_init(led->mpp_cfg->pwm_cfg, led->spmi_dev,
					led->cdev.name);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"Failed to initialize pwm\n");
			return rc;
		}
	}

	return 0;
}

static int qpnp_led_initialize(struct qpnp_led_data *led)
{
	int rc = 0;

	switch (led->id) {
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		rc = qpnp_rgb_init(led);
		if (rc)
			dev_err(&led->spmi_dev->dev,
				"RGB initialize failed(%d)\n", rc);
		break;
	case QPNP_ID_LED_MPP:
		rc = qpnp_mpp_init(led);
		if (rc)
			dev_err(&led->spmi_dev->dev,
				"MPP initialize failed(%d)\n", rc);
		break;
	default:
		dev_err(&led->spmi_dev->dev, "Invalid LED(%d)\n", led->id);
		return -EINVAL;
	}

	return rc;
}

static int qpnp_get_common_configs(struct qpnp_led_data *led,
				struct device_node *node)
{
	int rc;
	u32 val;
	const char *temp_string;

	led->cdev.default_trigger = LED_TRIGGER_DEFAULT;
	rc = of_property_read_string(node, "linux,default-trigger",
		&temp_string);
	if (!rc)
		led->cdev.default_trigger = temp_string;
	else if (rc != -EINVAL)
		return rc;

	led->default_on = false;
	rc = of_property_read_string(node, "qcom,default-state",
		&temp_string);
	if (!rc) {
		if (strncmp(temp_string, "on", sizeof("on")) == 0)
			led->default_on = true;
	} else if (rc != -EINVAL)
		return rc;

	led->turn_off_delay_ms = 0;
	rc = of_property_read_u32(node, "qcom,turn-off-delay-ms", &val);
	if (!rc)
		led->turn_off_delay_ms = val;
	else if (rc != -EINVAL)
		return rc;

	return 0;
}

static int qpnp_get_config_pwm(struct pwm_config_data *pwm_cfg,
				struct spmi_device *spmi_dev,
				struct device_node *node)
{
	struct property *prop;
	int rc, i, lut_max_size;
	u32 val;
	u8 *temp_cfg;

	pwm_cfg->pwm_dev = of_pwm_get(node, NULL);

	if (IS_ERR(pwm_cfg->pwm_dev)) {
		rc = PTR_ERR(pwm_cfg->pwm_dev);
		dev_err(&spmi_dev->dev, "Cannot get PWM device rc:(%d)\n", rc);
		pwm_cfg->pwm_dev = NULL;
		return rc;
	}

	pwm_cfg->pwm_coefficient = 255;
	rc = of_property_read_u32(node, "qcom,pwm_coefficient", &val);
	if (!rc)
		pwm_cfg->pwm_coefficient = val;
	else if (rc != -EINVAL)
		return rc;

	if (pwm_cfg->mode != MANUAL_MODE) {
		rc = of_property_read_u32(node, "qcom,pwm-us", &val);
		if (!rc)
			pwm_cfg->pwm_period_us = val;
		else
			return rc;
	}

	pwm_cfg->use_blink =
		of_property_read_bool(node, "qcom,use-blink");

	if (pwm_cfg->mode == LPG_MODE || pwm_cfg->use_blink) {
		pwm_cfg->duty_cycles =
			devm_kzalloc(&spmi_dev->dev,
			sizeof(struct pwm_duty_cycles), GFP_KERNEL);
		if (!pwm_cfg->duty_cycles) {
			dev_err(&spmi_dev->dev,
				"Unable to allocate memory\n");
			rc = -ENOMEM;
			goto bad_lpg_params;
		}

		prop = of_find_property(node, "qcom,duty-pcts",
			&pwm_cfg->duty_cycles->num_duty_pcts);
		if (!prop) {
			dev_err(&spmi_dev->dev, "Looking up property " \
				"node qcom,duty-pcts failed\n");
			rc =  -ENODEV;
			goto bad_lpg_params;
		} else if (!pwm_cfg->duty_cycles->num_duty_pcts) {
			dev_err(&spmi_dev->dev, "Invalid length of " \
				"duty pcts\n");
			rc =  -EINVAL;
			goto bad_lpg_params;
		}

		lut_max_size = PWM_LUT_MAX_SIZE;

		pwm_cfg->duty_cycles->duty_pcts =
			devm_kzalloc(&spmi_dev->dev,
			sizeof(int) * lut_max_size,
			GFP_KERNEL);
		if (!pwm_cfg->duty_cycles->duty_pcts) {
			dev_err(&spmi_dev->dev,
				"Unable to allocate memory\n");
			rc = -ENOMEM;
			goto bad_lpg_params;
		}

		pwm_cfg->old_duty_pcts =
			devm_kzalloc(&spmi_dev->dev,
			sizeof(int) * lut_max_size,
			GFP_KERNEL);
		if (!pwm_cfg->old_duty_pcts) {
			dev_err(&spmi_dev->dev,
				"Unable to allocate memory\n");
			rc = -ENOMEM;
			goto bad_lpg_params;
		}

		temp_cfg = devm_kzalloc(&spmi_dev->dev,
				pwm_cfg->duty_cycles->num_duty_pcts *
				sizeof(u8), GFP_KERNEL);
		if (!temp_cfg) {
			dev_err(&spmi_dev->dev, "Failed to allocate " \
				"memory for duty pcts\n");
			rc = -ENOMEM;
			goto bad_lpg_params;
		}

		memcpy(temp_cfg, prop->value,
			pwm_cfg->duty_cycles->num_duty_pcts);

		for (i = 0; i < pwm_cfg->duty_cycles->num_duty_pcts; i++)
			pwm_cfg->duty_cycles->duty_pcts[i] =
				(int) temp_cfg[i];

		rc = of_property_read_u32(node, "qcom,start-idx", &val);
		if (!rc) {
			pwm_cfg->lut_params.start_idx = val;
			pwm_cfg->duty_cycles->start_idx = val;
		} else
			goto bad_lpg_params;

		pwm_cfg->lut_params.lut_pause_hi = 0;
		rc = of_property_read_u32(node, "qcom,pause-hi", &val);
		if (!rc)
			pwm_cfg->lut_params.lut_pause_hi = val;
		else if (rc != -EINVAL)
			goto bad_lpg_params;

		pwm_cfg->lut_params.lut_pause_lo = 0;
		rc = of_property_read_u32(node, "qcom,pause-lo", &val);
		if (!rc)
			pwm_cfg->lut_params.lut_pause_lo = val;
		else if (rc != -EINVAL)
			goto bad_lpg_params;

		pwm_cfg->lut_params.ramp_step_ms =
				QPNP_LUT_RAMP_STEP_DEFAULT;
		rc = of_property_read_u32(node, "qcom,ramp-step-ms", &val);
		if (!rc)
			pwm_cfg->lut_params.ramp_step_ms = val;
		else if (rc != -EINVAL)
			goto bad_lpg_params;

		pwm_cfg->lut_params.flags = QPNP_LED_PWM_FLAGS;
		rc = of_property_read_u32(node, "qcom,lut-flags", &val);
		if (!rc)
			pwm_cfg->lut_params.flags = (u8) val;
		else if (rc != -EINVAL)
			goto bad_lpg_params;

		pwm_cfg->lut_params.idx_len =
			pwm_cfg->duty_cycles->num_duty_pcts;

	}
	return 0;

bad_lpg_params:
	pwm_cfg->use_blink = false;
	if (pwm_cfg->mode == PWM_MODE) {
		dev_err(&spmi_dev->dev, "LPG parameters not set for" \
			" blink mode, defaulting to PWM mode\n");
		return 0;
	}
	return rc;
};

static int qpnp_led_get_mode(const char *mode)
{
	if (strncmp(mode, "manual", strlen(mode)) == 0)
		return MANUAL_MODE;
	else if (strncmp(mode, "pwm", strlen(mode)) == 0)
		return PWM_MODE;
	else if (strncmp(mode, "lpg", strlen(mode)) == 0)
		return LPG_MODE;
	else
		return -EINVAL;
};

static int qpnp_get_config_rgb(struct qpnp_led_data *led,
				struct device_node *node)
{
	int rc;
	u8 led_mode;
	const char *mode;

	led->rgb_cfg = devm_kzalloc(&led->spmi_dev->dev,
				sizeof(struct rgb_config_data), GFP_KERNEL);
	if (!led->rgb_cfg) {
		dev_err(&led->spmi_dev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (led->id == QPNP_ID_RGB_RED)
		led->rgb_cfg->enable = RGB_LED_ENABLE_RED;
	else if (led->id == QPNP_ID_RGB_GREEN)
		led->rgb_cfg->enable = RGB_LED_ENABLE_GREEN;
	else if (led->id == QPNP_ID_RGB_BLUE)
		led->rgb_cfg->enable = RGB_LED_ENABLE_BLUE;
	else
		return -EINVAL;

	rc = of_property_read_string(node, "qcom,mode", &mode);
	if (!rc) {
		led_mode = qpnp_led_get_mode(mode);
		if ((led_mode == MANUAL_MODE) || (led_mode == -EINVAL)) {
			dev_err(&led->spmi_dev->dev, "Selected mode not " \
				"supported for rgb.\n");
			return -EINVAL;
		}
		led->rgb_cfg->pwm_cfg = devm_kzalloc(&led->spmi_dev->dev,
					sizeof(struct pwm_config_data),
					GFP_KERNEL);
		if (!led->rgb_cfg->pwm_cfg) {
			dev_err(&led->spmi_dev->dev,
				"Unable to allocate memory\n");
			return -ENOMEM;
		}
		led->rgb_cfg->pwm_cfg->mode = led_mode;
		led->rgb_cfg->pwm_cfg->default_mode = led_mode;
	} else
		return rc;

	rc = qpnp_get_config_pwm(led->rgb_cfg->pwm_cfg, led->spmi_dev, node);
	if (rc < 0)
		return rc;
	
	wake_lock_init(&pmic_led_rgb_wake_lock[led->id], WAKE_LOCK_SUSPEND, "qpnp_led");
	if (led->rgb_cfg->pwm_cfg->mode == LPG_MODE) {
		INIT_DELAYED_WORK(&led->fade_delayed_work, led_fade_do_work);
		led->rgb_cfg->pwm_cfg->lut_params.flags = QPNP_LED_PWM_FLAGS;
		led->rgb_cfg->pwm_cfg->duty_cycles->start_idx = 0;
		led->rgb_cfg->pwm_cfg->lut_params.start_idx = 0;
		led->rgb_cfg->pwm_cfg->lut_params.idx_len = 31;
		led->rgb_cfg->pwm_cfg->lut_params.ramp_step_ms = 64;
		led->rgb_cfg->pwm_cfg->lut_params.lut_pause_hi = 0;
		led->rgb_cfg->pwm_cfg->lut_params.lut_pause_lo = 0;
	}
	

	return 0;
}

static int qpnp_get_config_mpp(struct qpnp_led_data *led,
		struct device_node *node)
{
	int rc;
	u32 val;
	u8 led_mode;
	const char *mode;

	led->mpp_cfg = devm_kzalloc(&led->spmi_dev->dev,
			sizeof(struct mpp_config_data), GFP_KERNEL);
	if (!led->mpp_cfg) {
		dev_err(&led->spmi_dev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (of_find_property(of_get_parent(node), "mpp-power-supply", NULL)) {
		led->mpp_cfg->mpp_reg =
				regulator_get(&led->spmi_dev->dev,
							"mpp-power");
		if (IS_ERR(led->mpp_cfg->mpp_reg)) {
			rc = PTR_ERR(led->mpp_cfg->mpp_reg);
			dev_err(&led->spmi_dev->dev,
				"MPP regulator get failed(%d)\n", rc);
			return rc;
		}
	}

	if (led->mpp_cfg->mpp_reg) {
		rc = of_property_read_u32(of_get_parent(node),
					"qcom,mpp-power-max-voltage", &val);
		if (!rc)
			led->mpp_cfg->max_uV = val;
		else
			goto err_config_mpp;

		rc = of_property_read_u32(of_get_parent(node),
					"qcom,mpp-power-min-voltage", &val);
		if (!rc)
			led->mpp_cfg->min_uV = val;
		else
			goto err_config_mpp;

	} else {
		rc = of_property_read_u32(of_get_parent(node),
					"qcom,mpp-power-max-voltage", &val);
		if (!rc)
			dev_warn(&led->spmi_dev->dev,
						"No regulator specified\n");

		rc = of_property_read_u32(of_get_parent(node),
					"qcom,mpp-power-min-voltage", &val);
		if (!rc)
			dev_warn(&led->spmi_dev->dev,
						"No regulator specified\n");
	}

	led->mpp_cfg->current_setting = LED_MPP_CURRENT_MIN;
	rc = of_property_read_u32(node, "qcom,current-setting", &val);
	if (!rc) {
		if (led->mpp_cfg->current_setting < LED_MPP_CURRENT_MIN)
			led->mpp_cfg->current_setting = LED_MPP_CURRENT_MIN;
		else if (led->mpp_cfg->current_setting > LED_MPP_CURRENT_MAX)
			led->mpp_cfg->current_setting = LED_MPP_CURRENT_MAX;
		else
			led->mpp_cfg->current_setting = (u8) val;
	} else if (rc != -EINVAL)
		goto err_config_mpp;

	led->mpp_cfg->source_sel = LED_MPP_SOURCE_SEL_DEFAULT;
	rc = of_property_read_u32(node, "qcom,source-sel", &val);
	if (!rc)
		led->mpp_cfg->source_sel = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_mpp;

	led->mpp_cfg->mode_ctrl = LED_MPP_MODE_SINK;
	rc = of_property_read_u32(node, "qcom,mode-ctrl", &val);
	if (!rc)
		led->mpp_cfg->mode_ctrl = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_mpp;

	led->mpp_cfg->vin_ctrl = LED_MPP_VIN_CTRL_DEFAULT;
	rc = of_property_read_u32(node, "qcom,vin-ctrl", &val);
	if (!rc)
		led->mpp_cfg->vin_ctrl = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_mpp;

	led->mpp_cfg->min_brightness = 0;
	rc = of_property_read_u32(node, "qcom,min-brightness", &val);
	if (!rc)
		led->mpp_cfg->min_brightness = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_mpp;

	led->last_brightness = 0;

	led->base_pwm = 0;
	rc = of_property_read_u32(node, "vk,base-pwm", &val);
	if (!rc)
		led->base_pwm = val;

	rc = of_property_read_string(node, "qcom,mode", &mode);
	if (!rc) {
		led_mode = qpnp_led_get_mode(mode);
		led->mpp_cfg->pwm_mode = led_mode;
		if (led_mode == MANUAL_MODE)
			return MANUAL_MODE;
		else if (led_mode == -EINVAL) {
			dev_err(&led->spmi_dev->dev, "Selected mode not " \
				"supported for mpp.\n");
			rc = -EINVAL;
			goto err_config_mpp;
		}
		led->mpp_cfg->pwm_cfg = devm_kzalloc(&led->spmi_dev->dev,
					sizeof(struct pwm_config_data),
					GFP_KERNEL);
		if (!led->mpp_cfg->pwm_cfg) {
			dev_err(&led->spmi_dev->dev,
				"Unable to allocate memory\n");
			rc = -ENOMEM;
			goto err_config_mpp;
		}
		led->mpp_cfg->pwm_cfg->mode = led_mode;
		led->mpp_cfg->pwm_cfg->default_mode = led_mode;
	} else
		return rc;

	rc = qpnp_get_config_pwm(led->mpp_cfg->pwm_cfg, led->spmi_dev, node);
	if (rc < 0)
		goto err_config_mpp;

	if (led->mpp_cfg->pwm_mode == LPG_MODE)
		INIT_DELAYED_WORK(&led->fade_delayed_work, led_fade_do_work);

	return 0;

err_config_mpp:
	if (led->mpp_cfg->mpp_reg)
		regulator_put(led->mpp_cfg->mpp_reg);
	return rc;
}

#define CG_ID_LEN 5
#define BLACK_ID 1
#define WHITE_ID 2

static void get_brightness_mapping_table(struct device_node *node)
{
	struct property *prop;
	int current_table_level_num;
	int color_ID = BLACK_ID;
	int touch_solution = g_led_touch_solution;
	const char* cmdline;
	char* temp_cmdline;

	prop = of_find_property(node, "vk-pwm-array",
			&table_level_num);
	if(!prop) {
		LED_INFO("Not use color mapping table\n");
		return;
	}
    LED_INFO("%s, vk-pwm-array table_level_num: %d\n", __func__, table_level_num);
	use_color_table = 1;
	memcpy(color_table, prop->value, table_level_num);

	cmdline = kstrdup(saved_command_line, GFP_KERNEL);
	if (cmdline) {
		LED_INFO("Get cmdline success\n");
		temp_cmdline = strstr(cmdline, "color_ID=");
		if(temp_cmdline == NULL) {
			LED_INFO("No color_ID at devices\n");
			kfree(cmdline);
		} else {
			temp_cmdline += strlen("color_ID=");
			temp_cmdline[CG_ID_LEN] = '\0';
			if(of_property_match_string(node, "vk-black-cg-id-def", temp_cmdline) >= 0) {
				color_ID = BLACK_ID;
				LED_INFO("color_ID match %s, use color_ID: %d, touch_solution = %d\n", temp_cmdline, color_ID, touch_solution);
			} else if(of_property_match_string(node, "vk-white-cg-id-def", temp_cmdline) >= 0) {
				color_ID = WHITE_ID;
				LED_INFO("color_ID match %s, use color_ID: %d, touch_solution = %d\n", temp_cmdline, color_ID, touch_solution);
			} else {
				LED_INFO("No color_ID matched\n");
			}
			kfree(cmdline);
		}
	} else {
		LED_INFO("Get cmdline failed\n");
	}

	if(color_ID == BLACK_ID) {
		if(touch_solution == SEC_TOUCH_SOLUTION) {
			prop = of_find_property(node, "vk-black-pwm-array-sec",
				&table_level_num);
		} else {
			prop = of_find_property(node, "vk-black-pwm-array-def",
				&table_level_num);
		}
		if(!prop) {
			LED_INFO("Not use color_table\n");
		} else {
			memcpy(color_table, prop->value, table_level_num);
		}
	} else if(color_ID == WHITE_ID) {
		if(touch_solution == SEC_TOUCH_SOLUTION) {
			prop = of_find_property(node, "vk-white-pwm-array-sec",
				&table_level_num);
		} else {
			prop = of_find_property(node, "vk-white-pwm-array-def",
				&table_level_num);
		}
		if(!prop) {
			LED_INFO("Not use color_table\n");
		} else {
			memcpy(color_table, prop->value, table_level_num);
		}
	}

	if(touch_solution == SEC_TOUCH_SOLUTION) {
		prop = of_find_property(node, "vk-current-array-sec",
				&current_table_level_num);
	} else {
		prop = of_find_property(node, "vk-current-array-def",
				&current_table_level_num);
	}
	if(!prop) {
		use_current_table = 0;
	} else {
		use_current_table = 1;
		memcpy(current_table, prop->value, current_table_level_num);
	}
}

#ifdef CONFIG_LEDS_SYNC_TOUCH_SOLUTION
void set_led_touch_solution(uint16_t solution)
{
	struct device_node *temp;

	g_led_touch_solution = solution;

	if (g_led_virtual == NULL) {
		LED_INFO("%s, virtual key led probe not ready\n", __func__);
		return;
	}

	LED_INFO("%s, led_touch_solution = %d\n", __func__, g_led_touch_solution);

	for_each_child_of_node(g_led_virtual->spmi_dev->dev.of_node, temp) {
		get_brightness_mapping_table(temp);
	}
}
EXPORT_SYMBOL(set_led_touch_solution);
#endif

static int led_multicolor_short_blink(struct qpnp_led_data *led, int pwm_coefficient){
	int rc = 0;
	struct lut_params	lut_params;
	int *lut_short_blink;
	LED_INFO("%s, name:%s, brightness = %d status: %d\n", __func__, led->cdev.name, led->cdev.brightness, led->status);

	lut_params.flags = QPNP_LED_PWM_FLAGS | PM_PWM_LUT_PAUSE_HI_EN;
	lut_params.idx_len = SHORT_LUT_LEN;
	lut_params.ramp_step_ms = 64;
	lut_params.lut_pause_hi = 1792; 
	lut_params.lut_pause_lo = 0;

	switch(led->id){
		case QPNP_ID_RGB_RED:
			lut_short_blink = amber_short_lut;
			lut_params.start_idx = AMBER_SHORT_LUT_START;
			break;
		case QPNP_ID_RGB_GREEN:
			lut_short_blink = green_short_lut;
			lut_params.start_idx = GREEN_SHORT_LUT_START;
			break;
		case QPNP_ID_RGB_BLUE:
			lut_short_blink = blue_short_lut;
			lut_params.start_idx = BLUE_SHORT_LUT_START;
			break;
	}
	lut_short_blink[0] = pwm_coefficient;
	led->rgb_cfg->pwm_cfg->lut_params = lut_params;

	rc = pwm_lut_config(led->rgb_cfg->pwm_cfg->pwm_dev,
						PM_PWM_PERIOD_MIN,
						lut_short_blink,
						led->rgb_cfg->pwm_cfg->lut_params);
	rc = qpnp_led_masked_write(led,	RGB_LED_EN_CTL(led->base),
		led->rgb_cfg->enable, led->rgb_cfg->enable);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Failed to write led enable reg\n");
		return rc;
	}
	rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
	
	mdelay(10);
	rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
	led->status = ON;
	led->rgb_cfg->pwm_cfg->blinking = true;
	qpnp_dump_regs(led, rgb_pwm_debug_regs, ARRAY_SIZE(rgb_pwm_debug_regs));
	return rc;
}

static int led_multicolor_long_blink(struct qpnp_led_data *led, int pwm_coefficient){
	int rc = 0;
	struct lut_params	lut_params;
	int *lut_long_blink;
	LED_INFO("%s, name:%s, brightness = %d status: %d\n", __func__, led->cdev.name, led->cdev.brightness, led->status);

	lut_params.flags = QPNP_LED_PWM_FLAGS | PM_PWM_LUT_PAUSE_HI_EN | PM_PWM_LUT_PAUSE_LO_EN;
	lut_params.idx_len = LONG_LUT_LEN;
	lut_params.ramp_step_ms = 500;
	lut_params.lut_pause_hi = 0;
	lut_params.lut_pause_lo = 0;

	switch(led->id){
		case QPNP_ID_RGB_RED:
			lut_long_blink = amber_long_lut;
			lut_params.start_idx = AMBER_LONG_LUT_START;
			break;
		case QPNP_ID_RGB_GREEN:
			lut_long_blink = green_long_lut;
			lut_params.start_idx = GREEN_LONG_LUT_START;
			break;
		case QPNP_ID_RGB_BLUE:
			lut_long_blink = blue_long_lut;
			lut_params.start_idx = BLUE_LONG_LUT_START;
			break;
	}
	lut_long_blink[0] = pwm_coefficient;
	led->rgb_cfg->pwm_cfg->lut_params = lut_params;

	rc = pwm_lut_config(led->rgb_cfg->pwm_cfg->pwm_dev,
						PM_PWM_PERIOD_MIN,
						lut_long_blink,
						led->rgb_cfg->pwm_cfg->lut_params);
	rc = qpnp_led_masked_write(led,	RGB_LED_EN_CTL(led->base),
		led->rgb_cfg->enable, led->rgb_cfg->enable);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Failed to write led enable reg\n");
		return rc;
	}
	rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
	
	mdelay(10);
	rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
	led->status = ON;
	led->rgb_cfg->pwm_cfg->blinking = true;
	qpnp_dump_regs(led, rgb_pwm_debug_regs, ARRAY_SIZE(rgb_pwm_debug_regs));
	return rc;
}


static int lpg_blink(struct led_classdev *led_cdev, int val)
{
	struct qpnp_led_data *led;
	int rc;

	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	LED_DBG("%s: bank %d blink %d status %d\n", __func__, led->id, val, led->status);

	switch (val) {
	case BLINK_STOP:
		pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
		rc = qpnp_led_masked_write(led,
				RGB_LED_EN_CTL(led->base),
				led->rgb_cfg->enable, RGB_LED_DISABLE);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
					"Failed to write led enable reg\n");
			return rc;
		}
		led->status = OFF;
		break;
	case BLINK_UNCHANGE:
		if (led->cdev.brightness) {
			if (led->status == BLINK) {
				if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_PWM) {
					rc = pwm_config_us(led->rgb_cfg->pwm_cfg->pwm_dev, 640 * led->rgb_cfg->pwm_cfg->pwm_coefficient / 255, 640);
					if (rc < 0) {
					dev_err(&led->spmi_dev->dev, "Failed to " \
					"configure pwm for new values\n");
					return rc;
					}
				}
				rc = qpnp_led_masked_write(led,
				RGB_LED_EN_CTL(led->base),
				led->rgb_cfg->enable, led->rgb_cfg->enable);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
					"Failed to write led enable reg\n");
					return rc;
				}
				rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
				
				mdelay(10);
				rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
				led->status = ON;
			}
		} else {
			pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
			rc = qpnp_led_masked_write(led,
			RGB_LED_EN_CTL(led->base),
			led->rgb_cfg->enable, RGB_LED_DISABLE);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
				"Failed to write led enable reg\n");
				return rc;
			}
			led->status = OFF;
		}
		break;
	case BLINK_64MS_PER_2SEC:
		pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
		if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_PWM) {
			rc = pwm_config_us(led->rgb_cfg->pwm_cfg->pwm_dev, 64000, 2000000);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev, "Failed to " \
					"configure pwm for new values\n");
				return rc;
			}
		}
		rc = qpnp_led_masked_write(led,
			RGB_LED_EN_CTL(led->base),
			led->rgb_cfg->enable, led->rgb_cfg->enable);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"Failed to write led enable reg\n");
			return rc;
		}
		rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
		
		mdelay(10);
		rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
		led->status = BLINK;
		break;
	case BLINK_64MS_ON_310MS_PER_2SEC:
		cancel_delayed_work_sync(&led->blink_delayed_work);
		pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
		led->rgb_cfg->pwm_cfg->pwm_duty_us = 64000;
		led->rgb_cfg->pwm_cfg->pwm_period_us = 2000000;
		queue_delayed_work(g_led_work_queue, &led->blink_delayed_work,
					msecs_to_jiffies(310));
		break;
	case BLINK_64MS_ON_2SEC_PER_2SEC:
		cancel_delayed_work_sync(&led->blink_delayed_work);
		pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
		led->rgb_cfg->pwm_cfg->pwm_duty_us = 64000;
		led->rgb_cfg->pwm_cfg->pwm_period_us = 2000000;
		queue_delayed_work(g_led_work_queue, &led->blink_delayed_work,
				   msecs_to_jiffies(1000));
		break;
	case BLINK_1SEC_PER_2SEC:
		pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
		if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_PWM) {
				rc = pwm_config_us(led->rgb_cfg->pwm_cfg->pwm_dev, 1000000, 2000000);
				if (rc < 0) {
						dev_err(&led->spmi_dev->dev, "Failed to " \
								"configure pwm for new values\n");
						return rc;
				}
		}
		rc = qpnp_led_masked_write(led,
				RGB_LED_EN_CTL(led->base),
				led->rgb_cfg->enable, led->rgb_cfg->enable);
		if (rc) {
				dev_err(&led->spmi_dev->dev,
						"Failed to write led enable reg\n");
				return rc;
		}
		rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
		
		mdelay(10);
		rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
		led->status = BLINK;
		break;
	default:
		LED_ERR("%s: bank %d did not support blink type %d\n", __func__, led->id, val);
		return -EINVAL;
	}
	return 0;
}

static void led_multicolor_work_func(struct work_struct *work){

	struct qpnp_led_data *led;
	led = container_of(work, struct qpnp_led_data, led_multicolor_work);

	LED_INFO(" %s , Mode = %x\n" , __func__, led->mode);

	if(g_led_red){
		g_led_red->cdev.brightness = 0;
		qpnp_rgb_set(g_led_red);
	}
	if(g_led_green){
		g_led_green->cdev.brightness = 0;
		qpnp_rgb_set(g_led_green);
	}
	if(g_led_blue){
		g_led_blue->cdev.brightness = 0;
		qpnp_rgb_set(g_led_blue);
	}
	if (led->mode == 1){
		if(g_led_red){
			if(g_led_red->rgb_cfg->pwm_cfg->pwm_coefficient)
				g_led_red->cdev.brightness = 1;
			qpnp_rgb_set(g_led_red);
		}
		if(g_led_green){
			if(g_led_green->rgb_cfg->pwm_cfg->pwm_coefficient)
				g_led_green->cdev.brightness = 1;			
			qpnp_rgb_set(g_led_green);
		}
		if(g_led_blue){
			if(g_led_blue->rgb_cfg->pwm_cfg->pwm_coefficient)
				g_led_blue->cdev.brightness = 1;
			qpnp_rgb_set(g_led_blue);
		}
	}
	else if(led->mode == 2){
		if(g_led_red)
			if(g_led_red->rgb_cfg->pwm_cfg->pwm_coefficient)
				led_multicolor_short_blink(g_led_red, g_led_red->rgb_cfg->pwm_cfg->pwm_coefficient * 100 / 255);
		if(g_led_green)
			if(g_led_green->rgb_cfg->pwm_cfg->pwm_coefficient)
				led_multicolor_short_blink(g_led_green, g_led_green->rgb_cfg->pwm_cfg->pwm_coefficient * 100 / 255);
		if(g_led_blue)
			if(g_led_blue->rgb_cfg->pwm_cfg->pwm_coefficient)
				led_multicolor_short_blink(g_led_blue, g_led_blue->rgb_cfg->pwm_cfg->pwm_coefficient * 100 / 255);
	}
	else if(led->mode == 4){
		if(g_led_red)
			if(g_led_red->rgb_cfg->pwm_cfg->pwm_coefficient)
				led_multicolor_long_blink(g_led_red, g_led_red->rgb_cfg->pwm_cfg->pwm_coefficient * 100 / 255);
		if(g_led_green)
			if(g_led_green->rgb_cfg->pwm_cfg->pwm_coefficient)
				led_multicolor_long_blink(g_led_green, g_led_green->rgb_cfg->pwm_cfg->pwm_coefficient * 100 / 255);
		if(g_led_blue)
			if(g_led_blue->rgb_cfg->pwm_cfg->pwm_coefficient)
				led_multicolor_long_blink(g_led_blue, g_led_blue->rgb_cfg->pwm_cfg->pwm_coefficient * 100 / 255);
	}
}

static void led_blink_work_func(struct work_struct *work)
{
	struct qpnp_led_data *ldata;
	int rc;
	LED_DBG("%s +++\n", __func__);
	ldata = container_of(work, struct qpnp_led_data, led_blink_work);
	if (ldata->id == QPNP_ID_LED_MPP) {
		rc = qpnp_mpp_set(ldata);
		if (rc < 0)
			dev_err(&ldata->spmi_dev->dev, "MPP set brightness failed (%d)\n", rc);
	} else {
		lpg_blink(&ldata->cdev, ldata->mode);
	}

	LED_DBG("%s ---\n", __func__);
}

static void led_off_work_func(struct work_struct *work)
{
	struct qpnp_led_data *ldata;

	ldata = container_of(work, struct qpnp_led_data, led_off_work);
	LED_INFO("%s: bank %d\n", __func__, ldata->id);
	qpnp_led_turn_off(ldata);
}

void virtual_key_led_reset_blink(int onoff)
{
	int virtual_key_lut_table[VIRTUAL_LUT_LEN] = {0};
	int rc;
	u8 val;

	if(!g_led_virtual)
		return;

	LED_INFO("virtual_key_led_reset_blink +++, onoff = %d\n", onoff);
	if (onoff) {
		virtual_key_led_ignore_flag = 1;
		if (g_led_virtual->mpp_cfg->mpp_reg && !g_led_virtual->mpp_cfg->enable) {
			rc = regulator_set_voltage(g_led_virtual->mpp_cfg->mpp_reg,
					g_led_virtual->mpp_cfg->min_uV,
					g_led_virtual->mpp_cfg->max_uV);

			rc = regulator_enable(g_led_virtual->mpp_cfg->mpp_reg);
		}

		g_led_virtual->mpp_cfg->enable = true;

		rc = pwm_config_us(
				g_led_virtual->mpp_cfg->pwm_cfg->pwm_dev,
				300000,
				600000);

		pwm_enable(g_led_virtual->mpp_cfg->pwm_cfg->pwm_dev);

		val = (g_led_virtual->mpp_cfg->source_sel & LED_MPP_SRC_MASK) |
			(g_led_virtual->mpp_cfg->mode_ctrl & LED_MPP_MODE_CTRL_MASK);

		rc = qpnp_led_masked_write(g_led_virtual,
			LED_MPP_MODE_CTRL(g_led_virtual->base), LED_MPP_MODE_MASK,
			val);

		rc = qpnp_led_masked_write(g_led_virtual,
				LED_MPP_EN_CTRL(g_led_virtual->base), LED_MPP_EN_MASK,
				LED_MPP_EN_ENABLE);
	} else {
		virtual_key_led_ignore_flag = 0;
		if(g_led_virtual->cdev.brightness) {
			g_led_virtual->mpp_cfg->pwm_cfg->lut_params.flags = PM_PWM_LUT_RAMP_UP;
			g_led_virtual->mpp_cfg->pwm_cfg->lut_params.start_idx = VIRTUAL_LUT_START;
			g_led_virtual->mpp_cfg->pwm_cfg->lut_params.idx_len = VIRTUAL_LUT_LEN;
			g_led_virtual->mpp_cfg->pwm_cfg->lut_params.ramp_step_ms = VIRTUAL_RAMP_SETP_TIME;
			g_led_virtual->mpp_cfg->pwm_cfg->lut_params.lut_pause_hi = 0;
			g_led_virtual->mpp_cfg->pwm_cfg->lut_params.lut_pause_lo = 0;
			virtual_key_lut_table_set(virtual_key_lut_table, VIRTUAL_LUT_LEN, g_led_virtual->base_pwm, g_led_virtual->cdev.brightness, 0);
			g_led_virtual->last_brightness = g_led_virtual->cdev.brightness;
			rc = pwm_lut_config(g_led_virtual->mpp_cfg->pwm_cfg->pwm_dev,
						PM_PWM_PERIOD_MIN,
						virtual_key_lut_table,
						g_led_virtual->mpp_cfg->pwm_cfg->lut_params);
		}
		else {
			if (g_led_virtual->mpp_cfg->pwm_mode != MANUAL_MODE) {
				g_led_virtual->mpp_cfg->pwm_cfg->mode =
					g_led_virtual->mpp_cfg->pwm_cfg->default_mode;
				g_led_virtual->mpp_cfg->pwm_mode =
					g_led_virtual->mpp_cfg->pwm_cfg->default_mode;
				pwm_disable(g_led_virtual->mpp_cfg->pwm_cfg->pwm_dev);
			}
			rc = qpnp_led_masked_write(g_led_virtual,
						LED_MPP_MODE_CTRL(g_led_virtual->base),
						LED_MPP_MODE_MASK,
						LED_MPP_MODE_DISABLE);

			rc = qpnp_led_masked_write(g_led_virtual,
						LED_MPP_EN_CTRL(g_led_virtual->base),
						LED_MPP_EN_MASK,
						LED_MPP_EN_DISABLE);

			if (g_led_virtual->mpp_cfg->mpp_reg && g_led_virtual->mpp_cfg->enable) {
				rc = regulator_disable(g_led_virtual->mpp_cfg->mpp_reg);

				rc = regulator_set_voltage(g_led_virtual->mpp_cfg->mpp_reg,
							0, g_led_virtual->mpp_cfg->max_uV);
			}
			g_led_virtual->mpp_cfg->enable = false;
		}
	}

	return;
}

EXPORT_SYMBOL(virtual_key_led_reset_blink);


static ssize_t led_off_timer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	struct led_classdev *led_cdev;
	int min, sec;
	uint16_t off_timer;

	min = -1;
	sec = -1;
	sscanf(buf, "%d %d", &min, &sec);

	if (min < 0 || min > 255)
		return -EINVAL;
	if (sec < 0 || sec > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	LED_DBG("Setting %s off_timer to %d min %d sec \n", led_cdev->name, min, sec);
	off_timer = min * 60 + sec;

	return count;
}
static DEVICE_ATTR(off_timer, 0644, NULL, led_off_timer_store);


static ssize_t pm8xxx_led_blink_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
	return sprintf(buf, "%d\n", current_blink);
}

static ssize_t pm8xxx_led_blink_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	struct led_classdev *led_cdev;
	int val;
	bool ret;
	int retry_count = 0;

	val = -1;
	sscanf(buf, "%u", &val);
	if (val < -1 || val > 255)
			return -EINVAL;
	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	led = container_of(led_cdev, struct qpnp_led_data, cdev);
	led->mode = val;
	current_blink = val;
	LED_INFO("%s: blink: %d\n", __func__, val);
	switch(led->id) {
		case QPNP_ID_LED_MPP:
			led->mpp_cfg->blink_mode = val;
			if (led->cdev.brightness) {
				led->mpp_cfg->pwm_cfg->blinking = true;
				if (led->id == QPNP_ID_LED_MPP)
					led->mpp_cfg->pwm_mode = LPG_MODE;
				led->mpp_cfg->pwm_cfg->mode = LPG_MODE;
			} else {
				led->mpp_cfg->pwm_cfg->blinking = false;
				led->mpp_cfg->pwm_cfg->mode = led->mpp_cfg->pwm_cfg->default_mode;
				if (led->id == QPNP_ID_LED_MPP)
					led->mpp_cfg->pwm_mode = led->mpp_cfg->pwm_cfg->default_mode;
			}
		case QPNP_ID_RGB_RED:
		case QPNP_ID_RGB_GREEN:
		case QPNP_ID_RGB_BLUE:
			do {
				ret = queue_work(g_led_on_work_queue, &led->led_blink_work);
				if (!ret) {
					retry_count++;
					msleep(1);
					LED_INFO("%s led_blink_work already on queue, requeue!! ret:%d retry:%d\n", __func__, ret, retry_count);
				}
			} while ((!ret) && (retry_count < 3));
			break;
		case QPNP_ID_KPDBL:
			break;
		default:
			return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(blink, 0644, pm8xxx_led_blink_show, pm8xxx_led_blink_store);

static ssize_t led_multi_color_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", ModeRGB);
}

static ssize_t led_multi_color_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct qpnp_led_data *led;
	uint32_t val;
	sscanf(buf, "%x", &val);
	LED_INFO(" %s , ModeRGB = %x\n" , __func__, val);
	if (val < 0 || val > 0xFFFFFFFF)
		return -EINVAL;
	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	led = container_of(led_cdev, struct qpnp_led_data, cdev);
	led->mode = (val & Mode_Mask) >> 24;
	if(g_led_red)
		g_led_red->rgb_cfg->pwm_cfg->pwm_coefficient= ((val & Red_Mask) >> 16) * indicator_pwm_ratio / 255;
	if(g_led_green)
		g_led_green->rgb_cfg->pwm_cfg->pwm_coefficient = ((val & Green_Mask) >> 8) * indicator_pwm_ratio / 255;
	if(g_led_blue)
		g_led_blue->rgb_cfg->pwm_cfg->pwm_coefficient = (val & Blue_Mask) * indicator_pwm_ratio / 255;
	ModeRGB = val;
	LED_INFO(" %s , ModeRGB = %x\n" , __func__, val);
	queue_work(g_led_on_work_queue, &led->led_multicolor_work);
	return count;
}

static DEVICE_ATTR(ModeRGB, 0644, led_multi_color_show,
		led_multi_color_store);

static ssize_t led_mpp_current_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct qpnp_led_data *led;
	int val, rc;
	u8 read_data;

	sscanf(buf, "%d", &val);
	LED_INFO(" %s , current set to %d\n" , __func__, val);

	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	if (val < LED_MPP_CURRENT_MIN)
		val = LED_MPP_CURRENT_MIN;
	else if (val > LED_MPP_CURRENT_MAX)
		val = LED_MPP_CURRENT_MAX;
	else {
		val /= LED_MPP_CURRENT_MIN;
		val *= LED_MPP_CURRENT_MIN;
	}

	val = (val / LED_MPP_CURRENT_MIN) - 1;

	rc = qpnp_led_masked_write(led,
			LED_MPP_SINK_CTRL(led->base),
			LED_MPP_SINK_MASK, val);
	if (rc) {
		LED_ERR("Failed to write sink control reg\n");
		return rc;
	}

	rc = spmi_ext_register_readl(led->spmi_dev->ctrl, led->spmi_dev->sid,
		LED_MPP_SINK_CTRL(led->base), &read_data, 1);
	LED_INFO("led_mpp_current_store read_data = 0x%02x\n", read_data);

	return count;
}

static DEVICE_ATTR(current_set, 0200, NULL, led_mpp_current_store);

static ssize_t led_color_ID_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct device_node *node;
	struct property *prop;
	char color_ID_name[6];
	int current_table_level_num;
	int color_ID = BLACK_ID;
	int touch_solution = g_led_touch_solution;

	memcpy(color_ID_name, buf, CG_ID_LEN);
	color_ID_name[CG_ID_LEN] = '\0';

    LED_INFO("%s, Update color mapping talbe of color_ID: %s\n", __func__, color_ID_name);

	for_each_child_of_node(g_led_virtual->spmi_dev->dev.of_node, node) {
		if(of_property_match_string(node, "vk-black-cg-id-def", color_ID_name) >= 0) {
			color_ID = BLACK_ID;
			LED_INFO("color_ID match %s, use color_ID: %d, touch_solution = %d\n", color_ID_name, color_ID, touch_solution);
		} else if(of_property_match_string(node, "vk-white-cg-id-def", color_ID_name) >= 0) {
			color_ID = WHITE_ID;
			LED_INFO("color_ID match %s, use color_ID: %d, touch_solution = %d\n", color_ID_name, color_ID, touch_solution);
		} else {
			LED_INFO("No color_ID matched\n");
			return count;
		}

		if(color_ID == BLACK_ID) {
			if(touch_solution == SEC_TOUCH_SOLUTION) {
				prop = of_find_property(node, "vk-black-pwm-array-sec",
					&table_level_num);
			} else {
				prop = of_find_property(node, "vk-black-pwm-array-def",
					&table_level_num);
			}
			if(!prop) {
				LED_INFO("Not use color_table\n");
			} else {
				memcpy(color_table, prop->value, table_level_num);
			}
		} else if(color_ID == WHITE_ID) {
			if(touch_solution == SEC_TOUCH_SOLUTION) {
				prop = of_find_property(node, "vk-white-pwm-array-sec",
					&table_level_num);
			} else {
				prop = of_find_property(node, "vk-white-pwm-array-def",
					&table_level_num);
			}
			if(!prop) {
				LED_INFO("Not use color_table\n");
			} else {
				memcpy(color_table, prop->value, table_level_num);
			}
		}

		if(touch_solution == SEC_TOUCH_SOLUTION) {
			prop = of_find_property(node, "vk-current-array-sec",
					&current_table_level_num);
		} else {
			prop = of_find_property(node, "vk-current-array-def",
					&current_table_level_num);
		}
		if(!prop) {
			use_current_table = 0;
		} else {
			use_current_table = 1;
			memcpy(current_table, prop->value, current_table_level_num);
		}
	}
	return count;
}

static DEVICE_ATTR(set_color_ID, 0200, NULL, led_color_ID_store);

static ssize_t led_pwm_coefficient_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct qpnp_led_data *led;
	struct led_classdev *led_cdev;

	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	led = container_of(led_cdev, struct qpnp_led_data, cdev);
	switch(led->id) {
		case QPNP_ID_LED_MPP:
			return sprintf(buf, "%d\n", led->mpp_cfg->pwm_cfg->pwm_coefficient);
				break;
			case QPNP_ID_RGB_RED:
			case QPNP_ID_RGB_GREEN:
			case QPNP_ID_RGB_BLUE:
				return sprintf(buf, "%d\n", led->rgb_cfg->pwm_cfg->pwm_coefficient);
				break;
				default:
			return -EINVAL;
	}
}

static ssize_t led_pwm_coefficient_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int pwm_coefficient1 = 0;
	struct qpnp_led_data *led;
	struct led_classdev *led_cdev;
	int val;

	val = -1;
	sscanf(buf, "%u", &val);
	if (val < -1 || val > 255)
			return -EINVAL;
	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	sscanf(buf, "%d", &pwm_coefficient1);
	if ((pwm_coefficient1 < 0) || (pwm_coefficient1 > 255)) {
		LED_INFO("%s: pwm_coefficient = %d, out of range.\n",
			__func__, pwm_coefficient1);
		return -EINVAL;
	}

	LED_INFO("%s: pwm_coefficient %d\n", __func__, pwm_coefficient1);
	switch(led->id) {
	case QPNP_ID_LED_MPP:
		led->mpp_cfg->pwm_cfg->pwm_coefficient = pwm_coefficient1;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		led->rgb_cfg->pwm_cfg->pwm_coefficient = pwm_coefficient1;
		break;
	default:
		return -EINVAL;
		}
	return count;
}
static DEVICE_ATTR(pwm_coefficient, 0644, led_pwm_coefficient_show, led_pwm_coefficient_store);
#ifdef CONFIG_LEDS_VIRTUAL_KEY_CHECK_SOURCE
int check_power_source(void)
{
	u8 pcb_id = 255;
	u32 pid = 999;

	struct device_node *mfgnode = of_find_node_by_path("/chosen/mfg");
	struct device_node *boardnode = of_find_node_by_path("/chosen/board_info");

	if (mfgnode) {
		if (of_property_read_u8(mfgnode, "skuid.pcb_id", &pcb_id))
			LED_ERR(" %s, Failed to get property: pbc_id\n", __func__);
	} else {
			LED_ERR(" %s, Failed to find device node\n", __func__);
	}

	if (boardnode) {
		if (of_property_read_u32(boardnode, "pid", &pid))
			LED_ERR(" %s, Failed to get property: pid\n", __func__);
	} else {
			LED_ERR(" %s, Failed to find device node\n", __func__);
	}
	LED_INFO(" %s, pid = %d, pcb_id = %d\n", __func__, pid, pcb_id);
	switch(pid) {
		case 402:	
		case 405:	
			if(pcb_id < 2)
				return 0xa000;
			else
				return 0xa300;
			break;
		case 403:	
		case 406:	
			if(pcb_id < 1)
				return 0xa000;
			else
				return 0xa300;
			break;
		default:
			return 0xa300;
	}
}
#endif

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;

    LED_DBG("%s\n", __func__);
    if (evdata && evdata->data && event == FB_EVENT_BLANK && g_led_virtual) {
        blank = evdata->data;
        switch (*blank) {
        case FB_BLANK_UNBLANK:
			
            break;

        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
            
            if(g_led_virtual->cdev.brightness) {
				g_led_virtual->cdev.brightness = 0;
                qpnp_mpp_set(g_led_virtual);
            }
            break;
        }
    }
    return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void qpnp_led_early_suspend(struct early_suspend *handler)
{
}

static void qpnp_led_late_resume(struct early_suspend *handler)
{
}
#endif
static int qpnp_leds_probe(struct spmi_device *spmi)
{
	struct qpnp_led_data *led, *led_array;
	struct resource *led_resource;
	struct device_node *node, *temp;
	int rc, i, num_leds = 0, parsed_leds = 0;
	const char *led_label;
	bool indicator_used = false;
	LED_INFO("led driver probe\n");
	node = spmi->dev.of_node;
	if (node == NULL)
		return -ENODEV;

	temp = NULL;
	while ((temp = of_get_next_child(node, temp)))
		num_leds++;

	if (!num_leds)
		return -ECHILD;

	led_array = devm_kzalloc(&spmi->dev,
		(sizeof(struct qpnp_led_data) * num_leds), GFP_KERNEL);
	if (!led_array) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	for_each_child_of_node(node, temp) {
		led = &led_array[parsed_leds];
		led->num_leds = num_leds;
		led->spmi_dev = spmi;

		led_resource = spmi_get_resource(spmi, NULL, IORESOURCE_MEM, 0);
		if (!led_resource) {
			dev_err(&spmi->dev, "Unable to get LED base address\n");
			rc = -ENXIO;
			goto fail_id_check;
		}
		led->base = led_resource->start;
		rc = of_property_read_string(temp, "label", &led_label);
		if (rc < 0) {
			dev_err(&led->spmi_dev->dev,
				"Failure reading label, rc = %d\n", rc);
			goto fail_id_check;
		}
		rc = of_property_read_string(temp, "linux,name",
			&led->cdev.name);
		if (rc < 0) {
			dev_err(&led->spmi_dev->dev,
				"Failure reading led name, rc = %d\n", rc);
			goto fail_id_check;
		}
#ifdef CONFIG_LEDS_VIRTUAL_KEY_CHECK_SOURCE
		if(strcmp(led->cdev.name, "button-backlight") == 0) {
			if(check_power_source() != led->base) {
				LED_INFO("button-backlight not use power source 0x%04x\n", led->base);
				goto fail_id_check;
			}
		}
#endif
		if(strcmp(led->cdev.name, "indicator") == 0){
			indicator_used = true;
			led->id = QPNP_ID_MAX;
			led->default_on = false;
			led->turn_off_delay_ms = 0;
			rc = of_property_read_u8(temp, "pwm_ratio", &indicator_pwm_ratio);
			if (rc < 0 || !indicator_pwm_ratio)
				indicator_pwm_ratio = 255;
		}else{
			rc = of_property_read_u32(temp, "qcom,max-current",
				&led->max_current);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev,
					"Failure reading max_current, rc =  %d\n", rc);
				goto fail_id_check;
			}

			rc = of_property_read_u32(temp, "qcom,id", &led->id);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev,
					"Failure reading led id, rc =  %d\n", rc);
				goto fail_id_check;
			}

			rc = qpnp_get_common_configs(led, temp);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Failure reading common led configuration," \
					" rc = %d\n", rc);
				goto fail_id_check;
			}

			led->cdev.brightness_set    = qpnp_led_set;
			led->cdev.brightness_get    = qpnp_led_get;

			if (strncmp(led_label, "rgb", sizeof("rgb")) == 0) {
				rc = qpnp_get_config_rgb(led, temp);
				if (rc < 0) {
					dev_err(&led->spmi_dev->dev,
						"Unable to read rgb config data\n");
					goto fail_id_check;
				}
			} else if (strncmp(led_label, "mpp", sizeof("mpp")) == 0) {
				rc = qpnp_get_config_mpp(led, temp);
				if (rc < 0) {
					dev_err(&led->spmi_dev->dev,
							"Unable to read mpp config data\n");
					goto fail_id_check;
				}
			} else {
				dev_err(&led->spmi_dev->dev, "No LED matching label\n");
				rc = -EINVAL;
				goto fail_id_check;
			}
		}
		if (led->id != QPNP_ID_FLASH1_LED0 &&
					led->id != QPNP_ID_FLASH1_LED1)
			mutex_init(&led->lock);
		INIT_WORK(&led->work, qpnp_led_work);
		INIT_WORK(&led->led_blink_work, led_blink_work_func);
		INIT_WORK(&led->led_multicolor_work, led_multicolor_work_func);
		g_led_work_queue = create_singlethread_workqueue("qpnp-led");
		if (g_led_work_queue == NULL) {
			LED_ERR("failed to create workqueue\n");
			goto err_create_work_queue;
		}
		g_gpled_work_queue = create_singlethread_workqueue("qpnp-gpled");
		if (g_gpled_work_queue == NULL) {
			LED_ERR("failed to create workqueue\n");
			goto err_create_work_queue;
		}
		g_led_on_work_queue = create_singlethread_workqueue("pm8xxx-led-on");
		if (g_led_on_work_queue == NULL) {
			LED_ERR("failed to create workqueue\n");
			goto err_create_work_queue;
		}

		led->in_order_command_processing = of_property_read_bool
				(temp, "qcom,in-order-command-processing");

		if (led->in_order_command_processing) {
			led->workqueue = alloc_ordered_workqueue
							("led_workqueue", 0);
			if (!led->workqueue) {
				rc = -ENOMEM;
				goto fail_id_check;
			}
		}

		INIT_WORK(&led->work, qpnp_led_work);
		if(strcmp(led->cdev.name, "indicator") != 0){
			rc =  qpnp_led_initialize(led);
			if (rc < 0)
				goto fail_id_check;

			rc = qpnp_led_set_max_brightness(led);
			if (rc < 0)
				goto fail_id_check;
		}
		rc = led_classdev_register(&spmi->dev, &led->cdev);
		if (rc) {
			dev_err(&spmi->dev, "unable to register led %d,rc=%d\n",
						 led->id, rc);
			goto fail_id_check;
		}
		if (led->id == QPNP_ID_FLASH1_LED0 ||
			led->id == QPNP_ID_FLASH1_LED1) {
			rc = sysfs_create_group(&led->cdev.dev->kobj,
							&led_attr_group);
			if (rc)
				goto fail_id_check;

		}
		if (led->id == QPNP_ID_LED_MPP) {
			if (!led->mpp_cfg->pwm_cfg)
				break;
			if (led->mpp_cfg->pwm_cfg->mode == PWM_MODE) {
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&pwm_attr_group);
				if (rc)
					goto fail_id_check;
			}
			if (led->mpp_cfg->pwm_cfg->use_blink) {
				
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&blink_attr_group);
				if (rc)
					goto fail_id_check;

				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&lpg_attr_group);
				if (rc)
					goto fail_id_check;
			} else if (led->mpp_cfg->pwm_cfg->mode == LPG_MODE) {
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&lpg_attr_group);
				if (rc)
					goto fail_id_check;
			}
		} else if ((led->id == QPNP_ID_RGB_RED) ||
			(led->id == QPNP_ID_RGB_GREEN) ||
			(led->id == QPNP_ID_RGB_BLUE)) {
			if (led->rgb_cfg->pwm_cfg->mode == PWM_MODE) {
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&pwm_attr_group);
				if (rc)
					goto fail_id_check;
			}
			if (led->rgb_cfg->pwm_cfg->use_blink) {
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&blink_attr_group);
				if (rc)
					goto fail_id_check;

				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&lpg_attr_group);
				if (rc)
					goto fail_id_check;
			} else if (led->rgb_cfg->pwm_cfg->mode == LPG_MODE) {
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&lpg_attr_group);
				if (rc)
					goto fail_id_check;
			}
		}

		if(led->id == QPNP_ID_MAX){
			rc = device_create_file(led->cdev.dev, &dev_attr_ModeRGB);
			if (rc < 0) {
				LED_ERR("%s: Failed to create %s attr ModeRGB\n", __func__,  led->cdev.name);
			}
		}else{
			
			if (led->default_on) {
				led->cdev.brightness = led->cdev.max_brightness;
				__qpnp_led_work(led, led->cdev.brightness);
				if (led->turn_off_delay_ms > 0)
					qpnp_led_turn_off(led);
			} else
				led->cdev.brightness = LED_OFF;
			if (strncmp(led_label, "rgb", sizeof("rgb")) == 0) {
				rc = device_create_file(led->cdev.dev, &dev_attr_blink);
				rc = device_create_file(led->cdev.dev, &dev_attr_pwm_coefficient);
				if (rc < 0) {
					LED_ERR("%s: Failed to create %s attr blink\n", __func__,  led->cdev.name);
				}
				if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_PWM) {
					rc = device_create_file(led->cdev.dev, &dev_attr_off_timer);
					if (rc < 0) {
						LED_ERR("%s: Failed to create %s attr off_timer\n", __func__,  led->cdev.name);
					}
					
					INIT_WORK(&led->led_off_work, led_off_work_func); 
				}
				INIT_DELAYED_WORK(&led->blink_delayed_work, led_blink_do_work);
			}
			if (strncmp(led_label, "mpp", sizeof("mpp")) == 0) {
				get_brightness_mapping_table(temp);
#ifdef CONFIG_FB
				led->fb_notifier.notifier_call = fb_notifier_callback;
				fb_register_client(&led->fb_notifier);
#endif
				rc = device_create_file(led->cdev.dev, &dev_attr_blink);
				rc = device_create_file(led->cdev.dev, &dev_attr_pwm_coefficient);
				rc = device_create_file(led->cdev.dev, &dev_attr_current_set);
				rc = device_create_file(led->cdev.dev, &dev_attr_set_color_ID);
				if (rc < 0) {
					LED_ERR("%s: Failed to create %s attr blink\n", __func__,  led->cdev.name);
				}
				if (led->mpp_cfg->pwm_cfg->mode == PWM_MODE) {
					rc = device_create_file(led->cdev.dev, &dev_attr_off_timer);
					if (rc < 0) {
						LED_ERR("%s: Failed to create %s attr off_timer\n", __func__,  led->cdev.name);
					}
					
					INIT_WORK(&led->led_off_work, led_off_work_func); 
				}
				INIT_DELAYED_WORK(&led->blink_delayed_work, led_blink_do_work);
			}
		}

		switch(led->id){
			case QPNP_ID_RGB_RED:
				g_led_red = led;
				break;
			case QPNP_ID_RGB_GREEN:
				g_led_green = led;
				break;
			case QPNP_ID_RGB_BLUE:
				g_led_blue = led;
				break;
			case QPNP_ID_LED_MPP:
				g_led_virtual = led;
				break;
		}
		LED_INFO("led id = %d, array index = %d\n",led->id,parsed_leds);
		parsed_leds++;
	}
	dev_set_drvdata(&spmi->dev, led_array);
	LED_INFO("led driver probe --\n");
	return 0;
err_create_work_queue:
fail_id_check:
	for (i = 0; i < parsed_leds; i++) {
		if (led_array[i].id != QPNP_ID_FLASH1_LED0 &&
				led_array[i].id != QPNP_ID_FLASH1_LED1)
			mutex_destroy(&led_array[i].lock);
		if (led_array[i].in_order_command_processing)
			destroy_workqueue(led_array[i].workqueue);
		led_classdev_unregister(&led_array[i].cdev);
	}

	return rc;
}

static int qpnp_leds_remove(struct spmi_device *spmi)
{
	struct qpnp_led_data *led_array  = dev_get_drvdata(&spmi->dev);
	int i, parsed_leds = led_array->num_leds;

	for (i = 0; i < parsed_leds; i++) {
		cancel_work_sync(&led_array[i].work);
		if (led_array[i].id != QPNP_ID_FLASH1_LED0 &&
				led_array[i].id != QPNP_ID_FLASH1_LED1)
			mutex_destroy(&led_array[i].lock);

		if (led_array[i].in_order_command_processing)
			destroy_workqueue(led_array[i].workqueue);
		led_classdev_unregister(&led_array[i].cdev);
		switch (led_array[i].id) {
		case QPNP_ID_RGB_RED:
		case QPNP_ID_RGB_GREEN:
		case QPNP_ID_RGB_BLUE:
			if (led_array[i].rgb_cfg->pwm_cfg->mode == PWM_MODE)
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &pwm_attr_group);
			if (led_array[i].rgb_cfg->pwm_cfg->use_blink) {
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &blink_attr_group);
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &lpg_attr_group);
			} else if (led_array[i].rgb_cfg->pwm_cfg->mode\
					== LPG_MODE)
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &lpg_attr_group);
			break;
		case QPNP_ID_LED_MPP:
			if (!led_array[i].mpp_cfg->pwm_cfg)
				break;
			if (led_array[i].mpp_cfg->pwm_cfg->mode == PWM_MODE)
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &pwm_attr_group);
			if (led_array[i].mpp_cfg->pwm_cfg->use_blink) {
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &blink_attr_group);
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &lpg_attr_group);
			} else if (led_array[i].mpp_cfg->pwm_cfg->mode\
					== LPG_MODE)
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &lpg_attr_group);
			if (led_array[i].mpp_cfg->mpp_reg)
				regulator_put(led_array[i].mpp_cfg->mpp_reg);
			break;
		default:
			dev_err(&led_array[i].spmi_dev->dev,
					"Invalid LED(%d)\n",
					led_array[i].id);
			return -EINVAL;
		}
	}

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id spmi_match_table[] = {
	{ .compatible = "qcom,rgb-qpnp",},
	{ },
};
#else
#define spmi_match_table NULL
#endif

static struct spmi_driver qpnp_leds_driver = {
	.driver		= {
		.name	= "qcom,rgb-qpnp",
		.of_match_table = spmi_match_table,
	},
	.probe		= qpnp_leds_probe,
	.remove		= qpnp_leds_remove,
};

static int __init qpnp_led_init(void)
{
	return spmi_driver_register(&qpnp_leds_driver);
}
module_init(qpnp_led_init);

static void __exit qpnp_led_exit(void)
{
	spmi_driver_unregister(&qpnp_leds_driver);
}
module_exit(qpnp_led_exit);

MODULE_DESCRIPTION("QPNP LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-qpnp-rgb");

