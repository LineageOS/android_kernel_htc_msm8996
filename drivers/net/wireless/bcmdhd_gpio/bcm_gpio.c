/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
#include <linux/export.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/sched.h>
#include <linux/pm_qos.h>
#include <linux/esoc_client.h>
#include <linux/pinctrl/consumer.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/subsystem_notif.h>
#include <soc/qcom/ramdump.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>

#ifdef CONFIG_PCI_MSM
#include <linux/msm_pcie.h>
#else
#include <mach/msm_pcie.h>
#endif

#include "bcm_dev_reg.h"

#define WLAN_VREG_NAME		"vdd-wlan"
#define WLAN_VREG_IO_NAME	"vdd-wlan-io"
#define WLAN_VREG_XTAL_NAME	"vdd-wlan-xtal"
#define WLAN_SWREG_NAME		"wlan-soc-swreg"
#define WLAN_EN_GPIO_NAME	"wlan-en-gpio"
#define WLAN_IRQ_GPIO_NAME	"wlan-irq-gpio"
#define WLAN_SECI_IN_NAME   "wlan-seci-in"
#define WLAN_SECI_OUT_NAME  "wlan-seci-out"

#define SOC_SWREG_VOLT_MAX	1200000
#define SOC_SWREG_VOLT_MIN	1200000
#define WLAN_VREG_IO_MAX	1800000
#define WLAN_VREG_IO_MIN	1800000
#define WLAN_VREG_XTAL_MAX	1800000
#define WLAN_VREG_XTAL_MIN	1800000

#define WLAN_ENABLE_DELAY	10000
#define BCM_PINCTRL_STATE_ACTIVE "default"

#define BCM_ENUMERATE 1
#ifdef BCM_ENUMERATE
#undef BCM_ENUMERATE
#endif

struct wlan_vreg_info {
	struct regulator *wlan_reg;
	struct regulator *soc_swreg;
	struct regulator *wlan_reg_io;
	struct regulator *wlan_reg_xtal;
	bool state;
};

struct wlan_vreg_info vreg_info;
struct wlan_gpio_info gpio_wl_en;
struct wlan_gpio_info gpio_irq;
struct wlan_gpio_info gpio_seci_in;
struct wlan_gpio_info gpio_seci_out;
struct platform_device *gdev;
int htc_wifi_rc_num = -1;

static int htc_wifi_gpio_init(struct wlan_gpio_info *info)
{
	int ret = 0;

	ret = gpio_request(info->num, info->name);

	if (ret) {
		pr_err("[WLAN] can't get gpio %s ret %d\n", info->name, ret);
		goto err_gpio_req;
	}

    if ( !strcmp(info->name, WLAN_EN_GPIO_NAME) ) {
		pr_err("[WLAN] %s: gpio %d -> direction_output\n", __func__, info->num);
		ret = gpio_direction_output(info->num, info->init);

		if (ret) {
			pr_err("[WLAN] can't set gpio direction %s ret %d\n", info->name, ret);
			goto err_gpio_dir;
		}
    } else if (!strcmp(info->name, WLAN_IRQ_GPIO_NAME)) {
        pr_err("[WLAN] %s: gpio %d -> direction_input\n", __func__, info->num);
        ret = gpio_direction_input(info->num);

        if (ret) {
            pr_err("[WLAN] can't set gpio direction %s ret %d\n", info->name, ret);
            goto err_gpio_dir;
        }
	} else if (!strcmp(info->name, WLAN_SECI_IN_NAME)) {
		pr_err("[WLAN] %s: gpio %d -> direction_input\n", __func__, info->num);
		ret = gpio_direction_input(info->num);

		if (ret) {
		    pr_err("[WLAN] can't set gpio direction %s ret %d\n", info->name, ret);
		    goto err_gpio_dir;
		}
	} else if (!strcmp(info->name, WLAN_SECI_OUT_NAME)) {
		pr_err("[WLAN] %s: gpio %d -> direction_input\n", __func__, info->num);
		ret = gpio_direction_output(info->num, info->init);

		if (ret) {
			pr_err("[WLAN] can't set gpio direction %s ret %d\n", info->name, ret);
			goto err_gpio_dir;
		}
	}



	info->state = info->init;

	return ret;

err_gpio_dir:
	gpio_free(info->num);

err_gpio_req:

	return ret;
}

static int htc_wifi_pinctrl_init(struct platform_device *pdev)
{
	int ret;
    struct pinctrl *pinctrl;
    struct pinctrl_state *set_state;

    pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR_OR_NULL(pinctrl)) {
        pr_err("[WLAN] %s: Failed to get pinctrl!\n", __func__);
        return PTR_ERR(pinctrl);
    }

    set_state = pinctrl_lookup_state(pinctrl, "default");
    if (IS_ERR_OR_NULL(set_state)) {
        pr_err("[WLAN] %s: Can not get active pin state!\n", __func__);
        return PTR_ERR(set_state);
    }

    ret = pinctrl_select_state(pinctrl, set_state);

	return ret;
}

static int htc_wifi_get_resources(struct platform_device *pdev, struct wlan_gpio_info *gpio_info)
{
	int ret = 0;

	if (!of_find_property((&pdev->dev)->of_node, gpio_info->name, NULL)) {
		gpio_info->prop = false;
		goto end;
	}

	gpio_info->prop = true;
	ret = of_get_named_gpio((&pdev->dev)->of_node,
				gpio_info->name, 0);

	if (ret >= 0) {
		gpio_info->num = ret;
		ret = 0;
	} else {
		if (ret == -EPROBE_DEFER)
			pr_debug("[WLAN] get WLAN_EN GPIO probe defer\n");
		else
			pr_err("[WLAN] can't get gpio %s ret %d",
			       gpio_info->name, ret);
		goto err_get_gpio;
	}

	ret = htc_wifi_gpio_init(gpio_info);

	if (ret) {
		pr_err("[WLAN] gpio init failed\n");
		goto err_gpio_init;
	}

end:
	return ret;

err_gpio_init:
err_get_gpio:
	vreg_info.state = VREG_OFF;

	return ret;
}

int htc_wifi_gpio_set(struct wlan_gpio_info *info, bool state)
{
    int time = 0;
	if (!info->prop) {
		printk("[WLAN] of_find_property() failed for %s", info->name);
		return -1;
	}

	if (info->state == state) {
		printk("[WLAN] %s gpio is already %s\n",
			 info->name, state ? "high" : "low");
		return -1;
	}

	gpio_set_value(info->num, state);

    while (gpio_get_value(info->num) != state && (time < 40)) {
        usleep_range(50000, 50001);
        time++;
    }
	info->state = state;
    if (time >= 40) {
        printk("[WLAN] %s failed : %s\n", __func__,state ? "enabled" : "disabled");
    }
	printk("[WLAN] %s gpio is now %s, retry %d time\n", info->name, info->state ? "enabled" : "disabled", time);

	return 0;
}

static void htc_wifi_release_resources(void)
{
	gpio_free(gpio_wl_en.num);
	gpio_free(gpio_irq.num);
	gpio_free(gpio_seci_in.num);
	gpio_free(gpio_seci_out.num);
	gpio_wl_en.state = WLAN_EN_LOW;
	gpio_wl_en.prop = false;
	vreg_info.state = VREG_OFF;
}

static int htc_wifi_gpio_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;

	printk("[WLAN] %s enter\n", __FUNCTION__);
    gdev = pdev;

	
	gpio_wl_en.name = WLAN_EN_GPIO_NAME;
	gpio_wl_en.num = 0;
	gpio_wl_en.state = WLAN_EN_LOW;
	gpio_wl_en.init = WLAN_EN_LOW;
	gpio_wl_en.prop = false;

	
	gpio_irq.name = WLAN_IRQ_GPIO_NAME;
	gpio_irq.num = 0;
	gpio_irq.state = WLAN_EN_LOW;
	gpio_irq.init = WLAN_EN_LOW;
	gpio_irq.prop = false;

	
	gpio_seci_in.name = WLAN_SECI_IN_NAME;
	gpio_seci_in.num = 0;
	gpio_seci_in.state = WLAN_EN_LOW;
	gpio_seci_in.init = WLAN_EN_LOW;
	gpio_seci_in.prop = false;

	
	gpio_seci_out.name = WLAN_SECI_OUT_NAME;
	gpio_seci_out.num = 0;
	gpio_seci_out.state = WLAN_EN_LOW;
	gpio_seci_out.init = WLAN_EN_LOW;
	gpio_seci_out.prop = false;


	vreg_info.wlan_reg = NULL;
	vreg_info.state = VREG_OFF;

	ret = htc_wifi_get_resources(pdev, &gpio_wl_en);
	if (ret)
		goto err_get_wlan_res;

	ret = htc_wifi_get_resources(pdev, &gpio_irq);
	if (ret)
		goto err_get_wlan_res;

	ret = htc_wifi_get_resources(pdev, &gpio_seci_in);
	if (ret)
	    goto err_get_wlan_res;

	ret = htc_wifi_get_resources(pdev, &gpio_seci_out);
	if (ret)
	    goto err_get_wlan_res;

    ret = htc_wifi_pinctrl_init(pdev);
    if (ret) {
        pr_err("[WLAN] %s: pinctrl init failed!\n", __FUNCTION__);
        goto err_get_wlan_res;
    }

    ret = of_property_read_u32(dev->of_node, "htc,wlan-rc-num", &htc_wifi_rc_num);
    pr_err("[WLAN] htc_wifi_rc_num = %d\n", htc_wifi_rc_num);
    if (ret) {
        pr_err("[WLAN] %s: Failed to find PCIe RC number!\n", __FUNCTION__);
        goto err_get_rc;
    }

#ifdef BCM_ENUMERATE
    
	htc_wifi_gpio_set(&gpio_wl_en, WLAN_EN_HIGH);
	usleep(WLAN_ENABLE_DELAY);

	ret = msm_pcie_enumerate(rc_num);
	if (ret) {
		pr_err("[WLAN] %s: Failed to enable PCIe RC%x!\n", __FUNCTION__, rc_num);
		goto err_pcie_enumerate;
	}
#endif

	return 0;

err_get_wlan_res:
err_get_rc:
#ifdef BCM_ENUMERATE
err_pcie_enumerate:
#endif
	pr_err("[WLAN] %s@%d err! ret %d\n", __FUNCTION__, __LINE__, ret);
	return ret;
}

static int htc_wifi_gpio_remove(struct platform_device *pdev)
{
	htc_wifi_gpio_set(&gpio_wl_en, WLAN_EN_LOW);
	htc_wifi_release_resources();

	return 0;
}

static const struct of_device_id htc_wifi_dt_match[] = {
	{.compatible = "htc_wifi_gpio"},
	{}
};

MODULE_DEVICE_TABLE(of, htc_wifi_dt_match);

static struct platform_driver htc_wifi_gpio_driver = {
	.probe  = htc_wifi_gpio_probe,
	.remove = htc_wifi_gpio_remove,
	.driver = {
		.name = "wifi_gpio",
		.owner = THIS_MODULE,
		.of_match_table = htc_wifi_dt_match,
	},
};

static int __init htc_wifi_init(void)
{
	
	htc_wifi_dev_init();

	return platform_driver_register(&htc_wifi_gpio_driver);
}

static void __exit htc_wifi_exit(void)
{
	platform_driver_unregister(&htc_wifi_gpio_driver);
}

module_init(htc_wifi_init);
module_exit(htc_wifi_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DEVICE "Init BCM GPIO");
