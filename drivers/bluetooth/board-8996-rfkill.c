/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2014 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

#ifndef __LPM_BLUESLEEP__
#define __LPM_BLUESLEEP__   1
#endif

extern void bt_export_bd_address(void);
extern void msm_hs_uart_gpio_config_ext(int on);
#ifdef __LPM_BLUESLEEP__
extern void bluesleep_set_bt_pwr_state(int on);
#endif

/* BT chip power up and wakeup pins (these define not used)*/
#define BT_REG_ON    73
#define BT_WAKE_HOST 117
#define BT_WAKE_DEV  118

/* UART pins (these define not used)*/
#define BT_UART_RTSz  44
#define BT_UART_CTSz  43
#define BT_UART_RX    42
#define BT_UART_TX    41

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4359";

/* BT GPIO pins variable */
static int gpio_bt_reg_on;

/* BT_WAKE_HOST pin control */
struct pinctrl *bt_pinctrl;
struct pinctrl_state *bt_wake_host_set_state_on;
struct pinctrl_state *bt_wake_host_set_state_off;


static void htc_config_bt_on(void)
{
	int rc = 0;
	printk(KERN_INFO "[BT] == R ON ==\n");

	if (gpio_bt_reg_on < 0) {
		printk(KERN_INFO "[BT] bt_reg_on:%d !!\n", gpio_bt_reg_on);
	}

	//sleep clk auto enable since system boot up
	//htc_wifi_bt_sleep_clk_ctl(CLK_ON, ID_BT);
	//mdelay(2);

	/* Config UART pins */
	msm_hs_uart_gpio_config_ext(1);

	/* Host wake setup to I(PU) per data sheet */
	rc = pinctrl_select_state(bt_pinctrl, bt_wake_host_set_state_on);
	if (rc) printk("[BT] cannot set BT pinctrl gpio state on: %d\n", rc);

	/* Power up BT controller */
	rc = gpio_direction_output(gpio_bt_reg_on, 0);
	if (rc) printk(KERN_INFO "[BT] set REG_ON 0 fail: %d\n", rc);
	mdelay(5);
	rc = gpio_direction_output(gpio_bt_reg_on, 1);
	if (rc) printk(KERN_INFO "[BT] set REG_ON 1 fail: %d\n", rc);
	mdelay(5);

#ifdef __LPM_BLUESLEEP__
	/* Notify sleep driver BT state */
	bluesleep_set_bt_pwr_state(1);
#endif
}

static void htc_config_bt_off(void)
{
	int rc = 0;

	if (gpio_bt_reg_on < 0) {
		printk(KERN_INFO "[BT] bt_reg_on:%d !!\n", gpio_bt_reg_on);
	}

#ifdef __LPM_BLUESLEEP__
	/* Notify sleep driver BT state */
	bluesleep_set_bt_pwr_state(0);
#endif

	/* Power off BT controller */
	rc = gpio_direction_output(gpio_bt_reg_on, 0);
	if (rc) printk(KERN_INFO "[BT] set REG_ON 0 fail: %d\n", rc);

	/* Host wake setup to I(PD) per GPIO table */
	rc = pinctrl_select_state(bt_pinctrl, bt_wake_host_set_state_off);
	if (rc) printk("[BT] cannot set BT pinctrl gpio state off: %d\n", rc);

	mdelay(2);

	/* Config UART pins */
	msm_hs_uart_gpio_config_ext(0);

	printk(KERN_INFO "[BT] == R OFF ==\n");
}

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked) {
		htc_config_bt_on();
	} else
		htc_config_bt_off();

	return 0;
}

static struct rfkill_ops htc_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int htc_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true;  /* off */
	struct pinctrl_state *set_state;

	printk(KERN_INFO "[BT] == rfkill_probe ==\n");

	bt_export_bd_address();

	// Get bt_reg_pin
	if (pdev->dev.of_node) {
		gpio_bt_reg_on = of_get_named_gpio(pdev->dev.of_node,
							"brcm,bt-regon-gpio", 0);
		if (gpio_bt_reg_on < 0) {
			printk(KERN_ERR "[BT] bt-regon-gpio not provided in device tree!!!\n");
			return -EPROBE_DEFER;
		} else {
			rc = gpio_request(gpio_bt_reg_on, "BRCM_BT_EN");
			if (rc) {
				printk(KERN_ERR "[BT] can't get gpio: %d for bt-regon-gpio\n", gpio_bt_reg_on);
			} else {
				printk(KERN_INFO "[BT] bt-regon-gpio: %d\n", gpio_bt_reg_on);
			}
		}
	}

	// Init pin control
	bt_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(bt_pinctrl)) {
		if (PTR_ERR(bt_pinctrl) == -EPROBE_DEFER) {
			printk("[BT] bt_pinctrl EPROBE_DEFER\n");
			return -EPROBE_DEFER;
		}
	}

	if (bt_pinctrl) {
		printk("[BT] Init GPIO pins\n");
		set_state = pinctrl_lookup_state(bt_pinctrl, "bt_wake_host_gpio_on");
		if (IS_ERR(set_state)) {
			printk("[BT] cannot get BT pinctrl state bt_wake_host_gpio_on\n");
			//return PTR_ERR(set_state);
		} else
			bt_wake_host_set_state_on = set_state;

		set_state = pinctrl_lookup_state(bt_pinctrl, "bt_wake_host_gpio_off");
		if (IS_ERR(set_state)) {
			printk("[BT] cannot get BT pinctrl state bt_wake_host_gpio_off\n");
			//return PTR_ERR(set_state);
		} else
			bt_wake_host_set_state_off = set_state;

	}

	/* Sleep clock always on */
	/* htc_wifi_bt_sleep_clk_ctl(CLK_ON, ID_BT); */
	//mdelay(2);

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
				&htc_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_alloc;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	/* userspace cannot take exclusive control */

	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
    gpio_free(gpio_bt_reg_on);
	return rc;
}

static int htc_rfkill_remove(struct platform_device *dev)
{

	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);

	return 0;
}

static const struct of_device_id htc_rfkill_match_table[] = {
	{ .compatible = "htc,htcbt_rfkill", },
	{},
};

static struct platform_driver htc_rfkill_driver = {
	.probe = htc_rfkill_probe,
	.remove = htc_rfkill_remove,
	.driver = {
		.name = "htcbt_rfkill",
		.owner = THIS_MODULE,
		.of_match_table = htc_rfkill_match_table,
	},
};

static int __init htc_rfkill_init(void)
{
	return platform_driver_register(&htc_rfkill_driver);
}

static void __exit htc_rfkill_exit(void)
{
	platform_driver_unregister(&htc_rfkill_driver);
}

module_init(htc_rfkill_init);
module_exit(htc_rfkill_exit);
MODULE_DESCRIPTION("htc rfkill");
MODULE_AUTHOR("htc_ssdbt");
MODULE_LICENSE("GPL");
