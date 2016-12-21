/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define VREG_ON				1
#define VREG_OFF			0
#define WLAN_EN_HIGH		1
#define WLAN_EN_LOW			0
#define PCIE_LINK_UP		1
#define PCIE_LINK_DOWN		0

struct wlan_gpio_info {
	char *name;
	u32 num;
	bool state;
	bool init;
	bool prop;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
};

int htc_wifi_dev_init(void);
int htc_wifi_gpio_set(struct wlan_gpio_info *info, bool state);
