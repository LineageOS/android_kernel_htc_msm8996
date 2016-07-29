/*
 * include/linux/keycombo.h - platform data structure for keycombo driver
 *
 * Copyright (C) 2014 Google, Inc.
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

#ifndef _LINUX_KEYCOMBO_H
#define _LINUX_KEYCOMBO_H

#define KEYCOMBO_NAME "keycombo"

#define KEY_LOGD(fmt, args...) printk(KERN_DEBUG "[KEY] "fmt, ##args)
#define KEY_LOGI(fmt, args...) printk(KERN_INFO "[KEY] "fmt, ##args)
#define KEY_LOGE(fmt, args...) printk(KERN_ERR "[KEY][ERR] "fmt, ##args)

struct keycombo_platform_data {
	void (*key_down_fn)(void *);
	void (*key_up_fn)(void *);
	void *priv;
	uint32_t key_down_delay; 
	uint32_t *keys_up;
	uint32_t *keys_down; 
	
	uint32_t *vzw_keys_up;
	uint32_t *vzw_keys_down; 
#if defined(CONFIG_POWER_KEY_CLR_RESET)
	uint32_t clr_gpio;
#endif
};

#if defined(CONFIG_POWER_KEY_CLR_RESET)
void clear_hw_reset(void);
#endif

#endif 
