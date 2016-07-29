/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SUBSYS_RESTART_H
#define __SUBSYS_RESTART_H

#include <linux/spinlock.h>
#include <linux/interrupt.h>

struct subsys_device;

enum {
	RESET_SOC = 0,
	RESET_SUBSYS_COUPLED,
	RESET_LEVEL_MAX
};

#if defined(CONFIG_HTC_FEATURES_SSR)
enum {
        DISABLE_RAMDUMP = 0,
        ENABLE_RAMDUMP,
};
#endif

struct device;
struct module;

struct subsys_desc {
	const char *name;
	char fw_name[256];
	const char *depends_on;
	struct device *dev;
	struct module *owner;

	int (*shutdown)(const struct subsys_desc *desc, bool force_stop);
	int (*powerup)(const struct subsys_desc *desc);
	void (*crash_shutdown)(const struct subsys_desc *desc);
	int (*ramdump)(int, const struct subsys_desc *desc);
	void (*free_memory)(const struct subsys_desc *desc);
	irqreturn_t (*err_fatal_handler) (int irq, void *dev_id);
	irqreturn_t (*stop_ack_handler) (int irq, void *dev_id);
	irqreturn_t (*wdog_bite_handler) (int irq, void *dev_id);
	int is_not_loadable;
	int err_fatal_gpio;
	unsigned int err_fatal_irq;
	unsigned int err_ready_irq;
	unsigned int stop_ack_irq;
	unsigned int wdog_bite_irq;
        int apps_reboot_gpio;
        int force_stop_gpio;
	int ramdump_disable_gpio;
	int shutdown_ack_gpio;
	int ramdump_disable;
	bool no_auth;
	int ssctl_instance_id;
	u32 sysmon_pid;
	int sysmon_shutdown_ret;
	bool system_debug;
	const char *edge;
#if 1 
       irqreturn_t (*reboot_req_handler) (int irq, void *dev_id);
       unsigned int reboot_req_irq;
#endif 
};

struct notif_data {
	bool crashed;
	int enable_ramdump;
	int enable_mini_ramdumps;
	bool no_auth;
	struct platform_device *pdev;
};

extern void htc_smp2p_notify_modem_app_reboot( bool enable );
extern bool htc_check_modem_crash_status ( void );

#if defined(CONFIG_MSM_SUBSYSTEM_RESTART)

#if defined(CONFIG_HTC_DEBUG_SSR)
void subsys_set_restart_reason(struct subsys_device *dev, const char *reason);
#endif 

#if defined(CONFIG_HTC_FEATURES_SSR)
extern void subsys_set_enable_ramdump(struct subsys_device *dev, int enable);
extern void subsys_set_restart_level(struct subsys_device *dev, int level);
extern void subsys_config_modem_enable_ramdump(struct subsys_device *dev);
extern void subsys_config_modem_restart_level(struct subsys_device *dev);
#endif

#if defined(CONFIG_HTC_FEATURES_SSR)
void subsys_config_enable_ramdump(struct subsys_device *dev);
void subsys_config_restart_level(struct subsys_device *dev);
#else
static inline void subsys_config_enable_ramdump(struct subsys_device *dev)
{
	return;
}

static inline void subsys_config_restart_level(struct subsys_device *dev)
{
	return;
}
#endif

extern int subsys_get_restart_level(struct subsys_device *dev);
extern int subsystem_restart_dev(struct subsys_device *dev);
extern int subsystem_restart(const char *name);
extern int subsystem_crashed(const char *name);

extern void *subsystem_get(const char *name);
extern void *subsystem_get_with_fwname(const char *name, const char *fw_name);
extern void subsystem_put(void *subsystem);

extern struct subsys_device *subsys_register(struct subsys_desc *desc);
extern void subsys_unregister(struct subsys_device *dev);

extern void subsys_default_online(struct subsys_device *dev);
extern void subsys_set_crash_status(struct subsys_device *dev, bool crashed);
extern bool subsys_get_crash_status(struct subsys_device *dev);
void notify_proxy_vote(struct device *device);
void notify_proxy_unvote(struct device *device);
extern int wait_for_shutdown_ack(struct subsys_desc *desc);
#else

#if defined(CONFIG_HTC_DEBUG_SSR)
static inline void subsys_set_restart_reason(struct subsys_device *dev, const char *reason)
{
	return;
}
#endif 

static inline int subsys_get_restart_level(struct subsys_device *dev)
{
	return 0;
}

static inline int subsystem_restart_dev(struct subsys_device *dev)
{
	return 0;
}

static inline int subsystem_restart(const char *name)
{
	return 0;
}

static inline int subsystem_crashed(const char *name)
{
	return 0;
}

static inline void *subsystem_get(const char *name)
{
	return NULL;
}

static inline void *subsystem_get_with_fwname(const char *name,
				const char *fw_name) {
	return NULL;
}

static inline void subsystem_put(void *subsystem) { }

static inline
struct subsys_device *subsys_register(struct subsys_desc *desc)
{
	return NULL;
}

static inline void subsys_unregister(struct subsys_device *dev) { }

static inline void subsys_default_online(struct subsys_device *dev) { }
static inline
void subsys_set_crash_status(struct subsys_device *dev, bool crashed) { }
static inline bool subsys_get_crash_status(struct subsys_device *dev)
{
	return false;
}
static inline void notify_proxy_vote(struct device *device) { }
static inline void notify_proxy_unvote(struct device *device) { }
static inline int wait_for_shutdown_ack(struct subsys_desc *desc)
{
	return -ENOSYS;
}
#endif 

#endif
