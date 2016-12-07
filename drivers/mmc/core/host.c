/*
 *  linux/drivers/mmc/core/host.c
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *  Copyright (C) 2007-2008 Pierre Ossman
 *  Copyright (C) 2010 Linus Walleij
 *  Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  MMC host class device management
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pagemap.h>
#include <linux/export.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/suspend.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/slot-gpio.h>

#include "core.h"
#include "host.h"

#define cls_dev_to_mmc_host(d)	container_of(d, struct mmc_host, class_dev)
#define MMC_DEVFRQ_DEFAULT_UP_THRESHOLD 35
#define MMC_DEVFRQ_DEFAULT_DOWN_THRESHOLD 5
#define MMC_DEVFRQ_DEFAULT_POLLING_MSEC 100

extern struct workqueue_struct *stats_workqueue;

static void mmc_host_classdev_release(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	mutex_destroy(&host->slot.lock);
	kfree(host);
}

static struct class mmc_host_class = {
	.name		= "mmc_host",
	.dev_release	= mmc_host_classdev_release,
};

int mmc_register_host_class(void)
{
	return class_register(&mmc_host_class);
}

void mmc_unregister_host_class(void)
{
	class_unregister(&mmc_host_class);
}

static DEFINE_IDR(mmc_host_idr);
static DEFINE_SPINLOCK(mmc_host_lock);

#ifdef CONFIG_MMC_CLKGATE
static ssize_t clkgate_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	return snprintf(buf, PAGE_SIZE, "%lu\n", host->clkgate_delay);
}

static ssize_t clkgate_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long flags, value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clkgate_delay = value;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	return count;
}

static void mmc_host_clk_gate_delayed(struct mmc_host *host)
{
	unsigned long tick_ns;
	unsigned long freq = host->ios.clock;
	unsigned long flags;

	if (!freq) {
		pr_debug("%s: frequency set to 0 in disable function, "
			 "this means the clock is already disabled.\n",
			 mmc_hostname(host));
		return;
	}
	spin_lock_irqsave(&host->clk_lock, flags);

	if (!host->clk_requests) {
		spin_unlock_irqrestore(&host->clk_lock, flags);
		tick_ns = DIV_ROUND_UP(1000000000, freq);
		ndelay(host->clk_delay * tick_ns);
	} else {
		
		spin_unlock_irqrestore(&host->clk_lock, flags);
		return;
	}
	mutex_lock(&host->clk_gate_mutex);
	spin_lock_irqsave(&host->clk_lock, flags);
	if (!host->clk_requests) {
		spin_unlock_irqrestore(&host->clk_lock, flags);
		
		mmc_gate_clock(host);
		spin_lock_irqsave(&host->clk_lock, flags);
		pr_debug("%s: gated MCI clock\n", mmc_hostname(host));
	}
	spin_unlock_irqrestore(&host->clk_lock, flags);
	mutex_unlock(&host->clk_gate_mutex);
}

static void mmc_host_clk_gate_work(struct work_struct *work)
{
	struct mmc_host *host = container_of(work, struct mmc_host,
					      clk_gate_work.work);

	mmc_host_clk_gate_delayed(host);
}

void mmc_host_clk_hold(struct mmc_host *host)
{
	unsigned long flags;

	
	cancel_delayed_work_sync(&host->clk_gate_work);
	mutex_lock(&host->clk_gate_mutex);
	spin_lock_irqsave(&host->clk_lock, flags);
	if (host->clk_gated) {
		spin_unlock_irqrestore(&host->clk_lock, flags);
		mmc_ungate_clock(host);

		spin_lock_irqsave(&host->clk_lock, flags);
		pr_debug("%s: ungated MCI clock\n", mmc_hostname(host));
	}
	host->clk_requests++;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	mutex_unlock(&host->clk_gate_mutex);
}

bool mmc_host_may_gate_card(struct mmc_card *card)
{
	
	if (!card)
		return true;

	if (mmc_card_sdio(card) && card->cccr.async_intr_sup)
			return true;

	return !(card->quirks & MMC_QUIRK_BROKEN_CLK_GATING);
}

void mmc_host_clk_release(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clk_requests--;
	if (mmc_host_may_gate_card(host->card) &&
	    !host->clk_requests)
		schedule_delayed_work(&host->clk_gate_work,
				      msecs_to_jiffies(host->clkgate_delay));
	spin_unlock_irqrestore(&host->clk_lock, flags);
}

unsigned int mmc_host_clk_rate(struct mmc_host *host)
{
	unsigned long freq;
	unsigned long flags;

	spin_lock_irqsave(&host->clk_lock, flags);
	if (host->clk_gated)
		freq = host->clk_old;
	else
		freq = host->ios.clock;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	return freq;
}

static inline void mmc_host_clk_init(struct mmc_host *host)
{
	host->clk_requests = 0;
	
	host->clk_delay = 8;
	host->clkgate_delay = 0;
	host->clk_gated = false;
	INIT_DELAYED_WORK(&host->clk_gate_work, mmc_host_clk_gate_work);
	spin_lock_init(&host->clk_lock);
	mutex_init(&host->clk_gate_mutex);
}

static inline void mmc_host_clk_exit(struct mmc_host *host)
{
	if (cancel_delayed_work_sync(&host->clk_gate_work))
		mmc_host_clk_gate_delayed(host);
	if (host->clk_gated)
		mmc_host_clk_hold(host);
	
	WARN_ON(host->clk_requests > 1);
}

static inline void mmc_host_clk_sysfs_init(struct mmc_host *host)
{
	host->clkgate_delay_attr.show = clkgate_delay_show;
	host->clkgate_delay_attr.store = clkgate_delay_store;
	sysfs_attr_init(&host->clkgate_delay_attr.attr);
	host->clkgate_delay_attr.attr.name = "clkgate_delay";
	host->clkgate_delay_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(&host->class_dev, &host->clkgate_delay_attr))
		pr_err("%s: Failed to create clkgate_delay sysfs entry\n",
				mmc_hostname(host));
}
#else

static inline void mmc_host_clk_init(struct mmc_host *host)
{
}

static inline void mmc_host_clk_exit(struct mmc_host *host)
{
}

static inline void mmc_host_clk_sysfs_init(struct mmc_host *host)
{
}

#endif

int mmc_of_parse(struct mmc_host *host)
{
	struct device_node *np;
	u32 bus_width;
	int len, ret;
	bool cd_cap_invert, cd_gpio_invert = false;
	bool ro_cap_invert, ro_gpio_invert = false;

	if (!host->parent || !host->parent->of_node)
		return 0;

	np = host->parent->of_node;

	
	if (of_property_read_u32(np, "bus-width", &bus_width) < 0) {
		dev_dbg(host->parent,
			"\"bus-width\" property is missing, assuming 1 bit.\n");
		bus_width = 1;
	}

	switch (bus_width) {
	case 8:
		host->caps |= MMC_CAP_8_BIT_DATA;
		
	case 4:
		host->caps |= MMC_CAP_4_BIT_DATA;
		break;
	case 1:
		break;
	default:
		dev_err(host->parent,
			"Invalid \"bus-width\" value %u!\n", bus_width);
		return -EINVAL;
	}

	
	of_property_read_u32(np, "max-frequency", &host->f_max);


	
	if (of_find_property(np, "non-removable", &len)) {
		host->caps |= MMC_CAP_NONREMOVABLE;
	} else {
		cd_cap_invert = of_property_read_bool(np, "cd-inverted");

		if (of_find_property(np, "broken-cd", &len))
			host->caps |= MMC_CAP_NEEDS_POLL;

		ret = mmc_gpiod_request_cd(host, "cd", 0, true,
					   0, &cd_gpio_invert);
		if (ret) {
			if (ret == -EPROBE_DEFER)
				return ret;
			if (ret != -ENOENT) {
				dev_err(host->parent,
					"Failed to request CD GPIO: %d\n",
					ret);
			}
		} else
			dev_info(host->parent, "Got CD GPIO\n");

		if (cd_cap_invert ^ cd_gpio_invert)
			host->caps2 |= MMC_CAP2_CD_ACTIVE_HIGH;
	}

	
	ro_cap_invert = of_property_read_bool(np, "wp-inverted");

	ret = mmc_gpiod_request_ro(host, "wp", 0, false, 0, &ro_gpio_invert);
	if (ret) {
		if (ret == -EPROBE_DEFER)
			goto out;
		if (ret != -ENOENT) {
			dev_err(host->parent,
				"Failed to request WP GPIO: %d\n",
				ret);
		}
	} else
		dev_info(host->parent, "Got WP GPIO\n");

	
	if (ro_cap_invert ^ ro_gpio_invert)
		host->caps2 |= MMC_CAP2_RO_ACTIVE_HIGH;

	if (of_find_property(np, "cap-sd-highspeed", &len))
		host->caps |= MMC_CAP_SD_HIGHSPEED;
	if (of_find_property(np, "cap-mmc-highspeed", &len))
		host->caps |= MMC_CAP_MMC_HIGHSPEED;
	if (of_find_property(np, "sd-uhs-sdr12", &len))
		host->caps |= MMC_CAP_UHS_SDR12;
	if (of_find_property(np, "sd-uhs-sdr25", &len))
		host->caps |= MMC_CAP_UHS_SDR25;
	if (of_find_property(np, "sd-uhs-sdr50", &len))
		host->caps |= MMC_CAP_UHS_SDR50;
	if (of_find_property(np, "sd-uhs-sdr104", &len))
		host->caps |= MMC_CAP_UHS_SDR104;
	if (of_find_property(np, "sd-uhs-ddr50", &len))
		host->caps |= MMC_CAP_UHS_DDR50;
	if (of_find_property(np, "cap-power-off-card", &len))
		host->caps |= MMC_CAP_POWER_OFF_CARD;
	if (of_find_property(np, "cap-sdio-irq", &len))
		host->caps |= MMC_CAP_SDIO_IRQ;
	if (of_find_property(np, "full-pwr-cycle", &len))
		host->caps2 |= MMC_CAP2_FULL_PWR_CYCLE;
	if (of_find_property(np, "keep-power-in-suspend", &len))
		host->pm_caps |= MMC_PM_KEEP_POWER;
	if (of_find_property(np, "enable-sdio-wakeup", &len))
		host->pm_caps |= MMC_PM_WAKE_SDIO_IRQ;
	if (of_find_property(np, "mmc-ddr-1_8v", &len))
		host->caps |= MMC_CAP_1_8V_DDR;
	if (of_find_property(np, "mmc-ddr-1_2v", &len))
		host->caps |= MMC_CAP_1_2V_DDR;
	if (of_find_property(np, "mmc-hs200-1_8v", &len))
		host->caps2 |= MMC_CAP2_HS200_1_8V_SDR;
	if (of_find_property(np, "mmc-hs200-1_2v", &len))
		host->caps2 |= MMC_CAP2_HS200_1_2V_SDR;
	if (of_find_property(np, "mmc-hs400-1_8v", &len))
		host->caps2 |= MMC_CAP2_HS400_1_8V | MMC_CAP2_HS200_1_8V_SDR;
	if (of_find_property(np, "mmc-hs400-1_2v", &len))
		host->caps2 |= MMC_CAP2_HS400_1_2V | MMC_CAP2_HS200_1_2V_SDR;

	host->dsr_req = !of_property_read_u32(np, "dsr", &host->dsr);
	if (host->dsr_req && (host->dsr & ~0xffff)) {
		dev_err(host->parent,
			"device tree specified broken value for DSR: 0x%x, ignoring\n",
			host->dsr);
		host->dsr_req = 0;
	}

	return 0;

out:
	mmc_gpio_free_cd(host);
	return ret;
}

EXPORT_SYMBOL(mmc_of_parse);

struct mmc_host *mmc_alloc_host(int extra, struct device *dev)
{
	int err;
	struct mmc_host *host;

	host = kzalloc(sizeof(struct mmc_host) + extra, GFP_KERNEL);
	if (!host)
		return NULL;

	
	host->rescan_disable = 1;
	idr_preload(GFP_KERNEL);
	spin_lock(&mmc_host_lock);
	err = idr_alloc(&mmc_host_idr, host, 0, 0, GFP_NOWAIT);
	if (err >= 0)
		host->index = err;
	spin_unlock(&mmc_host_lock);
	idr_preload_end();
	if (err < 0)
		goto free;

	dev_set_name(&host->class_dev, "mmc%d", host->index);

	host->parent = dev;
	host->class_dev.parent = dev;
	host->class_dev.class = &mmc_host_class;
	device_initialize(&host->class_dev);

	mmc_host_clk_init(host);

	mutex_init(&host->slot.lock);
	host->slot.cd_irq = -EINVAL;

	spin_lock_init(&host->lock);
	init_waitqueue_head(&host->wq);
	INIT_DELAYED_WORK(&host->detect, mmc_rescan);
	INIT_DELAYED_WORK(&host->stats_work, mmc_stats);

	host->perf.cmdq_read_map = 0;
	host->perf.cmdq_write_map = 0;
#ifdef CONFIG_PM
	host->pm_notify.notifier_call = mmc_pm_notify;
#endif

	host->max_segs = 1;
	host->max_seg_size = PAGE_CACHE_SIZE;

	host->max_req_size = PAGE_CACHE_SIZE;
	host->max_blk_size = 512;
	host->max_blk_count = PAGE_CACHE_SIZE / 512;

	return host;

free:
	kfree(host);
	return NULL;
}

EXPORT_SYMBOL(mmc_alloc_host);

static ssize_t show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	if (!host)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n", mmc_can_scale_clk(host));
}

static ssize_t store_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value;

	if (!host || !host->card || kstrtoul(buf, 0, &value))
		return -EINVAL;

	mmc_get_card(host->card);

	if (!value) {
		
		mmc_exit_clk_scaling(host);
		host->caps2 &= ~MMC_CAP2_CLK_SCALE;
		host->clk_scaling.state = MMC_LOAD_HIGH;
		
		mmc_clk_update_freq(host, host->card->clk_scaling_highest,
					host->clk_scaling.state);
	} else if (value) {
		
		host->caps2 |= MMC_CAP2_CLK_SCALE;
		if (host->clk_scaling.enable)
			mmc_exit_clk_scaling(host);
		mmc_init_clk_scaling(host);
	}

	mmc_put_card(host->card);

	return count;
}

static ssize_t show_up_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	if (!host)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n", host->clk_scaling.upthreshold);
}

#define MAX_PERCENTAGE	100
static ssize_t store_up_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value;

	if (!host || kstrtoul(buf, 0, &value) || (value > MAX_PERCENTAGE))
		return -EINVAL;

	host->clk_scaling.upthreshold = value;

	pr_debug("%s: clkscale_up_thresh set to %lu\n",
			mmc_hostname(host), value);
	return count;
}

static ssize_t show_down_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	if (!host)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			host->clk_scaling.downthreshold);
}

static ssize_t store_down_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value;

	if (!host || kstrtoul(buf, 0, &value) || (value > MAX_PERCENTAGE))
		return -EINVAL;

	host->clk_scaling.downthreshold = value;

	pr_debug("%s: clkscale_down_thresh set to %lu\n",
			mmc_hostname(host), value);
	return count;
}

static ssize_t show_polling(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	if (!host)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%lu milliseconds\n",
			host->clk_scaling.polling_delay_ms);
}

static ssize_t store_polling(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value;

	if (!host || kstrtoul(buf, 0, &value))
		return -EINVAL;

	host->clk_scaling.polling_delay_ms = value;

	pr_debug("%s: clkscale_polling_delay_ms set to %lu\n",
			mmc_hostname(host), value);
	return count;
}

DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		show_enable, store_enable);
DEVICE_ATTR(polling_interval, S_IRUGO | S_IWUSR,
		show_polling, store_polling);
DEVICE_ATTR(up_threshold, S_IRUGO | S_IWUSR,
		show_up_threshold, store_up_threshold);
DEVICE_ATTR(down_threshold, S_IRUGO | S_IWUSR,
		show_down_threshold, store_down_threshold);

static struct attribute *clk_scaling_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_up_threshold.attr,
	&dev_attr_down_threshold.attr,
	&dev_attr_polling_interval.attr,
	NULL,
};

static struct attribute_group clk_scaling_attr_grp = {
	.name = "clk_scaling",
	.attrs = clk_scaling_attrs,
};

static ssize_t
show_perf(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int64_t rtime_drv, wtime_drv;
	unsigned long rbytes_drv, wbytes_drv, flags;

	spin_lock_irqsave(&host->lock, flags);

	rbytes_drv = host->perf.rbytes_drv;
	wbytes_drv = host->perf.wbytes_drv;

	rtime_drv = ktime_to_us(host->perf.rtime_drv);
	wtime_drv = ktime_to_us(host->perf.wtime_drv);

	spin_unlock_irqrestore(&host->lock, flags);

	return snprintf(buf, PAGE_SIZE, "Write performance at driver Level:"
					"%lu bytes in %lld microseconds\n"
					"Read performance at driver Level:"
					"%lu bytes in %lld microseconds\n",
					wbytes_drv, wtime_drv,
					rbytes_drv, rtime_drv);
}

static ssize_t
set_perf(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int64_t value;
	unsigned long flags;

	sscanf(buf, "%lld", &value);
	host->debug_mask = value;
	pr_info("%s: set debug 0x%llx\n", mmc_hostname(host), value);
	spin_lock_irqsave(&host->lock, flags);
	if (!value) {
		memset(&host->perf, 0, sizeof(host->perf));
		host->perf_enable = false;
	} else {
		host->perf_enable = true;
	}
	spin_unlock_irqrestore(&host->lock, flags);

	return count;
}

static DEVICE_ATTR(perf, S_IRUGO | S_IWUSR,
		show_perf, set_perf);


static struct attribute *dev_attrs[] = {
	&dev_attr_perf.attr,
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

int mmc_add_host(struct mmc_host *host)
{
	int err;

	WARN_ON((host->caps & MMC_CAP_SDIO_IRQ) &&
		!host->ops->enable_sdio_irq);

	err = device_add(&host->class_dev);
	if (err)
		return err;

	led_trigger_register_simple(dev_name(&host->class_dev), &host->led);

	host->clk_scaling.upthreshold = MMC_DEVFRQ_DEFAULT_UP_THRESHOLD;
	host->clk_scaling.downthreshold = MMC_DEVFRQ_DEFAULT_DOWN_THRESHOLD;
	host->clk_scaling.polling_delay_ms = MMC_DEVFRQ_DEFAULT_POLLING_MSEC;
	host->clk_scaling.skip_clk_scale_freq_update = false;

#ifdef CONFIG_DEBUG_FS
	mmc_add_host_debugfs(host);
#endif
	mmc_host_clk_sysfs_init(host);

	err = sysfs_create_group(&host->class_dev.kobj, &clk_scaling_attr_grp);
	if (err)
		pr_err("%s: failed to create clk scale sysfs group with err %d\n",
				__func__, err);

	err = sysfs_create_group(&host->class_dev.kobj, &dev_attr_grp);
	if (err)
		pr_err("%s: failed to create sysfs group with err %d\n",
							 __func__, err);

	mmc_start_host(host);
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		register_pm_notifier(&host->pm_notify);

	return 0;
}

EXPORT_SYMBOL(mmc_add_host);

void mmc_remove_host(struct mmc_host *host)
{
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		unregister_pm_notifier(&host->pm_notify);

	mmc_stop_host(host);

#ifdef CONFIG_DEBUG_FS
	mmc_remove_host_debugfs(host);
#endif
	sysfs_remove_group(&host->parent->kobj, &dev_attr_grp);
	sysfs_remove_group(&host->class_dev.kobj, &clk_scaling_attr_grp);

	device_del(&host->class_dev);

	led_trigger_unregister_simple(host->led);

	mmc_host_clk_exit(host);
}

EXPORT_SYMBOL(mmc_remove_host);

void mmc_free_host(struct mmc_host *host)
{
	spin_lock(&mmc_host_lock);
	idr_remove(&mmc_host_idr, host->index);
	spin_unlock(&mmc_host_lock);

	put_device(&host->class_dev);
}

EXPORT_SYMBOL(mmc_free_host);
