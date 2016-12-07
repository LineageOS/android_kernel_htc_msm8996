/*
 *  linux/drivers/mmc/core/core.c
 *
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  SD support Copyright (C) 2004 Ian Molton, All Rights Reserved.
 *  Copyright (C) 2005-2008 Pierre Ossman, All Rights Reserved.
 *  MMCv4 support Copyright (C) 2006 Philip Langdale, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/devfreq.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/pagemap.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/scatterlist.h>
#include <linux/log2.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/suspend.h>
#include <linux/fault-inject.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/jiffies.h>
#include <linux/statfs.h>
#include <linux/debugfs.h>
#include <linux/blkdev.h>

#include <trace/events/mmc.h>
#include <trace/events/mmcio.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/list_sort.h>

#include "core.h"
#include "bus.h"
#include "host.h"
#include "sdio_bus.h"

#include "mmc_ops.h"
#include "sd_ops.h"
#include "sdio_ops.h"

#define MMC_CORE_TIMEOUT_MS	(10 * 60 * 1000) 

extern unsigned int get_tamper_sf(void);

#define MMC_BKOPS_MAX_TIMEOUT	(30 * 1000) 
#define MMC_DETECT_RETRIES	5

#define MMC_WORKLOAD_DURATION (60 * 60 * 1000) 

static struct workqueue_struct *workqueue;
struct workqueue_struct *stats_workqueue = NULL;
static const unsigned freqs[] = { 400000, 300000, 200000, 100000 };

bool use_spi_crc = 1;
module_param(use_spi_crc, bool, 0);

#define IOTOP_INTERVAL			4500
#define IOREAD_DUMP_THRESHOLD		10485760 
#define IOWRITE_DUMP_THRESHOLD		1048576  
#define IOREAD_DUMP_TOTAL_THRESHOLD	10485760 
#define IOWRITE_DUMP_TOTAL_THRESHOLD	10485760 
struct io_account {
	char task_name[TASK_COMM_LEN];
	char gtask_name[TASK_COMM_LEN]; 
	char ptask_name[TASK_COMM_LEN]; 
	unsigned int pid;
	unsigned int tgid;
	unsigned int ppid;
	u64 io_amount;
	struct list_head list;
};
static LIST_HEAD(ioread_list);
static LIST_HEAD(iowrite_list);
static spinlock_t iolist_lock;
static unsigned long jiffies_next_iotop = 0;
static int iotop_cmp(void *priv, struct list_head *a, struct list_head *b)
{
	struct io_account *ioa = container_of(a, struct io_account, list);
	struct io_account *iob = container_of(b, struct io_account, list);

	return !(ioa->io_amount > iob->io_amount);
}

void collect_io_stats(size_t rw_bytes, int type)
{
	struct task_struct *process = current;
	struct io_account *io_act, *tmp;
	int found;
	struct list_head *io_list;
	unsigned long flags;

	if (get_tamper_sf() == 1)
		return;

	if (!rw_bytes)
		return;

	if (!strcmp(current->comm, "sdcard"))
		return;

	if (type == READ)
		io_list = &ioread_list;
	else if (type == WRITE)
		io_list = &iowrite_list;
	else
		return;

	found = 0;
	spin_lock_irqsave(&iolist_lock, flags);
	list_for_each_entry_safe(io_act, tmp, io_list, list) {
		if ((process->pid == io_act->pid) && !strcmp(process->comm, io_act->task_name)) {
			io_act->io_amount += rw_bytes;
			found = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&iolist_lock, flags);

	if (!found) {
		io_act = kmalloc(sizeof(struct io_account), GFP_ATOMIC);
		if (io_act) {
			snprintf(io_act->task_name, sizeof(io_act->task_name), "%s", process->comm);
			io_act->pid = process->pid;
			io_act->tgid = process->tgid;
			if (process->group_leader)
				snprintf(io_act->gtask_name, sizeof(io_act->gtask_name), "%s",
					process->group_leader->comm);
			if (process->parent) {
				snprintf(io_act->ptask_name, sizeof(io_act->ptask_name), "%s",
					process->parent->comm);
				io_act->ppid = process->parent->pid;
			}
			io_act->io_amount = rw_bytes;
			spin_lock_irqsave(&iolist_lock, flags);
			list_add(&io_act->list, io_list);
			spin_unlock_irqrestore(&iolist_lock, flags);
		}
	}
}

static void show_iotop(void)
{
	struct io_account *io_act, *tmp;
	int i = 0;
	unsigned int task_cnt = 0;
	unsigned long long total_bytes;
	unsigned long flags;

	if (get_tamper_sf() == 1)
		return;

	spin_lock_irqsave(&iolist_lock, flags);
	list_sort(NULL, &ioread_list, iotop_cmp);
	total_bytes = 0;
	list_for_each_entry_safe(io_act, tmp, &ioread_list, list) {
		list_del_init(&io_act->list);
		if (i++ < 5 && io_act->io_amount > IOREAD_DUMP_THRESHOLD)
			pr_info("[READ IOTOP%d] %s(pid %u, tgid %u(%s), ppid %u(%s)): %llu KB\n",
				i, io_act->task_name, io_act->pid, io_act->tgid, io_act->gtask_name,
				io_act->ppid, io_act->ptask_name, io_act->io_amount / 1024);
		task_cnt++;
		total_bytes += io_act->io_amount;
		kfree(io_act);
	}
	if (total_bytes > IOREAD_DUMP_TOTAL_THRESHOLD)
		pr_info("[IOTOP] READ total %u tasks, %llu KB\n", task_cnt, total_bytes / 1024);

	list_sort(NULL, &iowrite_list, iotop_cmp);
	i = 0;
	total_bytes = 0;
	task_cnt = 0;
	list_for_each_entry_safe(io_act, tmp, &iowrite_list, list) {
		list_del_init(&io_act->list);
		if (i++ < 5 && io_act->io_amount >= IOWRITE_DUMP_THRESHOLD)
			pr_info("[WRITE IOTOP%d] %s(pid %u, tgid %u(%s), ppid %u(%s)): %llu KB\n",
				i, io_act->task_name, io_act->pid, io_act->tgid, io_act->gtask_name,
				io_act->ppid, io_act->ptask_name, io_act->io_amount / 1024);
		task_cnt++;
		total_bytes += io_act->io_amount;
		kfree(io_act);
	}
	spin_unlock_irqrestore(&iolist_lock, flags);
	if (total_bytes > IOWRITE_DUMP_TOTAL_THRESHOLD)
		pr_info("[IOTOP] WRITE total %u tasks, %llu KB\n", task_cnt, total_bytes / 1024);
}

static int stats_interval = MMC_STATS_INTERVAL;
#define K(x) ((x) << (PAGE_SHIFT - 10))
void mmc_stats(struct work_struct *work)
{
	struct mmc_host *host =
		container_of(work, struct mmc_host, stats_work.work);
	unsigned long rtime, wtime;
	unsigned long rbytes, wbytes, rcnt, wcnt;
	unsigned long wperf = 0, rperf = 0;
	unsigned long flags;
	u64 val;
	struct kstatfs stat;
	unsigned long free = 0;
	int reset_low_perf_data = 1;
	
	unsigned long rtime_rand = 0, wtime_rand = 0;
	unsigned long rbytes_rand = 0, wbytes_rand = 0;
	unsigned long rcnt_rand = 0, wcnt_rand = 0;
	unsigned long wperf_rand = 0, rperf_rand = 0;
	
	unsigned long erase_time; 
	unsigned long erase_blks;
	unsigned long erase_rq;
	
	ktime_t workload_diff;
	unsigned long workload_time; 

	if (!host || !host->perf_enable || !stats_workqueue)
		return;

	spin_lock_irqsave(&host->lock, flags);

	rbytes = host->perf.rbytes_drv;
	wbytes = host->perf.wbytes_drv;
	rcnt = host->perf.rcount;
	wcnt = host->perf.wcount;
	rtime = (unsigned long)ktime_to_us(host->perf.rtime_drv);
	wtime = (unsigned long)ktime_to_us(host->perf.wtime_drv);

	host->perf.rbytes_drv = host->perf.wbytes_drv = 0;
	host->perf.rcount = host->perf.wcount = 0;
	host->perf.rtime_drv = ktime_set(0, 0);
	host->perf.wtime_drv = ktime_set(0, 0);

	
	erase_time = (unsigned long)ktime_to_ms(host->perf.erase_time);
	erase_blks = host->perf.erase_blks;
	erase_rq = host->perf.erase_rq;
	host->perf.erase_blks = 0;
	host->perf.erase_rq = 0;
	host->perf.erase_time = ktime_set(0, 0);

	
	if (host->debug_mask & MMC_DEBUG_RANDOM_RW) {
		rbytes_rand = host->perf.rbytes_drv_rand;
		wbytes_rand = host->perf.wbytes_drv_rand;
		rcnt_rand = host->perf.rcount_rand;
		wcnt_rand = host->perf.wcount_rand;
		rtime_rand = (unsigned long)ktime_to_us(host->perf.rtime_drv_rand);
		wtime_rand = (unsigned long)ktime_to_us(host->perf.wtime_drv_rand);

		host->perf.rbytes_drv_rand = host->perf.wbytes_drv_rand = 0;
		host->perf.rcount_rand = host->perf.wcount_rand = 0;
		host->perf.rtime_drv_rand = ktime_set(0, 0);
		host->perf.wtime_drv_rand = ktime_set(0, 0);
	}
	

	spin_unlock_irqrestore(&host->lock, flags);

	if (wtime) {
		val = ((u64)wbytes / 1024) * 1000000;
		do_div(val, wtime);
		wperf = (unsigned long)val;
	}
	if (rtime) {
		val = ((u64)rbytes / 1024) * 1000000;
		do_div(val, rtime);
		rperf = (unsigned long)val;
	}

	if (host->debug_mask & MMC_DEBUG_FREE_SPACE) {
		struct file *file;
		file = filp_open("/data", O_RDONLY, 0);
		if (!IS_ERR(file)) {
			vfs_statfs(&file->f_path, &stat);
			filp_close(file, NULL);
			free = (unsigned long)stat.f_bfree;
			free /= 256; 
		}
	}

	
	wtime /= 1000;
	rtime /= 1000;

	
	if (!wtime)
		reset_low_perf_data = 0;
	else if (wperf < 100) {
		host->perf.lp_duration += stats_interval; 
		host->perf.wbytes_low_perf += wbytes;
		host->perf.wtime_low_perf += wtime; 
		if (host->perf.lp_duration >= 600000) {
			unsigned long perf;
			val = ((u64)host->perf.wbytes_low_perf / 1024) * 1000;
			do_div(val, host->perf.wtime_low_perf);
			perf = (unsigned long)val;
			pr_err("%s Statistics: write %lu KB in %lu ms, perf %lu KB/s, duration %lu sec\n",
				mmc_hostname(host), host->perf.wbytes_low_perf / 1024,
				host->perf.wtime_low_perf,
				perf, host->perf.lp_duration / 1000);
		} else
			reset_low_perf_data = 0;
	}
	if (host->perf.lp_duration && reset_low_perf_data) {
		host->perf.lp_duration = 0;
		host->perf.wbytes_low_perf = 0;
		host->perf.wtime_low_perf = 0;
	}

	
	if ((wtime > 500) || (wtime && (stats_interval == MMC_STATS_LOG_INTERVAL))) {
		#if 0
		pr_info("%s Statistics: dirty %luKB, writeback %luKB\n",
			mmc_hostname(host),
			K(global_page_state(NR_FILE_DIRTY)),
			K(global_page_state(NR_WRITEBACK)));
		#endif
		if (host->debug_mask & MMC_DEBUG_FREE_SPACE)
			pr_info("%s Statistics: write %lu KB in %lu ms, perf %lu KB/s, rq %lu, /data free %lu MB\n",
				mmc_hostname(host), wbytes / 1024, wtime, wperf, wcnt, free);
		else
			pr_info("%s Statistics: write %lu KB in %lu ms, perf %lu KB/s, rq %lu\n",
				mmc_hostname(host), wbytes / 1024, wtime, wperf, wcnt);

		if (rtime) {
			if (host->debug_mask & MMC_DEBUG_FREE_SPACE)
				pr_info("%s Statistics: read %lu KB in %lu ms, perf %lu KB/s, rq %lu, /data free %lu MB\n",
					mmc_hostname(host), rbytes / 1024, rtime, rperf, rcnt, free);
			else
				pr_info("%s Statistics: read %lu KB in %lu ms, perf %lu KB/s, rq %lu\n",
					mmc_hostname(host), rbytes / 1024, rtime, rperf, rcnt);
		}
	}
	else if ((rtime > 500) || (rtime && (stats_interval == MMC_STATS_LOG_INTERVAL))) {
		if (host->debug_mask & MMC_DEBUG_FREE_SPACE)
			pr_info("%s Statistics: read %lu KB in %lu ms, perf %lu KB/s, rq %lu, /data free %lu MB\n",
				mmc_hostname(host), rbytes / 1024, rtime, rperf, rcnt, free);
		else
			pr_info("%s Statistics: read %lu KB in %lu ms, perf %lu KB/s, rq %lu\n",
				mmc_hostname(host), rbytes / 1024, rtime, rperf, rcnt);
	}

	
	workload_diff = ktime_sub(ktime_get(), host->perf.workload_time);
	workload_time = (unsigned long)ktime_to_ms(workload_diff);
	if (workload_time >= MMC_WORKLOAD_DURATION) {
		pr_info("%s Statistics: workload write %lu KB (%lu MB)\n",
			mmc_hostname(host),
			host->perf.wkbytes_drv, host->perf.wkbytes_drv / 1024);
		host->perf.wkbytes_drv = 0;
		host->perf.workload_time = ktime_get();
	}

	
	if (erase_time > 500)
		pr_info("%s Statistics: erase %lu blocks in %lu ms, rq %lu\n",
			mmc_hostname(host), erase_blks, erase_time, erase_rq);

	if (!jiffies_next_iotop || time_after(jiffies, jiffies_next_iotop)) {
		jiffies_next_iotop = jiffies + msecs_to_jiffies(IOTOP_INTERVAL);
		show_iotop();
	}

	
	if (host->debug_mask & MMC_DEBUG_RANDOM_RW) {
		if (wtime_rand) {
			val = ((u64)wbytes_rand / 1024) * 1000000;
			do_div(val, wtime_rand);
			wperf_rand = (unsigned long)val;
		}
		if (rtime_rand) {
			val = ((u64)rbytes_rand / 1024) * 1000000;
			do_div(val, rtime_rand);
			rperf_rand = (unsigned long)val;
		}
		wtime_rand /= 1000;
		rtime_rand /= 1000;
		if (wperf_rand && wtime_rand) {
			if (host->debug_mask & MMC_DEBUG_FREE_SPACE)
				pr_info("%s Statistics: random write %lu KB in %lu ms, perf %lu KB/s, rq %lu, /data free %lu MB\n",
					mmc_hostname(host), wbytes_rand / 1024, wtime_rand, wperf_rand, wcnt_rand, free);
			else
				pr_info("%s Statistics: random write %lu KB in %lu ms, perf %lu KB/s, rq %lu\n",
					mmc_hostname(host), wbytes_rand / 1024, wtime_rand, wperf_rand, wcnt_rand);
		}
		if (rperf_rand && rtime_rand) {
			if (host->debug_mask & MMC_DEBUG_FREE_SPACE)
				pr_info("%s Statistics: random read %lu KB in %lu ms, perf %lu KB/s, rq %lu, /data free %lu MB\n",
					mmc_hostname(host), rbytes_rand / 1024, rtime_rand, rperf_rand, rcnt_rand, free);
			else
				pr_info("%s Statistics: random read %lu KB in %lu ms, perf %lu KB/s, rq %lu\n",
					mmc_hostname(host), rbytes_rand / 1024, rtime_rand, rperf_rand, rcnt_rand);
		}
	}
	

	if (host->debug_mask & MMC_DEBUG_MEMORY) {
		struct sysinfo mi;
		long cached;
		si_meminfo(&mi);
		cached = global_page_state(NR_FILE_PAGES) -
			total_swapcache_pages();
		pr_info("meminfo: total %lu KB, free %lu KB, buffers %lu KB, cached %lu KB \n",
			K(mi.totalram),
			K(mi.freeram),
			K(mi.bufferram),
			K(cached));
	}

	queue_delayed_work(stats_workqueue, &host->stats_work,
		msecs_to_jiffies(stats_interval));
	return;
}

static int mmc_schedule_delayed_work(struct delayed_work *work,
				     unsigned long delay)
{
	return queue_delayed_work(workqueue, work, delay);
}

static void mmc_flush_scheduled_work(void)
{
	flush_workqueue(workqueue);
}

#ifdef CONFIG_FAIL_MMC_REQUEST

static void mmc_should_fail_request(struct mmc_host *host,
				    struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	static const int data_errors[] = {
		-ETIMEDOUT,
		-EILSEQ,
		-EIO,
	};

	if (!data)
		return;

	if (cmd->error || data->error ||
	    !should_fail(&host->fail_mmc_request, data->blksz * data->blocks))
		return;

	data->error = data_errors[prandom_u32() % ARRAY_SIZE(data_errors)];
	data->bytes_xfered = (prandom_u32() % (data->bytes_xfered >> 9)) << 9;
	data->fault_injected = true;
}

#else 

static inline void mmc_should_fail_request(struct mmc_host *host,
					   struct mmc_request *mrq)
{
}

#endif 

static bool mmc_is_data_request(struct mmc_request *mmc_request)
{
	switch (mmc_request->cmd->opcode) {
	case MMC_READ_SINGLE_BLOCK:
	case MMC_READ_MULTIPLE_BLOCK:
	case MMC_WRITE_BLOCK:
	case MMC_WRITE_MULTIPLE_BLOCK:
		return true;
	default:
		return false;
	}
}

static void mmc_clk_scaling_start_busy(struct mmc_host *host, bool lock_needed)
{
	struct mmc_devfeq_clk_scaling *clk_scaling = &host->clk_scaling;

	if (!clk_scaling->enable)
		return;

	if (lock_needed)
		spin_lock_bh(&clk_scaling->lock);

	clk_scaling->start_busy = ktime_get();
	clk_scaling->is_busy_started = true;

	if (lock_needed)
		spin_unlock_bh(&clk_scaling->lock);
}

static void mmc_clk_scaling_stop_busy(struct mmc_host *host, bool lock_needed)
{
	struct mmc_devfeq_clk_scaling *clk_scaling = &host->clk_scaling;

	if (!clk_scaling->enable)
		return;

	if (lock_needed)
		spin_lock_bh(&clk_scaling->lock);

	if (!clk_scaling->is_busy_started) {
		WARN_ON(1);
		goto out;
	}

	clk_scaling->total_busy_time_us +=
		ktime_to_us(ktime_sub(ktime_get(),
			clk_scaling->start_busy));
	pr_debug("%s: accumulated busy time is %lu usec\n",
		mmc_hostname(host), clk_scaling->total_busy_time_us);
	clk_scaling->is_busy_started = false;

out:
	if (lock_needed)
		spin_unlock_bh(&clk_scaling->lock);
}

void mmc_cmdq_clk_scaling_start_busy(struct mmc_host *host,
	bool lock_needed)
{
	if (!host->clk_scaling.enable)
		return;

	if (lock_needed)
		spin_lock_bh(&host->clk_scaling.lock);

	if (!host->clk_scaling.is_busy_started &&
		!test_bit(CMDQ_STATE_DCMD_ACTIVE,
			&host->cmdq_ctx.curr_state)) {
		host->clk_scaling.start_busy = ktime_get();
		host->clk_scaling.is_busy_started = true;
	}

	if (lock_needed)
		spin_unlock_bh(&host->clk_scaling.lock);
}
EXPORT_SYMBOL(mmc_cmdq_clk_scaling_start_busy);

void mmc_cmdq_clk_scaling_stop_busy(struct mmc_host *host,
	bool lock_needed, bool is_cmdq_dcmd)
{
	if (!host->clk_scaling.enable)
		return;

	if (lock_needed)
		spin_lock_bh(&host->clk_scaling.lock);

	if (is_cmdq_dcmd) {
		if (host->cmdq_ctx.data_active_reqs) {
			host->clk_scaling.is_busy_started = true;
			host->clk_scaling.start_busy = ktime_get();
		}
		goto out;
	}

	host->clk_scaling.total_busy_time_us +=
		ktime_to_us(ktime_sub(ktime_get(),
			host->clk_scaling.start_busy));

	if (host->cmdq_ctx.data_active_reqs) {
		host->clk_scaling.is_busy_started = true;
		host->clk_scaling.start_busy = ktime_get();
	} else {
		host->clk_scaling.is_busy_started = false;
	}
out:
	if (lock_needed)
		spin_unlock_bh(&host->clk_scaling.lock);

}
EXPORT_SYMBOL(mmc_cmdq_clk_scaling_stop_busy);

bool mmc_can_scale_clk(struct mmc_host *host)
{
	if (!host) {
		pr_err("bad host parameter\n");
		WARN_ON(1);
		return false;
	}

	return host->caps2 & MMC_CAP2_CLK_SCALE;
}
EXPORT_SYMBOL(mmc_can_scale_clk);

static int mmc_devfreq_get_dev_status(struct device *dev,
		struct devfreq_dev_status *status)
{
	struct mmc_host *host = container_of(dev, struct mmc_host, class_dev);
	struct mmc_devfeq_clk_scaling *clk_scaling;

	if (!host) {
		pr_err("bad host parameter\n");
		WARN_ON(1);
		return -EINVAL;
	}

	clk_scaling = &host->clk_scaling;

	if (!clk_scaling->enable)
		return 0;

	spin_lock_bh(&clk_scaling->lock);

	
	memset(status, 0, sizeof(*status));
	if (clk_scaling->is_busy_started) {
		if (mmc_card_cmdq(host->card)) {
			mmc_cmdq_clk_scaling_stop_busy(host, false, false);
		} else {
			mmc_clk_scaling_stop_busy(host, false);
			mmc_clk_scaling_start_busy(host, false);
		}
	}

	status->busy_time = clk_scaling->total_busy_time_us;
	status->total_time = ktime_to_us(ktime_sub(ktime_get(),
		clk_scaling->measure_interval_start));
	clk_scaling->total_busy_time_us = 0;
	status->current_frequency = clk_scaling->curr_freq;
	clk_scaling->measure_interval_start = ktime_get();

	pr_debug("%s: status: load = %lu%% - total_time=%lu busy_time = %lu, clk=%lu\n",
		mmc_hostname(host),
		(status->busy_time*100)/status->total_time,
		status->total_time, status->busy_time,
		status->current_frequency);

	spin_unlock_bh(&clk_scaling->lock);

	return 0;
}

static bool mmc_is_valid_state_for_clk_scaling(struct mmc_host *host)
{
	struct mmc_card *card = host->card;
	u32 status;

	if (!card || (mmc_card_mmc(card) &&
			(card->part_curr == EXT_CSD_PART_CONFIG_ACC_RPMB ||
			mmc_card_doing_bkops(card))))
		return false;

	if (mmc_send_status(card, &status)) {
		pr_err("%s: Get card status fail\n", mmc_hostname(card->host));
		return false;
	}

	return R1_CURRENT_STATE(status) == R1_STATE_TRAN;
}

int mmc_cmdq_halt_on_empty_queue(struct mmc_host *host)
{
	int err = 0;

	err = wait_event_interruptible(host->cmdq_ctx.queue_empty_wq,
				(!host->cmdq_ctx.active_reqs));
	if (host->cmdq_ctx.active_reqs) {
		pr_err("%s: %s: unexpected active requests (%lu)\n",
			mmc_hostname(host), __func__,
			host->cmdq_ctx.active_reqs);
		return -EPERM;
	}

	err = mmc_cmdq_halt(host, true);
	if (err) {
		pr_err("%s: %s: mmc_cmdq_halt failed (%d)\n",
		       mmc_hostname(host), __func__, err);
		goto out;
	}

out:
	return err;
}
EXPORT_SYMBOL(mmc_cmdq_halt_on_empty_queue);

int mmc_clk_update_freq(struct mmc_host *host,
		unsigned long freq, enum mmc_load state)
{
	int err = 0;
	bool cmdq_mode;

	if (!host) {
		pr_err("bad host parameter\n");
		WARN_ON(1);
		return -EINVAL;
	}

	mmc_host_clk_hold(host);
	cmdq_mode = mmc_card_cmdq(host->card);

	
	if (unlikely(freq > host->card->clk_scaling_highest)) {
		freq = host->card->clk_scaling_highest;
		pr_warn("%s: %s: frequency was overridden to %lu\n",
				mmc_hostname(host), __func__,
				host->card->clk_scaling_highest);
	}

	if (unlikely(freq < host->card->clk_scaling_lowest)) {
		freq = host->card->clk_scaling_lowest;
		pr_warn("%s: %s: frequency was overridden to %lu\n",
			mmc_hostname(host), __func__,
			host->card->clk_scaling_lowest);
	}

	if (freq == host->clk_scaling.curr_freq)
		goto out;

	if (host->ops->notify_load) {
		err = host->ops->notify_load(host, state);
		if (err) {
			pr_err("%s: %s: fail on notify_load\n",
				mmc_hostname(host), __func__);
			goto out;
		}
	}

	if (cmdq_mode) {
		err = mmc_cmdq_halt_on_empty_queue(host);
		if (err) {
			pr_err("%s: %s: failed halting queue (%d)\n",
				mmc_hostname(host), __func__, err);
			goto halt_failed;
		}
	}

	if (!mmc_is_valid_state_for_clk_scaling(host)) {
		pr_debug("%s: invalid state for clock scaling - skipping",
			mmc_hostname(host));
		goto invalid_state;
	}

	err = host->bus_ops->change_bus_speed(host, &freq);
	if (!err)
		host->clk_scaling.curr_freq = freq;
	else
		pr_err("%s: %s: failed (%d) at freq=%lu\n",
			mmc_hostname(host), __func__, err, freq);

invalid_state:
	if (cmdq_mode) {
		if (mmc_cmdq_halt(host, false))
			pr_err("%s: %s: cmdq unhalt failed\n",
			mmc_hostname(host), __func__);
	}

halt_failed:
	if (err) {
		
		if (host->ops->notify_load)
			if (host->ops->notify_load(host,
				host->clk_scaling.state))
				pr_err("%s: %s: fail on notify_load restore\n",
					mmc_hostname(host), __func__);
	}
out:
	mmc_host_clk_release(host);
	return err;
}
EXPORT_SYMBOL(mmc_clk_update_freq);

static int mmc_devfreq_set_target(struct device *dev,
				unsigned long *freq, u32 devfreq_flags)
{
	struct mmc_host *host = container_of(dev, struct mmc_host, class_dev);
	struct mmc_devfeq_clk_scaling *clk_scaling;
	int err = 0;
	int abort;

	if (!(host && freq)) {
		pr_err("%s: unexpected host/freq parameter\n", __func__);
		err = -EINVAL;
		goto out;
	}
	clk_scaling = &host->clk_scaling;

	if (!clk_scaling->enable)
		goto out;

	pr_debug("%s: target freq = %lu (%s)\n", mmc_hostname(host),
		*freq, current->comm);

	if ((clk_scaling->curr_freq == *freq) ||
		clk_scaling->skip_clk_scale_freq_update)
		goto out;

	
	if (!host->ios.clock)
		goto out;

	spin_lock_bh(&clk_scaling->lock);
	if (clk_scaling->clk_scaling_in_progress) {
		pr_debug("%s: clocks scaling is already in-progress by mmc thread\n",
			mmc_hostname(host));
		spin_unlock_bh(&clk_scaling->lock);
		goto out;
	}
	clk_scaling->need_freq_change = true;
	clk_scaling->target_freq = *freq;
	clk_scaling->state = *freq < clk_scaling->curr_freq ?
		MMC_LOAD_LOW : MMC_LOAD_HIGH;
	spin_unlock_bh(&clk_scaling->lock);

	abort = __mmc_claim_host(host, &clk_scaling->devfreq_abort);
	if (abort)
		goto out;

	clk_scaling->need_freq_change = false;

	mmc_host_clk_hold(host);
	err = mmc_clk_update_freq(host, *freq, clk_scaling->state);
	if (err && err != -EAGAIN)
		pr_err("%s: clock scale to %lu failed with error %d\n",
			mmc_hostname(host), *freq, err);
	else
		pr_debug("%s: clock change to %lu finished successfully (%s)\n",
			mmc_hostname(host), *freq, current->comm);


	mmc_host_clk_release(host);
	mmc_release_host(host);
out:
	return err;
}

void mmc_deferred_scaling(struct mmc_host *host)
{
	unsigned long target_freq;
	int err;

	if (!host->clk_scaling.enable)
		return;

	spin_lock_bh(&host->clk_scaling.lock);

	if (host->clk_scaling.clk_scaling_in_progress ||
		!(host->clk_scaling.need_freq_change)) {
		spin_unlock_bh(&host->clk_scaling.lock);
		return;
	}


	atomic_inc(&host->clk_scaling.devfreq_abort);
	target_freq = host->clk_scaling.target_freq;
	host->clk_scaling.clk_scaling_in_progress = true;
	host->clk_scaling.need_freq_change = false;
	spin_unlock_bh(&host->clk_scaling.lock);
	pr_debug("%s: doing deferred frequency change (%lu) (%s)\n",
				mmc_hostname(host),
				target_freq, current->comm);

	err = mmc_clk_update_freq(host, target_freq,
		host->clk_scaling.state);
	if (err && err != -EAGAIN)
		pr_err("%s: failed on deferred scale clocks (%d)\n",
			mmc_hostname(host), err);
	else
		pr_debug("%s: clocks were successfully scaled to %lu (%s)\n",
			mmc_hostname(host),
			target_freq, current->comm);
	host->clk_scaling.clk_scaling_in_progress = false;
	atomic_dec(&host->clk_scaling.devfreq_abort);
}
EXPORT_SYMBOL(mmc_deferred_scaling);

static int mmc_devfreq_create_freq_table(struct mmc_host *host)
{
	int i;
	struct mmc_devfeq_clk_scaling *clk_scaling = &host->clk_scaling;

	pr_debug("%s: supported: lowest=%lu, highest=%lu\n",
		mmc_hostname(host),
		host->card->clk_scaling_lowest,
		host->card->clk_scaling_highest);

	if (!clk_scaling->freq_table) {
		pr_debug("%s: no frequency table defined -  setting default\n",
			mmc_hostname(host));
		clk_scaling->freq_table = kzalloc(
			2*sizeof(*(clk_scaling->freq_table)), GFP_KERNEL);
		if (!clk_scaling->freq_table)
			return -ENOMEM;
		clk_scaling->freq_table[0] = host->card->clk_scaling_lowest;
		clk_scaling->freq_table[1] = host->card->clk_scaling_highest;
		clk_scaling->freq_table_sz = 2;
		goto out;
	}

	if (host->card->clk_scaling_lowest >
		clk_scaling->freq_table[0])
		pr_debug("%s: frequency table undershot possible freq\n",
			mmc_hostname(host));

	for (i = 0; i < clk_scaling->freq_table_sz; i++) {
		if (clk_scaling->freq_table[i] <=
			host->card->clk_scaling_highest)
			continue;
		clk_scaling->freq_table[i] =
			host->card->clk_scaling_highest;
		clk_scaling->freq_table_sz = i + 1;
		pr_debug("%s: frequency table overshot possible freq (%d)\n",
				mmc_hostname(host), clk_scaling->freq_table[i]);
		break;
	}

out:
	clk_scaling->devfreq_profile.freq_table = clk_scaling->freq_table;
	clk_scaling->devfreq_profile.max_state = clk_scaling->freq_table_sz;

	for (i = 0; i < clk_scaling->freq_table_sz; i++)
		pr_debug("%s: freq[%d] = %u\n",
			mmc_hostname(host), i, clk_scaling->freq_table[i]);

	return 0;
}

int mmc_init_clk_scaling(struct mmc_host *host)
{
	int err;

	if (!host || !host->card) {
		pr_err("%s: unexpected host/card parameters\n",
			__func__);
		return -EINVAL;
	}

	if (!mmc_can_scale_clk(host) ||
		!host->bus_ops->change_bus_speed) {
		pr_debug("%s: clock scaling is not supported\n",
			mmc_hostname(host));
		return 0;
	}

	pr_debug("registering %s dev (%p) to devfreq",
		mmc_hostname(host),
		mmc_classdev(host));

	if (host->clk_scaling.devfreq) {
		pr_err("%s: dev is already registered for dev %p\n",
			mmc_hostname(host),
			mmc_dev(host));
		return -EPERM;
	}
	spin_lock_init(&host->clk_scaling.lock);
	atomic_set(&host->clk_scaling.devfreq_abort, 0);
	host->clk_scaling.curr_freq = host->ios.clock;
	host->clk_scaling.clk_scaling_in_progress = false;
	host->clk_scaling.need_freq_change = false;
	host->clk_scaling.is_busy_started = false;

	host->clk_scaling.devfreq_profile.polling_ms =
		host->clk_scaling.polling_delay_ms;
	host->clk_scaling.devfreq_profile.get_dev_status =
		mmc_devfreq_get_dev_status;
	host->clk_scaling.devfreq_profile.target = mmc_devfreq_set_target;
	host->clk_scaling.devfreq_profile.initial_freq = host->ios.clock;

	host->clk_scaling.ondemand_gov_data.simple_scaling = true;
	host->clk_scaling.ondemand_gov_data.upthreshold =
		host->clk_scaling.upthreshold;
	host->clk_scaling.ondemand_gov_data.downdifferential =
		host->clk_scaling.upthreshold - host->clk_scaling.downthreshold;

	err = mmc_devfreq_create_freq_table(host);
	if (err) {
		pr_err("%s: fail to create devfreq frequency table\n",
			mmc_hostname(host));
		return err;
	}

	pr_debug("%s: adding devfreq with: upthreshold=%u downthreshold=%u polling=%u\n",
		mmc_hostname(host),
		host->clk_scaling.ondemand_gov_data.upthreshold,
		host->clk_scaling.ondemand_gov_data.downdifferential,
		host->clk_scaling.devfreq_profile.polling_ms);
	host->clk_scaling.devfreq = devfreq_add_device(
		mmc_classdev(host),
		&host->clk_scaling.devfreq_profile,
		"simple_ondemand",
		&host->clk_scaling.ondemand_gov_data);
	if (!host->clk_scaling.devfreq) {
		pr_err("%s: unable to register with devfreq\n",
			mmc_hostname(host));
		return -EPERM;
	}

	pr_debug("%s: clk scaling is enabled for device %s (%p) with devfreq %p (clock = %uHz)\n",
		mmc_hostname(host),
		dev_name(mmc_classdev(host)),
		mmc_classdev(host),
		host->clk_scaling.devfreq,
		host->ios.clock);

	host->clk_scaling.enable = true;

	return err;
}
EXPORT_SYMBOL(mmc_init_clk_scaling);

int mmc_suspend_clk_scaling(struct mmc_host *host)
{
	int err = 0;

	if (!host) {
		WARN(1, "bad host parameter\n");
		return -EINVAL;
	}

	if (!mmc_can_scale_clk(host) || !host->clk_scaling.enable)
		return 0;

	if (!host->clk_scaling.devfreq) {
		pr_err("%s: %s: no devfreq is assosiated with this device\n",
			mmc_hostname(host), __func__);
		return -EPERM;
	}

	atomic_inc(&host->clk_scaling.devfreq_abort);
	wake_up(&host->wq);
	err = devfreq_suspend_device(host->clk_scaling.devfreq);
	if (err) {
		pr_err("%s: %s: failed to suspend devfreq\n",
			mmc_hostname(host), __func__);
		return err;
	}
	host->clk_scaling.enable = false;

	host->clk_scaling.total_busy_time_us = 0;

	pr_debug("%s: devfreq suspended\n", mmc_hostname(host));

	return 0;
}
EXPORT_SYMBOL(mmc_suspend_clk_scaling);

int mmc_resume_clk_scaling(struct mmc_host *host)
{
	int err = 0;
	u32 max_clk_idx = 0;
	u32 devfreq_max_clk = 0;
	u32 devfreq_min_clk = 0;

	if (!host) {
		WARN(1, "bad host parameter\n");
		return -EINVAL;
	}

	if (!mmc_can_scale_clk(host))
		return 0;

	if (!host->clk_scaling.devfreq) {
		pr_err("%s: %s: no devfreq is assosiated with this device\n",
			mmc_hostname(host), __func__);
		return -EPERM;
	}

	atomic_set(&host->clk_scaling.devfreq_abort, 0);

	max_clk_idx = host->clk_scaling.freq_table_sz - 1;
	devfreq_max_clk = host->clk_scaling.freq_table[max_clk_idx];
	devfreq_min_clk = host->clk_scaling.freq_table[0];

	host->clk_scaling.curr_freq = devfreq_max_clk;
	if (host->ios.clock < host->card->clk_scaling_highest)
		host->clk_scaling.curr_freq = devfreq_min_clk;

	host->clk_scaling.clk_scaling_in_progress = false;
	host->clk_scaling.need_freq_change = false;

	err = devfreq_resume_device(host->clk_scaling.devfreq);
	if (err) {
		pr_err("%s: %s: failed to resume devfreq (%d)\n",
			mmc_hostname(host), __func__, err);
	} else {
		host->clk_scaling.enable = true;
		pr_debug("%s: devfreq resumed\n", mmc_hostname(host));
	}

	return err;
}
EXPORT_SYMBOL(mmc_resume_clk_scaling);

int mmc_exit_clk_scaling(struct mmc_host *host)
{
	int err;

	if (!host) {
		pr_err("%s: bad host parameter\n", __func__);
		WARN_ON(1);
		return -EINVAL;
	}

	if (!mmc_can_scale_clk(host))
		return 0;

	if (!host->clk_scaling.devfreq) {
		pr_err("%s: %s: no devfreq is assosiated with this device\n",
			mmc_hostname(host), __func__);
		return -EPERM;
	}

	err = mmc_suspend_clk_scaling(host);
	if (err) {
		pr_err("%s: %s: fail to suspend clock scaling (%d)\n",
			mmc_hostname(host), __func__,  err);
		return err;
	}

	err = devfreq_remove_device(host->clk_scaling.devfreq);
	if (err) {
		pr_err("%s: remove devfreq failed (%d)\n",
			mmc_hostname(host), err);
		return err;
	}

	host->clk_scaling.devfreq = NULL;
	atomic_set(&host->clk_scaling.devfreq_abort, 1);
	pr_debug("%s: devfreq was removed\n", mmc_hostname(host));

	return 0;
}
EXPORT_SYMBOL(mmc_exit_clk_scaling);

void mmc_request_done(struct mmc_host *host, struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	int err = cmd->error;
	ktime_t diff;

	if (host->clk_scaling.is_busy_started)
		mmc_clk_scaling_stop_busy(host, true);

	if (err && cmd->retries && mmc_host_is_spi(host)) {
		if (cmd->resp[0] & R1_SPI_ILLEGAL_COMMAND)
			cmd->retries = 0;
	}

	if (err && cmd->retries && !mmc_card_removed(host->card)) {
		if (mrq->done)
			mrq->done(mrq);
	} else {
		mmc_should_fail_request(host, mrq);

		pr_debug("%s: req done (CMD%u): %d: %08x %08x %08x %08x\n",
			mmc_hostname(host), cmd->opcode, err,
			cmd->resp[0], cmd->resp[1],
			cmd->resp[2], cmd->resp[3]);

		if (mrq->data) {
			if (host->perf_enable && cmd->opcode != MMC_SEND_EXT_CSD) {
				unsigned long flags;
				diff = ktime_sub(ktime_get(), host->perf.start);
				if (host->card && mmc_card_mmc(host->card))
					trace_mmc_request_done(&host->class_dev,
						cmd->opcode, mrq->cmd->arg,
						mrq->data->blocks, ktime_to_ms(diff), 0);
				spin_lock_irqsave(&host->lock, flags);
				if (mrq->data->flags == MMC_DATA_READ) {
					host->perf.rbytes_drv +=
							mrq->data->bytes_xfered;
					host->perf.rtime_drv =
						ktime_add(host->perf.rtime_drv,
							diff);
					host->perf.rcount++;
					if (host->debug_mask & MMC_DEBUG_RANDOM_RW) {
						if (mrq->data->bytes_xfered <= 32*1024) {
							host->perf.rbytes_drv_rand +=
								mrq->data->bytes_xfered;
							host->perf.rtime_drv_rand =
								ktime_add(host->perf.rtime_drv_rand,
									diff);
							host->perf.rcount_rand++;
						}
					}
				} else {
					host->perf.wbytes_drv +=
						mrq->data->bytes_xfered;
					host->perf.wkbytes_drv +=
						(mrq->data->bytes_xfered / 1024);
					host->perf.wtime_drv =
						ktime_add(host->perf.wtime_drv,
							diff);
					host->perf.wcount++;
					if (host->debug_mask & MMC_DEBUG_RANDOM_RW) {
						if (mrq->data->bytes_xfered <= 32*1024) {
							host->perf.wbytes_drv_rand +=
								mrq->data->bytes_xfered;
							host->perf.wtime_drv_rand =
								ktime_add(host->perf.wtime_drv_rand,
									diff);
							host->perf.wcount_rand++;
						}
					}
				}
				spin_unlock_irqrestore(&host->lock, flags);
			}
			pr_debug("%s:     %d bytes transferred: %d\n",
				mmc_hostname(host),
				mrq->data->bytes_xfered, mrq->data->error);
			trace_mmc_blk_rw_end(cmd->opcode, cmd->arg, mrq->data);
		}

		if (mrq->stop) {
			pr_debug("%s:     (CMD%u): %d: %08x %08x %08x %08x\n",
				mmc_hostname(host), mrq->stop->opcode,
				mrq->stop->error,
				mrq->stop->resp[0], mrq->stop->resp[1],
				mrq->stop->resp[2], mrq->stop->resp[3]);
		}

		if (mrq->done)
			mrq->done(mrq);

		mmc_host_clk_release(host);
	}
}

EXPORT_SYMBOL(mmc_request_done);

static void
mmc_start_request(struct mmc_host *host, struct mmc_request *mrq)
{
#ifdef CONFIG_MMC_DEBUG
	unsigned int i, sz;
	struct scatterlist *sg;
#endif

	if (mrq->sbc) {
		pr_debug("<%s: starting CMD%u arg %08x flags %08x>\n",
			 mmc_hostname(host), mrq->sbc->opcode,
			 mrq->sbc->arg, mrq->sbc->flags);
	}

	pr_debug("%s: starting CMD%u arg %08x flags %08x\n",
		 mmc_hostname(host), mrq->cmd->opcode,
		 mrq->cmd->arg, mrq->cmd->flags);

	if (mrq->data) {
		pr_debug("%s:     blksz %d blocks %d flags %08x "
			"tsac %d ms nsac %d\n",
			mmc_hostname(host), mrq->data->blksz,
			mrq->data->blocks, mrq->data->flags,
			mrq->data->timeout_ns / 1000000,
			mrq->data->timeout_clks);
		if (host->card && mmc_card_mmc(host->card))
			trace_mmc_req_start(&host->class_dev, mrq->cmd->opcode,
				mrq->cmd->arg, mrq->data->blocks, 0);
	}

	if (mrq->stop) {
		pr_debug("%s:     CMD%u arg %08x flags %08x\n",
			 mmc_hostname(host), mrq->stop->opcode,
			 mrq->stop->arg, mrq->stop->flags);
	}

	WARN_ON(!host->claimed);

	if (!host->claimed) {
		pr_err("%s %s host claim fail\n", mmc_hostname(host), __func__);
	}

	mrq->cmd->error = 0;
	mrq->cmd->mrq = mrq;
	if (mrq->data) {
		BUG_ON(mrq->data->blksz > host->max_blk_size);
		BUG_ON(mrq->data->blocks > host->max_blk_count);
		BUG_ON(mrq->data->blocks * mrq->data->blksz >
			host->max_req_size);

#ifdef CONFIG_MMC_DEBUG
		sz = 0;
		for_each_sg(mrq->data->sg, sg, mrq->data->sg_len, i)
			sz += sg->length;
		BUG_ON(sz != mrq->data->blocks * mrq->data->blksz);
#endif

		mrq->cmd->data = mrq->data;
		mrq->data->error = 0;
		mrq->data->mrq = mrq;
		if (mrq->stop) {
			mrq->data->stop = mrq->stop;
			mrq->stop->error = 0;
			mrq->stop->mrq = mrq;
		}
		if (host->perf_enable)
			host->perf.start = ktime_get();
	}
	mmc_host_clk_hold(host);

	if (mmc_is_data_request(mrq)) {
		mmc_deferred_scaling(host);
		mmc_clk_scaling_start_busy(host, true);
	}

	host->ops->request(host, mrq);
}

static void mmc_start_cmdq_request(struct mmc_host *host,
				   struct mmc_request *mrq)
{
	if (mrq->data) {
		pr_debug("%s:     blksz %d blocks %d flags %08x tsac %lu ms nsac %d\n",
			mmc_hostname(host), mrq->data->blksz,
			mrq->data->blocks, mrq->data->flags,
			mrq->data->timeout_ns / NSEC_PER_MSEC,
			mrq->data->timeout_clks);

		BUG_ON(mrq->data->blksz > host->max_blk_size);
		BUG_ON(mrq->data->blocks > host->max_blk_count);
		BUG_ON(mrq->data->blocks * mrq->data->blksz >
			host->max_req_size);
		mrq->data->error = 0;
		mrq->data->mrq = mrq;
	}

	if (mrq->cmd) {
		mrq->cmd->error = 0;
		mrq->cmd->mrq = mrq;
	}

	mmc_host_clk_hold(host);
	if (likely(host->cmdq_ops->request))
		host->cmdq_ops->request(host, mrq);
	else
		pr_err("%s: %s: issue request failed\n", mmc_hostname(host),
				__func__);
}

void mmc_blk_init_bkops_statistics(struct mmc_card *card)
{
	int i;
	struct mmc_bkops_stats *stats;

	if (!card)
		return;

	stats = &card->bkops.stats;
	spin_lock(&stats->lock);

	stats->manual_start = 0;
	stats->hpi = 0;
	stats->auto_start = 0;
	stats->auto_stop = 0;
	for (i = 0 ; i < MMC_BKOPS_NUM_SEVERITY_LEVELS ; i++)
		stats->level[i] = 0;
	stats->enabled = true;

	spin_unlock(&stats->lock);
}
EXPORT_SYMBOL(mmc_blk_init_bkops_statistics);

static void mmc_update_bkops_hpi(struct mmc_bkops_stats *stats)
{
	spin_lock_irq(&stats->lock);
	if (stats->enabled)
		stats->hpi++;
	spin_unlock_irq(&stats->lock);
}

static void mmc_update_bkops_start(struct mmc_bkops_stats *stats)
{
	spin_lock_irq(&stats->lock);
	if (stats->enabled)
		stats->manual_start++;
	spin_unlock_irq(&stats->lock);
}

static void mmc_update_bkops_auto_on(struct mmc_bkops_stats *stats)
{
	spin_lock_irq(&stats->lock);
	if (stats->enabled)
		stats->auto_start++;
	spin_unlock_irq(&stats->lock);
}

static void mmc_update_bkops_auto_off(struct mmc_bkops_stats *stats)
{
	spin_lock_irq(&stats->lock);
	if (stats->enabled)
		stats->auto_stop++;
	spin_unlock_irq(&stats->lock);
}

static void mmc_update_bkops_level(struct mmc_bkops_stats *stats,
					unsigned level)
{
	BUG_ON(level >= MMC_BKOPS_NUM_SEVERITY_LEVELS);
	spin_lock_irq(&stats->lock);
	if (stats->enabled)
		stats->level[level]++;
	spin_unlock_irq(&stats->lock);
}

int mmc_set_auto_bkops(struct mmc_card *card, bool enable)
{
	int ret = 0;
	u8 bkops_en;

	BUG_ON(!card);
	enable = !!enable;

	if (unlikely(!mmc_card_support_auto_bkops(card))) {
		pr_err("%s: %s: card doesn't support auto bkops\n",
				mmc_hostname(card->host), __func__);
		return -EPERM;
	}

	if (enable) {
		if (mmc_card_doing_auto_bkops(card))
			goto out;
		bkops_en = card->ext_csd.bkops_en | EXT_CSD_BKOPS_AUTO_EN;
	} else {
		if (!mmc_card_doing_auto_bkops(card))
			goto out;
		bkops_en = card->ext_csd.bkops_en & ~EXT_CSD_BKOPS_AUTO_EN;
	}

	ret = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_BKOPS_EN,
			bkops_en, 0);
	if (ret) {
		pr_err("%s: %s: error in setting auto bkops to %d (%d)\n",
			mmc_hostname(card->host), __func__, enable, ret);
	} else {
		if (enable) {
			mmc_card_set_auto_bkops(card);
			mmc_update_bkops_auto_on(&card->bkops.stats);
		} else {
			mmc_card_clr_auto_bkops(card);
			mmc_update_bkops_auto_off(&card->bkops.stats);
		}
		card->ext_csd.bkops_en = bkops_en;
		pr_debug("%s: %s: bkops state %x\n",
				mmc_hostname(card->host), __func__, bkops_en);
	}
out:
	return ret;
}
EXPORT_SYMBOL(mmc_set_auto_bkops);

void mmc_check_bkops(struct mmc_card *card)
{
	int err;

	BUG_ON(!card);

	if (mmc_card_doing_bkops(card))
		return;

	err = mmc_read_bkops_status(card);
	if (err) {
		pr_err("%s: Failed to read bkops status: %d\n",
		       mmc_hostname(card->host), err);
		return;
	}

	card->bkops.needs_check = false;

	mmc_update_bkops_level(&card->bkops.stats,
				card->ext_csd.raw_bkops_status);

	card->bkops.needs_bkops = card->ext_csd.raw_bkops_status > 0;
}
EXPORT_SYMBOL(mmc_check_bkops);

void mmc_start_manual_bkops(struct mmc_card *card)
{
	int err;

	BUG_ON(!card);

	if (unlikely(!mmc_card_configured_manual_bkops(card)))
		return;

	if (mmc_card_doing_bkops(card))
		return;

	err = __mmc_switch(card, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_BKOPS_START,
				1, 0, false, true, false);
	if (err) {
		pr_err("%s: Error %d starting manual bkops\n",
				mmc_hostname(card->host), err);
	} else {
		mmc_card_set_doing_bkops(card);
		mmc_update_bkops_start(&card->bkops.stats);
		card->bkops.needs_bkops = false;
	}
}
EXPORT_SYMBOL(mmc_start_manual_bkops);

static void mmc_wait_data_done(struct mmc_request *mrq)
{
	unsigned long flags;
	struct mmc_context_info *context_info = &mrq->host->context_info;

	spin_lock_irqsave(&context_info->lock, flags);
	mrq->host->context_info.is_done_rcv = true;
	wake_up_interruptible(&mrq->host->context_info.wait);
	spin_unlock_irqrestore(&context_info->lock, flags);
}

static void mmc_wait_done(struct mmc_request *mrq)
{
	complete(&mrq->completion);
}

static int __mmc_start_data_req(struct mmc_host *host, struct mmc_request *mrq)
{
	mrq->done = mmc_wait_data_done;
	mrq->host = host;
	if (mmc_card_removed(host->card)) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_wait_data_done(mrq);
		return -ENOMEDIUM;
	}
	mmc_start_request(host, mrq);

	return 0;
}

static int __mmc_start_req(struct mmc_host *host, struct mmc_request *mrq)
{
	init_completion(&mrq->completion);
	mrq->done = mmc_wait_done;
	if (mmc_card_removed(host->card)) {
		mrq->cmd->error = -ENOMEDIUM;
		complete(&mrq->completion);
		return -ENOMEDIUM;
	}
	mmc_start_request(host, mrq);
	return 0;
}

static int mmc_wait_for_data_req_done(struct mmc_host *host,
				      struct mmc_request *mrq,
				      struct mmc_async_req *next_req)
{
	struct mmc_command *cmd;
	struct mmc_context_info *context_info = &host->context_info;
	int err;
	bool is_done_rcv = false;
	unsigned long flags;

	while (1) {
		wait_event_interruptible(context_info->wait,
				(context_info->is_done_rcv ||
				 context_info->is_new_req));
		spin_lock_irqsave(&context_info->lock, flags);
		is_done_rcv = context_info->is_done_rcv;
		context_info->is_waiting_last_req = false;
		spin_unlock_irqrestore(&context_info->lock, flags);
		if (is_done_rcv) {
			context_info->is_done_rcv = false;
			context_info->is_new_req = false;
			cmd = mrq->cmd;

			if (!cmd->error || !cmd->retries ||
			    mmc_card_removed(host->card)) {
				err = host->areq->err_check(host->card,
							    host->areq);
				break; 
			} else {
				pr_info("%s: req failed (CMD%u): %d, retrying...\n",
					mmc_hostname(host),
					cmd->opcode, cmd->error);
				cmd->retries--;
				cmd->error = 0;
				host->ops->request(host, mrq);
				continue; 
			}
		} else if (context_info->is_new_req) {
			context_info->is_new_req = false;
			if (!next_req) {
				err = MMC_BLK_NEW_REQUEST;
				break; 
			}
		}
	}
	return err;
}

#define MMC_REQUEST_TIMEOUT	10000 
static void mmc_wait_for_req_done(struct mmc_host *host,
				  struct mmc_request *mrq)
{
	struct mmc_command *cmd;
	unsigned long timeout;
	int ret;

	while (1) {
		cmd = mrq->cmd;

		
		if (cmd->busy_timeout > MMC_REQUEST_TIMEOUT)
			timeout = (cmd->busy_timeout * 2) + 3000;
		else
			timeout = MMC_REQUEST_TIMEOUT + 3000;
		ret = wait_for_completion_io_timeout(&mrq->completion,
			msecs_to_jiffies(timeout));

		if (ret == 0) {
			pr_err("%s: CMD%u %s timeout (%lums)\n",
					mmc_hostname(host), cmd->opcode,
					__func__, timeout);
			cmd->error = -ETIMEDOUT;
			break;
		}
		if ((cmd->bkops_busy || cmd->sanitize_busy) && cmd->error == -ETIMEDOUT) {
			if (!mmc_interrupt_hpi(host->card)) {
				pr_warn("%s: %s: Interrupted sanitize/bkops\n",
					   mmc_hostname(host), __func__);
				cmd->error = 0;
				break;
			} else {
				pr_err("%s: %s: Failed to interrupt sanitize\n",
				       mmc_hostname(host), __func__);
			}
		}
		if (!cmd->error || !cmd->retries ||
		    mmc_card_removed(host->card))
			break;

		pr_debug("%s: req failed (CMD%u): %d, retrying...\n",
			 mmc_hostname(host), cmd->opcode, cmd->error);
		cmd->retries--;
		cmd->error = 0;
		host->ops->request(host, mrq);
	}
}

static void mmc_pre_req(struct mmc_host *host, struct mmc_request *mrq,
		 bool is_first_req)
{
	if (host->ops->pre_req) {
		mmc_host_clk_hold(host);
		host->ops->pre_req(host, mrq, is_first_req);
		mmc_host_clk_release(host);
	}
}

static void mmc_post_req(struct mmc_host *host, struct mmc_request *mrq,
			 int err)
{
	if (host->ops->post_req) {
		mmc_host_clk_hold(host);
		host->ops->post_req(host, mrq, err);
		mmc_host_clk_release(host);
	}
}

int mmc_cmdq_discard_queue(struct mmc_host *host, u32 tasks)
{
	return mmc_discard_queue(host, tasks);
}
EXPORT_SYMBOL(mmc_cmdq_discard_queue);


void mmc_cmdq_post_req(struct mmc_host *host, int tag, int err)
{
	if (likely(host->cmdq_ops->post_req))
		host->cmdq_ops->post_req(host, tag, err);
}
EXPORT_SYMBOL(mmc_cmdq_post_req);

int mmc_cmdq_halt(struct mmc_host *host, bool halt)
{
	int err = 0;

	if (mmc_host_cq_disable(host)) {
		pr_debug("%s: %s: CQE is already disabled\n",
				mmc_hostname(host), __func__);
		return 0;
	}

	if ((halt && mmc_host_halt(host)) ||
			(!halt && !mmc_host_halt(host))) {
		pr_debug("%s: %s: CQE is already %s\n", mmc_hostname(host),
				__func__, halt ? "halted" : "un-halted");
		return 0;
	}

	mmc_host_clk_hold(host);
	if (host->cmdq_ops->halt) {
		err = host->cmdq_ops->halt(host, halt);
		if (!err && host->ops->notify_halt)
			host->ops->notify_halt(host, halt);
		if (!err && halt)
			mmc_host_set_halt(host);
		else if (!err && !halt) {
			mmc_host_clr_halt(host);
			wake_up(&host->cmdq_ctx.wait);
		}
	} else {
		err = -ENOSYS;
	}
	mmc_host_clk_release(host);
	return err;
}
EXPORT_SYMBOL(mmc_cmdq_halt);

int mmc_cmdq_start_req(struct mmc_host *host, struct mmc_cmdq_req *cmdq_req)
{
	struct mmc_request *mrq = &cmdq_req->mrq;

	mrq->host = host;
	if (mmc_card_removed(host->card)) {
		mrq->cmd->error = -ENOMEDIUM;
		return -ENOMEDIUM;
	}
	mmc_start_cmdq_request(host, mrq);
	return 0;
}
EXPORT_SYMBOL(mmc_cmdq_start_req);

static void mmc_cmdq_dcmd_req_done(struct mmc_request *mrq)
{
	mmc_host_clk_release(mrq->host);
	complete(&mrq->completion);
}

int mmc_cmdq_wait_for_dcmd(struct mmc_host *host,
			struct mmc_cmdq_req *cmdq_req)
{
	struct mmc_request *mrq = &cmdq_req->mrq;
	struct mmc_command *cmd = mrq->cmd;
	int err = 0;

	init_completion(&mrq->completion);
	mrq->done = mmc_cmdq_dcmd_req_done;
	err = mmc_cmdq_start_req(host, cmdq_req);
	if (err)
		return err;

	wait_for_completion_io(&mrq->completion);
	if (cmd->error) {
		pr_err("%s: DCMD %d failed with err %d\n",
				mmc_hostname(host), cmd->opcode,
				cmd->error);
		err = cmd->error;
		mmc_host_clk_hold(host);
		host->cmdq_ops->dumpstate(host);
		mmc_host_clk_release(host);
	}
	return err;
}
EXPORT_SYMBOL(mmc_cmdq_wait_for_dcmd);

int mmc_cmdq_prepare_flush(struct mmc_command *cmd)
{
	return   __mmc_switch_cmdq_mode(cmd, EXT_CSD_CMD_SET_NORMAL,
				     EXT_CSD_FLUSH_CACHE, 1,
				     0, true, true);
}
EXPORT_SYMBOL(mmc_cmdq_prepare_flush);

struct mmc_async_req *mmc_start_req(struct mmc_host *host,
				    struct mmc_async_req *areq, int *error)
{
	int err = 0;
	int start_err = 0;
	struct mmc_async_req *data = host->areq;

	
	if (areq)
		mmc_pre_req(host, areq->mrq, !host->areq);

	if (host->areq) {
		err = mmc_wait_for_data_req_done(host, host->areq->mrq,	areq);
		if (err == MMC_BLK_NEW_REQUEST) {
			if (error)
				*error = err;
			return NULL;
		}
		if (host->card && mmc_card_mmc(host->card) &&
		    ((mmc_resp_type(host->areq->mrq->cmd) == MMC_RSP_R1) ||
		     (mmc_resp_type(host->areq->mrq->cmd) == MMC_RSP_R1B)) &&
		    (host->areq->mrq->cmd->resp[0] & R1_EXCEPTION_EVENT))
			mmc_check_bkops(host->card);
	}

	if (!err && areq) {
		trace_mmc_blk_rw_start(areq->mrq->cmd->opcode,
				       areq->mrq->cmd->arg,
				       areq->mrq->data);
		start_err = __mmc_start_data_req(host, areq->mrq);
	}

	if (host->areq)
		mmc_post_req(host, host->areq->mrq, 0);

	if (err && areq)
		mmc_post_req(host, areq->mrq, -EINVAL);

	if (err)
		host->areq = NULL;
	else
		host->areq = areq;

	if (error)
		*error = err;
	return data;
}
EXPORT_SYMBOL(mmc_start_req);

void mmc_wait_for_req(struct mmc_host *host, struct mmc_request *mrq)
{
#ifdef CONFIG_MMC_BLOCK_DEFERRED_RESUME
	if (mmc_bus_needs_resume(host))
		mmc_resume_bus(host);
#endif
	__mmc_start_req(host, mrq);
	mmc_wait_for_req_done(host, mrq);
}
EXPORT_SYMBOL(mmc_wait_for_req);

int mmc_interrupt_hpi(struct mmc_card *card)
{
	int err;
	u32 status;
	unsigned long prg_wait;

	BUG_ON(!card);

	if (!card->ext_csd.hpi_en) {
		pr_info("%s: HPI enable bit unset\n", mmc_hostname(card->host));
		return 1;
	}

	mmc_claim_host(card->host);
	err = mmc_send_status(card, &status);
	if (err) {
		pr_err("%s: Get card status fail\n", mmc_hostname(card->host));
		goto out;
	}

	switch (R1_CURRENT_STATE(status)) {
	case R1_STATE_IDLE:
	case R1_STATE_READY:
	case R1_STATE_STBY:
	case R1_STATE_TRAN:
		goto out;
	case R1_STATE_PRG:
		break;
	default:
		
		pr_debug("%s: HPI cannot be sent. Card state=%d\n",
			mmc_hostname(card->host), R1_CURRENT_STATE(status));
		err = -EINVAL;
		goto out;
	}

	err = mmc_send_hpi_cmd(card, &status);

	prg_wait = jiffies + msecs_to_jiffies(card->ext_csd.out_of_int_time);
	do {
		err = mmc_send_status(card, &status);

		if (!err && R1_CURRENT_STATE(status) == R1_STATE_TRAN)
			break;
		if (time_after(jiffies, prg_wait)) {
			err = mmc_send_status(card, &status);
			if (!err && R1_CURRENT_STATE(status) != R1_STATE_TRAN)
				err = -ETIMEDOUT;
			else
				break;
		}
	} while (!err);

out:
	mmc_release_host(card->host);
	return err;
}
EXPORT_SYMBOL(mmc_interrupt_hpi);

int mmc_wait_for_cmd(struct mmc_host *host, struct mmc_command *cmd, int retries)
{
	struct mmc_request mrq = {NULL};

	WARN_ON(!host->claimed);

	memset(cmd->resp, 0, sizeof(cmd->resp));
	cmd->retries = retries;

	mrq.cmd = cmd;
	cmd->data = NULL;

	mmc_wait_for_req(host, &mrq);

	return cmd->error;
}

EXPORT_SYMBOL(mmc_wait_for_cmd);

int mmc_stop_bkops(struct mmc_card *card)
{
	int err = 0;

	BUG_ON(!card);
	if (unlikely(!mmc_card_configured_manual_bkops(card)))
		goto out;
	if (!mmc_card_doing_bkops(card))
		goto out;

	err = mmc_interrupt_hpi(card);

	if (!err || (err == -EINVAL)) {
		mmc_card_clr_doing_bkops(card);
		mmc_update_bkops_hpi(&card->bkops.stats);
		err = 0;
	}
out:
	return err;
}
EXPORT_SYMBOL(mmc_stop_bkops);

int mmc_read_bkops_status(struct mmc_card *card)
{
	int err;
	u8 *ext_csd;

	ext_csd = kmalloc(512, GFP_KERNEL);
	if (!ext_csd) {
		pr_err("%s: could not allocate buffer to receive the ext_csd.\n",
		       mmc_hostname(card->host));
		return -ENOMEM;
	}

	mmc_claim_host(card->host);
	err = mmc_send_ext_csd(card, ext_csd);
	mmc_release_host(card->host);
	if (err)
		goto out;

	card->ext_csd.raw_bkops_status = ext_csd[EXT_CSD_BKOPS_STATUS] &
		MMC_BKOPS_URGENCY_MASK;
	card->ext_csd.raw_exception_status =
		ext_csd[EXT_CSD_EXP_EVENTS_STATUS] & (EXT_CSD_URGENT_BKOPS |
						      EXT_CSD_DYNCAP_NEEDED |
						      EXT_CSD_SYSPOOL_EXHAUSTED
						      | EXT_CSD_PACKED_FAILURE);
out:
	kfree(ext_csd);
	return err;
}
EXPORT_SYMBOL(mmc_read_bkops_status);

void mmc_set_data_timeout(struct mmc_data *data, const struct mmc_card *card)
{
	unsigned int mult;

	if (!card) {
		WARN_ON(1);
		return;
	}
	if (mmc_card_sdio(card)) {
		data->timeout_ns = 1000000000;
		data->timeout_clks = 0;
		return;
	}

	mult = mmc_card_sd(card) ? 100 : 10;

	if (data->flags & MMC_DATA_WRITE)
		mult <<= card->csd.r2w_factor;

	data->timeout_ns = card->csd.tacc_ns * mult;
	data->timeout_clks = card->csd.tacc_clks * mult;

	if (mmc_card_sd(card)) {
		unsigned int timeout_us, limit_us;

		timeout_us = data->timeout_ns / 1000;
		if (mmc_host_clk_rate(card->host))
			timeout_us += data->timeout_clks * 1000 /
				(mmc_host_clk_rate(card->host) / 1000);

		if (data->flags & MMC_DATA_WRITE)
			limit_us = 3000000;
		else
			limit_us = 100000;

		if (timeout_us > limit_us || mmc_card_blockaddr(card)) {
			data->timeout_ns = limit_us * 1000;
			data->timeout_clks = 0;
		}

		
		if (timeout_us == 0)
			data->timeout_ns = limit_us * 1000;
	}

	if (mmc_card_long_read_time(card) && data->flags & MMC_DATA_READ) {
		data->timeout_ns = 4000000000u;
		data->timeout_clks = 0;
	}

	if (mmc_host_is_spi(card->host)) {
		if (data->flags & MMC_DATA_WRITE) {
			if (data->timeout_ns < 1000000000)
				data->timeout_ns = 1000000000;	
		} else {
			if (data->timeout_ns < 100000000)
				data->timeout_ns =  100000000;	
		}
	}
	
	if (card->quirks & MMC_QUIRK_INAND_DATA_TIMEOUT) {
		data->timeout_ns = 4000000000u; 
		data->timeout_clks = 0;
	}
}
EXPORT_SYMBOL(mmc_set_data_timeout);

unsigned int mmc_align_data_size(struct mmc_card *card, unsigned int sz)
{
	sz = ((sz + 3) / 4) * 4;

	return sz;
}
EXPORT_SYMBOL(mmc_align_data_size);

int __mmc_claim_host(struct mmc_host *host, atomic_t *abort)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	int stop;

	might_sleep();

	add_wait_queue(&host->wq, &wait);

	spin_lock_irqsave(&host->lock, flags);
	while (1) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		stop = abort ? atomic_read(abort) : 0;
		if (stop || !host->claimed || host->claimer == current)
			break;
		spin_unlock_irqrestore(&host->lock, flags);
		schedule();
		spin_lock_irqsave(&host->lock, flags);
	}
	set_current_state(TASK_RUNNING);
	if (!stop) {
		host->claimed = 1;
		host->claimer = current;
		host->claim_cnt += 1;
	} else
		wake_up(&host->wq);
	spin_unlock_irqrestore(&host->lock, flags);
	remove_wait_queue(&host->wq, &wait);
	if (host->ops->enable && !stop && host->claim_cnt == 1)
		host->ops->enable(host);
	return stop;
}

EXPORT_SYMBOL(__mmc_claim_host);

void mmc_release_host(struct mmc_host *host)
{
	unsigned long flags;

	WARN_ON(!host->claimed);

	if (host->ops->disable && host->claim_cnt == 1)
		host->ops->disable(host);

	spin_lock_irqsave(&host->lock, flags);
	if (--host->claim_cnt) {
		
		spin_unlock_irqrestore(&host->lock, flags);
	} else {
		host->claimed = 0;
		host->claimer = NULL;
		spin_unlock_irqrestore(&host->lock, flags);
		wake_up(&host->wq);
	}
}
EXPORT_SYMBOL(mmc_release_host);

void mmc_get_card(struct mmc_card *card)
{
	pm_runtime_get_sync(&card->dev);
	mmc_claim_host(card->host);
}
EXPORT_SYMBOL(mmc_get_card);


void mmc_put_card(struct mmc_card *card)
{
	mmc_release_host(card->host);
	pm_runtime_mark_last_busy(&card->dev);
	pm_runtime_put_autosuspend(&card->dev);
}
EXPORT_SYMBOL(mmc_put_card);

void mmc_set_ios(struct mmc_host *host)
{
	struct mmc_ios *ios = &host->ios;

	pr_debug("%s: clock %uHz busmode %u powermode %u cs %u Vdd %u "
		"width %u timing %u\n",
		 mmc_hostname(host), ios->clock, ios->bus_mode,
		 ios->power_mode, ios->chip_select, ios->vdd,
		 ios->bus_width, ios->timing);

	if (ios->clock > 0)
		mmc_set_ungated(host);
	host->ops->set_ios(host, ios);
	if (ios->old_rate != ios->clock) {
		if (likely(ios->clk_ts)) {
			char trace_info[80];
			snprintf(trace_info, 80,
				"%s: freq_KHz %d --> %d | t = %d",
				mmc_hostname(host), ios->old_rate / 1000,
				ios->clock / 1000, jiffies_to_msecs(
					(long)jiffies - (long)ios->clk_ts));
			trace_mmc_clk(trace_info);
		}
		ios->old_rate = ios->clock;
		ios->clk_ts = jiffies;
	}
}
EXPORT_SYMBOL(mmc_set_ios);

void mmc_set_chip_select(struct mmc_host *host, int mode)
{
	mmc_host_clk_hold(host);
	host->ios.chip_select = mode;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

static void __mmc_set_clock(struct mmc_host *host, unsigned int hz)
{
	WARN_ON(hz && hz < host->f_min);

	if (hz > host->f_max)
		hz = host->f_max;

	host->ios.clock = hz;
	mmc_set_ios(host);
}

void mmc_set_clock(struct mmc_host *host, unsigned int hz)
{
	mmc_host_clk_hold(host);
	__mmc_set_clock(host, hz);
	mmc_host_clk_release(host);
}

#ifdef CONFIG_MMC_CLKGATE
void mmc_gate_clock(struct mmc_host *host)
{
	unsigned long flags;

	WARN_ON(!host->ios.clock);

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clk_old = host->ios.clock;
	host->ios.clock = 0;
	host->clk_gated = true;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	mmc_set_ios(host);
}

void mmc_ungate_clock(struct mmc_host *host)
{
	if (host->clk_old) {
		WARN_ON(host->ios.clock);
		
		__mmc_set_clock(host, host->clk_old);
	}
}

void mmc_set_ungated(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clk_gated = false;
	spin_unlock_irqrestore(&host->clk_lock, flags);
}

#else
void mmc_set_ungated(struct mmc_host *host)
{
}
#endif

void mmc_set_bus_mode(struct mmc_host *host, unsigned int mode)
{
	mmc_host_clk_hold(host);
	host->ios.bus_mode = mode;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

void mmc_set_bus_width(struct mmc_host *host, unsigned int width)
{
	mmc_host_clk_hold(host);
	host->ios.bus_width = width;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

static int mmc_vdd_to_ocrbitnum(int vdd, bool low_bits)
{
	const int max_bit = ilog2(MMC_VDD_35_36);
	int bit;

	if (vdd < 1650 || vdd > 3600)
		return -EINVAL;

	if (vdd >= 1650 && vdd <= 1950)
		return ilog2(MMC_VDD_165_195);

	if (low_bits)
		vdd -= 1;

	
	bit = (vdd - 2000) / 100 + 8;
	if (bit > max_bit)
		return max_bit;
	return bit;
}

u32 mmc_vddrange_to_ocrmask(int vdd_min, int vdd_max)
{
	u32 mask = 0;

	if (vdd_max < vdd_min)
		return 0;

	
	vdd_max = mmc_vdd_to_ocrbitnum(vdd_max, false);
	if (vdd_max < 0)
		return 0;

	
	vdd_min = mmc_vdd_to_ocrbitnum(vdd_min, true);
	if (vdd_min < 0)
		return 0;

	
	while (vdd_max >= vdd_min)
		mask |= 1 << vdd_max--;

	return mask;
}
EXPORT_SYMBOL(mmc_vddrange_to_ocrmask);

#ifdef CONFIG_OF

int mmc_of_parse_voltage(struct device_node *np, u32 *mask)
{
	const u32 *voltage_ranges;
	int num_ranges, i;

	voltage_ranges = of_get_property(np, "voltage-ranges", &num_ranges);
	num_ranges = num_ranges / sizeof(*voltage_ranges) / 2;
	if (!voltage_ranges || !num_ranges) {
		pr_info("%s: voltage-ranges unspecified\n", np->full_name);
		return -EINVAL;
	}

	for (i = 0; i < num_ranges; i++) {
		const int j = i * 2;
		u32 ocr_mask;

		ocr_mask = mmc_vddrange_to_ocrmask(
				be32_to_cpu(voltage_ranges[j]),
				be32_to_cpu(voltage_ranges[j + 1]));
		if (!ocr_mask) {
			pr_err("%s: voltage-range #%d is invalid\n",
				np->full_name, i);
			return -EINVAL;
		}
		*mask |= ocr_mask;
	}

	return 0;
}
EXPORT_SYMBOL(mmc_of_parse_voltage);

#endif 

#ifdef CONFIG_REGULATOR

int mmc_regulator_get_ocrmask(struct regulator *supply)
{
	int			result = 0;
	int			count;
	int			i;
	int			vdd_uV;
	int			vdd_mV;

	count = regulator_count_voltages(supply);
	if (count < 0)
		return count;

	for (i = 0; i < count; i++) {
		vdd_uV = regulator_list_voltage(supply, i);
		if (vdd_uV <= 0)
			continue;

		vdd_mV = vdd_uV / 1000;
		result |= mmc_vddrange_to_ocrmask(vdd_mV, vdd_mV);
	}

	if (!result) {
		vdd_uV = regulator_get_voltage(supply);
		if (vdd_uV <= 0)
			return vdd_uV;

		vdd_mV = vdd_uV / 1000;
		result = mmc_vddrange_to_ocrmask(vdd_mV, vdd_mV);
	}

	return result;
}
EXPORT_SYMBOL_GPL(mmc_regulator_get_ocrmask);

int mmc_regulator_set_ocr(struct mmc_host *mmc,
			struct regulator *supply,
			unsigned short vdd_bit)
{
	int			result = 0;
	int			min_uV, max_uV;

	if (vdd_bit) {
		int		tmp;

		tmp = vdd_bit - ilog2(MMC_VDD_165_195);
		if (tmp == 0) {
			min_uV = 1650 * 1000;
			max_uV = 1950 * 1000;
		} else {
			min_uV = 1900 * 1000 + tmp * 100 * 1000;
			max_uV = min_uV + 100 * 1000;
		}

		result = regulator_set_voltage(supply, min_uV, max_uV);
		if (result == 0 && !mmc->regulator_enabled) {
			result = regulator_enable(supply);
			if (!result)
				mmc->regulator_enabled = true;
		}
	} else if (mmc->regulator_enabled) {
		result = regulator_disable(supply);
		if (result == 0)
			mmc->regulator_enabled = false;
	}

	if (result)
		dev_err(mmc_dev(mmc),
			"could not set regulator OCR (%d)\n", result);
	return result;
}
EXPORT_SYMBOL_GPL(mmc_regulator_set_ocr);

#endif 

int mmc_regulator_get_supply(struct mmc_host *mmc)
{
	struct device *dev = mmc_dev(mmc);
	int ret;

	mmc->supply.vmmc = devm_regulator_get_optional(dev, "vmmc");
	mmc->supply.vqmmc = devm_regulator_get_optional(dev, "vqmmc");

	if (IS_ERR(mmc->supply.vmmc)) {
		if (PTR_ERR(mmc->supply.vmmc) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(dev, "No vmmc regulator found\n");
	} else {
		ret = mmc_regulator_get_ocrmask(mmc->supply.vmmc);
		if (ret > 0)
			mmc->ocr_avail = ret;
		else
			dev_warn(dev, "Failed getting OCR mask: %d\n", ret);
	}

	if (IS_ERR(mmc->supply.vqmmc)) {
		if (PTR_ERR(mmc->supply.vqmmc) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(dev, "No vqmmc regulator found\n");
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mmc_regulator_get_supply);

u32 mmc_select_voltage(struct mmc_host *host, u32 ocr)
{
	int bit;

	if (ocr & 0x7F) {
		dev_warn(mmc_dev(host),
		"card claims to support voltages below defined range\n");
		ocr &= ~0x7F;
	}

	ocr &= host->ocr_avail;
	if (!ocr) {
		dev_warn(mmc_dev(host), "no support for card's volts\n");
		return 0;
	}

	if (host->caps2 & MMC_CAP2_FULL_PWR_CYCLE) {
		bit = ffs(ocr) - 1;
		ocr &= 3 << bit;
		mmc_power_cycle(host, ocr);
	} else {
		bit = fls(ocr) - 1;
		ocr &= 3 << bit;
		if (bit != host->ios.vdd)
			dev_warn(mmc_dev(host), "exceeding card's volts\n");
	}

	return ocr;
}

int __mmc_set_signal_voltage(struct mmc_host *host, int signal_voltage)
{
	int err = 0;
	int old_signal_voltage = host->ios.signal_voltage;

	host->ios.signal_voltage = signal_voltage;
	if (host->ops->start_signal_voltage_switch) {
		mmc_host_clk_hold(host);
		err = host->ops->start_signal_voltage_switch(host, &host->ios);
		mmc_host_clk_release(host);
	}

	if (err)
		host->ios.signal_voltage = old_signal_voltage;

	return err;

}

int mmc_set_signal_voltage(struct mmc_host *host, int signal_voltage, u32 ocr)
{
	struct mmc_command cmd = {0};
	int err = 0;
	u32 clock;

	BUG_ON(!host);

	if (signal_voltage == MMC_SIGNAL_VOLTAGE_330)
		return __mmc_set_signal_voltage(host, signal_voltage);

	if (!host->ops->start_signal_voltage_switch)
		return -EPERM;
	if (!host->ops->card_busy)
		pr_warn("%s: cannot verify signal voltage switch\n",
			mmc_hostname(host));

	cmd.opcode = SD_SWITCH_VOLTAGE;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	mmc_host_clk_hold(host);
	err = mmc_wait_for_cmd(host, &cmd, 0);
	if (err)
		goto exit;

	if (!mmc_host_is_spi(host) && (cmd.resp[0] & R1_ERROR)) {
		err = -EIO;
		goto exit;
	}

	mmc_delay(1);
	if (host->ops->card_busy && !host->ops->card_busy(host)) {
		err = -EAGAIN;
		goto power_cycle;
	}
	host->card_clock_off = true;
	clock = host->ios.clock;
	host->ios.clock = 0;
	mmc_set_ios(host);

	if (__mmc_set_signal_voltage(host, signal_voltage)) {
		err = -EAGAIN;
		host->ios.clock = clock;
		mmc_set_ios(host);
		host->card_clock_off = false;
		goto power_cycle;
	}

	
	mmc_delay(5);
	host->ios.clock = clock;
	mmc_set_ios(host);

	host->card_clock_off = false;
	
	mmc_delay(1);

	if (host->ops->card_busy && host->ops->card_busy(host))
		err = -EAGAIN;

power_cycle:
	if (err) {
		pr_debug("%s: Signal voltage switch failed, "
			"power cycling card\n", mmc_hostname(host));
		mmc_power_cycle(host, ocr);
	}

exit:
	mmc_host_clk_release(host);

	return err;
}

void mmc_set_timing(struct mmc_host *host, unsigned int timing)
{
	mmc_host_clk_hold(host);
	host->ios.timing = timing;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

void mmc_set_driver_type(struct mmc_host *host, unsigned int drv_type)
{
	mmc_host_clk_hold(host);
	host->ios.drv_type = drv_type;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

void mmc_power_up(struct mmc_host *host, u32 ocr)
{
	if (host->ios.power_mode == MMC_POWER_ON)
		return;

	mmc_host_clk_hold(host);

	host->ios.vdd = fls(ocr) - 1;
	if (mmc_host_is_spi(host))
		host->ios.chip_select = MMC_CS_HIGH;
	else {
		host->ios.chip_select = MMC_CS_DONTCARE;
		host->ios.bus_mode = MMC_BUSMODE_OPENDRAIN;
	}
	host->ios.power_mode = MMC_POWER_UP;
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;
	mmc_set_ios(host);

	
	if (__mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_330) == 0)
		dev_dbg(mmc_dev(host), "Initial signal voltage of 3.3v\n");
	else if (__mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_180) == 0)
		dev_dbg(mmc_dev(host), "Initial signal voltage of 1.8v\n");
	else if (__mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_120) == 0)
		dev_dbg(mmc_dev(host), "Initial signal voltage of 1.2v\n");

	mmc_delay(10);

	host->ios.clock = host->f_init;

	host->ios.power_mode = MMC_POWER_ON;
	mmc_set_ios(host);

	mmc_delay(10);

	mmc_host_clk_release(host);
}

void mmc_power_off(struct mmc_host *host)
{
	if (host->ios.power_mode == MMC_POWER_OFF)
		return;

	mmc_host_clk_hold(host);

	host->ios.clock = 0;
	host->ios.vdd = 0;

	if (!mmc_host_is_spi(host)) {
		host->ios.bus_mode = MMC_BUSMODE_OPENDRAIN;
		host->ios.chip_select = MMC_CS_DONTCARE;
	}
	host->ios.power_mode = MMC_POWER_OFF;
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;
	mmc_set_ios(host);

	mmc_delay(1);

	mmc_host_clk_release(host);
}

void mmc_power_cycle(struct mmc_host *host, u32 ocr)
{
	mmc_power_off(host);
	
	mmc_delay(50);
	mmc_power_up(host, ocr);
}

static void __mmc_release_bus(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(host->bus_refs);
	BUG_ON(!host->bus_dead);

	host->bus_ops = NULL;
}

static inline void mmc_bus_get(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->bus_refs++;
	spin_unlock_irqrestore(&host->lock, flags);
}

static inline void mmc_bus_put(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->bus_refs--;
	if ((host->bus_refs == 0) && host->bus_ops)
		__mmc_release_bus(host);
	spin_unlock_irqrestore(&host->lock, flags);
}

int mmc_resume_bus(struct mmc_host *host)
{
	unsigned long flags;
	int err = 0;

	if (!mmc_bus_needs_resume(host))
		return -EINVAL;

	pr_debug("%s: Starting deferred resume\n", mmc_hostname(host));
	spin_lock_irqsave(&host->lock, flags);
	host->bus_resume_flags &= ~MMC_BUSRESUME_NEEDS_RESUME;
	spin_unlock_irqrestore(&host->lock, flags);

	mmc_bus_get(host);
	if (host->bus_ops && !host->bus_dead && host->card) {
		mmc_power_up(host, host->card->ocr);
		BUG_ON(!host->bus_ops->resume);
		host->bus_ops->resume(host);
		if (mmc_card_cmdq(host->card)) {
			err = mmc_cmdq_halt(host, false);
			if (err)
				pr_err("%s: %s: unhalt failed: %d\n",
				       mmc_hostname(host), __func__, err);
			else
				mmc_card_clr_suspended(host->card);
		}
	}

	mmc_bus_put(host);
	pr_debug("%s: Deferred resume completed\n", mmc_hostname(host));
	return 0;
}
EXPORT_SYMBOL(mmc_resume_bus);

void mmc_attach_bus(struct mmc_host *host, const struct mmc_bus_ops *ops)
{
	unsigned long flags;

	BUG_ON(!host);
	BUG_ON(!ops);

	WARN_ON(!host->claimed);

	spin_lock_irqsave(&host->lock, flags);

	BUG_ON(host->bus_ops);
	BUG_ON(host->bus_refs);

	host->bus_ops = ops;
	host->bus_refs = 1;
	host->bus_dead = 0;

	spin_unlock_irqrestore(&host->lock, flags);
}

void mmc_detach_bus(struct mmc_host *host)
{
	unsigned long flags;

	BUG_ON(!host);

	WARN_ON(!host->claimed);
	WARN_ON(!host->bus_ops);

	spin_lock_irqsave(&host->lock, flags);

	host->bus_dead = 1;

	spin_unlock_irqrestore(&host->lock, flags);

	mmc_bus_put(host);
}

static void _mmc_detect_change(struct mmc_host *host, unsigned long delay,
				bool cd_irq)
{
#ifdef CONFIG_MMC_DEBUG
	unsigned long flags;
	spin_lock_irqsave(&host->lock, flags);
	WARN_ON(host->removed);
	spin_unlock_irqrestore(&host->lock, flags);
#endif

	if (cd_irq && !(host->caps & MMC_CAP_NEEDS_POLL) &&
		device_can_wakeup(mmc_dev(host)))
		pm_wakeup_event(mmc_dev(host), 5000);
	else
		pm_wakeup_event(mmc_dev(host), 1000);

	host->detect_change = 1;
	mmc_schedule_delayed_work(&host->detect, delay);
}

void mmc_detect_change(struct mmc_host *host, unsigned long delay)
{
	_mmc_detect_change(host, delay, true);
}
EXPORT_SYMBOL(mmc_detect_change);

void mmc_init_erase(struct mmc_card *card)
{
	unsigned int sz;

	if (is_power_of_2(card->erase_size))
		card->erase_shift = ffs(card->erase_size) - 1;
	else
		card->erase_shift = 0;

	if (mmc_card_sd(card) && card->ssr.au) {
		card->pref_erase = card->ssr.au;
		card->erase_shift = ffs(card->ssr.au) - 1;
	} else if (card->ext_csd.hc_erase_size) {
		card->pref_erase = card->ext_csd.hc_erase_size;
	} else if (card->erase_size) {
		sz = (card->csd.capacity << (card->csd.read_blkbits - 9)) >> 11;
		if (sz < 128)
			card->pref_erase = 512 * 1024 / 512;
		else if (sz < 512)
			card->pref_erase = 1024 * 1024 / 512;
		else if (sz < 1024)
			card->pref_erase = 2 * 1024 * 1024 / 512;
		else
			card->pref_erase = 4 * 1024 * 1024 / 512;
		if (card->pref_erase < card->erase_size)
			card->pref_erase = card->erase_size;
		else {
			sz = card->pref_erase % card->erase_size;
			if (sz)
				card->pref_erase += card->erase_size - sz;
		}
	} else
		card->pref_erase = 0;
}

static unsigned int mmc_mmc_erase_timeout(struct mmc_card *card,
				          unsigned int arg, unsigned int qty)
{
	unsigned int erase_timeout;

	if (arg == MMC_DISCARD_ARG ||
	    (arg == MMC_TRIM_ARG && card->ext_csd.rev >= 6)) {
		erase_timeout = card->ext_csd.trim_timeout;
	} else if (card->ext_csd.erase_group_def & 1) {
		
		if (arg == MMC_TRIM_ARG)
			erase_timeout = card->ext_csd.trim_timeout;
		else
			erase_timeout = card->ext_csd.hc_erase_timeout;
	} else {
		
		unsigned int mult = (10 << card->csd.r2w_factor);
		unsigned int timeout_clks = card->csd.tacc_clks * mult;
		unsigned int timeout_us;

		
		if (card->csd.tacc_ns < 1000000)
			timeout_us = (card->csd.tacc_ns * mult) / 1000;
		else
			timeout_us = (card->csd.tacc_ns / 1000) * mult;

		timeout_clks <<= 1;
		timeout_us += (timeout_clks * 1000) /
			      (mmc_host_clk_rate(card->host) / 1000);

		erase_timeout = timeout_us / 1000;

		if (!erase_timeout)
			erase_timeout = 1;
	}

	erase_timeout *= qty;

	if (mmc_host_is_spi(card->host) && erase_timeout < 1000)
		erase_timeout = 1000;

	return erase_timeout;
}

static unsigned int mmc_sd_erase_timeout(struct mmc_card *card,
					 unsigned int arg,
					 unsigned int qty)
{
	unsigned int erase_timeout;

	if (card->ssr.erase_timeout) {
		
		erase_timeout = card->ssr.erase_timeout * qty +
				card->ssr.erase_offset;
	} else {
		erase_timeout = 250 * qty;
	}

	
	if (erase_timeout < 1000)
		erase_timeout = 1000;

	return erase_timeout;
}

static unsigned int mmc_erase_timeout(struct mmc_card *card,
				      unsigned int arg,
				      unsigned int qty)
{
	if (mmc_card_sd(card))
		return mmc_sd_erase_timeout(card, arg, qty);
	else
		return mmc_mmc_erase_timeout(card, arg, qty);
}

static u32 mmc_get_erase_qty(struct mmc_card *card, u32 from, u32 to)
{
	u32 qty = 0;
	if (card->erase_shift)
		qty += ((to >> card->erase_shift) -
			(from >> card->erase_shift)) + 1;
	else if (mmc_card_sd(card))
		qty += to - from + 1;
	else
		qty += ((to / card->erase_size) -
			(from / card->erase_size)) + 1;
	return qty;
}

static int mmc_cmdq_send_erase_cmd(struct mmc_cmdq_req *cmdq_req,
		struct mmc_card *card, u32 opcode, u32 arg, u32 qty)
{
	struct mmc_command *cmd = cmdq_req->mrq.cmd;
	int err;

	memset(cmd, 0, sizeof(struct mmc_command));

	cmd->opcode = opcode;
	cmd->arg = arg;
	if (cmd->opcode == MMC_ERASE) {
		cmd->flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
		cmd->busy_timeout = mmc_erase_timeout(card, arg, qty);
	} else {
		cmd->flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	}

	err = mmc_cmdq_wait_for_dcmd(card->host, cmdq_req);
	if (err) {
		pr_err("mmc_erase: group start error %d, status %#x\n",
				err, cmd->resp[0]);
		return -EIO;
	}
	return 0;
}

static int mmc_cmdq_do_erase(struct mmc_cmdq_req *cmdq_req,
			struct mmc_card *card, unsigned int from,
			unsigned int to, unsigned int arg)
{
	struct mmc_command *cmd = cmdq_req->mrq.cmd;
	unsigned int qty = 0;
	unsigned long timeout;
	unsigned int fr, nr;
	int err;
	ktime_t start, diff;

	fr = from;
	nr = to - from + 1;
	trace_mmc_blk_erase_start(arg, fr, nr);

	qty = mmc_get_erase_qty(card, from, to);

	if (!mmc_card_blockaddr(card)) {
		from <<= 9;
		to <<= 9;
	}

	err = mmc_cmdq_send_erase_cmd(cmdq_req, card, MMC_ERASE_GROUP_START,
			from, qty);
	if (err)
		goto out;

	err = mmc_cmdq_send_erase_cmd(cmdq_req, card, MMC_ERASE_GROUP_END,
			to, qty);
	if (err)
		goto out;

	trace_mmc_req_start(&(card->host->class_dev), MMC_ERASE,
			from, to - from + 1, 0);

	start = ktime_get();
	err = mmc_cmdq_send_erase_cmd(cmdq_req, card, MMC_ERASE,
			arg, qty);
	if (err)
		goto out;

	timeout = jiffies + msecs_to_jiffies(MMC_CORE_TIMEOUT_MS);
	do {
		memset(cmd, 0, sizeof(struct mmc_command));
		cmd->opcode = MMC_SEND_STATUS;
		cmd->arg = card->rca << 16;
		cmd->flags = MMC_RSP_R1 | MMC_CMD_AC;
		
		err = mmc_cmdq_wait_for_dcmd(card->host, cmdq_req);
		if (err || (cmd->resp[0] & 0xFDF92000)) {
			pr_err("error %d requesting status %#x\n",
				err, cmd->resp[0]);
			err = -EIO;
			goto out;
		}
		if (time_after(jiffies, timeout)) {
			pr_err("%s: Card stuck in programming state! %s\n",
				mmc_hostname(card->host), __func__);
			err =  -EIO;
			goto out;
		}
	} while (!(cmd->resp[0] & R1_READY_FOR_DATA) ||
		 (R1_CURRENT_STATE(cmd->resp[0]) == R1_STATE_PRG));

	diff = ktime_sub(ktime_get(), start);
	if (ktime_to_ms(diff) >= 3000)
		pr_info("%s: erase(sector %u to %u) takes %lld ms\n",
			mmc_hostname(card->host), from, to, ktime_to_ms(diff));

	card->host->perf.erase_blks += (to - from + 1);
	card->host->perf.erase_time =
		ktime_add(card->host->perf.erase_time, diff);
	card->host->perf.erase_rq++;

	trace_mmc_request_done(&(card->host->class_dev), MMC_ERASE,
			from, to - from + 1, ktime_to_ms(diff), 0);
out:
	trace_mmc_blk_erase_end(arg, fr, nr);
	return err;
}

static int mmc_do_erase(struct mmc_card *card, unsigned int from,
			unsigned int to, unsigned int arg)
{
	struct mmc_command cmd = {0};
	unsigned int qty = 0;
	unsigned long timeout;
	unsigned int fr, nr;
	int err;
	ktime_t start, diff;

	fr = from;
	nr = to - from + 1;
	trace_mmc_blk_erase_start(arg, fr, nr);

	qty = mmc_get_erase_qty(card, from, to);

	if (!mmc_card_blockaddr(card)) {
		from <<= 9;
		to <<= 9;
	}

	if (mmc_card_sd(card))
		cmd.opcode = SD_ERASE_WR_BLK_START;
	else
		cmd.opcode = MMC_ERASE_GROUP_START;
	cmd.arg = from;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_err("mmc_erase: group start error %d, "
		       "status %#x\n", err, cmd.resp[0]);
		err = -EIO;
		goto out;
	}

	memset(&cmd, 0, sizeof(struct mmc_command));
	if (mmc_card_sd(card))
		cmd.opcode = SD_ERASE_WR_BLK_END;
	else
		cmd.opcode = MMC_ERASE_GROUP_END;
	cmd.arg = to;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_err("mmc_erase: group end error %d, status %#x\n",
		       err, cmd.resp[0]);
		err = -EIO;
		goto out;
	}

	if (mmc_card_mmc(card)) {
		trace_mmc_req_start(&(card->host->class_dev), MMC_ERASE,
			from, to - from + 1, 0);
	}

	start = ktime_get();
	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_ERASE;
	cmd.arg = arg;
	cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	cmd.busy_timeout = mmc_erase_timeout(card, arg, qty);
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_err("mmc_erase: erase error %d, status %#x\n",
		       err, cmd.resp[0]);
		err = -EIO;
		goto out;
	}

	if (mmc_host_is_spi(card->host))
		goto out;

	timeout = jiffies + msecs_to_jiffies(MMC_CORE_TIMEOUT_MS);
	do {
		memset(&cmd, 0, sizeof(struct mmc_command));
		cmd.opcode = MMC_SEND_STATUS;
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
		
		err = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (err || (cmd.resp[0] & 0xFDF92000)) {
			pr_err("error %d requesting status %#x\n",
				err, cmd.resp[0]);
			err = -EIO;
			goto out;
		}

		if (time_after(jiffies, timeout)) {
			pr_err("%s: Card stuck in programming state! %s\n",
				mmc_hostname(card->host), __func__);
			err =  -EIO;
			goto out;
		}

	} while (!(cmd.resp[0] & R1_READY_FOR_DATA) ||
		 (R1_CURRENT_STATE(cmd.resp[0]) == R1_STATE_PRG));

	diff = ktime_sub(ktime_get(), start);
	if (ktime_to_ms(diff) >= 3000)
		pr_info("%s: erase(sector %u to %u) takes %lld ms\n",
			mmc_hostname(card->host), from, to, ktime_to_ms(diff));

	card->host->perf.erase_blks += (to - from + 1);
	card->host->perf.erase_time =
		ktime_add(card->host->perf.erase_time, diff);
	card->host->perf.erase_rq++;

	if (mmc_card_mmc(card)) {
		trace_mmc_request_done(&(card->host->class_dev), MMC_ERASE,
			from, to - from + 1, ktime_to_ms(diff), 0);
	}
out:

	trace_mmc_blk_erase_end(arg, fr, nr);
	return err;
}

int mmc_erase_sanity_check(struct mmc_card *card, unsigned int from,
		unsigned int nr, unsigned int arg)
{
	if (!(card->host->caps & MMC_CAP_ERASE) ||
	    !(card->csd.cmdclass & CCC_ERASE))
		return -EOPNOTSUPP;

	if (!card->erase_size)
		return -EOPNOTSUPP;

	if (mmc_card_sd(card) && arg != MMC_ERASE_ARG)
		return -EOPNOTSUPP;

	if ((arg & MMC_SECURE_ARGS) &&
	    !(card->ext_csd.sec_feature_support & EXT_CSD_SEC_ER_EN))
		return -EOPNOTSUPP;

	if ((arg & MMC_TRIM_ARGS) &&
	    !(card->ext_csd.sec_feature_support & EXT_CSD_SEC_GB_CL_EN))
		return -EOPNOTSUPP;

	return 0;
}

int mmc_cmdq_erase(struct mmc_cmdq_req *cmdq_req,
	      struct mmc_card *card, unsigned int from, unsigned int nr,
	      unsigned int arg)
{
	unsigned int rem, to = from + nr;
	int ret;

	ret = mmc_erase_sanity_check(card, from, nr, arg);
	if (ret)
		return ret;

	if (arg == MMC_ERASE_ARG) {
		rem = from % card->erase_size;
		if (rem) {
			rem = card->erase_size - rem;
			from += rem;
			if (nr > rem)
				nr -= rem;
			else
				return 0;
		}
		rem = nr % card->erase_size;
		if (rem)
			nr -= rem;
	}

	if (nr == 0)
		return 0;

	to = from + nr;

	if (to <= from)
		return -EINVAL;

	
	to -= 1;

	return mmc_cmdq_do_erase(cmdq_req, card, from, to, arg);
}
EXPORT_SYMBOL(mmc_cmdq_erase);

int mmc_erase(struct mmc_card *card, unsigned int from, unsigned int nr,
	      unsigned int arg)
{
	unsigned int rem, to = from + nr;
	int ret;

	ret = mmc_erase_sanity_check(card, from, nr, arg);
	if (ret)
		return ret;

	if (arg == MMC_ERASE_ARG) {
		rem = from % card->erase_size;
		if (rem) {
			rem = card->erase_size - rem;
			from += rem;
			if (nr > rem)
				nr -= rem;
			else
				return 0;
		}
		rem = nr % card->erase_size;
		if (rem)
			nr -= rem;
	}

	if (nr == 0)
		return 0;

	to = from + nr;

	if (to <= from)
		return -EINVAL;

	
	to -= 1;

	return mmc_do_erase(card, from, to, arg);
}
EXPORT_SYMBOL(mmc_erase);

int mmc_can_erase(struct mmc_card *card)
{
	if ((card->host->caps & MMC_CAP_ERASE) &&
	    (card->csd.cmdclass & CCC_ERASE) && card->erase_size)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_erase);

int mmc_can_trim(struct mmc_card *card)
{
	if (card->ext_csd.sec_feature_support & EXT_CSD_SEC_GB_CL_EN)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_trim);

int mmc_can_discard(struct mmc_card *card)
{
	if (card->ext_csd.feature_support & MMC_DISCARD_FEATURE)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_discard);

int mmc_can_sanitize(struct mmc_card *card)
{
	return 0;
}
EXPORT_SYMBOL(mmc_can_sanitize);

int mmc_can_secure_erase_trim(struct mmc_card *card)
{
	if ((card->ext_csd.sec_feature_support & EXT_CSD_SEC_ER_EN) &&
	    !(card->quirks & MMC_QUIRK_SEC_ERASE_TRIM_BROKEN))
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_secure_erase_trim);

int mmc_erase_group_aligned(struct mmc_card *card, unsigned int from,
			    unsigned int nr)
{
	if (!card->erase_size)
		return 0;
	if (from % card->erase_size || nr % card->erase_size)
		return 0;
	return 1;
}
EXPORT_SYMBOL(mmc_erase_group_aligned);

static unsigned int mmc_do_calc_max_discard(struct mmc_card *card,
					    unsigned int arg)
{
	struct mmc_host *host = card->host;
	unsigned int max_discard, x, y, qty = 0, max_qty, timeout;
	unsigned int last_timeout = 0;

	if (card->erase_shift)
		max_qty = UINT_MAX >> card->erase_shift;
	else if (mmc_card_sd(card))
		max_qty = UINT_MAX;
	else
		max_qty = UINT_MAX / card->erase_size;

	
	do {
		y = 0;
		for (x = 1; x && x <= max_qty && max_qty - x >= qty; x <<= 1) {
			timeout = mmc_erase_timeout(card, arg, qty + x);
			if (timeout > host->max_busy_timeout)
				break;
			if (timeout < last_timeout)
				break;
			last_timeout = timeout;
			y = x;
		}
		qty += y;
	} while (y);

	if (!qty)
		return 0;

	if (qty == 1)
		return 1;

	
	if (card->erase_shift)
		max_discard = --qty << card->erase_shift;
	else if (mmc_card_sd(card))
		max_discard = qty;
	else
		max_discard = --qty * card->erase_size;

	return max_discard;
}

unsigned int mmc_calc_max_discard(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	unsigned int max_discard, max_trim;

	if (!host->max_busy_timeout ||
			(host->caps2 & MMC_CAP2_MAX_DISCARD_SIZE))
		return UINT_MAX;

	if (mmc_card_mmc(card) && !(card->ext_csd.erase_group_def & 1))
		return card->pref_erase;

	max_discard = mmc_do_calc_max_discard(card, MMC_ERASE_ARG);
	if (mmc_can_trim(card)) {
		max_trim = mmc_do_calc_max_discard(card, MMC_TRIM_ARG);
		if (max_trim < max_discard)
			max_discard = max_trim;
	} else if (max_discard < card->erase_size) {
		max_discard = 0;
	}
	pr_debug("%s: calculated max. discard sectors %u for timeout %u ms\n",
		 mmc_hostname(host), max_discard, host->max_busy_timeout);
	return max_discard;
}
EXPORT_SYMBOL(mmc_calc_max_discard);

int mmc_set_blocklen(struct mmc_card *card, unsigned int blocklen)
{
	struct mmc_command cmd = {0};

	if (mmc_card_blockaddr(card) || mmc_card_ddr52(card))
		return 0;

	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.arg = blocklen;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	return mmc_wait_for_cmd(card->host, &cmd, 5);
}
EXPORT_SYMBOL(mmc_set_blocklen);

int mmc_set_blockcount(struct mmc_card *card, unsigned int blockcount,
			bool is_rel_write)
{
	struct mmc_command cmd = {0};

	cmd.opcode = MMC_SET_BLOCK_COUNT;
	cmd.arg = blockcount & 0x0000FFFF;
	if (is_rel_write)
		cmd.arg |= 1 << 31;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	return mmc_wait_for_cmd(card->host, &cmd, 5);
}
EXPORT_SYMBOL(mmc_set_blockcount);

static void mmc_hw_reset_for_init(struct mmc_host *host)
{
	if (!(host->caps & MMC_CAP_HW_RESET) || !host->ops->hw_reset)
		return;
	mmc_host_clk_hold(host);
	host->ops->hw_reset(host);
	mmc_host_clk_release(host);
}

int mmc_can_reset(struct mmc_card *card)
{
	u8 rst_n_function;

	if (mmc_card_sdio(card))
		return 0;

	if (mmc_card_mmc(card) && (card->host->caps & MMC_CAP_HW_RESET)) {
		rst_n_function = card->ext_csd.rst_n_function;
		if ((rst_n_function & EXT_CSD_RST_N_EN_MASK) !=
		    EXT_CSD_RST_N_ENABLED)
			return 0;
	}
	return 1;
}
EXPORT_SYMBOL(mmc_can_reset);

static int mmc_do_hw_reset(struct mmc_host *host, int check)
{
	struct mmc_card *card = host->card;
	int ret;

	if (!host->bus_ops->power_restore)
		return -EOPNOTSUPP;

	if (!card)
		return -EINVAL;

	if (!mmc_can_reset(card))
		return -EOPNOTSUPP;

	mmc_host_clk_hold(host);
	mmc_set_clock(host, host->f_init);

	if (mmc_card_mmc(card) && host->ops->hw_reset)
		host->ops->hw_reset(host);
	else
		mmc_power_cycle(host, host->ocr_avail);

	
	if (check) {
		struct mmc_command cmd = {0};
		int err;

		cmd.opcode = MMC_SEND_STATUS;
		if (!mmc_host_is_spi(card->host))
			cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
		err = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (!err) {
			mmc_host_clk_release(host);
			return -ENOSYS;
		}
	}

	if (mmc_host_is_spi(host)) {
		host->ios.chip_select = MMC_CS_HIGH;
		host->ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	} else {
		host->ios.chip_select = MMC_CS_DONTCARE;
		host->ios.bus_mode = MMC_BUSMODE_OPENDRAIN;
	}
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;
	mmc_set_ios(host);

	mmc_host_clk_release(host);
	mmc_claim_host(host);
	ret = host->bus_ops->power_restore(host);
	mmc_release_host(host);
	return ret;
}

int mmc_cmdq_hw_reset(struct mmc_host *host)
{
	if (!host->bus_ops->power_restore)
		return -EOPNOTSUPP;

	mmc_power_cycle(host, host->ocr_avail);
	mmc_select_voltage(host, host->card->ocr);
	return host->bus_ops->power_restore(host);
}
EXPORT_SYMBOL(mmc_cmdq_hw_reset);

int mmc_hw_reset(struct mmc_host *host)
{
	return mmc_do_hw_reset(host, 0);
}
EXPORT_SYMBOL(mmc_hw_reset);

int mmc_hw_reset_check(struct mmc_host *host)
{
	return mmc_do_hw_reset(host, 1);
}
EXPORT_SYMBOL(mmc_hw_reset_check);

static int mmc_rescan_try_freq(struct mmc_host *host, unsigned freq)
{
	host->f_init = freq;

#ifdef CONFIG_MMC_DEBUG
	pr_info("%s: %s: trying to init card at %u Hz\n",
		mmc_hostname(host), __func__, host->f_init);
#endif
	mmc_power_up(host, host->ocr_avail);

	mmc_hw_reset_for_init(host);

	sdio_reset(host);
	mmc_go_idle(host);

	mmc_send_if_cond(host, host->ocr_avail);

	
	if (!mmc_attach_sdio(host))
		return 0;
	if (!mmc_attach_sd(host))
		return 0;
	if (!mmc_attach_mmc(host))
		return 0;

	mmc_power_off(host);
	return -EIO;
}

int _mmc_detect_card_removed(struct mmc_host *host)
{
	int ret;

	if (host->caps & MMC_CAP_NONREMOVABLE)
		return 0;

	if (!host->card || mmc_card_removed(host->card))
		return 1;

	ret = host->bus_ops->alive(host);

	if (!ret && host->ops->get_cd && !host->ops->get_cd(host)) {
		mmc_detect_change(host, msecs_to_jiffies(200));
		pr_debug("%s: card removed too slowly\n", mmc_hostname(host));
	}

	if (ret) {
		mmc_card_set_removed(host->card);
		pr_debug("%s: card remove detected\n", mmc_hostname(host));
	}

	return ret;
}

int mmc_detect_card_removed(struct mmc_host *host)
{
	struct mmc_card *card = host->card;
	int ret;

	WARN_ON(!host->claimed);

	if (!card)
		return 1;

	ret = mmc_card_removed(card);
	if (!host->detect_change && !(host->caps & MMC_CAP_NEEDS_POLL))
		return ret;

	host->detect_change = 0;
	if (!ret) {
		ret = _mmc_detect_card_removed(host);
		if (ret && (host->caps & MMC_CAP_NEEDS_POLL)) {
			cancel_delayed_work(&host->detect);
			_mmc_detect_change(host, 0, false);
		}
	}

	return ret;
}
EXPORT_SYMBOL(mmc_detect_card_removed);

void mmc_rescan(struct work_struct *work)
{
	unsigned long flags;
	struct mmc_host *host =
		container_of(work, struct mmc_host, detect.work);

	pr_info("%s: %s rescan_disable %d nonremovable %d rescan_entered %d\n",
		mmc_hostname(host), __func__, host->rescan_disable,
		(host->caps & MMC_CAP_NONREMOVABLE) ? 1 : 0, host->rescan_entered);

	if (host->card && host->card->force_remove) {
		pr_info("%s: %s remove card (removed_cnt = %d)\n", mmc_hostname(host),
			__func__, host->removed_cnt);
		mmc_bus_get(host);
		if (host->bus_ops && host->bus_ops->remove)
			host->bus_ops->remove(host);
		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
		mmc_bus_put(host);
		if (host->ops->get_cd && (host->ops->get_cd(host) == 0
			|| host->removed_cnt > MMC_DETECT_RETRIES)) {
			pr_info("%s : SD status was (%d), or rescan up to limit (%d)\n",
				mmc_hostname(host), host->ops->get_cd(host),
				host->removed_cnt);
			return;
		} else if ((host->caps & MMC_CAP_NONREMOVABLE) &&
			(host->removed_cnt > MMC_DETECT_RETRIES)) {
			pr_info("%s : rescan up to limit (%d)\n",
				mmc_hostname(host),	host->removed_cnt);
			return;
		}
	}
	if (host->trigger_card_event && host->ops->card_event) {
		host->ops->card_event(host);
		host->trigger_card_event = false;
	}

	spin_lock_irqsave(&host->lock, flags);
	if (host->rescan_disable) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&host->lock, flags);

#if 0
	
	if ((host->caps & MMC_CAP_NONREMOVABLE) && host->rescan_entered)
		return;
#endif
	host->rescan_entered = 1;

	mmc_bus_get(host);

	if (host->bus_ops && !host->bus_dead
	    && !(host->caps & MMC_CAP_NONREMOVABLE))
		host->bus_ops->detect(host);

	host->detect_change = 0;

	mmc_bus_put(host);
	mmc_bus_get(host);

	
	if (host->bus_ops != NULL) {
		mmc_bus_put(host);
		goto out;
	}

	mmc_bus_put(host);

	if (!(host->caps & MMC_CAP_NONREMOVABLE) && host->ops->get_cd &&
			(host->ops->get_cd(host) == 0 || host->removed_cnt > MMC_DETECT_RETRIES)) {
		mmc_claim_host(host);
		mmc_power_off(host);
		mmc_release_host(host);
		pr_info("%s : SD status was (%d), or rescan up to limit2 (%d)\n",
			mmc_hostname(host), host->ops->get_cd(host),
			host->removed_cnt);
		goto out;
	}

	mmc_claim_host(host);
	(void) mmc_rescan_try_freq(host, host->f_min);
	mmc_release_host(host);

 out:
	if (host->caps & MMC_CAP_NEEDS_POLL)
		mmc_schedule_delayed_work(&host->detect, HZ);
}

void mmc_start_host(struct mmc_host *host)
{
	mmc_claim_host(host);
	host->f_init = max(freqs[0], host->f_min);
	host->rescan_disable = 0;
	host->ios.power_mode = MMC_POWER_UNDEFINED;
	if (host->caps2 & MMC_CAP2_NO_PRESCAN_POWERUP)
		mmc_power_off(host);
	else
		mmc_power_up(host, host->ocr_avail);
	mmc_gpiod_request_cd_irq(host);
	mmc_release_host(host);
	_mmc_detect_change(host, 0, false);
}

void mmc_stop_host(struct mmc_host *host)
{
#ifdef CONFIG_MMC_DEBUG
	unsigned long flags;
	spin_lock_irqsave(&host->lock, flags);
	host->removed = 1;
	spin_unlock_irqrestore(&host->lock, flags);
#endif
	if (host->slot.cd_irq >= 0)
		disable_irq(host->slot.cd_irq);

	host->rescan_disable = 1;
	cancel_delayed_work_sync(&host->detect);
	mmc_flush_scheduled_work();

	
	host->pm_flags = 0;

	mmc_bus_get(host);
	if (host->bus_ops && !host->bus_dead) {
		
		host->bus_ops->remove(host);
		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
		mmc_bus_put(host);
		return;
	}
	mmc_bus_put(host);

	BUG_ON(host->card);

	mmc_power_off(host);
}

int mmc_power_save_host(struct mmc_host *host)
{
	int ret = 0;

#ifdef CONFIG_MMC_DEBUG
	pr_info("%s: %s: powering down\n", mmc_hostname(host), __func__);
#endif

	mmc_bus_get(host);

	if (!host->bus_ops || host->bus_dead) {
		mmc_bus_put(host);
		return -EINVAL;
	}

	if (host->bus_ops->power_save)
		ret = host->bus_ops->power_save(host);

	mmc_bus_put(host);

	mmc_power_off(host);

	return ret;
}
EXPORT_SYMBOL(mmc_power_save_host);

int mmc_power_restore_host(struct mmc_host *host)
{
	int ret;

#ifdef CONFIG_MMC_DEBUG
	pr_info("%s: %s: powering up\n", mmc_hostname(host), __func__);
#endif

	mmc_bus_get(host);

	if (!host->bus_ops || host->bus_dead) {
		mmc_bus_put(host);
		return -EINVAL;
	}

	mmc_power_up(host, host->card->ocr);
	mmc_claim_host(host);
	ret = host->bus_ops->power_restore(host);
	mmc_release_host(host);

	mmc_bus_put(host);

	return ret;
}
EXPORT_SYMBOL(mmc_power_restore_host);

int mmc_cache_barrier(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	int err = 0;

	if (!card->ext_csd.cache_ctrl ||
	     (card->quirks & MMC_QUIRK_CACHE_DISABLE))
		goto out;

	if (!mmc_card_mmc(card))
		goto out;

	if (!card->ext_csd.barrier_en)
		return -ENOTSUPP;

	err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_FLUSH_CACHE, 0x2, 0);
	if (err)
		pr_err("%s: cache barrier error %d\n",
				mmc_hostname(host), err);
out:
	return err;
}
EXPORT_SYMBOL(mmc_cache_barrier);

int mmc_flush_cache(struct mmc_card *card)
{
	int err = 0;

	if (mmc_card_mmc(card) &&
			(card->ext_csd.cache_size > 0) &&
			(card->ext_csd.cache_ctrl & 1) &&
			(!(card->quirks & MMC_QUIRK_CACHE_DISABLE))) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_FLUSH_CACHE, 1, 0);
		if (err == -ETIMEDOUT) {
			pr_err("%s: cache flush timeout\n",
					mmc_hostname(card->host));
			err = mmc_interrupt_hpi(card);
			if (err) {
				pr_err("%s: mmc_interrupt_hpi() failed (%d)\n",
						mmc_hostname(card->host), err);
				err = -ENODEV;
			}
		} else if (err) {
			pr_err("%s: cache flush error %d\n",
					mmc_hostname(card->host), err);
		}
	}

	return err;
}
EXPORT_SYMBOL(mmc_flush_cache);

#ifdef CONFIG_PM

int mmc_pm_notify(struct notifier_block *notify_block,
					unsigned long mode, void *unused)
{
	struct mmc_host *host = container_of(
		notify_block, struct mmc_host, pm_notify);
	unsigned long flags;
	int err = 0;

	switch (mode) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
	case PM_RESTORE_PREPARE:
		spin_lock_irqsave(&host->lock, flags);
		host->rescan_disable = 1;
		host->retry_disable = 1;
		spin_unlock_irqrestore(&host->lock, flags);
		cancel_delayed_work_sync(&host->detect);
		host->retry_disable = 0;

		if (!host->bus_ops)
			break;

		
		if (host->bus_ops->pre_suspend)
			err = host->bus_ops->pre_suspend(host);
		if (!err)
			break;

		
		host->bus_ops->remove(host);
		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
		host->pm_flags = 0;
		break;

	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:

		spin_lock_irqsave(&host->lock, flags);
		host->rescan_disable = 0;
		if (mmc_bus_manual_resume(host)) {
			spin_unlock_irqrestore(&host->lock, flags);
			break;
		}
		spin_unlock_irqrestore(&host->lock, flags);
		_mmc_detect_change(host, 0, false);

	}

	return 0;
}
#endif

void mmc_prepare_mrq(struct mmc_card *card,
	struct mmc_request *mrq, struct scatterlist *sg, unsigned sg_len,
	unsigned dev_addr, unsigned blocks, unsigned blksz, int write)
{
	BUG_ON(!mrq || !mrq->cmd || !mrq->data || !mrq->stop);

	if (blocks > 1) {
		mrq->cmd->opcode = write ?
			MMC_WRITE_MULTIPLE_BLOCK : MMC_READ_MULTIPLE_BLOCK;
	} else {
		mrq->cmd->opcode = write ?
			MMC_WRITE_BLOCK : MMC_READ_SINGLE_BLOCK;
	}

	mrq->cmd->arg = dev_addr;
	if (!mmc_card_blockaddr(card))
		mrq->cmd->arg <<= 9;

	mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	if (blocks == 1)
		mrq->stop = NULL;
	else {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	mrq->data->blksz = blksz;
	mrq->data->blocks = blocks;
	mrq->data->flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	mrq->data->sg = sg;
	mrq->data->sg_len = sg_len;

	mmc_set_data_timeout(mrq->data, card);
}
EXPORT_SYMBOL(mmc_prepare_mrq);

static int mmc_busy(struct mmc_command *cmd)
{
	return !(cmd->resp[0] & R1_READY_FOR_DATA) ||
		(R1_CURRENT_STATE(cmd->resp[0]) == R1_STATE_PRG);
}

int mmc_wait_busy(struct mmc_card *card)
{
	int ret, busy = 0;
	struct mmc_command cmd = {0};

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_SEND_STATUS;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;

	do {
		ret = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (ret)
			break;

		if (!busy && mmc_busy(&cmd)) {
			busy = 1;
			if (card->host->caps & MMC_CAP_WAIT_WHILE_BUSY) {
				pr_warn("%s: Warning: Host did not "
					"wait for busy state to end.\n",
					mmc_hostname(card->host));
			}
		}

	} while (mmc_busy(&cmd));

	return ret;
}
EXPORT_SYMBOL(mmc_wait_busy);

int mmc_check_result(struct mmc_request *mrq)
{
	int ret;

	BUG_ON(!mrq || !mrq->cmd || !mrq->data);

	ret = 0;

	if (!ret && mrq->cmd->error)
		ret = mrq->cmd->error;
	if (!ret && mrq->data->error)
		ret = mrq->data->error;
	if (!ret && mrq->stop && mrq->stop->error)
		ret = mrq->stop->error;
	if (!ret && mrq->data->bytes_xfered !=
		mrq->data->blocks * mrq->data->blksz)
		ret = -EPERM;

	return ret;
}
EXPORT_SYMBOL(mmc_check_result);

int mmc_simple_transfer(struct mmc_card *card,
	struct scatterlist *sg, unsigned sg_len, unsigned dev_addr,
	unsigned blocks, unsigned blksz, int write)
{
	struct mmc_request mrq = {0};
	struct mmc_command cmd = {0};
	struct mmc_command stop = {0};
	struct mmc_data data = {0};

	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = &stop;

	mmc_prepare_mrq(card, &mrq, sg, sg_len, dev_addr,
		blocks, blksz, write);

	mmc_wait_for_req(card->host, &mrq);

	mmc_wait_busy(card);

	return mmc_check_result(&mrq);
}
EXPORT_SYMBOL(mmc_simple_transfer);


void mmc_init_context_info(struct mmc_host *host)
{
	spin_lock_init(&host->context_info.lock);
	host->context_info.is_new_req = false;
	host->context_info.is_done_rcv = false;
	host->context_info.is_waiting_last_req = false;
	init_waitqueue_head(&host->context_info.wait);
}

#ifdef CONFIG_MMC_EMBEDDED_SDIO
void mmc_set_embedded_sdio_data(struct mmc_host *host,
				struct sdio_cis *cis,
				struct sdio_cccr *cccr,
				struct sdio_embedded_func *funcs,
				int num_funcs)
{
	host->embedded_sdio_data.cis = cis;
	host->embedded_sdio_data.cccr = cccr;
	host->embedded_sdio_data.funcs = funcs;
	host->embedded_sdio_data.num_funcs = num_funcs;
}

EXPORT_SYMBOL(mmc_set_embedded_sdio_data);
#endif

static int __init mmc_init(void)
{
	int ret;

	workqueue = alloc_ordered_workqueue("kmmcd", 0);
	if (!workqueue)
		return -ENOMEM;

	stats_workqueue = create_singlethread_workqueue("mmc_stats");
	if (!stats_workqueue)
		return -ENOMEM;

	if (get_tamper_sf() == 1)
		stats_interval = MMC_STATS_LOG_INTERVAL;

	ret = mmc_register_bus();
	if (ret)
		goto destroy_workqueue;

	ret = mmc_register_host_class();
	if (ret)
		goto unregister_bus;

	ret = sdio_register_bus();
	if (ret)
		goto unregister_host_class;

	spin_lock_init(&iolist_lock);
	return 0;

unregister_host_class:
	mmc_unregister_host_class();
unregister_bus:
	mmc_unregister_bus();
destroy_workqueue:
	destroy_workqueue(workqueue);
	if (stats_workqueue)
		destroy_workqueue(stats_workqueue);

	return ret;
}

static void __exit mmc_exit(void)
{
	sdio_unregister_bus();
	mmc_unregister_host_class();
	mmc_unregister_bus();
	destroy_workqueue(workqueue);
}

subsys_initcall(mmc_init);
module_exit(mmc_exit);

MODULE_LICENSE("GPL");
