/* Copyright (c) 2015, HTC Corporation. All rights reserved.
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/sched.h>
#include <net/htc_net_debug.h>
#include <linux/slab.h>

#if 0
#include <linux/ipc_logging.h>
#endif

int htc_net_debug_enable = 1;
int htc_net_debug_dump = 1;
int htc_net_debug_print = 0;

#define DBG_MSG_LEN   128UL
#define DBG_MAX_MSG   256UL
#define DBG_ONELINE_MAX_MSG   30UL
#define TIME_BUF_LEN  20

static int htc_net_debug_dump_lines = DBG_MAX_MSG;

module_param_named(htc_net_debug_enable, htc_net_debug_enable,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(htc_net_debug_dump, htc_net_debug_dump,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(htc_net_debug_dump_lines, htc_net_debug_dump_lines,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(htc_net_debug_print, htc_net_debug_print,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

#if 0
void *net_ipc_log_txt = 0;
#endif

#define NET_IPC_LOG_PAGES 5

static struct {
	char (buf[DBG_MAX_MSG])[DBG_MSG_LEN]; 
	unsigned idx;
	rwlock_t lock;
} net_debug = {
	.idx = 0,
	.lock = __RW_LOCK_UNLOCKED(lock)
};

static struct {
	char (buf[DBG_ONELINE_MAX_MSG])[DBG_MSG_LEN]; 
	unsigned idx;
	rwlock_t lock;
} net_debug_oneline = {
	.idx = 0,
	.lock = __RW_LOCK_UNLOCKED(lock)
};

#if 0
static char net_stack_klog[PAGE_SIZE];
#endif

static void net_dbg_inc(unsigned *idx)
{
	*idx = (*idx + 1) & (DBG_MAX_MSG-1);
}

void net_get_kernel_timestamp(struct net_timestamp *timer)
{
    get_monotonic_boottime(&timer->ts);
}
EXPORT_SYMBOL(net_get_kernel_timestamp);

static char *net_get_timestamp(char *tbuf)
{
	unsigned long long t;
	unsigned long nanosec_rem;

	t = cpu_clock(smp_processor_id());
	nanosec_rem = do_div(t, 1000000000)/1000;
	scnprintf(tbuf, TIME_BUF_LEN, "[%5lu.%06lu] ", (unsigned long)t,
		nanosec_rem);
	return tbuf;
}

void net_events_print(void)
{
	unsigned long	flags;
	unsigned	i;
	unsigned lines = 0;

	pr_info("### Show NET Log Start ###\n");

	read_lock_irqsave(&net_debug.lock, flags);

	i = net_debug.idx;
	for (net_dbg_inc(&i); i != net_debug.idx; net_dbg_inc(&i)) {
		if (!strnlen(net_debug.buf[i], DBG_MSG_LEN))
			continue;
		pr_info("%s", net_debug.buf[i]);
		lines++;
		if ( lines > htc_net_debug_dump_lines )
			break;
	}

	read_unlock_irqrestore(&net_debug.lock, flags);

	pr_info("### Show NET Log End ###\n");
}

void net_dumplog(void)
{
#if 0
	int ret = 0;
#endif

	if (!htc_net_debug_enable) {
		pr_info("%s: htc_net_debug_enable=[%d]\n", __func__, htc_net_debug_enable);
		return;
	}

	if (!htc_net_debug_dump) {
		pr_info("%s: htc_net_debug_dump=[%d]\n", __func__, htc_net_debug_dump);
		return;
	}

#if 0
	if (!net_ipc_log_txt) {
		pr_info("%s: net_ipc_log_txt = NULL\n", __func__);
		net_events_print();
		return;
	}
#endif

	

	net_events_print();
#if 0
	do{
		memset(net_stack_klog, 0x0, PAGE_SIZE);
		ret = ipc_log_extract(net_ipc_log_txt, net_stack_klog, PAGE_SIZE);
		if ( ret >= 0 ){
			pr_info("%s\n", net_stack_klog);
		}

	}while ( ret > 0 );
#endif
	

}
EXPORT_SYMBOL(net_dumplog);

void net_dbg_log_event(const char * event, ...)
{
	unsigned long flags;
	char tbuf[TIME_BUF_LEN];
	char dbg_buff[DBG_MSG_LEN];
	va_list arg_list;

	if ( !htc_net_debug_enable ) {
		return;
	}

	va_start(arg_list, event);
	vsnprintf(dbg_buff,
			      DBG_MSG_LEN, event, arg_list);
	va_end(arg_list);

	write_lock_irqsave(&net_debug.lock, flags);

	scnprintf(net_debug.buf[net_debug.idx], DBG_MSG_LEN,
		"%s%s", net_get_timestamp(tbuf), dbg_buff);

	net_dbg_inc(&net_debug.idx);

	if (htc_net_debug_print)
		pr_info("%s", dbg_buff);
	write_unlock_irqrestore(&net_debug.lock, flags);

	return;

}
EXPORT_SYMBOL(net_dbg_log_event);

static int net_events_show(struct seq_file *s, void *unused)
{
	unsigned long	flags;
	unsigned	i;

	read_lock_irqsave(&net_debug.lock, flags);

	i = net_debug.idx;
	for (net_dbg_inc(&i); i != net_debug.idx; net_dbg_inc(&i)) {
		if (!strnlen(net_debug.buf[i], DBG_MSG_LEN))
			continue;
		seq_printf(s, "%s", net_debug.buf[i]);
	}

	read_unlock_irqrestore(&net_debug.lock, flags);

	return 0;
}

static int net_events_open(struct inode *inode, struct file *f)
{
	return single_open(f, net_events_show, inode->i_private);
}


void net_dbg_log_event_oneline(int idx, const char * event, ...)
{
	unsigned long flags;
	char tbuf[TIME_BUF_LEN];
	char dbg_buff[DBG_MSG_LEN];
	va_list arg_list;

	if ( !htc_net_debug_enable ||
		idx >=  DBG_ONELINE_MAX_MSG ||
		idx < 0 ||
		net_debug_oneline.buf[idx] == NULL)
	{
		return; 
	}

	va_start(arg_list, event);
	vsnprintf(dbg_buff, DBG_MSG_LEN, event, arg_list);
	va_end(arg_list);

	write_lock_irqsave(&net_debug_oneline.lock, flags);

	scnprintf(net_debug_oneline.buf[idx], DBG_MSG_LEN,
		"%s%s", net_get_timestamp(tbuf), dbg_buff);

	if (htc_net_debug_print)
		pr_info("%s\n", dbg_buff);
	write_unlock_irqrestore(&net_debug_oneline.lock, flags);

	return;

}
EXPORT_SYMBOL(net_dbg_log_event_oneline);

int net_dbg_get_free_log_event_oneline(void)
{
	unsigned long	flags;
	int ret = -1;

	write_lock_irqsave(&net_debug_oneline.lock, flags);
	if (net_debug_oneline.idx < DBG_ONELINE_MAX_MSG) {
		ret = net_debug_oneline.idx;
		net_debug_oneline.idx++;
	}
	write_unlock_irqrestore(&net_debug_oneline.lock, flags);

	return ret;
}
EXPORT_SYMBOL(net_dbg_get_free_log_event_oneline);

static int net_events_oneline_show(struct seq_file *s, void *unused)
{
	unsigned long	flags;
	unsigned	i;

	read_lock_irqsave(&net_debug_oneline.lock, flags);
	for (i = 0; i < net_debug_oneline.idx; i++) {
		seq_printf(s, "%s\n", net_debug_oneline.buf[i]);
	}
	read_unlock_irqrestore(&net_debug_oneline.lock, flags);

	return 0;
}


static int net_events_oneline_open(struct inode *inode, struct file *f)
{
	return single_open(f, net_events_oneline_show, inode->i_private);
}


const struct file_operations net_dbg_op = {
	.open = net_events_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


const struct file_operations net_dbg_oneline_op = {
	.open = net_events_oneline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init htc_net_debug_init(void)
{
#ifdef CONFIG_DEBUG_FS
	struct dentry *dent;

	dent = debugfs_create_dir("htc_net", 0);
	if (!IS_ERR(dent)) {
		debugfs_create_file("dumplog", S_IRUGO, dent, NULL, &net_dbg_op);
		debugfs_create_file("dumplog_oneline", S_IRUGO, dent, NULL, &net_dbg_oneline_op);
	}
#endif
   pr_info("[NET]%s start.\n", __func__);

#if 0
   net_ipc_log_txt = ipc_log_context_create(NET_IPC_LOG_PAGES, "net_debug", 0);

   pr_info("[NET]%s: net_ipc_log_txt:%p.\n", __func__, net_ipc_log_txt);
	if (!net_ipc_log_txt) {
		pr_err("%s : unable to create IPC Logging Context", __func__);
	}
#endif
   return 0;
}

module_init(htc_net_debug_init);

MODULE_DESCRIPTION("HTC Net Debug");
MODULE_LICENSE("GPL v2");
