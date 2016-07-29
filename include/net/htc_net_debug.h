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
#ifndef _HTC_NET_DEBUG_H
#define _HTC_NET_DEBUG_H

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/sched.h>
#include <linux/time.h>

#if 0
#include <linux/ipc_logging.h>
#endif

struct net_timestamp {
   struct timespec ts;
};

extern int htc_net_debug_dump;
extern int htc_net_debug_print;
extern int htc_net_debug_enable;
extern void net_dbg_log_event(const char * event, ...);
extern void net_get_kernel_timestamp(struct net_timestamp *ts);

#define NET_DEBUG(fmt, args...) \
do{ \
	if(htc_net_debug_enable) \
		net_dbg_log_event("[NET]" fmt, args); \
} while(0)

#endif 
