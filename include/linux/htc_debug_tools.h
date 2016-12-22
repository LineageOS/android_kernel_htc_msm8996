/* Copyright (c) 2013, HTC Corporation. All rights reserved.
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
#ifndef __HTC_DEBUG_TOOLS_H__
#define __HTC_DEBUG_TOOLS_H__

#include <linux/types.h>

/* n.b.:
 * 1. sched_clock is not irq safe
 * 2. 32 bit: overflows every 4,294,967,296 msecs
 */
unsigned long htc_debug_get_sched_clock_ms(void);

#if defined(CONFIG_HTC_DEBUG_BOOTLOADER_LOG)
ssize_t bldr_log_read_once(char __user *userbuf, ssize_t klog_size);
ssize_t bldr_last_log_read_once(char __user *userbuf, ssize_t klog_size);
ssize_t bldr_log_read(const void *lastk_buf, ssize_t lastk_size, char __user *userbuf,
		size_t count, loff_t *ppos);
int bldr_log_init(void);
void bldr_log_release(void);
#endif

#endif /* __HTC_DEBUG_TOOLS_H__ */
