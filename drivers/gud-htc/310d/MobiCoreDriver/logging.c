/*
 * Copyright (c) 2013-2015 TRUSTONIC LIMITED
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/debugfs.h>

#include "main.h"
#include "fastcall.h"
#include "logging.h"

#define MC_LOG_VERSION			2

#define LOG_BUF_ORDER			6

#define LOG_LINE_SIZE			256

#define LOG_TYPE_MASK			(0x0007)
#define LOG_TYPE_CHAR			0
#define LOG_TYPE_INTEGER		1

#define LOG_LENGTH_MASK			(0x00F8)
#define LOG_LENGTH_SHIFT		3

#define LOG_EOL				(0x0100)
#define LOG_INTEGER_DECIMAL		(0x0200)
#define LOG_INTEGER_SIGNED		(0x0400)

struct mc_logmsg {
	u16	ctrl;		
	u16	source;		
	u32	log_data;	
};

struct mc_trace_buf {
	u32	version;	
	u32	length;		
	u32	head;		
	u8	buff[];		
};

static struct logging_ctx {
	struct work_struct work;
	union {
		struct mc_trace_buf *trace_buf;	
		unsigned long trace_page;
	};
	bool	buffer_is_shared;	
	u32	tail;			
	u32	line_len;		
	int	thread_err;
	u16	prev_source;		
	char	line[LOG_LINE_SIZE];	
	u32	enabled;		
	bool	dead;
} log_ctx;

static inline void log_eol(u16 source)
{
	if (!strnlen(log_ctx.line, LOG_LINE_SIZE)) {
		
		log_ctx.line_len = 0;
		return;
	}

	if (log_ctx.prev_source)
		
		dev_info(g_ctx.mcd, "%03x|%s\n", log_ctx.prev_source,
			 log_ctx.line);
	else
		
		dev_info(g_ctx.mcd, "%s\n", log_ctx.line);

	log_ctx.line_len = 0;
	log_ctx.line[0] = 0;
}

static inline void log_char(char ch, u16 source)
{
	if (ch == '\n' || ch == '\r') {
		log_eol(source);
		return;
	}

	if ((log_ctx.line_len >= (LOG_LINE_SIZE - 1)) ||
	    (source != log_ctx.prev_source))
		log_eol(source);

	log_ctx.line[log_ctx.line_len++] = ch;
	log_ctx.line[log_ctx.line_len] = 0;
	log_ctx.prev_source = source;
}

static inline void log_string(u32 ch, u16 source)
{
	while (ch) {
		log_char(ch & 0xFF, source);
		ch >>= 8;
	}
}

static inline void log_number(u32 format, u32 value, u16 source)
{
	int width = (format & LOG_LENGTH_MASK) >> LOG_LENGTH_SHIFT;
	char fmt[16];
	char buffer[32];
	const char *reader = buffer;

	if (format & LOG_INTEGER_DECIMAL)
		if (format & LOG_INTEGER_SIGNED)
			snprintf(fmt, sizeof(fmt), "%%%ud", width);
		else
			snprintf(fmt, sizeof(fmt), "%%%uu", width);
	else
		snprintf(fmt, sizeof(fmt), "%%0%ux", width);

	snprintf(buffer, sizeof(buffer), fmt, value);
	while (*reader)
		log_char(*reader++, source);
}

static inline int log_msg(void *data)
{
	struct mc_logmsg *msg = (struct mc_logmsg *)data;
	int log_type = msg->ctrl & LOG_TYPE_MASK;

	switch (log_type) {
	case LOG_TYPE_CHAR:
		log_string(msg->log_data, msg->source);
		break;
	case LOG_TYPE_INTEGER:
		log_number(msg->ctrl, msg->log_data, msg->source);
		break;
	}
	if (msg->ctrl & LOG_EOL)
		log_eol(msg->source);

	return sizeof(*msg);
}

static void log_worker(struct work_struct *work)
{
	static DEFINE_MUTEX(local_mutex);

	mutex_lock(&local_mutex);
	while (log_ctx.trace_buf->head != log_ctx.tail) {
		if (log_ctx.trace_buf->version != MC_LOG_VERSION) {
			mc_dev_err("Bad log data v%d (exp. v%d), stop\n",
				   log_ctx.trace_buf->version, MC_LOG_VERSION);
			log_ctx.dead = true;
			break;
		}

		log_ctx.tail += log_msg(&log_ctx.trace_buf->buff[log_ctx.tail]);
		
		if ((log_ctx.tail + sizeof(struct mc_logmsg)) >
						log_ctx.trace_buf->length)
			log_ctx.tail = 0;
	}
	mutex_unlock(&local_mutex);
}

void mc_logging_run(void)
{
	if (log_ctx.enabled && !log_ctx.dead &&
	    (log_ctx.trace_buf->head != log_ctx.tail))
		schedule_work(&log_ctx.work);
}

int mc_logging_start(void)
{
	int ret = mc_fc_mem_trace(virt_to_phys((void *)(log_ctx.trace_page)),
				  BIT(LOG_BUF_ORDER) * PAGE_SIZE);

	if (ret) {
		mc_dev_err("shared traces setup failed\n");
		return ret;
	}

	log_ctx.buffer_is_shared = true;
	mc_dev_devel("fc_log version %u\n", log_ctx.trace_buf->version);
	mc_logging_run();
	return 0;
}

void mc_logging_stop(void)
{
	if (!mc_fc_mem_trace(0, 0))
		log_ctx.buffer_is_shared = false;

	mc_logging_run();
	flush_work(&log_ctx.work);
}

int mc_logging_init(void)
{
	log_ctx.trace_page = __get_free_pages(GFP_KERNEL | __GFP_ZERO,
					      LOG_BUF_ORDER);
	if (!log_ctx.trace_page)
		return -ENOMEM;

	INIT_WORK(&log_ctx.work, log_worker);
	log_ctx.enabled = true;
	debugfs_create_bool("swd_debug", 0600, g_ctx.debug_dir,
			    &log_ctx.enabled);
	return 0;
}

void mc_logging_exit(void)
{
	if (!log_ctx.buffer_is_shared)
		free_pages(log_ctx.trace_page, LOG_BUF_ORDER);
	else
		mc_dev_err("log buffer unregister not supported\n");
}
