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

#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/stringify.h>
#include <linux/version.h>

#include "public/mc_user.h"

#include "main.h"
#include "fastcall.h"
#include "logging.h"
#include "mcp.h"
#include "scheduler.h"

#define SCHEDULING_FREQ		5   

static struct sched_ctx {
	struct task_struct	*thread;
	bool			thread_run;
	struct completion	idle_complete;	
	struct completion	sleep_complete;	
	struct mutex		sleep_mutex;	
	struct mutex		request_mutex;	
	
	enum {
		NONE,		
		YIELD,		
		NSIQ,		
		SUSPEND,	
		RESUME,		
	}			request;
	bool			suspended;
} sched_ctx;

static int mc_scheduler_command(int command)
{
	if (IS_ERR_OR_NULL(sched_ctx.thread))
		return -EFAULT;

	mutex_lock(&sched_ctx.request_mutex);
	if (sched_ctx.request < command) {
		sched_ctx.request = command;
		complete(&sched_ctx.idle_complete);
	}

	mutex_unlock(&sched_ctx.request_mutex);
	return 0;
}

static int mc_scheduler_pm_command(int command)
{
	int ret = -EPERM;

	if (IS_ERR_OR_NULL(sched_ctx.thread))
		return -EFAULT;

	mutex_lock(&sched_ctx.sleep_mutex);

	
	mc_scheduler_command(command);

	
	wait_for_completion(&sched_ctx.sleep_complete);
	mutex_lock(&sched_ctx.request_mutex);
	if (command == SUSPEND) {
		if (sched_ctx.suspended)
			ret = 0;
	} else {
		if (!sched_ctx.suspended)
			ret = 0;
	}

	mutex_unlock(&sched_ctx.request_mutex);

	mutex_unlock(&sched_ctx.sleep_mutex);
	return ret;
}

static int mc_dev_command(enum mcp_scheduler_commands command)
{
	switch (command) {
	case MCP_YIELD:
		return mc_scheduler_command(YIELD);
	case MCP_NSIQ:
		return mc_scheduler_command(NSIQ);
	}

	return -EINVAL;
}

int mc_scheduler_suspend(void)
{
	return mc_scheduler_pm_command(SUSPEND);
}

int mc_scheduler_resume(void)
{
	return mc_scheduler_pm_command(RESUME);
}

static int tee_scheduler(void *arg)
{
	int timeslice = 0;	
	int ret = 0;

	while (1) {
		s32 timeout_ms = -1;
		bool pm_request = false;

		if (sched_ctx.suspended || mcp_get_idle_timeout(&timeout_ms)) {
			
			if (!timeout_ms)
				mc_scheduler_command(NSIQ);
			else if (timeout_ms < 0)
				wait_for_completion(&sched_ctx.idle_complete);
			else if (!wait_for_completion_timeout(
					&sched_ctx.idle_complete,
					msecs_to_jiffies(timeout_ms)))
				
				mc_scheduler_command(NSIQ);
		}

		if (kthread_should_stop() || !sched_ctx.thread_run)
			break;

		
		mutex_lock(&sched_ctx.request_mutex);
		if (sched_ctx.request == YIELD)
			
			timeslice++;
		else if (sched_ctx.request >= NSIQ) {
			
			timeslice = 0;
			if (sched_ctx.request == SUSPEND) {
				mcp_suspend();
				pm_request = true;
			} else if (sched_ctx.request == RESUME) {
				mcp_resume();
				pm_request = true;
			}
		}

		if (g_ctx.f_time)
			mcp_update_time();

		sched_ctx.request = NONE;
		mutex_unlock(&sched_ctx.request_mutex);

		
		mcp_reset_idle_timeout();
		if (timeslice--) {
			
			ret = mc_fc_yield();
		} else {
			timeslice = SCHEDULING_FREQ;
			
			ret = mc_fc_nsiq();
		}

		
		mc_logging_run();
		if (ret)
			break;

		
		mutex_lock(&sched_ctx.request_mutex);
		if (pm_request) {
			sched_ctx.suspended = mcp_suspended();
			complete(&sched_ctx.sleep_complete);
		}

		mutex_unlock(&sched_ctx.request_mutex);

		
		if (mcp_notifications_flush())
			complete(&sched_ctx.idle_complete);
	}

	mc_dev_devel("exit, ret is %d\n", ret);
	return ret;
}

int mc_scheduler_start(void)
{
	sched_ctx.thread_run = true;
	sched_ctx.thread = kthread_run(tee_scheduler, NULL, "tee_scheduler");
	if (IS_ERR(sched_ctx.thread)) {
		mc_dev_err("tee_scheduler thread creation failed\n");
		return PTR_ERR(sched_ctx.thread);
	}

	mcp_register_scheduler(mc_dev_command);
	complete(&sched_ctx.idle_complete);
	return 0;
}

void mc_scheduler_stop(void)
{
	mcp_register_scheduler(NULL);
	sched_ctx.thread_run = false;
	complete(&sched_ctx.idle_complete);
	kthread_stop(sched_ctx.thread);
}

int mc_scheduler_init(void)
{
	init_completion(&sched_ctx.idle_complete);
	init_completion(&sched_ctx.sleep_complete);
	mutex_init(&sched_ctx.sleep_mutex);
	mutex_init(&sched_ctx.request_mutex);
	return 0;
}
