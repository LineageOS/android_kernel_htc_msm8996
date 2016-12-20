/*
 * Copyright (c) 2015-2016 TRUSTONIC LIMITED
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
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/input/synaptics_dsx_v2.h>

#include <linux/ion.h>
#include <linux/msm_ion.h>
#include <linux/clk.h>

#include "dciTui.h"
#include "tui-hal.h"
#include "tlcTui.h"
#include "mobicore_driver_api.h"

uint32_t width;
uint32_t height;
uint32_t stride;

#define WB_USER_ALLOC

struct ion_client *g_iclnt;

struct clk *core_clk;
struct clk *iface_clk;

struct task_struct *irq_fwd_thread;

static char *touchscreen_path =
	"/devices/soc/75ba000.i2c/i2c-12/12-0020/input/input2";
DECLARE_COMPLETION(swd_completed);

uint64_t g_ion_phys[MAX_BUFFER_NUMBER];
uint32_t g_ion_size[MAX_BUFFER_NUMBER];

uint32_t hal_tui_alloc(struct tui_alloc_buffer_t *allocbuffer,
		       size_t allocsize,
		       uint32_t number)
{
	uint32_t i, ret = TUI_DCI_ERR_INTERNAL_ERROR;
	size_t aligned_alloc_size = 0;

	
	aligned_alloc_size = ALIGN(allocsize, SZ_1M);

	pr_debug("%s(%d): Allocating %d buffers of size %zu aligned_size=%zu\n",
		 __func__, __LINE__, number, allocsize, aligned_alloc_size);

	if (number != MAX_BUFFER_NUMBER) {
		pr_debug("%s(%d): Unexpected number of buffers requested (allocating only the work buffer)\n",
			 __func__, __LINE__);
		return TUI_DCI_ERR_INTERNAL_ERROR;
	}

	if (allocsize == 0) {
		pr_debug("%s(%d): Nothing to allocate\n", __func__, __LINE__);
		return TUI_DCI_OK;
	}

	if (!allocbuffer) {
		pr_debug("%s(%d): allocbuffer is null\n", __func__, __LINE__);
		return TUI_DCI_ERR_INTERNAL_ERROR;
	}

	
	memset(g_ion_phys, 0, sizeof(g_ion_phys));
	memset(g_ion_size, 0, sizeof(g_ion_size));

	
	g_iclnt = msm_ion_client_create("tlctui_wb");

#ifndef WB_USER_ALLOC
	struct ion_handle *ihandle = { 0 };
	ion_phys_addr_t phys = -1;
	size_t size = 0;
	int ion_ret = -1;

	
	ihandle = ion_alloc(g_iclnt, allocsize, SZ_1M,
			    ION_HEAP(ION_CP_MM_HEAP_ID), ION_SECURE);
	
	if (!IS_ERR(ihandle)) {
		ion_ret = ion_phys(g_iclnt, ihandle, &phys, &size);
		if (0 != ion_ret) {
			pr_info("ERROR %s:%d ion_phys failed (ret=%i)!\n",
				__func__, __LINE__, ion_ret);
			return TUI_DCI_ERR_INTERNAL_ERROR;
		}

		pr_debug("%s: phys=%p size=%zu\n",
			 __func__, (void *)phys, size);
		g_ion_phys[ION_PHYS_WORKING_BUFFER_IDX] =
			(uint64_t)phys;
		g_ion_size[ION_PHYS_WORKING_BUFFER_IDX] =
			(uint32_t)size;
		pr_debug("%s: g_ion_phys[%i] = 0x%0llx (g_ion_size[%i]=%u)\n",
			 __func__, ION_PHYS_WORKING_BUFFER_IDX,
			 g_ion_phys[ION_PHYS_WORKING_BUFFER_IDX],
			 ION_PHYS_WORKING_BUFFER_IDX,
			 g_ion_size[ION_PHYS_WORKING_BUFFER_IDX]);

	} else {
		pr_info("ERROR %s:%d Cannot create the tlctui_wb ion client!",
			__func__, __LINE__);
		
		return TUI_DCI_ERR_INTERNAL_ERROR;
	}

	ret = send_cmd_to_user(TLC_TUI_CMD_ALLOC_FB, number-1,
			       aligned_alloc_size);
#else
	
	ret = send_cmd_to_user(TLC_TUI_CMD_ALLOC_FB, number,
			       aligned_alloc_size);
#endif
	if (TUI_DCI_OK != ret) {
		
		pr_info("ERROR %s:%d TLC_TUI_CMD_ALLOC_FB failed with (%d)",
			__func__, __LINE__, ret);
		return TUI_DCI_ERR_INTERNAL_ERROR;
	}

	
	for (i = 0; i < dci->cmd_nwd.payload.alloc_data.num_of_buff; i++) {
		pr_debug("%s: g_ion_phys[%i] = 0x%0llx (g_ion_size[%i]=%u)\n",
			 __func__, i, g_ion_phys[i], i,
			 g_ion_size[i]);
		if (allocsize > g_ion_size[i]) {
			pr_info("ERROR %s:%d g_ion_size(%u)<allocsize(%zu)\n",
				__func__, __LINE__, g_ion_size[i], allocsize);
			return TUI_DCI_ERR_INTERNAL_ERROR;
		}
		allocbuffer[i].pa = g_ion_phys[i];
	}

	width = g_user_rsp.screen_metrics[0];
	height = g_user_rsp.screen_metrics[1];
	stride = g_user_rsp.screen_metrics[2];

	return TUI_DCI_OK;
}

void hal_tui_post_start(struct tlc_tui_response_t *rsp)
{
	
	struct ion_handle *ihandle = { 0 };
	ion_phys_addr_t ion_phys_addr = 0;
	size_t ion_length = 0;
	int ion_ret = 0, i = 0;
	unsigned char idx = 0;

	uint32_t num_of_buff = dci->cmd_nwd.payload.alloc_data.num_of_buff;

	if (TLC_TUI_OK != rsp->return_code) {
		pr_debug("ERROR %s:%d  TLC_TUI_CMD_ALLOC_FB failed (ret=%u)!\n",
			 __func__, __LINE__, rsp->return_code);
		return;
	}
#ifdef WB_USER_ALLOC
	for (i = 0; i < num_of_buff; i++) {
#else
	for (i = 0; i < num_of_buff - 1; i++) {
#endif
		pr_debug("%s: rsp.ion_fd[%d].fd=%d\n",
			 __func__, i, rsp->ion_fd[i]);
		
		if (!rsp->ion_fd[i]) {
			pr_debug("ERROR %s:%d  invalid ion_fd[%i]=%i!\n",
				 __func__, __LINE__, i, rsp->ion_fd[i]);
			goto clean_up;
		}
		
		ihandle = ion_import_dma_buf(g_iclnt, rsp->ion_fd[i]);
		pr_debug("%s: ihandle=%p\n", __func__, ihandle);

		ion_ret = ion_phys(g_iclnt, ihandle, &ion_phys_addr,
				   &ion_length);
		if (0 != ion_ret) {
			pr_debug("ERROR %s:%d ion_phys failed (ret=%i)!\n",
				 __func__, __LINE__, ion_ret);
		} else {
#ifdef WB_USER_ALLOC
			idx = i;
#else
			idx = i + ION_PHYS_FRAME_BUFFER_IDX;
#endif
			g_ion_phys[idx] = (uint64_t)ion_phys_addr;
			g_ion_size[idx] = (uint32_t)ion_length;
			pr_debug("%s: g_ion_phys[%d]=0x%0llX g_ion_size[%d]=%u\n",
				 __func__, idx, g_ion_phys[idx], idx,
				 g_ion_size[idx]);
		}
	}

	return;

clean_up:
	memset(g_ion_phys, 0, sizeof(g_ion_phys));
	memset(g_ion_size, 0, sizeof(g_ion_size));
}

void hal_tui_free(void)
{
	
	if (g_iclnt) {
		pr_debug("%s Destroy ion client.", __func__);
		ion_client_destroy(g_iclnt);
		g_iclnt = NULL;
	}

	
	int ret = send_cmd_to_user(TLC_TUI_CMD_FREE_FB, 0, 0);

	if (TUI_DCI_OK != ret) {
		
		pr_info("ERROR %s:%d TLC_TUI_CMD_FREE_FB failed with (%d)",
			__func__, __LINE__, ret);
	}
}

static int is_synaptics(struct device *dev, const void *data)
{
	int found = 0;

	if (dev) {
		char *path;

		path = kobject_get_path(&dev->kobj, GFP_KERNEL);
		if (!path)
			return 0;

		pr_debug("kobj_path: %s\n", path);

		if (0 == strcmp(path, touchscreen_path))
			found = 1;

		kfree(path);
		return found;
	}
	return 0;
}

static struct device *find_synaptic_driver(void)
{
	struct device *dev;

	dev = class_find_device(&input_class, NULL, NULL, is_synaptics);
	if (!dev)
		pr_debug("ERROR cannot get synaptics device\n");
	return dev;
}

static void synaptics_clk_control(struct device *dev, bool enable)
{
	int ret = -1;

	if (enable)
		core_clk = clk_get(dev->parent, "core_clk");
	else
		clk_put(core_clk);
	if (IS_ERR(core_clk)) {
		ret = PTR_ERR(core_clk);
		dev_err(dev->parent,
			"%s: error %s core_clk:%d\n", __func__,
			enable ? "enable" : "disable", ret);
		if (enable)
			return;
	}

	if (enable)
		iface_clk = clk_get(dev->parent, "iface_clk");
	else
		clk_put(iface_clk);
	if (IS_ERR(iface_clk)) {
		ret = PTR_ERR(iface_clk);
		dev_err(dev->parent,
			"%s: error %s iface_clk:%d\n", __func__,
			enable ? "enable" : "disable", ret);
		goto err_iface_clk;
	}

	return;

err_iface_clk:
	clk_put(core_clk);
}

static bool notify_touch_event(void)
{
	enum mc_result result;

	
	pr_debug("%s\n", __func__);

	dci->hal_cmd = 1;

	
	result = mc_notify(get_session_handle());

	if (MC_DRV_OK != result) {
		pr_debug("ERROR %s: mc_notify failed: %d\n", __func__, result);
		return false;
	}

	return true;
}

int forward_irq_thread_fn(void *data)
{
	struct synaptics_rmi4_data *rmi4_data =
		(struct synaptics_rmi4_data *)data;
	struct device *dev = &rmi4_data->input_dev->dev;

	pr_info("from irq kthread secure touch IRQ\n");
	while (1) {
		pr_info("NWd waiting for TUI event\n");
		
		wait_for_completion(&rmi4_data->st_irq_received);

		if (0 == atomic_read(&rmi4_data->st_enabled))
			do_exit(0);

		int i;
		while ((i = synaptics_secure_get_irq(dev))) {
			pr_info("NWd got an event to fwd to Swd\n");

			if (i < 0) {
				pr_err("synaptics_secure_get_irq returned %d\n",
				       i);
				if (-EBADF == i)
					do_exit(0);
				else if (-EINVAL == i)
					tlc_notify_event(1);
				break;
			}

			pr_info("NWd: i is %d\n", i);
			reinit_completion(&swd_completed);
			dci->hal_rsp = 0;
			notify_touch_event();
			while (!dci->hal_rsp &&
			       atomic_read(&rmi4_data->st_enabled))
				wait_for_completion(&swd_completed);

			pr_info("NWd: event has been handled by the SWd\n");
		}
	}
}

uint32_t hal_tui_deactivate(void)
{
	struct device *dev;

	dev = find_synaptic_driver();
	if (!dev) {
		pr_err("Could not find %s device", touchscreen_path);
		return TUI_DCI_ERR_UNKNOWN_CMD;
	}

	
	synaptics_secure_touch_enable_store(dev, NULL, "1",
					    strlen("1")+1);

	
	irq_fwd_thread = kthread_run(forward_irq_thread_fn,
				     dev_get_drvdata(dev),
				     "synaptics_forwarder");
	if (!irq_fwd_thread) {
		pr_err("Unable to start Trusted UI forwarder thread\n");
		put_device(dev);
		return TUI_DCI_ERR_UNKNOWN_CMD;
	}

	synaptics_clk_control(dev, true);

	return TUI_DCI_OK;
}

uint32_t hal_tui_activate(void)
{
	struct device *dev = find_synaptic_driver();

	if (dev) {
		
		synaptics_clk_control(dev, false);

		
		synaptics_secure_touch_enable_store(dev, NULL, "0",
						    strlen("0")+1);
		complete(&swd_completed);
	}

	return TUI_DCI_OK;
}

uint32_t hal_tui_init(void)
{
	return TUI_DCI_OK;
}

void hal_tui_exit(void)
{
	
}

static unsigned int get_buffer_id(uint64_t phys_addr)
{
	unsigned int i;
	
	for (i = ION_PHYS_FRAME_BUFFER_IDX; i < MAX_BUFFER_NUMBER; i++) {
		if (g_ion_phys[i] == phys_addr)
			break;
	}
#ifdef WB_USER_ALLOC
	return i;
#else
	return i - ION_PHYS_FRAME_BUFFER_IDX;
#endif
}

uint32_t hal_tui_process_cmd(struct tui_hal_cmd_t *cmd,
			     struct tui_hal_rsp_t *rsp)
{
	struct tui_hal_cmd_t swd_cmd = {0};
	uint32_t ret = TUI_DCI_ERR_INTERNAL_ERROR;

	pr_debug("%s\n", __func__);
	unsigned int buffer_id = -1;

	if (cmd) {
		memcpy(&swd_cmd, cmd, sizeof(struct tui_hal_cmd_t));
		pr_debug("%s: hal cmd id=%d\n", __func__, swd_cmd.id);
		switch (swd_cmd.id) {
		case CMD_TUI_HAL_QUEUE_BUFFER:
			pr_debug("%s: hal phys addr[0]=0x%llX\n",
				 __func__, swd_cmd.data[0]);
			buffer_id = get_buffer_id(swd_cmd.data[0]);
			pr_debug("%s: phys addr is at index %d\n",
				 __func__, buffer_id);
			ret =
			send_cmd_to_user(TLC_TUI_CMD_QUEUE,
					 buffer_id,
					 0);
			if (TUI_DCI_OK == ret) {
					rsp->data[0] = width;
					rsp->data[1] = height;
					rsp->data[2] = stride;
			}
		break;

		case CMD_TUI_HAL_QUEUE_DEQUEUE_BUFFER:
			pr_debug("%s: hal phys addr[0]=0x%llX\n",
				 __func__, swd_cmd.data[0]);
			buffer_id = get_buffer_id(swd_cmd.data[0]);
			pr_debug("%s: phys addr is at index %d\n",
				 __func__, buffer_id);

			ret =
			send_cmd_to_user(TLC_TUI_CMD_QUEUE_DEQUEUE,
					 buffer_id,
					 0);
		break;

		case CMD_TUI_HAL_CLEAR_TOUCH_INTERRUPT:
			complete(&swd_completed);
			pr_info("INFO %s:%d\n", __func__, __LINE__);
			break;

		case CMD_TUI_HAL_GET_RESOLUTION:
			ret = send_cmd_to_user(TLC_TUI_CMD_GET_RESOLUTION,
					       0,
					       0);
			if (TUI_DCI_OK == ret) {
				rsp->data[0] = g_user_rsp.screen_metrics[0];
				rsp->data[1] = g_user_rsp.screen_metrics[1];
			}
			break;

		case CMD_TUI_HAL_HIDE_SURFACE:
			send_cmd_to_user(TLC_TUI_CMD_HIDE_SURFACE,
					 0,
					 0);
			break;

		
		case CMD_TUI_HAL_NONE:
		default:
			pr_debug("ERROR %s:%d\n", __func__, __LINE__);
			break;
		}

		
		rsp->return_code = ret;
		rsp->id = RSP_ID(swd_cmd.id);
	}

	return ret;
}

uint32_t hal_tui_notif(void)
{
	complete(&swd_completed);
	return 0;
}
