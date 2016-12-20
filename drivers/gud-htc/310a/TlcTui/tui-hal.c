/*
 * Copyright (c) 2014 TRUSTONIC LIMITED
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
#include <linux/device.h>
#include <linux/fb.h>

#ifndef CONFIG_TRUSTONIC_TRUSTED_UI
#define CONFIG_TRUSTONIC_TRUSTED_UI
#endif
#include <t-base-tui.h>

#include "tui_ioctl.h"
#include "dciTui.h"
#include "tlcTui.h"
#include "tui-hal.h"


#define TUI_MEMPOOL_SIZE 0

struct tui_mempool {
	void *va;
	unsigned long pa;
	size_t size;
};

static struct tui_mempool g_tui_mem_pool;

static bool allocate_tui_memory_pool(struct tui_mempool *pool, size_t size)
{
	bool ret = false;
	void *tui_mem_pool = NULL;

	pr_info("%s %s:%d\n", __func__, __FILE__, __LINE__);
	if (!size) {
		pr_debug("TUI frame buffer: nothing to allocate.");
		return true;
	}

	tui_mem_pool = kmalloc(size, GFP_KERNEL);
	if (!tui_mem_pool) {
		pr_debug("ERROR Could not allocate TUI memory pool");
	} else if (ksize(tui_mem_pool) < size) {
		pr_err("TUI mem pool size too small: req'd=%zu alloc'd=%zu",
		       size, ksize(tui_mem_pool));
		kfree(tui_mem_pool);
	} else {
		pool->va = tui_mem_pool;
		pool->pa = virt_to_phys(tui_mem_pool);
		pool->size = ksize(tui_mem_pool);
		ret = true;
	}
	return ret;
}

static void free_tui_memory_pool(struct tui_mempool *pool)
{
	kfree(pool->va);
	memset(pool, 0, sizeof(*pool));
}

uint32_t hal_tui_init(void)
{
	if (!allocate_tui_memory_pool(&g_tui_mem_pool, TUI_MEMPOOL_SIZE))
		return TUI_DCI_ERR_INTERNAL_ERROR;

	return TUI_DCI_OK;
}

void hal_tui_exit(void)
{
	
	if (g_tui_mem_pool.va)
		free_tui_memory_pool(&g_tui_mem_pool);
}

uint32_t hal_tui_alloc(
	struct tui_alloc_buffer_t allocbuffer[MAX_DCI_BUFFER_NUMBER],
	size_t allocsize, uint32_t number)
{
	uint32_t ret = TUI_DCI_ERR_INTERNAL_ERROR;

	if (!allocbuffer) {
		pr_debug("%s(%d): allocbuffer is null\n", __func__, __LINE__);
		return TUI_DCI_ERR_INTERNAL_ERROR;
	}

	pr_debug("%s(%d): Requested size=0x%zx x %u chunks\n",
		 __func__, __LINE__, allocsize, number);

	if ((size_t)allocsize == 0) {
		pr_debug("%s(%d): Nothing to allocate\n", __func__, __LINE__);
		return TUI_DCI_OK;
	}

	if (number != 2) {
		pr_debug("%s(%d): Unexpected number of buffers requested\n",
			 __func__, __LINE__);
		return TUI_DCI_ERR_INTERNAL_ERROR;
	}

	if ((size_t)(allocsize*number) <= g_tui_mem_pool.size) {
		
		allocbuffer[0].pa = (uint64_t) g_tui_mem_pool.pa;
		allocbuffer[1].pa = (uint64_t) (g_tui_mem_pool.pa +
						g_tui_mem_pool.size/2);
		pr_debug("%s(%d): allocated at %llx\n", __func__, __LINE__,
			 allocbuffer[0].pa);
		pr_debug("%s(%d): allocated at %llx\n", __func__, __LINE__,
			 allocbuffer[1].pa);
		ret = TUI_DCI_OK;
	} else {
		pr_debug("%s(%d): Memory pool too small\n", __func__, __LINE__);
		ret = TUI_DCI_ERR_INTERNAL_ERROR;
	}

	return ret;
}

void hal_tui_free(void)
{
}

uint32_t hal_tui_deactivate(void)
{
	
	trustedui_set_mask(TRUSTEDUI_MODE_TUI_SESSION);
	trustedui_set_mask(TRUSTEDUI_MODE_VIDEO_SECURED|
			   TRUSTEDUI_MODE_INPUT_SECURED);

	return TUI_DCI_OK;
}

uint32_t hal_tui_activate(void)
{
	
	trustedui_clear_mask(TRUSTEDUI_MODE_VIDEO_SECURED|
			     TRUSTEDUI_MODE_INPUT_SECURED);
	
	trustedui_set_mode(TRUSTEDUI_MODE_OFF);
	return TUI_DCI_OK;
}

