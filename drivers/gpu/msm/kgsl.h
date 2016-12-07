/* Copyright (c) 2008-2016, The Linux Foundation. All rights reserved.
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
#ifndef __KGSL_H
#define __KGSL_H

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/msm_kgsl.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/regulator/consumer.h>
#include <linux/mm.h>
#include <linux/dma-attrs.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>
#include "kgsl_htc.h"

#define KGSL_MEMSTORE_SIZE	((int)(PAGE_SIZE * 2))
#define KGSL_MEMSTORE_GLOBAL	(0)
#define KGSL_PRIORITY_MAX_RB_LEVELS 4
#define KGSL_MEMSTORE_MAX	(KGSL_MEMSTORE_SIZE / \
	sizeof(struct kgsl_devmemstore) - 1 - KGSL_PRIORITY_MAX_RB_LEVELS)

#define MEMSTORE_RB_OFFSET(rb, field)	\
	KGSL_MEMSTORE_OFFSET(((rb)->id + KGSL_MEMSTORE_MAX), field)

#define MEMSTORE_ID_GPU_ADDR(dev, iter, field) \
	((dev)->memstore.gpuaddr + KGSL_MEMSTORE_OFFSET(iter, field))

#define MEMSTORE_RB_GPU_ADDR(dev, rb, field)	\
	((dev)->memstore.gpuaddr + \
	 KGSL_MEMSTORE_OFFSET(((rb)->id + KGSL_MEMSTORE_MAX), field))


#define SCRATCH_RPTR_OFFSET(id) ((id) * sizeof(unsigned int))
#define SCRATCH_RPTR_GPU_ADDR(dev, id) \
	((dev)->scratch.gpuaddr + SCRATCH_RPTR_OFFSET(id))

#define KGSL_TIMESTAMP_WINDOW 0x80000000


static inline void KGSL_STATS_ADD(uint64_t size, atomic_long_t *stat,
		atomic_long_t *max)
{
	uint64_t ret = atomic_long_add_return(size, stat);

	if (ret > atomic_long_read(max))
		atomic_long_set(max, ret);
}

#define KGSL_MAX_NUMIBS 100000
#define KGSL_MAX_SYNCPOINTS 32

struct kgsl_device;
struct kgsl_context;

struct kgsl_driver {
	struct cdev cdev;
	dev_t major;
	struct class *class;
	struct device virtdev;
	struct kobject *ptkobj;
	struct kobject *prockobj;
	struct kgsl_device *devp[KGSL_DEVICE_MAX];
	struct list_head process_list;
	struct list_head pagetable_list;
	spinlock_t ptlock;
	struct mutex process_mutex;
	struct mutex devlock;
	struct {
		atomic_long_t vmalloc;
		atomic_long_t vmalloc_max;
		atomic_long_t page_alloc;
		atomic_long_t page_alloc_max;
		atomic_long_t coherent;
		atomic_long_t coherent_max;
		atomic_long_t secure;
		atomic_long_t secure_max;
		atomic_long_t mapped;
		atomic_long_t mapped_max;
	} stats;
	unsigned int full_cache_threshold;
	struct workqueue_struct *workqueue;
	struct workqueue_struct *mem_workqueue;
	struct kgsl_driver_htc_priv priv;
};

extern struct kgsl_driver kgsl_driver;
extern struct mutex kgsl_mmu_sync;

struct kgsl_pagetable;
struct kgsl_memdesc;

struct kgsl_memdesc_ops {
	unsigned int vmflags;
	int (*vmfault)(struct kgsl_memdesc *, struct vm_area_struct *,
		       struct vm_fault *);
	void (*free)(struct kgsl_memdesc *memdesc);
	int (*map_kernel)(struct kgsl_memdesc *);
	void (*unmap_kernel)(struct kgsl_memdesc *);
};

#define KGSL_MEMDESC_GUARD_PAGE BIT(0)
#define KGSL_MEMDESC_GLOBAL BIT(1)
#define KGSL_MEMDESC_FROZEN BIT(2)
#define KGSL_MEMDESC_MAPPED BIT(3)
#define KGSL_MEMDESC_SECURE BIT(4)
#define KGSL_MEMDESC_PRIVILEGED BIT(6)
#define KGSL_MEMDESC_TZ_LOCKED BIT(7)
#define KGSL_MEMDESC_CONTIG BIT(8)

struct kgsl_memdesc {
	struct kgsl_pagetable *pagetable;
	void *hostptr;
	unsigned int hostptr_count;
	unsigned long useraddr;
	uint64_t gpuaddr;
	phys_addr_t physaddr;
	uint64_t size;
	uint64_t mapsize;
	unsigned int priv;
	struct sg_table *sgt;
	struct kgsl_memdesc_ops *ops;
	uint64_t flags;
	struct device *dev;
	struct dma_attrs attrs;
	struct page **pages;
	unsigned int page_count;

	struct kgsl_process_private *private;
};

#define KGSL_MEM_ENTRY_KERNEL 0
#define KGSL_MEM_ENTRY_USER (KGSL_USER_MEM_TYPE_ADDR + 1)
#define KGSL_MEM_ENTRY_ION (KGSL_USER_MEM_TYPE_ION + 1)
#define KGSL_MEM_ENTRY_PAGE_ALLOC (KGSL_USER_MEM_TYPE_ION + 2)
#define KGSL_MEM_ENTRY_MAX (KGSL_USER_MEM_TYPE_MAX + 1)

#define KGSL_MEM_TYPES \
	{ KGSL_MEM_ENTRY_KERNEL, "gpumem" }, \
	{ KGSL_MEM_ENTRY_USER, "usermem" }, \
	{ KGSL_MEM_ENTRY_ION, "ion" }

struct kgsl_mem_entry {
	struct kref refcount;
	struct kgsl_memdesc memdesc;
	void *priv_data;
	struct rb_node node;
	unsigned int id;
	struct kgsl_process_private *priv;
	int pending_free;
	char metadata[KGSL_GPUOBJ_ALLOC_METADATA_MAX + 1];
	struct work_struct work;
};

struct kgsl_device_private;
struct kgsl_event_group;

typedef void (*kgsl_event_func)(struct kgsl_device *, struct kgsl_event_group *,
		void *, int);

struct kgsl_event {
	struct kgsl_device *device;
	struct kgsl_context *context;
	unsigned int timestamp;
	kgsl_event_func func;
	void *priv;
	struct list_head node;
	unsigned int created;
	struct work_struct work;
	int result;
	struct kgsl_event_group *group;
};

typedef int (*readtimestamp_func)(struct kgsl_device *, void *,
	enum kgsl_timestamp_type, unsigned int *);

struct kgsl_event_group {
	struct kgsl_context *context;
	spinlock_t lock;
	struct list_head events;
	struct list_head group;
	unsigned int processed;
	char name[64];
	readtimestamp_func readtimestamp;
	void *priv;
};

struct kgsl_protected_registers {
	unsigned int base;
	int range;
};

long kgsl_ioctl_device_getproperty(struct kgsl_device_private *dev_priv,
					  unsigned int cmd, void *data);
long kgsl_ioctl_device_setproperty(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_device_waittimestamp_ctxtid(struct kgsl_device_private
				*dev_priv, unsigned int cmd, void *data);
long kgsl_ioctl_rb_issueibcmds(struct kgsl_device_private *dev_priv,
				      unsigned int cmd, void *data);
long kgsl_ioctl_submit_commands(struct kgsl_device_private *dev_priv,
				unsigned int cmd, void *data);
long kgsl_ioctl_cmdstream_readtimestamp_ctxtid(struct kgsl_device_private
					*dev_priv, unsigned int cmd,
					void *data);
long kgsl_ioctl_cmdstream_freememontimestamp_ctxtid(
						struct kgsl_device_private
						*dev_priv, unsigned int cmd,
						void *data);
long kgsl_ioctl_drawctxt_create(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_drawctxt_destroy(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_sharedmem_free(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_free_id(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_map_user_mem(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_sync_cache(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_sync_cache_bulk(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_sharedmem_flush_cache(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_alloc(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_alloc_id(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpumem_get_info(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_cff_syncmem(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_cff_user_event(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_timestamp_event(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_cff_sync_gpuobj(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpuobj_alloc(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpuobj_free(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpuobj_info(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpuobj_import(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpuobj_sync(struct kgsl_device_private *dev_priv,
					unsigned int cmd, void *data);
long kgsl_ioctl_gpu_command(struct kgsl_device_private *dev_priv,
				unsigned int cmd, void *data);
long kgsl_ioctl_gpuobj_set_info(struct kgsl_device_private *dev_priv,
				unsigned int cmd, void *data);

void kgsl_mem_entry_destroy(struct kref *kref);

struct kgsl_mem_entry * __must_check
kgsl_sharedmem_find(struct kgsl_process_private *private, uint64_t gpuaddr);

struct kgsl_mem_entry * __must_check
kgsl_sharedmem_find_id(struct kgsl_process_private *process, unsigned int id);

extern const struct dev_pm_ops kgsl_pm_ops;

int kgsl_suspend_driver(struct platform_device *pdev, pm_message_t state);
int kgsl_resume_driver(struct platform_device *pdev);

static inline int kgsl_gpuaddr_in_memdesc(const struct kgsl_memdesc *memdesc,
				uint64_t gpuaddr, uint64_t size)
{
	
	if (!size)
		size = 1;

	
	if (size > U64_MAX - gpuaddr)
		return 0;

	if (gpuaddr >= memdesc->gpuaddr &&
	    ((gpuaddr + size) <= (memdesc->gpuaddr + memdesc->size))) {
		return 1;
	}
	return 0;
}

static inline void *kgsl_memdesc_map(struct kgsl_memdesc *memdesc)
{
	if (memdesc->ops && memdesc->ops->map_kernel)
		memdesc->ops->map_kernel(memdesc);

	return memdesc->hostptr;
}

static inline void kgsl_memdesc_unmap(struct kgsl_memdesc *memdesc)
{
	if (memdesc->ops && memdesc->ops->unmap_kernel)
		memdesc->ops->unmap_kernel(memdesc);
}

static inline void *kgsl_gpuaddr_to_vaddr(struct kgsl_memdesc *memdesc,
					     uint64_t gpuaddr)
{
	void *hostptr = NULL;

	if ((gpuaddr >= memdesc->gpuaddr) &&
		(gpuaddr < (memdesc->gpuaddr + memdesc->size)))
		hostptr = kgsl_memdesc_map(memdesc);

	return hostptr != NULL ? hostptr + (gpuaddr - memdesc->gpuaddr) : NULL;
}

static inline int timestamp_cmp(unsigned int a, unsigned int b)
{
	
	if (a == b)
		return 0;

	
	if ((a > b) && (a - b < KGSL_TIMESTAMP_WINDOW))
		return 1;

	a += KGSL_TIMESTAMP_WINDOW;
	b += KGSL_TIMESTAMP_WINDOW;
	return ((a > b) && (a - b <= KGSL_TIMESTAMP_WINDOW)) ? 1 : -1;
}

static inline void kgsl_schedule_work(struct work_struct *work)
{
	queue_work(kgsl_driver.workqueue, work);
}

static inline int
kgsl_mem_entry_get(struct kgsl_mem_entry *entry)
{
	if (entry)
		return kref_get_unless_zero(&entry->refcount);
	return 0;
}

static inline void
kgsl_mem_entry_put(struct kgsl_mem_entry *entry)
{
	if (entry)
		kref_put(&entry->refcount, kgsl_mem_entry_destroy);
}

static inline bool kgsl_addr_range_overlap(uint64_t gpuaddr1,
		uint64_t size1, uint64_t gpuaddr2, uint64_t size2)
{
	if ((size1 > (U64_MAX - gpuaddr1)) || (size2 > (U64_MAX - gpuaddr2)))
		return false;
	return !(((gpuaddr1 + size1) <= gpuaddr2) ||
		(gpuaddr1 >= (gpuaddr2 + size2)));
}

static inline void *kgsl_malloc(size_t size)
{
	if (size <= PAGE_SIZE)
		return kzalloc(size, GFP_KERNEL);

	return vmalloc(size);
}

static inline void kgsl_free(void *ptr)
{
	if (ptr != NULL && is_vmalloc_addr(ptr))
		return vfree(ptr);

	kfree(ptr);
}

static inline int _copy_from_user(void *dest, void __user *src,
		unsigned int ksize, unsigned int usize)
{
	unsigned int copy = ksize < usize ? ksize : usize;

	if (copy == 0)
		return -EINVAL;

	return copy_from_user(dest, src, copy) ? -EFAULT : 0;
}

static inline void __user *to_user_ptr(uint64_t address)
{
	return (void __user *)(uintptr_t)address;
}

#endif 
