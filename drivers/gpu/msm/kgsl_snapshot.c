/* Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
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

#include <linux/export.h>
#include <linux/time.h>
#include <linux/sysfs.h>
#include <linux/utsname.h>
#include <linux/sched.h>
#include <linux/idr.h>

#include "kgsl.h"
#include "kgsl_log.h"
#include "kgsl_device.h"
#include "kgsl_sharedmem.h"
#include "kgsl_snapshot.h"
#include "adreno_cp_parser.h"


struct kgsl_snapshot_cp_obj {
	struct adreno_ib_object_list *ib_obj_list;
	struct list_head node;
};

struct snapshot_obj_itr {
	u8 *buf;      
	int pos;        
	loff_t offset;  
	size_t remain;  
	size_t write;   /* Bytes written so far */
};

static void obj_itr_init(struct snapshot_obj_itr *itr, u8 *buf,
	loff_t offset, size_t remain)
{
	itr->buf = buf;
	itr->offset = offset;
	itr->remain = remain;
	itr->pos = 0;
	itr->write = 0;
}

static int obj_itr_out(struct snapshot_obj_itr *itr, void *src, int size)
{
	if (itr->remain == 0)
		return 0;

	if ((itr->pos + size) <= itr->offset)
		goto done;

	

	if (itr->offset > itr->pos) {
		src += (itr->offset - itr->pos);
		size -= (itr->offset - itr->pos);

		
		itr->pos = itr->offset;
	}

	if (size > itr->remain)
		size = itr->remain;

	memcpy(itr->buf, src, size);

	itr->buf += size;
	itr->write += size;
	itr->remain -= size;

done:
	itr->pos += size;
	return size;
}


static int snapshot_context_count(int id, void *ptr, void *data)
{
	int *count = data;
	*count = *count + 1;

	return 0;
}


static u8 *_ctxtptr;

static int snapshot_context_info(int id, void *ptr, void *data)
{
	struct kgsl_snapshot_linux_context_v2 *header =
		(struct kgsl_snapshot_linux_context_v2 *)_ctxtptr;
	struct kgsl_context *context = ptr;
	struct kgsl_device *device;

	device = context->device;

	header->id = id;


	kgsl_readtimestamp(device, context, KGSL_TIMESTAMP_QUEUED,
		&header->timestamp_queued);
	kgsl_readtimestamp(device, context, KGSL_TIMESTAMP_CONSUMED,
		&header->timestamp_consumed);
	kgsl_readtimestamp(device, context, KGSL_TIMESTAMP_RETIRED,
		&header->timestamp_retired);

	_ctxtptr += sizeof(struct kgsl_snapshot_linux_context_v2);

	return 0;
}

static size_t snapshot_os(struct kgsl_device *device,
	u8 *buf, size_t remain, void *priv)
{
	struct kgsl_snapshot_linux_v2 *header =
		(struct kgsl_snapshot_linux_v2 *)buf;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int ctxtcount = 0;
	size_t size = sizeof(*header);
	struct kgsl_context *context;


	read_lock(&device->context_lock);
	idr_for_each(&device->context_idr, snapshot_context_count, &ctxtcount);
	read_unlock(&device->context_lock);

	size += ctxtcount * sizeof(struct kgsl_snapshot_linux_context_v2);

	
	if (remain < size) {
		SNAPSHOT_ERR_NOMEM(device, "OS");
		return 0;
	}

	memset(header, 0, sizeof(*header));

	header->osid = KGSL_SNAPSHOT_OS_LINUX_V3;

	
	strlcpy(header->release, utsname()->release, sizeof(header->release));
	strlcpy(header->version, utsname()->version, sizeof(header->version));

	
	header->seconds = get_seconds();

	
	header->power_flags = pwr->power_flags;
	header->power_level = pwr->active_pwrlevel;
	header->power_interval_timeout = pwr->interval_timeout;
	header->grpclk = kgsl_get_clkrate(pwr->grp_clks[0]);

	kgsl_sharedmem_readl(&device->memstore, &header->current_context,
		KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL, current_context));

	context = kgsl_context_get(device, header->current_context);

	
	 header->ptbase = kgsl_mmu_get_current_ttbr0(&device->mmu);

	
	if (context) {
		header->pid = context->tid;
		strlcpy(header->comm, context->proc_priv->comm,
				sizeof(header->comm));
		kgsl_context_put(context);
		context = NULL;
	}

	header->ctxtcount = ctxtcount;

	_ctxtptr = buf + sizeof(*header);
	

	read_lock(&device->context_lock);
	idr_for_each(&device->context_idr, snapshot_context_info, NULL);
	read_unlock(&device->context_lock);

	
	return size;
}

static void kgsl_snapshot_put_object(struct kgsl_snapshot_object *obj)
{
	list_del(&obj->node);

	obj->entry->memdesc.priv &= ~KGSL_MEMDESC_FROZEN;
	kgsl_mem_entry_put(obj->entry);

	kfree(obj);
}

int kgsl_snapshot_have_object(struct kgsl_snapshot *snapshot,
	struct kgsl_process_private *process,
	uint64_t gpuaddr, uint64_t size)
{
	struct kgsl_snapshot_object *obj;
	struct kgsl_snapshot_cp_obj *obj_cp;
	struct adreno_ib_object *ib_obj;
	int i;

	
	list_for_each_entry(obj_cp, &snapshot->cp_list, node) {
		if (obj_cp->ib_obj_list == NULL
			|| obj_cp->ib_obj_list->num_objs == 0)
			continue;

		ib_obj = &(obj_cp->ib_obj_list->obj_list[0]);
		if (ib_obj->entry == NULL || ib_obj->entry->priv != process)
			continue;

		for (i = 0; i < obj_cp->ib_obj_list->num_objs; i++) {
			ib_obj = &(obj_cp->ib_obj_list->obj_list[i]);
			if ((gpuaddr >= ib_obj->gpuaddr) &&
				((gpuaddr + size) <=
				(ib_obj->gpuaddr + ib_obj->size)))
				return 1;
		}
	}

	list_for_each_entry(obj, &snapshot->obj_list, node) {
		if (obj->entry == NULL || obj->entry->priv != process)
			continue;

		if ((gpuaddr >= obj->gpuaddr) &&
			((gpuaddr + size) <= (obj->gpuaddr + obj->size)))
			return 1;
	}

	return 0;
}
EXPORT_SYMBOL(kgsl_snapshot_have_object);

int kgsl_snapshot_get_object(struct kgsl_snapshot *snapshot,
	struct kgsl_process_private *process, uint64_t gpuaddr,
	uint64_t size, unsigned int type)
{
	struct kgsl_mem_entry *entry;
	struct kgsl_snapshot_object *obj;
	uint64_t offset;
	int ret = -EINVAL;
	unsigned int mem_type;

	if (!gpuaddr)
		return 0;

	entry = kgsl_sharedmem_find(process, gpuaddr);

	if (entry == NULL) {
		KGSL_CORE_ERR("Unable to find GPU buffer 0x%016llX\n", gpuaddr);
		return -EINVAL;
	}

	
	if (entry->memdesc.flags & KGSL_MEMFLAGS_USERMEM_MASK)
		goto err_put;

	mem_type = kgsl_memdesc_get_memtype(&entry->memdesc);
	if (KGSL_MEMTYPE_TEXTURE == mem_type ||
		KGSL_MEMTYPE_EGL_SURFACE == mem_type ||
		KGSL_MEMTYPE_EGL_IMAGE == mem_type) {
		ret = 0;
		goto err_put;
	}


	if (size == 0) {
		size = entry->memdesc.size;
		offset = 0;

		
		gpuaddr = entry->memdesc.gpuaddr;
	} else {
		offset = gpuaddr - entry->memdesc.gpuaddr;
	}

	if (size + offset > entry->memdesc.size) {
		KGSL_CORE_ERR("Invalid size for GPU buffer 0x%016llX\n",
			gpuaddr);
		goto err_put;
	}

	
	list_for_each_entry(obj, &snapshot->obj_list, node) {
		
		if (obj->entry->priv == process && obj->type == type &&
			kgsl_addr_range_overlap(obj->gpuaddr, obj->size,
				gpuaddr, size)) {
			uint64_t end1 = obj->gpuaddr + obj->size;
			uint64_t end2 = gpuaddr + size;
			if (obj->gpuaddr > gpuaddr)
				obj->gpuaddr = gpuaddr;
			if (end1 > end2)
				obj->size = end1 - obj->gpuaddr;
			else
				obj->size = end2 - obj->gpuaddr;
			obj->offset = obj->gpuaddr - entry->memdesc.gpuaddr;
			ret = 0;
			goto err_put;
		}
	}

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);

	if (obj == NULL)
		goto err_put;

	obj->type = type;
	obj->entry = entry;
	obj->gpuaddr = gpuaddr;
	obj->size = size;
	obj->offset = offset;

	list_add(&obj->node, &snapshot->obj_list);


	ret = (entry->memdesc.priv & KGSL_MEMDESC_FROZEN) ? 0
		: entry->memdesc.size;

	entry->memdesc.priv |= KGSL_MEMDESC_FROZEN;

	return ret;
err_put:
	kgsl_mem_entry_put(entry);
	return ret;
}
EXPORT_SYMBOL(kgsl_snapshot_get_object);

size_t kgsl_snapshot_dump_registers(struct kgsl_device *device, u8 *buf,
	size_t remain, void *priv)
{
	struct kgsl_snapshot_regs *header = (struct kgsl_snapshot_regs *)buf;
	struct kgsl_snapshot_registers *regs = priv;
	unsigned int *data = (unsigned int *)(buf + sizeof(*header));
	int count = 0, j, k;

	

	for (j = 0; j < regs->count; j++) {
		int start = regs->regs[j * 2];
		int end = regs->regs[j * 2 + 1];

		count += (end - start + 1);
	}

	if (remain < (count * 8) + sizeof(*header)) {
		SNAPSHOT_ERR_NOMEM(device, "REGISTERS");
		return 0;
	}

	for (j = 0; j < regs->count; j++) {
		unsigned int start = regs->regs[j * 2];
		unsigned int end = regs->regs[j * 2 + 1];

		for (k = start; k <= end; k++) {
			unsigned int val;

			kgsl_regread(device, k, &val);
			*data++ = k;
			*data++ = val;
		}
	}

	header->count = count;

	
	return (count * 8) + sizeof(*header);
}
EXPORT_SYMBOL(kgsl_snapshot_dump_registers);

struct kgsl_snapshot_indexed_registers {
	unsigned int index;
	unsigned int data;
	unsigned int start;
	unsigned int count;
};

static size_t kgsl_snapshot_dump_indexed_regs(struct kgsl_device *device,
	u8 *buf, size_t remain, void *priv)
{
	struct kgsl_snapshot_indexed_registers *iregs = priv;
	struct kgsl_snapshot_indexed_regs *header =
		(struct kgsl_snapshot_indexed_regs *)buf;
	unsigned int *data = (unsigned int *)(buf + sizeof(*header));
	int i;

	BUG_ON(!mutex_is_locked(&device->mutex));

	if (remain < (iregs->count * 4) + sizeof(*header)) {
		SNAPSHOT_ERR_NOMEM(device, "INDEXED REGS");
		return 0;
	}

	header->index_reg = iregs->index;
	header->data_reg = iregs->data;
	header->count = iregs->count;
	header->start = iregs->start;

	for (i = 0; i < iregs->count; i++) {
		kgsl_regwrite(device, iregs->index, iregs->start + i);
		kgsl_regread(device, iregs->data, &data[i]);
	}

	return (iregs->count * 4) + sizeof(*header);
}

void kgsl_snapshot_indexed_registers(struct kgsl_device *device,
		struct kgsl_snapshot *snapshot,
		unsigned int index, unsigned int data,
		unsigned int start,
		unsigned int count)
{
	struct kgsl_snapshot_indexed_registers iregs;
	iregs.index = index;
	iregs.data = data;
	iregs.start = start;
	iregs.count = count;

	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_INDEXED_REGS,
		snapshot, kgsl_snapshot_dump_indexed_regs, &iregs);
}
EXPORT_SYMBOL(kgsl_snapshot_indexed_registers);

void kgsl_snapshot_add_section(struct kgsl_device *device, u16 id,
	struct kgsl_snapshot *snapshot,
	size_t (*func)(struct kgsl_device *, u8 *, size_t, void *),
	void *priv)
{
	struct kgsl_snapshot_section_header *header =
		(struct kgsl_snapshot_section_header *)snapshot->ptr;
	u8 *data = snapshot->ptr + sizeof(*header);
	size_t ret = 0;


	if (snapshot->remain < sizeof(*header))
		return;

	
	if (func) {
		ret = func(device, data, snapshot->remain - sizeof(*header),
			priv);


		if (ret == 0)
			return;
	}

	header->magic = SNAPSHOT_SECTION_MAGIC;
	header->id = id;
	header->size = ret + sizeof(*header);

	snapshot->ptr += header->size;
	snapshot->remain -= header->size;
	snapshot->size += header->size;
}

void kgsl_device_snapshot(struct kgsl_device *device,
		struct kgsl_context *context)
{
	struct kgsl_snapshot_header *header = device->snapshot_memory.ptr;
	struct kgsl_snapshot *snapshot;
	struct timespec boot;
	phys_addr_t pa;

	if (device->snapshot_memory.ptr == NULL) {
		KGSL_DRV_ERR(device,
			"snapshot: no snapshot memory available\n");
		return;
	}

	BUG_ON(!kgsl_state_is_awake(device));
	
	device->snapshot_faultcount++;

	if (device->snapshot != NULL)
		return;

	
	snapshot = kzalloc(sizeof(*snapshot), GFP_KERNEL);
	if (snapshot == NULL)
		return;

	init_completion(&snapshot->dump_gate);
	INIT_LIST_HEAD(&snapshot->obj_list);
	INIT_LIST_HEAD(&snapshot->cp_list);
	INIT_WORK(&snapshot->work, kgsl_snapshot_save_frozen_objs);

	snapshot->start = device->snapshot_memory.ptr;
	snapshot->ptr = device->snapshot_memory.ptr;
	snapshot->remain = device->snapshot_memory.size;
	atomic_set(&snapshot->sysfs_read, 0);

	header = (struct kgsl_snapshot_header *) snapshot->ptr;

	header->magic = SNAPSHOT_MAGIC;
	header->gpuid = kgsl_gpuid(device, &header->chipid);

	snapshot->ptr += sizeof(*header);
	snapshot->remain -= sizeof(*header);
	snapshot->size += sizeof(*header);

	
	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_OS,
			snapshot, snapshot_os, NULL);

	
	if (device->ftbl->snapshot)
		device->ftbl->snapshot(device, snapshot, context);


	getboottime(&boot);
	snapshot->timestamp = get_seconds() - boot.tv_sec;

	
	device->snapshot = snapshot;

	
	pa = __pa(device->snapshot_memory.ptr);
	KGSL_DRV_ERR(device, "snapshot created at pa %pa size %zd\n",
			&pa, snapshot->size);

	sysfs_notify(&device->snapshot_kobj, NULL, "timestamp");

	kgsl_schedule_work(&snapshot->work);
}
EXPORT_SYMBOL(kgsl_device_snapshot);

struct kgsl_snapshot_attribute {
	struct attribute attr;
	ssize_t (*show)(struct kgsl_device *device, char *buf);
	ssize_t (*store)(struct kgsl_device *device, const char *buf,
		size_t count);
};

static void kgsl_snapshot_process_ib_obj_list(struct kgsl_snapshot *snapshot)
{
	struct kgsl_snapshot_cp_obj *obj, *obj_temp;
	struct adreno_ib_object *ib_obj;
	int i;

	list_for_each_entry_safe(obj, obj_temp, &snapshot->cp_list,
			node) {
		for (i = 0; i < obj->ib_obj_list->num_objs; i++) {
			ib_obj = &(obj->ib_obj_list->obj_list[i]);
			kgsl_snapshot_get_object(snapshot, ib_obj->entry->priv,
				ib_obj->gpuaddr, ib_obj->size,
				ib_obj->snapshot_obj_type);
		}
		list_del(&obj->node);
		adreno_ib_destroy_obj_list(obj->ib_obj_list);
		kfree(obj);
	}
}

#define to_snapshot_attr(a) \
container_of(a, struct kgsl_snapshot_attribute, attr)

#define kobj_to_device(a) \
container_of(a, struct kgsl_device, snapshot_kobj)

static ssize_t snapshot_show(struct file *filep, struct kobject *kobj,
	struct bin_attribute *attr, char *buf, loff_t off,
	size_t count)
{
	struct kgsl_device *device = kobj_to_device(kobj);
	struct kgsl_snapshot *snapshot;
	struct kgsl_snapshot_object *obj, *tmp;
	struct kgsl_snapshot_section_header head;
	struct snapshot_obj_itr itr;
	int ret;

	if (device == NULL)
		return 0;

	mutex_lock(&device->mutex);
	snapshot = device->snapshot;
	if (snapshot != NULL)
		atomic_inc(&snapshot->sysfs_read);
	mutex_unlock(&device->mutex);

	
	if (snapshot == NULL)
		return 0;

	ret = wait_for_completion_interruptible(&snapshot->dump_gate);
	if (ret) {
		atomic_dec(&snapshot->sysfs_read);
		return ret;
	}

	obj_itr_init(&itr, buf, off, count);

	ret = obj_itr_out(&itr, snapshot->start, snapshot->size);
	if (ret == 0)
		goto done;

	
	if (snapshot->mempool) {
		ret = obj_itr_out(&itr, snapshot->mempool,
				snapshot->mempool_size);
		if (ret == 0)
			goto done;
	}

	{
		head.magic = SNAPSHOT_SECTION_MAGIC;
		head.id = KGSL_SNAPSHOT_SECTION_END;
		head.size = sizeof(head);

		obj_itr_out(&itr, &head, sizeof(head));
	}

	/*
	 * Make sure everything has been written out before destroying things.
	 * The best way to confirm this is to go all the way through without
	 * writing any bytes - so only release if we get this far and
	 * itr->write is 0 and there are no concurrent reads pending
	 */

	if (itr.write == 0) {
		bool snapshot_free = false;

		mutex_lock(&device->mutex);
		if (atomic_dec_and_test(&snapshot->sysfs_read)) {
			device->snapshot = NULL;
			snapshot_free = true;
		}
		mutex_unlock(&device->mutex);

		if (snapshot_free) {
			list_for_each_entry_safe(obj, tmp,
						&snapshot->obj_list, node)
				kgsl_snapshot_put_object(obj);

			if (snapshot->mempool)
				vfree(snapshot->mempool);

			kfree(snapshot);
			KGSL_CORE_ERR("snapshot: objects released\n");
		}
		return 0;
	}

done:
	atomic_dec(&snapshot->sysfs_read);
	return itr.write;
}

static ssize_t faultcount_show(struct kgsl_device *device, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", device->snapshot_faultcount);
}

static ssize_t faultcount_store(struct kgsl_device *device, const char *buf,
	size_t count)
{
	if (device && count > 0)
		device->snapshot_faultcount = 0;

	return count;
}

static ssize_t force_panic_show(struct kgsl_device *device, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", device->force_panic);
}

static ssize_t force_panic_store(struct kgsl_device *device, const char *buf,
	size_t count)
{
	unsigned int val = 0;
	int ret;

	if (device && count > 0)
		device->force_panic = 0;

	ret = kgsl_sysfs_store(buf, &val);

	if (!ret && device)
		device->force_panic = (bool)val;

	return (ssize_t) ret < 0 ? ret : count;
}
static ssize_t timestamp_show(struct kgsl_device *device, char *buf)
{
	unsigned long timestamp =
		device->snapshot ? device->snapshot->timestamp : 0;

	return snprintf(buf, PAGE_SIZE, "%lu\n", timestamp);
}

static struct bin_attribute snapshot_attr = {
	.attr.name = "dump",
	.attr.mode = 0444,
	.size = 0,
	.read = snapshot_show
};

#define SNAPSHOT_ATTR(_name, _mode, _show, _store) \
struct kgsl_snapshot_attribute attr_##_name = { \
	.attr = { .name = __stringify(_name), .mode = _mode }, \
	.show = _show, \
	.store = _store, \
}

static SNAPSHOT_ATTR(timestamp, 0444, timestamp_show, NULL);
static SNAPSHOT_ATTR(faultcount, 0644, faultcount_show, faultcount_store);
static SNAPSHOT_ATTR(force_panic, 0644, force_panic_show, force_panic_store);

static ssize_t snapshot_sysfs_show(struct kobject *kobj,
	struct attribute *attr, char *buf)
{
	struct kgsl_snapshot_attribute *pattr = to_snapshot_attr(attr);
	struct kgsl_device *device = kobj_to_device(kobj);
	ssize_t ret;

	if (device && pattr->show)
		ret = pattr->show(device, buf);
	else
		ret = -EIO;

	return ret;
}

static ssize_t snapshot_sysfs_store(struct kobject *kobj,
	struct attribute *attr, const char *buf, size_t count)
{
	struct kgsl_snapshot_attribute *pattr = to_snapshot_attr(attr);
	struct kgsl_device *device = kobj_to_device(kobj);
	ssize_t ret;

	if (device && pattr->store)
		ret = pattr->store(device, buf, count);
	else
		ret = -EIO;

	return ret;
}

static const struct sysfs_ops snapshot_sysfs_ops = {
	.show = snapshot_sysfs_show,
	.store = snapshot_sysfs_store,
};

static struct kobj_type ktype_snapshot = {
	.sysfs_ops = &snapshot_sysfs_ops,
};

int kgsl_device_snapshot_init(struct kgsl_device *device)
{
	int ret;

	if (kgsl_property_read_u32(device, "qcom,snapshot-size",
		(unsigned int *) &(device->snapshot_memory.size)))
		device->snapshot_memory.size = KGSL_SNAPSHOT_MEMSIZE;

	if (device->snapshot_memory.size == 0)
		return 0;

	if (device->snapshot_memory.size < sizeof(struct kgsl_snapshot_header))
		device->snapshot_memory.size =
			sizeof(struct kgsl_snapshot_header);

	device->snapshot_memory.ptr = kzalloc(device->snapshot_memory.size,
		GFP_KERNEL);

	if (device->snapshot_memory.ptr == NULL)
		return -ENOMEM;

	device->snapshot = NULL;
	device->snapshot_faultcount = 0;
	device->force_panic = 0;

	ret = kobject_init_and_add(&device->snapshot_kobj, &ktype_snapshot,
		&device->dev->kobj, "snapshot");
	if (ret)
		goto done;

	ret = sysfs_create_bin_file(&device->snapshot_kobj, &snapshot_attr);
	if (ret)
		goto done;

	ret  = sysfs_create_file(&device->snapshot_kobj, &attr_timestamp.attr);
	if (ret)
		goto done;

	ret  = sysfs_create_file(&device->snapshot_kobj, &attr_faultcount.attr);
	if (ret)
		goto done;

	ret  = sysfs_create_file(&device->snapshot_kobj,
			&attr_force_panic.attr);
done:
	return ret;
}
EXPORT_SYMBOL(kgsl_device_snapshot_init);

void kgsl_device_snapshot_close(struct kgsl_device *device)
{
	sysfs_remove_bin_file(&device->snapshot_kobj, &snapshot_attr);
	sysfs_remove_file(&device->snapshot_kobj, &attr_timestamp.attr);

	kobject_put(&device->snapshot_kobj);

	kfree(device->snapshot_memory.ptr);

	device->snapshot_memory.ptr = NULL;
	device->snapshot_memory.size = 0;
	device->snapshot_faultcount = 0;
	device->force_panic = 0;
}
EXPORT_SYMBOL(kgsl_device_snapshot_close);

int kgsl_snapshot_add_ib_obj_list(struct kgsl_snapshot *snapshot,
	struct adreno_ib_object_list *ib_obj_list)
{
	struct kgsl_snapshot_cp_obj *obj;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		return -ENOMEM;
	obj->ib_obj_list = ib_obj_list;
	list_add(&obj->node, &snapshot->cp_list);
	return 0;
}

static size_t _mempool_add_object(struct kgsl_snapshot *snapshot, u8 *data,
		struct kgsl_snapshot_object *obj)
{
	struct kgsl_snapshot_section_header *section =
		(struct kgsl_snapshot_section_header *)data;
	struct kgsl_snapshot_gpu_object_v2 *header =
		(struct kgsl_snapshot_gpu_object_v2 *)(data + sizeof(*section));
	u8 *dest = data + sizeof(*section) + sizeof(*header);
	uint64_t size;

	size = obj->size;

	if (!kgsl_memdesc_map(&obj->entry->memdesc)) {
		KGSL_CORE_ERR("snapshot: failed to map GPU object\n");
		return 0;
	}

	section->magic = SNAPSHOT_SECTION_MAGIC;
	section->id = KGSL_SNAPSHOT_SECTION_GPU_OBJECT_V2;
	section->size = size + sizeof(*header) + sizeof(*section);

	header->size = size >> 2;
	header->gpuaddr = obj->gpuaddr;
	header->ptbase =
		kgsl_mmu_pagetable_get_ttbr0(obj->entry->priv->pagetable);
	header->type = obj->type;

	if (kgsl_addr_range_overlap(obj->gpuaddr, obj->size,
				snapshot->ib1base, snapshot->ib1size))
		snapshot->ib1dumped = true;

	if (kgsl_addr_range_overlap(obj->gpuaddr, obj->size,
				snapshot->ib2base, snapshot->ib2size))
		snapshot->ib2dumped = true;

	memcpy(dest, obj->entry->memdesc.hostptr + obj->offset, size);
	kgsl_memdesc_unmap(&obj->entry->memdesc);

	return section->size;
}

void kgsl_snapshot_save_frozen_objs(struct work_struct *work)
{
	struct kgsl_snapshot *snapshot = container_of(work,
				struct kgsl_snapshot, work);
	struct kgsl_device *device = kgsl_get_device(KGSL_DEVICE_3D0);
	struct kgsl_snapshot_object *obj, *tmp;
	size_t size = 0;
	void *ptr;

	if (IS_ERR_OR_NULL(device))
		return;

	kgsl_snapshot_process_ib_obj_list(snapshot);

	list_for_each_entry(obj, &snapshot->obj_list, node) {
		obj->size = ALIGN(obj->size, 4);

		size += ((size_t) obj->size +
			sizeof(struct kgsl_snapshot_gpu_object_v2) +
			sizeof(struct kgsl_snapshot_section_header));
	}

	if (size == 0)
		goto done;

	snapshot->mempool = vmalloc(size);

	ptr = snapshot->mempool;
	snapshot->mempool_size = 0;

	
	list_for_each_entry_safe(obj, tmp, &snapshot->obj_list, node) {
		if (snapshot->mempool) {
			size_t ret = _mempool_add_object(snapshot, ptr, obj);
			ptr += ret;
			snapshot->mempool_size += ret;
		}

		kgsl_snapshot_put_object(obj);
	}
done:
	kgsl_process_private_put(snapshot->process);
	snapshot->process = NULL;

	if (snapshot->ib1base && !snapshot->ib1dumped)
		KGSL_DRV_ERR(device,
				"snapshot: Active IB1:%016llx not dumped\n",
				snapshot->ib1base);
	else if (snapshot->ib2base && !snapshot->ib2dumped)
		KGSL_DRV_ERR(device,
			       "snapshot: Active IB2:%016llx not dumped\n",
				snapshot->ib2base);

	complete_all(&snapshot->dump_gate);
	BUG_ON(device->force_panic);
	return;
}
