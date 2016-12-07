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

#include "kgsl.h"
#include "kgsl_sharedmem.h"
#include "kgsl_snapshot.h"

#include "adreno.h"
#include "adreno_pm4types.h"
#include "a3xx_reg.h"
#include "adreno_cp_parser.h"
#include "adreno_snapshot.h"
#include "adreno_a5xx.h"

#define VPC_MEMORY_BANKS 4


#define SNAPSHOT_OBJ_BUFSIZE 64

#define SNAPSHOT_OBJ_TYPE_IB 0

static int ib_max_objs;

struct snapshot_rb_params {
	struct kgsl_snapshot *snapshot;
	struct adreno_ringbuffer *rb;
};

static size_t snapshot_frozen_objsize;

static struct kgsl_snapshot_object objbuf[SNAPSHOT_OBJ_BUFSIZE];

static unsigned int objbufptr;

static inline int adreno_rb_ctxtswitch(struct adreno_device *adreno_dev,
				   unsigned int *cmd)
{
	return cmd[0] == cp_packet(adreno_dev, CP_NOP, 1) &&
		cmd[1] == KGSL_CONTEXT_TO_MEM_IDENTIFIER;
}

static void push_object(int type,
	struct kgsl_process_private *process,
	uint64_t gpuaddr, uint64_t dwords)
{
	int index;
	struct kgsl_mem_entry *entry;

	if (process == NULL)
		return;


	for (index = 0; index < objbufptr; index++) {
		if (objbuf[index].gpuaddr == gpuaddr &&
			objbuf[index].entry->priv == process) {

			objbuf[index].size = max_t(uint64_t,
						objbuf[index].size,
						dwords << 2);
			return;
		}
	}

	if (objbufptr == SNAPSHOT_OBJ_BUFSIZE) {
		KGSL_CORE_ERR("snapshot: too many snapshot objects\n");
		return;
	}

	entry = kgsl_sharedmem_find(process, gpuaddr);
	if (entry == NULL) {
		KGSL_CORE_ERR("snapshot: Can't find entry for 0x%016llX\n",
			gpuaddr);
		return;
	}

	if (!kgsl_gpuaddr_in_memdesc(&entry->memdesc, gpuaddr, dwords << 2)) {
		KGSL_CORE_ERR("snapshot: Mem entry 0x%016llX is too small\n",
			gpuaddr);
		kgsl_mem_entry_put(entry);
		return;
	}

	
	objbuf[objbufptr].type = type;
	objbuf[objbufptr].gpuaddr = gpuaddr;
	objbuf[objbufptr].size = dwords << 2;
	objbuf[objbufptr++].entry = entry;
}


static int find_object(int type, uint64_t gpuaddr,
		struct kgsl_process_private *process)
{
	int index;

	for (index = 0; index < objbufptr; index++) {
		if (objbuf[index].gpuaddr == gpuaddr &&
			objbuf[index].entry->priv == process)
			return index;
	}
	return -ENOENT;
}

static int snapshot_freeze_obj_list(struct kgsl_snapshot *snapshot,
		struct kgsl_process_private *process,
		struct adreno_ib_object_list *ib_obj_list,
		uint64_t ib2base)
{
	int ret = 0;
	struct adreno_ib_object *ib_objs;
	int i;

	for (i = 0; i < ib_obj_list->num_objs; i++) {
		int temp_ret;
		int index;
		int freeze = 1;

		ib_objs = &(ib_obj_list->obj_list[i]);
		
		for (index = 0; index < objbufptr; index++) {
			if ((objbuf[index].gpuaddr <= ib_objs->gpuaddr) &&
				((objbuf[index].gpuaddr +
				(objbuf[index].size)) >=
				(ib_objs->gpuaddr + ib_objs->size)) &&
				(objbuf[index].entry->priv == process)) {
				freeze = 0;
				break;
			}
		}

		if (freeze) {
			
			if (ib2base == ib_objs->gpuaddr) {
				push_object(SNAPSHOT_OBJ_TYPE_IB,
				process, ib_objs->gpuaddr, ib_objs->size >> 2);
			} else {
				temp_ret = kgsl_snapshot_get_object(snapshot,
					process, ib_objs->gpuaddr,
					ib_objs->size,
					ib_objs->snapshot_obj_type);
				if (temp_ret < 0) {
					if (ret >= 0)
						ret = temp_ret;
				} else {
					snapshot_frozen_objsize += temp_ret;
				}
			}
		}
	}
	return ret;
}

static inline void parse_ib(struct kgsl_device *device,
		struct kgsl_snapshot *snapshot,
		struct kgsl_process_private *process,
		uint64_t gpuaddr, uint64_t dwords)
{
	struct adreno_ib_object_list *ib_obj_list;

	if (gpuaddr == snapshot->ib1base) {
		push_object(SNAPSHOT_OBJ_TYPE_IB, process,
			gpuaddr, dwords);
		return;
	}

	if (kgsl_snapshot_have_object(snapshot, process,
					gpuaddr, dwords << 2))
		return;

	if (-E2BIG == adreno_ib_create_object_list(device, process,
				gpuaddr, dwords, &ib_obj_list))
		ib_max_objs = 1;

	if (ib_obj_list)
		kgsl_snapshot_add_ib_obj_list(snapshot, ib_obj_list);

}

static inline bool iommu_is_setstate_addr(struct kgsl_device *device,
		uint64_t gpuaddr, uint64_t size)
{
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);

	if (kgsl_mmu_get_mmutype(device) != KGSL_MMU_TYPE_IOMMU)
		return false;

	return kgsl_gpuaddr_in_memdesc(&iommu->setstate, gpuaddr,
			size);
}

static void dump_all_ibs(struct kgsl_device *device,
			struct adreno_ringbuffer *rb,
			struct kgsl_snapshot *snapshot)
{
	int index = 0;
	unsigned int *rbptr;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	rbptr = rb->buffer_desc.hostptr;

	for (index = 0; index < KGSL_RB_DWORDS;) {

		if (adreno_cmd_is_ib(adreno_dev, rbptr[index])) {
			uint64_t ibaddr;
			uint64_t ibsize;

			if (ADRENO_LEGACY_PM4(adreno_dev)) {
				ibaddr = rbptr[index + 1];
				ibsize = rbptr[index + 2];
				index += 3;
			} else {
				ibaddr = rbptr[index + 2];
				ibaddr = ibaddr << 32 | rbptr[index + 1];
				ibsize = rbptr[index + 3];
				index += 4;
			}

			
			if (iommu_is_setstate_addr(device, ibaddr, ibsize))
				continue;

			if (kgsl_gpuaddr_in_memdesc(&adreno_dev->pwron_fixup,
				ibaddr, ibsize))
				continue;

			parse_ib(device, snapshot, snapshot->process, ibaddr,
				ibsize);
		} else
			index = index + 1;
	}
}

static void snapshot_rb_ibs(struct kgsl_device *device,
		struct adreno_ringbuffer *rb,
		struct kgsl_snapshot *snapshot)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int rptr, *rbptr;
	int index, i;
	int parse_ibs = 0, ib_parse_start;

	
	adreno_readreg(adreno_dev, ADRENO_REG_CP_RB_RPTR, &rptr);


	index = rptr;
	rbptr = rb->buffer_desc.hostptr;

	do {
		index--;

		if (index < 0) {
			if (ADRENO_LEGACY_PM4(adreno_dev))
				index = KGSL_RB_DWORDS - 3;
			else
				index = KGSL_RB_DWORDS - 4;

			
			if (index < rb->wptr) {
				index = rb->wptr;
				break;
			}
		}

		if (adreno_cmd_is_ib(adreno_dev, rbptr[index])) {
			if (ADRENO_LEGACY_PM4(adreno_dev)) {
				if (rbptr[index + 1] == snapshot->ib1base)
					break;
			} else {
				uint64_t ibaddr;

				ibaddr = rbptr[index + 2];
				ibaddr = ibaddr << 32 | rbptr[index + 1];
				if (ibaddr == snapshot->ib1base)
					break;
			}
		}
	} while (index != rb->wptr);


	if (index == rb->wptr) {
		dump_all_ibs(device, rb, snapshot);
		return;
	}


	while (index != rb->wptr) {
		index--;

		if (index < 0) {
			index = KGSL_RB_DWORDS - 2;


			if (index < rb->wptr) {
				index = rb->wptr;
				break;
			}
		}

		
		if ((rbptr[index] == cp_packet(adreno_dev, CP_NOP, 1)) &&
			(rbptr[index + 1] == KGSL_CONTEXT_TO_MEM_IDENTIFIER))
			break;
	}


	ib_parse_start = index;


	index = rb->wptr;
	for (i = 0; i < KGSL_RB_DWORDS; i++) {

		if (parse_ibs == 0 && index == ib_parse_start)
			parse_ibs = 1;
		else if (index == rptr || adreno_rb_ctxtswitch(adreno_dev,
							&rbptr[index]))
			parse_ibs = 0;

		if (parse_ibs && adreno_cmd_is_ib(adreno_dev, rbptr[index])) {
			uint64_t ibaddr;
			uint64_t ibsize;

			if (ADRENO_LEGACY_PM4(adreno_dev)) {
				ibaddr = rbptr[index + 1];
				ibsize = rbptr[index + 2];
			} else {
				ibaddr = rbptr[index + 2];
				ibaddr = ibaddr << 32 | rbptr[index + 1];
				ibsize = rbptr[index + 3];
			}

			
			if (iommu_is_setstate_addr(device, ibaddr, ibsize))
				continue;

			if (kgsl_gpuaddr_in_memdesc(&adreno_dev->pwron_fixup,
				ibaddr, ibsize))
				continue;

			parse_ib(device, snapshot, snapshot->process,
				ibaddr, ibsize);
		}

		index = (index + 1) % KGSL_RB_DWORDS;
	}

}

static size_t snapshot_rb(struct kgsl_device *device, u8 *buf,
	size_t remain, void *priv)
{
	struct kgsl_snapshot_rb_v2 *header = (struct kgsl_snapshot_rb_v2 *)buf;
	unsigned int *data = (unsigned int *)(buf + sizeof(*header));
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct snapshot_rb_params *snap_rb_params = priv;
	struct kgsl_snapshot *snapshot = snap_rb_params->snapshot;
	struct adreno_ringbuffer *rb = snap_rb_params->rb;


	if (remain < KGSL_RB_SIZE + sizeof(*header)) {
		KGSL_CORE_ERR("snapshot: Not enough memory for the rb section");
		return 0;
	}

	
	header->start = 0;
	header->end = KGSL_RB_DWORDS;
	header->wptr = rb->wptr;
	header->rptr = adreno_get_rptr(rb);
	header->rbsize = KGSL_RB_DWORDS;
	header->count = KGSL_RB_DWORDS;
	adreno_rb_readtimestamp(adreno_dev, rb, KGSL_TIMESTAMP_QUEUED,
					&header->timestamp_queued);
	adreno_rb_readtimestamp(adreno_dev, rb, KGSL_TIMESTAMP_RETIRED,
					&header->timestamp_retired);
	header->gpuaddr = rb->buffer_desc.gpuaddr;
	header->id = rb->id;

	if (rb == adreno_dev->cur_rb)
		snapshot_rb_ibs(device, rb, snapshot);

	
	memcpy(data, rb->buffer_desc.hostptr, KGSL_RB_SIZE);

	
	return KGSL_RB_SIZE + sizeof(*header);
}

static int _count_mem_entries(int id, void *ptr, void *data)
{
	int *count = data;
	*count = *count + 1;
	return 0;
}

struct mem_entry {
	uint64_t gpuaddr;
	uint64_t size;
	unsigned int type;
} __packed;

static int _save_mem_entries(int id, void *ptr, void *data)
{
	struct kgsl_mem_entry *entry = ptr;
	struct mem_entry *m = (struct mem_entry *) data;
	unsigned int index = id - 1;

	m[index].gpuaddr = entry->memdesc.gpuaddr;
	m[index].size = entry->memdesc.size;
	m[index].type = kgsl_memdesc_get_memtype(&entry->memdesc);

	return 0;
}

static size_t snapshot_capture_mem_list(struct kgsl_device *device,
		u8 *buf, size_t remain, void *priv)
{
	struct kgsl_snapshot_mem_list_v2 *header =
		(struct kgsl_snapshot_mem_list_v2 *)buf;
	int num_mem = 0;
	int ret = 0;
	unsigned int *data = (unsigned int *)(buf + sizeof(*header));
	struct kgsl_process_private *process = priv;

	
	if (process == NULL)
		return 0;

	spin_lock(&process->mem_lock);

	
	idr_for_each(&process->mem_idr, _count_mem_entries, &num_mem);

	if (num_mem == 0)
		goto out;

	if (remain < ((num_mem * sizeof(struct mem_entry)) + sizeof(*header))) {
		KGSL_CORE_ERR("snapshot: Not enough memory for the mem list");
		goto out;
	}

	header->num_entries = num_mem;
	header->ptbase = kgsl_mmu_pagetable_get_ttbr0(process->pagetable);

	idr_for_each(&process->mem_idr, _save_mem_entries, data);

	ret = sizeof(*header) + (num_mem * sizeof(struct mem_entry));
out:
	spin_unlock(&process->mem_lock);
	return ret;
}

struct snapshot_ib_meta {
	struct kgsl_snapshot *snapshot;
	struct kgsl_snapshot_object *obj;
	uint64_t ib1base;
	uint64_t ib1size;
	uint64_t ib2base;
	uint64_t ib2size;
};

void kgsl_snapshot_add_active_ib_obj_list(struct kgsl_device *device,
		struct kgsl_snapshot *snapshot)
{
	struct adreno_ib_object_list *ib_obj_list;
	int index = -ENOENT;

	if (!snapshot->ib1dumped)
		index = find_object(SNAPSHOT_OBJ_TYPE_IB, snapshot->ib1base,
				snapshot->process);

	
	if ((index != -ENOENT) &&
			(snapshot->ib1base == objbuf[index].gpuaddr)) {
		if (-E2BIG == adreno_ib_create_object_list(device,
					objbuf[index].entry->priv,
					objbuf[index].gpuaddr,
					objbuf[index].size >> 2,
					&ib_obj_list))
			ib_max_objs = 1;
		if (ib_obj_list) {
			
			snapshot_freeze_obj_list(snapshot,
					objbuf[index].entry->priv,
					ib_obj_list, snapshot->ib2base);
			adreno_ib_destroy_obj_list(ib_obj_list);
		}
	} else {
		
		index = find_object(SNAPSHOT_OBJ_TYPE_IB, snapshot->ib2base,
				snapshot->process);

		if (index != -ENOENT)
			parse_ib(device, snapshot, snapshot->process,
				snapshot->ib2base, objbuf[index].size >> 2);
	}
}

static bool active_ib_is_parsed(uint64_t gpuaddr, uint64_t size,
		struct kgsl_process_private *process)
{
	int  index;
	
	for (index = 0; index < objbufptr; index++) {
		if ((objbuf[index].gpuaddr <= gpuaddr) &&
				((objbuf[index].gpuaddr +
				  (objbuf[index].size)) >=
				 (gpuaddr + size)) &&
				(objbuf[index].entry->priv == process))
			return true;
	}
	return false;
}
static size_t snapshot_ib(struct kgsl_device *device, u8 *buf,
	size_t remain, void *priv)
{
	struct kgsl_snapshot_ib_v2 *header = (struct kgsl_snapshot_ib_v2 *)buf;
	struct snapshot_ib_meta *meta = priv;
	unsigned int *src;
	unsigned int *dst = (unsigned int *)(buf + sizeof(*header));
	struct adreno_ib_object_list *ib_obj_list;
	struct kgsl_snapshot *snapshot;
	struct kgsl_snapshot_object *obj;

	if (meta == NULL || meta->snapshot == NULL || meta->obj == NULL) {
		KGSL_CORE_ERR("snapshot: bad metadata");
		return 0;
	}
	snapshot = meta->snapshot;
	obj = meta->obj;

	if (remain < (obj->size + sizeof(*header))) {
		KGSL_CORE_ERR("snapshot: Not enough memory for the ib\n");
		return 0;
	}

	src = kgsl_gpuaddr_to_vaddr(&obj->entry->memdesc, obj->gpuaddr);
	if (src == NULL) {
		KGSL_DRV_ERR(device,
			"snapshot: Unable to map GPU memory object 0x%016llX into the kernel\n",
			obj->gpuaddr);
		return 0;
	}

	
	if (meta->ib1base == obj->gpuaddr) {

		snapshot->ib1dumped = active_ib_is_parsed(obj->gpuaddr,
					obj->size, obj->entry->priv);
		if (-E2BIG == adreno_ib_create_object_list(device,
				obj->entry->priv,
				obj->gpuaddr, obj->size >> 2,
				&ib_obj_list))
			ib_max_objs = 1;
		if (ib_obj_list) {
			
			snapshot_freeze_obj_list(snapshot,
						obj->entry->priv,
						ib_obj_list, meta->ib2base);
			adreno_ib_destroy_obj_list(ib_obj_list);
		}
	}


	if (meta->ib2base == obj->gpuaddr)
		snapshot->ib2dumped = active_ib_is_parsed(obj->gpuaddr,
					obj->size, obj->entry->priv);

	
	header->gpuaddr = obj->gpuaddr;
	header->ptbase =
		kgsl_mmu_pagetable_get_ttbr0(obj->entry->priv->pagetable);
	header->size = obj->size >> 2;

	
	memcpy((void *)dst, (void *)src, (size_t) obj->size);
	

	return obj->size + sizeof(*header);
}

static void dump_object(struct kgsl_device *device, int obj,
		struct kgsl_snapshot *snapshot)
{
	struct snapshot_ib_meta meta;

	switch (objbuf[obj].type) {
	case SNAPSHOT_OBJ_TYPE_IB:
		meta.snapshot = snapshot;
		meta.obj = &objbuf[obj];
		meta.ib1base = snapshot->ib1base;
		meta.ib1size = snapshot->ib1size;
		meta.ib2base = snapshot->ib2base;
		meta.ib2size = snapshot->ib2size;

		kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_IB_V2,
			snapshot, snapshot_ib, &meta);
		if (objbuf[obj].entry) {
			kgsl_memdesc_unmap(&(objbuf[obj].entry->memdesc));
			kgsl_mem_entry_put(objbuf[obj].entry);
		}
		break;
	default:
		KGSL_CORE_ERR("snapshot: Invalid snapshot object type: %d\n",
			objbuf[obj].type);
		break;
	}
}

static void setup_fault_process(struct kgsl_device *device,
				struct kgsl_snapshot *snapshot,
				struct kgsl_process_private *process)
{
	u64 hw_ptbase, proc_ptbase;

	if (process != NULL && !kgsl_process_private_get(process))
		process = NULL;

	
	hw_ptbase = kgsl_mmu_get_current_ttbr0(&device->mmu);

	
	if (process) {
		proc_ptbase = kgsl_mmu_pagetable_get_ttbr0(process->pagetable);
		
		if (hw_ptbase == proc_ptbase)
			goto done;

		kgsl_process_private_put(process);
		process = NULL;
		KGSL_CORE_ERR("snapshot: ptbase mismatch hw %llx sw %llx\n",
				hw_ptbase, proc_ptbase);
	}

	
	if (kgsl_mmu_is_perprocess(&device->mmu)) {
		struct kgsl_process_private *tmp;

		mutex_lock(&kgsl_driver.process_mutex);
		list_for_each_entry(tmp, &kgsl_driver.process_list, list) {
			u64 pt_ttbr0;

			pt_ttbr0 = kgsl_mmu_pagetable_get_ttbr0(tmp->pagetable);
			if ((pt_ttbr0 == hw_ptbase)
			    && kgsl_process_private_get(tmp)) {
				process = tmp;
				break;
			}
		}
		mutex_unlock(&kgsl_driver.process_mutex);
	}
done:
	snapshot->process = process;
}

static size_t snapshot_global(struct kgsl_device *device, u8 *buf,
	size_t remain, void *priv)
{
	struct kgsl_memdesc *memdesc = priv;

	struct kgsl_snapshot_gpu_object_v2 *header =
		(struct kgsl_snapshot_gpu_object_v2 *)buf;

	u8 *ptr = buf + sizeof(*header);

	if (memdesc->size == 0)
		return 0;

	if (remain < (memdesc->size + sizeof(*header))) {
		KGSL_CORE_ERR("snapshot: Not enough memory for the memdesc\n");
		return 0;
	}

	if (memdesc->hostptr == NULL) {
		KGSL_CORE_ERR("snapshot: no kernel mapping for global object 0x%016llX\n",
				memdesc->gpuaddr);
		return 0;
	}

	header->size = memdesc->size >> 2;
	header->gpuaddr = memdesc->gpuaddr;
	header->ptbase = MMU_DEFAULT_TTBR0(device);
	header->type = SNAPSHOT_GPU_OBJECT_GLOBAL;

	memcpy(ptr, memdesc->hostptr, memdesc->size);

	return memdesc->size + sizeof(*header);
}

static void adreno_snapshot_iommu(struct kgsl_device *device,
		struct kgsl_snapshot *snapshot)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);

	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_GPU_OBJECT_V2,
		snapshot, snapshot_global, &iommu->setstate);

	if (ADRENO_FEATURE(adreno_dev, ADRENO_PREEMPTION))
		kgsl_snapshot_add_section(device,
			KGSL_SNAPSHOT_SECTION_GPU_OBJECT_V2,
			snapshot, snapshot_global, &iommu->smmu_info);
}

static void adreno_snapshot_ringbuffer(struct kgsl_device *device,
		struct kgsl_snapshot *snapshot, struct adreno_ringbuffer *rb)
{
	struct snapshot_rb_params params = {
		.snapshot = snapshot,
		.rb = rb,
	};

	if (rb == NULL)
		return;

	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_RB_V2, snapshot,
		snapshot_rb, &params);
}

void adreno_snapshot(struct kgsl_device *device, struct kgsl_snapshot *snapshot,
			struct kgsl_context *context)
{
	unsigned int i;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);

	ib_max_objs = 0;
	
	objbufptr = 0;

	snapshot_frozen_objsize = 0;

	setup_fault_process(device, snapshot,
			context ? context->proc_priv : NULL);

	adreno_readreg64(adreno_dev, ADRENO_REG_CP_IB1_BASE,
			ADRENO_REG_CP_IB1_BASE_HI, &snapshot->ib1base);
	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB1_BUFSZ, &snapshot->ib1size);
	adreno_readreg64(adreno_dev, ADRENO_REG_CP_IB2_BASE,
			ADRENO_REG_CP_IB2_BASE_HI, &snapshot->ib2base);
	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB2_BUFSZ, &snapshot->ib2size);

	snapshot->ib1dumped = false;
	snapshot->ib2dumped = false;

	adreno_snapshot_ringbuffer(device, snapshot, adreno_dev->cur_rb);

	
	if (adreno_dev->prev_rb != adreno_dev->cur_rb)
		adreno_snapshot_ringbuffer(device, snapshot,
			adreno_dev->prev_rb);

	if ((adreno_dev->next_rb != adreno_dev->prev_rb) &&
		 (adreno_dev->next_rb != adreno_dev->cur_rb))
		adreno_snapshot_ringbuffer(device, snapshot,
			adreno_dev->next_rb);

	
	if (gpudev->snapshot)
		gpudev->snapshot(adreno_dev, snapshot);

	
	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_GPU_OBJECT_V2,
			snapshot, snapshot_global, &device->memstore);

	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_GPU_OBJECT_V2,
			snapshot, snapshot_global,
			&adreno_dev->pwron_fixup);

	if (kgsl_mmu_get_mmutype(device) == KGSL_MMU_TYPE_IOMMU)
		adreno_snapshot_iommu(device, snapshot);

	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_MEMLIST_V2,
			snapshot, snapshot_capture_mem_list, snapshot->process);

	if (-ENOENT == find_object(SNAPSHOT_OBJ_TYPE_IB, snapshot->ib1base,
			snapshot->process) && snapshot->ib1size) {
		push_object(SNAPSHOT_OBJ_TYPE_IB, snapshot->process,
			snapshot->ib1base, snapshot->ib1size);
		KGSL_CORE_ERR(
		"CP_IB1_BASE not found in the ringbuffer.Dumping %x dwords of the buffer.\n",
		snapshot->ib1size);
	}


	if (-ENOENT == find_object(SNAPSHOT_OBJ_TYPE_IB, snapshot->ib2base,
		snapshot->process)) {
		push_object(SNAPSHOT_OBJ_TYPE_IB, snapshot->process,
			snapshot->ib2base, snapshot->ib2size);
	}

	for (i = 0; i < objbufptr; i++)
		dump_object(device, i, snapshot);

	if (!snapshot->ib1dumped || !snapshot->ib2dumped)
		kgsl_snapshot_add_active_ib_obj_list(device, snapshot);

	if (ib_max_objs)
		KGSL_CORE_ERR("Max objects found in IB\n");
	if (snapshot_frozen_objsize)
		KGSL_CORE_ERR("GPU snapshot froze %zdKb of GPU buffers\n",
			snapshot_frozen_objsize / 1024);

}

size_t adreno_snapshot_cp_merciu(struct kgsl_device *device, u8 *buf,
		size_t remain, void *priv)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_snapshot_debug *header = (struct kgsl_snapshot_debug *)buf;
	unsigned int *data = (unsigned int *)(buf + sizeof(*header));
	int i, size = *((int *)priv);

	
	size = size << 1;

	if (remain < DEBUG_SECTION_SZ(size)) {
		SNAPSHOT_ERR_NOMEM(device, "CP MERCIU DEBUG");
		return 0;
	}

	header->type = SNAPSHOT_DEBUG_CP_MERCIU;
	header->size = size;

	adreno_writereg(adreno_dev, ADRENO_REG_CP_MERCIU_ADDR, 0x0);

	for (i = 0; i < size; i++) {
		adreno_readreg(adreno_dev, ADRENO_REG_CP_MERCIU_DATA,
			&data[(i * 2)]);
		adreno_readreg(adreno_dev, ADRENO_REG_CP_MERCIU_DATA2,
			&data[(i * 2) + 1]);
	}

	return DEBUG_SECTION_SZ(size);
}

size_t adreno_snapshot_cp_roq(struct kgsl_device *device, u8 *buf,
		size_t remain, void *priv)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_snapshot_debug *header = (struct kgsl_snapshot_debug *)buf;
	unsigned int *data = (unsigned int *)(buf + sizeof(*header));
	int i, size = *((int *)priv);

	if (remain < DEBUG_SECTION_SZ(size)) {
		SNAPSHOT_ERR_NOMEM(device, "CP ROQ DEBUG");
		return 0;
	}

	header->type = SNAPSHOT_DEBUG_CP_ROQ;
	header->size = size;

	adreno_writereg(adreno_dev, ADRENO_REG_CP_ROQ_ADDR, 0x0);
	for (i = 0; i < size; i++)
		adreno_readreg(adreno_dev, ADRENO_REG_CP_ROQ_DATA, &data[i]);

	return DEBUG_SECTION_SZ(size);
}

size_t adreno_snapshot_cp_pm4_ram(struct kgsl_device *device, u8 *buf,
		size_t remain, void *priv)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_snapshot_debug *header = (struct kgsl_snapshot_debug *)buf;
	unsigned int *data = (unsigned int *)(buf + sizeof(*header));
	int i;
	size_t size = adreno_dev->pm4_fw_size - 1;

	if (remain < DEBUG_SECTION_SZ(size)) {
		SNAPSHOT_ERR_NOMEM(device, "CP PM4 RAM DEBUG");
		return 0;
	}

	header->type = SNAPSHOT_DEBUG_CP_PM4_RAM;
	header->size = size;


	adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_RAM_RADDR, 0x0);
	for (i = 0; i < size; i++)
		adreno_readreg(adreno_dev, ADRENO_REG_CP_ME_RAM_DATA, &data[i]);

	return DEBUG_SECTION_SZ(size);
}

size_t adreno_snapshot_cp_pfp_ram(struct kgsl_device *device, u8 *buf,
		size_t remain, void *priv)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_snapshot_debug *header = (struct kgsl_snapshot_debug *)buf;
	unsigned int *data = (unsigned int *)(buf + sizeof(*header));
	int i, size = adreno_dev->pfp_fw_size - 1;

	if (remain < DEBUG_SECTION_SZ(size)) {
		SNAPSHOT_ERR_NOMEM(device, "CP PFP RAM DEBUG");
		return 0;
	}

	header->type = SNAPSHOT_DEBUG_CP_PFP_RAM;
	header->size = size;

	adreno_writereg(adreno_dev, ADRENO_REG_CP_PFP_UCODE_ADDR, 0x0);
	for (i = 0; i < size; i++)
		adreno_readreg(adreno_dev, ADRENO_REG_CP_PFP_UCODE_DATA,
				&data[i]);

	return DEBUG_SECTION_SZ(size);
}

size_t adreno_snapshot_vpc_memory(struct kgsl_device *device, u8 *buf,
		size_t remain, void *priv)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_snapshot_debug *header = (struct kgsl_snapshot_debug *)buf;
	unsigned int *data = (unsigned int *)(buf + sizeof(*header));
	int vpc_mem_size = *((int *)priv);
	size_t size = VPC_MEMORY_BANKS * vpc_mem_size;
	int bank, addr, i = 0;

	if (remain < DEBUG_SECTION_SZ(size)) {
		SNAPSHOT_ERR_NOMEM(device, "VPC MEMORY");
		return 0;
	}

	header->type = SNAPSHOT_DEBUG_VPC_MEMORY;
	header->size = size;

	for (bank = 0; bank < VPC_MEMORY_BANKS; bank++) {
		for (addr = 0; addr < vpc_mem_size; addr++) {
			unsigned int val = bank | (addr << 4);
			adreno_writereg(adreno_dev,
				ADRENO_REG_VPC_DEBUG_RAM_SEL, val);
			adreno_readreg(adreno_dev,
				ADRENO_REG_VPC_DEBUG_RAM_READ, &data[i++]);
		}
	}

	return DEBUG_SECTION_SZ(size);
}

size_t adreno_snapshot_cp_meq(struct kgsl_device *device, u8 *buf,
		size_t remain, void *priv)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_snapshot_debug *header = (struct kgsl_snapshot_debug *)buf;
	unsigned int *data = (unsigned int *)(buf + sizeof(*header));
	int i;
	int cp_meq_sz = *((int *)priv);

	if (remain < DEBUG_SECTION_SZ(cp_meq_sz)) {
		SNAPSHOT_ERR_NOMEM(device, "CP MEQ DEBUG");
		return 0;
	}

	header->type = SNAPSHOT_DEBUG_CP_MEQ;
	header->size = cp_meq_sz;

	adreno_writereg(adreno_dev, ADRENO_REG_CP_MEQ_ADDR, 0x0);
	for (i = 0; i < cp_meq_sz; i++)
		adreno_readreg(adreno_dev, ADRENO_REG_CP_MEQ_DATA, &data[i]);

	return DEBUG_SECTION_SZ(cp_meq_sz);
}

static const struct adreno_vbif_snapshot_registers *vbif_registers(
		struct adreno_device *adreno_dev,
		const struct adreno_vbif_snapshot_registers *list,
		unsigned int count)
{
	unsigned int version;
	unsigned int i;

	adreno_readreg(adreno_dev, ADRENO_REG_VBIF_VERSION, &version);

	for (i = 0; i < count; i++) {
		if ((list[i].version & list[i].mask) ==
				(version & list[i].mask))
			return &list[i];
	}

	KGSL_CORE_ERR(
		"snapshot: Registers for VBIF version %X register were not dumped\n",
		version);

	return NULL;
}

void adreno_snapshot_registers(struct kgsl_device *device,
		struct kgsl_snapshot *snapshot,
		const unsigned int *regs, unsigned int count)
{
	struct kgsl_snapshot_registers r;

	r.regs = regs;
	r.count = count;

	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_REGS, snapshot,
		kgsl_snapshot_dump_registers, &r);
}

void adreno_snapshot_vbif_registers(struct kgsl_device *device,
		struct kgsl_snapshot *snapshot,
		const struct adreno_vbif_snapshot_registers *list,
		unsigned int count)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_snapshot_registers regs;
	const struct adreno_vbif_snapshot_registers *vbif;

	vbif = vbif_registers(adreno_dev, list, count);

	if (vbif != NULL) {
		regs.regs = vbif->registers;
		regs.count = vbif->count;

		kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_REGS,
			snapshot, kgsl_snapshot_dump_registers, &regs);
	}
}
