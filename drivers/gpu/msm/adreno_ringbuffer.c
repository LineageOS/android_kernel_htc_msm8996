/* Copyright (c) 2002,2007-2016, The Linux Foundation. All rights reserved.
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
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/log2.h>
#include <linux/time.h>
#include <linux/delay.h>

#include "kgsl.h"
#include "kgsl_sharedmem.h"
#include "kgsl_cffdump.h"
#include "kgsl_trace.h"
#include "kgsl_pwrctrl.h"

#include "adreno.h"
#include "adreno_iommu.h"
#include "adreno_pm4types.h"
#include "adreno_ringbuffer.h"

#include "a3xx_reg.h"
#include "adreno_a5xx.h"

#define RB_HOSTPTR(_rb, _pos) \
	((unsigned int *) ((_rb)->buffer_desc.hostptr + \
		((_pos) * sizeof(unsigned int))))

#define RB_GPUADDR(_rb, _pos) \
	((_rb)->buffer_desc.gpuaddr + ((_pos) * sizeof(unsigned int)))

static void _cff_write_ringbuffer(struct adreno_ringbuffer *rb)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	uint64_t gpuaddr;
	unsigned int *hostptr;
	size_t size;

	if (device->cff_dump_enable == 0)
		return;

	BUG_ON(rb->_wptr < rb->last_wptr);

	size = (rb->_wptr - rb->last_wptr) * sizeof(unsigned int);

	hostptr = RB_HOSTPTR(rb, rb->last_wptr);
	gpuaddr = RB_GPUADDR(rb, rb->last_wptr);

	kgsl_cffdump_memcpy(device, gpuaddr, hostptr, size);
	rb->last_wptr = rb->_wptr;
}

static void adreno_get_submit_time(struct adreno_device *adreno_dev,
		struct adreno_submit_time *time)
{
	unsigned long flags;

	local_irq_save(flags);

	
	if (!adreno_is_a3xx(adreno_dev)) {
		adreno_readreg64(adreno_dev,
			ADRENO_REG_RBBM_ALWAYSON_COUNTER_LO,
			ADRENO_REG_RBBM_ALWAYSON_COUNTER_HI,
			&time->ticks);

		
		if (ADRENO_GPUREV(adreno_dev) >= 400 &&
				ADRENO_GPUREV(adreno_dev) <= ADRENO_REV_A530)
				time->ticks &= 0xFFFFFFFF;
	} else
		time->ticks = 0;

	
	time->ktime = local_clock();

	
	getnstimeofday(&time->utime);

	local_irq_restore(flags);
}

void adreno_ringbuffer_wptr(struct adreno_device *adreno_dev,
		struct adreno_ringbuffer *rb)
{
	unsigned long flags;

	spin_lock_irqsave(&rb->preempt_lock, flags);
	if (adreno_in_preempt_state(adreno_dev, ADRENO_PREEMPT_NONE)) {

		if (adreno_dev->cur_rb == rb) {
			kgsl_pwrscale_busy(KGSL_DEVICE(adreno_dev));
			adreno_writereg(adreno_dev, ADRENO_REG_CP_RB_WPTR,
				rb->_wptr);
		}
	}

	rb->wptr = rb->_wptr;
	spin_unlock_irqrestore(&rb->preempt_lock, flags);
}

void adreno_ringbuffer_submit(struct adreno_ringbuffer *rb,
		struct adreno_submit_time *time)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);

	
	_cff_write_ringbuffer(rb);

	if (time != NULL)
		adreno_get_submit_time(adreno_dev, time);

	adreno_ringbuffer_wptr(adreno_dev, rb);
}

int adreno_ringbuffer_submit_spin(struct adreno_ringbuffer *rb,
		struct adreno_submit_time *time, unsigned int timeout)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);

	adreno_ringbuffer_submit(rb, time);
	return adreno_spin_idle(adreno_dev, timeout);
}

unsigned int *adreno_ringbuffer_allocspace(struct adreno_ringbuffer *rb,
		unsigned int dwords)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	unsigned int rptr = adreno_get_rptr(rb);
	unsigned int ret;

	if (rptr <= rb->_wptr) {
		unsigned int *cmds;

		if (rb->_wptr + dwords <= (KGSL_RB_DWORDS - 2)) {
			ret = rb->_wptr;
			rb->_wptr = (rb->_wptr + dwords) % KGSL_RB_DWORDS;
			return RB_HOSTPTR(rb, ret);
		}

		if (dwords < rptr) {
			cmds = RB_HOSTPTR(rb, rb->_wptr);
			*cmds = cp_packet(adreno_dev, CP_NOP,
				KGSL_RB_DWORDS - rb->_wptr - 1);
			rb->_wptr = dwords;
			return RB_HOSTPTR(rb, 0);
		}
	}

	if (rb->_wptr + dwords < rptr) {
		ret = rb->_wptr;
		rb->_wptr = (rb->_wptr + dwords) % KGSL_RB_DWORDS;
		return RB_HOSTPTR(rb, ret);
	}

	return ERR_PTR(-ENOSPC);
}

int adreno_ringbuffer_start(struct adreno_device *adreno_dev,
	unsigned int start_type)
{
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_ringbuffer *rb;
	int i;

	
	FOR_EACH_RINGBUFFER(adreno_dev, rb, i) {
		kgsl_sharedmem_set(device, &(rb->buffer_desc),
				0, 0xAA, KGSL_RB_SIZE);
		kgsl_sharedmem_writel(device, &device->scratch,
				SCRATCH_RPTR_OFFSET(rb->id), 0);
		rb->wptr = 0;
		rb->_wptr = 0;
		rb->wptr_preempt_end = 0xFFFFFFFF;
		rb->starve_timer_state =
			ADRENO_DISPATCHER_RB_STARVE_TIMER_UNINIT;
	}

	
	return gpudev->rb_start(adreno_dev, start_type);
}

void adreno_ringbuffer_stop(struct adreno_device *adreno_dev)
{
	struct adreno_ringbuffer *rb;
	int i;

	FOR_EACH_RINGBUFFER(adreno_dev, rb, i)
		kgsl_cancel_events(KGSL_DEVICE(adreno_dev), &(rb->events));
}

static int _rb_readtimestamp(struct kgsl_device *device,
		void *priv, enum kgsl_timestamp_type type,
		unsigned int *timestamp)
{
	return adreno_rb_readtimestamp(ADRENO_DEVICE(device), priv, type,
		timestamp);
}

static int _adreno_ringbuffer_probe(struct adreno_device *adreno_dev,
		int id)
{
	struct adreno_ringbuffer *rb = &adreno_dev->ringbuffers[id];
	int ret;
	char name[64];

	rb->id = id;

	snprintf(name, sizeof(name), "rb_events-%d", id);
	kgsl_add_event_group(&rb->events, NULL, name,
		_rb_readtimestamp, rb);
	rb->timestamp = 0;
	init_waitqueue_head(&rb->ts_expire_waitq);

	spin_lock_init(&rb->preempt_lock);

	ret = kgsl_allocate_global(KGSL_DEVICE(adreno_dev), &rb->pagetable_desc,
		PAGE_SIZE, 0, KGSL_MEMDESC_PRIVILEGED);
	if (ret)
		return ret;

	return kgsl_allocate_global(KGSL_DEVICE(adreno_dev), &rb->buffer_desc,
			KGSL_RB_SIZE, KGSL_MEMFLAGS_GPUREADONLY, 0);
}

int adreno_ringbuffer_probe(struct adreno_device *adreno_dev, bool nopreempt)
{
	int status = 0;
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	int i;

	if (nopreempt == false && ADRENO_FEATURE(adreno_dev, ADRENO_PREEMPTION))
		adreno_dev->num_ringbuffers = gpudev->num_prio_levels;
	else
		adreno_dev->num_ringbuffers = 1;

	for (i = 0; i < adreno_dev->num_ringbuffers; i++) {
		status = _adreno_ringbuffer_probe(adreno_dev, i);
		if (status != 0)
			break;
	}

	if (status)
		adreno_ringbuffer_close(adreno_dev);
	else
		adreno_dev->cur_rb = &(adreno_dev->ringbuffers[0]);

	return status;
}

static void _adreno_ringbuffer_close(struct adreno_device *adreno_dev,
		struct adreno_ringbuffer *rb)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	kgsl_free_global(device, &rb->pagetable_desc);
	kgsl_free_global(device, &rb->preemption_desc);

	kgsl_free_global(device, &rb->buffer_desc);
	kgsl_del_event_group(&rb->events);
	memset(rb, 0, sizeof(struct adreno_ringbuffer));
}

void adreno_ringbuffer_close(struct adreno_device *adreno_dev)
{
	struct adreno_ringbuffer *rb;
	int i;

	FOR_EACH_RINGBUFFER(adreno_dev, rb, i)
		_adreno_ringbuffer_close(adreno_dev, rb);
}

int cp_secure_mode(struct adreno_device *adreno_dev, uint *cmds,
				int set)
{
	uint *start = cmds;

	if (adreno_is_a4xx(adreno_dev)) {
		cmds += cp_wait_for_idle(adreno_dev, cmds);
		cmds += cp_wait_for_me(adreno_dev, cmds);

		*cmds++ = cp_packet(adreno_dev, CP_SET_PROTECTED_MODE, 1);
		*cmds++ = 0;
		*cmds++ = cp_packet(adreno_dev, CP_WIDE_REG_WRITE, 2);
		*cmds++ = adreno_getreg(adreno_dev,
				ADRENO_REG_RBBM_SECVID_TRUST_CONTROL);
		*cmds++ = set;
		*cmds++ = cp_packet(adreno_dev, CP_SET_PROTECTED_MODE, 1);
		*cmds++ = 1;

		
		cmds += cp_wait_for_me(adreno_dev, cmds);
	} else {
		*cmds++ = cp_packet(adreno_dev, CP_SET_SECURE_MODE, 1);
		*cmds++ = set;
	}

	return cmds - start;
}

static inline int cp_mem_write(struct adreno_device *adreno_dev,
		unsigned int *cmds, uint64_t gpuaddr, unsigned int value)
{
	int dwords = 0;

	cmds[dwords++] = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	dwords += cp_gpuaddr(adreno_dev, &cmds[dwords], gpuaddr);
	cmds[dwords++] = value;

	return dwords;
}

static int
adreno_ringbuffer_addcmds(struct adreno_ringbuffer *rb,
				unsigned int flags, unsigned int *cmds,
				unsigned int sizedwords, uint32_t timestamp,
				struct adreno_submit_time *time)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	unsigned int *ringcmds, *start;
	unsigned int total_sizedwords = sizedwords;
	unsigned int i;
	unsigned int context_id = 0;
	bool profile_ready;
	struct adreno_context *drawctxt = rb->drawctxt_active;
	struct kgsl_context *context = NULL;
	bool secured_ctxt = false;
	static unsigned int _seq_cnt;

	if (drawctxt != NULL && kgsl_context_detached(&drawctxt->base) &&
		!(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE))
		return -ENOENT;

	
	if (adreno_gpu_fault(adreno_dev) != 0)
		return -EPROTO;

	rb->timestamp++;

	
	if (!drawctxt || (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE))
		timestamp = rb->timestamp;
	else {
		context_id = drawctxt->base.id;
		context = &drawctxt->base;
	}

	if (drawctxt) {
		drawctxt->internal_timestamp = rb->timestamp;
		if (drawctxt->base.flags & KGSL_CONTEXT_SECURE)
			secured_ctxt = true;
	}

	profile_ready = drawctxt &&
		adreno_profile_assignments_ready(&adreno_dev->profile) &&
		!(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE);

	total_sizedwords += flags & KGSL_CMD_FLAGS_PMODE ? 4 : 0;
	
	total_sizedwords += 2;
	
	total_sizedwords += (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE) ? 2 : 0;

	total_sizedwords += (secured_ctxt) ? 26 : 0;

	
	total_sizedwords += 4;

	
	if (adreno_is_a3xx(adreno_dev))
		total_sizedwords += 3;

	
	if (adreno_is_a4xx(adreno_dev) || adreno_is_a3xx(adreno_dev))
		total_sizedwords += 4;

	if (gpudev->preemption_pre_ibsubmit &&
				adreno_is_preemption_enabled(adreno_dev))
		total_sizedwords += 22;

	if (gpudev->preemption_post_ibsubmit &&
				adreno_is_preemption_enabled(adreno_dev))
		total_sizedwords += 5;

	total_sizedwords += 8; 
	total_sizedwords += 5; 

	if (drawctxt && !(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE)) {
		total_sizedwords += 4; 
	}

	if (flags & KGSL_CMD_FLAGS_WFI)
		total_sizedwords += 2; 

	if (profile_ready)
		total_sizedwords += 8;   

	
	if (flags & KGSL_CMD_FLAGS_PWRON_FIXUP)
		total_sizedwords += 9;

	if (test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE,
				&adreno_dev->ft_pf_policy))
		total_sizedwords += 1;

	ringcmds = adreno_ringbuffer_allocspace(rb, total_sizedwords);
	if (IS_ERR(ringcmds))
		return PTR_ERR(ringcmds);

	start = ringcmds;

	*ringcmds++ = cp_packet(adreno_dev, CP_NOP, 1);
	*ringcmds++ = KGSL_CMD_IDENTIFIER;

	if (adreno_is_preemption_enabled(adreno_dev) &&
				gpudev->preemption_pre_ibsubmit)
		ringcmds += gpudev->preemption_pre_ibsubmit(
					adreno_dev, rb, ringcmds, context);

	if (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE) {
		*ringcmds++ = cp_packet(adreno_dev, CP_NOP, 1);
		*ringcmds++ = KGSL_CMD_INTERNAL_IDENTIFIER;
	}

	if (flags & KGSL_CMD_FLAGS_PWRON_FIXUP) {
		
		*ringcmds++ = cp_packet(adreno_dev, CP_SET_PROTECTED_MODE, 1);
		*ringcmds++ = 0;

		*ringcmds++ = cp_packet(adreno_dev, CP_NOP, 1);
		*ringcmds++ = KGSL_PWRON_FIXUP_IDENTIFIER;
		*ringcmds++ = cp_mem_packet(adreno_dev,
				CP_INDIRECT_BUFFER_PFE, 2, 1);
		ringcmds += cp_gpuaddr(adreno_dev, ringcmds,
				adreno_dev->pwron_fixup.gpuaddr);
		*ringcmds++ = adreno_dev->pwron_fixup_dwords;

		
		*ringcmds++ = cp_packet(adreno_dev, CP_SET_PROTECTED_MODE, 1);
		*ringcmds++ = 1;
	}

	
	if (profile_ready)
		adreno_profile_preib_processing(adreno_dev, drawctxt,
				&flags, &ringcmds);

	
	if (drawctxt && !(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE))
		ringcmds += cp_mem_write(adreno_dev, ringcmds,
			MEMSTORE_ID_GPU_ADDR(device, context_id, soptimestamp),
			timestamp);

	
	ringcmds += cp_mem_write(adreno_dev, ringcmds,
		MEMSTORE_RB_GPU_ADDR(device, rb, soptimestamp), rb->timestamp);

	if (secured_ctxt)
		ringcmds += cp_secure_mode(adreno_dev, ringcmds, 1);

	if (flags & KGSL_CMD_FLAGS_PMODE) {
		
		*ringcmds++ = cp_packet(adreno_dev, CP_SET_PROTECTED_MODE, 1);
		*ringcmds++ = 0;
	}

	for (i = 0; i < sizedwords; i++)
		*ringcmds++ = cmds[i];

	if (flags & KGSL_CMD_FLAGS_PMODE) {
		
		*ringcmds++ = cp_packet(adreno_dev, CP_SET_PROTECTED_MODE, 1);
		*ringcmds++ = 1;
	}

	if (adreno_is_a4xx(adreno_dev) || adreno_is_a3xx(adreno_dev)) {
		*ringcmds++ = cp_packet(adreno_dev, CP_EVENT_WRITE, 1);
		*ringcmds++ = 0x07; 
		ringcmds += cp_wait_for_idle(adreno_dev, ringcmds);
	}

	if (profile_ready)
		adreno_profile_postib_processing(adreno_dev, &flags, &ringcmds);

	if (test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE,
				&adreno_dev->ft_pf_policy))
		*ringcmds++ = cp_packet(adreno_dev, CP_WAIT_MEM_WRITES, 0);

	ringcmds += cp_mem_write(adreno_dev, ringcmds,
		MEMSTORE_ID_GPU_ADDR(device, KGSL_MEMSTORE_GLOBAL,
			ref_wait_ts), ++_seq_cnt);

	*ringcmds++ = cp_mem_packet(adreno_dev, CP_EVENT_WRITE, 3, 1);
	if (drawctxt || (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE))
		*ringcmds++ = CACHE_FLUSH_TS | (1 << 31);
	else
		*ringcmds++ = CACHE_FLUSH_TS;

	if (drawctxt && !(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE)) {
		ringcmds += cp_gpuaddr(adreno_dev, ringcmds,
			MEMSTORE_ID_GPU_ADDR(device, context_id, eoptimestamp));
		*ringcmds++ = timestamp;

		
		ringcmds += cp_mem_write(adreno_dev, ringcmds,
			MEMSTORE_RB_GPU_ADDR(device, rb, eoptimestamp),
			rb->timestamp);
	} else {
		ringcmds += cp_gpuaddr(adreno_dev, ringcmds,
			MEMSTORE_RB_GPU_ADDR(device, rb, eoptimestamp));
		*ringcmds++ = timestamp;
	}

	if (adreno_is_a3xx(adreno_dev)) {
		
		*ringcmds++ = cp_packet(adreno_dev, CP_SET_CONSTANT, 2);
		*ringcmds++ =
			(0x4<<16) | (A3XX_HLSQ_CL_KERNEL_GROUP_X_REG - 0x2000);
		*ringcmds++ = 0;
	}

	if (flags & KGSL_CMD_FLAGS_WFI) {
		ringcmds += cp_wait_for_idle(adreno_dev, ringcmds);
	}

	if (secured_ctxt)
		ringcmds += cp_secure_mode(adreno_dev, ringcmds, 0);

	if (gpudev->preemption_post_ibsubmit &&
				adreno_is_preemption_enabled(adreno_dev))
		ringcmds += gpudev->preemption_post_ibsubmit(adreno_dev,
			ringcmds);

	if ((ringcmds - start) > total_sizedwords)
		BUG();
	rb->_wptr = rb->_wptr - (total_sizedwords - (ringcmds - start));

	adreno_ringbuffer_submit(rb, time);

	return 0;
}

int
adreno_ringbuffer_issuecmds(struct adreno_ringbuffer *rb,
				unsigned int flags,
				unsigned int *cmds,
				int sizedwords)
{
	flags |= KGSL_CMD_FLAGS_INTERNAL_ISSUE;

	return adreno_ringbuffer_addcmds(rb, flags, cmds,
		sizedwords, 0, NULL);
}

static inline bool _ringbuffer_verify_ib(struct kgsl_device_private *dev_priv,
		struct kgsl_context *context, struct kgsl_memobj_node *ib)
{
	struct kgsl_device *device = dev_priv->device;
	struct kgsl_process_private *private = dev_priv->process_priv;

	
	if (ib->size == 0 || ((ib->size >> 2) > 0xFFFFF)) {
		pr_context(device, context, "ctxt %d invalid ib size %lld\n",
			context->id, ib->size);
		return false;
	}

	
	if (!kgsl_mmu_gpuaddr_in_range(private->pagetable, ib->gpuaddr)) {
		pr_context(device, context, "ctxt %d invalid ib gpuaddr %llX\n",
			context->id, ib->gpuaddr);
		return false;
	}

	return true;
}

int
adreno_ringbuffer_issueibcmds(struct kgsl_device_private *dev_priv,
				struct kgsl_context *context,
				struct kgsl_cmdbatch *cmdbatch,
				uint32_t *timestamp)
{
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_context *drawctxt = ADRENO_CONTEXT(context);
	struct kgsl_memobj_node *ib;
	int ret;

	if (kgsl_context_invalid(context))
		return -EDEADLK;

	
	list_for_each_entry(ib, &cmdbatch->cmdlist, node)
		if (_ringbuffer_verify_ib(dev_priv, context, ib) == false)
			return -EINVAL;

	
	wait_for_completion(&device->cmdbatch_gate);

	if (!(cmdbatch->flags & KGSL_CMDBATCH_MARKER)
		&& !(cmdbatch->flags & KGSL_CMDBATCH_SYNC))
		device->flags &= ~KGSL_FLAG_WAKE_ON_TOUCH;

	
	if (adreno_is_a3xx(adreno_dev) &&
			(cmdbatch->flags & KGSL_CMDBATCH_PROFILING))
		return -EOPNOTSUPP;

	
	ret = adreno_dispatcher_queue_cmd(adreno_dev, drawctxt, cmdbatch,
		timestamp);

	if (!ret && test_and_clear_bit(ADRENO_CONTEXT_FAULT, &context->priv))
		ret = -EPROTO;

	return ret;
}

static void adreno_ringbuffer_set_constraint(struct kgsl_device *device,
			struct kgsl_cmdbatch *cmdbatch)
{
	struct kgsl_context *context = cmdbatch->context;
	if (context->pwr_constraint.type &&
		((context->flags & KGSL_CONTEXT_PWR_CONSTRAINT) ||
			(cmdbatch->flags & KGSL_CONTEXT_PWR_CONSTRAINT)))
		kgsl_pwrctrl_set_constraint(device, &context->pwr_constraint,
						context->id);
}

static inline int _get_alwayson_counter(struct adreno_device *adreno_dev,
		unsigned int *cmds, uint64_t gpuaddr)
{
	unsigned int *p = cmds;

	*p++ = cp_mem_packet(adreno_dev, CP_REG_TO_MEM, 2, 1);

	if (ADRENO_GPUREV(adreno_dev) >= 400 &&
		ADRENO_GPUREV(adreno_dev) <= ADRENO_REV_A530)
		*p++ = adreno_getreg(adreno_dev,
			ADRENO_REG_RBBM_ALWAYSON_COUNTER_LO);
	else
		*p++ = adreno_getreg(adreno_dev,
			ADRENO_REG_RBBM_ALWAYSON_COUNTER_LO) |
			(1 << 30) | (2 << 18);
	p += cp_gpuaddr(adreno_dev, p, gpuaddr);

	return (unsigned int)(p - cmds);
}

int adreno_ringbuffer_submitcmd(struct adreno_device *adreno_dev,
		struct kgsl_cmdbatch *cmdbatch, struct adreno_submit_time *time)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	struct kgsl_memobj_node *ib;
	unsigned int numibs = 0;
	unsigned int *link;
	unsigned int *cmds;
	struct kgsl_context *context;
	struct adreno_context *drawctxt;
	bool use_preamble = true;
	bool cmdbatch_user_profiling = false;
	bool cmdbatch_kernel_profiling = false;
	int flags = KGSL_CMD_FLAGS_NONE;
	int ret;
	struct adreno_ringbuffer *rb;
	struct kgsl_cmdbatch_profiling_buffer *profile_buffer = NULL;
	unsigned int dwords = 0;
	struct adreno_submit_time local;

	struct kgsl_mem_entry *entry = cmdbatch->profiling_buf_entry;
	if (entry)
		profile_buffer = kgsl_gpuaddr_to_vaddr(&entry->memdesc,
					cmdbatch->profiling_buffer_gpuaddr);

	context = cmdbatch->context;
	drawctxt = ADRENO_CONTEXT(context);

	
	list_for_each_entry(ib, &cmdbatch->cmdlist, node)
		numibs++;

	rb = drawctxt->rb;

	
	adreno_profile_process_results(adreno_dev);

	if (test_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->base.priv) &&
		(!test_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv))) {

		set_bit(KGSL_FT_SKIPCMD, &cmdbatch->fault_recovery);
		cmdbatch->fault_policy = drawctxt->fault_policy;
		set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &cmdbatch->priv);

		
		adreno_fault_skipcmd_detached(adreno_dev, drawctxt, cmdbatch);

		
		clear_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->base.priv);
		drawctxt->fault_policy = 0;
	}


	if ((drawctxt->base.flags & KGSL_CONTEXT_PREAMBLE) &&
		!test_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &cmdbatch->priv) &&
		(rb->drawctxt_active == drawctxt))
		use_preamble = false;

	if (test_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv)) {
		use_preamble = false;
		numibs = 0;
	}

	dwords = 7;

	
	dwords += (numibs * 30);

	if (cmdbatch->flags & KGSL_CMDBATCH_PROFILING &&
		!adreno_is_a3xx(adreno_dev) && profile_buffer) {
		cmdbatch_user_profiling = true;
		dwords += 6;

		if (adreno_is_a5xx(adreno_dev))
			dwords += 2;


		if (time == NULL)
			time = &local;
	}

	if (test_bit(CMDBATCH_FLAG_PROFILE, &cmdbatch->priv)) {
		cmdbatch_kernel_profiling = true;
		dwords += 6;
		if (adreno_is_a5xx(adreno_dev))
			dwords += 2;
	}

	if (gpudev->preemption_yield_enable &&
				adreno_is_preemption_enabled(adreno_dev))
		dwords += 8;

	link = kzalloc(sizeof(unsigned int) *  dwords, GFP_KERNEL);
	if (!link) {
		ret = -ENOMEM;
		goto done;
	}

	cmds = link;

	*cmds++ = cp_packet(adreno_dev, CP_NOP, 1);
	*cmds++ = KGSL_START_OF_IB_IDENTIFIER;

	if (cmdbatch_kernel_profiling) {
		cmds += _get_alwayson_counter(adreno_dev, cmds,
			adreno_dev->cmdbatch_profile_buffer.gpuaddr +
			ADRENO_CMDBATCH_PROFILE_OFFSET(cmdbatch->profile_index,
				started));
	}

	if (cmdbatch_user_profiling) {
		cmds += _get_alwayson_counter(adreno_dev, cmds,
			cmdbatch->profiling_buffer_gpuaddr +
			offsetof(struct kgsl_cmdbatch_profiling_buffer,
			gpu_ticks_submitted));
	}

	if (numibs) {
		list_for_each_entry(ib, &cmdbatch->cmdlist, node) {
			if (ib->priv & MEMOBJ_SKIP ||
				(ib->priv & MEMOBJ_PREAMBLE &&
				use_preamble == false))
				*cmds++ = cp_mem_packet(adreno_dev, CP_NOP,
						3, 1);

			*cmds++ = cp_mem_packet(adreno_dev,
					CP_INDIRECT_BUFFER_PFE, 2, 1);
			cmds += cp_gpuaddr(adreno_dev, cmds, ib->gpuaddr);
			*cmds++ = (unsigned int) ib->size >> 2;
			
			use_preamble = false;
		}
	}

	if (gpudev->preemption_yield_enable &&
				adreno_is_preemption_enabled(adreno_dev))
		cmds += gpudev->preemption_yield_enable(cmds);

	if (cmdbatch_kernel_profiling) {
		cmds += _get_alwayson_counter(adreno_dev, cmds,
			adreno_dev->cmdbatch_profile_buffer.gpuaddr +
			ADRENO_CMDBATCH_PROFILE_OFFSET(cmdbatch->profile_index,
				retired));
	}

	if (cmdbatch_user_profiling) {
		cmds += _get_alwayson_counter(adreno_dev, cmds,
			cmdbatch->profiling_buffer_gpuaddr +
			offsetof(struct kgsl_cmdbatch_profiling_buffer,
			gpu_ticks_retired));
	}

	*cmds++ = cp_packet(adreno_dev, CP_NOP, 1);
	*cmds++ = KGSL_END_OF_IB_IDENTIFIER;

	
	ret = adreno_drawctxt_switch(adreno_dev, rb, drawctxt,
		ADRENO_CONTEXT_SWITCH_FORCE_GPU);

	if (ret) {
		if (ret != -ENOSPC && ret != -ENOENT)
			KGSL_DRV_ERR(device,
				"Unable to switch draw context: %d\n", ret);
		goto done;
	}

	if (test_bit(CMDBATCH_FLAG_WFI, &cmdbatch->priv))
		flags = KGSL_CMD_FLAGS_WFI;


	if (test_and_clear_bit(ADRENO_DEVICE_PWRON, &adreno_dev->priv) &&
		test_bit(ADRENO_DEVICE_PWRON_FIXUP, &adreno_dev->priv))
		flags |= KGSL_CMD_FLAGS_PWRON_FIXUP;

	
	adreno_ringbuffer_set_constraint(device, cmdbatch);

	
	kgsl_cffdump_capture_ib_desc(device, context, cmdbatch);


	ret = adreno_ringbuffer_addcmds(rb, flags,
					&link[0], (cmds - link),
					cmdbatch->timestamp, time);

	if (!ret) {
		cmdbatch->global_ts = drawctxt->internal_timestamp;

		
		if (cmdbatch_user_profiling) {
			if (cmdbatch->flags & KGSL_CMDBATCH_PROFILING_KTIME) {
				uint64_t secs = time->ktime;

				profile_buffer->wall_clock_ns =
					do_div(secs, NSEC_PER_SEC);
				profile_buffer->wall_clock_s = secs;
			} else {
				profile_buffer->wall_clock_s =
					time->utime.tv_sec;
				profile_buffer->wall_clock_ns =
					time->utime.tv_nsec;
			}
			profile_buffer->gpu_ticks_queued = time->ticks;
		}
	}

	kgsl_cffdump_regpoll(device,
		adreno_getreg(adreno_dev, ADRENO_REG_RBBM_STATUS) << 2,
		0x00000000, 0x80000000);
done:
	
	if (entry)
		kgsl_memdesc_unmap(&entry->memdesc);


	trace_kgsl_issueibcmds(device, context->id, cmdbatch,
			numibs, cmdbatch->timestamp,
			cmdbatch->flags, ret, drawctxt->type);

	kfree(link);
	return ret;
}

static void adreno_ringbuffer_wait_callback(struct kgsl_device *device,
		struct kgsl_event_group *group,
		void *priv, int result)
{
	struct adreno_ringbuffer *rb = group->priv;
	wake_up_all(&rb->ts_expire_waitq);
}

static inline int adreno_ringbuffer_check_timestamp(
			struct adreno_ringbuffer *rb,
			unsigned int timestamp, int type)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	unsigned int ts;

	adreno_rb_readtimestamp(adreno_dev, rb, type, &ts);
	return (timestamp_cmp(ts, timestamp) >= 0);
}


int adreno_ringbuffer_waittimestamp(struct adreno_ringbuffer *rb,
					unsigned int timestamp,
					unsigned int msecs)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret;
	unsigned long wait_time;

	
	BUG_ON(0 == msecs);

	ret = kgsl_add_event(device, &rb->events, timestamp,
		adreno_ringbuffer_wait_callback, NULL);
	if (ret)
		return ret;

	mutex_unlock(&device->mutex);

	wait_time = msecs_to_jiffies(msecs);
	if (0 == wait_event_timeout(rb->ts_expire_waitq,
		!kgsl_event_pending(device, &rb->events, timestamp,
				adreno_ringbuffer_wait_callback, NULL),
		wait_time))
		ret  = -ETIMEDOUT;

	mutex_lock(&device->mutex);
	if (!ret && !adreno_ringbuffer_check_timestamp(rb,
		timestamp, KGSL_TIMESTAMP_RETIRED)) {
		ret = -EAGAIN;
	}

	return ret;
}
