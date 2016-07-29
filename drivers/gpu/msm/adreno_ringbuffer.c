/* Copyright (c) 2002,2007-2015, The Linux Foundation. All rights reserved.
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
#include "adreno_pm4types.h"
#include "adreno_ringbuffer.h"

#include "a3xx_reg.h"
#include "adreno_a5xx.h"

#define GSL_RB_NOP_SIZEDWORDS				2

#define ADRENO_RB_PREEMPT_TOKEN_IB_DWORDS	50
#define ADRENO_RB_PREEMPT_TOKEN_DWORDS		125

#define RB_HOSTPTR(_rb, _pos) \
	((unsigned int *) ((_rb)->buffer_desc.hostptr + \
		((_pos) * sizeof(unsigned int))))

#define RB_GPUADDR(_rb, _pos) \
	((_rb)->buffer_desc.gpuaddr + ((_pos) * sizeof(unsigned int)))

static void _cff_write_ringbuffer(struct adreno_ringbuffer *rb)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);
	struct kgsl_device *device = &adreno_dev->dev;
	uint64_t gpuaddr;
	unsigned int *hostptr;
	size_t size;

	if (device->cff_dump_enable == 0)
		return;

	BUG_ON(rb->wptr < rb->last_wptr);

	size = (rb->wptr - rb->last_wptr) * sizeof(unsigned int);

	hostptr = RB_HOSTPTR(rb, rb->last_wptr);
	gpuaddr = RB_GPUADDR(rb, rb->last_wptr);

	kgsl_cffdump_memcpy(device, gpuaddr, hostptr, size);
}

void adreno_ringbuffer_submit(struct adreno_ringbuffer *rb,
		struct adreno_submit_time *time)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);
	BUG_ON(rb->wptr == 0);

	
	_cff_write_ringbuffer(rb);


	if (time != NULL) {
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
		}
		else
			time->ticks = 0;

		
		time->ktime = local_clock();

		
		getnstimeofday(&time->utime);

		local_irq_restore(flags);
	}

	
	mb();

	if (adreno_preempt_state(adreno_dev, ADRENO_DISPATCHER_PREEMPT_CLEAR) &&
		(adreno_dev->cur_rb == rb)) {
		kgsl_pwrscale_busy(rb->device);
		adreno_writereg(adreno_dev, ADRENO_REG_CP_RB_WPTR, rb->wptr);
	}
}

int adreno_ringbuffer_submit_spin(struct adreno_ringbuffer *rb,
		struct adreno_submit_time *time, unsigned int timeout)
{
	adreno_ringbuffer_submit(rb, NULL);
	return adreno_spin_idle(rb->device, timeout);
}

int adreno_ringbuffer_submit_spin_retry(struct adreno_ringbuffer *rb,
		struct adreno_submit_time *time, unsigned int timeout,
		unsigned int retry)
{
	int ret = 0;
	int num_trial = 0;

	adreno_ringbuffer_submit(rb, NULL);
	do {
		ret = adreno_spin_idle(rb->device, timeout);
		if (ret == -ETIMEDOUT) {
			KGSL_DRV_ERR(rb->device,
					"hw initialization failed to idle after %d msec\n",
					(++num_trial)*timeout);
		}
		else
			break;
	} while (num_trial < retry);

	return ret;
}

static int
adreno_ringbuffer_waitspace(struct adreno_ringbuffer *rb,
				unsigned int numcmds, int wptr_ahead)
{
	int nopcount = 0;
	unsigned int freecmds;
	unsigned int wptr = rb->wptr;
	unsigned int *cmds = NULL;
	uint64_t gpuaddr;
	unsigned long wait_time;
	unsigned long wait_timeout = msecs_to_jiffies(ADRENO_IDLE_TIMEOUT);
	unsigned int rptr;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);

	
	if (wptr_ahead) {
		
		nopcount = KGSL_RB_DWORDS - rb->wptr - 1;

		cmds = RB_HOSTPTR(rb, rb->wptr);
		gpuaddr = RB_GPUADDR(rb, rb->wptr);

		rptr = adreno_get_rptr(rb);
		
		if ((adreno_dev->cur_rb != rb ||
				!adreno_preempt_state(adreno_dev,
				ADRENO_DISPATCHER_PREEMPT_CLEAR)) &&
			!rptr)
			return -ENOSPC;

		wait_time = jiffies + wait_timeout;
		while (!rptr) {
			rptr = adreno_get_rptr(rb);
			if (time_after(jiffies, wait_time))
				return -ETIMEDOUT;
		}

		rb->wptr = 0;
	}

	rptr = adreno_get_rptr(rb);
	freecmds = rptr - rb->wptr;
	if (freecmds == 0 || freecmds > numcmds)
		goto done;

	
	if (adreno_dev->cur_rb != rb ||
		!adreno_preempt_state(adreno_dev,
			ADRENO_DISPATCHER_PREEMPT_CLEAR)) {
		rb->wptr = wptr;
		return -ENOSPC;
	}

	wait_time = jiffies + wait_timeout;
	
	while (1) {
		rptr = adreno_get_rptr(rb);

		freecmds = rptr - rb->wptr;

		if (freecmds == 0 || freecmds > numcmds)
			break;

		if (time_after(jiffies, wait_time)) {
			KGSL_DRV_ERR(rb->device,
			"Timed out waiting for freespace in RB rptr: 0x%x, wptr: 0x%x, rb id %d\n",
			rptr, wptr, rb->id);
			return -ETIMEDOUT;
		}
	}
done:
	if (wptr_ahead) {
		*cmds = cp_packet(adreno_dev, CP_NOP, nopcount);
		kgsl_cffdump_write(rb->device, gpuaddr, *cmds);

	}
	return 0;
}

unsigned int *adreno_ringbuffer_allocspace(struct adreno_ringbuffer *rb,
					unsigned int numcmds)
{
	unsigned int *ptr = NULL;
	int ret = 0;
	unsigned int rptr;
	BUG_ON(numcmds >= KGSL_RB_DWORDS);

	rptr = adreno_get_rptr(rb);
	
	if (rb->wptr >= rptr) {
		
		
		if ((rb->wptr + numcmds) > (KGSL_RB_DWORDS -
				GSL_RB_NOP_SIZEDWORDS))
			ret = adreno_ringbuffer_waitspace(rb, numcmds, 1);
	} else {
		
		if ((rb->wptr + numcmds) >= rptr)
			ret = adreno_ringbuffer_waitspace(rb, numcmds, 0);
		
		
		if (!ret && (rb->wptr + numcmds) > (KGSL_RB_DWORDS -
				GSL_RB_NOP_SIZEDWORDS))
			ret = adreno_ringbuffer_waitspace(rb, numcmds, 1);
	}

	if (!ret) {
		rb->last_wptr = rb->wptr;

		ptr = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
		rb->wptr += numcmds;
	} else
		ptr = ERR_PTR(ret);

	return ptr;
}

static void _ringbuffer_setup_common(struct adreno_ringbuffer *rb)
{
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *rb_temp;
	int i;

	FOR_EACH_RINGBUFFER(adreno_dev, rb_temp, i) {
		kgsl_sharedmem_set(rb_temp->device,
			&(rb_temp->buffer_desc), 0,
			0xAA, KGSL_RB_SIZE);
		rb_temp->wptr = 0;
		rb_temp->rptr = 0;
		rb_temp->wptr_preempt_end = 0xFFFFFFFF;
		rb_temp->starve_timer_state =
		ADRENO_DISPATCHER_RB_STARVE_TIMER_UNINIT;
		adreno_iommu_set_pt_generate_rb_cmds(rb_temp,
					device->mmu.defaultpagetable);
	}


	adreno_writereg(adreno_dev, ADRENO_REG_CP_RB_CNTL,
		(ilog2(KGSL_RB_DWORDS >> 1) & 0x3F) |
		(1 << 27));

	adreno_writereg64(adreno_dev, ADRENO_REG_CP_RB_BASE,
			  ADRENO_REG_CP_RB_BASE_HI, rb->buffer_desc.gpuaddr);

	
	if (adreno_is_a3xx(adreno_dev)) {
		unsigned int val = 0x000E0602;

		if (adreno_is_a305b(adreno_dev) ||
				adreno_is_a310(adreno_dev) ||
				adreno_is_a330(adreno_dev))
			val = 0x003E2008;
		kgsl_regwrite(device, A3XX_CP_QUEUE_THRESHOLDS, val);
	}
}

static int _ringbuffer_start_common(struct adreno_ringbuffer *rb)
{
	int status;
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);

	
	adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_CNTL, 0);

	
	status = gpudev->rb_init(adreno_dev, rb);
	if (status)
		return status;

	return status;
}

int adreno_ringbuffer_start(struct adreno_device *adreno_dev,
	unsigned int start_type)
{
	int status;
	struct adreno_ringbuffer *rb = ADRENO_CURRENT_RINGBUFFER(adreno_dev);
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);

	_ringbuffer_setup_common(rb);

	status = gpudev->microcode_load(adreno_dev, start_type);
	if (status)
		return status;

	return _ringbuffer_start_common(rb);
}

void adreno_ringbuffer_stop(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_ringbuffer *rb;
	int i;
	FOR_EACH_RINGBUFFER(adreno_dev, rb, i)
		kgsl_cancel_events(device, &(rb->events));
}

static int _adreno_ringbuffer_init(struct adreno_device *adreno_dev,
				struct adreno_ringbuffer *rb, int id)
{
	int ret;
	char name[64];

	rb->device = &adreno_dev->dev;
	rb->id = id;

	snprintf(name, sizeof(name), "rb_events-%d", id);
	kgsl_add_event_group(&rb->events, NULL, name,
		adreno_rb_readtimestamp, rb);
	rb->timestamp = 0;
	init_waitqueue_head(&rb->ts_expire_waitq);

	ret = kgsl_allocate_global(&adreno_dev->dev, &rb->pagetable_desc,
		PAGE_SIZE, 0, KGSL_MEMDESC_PRIVILEGED);
	if (ret)
		return ret;

	ret = kgsl_allocate_global(&adreno_dev->dev, &rb->buffer_desc,
			KGSL_RB_SIZE, KGSL_MEMFLAGS_GPUREADONLY, 0);
	return ret;
}

int adreno_ringbuffer_init(struct adreno_device *adreno_dev, bool nopreempt)
{
	int status = 0;
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	struct adreno_ringbuffer *rb;
	int i;

	if (nopreempt == false && ADRENO_FEATURE(adreno_dev, ADRENO_PREEMPTION))
		adreno_dev->num_ringbuffers = gpudev->num_prio_levels;
	else
		adreno_dev->num_ringbuffers = 1;

	FOR_EACH_RINGBUFFER(adreno_dev, rb, i) {
		status = _adreno_ringbuffer_init(adreno_dev, rb, i);
		if (status)
			break;
	}
	if (status)
		adreno_ringbuffer_close(adreno_dev);
	else
		adreno_dev->cur_rb = &(adreno_dev->ringbuffers[0]);

	return status;
}

static void _adreno_ringbuffer_close(struct adreno_ringbuffer *rb)
{
	kgsl_free_global(&rb->pagetable_desc);
	kgsl_free_global(&rb->preemption_desc);

	memset(&rb->pt_update_desc, 0, sizeof(struct kgsl_memdesc));

	kgsl_free_global(&rb->buffer_desc);
	kgsl_del_event_group(&rb->events);
	memset(rb, 0, sizeof(struct adreno_ringbuffer));
}

void adreno_ringbuffer_close(struct adreno_device *adreno_dev)
{
	struct adreno_ringbuffer *rb;
	int i;

	FOR_EACH_RINGBUFFER(adreno_dev, rb, i)
		_adreno_ringbuffer_close(rb);
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

static int
adreno_ringbuffer_addcmds(struct adreno_ringbuffer *rb,
				unsigned int flags, unsigned int *cmds,
				unsigned int sizedwords, uint32_t timestamp,
				struct adreno_submit_time *time)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	struct kgsl_device *device = rb->device;
	unsigned int *ringcmds, *start;
	unsigned int total_sizedwords = sizedwords;
	unsigned int i;
	unsigned int context_id = 0;
	uint64_t gpuaddr = rb->device->memstore.gpuaddr;
	bool profile_ready;
	struct adreno_context *drawctxt = rb->drawctxt_active;
	struct kgsl_context *context = NULL;
	bool secured_ctxt = false;
	uint64_t cond_addr;

	if (drawctxt != NULL && kgsl_context_detached(&drawctxt->base) &&
		!(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE))
		return -ENOENT;

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

	
	if (adreno_is_a3xx(adreno_dev))
		total_sizedwords += 3;

	
	if (adreno_is_a4xx(adreno_dev) || adreno_is_a3xx(adreno_dev))
		total_sizedwords += 4;

	if (gpudev->preemption_pre_ibsubmit &&
				adreno_is_preemption_enabled(adreno_dev))
		total_sizedwords += 20;

	if (gpudev->preemption_post_ibsubmit &&
				adreno_is_preemption_enabled(adreno_dev))
		total_sizedwords += 13;

	total_sizedwords += 4; 
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
				gpudev->preemption_pre_ibsubmit) {
		cond_addr = device->memstore.gpuaddr +
					KGSL_MEMSTORE_OFFSET(context_id,
					 preempted);
		ringcmds += gpudev->preemption_pre_ibsubmit(
					adreno_dev, rb, ringcmds, context,
					cond_addr, NULL);
	}

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

	
	*ringcmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	if (drawctxt && !(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE))
		ringcmds += cp_gpuaddr(adreno_dev, ringcmds,
			gpuaddr + KGSL_MEMSTORE_OFFSET(context_id,
			soptimestamp));
	else
		ringcmds += cp_gpuaddr(adreno_dev, ringcmds,
			gpuaddr + KGSL_MEMSTORE_RB_OFFSET(rb, soptimestamp));
	*ringcmds++ = timestamp;

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

	*ringcmds++ = cp_mem_packet(adreno_dev, CP_EVENT_WRITE, 3, 1);
	if (drawctxt || (flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE))
		*ringcmds++ = CACHE_FLUSH_TS | (1 << 31);
	else
		*ringcmds++ = CACHE_FLUSH_TS;

	if (drawctxt && !(flags & KGSL_CMD_FLAGS_INTERNAL_ISSUE)) {
		ringcmds += cp_gpuaddr(adreno_dev, ringcmds, gpuaddr +
				KGSL_MEMSTORE_OFFSET(context_id, eoptimestamp));
		*ringcmds++ = timestamp;
		*ringcmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
		ringcmds += cp_gpuaddr(adreno_dev, ringcmds, gpuaddr +
				KGSL_MEMSTORE_RB_OFFSET(rb, eoptimestamp));
		*ringcmds++ = rb->timestamp;
	} else {
		ringcmds += cp_gpuaddr(adreno_dev, ringcmds, gpuaddr +
				KGSL_MEMSTORE_RB_OFFSET(rb, eoptimestamp));
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
					rb, ringcmds, &drawctxt->base);

	if ((ringcmds - start) > total_sizedwords)
		BUG();
	rb->wptr = rb->wptr - (total_sizedwords - (ringcmds - start));

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
	struct kgsl_device *device = &adreno_dev->dev;
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

		
		adreno_fault_skipcmd_detached(device, drawctxt, cmdbatch);

		
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

	ret = adreno_drawctxt_switch(adreno_dev, rb, drawctxt, cmdbatch->flags);

	if (ret)
		goto done;

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
			profile_buffer->wall_clock_s = time->utime.tv_sec;
			profile_buffer->wall_clock_ns = time->utime.tv_nsec;
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

static void adreno_ringbuffer_mmu_clk_disable_event(struct kgsl_device *device,
			struct kgsl_event_group *group, void *data, int type)
{
	kgsl_mmu_disable_clk(&device->mmu);
}

void
adreno_ringbuffer_mmu_disable_clk_on_ts(struct kgsl_device *device,
			struct adreno_ringbuffer *rb, unsigned int timestamp)
{
	if (kgsl_add_event(device, &(rb->events), timestamp,
		adreno_ringbuffer_mmu_clk_disable_event, NULL)) {
		KGSL_DRV_ERR(device,
			"Failed to add IOMMU disable clk event\n");
	}
}

static void adreno_ringbuffer_wait_callback(struct kgsl_device *device,
		struct kgsl_event_group *group,
		void *priv, int result)
{
	struct adreno_ringbuffer *rb = group->priv;
	wake_up_all(&rb->ts_expire_waitq);
}

int adreno_ringbuffer_waittimestamp(struct adreno_ringbuffer *rb,
					unsigned int timestamp,
					unsigned int msecs)
{
	struct kgsl_device *device = rb->device;
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

int adreno_ringbuffer_submit_preempt_token(struct adreno_ringbuffer *rb,
					struct adreno_ringbuffer *incoming_rb)
{
	unsigned int *ringcmds, *start;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);
	struct kgsl_device *device = &(adreno_dev->dev);
	struct kgsl_iommu *iommu = device->mmu.priv;
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	int ptname;
	struct kgsl_pagetable *pt;
	int pt_switch_sizedwords = 0, total_sizedwords = 20;
	unsigned link[ADRENO_RB_PREEMPT_TOKEN_DWORDS];
	uint i;
	uint64_t ttbr0;

	if (incoming_rb->preempted_midway) {

		if (adreno_is_a5xx(adreno_dev)) {
			kgsl_sharedmem_readq(&rb->pagetable_desc, &ttbr0,
				offsetof(struct adreno_ringbuffer_pagetable_info
				, ttbr0));
			kgsl_sharedmem_writeq(rb->device, &iommu->smmu_info,
				offsetof(struct a5xx_cp_smmu_info, ttbr0),
				ttbr0);
		} else {
			kgsl_sharedmem_readl(&incoming_rb->pagetable_desc,
				&ptname, offsetof(
				struct adreno_ringbuffer_pagetable_info,
				current_rb_ptname));
			pt = kgsl_mmu_get_pt_from_ptname(&(rb->device->mmu),
				ptname);
			BUG_ON(!pt);
			
			pt_switch_sizedwords =
				adreno_iommu_set_pt_generate_cmds(incoming_rb,
								&link[0], pt);
			total_sizedwords += pt_switch_sizedwords;

		}
	}

	ringcmds = adreno_ringbuffer_allocspace(rb, total_sizedwords);

	if (IS_ERR(ringcmds))
		return PTR_ERR(ringcmds);

	start = ringcmds;

	*ringcmds++ = cp_packet(adreno_dev, CP_SET_PROTECTED_MODE, 1);
	*ringcmds++ = 0;

	if (incoming_rb->preempted_midway) {
		for (i = 0; i < pt_switch_sizedwords; i++)
			*ringcmds++ = link[i];
	}

	*ringcmds++ = cp_register(adreno_dev, adreno_getreg(adreno_dev,
			ADRENO_REG_CP_PREEMPT_DISABLE), 1);
	*ringcmds++ = 0;

	*ringcmds++ = cp_packet(adreno_dev, CP_SET_PROTECTED_MODE, 1);
	*ringcmds++ = 1;

	ringcmds += gpudev->preemption_token(adreno_dev, rb, ringcmds,
				rb->device->memstore.gpuaddr +
				KGSL_MEMSTORE_RB_OFFSET(rb, preempted));

	if ((uint)(ringcmds - start) > total_sizedwords) {
		KGSL_DRV_ERR(device, "Insufficient rb size allocated\n");
		BUG();
	}

	rb->wptr = rb->wptr - (total_sizedwords - (uint)(ringcmds - start));

	
	mb();
	kgsl_pwrscale_busy(rb->device);
	adreno_writereg(adreno_dev, ADRENO_REG_CP_RB_WPTR, rb->wptr);
	return 0;
}
