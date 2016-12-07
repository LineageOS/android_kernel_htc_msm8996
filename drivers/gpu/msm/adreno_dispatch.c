/* Copyright (c) 2013-2016, The Linux Foundation. All rights reserved.
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

#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/err.h>

#include "kgsl.h"
#include "kgsl_cffdump.h"
#include "kgsl_sharedmem.h"
#include "adreno.h"
#include "adreno_ringbuffer.h"
#include "adreno_trace.h"
#include "kgsl_sharedmem.h"
#include "kgsl_htc.h"

#define CMDQUEUE_NEXT(_i, _s) (((_i) + 1) % (_s))

unsigned int adreno_dispatch_starvation_time = 2000;

unsigned int adreno_dispatch_time_slice = 25;

unsigned int adreno_disp_preempt_fair_sched;

static unsigned int _context_cmdqueue_size = 50;

static unsigned int _context_queue_wait = 10000;

static unsigned int _context_cmdbatch_burst = 5;

static unsigned int _fault_throttle_time = 3000;
static unsigned int _fault_throttle_burst = 3;

static unsigned int _dispatcher_q_inflight_hi = 15;

static unsigned int _dispatcher_q_inflight_lo = 4;

unsigned int adreno_cmdbatch_timeout = 2000;

static unsigned int _fault_timer_interval = 200;

#define CMDQUEUE_RB(_cmdqueue) \
	((struct adreno_ringbuffer *) \
		container_of((_cmdqueue), struct adreno_ringbuffer, dispatch_q))

#define CMDQUEUE(_ringbuffer) (&(_ringbuffer)->dispatch_q)

static int adreno_dispatch_retire_cmdqueue(struct adreno_device *adreno_dev,
		struct adreno_dispatcher_cmdqueue *cmdqueue);

static inline bool cmdqueue_is_current(
		struct adreno_dispatcher_cmdqueue *cmdqueue)
{
	struct adreno_ringbuffer *rb = CMDQUEUE_RB(cmdqueue);
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);

	return (adreno_dev->cur_rb == rb);
}

static void _add_context(struct adreno_device *adreno_dev,
		struct adreno_context *drawctxt)
{
	
	list_del_init(&drawctxt->active_node);

	
	drawctxt->active_time = jiffies;
	list_add(&drawctxt->active_node, &adreno_dev->active_list);
}

static int __count_context(struct adreno_context *drawctxt, void *data)
{
	unsigned long expires = drawctxt->active_time + msecs_to_jiffies(100);

	return time_after(jiffies, expires) ? 0 : 1;
}

static int __count_cmdqueue_context(struct adreno_context *drawctxt, void *data)
{
	unsigned long expires = drawctxt->active_time + msecs_to_jiffies(100);

	if (time_after(jiffies, expires))
		return 0;

	return (&drawctxt->rb->dispatch_q ==
			(struct adreno_dispatcher_cmdqueue *) data) ? 1 : 0;
}

static int _adreno_count_active_contexts(struct adreno_device *adreno_dev,
		int (*func)(struct adreno_context *, void *), void *data)
{
	struct adreno_context *ctxt;
	int count = 0;

	list_for_each_entry(ctxt, &adreno_dev->active_list, active_node) {
		if (func(ctxt, data) == 0)
			return count;

		count++;
	}

	return count;
}

static void _track_context(struct adreno_device *adreno_dev,
		struct adreno_dispatcher_cmdqueue *cmdqueue,
		struct adreno_context *drawctxt)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	spin_lock(&adreno_dev->active_list_lock);

	_add_context(adreno_dev, drawctxt);

	device->active_context_count =
			_adreno_count_active_contexts(adreno_dev,
					__count_context, NULL);
	cmdqueue->active_context_count =
			_adreno_count_active_contexts(adreno_dev,
					__count_cmdqueue_context, cmdqueue);

	spin_unlock(&adreno_dev->active_list_lock);
}


static inline int
_cmdqueue_inflight(struct adreno_dispatcher_cmdqueue *cmdqueue)
{
	return (cmdqueue->active_context_count > 1)
		? _dispatcher_q_inflight_lo : _dispatcher_q_inflight_hi;
}

static void fault_detect_read(struct adreno_device *adreno_dev)
{
	int i;

	if (!test_bit(ADRENO_DEVICE_SOFT_FAULT_DETECT, &adreno_dev->priv))
		return;

	for (i = 0; i < adreno_dev->num_ringbuffers; i++) {
		struct adreno_ringbuffer *rb = &(adreno_dev->ringbuffers[i]);
		adreno_rb_readtimestamp(adreno_dev, rb,
			KGSL_TIMESTAMP_RETIRED, &(rb->fault_detect_ts));
	}

	for (i = 0; i < adreno_ft_regs_num; i++) {
		if (adreno_ft_regs[i] != 0)
			kgsl_regread(KGSL_DEVICE(adreno_dev), adreno_ft_regs[i],
				&adreno_ft_regs_val[i]);
	}
}

static inline bool _isidle(struct adreno_device *adreno_dev)
{
	const struct adreno_gpu_core *gpucore = adreno_dev->gpucore;
	unsigned int reg_rbbm_status;

	if (!kgsl_state_is_awake(KGSL_DEVICE(adreno_dev)))
		goto ret;

	
	adreno_readreg(adreno_dev, ADRENO_REG_RBBM_STATUS, &reg_rbbm_status);

	if (reg_rbbm_status & gpucore->busy_mask)
		return false;

ret:
	
	memset(adreno_ft_regs_val, 0,
		adreno_ft_regs_num * sizeof(unsigned int));

	return true;
}

static int fault_detect_read_compare(struct adreno_device *adreno_dev)
{
	struct adreno_ringbuffer *rb = ADRENO_CURRENT_RINGBUFFER(adreno_dev);
	int i, ret = 0;
	unsigned int ts;

	
	if (_isidle(adreno_dev) == true)
		ret = 1;

	for (i = 0; i < adreno_ft_regs_num; i++) {
		unsigned int val;

		if (adreno_ft_regs[i] == 0)
			continue;
		kgsl_regread(KGSL_DEVICE(adreno_dev), adreno_ft_regs[i], &val);
		if (val != adreno_ft_regs_val[i])
			ret = 1;
		adreno_ft_regs_val[i] = val;
	}

	if (!adreno_rb_readtimestamp(adreno_dev, adreno_dev->cur_rb,
				KGSL_TIMESTAMP_RETIRED, &ts)) {
		if (ts != rb->fault_detect_ts)
			ret = 1;

		rb->fault_detect_ts = ts;
	}

	return ret;
}

static void start_fault_timer(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	if (adreno_soft_fault_detect(adreno_dev))
		mod_timer(&dispatcher->fault_timer,
			jiffies + msecs_to_jiffies(_fault_timer_interval));
}

static void _retire_marker(struct kgsl_cmdbatch *cmdbatch)
{
	struct kgsl_context *context = cmdbatch->context;
	struct adreno_context *drawctxt = ADRENO_CONTEXT(cmdbatch->context);
	struct kgsl_device *device = context->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	kgsl_sharedmem_writel(device, &device->memstore,
		KGSL_MEMSTORE_OFFSET(context->id, soptimestamp),
		cmdbatch->timestamp);

	kgsl_sharedmem_writel(device, &device->memstore,
		KGSL_MEMSTORE_OFFSET(context->id, eoptimestamp),
		cmdbatch->timestamp);


	
	kgsl_process_event_group(device, &context->events);

	if (adreno_is_a3xx(adreno_dev))
		trace_adreno_cmdbatch_retired(cmdbatch, -1, 0, 0, drawctxt->rb,
				0);
	else
		trace_adreno_cmdbatch_retired(cmdbatch, -1, 0, 0, drawctxt->rb,
			adreno_get_rptr(drawctxt->rb));
	kgsl_cmdbatch_destroy(cmdbatch);
}

static int _check_context_queue(struct adreno_context *drawctxt)
{
	int ret;

	spin_lock(&drawctxt->lock);


	if (kgsl_context_invalid(&drawctxt->base))
		ret = 1;
	else
		ret = drawctxt->queued < _context_cmdqueue_size ? 1 : 0;

	spin_unlock(&drawctxt->lock);

	return ret;
}

static bool _marker_expired(struct kgsl_cmdbatch *cmdbatch)
{
	return (cmdbatch->flags & KGSL_CMDBATCH_MARKER) &&
		kgsl_check_timestamp(cmdbatch->device, cmdbatch->context,
			cmdbatch->marker_timestamp);
}

static inline void _pop_cmdbatch(struct adreno_context *drawctxt)
{
	drawctxt->cmdqueue_head = CMDQUEUE_NEXT(drawctxt->cmdqueue_head,
		ADRENO_CONTEXT_CMDQUEUE_SIZE);
	drawctxt->queued--;
}
static struct kgsl_cmdbatch *_expire_markers(struct adreno_context *drawctxt)
{
	struct kgsl_cmdbatch *cmdbatch;

	if (drawctxt->cmdqueue_head == drawctxt->cmdqueue_tail)
		return NULL;

	cmdbatch = drawctxt->cmdqueue[drawctxt->cmdqueue_head];

	if (cmdbatch == NULL)
		return NULL;

	
	if ((cmdbatch->flags & KGSL_CMDBATCH_MARKER) &&
			_marker_expired(cmdbatch)) {
		_pop_cmdbatch(drawctxt);
		_retire_marker(cmdbatch);
		return _expire_markers(drawctxt);
	}

	if (cmdbatch->flags & KGSL_CMDBATCH_SYNC) {
		if (!kgsl_cmdbatch_events_pending(cmdbatch)) {
			_pop_cmdbatch(drawctxt);
			kgsl_cmdbatch_destroy(cmdbatch);
			return _expire_markers(drawctxt);
		}
	}

	return cmdbatch;
}

static void expire_markers(struct adreno_context *drawctxt)
{
	spin_lock(&drawctxt->lock);
	_expire_markers(drawctxt);
	spin_unlock(&drawctxt->lock);
}

static struct kgsl_cmdbatch *_get_cmdbatch(struct adreno_context *drawctxt)
{
	struct kgsl_cmdbatch *cmdbatch;
	bool pending = false;

	cmdbatch = _expire_markers(drawctxt);

	if (cmdbatch == NULL)
		return NULL;

	if ((cmdbatch->flags & KGSL_CMDBATCH_MARKER) &&
			(!test_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv)))
		pending = true;

	if (kgsl_cmdbatch_events_pending(cmdbatch))
		pending = true;

	if (pending) {
		if (!cmdbatch->timeout_jiffies) {
			cmdbatch->timeout_jiffies =
				jiffies + msecs_to_jiffies(5000);
			mod_timer(&cmdbatch->timer, cmdbatch->timeout_jiffies);
		}

		return ERR_PTR(-EAGAIN);
	}

	_pop_cmdbatch(drawctxt);
	return cmdbatch;
}

static struct kgsl_cmdbatch *adreno_dispatcher_get_cmdbatch(
		struct adreno_context *drawctxt)
{
	struct kgsl_cmdbatch *cmdbatch;

	spin_lock(&drawctxt->lock);
	cmdbatch = _get_cmdbatch(drawctxt);
	spin_unlock(&drawctxt->lock);

	if (!IS_ERR_OR_NULL(cmdbatch))
		del_timer_sync(&cmdbatch->timer);

	return cmdbatch;
}

static inline int adreno_dispatcher_requeue_cmdbatch(
		struct adreno_context *drawctxt, struct kgsl_cmdbatch *cmdbatch)
{
	unsigned int prev;
	spin_lock(&drawctxt->lock);

	if (kgsl_context_detached(&drawctxt->base) ||
		kgsl_context_invalid(&drawctxt->base)) {
		spin_unlock(&drawctxt->lock);
		
		kgsl_cmdbatch_destroy(cmdbatch);
		return -ENOENT;
	}

	prev = drawctxt->cmdqueue_head == 0 ?
		(ADRENO_CONTEXT_CMDQUEUE_SIZE - 1) :
		(drawctxt->cmdqueue_head - 1);


	BUG_ON(prev == drawctxt->cmdqueue_tail);

	drawctxt->cmdqueue[prev] = cmdbatch;
	drawctxt->queued++;

	
	drawctxt->cmdqueue_head = prev;
	spin_unlock(&drawctxt->lock);
	return 0;
}

static void  dispatcher_queue_context(struct adreno_device *adreno_dev,
		struct adreno_context *drawctxt)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	
	if (kgsl_context_detached(&drawctxt->base))
		return;

	spin_lock(&dispatcher->plist_lock);

	if (plist_node_empty(&drawctxt->pending)) {
		
		if (_kgsl_context_get(&drawctxt->base)) {
			trace_dispatch_queue_context(drawctxt);
			plist_add(&drawctxt->pending, &dispatcher->pending);
		}
	}

	spin_unlock(&dispatcher->plist_lock);
}

static int sendcmd(struct adreno_device *adreno_dev,
	struct kgsl_cmdbatch *cmdbatch)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	struct adreno_context *drawctxt = ADRENO_CONTEXT(cmdbatch->context);
	struct adreno_dispatcher_cmdqueue *dispatch_q =
				ADRENO_CMDBATCH_DISPATCH_CMDQUEUE(cmdbatch);
	struct adreno_submit_time time;
	uint64_t secs = 0;
	unsigned long nsecs = 0;
	int ret;

	mutex_lock(&device->mutex);
	if (adreno_gpu_halt(adreno_dev) != 0) {
		mutex_unlock(&device->mutex);
		return -EBUSY;
	}

	dispatcher->inflight++;
	dispatch_q->inflight++;

	if (dispatcher->inflight == 1 &&
			!test_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv)) {
		
		ret = kgsl_active_count_get(device);
		if (ret) {
			dispatcher->inflight--;
			dispatch_q->inflight--;
			mutex_unlock(&device->mutex);
			return ret;
		}

		set_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv);
	}

	if (test_bit(ADRENO_DEVICE_CMDBATCH_PROFILE, &adreno_dev->priv)) {
		set_bit(CMDBATCH_FLAG_PROFILE, &cmdbatch->priv);
		cmdbatch->profile_index = adreno_dev->cmdbatch_profile_index;
		adreno_dev->cmdbatch_profile_index =
			(adreno_dev->cmdbatch_profile_index + 1) %
			ADRENO_CMDBATCH_PROFILE_COUNT;
	}

	ret = adreno_ringbuffer_submitcmd(adreno_dev, cmdbatch, &time);


	if (dispatcher->inflight == 1) {
		if (ret == 0) {

			
			del_timer_sync(&dispatcher->fault_timer);

			fault_detect_read(adreno_dev);

			
			start_fault_timer(adreno_dev);

			if (!test_and_set_bit(ADRENO_DISPATCHER_ACTIVE,
				&dispatcher->priv))
				reinit_completion(&dispatcher->idle_gate);
		} else {
			kgsl_active_count_put(device);
			clear_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv);
		}
	}


	if (ret) {
		dispatcher->inflight--;
		dispatch_q->inflight--;

		mutex_unlock(&device->mutex);


		if (ret != -ENOENT && ret != -ENOSPC && ret != -EPROTO)
			KGSL_DRV_ERR(device,
				"Unable to submit command to the ringbuffer %d\n",
				ret);
		return ret;
	}

	secs = time.ktime;
	nsecs = do_div(secs, 1000000000);

	trace_adreno_cmdbatch_submitted(cmdbatch, (int) dispatcher->inflight,
		time.ticks, (unsigned long) secs, nsecs / 1000, drawctxt->rb,
		adreno_get_rptr(drawctxt->rb));

	mutex_unlock(&device->mutex);

	cmdbatch->submit_ticks = time.ticks;

	dispatch_q->cmd_q[dispatch_q->tail] = cmdbatch;
	dispatch_q->tail = (dispatch_q->tail + 1) %
		ADRENO_DISPATCH_CMDQUEUE_SIZE;


	if (dispatch_q->inflight == 1)
		dispatch_q->expires = jiffies +
			msecs_to_jiffies(adreno_cmdbatch_timeout);

	if (!adreno_in_preempt_state(adreno_dev, ADRENO_PREEMPT_NONE)) {
		if (cmdqueue_is_current(dispatch_q))
			mod_timer(&dispatcher->timer, dispatch_q->expires);
	}


	if (gpudev->preemption_schedule)
		gpudev->preemption_schedule(adreno_dev);
	return 0;
}

static int dispatcher_context_sendcmds(struct adreno_device *adreno_dev,
		struct adreno_context *drawctxt)
{
	struct adreno_dispatcher_cmdqueue *dispatch_q =
					&(drawctxt->rb->dispatch_q);
	int count = 0;
	int ret = 0;
	int inflight = _cmdqueue_inflight(dispatch_q);
	unsigned int timestamp;

	if (dispatch_q->inflight >= inflight) {
		expire_markers(drawctxt);
		return -EBUSY;
	}

	while ((count < _context_cmdbatch_burst) &&
		(dispatch_q->inflight < inflight)) {
		struct kgsl_cmdbatch *cmdbatch;

		if (adreno_gpu_fault(adreno_dev) != 0)
			break;

		cmdbatch = adreno_dispatcher_get_cmdbatch(drawctxt);


		if (IS_ERR_OR_NULL(cmdbatch)) {
			if (IS_ERR(cmdbatch))
				ret = PTR_ERR(cmdbatch);
			break;
		}


		if (cmdbatch->flags & KGSL_CMDBATCH_SYNC) {
			kgsl_cmdbatch_destroy(cmdbatch);
			continue;
		}

		timestamp = cmdbatch->timestamp;

		ret = sendcmd(adreno_dev, cmdbatch);

		if (ret != 0) {
			
			if (ret == -ENOENT)
				kgsl_cmdbatch_destroy(cmdbatch);
			else {
				int r = adreno_dispatcher_requeue_cmdbatch(
					drawctxt, cmdbatch);
				if (r)
					ret = r;
			}

			break;
		}

		drawctxt->submitted_timestamp = timestamp;

		count++;
	}


	if (_check_context_queue(drawctxt))
		wake_up_all(&drawctxt->wq);

	if (!ret)
		ret = count;

	
	return ret;
}

static void _adreno_dispatcher_issuecmds(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	struct adreno_context *drawctxt, *next;
	struct plist_head requeue, busy_list;
	int ret;

	
	if (adreno_gpu_fault(adreno_dev) != 0)
		return;

	plist_head_init(&requeue);
	plist_head_init(&busy_list);

	
	while (1) {

		
		if (adreno_gpu_fault(adreno_dev) != 0)
			break;

		if (0 != adreno_gpu_halt(adreno_dev))
			break;

		spin_lock(&dispatcher->plist_lock);

		if (plist_head_empty(&dispatcher->pending)) {
			spin_unlock(&dispatcher->plist_lock);
			break;
		}

		
		drawctxt = plist_first_entry(&dispatcher->pending,
			struct adreno_context, pending);

		plist_del(&drawctxt->pending, &dispatcher->pending);

		spin_unlock(&dispatcher->plist_lock);

		if (kgsl_context_detached(&drawctxt->base) ||
			kgsl_context_invalid(&drawctxt->base)) {
			kgsl_context_put(&drawctxt->base);
			continue;
		}

		ret = dispatcher_context_sendcmds(adreno_dev, drawctxt);

		
		if (ret != 0 && ret != -ENOENT) {
			spin_lock(&dispatcher->plist_lock);


			if (!plist_node_empty(&drawctxt->pending)) {
				plist_del(&drawctxt->pending,
						&dispatcher->pending);
				kgsl_context_put(&drawctxt->base);
			}

			if (ret == -EBUSY)
				
				plist_add(&drawctxt->pending, &busy_list);
			else
				plist_add(&drawctxt->pending, &requeue);

			spin_unlock(&dispatcher->plist_lock);
		} else {

			kgsl_context_put(&drawctxt->base);
		}
	}

	spin_lock(&dispatcher->plist_lock);

	
	plist_for_each_entry_safe(drawctxt, next, &busy_list, pending) {
		plist_del(&drawctxt->pending, &busy_list);
		plist_add(&drawctxt->pending, &dispatcher->pending);
	}

	
	plist_for_each_entry_safe(drawctxt, next, &requeue, pending) {
		plist_del(&drawctxt->pending, &requeue);
		plist_add(&drawctxt->pending, &dispatcher->pending);
	}

	spin_unlock(&dispatcher->plist_lock);
}

static void adreno_dispatcher_issuecmds(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	
	if (!mutex_trylock(&dispatcher->mutex)) {
		adreno_dispatcher_schedule(KGSL_DEVICE(adreno_dev));
		return;
	}

	_adreno_dispatcher_issuecmds(adreno_dev);
	mutex_unlock(&dispatcher->mutex);
}

static int get_timestamp(struct adreno_context *drawctxt,
		struct kgsl_cmdbatch *cmdbatch, unsigned int *timestamp)
{
	
	if (cmdbatch->flags & KGSL_CMDBATCH_SYNC) {
		*timestamp = 0;
		return 0;
	}

	if (drawctxt->base.flags & KGSL_CONTEXT_USER_GENERATED_TS) {
		if (timestamp_cmp(drawctxt->timestamp, *timestamp) >= 0)
			return -ERANGE;

		drawctxt->timestamp = *timestamp;
	} else
		drawctxt->timestamp++;

	*timestamp = drawctxt->timestamp;
	return 0;
}

int adreno_dispatcher_queue_cmd(struct adreno_device *adreno_dev,
		struct adreno_context *drawctxt, struct kgsl_cmdbatch *cmdbatch,
		uint32_t *timestamp)
{
	struct adreno_dispatcher_cmdqueue *dispatch_q =
				ADRENO_CMDBATCH_DISPATCH_CMDQUEUE(cmdbatch);
	int ret;

	spin_lock(&drawctxt->lock);

	if (kgsl_context_detached(&drawctxt->base)) {
		spin_unlock(&drawctxt->lock);
		return -ENOENT;
	}


	if (test_and_clear_bit(ADRENO_CONTEXT_FORCE_PREAMBLE,
				&drawctxt->base.priv))
		set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &cmdbatch->priv);


	if ((drawctxt->base.flags & KGSL_CONTEXT_CTX_SWITCH) ||
		(cmdbatch->flags & KGSL_CMDBATCH_CTX_SWITCH))
		set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &cmdbatch->priv);

	
	if (drawctxt->base.flags & KGSL_CONTEXT_IFH_NOP)
		set_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv);


	if (test_bit(ADRENO_CONTEXT_SKIP_EOF, &drawctxt->base.priv)) {
		set_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv);


		if (cmdbatch->flags & KGSL_CMDBATCH_END_OF_FRAME) {
			clear_bit(ADRENO_CONTEXT_SKIP_EOF,
				  &drawctxt->base.priv);

			set_bit(ADRENO_CONTEXT_FORCE_PREAMBLE,
				&drawctxt->base.priv);
		}
	}

	

	while (drawctxt->queued >= _context_cmdqueue_size) {
		trace_adreno_drawctxt_sleep(drawctxt);
		spin_unlock(&drawctxt->lock);

		ret = wait_event_interruptible_timeout(drawctxt->wq,
			_check_context_queue(drawctxt),
			msecs_to_jiffies(_context_queue_wait));

		spin_lock(&drawctxt->lock);
		trace_adreno_drawctxt_wake(drawctxt);

		if (ret <= 0) {
			spin_unlock(&drawctxt->lock);
			return (ret == 0) ? -ETIMEDOUT : (int) ret;
		}
	}

	if (kgsl_context_invalid(&drawctxt->base)) {
		spin_unlock(&drawctxt->lock);
		return -EDEADLK;
	}
	if (kgsl_context_detached(&drawctxt->base)) {
		spin_unlock(&drawctxt->lock);
		return -ENOENT;
	}

	ret = get_timestamp(drawctxt, cmdbatch, timestamp);
	if (ret) {
		spin_unlock(&drawctxt->lock);
		return ret;
	}

	cmdbatch->timestamp = *timestamp;

	if (cmdbatch->flags & KGSL_CMDBATCH_MARKER) {


		if (!drawctxt->queued && kgsl_check_timestamp(cmdbatch->device,
			cmdbatch->context, drawctxt->queued_timestamp)) {
			trace_adreno_cmdbatch_queued(cmdbatch,
				drawctxt->queued);

			_retire_marker(cmdbatch);
			spin_unlock(&drawctxt->lock);
			return 0;
		}


		cmdbatch->marker_timestamp = drawctxt->queued_timestamp;
	}

	
	if (!(cmdbatch->flags & KGSL_CONTEXT_SYNC))
		drawctxt->queued_timestamp = *timestamp;


	if (drawctxt->base.flags & KGSL_CONTEXT_NO_FAULT_TOLERANCE)
		set_bit(KGSL_FT_DISABLE, &cmdbatch->fault_policy);
	else
		cmdbatch->fault_policy = adreno_dev->ft_policy;

	
	drawctxt->cmdqueue[drawctxt->cmdqueue_tail] = cmdbatch;
	drawctxt->cmdqueue_tail = (drawctxt->cmdqueue_tail + 1) %
		ADRENO_CONTEXT_CMDQUEUE_SIZE;


	if (!(cmdbatch->flags & KGSL_CMDBATCH_MARKER)) {
		unsigned int i = drawctxt->cmdqueue_head;

		while (i != drawctxt->cmdqueue_tail) {
			if (drawctxt->cmdqueue[i]->flags & KGSL_CMDBATCH_MARKER)
				set_bit(CMDBATCH_FLAG_SKIP,
					&drawctxt->cmdqueue[i]->priv);

			i = CMDQUEUE_NEXT(i, ADRENO_CONTEXT_CMDQUEUE_SIZE);
		}
	}

	drawctxt->queued++;
	trace_adreno_cmdbatch_queued(cmdbatch, drawctxt->queued);

	_track_context(adreno_dev, dispatch_q, drawctxt);

	spin_unlock(&drawctxt->lock);

	kgsl_pwrctrl_update_l2pc(&adreno_dev->dev);

	
	dispatcher_queue_context(adreno_dev, drawctxt);


	if (dispatch_q->inflight < _context_cmdbatch_burst)
		adreno_dispatcher_issuecmds(adreno_dev);

	return 0;
}

static int _mark_context(int id, void *ptr, void *data)
{
	unsigned int guilty = *((unsigned int *) data);
	struct kgsl_context *context = ptr;


	if (guilty == 0 || guilty == context->id)
		context->reset_status =
			KGSL_CTX_STAT_GUILTY_CONTEXT_RESET_EXT;
	else if (context->reset_status !=
		KGSL_CTX_STAT_GUILTY_CONTEXT_RESET_EXT)
		context->reset_status =
			KGSL_CTX_STAT_INNOCENT_CONTEXT_RESET_EXT;

	return 0;
}

static void mark_guilty_context(struct kgsl_device *device, unsigned int id)
{
	

	read_lock(&device->context_lock);
	idr_for_each(&device->context_idr, _mark_context, &id);
	read_unlock(&device->context_lock);
}

static void cmdbatch_skip_ib(struct kgsl_cmdbatch *cmdbatch, uint64_t base)
{
	struct kgsl_memobj_node *ib;

	list_for_each_entry(ib, &cmdbatch->cmdlist, node) {
		if (ib->gpuaddr == base) {
			ib->priv |= MEMOBJ_SKIP;
			if (base)
				return;
		}
	}
}

static void cmdbatch_skip_cmd(struct kgsl_cmdbatch *cmdbatch,
	struct kgsl_cmdbatch **replay, int count)
{
	struct adreno_context *drawctxt = ADRENO_CONTEXT(cmdbatch->context);
	int i;

	for (i = 1; i < count; i++) {
		if (replay[i]->context->id == cmdbatch->context->id) {
			replay[i]->fault_policy = replay[0]->fault_policy;
			set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &replay[i]->priv);
			set_bit(KGSL_FT_SKIPCMD, &replay[i]->fault_recovery);
			break;
		}
	}

	if ((i == count) && drawctxt) {
		set_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->base.priv);
		drawctxt->fault_policy = replay[0]->fault_policy;
	}

	
	set_bit(CMDBATCH_FLAG_SKIP, &cmdbatch->priv);
	cmdbatch->fault_recovery = 0;
}

static void cmdbatch_skip_frame(struct kgsl_cmdbatch *cmdbatch,
	struct kgsl_cmdbatch **replay, int count)
{
	struct adreno_context *drawctxt = ADRENO_CONTEXT(cmdbatch->context);
	int skip = 1;
	int i;

	for (i = 0; i < count; i++) {


		if (replay[i]->context->id != cmdbatch->context->id)
			continue;


		if (skip) {
			set_bit(CMDBATCH_FLAG_SKIP, &replay[i]->priv);

			if (replay[i]->flags & KGSL_CMDBATCH_END_OF_FRAME)
				skip = 0;
		} else {
			set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &replay[i]->priv);
			return;
		}
	}


	if (skip && drawctxt)
		set_bit(ADRENO_CONTEXT_SKIP_EOF, &drawctxt->base.priv);


	if (!skip && drawctxt)
		set_bit(ADRENO_CONTEXT_FORCE_PREAMBLE, &drawctxt->base.priv);
}

static void remove_invalidated_cmdbatches(struct kgsl_device *device,
		struct kgsl_cmdbatch **replay, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		struct kgsl_cmdbatch *cmd = replay[i];
		if (cmd == NULL)
			continue;

		if (kgsl_context_detached(cmd->context) ||
			kgsl_context_invalid(cmd->context)) {
			replay[i] = NULL;

			mutex_lock(&device->mutex);
			kgsl_cancel_events_timestamp(device,
				&cmd->context->events, cmd->timestamp);
			mutex_unlock(&device->mutex);

			kgsl_cmdbatch_destroy(cmd);
		}
	}
}

static char _pidname[TASK_COMM_LEN];

static inline const char *_kgsl_context_comm(struct kgsl_context *context)
{
	if (context && context->proc_priv)
		strlcpy(_pidname, context->proc_priv->comm, sizeof(_pidname));
	else
		snprintf(_pidname, TASK_COMM_LEN, "unknown");

	return _pidname;
}

#define pr_fault(_d, _c, fmt, args...) \
		dev_err((_d)->dev, "%s[%d]: " fmt, \
		_kgsl_context_comm((_c)->context), \
		(_c)->context->proc_priv->pid, ##args)


static void adreno_fault_header(struct kgsl_device *device,
		struct adreno_ringbuffer *rb, struct kgsl_cmdbatch *cmdbatch)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int status, rptr, wptr, ib1sz, ib2sz;
	uint64_t ib1base, ib2base;

	adreno_readreg(adreno_dev , ADRENO_REG_RBBM_STATUS, &status);
	adreno_readreg(adreno_dev, ADRENO_REG_CP_RB_RPTR, &rptr);
	adreno_readreg(adreno_dev, ADRENO_REG_CP_RB_WPTR, &wptr);
	adreno_readreg64(adreno_dev, ADRENO_REG_CP_IB1_BASE,
					  ADRENO_REG_CP_IB1_BASE_HI, &ib1base);
	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB1_BUFSZ, &ib1sz);
	adreno_readreg64(adreno_dev, ADRENO_REG_CP_IB2_BASE,
					   ADRENO_REG_CP_IB2_BASE_HI, &ib2base);
	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB2_BUFSZ, &ib2sz);

	if (cmdbatch != NULL) {
		struct adreno_context *drawctxt =
			ADRENO_CONTEXT(cmdbatch->context);

		trace_adreno_gpu_fault(cmdbatch->context->id,
			cmdbatch->timestamp,
			status, rptr, wptr, ib1base, ib1sz,
			ib2base, ib2sz, drawctxt->rb->id);

		pr_fault(device, cmdbatch,
			"gpu fault ctx %d ts %d status %8.8X rb %4.4x/%4.4x ib1 %16.16llX/%4.4x ib2 %16.16llX/%4.4x\n",
			cmdbatch->context->id, cmdbatch->timestamp, status,
			rptr, wptr, ib1base, ib1sz, ib2base, ib2sz);

		if (rb != NULL)
			pr_fault(device, cmdbatch,
				"gpu fault rb %d rb sw r/w %4.4x/%4.4x\n",
				rb->id, rptr, rb->wptr);
	} else {
		int id = (rb != NULL) ? rb->id : -1;

		dev_err(device->dev,
			"RB[%d]: gpu fault status %8.8X rb %4.4x/%4.4x ib1 %16.16llX/%4.4x ib2 %16.16llX/%4.4x\n",
			id, status, rptr, wptr, ib1base, ib1sz, ib2base,
			ib2sz);
		if (rb != NULL)
			dev_err(device->dev,
				"RB[%d] gpu fault rb sw r/w %4.4x/%4.4x\n",
				rb->id, rptr, rb->wptr);
	}
}

void adreno_fault_skipcmd_detached(struct adreno_device *adreno_dev,
				 struct adreno_context *drawctxt,
				 struct kgsl_cmdbatch *cmdbatch)
{
	if (test_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->base.priv) &&
			kgsl_context_detached(&drawctxt->base)) {
		pr_context(KGSL_DEVICE(adreno_dev), cmdbatch->context,
			"gpu detached context %d\n", cmdbatch->context->id);
		clear_bit(ADRENO_CONTEXT_SKIP_CMD, &drawctxt->base.priv);
	}
}

static void process_cmdbatch_fault(struct kgsl_device *device,
		struct kgsl_cmdbatch **replay, int count,
		unsigned int base,
		int fault)
{
	struct kgsl_cmdbatch *cmdbatch = replay[0];
	int i;
	char *state = "failed";

	if (test_bit(KGSL_FT_THROTTLE, &cmdbatch->fault_policy)) {
		if (time_after(jiffies, (cmdbatch->context->fault_time
				+ msecs_to_jiffies(_fault_throttle_time)))) {
			cmdbatch->context->fault_time = jiffies;
			cmdbatch->context->fault_count = 1;
		} else {
			cmdbatch->context->fault_count++;
			if (cmdbatch->context->fault_count >
					_fault_throttle_burst) {
				set_bit(KGSL_FT_DISABLE,
						&cmdbatch->fault_policy);
				pr_context(device, cmdbatch->context,
					 "gpu fault threshold exceeded %d faults in %d msecs\n",
					 _fault_throttle_burst,
					 _fault_throttle_time);
			}
		}
	}


	if (test_bit(KGSL_FT_DISABLE, &cmdbatch->fault_policy) ||
		test_bit(KGSL_FT_TEMP_DISABLE, &cmdbatch->fault_policy)) {
		state = "skipped";
		bitmap_zero(&cmdbatch->fault_policy, BITS_PER_LONG);
	}

	
	if (kgsl_context_detached(cmdbatch->context)) {
		state = "detached";
		bitmap_zero(&cmdbatch->fault_policy, BITS_PER_LONG);
	}


	set_bit(KGSL_FT_SKIP_PMDUMP, &cmdbatch->fault_policy);


	if (fault & ADRENO_HARD_FAULT)
		clear_bit(KGSL_FT_REPLAY, &(cmdbatch->fault_policy));


	if (fault & ADRENO_TIMEOUT_FAULT)
		bitmap_zero(&cmdbatch->fault_policy, BITS_PER_LONG);


	if (test_bit(KGSL_CONTEXT_PRIV_PAGEFAULT,
		     &cmdbatch->context->priv)) {
		
		clear_bit(KGSL_FT_REPLAY, &cmdbatch->fault_policy);
		clear_bit(KGSL_CONTEXT_PRIV_PAGEFAULT,
			  &cmdbatch->context->priv);
	}


	
	if (test_and_clear_bit(KGSL_FT_REPLAY, &cmdbatch->fault_policy)) {
		trace_adreno_cmdbatch_recovery(cmdbatch, BIT(KGSL_FT_REPLAY));
		set_bit(KGSL_FT_REPLAY, &cmdbatch->fault_recovery);
		return;
	}


	if (test_and_clear_bit(KGSL_FT_SKIPIB, &cmdbatch->fault_policy)) {
		trace_adreno_cmdbatch_recovery(cmdbatch, BIT(KGSL_FT_SKIPIB));
		set_bit(KGSL_FT_SKIPIB, &cmdbatch->fault_recovery);

		for (i = 0; i < count; i++) {
			if (replay[i] != NULL &&
				replay[i]->context->id == cmdbatch->context->id)
				cmdbatch_skip_ib(replay[i], base);
		}

		return;
	}

	
	if (test_and_clear_bit(KGSL_FT_SKIPCMD, &cmdbatch->fault_policy)) {
		trace_adreno_cmdbatch_recovery(cmdbatch, BIT(KGSL_FT_SKIPCMD));

		
		cmdbatch_skip_cmd(cmdbatch, replay, count);

		return;
	}

	if (test_and_clear_bit(KGSL_FT_SKIPFRAME, &cmdbatch->fault_policy)) {
		trace_adreno_cmdbatch_recovery(cmdbatch,
			BIT(KGSL_FT_SKIPFRAME));
		set_bit(KGSL_FT_SKIPFRAME, &cmdbatch->fault_recovery);

		cmdbatch_skip_frame(cmdbatch, replay, count);
		return;
	}

	

	pr_context(device, cmdbatch->context, "gpu %s ctx %d ts %d\n",
		state, cmdbatch->context->id, cmdbatch->timestamp);

	
	mark_guilty_context(device, cmdbatch->context->id);

	
	adreno_drawctxt_invalidate(device, cmdbatch->context);
}

static void recover_dispatch_q(struct kgsl_device *device,
		struct adreno_dispatcher_cmdqueue *dispatch_q,
		int fault,
		unsigned int base)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_cmdbatch **replay = NULL;
	unsigned int ptr;
	int first = 0;
	int count = 0;
	int i;

	
	replay = kzalloc(sizeof(*replay) * dispatch_q->inflight, GFP_KERNEL);

	if (replay == NULL) {
		unsigned int ptr = dispatch_q->head;

		
		while (ptr != dispatch_q->tail) {
			struct kgsl_context *context =
				dispatch_q->cmd_q[ptr]->context;

			mark_guilty_context(device, context->id);
			adreno_drawctxt_invalidate(device, context);
			kgsl_cmdbatch_destroy(dispatch_q->cmd_q[ptr]);

			ptr = CMDQUEUE_NEXT(ptr, ADRENO_DISPATCH_CMDQUEUE_SIZE);
		}


		count = 0;
		goto replay;
	}

	
	ptr = dispatch_q->head;

	while (ptr != dispatch_q->tail) {
		replay[count++] = dispatch_q->cmd_q[ptr];
		ptr = CMDQUEUE_NEXT(ptr, ADRENO_DISPATCH_CMDQUEUE_SIZE);
	}

	if (fault && count)
		process_cmdbatch_fault(device, replay,
					count, base, fault);
replay:
	dispatch_q->inflight = 0;
	dispatch_q->head = dispatch_q->tail = 0;
	
	remove_invalidated_cmdbatches(device, replay, count);

	
	for (i = 0; i < count; i++) {

		int ret;

		if (replay[i] == NULL)
			continue;


		if (first == 0) {
			set_bit(CMDBATCH_FLAG_FORCE_PREAMBLE, &replay[i]->priv);
			first = 1;
		}


		set_bit(CMDBATCH_FLAG_WFI, &replay[i]->priv);

		ret = sendcmd(adreno_dev, replay[i]);


		if (ret) {
			pr_context(device, replay[i]->context,
				"gpu reset failed ctx %d ts %d\n",
				replay[i]->context->id, replay[i]->timestamp);

			
			mark_guilty_context(device, replay[i]->context->id);

			adreno_drawctxt_invalidate(device, replay[i]->context);
			remove_invalidated_cmdbatches(device, &replay[i],
				count - i);
		}
	}

	
	clear_bit(ADRENO_DEVICE_FAULT, &adreno_dev->priv);

	kfree(replay);
}

static void do_header_and_snapshot(struct kgsl_device *device,
		struct adreno_ringbuffer *rb, struct kgsl_cmdbatch *cmdbatch)
{
	
	if (cmdbatch == NULL) {
		adreno_fault_header(device, rb, NULL);
		kgsl_device_snapshot(device, NULL);
		return;
	}

	
	if (test_bit(KGSL_FT_SKIP_PMDUMP, &cmdbatch->fault_policy))
		return;

	
	adreno_fault_header(device, rb, cmdbatch);

	if (!(cmdbatch->context->flags & KGSL_CONTEXT_NO_SNAPSHOT))
		kgsl_device_snapshot(device, cmdbatch->context);
}

static int dispatcher_do_fault(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	struct adreno_dispatcher_cmdqueue *dispatch_q = NULL, *dispatch_q_temp;
	struct adreno_ringbuffer *rb;
	struct adreno_ringbuffer *hung_rb = NULL;
	unsigned int reg;
	uint64_t base;
	struct kgsl_cmdbatch *cmdbatch = NULL;
	int ret, i;
	int fault;
	int halt;
	int keepfault, fault_pid = 0;

	fault = atomic_xchg(&dispatcher->fault, 0);
	keepfault = fault;
	if (fault == 0)
		return 0;

	if (!(fault & ADRENO_IOMMU_PAGE_FAULT) && adreno_is_a5xx(adreno_dev)) {
		unsigned int val;
		mutex_lock(&device->mutex);
		adreno_readreg(adreno_dev, ADRENO_REG_RBBM_STATUS3, &val);
		mutex_unlock(&device->mutex);
		if (val & BIT(24))
			return 0;
	}

	
	del_timer_sync(&dispatcher->timer);
	del_timer_sync(&dispatcher->fault_timer);
	del_timer_sync(&adreno_dev->preempt.timer);

	mutex_lock(&device->mutex);

	
	kgsl_cffdump_hang(device);

	adreno_readreg64(adreno_dev, ADRENO_REG_CP_RB_BASE,
		ADRENO_REG_CP_RB_BASE_HI, &base);

	if (!(fault & ADRENO_HARD_FAULT)) {
		adreno_readreg(adreno_dev, ADRENO_REG_CP_ME_CNTL, &reg);
		if (adreno_is_a5xx(adreno_dev))
			reg |= 1 | (1 << 1);
		else
			reg |= (1 << 27) | (1 << 28);
		adreno_writereg(adreno_dev, ADRENO_REG_CP_ME_CNTL, reg);
	}
	FOR_EACH_RINGBUFFER(adreno_dev, rb, i) {
		adreno_dispatch_retire_cmdqueue(adreno_dev,
			&(rb->dispatch_q));
		
		if (base == rb->buffer_desc.gpuaddr) {
			dispatch_q = &(rb->dispatch_q);
			hung_rb = rb;
			if (adreno_dev->cur_rb != hung_rb) {
				adreno_dev->prev_rb = adreno_dev->cur_rb;
				adreno_dev->cur_rb = hung_rb;
			}
		}
		if (ADRENO_DISPATCHER_RB_STARVE_TIMER_ELAPSED ==
			rb->starve_timer_state) {
			adreno_put_gpu_halt(adreno_dev);
			rb->starve_timer_state =
			ADRENO_DISPATCHER_RB_STARVE_TIMER_UNINIT;
		}
	}

	if (!adreno_cmdqueue_is_empty(dispatch_q)) {
		cmdbatch = dispatch_q->cmd_q[dispatch_q->head];
		fault_pid = cmdbatch->context->proc_priv->pid;
		trace_adreno_cmdbatch_fault(cmdbatch, fault);
	}

	adreno_readreg64(adreno_dev, ADRENO_REG_CP_IB1_BASE,
		ADRENO_REG_CP_IB1_BASE_HI, &base);

	do_header_and_snapshot(device, hung_rb, cmdbatch);

	
	if (fault & ADRENO_IOMMU_PAGE_FAULT)
		kgsl_mmu_pagefault_resume(&device->mmu);

	
	dispatcher->inflight = 0;

	
	halt = adreno_gpu_halt(adreno_dev);
	adreno_clear_gpu_halt(adreno_dev);


	if (hung_rb != NULL) {
		kgsl_sharedmem_writel(device, &device->memstore,
				MEMSTORE_RB_OFFSET(hung_rb, soptimestamp),
				hung_rb->timestamp);

		kgsl_sharedmem_writel(device, &device->memstore,
				MEMSTORE_RB_OFFSET(hung_rb, eoptimestamp),
				hung_rb->timestamp);

		
		kgsl_process_event_group(device, &hung_rb->events);
	}

	ret = adreno_reset(device, fault);
	mutex_unlock(&device->mutex);
	
	atomic_set(&dispatcher->fault, 0);

	
	BUG_ON(ret);

	
	if (dispatch_q)
		recover_dispatch_q(device, dispatch_q, fault, base);
	FOR_EACH_RINGBUFFER(adreno_dev, rb, i) {
		dispatch_q_temp = &(rb->dispatch_q);
		if (dispatch_q_temp != dispatch_q)
			recover_dispatch_q(device, dispatch_q_temp, 0, base);
	}

	atomic_add(halt, &adreno_dev->halt);

	adreno_fault_panic(device, fault_pid, keepfault);

	return 1;
}

static inline int cmdbatch_consumed(struct kgsl_cmdbatch *cmdbatch,
		unsigned int consumed, unsigned int retired)
{
	return ((timestamp_cmp(cmdbatch->timestamp, consumed) >= 0) &&
		(timestamp_cmp(retired, cmdbatch->timestamp) < 0));
}

static void _print_recovery(struct kgsl_device *device,
		struct kgsl_cmdbatch *cmdbatch)
{
	static struct {
		unsigned int mask;
		const char *str;
	} flags[] = { ADRENO_FT_TYPES };

	int i, nr = find_first_bit(&cmdbatch->fault_recovery, BITS_PER_LONG);
	char *result = "unknown";

	for (i = 0; i < ARRAY_SIZE(flags); i++) {
		if (flags[i].mask == BIT(nr)) {
			result = (char *) flags[i].str;
			break;
		}
	}

	pr_context(device, cmdbatch->context,
		"gpu %s ctx %d ts %d policy %lX\n",
		result, cmdbatch->context->id, cmdbatch->timestamp,
		cmdbatch->fault_recovery);
}

static void cmdbatch_profile_ticks(struct adreno_device *adreno_dev,
	struct kgsl_cmdbatch *cmdbatch, uint64_t *start, uint64_t *retire)
{
	void *ptr = adreno_dev->cmdbatch_profile_buffer.hostptr;
	struct adreno_cmdbatch_profile_entry *entry;

	entry = (struct adreno_cmdbatch_profile_entry *)
		(ptr + (cmdbatch->profile_index * sizeof(*entry)));

	rmb();
	*start = entry->started;
	*retire = entry->retired;
}

static void retire_cmdbatch(struct adreno_device *adreno_dev,
		struct kgsl_cmdbatch *cmdbatch)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	struct adreno_context *drawctxt = ADRENO_CONTEXT(cmdbatch->context);
	uint64_t start = 0, end = 0;

	if (cmdbatch->fault_recovery != 0) {
		set_bit(ADRENO_CONTEXT_FAULT, &cmdbatch->context->priv);
		_print_recovery(KGSL_DEVICE(adreno_dev), cmdbatch);
	}

	if (test_bit(CMDBATCH_FLAG_PROFILE, &cmdbatch->priv))
		cmdbatch_profile_ticks(adreno_dev, cmdbatch, &start, &end);

	if (adreno_is_a3xx(adreno_dev))
		trace_adreno_cmdbatch_retired(cmdbatch,
				(int) dispatcher->inflight, start, end,
				ADRENO_CMDBATCH_RB(cmdbatch), 0);
	else
		trace_adreno_cmdbatch_retired(cmdbatch,
				(int) dispatcher->inflight, start, end,
				ADRENO_CMDBATCH_RB(cmdbatch),
				adreno_get_rptr(drawctxt->rb));

	drawctxt->submit_retire_ticks[drawctxt->ticks_index] =
		end - cmdbatch->submit_ticks;

	drawctxt->ticks_index = (drawctxt->ticks_index + 1) %
		SUBMIT_RETIRE_TICKS_SIZE;

	kgsl_cmdbatch_destroy(cmdbatch);
}

static int adreno_dispatch_retire_cmdqueue(struct adreno_device *adreno_dev,
		struct adreno_dispatcher_cmdqueue *cmdqueue)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	int count = 0;

	while (!adreno_cmdqueue_is_empty(cmdqueue)) {
		struct kgsl_cmdbatch *cmdbatch =
			cmdqueue->cmd_q[cmdqueue->head];

		if (!kgsl_check_timestamp(device, cmdbatch->context,
			cmdbatch->timestamp))
			break;

		retire_cmdbatch(adreno_dev, cmdbatch);

		dispatcher->inflight--;
		cmdqueue->inflight--;

		cmdqueue->cmd_q[cmdqueue->head] = NULL;

		cmdqueue->head = CMDQUEUE_NEXT(cmdqueue->head,
			ADRENO_DISPATCH_CMDQUEUE_SIZE);

		count++;
	}

	return count;
}

static void _adreno_dispatch_check_timeout(struct adreno_device *adreno_dev,
		struct adreno_dispatcher_cmdqueue *cmdqueue)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_cmdbatch *cmdbatch = cmdqueue->cmd_q[cmdqueue->head];

	
	if (time_is_after_jiffies(cmdqueue->expires))
		return;

	
	if (!adreno_long_ib_detect(adreno_dev))
		return;

	
	if (cmdbatch->context->flags & KGSL_CONTEXT_NO_FAULT_TOLERANCE)
		return;

	pr_context(device, cmdbatch->context, "gpu timeout ctx %d ts %d\n",
		cmdbatch->context->id, cmdbatch->timestamp);

	adreno_set_gpu_fault(adreno_dev, ADRENO_TIMEOUT_FAULT);
}

static int adreno_dispatch_process_cmdqueue(struct adreno_device *adreno_dev,
		struct adreno_dispatcher_cmdqueue *cmdqueue)
{
	int count = adreno_dispatch_retire_cmdqueue(adreno_dev, cmdqueue);

	
	if (adreno_cmdqueue_is_empty(cmdqueue))
		return count;

	
	if (!adreno_in_preempt_state(adreno_dev, ADRENO_PREEMPT_NONE))
		return count;

	
	if (!cmdqueue_is_current(cmdqueue))
		return count;


	if (count) {
		cmdqueue->expires = jiffies +
			msecs_to_jiffies(adreno_cmdbatch_timeout);
		return count;
	}

	_adreno_dispatch_check_timeout(adreno_dev, cmdqueue);
	return 0;
}

static void _dispatcher_update_timers(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	
	mutex_lock(&device->mutex);
	kgsl_pwrscale_update(device);
	mod_timer(&device->idle_timer,
		jiffies + device->pwrctrl.interval_timeout);
	mutex_unlock(&device->mutex);

	
	if (adreno_in_preempt_state(adreno_dev, ADRENO_PREEMPT_NONE)) {
		struct adreno_dispatcher_cmdqueue *cmdqueue =
			CMDQUEUE(adreno_dev->cur_rb);

		if (!adreno_cmdqueue_is_empty(cmdqueue))
			mod_timer(&dispatcher->timer, cmdqueue->expires);
	}
}

static void _dispatcher_power_down(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	mutex_lock(&device->mutex);

	if (test_and_clear_bit(ADRENO_DISPATCHER_ACTIVE, &dispatcher->priv))
		complete_all(&dispatcher->idle_gate);

	del_timer_sync(&dispatcher->fault_timer);

	if (test_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv)) {
		kgsl_active_count_put(device);
		clear_bit(ADRENO_DISPATCHER_POWER, &dispatcher->priv);
	}

	mutex_unlock(&device->mutex);
}

static void adreno_dispatcher_work(struct work_struct *work)
{
	struct adreno_dispatcher *dispatcher =
		container_of(work, struct adreno_dispatcher, work);
	struct adreno_device *adreno_dev =
		container_of(dispatcher, struct adreno_device, dispatcher);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	int count = 0;
	unsigned int i = 0;

	mutex_lock(&dispatcher->mutex);

	for (i = 0; i < adreno_dev->num_ringbuffers; i++) {
		struct adreno_dispatcher_cmdqueue *cmdqueue =
			CMDQUEUE(&adreno_dev->ringbuffers[i]);

		count += adreno_dispatch_process_cmdqueue(adreno_dev,
			cmdqueue);
		if (dispatcher->inflight == 0)
			break;
	}

	kgsl_process_event_groups(device);

	if (dispatcher_do_fault(adreno_dev) == 0) {

		
		if (gpudev->preemption_schedule)
			gpudev->preemption_schedule(adreno_dev);

		
		_adreno_dispatcher_issuecmds(adreno_dev);
	}

	if (dispatcher->inflight > 0)
		_dispatcher_update_timers(adreno_dev);
	else
		_dispatcher_power_down(adreno_dev);

	mutex_unlock(&dispatcher->mutex);
}

void adreno_dispatcher_schedule(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	kgsl_schedule_work(&dispatcher->work);
}

void adreno_dispatcher_queue_context(struct kgsl_device *device,
	struct adreno_context *drawctxt)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	dispatcher_queue_context(adreno_dev, drawctxt);
	adreno_dispatcher_schedule(device);
}


static void adreno_dispatcher_fault_timer(unsigned long data)
{
	struct adreno_device *adreno_dev = (struct adreno_device *) data;
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	
	if (!adreno_soft_fault_detect(adreno_dev))
		return;

	if (adreno_gpu_fault(adreno_dev)) {
		adreno_dispatcher_schedule(KGSL_DEVICE(adreno_dev));
		return;
	}


	if (!fault_detect_read_compare(adreno_dev)) {
		adreno_set_gpu_fault(adreno_dev, ADRENO_SOFT_FAULT);
		adreno_dispatcher_schedule(KGSL_DEVICE(adreno_dev));
	} else {
		mod_timer(&dispatcher->fault_timer,
			jiffies + msecs_to_jiffies(_fault_timer_interval));
	}
}

static void adreno_dispatcher_timer(unsigned long data)
{
	struct adreno_device *adreno_dev = (struct adreno_device *) data;

	adreno_dispatcher_schedule(KGSL_DEVICE(adreno_dev));
}

void adreno_dispatcher_start(struct kgsl_device *device)
{
	complete_all(&device->cmdbatch_gate);

	
	adreno_dispatcher_schedule(device);
}

void adreno_dispatcher_stop(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;

	del_timer_sync(&dispatcher->timer);
	del_timer_sync(&dispatcher->fault_timer);
}

void adreno_dispatcher_close(struct adreno_device *adreno_dev)
{
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	int i;
	struct adreno_ringbuffer *rb;

	mutex_lock(&dispatcher->mutex);
	del_timer_sync(&dispatcher->timer);
	del_timer_sync(&dispatcher->fault_timer);

	FOR_EACH_RINGBUFFER(adreno_dev, rb, i) {
		struct adreno_dispatcher_cmdqueue *dispatch_q =
			&(rb->dispatch_q);
		while (!adreno_cmdqueue_is_empty(dispatch_q)) {
			kgsl_cmdbatch_destroy(
				dispatch_q->cmd_q[dispatch_q->head]);
			dispatch_q->head = (dispatch_q->head + 1)
				% ADRENO_DISPATCH_CMDQUEUE_SIZE;
		}
	}

	mutex_unlock(&dispatcher->mutex);

	kobject_put(&dispatcher->kobj);
}

struct dispatcher_attribute {
	struct attribute attr;
	ssize_t (*show)(struct adreno_dispatcher *,
			struct dispatcher_attribute *, char *);
	ssize_t (*store)(struct adreno_dispatcher *,
			struct dispatcher_attribute *, const char *buf,
			size_t count);
	unsigned int max;
	unsigned int *value;
};

#define DISPATCHER_UINT_ATTR(_name, _mode, _max, _value) \
	struct dispatcher_attribute dispatcher_attr_##_name =  { \
		.attr = { .name = __stringify(_name), .mode = _mode }, \
		.show = _show_uint, \
		.store = _store_uint, \
		.max = _max, \
		.value = &(_value), \
	}

#define to_dispatcher_attr(_a) \
	container_of((_a), struct dispatcher_attribute, attr)
#define to_dispatcher(k) container_of(k, struct adreno_dispatcher, kobj)

static ssize_t _store_uint(struct adreno_dispatcher *dispatcher,
		struct dispatcher_attribute *attr,
		const char *buf, size_t size)
{
	unsigned int val = 0;
	int ret;

	ret = kgsl_sysfs_store(buf, &val);
	if (ret)
		return ret;

	if (!val || (attr->max && (val > attr->max)))
		return -EINVAL;

	*((unsigned int *) attr->value) = val;
	return size;
}

static ssize_t _show_uint(struct adreno_dispatcher *dispatcher,
		struct dispatcher_attribute *attr,
		char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",
		*((unsigned int *) attr->value));
}

static DISPATCHER_UINT_ATTR(inflight, 0644, ADRENO_DISPATCH_CMDQUEUE_SIZE,
	_dispatcher_q_inflight_hi);

static DISPATCHER_UINT_ATTR(inflight_low_latency, 0644,
	ADRENO_DISPATCH_CMDQUEUE_SIZE, _dispatcher_q_inflight_lo);
static DISPATCHER_UINT_ATTR(context_cmdqueue_size, 0644,
	ADRENO_CONTEXT_CMDQUEUE_SIZE - 1, _context_cmdqueue_size);
static DISPATCHER_UINT_ATTR(context_burst_count, 0644, 0,
	_context_cmdbatch_burst);
static DISPATCHER_UINT_ATTR(cmdbatch_timeout, 0644, 0,
	adreno_cmdbatch_timeout);
static DISPATCHER_UINT_ATTR(context_queue_wait, 0644, 0, _context_queue_wait);
static DISPATCHER_UINT_ATTR(fault_detect_interval, 0644, 0,
	_fault_timer_interval);
static DISPATCHER_UINT_ATTR(fault_throttle_time, 0644, 0,
	_fault_throttle_time);
static DISPATCHER_UINT_ATTR(fault_throttle_burst, 0644, 0,
	_fault_throttle_burst);
static DISPATCHER_UINT_ATTR(disp_preempt_fair_sched, 0644, 0,
	adreno_disp_preempt_fair_sched);
static DISPATCHER_UINT_ATTR(dispatch_time_slice, 0644, 0,
	adreno_dispatch_time_slice);
static DISPATCHER_UINT_ATTR(dispatch_starvation_time, 0644, 0,
	adreno_dispatch_starvation_time);

static struct attribute *dispatcher_attrs[] = {
	&dispatcher_attr_inflight.attr,
	&dispatcher_attr_inflight_low_latency.attr,
	&dispatcher_attr_context_cmdqueue_size.attr,
	&dispatcher_attr_context_burst_count.attr,
	&dispatcher_attr_cmdbatch_timeout.attr,
	&dispatcher_attr_context_queue_wait.attr,
	&dispatcher_attr_fault_detect_interval.attr,
	&dispatcher_attr_fault_throttle_time.attr,
	&dispatcher_attr_fault_throttle_burst.attr,
	&dispatcher_attr_disp_preempt_fair_sched.attr,
	&dispatcher_attr_dispatch_time_slice.attr,
	&dispatcher_attr_dispatch_starvation_time.attr,
	NULL,
};

static ssize_t dispatcher_sysfs_show(struct kobject *kobj,
				   struct attribute *attr, char *buf)
{
	struct adreno_dispatcher *dispatcher = to_dispatcher(kobj);
	struct dispatcher_attribute *pattr = to_dispatcher_attr(attr);
	ssize_t ret = -EIO;

	if (pattr->show)
		ret = pattr->show(dispatcher, pattr, buf);

	return ret;
}

static ssize_t dispatcher_sysfs_store(struct kobject *kobj,
				    struct attribute *attr,
				    const char *buf, size_t count)
{
	struct adreno_dispatcher *dispatcher = to_dispatcher(kobj);
	struct dispatcher_attribute *pattr = to_dispatcher_attr(attr);
	ssize_t ret = -EIO;

	if (pattr->store)
		ret = pattr->store(dispatcher, pattr, buf, count);

	return ret;
}

static const struct sysfs_ops dispatcher_sysfs_ops = {
	.show = dispatcher_sysfs_show,
	.store = dispatcher_sysfs_store
};

static struct kobj_type ktype_dispatcher = {
	.sysfs_ops = &dispatcher_sysfs_ops,
	.default_attrs = dispatcher_attrs,
};

int adreno_dispatcher_init(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	int ret;

	memset(dispatcher, 0, sizeof(*dispatcher));

	mutex_init(&dispatcher->mutex);

	setup_timer(&dispatcher->timer, adreno_dispatcher_timer,
		(unsigned long) adreno_dev);

	setup_timer(&dispatcher->fault_timer, adreno_dispatcher_fault_timer,
		(unsigned long) adreno_dev);

	INIT_WORK(&dispatcher->work, adreno_dispatcher_work);

	init_completion(&dispatcher->idle_gate);
	complete_all(&dispatcher->idle_gate);

	plist_head_init(&dispatcher->pending);
	spin_lock_init(&dispatcher->plist_lock);

	ret = kobject_init_and_add(&dispatcher->kobj, &ktype_dispatcher,
		&device->dev->kobj, "dispatch");

	return ret;
}

int adreno_dispatcher_idle(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_dispatcher *dispatcher = &adreno_dev->dispatcher;
	int ret;

	BUG_ON(!mutex_is_locked(&device->mutex));
	if (!test_bit(ADRENO_DEVICE_STARTED, &adreno_dev->priv))
		return 0;

	if (mutex_is_locked(&dispatcher->mutex) &&
		dispatcher->mutex.owner == current)
		BUG_ON(1);

	adreno_get_gpu_halt(adreno_dev);

	mutex_unlock(&device->mutex);

	ret = wait_for_completion_timeout(&dispatcher->idle_gate,
			msecs_to_jiffies(ADRENO_IDLE_TIMEOUT));
	if (ret == 0) {
		ret = -ETIMEDOUT;
		WARN(1, "Dispatcher halt timeout ");
	} else if (ret < 0) {
		KGSL_DRV_ERR(device, "Dispatcher halt failed %d\n", ret);
	} else {
		ret = 0;
	}

	mutex_lock(&device->mutex);
	adreno_put_gpu_halt(adreno_dev);
	adreno_dispatcher_schedule(device);
	return ret;
}
