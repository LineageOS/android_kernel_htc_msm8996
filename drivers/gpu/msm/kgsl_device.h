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
#ifndef __KGSL_DEVICE_H
#define __KGSL_DEVICE_H

#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/pm_qos.h>
#include <linux/sched.h>

#include "kgsl.h"
#include "kgsl_mmu.h"
#include "kgsl_pwrctrl.h"
#include "kgsl_log.h"
#include "kgsl_pwrscale.h"
#include "kgsl_snapshot.h"
#include "kgsl_sharedmem.h"
#include "kgsl_cmdbatch.h"

#include <linux/sync.h>

#define KGSL_IOCTL_FUNC(_cmd, _func) \
	[_IOC_NR((_cmd))] = \
		{ .cmd = (_cmd), .func = (_func) }


#define KGSL_STATE_NONE		0x00000000
#define KGSL_STATE_INIT		0x00000001
#define KGSL_STATE_ACTIVE	0x00000002
#define KGSL_STATE_NAP		0x00000004
#define KGSL_STATE_SLEEP	0x00000008
#define KGSL_STATE_SUSPEND	0x00000010
#define KGSL_STATE_AWARE	0x00000020
#define KGSL_STATE_SLUMBER	0x00000080
#define KGSL_STATE_DEEP_NAP	0x00000100

enum kgsl_event_results {
	KGSL_EVENT_RETIRED = 1,
	KGSL_EVENT_CANCELLED = 2,
};

#define KGSL_FLAG_WAKE_ON_TOUCH BIT(0)


#define KGSL_EVENT_TYPES \
	{ KGSL_EVENT_RETIRED, "retired" }, \
	{ KGSL_EVENT_CANCELLED, "cancelled" }

#define KGSL_CONTEXT_FLAGS \
	{ KGSL_CONTEXT_NO_GMEM_ALLOC , "NO_GMEM_ALLOC" }, \
	{ KGSL_CONTEXT_PREAMBLE, "PREAMBLE" }, \
	{ KGSL_CONTEXT_TRASH_STATE, "TRASH_STATE" }, \
	{ KGSL_CONTEXT_CTX_SWITCH, "CTX_SWITCH" }, \
	{ KGSL_CONTEXT_PER_CONTEXT_TS, "PER_CONTEXT_TS" }, \
	{ KGSL_CONTEXT_USER_GENERATED_TS, "USER_TS" }, \
	{ KGSL_CONTEXT_NO_FAULT_TOLERANCE, "NO_FT" }, \
	{ KGSL_CONTEXT_PWR_CONSTRAINT, "PWR" }, \
	{ KGSL_CONTEXT_SAVE_GMEM, "SAVE_GMEM" }

#define KGSL_CONTEXT_TYPES \
	{ KGSL_CONTEXT_TYPE_ANY, "ANY" }, \
	{ KGSL_CONTEXT_TYPE_GL, "GL" }, \
	{ KGSL_CONTEXT_TYPE_CL, "CL" }, \
	{ KGSL_CONTEXT_TYPE_C2D, "C2D" }, \
	{ KGSL_CONTEXT_TYPE_RS, "RS" }

#define KGSL_CONTEXT_ID(_context) \
	((_context != NULL) ? (_context)->id : KGSL_MEMSTORE_GLOBAL)

#define KGSL_SNAPSHOT_MEMSIZE (600 * 1024)

struct kgsl_device;
struct platform_device;
struct kgsl_device_private;
struct kgsl_context;
struct kgsl_power_stats;
struct kgsl_event;
struct kgsl_snapshot;

struct kgsl_functable {
	void (*regread) (struct kgsl_device *device,
		unsigned int offsetwords, unsigned int *value);
	void (*regwrite) (struct kgsl_device *device,
		unsigned int offsetwords, unsigned int value);
	int (*idle) (struct kgsl_device *device);
	bool (*isidle) (struct kgsl_device *device);
	int (*suspend_context) (struct kgsl_device *device);
	int (*init) (struct kgsl_device *device);
	int (*start) (struct kgsl_device *device, int priority);
	int (*stop) (struct kgsl_device *device);
	int (*getproperty) (struct kgsl_device *device,
		unsigned int type, void __user *value,
		size_t sizebytes);
	int (*getproperty_compat) (struct kgsl_device *device,
		unsigned int type, void __user *value,
		size_t sizebytes);
	int (*waittimestamp) (struct kgsl_device *device,
		struct kgsl_context *context, unsigned int timestamp,
		unsigned int msecs);
	int (*readtimestamp) (struct kgsl_device *device, void *priv,
		enum kgsl_timestamp_type type, unsigned int *timestamp);
	int (*issueibcmds) (struct kgsl_device_private *dev_priv,
		struct kgsl_context *context, struct kgsl_cmdbatch *cmdbatch,
		uint32_t *timestamps);
	void (*power_stats)(struct kgsl_device *device,
		struct kgsl_power_stats *stats);
	unsigned int (*gpuid)(struct kgsl_device *device, unsigned int *chipid);
	void (*snapshot)(struct kgsl_device *device,
		struct kgsl_snapshot *snapshot, struct kgsl_context *context);
	irqreturn_t (*irq_handler)(struct kgsl_device *device);
	int (*drain)(struct kgsl_device *device);
	struct kgsl_context *(*drawctxt_create) (struct kgsl_device_private *,
						uint32_t *flags);
	void (*drawctxt_detach)(struct kgsl_context *context);
	void (*drawctxt_destroy) (struct kgsl_context *context);
	void (*drawctxt_dump) (struct kgsl_device *device,
		struct kgsl_context *context);
	long (*ioctl) (struct kgsl_device_private *dev_priv,
		unsigned int cmd, unsigned long arg);
	long (*compat_ioctl) (struct kgsl_device_private *dev_priv,
		unsigned int cmd, unsigned long arg);
	int (*setproperty) (struct kgsl_device_private *dev_priv,
		unsigned int type, void __user *value,
		unsigned int sizebytes);
	int (*setproperty_compat) (struct kgsl_device_private *dev_priv,
		unsigned int type, void __user *value,
		unsigned int sizebytes);
	void (*drawctxt_sched)(struct kgsl_device *device,
		struct kgsl_context *context);
	void (*resume)(struct kgsl_device *device);
	int (*regulator_enable)(struct kgsl_device *);
	bool (*is_hw_collapsible)(struct kgsl_device *);
	void (*regulator_disable)(struct kgsl_device *);
	void (*pwrlevel_change_settings)(struct kgsl_device *device,
		unsigned int prelevel, unsigned int postlevel, bool post);
	void (*regulator_disable_poll)(struct kgsl_device *device);
};

struct kgsl_ioctl {
	unsigned int cmd;
	long (*func)(struct kgsl_device_private *, unsigned int, void *);
};

long kgsl_ioctl_helper(struct file *filep, unsigned int cmd, unsigned long arg,
		const struct kgsl_ioctl *cmds, int len);

#define MEMOBJ_PREAMBLE BIT(0)
#define MEMOBJ_SKIP BIT(1)

struct kgsl_memobj_node {
	struct list_head node;
	unsigned int id;
	uint64_t offset;
	uint64_t gpuaddr;
	uint64_t size;
	unsigned long flags;
	unsigned long priv;
};

struct kgsl_device {
	struct device *dev;
	const char *name;
	unsigned int ver_major;
	unsigned int ver_minor;
	uint32_t flags;
	enum kgsl_deviceid id;

	
	unsigned long reg_phys;

	
	void __iomem *reg_virt;

	
	unsigned int reg_len;

	
	void __iomem *shader_mem_virt;

	
	unsigned long shader_mem_phys;

	
	unsigned int shader_mem_len;
	struct kgsl_memdesc memstore;
	struct kgsl_memdesc scratch;
	const char *iomemname;
	const char *shadermemname;

	struct kgsl_mmu mmu;
	struct completion hwaccess_gate;
	struct completion cmdbatch_gate;
	const struct kgsl_functable *ftbl;
	struct work_struct idle_check_ws;
	struct timer_list idle_timer;
	struct kgsl_pwrctrl pwrctrl;
	int open_count;

	struct mutex mutex;
	uint32_t state;
	uint32_t requested_state;

	atomic_t active_cnt;

	wait_queue_head_t wait_queue;
	wait_queue_head_t active_cnt_wq;
	struct platform_device *pdev;
	struct dentry *d_debugfs;
	struct idr context_idr;
	rwlock_t context_lock;

	struct {
		void *ptr;
		size_t size;
	} snapshot_memory;

	struct kgsl_snapshot *snapshot;

	u32 snapshot_faultcount;	
	bool force_panic;		
	struct kobject snapshot_kobj;

	struct kobject ppd_kobj;

	
	int cmd_log;
	int ctxt_log;
	int drv_log;
	int mem_log;
	int pwr_log;
	struct kgsl_pwrscale pwrscale;

	int reset_counter; 
	int cff_dump_enable;
	struct workqueue_struct *events_wq;

	struct device *busmondev; 

	
	int active_context_count;

	
	int gpu_fault_no_panic;
};

#define KGSL_MMU_DEVICE(_mmu) \
	container_of((_mmu), struct kgsl_device, mmu)

#define KGSL_DEVICE_COMMON_INIT(_dev) \
	.hwaccess_gate = COMPLETION_INITIALIZER((_dev).hwaccess_gate),\
	.cmdbatch_gate = COMPLETION_INITIALIZER((_dev).cmdbatch_gate),\
	.idle_check_ws = __WORK_INITIALIZER((_dev).idle_check_ws,\
			kgsl_idle_check),\
	.context_idr = IDR_INIT((_dev).context_idr),\
	.wait_queue = __WAIT_QUEUE_HEAD_INITIALIZER((_dev).wait_queue),\
	.active_cnt_wq = __WAIT_QUEUE_HEAD_INITIALIZER((_dev).active_cnt_wq),\
	.mutex = __MUTEX_INITIALIZER((_dev).mutex),\
	.state = KGSL_STATE_NONE,\
	.ver_major = DRIVER_VERSION_MAJOR,\
	.ver_minor = DRIVER_VERSION_MINOR


enum kgsl_context_priv {
	KGSL_CONTEXT_PRIV_DETACHED = 0,
	KGSL_CONTEXT_PRIV_INVALID,
	KGSL_CONTEXT_PRIV_PAGEFAULT,
	KGSL_CONTEXT_PRIV_DEVICE_SPECIFIC = 16,
};

struct kgsl_process_private;

struct kgsl_context {
	struct kref refcount;
	uint32_t id;
	uint32_t priority;
	pid_t tid;
	struct kgsl_device_private *dev_priv;
	struct kgsl_process_private *proc_priv;
	unsigned long priv;
	struct kgsl_device *device;
	unsigned int reset_status;
	struct sync_timeline *timeline;
	struct kgsl_event_group events;
	unsigned int flags;
	struct kgsl_pwr_constraint pwr_constraint;
	unsigned int fault_count;
	unsigned long fault_time;
};

#define _context_comm(_c) \
	(((_c) && (_c)->proc_priv) ? (_c)->proc_priv->comm : "unknown")


#define pr_context(_d, _c, fmt, args...) \
		dev_err((_d)->dev, "%s[%d]: " fmt, \
		_context_comm((_c)), \
		(_c)->proc_priv->pid, ##args)

struct kgsl_process_private {
	unsigned long priv;
	pid_t pid;
	char comm[TASK_COMM_LEN];
	spinlock_t mem_lock;
	struct kref refcount;
	struct idr mem_idr;
	struct kgsl_pagetable *pagetable;
	struct list_head list;
	struct kobject kobj;
	struct dentry *debug_root;
	struct {
		atomic_long_t cur;
		uint64_t max;
	} stats[KGSL_MEM_ENTRY_MAX];
	struct idr syncsource_idr;
	spinlock_t syncsource_lock;
	int fd_count;
};

enum kgsl_process_priv_flags {
	KGSL_PROCESS_INIT = 0,
};

struct kgsl_device_private {
	struct kgsl_device *device;
	struct kgsl_process_private *process_priv;
};

struct kgsl_snapshot {
	uint64_t ib1base;
	uint64_t ib2base;
	unsigned int ib1size;
	unsigned int ib2size;
	bool ib1dumped;
	bool ib2dumped;
	u8 *start;
	size_t size;
	u8 *ptr;
	size_t remain;
	unsigned long timestamp;
	u8 *mempool;
	size_t mempool_size;
	struct list_head obj_list;
	struct list_head cp_list;
	struct work_struct work;
	struct completion dump_gate;
	struct kgsl_process_private *process;
	atomic_t sysfs_read;
};

struct kgsl_snapshot_object {
	uint64_t gpuaddr;
	uint64_t size;
	uint64_t offset;
	int type;
	struct kgsl_mem_entry *entry;
	struct list_head node;
};

struct kgsl_device *kgsl_get_device(int dev_idx);

static inline void kgsl_process_add_stats(struct kgsl_process_private *priv,
	unsigned int type, uint64_t size)
{
	uint64_t cur;
	if (type >= KGSL_MEM_ENTRY_MAX)
		return;

	spin_lock(&priv->mem_lock);
	atomic_long_add(size, &priv->stats[type].cur);
	cur = atomic_long_read(&priv->stats[type].cur);
	if (priv->stats[type].max < cur)
		priv->stats[type].max = cur;
	spin_unlock(&priv->mem_lock);
}

static inline void kgsl_process_sub_stats(struct kgsl_process_private *priv,
		unsigned int type, size_t size)
{
	atomic_long_sub(size, &priv->stats[type].cur);
}

static inline void kgsl_regread(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int *value)
{
	device->ftbl->regread(device, offsetwords, value);
}

static inline void kgsl_regwrite(struct kgsl_device *device,
				 unsigned int offsetwords,
				 unsigned int value)
{
	device->ftbl->regwrite(device, offsetwords, value);
}

static inline void kgsl_regrmw(struct kgsl_device *device,
		unsigned int offsetwords,
		unsigned int mask, unsigned int bits)
{
	unsigned int val = 0;

	device->ftbl->regread(device, offsetwords, &val);
	val &= ~mask;
	device->ftbl->regwrite(device, offsetwords, val | bits);
}

static inline int kgsl_idle(struct kgsl_device *device)
{
	return device->ftbl->idle(device);
}

static inline unsigned int kgsl_gpuid(struct kgsl_device *device,
	unsigned int *chipid)
{
	return device->ftbl->gpuid(device, chipid);
}

static inline int kgsl_create_device_sysfs_files(struct device *root,
	const struct device_attribute **list)
{
	int ret = 0, i;
	for (i = 0; list[i] != NULL; i++)
		ret |= device_create_file(root, list[i]);
	return ret;
}

static inline void kgsl_remove_device_sysfs_files(struct device *root,
	const struct device_attribute **list)
{
	int i;
	for (i = 0; list[i] != NULL; i++)
		device_remove_file(root, list[i]);
}

static inline struct kgsl_device *kgsl_device_from_dev(struct device *dev)
{
	int i;

	for (i = 0; i < KGSL_DEVICE_MAX; i++) {
		if (kgsl_driver.devp[i] && kgsl_driver.devp[i]->dev == dev)
			return kgsl_driver.devp[i];
	}

	return NULL;
}

static inline int kgsl_state_is_awake(struct kgsl_device *device)
{
	if (device->state == KGSL_STATE_ACTIVE ||
		device->state == KGSL_STATE_AWARE)
		return true;
	else
		return false;
}

int kgsl_readtimestamp(struct kgsl_device *device, void *priv,
		enum kgsl_timestamp_type type, unsigned int *timestamp);

int kgsl_check_timestamp(struct kgsl_device *device,
		struct kgsl_context *context, unsigned int timestamp);

int kgsl_device_platform_probe(struct kgsl_device *device);

void kgsl_device_platform_remove(struct kgsl_device *device);

const char *kgsl_pwrstate_to_str(unsigned int state);

int kgsl_device_snapshot_init(struct kgsl_device *device);
void kgsl_device_snapshot(struct kgsl_device *device,
			struct kgsl_context *context);
void kgsl_device_snapshot_close(struct kgsl_device *device);
void kgsl_snapshot_save_frozen_objs(struct work_struct *work);

void kgsl_events_init(void);
void kgsl_events_exit(void);

void kgsl_del_event_group(struct kgsl_event_group *group);

void kgsl_add_event_group(struct kgsl_event_group *group,
		struct kgsl_context *context, const char *name,
		readtimestamp_func readtimestamp, void *priv);

void kgsl_cancel_events_timestamp(struct kgsl_device *device,
		struct kgsl_event_group *group, unsigned int timestamp);
void kgsl_cancel_events(struct kgsl_device *device,
		struct kgsl_event_group *group);
void kgsl_cancel_event(struct kgsl_device *device,
		struct kgsl_event_group *group, unsigned int timestamp,
		kgsl_event_func func, void *priv);
bool kgsl_event_pending(struct kgsl_device *device,
		struct kgsl_event_group *group, unsigned int timestamp,
		kgsl_event_func func, void *priv);
int kgsl_add_event(struct kgsl_device *device, struct kgsl_event_group *group,
		unsigned int timestamp, kgsl_event_func func, void *priv);
void kgsl_process_event_group(struct kgsl_device *device,
	struct kgsl_event_group *group);
void kgsl_flush_event_group(struct kgsl_device *device,
		struct kgsl_event_group *group);
void kgsl_process_event_groups(struct kgsl_device *device);

void kgsl_context_destroy(struct kref *kref);

int kgsl_context_init(struct kgsl_device_private *, struct kgsl_context
		*context);

void kgsl_context_dump(struct kgsl_context *context);

int kgsl_memfree_find_entry(pid_t ptname, uint64_t *gpuaddr,
	uint64_t *size, uint64_t *flags, pid_t *pid);

long kgsl_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);

long kgsl_ioctl_copy_in(unsigned int kernel_cmd, unsigned int user_cmd,
		unsigned long arg, unsigned char *ptr);

long kgsl_ioctl_copy_out(unsigned int kernel_cmd, unsigned int user_cmd,
		unsigned long, unsigned char *ptr);

static inline void
kgsl_context_put(struct kgsl_context *context)
{
	if (context)
		kref_put(&context->refcount, kgsl_context_destroy);
}

static inline bool kgsl_context_detached(struct kgsl_context *context)
{
	return (context == NULL || test_bit(KGSL_CONTEXT_PRIV_DETACHED,
						&context->priv));
}

static inline bool kgsl_context_invalid(struct kgsl_context *context)
{
	return (context == NULL || test_bit(KGSL_CONTEXT_PRIV_INVALID,
						&context->priv));
}


static inline struct kgsl_context *kgsl_context_get(struct kgsl_device *device,
		uint32_t id)
{
	int result = 0;
	struct kgsl_context *context = NULL;

	read_lock(&device->context_lock);

	context = idr_find(&device->context_idr, id);

	
	if (kgsl_context_detached(context))
		context = NULL;
	else
		result = kref_get_unless_zero(&context->refcount);

	read_unlock(&device->context_lock);

	if (!result)
		return NULL;
	return context;
}

static inline int _kgsl_context_get(struct kgsl_context *context)
{
	int ret = 0;

	if (context)
		ret = kref_get_unless_zero(&context->refcount);

	return ret;
}

static inline struct kgsl_context *kgsl_context_get_owner(
		struct kgsl_device_private *dev_priv, uint32_t id)
{
	struct kgsl_context *context;

	context = kgsl_context_get(dev_priv->device, id);

	
	if (context != NULL && context->dev_priv != dev_priv) {
		kgsl_context_put(context);
		return NULL;
	}

	return context;
}

static inline int kgsl_process_private_get(struct kgsl_process_private *process)
{
	int ret = 0;
	if (process != NULL)
		ret = kref_get_unless_zero(&process->refcount);
	return ret;
}

void kgsl_process_private_put(struct kgsl_process_private *private);


struct kgsl_process_private *kgsl_process_private_find(pid_t pid);

static inline int kgsl_property_read_u32(struct kgsl_device *device,
	const char *prop, unsigned int *ptr)
{
	return of_property_read_u32(device->pdev->dev.of_node, prop, ptr);
}

static inline int kgsl_sysfs_store(const char *buf, unsigned int *ptr)
{
	unsigned int val;
	int rc;

	rc = kstrtou32(buf, 0, &val);
	if (rc)
		return rc;

	if (ptr)
		*ptr = val;

	return 0;
}

#define SNAPSHOT_ERR_NOMEM(_d, _s) \
	KGSL_DRV_ERR((_d), \
	"snapshot: not enough snapshot memory for section %s\n", (_s))

struct kgsl_snapshot_registers {
	const unsigned int *regs;
	unsigned int count;
};

size_t kgsl_snapshot_dump_registers(struct kgsl_device *device, u8 *buf,
		size_t remain, void *priv);

void kgsl_snapshot_indexed_registers(struct kgsl_device *device,
	struct kgsl_snapshot *snapshot, unsigned int index,
	unsigned int data, unsigned int start, unsigned int count);

int kgsl_snapshot_get_object(struct kgsl_snapshot *snapshot,
	struct kgsl_process_private *process, uint64_t gpuaddr,
	uint64_t size, unsigned int type);

int kgsl_snapshot_have_object(struct kgsl_snapshot *snapshot,
	struct kgsl_process_private *process,
	uint64_t gpuaddr, uint64_t size);

struct adreno_ib_object_list;

int kgsl_snapshot_add_ib_obj_list(struct kgsl_snapshot *snapshot,
	struct adreno_ib_object_list *ib_obj_list);

void kgsl_snapshot_add_section(struct kgsl_device *device, u16 id,
	struct kgsl_snapshot *snapshot,
	size_t (*func)(struct kgsl_device *, u8 *, size_t, void *),
	void *priv);

struct kgsl_pwr_limit {
	struct list_head node;
	unsigned int level;
	struct kgsl_device *device;
};

#endif  
