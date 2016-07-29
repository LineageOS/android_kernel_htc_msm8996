#include <linux/module.h>
#include <linux/debugfs.h>

#include "kgsl_htc.h"
#include "adreno.h"

static int gpu_fault_no_panic_set(void *data, u64 val)
{
	struct kgsl_device *device = data;
	device->gpu_fault_no_panic = val;
	printk("kgsl: %s: gpu_fault_no_panic = %d\n",
		__FUNCTION__, device->gpu_fault_no_panic);
	return 0;
}

static int gpu_fault_no_panic_get(void *data, u64 *val)
{
	struct kgsl_device *device = data;
	*val = device->gpu_fault_no_panic;
	printk("kgsl: %s: gpu_fault_no_panic = %d\n",
		__FUNCTION__, device->gpu_fault_no_panic);
	return 0;
}

static int ctx_dump_set(void* data, u64 val)
{
	struct kgsl_device *device = data;

	read_lock(&device->context_lock);
	kgsl_dump_contextpid_locked(&device->context_idr);
	read_unlock(&device->context_lock);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ctx_dump_fops,
				NULL,
				ctx_dump_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(gpu_fault_no_panic_fops,
				gpu_fault_no_panic_get,
				gpu_fault_no_panic_set, "%lld\n");

unsigned int kgsl_get_alloc_size(int detailed)
{
	struct kgsl_driver_htc_priv *priv = &kgsl_driver.priv;

	if (detailed && time_after(jiffies, priv->next_jiffies)) {
		priv->next_jiffies = jiffies + 20*HZ;
		schedule_work(&kgsl_driver.priv.work);
	}

	return atomic_long_read(&kgsl_driver.stats.page_alloc);
}

static void do_print_mem_detail(struct work_struct *work)
{
	struct kgsl_driver_htc_priv *priv = container_of(work,
			struct kgsl_driver_htc_priv, work);
	struct kgsl_driver *driver = container_of(priv,
			struct kgsl_driver, priv);
	struct kgsl_process_private *private;

	printk("kgsl: kgsl_driver.stats.page_alloc = %ld\n", atomic_long_read(&driver->stats.page_alloc));
	printk("kgsl: kgsl_driver.stats.page_alloc_kernel = %ld\n", atomic_long_read(&driver->stats.vmalloc));

	mutex_lock(&driver->process_mutex);
	list_for_each_entry(private, &driver->process_list, list) {
		if (!private)
			continue;

		if (atomic_long_read(&private->stats[KGSL_MEM_ENTRY_PAGE_ALLOC].cur) != 0)
			printk("kgsl: proc %5d alloc page %8ld bytes\n", private->pid,
							atomic_long_read(&private->stats[KGSL_MEM_ENTRY_PAGE_ALLOC].cur));
	}
	mutex_unlock(&driver->process_mutex);
}

int kgsl_driver_htc_init(struct kgsl_driver_htc_priv *priv)
{
	priv->next_jiffies = jiffies;
	INIT_WORK(&priv->work, do_print_mem_detail);
	return 0;
}

int kgsl_device_htc_init(struct kgsl_device *device)
{
	device->gpu_fault_no_panic = CONFIG_MSM_KGSL_DEFAULT_GPU_HUNG_NO_PANIC;

	if (!device->d_debugfs || IS_ERR(device->d_debugfs))
		return 1;
	debugfs_create_file("contexpid_dump",  0644, device->d_debugfs, device,
					&ctx_dump_fops);
	debugfs_create_file("gpu_fault_no_panic",  0644, device->d_debugfs, device,
                    &gpu_fault_no_panic_fops);
	return 0;
}

void kgsl_dump_contextpid_locked(struct idr *context_idr)
{
	int i = 0;
	struct kgsl_context *context;
	struct task_struct *task;
	struct task_struct *parent_task;
	char task_name[TASK_COMM_LEN+1];
	char task_parent_name[TASK_COMM_LEN+1];
	pid_t ppid;

	printk(" == [KGSL] context maximal count is %lu, dump context id, pid, name, group leader name ==\n",KGSL_MEMSTORE_MAX);
	for (i = 0; i <KGSL_MEMSTORE_MAX; i++) {

		context = idr_find(context_idr, i);

		if (context  && context->dev_priv &&  context->dev_priv->process_priv) {
			task = find_task_by_pid_ns(context->dev_priv->process_priv->pid, &init_pid_ns);
			if (!task) {
				printk("can't find context's task: context id %d\n", context->id);
				continue;
			}
			parent_task = task->group_leader;
			get_task_comm(task_name, task);

			if (parent_task) {
				get_task_comm(task_parent_name, parent_task);
				ppid = context->dev_priv->process_priv->pid;
			} else {
				task_parent_name[0] = '\0';
				ppid = 0;
			}

			printk("context id=%d\t\t pid=%d\t\t %s\t\t %s\n", context->id, ppid, task_name, task_parent_name);
		}
	}
}

void adreno_fault_panic(struct kgsl_device *device, unsigned int pid, int fault) {

	char fault_string[512];

	if (device->gpu_fault_no_panic > 0)
		device->gpu_fault_no_panic--;

	if(device->gpu_fault_no_panic)
		return;

	snprintf(fault_string, sizeof(fault_string), "Recoverable GPU Hang (0x%x)", fault);
	msleep(10000);
	panic(fault_string);
}
