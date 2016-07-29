#include <linux/irq_work.h>
#include <linux/rcupdate.h>
#include <linux/rculist.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/percpu.h>
#include <linux/init.h>
#include <linux/gfp.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/sched.h>

#include "smpboot.h"

#include <linux/htc_flags.h>

enum {
	CSD_FLAG_LOCK		= 0x01,
	CSD_FLAG_WAIT		= 0x02,
};

struct call_function_data {
	struct call_single_data	__percpu *csd;
	cpumask_var_t		cpumask;
};

static DEFINE_PER_CPU_SHARED_ALIGNED(struct call_function_data, cfd_data);

static DEFINE_PER_CPU_SHARED_ALIGNED(struct llist_head, call_single_queue);

static void flush_smp_call_function_queue(bool warn_cpu_offline);
static bool have_boot_cpu_mask;
static cpumask_var_t boot_cpu_mask;
int have_cpu_mask;
struct cpumask cpu_mask;

static int
hotplug_cfd(struct notifier_block *nfb, unsigned long action, void *hcpu)
{
	long cpu = (long)hcpu;
	struct call_function_data *cfd = &per_cpu(cfd_data, cpu);

	switch (action) {
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		if (!zalloc_cpumask_var_node(&cfd->cpumask, GFP_KERNEL,
				cpu_to_node(cpu)))
			return notifier_from_errno(-ENOMEM);
		cfd->csd = alloc_percpu(struct call_single_data);
		if (!cfd->csd) {
			free_cpumask_var(cfd->cpumask);
			return notifier_from_errno(-ENOMEM);
		}
		break;

#ifdef CONFIG_HOTPLUG_CPU
	case CPU_UP_CANCELED:
	case CPU_UP_CANCELED_FROZEN:
		

	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		free_cpumask_var(cfd->cpumask);
		free_percpu(cfd->csd);
		break;

	case CPU_DYING:
	case CPU_DYING_FROZEN:
		flush_smp_call_function_queue(false);
		break;
#endif
	};

	return NOTIFY_OK;
}

static struct notifier_block hotplug_cfd_notifier = {
	.notifier_call		= hotplug_cfd,
};

void __init call_function_init(void)
{
	void *cpu = (void *)(long)smp_processor_id();
	int i;

	for_each_possible_cpu(i)
		init_llist_head(&per_cpu(call_single_queue, i));

	hotplug_cfd(&hotplug_cfd_notifier, CPU_UP_PREPARE, cpu);
	register_cpu_notifier(&hotplug_cfd_notifier);
}

static void csd_lock_wait(struct call_single_data *csd)
{
	while (csd->flags & CSD_FLAG_LOCK)
		cpu_relax();
}

static void csd_lock(struct call_single_data *csd)
{
	csd_lock_wait(csd);
	csd->flags |= CSD_FLAG_LOCK;

	smp_mb();
}

static void csd_unlock(struct call_single_data *csd)
{
	WARN_ON((csd->flags & CSD_FLAG_WAIT) && !(csd->flags & CSD_FLAG_LOCK));

	smp_mb();

	csd->flags &= ~CSD_FLAG_LOCK;
}

static DEFINE_PER_CPU_SHARED_ALIGNED(struct call_single_data, csd_data);

static int generic_exec_single(int cpu, struct call_single_data *csd,
			       smp_call_func_t func, void *info, int wait)
{
	struct call_single_data csd_stack = { .flags = 0 };
	unsigned long flags;


	if (cpu == smp_processor_id()) {
		local_irq_save(flags);
		func(info);
		local_irq_restore(flags);
		return 0;
	}


	if ((unsigned)cpu >= nr_cpu_ids || !cpu_online(cpu))
		return -ENXIO;


	if (!csd) {
		csd = &csd_stack;
		if (!wait)
			csd = this_cpu_ptr(&csd_data);
	}

	csd_lock(csd);

	csd->func = func;
	csd->info = info;

	if (wait)
		csd->flags |= CSD_FLAG_WAIT;

	if (llist_add(&csd->llist, &per_cpu(call_single_queue, cpu)))
		arch_send_call_function_single_ipi(cpu);

	if (wait)
		csd_lock_wait(csd);

	return 0;
}

void generic_smp_call_function_single_interrupt(void)
{
	flush_smp_call_function_queue(true);
}

static void flush_smp_call_function_queue(bool warn_cpu_offline)
{
	struct llist_head *head;
	struct llist_node *entry;
	struct call_single_data *csd, *csd_next;
	static bool warned;

	WARN_ON(!irqs_disabled());

	head = this_cpu_ptr(&call_single_queue);
	entry = llist_del_all(head);
	entry = llist_reverse_order(entry);

	
	if (unlikely(warn_cpu_offline && !cpu_online(smp_processor_id()) &&
		     !warned && !llist_empty(head))) {
		warned = true;
		WARN(1, "IPI on offline CPU %d\n", smp_processor_id());

		llist_for_each_entry(csd, entry, llist)
			pr_warn("IPI callback %pS sent to offline CPU\n",
				csd->func);
	}

	llist_for_each_entry_safe(csd, csd_next, entry, llist) {
		csd->func(csd->info);
		csd_unlock(csd);
	}

	irq_work_run();
}

int smp_call_function_single(int cpu, smp_call_func_t func, void *info,
			     int wait)
{
	int this_cpu;
	int err;

	this_cpu = get_cpu();

	WARN_ON_ONCE(cpu_online(this_cpu) && irqs_disabled()
		     && !oops_in_progress);

	err = generic_exec_single(cpu, NULL, func, info, wait);

	put_cpu();

	return err;
}
EXPORT_SYMBOL(smp_call_function_single);

int smp_call_function_single_async(int cpu, struct call_single_data *csd)
{
	int err = 0;

	preempt_disable();
	err = generic_exec_single(cpu, csd, csd->func, csd->info, 0);
	preempt_enable();

	return err;
}
EXPORT_SYMBOL_GPL(smp_call_function_single_async);

int smp_call_function_any(const struct cpumask *mask,
			  smp_call_func_t func, void *info, int wait)
{
	unsigned int cpu;
	const struct cpumask *nodemask;
	int ret;

	
	cpu = get_cpu();
	if (cpumask_test_cpu(cpu, mask))
		goto call;

	
	nodemask = cpumask_of_node(cpu_to_node(cpu));
	for (cpu = cpumask_first_and(nodemask, mask); cpu < nr_cpu_ids;
	     cpu = cpumask_next_and(cpu, nodemask, mask)) {
		if (cpu_online(cpu))
			goto call;
	}

	
	cpu = cpumask_any_and(mask, cpu_online_mask);
call:
	ret = smp_call_function_single(cpu, func, info, wait);
	put_cpu();
	return ret;
}
EXPORT_SYMBOL_GPL(smp_call_function_any);

void smp_call_function_many(const struct cpumask *mask,
			    smp_call_func_t func, void *info, bool wait)
{
	struct call_function_data *cfd;
	int cpu, next_cpu, this_cpu = smp_processor_id();

	WARN_ON_ONCE(cpu_online(this_cpu) && irqs_disabled()
		     && !oops_in_progress && !early_boot_irqs_disabled);

	
	cpu = cpumask_first_and(mask, cpu_online_mask);
	if (cpu == this_cpu)
		cpu = cpumask_next_and(cpu, mask, cpu_online_mask);

	
	if (cpu >= nr_cpu_ids)
		return;

	
	next_cpu = cpumask_next_and(cpu, mask, cpu_online_mask);
	if (next_cpu == this_cpu)
		next_cpu = cpumask_next_and(next_cpu, mask, cpu_online_mask);

	
	if (next_cpu >= nr_cpu_ids) {
		smp_call_function_single(cpu, func, info, wait);
		return;
	}

	cfd = this_cpu_ptr(&cfd_data);

	cpumask_and(cfd->cpumask, mask, cpu_online_mask);
	cpumask_clear_cpu(this_cpu, cfd->cpumask);

	
	if (unlikely(!cpumask_weight(cfd->cpumask)))
		return;

	for_each_cpu(cpu, cfd->cpumask) {
		struct call_single_data *csd = per_cpu_ptr(cfd->csd, cpu);

		csd_lock(csd);
		csd->func = func;
		csd->info = info;
		llist_add(&csd->llist, &per_cpu(call_single_queue, cpu));
	}

	
	arch_send_call_function_ipi_mask(cfd->cpumask);

	if (wait) {
		for_each_cpu(cpu, cfd->cpumask) {
			struct call_single_data *csd;

			csd = per_cpu_ptr(cfd->csd, cpu);
			csd_lock_wait(csd);
		}
	}
}
EXPORT_SYMBOL(smp_call_function_many);

int smp_call_function(smp_call_func_t func, void *info, int wait)
{
	preempt_disable();
	smp_call_function_many(cpu_online_mask, func, info, wait);
	preempt_enable();

	return 0;
}
EXPORT_SYMBOL(smp_call_function);

unsigned int setup_max_cpus = NR_CPUS;
EXPORT_SYMBOL(setup_max_cpus);



void __weak arch_disable_smp_support(void) { }

static int __init nosmp(char *str)
{
	setup_max_cpus = 0;
	arch_disable_smp_support();

	return 0;
}

early_param("nosmp", nosmp);

static int __init nrcpus(char *str)
{
	int nr_cpus;

	get_option(&str, &nr_cpus);
	if (nr_cpus > 0 && nr_cpus < nr_cpu_ids)
		nr_cpu_ids = nr_cpus;

	return 0;
}

early_param("nr_cpus", nrcpus);

static int __init maxcpus(char *str)
{
	get_option(&str, &setup_max_cpus);
	if (setup_max_cpus == 0)
		arch_disable_smp_support();

	return 0;
}

early_param("maxcpus", maxcpus);

static int __init boot_cpus(char *str)
{
	alloc_bootmem_cpumask_var(&boot_cpu_mask);
	if (cpulist_parse(str, boot_cpu_mask) < 0) {
		pr_warn("SMP: Incorrect boot_cpus cpumask\n");
		return -EINVAL;
	}
	have_boot_cpu_mask = true;
	return 0;
}

early_param("boot_cpus", boot_cpus);

int nr_cpu_ids __read_mostly = NR_CPUS;
EXPORT_SYMBOL(nr_cpu_ids);

void __init setup_nr_cpu_ids(void)
{
	nr_cpu_ids = find_last_bit(cpumask_bits(cpu_possible_mask),NR_CPUS) + 1;
}

void __weak smp_announce(void)
{
	printk(KERN_INFO "Brought up %d CPUs\n", num_online_cpus());
}

static inline bool boot_cpu(int cpu)
{
	if (!have_boot_cpu_mask)
		return true;

	return cpumask_test_cpu(cpu, boot_cpu_mask);
}

static inline void free_boot_cpu_mask(void)
{
	if (have_boot_cpu_mask)	
		free_bootmem_cpumask_var(boot_cpu_mask);
}

void __init smp_init(void)
{
	unsigned int cpu;
	int mask;

	mask = get_cpumask_flag();
	if (mask) {
		struct cpumask dest;

		*cpu_mask.bits = (long)mask;
		if (!cpumask_test_cpu(0, &cpu_mask) && !cpumask_andnot(&dest, &cpu_mask, cpu_possible_mask))
			have_cpu_mask = 1;
		else
			printk(KERN_ERR "cpumask error : 0x%X\n", mask);
	}

	idle_threads_init();

	
	for_each_present_cpu(cpu) {
		if (num_online_cpus() >= setup_max_cpus)
			break;
		if (!cpu_online(cpu) && boot_cpu(cpu))
			cpu_up(cpu);
	}

	free_boot_cpu_mask();

	
	smp_announce();
	smp_cpus_done(setup_max_cpus);
}

int on_each_cpu(void (*func) (void *info), void *info, int wait)
{
	unsigned long flags;
	int ret = 0;

	preempt_disable();
	ret = smp_call_function(func, info, wait);
	local_irq_save(flags);
	func(info);
	local_irq_restore(flags);
	preempt_enable();
	return ret;
}
EXPORT_SYMBOL(on_each_cpu);

void on_each_cpu_mask(const struct cpumask *mask, smp_call_func_t func,
			void *info, bool wait)
{
	int cpu = get_cpu();

	smp_call_function_many(mask, func, info, wait);
	if (cpumask_test_cpu(cpu, mask)) {
		unsigned long flags;
		local_irq_save(flags);
		func(info);
		local_irq_restore(flags);
	}
	put_cpu();
}
EXPORT_SYMBOL(on_each_cpu_mask);

void on_each_cpu_cond(bool (*cond_func)(int cpu, void *info),
			smp_call_func_t func, void *info, bool wait,
			gfp_t gfp_flags)
{
	cpumask_var_t cpus;
	int cpu, ret;

	might_sleep_if(gfp_flags & __GFP_WAIT);

	if (likely(zalloc_cpumask_var(&cpus, (gfp_flags|__GFP_NOWARN)))) {
		preempt_disable();
		for_each_online_cpu(cpu)
			if (cond_func(cpu, info))
				cpumask_set_cpu(cpu, cpus);
		on_each_cpu_mask(cpus, func, info, wait);
		preempt_enable();
		free_cpumask_var(cpus);
	} else {
		preempt_disable();
		for_each_online_cpu(cpu)
			if (cond_func(cpu, info)) {
				ret = smp_call_function_single(cpu, func,
								info, wait);
				WARN_ON_ONCE(ret);
			}
		preempt_enable();
	}
}
EXPORT_SYMBOL(on_each_cpu_cond);

static void do_nothing(void *unused)
{
}

void kick_all_cpus_sync(void)
{
	
	smp_mb();
	smp_call_function(do_nothing, NULL, 1);
}
EXPORT_SYMBOL_GPL(kick_all_cpus_sync);

void wake_up_all_idle_cpus(void)
{
	int cpu;

	preempt_disable();
	for_each_online_cpu(cpu) {
		if (cpu == smp_processor_id())
			continue;

		wake_up_if_idle(cpu);
	}
	preempt_enable();
}
EXPORT_SYMBOL_GPL(wake_up_all_idle_cpus);
