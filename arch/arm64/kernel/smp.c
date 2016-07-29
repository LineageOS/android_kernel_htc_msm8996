/*
 * SMP initialisation and IPI support
 * Based on arch/arm/kernel/smp.c
 *
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/cache.h>
#include <linux/profile.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/percpu.h>
#include <linux/clockchips.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/irq_work.h>

#include <asm/alternative.h>
#include <asm/atomic.h>
#include <asm/cacheflush.h>
#include <asm/cpu.h>
#include <asm/cputype.h>
#include <asm/cpu_ops.h>
#include <asm/mmu_context.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/processor.h>
#include <asm/smp_plat.h>
#include <asm/sections.h>
#include <asm/tlbflush.h>
#include <asm/ptrace.h>
#include <asm/edac.h>

#define CREATE_TRACE_POINTS
#include <trace/events/ipi.h>

struct secondary_data secondary_data;
volatile unsigned long secondary_holding_pen_release = INVALID_HWID;

enum ipi_msg_type {
	IPI_RESCHEDULE,
	IPI_CALL_FUNC,
	IPI_CALL_FUNC_SINGLE,
	IPI_CPU_STOP,
	IPI_TIMER,
	IPI_IRQ_WORK,
	IPI_WAKEUP,
	IPI_CPU_BACKTRACE,
};

static int boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	if (cpu_ops[cpu]->cpu_boot)
		return cpu_ops[cpu]->cpu_boot(cpu);

	return -EOPNOTSUPP;
}

static DECLARE_COMPLETION(cpu_running);

int __cpu_up(unsigned int cpu, struct task_struct *idle)
{
	int ret;

	secondary_data.stack = task_stack_page(idle) + THREAD_START_SP;
	__flush_dcache_area(&secondary_data, sizeof(secondary_data));

	ret = boot_secondary(cpu, idle);
	if (ret == 0) {
		wait_for_completion_timeout(&cpu_running,
					    msecs_to_jiffies(1000));

		if (!cpu_online(cpu)) {
			pr_crit("CPU%u: failed to come online\n", cpu);
			ret = -EIO;
		}
	} else {
		pr_err("CPU%u: failed to boot: %d\n", cpu, ret);
	}

	secondary_data.stack = NULL;

	return ret;
}

static void smp_store_cpu_info(unsigned int cpuid)
{
	store_cpu_topology(cpuid);
}

asmlinkage void secondary_start_kernel(void)
{
	struct mm_struct *mm = &init_mm;
	unsigned int cpu = smp_processor_id();

	atomic_inc(&mm->mm_count);
	current->active_mm = mm;
	cpumask_set_cpu(cpu, mm_cpumask(mm));

	set_my_cpu_offset(per_cpu_offset(smp_processor_id()));
	pr_debug("CPU%u: Booted secondary processor\n", cpu);

	cpu_set_reserved_ttbr0();
	flush_tlb_all();

	preempt_disable();
	trace_hardirqs_off();

	if (cpu_ops[cpu]->cpu_postboot)
		cpu_ops[cpu]->cpu_postboot();

	cpuinfo_store_cpu();

	smp_store_cpu_info(cpu);

	notify_cpu_starting(cpu);

	set_cpu_online(cpu, true);
	complete(&cpu_running);

	local_dbg_enable();
	local_irq_enable();
	local_async_enable();

	cpu_startup_entry(CPUHP_ONLINE);
}

#ifdef CONFIG_HOTPLUG_CPU
static int op_cpu_disable(unsigned int cpu)
{
	if (!cpu_ops[cpu] || !cpu_ops[cpu]->cpu_die)
		return -EOPNOTSUPP;

	if (cpu_ops[cpu]->cpu_disable)
		return cpu_ops[cpu]->cpu_disable(cpu);

	return 0;
}

int __cpu_disable(void)
{
	unsigned int cpu = smp_processor_id();
	int ret;

	ret = op_cpu_disable(cpu);
	if (ret)
		return ret;

	set_cpu_online(cpu, false);

	migrate_irqs();

	clear_tasks_mm_cpumask(cpu);

	return 0;
}

static int op_cpu_kill(unsigned int cpu)
{
	if (!cpu_ops[cpu]->cpu_kill)
		return 1;

	return cpu_ops[cpu]->cpu_kill(cpu);
}

static DECLARE_COMPLETION(cpu_died);

void __cpu_die(unsigned int cpu)
{
	if (!wait_for_completion_timeout(&cpu_died, msecs_to_jiffies(5000))) {
		pr_crit("CPU%u: cpu didn't die\n", cpu);
		return;
	}
	pr_debug("CPU%u: shutdown\n", cpu);

	if (!op_cpu_kill(cpu))
		pr_warn("CPU%d may not have shut down cleanly\n", cpu);
}

void __ref cpu_die(void)
{
	unsigned int cpu = smp_processor_id();

	idle_task_exit();

	local_irq_disable();

	
	complete(&cpu_died);

	cpu_ops[cpu]->cpu_die(cpu);


	asm volatile("mov       sp, %0\n"
		     "mov       x29, #0\n"
		     "b         secondary_start_kernel"
		     : : "r" (task_stack_page(current) + THREAD_START_SP));
}
#endif

void __init smp_cpus_done(unsigned int max_cpus)
{
	pr_info("SMP: Total of %d processors activated.\n", num_online_cpus());
	apply_alternatives();
}

void __init smp_prepare_boot_cpu(void)
{
	set_my_cpu_offset(per_cpu_offset(smp_processor_id()));
}

void (*__smp_cross_call)(const struct cpumask *, unsigned int);
DEFINE_PER_CPU(bool, pending_ipi);

void smp_cross_call_common(const struct cpumask *cpumask, unsigned int func)
{
	unsigned int cpu;

	for_each_cpu(cpu, cpumask)
		per_cpu(pending_ipi, cpu) = true;

	__smp_cross_call(cpumask, func);
}

void __init smp_init_cpus(void)
{
	struct device_node *dn = NULL;
	unsigned int i, cpu = 1;
	bool bootcpu_valid = false;

	while ((dn = of_find_node_by_type(dn, "cpu"))) {
		const u32 *cell;
		u64 hwid;

		cell = of_get_property(dn, "reg", NULL);
		if (!cell) {
			pr_err("%s: missing reg property\n", dn->full_name);
			goto next;
		}
		hwid = of_read_number(cell, of_n_addr_cells(dn));

		if (hwid & ~MPIDR_HWID_BITMASK) {
			pr_err("%s: invalid reg property\n", dn->full_name);
			goto next;
		}

		for (i = 1; (i < cpu) && (i < NR_CPUS); i++) {
			if (cpu_logical_map(i) == hwid) {
				pr_err("%s: duplicate cpu reg properties in the DT\n",
					dn->full_name);
				goto next;
			}
		}

		if (hwid == cpu_logical_map(0)) {
			if (bootcpu_valid) {
				pr_err("%s: duplicate boot cpu reg property in DT\n",
					dn->full_name);
				goto next;
			}

			bootcpu_valid = true;

			continue;
		}

		if (cpu >= NR_CPUS)
			goto next;

		if (cpu_read_ops(dn, cpu) != 0)
			goto next;

		if (cpu_ops[cpu]->cpu_init(dn, cpu))
			goto next;

		pr_debug("cpu logical map 0x%llx\n", hwid);
		cpu_logical_map(cpu) = hwid;
next:
		cpu++;
	}

	
	if (cpu > NR_CPUS)
		pr_warning("no. of cores (%d) greater than configured maximum of %d - clipping\n",
			   cpu, NR_CPUS);

	if (!bootcpu_valid) {
		pr_err("DT missing boot CPU MPIDR, not enabling secondaries\n");
		return;
	}

	for (i = 0; i < NR_CPUS; i++)
		if (cpu_logical_map(i) != INVALID_HWID)
			set_cpu_possible(i, true);
}

void __init smp_prepare_cpus(unsigned int max_cpus)
{
	int err;
	unsigned int cpu, ncores = num_possible_cpus();

	init_cpu_topology();

	smp_store_cpu_info(smp_processor_id());

	if (max_cpus > ncores)
		max_cpus = ncores;

	
	if (max_cpus <= 1)
		return;

	max_cpus--;
	for_each_possible_cpu(cpu) {
		if (max_cpus == 0)
			break;

		if (cpu == smp_processor_id())
			continue;

		if (!cpu_ops[cpu])
			continue;

		err = cpu_ops[cpu]->cpu_prepare(cpu);
		if (err)
			continue;

		set_cpu_present(cpu, true);
		max_cpus--;
	}
}

void __init set_smp_cross_call(void (*fn)(const struct cpumask *, unsigned int))
{
	__smp_cross_call = fn;
}

void arch_send_call_function_ipi_mask(const struct cpumask *mask)
{
	smp_cross_call_common(mask, IPI_CALL_FUNC);
}

void arch_send_call_function_single_ipi(int cpu)
{
	smp_cross_call_common(cpumask_of(cpu), IPI_CALL_FUNC_SINGLE);
}

void arch_send_wakeup_ipi_mask(const struct cpumask *mask)
{
	smp_cross_call_common(mask, IPI_WAKEUP);
}

static const char *ipi_types[NR_IPI] __tracepoint_string = {
#define S(x,s)	[x] = s
	S(IPI_RESCHEDULE, "Rescheduling interrupts"),
	S(IPI_CALL_FUNC, "Function call interrupts"),
	S(IPI_CALL_FUNC_SINGLE, "Single function call interrupts"),
	S(IPI_CPU_STOP, "CPU stop interrupts"),
	S(IPI_TIMER, "Timer broadcast interrupts"),
	S(IPI_IRQ_WORK, "IRQ work interrupts"),
	S(IPI_WAKEUP, "CPU wakeup interrupts"),
	S(IPI_CPU_BACKTRACE, "CPU backtrace"),
};

static void smp_cross_call(const struct cpumask *target, unsigned int ipinr)
{
	trace_ipi_raise(target, ipi_types[ipinr]);
	__smp_cross_call(target, ipinr);
}

void show_ipi_list(struct seq_file *p, int prec)
{
	unsigned int cpu, i;

	for (i = 0; i < NR_IPI; i++) {
		seq_printf(p, "%*s%u:%s", prec - 1, "IPI", i,
			   prec >= 4 ? " " : "");
		for_each_online_cpu(cpu)
			seq_printf(p, "%10u ",
				   __get_irq_stat(cpu, ipi_irqs[i]));
		seq_printf(p, "      %s\n", ipi_types[i]);
	}
}

u64 smp_irq_stat_cpu(unsigned int cpu)
{
	u64 sum = 0;
	int i;

	for (i = 0; i < NR_IPI; i++)
		sum += __get_irq_stat(cpu, ipi_irqs[i]);

	return sum;
}

#ifdef CONFIG_IRQ_WORK
void arch_irq_work_raise(void)
{
	if (__smp_cross_call)
		smp_cross_call(cpumask_of(smp_processor_id()), IPI_IRQ_WORK);
}
#endif

static DEFINE_RAW_SPINLOCK(stop_lock);

DEFINE_PER_CPU(struct pt_regs, regs_before_stop);

static void ipi_cpu_stop(unsigned int cpu, struct pt_regs *regs)
{
	if (system_state == SYSTEM_BOOTING ||
	    system_state == SYSTEM_RUNNING) {
		per_cpu(regs_before_stop, cpu) = *regs;
		raw_spin_lock(&stop_lock);
		pr_crit("CPU%u: stopping\n", cpu);
		show_regs(regs);
		dump_stack();
		arm64_check_cache_ecc(NULL);
		raw_spin_unlock(&stop_lock);
	}

	set_cpu_active(cpu, false);

	flush_cache_all();
	local_irq_disable();

	while (1)
		cpu_relax();
}

static cpumask_t backtrace_mask;
static DEFINE_RAW_SPINLOCK(backtrace_lock);

static unsigned long backtrace_flag;

#if defined(CONFIG_HTC_DEBUG_WATCHDOG)
static void smp_send_all_cpu_backtrace(unsigned int backtrace_timeout)
#else
static void smp_send_all_cpu_backtrace(void)
#endif
{
	unsigned int this_cpu = smp_processor_id();
	int i;

	if (test_and_set_bit(0, &backtrace_flag))
		return;

	cpumask_copy(&backtrace_mask, cpu_online_mask);
	cpu_clear(this_cpu, backtrace_mask);

	pr_info("Backtrace for cpu %d (current):\n", this_cpu);
	dump_stack();

	pr_info("\nsending IPI to all other CPUs:\n");
	if (!cpus_empty(backtrace_mask))
		smp_cross_call_common(&backtrace_mask, IPI_CPU_BACKTRACE);

	
#if defined(CONFIG_HTC_DEBUG_WATCHDOG)
	for (i = 0; i < backtrace_timeout * 1000; i++)
#else
	for (i = 0; i < 10 * 1000; i++)
#endif
	{
		if (cpumask_empty(&backtrace_mask))
			break;
		mdelay(1);
	}

#if defined(CONFIG_HTC_DEBUG_WATCHDOG)
	if(i == backtrace_timeout * 1000)
		pr_info( " dump cpu backtrace timeout \n");
#endif

	clear_bit(0, &backtrace_flag);
	smp_mb__after_atomic();
}

static void ipi_cpu_backtrace(unsigned int cpu, struct pt_regs *regs)
{
	if (cpu_isset(cpu, backtrace_mask)) {
		raw_spin_lock(&backtrace_lock);
		pr_warn("IPI backtrace for cpu %d\n", cpu);
		show_regs(regs);
		raw_spin_unlock(&backtrace_lock);
		cpu_clear(cpu, backtrace_mask);
	}
}

#ifdef CONFIG_SMP
void arch_trigger_all_cpu_backtrace(void)
{
#if defined(CONFIG_HTC_DEBUG_WATCHDOG)
	smp_send_all_cpu_backtrace(10);
#else
	smp_send_all_cpu_backtrace();
#endif
}
#else
void arch_trigger_all_cpu_backtrace(void)
{
	dump_stack();
}
#endif

#if defined(CONFIG_HTC_DEBUG_WATCHDOG)
void arch_trigger_different_cpu_backtrace_dump_timeout(unsigned int time_out)
{
	pr_info(" dump cpu backtrace with timeout %u sec \n", time_out);
	smp_send_all_cpu_backtrace(time_out);
}
#endif

void handle_IPI(int ipinr, struct pt_regs *regs)
{
	unsigned int cpu = smp_processor_id();
	struct pt_regs *old_regs = set_irq_regs(regs);

	if ((unsigned)ipinr < NR_IPI) {
		trace_ipi_entry(ipi_types[ipinr]);
		__inc_irq_stat(cpu, ipi_irqs[ipinr]);
	}

	switch (ipinr) {
	case IPI_RESCHEDULE:
		scheduler_ipi();
		break;

	case IPI_CALL_FUNC:
		irq_enter();
		generic_smp_call_function_interrupt();
		irq_exit();
		break;

	case IPI_CALL_FUNC_SINGLE:
		irq_enter();
		generic_smp_call_function_single_interrupt();
		irq_exit();
		break;

	case IPI_CPU_STOP:
		irq_enter();
		ipi_cpu_stop(cpu, regs);
		irq_exit();
		break;

#ifdef CONFIG_GENERIC_CLOCKEVENTS_BROADCAST
	case IPI_TIMER:
		irq_enter();
		tick_receive_broadcast();
		irq_exit();
		break;
#endif
	case IPI_WAKEUP:
		break;

#ifdef CONFIG_IRQ_WORK
	case IPI_IRQ_WORK:
		irq_enter();
		irq_work_run();
		irq_exit();
		break;
#endif
	case IPI_CPU_BACKTRACE:
		ipi_cpu_backtrace(cpu, regs);
		break;

	default:
		pr_crit("CPU%u: Unknown IPI message 0x%x\n", cpu, ipinr);
		break;
	}

	if ((unsigned)ipinr < NR_IPI)
		trace_ipi_exit(ipi_types[ipinr]);
	per_cpu(pending_ipi, cpu) = false;
	set_irq_regs(old_regs);
}

void smp_send_reschedule(int cpu)
{
	BUG_ON(cpu_is_offline(cpu));
	smp_cross_call_common(cpumask_of(cpu), IPI_RESCHEDULE);
}

#ifdef CONFIG_GENERIC_CLOCKEVENTS_BROADCAST
void tick_broadcast(const struct cpumask *mask)
{
	smp_cross_call_common(mask, IPI_TIMER);
}
#endif

void smp_send_stop(void)
{
	unsigned long timeout;

	if (num_online_cpus() > 1) {
		cpumask_t mask;

		cpumask_copy(&mask, cpu_online_mask);
		cpu_clear(smp_processor_id(), mask);

		smp_cross_call_common(&mask, IPI_CPU_STOP);
	}

	
	timeout = USEC_PER_SEC;
	while (num_active_cpus() > 1 && timeout--)
		udelay(1);

	if (num_active_cpus() > 1)
		pr_warning("SMP: failed to stop secondary CPUs\n");
}

int setup_profiling_timer(unsigned int multiplier)
{
	return -EINVAL;
}
