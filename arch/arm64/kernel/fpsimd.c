/*
 * FP/SIMD context switching and fault handling
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Catalin Marinas <catalin.marinas@arm.com>
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

#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/hardirq.h>

#include <asm/fpsimd.h>
#include <asm/cputype.h>
#include <asm/app_api.h>

#define FPEXC_IOF	(1 << 0)
#define FPEXC_DZF	(1 << 1)
#define FPEXC_OFF	(1 << 2)
#define FPEXC_UFF	(1 << 3)
#define FPEXC_IXF	(1 << 4)
#define FPEXC_IDF	(1 << 7)

#define FP_SIMD_BIT	31

static DEFINE_PER_CPU(struct fpsimd_state *, fpsimd_last_state);
static DEFINE_PER_CPU(int, fpsimd_stg_enable);

static int fpsimd_settings = 0x1; 
module_param(fpsimd_settings, int, 0644);

void fpsimd_settings_enable(void)
{
	set_app_setting_bit(FP_SIMD_BIT);
}

void fpsimd_settings_disable(void)
{
	clear_app_setting_bit(FP_SIMD_BIT);
}

void do_fpsimd_acc(unsigned int esr, struct pt_regs *regs)
{
	if (!fpsimd_settings)
		return;

	fpsimd_disable_trap();
	fpsimd_settings_disable();
	this_cpu_write(fpsimd_stg_enable, 0);
}

void do_fpsimd_acc_compat(unsigned int esr, struct pt_regs *regs)
{
	if (!fpsimd_settings)
		return;

	fpsimd_disable_trap();
	fpsimd_settings_enable();
	this_cpu_write(fpsimd_stg_enable, 1);
}

void do_fpsimd_exc(unsigned int esr, struct pt_regs *regs)
{
	siginfo_t info;
	unsigned int si_code = 0;

	if (esr & FPEXC_IOF)
		si_code = FPE_FLTINV;
	else if (esr & FPEXC_DZF)
		si_code = FPE_FLTDIV;
	else if (esr & FPEXC_OFF)
		si_code = FPE_FLTOVF;
	else if (esr & FPEXC_UFF)
		si_code = FPE_FLTUND;
	else if (esr & FPEXC_IXF)
		si_code = FPE_FLTRES;

	memset(&info, 0, sizeof(info));
	info.si_signo = SIGFPE;
	info.si_code = si_code;
	info.si_addr = (void __user *)instruction_pointer(regs);

	send_sig_info(SIGFPE, &info, current);
}

void fpsimd_thread_switch(struct task_struct *next)
{
	if (current->mm && !test_thread_flag(TIF_FOREIGN_FPSTATE))
		fpsimd_save_state(&current->thread.fpsimd_state);

	if (fpsimd_settings && __this_cpu_read(fpsimd_stg_enable)) {
		fpsimd_settings_disable();
		this_cpu_write(fpsimd_stg_enable, 0);
	}

	if (next->mm) {
		struct fpsimd_state *st = &next->thread.fpsimd_state;

		if (__this_cpu_read(fpsimd_last_state) == st
		    && st->cpu == smp_processor_id())
			clear_ti_thread_flag(task_thread_info(next),
					     TIF_FOREIGN_FPSTATE);
		else
			set_ti_thread_flag(task_thread_info(next),
					   TIF_FOREIGN_FPSTATE);

		if (!fpsimd_settings)
			return;

		if (test_ti_thread_flag(task_thread_info(next), TIF_32BIT))
			fpsimd_enable_trap();
		else
			fpsimd_disable_trap();
	}
}

void fpsimd_flush_thread(void)
{
	memset(&current->thread.fpsimd_state, 0, sizeof(struct fpsimd_state));
	set_thread_flag(TIF_FOREIGN_FPSTATE);
}

void fpsimd_preserve_current_state(void)
{
	preempt_disable();
	if (!test_thread_flag(TIF_FOREIGN_FPSTATE))
		fpsimd_save_state(&current->thread.fpsimd_state);
	preempt_enable();
}

void fpsimd_restore_current_state(void)
{
	preempt_disable();
	if (test_and_clear_thread_flag(TIF_FOREIGN_FPSTATE)) {
		struct fpsimd_state *st = &current->thread.fpsimd_state;

		fpsimd_load_state(st);
		this_cpu_write(fpsimd_last_state, st);
		st->cpu = smp_processor_id();
	}
	preempt_enable();
}

void fpsimd_update_current_state(struct fpsimd_state *state)
{
	preempt_disable();
	fpsimd_load_state(state);
	if (test_and_clear_thread_flag(TIF_FOREIGN_FPSTATE)) {
		struct fpsimd_state *st = &current->thread.fpsimd_state;

		this_cpu_write(fpsimd_last_state, st);
		st->cpu = smp_processor_id();
	}
	preempt_enable();
}

void fpsimd_flush_task_state(struct task_struct *t)
{
	t->thread.fpsimd_state.cpu = NR_CPUS;
}

#ifdef CONFIG_KERNEL_MODE_NEON

static DEFINE_PER_CPU(struct fpsimd_partial_state, hardirq_fpsimdstate);
static DEFINE_PER_CPU(struct fpsimd_partial_state, softirq_fpsimdstate);

void kernel_neon_begin_partial(u32 num_regs)
{
	if (in_interrupt()) {
		struct fpsimd_partial_state *s = this_cpu_ptr(
			in_irq() ? &hardirq_fpsimdstate : &softirq_fpsimdstate);

		BUG_ON(num_regs > 32);
		fpsimd_save_partial_state(s, roundup(num_regs, 2));
	} else {
		preempt_disable();
		if (current->mm &&
		    !test_and_set_thread_flag(TIF_FOREIGN_FPSTATE))
			fpsimd_save_state(&current->thread.fpsimd_state);
		this_cpu_write(fpsimd_last_state, NULL);
	}
}
EXPORT_SYMBOL(kernel_neon_begin_partial);

void kernel_neon_end(void)
{
	if (in_interrupt()) {
		struct fpsimd_partial_state *s = this_cpu_ptr(
			in_irq() ? &hardirq_fpsimdstate : &softirq_fpsimdstate);
		fpsimd_load_partial_state(s);
	} else {
		preempt_enable();
	}
}
EXPORT_SYMBOL(kernel_neon_end);

#endif 

#ifdef CONFIG_CPU_PM
static int fpsimd_cpu_pm_notifier(struct notifier_block *self,
				  unsigned long cmd, void *v)
{
	switch (cmd) {
	case CPU_PM_ENTER:
		if (current->mm && !test_thread_flag(TIF_FOREIGN_FPSTATE))
			fpsimd_save_state(&current->thread.fpsimd_state);
		this_cpu_write(fpsimd_last_state, NULL);
		break;
	case CPU_PM_EXIT:
		if (current->mm)
			set_thread_flag(TIF_FOREIGN_FPSTATE);
		break;
	case CPU_PM_ENTER_FAILED:
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static struct notifier_block fpsimd_cpu_pm_notifier_block = {
	.notifier_call = fpsimd_cpu_pm_notifier,
};

static void fpsimd_pm_init(void)
{
	cpu_pm_register_notifier(&fpsimd_cpu_pm_notifier_block);
}

#else
static inline void fpsimd_pm_init(void) { }
#endif 

#ifdef CONFIG_HOTPLUG_CPU
static int fpsimd_cpu_hotplug_notifier(struct notifier_block *nfb,
				       unsigned long action,
				       void *hcpu)
{
	unsigned int cpu = (long)hcpu;

	switch (action) {
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		per_cpu(fpsimd_last_state, cpu) = NULL;
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block fpsimd_cpu_hotplug_notifier_block = {
	.notifier_call = fpsimd_cpu_hotplug_notifier,
};

static inline void fpsimd_hotplug_init(void)
{
	register_cpu_notifier(&fpsimd_cpu_hotplug_notifier_block);
}

#else
static inline void fpsimd_hotplug_init(void) { }
#endif

static int __init fpsimd_init(void)
{
	u64 pfr = read_cpuid(ID_AA64PFR0_EL1);

	if (pfr & (0xf << 16)) {
		pr_notice("Floating-point is not implemented\n");
		return 0;
	}
	elf_hwcap |= HWCAP_FP;

	if (pfr & (0xf << 20))
		pr_notice("Advanced SIMD is not implemented\n");
	else
		elf_hwcap |= HWCAP_ASIMD;

	fpsimd_pm_init();
	fpsimd_hotplug_init();

	return 0;
}
late_initcall(fpsimd_init);
