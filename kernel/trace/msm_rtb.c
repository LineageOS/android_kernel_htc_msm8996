/*
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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

#include <linux/atomic.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <asm-generic/sizes.h>
#include <linux/msm_rtb.h>

#define SENTINEL_BYTE_1 0xFF
#define SENTINEL_BYTE_2 0xAA
#define SENTINEL_BYTE_3 0xFF

#define RTB_COMPAT_STR	"qcom,msm-rtb"

#ifdef CONFIG_HTC_EARLY_RTB
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>

int early_rtb_stat = EARLY_RTB_INIT;
#endif

struct msm_rtb_layout {
	unsigned char sentinel[3];
	unsigned char log_type;
	uint32_t idx;
	uint64_t caller;
	uint64_t data;
	uint64_t timestamp;
} __attribute__ ((__packed__));


struct msm_rtb_state {
	struct msm_rtb_layout *rtb;
	phys_addr_t phys;
	int nentries;
	int size;
	int enabled;
	int initialized;
	uint32_t filter;
	int step_size;
};

#if defined(CONFIG_MSM_RTB_SEPARATE_CPUS)
DEFINE_PER_CPU(atomic_t, msm_rtb_idx_cpu);
#else
static atomic_t msm_rtb_idx;
#endif

static struct msm_rtb_state msm_rtb = {
#if defined(CONFIG_HTC_DEBUG_RTB)
	
	.filter = (1 << LOGK_READL)|(1 << LOGK_WRITEL)|(1 << LOGK_LOGBUF)|(1 << LOGK_HOTPLUG)|(1 << LOGK_CTXID)|(1 << LOGK_IRQ)|(1 << LOGK_DIE)|(1 << LOGK_INITCALL)|(1 << LOGK_SOFTIRQ),
#else
	.filter = 1 << LOGK_LOGBUF,
#endif
	.enabled = 1,
};

module_param_named(filter, msm_rtb.filter, uint, 0644);
module_param_named(enable, msm_rtb.enabled, int, 0644);

#if defined(CONFIG_HTC_DEBUG_RTB)
void msm_rtb_disable(void)
{
	msm_rtb.enabled = 0;
	return;
}
EXPORT_SYMBOL(msm_rtb_disable);
#endif 

static int msm_rtb_panic_notifier(struct notifier_block *this,
					unsigned long event, void *ptr)
{
	msm_rtb.enabled = 0;
	return NOTIFY_DONE;
}

static struct notifier_block msm_rtb_panic_blk = {
	.notifier_call  = msm_rtb_panic_notifier,
	.priority = INT_MAX,
};

int notrace msm_rtb_event_should_log(enum logk_event_type log_type)
{
#ifdef CONFIG_HTC_EARLY_RTB
	return ((early_rtb_stat == EARLY_RTB_RUNNING) || msm_rtb.initialized) &&
		msm_rtb.enabled && ((1 << (log_type & ~LOGTYPE_NOPC)) & msm_rtb.filter);
#else
	return msm_rtb.initialized && msm_rtb.enabled &&
		((1 << (log_type & ~LOGTYPE_NOPC)) & msm_rtb.filter);
#endif
}
EXPORT_SYMBOL(msm_rtb_event_should_log);

static void msm_rtb_emit_sentinel(struct msm_rtb_layout *start)
{
	start->sentinel[0] = SENTINEL_BYTE_1;
	start->sentinel[1] = SENTINEL_BYTE_2;
	start->sentinel[2] = SENTINEL_BYTE_3;
}

static void msm_rtb_write_type(enum logk_event_type log_type,
			struct msm_rtb_layout *start)
{
	start->log_type = (char)log_type;
}

static void msm_rtb_write_caller(uint64_t caller, struct msm_rtb_layout *start)
{
	start->caller = caller;
}

static void msm_rtb_write_idx(uint32_t idx,
				struct msm_rtb_layout *start)
{
	start->idx = idx;
}

static void msm_rtb_write_data(uint64_t data, struct msm_rtb_layout *start)
{
	start->data = data;
}

static void msm_rtb_write_timestamp(struct msm_rtb_layout *start)
{
	start->timestamp = sched_clock();
}

static void uncached_logk_pc_idx(enum logk_event_type log_type, uint64_t caller,
				 uint64_t data, int idx)
{
	struct msm_rtb_layout *start;

	start = &msm_rtb.rtb[idx & (msm_rtb.nentries - 1)];

	msm_rtb_emit_sentinel(start);
	msm_rtb_write_type(log_type, start);
	msm_rtb_write_caller(caller, start);
	msm_rtb_write_idx(idx, start);
	msm_rtb_write_data(data, start);
	msm_rtb_write_timestamp(start);
	mb();

	return;
}

static void uncached_logk_timestamp(int idx)
{
	unsigned long long timestamp;

	timestamp = sched_clock();
	uncached_logk_pc_idx(LOGK_TIMESTAMP|LOGTYPE_NOPC,
			(uint64_t)lower_32_bits(timestamp),
			(uint64_t)upper_32_bits(timestamp), idx);
}

#if defined(CONFIG_MSM_RTB_SEPARATE_CPUS)
static int msm_rtb_get_idx(void)
{
	int cpu, i, offset;
	atomic_t *index;

	cpu = raw_smp_processor_id();

	index = &per_cpu(msm_rtb_idx_cpu, cpu);

	i = atomic_add_return(msm_rtb.step_size, index);
	i -= msm_rtb.step_size;

	
	offset = (i & (msm_rtb.nentries - 1)) -
		 ((i - msm_rtb.step_size) & (msm_rtb.nentries - 1));
	if (offset < 0) {
		uncached_logk_timestamp(i);
		i = atomic_add_return(msm_rtb.step_size, index);
		i -= msm_rtb.step_size;
	}

	return i;
}
#else
static int msm_rtb_get_idx(void)
{
	int i, offset;

	i = atomic_inc_return(&msm_rtb_idx);
	i--;

	
	offset = (i & (msm_rtb.nentries - 1)) -
		 ((i - 1) & (msm_rtb.nentries - 1));
	if (offset < 0) {
		uncached_logk_timestamp(i);
		i = atomic_inc_return(&msm_rtb_idx);
		i--;
	}

	return i;
}
#endif

int notrace uncached_logk_pc(enum logk_event_type log_type, void *caller,
				void *data)
{
	int i;

	if (!msm_rtb_event_should_log(log_type))
		return 0;

	i = msm_rtb_get_idx();
	uncached_logk_pc_idx(log_type, (uint64_t)((unsigned long) caller),
				(uint64_t)((unsigned long) data), i);

	return 1;
}
EXPORT_SYMBOL(uncached_logk_pc);

noinline int notrace uncached_logk(enum logk_event_type log_type, void *data)
{
	return uncached_logk_pc(log_type, __builtin_return_address(0), data);
}
EXPORT_SYMBOL(uncached_logk);

#ifdef CONFIG_HTC_EARLY_RTB
int htc_early_rtb_init(void)
{
#if defined(CONFIG_MSM_RTB_SEPARATE_CPUS)
        unsigned int cpu;
#endif
	struct device_node *dt_node = NULL;
	char* rtb_node_name = "qcom,msm-rtb";
	char* rtb_resource_name = "msm_rtb_res";
	struct resource r;
	int i = 0, of_ret = 0;

	if(msm_rtb.initialized == 1) {
		pr_err("RTB already initialized, quit early rtb!!\n");
		early_rtb_stat = EARLY_RTB_ERROR;
		return 1;
	}

	if(early_rtb_stat != EARLY_RTB_INIT) {
		pr_err("early rtb not in initial state : %u !!\n", early_rtb_stat);
		early_rtb_stat = EARLY_RTB_ERROR;
		return 1;
	}

	
	dt_node = of_find_node_by_name(NULL, rtb_node_name);

	if(dt_node == NULL) {
		pr_err("RTB node node <%s> not found in DTB!!\n", rtb_node_name);
		early_rtb_stat = EARLY_RTB_ERROR;
		return 1;
	}

	for(i = 0 ; (of_ret = of_address_to_resource(dt_node, i, &r)) == 0 ; i++) {
		if(!strcmp(rtb_resource_name, r.name))
			break;
	}

	if(of_ret) {
		printk("early rtb : couldn't found resource \"%s\" in node %s\n", rtb_resource_name, rtb_node_name);
		early_rtb_stat = EARLY_RTB_ERROR;
		return 1;
	}
	

	msm_rtb.size = resource_size(&r);
	msm_rtb.phys = r.start;

	pr_info("early rtb size: 0x%x\n", (unsigned int)msm_rtb.size);
	pr_info("early rtb phys: 0x%x\n", (unsigned int)msm_rtb.phys);

	if((msm_rtb.phys == 0) || (msm_rtb.size == 0)) {
		pr_err("early rtb with error phys/size\n");
		early_rtb_stat = EARLY_RTB_ERROR;
		return 1;
	}

	msm_rtb.rtb = ioremap(msm_rtb.phys, msm_rtb.size);

	if (!msm_rtb.rtb) {
		pr_err("early rtb ioremap fail!!\n");
		early_rtb_stat = EARLY_RTB_ERROR;
		return -ENOMEM;
	}

	msm_rtb.nentries = msm_rtb.size / sizeof(struct msm_rtb_layout);
	msm_rtb.nentries = __rounddown_pow_of_two(msm_rtb.nentries);
	memset_io(msm_rtb.rtb, 0, msm_rtb.size);

#if defined(CONFIG_MSM_RTB_SEPARATE_CPUS)
        for_each_possible_cpu(cpu) {
                atomic_t *a = &per_cpu(msm_rtb_idx_cpu, cpu);
                atomic_set(a, cpu);
        }
        msm_rtb.step_size = num_possible_cpus();
#else
        atomic_set(&msm_rtb_idx, 0);
        msm_rtb.step_size = 1;
#endif

	early_rtb_stat = EARLY_RTB_RUNNING;
	smp_mb();

	return 0;
}

int htc_early_rtb_deinit(void)
{
	uint32_t backup_filter = msm_rtb.filter;
	int backup_enabled = msm_rtb.enabled;

	if(early_rtb_stat == EARLY_RTB_RUNNING) {
		
		
		
		early_rtb_stat = EARLY_RTB_STOP;
		smp_mb();
		mdelay(10);

		iounmap(msm_rtb.rtb);
		memset_io(&msm_rtb, 0, sizeof(struct msm_rtb_state));

		msm_rtb.filter = backup_filter;
		msm_rtb.enabled = backup_enabled;
		pr_info("early rtb deinitialized\n");

		return 0;
	}

	WARN(1, "Early-RTB deinit called with state : %u\n", early_rtb_stat);

	return 1;
}
#endif

static int msm_rtb_probe(struct platform_device *pdev)
{
	struct msm_rtb_platform_data *d = pdev->dev.platform_data;
	struct resource *res = NULL;
#if defined(CONFIG_MSM_RTB_SEPARATE_CPUS)
	unsigned int cpu;
#endif
	int ret;

#ifdef CONFIG_HTC_EARLY_RTB
	htc_early_rtb_deinit();
#endif

	if (!pdev->dev.of_node) {
		if (!d) {
			return -EINVAL;
		}
		msm_rtb.size = d->size;
	} else {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "msm_rtb_res");
		if (res) {
			msm_rtb.size = resource_size(res);
		} else {
			u64 size;
			struct device_node *pnode;

			pnode = of_parse_phandle(pdev->dev.of_node,
							"linux,contiguous-region", 0);
			if (pnode != NULL) {
				const u32 *addr;

				addr = of_get_address(pnode, 0, &size, NULL);
				if (!addr) {
					of_node_put(pnode);
					return -EINVAL;
				}
				of_node_put(pnode);
			} else {
				ret = of_property_read_u32(pdev->dev.of_node,
						"qcom,rtb-size",
						(u32 *)&size);
				if (ret < 0)
					return ret;
			}
			msm_rtb.size = size;
		}
	}
	pr_info("msm_rtb.size: 0x%x\n", msm_rtb.size);

	if (msm_rtb.size <= 0 || msm_rtb.size > SZ_1M)
		return -EINVAL;

	if (res) {
		msm_rtb.phys = res->start;
		msm_rtb.rtb = ioremap(msm_rtb.phys, msm_rtb.size);
	} else {
		msm_rtb.rtb = dma_alloc_coherent(&pdev->dev, msm_rtb.size, &msm_rtb.phys, GFP_KERNEL);
	}

	pr_info("msm_rtb set ok: phys: 0x%llx, size: 0x%x\n", msm_rtb.phys, msm_rtb.size);

	if (!msm_rtb.rtb)
		return -ENOMEM;

	msm_rtb.nentries = msm_rtb.size / sizeof(struct msm_rtb_layout);

	
	msm_rtb.nentries = __rounddown_pow_of_two(msm_rtb.nentries);

	memset_io(msm_rtb.rtb, 0, msm_rtb.size);


#if defined(CONFIG_MSM_RTB_SEPARATE_CPUS)
	for_each_possible_cpu(cpu) {
		atomic_t *a = &per_cpu(msm_rtb_idx_cpu, cpu);
		atomic_set(a, cpu);
	}
	msm_rtb.step_size = num_possible_cpus();
#else
	atomic_set(&msm_rtb_idx, 0);
	msm_rtb.step_size = 1;
#endif

	atomic_notifier_chain_register(&panic_notifier_list,
						&msm_rtb_panic_blk);
	msm_rtb.initialized = 1;
	return 0;
}

static struct of_device_id msm_match_table[] = {
	{.compatible = RTB_COMPAT_STR},
	{},
};

static struct platform_driver msm_rtb_driver = {
	.driver         = {
		.name = "msm_rtb",
		.owner = THIS_MODULE,
		.of_match_table = msm_match_table
	},
};

static int __init msm_rtb_init(void)
{
	return platform_driver_probe(&msm_rtb_driver, msm_rtb_probe);
}

static void __exit msm_rtb_exit(void)
{
	platform_driver_unregister(&msm_rtb_driver);
}
module_init(msm_rtb_init)
module_exit(msm_rtb_exit)
