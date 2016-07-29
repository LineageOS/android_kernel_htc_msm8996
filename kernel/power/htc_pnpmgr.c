/* linux/kernel/power/htc_pnpmgr.c
 *
 * Copyright (C) 2012 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/string.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/htc_pnpmgr.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/htc_fda.h>

#include "power.h"

#define MAX_BUF     100
#define MAX_VALUE   999999999    
#define MAX_CPU_NUM 8

enum {
	BC_TYPE = 0,
	LC_TYPE,
	MAX_TYPE,
};

#define DEBUG_DUMP 0

static struct kobject *mp_hotplug_kobj;
static struct kobject *thermal_kobj;
static struct kobject *apps_kobj;
static struct kobject *display_kobj;
static struct kobject *pnpmgr_kobj;
static struct kobject *sysinfo_kobj;
static struct kobject *battery_kobj;
static struct kobject *cluster_root_kobj;
static struct kobject *cluster_kobj[MAX_TYPE];
static struct kobject *hotplug_kobj[MAX_TYPE];
static struct kobject **cpuX_kobj[MAX_TYPE];

struct pnp_cluster_info {
	
	cpumask_t cpu_mask;        
	int mp_cpunum_max;          
	int mp_cpunum_min;          
	int user_cpunum_max;        
	int user_cpunum_min;        
	
	int max_freq_info;          
	int min_freq_info;          
	int *thermal_freq;          
	int user_perf_lvl;          
	int user_lvl_to_min_freq;   
	
	int cpu_seq[NR_CPUS];       
	int cpunum_max;             
	int cpunum_min;             
	int num_cpus;               
	int is_sync;                
	int *perf_table;            
};

#define define_string_show(_name, str_buf)				\
static ssize_t _name##_show						\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)		\
{									\
	return scnprintf(buf, sizeof(str_buf), "%s", str_buf);	\
}

#define define_string_store(_name, str_buf, store_cb)		\
static ssize_t _name##_store					\
(struct kobject *kobj, struct kobj_attribute *attr,		\
 const char *buf, size_t n)					\
{								\
	strncpy(str_buf, buf, sizeof(str_buf) - 1);				\
	str_buf[sizeof(str_buf) - 1] = '\0';				\
	(store_cb)(#_name);					\
	sysfs_notify(kobj, NULL, #_name);			\
	return n;						\
}

#define define_int_show(_name, int_val)				\
static ssize_t _name##_show					\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)	\
{								\
	return sprintf(buf, "%d", int_val);			\
}

#define define_int_store(_name, int_val, store_cb)		\
static ssize_t _name##_store					\
(struct kobject *kobj, struct kobj_attribute *attr,		\
 const char *buf, size_t n)					\
{								\
	int val;						\
	if (sscanf(buf, "%d", &val) > 0) {			\
		int_val = val;					\
		(store_cb)(#_name);				\
		sysfs_notify(kobj, NULL, #_name);		\
		return n;					\
	}							\
	return -EINVAL;						\
}

extern void set_ktm_freq_limit(uint32_t freq_limit);
extern void set_bcl_freq_limit(uint32_t freq_limit);

static char activity_buf[MAX_BUF];
static char non_activity_buf[MAX_BUF];
static char media_mode_buf[MAX_BUF];
static char virtual_display_buf[MAX_BUF];
static char call_sync_buf[MAX_BUF];
static int app_timeout_expired;
static int is_touch_boosted;
static int touch_boost_duration_value = 0;
static int single_layer_yuv_value = 0;
static int single_layer_rgb_value = 0;
static int is_long_duration_touch_boosted;
static int long_duration_touch_boost_duration_value = 1000;

static void null_cb(const char *attr) {
	do { } while (0);
}

define_string_show(activity_trigger, activity_buf);
define_string_store(activity_trigger, activity_buf, null_cb);
power_attr(activity_trigger);

define_string_show(non_activity_trigger, non_activity_buf);
define_string_store(non_activity_trigger, non_activity_buf, null_cb);
power_attr(non_activity_trigger);

define_string_show(media_mode, media_mode_buf);
define_string_store(media_mode, media_mode_buf, null_cb);
power_attr(media_mode);

define_int_show(single_layer_yuv,single_layer_yuv_value);
define_int_store(single_layer_yuv,single_layer_yuv_value, null_cb);
power_attr(single_layer_yuv);

define_int_show(single_layer_rgb,single_layer_rgb_value);
define_int_store(single_layer_rgb,single_layer_rgb_value, null_cb);
power_attr(single_layer_rgb);

define_string_show(virtual_display, virtual_display_buf);
define_string_store(virtual_display, virtual_display_buf, null_cb);
power_attr(virtual_display);

define_string_show(call_sync,call_sync_buf);
define_string_store(call_sync, call_sync_buf, null_cb);
power_attr(call_sync);


void set_call_sync(const char *buf)
{
	strcpy(call_sync_buf, buf);
	call_sync_buf[strlen(buf)] = '\0';

	sysfs_notify(apps_kobj, NULL, "call_sync");
}
EXPORT_SYMBOL(set_call_sync);

static int thermal_g0_value = MAX_VALUE;
static int thermal_final_bcpu_value = MAX_VALUE;
static int thermal_final_lcpu_value = MAX_VALUE;
static int thermal_final_gpu_value = MAX_VALUE;
static int thermal_cpus_offlined_value = 0;
static int thermal_batt_value = 0;
static int thermal_temp_emmc_value = 0;
static int data_throttling_value;
static int thermal_temp_cpu[MAX_CPU_NUM];
static int thermal_bcpu_notify_pnp_thres_value = 0;
static int thermal_lcpu_notify_pnp_thres_value = 0;
static int thermal_bcpu_notify_diff_value = 20;
static int thermal_lcpu_notify_diff_value = 10;
static int thermal_ktm_freq_limit_value = MAX_VALUE;
static int thermal_bcl_freq_limit_value = MAX_VALUE;
static int thermal_min_freq_limit_value = 0;
static int cpu_asn_value = 0;


define_int_show(thermal_temp_cpu0, thermal_temp_cpu[0]);
define_int_store(thermal_temp_cpu0, thermal_temp_cpu[0], null_cb);
power_attr(thermal_temp_cpu0);

define_int_show(thermal_temp_cpu1, thermal_temp_cpu[1]);
define_int_store(thermal_temp_cpu1, thermal_temp_cpu[1], null_cb);
power_attr(thermal_temp_cpu1);

define_int_show(thermal_temp_cpu2, thermal_temp_cpu[2]);
define_int_store(thermal_temp_cpu2, thermal_temp_cpu[2], null_cb);
power_attr(thermal_temp_cpu2);

define_int_show(thermal_temp_cpu3, thermal_temp_cpu[3]);
define_int_store(thermal_temp_cpu3, thermal_temp_cpu[3], null_cb);
power_attr(thermal_temp_cpu3);

define_int_show(thermal_bcpu_notify_pnp_thres, thermal_bcpu_notify_pnp_thres_value);
define_int_store(thermal_bcpu_notify_pnp_thres, thermal_bcpu_notify_pnp_thres_value, null_cb);
power_attr(thermal_bcpu_notify_pnp_thres);

define_int_show(thermal_lcpu_notify_pnp_thres, thermal_lcpu_notify_pnp_thres_value);
define_int_store(thermal_lcpu_notify_pnp_thres, thermal_lcpu_notify_pnp_thres_value, null_cb);
power_attr(thermal_lcpu_notify_pnp_thres);

define_int_show(thermal_bcpu_notify_diff, thermal_bcpu_notify_diff_value);
define_int_store(thermal_bcpu_notify_diff, thermal_bcpu_notify_diff_value, null_cb);
power_attr(thermal_bcpu_notify_diff);

define_int_show(thermal_lcpu_notify_diff, thermal_lcpu_notify_diff_value);
define_int_store(thermal_lcpu_notify_diff, thermal_lcpu_notify_diff_value, null_cb);
power_attr(thermal_lcpu_notify_diff);

define_int_show(thermal_min_freq_limit, thermal_min_freq_limit_value);
define_int_store(thermal_min_freq_limit, thermal_min_freq_limit_value, null_cb);
power_attr(thermal_min_freq_limit);

define_int_show(cpu_asn, cpu_asn_value);
define_int_store(cpu_asn, cpu_asn_value, null_cb);
power_attr(cpu_asn);


static const unsigned long big_cluster_mask = CONFIG_PERFORMANCE_CLUSTER_CPU_MASK;
static const unsigned long little_cluster_mask = CONFIG_POWER_CLUSTER_CPU_MASK;

int pnpmgr_cpu_temp_notify(int cpu, int temp)
{
	char buf[20];
	int temp_diff;
	static int pre_notify_temp[MAX_CPU_NUM];

	if (thermal_temp_cpu[cpu] != temp) {
		temp_diff = abs(pre_notify_temp[cpu] - temp);
		scnprintf(buf, sizeof(buf), "thermal_temp_cpu%d", cpu);
		buf[sizeof(buf) - 1] = '\0';
		if (cpumask_test_cpu(cpu, (struct cpumask *)&big_cluster_mask)){
			if(thermal_bcpu_notify_pnp_thres_value > 0 &&
				((temp > thermal_bcpu_notify_pnp_thres_value &&
				temp_diff > thermal_bcpu_notify_diff_value) ||
				(temp < thermal_bcpu_notify_pnp_thres_value &&
				pre_notify_temp[cpu] >= thermal_bcpu_notify_pnp_thres_value))){
				pr_debug("%s: bcpu[%d] cur_temp=%d pre_temp=%d\n",
					__func__, cpu, temp, pre_notify_temp[cpu]);
				pre_notify_temp[cpu] = temp;
				sysfs_notify(thermal_kobj, NULL, buf);
			}
		}else if (cpumask_test_cpu(cpu, (struct cpumask *)&little_cluster_mask)){
			if(thermal_lcpu_notify_pnp_thres_value > 0 &&
				((temp > thermal_lcpu_notify_pnp_thres_value &&
				temp_diff > thermal_lcpu_notify_diff_value) ||
				(temp < thermal_lcpu_notify_pnp_thres_value &&
				pre_notify_temp[cpu] >= thermal_lcpu_notify_pnp_thres_value))){
				pr_debug("%s: lcpu[%d] cur_temp=%d pre_temp=%d\n",
					__func__, cpu, temp, pre_notify_temp[cpu]);
				pre_notify_temp[cpu] = temp;
				sysfs_notify(thermal_kobj, NULL, buf);
			}
		}
		thermal_temp_cpu[cpu] = temp;
	}
	return 0;
}

define_int_show(thermal_temp_emmc, thermal_temp_emmc_value);
define_int_store(thermal_temp_emmc, thermal_temp_emmc_value, null_cb);
power_attr(thermal_temp_emmc);


define_int_show(thermal_g0, thermal_g0_value);
define_int_store(thermal_g0, thermal_g0_value, null_cb);
power_attr(thermal_g0);

define_int_show(thermal_batt, thermal_batt_value);
define_int_store(thermal_batt, thermal_batt_value, null_cb);
int pnpmgr_batt_thermal_notify(int temp)
{
	pr_debug("%s: thermal batt temp = %d\n", __func__, temp);
	if (thermal_batt_value != temp) {
		thermal_batt_value = temp;
		sysfs_notify(thermal_kobj, NULL, "thermal_batt");
	}
	return 0;
}
power_attr(thermal_batt);

define_int_show(thermal_final_bcpu, thermal_final_bcpu_value);
define_int_store(thermal_final_bcpu, thermal_final_bcpu_value, null_cb);
power_attr(thermal_final_bcpu);

define_int_show(thermal_final_lcpu, thermal_final_lcpu_value);
define_int_store(thermal_final_lcpu, thermal_final_lcpu_value, null_cb);
power_attr(thermal_final_lcpu);

define_int_show(thermal_final_gpu, thermal_final_gpu_value);
define_int_store(thermal_final_gpu, thermal_final_gpu_value, null_cb);
power_attr(thermal_final_gpu);

define_int_show(thermal_cpus_offlined, thermal_cpus_offlined_value);
define_int_store(thermal_cpus_offlined, thermal_cpus_offlined_value, null_cb);
power_attr(thermal_cpus_offlined);

define_int_show(thermal_ktm_freq_limit, thermal_ktm_freq_limit_value);
static ssize_t
thermal_ktm_freq_limit_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val;

	if (sscanf(buf, "%d", &val) > 0) {
		thermal_ktm_freq_limit_value = val;
		set_ktm_freq_limit(val);
		return n;
	}
	return -EINVAL;
}
power_attr(thermal_ktm_freq_limit);

define_int_show(thermal_bcl_freq_limit, thermal_bcl_freq_limit_value);
static ssize_t
thermal_bcl_freq_limit_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val;

	if (sscanf(buf, "%d", &val) > 0) {
		thermal_bcl_freq_limit_value = val;
		set_bcl_freq_limit(val);
		return n;
	}
	return -EINVAL;
}
power_attr(thermal_bcl_freq_limit);

static int default_rule_value = 1;
define_int_show(default_rule, default_rule_value);
power_ro_attr(default_rule);

static unsigned int info_gpu_max_clk;
void set_gpu_clk(unsigned int value)
{
	info_gpu_max_clk = value;
}

ssize_t
gpu_max_clk_show(struct kobject *kobj, struct kobj_attribute *attr,
                char *buf)
{
	int ret = 0;
	ret = sprintf(buf, "%u", info_gpu_max_clk);
	return ret;
}
power_ro_attr(gpu_max_clk);

define_int_show(pause_dt, data_throttling_value);
define_int_store(pause_dt, data_throttling_value, null_cb);
power_attr(pause_dt);

static int charging_enabled_value;
define_int_show(charging_enabled, charging_enabled_value);
ssize_t
charging_enabled_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	return 0;
}
power_attr(charging_enabled);

int pnpmgr_battery_charging_enabled(int charging_enabled)
{
	pr_debug("%s: result = %d\n", __func__, charging_enabled);
	if (charging_enabled_value != charging_enabled) {
		charging_enabled_value = charging_enabled;
		sysfs_notify(battery_kobj, NULL, "charging_enabled");
	}
	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static char mp_nw_arg[MAX_BUF];
static char mp_tw_arg[MAX_BUF];
static char mp_ns_arg[MAX_BUF];
static char mp_ts_arg[MAX_BUF];
static int mp_decision_ms_value;
static int mp_min_cpus_value;
static int mp_max_cpus_value;
static int mp_spc_enabled_value;
static int mp_sync_enabled_value;
static char mp_util_high_and_arg[MAX_BUF];
static char mp_util_high_or_arg[MAX_BUF];
static char mp_util_low_and_arg[MAX_BUF];
static char mp_util_low_or_arg[MAX_BUF];

define_string_show(mp_nw, mp_nw_arg);
define_string_store(mp_nw, mp_nw_arg, null_cb);
power_attr(mp_nw);

define_string_show(mp_tw, mp_tw_arg);
define_string_store(mp_tw, mp_tw_arg, null_cb);
power_attr(mp_tw);

define_string_show(mp_ns, mp_ns_arg);
define_string_store(mp_ns, mp_ns_arg, null_cb);
power_attr(mp_ns);

define_string_show(mp_ts, mp_ts_arg);
define_string_store(mp_ts, mp_ts_arg, null_cb);
power_attr(mp_ts);

define_int_show(mp_decision_ms, mp_decision_ms_value);
define_int_store(mp_decision_ms, mp_decision_ms_value, null_cb);
power_attr(mp_decision_ms);

define_int_show(mp_min_cpus, mp_min_cpus_value);
define_int_store(mp_min_cpus, mp_min_cpus_value, null_cb);
power_attr(mp_min_cpus);

define_int_show(mp_max_cpus, mp_max_cpus_value);
define_int_store(mp_max_cpus, mp_max_cpus_value, null_cb);
power_attr(mp_max_cpus);

define_int_show(mp_spc_enabled, mp_spc_enabled_value);
define_int_store(mp_spc_enabled, mp_spc_enabled_value, null_cb);
power_attr(mp_spc_enabled);

define_int_show(mp_sync_enabled, mp_sync_enabled_value);
define_int_store(mp_sync_enabled, mp_sync_enabled_value, null_cb);
power_attr(mp_sync_enabled);

define_string_show(mp_util_high_and, mp_util_high_and_arg);
define_string_store(mp_util_high_and, mp_util_high_and_arg, null_cb);
power_attr(mp_util_high_and);

define_string_show(mp_util_high_or, mp_util_high_or_arg);
define_string_store(mp_util_high_or, mp_util_high_or_arg, null_cb);
power_attr(mp_util_high_or);

define_string_show(mp_util_low_and, mp_util_low_and_arg);
define_string_store(mp_util_low_and, mp_util_low_and_arg, null_cb);
power_attr(mp_util_low_and);

define_string_show(mp_util_low_or, mp_util_low_or_arg);
define_string_store(mp_util_low_or, mp_util_low_or_arg, null_cb);
power_attr(mp_util_low_or);
#endif 

static int trace_trigger_value = 0;
define_int_show(trace_trigger, trace_trigger_value);
define_int_store(trace_trigger, trace_trigger_value, null_cb);
power_attr(trace_trigger);


define_int_show(touch_boost_duration, touch_boost_duration_value);
define_int_store(touch_boost_duration, touch_boost_duration_value, null_cb);
power_attr(touch_boost_duration);

static struct delayed_work touch_boost_work;
static ssize_t
touch_boost_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d", is_touch_boosted);
}
static ssize_t
touch_boost_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val;

	if (!touch_boost_duration_value)
		return n;

	if (sscanf(buf, "%d", &val) > 0) {
		if (val == 0) {
			cancel_delayed_work(&touch_boost_work);
			flush_scheduled_work();
			is_touch_boosted = 0;
			sysfs_notify(pnpmgr_kobj, NULL, "touch_boost");
		}
		else if (val) {
			if (!is_touch_boosted) {
				is_touch_boosted = 1;
				sysfs_notify(pnpmgr_kobj, NULL, "touch_boost");
			}
			mod_delayed_work(system_wq, &touch_boost_work, msecs_to_jiffies(touch_boost_duration_value));
			is_touch_boosted = 1;
		}
		return n;
	}
	return -EINVAL;
}
power_attr(touch_boost);

define_int_show(long_duration_touch_boost_duration, long_duration_touch_boost_duration_value);
define_int_store(long_duration_touch_boost_duration, long_duration_touch_boost_duration_value, null_cb);
power_attr(long_duration_touch_boost_duration);

static struct delayed_work long_duration_touch_boost_work;
static ssize_t
long_duration_touch_boost_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d", is_long_duration_touch_boosted);
}
static ssize_t
long_duration_touch_boost_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val;

	if (!long_duration_touch_boost_duration_value)
		return n;

	if (sscanf(buf, "%d", &val) > 0) {
		if (val == 0) {
			cancel_delayed_work(&long_duration_touch_boost_work);
			flush_scheduled_work();
			is_long_duration_touch_boosted = 0;
			sysfs_notify(pnpmgr_kobj, NULL, "long_duration_touch_boost");
		}
		else if (val) {
			if (!is_long_duration_touch_boosted) {
				is_long_duration_touch_boosted = 1;
				sysfs_notify(pnpmgr_kobj, NULL, "long_duration_touch_boost");
			}
			mod_delayed_work(system_wq, &long_duration_touch_boost_work, msecs_to_jiffies(long_duration_touch_boost_duration_value));
			is_long_duration_touch_boosted = 1;
		}
		return n;
	}
	return -EINVAL;
}
power_attr(long_duration_touch_boost);

int launch_event_enabled = 0;
define_int_show(launch_event, launch_event_enabled);
define_int_store(launch_event, launch_event_enabled, null_cb);
power_attr(launch_event);

static struct timer_list app_timer;
static ssize_t
app_timeout_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d", app_timeout_expired);
}
static ssize_t
app_timeout_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val;
	if (sscanf(buf, "%d", &val) > 0) {
		if (val == 0) {
			del_timer_sync(&app_timer);
			app_timeout_expired = 0;
			sysfs_notify(apps_kobj, NULL, "app_timeout");
		}
		else {
			del_timer_sync(&app_timer);
			app_timer.expires = jiffies + HZ * val;
			app_timer.data = 0;
			add_timer(&app_timer);
		}
		return n;
	}
	return -EINVAL;
}
power_attr(app_timeout);

static struct pnp_cluster_info info[MAX_TYPE];
static int cluster_num = 0;

static int get_cluster_type(struct kobject *kobj)
{
	struct kobject *parent = kobj->parent;
	if (!parent)
		return BC_TYPE;
	if (!strncmp(kobj->name, "big", 3))
		return BC_TYPE;
	if (!strncmp(parent->name, "big", 3))
		return BC_TYPE;
	else
		return LC_TYPE;
}

#define define_cluster_info_show(_name)			\
static ssize_t _name##_show					\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)	\
{								\
	int type = get_cluster_type(kobj);	\
	return sprintf(buf, "%d\n", info[type]._name);	\
}

static ssize_t
cpu_seq_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int i, bc, lc, num, cpu, cpu_seq[NR_CPUS];
	cpumask_t cpu_mask, bc_mask, lc_mask;
	char *pch, *tmp = (char *)buf;

	i = bc = lc = num = 0;
	cpumask_clear(&cpu_mask);
	cpumask_copy(&bc_mask, &info[BC_TYPE].cpu_mask);
	cpumask_copy(&lc_mask, &info[LC_TYPE].cpu_mask);

	
	while ((pch = strsep(&tmp, " ,.-=")) != NULL) {
		cpu_seq[num] = simple_strtoul(pch, NULL, 10);
		cpumask_set_cpu(cpu_seq[num], &cpu_mask);
		num++;
	}

	
	cpumask_and(&cpu_mask, &cpu_mask, cpu_possible_mask);

	
	for (i = 0; i < num; i++) {
		if (cpumask_test_cpu(cpu_seq[i], &info[BC_TYPE].cpu_mask)) {
			info[BC_TYPE].cpu_seq[bc++] = cpu_seq[i];
			cpumask_clear_cpu(cpu_seq[i], &bc_mask);
		}
		else if (cpumask_test_cpu(cpu_seq[i], &info[LC_TYPE].cpu_mask)) {
			info[LC_TYPE].cpu_seq[lc++] = cpu_seq[i];
			cpumask_clear_cpu(cpu_seq[i], &lc_mask);
		}
	}

	
	for (cpu = -1, i = bc; i < info[BC_TYPE].num_cpus; i++) {
		cpu = cpumask_next(cpu, &bc_mask);
		info[BC_TYPE].cpu_seq[i] = cpu;
	}
	for (cpu = -1, i = lc; i < info[LC_TYPE].num_cpus; i++) {
		cpu = cpumask_next(cpu, &lc_mask);
		info[LC_TYPE].cpu_seq[i] = cpu;
	}
	return n;
}
power_wo_attr(cpu_seq);

static ssize_t
cpu_mask_info_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int type = get_cluster_type(kobj);
	int i, cpu = -1, val = 0;

	for (i = 0; i < info[type].num_cpus; i++) {
		cpu = cpumask_next(cpu, &info[type].cpu_mask);
		val |= (1 << cpu);
	}
	return sprintf(buf, "%d\n", val);
}
power_ro_attr(cpu_mask_info);

define_cluster_info_show(mp_cpunum_max);
static ssize_t
mp_cpunum_max_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int type = get_cluster_type(kobj);
	int val;
	if (sscanf(buf, "%d", &val) > 0) {
		if ((type == BC_TYPE && val < 1)
			|| (type == LC_TYPE && val < 0))
			return -EINVAL;

		if (val > info[type].num_cpus)
			val = info[type].num_cpus;
		info[type].mp_cpunum_max = val;
		sysfs_notify(kobj, NULL, "mp_cpunum_max");
		return n;
	}
	return -EINVAL;
}
power_attr(mp_cpunum_max);

define_cluster_info_show(mp_cpunum_min);
static ssize_t
mp_cpunum_min_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int type = get_cluster_type(kobj);
	int val;

	if (sscanf(buf, "%d", &val) > 0) {
		if ((type == BC_TYPE && val < 1)
			|| (type == LC_TYPE && val < 0))
			return -EINVAL;

		if (val > info[type].num_cpus)
			val = info[type].num_cpus;
		info[type].mp_cpunum_min = val;
		sysfs_notify(kobj, NULL, "mp_cpunum_min");
		return n;
	}
	return -EINVAL;
}
power_attr(mp_cpunum_min);

static ssize_t
user_cpunum_max_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int type = get_cluster_type(kobj);
	int i;

	for (i = 0 ; i < info[type].num_cpus; i++) {
		if (info[type].cpunum_max & (1 << i))
		    break;
	}
	if (i >= info[type].num_cpus)
	    i = 0;
	else
	    i++;

	return sprintf(buf, "%d\n", i);
}
static ssize_t
user_cpunum_max_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int type = get_cluster_type(kobj);
	int val, bit, on;

	if (sscanf(buf, "%d", &val) > 0) {
		bit = val / 2;
		on = val % 2;
		if (bit >= info[type].num_cpus || bit < 0)
		    return -EINVAL;
		if (on)
		    info[type].cpunum_max |= (1 << bit);
		else
		    info[type].cpunum_max &= ~(1 << bit);
		sysfs_notify(kobj, NULL, "user_cpunum_max");
		return n;
	}
	return -EINVAL;
}
power_attr(user_cpunum_max);

static ssize_t
user_cpunum_min_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int type = get_cluster_type(kobj);
	int i;

	for (i = info[type].num_cpus-1 ; i >= 0 ; i--) {
		if (info[type].cpunum_min & (1 << i))
		    break;
	}
	if (i < 0)
	    i = 0;
	else
	    i++;

	return sprintf(buf, "%d\n", i);
}
static ssize_t
user_cpunum_min_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int type = get_cluster_type(kobj);
	int val, bit, on;

	if (sscanf(buf, "%d", &val) > 0) {
		bit = val / 2;
		on = val % 2;
		if (bit >= info[type].num_cpus || bit < 0)
		    return -EINVAL;
		if (on)
		    info[type].cpunum_min |= (1 << bit);
		else
		    info[type].cpunum_min &= ~(1 << bit);
		sysfs_notify(kobj, NULL, "user_cpunum_min");
		return n;
	}
	return -EINVAL;
}
power_attr(user_cpunum_min);

define_cluster_info_show(max_freq_info);
power_ro_attr(max_freq_info);

define_cluster_info_show(min_freq_info);
power_ro_attr(min_freq_info);

static inline void scaling_freq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
			policy->cpuinfo.max_freq);
}

static ssize_t
scaling_max_freq_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int type = get_cluster_type(kobj);
	int fcpu, rcpu, ret = 0;
	struct cpufreq_policy policy;

	if (info[type].is_sync) {
		for_each_cpu_and(rcpu, &info[type].cpu_mask, cpu_online_mask) {
			ret = 1;
			break;
		}
		if (!ret)
			return -EINVAL;
	} else {
		sscanf(kobj->name, "cpu%d", &fcpu);
		rcpu = info[type].cpu_seq[fcpu];
		if (!cpu_online(rcpu))
			return -EINVAL;
	}

	ret = cpufreq_get_policy(&policy, rcpu);
	if (ret)
		return -EINVAL;

	return sprintf(buf, "%u\n", policy.max);
}
static ssize_t
scaling_max_freq_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int type = get_cluster_type(kobj);
	int fcpu, rcpu, val, ret = 0;
	struct cpufreq_policy new_policy, *policy = NULL;

	get_online_cpus();

	if (info[type].is_sync) {
		for_each_cpu_and(rcpu, &info[type].cpu_mask, cpu_online_mask) {
			ret = 1;
			break;
		}
		if (!ret) {
			
			ret = -EPERM;
			goto failure;
		}
	} else {
		sscanf(kobj->name, "cpu%d", &fcpu);
		rcpu = info[type].cpu_seq[fcpu];
		if (!cpu_online(rcpu)) {
			pr_warn("%s: cpu%d is offline in cluster%d\n", __func__, rcpu, type);
			ret = -EPERM;
			goto failure;
		}
	}

	
	ret = cpufreq_get_policy(&new_policy, rcpu);
	if (ret) {
		pr_warn("%s: Fail to copy cpu%d policy\n", __func__, rcpu);
		ret = -EPERM;
		goto failure;
	}
	new_policy.max = new_policy.user_policy.max;
	new_policy.min = new_policy.user_policy.min;

	if (sscanf(buf, "%d", &val) > 0)
		new_policy.max = val;

	scaling_freq_verify(&new_policy);

	
	policy = cpufreq_cpu_get(rcpu);
	if (!policy || !policy->governor) {
		pr_warn("%s: cpu%d policy or governor is NULL\n", __func__, rcpu);
		ret = -EPERM;
		goto failure;
	}

	down_write(&policy->rwsem);
	policy->user_policy.max = new_policy.max;
	up_write(&policy->rwsem);

	ret = cpufreq_update_policy(rcpu);

	if (ret) {
		pr_warn("%s: Fail to update cpu%d max freq to %d\n", __func__, rcpu, val);
		ret = -EPERM;
		goto failure;
	}

	cpufreq_cpu_put(policy);
	put_online_cpus();
	return n;

failure:
	if (policy)
		cpufreq_cpu_put(policy);

	put_online_cpus();

	return ret;
}
power_attr(scaling_max_freq);

static ssize_t
scaling_min_freq_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int type = get_cluster_type(kobj);
	int fcpu, rcpu, ret = 0;
	struct cpufreq_policy policy;

	if (info[type].is_sync) {
		for_each_cpu_and(rcpu, &info[type].cpu_mask, cpu_online_mask) {
			ret = 1;
			break;
		}
		if (!ret)
			return -EINVAL;
	} else {
		sscanf(kobj->name, "cpu%d", &fcpu);
		rcpu = info[type].cpu_seq[fcpu];
		if (!cpu_online(rcpu))
			return -EINVAL;
	}

	ret = cpufreq_get_policy(&policy, rcpu);
	if (ret)
		return -EINVAL;

	return sprintf(buf, "%u\n", policy.min);
}
static ssize_t
scaling_min_freq_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int type = get_cluster_type(kobj);
	int fcpu, rcpu, val, ret = 0;
	struct cpufreq_policy new_policy, *policy = NULL;

	get_online_cpus();

	if (info[type].is_sync) {
		for_each_cpu_and(rcpu, &info[type].cpu_mask, cpu_online_mask) {
			ret = 1;
			break;
		}
		if (!ret) {
			
			ret = -EPERM;
			goto failure;
		}
	} else {
		sscanf(kobj->name, "cpu%d", &fcpu);
		rcpu = info[type].cpu_seq[fcpu];
		if (!cpu_online(rcpu)) {
			pr_warn("%s: cpu%d is offline in cluster%d\n", __func__, rcpu, type);
			ret = -EPERM;
			goto failure;
		}
	}

	
	ret = cpufreq_get_policy(&new_policy, rcpu);
	if (ret) {
		pr_warn("%s: Fail to copy cpu%d policy\n", __func__, rcpu);
		ret = -EPERM;
		goto failure;
	}

	new_policy.max = new_policy.user_policy.max;
	new_policy.min = new_policy.user_policy.min;

	if (sscanf(buf, "%d", &val) > 0)
		new_policy.min = val;

	scaling_freq_verify(&new_policy);

	
	policy = cpufreq_cpu_get(rcpu);
	if (!policy || !policy->governor) {
		pr_warn("%s: cpu%d policy or governor is NULL\n", __func__, rcpu);
		ret = -EPERM;
		goto failure;
	}

	down_write(&policy->rwsem);
	policy->user_policy.min = new_policy.min;
	up_write(&policy->rwsem);

	ret = cpufreq_update_policy(rcpu);

	if (ret) {
		pr_warn("%s: Fail to update cpu%d min freq to %d\n", __func__, rcpu, val);
		ret = -EPERM;
		goto failure;
	}

	cpufreq_cpu_put(policy);
	put_online_cpus();
	return n;

failure:
	if (policy)
		cpufreq_cpu_put(policy);

	put_online_cpus();

	return ret;
}
power_attr(scaling_min_freq);

static ssize_t
thermal_freq_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int type = get_cluster_type(kobj);
	int cpu;
	sscanf(kobj->name, "cpu%d", &cpu);
	return sprintf(buf, "%d\n", info[type].thermal_freq[cpu]);
}
static ssize_t
thermal_freq_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int type = get_cluster_type(kobj);
	int val, cpu;

	
	if (sscanf(buf, "%d", &val) > 0) {
		sscanf(kobj->name, "cpu%d", &cpu);
		if (thermal_min_freq_limit_value  > 0 && val < thermal_min_freq_limit_value ){
			pr_info("%s: set freq %d lower than min require. Set to %d\n", __func__, val, thermal_min_freq_limit_value);
			val = thermal_min_freq_limit_value;
		}
		info[type].thermal_freq[cpu] = val;
		sysfs_notify(kobj, NULL, "thermal_freq");
		return n;
	}
	return -EINVAL;;
}
power_attr(thermal_freq);

static int perf_value = 0;
static struct user_perf_data *pnp_perf_data;
void pnpmgr_init_perf_table(struct user_perf_data *pdata)
{
	pnp_perf_data = pdata;
}
EXPORT_SYMBOL(pnpmgr_init_perf_table);

static ssize_t
user_perf_lvl_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", perf_value);
}

static ssize_t
user_perf_lvl_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int type = get_cluster_type(kobj);
	int val, bit, on, i, min_freq = 0;

	if (!info[type].perf_table)
		return -EINVAL;

	if (sscanf(buf, "%d", &val) > 0) {
		bit = val / 2;
		on = val % 2;
		if (val < 0 || bit > USER_PERF_LVL_HIGHEST)
			return -EINVAL;

		if (on)
			perf_value |= (1 << bit);
		else
			perf_value &= ~(1 << bit);

		for (i = USER_PERF_LVL_HIGHEST; i >= USER_PERF_LVL_LOWEST; i--) {
			if (perf_value & (1 << i)) {
				min_freq = info[type].perf_table[i];
				break;
			}
		}

		info[type].user_lvl_to_min_freq = min_freq;
		sysfs_notify(kobj, NULL, "user_perf_lvl");
		sysfs_notify(kobj, NULL, "user_lvl_to_min_freq");
		return n;
	}
	return -EINVAL;
}
power_attr(user_perf_lvl);

define_cluster_info_show(user_lvl_to_min_freq);
power_ro_attr(user_lvl_to_min_freq);

static struct attribute *mp_hotplug_g[] = {
#ifdef CONFIG_HOTPLUG_CPU
	&mp_nw_attr.attr,
	&mp_tw_attr.attr,
	&mp_ns_attr.attr,
	&mp_ts_attr.attr,
	&mp_decision_ms_attr.attr,
	&mp_min_cpus_attr.attr,
	&mp_max_cpus_attr.attr,
	&mp_spc_enabled_attr.attr,
	&mp_sync_enabled_attr.attr,
	&mp_util_high_and_attr.attr,
	&mp_util_high_or_attr.attr,
	&mp_util_low_and_attr.attr,
	&mp_util_low_or_attr.attr,
#endif
	NULL,
};

static struct attribute *thermal_g[] = {
	&thermal_g0_attr.attr,
	&thermal_final_bcpu_attr.attr,
	&thermal_final_lcpu_attr.attr,
	&thermal_final_gpu_attr.attr,
	&thermal_batt_attr.attr,
	&thermal_cpus_offlined_attr.attr,
	&thermal_temp_emmc_attr.attr,
	&thermal_temp_cpu0_attr.attr,
	&thermal_temp_cpu1_attr.attr,
	&thermal_temp_cpu2_attr.attr,
	&thermal_temp_cpu3_attr.attr,
	&thermal_lcpu_notify_pnp_thres_attr.attr,
	&thermal_bcpu_notify_pnp_thres_attr.attr,
	&thermal_bcpu_notify_diff_attr.attr,
	&thermal_lcpu_notify_diff_attr.attr,
	&pause_dt_attr.attr,
	&thermal_ktm_freq_limit_attr.attr,
	&thermal_bcl_freq_limit_attr.attr,
	&thermal_min_freq_limit_attr.attr,
        &cpu_asn_attr.attr,
	NULL,
};

static struct attribute *apps_g[] = {
	&activity_trigger_attr.attr,
	&non_activity_trigger_attr.attr,
	&media_mode_attr.attr,
	&app_timeout_attr.attr,
	&call_sync_attr.attr,
	&trace_trigger_attr.attr,
	NULL,
};

static struct attribute *display_g[] = {
	&virtual_display_attr.attr,
	&single_layer_yuv_attr.attr,
	&single_layer_rgb_attr.attr,
	NULL,
};

static struct attribute *sysinfo_g[] = {
	&cpu_seq_attr.attr,
	&gpu_max_clk_attr.attr,
	NULL,
};

static struct attribute *battery_g[] = {
	&charging_enabled_attr.attr,
	NULL,
};

static struct attribute *pnpmgr_g[] = {
	&launch_event_attr.attr,
	&default_rule_attr.attr,
	&touch_boost_attr.attr,
	&touch_boost_duration_attr.attr,
	&long_duration_touch_boost_attr.attr,
	&long_duration_touch_boost_duration_attr.attr,
	NULL,
};

static struct attribute *hotplug_g[] = {
	&cpu_mask_info_attr.attr,
	&mp_cpunum_max_attr.attr,
	&mp_cpunum_min_attr.attr,
	&user_cpunum_max_attr.attr,
	&user_cpunum_min_attr.attr,
	NULL,
};

static struct attribute *cpuX_g[] = {
	&max_freq_info_attr.attr,
	&min_freq_info_attr.attr,
	&scaling_max_freq_attr.attr,
	&scaling_min_freq_attr.attr,
	&thermal_freq_attr.attr,
	NULL,
};

static struct attribute *cluster_type_g[] = {
	&user_perf_lvl_attr.attr,
	&user_lvl_to_min_freq_attr.attr,
	NULL,
};

static struct attribute_group mp_hotplug_attr_group = {
	.attrs = mp_hotplug_g,
};

static struct attribute_group thermal_attr_group = {
	.attrs = thermal_g,
};

static struct attribute_group apps_attr_group = {
	.attrs = apps_g,
};

static struct attribute_group display_attr_group = {
	.attrs = display_g,
};

static struct attribute_group sysinfo_attr_group = {
	.attrs = sysinfo_g,
};

static struct attribute_group battery_attr_group = {
	.attrs = battery_g,
};

static struct attribute_group pnpmgr_attr_group = {
	.attrs = pnpmgr_g,
};

static struct attribute_group hotplug_attr_group = {
	.attrs = hotplug_g,
};

static struct attribute_group cpuX_attr_group = {
	.attrs = cpuX_g,
};

static struct attribute_group cluster_type_attr_group = {
	.attrs = cluster_type_g,
};

#ifdef CONFIG_HOTPLUG_CPU
static int __cpuinit cpu_hotplug_callback(struct notifier_block *nfb, unsigned long action, void *hcpu)
{
	switch (action) {
		
		case CPU_ONLINE:
		case CPU_ONLINE_FROZEN:
			sysfs_notify(pnpmgr_kobj, NULL, "default_rule");
			break;
		case CPU_DEAD:
		case CPU_DEAD_FROZEN:
			break;
	}
	return NOTIFY_OK;
}

static struct notifier_block __refdata cpu_hotplug_notifier = {
	.notifier_call = cpu_hotplug_callback,
	.priority = -10, 
};
#endif

static void app_timeout_handler(unsigned long data)
{
	app_timeout_expired = 1;
	sysfs_notify(apps_kobj, NULL, "app_timeout");
}

static void touch_boost_handler(struct work_struct *work)
{
	is_touch_boosted = 0;
	sysfs_notify(pnpmgr_kobj, NULL, "touch_boost");
}

static void long_duration_touch_boost_handler(struct work_struct *work)
{
	is_long_duration_touch_boosted = 0;
	sysfs_notify(pnpmgr_kobj, NULL, "long_duration_touch_boost");
}

#define swap_value(a,b)	\
do {			\
	__typeof__(a) c;	\
	c = a; a = b; b = c;	\
} while (0)

extern int is_sync_cpu(struct cpumask *mask, int first_cpu);

static int get_cpu_frequency(struct cpumask *mask, int *min, int *max)
{
	struct cpufreq_frequency_table *table;
	int i = 0, cpu;

	for_each_cpu(cpu, mask) {
		table = cpufreq_frequency_get_table(cpu);
		if (table)
			break;
	}
	if (!table) {
		pr_err("Fail to get cpu[%d] cpufreq table\n", cpu);
		return -EINVAL;
	}

	while (table[i].frequency != CPUFREQ_TABLE_END)
		i++;

	if (--i < 0)
		i = 0;

	*min = table[0].frequency;
	*max = table[i].frequency;

	return 0;
}

static int init_cluster_info_by_dt(const struct device_node *node)
{
	int i, j, k, cluster_cnt = 0, ret = 0;
	uint32_t val = 0;
	char *key = "htc,cluster-map";
	cpumask_t cluster_cpus[MAX_TYPE];
	int first_cpu, minfreq, maxfreq;

	if (!of_get_property(node, key, &cluster_cnt)
		|| cluster_cnt <= 0) {
		pr_debug("Property %s not defined.\n", key);
		return -ENODEV;
	}
	cluster_cnt /= sizeof(__be32);

	pr_info("%s: find %d cluster\n", __func__, cluster_cnt);

	if (!cluster_cnt || cluster_cnt > MAX_TYPE) {
		pr_err("Invalid number(%d) of entry for %s\n",
				cluster_cnt, key);
		return -EINVAL;
	}

	for (i = 0; i < cluster_cnt; i++) {
		ret = of_property_read_u32_index(node, key,
							i , &val);
		pr_info("%s: get cluster[%d] cpu_mask %x\n", __func__, i, val);
		*cluster_cpus[i].bits = val;
	}

	for (i = 0; i < cluster_cnt; i++) {

		first_cpu = cpumask_first(&cluster_cpus[i]);
		if (get_cpu_frequency(&cluster_cpus[i], &minfreq, &maxfreq))
			return -EINVAL;

		info[i].mp_cpunum_max = info[i].num_cpus = cpumask_weight(&cluster_cpus[i]);
		info[i].max_freq_info = maxfreq;
		info[i].min_freq_info = minfreq;
		cpumask_copy(&info[i].cpu_mask, &cluster_cpus[i]);

		info[i].cpu_seq[0] = first_cpu;
		for (j = 1; j < info[i].num_cpus; j++)
			info[i].cpu_seq[j] = cpumask_next(info[i].cpu_seq[j-1], &cluster_cpus[i]);

		info[i].is_sync = is_sync_cpu(&cluster_cpus[i], first_cpu);

		k = info[i].is_sync ? 1 : info[i].num_cpus;

		info[i].thermal_freq = kzalloc(k * sizeof(int), GFP_KERNEL);

		for (j = 0; j < k; j++)
			info[i].thermal_freq[j] = MAX_VALUE;
	}

	cluster_num = cluster_cnt;

	return cluster_cnt;
}

#define PERF_TABLE "htc,perf_table"
static int perf_table_use_dt=0;
static int init_perf_table_by_dt(const struct device_node *node)
{
	uint32_t i, j, len = 0;
	int *perf_table[MAX_TYPE] = {NULL};
	int total_entry = cluster_num * USER_PERF_LVL_TOTAL;
	int ret;

	if (!of_get_property(node, PERF_TABLE, &len)
		|| len <= 0) {
		pr_debug("Property %s not defined.\n", PERF_TABLE);
		return -ENODEV;
	}

	len /= sizeof(__be32);

	pr_info("%s: find %d elements for %s \n", __func__, len, PERF_TABLE);

	if (len != total_entry) {
		pr_err("Invalid number(%d) of entry for %s, should be %d entry\n",
			len, PERF_TABLE, total_entry);
		return -EINVAL;
	}

	for(i = 0; i < cluster_num; i++)
	{
		perf_table[i] = kzalloc(USER_PERF_LVL_TOTAL * sizeof(int), GFP_KERNEL);
		if(!perf_table[i]){
			pr_err("%s: out of memory while allocating perf_table[%d]\n",
				__func__, i);
			ret = -ENOMEM;
			goto fail;
		}
	}

	for (j = 0; j < cluster_num; j++)
	{
		for (i = 0; i < USER_PERF_LVL_TOTAL; i++) {
			ret = of_property_read_u32_index(node, PERF_TABLE,
							i + (USER_PERF_LVL_TOTAL * j) , &perf_table[j][i]);
			pr_info("%s: get perf_tbl[%d]=%d\n", __func__, j, perf_table[j][i]);
			if (ret) {
				pr_err("Error reading index%d\n", i + (USER_PERF_LVL_TOTAL * j));
				ret = -EINVAL;
				goto fail;
			}
		}
		info[j].perf_table = perf_table[j];
	}

#if DEBUG_DUMP
	for (j = 0; j < cluster_num; j++)
	{
		for (i = 0; i < USER_PERF_LVL_TOTAL; i++) {
			pr_info("%s: dump info[%d].perf_tbl[%d]=%d\n", __func__, j, i, info[j].perf_table[i]);
		}
		info[j].perf_table = perf_table[j];
	}
#endif

	perf_table_use_dt = 1;

	return 0;
fail:
	for(i = 0; i < cluster_num; i++)
		if(perf_table[i])
			kfree(perf_table[i]);

	return ret;
}

static int init_cluster_info(void)
{
	const struct device_node *top = NULL;
	int ret = 0;

	top = of_find_node_by_path("/soc/htc_pnpmgr");

	if(top) {
		pr_info("%s: get cluster info from dt\n", __func__);
		ret = init_cluster_info_by_dt(top);
	}

	if (!top || ret < 0)
		pr_info("%s: cluster info init fail",__func__);

	if(top){
		pr_info("%s: get perf table info from dt\n", __func__);
		ret = init_perf_table_by_dt(top);
	}

	if( !top || ret < 0){
		pr_info("%s: perf dt fail or not found, try to get perf table info from board file\n",
			__func__);

		if (pnp_perf_data) {
			pr_info("%s:find perf table info in board file\n", __func__);
			info[BC_TYPE].perf_table = pnp_perf_data->bc_perf_table;
			info[LC_TYPE].perf_table = pnp_perf_data->lc_perf_table;
			ret = 0;
		}
	}

	return ret;
}

static int __init pnpmgr_init(void)
{
	int ret, i, j;
	char *name[MAX_TYPE] = {"big", "little"};
	char buf[10];

	init_timer(&app_timer);
	app_timer.function = app_timeout_handler;

	INIT_DELAYED_WORK(&touch_boost_work, touch_boost_handler);
	INIT_DELAYED_WORK(&long_duration_touch_boost_work, long_duration_touch_boost_handler);

	pnpmgr_kobj = kobject_create_and_add("pnpmgr", power_kobj);

	if (!pnpmgr_kobj) {
		pr_err("%s: Can not allocate enough memory for pnpmgr.\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	mp_hotplug_kobj = kobject_create_and_add("hotplug", pnpmgr_kobj);
	thermal_kobj = kobject_create_and_add("thermal", pnpmgr_kobj);
	apps_kobj = kobject_create_and_add("apps", pnpmgr_kobj);
	display_kobj = kobject_create_and_add("display", pnpmgr_kobj);
	sysinfo_kobj = kobject_create_and_add("sysinfo", pnpmgr_kobj);
	battery_kobj = kobject_create_and_add("battery", pnpmgr_kobj);
	cluster_root_kobj = kobject_create_and_add("cluster", pnpmgr_kobj);

	if (!mp_hotplug_kobj || !thermal_kobj || !apps_kobj || !display_kobj || !sysinfo_kobj || !battery_kobj || !cluster_root_kobj) {
		pr_err("%s: Can not allocate enough memory.\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	if ((ret = init_cluster_info()) < 0)
		goto err;

	for (i = 0; i < cluster_num; i++) {
		cluster_kobj[i] = kobject_create_and_add(name[i], cluster_root_kobj);
		if (!cluster_kobj[i]) {
			pr_err("%s: Can not allocate enough memory for cluster%d\n", __func__, i);
			ret = -ENOMEM;
			goto err;
		}

		hotplug_kobj[i] = kobject_create_and_add("hotplug", cluster_kobj[i]);
		if (!hotplug_kobj[i]) {
			pr_err("%s: Can not allocate enough memory for cluster%d hotplug\n", __func__, i);
			ret = -ENOMEM;
			goto err;
		}

		if (info[i].is_sync)
			cpuX_kobj[i] = kzalloc(sizeof(struct kobject *), GFP_KERNEL);
		else
			cpuX_kobj[i] = kzalloc(info[i].num_cpus * sizeof(struct kobject *), GFP_KERNEL);

		cpuX_kobj[i][0] = kobject_create_and_add("cpu0", cluster_kobj[i]);
		if (!cpuX_kobj[i][0]) {
			pr_err("%s: Can not allocate enough memory for cluster%d cpu0\n", __func__, i);
			ret = -ENOMEM;
			goto err;
		}

		for (j = 1; j < info[i].num_cpus; j++) {
			scnprintf(buf, sizeof(buf), "cpu%d", j);

			if (info[i].is_sync) {
				ret = sysfs_create_link_nowarn(cluster_kobj[i], cpuX_kobj[i][0], buf);
				if (ret) {
					pr_err("%s: Can not create symlink for cluster%d cpu%d\n", __func__, i, j);
					ret = -ENOENT;
					goto err;
				}
			} else {
				cpuX_kobj[i][j] = kobject_create_and_add(buf, cluster_kobj[i]);
				if (!cpuX_kobj[i][j]) {
					pr_err("%s: Can not allocate enough memory for cluster%d cpu%d\n", __func__, i, j);
					ret = -ENOMEM;
					goto err;
				}
			}
		}
	}

	ret = sysfs_create_group(pnpmgr_kobj, &pnpmgr_attr_group);
	ret |= sysfs_create_group(mp_hotplug_kobj, &mp_hotplug_attr_group);
	ret |= sysfs_create_group(thermal_kobj, &thermal_attr_group);
	ret |= sysfs_create_group(apps_kobj, &apps_attr_group);
	ret |= sysfs_create_group(display_kobj, &display_attr_group);
	ret |= sysfs_create_group(sysinfo_kobj, &sysinfo_attr_group);
	ret |= sysfs_create_group(battery_kobj, &battery_attr_group);

	for (i = 0; i < cluster_num; i++) {
		ret |= sysfs_create_group(cluster_kobj[i], &cluster_type_attr_group);
		ret |= sysfs_create_group(hotplug_kobj[i], &hotplug_attr_group);
		for (j = 0; j < info[i].num_cpus; j++) {
			ret |= sysfs_create_group(cpuX_kobj[i][j], &cpuX_attr_group);
			if (info[i].is_sync)
				break;
		}
	}

	if (ret) {
		pr_err("%s: sysfs_create_group failed\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

#ifdef CONFIG_HOTPLUG_CPU
	register_hotcpu_notifier(&cpu_hotplug_notifier);
#endif

	return 0;

err:
	fda_log_pnp("pnpmgr init fail\n");
	return ret;
}

static void  __exit pnpmgr_exit(void)
{
	int i, j;
	char buf[10];

	sysfs_remove_group(pnpmgr_kobj, &pnpmgr_attr_group);
	sysfs_remove_group(mp_hotplug_kobj, &mp_hotplug_attr_group);
	sysfs_remove_group(thermal_kobj, &thermal_attr_group);
	sysfs_remove_group(apps_kobj, &apps_attr_group);
	sysfs_remove_group(display_kobj, &display_attr_group);
	sysfs_remove_group(sysinfo_kobj, &sysinfo_attr_group);
	sysfs_remove_group(battery_kobj, &battery_attr_group);

	for (i = 0; i < cluster_num; i++) {
		sysfs_remove_group(cluster_kobj[i], &cluster_type_attr_group);
		sysfs_remove_group(hotplug_kobj[i], &hotplug_attr_group);
		sysfs_remove_group(cpuX_kobj[i][0], &cpuX_attr_group);

		for (j = 1; j < info[i].num_cpus; j++) {
			if (info[i].is_sync) {
				scnprintf(buf, sizeof(buf), "cpu%d", j);
				sysfs_remove_link(cluster_kobj[i], buf);
			} else {
				sysfs_remove_group(cpuX_kobj[i][j], &cpuX_attr_group);
			}
		}
		if(perf_table_use_dt)
			if(info[i].perf_table)
				kfree(info[i].perf_table);

		kfree(info[i].thermal_freq);
		kfree(cpuX_kobj[i]);
	}

#ifdef CONFIG_HOTPLUG_CPU
	unregister_hotcpu_notifier(&cpu_hotplug_notifier);
#endif
}

module_init(pnpmgr_init);
module_exit(pnpmgr_exit);
