/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#ifndef __REGULATOR_CPR3_REGULATOR_H__
#define __REGULATOR_CPR3_REGULATOR_H__

#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/power/qcom/apm.h>
#include <linux/regulator/driver.h>

struct cpr3_controller;
struct cpr3_thread;

struct cpr3_fuse_param {
	unsigned		row;
	unsigned		bit_start;
	unsigned		bit_end;
};

#define CPR3_RO_COUNT		16

#define CPR3_MAX_SENSOR_COUNT	256

#define MAX_CHARS_PER_INT	10

struct cpr3_corner {
	int			floor_volt;
	int			ceiling_volt;
	int			open_loop_volt;
	int			last_volt;
	int			system_volt;
	int			mem_acc_volt;
	u32			proc_freq;
	int			cpr_fuse_corner;
	u32			target_quot[CPR3_RO_COUNT];
	u32			ro_scale[CPR3_RO_COUNT];
	u32			ro_mask;
	u32			irq_en;
	int			aging_derate;
};

/**
 * struct cpr3_regulator - CPR3 logical regulator instance associated with a
 *			given CPR3 hardware thread
 * @of_node:		Device node associated with the device tree child node
 *			of this CPR3 regulator
 * @thread:		Pointer to the CPR3 thread which manages this CPR3
 *			regulator
 * @name:		Unique name for this CPR3 regulator which is filled
 *			using the device tree regulator-name property
 * @rdesc:		Regulator description for this CPR3 regulator
 * @rdev:		Regulator device pointer for the regulator registered
 *			for this CPR3 regulator
 * @mem_acc_regulator:	Pointer to the optional mem-acc supply regulator used
 *			to manage memory circuitry settings based upon CPR3
 *			regulator output voltage.
 * @ldo_regulator:	Pointer to the LDO supply regulator used to manage
 *			per-cluster LDO voltage and bypass state
 * @ldo_regulator_bypass: Cached copy of the LDO regulator bypass state
 * @ldo_ret_regulator:	Pointer to the LDO retention supply regulator used to
 *			manage LDO retention bypass state
 * @corner:		Array of all corners supported by this CPR3 regulator
 * @corner_count:	The number of elements in the corner array
 * @platform_fuses:	Pointer to platform specific CPR fuse data (only used by
 *			platform specific CPR3 driver)
 * @speed_bin_fuse:	Value read from the speed bin fuse parameter
 * @speed_bins_supported: The number of speed bins supported by the device tree
 *			configuration for this CPR3 regulator
 * @cpr_rev_fuse:	Value read from the CPR fusing revision fuse parameter
 * @fuse_combo:		Platform specific enum value identifying the specific
 *			combination of fuse values found on a given chip
 * @fuse_combos_supported: The number of fuse combinations supported by the
 *			device tree configuration for this CPR3 regulator
 * @fuse_corner_count:	Number of corners defined by fuse parameters
 * @fuse_combo_corner_sum: The sum of the corner counts across all fuse combos
 * @fuse_combo_offset:	The device tree property array offset for the selected
 *			fuse combo
 * @speed_bin_corner_sum: The sum of the corner counts across all speed bins
 *			This may be specified as 0 if per speed bin parsing
 *			support is not required.
 * @speed_bin_offset:	The device tree property array offset for the selected
 *			speed bin
 * @pd_bypass_mask:	Bit mask of power domains associated with this CPR3
 *			regulator
 * @dynamic_floor_corner: Index identifying the voltage corner for the CPR3
 *			regulator whose last_volt value should be used as the
 *			global CPR floor voltage if all of the power domains
 *			associated with this CPR3 regulator are bypassed
 * @uses_dynamic_floor: Boolean flag indicating that dynamic_floor_corner should
 *			be utilized for the CPR3 regulator
 * @current_corner:	Index identifying the currently selected voltage corner
 *			for the CPR3 regulator or less than 0 if no corner has
 *			been requested
 * @last_closed_loop_corner: Index identifying the last voltage corner for the
 *			CPR3 regulator which was configured when operating in
 *			CPR closed-loop mode or less than 0 if no corner has
 *			been requested.  CPR registers are only written to when
 *			using closed-loop mode.
 * @aggregated:		Boolean flag indicating that this CPR3 regulator
 *			participated in the last aggregation event
 * @debug_corner:	Index identifying voltage corner used for displaying
 *			corner configuration values in debugfs
 * @ldo_min_headroom_volt: Minimum voltage difference in microvolts required
 *			between the VDD supply voltage and the LDO output in
 *			order for the LDO operate
 * @ldo_max_headroom_volt: Maximum voltage difference in microvolts between
 *			the input and output of the active LDO hardware to
 *			maintain optimum operability.
 * @ldo_adjust_volt:	Voltage in microvolts used to offset margin assigned
 *			to IR drop between PMIC and CPU
 * @ldo_ret_volt:	The lowest supported CPU retention voltage in
 *			microvolts. This voltage may vary part-to-part based
 *			upon the value of hardware fuses.
 * @ldo_max_volt:	The maximum physically supported LDO voltage in
 *			microvolts
 * @ldo_mode_allowed:	Boolean which indicates if LDO mode is allowed for this
 *			CPR3 regulator
 * @vreg_enabled:	Boolean defining the enable state of the CPR3
 *			regulator's regulator within the regulator framework.
 * @aging_allowed:	Boolean defining if CPR aging adjustments are allowed
 *			for this CPR3 regulator given the fuse combo of the
 *			device
 * @aging_corner:	The corner that should be configured for this regulator
 *			when an aging measurement is performed.
 * @aging_max_adjust_volt: The maximum aging voltage margin in microvolts that
 *			may be added to the target quotients of this regulator.
 *			A value of 0 may be specified if this regulator does not
 *			require any aging adjustment.
 *
 * This structure contains both configuration and runtime state data.  The
 * elements current_corner, last_closed_loop_corner, aggregated, debug_corner,
 * ldo_mode_allowed, and vreg_enabled are state variables.
 */
struct cpr3_regulator {
	struct device_node	*of_node;
	struct cpr3_thread	*thread;
	const char		*name;
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
	struct regulator	*mem_acc_regulator;
	struct regulator	*ldo_regulator;
	bool			ldo_regulator_bypass;
	struct regulator	*ldo_ret_regulator;
	struct cpr3_corner	*corner;
	int			corner_count;

	void			*platform_fuses;
	int			speed_bin_fuse;
	int			speed_bins_supported;
	int			cpr_rev_fuse;
	int			fuse_combo;
	int			fuse_combos_supported;
	int			fuse_corner_count;
	int			fuse_combo_corner_sum;
	int			fuse_combo_offset;
	int			speed_bin_corner_sum;
	int			speed_bin_offset;
	u32			pd_bypass_mask;
	int			dynamic_floor_corner;
	bool			uses_dynamic_floor;

	int			current_corner;
	int			last_closed_loop_corner;
	bool			aggregated;
	int			debug_corner;
	int			ldo_min_headroom_volt;
	int			ldo_max_headroom_volt;
	int			ldo_adjust_volt;
	int			ldo_ret_volt;
	int			ldo_max_volt;
	bool			ldo_mode_allowed;
	bool			vreg_enabled;

	bool			aging_allowed;
	int			aging_corner;
	int			aging_max_adjust_volt;
};

/**
 * struct cpr3_thread - CPR3 hardware thread data structure
 * @thread_id:		Hardware thread ID
 * @of_node:		Device node associated with the device tree child node
 *			of this CPR3 thread
 * @ctrl:		Pointer to the CPR3 controller which manages this thread
 * @vreg:		Array of CPR3 regulators handled by the CPR3 thread
 * @vreg_count:		Number of elements in the vreg array
 * @aggr_corner:	CPR corner containing the in process aggregated voltage
 *			and target quotient configurations which will be applied
 * @last_closed_loop_aggr_corner: CPR corner containing the most recent
 *			configurations which were written into hardware
 *			registers when operating in closed loop mode (i.e. with
 *			CPR enabled)
 * @consecutive_up:	The number of consecutive CPR step up events needed to
 *			to trigger an up interrupt
 * @consecutive_down:	The number of consecutive CPR step down events needed to
 *			to trigger a down interrupt
 * @up_threshold:	The number CPR error steps required to generate an up
 *			event
 * @down_threshold:	The number CPR error steps required to generate a down
 *			event
 *
 * This structure contains both configuration and runtime state data.  The
 * elements aggr_corner and last_closed_loop_aggr_corner are state variables.
 */
struct cpr3_thread {
	u32			thread_id;
	struct device_node	*of_node;
	struct cpr3_controller	*ctrl;
	struct cpr3_regulator	*vreg;
	int			vreg_count;
	struct cpr3_corner	aggr_corner;
	struct cpr3_corner	last_closed_loop_aggr_corner;

	u32			consecutive_up;
	u32			consecutive_down;
	u32			up_threshold;
	u32			down_threshold;
};

enum cpr3_mem_acc_corners {
	CPR3_MEM_ACC_LOW_CORNER		= 0,
	CPR3_MEM_ACC_HIGH_CORNER	= 1,
	CPR3_MEM_ACC_CORNERS		= 2,
};

enum cpr3_count_mode {
	CPR3_COUNT_MODE_ALL_AT_ONCE_MIN	= 0,
	CPR3_COUNT_MODE_ALL_AT_ONCE_MAX	= 1,
	CPR3_COUNT_MODE_STAGGERED	= 2,
	CPR3_COUNT_MODE_ALL_AT_ONCE_AGE	= 3,
};

enum cpr_controller_type {
	CPR_CTRL_TYPE_CPR3,
	CPR_CTRL_TYPE_CPR4,
};

struct cpr3_aging_sensor_info {
	u32			sensor_id;
	u32			ro_scale;
	int			init_quot_diff;
	int			measured_quot_diff;
	u32			bypass_mask[CPR3_MAX_SENSOR_COUNT / 32];
};

struct cpr3_controller {
	struct device		*dev;
	const char		*name;
	void __iomem		*cpr_ctrl_base;
	void __iomem		*fuse_base;
	struct list_head	list;
	struct cpr3_thread	*thread;
	int			thread_count;
	u8			*sensor_owner;
	int			sensor_count;
	int			soc_revision;
	struct mutex		lock;
	struct regulator	*vdd_regulator;
	struct regulator	*system_regulator;
	struct regulator	*mem_acc_regulator;
	struct regulator	*vdd_limit_regulator;
	int			system_supply_max_volt;
	int			mem_acc_threshold_volt;
	int			mem_acc_corner_map[CPR3_MEM_ACC_CORNERS];
	struct clk		*core_clk;
	struct clk		*iface_clk;
	struct clk		*bus_clk;
	int			irq;
	int			ceiling_irq;
	struct msm_apm_ctrl_dev *apm;
	int			apm_threshold_volt;
	int			apm_adj_volt;
	enum msm_apm_supply	apm_high_supply;
	enum msm_apm_supply	apm_low_supply;
	u32			cpr_clock_rate;
	u32			sensor_time;
	u32			loop_time;
	u32			up_down_delay_time;
	u32			idle_clocks;
	u32			step_quot_init_min;
	u32			step_quot_init_max;
	int			step_volt;
	u32			down_error_step_limit;
	u32			up_error_step_limit;
	enum cpr3_count_mode	count_mode;
	u32			count_repeat;
	u32			proc_clock_throttle;
	bool			cpr_allowed_hw;
	bool			cpr_allowed_sw;
	bool			supports_hw_closed_loop;
	bool			use_hw_closed_loop;
	enum cpr_controller_type ctrl_type;
	bool			saw_use_unit_mV;
	struct cpr3_corner	aggr_corner;
	bool			cpr_enabled;
	bool			last_corner_was_closed_loop;
	bool			cpr_suspended;
	struct dentry		*debugfs;

	int			aging_ref_volt;
	unsigned int		aging_vdd_mode;
	unsigned int		aging_complete_vdd_mode;
	int			aging_ref_adjust_volt;
	bool			aging_required;
	bool			aging_succeeded;
	bool			aging_failed;
	struct cpr3_aging_sensor_info *aging_sensor;
	int			aging_sensor_count;
};

#define CPR3_ROUND(n, d) (DIV_ROUND_UP(n, d) * (d))

#define cpr3_err(cpr3_thread, message, ...) \
	pr_err("%s: " message, (cpr3_thread)->name, ##__VA_ARGS__)
#define cpr3_info(cpr3_thread, message, ...) \
	pr_info("%s: " message, (cpr3_thread)->name, ##__VA_ARGS__)
#define cpr3_debug(cpr3_thread, message, ...) \
	pr_debug("%s: " message, (cpr3_thread)->name, ##__VA_ARGS__)

#define CPR3_CORNER_OFFSET	1

#ifdef CONFIG_REGULATOR_CPR3

int cpr3_regulator_register(struct platform_device *pdev,
			struct cpr3_controller *ctrl);
int cpr3_regulator_unregister(struct cpr3_controller *ctrl);
int cpr3_regulator_suspend(struct cpr3_controller *ctrl);
int cpr3_regulator_resume(struct cpr3_controller *ctrl);

int cpr3_allocate_threads(struct cpr3_controller *ctrl, u32 min_thread_id,
			u32 max_thread_id);
int cpr3_map_fuse_base(struct cpr3_controller *ctrl,
			struct platform_device *pdev);
int cpr3_read_fuse_param(void __iomem *fuse_base_addr,
			const struct cpr3_fuse_param *param, u64 *param_value);
int cpr3_convert_open_loop_voltage_fuse(int ref_volt, int step_volt, u32 fuse,
			int fuse_len);
u64 cpr3_interpolate(u64 x1, u64 y1, u64 x2, u64 y2, u64 x);
int cpr3_parse_array_property(struct cpr3_regulator *vreg,
			const char *prop_name, int tuple_size, u32 *out);
int cpr3_parse_corner_array_property(struct cpr3_regulator *vreg,
			const char *prop_name, int tuple_size, u32 *out);
int cpr3_parse_common_corner_data(struct cpr3_regulator *vreg);
int cpr3_parse_thread_u32(struct cpr3_thread *thread, const char *propname,
			u32 *out_value, u32 value_min, u32 value_max);
int cpr3_parse_ctrl_u32(struct cpr3_controller *ctrl, const char *propname,
			u32 *out_value, u32 value_min, u32 value_max);
int cpr3_parse_common_thread_data(struct cpr3_thread *thread);
int cpr3_parse_common_ctrl_data(struct cpr3_controller *ctrl);
int cpr3_limit_open_loop_voltages(struct cpr3_regulator *vreg);
void cpr3_open_loop_voltage_as_ceiling(struct cpr3_regulator *vreg);
int cpr3_limit_floor_voltages(struct cpr3_regulator *vreg);
void cpr3_print_quots(struct cpr3_regulator *vreg);
int cpr3_adjust_fused_open_loop_voltages(struct cpr3_regulator *vreg,
			int *fuse_volt);
int cpr3_adjust_open_loop_voltages(struct cpr3_regulator *vreg);
int cpr3_quot_adjustment(int ro_scale, int volt_adjust);
int cpr3_voltage_adjustment(int ro_scale, int quot_adjust);
int cpr3_parse_closed_loop_voltage_adjustments(struct cpr3_regulator *vreg,
			u64 *ro_sel, int *volt_adjust,
			int *volt_adjust_fuse, int *ro_scale);
int cpr3_apm_init(struct cpr3_controller *ctrl);
int cpr3_mem_acc_init(struct cpr3_regulator *vreg);

#else

static inline int cpr3_regulator_register(struct platform_device *pdev,
			struct cpr3_controller *ctrl)
{
	return -ENXIO;
}

static inline int cpr3_regulator_unregister(struct cpr3_controller *ctrl)
{
	return -ENXIO;
}

static inline int cpr3_regulator_suspend(struct cpr3_controller *ctrl)
{
	return -ENXIO;
}

static inline int cpr3_regulator_resume(struct cpr3_controller *ctrl)
{
	return -ENXIO;
}

static inline int cpr3_get_thread_name(struct cpr3_thread *thread,
			struct device_node *thread_node)
{
	return -EPERM;
}

static inline int cpr3_allocate_threads(struct cpr3_controller *ctrl,
			u32 min_thread_id, u32 max_thread_id)
{
	return -EPERM;
}

static inline int cpr3_map_fuse_base(struct cpr3_controller *ctrl,
			struct platform_device *pdev)
{
	return -ENXIO;
}

static inline int cpr3_read_fuse_param(void __iomem *fuse_base_addr,
			const struct cpr3_fuse_param *param, u64 *param_value)
{
	return -EPERM;
}

static inline int cpr3_convert_open_loop_voltage_fuse(int ref_volt,
			int step_volt, u32 fuse, int fuse_len)
{
	return -EPERM;
}

static inline u64 cpr3_interpolate(u64 x1, u64 y1, u64 x2, u64 y2, u64 x)
{
	return 0;
}

static inline int cpr3_parse_array_property(struct cpr3_regulator *vreg,
			const char *prop_name, int tuple_size, u32 *out)
{
	return -EPERM;
}

static inline int cpr3_parse_corner_array_property(struct cpr3_regulator *vreg,
			const char *prop_name, int tuple_size, u32 *out)
{
	return -EPERM;
}

static inline int cpr3_parse_common_corner_data(struct cpr3_regulator *vreg)
{
	return -EPERM;
}

static inline int cpr3_parse_thread_u32(struct cpr3_thread *thread,
			const char *propname, u32 *out_value, u32 value_min,
			u32 value_max)
{
	return -EPERM;
}

static inline int cpr3_parse_ctrl_u32(struct cpr3_controller *ctrl,
			const char *propname, u32 *out_value, u32 value_min,
			u32 value_max)
{
	return -EPERM;
}

static inline int cpr3_parse_common_thread_data(struct cpr3_thread *thread)
{
	return -EPERM;
}

static inline int cpr3_parse_common_ctrl_data(struct cpr3_controller *ctrl)
{
	return -EPERM;
}

static inline int cpr3_limit_open_loop_voltages(struct cpr3_regulator *vreg)
{
	return -EPERM;
}

static inline void cpr3_open_loop_voltage_as_ceiling(
			struct cpr3_regulator *vreg)
{
	return;
}

static inline int cpr3_limit_floor_voltages(struct cpr3_regulator *vreg)
{
	return -EPERM;
}

static inline void cpr3_print_quots(struct cpr3_regulator *vreg)
{
	return;
}

static inline int cpr3_adjust_fused_open_loop_voltages(
			struct cpr3_regulator *vreg, int *fuse_volt)
{
	return -EPERM;
}

static inline int cpr3_adjust_open_loop_voltages(struct cpr3_regulator *vreg)
{
	return -EPERM;
}

static inline int cpr3_quot_adjustment(int ro_scale, int volt_adjust)
{
	return 0;
}

static inline int cpr3_voltage_adjustment(int ro_scale, int quot_adjust)
{
	return 0;
}

static inline int cpr3_parse_closed_loop_voltage_adjustments(
			struct cpr3_regulator *vreg, u64 *ro_sel,
			int *volt_adjust, int *volt_adjust_fuse, int *ro_scale)
{
	return 0;
}

static inline int cpr3_apm_init(struct cpr3_controller *ctrl)
{
	return 0;
}

static inline int cpr3_mem_acc_init(struct cpr3_regulator *vreg)
{
	return 0;
}

#endif 

#endif 
