/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_LOCAL_2_H
#define __ARCH_ARM_MACH_MSM_CLOCK_LOCAL_2_H

#include <linux/spinlock.h>
#include <linux/clk/msm-clk-provider.h>
#include <linux/clk/msm-clk.h>


struct clk_freq_tbl {
	unsigned long	freq_hz;
	unsigned long	src_freq;
	struct clk	*src_clk;
	u32	m_val;
	u32	n_val;
	u32	d_val;
	u32	div_src_val;
	const unsigned	sys_vdd;
};

#define FREQ_END	(ULONG_MAX-1)
#define F_END { .freq_hz = FREQ_END }
#define	FIXED_CLK_SRC	0
struct rcg_clk {
	u32 cmd_rcgr_reg;

	void   (*set_rate)(struct rcg_clk *, struct clk_freq_tbl *);

	struct clk_freq_tbl *freq_tbl;
	struct clk_freq_tbl *current_freq;
	struct clk	c;

	bool non_local_children;
	bool non_local_control;
	bool force_enable_rcgr;
	void *const __iomem *base;
};

static inline struct rcg_clk *to_rcg_clk(struct clk *clk)
{
	return container_of(clk, struct rcg_clk, c);
}

extern struct clk_freq_tbl rcg_dummy_freq;

struct branch_clk {
	void   (*set_rate)(struct branch_clk *, struct clk_freq_tbl *);
	struct clk c;
	u32 cbcr_reg;
	u32 bcr_reg;
	int has_sibling;
	u32 cur_div;
	u32 max_div;
	const u32 halt_check;
	bool toggle_memory;
	bool no_halt_check_on_disable;
	bool check_enable_bit;
	void *const __iomem *base;
};

static inline struct branch_clk *to_branch_clk(struct clk *clk)
{
	return container_of(clk, struct branch_clk, c);
}

struct local_vote_clk {
	struct clk c;
	u32 cbcr_reg;
	u32 vote_reg;
	u32 bcr_reg;
	u32 en_mask;
	const u32 halt_check;
	void * __iomem *base;
};

static inline struct local_vote_clk *to_local_vote_clk(struct clk *clk)
{
	return container_of(clk, struct local_vote_clk, c);
}

struct reset_clk {
	struct clk c;
	u32 reset_reg;
	void *__iomem *base;
};

static inline struct reset_clk *to_reset_clk(struct clk *clk)
{
	return container_of(clk, struct reset_clk, c);
}
struct measure_clk {
	u64 sample_ticks;
	u32 multiplier;
	u32 divider;

	struct clk c;
};

struct measure_clk_data {
	struct clk *cxo;
	u32 plltest_reg;
	u32 plltest_val;
	u32 xo_div4_cbcr;
	u32 ctl_reg;
	u32 status_reg;
	void *const __iomem *base;
};

static inline struct measure_clk *to_measure_clk(struct clk *clk)
{
	return container_of(clk, struct measure_clk, c);
}

struct gate_clk {
	struct clk c;
	u32 en_mask;
	u32 en_reg;
	unsigned int delay_us;
	void *const __iomem *base;
};

static inline struct gate_clk *to_gate_clk(struct clk *clk)
{
	return container_of(clk, struct gate_clk, c);
}

void set_rate_mnd(struct rcg_clk *clk, struct clk_freq_tbl *nf);
void set_rate_hid(struct rcg_clk *clk, struct clk_freq_tbl *nf);

extern spinlock_t local_clock_reg_lock;

extern struct clk_ops clk_ops_empty;
extern struct clk_ops clk_ops_rcg;
extern struct clk_ops clk_ops_rcg_mnd;
extern struct clk_ops clk_ops_branch;
extern struct clk_ops clk_ops_vote;
extern struct clk_ops clk_ops_rcg_hdmi;
extern struct clk_ops clk_ops_rcg_edp;
extern struct clk_ops clk_ops_byte;
extern struct clk_ops clk_ops_pixel;
extern struct clk_ops clk_ops_byte_multiparent;
extern struct clk_ops clk_ops_pixel_multiparent;
extern struct clk_ops clk_ops_edppixel;
extern struct clk_ops clk_ops_gate;
extern struct clk_ops clk_ops_rst;
extern struct clk_mux_ops mux_reg_ops;
extern struct mux_div_ops rcg_mux_div_ops;
extern struct clk_div_ops postdiv_reg_ops;

enum handoff pixel_rcg_handoff(struct clk *clk);
enum handoff byte_rcg_handoff(struct clk *clk);
unsigned long measure_get_rate(struct clk *c);

#define DEFINE_CLK_MEASURE(name) \
	struct clk name = { \
		.ops = &clk_ops_empty, \
		.dbg_name = #name, \
		CLK_INIT(name), \
	}; \

#endif 

