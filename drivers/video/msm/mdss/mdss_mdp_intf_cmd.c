/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/pm_runtime.h>

#include "mdss_mdp.h"
#include "mdss_panel.h"
#include "mdss_debug.h"
#include "mdss_mdp_trace.h"
#include "mdss_dsi_clk.h"

#define MAX_RECOVERY_TRIALS 10
#define MAX_SESSIONS 2

#define SPLIT_MIXER_OFFSET 0x800

#define STOP_TIMEOUT(hz) msecs_to_jiffies((1000 / hz) * (6 + 2))
#define POWER_COLLAPSE_TIME msecs_to_jiffies(100)
#define CMD_MODE_IDLE_TIMEOUT msecs_to_jiffies(16 * 4)
#define INPUT_EVENT_HANDLER_DELAY_USECS (16000 * 4)

static DEFINE_MUTEX(cmd_clk_mtx);

struct mdss_mdp_cmd_ctx {
	struct mdss_mdp_ctl *ctl;

	u32 default_pp_num;
	u32 current_pp_num;
	u32 aux_pp_num;

	u8 ref_cnt;
	struct completion stop_comp;
	struct completion readptr_done;
	struct completion pp_done;
	wait_queue_head_t pp_waitq;
	struct list_head vsync_handlers;
	int panel_power_state;
	atomic_t koff_cnt;
	u32 intf_stopped;
	struct mutex mdp_rdptr_lock;
	struct mutex clk_mtx;
	spinlock_t clk_lock;
	spinlock_t koff_lock;
	struct work_struct gate_clk_work;
	struct delayed_work delayed_off_clk_work;
	struct work_struct pp_done_work;
	struct work_struct early_wakeup_clk_work;
	struct mutex autorefresh_mtx;
	atomic_t pp_done_cnt;

	int autorefresh_pending_frame_cnt;
	bool autorefresh_off_pending;
	bool autorefresh_init;
	int vsync_irq_cnt;

	struct mdss_intf_recovery intf_recovery;
	struct mdss_mdp_cmd_ctx *sync_ctx; 
	u32 pp_timeout_report_cnt;
	bool pingpong_split_slave;

	u32 check_tepin;
};

struct mdss_mdp_cmd_ctx mdss_mdp_cmd_ctx_list[MAX_SESSIONS];

static int mdss_mdp_cmd_do_notifier(struct mdss_mdp_cmd_ctx *ctx);
static inline void mdss_mdp_cmd_clk_on(struct mdss_mdp_cmd_ctx *ctx);
static inline void mdss_mdp_cmd_clk_off(struct mdss_mdp_cmd_ctx *ctx);
static int mdss_mdp_cmd_wait4pingpong(struct mdss_mdp_ctl *ctl, void *arg);

static bool __mdss_mdp_cmd_is_aux_pp_needed(struct mdss_data_type *mdata,
	struct mdss_mdp_ctl *mctl)
{
	return (mdata && mctl && mctl->is_master &&
		mdss_has_quirk(mdata, MDSS_QUIRK_DSC_RIGHT_ONLY_PU) &&
		is_dsc_compression(&mctl->panel_data->panel_info) &&
		((mctl->mfd->split_mode == MDP_DUAL_LM_DUAL_DISPLAY) ||
		 ((mctl->mfd->split_mode == MDP_DUAL_LM_SINGLE_DISPLAY) &&
		  (mctl->panel_data->panel_info.dsc_enc_total == 1))) &&
		!mctl->mixer_left->valid_roi &&
		mctl->mixer_right->valid_roi);
}

static bool __mdss_mdp_cmd_is_panel_power_off(struct mdss_mdp_cmd_ctx *ctx)
{
	return mdss_panel_is_power_off(ctx->panel_power_state);
}

static bool __mdss_mdp_cmd_is_panel_power_on_interactive(
		struct mdss_mdp_cmd_ctx *ctx)
{
	return mdss_panel_is_power_on_interactive(ctx->panel_power_state);
}

static inline u32 mdss_mdp_cmd_line_count(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_mixer *mixer;
	u32 cnt = 0xffff;	
	u32 init;
	u32 height;

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);

	mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_LEFT);
	if (!mixer) {
		mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_RIGHT);
		if (!mixer) {
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
			goto exit;
		}
	}

	init = mdss_mdp_pingpong_read(mixer->pingpong_base,
		MDSS_MDP_REG_PP_VSYNC_INIT_VAL) & 0xffff;
	height = mdss_mdp_pingpong_read(mixer->pingpong_base,
		MDSS_MDP_REG_PP_SYNC_CONFIG_HEIGHT) & 0xffff;

	if (height < init) {
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
		goto exit;
	}

	cnt = mdss_mdp_pingpong_read(mixer->pingpong_base,
		MDSS_MDP_REG_PP_INT_COUNT_VAL) & 0xffff;

	if (cnt < init)		
		cnt += (height - init);
	else
		cnt -= init;

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);

	pr_debug("cnt=%d init=%d height=%d\n", cnt, init, height);
exit:
	return cnt;
}

static int mdss_mdp_tearcheck_enable(struct mdss_mdp_ctl *ctl, bool enable)
{
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	struct mdss_mdp_ctl *sctl;
	struct mdss_mdp_pp_tear_check *te;
	struct mdss_mdp_mixer *mixer =
		mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_LEFT);

	if (IS_ERR_OR_NULL(ctl->panel_data)) {
		pr_err("no panel data\n");
		return -ENODEV;
	}

	if (IS_ERR_OR_NULL(mixer)) {
		pr_err("mixer not configured\n");
		return -ENODEV;
	}

	sctl = mdss_mdp_get_split_ctl(ctl);
	te = &ctl->panel_data->panel_info.te;

	pr_debug("%s: enable=%d\n", __func__, enable);

	mdss_mdp_pingpong_write(mixer->pingpong_base,
		MDSS_MDP_REG_PP_TEAR_CHECK_EN,
		(te ? te->tear_check_en : 0) && enable);

	if (sctl) {
		mixer = mdss_mdp_mixer_get(sctl, MDSS_MDP_MIXER_MUX_LEFT);
		te = &sctl->panel_data->panel_info.te;
		mdss_mdp_pingpong_write(mixer->pingpong_base,
				MDSS_MDP_REG_PP_TEAR_CHECK_EN,
				(te ? te->tear_check_en : 0) && enable);
	}

	if (is_pingpong_split(ctl->mfd))
		mdss_mdp_pingpong_write(mdata->slave_pingpong_base,
				MDSS_MDP_REG_PP_TEAR_CHECK_EN,
				(te ? te->tear_check_en : 0) && enable);

	if (ctl->panel_data->panel_info.partial_update_enabled &&
	    is_dual_lm_single_display(ctl->mfd)) {

		mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_RIGHT);
		mdss_mdp_pingpong_write(mixer->pingpong_base,
				MDSS_MDP_REG_PP_TEAR_CHECK_EN,
				(te ? te->tear_check_en : 0) && enable);

	}

	return 0;
}

static int mdss_mdp_cmd_tearcheck_cfg(struct mdss_mdp_mixer *mixer,
		struct mdss_mdp_cmd_ctx *ctx, bool locked)
{
	struct mdss_mdp_pp_tear_check *te = NULL;
	struct mdss_panel_info *pinfo;
	u32 vsync_clk_speed_hz, total_lines, vclks_line, cfg = 0;
	char __iomem *pingpong_base;
	struct mdss_mdp_ctl *ctl = ctx->ctl;
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();

	if (IS_ERR_OR_NULL(ctl->panel_data)) {
		pr_err("no panel data\n");
		return -ENODEV;
	}

	pinfo = &ctl->panel_data->panel_info;
	te = &ctl->panel_data->panel_info.te;

	mdss_mdp_vsync_clk_enable(1, locked);

	vsync_clk_speed_hz =
		mdss_mdp_get_clk_rate(MDSS_CLK_MDP_VSYNC, locked);

	total_lines = mdss_panel_get_vtotal(pinfo);

	total_lines *= pinfo->mipi.frame_rate;

	vclks_line = (total_lines) ? vsync_clk_speed_hz/total_lines : 0;

	cfg = BIT(19);
	if (pinfo->mipi.hw_vsync_mode)
		cfg |= BIT(20);

	if (te->refx100) {
		vclks_line = vclks_line * pinfo->mipi.frame_rate *
			100 / te->refx100;
	} else {
		pr_warn("refx100 cannot be zero! Use 6000 as default\n");
		vclks_line = vclks_line * pinfo->mipi.frame_rate *
			100 / 6000;
	}

	cfg |= vclks_line;

	pr_debug("%s: yres=%d vclks=%x height=%d init=%d rd=%d start=%d\n",
		__func__, pinfo->yres, vclks_line, te->sync_cfg_height,
			te->vsync_init_val, te->rd_ptr_irq, te->start_pos);
	pr_debug("thrd_start =%d thrd_cont=%d\n",
		te->sync_threshold_start, te->sync_threshold_continue);

	pingpong_base = mixer->pingpong_base;

	if (ctx->pingpong_split_slave)
		pingpong_base = mdata->slave_pingpong_base;

	mdss_mdp_pingpong_write(pingpong_base,
		MDSS_MDP_REG_PP_SYNC_CONFIG_VSYNC, cfg);
	mdss_mdp_pingpong_write(pingpong_base,
		MDSS_MDP_REG_PP_SYNC_CONFIG_HEIGHT,
		te ? te->sync_cfg_height : 0);
	mdss_mdp_pingpong_write(pingpong_base,
		MDSS_MDP_REG_PP_VSYNC_INIT_VAL,
		te ? te->vsync_init_val : 0);
	mdss_mdp_pingpong_write(pingpong_base,
		MDSS_MDP_REG_PP_RD_PTR_IRQ,
		te ? te->rd_ptr_irq : 0);
	mdss_mdp_pingpong_write(pingpong_base,
		MDSS_MDP_REG_PP_START_POS,
		te ? te->start_pos : 0);
	mdss_mdp_pingpong_write(pingpong_base,
		MDSS_MDP_REG_PP_SYNC_THRESH,
		te ? ((te->sync_threshold_continue << 16) |
		 te->sync_threshold_start) : 0);
	mdss_mdp_pingpong_write(pingpong_base,
		MDSS_MDP_REG_PP_SYNC_WRCOUNT,
		te ? (te->start_pos + te->sync_threshold_start + 1) : 0);

	return 0;
}

static int mdss_mdp_cmd_tearcheck_setup(struct mdss_mdp_cmd_ctx *ctx,
		bool locked)
{
	int rc = 0;
	struct mdss_mdp_mixer *mixer;
	struct mdss_mdp_ctl *ctl = ctx->ctl;

	mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_LEFT);
	if (mixer) {
		rc = mdss_mdp_cmd_tearcheck_cfg(mixer, ctx, locked);
		if (rc)
			goto err;
	}

	if (ctl->panel_data->panel_info.partial_update_enabled &&
	    is_dual_lm_single_display(ctl->mfd)) {

		mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_RIGHT);
		if (mixer)
			rc = mdss_mdp_cmd_tearcheck_cfg(mixer, ctx, locked);
	}
err:
	return rc;
}

enum mdp_rsrc_ctl_events {
	MDP_RSRC_CTL_EVENT_KICKOFF = 1,
	MDP_RSRC_CTL_EVENT_PP_DONE,
	MDP_RSRC_CTL_EVENT_STOP,
	MDP_RSRC_CTL_EVENT_EARLY_WAKE_UP
};

enum {
	MDP_RSRC_CTL_STATE_OFF,
	MDP_RSRC_CTL_STATE_ON,
	MDP_RSRC_CTL_STATE_GATE,
};

static char *get_sw_event_name(u32 sw_event)
{
	switch (sw_event) {
	case MDP_RSRC_CTL_EVENT_KICKOFF:
		return "KICKOFF";
	case MDP_RSRC_CTL_EVENT_PP_DONE:
		return "PP_DONE";
	case MDP_RSRC_CTL_EVENT_STOP:
		return "STOP";
	case MDP_RSRC_CTL_EVENT_EARLY_WAKE_UP:
		return "EARLY_WAKE_UP";
	default:
		return "UNKNOWN";
	}
}

static char *get_clk_pwr_state_name(u32 pwr_state)
{
	switch (pwr_state) {
	case MDP_RSRC_CTL_STATE_ON:
		return "STATE_ON";
	case MDP_RSRC_CTL_STATE_OFF:
		return "STATE_OFF";
	case MDP_RSRC_CTL_STATE_GATE:
		return "STATE_GATE";
	default:
		return "UNKNOWN";
	}
}

int mdss_mdp_get_split_display_ctls(struct mdss_mdp_ctl **ctl,
	struct mdss_mdp_ctl **sctl)
{
	int rc = 0;
	*sctl = NULL;

	if (*ctl == NULL) {
		pr_err("%s invalid ctl\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	if ((*ctl)->mfd->split_mode == MDP_DUAL_LM_DUAL_DISPLAY) {
		*sctl = mdss_mdp_get_split_ctl(*ctl);
		if (*sctl) {
			
			pr_debug("%s ctls in correct order ctl:%d sctl:%d\n",
				__func__, (*ctl)->num, (*sctl)->num);
			goto exit;
		} else {
			*sctl = mdss_mdp_get_main_ctl(*ctl);
			if (!(*sctl)) {
				pr_err("%s cannot find master ctl\n",
					__func__);
				BUG();
			}
			pr_debug("ctl is not the master, swap pointers\n");
			swap(*ctl, *sctl);
		}
	} else {
		pr_debug("%s no split mode:%d\n", __func__,
			(*ctl)->mfd->split_mode);
	}
exit:
	return rc;
}

int mdss_mdp_resource_control(struct mdss_mdp_ctl *ctl, u32 sw_event)
{
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(ctl->mfd);
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	struct mdss_mdp_ctl *sctl = NULL;
	struct mdss_mdp_cmd_ctx *ctx, *sctx = NULL;
	struct dsi_panel_clk_ctrl clk_ctrl;
	u32 status;
	int rc = 0;
	bool schedule_off = false;

	
	mdss_mdp_get_split_display_ctls(&ctl, &sctl);

	ctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[MASTER_CTX];
	if (!ctx || !ctx->ctl ) {
		pr_err("%s invalid ctx\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	if (sctl)
		sctx = (struct mdss_mdp_cmd_ctx *) sctl->intf_ctx[MASTER_CTX];

	
	if (is_pingpong_split(ctl->mfd))
		sctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[SLAVE_CTX];

	pr_debug("%pS-->%s: task:%s ctl:%d pwr_state:%s event:%s\n",
		__builtin_return_address(0), __func__,
		current->group_leader->comm, ctl->num,
		get_clk_pwr_state_name(mdp5_data->resources_state),
		get_sw_event_name(sw_event));

	MDSS_XLOG(ctl->num, mdp5_data->resources_state, sw_event,
		XLOG_FUNC_ENTRY);

	switch (sw_event) {
	case MDP_RSRC_CTL_EVENT_KICKOFF:

		
		mdata->ao_bw_uc_idx = mdata->curr_bw_uc_idx;

		
		if (cancel_work_sync(&ctx->gate_clk_work)) {
			pr_debug("%s gate work canceled\n", __func__);

			if (mdp5_data->resources_state !=
					MDP_RSRC_CTL_STATE_ON)
				pr_debug("%s unexpected power state\n",
					__func__);
		}

		
		if (cancel_delayed_work_sync(&ctx->delayed_off_clk_work)) {
			pr_debug("%s off work canceled\n", __func__);

			if (mdp5_data->resources_state ==
					MDP_RSRC_CTL_STATE_OFF)
				pr_debug("%s unexpected OFF state\n",
					__func__);
		}

		mutex_lock(&ctl->rsrc_lock);
		MDSS_XLOG(ctl->num, mdp5_data->resources_state, sw_event, 0x11);
		
		if ((mdp5_data->resources_state == MDP_RSRC_CTL_STATE_OFF) ||
			(mdp5_data->resources_state ==
			MDP_RSRC_CTL_STATE_GATE)) {
			u32 flags = CTL_INTF_EVENT_FLAG_SKIP_BROADCAST;

			
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);

			clk_ctrl.state = MDSS_DSI_CLK_ON;
			clk_ctrl.client = DSI_CLK_REQ_MDP_CLIENT;
			mdss_mdp_ctl_intf_event 
				(ctx->ctl, MDSS_EVENT_PANEL_CLK_CTRL,
				(void *)&clk_ctrl, flags);

			if (sctx) { 
				if (sctx->pingpong_split_slave)
					flags |= CTL_INTF_EVENT_FLAG_SLAVE_INTF;

				mdss_mdp_ctl_intf_event(sctx->ctl,
					MDSS_EVENT_PANEL_CLK_CTRL,
					 (void *)&clk_ctrl, flags);
			}

			if (mdp5_data->resources_state ==
					MDP_RSRC_CTL_STATE_GATE)
				mdp5_data->resources_state =
					MDP_RSRC_CTL_STATE_ON;
		}

		
		if (mdp5_data->resources_state ==
				MDP_RSRC_CTL_STATE_OFF) {
			
			mdss_update_reg_bus_vote(mdata->reg_bus_clt,
				VOTE_INDEX_19_MHZ);

			
			mdss_mdp_cmd_clk_on(ctx);
			if (sctx)
				mdss_mdp_cmd_clk_on(sctx);

			mdp5_data->resources_state = MDP_RSRC_CTL_STATE_ON;
		}

		if (mdp5_data->resources_state != MDP_RSRC_CTL_STATE_ON) {
			
			pr_err("%s unexpected power state during:%s\n",
				__func__, get_sw_event_name(sw_event));
			BUG();
		}
		mutex_unlock(&ctl->rsrc_lock);
		break;
	case MDP_RSRC_CTL_EVENT_PP_DONE:
		if (mdp5_data->resources_state != MDP_RSRC_CTL_STATE_ON) {
			pr_err("%s unexpected power state during:%s\n",
				__func__, get_sw_event_name(sw_event));
			BUG();
		}

		
		 status = mdss_mdp_ctl_perf_get_transaction_status(ctl);

		if (sctl)
			status |= mdss_mdp_ctl_perf_get_transaction_status(
				sctl);

		if ((PERF_STATUS_DONE == status) &&
			!ctx->intf_stopped &&
			!ctx->autorefresh_init &&
			!ctl->mfd->validate_pending) {
			pr_debug("schedule release after:%d ms\n",
				jiffies_to_msecs
				(CMD_MODE_IDLE_TIMEOUT));

			MDSS_XLOG(ctl->num, mdp5_data->resources_state,
				sw_event, 0x22);

			
			if (mdata->enable_gate)
				schedule_work(&ctx->gate_clk_work);

			
			schedule_delayed_work(
					&ctx->delayed_off_clk_work,
					CMD_MODE_IDLE_TIMEOUT);
		}

		break;
	case MDP_RSRC_CTL_EVENT_STOP:

		
		mdss_mdp_cmd_wait4pingpong(ctl, NULL);

		if (sctl)
			mdss_mdp_cmd_wait4pingpong(sctl, NULL);

		
		if (cancel_work_sync(&ctx->early_wakeup_clk_work))
			pr_debug("early wakeup work canceled\n");


		
		if (cancel_work_sync(&ctx->gate_clk_work)) {
			pr_debug("gate work canceled\n");

			if (mdp5_data->resources_state !=
				MDP_RSRC_CTL_STATE_ON)
				pr_debug("%s power state is not ON\n",
					__func__);
		}

		
		if (cancel_delayed_work_sync(&ctx->delayed_off_clk_work)) {
			pr_debug("off work canceled\n");


			if (mdp5_data->resources_state ==
					MDP_RSRC_CTL_STATE_OFF)
				pr_debug("%s unexpected OFF state\n",
					__func__);
		}

		mutex_lock(&ctl->rsrc_lock);
		MDSS_XLOG(ctl->num, mdp5_data->resources_state, sw_event, 0x33);
		if ((mdp5_data->resources_state == MDP_RSRC_CTL_STATE_ON) ||
				(mdp5_data->resources_state
				== MDP_RSRC_CTL_STATE_GATE)) {

			
			if (mdp5_data->resources_state ==
					MDP_RSRC_CTL_STATE_GATE)
				mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);

			
			if (sctx)
				mdss_mdp_cmd_clk_off(sctx);

			
			mdss_mdp_cmd_clk_off(ctx);

			
			mdss_update_reg_bus_vote(mdata->reg_bus_clt,
				VOTE_INDEX_DISABLE);


			
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);

			
			mdp5_data->resources_state = MDP_RSRC_CTL_STATE_OFF;
		}
		mutex_unlock(&ctl->rsrc_lock);
		break;
	case MDP_RSRC_CTL_EVENT_EARLY_WAKE_UP:

		
		if ((ctx && __mdss_mdp_cmd_is_panel_power_off(ctx)) ||
			(sctx && __mdss_mdp_cmd_is_panel_power_off(sctx)))
			break;

		
		if (cancel_work_sync(&ctx->gate_clk_work)) {
			pr_debug("%s: %s - gate_work cancelled\n",
				 __func__, get_sw_event_name(sw_event));
			schedule_off = true;
		}

		
		if (cancel_delayed_work_sync(
				&ctx->delayed_off_clk_work)) {
			pr_debug("%s: %s - off work cancelled\n",
				 __func__, get_sw_event_name(sw_event));
			schedule_off = true;
		}

		mutex_lock(&ctl->rsrc_lock);
		MDSS_XLOG(ctl->num, mdp5_data->resources_state, sw_event,
			schedule_off, 0x44);
		if (mdp5_data->resources_state == MDP_RSRC_CTL_STATE_OFF) {
			u32 flags = CTL_INTF_EVENT_FLAG_SKIP_BROADCAST;

			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
			clk_ctrl.state = MDSS_DSI_CLK_ON;
			clk_ctrl.client = DSI_CLK_REQ_MDP_CLIENT;
			mdss_mdp_ctl_intf_event(ctx->ctl,
				MDSS_EVENT_PANEL_CLK_CTRL,
				(void *)&clk_ctrl, flags);

			if (sctx) { 
				if (sctx->pingpong_split_slave)
					flags |= CTL_INTF_EVENT_FLAG_SLAVE_INTF;

				mdss_mdp_ctl_intf_event(sctx->ctl,
					MDSS_EVENT_PANEL_CLK_CTRL,
					(void *)&clk_ctrl, flags);
			}

			mdss_mdp_cmd_clk_on(ctx);
			if (sctx)
				mdss_mdp_cmd_clk_on(sctx);

			mdp5_data->resources_state = MDP_RSRC_CTL_STATE_ON;
			schedule_off = true;
		}

		if (schedule_off) {
			schedule_delayed_work(&ctx->delayed_off_clk_work,
				      CMD_MODE_IDLE_TIMEOUT);
			pr_debug("off work scheduled\n");
		}
		mutex_unlock(&ctl->rsrc_lock);
		break;
	default:
		pr_warn("%s unexpected event (%d)\n", __func__, sw_event);
		break;
	}
	MDSS_XLOG(sw_event, mdp5_data->resources_state, XLOG_FUNC_EXIT);

exit:
	return rc;
}


static inline void mdss_mdp_cmd_clk_on(struct mdss_mdp_cmd_ctx *ctx)
{
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();

	pr_debug("%pS-->%s: task:%s ctx%d\n", __builtin_return_address(0),
		__func__, current->group_leader->comm, ctx->current_pp_num);

	mutex_lock(&ctx->clk_mtx);
	MDSS_XLOG(ctx->current_pp_num, atomic_read(&ctx->koff_cnt),
		mdata->bus_ref_cnt);

	mdss_bus_bandwidth_ctrl(true);

	mdss_mdp_hist_intr_setup(&mdata->hist_intr, MDSS_IRQ_RESUME);

	mutex_unlock(&ctx->clk_mtx);
}

static inline void mdss_mdp_cmd_clk_off(struct mdss_mdp_cmd_ctx *ctx)
{
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	struct dsi_panel_clk_ctrl clk_ctrl;

	if (ctx->autorefresh_init) {
		
		return;
	}

	pr_debug("%pS-->%s: task:%s ctx%d\n", __builtin_return_address(0),
		__func__, current->group_leader->comm, ctx->current_pp_num);

	mutex_lock(&ctx->clk_mtx);
	MDSS_XLOG(ctx->current_pp_num, atomic_read(&ctx->koff_cnt),
		mdata->bus_ref_cnt);

	mdss_mdp_hist_intr_setup(&mdata->hist_intr, MDSS_IRQ_SUSPEND);

	
	if (ctx->ctl) {
		u32 flags = CTL_INTF_EVENT_FLAG_SKIP_BROADCAST;

		if (ctx->pingpong_split_slave)
			flags |= CTL_INTF_EVENT_FLAG_SLAVE_INTF;

		clk_ctrl.state = MDSS_DSI_CLK_OFF;
		clk_ctrl.client = DSI_CLK_REQ_MDP_CLIENT;
		mdss_mdp_ctl_intf_event
			(ctx->ctl, MDSS_EVENT_PANEL_CLK_CTRL,
			(void *)&clk_ctrl, flags);
	} else {
		pr_err("OFF with ctl:NULL\n");
	}

	mdss_bus_bandwidth_ctrl(false);

	mutex_unlock(&ctx->clk_mtx);
}

static void mdss_mdp_cmd_readptr_done(void *arg)
{
	struct mdss_mdp_ctl *ctl = arg;
	struct mdss_mdp_cmd_ctx *ctx = ctl->intf_ctx[MASTER_CTX];
	struct mdss_mdp_vsync_handler *tmp;
	ktime_t vsync_time;
	u32 status;
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();

	if (!ctx) {
		pr_err("invalid ctx\n");
		return;
	}

	if (ctx->autorefresh_init || ctx->autorefresh_off_pending) {
		pr_debug("Completing read pointer done\n");
		complete_all(&ctx->readptr_done);
	}

	if (ctx->autorefresh_off_pending)
		ctx->autorefresh_off_pending = false;

	vsync_time = ktime_get();
	ctl->vsync_cnt++;
	status = readl_relaxed(mdata->mdp_base + MDSS_REG_HW_INTR2_STATUS);
	MDSS_XLOG(ctl->num, atomic_read(&ctx->koff_cnt), status);

	spin_lock(&ctx->clk_lock);
	list_for_each_entry(tmp, &ctx->vsync_handlers, list) {
		if (tmp->enabled && !tmp->cmd_post_flush &&
				!ctx->autorefresh_pending_frame_cnt)
			tmp->vsync_handler(ctl, vsync_time);
	}
	spin_unlock(&ctx->clk_lock);
}

static void mdss_mdp_cmd_intf_recovery(void *data, int event)
{
	struct mdss_mdp_cmd_ctx *ctx = data;
	unsigned long flags;
	bool reset_done = false, notify_frame_timeout = false;

	if (!data) {
		pr_err("%s: invalid ctx\n", __func__);
		return;
	}

	if (!ctx->ctl)
		return;

	if (event != MDP_INTF_DSI_CMD_FIFO_UNDERFLOW) {
		pr_warn("%s: unsupported recovery event:%d\n",
					__func__, event);
		return;
	}

	if (atomic_read(&ctx->koff_cnt)) {
		mdss_mdp_ctl_reset(ctx->ctl, true);
		reset_done = true;
	}

	spin_lock_irqsave(&ctx->koff_lock, flags);
	if (reset_done && atomic_add_unless(&ctx->koff_cnt, -1, 0)) {
		pr_debug("%s: intf_num=%d\n", __func__, ctx->ctl->intf_num);
		mdss_mdp_irq_disable_nosync(MDSS_MDP_IRQ_PING_PONG_COMP,
			ctx->current_pp_num);
		if (mdss_mdp_cmd_do_notifier(ctx))
			notify_frame_timeout = true;
	}
	spin_unlock_irqrestore(&ctx->koff_lock, flags);

	if (notify_frame_timeout)
		mdss_mdp_ctl_notify(ctx->ctl, MDP_NOTIFY_FRAME_TIMEOUT);

}

static void mdss_mdp_cmd_pingpong_done(void *arg)
{
	struct mdss_mdp_ctl *ctl = arg;
	struct mdss_mdp_cmd_ctx *ctx = ctl->intf_ctx[MASTER_CTX];
	struct mdss_mdp_vsync_handler *tmp;
	ktime_t vsync_time;
	bool sync_ppdone;

	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return;
	}

	mdss_mdp_ctl_perf_set_transaction_status(ctl,
		PERF_HW_MDP_STATE, PERF_STATUS_DONE);

	spin_lock(&ctx->clk_lock);
	list_for_each_entry(tmp, &ctx->vsync_handlers, list) {
		if (tmp->enabled && tmp->cmd_post_flush)
			tmp->vsync_handler(ctl, vsync_time);
	}
	spin_unlock(&ctx->clk_lock);

	spin_lock(&ctx->koff_lock);

	mdss_mdp_irq_disable_nosync(MDSS_MDP_IRQ_PING_PONG_COMP,
		ctx->current_pp_num);
	mdss_mdp_set_intr_callback_nosync(MDSS_MDP_IRQ_PING_PONG_COMP,
		ctx->current_pp_num, NULL, NULL);

	MDSS_XLOG(ctl->num, atomic_read(&ctx->koff_cnt), ctx->current_pp_num);

	sync_ppdone = mdss_mdp_cmd_do_notifier(ctx);

	if (atomic_add_unless(&ctx->koff_cnt, -1, 0)) {
		if (atomic_read(&ctx->koff_cnt))
			pr_err("%s: too many kickoffs=%d!\n", __func__,
			       atomic_read(&ctx->koff_cnt));
		if (sync_ppdone) {
			atomic_inc(&ctx->pp_done_cnt);
			schedule_work(&ctx->pp_done_work);

			mdss_mdp_resource_control(ctl,
				MDP_RSRC_CTL_EVENT_PP_DONE);
		}
		wake_up_all(&ctx->pp_waitq);
	} else if (!ctl->cmd_autorefresh_en) {
		pr_err("%s: should not have pingpong interrupt!\n", __func__);
	}

	pr_debug("%s: ctl_num=%d intf_num=%d ctx=%d cnt=%d\n", __func__,
			ctl->num, ctl->intf_num, ctx->current_pp_num,
			atomic_read(&ctx->koff_cnt));

	trace_mdp_cmd_pingpong_done(ctl, ctx->current_pp_num,
		atomic_read(&ctx->koff_cnt));

	spin_unlock(&ctx->koff_lock);
}

static void pingpong_done_work(struct work_struct *work)
{
	u32 status;
	struct mdss_mdp_cmd_ctx *ctx =
		container_of(work, typeof(*ctx), pp_done_work);

	if (ctx->ctl) {
		while (atomic_add_unless(&ctx->pp_done_cnt, -1, 0))
			mdss_mdp_ctl_notify(ctx->ctl, MDP_NOTIFY_FRAME_DONE);

		status = mdss_mdp_ctl_perf_get_transaction_status(ctx->ctl);
		if (status == 0)
			mdss_mdp_ctl_perf_release_bw(ctx->ctl);
	}
}

static void clk_ctrl_delayed_off_work(struct work_struct *work)
{
	struct mdss_overlay_private *mdp5_data;
	struct delayed_work *dw = to_delayed_work(work);
	struct mdss_mdp_cmd_ctx *ctx = container_of(dw,
		struct mdss_mdp_cmd_ctx, delayed_off_clk_work);
	struct mdss_mdp_ctl *ctl, *sctl;
	struct mdss_mdp_cmd_ctx *sctx = NULL;
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();

	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return;
	}

	ctl = ctx->ctl;
	if (!ctl || !ctl->panel_data) {
		pr_err("NULL ctl||panel_data\n");
		return;
	}

	mdp5_data = mfd_to_mdp5_data(ctl->mfd);
	ATRACE_BEGIN(__func__);

	if (mdata->enable_gate)
		flush_work(&ctx->gate_clk_work);

	pr_debug("ctl:%d pwr_state:%s\n", ctl->num,
		get_clk_pwr_state_name
		(mdp5_data->resources_state));

	mutex_lock(&ctl->rsrc_lock);
	MDSS_XLOG(ctl->num, mdp5_data->resources_state, XLOG_FUNC_ENTRY);

	if (ctl->mfd->split_mode == MDP_DUAL_LM_DUAL_DISPLAY) {
		mutex_lock(&cmd_clk_mtx);

		if (mdss_mdp_get_split_display_ctls(&ctl, &sctl)) {
			
			pr_err("cannot get both controllers for the split display\n");
			goto exit;
		}

		
		ctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[MASTER_CTX];
		sctx = (struct mdss_mdp_cmd_ctx *) sctl->intf_ctx[MASTER_CTX];
		if (!ctx || !sctx) {
			pr_err("invalid %s %s\n",
				ctx?"":"ctx", sctx?"":"sctx");
			goto exit;
		}
	} else if (is_pingpong_split(ctl->mfd)) {
		mutex_lock(&cmd_clk_mtx);
		sctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[SLAVE_CTX];
		if (!sctx) {
			pr_err("invalid sctx\n");
			goto exit;
		}
	}

	if (ctx->autorefresh_init) {
		pr_err("cannot disable with autorefresh\n");
		goto exit;
	}

	
	if (mdata->enable_gate && (mdp5_data->resources_state
			== MDP_RSRC_CTL_STATE_GATE))
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);

	
	if (sctx)
		mdss_mdp_cmd_clk_off(sctx);

	
	mdss_mdp_cmd_clk_off(ctx);

	
	mdss_update_reg_bus_vote(mdata->reg_bus_clt,
		VOTE_INDEX_DISABLE);

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);

	
	mdp5_data->resources_state = MDP_RSRC_CTL_STATE_OFF;

exit:
	
	if ((ctl->mfd->split_mode == MDP_DUAL_LM_DUAL_DISPLAY) ||
	    is_pingpong_split(ctl->mfd))
		mutex_unlock(&cmd_clk_mtx);

	MDSS_XLOG(ctl->num, mdp5_data->resources_state, XLOG_FUNC_EXIT);
	mutex_unlock(&ctl->rsrc_lock);

	ATRACE_END(__func__);
}

static void clk_ctrl_gate_work(struct work_struct *work)
{
	struct mdss_overlay_private *mdp5_data;
	struct mdss_mdp_cmd_ctx *ctx =
		container_of(work, typeof(*ctx), gate_clk_work);
	struct mdss_mdp_ctl *ctl, *sctl;
	struct mdss_mdp_cmd_ctx *sctx = NULL;
	struct dsi_panel_clk_ctrl clk_ctrl;

	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return;
	}

	ATRACE_BEGIN(__func__);
	ctl = ctx->ctl;
	if (!ctl) {
		pr_err("%s: invalid ctl\n", __func__);
		return;
	}

	mdp5_data = mfd_to_mdp5_data(ctl->mfd);
	if (!mdp5_data) {
		pr_err("%s: invalid mdp data\n", __func__);
		return;
	}

	pr_debug("%s ctl:%d pwr_state:%s\n", __func__,
		ctl->num, get_clk_pwr_state_name
		(mdp5_data->resources_state));

	mutex_lock(&ctl->rsrc_lock);
	MDSS_XLOG(ctl->num, mdp5_data->resources_state, XLOG_FUNC_ENTRY);


	if (ctl->mfd->split_mode == MDP_DUAL_LM_DUAL_DISPLAY) {
		mutex_lock(&cmd_clk_mtx);

		if (mdss_mdp_get_split_display_ctls(&ctl, &sctl)) {
			
			pr_err("%s cannot get both cts for the split display\n",
				__func__);
			goto exit;
		}

		
		ctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[MASTER_CTX];
		sctx = (struct mdss_mdp_cmd_ctx *) sctl->intf_ctx[MASTER_CTX];
		if (!ctx || !sctx) {
			pr_err("%s ERROR invalid %s %s\n", __func__,
				ctx?"":"ctx", sctx?"":"sctx");
			goto exit;
		}
	} else if (is_pingpong_split(ctl->mfd)) {
		mutex_lock(&cmd_clk_mtx);
		sctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[SLAVE_CTX];
		if (!sctx) {
			pr_err("invalid sctx\n");
			goto exit;
		}
	}

	if (ctx->autorefresh_init) {
		pr_err("cannot gate clocks with autorefresh\n");
		goto exit;
	}

	clk_ctrl.state = MDSS_DSI_CLK_EARLY_GATE;
	clk_ctrl.client = DSI_CLK_REQ_MDP_CLIENT;
	
	if (sctx) {
		u32 flags = CTL_INTF_EVENT_FLAG_SKIP_BROADCAST;

		if (sctx->pingpong_split_slave)
			flags |= CTL_INTF_EVENT_FLAG_SLAVE_INTF;

		mdss_mdp_ctl_intf_event(sctx->ctl,
			MDSS_EVENT_PANEL_CLK_CTRL,
			(void *)&clk_ctrl, flags);
	}

	
	mdss_mdp_ctl_intf_event
		(ctx->ctl, MDSS_EVENT_PANEL_CLK_CTRL,
		(void *)&clk_ctrl, CTL_INTF_EVENT_FLAG_SKIP_BROADCAST);

	
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);

	
	mdp5_data->resources_state = MDP_RSRC_CTL_STATE_GATE;

exit:
	
	if ((ctl->mfd->split_mode == MDP_DUAL_LM_DUAL_DISPLAY) ||
	    is_pingpong_split(ctl->mfd))
		mutex_unlock(&cmd_clk_mtx);

	MDSS_XLOG(ctl->num, mdp5_data->resources_state, XLOG_FUNC_EXIT);
	mutex_unlock(&ctl->rsrc_lock);

	ATRACE_END(__func__);
}

static int mdss_mdp_setup_vsync(struct mdss_mdp_cmd_ctx *ctx,
	bool enable)
{
	int changed = 0;

	mutex_lock(&ctx->mdp_rdptr_lock);

	if (enable) {
		if (ctx->vsync_irq_cnt == 0)
			changed++;
		ctx->vsync_irq_cnt++;
	} else {
		if (ctx->vsync_irq_cnt) {
			ctx->vsync_irq_cnt--;
			if (ctx->vsync_irq_cnt == 0)
				changed++;
		} else {
			pr_warn("%pS->%s: rd_ptr can not be turned off\n",
				__builtin_return_address(0), __func__);
		}
	}

	if (changed)
		MDSS_XLOG(ctx->vsync_irq_cnt, enable, current->pid);

	pr_debug("%pS->%s: vsync_cnt=%d changed=%d enable=%d ctl:%d pp:%d\n",
			__builtin_return_address(0), __func__,
			ctx->vsync_irq_cnt, changed, enable,
			ctx->ctl->num, ctx->default_pp_num);

	if (changed) {
		if (enable) {
			
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
			mdss_mdp_irq_enable(MDSS_MDP_IRQ_PING_PONG_RD_PTR,
				ctx->default_pp_num);
		} else {
			
			mdss_mdp_irq_disable(MDSS_MDP_IRQ_PING_PONG_RD_PTR,
				ctx->default_pp_num);
			mdss_mdp_intr_check_and_clear(
				MDSS_MDP_IRQ_PING_PONG_RD_PTR,
				ctx->default_pp_num);

			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
		}
	}

	mutex_unlock(&ctx->mdp_rdptr_lock);
	return ctx->vsync_irq_cnt;
}

static int mdss_mdp_cmd_add_vsync_handler(struct mdss_mdp_ctl *ctl,
		struct mdss_mdp_vsync_handler *handle)
{
	struct mdss_mdp_ctl *sctl = NULL;
	struct mdss_mdp_cmd_ctx *ctx, *sctx = NULL;
	unsigned long flags;
	bool enable_rdptr = false;
	int ret = 0;

	mutex_lock(&ctl->offlock);
	ctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[MASTER_CTX];
	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		ret = -ENODEV;
		goto done;
	}

	pr_debug("%pS->%s ctl:%d\n",
		__builtin_return_address(0), __func__, ctl->num);

	MDSS_XLOG(ctl->num, atomic_read(&ctx->koff_cnt));
	sctl = mdss_mdp_get_split_ctl(ctl);
	if (sctl)
		sctx = (struct mdss_mdp_cmd_ctx *) sctl->intf_ctx[MASTER_CTX];

	spin_lock_irqsave(&ctx->clk_lock, flags);
	if (!handle->enabled) {
		handle->enabled = true;
		list_add(&handle->list, &ctx->vsync_handlers);

		enable_rdptr = !handle->cmd_post_flush;
	}
	spin_unlock_irqrestore(&ctx->clk_lock, flags);

	if (enable_rdptr) {
		if (ctl->mfd->split_mode == MDP_DUAL_LM_DUAL_DISPLAY)
			mutex_lock(&cmd_clk_mtx);

		
		mdss_mdp_setup_vsync(ctx, true);

		if (ctl->mfd->split_mode == MDP_DUAL_LM_DUAL_DISPLAY)
			mutex_unlock(&cmd_clk_mtx);
	}

done:
	mutex_unlock(&ctl->offlock);

	return ret;
}

static int mdss_mdp_cmd_remove_vsync_handler(struct mdss_mdp_ctl *ctl,
		struct mdss_mdp_vsync_handler *handle)
{
	struct mdss_mdp_ctl *sctl;
	struct mdss_mdp_cmd_ctx *ctx, *sctx = NULL;
	unsigned long flags;
	bool disable_vsync_irq = false;

	ctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[MASTER_CTX];
	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return -ENODEV;
	}

	pr_debug("%pS->%s ctl:%d\n",
		__builtin_return_address(0), __func__, ctl->num);

	MDSS_XLOG(ctl->num, atomic_read(&ctx->koff_cnt), 0x88888);
	sctl = mdss_mdp_get_split_ctl(ctl);
	if (sctl)
		sctx = (struct mdss_mdp_cmd_ctx *) sctl->intf_ctx[MASTER_CTX];

	spin_lock_irqsave(&ctx->clk_lock, flags);
	if (handle->enabled) {
		handle->enabled = false;
		list_del_init(&handle->list);
		disable_vsync_irq = !handle->cmd_post_flush;
	}
	spin_unlock_irqrestore(&ctx->clk_lock, flags);

	if (disable_vsync_irq) {
		
		mdss_mdp_setup_vsync(ctx, false);
		complete(&ctx->stop_comp);
	}

	return 0;
}

int mdss_mdp_cmd_reconfigure_splash_done(struct mdss_mdp_ctl *ctl,
	bool handoff)
{
	struct mdss_panel_data *pdata;
	struct mdss_mdp_ctl *sctl = mdss_mdp_get_split_ctl(ctl);
	struct dsi_panel_clk_ctrl clk_ctrl;
	int ret = 0;

	pdata = ctl->panel_data;

	clk_ctrl.state = MDSS_DSI_CLK_OFF;
	clk_ctrl.client = DSI_CLK_REQ_MDP_CLIENT;
	if (sctl) {
		u32 flags = CTL_INTF_EVENT_FLAG_SKIP_BROADCAST;

		if (is_pingpong_split(sctl->mfd))
			flags |= CTL_INTF_EVENT_FLAG_SLAVE_INTF;

		mdss_mdp_ctl_intf_event(sctl, MDSS_EVENT_PANEL_CLK_CTRL,
			(void *)&clk_ctrl, flags);
	}

	mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_CLK_CTRL,
		(void *)&clk_ctrl, CTL_INTF_EVENT_FLAG_SKIP_BROADCAST);

	pdata->panel_info.cont_splash_enabled = 0;
	if (sctl)
		sctl->panel_data->panel_info.cont_splash_enabled = 0;
	else if (pdata->next && is_pingpong_split(ctl->mfd))
		pdata->next->panel_info.cont_splash_enabled = 0;

	return ret;
}

static int mdss_mdp_cmd_wait4pingpong(struct mdss_mdp_ctl *ctl, void *arg)
{
	struct mdss_mdp_cmd_ctx *ctx;
	struct mdss_panel_data *pdata;
	unsigned long flags;
	int rc = 0;
	u32 status;
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();

	ctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[MASTER_CTX];
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	pdata = ctl->panel_data;

	MDSS_XLOG(ctl->num, atomic_read(&ctx->koff_cnt), ctl->roi_bkup.w,
			ctl->roi_bkup.h);

	pr_debug("%s: intf_num=%d ctx=%p koff_cnt=%d\n", __func__,
			ctl->intf_num, ctx, atomic_read(&ctx->koff_cnt));

	rc = wait_event_timeout(ctx->pp_waitq,
			atomic_read(&ctx->koff_cnt) == 0,
			KOFF_TIMEOUT);

	trace_mdp_cmd_wait_pingpong(ctl->num,
				atomic_read(&ctx->koff_cnt));

	if (rc <= 0) {
		u32 status, mask;

		mask = BIT(MDSS_MDP_IRQ_PING_PONG_COMP + ctx->current_pp_num);
		status = mask & readl_relaxed(ctl->mdata->mdp_base +
				MDSS_MDP_REG_INTR_STATUS);
		if (status) {
			pr_warn("pp done but irq not triggered\n");
			mdss_mdp_irq_clear(ctl->mdata,
				MDSS_MDP_IRQ_PING_PONG_COMP,
				ctx->current_pp_num);
			local_irq_save(flags);
			mdss_mdp_cmd_pingpong_done(ctl);
			local_irq_restore(flags);
			rc = 1;
		}

		rc = atomic_read(&ctx->koff_cnt) == 0;
	}

	if (rc <= 0) {
		pr_err("%s: wait4pingpong timed out. ctl=%d rc=%d cnt=%d\n",
				__func__,
				ctl->num, rc, ctx->pp_timeout_report_cnt);
		if (ctx->pp_timeout_report_cnt == 0) {
			status = readl_relaxed(mdata->mdp_base +
				MDSS_REG_HW_INTR2_STATUS);
			pr_err("TE ISR status 0x%x\n", status);

			MDSS_XLOG(0xbad);
			MDSS_XLOG_TOUT_HANDLER("mdp", "dsi0_ctrl", "dsi0_phy",
				"dsi1_ctrl", "dsi1_phy", "vbif", "vbif_nrt",
				"dbg_bus", "vbif_dbg_bus");
		} else if (ctx->pp_timeout_report_cnt == MAX_RECOVERY_TRIALS) {
			MDSS_XLOG(0xbad2);
			MDSS_XLOG_TOUT_HANDLER("mdp", "dsi0_ctrl", "dsi0_phy",
				"dsi1_ctrl", "dsi1_phy", "vbif", "vbif_nrt",
				"dbg_bus", "vbif_dbg_bus", "panic");
			mdss_fb_report_panel_dead(ctl->mfd);
		}
		ctx->pp_timeout_report_cnt++;
		rc = -EPERM;
		if (atomic_add_unless(&ctx->koff_cnt, -1, 0)
			&& mdss_mdp_cmd_do_notifier(ctx))
			mdss_mdp_ctl_notify(ctl, MDP_NOTIFY_FRAME_TIMEOUT);

	} else {
		rc = 0;
		ctx->pp_timeout_report_cnt = 0;
	}

	cancel_work_sync(&ctx->pp_done_work);

	
	while (atomic_add_unless(&ctx->pp_done_cnt, -1, 0))
		mdss_mdp_ctl_notify(ctx->ctl, MDP_NOTIFY_FRAME_DONE);

	MDSS_XLOG(ctl->num, atomic_read(&ctx->koff_cnt), rc);

	return rc;
}

static int mdss_mdp_cmd_do_notifier(struct mdss_mdp_cmd_ctx *ctx)
{
	struct mdss_mdp_cmd_ctx *sctx;

	sctx = ctx->sync_ctx;
	if (!sctx || atomic_read(&sctx->koff_cnt) == 0)
		return 1;

	return 0;
}

static void mdss_mdp_cmd_set_sync_ctx(
		struct mdss_mdp_ctl *ctl, struct mdss_mdp_ctl *sctl)
{
	struct mdss_mdp_cmd_ctx *ctx, *sctx;

	ctx = (struct mdss_mdp_cmd_ctx *)ctl->intf_ctx[MASTER_CTX];

	if (!sctl) {
		ctx->sync_ctx = NULL;
		return;
	}

	sctx = (struct mdss_mdp_cmd_ctx *)sctl->intf_ctx[MASTER_CTX];

	if (!sctl->roi.w && !sctl->roi.h) {
		
		ctx->sync_ctx = NULL;
		sctx->sync_ctx = NULL;
	} else  {
		
		ctx->sync_ctx = sctx;
		sctx->sync_ctx = ctx;
	}
}

static void mdss_mdp_cmd_dsc_reconfig(struct mdss_mdp_ctl *ctl)
{
	struct mdss_panel_info *pinfo;
	bool changed = false;

	if (!ctl || !ctl->is_master)
		return;

	pinfo = &ctl->panel_data->panel_info;
	if (pinfo->compression_mode != COMPRESSION_DSC)
		return;

	changed = ctl->mixer_left->roi_changed;
	if (is_split_lm(ctl->mfd))
		changed |= ctl->mixer_right->roi_changed;

	if (changed)
		mdss_mdp_ctl_dsc_setup(ctl, pinfo);
}

static int mdss_mdp_cmd_set_partial_roi(struct mdss_mdp_ctl *ctl)
{
	int rc = -EINVAL;

	if (!ctl->panel_data->panel_info.partial_update_enabled)
		return rc;

	
	rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_ENABLE_PARTIAL_ROI,
				     NULL, CTL_INTF_EVENT_FLAG_DEFAULT);
	return rc;
}

static int mdss_mdp_cmd_set_stream_size(struct mdss_mdp_ctl *ctl)
{
	int rc = -EINVAL;

	if (!ctl->panel_data->panel_info.partial_update_enabled)
		return rc;

	
	rc = mdss_mdp_ctl_intf_event(ctl,
		MDSS_EVENT_DSI_STREAM_SIZE, NULL, CTL_INTF_EVENT_FLAG_DEFAULT);
	return rc;
}

static int mdss_mdp_cmd_panel_on(struct mdss_mdp_ctl *ctl,
	struct mdss_mdp_ctl *sctl)
{
	struct mdss_mdp_cmd_ctx *ctx, *sctx = NULL;
	int rc = 0;
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();

	ctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[MASTER_CTX];
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	if (sctl)
		sctx = (struct mdss_mdp_cmd_ctx *) sctl->intf_ctx[MASTER_CTX];

	
	if (is_pingpong_split(ctl->mfd))
		sctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[SLAVE_CTX];

	if (!__mdss_mdp_cmd_is_panel_power_on_interactive(ctx)) {
		if (ctl->pending_mode_switch != SWITCH_RESOLUTION) {
			rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_LINK_READY,
					NULL, CTL_INTF_EVENT_FLAG_DEFAULT);
			WARN(rc, "intf %d link ready error (%d)\n",
					ctl->intf_num, rc);

			rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_UNBLANK,
					NULL, CTL_INTF_EVENT_FLAG_DEFAULT);
			WARN(rc, "intf %d unblank error (%d)\n",
					ctl->intf_num, rc);

			rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_ON,
					NULL, CTL_INTF_EVENT_FLAG_DEFAULT);
			WARN(rc, "intf %d panel on error (%d)\n",
					ctl->intf_num, rc);

			rc = mdss_mdp_tearcheck_enable(ctl, true);
			WARN(rc, "intf %d tearcheck enable error (%d)\n",
					ctl->intf_num, rc);
		}

		ctx->panel_power_state = MDSS_PANEL_POWER_ON;
		if (sctx)
			sctx->panel_power_state = MDSS_PANEL_POWER_ON;

		mdss_mdp_ctl_intf_event(ctl,
			MDSS_EVENT_REGISTER_RECOVERY_HANDLER,
			(void *)&ctx->intf_recovery,
			CTL_INTF_EVENT_FLAG_DEFAULT);

		ctx->intf_stopped = 0;
		ctx->check_tepin = 5;
		pr_info("%s: INTR2 Status = 0x%x, %d\n", __func__,
			readl_relaxed(mdata->mdp_base + MDSS_REG_HW_INTR2_STATUS),
			ctx->check_tepin);
	} else {
		pr_err("%s: Panel already on\n", __func__);
	}

	return rc;
}

static int __mdss_mdp_cmd_configure_autorefresh(struct mdss_mdp_ctl *ctl, int
		frame_cnt, bool delayed)
{
	struct mdss_mdp_cmd_ctx *ctx;
	bool enable = frame_cnt ? true : false;

	if (!ctl || !ctl->mixer_left) {
		pr_err("invalid ctl structure\n");
		return -ENODEV;
	}
	ctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[MASTER_CTX];
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	if (frame_cnt == ctl->autorefresh_frame_cnt) {
		pr_debug("No change to the refresh count\n");
		return 0;
	}
	pr_debug("%s enable = %d frame_cnt = %d init=%d\n", __func__,
			enable, frame_cnt, ctx->autorefresh_init);

	mutex_lock(&ctx->autorefresh_mtx);

	if (enable) {
		if (delayed) {
			ctx->autorefresh_pending_frame_cnt = frame_cnt;
		} else {
			if (!ctx->autorefresh_init) {
				ctx->autorefresh_init = true;
			}

			mdss_mdp_pingpong_write(ctl->mixer_left->pingpong_base,
					MDSS_MDP_REG_PP_AUTOREFRESH_CONFIG,
					BIT(31) | frame_cnt);

			ctl->autorefresh_frame_cnt = frame_cnt;
			ctl->cmd_autorefresh_en = true;
		}
	} else {
		if (ctx->autorefresh_init) {

			mdss_mdp_pingpong_write(ctl->mixer_left->pingpong_base,
					MDSS_MDP_REG_PP_AUTOREFRESH_CONFIG, 0);
		}

		ctx->autorefresh_init = false;
		ctx->autorefresh_pending_frame_cnt = 0;
		ctx->autorefresh_off_pending = true;

		ctl->autorefresh_frame_cnt = 0;
		ctl->cmd_autorefresh_en = false;
	}

	mutex_unlock(&ctx->autorefresh_mtx);

	return 0;
}

int mdss_mdp_cmd_set_autorefresh_mode(struct mdss_mdp_ctl *ctl, int frame_cnt)
{
	return __mdss_mdp_cmd_configure_autorefresh(ctl, frame_cnt, true);
}

static int mdss_mdp_cmd_enable_cmd_autorefresh(struct mdss_mdp_ctl *ctl,
	int frame_cnt)
{
	return __mdss_mdp_cmd_configure_autorefresh(ctl, frame_cnt, false);
}

static int mdss_mdp_cmd_kickoff(struct mdss_mdp_ctl *ctl, void *arg)
{
	struct mdss_mdp_ctl *sctl = NULL, *mctl = ctl;
	struct mdss_mdp_cmd_ctx *ctx, *sctx = NULL;
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();

	ctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[MASTER_CTX];
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	if (ctx->intf_stopped) {
		pr_err("ctx=%d stopped already\n", ctx->current_pp_num);
		return -EPERM;
	}

	if (!ctl->is_master) {
		mctl = mdss_mdp_get_main_ctl(ctl);
	} else {
		sctl = mdss_mdp_get_split_ctl(ctl);
		if (sctl && (sctl->roi.w == 0 || sctl->roi.h == 0)) {
			
			sctl = NULL;
		}
	}

	reinit_completion(&ctx->readptr_done);
	mdss_mdp_ctl_perf_set_transaction_status(ctl,
		PERF_HW_MDP_STATE, PERF_STATUS_BUSY);

	if (sctl) {
		sctx = (struct mdss_mdp_cmd_ctx *) sctl->intf_ctx[MASTER_CTX];
		mdss_mdp_ctl_perf_set_transaction_status(sctl,
			PERF_HW_MDP_STATE, PERF_STATUS_BUSY);
	}

	if (__mdss_mdp_cmd_is_panel_power_off(ctx))
		mdss_mdp_cmd_panel_on(ctl, sctl);

	ctx->current_pp_num = ctx->default_pp_num;
	if (sctx)
		sctx->current_pp_num = sctx->default_pp_num;

	if (__mdss_mdp_cmd_is_aux_pp_needed(mdata, mctl))
		ctx->current_pp_num = ctx->aux_pp_num;

	MDSS_XLOG(ctl->num, ctx->current_pp_num,
		ctl->roi.x, ctl->roi.y, ctl->roi.w, ctl->roi.h);

	atomic_inc(&ctx->koff_cnt);
	if (sctx)
		atomic_inc(&sctx->koff_cnt);

	trace_mdp_cmd_kickoff(ctl->num, atomic_read(&ctx->koff_cnt));

	mdss_mdp_resource_control(ctl, MDP_RSRC_CTL_EVENT_KICKOFF);

	if (!ctl->is_master)
		mctl = mdss_mdp_get_main_ctl(ctl);
	mdss_mdp_cmd_dsc_reconfig(mctl);

	mdss_mdp_cmd_set_partial_roi(ctl);

	mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_DSI_CMDLIST_KOFF, NULL,
		CTL_INTF_EVENT_FLAG_DEFAULT);

	mdss_mdp_cmd_set_stream_size(ctl);

	mdss_mdp_cmd_set_sync_ctx(ctl, sctl);

	if (ctx->check_tepin > 0) {
		ctx->check_tepin--;
		pr_info("%s: INTR2 Status = 0x%x, %d\n", __func__,
			readl_relaxed(mdata->mdp_base + MDSS_REG_HW_INTR2_STATUS),
			ctx->check_tepin);
	}
	if (ctx->autorefresh_init || ctx->autorefresh_off_pending) {
		pr_debug("Wait for read pointer done before enabling PP irq\n");
		wait_for_completion(&ctx->readptr_done);
	}

	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_PING_PONG_COMP,
		ctx->current_pp_num, mdss_mdp_cmd_pingpong_done, ctl);
	mdss_mdp_irq_enable(MDSS_MDP_IRQ_PING_PONG_COMP, ctx->current_pp_num);
	if (sctx) {
		mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_PING_PONG_COMP,
			sctx->current_pp_num, mdss_mdp_cmd_pingpong_done, sctl);
		mdss_mdp_irq_enable(MDSS_MDP_IRQ_PING_PONG_COMP,
			sctx->current_pp_num);
	}

	mdss_mdp_ctl_perf_set_transaction_status(ctl,
		PERF_SW_COMMIT_STATE, PERF_STATUS_DONE);
	if (sctl) {
		mdss_mdp_ctl_perf_set_transaction_status(sctl,
			PERF_SW_COMMIT_STATE, PERF_STATUS_DONE);
	}

	writel_relaxed(0xffffffff, ctl->mdata->mdp_base + MDSS_REG_HW_INTR2_CLEAR);
	wmb();

	if (!ctx->autorefresh_pending_frame_cnt && !ctl->cmd_autorefresh_en) {
		
		mdss_mdp_ctl_write(ctl, MDSS_MDP_REG_CTL_START, 1);
	} else {
		pr_debug("Enabling autorefresh in hardware.\n");
		mdss_mdp_cmd_enable_cmd_autorefresh(ctl,
				ctx->autorefresh_pending_frame_cnt);
	}

	mb();
	MDSS_XLOG(ctl->num, ctx->current_pp_num,
		sctx ? sctx->current_pp_num : -1, atomic_read(&ctx->koff_cnt));
	return 0;
}

int mdss_mdp_cmd_restore(struct mdss_mdp_ctl *ctl, bool locked)
{
	struct mdss_mdp_cmd_ctx *ctx, *sctx = NULL;

	if (!ctl)
		return -EINVAL;

	pr_debug("%s: called for ctl%d\n", __func__, ctl->num);

	ctx = (struct mdss_mdp_cmd_ctx *)ctl->intf_ctx[MASTER_CTX];
	if (is_pingpong_split(ctl->mfd)) {
		sctx = (struct mdss_mdp_cmd_ctx *)ctl->intf_ctx[SLAVE_CTX];
	} else if (ctl->mfd->split_mode == MDP_DUAL_LM_DUAL_DISPLAY) {
		struct mdss_mdp_ctl *sctl = mdss_mdp_get_split_ctl(ctl);

		if (sctl)
			sctx = (struct mdss_mdp_cmd_ctx *)
					sctl->intf_ctx[MASTER_CTX];
	}

	if (mdss_mdp_cmd_tearcheck_setup(ctx, locked)) {
		pr_warn("%s: ctx%d tearcheck setup failed\n", __func__,
			ctx->current_pp_num);
	} else {
		if (sctx && mdss_mdp_cmd_tearcheck_setup(sctx, locked))
			pr_warn("%s: ctx%d tearcheck setup failed\n", __func__,
				sctx->current_pp_num);
		else
			mdss_mdp_tearcheck_enable(ctl, true);
	}

	return 0;
}

int mdss_mdp_cmd_ctx_stop(struct mdss_mdp_ctl *ctl,
		struct mdss_mdp_cmd_ctx *ctx, int panel_power_state)
{
	struct mdss_mdp_cmd_ctx *sctx = NULL;
	struct mdss_mdp_ctl *sctl = NULL;

	sctl = mdss_mdp_get_split_ctl(ctl);
	if (sctl)
		sctx = (struct mdss_mdp_cmd_ctx *) sctl->intf_ctx[MASTER_CTX];

	
	ctx->intf_stopped = 1;

	if (ctx->vsync_irq_cnt) {
		WARN(1, "vsync still enabled\n");
		while (mdss_mdp_setup_vsync(ctx, false))
			;
	}

	if (!ctl->pending_mode_switch) {
		mdss_mdp_ctl_intf_event(ctl,
			MDSS_EVENT_REGISTER_RECOVERY_HANDLER,
			NULL, CTL_INTF_EVENT_FLAG_DEFAULT);
	}

	
	mdss_mdp_resource_control(ctl, MDP_RSRC_CTL_EVENT_STOP);

	flush_work(&ctx->pp_done_work);
	mdss_mdp_tearcheck_enable(ctl, false);

	if (mdss_panel_is_power_on(panel_power_state)) {
		pr_debug("%s: intf stopped with panel on\n", __func__);
		return 0;
	}

	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_PING_PONG_RD_PTR,
		ctx->default_pp_num, NULL, NULL);

	memset(ctx, 0, sizeof(*ctx));

	return 0;
}

int mdss_mdp_cmd_intfs_stop(struct mdss_mdp_ctl *ctl, int session,
	int panel_power_state)
{
	struct mdss_mdp_cmd_ctx *ctx;

	if (session >= MAX_SESSIONS)
		return 0;

	ctx = ctl->intf_ctx[MASTER_CTX];
	if (!ctx->ref_cnt) {
		pr_err("invalid ctx session: %d\n", session);
		return -ENODEV;
	}

	mdss_mdp_cmd_ctx_stop(ctl, ctx, panel_power_state);

	if (is_pingpong_split(ctl->mfd)) {
		session += 1;

		if (session >= MAX_SESSIONS)
			return 0;

		ctx = ctl->intf_ctx[SLAVE_CTX];
		if (!ctx->ref_cnt) {
			pr_err("invalid ctx session: %d\n", session);
			return -ENODEV;
		}
		mdss_mdp_cmd_ctx_stop(ctl, ctx, panel_power_state);
	}
	pr_debug("%s:-\n", __func__);
	return 0;
}

static int mdss_mdp_cmd_stop_sub(struct mdss_mdp_ctl *ctl,
		int panel_power_state)
{
	struct mdss_mdp_cmd_ctx *ctx;
	struct mdss_mdp_vsync_handler *tmp, *handle;
	int session;

	ctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[MASTER_CTX];
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	list_for_each_entry_safe(handle, tmp, &ctx->vsync_handlers, list)
		mdss_mdp_cmd_remove_vsync_handler(ctl, handle);
	MDSS_XLOG(ctl->num, atomic_read(&ctx->koff_cnt), XLOG_FUNC_ENTRY);

	
	session = ctl->intf_num - MDSS_MDP_INTF1;
	return mdss_mdp_cmd_intfs_stop(ctl, session, panel_power_state);
}

int mdss_mdp_cmd_stop(struct mdss_mdp_ctl *ctl, int panel_power_state)
{
	struct mdss_mdp_cmd_ctx *ctx = ctl->intf_ctx[MASTER_CTX];
	struct mdss_mdp_ctl *sctl = mdss_mdp_get_split_ctl(ctl);
	bool panel_off = false;
	bool turn_off_clocks = false;
	bool send_panel_events = false;
	int ret = 0;

	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	if (__mdss_mdp_cmd_is_panel_power_off(ctx)) {
		pr_debug("%s: panel already off\n", __func__);
		return 0;
	}

	if (ctx->panel_power_state == panel_power_state) {
		pr_debug("%s: no transition needed %d --> %d\n", __func__,
			ctx->panel_power_state, panel_power_state);
		return 0;
	}

	pr_debug("%s: transition from %d --> %d\n", __func__,
		ctx->panel_power_state, panel_power_state);

	if (ctl->cmd_autorefresh_en) {
		int pre_suspend = ctx->autorefresh_pending_frame_cnt;
		int hz, rc;

		mdss_mdp_cmd_enable_cmd_autorefresh(ctl, 0);
		ctx->autorefresh_pending_frame_cnt = pre_suspend;

		reinit_completion(&ctx->readptr_done);

		
		mdss_mdp_setup_vsync(ctx, true);

		
		hz = mdss_panel_get_framerate(&ctl->panel_data->panel_info);
		rc = wait_for_completion_timeout(&ctx->readptr_done,
				STOP_TIMEOUT(hz));
		if (rc == 0)
			pr_err("Timed out waiting for read ptr!\n");

		mdss_mdp_setup_vsync(ctx, false);
	}

	mutex_lock(&ctl->offlock);
	if (mdss_panel_is_power_off(panel_power_state)) {
		
		send_panel_events = true;
		turn_off_clocks = true;
		panel_off = true;
	} else if (__mdss_mdp_cmd_is_panel_power_on_interactive(ctx)) {
		send_panel_events = true;
		if (mdss_panel_is_power_on_ulp(panel_power_state))
			turn_off_clocks = true;
	} else {
		
		if (mdss_panel_is_power_on_ulp(panel_power_state)) {
			pr_debug("%s: turn off clocks\n", __func__);
			turn_off_clocks = true;
		} else {
			pr_debug("%s: reset intf_stopped flag.\n", __func__);
			mdss_mdp_ctl_intf_event(ctl,
				MDSS_EVENT_REGISTER_RECOVERY_HANDLER,
				(void *)&ctx->intf_recovery,
				CTL_INTF_EVENT_FLAG_DEFAULT);
			ctx->intf_stopped = 0;
			goto end;
		}
	}

	if (!turn_off_clocks)
		goto panel_events;

	if (ctl->pending_mode_switch)
		send_panel_events = false;

	pr_debug("%s: turn off interface clocks\n", __func__);
	ret = mdss_mdp_cmd_stop_sub(ctl, panel_power_state);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s: unable to stop interface: %d\n",
				__func__, ret);
		goto end;
	}

	if (sctl) {
		mdss_mdp_cmd_stop_sub(sctl, panel_power_state);
		if (IS_ERR_VALUE(ret)) {
			pr_err("%s: unable to stop slave intf: %d\n",
					__func__, ret);
			goto end;
		}
	}

panel_events:
	if ((!is_panel_split(ctl->mfd) || is_pingpong_split(ctl->mfd) ||
	    (is_panel_split(ctl->mfd) && sctl)) && send_panel_events) {
		pr_debug("%s: send panel events\n", __func__);
		ret = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_BLANK,
				(void *) (long int) panel_power_state,
				CTL_INTF_EVENT_FLAG_DEFAULT);
		WARN(ret, "intf %d unblank error (%d)\n", ctl->intf_num, ret);

		ret = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_OFF,
				(void *) (long int) panel_power_state,
				CTL_INTF_EVENT_FLAG_DEFAULT);
		WARN(ret, "intf %d unblank error (%d)\n", ctl->intf_num, ret);
	}


	if (!panel_off) {
		pr_debug("%s: cmd_stop with panel always on\n", __func__);
		goto end;
	}

	pr_debug("%s: turn off panel\n", __func__);
	ctl->intf_ctx[MASTER_CTX] = NULL;
	ctl->intf_ctx[SLAVE_CTX] = NULL;
	ctl->ops.stop_fnc = NULL;
	ctl->ops.display_fnc = NULL;
	ctl->ops.wait_pingpong = NULL;
	ctl->ops.add_vsync_handler = NULL;
	ctl->ops.remove_vsync_handler = NULL;
	ctl->ops.reconfigure = NULL;

end:
	if (!IS_ERR_VALUE(ret)) {
		struct mdss_mdp_cmd_ctx *sctx = NULL;

		ctx->panel_power_state = panel_power_state;
		
		if (is_pingpong_split(ctl->mfd))
			sctx = (struct mdss_mdp_cmd_ctx *)
					ctl->intf_ctx[SLAVE_CTX];
		if (sctx)
			sctx->panel_power_state = panel_power_state;
	}

	MDSS_XLOG(ctl->num, atomic_read(&ctx->koff_cnt), XLOG_FUNC_EXIT);
	mutex_unlock(&ctl->offlock);
	pr_debug("%s:-\n", __func__);

	return ret;
}

static void early_wakeup_work(struct work_struct *work)
{
	int rc = 0;
	struct mdss_mdp_cmd_ctx *ctx =
		container_of(work, typeof(*ctx), early_wakeup_clk_work);
	struct mdss_mdp_ctl *ctl;

	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return;
	}

	ATRACE_BEGIN(__func__);
	ctl = ctx->ctl;

	if (!ctl) {
		pr_err("%s: invalid ctl\n", __func__);
		goto fail;
	}

	rc = mdss_mdp_resource_control(ctl, MDP_RSRC_CTL_EVENT_EARLY_WAKE_UP);
	if (rc)
		pr_err("%s: failed to control resources\n", __func__);

fail:
	ATRACE_END(__func__);
}

static int mdss_mdp_cmd_early_wake_up(struct mdss_mdp_ctl *ctl)
{
	u64 curr_time;
	struct mdss_mdp_cmd_ctx *ctx;

	curr_time = ktime_to_us(ktime_get());

	if ((curr_time - ctl->last_input_time) <
			INPUT_EVENT_HANDLER_DELAY_USECS)
		return 0;
	ctl->last_input_time = curr_time;

	ctx = (struct mdss_mdp_cmd_ctx *) ctl->intf_ctx[MASTER_CTX];
	if (ctx && !ctx->intf_stopped)
		schedule_work(&ctx->early_wakeup_clk_work);
	return 0;
}

static int mdss_mdp_cmd_ctx_setup(struct mdss_mdp_ctl *ctl,
	struct mdss_mdp_cmd_ctx *ctx, int default_pp_num, int aux_pp_num,
	bool pingpong_split_slave)
{
	int ret = 0;

	ctx->ctl = ctl;
	ctx->default_pp_num = default_pp_num;
	ctx->aux_pp_num = aux_pp_num;
	ctx->pingpong_split_slave = pingpong_split_slave;
	ctx->pp_timeout_report_cnt = 0;
	init_waitqueue_head(&ctx->pp_waitq);
	init_completion(&ctx->stop_comp);
	init_completion(&ctx->readptr_done);
	init_completion(&ctx->pp_done);
	spin_lock_init(&ctx->clk_lock);
	spin_lock_init(&ctx->koff_lock);
	mutex_init(&ctx->clk_mtx);
	mutex_init(&ctx->mdp_rdptr_lock);
	mutex_init(&ctx->autorefresh_mtx);
	INIT_WORK(&ctx->gate_clk_work, clk_ctrl_gate_work);
	INIT_DELAYED_WORK(&ctx->delayed_off_clk_work,
		clk_ctrl_delayed_off_work);
	INIT_WORK(&ctx->pp_done_work, pingpong_done_work);
	INIT_WORK(&ctx->early_wakeup_clk_work, early_wakeup_work);
	atomic_set(&ctx->pp_done_cnt, 0);
	ctx->autorefresh_off_pending = false;
	ctx->autorefresh_init = false;
	INIT_LIST_HEAD(&ctx->vsync_handlers);

	ctx->intf_recovery.fxn = mdss_mdp_cmd_intf_recovery;
	ctx->intf_recovery.data = ctx;

	ctx->intf_stopped = 0;

	pr_debug("%s: ctx=%p num=%d aux=%d\n", __func__, ctx,
		default_pp_num, aux_pp_num);
	MDSS_XLOG(ctl->num, atomic_read(&ctx->koff_cnt));

	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_PING_PONG_RD_PTR,
		ctx->default_pp_num, mdss_mdp_cmd_readptr_done, ctl);

	ret = mdss_mdp_cmd_tearcheck_setup(ctx, false);
	if (ret)
		pr_err("tearcheck setup failed\n");

	return ret;
}

static int mdss_mdp_cmd_intfs_setup(struct mdss_mdp_ctl *ctl,
			int session)
{
	struct mdss_mdp_cmd_ctx *ctx;
	struct mdss_mdp_ctl *sctl = NULL;
	struct mdss_mdp_mixer *mixer;
	int ret;
	u32 default_pp_num, aux_pp_num;

	if (session >= MAX_SESSIONS)
		return 0;

	sctl = mdss_mdp_get_split_ctl(ctl);
	ctx = &mdss_mdp_cmd_ctx_list[session];
	if (ctx->ref_cnt) {
		if (mdss_panel_is_power_on(ctx->panel_power_state)) {
			pr_debug("%s: cmd_start with panel always on\n",
				__func__);
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
			mdss_mdp_cmd_restore(ctl, false);
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);

			
			return mdss_mdp_cmd_panel_on(ctl, sctl);
		} else {
			pr_err("Intf %d already in use\n", session);
			return -EBUSY;
		}
	}
	ctx->ref_cnt++;
	ctl->intf_ctx[MASTER_CTX] = ctx;


	mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_LEFT);
	if (!mixer) {
		pr_err("mixer not setup correctly\n");
		return -ENODEV;
	}
	default_pp_num = mixer->num;

	if (is_split_lm(ctl->mfd)) {
		if (is_dual_lm_single_display(ctl->mfd)) {
			mixer = mdss_mdp_mixer_get(ctl,
				MDSS_MDP_MIXER_MUX_RIGHT);
			if (!mixer) {
				pr_err("right mixer not setup correctly for dual_lm_single_display\n");
				return -ENODEV;
			}
			aux_pp_num = mixer->num;
		} else { 
			struct mdss_mdp_ctl *mctl = ctl;

			if (!mctl->is_master) {
				mctl = mdss_mdp_get_main_ctl(ctl);
				if (!mctl) {
					pr_err("%s master ctl cannot be NULL\n",
						__func__);
					return -EINVAL;
				}
			}

			if (ctl->is_master) 
				mixer = mdss_mdp_mixer_get(mctl,
					MDSS_MDP_MIXER_MUX_RIGHT);
			else
				mixer = mdss_mdp_mixer_get(mctl,
					MDSS_MDP_MIXER_MUX_LEFT);

			if (!mixer) {
				pr_err("right mixer not setup correctly for dual_lm_dual_display\n");
				return -ENODEV;
			}
			aux_pp_num = mixer->num;
		}
	} else {
		aux_pp_num = default_pp_num;
	}

	ret = mdss_mdp_cmd_ctx_setup(ctl, ctx,
		default_pp_num, aux_pp_num, false);
	if (ret) {
		pr_err("mdss_mdp_cmd_ctx_setup failed for default_pp:%d aux_pp:%d\n",
			default_pp_num, aux_pp_num);
		ctx->ref_cnt--;
		return -ENODEV;
	}

	if (is_pingpong_split(ctl->mfd)) {
		session += 1;
		if (session >= MAX_SESSIONS)
			return 0;
		ctx = &mdss_mdp_cmd_ctx_list[session];
		if (ctx->ref_cnt) {
			if (mdss_panel_is_power_on(ctx->panel_power_state)) {
				pr_debug("%s: cmd_start with panel always on\n",
						__func__);
				mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
				mdss_mdp_cmd_restore(ctl, false);
				mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
				return mdss_mdp_cmd_panel_on(ctl, sctl);
			} else {
				pr_err("Intf %d already in use\n", session);
				return -EBUSY;
			}
		}
		ctx->ref_cnt++;

		ctl->intf_ctx[SLAVE_CTX] = ctx;

		ret = mdss_mdp_cmd_ctx_setup(ctl, ctx, session, session, true);
		if (ret) {
			pr_err("mdss_mdp_cmd_ctx_setup failed for slave ping pong block");
			ctx->ref_cnt--;
			return -EPERM;
		}
	}
	return 0;
}

void mdss_mdp_switch_roi_reset(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_ctl *mctl = ctl;
	struct mdss_mdp_ctl *sctl = mdss_mdp_get_split_ctl(ctl);

	if (!ctl->panel_data ||
	  !ctl->panel_data->panel_info.partial_update_supported)
		return;

	ctl->panel_data->panel_info.roi = ctl->roi;
	if (sctl && sctl->panel_data)
		sctl->panel_data->panel_info.roi = sctl->roi;

	if (!ctl->is_master)
		mctl = mdss_mdp_get_main_ctl(ctl);
	mdss_mdp_cmd_dsc_reconfig(mctl);

	mdss_mdp_cmd_set_partial_roi(ctl);
}

void mdss_mdp_switch_to_vid_mode(struct mdss_mdp_ctl *ctl, int prep)
{
	struct mdss_mdp_ctl *sctl = mdss_mdp_get_split_ctl(ctl);
	struct dsi_panel_clk_ctrl clk_ctrl;
	long int mode = MIPI_VIDEO_PANEL;

	pr_debug("%s start, prep = %d\n", __func__, prep);

	if (prep) {
		clk_ctrl.state = MDSS_DSI_CLK_ON;
		clk_ctrl.client = DSI_CLK_REQ_DSI_CLIENT;
		if (sctl)
			mdss_mdp_ctl_intf_event(sctl,
				MDSS_EVENT_PANEL_CLK_CTRL, (void *)&clk_ctrl,
				CTL_INTF_EVENT_FLAG_SKIP_BROADCAST);

		mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_CLK_CTRL,
			(void *)&clk_ctrl, CTL_INTF_EVENT_FLAG_SKIP_BROADCAST);

		return;
	}

	mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_DSI_RECONFIG_CMD,
			(void *) mode, CTL_INTF_EVENT_FLAG_DEFAULT);
}

static int mdss_mdp_cmd_reconfigure(struct mdss_mdp_ctl *ctl,
		enum dynamic_switch_modes mode, bool prep)
{
	struct dsi_panel_clk_ctrl clk_ctrl;
	int ret, rc = 0;

	if (mdss_mdp_ctl_is_power_off(ctl))
		return 0;

	pr_debug("%s: ctl=%d mode=%d prep=%d\n", __func__,
			ctl->num, mode, prep);

	if (mode == SWITCH_TO_VIDEO_MODE) {
		mdss_mdp_switch_to_vid_mode(ctl, prep);
	} else if (mode == SWITCH_RESOLUTION) {
		if (prep) {
			
			ret = mdss_mdp_cmd_wait4pingpong(ctl, NULL);
			if (ret)
				return ret;

			mdss_bus_bandwidth_ctrl(true);
			rc = mdss_iommu_ctrl(1);
			if (IS_ERR_VALUE(rc))
				pr_err("IOMMU attach failed\n");

			clk_ctrl.state = MDSS_DSI_CLK_ON;
			clk_ctrl.client = DSI_CLK_REQ_MDP_CLIENT;
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
			mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_CLK_CTRL,
					(void *)&clk_ctrl,
					CTL_INTF_EVENT_FLAG_DEFAULT);

			mdss_mdp_ctl_stop(ctl, MDSS_PANEL_POWER_OFF);
			mdss_mdp_ctl_intf_event(ctl,
					MDSS_EVENT_DSI_DYNAMIC_SWITCH,
					(void *) mode,
					CTL_INTF_EVENT_FLAG_DEFAULT);
		} else {
			
			clk_ctrl.state = MDSS_DSI_CLK_OFF;
			clk_ctrl.client = DSI_CLK_REQ_MDP_CLIENT;
			mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_CLK_CTRL,
					(void *)&clk_ctrl,
					CTL_INTF_EVENT_FLAG_DEFAULT);
			mdss_iommu_ctrl(0);
			mdss_bus_bandwidth_ctrl(false);
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
		}
	}

	return 0;
}

int mdss_mdp_cmd_start(struct mdss_mdp_ctl *ctl)
{
	int ret, session = 0;

	pr_debug("%s:+\n", __func__);

	
	session = ctl->intf_num - MDSS_MDP_INTF1;
	ret = mdss_mdp_cmd_intfs_setup(ctl, session);
	if (IS_ERR_VALUE(ret)) {
		pr_err("unable to set cmd interface: %d\n", ret);
		return ret;
	}

	ctl->ops.stop_fnc = mdss_mdp_cmd_stop;
	ctl->ops.display_fnc = mdss_mdp_cmd_kickoff;
	ctl->ops.wait_pingpong = mdss_mdp_cmd_wait4pingpong;
	ctl->ops.add_vsync_handler = mdss_mdp_cmd_add_vsync_handler;
	ctl->ops.remove_vsync_handler = mdss_mdp_cmd_remove_vsync_handler;
	ctl->ops.read_line_cnt_fnc = mdss_mdp_cmd_line_count;
	ctl->ops.restore_fnc = mdss_mdp_cmd_restore;
	ctl->ops.early_wake_up_fnc = mdss_mdp_cmd_early_wake_up;
	ctl->ops.reconfigure = mdss_mdp_cmd_reconfigure;
	pr_debug("%s:-\n", __func__);

	return 0;
}

