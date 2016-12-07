/* Copyright (c) 2008-2016, The Linux Foundation. All rights reserved.
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

#ifndef MDSS_PANEL_H
#define MDSS_PANEL_H

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/debugfs.h>

struct panel_id {
	u16 id;
	u16 type;
};

#define DEFAULT_FRAME_RATE	60
#define DEFAULT_ROTATOR_FRAME_RATE 120
#define ROTATOR_LOW_FRAME_RATE 30
#define MDSS_DSI_RST_SEQ_LEN	10
#define MDSS_MDP_MAX_PREFILL_FETCH 25

#define OVERRIDE_CFG	"override"
#define SIM_PANEL	"sim"
#define SIM_SW_TE_PANEL	"sim-swte"
#define SIM_HW_TE_PANEL	"sim-hwte"

#define NO_PANEL		0xffff	
#define MDDI_PANEL		1	
#define EBI2_PANEL		2	
#define LCDC_PANEL		3	
#define EXT_MDDI_PANEL		4	
#define TV_PANEL		5	
#define HDMI_PANEL		6	
#define DTV_PANEL		7	
#define MIPI_VIDEO_PANEL	8	
#define MIPI_CMD_PANEL		9	
#define WRITEBACK_PANEL		10	
#define LVDS_PANEL		11	
#define EDP_PANEL		12	

#define DSC_PPS_LEN		128

static inline const char *mdss_panel2str(u32 panel)
{
	static const char const *names[] = {
#define PANEL_NAME(n) [n ## _PANEL] = __stringify(n)
		PANEL_NAME(MIPI_VIDEO),
		PANEL_NAME(MIPI_CMD),
		PANEL_NAME(EDP),
		PANEL_NAME(HDMI),
		PANEL_NAME(DTV),
		PANEL_NAME(WRITEBACK),
#undef PANEL_NAME
	};

	if (panel >= ARRAY_SIZE(names) || !names[panel])
		return "UNKNOWN";

	return names[panel];
}

enum {
	DISPLAY_LCD = 0,	
	DISPLAY_LCDC,		
	DISPLAY_TV,		
	DISPLAY_EXT_MDDI,	
	DISPLAY_WRITEBACK,
};

enum {
	DISPLAY_1 = 0,		
	DISPLAY_2,		
	DISPLAY_3,		
	DISPLAY_4,		
	MAX_PHYS_TARGET_NUM,
};

enum {
	MDSS_PANEL_INTF_INVALID = -1,
	MDSS_PANEL_INTF_DSI,
	MDSS_PANEL_INTF_EDP,
	MDSS_PANEL_INTF_HDMI,
};

enum {
	MDSS_PANEL_POWER_OFF = 0,
	MDSS_PANEL_POWER_ON,
	MDSS_PANEL_POWER_LP1,
	MDSS_PANEL_POWER_LP2,
};

enum {
	MDSS_PANEL_BLANK_BLANK = 0,
	MDSS_PANEL_BLANK_UNBLANK,
	MDSS_PANEL_BLANK_LOW_POWER,
};

enum {
	MODE_GPIO_NOT_VALID = 0,
	MODE_GPIO_HIGH,
	MODE_GPIO_LOW,
};

enum {
	SIM_MODE = 1,
	SIM_SW_TE_MODE,
	SIM_HW_TE_MODE,
};

struct mdss_rect {
	u16 x;
	u16 y;
	u16 w;
	u16 h;
};

#define MDSS_MAX_PANEL_LEN      256
#define MDSS_INTF_MAX_NAME_LEN 5
#define MDSS_DISPLAY_ID_MAX_LEN 16
struct mdss_panel_intf {
	char name[MDSS_INTF_MAX_NAME_LEN];
	int  type;
};

struct mdss_panel_cfg {
	char arg_cfg[MDSS_MAX_PANEL_LEN + 1];
	int  pan_intf;
	bool lk_cfg;
	bool init_done;
};

#define MDP_INTF_DSI_CMD_FIFO_UNDERFLOW		0x0001
#define MDP_INTF_DSI_VIDEO_FIFO_OVERFLOW	0x0002


enum {
	MDP_INTF_CALLBACK_DSI_WAIT,
};

struct mdss_intf_recovery {
	void (*fxn)(void *ctx, int event);
	void *data;
};

enum mdss_intf_events {
	MDSS_EVENT_RESET = 1,
	MDSS_EVENT_LINK_READY,
	MDSS_EVENT_UNBLANK,
	MDSS_EVENT_PANEL_ON,
	MDSS_EVENT_POST_PANEL_ON,
	MDSS_EVENT_BLANK,
	MDSS_EVENT_PANEL_OFF,
	MDSS_EVENT_CLOSE,
	MDSS_EVENT_SUSPEND,
	MDSS_EVENT_RESUME,
	MDSS_EVENT_CHECK_PARAMS,
	MDSS_EVENT_CONT_SPLASH_BEGIN,
	MDSS_EVENT_CONT_SPLASH_FINISH,
	MDSS_EVENT_PANEL_UPDATE_FPS,
	MDSS_EVENT_FB_REGISTERED,
	MDSS_EVENT_PANEL_CLK_CTRL,
	MDSS_EVENT_DSI_CMDLIST_KOFF,
	MDSS_EVENT_ENABLE_PARTIAL_ROI,
	MDSS_EVENT_DSC_PPS_SEND,
	MDSS_EVENT_DSI_STREAM_SIZE,
	MDSS_EVENT_DSI_UPDATE_PANEL_DATA,
	MDSS_EVENT_REGISTER_RECOVERY_HANDLER,
	MDSS_EVENT_REGISTER_MDP_CALLBACK,
	MDSS_EVENT_DSI_PANEL_STATUS,
	MDSS_EVENT_DSI_DYNAMIC_SWITCH,
	MDSS_EVENT_DSI_RECONFIG_CMD,
	MDSS_EVENT_DSI_RESET_WRITE_PTR,
	MDSS_EVENT_PANEL_TIMING_SWITCH,
	MDSS_EVENT_PANEL_VDDIO_SWITCH_ON,
	MDSS_EVENT_PANEL_VDDIO_SWITCH_OFF,
	MDSS_EVENT_MAX,
};

struct lcd_panel_info {
	u32 h_back_porch;
	u32 h_front_porch;
	u32 h_pulse_width;
	u32 v_back_porch;
	u32 v_front_porch;
	u32 v_pulse_width;
	u32 border_clr;
	u32 underflow_clr;
	u32 hsync_skew;
	u32 border_top;
	u32 border_bottom;
	u32 border_left;
	u32 border_right;
	
	u32 xres_pad;
	
	u32 yres_pad;
	u32 frame_rate;
};


struct mdss_dsi_phy_ctrl {
	char regulator[7];	
	char timing[12];
	char ctrl[4];
	char strength[10];	
	char bistctrl[6];
	uint32_t pll[21];
	char lanecfg[45];	
	bool reg_ldo_mode;

	char timing_8996[40];
	char regulator_len;
	char strength_len;
	char lanecfg_len;
};

enum dynamic_mode_switch {
	DYNAMIC_MODE_SWITCH_DISABLED = 0,
	DYNAMIC_MODE_SWITCH_SUSPEND_RESUME,
	DYNAMIC_MODE_SWITCH_IMMEDIATE,
	DYNAMIC_MODE_RESOLUTION_SWITCH_IMMEDIATE,
};

enum dynamic_switch_modes {
	SWITCH_MODE_UNKNOWN = 0,
	SWITCH_TO_CMD_MODE,
	SWITCH_TO_VIDEO_MODE,
	SWITCH_RESOLUTION,
};

struct mipi_panel_info {
	char boot_mode;	
	char mode;		
	char interleave_mode;
	char crc_check;
	char ecc_check;
	char dst_format;	
	char data_lane0;
	char data_lane1;
	char data_lane2;
	char data_lane3;
	char rgb_swap;
	char b_sel;
	char g_sel;
	char r_sel;
	char rx_eot_ignore;
	char tx_eot_append;
	char t_clk_post; 
	char t_clk_pre;  
	char vc;	
	struct mdss_dsi_phy_ctrl dsi_phy_db;
	
	char pulse_mode_hsa_he;
	char hfp_power_stop;
	char hbp_power_stop;
	char hsa_power_stop;
	char eof_bllp_power_stop;
	char last_line_interleave_en;
	char bllp_power_stop;
	char traffic_mode;
	char frame_rate;
	
	char interleave_max;
	char insert_dcs_cmd;
	char wr_mem_continue;
	char wr_mem_start;
	char te_sel;
	char stream;	
	char mdp_trigger;
	char dma_trigger;
	
	enum dynamic_mode_switch dms_mode;

	u32 pixel_packing;
	u32 dsi_pclk_rate;
	
	char no_max_pkt_size;
	
	bool force_clk_lane_hs;

	char vsync_enable;
	char hw_vsync_mode;

	char lp11_init;
	u32  init_delay;
	u32  post_init_delay;
};

struct edp_panel_info {
	char frame_rate;	
};

struct dynamic_fps_data {
	u32 hfp;
	u32 hbp;
	u32 hpw;
	u32 clk_rate;
	u32 fps;
};

enum dynamic_fps_update {
	DFPS_SUSPEND_RESUME_MODE,
	DFPS_IMMEDIATE_CLK_UPDATE_MODE,
	DFPS_IMMEDIATE_PORCH_UPDATE_MODE_VFP,
	DFPS_IMMEDIATE_PORCH_UPDATE_MODE_HFP,
	DFPS_IMMEDIATE_MULTI_UPDATE_MODE_CLK_HFP,
	DFPS_IMMEDIATE_MULTI_MODE_HFP_CALC_CLK,
	DFPS_MODE_MAX
};

enum lvds_mode {
	LVDS_SINGLE_CHANNEL_MODE,
	LVDS_DUAL_CHANNEL_MODE,
};

struct lvds_panel_info {
	enum lvds_mode channel_mode;
	
	char channel_swap;
};

enum {
	COMPRESSION_NONE,
	COMPRESSION_DSC,
	COMPRESSION_FBC
};

struct dsc_desc {
	u8 version; 
	u8 scr_rev; 

	int pic_height;
	int pic_width;
	int initial_lines;

	int pkt_per_line;
	int bytes_in_slice;
	int bytes_per_pkt;
	int eol_byte_num;
	int pclk_per_line;	

	int full_frame_slices; 
	int slice_height;
	int slice_width;
	int chunk_size;

	int slice_last_group_size;
	int bpp;	
	int bpc;	
	int line_buf_depth;
	bool config_by_manufacture_cmd;
	bool block_pred_enable;
	int vbr_enable;
	int enable_422;
	int convert_rgb;
	int input_10_bits;
	int slice_per_pkt;

	int initial_dec_delay;
	int initial_xmit_delay;

	int initial_scale_value;
	int scale_decrement_interval;
	int scale_increment_interval;

	int first_line_bpg_offset;
	int nfl_bpg_offset;
	int slice_bpg_offset;

	int initial_offset;
	int final_offset;

	int rc_model_size;	

	int det_thresh_flatness;
	int max_qp_flatness;
	int min_qp_flatness;
	int edge_factor;
	int quant_incr_limit0;
	int quant_incr_limit1;
	int tgt_offset_hi;
	int tgt_offset_lo;
	u32 *buf_thresh;
	char *range_min_qp;
	char *range_max_qp;
	char *range_bpg_offset;
};

struct fbc_panel_info {
	u32 enabled;
	u32 target_bpp;
	u32 comp_mode;
	u32 qerr_enable;
	u32 cd_bias;
	u32 pat_enable;
	u32 vlc_enable;
	u32 bflc_enable;

	u32 line_x_budget;
	u32 block_x_budget;
	u32 block_budget;

	u32 lossless_mode_thd;
	u32 lossy_mode_thd;
	u32 lossy_rgb_thd;
	u32 lossy_mode_idx;

	u32 slice_height;
	bool pred_mode;
	bool enc_mode;
	u32 max_pred_err;
};

struct mdss_mdp_pp_tear_check {
	u32 tear_check_en;
	u32 sync_cfg_height;
	u32 vsync_init_val;
	u32 sync_threshold_start;
	u32 sync_threshold_continue;
	u32 start_pos;
	u32 rd_ptr_irq;
	u32 wr_ptr_irq;
	u32 refx100;
};

struct mdss_panel_roi_alignment {
	u32 xstart_pix_align;
	u32 width_pix_align;
	u32 ystart_pix_align;
	u32 height_pix_align;
	u32 min_width;
	u32 min_height;
};

struct htc_backlight1_table {
	int size;
	u16 *brt_data;
	u16 *bl_data;
};

enum {
	PANEL_POWER_CTRL_DEFAULT,
	PANEL_POWER_CTRL_HX8396C2,
};

struct mdss_panel_info {
	u32 xres;
	u32 yres;
	u32 physical_width;
	u32 physical_height;
	u32 bpp;
	u32 type;
	u32 wait_cycle;
	u32 pdest;
	u32 brightness_max;
	u32 bl_max;
	u32 bl_min;
	u32 fb_num;
	u64 clk_rate;
	u32 clk_min;
	u64 clk_max;
	u32 mdp_transfer_time_us;
	u32 frame_count;
	u32 is_3d_panel;
	u32 out_format;
	u32 rst_seq[MDSS_DSI_RST_SEQ_LEN];
	u32 rst_seq_len;
	u32 vic; 
	struct mdss_rect roi;
	int pwm_pmic_gpio;
	int pwm_lpg_chan;
	int pwm_period;
	bool dynamic_fps;
	bool ulps_feature_enabled;
	bool ulps_suspend_enabled;
	bool panel_ack_disabled;
	bool esd_check_enabled;
	bool allow_phy_power_off;
	char dfps_update;
	
	int new_fps;
	
	u32 default_fps;
	
	u32 saved_total;
	
	u32 saved_fporch;
	
	int current_fps;

	int panel_max_fps;
	int panel_max_vtotal;
	u32 mode_gpio_state;
	u32 min_fps;
	u32 max_fps;
	u32 prg_fet;
	struct mdss_panel_roi_alignment roi_alignment;

	u32 cont_splash_enabled;
	bool esd_rdy;
	bool partial_update_supported; 
	bool partial_update_enabled; 
	u32 dcs_cmd_by_left;
	u32 partial_update_roi_merge;
	struct ion_handle *splash_ihdl;
	int panel_power_state;
	int compression_mode;

	uint32_t panel_dead;
	u32 panel_force_dead;
	u32 panel_orientation;
	bool dynamic_switch_pending;
	bool is_lpm_mode;
	bool is_split_display; 
	bool use_pingpong_split;

	u32 lm_widths[2];

	bool is_prim_panel;
	bool is_pluggable;
	char display_id[MDSS_DISPLAY_ID_MAX_LEN];
	bool is_cec_supported;

	
	u8 sim_panel_mode;

	void *edid_data;
	void *dba_data;
	void *cec_data;

	char panel_name[MDSS_MAX_PANEL_LEN];
	struct mdss_mdp_pp_tear_check te;

	u8 dsc_enc_total; 
	struct dsc_desc dsc;

	bool send_pps_before_switch;

	struct lcd_panel_info lcdc;
	struct fbc_panel_info fbc;
	struct mipi_panel_info mipi;
	struct lvds_panel_info lvds;
	struct edp_panel_info edp;

	bool is_dba_panel;

	u32 adjust_timer_delay_ms;

	
	struct mdss_panel_debugfs_info *debugfs_info;

	
	struct htc_backlight1_table brt_bl_table;
	int camera_blk;
	int power_ctrl;
};

struct mdss_panel_timing {
	struct list_head list;
	const char *name;

	u32 xres;
	u32 yres;

	u32 h_back_porch;
	u32 h_front_porch;
	u32 h_pulse_width;
	u32 hsync_skew;
	u32 v_back_porch;
	u32 v_front_porch;
	u32 v_pulse_width;

	u32 border_top;
	u32 border_bottom;
	u32 border_left;
	u32 border_right;

	u32 lm_widths[2];

	u64 clk_rate;
	char frame_rate;

	u8 dsc_enc_total;
	struct dsc_desc dsc;
	struct fbc_panel_info fbc;
	u32 compression_mode;

	struct mdss_mdp_pp_tear_check te;
	struct mdss_panel_roi_alignment roi_alignment;
};

struct mdss_panel_data {
	struct mdss_panel_info panel_info;
	void (*set_backlight) (struct mdss_panel_data *pdata, u32 bl_level);
	unsigned char *mmss_cc_base;

	int (*event_handler) (struct mdss_panel_data *pdata, int e, void *arg);
	struct device_node *(*get_fb_node)(struct platform_device *pdev);

	struct list_head timings_list;
	struct mdss_panel_timing *current_timing;
	bool active;

	
	char dsc_cfg_np_name[MDSS_MAX_PANEL_LEN];
	struct mdss_panel_data *next;
};

struct mdss_panel_debugfs_info {
	struct dentry *root;
	struct dentry *parent;
	struct mdss_panel_info panel_info;
	u32 override_flag;
	struct mdss_panel_debugfs_info *next;
};

static inline u32 mdss_panel_get_framerate(struct mdss_panel_info *panel_info)
{
	u32 frame_rate, pixel_total;
	u64 rate;

	if (panel_info == NULL)
		return DEFAULT_FRAME_RATE;

	switch (panel_info->type) {
	case MIPI_VIDEO_PANEL:
	case MIPI_CMD_PANEL:
		frame_rate = panel_info->mipi.frame_rate;
		break;
	case EDP_PANEL:
		frame_rate = panel_info->edp.frame_rate;
		break;
	case WRITEBACK_PANEL:
		frame_rate = DEFAULT_FRAME_RATE;
		break;
	case DTV_PANEL:
		if (panel_info->dynamic_fps) {
			frame_rate = panel_info->lcdc.frame_rate / 1000;
			if (panel_info->lcdc.frame_rate % 1000)
				frame_rate += 1;
			break;
		}
	default:
		pixel_total = (panel_info->lcdc.h_back_porch +
			  panel_info->lcdc.h_front_porch +
			  panel_info->lcdc.h_pulse_width +
			  panel_info->xres) *
			 (panel_info->lcdc.v_back_porch +
			  panel_info->lcdc.v_front_porch +
			  panel_info->lcdc.v_pulse_width +
			  panel_info->yres);
		if (pixel_total) {
			rate = panel_info->clk_rate;
			do_div(rate, pixel_total);
			frame_rate = (u32)rate;
		} else {
			frame_rate = DEFAULT_FRAME_RATE;
		}
		break;
	}
	return frame_rate;
}

static inline int mdss_panel_get_vtotal(struct mdss_panel_info *pinfo)
{
	return pinfo->yres + pinfo->lcdc.v_back_porch +
			pinfo->lcdc.v_front_porch +
			pinfo->lcdc.v_pulse_width+
			pinfo->lcdc.border_top +
			pinfo->lcdc.border_bottom;
}

static inline int mdss_panel_get_htotal(struct mdss_panel_info *pinfo, bool
		compression)
{
	struct dsc_desc *dsc = NULL;

	int adj_xres = pinfo->xres + pinfo->lcdc.border_left +
				pinfo->lcdc.border_right;

	if (compression) {
		if (pinfo->compression_mode == COMPRESSION_DSC) {
			dsc = &pinfo->dsc;
			adj_xres = dsc->pclk_per_line;
		} else if (pinfo->fbc.enabled) {
			adj_xres = mult_frac(adj_xres,
				pinfo->fbc.target_bpp, pinfo->bpp);
		}
	}

	return adj_xres + pinfo->lcdc.h_back_porch +
		pinfo->lcdc.h_front_porch +
		pinfo->lcdc.h_pulse_width;
}

static inline bool is_dsc_compression(struct mdss_panel_info *pinfo)
{
	if (pinfo)
		return (pinfo->compression_mode == COMPRESSION_DSC);

	return false;
}

int mdss_register_panel(struct platform_device *pdev,
	struct mdss_panel_data *pdata);

static inline bool mdss_panel_is_power_off(int panel_power_state)
{
	return (panel_power_state == MDSS_PANEL_POWER_OFF);
}

static inline bool mdss_panel_is_power_on_interactive(int panel_power_state)
{
	return (panel_power_state == MDSS_PANEL_POWER_ON);
}

static inline bool mdss_panel_is_power_on(int panel_power_state)
{
	return !mdss_panel_is_power_off(panel_power_state);
}

static inline bool mdss_panel_is_power_on_lp(int panel_power_state)
{
	return !mdss_panel_is_power_off(panel_power_state) &&
		!mdss_panel_is_power_on_interactive(panel_power_state);
}

static inline bool mdss_panel_is_power_on_ulp(int panel_power_state)
{
	return panel_power_state == MDSS_PANEL_POWER_LP2;
}

static inline void mdss_panel_update_clk_rate(struct mdss_panel_info *pinfo,
	u32 fps)
{
	struct lcd_panel_info *lcdc = &pinfo->lcdc;
	u32 htotal, vtotal;

	if (pinfo->type == DTV_PANEL) {
		htotal = pinfo->xres + lcdc->h_front_porch +
				lcdc->h_back_porch + lcdc->h_pulse_width;
		vtotal = pinfo->yres + lcdc->v_front_porch +
				lcdc->v_back_porch + lcdc->v_pulse_width;

		pinfo->clk_rate = mult_frac(htotal * vtotal, fps, 1000);

		pr_debug("vtotal %d, htotal %d, rate %llu\n",
			vtotal, htotal, pinfo->clk_rate);
	}
}

static inline u8 mdss_panel_calc_frame_rate(struct mdss_panel_info *pinfo)
{
		u32 pixel_total = 0;
		u8 frame_rate = 0;
		unsigned long pclk_rate = pinfo->mipi.dsi_pclk_rate;
		u32 xres;

		xres = pinfo->xres;
		if (pinfo->compression_mode == COMPRESSION_DSC)
			xres /= 3;

		pixel_total = (pinfo->lcdc.h_back_porch +
			  pinfo->lcdc.h_front_porch +
			  pinfo->lcdc.h_pulse_width +
			  xres) *
			 (pinfo->lcdc.v_back_porch +
			  pinfo->lcdc.v_front_porch +
			  pinfo->lcdc.v_pulse_width +
			  pinfo->yres);

		if (pclk_rate && pixel_total)
			frame_rate =
				DIV_ROUND_CLOSEST(pclk_rate, pixel_total);
		else
			frame_rate = pinfo->panel_max_fps;

		return frame_rate;
}

struct mdss_panel_cfg *mdss_panel_intf_type(int intf_val);

bool mdss_is_ready(void);
int mdss_rect_cmp(struct mdss_rect *rect1, struct mdss_rect *rect2);

void mdss_panel_override_te_params(struct mdss_panel_info *pinfo);

void mdss_panel_dsc_parameters_calc(struct dsc_desc *dsc);

void mdss_panel_dsc_update_pic_dim(struct dsc_desc *dsc,
	int pic_width, int pic_height);

void mdss_panel_dsc_initial_line_calc(struct dsc_desc *dsc, int enc_ip_width);

void mdss_panel_dsc_pclk_param_calc(struct dsc_desc *dsc, int intf_width);

int mdss_panel_dsc_prepare_pps_buf(struct dsc_desc *dsc, char *buf,
	int pps_id);
#ifdef CONFIG_FB_MSM_MDSS
int mdss_panel_debugfs_init(struct mdss_panel_info *panel_info,
		char const *panel_name);
void mdss_panel_debugfs_cleanup(struct mdss_panel_info *panel_info);
void mdss_panel_debugfsinfo_to_panelinfo(struct mdss_panel_info *panel_info);

void mdss_panel_info_from_timing(struct mdss_panel_timing *pt,
		struct mdss_panel_info *pinfo);

struct mdss_panel_timing *mdss_panel_get_timing_by_name(
		struct mdss_panel_data *pdata,
		const char *name);
#else
static inline int mdss_panel_debugfs_init(
		struct mdss_panel_info *panel_info,
		char const *panel_name) { return 0; };
static inline void mdss_panel_debugfs_cleanup(
		struct mdss_panel_info *panel_info) { };
static inline void mdss_panel_debugfsinfo_to_panelinfo(
		struct mdss_panel_info *panel_info) { };
static inline void mdss_panel_info_from_timing(struct mdss_panel_timing *pt,
		struct mdss_panel_info *pinfo) { };
static inline struct mdss_panel_timing *mdss_panel_get_timing_by_name(
		struct mdss_panel_data *pdata,
		const char *name) { return NULL; };
#endif
#endif 
