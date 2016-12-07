/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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

#include <linux/ipa.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/ipa_uc_offload.h>
#include "ipa_api.h"

#define DRV_NAME "ipa"

#define IPA_API_DISPATCH_RETURN(api, p...) \
	do { \
		if (!ipa_api_ctrl) { \
			pr_err("IPA HW is not supported on this target\n"); \
			ret = -EPERM; \
		} \
		else { \
			if (ipa_api_ctrl->api) { \
				ret = ipa_api_ctrl->api(p); \
			} else { \
				pr_err("%s not implemented for IPA ver %d\n", \
						__func__, ipa_api_hw_type); \
				WARN_ON(1); \
				ret = -EPERM; \
			} \
		} \
	} while (0)

#define IPA_API_DISPATCH(api, p...) \
	do { \
		if (!ipa_api_ctrl) \
			pr_err("IPA HW is not supported on this target\n"); \
		else { \
			if (ipa_api_ctrl->api) { \
				ipa_api_ctrl->api(p); \
			} else { \
				pr_err("%s not implemented for IPA ver %d\n", \
						__func__, ipa_api_hw_type); \
				WARN_ON(1); \
			} \
		} \
	} while (0)

#define IPA_API_DISPATCH_RETURN_PTR(api, p...) \
	do { \
		if (!ipa_api_ctrl) { \
			pr_err("IPA HW is not supported on this target\n"); \
			ret = NULL; \
		} \
		else { \
			if (ipa_api_ctrl->api) { \
				ret = ipa_api_ctrl->api(p); \
			} else { \
				pr_err("%s not implemented for IPA ver %d\n", \
						__func__, ipa_api_hw_type); \
				WARN_ON(1); \
				ret = NULL; \
			} \
		} \
	} while (0)

#define IPA_API_DISPATCH_RETURN_BOOL(api, p...) \
	do { \
		if (!ipa_api_ctrl) { \
			pr_err("IPA HW is not supported on this target\n"); \
			ret = false; \
		} \
		else { \
			if (ipa_api_ctrl->api) { \
				ret = ipa_api_ctrl->api(p); \
			} else { \
				pr_err("%s not implemented for IPA ver %d\n", \
						__func__, ipa_api_hw_type); \
				WARN_ON(1); \
				ret = false; \
			} \
		} \
	} while (0)

static enum ipa_hw_type ipa_api_hw_type;
static struct ipa_api_controller *ipa_api_ctrl;

const char *ipa_clients_strings[IPA_CLIENT_MAX] = {
	__stringify(IPA_CLIENT_HSIC1_PROD),
	__stringify(IPA_CLIENT_WLAN1_PROD),
	__stringify(IPA_CLIENT_HSIC2_PROD),
	__stringify(IPA_CLIENT_USB2_PROD),
	__stringify(IPA_CLIENT_HSIC3_PROD),
	__stringify(IPA_CLIENT_USB3_PROD),
	__stringify(IPA_CLIENT_HSIC4_PROD),
	__stringify(IPA_CLIENT_USB4_PROD),
	__stringify(IPA_CLIENT_HSIC5_PROD),
	__stringify(IPA_CLIENT_USB_PROD),
	__stringify(IPA_CLIENT_A5_WLAN_AMPDU_PROD),
	__stringify(IPA_CLIENT_A2_EMBEDDED_PROD),
	__stringify(IPA_CLIENT_A2_TETHERED_PROD),
	__stringify(IPA_CLIENT_APPS_LAN_WAN_PROD),
	__stringify(IPA_CLIENT_APPS_CMD_PROD),
	__stringify(IPA_CLIENT_ODU_PROD),
	__stringify(IPA_CLIENT_MHI_PROD),
	__stringify(IPA_CLIENT_Q6_LAN_PROD),
	__stringify(IPA_CLIENT_Q6_WAN_PROD),
	__stringify(IPA_CLIENT_Q6_CMD_PROD),
	__stringify(IPA_CLIENT_MEMCPY_DMA_SYNC_PROD),
	__stringify(IPA_CLIENT_MEMCPY_DMA_ASYNC_PROD),
	__stringify(IPA_CLIENT_Q6_DECOMP_PROD),
	__stringify(IPA_CLIENT_Q6_DECOMP2_PROD),
	__stringify(IPA_CLIENT_UC_USB_PROD),

	
	__stringify(IPA_CLIENT_TEST_PROD),
	__stringify(IPA_CLIENT_TEST1_PROD),
	__stringify(IPA_CLIENT_TEST2_PROD),
	__stringify(IPA_CLIENT_TEST3_PROD),
	__stringify(IPA_CLIENT_TEST4_PROD),

	__stringify(IPA_CLIENT_HSIC1_CONS),
	__stringify(IPA_CLIENT_WLAN1_CONS),
	__stringify(IPA_CLIENT_HSIC2_CONS),
	__stringify(IPA_CLIENT_USB2_CONS),
	__stringify(IPA_CLIENT_WLAN2_CONS),
	__stringify(IPA_CLIENT_HSIC3_CONS),
	__stringify(IPA_CLIENT_USB3_CONS),
	__stringify(IPA_CLIENT_WLAN3_CONS),
	__stringify(IPA_CLIENT_HSIC4_CONS),
	__stringify(IPA_CLIENT_USB4_CONS),
	__stringify(IPA_CLIENT_WLAN4_CONS),
	__stringify(IPA_CLIENT_HSIC5_CONS),
	__stringify(IPA_CLIENT_USB_CONS),
	__stringify(IPA_CLIENT_USB_DPL_CONS),
	__stringify(IPA_CLIENT_A2_EMBEDDED_CONS),
	__stringify(IPA_CLIENT_A2_TETHERED_CONS),
	__stringify(IPA_CLIENT_A5_LAN_WAN_CONS),
	__stringify(IPA_CLIENT_APPS_LAN_CONS),
	__stringify(IPA_CLIENT_APPS_WAN_CONS),
	__stringify(IPA_CLIENT_ODU_EMB_CONS),
	__stringify(IPA_CLIENT_ODU_TETH_CONS),
	__stringify(IPA_CLIENT_MHI_CONS),
	__stringify(IPA_CLIENT_Q6_LAN_CONS),
	__stringify(IPA_CLIENT_Q6_WAN_CONS),
	__stringify(IPA_CLIENT_Q6_DUN_CONS),
	__stringify(IPA_CLIENT_MEMCPY_DMA_SYNC_CONS),
	__stringify(IPA_CLIENT_MEMCPY_DMA_ASYNC_CONS),
	__stringify(IPA_CLIENT_Q6_DECOMP_CONS),
	__stringify(IPA_CLIENT_Q6_DECOMP2_CONS),
	__stringify(IPA_CLIENT_Q6_LTE_WIFI_AGGR_CONS),
	
	__stringify(IPA_CLIENT_TEST_CONS),
	__stringify(IPA_CLIENT_TEST1_CONS),
	__stringify(IPA_CLIENT_TEST2_CONS),
	__stringify(IPA_CLIENT_TEST3_CONS),
	__stringify(IPA_CLIENT_TEST4_CONS),
};


int ipa_connect(const struct ipa_connect_params *in, struct ipa_sps_params *sps,
	u32 *clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_connect, in, sps, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_connect);

int ipa_disconnect(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_disconnect, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_disconnect);

int ipa_clear_endpoint_delay(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_clear_endpoint_delay, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_clear_endpoint_delay);

int ipa_reset_endpoint(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_reset_endpoint, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_reset_endpoint);

int ipa_disable_endpoint(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_disable_endpoint, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_disable_endpoint);


int ipa_cfg_ep(u32 clnt_hdl, const struct ipa_ep_cfg *ipa_ep_cfg)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep, clnt_hdl, ipa_ep_cfg);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep);

int ipa_cfg_ep_nat(u32 clnt_hdl, const struct ipa_ep_cfg_nat *ep_nat)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep_nat, clnt_hdl, ep_nat);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep_nat);

int ipa_cfg_ep_hdr(u32 clnt_hdl, const struct ipa_ep_cfg_hdr *ep_hdr)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep_hdr, clnt_hdl, ep_hdr);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep_hdr);

int ipa_cfg_ep_hdr_ext(u32 clnt_hdl,
		       const struct ipa_ep_cfg_hdr_ext *ep_hdr_ext)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep_hdr_ext, clnt_hdl, ep_hdr_ext);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep_hdr_ext);

int ipa_cfg_ep_mode(u32 clnt_hdl, const struct ipa_ep_cfg_mode *ep_mode)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep_mode, clnt_hdl, ep_mode);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep_mode);

int ipa_cfg_ep_aggr(u32 clnt_hdl, const struct ipa_ep_cfg_aggr *ep_aggr)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep_aggr, clnt_hdl, ep_aggr);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep_aggr);

int ipa_cfg_ep_deaggr(u32 clnt_hdl,
			const struct ipa_ep_cfg_deaggr *ep_deaggr)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep_deaggr, clnt_hdl, ep_deaggr);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep_deaggr);

int ipa_cfg_ep_route(u32 clnt_hdl, const struct ipa_ep_cfg_route *ep_route)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep_route, clnt_hdl, ep_route);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep_route);

int ipa_cfg_ep_holb(u32 clnt_hdl, const struct ipa_ep_cfg_holb *ep_holb)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep_holb, clnt_hdl, ep_holb);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep_holb);


int ipa_cfg_ep_cfg(u32 clnt_hdl, const struct ipa_ep_cfg_cfg *cfg)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep_cfg, clnt_hdl, cfg);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep_cfg);

int ipa_cfg_ep_metadata_mask(u32 clnt_hdl, const struct ipa_ep_cfg_metadata_mask
		*metadata_mask)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep_metadata_mask, clnt_hdl,
			metadata_mask);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep_metadata_mask);

int ipa_cfg_ep_holb_by_client(enum ipa_client_type client,
				const struct ipa_ep_cfg_holb *ep_holb)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep_holb_by_client, client, ep_holb);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep_holb_by_client);

int ipa_cfg_ep_ctrl(u32 clnt_hdl, const struct ipa_ep_cfg_ctrl *ep_ctrl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_cfg_ep_ctrl, clnt_hdl, ep_ctrl);

	return ret;
}
EXPORT_SYMBOL(ipa_cfg_ep_ctrl);

int ipa_add_hdr(struct ipa_ioc_add_hdr *hdrs)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_add_hdr, hdrs);

	return ret;
}
EXPORT_SYMBOL(ipa_add_hdr);

int ipa_del_hdr(struct ipa_ioc_del_hdr *hdls)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_del_hdr, hdls);

	return ret;
}
EXPORT_SYMBOL(ipa_del_hdr);

int ipa_commit_hdr(void)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_commit_hdr);

	return ret;
}
EXPORT_SYMBOL(ipa_commit_hdr);

int ipa_reset_hdr(void)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_reset_hdr);

	return ret;
}
EXPORT_SYMBOL(ipa_reset_hdr);

int ipa_get_hdr(struct ipa_ioc_get_hdr *lookup)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_get_hdr, lookup);

	return ret;
}
EXPORT_SYMBOL(ipa_get_hdr);

int ipa_put_hdr(u32 hdr_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_put_hdr, hdr_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_put_hdr);

int ipa_copy_hdr(struct ipa_ioc_copy_hdr *copy)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_copy_hdr, copy);

	return ret;
}
EXPORT_SYMBOL(ipa_copy_hdr);

int ipa_add_hdr_proc_ctx(struct ipa_ioc_add_hdr_proc_ctx *proc_ctxs)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_add_hdr_proc_ctx, proc_ctxs);

	return ret;
}
EXPORT_SYMBOL(ipa_add_hdr_proc_ctx);

int ipa_del_hdr_proc_ctx(struct ipa_ioc_del_hdr_proc_ctx *hdls)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_del_hdr_proc_ctx, hdls);

	return ret;
}
EXPORT_SYMBOL(ipa_del_hdr_proc_ctx);

int ipa_add_rt_rule(struct ipa_ioc_add_rt_rule *rules)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_add_rt_rule, rules);

	return ret;
}
EXPORT_SYMBOL(ipa_add_rt_rule);

int ipa_del_rt_rule(struct ipa_ioc_del_rt_rule *hdls)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_del_rt_rule, hdls);

	return ret;
}
EXPORT_SYMBOL(ipa_del_rt_rule);

int ipa_commit_rt(enum ipa_ip_type ip)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_commit_rt, ip);

	return ret;
}
EXPORT_SYMBOL(ipa_commit_rt);

int ipa_reset_rt(enum ipa_ip_type ip)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_reset_rt, ip);

	return ret;
}
EXPORT_SYMBOL(ipa_reset_rt);

int ipa_get_rt_tbl(struct ipa_ioc_get_rt_tbl *lookup)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_get_rt_tbl, lookup);

	return ret;
}
EXPORT_SYMBOL(ipa_get_rt_tbl);

int ipa_put_rt_tbl(u32 rt_tbl_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_put_rt_tbl, rt_tbl_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_put_rt_tbl);

int ipa_query_rt_index(struct ipa_ioc_get_rt_tbl_indx *in)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_query_rt_index, in);

	return ret;
}
EXPORT_SYMBOL(ipa_query_rt_index);

int ipa_mdfy_rt_rule(struct ipa_ioc_mdfy_rt_rule *hdls)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_mdfy_rt_rule, hdls);

	return ret;
}
EXPORT_SYMBOL(ipa_mdfy_rt_rule);

int ipa_add_flt_rule(struct ipa_ioc_add_flt_rule *rules)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_add_flt_rule, rules);

	return ret;
}
EXPORT_SYMBOL(ipa_add_flt_rule);

int ipa_del_flt_rule(struct ipa_ioc_del_flt_rule *hdls)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_del_flt_rule, hdls);

	return ret;
}
EXPORT_SYMBOL(ipa_del_flt_rule);

int ipa_mdfy_flt_rule(struct ipa_ioc_mdfy_flt_rule *hdls)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_mdfy_flt_rule, hdls);

	return ret;
}
EXPORT_SYMBOL(ipa_mdfy_flt_rule);

int ipa_commit_flt(enum ipa_ip_type ip)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_commit_flt, ip);

	return ret;
}
EXPORT_SYMBOL(ipa_commit_flt);

int ipa_reset_flt(enum ipa_ip_type ip)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_reset_flt, ip);

	return ret;
}
EXPORT_SYMBOL(ipa_reset_flt);

int allocate_nat_device(struct ipa_ioc_nat_alloc_mem *mem)
{
	int ret;

	IPA_API_DISPATCH_RETURN(allocate_nat_device, mem);

	return ret;
}
EXPORT_SYMBOL(allocate_nat_device);

int ipa_nat_init_cmd(struct ipa_ioc_v4_nat_init *init)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_nat_init_cmd, init);

	return ret;
}
EXPORT_SYMBOL(ipa_nat_init_cmd);

int ipa_nat_dma_cmd(struct ipa_ioc_nat_dma_cmd *dma)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_nat_dma_cmd, dma);

	return ret;
}
EXPORT_SYMBOL(ipa_nat_dma_cmd);

int ipa_nat_del_cmd(struct ipa_ioc_v4_nat_del *del)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_nat_del_cmd, del);

	return ret;
}
EXPORT_SYMBOL(ipa_nat_del_cmd);

int ipa_send_msg(struct ipa_msg_meta *meta, void *buff,
		  ipa_msg_free_fn callback)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_send_msg, meta, buff, callback);

	return ret;
}
EXPORT_SYMBOL(ipa_send_msg);

int ipa_register_pull_msg(struct ipa_msg_meta *meta, ipa_msg_pull_fn callback)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_register_pull_msg, meta, callback);

	return ret;
}
EXPORT_SYMBOL(ipa_register_pull_msg);

int ipa_deregister_pull_msg(struct ipa_msg_meta *meta)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_deregister_pull_msg, meta);

	return ret;
}
EXPORT_SYMBOL(ipa_deregister_pull_msg);

int ipa_register_intf(const char *name, const struct ipa_tx_intf *tx,
		       const struct ipa_rx_intf *rx)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_register_intf, name, tx, rx);

	return ret;
}
EXPORT_SYMBOL(ipa_register_intf);

int ipa_register_intf_ext(const char *name, const struct ipa_tx_intf *tx,
	const struct ipa_rx_intf *rx,
	const struct ipa_ext_intf *ext)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_register_intf_ext, name, tx, rx, ext);

	return ret;
}
EXPORT_SYMBOL(ipa_register_intf_ext);

int ipa_deregister_intf(const char *name)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_deregister_intf, name);

	return ret;
}
EXPORT_SYMBOL(ipa_deregister_intf);

int ipa_set_aggr_mode(enum ipa_aggr_mode mode)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_set_aggr_mode, mode);

	return ret;
}
EXPORT_SYMBOL(ipa_set_aggr_mode);


int ipa_set_qcncm_ndp_sig(char sig[3])
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_set_qcncm_ndp_sig, sig);

	return ret;
}
EXPORT_SYMBOL(ipa_set_qcncm_ndp_sig);

int ipa_set_single_ndp_per_mbim(bool enable)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_set_single_ndp_per_mbim, enable);

	return ret;
}
EXPORT_SYMBOL(ipa_set_single_ndp_per_mbim);

int ipa_tx_dp(enum ipa_client_type dst, struct sk_buff *skb,
		struct ipa_tx_meta *meta)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_tx_dp, dst, skb, meta);

	return ret;
}
EXPORT_SYMBOL(ipa_tx_dp);

int ipa_tx_dp_mul(enum ipa_client_type src,
			struct ipa_tx_data_desc *data_desc)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_tx_dp_mul, src, data_desc);

	return ret;
}
EXPORT_SYMBOL(ipa_tx_dp_mul);

void ipa_free_skb(struct ipa_rx_data *data)
{
	IPA_API_DISPATCH(ipa_free_skb, data);
}
EXPORT_SYMBOL(ipa_free_skb);

int ipa_setup_sys_pipe(struct ipa_sys_connect_params *sys_in, u32 *clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_setup_sys_pipe, sys_in, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_setup_sys_pipe);

int ipa_teardown_sys_pipe(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_teardown_sys_pipe, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_teardown_sys_pipe);

int ipa_sys_setup(struct ipa_sys_connect_params *sys_in,
	unsigned long *ipa_bam_or_gsi_hdl,
	u32 *ipa_pipe_num, u32 *clnt_hdl, bool en_status)

{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_sys_setup, sys_in, ipa_bam_or_gsi_hdl,
			ipa_pipe_num, clnt_hdl, en_status);

	return ret;
}
EXPORT_SYMBOL(ipa_sys_setup);

int ipa_sys_teardown(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_sys_teardown, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_sys_teardown);

int ipa_sys_update_gsi_hdls(u32 clnt_hdl, unsigned long gsi_ch_hdl,
	unsigned long gsi_ev_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_sys_update_gsi_hdls, clnt_hdl,
		gsi_ch_hdl, gsi_ev_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_sys_update_gsi_hdls);

int ipa_connect_wdi_pipe(struct ipa_wdi_in_params *in,
		struct ipa_wdi_out_params *out)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_connect_wdi_pipe, in, out);

	return ret;
}
EXPORT_SYMBOL(ipa_connect_wdi_pipe);

int ipa_disconnect_wdi_pipe(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_disconnect_wdi_pipe, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_disconnect_wdi_pipe);

int ipa_enable_wdi_pipe(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_enable_wdi_pipe, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_enable_wdi_pipe);

int ipa_disable_wdi_pipe(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_disable_wdi_pipe, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_disable_wdi_pipe);

int ipa_resume_wdi_pipe(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_resume_wdi_pipe, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_resume_wdi_pipe);

int ipa_suspend_wdi_pipe(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_suspend_wdi_pipe, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_suspend_wdi_pipe);

int ipa_get_wdi_stats(struct IpaHwStatsWDIInfoData_t *stats)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_get_wdi_stats, stats);

	return ret;
}
EXPORT_SYMBOL(ipa_get_wdi_stats);

u16 ipa_get_smem_restr_bytes(void)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_get_smem_restr_bytes);

	return ret;
}
EXPORT_SYMBOL(ipa_get_smem_restr_bytes);

int ipa_uc_wdi_get_dbpa(
	struct ipa_wdi_db_params *param)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_uc_wdi_get_dbpa, param);

	return ret;
}
EXPORT_SYMBOL(ipa_uc_wdi_get_dbpa);

int ipa_uc_reg_rdyCB(
	struct ipa_wdi_uc_ready_params *inout)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_uc_reg_rdyCB, inout);

	return ret;
}
EXPORT_SYMBOL(ipa_uc_reg_rdyCB);

int ipa_uc_dereg_rdyCB(void)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_uc_dereg_rdyCB);

	return ret;
}
EXPORT_SYMBOL(ipa_uc_dereg_rdyCB);

int teth_bridge_init(struct teth_bridge_init_params *params)
{
	int ret;

	IPA_API_DISPATCH_RETURN(teth_bridge_init, params);

	return ret;
}
EXPORT_SYMBOL(teth_bridge_init);

int teth_bridge_disconnect(enum ipa_client_type client)
{
	int ret;

	IPA_API_DISPATCH_RETURN(teth_bridge_disconnect, client);

	return ret;
}
EXPORT_SYMBOL(teth_bridge_disconnect);

int teth_bridge_connect(struct teth_bridge_connect_params *connect_params)
{
	int ret;

	IPA_API_DISPATCH_RETURN(teth_bridge_connect, connect_params);

	return ret;
}
EXPORT_SYMBOL(teth_bridge_connect);


void ipa_set_client(int index, enum ipacm_client_enum client, bool uplink)
{
	IPA_API_DISPATCH(ipa_set_client, index, client, uplink);
}
EXPORT_SYMBOL(ipa_set_client);

enum ipacm_client_enum ipa_get_client(int pipe_idx)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_get_client, pipe_idx);

	return ret;
}
EXPORT_SYMBOL(ipa_get_client);

bool ipa_get_client_uplink(int pipe_idx)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_get_client_uplink, pipe_idx);

	return ret;
}
EXPORT_SYMBOL(ipa_get_client_uplink);

int ipa_dma_init(void)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_dma_init);

	return ret;
}
EXPORT_SYMBOL(ipa_dma_init);

int ipa_dma_enable(void)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_dma_enable);

	return ret;
}
EXPORT_SYMBOL(ipa_dma_enable);

int ipa_dma_disable(void)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_dma_disable);

	return ret;
}
EXPORT_SYMBOL(ipa_dma_disable);

int ipa_dma_sync_memcpy(u64 dest, u64 src, int len)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_dma_sync_memcpy, dest, src, len);

	return ret;
}
EXPORT_SYMBOL(ipa_dma_sync_memcpy);

int ipa_dma_async_memcpy(u64 dest, u64 src, int len,
		void (*user_cb)(void *user1), void *user_param)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_dma_async_memcpy, dest, src, len, user_cb,
		user_param);

	return ret;
}
EXPORT_SYMBOL(ipa_dma_async_memcpy);

int ipa_dma_uc_memcpy(phys_addr_t dest, phys_addr_t src, int len)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_dma_uc_memcpy, dest, src, len);

	return ret;
}
EXPORT_SYMBOL(ipa_dma_uc_memcpy);

void ipa_dma_destroy(void)
{
	IPA_API_DISPATCH(ipa_dma_destroy);
}
EXPORT_SYMBOL(ipa_dma_destroy);

int ipa_mhi_init_engine(struct ipa_mhi_init_engine *params)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_mhi_init_engine, params);

	return ret;
}
EXPORT_SYMBOL(ipa_mhi_init_engine);

int ipa_connect_mhi_pipe(struct ipa_mhi_connect_params_internal *in,
		u32 *clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_connect_mhi_pipe, in, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_connect_mhi_pipe);

int ipa_disconnect_mhi_pipe(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_disconnect_mhi_pipe, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_disconnect_mhi_pipe);

bool ipa_mhi_stop_gsi_channel(enum ipa_client_type client)
{
	bool ret;

	IPA_API_DISPATCH_RETURN_BOOL(ipa_mhi_stop_gsi_channel, client);

	return ret;
}

int ipa_uc_mhi_reset_channel(int channelHandle)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_uc_mhi_reset_channel, channelHandle);

	return ret;
}

bool ipa_mhi_sps_channel_empty(enum ipa_client_type client)
{
	bool ret;

	IPA_API_DISPATCH_RETURN_BOOL(ipa_mhi_sps_channel_empty, client);

	return ret;
}

int ipa_qmi_enable_force_clear_datapath_send(
	struct ipa_enable_force_clear_datapath_req_msg_v01 *req)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_qmi_enable_force_clear_datapath_send, req);

	return ret;
}

int ipa_qmi_disable_force_clear_datapath_send(
	struct ipa_disable_force_clear_datapath_req_msg_v01 *req)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_qmi_disable_force_clear_datapath_send, req);

	return ret;
}

int ipa_generate_tag_process(void)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_generate_tag_process);

	return ret;
}

int ipa_disable_sps_pipe(enum ipa_client_type client)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_disable_sps_pipe, client);

	return ret;
}

int ipa_mhi_reset_channel_internal(enum ipa_client_type client)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_mhi_reset_channel_internal, client);

	return ret;
}

int ipa_mhi_start_channel_internal(enum ipa_client_type client)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_mhi_start_channel_internal, client);

	return ret;
}

void ipa_get_holb(int ep_idx, struct ipa_ep_cfg_holb *holb)
{
	IPA_API_DISPATCH(ipa_get_holb, ep_idx, holb);
}

void ipa_set_tag_process_before_gating(bool val)
{
	IPA_API_DISPATCH(ipa_set_tag_process_before_gating, val);
}

int ipa_mhi_query_ch_info(enum ipa_client_type client,
		struct gsi_chan_info *ch_info)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_mhi_query_ch_info, client, ch_info);

	return ret;
}

int ipa_uc_mhi_suspend_channel(int channelHandle)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_uc_mhi_suspend_channel, channelHandle);

	return ret;
}

int ipa_uc_mhi_stop_event_update_channel(int channelHandle)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_uc_mhi_stop_event_update_channel,
			channelHandle);

	return ret;
}

bool ipa_has_open_aggr_frame(enum ipa_client_type client)
{
	bool ret;

	IPA_API_DISPATCH_RETURN_BOOL(ipa_has_open_aggr_frame, client);

	return ret;
}

int ipa_mhi_resume_channels_internal(enum ipa_client_type client,
		bool LPTransitionRejected, bool brstmode_enabled,
		union __packed gsi_channel_scratch ch_scratch, u8 index)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_mhi_resume_channels_internal, client,
			LPTransitionRejected, brstmode_enabled, ch_scratch,
			index);

	return ret;
}

int ipa_uc_mhi_send_dl_ul_sync_info(union IpaHwMhiDlUlSyncCmdData_t *cmd)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_uc_mhi_send_dl_ul_sync_info,
			cmd);

	return ret;
}

int ipa_mhi_destroy_channel(enum ipa_client_type client)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_mhi_destroy_channel, client);

	return ret;
}

int ipa_uc_mhi_init(void (*ready_cb)(void),
		void (*wakeup_request_cb)(void))
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_uc_mhi_init, ready_cb, wakeup_request_cb);

	return ret;
}

void ipa_uc_mhi_cleanup(void)
{
	IPA_API_DISPATCH(ipa_uc_mhi_cleanup);
}

int ipa_uc_mhi_print_stats(char *dbg_buff, int size)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_uc_mhi_print_stats, dbg_buff, size);

	return ret;
}

int ipa_uc_state_check(void)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_uc_state_check);

	return ret;
}

int ipa_write_qmap_id(struct ipa_ioc_write_qmapid *param_in)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_write_qmap_id, param_in);

	return ret;
}
EXPORT_SYMBOL(ipa_write_qmap_id);

int ipa_add_interrupt_handler(enum ipa_irq_type interrupt,
	ipa_irq_handler_t handler,
	bool deferred_flag,
	void *private_data)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_add_interrupt_handler, interrupt, handler,
		deferred_flag, private_data);

	return ret;
}
EXPORT_SYMBOL(ipa_add_interrupt_handler);

int ipa_remove_interrupt_handler(enum ipa_irq_type interrupt)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_remove_interrupt_handler, interrupt);

	return ret;
}
EXPORT_SYMBOL(ipa_remove_interrupt_handler);

int ipa_restore_suspend_handler(void)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_restore_suspend_handler);

	return ret;
}
EXPORT_SYMBOL(ipa_restore_suspend_handler);

void ipa_bam_reg_dump(void)
{
	IPA_API_DISPATCH(ipa_bam_reg_dump);
}
EXPORT_SYMBOL(ipa_bam_reg_dump);

int ipa_get_ep_mapping(enum ipa_client_type client)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_get_ep_mapping, client);

	return ret;
}
EXPORT_SYMBOL(ipa_get_ep_mapping);

bool ipa_is_ready(void)
{
	if (!ipa_api_ctrl || !ipa_api_ctrl->ipa_is_ready)
		return false;
	return ipa_api_ctrl->ipa_is_ready();
}
EXPORT_SYMBOL(ipa_is_ready);

void ipa_proxy_clk_vote(void)
{
	IPA_API_DISPATCH(ipa_proxy_clk_vote);
}
EXPORT_SYMBOL(ipa_proxy_clk_vote);

void ipa_proxy_clk_unvote(void)
{
	IPA_API_DISPATCH(ipa_proxy_clk_unvote);
}
EXPORT_SYMBOL(ipa_proxy_clk_unvote);

enum ipa_hw_type ipa_get_hw_type(void)
{
	return ipa_api_hw_type;
}
EXPORT_SYMBOL(ipa_get_hw_type);

bool ipa_is_client_handle_valid(u32 clnt_hdl)
{
	if (!ipa_api_ctrl || !ipa_api_ctrl->ipa_is_client_handle_valid)
		return false;
	return ipa_api_ctrl->ipa_is_client_handle_valid(clnt_hdl);
}
EXPORT_SYMBOL(ipa_is_client_handle_valid);

enum ipa_client_type ipa_get_client_mapping(int pipe_idx)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_get_client_mapping, pipe_idx);

	return ret;
}
EXPORT_SYMBOL(ipa_get_client_mapping);

enum ipa_rm_resource_name ipa_get_rm_resource_from_ep(int pipe_idx)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_get_rm_resource_from_ep, pipe_idx);

	return ret;
}
EXPORT_SYMBOL(ipa_get_rm_resource_from_ep);

bool ipa_get_modem_cfg_emb_pipe_flt(void)
{
	if (!ipa_api_ctrl || !ipa_api_ctrl->ipa_get_modem_cfg_emb_pipe_flt)
		return false;
	return ipa_api_ctrl->ipa_get_modem_cfg_emb_pipe_flt();
}
EXPORT_SYMBOL(ipa_get_modem_cfg_emb_pipe_flt);

enum ipa_transport_type ipa_get_transport_type(void)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_get_transport_type);

	return ret;
}
EXPORT_SYMBOL(ipa_get_transport_type);

struct iommu_domain *ipa_get_smmu_domain(void)
{
	struct iommu_domain *ret;

	IPA_API_DISPATCH_RETURN_PTR(ipa_get_smmu_domain);

	return ret;
}
EXPORT_SYMBOL(ipa_get_smmu_domain);

int ipa_disable_apps_wan_cons_deaggr(uint32_t agg_size, uint32_t agg_count)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_disable_apps_wan_cons_deaggr, agg_size,
		agg_count);

	return ret;
}
EXPORT_SYMBOL(ipa_disable_apps_wan_cons_deaggr);

struct device *ipa_get_dma_dev(void)
{
	struct device *ret;

	IPA_API_DISPATCH_RETURN_PTR(ipa_get_dma_dev);

	return ret;
}
EXPORT_SYMBOL(ipa_get_dma_dev);

int ipa_release_wdi_mapping(u32 num_buffers, struct ipa_wdi_buffer_info *info)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_release_wdi_mapping, num_buffers, info);

	return ret;
}
EXPORT_SYMBOL(ipa_release_wdi_mapping);

int ipa_create_wdi_mapping(u32 num_buffers, struct ipa_wdi_buffer_info *info)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_create_wdi_mapping, num_buffers, info);

	return ret;
}
EXPORT_SYMBOL(ipa_create_wdi_mapping);

struct ipa_gsi_ep_config *ipa_get_gsi_ep_info(int ipa_ep_idx)
{
	if (!ipa_api_ctrl || !ipa_api_ctrl->ipa_get_gsi_ep_info)
		return NULL;
	return ipa_api_ctrl->ipa_get_gsi_ep_info(ipa_ep_idx);
}
EXPORT_SYMBOL(ipa_get_gsi_ep_info);

int ipa_stop_gsi_channel(u32 clnt_hdl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_stop_gsi_channel, clnt_hdl);

	return ret;
}
EXPORT_SYMBOL(ipa_stop_gsi_channel);

const char *ipa_get_version_string(enum ipa_hw_type ver)
{
	const char *str;

	switch (ver) {
	case IPA_HW_v1_0:
		str = "1.0";
		break;
	case IPA_HW_v1_1:
		str = "1.1";
		break;
	case IPA_HW_v2_0:
		str = "2.0";
		break;
	case IPA_HW_v2_1:
		str = "2.1";
		break;
	case IPA_HW_v2_5:
		str = "2.5/2.6";
		break;
	case IPA_HW_v2_6L:
		str = "2.6L";
		break;
	case IPA_HW_v3_0:
		str = "3.0";
		break;
	case IPA_HW_v3_1:
		str = "3.1";
		break;
	default:
		str = "Invalid version";
		break;
	}

	return str;
}
EXPORT_SYMBOL(ipa_get_version_string);

static struct of_device_id ipa_plat_drv_match[] = {
	{ .compatible = "qcom,ipa", },
	{ .compatible = "qcom,ipa-smmu-ap-cb", },
	{ .compatible = "qcom,ipa-smmu-wlan-cb", },
	{ .compatible = "qcom,ipa-smmu-uc-cb", },
	{ .compatible = "qcom,smp2pgpio-map-ipa-1-in", },
	{ .compatible = "qcom,smp2pgpio-map-ipa-1-out", },
	{}
};

static int ipa_generic_plat_drv_probe(struct platform_device *pdev_p)
{
	int result;

	pr_debug("ipa: IPA driver probing started for %s\n",
		pdev_p->dev.of_node->name);

	if (!ipa_api_ctrl) {
		ipa_api_ctrl = kzalloc(sizeof(*ipa_api_ctrl), GFP_KERNEL);
		if (!ipa_api_ctrl)
			return -ENOMEM;

		
		result = of_property_read_u32(pdev_p->dev.of_node,
			"qcom,ipa-hw-ver", &ipa_api_hw_type);
		if ((result) || (ipa_api_hw_type == 0)) {
			pr_err("ipa: get resource failed for ipa-hw-ver!\n");
			kfree(ipa_api_ctrl);
			ipa_api_ctrl = 0;
			return -ENODEV;
		}
		pr_debug("ipa: ipa_api_hw_type = %d", ipa_api_hw_type);
	}

	
	switch (ipa_api_hw_type) {
	case IPA_HW_v2_0:
	case IPA_HW_v2_1:
	case IPA_HW_v2_5:
	case IPA_HW_v2_6L:
		result = ipa_plat_drv_probe(pdev_p, ipa_api_ctrl,
			ipa_plat_drv_match);
		break;
	case IPA_HW_v3_0:
	case IPA_HW_v3_1:
		result = ipa3_plat_drv_probe(pdev_p, ipa_api_ctrl,
			ipa_plat_drv_match);
		break;
	default:
		pr_err("ipa: unsupported version %d\n", ipa_api_hw_type);
		return -EPERM;
	}

	if (result && result != -EPROBE_DEFER)
		pr_err("ipa: ipa_plat_drv_probe failed\n");

	return result;
}

static int ipa_ap_suspend(struct device *dev)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_ap_suspend, dev);

	return ret;
}

static int ipa_ap_resume(struct device *dev)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_ap_resume, dev);

	return ret;
}

int ipa_register_ipa_ready_cb(void (*ipa_ready_cb)(void *user_data),
			      void *user_data)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_register_ipa_ready_cb,
				ipa_ready_cb, user_data);

	return ret;
}
EXPORT_SYMBOL(ipa_register_ipa_ready_cb);

void ipa_inc_client_enable_clks(struct ipa_active_client_logging_info *id)
{
	IPA_API_DISPATCH(ipa_inc_client_enable_clks, id);
}
EXPORT_SYMBOL(ipa_inc_client_enable_clks);

void ipa_dec_client_disable_clks(struct ipa_active_client_logging_info *id)
{
	IPA_API_DISPATCH(ipa_dec_client_disable_clks, id);
}
EXPORT_SYMBOL(ipa_dec_client_disable_clks);

int ipa_inc_client_enable_clks_no_block(
	struct ipa_active_client_logging_info *id)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_inc_client_enable_clks_no_block, id);

	return ret;
}
EXPORT_SYMBOL(ipa_inc_client_enable_clks_no_block);

int ipa_suspend_resource_no_block(enum ipa_rm_resource_name resource)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_suspend_resource_no_block, resource);

	return ret;
}
EXPORT_SYMBOL(ipa_suspend_resource_no_block);
int ipa_resume_resource(enum ipa_rm_resource_name resource)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_resume_resource, resource);

	return ret;
}
EXPORT_SYMBOL(ipa_resume_resource);

int ipa_suspend_resource_sync(enum ipa_rm_resource_name resource)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_suspend_resource_sync, resource);

	return ret;
}
EXPORT_SYMBOL(ipa_suspend_resource_sync);

int ipa_set_required_perf_profile(enum ipa_voltage_level floor_voltage,
	u32 bandwidth_mbps)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_set_required_perf_profile, floor_voltage,
		bandwidth_mbps);

	return ret;
}
EXPORT_SYMBOL(ipa_set_required_perf_profile);

void *ipa_get_ipc_logbuf(void)
{
	void *ret;

	IPA_API_DISPATCH_RETURN_PTR(ipa_get_ipc_logbuf);

	return ret;
}
EXPORT_SYMBOL(ipa_get_ipc_logbuf);

void *ipa_get_ipc_logbuf_low(void)
{
	void *ret;

	IPA_API_DISPATCH_RETURN_PTR(ipa_get_ipc_logbuf_low);

	return ret;
}
EXPORT_SYMBOL(ipa_get_ipc_logbuf_low);

void ipa_assert(void)
{
	pr_err("IPA: unrecoverable error has occurred, asserting\n");
	BUG();
}

int ipa_setup_uc_ntn_pipes(struct ipa_ntn_conn_in_params *inp,
		ipa_notify_cb notify, void *priv, u8 hdr_len,
		struct ipa_ntn_conn_out_params *outp)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_setup_uc_ntn_pipes, inp,
		notify, priv, hdr_len, outp);

	return ret;
}

int ipa_tear_down_uc_offload_pipes(int ipa_ep_idx_ul,
		int ipa_ep_idx_dl)
{
	int ret;

	IPA_API_DISPATCH_RETURN(ipa_tear_down_uc_offload_pipes, ipa_ep_idx_ul,
		ipa_ep_idx_dl);

	return ret;
}

static const struct dev_pm_ops ipa_pm_ops = {
	.suspend_noirq = ipa_ap_suspend,
	.resume_noirq = ipa_ap_resume,
};

static struct platform_driver ipa_plat_drv = {
	.probe = ipa_generic_plat_drv_probe,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &ipa_pm_ops,
		.of_match_table = ipa_plat_drv_match,
	},
};

static int __init ipa_module_init(void)
{
	pr_debug("IPA module init\n");

	
	return platform_driver_register(&ipa_plat_drv);
}
subsys_initcall(ipa_module_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("IPA HW device driver");
