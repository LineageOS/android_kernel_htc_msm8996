#ifndef PUBLIC_INTERFACE_H
#define PUBLIC_INTERFACE_H
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include "anx_ohio_private_interface.h"
#include "anx_ohio_public_interface.h"

u8 send_pd_msg(PD_MSG_TYPE type, const char *buf, u8 size)
{
	u8 rst = 0;

	switch (type) {
	case TYPE_PWR_SRC_CAP:
		rst = send_src_cap(buf, size);
		break;
	case TYPE_PWR_SNK_CAP:
		rst = send_snk_cap(buf, size);
		break;
	case TYPE_DP_SNK_IDENTITY:
		rst = interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY,
					(u8 *)buf, size, INTERFACE_TIMEOUT);
		break;
	case TYPE_SVID:
		rst = send_svid(buf, size);
		break;
	case TYPE_GET_DP_SNK_CAP:
		rst = interface_send_msg_timeout(TYPE_GET_DP_SNK_CAP, NULL, 0,
					       INTERFACE_TIMEOUT);
		break;
	case TYPE_PSWAP_REQ:
		rst = send_power_swap();
		break;
	case TYPE_DSWAP_REQ:
		rst = send_data_swap();
		break;
	case TYPE_GOTO_MIN_REQ:
		rst = interface_send_gotomin();
		break;
	case TYPE_VDM:
		rst = send_vdm(buf, size);
		break;
	case TYPE_DP_SNK_CFG:
		rst = send_dp_snk_cfg(buf, size);
		break;
	case TYPE_PWR_OBJ_REQ:
		rst = send_rdo(buf, size);
		break;
	case TYPE_ACCEPT:
		rst = interface_send_accept();
		break;
	case TYPE_REJECT:
		rst = interface_send_reject();
		break;
	case TYPE_SOFT_RST:
		rst = interface_send_soft_rst();
		break;
	case TYPE_HARD_RST:
		rst = interface_send_hard_rst();
		break;
	default:
		pr_info("unknown type %x\n", type);
		rst = 0;
		break;
	}
	if (rst == CMD_FAIL) {
		pr_err("Cmd %x Fail.\n", type);
		return CMD_FAIL;
	}

	return rst;
}

u8 dispatch_rcvd_pd_msg(PD_MSG_TYPE type, void *para, u8 para_len)
{
	u8 rst = 0;
	pd_callback_t fnc = get_pd_callback_fnc(type);
	if (fnc != 0) {
		rst = (*fnc) (para, para_len);
		return rst;
	}

	switch (type) {
	case TYPE_PWR_SRC_CAP:
		
#ifndef AUTO_RDO_ENABLE
		rst = recv_pd_source_caps_default_callback(para, para_len);
#endif

#ifdef SUP_TRY_SRC_SINK
		
		OhioWriteReg(VBUS_DELAY_TIME, 0xa0);  
#endif
		downstream_pd_cap = 1;

		break;
	case TYPE_PWR_SNK_CAP:
		
		rst = recv_pd_sink_caps_default_callback(para, para_len);
		break;
	case TYPE_PWR_OBJ_REQ:
		
#ifndef AUTO_RDO_ENABLE
		rst = recv_pd_pwr_object_req_default_callback(para, para_len);
#endif

#ifdef SUP_TRY_SRC_SINK
		
		OhioWriteReg(VBUS_DELAY_TIME, 0xa0);   
#endif
		downstream_pd_cap = 1;

		break;
	case TYPE_DSWAP_REQ:
		
		rst = recv_pd_dswap_default_callback(para, para_len);
		break;
	case TYPE_PSWAP_REQ:
		
		rst = recv_pd_pswap_default_callback(para, para_len);
		break;
	case TYPE_VDM:
		break;
	case TYPE_ACCEPT:
		rst = recv_pd_accept_default_callback(para, para_len);
		break;
	case TYPE_RESPONSE_TO_REQ:
		
		rst = recv_pd_cmd_rsp_default_callback(para, para_len);
		break;

	case TYPE_DP_ALT_ENTER:
		break;
	case TYPE_DP_ALT_EXIT:
		break;
	case TYPE_HARD_RST:
		rst = recv_pd_hard_rst_default_callback(para, para_len);
		break;
	default:
		rst = 0;
		break;
	}
	return rst;
}
u8 register_pd_msg_callback_func(PD_MSG_TYPE type, pd_callback_t fnc)
{
	if (type > 256)
		return 1;
	set_pd_callback_fnc(type, fnc);

	return 0;
}

#endif
