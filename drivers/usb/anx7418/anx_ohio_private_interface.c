#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/rwlock_types.h>
#include <linux/completion.h>
#include <linux/power/htc_battery.h>
#include "anx_ohio_private_interface.h"
#include "anx_ohio_public_interface.h"

/* init setting for TYPE_PWR_SRC_CAP */
static u32 init_src_caps[1] = {
	/*5V, 1.5A, Fixed */
	//PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_1500MA, PDO_FIXED_FLAGS)
	/*5V, 0.9A, Fixed */
	PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_900MA, PDO_FIXED_FLAGS)
};

/* init setting for TYPE_PWR_SNK_CAP */
static u32 init_snk_cap[/*3*/ 1] = {
	/*5V, 0.9A, Fixed */
	PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_900MA, PDO_FIXED_FLAGS),
	/*min 5V, max 20V, power 60W, battery */
	//PDO_BATT(PD_VOLTAGE_5V, PD_MAX_VOLTAGE_21V, PD_POWER_15W),
	/*min5V, max 5V, current 3A, variable */
	//PDO_VAR(PD_VOLTAGE_5V, PD_MAX_VOLTAGE_21V, PD_CURRENT_3A)
};
/* init setting for TYPE_SVID */
//static u8 init_svid[4] = { 0x00, 0x00, 0x01, 0xff };
static u8 init_svid[PD_ONE_DATA_OBJECT_SIZE] = { 0x00, 0x00, 0x00, 0x00 };

/* ID Header VDO (USB PD v1.1 spec 6.4.4.3.1.1)
	|B31	|B30	|B29..27		|B26	|B25..16	|B15..0	|
	|Host	|Device	|Prd Type	|Mod Op	|zero	|VID		|
	|1		|1		|010		|0		|0...		|0x0bb4	|
												|0000 1011 1011 0100|
	{0xb4, 0x0b, 0x00, 0xd0}

	Prd Type (Product Type):
			010b - Peripheral
*/
/* init setting for TYPE_DP_SNK_IDENTITY */
//static u8 init_snk_ident[16] = {	0x00, 0x00, 0x00, 0xec,	/*snk_id_hdr */
//									0x00, 0x00, 0x00, 0x00,	/*snk_cert */
//									0x00, 0x00, 0x00, 0x00,	/*snk_prd*/
//									0x39, 0x00, 0x00, 0x51	/*snk_ama*/
//};
static u8 init_snk_ident[12] = {	0xb4, 0x0b, 0x00, 0xd0,	/*snk_id_hdr */
									0x00, 0x00, 0x00, 0x00,	/*snk_cert */
									0x00, 0x00, 0x00, 0x00,	/*snk_prd*/
};

u8 pd_src_pdo_cnt = 2;
u8 pd_src_pdo[VDO_SIZE] = {
	/*5V 0.9A , 5V 1.5 */
	0x5A, 0x90, 0x01, 0x2A, 0x96, 0x90, 0x01, 0x2A
};

u8 pd_snk_pdo_cnt = 3;
u8 pd_snk_pdo[VDO_SIZE];
u8 pd_rdo[PD_ONE_DATA_OBJECT_SIZE*PD_MAX_DATA_OBJECT_NUM];
u8 DP_caps[PD_ONE_DATA_OBJECT_SIZE];
u8 configure_DP_caps[PD_ONE_DATA_OBJECT_SIZE];
u8 src_dp_caps[PD_ONE_DATA_OBJECT_SIZE];
unsigned char downstream_pd_cap = 0;

#define INTR_MASK_SETTING 0x0
/* circular buffer driver */
unsigned char InterfaceSendBuf[32];
unsigned char InterfaceRecvBuf[32];

#define MAX_SEND_BUF_SIZE 8
#define MAX_RECV_BUF_SIZE 8

unsigned char pbuf_rx_front = 0;
unsigned char pbuf_tx_rear = 0;

#define TX_BUF_FRONT 0x11
#define TX_BUF_REAR   0x12
#define TX_BUF_START 0x18	/* 0x18-0x1f */

#define RX_BUF_FRONT 0x13
#define RX_BUF_REAR   0x14
#define RX_BUF_START  0x20	/* 0x20-0x27 */

#define RCVDER_ACK_STATUS 0x15
#define SENDER_ACK_STATUS 0x16


#define tx_buf_rear() pbuf_tx_rear
#define rx_buf_front() pbuf_rx_front

#define rx_buf_rear() OhioReadReg(RX_BUF_REAR)
#define tx_buf_front() OhioReadReg(TX_BUF_FRONT)

#define receiver_set_ack_status(val) OhioWriteReg(RCVDER_ACK_STATUS, val)
#define receiver_get_ack_status() OhioReadReg(RCVDER_ACK_STATUS)

#define sender_get_ack_status() ((OhioReadReg(SENDER_ACK_STATUS)) & 0x7F)
#define sender_set_ack_status(val) OhioWriteReg(SENDER_ACK_STATUS, val)

extern void dwc3_otg_set_id_state(int id);
extern void dwc3_pd_vbus_ctrl(int on);
extern int dwc3_pd_drswap(int new_role);
extern int usb_set_dwc_property(int prop_type,unsigned int value);
extern void usb_downgrade_func(void);

extern struct switch_dev sw_rara_cable;
extern bool is_RaRaCable;
extern void SendRaRaEvent(bool value);

DECLARE_COMPLETION(pr_swap_rsp);
DECLARE_COMPLETION(dr_swap_rsp);

#define TRY_ROLE_TIMEOUT  600
#define WAIT_VBUS_TIME  1200
/**
 * @desc:   Interface that change Ohio as source to charge downstream device
 *
 *
 * @return:  0: success.  1: fail
 *
 */
u8 try_source(void)
{
	unsigned long expire = 0;
	pr_info("Try source start.\n");
	if(!(OhioReadReg(0x40)&0x08)) {
		pr_info("Current role is DFP, no need Try source\n");
		return 1;
	}

	if(downstream_pd_cap) {
		//return interface_pr_swap();
		reinit_completion(&pr_swap_rsp);
		reinit_completion(&dr_swap_rsp);
		send_pd_msg(TYPE_PSWAP_REQ, NULL, 0);
		ohio_set_data_value(OHIO_MODE_CHANGE, SNK_TO_SRC);
		if (wait_for_completion_interruptible_timeout(&pr_swap_rsp, msecs_to_jiffies(1500))) {
			pr_info("%s: PR_SWAP result = %u\n", __func__, pr_swap_rsp.done);
			ohio_set_data_value(OHIO_MODE_CHANGE, PR_NOCHANGE);
			msleep(500); //delay 500ms to ensure power negotiation complete
			if (ohio_get_data_value(OHIO_DROLE) == DR_DEVICE) {
				send_pd_msg(TYPE_DSWAP_REQ, NULL, 0);
			}
			else
				pr_info("current data role != DR_DEVICE, skip DR_SWAP\n");
#if 0
			if (wait_for_completion_interruptible_timeout(&dr_swap_rsp, msecs_to_jiffies(500))) {
				pr_info("%s: dr_swap result = %u\n", __func__, dr_swap_rsp.done);
				ohio->pmode = MODE_DFP;
			}
			else
				pr_info("%s: dr_swap timeout\n", __func__);
#endif
			ohio_set_data_value(OHIO_PMODE, MODE_DFP);
			dual_role_instance_changed(ohio_get_dual_role_instance()); //send uevent
			return 0;
		}
		else {
			pr_err("%s: PR_SWAP timeout\n", __func__);
			return 1;
		}
	}
	else
	{
		OhioWriteReg(0x05,0x10);  	 // OCM reset
		OhioWriteReg(0x4a,OhioReadReg(0x4a)|0x01); //cc_soft_en
		//OhioWriteReg(0x47,0x8d);  	 // pull up to 4.7K
		OhioWriteReg(0x6e,0x01);  	 // skip check vbus disable

		OhioWriteReg(0x40, OhioReadReg(0x40)|0x02); //try DFP
		expire = msecs_to_jiffies(TRY_ROLE_TIMEOUT) + jiffies;

		while(!(OhioReadReg(0x48)&0x0f)) {
			if (time_before(expire, jiffies)) {
				pr_err("Try source timeout!0x48 = %x\n", OhioReadReg(0x48));
				goto try_src_fail;
			}
		}
#ifdef SUP_VBUS_CTL
		ohio_vbus_control(1);
#else
		dwc3_pd_vbus_ctrl(1);
		ohio_set_data_value(OHIO_PROLE, PR_SOURCE);
#endif
		OhioWriteReg(0x05,0x0);  //release ocm
		mdelay(50);
		OhioWriteReg(0x6e,0x0);  	 // skip check vbus enable
		if(OhioReadReg(0x40)&0x08) {
			pr_info("try source swap fail! \n");
			return 1;		      //role swap ng
		}
		else {
			pr_info("try source swap success! \n");
			//pd_drole_change_default_func(1);
			enable_drole_work_func(1);
			ohio_set_data_value(OHIO_PMODE, MODE_DFP);
			dual_role_instance_changed(ohio_get_dual_role_instance()); //send uevent
			return 0; 			// role swap ok
		}
	}
try_src_fail:
	pr_info("try source fail!\n");
	OhioWriteReg(0x40, OhioReadReg(0x40)&0xfd);  //recover to UFP
	OhioWriteReg(0x05,0x00);  //release ocm
	mdelay(1);
	OhioWriteReg(0x69,0x19);  //vbus off delay time 100ms
	OhioWriteReg(0x6e,0x0);  	 // skip check vbus enable
	return 1;

}

/**
 * @desc:   Interface that change Ohio as sink  for charging  from downstream device
 *
 *
 * @return:  0: success.  1: fail
 *
 */
u8 try_sink(void)
{
	unsigned long expire = 0;
	pr_info("Try sink start.\n");
	if(OhioReadReg(0x40)&0x08) {
		pr_info("Current role is UFP, no need Try sink\n");
		return 1;
	}

	if(downstream_pd_cap) {
		//return interface_pr_swap();
		reinit_completion(&pr_swap_rsp);
		reinit_completion(&dr_swap_rsp);
		send_pd_msg(TYPE_PSWAP_REQ, NULL, 0);
		ohio_set_data_value(OHIO_MODE_CHANGE, SRC_TO_SNK);
		if (wait_for_completion_interruptible_timeout(&pr_swap_rsp, msecs_to_jiffies(1500))) {
			pr_info("%s: PR_SWAP result = %u\n", __func__, pr_swap_rsp.done);
			ohio_set_data_value(OHIO_MODE_CHANGE, PR_NOCHANGE);
			msleep(800); // do we need this delay?
			if (ohio_get_data_value(OHIO_DROLE) == DR_HOST) {
				send_pd_msg(TYPE_DSWAP_REQ, NULL, 0);
			}
			else
				pr_info("current data role != DR_HOST, skip DR_SWAP\n");
#if 0
			if (wait_for_completion_interruptible_timeout(&dr_swap_rsp, msecs_to_jiffies(500))) {
				pr_info("%s: dr_swap result = %u\n", __func__, dr_swap_rsp.done);
				ohio->pmode = MODE_UFP;
			}
			else
				pr_info("%s: dr_swap timeout\n", __func__);
#endif
			ohio_set_data_value(OHIO_PMODE, MODE_UFP);
			return 0;
		}
		else {
			pr_err("%s: PR_SWAP timeout\n", __func__);
			return 1;
		}
	}
	else {
		OhioWriteReg(0x05,0x10);  	 // OCM reset
		OhioWriteReg(0x4a,OhioReadReg(0x4a) | 0x01); //cc_soft_en
#ifdef SUP_VBUS_CTL
		ohio_vbus_control(0);	//VBUS off
#else
		dwc3_pd_vbus_ctrl(0);
		ohio_set_data_value(OHIO_PROLE, PR_SINK);
#endif
		OhioWriteReg(0x3f,OhioReadReg(0x3f) | 0xdf); //VBUS off
		OhioWriteReg(0x36,OhioReadReg(0x36) | 0x80);//discharge en
		//OhioWriteReg(0x33,OhioReadReg(0x33) & 0x0f);//Vconn colse

		OhioWriteReg(0x40, OhioReadReg(0x40) & 0xfd);  //try UFP

		expire = msecs_to_jiffies(TRY_ROLE_TIMEOUT) + jiffies;
		//expire = msecs_to_jiffies(300) + jiffies;

		while(!(OhioReadReg(0x0d) & 0xfc)) {
			if (time_before(expire, jiffies)) {
				pr_info("Try sink timeout!0x0d = %x\n", OhioReadReg(0x0d));
				goto try_sink_fail;
			}
		}

		mdelay(650);
		//mdelay(300);
		expire = msecs_to_jiffies(WAIT_VBUS_TIME) + jiffies;
		//expire = msecs_to_jiffies(500) + jiffies;

		while(!(OhioReadReg(0x40) & 0x10)) {
			if (time_before(expire, jiffies)) {
				pr_info("wait vbus timeout!0x40 = %x\n", OhioReadReg(0x40));
				goto try_sink_fail;
			}
		}

		OhioWriteReg(0x36,OhioReadReg(0x36) & 0x7f); //discharge disable
		OhioWriteReg(0x05,0x00);  //release FW
		if(OhioReadReg(0x40)&0x08) {
			pr_info("try sink swap success! \n");
			msleep(500);
			//pd_drole_change_default_func(0);
			enable_drole_work_func(0);
			ohio_set_data_value(OHIO_PMODE, MODE_UFP);
			dual_role_instance_changed(ohio_get_dual_role_instance()); //send uevent
			return 0;		      //role swap ok
		}
		else {
			pr_info("try sink  swap fail! \n");
			return 1; 			  // role swap ng
		}
	}
try_sink_fail:
	pr_err("try sink fail!\n");
	OhioWriteReg(0x40, OhioReadReg(0x40)|0x02);  //recover to DFP
	OhioWriteReg(0x36,OhioReadReg(0x36) & 0x7f); //discharge disable
	OhioWriteReg(0x05,0x00);
	return 1;

}


/**
 * @desc:   Interface AP fetch OTP word 1 byte 4,
 *
 *
 * @return:
 *
 */
u8 get_otp_indicator_byte(void)
{
	u8 temp;

	OhioWriteReg(0xe5, 0xa0); //  disable ocm access otp & wake up otp
	OhioWriteReg(0xef, 0x7a); // OTP access key
	OhioWriteReg(0xd0, 0x00);   //high address
	OhioWriteReg(0xd1, 0x01); 		//low address
	OhioWriteReg(0xe5, 0xa1); // otp read start
	while(OhioReadReg(0xed) & 0x30); // wait for read done
	temp = OhioReadReg(0xe0);
	OhioWriteReg(0xef, 0x0); // OTP access key clear
	OhioWriteReg(0xe5, 0x0); //enable ocm
	return temp;
}

/**
 * @desc:   The interface AP will get the ohio's data role
 *
 * @param:  none
 *
 * @return:  data role , dfp 1 , ufp 0, other error: -1, not ready
 *
 */
s8 get_data_role(void)
{
	u8 status;

	/*fetch the data role */
	status = OhioReadReg(OHIO_SYSTEM_STSTUS);

	return ((status & DATA_ROLE) != 0);

}

/**
 * @desc:   The interface AP will get the ohio's power role
 *
 * @param:  none
 *
 * @return:  data role , source 1 , sink 0, other error, -1, not ready
 *
 */
s8 get_power_role(void)
{
	u8 status ;

	/*fetch the power role */
	status = OhioReadReg(0x40);

	return ((status & 0x08) == 0);
}

/**
 * @desc:   Interface AP fetch the source capability from Ohio
 *
 * @param:  pdo_buf: PDO buffer pointer of source capability in Ohio
 *          src_caps_size: source capability's size
 *
 * @return:  0: success 1: fail
 *
 */
u8 get_src_cap(const u8 *src_caps, u8 src_caps_size)
{

	return 1;
}

/**
 * @desc:   Interface that AP fetch the sink capability from Ohio's downstream device
 *
 * @param:  sink_caps: PDO buffer pointer of sink capability
 *            which will be responsed by Ohio's SINK Capablity Message
 *
 *          snk_caps_len: sink capability max length of the array
 *
 * @return:  sink capability array length>0: success.  1: fail
 *
 */
u8 get_snk_cap(u8 *snk_caps, u8 snk_caps_len)
{

	return 1;
}


/**
 * @desc:   The Interface AP set the source capability to Ohio
 *
 * @param:  pdo_buf: PDO buffer pointer of source capability,
 *                              which can be packed by PDO_FIXED_XXX macro
 *                eg: default5Vsafe src_cap(5V, 0.9A fixed) -->
 *			PDO_FIXED(5000,900, PDO_FIXED_FLAGS)
 *
 *                src_caps_size: source capability's size
 *
 * @return:  0: success 1: reject, 2: fail, 3: busy
 *
 */
u8 send_src_cap(const u8 *src_caps, u8 src_caps_size)
{
	if (NULL == src_caps)
		return CMD_FAIL;
	if ((src_caps_size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
	    (src_caps_size / PD_ONE_DATA_OBJECT_SIZE) >
	    PD_MAX_DATA_OBJECT_NUM) {
		return CMD_FAIL;
	}
	memcpy(pd_src_pdo, src_caps, src_caps_size);
	pd_src_pdo_cnt = src_caps_size / PD_ONE_DATA_OBJECT_SIZE;

	/*send source capabilities message to Ohio really */
	return interface_send_msg_timeout(TYPE_PWR_SRC_CAP, pd_src_pdo,
					  pd_src_pdo_cnt *
					  PD_ONE_DATA_OBJECT_SIZE,
					  INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure) the sink capability to Ohio's downstream device
 *
 * @param:  snk_caps: PDO buffer pointer of sink capability
 *
 *                snk_caps_size: sink capability length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_snk_cap(const u8 *snk_caps, u8 snk_caps_size)
{
	memcpy(pd_snk_pdo, snk_caps, snk_caps_size);
	pd_snk_pdo_cnt = snk_caps_size / PD_ONE_DATA_OBJECT_SIZE;

	/*configure sink cap */
	return interface_send_msg_timeout(TYPE_PWR_SNK_CAP, pd_snk_pdo,
					  pd_snk_pdo_cnt * 4,
					  INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure) the DP's sink capability to Ohio's downstream device
 *
 * @param:  dp_snk_caps: PDO buffer pointer of DP sink capability
 *
 *                dp_snk_caps_size: DP sink capability length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_dp_snk_cfg(const u8 *dp_snk_caps, u8 dp_snk_caps_size)
{
	memcpy(configure_DP_caps, dp_snk_caps, dp_snk_caps_size);
	/*configure sink cap */
	return interface_send_msg_timeout(TYPE_DP_SNK_CFG, configure_DP_caps,
					   4, INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP initialze the DP's capability of Ohio, as source device
 *
 * @param:  dp_caps: DP's capability  pointer of source
 *
 *                dp_caps_size: source DP capability length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_src_dp_cap(const u8 *dp_caps, u8 dp_caps_size)
{
	if (NULL == dp_caps)
		return CMD_FAIL;
	if ((dp_caps_size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
	    (dp_caps_size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
		return CMD_FAIL;
	}

	memcpy(src_dp_caps, dp_caps, dp_caps_size);

	/*configure source DP cap */
	return interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY,
					  src_dp_caps, dp_caps_size,
					  INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP initialze the DP's sink identity of Ohio, as sink device
 *
 * @param:  snk_ident: DP's sink identity
 *
 *                snk_ident_size: DP's sink identity length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_dp_snk_identity(const u8 *snk_ident, u8 snk_ident_size)
{
	return interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY,
					  (u8 *) snk_ident, snk_ident_size,
					  INTERFACE_TIMEOUT);
}

/**
 * @desc:   The Interface AP set the VDM packet to Ohio
 *
 * @param:  vdm:  object buffer pointer of VDM
 *
 *                size: vdm packet size
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_vdm(const u8 *vdm, u8 size)
{
	u8 tmp[32] = { 0 };
	if (NULL == vdm)
		return CMD_FAIL;
	if (size > 3 && size < 32) {
		memcpy(tmp, vdm, size);
		if (tmp[2] == 0x01 && tmp[3] == 0x00) {
			tmp[3] = 0x40;
			return interface_send_msg_timeout(TYPE_VDM, tmp, size,
							  INTERFACE_TIMEOUT);
		}
	}
	return 1;
}

/**
 * @desc:   The Interface AP set the SVID packet to Ohio
 *
 * @param:  svid:  object buffer pointer of svid
 *
 *                size: svid packet size
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_svid(const u8 *svid, u8 size)
{
	u8 tmp[4] = {
		0
	};
	if (NULL == svid || size != 4)
		return CMD_FAIL;
	memcpy(tmp, svid, size);
	return interface_send_msg_timeout(TYPE_SVID, tmp, size,
					  INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure) the sink capability to Ohio's downstream device
 *
 * @param:  snk_caps: PDO buffer pointer of sink capability
 *
 *                snk_caps_size: sink capability length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_rdo(const u8 *rdo, u8 size)
{
	u8 i;
	if (NULL == rdo)
		return CMD_FAIL;
	if ((size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
	    (size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
		return CMD_FAIL;
	}
	for (i = 0; i < size; i++)
		pd_rdo[i] = *rdo++;

	return interface_send_msg_timeout(TYPE_PWR_OBJ_REQ, pd_rdo, size,
					  INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send  PR_Swap command to Ohio
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_power_swap(void)
{
	return interface_pr_swap();
}

/**
 * @desc:   The interface AP will send DR_Swap command to Ohio
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_data_swap(void)
{
	return interface_dr_swap();
}

/**
 * @desc:   The interface AP will send accpet command to Ohio
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_accept(void)
{
	return interface_send_msg_timeout(TYPE_ACCEPT, 0, 0, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send reject command to Ohio
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_reject(void)
{
	return interface_send_msg_timeout(TYPE_REJECT, 0, 0, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send soft reset command to Ohio
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_soft_reset(void)
{
	return interface_send_soft_rst();
}

/**
 * @desc:   The interface AP will send hard reset command to Ohio
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_hard_reset(void)
{
	return interface_send_hard_rst();
}

char *interface_to_str(unsigned char header_type)
{
	return (header_type == TYPE_PWR_SRC_CAP) ? "src cap" :
	    (header_type == TYPE_PWR_SNK_CAP) ? "snk cap" :
	    (header_type == TYPE_PWR_OBJ_REQ) ? "RDO" :
	    (header_type == TYPE_DP_SNK_IDENTITY) ? "snk identity" :
	    (header_type == TYPE_SVID) ? "svid" :
	    (header_type == TYPE_PSWAP_REQ) ? "PR_SWAP" :
	    (header_type == TYPE_DSWAP_REQ) ? "DR_SWAP" :
	    (header_type == TYPE_GOTO_MIN_REQ) ? "GOTO_MIN" :
	    (header_type == TYPE_DP_ALT_ENTER) ? "DPALT_ENTER" :
	    (header_type == TYPE_DP_ALT_EXIT) ? "DPALT_EXIT" :
	    (header_type == TYPE_VCONN_SWAP_REQ) ? "VCONN_SWAP" :
	    (header_type == TYPE_GET_DP_SNK_CAP) ? "GET_SINK_DP_CAP" :
	    (header_type == TYPE_DP_SNK_CFG) ? "dp cap" :
	    (header_type == TYPE_SOFT_RST) ? "Soft Reset" :
	    (header_type == TYPE_HARD_RST) ? "Hard Reset" :
	    (header_type == TYPE_RESTART) ? "Restart" :
	    (header_type == TYPE_PD_STATUS_REQ) ? "PD Status" :
	    (header_type == TYPE_ACCEPT) ? "ACCEPT" :
	    (header_type == TYPE_REJECT) ? "REJECT" :
	    (header_type == TYPE_VDM) ? "VDM" :
	    (header_type ==
	     TYPE_RESPONSE_TO_REQ) ? "Response to Request" : "Unknown";
}

inline unsigned char cac_checksum(unsigned char *pSendBuf, unsigned char len)
{
	unsigned char i;
	unsigned char checksum;
	checksum = 0;
	for (i = 0; i < len; i++)
		checksum += *(pSendBuf + i);

	return (u8) (0 - checksum);
}

void printb(const char *buf, size_t size)
{
#ifdef OHIO_DEBUG
	while (size--)
		printk("%0x ", *buf++);
	printk("\n");
#endif
}

void interface_init(void)
{
	pbuf_rx_front = 0;
	pbuf_tx_rear = 0;
	downstream_pd_cap = 0;
}

void send_initialized_setting(void)
{

	/* send TYPE_PWR_SRC_CAP init setting */
	send_pd_msg(TYPE_PWR_SRC_CAP, (const char *)init_src_caps,
				sizeof(init_src_caps));

	/* send TYPE_PWR_SNK_CAP init setting */
	send_pd_msg(TYPE_PWR_SNK_CAP, (const char *)init_snk_cap,
				sizeof(init_snk_cap));

	/* send TYPE_DP_SNK_IDENTITY init setting */
	send_pd_msg(TYPE_DP_SNK_IDENTITY, init_snk_ident,
				sizeof(init_snk_ident));

	/* send TYPE_SVID init setting */
	send_pd_msg(TYPE_SVID, init_svid, sizeof(init_svid));
}

void chip_register_init(void)
{
	u8 val;
#ifdef PD_CTS_TEST
	/* modify the below registers for CTS test as specific platform */

	/* tVbusOFF timer :Delay time for pulling down Cable Det pin when CC disconnected
	  Timer = (this value * 4)ms */
	 //OhioWriteReg(VBUS_DELAY_TIME, 0x7d); // 0x7d -> 500ms

	/*HARD_RESET Delay time for OCM
	   Real value: 30ms - (T_TIME_1 & 0x0F)*/
	//OhioWriteReg(T_TIME_1, 0x07);
#else
	/*OHO-439, in AP side, for the interoperability,
	  set the try.UFP period to 0x96*2 = 300ms. */
	OhioWriteReg(TRY_UFP_TIMER, 0x96);
#endif

	val = OhioReadReg(0x44); //ANALOG_CTRL_3
	if (val >= 0x12 && val != 0x16) {
		// for 7418 to 7418 PR_SWAP issue, set this value to PS_RDY timing as 150ms
		val = OhioReadReg(0x6B);
		val = (val & 0xFC) | 0x02;
		OhioWriteReg(0x6B, val);
		pr_debug("%s: set register 0x6B => 0x%02X\n", __func__, val);
	}

#ifdef SUP_TRY_SRC_SINK
	OhioWriteReg(VBUS_DELAY_TIME, 0x19);  // 0x19 -> 100ms
#endif

#ifdef SUP_OHIO_INT_VECTOR
	/* set interrupt vector mask bit as platform needed  0: eable 1: disable*/
	OhioWriteReg(OHIO_INTERFACE_INTR_MASK, INTR_MASK_SETTING);
#else
	OhioWriteReg(OHIO_INTERFACE_INTR_MASK, 0xff);
#endif

#ifdef AUTO_RDO_ENABLE
	OhioWriteReg(AUTO_PD_MODE, OhioReadReg(AUTO_PD_MODE) | AUTO_PD_ENABLE);
	/*Maximum Voltage in 100mV units*/
	OhioWriteReg(MAX_VOLTAGE_SETTING, 0x32);  /* 5V */
	/*Maximum Power in 500mW units*/
	OhioWriteReg(MAX_POWER_SETTING, 0x0f);	 /* 7.5W */
	/*Minimum Power in 500mW units*/
	OhioWriteReg(MIN_POWER_SETTING, 0x002);  /* 1W */
#endif
}

inline void reciever_reset_queue(void)
{
	rx_buf_front() = rx_buf_rear();
	OhioWriteReg(RX_BUF_FRONT, rx_buf_front());
}

void handle_intr_vector(void)
{
	u8 intr_vector;
	u8 status;
	int ret;

	ret = ohio_read_reg(OHIO_SLAVE_I2C_ADDR, OHIO_INTERFACE_CHANGE_INT, (unchar *)(&intr_vector));
	if (ret < 0) {
		pr_err("%s: failed to read register %x\n", __func__, OHIO_INTERFACE_CHANGE_INT);
		return;
	}

#ifdef OHIO_DEBUG
	pr_info("%s: intr vector = %x\n", __func__, intr_vector);
#endif
	if (intr_vector) {
		OhioWriteReg(OHIO_INTERFACE_CHANGE_INT, intr_vector & (~intr_vector));
		clear_soft_interrupt();
		if ((~INTR_MASK_SETTING) & intr_vector & RECEIVED_MSG)
			polling_interface_msg(INTERACE_TIMEOUT_MS);
		if ((~INTR_MASK_SETTING) & intr_vector & VBUS_CHANGE) {
			status = OhioReadReg(OHIO_SYSTEM_STSTUS);
			pd_vbus_control_default_func(status & VBUS_STATUS);
		}
		if ((~INTR_MASK_SETTING) & intr_vector & VCONN_CHANGE) {
			status = OhioReadReg(OHIO_SYSTEM_STSTUS);
			pd_vconn_control_default_func(status & VCONN_STATUS);
		}
		if ((~INTR_MASK_SETTING) & intr_vector & CC_STATUS_CHANGE) {
			status = OhioReadReg(NEW_CC_STATUS);
			pd_cc_status_default_func(status);
		}
		if ((~INTR_MASK_SETTING) & intr_vector & DATA_ROLE_CHANGE) {
			status = OhioReadReg(OHIO_SYSTEM_STSTUS);
			//pd_drole_change_default_func(status & DATA_ROLE);
			enable_drole_work_func(status & DATA_ROLE);
		}
	}
}

/* in Tx's side routine check timeout & cable down */
#define TX_ROUTINE_CHECK() do {\
	if (time_before(expire, jiffies)) {\
		pr_info("TX Timeout %d\n", msg_total_len);\
		return CMD_FAIL;\
	}	\
} while (0)
#define RX_ROUTINE_CHECK() do {\
	if (time_before(expire, jiffies)) {\
		goto err_timeout;\
	}	\
} while (0)

/* Desc: send interface message to ocm
 * Args: timeout,  block timeout time
 @return:  0: success, 1: reject, 2: fail, 3: busy
 */
u8 interface_send_msg_timeout(u8 type, u8 *pbuf, u8 len, int timeout_ms)
{
	int msg_total_len = 0;
	unsigned long expire = 0;
	u8 tmp_len = 0;
	s8 rear, front;
	u8 buf[32] = { 0 };

	/* full, return 0 */
	buf[0] = len + 1;	/* cmd */
	buf[1] = type;
	memcpy(buf + 2, pbuf, len);
	/* cmd + checksum */
	buf[len + 2] = cac_checksum(buf, len + 1 + 1);
	msg_total_len = len + 3;
	rear = tx_buf_rear();

#ifdef OHIO_DEBUG
	pr_info("snd type=%d len=%d\n", type, msg_total_len);
#endif

	expire = msecs_to_jiffies(timeout_ms) + jiffies;
	while (msg_total_len > 0) {
		front = tx_buf_front();
		if (front == rear) {
			tmp_len = 7;

			if (tmp_len > msg_total_len)
					tmp_len = msg_total_len;

			if ((TX_BUF_START + rear + tmp_len) > (TX_BUF_START + MAX_SEND_BUF_SIZE -1)) {
				OhioWriteBlockReg(TX_BUF_START + rear, MAX_SEND_BUF_SIZE - rear, buf + len + 3 - msg_total_len);
				if(tmp_len - (MAX_SEND_BUF_SIZE - rear))
					OhioWriteBlockReg(TX_BUF_START, tmp_len - (MAX_SEND_BUF_SIZE - rear), buf + len + 3 - msg_total_len + (MAX_SEND_BUF_SIZE - rear));
			}
			else
				OhioWriteBlockReg(TX_BUF_START + rear, tmp_len, buf + len + 3 - msg_total_len);

			msg_total_len -= tmp_len;
			//update rear position
			rear = (rear + tmp_len) % MAX_SEND_BUF_SIZE;
			OhioWriteReg(TX_BUF_REAR, rear);
#ifdef OHIO_DEBUG
			pr_info("Tx len=%d remainder=%d, front=%d, rear=%d\n", tmp_len, msg_total_len, front, rear);
#endif
		}
		TX_ROUTINE_CHECK();
	}
	tx_buf_rear() = rear;

	expire = msecs_to_jiffies(timeout_ms) + jiffies;
	while ((msg_total_len = sender_get_ack_status()) == 0) {
		TX_ROUTINE_CHECK();
	}
	if (msg_total_len == 0x01) {
		/* HTC: print with "debug" log level if the message is
		 * "src cap" "snk cap" "snk identity" "svid" to reduce the log size.
		 * Otherwise, keep the "info" log level
		 */
		if (buf[1] < 0x04)
			pr_debug("succ << %s\n", interface_to_str(buf[1]));
		else
			pr_info("succ << %s\n", interface_to_str(buf[1]));
		// printb(buf, len + 3);
		return CMD_SUCCESS;
	} else {
		pr_info("Ack error %d\n", msg_total_len);
		printb(buf, len + 3);
		sender_set_ack_status(0x00);
		return CMD_FAIL;
	}
	return CMD_SUCCESS;
}

/* Desc: polling private interface interrupt request message
 * Args: timeout,  block timeout time
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 * Interface's Format:
 *	1Byte Len + 1Byte Type + Len Bytes Data + 1Byte checksum
 */
u8 polling_interface_msg(int timeout_ms)
{
	u8 checksum = 0;
	unsigned long expire = 0;
	u8 front, rear, i, tmp_len, msg_total_len, rec_len, first_len, sec_len;
	u8 buf[32] = { 0 };

	expire = msecs_to_jiffies(timeout_ms) + jiffies;

	 /*first block read to get total length*/
	rec_len = 0;
	front = rx_buf_front();
	rear = rx_buf_rear();
	if (front != rear) {

		if (rear > front) {
			tmp_len = rear - front;
			OhioReadBlockReg(RX_BUF_START + front, tmp_len, buf + rec_len);
		}
		else {
			tmp_len = (MAX_SEND_BUF_SIZE  - (front - rear)) % MAX_SEND_BUF_SIZE;
			OhioReadBlockReg(RX_BUF_START + front, MAX_SEND_BUF_SIZE  - front, buf + rec_len);
			if(rear > 0)
				OhioReadBlockReg(RX_BUF_START, rear, buf + rec_len + MAX_SEND_BUF_SIZE  - front);
		}
		rec_len += tmp_len;
		//update front position
		front = (front + tmp_len) % MAX_SEND_BUF_SIZE;
		OhioWriteReg(RX_BUF_FRONT, front);
#ifdef OHIO_DEBUG
		pr_info("Rx len=%d Received len=%d, front=%d, rear=%d\n", tmp_len, rec_len, front, rear);
#endif
	}

	if (buf[0]  == 0 || buf[0]  > 31) {
		goto err_rcv_len;
	}
	msg_total_len = buf[0] + 2;
#ifdef OHIO_DEBUG
	pr_info("total receive message len=%d\n", msg_total_len );
#endif

	/* remain block read */
	while (msg_total_len > rec_len) {

		rear = rx_buf_rear();

		if (rear > front)
			tmp_len = rear - front;
		else
			tmp_len = (MAX_SEND_BUF_SIZE  - front + rear) % MAX_SEND_BUF_SIZE;

		if (front == ((rear + 1) % MAX_SEND_BUF_SIZE ) || (rec_len + tmp_len) >= msg_total_len) {

			if (rear > front) {

				if(tmp_len > (msg_total_len -rec_len))
					tmp_len = msg_total_len -rec_len;
				OhioReadBlockReg(RX_BUF_START + front, tmp_len, buf + rec_len);
			}
			else {
				first_len = MAX_SEND_BUF_SIZE  - front;
				sec_len = rear;

				if(tmp_len > (msg_total_len -rec_len)) {
					tmp_len = msg_total_len -rec_len;
					if(first_len > (msg_total_len -rec_len)){
						first_len = msg_total_len -rec_len;
						sec_len = 0;
					}
					else
						sec_len =msg_total_len -rec_len - first_len;
				}

				OhioReadBlockReg(RX_BUF_START + front, first_len, buf + rec_len);
				if(sec_len > 0)
					OhioReadBlockReg(RX_BUF_START, sec_len, buf + rec_len + MAX_SEND_BUF_SIZE  - front);
			}
			rec_len += tmp_len;
			//update front position
			front = (front + tmp_len) % MAX_SEND_BUF_SIZE;
			OhioWriteReg(RX_BUF_FRONT, front);
			#ifdef OHIO_DEBUG
			pr_info("Rx len=%d Received len=%d, front=%d, rear=%d\n", tmp_len, rec_len, front, rear);
			#endif
		}

		RX_ROUTINE_CHECK();
	}
	rx_buf_front() = front;
	/* checksum judgement */
	for(i= 0; i < msg_total_len; i++)
		checksum += buf[i];
	if(checksum == 0) {
		receiver_set_ack_status(0x01);
		pr_info(">>%s\n", interface_to_str(buf[1]));
		//printb(buf, msg_total_len);
		dispatch_rcvd_pd_msg((PD_MSG_TYPE) buf[1], &(buf[2]), buf[0] - 1);
		return CMD_SUCCESS;
	} else {
		receiver_set_ack_status(0x02);
		pr_info("checksum error: %x\n", (u16)checksum);
		printb(buf, rec_len);
		return CMD_FAIL;
	}

err_timeout:
	sender_set_ack_status(0x03);
	reciever_reset_queue();
	pr_info("err: RX Timeout %d\n", rec_len);
	printb(buf, rec_len);
	return CMD_FAIL;
err_rcv_len:
	pr_info("err: rx length error len %d\n", buf[0]);
	receiver_set_ack_status(0x02);
	reciever_reset_queue();
	return CMD_FAIL;
}

/* define max request current 900mA and voltage 5V */
#define MAX_REQUEST_VOLTAGE 5000
//#define MAX_REQUEST_CURRENT 900
#define MAX_REQUEST_CURRENT 2000
#define set_rdo_value(v0, v1, v2, v3)	\
	do {				\
		pd_rdo[0] = (v0);	\
		pd_rdo[1] = (v1);	\
		pd_rdo[2] = (v2);	\
		pd_rdo[3] = (v3);	\
	} while (0)

u8 sel_voltage_pdo_index = 0x02;
/* default request max RDO */
/*
 *	Fixed and Variable Request Data Object
 *	Low bit ----------------------------------------------------------------------------------- High bit
 *	|B0..9			|B10..19			|B20..23	|B24	|B25		|B26	|B27		|B28..30	|B31	|
 *	|MaxOpCur		|OpCur			|Resv	|NoSus	|USBComm	|CapMis	|GiveBack	|ObjPos	|Resv	|
 *	|0101 1010 00	|01 0110 1000	|0000	|0		|0			|0		|0			|???		|0		|
 *	
 *	(Max)OpCur: 10mA units
 *		900mA -> 90 == 0x5a (high bit - 00 0101 1010 - low bit)
 *
 */
u8 build_rdo_from_source_caps(u8 obj_cnt, u8 *buf)
{
	u8 i = 0;
	u16 pdo_h, pdo_l, pdo_h_tmp, pdo_l_tmp;
	u16 max_request_ma;
	u32 pdo_max, pdo_max_tmp;

	struct htc_pd_data pdo_data;
	int device_max_ma = 0;

	pdo_max = 0;
	obj_cnt &= 0x07;

	/* find the max voltage pdo */
	for (i = 0; i < obj_cnt; i++) {
		if (i > 5) {    // max number of PDO = 6
			obj_cnt = 6;
			break;
		}
		pdo_l_tmp = buf[i * 4 + 0];
		pdo_l_tmp |= (u16) buf[i * 4 + 1] << 8;
		pdo_h_tmp = buf[i * 4 + 2];
		pdo_h_tmp |= (u16) buf[i * 4 + 3] << 8;

		/* get max voltage now */
		pdo_max_tmp =
		    (u16) (((((pdo_h_tmp & 0xf) << 6) | (pdo_l_tmp >> 10)) &
			    0x3ff) * 50);

		pdo_data.pd_list[i][0] = pdo_max_tmp;	// voltage (mV)
		pdo_data.pd_list[i][1] = (int)((pdo_l_tmp & 0x3ff) * 10);  // current (mA)

#if 0
		if (pdo_max_tmp > pdo_max) {
			pdo_max = pdo_max_tmp;
			pdo_l = pdo_l_tmp;
			pdo_h = pdo_h_tmp;
			sel_voltage_pdo_index = i;
		}
#endif
	}

	// call battery api
	sel_voltage_pdo_index = (u8) htc_battery_pd_charger_support(obj_cnt,
								pdo_data, &device_max_ma);
	pr_info("device_max_ma=%d, cnt %d index %d\n", device_max_ma, obj_cnt,
								sel_voltage_pdo_index);
	if (sel_voltage_pdo_index < 0) {
		pr_info("RDO Mismatch !!!\n");
		set_rdo_value(0x0A, 0x28, 0x00, 0x10);
		return 0;
	}

	pdo_l = buf[sel_voltage_pdo_index * 4 + 0];
	pdo_l |= (u16) buf[sel_voltage_pdo_index * 4 + 1] << 8;
	pdo_h = buf[sel_voltage_pdo_index * 4 + 2];
	pdo_h |= (u16) buf[sel_voltage_pdo_index * 4 + 3] << 8;

	max_request_ma = device_max_ma;

	// not Battery RDO
	//TODO: currently not support Variable RDO
	if ((pdo_h & (3 << 14)) != (PDO_TYPE_BATTERY >> 16)) {
		usb_set_dwc_property(PROPERTY_CURRENT_MAX, (unsigned int)max_request_ma * 1000);
		// if selected current is lower than 1000mA
		// set mismatch bit and Maximum Operating Current is set to 2000mA
		if (pdo_data.pd_list[sel_voltage_pdo_index][1] < 1000) {
			pdo_max = RDO_FIXED(sel_voltage_pdo_index + 1,
								max_request_ma,
								MAX_REQUEST_CURRENT, 0);
			pdo_max |= RDO_CAP_MISMATCH;
			set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
							(pdo_max >> 16) & 0xff,
							(pdo_max >> 24) & 0xff);
			pr_info("FIXED PDO: selected current value < 1000mA, +cap_mismatch\n");
			return 1;
		}
		else {	// selected current is 1000mA+
			pdo_max = RDO_FIXED(sel_voltage_pdo_index + 1,
								max_request_ma,
								max_request_ma,
								0);
			set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
							(pdo_max >> 16) & 0xff,
							(pdo_max >> 24) & 0xff);
			pr_info("FIXED PDO: selected current value >= 1000mA, send to PD charger\n");
			return 1;
		}
	}
	else {	// Battery Request Data Object
		usb_set_dwc_property(PROPERTY_CURRENT_MAX, MAX_REQUEST_CURRENT * 1000);
		pdo_max = RDO_FIXED(sel_voltage_pdo_index + 1,
							MAX_REQUEST_CURRENT,
							MAX_REQUEST_CURRENT, 0);
		set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
					(pdo_max >> 16) & 0xff, (pdo_max >> 24) & 0xff);
		return 1;
	}

#if 0  // anx original code
#ifdef OHIO_DEBUG
	pr_info("maxV=%d, cnt %d index %d\n", pdo_max_tmp, obj_cnt,
		sel_voltage_pdo_index);
#endif
	if ((pdo_h & (3 << 14)) != (PDO_TYPE_BATTERY >> 16)) {
		max_request_ma = (u16) ((pdo_l & 0x3ff) * 10);
#ifdef OHIO_DEBUG
		pr_info("maxMa %d\n", max_request_ma);
#endif
		/* less than 900mA */
		if (max_request_ma < MAX_REQUEST_CURRENT) {
			pdo_max =
			    RDO_FIXED(sel_voltage_pdo_index + 1, max_request_ma,
				      max_request_ma, 0);
			pdo_max |= RDO_CAP_MISMATCH;
			set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
				      (pdo_max >> 16) & 0xff,
				      (pdo_max >> 24) & 0xff);
			return 1;
		} else {
			pdo_max =
			    RDO_FIXED(sel_voltage_pdo_index + 1,
				      MAX_REQUEST_CURRENT, MAX_REQUEST_CURRENT,
				      0);
			set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
				      (pdo_max >> 16) & 0xff,
				      (pdo_max >> 24) & 0xff);

			return 1;
		}
	}
	else {		// Battery Request Data Object
		pdo_max =
		    RDO_FIXED(sel_voltage_pdo_index + 1, MAX_REQUEST_CURRENT,
			      MAX_REQUEST_CURRENT, 0);
		set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
			      (pdo_max >> 16) & 0xff, (pdo_max >> 24) & 0xff);
		return 1;
	}

	pr_info("RDO Mismatch !!!\n");
	set_rdo_value(0x0A, 0x28, 0x00, 0x10);

	return 0;
#endif
}

u32 change_bit_order(u8 *pbuf)
{
    return ((u32)pbuf[3] << 24) | ((u32)pbuf[2] << 16)
        | ((u32)pbuf[1] << 8) | pbuf[0];
}
u8 pd_check_requested_voltage(u32 rdo)
{
        int max_ma = rdo & 0x3FF;
        int op_ma = (rdo >> 10) & 0x3FF;
        int idx = rdo >> 28;

        u32 pdo;
        u32 pdo_max;

        if (!idx || idx > pd_src_pdo_cnt)
        {
               pr_info("rdo = %x, Requested RDO is %d, Provided RDO number is %d\n", rdo, (unsigned int)idx, (unsigned int)pd_src_pdo_cnt);
                return 0; /* Invalid index */
        }
        //Update to pass TD.PD.SRC.E12 Reject Request
        pdo = change_bit_order(pd_src_pdo + ((idx - 1) * 4));
        pdo_max = (pdo & 0x3ff);
#ifdef OHIO_DEBUG
        pr_info("pdo_max = %x\n", pdo_max);
#endif
        //TRACE3("Requested  %d/~%d mA, idx %d\n",      (u16)op_ma * 10, (u16)max_ma *10, (u16)idx);
        /* check current ... */
        if (op_ma > pdo_max)//Update to pass TD.PD.SRC.E12 Reject Request
                return 0; /* too much op current */
        if (max_ma > pdo_max)//Update to pass TD.PD.SRC.E12 Reject Request
                return 0; /* too much max current */



        return 1;
}

/* Recieve Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_source_caps_default_callback(void *para, u8 para_len)
{
	u8 *pdo = 0;
	u8 ret = 1;
	pdo = (u8 *) para;
	if (para_len % 4 != 0)
		return ret;
	if (build_rdo_from_source_caps(para_len / 4, para)) {
		ret = interface_send_request();
		pr_debug("Snd RDO %x %x %x %x succ\n", pd_rdo[0], pd_rdo[1],
			pd_rdo[2], pd_rdo[3]);
	}
	return ret;
}

/* Recieve Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_sink_caps_default_callback(void *para, u8 para_len)
{
	u8 *pdo = 0;

	pdo = (u8 *) para;
	if (para_len % 4 != 0)
		return 0;
	if (para_len > VDO_SIZE)
		return 0;
	/*do what you want to do */
	return 1;
}

/* Recieve Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_pwr_object_req_default_callback(void *para, u8 para_len)
{
	u8 *pdo = (u8 *) para;
	u8 ret = 1;
	u32 rdo = 0;
	if (para_len != 4)
		return ret;

	rdo = pdo[0] | (pdo[1] << 8) | (pdo[2] << 16) | (pdo[3] << 24);
	if (pd_check_requested_voltage(rdo))
		ret = send_accept();
	else
		ret = interface_send_reject();

	return ret;
}

/* Recieve accept message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
  * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_accept_default_callback(void *para, u8 para_len)
{
	return 1;
}

/* Recieve reject message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
  * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_reject_default_callback(void *para, u8 para_len)
{
	return 1;
}

/* Recieve reject message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
  * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_goto_min_default_callback(void *para, u8 para_len)
{
	return 1;
}

extern int qpnp_boost_status(u8 *value);
extern int qpnp_boost_int_status(u8 *value);
void pd_vbus_control_default_func(bool on)
{
	int vbus_mv = 0;
	u8 status, vconn_mask;
	u8 value1, value2;
#ifdef OHIO_DEBUG
	pr_info("vbus control %d\n", (int)on);
#endif
#ifdef SUP_VBUS_CTL
    ohio_vbus_control(on);
#else
	pr_info("vbus control %d\n", (int)on);
	/* turn on Vbus  */
	if (on) {
		/* WA for HDMI dongle Vconn connected to Vbus
		 * Before enabling OTG, we disable vconn first */
		ohio_hardware_disable_boost_5v();

		ohio_set_data_value(OHIO_PROLE, PR_SOURCE);
		ohio_set_data_value(OHIO_PMODE, MODE_DFP);
		enable_oc_work_func();
		dwc3_pd_vbus_ctrl(1);
		mdelay(5);
		vbus_mv = pmi8994_get_usbin_voltage_now()/1000;
		pr_debug("%s: vbus voltage (%d mv)\n", __func__, vbus_mv);
		if (vbus_mv < 3500) {
			pr_info("%s: Disable boost 5v due to abnormal vbus voltage (%d mv)\n", __func__, vbus_mv);
			dwc3_pd_vbus_ctrl(0);
		}
		else {
			/* the OTG is valid, disable the pulse skip mode. */
			usb_otg_pulse_skip_control(1);
		}

		/* Because we disable Vconn in the beginning,
 		 * re-enable Vconn if needed                 */
		status = OhioReadReg(NEW_CC_STATUS);
		if (((status & 0x0F) == CC1_STATUS_RA) || ((status & 0xF0) == CC2_STATUS_RA)) {
			if ((status & 0x0F) == CC1_STATUS_RA) {
				pr_info("%s: CC1 Ra\n", __func__);
				ohio_hardware_enable_vconn();
				vconn_mask = OhioReadReg(INTP_CTRL);
				vconn_mask &= 0x0F;
				OhioWriteReg(INTP_CTRL, (vconn_mask | 0x10));
			} else if ((status & 0xF0) == CC2_STATUS_RA) {
				pr_info("%s: CC2 Ra\n", __func__);
				ohio_hardware_enable_vconn();
				vconn_mask = OhioReadReg(INTP_CTRL);
				vconn_mask &= 0x0F;
				OhioWriteReg(INTP_CTRL, (vconn_mask | 0x20));
			}
			qpnp_boost_status(&value1);
			pr_debug("%s: qpnp_boost_status = 0x%02x\n", __func__, value1);
			qpnp_boost_int_status(&value2);
			pr_debug("%s: qpnp_boost_int_status = 0x%02x\n", __func__, value2);
			if (!value1 || !value2) {
				pr_info("%s: over current or no need Vconn, disabling Vconn\n", __func__);
				ohio_hardware_disable_vconn();
			}
		}

		if (ohio_get_data_value(OHIO_START_HOST_FLAG)) { // OCM FW v1.2+ only
			pr_info("for FW1.2 WA, start host\n");
			//pd_drole_change_default_func(1);
			enable_drole_work_func(1);
			ohio_set_data_value(OHIO_START_HOST_FLAG, 0);
		}
	}
	/* turn off Vbus  */
	else {
		ohio_set_data_value(OHIO_PROLE, PR_SINK);
		ohio_set_data_value(OHIO_PMODE, MODE_UFP);
		dwc3_pd_vbus_ctrl(0);
		dual_role_instance_changed(ohio_get_dual_role_instance()); //send uevent
	}
#endif
}

void pd_vconn_control_default_func(bool on)
{
	u8 status;
	/* to enable or disable VConn */
#ifdef OHIO_DEBUG
	pr_info("vconn control %d\n", (int)on);
#endif
	pr_info("vconn control %d\n", (int)on);
	status = OhioReadReg(NEW_CC_STATUS);
	pr_debug("%s: cc status 0x%x\n", __func__, status);

	if (on) { // enable Vconn
		/* Rely on 7418 for correct notification of Vconn enabling
		 * Here we only detect CC1 and CC2 for the existence of Ra
		 */
		// CC1 || CC2 connects to Ra
		if(((status & 0x0F) == CC1_STATUS_RA) || ((status & 0xF0) == CC2_STATUS_RA)) {
			ohio_hardware_enable_vconn();
		}
		else {
			pr_info("no Ra detected, do not enable Vconn\n");
			ohio_hardware_disable_vconn();
		}
	}
	else { // disable Vconn
		ohio_hardware_disable_vconn();
	}
}

void pd_cc_status_default_func(u8 cc_status)
{
	/* cc status */
#ifdef OHIO_DEBUG
	pr_info("cc status 0x%x\n", cc_status);
#endif
	pr_info("cc status 0x%x\n", cc_status);

	if ((cc_status == 0x20) || (cc_status == 0x02)) {
		pr_info("%s: open e-mark cable in\n", __func__);
		ohio_set_data_value(OHIO_EMARKER, 1);
	}
	else if (cc_status == 0x22) {
		is_RaRaCable = true;
		SendRaRaEvent(is_RaRaCable);
	}
	usb_downgrade_func();
}

/*void pd_drole_change_default_func(bool on)
{

}*/

/* Recieve comand response message's callback function.
 * it can be rewritten by customer just reimmplement this function,
 *  void *para :
 *   para_len :
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 */
u8 recv_pd_cmd_rsp_default_callback(void *para, u8 para_len)
{
	u8 pd_cmd, pd_response;

	pd_cmd =  *(u8 *)para;
	pd_response = *((u8 *)para + 1);

	switch (pd_cmd) {
	case TYPE_PWR_OBJ_REQ:
		if (pd_response == CMD_SUCCESS)
			pr_info("pd_cmd RDO request result is successful\n");
		else if (pd_response == CMD_REJECT)
			pr_info("pd_cmd RDO reques result is rejected\n");
		else if (pd_response == CMD_BUSY)
			pr_info("pd_cmd RDO reques result is busy\n");
		else if (pd_response == CMD_FAIL)
			pr_info("pd_cmd RDO reques result is fail\n");
		else
			pr_info("pd_cmd RDO reques result is unknown\n");
		break;
	case TYPE_VCONN_SWAP_REQ:
		if (pd_response == CMD_SUCCESS)
			pr_info("pd_cmd VCONN Swap result is successful\n");
		else if (pd_response == CMD_REJECT)
			pr_info("pd_cmd VCONN Swap result is rejected\n");
		else if (pd_response == CMD_BUSY)
			pr_info("pd_cmd VCONN Swap result is busy\n");
		else if (pd_response == CMD_FAIL)
			pr_info("pd_cmd VCONN Swap result is fail\n");
		else
			pr_info("pd_cmd VCONN Swap result is unknown\n");
		break;
	case TYPE_DSWAP_REQ:
		if (pd_response == CMD_SUCCESS)
			pr_info("pd_cmd DRSwap result is successful\n");
		else if (pd_response == CMD_REJECT)
			pr_info("pd_cmd DRSwap result is rejected\n");
		else if (pd_response == CMD_BUSY)
			pr_info("pd_cmd DRSwap result is busy\n");
		else if (pd_response == CMD_FAIL)
			pr_info("pd_cmd DRSwap result is fail\n");
		else
			pr_info("pd_cmd DRSwap result is unknown\n");
		break;
	case TYPE_PSWAP_REQ:
		if (pd_response == CMD_SUCCESS)
			pr_info("pd_cmd PRSwap result is successful\n");
		else if (pd_response == CMD_REJECT)
			pr_info("pd_cmd PRSwap result is rejected\n");
		else if (pd_response == CMD_BUSY)
			pr_info("pd_cmd PRSwap result is busy\n");
		else if (pd_response == CMD_FAIL)
			pr_info("pd_cmd PRSwap result is fail\n");
		else
			pr_info("pd_cmd PRSwap result is unknown\n");
		break;
	default:
		break;
	}

	return CMD_SUCCESS;
}

u8 recv_pd_hard_rst_default_callback(void *para, u8 para_len)
{
#ifdef OHIO_DEBUG
	pr_info("recv pd hard reset\n");
#endif
	return CMD_SUCCESS;
}

/* Recieve Data Role Swap message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through init_pd_msg_callback, it it pd_callback is not 0, using the default
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_dswap_default_callback(void *para, u8 para_len)
{
	/* dswap just notice AP, do nothing */
	return 1;
}

/* Recieve Power Role Swap message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through init_pd_msg_callback, it it pd_callback is not 0, using the default
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_pswap_default_callback(void *para, u8 para_len)
{
	/* pswap just notice AP, do nothing */
	return 1;
}
static pd_callback_t pd_callback_array[256] = { 0 };

pd_callback_t get_pd_callback_fnc(PD_MSG_TYPE type)
{
	pd_callback_t fnc = 0;
	if (type < 256)
		fnc = pd_callback_array[type];
	return fnc;
}

void set_pd_callback_fnc(PD_MSG_TYPE type, pd_callback_t fnc)
{
	pd_callback_array[type] = fnc;
}

void init_pd_msg_callback(void)
{
	u8 i = 0;
	for (i = 0; i < 256; i++)
		pd_callback_array[i] = 0x0;
}
