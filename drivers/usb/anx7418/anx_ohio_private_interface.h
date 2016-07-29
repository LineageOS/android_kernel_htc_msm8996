#ifndef ANX_OHIO_PRIVATE_INTERFACE_H
#define ANX_OHIO_PRIVATE_INTERFACE_H
#include <linux/types.h>
#include "anx_ohio_public_interface.h"

#ifdef pr_err
#undef pr_err
#endif
#define pr_err(fmt, args...) \
	printk(KERN_ERR "[OHIO] " pr_fmt(fmt), ## args)

#ifdef pr_info
#undef pr_info
#endif
#define pr_info(fmt, args...) \
	printk(KERN_INFO "[OHIO] " pr_fmt(fmt), ## args)


#ifdef OHIO_OCM_OTP_VERSION
#define OHIO_OCM_LOADING_TIME 20
#else
#define OHIO_OCM_LOADING_TIME 3200
#endif
#define CABLE_DET_PIN_HAS_GLITCH
#define SUP_OHIO_INT_VECTOR
#define SUP_TRY_SRC_SINK

#define YES     1
#define NO      0
#define ERR_CABLE_UNPLUG -1
#define PD_ONE_DATA_OBJECT_SIZE  4
#define PD_MAX_DATA_OBJECT_NUM  7
#define VDO_SIZE (PD_ONE_DATA_OBJECT_SIZE * PD_MAX_DATA_OBJECT_NUM)

#define PDO_FIXED_FLAGS (PDO_FIXED_DUAL_ROLE | PDO_FIXED_DATA_SWAP)

#define MV 1
#define MA 1
#define MW 1

#define PD_VOLTAGE_5V 5000

#define PD_MAX_VOLTAGE_20V 20000
#define PD_CURRENT_900MA   900
#define PD_CURRENT_1500MA 1500

#define PD_CURRENT_3A   3000

#define PD_POWER_15W  15000

#define PD_POWER_60W  60000

#define RDO_OBJ_POS(n)             (((u32)(n) & 0x7) << 28)
#define RDO_POS(rdo)               ((((32)rdo) >> 28) & 0x7)
#define RDO_GIVE_BACK              ((u32)1 << 27)
#define RDO_CAP_MISMATCH           ((u32)1 << 26)
#define RDO_COMM_CAP               ((u32)1 << 25)
#define RDO_NO_SUSPEND             ((u32)1 << 24)
#define RDO_FIXED_VAR_OP_CURR(ma)  (((((u32)ma) / 10) & 0x3FF) << 10)
#define RDO_FIXED_VAR_MAX_CURR(ma) (((((u32)ma) / 10) & 0x3FF) << 0)

#define RDO_BATT_OP_POWER(mw)      (((((u32)mw) / 250) & 0x3FF) << 10)
#define RDO_BATT_MAX_POWER(mw)     (((((u32)mw) / 250) & 0x3FF) << 10)

#define RDO_FIXED(n, op_ma, max_ma, flags)	\
	(RDO_OBJ_POS(n) | (flags) |		\
	RDO_FIXED_VAR_OP_CURR(op_ma) |		\
	RDO_FIXED_VAR_MAX_CURR(max_ma))

#define EXTERNALLY_POWERED  YES
#define SOURCE_PROFILE_NUMBER   1
#define SRC_PDO_SUPPLY_TYPE1    0
#define SRC_PDO_PEAK_CURRENT1   0
#define SRC_PDO_VOLTAGE1        5000
#define SRC_PDO_MAX_CURRENT1    500

#define IRQ_STATUS 0x53
#define IRQ_EXT_MASK_2 0x3d
#define IRQ_EXT_SOFT_RESET_BIT 0x04
#define IRQ_EXT_SOURCE_2 0x4F


#define INTERACE_TIMEOUT_MS 26

#define OHIO_SLAVE_I2C_ADDR 0x50
#define OHIO_INTERFACE_INTR_MASK 0x17
#define RECEIVED_MSG_MASK 1
#define RECEIVED_ACK_MASK 2
#define VCONN_CHANGE_MASK 4
#define VBUS_CHANGE_MASK 8
#define CC_STATUS_CHANGE_MASK 16
#define DATA_ROLE_CHANGE_MASK 32

#define OHIO_INTERFACE_CHANGE_INT 0x28
#define RECEIVED_MSG 0x01
#define RECEIVED_ACK 0x02
#define VCONN_CHANGE 0x04
#define VBUS_CHANGE 0x08
#define CC_STATUS_CHANGE 0x10
#define DATA_ROLE_CHANGE 0x20

#define OHIO_SYSTEM_STSTUS 0x29
#define VCONN_STATUS 0x04
#define VBUS_STATUS 0x08
#define DATA_ROLE 0x20

#define NEW_CC_STATUS 0x2A
#define INTP_CTRL 0x33

#define T_TIME_1 0x6C
#define T_HARDREST_VBUS_OFF_MASK 0x0F

#define VBUS_DELAY_TIME 0x69
#define TRY_UFP_TIMER 0x6A

#define AUTO_PD_MODE 0x6e
#define AUTO_PD_ENABLE 0x02

#define MAX_VOLTAGE_SETTING 0xd0
#define MAX_POWER_SETTING 0xd1
#define MIN_POWER_SETTING 0xd2

#define CC2_STATUS_RA (0x2 << 4)
#define CC1_STATUS_RA 0x2

int ohio_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf);
int ohio_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value);
unsigned char OhioReadReg(unsigned char RegAddr);
void OhioWriteReg(unsigned char RegAddr, unsigned char RegVal);

struct tagInterfaceHeader {
	unsigned char Indicator:1;	
	unsigned char Type:3;		
	unsigned char Length:4;		
};

struct tagInterfaceData {
	unsigned long SrcPDO[7];
	unsigned long SnkPDO[7];
	unsigned long RDO;
	unsigned long VDMHeader;
	unsigned long IDHeader;
	unsigned long CertStatVDO;
	unsigned long ProductVDO;
	unsigned long CableVDO;
	unsigned long AMM_VDO;
};

enum interface_status { CMD_SUCCESS, CMD_REJECT, CMD_FAIL, CMD_BUSY,
	CMD_STATUS
};

enum port_mode {
	MODE_UFP = 0,
	MODE_DFP,
	MODE_UNKNOWN,
};

enum power_role {
	PR_SOURCE = 0,
	PR_SINK,
	UNKNOWN_POWER_ROLE,
};

enum data_role {
	DR_HOST = 0,
	DR_DEVICE,
	UNKNOWN_DATA_ROLE,
};

enum vconn_supply {
	VCONN_SUPPLY_NO = 0,
	VCONN_SUPPLY_YES,
};

enum pr_change {
	PR_NOCHANGE = 0,
	SRC_TO_SNK,
	SNK_TO_SRC,
};

enum ohio_data_member {
	OHIO_DROLE = 0,
	OHIO_PROLE,
	OHIO_PMODE,
	OHIO_MODE_CHANGE,
	OHIO_VCONN,
	OHIO_START_HOST_FLAG,
	OHIO_EMARKER,
};

#define MAX_INTERFACE_COUNT 32
#define MAX_INTERFACE_MSG_LEN  32

#define INTERFACE_TIMEOUT 30
extern u8 pd_src_pdo_cnt;
extern u8 pd_src_pdo[];
extern u8 pd_snk_pdo_cnt;
extern u8 pd_snk_pdo[];
extern u8 pd_rdo[];
extern u8 DP_caps[];
extern u8 configure_DP_caps[];
extern u8 src_dp_caps[];
extern atomic_t ohio_power_status;
extern unsigned char downstream_pd_cap;

#define is_soft_reset_intr() \
	(OhioReadReg(IRQ_EXT_SOURCE_2) & IRQ_EXT_SOFT_RESET_BIT)

#define clear_soft_interrupt()	\
	OhioWriteReg(IRQ_EXT_SOURCE_2, IRQ_EXT_SOFT_RESET_BIT)

#define interface_pr_swap() \
	interface_send_msg_timeout(TYPE_PSWAP_REQ, 0, 0, INTERFACE_TIMEOUT)
#define interface_dr_swap() \
	interface_send_msg_timeout(TYPE_DSWAP_REQ, 0, 0, INTERFACE_TIMEOUT)
#define interface_vconn_swap() \
	interface_send_msg_timeout(TYPE_VCONN_SWAP_REQ, 0, 0, INTERFACE_TIMEOUT)
#define interface_get_dp_caps() \
	interface_send_msg_timeout(TYPE_GET_DP_SNK_CAP, 0, 0, INTERFACE_TIMEOUT)
#define interface_send_gotomin() \
	interface_send_msg_timeout(TYPE_GOTO_MIN_REQ, 0, 0, INTERFACE_TIMEOUT)
#define interface_send_soft_rst() \
	interface_send_msg_timeout(TYPE_SOFT_RST, 0, 0, INTERFACE_TIMEOUT)
#define interface_send_hard_rst() \
	interface_send_msg_timeout(TYPE_HARD_RST, 0, 0, INTERFACE_TIMEOUT)
#define interface_send_restart() \
	interface_send_msg_timeout(TYPE_RESTART, 0, 0, INTERFACE_TIMEOUT)
#define interface_send_accept() \
	interface_send_msg_timeout(TYPE_ACCEPT, 0, 0, INTERFACE_TIMEOUT)
#define interface_send_reject() \
	interface_send_msg_timeout(TYPE_REJECT, 0, 0, INTERFACE_TIMEOUT)
#define interface_send_dp_enter() \
	interface_send_msg_timeout(TYPE_DP_ALT_ENTER, 0, 0, INTERFACE_TIMEOUT)
#define interface_send_dp_exit() \
	interface_send_msg_timeout(TYPE_DP_ALT_EXIT, 0, 0, INTERFACE_TIMEOUT)
#define interface_send_src_cap() \
	interface_send_msg_timeout(TYPE_PWR_SRC_CAP, pd_src_pdo,\
	pd_src_pdo_cnt * 4, INTERFACE_TIMEOUT)
#define interface_send_snk_cap() \
	interface_send_msg_timeout(TYPE_PWR_SNK_CAP, pd_snk_pdo,\
	pd_snk_pdo_cnt * 4, INTERFACE_TIMEOUT)
#define interface_send_src_dp_cap() \
	interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY, src_dp_caps,\
	4, INTERFACE_TIMEOUT)
#define interface_config_dp_caps() \
	interface_send_msg_timeout(TYPE_DP_SNK_CFG, configure_DP_caps,\
	4, INTERFACE_TIMEOUT)
#define interface_send_request() \
	interface_send_msg_timeout(TYPE_PWR_OBJ_REQ, pd_rdo,\
	4, INTERFACE_TIMEOUT)
#define interface_send_vdm_data(buf, len)	\
	interface_send_msg_timeout(TYPE_VDM, buf, len, INTERFACE_TIMEOUT)

void send_initialized_setting(void);
void interface_init(void);
void chip_register_init(void);
u8 polling_interface_msg(int timeout_ms);
u8 try_source(void);
u8 try_sink(void);
u8 get_otp_indicator_byte(void);
u8 send_dp_snk_cfg(const u8 *dp_snk_caps, u8 dp_snk_caps_size);
u8 send_dp_snk_identity(const u8 *, u8);
u8 send_vdm(const u8 *vdm, u8 size);
u8 send_svid(const u8 *svid, u8 size);
u8 recv_pd_pwr_object_req_default_callback(void *para, u8 para_len);
u8 recv_pd_dswap_default_callback(void *para, u8 para_len);
u8 recv_pd_pswap_default_callback(void *para, u8 para_len);
u8 recv_pd_sink_caps_default_callback(void *para, u8 para_len);
u8 recv_pd_source_caps_default_callback(void *para, u8 para_len);
u8 recv_pd_cmd_rsp_default_callback(void *para, u8 para_len);
u8 recv_pd_goto_min_default_callback(void *para, u8 para_len);
u8 recv_pd_accept_default_callback(void *para, u8 para_len);
u8 recv_pd_reject_default_callback(void *para, u8 para_len);
u8 recv_pd_hard_rst_default_callback(void *para, u8 para_len);
char *interface_to_str(unsigned char header_type);

int OhioReadBlockReg(u8 RegAddr, u8 len, u8 *dat);
int OhioWriteBlockReg(u8 RegAddr, u8 len, const u8 *dat);
void ohio_vbus_control(bool value);

void pd_vbus_control_default_func(bool on);
void pd_vconn_control_default_func(bool on);
void pd_cc_status_default_func(u8 cc_status);
void handle_intr_vector(void);
struct dual_role_phy_instance *ohio_get_dual_role_instance(void);
int ohio_get_data_value(int data_member);
int ohio_set_data_value(int data_member, int val);
int ohio_hardware_enable_vconn(void);
int ohio_hardware_disable_vconn(void);
int ohio_hardware_disable_boost_5v(void);
void enable_drole_work_func(int on);
void enable_oc_work_func(void);
u8 interface_send_msg_timeout(u8 type, u8 *pbuf, u8 len, int timeout_ms);
pd_callback_t get_pd_callback_fnc(PD_MSG_TYPE type);
void set_pd_callback_fnc(PD_MSG_TYPE type, pd_callback_t fnc);


u8 send_src_cap(const u8 *src_caps, u8 src_caps_size);

u8 send_snk_cap(const u8 *snk_caps, u8 snk_caps_size);
u8 send_rdo(const u8 *rdo, u8 size);
u8 send_data_swap(void);
u8 send_power_swap(void);
u8 send_accept(void);



s8 get_data_role(void);

s8 get_power_role(void);


#endif
