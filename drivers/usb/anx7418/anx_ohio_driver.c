/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
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
#include <linux/firmware.h>
#include "anx_ohio_driver.h"
#include "anx_ohio_private_interface.h"
#include "anx_ohio_public_interface.h"
#include "usb_typec_fw_update.h"

#include <linux/usb/htc_info.h>
#include <linux/htc_flags.h> 
#include <linux/usb/gadget.h>
#include <linux/power/htc_battery.h>
#define CONFIG_ANX7418_EEPROM

#define DEBUG 0
#define VDEBUG 0

struct pinctrl *ohio_pinctrl;
struct pinctrl_state *ohio_default_state;
extern void dwc3_otg_set_id_state(int id);
extern int dwc3_pd_vbus_ctrl(int on);
extern int dwc3_pd_drswap(int new_role);
extern int usb_get_dwc_property(int prop_type);

extern int usb_lock_host_speed;
int cable_connected = 0;
int oc_enable = 0;
static int create_sysfs_interfaces(struct device *dev);
static int update_firmware_otp(struct usb_typec_fwu_notifier *notifier, struct firmware *fw);
static int update_firmware(struct usb_typec_fwu_notifier *notifier, struct firmware *fw);
static int check_usbc_fw_version(const struct firmware *fw, int fw_type);

static struct ohio_platform_data *g_pdata;

#define DONGLE_CABLE_DEBOUNCE_FAIL 2
#define DONGLE_CABLE_INSERT 1
#define DONGLE_CABLE_REMOVE 0

struct i2c_client *ohio_client;

struct ohio_platform_data {
	int gpio_p_on;
	int gpio_reset;
	int gpio_cbl_det;
	int gpio_intr_comm;
#ifdef SUP_VBUS_CTL
	int gpio_vbus_ctrl;
#endif
	int gpio_usb_ptn_c1;
	int gpio_usb_ptn_c2;
	int gpio_vconn_boost;
	int (*avdd_power)(unsigned int onoff);
	int (*dvdd_power)(unsigned int onoff);
	struct regulator *avdd_10;
	struct regulator *dvdd_10;
	struct regulator *boost_5v;
	spinlock_t lock;
};

struct ohio_data {
	struct ohio_platform_data *pdata;
	struct delayed_work work;
	struct delayed_work debounce_work;
	struct delayed_work comm_isr_work;
	struct delayed_work drole_work;
	struct delayed_work oc_enable_work;
	struct workqueue_struct *workqueue;
	struct workqueue_struct *comm_workqueue;
	struct mutex lock;
	struct mutex drole_lock;
	struct wake_lock ohio_lock;	
	int fake_irq_counter;
	int cbl_det_irq;
	int DFP_mode; 
	enum port_mode pmode;
	enum power_role prole;
	enum data_role drole;
	enum vconn_supply vconn;
	enum pr_change pr_change;
	struct usb_typec_fwu_notifier *usbc_fwu_notifier;
	struct dual_role_phy_instance *ohio_dual_role_instance;
	uint fw_version;
	int need_start_host;
	int emarker_flag;
	int non_standard_flag;
	int drole_on;
};

int ohio_dual_role_get_property(struct dual_role_phy_instance *dual_role,
			     enum dual_role_property prop,
			     unsigned int *val)
{
	struct ohio_data *ohio = NULL;

	if(ohio_client)
		ohio = i2c_get_clientdata(ohio_client);
	else
		return -ENODEV;

	switch (prop) {
		case DUAL_ROLE_PROP_SUPPORTED_MODES:
			*val = 0; 
			break;
		case DUAL_ROLE_PROP_MODE:
			*val = (unsigned int)ohio->pmode;
			break;
		case DUAL_ROLE_PROP_PR:
			*val = (unsigned int)ohio->prole;
			break;
		case DUAL_ROLE_PROP_DR:
			*val = (unsigned int)ohio->drole;
			break;
		case DUAL_ROLE_PROP_VCONN_SUPPLY:
			*val = (unsigned int)ohio->vconn;
			break;
		default:
			break;
	}
	return 0;
}

int ohio_dual_role_set_property(struct dual_role_phy_instance *dual_role,
			     enum dual_role_property prop,
			     const unsigned int *val)
{
	struct ohio_data *ohio = NULL;

	if(ohio_client)
		ohio = i2c_get_clientdata(ohio_client);
	else
		return -ENODEV;

	switch (prop) {
		case DUAL_ROLE_PROP_SUPPORTED_MODES:
			break;
		case DUAL_ROLE_PROP_MODE:
			pr_info("%s: set dual role mode %d -> %u\n", __func__, ohio->pmode, *val);
			switch (*val) {
				case MODE_UFP:
					if (ohio->pmode == MODE_DFP)
						try_sink();
					else
						pr_err("%s: change mode fail, %d -> %u\n", __func__, ohio->pmode, *val);
					break;
				case MODE_DFP:
					if (ohio->pmode == MODE_UFP)
						try_source();
					else
						pr_err("%s: change mode fail, %d -> %u\n", __func__, ohio->pmode, *val);
					break;
				case MODE_UNKNOWN:
				default:
					break;
			}
			break;
		case DUAL_ROLE_PROP_PR:
			pr_info("%s: set power role %d -> %u\n", __func__, ohio->pmode, *val);
			switch (*val) {
				case PR_SOURCE:
					if (ohio->prole == PR_SINK)
						send_pd_msg(TYPE_PSWAP_REQ, NULL, 0);
					break;
				case PR_SINK:
					if (ohio->prole == PR_SOURCE)
						send_pd_msg(TYPE_PSWAP_REQ, NULL, 0);
					break;
				case UNKNOWN_POWER_ROLE:
				default:
					break;
			}
			break;
		case DUAL_ROLE_PROP_DR:
			pr_info("%s: set data role %d -> %u\n", __func__, ohio->pmode, *val);
			switch (*val) {
				case DR_HOST:
					if (ohio->drole == DR_DEVICE)
						send_pd_msg(TYPE_DSWAP_REQ, NULL, 0);
					break;
				case DR_DEVICE:
					if (ohio->drole == DR_HOST)
						send_pd_msg(TYPE_DSWAP_REQ, NULL, 0);
					break;
				case UNKNOWN_DATA_ROLE:
				default:
					break;
			}
			break;
		case DUAL_ROLE_PROP_VCONN_SUPPLY:
			break;
		default:
			break;
	}
	return 0;
}

int ohio_dual_role_property_is_writeable(struct dual_role_phy_instance *dual_role,
			     enum dual_role_property prop)
{
	int val = 0;
	switch (prop) {
		case DUAL_ROLE_PROP_SUPPORTED_MODES:
			val = 0;
			break;
		case DUAL_ROLE_PROP_MODE:
			val = 1;
			break;
		case DUAL_ROLE_PROP_PR:
		case DUAL_ROLE_PROP_DR:
			val = 0;
			break;
		case DUAL_ROLE_PROP_VCONN_SUPPLY:
			val = 1;
			break;
		default:
			break;
	}
	return val;
}

struct dual_role_phy_instance *ohio_get_dual_role_instance(void)
{
	struct ohio_data *ohio = NULL;

	if(ohio_client)
		ohio = i2c_get_clientdata(ohio_client);
	else
		return NULL;

	return ohio->ohio_dual_role_instance;
}
EXPORT_SYMBOL(ohio_get_dual_role_instance);

int ohio_get_data_value(int data_member)
{
	struct ohio_data *ohio = NULL;

	if(ohio_client)
		ohio = i2c_get_clientdata(ohio_client);
	else
		return -ENODEV;

	switch (data_member) {
		case 0: 
			return ohio->drole;
		case 1: 
			return ohio->prole;
		case 2: 
			return ohio->pmode;
		case 3: 
			return ohio->pr_change;
		case 4: 
			return ohio->vconn;
		case 5: 
			if ((ohio->fw_version < 0x12) || (ohio->fw_version == 0x16))
				return 0;
			else
				return ohio->need_start_host;
		case 6: 
			return ohio->emarker_flag;
		case 7: 
			return ohio->non_standard_flag;
		case 8: 
			return downstream_pd_cap;
		default:
			return -1;
	}
}
EXPORT_SYMBOL(ohio_get_data_value);

int ohio_set_data_value(int data_member, int val)
{
	struct ohio_data *ohio = NULL;

	if(ohio_client)
		ohio = i2c_get_clientdata(ohio_client);
	else
		return -ENODEV;

	switch (data_member) {
		case 0: 
			ohio->drole = val;
			break;
		case 1: 
			ohio->prole = val;
			break;
		case 2: 
			ohio->pmode = val;
			break;
		case 3: 
			ohio->pr_change = val;
			break;
		case 4: 
			ohio->vconn = val;
			break;
		case 5: 
			ohio->need_start_host = val;
			break;
		case 6: 
			ohio->emarker_flag = val;
			break;
		default:
			break;
	}
	return 0;
}
EXPORT_SYMBOL(ohio_set_data_value);

bool ohio_is_connected(void)
{
	struct ohio_platform_data *pdata = NULL;
	bool result = false;

	if (!ohio_client)
		return false;

#ifdef CONFIG_OF
	pdata = g_pdata;
#else
	pdata = ohio_client->dev.platform_data;
#endif

	if (!pdata)
		return false;

	if (gpio_get_value_cansleep(pdata->gpio_cbl_det)) {
		mdelay(10);
		if (gpio_get_value_cansleep(pdata->gpio_cbl_det)) {
			pr_info("%s : Slimport Dongle is detected\n", __func__);
			result = true;
		}
	}

	return result;
}
EXPORT_SYMBOL(ohio_is_connected);

inline unsigned char OhioReadReg(unsigned char RegAddr)
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(ohio_client, RegAddr);
	if (ret < 0) {
		pr_err("%s: failed to read i2c addr=%x\n", __func__, OHIO_SLAVE_I2C_ADDR);
	}
	return (uint8_t) ret;
}

inline int OhioReadBlockReg(u8 RegAddr, u8 len, u8 *dat)
{
	int ret = 0;

	ret = i2c_smbus_read_i2c_block_data(ohio_client, RegAddr, len, dat);
	if (ret < 0) {
		pr_err("%s: failed to read i2c block addr=%x\n",
		       __func__, OHIO_SLAVE_I2C_ADDR);
		return -EPERM;
	}
	return (int)ret;
}

inline int OhioWriteBlockReg(u8 RegAddr, u8 len, const u8 *dat)
{
	int ret = 0;

	ret = i2c_smbus_write_i2c_block_data(ohio_client, RegAddr, len, dat);
	if (ret < 0) {
		pr_err("%s: failed to read i2c block addr=%x\n",
		       __func__, OHIO_SLAVE_I2C_ADDR);
		return -EPERM;
	}
	return (int)ret;
}

inline void OhioWriteReg(unsigned char RegAddr, unsigned char RegVal)
{
	int ret = 0;

	ret = i2c_smbus_write_byte_data(ohio_client, RegAddr, RegVal);
	if (ret < 0) {
		pr_err("%s: failed to write i2c addr=%x\n",
		       __func__, OHIO_SLAVE_I2C_ADDR);
	}
}

int ohio_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf)
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(ohio_client, offset);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c addr=%x\n", LOG_TAG,
		       __func__, slave_addr);
		return ret;
	}
	*buf = (uint8_t) ret;

	return 0;
}

int ohio_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value)
{
	int ret = 0;

	ret = i2c_smbus_write_byte_data(ohio_client, offset, value);
	if (ret < 0) {
		pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,
		       __func__, slave_addr);
	}
	return ret;
}

int check_chip_type(void)
{
	uint8_t temp = 0;
	int ret = 0;
	static int chip_type = UNKNOWN_CHIP_TYPE;

	if (atomic_read(&ohio_power_status) != 1) {
		pr_err("%s : need to power on ANX7418 before read reg\n", __func__);
		return chip_type;
	}

	if (chip_type == UNKNOWN_CHIP_TYPE) {
		ret = ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, DEBUG_EE_0, (unchar *)(&temp));
		if (ret < 0) {
			pr_err("%s : cannot read OTP or EEPROM mode\n", __func__);
			return chip_type;
		}

		if ((temp & R_EE_DEBUG_STATE) == OTP_MODE)
			chip_type = OTP;
		else
			chip_type = EEPROM;
	}

	pr_debug("%s : ANX7418 chip type = %s\n", __func__, chip_type_to_string(chip_type));
	return chip_type;
}
EXPORT_SYMBOL(check_chip_type);

enum ANX7418_fw_type check_usbc_fw_type(const struct firmware *fw)
{
	enum ANX7418_fw_type fw_type = UNKNOWN_FW_TYPE;

	if ((fw->data[EEPROM_FW_CHIPID_HIGH] == 0x74) && (fw->data[EEPROM_FW_CHIPID_LOW] == 0x18)) {
		fw_type = EEPROM_FW;
	} else if ((fw->data[OTP_FW_CHIPID_HIGH] == 0x74) && (fw->data[OTP_FW_CHIPID_LOW] == 0x18)) {
		fw_type = OTP_FW;
	}

	pr_debug("%s : ANX7418 fw type = %s\n", __func__, fw_type_to_string(fw_type));
	return fw_type;
}

void ohio_hardware_poweron(void)
{
	int i = 0;
	int retry_count = 0;
	enum ANX7418_chip_type chip_type;
#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif
	retry_count = 3;
	while(retry_count--){
		ohio_hardware_powerdown();
		
		gpio_set_value(pdata->gpio_usb_ptn_c1, 1);
		gpio_set_value(pdata->gpio_usb_ptn_c2, 1);
		pr_debug("%s USB3.0 re-drive status (c1, c2)= (%d, %d) retry_count = %d\n",
				__func__, gpio_get_value(pdata->gpio_usb_ptn_c1),
				gpio_get_value(pdata->gpio_usb_ptn_c2), retry_count);
		

		
		gpio_set_value(pdata->gpio_p_on, 1);
		mdelay(10);
		
		gpio_set_value(pdata->gpio_reset, 1);
		mdelay(1);

		
		for(i=0; i< 3200; i++) {
			
			if(OhioReadReg(0x16) == 0x80) {
				atomic_set(&ohio_power_status, 1);
				chip_register_init();
				interface_init();
				send_initialized_setting();
				pr_debug("init interface setting end at i = %d\n", i);
				break;
			}
			mdelay(1);
		}
		if (atomic_read(&ohio_power_status) != 1){
			pr_err("%s : system is not ready.\n", __func__);
			continue;
		}

		chip_type = check_chip_type();

		if (chip_type == EEPROM) {
			if(OhioReadReg(0x66) == 0x1b){
				pr_debug("EEPROM load sucess, and CRC is correct!\n");
				atomic_set(&ohio_power_status, 1);
				break;
			}
			else if(OhioReadReg(0x66) == 0x4b){
				pr_err("EEPROM load sucess, but  CRC is incorrect!\n");
				break;
			}
			else if(retry_count >0)
			{
				pr_debug("%s : EEPROM load retry.\n", __func__);
				continue;
			}
		} else if (chip_type == OTP) {
			break;
		} else {
			pr_err("%s : ANX7418 cannot read chip type\n", __func__);
			continue;
		}
	}
	pr_info("%s: ohio power on (power_en=%d, reset=%d)\n",
		__func__, gpio_get_value(pdata->gpio_p_on), gpio_get_value(pdata->gpio_reset));
}

void ohio_hardware_powerdown(void)
{
#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif
	atomic_set(&ohio_power_status, 0);
	
	gpio_set_value(pdata->gpio_reset, 0);
	mdelay(1);

	
	gpio_set_value(pdata->gpio_p_on, 0);

	
	gpio_set_value(pdata->gpio_usb_ptn_c1, 0);
	gpio_set_value(pdata->gpio_usb_ptn_c2, 0);

	pr_debug("%s USB3.0 re-drive status (c1, c2)= (%d, %d)\n",
		__func__, gpio_get_value(pdata->gpio_usb_ptn_c1),
		gpio_get_value(pdata->gpio_usb_ptn_c2));

	pr_info("%s: ohio power down (power_en=%d, reset=%d)\n",
		__func__, gpio_get_value(pdata->gpio_p_on), gpio_get_value(pdata->gpio_reset));
}

int ohio_hardware_enable_vconn(void)
{
	int rc;
	struct ohio_data *ohio = NULL;

	if(ohio_client)
		ohio = i2c_get_clientdata(ohio_client);
	else
		return -ENODEV;

	if (atomic_read(&ohio_power_status) == 1) {
		if (!ohio->pdata->boost_5v) {
			pr_info("%s: no boost 5v regulator, try to get it again...", __func__);
			ohio->pdata->boost_5v = devm_regulator_get(&ohio_client->dev, "V_USB_boost");
			if (IS_ERR(ohio->pdata->boost_5v)) {
				rc = PTR_ERR(ohio->pdata->boost_5v);
				pr_err("%s: still unable to get boost_5v regulator, %d\n", __func__, rc);
				ohio->pdata->boost_5v = NULL;
				return -1;
			}
		}

		if (!regulator_is_enabled(ohio->pdata->boost_5v)) {
			rc = regulator_enable(ohio->pdata->boost_5v);
			if (rc) {
				pr_err("%s: Unable to enable boost_5v regulator\n", __func__);
				return -1;
			}
			gpio_direction_output(ohio->pdata->gpio_vconn_boost, 1);
			ohio->vconn = VCONN_SUPPLY_YES;
			pr_info("Vconn enabled\n");
		}
		else
			pr_err("%s: ERR: boost_5v regulator is already enabled\n", __func__);
	}
	else {
		pr_err("%s: system not ready, abort enabling Vconn\n", __func__);
		return -1;
	}
	return 0;
}

int ohio_hardware_disable_vconn(void)
{
	u32 rc;
	uint8_t reg;
	struct ohio_data *ohio = NULL;

	if(ohio_client)
		ohio = i2c_get_clientdata(ohio_client);
	else
		return -ENODEV;

	ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, INTP_CTRL, (unchar *)(&reg));
	reg &= 0x0F;
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, INTP_CTRL, reg);
	pr_debug("%s: INTP_CTRL = 0x%02X\n", __func__, reg);

	gpio_direction_output(ohio->pdata->gpio_vconn_boost, 0);
	if(ohio->pdata->boost_5v) {
		if (regulator_is_enabled(ohio->pdata->boost_5v)) {
			rc = regulator_disable(ohio->pdata->boost_5v);
			if (rc) {
				pr_err("%s: Unable to disable boost_5v regulator\n", __func__);
				return -1;
			}
			ohio->vconn = VCONN_SUPPLY_NO;
			pr_info("Vconn disabled\n");
		}
	}
	return 0;
}

int ohio_hardware_disable_boost_5v(void)
{
	u32 rc;
	struct ohio_data *ohio = NULL;

	if(ohio_client)
		ohio = i2c_get_clientdata(ohio_client);
	else
		return -ENODEV;

	gpio_direction_output(ohio->pdata->gpio_vconn_boost, 0);
	if(ohio->pdata->boost_5v) {
		if (regulator_is_enabled(ohio->pdata->boost_5v)) {
			rc = regulator_disable(ohio->pdata->boost_5v);
			if (rc) {
				pr_err("%s: Unable to disable boost_5v regulator\n", __func__);
				return -1;
			}
			pr_info("boost_5v disabled\n");
		}
	}
	return 0;
}

int ohio_release_wakelock(void)
{
	struct ohio_data *ohio = NULL;

	if(ohio_client)
		ohio = i2c_get_clientdata(ohio_client);
	else
		return -ENODEV;

	wake_unlock(&ohio->ohio_lock);
	return 0;
}
EXPORT_SYMBOL(ohio_release_wakelock);

void ohio_vbus_control(bool value)
{
#ifdef SUP_VBUS_CTL

#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif

	if(value)
		gpio_set_value(pdata->gpio_vbus_ctrl, 1);
	else
		gpio_set_value(pdata->gpio_vbus_ctrl, 0);
#endif
}

static void ohio_free_gpio(struct ohio_data *ohio)
{
	gpio_free(ohio->pdata->gpio_cbl_det);
	gpio_free(ohio->pdata->gpio_reset);
	gpio_free(ohio->pdata->gpio_p_on);
	gpio_free(ohio->pdata->gpio_intr_comm);
#ifdef SUP_VBUS_CTL
	gpio_free(ohio->pdata->gpio_vbus_ctrl);
#endif
	gpio_free(ohio->pdata->gpio_usb_ptn_c1);
	gpio_free(ohio->pdata->gpio_usb_ptn_c2);
	gpio_free(ohio->pdata->gpio_vconn_boost);
}

static int ohio_init_gpio(struct ohio_data *ohio)
{
	int ret = 0;

	pr_debug("%s: ohio init gpio\n", __func__);

	
	ret = gpio_request(ohio->pdata->gpio_vconn_boost, "ohio_vconn");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_vconn_boost);
		goto err0;
	}
	gpio_direction_output(ohio->pdata->gpio_vconn_boost, 0);

	
	ret = gpio_request(ohio->pdata->gpio_p_on, "ohio_p_on_ctl");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_p_on);
		goto err0;
	}
	gpio_direction_output(ohio->pdata->gpio_p_on, 0);
	
	ret = gpio_request(ohio->pdata->gpio_reset, "ohio_reset_n");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_reset);
		goto err1;
	}
	gpio_direction_output(ohio->pdata->gpio_reset, 0);
	
	ret = gpio_request(ohio->pdata->gpio_cbl_det, "ohio_cbl_det");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_cbl_det);
		goto err2;
	}
	gpio_direction_input(ohio->pdata->gpio_cbl_det);
	pr_debug("cabled detect successfully set up\n");

	
	ret = gpio_request(ohio->pdata->gpio_intr_comm, "ohio_intr_comm");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_intr_comm);
		goto err3;
	}
	gpio_direction_input(ohio->pdata->gpio_intr_comm);

#ifdef SUP_VBUS_CTL
	
	ret = gpio_request(ohio->pdata->gpio_vbus_ctrl, "ohio_vbus_ctrl");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_vbus_ctrl);
		goto err4;
	}
	gpio_direction_output(ohio->pdata->gpio_vbus_ctrl, 0);
#endif

	
	ret = gpio_request(ohio->pdata->gpio_usb_ptn_c1, "ohio_usb_ptn_c1");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
			ohio->pdata->gpio_usb_ptn_c1);
		goto err5;
	}
	gpio_direction_input(ohio->pdata->gpio_usb_ptn_c1);

	
	ret = gpio_request(ohio->pdata->gpio_usb_ptn_c2, "ohio_usb_ptn_c2");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
			ohio->pdata->gpio_usb_ptn_c2);
		goto err6;
	}
	gpio_direction_output(ohio->pdata->gpio_usb_ptn_c2, 1);

	pr_debug("ohio init gpio succesfully\n");

	goto out;

err6:
	gpio_free(ohio->pdata->gpio_usb_ptn_c2);
err5:
	gpio_free(ohio->pdata->gpio_usb_ptn_c1);
#ifdef SUP_VBUS_CTL
err4:
	gpio_free(ohio->pdata->gpio_vbus_ctrl);
#endif
err3:
	gpio_free(ohio->pdata->gpio_intr_comm);
err2:
	gpio_free(ohio->pdata->gpio_cbl_det);
err1:
	gpio_free(ohio->pdata->gpio_reset);
err0:
	gpio_free(ohio->pdata->gpio_p_on);
	gpio_free(ohio->pdata->gpio_vconn_boost);
	return 1;
out:
	return 0;
}

static int __maybe_unused ohio_system_init(void)
{
	
	
		return 0;
}

void cable_disconnect(void *data)
{
	struct ohio_data *ohio = data;
	cancel_delayed_work(&ohio->drole_work);
	oc_enable = 0;
	ohio->DFP_mode = 0;
	ohio->pmode = MODE_UNKNOWN;
	ohio->need_start_host = 0;
	ohio->emarker_flag = 0;
	ohio->non_standard_flag = 0;
	ohio->pr_change = PR_NOCHANGE;
	dwc3_pd_vbus_ctrl(-1);
	dwc3_otg_set_id_state(1);
	ohio->prole = UNKNOWN_POWER_ROLE;
	ohio->drole = UNKNOWN_DATA_ROLE;

#ifdef SUP_VBUS_CTL
	gpio_set_value(ohio->pdata->gpio_vbus_ctrl, 0);
#endif
	ohio_hardware_powerdown();
	ohio_hardware_disable_vconn();
	
	wake_unlock(&ohio->ohio_lock); 
	
}

void update_pwr_sink_caps(void){
	u32 sink_caps[] = {PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_900MA, PDO_FIXED_FLAGS)};
	#if 0
		
		PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_900MA, PDO_FIXED_FLAGS)
		#if 0
		
		PDO_BATT(PD_VOLTAGE_5V, PD_MAX_VOLTAGE_20V, PD_POWER_15W),
		
		PDO_VAR(PD_VOLTAGE_5V, PD_MAX_VOLTAGE_20V, PD_CURRENT_3A)
		#endif
	};
	#endif

	pr_debug("update_pwr_sink_caps\n");
	
	send_pd_msg(TYPE_PWR_SNK_CAP, (const char *)sink_caps, 4  );
}


void update_VDM(void)
{

	u8 vdm[] = {
		0x00,0x00,0x01,0x00, 0x43, 0x45,0x54,0x056
	};

	pr_debug("update_vdm\n");
	
	send_pd_msg(TYPE_VDM, (const char *)vdm, sizeof(vdm));
}

static unsigned char confirmed_cable_det(void *data)
{
	struct ohio_data *anxohio = data;

#ifdef CABLE_DET_PIN_HAS_GLITCH
	unsigned char count = 5;
	unsigned char cable_det_count = 0;
	u8 val = 0;

	if ((val = gpio_get_value(anxohio->pdata->gpio_cbl_det))
			== DONGLE_CABLE_REMOVE) { 
		do {
			if ((val = gpio_get_value(anxohio->pdata->gpio_cbl_det))
					== DONGLE_CABLE_REMOVE) {
				cable_det_count++;
			} else {
				return DONGLE_CABLE_DEBOUNCE_FAIL;
			}
			if (cable_det_count >= 2)
				return DONGLE_CABLE_REMOVE;
			mdelay(3);
		} while (count--);
	} else { 
		do {
			if ((val = gpio_get_value(anxohio->pdata->gpio_cbl_det))
					== DONGLE_CABLE_INSERT) {
				cable_det_count++;
			} else {
				return DONGLE_CABLE_DEBOUNCE_FAIL;
			}
			if (cable_det_count >= 5)
				return DONGLE_CABLE_INSERT;
			mdelay(3);
		} while (count--);
	}
	return DONGLE_CABLE_DEBOUNCE_FAIL;

#else
	return gpio_get_value(anxohio->pdata->gpio_cbl_det);
#endif
}

static irqreturn_t ohio_cbl_det_isr(int irq, void *data)
{
	struct ohio_data *ohio = data;
	bool ret = 0;
	if (!atomic_read(&cbl_det_irq_status)) {
		disable_irq_nosync(ohio->cbl_det_irq);
		atomic_set(&cbl_det_irq_status, 1);
		ret = queue_delayed_work(ohio->workqueue, &ohio->work, 0);
		
		if (!ret) {
			flush_workqueue(ohio->workqueue);
			queue_delayed_work(ohio->workqueue, &ohio->work, 0);
		}
	} else
		pr_err("%s: skip unbalance cbl_det irq\n", __func__);
#if 0
	u8 val;
	pr_info("cbl_det_isr\n");
	val=gpio_get_value(ohio->pdata->gpio_cbl_det);
	if (val == DONGLE_CABLE_INSERT)
		queue_delayed_work(ohio->workqueue, &ohio->work, 0);
	else {
		cable_disconnect(ohio);
	}
#endif
	return IRQ_HANDLED;
}

void switch_i2c_slave_speed(void)
{
	uint8_t reg = 0;
	
	ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_7 , (unchar *)(&reg));
	reg &= ~(0xf << 4);
	reg |= (1 << 4);
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_7 , (reg));
	msleep(1);
	ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_7 , (unchar *)(&reg));
	pr_debug("%s : switch_i2c_slave_speed: %s(%x)\n", __func__, (reg >> 5) ?"400K":"1M", reg);

}

void ufp_switch_usb_speed(int on)
{
	uint8_t pre = 0, reg = 0;
	uint temp;

	if (atomic_read(&ohio_power_status) != 1){
		pr_err("%s : system is not ready.\n", __func__);
		return;
	}

	if (on) {
		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, POWER_DOWN_CTRL , (unchar *)(&temp));
		if (temp & CC1_ATTACH) {
			pr_debug("%s : CC pin connect to CC1\n", __func__);
			ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1 , (unchar *)(&reg));
			pre = reg;
			reg |= SSRX_SWITCH_CC1;
			ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1, reg);
			ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1 , (unchar *)(&reg));
			pr_debug("%s : ANALOG_CTRL_1 status: %x->%x\n", __func__, pre, reg);
			ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5 , (unchar *)(&reg));
			pre = reg;
			reg |= SSTX_SWITCH_CC1;
			ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5, reg);
			ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5 , (unchar *)(&reg));
			pr_debug("%s : ANALOG_CTRL_5 status: %x->%x\n", __func__, pre, reg);
		}
		else if (temp & CC2_ATTACH) {
			pr_debug("%s : CC pin connect to CC2\n", __func__);
			ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1 , (unchar *)(&reg));
			pre = reg;
			reg |= SSRX_SWITCH_CC2;
			ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1, reg);
			ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1 , (unchar *)(&reg));
			pr_debug("%s : ANALOG_CTRL_1 status: %x->%x\n", __func__, pre, reg);
			ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5 , (unchar *)(&reg));
			pre = reg;
			reg |= SSTX_SWITCH_CC2;
			ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5, reg);
			ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5 , (unchar *)(&reg));
			pr_debug("%s : ANALOG_CTRL_5 status: %x->%x\n", __func__, pre, reg);
		}
		else
			pr_err("%s : Unknown CC pin status\n", __func__);
	} else {
		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1 , (unchar *)(&reg));
		pre = reg;
		reg &= ~0x30;
		ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1, reg);
		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1 , (unchar *)(&reg));
		pr_debug("%s : ANALOG_CTRL_1 status: %x->%x\n", __func__, pre, reg);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5 , (unchar *)(&reg));
		pre = reg;
		reg &= ~0xc0;
		ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5, reg);
		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5 , (unchar *)(&reg));
		pr_debug("%s : ANALOG_CTRL_5 status: %x->%x\n", __func__, pre, reg);
	}
}

void dfp_downgrade_usb20(void)
{
	uint8_t pre = 0, reg = 0;

	if (atomic_read(&ohio_power_status) != 1){
		pr_err("%s : system is not ready.\n", __func__);
		return;
	}

	ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1 , (unchar *)(&reg));
	pre = reg;
	reg &= ~0x30;
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1, reg);
	ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1 , (unchar *)(&reg));
	pr_debug("%s : ANALOG_CTRL_1 status: %x->%x\n", __func__, pre, reg);

	ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5 , (unchar *)(&reg));
	pre = reg;
	reg &= ~0xc0;
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5, reg);
	ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5 , (unchar *)(&reg));
	pr_debug("%s : ANALOG_CTRL_5 status: %x->%x\n", __func__, pre, reg);
}

extern int usb_lock_speed;
void usb_downgrade_func(void)
{
	ohio_debug_dump();
	
	if (ohio_get_data_value(OHIO_PMODE) == MODE_DFP && usb_lock_host_speed)
		dfp_downgrade_usb20();
	if (ohio_get_data_value(OHIO_PMODE) == MODE_UFP && usb_lock_speed)
		ufp_switch_usb_speed(0);
}

void ohio_main_process(void)
{
	
	uint8_t val = 0;
	int ret = 0;
	int rc = 0;

	struct ohio_data *ohio = i2c_get_clientdata(ohio_client);

	if (atomic_read(&ohio_power_status) == 0){
		ohio_hardware_poweron();
		switch_i2c_slave_speed();

		if (ohio->fw_version == 0xFF) {
			val = OhioReadReg(ANALOG_CTRL_3);
			ohio->fw_version = val;
		}

		
		
		
		if (!ohio->pdata->boost_5v) {
			pr_info("%s: no boost 5v regulator, try to get it again...", __func__);
			ohio->pdata->boost_5v = devm_regulator_get(&ohio_client->dev, "V_USB_boost");
			if (IS_ERR(ohio->pdata->boost_5v)) {
				rc = PTR_ERR(ohio->pdata->boost_5v);
				pr_err("%s: still unable to get boost_5v regulator, %d\n", __func__, rc);
				ohio->pdata->boost_5v = NULL;
			}
		}

		if (!regulator_is_enabled(ohio->pdata->boost_5v)) {
			rc = regulator_enable(ohio->pdata->boost_5v);
			if (rc) {
				pr_err("%s: Unable to enable boost_5v regulator\n", __func__);
			}
			gpio_direction_output(ohio->pdata->gpio_vconn_boost, 1);
			pr_info("%s: boost_5v enabled\n", __func__);
		}
		else {
			pr_err("%s: ERR: boost_5v regulator is already enabled\n", __func__);
			gpio_direction_output(ohio->pdata->gpio_vconn_boost, 1);
		}

		
		ret = ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_STATUS, &val);
		if (ret < 0) {
			pr_err("%s : cannot read DFP or UFP status\n", __func__);
			goto exit;
		}
		if (val & DFP_OR_UFP) {
			
			pr_info("%s: UFP(%d) FW version 0x%02x\n",
					__func__, val & DFP_OR_UFP, ohio->fw_version);

			ohio->prole = PR_SINK;
			ohio->drole = DR_DEVICE;
			ohio->pmode = MODE_UFP;
			dual_role_instance_changed(ohio->ohio_dual_role_instance);
		} else {
			wake_lock(&ohio->ohio_lock); 
			
			ohio->prole = UNKNOWN_POWER_ROLE;
			ohio->pmode = MODE_DFP;
			pr_info("%s: DFP(%d) FW version 0x%02x\n",
					__func__, val & DFP_OR_UFP, ohio->fw_version);
			
			if (ohio->DFP_mode) {
				pr_err("%s: ERROR: hit double DFP issue, cancel starting host\n", __func__);
				goto exit;
			}

			ohio->DFP_mode = 1;
			ohio->need_start_host = 1;
#if 0
			val = try_sink();
			if (val) { 
				msleep(150);
				pr_info("try_sink failed at cbl_det\n");

				val = OhioReadReg(NEW_CC_STATUS);
				pr_info("%s: cc status 0x%x\n", __func__, val);
				if ((val == 0x20) || (val == 0x02)) {
					pr_info("%s: e-mark cable in\n", __func__);
					goto downgrade;
				}
				pr_info("enable Vbus and start Host\n");
				ret = dwc3_pd_vbus_ctrl(1);
				if (ret) {
					pr_err("%s: ERR: enable Vbus fail\n", __func__);
					goto exit;
				}
				ohio->prole = PR_SOURCE;
				dwc3_pd_drswap(DR_HOST);
				dwc3_otg_set_id_state(0);
				ohio->drole = DR_HOST;
			}
			else { 
				ohio->prole = PR_SINK;
				ohio->drole = DR_DEVICE;
				ohio->pmode = MODE_UFP;
				ohio->DFP_mode = 0;
			}
#endif
		}
	}
	else {
		pr_err("%s: power not ready\n", __func__);
		goto exit;
	}

exit:
	pr_debug("%s: exit\n", __func__);
}

extern int qpnp_boost_status(u8 *value);
extern int qpnp_boost_int_status(u8 *value);
static irqreturn_t ohio_intr_comm_isr(int irq, void *data)
{
	struct ohio_data *ohio = data;
	u8 value1, value2;

	if (atomic_read(&ohio_power_status) != 1)
	{
		pr_info("%s: ERROR: power_status != 1\n", __func__);
		return IRQ_NONE;
	}
	if (is_soft_reset_intr()) {
		queue_delayed_work(ohio->comm_workqueue, &ohio->comm_isr_work, 0);
	}

	qpnp_boost_status(&value1);
	pr_debug("%s: qpnp_boost_status = 0x%02x\n", __func__, value1);
	qpnp_boost_int_status(&value2);
	pr_debug("%s: qpnp_boost_int_status = 0x%02x\n", __func__, value2);
	if (!value1 || !value2) {
		pr_info("%s: over current or no need Vconn, disabling Vconn\n", __func__);
		ohio_hardware_disable_vconn();
	}

	return IRQ_HANDLED;
}

void enable_drole_work_func(int on)
{
	struct ohio_data *ohio = i2c_get_clientdata(ohio_client);
	ohio->drole_on = on;
	queue_delayed_work(ohio->workqueue, &ohio->drole_work, 150);
}

void enable_oc_work_func(void)
{
	struct ohio_data *ohio = i2c_get_clientdata(ohio_client);
	if (!delayed_work_pending(&ohio->oc_enable_work))
		queue_delayed_work(ohio->workqueue, &ohio->oc_enable_work, 20);
	else
		pr_err("%s: delay_queue pending, oc_enable fails\n", __func__);
}

static void oc_enable_work_func(struct work_struct *work)
{
	pr_info("%s: set oc_enable=1\n", __func__);
	oc_enable = 1;
}

static void drole_work_func(struct work_struct *work)
{
	struct ohio_data *td = i2c_get_clientdata(ohio_client);
		
#ifdef OHIO_DEBUG
		pr_info("data role change %d\n", (int)td->drole_on);
#endif
		pr_info("data role change %d\n", (int)td->drole_on);
		
		
		if (cable_connected == 0) {
			pr_err("%s: cable out, should not do anything with event irq\n", __func__);
			return;
		}
		mutex_lock(&td->drole_lock);
		if (td->drole_on) {
			dwc3_pd_drswap(DR_HOST);
			dwc3_otg_set_id_state(0);
			ohio_set_data_value(OHIO_DROLE, DR_HOST);
		}
		
		else {
			if (ohio_get_data_value(OHIO_EMARKER)){
				ohio_set_data_value(OHIO_START_HOST_FLAG, 0);
			}
			dwc3_pd_drswap(DR_DEVICE);
			dwc3_otg_set_id_state(1);
			ohio_set_data_value(OHIO_DROLE, DR_DEVICE);
		}
		dual_role_instance_changed(td->ohio_dual_role_instance);
		mutex_unlock(&td->drole_lock);
}

static void ohio_work_func(struct work_struct *work)
{
	struct ohio_data *td = container_of(work, struct ohio_data,
					       work.work);
	uint fake_irq_triggerred = 0;
	u8 val;

	cable_connected = confirmed_cable_det(td);
	pr_info("%s : detect cable insertion/remove, cable_connected = %d\n",
				__func__, cable_connected);
	if (cable_connected == DONGLE_CABLE_INSERT) {
		
		td->fake_irq_counter = 0;
		mutex_lock(&td->lock);
		ohio_main_process();
		mutex_unlock(&td->lock);
	} else if (cable_connected == DONGLE_CABLE_REMOVE) {
		td->fake_irq_counter++;
		mutex_lock(&td->drole_lock);
		cable_disconnect(td);
		mutex_unlock(&td->drole_lock);
	}
	if (td->fake_irq_counter > MAX_DEB_FAKE_IRQ_COUNT) {
		td->fake_irq_counter = 0;
		fake_irq_triggerred = 1;
	}
	if (fake_irq_triggerred) {
		queue_delayed_work(td->workqueue, &td->debounce_work, 500);
		pr_info("%s %s : Disable cbl_det IRQ due to triggered fake interrupt\n",
			LOG_TAG, __func__);
	} else {
		if (atomic_read(&cbl_det_irq_status)) {
			atomic_set(&cbl_det_irq_status, 0);
			enable_irq(td->cbl_det_irq);
			pr_info("%s: Enable cbl_det IRQ\n", __func__);
			mdelay(1);
			if ((cable_connected != DONGLE_CABLE_REMOVE) &&
				((val = gpio_get_value(td->pdata->gpio_cbl_det)) == DONGLE_CABLE_REMOVE)) {
				pr_info("%s: cable might be removed. do disconnect\n", __func__);
				cable_disconnect(td);
			}
		}
	}
}

static void ohio_comm_isr_work_func(struct work_struct *work)
{
	struct ohio_data *ohio = container_of(work, struct ohio_data,
							 comm_isr_work.work);
	pr_debug("%s : comm_isr work++\n", __func__);

	if ((ohio->fw_version >= 0x10) && (ohio->fw_version != 0x16))
		handle_intr_vector();
	else {
		polling_interface_msg(INTERACE_TIMEOUT_MS);
		clear_soft_interrupt();
	}
	pr_debug("%s : comm_isr work--\n", __func__);
}

static void ohio_debounce_work_func(struct work_struct *work)
{
	struct ohio_data *td = container_of(work, struct ohio_data,
					       debounce_work.work);
	if (atomic_read(&cbl_det_irq_status)) {
		atomic_set(&cbl_det_irq_status, 0);
		enable_irq(td->cbl_det_irq);
	}
	pr_info("%s : Enable cbl_det IRQ due to fake interrupt\n", __func__);
}

int workable_charging_cable(void)
{
	u8 val;
	int ret;
	struct ohio_data *ohio = NULL;

	if(ohio_client)
		ohio = i2c_get_clientdata(ohio_client);
	else
		return -ENODEV;

	if (atomic_read(&ohio_power_status) == 1) {
		ret = ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, NEW_CC_STATUS, &val);
		if (ret < 0) {
			pr_err("%s: i2c fail\n", __func__);
			return 2;
		}
		else {
			pr_info("%s: cc_status = 0x%02X\n", __func__, val);
			switch (val) {
				case 0x00:
					pr_err("%s: cable out\n", __func__);
					return -1;
				case 0x04:
				case 0x40:
				case 0x08:
				case 0x80:
					pr_info("%s: workable cable\n", __func__);
					return 1;
				case 0x0c:
				case 0xc0:
					pr_err("%s: illegal cable\n", __func__);
					return 0;
				default:
					pr_err("%s: unknown status\n", __func__);
					return -1;
			}
		}
	}
	else {
		pr_info("%s: power not ready\n", __func__);
		return 2;
	}
}
EXPORT_SYMBOL(workable_charging_cable);

#ifdef CONFIG_OF
int __maybe_unused ohio_regulator_configure(struct device *dev,
				struct ohio_platform_data *pdata)
{

	int rc = 0;
	pdata->avdd_10 = regulator_get(dev, "analogix,vdd_ana");

	if (IS_ERR(pdata->avdd_10)) {
		rc = PTR_ERR(pdata->avdd_10);
		pr_err("%s : Regulator get failed avdd_10 rc=%d\n",
		       __func__, rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->avdd_10) > 0) {
		rc = regulator_set_voltage(pdata->avdd_10, 1000000, 1000000);
		if (rc) {
			pr_err("%s : Regulator set_vtg failed rc=%d\n",
			       __func__, rc);
			goto error_set_vtg_avdd_10;
		}
	}

	pdata->dvdd_10 = regulator_get(dev, "analogix,vdd_dig");
	if (IS_ERR(pdata->dvdd_10)) {
		rc = PTR_ERR(pdata->dvdd_10);
		pr_err("%s : Regulator get failed dvdd_10 rc=%d\n",
		       __func__, rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->dvdd_10) > 0) {
		rc = regulator_set_voltage(pdata->dvdd_10, 1000000, 1000000);
		if (rc) {
			pr_err("%s : Regulator set_vtg failed rc=%d\n",
			       __func__, rc);
			goto error_set_vtg_dvdd_10;
		}
	}

	return 0;

error_set_vtg_dvdd_10:
	regulator_put(pdata->dvdd_10);
error_set_vtg_avdd_10:
	regulator_put(pdata->avdd_10);

	return rc;
}

static int ohio_parse_dt(struct device *dev,
			    struct ohio_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int temp;

	pdata->gpio_p_on =
	    of_get_named_gpio_flags(np, "analogix,p-on-gpio", 0, NULL);

	pdata->gpio_reset =
	    of_get_named_gpio_flags(np, "analogix,reset-gpio", 0, NULL);

	pdata->gpio_cbl_det =
	    of_get_named_gpio_flags(np, "analogix,cbl-det-gpio", 0, NULL);

#ifdef SUP_VBUS_CTL
	pdata->gpio_vbus_ctrl =
		of_get_named_gpio_flags(np, "analogix,vbus-ctrl-gpio", 0, NULL);
#endif

	pdata->gpio_intr_comm =
	    of_get_named_gpio_flags(np, "analogix,intr-comm-gpio", 0, NULL);

	pdata->gpio_usb_ptn_c1 =
		of_get_named_gpio_flags(np, "analogix,usb-ptn-c1-gpio", 0, NULL);

	pdata->gpio_usb_ptn_c2 =
		of_get_named_gpio_flags(np, "analogix,usb-ptn-c2-gpio", 0, NULL);

	pdata->gpio_vconn_boost =
		of_get_named_gpio_flags(np, "analogix,vconn-boost-gpio", 0, NULL);

	pdata->boost_5v = devm_regulator_get(dev, "V_USB_boost");
	
	if (IS_ERR(pdata->boost_5v)) {
		temp = PTR_ERR(pdata->boost_5v);
		pr_err("%s: Unable to get boost_5v regulator, %d\n", __func__, temp);
		pdata->boost_5v = NULL;
	}

	pr_debug("%s gpio p_on : %d, reset : %d,  gpio_cbl_det %d, gpio_intr_comm %d"
			", gpio_usb_ptn_c1 %d, gpio_usb_ptn_c2 %d, gpio_vconn_boost %d\n",
			LOG_TAG, pdata->gpio_p_on, pdata->gpio_reset, pdata->gpio_cbl_det,
			pdata->gpio_intr_comm, pdata->gpio_usb_ptn_c1, pdata->gpio_usb_ptn_c2,
			pdata->gpio_vconn_boost);

	return 0;
}
#else
static int ohio_parse_dt(struct device *dev,
			    struct ohio_platform_data *pdata)
{
	return -ENODEV;
}
#endif

void eeprom_burst_write(unsigned int eeprom_begin_addr, unsigned char *buf)
{
	unsigned char temp = 0;
	unsigned int i = 0;
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_EE_ADDR_H_BYTE, (eeprom_begin_addr >> 8 ) & 0xff); 
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_EE_ADDR_L_BYTE, (eeprom_begin_addr & 0xff)); 
	for (i = 0; i < 8; i++)
		ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_EE_BURST_DATA0 + i, buf[i]);
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_EE_CTL_2, R_EE_BURST_WR_EN | 0x01);
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_EE_CTL_1, R_ROM_LD_SPEED | R_LOAD_EEPROM_NUM | R_EE_WR_EN);
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_EE_CTL_2, 0x01);
	do{
		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, R_EE_STATE, (unchar *)(&temp));
		mdelay(1);
	} while(!(temp & R_EE_RW_DONE));
	return;
}

unsigned char eeprom_read_byte(unsigned int eeprom_begin_addr)
{
	unsigned char temp = 0;
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_EE_ADDR_H_BYTE, (eeprom_begin_addr >> 8 ) & 0xff); 
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_EE_ADDR_L_BYTE, (eeprom_begin_addr & 0xff)); 
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_EE_CTL_1, 0xCA);

	
	do{
		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, R_EE_STATE, (unchar *)(&temp));
	} while(!(temp & R_EE_RW_DONE));

	ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, R_EE_RD_DATA, (unchar *)(&temp));

	return temp;
}

static ssize_t version_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	u8 temp;
	ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_3 , &temp);
	pr_info("%s : FW version status: %x\n", __func__, temp);
	return snprintf(buf, PAGE_SIZE, "0x%x\n", temp);
}

static ssize_t fw_update_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	int update = 0;
	const struct firmware *fw;
	int ret;
	enum ANX7418_fw_type fw_type;
	enum ANX7418_chip_type chip_type;
	struct ohio_data *ohio = i2c_get_clientdata(ohio_client);
	struct ohio_platform_data *pdata = g_pdata;

	sscanf(buff, "%d", &update);

	if (update != 1)
		goto exit;

	if (!ohio) {
		pr_err("%s %s : Cannot get ohio data.\n", LOG_TAG, __func__);
		goto exit;
	}

	
	disable_irq_nosync(ohio->cbl_det_irq);
	ohio_hardware_powerdown();
	gpio_set_value(pdata->gpio_p_on, 1);
	mdelay(10);
	gpio_set_value(pdata->gpio_reset, 1);
	mdelay(1);
	atomic_set(&ohio_power_status, 1);

	chip_type = check_chip_type();

	switch (chip_type) {
		case EEPROM:
			
			ret = request_firmware(&fw, ANX7418FW, pdev);
			if (ret || fw == NULL) {
				pr_err("firmware request failed (ret = %d, fwptr = %p)", ret, fw);
				goto exit;
			}

			fw_type = check_usbc_fw_type(fw);

			if ((fw_type == EEPROM_FW) && (check_usbc_fw_version(fw, EEPROM) == 1))
				update_firmware(NULL, (struct firmware *)fw);

			release_firmware(fw);
			break;
		case OTP:
			
			ret = request_firmware(&fw, ANX7418FW_OTP, pdev);
			if (ret || fw == NULL) {
				pr_err("firmware request failed (ret = %d, fwptr = %p)", ret, fw);
				goto exit;
			}

			fw_type = check_usbc_fw_type(fw);

			if ((fw_type == OTP_FW) && (check_usbc_fw_version(fw, OTP) == 1))
				update_firmware_otp(NULL, (struct firmware *)fw);

			release_firmware(fw);
			break;
		default:
			pr_err("%s %s : ANX7418 cannot read chip type\n", LOG_TAG, __func__);
			break;
	}

exit:
	return size;
}

uint regOffset = 0;
#define CHECK_BIT(pos,var) ((var)&(1<<(pos)))?1:0
uint dtob(uint num)
{
	if (num == 0)
		return 0;
	else
		return (num % 2) + 10 * dtob(num / 2);
}

ssize_t dump_all_register(char *buf)
{
	int i = 0, len = 0;
	int turn_off_ohio_power = 0;
	uint8_t temp = 0;

	if (atomic_read(&ohio_power_status) == 0) {
		ohio_hardware_poweron();
		turn_off_ohio_power = 1;
	}

	for(i = 0; i < 256; i++){
		if ((i % 0x10 == 0) && i != 0) {
			printk("\n");
			len += snprintf(buf + len, PAGE_SIZE, "\n");
		}
		printk("%x ", OhioReadReg(i));
		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, i, (unchar *)(&temp));
		len += snprintf(buf + len, PAGE_SIZE, "0x%.2x ", temp);
	}

	printk("\n");

	if (turn_off_ohio_power == 1)
		ohio_hardware_powerdown();

	return len;
}
EXPORT_SYMBOL(dump_all_register);

static ssize_t dump_register_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	uint8_t temp = 0;

	ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, regOffset , (unchar *)(&temp));
	pr_info("%s %s : dump register 0x%.2x: 0x%.2x(%d)\n",
		LOG_TAG, __func__, regOffset, temp, dtob(temp));

	return snprintf(buf, PAGE_SIZE, "0x%.2x (%d)\n", temp, dtob(temp));
}

static ssize_t dump_register_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	sscanf(buff, "%d", &regOffset);

	pr_info("%s %s : register: %2x\n", LOG_TAG, __func__, regOffset);

	return size;
}

static ssize_t vconn_en_store(struct device *pdev, struct device_attribute *attr,
				const char *buff, size_t size)
{
	unsigned int temp;
	uint8_t vconn_mask;
	u32 rc;
	struct ohio_data *ohio = NULL;

	if(ohio_client)
		ohio = i2c_get_clientdata(ohio_client);
	else
		return size;

	sscanf(buff, "%u", &temp);

	if (atomic_read(&ohio_power_status) == 1) {
		if (!ohio->pdata->boost_5v) {
			pr_info("%s: no boost 5v regulator, try to get it again...", __func__);
			ohio->pdata->boost_5v = devm_regulator_get(&ohio_client->dev, "V_USB_boost");
			if (IS_ERR(ohio->pdata->boost_5v)) {
				rc = PTR_ERR(ohio->pdata->boost_5v);
				pr_err("%s: still unable to get boost_5v regulator, %d\n", __func__, rc);
				ohio->pdata->boost_5v = NULL;
				return size;
			}
		}

		switch (temp) {
			case 0:  
				vconn_mask = OhioReadReg(INTP_CTRL);
				vconn_mask &= 0x0F;
				OhioWriteReg(INTP_CTRL, vconn_mask);
				gpio_direction_output(ohio->pdata->gpio_vconn_boost, 0);
				rc = regulator_disable(ohio->pdata->boost_5v);
				if (rc) {
					pr_err("%s: Unable to disable boost_5v regulator\n", __func__);
					return size;
				}
				break;
			case 1:  
				rc = regulator_enable(ohio->pdata->boost_5v);
				if (rc) {
					pr_err("%s: Unable to enable boost_5v regulator\n", __func__);
					return size;
				}
				gpio_direction_output(ohio->pdata->gpio_vconn_boost, 1);
				vconn_mask = OhioReadReg(INTP_CTRL);
				vconn_mask &= 0x0F;
				OhioWriteReg(INTP_CTRL, (vconn_mask | 0x10));
				break;
			case 2:  
				rc = regulator_enable(ohio->pdata->boost_5v);
				if (rc) {
					pr_err("%s: Unable to enable boost_5v regulator\n", __func__);
					return size;
				}
				gpio_direction_output(ohio->pdata->gpio_vconn_boost, 1);
				vconn_mask = OhioReadReg(INTP_CTRL);
				vconn_mask &= 0x0F;
				OhioWriteReg(INTP_CTRL, (vconn_mask | 0x20));
				break;
			default:
				break;
		}
		pr_info("%s %s : vconn_en %2x\n", LOG_TAG, __func__, temp);
	}
	return size;
}

static DEVICE_ATTR(dump_register, S_IWUSR|S_IRUGO, dump_register_show, dump_register_store);
static DEVICE_ATTR(version, S_IRUGO, version_show, NULL);
static DEVICE_ATTR(fw_update, S_IWUSR, NULL, fw_update_store);
static DEVICE_ATTR(vconn_en, S_IWUSR, NULL, vconn_en_store);

static struct device_attribute *ohio_attributes[] = {
	&dev_attr_dump_register,
	&dev_attr_version,
	&dev_attr_fw_update,
	&dev_attr_vconn_en,
	NULL
};

static void usb_typec_fwu_progress(struct usb_typec_fwu_notifier *notifier, int percentage)
{
	usb_typec_fw_update_progress(notifier, percentage);
}

static int update_firmware(struct usb_typec_fwu_notifier *notifier, struct firmware *fw)
{
	unsigned int i, j;
	unsigned int pre_progress = 0, progress = 0;
	unsigned char read_temp[8] = {0};
	bool failFlag = 1;

	
	pr_info("%s %s : reset ANX7418 OCM\n", LOG_TAG, __func__);
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, RESET_CTRL_0, R_OCM_RESET);

	
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, EE_KEY_1, EEPROM_ACCESS_KEY_1);
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, EE_KEY_2, EEPROM_ACCESS_KEY_2);
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, EE_KEY_3, EEPROM_ACCESS_KEY_3);
	pr_info("%s %s : start flash OCM fw\n", LOG_TAG, __func__);
	for (j = 0; j < fw->size; j += 8) {
RawWrite_Retry:
		eeprom_burst_write(j, (unsigned char*)(fw->data + j));

		for (i = 0; i < 8; i++) {
			read_temp[i] = eeprom_read_byte(j+i);
			if (fw->data[j+i] != read_temp[i]) {
				failFlag = 1;
			}
		}
		if (failFlag) {
			pr_info("%s %s: verify fail:\n"
				"Image(%d) should be 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n"
				"EEPROM(%d) are 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
				LOG_TAG, __func__, j, fw->data[j+0], fw->data[j+1], fw->data[j+2], fw->data[j+3], fw->data[j+4], fw->data[j+5], fw->data[j+6], fw->data[j+7],
				j, read_temp[0], read_temp[1], read_temp[2], read_temp[3], read_temp[4], read_temp[5], read_temp[6], read_temp[7]);
			failFlag = 0;
			goto RawWrite_Retry;
		}
		progress = (j*100)/(fw->size);
		if (progress > pre_progress+4) {
			pr_info("%s %s : flash progress = %d %%\n", LOG_TAG, __func__, progress);
			pre_progress = progress;
		}
		if (notifier != NULL)
			usb_typec_fwu_progress(notifier, progress);
	}
	pr_info("%s %s : flash OCM fw done\n", LOG_TAG, __func__);
	return 0;
}

unsigned char ECC_encoder(unsigned char *pData)
{
    unsigned char i, j;
    unsigned char result;
    unsigned char c;

    for(result = 0, i = 0; i < 8; i++)
    {
        c = *pData;
        for(j = 0; j < 8; j++)
        {
            if(c & 0x1)
            {
                result ^= ECC_table[(i << 3) + j]; 
            }
            c >>= 1;
        }
        pData++;
    }
    return result;
}

void otp_read_word(unsigned int word_addr, unsigned char *pdata, int ECC_activate)
{
	int i, read_done_retry = 0;
	uint8_t temp = 0;

	if (ECC_activate)
		ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_CTL_1, R_MCU_ACCESS_DISABLE | R_OTP_WAKEUP);
	else
		ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_CTL_1, R_MCU_ACCESS_DISABLE | R_OTP_WAKEUP | R_OTP_ECC_BYPASS);

	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_ADDR_HIGH, (word_addr >> 8) & 0xff);
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_ADDR_LOW, word_addr & 0xff);

	if (ECC_activate)
		ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_CTL_1, R_MCU_ACCESS_DISABLE | R_OTP_WAKEUP | R_OTP_READ);
	else
		ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_CTL_1, R_MCU_ACCESS_DISABLE | R_OTP_WAKEUP | R_OTP_ECC_BYPASS | R_OTP_READ);

	
	while(1) {
		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_STATE, (unchar *)(&temp));
		if ((temp & R_OTP_READ_WORD_STATE) == 0) {
			read_done_retry = 0;
			break;
		} else {
			if (read_done_retry < OTP_READ_DONE_RETRY) {
				read_done_retry++;
				msleep(1);
			} else {
				read_done_retry = 0;
				pr_err("%s %s : OTP read failed at %d word.\n", LOG_TAG, __func__, word_addr);
				break;
			}
		}
	};

	for (i = 0; i < 9; i++)
		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_DATA_OUT_0 + i, (unchar *)(&pdata[i]));

#if VDEBUG
	pr_info("%s %s : address %d word : ", LOG_TAG, __func__, word_addr);
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 16, 1, pdata, 9, 1);
#endif

}

int otp_write_word(unsigned int word_addr, unsigned char *pdata)
{
	int i, count = 0, write_done_retry = 0;
	uint8_t temp = 0;
	unsigned char read_data[9];

retry:
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_ADDR_HIGH, (word_addr >> 8) & 0xff);
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_ADDR_LOW, word_addr & 0xff);

	for (i = 0; i < 8; i++)
		ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_DATA_IN_0 + i, pdata[i]); 

	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_ECC_IN, pdata[8]); 

#if VDEBUG
	print_hex_dump(KERN_INFO, "write data:", DUMP_PREFIX_ADDRESS, 16, 1, pdata, 9, 1);
#endif

	for (i = 0; i < OTP_PROGRAM_MAX; i++) {
		ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_CTL_1, R_MCU_ACCESS_DISABLE | R_OTP_WAKEUP | R_OTP_ECC_BYPASS | R_OTP_WRITE);
		
		while(1) {
			ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_STATE, (unchar *)(&temp));
			if ((temp & R_OTP_WRITE_WORD_STATE) == 0) {
				write_done_retry = 0;
				break;
			} else {
				if (write_done_retry < OTP_WRITE_DONE_RETRY) {
					write_done_retry++;
					msleep(1);
				} else {
					write_done_retry = 0;
					goto write_error;
				}
			}
		};
		otp_read_word(word_addr, read_data, 0); 
#if VDEBUG
		print_hex_dump(KERN_INFO, "read data with ECC bypass:", DUMP_PREFIX_ADDRESS, 16, 1, read_data, 9, 1);
#endif
		if (memcmp(pdata, read_data, 9) == 0) {
			if (count < OTP_WRITE_RETRY) {
				count++;
				goto retry;
			} else {
				return 1;
			}
		}
	}

	otp_read_word(word_addr, read_data, 1); 
#if VDEBUG
	print_hex_dump(KERN_INFO, "read data with ECC activate:", DUMP_PREFIX_ADDRESS, 16, 1, read_data, 9, 1);
#endif
	if (memcmp(pdata, read_data, 9) == 0) {
		if (count < OTP_WRITE_RETRY) {
			count++;
			goto retry;
		} else {
			return 1;
		}
	} else {
		if (count < OTP_WRITE_RETRY) {
			count++;
			goto retry;
		}
	}
write_error:
	pr_err("%s %s : OTP write failed at %d word\n", LOG_TAG, __func__, word_addr);

	print_hex_dump(KERN_INFO, "Expecting data:", DUMP_PREFIX_ADDRESS, 16, 1, pdata, 9, 1);
	print_hex_dump(KERN_INFO, "Read back data:", DUMP_PREFIX_ADDRESS, 16, 1, read_data, 9, 1);

	return 0;
}

static int update_firmware_otp(struct usb_typec_fwu_notifier *notifier, struct firmware *fw)
{
	unsigned char data[9];
	unsigned char read_data[9];
	unsigned char *pfw;
	int ret, i;
	unsigned int pre_progress = 0, progress = 0;
	unsigned int fw_head_addr, new_fw_start_addr, new_fw_size;
	unsigned int old_fw_start_addr, old_fw_size;
	unsigned int program_addr, real_fw_size;

	pr_info("%s %s : flash OCM fw start\n", LOG_TAG, __func__);

	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, POWER_DOWN_CTRL, R_POWER_DOWN_OCM);
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_CTL_1, R_MCU_ACCESS_DISABLE | R_OTP_WAKEUP | R_OTP_ECC_BYPASS);
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_ACC_PROTECT, OTP_ACCESS_KEY);

	
	otp_read_word(0, data, 1);
	if (memcmp(blank_word, data, 9) != 0) {
		pr_err("%s %s : first word is not blank word in OTP.\n", LOG_TAG, __func__);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 16, 1, data, 9, 1);
		goto exit;
	}

	otp_read_word(1, data, 1);
	if (memcmp(blank_word, data, 9) == 0) {
		pr_info("%s %s : Initializing OTP word 1.\n", LOG_TAG, __func__);
		ret = otp_write_word(1, (unsigned char*)(fw->data + 9));
		if (!ret) {
			pr_err("%s %s : OTP update fw header failed, please re-program.\n", LOG_TAG, __func__);
			goto exit;
		}
		fw_head_addr = 2; 
		new_fw_start_addr = 2 + OTP_UPDATE_MAX; 
	} else if (data[4] == 0) {
		pr_err("%s %s : word 1 has been programmed but its content is not expecting.\n", LOG_TAG, __func__);
		goto exit;
	} else {
		i = 1;
		do {
			if (++i >= 2 + OTP_UPDATE_MAX - 1) {
				pr_err("%s %s : OTP no free space for FW header.\n", LOG_TAG, __func__);
				goto exit;
			}
			otp_read_word(i, data, 1);
		} while (memcmp(inactive_word, data, 9) == 0);
		old_fw_start_addr = data[0] + (data[1] << 8); 
		old_fw_size = data[2] + (data[3] << 8); 
		fw_head_addr = i; 
		new_fw_start_addr = old_fw_start_addr + old_fw_size; 
		pr_info("%s %s : old fw start address %04x, old fw size %04x\n", LOG_TAG, __func__, old_fw_start_addr, old_fw_size);
	}

#if DEBUG
	otp_read_word(new_fw_start_addr - 2, data, 1);
	pr_info("%s %s : address %d word : ", LOG_TAG, __func__, new_fw_start_addr - 2);
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 16, 1, data, 9, 1);

	otp_read_word(new_fw_start_addr - 1, data, 1);
	pr_info("%s %s : address %d word : ", LOG_TAG, __func__, new_fw_start_addr - 1);
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 16, 1, data, 9, 1);

	otp_read_word(new_fw_start_addr, data, 1);
	pr_info("%s %s : address %d word : ", LOG_TAG, __func__, new_fw_start_addr);
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 16, 1, data, 9, 1);
#endif
	
	new_fw_size = fw->data[OTP_FW_HEADER_SIZE] + (fw->data[OTP_FW_HEADER_SIZE + 1] << 8);
	pr_info("%s %s : new fw start address %04x, new fw size %04x\n", LOG_TAG, __func__, new_fw_start_addr, new_fw_size);

	if ( (new_fw_size + new_fw_start_addr) > OTP_SIZE) {
		pr_err("%s %s : OTP no free space for FW update.\n", LOG_TAG, __func__);
		goto exit;
	}

	pfw = (unsigned char*)fw->data + OTP_FW_HEADER_LEN;
	program_addr = new_fw_start_addr;
	for (i = 0; i < new_fw_size; i++) {
		ret = otp_write_word(program_addr++, (unsigned char*)(pfw + i * 9));
		if (!ret) {
			pr_err("%s %s : OTP fw update failed, please re-program.\n", LOG_TAG, __func__);
			break;
		}
		progress = (i*100)/(new_fw_size);
		if (progress > pre_progress+4) {
			pr_info("%s %s : flash progress = %d %%\n", LOG_TAG, __func__, progress);
			pre_progress = progress;
		}
		if (notifier != NULL)
			usb_typec_fwu_progress(notifier, progress);
	}

	
	otp_read_word(fw_head_addr, data, 1);
	if (memcmp(blank_word, data, 9) != 0) {
		ret = otp_write_word(fw_head_addr, inactive_word);
		if (!ret) {
			pr_err("%s %s : OTP erasing old fw header failed, please re-program.\n", LOG_TAG, __func__);
			goto exit;
		}
		fw_head_addr++;
	}

	
	memcpy(data, fw->data + OTP_FW_HEADER_START_ADDR, 9);
	data[0] = new_fw_start_addr & 0xff;
	data[1] = (new_fw_start_addr >> 8) & 0xff;
	real_fw_size = program_addr - new_fw_start_addr;
	data[2] = real_fw_size & 0xff;
	data[3] = (real_fw_size >> 8) & 0xff;
	data[8] = ECC_encoder(data);
	print_hex_dump(KERN_INFO, "Update fw header :", DUMP_PREFIX_ADDRESS, 16, 1, data, 9, 1);
	ret = otp_write_word(fw_head_addr, data);
	while (!ret) {
		otp_read_word(fw_head_addr, read_data, 1); 
		pr_err("%s %s : Update new fw header failed.\n", LOG_TAG, __func__);
		print_hex_dump(KERN_INFO, "Read back data:", DUMP_PREFIX_ADDRESS, 16, 1, read_data, 9, 1);
		ret = otp_write_word(fw_head_addr, inactive_word);
		if (!ret) {
			pr_err("%s %s : OTP erasing old fw header failed, please re-program.\n", LOG_TAG, __func__);
			goto exit;
		}
		fw_head_addr++;
		if (fw_head_addr == 2 + OTP_UPDATE_MAX) {
			pr_err("%s %s : OTP no free space for FW header.\n", LOG_TAG, __func__);
			goto exit;
		}
		ret = otp_write_word(fw_head_addr, data);
	}

	pr_info("%s %s : flash OCM fw done\n", LOG_TAG, __func__);

exit:
	ohio_write_reg(OHIO_SLVAVE_I2C_ADDR, R_OTP_ACC_PROTECT, 0x00); 
	if (ret == 0)
		return -1;
	return 0;
}

static int check_usbc_fw_version(const struct firmware *fw, int fw_type)
{
	unchar curr_version, updating_version;
	int ret = 0;

	if (fw_type == EEPROM) {
		updating_version = fw->data[EEPROM_FW_VERSION_OFFSET];
	} else if (fw_type == OTP) {
		updating_version = fw->data[OTP_FW_VERSION_OFFSET];
	} else {
		pr_err("%s %s : cannot get updating firmware version\n", LOG_TAG, __func__);
		return 0; 
	}

	
	ret = ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_3 , (unchar *)(&curr_version));
	if (ret < 0) {
		pr_err("%s %s : cannot read firmware version\n", LOG_TAG, __func__);
		return 0; 
	}

	pr_info("%s %s : curr_version= %02x , updating_version= %02x\n",
			LOG_TAG, __func__, curr_version, updating_version);

	if ((curr_version == 0x16) && updating_version >= 0x10 && updating_version != 0x16) {
		
		pr_info("%s %s : Need Update\n", LOG_TAG, __func__);
		return 1;
	} else if (updating_version > curr_version && updating_version != 0x16) {
		
		pr_info("%s %s : Need Update\n", LOG_TAG, __func__);
		return 1;
	} else {
		pr_info("%s %s : Update Bypass\n", LOG_TAG, __func__);
		return 0; 
	}
}

int analogix_usbc_fw_update(struct usb_typec_fwu_notifier *notifier, struct firmware *fw)
{
	int ret;
	bool pre_power_status;
	enum ANX7418_chip_type chip_type;
	enum ANX7418_fw_type fw_type;

	struct ohio_data *ohio = NULL;

	if(ohio_client)
		ohio = i2c_get_clientdata(ohio_client);
	else {
		pr_err("%s : ANX7418 chip not ready\n", __func__);
		return 1; 
	}

	pr_info("%s\n", __func__);

	
	disable_irq_nosync(ohio_client->irq);
	disable_irq_nosync(ohio->cbl_det_irq);
	pr_info("%s: Waiting 500ms for current work end\n", __func__);
	mdelay(500); 
	if(atomic_read(&ohio_power_status) != 1) {
		ohio_hardware_poweron();
		pre_power_status = 0; 
	}
	else {
		pre_power_status = 1;
	}

	usb_typec_fwu_progress(notifier, 0);

	chip_type = check_chip_type();
	fw_type = check_usbc_fw_type(fw);

	switch (chip_type) {
		case EEPROM:
			if (fw_type != EEPROM_FW) {
				pr_err("%s %s : ANX7418 chip type not match to the firmware\n", LOG_TAG, __func__);
				ret = 1; 
				break;
			}
			ret = check_usbc_fw_version(fw, EEPROM);
			if (ret == 0) {
				ret = 1;
				goto exit;
			}
			ret = update_firmware(notifier, fw);
			break;
		case OTP:
			if (fw_type != OTP_FW) {
				pr_err("%s %s : ANX7418 chip type not match to the firmware\n", LOG_TAG, __func__);
				ret = 1; 
				break;
			}
			ret = check_usbc_fw_version(fw, OTP);
			if (ret == 0) {
				ret = 1;
				goto exit;
			}
			ret = update_firmware_otp(notifier, fw);
			break;
		default:
			pr_err("%s %s : ANX7418 cannot read chip type\n", LOG_TAG, __func__);
			ret = 1; 
			break;
	}

exit:
	usb_typec_fwu_progress(notifier, 100);
	enable_irq(ohio->cbl_det_irq);
	enable_irq(ohio_client->irq);
	if (!pre_power_status)
		ohio_hardware_powerdown();
	return ret;
}

int register_usb_typec_fw_update(struct usb_typec_fwu_notifier *notifier)
{
	notifier->fwupdate = analogix_usbc_fw_update;
	notifier->flash_timeout = 360;
	
	snprintf(notifier->fw_vendor,
			sizeof(notifier->fw_vendor), "%s", "ANALOGIX");
	return register_usbc_fw_update(notifier);

}

static void usb_typec_fw_update_deinit(struct usb_typec_fwu_notifier *notifier)
{
	unregister_usbc_fw_update(notifier);
	kfree(notifier);
	notifier = NULL;
}

enum dual_role_property ohio_properties[5] = {
	DUAL_ROLE_PROP_SUPPORTED_MODES,
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
	DUAL_ROLE_PROP_VCONN_SUPPLY,
};

static const struct dual_role_phy_desc ohio_desc = {
	.name = "otg_default",
	.properties = ohio_properties,
	.num_properties = 5,
	.get_property = ohio_dual_role_get_property,
	.set_property = ohio_dual_role_set_property,
	.property_is_writeable = ohio_dual_role_property_is_writeable,
};

static int ohio_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{

	struct device_attribute **attrs = ohio_attributes;
	struct device_attribute *attr;

	struct ohio_data *ohio;
	struct ohio_platform_data *pdata;
	int ret = 0;

	pr_info("%s start\n", __func__);

	
	if ((get_debug_flag() & 0x200)  && !((strcmp(htc_get_bootmode(), "download") == 0)
		|| (strcmp(htc_get_bootmode(), "RUU") == 0))) {
		pr_info("%s skip ANX7418 driver probe on %s mode\n", __func__, htc_get_bootmode());
		goto exit;
	}
	

	if (!i2c_check_functionality(client->adapter,
	I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("%s: i2c bus does not support the ohio\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	ohio = kzalloc(sizeof(struct ohio_data), GFP_KERNEL);
	if (!ohio) {
		pr_err("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct ohio_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			ret = -ENOMEM;
			goto err0;
		}
		client->dev.platform_data = pdata;
		
		ret = ohio_parse_dt(&client->dev, pdata);
		if (ret != 0)	
			goto err0;

		ohio->pdata = pdata;
	} else {
		ohio->pdata = client->dev.platform_data;
	}

	ohio->usbc_fwu_notifier = kzalloc(sizeof(struct usb_typec_fwu_notifier), GFP_KERNEL);
	if (!ohio->usbc_fwu_notifier) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err0;
	}

	register_usb_typec_fw_update(ohio->usbc_fwu_notifier);
	pr_debug("%s register usb typec FW update driver\n", __func__);

	
	g_pdata = ohio->pdata;
	ohio_client = client;
	ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);

	atomic_set(&ohio_power_status, 0);
	atomic_set(&cbl_det_irq_status, 0);

	mutex_init(&ohio->lock);
	mutex_init(&ohio->drole_lock);

	ohio_pinctrl = devm_pinctrl_get(&client->dev);
	ohio_default_state = pinctrl_lookup_state(ohio_pinctrl , "default");
	pinctrl_select_state(ohio_pinctrl , ohio_default_state);

	if (!ohio->pdata) {
		ret = -EINVAL;
		goto err0;
	}

	ret = ohio_init_gpio(ohio);
	if (ret) {
		pr_err("%s: failed to initialize gpio\n", __func__);
		goto err0;
	}

	INIT_DELAYED_WORK(&ohio->work, ohio_work_func);
	INIT_DELAYED_WORK(&ohio->debounce_work, ohio_debounce_work_func);
	INIT_DELAYED_WORK(&ohio->comm_isr_work, ohio_comm_isr_work_func);
	INIT_DELAYED_WORK(&ohio->drole_work, drole_work_func);
	INIT_DELAYED_WORK(&ohio->oc_enable_work, oc_enable_work_func);

	ohio->non_standard_flag = 0;

	ohio->comm_workqueue = create_singlethread_workqueue("ohio_comm_work");
	if (ohio->comm_workqueue == NULL) {
		pr_err("%s: failed to create comm work queue\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

	ohio->workqueue = create_singlethread_workqueue("ohio_work");
	if (ohio->workqueue == NULL) {
		pr_err("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

	ohio->cbl_det_irq = gpio_to_irq(ohio->pdata->gpio_cbl_det);
	if (ohio->cbl_det_irq < 0) {
		pr_err("%s : failed to get gpio irq\n", __func__);
		goto err2;
	}

	wake_lock_init(&ohio->ohio_lock,
		       WAKE_LOCK_SUSPEND, "ohio_wake_lock"); 

	ret = request_threaded_irq(ohio->cbl_det_irq, NULL, ohio_cbl_det_isr,
				   IRQF_TRIGGER_FALLING  | IRQF_TRIGGER_RISING
				   | IRQF_ONESHOT, "ohio-cbl-det", ohio);
	if (ret < 0) {
		pr_err("%s : failed to request irq\n", __func__);
		goto err3;
	}

	ret = irq_set_irq_wake(ohio->cbl_det_irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for cable detect", __func__);
		pr_err("interrupt wake set fail\n");
		goto err4;
	}

	ret = enable_irq_wake(ohio->cbl_det_irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for cable detect", __func__);
		pr_err("interrupt wake enable fail\n");
		goto err4;
	}

	client->irq = gpio_to_irq(ohio->pdata->gpio_intr_comm);
	if (client->irq < 0) {
		pr_err("%s : failed to get ohio gpio comm irq\n", __func__);
		goto err3;
	}

	ret = request_threaded_irq(client->irq, NULL, ohio_intr_comm_isr,
				   IRQF_TRIGGER_RISING | IRQF_ONESHOT, "ohio-intr-comm", ohio);
	if (ret < 0) {
		pr_err("%s : failed to request interface irq\n", __func__);
		goto err4;
	}

	ret = irq_set_irq_wake(client->irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for interface communaction", __func__);
		pr_err("interrupt wake set fail\n");
		goto err4;
	}

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for interface communaction", __func__);
		pr_err("interrupt wake enable fail\n");
		goto err4;
	}

	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		pr_err("%s : sysfs register failed", __func__);
		goto err4;
	}

	while ((attr = *attrs++))
		ret = device_create_file(&client->dev, attr);

	cable_connected = 0;

	
	gpio_direction_output(ohio->pdata->gpio_usb_ptn_c1, 0);
	gpio_set_value(ohio->pdata->gpio_usb_ptn_c2, 0);

	pr_debug("%s USB3.0 re-drive status (c1, c2)= (%d, %d)\n",
		__func__, gpio_get_value(ohio->pdata->gpio_usb_ptn_c1),
		gpio_get_value(ohio->pdata->gpio_usb_ptn_c2));

	
	queue_delayed_work(ohio->workqueue, &ohio->work, msecs_to_jiffies(2000));
	pr_info("%s: delay 2000ms to detect cbl_det\n", __func__);

	i2c_set_clientdata(client, ohio);

	ohio->ohio_dual_role_instance = devm_dual_role_instance_register(&client->dev, &ohio_desc);
	if (IS_ERR(ohio->ohio_dual_role_instance)) {
		pr_err("%s: dual_role_instance register fail\n", __func__);
		goto err5;
	}

	ohio->fw_version = 0xFF;

	pr_info("%s successfully\n", __func__);
	goto exit;

err5:
	devm_dual_role_instance_unregister(&client->dev, ohio->ohio_dual_role_instance);
err4:
	free_irq(client->irq, ohio);
err3:
	free_irq(ohio->cbl_det_irq, ohio);
err2:
	destroy_workqueue(ohio->workqueue);
	destroy_workqueue(ohio->comm_workqueue);
err1:
	ohio_free_gpio(ohio);
err0:
	ohio_client = NULL;
	kfree(ohio);
exit:
	return ret;
}

static int ohio_i2c_remove(struct i2c_client *client)
{
	struct ohio_data *ohio = i2c_get_clientdata(client);

	free_irq(client->irq, ohio);
	ohio_free_gpio(ohio);
	usb_typec_fw_update_deinit(ohio->usbc_fwu_notifier);
	destroy_workqueue(ohio->workqueue);
	wake_lock_destroy(&ohio->ohio_lock);  
	kfree(ohio);
	return 0;
}


static int ohio_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int ohio_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ohio_id[] = {
	{"ohio", 0x50},
	{}
};

MODULE_DEVICE_TABLE(i2c, ohio_id);

#ifdef CONFIG_OF
static struct of_device_id anx_match_table[] = {
	{.compatible = "analogix,ohio",},
	{},
};
#endif

static struct i2c_driver ohio_driver = {
	.driver = {
		   .name = "ohio",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = anx_match_table,
#endif
		   },
	.probe = ohio_i2c_probe,
	.remove = ohio_i2c_remove,
	.suspend = ohio_i2c_suspend,
	.resume = ohio_i2c_resume,
	.id_table = ohio_id,
};

static void __init ohio_init_async(void *data, async_cookie_t cookie)
{
	int ret = 0;

	ret = i2c_add_driver(&ohio_driver);
	if (ret < 0)
		pr_err("%s: failed to register ohio i2c drivern", __func__);
}

static int __init ohio_init(void)
{
	async_schedule(ohio_init_async, NULL);
	return 0;
}

static void __exit ohio_exit(void)
{
	i2c_del_driver(&ohio_driver);
}


static void __maybe_unused hardware_power_ctl(unchar enable)
{
	if(enable == 0)
		ohio_hardware_powerdown();
	else
		ohio_hardware_poweron();
}

void ohio_debug_dump(void)
{
	uint temp;
	uint ssrx, sstx;

	if (atomic_read(&ohio_power_status) == 1) {
#if DEBUG
		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_STATUS , (unchar *)(&temp));
		pr_info("%s %s : ANALOG_STATUS status: %x\n", LOG_TAG, __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, POWER_DOWN_CTRL , (unchar *)(&temp));
		pr_info("%s %s : POWER_DOWN_CTRL status: %x\n", LOG_TAG, __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_0 , (unchar *)(&temp));
		pr_info("%s %s : ANALOG_CTRL_0 status: %x\n", LOG_TAG, __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1 , (unchar *)(&temp));
		pr_info("%s %s : ANALOG_CTRL_1 status: %x\n", LOG_TAG, __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_2 , (unchar *)(&temp));
		pr_info("%s %s : ANALOG_CTRL_2 status: %x\n", LOG_TAG, __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_3 , (unchar *)(&temp));
		pr_info("%s %s : ANALOG_CTRL_3 status: %x\n", LOG_TAG, __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_4 , (unchar *)(&temp));
		pr_info("%s %s : ANALOG_CTRL_4 status: %x\n", LOG_TAG, __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5 , (unchar *)(&temp));
		pr_info("%s %s : ANALOG_CTRL_5 status: %x\n", LOG_TAG, __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_6 , (unchar *)(&temp));
		pr_info("%s %s : ANALOG_CTRL_6 status: %x\n", LOG_TAG, __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_7 , (unchar *)(&temp));
		pr_info("%s %s : ANALOG_CTRL_7 status: %x\n", LOG_TAG, __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_8 , (unchar *)(&temp));
		pr_info("%s %s : ANALOG_CTRL_8 status: %x\n", LOG_TAG, __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_9 , (unchar *)(&temp));
		pr_info("%s %s : ANALOG_CTRL_9 status: %x\n", LOG_TAG, __func__, temp);
#endif

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_STATUS , (unchar *)(&temp));
		pr_debug("%s : ANALOG_STATUS status: %02x\n", __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, INTP_CTRL, (unchar *)(&temp));
		pr_debug("%s : INTP_CTRL status: %02x\n", __func__, temp);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, POWER_DOWN_CTRL , (unchar *)(&temp));

		if (temp & CC1_ATTACH)
			pr_info("%s : CC pin connect to CC1\n", __func__);
		else if (temp & CC2_ATTACH)
			pr_info("%s : CC pin connect to CC2\n", __func__);
		else
			pr_info("%s : Unknown CC pin status\n", __func__);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_1 , (unchar *)(&ssrx));

		if (ssrx & SSRX_SWITCH_CC1)
			pr_debug("%s : SSRX switch to CC1\n", __func__);
		else if (ssrx & SSRX_SWITCH_CC2)
			pr_debug("%s : SSRX switch to CC2\n", __func__);
		else
			pr_debug("%s : Unknown SSRX status\n", __func__);

		ohio_read_reg(OHIO_SLVAVE_I2C_ADDR, ANALOG_CTRL_5 , (unchar *)(&sstx));

		if (sstx & SSTX_SWITCH_CC1)
			pr_debug("%s : SSTX switch to CC1\n", __func__);
		else if (sstx & SSTX_SWITCH_CC2)
			pr_debug("%s : SSTX switch to CC2\n", __func__);
		else
			pr_debug("%s : Unknown SSTX status\n", __func__);
	}
}

void dump_reg(void)
{
	int i = 0;
	u8 val = 0;
	printk("dump registerad:\n");
	printk("\n        0    1   2   3   4   5   6   7   8   9   A   B   C   D   E   F\n");
	for (i=0; i<256; i++){
		val = OhioReadReg(i);

		if ((i)%0x10 == 0x00){
			printk("\nReg[%d]: %2x  ", i, val);
		}
		else
			printk("%02x  ", val);
	}
	printk("\n");
}

ssize_t anx_ohio_send_pd_cmd(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
	int cmd;
	int result;

	result = sscanf(buf, "%d", &cmd);
	switch (cmd) {
	case TYPE_PWR_SRC_CAP:
		send_pd_msg(TYPE_PWR_SRC_CAP, 0, 0);
		break;
	case TYPE_PWR_SNK_CAP:  
		update_pwr_sink_caps();
		break;

	case TYPE_DP_SNK_IDENTITY:
		send_pd_msg(TYPE_DP_SNK_IDENTITY, 0, 0);
		break;

	case TYPE_PSWAP_REQ:
		send_pd_msg(TYPE_PSWAP_REQ, 0, 0);
		break;
	case TYPE_DSWAP_REQ:
		send_pd_msg(TYPE_DSWAP_REQ, 0, 0);
		break;

	case TYPE_GOTO_MIN_REQ:
		send_pd_msg(TYPE_GOTO_MIN_REQ, 0, 0);
		break;

	case TYPE_VDM:
		update_VDM();
		break;

	case TYPE_PWR_OBJ_REQ:
		interface_send_request();
		break;
	case TYPE_ACCEPT:
		interface_send_accept();
		break;
	case TYPE_REJECT:
		interface_send_reject();
		break;
	case TYPE_SOFT_RST:
		send_pd_msg(TYPE_SOFT_RST, 0,0);
		break;
	case TYPE_HARD_RST:
		send_pd_msg(TYPE_HARD_RST, 0,0);
		break;
	case 0xFD:
		pr_info("fetch powerrole: %d\n", get_power_role());
		break;
	case 0xFE:
		pr_info("fetch datarole: %d\n", get_data_role());
		break;

	case 0xff:
		dump_reg();
		break;
	}
	return count;
}

ssize_t __maybe_unused anx_ohio_send_thr_cmd(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
        int cmd;
        int result;

        result = sscanf(buf, "%d", &cmd);
       
        return count;
}

ssize_t anx_ohio_send_pswap(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", send_power_swap());
}

ssize_t anx_ohio_send_dswap(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", send_data_swap());
}

ssize_t anx_ohio_try_source(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", try_source());
}

ssize_t anx_ohio_try_sink(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", try_sink());
}

ssize_t anx_ohio_get_data_role(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", get_data_role());
}

ssize_t anx_ohio_get_power_role(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", get_power_role());
}

ssize_t anx_ohio_rd_reg(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
	int cmd;
	int result;

	result = sscanf(buf, "%d", &cmd);
	printk( "reg[%x] = %x\n", cmd, OhioReadReg(cmd));

	return count;
}

ssize_t anx_ohio_wr_reg(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
	int cmd, val;
	int result;

	result = sscanf(buf, "%d  %d", &cmd, &val);
	pr_info("c %x val %x\n", cmd, val);
	OhioWriteReg(cmd, val);
	pr_info("reg[%x] = %x\n", cmd, OhioReadReg(cmd));
	return count;
}
ssize_t anx_ohio_dump_register(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	int i = 0;
	for(i=0;i<256; i++){
		printk("%x", OhioReadReg(i));
		if (i % 0x10 == 0)
			pr_info("\n");

		snprintf(&buf[i], sizeof(u8), "%d", OhioReadReg(i));
	}

	printk("\n");

	return i;
}

ssize_t anx_ohio_select_rdo_index(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int cmd;
	cmd = sscanf(buf, "%d", &cmd);
	if (cmd <= 0)
		return 0;
	pr_info("NewRDO idx %d, Old idx %d\n", cmd, sel_voltage_pdo_index);
	sel_voltage_pdo_index = cmd;
	return count;
}


static struct device_attribute anx_ohio_device_attrs[] = {
	__ATTR(pdcmd, S_IWUSR, NULL,anx_ohio_send_pd_cmd),
	__ATTR(thrcmd, S_IWUSR, NULL, anx_ohio_send_thr_cmd),
	__ATTR(rdreg, S_IWUSR, NULL, anx_ohio_rd_reg),
	__ATTR(wrreg, S_IWUSR, NULL, anx_ohio_wr_reg),
	__ATTR(dumpreg, S_IRUGO, anx_ohio_dump_register, NULL),
	__ATTR(drole, S_IRUGO, anx_ohio_get_power_role, NULL),
	__ATTR(prole, S_IRUGO, anx_ohio_get_data_role, NULL),
	__ATTR(trysrc, S_IRUGO | S_IWUSR, anx_ohio_try_source, NULL),
	__ATTR(trysink, S_IRUGO | S_IWUSR, anx_ohio_try_sink, NULL),
	__ATTR(pswap, S_IRUGO | S_IWUSR, anx_ohio_send_pswap, NULL),
	__ATTR(dswap, S_IRUGO | S_IWUSR, anx_ohio_send_dswap, NULL)
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	pr_debug("ohio create system fs interface ...\n");
	for (i = 0; i < ARRAY_SIZE(anx_ohio_device_attrs); i++)
		if (device_create_file(dev, &anx_ohio_device_attrs[i]))
			goto error;
	pr_debug("success\n");
	return 0;
error:
	for (; i >= 0; i--)
		device_remove_file(dev, &anx_ohio_device_attrs[i]);
	pr_err("%s: ohio Unable to create interface", __func__);
	return -EINVAL;
}

module_init(ohio_init);
module_exit(ohio_exit);

MODULE_DESCRIPTION("USB PD Ohio driver");
MODULE_AUTHOR("Xia Junhua <jxia@analogixsemi.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.7");
