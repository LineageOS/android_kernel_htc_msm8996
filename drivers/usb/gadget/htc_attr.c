/*
 * Copyright (C) 2015 HTC, Inc.
 * Author: Dyson Lee <Dyson@intel.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

static unsigned int usb_project_pid; 
static bool bundle_headset = false;

#define REQUEST_RESET_DELAYED (HZ / 10)  

int htc_usb_enable_function(char *name, int ebl)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	char state_buf[60];
	char name_buf[60];
	char *function[3];
	if (!strcmp(name, "ffs"))
		snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%s", "adb");
	else
		snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%s", name);
	function[0] = state_buf;
	function[2] = NULL;

	if (ebl) {
		snprintf(name_buf, sizeof(name_buf),
				"SWITCH_NAME=%s", "function_switch_on");
		function[1] = name_buf;
		kobject_uevent_env(&cdev->sw_function_switch_on.dev->kobj, KOBJ_CHANGE,
				function);
	} else {
		snprintf(name_buf, sizeof(name_buf),
				"SWITCH_NAME=%s", "function_switch_off");
		function[1] = name_buf;
		kobject_uevent_env(&cdev->sw_function_switch_off.dev->kobj, KOBJ_CHANGE,
				function);
	}
	return 0;

}

bool is_bundle_headset(void)
{
	return bundle_headset;
}
EXPORT_SYMBOL_GPL(is_bundle_headset);

bool is_autosuspend(const u16 idVendor, const u16 idProduct, int on)
{
	bool match_vpid = false;
	if (idVendor == 0x170D && idProduct == 0x0523) 
		match_vpid = true;
	else if (idVendor == 0x0BDA && idProduct == 0x4805) 
		match_vpid = true;
	else if (idVendor == 0x0ECB && idProduct == 0x1ECB) 
		match_vpid = true;
	else
		match_vpid = false;
	
	
	
	if (match_vpid && on)
		bundle_headset = true;
	else
		bundle_headset = false;

	return match_vpid;
}
EXPORT_SYMBOL_GPL(is_autosuspend);

static ssize_t iSerial_show(struct device *dev, struct device_attribute *attr,
	char *buf);

static ssize_t store_dummy_usb_serial_number(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct android_dev *android_dev = _android_dev;
	int loop_i;

	if (size >= sizeof(serial_string)) {
		pr_info("%s(): input size > %zu\n", __func__, sizeof(serial_string));
		return -EINVAL;
	}

	for (loop_i = 0; loop_i < size; loop_i++) {
		if (buf[loop_i] >= 0x30 && buf[loop_i] <= 0x39) 
			continue;
		else if (buf[loop_i] >= 0x41 && buf[loop_i] <= 0x5A) 
			continue;
		if (buf[loop_i] == 0x0A) 
			continue;
		else {
			pr_info("%s(): get invaild char (0x%2.2X)\n",
					__func__, buf[loop_i]);
			return -EINVAL;
		}
	}
	strlcpy(serial_string, buf, sizeof(serial_string));
	strim(serial_string);
	strings_dev[STRING_SERIAL_IDX].s = serial_string;
	if (android_dev->connected)
		schedule_delayed_work(&android_dev->request_reset,REQUEST_RESET_DELAYED);
	return size;
}

const char * change_charging_to_ums(const char *buff) {
	if (!strcmp(buff, "charging"))
		return "mass_storage";
	else if (!strcmp(buff, "adb"))
		return "mass_storage,adb";
	return buff;
}

void change_charging_pid_to_ums(struct usb_composite_dev *cdev) {
	switch(cdev->desc.idProduct) {
		case 0x0f0b:
			cdev->desc.idVendor = 0x0bb4;
			cdev->desc.idProduct = 0x0ff9;
			break;
		case 0x0c81:
			cdev->desc.idVendor = 0x0bb4;
			cdev->desc.idProduct = 0x0f86;
			break;
		default:
			break;
	}
	return ;
}

const char * add_usb_radio_debug_function(const char *buff) {

	
	if (!strcmp(buff, "mtp,adb,mass_storage")) 
		return "mtp,adb,mass_storage,diag,modem,rmnet";
	else if (!strcmp(buff, "mtp,adb,mass_storage,acm")) 
		return "mtp,adb,mass_storage,diag,modem,rmnet";
	else if (!strcmp(buff, "mtp,mass_storage")) 
		return "mtp,mass_storage,diag,modem,rmnet";
	else if (!strcmp(buff, "mtp,mass_storage,acm")) 
		return "mtp,mass_storage,diag,modem,rmnet";

	
	if (!strcmp(buff, "ffs,acm")) 
		return "adb,diag,modem,acm";

	
	if (!strcmp(buff, "mass_storage,adb")) 
		return "mass_storage,adb,diag,modem,rmnet";
	else if (!strcmp(buff, "mass_storage")) 
		return "mass_storage,diag,modem,rmnet";
	else if (!strcmp(buff, "rndis,adb")) 
		return "rndis,adb,diag,modem";
	else if (!strcmp(buff, "rndis")) 
		return "rndis,diag,modem";
	else if (!strcmp(buff, "mtp")) 
		return "mtp,diag,modem,rmnet";
	else if (!strcmp(buff, "mtp,adb")) 
		return "mtp,adb,diag,modem,rmnet";

	return buff;
}

void check_usb_vid_pid(struct usb_composite_dev *cdev) {
	switch(cdev->desc.idProduct) {
		case 0x0c93:
			cdev->desc.idProduct = 0x0f12;
			break;
		case 0x0f90:
		case 0x0f63:
			cdev->desc.idProduct = 0x0fa2;
			break;
		case 0x0f25:
		case 0x0f26:
		case 0x0f91:
		case 0x0f64:
			cdev->desc.idProduct = 0x0fa3;
			break;
		case 0x0f15:
			cdev->desc.idProduct = 0x0f17;
			break;
		case 0x0f86:
			cdev->desc.idProduct = 0x0fd8;
			break;
		case 0x0f87:
		case 0x0ca8:
			cdev->desc.idProduct = 0x0f11;
			break;
		case 0x0ff9:
			cdev->desc.idProduct = 0x0fd9;
			break;
		case 0x0ffc:
			cdev->desc.idProduct = 0x0f83;
			break;
		case 0x0ffe:
			cdev->desc.idProduct = 0x0f82;
			break;
		default:
			break;
	}
	cdev->desc.idVendor = 0x0bb4;
	return;
}

void check_usb_project_pid(struct usb_composite_dev *cdev) {
	if (cdev->desc.idProduct == 0x0f90 && usb_project_pid != 0x0000) {
		cdev->desc.idVendor = 0x0bb4;
		cdev->desc.idProduct = usb_project_pid;
	}
	return;
}

static int __init get_usb_project_pid(char *str)
{
	int ret = kstrtouint(str, 0, &usb_project_pid);
	pr_info("androidusb.pid %d: %08x from %26s\r\n",
			ret, usb_project_pid, str);
	return ret;
} early_param("androidusb.pid", get_usb_project_pid);

extern int usb_get_dwc_property(int prop_type);
static ssize_t show_usb_ac_cable_status(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned length;
	length = sprintf(buf, "%d", usb_get_dwc_property(PROPERTY_CHG_STATUS));
	return length;
}

int usb_ats = 0;
static ssize_t store_ats(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &usb_ats);
	return count;
}

static ssize_t show_ats(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", (get_debug_flag() & 0x100) || usb_ats);
	USB_INFO("%s: %s\n", __func__, buf);
	return length;
}


static int usb_disable;

static ssize_t show_usb_disable_setting(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", usb_disable);
	return length;
}

void htc_dwc3_disable_usb(int disable_usb);
static ssize_t store_usb_disable_setting(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int disable_usb_function;

	sscanf(buf, "%d ", &disable_usb_function);
	USB_INFO("USB_disable set %d\n", disable_usb_function);
	usb_disable = disable_usb_function;
	htc_dwc3_disable_usb(disable_usb_function);
	return count;
}

static ssize_t show_usb_cable_connect(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	struct android_dev *and_dev = _android_dev;
	struct usb_composite_dev *cdev = and_dev->cdev;
	int ret = 0;
	if ((and_dev->connected == 1) && !usb_disable ? 1 : 0) {
		if (cdev->gadget->speed == USB_SPEED_SUPER) {
			ret = 2;
		} else
			ret = 1;
	}
	length = sprintf(buf, "%d", ret);
	return length;
}


static ssize_t show_os_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", os_type);
	pr_info("[USB] %s: %s\n", __func__, buf);
	return length;
}
static ssize_t store_usb_modem_enable_setting(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int usb_modem_enable;

	sscanf(buf, "%d ", &usb_modem_enable);
	USB_INFO("modem: enable %d\n", usb_modem_enable);
	htc_usb_enable_function("modem", usb_modem_enable?1:0);
	return count;
}


static ssize_t show_speed(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_dev *device = _android_dev;
	struct usb_composite_dev *cdev = device->cdev;

	return snprintf(buf, PAGE_SIZE, "%s\n",
		usb_speed_string(cdev->gadget->speed));
}
int usb_lock_speed = 1;
static ssize_t show_lock_speed(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	USB_INFO("usb_lock_speed: read lock_speed %d\n", usb_lock_speed);
	return snprintf(buf, PAGE_SIZE, "%d\n", usb_lock_speed);
}

static ssize_t store_lock_speed(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct android_dev *and_dev = _android_dev;

	if ((get_debug_flag() & 0x200)  && !((strcmp(htc_get_bootmode(), "download") == 0)
		|| (strcmp(htc_get_bootmode(), "RUU") == 0))) {
		pr_info("%s skip on %s mode\n", __func__, htc_get_bootmode());
		return count;
	}

	
	sscanf(buf, "%d ", &usb_lock_speed);
	USB_INFO("usb_lock_speed: enable %d\n", usb_lock_speed);
	if (usb_lock_speed == 1)
		ufp_switch_usb_speed(0);
	else
		ufp_switch_usb_speed(1);
	if (and_dev->connected && usb_lock_speed == 2)
		usb_get_dwc_property(PROPERTY_RESTART_USB);
	return count;
}

int usb_lock_host_speed = 1;
static ssize_t store_lock_host_speed(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &usb_lock_host_speed);
	USB_INFO("usb_lock_host_speed: enable %d\n", usb_lock_host_speed);

	return count;
}

extern ssize_t dump_all_register(char *buf);
static ssize_t show_typec_dump_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	USB_INFO("%s : dump ANX7418 register\n", __func__);
	return dump_all_register(buf);
}

extern int ohio_get_data_value(int data_member);
static ssize_t show_pd_cap(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int pd_cap;
	pd_cap = ohio_get_data_value(8);
	if (pd_cap < 0)
		pd_cap = 0;
	USB_INFO("%s : show pd capability = %d\n", __func__, pd_cap);
	return snprintf(buf, PAGE_SIZE, "%d\n", pd_cap);
}

static ssize_t show_typec_fw_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int typec_fw_version;
	typec_fw_version = ohio_get_data_value(9);
	if (typec_fw_version < 0)
		typec_fw_version = 0;
	USB_INFO("%s : show typec fw version = %02x\n", __func__, typec_fw_version);
	return snprintf(buf, PAGE_SIZE, "%02x\n", typec_fw_version);
}

static ssize_t show_typec_chip_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int typec_chip_version;
	typec_chip_version = ohio_get_data_value(10);
	if (typec_chip_version < 0)
		typec_chip_version = 0;
	USB_INFO("%s : show typec fw version = %02x\n", __func__, typec_chip_version);
	return snprintf(buf, PAGE_SIZE, "%02x\n", typec_chip_version);
}

static ssize_t show_typec_fw_update_info(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int typec_fw_update_info;
	typec_fw_update_info = ohio_get_data_value(11);
	if (typec_fw_update_info < 0)
		typec_fw_update_info = 0;
	USB_INFO("%s : show typec fw update info = %d\n", __func__, typec_fw_update_info);
	return snprintf(buf, PAGE_SIZE, "%d\n", typec_fw_update_info);
}

static char *typec_cable_info[] = {
	"No Cable",
	"SRC Rd",
	"SRC Ra",
	"SNK default (56kohm)",
	"SNK Power 1.5A (21kohm)",
	"SNK Power 3.0A (10kohm)",
	"Unknown status"
};

static ssize_t show_typec_cable_info(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val = 0;
	val = ohio_get_data_value(12);
	if (val < 0) {
		val = 6;
	}

	USB_INFO("%s : show typec cable info = %s\n", __func__, typec_cable_info[val]);
	return snprintf(buf, PAGE_SIZE, "%s\n", typec_cable_info[val]);
}

static DEVICE_ATTR(dummy_usb_serial_number, 0664, iSerial_show, store_dummy_usb_serial_number); 
static DEVICE_ATTR(usb_ac_cable_status, 0444, show_usb_ac_cable_status, NULL); 
static DEVICE_ATTR(ats, 0664, show_ats, store_ats); 
static DEVICE_ATTR(usb_disable, 0664,show_usb_disable_setting, store_usb_disable_setting); 
static DEVICE_ATTR(usb_cable_connect, 0444, show_usb_cable_connect, NULL); 
static DEVICE_ATTR(os_type, 0444, show_os_type, NULL); 
static DEVICE_ATTR(usb_modem_enable, S_IWUSR|S_IWGRP,NULL, store_usb_modem_enable_setting);
static DEVICE_ATTR(speed, 0444, show_speed, NULL); 
static DEVICE_ATTR(lock_speed, 0664, show_lock_speed, store_lock_speed); 
static DEVICE_ATTR(lock_host_speed, S_IWUSR, NULL, store_lock_host_speed); 
static DEVICE_ATTR(typec_dump_reg, 0440, show_typec_dump_reg, NULL); 
static DEVICE_ATTR(pd_cap, 0444, show_pd_cap, NULL); 
static DEVICE_ATTR(typec_fw_version, 0444, show_typec_fw_version, NULL);
static DEVICE_ATTR(typec_chip_version, 0444, show_typec_chip_version, NULL);
static DEVICE_ATTR(typec_fw_update_info, 0444, show_typec_fw_update_info, NULL);
static DEVICE_ATTR(typec_cable_info, 0444, show_typec_cable_info, NULL);


static __maybe_unused struct attribute *android_htc_usb_attributes[] = {
	&dev_attr_dummy_usb_serial_number.attr, 
	&dev_attr_usb_ac_cable_status.attr, 
	&dev_attr_usb_cable_connect.attr, 
	&dev_attr_ats.attr, 
	&dev_attr_usb_disable.attr, 
	&dev_attr_os_type.attr, 
	&dev_attr_usb_modem_enable.attr,
	&dev_attr_speed.attr, 
	&dev_attr_lock_speed.attr, 
	&dev_attr_lock_host_speed.attr, 
	&dev_attr_typec_dump_reg.attr, 
	&dev_attr_pd_cap.attr, 
	&dev_attr_typec_fw_version.attr,
	&dev_attr_typec_chip_version.attr,
	&dev_attr_typec_fw_update_info.attr,
	&dev_attr_typec_cable_info.attr,
	NULL
};

static  __maybe_unused const struct attribute_group android_usb_attr_group = {
	.attrs = android_htc_usb_attributes,
};

static void setup_vendor_info(struct android_dev *dev)
{
	if (sysfs_create_group(&dev->pdev->dev.kobj, &android_usb_attr_group))
		pr_err("%s: fail to create sysfs\n", __func__);
	
	if (sysfs_create_link(&platform_bus.kobj, &dev->pdev->dev.kobj, "android_usb"))
		pr_err("%s: fail to link android_usb to /sys/devices/platform/\n", __func__);
}
