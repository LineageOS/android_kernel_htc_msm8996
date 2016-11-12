/* include/linux/touch_fw_update.h
 *
 * Copyright (c)2014 HTC, Inc.
 *
 * Driver Version: 1.0.0
 * Release Date: Aug 28, 2014
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

#ifndef __CAPSENSOR_FWU_H
#define __CAPSENSOR_FWU_H

#include <linux/cdev.h>

struct capsensor_fwu_notifier {
	int (*fwupdate)(struct capsensor_fwu_notifier *notifier, struct firmware *fw);
	u32 flash_timeout;
	char fw_vendor[20];
	char fw_ver[20];
	struct data *fwu_data;
	u8 dev_id;
};

struct cdev_data {
	int size_count;
	size_t fw_size;
	unsigned char *buf;
	struct data *fwu_data;
};

struct data {
	int (*fwupdate)(struct capsensor_fwu_notifier *notifier, struct firmware *fw);
	u8 flash_status;
	u8 download_start;
	u8 update_bypass;
	char fw_vendor[20];
	char fw_ver[20];
	size_t fw_size;
	u32 flash_timeout;
	u32 flash_progress;
	struct firmware *fw;
	struct device dev;
	struct cdev fwu_cdev;
	struct cdev_data *fwu_cdev_data;
	struct capsensor_fwu_notifier *notifier;
	struct class *fwu_class;
	struct device *fwu_dev;
	u8 dev_id;
};

int register_cap_fw_update(struct capsensor_fwu_notifier *notifier);
void unregister_cap_fw_update(struct capsensor_fwu_notifier *notifier);
void capsensor_fw_update_progress(struct capsensor_fwu_notifier *notifier, int percentage);

#endif //__CAPSENSOR_FWU_H
