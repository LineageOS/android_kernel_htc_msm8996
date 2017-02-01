/* drivers/input/touchscreen/capsensor_fw_update.c
 *
 * Copyright (c)2015 HTC Corporation.
 *
 * Driver Version: 1.0.0.0
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
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <asm/byteorder.h>
//#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/input/capsensor_fw_update.h>

#define CAP_DEV_NAME "capsensor_fwu"
#define CAP_LOG_NAME "[cap][FWU]"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) CAP_LOG_NAME ": " fmt

#ifdef pr_info
#undef pr_info
#endif
#define pr_info(fmt, ...) printk(KERN_INFO pr_fmt(fmt) "\n", ##__VA_ARGS__)

#ifdef pr_err
#undef pr_err
#endif
#define pr_err(fmt, ...) printk(KERN_ERR CAP_LOG_NAME \
                       "[ERR]:(%s:%d): " fmt "\n", __func__, __LINE__, ##__VA_ARGS__)

#define CAP_FWU_IOCTL_CODE			(0x85)
#define FW_UPDATE_PROCCESS			_IO(CAP_FWU_IOCTL_CODE, 1)
#define FW_FILE_SIZE				_IOW(CAP_FWU_IOCTL_CODE, 2, uint32_t)
#define FW_FILE_REQUEST				_IO(CAP_FWU_IOCTL_CODE, 3)
#define FW_LOAD_DONE				_IO(CAP_FWU_IOCTL_CODE, 4)
#define FW_UPDATE_BYPASS			_IO(CAP_FWU_IOCTL_CODE, 5)

static u32 debug_mask = 0x00000000;
static int driver_probe_status = 0;

static int fw_update_process(struct data *fwu_data, struct firmware *fw);

static ssize_t debug_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%08X\n", debug_mask);
}

static ssize_t debug_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	if (kstrtou32(buf, 16, &debug_mask) != 0) {
		pr_err("bad parameter");
		return -EINVAL;
	}

	return count;
}

static ssize_t cap_vendor_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct data *fwu_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s", fwu_data->fw_vendor);
}

static ssize_t cap_fw_ver_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct data *fwu_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s", fwu_data->fw_ver);
}

static ssize_t fw_update_progress_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct data *fwu_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", fwu_data->flash_progress);
}

static ssize_t fw_update_timeout_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct data *fwu_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", fwu_data->flash_timeout);
}

static ssize_t fw_update_status_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct data *fwu_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", fwu_data->flash_status);
}

static ssize_t fw_update_status_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct data *fwu_data = dev_get_drvdata(dev);

	if (buf[0] == 0) {
		pr_info("%s: echo 0", __func__);
		fwu_data->download_start= 1;
		fwu_data->flash_status = 0;
	}

	return count;
}

static struct device_attribute dev_attr[] = {
	__ATTR(debug_level, (S_IWUSR|S_IRUGO), debug_show, debug_store),
	__ATTR(vendor, S_IRUGO, cap_vendor_show, NULL),
	__ATTR(fw_ver, S_IRUGO, cap_fw_ver_show, NULL),
	__ATTR(fw_update_status, (S_IWUSR|S_IRUGO), fw_update_status_show, fw_update_status_store),
	__ATTR(fw_update_timeout, S_IRUGO, fw_update_timeout_show, NULL),
	__ATTR(fw_update_progress, S_IRUGO, fw_update_progress_show, NULL),
};

static int capsensor_fwu_open(struct inode *inode, struct file *filp)
{
	struct data *fwu_data = container_of(inode->i_cdev, struct data, fwu_cdev);
	struct cdev_data *fwu_cdev_data = fwu_data->fwu_cdev_data;

	pr_info("%s", __func__);
	fwu_data->download_start= 1;
	fwu_data->flash_status = 1;
	fwu_data->update_bypass = 0;
	fwu_data->flash_progress = 0;
	fwu_cdev_data->size_count = 0;
	fwu_cdev_data->fw_size = 0;
	fwu_cdev_data->fwu_data = fwu_data;
	filp->private_data = fwu_cdev_data;
	return 0;
}

static int capsensor_fwu_release(struct inode *inode, struct file *filp)
{
	struct data *fwu_data = container_of(inode->i_cdev, struct data, fwu_cdev);
	struct cdev_data *fw_cdev = filp->private_data;
	unsigned char *buf;

	pr_info("%s", __func__);
	if (fw_cdev->buf != NULL) {
		pr_info("%s: free buf", __func__);
		buf = fw_cdev->buf;
		kfree(buf);
		fw_cdev->buf = NULL;
	}
	fw_cdev->size_count = 0;
	fw_cdev->fw_size = 0;
	if (fwu_data->update_bypass)
		fwu_data->flash_status = 3;
	else
		fwu_data->flash_status = 2;
	fwu_data->download_start= 0;
	fwu_data->flash_progress = 100;
	return 0;
}

static ssize_t capsensor_fwu_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	//struct cdev_data *fw_cdev = filp->private_data;
	pr_info("%s: %zu", __func__, count);
	return 0;
}

static ssize_t capsensor_fwu_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	struct cdev_data *fw_cdev = file->private_data;
	u16 *tmp;

	pr_info("%s: %zu", __func__, count);
	tmp = kzalloc(count, GFP_KERNEL);
	if (tmp == NULL) {
		pr_err("%s: allocate tmp failed\n", __func__);
		return -ENOMEM;
	}
	if(copy_from_user(tmp, buf, count)) {
		pr_err("%s: copy_from_user failed", __func__);
		kfree(tmp);
		return -ENOMEM;
	}

	memcpy(fw_cdev->buf+fw_cdev->size_count, tmp, count);
	fw_cdev->size_count += count;
	kfree(tmp);
	return 0;
}

static unsigned int capsensor_fwu_poll(struct file *filp, struct poll_table_struct *wait)
{
	return 0;
}

static long capsensor_fwu_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	int ret = 0;
	u32 data = 0;
	struct cdev_data *fw_cdev = filp->private_data;
	struct data *fwu_data = fw_cdev->fwu_data;
	unsigned char *buf;
	struct firmware *fw;

	pr_info("%s: cmd: %x", __func__, cmd);
	switch (cmd) {
		case FW_UPDATE_PROCCESS:
			pr_info("%s: FW_UPDATE_PROCCESS", __func__);
			fw = fwu_data->fw;
			if (fw == NULL) {
				   pr_err("%s, no fw data", __func__);
				   return -1;
			}
			ret = fw_update_process(fwu_data, fw);
			if (ret == 1) {
				   pr_info("%s: fw bypass", __func__);
				   fwu_data->update_bypass = 1;
			}
			pr_info("%s: free fw", __func__);
			kfree(fw);
			fwu_data->fw = NULL;
			break;
		case FW_FILE_SIZE:
			data = args;
			fwu_data->fw_size = data;
			fw_cdev->fw_size = data;
			pr_info("%s: FW_FILE_SIZE:%d", __func__, data);
			break;
		case FW_FILE_REQUEST:
			if (fw_cdev->fw_size) {
				if (fw_cdev->buf == NULL) {
					pr_info("%s: allocate buf", __func__);
					buf = kzalloc(
						fw_cdev->fw_size*sizeof(unsigned char),
						GFP_KERNEL);
					if (!buf) {
						pr_err("%s, allocate failed", __func__);
						return -1;
					}
					fw_cdev->buf = buf;
				}
			}
			pr_info("%s: FW_FILE_REQUEST", __func__);
			break;
		case FW_LOAD_DONE:
			pr_info("%s: FW_LOAD_DONE", __func__);
			if (fwu_data->fw != NULL) {
				pr_info("%s: free fw", __func__);
				fw = fwu_data->fw;
				kfree(fw);
			}
			pr_info("%s: allocate fw", __func__);
			fw = kzalloc(sizeof(struct firmware), GFP_KERNEL);
			if (fw == NULL) {
				pr_err("%s, no fw data", __func__);
				return -1;
			}
			fwu_data->fw = fw;
			fwu_data->fw->size = fw_cdev->fw_size;
			fwu_data->fw->data = fw_cdev->buf;
			break;
		case FW_UPDATE_BYPASS:
			pr_info("%s: FW_UPDATE_BYPASS", __func__);
			fwu_data->update_bypass = 1;
			break;
		default:
			break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long
capsensor_fwu_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return capsensor_fwu_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define capsensor_fwu_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int fw_update_process(struct data *fwu_data, struct firmware *fw)
{
	struct capsensor_fwu_notifier *notifier = fwu_data->notifier;
	pr_info("%s: enter", __func__);
	if (fwu_data->fwupdate)
		return fwu_data->fwupdate(notifier, fw);
	else
		return -1;
}

void capsensor_fw_update_progress(struct capsensor_fwu_notifier *notifier, int percentage)
{
	struct data *fwu_data = notifier->fwu_data;

	pr_info("%s: %d", __func__, percentage);
	if (percentage >= 100)
		fwu_data->flash_progress = 100;
	else if (percentage <= 0)
		fwu_data->flash_progress = 10;
	else
		fwu_data->flash_progress = percentage;
}
EXPORT_SYMBOL(capsensor_fw_update_progress);

static const struct file_operations capsensor_fwu_fops = {
	.owner  = THIS_MODULE,
	.read   = capsensor_fwu_read,
	.write  = capsensor_fwu_write,
	.unlocked_ioctl = capsensor_fwu_ioctl,
	.compat_ioctl = capsensor_fwu_compat_ioctl,
	.open   = capsensor_fwu_open,
	.release= capsensor_fwu_release,
	.poll   = capsensor_fwu_poll,
};

#define DEV_NUMBER 1
int register_cap_fw_update(struct capsensor_fwu_notifier *notifier)
{
	struct data *fwu_data;
	struct cdev_data *fwu_cdev_data;
	int ret = 0, attr_count = 0, major_num = 0;
	dev_t dev_no;
	char dev_name[16], buf[3];

	pr_info("%s: enter", __func__);

	fwu_data = kzalloc(sizeof(*fwu_data), GFP_KERNEL);
	if (!fwu_data) {
		pr_err("Fail to allocate fw update data memory");
		ret = -ENOMEM;
		goto err_device_init;
	}

	fwu_data->fwupdate = notifier->fwupdate;
	fwu_data->flash_timeout = notifier->flash_timeout;
	fwu_data->flash_status = 0;
	fwu_data->update_bypass = 0;
	fwu_data->flash_progress = 0;
	memset(fwu_data->fw_vendor, 0, sizeof(fwu_data->fw_vendor));
	memcpy(fwu_data->fw_vendor, notifier->fw_vendor, sizeof(notifier->fw_vendor));
	memcpy(fwu_data->fw_ver, notifier->fw_ver, sizeof(notifier->fw_ver));
	fwu_data->notifier = notifier;
	notifier->fwu_data = fwu_data;
	fwu_data->dev_id = notifier->dev_id;

	fwu_cdev_data = kzalloc(1*sizeof(struct cdev_data), GFP_KERNEL);
	if (fwu_cdev_data == NULL) {
		pr_err("Allocate cdev_data failed");
		ret = -ENOMEM;
		goto err_create_cdev_data;
	}

	if (major_num) {
		dev_no = MKDEV(major_num, DEV_NUMBER);
		ret = register_chrdev_region(dev_no, 1, CAP_DEV_NAME);
	} else {
		ret = alloc_chrdev_region(&dev_no, 0, 1, CAP_DEV_NAME);
		if (ret < 0) {
			pr_err("%s: Failed to allocate char device region\n", __func__);
			goto err_device_region;
		}

		major_num = MAJOR(dev_no);
		pr_info("%s: Major number of dev = %d\n",
					__func__, major_num);
	}

	cdev_init(&fwu_data->fwu_cdev, &capsensor_fwu_fops);

	ret = cdev_add(&fwu_data->fwu_cdev, dev_no, 1);
	if (ret < 0) {
		pr_err("%s: Failed to add char device\n", __func__);
		goto err_char_device;
	}
	fwu_data->fwu_cdev.owner = THIS_MODULE;
	fwu_data->fwu_cdev_data = fwu_cdev_data;

	fwu_data->fwu_class = class_create(THIS_MODULE, CAP_DEV_NAME);
	if (IS_ERR(fwu_data->fwu_class)) {
		pr_info("%s: create class fail\n", __func__);
		ret = PTR_ERR(fwu_data->fwu_class);
		goto err_create_class;
	}

	snprintf(dev_name, 16, "%s", CAP_DEV_NAME);
	if (fwu_data->dev_id != 0) {
		snprintf(buf, 3, "%d", fwu_data->dev_id);
		strlcat(dev_name, buf, sizeof(dev_name));
	}

	fwu_data->fwu_dev = device_create(fwu_data->fwu_class, NULL, dev_no,
						fwu_data, "%s", dev_name);
	if (unlikely(IS_ERR(fwu_data->fwu_dev))) {
		pr_info("%s: create dev fail\n", __func__);
		ret = PTR_ERR(fwu_data->fwu_dev);
		goto err_create_device;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(dev_attr); attr_count++) {
		if (sysfs_create_file(&fwu_data->fwu_dev->kobj, &dev_attr[attr_count].attr) < 0) {
			pr_err("failed to create sysfs file");
			ret = -1;
			goto err_create_sys_file;
		}
	}
	ret = sysfs_create_link(NULL, &fwu_data->fwu_dev->kobj, "android_cap_fwu");
	if (ret < 0) {
		pr_err("%s: subsystem_register failed\n", __func__);
		goto err_create_link_failed;
	}

	pr_info("%s: register success", __func__);
	return 0;

err_create_link_failed:
err_create_sys_file:
	for (attr_count--; attr_count>=0; attr_count--) {
			sysfs_remove_file(&fwu_data->fwu_dev->kobj, &dev_attr[attr_count].attr);
	}
	device_unregister(fwu_data->fwu_dev);
	fwu_data->fwu_dev = NULL;
err_create_device:
	class_destroy(fwu_data->fwu_class);
	fwu_data->fwu_class = NULL;
err_create_class:
err_char_device:
	cdev_del(&fwu_data->fwu_cdev);
	unregister_chrdev_region(dev_no, 1);
err_device_region:
	kfree(fwu_cdev_data);
err_create_cdev_data:
	driver_probe_status = 0;
	kfree(fwu_data);
err_device_init:
	return ret;
}
EXPORT_SYMBOL(register_cap_fw_update);

void unregister_cap_fw_update(struct capsensor_fwu_notifier *notifier)
{
	struct data *fwu_data = notifier->fwu_data;
	struct cdev_data *fwu_cdev_data = fwu_data->fwu_cdev_data;
	dev_t devno = fwu_data->fwu_cdev.dev;
	int attr_count = 0;

	pr_info("%s", __func__);

	fwu_data->fwupdate = NULL;
	fwu_data->flash_timeout = 0;
	fwu_data->flash_progress = 0;
	memset(fwu_data->fw_vendor, 0, sizeof(fwu_data->fw_vendor));
	snprintf(fwu_data->fw_vendor, sizeof(fwu_data->fw_vendor), "NULL");
	memset(fwu_data->fw_ver, 0, sizeof(fwu_data->fw_vendor));

	sysfs_remove_link(NULL, "android_cap_fwu");
	for (attr_count = 0; attr_count < ARRAY_SIZE(dev_attr); attr_count++) {
		sysfs_remove_file(&fwu_data->fwu_dev->kobj, &dev_attr[attr_count].attr);
	}
	device_unregister(fwu_data->fwu_dev);
	class_destroy(fwu_data->fwu_class);
	cdev_del(&fwu_data->fwu_cdev);
	unregister_chrdev_region(devno, 1);
	kfree(fwu_cdev_data);
	kfree(fwu_data);
	driver_probe_status = 0;
}
EXPORT_SYMBOL(unregister_cap_fw_update);

MODULE_AUTHOR("HTC");
MODULE_DESCRIPTION("Capsensor Firmware update Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0.0");
