/* drivers/input/touchscreen/crypress_cap.c
 *
 * Copyright (C) 2015 HTC Corporation.
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
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>
#include <linux/ctype.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_FB
#include <linux/fb.h>
#endif
#include <linux/input/cypress_cap.h>
#ifdef CONFIG_TOUCHSCREEN_CAPSENSOR_FW_UPDATE
#include <linux/input/capsensor_fw_update.h>
#endif
#ifdef CONFIG_AK8789_HALLSENSOR
#include <linux/hall_sensor.h>
#endif

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "[cap] " fmt
#define pr_err_fmt(fmt) "[cap][ERR]:" fmt

#ifdef pr_info
#undef pr_info
#endif
#define pr_info(fmt, ...) printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)

#ifdef pr_notice
#undef pr_notice
#endif
#define pr_notice(fmt, ...) printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)

#ifdef pr_err
#undef pr_err
#endif
#define pr_err(fmt, ...) printk(KERN_ERR pr_err_fmt(fmt), ##__VA_ARGS__)

#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug(fmt, ...) if(debug_mask & BIT(0)) printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)

#define CYPRESS_I2C_RETRY_TIMES		(10)
#define CYPRESS_KEYLOCKTIME			(1500)
#define CYPRESS_KEYLOCKDELAYTIME	(1000)
#define CYPRESS_KEYLOCKRESET		(6)

#ifdef CONFIG_TOUCHSCREEN_CAPSENSOR_FW_UPDATE
#define CAP_VENDOR "CYPRESS"
#define FW_FLASH_TIMEOUT 120
static void cypress_fwu_progress(struct capsensor_fwu_notifier *notifier, int percentage);
#endif

static LIST_HEAD(cs_list);
static LIST_HEAD(cs_fw_list);

#ifdef SUPPORT_CYPRESS_FW_UPDATE
struct cypress_fw_update_data {
	struct list_head list;
	struct i2c_client *client;
	struct cypress_bootloader_header header;
	unsigned long validRows[MAX_FLASH_ARRAYS];
	struct cypress_cap_data *cs;
	int id;
#ifdef CONFIG_TOUCHSCREEN_CAPSENSOR_FW_UPDATE
	struct capsensor_fwu_notifier cypress_cs_notifier;
#endif
};
#endif

struct cypress_cap_data {
	struct list_head list;
	struct i2c_client *client;
	struct input_dev *input_dev;
#ifdef CONFIG_FB
	struct notifier_block fb_notifier;
#endif
#ifdef CONFIG_AK8789_HALLSENSOR
	struct notifier_block hallsensor_handler;
	uint8_t cs_cover_mode;
#endif
	struct hrtimer timer;
	uint16_t version;
	struct info info;
	uint16_t gpio_irq;
	uint16_t gpio_ind;
	uint8_t cap_num;
	uint8_t raw_num;
	uint8_t status;
	uint8_t irq_enabled;
	int *keycode;
	int (*power)(int on);
	int (*reset)(struct i2c_client *client);
	uint8_t cs_bootloader_mode;
	struct cypress_fw_update_data *fw_update_data;
	int id;
	int disable_key;
	struct workqueue_struct *wq_raw;
	struct delayed_work work_raw;
	int reset_cnt;          
	int update_feature;
	struct wake_lock update_wake_lock;
	bool stay_awake;
	bool suspended;
	uint8_t cap_glove_mode_status;
#ifdef CONFIG_AK8789_HALLSENSOR
	uint8_t cap_hall_s_pole_status;
#endif
};

extern char *htc_get_bootmode(void);
static int cypress_get_version(struct cypress_cap_data *cs);
static int cypress_get_project_id(struct cypress_cap_data *cs);
static int cypress_init_sensor(struct cypress_cap_data *cs);
static irqreturn_t cypress_cap_irq_handler(int, void *);
static void cancel_print_raw(struct cypress_cap_data *cs);
#ifdef SUPPORT_CYPRESS_FW_UPDATE
static int cypress_enter_bootloader(struct cypress_cap_data *cs);
static int update_firmware(struct cypress_fw_update_data *fw_update_data,
	struct cypress_cap_data *cs, char *buf, int len);
#endif

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data);
static void cypress_cap_early_suspend(struct device *dev);
static void cypress_cap_late_resume(struct device *dev);
#endif
#ifdef CONFIG_AK8789_HALLSENSOR
static int hallsensor_handler_callback(struct notifier_block *self,
			unsigned long event, void *data);
#endif
enum cypress_sensitivity {
	CS_SENS_HIGH	= 0x01,
	CS_SENS_DEFAULT	= 0x02
};

static ssize_t cypress_cap_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t cypress_cap_gpio_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t cypress_cap_reset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t cypress_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t cypress_reg_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count);
static ssize_t cypress_debug_show(struct device *dev, struct device_attribute *attr,
	char *buf);
static ssize_t cypress_debug_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count);
static ssize_t cypress_disable_key_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t cypress_disable_key_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t cypress_inform_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t cypress_glove_setting_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t cypress_glove_setting_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count);
#ifdef CONFIG_AK8789_HALLSENSOR
static ssize_t cypress_cover_show(struct device *dev, struct device_attribute *attr, char *buf);
#endif
#ifdef SUPPORT_CYPRESS_FW_UPDATE
static ssize_t cypress_fw_update_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
#endif

static uint32_t debug_mask = BIT(1);

static struct device_attribute attrs[] = {
	__ATTR(vendor, S_IRUGO,
			cypress_cap_vendor_show,
			NULL),
	__ATTR(gpio, S_IRUGO,
			cypress_cap_gpio_show,
			NULL),
	__ATTR(reset, S_IWUSR,
			NULL,
			cypress_cap_reset_store),
	__ATTR(reg, (S_IRUGO|S_IWUSR),
			cypress_reg_show,
			cypress_reg_store),
	__ATTR(debug, (S_IRUGO|S_IWUSR),
			cypress_debug_show,
			cypress_debug_store),
	__ATTR(disable_key, (S_IRUGO|S_IWUSR),
			cypress_disable_key_show,
			cypress_disable_key_store),
	__ATTR(inform, S_IRUGO,
			cypress_inform_show,
			NULL),
	__ATTR(glove_setting, (S_IRUGO|S_IWUSR),
			cypress_glove_setting_show,
			cypress_glove_setting_store),
#ifdef CONFIG_AK8789_HALLSENSOR
	__ATTR(cover, S_IRUGO,
			cypress_cover_show,
			NULL),
#endif
#ifdef SUPPORT_CYPRESS_FW_UPDATE
	__ATTR(fw_update, S_IWUSR,
			NULL,
			cypress_fw_update_store),
#endif
};

#ifdef SUPPORT_CYPRESS_FW_UPDATE
static int i2c_tx_bytes(struct i2c_client *client, uint16_t len, uint8_t *buf)
{
	int ret;
	uint8_t retry;

	for (retry = 0; retry < CYPRESS_I2C_RETRY_TIMES; retry++) {
		ret = i2c_master_send(client, (char *)buf, (int)len);
		if (ret > 0) {
			if (ret == len)
				return ret;
			ret = -EIO;
			break;
		}
		mdelay(10);
	}

	pr_err("I2C TX fail (%d)\n", ret);
	return ret;
}

static int i2c_rx_bytes(struct i2c_client *client, uint16_t len, uint8_t *buf)
{
	int ret;
	uint8_t retry;

	for (retry = 0; retry < CYPRESS_I2C_RETRY_TIMES; retry++) {
		ret = i2c_master_recv(client, (char *)buf, (int)len);
		if (ret > 0) {
			if (ret == len)
				return ret;
			ret = -EIO;
			break;
		}
		mdelay(10);
	}

	pr_err("I2C RX fail (%d)\n", ret);
	return ret;
}

static int check_fw_version(const struct firmware *fw, int *tagLen, int version)
{
	char tag[40], fw_ver[5];
	int i = 0;

	if (fw->data[0] == 'C' && fw->data[1] == 'S') {
		while ((tag[i] = fw->data[i]) != '\n')
			i++;
		tag[i] = '\0';
		*tagLen = i+1;
		pr_info("tag=%s\n", tag);
		pr_info("version=%02X\n", version);
		if (version <= 0) {
			pr_info("Need Update\n");
			return 1;
		}

		scnprintf(fw_ver, 5,"%X", version);
		if (strstr(tag, fw_ver) != NULL) {
			pr_info("Update Bypass\n");
			return 0; 
		}
	}

	pr_info("Need Update\n");
	return 1;
}

static ssize_t cypress_fw_update_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);
	struct cypress_fw_update_data *fw_update_data = cs->fw_update_data;
	struct i2c_client *client = fw_update_data->client;
	int ret, tag_len = 0;
	const struct firmware *fw;

	pr_info("%s\n", __func__);
	cs->stay_awake = true;
	ret = request_firmware(&fw, "cs_CY8C.img", &client->dev);
	if (ret != 0) {
		pr_err("No Firmware file\n");
		return -1;
	}
	pr_info("%s: request FW done\n", __func__);
	if (debug_mask & BIT(1)) {
		cancel_print_raw(cs);
	}
	if (cs->irq_enabled) {
		disable_irq(cs->client->irq);
		cs->irq_enabled = 0;
	}
	ret = check_fw_version(fw, &tag_len, cs->info.version);
	if (ret == 0)
		goto exit;

	ret = update_firmware(fw_update_data, cs, (char *)fw->data + tag_len, fw->size - tag_len);
	cs->reset(cs->client);
exit:
	if (!cs->irq_enabled) {
		enable_irq(cs->client->irq);
		cs->irq_enabled = 1;
	}

	if (cypress_init_sensor(cs) < 0) {
		pr_err("Get chip info Err\n");
	}

	release_firmware(fw);
	cs->stay_awake = false;

	return count;
}

static unsigned short cypress_ComputeChecksum(unsigned char* buf, unsigned long size, uint8_t type)
{
	unsigned short crc = 0xffff;
	unsigned short tmp;
	unsigned short sum = 0;
	int i;

	pr_debug("%s\n", __func__);
	if (type == CRC_CHECKSUM) {
		if (size == 0)
			return (~crc);

		do {
			for (i = 0, tmp = 0x00ff & *buf++; i < 8; i++, tmp >>= 1) {
				if ((crc & 0x0001) ^ (tmp & 0x0001))
					crc = (crc >> 1) ^ 0x8408;
				else
					crc >>= 1;
			}
		} while (--size);

		crc = ~crc;
		tmp = crc;
		crc = (crc << 8) | (tmp >> 8 & 0xFF);
		return crc;
	}
	else { 
		while (size-- > 0)
			sum += *buf++;

		return (1 + ~sum);
	}
}

static int cypress_send_command(struct cypress_fw_update_data *fw_update_data,
		uint8_t command, uint16_t size, uint8_t *data)
{
	struct i2c_client *client = fw_update_data->client;
	int cmdSize = size + BASE_CMD_SIZE;
	int checksum = 0, ret, i;
	uint8_t cmdBuf[cmdSize];

	pr_debug("%s\n", __func__);
	cmdBuf[0] = CMD_START;
	cmdBuf[1] = command;
	cmdBuf[2] = (size & 0xFF);
	cmdBuf[3] = ((size >> 8) & 0xFF);
	for (i = 0; i < size; i++)
		cmdBuf[i + 4] = data[i];
	checksum = cypress_ComputeChecksum(cmdBuf, cmdSize-3, fw_update_data->header.packetChkSumType);
	cmdBuf[cmdSize-3] = (checksum & 0xFF);
	cmdBuf[cmdSize-2] = ((checksum >> 8) & 0xFF);
	cmdBuf[cmdSize-1] = CMD_STOP;

	ret = i2c_tx_bytes(client, cmdSize, cmdBuf);
	if (ret <= 0) {
		pr_err("%s: Send command error, command:%x, ret = %d\n", __func__, command, ret);
		return -1;
	}

	return 0;
}

static int cypress_receive_report(struct cypress_fw_update_data *fw_update_data,
		uint8_t *data, uint16_t size)
{
	struct i2c_client *client = fw_update_data->client;
	int reportSize = size + BASE_CMD_SIZE;
	int ret, i;
	uint8_t reportBuf[reportSize];

	pr_debug("%s\n", __func__);
	for (i = 0; i < CYPRESS_I2C_RETRY_TIMES; i++) {
		ret = i2c_rx_bytes(client, reportSize, reportBuf);
		if (ret <= 0) {
			pr_err("%s: Receive report error, ret = %d\n", __func__, ret);
			return -1;
		}
		if ((reportBuf[0] != CMD_START) | (reportBuf[reportSize-1] != CMD_STOP)) {
			pr_err("%s: Packet error\n", __func__);
			mdelay(1);
			continue;
			
		}
		if (reportBuf[1] != CYRET_SUCCESS) {
			pr_err("%s: status error = %d\n", __func__, reportBuf[1]);
			mdelay(1);
			continue;
			
		}
		break;
	}

	if (i == CYPRESS_I2C_RETRY_TIMES) {
		pr_err("%s: Retry Max times\n", __func__);
		return -reportBuf[1];
	}

	memcpy(data, reportBuf, reportSize * sizeof(uint8_t));

	return 0;
}

static int cypress_parse_fw_line(char *buf, int len)
{
	int i, line = 1;

	
	for (i = 0; i < len -1 ; i++) {
		if (buf[i] == ':')
			line++;
	}
	

	return line;
}

static int cypress_parse_fw_line_len(char *buf, int len)
{
	int i;

	
	for (i = 0; i < len -1 ; i++) {
		if (buf[i] == '\n')
			break;
	}
	

	return i+1;
}

static int cypress_parse_firmware(char *buf, char *fw, int len)
{
	int i, j = 0;

	pr_debug("%s\n", __func__);
	for (i = j = 0; i < len - 1; ++i) {
		char buffer[3];
		memcpy(buffer, buf + i, 2);
		if (!isxdigit(buffer[0])) {
			if (buffer[0] == ':')
				continue;
			else
				break;
		}
		buffer[2] = '\0';
		if (sscanf(buffer, "%2" SCNx8, fw + j) == 1) {
			++j;
			++i;
		}
	}

	return j;
}

static int cypress_parse_header(struct cypress_fw_update_data *fw_update_data,
	char *fw, char *buf, int len, int *used_len)
{
	int header_len = 0;
	int buf_len = 0;

	pr_debug("%s\n", __func__);
	buf_len = cypress_parse_fw_line_len(buf, len);
	header_len = cypress_parse_firmware(buf, fw, buf_len);

	if (header_len >= CS_LENGTH_CHECKSUM) {
		fw_update_data->header.packetChkSumType = fw[5];
	}
	if (header_len >= CS_LENGTH_ID) {
		fw_update_data->header.siliconID = (fw[0] << 24) | (fw[1] << 16) | (fw[2] << 8) | fw[3];
		fw_update_data->header.siliconRev = fw[4];
	}
	else
		return -1;

	pr_info("%s: siliconID:%lx, siliconRev:%x, packetChkSumType:%x\n",
			__func__, fw_update_data->header.siliconID,
			fw_update_data->header.siliconRev, fw_update_data->header.packetChkSumType);

	*used_len = buf_len;
	return header_len;
}

static int cypress_validate_row(struct cypress_fw_update_data *fw_update_data,
		unsigned char arrayId, unsigned short rowNum)
{
	int ret;
	int size = 4;
	uint8_t buf[size + BASE_CMD_SIZE];
	unsigned short minRow, maxRow;

	pr_debug("%s\n", __func__);

	if (arrayId >= MAX_FLASH_ARRAYS) {
		pr_err("%s: Array error \n", __func__);
		return -1;
	}

	if (fw_update_data->validRows[arrayId] == NO_FLASH_ARRAY_DATA) {
		ret = cypress_send_command(fw_update_data, CMD_GET_FLASH_SIZE, 1, &arrayId);
		if (ret < 0) {
			pr_err("%s: Send verify row error", __func__);
			return -1;
		}
		mdelay(25);
		ret = cypress_receive_report(fw_update_data, buf, size);
		if (ret < 0) {
			pr_err("%s: Receive verify row error", __func__);
			return ret;
		}
		minRow = (buf[5] << 8) | buf[4];
		maxRow = (buf[7] << 8) | buf[6];
		fw_update_data->validRows[arrayId] = (minRow << 16) + maxRow;
	}

	minRow = (fw_update_data->validRows[arrayId] >> 16) & 0xFFFF;
	maxRow = fw_update_data->validRows[arrayId] & 0xFFFF;
	if ((rowNum < minRow) || (rowNum > maxRow))
		return -1;

	return 0;
}

static int cypress_verify_row(struct cypress_fw_update_data *fw_update_data,
		unsigned char arrayId, unsigned short rowNum)
{
	int ret;
	int size = 1, cmd_size = 3;
	uint8_t buf[size + BASE_CMD_SIZE];
	uint8_t cmd_buf[cmd_size];
	unsigned char checksum;

	pr_debug("%s\n", __func__);
	ret = cypress_validate_row(fw_update_data, arrayId, rowNum);
	if (ret < 0) {
		pr_err("%s: validate error\n", __func__);
		return -1;
	}

	cmd_buf[0] = arrayId;
	cmd_buf[1] = (rowNum & 0xFF);
	cmd_buf[2] = ((rowNum >> 8) & 0xFF);

	ret = cypress_send_command(fw_update_data, CMD_VERIFY_ROW, cmd_size, cmd_buf);
	if (ret < 0) {
		pr_err("%s: Send Verify row error", __func__);
		return -1;
	}
	mdelay(25);
	ret = cypress_receive_report(fw_update_data, buf, size);
	if (ret < 0) {
		pr_err("%s: Receive Verify row error", __func__);
		return ret;
	}

	checksum = buf[4];
	return checksum;
}

static int cypress_program_row(struct cypress_fw_update_data *fw_update_data,
		unsigned char arrayId, unsigned short rowNum, unsigned char *row_data, unsigned short len)
{
	int ret;
	int size = 0, offset = 0, buf_len = 0, cmd_size = 3;
	uint8_t buf[size + BASE_CMD_SIZE];
	uint8_t *cmd_buf;

	pr_debug("%s\n", __func__);

	while ((len - offset + TRANSFER_HEADER_SIZE) > MAX_TRANDSFER_SIZE) {
		buf_len = MAX_TRANDSFER_SIZE - TRANSFER_HEADER_SIZE;
		ret = cypress_send_command(fw_update_data, CMD_SEND_DATA, buf_len, &row_data[offset]);
		if (ret < 0) {
			pr_err("%s: Send row data error", __func__);
			return -1;
		}
		mdelay(25);
		ret = cypress_receive_report(fw_update_data, buf, size);
		if (ret < 0) {
			pr_err("%s: Receive row data ack error", __func__);
			return ret;
		}

		offset += buf_len;
	}

	buf_len = len - offset;
	cmd_size += buf_len;
	cmd_buf = kzalloc((buf_len * sizeof(unsigned char)), GFP_KERNEL);
	if (cmd_buf == NULL) {
		pr_err("allocate cmd_buf failed\n");
		return -ENOMEM;
	}
	cmd_buf[0] = arrayId;
	cmd_buf[1] = (rowNum & 0xFF);
	cmd_buf[2] = ((rowNum >> 8) & 0xFF);
	memcpy(&cmd_buf[3], &row_data[offset], cmd_size);

	ret = cypress_send_command(fw_update_data, CMD_PROGRAM_ROW, cmd_size, cmd_buf);
	if (ret < 0) {
		pr_err("%s: Send Program row error", __func__);
		kfree(cmd_buf);
		return -1;
	}
	kfree(cmd_buf);
	mdelay(25);
	ret = cypress_receive_report(fw_update_data, buf, size);
	if (ret < 0) {
		pr_err("%s: Receive Program row error", __func__);
		return ret;
	}

	return 0;
}

static int cypress_flash_row(struct cypress_fw_update_data *fw_update_data,
		char *fw, char *buf, int len, int header_len, int used_len, int line)
{
	int total_len = len - used_len;
	int row_len = 0, buf_len = 0;
	int i, ret;
	char *buffer = buf + used_len;
	char *p_fw = fw + header_len;
	unsigned char arrayId, checksum;
	unsigned short rowNum, size;
	unsigned char *row_data;

	pr_debug("%s\n", __func__);
	
	for (i = 1; i < line; i++) {
		row_len = cypress_parse_fw_line_len(buffer, total_len);
		buf_len = cypress_parse_firmware(buffer, p_fw, row_len);

		arrayId = p_fw[0];
		rowNum = (p_fw[1] << 8) | (p_fw[2]);
		size = (p_fw[3] << 8) | (p_fw[4]);
		checksum = p_fw[buf_len - 1];
		row_data = kzalloc((size * sizeof(unsigned char)), GFP_KERNEL);
		if (row_data == NULL) {
			pr_err("%s: allocate row_data failed, line = %d\n", __func__, line);
			return -ENOMEM;
		}

		memcpy(row_data, &p_fw[5], size * sizeof(unsigned char));
		ret = cypress_validate_row(fw_update_data, arrayId, rowNum);
		if (ret < 0) {
			pr_err("%s: validate error, line = %d\n", __func__, line);
			goto err_cypress_flash_row;
		}

		ret = cypress_program_row(fw_update_data, arrayId, rowNum, row_data, size);
		if (ret < 0) {
			pr_err("%s: program firmware error, line = %d\n", __func__, line);
			goto err_cypress_flash_row;
		}

		mdelay(25);
		ret = cypress_verify_row(fw_update_data, arrayId, rowNum);
		if (ret < 0) {
			pr_err("%s: verify_row error, line = %d\n", __func__, line);
			goto err_cypress_flash_row;
		}

		p_fw = p_fw + buf_len;
		buffer = buffer + row_len;
		total_len -= row_len;

		kfree(row_data);
#ifdef CONFIG_TOUCHSCREEN_CAPSENSOR_FW_UPDATE
		if ((i % ((line + 4)/4)) == 0)
			cypress_fwu_progress(&fw_update_data->cypress_cs_notifier, 10+(90*i/line));
#endif
	}

	return 0;

err_cypress_flash_row:
	kfree(row_data);
	return -1;
}

static int cypress_start_bootloader(struct cypress_fw_update_data *fw_update_data)
{
	int ret, i;
	int size = 8;
	uint8_t buf[size + BASE_CMD_SIZE];
	struct cypress_bootloader_header header;
	unsigned long blVersion;

	pr_info("%s\n", __func__);

	for (i = 0; i < MAX_FLASH_ARRAYS; i++) {
		fw_update_data->validRows[i] = NO_FLASH_ARRAY_DATA;
	}

	ret = cypress_send_command(fw_update_data, CMD_ENTER_BOOTLOADER, 0, NULL);
	if (ret < 0) {
		pr_err("%s: Send Enter Bootloader error\n", __func__);
		return -1;
	}
	mdelay(25);
	ret = cypress_receive_report(fw_update_data, buf, size);
	if (ret < 0) {
		pr_err("%s: Receive Enter Bootloader error", __func__);
		return ret;
	}
	header.siliconID = (buf[7] << 24) | (buf[6] << 16) | (buf[5] << 8) | buf[4];
	header.siliconRev = buf[8];
	blVersion = (buf[11] << 16) | (buf[10] << 8) | buf[9];

	pr_info("%s: siliconID:%lx, siliconRev:%x, blVersion:%lx\n",
			__func__, header.siliconID, header.siliconRev, blVersion);

	return 0;
}

static int cypress_end_bootloader(struct cypress_fw_update_data *fw_update_data)
{
	int ret;
	uint8_t reset = 0x00;

	pr_info("%s\n", __func__);
	ret = cypress_send_command(fw_update_data, CMD_EXIT_BOOTLOADER, 1, &reset);
	if (ret < 0) {
		pr_err("%s: Send Exit Bootloader error", __func__);
		return -1;
	}
	return 0;
}

static int update_firmware(struct cypress_fw_update_data *fw_update_data,
	struct cypress_cap_data *cs, char *buf, int len)
{
	int ret = 0;
	uint8_t *fw;
	unsigned int line, header_len, used_len;

	pr_info("%s: bootloader mode=%d\n", __func__, cs->cs_bootloader_mode);
	if (cs->cs_bootloader_mode == 0) {
		cypress_enter_bootloader(cs);
		cs->cs_bootloader_mode = 1;
		mdelay(300);
	}
#ifdef CONFIG_TOUCHSCREEN_CAPSENSOR_FW_UPDATE
	cypress_fwu_progress(&fw_update_data->cypress_cs_notifier, 0);
#endif
	fw = kzalloc(len/2, GFP_KERNEL);
	if (fw == NULL) {
		pr_err("allocate fw failed\n");
		ret = -ENOMEM;
		goto exit;
	}
	line = cypress_parse_fw_line(buf, len);
	header_len = cypress_parse_header(fw_update_data, fw, buf, len, &used_len);
	ret = cypress_start_bootloader(fw_update_data);
	if (ret < 0) {
		pr_err("Start Bootloader failed\n");
		goto exit;
	}
#ifdef CONFIG_TOUCHSCREEN_CAPSENSOR_FW_UPDATE
	cypress_fwu_progress(&fw_update_data->cypress_cs_notifier, 10);
#endif
	ret = cypress_flash_row(fw_update_data,
		fw, buf, len, header_len, used_len, line);
	if (ret < 0) {
		pr_err("Parse row failed\n");
		goto exit;
	}

exit:
#ifdef CONFIG_TOUCHSCREEN_CAPSENSOR_FW_UPDATE
	cypress_fwu_progress(&fw_update_data->cypress_cs_notifier, 99);
#endif
	cypress_end_bootloader(fw_update_data);
	cs->cs_bootloader_mode = 0;
	if (fw)
		kfree(fw);
	pr_info("%s finished, ret = %d\n", __func__, ret);

	return ret;
}

static int cypress_fw_update_get_data(struct cypress_cap_data *cs)
{
	struct cypress_fw_update_data *fw_update_data;

	pr_info("%s\n", __func__);
	list_for_each_entry(fw_update_data, &cs_fw_list, list) {
		if (cs->id == fw_update_data->id) {
			cs->fw_update_data = fw_update_data;
			fw_update_data->cs = cs;
			pr_info("%s: match\n", __func__);
			return 0;
		}
	}
	return -1;
}

#ifdef CONFIG_TOUCHSCREEN_CAPSENSOR_FW_UPDATE
static void cypress_fwu_progress(struct capsensor_fwu_notifier *notifier, int percentage)
{
	capsensor_fw_update_progress(notifier, percentage);
}

int cypress_capsensor_fw_update(struct capsensor_fwu_notifier *notifier, struct firmware *fw)
{
	struct cypress_fw_update_data *fw_update_data
			= container_of(notifier, struct cypress_fw_update_data, cypress_cs_notifier);
	struct cypress_cap_data *cs = fw_update_data->cs;
	int i, ret, tag_len = 0;

	if (cs == NULL) {
		pr_info("%s: not probe\n", __func__);
		return 1;
	}

	pr_info("%s\n", __func__);

	cs->stay_awake = true;
	cypress_fwu_progress(notifier, 0);
	if (debug_mask & BIT(1)) {
		cancel_print_raw(cs);
	}
	if (cs->irq_enabled) {
		disable_irq(cs->client->irq);
		cs->irq_enabled = 0;
	}
	ret = check_fw_version(fw, &tag_len, cs->info.version);
	if (ret == 0) {
		ret = 1;
		goto exit;
	}

	for (i = 0; i < CYPRESS_I2C_RETRY_TIMES; i++) {
		ret = update_firmware(fw_update_data, cs, (char *)fw->data + tag_len, fw->size - tag_len);
		cs->reset(cs->client);
		if (ret == 0)
			break;
	}
	if (i == CYPRESS_I2C_RETRY_TIMES)
		pr_err("retry 3 times. FW update failed\n");

exit:
	if (!cs->irq_enabled) {
		enable_irq(cs->client->irq);
		cs->irq_enabled = 1;
	}
	if (cypress_init_sensor(cs) < 0) {
		pr_err("Get chip info Err\n");
	}

	cypress_fwu_progress(notifier, 100);
	cs->stay_awake = false;

	return ret;
}

int register_cypress_cs_fw_update(struct cypress_fw_update_data *fw_update_data)
{
	fw_update_data->cypress_cs_notifier.fwupdate = cypress_capsensor_fw_update;
	fw_update_data->cypress_cs_notifier.flash_timeout = FW_FLASH_TIMEOUT;
	fw_update_data->cypress_cs_notifier.dev_id = fw_update_data->id;
	scnprintf(fw_update_data->cypress_cs_notifier.fw_vendor,
			sizeof(fw_update_data->cypress_cs_notifier.fw_vendor), "%s", CAP_VENDOR);
	return register_cap_fw_update(&fw_update_data->cypress_cs_notifier);
}
#endif
static int cypress_fw_update_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct cypress_fw_update_data *fw_update_data;
	struct device_node *dt;
	int cfw_id;

	pr_info("%s\n", __func__);
	fw_update_data = kzalloc(sizeof(struct cypress_fw_update_data), GFP_KERNEL);
	if (fw_update_data == NULL) {
		pr_err("allocate fw_update_data failed\n");
		return -ENOMEM;
	}
	fw_update_data->client = client;
	i2c_set_clientdata(client, fw_update_data);
	list_add(&fw_update_data->list, &cs_fw_list);
	if (client->dev.of_node) {
		dt = client->dev.of_node;
		cfw_id = of_alias_get_id(dt, "capfw");
		if (cfw_id < 0)
			cfw_id = 0;
		fw_update_data->id = cfw_id;
	}

#ifdef CONFIG_TOUCHSCREEN_CAPSENSOR_FW_UPDATE
	register_cypress_cs_fw_update(fw_update_data);
#endif

	return 0;
}

static void cypress_fw_update_free_data(struct cypress_fw_update_data *fw_update_data)
{
	fw_update_data->cs = NULL;
}

static void cypress_fw_update_deinit(struct cypress_fw_update_data *fw_update_data)
{
#ifdef CONFIG_TOUCHSCREEN_CAPSENSOR_FW_UPDATE
	unregister_cap_fw_update(&fw_update_data->cypress_cs_notifier);
#endif

	list_del(&fw_update_data->list);
	kfree(fw_update_data);
}

static int cypress_fw_update_remove(struct i2c_client *client)
{
	struct cypress_fw_update_data *fw_update_data = i2c_get_clientdata(client);

	if (fw_update_data != NULL)
		cypress_fw_update_deinit(fw_update_data);

	return 0;
}

static struct of_device_id cypress_fw_update_match_table[] = {
	{ .compatible = "cypress_fw_update",},
	{ },
};

static const struct i2c_device_id cypress_fw_update_id[] = {
	{ CYPRESS_FW_UPDATE_NAME, 0 },
};

static struct i2c_driver cypress_fw_update_driver = {
	.probe		= cypress_fw_update_probe,
	.remove		= cypress_fw_update_remove,
	.id_table	= cypress_fw_update_id,

	.driver	= {
		.owner  = THIS_MODULE,
		.name = CYPRESS_FW_UPDATE_NAME,
		.of_match_table = cypress_fw_update_match_table,
	},
};
#endif

static char *keycode_check(int keycode)
{
	switch (keycode) {
		case KEY_HOME:
			return "HOME";
		case KEY_BACK:
			return "BACK";
		case KEY_APP_SWITCH:
			return "APP_SWITCH";
		case KEY_APPSELECT:
			return "APPSELECT";
		case KEY_MENU:
			return "MENU";
		case KEY_SEARCH:
			return "SEARCH";
		default:
			return "RESERVED";
	}
}

static int i2c_cypress_read(struct i2c_client *client, uint8_t addr, uint8_t *data, uint8_t length)
{
	int retry;
	struct cypress_cap_data *cs = i2c_get_clientdata(client);

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	gpio_direction_output(cs->gpio_ind, 0);
	for (retry = 0; retry < CYPRESS_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		mdelay(10);
	}
	gpio_direction_output(cs->gpio_ind, 1);

	if (retry == CYPRESS_I2C_RETRY_TIMES) {
		pr_err("i2c_read_block retry over %d\n",
			CYPRESS_I2C_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_cypress_write(struct i2c_client *client, uint8_t addr, uint8_t *data, uint8_t length)
{
	int retry, loop_i;
	uint8_t buf[length + 1];
	struct cypress_cap_data *cs = i2c_get_clientdata(client);

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = addr;
	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i + 1] = data[loop_i];

	gpio_direction_output(cs->gpio_ind, 0);
	for (retry = 0; retry < CYPRESS_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}
	gpio_direction_output(cs->gpio_ind, 1);

	if (retry == CYPRESS_I2C_RETRY_TIMES) {
		pr_err("i2c_write_block retry over %d\n",
			CYPRESS_I2C_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

int i2c_cypress_write_byte_data(struct i2c_client *client, uint8_t addr, uint8_t value)
{
	return i2c_cypress_write(client, addr, &value, 1);
}

static ssize_t cypress_cap_reset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);
	cs->reset(cs->client);
	return count;
}

static ssize_t cypress_cap_vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int pid = 0, version = 0, ret = 0;
	struct cypress_cap_data *cs = dev_get_drvdata(dev);

	version = cypress_get_version(cs);
	if (version < 0) {
		pr_err("i2c Read version Err\n");
		scnprintf(buf, PAGE_SIZE, "%s_ERR\n", CYPRESS_CS_NAME);
	} else {
		pid = cypress_get_project_id(cs);
		if (pid < 0) {
			pr_err("i2c Read pid Err\n");
			scnprintf(buf, PAGE_SIZE, "%s_V%x\n", CYPRESS_CS_NAME, version);
		} else
			scnprintf(buf, PAGE_SIZE, "%s_V%x-PID:%x\n", CYPRESS_CS_NAME, version, pid);
	}
	ret += strlen(buf)+1;

	return ret;
}

static ssize_t cypress_cap_gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cypress_cap_data *cs = dev_get_drvdata(dev);

	ret = gpio_get_value(cs->gpio_irq);
	pr_info("GPIO_CS_INT_N=%d\n", cs->gpio_irq);
	scnprintf(buf, PAGE_SIZE, "GPIO_CS_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;

	return ret;
}

static int cypress_print_reg(struct cypress_cap_data *cs, char *buf)
{
	int ret, i, p = 0;
	uint8_t data = 0;

	pr_info("%s\n", __func__);
	for (i = 0; i < CS_COMMON_REGISTER; i++) {
		ret = i2c_cypress_read(cs->client, i, &data, 1);
		if (ret < 0)
			pr_info("%s: err\n", __func__);
		p += scnprintf(buf+p, PAGE_SIZE-p, "addr: 0x%02X, val: 0x%02X\n", i, data);
		pr_info("addr: 0x%02X, val: 0x%02X\n", i, data);
		udelay(500);
	}

	for (i = CS_CP_START_REG; i < (CS_CP_START_REG + cs->raw_num); i++) {
		ret = i2c_cypress_read(cs->client, i, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "addr: 0x , val: 0x\n");
			pr_info("%s: err\n", __func__);
			return p;
		}
		p += scnprintf(buf+p, PAGE_SIZE-p, "addr: 0x%02X, val: 0x%02X\n", i, data);
		pr_info("addr: 0x%02X, val: 0x%02X\n", i, data);
		udelay(500);
	}

	for (i = CS_RAW_START_REG; i < (CS_RAW_START_REG + (cs->raw_num * 4)); i++) {
		ret = i2c_cypress_read(cs->client, i, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "addr: 0x , val: 0x\n");
			pr_info("%s: err\n", __func__);
			return p;
		}
		p += scnprintf(buf+p, PAGE_SIZE-p, "addr: 0x%02X, val: 0x%02X\n", i, data);
		pr_info("addr: 0x%02X, val: 0x%02X\n", i, data);
		udelay(500);
	}

	for (i = CS_BL_START_REG; i < (CS_BL_START_REG + (cs->raw_num * 2)); i++) {
		ret = i2c_cypress_read(cs->client, i, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "addr: 0x , val: 0x\n");
			pr_info("%s: err\n", __func__);
			return p;
		}
		p += scnprintf(buf+p, PAGE_SIZE-p, "addr: 0x%02X, val: 0x%02X\n", i, data);
		pr_info("addr: 0x%02X, val: 0x%02X\n", i, data);
		udelay(500);
	}

	for (i = CS_FG_TH_START_REG; i < (CS_FG_TH_START_REG + cs->raw_num); i++) {
		ret = i2c_cypress_read(cs->client, i, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "addr: 0x , val: 0x\n");
			pr_info("%s: err\n", __func__);
			return p;
		}
		p += scnprintf(buf+p, PAGE_SIZE-p, "addr: 0x%02X, val: 0x%02X\n", i, data);
		pr_info("addr: 0x%02X, val: 0x%02X\n", i, data);
		udelay(500);
	}

	for (i = CS_NS_TH_START_REG; i < (CS_NS_TH_START_REG + cs->raw_num); i++) {
		ret = i2c_cypress_read(cs->client, i, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "addr: 0x , val: 0x\n");
			pr_info("%s: err\n", __func__);
			return p;
		}
		p += scnprintf(buf+p, PAGE_SIZE-p, "addr: 0x%02X, val: 0x%02X\n", i, data);
		pr_info("addr: 0x%02X, val: 0x%02X\n", i, data);
		udelay(500);
	}
	return p;
}

static ssize_t cypress_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);
	pr_info("%s\n", __func__);

	return cypress_print_reg(cs, buf);
}

static ssize_t cypress_reg_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);
	unsigned int data[2];
	uint8_t reg[2];
	int err;

	sscanf(buf, "0x%x 0x%x", &data[0], &data[1]);
	pr_info("%s: reg=0x%x, value=0x%x\n", __func__, data[0], data[1]);
	reg[0] = (uint8_t)data[0];
	reg[1] = (uint8_t)data[1];

	err = i2c_cypress_write_byte_data(cs->client, reg[0], reg[1]);
	if (err < 0)
			pr_err("%s: I2C write fail. err %d\n", __func__, err);
	pr_info("%s ---", __func__);
	return count;
}

static int cypress_sensitivity_setup(struct cypress_cap_data *cs, enum cypress_sensitivity level)
{
	uint8_t i;
	int err = 0;

	pr_debug("%s: %d\n", __func__, level);

	for (i = CS_SENSITIVITY_START_REG; i < (CS_SENSITIVITY_START_REG + cs->cap_num); i++) {
		err = i2c_cypress_write_byte_data(cs->client, (uint8_t) i, (uint8_t) level);
		if (err < 0)
			pr_err("%s: I2C write fail. err %d, addr: 0x%02X.\n", __func__, err, i);
	}

	return err;
}

static ssize_t cypress_glove_setting_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);
	pr_info("%s\n", __func__);
	return scnprintf(buf, PAGE_SIZE, "%d\n", cs->cap_glove_mode_status);
}

static ssize_t cypress_glove_setting_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);
	unsigned long input;

	if (kstrtoul(buf, 10, &input) != 0) {
		pr_err("%s: Failed to get the inout value\n", __func__);
		return -EINVAL;
	}
	if (input > 2) {
		pr_err("%s: wrong parameter\n", __func__);
		return -EINVAL;
	}

	if (input == 1) {
		cypress_sensitivity_setup(cs, CS_SENS_HIGH);
		cs->cap_glove_mode_status = 1;
	} else if (input == 0) {
		if (cs->cap_hall_s_pole_status == HALL_FAR)
			cypress_sensitivity_setup(cs, CS_SENS_DEFAULT);
		cs->cap_glove_mode_status = 0;
	}

	pr_info("%s: glove_setting change to %d\n", __func__, cs->cap_glove_mode_status);

	return count;
}

#ifdef CONFIG_AK8789_HALLSENSOR
static ssize_t cypress_cover_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);
	pr_info("%s\n", __func__);
	return scnprintf(buf, PAGE_SIZE, "%d\n", cs->cap_hall_s_pole_status);
}

static int hallsensor_handler_callback(struct notifier_block *self,
		         unsigned long status, void *unused)
{
	int pole = 0, pole_value = 0;
	struct cypress_cap_data *cs = container_of(self, struct cypress_cap_data, hallsensor_handler);

	pole_value = status & 0x01;
	pole = (status & 0x02) >> HALL_POLE_BIT;
	if (pole == HALL_POLE_S) {
		if (pole_value == HALL_NEAR) {
			if (cs->suspended == false)
				cypress_sensitivity_setup(cs, CS_SENS_HIGH);
			cs->cap_hall_s_pole_status = HALL_NEAR;
		} else {
			if (cs->suspended == false && cs->cap_glove_mode_status == 0)
				cypress_sensitivity_setup(cs, CS_SENS_DEFAULT);
			cs->cap_hall_s_pole_status = HALL_FAR;
		}
		pr_info("[HL] %s[%s] cover_enable = %d\n",
				pole? "att_s" : "att_n", pole_value ? "Near" : "Far",
				cs->cap_hall_s_pole_status);
	} else
		pr_info("[HL] %s[%s]\n", pole? "att_s" : "att_n", pole_value ? "Near" : "Far");
	return NOTIFY_OK;
}
#endif

static ssize_t cypress_debug_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%08X\n", debug_mask);
}

static ssize_t cypress_debug_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	if (kstrtou32(buf, 16, &debug_mask) != 0) {
		pr_err("bad parameter");
		return -EINVAL;
	}

	return count;
}

static ssize_t cypress_disable_key_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);
	unsigned int input = 0;

	if (kstrtouint(buf, 10, &input) != 0) {
		pr_err("bad parameter");
		return -EINVAL;
	}

	if ((input != 0) && (input != 1)) {
		pr_err("bad parameter");
		return -EINVAL;
	}

	pr_info("%s: %d\n", __func__, input);
	cs->disable_key = input;

	return count;
}

static ssize_t cypress_disable_key_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", cs->disable_key);
}

static int cypress_print_raw(struct cypress_cap_data *cs, char *buf)
{
	int ret, i, j = 0, p = 0;
	uint8_t data = 0, status = 0, buffer = 0;
	uint8_t cp[cs->raw_num], finger_th[cs->raw_num], noise_th[cs->raw_num];
	uint16_t raw[cs->raw_num], diff[cs->raw_num], baseline[cs->raw_num];

	pr_info("%s\n", __func__);
	if (i2c_cypress_read(cs->client, CS_STATUS, &buffer, 1) < 0) {
		p = scnprintf(buf, PAGE_SIZE, "err\n");
		return p;
	}
	pr_info("CS_STATUS:%d\n", buffer);
	udelay(500);
	for (i = CS_CP_START_REG; i < (CS_CP_START_REG + cs->raw_num); i++) {
		ret = i2c_cypress_read(cs->client, i, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "err\n");
			return p;
		}
		cp[j] = data;
		j++;
		udelay(500);
	}

	j = 0;
	for (i = CS_RAW_START_REG; i < (CS_RAW_START_REG + (cs->raw_num * 4)); (i += 4)) {
		ret = i2c_cypress_read(cs->client, i, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "err\n");
			return p;
		}
		raw[j] = data << 8;
		udelay(500);
		ret = i2c_cypress_read(cs->client, i + 1, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "err\n");
			return p;
		}
		raw[j] |= ((uint16_t) data);
		udelay(500);
		ret = i2c_cypress_read(cs->client, i + 2, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "err\n");
			return p;
		}
		diff[j] = data << 8;
		udelay(500);
		ret = i2c_cypress_read(cs->client, i + 3, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "err\n");
			return p;
		}
		diff[j] |= ((uint16_t) data);
		j++;
		udelay(500);
	}

	j = 0;
	for (i = CS_BL_START_REG; i < (CS_BL_START_REG + (cs->raw_num * 2)); (i += 2)) {
		ret = i2c_cypress_read(cs->client, i, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "err\n");
			return p;
		}
		baseline[j] = data << 8;
		udelay(500);
		ret = i2c_cypress_read(cs->client, i + 1, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "err\n");
			return p;
		}
		baseline[j] |= ((uint16_t) data);
		j++;
		udelay(500);
	}

	j = 0;
	for (i = CS_FG_TH_START_REG; i < (CS_FG_TH_START_REG + cs->raw_num); i++) {
		ret = i2c_cypress_read(cs->client, i, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "addr: 0x , val: 0x\n");
			pr_info("%s: err\n", __func__);
			return p;
		}
		finger_th[j] = data;
		j++;
		udelay(500);
	}

	j = 0;
	for (i = CS_NS_TH_START_REG; i < (CS_NS_TH_START_REG + cs->raw_num); i++) {
		ret = i2c_cypress_read(cs->client, i, &data, 1);
		if (ret < 0) {
			p = scnprintf(buf, PAGE_SIZE, "addr: 0x , val: 0x\n");
			pr_info("%s: err\n", __func__);
			return p;
		}
		noise_th[j] = data;
		j++;
		udelay(500);
	}

	for (i = 0; i < cs->raw_num; i++) {
		p += scnprintf(buf+p, PAGE_SIZE-p, "BTN(%d): ", i);
		if (i < cs->cap_num) {
			status = (buffer & BIT(i)) >> i;
			p += scnprintf(buf+p, PAGE_SIZE-p, "STATUS:%d, ", status);
		}
		p += scnprintf(buf+p, PAGE_SIZE-p, "CP=%d, RAW=%d, DIFF=%d, BL=%d, FINGER_TH=%d, NOISE_TH=%d\n",
						cp[i], raw[i], diff[i], baseline[i], finger_th[i], noise_th[i]);
		pr_info("BTN(%d): CP=%d, RAW=%d, DIFF=%d, BL=%d, FINGER_TH=%d, NOISE_TH=%d\n",
					i, cp[i], raw[i], diff[i], baseline[i], finger_th[i], noise_th[i]);
	}

	return p;
}

static ssize_t cypress_inform_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);

	return cypress_print_raw(cs, buf);
}

static void cypress_rawdata_print(struct work_struct *work)
{
	char buf[1024] = {0};
	int i, ret = 0;
	struct cypress_cap_data *cs = container_of(work, struct cypress_cap_data,
						work_raw.work);

	cs->reset_cnt++;
	if (cs->reset_cnt % CYPRESS_KEYLOCKRESET == 0) {
		ret = cypress_print_raw(cs, buf);
		if (debug_mask & BIT(2)) {
			ret = cypress_print_reg(cs, buf);
		}
		for (i = 0; i < cs->cap_num; i++) {
			input_report_key(cs->input_dev, cs->keycode[i], 0);
			input_sync(cs->input_dev);
		}
		cs->status = 0;
		pr_info("keylock reset\n");
		cs->reset(cs->client);
		cs->reset_cnt = 0;
	} else {
		if (debug_mask & BIT(2)) {
			ret = cypress_print_raw(cs, buf);
		}
		queue_delayed_work(cs->wq_raw, &cs->work_raw,
					msecs_to_jiffies(CYPRESS_KEYLOCKDELAYTIME));
	}
}

static void cancel_print_raw(struct cypress_cap_data *cs)
{
	int ret;
	ret = cancel_delayed_work_sync(&cs->work_raw);
	if (!ret)
		cancel_delayed_work(&cs->work_raw);
	cs->reset_cnt = 0;
}

static int cypress_set_sleep_mode(struct cypress_cap_data *cs, int on)
{
	int ret = 0;

	pr_info("%s: %d\n", __func__, on);
	ret = i2c_cypress_write_byte_data(cs->client, CS_SET_SLEEP, on);
	if (ret != 0)
		pr_err("Failed to set sleep mode %d\n", ret);

	return ret;
}

static int cypress_get_version(struct cypress_cap_data *cs)
{
	uint8_t version = 0;
	int ret = 0;
	pr_info("%s\n", __func__);

	ret = i2c_cypress_read(cs->client, CS_FW_VERSION, &version, 1);
	if (ret < 0) {
		pr_err("Ver Read Err\n");
		goto err_fw_get_fail;
	}
	pr_info("Cypress Cap Sensor V%x\n", version);
	return version;

err_fw_get_fail:
	return ret;
}

static int cypress_get_project_id(struct cypress_cap_data *cs)
{
	uint8_t pid = 0;
	int ret = 0;
	pr_info("%s\n", __func__);

	ret = i2c_cypress_read(cs->client, CS_PROJECT_ID, &pid, 1);
	if (ret < 0) {
		pr_err("Ver Read Err\n");
		goto err_fw_get_fail;
	}
	pr_info("Cypress Cap Sensor project ID:0x%X\n", pid);
	return pid;

err_fw_get_fail:
	return ret;
}

#ifdef SUPPORT_CYPRESS_FW_UPDATE
static int cypress_enter_bootloader(struct cypress_cap_data *cs)
{
	int ret = 0;

	pr_info("%s\n", __func__);
	ret = i2c_cypress_write_byte_data(cs->client, CS_ENTER_BL, 1);
	if (ret != 0)
		pr_err("Enter bootloader Error\n");

	return ret;
}

static void cypress_start_up_update_func(const struct firmware *fw, void *context)
{
	struct cypress_cap_data *cs = (struct cypress_cap_data *)context;
	struct cypress_fw_update_data *fw_update_data = cs->fw_update_data;
	int i, ret, tag_len = 0;

	if (IS_ERR_OR_NULL(fw)) {
		pr_info("%s: No Firmware file\n", __func__);
		return;
	}

	pr_info("%s\n", __func__);
	wake_lock(&cs->update_wake_lock);
	cs->stay_awake = true;
	if (debug_mask & BIT(1)) {
		cancel_print_raw(cs);
	}
	if (cs->irq_enabled) {
		disable_irq(cs->client->irq);
		cs->irq_enabled = 0;
	}
	ret = check_fw_version(fw, &tag_len, cs->info.version);
	if (ret == 0) {
		goto exit;
	}
	for (i = 0; i < CYPRESS_I2C_RETRY_TIMES; i++) {
		ret = update_firmware(fw_update_data, cs, (char *)fw->data + tag_len, fw->size - tag_len);
		cs->reset(cs->client);
		if (ret == 0)
			break;
	}
	if (i == CYPRESS_I2C_RETRY_TIMES)
		pr_err("retry 3 times. FW update failed\n");

exit:
	if (!cs->irq_enabled) {
		enable_irq(cs->client->irq);
		cs->irq_enabled = 1;
	}
	if (cypress_init_sensor(cs) < 0) {
		pr_err("Get chip info Err\n");
	}
	cs->stay_awake = false;
	wake_unlock(&cs->update_wake_lock);
}

static int cypress_start_up_update(struct cypress_cap_data *cs)
{
	int ret = 0;
	struct i2c_client *client = cs->client;

	if ((strcmp(htc_get_bootmode(), "download") == 0)
		|| (strcmp(htc_get_bootmode(), "RUU") == 0)
		|| (strcmp(htc_get_bootmode(), "ftm") == 0)) {
		pr_info("%s mode, skip update work\n", htc_get_bootmode());
		return 0;
	}

	pr_info("%s\n", __func__);
	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG
			, "cs_CY8C.img"
			, &client->dev, GFP_KERNEL
			, cs, cypress_start_up_update_func);
	return ret;
}
#endif

static int cypress_init_sensor(struct cypress_cap_data *cs)
{
	int version = 0, ret = 0, pid = 0;
#ifdef SUPPORT_CYPRESS_FW_UPDATE
	struct cypress_fw_update_data *fw_update_data = cs->fw_update_data;
#endif

	pr_info("%s\n", __func__);

	version = cypress_get_version(cs);
	if (version < 0) {
#ifdef SUPPORT_CYPRESS_FW_UPDATE
		ret = cypress_start_bootloader(fw_update_data);
		if (ret < 0) {
			pr_info("Chip not exist\n");
			goto exit;
		}
		cypress_end_bootloader(fw_update_data);
		cs->cs_bootloader_mode = 1;
		cs->info.version = 0;
		pr_info("Chip in Bootload mode\n");
#else
		pr_info("Chip not exist\n");
		ret = -1;
		goto exit;
#endif
	} else {
		cs->info.version = version;
		cs->cs_bootloader_mode = 0;
		pid = cypress_get_project_id(cs);
		if (pid < 0) {
			cs->info.pid = 0;
			pr_err("get pid err\n");
			ret = -1;
			goto exit;
		}
		cs->info.pid = pid;
	}
	pr_info("%s, cs_bootloader_mode = %d\n", __func__, cs->cs_bootloader_mode);
	return 0;

exit:
	return ret;
}

static void report_key_func(struct cypress_cap_data *cs, uint8_t status)
{
	int i = 0;

	pr_debug("status = %x\n", status);

	if (status) {
		for (i = 0; i < cs->cap_num; i++) {
			if ((status & BIT(i)) != (cs->status & BIT(i))) {
				if (status & BIT(i)) {
					input_report_key(cs->input_dev, cs->keycode[i], 1);
					input_sync(cs->input_dev);
					pr_info("%s key pressed\n", keycode_check(cs->keycode[i]));
				} else {
					input_report_key(cs->input_dev, cs->keycode[i], 0);
					input_sync(cs->input_dev);
					pr_info("%s key released\n", keycode_check(cs->keycode[i]));
				}
			}
		}
	} else {
		for (i = 0; i < cs->cap_num; i++) {
			input_report_key(cs->input_dev, cs->keycode[i], 0);
			input_sync(cs->input_dev);
			if ((status & BIT(i)) != (cs->status & BIT(i))) {
				pr_info("%s key released\n", keycode_check(cs->keycode[i]));
			}
		}
	}
	cs->status = status;

	if (debug_mask & BIT(1)) {
		if (cs->status) {
			queue_delayed_work(cs->wq_raw, &cs->work_raw,
						msecs_to_jiffies(CYPRESS_KEYLOCKTIME));
		} else {
			cancel_print_raw(cs);
		}
	}
}

static irqreturn_t cypress_cap_irq_handler(int irq, void *dev_id)
{
	struct cypress_cap_data *cs = dev_id;
	uint8_t buf = 0;

	if (i2c_cypress_read(cs->client, CS_STATUS, &buf, 1) < 0) {
		pr_err("%s i2c read fail", __func__);
		return IRQ_HANDLED;
	}

	pr_debug("cap sensor status : 0x%x\n", buf);

	if (cs->disable_key) {
		pr_info("disable_key:status : 0x%x\n", buf);
		cs->status = buf;
	}
	else
		report_key_func(cs, buf);

	return IRQ_HANDLED;
}

static int cypress_cap_pinctrl_init(struct device *dev)
{
	int ret;
	struct cypress_cap_platform_data *pdata = dev_get_platdata(dev);

	pr_info("%s\n", __func__);
	
	pdata->cap_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pdata->cap_pinctrl)) {
		pr_info("Target does not use pinctrl\n");
		ret = PTR_ERR(pdata->cap_pinctrl);
		pdata->cap_pinctrl = NULL;
		return ret;
	}

	pdata->gpio_state_active
		= pinctrl_lookup_state(pdata->cap_pinctrl, "pmx_cap_active");
	if (IS_ERR_OR_NULL(pdata->gpio_state_active)) {
		pr_info("Can not get ts default pinstate\n");
		ret = PTR_ERR(pdata->gpio_state_active);
		pdata->cap_pinctrl = NULL;
		return ret;
	}

	pdata->gpio_state_suspend
		= pinctrl_lookup_state(pdata->cap_pinctrl, "pmx_cap_suspend");
	if (IS_ERR_OR_NULL(pdata->gpio_state_suspend)) {
		pr_info("Can not get ts sleep pinstate\n");
		ret = PTR_ERR(pdata->gpio_state_suspend);
		pdata->cap_pinctrl = NULL;
		return ret;
	}

	return 0;
}

static void cypress_cap_pinctrl_deinit(struct device *dev)
{
	struct cypress_cap_platform_data *pdata = dev_get_platdata(dev);

	pr_info("%s\n", __func__);
	
	if (pdata->cap_pinctrl != NULL) {
		pdata->gpio_state_active = NULL;
		pdata->gpio_state_suspend = NULL;
		devm_pinctrl_put(pdata->cap_pinctrl);
	}
}

static int cypress_cap_pinctrl_select(struct device *dev, int on)
{
	struct pinctrl_state *pins_state;
	struct cypress_cap_platform_data *pdata = dev_get_platdata(dev);
	int ret;

	pr_info("%s: set to %d\n", __func__, on);
	pins_state = on ? pdata->gpio_state_active
		: pdata->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(pdata->cap_pinctrl, pins_state);
		if (ret) {
			pr_err("can not set %s pins\n",
				on ? "pmx_cap_active" : "pmx_cap_suspend");
			return ret;
		}
	} else
		pr_err("not a valid '%s' pinstate\n",
				on ? "pmx_cap_active" : "pmx_cap_suspend");

	return 0;
}

static int cypress_cap_regulator_set(struct cypress_cap_platform_data *pdata)
{
	int enable = 0, ret = 0;

	
	if (gpio_is_valid(pdata->gpio_id)) {
		ret = gpio_request(pdata->gpio_id, "cap_id_pin");
		if (ret) {
			pr_info(" %s : request gpio%d fail\n",
				__func__, pdata->gpio_id);
		} else {
			gpio_direction_input(pdata->gpio_id);
			enable = (gpio_get_value(pdata->gpio_id) ? 1 : 0) ^ pdata->gpio_id_active_low;
			pr_info("%s: id pin status:%d, active low:%d, enable=%d\n", __func__,
				gpio_get_value(pdata->gpio_id), pdata->gpio_id_active_low, enable);
		}
	}

	if (pdata->cap_regulator && enable) {
		if (regulator_count_voltages(pdata->cap_regulator) > 0) {
			ret = regulator_set_voltage(pdata->cap_regulator,
				pdata->cap_regulator_volt, pdata->cap_regulator_volt);
			if (ret)
				pr_err("%s: cap regulator set voltage failed, ret=%d\n",
					__func__, ret);
			else
				pr_info("%s: cap regulator set to %duV\n", __func__,
						pdata->cap_regulator_volt);
		}
	} else
		pr_info("%s: cap regulator voltage is unchanged.\n", __func__);

	return ret;
}

static int cypress_cap_set_gpio(struct cypress_cap_platform_data *pdata)
{
	int ret;

	
	ret = gpio_request(pdata->gpio_irq, "cap_irq");
	if (ret) {
		pr_err("%s : request gpio%d fail\n",
			__func__, pdata->gpio_irq);
		return -ENODEV;
	}

	
	ret = gpio_request(pdata->gpio_rst, "cap_rst");
	if (ret) {
		pr_err(" %s : request gpio%d fail\n",
			__func__, pdata->gpio_rst);
		return -ENODEV;
	}

	
	ret = gpio_request(pdata->gpio_ind, "cap_i2c");
	if (ret) {
		pr_err(" %s : request gpio%d fail\n",
			__func__, pdata->gpio_ind);
		return -ENODEV;
	}
	gpio_direction_input(pdata->gpio_irq);
	gpio_direction_output(pdata->gpio_ind, 1);
	gpio_direction_output(pdata->gpio_rst, 1);

	return 0;
}

int cypress_cap_reset(struct i2c_client *client)
{
	struct cypress_cap_data *cs = dev_get_drvdata(&client->dev);
	struct cypress_cap_platform_data *pdata = client->dev.platform_data;

	pr_info("%s\n", __func__);
	gpio_direction_output(pdata->gpio_rst, 0);
	msleep(5);
	gpio_direction_output(pdata->gpio_rst, 1);
	msleep(50);

	if (cs->cap_glove_mode_status == 1
#ifdef CONFIG_AK8789_HALLSENSOR
		|| cs->cap_hall_s_pole_status == HALL_NEAR
#endif
		) {
		cypress_sensitivity_setup(cs, CS_SENS_HIGH);
	} else if (cs->cap_glove_mode_status == 0
#ifdef CONFIG_AK8789_HALLSENSOR
		&& cs->cap_hall_s_pole_status == HALL_FAR
#endif
		) {
		cypress_sensitivity_setup(cs, CS_SENS_DEFAULT);
	}
	return 0;
}

static int cypress_parse_dt(struct device *dev, struct cypress_cap_platform_data *pdata)
{
	struct device_node *dt = dev->of_node;
	struct device_node *pp = NULL;
	enum of_gpio_flags flags = 0x0;
	int *keycode = NULL;
	uint32_t data;
	int cap_id = 0;
	int id;

	pr_info("%s: Start\n", __func__);
	id = of_alias_get_id(dt, "cap");
	if (id < 0)
		id = 0;
	
	pdata->gpio_irq = of_get_named_gpio_flags(dt, "cap-sensor-irq",
				0, NULL);

	pdata->gpio_rst = of_get_named_gpio_flags(dt, "cap-sensor-rst",
				0, NULL);

	pdata->gpio_ind = of_get_named_gpio_flags(dt, "cap-sensor-ind",
				0, NULL);

	pdata->reset = cypress_cap_reset;
	pr_info("%s: gpio_irq:%d, gpio_rst:%d, gpio_ind:%d \n", __func__,
		pdata->gpio_irq, pdata->gpio_rst, pdata->gpio_ind);

	pdata->gpio_id  = of_get_named_gpio_flags(dt, "cap-sensor-id",
				0, &flags);

	pdata->gpio_id_active_low = flags & OF_GPIO_ACTIVE_LOW;

	pdata->cap_regulator = devm_regulator_get(dev, "cap");
	if (IS_ERR_OR_NULL(pdata->cap_regulator)) {
		pdata->cap_regulator = NULL;
		pdata->cap_regulator_volt = 0;
		pr_info("%s: cap-supply is not assigned.\n", __func__);
	} else {
		if (of_property_read_u32(dt, "cap-voltage-max", &data) == 0)
			pdata->cap_regulator_volt = data;
		else {
			pdata->cap_regulator = NULL;
			pdata->cap_regulator_volt = 0;
		}

		pr_info("%s: gpio_id:%d(Active %s); cap_regulator_volt:%duV.\n",
			__func__, pdata->gpio_id, pdata->gpio_id_active_low ? "Low" : "High",
			pdata->cap_regulator_volt);
	}

	if (of_property_read_u32(dt, "cap-num", &data) == 0)
		pdata->cap_num = data;

	if (of_property_read_u32(dt, "raw-cap", &data) == 0)
		pdata->raw_num = data;
	else
		pdata->raw_num = pdata->cap_num;

	if (of_property_read_u32(dt, "project-id", &data) == 0)
		pdata->info.pid = data;
	else
		pdata->info.pid = 0;
	pr_info("%s: project ID = 0x%X\n", __func__, pdata->info.pid);

	if (of_property_read_u32(dt, "update-feature", &data) == 0)
		pdata->update_feature = data;
	else
		pdata->update_feature = 0;

	keycode = kzalloc(pdata->cap_num * (sizeof(int)), GFP_KERNEL);
	if (keycode == NULL) {
		pr_err("Fail to allocate keycode\n");
		return -ENOMEM;
	}

	while ((pp = of_get_next_child(dt, pp))) {
		if (of_property_read_u32(pp, "cap-id", &cap_id)) {
			pr_err("Failed to get property: cap-id\n");
			break;
		}
		if (of_property_read_u32(pp, "keycode", &keycode[cap_id])) {
			pr_err("Failed to get property: key-code\n");
			break;
		}
		pr_info("%s: cap[%d]:%s \n", __func__, cap_id, keycode_check(keycode[cap_id]));
	}
	memcpy(pdata->keycode, keycode, (pdata->cap_num * sizeof(int)));
	kfree(keycode);

	return 0;
}

static int cypress_cap_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct cypress_cap_data *cs;
	struct cypress_cap_platform_data *pdata;
	int i, ret = 0, attr_count;

	pr_info( "%s: enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			pr_err("%s : cap-sensor probe fail because platform_data \
				is NULL\n", __func__);
			return  -ENODEV;
		}
		ret = cypress_parse_dt(&client->dev, pdata);
		if (ret) {
			pr_err("%s : cypress_parse_dt fail\n", __func__);
			ret = -ENODEV;
			goto err_exit;
		}
		client->dev.platform_data = pdata;
	} else {
		pdata = client->dev.platform_data;
		if (pdata == NULL) {
			pr_err("%s : cypress cap-sensor probe fail because platform_data \
					is NULL\n", __func__);
			return  -ENODEV;
		}
	}

	cs = kzalloc(sizeof(struct cypress_cap_data), GFP_KERNEL);
	if (cs == NULL) {
		pr_err("allocate cypress_cap_data failed\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	cs->client = client;
	i2c_set_clientdata(client, cs);

	if (cypress_cap_pinctrl_init(&client->dev) != 0) {
		pr_info("Can't use pinctrl\n");
	} else {
		cypress_cap_pinctrl_select(&client->dev, 1);
	}

	cypress_cap_regulator_set(pdata);

	cypress_cap_set_gpio(pdata);
	if (pdata) {
		pdata->reset(client);
		cs->gpio_irq = pdata->gpio_irq;
		cs->gpio_ind = pdata->gpio_ind;
	}

#ifdef SUPPORT_CYPRESS_FW_UPDATE
	if (cypress_fw_update_get_data(cs) < 0) {
		pr_info("Not support FW update\n");
		cs->fw_update_data = NULL;
	}
#endif

	if (cypress_init_sensor(cs) < 0) {
		pr_err("init failure, not probe up driver\n");
		ret = -ENODEV;
		goto err_init_sensor_failed;
	}

	if ((strcmp(htc_get_bootmode(), "offmode_charging") == 0)
		|| (strcmp(htc_get_bootmode(), "recovery") == 0)) {
		pr_info("%s mode. Set touch chip to sleep mode and skip touch driver probe\n", htc_get_bootmode());
		if (cs->cs_bootloader_mode == 0)
			cypress_set_sleep_mode(cs, 1);
		ret = -ENODEV;
		goto err_off_mode;
	}

	cs->input_dev = input_allocate_device();
	if (cs->input_dev == NULL) {
		ret = -ENOMEM;
		pr_err("Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	cs->input_dev->name       = "cypress-cap-sensor";
	cs->input_dev->id.product = cs->info.pid;
	cs->input_dev->id.version = cs->info.version;
	cs->keycode               = pdata->keycode;
	cs->reset                 = pdata->reset;
	cs->cap_num               = pdata->cap_num;
	cs->raw_num               = pdata->raw_num;
	cs->update_feature        = pdata->update_feature;
	cs->disable_key           = 0;
	cs->stay_awake            = false;
	cs->suspended             = false;

	set_bit(EV_SYN, cs->input_dev->evbit);
	set_bit(EV_KEY, cs->input_dev->evbit);
	for (i = 0; i < cs->cap_num; i++ ) {
		set_bit(cs->keycode[i], cs->input_dev->keybit);
	}

	ret = input_register_device(cs->input_dev);
	if (ret) {
		pr_err("unable to register %s input device\n",
			cs->input_dev->name);

		goto err_input_register_device_failed;
	}

#ifdef CONFIG_FB
	cs->fb_notifier.notifier_call = fb_notifier_callback;
	fb_register_client(&cs->fb_notifier);
#endif

	cs->cap_glove_mode_status = 0;
#ifdef CONFIG_AK8789_HALLSENSOR
	cs->cap_hall_s_pole_status = HALL_FAR;
	cs->hallsensor_handler.notifier_call = hallsensor_handler_callback;
	hallsensor_register_notifier(&cs->hallsensor_handler);
#endif

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		ret = sysfs_create_file(&client->dev.kobj,
				&attrs[attr_count].attr);
		if (ret < 0) {
			pr_err("%s: Failed to create sysfs attributes\n", __func__);
			goto err_create_sysfs_failed;
		}
	}
	ret = sysfs_create_link(NULL, &client->dev.kobj, "android_cap");
	if (ret < 0) {
		pr_err("%s: subsystem_register failed\n", __func__);
		goto err_create_link_failed;
	}

	list_add(&cs->list, &cs_list);

	if (debug_mask & BIT(1)) {
		pr_info("debug_mask = %02x\n", debug_mask);
		cs->wq_raw = create_singlethread_workqueue("cypress_print_rawdata");
		if (!cs->wq_raw) {
			pr_err("allocate cypress_print_rawdata failed\n");
			ret = -ENOMEM;
			goto err_create_thread_failed;
		}
		INIT_DELAYED_WORK(&cs->work_raw, cypress_rawdata_print);
	}

	if (client->irq) {
		ret = request_threaded_irq(client->irq, NULL, cypress_cap_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					CYPRESS_CS_NAME,
					cs);
		if (ret < 0) {
			pr_err("request_irq failed for gpio %d,"
				" irq %d\n", cs->gpio_irq, client->irq);
                       goto err_request_irq_failed;
		}
		else {
			pr_info("%s : Set IRQ %d falling edge trigger \n", __func__, client->irq);
			cs->irq_enabled = 1;
		}
	}

#ifdef SUPPORT_CYPRESS_FW_UPDATE
	if (cs->update_feature == 1) {
		wake_lock_init(&cs->update_wake_lock, WAKE_LOCK_SUSPEND, "cs_wake_lock");
		ret = cypress_start_up_update(cs);
		if (ret) {
			pr_err("%s: Fail to create update work:%d\n", __func__, ret);
		}
	}
#endif
	pr_info("%s: probe success\n", __func__);
	return 0;

err_request_irq_failed:
err_create_thread_failed:
	sysfs_remove_link(NULL, "android_cap");
err_create_link_failed:
err_create_sysfs_failed:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&client->dev.kobj,
				&attrs[attr_count].attr);
	}
#ifdef CONFIG_FB
	fb_unregister_client(&cs->fb_notifier);
#endif
err_input_register_device_failed:
	input_free_device(cs->input_dev);

err_input_dev_alloc_failed:
err_off_mode:
err_init_sensor_failed:
	if (cs->fw_update_data != NULL) {
		cypress_fw_update_free_data(cs->fw_update_data);
	}
	if (pdata->cap_pinctrl)
		cypress_cap_pinctrl_select(&client->dev, 0);
	cypress_cap_pinctrl_deinit(&client->dev);
	kfree(cs);

err_alloc_data_failed:
err_check_functionality_failed:
err_exit:
	pr_err("%s: probe fail\n", __func__);
	return ret;
}

static int cypress_cap_remove(struct i2c_client *client)
{
	struct cypress_cap_data *cs = i2c_get_clientdata(client);
	struct cypress_cap_platform_data *pdata = client->dev.platform_data;
	int attr_count = 0;

#ifdef SUPPORT_CYPRESS_FW_UPDATE
	wake_lock_destroy(&cs->update_wake_lock);
#endif

	free_irq(client->irq, cs);

	list_del(&cs->list);
	sysfs_remove_link(NULL, "android_cap");
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&client->dev.kobj,
				&attrs[attr_count].attr);
	}

#ifdef CONFIG_FB
	fb_unregister_client(&cs->fb_notifier);
#endif
#ifdef CONFIG_AK8789_HALLSENSOR
	hallsensor_unregister_notifier(&cs->hallsensor_handler);
#endif
	input_unregister_device(cs->input_dev);

	if (pdata->cap_pinctrl)
		cypress_cap_pinctrl_select(&client->dev, 0);
	cypress_cap_pinctrl_deinit(&client->dev);
	kfree(cs);

	return 0;
}

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
                               unsigned long event, void *data)
{
	struct fb_event	*evdata = data;
	int		*blank;
	struct cypress_cap_data *cs = container_of(self,
				struct cypress_cap_data, fb_notifier);

	pr_info("%s, event = %ld\n", __func__, event);
	if (evdata && evdata->data) {
		blank = evdata->data;
		pr_debug("%s, blank = %d\n", __func__, *blank);
		if (event == FB_EARLY_EVENT_BLANK) {
		        if (*blank == FB_BLANK_POWERDOWN)
				cypress_cap_early_suspend(&cs->client->dev);
		} else if (event == FB_EVENT_BLANK) {
		        if (*blank == FB_BLANK_UNBLANK)
				cypress_cap_late_resume(&cs->client->dev);
		}
	}
	return 0;
}

static void cypress_free_status(struct cypress_cap_data *cs)
{
	int i;

	for (i = 0; i < cs->cap_num; i++) {
		input_report_key(cs->input_dev, cs->keycode[i], 0);
		input_sync(cs->input_dev);
	}
	cs->status = 0;
}

static void cypress_cap_early_suspend(struct device *dev)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);
	struct cypress_cap_platform_data *pdata = dev_get_platdata(dev);

	pr_info("%s\n", __func__);

	if (debug_mask & BIT(1)) {
		cancel_print_raw(cs);
	}
	cypress_free_status(cs);

	if (cs->stay_awake)
		return;

	cs->suspended = true;
	cypress_set_sleep_mode(cs, 1);
	if (cs->irq_enabled) {
		disable_irq(cs->client->irq);
		cs->irq_enabled = 0;
	}

	if (pdata->cap_pinctrl)
		cypress_cap_pinctrl_select(dev, 0);
}

static void cypress_cap_late_resume(struct device *dev)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);
	struct cypress_cap_platform_data *pdata = dev_get_platdata(dev);

	pr_info("%s\n", __func__);

	if (cs->stay_awake)
		return;

	if (pdata->cap_pinctrl)
		cypress_cap_pinctrl_select(dev, 1);

	if (!cs->irq_enabled) {
		enable_irq(cs->client->irq);
		cs->irq_enabled = 1;
	}
	cypress_set_sleep_mode(cs, 0);

	if (cs->cap_glove_mode_status == 1
#ifdef CONFIG_AK8789_HALLSENSOR
		|| cs->cap_hall_s_pole_status == HALL_NEAR
#endif
		) {
		cypress_sensitivity_setup(cs, CS_SENS_HIGH);
	} else if (cs->cap_glove_mode_status == 0
#ifdef CONFIG_AK8789_HALLSENSOR
		&& cs->cap_hall_s_pole_status == HALL_FAR
#endif
		) {
		cypress_sensitivity_setup(cs, CS_SENS_DEFAULT);
	}
	cs->suspended = false;
}
#endif

#ifdef CONFIG_PM
static int cypress_cap_suspend(struct device *dev)
{
	struct cypress_cap_data *cs = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);

	if (cs->irq_enabled) {
		cypress_set_sleep_mode(cs, 1);
		disable_irq(cs->client->irq);
		cs->irq_enabled = 0;
	}

	return 0;
}

static int cypress_cap_resume(struct device *dev)
{

	pr_info("%s\n", __func__);
	return 0;
}

static const struct dev_pm_ops cypress_cap_pm_ops = {
	.suspend		= cypress_cap_suspend,
	.resume			= cypress_cap_resume,
};
#endif

static struct of_device_id cypress_cap_match_table[] = {
	{ .compatible = "cypress_cap",},
	{ },
};


static const struct i2c_device_id cypress_cap_id[] = {
	{ CYPRESS_CS_NAME, 0 },
};

static struct i2c_driver cypress_cap_driver = {
	.probe		= cypress_cap_probe,
	.remove		= cypress_cap_remove,
	.id_table	= cypress_cap_id,

	.driver = {
		.owner  = THIS_MODULE,
		.name = CYPRESS_CS_NAME,
		.of_match_table = cypress_cap_match_table,
#ifdef CONFIG_PM
		.pm = &cypress_cap_pm_ops,
#endif
	},
};

static void __init cypress_cap_init_async(void *unused, async_cookie_t cookie)
{
	pr_info("%s: +++++\n", __func__);
	i2c_add_driver(&cypress_fw_update_driver);
	i2c_add_driver(&cypress_cap_driver);
	pr_info("%s: -----\n", __func__);
}

static int __init cypress_cap_init(void)
{
	pr_info("%s: enter\n", __func__);
	async_schedule(cypress_cap_init_async, NULL);
	return 0;
}

static void __exit cypress_cap_exit(void)
{
	i2c_del_driver(&cypress_cap_driver);
	i2c_del_driver(&cypress_fw_update_driver);
}

module_init(cypress_cap_init);
module_exit(cypress_cap_exit);

MODULE_DESCRIPTION("cypress_cap driver");
MODULE_LICENSE("GPL");
