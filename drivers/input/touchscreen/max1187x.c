/* drivers/input/touchscreen/max1187x.c
 *
 * Copyright (c)2013 Maxim Integrated Products, Inc.
 *
 * Driver Version: 3.0.7.1
 * Release Date: Mar 15, 2013
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
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/firmware.h>
#include <linux/crc16.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/jiffies.h>
#include <asm/byteorder.h>
#include <linux/input/max1187x.h>
#ifdef CONFIG_TOUCHSCREEN_TOUCH_FW_UPDATE
#include <linux/input/touch_fw_update.h>
#endif
#ifdef CONFIG_SYNC_TOUCH_STATUS
#include <linux/CwMcuSensor.h>
#endif
#ifdef CONFIG_FB
#include <linux/fb.h>
#endif
#ifdef CONFIG_AK8789_HALLSENSOR
#include <linux/hall_sensor.h>
#endif

#ifdef pr_fmt
#undef pr_fmt
#endif
//#define pr_fmt(fmt) MAX1187X_LOG_NAME "(%s:%d): " fmt, __func__, __LINE__
#define pr_fmt(fmt) MAX1187X_LOG_NAME ": " fmt

#ifdef pr_info
#undef pr_info
#endif
#define pr_info(fmt, ...) printk(KERN_INFO pr_fmt(fmt) "\n", ##__VA_ARGS__)

#ifdef pr_err
#undef pr_err
#endif
#define pr_err(fmt, ...) printk(KERN_ERR MAX1187X_LOG_NAME \
			"[ERR]:(%s:%d): " fmt "\n", __func__, __LINE__, ##__VA_ARGS__)

#define pr_dbg(a, b, ...) do { if (debug_mask & a) \
			pr_info(b, ##__VA_ARGS__);	\
			} while (0)

#define pr_info_if(a, b, ...) do { if ((debug_mask >> 16) & a) \
			pr_info(b, ##__VA_ARGS__);	\
			} while (0)
#define debugmask_if(a) ((debug_mask >> 16) & a)

#define ENABLE_IRQ()                            \
do {                                            \
	mutex_lock(&ts->irq_mutex);             \
	if (ts->irq_disabled) {                 \
		enable_irq(ts->client->irq);    \
		ts->irq_disabled = 0;           \
	}                                       \
	mutex_unlock(&ts->irq_mutex);           \
} while (0)

#define DISABLE_IRQ()                           \
do {                                            \
	mutex_lock(&ts->irq_mutex);             \
	if (ts->irq_disabled == 0) {            \
		disable_irq(ts->client->irq);   \
		ts->irq_disabled = 1;           \
	}                                       \
	mutex_unlock(&ts->irq_mutex);           \
} while (0)

#define NWORDS(a)    (sizeof(a) / sizeof(u16))
#define BYTE_SIZE(a) ((a) * sizeof(u16))
#define BYTEH(a)     ((a) >> 8)
#define BYTEL(a)     ((a) & 0xFF)

#define PDATA(a)      (ts->pdata->a)

#define RETRY_TIMES 3
#define RETRY_TIMES_CONFIG 5
#define SHIFT_BITS 10

static u32 debug_mask = 0x00080000;
static struct kobject *android_touch_kobj;
static struct data *gl_ts;
static unsigned int support_htc_event_flag = 0;

/* tanlist - array containing tan(i)*(2^16-1) for i=[0,45], i in degrees */
u16 tanlist[] = {0, 1144, 2289, 3435, 4583, 5734,
			6888, 8047, 9210, 10380, 11556, 12739,
			13930, 15130, 16340, 17560, 18792, 20036,
			21294, 22566, 23853, 25157, 26478, 27818,
			29178, 30559, 31964, 33392, 34846, 36327,
			37837, 39377, 40951, 42559, 44204, 45888,
			47614, 49384, 51202, 53069, 54990, 56969,
			59008, 61112, 63286, 65535};

/* config num - touch, calib, private, lookup, image
	p7 config num, p8 config num */
u16 config_num[2][5] = {{42, 50, 23, 8, 1},
					{65, 74, 34, 8, 0}};

struct report_reader {
	u16 report_id;
	u16 reports_passed;
	struct semaphore sem;
	int status;
};

struct report_point {
	u8 state;
	int x;
	int y;
	int z;
	int w;
};

struct data {
	struct max1187x_pdata *pdata;
	struct max1187x_board_config  *fw_config;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *sr_input_dev;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
	u8 early_suspend_registered;
#endif
#ifdef CONFIG_FB
	struct notifier_block fb_notifier;
#endif
	atomic_t scheduled_work_irq;
	u32 irq_receive_time;
	struct mutex irq_mutex;
	struct mutex i2c_mutex;
	struct mutex report_mutex;
	struct semaphore report_sem;
	struct report_reader report_readers[MAX_REPORT_READERS];
	u8 irq_disabled;
	u8 report_readers_outstanding;
	u16 rx_report[1000]; /* with header */
	u16 rx_report_len;
	u16 rx_packet[MAX_WORDS_REPORT + 1]; /* with header */
	u32 irq_count;
	u16 framecounter;
	u8 got_report;
	int fw_index;
	u16 fw_crc16;
	u16 fw_version[MAX_WORDS_REPORT];
	u16 touch_config[MAX_WORDS_COMMAND_ALL];
	char phys[32];
	u8 fw_responsive;
	u8 have_fw;
	u8 have_touchcfg;
	u8 sysfs_created;
	u8 is_raw_mode;
	char debug_string[DEBUG_STRING_LEN_MAX];
	u16 max11871_Touch_Configuration_Data[MAX1187X_TOUCH_CONFIG_MAX+2];
	u16 max11871_Calibration_Table_Data[MAX1187X_CALIB_TABLE_MAX+2];
	u16 max11871_Private_Configuration_Data[MAX1187X_PRIVATE_CONFIG_MAX+2];
	u16 max11871_Lookup_Table_X_Data[MAX1187X_LOOKUP_TABLE_MAX+3];
	u16 max11871_Lookup_Table_Y_Data[MAX1187X_LOOKUP_TABLE_MAX+3];
	u16 max11871_Image_Factor_Table[MAX1187X_IMAGE_FACTOR_MAX];
	u8 config_protocol;
	struct report_point report_points[10];
	u32 width_factor;
	u32 height_factor;
	u32 width_offset;
	u32 height_offset;
	u8  noise_level;
	char fw_ver[16];
	u32 config_id;
	u8  protocol_ver;
	u16 vendor_pin;
	u8  baseline_mode;
	u16 frame_rate[2];
	u16 x_channel;
	u16 y_channel;
	int16_t report[1000];
	u8 vk_press;
	u8 finger_press;
	u8 finger_log;
	struct max1187x_virtual_key *button_data;
	u16 button0:1;
	u16 button1:1;
	u16 button2:1;
	u16 button3:1;
	u16 cycles:1;
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	u16 glove_setting;
	u16 glove_enable;
	u16 glove_status;
	u8 pen_status;
	u8 cover_mode;
	u8 i2c_to_mcu;
	u8 hall_block_touch_event;
};

//extern unsigned int get_tamper_sf(void);
//extern char *htc_get_bootmode(void);
//extern char *disp_vendor(void);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void early_suspend(struct early_suspend *h);
static void late_resume(struct early_suspend *h);
#endif
#ifdef CONFIG_SYNC_TOUCH_STATUS
static void switch_sensor_hub(struct data *ts, int mode);
#endif
#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);
static void early_suspend(struct device *dev);
static void late_resume(struct device *dev);
#endif

static int device_fw_load(struct data *ts, const struct firmware *fw,
	u16 fw_index, int tagLen);
static int device_init(struct i2c_client *client);
static int device_deinit(struct i2c_client *client);

static int bootloader_enter(struct data *ts);
static int bootloader_exit(struct data *ts);
static int bootloader_get_crc(struct data *ts, u16 *crc16,
		u16 addr, u16 len, u16 delay);
static int bootloader_set_byte_mode(struct data *ts);
static int bootloader_erase_flash(struct data *ts);
static int bootloader_write_flash(struct data *ts, const u8 *image, u16 length);

static int set_touch_frame(struct i2c_client *client,
			u16 idle_frame, u16 active_frame);
static int set_baseline_mode(struct i2c_client *client, u16 mode);
static int set_glove_mode(struct i2c_client *client, u16 enable);
static int set_cover_mode(struct i2c_client *client, u16 enable);
static int set_edge_filter(struct i2c_client *client, u16 enable, u16 *range);
static int change_touch_rpt(struct i2c_client *client, u16 to);
static int sreset(struct i2c_client *client);
static int get_touch_config(struct i2c_client *client);
static int get_fw_version(struct i2c_client *client);
static void propagate_report(struct data *ts, int status, u16 *report);
static int get_report(struct data *ts, u16 report_id, ulong timeout);
static void release_report(struct data *ts);
static void system_status_handler(struct data *ts);
static void set_chip_mode(struct data *ts);

#if 0
static u16 binary_search(const u16 *array, u16 len, u16 val);
static s16 max1187x_orientation(s16 x, s16 y);
static u16 max1187x_sqrt(u32 num);
#endif

static u8 bootloader;
static u8 init_state;
#ifdef CONFIG_TOUCHSCREEN_TOUCH_FW_UPDATE
#define TOUCH_VENDOR "MAXIM"
#define FW_FLASH_TIMEOUT 60
struct touch_fwu_notifier maxim_tp_notifier;
static void maxim_fwu_progress(int percentage);
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
		case KEY_MENU:
			return "MENU";
		case KEY_SEARCH:
			return "SEARCH";
		default:
			return "RESERVED";
	}
}

static char state2char(int status)
{
	switch (status) {
		case MAX1187X_PEN_MODE:
			return 'P';
		case MAX1187X_FINGER_MODE:
			return 'F';
		case MAX1187X_GLOVE_MODE:
			return 'G';
		default:
			return 'F';
	}
}

#ifdef CONFIG_AK8789_HALLSENSOR
static DEFINE_MUTEX(max1187x_block_mutex);
static void max1187x_block_touch(struct data *ts, int enable)
{
	mutex_lock(&max1187x_block_mutex);
	ts->hall_block_touch_event = enable;
	mutex_unlock(&max1187x_block_mutex);
	pr_info("%s: %d", __func__, ts->hall_block_touch_event);
}

static void max1187x_block_touch_work_func(struct work_struct *dummy)
{
	struct data *ts = gl_ts;
	max1187x_block_touch(ts, 0);
}
static DECLARE_DELAYED_WORK(max1187x_block_touch_work, max1187x_block_touch_work_func);

static void max1187x_handle_block_touch(struct data *ts, int enable)
{
	int ret;
	if (ts->hall_block_touch_event) {
		ret = cancel_delayed_work(&max1187x_block_touch_work);
		max1187x_block_touch(ts, 0);
	}
	if (enable) {
		pr_info("%s: %d", __func__, PDATA(hall_block_touch_time));
		ret = schedule_delayed_work(&max1187x_block_touch_work, HZ*PDATA(hall_block_touch_time)/1000);
		max1187x_block_touch(ts, 1);
	}
}
#endif

/* I2C communication */
/* debug_mask |= 0x10000 for I2C RX communication */
static int i2c_rx_bytes(struct data *ts, u8 *buf, u16 len)
{
	int i, ret, written;

	do {
		ret = i2c_master_recv(ts->client, (char *) buf, (int) len);
	} while (ret == -EAGAIN);
	if (ret < 0) {
		pr_err("I2C RX fail (%d)", ret);
		return ret;
	}

	len = ret;

	if (debugmask_if(1)) {
		pr_info("I2C RX (%d):", len);
		written = 0;
		for (i = 0; i < len; i++) {
			written += snprintf(ts->debug_string + written, 6,
					"0x%02X,", buf[i]);
			if (written + 6 >= DEBUG_STRING_LEN_MAX) {
				pr_info("%s", ts->debug_string);
				written = 0;
			}
		}
		if (written > 0)
			pr_info("%s", ts->debug_string);
	}

	return len;
}

static int i2c_rx_words(struct data *ts, u16 *buf, u16 len)
{
	int i, ret, written;

	do {
		ret = i2c_master_recv(ts->client,
			(char *) buf, (int) (len * 2));
	} while (ret == -EAGAIN);
	if (ret < 0) {
		pr_err("I2C RX fail (%d)", ret);
		return ret;
	}

	if ((ret % 2) != 0) {
		pr_err("I2C words RX fail: odd number of bytes (%d)", ret);
		return -EIO;
	}

	len = ret/2;

#ifdef __BIG_ENDIAN
	for (i = 0; i < len; i++)
		buf[i] = (buf[i] << 8) | (buf[i] >> 8);
#endif
	if (debugmask_if(1)) {
		pr_info("I2C RX (%d):", len);
		written = 0;
		for (i = 0; i < len; i++) {
			written += snprintf(ts->debug_string + written,
					8, "0x%04X,", buf[i]);
			if (written + 8 >= DEBUG_STRING_LEN_MAX) {
				pr_info("%s", ts->debug_string);
				written = 0;
			}
		}
		if (written > 0)
			pr_info("%s", ts->debug_string);
	}

	return len;
}

/* debug_mask |= 0x20000 for I2C TX communication */
static int i2c_tx_bytes(struct data *ts, u8 *buf, u16 len)
{
	int i, ret, written;

	do {
		ret = i2c_master_send(ts->client, (char *) buf, (int) len);
	} while (ret == -EAGAIN);
	if (ret < 0) {
		pr_err("I2C TX fail (%d)", ret);
		return ret;
	}

	len = ret;

	if (debugmask_if(2)) {
		pr_info("I2C TX (%d):", len);
		written = 0;
		for (i = 0; i < len; i++) {
			written += snprintf(ts->debug_string + written, 6,
					"0x%02X,", buf[i]);
			if (written + 6 >= DEBUG_STRING_LEN_MAX) {
				pr_info("%s", ts->debug_string);
				written = 0;
			}
		}
		if (written > 0)
			pr_info("%s", ts->debug_string);
	}

	return len;
}

static int i2c_tx_words(struct data *ts, u16 *buf, u16 len)
{
	int i, ret, written;

#ifdef __BIG_ENDIAN
	for (i = 0; i < len; i++)
		buf[i] = (buf[i] << 8) | (buf[i] >> 8);
#endif
	do {
		ret = i2c_master_send(ts->client,
			(char *) buf, (int) (len * 2));
	} while (ret == -EAGAIN);
	if (ret < 0) {
		pr_err("I2C TX fail (%d)", ret);
		return ret;
	}
	if ((ret % 2) != 0) {
		pr_err("I2C words TX fail: odd number of bytes (%d)", ret);
		return -EIO;
	}

	len = ret/2;

	if (debugmask_if(2)) {
		pr_info("I2C TX (%d):", len);
		written = 0;
		for (i = 0; i < len; i++) {
			written += snprintf(ts->debug_string + written, 8,
					"0x%04X,", buf[i]);
			if (written + 8 >= DEBUG_STRING_LEN_MAX) {
				pr_info("%s", ts->debug_string);
				written = 0;
			}
		}
		if (written > 0)
			pr_info("%s", ts->debug_string);
	}

	return len;
}

/* Read report */
static int read_mtp_report(struct data *ts, u16 *buf)
{
	int words = 1, words_tx, words_rx;
	int ret = 0, remainder = 0, offset = 0;
	u16 address = 0x000A;

	mutex_lock(&ts->i2c_mutex);
	/* read header, get size, read entire report */
	words_tx = i2c_tx_words(ts, &address, 1);
	if (words_tx != 1) {
		mutex_unlock(&ts->i2c_mutex);
		pr_err("Report RX fail: failed to set address");
		return -EIO;
	}

	if (ts->is_raw_mode == 0) {
		words_rx = i2c_rx_words(ts, buf, 2);
		if (words_rx != 2 || BYTEL(buf[0]) > MAX_WORDS_REPORT) {
			ret = -EIO;
			pr_err("Report RX fail: received (%d) " \
					"expected (%d) words, " \
					"header (%04X)",
					words_rx, words, buf[0]);
			mutex_unlock(&ts->i2c_mutex);
			return ret;
		}

		words = BYTEL(buf[0]) + 1;
		words_tx = i2c_tx_words(ts, &address, 1);
		if (words_tx != 1) {
			mutex_unlock(&ts->i2c_mutex);
			pr_err("Report RX fail:" \
				"failed to set address");
			return -EIO;
		}

		words_rx = i2c_rx_words(ts, &buf[offset], words);
		if (words_rx != words) {
			mutex_unlock(&ts->i2c_mutex);
			pr_err("Report RX fail 0x%X: received (%d) " \
				"expected (%d) words",
				address, words_rx, remainder);
			return -EIO;

		}
	} else {
		words_rx = i2c_rx_words(ts, buf,
				(u16) PDATA(i2c_words));
		if (words_rx != (u16) PDATA(i2c_words) || BYTEL(buf[0])
				> MAX_WORDS_REPORT) {
			ret = -EIO;
			pr_err("Report RX fail: received (%d) " \
				"expected (%d) words, header (%04X)",
				words_rx, words, buf[0]);
			mutex_unlock(&ts->i2c_mutex);
			return ret;
		}

		words = BYTEL(buf[0]) + 1;
		remainder = words;

		if (remainder - (u16) PDATA(i2c_words) > 0) {
			remainder -= (u16) PDATA(i2c_words);
			offset += (u16) PDATA(i2c_words);
			address += (u16) PDATA(i2c_words);
		}

		words_tx = i2c_tx_words(ts, &address, 1);
		if (words_tx != 1) {
			mutex_unlock(&ts->i2c_mutex);
			pr_err("Report RX fail: failed to set " \
				"address 0x%X", address);
			return -EIO;
		}

		words_rx = i2c_rx_words(ts, &buf[offset], remainder);
		if (words_rx != remainder) {
			mutex_unlock(&ts->i2c_mutex);
			pr_err("Report RX fail 0x%X: received (%d) " \
					"expected (%d) words",
					address, words_rx, remainder);
			return -EIO;
		}
	}

	mutex_unlock(&ts->i2c_mutex);
	return ret;
}

/* Send command */
static int send_mtp_command(struct data *ts, u16 *buf, u16 len)
{
	u16 tx_buf[MAX_WORDS_COMMAND + 2]; /* with address and header */
	u16 packets, words, words_tx;
	int i, ret = 0;

	/* check basics */
	if (len < 2) {
		pr_err("Command too short (%d); 2 words minimum", len);
		return -EINVAL;
	}
	if ((buf[1] + 2) != len) {
		pr_err("Inconsistent command length: " \
				"expected (%d) given (%d)", (buf[1] + 2), len);
		return -EINVAL;
	}

	if (len > MAX_WORDS_COMMAND_ALL) {
		pr_err("Command too long (%d); maximum (%d) words",
				len, MAX_WORDS_COMMAND_ALL);
		return -EINVAL;
	}

	/* packetize and send */
	packets = len / MAX_WORDS_COMMAND;
	if (len % MAX_WORDS_COMMAND)
		packets++;
	tx_buf[0] = 0x0000;

	mutex_lock(&ts->i2c_mutex);
	for (i = 0; i < packets; i++) {
		words = (i == (packets - 1)) ? len : MAX_WORDS_COMMAND;
		tx_buf[1] = (packets << 12) | ((i + 1) << 8) | words;
		memcpy(&tx_buf[2], &buf[i * MAX_WORDS_COMMAND],
			BYTE_SIZE(words));
		words_tx = i2c_tx_words(ts, tx_buf, words + 2);
		if (words_tx != (words + 2)) {
			ret = -1;
			pr_err("Command TX fail: transmitted (%d) " \
				"expected (%d) words, packet (%d)",
				words_tx, words + 2, i);
		}
		len -= MAX_WORDS_COMMAND;
	}
	ts->got_report = 0;
	mutex_unlock(&ts->i2c_mutex);

	return ret;
}

/* Integer math operations */
#if 0
/* Returns index of element in array closest to val */
static u16 binary_search(const u16 *array, u16 len, u16 val)
{
	s16 lt, rt, mid;
	if (len < 2)
		return 0;

	lt = 0;
	rt = len - 1;

	while (lt <= rt) {
		mid = (lt + rt)/2;
		if (val == array[mid])
			return mid;
		if (val < array[mid])
			rt = mid - 1;
		else
			lt = mid + 1;
	}

	if (lt >= len)
		return len - 1;
	if (rt < 0)
		return 0;
	if (array[lt] - val > val - array[lt-1])
		return lt-1;
	else
		return lt;
}

/* Given values of x and y, it calculates the orientation
 * with respect to y axis by calculating atan(x/y)
 */
static s16 max1187x_orientation(s16 x, s16 y)
{
	u16 sign = 0;
	u16 len = sizeof(tanlist)/sizeof(tanlist[0]);
	u32 quotient;
	s16 angle;

	if (x == y) {
		angle = 45;
		return angle;
	}
	if (x == 0) {
		angle = 0;
		return angle;
	}
	if (y == 0) {
		if (x > 0)
			angle = 90;
		else
			angle = -90;
		return angle;
	}

	if (x < 0) {
		sign = ~sign;
		x = -x;
	}
	if (y < 0) {
		sign = ~sign;
		y = -y;
	}

	if (x == y)
		angle = 45;
	else if (x < y) {
		quotient = ((u32)x << 16) - (u32)x;
		quotient = quotient / y;
		angle = binary_search(tanlist, len, quotient);
	} else {
		quotient = ((u32)y << 16) - (u32)y;
		quotient = quotient / x;
		angle = binary_search(tanlist, len, quotient);
		angle = 90 - angle;
	}
	if (sign == 0)
		return angle;
	else
		return -angle;
}

u16 max1187x_sqrt(u32 num)
{
	u16 mask = 0x8000;
	u16 guess = 0;
	u32 prod = 0;

	if (num < 2)
		return num;

	while (mask) {
		guess = guess ^ mask;
		prod = guess*guess;
		if (num < prod)
			guess = guess ^ mask;
		mask = mask>>1;
	}
	if (guess != 0xFFFF) {
		prod = guess*guess;
		if ((num - prod) > (prod + 2*guess + 1 - num))
			guess++;
	}

	return guess;
}
#endif

static void button_report(struct data *ts, int index, int state)
{
	if (!ts->button_data)
		return;

	if (state) {
		switch (PDATA(input_protocol)) {
			case MAX1187X_PROTOCOL_A:
				if (PDATA(support_htc_event)) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
						(3000 << 16) | 0x0A);
					input_report_abs(ts->input_dev, ABS_MT_POSITION,
						(1 << 31) | (ts->button_data[index].x_position << 16) | ts->button_data[index].y_position);
#endif
				}
				input_report_abs(ts->input_dev,
					ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_X, ts->button_data[index].x_position);
				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_Y, ts->button_data[index].y_position);
				input_report_abs(ts->input_dev,
					ABS_MT_PRESSURE, 3000);
				if (PDATA(report_mode) == MAX1187X_REPORT_MODE_EXTEND)
					input_report_abs(ts->input_dev,
								ABS_MT_WIDTH_MAJOR, 5);
				input_sync(ts->input_dev);
				break;
			case MAX1187X_PROTOCOL_B:
				if (PDATA(support_htc_event)) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
						(3000 << 16) | 0x0A);
					input_report_abs(ts->input_dev, ABS_MT_POSITION,
						(1 << 31) | (ts->button_data[index].x_position << 16) | ts->button_data[index].y_position);
#endif
				}
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev,
					MT_TOOL_FINGER, 1);
				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_X, ts->button_data[index].x_position);
				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_Y, ts->button_data[index].y_position);
				input_report_abs(ts->input_dev,
					ABS_MT_PRESSURE, 3000);
				if (PDATA(report_mode) == MAX1187X_REPORT_MODE_EXTEND)
					input_report_abs(ts->input_dev,
								ABS_MT_WIDTH_MAJOR, 5);
				input_sync(ts->input_dev);
				break;
			case MAX1187X_PROTOCOL_CUSTOM1:
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
					(3000 << 16) | 0x0A);
				input_report_abs(ts->input_dev, ABS_MT_POSITION,
					(1 << 31) | (ts->button_data[index].x_position << 16) | ts->button_data[index].y_position);
#endif
				break;
		}
	} else {
		switch (PDATA(input_protocol)) {
			case MAX1187X_PROTOCOL_A:
				if (PDATA(support_htc_event)) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif
				}
				input_mt_sync(ts->input_dev);
				input_sync(ts->input_dev);
				break;
			case MAX1187X_PROTOCOL_B:
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
				if (PDATA(support_htc_event)) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif
				}
				input_sync(ts->input_dev);
				break;
			case MAX1187X_PROTOCOL_CUSTOM1:
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif
				break;
			default:
				break;
		}
	}
}

/* debug_mask |= 0x40000 for touch reports */
static void process_touch_report(struct data *ts, u16 *buf)
{
	u32 i;
	u16 x, y, swap_u16;
	u8 state[10] = {0};
	u16 glove_status = 0, finger_status;
	u8 pen_status = 0;
#if 0
	u32 area;
	u32 major_axis, minor_axis;
	s16 xsize, ysize, orientation, swap_s16;
#endif

	struct max1187x_touch_report_header *header;
	struct max1187x_touch_report_basic *reportb;
	struct max1187x_touch_report_extended *reporte;

	pr_dbg(1, "Touch:");
		for (i = 0; i < (buf[0]&0x00FF); i++)
			pr_dbg(1, " %04x", buf[i]);
	pr_dbg(1, "\n");

	header = (struct max1187x_touch_report_header *) buf;

	if (!ts->input_dev)
		goto err_process_touch_report_inputdev;
	if (BYTEH(header->header) != 0x11)
		goto err_process_touch_report_header;

	if (header->report_id != MAX1187X_TOUCH_REPORT_BASIC &&
			header->report_id != MAX1187X_TOUCH_REPORT_EXTENDED) {
		if (header->report_id == 0x0134)
			pr_info("Reset Baseline, Framecount number = %04x", buf[3]);
		if (header->report_id == 0x01A0) {
			pr_info("System status report: %04X", buf[3]);
			system_status_handler(ts);
		}
		goto err_process_touch_report_reportid;
	}

	if (ts->framecounter == header->framecounter) {
		pr_info("Same framecounter (%u) encountered at irq (%u)!\n",
				ts->framecounter, ts->irq_count);
		goto err_process_touch_report_framecounter;
	}
	ts->framecounter = header->framecounter;

	if (!ts->finger_press) {
		ts->finger_log = 0;
		if (header->button0 != ts->button0) {
			if (header->button0) {
				ts->vk_press = 1;
				pr_info("%s key pressed", keycode_check(PDATA(button_code0)));
			} else {
				ts->vk_press = 0;
				pr_info("%s key released", keycode_check(PDATA(button_code0)));
			}
			if (ts->button_data)
				button_report(ts, 0, header->button0);
			else {
				input_report_key(ts->input_dev, PDATA(button_code0), header->button0);
				input_sync(ts->input_dev);
			}
			ts->button0 = header->button0;
		}
		if (header->button1 != ts->button1) {
			if (header->button1) {
				ts->vk_press = 1;
				pr_info("%s key pressed", keycode_check(PDATA(button_code1)));
			} else {
				ts->vk_press = 0;
				pr_info("%s key released", keycode_check(PDATA(button_code1)));
			}
			if (ts->button_data)
				button_report(ts, 1, header->button1);
			else {
				input_report_key(ts->input_dev, PDATA(button_code1), header->button1);
				input_sync(ts->input_dev);
			}
			ts->button1 = header->button1;
		}
		if (header->button2 != ts->button2) {
			if (header->button2) {
				ts->vk_press = 1;
				pr_info("%s key pressed", keycode_check(PDATA(button_code2)));
			} else {
				ts->vk_press = 0;
				pr_info("%s key released", keycode_check(PDATA(button_code2)));
			}
			if (ts->button_data)
				button_report(ts, 2, header->button2);
			else {
				input_report_key(ts->input_dev, PDATA(button_code2), header->button2);
				input_sync(ts->input_dev);
			}
			ts->button2 = header->button2;
		}
		if (header->button3 != ts->button3) {
			if (header->button3) {
				ts->vk_press = 1;
				pr_info("%s key pressed", keycode_check(PDATA(button_code3)));
			} else {
				ts->vk_press = 0;
				pr_info("%s key released", keycode_check(PDATA(button_code3)));
			}
			if (ts->button_data)
				button_report(ts, 3, header->button3);
			else {
				input_report_key(ts->input_dev, PDATA(button_code3), header->button3);
				input_sync(ts->input_dev);
			}
			ts->button3 = header->button3;
		}
	} else if ((header->button0 | header->button1 | header->button2 | header->button3) && !ts->finger_log) {
		pr_info("Finger pressed! Ignore vkey press event.");
		ts->finger_log = 1;
	} else if (!(header->button0 | header->button1 | header->button2 | header->button3) && ts->finger_log) {
		pr_info("Finger pressed! Ignore vkey release event.");
		ts->finger_log = 0;
	}

	if (header->touch_count > 10) {
		pr_err("Touch count (%u) out of bounds [0,10]!",
				header->touch_count);
		goto err_process_touch_report_touchcount;
	}

	if(header->touch_status != ts->noise_level) {
		pr_info("Noise level %d -> %d", ts->noise_level, header->touch_status);
		ts->noise_level = (u8)header->touch_status;
	}

	if(header->cycles != ts->cycles) {
		pr_info("Cycles: %d -> %d", (ts->cycles == 1)? 32 : 16, (header->cycles == 1)? 32 : 16);
		ts->cycles = header->cycles;
	}

	if (header->touch_count == 0) {
		if (!ts->finger_press && ts->vk_press) {
			return;
		}
		for (i = 0; i < MAX1187X_TOUCH_COUNT_MAX; i++) {
			if (ts->report_points[i].state==1 && state[i]==0) {
				if (PDATA(input_protocol) == MAX1187X_PROTOCOL_B) {
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
					if (ts->glove_status) {
						input_report_abs(ts->input_dev, ABS_MT_GLOVE, 1);
						glove_status = 0;
					} else {
						input_report_abs(ts->input_dev, ABS_MT_GLOVE, 0);
						pen_status = 0;
					}
				}
				ts->report_points[i].state = 0;
				if (debug_mask & BIT(3)) {
					if (ts->glove_status)
						finger_status = MAX1187X_GLOVE_MODE;
					else if (ts->pen_status)
						finger_status = MAX1187X_PEN_MODE;
					else
						finger_status = MAX1187X_FINGER_MODE;
					if(ts->width_factor && ts->height_factor) {
						pr_dbg(8, "Screen:%c[%02d]:Up, X=%d, Y=%d, Z=%d, W=%d",
							state2char(finger_status),
							i+1, ((ts->report_points[i].x-ts->width_offset)*ts->width_factor)>>SHIFT_BITS,
							((ts->report_points[i].y-ts->height_offset)*ts->height_factor)>>SHIFT_BITS,
							ts->report_points[i].z, ts->report_points[i].w);
					}
					else {
						pr_dbg(8, "Raw:%c[%02d]:Up, X=%d, Y=%d, Z=%d, W=%d",
							state2char(finger_status),
							i+1, ts->report_points[i].x, ts->report_points[i].y,
							ts->report_points[i].z, ts->report_points[i].w);
					}
				}
			}
		}
		switch (PDATA(input_protocol)) {
			case MAX1187X_PROTOCOL_A:
				if (PDATA(support_htc_event)) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif
				}
				input_mt_sync(ts->input_dev);
				input_sync(ts->input_dev);
				break;
			case MAX1187X_PROTOCOL_B:
				if (PDATA(support_htc_event)) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif
				}
				input_sync(ts->input_dev);
				break;
			case MAX1187X_PROTOCOL_CUSTOM1:
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif
				break;
			default:
				break;
		}
		pr_info_if(4, "(TOUCH): Fingers up, Frame(%d) Noise(%d) Cycles(%d)",
			ts->framecounter, ts->noise_level, (ts->cycles == 1)? 32 : 16);
		pr_dbg(2, "Finger leave, Noise:%d, Cycles:%d", ts->noise_level, (ts->cycles == 1)? 32 : 16);
	} else {
		if (ts->vk_press) {
			//pr_info("Vkey pressed! Ignore finger event.");
			return;
		}
		reportb = (struct max1187x_touch_report_basic *)
				((u8 *)buf + sizeof(*header));
		reporte = (struct max1187x_touch_report_extended *)
				((u8 *)buf + sizeof(*header));
		for (i = 0; i < header->touch_count; i++) {
			x = reportb->x;
			y = reportb->y;
			if (PDATA(coordinate_settings) & MAX1187X_SWAP_XY) {
				swap_u16 = x;
				x = y;
				y = swap_u16;
			}
			if (PDATA(coordinate_settings) & MAX1187X_REVERSE_X) {
				x = PDATA(panel_max_x) + PDATA(panel_min_x) - x;
			}
			if (PDATA(coordinate_settings) & MAX1187X_REVERSE_Y) {
				y = PDATA(panel_max_y) + PDATA(panel_min_y) - y;
			}
			if (reportb->z == 0)
				reportb->z++;

			if (reportb->finger_status == MAX1187X_GLOVE_MODE) {
				glove_status |= 1 << reportb->finger_id;
			}
			else if (reportb->finger_status == MAX1187X_PEN_MODE) {
				pen_status = 1;
			} else
				pen_status = 0;

			pr_info_if(4, "(TOUCH): (%u) Finger %u: "\
				"X(%d) Y(%d) Z(%d) Frame(%d) Noise(%d) Finger status(%X) Cycles(%d)",
				header->framecounter, reportb->finger_id,
				x, y, reportb->z, ts->framecounter, ts->noise_level, reportb->finger_status,
				(ts->cycles == 1)? 32 : 16);
			pr_dbg(2, "Finger %d=> X:%d, Y:%d, Z:%d, Noise:%d, Cycles:%d",
				reportb->finger_id+1, x, y, reportb->z, ts->noise_level, (ts->cycles == 1)? 32 : 16);
		if (ts->hall_block_touch_event == 0) {
			switch (PDATA(input_protocol)) {
				case MAX1187X_PROTOCOL_A:
					if (PDATA(support_htc_event)) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
						input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
							(reportb->z << 16) | 0x0A);
						input_report_abs(ts->input_dev, ABS_MT_POSITION,
							((i == (header->touch_count - 1)) << 31) | (x << 16) | y);
#endif
					}
					input_report_abs(ts->input_dev,
						ABS_MT_TRACKING_ID, reportb->finger_id);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
					input_report_abs(ts->input_dev,	ABS_MT_POSITION_Y, y);
					input_report_abs(ts->input_dev,
						ABS_MT_PRESSURE, reportb->z);
					break;
				case MAX1187X_PROTOCOL_B:
					if (PDATA(support_htc_event)) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
						input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
							(reportb->z << 16) | 0x0A);
						input_report_abs(ts->input_dev, ABS_MT_POSITION,
							((i == (header->touch_count - 1)) << 31) | (x << 16) | y);
#endif
					}
					input_mt_slot(ts->input_dev, reportb->finger_id);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
					if (reportb->finger_status == MAX1187X_GLOVE_MODE) {
						input_report_abs(ts->input_dev, ABS_MT_GLOVE, 1);
					} else {
						input_report_abs(ts->input_dev, ABS_MT_GLOVE, 0);
					}
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
					input_report_abs(ts->input_dev,	ABS_MT_POSITION_Y, y);
					input_report_abs(ts->input_dev,
						ABS_MT_PRESSURE, reportb->z);
					break;
				case MAX1187X_PROTOCOL_CUSTOM1:
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
						(reportb->z << 16) | 0x0A);
					input_report_abs(ts->input_dev, ABS_MT_POSITION,
						((i == (header->touch_count - 1)) << 31) | (x << 16) | y);
#endif
					break;
			}
		}
			ts->report_points[reportb->finger_id].x = x;
			ts->report_points[reportb->finger_id].y = y;
			ts->report_points[reportb->finger_id].z = reportb->z;
			state[reportb->finger_id] = 1;

			if (header->report_id
				== MAX1187X_TOUCH_REPORT_EXTENDED) {
				pr_info_if(4, "(TOUCH): speed(%d,%d), pixel(%d,%d), area(%d), x(%d,%d), y(%d,%d)",
					reporte->xspeed, reporte->yspeed, reporte->xpixel, reporte->ypixel, reporte->area,
					reporte->xmin, reporte->xmax, reporte->ymin, reporte->ymax);
		if (ts->hall_block_touch_event == 0) {
				switch (PDATA(input_protocol)) {
					case MAX1187X_PROTOCOL_A:
					case MAX1187X_PROTOCOL_B:
						input_report_abs(ts->input_dev,
								ABS_MT_WIDTH_MAJOR, reporte->area);
						break;
					default:
						break;
				}
		}
				ts->report_points[reportb->finger_id].w = reporte->area;
#if 0
				xsize = (reporte->xpixel - 1)
					* (s16)(PDATA(lcd_x)/PDATA(num_rows));
				ysize = (reporte->ypixel - 1)
					* (s16)(PDATA(lcd_y)/PDATA(num_cols));
				if (PDATA(coordinate_settings)
						& MAX1187X_REVERSE_X)
					xsize = -xsize;
				if (PDATA(coordinate_settings)
						& MAX1187X_REVERSE_Y)
					ysize = -ysize;
				if (PDATA(coordinate_settings)
						& MAX1187X_SWAP_XY) {
					swap_s16 = xsize;
					xsize = ysize;
					ysize = swap_s16;
				}
				/* Calculate orientation as
				 * arctan of xsize/ysize) */
				orientation =
					max1187x_orientation(xsize, ysize);
				area = reporte->area
					* (PDATA(lcd_x)/PDATA(num_rows))
					* (PDATA(lcd_y)/PDATA(num_cols));
				/* Major axis of ellipse if hypotenuse
				 * formed by xsize and ysize */
				major_axis = xsize*xsize + ysize*ysize;
				major_axis = max1187x_sqrt(major_axis);
				/* Minor axis can be reverse calculated
				 * using the area of ellipse:
				 * Area of ellipse =
				 *		pi / 4 * Major axis * Minor axis
				 * Minor axis =
				 *		4 * Area / (pi * Major axis)
				 */
				minor_axis = (2 * area) / major_axis;
				minor_axis = (minor_axis<<17) / MAX1187X_PI;
				pr_info_if(4, "(TOUCH): Finger %u: " \
					"Orientation(%d) Area(%u) Major_axis(%u) Minor_axis(%u)",
					reportb->finger_id,	orientation,
					area, major_axis, minor_axis);
				input_report_abs(ts->input_dev,
					ABS_MT_ORIENTATION, orientation);
				input_report_abs(ts->input_dev,
						ABS_MT_TOUCH_MAJOR, major_axis);
				input_report_abs(ts->input_dev,
						ABS_MT_TOUCH_MINOR, minor_axis);
#endif
				reporte++;
				reportb = (struct max1187x_touch_report_basic *)
						((u8 *) reporte);
			} else {
				reportb++;
			}

		if (ts->hall_block_touch_event == 0) {
			switch (PDATA(input_protocol)) {
				case MAX1187X_PROTOCOL_A:
					input_mt_sync(ts->input_dev);
					break;
				default:
					break;
			}
		}
		}
		for (i = 0; i < MAX1187X_TOUCH_COUNT_MAX; i++) {
			if (ts->report_points[i].state==1 && state[i]==0) {
				if (PDATA(input_protocol) == MAX1187X_PROTOCOL_B) {
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
					if (ts->glove_status & (1 << i)) {
						input_report_abs(ts->input_dev, ABS_MT_GLOVE, 1);
					} else {
						input_report_abs(ts->input_dev, ABS_MT_GLOVE, 0);
					}
				}
				ts->report_points[i].state = 0;
				if (debug_mask & BIT(3)) {
					if (ts->glove_status & (1 << i)) {
						finger_status = MAX1187X_GLOVE_MODE;
					}
					else if (ts->pen_status)
						finger_status = MAX1187X_PEN_MODE;
					else
						finger_status = MAX1187X_FINGER_MODE;
					if(ts->width_factor && ts->height_factor) {
						pr_dbg(8, "Screen:%c[%02d]:Up, X=%d, Y=%d, Z=%d, W=%d",
							state2char(finger_status),
							i+1, ((ts->report_points[i].x-ts->width_offset)*ts->width_factor)>>SHIFT_BITS,
							((ts->report_points[i].y-ts->height_offset)*ts->height_factor)>>SHIFT_BITS,
							ts->report_points[i].z, ts->report_points[i].w);
					}
					else {
						pr_dbg(8, "Raw:%c[%02d]:Up, X=%d, Y=%d, Z=%d, W=%d",
							state2char(finger_status),
							i+1, ts->report_points[i].x, ts->report_points[i].y,
							ts->report_points[i].z, ts->report_points[i].w);
					}
				}
			}
			else if (ts->report_points[i].state ==0 && state[i]==1) {
				ts->report_points[i].state = 1;
				if (debug_mask & BIT(3)) {
					if (glove_status & (1 << i)) {
						finger_status = MAX1187X_GLOVE_MODE;
					}
					else if (pen_status)
						finger_status = MAX1187X_PEN_MODE;
					else
						finger_status = MAX1187X_FINGER_MODE;
					if (ts->width_factor && ts->height_factor) {
						pr_dbg(8, "Screen:%c[%02d]:Down, X=%d, Y=%d, Z=%d, W=%d",
							state2char(finger_status),
							i+1, ((ts->report_points[i].x-ts->width_offset)*ts->width_factor)>>SHIFT_BITS,
							((ts->report_points[i].y-ts->height_offset)*ts->height_factor)>>SHIFT_BITS,
							ts->report_points[i].z, ts->report_points[i].w);
					}
					else {
						pr_dbg(8, "Raw:%c[%02d]:Down, X=%d, Y=%d, Z=%d, W=%d",
							state2char(finger_status),
							i+1, ts->report_points[i].x, ts->report_points[i].y,
							ts->report_points[i].z, ts->report_points[i].w);
					}
				}
			}
		}
		switch (PDATA(input_protocol) ) {
			case MAX1187X_PROTOCOL_A:
			case MAX1187X_PROTOCOL_B:
				input_sync(ts->input_dev);
				break;
			case MAX1187X_PROTOCOL_CUSTOM1:
				break;
		}
	}
	ts->glove_status = glove_status;
	ts->pen_status = pen_status;
	ts->finger_press = header->touch_count;
err_process_touch_report_touchcount:
err_process_touch_report_inputdev:
err_process_touch_report_header:
err_process_touch_report_reportid:
err_process_touch_report_framecounter:
	return;
}

static irqreturn_t irq_handler(int irq, void *context)
{
	struct data *ts = (struct data *) context;
	int read_retval;
	u64	time_elapsed = jiffies;
	struct timespec time_start, time_end, time_delta;

	if (atomic_read(&ts->scheduled_work_irq) != 0)
		return IRQ_HANDLED;

	if (gpio_get_value(ts->pdata->gpio_tirq) != 0)
		return IRQ_HANDLED;

	/* disable_irq_nosync(ts->client->irq); */
	atomic_inc(&ts->scheduled_work_irq);
	ts->irq_receive_time = jiffies;
	ts->irq_count++;

	if (debug_mask & BIT(2)) {
		getnstimeofday(&time_start);
	}

	read_retval = read_mtp_report(ts, ts->rx_packet);

	if (time_elapsed >= ts->irq_receive_time)
		time_elapsed = time_elapsed - ts->irq_receive_time;
	else
		time_elapsed = time_elapsed +
					0x100000000 - ts->irq_receive_time;

	if (read_retval == 0 || time_elapsed > 2 * HZ) {
		process_touch_report(ts, ts->rx_packet);
		if (debug_mask & BIT(2)) {
			getnstimeofday(&time_end);
			time_delta.tv_nsec = (time_end.tv_sec*1000000000+time_end.tv_nsec)
				-(time_start.tv_sec*1000000000+time_start.tv_nsec);
			pr_dbg(4, "Touch latency = %ld us", time_delta.tv_nsec/1000);
		}
		propagate_report(ts, 0, ts->rx_packet);
	}
	atomic_dec(&ts->scheduled_work_irq);
	/* enable_irq(ts->client->irq); */
	return IRQ_HANDLED;
}

static ssize_t init_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", init_state);
}

static ssize_t init_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int value, ret;

	if (sscanf(buf, "%d", &value) != 1) {
		pr_err("bad parameter");
		return -EINVAL;
	}
	switch (value) {
	case 0:
		if (init_state == 0)
			break;
		ret = device_deinit(to_i2c_client(dev));
		if (ret != 0) {
			pr_err("deinit error (%d)", ret);
			return ret;
		}
		break;
	case 1:
		if (init_state == 1)
			break;
		ret = device_init(to_i2c_client(dev));
		if (ret != 0) {
			pr_err("init error (%d)", ret);
			return ret;
		}
		break;
	case 2:
		if (init_state == 1) {
			ret = device_deinit(to_i2c_client(dev));
			if (ret != 0) {
				pr_err("deinit error (%d)", ret);
				return ret;
			}
		}
		ret = device_init(to_i2c_client(dev));
		if (ret != 0) {
			pr_err("init error (%d)", ret);
			return ret;
		}
		break;
	default:
		pr_err("bad value");
		return -EINVAL;
	}

	return count;
}

static ssize_t hreset_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);
	struct max1187x_pdata *pdata = client->dev.platform_data;

	if (!pdata->gpio_reset)
		return count;

	DISABLE_IRQ();
	mutex_lock(&ts->i2c_mutex);
	gpio_set_value(pdata->gpio_reset, 0);
	usleep_range(10000, 11000);
	gpio_set_value(pdata->gpio_reset, 1);
	bootloader = 0;
	ts->got_report = 0;
	mutex_unlock(&ts->i2c_mutex);
	if (get_report(ts, 0x01A0, 3000) != 0) {
		pr_err("Failed to receive system status report");
		return count;
	}
	release_report(ts);

	return count;
}

static ssize_t sreset_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	DISABLE_IRQ();
	if (sreset(client) != 0) {
		pr_err("Failed to do soft reset.");
		return count;
	}
	if (get_report(ts, 0x01A0, 3000) != 0) {
		pr_err("Failed to receive system status report");
		return count;
	}

	release_report(ts);
	return count;
}

static ssize_t reflash_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);
	int ret, i, value;
	const struct firmware *fw;
	u16 chip_id;

	if (sscanf(buf, "%d", &value) != 1) {
		pr_err("bad parameter");
		return -EINVAL;
	}

	if (value == 1) {
		chip_id = ts->have_fw ? BYTEH(ts->fw_version[3]) : \
				PDATA(default_chip_id);

		for (i = 0; i < PDATA(num_fw_mappings); i++) {
			if (PDATA(fw_mapping[i]).chip_id == chip_id)
				break;
		}

		if (i == PDATA(num_fw_mappings)) {
			pr_err("FW not found for chipID(0x%04X)", chip_id);
			goto err_request;
		}

		pr_info("Firmware file (%s)",
			PDATA(fw_mapping[i]).filename);

		ret = request_firmware(&fw, PDATA(fw_mapping[i]).filename,
						&ts->client->dev);

		if (ret || fw == NULL) {
			pr_info("firmware request failed (ret = %d, fwptr = %p)",
				ret, fw);
			goto err_request;
		}

		if (device_fw_load(ts, fw, i, 0)) {
			pr_err("firmware download failed");
			goto err_load;
		}

		pr_info("firmware download OK");
err_load:
		release_firmware(fw);
	}

err_request:
	return count;
}

static ssize_t irq_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%u\n", ts->irq_count);
}

static ssize_t irq_count_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	ts->irq_count = 0;
	return count;
}

static ssize_t dflt_cfg_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%u 0x%x 0x%x\n", PDATA(defaults_allow),
			PDATA(default_config_id), PDATA(default_chip_id));
}

static ssize_t dflt_cfg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	(void) sscanf(buf, "%u 0x%x 0x%x", &PDATA(defaults_allow),
			&PDATA(default_config_id), &PDATA(default_chip_id));
	return count;
}

static ssize_t panel_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%u %u %u %u %u %u\n",
			PDATA(panel_max_x), PDATA(panel_min_x),
			PDATA(panel_max_y), PDATA(panel_min_y),
			PDATA(lcd_x), PDATA(lcd_y));
}

static ssize_t panel_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	(void) sscanf(buf, "%u %u %u %u %u %u",
			&PDATA(panel_max_x), &PDATA(panel_min_x),
			&PDATA(panel_max_y), &PDATA(panel_min_y),
			&PDATA(lcd_x), &PDATA(lcd_y));
	return count;
}

static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);
	u16 build_number = 0;
	u8 branch = BYTEL(ts->fw_version[3]) >> 6;

	if (ts->fw_version[1] >= 3)
		build_number = ts->fw_version[4];
	return snprintf(
			buf,
			PAGE_SIZE,
			"%u.%u.%u p%u%c "
				"(CRC16 0x%04X=>0x%04X) Chip ID 0x%02X\n",
			BYTEH(ts->fw_version[2]),
			BYTEL(ts->fw_version[2]),
			build_number,
			BYTEL(ts->fw_version[3]) & 0x3F,
			(branch == 0) ? ' ' : (branch - 1 + 'a'),
			(ts->fw_index != -1) ? \
			PDATA(fw_mapping[ts->fw_index]).file_codesize \
			: 0, ts->fw_crc16, BYTEH(ts->fw_version[3]));
}

static ssize_t driver_ver_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "3.0.7.1: Mar 15, 2013\n");
}

static ssize_t debug_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%08X\n", debug_mask);
}

static ssize_t debug_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	if (sscanf(buf, "%ix", &debug_mask) != 1) {
		pr_err("bad parameter");
		return -EINVAL;
	}

	return count;
}

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);
	u16 buffer[MAX_WORDS_COMMAND_ALL];
	char scan_buf[5];
	int i;

	count--; /* ignore carriage return */
	if ((count % 4) != 0) {
		pr_err("words not properly defined");
		return -EINVAL;
	}
	scan_buf[4] = '\0';
	for (i = 0; i < count; i += 4) {
		memcpy(scan_buf, &buf[i], 4);
		if (sscanf(scan_buf, "%hx", &buffer[i / 4]) != 1) {
			pr_err("bad word (%s)", scan_buf);
			return -EINVAL;
		}

	}
	if (send_mtp_command(ts, buffer, count / 4))
		pr_err("MTP command failed");
	else {
		if (buffer[0] == 0x0018)
			ts->is_raw_mode = !buffer[2];
	}
	return ++count;
}

static ssize_t report_read(struct file *file, struct kobject *kobj,
	struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	struct data *ts = i2c_get_clientdata(client);
	int printed, i, offset = 0, payload;
	int full_packet;
	int num_term_char;

	if (get_report(ts, 0xFFFF, 0xFFFFFFFF))
		return 0;

	payload = ts->rx_report_len;
	full_packet = payload;
	num_term_char = 2; /* number of term char */
	if (count < (4 * full_packet + num_term_char))
		return -EIO;
	if (count > (4 * full_packet + num_term_char))
		count = 4 * full_packet + num_term_char;

	for (i = 1; i <= payload; i++) {
		printed = snprintf(&buf[offset], PAGE_SIZE, "%04X\n",
			ts->rx_report[i]);
		if (printed <= 0)
			return -EIO;
		offset += printed - 1;
	}
	snprintf(&buf[offset], PAGE_SIZE, ",\n");
	release_report(ts);

	return count;
}

static ssize_t touch_vendor_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct data *ts = gl_ts;

	return snprintf(buf, PAGE_SIZE, "Maxim-%s_p%u_chipID-0x%02X_twID-%02X\n",
		ts->fw_ver, ts->protocol_ver, BYTEH(ts->fw_version[3]), ts->vendor_pin);
}

static ssize_t config_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct data *ts = gl_ts;
	int i, ret;
	size_t count = 0;
	u16 mtpdata[]={0x0000, 0x0000, 0x0000};

	for(i=0; i<RETRY_TIMES; i++) {
		DISABLE_IRQ();
		//Get touch configuration
		ret = get_touch_config(ts->client);
		if (ret < 0)
			pr_info("[W] Failed to retrieve touch config");
		if (ret == 0) {
			ret = get_report(ts, 0x0102, 150);
			if (ret != 0)
				pr_info("[W] Get touch config time out-%d, retry", i);
			if (ret == 0) {
				count += snprintf(buf + count, PAGE_SIZE, "Touch config:\n");
				for (i = 3; i < config_num[ts->config_protocol][0]+3; i++) {
					count += snprintf(buf + count, PAGE_SIZE, "%04X ", ts->rx_report[i]);
					if (((i-3) % 16) == (16 - 1))
						count += snprintf(buf + count, PAGE_SIZE, "\n");
				}
				count += snprintf(buf + count, PAGE_SIZE, "\n");
				release_report(ts);
				break;
			}
		}
	}
	if (i==RETRY_TIMES && ret!=0)
		pr_err("Failed to receive touch config report");

	for(i=0; i<RETRY_TIMES; i++) {
		DISABLE_IRQ();
		//Get calibration table
		mtpdata[0]=0x0011;
		mtpdata[1]=0x0000;
		ret = send_mtp_command(ts, mtpdata, 2);
		if (ret < 0)
			pr_info("[W] Failed to retrieve calibration table");
		if (ret == 0) {
			ret = get_report(ts, 0x0111, 150);
			if (ret != 0)
				pr_info("[W] Get calibration table time out-%d, retry", i);
			if (ret == 0) {
				count += snprintf(buf + count, PAGE_SIZE, "Calibration Table:\n");
				for (i = 3; i < config_num[ts->config_protocol][1]+3; i++) {
					count += snprintf(buf + count, PAGE_SIZE, "%04X ", ts->rx_report[i]);
					if (((i-3) % 16) == (16 - 1))
						count += snprintf(buf + count, PAGE_SIZE, "\n");
				}
				count += snprintf(buf + count, PAGE_SIZE, "\n");
				release_report(ts);
				break;
			}
		}
	}
	if (i==RETRY_TIMES && ret!=0)
		pr_err("Failed to receive calibration table report");

	for(i=0; i<RETRY_TIMES; i++) {
		DISABLE_IRQ();
		//Get private configuration
		mtpdata[0]=0x0004;
		mtpdata[1]=0x0000;
		ret = send_mtp_command(ts, mtpdata, 2);
		if (ret < 0)
			pr_info("[W] Failed to retrieve private config");
		if (ret == 0) {
			ret = get_report(ts, 0x0104, 150);
			if (ret != 0)
				pr_info("[W] Get private config time out-%d, retry", i);
			if (ret == 0) {
				count += snprintf(buf + count, PAGE_SIZE, "Private Config:\n");
				for (i = 3; i < config_num[ts->config_protocol][2]+3; i++) {
					count += snprintf(buf + count, PAGE_SIZE, "%04X ", ts->rx_report[i]);
					if (((i-3) % 16) == (16 - 1))
						count += snprintf(buf + count, PAGE_SIZE, "\n");
				}
				count += snprintf(buf + count, PAGE_SIZE, "\n");
				release_report(ts);
				break;
			}
		}
	}
	if (i==RETRY_TIMES && ret!=0)
		pr_err("Failed to receive private config report");

	for(i=0; i<RETRY_TIMES; i++) {
		DISABLE_IRQ();
		//Get Lookup table X
		mtpdata[0]=0x0031;
		mtpdata[1]=0x0001;
		mtpdata[2]=0x0000;
		ret = send_mtp_command(ts, mtpdata, 3);
		if (ret < 0)
			pr_info("[W] Failed to retrieve Lookup table X");
		if (ret == 0) {
			ret = get_report(ts, 0x0131, 150);
			if (ret != 0)
				pr_info("[W] Get Lookup table X time out-%d, retry", i);
			if (ret == 0) {
				count += snprintf(buf + count, PAGE_SIZE, "Lookup Table X:\n");
				for (i = 3; i < config_num[ts->config_protocol][3]+3; i++) {
					count += snprintf(buf + count, PAGE_SIZE, "%04X ", ts->rx_report[i]);
					if (((i-3) % 16) == (16 - 1))
						count += snprintf(buf + count, PAGE_SIZE, "\n");
				}
				count += snprintf(buf + count, PAGE_SIZE, "\n");
				release_report(ts);
				break;
			}
		}
	}
	if (i==RETRY_TIMES && ret!=0)
		pr_err("Failed to receive Lookup table X report");

	for(i=0; i<RETRY_TIMES; i++) {
		DISABLE_IRQ();
		//Get Lookup table Y
		mtpdata[0]=0x0031;
		mtpdata[1]=0x0001;
		mtpdata[2]=0x0001;
		ret = send_mtp_command(ts, mtpdata, 3);
		if (ret < 0)
			pr_info("[W] Failed to retrieve Lookup table Y");
		if (ret == 0) {
			ret = get_report(ts, 0x0131, 150);
			if (ret != 0)
				pr_info("[W] Get Lookup table Y time out-%d, retry", i);
			if (ret == 0) {
				count += snprintf(buf + count, PAGE_SIZE, "Lookup Table Y:\n");
				for (i = 3; i < config_num[ts->config_protocol][3]+3; i++) {
					count += snprintf(buf + count, PAGE_SIZE, "%04X ", ts->rx_report[i]);
					if (((i-3) % 16) == (16 - 1))
						count += snprintf(buf + count, PAGE_SIZE, "\n");
				}
				count += snprintf(buf + count, PAGE_SIZE, "\n");
				release_report(ts);
				break;
			}
		}
	}
	if (i==RETRY_TIMES && ret!=0)
		pr_err("Failed to receive Lookup table Y report");

	return count;
}

static ssize_t gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct data *ts = gl_ts;
	struct max1187x_pdata *pdata = ts->client->dev.platform_data;

	if (!pdata->gpio_tirq)
		return ret;

	ret = gpio_get_value(pdata->gpio_tirq);
	printk(KERN_DEBUG "[TP] GPIO_TP_INT_N=%d\n", ret);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct data *ts = gl_ts;
	size_t count = 0;
	uint16_t i, j;
	int ret, button_count = 0;

	if (ts->baseline_mode != MAX1187X_AUTO_BASELINE)
		if (set_baseline_mode(ts->client, ts->baseline_mode) < 0) {
			pr_err("Failed to set up baseline mode");
			return -1;
		}
	if (set_touch_frame(ts->client, ts->frame_rate[1], 0x0A) < 0) {
		pr_err("Failed to set up frame rate");
		return -1;
	}

	DISABLE_IRQ();
	if (change_touch_rpt(ts->client, 0) < 0) {
		pr_err("Failed to set up raw data report");
		return -1;
	}
	ts->is_raw_mode = 1;
	ret = get_report(ts, 0x0800, 500);
	if (ret != 0)
		pr_info("Failed to receive raw data report");

	if (ret==0) {
		memcpy(ts->report, &ts->rx_report[5], BYTE_SIZE(ts->rx_report[2] - 2));
		if (ts->rx_report[2] - 2 > (ts->x_channel*ts->y_channel))
			button_count = ts->rx_report[2] - 2 - (ts->x_channel*ts->y_channel);
		count += sprintf(buf + count, "Channel: %dx%d\n", ts->x_channel, ts->y_channel);
		for (i = 0; i < ts->y_channel; i++) {
			for (j = 0; j < ts->x_channel; j++) {
				count += sprintf(buf + count, "%6d", ts->report[i*ts->x_channel + j]);
			}
			count += sprintf(buf + count, "\n");
		}
		if (button_count) {
			for (i = 0; i < button_count; i++) {
				count += sprintf(buf + count, "%6d", ts->report[ts->x_channel*ts->y_channel + i]);
			}
			count += sprintf(buf + count, "\n");
		}
		release_report(ts);
	}

	DISABLE_IRQ();
	ts->is_raw_mode = 0;
	if (change_touch_rpt(ts->client, 1) < 0) {
		pr_err("Failed to set up raw data report");
		return -1;
	}
	if (set_touch_frame(ts->client, ts->frame_rate[1], ts->frame_rate[0]) < 0) {
		pr_err("Failed to set up frame rate");
		return -1;
	}
	if (ts->baseline_mode != MAX1187X_AUTO_BASELINE)
		if (set_baseline_mode(ts->client, 2) < 0) {
			pr_err("Failed to set up baseline mode");
			return -1;
		}
	ENABLE_IRQ();

	return count;
}

static ssize_t diag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct data *ts = gl_ts;
	if (buf[0] == '1')
		ts->baseline_mode = MAX1187X_AUTO_BASELINE;
	else if (buf[0] == '2')
		ts->baseline_mode = MAX1187X_NO_BASELINE;
	else if (buf[0] == '3')
		ts->baseline_mode = MAX1187X_FIX_BASELINE;

	return count;
}

static ssize_t unlock_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int unlock = -1;
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		unlock = buf[0] - '0';

	pr_info("Touch: unlock change to %d", unlock);
	return count;
}

static ssize_t reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct data *ts = gl_ts;
	struct max1187x_pdata *pdata = ts->client->dev.platform_data;

	if (!pdata->gpio_reset)
		return count;

	DISABLE_IRQ();
	mutex_lock(&ts->i2c_mutex);
	gpio_set_value(pdata->gpio_reset, 0);
	usleep_range(10000, 11000);
	gpio_set_value(pdata->gpio_reset, 1);
	bootloader = 0;
	ts->got_report = 0;
	mutex_unlock(&ts->i2c_mutex);
	if (get_report(ts, 0x01A0, 3000) != 0) {
		pr_err("Failed to receive system status report");
		return count;
	}
	release_report(ts);

	return count;
}

static ssize_t cover_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct data *ts = gl_ts;
	size_t count = 0;

	if (ts->fw_config->support_cover) {
		count += snprintf(buf + count, PAGE_SIZE, "%d\n", ts->cover_mode);
	} else {
		count += snprintf(buf + count, PAGE_SIZE, "0\n");
	}

	return count;
}

static ssize_t cover_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct data *ts = gl_ts;
	int value;

	if (!ts->fw_config->support_cover)
		return count;

	if (sysfs_streq(buf, "0")) {
		value = 0;
	}
	else if (sysfs_streq(buf, "1")) {
		value = 1;
	}
	else {
		pr_info("%s: wrong parameter", __func__);
		return -EINVAL;
	}

	if (set_cover_mode(ts->client, value)) {
		pr_err("set cover mode error");
		return -EINVAL;
	}

	ts->cover_mode = value;
	pr_info("%s: cover_mode change to %d", __func__, ts->cover_mode);

	if (ts->fw_config->filter_range != NULL) {
		if (ts->cover_mode) {
			set_edge_filter(ts->client, 2, ts->fw_config->filter_range);
			pr_info("%s: enable filter", __func__);
		} else {
			set_edge_filter(ts->client, 0, NULL);
			pr_info("%s: disable filter", __func__);
		}
	}

	return count;
}

static ssize_t glove_setting_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct data *ts = gl_ts;
	size_t count = 0;

	if (ts->fw_config->support_glove) {
		count += snprintf(buf + count, PAGE_SIZE, "%d\n", ts->glove_setting);
	} else {
		count += snprintf(buf + count, PAGE_SIZE, "0\n");
	}

	return count;
}

static ssize_t glove_setting_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct data *ts = gl_ts;
	int value;

	if (!ts->fw_config->support_glove)
		return count;

	if (sscanf(buf, "%d", &value) != 1) {
		pr_err("bad parameter");
		return -EINVAL;
	}

	if (value > 3 || value < 0) {
		pr_info("%s: wrong parameter", __func__);
		return -EINVAL;
	}

	ts->glove_setting = value;
	pr_info("%s: glove_setting change to %d", __func__, ts->glove_setting);
	value = value & 0x01;

	if (ts->cover_mode) {
		pr_info("%s: Under cover mode, will not change glove_mode", __func__);
		return count;
	}

	if (!ts->i2c_to_mcu) {
		if (set_glove_mode(ts->client, value)) {
			pr_err("set glove mode error");
			return -EINVAL;
		}
	} else
		pr_info("%s: I2C already switch to MCU side", __func__);

	ts->glove_enable = value;
	pr_info("%s: glove_mode change to %d", __func__, ts->glove_enable);

	return count;
}

static ssize_t int_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct data *ts = gl_ts;
	size_t count = 0;

	count += snprintf(buf + count, PAGE_SIZE, "%d ", !ts->irq_disabled);
	count += snprintf(buf + count, PAGE_SIZE, "\n");

	return count;
}

static ssize_t int_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct data *ts = gl_ts;
	int value, ret=0;

	if (sysfs_streq(buf, "0"))
		value = false;
	else if (sysfs_streq(buf, "1"))
		value = true;
	else
		return -EINVAL;

	if (value) {
		ret = request_threaded_irq(ts->client->irq, NULL, irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, ts->client->name, ts);
		if (ret == 0) {
			ts->irq_disabled = 0;
			pr_info("%s: interrupt enable: %x\n", __func__, !ts->irq_disabled);
			if (ret)
				free_irq(ts->client->irq, ts);
		}
	} else {
		disable_irq(ts->client->irq);
		free_irq(ts->client->irq, ts);
		ts->irq_disabled = 1;
	}

	return count;
}

enum SR_REG_STATE{
	ALLOCATE_DEV_FAIL = -2,
	REGISTER_DEV_FAIL,
	SUCCESS,
};

static char *vk_name = "virtualkeys.sr_touchscreen";
static struct kobj_attribute vk_dev;

static int register_sr_touch_device(void)
{
	struct data *ts = gl_ts;
	struct max1187x_pdata *pdata = ts->client->dev.platform_data;
	int ret = 0;

	ts->sr_input_dev = input_allocate_device();

	if (ts->sr_input_dev == NULL) {
		printk(KERN_ERR "[TP][TOUCH_ERR]%s: Failed to allocate SR input device\n", __func__);
		return ALLOCATE_DEV_FAIL;
	}

	if (pdata->vk_obj) {
		memcpy(&vk_dev, pdata->vk2Use, sizeof(struct kobj_attribute));
		vk_dev.attr.name = vk_name;
		ret = sysfs_create_file(pdata->vk_obj, &(vk_dev.attr));
	}

	ts->sr_input_dev->name = "sr_touchscreen";
	set_bit(EV_SYN, ts->sr_input_dev->evbit);
	set_bit(EV_ABS, ts->sr_input_dev->evbit);
	set_bit(EV_KEY, ts->sr_input_dev->evbit);

	set_bit(KEY_BACK, ts->sr_input_dev->keybit);
	set_bit(KEY_HOME, ts->sr_input_dev->keybit);
	set_bit(KEY_MENU, ts->sr_input_dev->keybit);
	set_bit(KEY_SEARCH, ts->sr_input_dev->keybit);
	set_bit(BTN_TOUCH, ts->sr_input_dev->keybit);
	set_bit(KEY_APP_SWITCH, ts->sr_input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->sr_input_dev->propbit);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_TRACKING_ID,
		0, MAX1187X_TOUCH_COUNT_MAX, 0, 0);
	pr_info("[SR]input_set_abs_params: mix_x %d, max_x %d,"
		" min_y %d, max_y %d", PDATA(panel_min_x),
		 PDATA(panel_max_x), PDATA(panel_min_y), PDATA(panel_max_y));

	input_set_abs_params(ts->sr_input_dev, ABS_MT_POSITION_X,
			PDATA(panel_min_x), PDATA(panel_max_x), 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_POSITION_Y,
			PDATA(panel_min_y), PDATA(panel_max_y), 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_TOUCH_MAJOR,
		0, 255, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_PRESSURE,
		0, 30, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_WIDTH_MAJOR,
		0, 30, 0, 0);

	if (input_register_device(ts->sr_input_dev)) {
		input_free_device(ts->sr_input_dev);
		pr_info("[SR][TOUCH_ERR]%s: Unable to register %s input device\n",
			__func__, ts->sr_input_dev->name);
		return REGISTER_DEV_FAIL;
	}
	return SUCCESS;
}

static ssize_t set_en_sr(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct data *ts = gl_ts;
	if (buf[0]) {
		if (ts->sr_input_dev)
			printk(KERN_INFO "[TP]%s: SR device already exist!\n", __func__);
		else
			printk(KERN_INFO "[TP]%s: SR touch device enable result:%X\n", __func__, register_sr_touch_device());
	}
	return count;
}

static ssize_t get_en_sr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct data *ts = gl_ts;
	size_t count = 0;
	if (ts->sr_input_dev)
	{
		count += sprintf(buf + count, "%s ", ts->sr_input_dev->name);
		count += sprintf(buf + count, "\n");
	}
	else
		count += sprintf(buf + count, "0\n");


	return count;
}

static DEVICE_ATTR(init, (S_IWUSR|S_IRUGO), init_show, init_store);
static DEVICE_ATTR(hreset, S_IWUSR, NULL, hreset_store);
static DEVICE_ATTR(sreset, S_IWUSR, NULL, sreset_store);
static DEVICE_ATTR(reflash, S_IWUSR, NULL, reflash_store);
static DEVICE_ATTR(irq_count, (S_IWUSR|S_IRUGO), irq_count_show, irq_count_store);
static DEVICE_ATTR(dflt_cfg, (S_IWUSR|S_IRUGO), dflt_cfg_show, dflt_cfg_store);
static DEVICE_ATTR(panel, (S_IWUSR|S_IRUGO), panel_show, panel_store);
static DEVICE_ATTR(fw_ver, S_IRUGO, fw_ver_show, NULL);
static DEVICE_ATTR(driver_ver, S_IRUGO, driver_ver_show, NULL);
static DEVICE_ATTR(debug, (S_IWUSR|S_IRUGO), debug_show, debug_store);
static DEVICE_ATTR(command, S_IWUSR, NULL, command_store);
static struct bin_attribute dev_attr_report = {
		.attr = {.name = "report", .mode = S_IRUGO}, .read = report_read };

static struct device_attribute *dev_attrs[] = {
		&dev_attr_hreset,
		&dev_attr_sreset,
		&dev_attr_reflash,
		&dev_attr_irq_count,
		&dev_attr_dflt_cfg,
		&dev_attr_panel,
		&dev_attr_fw_ver,
		&dev_attr_driver_ver,
		&dev_attr_debug,
		&dev_attr_command,
		NULL };

static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO), debug_show, debug_store);
static DEVICE_ATTR(vendor, S_IRUGO, touch_vendor_show, NULL);
static DEVICE_ATTR(config, S_IRUGO, config_show, NULL);
static DEVICE_ATTR(gpio, S_IRUGO, gpio_show, NULL);
static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO), diag_show, diag_store);
static DEVICE_ATTR(unlock, (S_IWUSR|S_IRUGO), NULL, unlock_store);
static DEVICE_ATTR(reset, S_IWUSR, NULL, reset_store);
static DEVICE_ATTR(enabled, (S_IWUSR|S_IRUGO), int_status_show, int_status_store);
static DEVICE_ATTR(sr_en, (S_IWUSR|S_IRUGO), get_en_sr, set_en_sr);
static DEVICE_ATTR(cover, (S_IWUSR|S_IRUGO), cover_show, cover_store);
static DEVICE_ATTR(glove_setting, (S_IWUSR|S_IRUGO), glove_setting_show, glove_setting_store);

static void set_chip_mode(struct data *ts)
{
	if (ts->fw_config->support_glove && (!ts->i2c_to_mcu)) {
		if ((ts->glove_setting & 0x01) || ts->cover_mode) {
			set_glove_mode(ts->client, 1);
			ts->glove_enable = 1;
			pr_info("%s: enable glove mode", __func__);
		}
		else {
			set_glove_mode(ts->client, 0);
			ts->glove_enable = 0;
			pr_info("%s: disable glove mode", __func__);
		}
	}

	if (ts->fw_config->support_cover) {
		set_cover_mode(ts->client, ts->cover_mode);
		if (ts->fw_config->filter_range != NULL) {
			if (ts->cover_mode) {
				set_edge_filter(ts->client, 2, ts->fw_config->filter_range);
				pr_info("%s: enable filter", __func__);
			} else {
				set_edge_filter(ts->client, 0, NULL);
				pr_info("%s: disable filter", __func__);
			}
		}
	}
	pr_info("%s: version: %s-%02X-%04X\n", __func__,
		ts->fw_ver, ts->vendor_pin, ts->config_id);
}

static void system_status_handler(struct data *ts)
{
	set_chip_mode(ts);
}

/* debug_mask |= 0x80000 for all driver INIT */
static void collect_chip_data(struct data *ts)
{
	int ret, i, build_number = 0;

	ret = get_report(ts, 0x01A0, 3000);
	if (ret != 0) {
		pr_err("Failed to receive system status report");
		if (PDATA(defaults_allow) == 0)
			msleep(5000);
	} else {
		ts->vendor_pin = BYTEH(ts->rx_report[3]) & PDATA(tw_mask);
		pr_info_if(8, "(INIT): vendor_pin=%x", ts->vendor_pin);
		release_report(ts);
		ts->fw_responsive = 1;
	}
#if 0 /* Debug report */
	DISABLE_IRQ();
	ret = get_report(ts, 0x0121, 500);
	if (ret == 0) {
		pr_info_if(8, "(INIT): Get power mode report:%d", ts->rx_report[2]);
		release_report(ts);
	}
#endif
	for	(i = 0; i < RETRY_TIMES; i ++) {
		DISABLE_IRQ();
		ret = get_fw_version(ts->client);
		if (ret < 0)
			pr_err("Failed to retrieve firmware version");
		if (ret == 0) {
			ret = get_report(ts, 0x0140, 100);
			if (ret != 0)
				pr_info("[W] Get firmware version time out-%d, retry", i);
			if (ret == 0) {
				memcpy(ts->fw_version, &ts->rx_report[1],
						BYTE_SIZE(ts->rx_report[2] + 2));
				release_report(ts);
				ts->have_fw = 1;
				break;
			}
		}
	}
	if (i==RETRY_TIMES && ret!=0)
		pr_err("Failed to receive firmware version report");
	for	(i = 0; i < RETRY_TIMES; i ++) {
		DISABLE_IRQ();
		ret = get_touch_config(ts->client);
		if (ret < 0)
			pr_err("Failed to retrieve touch config");
		if (ret == 0) {
			ret = get_report(ts, 0x0102, 100);
			if (ret != 0)
				pr_info("[W] Get touch config time out-%d, retry", i);
			if (ret == 0) {
				memcpy(ts->touch_config, &ts->rx_report[1],
						BYTE_SIZE(ts->rx_report[2] + 2));
				release_report(ts);
				ts->have_touchcfg = 1;
				break;
			}
		}
	}
	if (i==RETRY_TIMES && ret!=0)
		pr_err("Failed to receive touch config report");
	ENABLE_IRQ();
	pr_info_if(8, "(INIT): firmware responsive: (%u)", ts->fw_responsive);
	if (ts->fw_responsive) {
		if (ts->have_fw) {
			if (ts->fw_version[1] >= 3)
				build_number = ts->fw_version[4];
			snprintf(ts->fw_ver, 16, "%u.%u.%u", BYTEH(ts->fw_version[2]),
						BYTEL(ts->fw_version[2]), build_number);
			ts->protocol_ver = BYTEL(ts->fw_version[3]) & 0x3F;
			pr_info_if(8, "(INIT): firmware version: %u.%u.%u_p%u Chip ID: "
				"0x%02X", BYTEH(ts->fw_version[2]),
				BYTEL(ts->fw_version[2]),
				build_number,
				BYTEL(ts->fw_version[3]) & 0x3F,
				BYTEH(ts->fw_version[3]));
		} else
			snprintf(ts->fw_ver, 16, "Bootloader");
		if (ts->have_touchcfg) {
			ts->config_id = ts->touch_config[2];
			pr_info_if(8, "(INIT): configuration ID: 0x%04X",
					ts->touch_config[2]);
			ts->x_channel = BYTEH(ts->touch_config[3]);
			ts->y_channel = BYTEL(ts->touch_config[3]);
			pr_info_if(8, "(INIT): Channel=(%d,%d)", ts->x_channel, ts->y_channel);
			ts->frame_rate[0] = ts->touch_config[4];
			ts->frame_rate[1] = ts->touch_config[5];
			pr_info_if(8, "(INIT): Frame Rate=(%d,%d)", ts->frame_rate[0], ts->frame_rate[1]);
			if (ts->protocol_ver != 0) {
				if (ts->protocol_ver <= 7) {
					if (PDATA(coordinate_settings) & MAX1187X_SWAP_XY) {
						PDATA(panel_min_x) = 0;
						PDATA(panel_max_x) = ts->touch_config[28] & 0x7FFF;
						PDATA(panel_min_y) = 0;
						PDATA(panel_max_y) = ts->touch_config[27] & 0x7FFF;
					} else {
						PDATA(panel_min_x) = 0;
						PDATA(panel_max_x) = ts->touch_config[27] & 0x7FFF;
						PDATA(panel_min_y) = 0;
						PDATA(panel_max_y) = ts->touch_config[28] & 0x7FFF;
					}
				} else {
					if (PDATA(coordinate_settings) & MAX1187X_SWAP_XY) {
						PDATA(panel_min_x) = 0;
						PDATA(panel_max_x) = ts->touch_config[44] & 0x7FFF;
						PDATA(panel_min_y) = 0;
						PDATA(panel_max_y) = ts->touch_config[43] & 0x7FFF;
					} else {
						PDATA(panel_min_x) = 0;
						PDATA(panel_max_x) = ts->touch_config[43] & 0x7FFF;
						PDATA(panel_min_y) = 0;
						PDATA(panel_max_y) = ts->touch_config[44] & 0x7FFF;
					}
				}
				pr_info_if(8, "(INIT): Resolution=(%d,%d)", PDATA(panel_max_x), PDATA(panel_max_y));
			}
		}
	}
	else
		snprintf(ts->fw_ver, 16, "Failed");
}

static int device_fw_load(struct data *ts, const struct firmware *fw,
	u16 fw_index, int tagLen)
{
	u16 filesize, file_codesize, loopcounter;
	u16 file_crc16_1, file_crc16_2, local_crc16;
	int chip_crc16_1 = -1, chip_crc16_2 = -1, ret;

	filesize = PDATA(fw_mapping[fw_index]).filesize;
	file_codesize = PDATA(fw_mapping[fw_index]).file_codesize;

	if (fw->size-tagLen != filesize) {
		pr_err("filesize (%ld) is not equal to expected size (%d)",
				fw->size, filesize);
		return -EIO;
	}

	file_crc16_1 = crc16(0, fw->data+tagLen, file_codesize);

	loopcounter = 0;
	do {
		ret = bootloader_enter(ts);
		if (ret == 0)
			ret = bootloader_get_crc(ts, &local_crc16,
				0, file_codesize, 200);
		if (ret == 0)
			chip_crc16_1 = local_crc16;
		ret = bootloader_exit(ts);
		loopcounter++;
	} while (loopcounter < MAX_FW_RETRIES && chip_crc16_1 == -1);

	pr_info_if(8, "(INIT): file_crc16_1 = 0x%04x, chip_crc16_1 = 0x%04x\n",
			file_crc16_1, chip_crc16_1);

	ts->fw_index = fw_index;
	ts->fw_crc16 = chip_crc16_1;

	if (file_crc16_1 != chip_crc16_1) {
		loopcounter = 0;
		file_crc16_2 = crc16(0, fw->data+tagLen, filesize);

		while (loopcounter < MAX_FW_RETRIES && file_crc16_2
				!= chip_crc16_2) {
			pr_info_if(8, "(INIT): Reprogramming chip. Attempt %d",
					loopcounter+1);
			ret = bootloader_enter(ts);
			if (ret == 0)
				ret = bootloader_erase_flash(ts);
			if (ret == 0)
				ret = bootloader_set_byte_mode(ts);
			if (ret == 0)
				ret = bootloader_write_flash(ts, fw->data+tagLen,
					filesize);
			if (ret == 0)
				ret = bootloader_get_crc(ts, &local_crc16,
					0, filesize, 200);
			if (ret == 0)
				chip_crc16_2 = local_crc16;
			pr_info_if(8, "(INIT): file_crc16_2 = 0x%04x, "\
					"chip_crc16_2 = 0x%04x\n",
					file_crc16_2, chip_crc16_2);
			ret = bootloader_exit(ts);
			loopcounter++;
		}

		if (file_crc16_2 != chip_crc16_2)
			return -EAGAIN;
	}

	loopcounter = 0;
	do {
		ret = bootloader_exit(ts);
		loopcounter++;
	} while (loopcounter < MAX_FW_RETRIES && ret != 0);

	if (ret != 0)
		return -EIO;

	ts->fw_crc16 = file_crc16_1;

	collect_chip_data(ts);
	if (ts->have_fw == 0 || ts->have_touchcfg == 0) {
		pr_err("firmware is unresponsive or inconsistent and "\
				"no valid configuration is present");
		return -ENXIO;
	}

	return 0;
}

static int is_booting(void)
{
	unsigned long long t;
	unsigned long nanosec_rem;

	t = cpu_clock(smp_processor_id());
	nanosec_rem = do_div(t, 1000000000);
	return (t < 30) ? 1 : 0;
}

static int compare_u16_arrays(u16 *buf1, u16 *buf2, u16 n)
{
	int i;
	for (i = 0; i < n; i++) {
		if (buf1[i] != buf2[i])
			return 1;
	}
	return 0;
}

u16 calculate_checksum(u16 *buf, u16 n)
{
	u16 i, cs = 0;
	for (i = 0; i < n; i++)
		cs += buf[i];
	return cs;
}

static u8 compare_Touch_Configuration(struct data *ts) {
	int i, ret;

	if (ts->max11871_Touch_Configuration_Data) {
		for (i=0; i<RETRY_TIMES; i++) {
			DISABLE_IRQ();
			ret = get_touch_config(ts->client);
			if (ret < 0)
				pr_info("[W] Failed to retrieve touch config");
			if (ret == 0) {
				ret = get_report(ts, 0x0102, 150);
				if (ret != 0)
					pr_info("[W] Get touch config time out-%d, retry", i);
				if (ret == 0) {
					if (compare_u16_arrays(&ts->rx_report[2],
						&ts->max11871_Touch_Configuration_Data[1], config_num[ts->config_protocol][0]+1)!=0) {
						pr_info_if(8, "(Config): Touch Configuration Data mismatch");
						ret = 1;
					} else {
						pr_info_if(8, "(Config): Touch Configuration Data okay");
					}
					release_report(ts);
					if (ret == 1)
						return 1;
					else
						break;
				}
			}
		}
		if (i==RETRY_TIMES && ret!=0)
			pr_err("Failed to receive touch config report");
	}
	return 0;
}

static u8 compare_Calibration_Table(struct data *ts) {
	int i, ret;
	u16 mtpdata[]={0x0000, 0x0000};

	if (ts->max11871_Calibration_Table_Data) {
		for (i=0; i<RETRY_TIMES; i++) {
			DISABLE_IRQ();
			//Get calibration table
			mtpdata[0]=0x0011;
			mtpdata[1]=0x0000;
			ret = send_mtp_command(ts, mtpdata, 2);
			if (ret < 0)
				pr_info("[W] Failed to retrieve calibration table");
			if (ret == 0) {
				ret = get_report(ts, 0x0111, 150);
				if (ret != 0)
					pr_info("[W] Get calibration table time out-%d, retry", i);
				if (ret == 0) {
					if (compare_u16_arrays(&ts->rx_report[2],
						&ts->max11871_Calibration_Table_Data[1], config_num[ts->config_protocol][1]+1)!=0) {
						pr_info_if(8, "(Config): Calibration Table Data mismatch");
						ret = 1;
					} else {
						pr_info_if(8, "(Config): Calibration Table Data okay");
					}
					release_report(ts);
					if (ret == 1)
						return 1;
					else
						break;
				}
			}
		}
		if (i==RETRY_TIMES && ret!=0)
			pr_err("Failed to receive calibration table report");
	}
	return 0;
}

static u8 compare_Private_Configuration(struct data *ts) {
	int i, ret;
	u16 mtpdata[]={0x0000, 0x0000};

	if (ts->max11871_Private_Configuration_Data) {
		for (i=0; i<RETRY_TIMES; i++) {
			DISABLE_IRQ();
			//Get private configuration
			mtpdata[0]=0x0004;
			mtpdata[1]=0x0000;
			ret = send_mtp_command(ts, mtpdata, 2);
			if (ret < 0)
				pr_info("[W] Failed to retrieve private config");
			if (ret == 0) {
				ret = get_report(ts, 0x0104, 150);
				if (ret != 0)
					pr_info("[W] Get private config time out-%d, retry", i);
				if (ret == 0) {
					if (compare_u16_arrays(&ts->rx_report[2],
						&ts->max11871_Private_Configuration_Data[1], config_num[ts->config_protocol][2]+1)!=0) {
						pr_info_if(8, "(Config): Private Configuration Data mismatch");
						ret = 1;
					} else {
						pr_info_if(8, "(Config): Private Configuration Data okay");
					}
					release_report(ts);
					if (ret == 1)
						return 1;
					else
						break;
				}
			}
		}
		if (i==RETRY_TIMES && ret!=0)
			pr_err("Failed to receive private config report");
	}
	return 0;
}

static u8 compare_Lookup_Table_X(struct data *ts) {
	int i, ret;
	u16 mtpdata[]={0x0000, 0x0000, 0x0000};

	if (ts->max11871_Lookup_Table_X_Data) {
		for (i=0; i<RETRY_TIMES; i++) {
			DISABLE_IRQ();
			//Get Lookup table X
			mtpdata[0]=0x0031;
			mtpdata[1]=0x0001;
			mtpdata[2]=0x0000;
			ret = send_mtp_command(ts, mtpdata, 3);
			if (ret < 0)
				pr_info("[W] Failed to retrieve Lookup table X");
			if (ret == 0) {
				ret = get_report(ts, 0x0131, 150);
				if (ret != 0)
					pr_info("[W] Get Lookup table X time out-%d, retry", i);
				if (ret == 0) {
					if (compare_u16_arrays(&ts->rx_report[3],
						&ts->max11871_Lookup_Table_X_Data[3], config_num[ts->config_protocol][3])!=0) {
						pr_info_if(8, "(Config): Lookup Table X Data mismatch");
						ret = 1;
					} else {
						pr_info_if(8, "(Config): Lookup Table X Data okay");
					}
					release_report(ts);
					if (ret == 1)
						return 1;
					else
						break;
				}
			}
		}
		if (i==RETRY_TIMES && ret!=0)
			pr_err("Failed to receive Lookup table X report");
	}
	return 0;
}

static u8 compare_Lookup_Table_Y(struct data *ts) {
	int i, ret;
	u16 mtpdata[]={0x0000, 0x0000, 0x0000};

	if (ts->max11871_Lookup_Table_Y_Data) {
		for (i=0; i<RETRY_TIMES; i++) {
			DISABLE_IRQ();
			//Get Lookup table Y
			mtpdata[0]=0x0031;
			mtpdata[1]=0x0001;
			mtpdata[2]=0x0001;
			ret = send_mtp_command(ts, mtpdata, 3);
			if (ret < 0)
				pr_info("[W] Failed to retrieve Lookup table Y");
			if (ret == 0) {
				ret = get_report(ts, 0x0131, 150);
				if (ret != 0)
					pr_info("[W] Get Lookup table Y time out-%d, retry", i);
				if (ret == 0) {
					if (compare_u16_arrays(&ts->rx_report[3],
						&ts->max11871_Lookup_Table_Y_Data[3], config_num[ts->config_protocol][3])!=0) {
						pr_info_if(8, "(Config): Lookup Table Y Data mismatch");
						ret = 1;
					} else {
						pr_info_if(8, "(Config): Lookup Table Y Data okay");
					}
					release_report(ts);
					if (ret == 1)
						return 1;
					else
						break;
				}
			}
		}
		if (i==RETRY_TIMES && ret!=0)
			pr_err("Failed to receive Lookup table Y report");
	}
	return 0;
}

static u8 compare_Image_Factor(struct data *ts) {
	int i, ret;
	u16 mtpdata[]={0x0000, 0x0000};

	if (ts->max11871_Image_Factor_Table && config_num[ts->config_protocol][4]) {
		for (i=0; i<RETRY_TIMES; i++) {
			DISABLE_IRQ();
			//Get Image Factor Table
			mtpdata[0]=0x0047;
			mtpdata[1]=0x0000;
			ret = send_mtp_command(ts, mtpdata, 2);
			if (ret < 0)
				pr_info("[W] Failed to retrieve Image Factor Table");
			if (ret == 0) {
				ret = get_report(ts, 0x0147, 150);
				if (ret != 0)
					pr_info("[W] Get Image Factor Table time out-%d, retry", i);
				if (ret == 0) {
					if (ts->rx_report[3] !=
						calculate_checksum(ts->max11871_Image_Factor_Table, 460)) {
						pr_info_if(8, "(Config): Image Factor Table mismatch");
						ret = 1;
					} else {
						pr_info_if(8, "(Config): Image Factor Table okay");
					}
					release_report(ts);
					if (ret == 1)
						return 1;
					else
						break;
				}
			}
		}
		if (i==RETRY_TIMES && ret!=0)
			pr_err("Failed to receive Image Factor Table report");
	}
	return 0;
}

static void update_config(struct data *ts)
{
	int i, ret;
	u16 reload_touch_config=0, reload_calib_table=0, reload_private_config=0;
	u16 reload_lookup_x=0, reload_lookup_y=0, reload_imagefactor_table=0;
	u16 imagefactor_data[104];
	u32 retry_config_update_delay = PDATA(retry_config_update_delay);


	/* configure the chip */
	reload_touch_config = compare_Touch_Configuration(ts);
	reload_calib_table = compare_Calibration_Table(ts);
	reload_private_config = compare_Private_Configuration(ts);
	reload_lookup_x = compare_Lookup_Table_X(ts);
	reload_lookup_y = compare_Lookup_Table_Y(ts);
	reload_imagefactor_table = compare_Image_Factor(ts);

	//Configuration check has been done
	//Now download correct configurations if required
	if (reload_touch_config) {
		pr_info_if(8, "(Config): Update Configuration Table");
		DISABLE_IRQ();
		for (i=0; i<RETRY_TIMES_CONFIG; i++) {
			ret = send_mtp_command(ts, ts->max11871_Touch_Configuration_Data, config_num[ts->config_protocol][0]+2);
			msleep(retry_config_update_delay + i*50);
			if(ret < 0)
				pr_err("Failed to send Touch Config");
			else if (!compare_Touch_Configuration(ts))
				break;
		}
		if (i==RETRY_TIMES_CONFIG)
			pr_err("Fail to update Touch Config");
		ENABLE_IRQ();
	}
	if (reload_calib_table) {
		pr_info_if(8, "(Config): Update Calibration Table");
		DISABLE_IRQ();
		for (i=0; i<RETRY_TIMES_CONFIG; i++) {
			ret = send_mtp_command(ts, ts->max11871_Calibration_Table_Data, config_num[ts->config_protocol][1]+2);
			msleep(retry_config_update_delay + i*50);
			if(ret < 0)
				pr_err("Failed to send Calib Table");
			else if (!compare_Calibration_Table(ts))
					break;
		}
		if (i==RETRY_TIMES_CONFIG)
			pr_err("Fail to update Calib Table");
		ENABLE_IRQ();
	}
	if (reload_private_config) {
		pr_info_if(8, "(Config): Update Private Configuration Table");
		DISABLE_IRQ();
		for (i=0; i<RETRY_TIMES_CONFIG; i++) {
			ret = send_mtp_command(ts, ts->max11871_Private_Configuration_Data, config_num[ts->config_protocol][2]+2);
			msleep(retry_config_update_delay + i*50);
			if(ret < 0)
				pr_err("Failed to send Private Config");
			else if (!compare_Private_Configuration(ts))
				break;
		}
		if (i==RETRY_TIMES_CONFIG)
			pr_err("Fail to update Private Config");
		ENABLE_IRQ();
	}
	if (reload_lookup_x) {
		pr_info_if(8, "(Config): Update Lookup Table X");
		DISABLE_IRQ();
		for (i=0; i<RETRY_TIMES_CONFIG; i++) {
			ret = send_mtp_command(ts, ts->max11871_Lookup_Table_X_Data, config_num[ts->config_protocol][3]+3);
			msleep(retry_config_update_delay + i*50);
			if(ret < 0)
				pr_err("Failed to send Lookup Table X");
			else if (!compare_Lookup_Table_X(ts))
				break;
		}
		if (i==RETRY_TIMES_CONFIG)
			pr_err("Fail to update Lookup Table X");
		ENABLE_IRQ();
	}
	if (reload_lookup_y) {
		pr_info_if(8, "(Config): Update Lookup Table Y");
		DISABLE_IRQ();
		for (i=0; i<RETRY_TIMES_CONFIG; i++) {
			ret = send_mtp_command(ts, ts->max11871_Lookup_Table_Y_Data, config_num[ts->config_protocol][3]+3);
			msleep(retry_config_update_delay + i*50);
			if(ret < 0)
				pr_err("Failed to send Lookup Table Y");
			else if (!compare_Lookup_Table_Y(ts))
				break;
		}
		if (i==RETRY_TIMES_CONFIG)
			pr_err("Fail to update Lookup Table Y");
		ENABLE_IRQ();
	}
	if (reload_imagefactor_table && config_num[ts->config_protocol][4]) {
		pr_info_if(8, "(Config): Update Image Factor Table");
		DISABLE_IRQ();
		for (i=0; i<RETRY_TIMES_CONFIG; i++) {
			//0-59 words
			imagefactor_data[0] = 0x0046;
			imagefactor_data[1] = 0x003E;
			imagefactor_data[2] = 0x0000;
			memcpy(imagefactor_data+3, ts->max11871_Image_Factor_Table, 60 << 1);
			imagefactor_data[63] = calculate_checksum(imagefactor_data+2,61);
			send_mtp_command(ts, imagefactor_data, 64);
			msleep(100);
			//60-159 words
			imagefactor_data[0] = 0x0046;
			imagefactor_data[1] = 0x0066;
			imagefactor_data[2] = 0x003C;
			memcpy(imagefactor_data+3, ts->max11871_Image_Factor_Table+60, 100 << 1);
			imagefactor_data[103] = calculate_checksum(imagefactor_data+2,101);
			send_mtp_command(ts, imagefactor_data, 104);
			msleep(100);
			//160-259 words
			imagefactor_data[0] = 0x0046;
			imagefactor_data[1] = 0x0066;
			imagefactor_data[2] = 0x00A0;
			memcpy(imagefactor_data+3, ts->max11871_Image_Factor_Table+160, 100 << 1);
			imagefactor_data[103] = calculate_checksum(imagefactor_data+2,101);
			send_mtp_command(ts, imagefactor_data, 104);
			msleep(100);
			//260-359 words
			imagefactor_data[0] = 0x0046;
			imagefactor_data[1] = 0x0066;
			imagefactor_data[2] = 0x0104;
			memcpy(imagefactor_data+3, ts->max11871_Image_Factor_Table+260, 100 << 1);
			imagefactor_data[103] = calculate_checksum(imagefactor_data+2,101);
			send_mtp_command(ts, imagefactor_data, 104);
			msleep(100);
			//360-459 words
			imagefactor_data[0] = 0x0046;
			imagefactor_data[1] = 0x0066;
			imagefactor_data[2] = 0x8168;
			memcpy(imagefactor_data+3, ts->max11871_Image_Factor_Table+360, 100 << 1);
			imagefactor_data[103] = calculate_checksum(imagefactor_data+2,101);
			send_mtp_command(ts, imagefactor_data, 104);

			msleep(retry_config_update_delay + i*50);
			if (!compare_Image_Factor(ts))
				break;
		}
		if (i==RETRY_TIMES_CONFIG)
			pr_err("Fail to update Image Factor Table");
		ENABLE_IRQ();
	}
	if (reload_touch_config || reload_calib_table || reload_private_config ||
		reload_lookup_x || reload_lookup_y || reload_imagefactor_table) {
		DISABLE_IRQ();
		if (sreset(ts->client) != 0) {
			pr_err("Failed to do soft reset.");
			return;
		}
		collect_chip_data(ts);
		if (ts->have_fw == 0 || ts->have_touchcfg == 0) {
			pr_err("firmware is unresponsive or inconsistent and "\
				"no valid configuration is present");
			return;
		}
		pr_info_if(8, "(INIT): Update config complete");
	}
}

static int check_bin_version(const struct firmware *fw, int *tagLen, char *fw_ver)
{
	char tag[40];
	int i = 0;

	if (fw->data[0] == 'T' && fw->data[1] == 'P') {
		while ((tag[i] = fw->data[i]) != '\n')
			i++;
		tag[i] = '\0';
		*tagLen = i+1;
		pr_info_if(8, "(INIT): tag=%s", tag);
		if (strstr(tag, fw_ver) != NULL) {
			pr_info_if(8, "(INIT): Update Bypass");
			return 0; /* bypass */
		}
	}

	pr_info_if(8, "(INIT): Need Update");
	return 1;
}

static void check_fw_and_config(struct data *ts)
{
	u16 config_id, chip_id;
	const struct firmware *fw;
	int i, ret, tagLen = 0;

	sreset(ts->client);
	collect_chip_data(ts);
	if ((ts->have_fw == 0 || ts->have_touchcfg == 0) &&
			PDATA(defaults_allow) == 0) {
		pr_err("firmware is unresponsive or inconsistent "\
				"and default selections are disabled");
		return;
	}
	config_id = ts->have_touchcfg ? ts->touch_config[2]
			: PDATA(default_config_id);
	chip_id = ts->have_fw ? BYTEH(ts->fw_version[3]) : \
			PDATA(default_chip_id);

	if (PDATA(update_feature)&MAX1187X_UPDATE_BIN) {
		for (i = 0; i < PDATA(num_fw_mappings); i++) {
			if (PDATA(fw_mapping[i]).chip_id == chip_id)
				break;
		}

		if (i == PDATA(num_fw_mappings)) {
			pr_err("FW not found for configID(0x%04X) and chipID(0x%04X)",
				config_id, chip_id);
			return;
		}

		pr_info_if(8, "(INIT): Firmware file (%s)",
			PDATA(fw_mapping[i]).filename);

		ret = request_firmware(&fw, PDATA(fw_mapping[i]).filename,
						&ts->client->dev);

		if (ret || fw == NULL) {
			pr_info("firmware request failed (ret = %d, fwptr = %p)",
				ret, fw);
			return;
		}

		if (check_bin_version(fw, &tagLen, ts->fw_ver)) {
			if (device_fw_load(ts, fw, i, tagLen)) {
				release_firmware(fw);
				pr_err("firmware download failed");
				return;
			}
		}

		release_firmware(fw);
		pr_info_if(8, "(INIT): firmware download OK");
	}

	/* configure the chip */
	if (PDATA(update_feature)&MAX1187X_UPDATE_CONFIG) {
		while(ts->fw_config->config_id != 0) {
			if(ts->fw_config->chip_id != chip_id) {
				ts->fw_config++;
				continue;
			}
			if(ts->fw_config->protocol_ver != ts->protocol_ver) {
				ts->fw_config++;
				continue;
			}
			if(ts->fw_config->major_ver > BYTEH(ts->fw_version[2])) {
				ts->fw_config++;
				continue;
			}
			if(ts->fw_config->minor_ver > BYTEL(ts->fw_version[2])) {
				ts->fw_config++;
				continue;
			}
/*			if(PDATA(disp_panel_check) && PDATA(disp_panel)
					&& ts->fw_config->disp_panel) {
				if(!strstr(PDATA(disp_panel), ts->fw_config->disp_panel)) {
					ts->fw_config++;
					continue;
				}
			}*/
			if(PDATA(tw_mask) && ts->fw_config->vendor_pin) {
				if(ts->fw_config->vendor_pin != ts->vendor_pin) {
					ts->fw_config++;
					continue;
				}
			}
			if ((PDATA(eng_id) != ts->fw_config->eng_id)) {
				ts->fw_config++;
				continue;
			}
			if(ts->fw_config->protocol_ver <= 7)
				ts->config_protocol = 0;
			else
				ts->config_protocol = 1;
			if(ts->fw_config->config_touch) {
				ts->max11871_Touch_Configuration_Data[0] = 0x0001;
				ts->max11871_Touch_Configuration_Data[1] = config_num[ts->config_protocol][0];
				memcpy(ts->max11871_Touch_Configuration_Data+2,
					ts->fw_config->config_touch, BYTE_SIZE(config_num[ts->config_protocol][0]));
			}
			if(ts->fw_config->config_cal) {
				ts->max11871_Calibration_Table_Data[0] = 0x0010;
				ts->max11871_Calibration_Table_Data[1] = config_num[ts->config_protocol][1];
				memcpy(ts->max11871_Calibration_Table_Data+2,
					ts->fw_config->config_cal, BYTE_SIZE(config_num[ts->config_protocol][1]));
			}
			if(ts->fw_config->config_private) {
				ts->max11871_Private_Configuration_Data[0] = 0x0003;
				ts->max11871_Private_Configuration_Data[1] = config_num[ts->config_protocol][2];
				memcpy(ts->max11871_Private_Configuration_Data+2,
					ts->fw_config->config_private, BYTE_SIZE(config_num[ts->config_protocol][2]));
			}
			if(ts->fw_config->config_lin_x) {
				ts->max11871_Lookup_Table_X_Data[0] = 0x0030;
				ts->max11871_Lookup_Table_X_Data[1] = config_num[ts->config_protocol][3]+1;
				ts->max11871_Lookup_Table_X_Data[2] = 0x0000;
				memcpy(ts->max11871_Lookup_Table_X_Data+3,
					ts->fw_config->config_lin_x, BYTE_SIZE(config_num[ts->config_protocol][3]));
			}
			if(ts->fw_config->config_lin_y) {
				ts->max11871_Lookup_Table_Y_Data[0] = 0x0030;
				ts->max11871_Lookup_Table_Y_Data[1] = config_num[ts->config_protocol][3]+1;
				ts->max11871_Lookup_Table_Y_Data[2] = 0x0001;
				memcpy(ts->max11871_Lookup_Table_Y_Data+3,
					ts->fw_config->config_lin_y, BYTE_SIZE(config_num[ts->config_protocol][3]));
			}
			if(ts->fw_config->config_ifactor && config_num[ts->config_protocol][4]) {
				memcpy(ts->max11871_Image_Factor_Table,
					ts->fw_config->config_ifactor, BYTE_SIZE(MAX1187X_IMAGE_FACTOR_MAX));
			}
			break;
		}
		if(ts->fw_config->config_id != 0) {
			update_config(ts);
			pr_info_if(8, "(INIT): Check config finish");
		}
	}

	ENABLE_IRQ();

	if (change_touch_rpt(ts->client, PDATA(report_mode)) < 0) {
		pr_err("Failed to set up touch report mode");
		return;
	}
}

#ifdef CONFIG_AK8789_HALLSENSOR
static int hallsensor_status_handler_func(struct notifier_block *this,
	unsigned long status, void *unused)
{
	int pole = 0, pole_value = 0;
	struct data *ts = gl_ts;

	pole_value = status & 0x01;
	pole = (status & 0x02) >> HALL_POLE_BIT;
	pr_info("[HL] %s[%s]", pole? "att_s" : "att_n", pole_value ? "Near" : "Far");

	if ((pole == HALL_POLE_S) && ts->fw_config->support_cover) {
		if (pole_value == HALL_FAR) {
			ts->cover_mode = 0;
			if (ts->fw_config->support_glove) {
				if ((ts->glove_setting & 0x01) == 0)
					ts->glove_enable = 0;
				else
					ts->glove_enable = 1;
			}
			if (PDATA(hall_block_touch_time) > 1)
				max1187x_handle_block_touch(ts, 0);
		}
		else {
			ts->cover_mode = 1;
			if (ts->fw_config->support_glove)
				ts->glove_enable = 1;
			if (PDATA(hall_block_touch_time) > 1)
				max1187x_handle_block_touch(ts, 1);
		}

		if (!ts->i2c_to_mcu) {
			if (ts->fw_config->support_glove) {
				set_glove_mode(ts->client, ts->glove_enable);
				pr_info("%s: glove_enable = %d", __func__, ts->glove_enable);
			}
			set_cover_mode(ts->client, ts->cover_mode);
			if (ts->fw_config->filter_range != NULL) {
				if (ts->cover_mode) {
					set_edge_filter(ts->client, 2, ts->fw_config->filter_range);
					pr_info("%s: enable filter", __func__);
				} else {
					set_edge_filter(ts->client, 0, NULL);
					pr_info("%s: disable filter", __func__);
				}
			}
		}
		pr_info("[HL] %s: cover_enable = %d.", __func__, ts->cover_mode);
	}

	return NOTIFY_OK;
}

static struct notifier_block hallsensor_status_handler = {
	.notifier_call = hallsensor_status_handler_func,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_TOUCH_FW_UPDATE
static void maxim_fwu_progress(int percentage)
{
	touch_fw_update_progress(percentage);
}

int maxim_touch_fw_update(struct firmware *fw)
{
	struct data *ts = gl_ts;
	int tagLen = 0, i, ret = 0;
	u16 chip_id;

	chip_id = ts->have_fw ? BYTEH(ts->fw_version[3]) : \
			PDATA(default_chip_id);

	for (i = 0; i < PDATA(num_fw_mappings); i++) {
		if (PDATA(fw_mapping[i]).chip_id == chip_id)
			break;
	}

	if (i == PDATA(num_fw_mappings)) {
		pr_err("FW not found for chipID(0x%04X)", chip_id);
		return -1;
	}

	maxim_fwu_progress(0);
	if (check_bin_version(fw, &tagLen, ts->fw_ver)) {
		maxim_fwu_progress(50);
		if (device_fw_load(ts, fw, i, tagLen)) {
			pr_err("firmware download failed");
			return -1;
		}
		ret = 0;
	} else
		ret = 1;

	ENABLE_IRQ();
	if (change_touch_rpt(ts->client, PDATA(report_mode)) < 0) {
		pr_err("Failed to set up touch report mode");
	}
	maxim_fwu_progress(100);

	return ret;
}

int register_maxim_touch_fw_update(void)
{
	struct data *ts = gl_ts;
	maxim_tp_notifier.fwupdate = maxim_touch_fw_update;
	maxim_tp_notifier.flash_timeout = FW_FLASH_TIMEOUT;
	snprintf(maxim_tp_notifier.fw_vendor, sizeof(maxim_tp_notifier.fw_vendor), "%s", TOUCH_VENDOR);
	snprintf(maxim_tp_notifier.fw_ver, sizeof(maxim_tp_notifier.fw_ver), "%s", ts->fw_ver);
	return register_fw_update(&maxim_tp_notifier);
}
#endif

#ifdef CONFIG_OF
static u32 parse_eng_id(u32 eng_id_mask){
	u32 eng_id = 0;
	u8 temp[4] = {0};
	struct device_node *devnode = of_find_node_by_path("/chosen/mfg");

	if (devnode) {
		if (of_property_read_u8_array(devnode, "skuid.engineer_id", temp, sizeof(u32)/sizeof(u8)))
			pr_err(" %s, Failed to get property: engineer_id", __func__);
	} else {
		pr_err(" %s, Failed to find device node", __func__);
		return 0;
	}

	eng_id = temp[3] << 24 | temp[2] << 16 | temp[1] << 8 | temp[0];
	pr_debug(" %s, eng_id_mask = 0x%8X, temp = 0x%2X%2X%2X%2X, eng_id = 0x%X",
			__func__, eng_id_mask, temp[0], temp[1], temp[2], temp[3], eng_id);

	eng_id &= eng_id_mask;
	pr_info(" %s, eng_id = 0x%X", __func__, eng_id);
	return eng_id;
}

static void swap_buf(u16 *buf1, u16 *buf2, int len)
{
	int i;

	for (i=0; i<len; i++) {
		buf1[i] = (buf2[i]>>8) | (buf2[i]<<8);
	}
}

static int parse_config(struct device *dev, struct max1187x_pdata *pdata)
{
	struct max1187x_board_config *fw_config = NULL;
	struct device_node *devnode = dev->of_node;
	struct device_node *pp = NULL;
	struct property *prop;
	uint8_t cnt = 0, i = 0;
	u32 data = 0;
	int len = 0;
	u16 tmp_buf[80];
	u32 range[4];
//	const char *disp_panel;

	pr_info("(PROBE): %s", __func__);
	if (devnode == NULL) {
		pr_err(" %s, can't find device_node", __func__);
		return -ENODEV;
	}

	while ((pp = of_get_next_child(devnode, pp)))
		cnt++;

	if (!cnt)
		return -ENODEV;

	fw_config = kzalloc(cnt * (sizeof *fw_config), GFP_KERNEL);
	if (!fw_config)
		return -ENOMEM;

	pp = NULL;
	while ((pp = of_get_next_child(devnode, pp))) {
		if (of_property_read_u32(pp, "config_id", &data) == 0)
			fw_config[i].config_id = data;

		if (of_property_read_u32(pp, "chip_id", &data) == 0)
			fw_config[i].chip_id = data;

		if (of_property_read_u32(pp, "major_ver", &data) == 0)
			fw_config[i].major_ver = data;

		if (of_property_read_u32(pp, "minor_ver", &data) == 0)
			fw_config[i].minor_ver = data;

		if (of_property_read_u32(pp, "protocol_ver", &data) == 0)
			fw_config[i].protocol_ver = data;

		if (of_property_read_u32(pp, "vendor_pin", &data) == 0)
			fw_config[i].vendor_pin = data;

		if (of_property_read_u32(pp, "support_glove", &data) == 0)
			fw_config[i].support_glove = data;

		if (of_property_read_u32(pp, "support_cover", &data) == 0)
			fw_config[i].support_cover = data;

		if (of_property_read_u32(pp, "eng_id", &data) == 0)
			fw_config[i].eng_id = data;
		else
			fw_config[i].eng_id = 0;

/*		if (of_property_read_string(pp, "disp_panel", &disp_panel))
			memset(fw_config[i].disp_panel, 0, ARRAY_SIZE(fw_config[i].disp_panel));
		else
			scnprintf(fw_config[i].disp_panel, ARRAY_SIZE(fw_config[i].disp_panel), disp_panel);*/

		prop = of_find_property(pp, "filter_range", NULL);
		if (prop) {
			len = prop->length / sizeof(u32);
			if (len != 4) {
				pr_info("%s:Invalid filter_range size %d", __func__, len);
				goto err_max1187x_get_config_dt;
			}
			if (of_property_read_u32_array(pp, "filter_range", range, len) == 0) {
				fw_config[i].filter_range = kzalloc(4 * sizeof(u16), GFP_KERNEL);
				fw_config[i].filter_range[0] = range[0];
				fw_config[i].filter_range[1] = range[1];
				fw_config[i].filter_range[2] = range[2];
				fw_config[i].filter_range[3] = range[3];
				pr_info("%s: filter_range [%d][%d][%d][%d]", __func__, range[0], range[1], range[2], range[3]);
			} else
				fw_config[i].filter_range = NULL;
		} else
			fw_config[i].filter_range = NULL;

		prop = of_find_property(pp, "config_touch", &len);
		if (!prop) {
			pr_err(" %s:Looking up %s property in node %s failed",
				__func__, "config_touch", pp->full_name);
			goto err_max1187x_get_config_dt;
		} else if (!len) {
			pr_err(" %s:Invalid length of config_touch data\n",
				__func__);
			goto err_max1187x_get_config_dt;
		}
		memcpy(tmp_buf, prop->value, len);
		swap_buf(fw_config[i].config_touch, tmp_buf, len/2);

		prop = of_find_property(pp, "config_cal", &len);
		if (!prop) {
			pr_err(" %s:Looking up %s property in node %s failed",
				__func__, "config_cal", pp->full_name);
			goto err_max1187x_get_config_dt;
		} else if (!len) {
			pr_err(" %s:Invalid length of config_cal data\n",
				__func__);
			goto err_max1187x_get_config_dt;
		}
		memcpy(tmp_buf, prop->value, len);
		swap_buf(fw_config[i].config_cal, tmp_buf, len/2);

		prop = of_find_property(pp, "config_private", &len);
		if (!prop) {
			pr_err(" %s:Looking up %s property in node %s failed",
				__func__, "config_private", pp->full_name);
			goto err_max1187x_get_config_dt;
		} else if (!len) {
			pr_err(" %s:Invalid length of config_private data\n",
				__func__);
			goto err_max1187x_get_config_dt;
		}
		memcpy(tmp_buf, prop->value, len);
		swap_buf(fw_config[i].config_private, tmp_buf, len/2);

		prop = of_find_property(pp, "config_lin_x", &len);
		if (!prop) {
			pr_err(" %s:Looking up %s property in node %s failed",
				__func__, "config_lin_x", pp->full_name);
			goto err_max1187x_get_config_dt;
		} else if (!len) {
			pr_err(" %s:Invalid length of config_lin_x data\n",
				__func__);
			goto err_max1187x_get_config_dt;
		}
		memcpy(tmp_buf, prop->value, len);
		swap_buf(fw_config[i].config_lin_x, tmp_buf, len/2);

		prop = of_find_property(pp, "config_lin_y", &len);
		if (!prop) {
			pr_err(" %s:Looking up %s property in node %s failed",
				__func__, "config_lin_y", pp->full_name);
			goto err_max1187x_get_config_dt;
		} else if (!len) {
			pr_err(" %s:Invalid length of config_lin_y data\n",
				__func__);
			goto err_max1187x_get_config_dt;
		}
		memcpy(tmp_buf, prop->value, len);
		swap_buf(fw_config[i].config_lin_y, tmp_buf, len/2);
		i++;
	}

	pdata->fw_config = fw_config;

	return 0;

err_max1187x_get_config_dt:
	kfree(fw_config);
	return -ENODEV;
}

static struct max1187x_pdata *max1187x_get_platdata_dt(struct device *dev)
{
	struct max1187x_pdata *pdata = NULL;
	struct device_node *devnode = dev->of_node;
	u32 i, data;
	u32 datalist[MAX1187X_NUM_FW_MAPPINGS_MAX];

	pr_info("(PROBE): %s\n", __func__);
	if (!devnode)
		return NULL;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("Failed to allocate memory for pdata\n");
		return NULL;
	}

	/* Parse gpio_tirq */
	pdata->gpio_tirq = of_get_named_gpio(devnode, "gpio_tirq", 0);
	if (!gpio_is_valid(pdata->gpio_tirq)) {
		pr_err("Failed to get property: gpio_tirq\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse gpio_reset */
	pdata->gpio_reset = of_get_named_gpio(devnode, "gpio_reset", 0);
	if (!gpio_is_valid(pdata->gpio_reset)) {
		pr_err("Failed to get property: gpio_reset\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse gpio_3v3 */
	pdata->gpio_3v3 = of_get_named_gpio(devnode, "gpio_3v3", 0);
	if (!gpio_is_valid(pdata->gpio_3v3)) {
		pr_info("No property: gpio_3v3\n");
	}

	/* Parse gpio_switch */
	pdata->gpio_switch = of_get_named_gpio(devnode, "gpio_switch", 0);
	if (!gpio_is_valid(pdata->gpio_switch)) {
		pr_info("No property: gpio_switch\n");
	}

	pr_info("(PROBE): %s: gpio_tirq:%d, gpio_reset:%d, gpio_3v3:%d, gpio_switch:%d",
		__func__, pdata->gpio_tirq, pdata->gpio_reset, pdata->gpio_3v3, pdata->gpio_switch);
	/* Parse num_fw_mappings */
	if (of_property_read_u32(devnode, "num_fw_mappings",
		&pdata->num_fw_mappings)) {
		pr_err("Failed to get property: num_fw_mappings\n");
		goto err_max1187x_get_platdata_dt;
	}

	if (pdata->num_fw_mappings > MAX1187X_NUM_FW_MAPPINGS_MAX)
		pdata->num_fw_mappings = MAX1187X_NUM_FW_MAPPINGS_MAX;

	/* Parse chip_id */
	if (of_property_read_u32_array(devnode, "chip_id", datalist,
			pdata->num_fw_mappings)) {
		pr_err("Failed to get property: chip_id\n");
		goto err_max1187x_get_platdata_dt;
	}

	for (i = 0; i < pdata->num_fw_mappings; i++)
		pdata->fw_mapping[i].chip_id = datalist[i];

	/* Parse filename */
	for (i = 0; i < pdata->num_fw_mappings; i++) {
		if (of_property_read_string_index(devnode, "filename", i,
			(const char **) &pdata->fw_mapping[i].filename)) {
				pr_err("Failed to get property: "\
					"filename[%d]\n", i);
				goto err_max1187x_get_platdata_dt;
			}
	}

	/* Parse filesize */
	if (of_property_read_u32_array(devnode, "filesize", datalist,
		pdata->num_fw_mappings)) {
		pr_err("Failed to get property: filesize\n");
		goto err_max1187x_get_platdata_dt;
	}

	for (i = 0; i < pdata->num_fw_mappings; i++)
		pdata->fw_mapping[i].filesize = datalist[i];

	/* Parse file_codesize */
	if (of_property_read_u32_array(devnode, "file_codesize", datalist,
		pdata->num_fw_mappings)) {
		pr_err("Failed to get property: file_codesize\n");
		goto err_max1187x_get_platdata_dt;
	}

	for (i = 0; i < pdata->num_fw_mappings; i++)
		pdata->fw_mapping[i].file_codesize = datalist[i];

	/* Parse defaults_allow */
	if (of_property_read_u32(devnode, "defaults_allow",
		&pdata->defaults_allow)) {
		pr_err("Failed to get property: defaults_allow\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse default_config_id */
	if (of_property_read_u32(devnode, "default_config_id",
		&pdata->default_config_id)) {
		pr_err("Failed to get property: default_config_id\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse default_chip_id */
	if (of_property_read_u32(devnode, "default_chip_id",
		&pdata->default_chip_id)) {
		pr_err("Failed to get property: default_chip_id\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse i2c_words */
	if (of_property_read_u32(devnode, "i2c_words", &pdata->i2c_words)) {
		pr_err("Failed to get property: i2c_words\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse coordinate_settings */
	if (of_property_read_u32(devnode, "coordinate_settings",
		&pdata->coordinate_settings)) {
		pr_err("Failed to get property: coordinate_settings\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse panel_max_x */
	if (of_property_read_u32(devnode, "panel_max_x",
		&pdata->panel_max_x)) {
		pr_err("Failed to get property: panel_max_x\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse panel_min_x */
	if (of_property_read_u32(devnode, "panel_min_x",
		&pdata->panel_min_x)) {
		pr_err("Failed to get property: panel_min_x\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse panel_max_y */
	if (of_property_read_u32(devnode, "panel_max_y",
		&pdata->panel_max_y)) {
		pr_err("Failed to get property: panel_max_y\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse panel_min_y */
	if (of_property_read_u32(devnode, "panel_min_y",
		&pdata->panel_min_y)) {
		pr_err("Failed to get property: panel_min_y\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse lcd_x */
	if (of_property_read_u32(devnode, "lcd_x", &pdata->lcd_x)) {
		pr_err("Failed to get property: lcd_x\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse lcd_y */
	if (of_property_read_u32(devnode, "lcd_y", &pdata->lcd_y)) {
		pr_err("Failed to get property: lcd_y\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse row_count */
	if (of_property_read_u32(devnode, "num_rows",
		&pdata->num_rows)) {
		pr_err("Failed to get property: num_rows\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse num_cols */
	if (of_property_read_u32(devnode, "num_cols",
		&pdata->num_cols)) {
		pr_err("Failed to get property: num_cols\n");
		goto err_max1187x_get_platdata_dt;
	}

	if (of_property_read_u32(devnode, "tw_mask", &data)) {
		pr_err("Failed to get property: tw_mask\n");
		goto err_max1187x_get_platdata_dt;
	}
	pdata->tw_mask = (u16)data;

	/* Parse button_code0 */
	if (of_property_read_u32(devnode, "button_code0",
		&pdata->button_code0)) {
		pr_debug("Failed to get property: button_code0\n");
	}

	/* Parse button_code1 */
	if (of_property_read_u32(devnode, "button_code1",
		&pdata->button_code1)) {
		pr_debug("Failed to get property: button_code1\n");
	}

	/* Parse button_code2 */
	if (of_property_read_u32(devnode, "button_code2",
		&pdata->button_code2)) {
		pr_debug("Failed to get property: button_code2\n");
	}

	/* Parse button_code3 */
	if (of_property_read_u32(devnode, "button_code3",
		&pdata->button_code3)) {
		pr_debug("Failed to get property: button_code3\n");
	}

	if (of_property_read_u32(devnode, "input_protocol", &data)) {
		pr_err("Failed to get property: input_protocol\n");
		goto err_max1187x_get_platdata_dt;
	}
	pdata->input_protocol = (u8)data;

	if (of_property_read_u32(devnode, "update_feature", &data)) {
		pr_err("Failed to get property: update_feature\n");
		goto err_max1187x_get_platdata_dt;
	}
	pdata->update_feature = (u8)data;

	if (of_property_read_u32(devnode, "report_mode", &data)) {
		pr_err("Failed to get property: report_mode\n");
		goto err_max1187x_get_platdata_dt;
	}
	pdata->report_mode = (u8)data;

	if (of_property_read_u32(devnode, "hall_block_touch_time",
		&pdata->hall_block_touch_time)) {
		pdata->hall_block_touch_time = 0;
	} else
		pr_info("hall_block_touch_time = %d", pdata->hall_block_touch_time);

	/* Parse retry_config_update_delay */
	if (of_property_read_u32(devnode, "retry_config_update_delay",
		&pdata->retry_config_update_delay)) {
		pdata->retry_config_update_delay = 100;
	}
	pr_info("parse retry_config_update_delay = %d", pdata->retry_config_update_delay);

	/* Parse eng_id */
	if (of_property_read_u32(devnode, "eng_id", &pdata->eng_id) == 0) {
		pr_info("(INIT) eng_id = %d", pdata->eng_id);
	} else if (of_property_read_u32(devnode, "eng_id_mask", &pdata->eng_id_mask) == 0) {
		pdata->eng_id = parse_eng_id(pdata->eng_id_mask);
		pr_info("(INIT) parse eng_id = %d", pdata->eng_id);
	} else {
		pdata->eng_id = 0;
	}

	/* Parse disp_panel */
/*	if (of_property_read_bool(devnode, "disp_panel_check")) {
		scnprintf(pdata->disp_panel, ARRAY_SIZE(pdata->disp_panel), disp_vendor());
		pr_info("(INIT) parse disp_panel = %s", pdata->disp_panel);
		if (pdata->disp_panel)
			pdata->disp_panel_check = true;
		else
			pdata->disp_panel_check = false;
	} else
		pdata->disp_panel_check = false;	*/

	if (parse_config(dev, pdata)) {
		pr_err("Failed to parse config\n");
		goto err_max1187x_get_platdata_dt;
	}

	return pdata;

err_max1187x_get_platdata_dt:
	devm_kfree(dev, pdata);
	return NULL;
}

#else
static inline struct max1187x_pdata *
	max1187x_get_platdata_dt(struct device *dev)
{
	return NULL;
}
#endif


static int validate_pdata(struct max1187x_pdata *pdata)
{
	if (pdata == NULL) {
		pr_err("Platform data not found!\n");
		goto err_validate_pdata;
	}

	if (pdata->gpio_tirq == 0) {
		pr_err("gpio_tirq (%u) not defined!\n", pdata->gpio_tirq);
		goto err_validate_pdata;
	}

	if (pdata->num_rows == 0 || pdata->num_rows > 40) {
		pr_err("num_rows (%u) out of range!\n", pdata->num_rows);
		goto err_validate_pdata;
	}

	if (pdata->num_cols == 0 || pdata->num_cols > 40) {
		pr_err("num_cols (%u) out of range!\n", pdata->num_cols);
		goto err_validate_pdata;
	}

	return 0;

err_validate_pdata:
	return -ENXIO;
}

static int max1187x_pinctrl_init(struct device *dev)
{
	int ret;
	struct max1187x_pdata *pdata = dev_get_platdata(dev);

	pr_info("%s", __func__);
	/* Get pinctrl if target uses pinctrl */
	pdata->ts_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pdata->ts_pinctrl)) {
		pr_info("Target does not use pinctrl");
		ret = PTR_ERR(pdata->ts_pinctrl);
		pdata->ts_pinctrl = NULL;
		return ret;
	}

	pdata->gpio_state_active
		= pinctrl_lookup_state(pdata->ts_pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(pdata->gpio_state_active)) {
		pr_info("Can not get ts default pinstate");
		ret = PTR_ERR(pdata->gpio_state_active);
		pdata->ts_pinctrl = NULL;
		return ret;
	}

	pdata->gpio_state_suspend
		= pinctrl_lookup_state(pdata->ts_pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(pdata->gpio_state_suspend)) {
		pr_info("Can not get ts sleep pinstate");
		ret = PTR_ERR(pdata->gpio_state_suspend);
		pdata->ts_pinctrl = NULL;
		return ret;
	}

	return 0;
}

static void max1187x_pinctrl_deinit(struct device *dev)
{
	struct max1187x_pdata *pdata = dev_get_platdata(dev);

	pr_info("%s", __func__);
	/* Free pinctrl if target uses pinctrl */
	if (pdata->ts_pinctrl != NULL) {
		pdata->gpio_state_active = NULL;
		pdata->gpio_state_suspend = NULL;
		devm_pinctrl_put(pdata->ts_pinctrl);
	}
}

static int max1187x_pinctrl_select(struct device *dev, int on)
{
	struct pinctrl_state *pins_state;
	struct max1187x_pdata *pdata = dev_get_platdata(dev);
	int ret;

	pr_info("%s: set to %d", __func__, on);
	pins_state = on ? pdata->gpio_state_active
		: pdata->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(pdata->ts_pinctrl, pins_state);
		if (ret) {
			pr_err("can not set %s pins",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else
		pr_err("not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");

	return 0;
}

static int max1187x_chip_init(struct max1187x_pdata *pdata, int value)
{
	int  ret;

	pr_info("(PROBE): max1187x_%s: %d", __func__, value);
	if (value) {
		if (gpio_is_valid(pdata->gpio_switch)) {
			ret = gpio_request(pdata->gpio_switch, "max1187x_switch");
			if (ret) {
				pr_info("gpio request failed for max1187x_switch (%d)\n",
					pdata->gpio_switch);
			}
			if (pdata->gpio_switch) {
				gpio_direction_output(pdata->gpio_switch, 0);
			}
		}

		if (gpio_is_valid(pdata->gpio_3v3)) {
			ret = gpio_request(pdata->gpio_3v3, "max1187x_pwr_3v3");
			if (ret) {
				pr_err("GPIO request failed for max1187x_pwr_3v3 (%d)\n",
					pdata->gpio_3v3);
				return -EIO;
			}
			if (pdata->gpio_3v3) {
				gpio_direction_output(pdata->gpio_3v3, 1);
				mdelay(10);
			}
		}

		if (gpio_is_valid(pdata->gpio_reset)) {
			ret = gpio_request(pdata->gpio_reset, "max1187x_reset");
			if (ret) {
				pr_err("GPIO request failed for max1187x_reset (%d)\n",
					pdata->gpio_reset);
				return -EIO;
			}
			if (pdata->gpio_reset) {
				gpio_direction_output(pdata->gpio_reset, 1);
				mdelay(10);
			}
		}

		ret = gpio_request(pdata->gpio_tirq, "max1187x_tirq");
		if (ret) {
			pr_info("GPIO request failed for max1187x_tirq (%d)\n",
				pdata->gpio_tirq);
			return -EIO;
		}
		ret = gpio_direction_input(pdata->gpio_tirq);
		if (ret) {
			pr_err("GPIO set input direction failed for "\
				"max1187x_tirq (%d)\n", pdata->gpio_tirq);
			gpio_free(pdata->gpio_tirq);
			return -EIO;
		}


	} else {
		gpio_free(pdata->gpio_tirq);
		if (gpio_is_valid(pdata->gpio_reset))
			gpio_free(pdata->gpio_reset);
		if (gpio_is_valid(pdata->gpio_3v3))
			gpio_free(pdata->gpio_3v3);
		if (gpio_is_valid(pdata->gpio_switch))
			gpio_free(pdata->gpio_switch);
	}

	return 0;
}

static int device_init_thread(void *arg)
{
	return device_init((struct i2c_client *) arg);
}

static int device_init(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct data *ts = NULL;
	struct max1187x_pdata *pdata = NULL;
	struct device_attribute **dev_attr = dev_attrs;
	int ret = 0;

	init_state = 1;
	dev_info(dev, "(INIT): Start");

	/* allocate control block; nothing more to do if we can't */
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		pr_err("Failed to allocate control block memory");
		ret = -ENOMEM;
		goto err_device_init;
	}

	pdata = dev_get_platdata(dev);

	/* Validate if pdata values are okay */
	ret = validate_pdata(pdata);
	if (ret < 0)
		goto err_device_init_pdata;
	pr_info_if(8, "(INIT): Platform data OK");

	ts->pdata = pdata;
	ts->fw_config = pdata->fw_config;
	ts->client = client;
	i2c_set_clientdata(client, ts);
	mutex_init(&ts->irq_mutex);
	mutex_init(&ts->i2c_mutex);
	mutex_init(&ts->report_mutex);
	sema_init(&ts->report_sem, 1);
	ts->fw_index = -1;
	ts->noise_level = 0;
	ts->baseline_mode = MAX1187X_AUTO_BASELINE;
	ts->button0 = 0;
	ts->button1 = 0;
	ts->button2 = 0;
	ts->button3 = 0;
	ts->hall_block_touch_event = 0;
	if (PDATA(button_data))
		ts->button_data = PDATA(button_data);
	if (!PDATA(report_mode))
		PDATA(report_mode) = MAX1187X_REPORT_MODE_BASIC;

	atomic_set(&ts->scheduled_work_irq, 0);

	pr_info_if(8, "(INIT): Memory allocation OK");

	//if (get_tamper_sf()==0) {
		debug_mask |= BIT(3);
		pr_info_if(8, "(INIT): Debug level=0x%08X", debug_mask);
	//}

	/* Setup IRQ and handler */
	if (request_threaded_irq(client->irq, NULL, irq_handler,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ts) != 0) {
			pr_err("Failed to setup IRQ handler");
			ret = -EIO;
			goto err_device_init_gpio;
	}
	pr_info_if(8, "(INIT): IRQ handler OK");

	/* collect controller ID and configuration ID data from firmware   */
	/* and perform firmware comparison/download if we have valid image */
	check_fw_and_config(ts);

	/* allocate and register touch device */
	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		pr_err("Failed to allocate touch input device");
		ret = -ENOMEM;
		goto err_device_init_alloc_inputdev;
	}
	snprintf(ts->phys, sizeof(ts->phys), "%s/input0",
			dev_name(dev));
	ts->input_dev->name = MAX1187X_TOUCH;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	if (PDATA(input_protocol) == MAX1187X_PROTOCOL_B) {
		input_mt_init_slots(ts->input_dev, MAX1187X_TOUCH_COUNT_MAX, 0);
		if (ts->fw_config->support_glove)
			input_set_abs_params(ts->input_dev, ABS_MT_GLOVE, 0,
				1, 0, 0);
	} else {
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0,
			10, 0, 0);
	}
	if (PDATA(input_protocol) == MAX1187X_PROTOCOL_CUSTOM1 || PDATA(support_htc_event)) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
		input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE, 0,
			0xFF14, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_POSITION, 0,
			((1 << 31) | (PDATA(panel_max_x) << 16) | PDATA(panel_max_y)), 0, 0);
#endif
	}
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			PDATA(panel_min_x), PDATA(panel_max_x), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			PDATA(panel_min_y), PDATA(panel_max_y), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 0xFFFF, 0, 0);
	if (PDATA(report_mode) == MAX1187X_REPORT_MODE_EXTEND)
		input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 0x64, 0, 0);
#if 0
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
			0, max(PDATA(panel_max_x)-PDATA(panel_min_x),
			PDATA(panel_max_y)-PDATA(panel_min_y)), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MINOR,
			0, min(PDATA(panel_max_x)-PDATA(panel_min_x),
			PDATA(panel_max_y)-PDATA(panel_min_y)), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_ORIENTATION, -90, 90, 0, 0);
#endif
	if (PDATA(button_code0) != KEY_RESERVED)
		set_bit(pdata->button_code0, ts->input_dev->keybit);
	if (PDATA(button_code1) != KEY_RESERVED)
		set_bit(pdata->button_code1, ts->input_dev->keybit);
	if (PDATA(button_code2) != KEY_RESERVED)
		set_bit(pdata->button_code2, ts->input_dev->keybit);
	if (PDATA(button_code3) != KEY_RESERVED)
		set_bit(pdata->button_code3, ts->input_dev->keybit);

	if (input_register_device(ts->input_dev)) {
		pr_err("Failed to register touch input device");
		ret = -EPERM;
		goto err_device_init_register_inputdev;
	}
	pr_info_if(8, "(INIT): Input touch device OK");

	if (ts->fw_config->support_glove) {
		set_glove_mode(ts->client, 0);
		ts->glove_enable = 0;
		ts->glove_setting = 0;
		pr_info("(INIT): glove mode default off");
	}

	if (PDATA(panel_max_x)-PDATA(panel_min_x)!=0 &&
		PDATA(panel_max_y)-PDATA(panel_min_y)!=0) {
		ts->width_factor = (PDATA(lcd_x)<<SHIFT_BITS) /
				(PDATA(panel_max_x)-PDATA(panel_min_x));
		ts->height_factor = (PDATA(lcd_y)<<SHIFT_BITS) /
				(PDATA(panel_max_y)-PDATA(panel_min_y));
		ts->width_offset = PDATA(panel_min_x);
		ts->height_offset = PDATA(panel_min_y);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	/* configure suspend/resume */
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = early_suspend;
	ts->early_suspend.resume = late_resume;
	register_early_suspend(&ts->early_suspend);
	ts->early_suspend_registered = 1;
#endif
#ifdef CONFIG_FB
	ts->fb_notifier.notifier_call = fb_notifier_callback;
	fb_register_client(&ts->fb_notifier);
#endif
	pr_info_if(8, "(INIT): suspend/resume registration OK");

	gl_ts = ts;
	/* set up debug interface */
	if (sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr) < 0) {
		pr_err("failed to create sysfs file [debug_level]");
		return 0;
	}
	if (sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr) < 0) {
		pr_err("failed to create sysfs file [vendor]");
		return 0;
	}
	if (sysfs_create_file(android_touch_kobj, &dev_attr_config.attr) < 0) {
		pr_err("failed to create sysfs file [config]");
		return 0;
	}
	if (sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr) < 0) {
		pr_err("failed to create sysfs file [gpio]");
		return 0;
	}
	if (sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr) < 0) {
		pr_err("failed to create sysfs file [diag]");
		return 0;
	}
	if (sysfs_create_file(android_touch_kobj, &dev_attr_unlock.attr) < 0) {
		pr_err("failed to create sysfs file [unlock]");
		return 0;
	}
	if (sysfs_create_file(android_touch_kobj, &dev_attr_reset.attr) < 0) {
		pr_err("failed to create sysfs file [reset]");
		return 0;
	}
	if (sysfs_create_file(android_touch_kobj, &dev_attr_enabled.attr) < 0) {
		pr_err("failed to create sysfs file [enable]");
		return 0;
	}
	if (sysfs_create_file(android_touch_kobj, &dev_attr_sr_en.attr) < 0) {
		pr_err("failed to create sysfs file [sr_en]");
		return 0;
	}
	if (sysfs_create_file(android_touch_kobj, &dev_attr_cover.attr) < 0) {
		pr_err("failed to create sysfs file [cover]");
		return 0;
	}
	if (ts->fw_config->support_glove) {
		if (sysfs_create_file(android_touch_kobj, &dev_attr_glove_setting.attr) < 0) {
			pr_err("failed to create sysfs file [glove_setting]");
			return 0;
		}
	}
	while (*dev_attr) {
		if (device_create_file(&client->dev, *dev_attr) < 0) {
			pr_err("failed to create sysfs file");
			return 0;
		}
		ts->sysfs_created++;
		dev_attr++;
	}

	if (device_create_bin_file(&client->dev, &dev_attr_report) < 0) {
		pr_err("failed to create sysfs file [report]");
		return 0;
	}
	ts->sysfs_created++;

#ifdef CONFIG_AK8789_HALLSENSOR
	hallsensor_register_notifier(&hallsensor_status_handler);
#endif
#ifdef CONFIG_TOUCHSCREEN_TOUCH_FW_UPDATE
	register_maxim_touch_fw_update();
#endif

#ifdef CONFIG_SYNC_TOUCH_STATUS
	touch_solution(0);
	pr_info("(INIT) Register to sensor_hub driver");
#endif

	pr_info("(INIT): Done");
	return 0;

err_device_init_register_inputdev:
	input_free_device(ts->input_dev);
	ts->input_dev = NULL;
err_device_init_alloc_inputdev:
err_device_init_gpio:
err_device_init_pdata:
	kfree(ts);
err_device_init:
	return ret;
}

static int device_deinit(struct i2c_client *client)
{
	struct data *ts = i2c_get_clientdata(client);
	struct device_attribute **dev_attr = dev_attrs;

	if (ts == NULL)
		return 0;

#ifdef CONFIG_TOUCHSCREEN_TOUCH_FW_UPDATE
	unregister_fw_update();
#endif
#ifdef CONFIG_AK8789_HALLSENSOR
	hallsensor_unregister_notifier(&hallsensor_status_handler);
#endif

	propagate_report(ts, -1, NULL);

	init_state = 0;
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_config.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_unlock.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_reset.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_enabled.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_sr_en.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_cover.attr);
	if (ts->fw_config->support_glove)
		sysfs_remove_file(android_touch_kobj, &dev_attr_glove_setting.attr);
	while (*dev_attr) {
		if (ts->sysfs_created && ts->sysfs_created--)
			device_remove_file(&client->dev, *dev_attr);
		dev_attr++;
	}
	if (ts->sysfs_created && ts->sysfs_created--)
		device_remove_bin_file(&client->dev, &dev_attr_report);

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (ts->early_suspend_registered)
		unregister_early_suspend(&ts->early_suspend);
#endif
#ifdef CONFIG_FB
	fb_unregister_client(&ts->fb_notifier);
#endif

	if (ts->input_dev)
		input_unregister_device(ts->input_dev);

	if (client->irq)
		free_irq(client->irq, ts);
	kfree(ts);

	pr_info("(INIT): Deinitialized\n");
	return 0;
}

static int check_chip_exist(struct i2c_client *client)
{
	char buf[32];
	int read_len = 0, i;

	/* if I2C functionality is not present we are done */
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("I2C core driver does not support I2C functionality");
		return -1;
	}

	for (i=0; i<RETRY_TIMES; i++) {
		if (i != 0)
			msleep(20);
		buf[0] = 0x0A;
		buf[1] = 0x00;
		if (i2c_master_send(client, buf, 2) <= 0)
			continue;
		if (i2c_master_recv(client, buf, 2) <= 0)
			continue;
		read_len = (buf[0] + 1) << 1;
		if (read_len>10)
			read_len = 10;
		if (i2c_master_recv(client, buf, read_len) <= 0)
			continue;
		break;
	}
	if (i == RETRY_TIMES) {
		pr_info("(PROBE) No Maxim chip");
		return -1;
	}

	pr_info("(PROBE): I2C functionality OK");
	pr_info("(PROBR): Chip exist");
	return 0;
}

/*static void off_mode_suspend(struct i2c_client *client)
{
	char data[] = {0x00, 0x00, 0x03, 0x11, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
	int ret;
	pr_info("(PROBE): Set touch chip to sleep mode and skip touch driver probe");
	do {ret = i2c_master_send(client, data, 10);
	} while (ret == -EAGAIN);
	if(ret <= 0)
		pr_err("Send Suspend Commamd Failed");
}*/

static int probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max1187x_pdata *pdata = NULL;
	pr_info("(PROBE): max1187x_%s Enter", __func__);

	/* Get platform data */
	if (client->dev.of_node) {
		pdata = max1187x_get_platdata_dt(dev);
		dev->platform_data = pdata;
		if (pdata == NULL)
			return -ENOMEM;
	} else
		pdata = client->dev.platform_data;

	if (max1187x_pinctrl_init(dev) != 0) {
		pr_info("Can't use pinctrl");
	} else {
		max1187x_pinctrl_select(dev, 1);
	}

	/* Initialize GPIO pins */
	if (max1187x_chip_init(pdata, 1) < 0) {
		goto err_chip_init;
	}

	pr_info_if(8, "(PROBE): chip init OK");
	if (check_chip_exist(client) < 0) {
		goto err_chip_exist;
	}

/*	if ((strcmp(htc_get_bootmode(), "offmode_charging") == 0)
		|| (strcmp(htc_get_bootmode(), "recovery") == 0)) {
		pr_info("(PROBE): %s mode", htc_get_bootmode());
		off_mode_suspend(client);
		goto err_off_mode;
	}*/

	if(support_htc_event_flag == 1) {
		pdata->support_htc_event = 1;
		pr_info("support_htc_event = %d", pdata->support_htc_event);
	}

	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		pr_err("failed to create kobj");
		return 0;
	}
	if (device_create_file(&client->dev, &dev_attr_init) < 0) {
		pr_err("failed to create sysfs file [init]");
		return 0;
	}
	if (sysfs_create_link(android_touch_kobj, &client->dev.kobj, "maxim1187x") < 0) {
		pr_err("failed to create link");
		return 0;
	}

	if (!is_booting())
		return device_init(client);
	if (IS_ERR(kthread_run(device_init_thread, (void *) client,
			MAX1187X_NAME))) {
		pr_err("failed to start kernel thread");
		max1187x_chip_init(pdata, 0);
		max1187x_pinctrl_deinit(dev);
		return -EAGAIN;
	}
	return 0;

//err_off_mode:
err_chip_exist:
	max1187x_chip_init(pdata, 0);
err_chip_init:
	max1187x_pinctrl_deinit(dev);
	return -EIO;
}

static int remove(struct i2c_client *client)
{
	struct data *ts = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	struct max1187x_pdata *pdata = ts->pdata;
	int ret = device_deinit(client);

	max1187x_chip_init(pdata, 0);
	if (pdata->ts_pinctrl)
		max1187x_pinctrl_select(dev, 0);
	max1187x_pinctrl_deinit(dev);
	sysfs_remove_link(android_touch_kobj, "maxim1187x");
	device_remove_file(&client->dev, &dev_attr_init);
	kobject_del(android_touch_kobj);
	return ret;
}

/*
 COMMANDS
 */
static int sreset(struct i2c_client *client)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 data[] = { 0x00E9, 0x0000 };
	return send_mtp_command(ts, data, NWORDS(data));
}

static int get_touch_config(struct i2c_client *client)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 data[] = { 0x0002, 0x0000 };
	return send_mtp_command(ts, data, NWORDS(data));
}

static int get_fw_version(struct i2c_client *client)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 data[] = { 0x0040, 0x0000 };
	return send_mtp_command(ts, data, NWORDS(data));
}

static int change_touch_rpt(struct i2c_client *client, u16 to)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 data[] = { 0x0018, 0x0001, to & 0x0003 };
	return send_mtp_command(ts, data, NWORDS(data));
}

static int set_touch_frame(struct i2c_client *client,
			u16 idle_frame, u16 active_frame)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 data[] = {0x0026, 0x0001,
		(((idle_frame & 0xFF) << 8) | (active_frame & 0xFF))};
	return send_mtp_command(ts, data, NWORDS(data));
}

static int set_baseline_mode(struct i2c_client *client, u16 mode)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 data[] = {0x0028, 0x0001, mode & 0x0003};
	return send_mtp_command(ts, data, NWORDS(data));
}

static int set_cover_mode(struct i2c_client *client, u16 enable)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 data[] = {0x0088, 0x0001, (enable & 0x1)};
	return send_mtp_command(ts, data, NWORDS(data));
}

static int set_glove_mode(struct i2c_client *client, u16 enable)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 data[] = {0x0083, 0x0002, 0x0000, (enable & 0x1)};
	return send_mtp_command(ts, data, NWORDS(data));
}

static int set_edge_filter(struct i2c_client *client, u16 enable, u16 *range)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 len = (range==NULL) ? 1:5;
	u16 cmd[] = {0x0089, len, enable};
	u16 data[len+2];

	memcpy(data, cmd, sizeof(cmd));
	if (range != NULL) {
		memcpy(&data[3], range, sizeof(u16)*(len-1));
	}
	return send_mtp_command(ts, data, NWORDS(data));
}

static int combine_multipacketreport(struct data *ts, u16 *report)
{
	u16 packet_header = report[0];
	u8 packet_seq_num = BYTEH(packet_header);
	u8 packet_size = BYTEL(packet_header);
	u16 total_packets, this_packet_num, offset;
	static u16 packet_seq_combined;

	if (packet_seq_num == 0x11) {
		memcpy(ts->rx_report, report, (packet_size + 1) << 1);
		ts->rx_report_len = packet_size;
		packet_seq_combined = 1;
		return 0;
	}

	total_packets = (packet_seq_num & 0xF0) >> 4;
	this_packet_num = packet_seq_num & 0x0F;

	if (this_packet_num == 1) {
		if (report[1] == 0x0800) {
			ts->rx_report_len = report[2] + 2;
			packet_seq_combined = 1;
			memcpy(ts->rx_report, report, (packet_size + 1) << 1);
			return -EAGAIN;
		} else {
			return -EIO;
		}
	} else if (this_packet_num == packet_seq_combined + 1) {
		packet_seq_combined++;
		offset = (this_packet_num - 1) * 0xF4 + 1;
		memcpy(ts->rx_report + offset, report + 1, packet_size << 1);
		if (total_packets == this_packet_num)
			return 0;
		else
			return -EIO;
	}
	return -EIO;
}

static void propagate_report(struct data *ts, int status, u16 *report)
{
	int i, ret;

	down(&ts->report_sem);
	mutex_lock(&ts->report_mutex);

	if (report) {
		ret = combine_multipacketreport(ts, report);
		if (ret) {
			up(&ts->report_sem);
			mutex_unlock(&ts->report_mutex);
			return;
		}
	}

	for (i = 0; i < MAX_REPORT_READERS; i++) {
		if (status == 0) {
			if (ts->report_readers[i].report_id == 0xFFFF
				|| (ts->rx_report[1] != 0
				&& ts->report_readers[i].report_id
				== ts->rx_report[1])) {
				up(&ts->report_readers[i].sem);
				ts->report_readers[i].reports_passed++;
				ts->report_readers_outstanding++;
			}
		} else {
			if (ts->report_readers[i].report_id != 0) {
				ts->report_readers[i].status = status;
				up(&ts->report_readers[i].sem);
			}
		}
	}
	if (ts->report_readers_outstanding == 0) {
		up(&ts->report_sem);
	}
	mutex_unlock(&ts->report_mutex);
}

static int get_report(struct data *ts, u16 report_id, ulong timeout)
{
	int i, ret, status;

	mutex_lock(&ts->report_mutex);
	for (i = 0; i < MAX_REPORT_READERS; i++)
		if (ts->report_readers[i].report_id == 0)
			break;
	if (i == MAX_REPORT_READERS) {
		mutex_unlock(&ts->report_mutex);
		ENABLE_IRQ();
		pr_err("maximum readers reached");
		return -EBUSY;
	}
	ts->report_readers[i].report_id = report_id;
	sema_init(&ts->report_readers[i].sem, 1);
	down(&ts->report_readers[i].sem);
	ts->report_readers[i].status = 0;
	ts->report_readers[i].reports_passed = 0;
	mutex_unlock(&ts->report_mutex);
	ENABLE_IRQ();

	if (timeout == 0xFFFFFFFF)
		ret = down_interruptible(&ts->report_readers[i].sem);
	else
		ret = down_timeout(&ts->report_readers[i].sem,
			(timeout * HZ) / 1000);

	mutex_lock(&ts->report_mutex);
	if (ret && ts->report_readers[i].reports_passed > 0)
		if (--ts->report_readers_outstanding == 0)
			up(&ts->report_sem);
	status = ts->report_readers[i].status;
	ts->report_readers[i].report_id = 0;
	mutex_unlock(&ts->report_mutex);

	return (status == 0) ? ret : status;
}

static void release_report(struct data *ts)
{
	mutex_lock(&ts->report_mutex);
	if (--ts->report_readers_outstanding == 0)
		up(&ts->report_sem);
	mutex_unlock(&ts->report_mutex);
}

#ifdef CONFIG_SYNC_TOUCH_STATUS
static void free_finger(struct data *ts)
{
	int i;

	if (ts->vk_press) {
		//pr_info("%s: free vkey", __func__);
		if (ts->button0) {
			if (ts->button_data)
				button_report(ts, 0, ts->button0);
			else {
				input_report_key(ts->input_dev, PDATA(button_code0), ts->button0);
				input_sync(ts->input_dev);
			}
			ts->button0 = 0;
		}
		if (ts->button1) {
			if (ts->button_data)
				button_report(ts, 0, ts->button1);
			else {
				input_report_key(ts->input_dev, PDATA(button_code1), ts->button1);
				input_sync(ts->input_dev);
			}
			ts->button1 = 0;
		}
		if (ts->button2) {
			if (ts->button_data)
				button_report(ts, 0, ts->button2);
			else {
				input_report_key(ts->input_dev, PDATA(button_code2), ts->button2);
				input_sync(ts->input_dev);
			}
			ts->button2 = 0;
		}
		if (ts->button3) {
			if (ts->button_data)
				button_report(ts, 0, ts->button3);
			else {
				input_report_key(ts->input_dev, PDATA(button_code3), ts->button3);
				input_sync(ts->input_dev);
			}
			ts->button3 = 0;
		}
		ts->vk_press = 0;
	}

	if (ts->finger_press) {
		//pr_info("%s: free finger", __func__);
		switch (PDATA(input_protocol)) {
			case MAX1187X_PROTOCOL_A:
				if (PDATA(support_htc_event)) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif
				}
				input_mt_sync(ts->input_dev);
				break;
			case MAX1187X_PROTOCOL_B:
				for (i = 0; i < MAX1187X_TOUCH_COUNT_MAX; i++) {
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
					if (ts->glove_status & (1 << i)) {
						input_report_abs(ts->input_dev, ABS_MT_GLOVE, 1);
					} else {
						input_report_abs(ts->input_dev, ABS_MT_GLOVE, 0);
					}

					if (PDATA(support_htc_event)) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
						input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
						input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif
					}
				}
				break;
		}
		input_sync(ts->input_dev);
		ts->finger_press = 0;
		ts->glove_status = 0;
	}
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void early_suspend(struct early_suspend *h)
{
	u16 data[] = {0x0020, 0x0001, 0x0000};
	struct data *ts;
	ts = container_of(h, struct data, early_suspend);

	pr_info("max1187x_%s", __func__);
	DISABLE_IRQ();
	(void)send_mtp_command(ts, data, NWORDS(data));
	pr_info("max1187x_%s: sleep", __func__);
#ifdef CONFIG_SYNC_TOUCH_STATUS
	switch_sensor_hub(ts, 1);
#endif
}

static void late_resume(struct early_suspend *h)
{
	u16 data[] = {0x0020, 0x0001, 0x0002};
	struct data *ts;
	ts = container_of(h, struct data, early_suspend);

	pr_info("max1187x_%s", __func__);

#ifdef CONFIG_SYNC_TOUCH_STATUS
	free_finger(ts);
	switch_sensor_hub(ts, 0);
#endif
	ENABLE_IRQ();

	(void)send_mtp_command(ts, data, NWORDS(data));
	pr_info("max1187x_%s: wake up", __func__);

	(void)change_touch_rpt(ts->client, PDATA(report_mode));
	pr_info("max1187x_%s: change report type:%d", __func__, PDATA(report_mode));

	set_chip_mode(ts);
}
#endif
#ifdef CONFIG_SYNC_TOUCH_STATUS
static void switch_sensor_hub(struct data *ts, int mode)
{
	int mask = 0;

	pr_info("%s: %d", __func__, mode);
	if (ts->fw_config->support_glove) {
		mask = (ts->glove_setting & 0x01) << 1;
	}

	if (gpio_is_valid(PDATA(gpio_switch))) {
		switch(mode) {
		case 0:
			touch_status(0 | mask);
			gpio_direction_output(PDATA(gpio_switch), 0);
			ts->i2c_to_mcu = 0;
			pr_info("[SensorHub] Switch touch i2c to CPU");
			break;
		case 1:
			gpio_direction_output(PDATA(gpio_switch), 1);
			ts->i2c_to_mcu = 1;
			pr_info("[SensorHub] Switch touch i2c to MCU");
			touch_status(1 | mask);
			break;
		}
	}
}
#endif
#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event  *evdata = data;
	int              *blank;
	struct data  *ts = container_of(self,
					    struct data, fb_notifier);

	pr_info("%s, event = %ld", __func__, event);
	if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			late_resume(&ts->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			early_suspend(&ts->client->dev);
	}
	return 0;
}

static void early_suspend(struct device *dev)
{
	u16 data[] = {0x0020, 0x0001, 0x0000};
	struct data *ts = dev_get_drvdata(dev);

	pr_info("max1187x_%s", __func__);
	DISABLE_IRQ();
	(void)send_mtp_command(ts, data, NWORDS(data));
	pr_info("max1187x_%s: sleep", __func__);
#ifdef CONFIG_SYNC_TOUCH_STATUS
	switch_sensor_hub(ts, 1);
#endif
}

static void late_resume(struct device *dev)
{
	u16 data[] = {0x0020, 0x0001, 0x0002};
	struct data *ts = dev_get_drvdata(dev);

	pr_info("max1187x_%s", __func__);

#ifdef CONFIG_SYNC_TOUCH_STATUS
	free_finger(ts);
	switch_sensor_hub(ts, 0);
#endif
	ENABLE_IRQ();

	(void)send_mtp_command(ts, data, NWORDS(data));
	pr_info("max1187x_%s: wake up", __func__);

	(void)change_touch_rpt(ts->client, PDATA(report_mode));
	pr_info("max1187x_%s: change report type:%d", __func__, PDATA(report_mode));

	set_chip_mode(ts);
}
#endif

#define STATUS_ADDR_H 0x00
#define STATUS_ADDR_L 0xFF
#define DATA_ADDR_H   0x00
#define DATA_ADDR_L   0xFE
#define STATUS_READY_H 0xAB
#define STATUS_READY_L 0xCC
#define RXTX_COMPLETE_H 0x54
#define RXTX_COMPLETE_L 0x32
static int bootloader_read_status_reg(struct data *ts, const u8 byteL,
	const u8 byteH)
{
	u8 buffer[] = { STATUS_ADDR_L, STATUS_ADDR_H }, i;

	for (i = 0; i < 3; i++) {
		if (i2c_tx_bytes(ts, buffer, 2) != 2) {
			pr_err("TX fail");
			return -EIO;
		}
		if (i2c_rx_bytes(ts, buffer, 2) != 2) {
			pr_err("RX fail");
			return -EIO;
		}
		if (buffer[0] == byteL && buffer[1] == byteH)
			break;
	}
	if (i == 3) {
		pr_err("Unexpected status => %02X%02X vs %02X%02X",
				buffer[0], buffer[1], byteL, byteH);
		return -EIO;
	}

	return 0;
}

static int bootloader_write_status_reg(struct data *ts, const u8 byteL,
	const u8 byteH)
{
	u8 buffer[] = { STATUS_ADDR_L, STATUS_ADDR_H, byteL, byteH };

	if (i2c_tx_bytes(ts, buffer, 4) != 4) {
		pr_err("TX fail");
		return -EIO;
	}
	return 0;
}

static int bootloader_rxtx_complete(struct data *ts)
{
	return bootloader_write_status_reg(ts, RXTX_COMPLETE_L,
				RXTX_COMPLETE_H);
}

static int bootloader_read_data_reg(struct data *ts, u8 *byteL, u8 *byteH)
{
	u8 buffer[] = { DATA_ADDR_L, DATA_ADDR_H, 0x00, 0x00 };

	if (i2c_tx_bytes(ts, buffer, 2) != 2) {
		pr_err("TX fail");
		return -EIO;
	}
	if (i2c_rx_bytes(ts, buffer, 4) != 4) {
		pr_err("RX fail");
		return -EIO;
	}
	if (buffer[2] != 0xCC && buffer[3] != 0xAB) {
		pr_err("Status is not ready");
		return -EIO;
	}

	*byteL = buffer[0];
	*byteH = buffer[1];
	return bootloader_rxtx_complete(ts);
}

static int bootloader_write_data_reg(struct data *ts, const u8 byteL,
	const u8 byteH)
{
	u8 buffer[6] = { DATA_ADDR_L, DATA_ADDR_H, byteL, byteH,
			RXTX_COMPLETE_L, RXTX_COMPLETE_H };

	if (bootloader_read_status_reg(ts, STATUS_READY_L,
		STATUS_READY_H) < 0) {
		pr_err("read status register fail");
		return -EIO;
	}
	if (i2c_tx_bytes(ts, buffer, 6) != 6) {
		pr_err("TX fail");
		return -EIO;
	}
	return 0;
}

static int bootloader_rxtx(struct data *ts, u8 *byteL, u8 *byteH,
	const int tx)
{
	if (tx > 0) {
		if (bootloader_write_data_reg(ts, *byteL, *byteH) < 0) {
			pr_err("write data register fail");
			return -EIO;
		}
		return 0;
	}

	if (bootloader_read_data_reg(ts, byteL, byteH) < 0) {
		pr_err("read data register fail");
		return -EIO;
	}
	return 0;
}

static int bootloader_get_cmd_conf(struct data *ts, int retries)
{
	u8 byteL, byteH;

	do {
		if (bootloader_read_data_reg(ts, &byteL, &byteH) >= 0) {
			if (byteH == 0x00 && byteL == 0x3E)
				return 0;
		}
		retries--;
	} while (retries > 0);

	return -EIO;
}

static int bootloader_write_buffer(struct data *ts, u8 *buffer, int size)
{
	u8 byteH = 0x00;
	int k;

	for (k = 0; k < size; k++) {
		if (bootloader_rxtx(ts, &buffer[k], &byteH, 1) < 0) {
			pr_err("bootloader RX-TX fail");
			return -EIO;
		}
	}
	return 0;
}

static int bootloader_enter(struct data *ts)
{
	int i;
	u16 enter[3][2] = { { 0x7F00, 0x0047 }, { 0x7F00, 0x00C7 }, { 0x7F00,
			0x0007 } };

	DISABLE_IRQ();
	for (i = 0; i < 3; i++) {
		if (i2c_tx_words(ts, enter[i], 2) != 2) {
			ENABLE_IRQ();
			pr_err("Failed to enter bootloader");
			return -EIO;
		}
	}

	if (bootloader_get_cmd_conf(ts, 5) < 0) {
		ENABLE_IRQ();
		pr_err("Failed to enter bootloader mode");
		return -EIO;
	}
	bootloader = 1;
	return 0;
}

static int bootloader_exit(struct data *ts)
{
	int i;
	u16 exit[3][2] = { { 0x7F00, 0x0040 }, { 0x7F00, 0x00C0 }, { 0x7F00,
			0x0000 } };

	bootloader = 0;
	ts->got_report = 0;
	for (i = 0; i < 3; i++) {
		if (i2c_tx_words(ts, exit[i], 2) != 2) {
			pr_err("Failed to exit bootloader");
			return -EIO;
		}
	}
	return 0;
}

static int bootloader_get_crc(struct data *ts, u16 *crc16,
		u16 addr, u16 len, u16 delay)
{
	u8 crc_command[] = {0x30, 0x02, BYTEL(addr),
			BYTEH(addr), BYTEL(len), BYTEH(len)};
	u8 byteL = 0, byteH = 0;
	u16 rx_crc16 = 0;

	if (bootloader_write_buffer(ts, crc_command, 6) < 0) {
		pr_err("write buffer fail");
		return -EIO;
	}
	msleep(delay);

	/* reads low 8bits (crcL) */
	if (bootloader_rxtx(ts, &byteL, &byteH, 0) < 0) {
		pr_err("Failed to read low byte of crc response!");
		return -EIO;
	}
	rx_crc16 = (u16) byteL;

	/* reads high 8bits (crcH) */
	if (bootloader_rxtx(ts, &byteL, &byteH, 0) < 0) {
		pr_err("Failed to read high byte of crc response!");
		return -EIO;
	}
	rx_crc16 = (u16)(byteL << 8) | rx_crc16;

	if (bootloader_get_cmd_conf(ts, 5) < 0) {
		pr_err("CRC get failed!");
		return -EIO;
	}
	*crc16 = rx_crc16;

	return 0;
}

static int bootloader_set_byte_mode(struct data *ts)
{
	u8 buffer[2] = { 0x0A, 0x00 };

	if (bootloader_write_buffer(ts, buffer, 2) < 0) {
		pr_err("write buffer fail");
		return -EIO;
	}
	if (bootloader_get_cmd_conf(ts, 10) < 0) {
		pr_err("command confirm fail");
		return -EIO;
	}
	return 0;
}

static int bootloader_erase_flash(struct data *ts)
{
	u8 byteL = 0x02, byteH = 0x00;
	int i, verify = 0;

	if (bootloader_rxtx(ts, &byteL, &byteH, 1) < 0) {
		pr_err("bootloader RX-TX fail");
		return -EIO;
	}

	for (i = 0; i < 10; i++) {
		msleep(60); /* wait 60ms */

		if (bootloader_get_cmd_conf(ts, 0) < 0)
			continue;

		verify = 1;
		break;
	}

	if (verify != 1) {
		pr_err("Flash Erase failed");
		return -EIO;
	}

	return 0;
}

static int bootloader_write_flash(struct data *ts, const u8 *image, u16 length)
{
	u8 buffer[130];
	u8 length_L = length & 0xFF;
	u8 length_H = (length >> 8) & 0xFF;
	u8 command[] = { 0xF0, 0x00, length_H, length_L, 0x00 };
	u16 blocks_of_128bytes;
	int i, j;

	if (bootloader_write_buffer(ts, command, 5) < 0) {
		pr_err("write buffer fail");
		return -EIO;
	}

	blocks_of_128bytes = length >> 7;

	for (i = 0; i < blocks_of_128bytes; i++) {
		for (j = 0; j < 100; j++) {
			usleep_range(1500, 2000);
			if (bootloader_read_status_reg(ts, STATUS_READY_L,
			STATUS_READY_H)	== 0)
				break;
		}
		if (j == 100) {
			pr_err("Failed to read Status register!");
			return -EIO;
		}

		buffer[0] = ((i % 2) == 0) ? 0x00 : 0x40;
		buffer[1] = 0x00;
		memcpy(buffer + 2, image + i * 128, 128);

		if (i2c_tx_bytes(ts, buffer, 130) != 130) {
			pr_err("Failed to write data (%d)", i);
			return -EIO;
		}
		if (bootloader_rxtx_complete(ts) < 0) {
			pr_err("Transfer failure (%d)", i);
			return -EIO;
		}
	}

	usleep_range(10000, 11000);
	if (bootloader_get_cmd_conf(ts, 5) < 0) {
		pr_err("Flash programming failed");
		return -EIO;
	}
	return 0;
}

/****************************************
 *
 * Standard Driver Structures/Functions
 *
 ****************************************/
static const struct i2c_device_id id[] = { { MAX1187X_NAME, 0 }, { } };

MODULE_DEVICE_TABLE(i2c, id);

static struct of_device_id max1187x_dt_match[] = {
	{ .compatible = "maxim,max1187x_tsc" },	{ } };

static struct i2c_driver driver = {
		.probe = probe,
		.remove = remove,
		.id_table = id,
		.driver = {
			.name = MAX1187X_NAME,
			.owner	= THIS_MODULE,
			.of_match_table = max1187x_dt_match,
		},
};

static int __init max1187x_init(void)
{
	return i2c_add_driver(&driver);
}

static void __exit max1187x_exit(void)
{
	i2c_del_driver(&driver);
}

module_init(max1187x_init);
module_exit(max1187x_exit);

static int __init get_htc_event_support_flag(char *str)
{
	int ret = kstrtouint(str, 0, &support_htc_event_flag);
	pr_info("androidtouch.htc_event %d: %d from %s",
			ret, support_htc_event_flag, str);
	return ret;
} early_param("androidtouch.htc_event", get_htc_event_support_flag);

MODULE_AUTHOR("Maxim Integrated Products, Inc.");
MODULE_DESCRIPTION("MAX1187X Touchscreen Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("3.0.7.1");
