/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <soc/qcom/scm.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/cdev.h>

#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/input.h>
#include <linux/qmi_encdec.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/msm_qmi_interface.h>
#include <linux/rtc.h>

#define FPC1020_RESET_LOW_US 1000
#define FPC1020_RESET_HIGH1_US 100
#define FPC1020_RESET_HIGH2_US 1250
#define PWR_ON_STEP_SLEEP 100
#define PWR_ON_STEP_RANGE1 100
#define PWR_ON_STEP_RANGE2 900
#define FPC_TTW_HOLD_TIME 1000
#define NUM_PARAMS_REG_ENABLE_SET 2

#define SNS_QFP_OPEN_REQ_V01 0x0020
#define SNS_QFP_OPEN_REQ_MSG_V01_MAX_MSG_LEN 15
#define SNS_QFP_OPEN_RESP_V01 0x0020
#define SNS_QFP_KEEP_ALIVE_REQ_V01 0x0022
#define SNS_QFP_KEEP_ALIVE_RESP_V01 0x0022
#define SNS_QFP_CLOSE_REQ_V01 0x0021
#define SNS_QFP_CLOSE_RESP_V01 0x0021
#define QMI_COMMON_TLV_TYPE 0
#define SNS_QFP_OPEN_RESP_MSG_V01_MAX_MSG_LEN 5
#define SNS_QFP_KEEP_ALIVE_REQ_MSG_V01_MAX_MSG_LEN 4
#define SNS_QFP_KEEP_ALIVE_RESP_MSG_V01_MAX_MSG_LEN 5
#define TIMEOUT_MS          (500)

#define TZ_BLSP_MODIFY_OWNERSHIP_ARGINFO    2
#define TZ_BLSP_MODIFY_OWNERSHIP_SVC_ID     4 
#define TZ_BLSP_MODIFY_OWNERSHIP_FUNC_ID    3
#define SET_PIPE_OWNERSHIP 1
#define CONFIG_PM 1

#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING
#include <linux/power/htc_battery.h>
#endif 

static const char * const pctl_names[] = {
    "fpc1020_irq_active",
	"fpc1020_reset_reset",
	"fpc1020_reset_active"
};

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

static const struct vreg_config const vreg_conf[] = {
	{ "vdd_ana", 1800000UL, 1800000UL, 6000, },
	{ "vcc_spi", 1800000UL, 1800000UL, 10, },
	{ "vdd_io", 1800000UL, 1800000UL, 6000, },
};

struct fpc1020_data {
	struct device *dev;
	struct spi_device *spi;
	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
	struct clk *iface_clk;
	struct clk *core_clk;
	struct regulator *vreg[ARRAY_SIZE(vreg_conf)];

	struct wake_lock ttw_wl;
	int irq_gpio;
	int cs0_gpio;
	int cs1_gpio;
	int rst_gpio;
	int qup_id;
	struct mutex lock;
	bool prepared;
	bool wakeup_enabled;
	bool clocks_enabled;
	bool clocks_suspended;
#ifdef CONFIG_FPC_HTC_RECORD_IRQ_COUNT
	int irq_count;
#endif 

#ifdef CONFIG_FPC_HTC_ADD_INPUT_DEVICE
	int event_type;
	int event_code;
	struct input_dev *idev;
#endif 
};


static char hal_footprint_str[128] = {0};
static ssize_t hal_footprint_set(struct device *dev,
   struct device_attribute *attr, const char *buf, size_t count)
{
    struct timespec ts;
    struct rtc_time tm;

    
    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec, &tm);

    scnprintf(hal_footprint_str, sizeof(hal_footprint_str), "fpc footprint:%s at (%02d-%02d %02d:%02d:%02d.%03lu)", buf,
        tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec/1000000);

    pr_info("%s\n", hal_footprint_str);

    return count;
}
static DEVICE_ATTR(hal_footprint, S_IRUGO, NULL, hal_footprint_set);

static int read_irq_gpio(struct fpc1020_data *fpc1020)
{
    int irq_gpio;

    irq_gpio = gpio_get_value(fpc1020->irq_gpio);

    return irq_gpio;
}

static ssize_t irq_gpio_show(struct device *dev,
               struct device_attribute *attr, char *buf)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
    int irq_gpio = read_irq_gpio(fpc1020);
    pr_info("fpc %s:%d\n", __func__, irq_gpio);

    return scnprintf(buf, PAGE_SIZE, "%i\n", irq_gpio);
}
static DEVICE_ATTR(irq_gpio, S_IRUGO, irq_gpio_show, NULL);



static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
	size_t i;
	int rc;
	struct device *dev = fpc1020->dev;
	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
					fpc1020->pinctrl_state[i]);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
				dev_dbg(dev, "Selected '%s'\n", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	dev_err(dev, "%s:'%s' not found\n", __func__, name);
exit:
	return rc;
}


static ssize_t pinctl_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc = select_pin_ctl(fpc1020, buf);
	return rc ? rc : count;
}
static DEVICE_ATTR(pinctl_set, S_IWUSR, NULL, pinctl_set);




static int hw_reset(struct  fpc1020_data *fpc1020)
{
	int irq_gpio;
	struct device *dev = fpc1020->dev;

	int rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_LOW_US, FPC1020_RESET_LOW_US + 100);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	dev_info(dev, "IRQ after reset %d\n", irq_gpio);
exit:
	return rc;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset")))
		rc = hw_reset(fpc1020);
	else
		return -EINVAL;
	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

static int device_prepare(struct  fpc1020_data *fpc1020, bool enable)
{
    int rc = 0;
    return rc;  
    mutex_lock(&fpc1020->lock);
    if (enable && !fpc1020->prepared)
    {
        fpc1020->prepared = true;
        
        (void)select_pin_ctl(fpc1020, "fpc1020_reset_reset");
        usleep_range(PWR_ON_STEP_SLEEP, PWR_ON_STEP_RANGE2);
    }
    else {
        rc = 0;
    }
    mutex_unlock(&fpc1020->lock);
    return rc;
}



static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable"))) {
		fpc1020->wakeup_enabled = true;
		smp_wmb();
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		fpc1020->wakeup_enabled = false;
		smp_wmb();
	} else
		return -EINVAL;

	return count;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

static ssize_t irq_get(struct device *device,
			     struct device_attribute *attribute,
			     char* buffer)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	int irq = gpio_get_value(fpc1020->irq_gpio);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}

static ssize_t irq_ack(struct device *device,
			     struct device_attribute *attribute,
			     const char *buffer, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	dev_dbg(fpc1020->dev, "%s\n", __func__);
	return count;
}

static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

#ifdef CONFIG_FPC_HTC_RECORD_IRQ_COUNT
static ssize_t irq_count_get(struct device *device,
			     struct device_attribute *attribute,
			     char* buffer)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", fpc1020->irq_count);
}
static DEVICE_ATTR(irq_count, S_IRUSR, irq_count_get, NULL);
#endif 



#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING
static ssize_t fp_disable_charging_set(struct device *device,
			struct device_attribute *attribute, const char *buf, size_t count)
{
	if (!strncmp(buf, "enable", strlen("enable"))) {
		pr_info("[fp] %s:%s\n", __func__, buf);
		htc_battery_charger_switch_internal(ENABLE_PWRSRC_FINGERPRINT);
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		pr_info("[fp] %s:%s\n", __func__, buf);
		htc_battery_charger_switch_internal(DISABLE_PWRSRC_FINGERPRINT);
	} else {
		pr_err("[fp] Wrong Parameter!!:%s\n", buf);
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(fp_disable_charge, S_IWUSR, NULL, fp_disable_charging_set);
#endif 

static struct attribute *attributes[] = {
	&dev_attr_pinctl_set.attr,
	&dev_attr_irq.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
#ifdef CONFIG_FPC_HTC_RECORD_IRQ_COUNT
	&dev_attr_irq_count.attr,
#endif 
	&dev_attr_hal_footprint.attr, 
	&dev_attr_irq_gpio.attr, 
#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING
	&dev_attr_fp_disable_charge.attr,
#endif 

	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;

	dev_dbg(fpc1020->dev, "%s\n", __func__);

	smp_rmb();

	if (fpc1020->wakeup_enabled) {
		wake_lock_timeout(&fpc1020->ttw_wl,
					msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	}

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

#ifdef CONFIG_FPC_HTC_RECORD_IRQ_COUNT
	fpc1020->irq_count++;
#endif 

	return IRQ_HANDLED;
}

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
		const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);
	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_dbg(dev, "%s %d\n", label, *gpio);
	return 0;
}

static int fpc1020_probe(struct platform_device *spi)
{
#ifdef CONFIG_FPC_HTC_ADD_INPUT_DEVICE
		struct input_dev *input_dev;
#endif 
	struct device *dev = &spi->dev;
	int rc = 0;
	size_t i;
	int irqf;
	struct device_node *np = dev->of_node;
	
	struct fpc1020_data *fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020),
			GFP_KERNEL);
    printk("Check fpc1020_probe");

    if (!fpc1020) {
        dev_err(dev,"failed to allocate memory for struct fpc1020_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->dev = dev;
	dev_set_drvdata(dev, fpc1020);
	

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}
	rc = fpc1020_request_named_gpio(fpc1020, "fpc_gpio_irq",
			&fpc1020->irq_gpio);
	if (rc)
		goto exit;
	rc = fpc1020_request_named_gpio(fpc1020, "fpc_gpio_rst",
			&fpc1020->rst_gpio);
	if (rc)
		goto exit;
    fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
		if (PTR_ERR(fpc1020->fingerprint_pinctrl) == -EPROBE_DEFER) {
			dev_info(dev, "pinctrl not ready\n");
			rc = -EPROBE_DEFER;
			goto exit;
		}
		dev_err(dev, "Target does not use pinctrl\n");
		fpc1020->fingerprint_pinctrl = NULL;
		rc = -EINVAL;
		goto exit;
	}
	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			dev_err(dev, "cannot find '%s'\n", n);
			rc = -EINVAL;
			goto exit;
		}
		dev_info(dev, "found pin control %s\n", n);
		fpc1020->pinctrl_state[i] = state;
        printk("i=%d\n",(int) i);
	}

    rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;
	rc = select_pin_ctl(fpc1020, "fpc1020_irq_active");
	if (rc)
		goto exit;

#ifdef CONFIG_FPC_HTC_ADD_INPUT_DEVICE
	input_dev = devm_input_allocate_device(dev);
	if (!input_dev) {
		dev_err(dev, "%s failed to allocate input device\n", __func__);
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->idev = input_dev;

	input_dev->name = "fpc1020";

	fpc1020->event_type = EV_MSC;
	fpc1020->event_code = MSC_SCAN;

	input_set_capability(fpc1020->idev, EV_MSC, MSC_SCAN);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_WAKEUP, input_dev->keybit);
	rc = input_register_device(input_dev);
	if (rc) {
		dev_err(dev, "%s failed to register input device\n", __func__);
		goto exit;
	}
#endif 

	fpc1020->wakeup_enabled = false;
	fpc1020->clocks_enabled = false;
	fpc1020->clocks_suspended = false;
	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}
	mutex_init(&fpc1020->lock);
	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto exit;
	}
	dev_dbg(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

	
	enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));

	wake_lock_init(&fpc1020->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

	if (of_property_read_bool(dev->of_node, "fpc,enable-on-boot")) {
		dev_info(dev, "Enabling hardware\n");
		(void)device_prepare(fpc1020, true);
	}

#ifdef CONFIG_FPC_HTC_RECORD_IRQ_COUNT
	fpc1020->irq_count = 0;
#endif 

	dev_info(dev, "%s: ok\n", __func__);
exit:
	return rc;
}

static int fpc1020_remove(struct platform_device *spi)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(&spi->dev);

	sysfs_remove_group(&spi->dev.kobj, &attribute_group);
	mutex_destroy(&fpc1020->lock);
	wake_lock_destroy(&fpc1020->ttw_wl);


    dev_info(&spi->dev, "%s\n", __func__);
	return 0;
}

static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);
#ifdef CONFIG_PM
static int fpc1020_suspend(struct device *dev)
{
    int ret = 0;
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
    int irq_gpio_level;

    irq_gpio_level = read_irq_gpio(fpc1020);
    if (irq_gpio_level != 0)
        dev_info(dev, "%s irq_gpio_level:%d\n", __func__, irq_gpio_level);

    return ret;
}

static int fpc1020_resume(struct device *dev)
{
    int ret = 0;
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
    int irq_gpio_level;

    irq_gpio_level = read_irq_gpio(fpc1020);
    if (irq_gpio_level != 0)
        dev_info(dev, "%s irq_gpio_level:%d\n", __func__, irq_gpio_level);

    return ret;
}

static int fpc1020_suspend_noirq(struct device *dev)
{
    int ret = 0;
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
    int irq_gpio_level;

    irq_gpio_level = read_irq_gpio(fpc1020);
    if (irq_gpio_level != 0)
        dev_info(dev, "%s irq_gpio_level:%d\n", __func__, irq_gpio_level);

    return ret;
}

static int fpc1020_resume_noirq(struct device *dev)
{
    int ret = 0;
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
    int irq_gpio_level;

    irq_gpio_level = read_irq_gpio(fpc1020);
    if (irq_gpio_level != 0)
        dev_info(dev, "%s irq_gpio_level:%d\n", __func__, irq_gpio_level);

    return ret;
}

static const struct dev_pm_ops fpc1020_power_dev_pm_ops = {
    .suspend_noirq = fpc1020_suspend_noirq,
    .resume_noirq = fpc1020_resume_noirq,
    .suspend = fpc1020_suspend,
    .resume = fpc1020_resume,
};
#endif 



static struct platform_driver fpc1020_driver = {
    .driver = {
        .name   = "fpc1020",
        .owner  = THIS_MODULE,
        .of_match_table = fpc1020_of_match,
#ifdef CONFIG_PM
          .pm = &fpc1020_power_dev_pm_ops,
#endif
    },
    .probe  = fpc1020_probe,
    .remove = fpc1020_remove,
};

static int __init fpc1020_init(void)
{
    printk("%s\n", __func__);
    platform_driver_register(&fpc1020_driver);
    return 0;
}

static void __exit fpc1020_exit(void)
{
    printk("%s\n", __func__);
    platform_driver_unregister(&fpc1020_driver);
}


module_init(fpc1020_init);
module_exit(fpc1020_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");

