/* drivers/misc/timed_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "timed_output.h"
#include "timed_gpio.h"


struct timed_gpio_data {
	struct timed_output_dev dev;
	struct hrtimer timer;
	spinlock_t lock;
	unsigned gpio;
	int max_timeout;
	u8 active_low;
};

static enum hrtimer_restart gpio_timer_func(struct hrtimer *timer)
{
	struct timed_gpio_data *data =
		container_of(timer, struct timed_gpio_data, timer);

	gpio_direction_output(data->gpio, data->active_low ? 1 : 0);
	VIB_INFO_LOG("off, active_low(%d)\n", data->active_low);

	return HRTIMER_NORESTART;
}

static int gpio_get_time(struct timed_output_dev *dev)
{
	struct timed_gpio_data *data;
	struct timeval t;

	data = container_of(dev, struct timed_gpio_data, dev);

	if (!hrtimer_active(&data->timer))
		return 0;

	t = ktime_to_timeval(hrtimer_get_remaining(&data->timer));

	return t.tv_sec * 1000 + t.tv_usec / 1000;
}

static void gpio_enable(struct timed_output_dev *dev, int value)
{
	struct timed_gpio_data	*data =
		container_of(dev, struct timed_gpio_data, dev);
	unsigned long	flags;

	if(value < 0) {
		VIB_INFO_LOG("%s unsupport value: %d\n", __func__, value);
		return;
	}
	VIB_INFO_LOG("%s= %d\n", __func__, value);

	spin_lock_irqsave(&data->lock, flags);

	
	hrtimer_cancel(&data->timer);

	gpio_direction_output(data->gpio, data->active_low ? !value : !!value);

	if(value > 0) {
		VIB_INFO_LOG("on, active_low(%d)\n", data->active_low);
		if (value > data->max_timeout)
			value = data->max_timeout;

		hrtimer_start(&data->timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	} else	
		VIB_INFO_LOG("off, active_low(%d)\n", data->active_low);

	spin_unlock_irqrestore(&data->lock, flags);
}

#ifdef CONFIG_OF
static int timed_gpio_dt_parse(struct device_node *node, struct timed_gpio_platform_data *pdata)
{
	int ret = 0;

	ret = of_property_read_u32(node, "vib,gpio_used", (u32 *)&pdata->num_gpios);
	if(ret) {
		VIB_ERR_LOG("%s, vib,gpio_used error\n", __func__);
		return ret;
	}

	ret = of_get_named_gpio(node, "vib,gpio", 0);
	if(!gpio_is_valid(ret)) {
		VIB_ERR_LOG("%s, vib,gpio error\n", __func__);
		return -EINVAL;
	}
	else {
		pdata->gpios = kzalloc(sizeof(struct timed_gpio), GFP_KERNEL);
		pdata->gpios->gpio = ret;
	}

	ret = of_property_read_string(node, "vib,gpio_name", &pdata->gpios->name);
	if(ret)
	{
		VIB_ERR_LOG("%s, vib,gpio_name error\n", __func__);
		return ret;
	}

	ret = of_property_read_u32(node, "vib,gpio_timeout", (u32 *)&pdata->gpios->max_timeout);
	if(ret) {
		VIB_ERR_LOG("%s, vib,gpio_timeout error\n", __func__);
		return ret;
	}
	VIB_INFO_LOG("%s, set max_timeout : %d\n", __func__, pdata->gpios->max_timeout);

	ret = of_property_read_u32(node, "vib,active_low", (u32 *)&pdata->gpios->active_low);
	if(ret)
	{
		VIB_ERR_LOG("%s, vib,active_low error\n", __func__);
		return ret;
	}

	return 0;
}
static struct of_device_id timed_gpio_of_match[] = {
	{ .compatible = "htc,timed_gpio_vibrator", },
	{ },
};
MODULE_DEVICE_TABLE(of, timed_gpio_of_match);
#else
static int timed_gpio_dt_parse(struct device_node *node, struct timed_gpio_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int timed_gpio_probe(struct platform_device *pdev)
{
	struct timed_gpio_platform_data *pdata = pdev->dev.platform_data;
	struct timed_gpio *cur_gpio;
	struct timed_gpio_data *gpio_data, *gpio_dat;
	int i, ret;

	VIB_INFO_LOG	("%s,++++++++++++++++++\n", __func__);
	if (!pdata) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		ret = timed_gpio_dt_parse(pdev->dev.of_node, pdata);
		if (IS_ERR(pdata))
		{
			VIB_INFO_LOG	("%s, dt parse error\n", __func__);
			return PTR_ERR(pdata);
		}
	}

	gpio_data = devm_kzalloc(&pdev->dev,
			sizeof(struct timed_gpio_data) * pdata->num_gpios,
			GFP_KERNEL);
	if (!gpio_data)
		return -ENOMEM;

	gpio_dat = devm_kzalloc(&pdev->dev,
			sizeof(struct timed_gpio_data) * pdata->num_gpios,
			GFP_KERNEL);
	if (!gpio_dat)
		return -ENOMEM;

	for (i = 0; i < pdata->num_gpios; i++) {
		cur_gpio = &pdata->gpios[i];
		gpio_dat = &gpio_data[i];

		hrtimer_init(&gpio_dat->timer, CLOCK_MONOTONIC,
				HRTIMER_MODE_REL);
		gpio_dat->timer.function = gpio_timer_func;
		spin_lock_init(&gpio_dat->lock);

		gpio_dat->dev.name = cur_gpio->name;
		gpio_dat->dev.get_time = gpio_get_time;
		gpio_dat->dev.enable = gpio_enable;
		ret = gpio_request(cur_gpio->gpio, cur_gpio->name);

		if (ret < 0)
			goto err_out;
		ret = timed_output_dev_register(&gpio_dat->dev);

		if (ret < 0) {
			gpio_free(cur_gpio->gpio);
			goto err_out;
		}

		gpio_dat->gpio = cur_gpio->gpio;
		gpio_dat->max_timeout = cur_gpio->max_timeout;
		gpio_dat->active_low = cur_gpio->active_low;
		gpio_direction_output(gpio_dat->gpio, gpio_dat->active_low);
	}
	platform_set_drvdata(pdev, gpio_data);

	VIB_INFO_LOG	("%s,----------------------\n", __func__);
	return 0;

err_out:
	while (--i >= 0) {
		timed_output_dev_unregister(&gpio_data[i].dev);
		gpio_free(gpio_data[i].gpio);
	}

	return ret;
}

static int timed_gpio_remove(struct platform_device *pdev)
{
	struct timed_gpio_platform_data *pdata = pdev->dev.platform_data;
	struct timed_gpio_data *gpio_data = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < pdata->num_gpios; i++) {
		timed_output_dev_unregister(&gpio_data[i].dev);
		gpio_free(gpio_data[i].gpio);
	}

	return 0;
}

static struct platform_driver timed_gpio_driver = {
	.probe		= timed_gpio_probe,
	.remove		= timed_gpio_remove,
	.driver		= {
		.name		= TIMED_GPIO_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = timed_gpio_of_match,
	},
};

module_platform_driver(timed_gpio_driver);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("timed gpio driver");
MODULE_LICENSE("GPL");
