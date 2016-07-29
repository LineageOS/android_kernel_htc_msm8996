#ifndef _GPIO_KEYS_H
#define _GPIO_KEYS_H

#define KEY_LOGD(fmt, args...) printk(KERN_DEBUG "[KEY] "fmt, ##args)
#define KEY_LOGI(fmt, args...) printk(KERN_INFO "[KEY] "fmt, ##args)
#define KEY_LOGE(fmt, args...) printk(KERN_ERR "[KEY][ERR] "fmt, ##args)

#define GPIO_KEYS_DEV_NAME "gpio-keys"

struct device;

struct gpio_keys_button {
	unsigned int code;
	int gpio;
	int active_low;
	const char *desc;
	unsigned int type;
	int wakeup;
	int debounce_interval;
	bool can_disable;
	int value;
	unsigned int irq;
};

struct gpio_keys_platform_data {
	struct gpio_keys_button *buttons;
	int nbuttons;
	unsigned int poll_interval;
	unsigned int rep:1;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	const char *name;		
	bool use_syscore;
};

#endif
