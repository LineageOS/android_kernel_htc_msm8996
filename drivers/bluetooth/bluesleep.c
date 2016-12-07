/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.


 * Copyright (C) 2006-2007 - Motorola
 * Copyright (c) 2008-2010, The Linux Foundation. All rights reserved.
 * Copyright (c) 2013, LGE Inc.
 * Copyright (c) 2014, HTC Corporation.

 * Date         Author           Comment
 * -----------  --------------   --------------------------------
 * 2006-Apr-28	Motorola	 The kernel module for running the Bluetooth(R)
 *                               Sleep-Mode Protocol from the Host side
 *  2006-Sep-08  Motorola        Added workqueue for handling sleep work.
 *  2007-Jan-24  Motorola        Added mbm_handle_ioi() call to ISR.
 *  2009-Aug-10  Motorola        Changed "add_timer" to "mod_timer" to solve
 *                               race when flurry of queued work comes in.
 */

#ifndef pr_fmt
#define pr_fmt(fmt)	"Bluetooth: %s: " fmt, __func__
#endif

#include <linux/module.h>	
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/param.h>
#include <linux/bitops.h>
#include <linux/termios.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/serial_core.h>
#include <linux/platform_data/msm_serial_hs.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h> 
#include "hci_uart.h"

#define BT_SLEEP_DBG
#ifndef BT_SLEEP_DBG
#define BT_DBG(fmt, arg...)
#endif


#define VERSION		"1.1"
#define PROC_DIR	"bluetooth/sleep"

#define POLARITY_LOW 0
#define POLARITY_HIGH 1

#define BT_UART_PORT_ID 0

#define BT_ENABLE_IRQ_WAKE 1

#define BT_BLUEDROID_SUPPORT 1

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 1,
	DEBUG_BTWAKE = 1U << 2,
	DEBUG_VERBOSE = 1U << 3,
};

static int debug_mask = DEBUG_USER_STATE | DEBUG_SUSPEND | DEBUG_BTWAKE | DEBUG_VERBOSE; 
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

struct bluesleep_info {
	unsigned host_wake;
	unsigned ext_wake;
	unsigned host_wake_irq;
	struct uart_port *uport;
	struct wake_lock wake_lock;
	int irq_polarity;
	int has_ext_wake;
	struct mutex state_mutex; 
};

static void bluesleep_sleep_work(struct work_struct *work);

DECLARE_DELAYED_WORK(sleep_workqueue, bluesleep_sleep_work);

#define bluesleep_rx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_tx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_rx_idle()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_tx_idle()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_hsuart_clk_check()     schedule_delayed_work(&sleep_workqueue, msecs_to_jiffies(10))

#define TX_TIMER_INTERVAL  1 

#define BT_PROTO		0x01    
#define BT_TXDATA		0x02    
#define BT_ASLEEP		0x04    
#define BT_EXT_WAKE		0x08    
#define BT_SUSPEND		0x10
#define BT_ASLEEPING	0x20    

#if BT_BLUEDROID_SUPPORT
static bool has_lpm_enabled = false;
static int bluesleep_reset_lpm_internal(void);
#else
static struct hci_dev *bluesleep_hdev;
#endif

static bool bt_pwr_enabled = false; 

static struct platform_device *bluesleep_uart_dev;
static struct bluesleep_info *bsi;

static atomic_t open_count = ATOMIC_INIT(1);

#if !BT_BLUEDROID_SUPPORT
static int bluesleep_hci_event(struct notifier_block *this,
			unsigned long event, void *data);
#endif
static int bluesleep_start(void);
static void bluesleep_stop(void);


static unsigned long flags;

static struct tasklet_struct hostwake_task;

static void bluesleep_tx_timer_expire(unsigned long data);
static DEFINE_TIMER(tx_timer, bluesleep_tx_timer_expire, 0, 0);

static spinlock_t rw_lock;

#if !BT_BLUEDROID_SUPPORT
struct notifier_block hci_event_nblock = {
	.notifier_call = bluesleep_hci_event,
};
#endif
enum msm_hs_states {
	MSM_HS_PORT_OFF,       
	MSM_HS_PORT_ON,         
};


struct proc_dir_entry *bluetooth_dir, *sleep_dir;

extern unsigned int msm_hs_tx_empty_brcmbt(struct uart_port *uport);
extern void msm_hs_request_clock_off_brcmbt(struct uart_port *uport);
extern void msm_hs_request_clock_on_brcmbt(struct uart_port *uport);
extern struct uart_port *msm_hs_get_uart_port_brcmbt(int port_index);
extern void msm_hs_set_mctrl_brcmbt(struct uart_port *uport,
				    unsigned int mctrl);

extern int msm_hs_uart_get_clk_state(void);



static void hsuart_power(int on)
{
	
	if (!test_bit(BT_PROTO, &flags) && !has_lpm_enabled) {
		pr_err("not bluesleep (0x%lx)\n", flags);
		return;
	}

	if (test_bit(BT_SUSPEND, &flags)) {
		pr_info("suspend already\n");
		return;
	}

	if (!bt_pwr_enabled)
		pr_warn("control UART under bt off\n");

	if (bsi->uport == NULL) {
		pr_err("NULL UART\n");
		return;
	}

	pr_info("(%d)+\n", on);
	if (on) {
		msm_hs_request_clock_on_brcmbt(bsi->uport);
		msm_hs_set_mctrl_brcmbt(bsi->uport, TIOCM_RTS);
	} else {
		msm_hs_set_mctrl_brcmbt(bsi->uport, 0);
		msm_hs_request_clock_off_brcmbt(bsi->uport);
	}
	pr_devel("(%d)-\n", on);
}
void bluesleep_set_bt_pwr_state(int on)
{
	if (on) {
		bt_pwr_enabled = true;
	} else {
		bt_pwr_enabled = false;
#if BT_BLUEDROID_SUPPORT
		bluesleep_reset_lpm_internal();
#endif
	}
}
EXPORT_SYMBOL(bluesleep_set_bt_pwr_state);

int bluesleep_can_sleep(void)
{
	
	return ((gpio_get_value(bsi->host_wake) != bsi->irq_polarity) &&
		(test_bit(BT_EXT_WAKE, &flags)) &&  
		(!test_bit(BT_TXDATA, &flags)) &&   
		(bsi->uport != NULL));
}

void bluesleep_sleep_wakeup(void)
{
	static int clk_retry = 0;

	mutex_lock(&bsi->state_mutex);
	if (test_bit(BT_ASLEEPING, &flags) || test_bit(BT_ASLEEP, &flags)) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("waking up...\n");

		wake_lock(&bsi->wake_lock);

		
		if ((test_bit(BT_ASLEEPING, &flags)) && clk_retry < 10) {

			pr_info("not access UART when REQUEST_OFF retry:%d\n", clk_retry);
			clk_retry++;

			
			bluesleep_hsuart_clk_check();
		} else {
			if (clk_retry != 0)
				pr_info("clk state changed retry:%d\n", clk_retry);
			clk_retry = 0;
			
			mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL * HZ));
			if (debug_mask & DEBUG_BTWAKE)
				pr_devel("BT WAKE: set to wake\n");
			if (bsi->has_ext_wake == 1)
				gpio_set_value(bsi->ext_wake, 0);
			clear_bit(BT_EXT_WAKE, &flags);
			clear_bit(BT_ASLEEP, &flags);
			
			hsuart_power(1);
		}
	} else if (test_bit(BT_EXT_WAKE, &flags)) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("waking up without power up UART\n");
		wake_lock(&bsi->wake_lock);
		
		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL * HZ));
		if (debug_mask & DEBUG_BTWAKE)
			pr_devel("BT WAKE: set to wake\n");
		if (bsi->has_ext_wake == 1)
			gpio_set_value(bsi->ext_wake, 0);
		clear_bit(BT_EXT_WAKE, &flags);
	}
	mutex_unlock(&bsi->state_mutex);
}

static void bluesleep_sleep_work(struct work_struct *work)
{
	if (bluesleep_can_sleep()) {
		
		if (test_bit(BT_ASLEEP, &flags)) {
			pr_info("already asleep (0x%lx)\n", flags);
			return;
		}

		
		
		if (msm_hs_uart_get_clk_state() == MSM_HS_PORT_OFF) {
			pr_info("UART port already off and stop lpm\n");
#if BT_BLUEDROID_SUPPORT
			bluesleep_reset_lpm_internal();
#endif
			return;
		}
		

		mutex_lock(&bsi->state_mutex);
		set_bit(BT_ASLEEPING, &flags);
		if (msm_hs_tx_empty_brcmbt(bsi->uport)) {
			if (debug_mask & DEBUG_SUSPEND)
				pr_info("going to sleep\n");
			set_bit(BT_ASLEEP, &flags);
			
			hsuart_power(0);
			wake_lock_timeout(&bsi->wake_lock, HZ / 2);
			clear_bit(BT_ASLEEPING, &flags);
		} else {
			clear_bit(BT_ASLEEPING, &flags);
			if (debug_mask & DEBUG_SUSPEND)
				pr_info("UART tx empty false\n");
			mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL * HZ));
		}
		mutex_unlock(&bsi->state_mutex);
	} else if (test_bit(BT_EXT_WAKE, &flags)
			&& !test_bit(BT_ASLEEP, &flags)) {
		
		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL * HZ));
		if (debug_mask & DEBUG_BTWAKE)
			pr_devel("BT WAKE: keep waking\n");
		if (bsi->has_ext_wake == 1)
			gpio_set_value(bsi->ext_wake, 0);
		clear_bit(BT_EXT_WAKE, &flags);
	} else {
		
		
		bluesleep_sleep_wakeup();
	}
}

static void bluesleep_hostwake_task(unsigned long data)
{
	
	

	spin_lock(&rw_lock);
	if ((gpio_get_value(bsi->host_wake) == bsi->irq_polarity))
		bluesleep_rx_busy();
	else
		bluesleep_rx_idle();

	spin_unlock(&rw_lock);
}

static void bluesleep_outgoing_data(void)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);

	
	set_bit(BT_TXDATA, &flags);

	
	if (test_bit(BT_EXT_WAKE, &flags)) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("wake up the sleeping Tx\n");
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		bluesleep_sleep_wakeup();
	} else
		spin_unlock_irqrestore(&rw_lock, irq_flags);
}

#if BT_BLUEDROID_SUPPORT
static int bluesleep_lpm_enable (int en)
{
	mutex_lock(&bsi->state_mutex);
	if (!en) {
		
		bluesleep_stop();
		has_lpm_enabled = false;
		bsi->uport = NULL;
	} else {
		
		if (!has_lpm_enabled) {
			has_lpm_enabled = true;
			bsi->uport = msm_hs_get_uart_port_brcmbt(BT_UART_PORT_ID);
			
			bluesleep_start();
		}
	}
	mutex_unlock(&bsi->state_mutex);
	return 0;
}

static int bluesleep_reset_lpm_internal()
{
	if (has_lpm_enabled) {
		pr_warn("no bluesleep_write_proc_lpm\n");
		bluesleep_lpm_enable(0);
	}
	return 0;
}

static ssize_t bluesleep_read_proc_lpm(struct file *file, char __user *buf,
			       size_t size, loff_t *ppos)
{
	pr_info("not supported\n");
	return 0;
}

static ssize_t bluesleep_write_proc_lpm(struct file *file, const char __user *input,
				size_t size, loff_t *ppos)
{
	char b = '1';

	if (size < 1) {
		pr_info("wrong size: %ld\n", size);
		return -EINVAL;
	}

	if (copy_from_user(&b, input, 1)) {
		pr_info("copy_from_user failed\n");
		return -EFAULT;
	}

	pr_warn("%c\n", b);

	if (b == '0') {
		bluesleep_lpm_enable(0);
	} else {
		bluesleep_lpm_enable(1);
	}

	return size;
}

static ssize_t bluesleep_read_proc_btwrite(struct file *file, char __user *buf,
			       size_t size, loff_t *ppos)
{
	pr_info("not supported\n");
	return 0;
}

static ssize_t bluesleep_write_proc_btwrite(struct file *file, const char __user *input,
				size_t size, loff_t *ppos)
{
	char b = '0';
	int i = 0;

	if (size < 1) {
		pr_info("wrong size: %ld\n", size);
		return -EINVAL;
	}

	if (copy_from_user(&b, input, 1)) {
		pr_info("copy_from_user failed\n");
		return -EFAULT;
	}

	pr_info("%c\n", b);

	
	if (b != '0') {
		bluesleep_outgoing_data();

		
		while (i <= 20) {
			if (!test_bit(BT_ASLEEPING, &flags)) {
				if (i != 0)
					pr_info("clk ready count:%d\n", i);
				break;
			}
			if ((i % 5) == 0)
				pr_info("clk not yet on count:%d\n", i);
			i++;
			msleep(5);
		}
	}

	return size;
}
#else
static int bluesleep_hci_event(struct notifier_block *this,
				unsigned long event, void *data)
{
	struct hci_dev *hdev = (struct hci_dev *) data;
	struct hci_uart *hu;
	struct uart_state *state;

	if (!hdev)
		return NOTIFY_DONE;

	switch (event) {
	case HCI_DEV_REG:
		if (!bluesleep_hdev) {
			bluesleep_hdev = hdev;
			hu  = (struct hci_uart *) hdev->driver_data;
			state = (struct uart_state *) hu->tty->driver_data;
			bsi->uport = state->uart_port;
			
			bluesleep_start();
		}
		break;
	case HCI_DEV_UNREG:
		bluesleep_stop();
		bluesleep_hdev = NULL;
		bsi->uport = NULL;
		
		break;
	case HCI_DEV_WRITE:
		bluesleep_outgoing_data();
		break;
	}

	return NOTIFY_DONE;
}
#endif

static void bluesleep_tx_timer_expire(unsigned long data)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);
	if (debug_mask & DEBUG_VERBOSE)
		pr_devel("Tx timer expired\n");

	
	if (!test_bit(BT_TXDATA, &flags)) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("Tx has been idle\n");
		if (debug_mask & DEBUG_BTWAKE)
			pr_devel("BT WAKE: set to sleep\n");
		if (bsi->has_ext_wake == 1)
			gpio_set_value(bsi->ext_wake, 1);
		set_bit(BT_EXT_WAKE, &flags);
		bluesleep_tx_idle();
	} else {
		if (debug_mask & DEBUG_SUSPEND)
			pr_devel("Tx data during last period\n");
		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL*HZ));
	}

	
	clear_bit(BT_TXDATA, &flags);

	spin_unlock_irqrestore(&rw_lock, irq_flags);
}

static irqreturn_t bluesleep_hostwake_isr(int irq, void *dev_id)
{
       if (test_bit(BT_ASLEEPING, &flags) || test_bit(BT_ASLEEP, &flags)) {
          pr_info("hostwake_isr\n");
       }
	
	tasklet_schedule(&hostwake_task);
	return IRQ_HANDLED;
}

static int bluesleep_start(void)
{
	int retval;
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);

	if (test_bit(BT_PROTO, &flags)) {
		pr_err("already (0x%lx)\n", flags);
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return 0;
	}

	spin_unlock_irqrestore(&rw_lock, irq_flags);

	if (!atomic_dec_and_test(&open_count)) {
		atomic_inc(&open_count);
		return -EBUSY;
	}

	
	mod_timer(&tx_timer, jiffies + (5*TX_TIMER_INTERVAL*HZ));
	
	
	

	
	if (debug_mask & DEBUG_BTWAKE)
		pr_devel("BT WAKE: set to wake\n");
	if (bsi->has_ext_wake == 1)
		gpio_set_value(bsi->ext_wake, 0);
	clear_bit(BT_EXT_WAKE, &flags);
#if BT_ENABLE_IRQ_WAKE
	retval = enable_irq_wake(bsi->host_wake_irq);
	if (retval < 0) {
		pr_err("Couldn't enable BT_HOST_WAKE as wakeup interrupt\n");
		goto fail;
	}
#endif
	set_bit(BT_PROTO, &flags);
	wake_lock(&bsi->wake_lock);
	return 0;
fail:
	del_timer(&tx_timer);
	atomic_inc(&open_count);

	return retval;
}

static void bluesleep_stop(void)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);

	if (!test_bit(BT_PROTO, &flags)) {
		pr_err("already (0x%lx)\n", flags);
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return;
	}

	
	if (debug_mask & DEBUG_BTWAKE)
		pr_devel("BT WAKE: set to wake\n");
	if (bsi->has_ext_wake == 1)
		gpio_set_value(bsi->ext_wake, 0);
	clear_bit(BT_EXT_WAKE, &flags);
	del_timer(&tx_timer);
	clear_bit(BT_PROTO, &flags);

	spin_unlock_irqrestore(&rw_lock, irq_flags);

	if (test_bit(BT_ASLEEP, &flags)) {
		clear_bit(BT_ASLEEP, &flags);
		hsuart_power(1);
	}

	atomic_inc(&open_count);

#if BT_ENABLE_IRQ_WAKE
	if (disable_irq_wake(bsi->host_wake_irq))
		pr_err("Couldn't disable hostwake IRQ wakeup mode\n");
#endif
	wake_lock_timeout(&bsi->wake_lock, HZ / 2);
}
static ssize_t bluepower_read_proc_btwake(struct file *file, char __user *buf,
			       size_t size, loff_t *ppos)
{
	pr_info("not supported\n");
	return 0;
}

static ssize_t bluepower_write_proc_btwake(struct file *file, const char __user *input,
				size_t size, loff_t *ppos)
{
	pr_info("not supported\n");
	return size;
}

static ssize_t bluepower_read_proc_hostwake(struct file *file, char __user *buf,
			       size_t size, loff_t *ppos)
{
	pr_info("not supported\n");
	return 0;
}

static ssize_t bluesleep_read_proc_asleep(struct file *file, char __user *buf,
			       size_t size, loff_t *ppos)
{
	pr_info("not supported\n");
	return 0;
}

static ssize_t bluesleep_read_proc_proto(struct file *file, char __user *buf,
			       size_t size, loff_t *ppos)
{
	pr_info("not supported\n");
	return 0;
}

static ssize_t bluesleep_write_proc_proto(struct file *file, const char __user *input,
				size_t size, loff_t *ppos)
{
	pr_info("not supported\n");


	
	return size;
}

void bluesleep_setup_uart_port(struct platform_device *uart_dev)
{
	bluesleep_uart_dev = uart_dev;
}

static int bluesleep_populate_dt_pinfo(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int tmp;

	tmp = of_get_named_gpio(np, "brcm_bt_host_wake", 0);
	if (tmp < 0) {
		pr_err("couldn't find host_wake gpio\n");
		return -ENODEV;
	}
	bsi->host_wake = tmp;

	tmp = of_get_named_gpio(np, "brcm_bt_wake_dev", 0);
	if (tmp < 0)
		bsi->has_ext_wake = 0;
	else
		bsi->has_ext_wake = 1;

	if (bsi->has_ext_wake)
		bsi->ext_wake = tmp;

	pr_info("bt_host_wake %d, bt_ext_wake %d\n",
			bsi->host_wake,
			bsi->ext_wake);
	return 0;
}

static int bluesleep_populate_pinfo(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
				"gpio_host_wake");
	if (!res) {
		pr_err("couldn't find host_wake gpio\n");
		return -ENODEV;
	}
	bsi->host_wake = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
				"gpio_ext_wake");
	if (!res)
		bsi->has_ext_wake = 0;
	else
		bsi->has_ext_wake = 1;

	if (bsi->has_ext_wake)
		bsi->ext_wake = res->start;

	return 0;
}

static int bluesleep_probe(struct platform_device *pdev)
{
	int ret;

	bsi = kzalloc(sizeof(struct bluesleep_info), GFP_KERNEL);
	if (!bsi)
		return -ENOMEM;

	if (pdev->dev.of_node) {
		ret = bluesleep_populate_dt_pinfo(pdev);
		if (ret < 0) {
			pr_err("couldn't populate info from dt\n");
			return ret;
		}
	} else {
		ret = bluesleep_populate_pinfo(pdev);
		if (ret < 0) {
			pr_err("couldn't populate info\n");
			return ret;
		}
	}

	
	ret = gpio_request_one(bsi->host_wake, GPIOF_IN, "bt_host_wake");
	if (ret < 0) {
		pr_err("failed to configure input"
				" direction for GPIO %d, error %d\n",
				bsi->host_wake, ret);
		goto free_bsi;
	}

	if (debug_mask & DEBUG_BTWAKE)
		pr_devel("BT WAKE: set to wake\n");
	if (bsi->has_ext_wake) {
		
		ret = gpio_request_one(bsi->ext_wake,
				GPIOF_OUT_INIT_LOW, "bt_ext_wake");
		if (ret < 0) {
			pr_err("failed to configure output"
				" direction for GPIO %d, error %d\n",
				  bsi->ext_wake, ret);
			goto free_bt_host_wake;
		}
	}
	clear_bit(BT_EXT_WAKE, &flags);
#if 0
	res = platform_get_irq_byname(pdev, "host_wake");
	if (!res) {
		pr_err("couldn't find host_wake irq\n");
		ret = -ENODEV;
		goto free_bt_host_wake;
	}
	bsi->host_wake_irq = res;
#else
        bsi->host_wake_irq = gpio_to_irq(bsi->host_wake);
#endif
	if (bsi->host_wake_irq < 0) {
		pr_err("couldn't find host_wake irq res\n");
		ret = -ENODEV;
		goto free_bt_ext_wake;
	}

	bsi->irq_polarity = POLARITY_LOW;

	wake_lock_init(&bsi->wake_lock, WAKE_LOCK_SUSPEND, "bluesleep");
	clear_bit(BT_SUSPEND, &flags);

	pr_info("host_wake_irq %d, polarity %d\n",
			bsi->host_wake_irq,
			bsi->irq_polarity);

	
	spin_lock_init(&rw_lock);

	mutex_init(&bsi->state_mutex);

	
	tasklet_init(&hostwake_task, bluesleep_hostwake_task, 0);

	
	ret = request_irq(bsi->host_wake_irq, bluesleep_hostwake_isr,
			IRQF_DISABLED | IRQF_TRIGGER_FALLING,
			"bluetooth hostwake", NULL);
	if (ret  < 0) {
		pr_err("Couldn't acquire BT_HOST_WAKE IRQ\n");
		goto free_bt_ext_wake;
	}

	return 0;

free_bt_ext_wake:
	gpio_free(bsi->ext_wake);
	mutex_destroy(&bsi->state_mutex);
free_bt_host_wake:
	gpio_free(bsi->host_wake);
free_bsi:
	kfree(bsi);
	return ret;
}

static int bluesleep_remove(struct platform_device *pdev)
{
	free_irq(bsi->host_wake_irq, NULL);
	gpio_free(bsi->host_wake);
	gpio_free(bsi->ext_wake);
	wake_lock_destroy(&bsi->wake_lock);
	mutex_destroy(&bsi->state_mutex);
	kfree(bsi);
	return 0;
}


static int bluesleep_resume(struct platform_device *pdev)
{
	if (test_bit(BT_SUSPEND, &flags)) {
		if ((bsi->uport != NULL) &&
			(gpio_get_value(bsi->host_wake) == bsi->irq_polarity)) {
			if (debug_mask & DEBUG_SUSPEND)
				pr_info("from BT event\n");

			if (!bt_pwr_enabled)
				pr_info("control UART under bt off\n");
			msm_hs_request_clock_on_brcmbt(bsi->uport);
			msm_hs_set_mctrl_brcmbt(bsi->uport, TIOCM_RTS);
		}
		clear_bit(BT_SUSPEND, &flags);
	}
	return 0;
}

static int bluesleep_suspend(struct platform_device *pdev, pm_message_t state)
{
	set_bit(BT_SUSPEND, &flags);
	return 0;
}

static struct of_device_id bluesleep_match_table[] = {
	{ .compatible = "htc,bluesleep_bcm" },
	{}
};

static struct platform_driver bluesleep_driver = {
	.probe = bluesleep_probe,
	.remove = bluesleep_remove,
	.suspend = bluesleep_suspend,
	.resume = bluesleep_resume,
	.driver = {
		.name = "bluesleep",
		.owner = THIS_MODULE,
		.of_match_table = bluesleep_match_table,
	},
};


static const struct file_operations bluesleep_proc_fops_btwake = {
	.read = bluepower_read_proc_btwake,
	.write = bluepower_write_proc_btwake,
};

static const struct file_operations bluesleep_proc_fops_hostwake = {
	.read = bluepower_read_proc_hostwake,
};

static const struct file_operations bluesleep_proc_fops_proto = {
	.read = bluesleep_read_proc_proto,
	.write = bluesleep_write_proc_proto,
};

static const struct file_operations bluesleep_proc_fops_asleep = {
	.read = bluesleep_read_proc_asleep,
};

#if BT_BLUEDROID_SUPPORT
static const struct file_operations bluesleep_proc_fops_lpm = {
	.read = bluesleep_read_proc_lpm,
	.write = bluesleep_write_proc_lpm,
};

static const struct file_operations bluesleep_proc_fops_btwrite = {
	.read = bluesleep_read_proc_btwrite,
	.write = bluesleep_write_proc_btwrite,
};
#endif

static int __init bluesleep_init(void)
{
	int retval;
	struct proc_dir_entry *ent;

	pr_info("BlueSleep Mode Driver Ver %s\n", VERSION);

	retval = platform_driver_register(&bluesleep_driver);
	if (retval)
		return retval;

	if (bsi == NULL)
		return 0;

#if !BT_BLUEDROID_SUPPORT
	bluesleep_hdev = NULL;
#endif

	bluetooth_dir = proc_mkdir("bluetooth", NULL);
	if (bluetooth_dir == NULL) {
		pr_err("Unable to create /proc/bluetooth directory\n");
		return -ENOMEM;
	}

	sleep_dir = proc_mkdir("sleep", bluetooth_dir);
	if (sleep_dir == NULL) {
		pr_err("Unable to create /proc/%s directory\n", PROC_DIR);
		return -ENOMEM;
	}

#if 0 
	
	ent = proc_create("btwake", S_IRUGO | S_IWUSR | S_IWGRP,
			sleep_dir, &bluesleep_proc_fops_btwake);
	if (ent == NULL) {
		pr_err("Unable to create /proc/%s/btwake entry\n", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	
	if (proc_create("hostwake", S_IRUGO, sleep_dir,
				&bluesleep_proc_fops_hostwake) == NULL) {
		pr_err("Unable to create /proc/%s/hostwake entry\n", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	
	ent = proc_create("proto", S_IRUGO | S_IWUSR | S_IWGRP,
			sleep_dir, &bluesleep_proc_fops_proto);
	if (ent == NULL) {
		pr_err("Unable to create /proc/%s/proto entry\n", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	
	if (proc_create("asleep", S_IRUGO,
			sleep_dir, &bluesleep_proc_fops_asleep) == NULL) {
		pr_err("Unable to create /proc/%s/asleep entry\n", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}
#endif

#if BT_BLUEDROID_SUPPORT
	
	ent = proc_create("lpm", S_IRUSR | S_IWUSR | S_IWGRP | S_IWOTH,
			sleep_dir, &bluesleep_proc_fops_lpm);
	if (ent == NULL) {
		pr_err("Unable to create /proc/%s/lpm entry\n", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	
	ent = proc_create("btwrite", S_IRUSR | S_IWUSR | S_IWGRP | S_IWOTH,
			sleep_dir, &bluesleep_proc_fops_btwrite);
	if (ent == NULL) {
		pr_err("Unable to create /proc/%s/btwrite entry\n", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}
#endif

	flags = 0; 

	
	init_timer(&tx_timer);
	tx_timer.function = bluesleep_tx_timer_expire;
	tx_timer.data = 0;

#if !BT_BLUEDROID_SUPPORT
	hci_register_notifier(&hci_event_nblock);
#endif

	pr_info("BlueSleep Mode Driver Initialized\n");

	return 0;

fail:
#if BT_BLUEDROID_SUPPORT
	remove_proc_entry("btwrite", sleep_dir);
	remove_proc_entry("lpm", sleep_dir);
#endif
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
	return retval;
}

static void __exit bluesleep_exit(void)
{
	if (bsi == NULL)
		return;

	
	if (bsi->has_ext_wake == 1)
		gpio_set_value(bsi->ext_wake, 0);
	clear_bit(BT_EXT_WAKE, &flags);
	if (test_bit(BT_PROTO, &flags)) {
		if (disable_irq_wake(bsi->host_wake_irq))
			pr_err("Couldn't disable hostwake IRQ wakeup mode\n");
		free_irq(bsi->host_wake_irq, NULL);
		del_timer(&tx_timer);
		if (test_bit(BT_ASLEEP, &flags))
			hsuart_power(1);
	}

#if !BT_BLUEDROID_SUPPORT
	hci_unregister_notifier(&hci_event_nblock);
#endif
	platform_driver_unregister(&bluesleep_driver);

#if BT_BLUEDROID_SUPPORT
	remove_proc_entry("btwrite", sleep_dir);
	remove_proc_entry("lpm", sleep_dir);
#endif
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
}

module_init(bluesleep_init);
module_exit(bluesleep_exit);

MODULE_DESCRIPTION("Bluetooth Sleep Mode Driver ver %s " VERSION);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
