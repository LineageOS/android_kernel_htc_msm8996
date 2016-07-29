/*
 * u_serial.c - utilities for USB gadget "serial port"/TTY support
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 * Copyright (c) 2013-2015 The Linux Foundation. All rights reserved.
 *
 * This code also borrows from usbserial.c, which is
 * Copyright (C) 1999 - 2002 Greg Kroah-Hartman (greg@kroah.com)
 * Copyright (C) 2000 Peter Berger (pberger@brimson.com)
 * Copyright (C) 2000 Al Borchers (alborchers@steinerpoint.com)
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */


#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>

#include "u_serial.h"



#define TX_QUEUE_SIZE		8
#define TX_BUF_SIZE		4096
#define WRITE_BUF_SIZE		8192		

#define RX_QUEUE_SIZE		8
#define RX_BUF_SIZE		4096
#define EXTRA_ALLOCATION_SIZE	256


struct gs_buf {
	unsigned		buf_size;
	char			*buf_buf;
	char			*buf_get;
	char			*buf_put;
};

struct gs_port {
	struct tty_port		port;
	spinlock_t		port_lock;	

	struct gserial		*port_usb;

	bool			openclose;	
	u8			port_num;

	struct list_head	read_pool;
	int read_started;
	int read_allocated;
	struct list_head	read_queue;
	unsigned		n_read;
	struct work_struct	push;

	struct list_head	write_pool;
	int write_started;
	int write_allocated;
	struct gs_buf		port_write_buf;
	wait_queue_head_t	drain_wait;	

	
	struct usb_cdc_line_coding port_line_coding;	
	unsigned long           nbytes_from_host;
	unsigned long           nbytes_to_tty;
	unsigned long           nbytes_from_tty;
	unsigned long           nbytes_to_host;
};

static struct portmaster {
	struct mutex	lock;			
	struct gs_port	*port;
} ports[MAX_U_SERIAL_PORTS];

static struct workqueue_struct *gserial_wq;

#define GS_CLOSE_TIMEOUT		15		



#ifdef VERBOSE_DEBUG
#ifndef pr_vdebug
#define pr_vdebug(fmt, arg...) \
	pr_debug(fmt, ##arg)
#endif 
#else
#ifndef pr_vdebug
#define pr_vdebug(fmt, arg...) \
	({ if (0) pr_debug(fmt, ##arg); })
#endif 
#endif



static int gs_buf_alloc(struct gs_buf *gb, unsigned size)
{
	gb->buf_buf = kmalloc(size, GFP_KERNEL);
	if (gb->buf_buf == NULL)
		return -ENOMEM;

	gb->buf_size = size;
	gb->buf_put = gb->buf_buf;
	gb->buf_get = gb->buf_buf;

	return 0;
}

static void gs_buf_free(struct gs_buf *gb)
{
	kfree(gb->buf_buf);
	gb->buf_buf = NULL;
}

static void gs_buf_clear(struct gs_buf *gb)
{
	gb->buf_get = gb->buf_put;
	
}

/*
 * gs_buf_data_avail
 *
 * Return the number of bytes of data written into the circular
 * buffer.
 */
static unsigned gs_buf_data_avail(struct gs_buf *gb)
{
	return (gb->buf_size + gb->buf_put - gb->buf_get) % gb->buf_size;
}

static unsigned gs_buf_space_avail(struct gs_buf *gb)
{
	return (gb->buf_size + gb->buf_get - gb->buf_put - 1) % gb->buf_size;
}

static unsigned
gs_buf_put(struct gs_buf *gb, const char *buf, unsigned count)
{
	unsigned len;

	len  = gs_buf_space_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_put;
	if (count > len) {
		memcpy(gb->buf_put, buf, len);
		memcpy(gb->buf_buf, buf+len, count - len);
		gb->buf_put = gb->buf_buf + count - len;
	} else {
		memcpy(gb->buf_put, buf, count);
		if (count < len)
			gb->buf_put += count;
		else 
			gb->buf_put = gb->buf_buf;
	}

	return count;
}

static unsigned
gs_buf_get(struct gs_buf *gb, char *buf, unsigned count)
{
	unsigned len;

	len = gs_buf_data_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_get;
	if (count > len) {
		memcpy(buf, gb->buf_get, len);
		memcpy(buf+len, gb->buf_buf, count - len);
		gb->buf_get = gb->buf_buf + count - len;
	} else {
		memcpy(buf, gb->buf_get, count);
		if (count < len)
			gb->buf_get += count;
		else 
			gb->buf_get = gb->buf_buf;
	}

	return count;
}



struct usb_request *
gs_alloc_req(struct usb_ep *ep, unsigned len, size_t extra_sz,
		gfp_t kmalloc_flags)
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, kmalloc_flags);

	if (req != NULL) {
		req->length = len;
		req->buf = kmalloc(len + extra_sz, kmalloc_flags);
		if (req->buf == NULL) {
			usb_ep_free_request(ep, req);
			return NULL;
		}
	}

	return req;
}
EXPORT_SYMBOL_GPL(gs_alloc_req);

void gs_free_req(struct usb_ep *ep, struct usb_request *req)
{
	kfree(req->buf);
	usb_ep_free_request(ep, req);
}
EXPORT_SYMBOL_GPL(gs_free_req);

static unsigned
gs_send_packet(struct gs_port *port, char *packet, unsigned size)
{
	unsigned len;

	len = gs_buf_data_avail(&port->port_write_buf);
	if (len < size)
		size = len;
	if (size != 0)
		size = gs_buf_get(&port->port_write_buf, packet, size);
	return size;
}

static int gs_start_tx(struct gs_port *port)
{
	struct list_head	*pool;
	struct usb_ep		*in;
	int			status = 0;
	static long 		prev_len;
	bool			do_tty_wake = false;

	if (!port || !port->port_usb) {
		pr_err("Error - port or port->usb is NULL.");
		return -EIO;
	}

	pool = &port->write_pool;
	in   = port->port_usb->in;

	while (!list_empty(pool)) {
		struct usb_request	*req;
		int			len;

		if (port->write_started >= TX_QUEUE_SIZE)
			break;

		req = list_entry(pool->next, struct usb_request, list);
		len = gs_send_packet(port, req->buf, TX_BUF_SIZE);
		if (len == 0) {
			if (prev_len && (prev_len % in->maxpacket == 0)) {
				req->length = 0;
				list_del(&req->list);
				spin_unlock(&port->port_lock);
				status = usb_ep_queue(in, req, GFP_ATOMIC);
				spin_lock(&port->port_lock);
				if (!port->port_usb) {
					gs_free_req(in, req);
					break;
				}
				if (status) {
					printk(KERN_ERR "%s: %s err %d\n",
					__func__, "queue", status);
					list_add(&req->list, pool);
				}
				prev_len = 0;
			}
			wake_up_interruptible(&port->drain_wait);
			break;
		}
		do_tty_wake = true;

		req->length = len;
		list_del(&req->list);

		pr_vdebug("ttyGS%d: tx len=%d, 0x%02x 0x%02x 0x%02x ...\n",
			  port->port_num, len, *((u8 *)req->buf),
			  *((u8 *)req->buf+1), *((u8 *)req->buf+2));

		spin_unlock(&port->port_lock);
		status = usb_ep_queue(in, req, GFP_ATOMIC);
		spin_lock(&port->port_lock);
		if (!port->port_usb) {
			do_tty_wake = false;
			gs_free_req(in, req);
			break;
		}
		if (status) {
			pr_debug("%s: %s %s err %d\n",
					__func__, "queue", in->name, status);
			list_add(&req->list, pool);
			break;
		}
		prev_len = req->length;
		port->nbytes_from_tty += req->length;

		port->write_started++;

	}

	if (do_tty_wake && port->port.tty)
		tty_wakeup(port->port.tty);
	return status;
}

static unsigned gs_start_rx(struct gs_port *port)
{
	struct list_head	*pool;
	struct usb_ep		*out;
	unsigned		started = 0;

	if (!port || !port->port_usb) {
		pr_err("Error - port or port->usb is NULL.");
		return -EIO;
	}

	pool = &port->read_pool;
	out  = port->port_usb->out;

	while (!list_empty(pool)) {
		struct usb_request	*req;
		int			status;
		struct tty_struct	*tty;

		
		tty = port->port.tty;
		if (!tty)
			break;

		if (port->read_started >= RX_QUEUE_SIZE)
			break;

		req = list_entry(pool->next, struct usb_request, list);
		list_del(&req->list);
		req->length = RX_BUF_SIZE;

		spin_unlock(&port->port_lock);
		status = usb_ep_queue(out, req, GFP_ATOMIC);
		spin_lock(&port->port_lock);
		if (!port->port_usb) {
			started = 0;
			gs_free_req(out, req);
			break;
		}
		if (status) {
			pr_debug("%s: %s %s err %d\n",
					__func__, "queue", out->name, status);
			list_add(&req->list, pool);
			break;
		}
		port->read_started++;

	}
	return port->read_started;
}

static void gs_rx_push(struct work_struct *w)
{
	struct gs_port		*port = container_of(w, struct gs_port, push);
	struct tty_struct	*tty;
	struct list_head	*queue = &port->read_queue;
	bool			disconnect = false;
	bool			do_push = false;

	
	spin_lock_irq(&port->port_lock);
	tty = port->port.tty;
	while (!list_empty(queue)) {
		struct usb_request	*req;

		req = list_first_entry(queue, struct usb_request, list);

		
		if (tty && test_bit(TTY_THROTTLED, &tty->flags))
			break;

		switch (req->status) {
		case -ESHUTDOWN:
			disconnect = true;
			pr_vdebug("ttyGS%d: shutdown\n", port->port_num);
			break;

		default:
			
			pr_warn("ttyGS%d: unexpected RX status %d\n",
				port->port_num, req->status);
			
		case 0:
			
			break;
		}

		
		if (req->actual) {
			char		*packet = req->buf;
			unsigned	size = req->actual;
			unsigned	n;
			int		count;

			
			n = port->n_read;
			if (n) {
				packet += n;
				size -= n;
			}

			count = tty_insert_flip_string(&port->port, packet,
					size);
			port->nbytes_to_tty += count;
			if (count)
				do_push = true;
			if (count != size) {
				
				port->n_read += count;
				pr_vdebug("ttyGS%d: rx block %d/%d\n",
					  port->port_num, count, req->actual);
				break;
			}
			port->n_read = 0;
		}

		list_move(&req->list, &port->read_pool);
		port->read_started--;
	}

	if (do_push) {
		if (port->port.low_latency)
			spin_unlock(&port->port_lock);
		tty_flip_buffer_push(&port->port);
		if (port->port.low_latency)
			spin_lock(&port->port_lock);
	}

	if (!list_empty(queue) && tty) {
		if (!test_bit(TTY_THROTTLED, &tty->flags)) {
			if (do_push)
				queue_work(gserial_wq, &port->push);
			else
				pr_warn("ttyGS%d: RX not scheduled?\n",
					port->port_num);
		}
	}

	
	if (!disconnect && port->port_usb)
		gs_start_rx(port);

	spin_unlock_irq(&port->port_lock);
}

static void gs_read_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct gs_port	*port = ep->driver_data;
	unsigned long flags;

	
	spin_lock_irqsave(&port->port_lock, flags);
	port->nbytes_from_host += req->actual;
	list_add_tail(&req->list, &port->read_queue);
	queue_work(gserial_wq, &port->push);
	spin_unlock_irqrestore(&port->port_lock, flags);
}

static void gs_write_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct gs_port	*port = ep->driver_data;
	unsigned long flags;

	spin_lock_irqsave(&port->port_lock, flags);
	port->nbytes_to_host += req->actual;
	list_add(&req->list, &port->write_pool);
	port->write_started--;

	switch (req->status) {
	default:
		
		pr_warning("%s: unexpected %s status %d\n",
				__func__, ep->name, req->status);
		
	case 0:
		
		if (port->port_usb)
			gs_start_tx(port);
		break;

	case -ESHUTDOWN:
		
		pr_vdebug("%s: %s shutdown\n", __func__, ep->name);
		break;
	}

	spin_unlock_irqrestore(&port->port_lock, flags);
}

static void gs_free_requests(struct usb_ep *ep, struct list_head *head,
							 int *allocated)
{
	struct usb_request	*req;

	while (!list_empty(head)) {
		req = list_entry(head->next, struct usb_request, list);
		list_del(&req->list);
		gs_free_req(ep, req);
		if (allocated)
			(*allocated)--;
	}
}

static int gs_alloc_requests(struct usb_ep *ep, struct list_head *head,
		int queue_size, int req_size, size_t extra_sz,
		void (*fn)(struct usb_ep *, struct usb_request *),
		int *allocated)
{
	int			i;
	struct usb_request	*req;
	int n = allocated ? queue_size - *allocated : queue_size;

	for (i = 0; i < n; i++) {
		req = gs_alloc_req(ep, req_size, extra_sz, GFP_ATOMIC);
		if (!req)
			return list_empty(head) ? -ENOMEM : 0;
		req->complete = fn;
		list_add_tail(&req->list, head);
		if (allocated)
			(*allocated)++;
	}
	return 0;
}

static int gs_start_io(struct gs_port *port)
{
	struct list_head	*head;
	struct usb_ep		*ep;
	int			status;
	unsigned		started;

	if (!port || !port->port_usb) {
		pr_err("Error - port or port->usb is NULL.");
		return -EIO;
	}

	head = &port->read_pool;
	ep = port->port_usb->out;

	status = gs_alloc_requests(ep, head, RX_QUEUE_SIZE, RX_BUF_SIZE, 0,
			 gs_read_complete, &port->read_allocated);
	if (status)
		return status;

	status = gs_alloc_requests(port->port_usb->in, &port->write_pool,
			TX_QUEUE_SIZE, TX_BUF_SIZE, EXTRA_ALLOCATION_SIZE,
			gs_write_complete, &port->write_allocated);
	if (status) {
		gs_free_requests(ep, head, &port->read_allocated);
		return status;
	}

	
	port->n_read = 0;
	started = gs_start_rx(port);

	if (!port->port_usb)
		return -EIO;
	
	if (started) {
		tty_wakeup(port->port.tty);
	} else {
		gs_free_requests(ep, head, &port->read_allocated);
		gs_free_requests(port->port_usb->in, &port->write_pool,
			&port->write_allocated);
		status = -EIO;
	}

	return status;
}



static int gs_open(struct tty_struct *tty, struct file *file)
{
	int		port_num = tty->index;
	struct gs_port	*port;
	int		status;

	do {
		mutex_lock(&ports[port_num].lock);
		port = ports[port_num].port;
		if (!port)
			status = -ENODEV;
		else {
			spin_lock_irq(&port->port_lock);

			
			if (port->port.count) {
				status = 0;
				port->port.count++;

			
			} else if (port->openclose) {
				status = -EBUSY;

			
			} else {
				status = -EAGAIN;
				port->openclose = true;
			}
			spin_unlock_irq(&port->port_lock);
		}
		mutex_unlock(&ports[port_num].lock);

		switch (status) {
		default:
			
			return status;
		case -EAGAIN:
			
			break;
		case -EBUSY:
			
			msleep(1);
			break;
		}
	} while (status != -EAGAIN);

	
	spin_lock_irq(&port->port_lock);

	
	if (port->port_write_buf.buf_buf == NULL) {

		spin_unlock_irq(&port->port_lock);
		status = gs_buf_alloc(&port->port_write_buf, WRITE_BUF_SIZE);
		spin_lock_irq(&port->port_lock);

		if (status) {
			pr_debug("gs_open: ttyGS%d (%p,%p) no buffer\n",
				port->port_num, tty, file);
			port->openclose = false;
			goto exit_unlock_port;
		}
	}


	

	tty->driver_data = port;
	port->port.tty = tty;

	port->port.count = 1;
	port->openclose = false;

	
	if (port->port_usb) {
		struct gserial	*gser = port->port_usb;

		if (gser->flags & ASYNC_LOW_LATENCY) {
			pr_debug("%s: Setting to low latency", __func__);
			tty->port->low_latency = 1;
		}

		pr_debug("gs_open: start ttyGS%d\n", port->port_num);
		gs_start_io(port);

		if (gser->connect)
			gser->connect(gser);
	}

	pr_debug("gs_open: ttyGS%d (%p,%p)\n", port->port_num, tty, file);

	status = 0;

exit_unlock_port:
	spin_unlock_irq(&port->port_lock);
	return status;
}

static int gs_writes_finished(struct gs_port *p)
{
	int cond;

	
	spin_lock_irq(&p->port_lock);
	cond = (p->port_usb == NULL) || !gs_buf_data_avail(&p->port_write_buf);
	spin_unlock_irq(&p->port_lock);

	return cond;
}

static void gs_close(struct tty_struct *tty, struct file *file)
{
	struct gs_port *port = tty->driver_data;
	struct gserial	*gser;

	spin_lock_irq(&port->port_lock);

	if (port->port.count != 1) {
		if (port->port.count == 0)
			WARN_ON(1);
		else
			--port->port.count;
		goto exit;
	}

	pr_debug("gs_close: ttyGS%d (%p,%p) ...\n", port->port_num, tty, file);

	port->openclose = true;
	port->port.count = 0;

	gser = port->port_usb;
	if (gser && gser->disconnect)
		gser->disconnect(gser);

	if (gs_buf_data_avail(&port->port_write_buf) > 0 && gser) {
		spin_unlock_irq(&port->port_lock);
		wait_event_interruptible_timeout(port->drain_wait,
					gs_writes_finished(port),
					GS_CLOSE_TIMEOUT * HZ);
		spin_lock_irq(&port->port_lock);
		gser = port->port_usb;
	}

	if (gser == NULL)
		gs_buf_free(&port->port_write_buf);
	else
		gs_buf_clear(&port->port_write_buf);

	tty->driver_data = NULL;
	port->port.tty = NULL;

	port->openclose = false;

	pr_debug("gs_close: ttyGS%d (%p,%p) done!\n",
			port->port_num, tty, file);

	wake_up(&port->port.close_wait);
exit:
	spin_unlock_irq(&port->port_lock);
}

static int gs_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	struct gs_port	*port = tty->driver_data;
	unsigned long	flags;
	int		status;

	if (!port)
		return 0;
	pr_vdebug("gs_write: ttyGS%d (%p) writing %d bytes\n",
			port->port_num, tty, count);

	spin_lock_irqsave(&port->port_lock, flags);
	if (count)
		count = gs_buf_put(&port->port_write_buf, buf, count);
	
	if (port->port_usb)
		status = gs_start_tx(port);
	spin_unlock_irqrestore(&port->port_lock, flags);

	return count;
}

static int gs_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct gs_port	*port = tty->driver_data;
	unsigned long	flags;
	int		status;

	if (!port)
		return 0;
	pr_vdebug("gs_put_char: (%d,%p) char=0x%x, called from %pf\n",
		port->port_num, tty, ch, __builtin_return_address(0));

	spin_lock_irqsave(&port->port_lock, flags);
	status = gs_buf_put(&port->port_write_buf, &ch, 1);
	spin_unlock_irqrestore(&port->port_lock, flags);

	return status;
}

static void gs_flush_chars(struct tty_struct *tty)
{
	struct gs_port	*port = tty->driver_data;
	unsigned long	flags;

	if (!port)
		return;
	pr_vdebug("gs_flush_chars: (%d,%p)\n", port->port_num, tty);

	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb)
		gs_start_tx(port);
	spin_unlock_irqrestore(&port->port_lock, flags);
}

static int gs_write_room(struct tty_struct *tty)
{
	struct gs_port	*port = tty->driver_data;
	unsigned long	flags;
	int		room = 0;

	if (!port)
		return 0;
	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb)
		room = gs_buf_space_avail(&port->port_write_buf);
	spin_unlock_irqrestore(&port->port_lock, flags);

	pr_vdebug("gs_write_room: (%d,%p) room=%d\n",
		port->port_num, tty, room);

	return room;
}

static int gs_chars_in_buffer(struct tty_struct *tty)
{
	struct gs_port	*port = tty->driver_data;
	unsigned long	flags;
	int		chars = 0;

	if (!port)
		return 0;
	spin_lock_irqsave(&port->port_lock, flags);
	chars = gs_buf_data_avail(&port->port_write_buf);
	spin_unlock_irqrestore(&port->port_lock, flags);

	pr_vdebug("gs_chars_in_buffer: (%d,%p) chars=%d\n",
		port->port_num, tty, chars);

	return chars;
}

static void gs_unthrottle(struct tty_struct *tty)
{
	struct gs_port		*port = tty->driver_data;
	unsigned long		flags;

	if (!port)
		return;

	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb) {
		queue_work(gserial_wq, &port->push);
		pr_vdebug("ttyGS%d: unthrottle\n", port->port_num);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);
}

static int gs_break_ctl(struct tty_struct *tty, int duration)
{
	struct gs_port	*port = tty->driver_data;
	int		status = 0;
	struct gserial	*gser;

	if (!port)
		return 0;
	pr_vdebug("gs_break_ctl: ttyGS%d, send break (%d) \n",
			port->port_num, duration);

	spin_lock_irq(&port->port_lock);
	gser = port->port_usb;
	if (gser && gser->send_break)
		status = gser->send_break(gser, duration);
	spin_unlock_irq(&port->port_lock);

	return status;
}

static int gs_tiocmget(struct tty_struct *tty)
{
	struct gs_port	*port = tty->driver_data;
	struct gserial	*gser;
	unsigned int result = 0;

	if (!port)
		return -ENODEV;
	spin_lock_irq(&port->port_lock);
	gser = port->port_usb;
	if (!gser) {
		result = -ENODEV;
		goto fail;
	}

	if (gser->get_dtr)
		result |= (gser->get_dtr(gser) ? TIOCM_DTR : 0);

	if (gser->get_rts)
		result |= (gser->get_rts(gser) ? TIOCM_RTS : 0);

	if (gser->serial_state & TIOCM_CD)
		result |= TIOCM_CD;

	if (gser->serial_state & TIOCM_RI)
		result |= TIOCM_RI;
fail:
	spin_unlock_irq(&port->port_lock);
	return result;
}

static int gs_tiocmset(struct tty_struct *tty,
	unsigned int set, unsigned int clear)
{
	struct gs_port	*port = tty->driver_data;
	struct gserial *gser;
	int	status = 0;

	if (!port)
		return -ENODEV;
	spin_lock_irq(&port->port_lock);
	gser = port->port_usb;
	if (!gser) {
		status = -ENODEV;
		goto fail;
	}

	if (set & TIOCM_RI) {
		if (gser->send_ring_indicator) {
			gser->serial_state |= TIOCM_RI;
			status = gser->send_ring_indicator(gser, 1);
		}
	}
	if (clear & TIOCM_RI) {
		if (gser->send_ring_indicator) {
			gser->serial_state &= ~TIOCM_RI;
			status = gser->send_ring_indicator(gser, 0);
		}
	}
	if (set & TIOCM_CD) {
		if (gser->send_carrier_detect) {
			gser->serial_state |= TIOCM_CD;
			status = gser->send_carrier_detect(gser, 1);
		}
	}
	if (clear & TIOCM_CD) {
		if (gser->send_carrier_detect) {
			gser->serial_state &= ~TIOCM_CD;
			status = gser->send_carrier_detect(gser, 0);
		}
	}
fail:
	spin_unlock_irq(&port->port_lock);
	return status;
}
static const struct tty_operations gs_tty_ops = {
	.open =			gs_open,
	.close =		gs_close,
	.write =		gs_write,
	.put_char =		gs_put_char,
	.flush_chars =		gs_flush_chars,
	.write_room =		gs_write_room,
	.chars_in_buffer =	gs_chars_in_buffer,
	.unthrottle =		gs_unthrottle,
	.break_ctl =		gs_break_ctl,
	.tiocmget  =		gs_tiocmget,
	.tiocmset  =		gs_tiocmset,
};


static struct tty_driver *gs_tty_driver;

static int
gs_port_alloc(unsigned port_num, struct usb_cdc_line_coding *coding)
{
	struct gs_port	*port;
	int		ret = 0;

	mutex_lock(&ports[port_num].lock);
	if (ports[port_num].port) {
		ret = -EBUSY;
		goto out;
	}

	port = kzalloc(sizeof(struct gs_port), GFP_KERNEL);
	if (port == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	tty_port_init(&port->port);
	tty_buffer_set_limit(&port->port, 131072);
	spin_lock_init(&port->port_lock);
	init_waitqueue_head(&port->drain_wait);

	INIT_WORK(&port->push, gs_rx_push);

	INIT_LIST_HEAD(&port->read_pool);
	INIT_LIST_HEAD(&port->read_queue);
	INIT_LIST_HEAD(&port->write_pool);

	port->port_num = port_num;
	port->port_line_coding = *coding;

	ports[port_num].port = port;
out:
	mutex_unlock(&ports[port_num].lock);
	return ret;
}


#if defined(CONFIG_DEBUG_FS)

#define BUF_SIZE	512

static ssize_t debug_read_status(struct file *file, char __user *ubuf,
					size_t count, loff_t *ppos)
{
	struct gs_port *ui_dev = file->private_data;
	struct tty_struct       *tty;
	struct gserial		*gser;
	char *buf;
	unsigned long flags;
	int i = 0;
	int ret;
	int result = 0;

	tty = ui_dev->port.tty;
	gser = ui_dev->port_usb;

	buf = kzalloc(sizeof(char) * BUF_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	spin_lock_irqsave(&ui_dev->port_lock, flags);

	i += scnprintf(buf + i, BUF_SIZE - i,
		"nbytes_from_host: %lu\n", ui_dev->nbytes_from_host);

	i += scnprintf(buf + i, BUF_SIZE - i,
		"nbytes_to_tty: %lu\n", ui_dev->nbytes_to_tty);

	i += scnprintf(buf + i, BUF_SIZE - i, "nbytes_with_usb_OUT_txr: %lu\n",
			(ui_dev->nbytes_from_host - ui_dev->nbytes_to_tty));

	i += scnprintf(buf + i, BUF_SIZE - i,
		"nbytes_from_tty: %lu\n", ui_dev->nbytes_from_tty);

	i += scnprintf(buf + i, BUF_SIZE - i,
		"nbytes_to_host: %lu\n", ui_dev->nbytes_to_host);

	i += scnprintf(buf + i, BUF_SIZE - i, "nbytes_with_usb_IN_txr: %lu\n",
			(ui_dev->nbytes_from_tty - ui_dev->nbytes_to_host));

	if (tty)
		i += scnprintf(buf + i, BUF_SIZE - i,
			"tty_flags: %lu\n", tty->flags);

	if (gser->get_dtr) {
		result |= (gser->get_dtr(gser) ? TIOCM_DTR : 0);
		i += scnprintf(buf + i, BUF_SIZE - i,
			"DTR_status: %d\n", result);
	}

	spin_unlock_irqrestore(&ui_dev->port_lock, flags);

	ret = simple_read_from_buffer(ubuf, count, ppos, buf, i);

	kfree(buf);

	return ret;
}

static ssize_t debug_write_reset(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	struct gs_port *ui_dev = file->private_data;
	unsigned long flags;

	spin_lock_irqsave(&ui_dev->port_lock, flags);
	ui_dev->nbytes_from_host = ui_dev->nbytes_to_tty =
			ui_dev->nbytes_from_tty = ui_dev->nbytes_to_host = 0;
	spin_unlock_irqrestore(&ui_dev->port_lock, flags);

	return count;
}

static int serial_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

const struct file_operations debug_rst_ops = {
	.open = serial_debug_open,
	.write = debug_write_reset,
};

const struct file_operations debug_adb_ops = {
	.open = serial_debug_open,
	.read = debug_read_status,
};

struct dentry *gs_dent;
static void usb_debugfs_init(struct gs_port *ui_dev, int port_num)
{
	char buf[48];

	snprintf(buf, 48, "usb_serial%d", port_num);
	gs_dent = debugfs_create_dir(buf, 0);
	if (!gs_dent || IS_ERR(gs_dent))
		return;

	debugfs_create_file("readstatus", 0444, gs_dent, ui_dev,
			&debug_adb_ops);
	debugfs_create_file("reset", S_IRUGO | S_IWUSR,
			gs_dent, ui_dev, &debug_rst_ops);
}

static void usb_debugfs_remove(void)
{
	debugfs_remove_recursive(gs_dent);
}
#else
static inline void usb_debugfs_init(struct gs_port *ui_dev, int port_num) {}
static inline void usb_debugfs_remove(void) {}
#endif

static int gs_closed(struct gs_port *port)
{
	int cond;

	spin_lock_irq(&port->port_lock);
	cond = (port->port.count == 0) && !port->openclose;
	spin_unlock_irq(&port->port_lock);
	return cond;
}

static void gserial_free_port(struct gs_port *port)
{
	cancel_work_sync(&port->push);
	
	wait_event(port->port.close_wait, gs_closed(port));
	WARN_ON(port->port_usb != NULL);
	tty_port_destroy(&port->port);
	kfree(port);
}

void gserial_free_line(unsigned char port_num)
{
	struct gs_port	*port;

	mutex_lock(&ports[port_num].lock);
	if (WARN_ON(!ports[port_num].port)) {
		mutex_unlock(&ports[port_num].lock);
		return;
	}
	port = ports[port_num].port;
	ports[port_num].port = NULL;
	mutex_unlock(&ports[port_num].lock);

	gserial_free_port(port);
	tty_unregister_device(gs_tty_driver, port_num);
}
EXPORT_SYMBOL_GPL(gserial_free_line);

int gserial_alloc_line(unsigned char *line_num)
{
	struct usb_cdc_line_coding	coding;
	struct device			*tty_dev;
	int				ret;
	int				port_num;

	coding.dwDTERate = cpu_to_le32(9600);
	coding.bCharFormat = 8;
	coding.bParityType = USB_CDC_NO_PARITY;
	coding.bDataBits = USB_CDC_1_STOP_BITS;

	for (port_num = 0; port_num < MAX_U_SERIAL_PORTS; port_num++) {
		ret = gs_port_alloc(port_num, &coding);
		if (ret == -EBUSY)
			continue;
		if (ret)
			return ret;
		break;
	}
	if (ret)
		return ret;

	

	tty_dev = tty_port_register_device(&ports[port_num].port->port,
			gs_tty_driver, port_num, NULL);

	if (IS_ERR(tty_dev)) {
		struct gs_port	*port;
		pr_err("%s: failed to register tty for port %d, err %ld\n",
				__func__, port_num, PTR_ERR(tty_dev));

		ret = PTR_ERR(tty_dev);
		port = ports[port_num].port;
		ports[port_num].port = NULL;
		gserial_free_port(port);
		goto err;
	}
	*line_num = port_num;
err:
	return ret;
}
EXPORT_SYMBOL_GPL(gserial_alloc_line);

/**
 * gserial_connect - notify TTY I/O glue that USB link is active
 * @gser: the function, set up with endpoints and descriptors
 * @port_num: which port is active
 * Context: any (usually from irq)
 *
 * This is called activate endpoints and let the TTY layer know that
 * the connection is active ... not unlike "carrier detect".  It won't
 * necessarily start I/O queues; unless the TTY is held open by any
 * task, there would be no point.  However, the endpoints will be
 * activated so the USB host can perform I/O, subject to basic USB
 * hardware flow control.
 *
 * Caller needs to have set up the endpoints and USB function in @dev
 * before calling this, as well as the appropriate (speed-specific)
 * endpoint descriptors, and also have allocate @port_num by calling
 * @gserial_alloc_line().
 *
 * Returns negative errno or zero.
 * On success, ep->driver_data will be overwritten.
 */
int gserial_connect(struct gserial *gser, u8 port_num)
{
	struct gs_port	*port;
	unsigned long	flags;
	int		status;

	if (port_num >= MAX_U_SERIAL_PORTS)
		return -ENXIO;

	port = ports[port_num].port;
	if (!port) {
		pr_err("serial line %d not allocated.\n", port_num);
		return -EINVAL;
	}
	if (port->port_usb) {
		pr_err("serial line %d is in use.\n", port_num);
		return -EBUSY;
	}

	
	status = usb_ep_enable(gser->in);
	if (status < 0)
		return status;
	gser->in->driver_data = port;

	status = usb_ep_enable(gser->out);
	if (status < 0)
		goto fail_out;
	gser->out->driver_data = port;

	
	spin_lock_irqsave(&port->port_lock, flags);
	gser->ioport = port;
	port->port_usb = gser;

	gser->port_line_coding = port->port_line_coding;

	

	if (port->port.count) {
		pr_debug("gserial_connect: start ttyGS%d\n", port->port_num);
		if (gser->flags & ASYNC_LOW_LATENCY) {
			pr_debug("%s: Setting to low latency", __func__);
			gser->ioport->port.tty->port->low_latency = 1;
		}
		gs_start_io(port);
		if (gser->connect)
			gser->connect(gser);
	} else {
		if (gser->disconnect)
			gser->disconnect(gser);
	}

	spin_unlock_irqrestore(&port->port_lock, flags);

	return status;

fail_out:
	usb_ep_disable(gser->in);
	gser->in->driver_data = NULL;
	return status;
}
EXPORT_SYMBOL_GPL(gserial_connect);
void gserial_disconnect(struct gserial *gser)
{
	struct gs_port	*port = gser->ioport;
	unsigned long	flags;

	if (!port)
		return;

	
	spin_lock_irqsave(&port->port_lock, flags);

	
	port->port_line_coding = gser->port_line_coding;

	port->port_usb = NULL;
	gser->ioport = NULL;
	if (port->port.count > 0 || port->openclose) {
		wake_up_interruptible(&port->drain_wait);
		if (port->port.tty)
			tty_hangup(port->port.tty);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);

	
	usb_ep_disable(gser->out);
	gser->out->driver_data = NULL;

	usb_ep_disable(gser->in);
	gser->in->driver_data = NULL;

	
	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port.count == 0 && !port->openclose)
		gs_buf_free(&port->port_write_buf);
	gs_free_requests(gser->out, &port->read_pool, NULL);
	gs_free_requests(gser->out, &port->read_queue, NULL);
	gs_free_requests(gser->in, &port->write_pool, NULL);

	port->read_allocated = port->read_started =
		port->write_allocated = port->write_started = 0;

	port->nbytes_from_host = port->nbytes_to_tty =
		port->nbytes_from_tty = port->nbytes_to_host = 0;

	spin_unlock_irqrestore(&port->port_lock, flags);
}
EXPORT_SYMBOL_GPL(gserial_disconnect);

static int userial_init(void)
{
	unsigned			i;
	int				status;

	gs_tty_driver = alloc_tty_driver(MAX_U_SERIAL_PORTS);
	if (!gs_tty_driver)
		return -ENOMEM;

	gs_tty_driver->driver_name = "g_serial";
	gs_tty_driver->name = "ttyHSUSB"; 
	

	gs_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	gs_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	gs_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV
				| TTY_DRIVER_RESET_TERMIOS;
	gs_tty_driver->init_termios = tty_std_termios;

	gs_tty_driver->init_termios.c_cflag =
			B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	gs_tty_driver->init_termios.c_ispeed = 9600;
	gs_tty_driver->init_termios.c_ospeed = 9600;

	gs_tty_driver->init_termios.c_lflag = 0;
	gs_tty_driver->init_termios.c_iflag = 0;
	gs_tty_driver->init_termios.c_oflag = 0;

	tty_set_operations(gs_tty_driver, &gs_tty_ops);
	for (i = 0; i < MAX_U_SERIAL_PORTS; i++)
		mutex_init(&ports[i].lock);

	gserial_wq = create_singlethread_workqueue("k_gserial");
	if (!gserial_wq) {
		status = -ENOMEM;
		goto fail;
	}

	
	status = tty_register_driver(gs_tty_driver);
	if (status) {
		pr_err("%s: cannot register, err %d\n",
				__func__, status);
		goto fail;
	}

	for (i = 0; i < MAX_U_SERIAL_PORTS; i++)
		usb_debugfs_init(ports[i].port, i);

	pr_debug("%s: registered %d ttyGS* device%s\n", __func__,
			MAX_U_SERIAL_PORTS,
			(MAX_U_SERIAL_PORTS == 1) ? "" : "s");

	return status;
fail:
	put_tty_driver(gs_tty_driver);
	if (gserial_wq)
		destroy_workqueue(gserial_wq);
	gs_tty_driver = NULL;
	return status;
}
module_init(userial_init);

static void userial_cleanup(void)
{
	usb_debugfs_remove();
	destroy_workqueue(gserial_wq);
	tty_unregister_driver(gs_tty_driver);
	put_tty_driver(gs_tty_driver);
	gs_tty_driver = NULL;
}
module_exit(userial_cleanup);

MODULE_LICENSE("GPL");
