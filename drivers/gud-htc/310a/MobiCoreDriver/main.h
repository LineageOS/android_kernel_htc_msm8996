/*
 * Copyright (c) 2013-2015 TRUSTONIC LIMITED
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _MC_MAIN_H_
#define _MC_MAIN_H_

#include <linux/slab.h>		
#include <linux/fs.h>		

#define MC_VERSION(major, minor) \
		(((major & 0x0000ffff) << 16) | (minor & 0x0000ffff))
#define MC_VERSION_MAJOR(x) ((x) >> 16)
#define MC_VERSION_MINOR(x) ((x) & 0xffff)

#define mc_dev_err(fmt, ...) \
	dev_err(g_ctx.mcd, "%s: " fmt, __func__, ##__VA_ARGS__)

#define mc_dev_info(fmt, ...) \
	dev_info(g_ctx.mcd, "%s: " fmt, __func__, ##__VA_ARGS__)

#ifdef DEBUG
#define mc_dev_devel(fmt, ...) \
	dev_info(g_ctx.mcd, "%s: " fmt, __func__, ##__VA_ARGS__)
#else 
#define mc_dev_devel(...)		do {} while (0)
#endif 

struct mc_device_ctx {
	struct device		*mcd;
	
	struct dentry		*debug_dir;

	
	
	bool			f_lpae;
	
	bool			f_timeout;
	
	bool			f_mem_ext;
	
	bool			f_ta_auth;
	
	bool			f_multimap;
	
	bool			f_client_login;
	
	bool			f_time;

	
	atomic_t		c_clients;
	atomic_t		c_cbufs;
	atomic_t		c_sessions;
	atomic_t		c_wsms;
	atomic_t		c_mmus;
	atomic_t		c_maps;
};

extern struct mc_device_ctx g_ctx;

struct kasnprintf_buf {
	gfp_t gfp;
	void *buf;
	int size;
	int off;
};

extern __printf(2, 3)
int kasnprintf(struct kasnprintf_buf *buf, const char *fmt, ...);
ssize_t debug_generic_read(struct file *file, char __user *user_buf,
			   size_t count, loff_t *ppos,
			   int (*function)(struct kasnprintf_buf *buf));
int debug_generic_release(struct inode *inode, struct file *file);

static inline int kref_read(struct kref *kref)
{
	return atomic_read(&kref->refcount);
}

#endif 
