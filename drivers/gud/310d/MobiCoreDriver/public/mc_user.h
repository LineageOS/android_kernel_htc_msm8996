/*
 * Copyright (c) 2013-2016 TRUSTONIC LIMITED
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

#ifndef _MC_USER_H_
#define _MC_USER_H_

#define MCDRVMODULEAPI_VERSION_MAJOR 3
#define MCDRVMODULEAPI_VERSION_MINOR 1

#include <linux/types.h>

#define MC_USER_DEVNODE		"mobicore-user"

#define MC_PRODUCT_ID_LEN	64

#define MC_MAP_MAX		4

#define BUFFER_LENGTH_MAX	0x100000

#define MC_IO_MAP_INPUT		0x1
#define MC_IO_MAP_OUTPUT	0x2
#define MC_IO_MAP_INPUT_OUTPUT	(MC_IO_MAP_INPUT | MC_IO_MAP_OUTPUT)

struct mc_uuid_t {
	__u8		value[16];	
};

enum mc_login_type {
	LOGIN_PUBLIC = 0,
	LOGIN_USER,
	LOGIN_GROUP,
	LOGIN_APPLICATION = 4,
	LOGIN_USER_APPLICATION,
	LOGIN_GROUP_APPLICATION,
};

struct mc_identity {
	enum mc_login_type	login_type;
	union {
		__u8		login_data[16];
		gid_t		gid;		
		struct {
			uid_t	euid;
			uid_t	ruid;
		} uid;
	};
	pid_t			pid;		
};

struct mc_ioctl_open_session {
	struct mc_uuid_t uuid;		
	__u32		is_gp_uuid;	
	__u32		sid;            
	__u64		tci;		
	__u32		tcilen;		
	struct mc_identity identity;	
};

struct mc_ioctl_open_trustlet {
	__u32		sid;		
	__u32		spid;		
	__u64		buffer;		
	__u32		tlen;		
	__u64		tci;		
	__u32		tcilen;		
};

struct mc_ioctl_wait {
	__u32		sid;		
	__s32		timeout;	
	__u32		partial;	
};

struct mc_ioctl_alloc {
	__u32		len;		
	__u32		handle;		
};

struct mc_ioctl_buffer {
	__u64		va;		
	__u32		len;		
	__u64		sva;		
	__u32		flags;		
};

struct mc_ioctl_map {
	__u32		sid;		
	struct mc_ioctl_buffer bufs[MC_MAP_MAX]; 
};

struct mc_ioctl_geterr {
	__u32		sid;		
	__s32		value;		
};

struct mc_version_info {
	char product_id[MC_PRODUCT_ID_LEN]; 
	__u32 version_mci;		
	__u32 version_so;		
	__u32 version_mclf;		
	__u32 version_container;	
	__u32 version_mc_config;	
	__u32 version_tl_api;		
	__u32 version_dr_api;		
	__u32 version_nwd;		
};

struct mc_authenticator_check {
	pid_t		pid;
};

#define MC_IOC_MAGIC	'M'

#define MC_IO_OPEN_SESSION \
	_IOWR(MC_IOC_MAGIC, 0, struct mc_ioctl_open_session)
#define MC_IO_OPEN_TRUSTLET \
	_IOWR(MC_IOC_MAGIC, 1, struct mc_ioctl_open_trustlet)
#define MC_IO_CLOSE_SESSION \
	_IO(MC_IOC_MAGIC, 2)
#define MC_IO_NOTIFY \
	_IO(MC_IOC_MAGIC, 3)
#define MC_IO_WAIT \
	_IOW(MC_IOC_MAGIC, 4, struct mc_ioctl_wait)
#define MC_IO_MAP \
	_IOWR(MC_IOC_MAGIC, 5, struct mc_ioctl_map)
#define MC_IO_UNMAP \
	_IOW(MC_IOC_MAGIC, 6, struct mc_ioctl_map)
#define MC_IO_ERR \
	_IOWR(MC_IOC_MAGIC, 7, struct mc_ioctl_geterr)
#define MC_IO_HAS_SESSIONS \
	_IO(MC_IOC_MAGIC, 8)
#define MC_IO_VERSION \
	_IOR(MC_IOC_MAGIC, 9, struct mc_version_info)
#define MC_IO_AUTHENTICATOR_CHECK \
	_IOW(MC_IOC_MAGIC, 10, struct mc_authenticator_check)

#endif 
