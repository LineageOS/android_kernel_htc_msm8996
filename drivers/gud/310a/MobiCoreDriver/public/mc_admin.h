/*
 * Copyright (c) 2013-2014 TRUSTONIC LIMITED
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

#ifndef __MC_ADMIN_IOCTL_H__
#define __MC_ADMIN_IOCTL_H__

#include <linux/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MC_ADMIN_DEVNODE "mobicore"

enum {
	
	MC_DRV_GET_ROOT_CONTAINER = 1,
	MC_DRV_GET_SP_CONTAINER = 2,
	MC_DRV_GET_TRUSTLET_CONTAINER = 3,
	MC_DRV_GET_TRUSTLET = 4,
	MC_DRV_SIGNAL_CRASH = 5,
};

#define MC_IOC_MAGIC    'M'

struct mc_admin_request {
	__u32		request_id;	
	__u32		command;	
	struct mc_uuid_t uuid;		
	__u32		is_gp;		
	__u32		spid;		
};

struct mc_admin_response {
	__u32		request_id;	
	__u32		error_no;	
	__u32		spid;		
	__u32		service_type;	
	__u32		length;		
	
};

struct mc_admin_driver_info {
	
	__u32		drv_version;
	__u32		initial_cmd_id;
};

struct mc_admin_load_info {
	__u32		spid;		
	__u64		address;	
	__u32		length;		
};

#define MC_ADMIN_IO_GET_DRIVER_REQUEST \
	_IOR(MC_IOC_MAGIC, 0, struct mc_admin_request)
#define MC_ADMIN_IO_GET_INFO  \
	_IOR(MC_IOC_MAGIC, 1, struct mc_admin_driver_info)
#define MC_ADMIN_IO_LOAD_DRIVER \
	_IOW(MC_IOC_MAGIC, 2, struct mc_admin_load_info)
#define MC_ADMIN_IO_LOAD_TOKEN \
	_IOW(MC_IOC_MAGIC, 3, struct mc_admin_load_info)
#define MC_ADMIN_IO_LOAD_CHECK \
	_IOW(MC_IOC_MAGIC, 4, struct mc_admin_load_info)

#ifdef __cplusplus
}
#endif
#endif 
