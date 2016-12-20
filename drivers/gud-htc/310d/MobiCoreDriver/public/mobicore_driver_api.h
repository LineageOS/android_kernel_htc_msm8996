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
#ifndef _MOBICORE_DRIVER_API_H_
#define _MOBICORE_DRIVER_API_H_

#include "mc_user.h"

#define __MC_CLIENT_LIB_API

enum mc_result {
	
	MC_DRV_OK				= 0,
	
	MC_DRV_NO_NOTIFICATION			= 1,
	
	MC_DRV_ERR_NOTIFICATION			= 2,
	
	MC_DRV_ERR_NOT_IMPLEMENTED		= 3,
	
	MC_DRV_ERR_OUT_OF_RESOURCES		= 4,
	
	MC_DRV_ERR_INIT				= 5,
	
	MC_DRV_ERR_UNKNOWN			= 6,
	
	MC_DRV_ERR_UNKNOWN_DEVICE		= 7,
	
	MC_DRV_ERR_UNKNOWN_SESSION		= 8,
	
	MC_DRV_ERR_INVALID_OPERATION		= 9,
	
	MC_DRV_ERR_INVALID_RESPONSE		= 10,
	
	MC_DRV_ERR_TIMEOUT			= 11,
	
	MC_DRV_ERR_NO_FREE_MEMORY		= 12,
	
	MC_DRV_ERR_FREE_MEMORY_FAILED		= 13,
	
	MC_DRV_ERR_SESSION_PENDING		= 14,
	
	MC_DRV_ERR_DAEMON_UNREACHABLE		= 15,
	
	MC_DRV_ERR_INVALID_DEVICE_FILE		= 16,
	
	MC_DRV_ERR_INVALID_PARAMETER		= 17,
	
	MC_DRV_ERR_KERNEL_MODULE		= 18,
	
	MC_DRV_ERR_BULK_MAPPING			= 19,
	
	MC_DRV_ERR_BULK_UNMAPPING		= 20,
	
	MC_DRV_INFO_NOTIFICATION		= 21,
	
	MC_DRV_ERR_NQ_FAILED			= 22,
	
	MC_DRV_ERR_DAEMON_VERSION		= 23,
	
	MC_DRV_ERR_CONTAINER_VERSION		= 24,
	
	MC_DRV_ERR_WRONG_PUBLIC_KEY		= 25,
	
	MC_DRV_ERR_CONTAINER_TYPE_MISMATCH	= 26,
	
	MC_DRV_ERR_CONTAINER_LOCKED		= 27,
	
	MC_DRV_ERR_SP_NO_CHILD			= 28,
	
	MC_DRV_ERR_TL_NO_CHILD			= 29,
	
	MC_DRV_ERR_UNWRAP_ROOT_FAILED		= 30,
	
	MC_DRV_ERR_UNWRAP_SP_FAILED		= 31,
	
	MC_DRV_ERR_UNWRAP_TRUSTLET_FAILED	= 32,
	
	MC_DRV_ERR_DAEMON_DEVICE_NOT_OPEN	= 33,
	
	MC_DRV_ERR_TA_ATTESTATION_ERROR		= 34,
	
	MC_DRV_ERR_INTERRUPTED_BY_SIGNAL	= 35,
	
	MC_DRV_ERR_SERVICE_BLOCKED		= 36,
	
	MC_DRV_ERR_SERVICE_LOCKED		= 37,
	
	MC_DRV_ERR_SERVICE_KILLED		= 38,
	
	MC_DRV_ERR_NO_FREE_INSTANCES		= 39,
	
	MC_DRV_ERR_TA_HEADER_ERROR		= 40,
};

struct mc_session_handle {
	u32	session_id;		
	u32	device_id;		
};

struct mc_bulk_map {
	u32	secure_virt_addr;
	u32	secure_virt_len;	
};

#define MC_DEVICE_ID_DEFAULT	0
#define MC_INFINITE_TIMEOUT	((s32)(-1))
#define MC_NO_TIMEOUT		0
#define MC_MAX_TCI_LEN		0x100000

__MC_CLIENT_LIB_API enum mc_result mc_open_device(
	u32				device_id);

__MC_CLIENT_LIB_API enum mc_result mc_close_device(
	u32				device_id);

__MC_CLIENT_LIB_API enum mc_result mc_open_session(
	struct mc_session_handle	*session,
	const struct mc_uuid_t		*uuid,
	u8				*tci,
	u32				tci_len);

__MC_CLIENT_LIB_API enum mc_result mc_open_trustlet(
	struct mc_session_handle	*session,
	u32				spid,
	u8				*trustlet,
	u32				trustlet_len,
	u8				*tci,
	u32				len);

__MC_CLIENT_LIB_API enum mc_result mc_close_session(
	struct mc_session_handle	*session);

__MC_CLIENT_LIB_API enum mc_result mc_notify(
	struct mc_session_handle	*session);

__MC_CLIENT_LIB_API enum mc_result mc_wait_notification(
	struct mc_session_handle	*session,
	s32				timeout);

__MC_CLIENT_LIB_API enum mc_result mc_malloc_wsm(
	u32				device_id,
	u32				align,
	u32				len,
	u8				**wsm,
	u32				wsm_flags);

__MC_CLIENT_LIB_API enum mc_result mc_free_wsm(
	u32				device_id,
	u8				*wsm);

__MC_CLIENT_LIB_API enum mc_result mc_map(
	struct mc_session_handle	*session,
	void				*buf,
	u32				len,
	struct mc_bulk_map		*map_info);

__MC_CLIENT_LIB_API enum mc_result mc_unmap(
	struct mc_session_handle	*session,
	void				*buf,
	struct mc_bulk_map		*map_info);

__MC_CLIENT_LIB_API enum mc_result mc_get_session_error_code(
	struct mc_session_handle	*session,
	s32				*exit_code);

#endif 
