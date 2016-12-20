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

#ifndef MCP_H_
#define MCP_H_

#include "mci/mcloadformat.h"

#define FLAG_RESPONSE		BIT(31)

#define MCP_MAP_MAX_BUF		4

enum mcp_result {
	
	MC_MCP_RET_OK                                   =  0,
	
	MC_MCP_RET_ERR_INVALID_SESSION                  =  1,
	
	MC_MCP_RET_ERR_UNKNOWN_UUID                     =  2,
	
	MC_MCP_RET_ERR_UNKNOWN_DRIVER_ID                =  3,
	
	MC_MCP_RET_ERR_NO_MORE_SESSIONS                 =  4,
	
	MC_MCP_RET_ERR_CONTAINER_INVALID                =  5,
	
	MC_MCP_RET_ERR_TRUSTLET_INVALID                 =  6,
	
	MC_MCP_RET_ERR_ALREADY_MAPPED                   =  7,
	
	MC_MCP_RET_ERR_INVALID_PARAM                    =  8,
	
	MC_MCP_RET_ERR_OUT_OF_RESOURCES                 =  9,
	
	MC_MCP_RET_ERR_INVALID_WSM                      = 10,
	
	MC_MCP_RET_ERR_UNKNOWN                          = 11,
	
	MC_MCP_RET_ERR_INVALID_MAPPING_LENGTH           = 12,
	
	MC_MCP_RET_ERR_MAPPING_TARGET                   = 13,
	
	MC_MCP_RET_ERR_OUT_OF_CRYPTO_RESOURCES          = 14,
	
	MC_MCP_RET_ERR_SIGNATURE_VERIFICATION_FAILED    = 15,
	
	MC_MCP_RET_ERR_WRONG_PUBLIC_KEY                 = 16,
	
	MC_MCP_RET_ERR_CONTAINER_TYPE_MISMATCH          = 17,
	
	MC_MCP_RET_ERR_CONTAINER_LOCKED                 = 18,
	
	MC_MCP_RET_ERR_SP_NO_CHILD                      = 19,
	
	MC_MCP_RET_ERR_TL_NO_CHILD                      = 20,
	
	MC_MCP_RET_ERR_UNWRAP_ROOT_FAILED               = 21,
	
	MC_MCP_RET_ERR_UNWRAP_SP_FAILED                 = 22,
	
	MC_MCP_RET_ERR_UNWRAP_TRUSTLET_FAILED           = 23,
	
	MC_MCP_RET_ERR_CONTAINER_VERSION_MISMATCH       = 24,
	
	MC_MCP_RET_ERR_SP_TL_DECRYPTION_FAILED          = 25,
	
	MC_MCP_RET_ERR_SP_TL_HASH_CHECK_FAILED          = 26,
	
	MC_MCP_RET_ERR_LAUNCH_TASK_FAILED               = 27,
	
	MC_MCP_RET_ERR_CLOSE_TASK_FAILED                = 28,
	
	MC_MCP_RET_ERR_SERVICE_BLOCKED                  = 29,
	
	MC_MCP_RET_ERR_SERVICE_LOCKED                   = 30,
	
	MC_MCP_RET_ERR_SERVICE_KILLED                   = 31,
	
	MC_MCP_RET_ERR_UNKNOWN_COMMAND                  = 50,
	
	MC_MCP_RET_ERR_INVALID_DATA                     = 51
};

enum cmd_id {
	
	MC_MCP_CMD_ID_INVALID		= 0x00,
	
	MC_MCP_CMD_OPEN_SESSION		= 0x01,
	
	MC_MCP_CMD_CLOSE_SESSION	= 0x03,
	
	MC_MCP_CMD_MAP			= 0x04,
	
	MC_MCP_CMD_UNMAP		= 0x05,
	
	MC_MCP_CMD_SUSPEND		= 0x06,
	
	MC_MCP_CMD_RESUME		= 0x07,
	
	MC_MCP_CMD_GET_MOBICORE_VERSION	= 0x09,
	
	MC_MCP_CMD_CLOSE_MCP		= 0x0A,
	
	MC_MCP_CMD_LOAD_TOKEN		= 0x0B,
	
	MC_MCP_CMD_CHECK_LOAD_TA	= 0x0C,
	
	MC_MCP_CMD_MULTIMAP		= 0x0D,
	
	MC_MCP_CMD_MULTIUNMAP		= 0x0E,
};

#define WSM_TYPE_MASK		0xFF
#define WSM_INVALID		0	
#define WSM_L2			2	
#define WSM_L1			3	

#define MC_GP_CLIENT_AUTH_MAGIC	0x47504131	

#define MC_IV_FLAG_IRQ		BIT(0)	
#define MC_IV_FLAG_TIME		BIT(1)	

struct init_values {
	u32	flags;
	u32	irq;
	u32	time_ofs;
	u32	time_len;
};

struct cmd_header {
	enum cmd_id	cmd_id;	
};

/** Response header.
 * MobiCore will reply to every MCP command with an MCP response.  Like the MCP
 * command the response consists of a header followed by response data. The
 * response is written to the same memory location as the MCP command.
 */
struct rsp_header {
	u32		rsp_id;	
	enum mcp_result	result;	
};




struct cmd_get_version {
	struct cmd_header	cmd_header;	
};

struct rsp_get_version {
	struct rsp_header	rsp_header;	
	struct mc_version_info	version_info;	
};



struct cmd_suspend {
	struct cmd_header	cmd_header;	
};

struct rsp_suspend {
	struct rsp_header	rsp_header;	
};


struct cmd_resume {
	struct cmd_header	cmd_header;	
};

struct rsp_resume {
	struct rsp_header	rsp_header;	
};



struct cmd_open_data {
	u32		mclf_magic;	
	struct identity	identity;	
};

struct cmd_open {
	struct cmd_header cmd_header;	
	struct mc_uuid_t uuid;		
	u8		unused[4];	
	u64		adr_tci_buffer;	
	u64		adr_load_data;	
	u32		ofs_tci_buffer;	
	u32		len_tci_buffer;	
	u32		wsmtype_tci;	
	u32		wsm_data_type;	
	u32		ofs_load_data;	
	u32		len_load_data;	
	union {
		struct cmd_open_data	cmd_open_data;	
		union mclf_header	tl_header;	
	};
	u32		is_gpta;	
};

struct rsp_open {
	struct rsp_header	rsp_header;	
	u32	session_id;	
};

struct cmd_check_load {
	struct cmd_header cmd_header;	
	struct mc_uuid_t uuid;	
	u8      unused[4];      
	u64		adr_load_data;	
	u32		wsm_data_type;	
	u32		ofs_load_data;	
	u32		len_load_data;	
	union mclf_header tl_header;	
};

struct rsp_check_load {
	struct rsp_header	rsp_header;	
};


struct cmd_close {
	struct cmd_header	cmd_header;	
	u32		session_id;	
};

struct rsp_close {
	struct rsp_header	rsp_header;	
};


struct cmd_map {
	struct cmd_header cmd_header;	
	u32		session_id;	
	u32		wsm_type;	
	u32		ofs_buffer;	
	u64		adr_buffer;	
	u32		len_buffer;	
	u8		unused[4];	
};

#define MCP_MAP_MAX         0x100000    

struct rsp_map {
	struct rsp_header rsp_header;	
	
	u32		secure_va;
};


struct cmd_unmap {
	struct cmd_header cmd_header;	
	u32		session_id;	
	u32		wsm_type;	
	
	u32		secure_va;
	u32		virtual_buffer_len;  
};

struct rsp_unmap {
	struct rsp_header rsp_header;	
};


struct cmd_load_token {
	struct cmd_header cmd_header;	
	u32		wsm_data_type;	
	u64		adr_load_data;	
	u64		ofs_load_data;	
	u64		len_load_data;	
};

struct rsp_load_token {
	struct rsp_header rsp_header;	
};


struct buffer_map {
	u64		adr_buffer;	
	u32		ofs_buffer;	
	u32		len_buffer;	
	u32		wsm_type;	
	u8		unused[4];	
};

struct cmd_multimap {
	struct cmd_header cmd_header;	
	u32		session_id;	
	struct buffer_map bufs[MC_MAP_MAX]; 
};

struct rsp_multimap {
	struct rsp_header rsp_header;	
	
	u64		secure_va[MC_MAP_MAX];
};


struct buffer_unmap {
	u64		secure_va;	
	u32		len_buffer;	
	u8		unused[4];	
};

struct cmd_multiunmap {
	struct cmd_header cmd_header;	
	u32		session_id;	
	struct buffer_unmap bufs[MC_MAP_MAX]; 
};

struct rsp_multiunmap {
	struct rsp_header rsp_header;	
};

union mcp_message {
	struct init_values	init_values;	
	struct cmd_header	cmd_header;	
	struct rsp_header	rsp_header;
	struct cmd_open		cmd_open;	
	struct rsp_open		rsp_open;
	struct cmd_close	cmd_close;	
	struct rsp_close	rsp_close;
	struct cmd_map		cmd_map;	
	struct rsp_map		rsp_map;
	struct cmd_unmap	cmd_unmap;	
	struct rsp_unmap	rsp_unmap;
	struct cmd_suspend	cmd_suspend;	
	struct rsp_suspend	rsp_suspend;
	struct cmd_resume	cmd_resume;	
	struct rsp_resume	rsp_resume;
	struct cmd_get_version	cmd_get_version; 
	struct rsp_get_version	rsp_get_version;
	struct cmd_load_token	cmd_load_token;	
	struct rsp_load_token	rsp_load_token;
	struct cmd_check_load	cmd_check_load;	
	struct rsp_check_load	rsp_check_load;
	struct cmd_multimap	cmd_multimap;	
	struct rsp_multimap	rsp_multimap;
	struct cmd_multiunmap	cmd_multiunmap;	
	struct rsp_multiunmap	rsp_multiunmap;
};

#define MIN_MCP_LEN         sizeof(mcp_message_t)

#define MC_FLAG_NO_SLEEP_REQ   0
#define MC_FLAG_REQ_TO_SLEEP   1

#define MC_STATE_NORMAL_EXECUTION 0
#define MC_STATE_READY_TO_SLEEP   1

struct sleep_mode {
	u16		sleep_req;	
	u16		ready_to_sleep;	
};

struct mcp_flags {
	
	u32		schedule;
	struct sleep_mode sleep_mode;
	
	s32		timeout_ms;
	
	u32		RFU3;
};

#define MC_FLAG_SCHEDULE_IDLE      0
#define MC_FLAG_SCHEDULE_NON_IDLE  1

struct mcp_buffer {
	struct mcp_flags flags;		
	union mcp_message message;	
};

#endif 
