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

#ifndef __DCITUI_H__
#define __DCITUI_H__

#define RSP_ID_MASK (1U << 31)
#define RSP_ID(cmd_id) (((uint32_t)(cmd_id)) | RSP_ID_MASK)
#define IS_CMD(cmd_id) ((((uint32_t)(cmd_id)) & RSP_ID_MASK) == 0)
#define IS_RSP(cmd_id) ((((uint32_t)(cmd_id)) & RSP_ID_MASK) == RSP_ID_MASK)
#define CMD_ID_FROM_RSP(rsp_id) (rsp_id & (~RSP_ID_MASK))

#define TUI_DCI_OK                      0x00030000
#define TUI_DCI_ERR_UNKNOWN_CMD         0x00030001
#define TUI_DCI_ERR_NOT_SUPPORTED       0x00030002
#define TUI_DCI_ERR_INTERNAL_ERROR      0x00030003
#define TUI_DCI_ERR_NO_RESPONSE         0x00030004
#define TUI_DCI_ERR_BAD_PARAMETERS      0x00030005
#define TUI_DCI_ERR_NO_EVENT            0x00030006
#define TUI_DCI_ERR_OUT_OF_DISPLAY      0x00030007

#define NOT_TUI_NONE                0
#define NOT_TUI_CANCEL_EVENT        1
#define NOT_TUI_HAL_TOUCH_EVENT     0x80000001

#define CMD_TUI_SW_NONE             0
#define CMD_TUI_SW_OPEN_SESSION     1
#define CMD_TUI_SW_CLOSE_SESSION    2
#define CMD_TUI_SW_STOP_DISPLAY     3
#define CMD_TUI_SW_GET_VERSION      4
#define CMD_TUI_SW_HAL              5

#define CMD_TUI_HAL_NONE                    0
#define CMD_TUI_HAL_QUEUE_BUFFER            1
#define CMD_TUI_HAL_QUEUE_DEQUEUE_BUFFER    2
#define CMD_TUI_HAL_CLEAR_TOUCH_INTERRUPT   3
#define CMD_TUI_HAL_HIDE_SURFACE            4
#define CMD_TUI_HAL_GET_RESOLUTION          5

#define MAX_DCI_DATA_LEN (1024*100)

#define TUI_DCI_VERSION_MAJOR   (1u)
#define TUI_DCI_VERSION_MINOR   (1u)

#define TUI_DCI_VERSION(major, minor) \
	(((major & 0x0000ffff) << 16) | (minor & 0x0000ffff))
#define TUI_DCI_VERSION_GET_MAJOR(version) ((version >> 16) & 0x0000ffff)
#define TUI_DCI_VERSION_GET_MINOR(version) (version & 0x0000ffff)


struct tui_disp_data_t {
	uint32_t buff_id;
};

struct tui_hal_cmd_t {
	uint32_t id;    
	uint32_t size;  
	uint64_t data[2];   
};

struct tui_hal_rsp_t {
	uint32_t id;    
	uint32_t return_code;   
	uint32_t size;  
	uint32_t data[3];   
};

struct tui_alloc_data_t {
	uint32_t alloc_size;
	uint32_t num_of_buff;
};

union dci_cmd_payload_t {
	struct tui_alloc_data_t alloc_data;
	struct tui_disp_data_t  disp_data;
	struct tui_hal_cmd_t    hal;
};

struct dci_command_t {
	uint32_t id;
	union dci_cmd_payload_t payload;
};

struct tui_alloc_buffer_t {
	uint64_t    pa;
};

#define MAX_DCI_BUFFER_NUMBER 4

struct dci_response_t {
	uint32_t	id; 
	uint32_t		return_code;
	union {
		struct tui_alloc_buffer_t alloc_buffer[MAX_DCI_BUFFER_NUMBER];
		struct tui_hal_rsp_t hal_rsp;
	};
};

struct tui_dci_msg_t {
	uint32_t version;
	uint32_t     nwd_notif; 
	struct dci_command_t  cmd_nwd;   
	struct dci_response_t nwd_rsp;   
	uint32_t     hal_cmd;
	uint32_t     hal_rsp;
};

#define DR_TUI_UUID { { 7, 0xC, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } }

#endif 
