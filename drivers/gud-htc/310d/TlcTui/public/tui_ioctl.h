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

#ifndef TUI_IOCTL_H_
#define TUI_IOCTL_H_

#define MAX_BUFFER_NUMBER 3

struct tlc_tui_command_t {
	uint32_t     id;
	uint32_t     data[2];
};

struct tlc_tui_response_t {
	uint32_t	id;
	uint32_t	return_code;
	int		ion_fd[MAX_BUFFER_NUMBER];
	uint32_t	screen_metrics[3];
};

#define TLC_TUI_CMD_NONE                0
#define TLC_TUI_CMD_START_ACTIVITY      1
#define TLC_TUI_CMD_STOP_ACTIVITY       2
#define TLC_TUI_CMD_QUEUE               3
#define TLC_TUI_CMD_QUEUE_DEQUEUE       4
#define TLC_TUI_CMD_ALLOC_FB            5
#define TLC_TUI_CMD_FREE_FB             6
#define TLC_TUI_CMD_HIDE_SURFACE        7
#define TLC_TUI_CMD_GET_RESOLUTION      8

#define TLC_TUI_OK                  0
#define TLC_TUI_ERROR               1
#define TLC_TUI_ERR_UNKNOWN_CMD     2

#define TUI_DEV_NAME	"t-base-tui"

#define TUI_IO_MAGIC	't'

#define TUI_IO_NOTIFY	_IOW(TUI_IO_MAGIC, 1, uint32_t)
#define TUI_IO_WAITCMD	_IOR(TUI_IO_MAGIC, 2, struct tlc_tui_command_t)
#define TUI_IO_ACK	_IOW(TUI_IO_MAGIC, 3, struct tlc_tui_response_t)
#define TUI_IO_INIT_DRIVER	_IO(TUI_IO_MAGIC, 4)

#ifdef INIT_COMPLETION
#define reinit_completion(x) INIT_COMPLETION(*(x))
#endif

#endif 
