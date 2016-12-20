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

#ifndef _MC_MCP_H_
#define _MC_MCP_H_

#include "mci/mcloadformat.h"		

struct tee_object {
	u32	length;		
	u32	header_length;	
	u8	data[];		
};

struct mcp_buffer_map {
	u64	phys_addr;	
	u64	secure_va;	
	u32	offset;		
	u32	length;		
	u32	type;		
};

struct mcp_session {
	
	struct work_struct	close_work;
	
	struct list_head	list;
	
	struct list_head	notifications_list;
	
	struct mutex		notif_wait_lock;	
	
	enum mcp_notification_state {
		MCP_NOTIF_IDLE,		
		MCP_NOTIF_QUEUED,	
		MCP_NOTIF_SENT,		
		MCP_NOTIF_RECEIVED,	
		MCP_NOTIF_CONSUMED,	
		MCP_NOTIF_DEAD,		
	}			notif_state;
	
	struct completion	completion;
	
	struct mutex		exit_code_lock;
	
	s32			exit_code;
	
	u32		id;
	
	enum mcp_session_state {
		MCP_SESSION_RUNNING,
		MCP_SESSION_CLOSE_FAILED,
		MCP_SESSION_CLOSE_REQUESTED,
		MCP_SESSION_CLOSE_NOTIFIED,
		MCP_SESSION_CLOSING_GP,
		MCP_SESSION_CLOSED,
	}			state;
	
	bool			is_gp;
	
	struct identity		identity;
};

void mcp_session_init(struct mcp_session *session, bool is_gp,
		      const struct identity *identity);
int mcp_session_waitnotif(struct mcp_session *session, s32 timeout,
			  bool silent_expiry);
s32 mcp_session_exitcode(struct mcp_session *mcp_session);

int mcp_suspend(void);
int mcp_resume(void);
bool mcp_suspended(void);

enum mcp_scheduler_commands {
	MCP_YIELD,
	MCP_NSIQ,
};

void mcp_register_scheduler(int (*scheduler_cb)(enum mcp_scheduler_commands));
bool mcp_notifications_flush(void);
void mcp_register_crashhandler(void (*crashhandler_cb)(void));

bool mcp_get_idle_timeout(s32 *timeout);
void mcp_reset_idle_timeout(void);
void mcp_update_time(void);

int mcp_get_version(struct mc_version_info *version_info);
int mcp_load_token(uintptr_t data, const struct mcp_buffer_map *buffer_map);
int mcp_load_check(const struct tee_object *obj,
		   const struct mcp_buffer_map *buffer_map);
int mcp_open_session(struct mcp_session *session,
		     const struct tee_object *obj,
		     const struct mcp_buffer_map *map,
		     const struct mcp_buffer_map *tci_map);
int mcp_close_session(struct mcp_session *session);
void mcp_kill_session(struct mcp_session *session);
int mcp_map(u32 session_id, struct mcp_buffer_map *buffer_map);
int mcp_unmap(u32 session_id, const struct mcp_buffer_map *buffer_map);
int mcp_multimap(u32 session_id, struct mcp_buffer_map *buffer_maps);
int mcp_multiunmap(u32 session_id, const struct mcp_buffer_map *buffer_maps);
int mcp_notify(struct mcp_session *mcp_session);

int mcp_init(void);
void mcp_exit(void);
int mcp_start(void);
void mcp_stop(void);

int mcp_debug_sessions(struct kasnprintf_buf *buf);
int mcp_debug_mcpcmds(struct kasnprintf_buf *buf);

#endif 
