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

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/delay.h>
#include "platform.h"		
#ifndef MC_NO_UIDGIT_H
#include <linux/uidgid.h>
#else 
#define kuid_t uid_t
static inline uid_t __kuid_val(kuid_t uid)
{
	return uid;
}
#endif 

#include "public/mc_user.h"
#include "public/mc_admin.h"

#include "mci/mcloadformat.h"

#include "main.h"
#include "mmu.h"	
#include "mcp.h"
#include "client.h"
#include "admin.h"

struct service {
	pid_t tgid;
	struct file *file;
	u32 role;
};

static struct admin_ctx {
	struct mutex services_mutex;	
	struct service services[2];
	int (*tee_start_cb)(void);
	void (*tee_stop_cb)(void);
	int last_start_ret;
} admin_ctx;

static struct mc_admin_driver_request {
	
	struct mutex mutex;		
	struct mutex states_mutex;	
	enum client_state {
		IDLE,
		REQUEST_SENT,
		BUFFERS_READY,
	} client_state;
	enum server_state {
		NOT_CONNECTED,		
		READY,			
		REQUEST_RECEIVED,	
		RESPONSE_SENT,		
		DATA_SENT,		
	} server_state;
	
	u32 request_id;
	struct mc_admin_request request;
	struct completion client_complete;
	
	struct mc_admin_response response;
	struct completion server_complete;
	void *buffer;			
	size_t size;			
} g_request;

static struct tee_object *tee_object_alloc(bool is_sp_trustlet, size_t length)
{
	struct tee_object *obj;
	size_t size = sizeof(*obj) + length;
	size_t header_length = 0;

	
	if (is_sp_trustlet) {
		
		header_length = sizeof(struct mc_blob_len_info);
		size += header_length + 3 * MAX_SO_CONT_SIZE;
	}

	
	obj = vzalloc(size);
	if (!obj)
		return NULL;

	
	obj->header_length = (u32)header_length;
	obj->length = (u32)length;
	return obj;
}

void tee_object_free(struct tee_object *robj)
{
	vfree(robj);
}

static inline void client_state_change(enum client_state state)
{
	mutex_lock(&g_request.states_mutex);
	g_request.client_state = state;
	mutex_unlock(&g_request.states_mutex);
}

static inline bool client_state_is(enum client_state state)
{
	bool is;

	mutex_lock(&g_request.states_mutex);
	is = g_request.client_state == state;
	mutex_unlock(&g_request.states_mutex);
	return is;
}

static inline void server_state_change(enum server_state state)
{
	mutex_lock(&g_request.states_mutex);
	g_request.server_state = state;
	mutex_unlock(&g_request.states_mutex);
}

static inline bool server_state_is(enum server_state state)
{
	bool is;

	mutex_lock(&g_request.states_mutex);
	is = g_request.server_state == state;
	mutex_unlock(&g_request.states_mutex);
	return is;
}

static void request_cancel(void);

static int request_send(u32 command, const struct mc_uuid_t *uuid, bool is_gp,
			u32 spid)
{
	int counter = 10;
	int ret = 0;

	
	mutex_lock(&g_request.states_mutex);
	
	while ((g_request.server_state == NOT_CONNECTED) && counter--) {
		mutex_unlock(&g_request.states_mutex);
		ssleep(1);
		mutex_lock(&g_request.states_mutex);
	}

	WARN_ON(g_request.client_state != IDLE);
	if (g_request.server_state != READY) {
		mutex_unlock(&g_request.states_mutex);
		if (g_request.server_state != NOT_CONNECTED) {
			mc_dev_err("invalid daemon state %d\n",
				   g_request.server_state);
			ret = -EPROTO;
			goto end;
		} else {
			mc_dev_err("daemon not connected\n");
			ret = -EHOSTUNREACH;
			goto end;
		}
	}

	memset(&g_request.request, 0, sizeof(g_request.request));
	memset(&g_request.response, 0, sizeof(g_request.response));
	g_request.request.request_id = g_request.request_id;
	g_request.request.command = command;
	if (uuid)
		memcpy(&g_request.request.uuid, uuid, sizeof(*uuid));
	else
		memset(&g_request.request.uuid, 0, sizeof(*uuid));

	g_request.request.is_gp = is_gp;
	g_request.request.spid = spid;
	g_request.client_state = REQUEST_SENT;
	mutex_unlock(&g_request.states_mutex);

	
	complete(&g_request.client_complete);

	
	wait_for_completion(&g_request.server_complete);

	
	mutex_lock(&g_request.states_mutex);
	switch (g_request.server_state) {
	case NOT_CONNECTED:
		
		ret = -EPIPE;
		break;
	case READY:
		
		ret = -g_request.response.error_no;
		break;
	case RESPONSE_SENT:
	case DATA_SENT:
		
		ret = 0;
		break;
	case REQUEST_RECEIVED:
		
		mc_dev_err("daemon is in a bad state: %d\n",
			   g_request.server_state);
		ret = -EPIPE;
		break;
	}

	mutex_unlock(&g_request.states_mutex);

end:
	if (ret)
		request_cancel();

	return ret;
}

static int request_receive(void *address, u32 size)
{

	
	bool server_ok;

	mutex_lock(&g_request.states_mutex);
	server_ok = (g_request.server_state == RESPONSE_SENT) ||
		    (g_request.server_state == DATA_SENT);
	mutex_unlock(&g_request.states_mutex);
	if (!server_ok) {
		mc_dev_err("expected server state %d or %d, not %d\n",
			   RESPONSE_SENT, DATA_SENT, g_request.server_state);
		request_cancel();
		return -EPIPE;
	}

	
	g_request.buffer = address;
	g_request.size = size;
	client_state_change(BUFFERS_READY);

	
	complete(&g_request.client_complete);

	
	wait_for_completion(&g_request.server_complete);

	
	g_request.buffer = NULL;
	g_request.size = 0;

	
	client_state_change(IDLE);
	return 0;
}

static void request_cancel(void)
{
	
	mutex_lock(&g_request.states_mutex);
	if (g_request.server_state == DATA_SENT)
		complete(&g_request.client_complete);

	
	g_request.client_state = IDLE;
	mutex_unlock(&g_request.states_mutex);
}

static int admin_get_root_container(void *address)
{
	int ret = 0;

	
	mutex_lock(&g_request.mutex);

	
	ret = request_send(MC_DRV_GET_ROOT_CONTAINER, 0, 0, 0);
	if (ret)
		goto end;

	
	if (g_request.response.length >= MAX_SO_CONT_SIZE) {
		request_cancel();
		mc_dev_err("response length exceeds maximum\n");
		ret = EREMOTEIO;
		goto end;
	}

	
	ret = request_receive(address, g_request.response.length);
	if (!ret)
		ret = g_request.response.length;

end:
	mutex_unlock(&g_request.mutex);
	return ret;
}

static int admin_get_sp_container(void *address, u32 spid)
{
	int ret = 0;

	
	mutex_lock(&g_request.mutex);

	
	ret = request_send(MC_DRV_GET_SP_CONTAINER, 0, 0, spid);
	if (ret)
		goto end;

	
	if (g_request.response.length >= MAX_SO_CONT_SIZE) {
		request_cancel();
		mc_dev_err("response length exceeds maximum\n");
		ret = EREMOTEIO;
		goto end;
	}

	
	ret = request_receive(address, g_request.response.length);
	if (!ret)
		ret = g_request.response.length;

end:
	mutex_unlock(&g_request.mutex);
	return ret;
}

static int admin_get_trustlet_container(void *address,
					const struct mc_uuid_t *uuid, u32 spid)
{
	int ret = 0;

	
	mutex_lock(&g_request.mutex);

	
	ret = request_send(MC_DRV_GET_TRUSTLET_CONTAINER, uuid, 0, spid);
	if (ret)
		goto end;

	
	if (g_request.response.length >= MAX_SO_CONT_SIZE) {
		request_cancel();
		mc_dev_err("response length exceeds maximum\n");
		ret = EREMOTEIO;
		goto end;
	}

	
	ret = request_receive(address, g_request.response.length);
	if (!ret)
		ret = g_request.response.length;

end:
	mutex_unlock(&g_request.mutex);
	return ret;
}

static struct tee_object *admin_get_trustlet(const struct mc_uuid_t *uuid,
					     bool is_gp, u32 *spid)
{
	struct tee_object *obj = NULL;
	bool is_sp_tl;
	int ret = 0;

	
	mutex_lock(&g_request.mutex);

	
	ret = request_send(MC_DRV_GET_TRUSTLET, uuid, is_gp, 0);
	if (ret)
		goto end;

	
	is_sp_tl = g_request.response.service_type == SERVICE_TYPE_SP_TRUSTLET;
	obj = tee_object_alloc(is_sp_tl, g_request.response.length);
	if (!obj) {
		request_cancel();
		ret = -ENOMEM;
		goto end;
	}

	
	ret = request_receive(&obj->data[obj->header_length], obj->length);
	*spid = g_request.response.spid;

end:
	mutex_unlock(&g_request.mutex);
	if (ret)
		return ERR_PTR(ret);

	return obj;
}

static void mc_admin_sendcrashdump(void)
{
	int ret = 0;

	
	mutex_lock(&g_request.mutex);

	
	ret = request_send(MC_DRV_SIGNAL_CRASH, NULL, false, 0);
	if (ret)
		goto end;

	
	request_cancel();

end:
	mutex_unlock(&g_request.mutex);
}

static int tee_object_make(u32 spid, struct tee_object *obj)
{
	struct mc_blob_len_info *l_info = (struct mc_blob_len_info *)obj->data;
	u8 *address = &obj->data[obj->header_length + obj->length];
	struct mclf_header_v2 *thdr;
	int ret;

	
	ret = admin_get_root_container(address);
	if (ret < 0)
		goto err;

	l_info->root_size = ret;
	address += ret;

	
	ret = admin_get_sp_container(address, spid);
	if (ret < 0)
		goto err;

	l_info->sp_size = ret;
	address += ret;

	
	thdr = (struct mclf_header_v2 *)&obj->data[obj->header_length];
	ret = admin_get_trustlet_container(address, &thdr->uuid, spid);
	if (ret < 0)
		goto err;

	l_info->ta_size = ret;
	address += ret;

	
	l_info->magic = MC_TLBLOBLEN_MAGIC;
	obj->length += sizeof(*l_info);
	obj->length += l_info->root_size + l_info->sp_size + l_info->ta_size;
	ret = 0;

err:
	return ret;
}

struct tee_object *tee_object_read(u32 spid, uintptr_t address, size_t length)
{
	char __user *addr = (char __user *)address;
	struct tee_object *obj;
	u8 *data;
	struct mclf_header_v2 thdr;
	int ret;

	
	if (length < sizeof(thdr)) {
		mc_dev_err("buffer shorter than header size\n");
		return ERR_PTR(-EFAULT);
	}

	
	if (copy_from_user(&thdr, addr, sizeof(thdr))) {
		mc_dev_err("header: copy_from_user failed\n");
		return ERR_PTR(-EFAULT);
	}

	
	obj = tee_object_alloc(thdr.service_type == SERVICE_TYPE_SP_TRUSTLET,
			       length);
	if (!obj)
		return ERR_PTR(-ENOMEM);

	
	data = &obj->data[obj->header_length];
	memcpy(data, &thdr, sizeof(thdr));
	
	data += sizeof(thdr);
	if (copy_from_user(data, &addr[sizeof(thdr)], length - sizeof(thdr))) {
		mc_dev_err("data: copy_from_user failed\n");
		vfree(obj);
		return ERR_PTR(-EFAULT);
	}

	if (obj->header_length) {
		ret = tee_object_make(spid, obj);
		if (ret) {
			vfree(obj);
			return ERR_PTR(ret);
		}
	}

	return obj;
}

struct tee_object *tee_object_select(const struct mc_uuid_t *uuid)
{
	struct tee_object *obj;
	struct mclf_header_v2 *thdr;

	obj = tee_object_alloc(false, sizeof(*thdr));
	if (!obj)
		return ERR_PTR(-ENOMEM);

	thdr = (struct mclf_header_v2 *)&obj->data[obj->header_length];
	memcpy(&thdr->uuid, uuid, sizeof(thdr->uuid));
	return obj;
}

struct tee_object *tee_object_get(const struct mc_uuid_t *uuid, bool is_gp)
{
	struct tee_object *obj;
	u32 spid = 0;

	
	obj = admin_get_trustlet(uuid, is_gp, &spid);
	if (IS_ERR(obj))
		return obj;

	
	if (obj->header_length) {
		int ret;

		
		if (!spid) {
			vfree(obj);
			return ERR_PTR(-ENOENT);
		}

		ret = tee_object_make(spid, obj);
		if (ret) {
			vfree(obj);
			return ERR_PTR(ret);
		}
	}

	return obj;
}

static inline int load_driver(struct tee_client *client,
			      struct mc_admin_load_info *info)
{
	struct tee_object *obj;
	struct mclf_header_v2 *thdr;
	struct mc_identity identity = {
		.login_type = LOGIN_PUBLIC,
	};
	uintptr_t dci = 0;
	u32 dci_len = 0;
	u32 sid;
	int ret;

	obj = tee_object_read(info->spid, info->address, info->length);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	thdr = (struct mclf_header_v2 *)&obj->data[obj->header_length];
	if (!(thdr->flags & MC_SERVICE_HEADER_FLAGS_NO_CONTROL_INTERFACE)) {
		dci_len = PAGE_SIZE;
		ret = client_cbuf_create(client, dci_len, &dci, NULL);
		if (ret)
			goto end;
	}

	
	ret = client_add_session(client, obj, dci, dci_len, &sid, false,
				 &identity);
	if (!ret)
		mc_dev_devel("driver loaded with sid %x", sid);

	client_cbuf_free(client, dci);
end:
	vfree(obj);
	return ret;
}

static inline int load_token(struct mc_admin_load_info *token)
{
	struct tee_mmu *mmu;
	struct mcp_buffer_map map;
	int ret;

	mmu = tee_mmu_create(current, (void *)(uintptr_t)token->address,
			     token->length);
	if (IS_ERR(mmu))
		return PTR_ERR(mmu);

	tee_mmu_buffer(mmu, &map);
	ret = mcp_load_token(token->address, &map);
	tee_mmu_delete(mmu);
	return ret;
}

static inline int load_check(struct mc_admin_load_info *info)
{
	struct tee_object *obj;
	struct tee_mmu *mmu;
	struct mcp_buffer_map map;
	int ret;

	obj = tee_object_read(info->spid, info->address, info->length);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	mmu = tee_mmu_create(NULL, obj->data, obj->length);
	if (IS_ERR(mmu))
		return PTR_ERR(mmu);

	tee_mmu_buffer(mmu, &map);
	ret = mcp_load_check(obj, &map);
	tee_mmu_delete(mmu);
	return ret;
}

static ssize_t admin_write(struct file *file, const char __user *user,
			   size_t len, loff_t *off)
{
	int ret;

	
	if (*off) {
		mc_dev_err("offset not supported\n");
		g_request.response.error_no = EPIPE;
		ret = -ECOMM;
		goto err;
	}

	if (server_state_is(REQUEST_RECEIVED)) {
		
		if (!client_state_is(REQUEST_SENT)) {
			mc_dev_err("expected client state %d, not %d\n",
				   REQUEST_SENT, g_request.client_state);
			g_request.response.error_no = EPIPE;
			ret = -EPIPE;
			goto err;
		}

		
		if (copy_from_user(&g_request.response, user,
				   sizeof(g_request.response))) {
			mc_dev_err("failed to get response from daemon\n");
			g_request.response.error_no = EPIPE;
			ret = -ECOMM;
			goto err;
		}

		
		if (g_request.request.request_id !=
						g_request.response.request_id) {
			mc_dev_err("expected id %d, not %d\n",
				   g_request.request.request_id,
				   g_request.response.request_id);
			g_request.response.error_no = EPIPE;
			ret = -EBADE;
			goto err;
		}

		
		ret = sizeof(g_request.response);
		if (g_request.response.length)
			server_state_change(RESPONSE_SENT);
		else
			server_state_change(READY);

		goto end;
	} else if (server_state_is(RESPONSE_SENT)) {
		
		server_state_change(DATA_SENT);

		
		ret = wait_for_completion_interruptible(
						&g_request.client_complete);

		
		if (ret) {
			server_state_change(RESPONSE_SENT);
			return ret;
		}

		
		if (!client_state_is(BUFFERS_READY)) {
			mc_dev_err("expected client state %d, not %d\n",
				   BUFFERS_READY, g_request.client_state);
			g_request.response.error_no = EPIPE;
			ret = -EPIPE;
			goto err;
		}

		
		if (len != g_request.size)
			len = g_request.size;

		ret = copy_from_user(g_request.buffer, user, len);
		if (ret) {
			mc_dev_err("failed to get data from daemon\n");
			g_request.response.error_no = EPIPE;
			ret = -ECOMM;
			goto err;
		}

		ret = len;
		server_state_change(READY);
		goto end;
	} else {
		ret = -ECOMM;
		goto err;
	}

err:
	server_state_change(READY);
end:
	complete(&g_request.server_complete);
	return ret;
}

int is_authenticator_pid(pid_t pid)
{
	struct service *service = NULL;
	struct task_struct *task;
	int ret = 0;

	
	mutex_lock(&admin_ctx.services_mutex);
	if (admin_ctx.services[0].role == TEE_ROLE_AUTHENTICATOR)
		service = &admin_ctx.services[0];
	else if (admin_ctx.services[1].role == TEE_ROLE_AUTHENTICATOR)
		service = &admin_ctx.services[1];

	
	if (!service) {
		mc_dev_err("No authenticator connected\n");
		return -ENOTCONN;
	}

	
	rcu_read_lock();
	task = pid_task(find_vpid(pid), PIDTYPE_PID);
	if (!task) {
		mc_dev_err("No task for PID %d\n", pid);
		ret = -EINVAL;
	} else if (task->tgid != service->tgid) {
		mc_dev_err("PID %d is not an authenticator\n", pid);
		ret = -EPERM;
	}
	rcu_read_unlock();
	mutex_unlock(&admin_ctx.services_mutex);
	return ret;
}

static long admin_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	int ret = -EINVAL;

	mc_dev_devel("%u from %s\n", _IOC_NR(cmd), current->comm);

	switch (cmd) {
	case MC_ADMIN_IO_GET_DRIVER_REQUEST: {
		struct service *service = NULL;

		
		mutex_lock(&admin_ctx.services_mutex);
		if (file == admin_ctx.services[0].file) {
			if (admin_ctx.services[0].role == TEE_ROLE_LISTENER)
				service = &admin_ctx.services[0];
		} else {
			if (admin_ctx.services[1].role == TEE_ROLE_LISTENER)
				service = &admin_ctx.services[1];
		}

		if (service) {
			
			if (service->tgid != current->tgid) {
				mc_dev_info("admin TGID changed %d -> %d\n",
					    service->tgid, current->tgid);
				service->tgid = current->tgid;
			}
			ret = 0;
		} else {
			mc_dev_err("admin TGID %d is not a listener\n",
				   current->tgid);
			ret = -EPERM;
		}
		mutex_unlock(&admin_ctx.services_mutex);
		if (ret)
			break;

		
		ret = wait_for_completion_interruptible(
						&g_request.client_complete);
		if (ret)
			
			break;

		
		if (!client_state_is(REQUEST_SENT)) {
			mc_dev_err("expected client state %d, not %d\n",
				   REQUEST_SENT, g_request.client_state);
			g_request.response.error_no = EPIPE;
			complete(&g_request.server_complete);
			ret = -EPIPE;
			break;
		}

		
		ret = copy_to_user(uarg, &g_request.request,
				   sizeof(g_request.request));
		if (ret) {
			server_state_change(READY);
			complete(&g_request.server_complete);
			ret = -EPROTO;
			break;
		}

		
		g_request.request_id++;

		server_state_change(REQUEST_RECEIVED);
		break;
	}
	case MC_ADMIN_IO_GET_INFO: {
		struct mc_admin_driver_info info;

		info.drv_version = MC_VERSION(MCDRVMODULEAPI_VERSION_MAJOR,
					      MCDRVMODULEAPI_VERSION_MINOR);
		info.initial_cmd_id = g_request.request_id;
		ret = copy_to_user(uarg, &info, sizeof(info));
		break;
	}
	case MC_ADMIN_IO_LOAD_DRIVER: {
		struct tee_client *client = file->private_data;
		struct mc_admin_load_info info;

		if (copy_from_user(&info, uarg, sizeof(info))) {
			ret = -EFAULT;
			break;
		}

		
		if (!client) {
			client = client_create(true);
			
			file->private_data = client;
		}

		if (!client) {
			ret = -ENOMEM;
			break;
		}

		ret = load_driver(client, &info);
		break;
	}
	case MC_ADMIN_IO_LOAD_TOKEN: {
		struct mc_admin_load_info info;

		if (copy_from_user(&info, uarg, sizeof(info))) {
			ret = -EFAULT;
			break;
		}

		ret = load_token(&info);
		break;
	}
	case MC_ADMIN_IO_LOAD_CHECK: {
		struct mc_admin_load_info info;

		if (copy_from_user(&info, uarg, sizeof(info))) {
			ret = -EFAULT;
			break;
		}

		ret = load_check(&info);
		break;
	}
	case MC_ADMIN_IO_REQUEST_ROLE: {
		struct service *service;
		u32 role, other_role;

		ret = 0;
		if (copy_from_user(&role, uarg, sizeof(role))) {
			ret = -EFAULT;
			break;
		}

		if ((role != TEE_ROLE_LISTENER) &&
		    (role != TEE_ROLE_AUTHENTICATOR)) {
			ret = -EINVAL;
			break;
		}

		mutex_lock(&admin_ctx.services_mutex);
		if (file == admin_ctx.services[0].file) {
			service = &admin_ctx.services[0];
			other_role = admin_ctx.services[1].role;
		} else {
			service = &admin_ctx.services[1];
			other_role = admin_ctx.services[0].role;
		}

		if ((service->role != TEE_ROLE_NONE) || (role == other_role))
			ret = -EBUSY;

		if (!ret) {
			service->role = role;
			mc_dev_devel("TGID %d has taken role %d\n",
				     current->tgid, service->role);
		} else {
			mc_dev_err("TGID %d failed to take role %d: ret %d\n",
				   current->tgid, role, ret);
		}

		if (service->role == TEE_ROLE_LISTENER) {
			
			g_request.request_id = 42;
			server_state_change(READY);
		}

		mutex_unlock(&admin_ctx.services_mutex);
		break;
	}
	default:
		ret = -ENOIOCTLCMD;
	}

	return ret;
}

static int admin_release(struct inode *inode, struct file *file)
{
	struct service *service;

	
	if (file->private_data)
		client_close((struct tee_client *)file->private_data);

	
	mutex_lock(&admin_ctx.services_mutex);
	if (file == admin_ctx.services[0].file)
		service = &admin_ctx.services[0];
	else
		service = &admin_ctx.services[1];
	mutex_unlock(&admin_ctx.services_mutex);

	if (service->role == TEE_ROLE_LISTENER) {
		
		mutex_lock(&g_request.states_mutex);
		g_request.server_state = NOT_CONNECTED;
		
		if (g_request.client_state != IDLE) {
			g_request.response.error_no = ESHUTDOWN;
			complete(&g_request.server_complete);
		}
		mutex_unlock(&g_request.states_mutex);
	}

	mc_dev_info("admin connection closed, TGID %d\n", service->tgid);
	mutex_lock(&admin_ctx.services_mutex);
	memset(service, 0, sizeof(*service));
	mutex_unlock(&admin_ctx.services_mutex);
	return 0;
}

static int admin_open(struct inode *inode, struct file *file)
{
	struct service *service = NULL;
	int ret = 0;

	
	mutex_lock(&admin_ctx.services_mutex);
	if (!admin_ctx.services[0].tgid) {
		service = &admin_ctx.services[0];
		mc_dev_devel("admin connection #0, TGID %d\n", current->tgid);
	} else if (!admin_ctx.services[1].tgid) {
		service = &admin_ctx.services[1];
		mc_dev_devel("admin connection #1, TGID %d\n", current->tgid);
	} else {
		mc_dev_err("both admin connections already open\n");
		ret = -EBUSY;
	}

	if (service) {
		service->tgid = current->tgid;
		service->file = file;
	}
	mutex_unlock(&admin_ctx.services_mutex);
	if (ret)
		return ret;

	
	mc_dev_devel("accept %s as TEE admin\n", current->comm);

	mutex_lock(&admin_ctx.services_mutex);
	if (admin_ctx.last_start_ret > 0)
		admin_ctx.last_start_ret = admin_ctx.tee_start_cb();

	
	if (admin_ctx.last_start_ret) {
		memset(service, 0, sizeof(*service));
		ret = admin_ctx.last_start_ret;
	}
	mutex_unlock(&admin_ctx.services_mutex);
	if (ret)
		return ret;

	
	mc_dev_info("admin connection open, TGID %d\n", service->tgid);
	return 0;
}

static const struct file_operations mc_admin_fops = {
	.owner = THIS_MODULE,
	.open = admin_open,
	.release = admin_release,
	.unlocked_ioctl = admin_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = admin_ioctl,
#endif
	.write = admin_write,
};

int mc_admin_init(struct cdev *cdev, int (*tee_start_cb)(void),
		  void (*tee_stop_cb)(void))
{
	mutex_init(&admin_ctx.services_mutex);
	
	mutex_init(&g_request.mutex);
	mutex_init(&g_request.states_mutex);
	init_completion(&g_request.client_complete);
	init_completion(&g_request.server_complete);
	mcp_register_crashhandler(mc_admin_sendcrashdump);
	
	cdev_init(cdev, &mc_admin_fops);
	
	admin_ctx.tee_start_cb = tee_start_cb;
	admin_ctx.tee_stop_cb = tee_stop_cb;
	admin_ctx.last_start_ret = 1;
	return 0;
}

void mc_admin_exit(void)
{
	if (!admin_ctx.last_start_ret)
		admin_ctx.tee_stop_cb();
}
