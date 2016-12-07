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
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/err.h>

#include "public/mc_user.h"
#include "public/mc_admin.h"

#include "main.h"
#include "admin.h"	
#include "mcp.h"
#include "mmu.h"
#include "session.h"
#include "client.h"

static struct client_ctx {
	
	struct mutex		clients_lock;
	struct list_head	clients;
	
	struct mutex		closing_clients_lock;
	struct list_head	closing_clients;
} client_ctx;

struct cbuf {
	
	struct tee_client	*client;
	
	struct list_head	list;
	
	struct kref		kref;
	
	uintptr_t		addr;
	
	uintptr_t		uaddr;
	
	phys_addr_t		phys;
	
	unsigned int		order;
	
	u32			len;
	
	bool			api_freed;
};

static inline void cbuf_get(struct cbuf *cbuf)
{
	kref_get(&cbuf->kref);
}

static void cbuf_release(struct kref *kref)
{
	struct cbuf *cbuf = container_of(kref, struct cbuf, kref);
	struct tee_client *client = cbuf->client;

	
	mutex_lock(&client->cbufs_lock);
	list_del_init(&cbuf->list);
	mutex_unlock(&client->cbufs_lock);
	
	client_put(client);
	
	free_pages(cbuf->addr, cbuf->order);
	mc_dev_devel("freed cbuf %p: client %p addr %lx uaddr %lx len %u\n",
		     cbuf, client, cbuf->addr, cbuf->uaddr, cbuf->len);
	kfree(cbuf);
	
	atomic_dec(&g_ctx.c_cbufs);
}

static inline void cbuf_put(struct cbuf *cbuf)
{
	kref_put(&cbuf->kref, cbuf_release);
}

static int cbuf_map(struct vm_area_struct *vmarea, uintptr_t addr, u32 len,
		    uintptr_t *uaddr)
{
	int ret;

	if (WARN(!uaddr, "No uaddr pointer available"))
		return -EINVAL;

	if (WARN(!vmarea, "No vma available"))
		return -EINVAL;

	if (WARN(!addr, "No addr available"))
		return -EINVAL;

	if (len != (u32)(vmarea->vm_end - vmarea->vm_start)) {
		mc_dev_err("cbuf incompatible with vma\n");
		return -EINVAL;
	}

	vmarea->vm_flags |= VM_IO;
	ret = remap_pfn_range(vmarea, vmarea->vm_start,
			      page_to_pfn(virt_to_page(addr)),
			      vmarea->vm_end - vmarea->vm_start,
			      vmarea->vm_page_prot);
	if (ret) {
		*uaddr = 0;
		mc_dev_err("User mapping failed\n");
		return ret;
	}

	*uaddr = vmarea->vm_start;
	return 0;
}

struct tee_client *client_create(bool is_from_kernel)
{
	struct tee_client *client;

	
	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return NULL;

	
	atomic_inc(&g_ctx.c_clients);
	
	client->pid = is_from_kernel ? 0 : current->pid;
	memcpy(client->comm, current->comm, sizeof(client->comm));
	kref_init(&client->kref);
	INIT_LIST_HEAD(&client->cbufs);
	mutex_init(&client->cbufs_lock);
	INIT_LIST_HEAD(&client->sessions);
	INIT_LIST_HEAD(&client->closing_sessions);
	mutex_init(&client->sessions_lock);
	INIT_LIST_HEAD(&client->list);
	
	mutex_lock(&client_ctx.clients_lock);
	list_add_tail(&client->list, &client_ctx.clients);
	mutex_unlock(&client_ctx.clients_lock);
	mc_dev_devel("created client %p\n", client);
	return client;
}

static void client_release(struct kref *kref)
{
	struct tee_client *client;

	client = container_of(kref, struct tee_client, kref);
	
	mutex_lock(&client_ctx.closing_clients_lock);
	list_del(&client->list);
	mutex_unlock(&client_ctx.closing_clients_lock);
	mc_dev_devel("freed client %p\n", client);
	kfree(client);
	
	atomic_dec(&g_ctx.c_clients);
}

void client_put(struct tee_client *client)
{
	kref_put(&client->kref, client_release);
}

static inline bool client_is_kernel(struct tee_client *client)
{
	return !client->pid;
}

bool client_has_sessions(struct tee_client *client)
{
	bool ret;

	
	mutex_lock(&client->sessions_lock);
	ret = !list_empty(&client->sessions);
	mutex_unlock(&client->sessions_lock);
	mc_dev_devel("client %p, exit with %d\n", client, ret);
	return ret;
}

static void client_close_sessions(struct tee_client *client)
{
	struct tee_session *session;

	mutex_lock(&client->sessions_lock);
	while (!list_empty(&client->sessions)) {
		session = list_first_entry(&client->sessions,
					   struct tee_session, list);

		
		list_move(&session->list, &client->closing_sessions);
		
		mutex_unlock(&client->sessions_lock);
		session_close(session);
		mutex_lock(&client->sessions_lock);
	}

	mutex_unlock(&client->sessions_lock);
}

static void client_close_kernel_cbufs(struct tee_client *client)
{
	
	if (!client_is_kernel(client))
		return;

	
	while (true) {
		struct cbuf *cbuf = NULL, *candidate;

		mutex_lock(&client->cbufs_lock);
		list_for_each_entry(candidate, &client->cbufs, list) {
			if (!candidate->api_freed) {
				candidate->api_freed = true;
				cbuf = candidate;
				break;
			}
		}
		mutex_unlock(&client->cbufs_lock);

		if (!cbuf)
			break;

		cbuf_put(cbuf);
	}
}

void client_close(struct tee_client *client)
{
	
	mutex_lock(&client_ctx.clients_lock);
	mutex_lock(&client_ctx.closing_clients_lock);
	list_move(&client->list, &client_ctx.closing_clients);
	mutex_unlock(&client_ctx.closing_clients_lock);
	mutex_unlock(&client_ctx.clients_lock);
	client_close_kernel_cbufs(client);
	
	client_close_sessions(client);
	client_put(client);
	mc_dev_devel("client %p closed\n", client);
}

void clients_kill_sessions(void)
{
	struct tee_client *client;

	mutex_lock(&client_ctx.clients_lock);
	list_for_each_entry(client, &client_ctx.clients, list) {
		mutex_lock(&client->sessions_lock);
		while (!list_empty(&client->sessions)) {
			struct tee_session *session;

			session = list_first_entry(&client->sessions,
						   struct tee_session, list);
			list_del(&session->list);
			session_kill(session);
		}
		mutex_unlock(&client->sessions_lock);
	}
	mutex_unlock(&client_ctx.clients_lock);
}

int client_open_session(struct tee_client *client, u32 *session_id,
			const struct mc_uuid_t *uuid, uintptr_t	 tci,
			size_t tci_len, bool is_gp_uuid,
			struct mc_identity *identity)
{
	int err = 0;
	u32 sid = 0;
	struct tee_object *obj;

	
	obj = tee_object_get(uuid, is_gp_uuid);
	if (IS_ERR(obj)) {
		
		if ((PTR_ERR(obj) == -ENOENT) && g_ctx.f_ta_auth)
			obj = tee_object_select(uuid);

		if (IS_ERR(obj)) {
			err = PTR_ERR(obj);
			goto end;
		}
	}

	
	err = client_add_session(client, obj, tci, tci_len, &sid, is_gp_uuid,
				 identity);
	
	if (!err)
		*session_id = sid;

	
	tee_object_free(obj);

end:

	mc_dev_devel("session %x, exit with %d\n", sid, err);
	return err;
}

int client_open_trustlet(struct tee_client *client, u32 *session_id, u32 spid,
			 uintptr_t trustlet, size_t trustlet_len,
			 uintptr_t tci, size_t tci_len)
{
	struct tee_object *obj;
	struct mc_identity identity = {
		.login_type = LOGIN_PUBLIC,
	};
	u32 sid = 0;
	int err = 0;

	
	obj = tee_object_read(spid, trustlet, trustlet_len);
	if (IS_ERR(obj)) {
		err = PTR_ERR(obj);
		goto end;
	}

	
	err = client_add_session(client, obj, tci, tci_len, &sid, false,
				 &identity);
	
	if (!err)
		*session_id = sid;

	
	tee_object_free(obj);

end:
	mc_dev_devel("session %x, exit with %d\n", sid, err);
	return err;
}

int client_add_session(struct tee_client *client, const struct tee_object *obj,
		       uintptr_t tci, size_t len, u32 *session_id, bool is_gp,
		       struct mc_identity *identity)
{
	struct tee_session *session = NULL;
	struct tee_mmu *obj_mmu = NULL;
	int ret = 0;

	session = session_create(client, is_gp, identity);
	if (IS_ERR(session))
		return PTR_ERR(session);

	
	obj_mmu = tee_mmu_create(NULL, obj->data, obj->length);
	if (IS_ERR(obj_mmu)) {
		ret = PTR_ERR(obj_mmu);
		goto err;
	}

	
	ret = session_open(session, obj, obj_mmu, tci, len);
	
	tee_mmu_delete(obj_mmu);
	if (ret)
		goto err;

	mutex_lock(&client->sessions_lock);
	
	list_add_tail(&session->list, &client->sessions);
	
	*session_id = session->mcp_session.id;
	mutex_unlock(&client->sessions_lock);

err:
	
	if (ret == -ENODEV) {
		
		list_add_tail(&session->list, &client->closing_sessions);
		session_close(session);
	} else if (ret) {
		session_put(session);
	}

	return ret;
}

int client_remove_session(struct tee_client *client, u32 session_id)
{
	struct tee_session *session = NULL, *candidate;

	
	mutex_lock(&client->sessions_lock);
	list_for_each_entry(candidate, &client->sessions, list) {
		if (candidate->mcp_session.id == session_id) {
			session = candidate;
			list_move(&session->list, &client->closing_sessions);
			break;
		}
	}

	mutex_unlock(&client->sessions_lock);
	if (!session)
		return -ENXIO;

	
	return session_close(session);
}

static struct tee_session *client_get_session(struct tee_client *client,
					      u32 session_id)
{
	struct tee_session *session = NULL, *candidate;

	mutex_lock(&client->sessions_lock);
	list_for_each_entry(candidate, &client->sessions, list) {
		if (candidate->mcp_session.id == session_id) {
			session = candidate;
			session_get(session);
			break;
		}
	}

	mutex_unlock(&client->sessions_lock);
	if (!session)
		mc_dev_err("session %x not found\n", session_id);

	return session;
}

int client_notify_session(struct tee_client *client, u32 session_id)
{
	struct tee_session *session;
	int ret;

	
	session = client_get_session(client, session_id);
	if (!session)
		return -ENXIO;

	
	ret = session_notify_swd(session);
	
	session_put(session);
	mc_dev_devel("session %x, exit with %d\n", session_id, ret);
	return ret;
}

int client_waitnotif_session(struct tee_client *client, u32 session_id,
			     s32 timeout, bool silent_expiry)
{
	struct tee_session *session;
	int ret;

	
	session = client_get_session(client, session_id);
	if (!session)
		return -ENXIO;

	ret = session_waitnotif(session, timeout, silent_expiry);
	
	session_put(session);
	mc_dev_devel("session %x, exit with %d\n", session_id, ret);
	return ret;
}

int client_get_session_exitcode(struct tee_client *client, u32 session_id,
				s32 *exit_code)
{
	struct tee_session *session;

	
	session = client_get_session(client, session_id);
	if (!session)
		return -ENXIO;

	
	*exit_code = session_exitcode(session);
	
	session_put(session);
	mc_dev_devel("session %x, exit code %d\n", session_id, *exit_code);
	return 0;
}

int client_map_session_wsms(struct tee_client *client, u32 session_id,
			    struct mc_ioctl_buffer *bufs)
{
	struct tee_session *session;
	int ret;

	
	session = client_get_session(client, session_id);
	if (!session)
		return -ENXIO;

	
	ret = session_wsms_add(session, bufs);
	
	session_put(session);
	mc_dev_devel("session %x, exit with %d\n", session_id, ret);
	return ret;
}

int client_unmap_session_wsms(struct tee_client *client, u32 session_id,
			      const struct mc_ioctl_buffer *bufs)
{
	struct tee_session *session;
	int ret;

	
	session = client_get_session(client, session_id);
	if (!session)
		return -ENXIO;

	
	ret = session_wsms_remove(session, bufs);
	
	session_put(session);
	mc_dev_devel("session %x, exit with %d\n", session_id, ret);
	return ret;
}

static void cbuf_vm_open(struct vm_area_struct *vmarea)
{
	struct cbuf *cbuf = vmarea->vm_private_data;

	cbuf_get(cbuf);
}

static void cbuf_vm_close(struct vm_area_struct *vmarea)
{
	struct cbuf *cbuf = vmarea->vm_private_data;

	cbuf_put(cbuf);
}

static struct vm_operations_struct cbuf_vm_ops = {
	.open = cbuf_vm_open,
	.close = cbuf_vm_close,
};

int client_cbuf_create(struct tee_client *client, u32 len, uintptr_t *addr,
		       struct vm_area_struct *vmarea)
{
	int err = 0;
	struct cbuf *cbuf = NULL;
	unsigned int order;

	if (WARN(!client, "No client available"))
		return -EINVAL;

	if (!len || (len > BUFFER_LENGTH_MAX))
		return -EINVAL;

	order = get_order(len);
	if (order > MAX_ORDER) {
		mc_dev_err("Buffer size too large\n");
		return -ENOMEM;
	}

	
	cbuf = kzalloc(sizeof(*cbuf), GFP_KERNEL);
	if (!cbuf)
		return -ENOMEM;

	
	atomic_inc(&g_ctx.c_cbufs);
	
	cbuf->addr = __get_free_pages(GFP_USER | __GFP_ZERO, order);
	if (!cbuf->addr) {
		kfree(cbuf);
		
		atomic_dec(&g_ctx.c_cbufs);
		return -ENOMEM;
	}

	
	if (!client_is_kernel(client)) {
		err = cbuf_map(vmarea, cbuf->addr, len, &cbuf->uaddr);
		if (err) {
			free_pages(cbuf->addr, order);
			kfree(cbuf);
			
			atomic_dec(&g_ctx.c_cbufs);
			return err;
		}
	}

	
	cbuf->client = client;
	cbuf->phys = virt_to_phys((void *)cbuf->addr);
	cbuf->len = len;
	cbuf->order = order;
	kref_init(&cbuf->kref);
	INIT_LIST_HEAD(&cbuf->list);

	
	if (vmarea) {
		vmarea->vm_private_data = cbuf;
		vmarea->vm_ops = &cbuf_vm_ops;
	}

	
	if (addr)
		*addr = cbuf->addr;

	
	client_get(client);

	
	mutex_lock(&client->cbufs_lock);
	list_add_tail(&cbuf->list, &client->cbufs);
	mutex_unlock(&client->cbufs_lock);
	mc_dev_devel("created cbuf %p: client %p addr %lx uaddr %lx len %u\n",
		     cbuf, client, cbuf->addr, cbuf->uaddr, cbuf->len);
	return err;
}

static struct cbuf *cbuf_get_by_addr(struct tee_client *client, uintptr_t addr)
{
	struct cbuf *cbuf = NULL, *candidate;
	bool is_kernel = client_is_kernel(client);

	mutex_lock(&client->cbufs_lock);
	list_for_each_entry(candidate, &client->cbufs, list) {
		
		uintptr_t start = is_kernel ?
			candidate->addr : candidate->uaddr;
		uintptr_t end = start + candidate->len;

		
		if (!start)
			break;

		if ((addr >= start) && (addr < end)) {
			cbuf = candidate;
			break;
		}
	}

	if (cbuf)
		cbuf_get(cbuf);

	mutex_unlock(&client->cbufs_lock);
	return cbuf;
}

int client_cbuf_free(struct tee_client *client, uintptr_t addr)
{
	struct cbuf *cbuf = cbuf_get_by_addr(client, addr);

	if (!cbuf) {
		mc_dev_err("cbuf %lu not found\n", addr);
		return -EINVAL;
	}

	
	cbuf_put(cbuf);
	mutex_lock(&client->cbufs_lock);
	cbuf->api_freed = true;
	mutex_unlock(&client->cbufs_lock);
	cbuf_put(cbuf);
	return 0;
}

struct tee_mmu *client_mmu_create(struct tee_client *client, uintptr_t va,
				  u32 len, struct cbuf **cbuf_p)
{
	
	struct cbuf *cbuf = cbuf_get_by_addr(client, va);
	struct task_struct *task = NULL;
	struct tee_mmu *mmu;

	*cbuf_p = cbuf;
	if (cbuf) {
		uintptr_t offset;

		if (client_is_kernel(client)) {
			offset = va - cbuf->addr;
		} else {
			offset = va - cbuf->uaddr;
			
			va = cbuf->addr + offset;
		}

		if ((offset + len) > cbuf->len) {
			mc_dev_err("crosses cbuf boundary\n");
			cbuf_put(cbuf);
			return ERR_PTR(-EINVAL);
		}
	} else if (!client_is_kernel(client)) {
		
		task = current;
	}

	
	mmu = tee_mmu_create(task, (void *)va, len);
	if (IS_ERR_OR_NULL(mmu) && cbuf)
		cbuf_put(cbuf);

	return mmu;
}

void client_mmu_free(struct tee_client *client, uintptr_t va,
		     struct tee_mmu *mmu, struct cbuf *cbuf)
{
	tee_mmu_delete(mmu);
	if (cbuf)
		cbuf_put(cbuf);
}

void client_init(void)
{
	INIT_LIST_HEAD(&client_ctx.clients);
	mutex_init(&client_ctx.clients_lock);

	INIT_LIST_HEAD(&client_ctx.closing_clients);
	mutex_init(&client_ctx.closing_clients_lock);
}

static inline int cbuf_debug_structs(struct kasnprintf_buf *buf,
				     struct cbuf *cbuf)
{
	return kasnprintf(buf, "\tcbuf %p [%d]: addr %lx uaddr %lx len %u\n",
			  cbuf, kref_read(&cbuf->kref), cbuf->addr,
			  cbuf->uaddr, cbuf->len);
}

static int client_debug_structs(struct kasnprintf_buf *buf,
				struct tee_client *client, bool is_closing)
{
	struct cbuf *cbuf;
	struct tee_session *session;
	int ret;

	if (client->pid)
		ret = kasnprintf(buf, "client %p [%d]: %s (%d)%s\n",
				 client, kref_read(&client->kref),
				 client->comm, client->pid,
				 is_closing ? " <closing>" : "");
	else
		ret = kasnprintf(buf, "client %p [%d]: [kernel]%s\n",
				 client, kref_read(&client->kref),
				 is_closing ? " <closing>" : "");

	if (ret < 0)
		return ret;

	
	mutex_lock(&client->cbufs_lock);
	if (list_empty(&client->cbufs))
		goto done_cbufs;

	list_for_each_entry(cbuf, &client->cbufs, list) {
		ret = cbuf_debug_structs(buf, cbuf);
		if (ret < 0)
			goto done_cbufs;
	}

done_cbufs:
	mutex_unlock(&client->cbufs_lock);
	if (ret < 0)
		return ret;

	
	mutex_lock(&client->sessions_lock);
	list_for_each_entry(session, &client->sessions, list) {
		ret = session_debug_structs(buf, session, false);
		if (ret < 0)
			goto done_sessions;
	}

	list_for_each_entry(session, &client->closing_sessions, list) {
		ret = session_debug_structs(buf, session, true);
		if (ret < 0)
			goto done_sessions;
	}

done_sessions:
	mutex_unlock(&client->sessions_lock);

	if (ret < 0)
		return ret;

	return 0;
}

int clients_debug_structs(struct kasnprintf_buf *buf)
{
	struct tee_client *client;
	ssize_t ret = 0;

	mutex_lock(&client_ctx.clients_lock);
	list_for_each_entry(client, &client_ctx.clients, list) {
		ret = client_debug_structs(buf, client, false);
		if (ret < 0)
			break;
	}
	mutex_unlock(&client_ctx.clients_lock);

	if (ret < 0)
		return ret;

	mutex_lock(&client_ctx.closing_clients_lock);
	list_for_each_entry(client, &client_ctx.closing_clients, list) {
		ret = client_debug_structs(buf, client, true);
		if (ret < 0)
			break;
	}
	mutex_unlock(&client_ctx.closing_clients_lock);

	return ret;
}
