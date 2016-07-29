/*
 *  linux/fs/file.c
 *
 *  Copyright (C) 1998-1999, Stephen Tweedie and Bill Hawes
 *
 *  Manage the dynamic fd arrays in the process files_struct.
 */

#include <linux/syscalls.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/workqueue.h>
#include <linux/crc32c.h>
#include <linux/htc_debug_tools.h>

int sysctl_nr_open __read_mostly = 1024*1024;
int sysctl_nr_open_min = BITS_PER_LONG;
#define __const_max(x, y) ((x) < (y) ? (x) : (y))
int sysctl_nr_open_max = __const_max(INT_MAX, ~(size_t)0/sizeof(void *)) &
			 -BITS_PER_LONG;

static void *alloc_fdmem(size_t size)
{
	if (size <= (PAGE_SIZE << PAGE_ALLOC_COSTLY_ORDER)) {
		void *data = kmalloc(size, GFP_KERNEL|__GFP_NOWARN|__GFP_NORETRY);
		if (data != NULL)
			return data;
	}
	return vmalloc(size);
}

static void __free_fdtable(struct fdtable *fdt)
{
	kvfree(fdt->fd);
	kvfree(fdt->open_fds);
	kvfree(fdt->user);
	kfree(fdt);
}

static void free_fdtable_rcu(struct rcu_head *rcu)
{
	__free_fdtable(container_of(rcu, struct fdtable, rcu));
}

static void copy_fdtable(struct fdtable *nfdt, struct fdtable *ofdt)
{
	unsigned int cpy, set;

	BUG_ON(nfdt->max_fds < ofdt->max_fds);

	cpy = ofdt->max_fds * sizeof(struct file *);
	set = (nfdt->max_fds - ofdt->max_fds) * sizeof(struct file *);
	memcpy(nfdt->fd, ofdt->fd, cpy);
	memset((char *)(nfdt->fd) + cpy, 0, set);

	cpy = ofdt->max_fds / BITS_PER_BYTE;
	set = (nfdt->max_fds - ofdt->max_fds) / BITS_PER_BYTE;
	memcpy(nfdt->open_fds, ofdt->open_fds, cpy);
	memset((char *)(nfdt->open_fds) + cpy, 0, set);
	memcpy(nfdt->close_on_exec, ofdt->close_on_exec, cpy);
	memset((char *)(nfdt->close_on_exec) + cpy, 0, set);

	memcpy(nfdt->user, ofdt->user, ofdt->max_fds * sizeof(*nfdt->user));
	memset(nfdt->user + ofdt->max_fds, 0, (nfdt->max_fds - ofdt->max_fds) * sizeof(*nfdt->user));
}

static struct fdtable * alloc_fdtable(unsigned int nr)
{
	struct fdtable *fdt;
	void *data;

	nr /= (1024 / sizeof(struct file *));
	nr = roundup_pow_of_two(nr + 1);
	nr *= (1024 / sizeof(struct file *));
	if (unlikely(nr > sysctl_nr_open))
		nr = ((sysctl_nr_open - 1) | (BITS_PER_LONG - 1)) + 1;

	fdt = kmalloc(sizeof(struct fdtable), GFP_KERNEL);
	if (!fdt)
		goto out;
	fdt->max_fds = nr;
	data = alloc_fdmem(nr * sizeof(struct file *));
	if (!data)
		goto out_fdt;
	fdt->fd = data;

	data = alloc_fdmem(max_t(size_t,
				 2 * nr / BITS_PER_BYTE, L1_CACHE_BYTES));
	if (!data)
		goto out_arr;
	fdt->open_fds = data;
	data += nr / BITS_PER_BYTE;
	fdt->close_on_exec = data;

	data = alloc_fdmem(sizeof(*fdt->user) * nr);
	if (!data)
		goto out_open;

	fdt->user = (struct fdt_user*) data;
	memset(fdt->user, 0, sizeof(*fdt->user));

	return fdt;

out_open:
	kvfree(fdt->open_fds);
out_arr:
	kvfree(fdt->fd);
out_fdt:
	kfree(fdt);
out:
	return NULL;
}

static int expand_fdtable(struct files_struct *files, int nr)
	__releases(files->file_lock)
	__acquires(files->file_lock)
{
	struct fdtable *new_fdt, *cur_fdt;

	spin_unlock(&files->file_lock);
	new_fdt = alloc_fdtable(nr);
	spin_lock(&files->file_lock);
	if (!new_fdt)
		return -ENOMEM;
	if (unlikely(new_fdt->max_fds <= nr)) {
		__free_fdtable(new_fdt);
		return -EMFILE;
	}
	cur_fdt = files_fdtable(files);
	if (nr >= cur_fdt->max_fds) {
		
		copy_fdtable(new_fdt, cur_fdt);
		rcu_assign_pointer(files->fdt, new_fdt);
		if (cur_fdt != &files->fdtab)
			call_rcu(&cur_fdt->rcu, free_fdtable_rcu);
	} else {
		
		__free_fdtable(new_fdt);
	}
	return 1;
}

static int expand_files(struct files_struct *files, int nr)
{
	struct fdtable *fdt;

	fdt = files_fdtable(files);

	
	if (nr < fdt->max_fds)
		return 0;

	
	if (nr >= sysctl_nr_open)
		return -EMFILE;

	
	return expand_fdtable(files, nr);
}

static inline void __set_close_on_exec(int fd, struct fdtable *fdt)
{
	__set_bit(fd, fdt->close_on_exec);
}

static inline void __clear_close_on_exec(int fd, struct fdtable *fdt)
{
	__clear_bit(fd, fdt->close_on_exec);
}

static inline void __set_open_fd(int fd, struct fdtable *fdt)
{
	__set_bit(fd, fdt->open_fds);
}

static inline void __clear_open_fd(int fd, struct fdtable *fdt)
{
	__clear_bit(fd, fdt->open_fds);
}

static int count_open_files(struct fdtable *fdt)
{
	int size = fdt->max_fds;
	int i;

	
	for (i = size / BITS_PER_LONG; i > 0; ) {
		if (fdt->open_fds[--i])
			break;
	}
	i = (i + 1) * BITS_PER_LONG;
	return i;
}

struct files_struct *dup_fd(struct files_struct *oldf, int *errorp)
{
	struct files_struct *newf;
	struct file **old_fds, **new_fds;
	int open_files, size, i;
	struct fdtable *old_fdt, *new_fdt;

	*errorp = -ENOMEM;
	newf = kmem_cache_alloc(files_cachep, GFP_KERNEL);
	if (!newf)
		goto out;

	atomic_set(&newf->count, 1);

	spin_lock_init(&newf->file_lock);
	newf->next_fd = 0;
	new_fdt = &newf->fdtab;
	new_fdt->max_fds = NR_OPEN_DEFAULT;
	new_fdt->close_on_exec = newf->close_on_exec_init;
	new_fdt->open_fds = newf->open_fds_init;
	new_fdt->fd = &newf->fd_array[0];
	new_fdt->user = &newf->user_array[0];

	spin_lock(&oldf->file_lock);
	old_fdt = files_fdtable(oldf);
	open_files = count_open_files(old_fdt);

	while (unlikely(open_files > new_fdt->max_fds)) {
		spin_unlock(&oldf->file_lock);

		if (new_fdt != &newf->fdtab)
			__free_fdtable(new_fdt);

		new_fdt = alloc_fdtable(open_files - 1);
		if (!new_fdt) {
			*errorp = -ENOMEM;
			goto out_release;
		}

		
		if (unlikely(new_fdt->max_fds < open_files)) {
			__free_fdtable(new_fdt);
			*errorp = -EMFILE;
			goto out_release;
		}

		spin_lock(&oldf->file_lock);
		old_fdt = files_fdtable(oldf);
		open_files = count_open_files(old_fdt);
	}

	old_fds = old_fdt->fd;
	new_fds = new_fdt->fd;

	memcpy(new_fdt->open_fds, old_fdt->open_fds, open_files / 8);
	memcpy(new_fdt->close_on_exec, old_fdt->close_on_exec, open_files / 8);
	memset(new_fdt->user, 0, open_files * sizeof(*old_fdt->user));

	for (i = open_files; i != 0; i--) {
		struct file *f = *old_fds++;
		if (f) {
			get_file(f);
		} else {
			__clear_open_fd(open_files - i, new_fdt);
		}
		rcu_assign_pointer(*new_fds++, f);
	}
	spin_unlock(&oldf->file_lock);

	
	size = (new_fdt->max_fds - open_files) * sizeof(struct file *);

	
	memset(new_fds, 0, size);

	if (new_fdt->max_fds > open_files) {
		int left = (new_fdt->max_fds - open_files) / 8;
		int start = open_files / BITS_PER_LONG;

		memset(&new_fdt->open_fds[start], 0, left);
		memset(&new_fdt->close_on_exec[start], 0, left);
		memset(&new_fdt->user[open_files], 0, (new_fdt->max_fds - open_files) * sizeof(*new_fdt->user));
	}

	rcu_assign_pointer(newf->fdt, new_fdt);

	return newf;

out_release:
	kmem_cache_free(files_cachep, newf);
out:
	return NULL;
}

static struct fdtable *close_files(struct files_struct * files)
{
	struct fdtable *fdt = rcu_dereference_raw(files->fdt);
	int i, j = 0;

	for (;;) {
		unsigned long set;
		i = j * BITS_PER_LONG;
		if (i >= fdt->max_fds)
			break;
		set = fdt->open_fds[j++];
		while (set) {
			if (set & 1) {
				struct file * file = xchg(&fdt->fd[i], NULL);
				if (file) {
					filp_close(file, files);
					cond_resched_rcu_qs();
				}
			}
			i++;
			set >>= 1;
		}
	}

	return fdt;
}

struct files_struct *get_files_struct(struct task_struct *task)
{
	struct files_struct *files;

	task_lock(task);
	files = task->files;
	if (files)
		atomic_inc(&files->count);
	task_unlock(task);

	return files;
}

void put_files_struct(struct files_struct *files)
{
	if (atomic_dec_and_test(&files->count)) {
		struct fdtable *fdt = close_files(files);

		
		if (fdt != &files->fdtab)
			__free_fdtable(fdt);
		kmem_cache_free(files_cachep, files);
	}
}

void reset_files_struct(struct files_struct *files)
{
	struct task_struct *tsk = current;
	struct files_struct *old;

	old = tsk->files;
	task_lock(tsk);
	tsk->files = files;
	task_unlock(tsk);
	put_files_struct(old);
}

void exit_files(struct task_struct *tsk)
{
	struct files_struct * files = tsk->files;

	if (files) {
		task_lock(tsk);
		tsk->files = NULL;
		task_unlock(tsk);
		put_files_struct(files);
	}
}

struct files_struct init_files = {
	.count		= ATOMIC_INIT(1),
	.fdt		= &init_files.fdtab,
	.fdtab		= {
		.max_fds	= NR_OPEN_DEFAULT,
		.fd		= &init_files.fd_array[0],
		.close_on_exec	= init_files.close_on_exec_init,
		.open_fds	= init_files.open_fds_init,
		.user		= &init_files.user_array[0],
	},
	.file_lock	= __SPIN_LOCK_UNLOCKED(init_files.file_lock),
};

static void fdtable_usage_dump(struct fdtable *fdt)
{
	int i, last_crc = 0, repeats = 0;
	char* buf;
	struct file* file;
	struct task_struct* user = NULL;
	int this_crc, pid;
	char* path;
	char* spath;
	unsigned long timestamp;

	buf = (char*) kmalloc(PATH_MAX, GFP_ATOMIC);
	if (!buf) {
		pr_err("%s: fail to alloc buffer\n", __func__);
		return;
	}

	rcu_read_lock();
	for (i = 0; i < fdt->max_fds; i++) {

		file = fdt->fd[i];
		if (!file)
			continue;

		pid = fdt->user[i].installer;
		timestamp = fdt->user[i].install_ts;

		user = find_task_by_vpid(pid);
		if (user)
			get_task_struct(user);

		path = d_path(&file->f_path, buf, PATH_MAX);

		if (IS_ERR(path))
			path = "<unknown>";
		else {
			spath = strstr(path, ":[");
			if (spath) spath[0] = '\0';
		}

		this_crc = crc32c(pid, path, strlen(path));
		if (this_crc != last_crc || i == fdt->max_fds - 1) {
			if (repeats)
				pr_warn(" < ... repeats %d time%s ... >\n", repeats, repeats > 1 ? "s" : "");
				pr_warn("%d->fd[%d] file: %s, user: %d (%s %d:%d), opened at %lu ms\n", current->tgid, i, path, pid,
				user ? user->comm : "<unknown>", user ? user->tgid : -1, user ? user->pid : -1, timestamp);

				last_crc = this_crc;
				repeats = 0;
		} else
			repeats++;

		if (user)
			put_task_struct(user);
	}
	rcu_read_unlock();
	kfree(buf);
}

int __alloc_fd(struct files_struct *files,
	       unsigned start, unsigned end, unsigned flags)
{
	unsigned int fd;
	int error;
	struct fdtable *fdt;
	static unsigned long debugging_ratelimit = 0;
	const unsigned long debugging_delay_ms = 30000;

	spin_lock(&files->file_lock);
repeat:
	fdt = files_fdtable(files);
	fd = start;
	if (fd < files->next_fd)
		fd = files->next_fd;

	if (fd < fdt->max_fds)
		fd = find_next_zero_bit(fdt->open_fds, fdt->max_fds, fd);

	error = -EMFILE;
	if (fd >= end)
		goto out;

	error = expand_files(files, fd);
	if (error < 0)
		goto out;

	if (error)
		goto repeat;

	if (start <= files->next_fd)
		files->next_fd = fd + 1;

	__set_open_fd(fd, fdt);
	if (flags & O_CLOEXEC)
		__set_close_on_exec(fd, fdt);
	else
		__clear_close_on_exec(fd, fdt);

	error = fd;
#if 1
	
	if (rcu_access_pointer(fdt->fd[fd]) != NULL) {
		printk(KERN_WARNING "alloc_fd: slot %d not NULL!\n", fd);
		rcu_assign_pointer(fdt->fd[fd], NULL);
	}
#endif

out:
	if (unlikely(error == -EMFILE)) {
		if (jiffies > debugging_ratelimit) {
			debugging_ratelimit = jiffies + msecs_to_jiffies(debugging_delay_ms);

			pr_warn("[%s] Too many open files (%d/%u), dump all fdt users:\n",
			__func__, count_open_files(fdt), fdt->max_fds);
			dump_stack();
			fdtable_usage_dump(fdt);
			pr_warn("[%s] end of dump\n", __func__);
		}
	}

	spin_unlock(&files->file_lock);

	return error;
}

static int alloc_fd(unsigned start, unsigned flags)
{
	return __alloc_fd(current->files, start, rlimit(RLIMIT_NOFILE), flags);
}

int get_unused_fd_flags(unsigned flags)
{
	return __alloc_fd(current->files, 0, rlimit(RLIMIT_NOFILE), flags);
}
EXPORT_SYMBOL(get_unused_fd_flags);

static void __put_unused_fd(struct files_struct *files, unsigned int fd)
{
	struct fdtable *fdt = files_fdtable(files);
	__clear_open_fd(fd, fdt);
	if (fd < files->next_fd)
		files->next_fd = fd;
}

void put_unused_fd(unsigned int fd)
{
	struct files_struct *files = current->files;
	spin_lock(&files->file_lock);
	__put_unused_fd(files, fd);
	spin_unlock(&files->file_lock);
}

EXPORT_SYMBOL(put_unused_fd);


void __fd_install(struct files_struct *files, unsigned int fd,
		struct file *file)
{
	struct fdtable *fdt;
	spin_lock(&files->file_lock);
	fdt = files_fdtable(files);
	BUG_ON(fdt->fd[fd] != NULL);
	rcu_assign_pointer(fdt->fd[fd], file);
	fdt->user[fd].installer = current->pid;
	fdt->user[fd].install_ts = htc_debug_get_sched_clock_ms();
	spin_unlock(&files->file_lock);
}

void fd_install(unsigned int fd, struct file *file)
{
	__fd_install(current->files, fd, file);
}

EXPORT_SYMBOL(fd_install);

int __close_fd(struct files_struct *files, unsigned fd)
{
	struct file *file;
	struct fdtable *fdt;
	struct fdt_user* user;
	struct task_struct* task;


	spin_lock(&files->file_lock);
	fdt = files_fdtable(files);
	if (fd >= fdt->max_fds) {
		pr_debug("[%s] fd %u exceeds max_fds %u (user: %s %d:%d)\n", __func__, fd, fdt->max_fds,
		current->comm, current->tgid, current->pid);
		goto out_unlock;
	}
	file = fdt->fd[fd];
	if (!file) {
		user = &fdt->user[fd];
		if (unlikely(user->remover && user->remover != current->pid)) {
			task = find_task_by_vpid(user->remover);
			pr_warn("[%s] fd %u of %s %d:%d is already closed by thread %d (%s %d:%d) at %lu ms, opened at %lu ms\n",
			__func__, fd, current->comm, current->tgid, current->pid, user->remover,
			task ? task->comm : "<unknown>", task ? task->tgid : -1, task ? task->pid : -1, user->remove_ts, user->install_ts);
		}
		goto out_unlock;
	}
	rcu_assign_pointer(fdt->fd[fd], NULL);
	fdt->user[fd].remover = current->pid;
	fdt->user[fd].remove_ts = htc_debug_get_sched_clock_ms();
	__clear_close_on_exec(fd, fdt);
	__put_unused_fd(files, fd);
	spin_unlock(&files->file_lock);
	return filp_close(file, files);

out_unlock:
	spin_unlock(&files->file_lock);
	return -EBADF;
}

void do_close_on_exec(struct files_struct *files)
{
	unsigned i;
	struct fdtable *fdt;

	
	spin_lock(&files->file_lock);
	for (i = 0; ; i++) {
		unsigned long set;
		unsigned fd = i * BITS_PER_LONG;
		fdt = files_fdtable(files);
		if (fd >= fdt->max_fds)
			break;
		set = fdt->close_on_exec[i];
		if (!set)
			continue;
		fdt->close_on_exec[i] = 0;
		for ( ; set ; fd++, set >>= 1) {
			struct file *file;
			if (!(set & 1))
				continue;
			file = fdt->fd[fd];
			if (!file)
				continue;
			rcu_assign_pointer(fdt->fd[fd], NULL);
			__put_unused_fd(files, fd);
			spin_unlock(&files->file_lock);
			filp_close(file, files);
			cond_resched();
			spin_lock(&files->file_lock);
		}

	}
	spin_unlock(&files->file_lock);
}

static struct file *__fget(unsigned int fd, fmode_t mask)
{
	struct files_struct *files = current->files;
	struct file *file;

	rcu_read_lock();
	file = fcheck_files(files, fd);
	if (file) {
		
		if ((file->f_mode & mask) ||
		    !atomic_long_inc_not_zero(&file->f_count))
			file = NULL;
	}
	rcu_read_unlock();

	return file;
}

struct file *fget(unsigned int fd)
{
	return __fget(fd, FMODE_PATH);
}
EXPORT_SYMBOL(fget);

struct file *fget_raw(unsigned int fd)
{
	return __fget(fd, 0);
}
EXPORT_SYMBOL(fget_raw);

static unsigned long __fget_light(unsigned int fd, fmode_t mask)
{
	struct files_struct *files = current->files;
	struct file *file;

	if (atomic_read(&files->count) == 1) {
		file = __fcheck_files(files, fd);
		if (!file || unlikely(file->f_mode & mask))
			return 0;
		return (unsigned long)file;
	} else {
		file = __fget(fd, mask);
		if (!file)
			return 0;
		return FDPUT_FPUT | (unsigned long)file;
	}
}
unsigned long __fdget(unsigned int fd)
{
	return __fget_light(fd, FMODE_PATH);
}
EXPORT_SYMBOL(__fdget);

unsigned long __fdget_raw(unsigned int fd)
{
	return __fget_light(fd, 0);
}

unsigned long __fdget_pos(unsigned int fd)
{
	unsigned long v = __fdget(fd);
	struct file *file = (struct file *)(v & ~3);

	if (file && (file->f_mode & FMODE_ATOMIC_POS)) {
		if (file_count(file) > 1) {
			v |= FDPUT_POS_UNLOCK;
			mutex_lock(&file->f_pos_lock);
		}
	}
	return v;
}


void set_close_on_exec(unsigned int fd, int flag)
{
	struct files_struct *files = current->files;
	struct fdtable *fdt;
	spin_lock(&files->file_lock);
	fdt = files_fdtable(files);
	if (flag)
		__set_close_on_exec(fd, fdt);
	else
		__clear_close_on_exec(fd, fdt);
	spin_unlock(&files->file_lock);
}

bool get_close_on_exec(unsigned int fd)
{
	struct files_struct *files = current->files;
	struct fdtable *fdt;
	bool res;
	rcu_read_lock();
	fdt = files_fdtable(files);
	res = close_on_exec(fd, fdt);
	rcu_read_unlock();
	return res;
}

static int do_dup2(struct files_struct *files,
	struct file *file, unsigned fd, unsigned flags)
__releases(&files->file_lock)
{
	struct file *tofree;
	struct fdtable *fdt;

	fdt = files_fdtable(files);
	tofree = fdt->fd[fd];
	if (!tofree && fd_is_open(fd, fdt))
		goto Ebusy;
	get_file(file);
	rcu_assign_pointer(fdt->fd[fd], file);
	__set_open_fd(fd, fdt);
	if (flags & O_CLOEXEC)
		__set_close_on_exec(fd, fdt);
	else
		__clear_close_on_exec(fd, fdt);
	spin_unlock(&files->file_lock);

	if (tofree)
		filp_close(tofree, files);

	return fd;

Ebusy:
	spin_unlock(&files->file_lock);
	return -EBUSY;
}

int replace_fd(unsigned fd, struct file *file, unsigned flags)
{
	int err;
	struct files_struct *files = current->files;

	if (!file)
		return __close_fd(files, fd);

	if (fd >= rlimit(RLIMIT_NOFILE))
		return -EBADF;

	spin_lock(&files->file_lock);
	err = expand_files(files, fd);
	if (unlikely(err < 0))
		goto out_unlock;
	return do_dup2(files, file, fd, flags);

out_unlock:
	spin_unlock(&files->file_lock);
	return err;
}

SYSCALL_DEFINE3(dup3, unsigned int, oldfd, unsigned int, newfd, int, flags)
{
	int err = -EBADF;
	struct file *file;
	struct files_struct *files = current->files;

	if ((flags & ~O_CLOEXEC) != 0)
		return -EINVAL;

	if (unlikely(oldfd == newfd))
		return -EINVAL;

	if (newfd >= rlimit(RLIMIT_NOFILE))
		return -EBADF;

	spin_lock(&files->file_lock);
	err = expand_files(files, newfd);
	file = fcheck(oldfd);
	if (unlikely(!file))
		goto Ebadf;
	if (unlikely(err < 0)) {
		if (err == -EMFILE)
			goto Ebadf;
		goto out_unlock;
	}
	return do_dup2(files, file, newfd, flags);

Ebadf:
	err = -EBADF;
out_unlock:
	spin_unlock(&files->file_lock);
	return err;
}

SYSCALL_DEFINE2(dup2, unsigned int, oldfd, unsigned int, newfd)
{
	if (unlikely(newfd == oldfd)) { 
		struct files_struct *files = current->files;
		int retval = oldfd;

		rcu_read_lock();
		if (!fcheck_files(files, oldfd))
			retval = -EBADF;
		rcu_read_unlock();
		return retval;
	}
	return sys_dup3(oldfd, newfd, 0);
}

SYSCALL_DEFINE1(dup, unsigned int, fildes)
{
	int ret = -EBADF;
	struct file *file = fget_raw(fildes);

	if (file) {
		ret = get_unused_fd();
		if (ret >= 0)
			fd_install(ret, file);
		else
			fput(file);
	}
	return ret;
}

int f_dupfd(unsigned int from, struct file *file, unsigned flags)
{
	int err;
	if (from >= rlimit(RLIMIT_NOFILE))
		return -EINVAL;
	err = alloc_fd(from, flags);
	if (err >= 0) {
		get_file(file);
		fd_install(err, file);
	}
	return err;
}

int iterate_fd(struct files_struct *files, unsigned n,
		int (*f)(const void *, struct file *, unsigned),
		const void *p)
{
	struct fdtable *fdt;
	int res = 0;
	if (!files)
		return 0;
	spin_lock(&files->file_lock);
	for (fdt = files_fdtable(files); n < fdt->max_fds; n++) {
		struct file *file;
		file = rcu_dereference_check_fdtable(files, fdt->fd[n]);
		if (!file)
			continue;
		res = f(p, file, n);
		if (res)
			break;
	}
	spin_unlock(&files->file_lock);
	return res;
}
EXPORT_SYMBOL(iterate_fd);
