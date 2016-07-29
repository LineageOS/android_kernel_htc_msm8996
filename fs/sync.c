
#include <linux/kernel.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/namei.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/syscalls.h>
#include <linux/linkage.h>
#include <linux/pagemap.h>
#include <linux/quotaops.h>
#include <linux/backing-dev.h>
#include "internal.h"
#include <trace/events/mmcio.h>

#define VALID_FLAGS (SYNC_FILE_RANGE_WAIT_BEFORE|SYNC_FILE_RANGE_WRITE| \
			SYNC_FILE_RANGE_WAIT_AFTER)

static int __sync_filesystem(struct super_block *sb, int wait)
{
	if (wait)
		sync_inodes_sb(sb);
	else
		writeback_inodes_sb(sb, WB_REASON_SYNC);

	if (sb->s_op->sync_fs)
		sb->s_op->sync_fs(sb, wait);
	return __sync_blockdev(sb->s_bdev, wait);
}

int sync_filesystem(struct super_block *sb)
{
	int ret;

	WARN_ON(!rwsem_is_locked(&sb->s_umount));

	if (sb->s_flags & MS_RDONLY)
		return 0;

	ret = __sync_filesystem(sb, 0);
	if (ret < 0)
		return ret;
	return __sync_filesystem(sb, 1);
}
EXPORT_SYMBOL(sync_filesystem);

static void sync_inodes_one_sb(struct super_block *sb, void *arg)
{
	if (!(sb->s_flags & MS_RDONLY))
		sync_inodes_sb(sb);
}

static void sync_fs_one_sb(struct super_block *sb, void *arg)
{
	if (!(sb->s_flags & MS_RDONLY) && sb->s_op->sync_fs)
		sb->s_op->sync_fs(sb, *(int *)arg);
}

static void fdatawrite_one_bdev(struct block_device *bdev, void *arg)
{
	filemap_fdatawrite(bdev->bd_inode->i_mapping);
}

static void fdatawait_one_bdev(struct block_device *bdev, void *arg)
{
	filemap_fdatawait(bdev->bd_inode->i_mapping);
}

SYSCALL_DEFINE0(sync)
{
	int nowait = 0, wait = 1;

	trace_sys_sync(0);

	wakeup_flusher_threads(0, WB_REASON_SYNC);
	iterate_supers(sync_inodes_one_sb, NULL);
	iterate_supers(sync_fs_one_sb, &nowait);
	iterate_supers(sync_fs_one_sb, &wait);
	iterate_bdevs(fdatawrite_one_bdev, NULL);
	iterate_bdevs(fdatawait_one_bdev, NULL);
	if (unlikely(laptop_mode))
		laptop_sync_completion();

	trace_sys_sync_done(0);

	return 0;
}

static void do_sync_work(struct work_struct *work)
{
	int nowait = 0;

	iterate_supers(sync_inodes_one_sb, &nowait);
	iterate_supers(sync_fs_one_sb, &nowait);
	iterate_bdevs(fdatawrite_one_bdev, NULL);
	iterate_supers(sync_inodes_one_sb, &nowait);
	iterate_supers(sync_fs_one_sb, &nowait);
	iterate_bdevs(fdatawrite_one_bdev, NULL);
	printk("Emergency Sync complete\n");
	kfree(work);
}

void emergency_sync(void)
{
	struct work_struct *work;

	work = kmalloc(sizeof(*work), GFP_ATOMIC);
	if (work) {
		INIT_WORK(work, do_sync_work);
		schedule_work(work);
	}
}

SYSCALL_DEFINE1(syncfs, int, fd)
{
	struct fd f = fdget(fd);
	struct super_block *sb;
	int ret;

	if (!f.file)
		return -EBADF;
	sb = f.file->f_dentry->d_sb;

	down_read(&sb->s_umount);
	ret = sync_filesystem(sb);
	up_read(&sb->s_umount);

	fdput(f);
	return ret;
}

extern int cancel_fsync;
static int async_fsync(struct file *file)
{
	struct inode *inode = file->f_mapping->host;
	struct super_block *sb = inode->i_sb;

	return (sb->fsync_flags & FLAG_ASYNC_FSYNC) && cancel_fsync;
}

/**
 * vfs_fsync_range - helper to sync a range of data & metadata to disk
 * @file:		file to sync
 * @start:		offset in bytes of the beginning of data range to sync
 * @end:		offset in bytes of the end of data range (inclusive)
 * @datasync:		perform only datasync
 *
 * Write back data in range @start..@end and metadata for @file to disk.  If
 * @datasync is set only metadata needed to access modified file data is
 * written.
 */
int vfs_fsync_range(struct file *file, loff_t start, loff_t end, int datasync)
{
	int err;

	if (!file->f_op->fsync)
		return -EINVAL;

	if (async_fsync(file))
		return 0;

	trace_vfs_fsync(file);
	err = file->f_op->fsync(file, start, end, datasync);
	trace_vfs_fsync_done(file);
	return err;
}
EXPORT_SYMBOL(vfs_fsync_range);

/**
 * vfs_fsync - perform a fsync or fdatasync on a file
 * @file:		file to sync
 * @datasync:		only perform a fdatasync operation
 *
 * Write back data and metadata for @file to disk.  If @datasync is
 * set only metadata needed to access modified file data is written.
 */
int vfs_fsync(struct file *file, int datasync)
{
	return vfs_fsync_range(file, 0, LLONG_MAX, datasync);
}
EXPORT_SYMBOL(vfs_fsync);

static int do_fsync(unsigned int fd, int datasync)
{
	struct fd f = fdget(fd);
	int ret = -EBADF;

	if (f.file) {
		ret = vfs_fsync(f.file, datasync);
		fdput(f);
	}
	return ret;
}

SYSCALL_DEFINE1(fsync, unsigned int, fd)
{
	return do_fsync(fd, 0);
}

SYSCALL_DEFINE1(fdatasync, unsigned int, fd)
{
	return do_fsync(fd, 1);
}

SYSCALL_DEFINE4(sync_file_range, int, fd, loff_t, offset, loff_t, nbytes,
				unsigned int, flags)
{
	int ret;
	struct fd f;
	struct address_space *mapping;
	loff_t endbyte;			
	umode_t i_mode;

	ret = -EINVAL;
	if (flags & ~VALID_FLAGS)
		goto out;

	endbyte = offset + nbytes;

	if ((s64)offset < 0)
		goto out;
	if ((s64)endbyte < 0)
		goto out;
	if (endbyte < offset)
		goto out;

	if (sizeof(pgoff_t) == 4) {
		if (offset >= (0x100000000ULL << PAGE_CACHE_SHIFT)) {
			ret = 0;
			goto out;
		}
		if (endbyte >= (0x100000000ULL << PAGE_CACHE_SHIFT)) {
			nbytes = 0;
		}
	}

	if (nbytes == 0)
		endbyte = LLONG_MAX;
	else
		endbyte--;		

	ret = -EBADF;
	f = fdget(fd);
	if (!f.file)
		goto out;

	i_mode = file_inode(f.file)->i_mode;
	ret = -ESPIPE;
	if (!S_ISREG(i_mode) && !S_ISBLK(i_mode) && !S_ISDIR(i_mode) &&
			!S_ISLNK(i_mode))
		goto out_put;

	mapping = f.file->f_mapping;
	if (!mapping) {
		ret = -EINVAL;
		goto out_put;
	}

	ret = 0;
	if (flags & SYNC_FILE_RANGE_WAIT_BEFORE) {
		ret = filemap_fdatawait_range(mapping, offset, endbyte);
		if (ret < 0)
			goto out_put;
	}

	if (flags & SYNC_FILE_RANGE_WRITE) {
		ret = filemap_fdatawrite_range(mapping, offset, endbyte);
		if (ret < 0)
			goto out_put;
	}

	if (flags & SYNC_FILE_RANGE_WAIT_AFTER)
		ret = filemap_fdatawait_range(mapping, offset, endbyte);

out_put:
	fdput(f);
out:
	return ret;
}

SYSCALL_DEFINE4(sync_file_range2, int, fd, unsigned int, flags,
				 loff_t, offset, loff_t, nbytes)
{
	return sys_sync_file_range(fd, offset, nbytes, flags);
}
