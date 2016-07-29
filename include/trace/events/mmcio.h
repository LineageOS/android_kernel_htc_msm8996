#undef TRACE_SYSTEM
#define TRACE_SYSTEM mmcio

#if !defined(_TRACE_MMCIO_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MMCIO_H

#include <linux/tracepoint.h>

TRACE_EVENT(sys_sync,
	TP_PROTO(int i),

	TP_ARGS(i),

	TP_STRUCT__entry(
		__array( char,		comm,	TASK_COMM_LEN	)
	),

	TP_fast_assign(
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
	),

	TP_printk("[%s]", __entry->comm)
);

TRACE_EVENT(sys_sync_done,
	TP_PROTO(int i),

	TP_ARGS(i),

	TP_STRUCT__entry(
		__array( char,		comm,	TASK_COMM_LEN	)
	),

	TP_fast_assign(
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
	),

	TP_printk("[%s]", __entry->comm)
);

DECLARE_EVENT_CLASS(readpage_ops,
	TP_PROTO(char *path, int size),

	TP_ARGS(path, size),

	TP_STRUCT__entry(
		__field(unsigned, pid)
		__field(int, size)
		__array( char,		comm,	TASK_COMM_LEN	)
		__array( char,		path,	256	)
	),

	TP_fast_assign(
		__entry->pid		= current->pid;
		__entry->size = size;
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
		strncpy(__entry->path, path, 256);
	),

	TP_printk("file %s pages %d [%s:%u]",
		__entry->path, __entry->size, __entry->comm, __entry->pid)
);

DEFINE_EVENT(readpage_ops, ext4_read_page,

	TP_PROTO(char *path, int size),

	TP_ARGS(path, size)
);

DEFINE_EVENT(readpage_ops, fuse_read_page,

	TP_PROTO(char *path, int size),

	TP_ARGS(path, size)
);

DEFINE_EVENT(readpage_ops, readahead,

	TP_PROTO(char *path, int size),

	TP_ARGS(path, size)
);
#if 0
TRACE_EVENT(readahead,
	TP_PROTO(struct file *file, int size),

	TP_ARGS(file, size),

	TP_STRUCT__entry(
		__field(unsigned, pid)
		__field(int, size)
		__field(	dev_t,	dev			)
		__array( char,		comm,	TASK_COMM_LEN	)
		__dynamic_array(unsigned char,	name, file->f_path.dentry->d_name.len + 1)
	),

	TP_fast_assign(
		__entry->pid		= current->pid;
		__entry->dev		= file->f_path.dentry->d_inode->i_sb->s_dev;
		__entry->size = size;
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
		memcpy(__get_dynamic_array(name), file->f_path.dentry->d_name.name,
			file->f_path.dentry->d_name.len + 1);
	),

	TP_printk("dev %d,%d (%s) pages %d [%s:%u]",
		MAJOR(__entry->dev), MINOR(__entry->dev),
		__get_str(name), __entry->size, __entry->comm, __entry->pid)
);
#endif
DECLARE_EVENT_CLASS(file_op,
	TP_PROTO(struct file *file),

	TP_ARGS(file),

	TP_STRUCT__entry(
		__array( char,		comm,	TASK_COMM_LEN	)
		__dynamic_array(unsigned char,	name, file->f_path.dentry->d_name.len + 1)
	),

	TP_fast_assign(
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
		memcpy(__get_dynamic_array(name), file->f_path.dentry->d_name.name,
			file->f_path.dentry->d_name.len + 1);
	),

	TP_printk("(%s) [%s]", __get_str(name), __entry->comm)
);

DEFINE_EVENT(file_op, readahead_exit,

	TP_PROTO(struct file *file),

	TP_ARGS(file)
);

DEFINE_EVENT(file_op, file_write_done,

	TP_PROTO(struct file *file),

	TP_ARGS(file)
);

DEFINE_EVENT(file_op, vfs_fsync,

	TP_PROTO(struct file *file),

	TP_ARGS(file)
);

DEFINE_EVENT(file_op, vfs_fsync_done,

	TP_PROTO(struct file *file),

	TP_ARGS(file)
);

TRACE_EVENT(mmc_req_start,

	TP_PROTO(struct device *dev, int opcode, unsigned start,  int blocks),

	TP_ARGS(dev, opcode, start, blocks),

	TP_STRUCT__entry(
		__array(char, host, 6)
		__field(int, opcode)
		__field(unsigned, start)
		__field(int, blocks)
	),

	TP_fast_assign(
		__entry->opcode = opcode;
		__entry->start = start;
		__entry->blocks = blocks;
		strncpy(__entry->host, dev_name(dev), 6);
	),

	TP_printk("%s: CMD%d start %u blocks %d",
		__entry->host, __entry->opcode,
		__entry->start, __entry->blocks)
);

TRACE_EVENT(mmc_req_end,

	TP_PROTO(struct device *dev, int opcode),

	TP_ARGS(dev, opcode),

	TP_STRUCT__entry(
		__array(char, host, 6)
		__field(int, opcode)
	),

	TP_fast_assign(
		__entry->opcode = opcode;
		strncpy(__entry->host, dev_name(dev), 6);
	),

	TP_printk("%s: CMD%d end", __entry->host, __entry->opcode)
);

TRACE_EVENT(mmc_request_done,

	TP_PROTO(struct device *dev, int opcode, unsigned start,  int blocks,  s64 time),

	TP_ARGS(dev, opcode, start, blocks, time),

	TP_STRUCT__entry(
		__array(char, host, 6)
		__field(int, opcode)
		__field(unsigned, start)
		__field(int, blocks)
		__field(s64, time)
	),

	TP_fast_assign(
		__entry->opcode = opcode;
		__entry->start = start;
		__entry->blocks = blocks;
		__entry->time = time;
		strncpy(__entry->host, dev_name(dev), 6);
	),

	TP_printk("%s: CMD%d start %u blocks %d, %lldms",
		__entry->host, __entry->opcode,
		__entry->start, __entry->blocks, __entry->time)
);

DECLARE_EVENT_CLASS(file_write_op,
	TP_PROTO(struct dentry *dentry, size_t nr_bytes),

	TP_ARGS(dentry, nr_bytes),

	TP_STRUCT__entry(
		__field(unsigned, pid)
		__field(	dev_t,	dev		)
		__field(	size_t,	nr_bytes		)
		__array( char,		comm,	TASK_COMM_LEN	)
		__dynamic_array(unsigned char,	name, dentry->d_name.len + 1)
	),

	TP_fast_assign(
		__entry->pid		= current->pid;
		__entry->dev		= dentry->d_inode->i_sb->s_dev;
		__entry->nr_bytes	= nr_bytes;
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
		memcpy(__get_dynamic_array(name), dentry->d_name.name,
			dentry->d_name.len + 1);
	),

	TP_printk("dev %d,%d %s nr_bytes %ld [%s:%u]",
		  MAJOR(__entry->dev), MINOR(__entry->dev),
		  __get_str(name),
		  (unsigned long)__entry->nr_bytes,
		  __entry->comm, __entry->pid)
);

DEFINE_EVENT(file_write_op, ext4_file_write,

	TP_PROTO(struct dentry *dentry, size_t nr_bytes),

	TP_ARGS(dentry, nr_bytes)
);

DEFINE_EVENT(file_write_op, fuse_file_write,

	TP_PROTO(struct dentry *dentry, size_t nr_bytes),

	TP_ARGS(dentry, nr_bytes)
);

DEFINE_EVENT(file_write_op, fat_file_write,

	TP_PROTO(struct dentry *dentry, size_t nr_bytes),

	TP_ARGS(dentry, nr_bytes)
);

DEFINE_EVENT(file_write_op, blkdev_file_write,

	TP_PROTO(struct dentry *dentry, size_t nr_bytes),

	TP_ARGS(dentry, nr_bytes)
);

TRACE_EVENT(vfs_unlink,
	TP_PROTO(struct dentry *dentry, u64 size),

	TP_ARGS(dentry, size),

	TP_STRUCT__entry(
		__field(u64, size)
		__array( char,		comm,	TASK_COMM_LEN	)
		__dynamic_array(unsigned char,	name, dentry->d_name.len + 1)
	),

	TP_fast_assign(
		__entry->size = size;
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
		memcpy(__get_dynamic_array(name), dentry->d_name.name,
			dentry->d_name.len + 1);
	),

	TP_printk("size %llu (%s) [%s]", __entry->size,
		__get_str(name), __entry->comm)
);

TRACE_EVENT(vfs_unlink_done,
	TP_PROTO(struct dentry *dentry),

	TP_ARGS(dentry),

	TP_STRUCT__entry(
		__array( char,		comm,	TASK_COMM_LEN	)
		__dynamic_array(unsigned char,	name, dentry->d_name.len + 1)
	),

	TP_fast_assign(
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
		memcpy(__get_dynamic_array(name), dentry->d_name.name,
			dentry->d_name.len + 1);
	),

	TP_printk("(%s) [%s]", __get_str(name), __entry->comm)
);
#endif 

#include <trace/define_trace.h>
