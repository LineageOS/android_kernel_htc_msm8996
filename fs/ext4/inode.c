/*
 *  linux/fs/ext4/inode.c
 *
 * Copyright (C) 1992, 1993, 1994, 1995
 * Remy Card (card@masi.ibp.fr)
 * Laboratoire MASI - Institut Blaise Pascal
 * Universite Pierre et Marie Curie (Paris VI)
 *
 *  from
 *
 *  linux/fs/minix/inode.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  64-bit file support on 64-bit platforms by Jakub Jelinek
 *	(jj@sunsite.ms.mff.cuni.cz)
 *
 *  Assorted race fixes, rewrite of ext4_get_block() by Al Viro, 2000
 */

#include <linux/fs.h>
#include <linux/time.h>
#include <linux/jbd2.h>
#include <linux/highuid.h>
#include <linux/pagemap.h>
#include <linux/quotaops.h>
#include <linux/string.h>
#include <linux/buffer_head.h>
#include <linux/writeback.h>
#include <linux/pagevec.h>
#include <linux/mpage.h>
#include <linux/namei.h>
#include <linux/uio.h>
#include <linux/bio.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <linux/aio.h>
#include <linux/bitops.h>

#include "ext4_jbd2.h"
#include "xattr.h"
#include "acl.h"
#include "truncate.h"

#include <trace/events/ext4.h>
#include <trace/events/mmcio.h>

#define MPAGE_DA_EXTENT_TAIL 0x01

static __u32 ext4_inode_csum(struct inode *inode, struct ext4_inode *raw,
			      struct ext4_inode_info *ei)
{
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	__u16 csum_lo;
	__u16 csum_hi = 0;
	__u32 csum;

	csum_lo = le16_to_cpu(raw->i_checksum_lo);
	raw->i_checksum_lo = 0;
	if (EXT4_INODE_SIZE(inode->i_sb) > EXT4_GOOD_OLD_INODE_SIZE &&
	    EXT4_FITS_IN_INODE(raw, ei, i_checksum_hi)) {
		csum_hi = le16_to_cpu(raw->i_checksum_hi);
		raw->i_checksum_hi = 0;
	}

	csum = ext4_chksum(sbi, ei->i_csum_seed, (__u8 *)raw,
			   EXT4_INODE_SIZE(inode->i_sb));

	raw->i_checksum_lo = cpu_to_le16(csum_lo);
	if (EXT4_INODE_SIZE(inode->i_sb) > EXT4_GOOD_OLD_INODE_SIZE &&
	    EXT4_FITS_IN_INODE(raw, ei, i_checksum_hi))
		raw->i_checksum_hi = cpu_to_le16(csum_hi);

	return csum;
}

static int ext4_inode_csum_verify(struct inode *inode, struct ext4_inode *raw,
				  struct ext4_inode_info *ei)
{
	__u32 provided, calculated;

	if (EXT4_SB(inode->i_sb)->s_es->s_creator_os !=
	    cpu_to_le32(EXT4_OS_LINUX) ||
	    !ext4_has_metadata_csum(inode->i_sb))
		return 1;

	provided = le16_to_cpu(raw->i_checksum_lo);
	calculated = ext4_inode_csum(inode, raw, ei);
	if (EXT4_INODE_SIZE(inode->i_sb) > EXT4_GOOD_OLD_INODE_SIZE &&
	    EXT4_FITS_IN_INODE(raw, ei, i_checksum_hi))
		provided |= ((__u32)le16_to_cpu(raw->i_checksum_hi)) << 16;
	else
		calculated &= 0xFFFF;

	return provided == calculated;
}

static void ext4_inode_csum_set(struct inode *inode, struct ext4_inode *raw,
				struct ext4_inode_info *ei)
{
	__u32 csum;

	if (EXT4_SB(inode->i_sb)->s_es->s_creator_os !=
	    cpu_to_le32(EXT4_OS_LINUX) ||
	    !ext4_has_metadata_csum(inode->i_sb))
		return;

	csum = ext4_inode_csum(inode, raw, ei);
	raw->i_checksum_lo = cpu_to_le16(csum & 0xFFFF);
	if (EXT4_INODE_SIZE(inode->i_sb) > EXT4_GOOD_OLD_INODE_SIZE &&
	    EXT4_FITS_IN_INODE(raw, ei, i_checksum_hi))
		raw->i_checksum_hi = cpu_to_le16(csum >> 16);
}

static inline int ext4_begin_ordered_truncate(struct inode *inode,
					      loff_t new_size)
{
	trace_ext4_begin_ordered_truncate(inode, new_size);
	if (!EXT4_I(inode)->jinode)
		return 0;
	return jbd2_journal_begin_ordered_truncate(EXT4_JOURNAL(inode),
						   EXT4_I(inode)->jinode,
						   new_size);
}

static void ext4_invalidatepage(struct page *page, unsigned int offset,
				unsigned int length);
static int __ext4_journalled_writepage(struct page *page, unsigned int len);
static int ext4_bh_delay_or_unwritten(handle_t *handle, struct buffer_head *bh);
static int ext4_meta_trans_blocks(struct inode *inode, int lblocks,
				  int pextents);

static int ext4_inode_is_fast_symlink(struct inode *inode)
{
        int ea_blocks = EXT4_I(inode)->i_file_acl ?
		EXT4_CLUSTER_SIZE(inode->i_sb) >> 9 : 0;

	if (ext4_has_inline_data(inode))
		return 0;

	return (S_ISLNK(inode->i_mode) && inode->i_blocks - ea_blocks == 0);
}

int ext4_truncate_restart_trans(handle_t *handle, struct inode *inode,
				 int nblocks)
{
	int ret;

	BUG_ON(EXT4_JOURNAL(inode) == NULL);
	jbd_debug(2, "restarting handle %p\n", handle);
	up_write(&EXT4_I(inode)->i_data_sem);
	ret = ext4_journal_restart(handle, nblocks);
	down_write(&EXT4_I(inode)->i_data_sem);
	ext4_discard_preallocations(inode);

	return ret;
}

void ext4_evict_inode(struct inode *inode)
{
	handle_t *handle;
	int err;

	trace_ext4_evict_inode(inode);

	if (inode->i_nlink) {
		if (ext4_should_journal_data(inode) &&
		    (S_ISLNK(inode->i_mode) || S_ISREG(inode->i_mode)) &&
		    inode->i_ino != EXT4_JOURNAL_INO) {
			journal_t *journal = EXT4_SB(inode->i_sb)->s_journal;
			tid_t commit_tid = EXT4_I(inode)->i_datasync_tid;

			jbd2_complete_transaction(journal, commit_tid);
			filemap_write_and_wait(&inode->i_data);
		}
		truncate_inode_pages_final(&inode->i_data);

		WARN_ON(atomic_read(&EXT4_I(inode)->i_ioend_count));
		goto no_delete;
	}

	if (is_bad_inode(inode))
		goto no_delete;
	dquot_initialize(inode);

	if (ext4_should_order_data(inode))
		ext4_begin_ordered_truncate(inode, 0);
	truncate_inode_pages_final(&inode->i_data);

	WARN_ON(atomic_read(&EXT4_I(inode)->i_ioend_count));

	sb_start_intwrite(inode->i_sb);
	handle = ext4_journal_start(inode, EXT4_HT_TRUNCATE,
				    ext4_blocks_for_truncate(inode)+3);
	if (IS_ERR(handle)) {
		ext4_std_error(inode->i_sb, PTR_ERR(handle));
		ext4_orphan_del(NULL, inode);
		sb_end_intwrite(inode->i_sb);
		goto no_delete;
	}

	if (IS_SYNC(inode))
		ext4_handle_sync(handle);
	inode->i_size = 0;
	err = ext4_mark_inode_dirty(handle, inode);
	if (err) {
		ext4_warning(inode->i_sb,
			     "couldn't mark inode dirty (err %d)", err);
		goto stop_handle;
	}
	if (inode->i_blocks)
		ext4_truncate(inode);

	if (!ext4_handle_has_enough_credits(handle, 3)) {
		err = ext4_journal_extend(handle, 3);
		if (err > 0)
			err = ext4_journal_restart(handle, 3);
		if (err != 0) {
			ext4_warning(inode->i_sb,
				     "couldn't extend journal (err %d)", err);
		stop_handle:
			ext4_journal_stop(handle);
			ext4_orphan_del(NULL, inode);
			sb_end_intwrite(inode->i_sb);
			goto no_delete;
		}
	}

	ext4_orphan_del(handle, inode);
	EXT4_I(inode)->i_dtime	= get_seconds();

	if (ext4_mark_inode_dirty(handle, inode))
		
		ext4_clear_inode(inode);
	else
		ext4_free_inode(handle, inode);
	ext4_journal_stop(handle);
	sb_end_intwrite(inode->i_sb);
	return;
no_delete:
	ext4_clear_inode(inode);	
}

#ifdef CONFIG_QUOTA
qsize_t *ext4_get_reserved_space(struct inode *inode)
{
	return &EXT4_I(inode)->i_reserved_quota;
}
#endif

void ext4_da_update_reserve_space(struct inode *inode,
					int used, int quota_claim)
{
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	struct ext4_inode_info *ei = EXT4_I(inode);

	spin_lock(&ei->i_block_reservation_lock);
	trace_ext4_da_update_reserve_space(inode, used, quota_claim);
	if (unlikely(used > ei->i_reserved_data_blocks)) {
		ext4_warning(inode->i_sb, "%s: ino %lu, used %d "
			 "with only %d reserved data blocks",
			 __func__, inode->i_ino, used,
			 ei->i_reserved_data_blocks);
		WARN_ON(1);
		used = ei->i_reserved_data_blocks;
	}

	
	ei->i_reserved_data_blocks -= used;
	percpu_counter_sub(&sbi->s_dirtyclusters_counter, used);

	spin_unlock(&EXT4_I(inode)->i_block_reservation_lock);

	
	if (quota_claim)
		dquot_claim_block(inode, EXT4_C2B(sbi, used));
	else {
		dquot_release_reservation_block(inode, EXT4_C2B(sbi, used));
	}

	if ((ei->i_reserved_data_blocks == 0) &&
	    (atomic_read(&inode->i_writecount) == 0))
		ext4_discard_preallocations(inode);
}

static int __check_block_validity(struct inode *inode, const char *func,
				unsigned int line,
				struct ext4_map_blocks *map)
{
	if (!ext4_data_block_valid(EXT4_SB(inode->i_sb), map->m_pblk,
				   map->m_len)) {
		ext4_error_inode(inode, func, line, map->m_pblk,
				 "lblock %lu mapped to illegal pblock "
				 "(length %d)", (unsigned long) map->m_lblk,
				 map->m_len);
		return -EIO;
	}
	return 0;
}

#define check_block_validity(inode, map)	\
	__check_block_validity((inode), __func__, __LINE__, (map))

#ifdef ES_AGGRESSIVE_TEST
static void ext4_map_blocks_es_recheck(handle_t *handle,
				       struct inode *inode,
				       struct ext4_map_blocks *es_map,
				       struct ext4_map_blocks *map,
				       int flags)
{
	int retval;

	map->m_flags = 0;
	/*
	 * There is a race window that the result is not the same.
	 * e.g. xfstests #223 when dioread_nolock enables.  The reason
	 * is that we lookup a block mapping in extent status tree with
	 * out taking i_data_sem.  So at the time the unwritten extent
	 * could be converted.
	 */
	if (!(flags & EXT4_GET_BLOCKS_NO_LOCK))
		down_read(&EXT4_I(inode)->i_data_sem);
	if (ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS)) {
		retval = ext4_ext_map_blocks(handle, inode, map, flags &
					     EXT4_GET_BLOCKS_KEEP_SIZE);
	} else {
		retval = ext4_ind_map_blocks(handle, inode, map, flags &
					     EXT4_GET_BLOCKS_KEEP_SIZE);
	}
	if (!(flags & EXT4_GET_BLOCKS_NO_LOCK))
		up_read((&EXT4_I(inode)->i_data_sem));
	map->m_flags &= ~(EXT4_MAP_FROM_CLUSTER | EXT4_MAP_BOUNDARY);

	if (es_map->m_lblk != map->m_lblk ||
	    es_map->m_flags != map->m_flags ||
	    es_map->m_pblk != map->m_pblk) {
		printk("ES cache assertion failed for inode: %lu "
		       "es_cached ex [%d/%d/%llu/%x] != "
		       "found ex [%d/%d/%llu/%x] retval %d flags %x\n",
		       inode->i_ino, es_map->m_lblk, es_map->m_len,
		       es_map->m_pblk, es_map->m_flags, map->m_lblk,
		       map->m_len, map->m_pblk, map->m_flags,
		       retval, flags);
	}
}
#endif 

/*
 * The ext4_map_blocks() function tries to look up the requested blocks,
 * and returns if the blocks are already mapped.
 *
 * Otherwise it takes the write lock of the i_data_sem and allocate blocks
 * and store the allocated blocks in the result buffer head and mark it
 * mapped.
 *
 * If file type is extents based, it will call ext4_ext_map_blocks(),
 * Otherwise, call with ext4_ind_map_blocks() to handle indirect mapping
 * based files
 *
 * On success, it returns the number of blocks being mapped or allocated.
 * if create==0 and the blocks are pre-allocated and unwritten block,
 * the result buffer head is unmapped. If the create ==1, it will make sure
 * the buffer head is mapped.
 *
 * It returns 0 if plain look up failed (blocks have not been allocated), in
 * that case, buffer head is unmapped
 *
 * It returns the error in case of allocation failure.
 */
int ext4_map_blocks(handle_t *handle, struct inode *inode,
		    struct ext4_map_blocks *map, int flags)
{
	struct extent_status es;
	int retval;
	int ret = 0;
#ifdef ES_AGGRESSIVE_TEST
	struct ext4_map_blocks orig_map;

	memcpy(&orig_map, map, sizeof(*map));
#endif

	map->m_flags = 0;
	ext_debug("ext4_map_blocks(): inode %lu, flag %d, max_blocks %u,"
		  "logical block %lu\n", inode->i_ino, flags, map->m_len,
		  (unsigned long) map->m_lblk);

	if (unlikely(map->m_len > INT_MAX))
		map->m_len = INT_MAX;

	
	if (unlikely(map->m_lblk >= EXT_MAX_BLOCKS))
		return -EIO;

	
	if (ext4_es_lookup_extent(inode, map->m_lblk, &es)) {
		ext4_es_lru_add(inode);
		if (ext4_es_is_written(&es) || ext4_es_is_unwritten(&es)) {
			map->m_pblk = ext4_es_pblock(&es) +
					map->m_lblk - es.es_lblk;
			map->m_flags |= ext4_es_is_written(&es) ?
					EXT4_MAP_MAPPED : EXT4_MAP_UNWRITTEN;
			retval = es.es_len - (map->m_lblk - es.es_lblk);
			if (retval > map->m_len)
				retval = map->m_len;
			map->m_len = retval;
		} else if (ext4_es_is_delayed(&es) || ext4_es_is_hole(&es)) {
			retval = 0;
		} else {
			BUG_ON(1);
		}
#ifdef ES_AGGRESSIVE_TEST
		ext4_map_blocks_es_recheck(handle, inode, map,
					   &orig_map, flags);
#endif
		goto found;
	}

	if (!(flags & EXT4_GET_BLOCKS_NO_LOCK))
		down_read(&EXT4_I(inode)->i_data_sem);
	if (ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS)) {
		retval = ext4_ext_map_blocks(handle, inode, map, flags &
					     EXT4_GET_BLOCKS_KEEP_SIZE);
	} else {
		retval = ext4_ind_map_blocks(handle, inode, map, flags &
					     EXT4_GET_BLOCKS_KEEP_SIZE);
	}
	if (retval > 0) {
		unsigned int status;

		if (unlikely(retval != map->m_len)) {
			ext4_warning(inode->i_sb,
				     "ES len assertion failed for inode "
				     "%lu: retval %d != map->m_len %d",
				     inode->i_ino, retval, map->m_len);
			WARN_ON(1);
		}

		status = map->m_flags & EXT4_MAP_UNWRITTEN ?
				EXTENT_STATUS_UNWRITTEN : EXTENT_STATUS_WRITTEN;
		if (!(flags & EXT4_GET_BLOCKS_DELALLOC_RESERVE) &&
		    !(status & EXTENT_STATUS_WRITTEN) &&
		    ext4_find_delalloc_range(inode, map->m_lblk,
					     map->m_lblk + map->m_len - 1))
			status |= EXTENT_STATUS_DELAYED;
		ret = ext4_es_insert_extent(inode, map->m_lblk,
					    map->m_len, map->m_pblk, status);
		if (ret < 0)
			retval = ret;
	}
	if (!(flags & EXT4_GET_BLOCKS_NO_LOCK))
		up_read((&EXT4_I(inode)->i_data_sem));

found:
	if (retval > 0 && map->m_flags & EXT4_MAP_MAPPED) {
		ret = check_block_validity(inode, map);
		if (ret != 0)
			return ret;
	}

	
	if ((flags & EXT4_GET_BLOCKS_CREATE) == 0)
		return retval;

	if (retval > 0 && map->m_flags & EXT4_MAP_MAPPED)
		/*
		 * If we need to convert extent to unwritten
		 * we continue and do the actual work in
		 * ext4_ext_map_blocks()
		 */
		if (!(flags & EXT4_GET_BLOCKS_CONVERT_UNWRITTEN))
			return retval;

	map->m_flags &= ~EXT4_MAP_FLAGS;

	/*
	 * New blocks allocate and/or writing to unwritten extent
	 * will possibly result in updating i_data, so we take
	 * the write lock of i_data_sem, and call get_block()
	 * with create == 1 flag.
	 */
	down_write(&EXT4_I(inode)->i_data_sem);

	if (ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS)) {
		retval = ext4_ext_map_blocks(handle, inode, map, flags);
	} else {
		retval = ext4_ind_map_blocks(handle, inode, map, flags);

		if (retval > 0 && map->m_flags & EXT4_MAP_NEW) {
			ext4_clear_inode_state(inode, EXT4_STATE_EXT_MIGRATE);
		}

		if ((retval > 0) &&
			(flags & EXT4_GET_BLOCKS_DELALLOC_RESERVE))
			ext4_da_update_reserve_space(inode, retval, 1);
	}

	if (retval > 0) {
		unsigned int status;

		if (unlikely(retval != map->m_len)) {
			ext4_warning(inode->i_sb,
				     "ES len assertion failed for inode "
				     "%lu: retval %d != map->m_len %d",
				     inode->i_ino, retval, map->m_len);
			WARN_ON(1);
		}

		if ((flags & EXT4_GET_BLOCKS_PRE_IO) &&
		    ext4_es_lookup_extent(inode, map->m_lblk, &es)) {
			if (ext4_es_is_written(&es))
				goto has_zeroout;
		}
		status = map->m_flags & EXT4_MAP_UNWRITTEN ?
				EXTENT_STATUS_UNWRITTEN : EXTENT_STATUS_WRITTEN;
		if (!(flags & EXT4_GET_BLOCKS_DELALLOC_RESERVE) &&
		    !(status & EXTENT_STATUS_WRITTEN) &&
		    ext4_find_delalloc_range(inode, map->m_lblk,
					     map->m_lblk + map->m_len - 1))
			status |= EXTENT_STATUS_DELAYED;
		ret = ext4_es_insert_extent(inode, map->m_lblk, map->m_len,
					    map->m_pblk, status);
		if (ret < 0)
			retval = ret;
	}

has_zeroout:
	up_write((&EXT4_I(inode)->i_data_sem));
	if (retval > 0 && map->m_flags & EXT4_MAP_MAPPED) {
		ret = check_block_validity(inode, map);
		if (ret != 0)
			return ret;
	}
	return retval;
}

#define DIO_MAX_BLOCKS 4096

static int _ext4_get_block(struct inode *inode, sector_t iblock,
			   struct buffer_head *bh, int flags)
{
	handle_t *handle = ext4_journal_current_handle();
	struct ext4_map_blocks map;
	int ret = 0, started = 0;
	int dio_credits;

	if (ext4_has_inline_data(inode))
		return -ERANGE;

	map.m_lblk = iblock;
	map.m_len = bh->b_size >> inode->i_blkbits;

	if (flags && !(flags & EXT4_GET_BLOCKS_NO_LOCK) && !handle) {
		
		if (map.m_len > DIO_MAX_BLOCKS)
			map.m_len = DIO_MAX_BLOCKS;
		dio_credits = ext4_chunk_trans_blocks(inode, map.m_len);
		handle = ext4_journal_start(inode, EXT4_HT_MAP_BLOCKS,
					    dio_credits);
		if (IS_ERR(handle)) {
			ret = PTR_ERR(handle);
			return ret;
		}
		started = 1;
	}

	ret = ext4_map_blocks(handle, inode, &map, flags);
	if (ret > 0) {
		ext4_io_end_t *io_end = ext4_inode_aio(inode);

		map_bh(bh, inode->i_sb, map.m_pblk);
		bh->b_state = (bh->b_state & ~EXT4_MAP_FLAGS) | map.m_flags;
		if (io_end && io_end->flag & EXT4_IO_END_UNWRITTEN)
			set_buffer_defer_completion(bh);
		bh->b_size = inode->i_sb->s_blocksize * map.m_len;
		ret = 0;
	}
	if (started)
		ext4_journal_stop(handle);
	return ret;
}

int ext4_get_block(struct inode *inode, sector_t iblock,
		   struct buffer_head *bh, int create)
{
	return _ext4_get_block(inode, iblock, bh,
			       create ? EXT4_GET_BLOCKS_CREATE : 0);
}

struct buffer_head *ext4_getblk(handle_t *handle, struct inode *inode,
				ext4_lblk_t block, int create)
{
	struct ext4_map_blocks map;
	struct buffer_head *bh;
	int err;

	J_ASSERT(handle != NULL || create == 0);

	map.m_lblk = block;
	map.m_len = 1;
	err = ext4_map_blocks(handle, inode, &map,
			      create ? EXT4_GET_BLOCKS_CREATE : 0);

	if (err == 0)
		return create ? ERR_PTR(-ENOSPC) : NULL;
	if (err < 0)
		return ERR_PTR(err);

	bh = sb_getblk(inode->i_sb, map.m_pblk);
	if (unlikely(!bh))
		return ERR_PTR(-ENOMEM);
	if (map.m_flags & EXT4_MAP_NEW) {
		J_ASSERT(create != 0);
		J_ASSERT(handle != NULL);

		lock_buffer(bh);
		BUFFER_TRACE(bh, "call get_create_access");
		err = ext4_journal_get_create_access(handle, bh);
		if (unlikely(err)) {
			unlock_buffer(bh);
			goto errout;
		}
		if (!buffer_uptodate(bh)) {
			memset(bh->b_data, 0, inode->i_sb->s_blocksize);
			set_buffer_uptodate(bh);
		}
		unlock_buffer(bh);
		BUFFER_TRACE(bh, "call ext4_handle_dirty_metadata");
		err = ext4_handle_dirty_metadata(handle, inode, bh);
		if (unlikely(err))
			goto errout;
	} else
		BUFFER_TRACE(bh, "not a new buffer");
	return bh;
errout:
	brelse(bh);
	return ERR_PTR(err);
}

struct buffer_head *ext4_bread(handle_t *handle, struct inode *inode,
			       ext4_lblk_t block, int create)
{
	struct buffer_head *bh;

	bh = ext4_getblk(handle, inode, block, create);
	if (IS_ERR(bh))
		return bh;
	if (!bh || buffer_uptodate(bh))
		return bh;
	ll_rw_block(READ | REQ_META | REQ_PRIO, 1, &bh);
	wait_on_buffer(bh);
	if (buffer_uptodate(bh))
		return bh;
	put_bh(bh);
	return ERR_PTR(-EIO);
}

int ext4_walk_page_buffers(handle_t *handle,
			   struct buffer_head *head,
			   unsigned from,
			   unsigned to,
			   int *partial,
			   int (*fn)(handle_t *handle,
				     struct buffer_head *bh))
{
	struct buffer_head *bh;
	unsigned block_start, block_end;
	unsigned blocksize = head->b_size;
	int err, ret = 0;
	struct buffer_head *next;

	for (bh = head, block_start = 0;
	     ret == 0 && (bh != head || !block_start);
	     block_start = block_end, bh = next) {
		next = bh->b_this_page;
		block_end = block_start + blocksize;
		if (block_end <= from || block_start >= to) {
			if (partial && !buffer_uptodate(bh))
				*partial = 1;
			continue;
		}
		err = (*fn)(handle, bh);
		if (!ret)
			ret = err;
	}
	return ret;
}

int do_journal_get_write_access(handle_t *handle,
				struct buffer_head *bh)
{
	int dirty = buffer_dirty(bh);
	int ret;

	if (!buffer_mapped(bh) || buffer_freed(bh))
		return 0;
	if (dirty)
		clear_buffer_dirty(bh);
	BUFFER_TRACE(bh, "get write access");
	ret = ext4_journal_get_write_access(handle, bh);
	if (!ret && dirty)
		ret = ext4_handle_dirty_metadata(handle, NULL, bh);
	return ret;
}

static int ext4_get_block_write_nolock(struct inode *inode, sector_t iblock,
		   struct buffer_head *bh_result, int create);
static int ext4_write_begin(struct file *file, struct address_space *mapping,
			    loff_t pos, unsigned len, unsigned flags,
			    struct page **pagep, void **fsdata)
{
	struct inode *inode = mapping->host;
	int ret, needed_blocks;
	handle_t *handle;
	int retries = 0;
	struct page *page;
	pgoff_t index;
	unsigned from, to;

	trace_ext4_write_begin(inode, pos, len, flags);
	needed_blocks = ext4_writepage_trans_blocks(inode) + 1;
	index = pos >> PAGE_CACHE_SHIFT;
	from = pos & (PAGE_CACHE_SIZE - 1);
	to = from + len;

	if (ext4_test_inode_state(inode, EXT4_STATE_MAY_INLINE_DATA)) {
		ret = ext4_try_to_write_inline_data(mapping, inode, pos, len,
						    flags, pagep);
		if (ret < 0)
			return ret;
		if (ret == 1)
			return 0;
	}

	/*
	 * grab_cache_page_write_begin() can take a long time if the
	 * system is thrashing due to memory pressure, or if the page
	 * is being written back.  So grab it first before we start
	 * the transaction handle.  This also allows us to allocate
	 * the page (if needed) without using GFP_NOFS.
	 */
retry_grab:
	page = grab_cache_page_write_begin(mapping, index, flags);
	if (!page)
		return -ENOMEM;
	unlock_page(page);

retry_journal:
	handle = ext4_journal_start(inode, EXT4_HT_WRITE_PAGE, needed_blocks);
	if (IS_ERR(handle)) {
		page_cache_release(page);
		return PTR_ERR(handle);
	}

	lock_page(page);
	if (page->mapping != mapping) {
		
		unlock_page(page);
		page_cache_release(page);
		ext4_journal_stop(handle);
		goto retry_grab;
	}
	
	wait_for_stable_page(page);

	if (ext4_should_dioread_nolock(inode))
		ret = __block_write_begin(page, pos, len, ext4_get_block_write);
	else
		ret = __block_write_begin(page, pos, len, ext4_get_block);

	if (!ret && ext4_should_journal_data(inode)) {
		ret = ext4_walk_page_buffers(handle, page_buffers(page),
					     from, to, NULL,
					     do_journal_get_write_access);
	}

	if (ret) {
		unlock_page(page);
		if (pos + len > inode->i_size && ext4_can_truncate(inode))
			ext4_orphan_add(handle, inode);

		ext4_journal_stop(handle);
		if (pos + len > inode->i_size) {
			ext4_truncate_failed_write(inode);
			if (inode->i_nlink)
				ext4_orphan_del(NULL, inode);
		}

		if (ret == -ENOSPC &&
		    ext4_should_retry_alloc(inode->i_sb, &retries))
			goto retry_journal;
		page_cache_release(page);
		return ret;
	}
	*pagep = page;
	return ret;
}

static int write_end_fn(handle_t *handle, struct buffer_head *bh)
{
	int ret;
	if (!buffer_mapped(bh) || buffer_freed(bh))
		return 0;
	set_buffer_uptodate(bh);
	ret = ext4_handle_dirty_metadata(handle, NULL, bh);
	clear_buffer_meta(bh);
	clear_buffer_prio(bh);
	return ret;
}

static int ext4_write_end(struct file *file,
			  struct address_space *mapping,
			  loff_t pos, unsigned len, unsigned copied,
			  struct page *page, void *fsdata)
{
	handle_t *handle = ext4_journal_current_handle();
	struct inode *inode = mapping->host;
	int ret = 0, ret2;
	int i_size_changed = 0;

	trace_ext4_write_end(inode, pos, len, copied);
	if (ext4_test_inode_state(inode, EXT4_STATE_ORDERED_MODE)) {
		ret = ext4_jbd2_file_inode(handle, inode);
		if (ret) {
			unlock_page(page);
			page_cache_release(page);
			goto errout;
		}
	}

	if (ext4_has_inline_data(inode)) {
		ret = ext4_write_inline_data_end(inode, pos, len,
						 copied, page);
		if (ret < 0)
			goto errout;
		copied = ret;
	} else
		copied = block_write_end(file, mapping, pos,
					 len, copied, page, fsdata);
	i_size_changed = ext4_update_inode_size(inode, pos + copied);
	unlock_page(page);
	page_cache_release(page);

	if (i_size_changed)
		ext4_mark_inode_dirty(handle, inode);

	if (pos + len > inode->i_size && ext4_can_truncate(inode))
		ext4_orphan_add(handle, inode);
errout:
	ret2 = ext4_journal_stop(handle);
	if (!ret)
		ret = ret2;

	if (pos + len > inode->i_size) {
		ext4_truncate_failed_write(inode);
		if (inode->i_nlink)
			ext4_orphan_del(NULL, inode);
	}

	return ret ? ret : copied;
}

static int ext4_journalled_write_end(struct file *file,
				     struct address_space *mapping,
				     loff_t pos, unsigned len, unsigned copied,
				     struct page *page, void *fsdata)
{
	handle_t *handle = ext4_journal_current_handle();
	struct inode *inode = mapping->host;
	int ret = 0, ret2;
	int partial = 0;
	unsigned from, to;
	int size_changed = 0;

	trace_ext4_journalled_write_end(inode, pos, len, copied);
	from = pos & (PAGE_CACHE_SIZE - 1);
	to = from + len;

	BUG_ON(!ext4_handle_valid(handle));

	if (ext4_has_inline_data(inode))
		copied = ext4_write_inline_data_end(inode, pos, len,
						    copied, page);
	else {
		if (copied < len) {
			if (!PageUptodate(page))
				copied = 0;
			page_zero_new_buffers(page, from+copied, to);
		}

		ret = ext4_walk_page_buffers(handle, page_buffers(page), from,
					     to, &partial, write_end_fn);
		if (!partial)
			SetPageUptodate(page);
	}
	size_changed = ext4_update_inode_size(inode, pos + copied);
	ext4_set_inode_state(inode, EXT4_STATE_JDATA);
	EXT4_I(inode)->i_datasync_tid = handle->h_transaction->t_tid;
	unlock_page(page);
	page_cache_release(page);

	if (size_changed) {
		ret2 = ext4_mark_inode_dirty(handle, inode);
		if (!ret)
			ret = ret2;
	}

	if (pos + len > inode->i_size && ext4_can_truncate(inode))
		ext4_orphan_add(handle, inode);

	ret2 = ext4_journal_stop(handle);
	if (!ret)
		ret = ret2;
	if (pos + len > inode->i_size) {
		ext4_truncate_failed_write(inode);
		if (inode->i_nlink)
			ext4_orphan_del(NULL, inode);
	}

	return ret ? ret : copied;
}

static int ext4_da_reserve_space(struct inode *inode, ext4_lblk_t lblock)
{
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	struct ext4_inode_info *ei = EXT4_I(inode);
	unsigned int md_needed;
	int ret;

	ret = dquot_reserve_block(inode, EXT4_C2B(sbi, 1));
	if (ret)
		return ret;

	spin_lock(&ei->i_block_reservation_lock);
	md_needed = 0;
	trace_ext4_da_reserve_space(inode, 0);

	if (ext4_claim_free_clusters(sbi, 1, 0)) {
		spin_unlock(&ei->i_block_reservation_lock);
		dquot_release_reservation_block(inode, EXT4_C2B(sbi, 1));
		return -ENOSPC;
	}
	ei->i_reserved_data_blocks++;
	spin_unlock(&ei->i_block_reservation_lock);

	return 0;       
}

static void ext4_da_release_space(struct inode *inode, int to_free)
{
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	struct ext4_inode_info *ei = EXT4_I(inode);

	if (!to_free)
		return;		

	spin_lock(&EXT4_I(inode)->i_block_reservation_lock);

	trace_ext4_da_release_space(inode, to_free);
	if (unlikely(to_free > ei->i_reserved_data_blocks)) {
		ext4_warning(inode->i_sb, "ext4_da_release_space: "
			 "ino %lu, to_free %d with only %d reserved "
			 "data blocks", inode->i_ino, to_free,
			 ei->i_reserved_data_blocks);
		WARN_ON(1);
		to_free = ei->i_reserved_data_blocks;
	}
	ei->i_reserved_data_blocks -= to_free;

	
	percpu_counter_sub(&sbi->s_dirtyclusters_counter, to_free);

	spin_unlock(&EXT4_I(inode)->i_block_reservation_lock);

	dquot_release_reservation_block(inode, EXT4_C2B(sbi, to_free));
}

static void ext4_da_page_release_reservation(struct page *page,
					     unsigned int offset,
					     unsigned int length)
{
	int to_release = 0, contiguous_blks = 0;
	struct buffer_head *head, *bh;
	unsigned int curr_off = 0;
	struct inode *inode = page->mapping->host;
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	unsigned int stop = offset + length;
	int num_clusters;
	ext4_fsblk_t lblk;

	BUG_ON(stop > PAGE_CACHE_SIZE || stop < length);

	head = page_buffers(page);
	bh = head;
	do {
		unsigned int next_off = curr_off + bh->b_size;

		if (next_off > stop)
			break;

		if ((offset <= curr_off) && (buffer_delay(bh))) {
			to_release++;
			contiguous_blks++;
			clear_buffer_delay(bh);
		} else if (contiguous_blks) {
			lblk = page->index <<
			       (PAGE_CACHE_SHIFT - inode->i_blkbits);
			lblk += (curr_off >> inode->i_blkbits) -
				contiguous_blks;
			ext4_es_remove_extent(inode, lblk, contiguous_blks);
			contiguous_blks = 0;
		}
		curr_off = next_off;
	} while ((bh = bh->b_this_page) != head);

	if (contiguous_blks) {
		lblk = page->index << (PAGE_CACHE_SHIFT - inode->i_blkbits);
		lblk += (curr_off >> inode->i_blkbits) - contiguous_blks;
		ext4_es_remove_extent(inode, lblk, contiguous_blks);
	}

	num_clusters = EXT4_NUM_B2C(sbi, to_release);
	while (num_clusters > 0) {
		lblk = (page->index << (PAGE_CACHE_SHIFT - inode->i_blkbits)) +
			((num_clusters - 1) << sbi->s_cluster_bits);
		if (sbi->s_cluster_ratio == 1 ||
		    !ext4_find_delalloc_cluster(inode, lblk))
			ext4_da_release_space(inode, 1);

		num_clusters--;
	}
}


struct mpage_da_data {
	struct inode *inode;
	struct writeback_control *wbc;

	pgoff_t first_page;	
	pgoff_t next_page;	
	pgoff_t last_page;	
	/*
	 * Extent to map - this can be after first_page because that can be
	 * fully mapped. We somewhat abuse m_flags to store whether the extent
	 * is delalloc or unwritten.
	 */
	struct ext4_map_blocks map;
	struct ext4_io_submit io_submit;	
};

static void mpage_release_unused_pages(struct mpage_da_data *mpd,
				       bool invalidate)
{
	int nr_pages, i;
	pgoff_t index, end;
	struct pagevec pvec;
	struct inode *inode = mpd->inode;
	struct address_space *mapping = inode->i_mapping;

	
	if (mpd->first_page >= mpd->next_page)
		return;

	index = mpd->first_page;
	end   = mpd->next_page - 1;
	if (invalidate) {
		ext4_lblk_t start, last;
		start = index << (PAGE_CACHE_SHIFT - inode->i_blkbits);
		last = end << (PAGE_CACHE_SHIFT - inode->i_blkbits);
		ext4_es_remove_extent(inode, start, last - start + 1);
	}

	pagevec_init(&pvec, 0);
	while (index <= end) {
		nr_pages = pagevec_lookup(&pvec, mapping, index, PAGEVEC_SIZE);
		if (nr_pages == 0)
			break;
		for (i = 0; i < nr_pages; i++) {
			struct page *page = pvec.pages[i];
			if (page->index > end)
				break;
			BUG_ON(!PageLocked(page));
			BUG_ON(PageWriteback(page));
			if (invalidate) {
				block_invalidatepage(page, 0, PAGE_CACHE_SIZE);
				ClearPageUptodate(page);
			}
			unlock_page(page);
		}
		index = pvec.pages[nr_pages - 1]->index + 1;
		pagevec_release(&pvec);
	}
}

static void ext4_print_free_blocks(struct inode *inode)
{
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	struct super_block *sb = inode->i_sb;
	struct ext4_inode_info *ei = EXT4_I(inode);

	ext4_msg(sb, KERN_CRIT, "Total free blocks count %lld",
	       EXT4_C2B(EXT4_SB(inode->i_sb),
			ext4_count_free_clusters(sb)));
	ext4_msg(sb, KERN_CRIT, "Free/Dirty block details");
	ext4_msg(sb, KERN_CRIT, "free_blocks=%lld",
	       (long long) EXT4_C2B(EXT4_SB(sb),
		percpu_counter_sum(&sbi->s_freeclusters_counter)));
	ext4_msg(sb, KERN_CRIT, "dirty_blocks=%lld",
	       (long long) EXT4_C2B(EXT4_SB(sb),
		percpu_counter_sum(&sbi->s_dirtyclusters_counter)));
	ext4_msg(sb, KERN_CRIT, "Block reservation details");
	ext4_msg(sb, KERN_CRIT, "i_reserved_data_blocks=%u",
		 ei->i_reserved_data_blocks);
	return;
}

static int ext4_bh_delay_or_unwritten(handle_t *handle, struct buffer_head *bh)
{
	return (buffer_delay(bh) || buffer_unwritten(bh)) && buffer_dirty(bh);
}

static int ext4_da_map_blocks(struct inode *inode, sector_t iblock,
			      struct ext4_map_blocks *map,
			      struct buffer_head *bh)
{
	struct extent_status es;
	int retval;
	sector_t invalid_block = ~((sector_t) 0xffff);
#ifdef ES_AGGRESSIVE_TEST
	struct ext4_map_blocks orig_map;

	memcpy(&orig_map, map, sizeof(*map));
#endif

	if (invalid_block < ext4_blocks_count(EXT4_SB(inode->i_sb)->s_es))
		invalid_block = ~0;

	map->m_flags = 0;
	ext_debug("ext4_da_map_blocks(): inode %lu, max_blocks %u,"
		  "logical block %lu\n", inode->i_ino, map->m_len,
		  (unsigned long) map->m_lblk);

	
	if (ext4_es_lookup_extent(inode, iblock, &es)) {
		ext4_es_lru_add(inode);
		if (ext4_es_is_hole(&es)) {
			retval = 0;
			down_read(&EXT4_I(inode)->i_data_sem);
			goto add_delayed;
		}

		if (ext4_es_is_delayed(&es) && !ext4_es_is_unwritten(&es)) {
			map_bh(bh, inode->i_sb, invalid_block);
			set_buffer_new(bh);
			set_buffer_delay(bh);
			return 0;
		}

		map->m_pblk = ext4_es_pblock(&es) + iblock - es.es_lblk;
		retval = es.es_len - (iblock - es.es_lblk);
		if (retval > map->m_len)
			retval = map->m_len;
		map->m_len = retval;
		if (ext4_es_is_written(&es))
			map->m_flags |= EXT4_MAP_MAPPED;
		else if (ext4_es_is_unwritten(&es))
			map->m_flags |= EXT4_MAP_UNWRITTEN;
		else
			BUG_ON(1);

#ifdef ES_AGGRESSIVE_TEST
		ext4_map_blocks_es_recheck(NULL, inode, map, &orig_map, 0);
#endif
		return retval;
	}

	down_read(&EXT4_I(inode)->i_data_sem);
	if (ext4_has_inline_data(inode)) {
		if ((EXT4_SB(inode->i_sb)->s_cluster_ratio > 1) &&
		    ext4_find_delalloc_cluster(inode, map->m_lblk))
			map->m_flags |= EXT4_MAP_FROM_CLUSTER;
		retval = 0;
	} else if (ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS))
		retval = ext4_ext_map_blocks(NULL, inode, map,
					     EXT4_GET_BLOCKS_NO_PUT_HOLE);
	else
		retval = ext4_ind_map_blocks(NULL, inode, map,
					     EXT4_GET_BLOCKS_NO_PUT_HOLE);

add_delayed:
	if (retval == 0) {
		int ret;
		if (!(map->m_flags & EXT4_MAP_FROM_CLUSTER)) {
			ret = ext4_da_reserve_space(inode, iblock);
			if (ret) {
				
				retval = ret;
				goto out_unlock;
			}
		}

		ret = ext4_es_insert_extent(inode, map->m_lblk, map->m_len,
					    ~0, EXTENT_STATUS_DELAYED);
		if (ret) {
			retval = ret;
			goto out_unlock;
		}

		map->m_flags &= ~EXT4_MAP_FROM_CLUSTER;

		map_bh(bh, inode->i_sb, invalid_block);
		set_buffer_new(bh);
		set_buffer_delay(bh);
	} else if (retval > 0) {
		int ret;
		unsigned int status;

		if (unlikely(retval != map->m_len)) {
			ext4_warning(inode->i_sb,
				     "ES len assertion failed for inode "
				     "%lu: retval %d != map->m_len %d",
				     inode->i_ino, retval, map->m_len);
			WARN_ON(1);
		}

		status = map->m_flags & EXT4_MAP_UNWRITTEN ?
				EXTENT_STATUS_UNWRITTEN : EXTENT_STATUS_WRITTEN;
		ret = ext4_es_insert_extent(inode, map->m_lblk, map->m_len,
					    map->m_pblk, status);
		if (ret != 0)
			retval = ret;
	}

out_unlock:
	up_read((&EXT4_I(inode)->i_data_sem));

	return retval;
}

/*
 * This is a special get_block_t callback which is used by
 * ext4_da_write_begin().  It will either return mapped block or
 * reserve space for a single block.
 *
 * For delayed buffer_head we have BH_Mapped, BH_New, BH_Delay set.
 * We also have b_blocknr = -1 and b_bdev initialized properly
 *
 * For unwritten buffer_head we have BH_Mapped, BH_New, BH_Unwritten set.
 * We also have b_blocknr = physicalblock mapping unwritten extent and b_bdev
 * initialized properly.
 */
int ext4_da_get_block_prep(struct inode *inode, sector_t iblock,
			   struct buffer_head *bh, int create)
{
	struct ext4_map_blocks map;
	int ret = 0;

	BUG_ON(create == 0);
	BUG_ON(bh->b_size != inode->i_sb->s_blocksize);

	map.m_lblk = iblock;
	map.m_len = 1;

	ret = ext4_da_map_blocks(inode, iblock, &map, bh);
	if (ret <= 0)
		return ret;

	map_bh(bh, inode->i_sb, map.m_pblk);
	bh->b_state = (bh->b_state & ~EXT4_MAP_FLAGS) | map.m_flags;

	if (buffer_unwritten(bh)) {
		/* A delayed write to unwritten bh should be marked
		 * new and mapped.  Mapped ensures that we don't do
		 * get_block multiple times when we write to the same
		 * offset and new ensures that we do proper zero out
		 * for partial write.
		 */
		set_buffer_new(bh);
		set_buffer_mapped(bh);
	}
	return 0;
}

static int bget_one(handle_t *handle, struct buffer_head *bh)
{
	get_bh(bh);
	return 0;
}

static int bput_one(handle_t *handle, struct buffer_head *bh)
{
	put_bh(bh);
	return 0;
}

static int __ext4_journalled_writepage(struct page *page,
				       unsigned int len)
{
	struct address_space *mapping = page->mapping;
	struct inode *inode = mapping->host;
	struct buffer_head *page_bufs = NULL;
	handle_t *handle = NULL;
	int ret = 0, err = 0;
	int inline_data = ext4_has_inline_data(inode);
	struct buffer_head *inode_bh = NULL;

	ClearPageChecked(page);

	if (inline_data) {
		BUG_ON(page->index != 0);
		BUG_ON(len > ext4_get_max_inline_size(inode));
		inode_bh = ext4_journalled_write_inline_data(inode, len, page);
		if (inode_bh == NULL)
			goto out;
	} else {
		page_bufs = page_buffers(page);
		if (!page_bufs) {
			BUG();
			goto out;
		}
		ext4_walk_page_buffers(handle, page_bufs, 0, len,
				       NULL, bget_one);
	}
	get_page(page);
	unlock_page(page);

	handle = ext4_journal_start(inode, EXT4_HT_WRITE_PAGE,
				    ext4_writepage_trans_blocks(inode));
	if (IS_ERR(handle)) {
		ret = PTR_ERR(handle);
		put_page(page);
		goto out_no_pagelock;
	}
	BUG_ON(!ext4_handle_valid(handle));

	lock_page(page);
	put_page(page);
	if (page->mapping != mapping) {
		
		ext4_journal_stop(handle);
		ret = 0;
		goto out;
	}

	if (inline_data) {
		BUFFER_TRACE(inode_bh, "get write access");
		ret = ext4_journal_get_write_access(handle, inode_bh);

		err = ext4_handle_dirty_metadata(handle, inode, inode_bh);

	} else {
		ret = ext4_walk_page_buffers(handle, page_bufs, 0, len, NULL,
					     do_journal_get_write_access);

		err = ext4_walk_page_buffers(handle, page_bufs, 0, len, NULL,
					     write_end_fn);
	}
	if (ret == 0)
		ret = err;
	EXT4_I(inode)->i_datasync_tid = handle->h_transaction->t_tid;
	err = ext4_journal_stop(handle);
	if (!ret)
		ret = err;

	if (!ext4_has_inline_data(inode))
		ext4_walk_page_buffers(NULL, page_bufs, 0, len,
				       NULL, bput_one);
	ext4_set_inode_state(inode, EXT4_STATE_JDATA);
out:
	unlock_page(page);
out_no_pagelock:
	brelse(inode_bh);
	return ret;
}

/*
 * Note that we don't need to start a transaction unless we're journaling data
 * because we should have holes filled from ext4_page_mkwrite(). We even don't
 * need to file the inode to the transaction's list in ordered mode because if
 * we are writing back data added by write(), the inode is already there and if
 * we are writing back data modified via mmap(), no one guarantees in which
 * transaction the data will hit the disk. In case we are journaling data, we
 * cannot start transaction directly because transaction start ranks above page
 * lock so we have to do some magic.
 *
 * This function can get called via...
 *   - ext4_writepages after taking page lock (have journal handle)
 *   - journal_submit_inode_data_buffers (no journal handle)
 *   - shrink_page_list via the kswapd/direct reclaim (no journal handle)
 *   - grab_page_cache when doing write_begin (have journal handle)
 *
 * We don't do any block allocation in this function. If we have page with
 * multiple blocks we need to write those buffer_heads that are mapped. This
 * is important for mmaped based write. So if we do with blocksize 1K
 * truncate(f, 1024);
 * a = mmap(f, 0, 4096);
 * a[0] = 'a';
 * truncate(f, 4096);
 * we have in the page first buffer_head mapped via page_mkwrite call back
 * but other buffer_heads would be unmapped but dirty (dirty done via the
 * do_wp_page). So writepage should write the first block. If we modify
 * the mmap area beyond 1024 we will again get a page_fault and the
 * page_mkwrite callback will do the block allocation and mark the
 * buffer_heads mapped.
 *
 * We redirty the page if we have any buffer_heads that is either delay or
 * unwritten in the page.
 *
 * We can get recursively called as show below.
 *
 *	ext4_writepage() -> kmalloc() -> __alloc_pages() -> page_launder() ->
 *		ext4_writepage()
 *
 * But since we don't do any block allocation we should not deadlock.
 * Page also have the dirty flag cleared so we don't get recurive page_lock.
 */
static int ext4_writepage(struct page *page,
			  struct writeback_control *wbc)
{
	int ret = 0;
	loff_t size;
	unsigned int len;
	struct buffer_head *page_bufs = NULL;
	struct inode *inode = page->mapping->host;
	struct ext4_io_submit io_submit;
	bool keep_towrite = false;

	trace_ext4_writepage(page);
	size = i_size_read(inode);
	if (page->index == size >> PAGE_CACHE_SHIFT)
		len = size & ~PAGE_CACHE_MASK;
	else
		len = PAGE_CACHE_SIZE;

	page_bufs = page_buffers(page);
	if (ext4_walk_page_buffers(NULL, page_bufs, 0, len, NULL,
				   ext4_bh_delay_or_unwritten)) {
		redirty_page_for_writepage(wbc, page);
		if (current->flags & PF_MEMALLOC) {
			WARN_ON_ONCE((current->flags & (PF_MEMALLOC|PF_KSWAPD))
							== PF_MEMALLOC);
			unlock_page(page);
			return 0;
		}
		keep_towrite = true;
	}

	if (PageChecked(page) && ext4_should_journal_data(inode))
		return __ext4_journalled_writepage(page, len);

	ext4_io_submit_init(&io_submit, wbc);
	io_submit.io_end = ext4_init_io_end(inode, GFP_NOFS);
	if (!io_submit.io_end) {
		redirty_page_for_writepage(wbc, page);
		unlock_page(page);
		return -ENOMEM;
	}
	ret = ext4_bio_write_page(&io_submit, page, len, wbc, keep_towrite);
	ext4_io_submit(&io_submit);
	
	ext4_put_io_end_defer(io_submit.io_end);
	return ret;
}

static int mpage_submit_page(struct mpage_da_data *mpd, struct page *page)
{
	int len;
	loff_t size = i_size_read(mpd->inode);
	int err;

	BUG_ON(page->index != mpd->first_page);
	if (page->index == size >> PAGE_CACHE_SHIFT)
		len = size & ~PAGE_CACHE_MASK;
	else
		len = PAGE_CACHE_SIZE;
	clear_page_dirty_for_io(page);
	err = ext4_bio_write_page(&mpd->io_submit, page, len, mpd->wbc, false);
	if (!err)
		mpd->wbc->nr_to_write--;
	mpd->first_page++;

	return err;
}

#define BH_FLAGS ((1 << BH_Unwritten) | (1 << BH_Delay))

#define MAX_WRITEPAGES_EXTENT_LEN 2048

static bool mpage_add_bh_to_extent(struct mpage_da_data *mpd, ext4_lblk_t lblk,
				   struct buffer_head *bh)
{
	struct ext4_map_blocks *map = &mpd->map;

	
	if (!buffer_dirty(bh) || !buffer_mapped(bh) ||
	    (!buffer_delay(bh) && !buffer_unwritten(bh))) {
		
		if (map->m_len == 0)
			return true;
		return false;
	}

	
	if (map->m_len == 0) {
		map->m_lblk = lblk;
		map->m_len = 1;
		map->m_flags = bh->b_state & BH_FLAGS;
		return true;
	}

	
	if (map->m_len >= MAX_WRITEPAGES_EXTENT_LEN)
		return false;

	
	if (lblk == map->m_lblk + map->m_len &&
	    (bh->b_state & BH_FLAGS) == map->m_flags) {
		map->m_len++;
		return true;
	}
	return false;
}

static int mpage_process_page_bufs(struct mpage_da_data *mpd,
				   struct buffer_head *head,
				   struct buffer_head *bh,
				   ext4_lblk_t lblk)
{
	struct inode *inode = mpd->inode;
	int err;
	ext4_lblk_t blocks = (i_size_read(inode) + (1 << inode->i_blkbits) - 1)
							>> inode->i_blkbits;

	do {
		BUG_ON(buffer_locked(bh));

		if (lblk >= blocks || !mpage_add_bh_to_extent(mpd, lblk, bh)) {
			
			if (mpd->map.m_len)
				return 0;
			
			break;
		}
	} while (lblk++, (bh = bh->b_this_page) != head);
	
	if (mpd->map.m_len == 0) {
		err = mpage_submit_page(mpd, head->b_page);
		if (err < 0)
			return err;
	}
	return lblk < blocks;
}

/*
 * mpage_map_buffers - update buffers corresponding to changed extent and
 *		       submit fully mapped pages for IO
 *
 * @mpd - description of extent to map, on return next extent to map
 *
 * Scan buffers corresponding to changed extent (we expect corresponding pages
 * to be already locked) and update buffer state according to new extent state.
 * We map delalloc buffers to their physical location, clear unwritten bits,
 * and mark buffers as uninit when we perform writes to unwritten extents
 * and do extent conversion after IO is finished. If the last page is not fully
 * mapped, we update @map to the next extent in the last page that needs
 * mapping. Otherwise we submit the page for IO.
 */
static int mpage_map_and_submit_buffers(struct mpage_da_data *mpd)
{
	struct pagevec pvec;
	int nr_pages, i;
	struct inode *inode = mpd->inode;
	struct buffer_head *head, *bh;
	int bpp_bits = PAGE_CACHE_SHIFT - inode->i_blkbits;
	pgoff_t start, end;
	ext4_lblk_t lblk;
	sector_t pblock;
	int err;

	start = mpd->map.m_lblk >> bpp_bits;
	end = (mpd->map.m_lblk + mpd->map.m_len - 1) >> bpp_bits;
	lblk = start << bpp_bits;
	pblock = mpd->map.m_pblk;

	pagevec_init(&pvec, 0);
	while (start <= end) {
		nr_pages = pagevec_lookup(&pvec, inode->i_mapping, start,
					  PAGEVEC_SIZE);
		if (nr_pages == 0)
			break;
		for (i = 0; i < nr_pages; i++) {
			struct page *page = pvec.pages[i];

			if (page->index > end)
				break;
			
			BUG_ON(page->index != start);
			bh = head = page_buffers(page);
			do {
				if (lblk < mpd->map.m_lblk)
					continue;
				if (lblk >= mpd->map.m_lblk + mpd->map.m_len) {
					mpd->map.m_len = 0;
					mpd->map.m_flags = 0;
					err = mpage_process_page_bufs(mpd, head,
								      bh, lblk);
					pagevec_release(&pvec);
					if (err > 0)
						err = 0;
					return err;
				}
				if (buffer_delay(bh)) {
					clear_buffer_delay(bh);
					bh->b_blocknr = pblock++;
				}
				clear_buffer_unwritten(bh);
			} while (lblk++, (bh = bh->b_this_page) != head);

			mpd->io_submit.io_end->size += PAGE_CACHE_SIZE;
			
			err = mpage_submit_page(mpd, page);
			if (err < 0) {
				pagevec_release(&pvec);
				return err;
			}
			start++;
		}
		pagevec_release(&pvec);
	}
	
	mpd->map.m_len = 0;
	mpd->map.m_flags = 0;
	return 0;
}

static int mpage_map_one_extent(handle_t *handle, struct mpage_da_data *mpd)
{
	struct inode *inode = mpd->inode;
	struct ext4_map_blocks *map = &mpd->map;
	int get_blocks_flags;
	int err, dioread_nolock;

	trace_ext4_da_write_pages_extent(inode, map);
	/*
	 * Call ext4_map_blocks() to allocate any delayed allocation blocks, or
	 * to convert an unwritten extent to be initialized (in the case
	 * where we have written into one or more preallocated blocks).  It is
	 * possible that we're going to need more metadata blocks than
	 * previously reserved. However we must not fail because we're in
	 * writeback and there is nothing we can do about it so it might result
	 * in data loss.  So use reserved blocks to allocate metadata if
	 * possible.
	 *
	 * We pass in the magic EXT4_GET_BLOCKS_DELALLOC_RESERVE if
	 * the blocks in question are delalloc blocks.  This indicates
	 * that the blocks and quotas has already been checked when
	 * the data was copied into the page cache.
	 */
	get_blocks_flags = EXT4_GET_BLOCKS_CREATE |
			   EXT4_GET_BLOCKS_METADATA_NOFAIL;
	dioread_nolock = ext4_should_dioread_nolock(inode);
	if (dioread_nolock)
		get_blocks_flags |= EXT4_GET_BLOCKS_IO_CREATE_EXT;
	if (map->m_flags & (1 << BH_Delay))
		get_blocks_flags |= EXT4_GET_BLOCKS_DELALLOC_RESERVE;

	err = ext4_map_blocks(handle, inode, map, get_blocks_flags);
	if (err < 0)
		return err;
	if (dioread_nolock && (map->m_flags & EXT4_MAP_UNWRITTEN)) {
		if (!mpd->io_submit.io_end->handle &&
		    ext4_handle_valid(handle)) {
			mpd->io_submit.io_end->handle = handle->h_rsv_handle;
			handle->h_rsv_handle = NULL;
		}
		ext4_set_io_unwritten_flag(inode, mpd->io_submit.io_end);
	}

	BUG_ON(map->m_len == 0);
	if (map->m_flags & EXT4_MAP_NEW) {
		struct block_device *bdev = inode->i_sb->s_bdev;
		int i;

		for (i = 0; i < map->m_len; i++)
			unmap_underlying_metadata(bdev, map->m_pblk + i);
	}
	return 0;
}

/*
 * mpage_map_and_submit_extent - map extent starting at mpd->lblk of length
 *				 mpd->len and submit pages underlying it for IO
 *
 * @handle - handle for journal operations
 * @mpd - extent to map
 * @give_up_on_write - we set this to true iff there is a fatal error and there
 *                     is no hope of writing the data. The caller should discard
 *                     dirty pages to avoid infinite loops.
 *
 * The function maps extent starting at mpd->lblk of length mpd->len. If it is
 * delayed, blocks are allocated, if it is unwritten, we may need to convert
 * them to initialized or split the described range from larger unwritten
 * extent. Note that we need not map all the described range since allocation
 * can return less blocks or the range is covered by more unwritten extents. We
 * cannot map more because we are limited by reserved transaction credits. On
 * the other hand we always make sure that the last touched page is fully
 * mapped so that it can be written out (and thus forward progress is
 * guaranteed). After mapping we submit all mapped pages for IO.
 */
static int mpage_map_and_submit_extent(handle_t *handle,
				       struct mpage_da_data *mpd,
				       bool *give_up_on_write)
{
	struct inode *inode = mpd->inode;
	struct ext4_map_blocks *map = &mpd->map;
	int err;
	loff_t disksize;
	int progress = 0;

	mpd->io_submit.io_end->offset =
				((loff_t)map->m_lblk) << inode->i_blkbits;
	do {
		err = mpage_map_one_extent(handle, mpd);
		if (err < 0) {
			struct super_block *sb = inode->i_sb;

			if (EXT4_SB(sb)->s_mount_flags & EXT4_MF_FS_ABORTED)
				goto invalidate_dirty_pages;
			if ((err == -ENOMEM) ||
			    (err == -ENOSPC && ext4_count_free_clusters(sb))) {
				if (progress)
					goto update_disksize;
				return err;
			}
			ext4_msg(sb, KERN_CRIT,
				 "Delayed block allocation failed for "
				 "inode %lu at logical offset %llu with"
				 " max blocks %u with error %d",
				 inode->i_ino,
				 (unsigned long long)map->m_lblk,
				 (unsigned)map->m_len, -err);
			ext4_msg(sb, KERN_CRIT,
				 "This should not happen!! Data will "
				 "be lost\n");
			if (err == -ENOSPC)
				ext4_print_free_blocks(inode);
		invalidate_dirty_pages:
			*give_up_on_write = true;
			return err;
		}
		progress = 1;
		err = mpage_map_and_submit_buffers(mpd);
		if (err < 0)
			goto update_disksize;
	} while (map->m_len);

update_disksize:
	disksize = ((loff_t)mpd->first_page) << PAGE_CACHE_SHIFT;
	if (disksize > EXT4_I(inode)->i_disksize) {
		int err2;
		loff_t i_size;

		down_write(&EXT4_I(inode)->i_data_sem);
		i_size = i_size_read(inode);
		if (disksize > i_size)
			disksize = i_size;
		if (disksize > EXT4_I(inode)->i_disksize)
			EXT4_I(inode)->i_disksize = disksize;
		err2 = ext4_mark_inode_dirty(handle, inode);
		up_write(&EXT4_I(inode)->i_data_sem);
		if (err2)
			ext4_error(inode->i_sb,
				   "Failed to mark inode %lu dirty",
				   inode->i_ino);
		if (!err)
			err = err2;
	}
	return err;
}

static int ext4_da_writepages_trans_blocks(struct inode *inode)
{
	int bpp = ext4_journal_blocks_per_page(inode);

	return ext4_meta_trans_blocks(inode,
				MAX_WRITEPAGES_EXTENT_LEN + bpp - 1, bpp);
}

/*
 * mpage_prepare_extent_to_map - find & lock contiguous range of dirty pages
 * 				 and underlying extent to map
 *
 * @mpd - where to look for pages
 *
 * Walk dirty pages in the mapping. If they are fully mapped, submit them for
 * IO immediately. When we find a page which isn't mapped we start accumulating
 * extent of buffers underlying these pages that needs mapping (formed by
 * either delayed or unwritten buffers). We also lock the pages containing
 * these buffers. The extent found is returned in @mpd structure (starting at
 * mpd->lblk with length mpd->len blocks).
 *
 * Note that this function can attach bios to one io_end structure which are
 * neither logically nor physically contiguous. Although it may seem as an
 * unnecessary complication, it is actually inevitable in blocksize < pagesize
 * case as we need to track IO to all buffers underlying a page in one io_end.
 */
static int mpage_prepare_extent_to_map(struct mpage_da_data *mpd)
{
	struct address_space *mapping = mpd->inode->i_mapping;
	struct pagevec pvec;
	unsigned int nr_pages;
	long left = mpd->wbc->nr_to_write;
	pgoff_t index = mpd->first_page;
	pgoff_t end = mpd->last_page;
	int tag;
	int i, err = 0;
	int blkbits = mpd->inode->i_blkbits;
	ext4_lblk_t lblk;
	struct buffer_head *head;

	if (mpd->wbc->sync_mode == WB_SYNC_ALL || mpd->wbc->tagged_writepages)
		tag = PAGECACHE_TAG_TOWRITE;
	else
		tag = PAGECACHE_TAG_DIRTY;

	pagevec_init(&pvec, 0);
	mpd->map.m_len = 0;
	mpd->next_page = index;
	while (index <= end) {
		nr_pages = pagevec_lookup_tag(&pvec, mapping, &index, tag,
			      min(end - index, (pgoff_t)PAGEVEC_SIZE-1) + 1);
		if (nr_pages == 0)
			goto out;

		for (i = 0; i < nr_pages; i++) {
			struct page *page = pvec.pages[i];

			if (page->index > end)
				goto out;

			if (mpd->wbc->sync_mode == WB_SYNC_NONE && left <= 0)
				goto out;

			
			if (mpd->map.m_len > 0 && mpd->next_page != page->index)
				goto out;

			lock_page(page);
			if (!PageDirty(page) ||
			    (PageWriteback(page) &&
			     (mpd->wbc->sync_mode == WB_SYNC_NONE)) ||
			    unlikely(page->mapping != mapping)) {
				unlock_page(page);
				continue;
			}

			wait_on_page_writeback(page);
			BUG_ON(PageWriteback(page));

			if (mpd->map.m_len == 0)
				mpd->first_page = page->index;
			mpd->next_page = page->index + 1;
			
			lblk = ((ext4_lblk_t)page->index) <<
				(PAGE_CACHE_SHIFT - blkbits);
			head = page_buffers(page);
			err = mpage_process_page_bufs(mpd, head, head, lblk);
			if (err <= 0)
				goto out;
			err = 0;
			left--;
		}
		pagevec_release(&pvec);
		cond_resched();
	}
	return 0;
out:
	pagevec_release(&pvec);
	return err;
}

static int __writepage(struct page *page, struct writeback_control *wbc,
		       void *data)
{
	struct address_space *mapping = data;
	int ret = ext4_writepage(page, wbc);
	mapping_set_error(mapping, ret);
	return ret;
}

static int ext4_writepages(struct address_space *mapping,
			   struct writeback_control *wbc)
{
	pgoff_t	writeback_index = 0;
	long nr_to_write = wbc->nr_to_write;
	int range_whole = 0;
	int cycled = 1;
	handle_t *handle = NULL;
	struct mpage_da_data mpd;
	struct inode *inode = mapping->host;
	int needed_blocks, rsv_blocks = 0, ret = 0;
	struct ext4_sb_info *sbi = EXT4_SB(mapping->host->i_sb);
	bool done;
	struct blk_plug plug;
	bool give_up_on_write = false;

	trace_ext4_writepages(inode, wbc);

	if (!mapping->nrpages || !mapping_tagged(mapping, PAGECACHE_TAG_DIRTY))
		goto out_writepages;

	if (ext4_should_journal_data(inode)) {
		struct blk_plug plug;

		blk_start_plug(&plug);
		ret = write_cache_pages(mapping, wbc, __writepage, mapping);
		blk_finish_plug(&plug);
		goto out_writepages;
	}

	if (unlikely(sbi->s_mount_flags & EXT4_MF_FS_ABORTED)) {
		ret = -EROFS;
		goto out_writepages;
	}

	if (ext4_should_dioread_nolock(inode)) {
		rsv_blocks = 1 + (PAGE_CACHE_SIZE >> inode->i_blkbits);
	}

	if (ext4_has_inline_data(inode)) {
		
		handle = ext4_journal_start(inode, EXT4_HT_INODE, 1);
		if (IS_ERR(handle)) {
			ret = PTR_ERR(handle);
			goto out_writepages;
		}
		BUG_ON(ext4_test_inode_state(inode,
				EXT4_STATE_MAY_INLINE_DATA));
		ext4_destroy_inline_data(handle, inode);
		ext4_journal_stop(handle);
	}

	if (wbc->range_start == 0 && wbc->range_end == LLONG_MAX)
		range_whole = 1;

	if (wbc->range_cyclic) {
		writeback_index = mapping->writeback_index;
		if (writeback_index)
			cycled = 0;
		mpd.first_page = writeback_index;
		mpd.last_page = -1;
	} else {
		mpd.first_page = wbc->range_start >> PAGE_CACHE_SHIFT;
		mpd.last_page = wbc->range_end >> PAGE_CACHE_SHIFT;
	}

	mpd.inode = inode;
	mpd.wbc = wbc;
	ext4_io_submit_init(&mpd.io_submit, wbc);
retry:
	if (wbc->sync_mode == WB_SYNC_ALL || wbc->tagged_writepages)
		tag_pages_for_writeback(mapping, mpd.first_page, mpd.last_page);
	done = false;
	blk_start_plug(&plug);
	while (!done && mpd.first_page <= mpd.last_page) {
		
		mpd.io_submit.io_end = ext4_init_io_end(inode, GFP_KERNEL);
		if (!mpd.io_submit.io_end) {
			ret = -ENOMEM;
			break;
		}

		BUG_ON(ext4_should_journal_data(inode));
		needed_blocks = ext4_da_writepages_trans_blocks(inode);

		
		handle = ext4_journal_start_with_reserve(inode,
				EXT4_HT_WRITE_PAGE, needed_blocks, rsv_blocks);
		if (IS_ERR(handle)) {
			ret = PTR_ERR(handle);
			ext4_msg(inode->i_sb, KERN_CRIT, "%s: jbd2_start: "
			       "%ld pages, ino %lu; err %d", __func__,
				wbc->nr_to_write, inode->i_ino, ret);
			
			ext4_put_io_end(mpd.io_submit.io_end);
			break;
		}

		trace_ext4_da_write_pages(inode, mpd.first_page, mpd.wbc);
		ret = mpage_prepare_extent_to_map(&mpd);
		if (!ret) {
			if (mpd.map.m_len)
				ret = mpage_map_and_submit_extent(handle, &mpd,
					&give_up_on_write);
			else {
				done = true;
			}
		}
		ext4_journal_stop(handle);
		
		ext4_io_submit(&mpd.io_submit);
		
		mpage_release_unused_pages(&mpd, give_up_on_write);
		
		ext4_put_io_end(mpd.io_submit.io_end);

		if (ret == -ENOSPC && sbi->s_journal) {
			jbd2_journal_force_commit_nested(sbi->s_journal);
			ret = 0;
			continue;
		}
		
		if (ret)
			break;
	}
	blk_finish_plug(&plug);
	if (!ret && !cycled && wbc->nr_to_write > 0) {
		cycled = 1;
		mpd.last_page = writeback_index - 1;
		mpd.first_page = 0;
		goto retry;
	}

	
	if (wbc->range_cyclic || (range_whole && wbc->nr_to_write > 0))
		mapping->writeback_index = mpd.first_page;

out_writepages:
	trace_ext4_writepages_result(inode, wbc, ret,
				     nr_to_write - wbc->nr_to_write);
	return ret;
}

static int ext4_nonda_switch(struct super_block *sb)
{
	s64 free_clusters, dirty_clusters;
	struct ext4_sb_info *sbi = EXT4_SB(sb);

	free_clusters =
		percpu_counter_read_positive(&sbi->s_freeclusters_counter);
	dirty_clusters =
		percpu_counter_read_positive(&sbi->s_dirtyclusters_counter);
	if (dirty_clusters && (free_clusters < 2 * dirty_clusters))
		try_to_writeback_inodes_sb(sb, WB_REASON_FS_FREE_SPACE);

	if (2 * free_clusters < 3 * dirty_clusters ||
	    free_clusters < (dirty_clusters + EXT4_FREECLUSTERS_WATERMARK)) {
		return 1;
	}
	return 0;
}

static int ext4_da_write_credits(struct inode *inode, loff_t pos, unsigned len)
{
	if (likely(EXT4_HAS_RO_COMPAT_FEATURE(inode->i_sb,
				EXT4_FEATURE_RO_COMPAT_LARGE_FILE)))
		return 1;

	if (pos + len <= 0x7fffffffULL)
		return 1;

	
	return 2;
}

static int ext4_da_write_begin(struct file *file, struct address_space *mapping,
			       loff_t pos, unsigned len, unsigned flags,
			       struct page **pagep, void **fsdata)
{
	int ret, retries = 0;
	struct page *page;
	pgoff_t index;
	struct inode *inode = mapping->host;
	handle_t *handle;

	index = pos >> PAGE_CACHE_SHIFT;

	if (ext4_nonda_switch(inode->i_sb)) {
		*fsdata = (void *)FALL_BACK_TO_NONDELALLOC;
		return ext4_write_begin(file, mapping, pos,
					len, flags, pagep, fsdata);
	}
	*fsdata = (void *)0;
	trace_ext4_da_write_begin(inode, pos, len, flags);

	if (ext4_test_inode_state(inode, EXT4_STATE_MAY_INLINE_DATA)) {
		ret = ext4_da_write_inline_data_begin(mapping, inode,
						      pos, len, flags,
						      pagep, fsdata);
		if (ret < 0)
			return ret;
		if (ret == 1)
			return 0;
	}

	/*
	 * grab_cache_page_write_begin() can take a long time if the
	 * system is thrashing due to memory pressure, or if the page
	 * is being written back.  So grab it first before we start
	 * the transaction handle.  This also allows us to allocate
	 * the page (if needed) without using GFP_NOFS.
	 */
retry_grab:
	page = grab_cache_page_write_begin(mapping, index, flags);
	if (!page)
		return -ENOMEM;
	unlock_page(page);

retry_journal:
	handle = ext4_journal_start(inode, EXT4_HT_WRITE_PAGE,
				ext4_da_write_credits(inode, pos, len));
	if (IS_ERR(handle)) {
		page_cache_release(page);
		return PTR_ERR(handle);
	}

	lock_page(page);
	if (page->mapping != mapping) {
		
		unlock_page(page);
		page_cache_release(page);
		ext4_journal_stop(handle);
		goto retry_grab;
	}
	
	wait_for_stable_page(page);

	ret = __block_write_begin(page, pos, len, ext4_da_get_block_prep);
	if (ret < 0) {
		unlock_page(page);
		ext4_journal_stop(handle);
		if (pos + len > inode->i_size)
			ext4_truncate_failed_write(inode);

		if (ret == -ENOSPC &&
		    ext4_should_retry_alloc(inode->i_sb, &retries))
			goto retry_journal;

		page_cache_release(page);
		return ret;
	}

	*pagep = page;
	return ret;
}

static int ext4_da_should_update_i_disksize(struct page *page,
					    unsigned long offset)
{
	struct buffer_head *bh;
	struct inode *inode = page->mapping->host;
	unsigned int idx;
	int i;

	bh = page_buffers(page);
	idx = offset >> inode->i_blkbits;

	for (i = 0; i < idx; i++)
		bh = bh->b_this_page;

	if (!buffer_mapped(bh) || (buffer_delay(bh)) || buffer_unwritten(bh))
		return 0;
	return 1;
}

static int ext4_da_write_end(struct file *file,
			     struct address_space *mapping,
			     loff_t pos, unsigned len, unsigned copied,
			     struct page *page, void *fsdata)
{
	struct inode *inode = mapping->host;
	int ret = 0, ret2;
	handle_t *handle = ext4_journal_current_handle();
	loff_t new_i_size;
	unsigned long start, end;
	int write_mode = (int)(unsigned long)fsdata;

	if (write_mode == FALL_BACK_TO_NONDELALLOC)
		return ext4_write_end(file, mapping, pos,
				      len, copied, page, fsdata);

	trace_ext4_da_write_end(inode, pos, len, copied);
	start = pos & (PAGE_CACHE_SIZE - 1);
	end = start + copied - 1;

	new_i_size = pos + copied;
	if (copied && new_i_size > EXT4_I(inode)->i_disksize) {
		if (ext4_has_inline_data(inode) ||
		    ext4_da_should_update_i_disksize(page, end)) {
			ext4_update_i_disksize(inode, new_i_size);
			ext4_mark_inode_dirty(handle, inode);
		}
	}

	if (write_mode != CONVERT_INLINE_DATA &&
	    ext4_test_inode_state(inode, EXT4_STATE_MAY_INLINE_DATA) &&
	    ext4_has_inline_data(inode))
		ret2 = ext4_da_write_inline_data_end(inode, pos, len, copied,
						     page);
	else
		ret2 = generic_write_end(file, mapping, pos, len, copied,
							page, fsdata);

	copied = ret2;
	if (ret2 < 0)
		ret = ret2;
	ret2 = ext4_journal_stop(handle);
	if (!ret)
		ret = ret2;

	return ret ? ret : copied;
}

static void ext4_da_invalidatepage(struct page *page, unsigned int offset,
				   unsigned int length)
{
	BUG_ON(!PageLocked(page));
	if (!page_has_buffers(page))
		goto out;

	ext4_da_page_release_reservation(page, offset, length);

out:
	ext4_invalidatepage(page, offset, length);

	return;
}

int ext4_alloc_da_blocks(struct inode *inode)
{
	trace_ext4_alloc_da_blocks(inode);

	if (!EXT4_I(inode)->i_reserved_data_blocks)
		return 0;

	return filemap_flush(inode->i_mapping);
}

/*
 * bmap() is special.  It gets used by applications such as lilo and by
 * the swapper to find the on-disk block of a specific piece of data.
 *
 * Naturally, this is dangerous if the block concerned is still in the
 * journal.  If somebody makes a swapfile on an ext4 data-journaling
 * filesystem and enables swap, then they may get a nasty shock when the
 * data getting swapped to that swapfile suddenly gets overwritten by
 * the original zero's written out previously to the journal and
 * awaiting writeback in the kernel's buffer cache.
 *
 * So, if we see any bmap calls here on a modified, data-journaled file,
 * take extra steps to flush any blocks which might be in the cache.
 */
static sector_t ext4_bmap(struct address_space *mapping, sector_t block)
{
	struct inode *inode = mapping->host;
	journal_t *journal;
	int err;

	if (ext4_has_inline_data(inode))
		return 0;

	if (mapping_tagged(mapping, PAGECACHE_TAG_DIRTY) &&
			test_opt(inode->i_sb, DELALLOC)) {
		filemap_write_and_wait(mapping);
	}

	if (EXT4_JOURNAL(inode) &&
	    ext4_test_inode_state(inode, EXT4_STATE_JDATA)) {

		ext4_clear_inode_state(inode, EXT4_STATE_JDATA);
		journal = EXT4_JOURNAL(inode);
		jbd2_journal_lock_updates(journal);
		err = jbd2_journal_flush(journal);
		jbd2_journal_unlock_updates(journal);

		if (err)
			return 0;
	}

	return generic_block_bmap(mapping, block, ext4_get_block);
}

static int ext4_readpage(struct file *file, struct page *page)
{
	int ret = -EAGAIN;
	struct inode *inode = page->mapping->host;

	trace_ext4_readpage(page);

	if (ext4_has_inline_data(inode))
		ret = ext4_readpage_inline(inode, page);

	if (ret == -EAGAIN)
		return mpage_readpage(page, ext4_get_block);

	return ret;
}

static int
ext4_readpages(struct file *file, struct address_space *mapping,
		struct list_head *pages, unsigned nr_pages)
{
	struct inode *inode = mapping->host;

	
	if (ext4_has_inline_data(inode))
		return 0;

	return mpage_readpages(mapping, pages, nr_pages, ext4_get_block);
}

static void ext4_invalidatepage(struct page *page, unsigned int offset,
				unsigned int length)
{
	trace_ext4_invalidatepage(page, offset, length);

	
	WARN_ON(page_has_buffers(page) && buffer_jbd(page_buffers(page)));

	block_invalidatepage(page, offset, length);
}

static int __ext4_journalled_invalidatepage(struct page *page,
					    unsigned int offset,
					    unsigned int length)
{
	journal_t *journal = EXT4_JOURNAL(page->mapping->host);

	trace_ext4_journalled_invalidatepage(page, offset, length);

	if (offset == 0 && length == PAGE_CACHE_SIZE)
		ClearPageChecked(page);

	return jbd2_journal_invalidatepage(journal, page, offset, length);
}

static void ext4_journalled_invalidatepage(struct page *page,
					   unsigned int offset,
					   unsigned int length)
{
	WARN_ON(__ext4_journalled_invalidatepage(page, offset, length) < 0);
}

static int ext4_releasepage(struct page *page, gfp_t wait)
{
	journal_t *journal = EXT4_JOURNAL(page->mapping->host);

	trace_ext4_releasepage(page);

	
	if (PageChecked(page))
		return 0;
	if (journal)
		return jbd2_journal_try_to_free_buffers(journal, page, wait);
	else
		return try_to_free_buffers(page);
}

int ext4_get_block_write(struct inode *inode, sector_t iblock,
		   struct buffer_head *bh_result, int create)
{
	ext4_debug("ext4_get_block_write: inode %lu, create flag %d\n",
		   inode->i_ino, create);
	return _ext4_get_block(inode, iblock, bh_result,
			       EXT4_GET_BLOCKS_IO_CREATE_EXT);
}

static int ext4_get_block_write_nolock(struct inode *inode, sector_t iblock,
		   struct buffer_head *bh_result, int create)
{
	ext4_debug("ext4_get_block_write_nolock: inode %lu, create flag %d\n",
		   inode->i_ino, create);
	return _ext4_get_block(inode, iblock, bh_result,
			       EXT4_GET_BLOCKS_NO_LOCK);
}

static void ext4_end_io_dio(struct kiocb *iocb, loff_t offset,
			    ssize_t size, void *private)
{
        ext4_io_end_t *io_end = iocb->private;

	
	if (!io_end)
		return;

	ext_debug("ext4_end_io_dio(): io_end 0x%p "
		  "for inode %lu, iocb 0x%p, offset %llu, size %zd\n",
 		  iocb->private, io_end->inode->i_ino, iocb, offset,
		  size);

	iocb->private = NULL;
	io_end->offset = offset;
	io_end->size = size;
	ext4_put_io_end(io_end);
}

/*
 * For ext4 extent files, ext4 will do direct-io write to holes,
 * preallocated extents, and those write extend the file, no need to
 * fall back to buffered IO.
 *
 * For holes, we fallocate those blocks, mark them as unwritten
 * If those blocks were preallocated, we mark sure they are split, but
 * still keep the range to write as unwritten.
 *
 * The unwritten extents will be converted to written when DIO is completed.
 * For async direct IO, since the IO may still pending when return, we
 * set up an end_io call back function, which will do the conversion
 * when async direct IO completed.
 *
 * If the O_DIRECT write will extend the file then add this inode to the
 * orphan list.  So recovery will truncate it back to the original size
 * if the machine crashes during the write.
 *
 */
static ssize_t ext4_ext_direct_IO(int rw, struct kiocb *iocb,
			      struct iov_iter *iter, loff_t offset)
{
	struct file *file = iocb->ki_filp;
	struct inode *inode = file->f_mapping->host;
	ssize_t ret;
	size_t count = iov_iter_count(iter);
	int overwrite = 0;
	get_block_t *get_block_func = NULL;
	int dio_flags = 0;
	loff_t final_size = offset + count;
	ext4_io_end_t *io_end = NULL;

	
	if (rw != WRITE || final_size > inode->i_size)
		return ext4_ind_direct_IO(rw, iocb, iter, offset);

	BUG_ON(iocb->private == NULL);

	if (rw == WRITE)
		atomic_inc(&inode->i_dio_count);

	
	overwrite = *((int *)iocb->private);

	if (overwrite) {
		down_read(&EXT4_I(inode)->i_data_sem);
		mutex_unlock(&inode->i_mutex);
	}

	/*
	 * We could direct write to holes and fallocate.
	 *
	 * Allocated blocks to fill the hole are marked as
	 * unwritten to prevent parallel buffered read to expose
	 * the stale data before DIO complete the data IO.
	 *
	 * As to previously fallocated extents, ext4 get_block will
	 * just simply mark the buffer mapped but still keep the
	 * extents unwritten.
	 *
	 * For non AIO case, we will convert those unwritten extents
	 * to written after return back from blockdev_direct_IO.
	 *
	 * For async DIO, the conversion needs to be deferred when the
	 * IO is completed. The ext4 end_io callback function will be
	 * called to take care of the conversion work.  Here for async
	 * case, we allocate an io_end structure to hook to the iocb.
	 */
	iocb->private = NULL;
	if (overwrite) {
		get_block_func = ext4_get_block_write_nolock;
	} else {
		ext4_inode_aio_set(inode, NULL);
		if (!is_sync_kiocb(iocb)) {
			io_end = ext4_init_io_end(inode, GFP_NOFS);
			if (!io_end) {
				ret = -ENOMEM;
				goto retake_lock;
			}
			iocb->private = ext4_get_io_end(io_end);
			/*
			 * we save the io structure for current async direct
			 * IO, so that later ext4_map_blocks() could flag the
			 * io structure whether there is a unwritten extents
			 * needs to be converted when IO is completed.
			 */
			ext4_inode_aio_set(inode, io_end);
		}
		get_block_func = ext4_get_block_write;
		dio_flags = DIO_LOCKING;
	}
	ret = __blockdev_direct_IO(rw, iocb, inode,
				   inode->i_sb->s_bdev, iter,
				   offset,
				   get_block_func,
				   ext4_end_io_dio,
				   NULL,
				   dio_flags);

	if (io_end) {
		ext4_inode_aio_set(inode, NULL);
		ext4_put_io_end(io_end);
		if (ret <= 0 && ret != -EIOCBQUEUED && iocb->private) {
			WARN_ON(iocb->private != io_end);
			WARN_ON(io_end->flag & EXT4_IO_END_UNWRITTEN);
			ext4_put_io_end(io_end);
			iocb->private = NULL;
		}
	}
	if (ret > 0 && !overwrite && ext4_test_inode_state(inode,
						EXT4_STATE_DIO_UNWRITTEN)) {
		int err;
		err = ext4_convert_unwritten_extents(NULL, inode,
						     offset, ret);
		if (err < 0)
			ret = err;
		ext4_clear_inode_state(inode, EXT4_STATE_DIO_UNWRITTEN);
	}

retake_lock:
	if (rw == WRITE)
		inode_dio_done(inode);
	
	if (overwrite) {
		up_read(&EXT4_I(inode)->i_data_sem);
		mutex_lock(&inode->i_mutex);
	}

	return ret;
}

static ssize_t ext4_direct_IO(int rw, struct kiocb *iocb,
			      struct iov_iter *iter, loff_t offset)
{
	struct file *file = iocb->ki_filp;
	struct inode *inode = file->f_mapping->host;
	size_t count = iov_iter_count(iter);
	ssize_t ret;

	if (ext4_should_journal_data(inode))
		return 0;

	
	if (ext4_has_inline_data(inode))
		return 0;

	trace_ext4_direct_IO_enter(inode, offset, count, rw);
	if (ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS))
		ret = ext4_ext_direct_IO(rw, iocb, iter, offset);
	else
		ret = ext4_ind_direct_IO(rw, iocb, iter, offset);
	trace_ext4_direct_IO_exit(inode, offset, count, rw, ret);
	return ret;
}

static int ext4_journalled_set_page_dirty(struct page *page)
{
	SetPageChecked(page);
	return __set_page_dirty_nobuffers(page);
}

static const struct address_space_operations ext4_aops = {
	.readpage		= ext4_readpage,
	.readpages		= ext4_readpages,
	.writepage		= ext4_writepage,
	.writepages		= ext4_writepages,
	.write_begin		= ext4_write_begin,
	.write_end		= ext4_write_end,
	.bmap			= ext4_bmap,
	.invalidatepage		= ext4_invalidatepage,
	.releasepage		= ext4_releasepage,
	.direct_IO		= ext4_direct_IO,
	.migratepage		= buffer_migrate_page,
	.is_partially_uptodate  = block_is_partially_uptodate,
	.error_remove_page	= generic_error_remove_page,
};

static const struct address_space_operations ext4_journalled_aops = {
	.readpage		= ext4_readpage,
	.readpages		= ext4_readpages,
	.writepage		= ext4_writepage,
	.writepages		= ext4_writepages,
	.write_begin		= ext4_write_begin,
	.write_end		= ext4_journalled_write_end,
	.set_page_dirty		= ext4_journalled_set_page_dirty,
	.bmap			= ext4_bmap,
	.invalidatepage		= ext4_journalled_invalidatepage,
	.releasepage		= ext4_releasepage,
	.direct_IO		= ext4_direct_IO,
	.is_partially_uptodate  = block_is_partially_uptodate,
	.error_remove_page	= generic_error_remove_page,
};

static const struct address_space_operations ext4_da_aops = {
	.readpage		= ext4_readpage,
	.readpages		= ext4_readpages,
	.writepage		= ext4_writepage,
	.writepages		= ext4_writepages,
	.write_begin		= ext4_da_write_begin,
	.write_end		= ext4_da_write_end,
	.bmap			= ext4_bmap,
	.invalidatepage		= ext4_da_invalidatepage,
	.releasepage		= ext4_releasepage,
	.direct_IO		= ext4_direct_IO,
	.migratepage		= buffer_migrate_page,
	.is_partially_uptodate  = block_is_partially_uptodate,
	.error_remove_page	= generic_error_remove_page,
};

void ext4_set_aops(struct inode *inode)
{
	switch (ext4_inode_journal_mode(inode)) {
	case EXT4_INODE_ORDERED_DATA_MODE:
		ext4_set_inode_state(inode, EXT4_STATE_ORDERED_MODE);
		break;
	case EXT4_INODE_WRITEBACK_DATA_MODE:
		ext4_clear_inode_state(inode, EXT4_STATE_ORDERED_MODE);
		break;
	case EXT4_INODE_JOURNAL_DATA_MODE:
		inode->i_mapping->a_ops = &ext4_journalled_aops;
		return;
	default:
		BUG();
	}
	if (test_opt(inode->i_sb, DELALLOC))
		inode->i_mapping->a_ops = &ext4_da_aops;
	else
		inode->i_mapping->a_ops = &ext4_aops;
}

static int ext4_block_zero_page_range(handle_t *handle,
		struct address_space *mapping, loff_t from, loff_t length)
{
	ext4_fsblk_t index = from >> PAGE_CACHE_SHIFT;
	unsigned offset = from & (PAGE_CACHE_SIZE-1);
	unsigned blocksize, max, pos;
	ext4_lblk_t iblock;
	struct inode *inode = mapping->host;
	struct buffer_head *bh;
	struct page *page;
	int err = 0;

	page = find_or_create_page(mapping, from >> PAGE_CACHE_SHIFT,
				   mapping_gfp_mask(mapping) & ~__GFP_FS);
	if (!page)
		return -ENOMEM;

	blocksize = inode->i_sb->s_blocksize;
	max = blocksize - (offset & (blocksize - 1));

	if (length > max || length < 0)
		length = max;

	iblock = index << (PAGE_CACHE_SHIFT - inode->i_sb->s_blocksize_bits);

	if (!page_has_buffers(page))
		create_empty_buffers(page, blocksize, 0);

	
	bh = page_buffers(page);
	pos = blocksize;
	while (offset >= pos) {
		bh = bh->b_this_page;
		iblock++;
		pos += blocksize;
	}
	if (buffer_freed(bh)) {
		BUFFER_TRACE(bh, "freed: skip");
		goto unlock;
	}
	if (!buffer_mapped(bh)) {
		BUFFER_TRACE(bh, "unmapped");
		ext4_get_block(inode, iblock, bh, 0);
		
		if (!buffer_mapped(bh)) {
			BUFFER_TRACE(bh, "still unmapped");
			goto unlock;
		}
	}

	
	if (PageUptodate(page))
		set_buffer_uptodate(bh);

	if (!buffer_uptodate(bh)) {
		err = -EIO;
		ll_rw_block(READ, 1, &bh);
		wait_on_buffer(bh);
		
		if (!buffer_uptodate(bh))
			goto unlock;
	}
	if (ext4_should_journal_data(inode)) {
		BUFFER_TRACE(bh, "get write access");
		err = ext4_journal_get_write_access(handle, bh);
		if (err)
			goto unlock;
	}
	zero_user(page, offset, length);
	BUFFER_TRACE(bh, "zeroed end of block");

	if (ext4_should_journal_data(inode)) {
		err = ext4_handle_dirty_metadata(handle, inode, bh);
	} else {
		err = 0;
		mark_buffer_dirty(bh);
		if (ext4_test_inode_state(inode, EXT4_STATE_ORDERED_MODE))
			err = ext4_jbd2_file_inode(handle, inode);
	}

unlock:
	unlock_page(page);
	page_cache_release(page);
	return err;
}

static int ext4_block_truncate_page(handle_t *handle,
		struct address_space *mapping, loff_t from)
{
	unsigned offset = from & (PAGE_CACHE_SIZE-1);
	unsigned length;
	unsigned blocksize;
	struct inode *inode = mapping->host;

	blocksize = inode->i_sb->s_blocksize;
	length = blocksize - (offset & (blocksize - 1));

	return ext4_block_zero_page_range(handle, mapping, from, length);
}

int ext4_zero_partial_blocks(handle_t *handle, struct inode *inode,
			     loff_t lstart, loff_t length)
{
	struct super_block *sb = inode->i_sb;
	struct address_space *mapping = inode->i_mapping;
	unsigned partial_start, partial_end;
	ext4_fsblk_t start, end;
	loff_t byte_end = (lstart + length - 1);
	int err = 0;

	partial_start = lstart & (sb->s_blocksize - 1);
	partial_end = byte_end & (sb->s_blocksize - 1);

	start = lstart >> sb->s_blocksize_bits;
	end = byte_end >> sb->s_blocksize_bits;

	
	if (start == end &&
	    (partial_start || (partial_end != sb->s_blocksize - 1))) {
		err = ext4_block_zero_page_range(handle, mapping,
						 lstart, length);
		return err;
	}
	
	if (partial_start) {
		err = ext4_block_zero_page_range(handle, mapping,
						 lstart, sb->s_blocksize);
		if (err)
			return err;
	}
	
	if (partial_end != sb->s_blocksize - 1)
		err = ext4_block_zero_page_range(handle, mapping,
						 byte_end - partial_end,
						 partial_end + 1);
	return err;
}

int ext4_can_truncate(struct inode *inode)
{
	if (S_ISREG(inode->i_mode))
		return 1;
	if (S_ISDIR(inode->i_mode))
		return 1;
	if (S_ISLNK(inode->i_mode))
		return !ext4_inode_is_fast_symlink(inode);
	return 0;
}


int ext4_punch_hole(struct inode *inode, loff_t offset, loff_t length)
{
#if 0
	struct super_block *sb = inode->i_sb;
	ext4_lblk_t first_block, stop_block;
	struct address_space *mapping = inode->i_mapping;
	loff_t first_block_offset, last_block_offset;
	handle_t *handle;
	unsigned int credits;
	int ret = 0;

	if (!S_ISREG(inode->i_mode))
		return -EOPNOTSUPP;

	trace_ext4_punch_hole(inode, offset, length, 0);

	if (mapping->nrpages && mapping_tagged(mapping, PAGECACHE_TAG_DIRTY)) {
		ret = filemap_write_and_wait_range(mapping, offset,
						   offset + length - 1);
		if (ret)
			return ret;
	}

	mutex_lock(&inode->i_mutex);

	
	if (offset >= inode->i_size)
		goto out_mutex;

	if (offset + length > inode->i_size) {
		length = inode->i_size +
		   PAGE_CACHE_SIZE - (inode->i_size & (PAGE_CACHE_SIZE - 1)) -
		   offset;
	}

	if (offset & (sb->s_blocksize - 1) ||
	    (offset + length) & (sb->s_blocksize - 1)) {
		ret = ext4_inode_attach_jinode(inode);
		if (ret < 0)
			goto out_mutex;

	}

	first_block_offset = round_up(offset, sb->s_blocksize);
	last_block_offset = round_down((offset + length), sb->s_blocksize) - 1;

	
	if (last_block_offset > first_block_offset)
		truncate_pagecache_range(inode, first_block_offset,
					 last_block_offset);

	
	ext4_inode_block_unlocked_dio(inode);
	inode_dio_wait(inode);

	if (ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS))
		credits = ext4_writepage_trans_blocks(inode);
	else
		credits = ext4_blocks_for_truncate(inode);
	handle = ext4_journal_start(inode, EXT4_HT_TRUNCATE, credits);
	if (IS_ERR(handle)) {
		ret = PTR_ERR(handle);
		ext4_std_error(sb, ret);
		goto out_dio;
	}

	ret = ext4_zero_partial_blocks(handle, inode, offset,
				       length);
	if (ret)
		goto out_stop;

	first_block = (offset + sb->s_blocksize - 1) >>
		EXT4_BLOCK_SIZE_BITS(sb);
	stop_block = (offset + length) >> EXT4_BLOCK_SIZE_BITS(sb);

	
	if (first_block >= stop_block)
		goto out_stop;

	down_write(&EXT4_I(inode)->i_data_sem);
	ext4_discard_preallocations(inode);

	ret = ext4_es_remove_extent(inode, first_block,
				    stop_block - first_block);
	if (ret) {
		up_write(&EXT4_I(inode)->i_data_sem);
		goto out_stop;
	}

	if (ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS))
		ret = ext4_ext_remove_space(inode, first_block,
					    stop_block - 1);
	else
		ret = ext4_ind_remove_space(handle, inode, first_block,
					    stop_block);

	up_write(&EXT4_I(inode)->i_data_sem);
	if (IS_SYNC(inode))
		ext4_handle_sync(handle);

	
	if (last_block_offset > first_block_offset)
		truncate_pagecache_range(inode, first_block_offset,
					 last_block_offset);

	inode->i_mtime = inode->i_ctime = ext4_current_time(inode);
	ext4_mark_inode_dirty(handle, inode);
out_stop:
	ext4_journal_stop(handle);
out_dio:
	ext4_inode_resume_unlocked_dio(inode);
out_mutex:
	mutex_unlock(&inode->i_mutex);
	return ret;
#else
	return -EOPNOTSUPP;
#endif
}

int ext4_inode_attach_jinode(struct inode *inode)
{
	struct ext4_inode_info *ei = EXT4_I(inode);
	struct jbd2_inode *jinode;

	if (ei->jinode || !EXT4_SB(inode->i_sb)->s_journal)
		return 0;

	jinode = jbd2_alloc_inode(GFP_KERNEL);
	spin_lock(&inode->i_lock);
	if (!ei->jinode) {
		if (!jinode) {
			spin_unlock(&inode->i_lock);
			return -ENOMEM;
		}
		ei->jinode = jinode;
		jbd2_journal_init_jbd_inode(ei->jinode, inode);
		jinode = NULL;
	}
	spin_unlock(&inode->i_lock);
	if (unlikely(jinode != NULL))
		jbd2_free_inode(jinode);
	return 0;
}

void ext4_truncate(struct inode *inode)
{
	struct ext4_inode_info *ei = EXT4_I(inode);
	unsigned int credits;
	handle_t *handle;
	struct address_space *mapping = inode->i_mapping;

	if (!(inode->i_state & (I_NEW|I_FREEING)))
		WARN_ON(!mutex_is_locked(&inode->i_mutex));
	trace_ext4_truncate_enter(inode);

	if (!ext4_can_truncate(inode))
		return;

	ext4_clear_inode_flag(inode, EXT4_INODE_EOFBLOCKS);

	if (inode->i_size == 0 && !test_opt(inode->i_sb, NO_AUTO_DA_ALLOC))
		ext4_set_inode_state(inode, EXT4_STATE_DA_ALLOC_CLOSE);

	if (ext4_has_inline_data(inode)) {
		int has_inline = 1;

		ext4_inline_data_truncate(inode, &has_inline);
		if (has_inline)
			return;
	}

	
	if (inode->i_size & (inode->i_sb->s_blocksize - 1)) {
		if (ext4_inode_attach_jinode(inode) < 0)
			return;
	}

	if (ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS))
		credits = ext4_writepage_trans_blocks(inode);
	else
		credits = ext4_blocks_for_truncate(inode);

	handle = ext4_journal_start(inode, EXT4_HT_TRUNCATE, credits);
	if (IS_ERR(handle)) {
		ext4_std_error(inode->i_sb, PTR_ERR(handle));
		return;
	}

	if (inode->i_size & (inode->i_sb->s_blocksize - 1))
		ext4_block_truncate_page(handle, mapping, inode->i_size);

	if (ext4_orphan_add(handle, inode))
		goto out_stop;

	down_write(&EXT4_I(inode)->i_data_sem);

	ext4_discard_preallocations(inode);

	if (ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS))
		ext4_ext_truncate(handle, inode);
	else
		ext4_ind_truncate(handle, inode);

	up_write(&ei->i_data_sem);

	if (IS_SYNC(inode))
		ext4_handle_sync(handle);

out_stop:
	if (inode->i_nlink)
		ext4_orphan_del(handle, inode);

	inode->i_mtime = inode->i_ctime = ext4_current_time(inode);
	ext4_mark_inode_dirty(handle, inode);
	ext4_journal_stop(handle);

	trace_ext4_truncate_exit(inode);
}

static int __ext4_get_inode_loc(struct inode *inode,
				struct ext4_iloc *iloc, int in_mem)
{
	struct ext4_group_desc	*gdp;
	struct buffer_head	*bh;
	struct super_block	*sb = inode->i_sb;
	ext4_fsblk_t		block;
	int			inodes_per_block, inode_offset;

	iloc->bh = NULL;
	if (!ext4_valid_inum(sb, inode->i_ino))
		return -EIO;

	iloc->block_group = (inode->i_ino - 1) / EXT4_INODES_PER_GROUP(sb);
	gdp = ext4_get_group_desc(sb, iloc->block_group, NULL);
	if (!gdp)
		return -EIO;

	inodes_per_block = EXT4_SB(sb)->s_inodes_per_block;
	inode_offset = ((inode->i_ino - 1) %
			EXT4_INODES_PER_GROUP(sb));
	block = ext4_inode_table(sb, gdp) + (inode_offset / inodes_per_block);
	iloc->offset = (inode_offset % inodes_per_block) * EXT4_INODE_SIZE(sb);

	bh = sb_getblk(sb, block);
	if (unlikely(!bh))
		return -ENOMEM;
	if (!buffer_uptodate(bh)) {
		lock_buffer(bh);

		if (buffer_write_io_error(bh) && !buffer_uptodate(bh))
			set_buffer_uptodate(bh);

		if (buffer_uptodate(bh)) {
			
			unlock_buffer(bh);
			goto has_buffer;
		}

		if (in_mem) {
			struct buffer_head *bitmap_bh;
			int i, start;

			start = inode_offset & ~(inodes_per_block - 1);

			
			bitmap_bh = sb_getblk(sb, ext4_inode_bitmap(sb, gdp));
			if (unlikely(!bitmap_bh))
				goto make_io;

			if (!buffer_uptodate(bitmap_bh)) {
				brelse(bitmap_bh);
				goto make_io;
			}
			for (i = start; i < start + inodes_per_block; i++) {
				if (i == inode_offset)
					continue;
				if (ext4_test_bit(i, bitmap_bh->b_data))
					break;
			}
			brelse(bitmap_bh);
			if (i == start + inodes_per_block) {
				
				memset(bh->b_data, 0, bh->b_size);
				set_buffer_uptodate(bh);
				unlock_buffer(bh);
				goto has_buffer;
			}
		}

make_io:
		if (EXT4_SB(sb)->s_inode_readahead_blks) {
			ext4_fsblk_t b, end, table;
			unsigned num;
			__u32 ra_blks = EXT4_SB(sb)->s_inode_readahead_blks;

			table = ext4_inode_table(sb, gdp);
			
			b = block & ~((ext4_fsblk_t) ra_blks - 1);
			if (table > b)
				b = table;
			end = b + ra_blks;
			num = EXT4_INODES_PER_GROUP(sb);
			if (ext4_has_group_desc_csum(sb))
				num -= ext4_itable_unused_count(sb, gdp);
			table += num / inodes_per_block;
			if (end > table)
				end = table;
			while (b <= end)
				sb_breadahead(sb, b++);
		}

		trace_ext4_load_inode(inode);
		get_bh(bh);
		bh->b_end_io = end_buffer_read_sync;
		submit_bh(READ | REQ_META | REQ_PRIO, bh);
		wait_on_buffer(bh);
		if (!buffer_uptodate(bh)) {
			EXT4_ERROR_INODE_BLOCK(inode, block,
					       "unable to read itable block");
			brelse(bh);
			return -EIO;
		}
	}
has_buffer:
	iloc->bh = bh;
	return 0;
}

int ext4_get_inode_loc(struct inode *inode, struct ext4_iloc *iloc)
{
	
	return __ext4_get_inode_loc(inode, iloc,
		!ext4_test_inode_state(inode, EXT4_STATE_XATTR));
}

void ext4_set_inode_flags(struct inode *inode)
{
	unsigned int flags = EXT4_I(inode)->i_flags;
	unsigned int new_fl = 0;

	if (flags & EXT4_SYNC_FL)
		new_fl |= S_SYNC;
	if (flags & EXT4_APPEND_FL)
		new_fl |= S_APPEND;
	if (flags & EXT4_IMMUTABLE_FL)
		new_fl |= S_IMMUTABLE;
	if (flags & EXT4_NOATIME_FL)
		new_fl |= S_NOATIME;
	if (flags & EXT4_DIRSYNC_FL)
		new_fl |= S_DIRSYNC;
	inode_set_flags(inode, new_fl,
			S_SYNC|S_APPEND|S_IMMUTABLE|S_NOATIME|S_DIRSYNC);
}

void ext4_get_inode_flags(struct ext4_inode_info *ei)
{
	unsigned int vfs_fl;
	unsigned long old_fl, new_fl;

	do {
		vfs_fl = ei->vfs_inode.i_flags;
		old_fl = ei->i_flags;
		new_fl = old_fl & ~(EXT4_SYNC_FL|EXT4_APPEND_FL|
				EXT4_IMMUTABLE_FL|EXT4_NOATIME_FL|
				EXT4_DIRSYNC_FL);
		if (vfs_fl & S_SYNC)
			new_fl |= EXT4_SYNC_FL;
		if (vfs_fl & S_APPEND)
			new_fl |= EXT4_APPEND_FL;
		if (vfs_fl & S_IMMUTABLE)
			new_fl |= EXT4_IMMUTABLE_FL;
		if (vfs_fl & S_NOATIME)
			new_fl |= EXT4_NOATIME_FL;
		if (vfs_fl & S_DIRSYNC)
			new_fl |= EXT4_DIRSYNC_FL;
	} while (cmpxchg(&ei->i_flags, old_fl, new_fl) != old_fl);
}

static blkcnt_t ext4_inode_blocks(struct ext4_inode *raw_inode,
				  struct ext4_inode_info *ei)
{
	blkcnt_t i_blocks ;
	struct inode *inode = &(ei->vfs_inode);
	struct super_block *sb = inode->i_sb;

	if (EXT4_HAS_RO_COMPAT_FEATURE(sb,
				EXT4_FEATURE_RO_COMPAT_HUGE_FILE)) {
		
		i_blocks = ((u64)le16_to_cpu(raw_inode->i_blocks_high)) << 32 |
					le32_to_cpu(raw_inode->i_blocks_lo);
		if (ext4_test_inode_flag(inode, EXT4_INODE_HUGE_FILE)) {
			
			return i_blocks  << (inode->i_blkbits - 9);
		} else {
			return i_blocks;
		}
	} else {
		return le32_to_cpu(raw_inode->i_blocks_lo);
	}
}

static inline void ext4_iget_extra_inode(struct inode *inode,
					 struct ext4_inode *raw_inode,
					 struct ext4_inode_info *ei)
{
	__le32 *magic = (void *)raw_inode +
			EXT4_GOOD_OLD_INODE_SIZE + ei->i_extra_isize;
	if (*magic == cpu_to_le32(EXT4_XATTR_MAGIC)) {
		ext4_set_inode_state(inode, EXT4_STATE_XATTR);
		ext4_find_inline_data_nolock(inode);
	} else
		EXT4_I(inode)->i_inline_off = 0;
}

struct inode *ext4_iget(struct super_block *sb, unsigned long ino)
{
	struct ext4_iloc iloc;
	struct ext4_inode *raw_inode;
	struct ext4_inode_info *ei;
	struct inode *inode;
	journal_t *journal = EXT4_SB(sb)->s_journal;
	long ret;
	int block;
	uid_t i_uid;
	gid_t i_gid;

	inode = iget_locked(sb, ino);
	if (!inode)
		return ERR_PTR(-ENOMEM);
	if (!(inode->i_state & I_NEW))
		return inode;

	ei = EXT4_I(inode);
	iloc.bh = NULL;

	ret = __ext4_get_inode_loc(inode, &iloc, 0);
	if (ret < 0)
		goto bad_inode;
	raw_inode = ext4_raw_inode(&iloc);

	if (EXT4_INODE_SIZE(inode->i_sb) > EXT4_GOOD_OLD_INODE_SIZE) {
		ei->i_extra_isize = le16_to_cpu(raw_inode->i_extra_isize);
		if (EXT4_GOOD_OLD_INODE_SIZE + ei->i_extra_isize >
		    EXT4_INODE_SIZE(inode->i_sb)) {
			EXT4_ERROR_INODE(inode, "bad extra_isize (%u != %u)",
				EXT4_GOOD_OLD_INODE_SIZE + ei->i_extra_isize,
				EXT4_INODE_SIZE(inode->i_sb));
			ret = -EIO;
			goto bad_inode;
		}
	} else
		ei->i_extra_isize = 0;

	
	if (ext4_has_metadata_csum(sb)) {
		struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
		__u32 csum;
		__le32 inum = cpu_to_le32(inode->i_ino);
		__le32 gen = raw_inode->i_generation;
		csum = ext4_chksum(sbi, sbi->s_csum_seed, (__u8 *)&inum,
				   sizeof(inum));
		ei->i_csum_seed = ext4_chksum(sbi, csum, (__u8 *)&gen,
					      sizeof(gen));
	}

	if (!ext4_inode_csum_verify(inode, raw_inode, ei)) {
		EXT4_ERROR_INODE(inode, "checksum invalid");
		ret = -EIO;
		goto bad_inode;
	}

	inode->i_mode = le16_to_cpu(raw_inode->i_mode);
	i_uid = (uid_t)le16_to_cpu(raw_inode->i_uid_low);
	i_gid = (gid_t)le16_to_cpu(raw_inode->i_gid_low);
	if (!(test_opt(inode->i_sb, NO_UID32))) {
		i_uid |= le16_to_cpu(raw_inode->i_uid_high) << 16;
		i_gid |= le16_to_cpu(raw_inode->i_gid_high) << 16;
	}
	i_uid_write(inode, i_uid);
	i_gid_write(inode, i_gid);
	set_nlink(inode, le16_to_cpu(raw_inode->i_links_count));

	ext4_clear_state_flags(ei);	
	ei->i_inline_off = 0;
	ei->i_dir_start_lookup = 0;
	ei->i_dtime = le32_to_cpu(raw_inode->i_dtime);
	if (inode->i_nlink == 0) {
		if ((inode->i_mode == 0 ||
		     !(EXT4_SB(inode->i_sb)->s_mount_state & EXT4_ORPHAN_FS)) &&
		    ino != EXT4_BOOT_LOADER_INO) {
			
			ret = -ESTALE;
			goto bad_inode;
		}
	}
	ei->i_flags = le32_to_cpu(raw_inode->i_flags);
	inode->i_blocks = ext4_inode_blocks(raw_inode, ei);
	ei->i_file_acl = le32_to_cpu(raw_inode->i_file_acl_lo);
	if (EXT4_HAS_INCOMPAT_FEATURE(sb, EXT4_FEATURE_INCOMPAT_64BIT))
		ei->i_file_acl |=
			((__u64)le16_to_cpu(raw_inode->i_file_acl_high)) << 32;
	inode->i_size = ext4_isize(raw_inode);
	ei->i_disksize = inode->i_size;
#ifdef CONFIG_QUOTA
	ei->i_reserved_quota = 0;
#endif
	inode->i_generation = le32_to_cpu(raw_inode->i_generation);
	ei->i_block_group = iloc.block_group;
	ei->i_last_alloc_group = ~0;
	for (block = 0; block < EXT4_N_BLOCKS; block++)
		ei->i_data[block] = raw_inode->i_block[block];
	INIT_LIST_HEAD(&ei->i_orphan);

	if (journal) {
		transaction_t *transaction;
		tid_t tid;

		read_lock(&journal->j_state_lock);
		if (journal->j_running_transaction)
			transaction = journal->j_running_transaction;
		else
			transaction = journal->j_committing_transaction;
		if (transaction)
			tid = transaction->t_tid;
		else
			tid = journal->j_commit_sequence;
		read_unlock(&journal->j_state_lock);
		ei->i_sync_tid = tid;
		ei->i_datasync_tid = tid;
	}

	if (EXT4_INODE_SIZE(inode->i_sb) > EXT4_GOOD_OLD_INODE_SIZE) {
		if (ei->i_extra_isize == 0) {
			
			ei->i_extra_isize = sizeof(struct ext4_inode) -
					    EXT4_GOOD_OLD_INODE_SIZE;
		} else {
			ext4_iget_extra_inode(inode, raw_inode, ei);
		}
	}

	EXT4_INODE_GET_XTIME(i_ctime, inode, raw_inode);
	EXT4_INODE_GET_XTIME(i_mtime, inode, raw_inode);
	EXT4_INODE_GET_XTIME(i_atime, inode, raw_inode);
	EXT4_EINODE_GET_XTIME(i_crtime, ei, raw_inode);

	if (likely(!test_opt2(inode->i_sb, HURD_COMPAT))) {
		inode->i_version = le32_to_cpu(raw_inode->i_disk_version);
		if (EXT4_INODE_SIZE(inode->i_sb) > EXT4_GOOD_OLD_INODE_SIZE) {
			if (EXT4_FITS_IN_INODE(raw_inode, ei, i_version_hi))
				inode->i_version |=
		    (__u64)(le32_to_cpu(raw_inode->i_version_hi)) << 32;
		}
	}

	ret = 0;
	if (ei->i_file_acl &&
	    !ext4_data_block_valid(EXT4_SB(sb), ei->i_file_acl, 1)) {
		EXT4_ERROR_INODE(inode, "bad extended attribute block %llu",
				 ei->i_file_acl);
		ret = -EIO;
		goto bad_inode;
	} else if (!ext4_has_inline_data(inode)) {
		if (ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS)) {
			if ((S_ISREG(inode->i_mode) || S_ISDIR(inode->i_mode) ||
			    (S_ISLNK(inode->i_mode) &&
			     !ext4_inode_is_fast_symlink(inode))))
				
				ret = ext4_ext_check_inode(inode);
		} else if (S_ISREG(inode->i_mode) || S_ISDIR(inode->i_mode) ||
			   (S_ISLNK(inode->i_mode) &&
			    !ext4_inode_is_fast_symlink(inode))) {
			
			ret = ext4_ind_check_inode(inode);
		}
	}
	if (ret)
		goto bad_inode;

	if (S_ISREG(inode->i_mode)) {
		inode->i_op = &ext4_file_inode_operations;
		inode->i_fop = &ext4_file_operations;
		ext4_set_aops(inode);
	} else if (S_ISDIR(inode->i_mode)) {
		inode->i_op = &ext4_dir_inode_operations;
		inode->i_fop = &ext4_dir_operations;
	} else if (S_ISLNK(inode->i_mode)) {
		if (ext4_inode_is_fast_symlink(inode)) {
			inode->i_op = &ext4_fast_symlink_inode_operations;
			nd_terminate_link(ei->i_data, inode->i_size,
				sizeof(ei->i_data) - 1);
		} else {
			inode->i_op = &ext4_symlink_inode_operations;
			ext4_set_aops(inode);
		}
	} else if (S_ISCHR(inode->i_mode) || S_ISBLK(inode->i_mode) ||
	      S_ISFIFO(inode->i_mode) || S_ISSOCK(inode->i_mode)) {
		inode->i_op = &ext4_special_inode_operations;
		if (raw_inode->i_block[0])
			init_special_inode(inode, inode->i_mode,
			   old_decode_dev(le32_to_cpu(raw_inode->i_block[0])));
		else
			init_special_inode(inode, inode->i_mode,
			   new_decode_dev(le32_to_cpu(raw_inode->i_block[1])));
	} else if (ino == EXT4_BOOT_LOADER_INO) {
		make_bad_inode(inode);
	} else {
		ret = -EIO;
		EXT4_ERROR_INODE(inode, "bogus i_mode (%o)", inode->i_mode);
		goto bad_inode;
	}
	brelse(iloc.bh);
	ext4_set_inode_flags(inode);
	unlock_new_inode(inode);
	return inode;

bad_inode:
	brelse(iloc.bh);
	iget_failed(inode);
	return ERR_PTR(ret);
}

struct inode *ext4_iget_normal(struct super_block *sb, unsigned long ino)
{
	if (ino < EXT4_FIRST_INO(sb) && ino != EXT4_ROOT_INO)
		return ERR_PTR(-EIO);
	return ext4_iget(sb, ino);
}

static int ext4_inode_blocks_set(handle_t *handle,
				struct ext4_inode *raw_inode,
				struct ext4_inode_info *ei)
{
	struct inode *inode = &(ei->vfs_inode);
	u64 i_blocks = inode->i_blocks;
	struct super_block *sb = inode->i_sb;

	if (i_blocks <= ~0U) {
		raw_inode->i_blocks_lo   = cpu_to_le32(i_blocks);
		raw_inode->i_blocks_high = 0;
		ext4_clear_inode_flag(inode, EXT4_INODE_HUGE_FILE);
		return 0;
	}
	if (!EXT4_HAS_RO_COMPAT_FEATURE(sb, EXT4_FEATURE_RO_COMPAT_HUGE_FILE))
		return -EFBIG;

	if (i_blocks <= 0xffffffffffffULL) {
		raw_inode->i_blocks_lo   = cpu_to_le32(i_blocks);
		raw_inode->i_blocks_high = cpu_to_le16(i_blocks >> 32);
		ext4_clear_inode_flag(inode, EXT4_INODE_HUGE_FILE);
	} else {
		ext4_set_inode_flag(inode, EXT4_INODE_HUGE_FILE);
		
		i_blocks = i_blocks >> (inode->i_blkbits - 9);
		raw_inode->i_blocks_lo   = cpu_to_le32(i_blocks);
		raw_inode->i_blocks_high = cpu_to_le16(i_blocks >> 32);
	}
	return 0;
}

static int ext4_do_update_inode(handle_t *handle,
				struct inode *inode,
				struct ext4_iloc *iloc)
{
	struct ext4_inode *raw_inode = ext4_raw_inode(iloc);
	struct ext4_inode_info *ei = EXT4_I(inode);
	struct buffer_head *bh = iloc->bh;
	struct super_block *sb = inode->i_sb;
	int err = 0, rc, block;
	int need_datasync = 0, set_large_file = 0;
	uid_t i_uid;
	gid_t i_gid;

	spin_lock(&ei->i_raw_lock);

	if (ext4_test_inode_state(inode, EXT4_STATE_NEW))
		memset(raw_inode, 0, EXT4_SB(inode->i_sb)->s_inode_size);

	ext4_get_inode_flags(ei);
	raw_inode->i_mode = cpu_to_le16(inode->i_mode);
	i_uid = i_uid_read(inode);
	i_gid = i_gid_read(inode);
	if (!(test_opt(inode->i_sb, NO_UID32))) {
		raw_inode->i_uid_low = cpu_to_le16(low_16_bits(i_uid));
		raw_inode->i_gid_low = cpu_to_le16(low_16_bits(i_gid));
		if (!ei->i_dtime) {
			raw_inode->i_uid_high =
				cpu_to_le16(high_16_bits(i_uid));
			raw_inode->i_gid_high =
				cpu_to_le16(high_16_bits(i_gid));
		} else {
			raw_inode->i_uid_high = 0;
			raw_inode->i_gid_high = 0;
		}
	} else {
		raw_inode->i_uid_low = cpu_to_le16(fs_high2lowuid(i_uid));
		raw_inode->i_gid_low = cpu_to_le16(fs_high2lowgid(i_gid));
		raw_inode->i_uid_high = 0;
		raw_inode->i_gid_high = 0;
	}
	raw_inode->i_links_count = cpu_to_le16(inode->i_nlink);

	EXT4_INODE_SET_XTIME(i_ctime, inode, raw_inode);
	EXT4_INODE_SET_XTIME(i_mtime, inode, raw_inode);
	EXT4_INODE_SET_XTIME(i_atime, inode, raw_inode);
	EXT4_EINODE_SET_XTIME(i_crtime, ei, raw_inode);

	err = ext4_inode_blocks_set(handle, raw_inode, ei);
	if (err) {
		spin_unlock(&ei->i_raw_lock);
		goto out_brelse;
	}
	raw_inode->i_dtime = cpu_to_le32(ei->i_dtime);
	raw_inode->i_flags = cpu_to_le32(ei->i_flags & 0xFFFFFFFF);
	if (likely(!test_opt2(inode->i_sb, HURD_COMPAT)))
		raw_inode->i_file_acl_high =
			cpu_to_le16(ei->i_file_acl >> 32);
	raw_inode->i_file_acl_lo = cpu_to_le32(ei->i_file_acl);
	if (ei->i_disksize != ext4_isize(raw_inode)) {
		ext4_isize_set(raw_inode, ei->i_disksize);
		need_datasync = 1;
	}
	if (ei->i_disksize > 0x7fffffffULL) {
		if (!EXT4_HAS_RO_COMPAT_FEATURE(sb,
				EXT4_FEATURE_RO_COMPAT_LARGE_FILE) ||
				EXT4_SB(sb)->s_es->s_rev_level ==
		    cpu_to_le32(EXT4_GOOD_OLD_REV))
			set_large_file = 1;
	}
	raw_inode->i_generation = cpu_to_le32(inode->i_generation);
	if (S_ISCHR(inode->i_mode) || S_ISBLK(inode->i_mode)) {
		if (old_valid_dev(inode->i_rdev)) {
			raw_inode->i_block[0] =
				cpu_to_le32(old_encode_dev(inode->i_rdev));
			raw_inode->i_block[1] = 0;
		} else {
			raw_inode->i_block[0] = 0;
			raw_inode->i_block[1] =
				cpu_to_le32(new_encode_dev(inode->i_rdev));
			raw_inode->i_block[2] = 0;
		}
	} else if (!ext4_has_inline_data(inode)) {
		for (block = 0; block < EXT4_N_BLOCKS; block++)
			raw_inode->i_block[block] = ei->i_data[block];
	}

	if (likely(!test_opt2(inode->i_sb, HURD_COMPAT))) {
		raw_inode->i_disk_version = cpu_to_le32(inode->i_version);
		if (ei->i_extra_isize) {
			if (EXT4_FITS_IN_INODE(raw_inode, ei, i_version_hi))
				raw_inode->i_version_hi =
					cpu_to_le32(inode->i_version >> 32);
			raw_inode->i_extra_isize =
				cpu_to_le16(ei->i_extra_isize);
		}
	}

	ext4_inode_csum_set(inode, raw_inode, ei);

	spin_unlock(&ei->i_raw_lock);

	BUFFER_TRACE(bh, "call ext4_handle_dirty_metadata");
	rc = ext4_handle_dirty_metadata(handle, NULL, bh);
	if (!err)
		err = rc;
	ext4_clear_inode_state(inode, EXT4_STATE_NEW);
	if (set_large_file) {
		BUFFER_TRACE(EXT4_SB(sb)->s_sbh, "get write access");
		err = ext4_journal_get_write_access(handle, EXT4_SB(sb)->s_sbh);
		if (err)
			goto out_brelse;
		ext4_update_dynamic_rev(sb);
		EXT4_SET_RO_COMPAT_FEATURE(sb,
					   EXT4_FEATURE_RO_COMPAT_LARGE_FILE);
		ext4_handle_sync(handle);
		err = ext4_handle_dirty_super(handle, sb);
	}
	ext4_update_inode_fsync_trans(handle, inode, need_datasync);
out_brelse:
	brelse(bh);
	ext4_std_error(inode->i_sb, err);
	return err;
}

int ext4_write_inode(struct inode *inode, struct writeback_control *wbc)
{
	int err;

	if (WARN_ON_ONCE(current->flags & PF_MEMALLOC))
		return 0;

	if (EXT4_SB(inode->i_sb)->s_journal) {
		if (ext4_journal_current_handle()) {
			jbd_debug(1, "called recursively, non-PF_MEMALLOC!\n");
			dump_stack();
			return -EIO;
		}

		/*
		 * No need to force transaction in WB_SYNC_NONE mode. Also
		 * ext4_sync_fs() will force the commit after everything is
		 * written.
		 */
		if (wbc->sync_mode != WB_SYNC_ALL || wbc->for_sync)
			return 0;

		err = ext4_force_commit(inode->i_sb);
	} else {
		struct ext4_iloc iloc;

		err = __ext4_get_inode_loc(inode, &iloc, 0);
		if (err)
			return err;
		if (wbc->sync_mode == WB_SYNC_ALL && !wbc->for_sync)
			sync_dirty_buffer(iloc.bh);
		if (buffer_req(iloc.bh) && !buffer_uptodate(iloc.bh)) {
			EXT4_ERROR_INODE_BLOCK(inode, iloc.bh->b_blocknr,
					 "IO error syncing inode");
			err = -EIO;
		}
		brelse(iloc.bh);
	}
	return err;
}

static void ext4_wait_for_tail_page_commit(struct inode *inode)
{
	struct page *page;
	unsigned offset;
	journal_t *journal = EXT4_SB(inode->i_sb)->s_journal;
	tid_t commit_tid = 0;
	int ret;

	offset = inode->i_size & (PAGE_CACHE_SIZE - 1);
	if (offset > PAGE_CACHE_SIZE - (1 << inode->i_blkbits))
		return;
	while (1) {
		page = find_lock_page(inode->i_mapping,
				      inode->i_size >> PAGE_CACHE_SHIFT);
		if (!page)
			return;
		ret = __ext4_journalled_invalidatepage(page, offset,
						PAGE_CACHE_SIZE - offset);
		unlock_page(page);
		page_cache_release(page);
		if (ret != -EBUSY)
			return;
		commit_tid = 0;
		read_lock(&journal->j_state_lock);
		if (journal->j_committing_transaction)
			commit_tid = journal->j_committing_transaction->t_tid;
		read_unlock(&journal->j_state_lock);
		if (commit_tid)
			jbd2_log_wait_commit(journal, commit_tid);
	}
}

/*
 * ext4_setattr()
 *
 * Called from notify_change.
 *
 * We want to trap VFS attempts to truncate the file as soon as
 * possible.  In particular, we want to make sure that when the VFS
 * shrinks i_size, we put the inode on the orphan list and modify
 * i_disksize immediately, so that during the subsequent flushing of
 * dirty pages and freeing of disk blocks, we can guarantee that any
 * commit will leave the blocks being flushed in an unused state on
 * disk.  (On recovery, the inode will get truncated and the blocks will
 * be freed, so we have a strong guarantee that no future commit will
 * leave these blocks visible to the user.)
 *
 * Another thing we have to assure is that if we are in ordered mode
 * and inode is still attached to the committing transaction, we must
 * we start writeout of all the dirty pages which are being truncated.
 * This way we are sure that all the data written in the previous
 * transaction are already on disk (truncate waits for pages under
 * writeback).
 *
 * Called with inode->i_mutex down.
 */
int ext4_setattr(struct dentry *dentry, struct iattr *attr)
{
	struct inode *inode = dentry->d_inode;
	int error, rc = 0;
	int orphan = 0;
	const unsigned int ia_valid = attr->ia_valid;

	error = inode_change_ok(inode, attr);
	if (error)
		return error;

	if (is_quota_modification(inode, attr))
		dquot_initialize(inode);
	if ((ia_valid & ATTR_UID && !uid_eq(attr->ia_uid, inode->i_uid)) ||
	    (ia_valid & ATTR_GID && !gid_eq(attr->ia_gid, inode->i_gid))) {
		handle_t *handle;

		handle = ext4_journal_start(inode, EXT4_HT_QUOTA,
			(EXT4_MAXQUOTAS_INIT_BLOCKS(inode->i_sb) +
			 EXT4_MAXQUOTAS_DEL_BLOCKS(inode->i_sb)) + 3);
		if (IS_ERR(handle)) {
			error = PTR_ERR(handle);
			goto err_out;
		}
		error = dquot_transfer(inode, attr);
		if (error) {
			ext4_journal_stop(handle);
			return error;
		}
		if (attr->ia_valid & ATTR_UID)
			inode->i_uid = attr->ia_uid;
		if (attr->ia_valid & ATTR_GID)
			inode->i_gid = attr->ia_gid;
		error = ext4_mark_inode_dirty(handle, inode);
		ext4_journal_stop(handle);
	}

	if (attr->ia_valid & ATTR_SIZE && attr->ia_size != inode->i_size) {
		handle_t *handle;

		if (!(ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS))) {
			struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);

			if (attr->ia_size > sbi->s_bitmap_maxbytes)
				return -EFBIG;
		}

		if (IS_I_VERSION(inode) && attr->ia_size != inode->i_size)
			inode_inc_iversion(inode);

		if (S_ISREG(inode->i_mode) &&
		    (attr->ia_size < inode->i_size)) {
			if (ext4_should_order_data(inode)) {
				error = ext4_begin_ordered_truncate(inode,
							    attr->ia_size);
				if (error)
					goto err_out;
			}
			handle = ext4_journal_start(inode, EXT4_HT_INODE, 3);
			if (IS_ERR(handle)) {
				error = PTR_ERR(handle);
				goto err_out;
			}
			if (ext4_handle_valid(handle)) {
				error = ext4_orphan_add(handle, inode);
				orphan = 1;
			}
			down_write(&EXT4_I(inode)->i_data_sem);
			EXT4_I(inode)->i_disksize = attr->ia_size;
			rc = ext4_mark_inode_dirty(handle, inode);
			if (!error)
				error = rc;
			if (!error)
				i_size_write(inode, attr->ia_size);
			up_write(&EXT4_I(inode)->i_data_sem);
			ext4_journal_stop(handle);
			if (error) {
				ext4_orphan_del(NULL, inode);
				goto err_out;
			}
		} else {
			loff_t oldsize = inode->i_size;

			i_size_write(inode, attr->ia_size);
			pagecache_isize_extended(inode, oldsize, inode->i_size);
		}

		if (orphan) {
			if (!ext4_should_journal_data(inode)) {
				ext4_inode_block_unlocked_dio(inode);
				inode_dio_wait(inode);
				ext4_inode_resume_unlocked_dio(inode);
			} else
				ext4_wait_for_tail_page_commit(inode);
		}
			truncate_pagecache(inode, inode->i_size);
	}
	if (attr->ia_valid & ATTR_SIZE)
		ext4_truncate(inode);

	if (!rc) {
		setattr_copy(inode, attr);
		mark_inode_dirty(inode);
	}

	if (orphan && inode->i_nlink)
		ext4_orphan_del(NULL, inode);

	if (!rc && (ia_valid & ATTR_MODE))
		rc = posix_acl_chmod(inode, inode->i_mode);

err_out:
	ext4_std_error(inode->i_sb, error);
	if (!error)
		error = rc;
	return error;
}

int ext4_getattr(struct vfsmount *mnt, struct dentry *dentry,
		 struct kstat *stat)
{
	struct inode *inode;
	unsigned long long delalloc_blocks;

	inode = dentry->d_inode;
	generic_fillattr(inode, stat);

	if (unlikely(ext4_has_inline_data(inode)))
		stat->blocks += (stat->size + 511) >> 9;

	delalloc_blocks = EXT4_C2B(EXT4_SB(inode->i_sb),
				   EXT4_I(inode)->i_reserved_data_blocks);
	stat->blocks += delalloc_blocks << (inode->i_sb->s_blocksize_bits - 9);
	return 0;
}

static int ext4_index_trans_blocks(struct inode *inode, int lblocks,
				   int pextents)
{
	if (!(ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS)))
		return ext4_ind_trans_blocks(inode, lblocks);
	return ext4_ext_index_trans_blocks(inode, pextents);
}

static int ext4_meta_trans_blocks(struct inode *inode, int lblocks,
				  int pextents)
{
	ext4_group_t groups, ngroups = ext4_get_groups_count(inode->i_sb);
	int gdpblocks;
	int idxblocks;
	int ret = 0;

	idxblocks = ext4_index_trans_blocks(inode, lblocks, pextents);

	ret = idxblocks;

	groups = idxblocks + pextents;
	gdpblocks = groups;
	if (groups > ngroups)
		groups = ngroups;
	if (groups > EXT4_SB(inode->i_sb)->s_gdb_count)
		gdpblocks = EXT4_SB(inode->i_sb)->s_gdb_count;

	
	ret += groups + gdpblocks;

	
	ret += EXT4_META_TRANS_BLOCKS(inode->i_sb);

	return ret;
}

int ext4_writepage_trans_blocks(struct inode *inode)
{
	int bpp = ext4_journal_blocks_per_page(inode);
	int ret;

	ret = ext4_meta_trans_blocks(inode, bpp, bpp);

	
	if (ext4_should_journal_data(inode))
		ret += bpp;
	return ret;
}

int ext4_chunk_trans_blocks(struct inode *inode, int nrblocks)
{
	return ext4_meta_trans_blocks(inode, nrblocks, 1);
}

int ext4_mark_iloc_dirty(handle_t *handle,
			 struct inode *inode, struct ext4_iloc *iloc)
{
	int err = 0;

	if (IS_I_VERSION(inode))
		inode_inc_iversion(inode);

	
	get_bh(iloc->bh);

	
	err = ext4_do_update_inode(handle, inode, iloc);
	put_bh(iloc->bh);
	return err;
}


int
ext4_reserve_inode_write(handle_t *handle, struct inode *inode,
			 struct ext4_iloc *iloc)
{
	int err;

	err = ext4_get_inode_loc(inode, iloc);
	if (!err) {
		BUFFER_TRACE(iloc->bh, "get_write_access");
		err = ext4_journal_get_write_access(handle, iloc->bh);
		if (err) {
			brelse(iloc->bh);
			iloc->bh = NULL;
		}
	}
	ext4_std_error(inode->i_sb, err);
	return err;
}

static int ext4_expand_extra_isize(struct inode *inode,
				   unsigned int new_extra_isize,
				   struct ext4_iloc iloc,
				   handle_t *handle)
{
	struct ext4_inode *raw_inode;
	struct ext4_xattr_ibody_header *header;

	if (EXT4_I(inode)->i_extra_isize >= new_extra_isize)
		return 0;

	raw_inode = ext4_raw_inode(&iloc);

	header = IHDR(inode, raw_inode);

	
	if (!ext4_test_inode_state(inode, EXT4_STATE_XATTR) ||
	    header->h_magic != cpu_to_le32(EXT4_XATTR_MAGIC)) {
		memset((void *)raw_inode + EXT4_GOOD_OLD_INODE_SIZE, 0,
			new_extra_isize);
		EXT4_I(inode)->i_extra_isize = new_extra_isize;
		return 0;
	}

	
	return ext4_expand_extra_isize_ea(inode, new_extra_isize,
					  raw_inode, handle);
}

/*
 * What we do here is to mark the in-core inode as clean with respect to inode
 * dirtiness (it may still be data-dirty).
 * This means that the in-core inode may be reaped by prune_icache
 * without having to perform any I/O.  This is a very good thing,
 * because *any* task may call prune_icache - even ones which
 * have a transaction open against a different journal.
 *
 * Is this cheating?  Not really.  Sure, we haven't written the
 * inode out, but prune_icache isn't a user-visible syncing function.
 * Whenever the user wants stuff synced (sys_sync, sys_msync, sys_fsync)
 * we start and wait on commits.
 */
int ext4_mark_inode_dirty(handle_t *handle, struct inode *inode)
{
	struct ext4_iloc iloc;
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	static unsigned int mnt_count;
	int err, ret;

	might_sleep();
	trace_ext4_mark_inode_dirty(inode, _RET_IP_);
	err = ext4_reserve_inode_write(handle, inode, &iloc);
	if (ext4_handle_valid(handle) &&
	    EXT4_I(inode)->i_extra_isize < sbi->s_want_extra_isize &&
	    !ext4_test_inode_state(inode, EXT4_STATE_NO_EXPAND)) {
		if ((jbd2_journal_extend(handle,
			     EXT4_DATA_TRANS_BLOCKS(inode->i_sb))) == 0) {
			ret = ext4_expand_extra_isize(inode,
						      sbi->s_want_extra_isize,
						      iloc, handle);
			if (ret) {
				ext4_set_inode_state(inode,
						     EXT4_STATE_NO_EXPAND);
				if (mnt_count !=
					le16_to_cpu(sbi->s_es->s_mnt_count)) {
					ext4_warning(inode->i_sb,
					"Unable to expand inode %lu. Delete"
					" some EAs or run e2fsck.",
					inode->i_ino);
					mnt_count =
					  le16_to_cpu(sbi->s_es->s_mnt_count);
				}
			}
		}
	}
	if (!err)
		err = ext4_mark_iloc_dirty(handle, inode, &iloc);
	return err;
}

void ext4_dirty_inode(struct inode *inode, int flags)
{
	handle_t *handle;

	handle = ext4_journal_start(inode, EXT4_HT_INODE, 2);
	if (IS_ERR(handle))
		goto out;

	ext4_mark_inode_dirty(handle, inode);

	ext4_journal_stop(handle);
out:
	return;
}

#if 0
static int ext4_pin_inode(handle_t *handle, struct inode *inode)
{
	struct ext4_iloc iloc;

	int err = 0;
	if (handle) {
		err = ext4_get_inode_loc(inode, &iloc);
		if (!err) {
			BUFFER_TRACE(iloc.bh, "get_write_access");
			err = jbd2_journal_get_write_access(handle, iloc.bh);
			if (!err)
				err = ext4_handle_dirty_metadata(handle,
								 NULL,
								 iloc.bh);
			brelse(iloc.bh);
		}
	}
	ext4_std_error(inode->i_sb, err);
	return err;
}
#endif

int ext4_change_inode_journal_flag(struct inode *inode, int val)
{
	journal_t *journal;
	handle_t *handle;
	int err;


	journal = EXT4_JOURNAL(inode);
	if (!journal)
		return 0;
	if (is_journal_aborted(journal))
		return -EROFS;
	if (val && test_opt(inode->i_sb, DELALLOC)) {
		err = ext4_alloc_da_blocks(inode);
		if (err < 0)
			return err;
	}

	
	ext4_inode_block_unlocked_dio(inode);
	inode_dio_wait(inode);

	jbd2_journal_lock_updates(journal);


	if (val)
		ext4_set_inode_flag(inode, EXT4_INODE_JOURNAL_DATA);
	else {
		err = jbd2_journal_flush(journal);
		if (err < 0) {
			jbd2_journal_unlock_updates(journal);
			ext4_inode_resume_unlocked_dio(inode);
			return err;
		}
		ext4_clear_inode_flag(inode, EXT4_INODE_JOURNAL_DATA);
	}
	ext4_set_aops(inode);

	jbd2_journal_unlock_updates(journal);
	ext4_inode_resume_unlocked_dio(inode);

	

	handle = ext4_journal_start(inode, EXT4_HT_INODE, 1);
	if (IS_ERR(handle))
		return PTR_ERR(handle);

	err = ext4_mark_inode_dirty(handle, inode);
	ext4_handle_sync(handle);
	ext4_journal_stop(handle);
	ext4_std_error(inode->i_sb, err);

	return err;
}

static int ext4_bh_unmapped(handle_t *handle, struct buffer_head *bh)
{
	return !buffer_mapped(bh);
}

int ext4_page_mkwrite(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct page *page = vmf->page;
	loff_t size;
	unsigned long len;
	int ret;
	struct file *file = vma->vm_file;
	struct inode *inode = file_inode(file);
	struct address_space *mapping = inode->i_mapping;
	handle_t *handle;
	get_block_t *get_block;
	int retries = 0;

	sb_start_pagefault(inode->i_sb);
	file_update_time(vma->vm_file);
	
	if (test_opt(inode->i_sb, DELALLOC) &&
	    !ext4_should_journal_data(inode) &&
	    !ext4_nonda_switch(inode->i_sb)) {
		do {
			ret = __block_page_mkwrite(vma, vmf,
						   ext4_da_get_block_prep);
		} while (ret == -ENOSPC &&
		       ext4_should_retry_alloc(inode->i_sb, &retries));
		goto out_ret;
	}

	lock_page(page);
	size = i_size_read(inode);
	
	if (page->mapping != mapping || page_offset(page) > size) {
		unlock_page(page);
		ret = VM_FAULT_NOPAGE;
		goto out;
	}

	if (page->index == size >> PAGE_CACHE_SHIFT)
		len = size & ~PAGE_CACHE_MASK;
	else
		len = PAGE_CACHE_SIZE;
	if (page_has_buffers(page)) {
		if (!ext4_walk_page_buffers(NULL, page_buffers(page),
					    0, len, NULL,
					    ext4_bh_unmapped)) {
			
			wait_for_stable_page(page);
			ret = VM_FAULT_LOCKED;
			goto out;
		}
	}
	unlock_page(page);
	
	if (ext4_should_dioread_nolock(inode))
		get_block = ext4_get_block_write;
	else
		get_block = ext4_get_block;
retry_alloc:
	handle = ext4_journal_start(inode, EXT4_HT_WRITE_PAGE,
				    ext4_writepage_trans_blocks(inode));
	if (IS_ERR(handle)) {
		ret = VM_FAULT_SIGBUS;
		goto out;
	}
	ret = __block_page_mkwrite(vma, vmf, get_block);
	if (!ret && ext4_should_journal_data(inode)) {
		if (ext4_walk_page_buffers(handle, page_buffers(page), 0,
			  PAGE_CACHE_SIZE, NULL, do_journal_get_write_access)) {
			unlock_page(page);
			ret = VM_FAULT_SIGBUS;
			ext4_journal_stop(handle);
			goto out;
		}
		ext4_set_inode_state(inode, EXT4_STATE_JDATA);
	}
	ext4_journal_stop(handle);
	if (ret == -ENOSPC && ext4_should_retry_alloc(inode->i_sb, &retries))
		goto retry_alloc;
out_ret:
	ret = block_page_mkwrite_return(ret);
out:
	sb_end_pagefault(inode->i_sb);
	return ret;
}
