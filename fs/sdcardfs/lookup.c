/*
 * fs/sdcardfs/lookup.c
 *
 * Copyright (c) 2013 Samsung Electronics Co. Ltd
 *   Authors: Daeho Jeong, Woojoong Lee, Seunghwan Hyun,
 *               Sunghwan Yun, Sungjong Seo
 *
 * This program has been developed as a stackable file system based on
 * the WrapFS which written by
 *
 * Copyright (c) 1998-2011 Erez Zadok
 * Copyright (c) 2009     Shrikar Archak
 * Copyright (c) 2003-2011 Stony Brook University
 * Copyright (c) 2003-2011 The Research Foundation of SUNY
 *
 * This file is dual licensed.  It may be redistributed and/or modified
 * under the terms of the Apache 2.0 License OR version 2 of the GNU
 * General Public License.
 */

#include "sdcardfs.h"
#include "linux/delay.h"

static struct kmem_cache *sdcardfs_dentry_cachep;

int sdcardfs_init_dentry_cache(void)
{
	sdcardfs_dentry_cachep =
		kmem_cache_create("sdcardfs_dentry",
				  sizeof(struct sdcardfs_dentry_info),
				  0, SLAB_RECLAIM_ACCOUNT, NULL);

	return sdcardfs_dentry_cachep ? 0 : -ENOMEM;
}

void sdcardfs_destroy_dentry_cache(void)
{
	if (sdcardfs_dentry_cachep)
		kmem_cache_destroy(sdcardfs_dentry_cachep);
}

void free_dentry_private_data(struct dentry *dentry)
{
	if (!dentry || !dentry->d_fsdata)
		return;
	kmem_cache_free(sdcardfs_dentry_cachep, dentry->d_fsdata);
	dentry->d_fsdata = NULL;
}

int new_dentry_private_data(struct dentry *dentry)
{
	struct sdcardfs_dentry_info *info = SDCARDFS_D(dentry);

	
	info = kmem_cache_zalloc(sdcardfs_dentry_cachep, GFP_ATOMIC);
	if (!info)
		return -ENOMEM;

	spin_lock_init(&info->lock);
	dentry->d_fsdata = info;

	return 0;
}

struct inode_data {
	struct inode *lower_inode;
	userid_t id;
};

static int sdcardfs_inode_test(struct inode *inode, void *candidate_data)
{
	struct inode *current_lower_inode = sdcardfs_lower_inode(inode);
	userid_t current_userid = SDCARDFS_I(inode)->userid;
	if (current_lower_inode == ((struct inode_data *)candidate_data)->lower_inode &&
			current_userid == ((struct inode_data *)candidate_data)->id)
		return 1; 
	else
		return 0; 
}

static int sdcardfs_inode_set(struct inode *inode, void *lower_inode)
{
	
	return 0;
}

struct inode *sdcardfs_iget(struct super_block *sb, struct inode *lower_inode, userid_t id)
{
	struct sdcardfs_inode_info *info;
	struct inode_data data;
	struct inode *inode; 
	int err;

	data.id = id;
	data.lower_inode = lower_inode;
	inode = iget5_locked(sb, 
			     lower_inode->i_ino, 
			     sdcardfs_inode_test,	
			     sdcardfs_inode_set, 
			     &data); 
	if (!inode) {
		err = -EACCES;
		iput(lower_inode);
		return ERR_PTR(err);
	}
	
	if (!(inode->i_state & I_NEW))
		return inode;

	
	info = SDCARDFS_I(inode);

	inode->i_ino = lower_inode->i_ino;
	if (!igrab(lower_inode)) {
		err = -ESTALE;
		return ERR_PTR(err);
	}
	sdcardfs_set_lower_inode(inode, lower_inode);

	inode->i_version++;

	
	if (S_ISDIR(lower_inode->i_mode))
		inode->i_op = &sdcardfs_dir_iops;
	else if (S_ISLNK(lower_inode->i_mode))
		inode->i_op = &sdcardfs_symlink_iops;
	else
		inode->i_op = &sdcardfs_main_iops;

	
	if (S_ISDIR(lower_inode->i_mode))
		inode->i_fop = &sdcardfs_dir_fops;
	else
		inode->i_fop = &sdcardfs_main_fops;

	inode->i_mapping->a_ops = &sdcardfs_aops;

	inode->i_atime.tv_sec = 0;
	inode->i_atime.tv_nsec = 0;
	inode->i_mtime.tv_sec = 0;
	inode->i_mtime.tv_nsec = 0;
	inode->i_ctime.tv_sec = 0;
	inode->i_ctime.tv_nsec = 0;

	
	if (S_ISBLK(lower_inode->i_mode) || S_ISCHR(lower_inode->i_mode) ||
	    S_ISFIFO(lower_inode->i_mode) || S_ISSOCK(lower_inode->i_mode))
		init_special_inode(inode, lower_inode->i_mode,
				   lower_inode->i_rdev);

	
	sdcardfs_copy_and_fix_attrs(inode, lower_inode);
	fsstack_copy_inode_size(inode, lower_inode);

	unlock_new_inode(inode);
	return inode;
}

int sdcardfs_interpose(struct dentry *dentry, struct super_block *sb,
		     struct path *lower_path, userid_t id)
{
	int err = 0;
	struct inode *inode;
	struct inode *lower_inode;
	struct super_block *lower_sb;

	lower_inode = lower_path->dentry->d_inode;
	lower_sb = sdcardfs_lower_super(sb);

	
	if (lower_inode->i_sb != lower_sb) {
		err = -EXDEV;
		goto out;
	}


	
	inode = sdcardfs_iget(sb, lower_inode, id);
	if (IS_ERR(inode)) {
		err = PTR_ERR(inode);
		goto out;
	}

	d_add(dentry, inode);
	update_derived_permission_lock(dentry);
out:
	return err;
}

static struct dentry *__sdcardfs_lookup(struct dentry *dentry,
		unsigned int flags, struct path *lower_parent_path, userid_t id)
{
	int err = 0;
	struct vfsmount *lower_dir_mnt;
	struct dentry *lower_dir_dentry = NULL;
	struct dentry *lower_dentry;
	const char *name;
	struct path lower_path;
	struct qstr this;
	struct sdcardfs_sb_info *sbi;

	sbi = SDCARDFS_SB(dentry->d_sb);
	
	d_set_d_op(dentry, &sdcardfs_ci_dops);

	if (IS_ROOT(dentry))
		goto out;

	name = dentry->d_name.name;

	
	lower_dir_dentry = lower_parent_path->dentry;
	lower_dir_mnt = lower_parent_path->mnt;
	
#ifdef CONFIG_SDCARD_FS_CI_SEARCH
	err = vfs_path_lookup(lower_dir_dentry, lower_dir_mnt, name,
				LOOKUP_CASE_INSENSITIVE, &lower_path);
#else
	err = vfs_path_lookup(lower_dir_dentry, lower_dir_mnt, name, 0,
				&lower_path);
	
	if (err == -ENOENT) {
		struct dentry *child;
		struct dentry *match = NULL;
		spin_lock(&lower_dir_dentry->d_lock);
		list_for_each_entry(child, &lower_dir_dentry->d_subdirs, d_child) {
			if (child && child->d_inode) {
				if (strcasecmp(child->d_name.name, name)==0) {
					printk("sdcardfs match %s", child->d_name.name);
					match = dget(child);
					break;
				}
			}
		}
		spin_unlock(&lower_dir_dentry->d_lock);
		if (match) {
			err = vfs_path_lookup(lower_dir_dentry,
						lower_dir_mnt,
						match->d_name.name, 0,
						&lower_path);
			dput(match);
		}
	}
#endif

	
	if (!err) {

		if(need_graft_path(dentry)) {

			err = setup_obb_dentry(dentry, &lower_path);

			if(err) {
				printk(KERN_INFO "sdcardfs: base obbpath is not available\n");
				sdcardfs_put_reset_orig_path(dentry);
				goto out;
			}
		}

		sdcardfs_set_lower_path(dentry, &lower_path);
		err = sdcardfs_interpose(dentry, dentry->d_sb, &lower_path, id);
		if (err) 
			sdcardfs_put_reset_lower_path(dentry);
		goto out;
	}

	if (err && err != -ENOENT)
		goto out;

	
	this.name = name;
	this.len = strlen(name);
	this.hash = full_name_hash(this.name, this.len);
	lower_dentry = d_lookup(lower_dir_dentry, &this);
	if (lower_dentry)
		goto setup_lower;

	lower_dentry = d_alloc(lower_dir_dentry, &this);
	if (!lower_dentry) {
		err = -ENOMEM;
		goto out;
	}
	d_add(lower_dentry, NULL); 

setup_lower:
	lower_path.dentry = lower_dentry;
	lower_path.mnt = mntget(lower_dir_mnt);
	sdcardfs_set_lower_path(dentry, &lower_path);

	if (flags & (LOOKUP_CREATE|LOOKUP_RENAME_TARGET))
		err = 0;

out:
	return ERR_PTR(err);
}

struct dentry *sdcardfs_lookup(struct inode *dir, struct dentry *dentry,
			     unsigned int flags)
{
	struct dentry *ret = NULL, *parent;
	struct path lower_parent_path;
	int err = 0;
	const struct cred *saved_cred = NULL;

	parent = dget_parent(dentry);

	if(!check_caller_access_to_name(parent->d_inode, dentry->d_name.name)) {
		ret = ERR_PTR(-EACCES);
		printk(KERN_INFO "%s: need to check the caller's gid in packages.list\n"
                         "	dentry: %s, task:%s\n",
						 __func__, dentry->d_name.name, current->comm);
		goto out_err;
        }

	
	OVERRIDE_CRED_PTR(SDCARDFS_SB(dir->i_sb), saved_cred);

	sdcardfs_get_lower_path(parent, &lower_parent_path);

	
	err = new_dentry_private_data(dentry);
	if (err) {
		ret = ERR_PTR(err);
		goto out;
	}

	ret = __sdcardfs_lookup(dentry, flags, &lower_parent_path, SDCARDFS_I(dir)->userid);
	if (IS_ERR(ret))
	{
		goto out;
	}
	if (ret)
		dentry = ret;
	if (dentry->d_inode) {
		fsstack_copy_attr_times(dentry->d_inode,
					sdcardfs_lower_inode(dentry->d_inode));
		
		get_derived_permission(parent, dentry);
		fix_derived_permission(dentry->d_inode);
	}
	
	fsstack_copy_attr_atime(parent->d_inode,
				sdcardfs_lower_inode(parent->d_inode));

out:
	sdcardfs_put_lower_path(parent, &lower_parent_path);
	REVERT_CRED(saved_cred);
out_err:
	dput(parent);
	return ret;
}
