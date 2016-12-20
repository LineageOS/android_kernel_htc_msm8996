/*
 * Copyright (C) 2015 HTC, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/of.h>

#define DEVICE_TREE_MISC_PATH "/chosen/misc"
#define SERIAL_NUM_DECIMAL_SIZE 20

#define PROC_MSM_SERIAL "driver/msm_serial"

#define LABEL_MSM_SERIAL_NUM "QFPROM_CORR_MSM_SERIAL_NUM"


#if 1
#define SECMSG(s...) pr_info("[SECURITY] "s)
#define SECERR(s...) pr_err("[SECURITY] "s)
#else
#define SECMSG(s...) do{} while(0)
#define SECERR(s...) pr_err("[SECURITY] "s)
#endif

static char htc_msm_serial_num[SERIAL_NUM_DECIMAL_SIZE+1]={0};

static int htc_msm_serial_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", htc_msm_serial_num);
    return 0;
}

static int htc_msm_serial_open(struct inode *inode, struct file *file)
{
    return single_open(file, htc_msm_serial_read, NULL);
}

static const struct file_operations htc_msm_serial_fops = {
    .owner      = THIS_MODULE,
    .open       = htc_msm_serial_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static void init_from_device_tree(void)
{
    struct device_node *misc_node = NULL;
    char *data = NULL;
    int property_size = 0;

    misc_node = of_find_node_by_path(DEVICE_TREE_MISC_PATH);
    if(NULL == misc_node)
        return;

	data = (char *)of_get_property(misc_node, LABEL_MSM_SERIAL_NUM, &property_size);
	SECMSG("%s - length: %d\n", __func__, property_size);
	if(property_size > sizeof(htc_msm_serial_num))
		return;

	SECMSG("%s - LABEL_MSM_SERIAL_NUM: %s\n", __func__, data);
	memcpy(htc_msm_serial_num, data, sizeof(htc_msm_serial_num));

}

static int __init msm_serial_proc_init(void)
{
    struct proc_dir_entry *entry = NULL;

    SECMSG("%s: Init HTC msm-serial proc interface.\n", __func__);

    init_from_device_tree();

    /* NOTE: kernel 3.10 use proc_create_data to create /proc file node */
    entry = proc_create_data(PROC_MSM_SERIAL, 0664, NULL, &htc_msm_serial_fops, NULL);
    if (entry == NULL) {
        SECERR("%s: unable to create /proc%s entry\n", __func__, PROC_MSM_SERIAL);
        return -ENOMEM;
    }

    return 0;
}


module_init(msm_serial_proc_init);
MODULE_AUTHOR("HTC");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
