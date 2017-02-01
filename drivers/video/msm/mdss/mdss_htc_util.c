/* Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/debugfs.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "mdss_htc_util.h"
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include <linux/CwMcuSensor.h>

struct attribute_status htc_attr_status[] = {
	{"cabc_level_ctl", 0, 0, 0},
	{"color_temp_ctl", UNDEF_VALUE, UNDEF_VALUE, UNDEF_VALUE},
	{"color_profile_ctl", 0, 0, 0},
	{"vddio_switch", 0, 0, 0},
	{"burst_switch", 0, 0, 0},
};

#define DEBUG_BUF   2048
#define MIN_COUNT   9
#define DCS_MAX_CNT   128

static char debug_buf[DEBUG_BUF];
struct mdss_dsi_ctrl_pdata *ctrl_instance = NULL;
static char dcs_cmds[DCS_MAX_CNT];
static char *tmp;
static struct dsi_cmd_desc debug_cmd = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, 1}, dcs_cmds
};
static char dsi_rbuf[4];
static void dsi_read_cb(int len)
{
	unsigned *lp;

	lp = (uint32_t *)dsi_rbuf;
	pr_info("%s: data=0x%x len=%d\n", __func__,*lp, len);
}
static ssize_t dsi_cmd_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	u32 type = 0, value = 0;
	int cnt, i;
	struct dcs_cmd_req cmdreq;

	if (count >= sizeof(debug_buf) || count < MIN_COUNT)
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	if (!ctrl_instance)
		return count;

	/* end of string */
	debug_buf[count] = 0;

	/* Format:
	ex: echo 39 51 ff > dsi_cmd
	read ex: echo 06 52 00 > dsi_cmd
	     [type] space [addr] space [value]
	     +--+--+-----+--+--+------+--+--+-
	bit   0  1    2   3  4     5   6  7
	ex:    39          51           ff
	*/
	/* Calc the count, min count = 9, format: type addr value */
	cnt = (count) / 3 - 1;
	debug_cmd.dchdr.dlen = cnt;

	/* Get the type */
	sscanf(debug_buf, "%x", &type);

	if (type == DTYPE_DCS_LWRITE)
		debug_cmd.dchdr.dtype = DTYPE_DCS_LWRITE;
	else if (type == DTYPE_GEN_LWRITE)
		debug_cmd.dchdr.dtype = DTYPE_GEN_LWRITE;
	else if (type == DTYPE_DCS_READ)
		debug_cmd.dchdr.dtype = DTYPE_DCS_READ;
	else
		return -EFAULT;

	pr_info("%s: cnt=%d, type=0x%x\n", __func__, cnt, type);

	/* Get the cmd and value */
	for (i = 0; i < cnt; i++) {
		if (i >= DCS_MAX_CNT) {
			pr_info("%s: DCS command count over DCS_MAX_CNT, Skip these commands.\n", __func__);
			break;
		}
		tmp = debug_buf + (3 * (i + 1));
		sscanf(tmp, "%x", &value);
		dcs_cmds[i] = value;
		pr_info("%s: value=0x%x\n", __func__, dcs_cmds[i]);
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	memset(&dsi_rbuf, 0, sizeof(dsi_rbuf));

	if (type == DTYPE_DCS_READ){
		cmdreq.cmds = &debug_cmd;
		cmdreq.cmds_cnt = 1;
		cmdreq.flags = CMD_REQ_COMMIT | CMD_REQ_RX;
		cmdreq.rlen = 4;
		cmdreq.rbuf = dsi_rbuf;
		cmdreq.cb = dsi_read_cb; /* call back */

		mdss_dsi_cmdlist_put(ctrl_instance, &cmdreq);
	} else {
		cmdreq.cmds = &debug_cmd;
		cmdreq.cmds_cnt = 1;
		cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;
		mdss_dsi_cmdlist_put(ctrl_instance, &cmdreq);
		pr_info("%s %ld\n", __func__, count);
	}
	return count;
}

static const struct file_operations dsi_cmd_fops = {
	.write = dsi_cmd_write,
};

void htc_debugfs_init(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct dentry *dent = debugfs_create_dir("htc_debug", NULL);

	pr_info("%s\n", __func__);

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_instance = container_of(pdata, struct mdss_dsi_ctrl_pdata,
						panel_data);

	if (IS_ERR(dent)) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return;
	}

	if (debugfs_create_file("dsi_cmd", 0644, dent, 0, &dsi_cmd_fops)
			== NULL) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return;
	}
	return;
}

static unsigned backlightvalue = 0;
static ssize_t camera_bl_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	ret = scnprintf(buf, PAGE_SIZE, "%s%u\n", "BL_CAM_MIN=", backlightvalue);
	return ret;
}

static ssize_t attrs_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(htc_attr_status); i++) {
		if (!strcmp(attr->attr.name, htc_attr_status[i].title)) {
			ret = scnprintf(buf, PAGE_SIZE, "%d\n", htc_attr_status[i].cur_value);
			break;
		}
	}

	return ret;
}

static ssize_t attr_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	unsigned long res;
	int rc, i;

	rc = kstrtoul(buf, 10, &res);
	if (rc) {
		pr_err("invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}

	for (i = 0; i < ARRAY_SIZE(htc_attr_status); i++) {
		if (!strcmp(attr->attr.name, htc_attr_status[i].title)) {
			htc_attr_status[i].req_value = res;
			break;
		}
	}

err_out:
	return count;
}

static ssize_t switch_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	int ret;
	struct msm_fb_data_type *mfd = dev_get_drvdata(dev->parent);
	ret = attr_store(dev, attr, buf, count);
	htc_set_vddio_switch(mfd);
	return ret;
}

/*
HTC native mipi command format :

	"format string", < sleep ms >,  <cmd leng>, ['cmd bytes'...] ;

	"format string" :
		"DTYPE_DCS_WRITE"  : 0x05
		"DTYPE_DCS_WRITE1" : 0x15
		"DTYPE_DCS_LWRITE" : 0x39
		"DTYPE_GEN_WRITE"  : 0x03
		"DTYPE_GEN_WRITE1" : 0x13
		"DTYPE_GEN_WRITE2" : 0x23
		"DTYPE_GEN_LWRITE" : 0x29

	Example :
		htc-fmt,mdss-dsi-off-command =
					"DTYPE_DCS_WRITE", <1>, <0x02>, [28 00],
					"DTYPE_DCS_WRITE", <120>, <0x02>, [10 00];
		htc-fmt,display-on-cmds = "DTYPE_DCS_WRITE", <0x0A>, <0x02>, [29 00];
		htc-fmt,cabc-off-cmds = "DTYPE_DCS_LWRITE", <1>, <0x02>, [55 00];
		htc-fmt,cabc-ui-cmds =
			"DTYPE_DCS_LWRITE", <5>, <0x02>, [55 02],
			"DTYPE_DCS_LWRITE", <1>, <0x02>, [5E 11],
			"DTYPE_DCS_LWRITE", <1>, <0x0A>, [CA 2D 27 26 25 24 22 21 21 20],
			"DTYPE_DCS_LWRITE", <1>, <0x23>, [CE 00 00 00 00 10 10 16 16 16 16 16 16 16 16 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00];

*/

#define SLEEPMS_OFFSET(strlen) (strlen+1) /* < sleep ms>, [cmd len ... */
#define CMDLEN_OFFSET(strlen)  (SLEEPMS_OFFSET(strlen)+sizeof(const __be32))
#define CMD_OFFSET(strlen)     (CMDLEN_OFFSET(strlen)+sizeof(const __be32))

static struct __dsi_cmd_map{
	char *cmdtype_str;
	int  cmdtype_strlen;
	int  dtype;
} dsi_cmd_map[] = {
	{ "DTYPE_DCS_WRITE", 0, DTYPE_DCS_WRITE },
	{ "DTYPE_DCS_WRITE1", 0, DTYPE_DCS_WRITE1 },
	{ "DTYPE_DCS_LWRITE", 0, DTYPE_DCS_LWRITE },
	{ "DTYPE_GEN_WRITE", 0, DTYPE_GEN_WRITE },
	{ "DTYPE_GEN_WRITE1", 0, DTYPE_GEN_WRITE1 },
	{ "DTYPE_GEN_WRITE2", 0, DTYPE_GEN_WRITE2 },
	{ "DTYPE_GEN_LWRITE", 0, DTYPE_GEN_LWRITE },
	{ NULL, 0, 0 }
};

int htc_mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len = 0;
	char *buf;
	struct property *prop;
	struct dsi_ctrl_hdr *pdchdr;
	int i, cnt;
	int curcmdtype;

	i = 0;
	while (dsi_cmd_map[i].cmdtype_str) {
		if (!dsi_cmd_map[i].cmdtype_strlen) {
			dsi_cmd_map[i].cmdtype_strlen = strlen(dsi_cmd_map[i].cmdtype_str);
		}
		i++;
	}

	prop = of_find_property( np, cmd_key, &len);
	if (!prop || !len || !(prop->length) || !(prop->value)) {
		pr_err("%s: failed, key=%s  [%d : %d : %p]\n", __func__, cmd_key,
			len, (prop ? prop->length : -1), (prop ? prop->value : 0) );
		//pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	data = prop->value;
	blen = 0;
	cnt = 0;
	while (len > 0) {
		curcmdtype = 0;
		while (dsi_cmd_map[curcmdtype].cmdtype_strlen) {
			if( !strncmp( data, dsi_cmd_map[curcmdtype].cmdtype_str,
						dsi_cmd_map[curcmdtype].cmdtype_strlen ) &&
				data[dsi_cmd_map[curcmdtype].cmdtype_strlen] == '\0' )
				break;
			curcmdtype++;
		};
		if( !dsi_cmd_map[curcmdtype].cmdtype_strlen ) /* no matching */
			break;

		i = be32_to_cpup((__be32 *)&data[CMDLEN_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen)]);
		blen += i;
		cnt++;

		data = data + CMD_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen) + i;
		len = len - CMD_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen) - i;
	}

	if(len || !cnt || !blen){
		pr_err("%s: failed, key[%s] : %d cmds, remain=%d bytes \n", __func__, cmd_key, cnt, len);
		return -ENOMEM;
	}

	i = (sizeof(char)*blen+sizeof(struct dsi_ctrl_hdr)*cnt);
	buf = kzalloc( i, GFP_KERNEL);
	if (!buf){
		pr_err("%s: create dsi ctrl oom failed \n", __func__);
		return -ENOMEM;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc), GFP_KERNEL);
	if (!pcmds->cmds){
		pr_err("%s: create dsi commands oom failed \n", __func__);
		goto exit_free;
	}

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = i;
	data = prop->value;
	for(i=0; i<cnt; i++){
		pdchdr = &pcmds->cmds[i].dchdr;

		curcmdtype = 0;
		while(dsi_cmd_map[curcmdtype].cmdtype_strlen){
			if( !strncmp( data, dsi_cmd_map[curcmdtype].cmdtype_str,
						dsi_cmd_map[curcmdtype].cmdtype_strlen ) &&
				data[dsi_cmd_map[curcmdtype].cmdtype_strlen] == '\0' ){
				pdchdr->dtype = dsi_cmd_map[curcmdtype].dtype;
				break;
			}
			curcmdtype ++;
		}

		pdchdr->last = 0x01;
		pdchdr->vc = 0x00;
		pdchdr->ack = 0x00;
		pdchdr->wait = be32_to_cpup((__be32 *)&data[SLEEPMS_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen)]) & 0xff;
		pdchdr->dlen = be32_to_cpup((__be32 *)&data[CMDLEN_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen)]);
		memcpy( buf, pdchdr, sizeof(struct dsi_ctrl_hdr) );
		buf += sizeof(struct dsi_ctrl_hdr);
		memcpy( buf, &data[CMD_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen)], pdchdr->dlen);
		pcmds->cmds[i].payload = buf;
		buf += pdchdr->dlen;
		data = data + CMD_OFFSET(dsi_cmd_map[curcmdtype].cmdtype_strlen) + pdchdr->dlen;
	}

	data = of_get_property(np, link_key, NULL);
	if (data) {
		if (!strncmp(data, "dsi_hs_mode", 11))
			pcmds->link_state = DSI_HS_MODE;
		else
			pcmds->link_state = DSI_LP_MODE;
	} else {
		pcmds->link_state = DSI_HS_MODE;
	}
	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}

static DEVICE_ATTR(backlight_info, S_IRUGO, camera_bl_show, NULL);
static DEVICE_ATTR(cabc_level_ctl, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(color_temp_ctl, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(color_profile_ctl, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(vddio_switch, S_IRUGO | S_IWUSR, attrs_show, switch_store);
static DEVICE_ATTR(burst_switch, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static struct attribute *htc_extend_attrs[] = {
	&dev_attr_backlight_info.attr,
	&dev_attr_cabc_level_ctl.attr,
	&dev_attr_color_temp_ctl.attr,
	&dev_attr_color_profile_ctl.attr,
	&dev_attr_vddio_switch.attr,
	&dev_attr_burst_switch.attr,
	NULL,
};

static struct attribute_group htc_extend_attr_group = {
	.attrs = htc_extend_attrs,
};

void htc_register_attrs(struct kobject *led_kobj, struct msm_fb_data_type *mfd)
{
	int rc;

	pr_err("htc_register_attrs\n");

	rc = sysfs_create_group(led_kobj, &htc_extend_attr_group);
	if (rc)
		pr_err("sysfs group creation failed, rc=%d\n", rc);

	mfd->compass_notifier_block.notifier_call = compass_notifier_fn;
	compass_en_register_notifier(&mfd->compass_notifier_block);
	return;
}

void htc_reset_status(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(htc_attr_status); i++) {
		htc_attr_status[i].cur_value = htc_attr_status[i].def_value;
	}

	return;
}

void htc_register_camera_bkl(int level)
{
	backlightvalue = level;
}

void htc_set_cabc(struct msm_fb_data_type *mfd, bool force)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (htc_attr_status[CABC_INDEX].req_value > 2)
		return;

	if (!ctrl_pdata->cabc_off_cmds.cmds)
		return;

	if (!ctrl_pdata->cabc_ui_cmds.cmds)
		return;

	if (!ctrl_pdata->cabc_video_cmds.cmds)
		return;

	if (!force && (htc_attr_status[CABC_INDEX].req_value == htc_attr_status[CABC_INDEX].cur_value))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (htc_attr_status[CABC_INDEX].req_value == 0) {
		cmdreq.cmds = ctrl_pdata->cabc_off_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_off_cmds.cmd_cnt;
	} else if (htc_attr_status[CABC_INDEX].req_value == 1) {
		cmdreq.cmds = ctrl_pdata->cabc_ui_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_ui_cmds.cmd_cnt;
	} else if (htc_attr_status[CABC_INDEX].req_value == 2) {
		cmdreq.cmds = ctrl_pdata->cabc_video_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_video_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->cabc_ui_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_ui_cmds.cmd_cnt;
	}

	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[CABC_INDEX].cur_value = htc_attr_status[CABC_INDEX].req_value;
	pr_info("%s: cabc mode=%d\n", __func__, htc_attr_status[CABC_INDEX].cur_value);
	return;
}

void htc_set_color_temp(struct msm_fb_data_type *mfd, bool force)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;
	int req_mode = 0;
	int i = 0;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (!ctrl_pdata->color_temp_cnt)
		return;

	for (i = 0; i < ctrl_pdata->color_temp_cnt ; i++) {
		if (!ctrl_pdata->color_temp_cmds[i].cmds)
			return;
	}

	if (htc_attr_status[COLOR_TEMP_INDEX].req_value >= ctrl_pdata->color_temp_cnt)
		return;

	if (!force && (htc_attr_status[COLOR_TEMP_INDEX].req_value == htc_attr_status[COLOR_TEMP_INDEX].cur_value))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	req_mode = htc_attr_status[COLOR_TEMP_INDEX].req_value;
	cmdreq.cmds = ctrl_pdata->color_temp_cmds[req_mode].cmds;
	cmdreq.cmds_cnt = ctrl_pdata->color_temp_cmds[req_mode].cmd_cnt;

	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[COLOR_TEMP_INDEX].cur_value = htc_attr_status[COLOR_TEMP_INDEX].req_value;
	pr_info("%s: color temp mode=%d\n", __func__, htc_attr_status[COLOR_TEMP_INDEX].cur_value);
	return;
}

void htc_set_color_profile(struct msm_fb_data_type *mfd, bool force)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (!ctrl_pdata->color_default_cmds.cmd_cnt || !ctrl_pdata->color_srgb_cmds.cmd_cnt)
		return;

	if ((htc_attr_status[COLOR_PROFILE_INDEX].req_value > SRGB_MODE) || (htc_attr_status[COLOR_PROFILE_INDEX].req_value < DEFAULT_MODE))
		return;

	if (!force && (htc_attr_status[COLOR_PROFILE_INDEX].req_value == htc_attr_status[COLOR_PROFILE_INDEX].cur_value))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (htc_attr_status[COLOR_PROFILE_INDEX].req_value == SRGB_MODE) {
		cmdreq.cmds = ctrl_pdata->color_srgb_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->color_srgb_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->color_default_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->color_default_cmds.cmd_cnt;
	}

	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[COLOR_PROFILE_INDEX].cur_value = htc_attr_status[COLOR_PROFILE_INDEX].req_value;
	pr_info("%s: color profile mode=%d\n", __func__, htc_attr_status[COLOR_PROFILE_INDEX].cur_value);
	return;
}

void compass_set_vddio_switch(struct msm_fb_data_type *mfd, int enable)
{
	struct mdss_mdp_ctl *ctl = mfd_to_ctl(mfd);
	int event = enable ? MDSS_EVENT_PANEL_VDDIO_SWITCH_ON : MDSS_EVENT_PANEL_VDDIO_SWITCH_OFF;

	mdss_mdp_ctl_intf_event(ctl, event, NULL, CTL_INTF_EVENT_FLAG_DEFAULT);
}

void htc_set_vddio_switch(struct msm_fb_data_type *mfd)
{
	if (htc_attr_status[VDDIO_INDEX].req_value == htc_attr_status[VDDIO_INDEX].cur_value)
		return;

	compass_set_vddio_switch(mfd, htc_attr_status[VDDIO_INDEX].req_value);

	htc_attr_status[VDDIO_INDEX].cur_value = htc_attr_status[VDDIO_INDEX].req_value;
	pr_info("%s: vddio switch=%d\n", __func__, htc_attr_status[VDDIO_INDEX].cur_value);
	return;
}

int compass_notifier_fn(struct notifier_block *nb,
                        unsigned long action, void *data)
{
	struct msm_fb_data_type *mfd;
	mfd = container_of(nb, struct msm_fb_data_type, compass_notifier_block);
	pr_info("%s: action=%d\n", __func__, (int)action);

	compass_set_vddio_switch(mfd, (action) ? 1 : 0);

	return 0;
}

void htc_set_burst(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;
	static int burst_mode = 0;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (ctrl_pdata->burst_on_level == 0 || ctrl_pdata->burst_off_level == 0)
		return;

	if (htc_attr_status[BURST_SWITCH_INDEX].req_value < 0)
		return;

	if (!ctrl_pdata->burst_off_cmds.cmds)
		return;

	if (!ctrl_pdata->burst_on_cmds.cmds)
		return;

	if (htc_attr_status[BURST_SWITCH_INDEX].req_value == htc_attr_status[BURST_SWITCH_INDEX].cur_value)
		return;

	if(burst_mode == 1 && htc_attr_status[BURST_SWITCH_INDEX].req_value <= ctrl_pdata->burst_off_level) {
		burst_mode = 0;
	} else if (burst_mode == 0 && htc_attr_status[BURST_SWITCH_INDEX].req_value >= ctrl_pdata->burst_on_level){
		burst_mode = 1;
	} else {
		htc_attr_status[BURST_SWITCH_INDEX].cur_value = htc_attr_status[BURST_SWITCH_INDEX].req_value;
		return;
	}

	if(!mdss_fb_is_power_on(mfd))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (burst_mode) {
		cmdreq.cmds = ctrl_pdata->burst_on_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->burst_on_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->burst_off_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->burst_off_cmds.cmd_cnt;
	}

	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[BURST_SWITCH_INDEX].cur_value = htc_attr_status[BURST_SWITCH_INDEX].req_value;
	pr_info("%s burst mode=%d\n", __func__, htc_attr_status[BURST_SWITCH_INDEX].cur_value);
	return;
}
