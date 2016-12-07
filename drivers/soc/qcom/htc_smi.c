/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <uapi/linux/htc_smi.h>
#include <soc/qcom/scm.h>
#include <qseecom_kernel_htc.h>

#include <soc/qcom/msm_qmi_interface.h>
#define HTC_SMI_SNS_SERVICE_ID		0x138 
#define HTC_SMI_SNS_SERVICE_VER_ID	1
#define HTC_SMI_SNS_INSTANCE_INST_ID	0

static void htc_smi_qmi_clnt_svc_arrive(struct work_struct *work);
static void htc_smi_qmi_clnt_svc_exit(struct work_struct *work);
static void htc_smi_qmi_clnt_recv_msg(struct work_struct *work);

#define HTC_SMI_DEV "htc_smi"
#define HTC_SMI_IN_DEV_VERSION 0x0100

#define TZ_BLSP_MODIFY_OWNERSHIP_ARGINFO    2
#define TZ_BLSP_MODIFY_OWNERSHIP_SVC_ID     4 
#define TZ_BLSP_MODIFY_OWNERSHIP_FUNC_ID    3

enum sensor_connection_types {
	SPI,
	SSC_SPI,
};

static uint32_t g_app_buf_size = SZ_256K;

struct htc_smi_drvdata {
	struct class	*htc_smi_class;
	struct cdev	htc_smi_cdev;
	struct device	*dev;
	char		*htc_smi_node;
	enum sensor_connection_types sensor_conn_type;
	struct clk	**clocks;
	unsigned	clock_count;
	uint8_t		clock_state;
	uint8_t		ssc_state;
	unsigned	root_clk_idx;
	unsigned	frequency;
	atomic_t	available;
	struct mutex	mutex;
	struct work_struct qmi_svc_arrive;
	struct work_struct qmi_svc_exit;
	struct work_struct qmi_svc_rcv_msg;
	struct qmi_handle *qmi_handle;
	struct notifier_block qmi_svc_notifier;
	uint32_t	tz_subsys_id;
	uint32_t	ssc_subsys_id;
	uint32_t	ssc_spi_port;
	uint32_t	ssc_spi_port_slave_index;
};

static int get_cmd_rsp_buffers(struct qseecom_handle *hdl,
	void **cmd,
	uint32_t *cmd_len,
	void **rsp,
	uint32_t *rsp_len)
{
	
	*cmd_len = ALIGN(*cmd_len, 64);
	*rsp_len = ALIGN(*rsp_len, 64);

	if ((*rsp_len + *cmd_len) > g_app_buf_size)
		return -ENOMEM;

	*cmd = hdl->sbuf;
	*rsp = hdl->sbuf + *cmd_len;

	return 0;
}

#define SNS_SPI_OPEN_REQ_V01 0x0020
#define SNS_SPI_OPEN_RESP_V01 0x0020
#define SNS_SPI_KEEP_ALIVE_REQ_V01 0x0022
#define SNS_SPI_KEEP_ALIVE_RESP_V01 0x0022
#define SNS_SPI_CLOSE_REQ_V01 0x0021
#define SNS_SPI_CLOSE_RESP_V01 0x0021
#define SNS_SPI_VERSION_REQ_V01 0x0001
#define TIMEOUT_MS			(500)

struct sns_common_resp_s_v01 {
	uint8_t sns_result_t;
	uint8_t sns_err_t;
};

struct sns_spi_open_req_msg_v01 {
	uint8_t port_id;
	uint8_t slave_index;
	uint32_t freq;
};
#define SNS_SPI_OPEN_REQ_MSG_V01_MAX_MSG_LEN 15

struct sns_spi_open_resp_msg_v01 {
	struct sns_common_resp_s_v01 resp;
};
#define SNS_SPI_OPEN_RESP_MSG_V01_MAX_MSG_LEN 5

struct sns_spi_close_req_msg_v01 {
	char placeholder;
};
#define SNS_SPI_CLOSE_REQ_MSG_V01_MAX_MSG_LEN 0

struct sns_spi_close_resp_msg_v01 {
	struct sns_common_resp_s_v01 resp;
};
#define SNS_SPI_CLOSE_RESP_MSG_V01_MAX_MSG_LEN 5

struct sns_spi_keep_alive_req_msg_v01 {
	uint8_t enable;
};
#define SNS_SPI_KEEP_ALIVE_REQ_MSG_V01_MAX_MSG_LEN 4

struct sns_spi_keep_alive_resp_msg_v01 {
	struct sns_common_resp_s_v01 resp;
};
#define SNS_SPI_KEEP_ALIVE_RESP_MSG_V01_MAX_MSG_LEN 5

static struct elem_info sns_common_resp_s_v01_ei[] = {
	{
		.data_type    = QMI_UNSIGNED_1_BYTE,
		.elem_len     = 1,
		.elem_size    = sizeof(uint8_t),
		.is_array     = NO_ARRAY,
		.tlv_type     = 0,
		.offset       = offsetof(struct sns_common_resp_s_v01,
					 sns_result_t),
	},
	{
		.data_type    = QMI_UNSIGNED_1_BYTE,
		.elem_len     = 1,
		.elem_size    = sizeof(uint8_t),
		.is_array     = NO_ARRAY,
		.tlv_type     = 0,
		.offset       = offsetof(struct sns_common_resp_s_v01,
					 sns_err_t),
	},
	{
		.data_type    = QMI_EOTI,
		.is_array     = NO_ARRAY,
		.is_array     = QMI_COMMON_TLV_TYPE,
	},
};

static struct elem_info sns_spi_open_req_msg_v01_ei[] = {
	{
		.data_type    = QMI_UNSIGNED_1_BYTE,
		.elem_len     = 1,
		.elem_size    = sizeof(uint8_t),
		.is_array     = NO_ARRAY,
		.tlv_type     = 0x01,
		.offset       = offsetof(struct sns_spi_open_req_msg_v01,
					 port_id),
	},
	{
		.data_type    = QMI_UNSIGNED_1_BYTE,
		.elem_len     = 1,
		.elem_size    = sizeof(uint8_t),
		.is_array     = NO_ARRAY,
		.tlv_type     = 0x02,
		.offset       = offsetof(struct sns_spi_open_req_msg_v01,
					 slave_index),
	},
	{
		.data_type    = QMI_UNSIGNED_4_BYTE,
		.elem_len     = 1,
		.elem_size    = sizeof(uint32_t),
		.is_array     = NO_ARRAY,
		.tlv_type     = 0x03,
		.offset       = offsetof(struct sns_spi_open_req_msg_v01,
					 freq),
	},
	{
		.data_type    = QMI_EOTI,
		.is_array     = NO_ARRAY,
		.is_array     = QMI_COMMON_TLV_TYPE,
	},
};

static struct elem_info sns_spi_open_resp_msg_v01_ei[] = {
	{
		.data_type    = QMI_STRUCT,
		.elem_len     = 1,
		.elem_size    = sizeof(struct sns_common_resp_s_v01),
		.is_array     = NO_ARRAY,
		.tlv_type     = 0x01,
		.offset       = offsetof(struct sns_spi_open_resp_msg_v01,
					 resp),
		.ei_array     = sns_common_resp_s_v01_ei,
	},
	{
		.data_type    = QMI_EOTI,
		.is_array     = NO_ARRAY,
		.is_array     = QMI_COMMON_TLV_TYPE,
	},
};

static struct elem_info sns_spi_close_req_msg_v01_ei[] = {
	{
		.data_type    = QMI_EOTI,
		.is_array     = NO_ARRAY,
		.is_array     = QMI_COMMON_TLV_TYPE,
	},
};

static struct elem_info sns_spi_close_resp_msg_v01_ei[] = {
	{
		.data_type    = QMI_STRUCT,
		.elem_len     = 1,
		.elem_size    = sizeof(struct sns_common_resp_s_v01),
		.is_array     = NO_ARRAY,
		.tlv_type     = 0x01,
		.offset       = offsetof(struct sns_spi_close_resp_msg_v01,
					 resp),
		.ei_array     = sns_common_resp_s_v01_ei,
	},
	{
		.data_type    = QMI_EOTI,
		.is_array     = NO_ARRAY,
		.is_array     = QMI_COMMON_TLV_TYPE,
	},
};

static struct elem_info sns_spi_keep_alive_req_msg_v01_ei[] = {
	{
		.data_type    = QMI_UNSIGNED_1_BYTE,
		.elem_len     = 1,
		.elem_size    = sizeof(uint8_t),
		.is_array     = NO_ARRAY,
		.tlv_type     = 0x01,
		.offset       = offsetof(struct sns_spi_keep_alive_req_msg_v01,
					 enable),
	},
	{
		.data_type    = QMI_EOTI,
		.is_array     = NO_ARRAY,
		.is_array     = QMI_COMMON_TLV_TYPE,
	},
};

static struct elem_info sns_spi_keep_alive_resp_msg_v01_ei[] = {
	{
		.data_type    = QMI_STRUCT,
		.elem_len     = 1,
		.elem_size    = sizeof(struct sns_common_resp_s_v01),
		.is_array     = NO_ARRAY,
		.tlv_type     = 0x01,
		.offset       = offsetof(struct sns_spi_keep_alive_resp_msg_v01,
					 resp),
		.ei_array     = sns_common_resp_s_v01_ei,
	},
	{
		.data_type    = QMI_EOTI,
		.is_array     = NO_ARRAY,
		.is_array     = QMI_COMMON_TLV_TYPE,
	},
};

static int htc_smi_sns_open_req(struct htc_smi_drvdata *drvdata)
{
	struct sns_spi_open_req_msg_v01 req;
	struct sns_spi_open_resp_msg_v01 resp = { { 0, 0 } };

	struct msg_desc req_desc, resp_desc;
	int ret = -EINVAL;

	mutex_lock(&drvdata->mutex);

	if (!drvdata->qmi_handle) {
		dev_info(drvdata->dev,
			 "%s: QMI service unavailable. Wait 3 seconds\n",
			 __func__);
		msleep(3000);
		if (!drvdata->qmi_handle) {
			dev_info(drvdata->dev,
				 "%s: QMI service unavailable. Skipping QMI requests\n",
				 __func__);
			goto err;
		}
		dev_info(drvdata->dev,
			 "%s: QMI service Available.\n",
			 __func__);
	}

	req.port_id = drvdata->ssc_spi_port;
	req.slave_index = drvdata->ssc_spi_port_slave_index;
	req.freq = drvdata->frequency;

	req_desc.msg_id = SNS_SPI_OPEN_REQ_V01;
	req_desc.max_msg_len = SNS_SPI_OPEN_REQ_MSG_V01_MAX_MSG_LEN;
	req_desc.ei_array = sns_spi_open_req_msg_v01_ei;

	resp_desc.msg_id = SNS_SPI_OPEN_RESP_V01;
	resp_desc.max_msg_len = SNS_SPI_OPEN_RESP_MSG_V01_MAX_MSG_LEN;
	resp_desc.ei_array = sns_spi_open_resp_msg_v01_ei;

	ret = qmi_send_req_wait(drvdata->qmi_handle,
				&req_desc, &req, sizeof(req),
				&resp_desc, &resp, sizeof(resp),
				TIMEOUT_MS);

	if (ret < 0) {
		dev_err(drvdata->dev, "%s: QMI send req failed %d\n", __func__,
			ret);
		goto err;
	}

	if (resp.resp.sns_result_t != 0) {
		dev_err(drvdata->dev, "%s: QMI request failed %d %d\n",
			__func__, resp.resp.sns_result_t, resp.resp.sns_err_t);
		ret = -EREMOTEIO;
		goto err;
	}

err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static int htc_smi_sns_keep_alive_req(struct htc_smi_drvdata *drvdata,
				      uint8_t enable)
{
	struct sns_spi_keep_alive_req_msg_v01 req;
	struct sns_spi_keep_alive_resp_msg_v01 resp = { { 0, 0 } };

	struct msg_desc req_desc, resp_desc;
	int ret = -EINVAL;

	mutex_lock(&drvdata->mutex);

	if (!drvdata->qmi_handle) {
		dev_info(drvdata->dev,
			 "%s: QMI service unavailable. Wait 3 seconds\n",
			 __func__);
		msleep(3000);
		if (!drvdata->qmi_handle) {
			dev_info(drvdata->dev,
				 "%s: QMI service unavailable. Skipping QMI requests\n",
				 __func__);
			goto err;
		}
		dev_info(drvdata->dev,
			 "%s: QMI service Available.\n",
			 __func__);
	}

	req.enable = enable;

	req_desc.msg_id = SNS_SPI_KEEP_ALIVE_REQ_V01;
	req_desc.max_msg_len = SNS_SPI_KEEP_ALIVE_REQ_MSG_V01_MAX_MSG_LEN;
	req_desc.ei_array = sns_spi_keep_alive_req_msg_v01_ei;

	resp_desc.msg_id = SNS_SPI_KEEP_ALIVE_RESP_V01;
	resp_desc.max_msg_len = SNS_SPI_KEEP_ALIVE_RESP_MSG_V01_MAX_MSG_LEN;
	resp_desc.ei_array = sns_spi_keep_alive_resp_msg_v01_ei;

	ret = qmi_send_req_wait(drvdata->qmi_handle,
				&req_desc, &req, sizeof(req),
				&resp_desc, &resp, sizeof(resp),
				TIMEOUT_MS);

	if (ret < 0) {
		dev_err(drvdata->dev, "%s: QMI send req failed %d\n", __func__,
			ret);
		goto err;
	}

	if (resp.resp.sns_result_t != 0) {
		dev_err(drvdata->dev, "%s: QMI request failed %d %d\n",
			__func__, resp.resp.sns_result_t, resp.resp.sns_err_t);
		ret = -EREMOTEIO;
		goto err;
	}

err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static int htc_smi_sns_close_req(struct htc_smi_drvdata *drvdata)
{
	struct sns_spi_close_req_msg_v01 req;
	struct sns_spi_close_resp_msg_v01 resp = { { 0, 0 } };

	struct msg_desc req_desc, resp_desc;
	int ret = -EINVAL;

	mutex_lock(&drvdata->mutex);

	if (!drvdata->qmi_handle) {
		dev_info(drvdata->dev,
			 "%s: QMI service unavailable. Wait 3 seconds\n",
			 __func__);
		msleep(3000);
		if (!drvdata->qmi_handle) {
			dev_info(drvdata->dev,
				 "%s: QMI service unavailable. Skipping QMI requests\n",
				 __func__);
			goto err;
		}
		dev_info(drvdata->dev,
			 "%s: QMI service Available.\n",
			 __func__);
	}

	req.placeholder = 1;

	req_desc.msg_id = SNS_SPI_CLOSE_REQ_V01;
	req_desc.max_msg_len = SNS_SPI_CLOSE_REQ_MSG_V01_MAX_MSG_LEN;
	req_desc.ei_array = sns_spi_close_req_msg_v01_ei;

	resp_desc.msg_id = SNS_SPI_CLOSE_RESP_V01;
	resp_desc.max_msg_len = SNS_SPI_CLOSE_RESP_MSG_V01_MAX_MSG_LEN;
	resp_desc.ei_array = sns_spi_close_resp_msg_v01_ei;

	ret = qmi_send_req_wait(drvdata->qmi_handle,
				&req_desc, &req, sizeof(req),
				&resp_desc, &resp, sizeof(resp),
				TIMEOUT_MS);

	if (ret < 0) {
		dev_err(drvdata->dev, "%s: QMI send req failed %d\n", __func__,
			ret);
		goto err;
	}

	if (resp.resp.sns_result_t != 0) {
		dev_err(drvdata->dev, "%s: QMI request failed %d %d\n",
			__func__, resp.resp.sns_result_t, resp.resp.sns_err_t);
		ret = -EREMOTEIO;
		goto err;
	}

err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static void htc_smi_sns_notify(struct qmi_handle *handle,
			     enum qmi_event_type event, void *notify_priv)
{

	struct htc_smi_drvdata *drvdata =
					(struct htc_smi_drvdata *)notify_priv;

	switch (event) {
	case QMI_RECV_MSG:
		schedule_work(&drvdata->qmi_svc_rcv_msg);
		break;
	default:
		break;
	}
}

static int htc_smi_qmi_svc_event_notify(struct notifier_block *this,
					unsigned long code,
					void *_cmd)
{
	struct htc_smi_drvdata *drvdata = container_of(this,
						struct htc_smi_drvdata,
						qmi_svc_notifier);

	switch (code) {
	case QMI_SERVER_ARRIVE:
		schedule_work(&drvdata->qmi_svc_arrive);
		break;
	case QMI_SERVER_EXIT:
		schedule_work(&drvdata->qmi_svc_exit);
		break;
	default:
		break;
	}
	return 0;
}

static void htc_smi_qmi_ind_cb(struct qmi_handle *handle, unsigned int msg_id,
			void *msg, unsigned int msg_len, void *ind_cb_priv)
{
}

static void htc_smi_qmi_connect_to_service(struct htc_smi_drvdata *drvdata)
{
	int rc = 0;

	
	drvdata->qmi_handle = qmi_handle_create(htc_smi_sns_notify, drvdata);
	if (!drvdata->qmi_handle) {
		dev_err(drvdata->dev, "%s: QMI client handle alloc failed\n",
			__func__);
		return;
	}

	rc = qmi_connect_to_service(drvdata->qmi_handle,
				    HTC_SMI_SNS_SERVICE_ID,
				    HTC_SMI_SNS_SERVICE_VER_ID,
				    HTC_SMI_SNS_INSTANCE_INST_ID);
	if (rc < 0) {
		dev_err(drvdata->dev, "%s: Could not connect to SNS service\n",
			__func__);
		goto err;
	}

	rc = qmi_register_ind_cb(drvdata->qmi_handle, htc_smi_qmi_ind_cb,
							(void *)drvdata);
	if (rc < 0) {
		dev_err(drvdata->dev, "%s: Could not register the QMI ind cb\n",
			__func__);
		goto err;
	}

	return;

err:
	qmi_handle_destroy(drvdata->qmi_handle);
	drvdata->qmi_handle = NULL;
}

static void htc_smi_qmi_clnt_svc_arrive(struct work_struct *work)
{
	struct htc_smi_drvdata *drvdata = container_of(work,
					struct htc_smi_drvdata, qmi_svc_arrive);

	htc_smi_qmi_connect_to_service(drvdata);
}

static void htc_smi_qmi_clnt_svc_exit(struct work_struct *work)
{
	struct htc_smi_drvdata *drvdata = container_of(work,
					struct htc_smi_drvdata, qmi_svc_exit);

	qmi_handle_destroy(drvdata->qmi_handle);
	drvdata->qmi_handle = NULL;
}

static void htc_smi_qmi_clnt_recv_msg(struct work_struct *work)
{
	int rc;
	struct htc_smi_drvdata *drvdata = container_of(work,
				struct htc_smi_drvdata, qmi_svc_rcv_msg);

	do {
		rc = qmi_recv_msg(drvdata->qmi_handle);
	} while (rc == 0);
	if (rc != -ENOMSG)
		dev_err(drvdata->dev, "%s: Error receiving QMI message\n",
		 __func__);
}

static int htc_smi_set_blsp_ownership(struct htc_smi_drvdata *drvdata,
				      uint8_t owner_id)
{
	int rc = 0;
	struct scm_desc desc = {0};

	desc.arginfo = TZ_BLSP_MODIFY_OWNERSHIP_ARGINFO;
	desc.args[0] = drvdata->ssc_spi_port + 11;
	desc.args[1] = owner_id;


	rc = scm_call2(SCM_SIP_FNID(TZ_BLSP_MODIFY_OWNERSHIP_SVC_ID,
				    TZ_BLSP_MODIFY_OWNERSHIP_FUNC_ID),
				    &desc);

	if (rc < 0)
		dev_err(drvdata->dev, "%s: Error blsp ownership switch to %d\n",
			__func__, owner_id);

	return rc;
}

static int htc_smi_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	struct htc_smi_drvdata *drvdata = container_of(inode->i_cdev,
						   struct htc_smi_drvdata,
						   htc_smi_cdev);
	file->private_data = drvdata;

	
	if (!atomic_dec_and_test(&drvdata->available)) {
		atomic_inc(&drvdata->available);
		return -EBUSY;
	}

	if (drvdata->sensor_conn_type == SPI) {
	} else if (drvdata->sensor_conn_type == SSC_SPI) {
		rc = htc_smi_sns_open_req(drvdata);
		if (rc < 0) {
			dev_err(drvdata->dev, "%s: Error sensor open request\n",
				__func__);
			htc_smi_sns_close_req(drvdata);
			goto out;
		}
		rc = htc_smi_sns_keep_alive_req(drvdata, 1);
		if (rc < 0) {
			dev_err(drvdata->dev,
				"%s: Error sensor keep-alive request\n",
				 __func__);
			htc_smi_sns_close_req(drvdata);
			goto out;
		}
		rc = htc_smi_set_blsp_ownership(drvdata, drvdata->tz_subsys_id);
		if (rc < 0) {
			dev_err(drvdata->dev,
				"%s: Error setting blsp ownership\n", __func__);
			htc_smi_sns_keep_alive_req(drvdata, 0);
			htc_smi_sns_close_req(drvdata);
			goto out;
		}
		drvdata->ssc_state = 1;
	}

out:
	
	if (rc)
		atomic_inc(&drvdata->available);

	return rc;
}

static int htc_smi_release(struct inode *inode, struct file *file)
{
	struct htc_smi_drvdata *drvdata = file->private_data;

	if (drvdata->sensor_conn_type == SPI) {
	} else if (drvdata->sensor_conn_type == SSC_SPI) {
		htc_smi_set_blsp_ownership(drvdata, drvdata->ssc_subsys_id);
		htc_smi_sns_keep_alive_req(drvdata, 0);
		htc_smi_sns_close_req(drvdata);
		drvdata->ssc_state = 0;
	}

	atomic_inc(&drvdata->available);
	return 0;
}

static long htc_smi_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
	int rc = 0;
	void __user *priv_arg = (void __user *)arg;
	struct htc_smi_drvdata *drvdata;

	drvdata = file->private_data;

	if (IS_ERR(priv_arg)) {
		dev_err(drvdata->dev, "%s: invalid user space pointer %lu\n",
			__func__, arg);
		return -EINVAL;
	}

	mutex_lock(&drvdata->mutex);
	if (((drvdata->sensor_conn_type == SPI) && (!drvdata->clock_state)) ||
	    ((drvdata->sensor_conn_type == SSC_SPI) && (!drvdata->ssc_state))) {
		rc = -EPERM;
		dev_err(drvdata->dev, "%s: IOCTL call in invalid state\n",
			__func__);
		goto end;
	}

	switch (cmd) {
	case HTC_SMI_LOAD_APP:
	{
		struct htc_smi_app app;

		if (copy_from_user(&app, priv_arg,
			sizeof(app)) != 0) {
			rc = -ENOMEM;
			dev_err(drvdata->dev,
				"%s: Failed copy from user space-LOAD\n",
				__func__);
			goto end;
		}

		
		rc = qseecom_start_app(app.app_handle, app.name, app.size);
		if (rc == 0) {
			g_app_buf_size = app.size;
		} else {
			dev_err(drvdata->dev, "%s: App %s failed to load\n",
				__func__, app.name);
			goto end;
		}

		break;
	}
	case HTC_SMI_UNLOAD_APP:
	{
		struct htc_smi_app app;

		if (copy_from_user(&app, priv_arg,
			sizeof(app)) != 0) {
			rc = -ENOMEM;
			dev_err(drvdata->dev,
				"%s: Failed copy from user space-LOAD\n",
				 __func__);
			goto end;
		}

		
		if (!app.app_handle) {
			dev_err(drvdata->dev, "%s: App not loaded\n",
				__func__);
			rc = -EINVAL;
			goto end;
		}

		rc = qseecom_shutdown_app(app.app_handle);
		if (rc != 0) {
			dev_err(drvdata->dev, "%s: App failed to shutdown\n",
				__func__);
			goto end;
		}

		break;
	}
	case HTC_SMI_SEND_TZCMD:
	{
		void *aligned_cmd;
		void *aligned_rsp;
		uint32_t aligned_cmd_len;
		uint32_t aligned_rsp_len;

		struct htc_smi_send_tz_cmd tzcmd;

		if (copy_from_user(&tzcmd, priv_arg,
			sizeof(tzcmd))
				!= 0) {
			rc = -ENOMEM;
			dev_err(drvdata->dev,
				"%s: Failed copy from user space-LOAD\n",
				 __func__);
			goto end;
		}

		
		if (!tzcmd.app_handle) {
			dev_err(drvdata->dev, "%s: App not loaded\n",
				__func__);
			rc = -EINVAL;
			goto end;
		}

		
		aligned_cmd_len = tzcmd.req_buf_len;
		aligned_rsp_len = tzcmd.rsp_buf_len;
		rc = get_cmd_rsp_buffers(tzcmd.app_handle,
			(void **)&aligned_cmd,
			&aligned_cmd_len,
			(void **)&aligned_rsp,
			&aligned_rsp_len);
		if (rc != 0)
			goto end;

		rc = copy_from_user(aligned_cmd, (void __user *)tzcmd.req_buf,
				tzcmd.req_buf_len);
		if (rc != 0) {
			dev_err(drvdata->dev,
				"%s: Failure to copy user space buf %d\n",
				 __func__, rc);
			goto end;
		}

		
		rc = qseecom_send_command(tzcmd.app_handle,
			aligned_cmd,
			aligned_cmd_len,
			aligned_rsp,
			aligned_rsp_len);

		if (rc == 0) {
			
			rc = copy_to_user((void __user *)tzcmd.rsp_buf,
				 aligned_rsp, tzcmd.rsp_buf_len);
			if (rc != 0) {
				dev_err(drvdata->dev,
					"%s: Failed copy 2us rc:%d bytes %d:\n",
					 __func__, rc, tzcmd.rsp_buf_len);
				goto end;
			}
		} else {
			dev_err(drvdata->dev, "%s: Failure to send tz cmd %d\n",
				__func__, rc);
			goto end;
		}

		break;
	}
	default:
		dev_err(drvdata->dev, "%s: Invalid cmd %d\n", __func__, cmd);
		rc = -EINVAL;
		goto end;
	}

end:
	mutex_unlock(&drvdata->mutex);
	return rc;
}

static const struct file_operations htc_smi_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = htc_smi_ioctl,
	.open = htc_smi_open,
	.release = htc_smi_release
};

static int htc_smi_dev_register(struct htc_smi_drvdata *drvdata)
{
	dev_t dev_no;
	int ret = 0;
	size_t node_size;
	char *node_name = HTC_SMI_DEV;
	struct device *dev = drvdata->dev;
	struct device *device;

	node_size = strlen(node_name) + 1;

	drvdata->htc_smi_node = devm_kzalloc(dev, node_size, GFP_KERNEL);
	if (!drvdata->htc_smi_node) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	strlcpy(drvdata->htc_smi_node, node_name, node_size);

	ret = alloc_chrdev_region(&dev_no, 0, 1, drvdata->htc_smi_node);
	if (ret) {
		dev_err(drvdata->dev, "%s: alloc_chrdev_region failed %d\n",
			__func__, ret);
		goto err_alloc;
	}

	cdev_init(&drvdata->htc_smi_cdev, &htc_smi_fops);

	drvdata->htc_smi_cdev.owner = THIS_MODULE;
	ret = cdev_add(&drvdata->htc_smi_cdev, dev_no, 1);
	if (ret) {
		dev_err(drvdata->dev, "%s: cdev_add failed %d\n", __func__,
			ret);
		goto err_cdev_add;
	}

	drvdata->htc_smi_class = class_create(THIS_MODULE,
					   drvdata->htc_smi_node);
	if (IS_ERR(drvdata->htc_smi_class)) {
		ret = PTR_ERR(drvdata->htc_smi_class);
		dev_err(drvdata->dev, "%s: class_create failed %d\n", __func__,
			ret);
		goto err_class_create;
	}

	device = device_create(drvdata->htc_smi_class, NULL,
			       drvdata->htc_smi_cdev.dev, drvdata,
			       drvdata->htc_smi_node);
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		dev_err(drvdata->dev, "%s: device_create failed %d\n",
			__func__, ret);
		goto err_dev_create;
	}

	return 0;

err_dev_create:
	class_destroy(drvdata->htc_smi_class);
err_class_create:
	cdev_del(&drvdata->htc_smi_cdev);
err_cdev_add:
	unregister_chrdev_region(drvdata->htc_smi_cdev.dev, 1);
err_alloc:
	return ret;
}

static int htc_smi_read_spi_conn_properties(struct device_node *node,
					    struct htc_smi_drvdata *drvdata)
{
	int rc = 0;
	int index = 0;
	uint32_t rate;
	uint8_t clkcnt = 0;
	const char *clock_name;

	if ((node == NULL) || (drvdata == NULL))
		return -EINVAL;

	drvdata->sensor_conn_type = SPI;

	
	clkcnt = of_property_count_strings(node, "clock-names");
	if (IS_ERR_VALUE(drvdata->clock_count)) {
			dev_err(drvdata->dev, "%s: Failed to get clock names\n",
				__func__);
			return -EINVAL;
	}

	
	if (clkcnt > 16) {
			dev_err(drvdata->dev, "%s: Invalid clock count %d\n",
				__func__, clkcnt);
			return -EINVAL;
	}

	
	drvdata->clock_count = clkcnt;
	drvdata->clocks = devm_kzalloc(drvdata->dev,
				sizeof(struct clk *) * drvdata->clock_count,
				GFP_KERNEL);
	if (!drvdata->clocks) {
			dev_err(drvdata->dev,
				"%s: Failed to alloc memory for clocks\n",
				__func__);
			return -ENOMEM;
	}

	
	for (index = 0; index < drvdata->clock_count; index++) {
			of_property_read_string_index(node,
					"clock-names",
					index, &clock_name);
			drvdata->clocks[index] = devm_clk_get(drvdata->dev,
							      clock_name);
			if (IS_ERR(drvdata->clocks[index])) {
				rc = PTR_ERR(drvdata->clocks[index]);
				if (rc != -EPROBE_DEFER)
					dev_err(drvdata->dev,
						"%s: Failed get %s\n",
						__func__, clock_name);
					return rc;
			}

			if (!strcmp(clock_name, "spi_clk"))
				drvdata->root_clk_idx = index;
	}

	
	if (of_property_read_u32(node, "clock-frequency", &rate) == 0)
		drvdata->frequency = rate;

	return 0;
}

static int htc_smi_read_ssc_spi_conn_properties(struct device_node *node,
						struct htc_smi_drvdata *drvdata)
{
	int rc = 0;
	uint32_t rate;

	if ((node == NULL) || (drvdata == NULL))
		return -EINVAL;

	drvdata->sensor_conn_type = SSC_SPI;

	
	if (of_property_read_u32(node, "qcom,spi-port-id",
				 &drvdata->ssc_spi_port) != 0)
			return -EINVAL;

	
	if (of_property_read_u32(node, "qcom,spi-port-slave-index",
				 &drvdata->ssc_spi_port_slave_index) != 0)
		return -EINVAL;

	
	if (of_property_read_u32(node, "qcom,tz-subsys-id",
				 &drvdata->tz_subsys_id) != 0)
		return -EINVAL;

	
	if (of_property_read_u32(node, "qcom,ssc-subsys-id",
				 &drvdata->ssc_subsys_id) != 0)
		return -EINVAL;

	
	if (of_property_read_u32(node, "clock-frequency", &rate) != 0)
		return -EINVAL;

	drvdata->frequency = rate;

	INIT_WORK(&drvdata->qmi_svc_arrive, htc_smi_qmi_clnt_svc_arrive);
	INIT_WORK(&drvdata->qmi_svc_exit, htc_smi_qmi_clnt_svc_exit);
	INIT_WORK(&drvdata->qmi_svc_rcv_msg, htc_smi_qmi_clnt_recv_msg);

	drvdata->qmi_svc_notifier.notifier_call = htc_smi_qmi_svc_event_notify;
	drvdata->qmi_handle = NULL;
	rc = qmi_svc_event_notifier_register(HTC_SMI_SNS_SERVICE_ID,
					     HTC_SMI_SNS_SERVICE_VER_ID,
					     HTC_SMI_SNS_INSTANCE_INST_ID,
					     &drvdata->qmi_svc_notifier);
	if (rc < 0)
		dev_err(drvdata->dev, "%s: QMI service notifier reg failed\n",
			__func__);

	return rc;
}

static int htc_smi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct htc_smi_drvdata *drvdata;
	int rc = 0;
	int child_node_cnt = 0;
	struct device_node *child_node;
    printk("htc-smi probe++");
	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	
	child_node_cnt = of_get_child_count(pdev->dev.of_node);
	if (child_node_cnt != 1) {
		dev_err(drvdata->dev, "%s: Invalid number of child nodes %d\n",
			__func__, child_node_cnt);
		rc = -EINVAL;
		goto end;
	}

	for_each_child_of_node(pdev->dev.of_node, child_node) {
		if (!of_node_cmp(child_node->name,
				 "qcom,fingerprint-sensor-spi-conn")) {
			
			rc = htc_smi_read_spi_conn_properties(child_node,
							      drvdata);
			if (rc != 0) {
				dev_err(drvdata->dev,
					"%s: Failed to read SPI conn prop\n",
					__func__);
				goto end;
			}
		} else if (!of_node_cmp(child_node->name,
				"qcom,fingerprint-sensor-ssc-spi-conn")) {
			
			rc = htc_smi_read_ssc_spi_conn_properties(child_node,
								  drvdata);
			if (rc != 0) {
				dev_err(drvdata->dev,
					"%s: Failed to read SPI conn prop\n",
					__func__);
				goto end;
			}
		} else {
			dev_err(drvdata->dev, "%s: Invalid child node %s\n",
				__func__, child_node->name);
			rc = -EINVAL;
			goto end;
		}
	}

	atomic_set(&drvdata->available, 1);

	mutex_init(&drvdata->mutex);

	rc = htc_smi_dev_register(drvdata);
	if (rc < 0)
		goto end;

end:
	return rc;
}

static int htc_smi_remove(struct platform_device *pdev)
{
	struct htc_smi_drvdata *drvdata = platform_get_drvdata(pdev);

	if (drvdata->sensor_conn_type == SPI) {
	} else if (drvdata->sensor_conn_type == SSC_SPI) {
		htc_smi_set_blsp_ownership(drvdata, drvdata->ssc_subsys_id);
		htc_smi_sns_keep_alive_req(drvdata, 0);
		htc_smi_sns_close_req(drvdata);
		qmi_handle_destroy(drvdata->qmi_handle);
		qmi_svc_event_notifier_unregister(HTC_SMI_SNS_SERVICE_ID,
						  HTC_SMI_SNS_SERVICE_VER_ID,
						  HTC_SMI_SNS_INSTANCE_INST_ID,
						  &drvdata->qmi_svc_notifier);
	}
	mutex_destroy(&drvdata->mutex);

	device_destroy(drvdata->htc_smi_class, drvdata->htc_smi_cdev.dev);
	class_destroy(drvdata->htc_smi_class);
	cdev_del(&drvdata->htc_smi_cdev);
	unregister_chrdev_region(drvdata->htc_smi_cdev.dev, 1);
	return 0;
}

static int htc_smi_suspend(struct platform_device *pdev, pm_message_t state)
{
	int rc = 0;
	struct htc_smi_drvdata *drvdata = platform_get_drvdata(pdev);

	if(!mutex_trylock(&drvdata->mutex))
		return -EBUSY;
	if (((drvdata->sensor_conn_type == SPI) && (drvdata->clock_state)) ||
	    ((drvdata->sensor_conn_type == SSC_SPI) && (drvdata->ssc_state)))
		rc = -EBUSY;
	mutex_unlock(&drvdata->mutex);

	return rc;
}

static struct of_device_id htc_smi_match[] = {
	{ .compatible = "qcom,htc_smi" },
	{}
};

static struct platform_driver htc_smi_plat_driver = {
	.probe = htc_smi_probe,
	.remove = htc_smi_remove,
	.suspend = htc_smi_suspend,
	.driver = {
		.name = "htc_smi",
		.owner = THIS_MODULE,
		.of_match_table = htc_smi_match,
	},
};

static int htc_smi_init(void)
{
	return platform_driver_register(&htc_smi_plat_driver);
}
module_init(htc_smi_init);

static void htc_smi_exit(void)
{
	platform_driver_unregister(&htc_smi_plat_driver);
}
module_exit(htc_smi_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm Technologies, Inc. htc_smi driver");
