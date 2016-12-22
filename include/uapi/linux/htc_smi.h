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
#ifndef _UAPI_HTC_SMI_H_
#define _UAPI_HTC_SMI_H_

#define MAX_NAME_SIZE					 32

enum htc_smi_commands {
	HTC_SMI_LOAD_APP = 100,
	HTC_SMI_UNLOAD_APP = 101,
	HTC_SMI_SEND_TZCMD = 102
};

struct htc_smi_app {
	struct qseecom_handle **app_handle;
	char name[MAX_NAME_SIZE];
	uint32_t size;
	uint8_t high_band_width;
};

struct htc_smi_send_tz_cmd {
	struct qseecom_handle *app_handle;
	uint8_t *req_buf;
	uint32_t req_buf_len;
	uint8_t *rsp_buf;
	uint32_t rsp_buf_len;
};

#endif 
