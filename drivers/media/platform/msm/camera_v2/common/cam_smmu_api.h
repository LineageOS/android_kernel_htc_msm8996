/* Copyright (c) 2014-2016, The Linux Foundation. All rights reserved.
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
#ifndef _CAM_SMMU_API_H_
#define _CAM_SMMU_API_H_

#include <linux/dma-direction.h>
#include <linux/module.h>
#include <linux/dma-buf.h>
#include <asm/dma-iommu.h>
#include <linux/dma-direction.h>
#include <linux/dma-attrs.h>
#include <linux/of_platform.h>
#include <linux/iommu.h>
#include <linux/random.h>
#include <linux/spinlock_types.h>
#include <linux/mutex.h>


enum cam_smmu_ops_param {
	CAM_SMMU_ATTACH,
	CAM_SMMU_DETACH,
	CAM_SMMU_VOTE,
	CAM_SMMU_DEVOTE,
	CAM_SMMU_OPS_INVALID
};

enum cam_smmu_map_dir {
	CAM_SMMU_MAP_READ,
	CAM_SMMU_MAP_WRITE,
	CAM_SMMU_MAP_RW,
	CAM_SMMU_MAP_INVALID
};

int cam_smmu_get_handle(char *identifier, int *handle_ptr);

int cam_smmu_ops(int handle, enum cam_smmu_ops_param op);

int cam_smmu_get_phy_addr(int handle,
				int ion_fd, enum cam_smmu_map_dir dir,
				dma_addr_t *dma_addr, size_t *len_ptr);

int cam_smmu_put_phy_addr(int handle, int ion_fd);


int cam_smmu_get_phy_addr_scratch(int handle,
				  enum cam_smmu_map_dir dir,
				  dma_addr_t *paddr_ptr,
				  size_t virt_len,
				  size_t phys_len);


int cam_smmu_put_phy_addr_scratch(int handle,
				  dma_addr_t paddr);

int cam_smmu_destroy_handle(int handle);

int cam_smmu_get_num_of_clients(void);

int cam_smmu_find_index_by_handle(int hdl);

void cam_smmu_reg_client_page_fault_handler(int handle,
		void (*client_page_fault_handler)(struct iommu_domain *,
		struct device *, unsigned long,
		int, void*), void *token);

#endif 
