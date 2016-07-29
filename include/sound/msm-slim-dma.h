/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
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
#ifndef _MSM_SLIMBUS_DMA_H
#define _MSM_SLIMBUS_DMA_H

#include <linux/slimbus/slimbus.h>

enum msm_dai_slim_event {
	MSM_DAI_SLIM_ENABLE = 1,
	MSM_DAI_SLIM_PRE_DISABLE,
	MSM_DAI_SLIM_DISABLE,
};

struct msm_slim_dma_data {

	
	struct slim_device *sdev;

	
	u32 ph;

	
	int (*dai_channel_ctl) (struct msm_slim_dma_data *dma_data,
				struct snd_soc_dai *dai,
				enum msm_dai_slim_event);
};

#endif
