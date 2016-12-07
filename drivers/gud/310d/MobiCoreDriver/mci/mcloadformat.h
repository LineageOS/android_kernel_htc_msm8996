/*
 * Copyright (c) 2013-2015 TRUSTONIC LIMITED
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef MCLOADFORMAT_H_
#define MCLOADFORMAT_H_

#define MC_TLBLOBLEN_MAGIC	0x7672746C	
#define MAX_SO_CONT_SIZE	512		

#define MC_SERVICE_HEADER_FLAGS_PERMANENT		BIT(0)
#define MC_SERVICE_HEADER_FLAGS_NO_CONTROL_INTERFACE	BIT(1)
#define MC_SERVICE_HEADER_FLAGS_DEBUGGABLE		BIT(2)
#define MC_SERVICE_HEADER_FLAGS_EXTENDED_LAYOUT		BIT(3)

enum service_type {
	SERVICE_TYPE_ILLEGAL		= 0,
	SERVICE_TYPE_DRIVER		= 1,
	SERVICE_TYPE_SP_TRUSTLET	= 2,
	SERVICE_TYPE_SYSTEM_TRUSTLET	= 3,
	SERVICE_TYPE_MIDDLEWARE		= 4,
	SERVICE_TYPE_LAST_ENTRY		= 5,
};

struct segment_descriptor {
	u32	start;	
	u32	len;	
};

struct mclf_intro {
	u32	magic;		
	u32	version;	
};


struct identity {
	
	u32	login_type;
	
	u8	login_data[16];
};

struct mclf_header_v2 {
	
	struct mclf_intro	intro;
	
	u32	flags;
	
	u32	mem_type;
	
	enum service_type	service_type;
	
	u32	num_instances;
	
	struct mc_uuid_t	uuid;
	
	u32	driver_id;
	u32	num_threads;
	
	struct segment_descriptor text;
	
	struct segment_descriptor data;
	
	u32	bss_len;
	
	u32	entry;
	
	u32	service_version;
};


union mclf_header {
	
	struct mclf_intro	intro;
	
	struct mclf_header_v2	mclf_header_v2;
};

struct mc_blob_len_info {
	u32	magic;		
	u32	root_size;	
	u32	sp_size;	
	u32	ta_size;	
	u32	reserved[4];	
};

#endif 
