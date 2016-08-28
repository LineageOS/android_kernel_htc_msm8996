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

/** Trustlet Blob length info */
#define MC_TLBLOBLEN_MAGIC	0x7672746C	/* Magic for SWd: vrtl */
#define MAX_SO_CONT_SIZE	512		/* Max size for a container */

/** MCLF flags */
/**< Loaded service cannot be unloaded from MobiCore. */
#define MC_SERVICE_HEADER_FLAGS_PERMANENT		BIT(0)
/**< Service has no WSM control interface. */
#define MC_SERVICE_HEADER_FLAGS_NO_CONTROL_INTERFACE	BIT(1)
/**< Service can be debugged. */
#define MC_SERVICE_HEADER_FLAGS_DEBUGGABLE		BIT(2)
/**< New-layout trusted application or trusted driver. */
#define MC_SERVICE_HEADER_FLAGS_EXTENDED_LAYOUT		BIT(3)

/** Service type.
 * The service type defines the type of executable.
 */
enum service_type {
	SERVICE_TYPE_ILLEGAL		= 0,
	SERVICE_TYPE_DRIVER		= 1,
	SERVICE_TYPE_SP_TRUSTLET	= 2,
	SERVICE_TYPE_SYSTEM_TRUSTLET	= 3,
	SERVICE_TYPE_MIDDLEWARE		= 4,
	SERVICE_TYPE_LAST_ENTRY		= 5,
};

/**
 * Descriptor for a memory segment.
 */
struct segment_descriptor {
	u32	start;	
	u32	len;	
};

/**
 * MCLF intro for data structure identification.
 * Must be the first element of a valid MCLF file.
 */
struct mclf_intro {
	u32	magic;		
	u32	version;	
};

/**
 * @defgroup MCLF_VER_V2   MCLF Version 32
 * @ingroup MCLF_VER
 *
 * @addtogroup MCLF_VER_V2
 */

/*
 * GP TA identity.
 */
struct identity {
	
	u32	login_type;
	
	u8	login_data[16];
};

/**
 * Version 2.1/2.2 MCLF header.
 */
struct mclf_header_v2 {
	/**< MCLF header start with the mandatory intro */
	struct mclf_intro	intro;
	
	u32	flags;
	
	u32	mem_type;
	
	enum service_type	service_type;
	
	u32	num_instances;
	
	struct mc_uuid_t	uuid;
	
	u32	driver_id;
	u32	num_threads;
	
	struct segment_descriptor text;
	/**< Virtual data segment */
	struct segment_descriptor data;
	
	u32	bss_len;
	
	u32	entry;
	
	u32	service_version;
};

/**
 * @addtogroup MCLF
 */

/** MCLF header */
union mclf_header {
	/**< Intro for data identification */
	struct mclf_intro	intro;
	/**< Version 2 header */
	struct mclf_header_v2	mclf_header_v2;
};

struct mc_blob_len_info {
	u32	magic;		
	u32	root_size;	
	u32	sp_size;	
	u32	ta_size;	
	u32	reserved[4];	
};

#endif /* MCLOADFORMAT_H_ */
