/* include/linux/misc/rt-regmap.h
 * Header of Richtek regmap with debugfs Driver
 *
 * Copyright (C) 2014 Richtek Technology Corp.
 * Jeff Chang <jeff_chang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LINUX_MISC_RT_REGMAP_H
#define LINUX_MISC_RT_REGMAP_H

#include <linux/debugfs.h>

#define RT_REGMAP_VERSION	"1.1.6_G"

enum rt_access_mode {
	RT_1BYTE_MODE = 1,
	RT_2BYTE_MODE = 2,
	RT_4BYTE_MODE = 4,
};

struct rt_access_group {
	u32 start;
	u32 end;
	enum rt_access_mode mode;
};


#define RT_REG_TYPE_MASK	(0x03)
#define RT_NORMAL		(0x00)
#define RT_WBITS		(0x01)
#define RT_VOLATILE		(0x02)
#define RT_RESERVE		(0x03)

#define RT_WR_ONCE		(0x08)
#define RT_NORMAL_WR_ONCE	(RT_NORMAL|RT_WR_ONCE)
#define RT_WBITS_WR_ONCE	(RT_WBITS|RT_WR_ONCE)

enum rt_data_format {
	RT_LITTLE_ENDIAN,
	RT_BIG_ENDIAN,
};


#define RT_BYTE_MODE_MASK	(0x01)
#define RT_SINGLE_BYTE		(0 << 0)
#define RT_MULTI_BYTE		(1 << 0)

#define RT_CACHE_MODE_MASK	 (0x06)
#define RT_CACHE_WR_THROUGH	 (0 << 1)
#define RT_CACHE_WR_BACK	 (1 << 1)
#define RT_CACHE_DISABLE	 (2 << 1)

#define RT_IO_BLK_MODE_MASK (0x18)
#define RT_IO_PASS_THROUGH	(0 << 3)
#define RT_IO_BLK_ALL		(1 << 3)
#define RT_IO_BLK_CACHE		(2 << 3)
#define RT_IO_BLK_CHIP		(3 << 3)

#define DBG_MODE_MASK	(0x20)
#define RT_DBG_GENERAL	(0 << 5)
#define RT_DBG_SPECIAL	(1 << 5)


struct rt_register {
	u32 addr;
	const char *name;
	unsigned int size;
	unsigned char reg_type;
	unsigned char *wbit_mask;
	unsigned char *cache_data;
};

#define RT_REG_DECL(_addr, _reg_length, _reg_type, _mask_...)	\
	static unsigned char rt_writable_mask_##_addr[_reg_length] = _mask_;\
	static struct rt_register rt_register_##_addr = {	\
		.addr = _addr, \
		.size = _reg_length,\
		.reg_type = _reg_type,\
		.wbit_mask = rt_writable_mask_##_addr,\
	}

#define RT_NAMED_REG_DECL(_addr, _name, _reg_length, _reg_type, _mask_...) \
	static unsigned char rt_writable_mask_##_addr[_reg_length] = _mask_;\
	static struct rt_register rt_register_##_addr = {	\
		.addr = _addr, \
		.name = _name, \
		.size = _reg_length,\
		.reg_type = _reg_type,\
		.wbit_mask = rt_writable_mask_##_addr,\
	}

typedef struct rt_register *rt_register_map_t;

#define RT_REG(_addr) (&rt_register_##_addr)

struct rt_regmap_properties {
	const char *name;
	const char *aliases;
	int register_num;
	const rt_register_map_t *rm;
	struct rt_access_group *group;
	enum rt_data_format rt_format;
	unsigned char rt_regmap_mode;
	unsigned char io_log_en:1;
};

struct rt_reg_data {
	u32 reg;
	u32 mask;
	union {
		u32 data_u32;
		u16 data_u16;
		u8 data_u8;
		u8 data[4];
	} rt_data;
};

struct rt_regmap_device;

struct rt_debug_st {
	void *info;
	int id;
};

struct rt_regmap_fops {
	int (*read_device)(void *client, u32 addr, int leng, void *dst);
	int (*write_device)(void *client, u32 addr, int leng, const void *src);
};

extern struct rt_regmap_device*
	rt_regmap_device_register(struct rt_regmap_properties *props,
				struct rt_regmap_fops *rops,
				struct device *parent,
				void *client, void *drvdata);

extern void rt_regmap_device_unregister(struct rt_regmap_device *rd);

extern int rt_regmap_cache_init(struct rt_regmap_device *rd);

extern int rt_regmap_cache_reload(struct rt_regmap_device *rd);

extern int rt_regmap_block_write(struct rt_regmap_device *rd, u32 reg,
					int bytes, const void *rc);
extern int rt_asyn_regmap_block_write(struct rt_regmap_device *rd, u32 reg,
					int bytes, const void *rc);
extern int rt_regmap_block_read(struct rt_regmap_device *rd, u32 reg,
					int bytes, void *dst);

extern int _rt_regmap_reg_read(struct rt_regmap_device *rd,
					struct rt_reg_data *rrd);
extern int _rt_regmap_reg_write(struct rt_regmap_device *rd,
					struct rt_reg_data *rrd);
extern int _rt_asyn_regmap_reg_write(struct rt_regmap_device *rd,
					struct rt_reg_data *rrd);
extern int _rt_regmap_update_bits(struct rt_regmap_device *rd,
					struct rt_reg_data *rrd);

static inline int rt_regmap_reg_read(struct rt_regmap_device *rd,
					struct rt_reg_data *rrd, u32 reg)
{
	rrd->reg = reg;
	return _rt_regmap_reg_read(rd, rrd);
};

static inline int rt_regmap_reg_write(struct rt_regmap_device *rd,
			struct rt_reg_data *rrd, u32 reg, const u32 data)
{
	rrd->reg = reg;
	rrd->rt_data.data_u32 = data;
	return _rt_regmap_reg_write(rd, rrd);
};

static inline int rt_asyn_regmap_reg_write(struct rt_regmap_device *rd,
			struct rt_reg_data *rrd, u32 reg, const u32 data)
{
	rrd->reg = reg;
	rrd->rt_data.data_u32 = data;
	return _rt_asyn_regmap_reg_write(rd, rrd);
};

static inline int rt_regmap_update_bits(struct rt_regmap_device *rd,
			struct rt_reg_data *rrd, u32 reg, u32 mask, u32 data)
{
	rrd->reg = reg;
	rrd->mask = mask;
	rrd->rt_data.data_u32 = data;
	return _rt_regmap_update_bits(rd, rrd);
}

extern void rt_regmap_cache_backup(struct rt_regmap_device *rd);

extern void rt_regmap_cache_sync(struct rt_regmap_device *rd);
extern void rt_regmap_cache_write_back(struct rt_regmap_device *rd, u32 reg);

extern int rt_is_reg_readable(struct rt_regmap_device *rd, u32 reg);
extern int rt_is_reg_volatile(struct rt_regmap_device *rd, u32 reg);
extern int rt_get_regsize(struct rt_regmap_device *rd, u32 reg);
extern void rt_cache_getlasterror(struct rt_regmap_device *rd, char *buf);
extern void rt_cache_clrlasterror(struct rt_regmap_device *rd);

extern int rt_regmap_add_debugfs(struct rt_regmap_device *rd, const char *name,
		umode_t mode, void *data, const struct file_operations *fops);

#define to_rt_regmap_device(obj) container_of(obj, struct rt_regmap_device, dev)

#endif 
