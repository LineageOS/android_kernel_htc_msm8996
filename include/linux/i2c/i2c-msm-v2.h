/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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

#ifndef _I2C_MSM_V2_H
#define _I2C_MSM_V2_H

#include <linux/bitops.h>
#include <linux/dmaengine.h>

enum msm_i2_debug_level {
	MSM_ERR,	
	MSM_PROF,	
	MSM_DBG,	
};

#define i2c_msm_dbg(ctrl, dbg_level, fmt, ...) do {\
		if (ctrl->dbgfs.dbg_lvl >= dbg_level)\
			dev_info(ctrl->dev, pr_fmt(fmt), ##__VA_ARGS__);\
	} while (0)

#define BIT_IS_SET(val, idx)        ((val >> idx) & 0x1)
#define BITS_AT(val, idx, n_bits)(((val) & (((1 << n_bits) - 1) << idx)) >> idx)
#define MASK_IS_SET(val, mask)      ((val & mask) == mask)
#define MASK_IS_SET_BOOL(val, mask) (MASK_IS_SET(val, mask) ? 1 : 0)
#define KHz(freq) (1000 * freq)
#define I2C_MSM_CLK_FAST_PLUS_FREQ  (1000000)

enum {
	QUP_CONFIG              = 0x0,
	QUP_STATE               = 0x4,
	QUP_IO_MODES            = 0x8,
	QUP_SW_RESET            = 0xC,
	QUP_OPERATIONAL         = 0x18,
	QUP_ERROR_FLAGS         = 0x1C,
	QUP_ERROR_FLAGS_EN      = 0x20,
	QUP_TEST_CTRL           = 0x24,
	QUP_OPERATIONAL_MASK    = 0x28,
	QUP_HW_VERSION          = 0x30,
	QUP_MX_READ_COUNT       = 0x208,
	QUP_MX_WRITE_COUNT      = 0x150,
	QUP_MX_OUTPUT_COUNT     = 0x100,
	QUP_MX_INPUT_COUNT      = 0x200,
	QUP_MX_WR_CNT           = 0x100,
	QUP_OUT_DEBUG           = 0x108,
	QUP_OUT_FIFO_CNT        = 0x10C,
	QUP_OUT_FIFO_BASE       = 0x110,
	QUP_IN_READ_CUR         = 0x20C,
	QUP_IN_DEBUG            = 0x210,
	QUP_IN_FIFO_CNT         = 0x214,
	QUP_IN_FIFO_BASE        = 0x218,
	QUP_I2C_MASTER_CLK_CTL  = 0x400,
	QUP_I2C_STATUS          = 0x404,
	QUP_I2C_MASTER_CONFIG   = 0x408,
	QUP_I2C_MASTER_BUS_CLR  = 0x40C,
};

enum i2c_msm_qup_state {
	QUP_STATE_RESET         = 0,
	QUP_STATE_RUN           = 1U,
	QUP_STATE_PAUSE         = 3U,
};

enum {
	QUP_STATE_MASK          = 3U,
	QUP_STATE_VALID         = BIT(2),
	QUP_I2C_MAST_GEN        = BIT(4),
	QUP_I2C_FLUSH           = BIT(6),
	QUP_I2C_STATUS_RESET    = 0x42,
};


enum {
	QUP_MINI_CORE_MASK      = 0xF00,
	QUP_MINI_CORE_I2C_VAL   = 0x200,
	QUP_N_MASK              = 0x1F,
	QUP_N_VAL               = 0x7, 
	QUP_NO_OUPUT            = BIT(6),
	QUP_NO_INPUT            = BIT(7),
	QUP_APP_CLK_ON_EN       = BIT(12),
	QUP_CORE_CLK_ON_EN      = BIT(13),
	QUP_FIFO_CLK_GATE_EN    = BIT(14),
};

enum {
	QUP_INPUT_FIFO_NOT_EMPTY = BIT(5),
	QUP_OUTPUT_SERVICE_FLAG  = BIT(8),
	QUP_INPUT_SERVICE_FLAG   = BIT(9),
	QUP_MAX_OUTPUT_DONE_FLAG = BIT(10),
	QUP_MAX_INPUT_DONE_FLAG  = BIT(11),
	QUP_OUT_BLOCK_WRITE_REQ  = BIT(12),
	QUP_IN_BLOCK_READ_REQ    = BIT(13),
};

enum {
	QUP_INPUT_SERVICE_MASK  = BIT(9),
	QUP_OUTPUT_SERVICE_MASK = BIT(8),
};

enum {
	QUP_OUTPUT_MODE         = BIT(10) | BIT(11),
	QUP_INPUT_MODE          = BIT(12) | BIT(13),
	QUP_UNPACK_EN           = BIT(14),
	QUP_PACK_EN             = BIT(15),
	QUP_OUTPUT_BIT_SHIFT_EN = BIT(16),
};

enum {
	QUP_BUS_ERROR           = BIT(2),
	QUP_PACKET_NACKED       = BIT(3),
	QUP_ARB_LOST            = BIT(4),
	QUP_INVALID_WRITE       = BIT(5),
	QUP_FAILED              = BIT(6),
	QUP_BUS_ACTIVE          = BIT(8),
	QUP_BUS_MASTER          = BIT(9),
	QUP_INVALID_TAG         = BIT(23),
	QUP_INVALID_READ_ADDR   = BIT(24),
	QUP_INVALID_READ_SEQ    = BIT(25),
	QUP_I2C_SDA             = BIT(26),
	QUP_I2C_SCL             = BIT(27),
	QUP_MSTR_STTS_ERR_MASK  = 0x380003C,
};

enum {
	QUP_EN_VERSION_TWO_TAG  = 1U,
};

#define I2C_MSM_SCL_NOISE_REJECTION(reg_val, noise_rej_val) \
		(((reg_val) & ~(0x3 << 24)) | (((noise_rej_val) & 0x3) << 24))
#define I2C_MSM_SDA_NOISE_REJECTION(reg_val, noise_rej_val) \
		(((reg_val) & ~(0x3 << 26)) | (((noise_rej_val) & 0x3) << 26))

enum {
	QUP_OUTPUT_OVER_RUN_ERR_EN  = BIT(5),
	QUP_INPUT_UNDER_RUN_ERR_EN  = BIT(4),
	QUP_OUTPUT_UNDER_RUN_ERR_EN = BIT(3),
	QUP_INPUT_OVER_RUN_ERR_EN   = BIT(2),
};

enum {
	I2C_STATUS_WR_BUFFER_FULL  = BIT(0),
	I2C_STATUS_BUS_ACTIVE      = BIT(8),
	I2C_STATUS_BUS_MASTER      = BIT(9),
	I2C_STATUS_ERROR_MASK      = 0x38000FC,
	QUP_I2C_NACK_FLAG          = BIT(3),
	QUP_IN_NOT_EMPTY           = BIT(5),
	QUP_ERR_FLGS_MASK           = 0x3C,
};

enum {
	I2C_CLK_RESET_BUSIDLE_STATE = 0,
	I2C_CLK_FORCED_LOW_STATE    = 5,
};

enum i2c_msm_power_state {
	I2C_MSM_PM_RT_ACTIVE,
	I2C_MSM_PM_RT_SUSPENDED,
	I2C_MSM_PM_SYS_SUSPENDED
};

#define I2C_MSM_TAG2_MAX_LEN            (4)
#define I2C_MSM_DMA_TX_SZ             (64) 
#define I2C_MSM_DMA_RX_SZ             (32) 
#define I2C_MSM_DMA_DESC_ARR_SIZ  (I2C_MSM_DMA_TX_SZ + I2C_MSM_DMA_RX_SZ)
#define I2C_MSM_REG_2_STR_BUF_SZ        (128)
#define I2C_MSM_MAX_ERR_BUF_SZ		(256)
#define I2C_MSM_BUF_DUMP_MAX_BC         (20)
#define I2C_MSM_MAX_POLL_MSEC           (100)
#define I2C_MSM_TIMEOUT_SAFTY_COEF      (10)
#define I2C_MSM_TIMEOUT_MIN_USEC        (500000)
#define I2C_QUP_MAX_BUS_RECOVERY_RETRY  (10)

#define QUP_TAG2_DATA_WRITE        (0x82ULL)
#define QUP_TAG2_DATA_WRITE_N_STOP (0x83ULL)
#define QUP_TAG2_DATA_READ         (0x85ULL)
#define QUP_TAG2_DATA_READ_N_STOP  (0x87ULL)
#define QUP_TAG2_START             (0x81ULL)
#define QUP_TAG2_DATA_READ_N_NACK  (0x86ULL)
#define QUP_TAG2_START_STOP        (0x8AULL)
#define QUP_TAG2_INPUT_EOT         (0x93ULL)
#define QUP_TAG2_FLUSH_STOP        (0x96ULL)
#define QUP_BUF_OVERHD_BC          (2)
#define QUP_MAX_BUF_SZ             (256)

enum i2c_msm_clk_path_vec_idx {
	I2C_MSM_CLK_PATH_SUSPEND_VEC,
	I2C_MSM_CLK_PATH_RESUME_VEC,
};
#define I2C_MSM_CLK_PATH_AVRG_BW(ctrl) (0)
#define I2C_MSM_CLK_PATH_BRST_BW(ctrl) (ctrl->rsrcs.clk_freq_in * 8)

enum i2c_msm_gpio_name_idx {
	I2C_MSM_GPIO_SCL,
	I2C_MSM_GPIO_SDA,
};

extern const char * const i2c_msm_mode_str_tbl[];

struct i2c_msm_ctrl;

struct i2c_msm_dma_mem {
	dma_addr_t               phy_addr;
	void                    *vrtl_addr;
};

struct i2c_msm_tag {
	u64                    val;
	int                    len;
};

struct i2c_msm_dma_tag {
	dma_addr_t             buf;
	size_t                 len;
};

struct i2c_msm_dma_buf {
	struct i2c_msm_dma_mem   ptr;
	enum dma_data_direction  dma_dir;
	size_t                   len;
	bool                     is_rx;
	bool                     is_last;
	struct i2c_msm_dma_tag   tag;
	
	struct scatterlist	sg_list[2];
};

struct i2c_msm_dma_chan {
	bool                     is_init;
	const char              *name;
	size_t                   desc_cnt_cur;
	struct dma_chan         *dma_chan;
	enum dma_transfer_direction dir;
};

enum i2c_msm_dma_chan_dir {
	I2C_MSM_DMA_TX,
	I2C_MSM_DMA_RX,
	I2C_MSM_DMA_CNT,
};

enum i2c_msm_dma_state {
	I2C_MSM_DMA_INIT_NONE, 
	I2C_MSM_DMA_INIT_CORE, 
	I2C_MSM_DMA_INIT_CHAN, 
};

struct i2c_msm_xfer_mode_dma {
	enum i2c_msm_dma_state   state;
	size_t                   buf_arr_cnt;
	struct i2c_msm_dma_buf   buf_arr[I2C_MSM_DMA_DESC_ARR_SIZ];
	struct i2c_msm_dma_mem   tag_arr;
	struct i2c_msm_dma_mem   eot_n_flush_stop_tags;
	struct i2c_msm_dma_mem   input_tag;
	struct i2c_msm_dma_chan  chan[I2C_MSM_DMA_CNT];
};

#define I2C_MSM_DMA_TAG_MEM_SZ  \
	((I2C_MSM_DMA_DESC_ARR_SIZ + 2) * I2C_MSM_TAG2_MAX_LEN)

struct i2c_msm_xfer_mode_fifo {
	size_t                   input_fifo_sz;
	size_t                   output_fifo_sz;
	size_t                   in_rem;
	size_t                   out_rem;
	u8                       out_buf[4];
	int                      out_buf_idx;
};

/* i2c_msm_xfer_mode_blk: operations and state of Block mode
 *
 * @is_init when true, struct is initialized and requires mem free on exit
 * @in_blk_sz size of input/rx block
 * @out_blk_sz size of output/tx block
 * @tx_cache internal buffer to store tx data
 * @rx_cache internal buffer to store rx data
 * @rx_cache_idx points to the next unread index in rx cache
 * @tx_cache_idx points to the next unwritten index in tx cache
 * @wait_rx_blk completion object to wait on for end of blk rx transfer.
 * @wait_tx_blk completion object to wait on for end of blk tx transfer.
 * @complete_mask applied to QUP_OPERATIONAL to determine when blk
 *  xfer is complete.
 */
struct i2c_msm_xfer_mode_blk {
	bool                     is_init;
	size_t                   in_blk_sz;
	size_t                   out_blk_sz;
	u8                       *tx_cache;
	u8                       *rx_cache;
	int                      rx_cache_idx;
	int                      tx_cache_idx;
	struct completion        wait_rx_blk;
	struct completion        wait_tx_blk;
	u32                      complete_mask;
};

enum i2c_msm_xfer_mode_id {
	I2C_MSM_XFER_MODE_FIFO,
	I2C_MSM_XFER_MODE_BLOCK,
	I2C_MSM_XFER_MODE_DMA,
	I2C_MSM_XFER_MODE_NONE, 
};


struct i2c_msm_dbgfs {
	struct dentry             *root;
	enum msm_i2_debug_level    dbg_lvl;
	enum i2c_msm_xfer_mode_id  force_xfer_mode;
};

struct qup_i2c_clk_path_vote {
	u32                         mstr_id;
	u32                         client_hdl;
	struct msm_bus_scale_pdata *pdata;
	bool                        reg_err;
	bool                        actv_only;
};

struct i2c_msm_resources {
	struct resource             *mem;
	void __iomem                *base; 
	struct clk                  *core_clk;
	struct clk                  *iface_clk;
	int                          clk_freq_in;
	int                          clk_freq_out;
	struct qup_i2c_clk_path_vote clk_path_vote;
	int                          irq;
	bool                         disable_dma;
	struct pinctrl              *pinctrl;
	struct pinctrl_state        *gpio_state_active;
	struct pinctrl_state        *gpio_state_suspend;
	u32                         nack_as_normal;
};

#define I2C_MSM_PINCTRL_ACTIVE       "i2c_active"
#define I2C_MSM_PINCTRL_SUSPEND      "i2c_sleep"

struct i2c_msm_xfer_buf {
	bool                       is_init;
	int                        msg_idx;
	int                        byte_idx;
	int                        end_idx;
	int                        len;
	bool                       is_rx;
	bool                       is_last;
	u16                        slv_addr;
	struct i2c_msm_tag         in_tag;
	struct i2c_msm_tag         out_tag;
};

#ifdef DEBUG
#define I2C_MSM_PROF_MAX_EVNTS   (64)
#else
#define I2C_MSM_PROF_MAX_EVNTS   (16)
#endif

struct i2c_msm_prof_event {
	struct timespec time;
	u64             data0;
	u32             data1;
	u32             data2;
	u8              type;
	u8              dump_func_id;
};

enum i2c_msm_err {
	I2C_MSM_NO_ERR = 0,
	I2C_MSM_ERR_NACK,
	I2C_MSM_ERR_ARB_LOST,
	I2C_MSM_ERR_BUS_ERR,
	I2C_MSM_ERR_TIMEOUT,
	I2C_MSM_ERR_CORE_CLK,
	I2C_MSM_ERR_OVR_UNDR_RUN,
};

struct i2c_msm_xfer {
	struct i2c_msg            *msgs;
	int                        msg_cnt;
	enum i2c_msm_xfer_mode_id  mode_id;
	struct completion          complete;
	size_t                     rx_cnt;
	size_t                     tx_cnt;
	size_t                     rx_ovrhd_cnt;
	size_t                     tx_ovrhd_cnt;
	struct i2c_msm_xfer_buf    cur_buf;
	u32                        timeout;
	bool                       last_is_rx;
	enum i2c_msm_err           err;
	struct i2c_msm_prof_event  event[I2C_MSM_PROF_MAX_EVNTS];
	atomic_t                   event_cnt;
	atomic_t                   is_active;
	struct mutex               mtx;
	struct i2c_msm_xfer_mode_fifo	fifo;
	struct i2c_msm_xfer_mode_blk	blk;
	struct i2c_msm_xfer_mode_dma	dma;
};

struct i2c_msm_ctrl {
	struct device             *dev;
	struct i2c_adapter         adapter;
	struct i2c_msm_xfer        xfer;
	struct i2c_msm_dbgfs       dbgfs;
	struct i2c_msm_resources   rsrcs;
	u32                        mstr_clk_ctl;
	enum i2c_msm_power_state   pwr_state;
};

enum i2c_msm_prof_evnt_type {
	I2C_MSM_VALID_END,
	I2C_MSM_PIP_DSCN,
	I2C_MSM_PIP_CNCT,
	I2C_MSM_ACTV_END,
	I2C_MSM_IRQ_BGN,
	I2C_MSM_IRQ_END,
	I2C_MSM_XFER_BEG,
	I2C_MSM_XFER_END,
	I2C_MSM_SCAN_SUM,
	I2C_MSM_NEXT_BUF,
	I2C_MSM_COMPLT_OK,
	I2C_MSM_COMPLT_FL,
	I2C_MSM_PROF_RESET,
};

#ifdef CONFIG_I2C_MSM_PROF_DBG
void i2c_msm_dbgfs_init(struct i2c_msm_ctrl *ctrl);

void i2c_msm_dbgfs_teardown(struct i2c_msm_ctrl *ctrl);

const char *i2c_msm_dbg_tag_to_str(const struct i2c_msm_tag *tag,
						char *buf, size_t buf_len);

void i2c_msm_prof_evnt_dump(struct i2c_msm_ctrl *ctrl);

void i2c_msm_prof_evnt_add(struct i2c_msm_ctrl *ctrl,
				enum msm_i2_debug_level dbg_level,
				enum i2c_msm_prof_evnt_type event,
				u64 data0, u32 data1, u32 data2);

int i2c_msm_dbg_qup_reg_dump(struct i2c_msm_ctrl *ctrl);

const char *
i2c_msm_dbg_dma_tag_to_str(const struct i2c_msm_dma_tag *dma_tag, char *buf,
								size_t buf_len);
#else
static inline void i2c_msm_dbgfs_init(struct i2c_msm_ctrl *ctrl) {}
static inline void i2c_msm_dbgfs_teardown(struct i2c_msm_ctrl *ctrl) {}

static inline const char *i2c_msm_dbg_tag_to_str(const struct i2c_msm_tag *tag,
						char *buf, size_t buf_len)
{
	return NULL;
}
static inline void i2c_msm_prof_evnt_dump(struct i2c_msm_ctrl *ctrl) {}

static inline void i2c_msm_prof_evnt_add(struct i2c_msm_ctrl *ctrl,
				enum msm_i2_debug_level dbg_level,
				enum i2c_msm_prof_evnt_type event,
				u64 data0, u32 data1, u32 data2) {}

static inline int i2c_msm_dbg_qup_reg_dump(struct i2c_msm_ctrl *ctrl)
{
	return true;
}
static inline const char *i2c_msm_dbg_dma_tag_to_str(const struct
			i2c_msm_dma_tag * dma_tag, char *buf, size_t buf_len)
{
	return NULL;
}
#endif 
#endif 
