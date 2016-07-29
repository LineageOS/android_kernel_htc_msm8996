/*
 *  linux/include/linux/mmc/host.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Host driver specific definitions.
 */
#ifndef LINUX_MMC_HOST_H
#define LINUX_MMC_HOST_H

#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/devfreq.h>
#include <linux/fault-inject.h>

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/pm.h>

#define MMC_AUTOSUSPEND_DELAY_MS	3000
#define MMC_STATS_INTERVAL		5000	
#define MMC_STATS_LOG_INTERVAL		60000	
extern struct workqueue_struct *stats_workqueue;

struct mmc_ios {
	unsigned int	clock;			
	unsigned int	old_rate;       
	unsigned long	clk_ts;         
	unsigned short	vdd;


	unsigned char	bus_mode;		

#define MMC_BUSMODE_OPENDRAIN	1
#define MMC_BUSMODE_PUSHPULL	2

	unsigned char	chip_select;		

#define MMC_CS_DONTCARE		0
#define MMC_CS_HIGH		1
#define MMC_CS_LOW		2

	unsigned char	power_mode;		

#define MMC_POWER_OFF		0
#define MMC_POWER_UP		1
#define MMC_POWER_ON		2
#define MMC_POWER_UNDEFINED	3

	unsigned char	bus_width;		

#define MMC_BUS_WIDTH_1		0
#define MMC_BUS_WIDTH_4		2
#define MMC_BUS_WIDTH_8		3

	unsigned char	timing;			

#define MMC_TIMING_LEGACY	0
#define MMC_TIMING_MMC_HS	1
#define MMC_TIMING_SD_HS	2
#define MMC_TIMING_UHS_SDR12	3
#define MMC_TIMING_UHS_SDR25	4
#define MMC_TIMING_UHS_SDR50	5
#define MMC_TIMING_UHS_SDR104	6
#define MMC_TIMING_UHS_DDR50	7
#define MMC_TIMING_MMC_DDR52	8
#define MMC_TIMING_MMC_HS200	9
#define MMC_TIMING_MMC_HS400	10

	unsigned char	signal_voltage;		

#define MMC_SIGNAL_VOLTAGE_330	0
#define MMC_SIGNAL_VOLTAGE_180	1
#define MMC_SIGNAL_VOLTAGE_120	2

	unsigned char	drv_type;		

#define MMC_SET_DRIVER_TYPE_B	0
#define MMC_SET_DRIVER_TYPE_A	1
#define MMC_SET_DRIVER_TYPE_C	2
#define MMC_SET_DRIVER_TYPE_D	3
};

enum mmc_load {
	MMC_LOAD_HIGH,
	MMC_LOAD_LOW,
};

struct mmc_cmdq_host_ops {
	int (*init)(struct mmc_host *host);
	int (*enable)(struct mmc_host *host);
	void (*disable)(struct mmc_host *host, bool soft);
	int (*request)(struct mmc_host *host, struct mmc_request *mrq);
	void (*post_req)(struct mmc_host *host, int tag, int err);
	int (*halt)(struct mmc_host *host, bool halt);
	void (*reset)(struct mmc_host *host, bool soft);
	void (*dumpstate)(struct mmc_host *host);
};

struct mmc_host_ops {
	int (*init)(struct mmc_host *host);
	int (*enable)(struct mmc_host *host);
	int (*disable)(struct mmc_host *host);
	void	(*post_req)(struct mmc_host *host, struct mmc_request *req,
			    int err);
	void	(*pre_req)(struct mmc_host *host, struct mmc_request *req,
			   bool is_first_req);
	void	(*request)(struct mmc_host *host, struct mmc_request *req);
	void	(*set_ios)(struct mmc_host *host, struct mmc_ios *ios);
	int	(*get_ro)(struct mmc_host *host);
	int	(*get_cd)(struct mmc_host *host);

	void	(*enable_sdio_irq)(struct mmc_host *host, int enable);

	
	void	(*init_card)(struct mmc_host *host, struct mmc_card *card);

	int	(*start_signal_voltage_switch)(struct mmc_host *host, struct mmc_ios *ios);

	
	int	(*card_busy)(struct mmc_host *host);

	
	int	(*execute_tuning)(struct mmc_host *host, u32 opcode);

	
	int	(*prepare_hs400_tuning)(struct mmc_host *host, struct mmc_ios *ios);
	int	(*enhanced_strobe)(struct mmc_host *host);
	int	(*select_drive_strength)(unsigned int max_dtr, int host_drv, int card_drv);
	void	(*hw_reset)(struct mmc_host *host);
	void	(*card_event)(struct mmc_host *host);

	int	(*multi_io_quirk)(struct mmc_card *card,
				  unsigned int direction, int blk_size);

	unsigned long (*get_max_frequency)(struct mmc_host *host);
	unsigned long (*get_min_frequency)(struct mmc_host *host);
	int	(*notify_load)(struct mmc_host *, enum mmc_load);
	void	(*notify_halt)(struct mmc_host *mmc, bool halt);
	void	(*force_err_irq)(struct mmc_host *host, u64 errmask);
};

struct mmc_card;
struct device;

struct mmc_cmdq_req {
	unsigned int cmd_flags;
	u32 blk_addr;
	
	struct mmc_request	mrq;
	struct mmc_data		data;
	struct mmc_command	cmd;
#define DCMD		(1 << 0)
#define QBR		(1 << 1)
#define DIR		(1 << 2)
#define PRIO		(1 << 3)
#define REL_WR		(1 << 4)
#define DAT_TAG	(1 << 5)
#define FORCED_PRG	(1 << 6)
	unsigned int		cmdq_req_flags;

	unsigned int		resp_idx;
	unsigned int		resp_arg;
	unsigned int		dev_pend_tasks;
	bool			resp_err;
	int			tag; 
	u8			ctx_id;
};

struct mmc_async_req {
	
	struct mmc_request	*mrq;
	int (*err_check) (struct mmc_card *, struct mmc_async_req *);
};

struct mmc_slot {
	int cd_irq;
	struct mutex lock;
	void *handler_priv;
};


struct mmc_cmdq_context_info {
	unsigned long	active_reqs; 
	unsigned long	data_active_reqs; 
	unsigned long	curr_state;
#define	CMDQ_STATE_ERR 0
#define	CMDQ_STATE_DCMD_ACTIVE 1
#define	CMDQ_STATE_HALT 2
#define	CMDQ_STATE_CQ_DISABLE 3
#define	CMDQ_STATE_REQ_TIMED_OUT 4
	wait_queue_head_t	queue_empty_wq;
	wait_queue_head_t	wait;
	int active_small_sector_read_reqs;
};

struct mmc_context_info {
	bool			is_done_rcv;
	bool			is_new_req;
	bool			is_waiting_last_req;
	wait_queue_head_t	wait;
	spinlock_t		lock;
};

struct regulator;

struct mmc_supply {
	struct regulator *vmmc;		
	struct regulator *vqmmc;	
};

enum dev_state {
	DEV_SUSPENDING = 1,
	DEV_SUSPENDED,
	DEV_RESUMED,
};

struct mmc_devfeq_clk_scaling {
	spinlock_t	lock;
	struct		devfreq *devfreq;
	struct		devfreq_dev_profile devfreq_profile;
	struct		devfreq_simple_ondemand_data ondemand_gov_data;
	enum mmc_load	state;
	ktime_t		start_busy;
	ktime_t		measure_interval_start;
	atomic_t	devfreq_abort;
	u32		skip_clk_scale_freq_update;
	int		freq_table_sz;
	u32		*freq_table;
	unsigned long	total_busy_time_us;
	unsigned long	target_freq;
	unsigned long	curr_freq;
	unsigned long	polling_delay_ms;
	unsigned int	upthreshold;
	unsigned int	downthreshold;
	unsigned int	lower_bus_speed_mode;
#define MMC_SCALING_LOWER_DDR52_MODE	1
	bool		need_freq_change;
	bool		clk_scaling_in_progress;
	bool		is_busy_started;
	bool		enable;
};

struct mmc_host {
	struct device		*parent;
	struct device		class_dev;
	struct mmc_devfeq_clk_scaling	clk_scaling;
	int			index;
	const struct mmc_host_ops *ops;
	const struct mmc_cmdq_host_ops *cmdq_ops;
	unsigned int		f_min;
	unsigned int		f_max;
	unsigned int		f_init;
	u32			ocr_avail;
	u32			ocr_avail_sdio;	
	u32			ocr_avail_sd;	
	u32			ocr_avail_mmc;	
	struct notifier_block	pm_notify;
#define MMC_DEBUG_MEMORY	0x01
#define MMC_DEBUG_FREE_SPACE	0x02
#define MMC_DEBUG_RANDOM_RW	0x04
	unsigned int            debug_mask;
	u32			max_current_330;
	u32			max_current_300;
	u32			max_current_180;

#define MMC_VDD_165_195		0x00000080	
#define MMC_VDD_20_21		0x00000100	
#define MMC_VDD_21_22		0x00000200	
#define MMC_VDD_22_23		0x00000400	
#define MMC_VDD_23_24		0x00000800	
#define MMC_VDD_24_25		0x00001000	
#define MMC_VDD_25_26		0x00002000	
#define MMC_VDD_26_27		0x00004000	
#define MMC_VDD_27_28		0x00008000	
#define MMC_VDD_28_29		0x00010000	
#define MMC_VDD_29_30		0x00020000	
#define MMC_VDD_30_31		0x00040000	
#define MMC_VDD_31_32		0x00080000	
#define MMC_VDD_32_33		0x00100000	
#define MMC_VDD_33_34		0x00200000	
#define MMC_VDD_34_35		0x00400000	
#define MMC_VDD_35_36		0x00800000	

	u32			caps;		
	u32                     caps_uhs;       

#define MMC_CAP_4_BIT_DATA	(1 << 0)	
#define MMC_CAP_MMC_HIGHSPEED	(1 << 1)	
#define MMC_CAP_SD_HIGHSPEED	(1 << 2)	
#define MMC_CAP_SDIO_IRQ	(1 << 3)	
#define MMC_CAP_SPI		(1 << 4)	
#define MMC_CAP_NEEDS_POLL	(1 << 5)	
#define MMC_CAP_8_BIT_DATA	(1 << 6)	
#define MMC_CAP_AGGRESSIVE_PM	(1 << 7)	
#define MMC_CAP_NONREMOVABLE	(1 << 8)	
#define MMC_CAP_WAIT_WHILE_BUSY	(1 << 9)	
#define MMC_CAP_ERASE		(1 << 10)	
#define MMC_CAP_1_8V_DDR	(1 << 11)	
						
#define MMC_CAP_1_2V_DDR	(1 << 12)	
						
#define MMC_CAP_POWER_OFF_CARD	(1 << 13)	
#define MMC_CAP_BUS_WIDTH_TEST	(1 << 14)	
#define MMC_CAP_UHS_SDR12	(1 << 15)	
#define MMC_CAP_UHS_SDR25	(1 << 16)	
#define MMC_CAP_UHS_SDR50	(1 << 17)	
#define MMC_CAP_UHS_SDR104	(1 << 18)	
#define MMC_CAP_UHS_DDR50	(1 << 19)	
#define MMC_CAP_RUNTIME_RESUME	(1 << 20)	
#define MMC_CAP_DRIVER_TYPE_A	(1 << 23)	
#define MMC_CAP_DRIVER_TYPE_C	(1 << 24)	
#define MMC_CAP_DRIVER_TYPE_D	(1 << 25)	
#define MMC_CAP_CMD23		(1 << 30)	
#define MMC_CAP_HW_RESET	(1 << 31)	

	u32			caps2;		

#define MMC_CAP2_BOOTPART_NOACC	(1 << 0)	
#define MMC_CAP2_FULL_PWR_CYCLE	(1 << 2)	
#define MMC_CAP2_HS200_1_8V_SDR	(1 << 5)        
#define MMC_CAP2_HS200_1_2V_SDR	(1 << 6)        
#define MMC_CAP2_HS200		(MMC_CAP2_HS200_1_8V_SDR | \
				 MMC_CAP2_HS200_1_2V_SDR)
#define MMC_CAP2_HC_ERASE_SZ	(1 << 9)	
#define MMC_CAP2_CD_ACTIVE_HIGH	(1 << 10)	
#define MMC_CAP2_RO_ACTIVE_HIGH	(1 << 11)	
#define MMC_CAP2_PACKED_RD	(1 << 12)	
#define MMC_CAP2_PACKED_WR	(1 << 13)	
#define MMC_CAP2_PACKED_CMD	(MMC_CAP2_PACKED_RD | \
				 MMC_CAP2_PACKED_WR)
#define MMC_CAP2_NO_PRESCAN_POWERUP (1 << 14)	
#define MMC_CAP2_HS400_1_8V	(1 << 15)	
#define MMC_CAP2_HS400_1_2V	(1 << 16)	
#define MMC_CAP2_HS400		(MMC_CAP2_HS400_1_8V | \
				 MMC_CAP2_HS400_1_2V)
#define MMC_CAP2_SDIO_IRQ_NOTHREAD (1 << 17)
#define MMC_CAP2_PACKED_WR_CONTROL (1 << 18)	
#define MMC_CAP2_CLK_SCALE	(1 << 19)	
#define MMC_CAP2_ASYNC_SDIO_IRQ_4BIT_MODE (1 << 20)
#define MMC_CAP2_HS400_POST_TUNING	(1 << 21)
#define MMC_CAP2_NONHOTPLUG	(1 << 25)	
#define MMC_CAP2_CMD_QUEUE	(1 << 26)	
#define MMC_CAP2_SANITIZE       (1 << 27)               
#define MMC_CAP2_SLEEP_AWAKE	(1 << 28)	
#define MMC_CAP2_MAX_DISCARD_SIZE	(1 << 29)

	mmc_pm_flag_t		pm_caps;	

#ifdef CONFIG_MMC_CLKGATE
	int			clk_requests;	
	unsigned int		clk_delay;	
	bool			clk_gated;	
	struct delayed_work	clk_gate_work; 
	unsigned int		clk_old;	
	spinlock_t		clk_lock;	
	struct mutex		clk_gate_mutex;	
	struct device_attribute clkgate_delay_attr;
	unsigned long           clkgate_delay;
#endif

	
	unsigned int		max_seg_size;	
	unsigned short		max_segs;	
	unsigned short		unused;
	unsigned int		max_req_size;	
	unsigned int		max_blk_size;	
	unsigned int		max_blk_count;	
	unsigned int		max_busy_timeout; 

	
	spinlock_t		lock;		

	struct mmc_ios		ios;		
	struct mmc_ios		cached_ios;

	
	unsigned int		use_spi_crc:1;
	unsigned int		claimed:1;	
	unsigned int		bus_dead:1;	
#ifdef CONFIG_MMC_DEBUG
	unsigned int		removed:1;	
#endif

	int			rescan_disable;	
	int			rescan_entered;	

	bool			trigger_card_event; 

	struct mmc_card		*card;		

	wait_queue_head_t	wq;
	struct task_struct	*claimer;	
	struct task_struct	*suspend_task;
	int			claim_cnt;	

	struct delayed_work	detect;
	struct delayed_work	enable_detect;
	struct delayed_work	stats_work;
	int			detect_change;	
	struct mmc_slot		slot;

	const struct mmc_bus_ops *bus_ops;	
	unsigned int		bus_refs;	

	unsigned int		sdio_irqs;
	struct task_struct	*sdio_irq_thread;
	bool			sdio_irq_pending;
	atomic_t		sdio_irq_thread_abort;

	mmc_pm_flag_t		pm_flags;	

	struct led_trigger	*led;		

#ifdef CONFIG_REGULATOR
	bool			regulator_enabled; 
#endif
	struct mmc_supply	supply;

	struct dentry		*debugfs_root;

	struct mmc_async_req	*areq;		
	struct mmc_context_info	context_info;	

#ifdef CONFIG_FAIL_MMC_REQUEST
	struct fault_attr	fail_mmc_request;
#endif

	unsigned int		actual_clock;	

	unsigned int		slotno;	

	int			dsr_req;	
	u32			dsr;	

#ifdef CONFIG_MMC_EMBEDDED_SDIO
	struct {
		struct sdio_cis			*cis;
		struct sdio_cccr		*cccr;
		struct sdio_embedded_func	*funcs;
		int				num_funcs;
	} embedded_sdio_data;
#endif

	bool			card_clock_off;
	unsigned int            crc_count;
	unsigned int            removed_cnt;

	struct {

		unsigned long rbytes_drv;  
		unsigned long wbytes_drv;  
		ktime_t rtime_drv;	   
		ktime_t wtime_drv;	   

		unsigned long rcount;		
		unsigned long wcount;		

		
		unsigned long rbytes_drv_rand;	
		unsigned long wbytes_drv_rand;	
		unsigned long rcount_rand;	
		unsigned long wcount_rand;	
		ktime_t rtime_drv_rand;		
		ktime_t wtime_drv_rand;		
		unsigned long wbytes_low_perf;
		unsigned long wtime_low_perf;
		unsigned long lp_duration;	

		
		unsigned long erase_rq;		
		unsigned long erase_blks;	
		ktime_t erase_time;		

		
		unsigned long wkbytes_drv;
		ktime_t workload_time;

		ktime_t start;
	} perf;
	bool perf_enable;

	enum dev_state dev_status;
	bool			wakeup_on_idle;
	struct mmc_cmdq_context_info	cmdq_ctx;
	int num_cq_slots;
	int dcmd_cq_slot;
	u32			cmdq_thist_enabled;
	void *cmdq_private;
	struct mmc_request	*err_mrq;
	unsigned long		private[0] ____cacheline_aligned;
};

struct mmc_host *mmc_alloc_host(int extra, struct device *);
extern bool mmc_host_may_gate_card(struct mmc_card *);
int mmc_add_host(struct mmc_host *);
void mmc_remove_host(struct mmc_host *);
void mmc_free_host(struct mmc_host *);
int mmc_of_parse(struct mmc_host *host);

#ifdef CONFIG_MMC_EMBEDDED_SDIO
extern void mmc_set_embedded_sdio_data(struct mmc_host *host,
				       struct sdio_cis *cis,
				       struct sdio_cccr *cccr,
				       struct sdio_embedded_func *funcs,
				       int num_funcs);
#endif

static inline void *mmc_priv(struct mmc_host *host)
{
	return (void *)host->private;
}

static inline void *mmc_cmdq_private(struct mmc_host *host)
{
	return host->cmdq_private;
}

#define mmc_host_is_spi(host)	((host)->caps & MMC_CAP_SPI)

#define mmc_dev(x)	((x)->parent)
#define mmc_classdev(x)	(&(x)->class_dev)
#define mmc_hostname(x)	(dev_name(&(x)->class_dev))

int mmc_power_save_host(struct mmc_host *host);
int mmc_power_restore_host(struct mmc_host *host);

void mmc_detect_change(struct mmc_host *, unsigned long delay);
void mmc_request_done(struct mmc_host *, struct mmc_request *);

static inline void mmc_signal_sdio_irq(struct mmc_host *host)
{
	host->ops->enable_sdio_irq(host, 0);
	host->sdio_irq_pending = true;
	wake_up_process(host->sdio_irq_thread);
}

void sdio_run_irqs(struct mmc_host *host);

int mmc_is_sd_host(struct mmc_host *mmc);
int mmc_is_mmc_host(struct mmc_host *mmc);

#ifdef CONFIG_REGULATOR
int mmc_regulator_get_ocrmask(struct regulator *supply);
int mmc_regulator_set_ocr(struct mmc_host *mmc,
			struct regulator *supply,
			unsigned short vdd_bit);
#else
static inline int mmc_regulator_get_ocrmask(struct regulator *supply)
{
	return 0;
}

static inline int mmc_regulator_set_ocr(struct mmc_host *mmc,
				 struct regulator *supply,
				 unsigned short vdd_bit)
{
	return 0;
}
#endif

int mmc_regulator_get_supply(struct mmc_host *mmc);

int mmc_pm_notify(struct notifier_block *notify_block, unsigned long, void *);

static inline int mmc_card_is_removable(struct mmc_host *host)
{
	return !(host->caps & MMC_CAP_NONREMOVABLE);
}

static inline int mmc_card_keep_power(struct mmc_host *host)
{
	return host->pm_flags & MMC_PM_KEEP_POWER;
}

static inline int mmc_card_wake_sdio_irq(struct mmc_host *host)
{
	return host->pm_flags & MMC_PM_WAKE_SDIO_IRQ;
}

static inline int mmc_host_cmd23(struct mmc_host *host)
{
	return host->caps & MMC_CAP_CMD23;
}

static inline int mmc_boot_partition_access(struct mmc_host *host)
{
	return !(host->caps2 & MMC_CAP2_BOOTPART_NOACC);
}

static inline bool mmc_card_and_host_support_async_int(struct mmc_host *host)
{
	return ((host->caps2 & MMC_CAP2_ASYNC_SDIO_IRQ_4BIT_MODE) &&
			(host->card->cccr.async_intr_sup));
}

static inline int mmc_host_uhs(struct mmc_host *host)
{
	return host->caps &
		(MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
		 MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_SDR104 |
		 MMC_CAP_UHS_DDR50);
}

static inline int mmc_host_packed_wr(struct mmc_host *host)
{
	return host->caps2 & MMC_CAP2_PACKED_WR;
}

static inline void mmc_host_set_halt(struct mmc_host *host)
{
	set_bit(CMDQ_STATE_HALT, &host->cmdq_ctx.curr_state);
}

static inline void mmc_host_clr_halt(struct mmc_host *host)
{
	clear_bit(CMDQ_STATE_HALT, &host->cmdq_ctx.curr_state);
}

static inline int mmc_host_halt(struct mmc_host *host)
{
	return test_bit(CMDQ_STATE_HALT, &host->cmdq_ctx.curr_state);
}

static inline void mmc_host_set_cq_disable(struct mmc_host *host)
{
	set_bit(CMDQ_STATE_CQ_DISABLE, &host->cmdq_ctx.curr_state);
}

static inline void mmc_host_clr_cq_disable(struct mmc_host *host)
{
	clear_bit(CMDQ_STATE_CQ_DISABLE, &host->cmdq_ctx.curr_state);
}

static inline int mmc_host_cq_disable(struct mmc_host *host)
{
	return test_bit(CMDQ_STATE_CQ_DISABLE, &host->cmdq_ctx.curr_state);
}

#ifdef CONFIG_MMC_CLKGATE
void mmc_host_clk_hold(struct mmc_host *host);
void mmc_host_clk_release(struct mmc_host *host);
unsigned int mmc_host_clk_rate(struct mmc_host *host);

#else
static inline void mmc_host_clk_hold(struct mmc_host *host)
{
}

static inline void mmc_host_clk_release(struct mmc_host *host)
{
}

static inline unsigned int mmc_host_clk_rate(struct mmc_host *host)
{
	return host->ios.clock;
}
#endif

static inline int mmc_card_hs(struct mmc_card *card)
{
	return card->host->ios.timing == MMC_TIMING_SD_HS ||
		card->host->ios.timing == MMC_TIMING_MMC_HS;
}

static inline int mmc_card_uhs(struct mmc_card *card)
{
	return card->host->ios.timing >= MMC_TIMING_UHS_SDR12 &&
		card->host->ios.timing <= MMC_TIMING_UHS_DDR50;
}

static inline bool mmc_card_hs200(struct mmc_card *card)
{
	return card->host->ios.timing == MMC_TIMING_MMC_HS200;
}

static inline bool mmc_card_ddr52(struct mmc_card *card)
{
	return card->host->ios.timing == MMC_TIMING_MMC_DDR52;
}

static inline bool mmc_card_hs400(struct mmc_card *card)
{
	return card->host->ios.timing == MMC_TIMING_MMC_HS400;
}

#endif 
