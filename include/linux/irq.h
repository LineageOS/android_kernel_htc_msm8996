#ifndef _LINUX_IRQ_H
#define _LINUX_IRQ_H


#include <linux/smp.h>
#include <linux/linkage.h>
#include <linux/cache.h>
#include <linux/spinlock.h>
#include <linux/cpumask.h>
#include <linux/gfp.h>
#include <linux/irqhandler.h>
#include <linux/irqreturn.h>
#include <linux/irqnr.h>
#include <linux/errno.h>
#include <linux/topology.h>
#include <linux/wait.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/ptrace.h>
#include <asm/irq_regs.h>

struct seq_file;
struct module;
struct msi_msg;

enum {
	IRQ_TYPE_NONE		= 0x00000000,
	IRQ_TYPE_EDGE_RISING	= 0x00000001,
	IRQ_TYPE_EDGE_FALLING	= 0x00000002,
	IRQ_TYPE_EDGE_BOTH	= (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING),
	IRQ_TYPE_LEVEL_HIGH	= 0x00000004,
	IRQ_TYPE_LEVEL_LOW	= 0x00000008,
	IRQ_TYPE_LEVEL_MASK	= (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH),
	IRQ_TYPE_SENSE_MASK	= 0x0000000f,
	IRQ_TYPE_DEFAULT	= IRQ_TYPE_SENSE_MASK,

	IRQ_TYPE_PROBE		= 0x00000010,

	IRQ_LEVEL		= (1 <<  8),
	IRQ_PER_CPU		= (1 <<  9),
	IRQ_NOPROBE		= (1 << 10),
	IRQ_NOREQUEST		= (1 << 11),
	IRQ_NOAUTOEN		= (1 << 12),
	IRQ_NO_BALANCING	= (1 << 13),
	IRQ_MOVE_PCNTXT		= (1 << 14),
	IRQ_NESTED_THREAD	= (1 << 15),
	IRQ_NOTHREAD		= (1 << 16),
	IRQ_PER_CPU_DEVID	= (1 << 17),
	IRQ_IS_POLLED		= (1 << 18),
};

#define IRQF_MODIFY_MASK	\
	(IRQ_TYPE_SENSE_MASK | IRQ_NOPROBE | IRQ_NOREQUEST | \
	 IRQ_NOAUTOEN | IRQ_MOVE_PCNTXT | IRQ_LEVEL | IRQ_NO_BALANCING | \
	 IRQ_PER_CPU | IRQ_NESTED_THREAD | IRQ_NOTHREAD | IRQ_PER_CPU_DEVID | \
	 IRQ_IS_POLLED)

#define IRQ_NO_BALANCING_MASK	(IRQ_PER_CPU | IRQ_NO_BALANCING)

enum {
	IRQ_SET_MASK_OK = 0,
	IRQ_SET_MASK_OK_NOCOPY,
	IRQ_SET_MASK_OK_DONE,
};

struct msi_desc;
struct irq_domain;

struct irq_data {
	u32			mask;
	unsigned int		irq;
	unsigned long		hwirq;
	unsigned int		node;
	unsigned int		state_use_accessors;
	struct irq_chip		*chip;
	struct irq_domain	*domain;
#ifdef	CONFIG_IRQ_DOMAIN_HIERARCHY
	struct irq_data		*parent_data;
#endif
	void			*handler_data;
	void			*chip_data;
	struct msi_desc		*msi_desc;
	cpumask_var_t		affinity;
};

enum {
	IRQD_TRIGGER_MASK		= 0xf,
	IRQD_SETAFFINITY_PENDING	= (1 <<  8),
	IRQD_NO_BALANCING		= (1 << 10),
	IRQD_PER_CPU			= (1 << 11),
	IRQD_AFFINITY_SET		= (1 << 12),
	IRQD_LEVEL			= (1 << 13),
	IRQD_WAKEUP_STATE		= (1 << 14),
	IRQD_MOVE_PCNTXT		= (1 << 15),
	IRQD_IRQ_DISABLED		= (1 << 16),
	IRQD_IRQ_MASKED			= (1 << 17),
	IRQD_IRQ_INPROGRESS		= (1 << 18),
	IRQD_WAKEUP_ARMED		= (1 << 19),
};

static inline bool irqd_is_setaffinity_pending(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_SETAFFINITY_PENDING;
}

static inline bool irqd_is_per_cpu(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_PER_CPU;
}

static inline bool irqd_can_balance(struct irq_data *d)
{
	return !(d->state_use_accessors & (IRQD_PER_CPU | IRQD_NO_BALANCING));
}

static inline bool irqd_affinity_was_set(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_AFFINITY_SET;
}

static inline void irqd_mark_affinity_was_set(struct irq_data *d)
{
	d->state_use_accessors |= IRQD_AFFINITY_SET;
}

static inline u32 irqd_get_trigger_type(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_TRIGGER_MASK;
}

static inline void irqd_set_trigger_type(struct irq_data *d, u32 type)
{
	d->state_use_accessors &= ~IRQD_TRIGGER_MASK;
	d->state_use_accessors |= type & IRQD_TRIGGER_MASK;
}

static inline bool irqd_is_level_type(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_LEVEL;
}

static inline bool irqd_is_wakeup_set(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_WAKEUP_STATE;
}

static inline bool irqd_can_move_in_process_context(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_MOVE_PCNTXT;
}

static inline bool irqd_irq_disabled(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_IRQ_DISABLED;
}

static inline bool irqd_irq_masked(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_IRQ_MASKED;
}

static inline bool irqd_irq_inprogress(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_IRQ_INPROGRESS;
}

static inline bool irqd_is_wakeup_armed(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_WAKEUP_ARMED;
}


static inline void irqd_set_chained_irq_inprogress(struct irq_data *d)
{
	d->state_use_accessors |= IRQD_IRQ_INPROGRESS;
}

static inline void irqd_clr_chained_irq_inprogress(struct irq_data *d)
{
	d->state_use_accessors &= ~IRQD_IRQ_INPROGRESS;
}

static inline irq_hw_number_t irqd_to_hwirq(struct irq_data *d)
{
	return d->hwirq;
}

struct irq_chip {
	const char	*name;
	unsigned int	(*irq_startup)(struct irq_data *data);
	void		(*irq_shutdown)(struct irq_data *data);
	void		(*irq_enable)(struct irq_data *data);
	void		(*irq_disable)(struct irq_data *data);

	void		(*irq_ack)(struct irq_data *data);
	void		(*irq_mask)(struct irq_data *data);
	void		(*irq_mask_ack)(struct irq_data *data);
	void		(*irq_unmask)(struct irq_data *data);
	void		(*irq_eoi)(struct irq_data *data);

	int		(*irq_set_affinity)(struct irq_data *data, const struct cpumask *dest, bool force);
	int		(*irq_retrigger)(struct irq_data *data);
	int		(*irq_set_type)(struct irq_data *data, unsigned int flow_type);
	int		(*irq_read_line)(struct irq_data *data);
	int		(*irq_set_wake)(struct irq_data *data, unsigned int on);

	void		(*irq_bus_lock)(struct irq_data *data);
	void		(*irq_bus_sync_unlock)(struct irq_data *data);

	void		(*irq_cpu_online)(struct irq_data *data);
	void		(*irq_cpu_offline)(struct irq_data *data);

	void		(*irq_suspend)(struct irq_data *data);
	void		(*irq_resume)(struct irq_data *data);
	void		(*irq_pm_shutdown)(struct irq_data *data);

	void		(*irq_calc_mask)(struct irq_data *data);

	void		(*irq_print_chip)(struct irq_data *data, struct seq_file *p);
	int		(*irq_request_resources)(struct irq_data *data);
	void		(*irq_release_resources)(struct irq_data *data);

	void		(*irq_compose_msi_msg)(struct irq_data *data, struct msi_msg *msg);
	void		(*irq_write_msi_msg)(struct irq_data *data, struct msi_msg *msg);

	unsigned long	flags;
};

enum {
	IRQCHIP_SET_TYPE_MASKED		= (1 <<  0),
	IRQCHIP_EOI_IF_HANDLED		= (1 <<  1),
	IRQCHIP_MASK_ON_SUSPEND		= (1 <<  2),
	IRQCHIP_ONOFFLINE_ENABLED	= (1 <<  3),
	IRQCHIP_SKIP_SET_WAKE		= (1 <<  4),
	IRQCHIP_ONESHOT_SAFE		= (1 <<  5),
	IRQCHIP_EOI_THREADED		= (1 <<  6),
};

#include <linux/irqdesc.h>

#include <asm/hw_irq.h>

#ifndef NR_IRQS_LEGACY
# define NR_IRQS_LEGACY 0
#endif

#ifndef ARCH_IRQ_INIT_FLAGS
# define ARCH_IRQ_INIT_FLAGS	0
#endif

#define IRQ_DEFAULT_INIT_FLAGS	ARCH_IRQ_INIT_FLAGS

struct irqaction;
extern int setup_irq(unsigned int irq, struct irqaction *new);
extern void remove_irq(unsigned int irq, struct irqaction *act);
extern int setup_percpu_irq(unsigned int irq, struct irqaction *new);
extern void remove_percpu_irq(unsigned int irq, struct irqaction *act);

extern void irq_cpu_online(void);
extern void irq_cpu_offline(void);
extern int irq_set_affinity_locked(struct irq_data *data,
				   const struct cpumask *cpumask, bool force);

#if defined(CONFIG_SMP) && defined(CONFIG_GENERIC_PENDING_IRQ)
void irq_move_irq(struct irq_data *data);
void irq_move_masked_irq(struct irq_data *data);
#else
static inline void irq_move_irq(struct irq_data *data) { }
static inline void irq_move_masked_irq(struct irq_data *data) { }
#endif

extern int no_irq_affinity;

#ifdef CONFIG_HARDIRQS_SW_RESEND
int irq_set_parent(int irq, int parent_irq);
#else
static inline int irq_set_parent(int irq, int parent_irq)
{
	return 0;
}
#endif

extern void handle_level_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_fasteoi_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_edge_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_edge_eoi_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_simple_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_percpu_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_percpu_devid_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_bad_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_nested_irq(unsigned int irq);

extern int irq_chip_compose_msi_msg(struct irq_data *data, struct msi_msg *msg);
#ifdef	CONFIG_IRQ_DOMAIN_HIERARCHY
extern void irq_chip_ack_parent(struct irq_data *data);
extern int irq_chip_retrigger_hierarchy(struct irq_data *data);
extern void irq_chip_mask_parent(struct irq_data *data);
extern void irq_chip_unmask_parent(struct irq_data *data);
extern void irq_chip_eoi_parent(struct irq_data *data);
extern int irq_chip_set_affinity_parent(struct irq_data *data,
					const struct cpumask *dest,
					bool force);
#endif

extern void note_interrupt(unsigned int irq, struct irq_desc *desc,
			   irqreturn_t action_ret);

void check_irq_resend(struct irq_desc *desc, unsigned int irq);

extern int noirqdebug_setup(char *str);

extern int can_request_irq(unsigned int irq, unsigned long irqflags);

extern struct irq_chip no_irq_chip;
extern struct irq_chip dummy_irq_chip;

extern void
irq_set_chip_and_handler_name(unsigned int irq, struct irq_chip *chip,
			      irq_flow_handler_t handle, const char *name);

static inline void irq_set_chip_and_handler(unsigned int irq, struct irq_chip *chip,
					    irq_flow_handler_t handle)
{
	irq_set_chip_and_handler_name(irq, chip, handle, NULL);
}

extern int irq_set_percpu_devid(unsigned int irq);

extern void
__irq_set_handler(unsigned int irq, irq_flow_handler_t handle, int is_chained,
		  const char *name);

static inline void
irq_set_handler(unsigned int irq, irq_flow_handler_t handle)
{
	__irq_set_handler(irq, handle, 0, NULL);
}

static inline void
irq_set_chained_handler(unsigned int irq, irq_flow_handler_t handle)
{
	__irq_set_handler(irq, handle, 1, NULL);
}

void irq_modify_status(unsigned int irq, unsigned long clr, unsigned long set);

static inline void irq_set_status_flags(unsigned int irq, unsigned long set)
{
	irq_modify_status(irq, 0, set);
}

static inline void irq_clear_status_flags(unsigned int irq, unsigned long clr)
{
	irq_modify_status(irq, clr, 0);
}

static inline void irq_set_noprobe(unsigned int irq)
{
	irq_modify_status(irq, 0, IRQ_NOPROBE);
}

static inline void irq_set_probe(unsigned int irq)
{
	irq_modify_status(irq, IRQ_NOPROBE, 0);
}

static inline void irq_set_nothread(unsigned int irq)
{
	irq_modify_status(irq, 0, IRQ_NOTHREAD);
}

static inline void irq_set_thread(unsigned int irq)
{
	irq_modify_status(irq, IRQ_NOTHREAD, 0);
}

static inline void irq_set_nested_thread(unsigned int irq, bool nest)
{
	if (nest)
		irq_set_status_flags(irq, IRQ_NESTED_THREAD);
	else
		irq_clear_status_flags(irq, IRQ_NESTED_THREAD);
}

static inline void irq_set_percpu_devid_flags(unsigned int irq)
{
	irq_set_status_flags(irq,
			     IRQ_NOAUTOEN | IRQ_PER_CPU | IRQ_NOTHREAD |
			     IRQ_NOPROBE | IRQ_PER_CPU_DEVID);
}

extern int irq_set_chip(unsigned int irq, struct irq_chip *chip);
extern int irq_set_handler_data(unsigned int irq, void *data);
extern int irq_set_chip_data(unsigned int irq, void *data);
extern int irq_set_irq_type(unsigned int irq, unsigned int type);
extern int irq_set_msi_desc(unsigned int irq, struct msi_desc *entry);
extern int irq_set_msi_desc_off(unsigned int irq_base, unsigned int irq_offset,
				struct msi_desc *entry);
extern struct irq_data *irq_get_irq_data(unsigned int irq);

static inline struct irq_chip *irq_get_chip(unsigned int irq)
{
	struct irq_data *d = irq_get_irq_data(irq);
	return d ? d->chip : NULL;
}

static inline struct irq_chip *irq_data_get_irq_chip(struct irq_data *d)
{
	return d->chip;
}

static inline void *irq_get_chip_data(unsigned int irq)
{
	struct irq_data *d = irq_get_irq_data(irq);
	return d ? d->chip_data : NULL;
}

static inline void *irq_data_get_irq_chip_data(struct irq_data *d)
{
	return d->chip_data;
}

static inline void *irq_get_handler_data(unsigned int irq)
{
	struct irq_data *d = irq_get_irq_data(irq);
	return d ? d->handler_data : NULL;
}

static inline void *irq_data_get_irq_handler_data(struct irq_data *d)
{
	return d->handler_data;
}

static inline struct msi_desc *irq_get_msi_desc(unsigned int irq)
{
	struct irq_data *d = irq_get_irq_data(irq);
	return d ? d->msi_desc : NULL;
}

static inline struct msi_desc *irq_data_get_msi_desc(struct irq_data *d)
{
	return d->msi_desc;
}

static inline u32 irq_get_trigger_type(unsigned int irq)
{
	struct irq_data *d = irq_get_irq_data(irq);
	return d ? irqd_get_trigger_type(d) : 0;
}

unsigned int arch_dynirq_lower_bound(unsigned int from);

int __irq_alloc_descs(int irq, unsigned int from, unsigned int cnt, int node,
		struct module *owner);

#define irq_alloc_descs(irq, from, cnt, node)	\
	__irq_alloc_descs(irq, from, cnt, node, THIS_MODULE)

#define irq_alloc_desc(node)			\
	irq_alloc_descs(-1, 0, 1, node)

#define irq_alloc_desc_at(at, node)		\
	irq_alloc_descs(at, at, 1, node)

#define irq_alloc_desc_from(from, node)		\
	irq_alloc_descs(-1, from, 1, node)

#define irq_alloc_descs_from(from, cnt, node)	\
	irq_alloc_descs(-1, from, cnt, node)

void irq_free_descs(unsigned int irq, unsigned int cnt);
static inline void irq_free_desc(unsigned int irq)
{
	irq_free_descs(irq, 1);
}

#ifdef CONFIG_GENERIC_IRQ_LEGACY_ALLOC_HWIRQ
unsigned int irq_alloc_hwirqs(int cnt, int node);
static inline unsigned int irq_alloc_hwirq(int node)
{
	return irq_alloc_hwirqs(1, node);
}
void irq_free_hwirqs(unsigned int from, int cnt);
static inline void irq_free_hwirq(unsigned int irq)
{
	return irq_free_hwirqs(irq, 1);
}
int arch_setup_hwirq(unsigned int irq, int node);
void arch_teardown_hwirq(unsigned int irq);
#endif

#ifdef CONFIG_GENERIC_IRQ_LEGACY
void irq_init_desc(unsigned int irq);
#endif

struct irq_chip_regs {
	unsigned long		enable;
	unsigned long		disable;
	unsigned long		mask;
	unsigned long		ack;
	unsigned long		eoi;
	unsigned long		type;
	unsigned long		polarity;
};

struct irq_chip_type {
	struct irq_chip		chip;
	struct irq_chip_regs	regs;
	irq_flow_handler_t	handler;
	u32			type;
	u32			mask_cache_priv;
	u32			*mask_cache;
};

struct irq_chip_generic {
	raw_spinlock_t		lock;
	void __iomem		*reg_base;
	u32			(*reg_readl)(void __iomem *addr);
	void			(*reg_writel)(u32 val, void __iomem *addr);
	unsigned int		irq_base;
	unsigned int		irq_cnt;
	u32			mask_cache;
	u32			type_cache;
	u32			polarity_cache;
	u32			wake_enabled;
	u32			wake_active;
	unsigned int		num_ct;
	void			*private;
	unsigned long		installed;
	unsigned long		unused;
	struct irq_domain	*domain;
	struct list_head	list;
	struct irq_chip_type	chip_types[0];
};

enum irq_gc_flags {
	IRQ_GC_INIT_MASK_CACHE		= 1 << 0,
	IRQ_GC_INIT_NESTED_LOCK		= 1 << 1,
	IRQ_GC_MASK_CACHE_PER_TYPE	= 1 << 2,
	IRQ_GC_NO_MASK			= 1 << 3,
	IRQ_GC_BE_IO			= 1 << 4,
};

struct irq_domain_chip_generic {
	unsigned int		irqs_per_chip;
	unsigned int		num_chips;
	unsigned int		irq_flags_to_clear;
	unsigned int		irq_flags_to_set;
	enum irq_gc_flags	gc_flags;
	struct irq_chip_generic	*gc[0];
};

void irq_gc_noop(struct irq_data *d);
void irq_gc_mask_disable_reg(struct irq_data *d);
void irq_gc_mask_set_bit(struct irq_data *d);
void irq_gc_mask_clr_bit(struct irq_data *d);
void irq_gc_unmask_enable_reg(struct irq_data *d);
void irq_gc_ack_set_bit(struct irq_data *d);
void irq_gc_ack_clr_bit(struct irq_data *d);
void irq_gc_mask_disable_reg_and_ack(struct irq_data *d);
void irq_gc_eoi(struct irq_data *d);
int irq_gc_set_wake(struct irq_data *d, unsigned int on);

int irq_map_generic_chip(struct irq_domain *d, unsigned int virq,
			 irq_hw_number_t hw_irq);
struct irq_chip_generic *
irq_alloc_generic_chip(const char *name, int nr_ct, unsigned int irq_base,
		       void __iomem *reg_base, irq_flow_handler_t handler);
void irq_setup_generic_chip(struct irq_chip_generic *gc, u32 msk,
			    enum irq_gc_flags flags, unsigned int clr,
			    unsigned int set);
int irq_setup_alt_chip(struct irq_data *d, unsigned int type);
void irq_remove_generic_chip(struct irq_chip_generic *gc, u32 msk,
			     unsigned int clr, unsigned int set);

struct irq_chip_generic *irq_get_domain_generic_chip(struct irq_domain *d, unsigned int hw_irq);
int irq_alloc_domain_generic_chips(struct irq_domain *d, int irqs_per_chip,
				   int num_ct, const char *name,
				   irq_flow_handler_t handler,
				   unsigned int clr, unsigned int set,
				   enum irq_gc_flags flags);


static inline struct irq_chip_type *irq_data_get_chip_type(struct irq_data *d)
{
	return container_of(d->chip, struct irq_chip_type, chip);
}

#define IRQ_MSK(n) (u32)((n) < 32 ? ((1 << (n)) - 1) : UINT_MAX)

#ifdef CONFIG_SMP
static inline void irq_gc_lock(struct irq_chip_generic *gc)
{
	raw_spin_lock(&gc->lock);
}

static inline void irq_gc_unlock(struct irq_chip_generic *gc)
{
	raw_spin_unlock(&gc->lock);
}
#else
static inline void irq_gc_lock(struct irq_chip_generic *gc) { }
static inline void irq_gc_unlock(struct irq_chip_generic *gc) { }
#endif

#ifdef CONFIG_HTC_POWER_DEBUG
void htc_show_interrupts(void);
#endif

static inline void irq_reg_writel(struct irq_chip_generic *gc,
				  u32 val, int reg_offset)
{
	if (gc->reg_writel)
		gc->reg_writel(val, gc->reg_base + reg_offset);
	else
		writel(val, gc->reg_base + reg_offset);
}

static inline u32 irq_reg_readl(struct irq_chip_generic *gc,
				int reg_offset)
{
	if (gc->reg_readl)
		return gc->reg_readl(gc->reg_base + reg_offset);
	else
		return readl(gc->reg_base + reg_offset);
}

#endif 
