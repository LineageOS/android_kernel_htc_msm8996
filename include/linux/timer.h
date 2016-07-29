#ifndef _LINUX_TIMER_H
#define _LINUX_TIMER_H

#include <linux/list.h>
#include <linux/ktime.h>
#include <linux/stddef.h>
#include <linux/debugobjects.h>
#include <linux/stringify.h>

struct tvec_base;

struct timer_list {
	struct list_head entry;
	unsigned long expires;
	struct tvec_base *base;

	void (*function)(unsigned long);
	unsigned long data;

	int slack;

#ifdef CONFIG_TIMER_STATS
	int start_pid;
	void *start_site;
	char start_comm[16];
#endif
#ifdef CONFIG_LOCKDEP
	struct lockdep_map lockdep_map;
#endif
};

extern struct tvec_base boot_tvec_bases;
#ifdef CONFIG_SMP
extern struct tvec_base tvec_base_deferrable;
#endif

#ifdef CONFIG_LOCKDEP
#define __TIMER_LOCKDEP_MAP_INITIALIZER(_kn)				\
	.lockdep_map = STATIC_LOCKDEP_MAP_INIT(_kn, &_kn),
#else
#define __TIMER_LOCKDEP_MAP_INITIALIZER(_kn)
#endif

#define TIMER_DEFERRABLE		0x1LU
#define TIMER_IRQSAFE			0x2LU

#define TIMER_FLAG_MASK			0x3LU

#ifdef CONFIG_SMP
#define __TIMER_BASE(_flags) \
	((_flags) & TIMER_DEFERRABLE ? \
	 (unsigned long)&tvec_base_deferrable + (_flags) : \
	 (unsigned long)&boot_tvec_bases + (_flags))
#else
#define __TIMER_BASE(_flags) ((unsigned long)&boot_tvec_bases + (_flags))
#endif

#define __TIMER_INITIALIZER(_function, _expires, _data, _flags) { \
		.entry = { .prev = TIMER_ENTRY_STATIC },	\
		.function = (_function),			\
		.expires = (_expires),				\
		.data = (_data),				\
		.base = (void *)(__TIMER_BASE(_flags)),		\
		.slack = -1,					\
		__TIMER_LOCKDEP_MAP_INITIALIZER(		\
			__FILE__ ":" __stringify(__LINE__))	\
	}

#define TIMER_INITIALIZER(_function, _expires, _data)		\
	__TIMER_INITIALIZER((_function), (_expires), (_data), 0)

#define TIMER_DEFERRED_INITIALIZER(_function, _expires, _data)	\
	__TIMER_INITIALIZER((_function), (_expires), (_data), TIMER_DEFERRABLE)

#define DEFINE_TIMER(_name, _function, _expires, _data)		\
	struct timer_list _name =				\
		TIMER_INITIALIZER(_function, _expires, _data)

void init_timer_key(struct timer_list *timer, unsigned int flags,
		    const char *name, struct lock_class_key *key);

#ifdef CONFIG_DEBUG_OBJECTS_TIMERS
extern void init_timer_on_stack_key(struct timer_list *timer,
				    unsigned int flags, const char *name,
				    struct lock_class_key *key);
extern void destroy_timer_on_stack(struct timer_list *timer);
#else
static inline void destroy_timer_on_stack(struct timer_list *timer) { }
static inline void init_timer_on_stack_key(struct timer_list *timer,
					   unsigned int flags, const char *name,
					   struct lock_class_key *key)
{
	init_timer_key(timer, flags, name, key);
}
#endif

#ifdef CONFIG_LOCKDEP
#define __init_timer(_timer, _flags)					\
	do {								\
		static struct lock_class_key __key;			\
		init_timer_key((_timer), (_flags), #_timer, &__key);	\
	} while (0)

#define __init_timer_on_stack(_timer, _flags)				\
	do {								\
		static struct lock_class_key __key;			\
		init_timer_on_stack_key((_timer), (_flags), #_timer, &__key); \
	} while (0)
#else
#define __init_timer(_timer, _flags)					\
	init_timer_key((_timer), (_flags), NULL, NULL)
#define __init_timer_on_stack(_timer, _flags)				\
	init_timer_on_stack_key((_timer), (_flags), NULL, NULL)
#endif

#define init_timer(timer)						\
	__init_timer((timer), 0)
#define init_timer_deferrable(timer)					\
	__init_timer((timer), TIMER_DEFERRABLE)
#define init_timer_on_stack(timer)					\
	__init_timer_on_stack((timer), 0)

#define __setup_timer(_timer, _fn, _data, _flags)			\
	do {								\
		__init_timer((_timer), (_flags));			\
		(_timer)->function = (_fn);				\
		(_timer)->data = (_data);				\
	} while (0)

#define __setup_timer_on_stack(_timer, _fn, _data, _flags)		\
	do {								\
		__init_timer_on_stack((_timer), (_flags));		\
		(_timer)->function = (_fn);				\
		(_timer)->data = (_data);				\
	} while (0)

#define setup_timer(timer, fn, data)					\
	__setup_timer((timer), (fn), (data), 0)
#define setup_timer_on_stack(timer, fn, data)				\
	__setup_timer_on_stack((timer), (fn), (data), 0)
#define setup_deferrable_timer_on_stack(timer, fn, data)		\
	__setup_timer_on_stack((timer), (fn), (data), TIMER_DEFERRABLE)

static inline int timer_pending(const struct timer_list * timer)
{
	return timer->entry.next != NULL;
}

extern void add_timer_on(struct timer_list *timer, int cpu);
extern int del_timer(struct timer_list * timer);
extern int mod_timer(struct timer_list *timer, unsigned long expires);
extern int mod_timer_pending(struct timer_list *timer, unsigned long expires);
extern int mod_timer_pinned(struct timer_list *timer, unsigned long expires);

extern void set_timer_slack(struct timer_list *time, int slack_hz);
#ifdef CONFIG_SMP
extern bool check_pending_deferrable_timers(int cpu);
#endif

#define TIMER_NOT_PINNED	0
#define TIMER_PINNED		1
#define NEXT_TIMER_MAX_DELTA	((1UL << 30) - 1)

extern unsigned long get_next_timer_interrupt(unsigned long now);

#ifdef CONFIG_TIMER_STATS

extern int timer_stats_active;

#define TIMER_STATS_FLAG_DEFERRABLE	0x1

extern void init_timer_stats(void);

extern void timer_stats_update_stats(void *timer, pid_t pid, void *startf,
				     void *timerf, char *comm,
				     unsigned int timer_flag);

extern void __timer_stats_timer_set_start_info(struct timer_list *timer,
					       void *addr);

static inline void timer_stats_timer_set_start_info(struct timer_list *timer)
{
	if (likely(!timer_stats_active))
		return;
	__timer_stats_timer_set_start_info(timer, __builtin_return_address(0));
}

static inline void timer_stats_timer_clear_start_info(struct timer_list *timer)
{
	timer->start_site = NULL;
}
#else
static inline void init_timer_stats(void)
{
}

static inline void timer_stats_timer_set_start_info(struct timer_list *timer)
{
}

static inline void timer_stats_timer_clear_start_info(struct timer_list *timer)
{
}
#endif

extern void add_timer(struct timer_list *timer);

extern int try_to_del_timer_sync(struct timer_list *timer);

#ifdef CONFIG_SMP
  extern int del_timer_sync(struct timer_list *timer);
#else
# define del_timer_sync(t)		del_timer(t)
#endif

#define del_singleshot_timer_sync(t) del_timer_sync(t)

extern void init_timers(void);
extern void run_local_timers(void);
struct hrtimer;
extern enum hrtimer_restart it_real_fn(struct hrtimer *);

unsigned long __round_jiffies(unsigned long j, int cpu);
unsigned long __round_jiffies_relative(unsigned long j, int cpu);
unsigned long round_jiffies(unsigned long j);
unsigned long round_jiffies_relative(unsigned long j);

unsigned long __round_jiffies_up(unsigned long j, int cpu);
unsigned long __round_jiffies_up_relative(unsigned long j, int cpu);
unsigned long round_jiffies_up(unsigned long j);
unsigned long round_jiffies_up_relative(unsigned long j);
#ifdef CONFIG_HTC_POWER_DEBUG
void htc_timer_stats_show(u16 water_mark);
void htc_timer_stats_onoff(char onoff);
#endif
#endif
