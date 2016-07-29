#ifndef _LINUX_MM_TYPES_H
#define _LINUX_MM_TYPES_H

#include <linux/auxvec.h>
#include <linux/types.h>
#include <linux/threads.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rbtree.h>
#include <linux/rwsem.h>
#include <linux/stacktrace.h>
#include <linux/completion.h>
#include <linux/cpumask.h>
#include <linux/page-debug-flags.h>
#include <linux/uprobes.h>
#include <linux/page-flags-layout.h>
#include <asm/page.h>
#include <asm/mmu.h>
#include <htc_debug/stability/debug_page_user_trace.h>

#ifndef AT_VECTOR_SIZE_ARCH
#define AT_VECTOR_SIZE_ARCH 0
#endif
#define AT_VECTOR_SIZE (2*(AT_VECTOR_SIZE_ARCH + AT_VECTOR_SIZE_BASE + 1))

struct address_space;

#define USE_SPLIT_PTE_PTLOCKS	(NR_CPUS >= CONFIG_SPLIT_PTLOCK_CPUS)
#define USE_SPLIT_PMD_PTLOCKS	(USE_SPLIT_PTE_PTLOCKS && \
		IS_ENABLED(CONFIG_ARCH_ENABLE_SPLIT_PMD_PTLOCK))
#define ALLOC_SPLIT_PTLOCKS	(SPINLOCK_SIZE > BITS_PER_LONG/8)

struct page {
	
	unsigned long flags;		
	union {
		struct address_space *mapping;	
		void *s_mem;			
	};

	
	struct {
		union {
			pgoff_t index;		
			void *freelist;		
			bool pfmemalloc;	
		};

		union {
#if defined(CONFIG_HAVE_CMPXCHG_DOUBLE) && \
	defined(CONFIG_HAVE_ALIGNED_STRUCT_PAGE)
			
			unsigned long counters;
#else
			unsigned counters;
#endif

			struct {

				union {
					atomic_t _mapcount;

					struct { 
						unsigned inuse:16;
						unsigned objects:15;
						unsigned frozen:1;
					};
					int units;	
				};
				atomic_t _count;		
			};
			unsigned int active;	
		};
	};

	
	union {
		struct list_head lru;	
		struct {		
			struct page *next;	
#ifdef CONFIG_64BIT
			int pages;	
			int pobjects;	
#else
			short int pages;
			short int pobjects;
#endif
		};

		struct slab *slab_page; 
		struct rcu_head rcu_head;	
#if defined(CONFIG_TRANSPARENT_HUGEPAGE) && USE_SPLIT_PMD_PTLOCKS
		pgtable_t pmd_huge_pte; 
#endif
	};

	
	union {
		unsigned long private;		
#if USE_SPLIT_PTE_PTLOCKS
#if ALLOC_SPLIT_PTLOCKS
		spinlock_t *ptl;
#else
		spinlock_t ptl;
#endif
#endif
		struct kmem_cache *slab_cache;	
		struct page *first_page;	
	};

#if defined(WANT_PAGE_VIRTUAL)
	void *virtual;			
#endif 
#ifdef CONFIG_WANT_PAGE_DEBUG_FLAGS
	unsigned long debug_flags;	
#endif
#ifdef CONFIG_BLK_DEV_IO_TRACE
	struct task_struct *tsk_dirty;	
#endif

#ifdef CONFIG_KMEMCHECK
	void *shadow;
#endif

#ifdef LAST_CPUPID_NOT_IN_PAGE_FLAGS
	int _last_cpupid;
#endif
#ifdef CONFIG_PAGE_OWNER
	int order;
	gfp_t gfp_mask;
	struct stack_trace trace;
	unsigned long trace_entries[8];
#endif

	DECLARE_PAGE_USER_TRACE(trace_alloc);
	DECLARE_PAGE_USER_TRACE(trace_free);
}
#ifdef CONFIG_HAVE_ALIGNED_STRUCT_PAGE
	__aligned(2 * sizeof(unsigned long))
#endif
;

struct page_frag {
	struct page *page;
#if (BITS_PER_LONG > 32) || (PAGE_SIZE >= 65536)
	__u32 offset;
	__u32 size;
#else
	__u16 offset;
	__u16 size;
#endif
};

typedef unsigned long __nocast vm_flags_t;

struct vm_region {
	struct rb_node	vm_rb;		
	vm_flags_t	vm_flags;	
	unsigned long	vm_start;	
	unsigned long	vm_end;		
	unsigned long	vm_top;		
	unsigned long	vm_pgoff;	
	struct file	*vm_file;	

	int		vm_usage;	
	bool		vm_icache_flushed : 1; 
};

struct vm_area_struct {
	

	unsigned long vm_start;		
	unsigned long vm_end;		

	
	struct vm_area_struct *vm_next, *vm_prev;

	struct rb_node vm_rb;

	unsigned long rb_subtree_gap;

	

	struct mm_struct *vm_mm;	
	pgprot_t vm_page_prot;		
	unsigned long vm_flags;		

	union {
		struct {
			struct rb_node rb;
			unsigned long rb_subtree_last;
		} linear;
		struct list_head nonlinear;
		const char __user *anon_name;
	} shared;

	struct list_head anon_vma_chain; 
	struct anon_vma *anon_vma;	

	
	const struct vm_operations_struct *vm_ops;

	
	unsigned long vm_pgoff;		
	struct file * vm_file;		
	void * vm_private_data;		

#ifndef CONFIG_MMU
	struct vm_region *vm_region;	
#endif
#ifdef CONFIG_NUMA
	struct mempolicy *vm_policy;	
#endif
};

struct core_thread {
	struct task_struct *task;
	struct core_thread *next;
};

struct core_state {
	atomic_t nr_threads;
	struct core_thread dumper;
	struct completion startup;
};

enum {
	MM_FILEPAGES,
	MM_ANONPAGES,
	MM_SWAPENTS,
	NR_MM_COUNTERS
};

#if USE_SPLIT_PTE_PTLOCKS && defined(CONFIG_MMU)
#define SPLIT_RSS_COUNTING
struct task_rss_stat {
	int events;	
	int count[NR_MM_COUNTERS];
};
#endif 

struct mm_rss_stat {
	atomic_long_t count[NR_MM_COUNTERS];
};

struct kioctx_table;
struct mm_struct {
	struct vm_area_struct *mmap;		
	struct rb_root mm_rb;
	u32 vmacache_seqnum;                   
#ifdef CONFIG_MMU
	unsigned long (*get_unmapped_area) (struct file *filp,
				unsigned long addr, unsigned long len,
				unsigned long pgoff, unsigned long flags);
#endif
	unsigned long mmap_base;		
	unsigned long mmap_legacy_base;         
	unsigned long task_size;		
	unsigned long highest_vm_end;		
	pgd_t * pgd;
	atomic_t mm_users;			
	atomic_t mm_count;			
	atomic_long_t nr_ptes;			
	int map_count;				

	spinlock_t page_table_lock;		
	struct rw_semaphore mmap_sem;

	struct list_head mmlist;		


	unsigned long hiwater_rss;	
	unsigned long hiwater_vm;	

	unsigned long total_vm;		
	unsigned long locked_vm;	
	unsigned long pinned_vm;	
	unsigned long shared_vm;	
	unsigned long exec_vm;		
	unsigned long stack_vm;		
	unsigned long def_flags;
	unsigned long start_code, end_code, start_data, end_data;
	unsigned long start_brk, brk, start_stack;
	unsigned long arg_start, arg_end, env_start, env_end;

	unsigned long saved_auxv[AT_VECTOR_SIZE]; 

	struct mm_rss_stat rss_stat;

	struct linux_binfmt *binfmt;

	cpumask_var_t cpu_vm_mask_var;

	
	mm_context_t context;

	unsigned long flags; 

	struct core_state *core_state; 
#ifdef CONFIG_AIO
	spinlock_t			ioctx_lock;
	struct kioctx_table __rcu	*ioctx_table;
#endif
#ifdef CONFIG_MEMCG
	struct task_struct __rcu *owner;
#endif

	
	struct file *exe_file;
#ifdef CONFIG_MMU_NOTIFIER
	struct mmu_notifier_mm *mmu_notifier_mm;
#endif
#if defined(CONFIG_TRANSPARENT_HUGEPAGE) && !USE_SPLIT_PMD_PTLOCKS
	pgtable_t pmd_huge_pte; 
#endif
#ifdef CONFIG_CPUMASK_OFFSTACK
	struct cpumask cpumask_allocation;
#endif
#ifdef CONFIG_NUMA_BALANCING
	unsigned long numa_next_scan;

	
	unsigned long numa_scan_offset;

	
	int numa_scan_seq;
#endif
#if defined(CONFIG_NUMA_BALANCING) || defined(CONFIG_COMPACTION)
	bool tlb_flush_pending;
#endif
	struct uprobes_state uprobes_state;
#ifdef CONFIG_MSM_APP_SETTINGS
	int app_setting;
#endif

};

static inline void mm_init_cpumask(struct mm_struct *mm)
{
#ifdef CONFIG_CPUMASK_OFFSTACK
	mm->cpu_vm_mask_var = &mm->cpumask_allocation;
#endif
	cpumask_clear(mm->cpu_vm_mask_var);
}

static inline cpumask_t *mm_cpumask(struct mm_struct *mm)
{
	return mm->cpu_vm_mask_var;
}

#if defined(CONFIG_NUMA_BALANCING) || defined(CONFIG_COMPACTION)
static inline bool mm_tlb_flush_pending(struct mm_struct *mm)
{
	barrier();
	return mm->tlb_flush_pending;
}
static inline void set_tlb_flush_pending(struct mm_struct *mm)
{
	mm->tlb_flush_pending = true;

	smp_mb__before_spinlock();
}
static inline void clear_tlb_flush_pending(struct mm_struct *mm)
{
	barrier();
	mm->tlb_flush_pending = false;
}
#else
static inline bool mm_tlb_flush_pending(struct mm_struct *mm)
{
	return false;
}
static inline void set_tlb_flush_pending(struct mm_struct *mm)
{
}
static inline void clear_tlb_flush_pending(struct mm_struct *mm)
{
}
#endif

struct vm_special_mapping
{
	const char *name;
	struct page **pages;
};

enum tlb_flush_reason {
	TLB_FLUSH_ON_TASK_SWITCH,
	TLB_REMOTE_SHOOTDOWN,
	TLB_LOCAL_SHOOTDOWN,
	TLB_LOCAL_MM_SHOOTDOWN,
	NR_TLB_FLUSH_REASONS,
};

static inline const char __user *vma_get_anon_name(struct vm_area_struct *vma)
{
	if (vma->vm_file)
		return NULL;

	return vma->shared.anon_name;
}

#endif 
