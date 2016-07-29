#ifndef _LINUX_SWAP_H
#define _LINUX_SWAP_H

#include <linux/spinlock.h>
#include <linux/linkage.h>
#include <linux/mmzone.h>
#include <linux/list.h>
#include <linux/memcontrol.h>
#include <linux/sched.h>
#include <linux/node.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/page-flags.h>
#include <asm/page.h>

struct notifier_block;

struct bio;

#define SWAP_FLAG_PREFER	0x8000	
#define SWAP_FLAG_PRIO_MASK	0x7fff
#define SWAP_FLAG_PRIO_SHIFT	0
#define SWAP_FLAG_DISCARD	0x10000 
#define SWAP_FLAG_DISCARD_ONCE	0x20000 
#define SWAP_FLAG_DISCARD_PAGES 0x40000 

#define SWAP_FLAGS_VALID	(SWAP_FLAG_PRIO_MASK | SWAP_FLAG_PREFER | \
				 SWAP_FLAG_DISCARD | SWAP_FLAG_DISCARD_ONCE | \
				 SWAP_FLAG_DISCARD_PAGES)

static inline int current_is_kswapd(void)
{
	return current->flags & PF_KSWAPD;
}

#define MAX_SWAPFILES_SHIFT	5


#ifdef CONFIG_MIGRATION
#define SWP_MIGRATION_NUM 2
#define SWP_MIGRATION_READ	(MAX_SWAPFILES + SWP_HWPOISON_NUM)
#define SWP_MIGRATION_WRITE	(MAX_SWAPFILES + SWP_HWPOISON_NUM + 1)
#else
#define SWP_MIGRATION_NUM 0
#endif

#ifdef CONFIG_MEMORY_FAILURE
#define SWP_HWPOISON_NUM 1
#define SWP_HWPOISON		MAX_SWAPFILES
#else
#define SWP_HWPOISON_NUM 0
#endif

#define MAX_SWAPFILES \
	((1 << MAX_SWAPFILES_SHIFT) - SWP_MIGRATION_NUM - SWP_HWPOISON_NUM)

union swap_header {
	struct {
		char reserved[PAGE_SIZE - 10];
		char magic[10];			
	} magic;
	struct {
		char		bootbits[1024];	
		__u32		version;
		__u32		last_page;
		__u32		nr_badpages;
		unsigned char	sws_uuid[16];
		unsigned char	sws_volume[16];
		__u32		padding[117];
		__u32		badpages[1];
	} info;
};

typedef struct {
	unsigned long val;
} swp_entry_t;

struct reclaim_state {
	unsigned long reclaimed_slab;
	int trigger_lmk;
};

#ifdef __KERNEL__

struct address_space;
struct sysinfo;
struct writeback_control;
struct zone;

struct swap_extent {
	struct list_head list;
	pgoff_t start_page;
	pgoff_t nr_pages;
	sector_t start_block;
};

#define __swapoffset(x) ((unsigned long)&((union swap_header *)0)->x)
#define MAX_SWAP_BADPAGES \
	((__swapoffset(magic.magic) - __swapoffset(info.badpages)) / sizeof(int))

enum {
	SWP_USED	= (1 << 0),	
	SWP_WRITEOK	= (1 << 1),	
	SWP_DISCARDABLE = (1 << 2),	
	SWP_DISCARDING	= (1 << 3),	
	SWP_SOLIDSTATE	= (1 << 4),	
	SWP_CONTINUED	= (1 << 5),	
	SWP_BLKDEV	= (1 << 6),	
	SWP_FILE	= (1 << 7),	
	SWP_AREA_DISCARD = (1 << 8),	
	SWP_PAGE_DISCARD = (1 << 9),	
					
	SWP_FAST	= (1 << 10),	
	SWP_SCANNING	= (1 << 11),	
};

#define SWAP_CLUSTER_MAX 32UL
#define COMPACT_CLUSTER_MAX SWAP_CLUSTER_MAX

#define KSWAPD_ZONE_BALANCE_GAP_RATIO 100

#define SWAP_MAP_MAX	0x3e	
#define SWAP_MAP_BAD	0x3f	
#define SWAP_HAS_CACHE	0x40	
#define SWAP_CONT_MAX	0x7f	
#define COUNT_CONTINUED	0x80	
#define SWAP_MAP_SHMEM	0xbf	

struct swap_cluster_info {
	unsigned int data:24;
	unsigned int flags:8;
};
#define CLUSTER_FLAG_FREE 1 
#define CLUSTER_FLAG_NEXT_NULL 2 

struct percpu_cluster {
	struct swap_cluster_info index; 
	unsigned int next; 
};

struct swap_info_struct {
	unsigned long	flags;		
	signed short	prio;		
	struct plist_node list;		
	struct plist_node avail_list;	
	signed char	type;		
	unsigned int	max;		
	unsigned char *swap_map;	
	struct swap_cluster_info *cluster_info; 
	struct swap_cluster_info free_cluster_head; 
	struct swap_cluster_info free_cluster_tail; 
	unsigned int lowest_bit;	
	unsigned int highest_bit;	
	unsigned int pages;		
	unsigned int inuse_pages;	
	unsigned int cluster_next;	
	unsigned int cluster_nr;	
	struct percpu_cluster __percpu *percpu_cluster; 
	struct swap_extent *curr_swap_extent;
	struct swap_extent first_swap_extent;
	struct block_device *bdev;	
	struct file *swap_file;		
	unsigned int old_block_size;	
#ifdef CONFIG_FRONTSWAP
	unsigned long *frontswap_map;	
	atomic_t frontswap_pages;	
#endif
	spinlock_t lock;		
	struct work_struct discard_work; 
	struct swap_cluster_info discard_cluster_head; 
	struct swap_cluster_info discard_cluster_tail; 
};

void *workingset_eviction(struct address_space *mapping, struct page *page);
bool workingset_refault(void *shadow);
void workingset_activation(struct page *page);
extern struct list_lru workingset_shadow_nodes;

static inline unsigned int workingset_node_pages(struct radix_tree_node *node)
{
	return node->count & RADIX_TREE_COUNT_MASK;
}

static inline void workingset_node_pages_inc(struct radix_tree_node *node)
{
	node->count++;
}

static inline void workingset_node_pages_dec(struct radix_tree_node *node)
{
	node->count--;
}

static inline unsigned int workingset_node_shadows(struct radix_tree_node *node)
{
	return node->count >> RADIX_TREE_COUNT_SHIFT;
}

static inline void workingset_node_shadows_inc(struct radix_tree_node *node)
{
	node->count += 1U << RADIX_TREE_COUNT_SHIFT;
}

static inline void workingset_node_shadows_dec(struct radix_tree_node *node)
{
	node->count -= 1U << RADIX_TREE_COUNT_SHIFT;
}

extern unsigned long totalram_pages;
extern unsigned long totalreserve_pages;
extern unsigned long dirty_balance_reserve;
extern unsigned long nr_free_buffer_pages(void);
extern unsigned long nr_free_pagecache_pages(void);

#define nr_free_pages() global_page_state(NR_FREE_PAGES)


extern void lru_cache_add(struct page *);
extern void lru_cache_add_anon(struct page *page);
extern void lru_cache_add_file(struct page *page);
extern void lru_add_page_tail(struct page *page, struct page *page_tail,
			 struct lruvec *lruvec, struct list_head *head);
extern void activate_page(struct page *);
extern void mark_page_accessed(struct page *);
extern void lru_add_drain(void);
extern void lru_add_drain_cpu(int cpu);
extern void lru_add_drain_all(void);
extern void rotate_reclaimable_page(struct page *page);
extern void deactivate_page(struct page *page);
extern void swap_setup(void);

extern void add_page_to_unevictable_list(struct page *page);

extern void lru_cache_add_active_or_unevictable(struct page *page,
						struct vm_area_struct *vma);

extern unsigned long try_to_free_pages(struct zonelist *zonelist, int order,
					gfp_t gfp_mask, nodemask_t *mask);
extern int __isolate_lru_page(struct page *page, isolate_mode_t mode);
extern unsigned long try_to_free_mem_cgroup_pages(struct mem_cgroup *memcg,
						  unsigned long nr_pages,
						  gfp_t gfp_mask,
						  bool may_swap);
extern unsigned long mem_cgroup_shrink_node_zone(struct mem_cgroup *mem,
						gfp_t gfp_mask, bool noswap,
						struct zone *zone,
						unsigned long *nr_scanned);
extern unsigned long shrink_all_memory(unsigned long nr_pages);
extern int vm_swappiness;
extern int remove_mapping(struct address_space *mapping, struct page *page);
extern unsigned long vm_total_pages;

#ifdef CONFIG_NUMA
extern int zone_reclaim_mode;
extern int sysctl_min_unmapped_ratio;
extern int sysctl_min_slab_ratio;
extern int zone_reclaim(struct zone *, gfp_t, unsigned int);
#else
#define zone_reclaim_mode 0
static inline int zone_reclaim(struct zone *z, gfp_t mask, unsigned int order)
{
	return 0;
}
#endif

extern int page_evictable(struct page *page);
extern void check_move_unevictable_pages(struct page **, int nr_pages);

extern int kswapd_run(int nid);
extern void kswapd_stop(int nid);
#ifdef CONFIG_MEMCG
extern int mem_cgroup_swappiness(struct mem_cgroup *mem);
#else
static inline int mem_cgroup_swappiness(struct mem_cgroup *mem)
{
	return vm_swappiness;
}
#endif
#ifdef CONFIG_MEMCG_SWAP
extern void mem_cgroup_swapout(struct page *page, swp_entry_t entry);
extern void mem_cgroup_uncharge_swap(swp_entry_t entry);
#else
static inline void mem_cgroup_swapout(struct page *page, swp_entry_t entry)
{
}
static inline void mem_cgroup_uncharge_swap(swp_entry_t entry)
{
}
#endif
#ifdef CONFIG_SWAP
extern int swap_readpage(struct page *);
extern int swap_writepage(struct page *page, struct writeback_control *wbc);
extern void end_swap_bio_write(struct bio *bio, int err);
extern int __swap_writepage(struct page *page, struct writeback_control *wbc,
	void (*end_write_func)(struct bio *, int));
extern int swap_set_page_dirty(struct page *page);
extern void end_swap_bio_read(struct bio *bio, int err);

int add_swap_extent(struct swap_info_struct *sis, unsigned long start_page,
		unsigned long nr_pages, sector_t start_block);
int generic_swapfile_activate(struct swap_info_struct *, struct file *,
		sector_t *);

extern struct address_space swapper_spaces[];
#define swap_address_space(entry) (&swapper_spaces[swp_type(entry)])
extern unsigned long total_swapcache_pages(void);
extern void show_swap_cache_info(void);
extern int add_to_swap(struct page *, struct list_head *list);
extern int add_to_swap_cache(struct page *, swp_entry_t, gfp_t);
extern int __add_to_swap_cache(struct page *page, swp_entry_t entry);
extern void __delete_from_swap_cache(struct page *);
extern void delete_from_swap_cache(struct page *);
extern void free_page_and_swap_cache(struct page *);
extern void free_pages_and_swap_cache(struct page **, int);
extern struct page *lookup_swap_cache(swp_entry_t);
extern struct page *read_swap_cache_async(swp_entry_t, gfp_t,
			struct vm_area_struct *vma, unsigned long addr);
extern struct page *swapin_readahead(swp_entry_t, gfp_t,
			struct vm_area_struct *vma, unsigned long addr);

extern atomic_long_t nr_swap_pages;
extern long total_swap_pages;
extern bool is_swap_fast(swp_entry_t entry);

static inline bool vm_swap_full(struct swap_info_struct *si)
{
	if (si->flags & SWP_FAST)
		return true;

	return atomic_long_read(&nr_swap_pages) * 2 < total_swap_pages;
}

static inline long get_nr_swap_pages(void)
{
	return atomic_long_read(&nr_swap_pages);
}

extern void si_swapinfo(struct sysinfo *);
extern swp_entry_t get_swap_page(void);
extern swp_entry_t get_swap_page_of_type(int);
extern int add_swap_count_continuation(swp_entry_t, gfp_t);
extern void swap_shmem_alloc(swp_entry_t);
extern int swap_duplicate(swp_entry_t);
extern int swapcache_prepare(swp_entry_t);
extern void swap_free(swp_entry_t);
extern void swapcache_free(swp_entry_t);
extern int free_swap_and_cache(swp_entry_t);
extern int swap_type_of(dev_t, sector_t, struct block_device **);
extern unsigned int count_swap_pages(int, int);
extern sector_t map_swap_page(struct page *, struct block_device **);
extern sector_t swapdev_block(int, pgoff_t);
extern int page_swapcount(struct page *);
extern struct swap_info_struct *page_swap_info(struct page *);
extern int reuse_swap_page(struct page *);
extern int try_to_free_swap(struct page *);
struct backing_dev_info;

#ifdef CONFIG_MEMCG
extern void
mem_cgroup_uncharge_swapcache(struct page *page, swp_entry_t ent, bool swapout);
#else
static inline void
mem_cgroup_uncharge_swapcache(struct page *page, swp_entry_t ent, bool swapout)
{
}
#endif

#else 

#define swap_address_space(entry)		(NULL)
#define get_nr_swap_pages()			0L
#define total_swap_pages			0L
#define total_swapcache_pages()			0UL
#define vm_swap_full(si)			0

#define si_swapinfo(val) \
	do { (val)->freeswap = (val)->totalswap = 0; } while (0)
#define free_page_and_swap_cache(page) \
	page_cache_release(page)
#define free_pages_and_swap_cache(pages, nr) \
	release_pages((pages), (nr), false);

static inline void show_swap_cache_info(void)
{
}

#define free_swap_and_cache(swp)	is_migration_entry(swp)
#define swapcache_prepare(swp)		is_migration_entry(swp)

static inline int add_swap_count_continuation(swp_entry_t swp, gfp_t gfp_mask)
{
	return 0;
}

static inline void swap_shmem_alloc(swp_entry_t swp)
{
}

static inline int swap_duplicate(swp_entry_t swp)
{
	return 0;
}

static inline void swap_free(swp_entry_t swp)
{
}

static inline void swapcache_free(swp_entry_t swp)
{
}

static inline struct page *swapin_readahead(swp_entry_t swp, gfp_t gfp_mask,
			struct vm_area_struct *vma, unsigned long addr)
{
	return NULL;
}

static inline int swap_writepage(struct page *p, struct writeback_control *wbc)
{
	return 0;
}

static inline struct page *lookup_swap_cache(swp_entry_t swp)
{
	return NULL;
}

static inline int add_to_swap(struct page *page, struct list_head *list)
{
	return 0;
}

static inline int add_to_swap_cache(struct page *page, swp_entry_t entry,
							gfp_t gfp_mask)
{
	return -1;
}

static inline void __delete_from_swap_cache(struct page *page)
{
}

static inline void delete_from_swap_cache(struct page *page)
{
}

static inline int page_swapcount(struct page *page)
{
	return 0;
}

#define reuse_swap_page(page)	(page_mapcount(page) == 1)

static inline int try_to_free_swap(struct page *page)
{
	return 0;
}

static inline swp_entry_t get_swap_page(void)
{
	swp_entry_t entry;
	entry.val = 0;
	return entry;
}

static inline void
mem_cgroup_uncharge_swapcache(struct page *page, swp_entry_t ent)
{
}

#endif 
#endif 
#endif 
