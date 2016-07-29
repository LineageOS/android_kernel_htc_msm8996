/* internal.h: mm/ internal definitions
 *
 * Copyright (C) 2004 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#ifndef __MM_INTERNAL_H
#define __MM_INTERNAL_H

#include <linux/fs.h>
#include <linux/mm.h>

#define FOREGROUND_RECLAIM_100MS		100
#define FOREGROUND_RECLAIM_250MS		250
#define FOREGROUND_RECLAIM_500MS		500
#define FOREGROUND_RECLAIM_1000MS		1000

void free_pgtables(struct mmu_gather *tlb, struct vm_area_struct *start_vma,
		unsigned long floor, unsigned long ceiling);

static inline void set_page_count(struct page *page, int v)
{
	atomic_set(&page->_count, v);
}

extern int __do_page_cache_readahead(struct address_space *mapping,
		struct file *filp, pgoff_t offset, unsigned long nr_to_read,
		unsigned long lookahead_size);

static inline unsigned long ra_submit(struct file_ra_state *ra,
		struct address_space *mapping, struct file *filp)
{
	return __do_page_cache_readahead(mapping, filp,
					ra->start, ra->size, ra->async_size);
}

static inline void set_page_refcounted(struct page *page)
{
	VM_BUG_ON_PAGE(PageTail(page), page);
	VM_BUG_ON_PAGE(atomic_read(&page->_count), page);
	set_page_count(page, 1);
}

static inline void __get_page_tail_foll(struct page *page,
					bool get_page_head)
{
	VM_BUG_ON_PAGE(atomic_read(&page->first_page->_count) <= 0, page);
	if (get_page_head)
		atomic_inc(&page->first_page->_count);
	get_huge_page_tail(page);
}

static inline void get_page_foll(struct page *page)
{
	if (unlikely(PageTail(page)))
		__get_page_tail_foll(page, true);
	else {
		VM_BUG_ON_PAGE(atomic_read(&page->_count) <= 0, page);
		atomic_inc(&page->_count);
	}
}

extern unsigned long highest_memmap_pfn;

extern int isolate_lru_page(struct page *page);
extern void putback_lru_page(struct page *page);
extern bool zone_reclaimable(struct zone *zone);

extern pmd_t *mm_find_pmd(struct mm_struct *mm, unsigned long address);


static inline unsigned long
__find_buddy_index(unsigned long page_idx, unsigned int order)
{
	return page_idx ^ (1 << order);
}

extern int __isolate_free_page(struct page *page, unsigned int order);
extern void __free_pages_bootmem(struct page *page, unsigned int order);
extern void prep_compound_page(struct page *page, unsigned long order);
#ifdef CONFIG_MEMORY_FAILURE
extern bool is_free_buddy_page(struct page *page);
#endif
extern int user_min_free_kbytes;

#if defined CONFIG_COMPACTION || defined CONFIG_CMA

struct compact_control {
	struct list_head freepages;	
	struct list_head migratepages;	
	unsigned long nr_freepages;	
	unsigned long nr_migratepages;	
	unsigned long free_pfn;		
	unsigned long migrate_pfn;	
	enum migrate_mode mode;		
	bool ignore_skip_hint;		
	bool finished_update_free;	
	bool finished_update_migrate;

	int order;			
	const gfp_t gfp_mask;		
	const int alloc_flags;		
	const int classzone_idx;	
	struct zone *zone;
	int contended;			
};

unsigned long
isolate_freepages_range(struct compact_control *cc,
			unsigned long start_pfn, unsigned long end_pfn);
unsigned long
isolate_migratepages_range(struct compact_control *cc,
			   unsigned long low_pfn, unsigned long end_pfn);

#endif

static inline unsigned long page_order(struct page *page)
{
	
	return page_private(page);
}

#define page_order_unsafe(page)		ACCESS_ONCE(page_private(page))

static inline bool is_cow_mapping(vm_flags_t flags)
{
	return (flags & (VM_SHARED | VM_MAYWRITE)) == VM_MAYWRITE;
}

void __vma_link_list(struct mm_struct *mm, struct vm_area_struct *vma,
		struct vm_area_struct *prev, struct rb_node *rb_parent);

#ifdef CONFIG_MMU
extern long __mlock_vma_pages_range(struct vm_area_struct *vma,
		unsigned long start, unsigned long end, int *nonblocking);
extern void munlock_vma_pages_range(struct vm_area_struct *vma,
			unsigned long start, unsigned long end);
static inline void munlock_vma_pages_all(struct vm_area_struct *vma)
{
	munlock_vma_pages_range(vma, vma->vm_start, vma->vm_end);
}

extern void mlock_vma_page(struct page *page);
extern unsigned int munlock_vma_page(struct page *page);

extern void clear_page_mlock(struct page *page);

static inline void mlock_migrate_page(struct page *newpage, struct page *page)
{
	if (TestClearPageMlocked(page)) {
		unsigned long flags;
		int nr_pages = hpage_nr_pages(page);

		local_irq_save(flags);
		__mod_zone_page_state(page_zone(page), NR_MLOCK, -nr_pages);
		SetPageMlocked(newpage);
		__mod_zone_page_state(page_zone(newpage), NR_MLOCK, nr_pages);
		local_irq_restore(flags);
	}
}

extern pmd_t maybe_pmd_mkwrite(pmd_t pmd, struct vm_area_struct *vma);

extern unsigned long vma_address(struct page *page,
				 struct vm_area_struct *vma);
#else 
static inline void clear_page_mlock(struct page *page) { }
static inline void mlock_vma_page(struct page *page) { }
static inline void mlock_migrate_page(struct page *new, struct page *old) { }

#endif 

static inline struct page *mem_map_offset(struct page *base, int offset)
{
	if (unlikely(offset >= MAX_ORDER_NR_PAGES))
		return nth_page(base, offset);
	return base + offset;
}

static inline struct page *mem_map_next(struct page *iter,
						struct page *base, int offset)
{
	if (unlikely((offset & (MAX_ORDER_NR_PAGES - 1)) == 0)) {
		unsigned long pfn = page_to_pfn(base) + offset;
		if (!pfn_valid(pfn))
			return NULL;
		return pfn_to_page(pfn);
	}
	return iter + 1;
}

#ifdef CONFIG_SPARSEMEM
#define __paginginit __meminit
#else
#define __paginginit __init
#endif

enum mminit_level {
	MMINIT_WARNING,
	MMINIT_VERIFY,
	MMINIT_TRACE
};

#ifdef CONFIG_DEBUG_MEMORY_INIT

extern int mminit_loglevel;

#define mminit_dprintk(level, prefix, fmt, arg...) \
do { \
	if (level < mminit_loglevel) { \
		printk(level <= MMINIT_WARNING ? KERN_WARNING : KERN_DEBUG); \
		printk(KERN_CONT "mminit::" prefix " " fmt, ##arg); \
	} \
} while (0)

extern void mminit_verify_pageflags_layout(void);
extern void mminit_verify_page_links(struct page *page,
		enum zone_type zone, unsigned long nid, unsigned long pfn);
extern void mminit_verify_zonelist(void);

#else

static inline void mminit_dprintk(enum mminit_level level,
				const char *prefix, const char *fmt, ...)
{
}

static inline void mminit_verify_pageflags_layout(void)
{
}

static inline void mminit_verify_page_links(struct page *page,
		enum zone_type zone, unsigned long nid, unsigned long pfn)
{
}

static inline void mminit_verify_zonelist(void)
{
}
#endif 

#if defined(CONFIG_SPARSEMEM)
extern void mminit_validate_memmodel_limits(unsigned long *start_pfn,
				unsigned long *end_pfn);
#else
static inline void mminit_validate_memmodel_limits(unsigned long *start_pfn,
				unsigned long *end_pfn)
{
}
#endif 

#define ZONE_RECLAIM_NOSCAN	-2
#define ZONE_RECLAIM_FULL	-1
#define ZONE_RECLAIM_SOME	0
#define ZONE_RECLAIM_SUCCESS	1

extern int hwpoison_filter(struct page *p);

extern u32 hwpoison_filter_dev_major;
extern u32 hwpoison_filter_dev_minor;
extern u64 hwpoison_filter_flags_mask;
extern u64 hwpoison_filter_flags_value;
extern u64 hwpoison_filter_memcg;
extern u32 hwpoison_filter_enable;

extern unsigned long vm_mmap_pgoff(struct file *, unsigned long,
        unsigned long, unsigned long,
        unsigned long, unsigned long);

extern void set_pageblock_order(void);
unsigned long reclaim_clean_pages_from_list(struct zone *zone,
					    struct list_head *page_list);
#define ALLOC_WMARK_MIN		WMARK_MIN
#define ALLOC_WMARK_LOW		WMARK_LOW
#define ALLOC_WMARK_HIGH	WMARK_HIGH
#define ALLOC_NO_WATERMARKS	0x04 

#define ALLOC_WMARK_MASK	(ALLOC_NO_WATERMARKS-1)

#define ALLOC_HARDER		0x10 
#define ALLOC_HIGH		0x20 
#define ALLOC_CPUSET		0x40 
#define ALLOC_CMA		0x80 
#define ALLOC_FAIR		0x100 

#endif	
