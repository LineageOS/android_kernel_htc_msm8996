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

#include <asm/pgtable.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/highmem.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/pagemap.h>
#include <linux/device.h>

#include "public/mc_user.h"

#include "mci/mcimcp.h"

#include "platform.h"	
#include "main.h"
#include "mcp.h"	
#include "mmu.h"

#define MMU_BUFFERABLE		BIT(2)		
#define MMU_CACHEABLE		BIT(3)		
#define MMU_EXT_NG		BIT(11)		

#define MMU_TYPE_PAGE		(3 << 0)
#define MMU_NS			BIT(5)
#define MMU_AP_RW_ALL		BIT(6) 
#define MMU_EXT_SHARED_64	(3 << 8)	
#define MMU_EXT_AF		BIT(10)		
#define MMU_EXT_XN		(((u64)1) << 54) 

#define MMU_TYPE_EXT		(3 << 0)	
#define MMU_TYPE_SMALL		(2 << 0)
#define MMU_EXT_AP0		BIT(4)
#define MMU_EXT_AP1		(2 << 4)
#define MMU_EXT_TEX(x)		((x) << 6)	
#define MMU_EXT_SHARED_32	BIT(10)		

#define L2_ENTRIES_MAX	256

#define L1_ENTRIES_MAX	512

union l1_table {
	u64		*pages_phys;	
	unsigned long	page;
};

union l2_table {
	union {				
		u32	*ptes_32;
		u64	*ptes_64;
	};
	unsigned long	page;
};

struct tee_mmu {
	union l2_table	l2_tables[L1_ENTRIES_MAX];	
	size_t		l2_tables_nr;	
	union l1_table	l1_table;	
	union l2_table	l1_l2_table;	
	u32		offset;
	u32		length;
	bool		user;		
};

static u64 pte_flags_64 = MMU_BUFFERABLE | MMU_CACHEABLE | MMU_EXT_NG |
#ifdef CONFIG_SMP
			  MMU_EXT_SHARED_64 |
#endif 
			  MMU_EXT_XN | MMU_EXT_AF | MMU_AP_RW_ALL |
			  MMU_NS | MMU_TYPE_PAGE;

static u32 pte_flags_32 = MMU_BUFFERABLE | MMU_CACHEABLE | MMU_EXT_NG |
#ifdef CONFIG_SMP
			  MMU_EXT_SHARED_32 | MMU_EXT_TEX(1) |
#endif 
			  MMU_EXT_AP1 | MMU_EXT_AP0 |
			  MMU_TYPE_SMALL | MMU_TYPE_EXT;

static uintptr_t mmu_table_pointer(const struct tee_mmu *mmu)
{
	if (mmu->l1_table.page) {
		return g_ctx.f_lpae ?
			(uintptr_t)mmu->l1_l2_table.ptes_64 :
			(uintptr_t)mmu->l1_l2_table.ptes_32;
	} else {
		return g_ctx.f_lpae ?
			(uintptr_t)mmu->l2_tables[0].ptes_64 :
			(uintptr_t)mmu->l2_tables[0].ptes_32;
	}
}

static void free_all_pages(struct tee_mmu *mmu_table)
{
	union l2_table *l2_table = &mmu_table->l2_tables[0];
	size_t i;

	for (i = 0; i < mmu_table->l2_tables_nr; i++, l2_table++) {
		if (!l2_table->page)
			break;

		free_page(l2_table->page);
	}

	if (mmu_table->l1_l2_table.page)
		free_page(mmu_table->l1_l2_table.page);

	if (mmu_table->l1_table.page)
		free_page(mmu_table->l1_table.page);
}

static inline int map_buffer(struct task_struct *task, const void *data,
			     unsigned int length, struct tee_mmu *mmu_table)
{
	const void      *reader = (const void *)((uintptr_t)data & PAGE_MASK);
	struct page	**pages;	
	unsigned long	pages_page;	
	size_t		chunk;
	unsigned long	total_pages_nr;
	int		l1_entries_max;
	int		ret = 0;

	
	mmu_table->length = length;
	mmu_table->offset = (uintptr_t)data & ~PAGE_MASK;
	total_pages_nr = PAGE_ALIGN(mmu_table->offset + length) / PAGE_SIZE;
	if (g_ctx.f_mem_ext)
		l1_entries_max = L1_ENTRIES_MAX;
	 else
		l1_entries_max = 1;

	if (total_pages_nr > (l1_entries_max * L2_ENTRIES_MAX)) {
		mc_dev_err("data mapping exceeds %d pages",
			   l1_entries_max * L2_ENTRIES_MAX);
		return -EINVAL;
	}

	
	mmu_table->l2_tables_nr = (total_pages_nr + L2_ENTRIES_MAX - 1) /
				  L2_ENTRIES_MAX;
	mc_dev_devel("total_pages_nr %lu l2_tables_nr %zu",
		     total_pages_nr, mmu_table->l2_tables_nr);

	
	pages_page = get_zeroed_page(GFP_KERNEL);
	if (!pages_page)
		return -ENOMEM;

	pages = (struct page **)pages_page;

	
	if (mmu_table->l2_tables_nr > 1) {
		mmu_table->l1_table.page = get_zeroed_page(GFP_KERNEL);
		mmu_table->l1_l2_table.page = get_zeroed_page(GFP_KERNEL);
		if (!mmu_table->l1_table.page || !mmu_table->l1_l2_table.page) {
			ret = -ENOMEM;
			goto end;
		}

		
		if (g_ctx.f_lpae) {
			u64 *pte;

			pte = &mmu_table->l1_l2_table.ptes_64[0];
			*pte = virt_to_phys(mmu_table->l1_table.pages_phys);
			*pte |= pte_flags_64;
		} else {
			u32 *pte;

			pte = &mmu_table->l1_l2_table.ptes_32[0];
			*pte = virt_to_phys(mmu_table->l1_table.pages_phys);
			*pte |= pte_flags_32;
		}
	}

	for (chunk = 0; chunk < mmu_table->l2_tables_nr; chunk++) {
		unsigned long pages_nr, i;
		struct page **page_ptr;

		
		if (chunk == (mmu_table->l2_tables_nr - 1))
			pages_nr = ((total_pages_nr - 1) % L2_ENTRIES_MAX) + 1;
		else
			pages_nr = L2_ENTRIES_MAX;

		
		mmu_table->l2_tables[chunk].page = get_zeroed_page(GFP_KERNEL);
		if (!mmu_table->l2_tables[chunk].page) {
			ret = -ENOMEM;
			goto end;
		}

		
		if (mmu_table->l1_table.page) {
			void *table;

			if (g_ctx.f_lpae)
				table = mmu_table->l2_tables[chunk].ptes_64;
			else
				table = mmu_table->l2_tables[chunk].ptes_32;

			mmu_table->l1_table.pages_phys[chunk] =
				virt_to_phys(table);
		}

		
		if (task) {
			long gup_ret;

			
			down_read(&task->mm->mmap_sem);
			gup_ret = get_user_pages(task, task->mm,
						 (uintptr_t)reader, pages_nr,
						 1, 0, pages, 0);
			up_read(&task->mm->mmap_sem);
			if (gup_ret < 0) {
				ret = gup_ret;
				mc_dev_err("failed to get user pages @%p: %d",
					   reader, ret);
				goto end;
			}

			
			if (gup_ret != pages_nr) {
				mc_dev_err("get_user_pages() failed, ret: %ld",
					   gup_ret);
				release_pages(pages, gup_ret, 0);
				ret = -ENOMEM;
				goto end;
			}

			reader += pages_nr * PAGE_SIZE;
			mmu_table->user = true;
		} else if (is_vmalloc_addr(data)) {
			
			page_ptr = &pages[0];
			for (i = 0; i < pages_nr; i++) {
				struct page *page = vmalloc_to_page(reader);

				if (!page) {
					mc_dev_err("failed to map address");
					ret = -EINVAL;
					goto end;
				}

				*page_ptr++ = page;
				reader += PAGE_SIZE;
			}
		} else {
			
			struct page *page = virt_to_page(reader);

			reader += pages_nr * PAGE_SIZE;
			page_ptr = &pages[0];
			for (i = 0; i < pages_nr; i++)
				*page_ptr++ = page++;
		}

		
		page_ptr = &pages[0];

		if (g_ctx.f_lpae) {
			u64 *pte = &mmu_table->l2_tables[chunk].ptes_64[0];

			for (i = 0; i < pages_nr; i++, page_ptr++, pte++) {
				unsigned long phys = page_to_phys(*page_ptr);
				*pte = phys;
				*pte |= pte_flags_64;
			}
		} else {
			u32 *pte = &mmu_table->l2_tables[chunk].ptes_32[0];

			for (i = 0; i < pages_nr; i++, page_ptr++, pte++) {
				unsigned long phys = page_to_phys(*page_ptr);
#if defined CONFIG_ARM64
				if (phys & 0xffffffff00000000) {
					mc_dev_err("64-bit pointer: 0x%16lx",
						   phys);
					ret = -EFAULT;
					goto end;
				}
#endif
				*pte = phys;
				*pte |= pte_flags_32;
			}
		}
	}

end:
	if (ret)
		free_all_pages(mmu_table);

	free_page(pages_page);
	return ret;
}

static inline void unmap_buffer(struct tee_mmu *mmu_table)
{
	int t;

	mc_dev_devel("clear MMU table, virt %p", mmu_table);
	if (!mmu_table->user)
		goto end;

	
	for (t = 0; t < mmu_table->l2_tables_nr; t++) {
		if (g_ctx.f_lpae) {
			u64 *pte = mmu_table->l2_tables[t].ptes_64;
			int i;

			for (i = 0; i < L2_ENTRIES_MAX; i++, pte++) {
				
				if (!*pte)
					break;

				
				page_cache_release(pte_page(*pte));
			}
		} else {
			u32 *pte = mmu_table->l2_tables[t].ptes_32;
			int i;

			for (i = 0; i < L2_ENTRIES_MAX; i++, pte++) {
				
				if (!*pte)
					break;

				
				page_cache_release(pte_page(*pte));
			}
		}
	}

end:
	free_all_pages(mmu_table);
}

void tee_mmu_delete(struct tee_mmu *mmu)
{
	if (WARN(!mmu, "NULL mmu pointer given"))
		return;

	unmap_buffer(mmu);
	mc_dev_devel("freed mmu %p: %s len %u off %u L%d table %lx\n",
		     mmu, mmu->user ? "user" : "kernel", mmu->length,
		     mmu->offset, mmu->l1_table.page ? 1 : 2,
		     mmu_table_pointer(mmu));
	kfree(mmu);
	
	atomic_dec(&g_ctx.c_mmus);
}

struct tee_mmu *tee_mmu_create(struct task_struct *task, const void *addr,
			       unsigned int length)
{
	struct tee_mmu *mmu;
	int ret;

	
	if (WARN(!addr, "data address is NULL"))
		return ERR_PTR(-EINVAL);

	if (WARN(!length, "data length is 0"))
		return ERR_PTR(-EINVAL);

	
	mmu = kzalloc(sizeof(*mmu), GFP_KERNEL);
	if (!mmu)
		return ERR_PTR(-ENOMEM);

	
	atomic_inc(&g_ctx.c_mmus);
	
	ret = map_buffer(task, addr, length, mmu);
	if (ret) {
		kfree(mmu);
		
		atomic_dec(&g_ctx.c_mmus);
		return ERR_PTR(ret);
	}

	mc_dev_devel("created mmu %p: %s addr %p len %u off %u L%d table %lx\n",
		     mmu, mmu->user ? "user" : "kernel", addr, mmu->length,
		     mmu->offset, mmu->l1_table.page ? 1 : 2,
		     mmu_table_pointer(mmu));
	return mmu;
}

void tee_mmu_buffer(const struct tee_mmu *mmu, struct mcp_buffer_map *map)
{
	uintptr_t table = mmu_table_pointer(mmu);

	map->phys_addr = virt_to_phys((void *)table);
	map->secure_va = 0;
	map->offset = mmu->offset;
	map->length = mmu->length;
	if (mmu->l1_table.page)
		map->type = WSM_L1;
	else
		map->type = WSM_L2;
}

int tee_mmu_debug_structs(struct kasnprintf_buf *buf, const struct tee_mmu *mmu)
{
	return kasnprintf(buf,
			  "\t\t\tmmu %p: %s len %u off %u table %lx type L%d\n",
			  mmu, mmu->user ? "user" : "kernel", mmu->length,
			  mmu->offset, mmu_table_pointer(mmu),
			  mmu->l1_table.page ? 1 : 2);
}
