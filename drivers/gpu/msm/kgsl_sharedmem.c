/* Copyright (c) 2002,2007-2015, The Linux Foundation. All rights reserved.
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

#include <linux/export.h>
#include <linux/vmalloc.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/kmemleak.h>
#include <linux/highmem.h>
#include <linux/scatterlist.h>
#include <linux/msm_kgsl.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/secure_buffer.h>

#include "kgsl.h"
#include "kgsl_sharedmem.h"
#include "kgsl_cffdump.h"
#include "kgsl_device.h"
#include "kgsl_log.h"
#include "kgsl_mmu.h"


static bool sharedmem_noretry_flag;

static DEFINE_MUTEX(kernel_map_global_lock);

struct cp2_mem_chunks {
	unsigned int chunk_list;
	unsigned int chunk_list_size;
	unsigned int chunk_size;
} __attribute__ ((__packed__));

struct cp2_lock_req {
	struct cp2_mem_chunks chunks;
	unsigned int mem_usage;
	unsigned int lock;
} __attribute__ ((__packed__));

#define MEM_PROTECT_LOCK_ID2		0x0A
#define MEM_PROTECT_LOCK_ID2_FLAT	0x11

struct kgsl_mem_entry_attribute {
	struct attribute attr;
	int memtype;
	ssize_t (*show)(struct kgsl_process_private *priv,
		int type, char *buf);
};

#define to_mem_entry_attr(a) \
container_of(a, struct kgsl_mem_entry_attribute, attr)

#define __MEM_ENTRY_ATTR(_type, _name, _show) \
{ \
	.attr = { .name = __stringify(_name), .mode = 0444 }, \
	.memtype = _type, \
	.show = _show, \
}


struct mem_entry_stats {
	int memtype;
	struct kgsl_mem_entry_attribute attr;
	struct kgsl_mem_entry_attribute max_attr;
};


#define MEM_ENTRY_STAT(_type, _name) \
{ \
	.memtype = _type, \
	.attr = __MEM_ENTRY_ATTR(_type, _name, mem_entry_show), \
	.max_attr = __MEM_ENTRY_ATTR(_type, _name##_max, \
		mem_entry_max_show), \
}

static void kgsl_cma_unlock_secure(struct kgsl_memdesc *memdesc);


static ssize_t
mem_entry_show(struct kgsl_process_private *priv, int type, char *buf)
{
	uint64_t cur = atomic_long_read(&priv->stats[type].cur);
	return snprintf(buf, PAGE_SIZE, "%llu\n", cur);
}


static ssize_t
mem_entry_max_show(struct kgsl_process_private *priv, int type, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%llu\n", priv->stats[type].max);
}

static ssize_t mem_entry_sysfs_show(struct kobject *kobj,
	struct attribute *attr, char *buf)
{
	struct kgsl_mem_entry_attribute *pattr = to_mem_entry_attr(attr);
	struct kgsl_process_private *priv;
	ssize_t ret;

	priv = kobj ? container_of(kobj, struct kgsl_process_private, kobj) :
			NULL;

	if (priv && pattr->show)
		ret = pattr->show(priv, pattr->memtype, buf);
	else
		ret = -EIO;

	return ret;
}

static const struct sysfs_ops mem_entry_sysfs_ops = {
	.show = mem_entry_sysfs_show,
};

static struct kobj_type ktype_mem_entry = {
	.sysfs_ops = &mem_entry_sysfs_ops,
};

static struct mem_entry_stats mem_stats[] = {
	MEM_ENTRY_STAT(KGSL_MEM_ENTRY_KERNEL, kernel),
	MEM_ENTRY_STAT(KGSL_MEM_ENTRY_USER, user),
#ifdef CONFIG_ION
	MEM_ENTRY_STAT(KGSL_MEM_ENTRY_ION, ion),
#endif
};

void
kgsl_process_uninit_sysfs(struct kgsl_process_private *private)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mem_stats); i++) {
		sysfs_remove_file(&private->kobj, &mem_stats[i].attr.attr);
		sysfs_remove_file(&private->kobj,
			&mem_stats[i].max_attr.attr);
	}

	kobject_put(&private->kobj);
	
	kgsl_process_private_put(private);
}

void kgsl_process_init_sysfs(struct kgsl_device *device,
		struct kgsl_process_private *private)
{
	unsigned char name[16];
	int i;

	
	kgsl_process_private_get(private);

	snprintf(name, sizeof(name), "%d", private->pid);

	if (kobject_init_and_add(&private->kobj, &ktype_mem_entry,
		kgsl_driver.prockobj, name)) {
		WARN(1, "Unable to add sysfs dir '%s'\n", name);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(mem_stats); i++) {
		if (sysfs_create_file(&private->kobj,
			&mem_stats[i].attr.attr))
			WARN(1, "Couldn't create sysfs file '%s'\n",
				mem_stats[i].attr.attr.name);

		if (sysfs_create_file(&private->kobj,
			&mem_stats[i].max_attr.attr))
			WARN(1, "Couldn't create sysfs file '%s'\n",
				mem_stats[i].max_attr.attr.name);

	}
}

static ssize_t kgsl_drv_memstat_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	uint64_t val = 0;

	if (!strcmp(attr->attr.name, "vmalloc"))
		val = atomic_long_read(&kgsl_driver.stats.vmalloc);
	else if (!strcmp(attr->attr.name, "vmalloc_max"))
		val = atomic_long_read(&kgsl_driver.stats.vmalloc_max);
	else if (!strcmp(attr->attr.name, "page_alloc"))
		val = atomic_long_read(&kgsl_driver.stats.page_alloc);
	else if (!strcmp(attr->attr.name, "page_alloc_max"))
		val = atomic_long_read(&kgsl_driver.stats.page_alloc_max);
	else if (!strcmp(attr->attr.name, "coherent"))
		val = atomic_long_read(&kgsl_driver.stats.coherent);
	else if (!strcmp(attr->attr.name, "coherent_max"))
		val = atomic_long_read(&kgsl_driver.stats.coherent_max);
	else if (!strcmp(attr->attr.name, "secure"))
		val = atomic_long_read(&kgsl_driver.stats.secure);
	else if (!strcmp(attr->attr.name, "secure_max"))
		val = atomic_long_read(&kgsl_driver.stats.secure_max);
	else if (!strcmp(attr->attr.name, "mapped"))
		val = atomic_long_read(&kgsl_driver.stats.mapped);
	else if (!strcmp(attr->attr.name, "mapped_max"))
		val = atomic_long_read(&kgsl_driver.stats.mapped_max);

	return snprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t kgsl_drv_full_cache_threshold_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret;
	unsigned int thresh = 0;

	ret = kgsl_sysfs_store(buf, &thresh);
	if (ret)
		return ret;

	kgsl_driver.full_cache_threshold = thresh;
	return count;
}

static ssize_t kgsl_drv_full_cache_threshold_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			kgsl_driver.full_cache_threshold);
}

static ssize_t kgsl_alloc_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			kgsl_get_alloc_size(true));
}


static DEVICE_ATTR(vmalloc, 0444, kgsl_drv_memstat_show, NULL);
static DEVICE_ATTR(vmalloc_max, 0444, kgsl_drv_memstat_show, NULL);
static DEVICE_ATTR(page_alloc, 0444, kgsl_drv_memstat_show, NULL);
static DEVICE_ATTR(page_alloc_max, 0444, kgsl_drv_memstat_show, NULL);
static DEVICE_ATTR(coherent, 0444, kgsl_drv_memstat_show, NULL);
static DEVICE_ATTR(coherent_max, 0444, kgsl_drv_memstat_show, NULL);
static DEVICE_ATTR(secure, 0444, kgsl_drv_memstat_show, NULL);
static DEVICE_ATTR(secure_max, 0444, kgsl_drv_memstat_show, NULL);
static DEVICE_ATTR(mapped, 0444, kgsl_drv_memstat_show, NULL);
static DEVICE_ATTR(mapped_max, 0444, kgsl_drv_memstat_show, NULL);
static DEVICE_ATTR(full_cache_threshold, 0644,
		kgsl_drv_full_cache_threshold_show,
		kgsl_drv_full_cache_threshold_store);
static DEVICE_ATTR(kgsl_alloc, 0444, kgsl_alloc_show, NULL);

static const struct device_attribute *drv_attr_list[] = {
	&dev_attr_vmalloc,
	&dev_attr_vmalloc_max,
	&dev_attr_page_alloc,
	&dev_attr_page_alloc_max,
	&dev_attr_coherent,
	&dev_attr_coherent_max,
	&dev_attr_secure,
	&dev_attr_secure_max,
	&dev_attr_mapped,
	&dev_attr_mapped_max,
	&dev_attr_full_cache_threshold,
	&dev_attr_kgsl_alloc,
	NULL
};

void
kgsl_sharedmem_uninit_sysfs(void)
{
	kgsl_remove_device_sysfs_files(&kgsl_driver.virtdev, drv_attr_list);
}

int
kgsl_sharedmem_init_sysfs(void)
{
	return kgsl_create_device_sysfs_files(&kgsl_driver.virtdev,
		drv_attr_list);
}

static int kgsl_sharedmem_page_alloc_user(struct kgsl_memdesc *memdesc,
				struct kgsl_pagetable *pagetable,
				uint64_t size);

static int kgsl_cma_alloc_secure(struct kgsl_device *device,
			struct kgsl_memdesc *memdesc, uint64_t size);

static int kgsl_allocate_secure(struct kgsl_device *device,
				struct kgsl_memdesc *memdesc,
				struct kgsl_pagetable *pagetable,
				uint64_t size) {
	int ret;

	if (MMU_FEATURE(&device->mmu, KGSL_MMU_HYP_SECURE_ALLOC))
		ret = kgsl_sharedmem_page_alloc_user(memdesc, pagetable, size);
	else
		ret = kgsl_cma_alloc_secure(device, memdesc, size);

	return ret;
}

int kgsl_allocate_user(struct kgsl_device *device,
		struct kgsl_memdesc *memdesc,
		struct kgsl_pagetable *pagetable,
		uint64_t size, uint64_t flags)
{
	int ret;

	memdesc->flags = flags;

	if (kgsl_mmu_get_mmutype() == KGSL_MMU_TYPE_NONE)
		ret = kgsl_sharedmem_alloc_contig(device, memdesc,
				pagetable, size);
	else if (flags & KGSL_MEMFLAGS_SECURE)
		ret = kgsl_allocate_secure(device, memdesc, pagetable, size);
	else
		ret = kgsl_sharedmem_page_alloc_user(memdesc, pagetable, size);

	return ret;
}

static int kgsl_page_alloc_vmfault(struct kgsl_memdesc *memdesc,
				struct vm_area_struct *vma,
				struct vm_fault *vmf)
{
	int i, pgoff;
	struct scatterlist *s = memdesc->sgt->sgl;
	unsigned int offset;

	offset = ((unsigned long) vmf->virtual_address - vma->vm_start);

	if (offset >= memdesc->size)
		return VM_FAULT_SIGBUS;

	pgoff = offset >> PAGE_SHIFT;


	for (i = 0; i < memdesc->sgt->nents; i++) {
		int npages = s->length >> PAGE_SHIFT;

		if (pgoff < npages) {
			struct page *page = sg_page(s);

			page = nth_page(page, pgoff);

			get_page(page);
			vmf->page = page;

			return 0;
		}

		pgoff -= npages;
		s = sg_next(s);
	}

	return VM_FAULT_SIGBUS;
}

static void kgsl_page_alloc_unmap_kernel(struct kgsl_memdesc *memdesc)
{
	mutex_lock(&kernel_map_global_lock);
	if (!memdesc->hostptr) {
		BUG_ON(memdesc->hostptr_count);
		goto done;
	}
	memdesc->hostptr_count--;
	if (memdesc->hostptr_count)
		goto done;
	vunmap(memdesc->hostptr);

	atomic_long_sub(memdesc->size, &kgsl_driver.stats.vmalloc);
	memdesc->hostptr = NULL;
done:
	mutex_unlock(&kernel_map_global_lock);
}

static void kgsl_page_alloc_free(struct kgsl_memdesc *memdesc)
{
	unsigned int i = 0;
	struct scatterlist *sg;
	struct kgsl_process_private *priv = memdesc->private;

	kgsl_page_alloc_unmap_kernel(memdesc);
	
	BUG_ON(memdesc->hostptr);

	
	if (memdesc->priv & KGSL_MEMDESC_TZ_LOCKED) {
		int ret;
		int dest_perms = PERM_READ | PERM_WRITE | PERM_EXEC;
		int source_vm = VMID_CP_PIXEL;
		int dest_vm = VMID_HLOS;

		ret = hyp_assign_table(memdesc->sgt, &source_vm, 1,
					&dest_vm, &dest_perms, 1);
		if (ret) {
			pr_err("Secure buf unlock failed: gpuaddr: %llx size: %llx ret: %d\n",
					memdesc->gpuaddr, memdesc->size, ret);
			BUG();
		}

		atomic_long_sub(memdesc->size, &kgsl_driver.stats.secure);
	} else {
		atomic_long_sub(memdesc->size, &kgsl_driver.stats.page_alloc);
	}

	for_each_sg(memdesc->sgt->sgl, sg, memdesc->sgt->nents, i) {
		struct page *p = sg_page(sg), *next;
		unsigned int j = 0, count;
		while (j < (sg->length/PAGE_SIZE)) {
			if (memdesc->priv & KGSL_MEMDESC_TZ_LOCKED)
				ClearPagePrivate(p);

			count = 1 << compound_order(p);
			next = nth_page(p, count);
			__free_pages(p, compound_order(p));
			p = next;
			j += count;

		}
	}

	if (priv)
		kgsl_process_sub_stats(priv, KGSL_MEM_ENTRY_PAGE_ALLOC, memdesc->size);
}

static int kgsl_page_alloc_map_kernel(struct kgsl_memdesc *memdesc)
{
	int ret = 0;

	
	if (memdesc->size > ULONG_MAX)
		return -ENOMEM;

	mutex_lock(&kernel_map_global_lock);
	if (!memdesc->hostptr) {
		pgprot_t page_prot = pgprot_writecombine(PAGE_KERNEL);
		struct page **pages = NULL;
		struct scatterlist *sg;
		int npages = PAGE_ALIGN(memdesc->size) >> PAGE_SHIFT;
		int sglen = memdesc->sgt->nents;
		int i, count = 0;

		
		pages = kgsl_malloc(npages * sizeof(struct page *));
		if (pages == NULL) {
			ret = -ENOMEM;
			goto done;
		}

		for_each_sg(memdesc->sgt->sgl, sg, sglen, i) {
			struct page *page = sg_page(sg);
			int j;

			for (j = 0; j < sg->length >> PAGE_SHIFT; j++)
				pages[count++] = page++;
		}


		memdesc->hostptr = vmap(pages, count,
					VM_IOREMAP, page_prot);
		if (memdesc->hostptr)
			KGSL_STATS_ADD(memdesc->size,
				&kgsl_driver.stats.vmalloc,
				&kgsl_driver.stats.vmalloc_max);
		else
			ret = -ENOMEM;
		kgsl_free(pages);
	}
	if (memdesc->hostptr)
		memdesc->hostptr_count++;
done:
	mutex_unlock(&kernel_map_global_lock);

	return ret;
}

static int kgsl_contiguous_vmfault(struct kgsl_memdesc *memdesc,
				struct vm_area_struct *vma,
				struct vm_fault *vmf)
{
	unsigned long offset, pfn;
	int ret;

	offset = ((unsigned long) vmf->virtual_address - vma->vm_start) >>
		PAGE_SHIFT;

	pfn = (memdesc->physaddr >> PAGE_SHIFT) + offset;
	ret = vm_insert_pfn(vma, (unsigned long) vmf->virtual_address, pfn);

	if (ret == -ENOMEM || ret == -EAGAIN)
		return VM_FAULT_OOM;
	else if (ret == -EFAULT)
		return VM_FAULT_SIGBUS;

	return VM_FAULT_NOPAGE;
}

static void kgsl_cma_coherent_free(struct kgsl_memdesc *memdesc)
{
	struct dma_attrs *attrs = NULL;

	if (memdesc->hostptr) {
		if (memdesc->priv & KGSL_MEMDESC_SECURE) {
			atomic_long_sub(memdesc->size,
				&kgsl_driver.stats.secure);

			kgsl_cma_unlock_secure(memdesc);
			attrs = &memdesc->attrs;
		} else
			atomic_long_sub(memdesc->size,
				&kgsl_driver.stats.coherent);

		dma_free_attrs(memdesc->dev, (size_t) memdesc->size,
			memdesc->hostptr, memdesc->physaddr, attrs);
	}
}

static struct kgsl_memdesc_ops kgsl_page_alloc_ops = {
	.free = kgsl_page_alloc_free,
	.vmflags = VM_DONTDUMP | VM_DONTEXPAND | VM_DONTCOPY,
	.vmfault = kgsl_page_alloc_vmfault,
	.map_kernel = kgsl_page_alloc_map_kernel,
	.unmap_kernel = kgsl_page_alloc_unmap_kernel,
};

static struct kgsl_memdesc_ops kgsl_cma_ops = {
	.free = kgsl_cma_coherent_free,
	.vmflags = VM_DONTDUMP | VM_PFNMAP | VM_DONTEXPAND | VM_DONTCOPY,
	.vmfault = kgsl_contiguous_vmfault,
};

#ifdef CONFIG_ARM64
static inline unsigned int _fixup_cache_range_op(unsigned int op)
{
	if (op == KGSL_CACHE_OP_INV)
		return KGSL_CACHE_OP_FLUSH;
	return op;
}
#else
static inline unsigned int _fixup_cache_range_op(unsigned int op)
{
	return op;
}
#endif

int kgsl_cache_range_op(struct kgsl_memdesc *memdesc, uint64_t offset,
		uint64_t size, unsigned int op)
{

	void *addr = (memdesc->hostptr) ?
		memdesc->hostptr : (void *) memdesc->useraddr;

	
	if (!size)
		return -EINVAL;

	
	if ((offset + size) > ULONG_MAX)
		return -ERANGE;

	
	if (addr + ((size_t) offset + (size_t) size) < addr)
		return -ERANGE;

	
	if (offset + size > memdesc->size)
		return -ERANGE;

	
	if (addr == NULL)
		return 0;

	addr = addr + offset;


	switch (_fixup_cache_range_op(op)) {
	case KGSL_CACHE_OP_FLUSH:
		dmac_flush_range(addr, addr + (size_t) size);
		break;
	case KGSL_CACHE_OP_CLEAN:
		dmac_clean_range(addr, addr + (size_t) size);
		break;
	case KGSL_CACHE_OP_INV:
		dmac_inv_range(addr, addr + (size_t) size);
		break;
	}

	return 0;
}
EXPORT_SYMBOL(kgsl_cache_range_op);

#ifndef CONFIG_ALLOC_BUFFERS_IN_4K_CHUNKS
static inline int get_page_size(size_t size, unsigned int align)
{
	return (align >= ilog2(SZ_64K) && size >= SZ_64K)
					? SZ_64K : PAGE_SIZE;
}
#else
static inline int get_page_size(size_t size, unsigned int align)
{
	return PAGE_SIZE;
}
#endif

static int
kgsl_sharedmem_page_alloc_user(struct kgsl_memdesc *memdesc,
			struct kgsl_pagetable *pagetable,
			uint64_t size)
{
	int ret = 0;
	unsigned int j, pcount = 0, page_size, len_alloc;
	size_t len;
	struct page **pages = NULL;
	pgprot_t page_prot = pgprot_writecombine(PAGE_KERNEL);
	void *ptr;
	unsigned int align;
	unsigned int step = ((VMALLOC_END - VMALLOC_START)/8) >> PAGE_SHIFT;

	size = PAGE_ALIGN(size);
	if (size == 0 || size > UINT_MAX)
		return -EINVAL;

	align = (memdesc->flags & KGSL_MEMALIGN_MASK) >> KGSL_MEMALIGN_SHIFT;

	page_size = get_page_size(size, align);


	if (align < ilog2(page_size))
		kgsl_memdesc_set_align(memdesc, ilog2(page_size));


	len_alloc = PAGE_ALIGN(size) >> PAGE_SHIFT;

	memdesc->pagetable = pagetable;
	memdesc->ops = &kgsl_page_alloc_ops;

	memdesc->sgt = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (memdesc->sgt == NULL)
		return -ENOMEM;


	pages = kgsl_malloc(len_alloc * sizeof(struct page *));

	if (pages == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	len = size;

	while (len > 0) {
		struct page *page;
		gfp_t gfp_mask = __GFP_HIGHMEM;
		int j;

		
		if (len < page_size)
			page_size = PAGE_SIZE;

		if (page_size != PAGE_SIZE)
			gfp_mask |= __GFP_COMP | __GFP_NORETRY |
				__GFP_NO_KSWAPD | __GFP_NOWARN;
		else
			gfp_mask |= GFP_KERNEL;

		if (sharedmem_noretry_flag == true)
			gfp_mask |= __GFP_NORETRY | __GFP_NOWARN;

		page = alloc_pages(gfp_mask, get_order(page_size));

		if (page == NULL) {
			if (page_size != PAGE_SIZE) {
				page_size = PAGE_SIZE;
				continue;
			}

			memdesc->size = (size - len);

			if (sharedmem_noretry_flag != true)
				KGSL_CORE_ERR(
					"Out of memory: only allocated %lldKB of %lldKB requested\n",
					(size - len) >> 10, size >> 10);

			ret = -ENOMEM;
			goto done;
		}

		for (j = 0; j < page_size >> PAGE_SHIFT; j++)
			pages[pcount++] = nth_page(page, j);

		len -= page_size;
		memdesc->size += page_size;
	}

	ret = sg_alloc_table_from_pages(memdesc->sgt, pages, pcount, 0,
				memdesc->size, GFP_KERNEL);
	if (ret)
		goto done;

	
	if (memdesc->flags & KGSL_MEMFLAGS_SECURE) {
		unsigned int i;
		struct scatterlist *sg;
		int dest_perms = PERM_READ | PERM_WRITE;
		int source_vm = VMID_HLOS;
		int dest_vm = VMID_CP_PIXEL;

		ret = hyp_assign_table(memdesc->sgt, &source_vm, 1,
					&dest_vm, &dest_perms, 1);
		if (ret)
			goto done;

		
		for_each_sg(memdesc->sgt->sgl, sg, memdesc->sgt->nents, i)
			SetPagePrivate(sg_page(sg));

		memdesc->priv |= KGSL_MEMDESC_TZ_LOCKED;

		
		KGSL_STATS_ADD(memdesc->size, &kgsl_driver.stats.secure,
			&kgsl_driver.stats.secure_max);

		
		goto done;
	}

	for (j = 0; j < pcount; j += step) {
		step = min(step, pcount - j);

		ptr = vmap(&pages[j], step, VM_IOREMAP, page_prot);

		if (ptr != NULL) {
			memset(ptr, 0, step * PAGE_SIZE);
			dmac_flush_range(ptr, ptr + step * PAGE_SIZE);
			vunmap(ptr);
		} else {
			int k;
			

			for (k = j; k < j + step; k++) {
				ptr = kmap_atomic(pages[k]);
				memset(ptr, 0, PAGE_SIZE);
				dmac_flush_range(ptr, ptr + PAGE_SIZE);
				kunmap_atomic(ptr);
			}
			
			if (step > 1)
				step >>= 1;
		}
	}

	KGSL_STATS_ADD(memdesc->size, &kgsl_driver.stats.page_alloc,
		&kgsl_driver.stats.page_alloc_max);
	if (memdesc->private)
		kgsl_process_add_stats(memdesc->private, KGSL_MEM_ENTRY_PAGE_ALLOC, size);

done:
	if (ret) {
		unsigned int count = 1;
		for (j = 0; j < pcount; j += count) {
			count = 1 << compound_order(pages[j]);
			__free_pages(pages[j], compound_order(pages[j]));
		}

		kfree(memdesc->sgt);
		memset(memdesc, 0, sizeof(*memdesc));
	}
	kgsl_free(pages);

	return ret;
}

void kgsl_sharedmem_free(struct kgsl_memdesc *memdesc)
{
	if (memdesc == NULL || memdesc->size == 0)
		return;

	if (memdesc->gpuaddr) {
		kgsl_mmu_unmap(memdesc->pagetable, memdesc);
		kgsl_mmu_put_gpuaddr(memdesc->pagetable, memdesc);
	}

	if (memdesc->ops && memdesc->ops->free)
		memdesc->ops->free(memdesc);

	if (memdesc->sgt) {
		sg_free_table(memdesc->sgt);
		kfree(memdesc->sgt);
	}

	memset(memdesc, 0, sizeof(*memdesc));
}
EXPORT_SYMBOL(kgsl_sharedmem_free);

int
kgsl_sharedmem_readl(const struct kgsl_memdesc *memdesc,
			uint32_t *dst,
			uint64_t offsetbytes)
{
	uint32_t *src;
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL || dst == NULL);
	WARN_ON(offsetbytes % sizeof(uint32_t) != 0);
	if (offsetbytes % sizeof(uint32_t) != 0)
		return -EINVAL;

	WARN_ON(offsetbytes + sizeof(uint32_t) > memdesc->size);
	if (offsetbytes + sizeof(uint32_t) > memdesc->size)
		return -ERANGE;

	rmb();
	src = (uint32_t *)(memdesc->hostptr + offsetbytes);
	*dst = *src;
	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_readl);

int
kgsl_sharedmem_writel(struct kgsl_device *device,
			const struct kgsl_memdesc *memdesc,
			uint64_t offsetbytes,
			uint32_t src)
{
	uint32_t *dst;
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL);
	WARN_ON(offsetbytes % sizeof(uint32_t) != 0);
	if (offsetbytes % sizeof(uint32_t) != 0)
		return -EINVAL;

	WARN_ON(offsetbytes + sizeof(uint32_t) > memdesc->size);
	if (offsetbytes + sizeof(uint32_t) > memdesc->size)
		return -ERANGE;
	kgsl_cffdump_write(device,
		memdesc->gpuaddr + offsetbytes,
		src);
	dst = (uint32_t *)(memdesc->hostptr + offsetbytes);
	*dst = src;

	wmb();

	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_writel);

int
kgsl_sharedmem_readq(const struct kgsl_memdesc *memdesc,
			uint64_t *dst,
			uint64_t offsetbytes)
{
	uint64_t *src;
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL || dst == NULL);
	WARN_ON(offsetbytes % sizeof(uint32_t) != 0);
	if (offsetbytes % sizeof(uint32_t) != 0)
		return -EINVAL;

	WARN_ON(offsetbytes + sizeof(uint32_t) > memdesc->size);
	if (offsetbytes + sizeof(uint32_t) > memdesc->size)
		return -ERANGE;

	rmb();
	src = (uint64_t *)(memdesc->hostptr + offsetbytes);
	*dst = *src;
	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_readq);

int
kgsl_sharedmem_writeq(struct kgsl_device *device,
			const struct kgsl_memdesc *memdesc,
			uint64_t offsetbytes,
			uint64_t src)
{
	uint64_t *dst;
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL);
	WARN_ON(offsetbytes % sizeof(uint32_t) != 0);
	if (offsetbytes % sizeof(uint32_t) != 0)
		return -EINVAL;

	WARN_ON(offsetbytes + sizeof(uint32_t) > memdesc->size);
	if (offsetbytes + sizeof(uint32_t) > memdesc->size)
		return -ERANGE;
	kgsl_cffdump_write(device,
		lower_32_bits(memdesc->gpuaddr + offsetbytes), src);
	kgsl_cffdump_write(device,
		upper_32_bits(memdesc->gpuaddr + offsetbytes), src);
	dst = (uint64_t *)(memdesc->hostptr + offsetbytes);
	*dst = src;

	wmb();

	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_writeq);

int
kgsl_sharedmem_set(struct kgsl_device *device,
		const struct kgsl_memdesc *memdesc, uint64_t offsetbytes,
		unsigned int value, uint64_t sizebytes)
{
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL);
	BUG_ON(offsetbytes + sizebytes > memdesc->size);

	kgsl_cffdump_memset(device,
		memdesc->gpuaddr + offsetbytes, value,
		sizebytes);
	memset(memdesc->hostptr + offsetbytes, value, sizebytes);
	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_set);

static const char * const memtype_str[] = {
	[KGSL_MEMTYPE_OBJECTANY] = "any(0)",
	[KGSL_MEMTYPE_FRAMEBUFFER] = "framebuffer",
	[KGSL_MEMTYPE_RENDERBUFFER] = "renderbuffer",
	[KGSL_MEMTYPE_ARRAYBUFFER] = "arraybuffer",
	[KGSL_MEMTYPE_ELEMENTARRAYBUFFER] = "elementarraybuffer",
	[KGSL_MEMTYPE_VERTEXARRAYBUFFER] = "vertexarraybuffer",
	[KGSL_MEMTYPE_TEXTURE] = "texture",
	[KGSL_MEMTYPE_SURFACE] = "surface",
	[KGSL_MEMTYPE_EGL_SURFACE] = "egl_surface",
	[KGSL_MEMTYPE_GL] = "gl",
	[KGSL_MEMTYPE_CL] = "cl",
	[KGSL_MEMTYPE_CL_BUFFER_MAP] = "cl_buffer_map",
	[KGSL_MEMTYPE_CL_BUFFER_NOMAP] = "cl_buffer_nomap",
	[KGSL_MEMTYPE_CL_IMAGE_MAP] = "cl_image_map",
	[KGSL_MEMTYPE_CL_IMAGE_NOMAP] = "cl_image_nomap",
	[KGSL_MEMTYPE_CL_KERNEL_STACK] = "cl_kernel_stack",
	[KGSL_MEMTYPE_COMMAND] = "command",
	[KGSL_MEMTYPE_2D] = "2d",
	[KGSL_MEMTYPE_EGL_IMAGE] = "egl_image",
	[KGSL_MEMTYPE_EGL_SHADOW] = "egl_shadow",
	[KGSL_MEMTYPE_MULTISAMPLE] = "egl_multisample",
	
};

void kgsl_get_memory_usage(char *name, size_t name_size, uint64_t memflags)
{
	unsigned int type = MEMFLAGS(memflags, KGSL_MEMTYPE_MASK,
		KGSL_MEMTYPE_SHIFT);

	if (type == KGSL_MEMTYPE_KERNEL)
		strlcpy(name, "kernel", name_size);
	else if (type < ARRAY_SIZE(memtype_str) && memtype_str[type] != NULL)
		strlcpy(name, memtype_str[type], name_size);
	else
		snprintf(name, name_size, "unknown(%3d)", type);
}
EXPORT_SYMBOL(kgsl_get_memory_usage);

int kgsl_sharedmem_alloc_contig(struct kgsl_device *device,
			struct kgsl_memdesc *memdesc,
			struct kgsl_pagetable *pagetable, uint64_t size)
{
	int result = 0;

	size = PAGE_ALIGN(size);
	if (size == 0 || size > SIZE_MAX)
		return -EINVAL;

	memdesc->size = size;
	memdesc->pagetable = pagetable;
	memdesc->ops = &kgsl_cma_ops;
	memdesc->dev = device->dev->parent;

	memdesc->hostptr = dma_alloc_attrs(memdesc->dev, (size_t) size,
		&memdesc->physaddr, GFP_KERNEL, NULL);

	if (memdesc->hostptr == NULL) {
		result = -ENOMEM;
		goto err;
	}

	result = memdesc_sg_dma(memdesc, memdesc->physaddr, size);
	if (result)
		goto err;

	

	if (kgsl_mmu_get_mmutype() == KGSL_MMU_TYPE_NONE)
		memdesc->gpuaddr = memdesc->physaddr;

	KGSL_STATS_ADD(size, &kgsl_driver.stats.coherent,
		&kgsl_driver.stats.coherent_max);

err:
	if (result)
		kgsl_sharedmem_free(memdesc);

	return result;
}
EXPORT_SYMBOL(kgsl_sharedmem_alloc_contig);

static int scm_lock_chunk(struct kgsl_memdesc *memdesc, int lock)
{
	struct cp2_lock_req request;
	unsigned int resp;
	unsigned int *chunk_list;
	struct scm_desc desc = {0};
	int result;

	chunk_list = kzalloc(sizeof(unsigned int), GFP_KERNEL);
	if (!chunk_list)
		return -ENOMEM;

	chunk_list[0] = memdesc->physaddr;
	dmac_flush_range((void *)chunk_list, (void *)chunk_list + 1);

	request.chunks.chunk_list = virt_to_phys(chunk_list);
	desc.args[0] = virt_to_phys(chunk_list);
	desc.args[1] = request.chunks.chunk_list_size = 1;
	desc.args[2] = request.chunks.chunk_size = (unsigned int) memdesc->size;
	desc.args[3] = request.mem_usage = 0;
	desc.args[4] = request.lock = lock;
	desc.args[5] = 0;
	desc.arginfo = SCM_ARGS(6, SCM_RW, SCM_VAL, SCM_VAL, SCM_VAL, SCM_VAL,
				SCM_VAL);
	kmap_flush_unused();
	kmap_atomic_flush_unused();
	if (!is_scm_armv8()) {
		result = scm_call(SCM_SVC_MP, MEM_PROTECT_LOCK_ID2,
				&request, sizeof(request), &resp, sizeof(resp));
	} else {
		result = scm_call2(SCM_SIP_FNID(SCM_SVC_MP,
				   MEM_PROTECT_LOCK_ID2_FLAT), &desc);
		resp = desc.ret[0];
	}

	kfree(chunk_list);
	return result;
}

static int kgsl_cma_alloc_secure(struct kgsl_device *device,
			struct kgsl_memdesc *memdesc, uint64_t size)
{
	struct kgsl_iommu *iommu = device->mmu.priv;
	int result = 0;
	struct kgsl_pagetable *pagetable = device->mmu.securepagetable;
	size_t aligned;

	
	aligned = ALIGN(size, SZ_1M);

	
	if (aligned == 0 || aligned > UINT_MAX)
		return -EINVAL;


	if (memdesc->priv & KGSL_MEMDESC_GUARD_PAGE)
		if (aligned - size >= SZ_4K)
			memdesc->priv &= ~KGSL_MEMDESC_GUARD_PAGE;

	memdesc->size = aligned;
	memdesc->pagetable = pagetable;
	memdesc->ops = &kgsl_cma_ops;
	memdesc->dev = iommu->ctx[KGSL_IOMMU_CONTEXT_SECURE].dev;

	init_dma_attrs(&memdesc->attrs);
	dma_set_attr(DMA_ATTR_STRONGLY_ORDERED, &memdesc->attrs);

	memdesc->hostptr = dma_alloc_attrs(memdesc->dev, aligned,
		&memdesc->physaddr, GFP_KERNEL, &memdesc->attrs);

	if (memdesc->hostptr == NULL) {
		result = -ENOMEM;
		goto err;
	}

	result = memdesc_sg_dma(memdesc, memdesc->physaddr, aligned);
	if (result)
		goto err;

	result = scm_lock_chunk(memdesc, 1);

	if (result != 0)
		goto err;

	
	SetPagePrivate(sg_page(memdesc->sgt->sgl));

	memdesc->priv |= KGSL_MEMDESC_TZ_LOCKED;

	
	KGSL_STATS_ADD(aligned, &kgsl_driver.stats.secure,
	       &kgsl_driver.stats.secure_max);
err:
	if (result)
		kgsl_sharedmem_free(memdesc);

	return result;
}

static void kgsl_cma_unlock_secure(struct kgsl_memdesc *memdesc)
{
	if (memdesc->size == 0 || !(memdesc->priv & KGSL_MEMDESC_TZ_LOCKED))
		return;

	if (!scm_lock_chunk(memdesc, 0))
		ClearPagePrivate(sg_page(memdesc->sgt->sgl));
}

void kgsl_sharedmem_set_noretry(bool val)
{
	sharedmem_noretry_flag = val;
}

bool kgsl_sharedmem_get_noretry(void)
{
	return sharedmem_noretry_flag;
}
