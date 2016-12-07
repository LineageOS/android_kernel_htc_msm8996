/* Copyright (c) 2002,2007-2016, The Linux Foundation. All rights reserved.
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
#include "adreno.h"
#include "kgsl_sharedmem.h"
#include "a3xx_reg.h"
#include "adreno_pm4types.h"

#define A5XX_PFP_PER_PROCESS_UCODE_VER 0x5FF064
#define A5XX_PM4_PER_PROCESS_UCODE_VER 0x5FF052

static unsigned int _wait_reg(struct adreno_device *adreno_dev,
			unsigned int *cmds, unsigned int addr,
			unsigned int val, unsigned int mask,
			unsigned int interval)
{
	unsigned int *start = cmds;

	if (adreno_is_a3xx(adreno_dev)) {
		*cmds++ = cp_packet(adreno_dev, CP_WAIT_REG_EQ, 4);
		*cmds++ = addr;
		*cmds++ = val;
		*cmds++ = mask;
		*cmds++ = interval;
	} else {
		*cmds++ = cp_mem_packet(adreno_dev, CP_WAIT_REG_MEM, 5, 1);
		*cmds++ = 0x3; 
		cmds += cp_gpuaddr(adreno_dev, cmds, addr); 
		*cmds++ = val; 
		*cmds++ = mask;
		*cmds++ = interval;

		
		*cmds++ = cp_packet(adreno_dev, CP_SET_PROTECTED_MODE, 1);
		*cmds++ = 0;
	}

	return cmds - start;
}

#define KGSL_MMU(_dev) \
	((struct kgsl_mmu *) (&(KGSL_DEVICE((_dev))->mmu)))

static unsigned int  _iommu_lock(struct adreno_device *adreno_dev,
				 unsigned int *cmds)
{
	unsigned int *start = cmds;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);

	BUG_ON(iommu->micro_mmu_ctrl == UINT_MAX);

	cmds += _wait_reg(adreno_dev, cmds,
			adreno_getreg(adreno_dev, ADRENO_REG_CP_WFI_PEND_CTR),
			1, 0xFFFFFFFF, 0xF);

	
	*cmds++ = cp_packet(adreno_dev, CP_REG_RMW, 3);
	*cmds++ = iommu->micro_mmu_ctrl >> 2;
	
	*cmds++ = ~(KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT);
	
	*cmds++ = KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT;

	
	cmds += _wait_reg(adreno_dev, cmds, iommu->micro_mmu_ctrl >> 2,
			KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE,
			KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE, 0xF);

	return cmds - start;
}

static unsigned int _iommu_unlock(struct adreno_device *adreno_dev,
				  unsigned int *cmds)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);
	unsigned int *start = cmds;

	BUG_ON(iommu->micro_mmu_ctrl == UINT_MAX);

	
	*cmds++ = cp_packet(adreno_dev, CP_REG_RMW, 3);
	*cmds++ = iommu->micro_mmu_ctrl >> 2;
	
	*cmds++ = ~(KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT);
	
	*cmds++ = 0;

	
	cmds += cp_wait_for_me(adreno_dev, cmds);

	return cmds - start;
}

static unsigned int _vbif_lock(struct adreno_device *adreno_dev,
			unsigned int *cmds)
{
	unsigned int *start = cmds;
	cmds += _wait_reg(adreno_dev, cmds,
			adreno_getreg(adreno_dev, ADRENO_REG_CP_WFI_PEND_CTR),
			1, 0xFFFFFFFF, 0xF);

	
	*cmds++ = cp_packet(adreno_dev, CP_REG_RMW, 3);
	*cmds++ = A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL0;
	
	*cmds++ = ~(VBIF_RECOVERABLE_HALT_CTRL);
	
	*cmds++ = 0x1;

	
	cmds += _wait_reg(adreno_dev, cmds,
			A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL1,
			1, 0xFFFFFFFF, 0xF);

	return cmds - start;
}

static unsigned int _vbif_unlock(struct adreno_device *adreno_dev,
				unsigned int *cmds)
{
	unsigned int *start = cmds;

	
	*cmds++ = cp_packet(adreno_dev, CP_REG_RMW, 3);
	*cmds++ = A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL0;
	
	*cmds++ = ~(VBIF_RECOVERABLE_HALT_CTRL);
	
	*cmds++ = 0;

	
	cmds += cp_wait_for_me(adreno_dev, cmds);
	return cmds - start;
}

static unsigned int _cp_smmu_reg(struct adreno_device *adreno_dev,
				unsigned int *cmds,
				enum kgsl_iommu_reg_map reg,
				unsigned int num)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);
	unsigned int *start = cmds;
	unsigned int offset;

	offset = kgsl_mmu_get_reg_ahbaddr(KGSL_MMU(adreno_dev),
					  KGSL_IOMMU_CONTEXT_USER, reg) >> 2;

	if (adreno_is_a5xx(adreno_dev) || iommu->version == 1) {
		*cmds++ = cp_register(adreno_dev, offset, num);
	} else if (adreno_is_a3xx(adreno_dev)) {
		*cmds++ = cp_packet(adreno_dev, CP_REG_WR_NO_CTXT, num + 1);
		*cmds++ = offset;
	} else if (adreno_is_a4xx(adreno_dev)) {
		*cmds++ = cp_packet(adreno_dev, CP_WIDE_REG_WRITE, num + 1);
		*cmds++ = offset;
	} else  {
		BUG();
	}
	return cmds - start;
}

static unsigned int _tlbiall(struct adreno_device *adreno_dev,
				unsigned int *cmds)
{
	unsigned int *start = cmds;
	unsigned int tlbstatus;

	tlbstatus = kgsl_mmu_get_reg_ahbaddr(KGSL_MMU(adreno_dev),
			KGSL_IOMMU_CONTEXT_USER,
			KGSL_IOMMU_CTX_TLBSTATUS) >> 2;

	cmds += _cp_smmu_reg(adreno_dev, cmds, KGSL_IOMMU_CTX_TLBIALL, 1);
	*cmds++ = 1;

	cmds += _cp_smmu_reg(adreno_dev, cmds, KGSL_IOMMU_CTX_TLBSYNC, 1);
	*cmds++ = 0;

	cmds += _wait_reg(adreno_dev, cmds, tlbstatus, 0,
			KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE, 0xF);

	return cmds - start;
}


static inline int _adreno_iommu_add_idle_cmds(struct adreno_device *adreno_dev,
							unsigned int *cmds)
{
	unsigned int *start = cmds;

	cmds += cp_wait_for_idle(adreno_dev, cmds);

	if (adreno_is_a3xx(adreno_dev))
		cmds += cp_wait_for_me(adreno_dev, cmds);

	return cmds - start;
}

static void _invalidate_uche_cpu(struct adreno_device *adreno_dev)
{
	
	if (adreno_is_a5xx(adreno_dev))
		adreno_writereg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE0, 0x12);
	else if (adreno_is_a4xx(adreno_dev)) {
		adreno_writereg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE0, 0);
		adreno_writereg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE1, 0x12);
	} else if (adreno_is_a3xx(adreno_dev)) {
		adreno_writereg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE0, 0);
		adreno_writereg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE1,
			0x90000000);
	} else {
		BUG();
	}
}

static bool _ctx_switch_use_cpu_path(
				struct adreno_device *adreno_dev,
				struct kgsl_pagetable *new_pt,
				struct adreno_ringbuffer *rb)
{
	struct kgsl_mmu *mmu = KGSL_MMU(adreno_dev);

	if (adreno_dev->cur_rb == rb)
		return adreno_isidle(KGSL_DEVICE(adreno_dev)) &&
			(new_pt == mmu->defaultpagetable);
	else if (adreno_rb_empty(rb) &&
			(new_pt == mmu->defaultpagetable))
		return true;

	return false;
}

static unsigned int adreno_iommu_set_apriv(struct adreno_device *adreno_dev,
				unsigned int *cmds, int set)
{
	unsigned int *cmds_orig = cmds;

	
	if (adreno_is_a3xx(adreno_dev))
		return 0;

	cmds += cp_wait_for_idle(adreno_dev, cmds);
	cmds += cp_wait_for_me(adreno_dev, cmds);
	*cmds++ = cp_register(adreno_dev, adreno_getreg(adreno_dev,
				ADRENO_REG_CP_CNTL), 1);
	if (set)
		*cmds++ = 1;
	else
		*cmds++ = 0;

	return cmds - cmds_orig;
}

static inline int _adreno_iommu_add_idle_indirect_cmds(
			struct adreno_device *adreno_dev,
			unsigned int *cmds, uint64_t nop_gpuaddr)
{
	unsigned int *start = cmds;
	cmds += cp_wait_for_me(adreno_dev, cmds);
	*cmds++ = cp_mem_packet(adreno_dev, CP_INDIRECT_BUFFER_PFE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds, nop_gpuaddr);
	*cmds++ = 2;
	cmds += cp_wait_for_idle(adreno_dev, cmds);
	return cmds - start;
}

static unsigned int _adreno_mmu_set_pt_update_condition(
			struct adreno_ringbuffer *rb,
			unsigned int *cmds, unsigned int ptname)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	unsigned int *cmds_orig = cmds;
	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(switch_pt_enable)));
	*cmds++ = 1;
	*cmds++ = cp_packet(adreno_dev, CP_WAIT_MEM_WRITES, 1);
	*cmds++ = 0;
	cmds += cp_wait_for_me(adreno_dev, cmds);
	*cmds++ = cp_mem_packet(adreno_dev, CP_COND_WRITE, 6, 2);
	
	*cmds++ = (1 << 8) | (1 << 4) | 3;
	cmds += cp_gpuaddr(adreno_dev, cmds,
	   (adreno_dev->ringbuffers[0].pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(current_global_ptname)));
	*cmds++ = ptname;
	*cmds++ = 0xFFFFFFFF;
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(switch_pt_enable)));
	*cmds++ = 0;
	*cmds++ = cp_packet(adreno_dev, CP_WAIT_MEM_WRITES, 1);
	*cmds++ = 0;
	cmds += cp_wait_for_me(adreno_dev, cmds);

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_pt_update_pid_to_mem(
				struct adreno_ringbuffer *rb,
				unsigned int *cmds, int ptname)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	unsigned int *cmds_orig = cmds;

	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(current_rb_ptname)));
	*cmds++ = ptname;
	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds,
		(adreno_dev->ringbuffers[0].pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(current_global_ptname)));
	*cmds++ = ptname;
	
	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(switch_pt_enable)));
	*cmds++ = 0;
	*cmds++ = cp_packet(adreno_dev, CP_WAIT_MEM_WRITES, 1);
	*cmds++ = 0;
	cmds += cp_wait_for_me(adreno_dev, cmds);

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_set_pt_v1(struct adreno_ringbuffer *rb,
					unsigned int *cmds_orig,
					u64 ttbr0, u32 contextidr, u32 ptname)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	unsigned int *cmds = cmds_orig;
	unsigned int *cond_exec_ptr;

	cmds += _adreno_iommu_add_idle_cmds(adreno_dev, cmds);

	
	cmds += _adreno_mmu_set_pt_update_condition(rb, cmds, ptname);
	*cmds++ = cp_mem_packet(adreno_dev, CP_COND_EXEC, 4, 2);
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(switch_pt_enable)));
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(switch_pt_enable)));
	*cmds++ = 1;
	
	cond_exec_ptr = cmds;
	cmds++;

	cmds += cp_wait_for_idle(adreno_dev, cmds);

	cmds += _iommu_lock(adreno_dev, cmds);

	cmds += _cp_smmu_reg(adreno_dev, cmds, KGSL_IOMMU_CTX_TTBR0, 2);
	*cmds++ = lower_32_bits(ttbr0);
	*cmds++ = upper_32_bits(ttbr0);
	cmds += _cp_smmu_reg(adreno_dev, cmds,
			KGSL_IOMMU_CTX_CONTEXTIDR, 1);
	*cmds++ = contextidr;

	
	if (adreno_is_a3xx(adreno_dev))
		cmds += _iommu_unlock(adreno_dev, cmds);

	cmds += _tlbiall(adreno_dev, cmds);

	
	if (!adreno_is_a3xx(adreno_dev))
		cmds += _iommu_unlock(adreno_dev, cmds);
	else
		cmds += cp_wait_for_me(adreno_dev, cmds);

	
	*cond_exec_ptr = (cmds - cond_exec_ptr - 1);
	cmds += _adreno_iommu_add_idle_cmds(adreno_dev, cmds);
	cmds += _adreno_iommu_pt_update_pid_to_mem(rb, cmds, ptname);

	return cmds - cmds_orig;
}


static unsigned int _adreno_iommu_set_pt_v2_a3xx(struct kgsl_device *device,
					unsigned int *cmds_orig,
					u64 ttbr0, u32 contextidr)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int *cmds = cmds_orig;

	cmds += _adreno_iommu_add_idle_cmds(adreno_dev, cmds);

	cmds += _vbif_lock(adreno_dev, cmds);

	cmds += _cp_smmu_reg(adreno_dev, cmds, KGSL_IOMMU_CTX_TTBR0, 2);
	*cmds++ = lower_32_bits(ttbr0);
	*cmds++ = upper_32_bits(ttbr0);
	cmds += _cp_smmu_reg(adreno_dev, cmds, KGSL_IOMMU_CTX_CONTEXTIDR, 1);
	*cmds++ = contextidr;

	cmds += _vbif_unlock(adreno_dev, cmds);

	cmds += _tlbiall(adreno_dev, cmds);

	
	cmds += cp_wait_for_me(adreno_dev, cmds);

	cmds += _adreno_iommu_add_idle_cmds(adreno_dev, cmds);

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_set_pt_v2_a4xx(struct kgsl_device *device,
					unsigned int *cmds_orig,
					u64 ttbr0, u32 contextidr)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int *cmds = cmds_orig;

	cmds += _adreno_iommu_add_idle_cmds(adreno_dev, cmds);

	cmds += _vbif_lock(adreno_dev, cmds);

	cmds += _cp_smmu_reg(adreno_dev, cmds, KGSL_IOMMU_CTX_TTBR0, 2);
	*cmds++ = lower_32_bits(ttbr0);
	*cmds++ = upper_32_bits(ttbr0);
	cmds += _cp_smmu_reg(adreno_dev, cmds, KGSL_IOMMU_CTX_CONTEXTIDR, 1);
	*cmds++ = contextidr;

	cmds += _vbif_unlock(adreno_dev, cmds);

	cmds += _tlbiall(adreno_dev, cmds);

	
	cmds += cp_wait_for_me(adreno_dev, cmds);

	cmds += _adreno_iommu_add_idle_cmds(adreno_dev, cmds);

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_set_pt_v2_a5xx(struct kgsl_device *device,
					unsigned int *cmds_orig,
					u64 ttbr0, u32 contextidr,
					struct adreno_ringbuffer *rb)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int *cmds = cmds_orig;

	cmds += _adreno_iommu_add_idle_cmds(adreno_dev, cmds);
	cmds += cp_wait_for_me(adreno_dev, cmds);

	
	*cmds++ = cp_packet(adreno_dev, CP_SMMU_TABLE_UPDATE, 3);
	*cmds++ = lower_32_bits(ttbr0);
	*cmds++ = upper_32_bits(ttbr0);
	*cmds++ = contextidr;

	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 4, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(ttbr0)));
	*cmds++ = lower_32_bits(ttbr0);
	*cmds++ = upper_32_bits(ttbr0);
	*cmds++ = contextidr;

	
	cmds += cp_wait_for_me(adreno_dev, cmds);

	cmds += _adreno_iommu_add_idle_cmds(adreno_dev, cmds);

	return cmds - cmds_orig;
}

unsigned int adreno_iommu_set_pt_generate_cmds(
					struct adreno_ringbuffer *rb,
					unsigned int *cmds,
					struct kgsl_pagetable *pt)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);
	u64 ttbr0;
	u32 contextidr;
	unsigned int *cmds_orig = cmds;

	ttbr0 = kgsl_mmu_pagetable_get_ttbr0(pt);
	contextidr = kgsl_mmu_pagetable_get_contextidr(pt);

	cmds += adreno_iommu_set_apriv(adreno_dev, cmds, 1);

	cmds += _adreno_iommu_add_idle_indirect_cmds(adreno_dev, cmds,
		iommu->setstate.gpuaddr + KGSL_IOMMU_SETSTATE_NOP_OFFSET);

	if (iommu->version >= 2) {
		if (adreno_is_a5xx(adreno_dev))
			cmds += _adreno_iommu_set_pt_v2_a5xx(device, cmds,
						ttbr0, contextidr, rb);
		else if (adreno_is_a4xx(adreno_dev))
			cmds += _adreno_iommu_set_pt_v2_a4xx(device, cmds,
						ttbr0, contextidr);
		else if (adreno_is_a3xx(adreno_dev))
			cmds += _adreno_iommu_set_pt_v2_a3xx(device, cmds,
						ttbr0, contextidr);
		else
			BUG(); 
	} else {
		cmds += _adreno_iommu_set_pt_v1(rb, cmds, ttbr0, contextidr,
						pt->name);
	}

	
	cmds += cp_invalidate_state(adreno_dev, cmds);

	cmds += adreno_iommu_set_apriv(adreno_dev, cmds, 0);

	return cmds - cmds_orig;
}

static unsigned int __add_curr_ctxt_cmds(struct adreno_ringbuffer *rb,
			unsigned int *cmds,
			struct adreno_context *drawctxt)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	unsigned int *cmds_orig = cmds;

	
	*cmds++ = cp_packet(adreno_dev, CP_NOP, 1);
	*cmds++ = KGSL_CONTEXT_TO_MEM_IDENTIFIER;

	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds,
			MEMSTORE_RB_GPU_ADDR(device, rb, current_context));
	*cmds++ = (drawctxt ? drawctxt->base.id : 0);

	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds,
			MEMSTORE_ID_GPU_ADDR(device,
				KGSL_MEMSTORE_GLOBAL, current_context));
	*cmds++ = (drawctxt ? drawctxt->base.id : 0);

	
	if (adreno_is_a5xx(adreno_dev)) {
		*cmds++ = cp_register(adreno_dev,
			adreno_getreg(adreno_dev,
		ADRENO_REG_UCHE_INVALIDATE0), 1);
		*cmds++ = 0x12;
	} else if (adreno_is_a4xx(adreno_dev)) {
		*cmds++ = cp_register(adreno_dev,
			adreno_getreg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE0), 2);
		*cmds++ = 0;
		*cmds++ = 0x12;
	} else if (adreno_is_a3xx(adreno_dev)) {
		*cmds++ = cp_register(adreno_dev,
			adreno_getreg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE0), 2);
		*cmds++ = 0;
		*cmds++ = 0x90000000;
	} else
		BUG();

	return cmds - cmds_orig;
}

static void _set_ctxt_cpu(struct adreno_ringbuffer *rb,
			struct adreno_context *drawctxt)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	if (rb == adreno_dev->cur_rb) {
		_invalidate_uche_cpu(adreno_dev);
		
		kgsl_sharedmem_writel(device, &device->memstore,
			KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL,
						current_context),
			drawctxt ? drawctxt->base.id : 0);
	}
	
	kgsl_sharedmem_writel(device, &device->memstore,
		MEMSTORE_RB_OFFSET(rb, current_context),
		drawctxt ? drawctxt->base.id : 0);
}

static int _set_ctxt_gpu(struct adreno_ringbuffer *rb,
			struct adreno_context *drawctxt)
{
	unsigned int link[15], *cmds;
	int result;

	cmds = &link[0];
	cmds += __add_curr_ctxt_cmds(rb, cmds, drawctxt);
	result = adreno_ringbuffer_issuecmds(rb, 0, link,
			(unsigned int)(cmds - link));
	return result;
}

static int _set_pagetable_cpu(struct adreno_ringbuffer *rb,
			struct kgsl_pagetable *new_pt)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int result;

	
	if (adreno_dev->cur_rb == rb) {
		result = kgsl_mmu_set_pt(&device->mmu, new_pt);
		if (result)
			return result;
		
		adreno_ringbuffer_set_global(adreno_dev, new_pt->name);
	}

	
	adreno_ringbuffer_set_pagetable(rb, new_pt);

	return 0;
}

static int _set_pagetable_gpu(struct adreno_ringbuffer *rb,
			struct kgsl_pagetable *new_pt)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	unsigned int *link = NULL, *cmds;
	int result;

	link = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (link == NULL)
		return -ENOMEM;

	cmds = link;

	
	if (test_bit(ADRENO_DEVICE_FAULT, &adreno_dev->priv)) {
		kfree(link);
		return 0;
	}

	cmds += adreno_iommu_set_pt_generate_cmds(rb, cmds, new_pt);

	if ((unsigned int) (cmds - link) > (PAGE_SIZE / sizeof(unsigned int))) {
		KGSL_DRV_ERR(KGSL_DEVICE(adreno_dev),
			"Temp command buffer overflow\n");
		BUG();
	}
	result = adreno_ringbuffer_issuecmds(rb,
			KGSL_CMD_FLAGS_PMODE, link,
			(unsigned int)(cmds - link));

	kfree(link);
	return result;
}

int adreno_iommu_init(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);

	if (kgsl_mmu_get_mmutype(device) == KGSL_MMU_TYPE_NONE)
		return 0;


	kgsl_sharedmem_writel(device, &iommu->setstate,
				KGSL_IOMMU_SETSTATE_NOP_OFFSET,
				cp_packet(adreno_dev, CP_NOP, 1));

	
	if (adreno_is_a420(adreno_dev))
		device->mmu.features |= KGSL_MMU_FLUSH_TLB_ON_MAP;

	if (adreno_is_a5xx(adreno_dev) &&
		!MMU_FEATURE(&device->mmu, KGSL_MMU_GLOBAL_PAGETABLE)) {
		if ((adreno_compare_pfp_version(adreno_dev,
				A5XX_PFP_PER_PROCESS_UCODE_VER) < 0) ||
		    (adreno_compare_pm4_version(adreno_dev,
				A5XX_PM4_PER_PROCESS_UCODE_VER) < 0)) {
			KGSL_DRV_ERR(device,
				"Invalid ucode for per process pagetables\n");
			return -ENODEV;
		}
	}

	
	if (adreno_is_a3xx(adreno_dev) || adreno_is_a4xx(adreno_dev))
		device->mmu.features |= KGSL_MMU_NEED_GUARD_PAGE;

	return 0;
}

int adreno_iommu_set_pt_ctx(struct adreno_ringbuffer *rb,
			struct kgsl_pagetable *new_pt,
			struct adreno_context *drawctxt,
			unsigned long flags)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pagetable *cur_pt = device->mmu.defaultpagetable;
	int result = 0;
	int cpu_path = 0;

	if (rb->drawctxt_active)
		cur_pt = rb->drawctxt_active->base.proc_priv->pagetable;

	cpu_path = !(flags & ADRENO_CONTEXT_SWITCH_FORCE_GPU) &&
		_ctx_switch_use_cpu_path(adreno_dev, new_pt, rb);

	
	if (new_pt != cur_pt) {
		if (cpu_path)
			result = _set_pagetable_cpu(rb, new_pt);
		else
			result = _set_pagetable_gpu(rb, new_pt);
	}

	if (result)
		return result;

	
	if (cpu_path)
		_set_ctxt_cpu(rb, drawctxt);
	else
		result = _set_ctxt_gpu(rb, drawctxt);

	return result;
}
