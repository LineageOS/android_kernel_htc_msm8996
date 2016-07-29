/** @file circularbuf.c
 *
 * PCIe full dongle related circular buffer definition, only used by PHANTOM PCIe chip types.
 *
 * PCIe host driver and dongle firmware need to communicate with each other. The mechanism consists
 * of multiple circular buffers located in (DMA'able) host memory. A circular buffer is either used
 * for host -> dongle (h2d) or dongle -> host communication. Both host driver and firmware make use
 * of this source file. This source file contains functions to manage such a set of circular
 * buffers, but does not contain the code to read or write the data itself into the buffers. It
 * leaves that up to the software layer that uses this file, which can be implemented either using
 * pio or DMA transfers. It also leaves the format of the data that is written and read to a higher
 * layer. Typically the data is in the form of so-called 'message buffers'.
 *
 * Copyright (C) 1999-2016, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id: circularbuf.c 514727 2014-11-12 03:02:48Z $
 */

#include <circularbuf.h>
#include <bcmmsgbuf.h>
#include <osl.h>

#define CIRCULARBUF_READ_SPACE_AT_END(x)		\
			((x->w_ptr >= x->rp_ptr) ? (x->w_ptr - x->rp_ptr) : (x->e_ptr - x->rp_ptr))

#define CIRCULARBUF_READ_SPACE_AVAIL(x)		\
			(((CIRCULARBUF_READ_SPACE_AT_END(x) == 0) && (x->w_ptr < x->rp_ptr)) ? \
				x->w_ptr : CIRCULARBUF_READ_SPACE_AT_END(x))

int cbuf_msg_level = CBUF_ERROR_VAL | CBUF_TRACE_VAL | CBUF_INFORM_VAL;

#ifdef CBUF_DEBUG
#define CBUF_DEBUG_CHECK(x)	x
#else
#define CBUF_DEBUG_CHECK(x)
#endif	

void
circularbuf_init(circularbuf_t *handle, void *buf_base_addr, uint16 total_buf_len)
{
	handle->buf_addr = buf_base_addr;

	handle->depth = handle->e_ptr = HTOL32(total_buf_len);

	
	handle->w_ptr = handle->r_ptr = handle->wp_ptr = handle->rp_ptr = HTOL32(0);
	handle->mb_ring_bell = NULL;
	handle->mb_ctx = NULL;

	return;
}

void
circularbuf_register_cb(circularbuf_t *handle, mb_ring_t mb_ring_func, void *ctx)
{
	handle->mb_ring_bell = mb_ring_func;
	handle->mb_ctx = ctx;
}

#ifdef CBUF_DEBUG
static void
circularbuf_check_sanity(circularbuf_t *handle)
{
	if ((handle->e_ptr > handle->depth) ||
	    (handle->r_ptr > handle->e_ptr) ||
		(handle->rp_ptr > handle->e_ptr) ||
		(handle->w_ptr > handle->e_ptr))
	{
		printf("%s:%d: Pointers are corrupted.\n", __FUNCTION__, __LINE__);
		circularbuf_debug_print(handle);
		ASSERT(0);
	}
	return;
}
#endif 

/**
 * -----------------------------------------------------------------------------
 * Function   : circularbuf_reserve_for_write
 *
 * Description:
 * This function reserves N bytes for write in the circular buffer. The circularbuf
 * implementation will only reserve space in the circular buffer and return
 * the pointer to the address where the new data can be written.
 * The actual write implementation (bcopy/dma) is outside the scope of
 * circularbuf implementation.
 *
 * Input Args :
 *		size - No. of bytes to reserve for write
 *
 * Return Values :
 *		void * : Pointer to the reserved location. This is the address
 *		          that will be used for write (dma/bcopy)
 *
 * -----------------------------------------------------------------------------
 */
void * BCMFASTPATH
circularbuf_reserve_for_write(circularbuf_t *handle, uint16 size)
{
	int16 avail_space;
	void *ret_ptr = NULL;

	CBUF_DEBUG_CHECK(circularbuf_check_sanity(handle));
	ASSERT(size < handle->depth);

	if (handle->wp_ptr >= handle->r_ptr)
		avail_space = handle->depth - handle->wp_ptr;
	else
		avail_space = handle->r_ptr - handle->wp_ptr;

	ASSERT(avail_space <= handle->depth);
	if (avail_space > size)
	{
		
		ret_ptr = CIRCULARBUF_START(handle) + handle->wp_ptr;

		handle->wp_ptr += size;
		return ret_ptr;
	}

	if (handle->wp_ptr >= handle->r_ptr)
	{
		avail_space = handle->r_ptr;
		if (avail_space > size)
		{
			handle->e_ptr  = handle->wp_ptr;
			handle->wp_ptr = size;

			return CIRCULARBUF_START(handle);
		}
	}

	
	return NULL;
}

/**
 * -----------------------------------------------------------------------------
 * Function   : circularbuf_write_complete
 *
 * Description:
 * This function has to be called by the producer end of circularbuf to indicate to
 * the circularbuf layer that data has been written and the write pointer can be
 * updated. In the process, if there was a doorbell callback registered, that
 * function would also be invoked as to notify the consuming party.
 *
 * Input Args :
 *		dest_addr	  : Address where the data was written. This would be the
 *					    same address that was reserved earlier.
 *		bytes_written : Length of data written
 *
 * -----------------------------------------------------------------------------
 */
void BCMFASTPATH
circularbuf_write_complete(circularbuf_t *handle, uint16 bytes_written)
{
	CBUF_DEBUG_CHECK(circularbuf_check_sanity(handle));

	
	if ((handle->w_ptr + bytes_written) >= handle->depth) {
		OSL_CACHE_FLUSH((void *) CIRCULARBUF_START(handle), bytes_written);
		handle->w_ptr = bytes_written;
	} else {
		OSL_CACHE_FLUSH((void *) (CIRCULARBUF_START(handle) + handle->w_ptr),
			bytes_written);
		handle->w_ptr += bytes_written;
	}

	if (handle->mb_ring_bell)
		handle->mb_ring_bell(handle->mb_ctx);
}

void * BCMFASTPATH
circularbuf_get_read_ptr(circularbuf_t *handle, uint16 *available_len)
{
	uint8 *ret_addr;

	CBUF_DEBUG_CHECK(circularbuf_check_sanity(handle));

	
	*available_len = CIRCULARBUF_READ_SPACE_AVAIL(handle);
	if (*available_len == 0)
		return NULL;

	if (CIRCULARBUF_READ_SPACE_AT_END(handle) == 0)
		handle->rp_ptr = 0;

	ret_addr = CIRCULARBUF_START(handle) + handle->rp_ptr;

	handle->rp_ptr = (uint16)(ret_addr - CIRCULARBUF_START(handle) + *available_len);

	ASSERT(*available_len <= handle->depth);

	OSL_CACHE_INV((void *) ret_addr, *available_len);

	return ret_addr;
}

circularbuf_ret_t BCMFASTPATH
circularbuf_read_complete(circularbuf_t *handle, uint16 bytes_read)
{
	CBUF_DEBUG_CHECK(circularbuf_check_sanity(handle));
	ASSERT(bytes_read < handle->depth);

	
	if ((handle->w_ptr < handle->e_ptr) && (handle->r_ptr + bytes_read) > handle->e_ptr)
		handle->r_ptr = bytes_read;
	else
		handle->r_ptr += bytes_read;

	return CIRCULARBUF_SUCCESS;
}

circularbuf_ret_t
circularbuf_revert_rp_ptr(circularbuf_t *handle, uint16 bytes)
{
	CBUF_DEBUG_CHECK(circularbuf_check_sanity(handle));
	ASSERT(bytes < handle->depth);

	handle->rp_ptr -= bytes;

	return CIRCULARBUF_SUCCESS;
}
