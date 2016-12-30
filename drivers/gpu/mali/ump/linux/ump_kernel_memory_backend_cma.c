/*
 * Based on code originally written by ARM Limited.
 * Copyright (C) 2010-2011 ARM Limited. All rights reserved.
 * Copyright 2015    Martin Ostertag Martin.Ostertag@gmx.de
 * 
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/* needed to detect kernel version specific code */
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
#include <linux/semaphore.h>
#else /* pre 2.6.26 the file was in the arch specific location */
#include <asm/semaphore.h>
#endif

#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/atomic.h>
#include <linux/vmalloc.h>
#include <asm/cacheflush.h>
#include "ump_kernel_common.h"
#include "ump_kernel_memory_backend.h"


extern struct ump_device ump_device;

typedef struct os_allocator
{
	struct semaphore mutex;
	u32 mem_maximum;       	/**< Maximum number of pages to allocate from the CMA */
	u32 mem_allocated; 	/**< Number of pages allocated from the CMA */
} cma_allocator;



static void cma_free(void* ctx, ump_dd_mem * descriptor);
static int cma_allocate(void* ctx, ump_dd_mem * descriptor);
static void cma_memory_backend_destroy(ump_memory_backend * backend);
static u32 cma_stat(struct ump_memory_backend *backend);



/*
 * Create CMA memory backend
 */
ump_memory_backend * ump_cma_memory_backend_create(const int max_allocation)
{
	ump_memory_backend * backend;
	cma_allocator * info;

	info = kmalloc(sizeof(cma_allocator), GFP_KERNEL);
	if (NULL == info)
	{
		return NULL;
	}

	info->mem_maximum = max_allocation;
	info->mem_allocated = 0;

	sema_init(&info->mutex, 1);

	backend = kmalloc(sizeof(ump_memory_backend), GFP_KERNEL);
	if (NULL == backend)
	{
		kfree(info);
		return NULL;
	}

	backend->ctx = info;
	backend->allocate = cma_allocate;
	backend->release = cma_free;
	backend->shutdown = cma_memory_backend_destroy;
	backend->stat = cma_stat;
	backend->pre_allocate_physical_check = NULL;
	backend->adjust_to_mali_phys = NULL;

	return backend;
}



/*
 * Destroy specified OS memory backend
 */
static void cma_memory_backend_destroy(ump_memory_backend * backend)
{
	cma_allocator * info = (cma_allocator*)backend->ctx;

	DBG_MSG_IF(1, 0 != info->mem_allocated, ("%d bytes still in use during shutdown\n", info->mem_allocated));

	kfree(info);
	kfree(backend);
}



/*
 * Allocate UMP memory
 */
static int cma_allocate(void* ctx, ump_dd_mem * descriptor)
{
	u32 left;
	cma_allocator * info;
	int pages_allocated = 0;
	int is_cached;
	dma_addr_t paddr;

	BUG_ON(!descriptor);
	BUG_ON(!ctx);

	info = (cma_allocator*)ctx;
	left = descriptor->size_bytes;
	is_cached = descriptor->is_cached;

	if (down_interruptible(&info->mutex))
	{
		DBG_MSG(1, ("Failed to get mutex in os_free\n"));
		return 0; /* failure */
	}

	descriptor->backend_info = NULL;
	descriptor->nr_blocks = ((left + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1)) >> PAGE_SHIFT;

	DBG_MSG(5, ("Allocating page array. Size: %lu\n", descriptor->nr_blocks * sizeof(ump_dd_physical_block)));

	descriptor->block_array = (ump_dd_physical_block *)vmalloc(sizeof(ump_dd_physical_block) * descriptor->nr_blocks);
	if (NULL == descriptor->block_array)
	{
		up(&info->mutex);
		DBG_MSG(1, ("Block array could not be allocated\n"));
		return 0; /* failure */
	}

	descriptor->cpu_addr = dma_alloc_coherent(/* ump_device.mdev */ NULL, descriptor->size_bytes, &paddr, GFP_KERNEL | GFP_DMA);
	if(! descriptor->cpu_addr)
	{
		up(&info->mutex);
		DBG_MSG(1, ("Failed to allocate needed CMA memory %d bytes\n", descriptor->size_bytes));
		return 0; /* failure */
	}

	if(is_cached)
		DBG_MSG(1, ("caching not supported\n"));

	for (; pages_allocated < descriptor->nr_blocks; pages_allocated++)
	{
		descriptor->block_array[pages_allocated].addr = ((char*)paddr) + (PAGE_SIZE * pages_allocated);
		descriptor->block_array[pages_allocated].size = PAGE_SIZE;
	}

	DBG_MSG(5, ("Alloce for ID:%2d got %d pages\n", descriptor->secure_id,  pages_allocated));

	info->mem_allocated += descriptor->size_bytes;

	DBG_MSG(6, ("%d out of %d bytes now allocated\n", info->mem_allocated, info->mem_maximum));

	up(&info->mutex);

	return 1; /* success*/
}


/*
 * Free specified UMP memory
 */
static void cma_free(void* ctx, ump_dd_mem * descriptor)
{
	cma_allocator * info;
	int i;

	BUG_ON(!ctx);
	BUG_ON(!descriptor);

	info = (cma_allocator*)ctx;

	BUG_ON(descriptor->nr_blocks * PAGE_SIZE > info->mem_allocated);

	if (down_interruptible(&info->mutex))
	{
		DBG_MSG(1, ("Failed to get mutex in cma_free\n"));
		return;
	}

	DBG_MSG(5, ("Releasing %lu OS pages\n", descriptor->nr_blocks));

	info->mem_allocated -= descriptor->nr_blocks * PAGE_SIZE;

	up(&info->mutex);

	dma_free_coherent(ump_device.mdev, descriptor->nr_blocks * PAGE_SIZE, 
			  descriptor->cpu_addr, descriptor->block_array[0].addr);
	vfree(descriptor->block_array);
}


static u32 cma_stat(struct ump_memory_backend *backend)
{
	cma_allocator *info;
	info = (cma_allocator*)backend->ctx;
	return info->mem_allocated;
}
