
/**
 * @ingroup os_mem_alloc Memory Allocation
 * Defines balloc and bfree functions for dynamic memory allocation.
 * @{
 */

#include <stdio.h>

#include "os/os.h"
#include "infra/log.h"
#include "dccm_alloc.h"

#ifdef CONFIG_MEMORY_POOLS_BALLOC_TRACK_OWNER
#include "misc/printk.h"
#include <string.h>
#endif
extern void panic(int x);
#define BITS_PER_U32 (sizeof(uint32_t) * 8)

/** If defined, allow to use a block larger than required when all smaller blocks are already reserved */
#define MALLOC_ALLOW_OUTCLASS

/** Descriptor for a memory pool */
typedef struct {
    uint32_t *track;        /** block allocation tracker */
    uint32_t start;         /** start address of the pool */
    uint32_t end;           /** end address of the pool */
    uint16_t count;         /** total number of blocks within the pool */
    uint16_t size;          /** size of each memory block within the pool */
#ifdef CONFIG_MEMORY_POOLS_BALLOC_STATISTICS
#ifdef CONFIG_MEMORY_POOLS_BALLOC_TRACK_OWNER
    uint32_t **owners;
#endif
    uint32_t max;           /** maximum number of allocated blocks at the same time */
    uint32_t cur;           /** current number of allocated blocks */
    uint32_t sum;           /** Cumulative size in bytes */
    uint32_t nbrs;          /** Cumulative block allocated */
#endif
}T_POOL_DESC;

/**********************************************************
************** Private variables  ************************
**********************************************************/

#ifdef CONFIG_MEMORY_POOLS_BALLOC_STATISTICS

/** Allocate the memory blocks and tracking variables for each pool */
#ifdef CONFIG_MEMORY_POOLS_BALLOC_TRACK_OWNER
#define DECLARE_MEMORY_POOL(index, size, count) \
    uint8_t mblock_ ## index[count][size] __aligned(4); \
    uint32_t mblock_alloc_track_ ## index[count / BITS_PER_U32 + 1] = { 0 }; \
    uint32_t *mblock_owners_ ## index[count] = { 0 };
#else
#define DECLARE_MEMORY_POOL(index, size, count) \
    uint8_t mblock_ ## index[count][size] __aligned(4); \
    uint32_t mblock_alloc_track_ ## index[count / BITS_PER_U32 + \
                          1] = { 0 };
#endif

#include "memory_pool_list.def"

/** Pool descriptor definition */
T_POOL_DESC mpool[] =
{
#ifdef CONFIG_MEMORY_POOLS_BALLOC_TRACK_OWNER
#define DECLARE_MEMORY_POOL(index, size, count) \
    { \
/* T_POOL_DESC.track */ mblock_alloc_track_ ## index, \
/* T_POOL_DESC.start */ (uint32_t)mblock_ ## index, \
/* T_POOL_DESC.end */ (uint32_t)mblock_ ## index + count * size, \
/* T_POOL_DESC.count */ count, \
/* T_POOL_DESC.size */ size, \
/* T_POOL_DESC.owners */ mblock_owners_ ## index, \
/* T_POOL_DESC.max */ 0, \
/* T_POOL_DESC.cur */ 0, \
/* T_POOL_DESC.sum */ 0, \
/* T_POOL_DESC.nbrs */ 0 \
    },
#else
#define DECLARE_MEMORY_POOL(index, size, count) \
    { \
/* T_POOL_DESC.track */ mblock_alloc_track_ ## index, \
/* T_POOL_DESC.start */ (uint32_t)mblock_ ## index, \
/* T_POOL_DESC.end */ (uint32_t)mblock_ ## index + count * size, \
/* T_POOL_DESC.count */ count, \
/* T_POOL_DESC.size */ size, \
/* T_POOL_DESC.max */ 0, \
/* T_POOL_DESC.cur */ 0, \
/* T_POOL_DESC.sum */ 0, \
/* T_POOL_DESC.nbrs */ 0 \
    },
#endif

#include "memory_pool_list.def"
};


#else

/** Allocate the memory blocks and tracking variables for each pool */
#define DECLARE_MEMORY_POOL(index, size, count) \
    uint32_t mblock_alloc_track_ ## index[count / BITS_PER_U32 + 1] = { 0 };

#include "memory_pool_list.def"


/** Pool descriptor definition */
T_POOL_DESC mpool [] =
{
#define DECLARE_MEMORY_POOL(index, size, count) \
    { \
/* T_POOL_DESC.track */ mblock_alloc_track_ ## index, \
/* T_POOL_DESC.start */ 0, \
/* T_POOL_DESC.end */ 0, \
/* T_POOL_DESC.count */ count, \
/* T_POOL_DESC.size */ size \
    },

#include "memory_pool_list.def"
};



#endif


/** Number of memory pools */
#define NB_MEMORY_POOLS   (sizeof(mpool) / sizeof(T_POOL_DESC))

/**********************************************************
************** Private functions  ************************
**********************************************************/

/**
 * Return the next free block of a pool and
 *   mark it as reserved/allocated.
 *
 * @param pool index of the pool in mpool
 *
 * @return allocated buffer or NULL if none is
 *   available
 */
static void *memblock_alloc(uint32_t pool)
{
    uint16_t block;
    uint32_t flags = interrupt_lock();//irq_lock();

    for (block = 0; block < mpool[pool].count; block++) {
        if (((mpool[pool].track)[block / BITS_PER_U32] & 1 <<
             (BITS_PER_U32 - 1 - (block % BITS_PER_U32))) == 0) {
            (mpool[pool].track)[block / BITS_PER_U32] =
                (mpool[pool].track)[block / BITS_PER_U32] |
                (1 << (BITS_PER_U32 - 1 - (block % BITS_PER_U32)));
#ifdef CONFIG_MEMORY_POOLS_BALLOC_STATISTICS
            mpool[pool].cur = mpool[pool].cur + 1;
#ifdef CONFIG_MEMORY_POOLS_BALLOC_TRACK_OWNER
            /* get return address */
            uint32_t ret_a = (uint32_t)__builtin_return_address(0);
            mpool[pool].owners[block] =
                (uint32_t *)(((ret_a & 0xFFFF0U) >> 4) |
                         ((get_uptime_ms() & 0xFFFF0) << 12));
#endif
            if (mpool[pool].cur > mpool[pool].max)
                mpool[pool].max = mpool[pool].cur;
#endif
            interrupt_unlock(flags);//irq_unlock(flags);
            return (void *)(mpool[pool].start +
                    mpool[pool].size * block);
        }
    }
    //irq_unlock(flags);
    interrupt_unlock(flags);
    return NULL;
}



/**
 * Free an allocated block from a pool.
 *
 * @param pool index of the pool in mpool
 *
 * @param ptr points to the start of the block
 *     to free
 *
 */
static void memblock_free(uint32_t pool, void *ptr)
{
    uint16_t block;
    uint32_t flags;

    block = ((uint32_t)ptr - mpool[pool].start) / mpool[pool].size;
    if (block < mpool[pool].count) {
        flags = interrupt_lock();//irq_lock();
        (mpool[pool].track)[block / BITS_PER_U32] &=
            ~(1 << (BITS_PER_U32 - 1 - (block % BITS_PER_U32)));
        interrupt_unlock(flags);//irq_unlock(flags);
#ifdef CONFIG_MEMORY_POOLS_BALLOC_STATISTICS
        mpool[pool].cur = mpool[pool].cur - 1;
#endif
    } else {
        pr_debug(
            LOG_MODULE_UTIL,
            "ERR: memblock_free: ptr 0x%X is not within pool %d [0x%X , 0x%X]",
            ptr, pool, mpool[pool].start, mpool[pool].end);
    }
}




/**
 * Test if a block is allocated.
 *
 * @param pool index of the pool in mpool
 *
 * @param ptr points to the start of the block
 *
 * @return true if the block is allocated/reserved,
 *   false if the block is free
 *
 */
static bool memblock_used(uint32_t pool, void *ptr)
{
    uint16_t block;

    block = ((uint32_t)ptr - mpool[pool].start) / mpool[pool].size;
    if (block < mpool[pool].count) {
        if (((mpool[pool].track)[block / BITS_PER_U32] &
             (1 << (BITS_PER_U32 - 1 - (block % BITS_PER_U32)))) != 0)
            return true;
    }
    return false;
}


#ifdef CONFIG_MEMORY_POOLS_BALLOC_STATISTICS
#ifdef CONFIG_MEMORY_POOLS_BALLOC_TRACK_OWNER

#define PRINT_METHOD_PRINTK   0
#define PRINT_METHOD_TCMD_RSP 1
#define PRINT_METHOD_PR_INFO  2
#define PRINT_POOL(method, str, ctx) \
    do { \
        if (method == PRINT_METHOD_PRINTK) { \
            printk("%s\n", str); } \
        else if (method == PRINT_METHOD_TCMD_RSP) { \
            TCMD_RSP_PROVISIONAL(((struct tcmd_handler_ctx *)ctx), \
                         str); } \
        else if (method == PRINT_METHOD_PR_INFO) { \
            pr_info(LOG_MODULE_UTIL, str); \
            local_task_sleep_ms(100); \
        } \
    } while (0);
static void print_pool(int method, void *ctx)
{
    char tmp[128];
    uint32_t pool;
    uint32_t average;
    uint16_t block;
    uint8_t str_count;
    char *cur = tmp;

    for (pool = 0; pool < NB_MEMORY_POOLS; pool++) {
        str_count = 0;
        average = 0;
        if (mpool[pool].nbrs)
            average = mpool[pool].sum / mpool[pool].nbrs;
        snprintf(
            tmp, sizeof(tmp),
            "\npool %-4d bytes count:%-2d cur:%-2d max:%-2d avg: %-3d \n",
            mpool[pool].size,
            mpool[pool].count,
            mpool[pool].cur,
            mpool[pool].max,
            average);
        PRINT_POOL(method, tmp, ctx);

        memset(tmp, 0, sizeof(tmp));
        str_count = 0;

        for (block = 0; block < mpool[pool].count; block++) {
            if (((mpool[pool].track)[block / BITS_PER_U32] & 1 <<
                 (BITS_PER_U32 - 1 -
                  (block % BITS_PER_U32)))) {
                if (str_count == 0) {
                    cur = tmp;
                    PRINT_POOL(method, " owners:", ctx);
                }
                str_count++;
                snprintf(cur, 7, " T%04u",
                     (((uint32_t)mpool[pool].owners[
                           block]) &
                      (uint32_t)0xFFFF0000) >> 16);
                cur += 6;
                snprintf(cur, 6, "C%04x",
                     (((uint32_t)mpool[pool].owners[
                           block]) &
                      (uint32_t)0xFFFF));
                cur += 5; /* hack to print the owner */
                if (str_count % 4 == 0) {
                    PRINT_POOL(method, tmp, ctx);
                    memset(tmp, 0, sizeof(tmp));
                    cur = tmp;
                }
            }
        }
        if (str_count % 4)
            PRINT_POOL(method, tmp, ctx);
    }
    PRINT_POOL(method, "*** END", ctx);
}

#endif
#endif


/**********************************************************
************** Exported functions ************************
**********************************************************/

/*----- Initialization  */

/**
 * Initialize the resources used by the framework's memory allocation services
 *
 * IMPORTANT : This function must be called during the initialization
 *             of the OS abstraction layer.
 *             This function shall only be called once after reset.
 */
void os_abstraction_init_malloc(void)
{
  int indx;
  uint32_t bufSize;

  for (indx=0; indx < NB_MEMORY_POOLS; indx++) {
    bufSize = mpool[indx].count * mpool[indx].size;
    mpool[indx].start = (uint32_t)dccm_memalign((uint16_t)bufSize);
    mpool[indx].end = mpool[indx].start + bufSize;
  }
}

/**
 * Reserves a block of memory.
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * This function returns a pointer on the start of
 * a reserved memory block whose size is equal or
 * larger than the requested size.
 *
 * The returned pointer shall be null if the function
 * fails.
 *
 * This function may panic if err is null and
 *  - size is null or bigger than allowed, or
 *  - there is not enough available memory
 *
 * @param size number of bytes to reserve
 *
 *
 * @param err execution status:
 *    E_OS_OK : block was successfully reserved
 *    E_OS_ERR : size is null
 *    E_OS_ERR_NO_MEMORY: there is not enough available
 *              memory
 *    E_OS_ERR_NOT_ALLOWED : size is bigger than the
 *         biggest block size defined in os_config.h
 *
 * @return pointer to the reserved memory block
 *    or null if no block is available
 */
void *balloc(uint32_t size, OS_ERR_TYPE *err)
{
    OS_ERR_TYPE localErr = E_OS_OK;
    void *buffer = NULL;
    uint8_t poolIdx;

    if (size > 0) {
        /* find the first block size greater or equal to requested size */
        poolIdx = 0;
        while (poolIdx < NB_MEMORY_POOLS &&
               (size > mpool[poolIdx].size))
            poolIdx++;

        /* reserve the block */
        if (poolIdx < NB_MEMORY_POOLS) {
#ifdef MALLOC_ALLOW_OUTCLASS
            /* loop until an available (maybe larger) block is found */
            do {
                if (size <= mpool[poolIdx].size) { /* this condition may be false if pools are not sorted according to block size */
#endif
            buffer = memblock_alloc(poolIdx);
#ifdef CONFIG_MEMORY_POOLS_BALLOC_STATISTICS
            if ((buffer != NULL) &&
                ((poolIdx == 0) ||
                 (size > mpool[poolIdx - 1].size))) {
                mpool[poolIdx].nbrs += 1;
                mpool[poolIdx].sum += size;
            }
#endif
#ifdef MALLOC_ALLOW_OUTCLASS
        }

        if (NULL == buffer)
            poolIdx++;
    }
    while ((poolIdx < NB_MEMORY_POOLS) && (NULL == buffer)) ;
#endif
            if (NULL == buffer) { /* All blocks of relevant size are already reserved */
                pr_debug(LOG_MODULE_UTIL,
                     "Attempt to allocate %d bytes failed",
                     size);
                localErr = E_OS_ERR_NO_MEMORY;
            }
        } else { /* Configuration does not define blocks large enough for the requested size */
            localErr = E_OS_ERR_NOT_ALLOWED;
        }
    } else { /* invalid function parameter */
        localErr = E_OS_ERR;
    }

    /* set err or panic if err == NULL and localErr != E_OS_OK */
    if (err != NULL) {
        *err = localErr;
    } else {
        if (localErr != E_OS_OK) {
#ifdef CONFIG_MEMORY_POOLS_BALLOC_TRACK_OWNER
#ifdef CONFIG_NANOKERNEL
            /* disable stack checking */
            uint32_t status32 = _arc_v2_aux_reg_read(
                _ARC_V2_STATUS32);
            status32 &= ~(_ARC_V2_STATUS32_SC);
            __asm__ volatile ("kflag %0" : : "ir" (status32));
#endif
            if (localErr == E_OS_ERR_NO_MEMORY)
                print_pool(PRINT_METHOD_PRINTK, NULL);
#endif
            panic(localErr);
        }
    }

    return buffer;
}

/**
 * Frees a block of memory.
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * This function frees a memory block that was
 * reserved by malloc.
 *
 * The "buffer" parameter must point to the
 * start of the reserved block (i.e. it shall
 * be a pointer returned by malloc).
 *
 * @param buffer pointer returned by malloc
 *
 * @return execution status:
 *    E_OS_OK : block was successfully freed
 *    E_OS_ERR : "buffer" param did not match
 *        any reserved block
 */
OS_ERR_TYPE bfree(void *buffer)
{
    OS_ERR_TYPE err = E_OS_ERR;
    uint8_t poolIdx;

    /* find which pool the buffer was allocated from */
    poolIdx = 0;
    while ((NULL != buffer) && (poolIdx < NB_MEMORY_POOLS)) {
        /* check if buffer is within mpool[poolIdx] */
        if (((uint32_t)buffer >= mpool[poolIdx].start) &&
            ((uint32_t)buffer < mpool[poolIdx].end)) {
            if (false != memblock_used(poolIdx, buffer)) {
                memblock_free(poolIdx, buffer);
                err = E_OS_OK;
            }
            /* else: buffer is not marked as used, keep err = E_OS_ERR */
            else {
                pr_debug(
                    LOG_MODULE_UTIL,
                    "ERR: memory_free: buffer %p is already free\n",
                    buffer);
            }
            buffer = NULL;  /* buffer was found in the pools, end the loop */
        } else {                /* buffer does not belong to mpool[poolIdx], go to the next one */
            poolIdx++;
        }
    }
    return err;
}


#ifdef CONFIG_DBG_POOL_TCMD

void tcmd_pool(int argc, char *argv[], struct tcmd_handler_ctx *ctx)
{
#ifdef CONFIG_QUARK
    /* Display with TCMD response on Quark */
    print_pool(PRINT_METHOD_TCMD_RSP, ctx);
#endif
#ifdef CONFIG_ARC
    /* Display with pr_info on ARC to avoid message overflow and panic */
    print_pool(PRINT_METHOD_PR_INFO, ctx);
#endif
    TCMD_RSP_FINAL(ctx, "");
}


DECLARE_TEST_COMMAND_ENG(dbg, pool, tcmd_pool);

#endif

/** @} */
