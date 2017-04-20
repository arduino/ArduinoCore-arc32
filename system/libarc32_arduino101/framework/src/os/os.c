/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "cfw/cfw.h"
#include "os/os.h"
#include "infra/log.h"
#include "misc/util.h"

/*************************    MEMORY   *************************/
// See the balloc.c

/*************************    QUEUES   *************************/

typedef struct queue_ {
    list_head_t lh;
    int count;
    int used;
} q_t;

static q_t q_pool[10];

void queue_put(void *queue, void *msg) {
    q_t * q = (q_t*) queue;
    list_add(&q->lh, (list_t *)msg);
#ifdef DEBUG_OS
    cfw_log("queue_put: %p <- %p\n", queue, msg);
#endif
}

void * queue_wait(void *queue) {
    q_t * q = (q_t*) queue;
    void * elem = (void *)list_get(&q->lh);
#ifdef DEBUG_OS
    cfw_log("queue_wait: %p -> %p\n", queue, elem);
#endif
    return elem;
}

void queue_get_message (T_QUEUE queue, T_QUEUE_MESSAGE* message, int timeout, OS_ERR_TYPE* err) {
    *message = queue_wait(queue);
    *err = E_OS_OK;
}

void queue_send_message (T_QUEUE queue, T_QUEUE_MESSAGE message, OS_ERR_TYPE* err) {
    queue_put(queue, message);
    *err = E_OS_OK;
}

T_QUEUE queue_create(uint32_t  max_size, OS_ERR_TYPE*err) {
    int i, found=0;
    q_t * q = NULL;
    
    for (i = 0; i < ARRAY_SIZE(q_pool); i++)
    {
        q = &q_pool[i];
        if (q->used == 0) 
        {
            q->used = 1;
            found = 1;
            break;
        }
    }
    
    if (!found) 
    {
        return (T_QUEUE)NULL;
    }
    
    list_init(&q->lh);
    q->count = 0;
    return (T_QUEUE) q;
}

void queue_delete(T_QUEUE queue, OS_ERR_TYPE* err) {
    void * element = NULL;
    q_t * q = (q_t*) queue;
    while((element = list_get(&q->lh)) != NULL)
        list_remove(&q->lh, element);
    //cfw_free(q, NULL);
    q->used = 0;
}

/*************************    MUTEXES   *************************/
T_MUTEX mutex_create(OS_ERR_TYPE* err)
{
    return (T_MUTEX) NULL;
}

void mutex_delete(T_MUTEX mutex, OS_ERR_TYPE* err)
{
    return;
}

/* Stub - not implemented */
void mutex_unlock(T_MUTEX mutex, OS_ERR_TYPE* err)
{
    return;
}

/* Stub - not implemented */
OS_ERR_TYPE mutex_lock(T_MUTEX mutex, int timeout)
{
    return E_OS_OK;
}

// FIFO
void fifo_init(struct os_fifo *fifo)
{
    if (fifo->queue == NULL)
    {
        fifo->queue = queue_create (10, NULL);
    }
}

void *fifo_get(struct os_fifo *fifo, int32_t timeout)
{
    return queue_wait(fifo->queue);
}

void fifo_put(struct os_fifo *fifo, void *data)
{
    queue_put(fifo->queue, data);
}

