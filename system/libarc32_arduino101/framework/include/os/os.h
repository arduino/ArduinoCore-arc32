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

#ifndef __OS_H__
#define __OS_H__

#include "os/os_types.h"
#include "panic_api.h" /* To be provided by the platform */
#include "interrupt.h"

/**
 * @defgroup os OS Abstraction Layer
 * Definition of the OS Abstraction Layer API.
 * @{
 */

/**********************************************************
 ************** GENERIC SERVICES***************************
 **********************************************************/
extern void panic (OS_ERR_TYPE err);

/**********************************************************
 ************** OS ABSTRACTION ****************************
 **********************************************************/

/**
 * Delete a queue.
 *
 * Free a whole queue.
 * \param queue - The queue to free.
 */
void queue_delete(T_QUEUE queue, OS_ERR_TYPE* err);

/**
 * \brief Create a message queue.
 *
 *     Create a message queue.
 *     This service may panic if err parameter is NULL and:
 *     -# no queue is available, or
 *     -# when called from an ISR.
 *
 *     Authorized execution levels:  task, fiber.
 *
 *     As for semaphores and mutexes, queues are picked from a pool of
 *     statically-allocated objects.
 *
 * \param maxSize: maximum number of  messages in the queue.
 *     (Rationale: queues only contain pointer to messages)
 *
 * \param err (out): execution status:
 *          -# E_OS_OK : queue was created
 *          -# E_OS_ERR: all queues from the pool are already being used
 *          -# E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 *
 * \return Handler on the created queue.
 *     NULL if all allocated queues are already being used.
 */
extern T_QUEUE queue_create(uint32_t maxSize, OS_ERR_TYPE* err);

/**
 * \brief Read a message from a queue
 *
 *     Read and dequeue a message.
 *     This service may panic if err parameter is NULL and:
 *      -# queue parameter is invalid, or
 *      -# message parameter is NULL, or
 *      -# when called from an ISR.
 *
 *     Authorized execution levels:  task, fiber.
 *
 * \param queue : handler on the queue (value returned by queue_create).
 *
 * \param message (out): pointer to read message.
 *
 * \param timeout: maximum number of milliseconds to wait for the message. Special
 *                values OS_NO_WAIT and OS_WAIT_FOREVER may be used.
 *
 * \param err (out): execution status:
 *          -# E_OS_OK : a message was read
 *          -# E_OS_ERR_TIMEOUT: no message was received
 *          -# E_OS_ERR_EMPTY: the queue is empty
 *          -# E_OS_ERR: invalid parameter
 *          -# E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 */
extern void queue_get_message (T_QUEUE queue, T_QUEUE_MESSAGE* message, int timeout, OS_ERR_TYPE* err);

/**
 * \brief Send a message on a queue
 *
 *     Send / queue a message.
 *     This service may panic if err parameter is NULL and:
 *      -# queue parameter is invalid, or
 *      -# the queue is already full, or
 *
 *     Authorized execution levels:  task, fiber, ISR.
 *
 * \param queue: handler on the queue (value returned by queue_create).
 *
 * \param message (in): pointer to the message to send.
 *
 * \param err (out): execution status:
 *          -# E_OS_OK : a message was read
 *          -# E_OS_ERR_OVERFLOW: the queue is full (message was not posted)
 *          -# E_OS_ERR: invalid parameter
 */
extern void queue_send_message(T_QUEUE queue, T_QUEUE_MESSAGE message, OS_ERR_TYPE* err );

/**
 * \brief Reserves a block of memory
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * This function returns a pointer on the start of
 * a reserved memory block whose size is equal or
 * larger than the requested size.
 *
 * If there is not enough available memory, this
 * function returns a null pointer and sets
 * "err" parameter to E_OS_ERR_NO_MEMORY, or panic
 * if "err" pointer is null.
 *
 * This function may panic if err is null and
 *  - size is null, or
 *  - there is not enough available memory
 *
 * \param size number of bytes to reserve
 *
 *
 * \param err execution status:
 *    E_OS_OK : block was successfully reserved
 *    E_OS_ERR : size is null
 *    E_OS_ERR_NO_MEMORY: there is not enough available
 *              memory
 *
 * \return pointer to the reserved memory block
 *    or null if no block is available
 */
extern void* balloc (uint32_t size, OS_ERR_TYPE* err);

/**
 * \brief Frees a block of memory
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
 * \param buffer pointer returned by malloc
 *
 * \return execution status:
 *    E_OS_OK : block was successfully freed
 *    E_OS_ERR : "buffer" param did not match
 *        any reserved block
 */
extern OS_ERR_TYPE bfree(void* buffer);

/* MUTEX STUB FUNCTIONS */
extern T_MUTEX mutex_create(OS_ERR_TYPE* err);
extern void mutex_delete(T_MUTEX mutex, OS_ERR_TYPE* err );
extern void mutex_unlock (T_MUTEX mutex, OS_ERR_TYPE* err);
extern OS_ERR_TYPE mutex_lock (T_MUTEX mutex, int timeout);

/** @} */

#endif
