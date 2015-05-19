/*  INTEL CONFIDENTIAL Copyright 2014-2015 Intel Corporation All Rights Reserved.
  *
  * The source code contained or described herein and all documents related to
  * the source code ("Material") are owned by Intel Corporation or its suppliers
  * or licensors.
  * Title to the Material remains with Intel Corporation or its suppliers and
  * licensors.
  * The Material contains trade secrets and proprietary and confidential information
  * of Intel or its suppliers and licensors. The Material is protected by worldwide
  * copyright and trade secret laws and treaty provisions.
  * No part of the Material may be used, copied, reproduced, modified, published,
  * uploaded, posted, transmitted, distributed, or disclosed in any way without
  * Intel?s prior express written permission.
  *
  * No license under any patent, copyright, trade secret or other intellectual
  * property right is granted to or conferred upon you by disclosure or delivery
  * of the Materials, either expressly, by implication, inducement, estoppel or
  * otherwise.
  *
  * Any license under such intellectual property rights must be express and
  * approved by Intel in writing
  *
  ******************************************************************************/

/**
 * \defgroup os OS API
 * \brief Definition of the OS API.
 * @{
 */

/**
 * \addtogroup os
 * @{
 * \defgroup os_al OS Abstraction Layer
 * @{
 * \brief Definition of the OS abstraction layer API.
 */

#ifndef __OS_H__
#define __OS_H__

#include "os/os_types.h"
#include "panic_api.h" /* To be provided by the platform */
#include "interrupt.h"

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
 * \brief Get the current tick in ms
 *
 *     Return the current tick converted in milliseconds
 *
 *     Authorized execution levels:  task, fiber, ISR
 *
 * \return current tick converted in milliseconds
 */
extern uint32_t get_time_ms (void);

/**
 * \brief Get the current tick in us
 *
 *     Return the current tick converted in microseconds
 *
 *     Authorized execution levels:  task, fiber, ISR
 *
 * \return current tick converted in microseconds
 */
extern uint64_t get_time_us (void);

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

/**
 * \brief Initialize the OS abstraction layer
 *
 */
extern void os_init (void);

#endif

/**@} @} @}*/
