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

#ifndef __INFRA_PORT_H__
#define __INFRA_PORT_H__
#include "os/os.h"
#include "infra/message.h"

/**
 * @defgroup ports Communication port management
 * The ports are a messaging feature that allows to communicate between nodes
 * in a transparent maner.
 * @ingroup messaging
 * @{
 */

#define MAX_PORTS       50


#ifndef CONFIG_INFRA_IS_MASTER
/**
 * Set the port table structure for a remote, shared mem enabled CPU.
 */
void * port_alloc_port_table(int numport);
void port_set_ports_table(void * ptbl);
#endif

/**
 * Allocate a port.
 *
 * The allocation of the port id is global to
 * the platform. Only the Main Service Manager is allocating.
 * The slave frameworks send messages to the master fw in order
 * to allocate and get the port.
 * The message handler for this port shall be set by the framework.
 *
 * return the allocated port structure ( filed with the allocated id)
 */
uint16_t port_alloc(void * queue);


/**
 * Sets the message handler for this port.
 *
 * @param port_id the port to which set the given handler.
 * @param handler the message handler.
 * @param param the private parameter to pass to the handler message
 *              in addition to the message.
 */
void port_set_handler(uint16_t port_id, void (*handler)(struct message *, void *),
		void * param);

/**
 * Call the handler function attached to this port.
 *
 * This function shall be called when a message is retrieved from a queue.
 * @param msg the message to process
 */
void port_process_message(struct message * msg);

/**
 * Send a message to the destination port set in the message.
 *
 * @param msg the message to send
 */
int port_send_message(struct message * msg);

/**
 * Set the port id of the given port.
 *
 * This is done to initialize a port in the context of a slave node.
 *
 * @param port_id the port id to initialize
 */
void port_set_port_id(uint16_t port_id);

/**
 * Set the cpu_id of the given port.
 *
 * This is used to know if the internal or ipc messaging needs to be used.
 *
 * @param port_id the port_id to set cpu_id for
 * @param cpu_id the cpu_id to associate with the port
 */
void port_set_cpu_id(uint16_t port_id, uint8_t cpu_id);

/**
 * Get the cpu_id of the given port.
 *
 * @param port_id the port to retrieve cpu_id from
 * @return the cpu_id associated with the port
 */
uint8_t port_get_cpu_id(uint16_t port_id);

/**
 * Retrieve the pointer to the global port table.
 *
 * This should only be used by the master in the context of shared memory
 * communications, in order to pass the port table to a slave node with
 * memory shared with the master node.
 *
 * @return the address of the global port table
 */
void * port_get_port_table();

/**
 * Process the next message in a queue.
 *
 * Gets the first pending message out of the queue and calls the appropriate
 * port handler
 *
 * @param queue The queue to fetch the message from
 *
 * @return the message id or 0
 */
uint16_t queue_process_message(T_QUEUE queue);

/**
 * Process the next message in a queue with a timeout option.
 *
 * Gets the first pending message out of the queue and calls the appropriate
 * port handler, returning on reached timeout.
 *
 * @param queue   The queue to fetch the message from
 * @param timeout millis desired timeout.
 * @param err     Return error gotten during timeout. For example you can check the value if
 *                return value is 0.
 *
 * @return the message id or 0
 */
uint16_t queue_process_message_wait(T_QUEUE queue, uint32_t timeout, OS_ERR_TYPE* err);

/**
 * Process the next message in a queue.
 *
 * Gets the first pending message out of the queue and calls the appropriate
 * port handler
 *
 * \param queue The queue to fetch the message from
 *
 * \return the message id or 0
 *
 */
uint16_t queue_process_message(T_QUEUE queue);

/**
 * Multi CPU support APIs.
 */

/**
 * Set the cpu id for this (port) instance.
 *
 * @param cpu_id the cpu id to set.
 */
void set_cpu_id(uint8_t cpu_id);

/**
 * Get the cpu id for this (port) instance.
 *
 * @return cpu_id of the instance.
 */
uint8_t get_cpu_id(void);

/*
 * Multi cpu APIs.
 */

/**
 * Set the callback to be used to send a message to the given cpu.
 *
 * @param cpu_id the cpu id for which we want to set the handler.
 * @param handler the callback used to send a message to this cpu.
 */
void set_cpu_message_sender(uint8_t cpu_id, int (*handler)(struct message * msg));

/**
 * Set the callback to be used to request free a message to a cpu.
 *
 * @param cpu_id the cpu id for which we want to set the handler.
 * @param free_handler the callback used to request message free on.
 */
void set_cpu_free_handler(uint8_t cpu_id, void (*free_handler)(struct message *));
/**@} */
#endif /* __INFRA_PORT_H_ */
