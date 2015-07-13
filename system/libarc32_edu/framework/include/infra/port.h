/*
 * INTEL CONFIDENTIAL Copyright 2015 Intel Corporation All Rights Reserved.
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
 * Intel's prior express written permission.
 *
 * No license under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or delivery
 * of the Materials, either expressly, by implication, inducement, estoppel or
 * otherwise.
 *
 * Any license under such intellectual property rights must be express and
 * approved by Intel in writing
 */
#ifndef __INFRA_PORT_H__
#define __INFRA_PORT_H__
#include "os/os.h"
#include "infra/message.h"

#define MAX_PORTS       50


#ifndef INFRA_IS_MASTER
/**
 * Set the port table structure for a remote, shared mem enabled CPU.
 */
void * port_alloc_port_table(int numport);
void port_set_ports_table(void * ptbl);
#endif

/**
 * Allocate a port. The allocation of the port id is global to
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
 * \param port the port to which set the given handler.
 * \param handler the message handler.
 * \param param the private parameter to pass to the handler message
 *              in addition to the message.
 */
void port_set_handler(uint16_t port_id, void (*handler)(struct message *, void *),
		void * param);

/**
 * call the handler function attached to this port.
 * This function shall be called when a message is retrieved from a queue.
 * \param msg the message to process
 */
void port_process_message(struct message * msg);

/**
 * send a message to the destination port set in the message.
 *
 * \param msg the message to send
 */
int port_send_message(struct message * msg);

/**
 * set the port id of the given port.
 * This is done to initialize a port in the context of a slave node.
 *
 * \param port_id the port id to initialize
 */
void port_set_port_id(uint16_t port_id);

/**
 * set the cpu_id of the given port.
 * This is used to know if the internal or ipc messaging needs to be used.
 *
 * \param port_id the port_id to set cpu_id for
 * \param cpu_id the cpu_id to associate with the port
 */
void port_set_cpu_id(uint16_t port_id, uint8_t cpu_id);

/**
 * get the cpu_id of the given port.
 *
 * \param port_id the port to retrieve cpu_id from
 * \return the cpu_id associated with the port
 */
uint8_t port_get_cpu_id(uint16_t port_id);

/**
 * Retrieve the pointer to the global port table.
 * This should only be used by the master in the context of shared memory
 * communications, in order to pass the port table to a slave node with
 * memory shared with the master node.
 *
 * \return the address of the global port table
 */
void * port_get_port_table();

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
 * \param cpu_id the cpu id to set.
 */
void set_cpu_id(uint8_t cpu_id);

/**
 * Get the cpu id for this (port) instance.
 *
 * \return cpu_id of the instance.
 */
uint8_t get_cpu_id(void);

/*
 * Multi cpu APIs.
 */

/**
 * \brief set the callback to be used to send a message to the given cpu.
 *
 * \param cpu_id the cpu id for which we want to set the handler.
 * \param handler the callback used to send a message to this cpu.
 */
void set_cpu_message_sender(uint8_t cpu_id, int (*handler)(struct message * msg));

/**
 * \brief set the callback to be used to request free a message to a cpu.
 *
 * \param cpu_id the cpu id for which we want to set the handler.
 * \param handler the callback used to request message free on.
 */
void set_cpu_free_handler(uint8_t cpu_id, void (*free_handler)(struct message *));

#endif /* __INFRA_PORT_H_ */
