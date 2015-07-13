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
#include "os/os.h"
#include "util/list.h"
#include "infra/port.h"
#include "infra/log.h"
#include "infra/ipc.h"
#include <string.h>

//#define PORT_DEBUG

/**
 * Internal definition of a port structure.
 * External usage is done with a port id integer.
 */
struct port {
    uint16_t id;
    uint8_t cpu_id;
    void *handle_param;
    void * queue;
    void (*handle_message)(struct message *msg, void *param);
};

static uint8_t this_cpu_id = 0;

/* required by port_alloc() and other cfw APIs */
uint8_t get_cpu_id(void)
{
	return this_cpu_id;
}

#ifdef INFRA_IS_MASTER
static struct port ports[MAX_PORTS];
static int registered_port_count = 0;
#else
static struct port * ports = NULL;
#ifndef HAS_SHARED_MEM
static int allocated_port_count = 0;
static uint8_t port_id_to_port[MAX_PORTS] = { 0 };
#endif
void port_set_ports_table(void * ptbl)
{
	ports = (struct port *) ptbl;
}

void * port_alloc_port_table(int numports)
{
	int size = numports*sizeof(struct port);
	ports = balloc(size, NULL);
#ifndef HAS_SHARED_MEM
	allocated_port_count = numports;
	memset(ports, 0, size);
#endif
	return ports;
}
#endif

void * port_get_port_table()
{
	return (void *)&ports[0];
}

#if (!defined HAS_SHARED_MEM && !defined INFRA_IS_MASTER)
static struct port * get_port(uint16_t port_id)
{
	int i;
	if (port_id == 0 || port_id > MAX_PORTS) {
		panic(-1); /*TODO: replace with an assert */
	}

	if (port_id_to_port[port_id - 1] == 0) {
		for (i=0; i < allocated_port_count; i++) {
			int flags = interrupt_lock();
			if (ports[i].id == 0) {
				ports[i].id = port_id;
				port_id_to_port[port_id-1] = i+1;
				interrupt_unlock(flags);
				return &ports[i];
			}
			interrupt_unlock(flags);
		}
		panic(E_OS_ERR_OVERFLOW);
	} else {
		return &ports[port_id_to_port[port_id - 1] - 1];
	}

	panic(-1); /*TODO: replace with an assert */
	return NULL;
}
#else
static struct port * get_port(uint16_t port_id)
{
	if (port_id == 0 || port_id > MAX_PORTS) {
		pr_error(LOG_MODULE_MAIN, "Invalid port: %d", port_id);
		panic(-1); /*TODO: replace with an assert */
	}
	return &ports[port_id - 1];
}
#endif
void port_set_queue(uint16_t port_id, void * queue)
{
	struct port * p = get_port(port_id);
	p->queue = queue;
}

#ifdef INFRA_IS_MASTER
uint16_t port_alloc(void * queue)
{
    struct port * ret = NULL;
    uint32_t flags = interrupt_lock();
    if (registered_port_count < MAX_PORTS) {
        ports[registered_port_count].id = registered_port_count + 1; /* don't use 0 as port.*/
        ports[registered_port_count].cpu_id = get_cpu_id(); /* is overwritten in case of ipc */
        ports[registered_port_count].queue = queue;
#ifdef PORT_DEBUG
        pr_info(LOG_MODULE_MAIN, "%s: port: %p id: %d queue: %p", __func__,
                &ports[registered_port_count], registered_port_count, queue);
#endif
        ret = &ports[registered_port_count];
        registered_port_count++;
    } else {
        panic(E_OS_ERR_NO_MEMORY);
    }
    interrupt_unlock(flags);
    return ret->id;
}
#else
uint16_t port_alloc(void *queue)
{
	struct port * port = NULL;
	int ret = ipc_request_sync_int(IPC_REQUEST_ALLOC_PORT, 0, 0, NULL);
	port = get_port((unsigned int)ret);
#ifndef HAS_SHARED_MEM
	port->id = ret;
#endif
	if (port != NULL) {
		port->queue = queue;
	}
	return port->id;
}
#endif
void port_set_handler(uint16_t port_id, void (*handler)(struct message*, void*), void *param)
{
	struct port * port = get_port(port_id);
	port->handle_message = handler;
	port->handle_param = param;
}

struct message * message_alloc(int size, OS_ERR_TYPE * err)
{
    struct message * msg = (struct message *) balloc(size, err);
    if (msg) {
        memset(msg, 0, size);
    }

	return msg;
}

void port_process_message(struct message * msg)
{
	struct port * p = get_port(msg->dst_port_id);
	if (p->handle_message != NULL) {
		p->handle_message(msg, p->handle_param);
	}
}

void port_set_cpu_id(uint16_t port_id, uint8_t cpu_id)
{
	struct port * p = get_port(port_id);
	p->cpu_id = cpu_id;
}

void port_set_port_id(uint16_t port_id)
{
	struct port * p = get_port(port_id);
	p->id = port_id;
}

uint8_t port_get_cpu_id(uint16_t port_id)
{
	struct port * p = get_port(port_id);
	return p->cpu_id;
}

#ifdef INFRA_MULTI_CPU_SUPPORT
#include "platform.h"

typedef int (*send_msg_t)(struct message * m);

struct ipc_handler {
    send_msg_t send_message;
    void (*free)(struct message * message);
};

struct ipc_handler ipc_handler[NUM_CPU];

void set_cpu_id(uint8_t cpu_id)
{
    this_cpu_id = cpu_id;
}

send_msg_t get_ipc_handler(uint8_t cpu_id) {
    return ipc_handler[cpu_id].send_message;
}

void set_cpu_message_sender(uint8_t cpu_id, send_msg_t handler) {
    ipc_handler[cpu_id].send_message = handler;
}

void set_cpu_free_handler(uint8_t cpu_id, void (*free_handler)(struct message *)) {
    ipc_handler[cpu_id].free = free_handler;
}

int port_send_message(struct message * message)
{
    OS_ERR_TYPE err = 0;
    struct port * port = get_port(MESSAGE_DST(message));
    if (port == NULL) {
        pr_error(LOG_MODULE_MAIN, "Invalid destination port (%d)", MESSAGE_DST(message));
        return E_OS_ERR;
    }
    if (port->cpu_id == get_cpu_id()) {
#ifdef PORT_DEBUG
        pr_info(LOG_MODULE_MAIN, "Sending message %p to port %p(q:%p) ret: %d", message, port, port->queue, err);
#endif
        queue_send_message(port->queue, message, &err);
        return err;
    } else {
#ifdef PORT_DEBUG
        pr_info(LOG_MODULE_MAIN, "Remote port ! using: %p handler", ipc_handler[port->cpu_id].send_message);
#endif
        return ipc_handler[port->cpu_id].send_message(message);
    }
}

void message_free(struct message * msg)
{
    struct port * port = get_port(MESSAGE_SRC(msg));
    pr_debug(LOG_MODULE_MAIN, "free message %p: port %p[%d] this %d id %d",
            msg, port, port->cpu_id, get_cpu_id(), MESSAGE_SRC(msg));
    if (port->cpu_id == get_cpu_id()) {
        bfree(msg);
    } else {
        ipc_handler[port->cpu_id].free(msg);
    }
}

#else /* Single CPU support */

int port_send_message(struct message * msg)
{
	struct port * port = get_port(MESSAGE_DST(msg));
	OS_ERR_TYPE err;
	queue_send_message(port->queue, msg, &err);
	return err;
}

void message_free(struct message * msg)
{
	bfree(msg);
}
#endif

uint16_t queue_process_message(T_QUEUE queue)
{
	T_QUEUE_MESSAGE m;
	OS_ERR_TYPE err;
	struct message * message;
	uint16_t id = 0;
	queue_get_message(queue, &m, OS_NO_WAIT, &err);
	message = (struct message *) m;
	if ( message != NULL && err == E_OS_OK) {
		id = MESSAGE_ID(message);
		port_process_message(message);
	}
	return id;
}
