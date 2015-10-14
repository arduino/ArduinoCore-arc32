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

#ifndef __CFW_SERVICE_H_
#define __CFW_SERVICE_H_

#include "os/os.h"
#include "cfw/cfw.h"
#include "cfw/cfw_internal.h"
#include "infra/message.h"
#include "infra/port.h"

struct service;
struct _port;

/**
 * @defgroup cfw_service CFW Service API
 * @brief CFW Service API.
 * @ingroup cfw
 * @{
 */

/**
 * \struct conn_handle_t
 *
 * This structure is used by the framework to maintain a connection
 * between a client and a service.
 */
typedef struct cfw_conn_ {
	/** Port to reach the client */
	uint16_t client_port;
	/** The service pointer.*/
	struct service * svc;
	/** pointer for the service to store client-specific data. */
	void * priv_data;
	/** Pointer to store the client-side specific handle. */
	void * client_handle;
} conn_handle_t;

/**
 * Internal definition of a service.
 */
typedef struct service {
	uint16_t port_id;
	/** Identifier of the service.*/
	int service_id;
	/** This callback is called when a client connects.
	 * called in the context of the service manager.
	 */
	void (*client_connected)(conn_handle_t *);
	/** This callback is called when a client disconnects.
	 * Called in the context of the service manager.
	 */
	void (*client_disconnected)(conn_handle_t *);
	/* This callback is called when the registered event
         * list is modified.
	 * Called in the context of the service manager.
         */
	void (*registered_events_changed)(conn_handle_t *);
} service_t;

#ifdef CFW_MULTI_CPU_SUPPORT
void _cfw_init_proxy(T_QUEUE queue, void * p, service_t**s, uint16_t svc_mgr_port);
#endif
service_t * cfw_get_service(int service_id);

/**
 * Gets the service manager port id.
 *
 * \return service manager port id
 */
uint16_t cfw_get_service_mgr_port_id(void);

/**
 * Allocate and build a response message for a specific request.
 *
 * \param req the request message
 * \param msg_id the id of the response message
 * \param size the size of the response message
 *
 * \return the allocated and initialized response message
 */
struct cfw_rsp_message * cfw_alloc_rsp_msg(const struct cfw_message *req, int msg_id, int size);

/**
 * Allocate an event message.
 *
 * \param svc the service allocating the event message.
 * \param msg_id the message identifier of the event.
 * \param size the size of the message to be allocated.
 *
 * \return the allocated event message.
 */
struct cfw_message * cfw_alloc_evt_msg(service_t *svc, int msg_id, int size);

/**
 * Allocate an internal event message.
 *
 * \param msg_id the message identifier of the event.
 * \param size the size of the message to be allocated.
 * \param priv the private data passed with the event.
 *
 * \return the allocated event message.
 */
struct cfw_message * cfw_alloc_internal_msg(int msg_id, int size, void * priv);

/**
 * Register events requested by a client.
 * This is called when a client connects to a service.
 *
 * \param handle handle of the connection of the service
 * \param msgId the indication message id that we want to receive
 * \param handle the connection handle for the client to connect
 *          to events.
 */
void _cfw_register_event(conn_handle_t *handle, int msgId);

/**
 * Unregister events registered by a client.
 * This is called when a client disconnects from a service.
 *
 * \param h the connection handle for the client to disconnect
 *          from events.
 */
void _cfw_unregister_event(conn_handle_t * h);

/**
 * register a service to the system.
 * registering a service makes it available to all other services
 * and applications.
 *
 * \param queue the queue on which the service is to be run
 * \param service the service structure to register
 * \param handle_message the service's message handler
 * \param param the parameter passed to message handler
 */
int cfw_register_service(T_QUEUE queue, service_t * service,
        handle_msg_cb_t handle_message, void * param);

/**
 * un-register a service from the system.
 * all other services and applications cannot see it anymore.
 *
 * \param service the service structure to unregister.
 */
int cfw_unregister_service(service_t * service);

/**
 * Send an indication message to the registered clients.
 *
 * \param msg the indication message to send.
 */
void cfw_send_event(struct cfw_message * msg);

/** @} */

#endif /* __CFW_SERVICE_H_ */
