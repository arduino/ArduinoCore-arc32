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

#ifndef _CFW_H_
#define _CFW_H_

#include <stdio.h>
#include <stdlib.h>

#include "os/os.h"
#include "util/list.h"
#include "infra/port.h"
#include "infra/message.h"

/**
 * @defgroup cfw Component Framework
 * The Component Framework is the main building block in the thunderdome
 * platform.
 *
 * @{
 * @defgroup services Services
 * Component framework services definition
 * @ingroup cfw
 * @}
 * @{
 */

struct cfw_message {
	struct message m;
	
    /* The two following fields are specific to framework
     * messages, and could be avoided / re-used for other
     * types of messages.
     */

	/** The belonging connection of the message */
    void * conn;
	/** The private data passed with the request. */
	void * priv;
};

#define CFW_MESSAGE_ID(msg)     MESSAGE_ID(&(msg)->m)
#define CFW_MESSAGE_SRC(msg)    MESSAGE_SRC(&(msg)->m)
#define CFW_MESSAGE_DST(msg)    MESSAGE_DST(&(msg)->m)
#define CFW_MESSAGE_LEN(msg)    MESSAGE_LEN(&(msg)->m)
#define CFW_MESSAGE_TYPE(msg)   MESSAGE_TYPE(&(msg)->m)
#define CFW_MESSAGE_CONN(msg)   (msg)->conn
#define CFW_MESSAGE_PRIV(msg)   (msg)->priv
#define CFW_MESSAGE_HEADER(msg) (&(msg)->m)

struct cfw_rsp_message {
	/** Common message header */
	struct cfw_message header;
	/** response status code.*/
	int	status;
};

/**
 * Message handler definition.
 */
typedef void (*handle_msg_cb_t)(struct cfw_message *, void *);

/**
 * Framework client handle.
 *
 * This type is used to communicate with the service manager.
 */
typedef void * cfw_handle_t;

/**
 * \struct svc_client_handle_t
 * \brief the structure used to manage a connection between a client
 * and a service.
 *
 * This structure is returned in \ref cfw_open_conn_req_msg_t in
 * the client_handle field
 */
typedef struct svc_client_handle_ {
	/** Port to attain the service. */
	uint16_t port;
	/** Service id */
	int service_id;
	/** Framework handle. */
	cfw_handle_t cfw_handle;
	/** Pointer to store the server-side connection handle.
	 * Passed in the conn field of struct cfw_message for request messages
	 */
	void * server_handle;
} svc_client_handle_t;

struct cfw_message * cfw_alloc_message(int size, OS_ERR_TYPE * err);

/**
 * \brief free a message.
 *
 * This function will take care to send the freeing request to
 * the core that allocated the message. (based on the source
 * port)
 *
 * \param ptr the message to be freed.
 */
void cfw_msg_free(struct cfw_message * msg);

/**
 * Create a copy of a given message.
 *
 * \param msg the message to be cloned.
 * \return the cloned message
 */
struct cfw_message * cfw_clone_message(struct cfw_message * msg);


/**
 * Send a message.
 * The message should be filed with the destination port,
 * source port, message identifier, etc...
 *
 * \param msg the parameter to send.
 */
int _cfw_send_message(struct cfw_message * msg);

/**
 * This macro conveniently casts the parameter to a message header pointer.
 */
#define cfw_send_message(_msg_) _cfw_send_message((struct cfw_message*) (_msg_))

/**
 * get the local identifier of the service.
 */
int _find_service(int);

/**
 * Get the port id of the given service.
 */
int _cfw_get_service_port(int);

/** @} */

#endif /* #ifndef _CFW_H_ */
