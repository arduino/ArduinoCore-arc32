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

#ifndef __CFW_CLIENT_H__
#define __CFW_CLIENT_H__

#include "cfw/cfw.h"

/**
 * @defgroup cfw_client CFW Client API
 * CFW Client API, i.e for users of services.
 * @ingroup cfw
 * @{
 */

/**
 * Create a handle to the component framework.
 * This handle is to be used for all other requests
 * to the component framework
 *
 * Implementation is different in the master and the slave contexts.
 * The master context will be pseudo-synchronous, while the slave
 * implementation will actually pass a message to the master context
 * in order to register a new client.
 *
 * \param queue pointer to service queue
 * \param cb the callback that will be called for each message reception
 * \param param the param passed along with the message to the callback
 */
cfw_handle_t cfw_init(void * queue, handle_msg_cb_t cb, void * param);


/**
 * Allocate a request message for a service.
 * This will fill the needed common message fields needed to interact
 * with a service.
 */
struct cfw_message * cfw_alloc_message_for_service(const svc_client_handle_t * h,
		int msg_id, int msg_size, void * priv);

/** Helper macro to allocate a message for a service */
#define CFW_ALLOC_FOR_SVC(t, m, h, i, e, p) \
	t *m = (t *)cfw_alloc_message_for_service(h, i, sizeof(t) + (e), p)

/**
 * Open a connection to the specified service.
 * The connection handle is returned in the OPEN_CONNECTION
 * confirmation message.
 *
 * \msc
 * Client,"FW API","Service Manager";
 *
 * Client=>"FW API" [label="cfw_open_connection"];
 * "FW API"->"Service Manager" [label="CFW_OPEN_SERVICE REQ", URL="\ref cfw_open_conn_req_msg_t", ID="1"];
 * Client<<"FW API" ;
 * Client<-"Service Manager" [label="CFW_OPEN_SERVICE CNF", URL="\ref cfw_open_conn_rsp_msg_t", ID="2"];

 * \endmsc
 *
 * \param handle the handle to the component framework, as returned by \ref cfw_init
 * \param service_id the unique service identifier.
 * \param param pointer to private data of the service.
 * \return 0 if request succeeded and != 0 if error occurred.
 */
int cfw_open_service(cfw_handle_t handle, int service_id, void *param);

/**
 * Closes a connection.
 *
 * \param handle the client handle representing the connection
 * \param priv an opaque data passed back in the close response message
 * \ return 0 if request succeeded and !=0 otherwise
 */
int cfw_close_service(const svc_client_handle_t *handle, void *priv);

/**
 * Register to service events.
 *
 * \param handle the service connection handle, as returned in the \ref cfw_open_conn_rsp_msg_t
 * \param msg_ids the array of event message ids to register to.
 * \param size the size of the msg_ids array.
 * \param param the void * private param that will be returned in the \ref cfw_register_evt_rsp_msg_t
 */
int cfw_register_events(const svc_client_handle_t * handle, int * msg_ids, int size, void*param);

/**
 * Register to service availability events.
 * Whenever a service is registered, the clients that registered to service_available will
 * receive a cfw_svc_available_evt_msg_t message whenever a service is available.
 *
 * \param handle the framework handle.
 * \param service_id the unique service identifier.
 * \param param the private param sent back with the response message.
 */
int cfw_register_svc_available(cfw_handle_t handle, int service_id, void *param);

/** @} */

#endif
