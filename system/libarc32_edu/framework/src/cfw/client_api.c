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
#include "cfw/cfw_client.h"
#include "cfw/cfw_internal.h"
#include "cfw/cfw_messages.h"

/**
 * \file client_api.c
 *
 * Implementation of the client interface.
 *
 */

int cfw_open_service(cfw_handle_t handle, int service_id, void * priv) {
    cfw_open_conn_req_msg_t * msg = (cfw_open_conn_req_msg_t*)
    	cfw_alloc_message(sizeof(*msg), NULL);
    CFW_MESSAGE_ID(&msg->header) = MSG_ID_CFW_OPEN_SERVICE;
    CFW_MESSAGE_LEN(&msg->header) = sizeof(*msg);
    CFW_MESSAGE_DST(&msg->header) = cfw_get_service_mgr_port_id();
    CFW_MESSAGE_SRC(&msg->header) = ((_cfw_handle_t*)handle)->client_port_id;
    CFW_MESSAGE_TYPE(&msg->header) = TYPE_REQ;

    svc_client_handle_t * svch = (svc_client_handle_t*)balloc(sizeof(*svch), NULL);
    svch->port = CFW_MESSAGE_SRC(&msg->header);
    svch->cfw_handle = handle;
    svch->server_handle = NULL;
    svch->service_id = service_id;

    msg->service_id = service_id;
    msg->client_handle = svch;
    msg->client_cpu_id = port_get_cpu_id(CFW_MESSAGE_SRC(&msg->header));
    msg->header.priv = priv;
    msg->header.conn = NULL;
    cfw_send_message(msg);
    return 0;
}

int cfw_close_service(const svc_client_handle_t *handle, void *priv)
{
    cfw_close_conn_req_msg_t *msg = (cfw_close_conn_req_msg_t*)
          cfw_alloc_message(sizeof(*msg), NULL);
    CFW_MESSAGE_ID(&msg->header) = MSG_ID_CFW_CLOSE_SERVICE;
    CFW_MESSAGE_LEN(&msg->header) = sizeof(*msg);
    CFW_MESSAGE_DST(&msg->header) = cfw_get_service_mgr_port_id();
    CFW_MESSAGE_SRC(&msg->header) =
        ((_cfw_handle_t*)handle->cfw_handle)->client_port_id;
    msg->header.priv = priv;
    msg->header.conn = handle->server_handle;
    msg->service_id = handle->service_id;
    msg->inst = NULL;
    cfw_send_message(msg);
    return 0;
}

int cfw_register_events(const svc_client_handle_t * h, int * msg_ids, int size, void * priv) {
    int msg_size = sizeof(struct cfw_message) + sizeof(int) * (size+1);
    int i;
    struct cfw_message * msg = cfw_alloc_message_for_service(h, MSG_ID_CFW_REGISTER_EVT, msg_size,
        priv);
    CFW_MESSAGE_DST(msg) = cfw_get_service_mgr_port_id();
    ((int *) &msg[1])[0] = size;
    for (i = 0; i < size; i++) {
        ((int *) (&msg[1]))[i+1] = msg_ids[i];
    }
    cfw_send_message(msg);
    return 0;
}

int cfw_register_svc_available(cfw_handle_t handle, int service_id, void *priv)
{
    cfw_register_svc_avail_req_msg_t * msg =
        (cfw_register_svc_avail_req_msg_t *) cfw_alloc_message(sizeof(*msg), NULL);
    CFW_MESSAGE_ID(&msg->header) = MSG_ID_CFW_REGISTER_SVC_AVAIL;
    CFW_MESSAGE_LEN(&msg->header) = sizeof(*msg);
    CFW_MESSAGE_SRC(&msg->header) = ((_cfw_handle_t*)handle)->client_port_id;
    CFW_MESSAGE_DST(&msg->header) = cfw_get_service_mgr_port_id();
    CFW_MESSAGE_TYPE(&msg->header) = TYPE_REQ;
    msg->header.priv = priv;
    msg->header.conn = NULL;
    msg->service_id = service_id;
    cfw_send_message(msg);
    return 0;
}

struct cfw_message * cfw_alloc_message_for_service(const svc_client_handle_t * h, int msg_id, int msg_size,
        void * priv) {
    struct cfw_message * msg = (struct cfw_message*) cfw_alloc_message(msg_size, NULL);
    CFW_MESSAGE_ID(msg) = msg_id;
    CFW_MESSAGE_LEN(msg) = msg_size;
    CFW_MESSAGE_SRC(msg) = ((_cfw_handle_t*)h->cfw_handle)->client_port_id;
    CFW_MESSAGE_DST(msg) = h->port;
    CFW_MESSAGE_TYPE(msg) = TYPE_REQ;
    msg->priv = priv;
    msg->conn = h->server_handle;
    return msg;
}

struct cfw_message * cfw_alloc_message(int size, OS_ERR_TYPE * err)
{
	struct cfw_message * msg = (struct cfw_message *) message_alloc(size, err);

	return msg;
}
