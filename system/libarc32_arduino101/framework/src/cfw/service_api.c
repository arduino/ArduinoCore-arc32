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

#include "util/list.h"
#include "os/os.h"
#include "infra/message.h"
#include "infra/port.h"
#include "infra/log.h"

#include "cfw/cfw.h"
#include "cfw/cfw_debug.h"
#include "cfw/cfw_messages.h"
#include "cfw/cfw_internal.h"
#include "cfw/cfw_service.h"
#include "cfw_private.h"

/**
 * \file service_api.c implementation of the service_manager API
 */

/* Structure for holding if a service is local or remote. */

#define SEND_MESSAGE(_msg_) cfw_send_message(&(_msg_)->header)

void cfw_port_set_handler(uint16_t port_id, void (*handler)(struct cfw_message*, void*), void * param) {
#ifdef SVC_API_DEBUG
    pr_debug(LOG_MODULE_CFW, "%s: port: %p h: %p", __func__, port, handler);
#endif
	port_set_handler(port_id, (void (*)(struct message*, void*))handler, param);
}

struct cfw_message * cfw_clone_message(struct cfw_message * msg) {
    struct cfw_message * ret = (struct cfw_message *)
            message_alloc(CFW_MESSAGE_LEN(msg), NULL);
    if (ret == NULL) {
        pr_error(LOG_MODULE_CFW, "%s: Error allocating message", __func__);
    } else {
		memcpy(ret, msg, CFW_MESSAGE_LEN(msg));
    }
    return ret;
}

int cfw_register_service(T_QUEUE queue, service_t * svc,
        handle_msg_cb_t handle_message, void * data) {
    uint16_t port_id = port_alloc(queue);

    cfw_port_set_handler(port_id, handle_message, data);
    svc->port_id = port_id;
    return _cfw_register_service(svc);
}

int cfw_deregister_service(cfw_handle_t handle, service_t * svc) {
    return _cfw_deregister_service(handle, svc);
}

struct cfw_rsp_message * cfw_alloc_rsp_msg(const struct cfw_message *req, int msg_id, int size) {
    struct cfw_rsp_message * rsp = (struct cfw_rsp_message *) cfw_alloc_message(size, NULL);
    CFW_MESSAGE_TYPE(&rsp->header) = TYPE_RSP;
    CFW_MESSAGE_ID(&rsp->header) = msg_id;
    CFW_MESSAGE_LEN(&rsp->header) = size;
    CFW_MESSAGE_DST(&rsp->header) = CFW_MESSAGE_SRC(req);
    CFW_MESSAGE_SRC(&rsp->header) = CFW_MESSAGE_DST(req);
    rsp->header.priv = req->priv;
    /* Substitute server-side with client-side conn */
    if (req->conn != NULL)
        rsp->header.conn = ((conn_handle_t*)req->conn)->client_handle;
    else
        rsp->header.conn = NULL;
    return rsp;
}

struct cfw_message * cfw_alloc_evt_msg(service_t *svc, int msg_id, int size) {
    struct cfw_message * evt = (struct cfw_message *) cfw_alloc_message(size, NULL);
    CFW_MESSAGE_TYPE(evt) = TYPE_EVT;
    CFW_MESSAGE_ID(evt) = msg_id;
    CFW_MESSAGE_LEN(evt) = size;
    CFW_MESSAGE_SRC(evt) = svc->port_id;
    /* 3 fields below whould be filed by send_event method*/
    CFW_MESSAGE_DST(evt) = 0;
    evt->priv = NULL;
    evt->conn = NULL;
    return evt;
}

struct cfw_message * cfw_alloc_internal_msg(int msg_id, int size, void * priv) {
    struct cfw_message * evt = (struct cfw_message *) cfw_alloc_message(size, NULL);
    if (NULL == evt)
    {
        return NULL;
    }
    
    CFW_MESSAGE_TYPE(evt) = TYPE_INT;
    CFW_MESSAGE_ID(evt) = msg_id;
    CFW_MESSAGE_LEN(evt) = size;
    CFW_MESSAGE_SRC(evt) = 0;
    /* 3 fields below whould be filed by send_event method*/
    CFW_MESSAGE_DST(evt) = 0;
    evt->priv = priv;
    evt->conn = NULL;
    return evt;
}


int _cfw_send_message(struct cfw_message * message)
{
	return port_send_message(CFW_MESSAGE_HEADER(message));
}

void cfw_msg_free(struct cfw_message * msg)
{
	message_free(CFW_MESSAGE_HEADER(msg));
}
