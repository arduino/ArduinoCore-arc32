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
#include "cfw/cfw_debug.h"
#include "cfw/cfw_internal.h"

#include "infra/port.h"
#include "infra/log.h"

char * cfw_get_msg_type_str(struct cfw_message *msg)
{
    switch(CFW_MESSAGE_TYPE(msg)) {
        case TYPE_REQ:
            return "REQ";
        case TYPE_RSP:
            return "RSP";
        case TYPE_EVT:
            return "EVT";
        case TYPE_INT:
            return "INT";
        default:
            return "INVALID";
    }
}

void cfw_dump_service_handle(svc_client_handle_t * svc_handle)
{
    pr_info(LOG_MODULE_CFW, "svc_handle: port: %d, fw_handle: %p, server_handle: %p",
            svc_handle->port,
            svc_handle->cfw_handle,
            svc_handle->server_handle);
}

void cfw_dump_message(struct cfw_message * msg)
{
#if 1
    pr_debug(LOG_MODULE_CFW, "%p id: %x src: %d[cpu:%d] dst: %d[cpu:%d] type: %s",
    		msg, CFW_MESSAGE_ID(msg), CFW_MESSAGE_SRC(msg),
    		port_get_cpu_id(CFW_MESSAGE_SRC(msg)),
            CFW_MESSAGE_DST(msg), port_get_cpu_id(CFW_MESSAGE_DST(msg)),
            cfw_get_msg_type_str(msg));
#else
    pr_debug(LOG_MODULE_CFW, "id: %x src: %d dst: %d type: %s", msg->id,
            msg->src, msg->dst, cfw_get_msg_type_str(msg));
#endif
}
