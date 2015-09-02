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

#ifndef __CFW_MESSAGES_H__
#define __CFW_MESSAGES_H__

#include <stdlib.h>
#include <string.h>
/**
 * @defgroup cfw_messages CFW base messages
 *
 * This file defines the base messages for the component framework.
 *
 * @ingroup cfw
 * @{
 */
enum
{
    /** Open service message */
    MSG_ID_CFW_OPEN_SERVICE = 0x100,
    /** Register service message */
    MSG_ID_CFW_SERVICE_REGISTER,
    /** Register for service availability message */
    MSG_ID_CFW_REGISTER_SVC_AVAIL,
    /** Indication of service availability message */
    MSG_ID_CFW_SVC_AVAIL_EVT,
    /** Register for service events message */
    MSG_ID_CFW_REGISTER_EVT,
    /** Unregister from service events message */
    MSG_ID_CFW_UNREGISTER_EVT,
    /** @deprecated Allocate a port message */
    MSG_ID_CFW_ALLOC_PORT,
    /** Close connection to a service message */
    MSG_ID_CFW_CLOSE_SERVICE,
    MSG_ID_CFW_LAST
};

/**
 * \struct cfw_open_conn_req_msg_t
 * request message sent by cfw_open_connection() API.
 */
typedef struct {
	/** common message header */
	struct cfw_message header;
	/** service to open connection to */
	int service_id;
	/** client side service handle */
	void *client_handle;
	/** client side cpu id, required for remote node services */
	uint8_t client_cpu_id;
} cfw_open_conn_req_msg_t;

/**
 * \struct cfw_open_conn_cnf_msg_t
 * response message to the cfw_open_connection() API.
 */
typedef struct {
	/** common response message header */
	struct cfw_rsp_message rsp_header;
	/** port to attain this service */
	uint16_t port;
	/** cpu_id of service */
	uint8_t cpu_id;
	/** service handle for accessing the service */
	void * svc_server_handle;
	/** client side service handle as passed in
	 * \ref cfw_open_conn_req_msg_t */
	void * client_handle;
} cfw_open_conn_rsp_msg_t;

/**
 * \struct cfw_close_conn_req_msg_t
 * request message sent by cfw_close_connection() API.
 */
typedef struct {
	/** common message header */
	struct cfw_message header;
	/** service id to close */
	int service_id;
	/** service to open connection to */
	void * inst;
} cfw_close_conn_req_msg_t;

/**
 * \struct cfw_close_conn_cnf_msg_t
 * response message to the cfw_close_connection() API.
 */
typedef struct {
	/** common response message header */
	struct cfw_rsp_message rsp_header;
} cfw_close_conn_rsp_msg_t;

typedef struct {
	/** common message header */
	struct cfw_message header;
	/** indication message identifier.
	 * all subsequent indication with this identifier will be sent
	 * to the src port of this request message.
	 */
	int evt;
} cfw_register_evt_req_msg_t;

typedef struct {
	/** common response message header */
	struct cfw_rsp_message rsp_header;
} cfw_register_evt_rsp_msg_t;


typedef struct {
	struct cfw_message header;
	int service_id;
} cfw_register_svc_avail_req_msg_t;

typedef struct {
	/** common response message header */
	struct cfw_rsp_message rsp_header;
} cfw_register_svc_avail_rsp_msg_t;
/**
 * This message is sent to clients that called cfw_register_svc_available() api.
 * it notifies of the availability of a service.
 *
 */
typedef struct {
    /** common message header */
    struct cfw_message header;
    /** Service id of the newly available service. */
    int service_id;
} cfw_svc_available_evt_msg_t;

/* @} */
#endif /* __CFW_MESSAGE_H__ */
