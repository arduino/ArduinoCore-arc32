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

#include <stdbool.h>
#include "os/os.h"
#include "util/list.h"
#include "infra/log.h"
#include "infra/port.h"
#include "cfw/cfw.h"
#include "cfw/cfw_debug.h"
#include "cfw/cfw_messages.h"
#include "cfw/cfw_internal.h"
#include "cfw/cfw_service.h"
#include "cfw_private.h"
#ifdef INFRA_MULTI_CPU_SUPPORT
#include "platform.h" /* NUM_CPU */
#endif
#include <stdint.h>

#define PANIC_NO_SVC_SLOT 1 /*!< Panic error code when all SVC slots are used */

/**
 * \file service_manager.c implementation of the service_manager
 */
service_t * services[MAX_SERVICES];

static int registered_service_count = 0;

int service_mgr_port_id = 0;

#ifdef INFRA_MULTI_CPU_SUPPORT
struct proxy {
	int port_id;
};
static struct proxy proxies[NUM_CPU];
#endif

/**
 * This function is called when a message needs to be sent through the IPC.
 *
 */

list_head_t service_avail_listeners = {NULL};

struct service_avail_listener {
    list_t l;
    uint16_t port;
    int service_id;
    void *priv;
};

void add_service_avail_listener(uint16_t port, int service_id, void *priv)
{
    struct service_avail_listener * l =
            (struct service_avail_listener *) balloc(sizeof(*l), NULL);
    l->port = port;
    l->service_id = service_id;
    l->priv = priv;
    pr_debug(LOG_MODULE_MAIN, "Register : %d to %d", port, service_id);
    list_add(&service_avail_listeners, &l->l);
}

static void send_service_avail_evt(int service_id, uint16_t port_id, void *param)
{
	cfw_svc_available_evt_msg_t * evt =
            (cfw_svc_available_evt_msg_t*) balloc(sizeof(*evt), NULL);
    evt->service_id = service_id;
    CFW_MESSAGE_LEN(&evt->header) = sizeof(*evt);
    CFW_MESSAGE_ID(&evt->header) = MSG_ID_CFW_SVC_AVAIL_EVT;
    CFW_MESSAGE_SRC(&evt->header) = service_mgr_port_id;
    CFW_MESSAGE_DST(&evt->header) = port_id;
    CFW_MESSAGE_TYPE(&evt->header) = TYPE_EVT;
    evt->header.priv = param;

    pr_debug(LOG_MODULE_MAIN, "Notify : %d to %d", service_id, port_id);
    cfw_send_message(evt);
}

int notify_service_avail_cb(void *item, void *param)
{
    struct service_avail_listener * l = (struct service_avail_listener *) item;
    int service_id = *(int*)param;
    if (l->service_id == service_id) {
        send_service_avail_evt(service_id, l->port, l->priv);
        bfree(l);
        return 1; /* remove from list */
    }
    return 0;
}

void notify_service_avail(int service_id)
{
    int svc_id = service_id;
    list_foreach_del(&service_avail_listeners, notify_service_avail_cb, &svc_id);
}

/**
 * Handle sync ipc request.
 * used to register a service or allocate a port.
 * Should be called in the context of the IPC interrupt.
 */
int handle_ipc_sync_request(uint8_t cpu_id, int request, int param1, int param2, void * ptr)
{
#ifdef SVC_MANAGER_DEBUG
    pr_debug(LOG_MODULE_CFW, "%s: from %d, req:%d (%d, %d, %p)", __func__, cpu_id, request, param1,
            param2, ptr);
#endif
    switch (request) {
    case IPC_MSG_TYPE_MESSAGE:
        {
            struct cfw_message * msg = (struct cfw_message *) ptr;
            _cfw_send_message(msg);
            break;
        }
    case IPC_REQUEST_REGISTER_SERVICE:
        {
            service_t * svc;
#ifndef CONFIG_SHARED_MEM
            svc = balloc(sizeof(*svc), NULL);
            svc->service_id = param1;
            svc->port_id = param2;
            svc->client_connected = NULL;
            svc->client_disconnected = NULL;
            svc->registered_events_changed = NULL;
#else
            svc = (service_t *) ptr;
#endif
            _cfw_register_service(svc);
        }
        break;

#ifdef INFRA_MULTI_CPU_SUPPORT
    case IPC_REQUEST_REGISTER_PROXY:
#ifdef SVC_MANAGER_DEBUG
	pr_debug(LOG_MODULE_CFW, "%s(): proxy registered for cpu %d @port %d",
		__func__, cpu_id, param1);
#endif
	proxies[cpu_id].port_id = param1;
	break;
#endif

    case IPC_REQUEST_DEREGISTER_SERVICE:
        {

        }
        break;

    default:
        pr_debug(LOG_MODULE_CFW, "%s: unhandled ipc request: %x", __func__, request);
        break;

    }
    return 0;
}

void internal_handle_message(struct cfw_message * msg, void * param)
{
    int free_msg = 1; /* by default free message */
    switch (CFW_MESSAGE_ID(msg))
        {
    case MSG_ID_CFW_ALLOC_PORT:
        {
            //TODO: Enable this, currently relies on sync IPC API.
            //_port_t * port = cfw_port_alloc(NULL);
            //cfw_port_set_handler(port, send_message_ipc, NULL);
            break;
        }

    case MSG_ID_CFW_OPEN_SERVICE:
        {
            cfw_open_conn_req_msg_t * req = (cfw_open_conn_req_msg_t *) msg;
            service_t * svc = cfw_get_service(req->service_id);
            if (svc == NULL) {
                pr_error(LOG_MODULE_MAIN, "try to open non registered service %d", req->service_id);
                cfw_open_conn_rsp_msg_t * resp =
                        (cfw_open_conn_rsp_msg_t *) cfw_alloc_rsp_msg(msg,
                                MSG_ID_CFW_OPEN_SERVICE, sizeof(*resp));
                resp->rsp_header.status = E_OS_ERR_UNKNOWN;
                cfw_send_message(resp);
                break;
            }
            uint8_t svc_cpu_id = port_get_cpu_id(svc->port_id);
            if (svc_cpu_id != get_cpu_id()) {
#ifdef INFRA_MULTI_CPU_SUPPORT
                pr_debug(LOG_MODULE_MAIN, "forward open service to proxy");
                CFW_MESSAGE_DST(msg) = proxies[svc_cpu_id].port_id;
                cfw_send_message(msg);
                free_msg = 0; /* the lower layers will free it! */
#else
                pr_error(LOG_MODULE_MAIN, "incorrect cpu_id settings, single cpu!");
#endif
            } else {
                conn_handle_t * conn_handle;

                conn_handle = (conn_handle_t *) balloc(sizeof(*conn_handle),
                        NULL );
                conn_handle->client_port = CFW_MESSAGE_SRC(msg);
                conn_handle->priv_data = NULL;
                conn_handle->svc = svc;
                conn_handle->client_handle = req->client_handle;
                /* For OPEN_SERVICE, conn is not know yet, it is just alloc'ed.
                 * set it here.*/
                req->header.conn = conn_handle;
                if (svc->client_connected != NULL &&
                        svc_cpu_id == get_cpu_id())
                    svc->client_connected(conn_handle);
                cfw_open_conn_rsp_msg_t * resp =
                        (cfw_open_conn_rsp_msg_t *) cfw_alloc_rsp_msg(msg,
                                MSG_ID_CFW_OPEN_SERVICE, sizeof(*resp));
                resp->port = svc->port_id;
                resp->cpu_id = svc_cpu_id;
#ifdef SVC_MANAGER_DEBUG
                pr_debug(LOG_MODULE_CFW, "OPEN_SERVICE: %d, svc:%p port:%d", req->service_id,
                        svc, svc->port_id);
#endif
                resp->svc_server_handle = conn_handle;
                resp->client_handle = req->client_handle;
                cfw_send_message(resp);
            }
            break;
        }

    case MSG_ID_CFW_CLOSE_SERVICE:
        {
            cfw_close_conn_req_msg_t * req = (cfw_close_conn_req_msg_t*) msg;
            service_t * svc = cfw_get_service(req->service_id);
            if (svc == NULL) {
                pr_debug(LOG_MODULE_MAIN, "try close unregistered service %d",
                         req->service_id);
                cfw_close_conn_rsp_msg_t * resp =
                        (cfw_close_conn_rsp_msg_t *) cfw_alloc_rsp_msg(msg,
                                MSG_ID_CFW_CLOSE_SERVICE, sizeof(*resp));
                resp->rsp_header.status = E_OS_ERR_UNKNOWN;
                cfw_send_message(resp);
                break;
            }
            uint8_t svc_cpu_id = port_get_cpu_id(svc->port_id);
            if (svc_cpu_id != get_cpu_id()) {
#ifdef INFRA_MULTI_CPU_SUPPORT
                CFW_MESSAGE_DST(msg) = proxies[svc_cpu_id].port_id;
                cfw_send_message(msg);
                free_msg=0;
#else
                pr_error(LOG_MODULE_MAIN, "incorrect cpu_id!");
#endif
            } else {
                cfw_close_conn_rsp_msg_t * resp =
                            (cfw_close_conn_rsp_msg_t *) cfw_alloc_rsp_msg(msg,
                                    MSG_ID_CFW_CLOSE_SERVICE, sizeof(*resp));
                conn_handle_t * conn = (conn_handle_t*) msg->conn;
                if (conn != NULL && conn->svc != NULL
                        && conn->svc->client_disconnected != NULL )
                    conn->svc->client_disconnected(conn);
                cfw_send_message(resp);
                /* Free server-side conn */
                bfree(conn);
            }
            break;
        }

    case MSG_ID_CFW_REGISTER_EVT:
        {
            int * params = (int *) &msg[1];
            int i;
            for (i = 0; i < params[0]; i++) {
                _cfw_register_event((conn_handle_t*) msg->conn, params[i + 1]);
            }
            cfw_register_evt_rsp_msg_t * resp =
                    (cfw_register_evt_rsp_msg_t *) cfw_alloc_rsp_msg(msg,
                            MSG_ID_CFW_REGISTER_EVT, (sizeof(*resp)));
            conn_handle_t * conn = (conn_handle_t*) msg->conn;
            if (conn != NULL && conn->svc != NULL
                    && conn->svc->registered_events_changed != NULL )
                conn->svc->registered_events_changed(conn);

            cfw_send_message(resp);
            break;
        }

    case MSG_ID_CFW_REGISTER_SVC_AVAIL:
        {
            bool already_avail = true;
            cfw_register_svc_avail_req_msg_t * req =
                    (cfw_register_svc_avail_req_msg_t *) msg;
            int flags = interrupt_lock();
            if (_find_service(req->service_id) == -1) {
                add_service_avail_listener(CFW_MESSAGE_SRC(msg),
                        req->service_id, msg->priv);
                already_avail = false;
            }
            interrupt_unlock(flags);
            cfw_register_svc_avail_rsp_msg_t * resp =
                (cfw_register_svc_avail_rsp_msg_t *) cfw_alloc_rsp_msg(msg,
                        MSG_ID_CFW_REGISTER_SVC_AVAIL, (sizeof(*resp)));
            cfw_send_message(resp);
            if (already_avail) {
                send_service_avail_evt(req->service_id, CFW_MESSAGE_SRC(msg), msg->priv);
            }
            break;
        }

    default:
        pr_warning(LOG_MODULE_CFW, "%s: unhandled message id: %x", __func__, CFW_MESSAGE_ID(msg));
        break;
        }
    if (free_msg)
        cfw_msg_free(msg);
}

/**
 * Indication list
 * Holds a list of receivers.
 */
typedef struct
{
    list_t list;
    conn_handle_t * conn_handle;
} indication_list_t;

/**
 * \struct registered_int_list_t holds a list of registered clients to an indication
 *
 * Holds a list of registered receiver for each indication.
 */
typedef struct registered_evt_list_
{
    list_t list; /*! Linking stucture */
    list_head_t lh; /*! List of client */
    int ind; /*! Indication message id */
} registered_evt_list_t;

list_head_t registered_evt_list;

registered_evt_list_t * get_event_registered_list(int msg_id)
{
    registered_evt_list_t * l =
            (registered_evt_list_t*) registered_evt_list.head;
    while (l) {
        if (l->ind == msg_id) {
            return l;
        }
        l = (registered_evt_list_t*) l->list.next;
    }
    return NULL ;
}

list_head_t * get_event_list(int msg_id)
{
    registered_evt_list_t * l = get_event_registered_list(msg_id);
    if (l)
        return &l->lh;
    return NULL ;
}

static void send_event_callback(void * item, void * param)
{
    struct cfw_message * msg = (struct cfw_message *) param;
    indication_list_t * ind = (indication_list_t *) item;
    struct cfw_message * m = cfw_clone_message(msg);
    if (m != NULL ) {
        CFW_MESSAGE_DST(m) = ind->conn_handle->client_port;
        cfw_send_message(m);
    }
}

void cfw_send_event(struct cfw_message * msg)
{
#ifdef SVC_MANAGER_DEBUG
    pr_debug(LOG_MODULE_CFW, "%s : msg:%d", __func__, CFW_MESSAGE_ID(msg));
#endif
    list_head_t * list = get_event_list(CFW_MESSAGE_ID(msg));
    if (list != NULL ) {
        list_foreach(list, send_event_callback, msg);
    }
}

void _cfw_register_event(conn_handle_t * h, int msg_id)
{
#ifdef SVC_MANAGER_DEBUG
    pr_debug(LOG_MODULE_CFW, "%s : msg:%d port %d h:%p", __func__, msg_id, h->client_port, h);
#endif
    registered_evt_list_t * ind =
            (registered_evt_list_t *) get_event_registered_list(msg_id);

    if (ind == NULL ) {
        ind = (registered_evt_list_t *) balloc(sizeof(*ind), NULL );
        ind->ind = msg_id;
        list_init(&ind->lh);
        list_add(&registered_evt_list, &ind->list);
    }

    indication_list_t * e = (indication_list_t *) balloc(sizeof(*e), NULL );
    e->conn_handle = h;
    list_add(&ind->lh, (list_t *) e);
}

int _find_service(int service_id)
{
    int i;
    for (i = 0; i < MAX_SERVICES; i++) {
        if (services[i] != NULL && services[i]->service_id == service_id) {
            return i;
        }
    }
    return -1;
}

service_t * cfw_get_service(int service_id)
{
    int index;
    index = _find_service(service_id);
    return (index == -1) ? NULL : services[index];
}

int _cfw_get_service_port(int service_id)
{
    service_t * svc = cfw_get_service(service_id);
    if (svc != NULL ) {
        return svc->port_id;
    }
    return -1;
}

void _cfw_loop(void * queue)
{
    struct cfw_message * message;
    T_QUEUE_MESSAGE m;
    while (1) { /* This infinite loop is intentional and requested by design */
        queue_get_message(queue, &m, OS_WAIT_FOREVER, NULL );
        message = (struct cfw_message *) m;
        if (message != NULL ) {
            port_process_message(&message->m);
        }
    }
}

uint16_t cfw_get_service_mgr_port_id()
{
    return service_mgr_port_id;
}

void _cfw_init(void * queue)
{
    uint16_t port_id = port_alloc(queue);
    port_set_handler(port_id, (void(*)(struct message*, void *))internal_handle_message, NULL );
    service_mgr_port_id = port_id;
#ifdef SVC_MANAGER_DEBUG
    pr_debug(LOG_MODULE_CFW, "%s queue: %p", __func__, queue);
#endif
}

void _add_service(service_t * svc)
{
    int i;
    for(i=0; i<MAX_SERVICES;i++) {
        if (services[i] == NULL) {
            services[i] = svc;
            registered_service_count++;
            return;
        }
    }
    panic(PANIC_NO_SVC_SLOT); /* No more free slots. */
}

int _cfw_register_service(service_t * svc)
{
    pr_debug(LOG_MODULE_MAIN, "%s: %p id:%d port: %d\n", __func__, svc, svc->service_id,
            svc->port_id);
    int flags = interrupt_lock();
    if (_find_service(svc->service_id) != -1) {
        interrupt_unlock(flags);
        pr_error(LOG_MODULE_MAIN, "Error: service %d already registered\n",
                svc->service_id);
        return -1;
    }

    _add_service(svc);

    notify_service_avail(svc->service_id);

    interrupt_unlock(flags);
    return 0;
}

int _cfw_deregister_service(cfw_handle_t handle, service_t * svc)
{
    int index;
#ifdef SVC_MANAGER_DEBUG
    pr_debug(LOG_MODULE_CFW, "%s", __func__);
#endif
    if ((index = _find_service(svc->service_id)) == -1) {
        pr_error(LOG_MODULE_CFW, "Error: service %d was not registered", svc->service_id);
        return -1;
    }
    registered_service_count--;
    services[index] = NULL;
    return 0;
}

/**
 * \deprecated
 * Only accepted usage is tests
 */
int cfw_service_registered(int service_id)
{
    return (_find_service(service_id) != -1);
}
