#include "util/list.h"
#include "cfw/cfw.h"
#include "cfw/cfw_debug.h"
#include "cfw/cfw_messages.h"
#include "cfw/cfw_internal.h"
#include "cfw/cfw_service.h"

#include "infra/port.h"
#include "infra/ipc.h"
#include "infra/log.h"

/**
 * \file service_manager_proxy.c implementation of the service_manager proxy
 *
 * This file implements the proxy for the service manager.
 * It has to be compiled in all cores except the one that holds the service
 * manager.
 */
int service_mgr_port_id = 0;

struct service_item {
	list_t list;
	service_t * svc;
};

list_head_t local_services = {NULL, NULL};

void internal_handle_message(struct message * m, void * param) {
	struct cfw_message * msg = (struct cfw_message *) m;
	switch(CFW_MESSAGE_ID(msg)) {
        case MSG_ID_CFW_ALLOC_PORT: {
            //_port_t * port = cfw_port_alloc(NULL);
            //cfw_port_set_handler(port, send_message_ipc, NULL);
            break;
        }

        case MSG_ID_CFW_OPEN_SERVICE: {
            conn_handle_t * conn_handle;
            cfw_open_conn_req_msg_t * req = (cfw_open_conn_req_msg_t *) msg;
	    service_t * svc = cfw_get_service(req->service_id);
	    uint8_t svc_cpu_id = port_get_cpu_id(svc->port_id);
	    if (svc_cpu_id != req->client_cpu_id) {
                port_set_port_id(CFW_MESSAGE_SRC(msg));
                /* allow correct routing of responses to client cpu */
                port_set_cpu_id(CFW_MESSAGE_SRC(msg), req->client_cpu_id);
            }
#ifdef SVC_MANAGER_DEBUG
            pr_debug(LOG_MODULE_CFW, "%s(): service id %d, client_port: %d client_cpu_id: %d",
        		    __func__, req->service_id, CFW_MESSAGE_SRC(msg),
        		    req->client_cpu_id);
#endif
            conn_handle = (conn_handle_t *) cfw_alloc(sizeof(*conn_handle), NULL);
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
            cfw_open_conn_rsp_msg_t * resp = (cfw_open_conn_rsp_msg_t *) cfw_alloc_rsp_msg(msg,
                    MSG_ID_CFW_OPEN_SERVICE, sizeof(*resp));
            resp->port = svc->port_id;
            resp->cpu_id = svc_cpu_id;
            resp->svc_server_handle = conn_handle;
            resp->client_handle = req->client_handle;
            cfw_send_message(resp);
            break;
        }

        case MSG_ID_CFW_CLOSE_SERVICE: {
            cfw_close_conn_rsp_msg_t * resp = (cfw_close_conn_rsp_msg_t *) cfw_alloc_rsp_msg(msg,
                    MSG_ID_CFW_CLOSE_SERVICE, sizeof(*resp));
            conn_handle_t * conn = (conn_handle_t*)msg->conn;
            if ( conn != NULL && conn->svc != NULL && conn->svc->client_disconnected != NULL)
                conn->svc->client_disconnected(conn);
            cfw_send_message(resp);
            /* Free server-side conn */
            cfw_free(conn, NULL);
            break;
        }

        case MSG_ID_CFW_REGISTER_EVT: {
            int * params = (int *) &msg[1];
            int i;
            for (i = 0; i < params[0]; i++) {
                _cfw_register_event((conn_handle_t*)msg->conn, params[i+1]);
            }
            cfw_register_evt_rsp_msg_t * resp =
                    (cfw_register_evt_rsp_msg_t *) cfw_alloc_rsp_msg(msg,
                            MSG_ID_CFW_REGISTER_EVT, (sizeof(*resp)));
            conn_handle_t * conn = (conn_handle_t*)msg->conn;
            if ( conn != NULL && conn->svc != NULL && conn->svc->registered_events_changed != NULL)
                conn->svc->registered_events_changed(conn);

            cfw_send_message(resp);
            break;
        }

        default:
            pr_warning(LOG_MODULE_CFW, "%s: unhandled message id: %x", __func__, CFW_MESSAGE_ID(msg));
            break;
    }
    cfw_msg_free(msg);
}

int handle_ipc_sync_request(uint8_t cpu_id, int request, int param1, int param2, void * ptr)
{
#ifdef SVC_MANAGER_DEBUG
    pr_debug(LOG_MODULE_CFW, "%s: from %d, req:%d (%d, %d, %p)", __func__, cpu_id, request, param1,
            param2, ptr);
#endif
    switch (request) {
    case IPC_MSG_TYPE_FREE:
        cfw_free(ptr, NULL);
        break;
    case IPC_MSG_TYPE_MESSAGE:
        {
            struct cfw_message * msg = (struct cfw_message *) ptr;
            _cfw_send_message(msg);
            break;
        }
    case IPC_REQUEST_ALLOC_PORT:
        {
#ifdef SVC_MANAGER_DEBUG
            pr_debug(LOG_MODULE_CFW, "Updating port id %d", param1);
#endif
            port_set_port_id((uint16_t)param1);
            port_set_cpu_id((uint16_t)param1, get_cpu_id()); /* assign instance cpu id as this is a response */
            break;
        }
    case IPC_REQUEST_REGISTER_PROXY:
        {
#ifdef SVC_MANAGER_DEBUG
            pr_debug(LOG_MODULE_CFW, "Sync rsp");
#endif
            break;
        }
    default:
        pr_warning(LOG_MODULE_CFW, "%s: unexpected ipc_sync_request: %d", __func__, request);
        break;
    }
    return 0;
}


uint16_t cfw_port_alloc(void * queue) {
	return port_alloc(queue);
}


/**
 * Indication list
 * Holds a list of receivers.
 */
typedef struct {
    list_t list;
    conn_handle_t * conn_handle;
} indication_list_t;

/**
 * \struct registered_int_list_t holds a list of registered clients to an indication
 *
 * Holds a list of registered receiver for each indication.
 */
typedef struct registered_evt_list_ {
    list_t list;     /*! Linking stucture */
    list_head_t lh;  /*! List of client */
    int ind;         /*! Indication message id */
} registered_evt_list_t;

list_head_t registered_evt_list;



registered_evt_list_t * get_event_registered_list(int msg_id) {
    registered_evt_list_t * l = (registered_evt_list_t*)registered_evt_list.head;
    while(l) {
        if (l->ind == msg_id) {
            return l;
        }
        l = (registered_evt_list_t*)l->list.next;
    }
    return NULL;
}

list_head_t * get_event_list(int msg_id) {
    registered_evt_list_t * l = get_event_registered_list(msg_id);
    if (l)
        return &l->lh;
    return NULL;
}

static void send_event_callback(void * item, void * param) {
    struct cfw_message * msg = (struct cfw_message *) param;
    indication_list_t * ind = (indication_list_t *)item;
    struct cfw_message * m = cfw_clone_message(msg);
    if (m != NULL) {
        CFW_MESSAGE_DST(m) = ind->conn_handle->client_port;
        cfw_send_message(m);
    }
}

void cfw_send_event(struct cfw_message * msg) {
#ifdef SVC_MANAGER_DEBUG
    pr_debug(LOG_MODULE_CFW, "%s : msg:%d", __func__, CFW_MESSAGE_ID(msg));
#endif
    list_head_t * list = get_event_list(CFW_MESSAGE_ID(msg));
    if (list != NULL) {
        list_foreach(list, send_event_callback, msg);
    }
}

void _cfw_register_event(conn_handle_t * h, int msg_id) {
#ifdef SVC_MANAGER_DEBUG
    pr_debug(LOG_MODULE_CFW, "%s : msg:%d port %d h:%p", __func__, msg_id, h->client_port, h);
#endif
    registered_evt_list_t * ind = (registered_evt_list_t *) get_event_registered_list(msg_id);

    if (ind == NULL) {
        ind = (registered_evt_list_t *) cfw_alloc(sizeof(*ind), NULL);
        ind->ind = msg_id;
        list_init(&ind->lh);
        list_add(&registered_evt_list, &ind->list);
    }

    indication_list_t * e = (indication_list_t *)cfw_alloc(sizeof(*e), NULL);
    e->conn_handle = h;
    list_add(&ind->lh, (list_t *)e);
}

void _cfw_loop(void * queue) {
    struct cfw_message * message;
    T_QUEUE_MESSAGE m;
    while(1) {
        queue_get_message(queue, &m, OS_WAIT_FOREVER, NULL);
        message = (struct cfw_message *) m;
        if ( message != NULL ) {
            port_process_message(&message->m);
        }
    }
}

struct get_service_cb_arg {
	service_t * svc;
	int service_id;
};

void get_service_cb(void * list, void *param)
{
	struct get_service_cb_arg * arg = (struct get_service_cb_arg *)param;
	struct service_item * item = (struct service_item *) list;
	if (item->svc->service_id == arg->service_id)
	{
		arg->svc = item->svc;
	}
}

service_t * cfw_get_service(int service_id) {
    struct get_service_cb_arg arg = {NULL, service_id};
    list_foreach(&local_services, get_service_cb, &arg);
    return arg.svc;
}

/**
 * @deprecated replace by svc->port_id
 */
int _cfw_get_service_port(int service_id) {
    service_t * svc = cfw_get_service(service_id);
    if (svc != NULL) {
        return svc->port_id;
    }
    return -1;
}

typedef void(*send_msg_t)(struct cfw_message * m);
extern send_msg_t get_ipc_handler(uint8_t cpu_id);

void svc_manager_handle_message(struct cfw_message * msg, void * param) {
    get_ipc_handler(0)(msg);
}

uint16_t cfw_get_service_mgr_port_id()
{
    return service_mgr_port_id;
}

uint16_t proxy_port_id;

void _cfw_init_proxy(T_QUEUE queue, void *p, service_t **s, uint16_t svc_mgr_port) {
    port_set_ports_table(p);
    service_mgr_port_id = svc_mgr_port;
    proxy_port_id = cfw_port_alloc(queue);
    port_set_handler(proxy_port_id, internal_handle_message, NULL);
    ipc_request_sync_int(IPC_REQUEST_REGISTER_PROXY, proxy_port_id, 0, NULL);
    pr_debug(LOG_MODULE_CFW, "%s queue: %p", __func__, queue);
}


int _cfw_register_service(service_t * svc) {
	struct service_item * item = (struct service_item *) balloc(sizeof(*item), NULL);
	item->svc = svc;
	list_add(&local_services, &item->list);
	return ipc_request_sync_int(IPC_REQUEST_REGISTER_SERVICE, svc->service_id, svc->port_id, svc);
}

int _cfw_deregister_service(cfw_handle_t handle, service_t * svc) {
    return ipc_request_sync_int(IPC_REQUEST_DEREGISTER_SERVICE, svc->service_id, 0, svc);
}
