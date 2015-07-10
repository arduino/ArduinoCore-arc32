#ifndef __CFW_MESSAGES_H__
#define __CFW_MESSAGES_H__

#include <stdlib.h>
#include <string.h>

enum
{
    MSG_ID_CFW_OPEN_SERVICE = 0x100,
    MSG_ID_CFW_SERVICE_REGISTER,
    MSG_ID_CFW_REGISTER_SVC_AVAIL,
    MSG_ID_CFW_SVC_AVAIL_EVT,
    MSG_ID_CFW_REGISTER_EVT,
    MSG_ID_CFW_UNREGISTER_EVT,
    MSG_ID_CFW_ALLOC_PORT,
    MSG_ID_CFW_CLOSE_SERVICE,
    MSG_ID_CFW_LAST
};

/**
 * \struct cfw_open_conn_req_msg_t
 * request message sent by \ref cfw_open_connection API.
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
 * response message to the \ref cfw_open_connection API.
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
 * request message sent by \ref cfw_close_connection API.
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
 * response message to the \ref cfw_close_connection API.
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
 * This message is sent to clients that called \ref cfw_register_svc_available api.
 * it notifies of the availability of a service.
 *
 */
typedef struct {
    /** common message header */
    struct cfw_message header;
    /** Service id of the newly available service. */
    int service_id;
} cfw_svc_available_evt_msg_t;


#endif /* __CFW_MESSAGE_H__ */
