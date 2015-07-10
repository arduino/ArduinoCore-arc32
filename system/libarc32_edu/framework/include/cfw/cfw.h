#ifndef _CFW_H_
#define _CFW_H_

#include <stdio.h>
#include <stdlib.h>

#include "os/os.h"
#include "util/list.h"
#include "infra/port.h"
#include "infra/message.h"

/**
 * @defgroup cfw CFW: Component Framework
 * The Component Framework is the main building block for custom applications.
 *
 * It consists of:
 *  - Service APIs
 *  - Service Manager API
 *  - Main loop handler that wraps the messaging and exposes a functional /
 * callback interface to the application.
 *
 * An application always need to create a thread managed by the framework in
 * order to be interfaced with it.
 * It could have other threads for its specific need and would have to manage
 * communication with the framework thread for interface with the rest of the platform.
 *
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
	 * Passed in the conn field of \ref struct cfw_message for request messages
	 */
	void * server_handle;
} svc_client_handle_t;


/**
 * \brief Allocate a memory buffer
 * it is mandatory to free the memory with \ref cfw_free
 *
 * \param size size of the memory to allocate
 *
 * \param err (out): execution status:
 *         -# E_OS_OK : memory was allocated
 *         -# E_OS_ERR: unable to allocate memory
 *
 * \return pointer to the allocated memory or NULL if failed.
 */
void * cfw_alloc(int size, OS_ERR_TYPE * err);

struct cfw_message * cfw_alloc_message(int size, OS_ERR_TYPE * err);

/**
 * \brief free a block of memory allocated with \ref cfw_alloc
 *
 * \param ptr the block address to free.
 *
 * \param err (out): execution status:
 *         -# E_OS_OK : memory was freed
 *         -# E_OS_ERR: unable to free memory
 */
void cfw_free(void * ptr, OS_ERR_TYPE * err);

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
