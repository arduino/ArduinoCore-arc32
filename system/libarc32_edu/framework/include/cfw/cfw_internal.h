#ifndef __CFW_INTERNAL_H__
#define __CFW_INTERNAL_H__

#include "cfw/cfw.h"
#include "cfw/cfw_service.h"
#include "infra/ipc_requests.h"

#define MAX_SERVICES    16
typedef struct {
	handle_msg_cb_t handle_msg;
	void *data;
	uint16_t client_port_id;
}_cfw_handle_t;

/**
 * This function is called when an sync IPC request is issued
 * from a secondary CPU.
 *
 * \param cpu_id the cpu_id that originated the request
 * \param request the request id.
 * \param param1 first param
 * \param param2 second param
 * \param ptr third param
 *
 * \return the value to be passed as response to the requestor
 */
int handle_ipc_sync_request(uint8_t cpu_id, int request, int param1,
        int param2, void * ptr);

#endif /* __CFW_INTERNAL_H__ */
