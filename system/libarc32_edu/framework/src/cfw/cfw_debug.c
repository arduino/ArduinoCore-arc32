#include "cfw/cfw.h"
#include "cfw/cfw_debug.h"
#include "cfw/cfw_internal.h"
#include "infra/port.h"

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
    cfw_log("svc_handle: port: %d, fw_handle: %p, server_handle: %p\n",
            svc_handle->port,
            svc_handle->cfw_handle,
            svc_handle->server_handle);
}

void cfw_dump_message(struct cfw_message * msg)
{
#if 1
    cfw_log("%p id: %x src: %d[cpu:%d] dst: %d[cpu:%d] type: %s\n",
    		msg, CFW_MESSAGE_ID(msg), CFW_MESSAGE_SRC(msg),
    		port_get_cpu_id(CFW_MESSAGE_SRC(msg)),
            CFW_MESSAGE_DST(msg), port_get_cpu_id(CFW_MESSAGE_DST(msg)),
            cfw_get_msg_type_str(msg));
#else
    cfw_log("id: %x src: %d dst: %d type: %s\n", msg->id,
            msg->src, msg->dst, cfw_get_msg_type_str(msg));
#endif
}
