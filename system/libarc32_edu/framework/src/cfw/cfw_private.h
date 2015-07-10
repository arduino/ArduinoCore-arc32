#ifndef __CFW_PRIVATE_H__
#define __CFW_PRIVATE_H__

#include "cfw/cfw_service.h"
int _cfw_register_service(service_t * svc);
int _cfw_deregister_service(cfw_handle_t handle, service_t * svc);
#endif
