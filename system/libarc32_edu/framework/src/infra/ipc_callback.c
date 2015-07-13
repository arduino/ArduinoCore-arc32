/* INTEL CONFIDENTIAL Copyright 2014 Intel Corporation All Rights Reserved.
  *
  * The source code contained or described herein and all documents related to
  * the source code ("Material") are owned by Intel Corporation or its suppliers
  * or licensors.
  * Title to the Material remains with Intel Corporation or its suppliers and
  * licensors.
  * The Material contains trade secrets and proprietary and confidential information
  * of Intel or its suppliers and licensors. The Material is protected by worldwide
  * copyright and trade secret laws and treaty provisions.
  * No part of the Material may be used, copied, reproduced, modified, published,
  * uploaded, posted, transmitted, distributed, or disclosed in any way without
  * Intel’s prior express written permission.
  *
  * No license under any patent, copyright, trade secret or other intellectual
  * property right is granted to or conferred upon you by disclosure or delivery
  * of the Materials, either expressly, by implication, inducement, estoppel or
  * otherwise.
  *
  * Any license under such intellectual property rights must be express and
  * approved by Intel in writing
  *
  ******************************************************************************/
/* For the framework IPC message handler*/
#include "cfw/cfw_internal.h"

#ifdef CONFIG_TCMD_MASTER
#include "tcmd/master.h"
#endif /* CONFIG_TCMD_MASTER */
#include "infra/port.h"
#ifdef CONFIG_PANIC
#include "infra/panic.h"
#endif
#ifdef CONFIG_LOG_MULTI_CPU_SUPPORT
#include "infra/log_impl_cbuffer.h"
#endif

int ipc_sync_callback(uint8_t cpu_id, int request, int param1, int param2,
		void *ptr)
{
	int ret = 0;

	switch (request) {
#ifdef CONFIG_TCMD_MASTER
		case IPC_REQUEST_REG_TCMD_ENGINE:
		{
			/* This is a Test Command engine registration message */
			ret = tcmd_ipc_sync_callback(cpu_id, request, param1, param2, ptr);
			break;
		}
#endif /* CONFIG_TCMD_MASTER */
		case IPC_REQUEST_ALLOC_PORT:
		{
			uint16_t port_id = port_alloc(NULL);
			port_set_cpu_id(port_id, cpu_id);
			ret = port_id;
			break;
		}
		case IPC_MSG_TYPE_FREE:
			message_free(ptr);
			break;
#ifdef CONFIG_PANIC
		case IPC_PANIC_NOTIFICATION:
			// Handle panic notification of core id "param1"
			handle_panic_notification(param1);
			break;
#endif
#ifdef CONFIG_LOG_MULTI_CPU_SUPPORT
		case IPC_REQUEST_LOGGER:
		{
			log_cb_ipc(cpu_id, ptr);
			break;
		}
#endif
		default:
		{
			/* This is a framework message */
			ret = handle_ipc_sync_request(cpu_id, request, param1, param2,
					ptr);
		}
	}
	return ret;
}
