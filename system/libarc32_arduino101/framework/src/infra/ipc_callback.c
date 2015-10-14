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
#include "log_impl.h"
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
