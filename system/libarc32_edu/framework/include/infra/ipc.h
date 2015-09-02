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

#ifndef __IPC_H__
#define __IPC_H__

#include "infra/ipc_requests.h"
#include "infra/message.h"

/**
 * @defgroup ipc IPC layer
 * @ingroup infra
 * @{
 * @defgroup ipc_mbx IPC Mailbox interface
 * Defines the interface used for ARC/LMT IPC
 * @{
 */

/**
 * Initialize ipc component.
 *
 * @param tx_channel the IPC tx mailbox.
 * @param rx_channel the IPC rx mailbox.
 * @param tx_ack_channel the tx acknowledge mailbox.
 * @param rx_ack_channel the rx acknowledge mailbox.
 * @param remote_cpu_id the remote cpu id of this IPC
 */
void ipc_init(int tx_channel, int rx_channel, int tx_ack_channel,
        int rx_ack_channel, uint8_t remote_cpu_id);

/**
 * Request a synchronous ipc call.
 *
 * This method blocks until the command is answered.
 *
 * @param request_id the synchronous request id
 * @param param1 the first param for the request
 * @param param2 the second param for the request
 * @param ptr the third param for the request
 *
 * @return the synchronous command response.
 */
int ipc_request_sync_int(int request_id, int param1, int param2, void *ptr);

/**
 * Polling mode polling call.
 *
 * In polling mode, this method has to be called in order to handle
 * ipc requests.
 */
void ipc_handle_message();

/**
 * Called by platform specific code when an IPC synchronous request is
 * received.
 *
 * Inter-processors request callback called in the context of an interrupt
 *
 * @param cpu_id the cpu this request comes from
 * @param request request ID
 * @param param1 a first int parameter
 * @param param2 a second int parameter
 * @param ptr    a last pointer parameter
 *
 * @return 0 if success, -1 otherwise
 */
int ipc_sync_callback(uint8_t cpu_id, int request, int param1, int param2,
		void *ptr);

/**
 * Called by port implementation when a message has to be sent to a
 * CPU connected through the mailbox ipc mechanism.
 *
 * @param message the message to send to the other end
 *
 * @return 0 if success, -1 otherwise
 */
int ipc_async_send_message(struct message *message);

/**
 * Called by port implementation when a message that comes from a
 * different CPU than ours has to be freed.
 *
 * @param message the message to be freed.
 */
void ipc_async_free_message(struct message *message);

/**
 * Initialize the async message sender / freeing mechanism for remote
 * message sending and freeing.
 *
 * Async message sender / freeing is needed as message can be sent / freed in
 * the context of interrupts. And in interrupt context, the ipc requests sync
 * cannot be called.
 *
 * @param queue the queue on which context the ipc requests have to be called.
 */
void ipc_async_init(T_QUEUE queue);
/** @} */
/** @} */
#endif
