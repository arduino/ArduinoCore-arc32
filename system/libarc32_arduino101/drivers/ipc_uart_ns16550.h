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

#ifndef _IPC_UART_NS16550_H_
#define _IPC_UART_NS16550_H_

#define IPC_UART 0


/** IPC UART return codes */
enum IPC_UART_RESULT_CODES {
	IPC_UART_ERROR_OK = 0,
	IPC_UART_ERROR_DATA_TO_BIG,
	IPC_UART_TX_BUSY /**< A transmission is already ongoing, message is NOT sent */
};


/**
 * Channel list
 */
enum ipc_channels {
	RPC_CHANNEL=0, /**< RPC channel */
	IPC_UART_MAX_CHANNEL = 4
};

/**
 * Channel state
 */
enum ipc_channel_state {
	IPC_CHANNEL_STATE_CLOSED = 0,
	IPC_CHANNEL_STATE_OPEN
};

/**
 * Definitions valid for NONE sync IPC UART headers
 * |len|channel|cpu_id|request|payload|
 *
 * len = len(request)+len(payload)
 */

/**
 * @note this structure must be self-aligned and self-packed
 */
struct ipc_uart_header {
	uint16_t len;       /**< Length of IPC message, (request + payload) */
	uint8_t channel;    /**< Channel number of IPC message. */
	uint8_t src_cpu_id; /**< CPU id of IPC sender. */
};

/**
 * IPC channel description
 */
struct ipc_uart_channels {
	uint16_t index; /**< Channel number */
	uint16_t state; /**< @ref ipc_channel_state */
	int (*cb)(int chan, int request, int len, void *data);
	/**< Callback of the channel.
	 * @param chan Channel index used
	 * @param request Request id (defined in ipc_requests.h)
	 * @param len Payload size
	 * @param data Pointer to data
	 */
};

void ipc_uart_init(int num);
void ipc_uart_isr();
//static void ipc_uart_push_frame(uint16_t len, uint8_t *p_data);
void ipc_uart_ns16550_disable(int num);
void ipc_uart_close_channel(int channel_id);
void ipc_uart_ns16550_set_tx_cb(void (*cb)(bool, void *), void *param);
int ipc_uart_ns16550_send_pdu(void *handle, int len, void *p_data);
void *ipc_uart_channel_open(int channel_id,
			    int (*cb)(int, int, int, void *));


#endif /* _IPC_UART_NS16550_H_ */
