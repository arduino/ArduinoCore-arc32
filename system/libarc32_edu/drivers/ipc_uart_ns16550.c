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

#include "platform.h"
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "infra/ipc_uart.h"
#include "infra/log.h"
#include "uart.h"
#include "scss_registers.h"
#include "infra/ipc_requests.h"
#include "ipc_uart_ns16550.h"
#include "os/os_types.h"
#include "os/os.h"
#include "infra/port.h"

#include "portable.h"

#define BOTH_EMPTY (0x40|0x20) /* TEMT & THRE emtpy */
#define IPC_UART_MAX_PAYLOAD 128 /* TODO: make it a globals setting to be in sync with nordic */

#define IPC_UART_HDR_REQUEST_LEN (IPC_HEADER_LEN+sizeof(uint32_t)) /* ipc header + request len */

struct ipc_uart_channels channels[IPC_UART_MAX_CHANNEL] = { {0, 0, NULL},};
static uint16_t send_counter = 0;
static uint16_t received_counter = 0;
static uint8_t * ipc_uart_tx = NULL;
static uint8_t * ipc_uart_rx = NULL;
static uint8_t ipc_uart_tx_state = 0;

static void * ble_cfw_channel;

static const struct uart_init_info uart_dev_info[] = {
	{
		.regs = PERIPH_ADDR_BASE_UART0,
		.sys_clk_freq = SYSCLK_DEFAULT_IOSC_HZ,
		.baud_rate = CONFIG_IPC_UART_BAUDRATE,
		.options = UART_OPTION_AFCE,
		.irq = IRQ_UART0_INTR,
		.int_pri = 0,
		.async_format = (LCR_CS8 | LCR_1_STB | LCR_PDIS),
	},
	{
		.regs = PERIPH_ADDR_BASE_UART1,
		.sys_clk_freq = SYSCLK_DEFAULT_IOSC_HZ,
		.baud_rate = CONFIG_IPC_UART_BAUDRATE,
		.options = UART_OPTION_AFCE,
		.irq = IRQ_UART1_INTR,
		.int_pri = 0,
		.async_format = (LCR_CS8 | LCR_1_STB | LCR_PDIS),
	},
};

void uart_ipc_close_channel(int channel_id)
{
	channels[channel_id].state = IPC_CHANNEL_STATE_CLOSED;
	channels[channel_id].cb = NULL;
	channels[channel_id].index = channel_id;
}

void uart_ipc_disable(int num)
{
	int i;
	for (i = 0; i < IPC_UART_MAX_CHANNEL; i++)
		uart_ipc_close_channel(i);
	UART_IRQ_TX_DISABLE(num);
	UART_IRQ_RX_DISABLE(num);
}

void uart_ipc_init(int num)
{
	int i;
	(void)num;
	uint8_t c;

	for (i = 0; i < IPC_UART_MAX_CHANNEL; i++)
		uart_ipc_close_channel(i);

	pr_info(LOG_MODULE_IPC, "uart_ipc_init(nr: %d), baudrate %d, options:"
			"0x%x, irq: %d",IPC_UART,
			uart_dev_info[IPC_UART].baud_rate,
			uart_dev_info[IPC_UART].options,
			uart_dev_info[IPC_UART].irq);
	uart_init(IPC_UART, &uart_dev_info[IPC_UART]);
	/* Drain RX FIFOs (no need to disable IRQ at this stage) */
	while (uart_poll_in(IPC_UART, &c) != -1);
	uart_int_connect(IPC_UART, uart_ipc_isr, NULL, NULL);

	UART_IRQ_RX_ENABLE(IPC_UART);
}

void uart_ipc_set_channel(void * ipc_channel)
{
	ble_cfw_channel = ipc_channel;
}

void * uart_ipc_get_channel(void)
{
	return ble_cfw_channel;
}

void uart_ipc_push_frame(void) {
	void * frame;
	OS_ERR_TYPE error = E_OS_OK;

	if (NULL == ipc_uart_rx)
		return;
	int len = IPC_FRAME_GET_LEN(ipc_uart_rx);
	int channel = IPC_FRAME_GET_CHANNEL(ipc_uart_rx);
	uint8_t cpu_id = IPC_FRAME_GET_SRC(ipc_uart_rx);

	pr_debug(LOG_MODULE_IPC, "%s: received frame: len %d, channel %d, src "
			"%d", __func__, len, channel, cpu_id);

	if (channels[channel].cb != NULL) {
		frame = balloc(len, &error);
		if (error != E_OS_OK) {
			pr_error(LOG_MODULE_IPC, "NO MEM: error: %d size: %d",
					error, len);
		} else {
			memcpy(frame, &ipc_uart_rx[IPC_HEADER_LEN], len);

			channels[channel].cb(cpu_id, channel, len, frame);
		}
	}
	if (ipc_uart_rx)
		bfree(ipc_uart_rx);
	ipc_uart_rx = NULL;
}

void uart_ipc_isr()
{
	uint8_t *p_rx;
	uint8_t *p_tx;

	while (UART_IRQ_HW_UPDATE(IPC_UART) && UART_IRQ_IS_PENDING(IPC_UART)) {
		if (UART_IRQ_ERR_DETECTED(IPC_UART)) {
			uint8_t c;
			if (UART_BREAK_CHECK(IPC_UART)){
				panic();
			}
			UART_POLL_IN(IPC_UART, &c);
		} else if (UART_IRQ_RX_READY(IPC_UART)) {
			int received;
			if (received_counter < 2) {
				if (NULL == ipc_uart_rx)
					ipc_uart_rx =
						balloc(IPC_UART_MAX_PAYLOAD, NULL);
				p_rx = ipc_uart_rx;
				received = UART_FIFO_READ(IPC_UART,
						&p_rx[received_counter],
						1);
				received_counter += received;
			} else {
				p_rx = ipc_uart_rx;
				received = UART_FIFO_READ(IPC_UART,
						&p_rx[received_counter],
						IPC_FRAME_GET_LEN(p_rx) +
						IPC_HEADER_LEN
						- received_counter);
				received_counter += received;
				if (received_counter == IPC_FRAME_GET_LEN(p_rx)
						+ IPC_HEADER_LEN) {
#ifdef IPC_UART_DBG_RX
					for(int i = 0; i < received_counter; i++) {
						pr_debug(LOG_MODULE_IPC, "%s: %d byte is %d", __func__, i, p_rx[i]);
					}
#endif
					received_counter = 0;
					uart_ipc_push_frame();
				}
			}
		} else if (UART_IRQ_TX_READY(IPC_UART)) {
			int transmitted;
			if (ipc_uart_tx_state == STATUS_TX_IDLE) {
				uint8_t lsr = UART_LINE_STATUS(IPC_UART);
				UART_IRQ_TX_DISABLE(IPC_UART);

				pr_debug(LOG_MODULE_IPC, "ipc_isr_tx: disable TXint, LSR: 0x%2x\n",
						lsr);
				/* wait for FIFO AND THR being empty! */
				while ((lsr & BOTH_EMPTY) != BOTH_EMPTY) {
					lsr = UART_LINE_STATUS(IPC_UART);
				}
				return;
			}
			if(NULL == ipc_uart_tx){
				pr_warning(LOG_MODULE_IPC, "%s: Bad Tx data",__func__);
				return;
			}
			p_tx = ipc_uart_tx;
			transmitted = UART_FIFO_FILL(IPC_UART, &p_tx[send_counter],
					IPC_FRAME_GET_LEN(p_tx) +
					IPC_HEADER_LEN - send_counter);
			send_counter += transmitted;
			if (send_counter == IPC_FRAME_GET_LEN(p_tx) +
					IPC_HEADER_LEN) {
				send_counter = 0;
#ifdef IPC_UART_DBG_TX
				pr_debug(LOG_MODULE_IPC, "%s: sent IPC FRAME "
						"len %d", __func__,
						IPC_FRAME_GET_LEN(p_tx));
				for (int i = 0; i < send_counter; i++) {
					pr_debug(LOG_MODULE_IPC, "%s: %d  sent "
							"byte is %d",
							__func__, i, p_tx[i]);
				}
#endif
				bfree(ipc_uart_tx);
				ipc_uart_tx = NULL;

				ipc_uart_tx_state = STATUS_TX_IDLE;
#ifdef IPC_UART_DBG_TX
				uint8_t lsr = UART_LINE_STATUS(IPC_UART);
				pr_info(LOG_MODULE_IPC, "ipc_isr_tx: tx_idle LSR: 0x%2x\n",
						lsr);
#endif
			}
		} else {
			pr_warning(LOG_MODULE_IPC, "%s: Unknown ISR src",
					__func__);
		}
	}
}

void *uart_ipc_channel_open(int channel_id,
			void (*cb) (uint8_t, int, int, void *))
{
	struct ipc_uart_channels *chan;

	if (channel_id > IPC_UART_MAX_CHANNEL - 1)
		return NULL;

	chan = &channels[channel_id];

	if (chan->state != IPC_CHANNEL_STATE_CLOSED)
		return NULL;

	chan->state = IPC_CHANNEL_STATE_OPEN;
	chan->cb = cb;

	return chan;
}

int uart_ipc_send_message(void *handle, int len, void *p_data)
{
	struct ipc_uart_channels *chan = (struct ipc_uart_channels *) handle;

	int flags = interrupt_lock();
	if (ipc_uart_tx_state == STATUS_TX_BUSY) {
		interrupt_unlock(flags);
		return IPC_UART_ERROR_WRONG_STATE;
	}
	ipc_uart_tx_state = STATUS_TX_BUSY;
	interrupt_unlock(flags);

	uint8_t *p_tx = ipc_uart_tx = balloc(len + IPC_UART_HDR_REQUEST_LEN,
			NULL);

	/* Adding size of the message request field*/
	int size = len + sizeof(uint32_t);

	/* length = cfw_message size + message request field*/
	IPC_FRAME_SET_LEN(p_tx, size);
	IPC_FRAME_SET_CHANNEL(p_tx, chan->index);
	IPC_FRAME_SET_SRC(p_tx, get_cpu_id());
	IPC_FRAME_SET_REQUEST(p_tx, IPC_MSG_TYPE_MESSAGE);

	/* IPC_HEADER + request_ID + cfw_message */
	/* copy cfw_message within IPC frame*/
	memcpy(IPC_FRAME_DATA(p_tx), p_data, len);

	pr_debug(LOG_MODULE_IPC, "%s: tx: channel %d, len %d, request 0x%x",
			__func__, p_tx[2], len, p_tx[4]);

	UART_IRQ_TX_ENABLE(IPC_UART);

	return IPC_UART_ERROR_OK;
}

int uart_ipc_send_sync_resp(int channel, int request_id, int param1, int param2,
			    void * ptr)
{
	if (ipc_uart_tx_state == STATUS_TX_BUSY)
		return IPC_UART_ERROR_WRONG_STATE;
	ipc_uart_tx_state = STATUS_TX_BUSY;

	uint8_t *p_tx = ipc_uart_tx = balloc(IPC_UART_HDR_REQUEST_LEN + 12,
			NULL);

	IPC_FRAME_SET_LEN(p_tx, 16);
	IPC_FRAME_SET_CHANNEL(p_tx, channel);
	IPC_FRAME_SET_SRC(ipc_uart_tx, get_cpu_id());

	IPC_FRAME_SET_REQUEST(p_tx, request_id);
	SYNC_FRAME_SET_PARAM1(p_tx, param1);
	SYNC_FRAME_SET_PARAM2(p_tx, param2);
	SYNC_FRAME_SET_PTR(p_tx, ptr);

#ifdef IPC_UART_DBG_SYNC_RESP
	for (int i = 0; i < 20; i++) {
		pr_debug(LOG_MODULE_IPC, "%s: IPC sync resp %d byte : %d",
				__func__,i, p_tx[i]);
	}
	pr_debug(LOG_MODULE_IPC, "%s: tx: channel %d, request %xh", __func__,
			p_tx[2], p_tx[4]);
#endif

	UART_IRQ_TX_ENABLE(IPC_UART);

	return IPC_UART_ERROR_OK;
}
