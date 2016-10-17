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

enum {
	STATUS_TX_IDLE = 0,
	STATUS_TX_BUSY,
	STATUS_TX_DONE,
};

enum {
	STATUS_RX_IDLE = 0,
	STATUS_RX_HDR,
	STATUS_RX_DATA
};

struct ipc_uart {
	uint8_t *tx_data;
	uint8_t *rx_ptr;
	struct ipc_uart_channels channels[IPC_UART_MAX_CHANNEL];
	struct ipc_uart_header tx_hdr;
	struct ipc_uart_header rx_hdr;
	uint16_t send_counter;
	uint16_t rx_size;
	uint8_t tx_state;
	uint8_t rx_state;
	uint8_t uart_enabled;
	/* protect against multiple wakelock and wake assert calls */
	uint8_t tx_wakelock_acquired;
	/* TODO: remove once IRQ will take a parameter */
	//struct td_device *device;
	void (*tx_cb)(bool wake_state, void *); /*!< Callback to be called to set wake state when TX is starting or ending */
	void *tx_cb_param;              /*!< tx_cb function parameter */
};

static struct ipc_uart ipc = {};

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

void ipc_uart_close_channel(int channel_id)
{
	ipc.channels[channel_id].state = IPC_CHANNEL_STATE_CLOSED;
	ipc.channels[channel_id].cb = NULL;
	ipc.channels[channel_id].index = channel_id;

	ipc.uart_enabled = 0;
	ipc.tx_wakelock_acquired = 0;
}

void ipc_uart_ns16550_disable(int num)
{
	int i;
	for (i = 0; i < IPC_UART_MAX_CHANNEL; i++)
		ipc_uart_close_channel(i);
	if (ipc.tx_cb)
		ipc.tx_cb(0, ipc.tx_cb_param);
    
	UART_IRQ_TX_DISABLE(num);
	UART_IRQ_RX_DISABLE(num);
}

void ipc_uart_init(int num)
{
	int i;
	(void)num;

	for (i = 0; i < IPC_UART_MAX_CHANNEL; i++)
		ipc_uart_close_channel(i);

	pr_info(LOG_MODULE_IPC, "%s(nr: %d), baudrate %d, options:"
			"0x%x, irq: %d",__FUNCTION__, IPC_UART,
			uart_dev_info[IPC_UART].baud_rate,
			uart_dev_info[IPC_UART].options,
			uart_dev_info[IPC_UART].irq);
	uart_init(IPC_UART, &uart_dev_info[IPC_UART]);
	
	ipc.uart_enabled = 0;
	ipc.tx_wakelock_acquired = 0;

	/* Initialize the reception pointer */
	ipc.rx_size = sizeof(ipc.rx_hdr);
	ipc.rx_ptr = (uint8_t *)&ipc.rx_hdr;
	ipc.rx_state = STATUS_RX_IDLE;
}

static void ipc_uart_push_frame(uint16_t len, uint8_t *p_data)
{
	//pr_debug(LOG_MODULE_IPC, "push_frame: received:frame len: %d, p_data: "
	//	 "len %d, src %d, channel %d", ipc.rx_hdr.len, len,
	//	 ipc.rx_hdr.src_cpu_id,
	//	 ipc.rx_hdr.channel);
    //pr_debug(LOG_MODULE_IPC,"data[0 - 1]: %x-%x", p_data[0], p_data[1]);

	if ((ipc.rx_hdr.channel < IPC_UART_MAX_CHANNEL) &&
	    (ipc.channels[ipc.rx_hdr.channel].cb != NULL)) {
		ipc.channels[ipc.rx_hdr.channel].cb(ipc.rx_hdr.channel,
						    IPC_MSG_TYPE_MESSAGE,
						    len,
						    p_data);
	} else {
		bfree(p_data);
		pr_error(LOG_MODULE_IPC, "uart_ipc: bad channel %d",
			 ipc.rx_hdr.channel);
	}
}

void ipc_uart_isr()
{
    /* TODO: remove once IRQ supports parameter */
    uint8_t *p_tx;

    while (UART_IRQ_HW_UPDATE(IPC_UART) && 
           UART_IRQ_IS_PENDING(IPC_UART)) {
        if (UART_IRQ_ERR_DETECTED(IPC_UART))
        {
            uint8_t c;
            if (UART_BREAK_CHECK(IPC_UART)) {
                panic(-1);
            }
            UART_POLL_IN(IPC_UART, &c);
        }
        if (UART_IRQ_RX_READY(IPC_UART)) {
            int rx_cnt;

            while ((rx_cnt =
                    UART_FIFO_READ(IPC_UART,
                               ipc.rx_ptr,
                               ipc.rx_size)) != 0)
            {
                if ((ipc.uart_enabled) &&
                    (ipc.rx_state == STATUS_RX_IDLE)) {
                    /* acquire wakelock until frame is fully received */
                    //pm_wakelock_acquire(&info->rx_wl);
                    ipc.rx_state = STATUS_RX_HDR;
                }

                /* Until UART has enabled at least one channel, data should be discarded */
                if (ipc.uart_enabled) {
                    ipc.rx_size -= rx_cnt;
                    ipc.rx_ptr += rx_cnt;
                }

                if (ipc.rx_size == 0) {
                    if (ipc.rx_state == STATUS_RX_HDR) {
    //pr_error(0, "%s-%d", __FUNCTION__, ipc.rx_hdr.len);
                        ipc.rx_ptr = balloc(
                            ipc.rx_hdr.len, NULL);
                        
                            //pr_debug(
                            //  LOG_MODULE_IPC,
                            //  "ipc_uart_isr: rx_ptr is %p",
                            //  ipc.rx_ptr);
                        ipc.rx_size = ipc.rx_hdr.len;
                        ipc.rx_state = STATUS_RX_DATA;
                    } else {
#ifdef IPC_UART_DBG_RX
                        uint8_t *p_rx = ipc.rx_ptr -
                                ipc.rx_hdr.len;
                        for (int i = 0;
                             i < ipc.rx_hdr.len;
                             i++) {
                            pr_debug(
                                LOG_MODULE_IPC,
                                "ipc_uart_isr: %d byte is %d",
                                i, p_rx[i]);
                        }
#endif

                        ipc_uart_push_frame(
                            ipc.rx_hdr.len,
                            ipc.rx_ptr -
                            ipc.rx_hdr.len);
                        ipc.rx_size = sizeof(ipc.rx_hdr);
                        ipc.rx_ptr =
                            (uint8_t *)&ipc.rx_hdr;
                        ipc.rx_state = STATUS_RX_IDLE;
                    }
                }
            }
        } 
        if (UART_IRQ_TX_READY(IPC_UART)) {
            int tx_len;

            if (ipc.tx_state == STATUS_TX_DONE) {
                uint8_t lsr = UART_LINE_STATUS(IPC_UART);
                ipc.tx_state = STATUS_TX_IDLE;
                UART_IRQ_TX_DISABLE(IPC_UART);
                
                /* wait for FIFO AND THR being empty! */
                while ((lsr & BOTH_EMPTY) != BOTH_EMPTY) {
                    lsr = UART_LINE_STATUS(IPC_UART);
                }

                /* No more TX activity, send event and release wakelock */
                if (ipc.tx_cb) {
                    ipc.tx_cb(0, ipc.tx_cb_param);
                }
                //pm_wakelock_release(&info->tx_wl);
                ipc.tx_wakelock_acquired = 0;
                return;
            }
            if (NULL == ipc.tx_data) {
                pr_warning(LOG_MODULE_IPC,
                       "ipc_uart_isr: Bad Tx data");
                return;
            }

            if (!ipc.tx_wakelock_acquired) {
                ipc.tx_wakelock_acquired = 1;
                /* Starting TX activity, send wake assert event and acquire wakelock */
                if (ipc.tx_cb) {
                    ipc.tx_cb(1, ipc.tx_cb_param);
                }
                //pm_wakelock_acquire(&info->tx_wl);
            }
            if (ipc.send_counter < sizeof(ipc.tx_hdr)) {
                p_tx = (uint8_t *)&ipc.tx_hdr +
                       ipc.send_counter;
                tx_len = sizeof(ipc.tx_hdr) - ipc.send_counter;
            } else {
                p_tx = ipc.tx_data +
                       (ipc.send_counter - sizeof(ipc.tx_hdr));
                tx_len = ipc.tx_hdr.len -
                     (ipc.send_counter - sizeof(ipc.tx_hdr));
            }
            ipc.send_counter += UART_FIFO_FILL(IPC_UART, 
                                               p_tx,
                                               tx_len);

            if (ipc.send_counter ==
                (ipc.tx_hdr.len + sizeof(ipc.tx_hdr))) {
                ipc.send_counter = 0;
#ifdef IPC_UART_DBG_TX
                pr_debug(
                    LOG_MODULE_IPC,
                    "ipc_uart_isr: sent IPC FRAME "
                    "len %d", ipc.tx_hdr.len);
#endif

                p_tx = ipc.tx_data;
                ipc.tx_data = NULL;
                ipc.tx_state = STATUS_TX_DONE;

                /* free sent message and pull send next frame one in the queue */
                if (ipc.channels[ipc.tx_hdr.channel].cb)
                {
                    ipc.channels[ipc.tx_hdr.channel].cb(
                        ipc.tx_hdr.channel,
                        IPC_MSG_TYPE_FREE,
                        ipc.tx_hdr.len,
                        p_tx);
                }
                else
                {
                    bfree(p_tx);
                }
                
#ifdef IPC_UART_DBG_TX
                uint8_t lsr = UART_LINE_STATUS(IPC_UART);//(info->uart_num);
                pr_debug(LOG_MODULE_IPC,
                     "ipc_isr_tx: tx_idle LSR: 0x%2x\n",
                     lsr);
#endif
            }
        }
        
    }
}

void *ipc_uart_channel_open(int channel_id,
			    int (*cb)(int, int, int, void *))
{
	struct ipc_uart_channels *chan;
	uint8_t c;

	if (channel_id > (IPC_UART_MAX_CHANNEL - 1))
		return NULL;

	chan = &ipc.channels[channel_id];

	if (chan->state != IPC_CHANNEL_STATE_CLOSED)
		return NULL;

	chan->state = IPC_CHANNEL_STATE_OPEN;
	chan->cb = cb;

	ipc.uart_enabled = 1;
	ipc.tx_wakelock_acquired = 0;
	
    pr_debug(LOG_MODULE_IPC, "%s: open chan success", __FUNCTION__);
    
	/* Drain RX FIFOs (no need to disable IRQ at this stage) */
	while (uart_poll_in(IPC_UART, &c) != -1);
	uart_int_connect(IPC_UART, ipc_uart_isr, NULL, NULL);
	
	UART_IRQ_RX_ENABLE(IPC_UART);
	
	return chan;
}

int ipc_uart_ns16550_send_pdu(void *handle, int len, void *p_data)
{
	struct ipc_uart_channels *chan = (struct ipc_uart_channels *)handle;

    //pr_debug(LOG_MODULE_IPC, "%s: %d", __FUNCTION__, ipc.tx_state);

	if (ipc.tx_state == STATUS_TX_BUSY) {
		return IPC_UART_TX_BUSY;
	}
	
	/* It is eventually possible to be in DONE state (sending last bytes of previous message),
	 * so we move immediately to BUSY and configure the next frame */
	ipc.tx_state = STATUS_TX_BUSY;

	ipc.tx_hdr.len = len;
	ipc.tx_hdr.channel = chan->index;
	ipc.tx_hdr.src_cpu_id = 0;
	ipc.tx_data = p_data;

	/* Enable the interrupt (ready will expire if it was disabled) */
	UART_IRQ_TX_ENABLE(IPC_UART);

	return IPC_UART_ERROR_OK;
}

void ipc_uart_ns16550_set_tx_cb(void (*cb)(bool, void *), void *param)
{
	ipc.tx_cb = cb;
	ipc.tx_cb_param = param;
}

