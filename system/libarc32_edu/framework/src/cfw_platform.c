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

#include <stdarg.h>
#include <string.h>

#include "portable.h"
#include "os/os_types.h"
#include "infra/ipc.h"
#include "infra/log.h"
#include "cfw/cfw.h"
#include "cfw/cfw_service.h"
#include "platform.h"
#include "cfw_platform.h"

#include "cfw/cfw_messages.h"
#include "services/cdc_serial_service.h"

#define IPC_QUEUE_DEPTH 64

static T_QUEUE service_mgr_queue;

static const uint8_t ipc_tx_chan = 5;
static const uint8_t ipc_rx_chan = 0;
static const uint8_t ipc_tx_ack_chan = 6;
static const uint8_t ipc_rx_ack_chan = 1;
static const uint8_t ipc_remote_cpu = CPU_ID_LMT;

static cfw_handle_t *cfw_handle;

volatile int cdc_service_available = 0;

static uint8_t ipc_irq_enabled;

/* ISR Callback to handle new messages received via Mailbox from LMT */
static void ipc_mbx_isr(void)
{
    while (MBX_STS(ipc_rx_chan) & 0x2) {
        /* Pop a message from the h/w mailbox FIFO into s/w queue, and ack it */
        ipc_handle_message();
        /* Pop the message from the s/w queue and process it
         * (CAUTION: may invoke application callbacks!)
         */
        queue_process_message(service_mgr_queue);
    }

    /* TODO - this is effectively putting messages into a software queue
     * and then taking them out again - trading efficiency for code re-use.
     * Consider optimising this later by implementing a queue-bypass option
     * in framework/src/infra/port.c
     */
}

static int send_message_ipc(struct message *msg) {
    return ipc_request_sync_int(IPC_MSG_TYPE_MESSAGE, 0, 0, msg);
}

static void free_message_ipc(struct message *msg) {
    ipc_request_sync_int(IPC_MSG_TYPE_FREE, 0, 0, msg);
}

#ifdef __cplusplus
extern "C" {
#endif

svc_client_handle_t * cdc_serial_service_handle = NULL;

static void my_handle_message(struct cfw_message * msg, void * param)
{
	cfw_open_conn_rsp_msg_t *cnf;
	int events[1] = {MSG_ID_CDC_SERIAL_RX_EVT};

    switch (CFW_MESSAGE_ID(msg)) {
		case MSG_ID_CFW_OPEN_SERVICE:
			cnf = (cfw_open_conn_rsp_msg_t*)msg;
			cdc_serial_service_handle = cnf->client_handle;
			cfw_register_events(cdc_serial_service_handle, events,
								1, CFW_MESSAGE_PRIV(msg));
			break;

		case MSG_ID_CFW_REGISTER_EVT:
			cdc_service_available = 1;
			break;

		case MSG_ID_CFW_REGISTER_SVC_AVAIL:
			cfw_open_service(cfw_handle, CDC_SERIAL_SERVICE_ID, "conn1");
			break;

		case MSG_ID_CDC_SERIAL_TX_ACK:
			// Tell the CDCSerialClass that the Tx has been done.
			cdc_serial_service_sent(msg);
			break;

		case MSG_ID_CDC_SERIAL_RX_EVT:
			// Tell the CDCSerialClass that Rx has been done.
			cdc_serial_service_receive(msg);
			break;
	}
    cfw_msg_free(msg);
}

static void cdc_serial_client_init(T_QUEUE queue)
{
	cfw_handle = cfw_init(service_mgr_queue, my_handle_message, "client");

}


/* Initialise the IPC framework */
void cfw_platform_init(bool irq_enable)
{
    /* CFW IPC initialisation */
    ipc_init(ipc_tx_chan, ipc_rx_chan,
             ipc_tx_ack_chan, ipc_rx_ack_chan,
             ipc_remote_cpu);

    if (irq_enable) {
        /* Set up mailbox interrupt handler */
        interrupt_connect(SOC_MBOX_INTERRUPT, ipc_mbx_isr);
        interrupt_enable(SOC_MBOX_INTERRUPT);
        /* Enable interrupt for ARC IPC rx channel */
        SOC_MBX_INT_UNMASK(ipc_rx_chan);
    }
    ipc_irq_enabled = irq_enable;

    set_cpu_id(CPU_ID_ARC);

    /* Notify LMT that ARC started. */
    shared_data->arc_ready = 1;

    service_mgr_queue = queue_create(IPC_QUEUE_DEPTH, NULL);

    _cfw_init_proxy(service_mgr_queue, shared_data->ports,
                    shared_data->services, shared_data->service_mgr_port_id);
    set_cpu_message_sender(ipc_remote_cpu, send_message_ipc);
    set_cpu_free_handler(ipc_remote_cpu, free_message_ipc);

    cdc_serial_client_init(service_mgr_queue);
	cfw_register_svc_available(cfw_handle, CDC_SERIAL_SERVICE_ID, NULL);
}

/* Poll for new messages received via Mailbox from LMT */
void cfw_platform_loop(void)
{
    /* Should not normally be called if IPC interrupts enabled */
    if (ipc_irq_enabled)
        return;

    ipc_handle_message();
    queue_process_message(service_mgr_queue);
}

#ifdef __cplusplus
}
#endif
