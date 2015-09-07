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

#include "nordic_interface.h"

/* FIXME: Service manager API */
extern void _cfw_init(void *);

extern void *services;

#define IPC_QUEUE_DEPTH 64

static T_QUEUE service_mgr_queue;

static const uint8_t ipc_tx_chan = 5;
static const uint8_t ipc_rx_chan = 0;
static const uint8_t ipc_tx_ack_chan = 6;
static const uint8_t ipc_rx_ack_chan = 1;
static const uint8_t ipc_remote_cpu = CPU_ID_LMT;

/* ISR Callback to handle new messages received via Mailbox from LMT */
static void ipc_mbx_isr(void)
{
    while (MBX_STS(ipc_rx_chan) & 0x2) {
        /* Pop a message from the h/w mailbox FIFO, process it, and ack it */
        ipc_handle_message();
        while((queue_process_message(service_mgr_queue)) != 0);
    }
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


void cfw_platform_nordic_init(void)
{
    /* Setup UART0 for BLE communication, HW flow control required  */
    SET_PIN_MODE(18, QRK_PMUX_SEL_MODEA); /* UART0_RXD        */
    SET_PIN_MODE(19, QRK_PMUX_SEL_MODEA); /* UART0_TXD        */
    SET_PIN_MODE(40, QRK_PMUX_SEL_MODEB); /* UART0_CTS_B      */
    SET_PIN_MODE(41, QRK_PMUX_SEL_MODEB); /* UART0_RTS_B      */

    /* Reset the nordic to force sync - Warning: not working everytime */
    nordic_interface_init(service_mgr_queue);
    uart_ipc_init(0);
    uart_ipc_set_channel(uart_ipc_channel_open(SYNC_CHANNEL, uart_ipc_message_cback));
}

static void cfw_platform_mbx_int_enable(void)
{
    /* Set up mailbox interrupt handler */
    interrupt_connect(SOC_MBOX_INTERRUPT, ipc_mbx_isr);
    interrupt_enable(SOC_MBOX_INTERRUPT);
    /* Enable interrupt for ARC IPC rx channel */
    SOC_MBX_INT_UNMASK(ipc_rx_chan);
}

/* Initialise the IPC framework */
void cfw_platform_init(void)
{
    /* CFW IPC initialisation */
    ipc_init(ipc_tx_chan, ipc_rx_chan,
             ipc_tx_ack_chan, ipc_rx_ack_chan,
             ipc_remote_cpu);

    set_cpu_id(CPU_ID_ARC);
    set_cpu_message_sender(ipc_remote_cpu, send_message_ipc);
    set_cpu_free_handler(ipc_remote_cpu, free_message_ipc);
    set_cpu_message_sender(CPU_ID_BLE, send_message_ipc_uart);
    set_cpu_free_handler(CPU_ID_BLE, free_message_ipc_uart);

    service_mgr_queue = queue_create(IPC_QUEUE_DEPTH, NULL);

#ifndef CONFIG_INFRA_IS_MASTER
    cfw_platform_mbx_int_enable();

    /* Notify LMT that ARC started. */
    shared_data->arc_ready = 1;

    _cfw_init_proxy(service_mgr_queue, shared_data->ports,
                    shared_data->services, shared_data->service_mgr_port_id);
#else
    _cfw_init(service_mgr_queue);

    /* Initialized shared structure. */
    shared_data->ports = port_get_port_table();
    shared_data->services = services;
    shared_data->service_mgr_port_id = cfw_get_service_mgr_port_id();

    cfw_platform_mbx_int_enable();

    /* Notify LMT that ARC started. */
    shared_data->arc_ready = 1;
#endif
}

T_QUEUE cfw_get_service_queue(void)
{
    return service_mgr_queue;
}

#ifdef __cplusplus
}
#endif
