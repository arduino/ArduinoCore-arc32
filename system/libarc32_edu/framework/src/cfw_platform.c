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

#include <stdarg.h>
#include <string.h>

#include "portable.h"
#include "os/os_types.h"
#include "infra/ipc.h"
#include "cfw/cfw.h"
#include "cfw/cfw_service.h"
#include "platform.h"
#include "cfw_platform.h"

#include "cfw/cfw_messages.h"
#include "services/test_service.h"

#define IPC_QUEUE_DEPTH 64

static T_QUEUE service_mgr_queue;

static const uint8_t ipc_tx_chan = 5;
static const uint8_t ipc_rx_chan = 0;
static const uint8_t ipc_tx_ack_chan = 6;
static const uint8_t ipc_rx_ack_chan = 1;
static const uint8_t ipc_remote_cpu = CPU_ID_LMT;

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

#ifdef CONFIG_CFW_SERVICES_TEST

svc_client_handle_t * test_service_handle = NULL;


static void my_handle_message(struct cfw_message * msg, void * param)
{
    switch (CFW_MESSAGE_ID(msg)) {
    case MSG_ID_CFW_OPEN_SERVICE: {
        cfw_open_conn_rsp_msg_t * cnf = (cfw_open_conn_rsp_msg_t*)msg;
        int events[1] = {MSG_ID_TEST_1_EVT};
        test_service_handle = cnf->client_handle;
        cfw_register_events(test_service_handle, events,
                            1, CFW_MESSAGE_PRIV(msg));
        }
        break;

    case MSG_ID_CFW_REGISTER_EVT:
        if (!strcmp(msg->priv, "conn1")) {
            test_service_test_1(test_service_handle, "Coucou");
        }
        break;

    case MSG_ID_TEST_1_RSP: {
        /* Light the COM LED */
        SET_MMIO_BIT(SOC_GPIO_BASE_ADDR + SOC_GPIO_SWPORTA_DDR, 12);
        CLEAR_MMIO_BIT(SOC_GPIO_BASE_ADDR + SOC_GPIO_SWPORTA_DR, 12);
        test_service_test_2(test_service_handle, "Testing 2");
        break;
        }

    case MSG_ID_TEST_2_RSP: {
        /* Light the FAULT LED */
        SET_MMIO_BIT(SOC_GPIO_BASE_ADDR + SOC_GPIO_SWPORTA_DDR, 26);
        CLEAR_MMIO_BIT(SOC_GPIO_BASE_ADDR + SOC_GPIO_SWPORTA_DR, 26);
        //test_service_test_1(test_service_handle, "Coucou");
        break;
        }

    case MSG_ID_TEST_1_EVT: {
        break;
        }
    }
    cfw_msg_free(msg);
}

static void test_client_init(T_QUEUE queue)
{
    cfw_handle_t *h = cfw_init(service_mgr_queue, my_handle_message, "client");

    cfw_open_service(h, TEST_SERVICE_ID, "conn1");
}

#endif

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

#ifdef CONFIG_CFW_SERVICES_TEST
    test_client_init(service_mgr_queue);
#endif
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
