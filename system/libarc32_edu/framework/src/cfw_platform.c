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
#include "infra/ipc.h"
#include "cfw/cfw.h"
#include "cfw/cfw_service.h"
#include "platform.h"
#include "cfw_platform.h"

#define IPC_QUEUE_DEPTH 10

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

static int send_message_ipc(struct cfw_message * msg) {
    return ipc_request_sync_int(IPC_MSG_TYPE_MESSAGE, 0, 0, msg);
}

static void free_message_ipc(void * msg) {
    ipc_request_sync_int(IPC_MSG_TYPE_FREE, 0, 0, msg);
}

#ifdef __cplusplus
extern "C" {
#endif

#define TMP_SIZE 256
static char tmp[TMP_SIZE];

/* Sends a log message to LMT for display on a debug console */
void cfw_log(char * fmt, ... )
{
    va_list args;
    uint32_t timeout;
    va_start(args, fmt);
    unsigned int tstamp = SCSS_REG_VAL(SCSS_AONC_CNT);
    int hours = tstamp / (32191 * 60 * 60);
    tstamp -= hours * (32191 * 60 * 60);
    int minutes = (tstamp) / (32191 * 60);
    tstamp -= minutes * (32191 * 60);
    int seconds = (tstamp) / 32191;
    tstamp -= seconds * 32191;
    int millis = (tstamp) / 32;
    int it = interrupt_lock();

    sprintf(tmp, "%03d:%02d:%02d.%03d  ", hours, minutes, seconds, millis);
    vsnprintf(tmp + strlen(tmp) - 1, TMP_SIZE, fmt, args);
    MBX_DAT0(4) = (unsigned int)tmp;
    MBX_DAT1(4) = 0;
    MBX_CTRL(4) = 0x80000000;
    timeout = get_timestamp() + 10000;
    while((MBX_STS(4) & 0x1) && (get_timestamp() < timeout))
        ;

    interrupt_unlock(it);
    va_end(args);
}

/* Initialise the IPC framework */
void cfw_platform_init(bool irq_enable)
{
    /* CFW IPC initialisation */
    ipc_init(ipc_tx_chan, ipc_rx_chan,
             ipc_tx_ack_chan, ipc_rx_ack_chan,
             ipc_remote_cpu);
    service_mgr_queue = queue_create(IPC_QUEUE_DEPTH, NULL);
    _cfw_init_proxy(service_mgr_queue, shared_data->ports,
                    shared_data->services, shared_data->service_mgr_port_id);
    set_cpu_id(CPU_ID_ARC);
    set_cpu_message_sender(ipc_remote_cpu, send_message_ipc);
    set_cpu_free_handler(ipc_remote_cpu, free_message_ipc);

    if (irq_enable) {
        /* Set up mailbox interrupt handler */
        interrupt_connect(SOC_MBOX_INTERRUPT, ipc_mbx_isr, NULL);
        interrupt_enable(SOC_MBOX_INTERRUPT);
        SOC_UNMASK_INTERRUPTS(INT_MAILBOX_MASK);
        /* Enable interrupt for ARC IPC rx channel */
        SOC_MBX_INT_UNMASK(ipc_rx_chan);
    }
    ipc_irq_enabled = irq_enable;
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
