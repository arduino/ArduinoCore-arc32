/** INTEL CONFIDENTIAL Copyright 2014 Intel Corporation All Rights Reserved.
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

#include <stdint.h>
#include "os/os_types.h"
#include "infra/ipc.h"
#include "cfw/cfw.h"
#include "soc_register.h"
#include "infra/log.h"

#define MBX_IPC_SYNC_ARC_TO_LMT 5

static int rx_chan = 0;
static int tx_chan = 0;
static int tx_ack_chan = 0;
static int rx_ack_chan = 0;
static int remote_cpu = 0;

/*****************************************************************************
 * IPC Protocol:
 * 2 Mailboxes per side.
 * One mailbox for passing data, one mailbox for acknowledging the transfer
 *
 ****************************************************************************/

unsigned int get_timestamp()
{
    return SCSS_REG_VAL(SCSS_AONC_CNT);
}

void ipc_init(int tx_channel, int rx_channel, int tx_ack_channel,
    int rx_ack_channel, int remote_cpu_id)
{
    rx_chan = rx_channel;
    tx_chan = tx_channel;
    tx_ack_chan = tx_ack_channel;
    rx_ack_chan = rx_ack_channel;
    remote_cpu = remote_cpu_id;
}

void ipc_handle_message()
{
    int ret = 0;
    if (!MBX_STS(rx_chan)) return;
    int request = MBX_DAT0(rx_chan);
    int param1 = MBX_DAT1(rx_chan);
    int param2 = MBX_DAT2(rx_chan);
    void * ptr = (void *)MBX_DAT3(rx_chan);

    ret = ipc_sync_callback(remote_cpu, request, param1, param2, ptr);

    MBX_CTRL(rx_chan) = 0x80000000;

    MBX_DAT0(tx_ack_chan) = ret;
    MBX_CTRL(tx_ack_chan) = 0x80000000;
    pr_debug(LOG_MODULE_MAIN, "read message on %d : ack [%d] %p", rx_chan, tx_ack_chan,
            MBX_DAT0(tx_ack_chan));
    MBX_STS(rx_chan) = 3;
}

int ipc_request_sync_int(int request_id, int param1, int param2, void * ptr)
{
    int ret;
    int timeout;

    pr_debug(LOG_MODULE_MAIN, "send request %d from: %p", request_id, &ret);
    while (MBX_CTRL(tx_chan) & 0x80000000) {
        pr_info(LOG_MODULE_MAIN, "Channel busy %d for request: %d msg: %p",
                tx_chan, request_id, param1);
        pr_info(LOG_MODULE_MAIN, "current request: %p msg: %p",
                MBX_CTRL(tx_chan), MBX_DAT0(tx_chan));
    }

    MBX_STS(rx_ack_chan) = 3;

    MBX_DAT0(tx_chan) = request_id;
    MBX_DAT1(tx_chan) = param1;
    MBX_DAT2(tx_chan) = param2;
    MBX_DAT3(tx_chan) = (unsigned int )ptr;
    MBX_CTRL(tx_chan) = 0x80000000 | IPC_MSG_TYPE_SYNC;

    timeout = get_timestamp() + 32768;
    while(!MBX_STS(rx_ack_chan)) {
        if (get_timestamp() > timeout) {
            pr_error(LOG_MODULE_MAIN, "Timeout waiting ack %p", &ret);
            break;
        }
    }
    ret = MBX_DAT0(rx_ack_chan);
    MBX_DAT0(rx_ack_chan) = 0;
    MBX_STS(rx_ack_chan) = 3;
    pr_debug(LOG_MODULE_MAIN, "ipc_request_sync returns: [%d] %p", rx_ack_chan, ret);
    return ret;
}
