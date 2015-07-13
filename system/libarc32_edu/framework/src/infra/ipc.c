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

#include <stdint.h>
#include "os/os_types.h"
#include "infra/ipc.h"
#include "infra/port.h"
#include "infra/message.h"
#include "soc_register.h"
#include "infra/log.h"

#define MBX_IPC_SYNC_ARC_TO_LMT 5

static int rx_chan = 0;
static int tx_chan = 0;
static int tx_ack_chan = 0;
static int rx_ack_chan = 0;
static uint8_t remote_cpu = 0;

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

static T_MUTEX ipc_mutex;

void ipc_init(int tx_channel, int rx_channel, int tx_ack_channel,
    int rx_ack_channel, uint8_t remote_cpu_id)
{
    rx_chan = rx_channel;
    tx_chan = tx_channel;
    tx_ack_chan = tx_ack_channel;
    rx_ack_chan = rx_ack_channel;
    remote_cpu = remote_cpu_id;
    ipc_mutex = mutex_create(NULL);
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

    ret = mutex_lock(ipc_mutex, OS_WAIT_FOREVER);
    if (ret != E_OS_OK) {
        pr_error(LOG_MODULE_MAIN, "Error locking ipc %d", ret);
        return ret;
    }
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
            pr_error(LOG_MODULE_MAIN, "Timeout waiting ack %p", request_id);
            break;
        }
    }
    ret = MBX_DAT0(rx_ack_chan);
    MBX_DAT0(rx_ack_chan) = 0;
    MBX_STS(rx_ack_chan) = 3;
    pr_debug(LOG_MODULE_MAIN, "ipc_request_sync returns: [%d] %p", rx_ack_chan, ret);
    mutex_unlock(ipc_mutex, NULL);
    return ret;
}

#define IPC_MESSAGE_SEND 1
#define IPC_MESSAGE_FREE 2

static uint16_t ipc_port;

struct ipc_async_msg {
	struct message h;
	void * data;
};

/**
 * \brief this function is called in the context of the queue set by
 * ipc_async_init().
 * When the ipc_async_send_message() and ipc_async_free_message() are called,
 * A message is generated and sent to the port whose callback is this function.
 *
 * \param m the message to handle
 * \param data the parameter passed to port_set_handler()
 */
static void handle_ipc_request_port(struct message *m, void *data)
{
	struct ipc_async_msg * msg = (struct ipc_async_msg *)m;
	switch(MESSAGE_ID(&msg->h)) {
		case IPC_MESSAGE_SEND:
			pr_debug(LOG_MODULE_MAIN, "Send message: %p",
				 msg->data);
			ipc_request_sync_int(IPC_MSG_TYPE_MESSAGE,
					     0, 0, msg->data);
			break;
		case IPC_MESSAGE_FREE:
			pr_debug(LOG_MODULE_MAIN, "Free message: %p",
				 msg->data);
			ipc_request_sync_int(IPC_MSG_TYPE_FREE,
					     0, 0, msg->data);
			break;
	}
	bfree(msg);
}

/**
 * \brief send a message to handle_ipc_request_port()
 *
 * \param msgid the message id to generate. can be \ref IPC_MESSAGE_FREE
 *              or \ref IPC_MESSAGE_SEND
 * \param message the message data to send / free
 */
static int ipc_request_send(uint16_t msgid, void * message)
{
	OS_ERR_TYPE err = E_OS_OK;
	struct ipc_async_msg * msg =
		(struct ipc_async_msg *) balloc(sizeof(*msg), &err);
	if (err == E_OS_OK) {
		MESSAGE_ID(&msg->h) = msgid;
		MESSAGE_DST(&msg->h) = ipc_port;
		MESSAGE_SRC(&msg->h) = ipc_port;
		msg->data = message;
		port_send_message(&msg->h);
	}
	return err;
}

int ipc_async_send_message(struct message *message)
{
	return ipc_request_send(IPC_MESSAGE_SEND, message);
}

void ipc_async_free_message(struct message *message)
{
	ipc_request_send(IPC_MESSAGE_FREE, message);
}

void ipc_async_init(T_QUEUE queue) {
	ipc_port = port_alloc(queue);
	port_set_handler(ipc_port, handle_ipc_request_port, NULL);
	pr_debug(LOG_MODULE_MAIN, "%s: done port: %d", __func__, ipc_port);
}
