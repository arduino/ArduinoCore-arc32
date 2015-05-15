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
#ifndef __IPC_H__
#define __IPC_H__

#include "infra/ipc_requests.h"

/**
 * \brief initialize ipc component.
 *
 * \param tx_channel the IPC tx mailbox.
 * \param rx_channel the IPC rx mailbox.
 * \param tx_ack_channel the tx acknowledge mailbox.
 * \param rx_ack_channel the rx acknowledge mailbox.
 * \param remote_cpu_id the remote cpu id of this IPC
 */
void ipc_init(int tx_channel, int rx_channel, int tx_ack_channel,
        int rx_ack_channel, int remote_cpu_id);

/**
 * \brief request a synchronous ipc call.
 *
 * This method blocks until the command is answered.
 *
 * \param request_id the synchronous request id
 * \param param1 the first param for the request
 * \param param2 the second param for the request
 * \param ptr the third param for the request
 *
 * \return the synchronous command response.
 */
int ipc_request_sync_int(int request_id, int param1, int param2, void*ptr);

/**
 * \brief polling mode polling call.
 *
 * In polling mode, this method has to be called in order to handle
 * ipc requests.
 */
void ipc_handle_message();

/**
 * \brief Called by platform specific code when an IPC synchronous request is
 * received.
 *
 * Inter-processors request callback called in the context of an interrupt
 *
 * \param cpu_id the cpu this request comes from
 * \param param1 a first int parameter
 * \param param2 a second int parameter
 * \param ptr    a last pointer parameter
 *
 * \return 0 if success, -1 otherwise
 */
int ipc_sync_callback(int cpu_id, int request, int param1, int param2,
		void *ptr);

#endif
