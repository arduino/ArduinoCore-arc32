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
#ifndef __INFRA_MESSAGE_H_
#define __INFRA_MESSAGE_H_

#include <stdint.h>
#include "util/list.h"

/* Message ranges */
#define INFRA_MSG_TCMD_BASE		0xFF00


/**
 * Message types definition.
 */
typedef enum {
	TYPE_REQ,
	TYPE_RSP,
	TYPE_EVT,
	TYPE_INT
} msg_type_t;

/**
 * message class definition.
 */
typedef enum {
	CLASS_NORMAL,
	CLASS_NO_WAKE,
	CLASS_NO_WAKE_REPLACE,
	CLASS_REPLACE
} msg_class_t;

/**
 * Message flags definitions.
 *
 * prio: priority of the message.
 * class: class of the message
 * type: type of message
 */
struct msg_flags {
	uint16_t f_prio: 8;
	uint16_t f_class: 3;
	uint16_t f_type: 2;
};

/**
 * Message header structure definition.
 * Data message follows the header structure.
 * Header must be packed in order to ensure that
 * it can be simply exchanged between embedded cores.
 */
struct message {
#ifndef NO_MSG_LIST
    /** Message queueing member */
    list_t l;
#endif
    /** Message identifier */
	uint16_t id;
	/** Message destination port */
	uint16_t dst_port_id;
	/** Message source port */
	uint16_t src_port_id;
	/** Message length */
	uint16_t len;
	/** The message flags */
	struct msg_flags flags;
};

#define MESSAGE_ID(msg)     (msg)->id
#define MESSAGE_SRC(msg)    (msg)->src_port_id
#define MESSAGE_DST(msg)    (msg)->dst_port_id
#define MESSAGE_LEN(msg)    (msg)->len
#define MESSAGE_TYPE(msg)   (msg)->flags.f_type
#define MESSAGE_PRIO(msg)   (msg)->flags.f_prio
#define MESSAGE_CLASS(msg)  (msg)->flags.f_class

struct message * message_alloc(int size, OS_ERR_TYPE * err);
void message_free(struct message *message);


#endif /* __INFRA_MESSAGE_H_ */
