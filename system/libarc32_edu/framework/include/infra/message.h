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

#ifndef __INFRA_MESSAGE_H_
#define __INFRA_MESSAGE_H_

#include "os/os_types.h"
#include "util/list.h"

/**
 * @defgroup messaging Messaging infrastructure
 * Messaging infrastructure
 *
 * @ingroup infra
 * @{
 * @defgroup message Message
 * Message structure defintion
 * @{
 */

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

/**
 * Allocate a message
 *
 * \param size the size of the message to allocate.
 * \param err the eventual return code pointer. If err is NULL, the function
 *            will panic in case of allocation failed.
 *
 * \return the allocated message or NULL if allocation failed and err != NULL
 */
struct message * message_alloc(int size, OS_ERR_TYPE * err);

/**
 * Free an allocated message
 *
 * \param message the message allocated with message_alloc()
 */
void message_free(struct message *message);

/** @} */
/** @} */
#endif /* __INFRA_MESSAGE_H_ */
