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

/*
 * This private header provides:
 *  - the definition of the functions that all log implementations must
 *    implement, including the ones required for adding multi CPU log support.
 *  - A set of helper functions that can be factorized between log
 *    implementations.
 */

#ifndef LOG_IMPL_H
#define LOG_IMPL_H

#include <stdint.h>
#include "infra/log.h"

/**
 * Pushes a user's log message.
 *
 * This function must be defined by all implementations of log.
 *
 * @return message's length if inserted, -1 if an error occurs, 0 if
 * message was discarded.
 */
uint32_t log_write_msg(uint8_t level, uint8_t module, const char *format,
				va_list args);

/**
 * Implementation-specific logger init.
 *
 * This function must be defined by all implementations of log.
 */
void log_impl_init();

/**
 * Describes a message to be printed on a log_backend.
 */
typedef struct __attribute__ ((__packed__)) log_message {
	/** If not 0 indicates that there was a log saturation at the time this
	 * message was logged */
	uint8_t has_saturated;
	uint8_t buf_size;         /*!< number of valid characters in buf */
	uint8_t level;            /*!< log level for this message */
	uint8_t module;           /*!< log module for this message */
	uint32_t timestamp;       /*!< timestamp for this message */
	uint8_t cpu_id;           /*!< CPU ID from which this message comes from */
	/** The number of lost message during a previous log saturation */
	uint8_t lost_messages_count;
	/** The message text of size buf_size, NULL-terminated */
	char buf[LOG_MAX_MSG_LEN];
} log_message_t;

#if defined(CONFIG_LOG_MASTER) || !defined(CONFIG_LOG_MULTI_CPU_SUPPORT)
/**
 * Output one message on the backend.
 */
void output_one_message(const log_message_t* msg);
#endif

#ifdef CONFIG_LOG_MASTER

/* Core ids. */
#define DEFINE_LOGGER_CORE(_id,_k_,_l_,_n_) _id,
enum {
#include "log_cores"
	LOG_CORE_NUM /* gives the number of log cores */
};
#undef DEFINE_LOGGER_CORE

/**
 * Describes a log core, as seen from the log "master".
 */
struct log_core {
	/** The CPU ID as defined in the IPC mechanism */
	uint8_t cpu_id;
	/** The human-friendly name of the the log core */
	const char* name;
	/** The IPC function that the master must call to notify it is ready to
	 * receive a new log message. */
	int (*send_buffer)(int request_id, int param1, int param2, void*ptr);
};

/** The list of all log cores used on this system. */
extern const struct log_core log_cores[LOG_CORE_NUM];

/**
 * Helper function to convert a CPU ID as used in the IPC system into
 * a log core ID.
 *
 * @param cpu_id the ID of the CPU
 * @return the log core ID, i.e. the index of this log core in the log_cores
 * array
 */
uint8_t cpu_id_to_logcore_id(uint8_t cpu_id);

/**
 * The log IPC callback called on the master when the slave has a new log
 * message.
 *
 * This function must be defined by all implementations of log master.
 *
 * @param cpu_id ID of the cpu from which the IPC comes from
 * @param msg the incoming log_message. On shared memory implementations, this
 * pointer is the same as the one passed in the XXX IPC callback on the slave.
 */
void log_incoming_msg_from_slave(int cpu_id, const log_message_t* msg);

#endif

#ifdef CONFIG_LOG_SLAVE

/**
 * The log IPC callback called on the slave when the master is ready to receive
 * a new log message.
 *
 * This function must be defined by all implementations of log slave.
 *
 * @param cpu_id ID of the cpu from which the IPC comes from
 * @param msg pointer on the passed buffer in case of shared memory
 * implementations or NULL if the buffer is allocated by the IPC mechanism.
 */
void log_master_ready_for_new_msg(int cpu_id, log_message_t* msg);

#endif

#endif /* LOG_IMPL_H */
