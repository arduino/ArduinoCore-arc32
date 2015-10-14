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

#ifndef __LOG_H
#define __LOG_H

#include <stdint.h>
#include <stdbool.h>
#include <infra/log_backend.h>
#include <stdarg.h>

/**
 * @defgroup infra_log Log
 * @ingroup infra
 * @{
 */

/** Log levels. */
enum {
	LOG_LEVEL_ERROR,    /*!< Error log level */
	LOG_LEVEL_WARNING,  /*!< Warning log level */
	LOG_LEVEL_INFO,     /*!< Info log level */
	LOG_LEVEL_DEBUG,    /*!< Debug log level, requires debug to be activated for
	                     * this module at compile time */
	LOG_LEVEL_NUM
};

/* Generate a list of module IDs */
#define DEFINE_LOGGER_MODULE(_id,_name,...) _id,
/**
 * The list of all available log modules on this project.
 */
enum {
#include "log_modules"
	LOG_MODULE_NUM /* gives the number of modules */
};
#undef DEFINE_LOGGER_MODULE


/** Message sent to log_printk larger than this size will be truncated on some
 * implementations. */
#define LOG_MAX_MSG_LEN (80)

/**
 * Initializes the log instance.
 *
 * This function must be called before writing any logs. In most case, a call to
 * log_set_backend() is also required to get log output. An exception to this
 * rule is the case of a log "slave" when multi-core log is used which doesn't
 * need a backend.
 */
void log_init();

/**
 * Set the log backend.
 *
 * This function can be called before log_init(), and also at any time later
 * for example to change the log backend at a given point in time.
 *
 * @param backend the log_backend to be used by this logger instance
 */
void log_set_backend(struct log_backend backend);

/**
 * Send a user's log message into the backend.
 *
 * On some buffered implementations the message is not immediately output to the
 * backend. Call the log_flush() function to make sure that all messages are
 * really output.
 * Message longer than LOG_MAX_MSG_LEN will be truncated on some implementations.
 *
 * @param level the log level for this message
 * @param module the ID of the log module
 * @param format the printf-like string format
 * @return message's length in in case of success, -1 if no available memory,
 * 0 if message was discarded
 */
int8_t log_printk(uint8_t level, uint8_t module, const char *format, ...);

/**
 * Same as log_printk() except that this function is called with a va_list
 * instead of a variable number of arguments.
 *
 * @param level the log level for this message
 * @param module the ID of the log module
 * @param format the printf-like string format
 * @param args
 * @return message's length in in case of success, -1 if no available memory,
 * 0 if message was discarded
 */
int8_t log_vprintk(uint8_t level, uint8_t module, const char *format, va_list args);

/**
 * Get the human-friendly name of a log module.
 *
 * @param module the ID of the log module
 * @return the log module name or NULL if not found
 */
const char* log_get_module_name(uint8_t module);

/**
 * Get the human-friendly name of a log level.
 *
 * @param level the level id
 * @return the log level name or NULL if not found
 */
const char* log_get_level_name(uint8_t level);

/**
 * Set the global log level value. This acts as a maximum overall level limit.
 *
 * @param level  The new log level value, from the log level enum
 * @return -1 if error,
 *          0 if new level was set
 */
int8_t log_set_global_level(uint8_t level);

/**
 * Disable or enable logging for a module.
 *
 * @param module the ID of the log module
 * @param b true to enable, false to disable
 * @return -1 if the module_id or action is incorrect,
 *          0 if module filter was disabled
 */
int8_t log_module_toggle(uint8_t module, bool b);

/**
 * Set the log level of a log module.
 *
 * @param module the ID of the log module
 * @param level the new log level for this module
 * @return -2 if the module_id is incorrect,
 *         -1 if the level is incorrect,
 *          0 in case of success
 */
int8_t log_module_set_level(uint8_t module, uint8_t level);

/**
 * Get the global log level applying to all modules.
 *
 * @return the global log level
 */
uint8_t log_get_global_level();

/**
 * Get whether a log module is activated.
 *
 * @param module the ID of the log module
 * @return true if this log module is activated, false otherwise
 */
bool log_get_module_status(uint8_t module);

/**
 * Get the log level of a log module.
 *
 * @param module the ID of the log module
 * @return the log level for this log module
 */
int8_t log_get_module_level(uint8_t module);

/**
 * On bufferized implementations, make sure that all pending messages are
 * flushed to the log_backend. This function has no effect on unbuffered
 * implementations.
 */
void log_flush();

/**
 * Suspend logger task.
 * This is used when we don't want logging to interrupt any lower priority
 * work. deep sleep process is an example.
 */
void log_suspend();

/**
 * Resume logger task.
 */
void log_resume();

/**
 * Log an error message.
 *
 * @param module the ID of the module related to this message
 * @param format the printf-like string format
 */
#define pr_error(module, format,...) log_printk(LOG_LEVEL_ERROR, module, format, ##__VA_ARGS__)

/**
 * Log a warning message.
 *
 * @param module the ID of the log module related to this message
 * @param format the printf-like string format
 */
#define pr_warning(module, format,...) log_printk(LOG_LEVEL_WARNING, module, format, ##__VA_ARGS__)

/**
 * Log an info message.
 *
 * @param module the ID of the log module related to this message
 * @param format the printf-like string format
 */
#define pr_info(module, format,...) log_printk(LOG_LEVEL_INFO, module, format, ##__VA_ARGS__)

/* The following function is defined for each log module. After preprocessing:
 * 1) The content of the "if" is constant, so compiler can optimize and remove
 * dead code (the log_printk path), leading to an empty function.
 * 2) Since the function is inline static, the call to the function (and then
 * the string argument) is removed.
 * This 2 passes optimization leads to a full removal of "pr_debug" code and
 * arguments if the third argument of "DEFINE_LOGGER_MODULE" is null or absent.
 */
#define DEFINE_LOGGER_MODULE(_id,_name,...) inline static int8_t pr_debug_ ## _id(const char *format,...) { \
	int8_t ret = 0; \
	if( (sizeof(#__VA_ARGS__) == sizeof("")) || 0x0##__VA_ARGS__) {\
		va_list args;\
		va_start(args, format);\
		ret = log_vprintk(LOG_LEVEL_DEBUG, _id, format, args);\
		va_end(args);\
	}\
	return ret;\
}
#include "log_modules"
#undef DEFINE_LOGGER_MODULE

/**
 * Log a debug message.
 *
 * Note that this call will have an effect only if debug log level is activated
 * for this module at compilation time. This is done by setting the 3rd
 * parameter of the DEFINE_LOGGER_MODULE X_MACRO to 1.
 *
 * @param module the ID of the log module related to this message
 * @param format the printf-like string format
 */
#define pr_debug(module, format,...) pr_debug_ ## module(format, ##__VA_ARGS__)

/** @} */

#endif /* __LOG_H */
