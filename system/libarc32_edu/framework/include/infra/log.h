/* INTEL CONFIDENTIAL Copyright 2015 Intel Corporation All Rights Reserved.
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
 * Intel's prior express written permission.
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

#ifndef __LOG_H
#define __LOG_H

#include <stdint.h>
#include <stdbool.h>
#include <infra/log_backend.h>
#include <stdarg.h>

/**
 * @defgroup infra_log Log
 * Log infrastructure.
 * @ingroup infra
 * @{
 */

/* log levels */
enum {
	LOG_LEVEL_ERROR,
	LOG_LEVEL_WARNING,
	LOG_LEVEL_INFO,
	LOG_LEVEL_DEBUG,
	LOG_LEVEL_NUM /* gives the number of log levels */
};

/* module ids */
#define DEFINE_LOGGER_MODULE(_i_,_n_,...) _i_,
enum {
#include "log_modules"
	LOG_MODULE_NUM /* gives the number of modules */
};
#undef DEFINE_LOGGER_MODULE


/* Message sent to log_printk larger than this size will
 * be truncated for some implementations */
#define LOG_MAX_MSG_LEN (80)

/**
 * Initializes the logger thread, queue and settings, and calls the
 * initialization functions.
 */
void log_init();

/**
 * Set the log backend. Can be called before log_init().
 * @param backend the backend to set
 */
void log_set_backend(struct log_backend backend);

/** @deprecated use log_set_backend() instead */
void log_register_backend(void (*print)(const char *buffer), void (*puts)(const char *buffer, uint16_t len));

/**
 * Creates and pushes a user's log message into the logging queue.
 *
 * Message longer than LOG_MAX_MSG_LEN will be truncated on some implementations.
 *
 * @return Message's length if inserted, -1 if no available memory, 0 if
 * message was discarded.
 */
int8_t log_printk(uint8_t level, uint8_t module, const char *format, ...);

/**
 * Same as log_printk() except that this function is called with a va_list
 * instead of a variable number of arguments.
 *
 * Message longer than LOG_MAX_MSG_LEN will be truncated on some implementations.
 *
 * @return Message's length if inserted, -1 if no available memory, 0 if
 * message was discarded.
 */
int8_t log_vprintk(uint8_t level, uint8_t module, const char *format, va_list args);

/**
 * Returns a log module's name, as string.
 *
 * @param module  The module id
 *
 * @return  Const pointer on the module name (NULL if module not found).
 */
const char *log_get_module_name(uint8_t module);

/**
 * Returns a log level's name, as string.
 *
 * @param level  The level id
 *
 * @return  Const pointer on the module name (NULL if module not found).
 */
const char *log_get_level_name(uint8_t level);

/**
 * Returns sending log's core, as string.
 *
 * @param core  The core id
 *
 * @return  Const pointer on the core name ("NULL" if core not found).
 */
const char * log_get_core_name(uint8_t logcore_id);

/**
  * Set the global log level value. This acts as a
  * maximum overall level limit.
  *
  * @param level  The new log level value, from the log level enum.
  *
  * @return -1 if error,
  *          0 if new level was set
  */
int8_t log_set_global_level(uint8_t level);

/**
 * Disable or enable the logging for a module.
 *
 * @param module_id  The module id to be disabled
 * @param action     Disable (0) or enable (1)
 *
 * @return -1 if the module_id or action is incorrect,
 *          0 if module filter was disabled
 */
int8_t log_module_toggle(uint8_t module_id, bool action);

/**
 * Set the level limit per module id.
 *
 * @param module_id  The module id to be changed
 * @param level      The new log level limit for this module
 * @return -2 if the module_id is incorrect,
 *         -1 if the level is incorrect,
 *          0 if module filter was changed
 */
int8_t log_module_set_level(uint8_t module_id, uint8_t level);

/**
 * Get the level limit.
 *
 * @return level value
 */
uint8_t log_get_global_level();

/**
 * Get the module status.
 *
 * @param module  The module id
 * @return module status
 */
bool log_get_module_status(uint8_t module);

/**
 * Get the module level per module id.
 *
 * @param module  The module id
 * @return level module
 */
int8_t log_get_module_level(uint8_t module);

/**
 * Flushes the log messages queue, called from the panic handler.
 */
void log_flush();

/* log function format */
#define pr_error(module, format,...) log_printk(LOG_LEVEL_ERROR, module, format,##__VA_ARGS__)

#define pr_warning(module, format,...) log_printk(LOG_LEVEL_WARNING, module, format,##__VA_ARGS__)

#define pr_info(module, format,...) log_printk(LOG_LEVEL_INFO, module, format,##__VA_ARGS__)

/* The following function is defined for each log module. After preprocessing:
 * 1) the content of the "if" is constant, so compiler can optimize and remove
 * dead code (the log_printk path), leading to an empty function.
 * 2) since the function is inline static, the call to the function (and then
 * the string argument) is removed.
 * This 2 passes optimization leads to a full removal of "pr_debug" code and
 * arguments if the third argument of "DEFINE_LOGGER_MODULE" is null or absent.
 * */
#define DEFINE_LOGGER_MODULE(_i_,_n_,...) inline static int8_t pr_debug_ ## _i_(const char *format,...) { \
	int8_t ret = 0; \
	if( (sizeof(#__VA_ARGS__) == sizeof("")) || 0x0##__VA_ARGS__) {\
		va_list args;\
		va_start(args, format);\
		ret = log_vprintk(LOG_LEVEL_DEBUG, _i_, format, args);\
		va_end(args);\
	}\
	return ret;\
}
#include "log_modules"
#undef DEFINE_LOGGER_MODULE

#define pr_debug(module, format,...) pr_debug_ ## module(format, ##__VA_ARGS__)

#ifdef CONFIG_LOG_MULTI_CPU_SUPPORT

/** The function that the master call when it is done with
 * processing the log buffer for a log core */
typedef int (*ipc_log_func)(int request_id, int param1, int param2, void*ptr);

/** A log core and it's attributes */
struct log_core {
	uint8_t cpu_id;
	const char* name;
	ipc_log_func send_buffer;
};

/* core ids */
#define DEFINE_LOGGER_CORE(_i_,_k_,_l_,_n_) _i_,
enum {
#include "log_cores"
	LOG_CORE_NUM /* gives the number of log cores */
};
#undef DEFINE_LOGGER_CORE

/** The list of all log cores used on this system */
extern const struct log_core log_cores[LOG_CORE_NUM];

#endif /* CONFIG_LOG_MULTI_CPU_SUPPORT */

/** @} */

#endif /* __LOG_H */
