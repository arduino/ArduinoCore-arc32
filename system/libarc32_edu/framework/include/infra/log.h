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

/**
 * \addtogroup infra
 * @{
 * \defgroup infra_log Logger infrastructure
 * @{
 * \brief Logger infrastructure
 *
 */

#include <stdint.h>
#include <string.h>

#include "infra/log_backends.h"

/* Maximum delay for the log_task to wait for a new message notification */
#define LOG_MAX_DELAY 0xffffffff

/* log configuration */
#define LOG_MAX_MSG_LEN              (80) /* Max allowed len: uint8_t - 1 */

/* log levels */
enum {
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_NUM /* gives the number of log levels */
};

/* module ids */
#define DEFINE_LOGGER_MODULE(_i_,_n_) _i_,
enum {
#include "log_modules"
    LOG_MODULE_NUM /* gives the number of modules */
};
#undef DEFINE_LOGGER_MODULE

#define LOG_LEVEL_DEFAULT           (LOG_LEVEL_INFO)

/**
 * Initializes the logger thread, queue and settings, and calls the
 * initialization functions.
 */
void log_init(void);

/**
 * Flushes the log messages queue, called from the panic handler.
 */
void log_flush(void (*be)(const char *s));

/**
 * Creates and pushes a user's log message into the logging queue.
 *
 * @return Message's length if inserted, -1 if no available memory, 0 if
 * message was discarded.
 */
int8_t log_printk(uint8_t level, uint8_t  module, const char *format, ...);

/**
 * Returns a log module's name, as string.
 *
 * @param module  The module id
 *
 * @return  Const pointer on the module name (NULL if module not found).
 */
const char * log_get_module_name(uint8_t module);

/**
 * Returns a log level's name, as string.
 *
 * @param level  The level id
 *
 * @return  Const pointer on the module name (NULL if module not found).
 */
const char * log_get_level_name(uint8_t level);

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
int8_t log_module_toggle(uint8_t module_id, uint8_t action);

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

/* TODO - implement proper logging handler later */
#define log_printk(module, format,...)

/* log function format */
#define pr_error(module, format,...) log_printk(LOG_LEVEL_ERROR, module, format,##__VA_ARGS__)

#define pr_warning(module, format,...) log_printk(LOG_LEVEL_WARNING, module, format,##__VA_ARGS__)

#define pr_info(module, format,...) log_printk(LOG_LEVEL_INFO, module, format,##__VA_ARGS__)

#define pr_debug(module, format,...) log_printk(LOG_LEVEL_DEBUG, module, format,##__VA_ARGS__)


/* @}*/
#endif /* __LOG_H */
