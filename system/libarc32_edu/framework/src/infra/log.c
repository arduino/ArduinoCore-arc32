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

#include <stdarg.h>
#include "os/os.h"
#include "infra/log.h"
#include "log_impl.h"

#define DEFINE_LOGGER_MODULE(_i_,_n_,...)[_i_]=_n_,
static const char *modules_string[LOG_MODULE_NUM] = {
#include "log_modules"
};
#undef DEFINE_LOGGER_MODULE

#ifdef CONFIG_LOG_MASTER
#include "platform.h"
#include "infra/ipc.h"
#define DEFINE_LOGGER_CORE(_i_,_k_,_l_,_n_) \
		[_i_] = { \
			.cpu_id = _n_, \
			.name = _k_, \
			.send_buffer = _l_ \
		},
const struct log_core log_cores[] = {
#include "log_cores"
};
#undef DEFINE_LOGGER_CORE

const char *log_get_core_name(uint8_t logcore_id)
{
	if (logcore_id >= LOG_CORE_NUM)
		panic(E_OS_ERR);
	return log_cores[logcore_id].name;
}
#else
const char *log_get_core_name(uint8_t logcore_id)
{
	return "";
}
#endif

typedef struct modules_infos {
	uint8_t log_level; /* Level limit */
	bool status; /* Enabled / Disabled */
} modules_infos_t;

typedef struct log_global_infos {
	/* modules_filter[module_id] = { 0: enabled/disabled, 1: level limit} */
	modules_infos_t modules_filter[LOG_MODULE_NUM];
	uint8_t log_level_limit;
} log_global_info_t;

static log_global_info_t global_infos;

static const char *levels_string[LOG_LEVEL_NUM] = {[LOG_LEVEL_ERROR] =
		"ERROR",[LOG_LEVEL_WARNING] = "WARN",[LOG_LEVEL_INFO] =
		"INFO",[LOG_LEVEL_DEBUG] = "DEBUG"
};

void log_init()
{
	uint8_t i;

	for (i = 0; i < LOG_MODULE_NUM; i++) {
		global_infos.modules_filter[i].status = 1;
		global_infos.modules_filter[i].log_level = LOG_LEVEL_INFO;
	}
	global_infos.log_level_limit = LOG_LEVEL_INFO;

	log_impl_init();
}

int8_t log_vprintk(uint8_t level, uint8_t module, const char *format,
		   va_list args)
{
	/* filter by global level limit */
	if (level > global_infos.log_level_limit)
		return 0;

	if (module >= LOG_MODULE_NUM)
		return -1;

	/* filter by module level limit and module enable */
	if (level > global_infos.modules_filter[module].log_level ||
			global_infos.modules_filter[module].status == false)
		return 0;

	return log_write_msg(level, module, format, args);
}

int8_t log_printk(uint8_t level, uint8_t module, const char *format, ...)
{
	va_list args;
	va_start(args, format);
	int8_t ret = log_vprintk(level, module, format, args);
	va_end(args);
	return ret;
}

const char *log_get_module_name(uint8_t module)
{
	if (module >= LOG_MODULE_NUM) {
		return "";
	}
	return modules_string[module];
}

const char *log_get_level_name(uint8_t level)
{
	if (level >= LOG_LEVEL_NUM)
		return "";
	return levels_string[level];
}

int8_t log_set_global_level(uint8_t new_level)
{
	if (new_level >= LOG_LEVEL_NUM) {
		return -1;
	}
	global_infos.log_level_limit = new_level;
	return 0;
}

int8_t log_module_toggle(uint8_t module_id, bool action)
{
	if ((module_id >= LOG_MODULE_NUM) || (action > 1)) {
		return -1;
	}
	global_infos.modules_filter[module_id].status = action;
	return 0;
}

int8_t log_module_set_level(uint8_t module_id, uint8_t new_level)
{
	if (module_id >= LOG_MODULE_NUM)
		return -1;

	if (new_level >= LOG_LEVEL_NUM)
		return -1;

	global_infos.modules_filter[module_id].log_level = new_level;
	return 0;
}

uint8_t log_get_global_level()
{
	return global_infos.log_level_limit;
}

bool log_get_module_status(uint8_t module)
{
	return global_infos.modules_filter[module].status;
}

int8_t log_get_module_level(uint8_t module)
{
	return global_infos.modules_filter[module].log_level;
}
