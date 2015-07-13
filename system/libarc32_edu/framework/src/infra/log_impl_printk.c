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
#include "log_impl.h"
#include "infra/log_backend.h"
#include "os/os.h"

extern int printk(const char * format, ...);

void log_cb_ipc(int cpu_id, void *buffer)
{
	return;
}

uint32_t log_write_msg(uint8_t level, uint8_t module, const char *format,
				va_list args)
{
// TODO - implement printk
//	printk(format, args);
	return 0;
}

void log_flush() {
	return;
}

void log_impl_init() {
	return;
}

void log_set_backend(struct log_backend backend) {
	return;
}
