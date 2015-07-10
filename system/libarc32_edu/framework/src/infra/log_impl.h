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


#ifndef LOG_IMPL_H
#define LOG_IMPL_H

#include <stdint.h>

/**
 * Creates and pushes a user's log message into the logging queue.
 *
 * @retval Message's length if inserted, -1 if an error occurs, 0 if
 * message was discarded.
 */
uint32_t log_write_msg(uint8_t level, uint8_t module, const char *format,
				va_list args);

/**
 * Logger init for specific implementations.
 */
void log_impl_init();

#endif /* LOG_IMPL_H */
