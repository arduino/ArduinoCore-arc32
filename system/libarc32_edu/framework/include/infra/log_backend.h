/*
 * INTEL CONFIDENTIAL Copyright 2015 Intel Corporation All Rights Reserved.
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
 */

#ifndef __LOG_BACKEND_H
#define __LOG_BACKEND_H

#include <stdint.h>

/**
 * @ingroup infra_log
 * @{
 */

/**
 * A log backends provide the final functions used to output logs.
 */
struct log_backend {
	/** Backend print function */
	void (*print)(const char *buffer);
	/** Backend puts function */
	void (*puts)(const char *buffer, uint16_t len);
};

/** @} */

#endif /* __LOG_BACKEND_H */
