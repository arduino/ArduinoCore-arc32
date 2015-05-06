/** INTEL CONFIDENTIAL Copyright 2015 Intel Corporation All Rights Reserved.
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

#ifndef DATA_TYPE_H_
#define DATA_TYPE_H_

#include <stddef.h>
#include <stdint.h>

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (!FALSE)
#endif

typedef uint8_t boolean_t;
typedef volatile uint32_t * io_register_t;

/*!
 * Common driver function return codes
 */
typedef enum {
    DRV_RC_OK = 0,
    DRV_RC_FAIL,
    DRV_RC_TIMEOUT,
    DRV_RC_INVALID_CONFIG,              /*!< configuration parameters incorrect */
    DRV_RC_MODE_NOT_SUPPORTED,          /*!< controller/driver doesn't support this mode (master/slave) */
    DRV_RC_CONTROLLER_IN_USE,           /*!< controller is in use */
    DRV_RC_CONTROLLER_NOT_ACCESSIBLE,   /*!< controller not accessible from this core */
    DRV_RC_INVALID_OPERATION,           /*!< attempt to perform an operation that is invalid */
    DRV_RC_WRITE_PROTECTED,             /*!< Attempt to erase/program a memory region that is write protected */
    DRV_RC_READ_PROTECTED,              /*!< Attempt to read a memory region that is read protected */
    DRV_RC_CHECK_FAIL,                  /*!< Read back data after programming does not match the word written to memory */
    DRV_RC_OUT_OF_MEM,                  /*!< Attempt to program data outside the memory boundaries */
    DRV_RC_ERASE_PC,                    /*!< Attempt to write/erase executable code currently in use */
    DRV_RC_TOTAL_RC_CODE                /*!< Number of DRIVER_API_RC codes (used to extend this enum) */
} DRIVER_API_RC;

#endif
