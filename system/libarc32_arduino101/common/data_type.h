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

/**
 * @defgroup common_driver_defs Common Definitions
 * Common driver function return codes.
 * @ingroup common_drivers
 * @{
 */

/**
 * Common driver function return codes.
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
/** @} */
#endif
