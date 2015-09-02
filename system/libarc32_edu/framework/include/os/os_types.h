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

#ifndef __OS_TYPES_H__
#define __OS_TYPES_H__

#include "stddef.h"
#include "stdbool.h"
#include "stdint.h"


/**********************************************************
 ************** General definitions  **********************
 **********************************************************/
#define UNUSED(p) (void)(p)

/**
 * \enum  OS_ERR_TYPE
 * \brief Generic type for execution status */
typedef enum {
    E_OS_OK = 0,                  /** generic OK status */
    /* use negative values for errors */
    E_OS_ERR = -1,                /** generic error status */
    E_OS_ERR_TIMEOUT = -2,        /** timeout expired */
    E_OS_ERR_BUSY = -3,           /** resource is not available */
    E_OS_ERR_OVERFLOW = -4,       /** service would cause an overflow */
    E_OS_ERR_EMPTY = -5,          /** no data available (e.g.  queue is empty) */
    E_OS_ERR_NOT_ALLOWED = -6,    /** service is not allowed in current execution context */
    E_OS_ERR_NO_MEMORY = -7,      /** all allocated resources are already in use */
    E_OS_ERR_NOT_SUPPORTED = -8,  /** service is not supported on current context or OS */
    /* more error codes to be defined */
    E_OS_ERR_UNKNOWN = - 100,     /** invalid error code (bug?) */
} OS_ERR_TYPE;



/** Types for kernel objects */
typedef void* T_MUTEX;
typedef void* T_QUEUE;
typedef void* T_QUEUE_MESSAGE;

/** Special values for "timeout" parameter */
#define OS_NO_WAIT                0
#define OS_WAIT_FOREVER           -1


#endif
