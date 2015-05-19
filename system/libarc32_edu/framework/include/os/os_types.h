/*  INTEL CONFIDENTIAL Copyright 2014-2015 Intel Corporation All Rights Reserved.
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
typedef void* T_SEMAPHORE ;
typedef void* T_MUTEX;
typedef void* T_QUEUE;
typedef void* T_QUEUE_MESSAGE;
typedef void* T_TIMER;
typedef void* T_TASK ;
typedef uint8_t T_TASK_PRIO ;
#define HIGHEST_TASK_PRIO   OS_SPECIFIC_HIGHEST_PRIO
#define LOWEST_TASK_PRIO    OS_SPECIFIC_LOWEST_PRIO

typedef enum {
    E_TASK_UNCREATED = 0,
    E_TASK_RUNNING,
    E_TASK_SUSPENDED,
} T_TASK_STATE;


/** Special values for "timeout" parameter */
#define OS_NO_WAIT                0
#define OS_WAIT_FOREVER           -1


#endif
