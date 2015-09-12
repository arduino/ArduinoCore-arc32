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

#ifndef __BLE_SERVICE_H__
#define __BLE_SERVICE_H__

#include <stdint.h>

/**
 * @addtogroup ble_service
 * @{
 *
 */

/** BLE response/event status codes. */
enum BLE_STATUS {
	BLE_STATUS_SUCCESS = 0, /**< General BLE Success code */
	BLE_STATUS_PENDING, /**< Request received and execution started, response pending */
	BLE_STATUS_TIMEOUT, /**< Request timed out */
	BLE_STATUS_NOT_SUPPORTED, /**< Request/feature/parameter not supported */
	BLE_STATUS_NOT_ALLOWED, /**< Request not allowed */
	BLE_STATUS_LINK_TIMEOUT, /**< Link timeout (link loss) */
	BLE_STATUS_NOT_ENABLED, /**< BLE not enabled, @ref ble_enable */
	BLE_STATUS_ERROR,	/**< Generic Error */
	BLE_STATUS_ALREADY_REGISTERED, /**< BLE service already registered */
	BLE_STATUS_WRONG_STATE, /**< Wrong state for request */
	BLE_STATUS_ERROR_PARAMETER, /**< Parameter in request is wrong */
	BLE_STATUS_GAP_BASE = 0x100, /**< GAP specific error base */
	BLE_STATUS_GATT_BASE = 0x200, /**< GATT specific Error base */
};

typedef uint16_t ble_status_t; /**< Response and event BLE service status type @ref BLE_STATUS */
/** @}*/
#endif
