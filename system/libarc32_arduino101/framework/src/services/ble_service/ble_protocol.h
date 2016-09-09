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

#ifndef __BLE_PROTOCOL_H__
#define __BLE_PROTOCOL_H__

/**
 *  @defgroup ble_protocol BLE protocol definitions
 *
 * BT Spec definitions.
 * @ingroup ble_service
 * @{
 *
 * Bluetooth SIG defined macros and enum extracted from Bluetooth Spec 4.2
 */
#define BLE_MAX_DEVICE_NAME  20 /**< Max BLE device name length 20 + NULL, spec size: 248 */
#define BLE_MAX_ADV_SIZE     31

#define BLE_GATT_MTU_SIZE 23 /**< Default MTU size */

/** Manufacturer IDs */
#define INTEL_MANUFACTURER 0x0002

/* HCI status (error) codes as per BT spec */
#define HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF     0x15
#define HCI_LOCAL_HOST_TERMINATED_CONNECTION            0x16

/** @} */

#endif
