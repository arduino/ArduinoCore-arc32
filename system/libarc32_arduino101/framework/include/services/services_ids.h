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

#ifndef __SERVICES_IDS__
#define __SERVICES_IDS__

/**
 * Services id definitions
 *
 * Keep them numbered manually to avoid shifting on
 * removal/addition of services
 */
enum {
	FRAMEWORK_SERVICE_ID  = 1,
	TEST2_SERVICE_ID      = 2,
	TEST_SERVICE_ID       = 3,
	BLE_SERVICE_ID        = 4,
	BLE_CORE_SERVICE_ID   = 5,
	SS_GPIO_SERVICE_ID    = 6,
	SOC_GPIO_SERVICE_ID   = 7,
	SS_ADC_SERVICE_ID     = 8,
	LL_STOR_SERVICE_ID    = 9,
	BATTERY_SERVICE_ID    = 10,
	UI_SVC_SERVICE_ID     = 11,
	PROPERTIES_SERVICE_ID = 12,
	ARC_SC_SVC_ID         = 13,
	LMT_SS_SVC_ID         = 14,
	AON_GPIO_SERVICE_ID   = 15,
	CDC_SERIAL_SERVICE_ID = 16,
	CFW_LAST_SERVICE_ID   = 17
};

#define BLE_SERVICE_MSG_BASE      (BLE_SERVICE_ID << 10)
#define BLE_SERVICE_GAP_MSG_BASE  (BLE_CORE_SERVICE_ID << 10)
#define MSG_ID_GPIO_BASE          (SOC_GPIO_SERVICE_ID << 10)
#define MSG_ID_ADC_SERVICE_BASE   (SS_ADC_SERVICE_ID << 10)
#define MSG_ID_LL_STORAGE_BASE    (LL_STOR_SERVICE_ID << 10)
#define MSG_ID_BATT_SERVICE_BASE  (BATTERY_SERVICE_ID << 10)
#define MSG_ID_UI_SERVICE_BASE    (UI_SVC_SERVICE_ID << 10)
#define MSG_ID_PROP_SERVICE_BASE  (PROPERTIES_SERVICE_ID << 10)
#define MSG_ID_SS_SERVICE_BASE    (ARC_SC_SVC_ID << 10)
#define MSG_ID_CDC_SERIAL_BASE    (CDC_SERIAL_SERVICE_ID << 10)

#endif
