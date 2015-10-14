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
#define BLE_MAX_DEVICE_NAME  20 /**< Max BLE device name length, spec size: 248 */
#define BLE_MAX_ADV_SIZE     31

/** Advertising Data Type. */
#define BLE_ADV_TYPE_FLAGS                         0x01	/* Flags */
#define BLE_ADV_TYPE_INC_16_UUID                   0x02	/* Incomplete List of 16-bit Service Class UUIDs */
#define BLE_ADV_TYPE_COMP_16_UUID                  0x03	/* Complete List of 16-bit Service Class UUIDs */
#define BLE_ADV_TYPE_INC_32_UUID                   0x04	/* Incomplete List of 32-bit Service Class UUIDs */
#define BLE_ADV_TYPE_COMP_32_UUID                  0x05	/* Complete List of 32-bit Service Class UUIDs */
#define BLE_ADV_TYPE_INC_128_UUID                  0x06	/* Incomplete List of 128-bit Service Class UUIDs */
#define BLE_ADV_TYPE_COMP_128_UUID                 0x07	/* Complete List of 128-bit Service Class UUIDs */
#define BLE_ADV_TYPE_SHORT_LOCAL_NAME              0x08	/* Shortened Local Name */
#define BLE_ADV_TYPE_COMP_LOCAL_NAME               0x09	/* Complete Local Name */
#define BLE_ADV_TYPE_TX_POWER                      0x0A	/* Tx Power Level */
#define BLE_ADV_TYPE_CLASS                         0x0D	/* Class of Device */
#define BLE_ADV_TYPE_PAIRING_C                     0x0E	/* Simple Pairing Hash C */
#define BLE_ADV_TYPE_PAIRING_C_192                 0x0E	/* Simple Pairing Hash C-192 */
#define BLE_ADV_TYPE_PAIRING_R_192                 0x0F	/* Simple Pairing Randomizer R-192 */
#define BLE_ADV_TYPE_DEVICE_ID                     0x10	/* Device ID */
#define BLE_ADV_TYPE_TK_VALUE                      0x10	/* Security Manager TK Value */
#define BLE_ADV_TYPE_OOB_FLAGS                     0x11	/* Security Manager Out of Band Flags */
#define BLE_ADV_TYPE_CONN_INTERVAL_RANGE           0x12	/* Slave Connection Interval Range */
#define BLE_ADV_TYPE_SERVICE_SOLICITATION_16_UUID  0x14	/* List of 16-bit Service Solicitation UUIDs */
#define BLE_ADV_TYPE_SERVICE_SOLICITATION_32_UUID  0x1F	/* List of 32-bit Service Solicitation UUIDs */
#define BLE_ADV_TYPE_SERVICE_SOLICITATION_128_UUID 0x15	/* List of 128-bit Service Solicitation UUIDs */
#define BLE_ADV_TYPE_SERVICE_DATA                  0x16	/* Service Data */
#define BLE_ADV_TYPE_SERVICE_DATA_16_UUID          0x16	/* Service Data - 16-bit UUID */
#define BLE_ADV_TYPE_SERVICE_DATA_32_UUID          0x20	/* Service Data - 32-bit UUID */
#define BLE_ADV_TYPE_SERVICE_DATA_128_UUID         0x21	/* Service Data - 128-bit UUID */
#define BLE_ADV_TYPE_SEC_CONF_VALUE                0x22	/* LE Secure Connections Confirmation Value */
#define BLE_ADV_TYPE_SEC_RANDOM_VALUE              0x23 /* LE Secure Connections Random Value */
#define BLE_ADV_TYPE_PUB_TARGET_ADDR               0x17	/* Public Target Address */
#define BLE_ADV_TYPE_APPEARANCE                    0x19	/* Appearance */
#define BLE_ADV_TYPE_ADV_INTERVAL                  0x1A	/* Advertising Interval */
#define BLE_ADV_TYPE_DEVICE_ADDR                   0x1B	/* LE Bluetooth Device Address */
#define BLE_ADV_TYPE_ROLE                          0x1C	/* LE Role */
#define BLE_ADV_TYPE_PAIRING_C_256                 0x1D	/* Simple Pairing Hash C-256 */
#define BLE_ADV_TYPE_PAIRING_R_256                 0x1E	/* Simple Pairing Randomizer R-256 */
#define BLE_ADV_TYPE_3D                            0x3D	/* 3D Information Data */
#define BLE_ADV_TYPE_MANUFACTURER                  0xFF	/* Manufacturer Specific Data */

/** BLE Service UUID Definitions. */
#define BLE_SVC_UUID_IMMEDIATE_ALERT_SERVICE            0x1802	   /**< Immediate Alert service UUID. */
#define BLE_SVC_UUID_LINK_LOSS_SERVICE                  0x1803	   /**< Link Loss service UUID. */
#define BLE_SVC_UUID_TX_POWER_SERVICE                   0x1804	   /**< TX Power service UUID. */
#define BLE_SVC_UUID_CURRENT_TIME_SERVICE               0x1805	   /**< Current Time service UUID. */
#define BLE_SVC_UUID_REFERENCE_TIME_UPDATE_SERVICE      0x1806	   /**< Reference Time Update service UUID. */
#define BLE_SVC_UUID_NEXT_DST_CHANGE_SERVICE            0x1807	   /**< Next Dst Change service UUID. */
#define BLE_SVC_UUID_GLUCOSE_SERVICE                    0x1808	   /**< Glucose service UUID. */
#define BLE_SVC_UUID_HEALTH_THERMOMETER_SERVICE         0x1809	   /**< Health Thermometer service UUID. */
#define BLE_SVC_UUID_DEVICE_INFORMATION_SERVICE         0x180A	   /**< Device Information service UUID. */
#define BLE_SVC_UUID_HEART_RATE_SERVICE                 0x180D	   /**< Heart Rate service UUID. */
#define BLE_SVC_UUID_PHONE_ALERT_STATUS_SERVICE         0x180E	   /**< Phone Alert Status service UUID. */
#define BLE_SVC_UUID_BATTERY_SERVICE                    0x180F	   /**< Battery service UUID. */
#define BLE_SVC_UUID_BLOOD_PRESSURE_SERVICE             0x1810	   /**< Blood Pressure service UUID. */
#define BLE_SVC_UUID_ALERT_NOTIFICATION_SERVICE         0x1811	   /**< Alert Notification service UUID. */
#define BLE_SVC_UUID_HUMAN_INTERFACE_DEVICE_SERVICE     0x1812	   /**< Human Interface Device service UUID. */
#define BLE_SVC_UUID_SCAN_PARAMETERS_SERVICE            0x1813	   /**< Scan Parameters service UUID. */
#define BLE_SVC_UUID_RUNNING_SPEED_AND_CADENCE          0x1814	   /**< Running Speed and Cadence service UUID. */
#define BLE_SVC_UUID_CYCLING_SPEED_AND_CADENCE          0x1816	   /**< Cycling Speed and Cadence service UUID. */

#define BLE_SVC_GAP_ADV_FLAG_LE_LIMITED_DISC_MODE	(0x01)   /**< LE Limited Discoverable Mode. */
#define BLE_SVC_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE	(0x02)   /**< LE General Discoverable Mode. */
#define BLE_SVC_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED	(0x04)   /**< BR/EDR not supported. */
#define BLE_SVC_GAP_ADV_FLAG_LE_BR_EDR_CONTROLLER	(0x08)   /**< Simultaneous LE and BR/EDR, Controller. */
#define BLE_SVC_GAP_ADV_FLAG_LE_BR_EDR_HOST		(0x10)   /**< Simultaneous LE and BR/EDR, Host. */
#define BLE_SVC_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE   (BLE_SVC_GAP_ADV_FLAG_LE_LIMITED_DISC_MODE | BLE_SVC_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED)   /**< LE Limited Discoverable Mode, BR/EDR not supported. */
#define BLE_SVC_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE   (BLE_SVC_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE | BLE_SVC_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED)   /**< LE General Discoverable Mode, BR/EDR not supported. */


/**
 * Characteristics UUID definitions.
 */

/* GAP */
#define BLE_GAP_DEVICE_NAME                     0x2A00
#define BLE_GAP_APPEARANCE                      0x2A01
#define BLE_GAP_PER_PRIVACY_FLAG                0x2A02
#define BLE_GAP_RECONN_ADDR                     0x2A03
#define BLE_GAP_PREF_CONN_PARAM                 0x2A04

/* DIS */
#define BLE_DIS_MANUFACTURER_NAME               0x2A29
#define BLE_DIS_MODEL_NB                        0x2A24
#define BLE_DIS_SERIAL_NB                       0x2A25
#define BLE_DIS_FW_REV                          0x2A26
#define BLE_DIS_HW_REV                          0x2A27
#define BLE_DIS_SW_REV                          0x2A28
#define BLE_DIS_SYS_ID                          0x2A23
#define BLE_DIS_CERTIF_DATA_LIST                0x2A2A
#define BLE_DIS_PNP_ID                          0x2A50

/* BATTERY */
#define BLE_BAT_BAT_LEVEL                       0x2A19

/* HR */
#define BLE_HEART_RATE_MEASUREMENT              0x2A37
#define BLE_HEART_RATE_SENSOR_LOCATION          0x2A38
#define BLE_HEART_RATE_CONTROL_POINT            0x2A39

/* RSC */
#define BLE_RSC_MEASUREMENT                     0x2A53
#define BLE_RSC_SENSOR_LOCATION                 0x2A5D
#define BLE_RSC_SUPPORTED_FEATURE               0x2A54
#define BLE_RSC_CONTROL_POINT                   0x2A55

/*CSC */
#define BLE_CSC_MEASUREMENT                     0x2A5B
#define BLE_CSC_SENSOR_LOCATION                 BLE_RSC_SENSOR_LOCATION
#define BLE_CSC_SUPPORTED_FEATURE               0x2A5C
#define BLE_CSC_CONTROL_POINT                   0x2A55

/* CP */
#define BLE_CP_MEASUREMENT                      0x2A63
#define BLE_CP_SENSOR_LOCATION                  BLE_RSC_SENSOR_LOCATION
#define BLE_CP_SUPPORTED_FEATURE                0x2A65
#define BLE_CP_POWER_VECTOR                     0x2A64
#define BLE_CP_CONTROL_POINT                    0x2A66

/* HCI status (error) codes as per BT spec */
#define HCI_REMOTE_USER_TERMINATED_CONNECTION           0x13
#define HCI_REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES 0x14
#define HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF     0x15
#define HCI_LOCAL_HOST_TERMINATED_CONNECTION            0x16

/** BLE GAP Appearance Characteristic definitions.
 *
 * See http://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
 */
#define BLE_GAP_APPEARANCE_TYPE_UNKNOWN                                0
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_PHONE                         64
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_COMPUTER                     128
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_WATCH                        192
#define BLE_GAP_APPEARANCE_TYPE_WATCH_SPORTS_WATCH                   193
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_CLOCK                        256
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_DISPLAY                      320
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_REMOTE_CONTROL               384
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_EYE_GLASSES                  448
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_TAG                          512
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_KEYRING                      576
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_MEDIA_PLAYER                 640
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_BARCODE_SCANNER              704
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_THERMOMETER                  768
#define BLE_GAP_APPEARANCE_TYPE_THERMOMETER_EAR                      769
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_HEART_RATE_SENSOR            832
#define BLE_GAP_APPEARANCE_TYPE_HEART_RATE_SENSOR_HEART_RATE_BELT    833
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_BLOOD_PRESSURE               896
#define BLE_GAP_APPEARANCE_TYPE_BLOOD_PRESSURE_ARM                   897
#define BLE_GAP_APPEARANCE_TYPE_BLOOD_PRESSURE_WRIST                 898
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_HID                          960
#define BLE_GAP_APPEARANCE_TYPE_HID_KEYBOARD                         961
#define BLE_GAP_APPEARANCE_TYPE_HID_MOUSE                            962
#define BLE_GAP_APPEARANCE_TYPE_HID_JOYSTICK                         963
#define BLE_GAP_APPEARANCE_TYPE_HID_GAMEPAD                          964
#define BLE_GAP_APPEARANCE_TYPE_HID_DIGITIZERSUBTYPE                 965
#define BLE_GAP_APPEARANCE_TYPE_HID_CARD_READER                      966
#define BLE_GAP_APPEARANCE_TYPE_HID_DIGITAL_PEN                      967
#define BLE_GAP_APPEARANCE_TYPE_HID_BARCODE                          968
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_GLUCOSE_METER               1024
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_RUNNING_WALKING_SENSOR      1088
#define BLE_GAP_APPEARANCE_TYPE_RUNNING_WALKING_SENSOR_IN_SHOE      1089
#define BLE_GAP_APPEARANCE_TYPE_RUNNING_WALKING_SENSOR_ON_SHOE      1090
#define BLE_GAP_APPEARANCE_TYPE_RUNNING_WALKING_SENSOR_ON_HIP       1091
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_CYCLING                     1152
#define BLE_GAP_APPEARANCE_TYPE_CYCLING_CYCLING_COMPUTER            1153
#define BLE_GAP_APPEARANCE_TYPE_CYCLING_SPEED_SENSOR                1154
#define BLE_GAP_APPEARANCE_TYPE_CYCLING_CADENCE_SENSOR              1155
#define BLE_GAP_APPEARANCE_TYPE_CYCLING_POWER_SENSOR                1156
#define BLE_GAP_APPEARANCE_TYPE_CYCLING_SPEED_CADENCE_SENSOR        1157
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_PULSE_OXIMETER              3136
#define BLE_GAP_APPEARANCE_TYPE_PULSE_OXIMETER_FINGERTIP            3137
#define BLE_GAP_APPEARANCE_TYPE_PULSE_OXIMETER_WRIST_WORN           3138
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_WEIGHT_SCALE                3200
#define BLE_GAP_APPEARANCE_TYPE_GENERIC_OUTDOOR_SPORTS_ACT          5184
#define BLE_GAP_APPEARANCE_TYPE_OUTDOOR_SPORTS_ACT_LOC_DISP         5185
#define BLE_GAP_APPEARANCE_TYPE_OUTDOOR_SPORTS_ACT_LOC_AND_NAV_DISP 5186
#define BLE_GAP_APPEARANCE_TYPE_OUTDOOR_SPORTS_ACT_LOC_POD          5187
#define BLE_GAP_APPEARANCE_TYPE_OUTDOOR_SPORTS_ACT_LOC_AND_NAV_POD  5188

/**
 * DTM commands, opcodes, indexes.
 */
#define	 DTM_HCI_CMD          0x01
#define	 DTM_HCI_OPCODE2      0x20

#define DTM_HCI_STATUS_IDX    6
#define DTM_HCI_LE_END_IDX    (DTM_HCI_STATUS_IDX + 1)

/** @} */

#endif
