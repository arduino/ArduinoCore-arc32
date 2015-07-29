/* INTEL CONFIDENTIAL Copyright 2014-2015 Intel Corporation All Rights Reserved.
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

#ifndef __PLATFORM_H_
#define __PLATFORM_H_
#include <stdint.h>

#define CPU_ID_LMT  0
#define CPU_ID_ARC  1
#define CPU_ID_BLE  2
#define CPU_ID_HOST 3
#define NUM_CPU     4

/**
 * LMT / ARC global shared structure. This structure lies in the beginning of
 * the RAM.
 */
struct platform_shared_block_ {
    /** Arc reset vector */
    unsigned int arc_start;
    /** Port table address */
    void * ports;
    /** Service table address */
    void * services;
    /** Port id of the service manager */
    uint16_t service_mgr_port_id;
    /** ARC boot synchronization flag.
     * This value is set to 0 prior to start ARC, and is polled until set to 1
     * by ARC in order to allow LMT to wait for ARC to be started. Usefull for
     * debugging ARC startup code.
     */
    uint8_t arc_ready;

    /** used to send suspend resume arc core
     * bit usage
     * [0-7] 	PM_POWERSTATE
     * [8-9] 	ACK
     * [16-31]	Magic number
     */
    uint32_t pm_request;
};

#ifdef CONFIG_BOARD_ATLASPEAK_EMU
#define RAM_START           0xffb00000
#else
#define RAM_START           0xA8000000
#endif

#define shared_data ((volatile struct platform_shared_block_ *) RAM_START)

/* Use a ROM address as a temporary factory_data pointer */
#define FACTORY_DATA_ADDR 0xffffe000

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
	CDC_SERIAL_SERVICE_ID = 16
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

#define ADC_VOLTAGE_CHANNEL                 4   /**< TODO arbitrary value. Looking for final decision*/
#define ADC_TEMPERATURE_CHANNEL             5   /**< TODO arbitrary value. Looking for final decision*/

unsigned int get_timestamp(void);

/* GPIO */
// soc gpio 32 bit count
#if defined(CONFIG_SOC_GPIO_32)
#define SOC_GPIO_32_BITS    (32)
#endif

// soc gpio aon bit count
#if defined(CONFIG_SOC_GPIO_AON)
#define SOC_GPIO_AON_BITS    (6)
#endif

#define SS_GPIO_8B0_BITS    (8)
#define SS_GPIO_8B1_BITS    (8)

/* I2C */
/*!
* List of all controllers in system ( IA and SS )
*/

typedef enum {
    SOC_I2C_0 = 0,     /*!< General Purpose I2C controller 0, accessible by both processing entities */
    SOC_I2C_1,         /*!< General Purpose I2C controller 1, accessible by both processing entities */
} SOC_I2C_CONTROLLER_PF;

/* SPI */

/*!
 * List of all controllers in host processor
 */
typedef enum {
    SOC_SPI_MASTER_0 = 0,     /* SPI master controller 0, accessible by both processing entities */
    SOC_SPI_MASTER_1,         /* SPI master controller 1, accessible by both processing entities */
    SOC_SPI_SLAVE_0           /* SPI slave controller */
}SOC_SPI_CONTROLLER_PF;

/* SOC COMPARATOR */
/*!
 * Number of analog comparator in Lakemont
 */
#if defined(CONFIG_SOC_COMPARATOR)
#define CMP_COUNT	    19
#endif

typedef enum {
	ROOT_DEVICE_ID    = 0,
	COMPARATOR_ID     = 1,
	RTC_ID            = 2,
	UART0_PM_ID       = 3,
	UART1_PM_ID       = 4,
	SOC_GPIO_32_ID    = 5,
	SOC_FLASH_ID      = 6,
	SOC_GPIO_AON_ID   = 7,
	SPA_SPI0_ID       = 8,
	SBA_I2C1_ID       = 9,
	SPI_FLASH_0_ID    = 10,
	SBA_SOC_SPI_0_ID  = 11,
	SBA_SOC_I2C1      = 12,
	SBA_SS_SPI_0_ID   = 13,
	SBA_SS_SPI_1_ID   = 14,
	SBA_SS_I2C_0_ID   = 15,
	SPI_BMI160_ID     = 16,
	I2C_BMI160_ID     = 17,
	USB_PM_ID         = 18,
	SOC_LED_ID        = 19,
	HD44780_ID        = 20,
	SS_ADC_ID         = 21,
	SS_GPIO_8B0_ID    = 22,
	SS_GPIO_8B1_ID    = 23,
	WDT_ID            = 24,
} DEVICE_ID;

#endif
