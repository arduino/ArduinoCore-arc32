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

#ifndef __PLATFORM_H_
#define __PLATFORM_H_
#include <stdint.h>
#include "services/services_ids.h"

#define CPU_ID_LMT  0
#define CPU_ID_ARC  1
#define CPU_ID_BLE  2
#define CPU_ID_HOST 3
#define NUM_CPU     4

#define SERIAL_BUFFER_SIZE 256

struct cdc_ring_buffer
{
    /** Ring buffer data */
    uint8_t data[SERIAL_BUFFER_SIZE];
    /** Ring buffer head pointer, modified by producer */
    int head;
    /** Ring buffer head pointer, modified by consumer */
    int tail;
};

struct cdc_acm_shared_data {
    /** Ring buffer to pass CDC-ACM data from LMT to ARC */
    struct cdc_ring_buffer *rx_buffer;
    /** Ring buffer to pass CDC-ACM data from ARC to LMT */
    struct cdc_ring_buffer *tx_buffer;
    /** Boolean flag set by LMT when CDC-ACM host connection is opened */
    int host_open;
    /** Boolean flag set by ARC when CDC-ACM endpoint connection is opened */
    int device_open;
};

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

    /** ARC wakelocks status info variables
     * Used in order to share if any wakelock
     * is taken, on ARC side.
     */
    uint8_t any_arc_wakelock_taken;

    /** LMT wakelocks status info variables
     * Used in order to share if any wakelock
     * is taken, on LMT side.
     */
    uint8_t any_lmt_wakelock_taken;

    /* Pointer to shared structure used by CDC-ACM.
     *
     * The QRK core is responsible for allocating memory and initialising the
     * pointers of this structure.
     * The ARC core counts on QRK to find valid pointers in place.
     */
    struct cdc_acm_shared_data	* cdc_acm_buffers;
};

#define RAM_START           0xA8000000

#define shared_data ((volatile struct platform_shared_block_ *) RAM_START)

/* Use a ROM address as a temporary factory_data pointer */
#define FACTORY_DATA_ADDR 0xffffe000

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
