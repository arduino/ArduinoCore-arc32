/** INTEL CONFIDENTIAL Copyright 2014-2015 Intel Corporation All Rights Reserved.
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

#ifndef GPIO_IFACE_H_
#define GPIO_IFACE_H_

#include "data_type.h"

typedef void (*gpio_callback_fn)( uint32_t , void*);

/*!
 * GPIO types
 */
typedef enum {
    GPIO_INPUT,      /*!< Configure GPIO pin as input */
    GPIO_OUTPUT,     /*!< Configure GPIO pin as output */
    GPIO_INTERRUPT   /*!< Configure GPIO pin as interrupt */
} GPIO_TYPE;

/*!
 * Interrupt types
 */
typedef enum {
    LEVEL,           /*!< Configure an interrupt triggered on level */
    EDGE,            /*!< Configure an interrupt triggered on single edge */
    DOUBLE_EDGE      /*!< Configure an interrupt triggered on both rising and falling edges */
} INT_TYPE;

/*!
 * Polarity configuration for interrupts
 */
typedef enum {
    ACTIVE_LOW,      /*!< Configure an interrupt on low level or falling edge */
    ACTIVE_HIGH      /*!< Configure an interrupt on high level or rising edge */
} INT_POLARITY;

/*!
 * Debounce configuration for interrupts
 */
typedef enum {
    DEBOUNCE_OFF,    /*!< Disable debounce for interrupt */
    DEBOUNCE_ON      /*!< Enable debounce for interrupt */
} INT_DEBOUNCE;

/*!
 * ls_sync configuration for interrupts
 */
typedef enum {
    LS_SYNC_OFF,     /*!< Disable ls sync for interrupt */
    LS_SYNC_ON       /*!< Enable ls sync for interrupt */
} INT_LS_SYNC;

/*!
 * GPIO configuration structure
 */
typedef struct gpio_cfg_data {
    GPIO_TYPE         gpio_type;         /*!< GPIO type */
    INT_TYPE          int_type;          /*!< GPIO interrupt type */
    INT_POLARITY      int_polarity;      /*!< GPIO polarity configuration */
    INT_DEBOUNCE      int_debounce;      /*!< GPIO debounce configuration */
    INT_LS_SYNC       int_ls_sync;       /*!< GPIO ls sync configuration */
    gpio_callback_fn  gpio_cb;           /*!< Callback function called when an interrupt is triggered on this pin */
    void             *gpio_cb_arg;       /*!< Data passed as an argument for the callback function */
} gpio_cfg_data_t;

/*!
 * GPIO port configuration structure
 */
typedef struct gpio_port_cfg_data {
    uint32_t           gpio_type;        /*!< Set port type (0=INPUT, 1=OUTPUT) */
    uint32_t           is_interrupt;     /*!< Enable interrupt on GPIO (0=OFF, 1=ON) */
    uint32_t           int_type;         /*!< Sets interrupt type (0=LEVEL, 1=EDGE) */
    uint32_t           int_bothedge;     /*!< Enable interrupt on both rising and falling edges (0=OFF, 1=ON) */
    uint32_t           int_polarity;     /*!< GPIO polarity configuration (0=LOW, 1=HIGH) */
    uint32_t           int_debounce;     /*!< GPIO debounce configuration (0=OFF, 1=ON) */
    uint32_t           int_ls_sync;      /*!< GPIO ls sync configuration (0=OFF, 1=ON) */
    gpio_callback_fn   gpio_cb[32];      /*!< Callback function called when an interrupt is triggered on this pin */
    void              *gpio_cb_arg[32];  /*!< Data passed as an argument for the callback function */
} gpio_port_cfg_data_t;

#endif  /* GPIO_IFACE_H_ */
