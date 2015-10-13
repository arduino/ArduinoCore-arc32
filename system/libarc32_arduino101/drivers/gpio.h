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

#ifndef GPIO_IFACE_H_
#define GPIO_IFACE_H_

#include "data_type.h"

typedef void (*gpio_callback_fn)(void);

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
} gpio_port_cfg_data_t;

#endif  /* GPIO_IFACE_H_ */
