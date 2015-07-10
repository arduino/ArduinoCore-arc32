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

#ifndef __GPIO_SERVICE_H__
#define __GPIO_SERVICE_H__

#include <stdint.h>

#include "cfw/cfw.h"
#include "cfw/cfw_client.h"

#include "platform.h"

/**
 * @defgroup services Services
 * CFW Services.
 */

/**
 * @defgroup gpio_service GPIO Service
 * GPIO service API.
 * Supports:
 *  - \b SS_GPIO_SERVICE_ID - ARC specific GPIO
 *  - \b SOC_GPIO_SERVICE_ID - common GPIO for LMT and ARC
 *  - \b AON_GPIO_SERVICE_ID - AON GPIO
 *
 * @ingroup services
 * @{
 */

/** service internal message ID for @ref gpio_configure */
#define MSG_ID_GPIO_CONFIGURE_REQ   MSG_ID_GPIO_BASE
/** service internal message ID for @ref gpio_set_state */
#define MSG_ID_GPIO_SET_REQ         (MSG_ID_GPIO_BASE + 1)
/** service internal message ID for @ref gpio_get_state */
#define MSG_ID_GPIO_GET_REQ         (MSG_ID_GPIO_BASE + 2)
/** service internal message ID for @ref gpio_listen */
#define MSG_ID_GPIO_LISTEN_REQ      (MSG_ID_GPIO_BASE + 3)
/** service internal message ID for @ref gpio_unlisten */
#define MSG_ID_GPIO_UNLISTEN_REQ    (MSG_ID_GPIO_BASE + 4)

/** message ID of service response for @ref gpio_configure */
#define MSG_ID_GPIO_CONFIGURE_RSP   (MSG_ID_GPIO_CONFIGURE_REQ | 0x40)
/** message ID of service response for @ref gpio_set_state */
#define MSG_ID_GPIO_SET_RSP         (MSG_ID_GPIO_SET_REQ | 0x40)
/** message ID of service response for @ref gpio_get_state */
#define MSG_ID_GPIO_GET_RSP         (MSG_ID_GPIO_GET_REQ | 0x40)
/** message ID of service response for @ref gpio_listen */
#define MSG_ID_GPIO_LISTEN_RSP      (MSG_ID_GPIO_LISTEN_REQ | 0x40)
/** message ID of service response for @ref gpio_unlisten */
#define MSG_ID_GPIO_UNLISTEN_RSP    (MSG_ID_GPIO_UNLISTEN_REQ | 0x40)
/** message ID of service response for GPIO events */
#define MSG_ID_GPIO_EVT             (MSG_ID_GPIO_BASE | 0x80)

/** GPIO interrupt types */
typedef enum {
    RISING_EDGE,
    FALLING_EDGE,
    BOTH_EDGE     // Used on soc GPIO only
} gpio_service_isr_mode_t;

/** Debounce configuration */
typedef enum {
    DEB_OFF = 0,
    DEB_ON
} gpio_service_debounce_mode_t;

/** Request message structure for @ref gpio_configure */
typedef struct gpio_configure_req_msg {
    struct cfw_message header;
    gpio_service_isr_mode_t mode; /*!< requested mode for each gpio: 0 - gpio is an output; 1 - gpio is an input */
    uint8_t index;                /*!< index of the gpio to configure in the port */
} gpio_configure_req_msg_t;

/** Response message structure for @ref gpio_configure */
typedef struct gpio_configure_rsp_msg {
	struct cfw_rsp_message rsp_header;
} gpio_configure_rsp_msg_t;

/** Request message structure for @ref gpio_set_state */
typedef struct gpio_set_req_msg {
    struct cfw_message header;
    uint8_t index;  /*!< index of the gpio in the port */
    uint8_t state;  /*!< state of the gpio to set */
} gpio_set_req_msg_t;

/** Response message structure for @ref gpio_set_state */
typedef struct gpio_set_rsp_msg {
    struct cfw_rsp_message rsp_header;
} gpio_set_rsp_msg_t;

/** Request message structure for @ref gpio_listen */
typedef struct gpio_listen_req_msg {
    struct cfw_message header;
    uint8_t index;                         /*!< index of GPIO pin to monitor */
    gpio_service_isr_mode_t mode;          /*!< interrupt mode */
    gpio_service_debounce_mode_t debounce; /*!< debounce mode */
} gpio_listen_req_msg_t;

/** Response message structure @ref gpio_listen */
typedef struct gpio_listen_rsp_msg {
    struct cfw_rsp_message rsp_header;
    uint8_t index; /*!< index of GPIO pin to monitor */
} gpio_listen_rsp_msg_t;

/** Event message structure for @ref gpio_listen */
typedef struct gpio_listen_evt_msg {
    struct cfw_message header;
    uint8_t index;      /*!< index of GPIO which triggered the interrupt */
    uint32_t pin_state; /*!< gpio current value */
} gpio_listen_evt_msg_t;

/** Request message structure for @ref gpio_unlisten */
typedef struct gpio_unlisten_req_msg {
    struct cfw_message header;
    uint8_t index; /*!< index of GPIO to monitor */
} gpio_unlisten_req_msg_t;

/** Response message structure @ref gpio_unlisten */
typedef struct gpio_unlisten_rsp_msg {
    struct cfw_rsp_message rsp_header;
    uint8_t index; /*!< index of GPIO to stop monitoring */
} gpio_unlisten_rsp_msg_t;

/** Request message structure for @ref gpio_get_state */
typedef struct gpio_get_req_msg {
    struct cfw_message header;
} gpio_get_req_msg_t;

/** Response message structure @ref gpio_get_state */
typedef struct gpio_get_rsp_msg {
    struct cfw_rsp_message rsp_header;
    uint32_t state; /*!< state of the gpio port */
} gpio_get_rsp_msg_t;

/**
 * Intialize/register the GPIO service.
 *
 * @param queue the queue this service will use for processing its messages
 * @param service_id the id this service is assigned
 */
void gpio_service_init(void * queue, int service_id);

/**
 * Configure a GPIO line.
 *
 * @param svc_handle the service connection handle
 * @param index the GPIO index in the port to configure
 * @param mode the mode of the GPIO line (0 - input, 1 - output)
 * @param priv the private data passed back in the
 *             response message.
 */
int gpio_configure(svc_client_handle_t * svc_handle, uint8_t index,
        uint8_t mode, void * priv);

/**
 * Set the state of a GPIO line.
 *
 * @param svc_handle the service connection handle
 * @param index the GPIO index in the port to configure
 * @param value the state to set the GPIO line (0 - low level, 1 - high level)
 * @param priv the private data passed back in the
 *             response message.
 */
int gpio_set_state(svc_client_handle_t * svc_handle, uint8_t index,
        uint8_t value, void * priv);

/**
 * Get state of a GPIO line
 *
 * @param svc_handle the service connection handle
 * @param priv the private data passed back in the
 *             response message.
 *
 * GPIO state will be available in the @ref gpio_get_rsp_msg
 */
int gpio_get_state(svc_client_handle_t * svc_handle, void * priv);

/**
 * Register to gpio state change.
 *
 * @param h the service connection handle
 * @param pin the GPIO index in the port to configure
 * @param mode interrupt mode (RISING_EDGE, FALLING_EDGE)
 * @param debounce debounce config (DEB_OFF, DEB_ON)
 * @param priv the private data passed back in the response message.
 *
 * @msc
 *  Client,"GPIO Service","GPIO driver";
 *
 *  Client->"GPIO Service" [label="register state change", URL="\ref gpio_listen_req_msg"];
 *  "GPIO Service"=>"GPIO driver" [label="call driver configuration"];
 *  "GPIO Service"<<"GPIO driver" [label="configuration status"];
 *  Client<-"GPIO Service" [label="status", URL="\ref gpio_listen_rsp_msg"];
 *  --- [label="GPIO state change"];
 *  Client<-"GPIO Service" [label="state change event", URL="\ref gpio_listen_evt_msg"];
 * @endmsc
 *
 * GPIO state and changed mask are available in the @ref gpio_listen_evt_msg message.
 */
int gpio_listen(svc_client_handle_t * h, uint8_t pin, gpio_service_isr_mode_t mode, uint8_t debounce, void *priv);

/**
 * Unregister to gpio state change.
 *
 * @param h the service connection handle
 * @param pin the GPIO index in the port to configure
 * @param priv the private data passed back in the response message.
 *
 *
 * GPIO state and changed mask are available in the @ref gpio_unlisten_rsp_msg message.
 */
int gpio_unlisten(svc_client_handle_t * h, uint8_t pin, void *priv);

/** @} */

#endif
