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

/**
 * \defgroup services Services
 * \brief Definition of the structure and functions used by services implementation.
 *
 * It consists of:
 *  - Service APIs
 *
 * @{
 */

/**
 * \addtogroup services
 * @{
 * \defgroup gpio_service GPIO Service API
 * @{
 * \brief Definition of the structure and functions used by GPIO services implementation.
 */

#ifndef __GPIO_SERVICE_H__
#define __GPIO_SERVICE_H__
#include "cfw/cfw.h"
#include "cfw/cfw_client.h"
#include "stdint.h"
#include "platform.h"

#define MSG_ID_GPIO_CONFIGURE_REQ   MSG_ID_GPIO_BASE
#define MSG_ID_GPIO_SET_REQ         (MSG_ID_GPIO_BASE + 1)
#define MSG_ID_GPIO_GET_REQ         (MSG_ID_GPIO_BASE + 2)
#define MSG_ID_GPIO_LISTEN_REQ      (MSG_ID_GPIO_BASE + 3)
#define MSG_ID_GPIO_UNLISTEN_REQ    (MSG_ID_GPIO_BASE + 4)

#define MSG_ID_GPIO_CONFIGURE_RSP   (MSG_ID_GPIO_CONFIGURE_REQ | 0x40)
#define MSG_ID_GPIO_SET_RSP         (MSG_ID_GPIO_SET_REQ | 0x40)
#define MSG_ID_GPIO_GET_RSP         (MSG_ID_GPIO_GET_REQ | 0x40)
#define MSG_ID_GPIO_LISTEN_RSP      (MSG_ID_GPIO_LISTEN_REQ | 0x40)
#define MSG_ID_GPIO_UNLISTEN_RSP    (MSG_ID_GPIO_UNLISTEN_REQ | 0x40)

#define MSG_ID_GPIO_EVT             (MSG_ID_GPIO_BASE | 0x80)

typedef enum {
    RISING_EDGE,
    FALLING_EDGE,
    BOTH_EDGE     // Used on soc GPIO only
} gpio_service_isr_mode_t;

typedef enum {
    DEB_OFF = 0,
    DEB_ON
} gpio_service_debounce_mode_t;

void gpio_service_init(void * queue, int service_id);

typedef struct gpio_configure_req_msg {
    struct cfw_message header;
    /** requested mode for each gpio
     * 0 - gpio is an output
     * 1 - gpio is an input
     */
    gpio_service_isr_mode_t mode;
    /** index of the gpio to configure in the port */
    uint8_t index;
} gpio_configure_req_msg_t;

typedef struct gpio_configure_rsp_msg {
	struct cfw_rsp_message rsp_header;
} gpio_configure_rsp_msg_t;

typedef struct gpio_set_req_msg {
    struct cfw_message header;
    /** index of the gpio in the port */
    uint8_t index;
    /** state of the gpio to set */
    uint8_t state;
} gpio_set_req_msg_t;

typedef struct gpio_set_rsp_msg {
    struct cfw_rsp_message rsp_header;
} gpio_set_rsp_msg_t;

typedef struct gpio_listen_req_msg {
    struct cfw_message header;
    /** index of GPIO to monitor */
    uint8_t index;
    /** interrupt mode */
    gpio_service_isr_mode_t mode;
    /** debounce mode */
    gpio_service_debounce_mode_t debounce;
} gpio_listen_req_msg_t;

typedef struct gpio_listen_rsp_msg {
    struct cfw_rsp_message rsp_header;
    /** index of GPIO to monitor */
    uint8_t index;
} gpio_listen_rsp_msg_t;

typedef struct gpio_listen_evt_msg {
    struct cfw_message header;
    /** index of GPIO which triggered the interrupt */
    uint8_t index;
    /** gpio current value */
    uint32_t pin_state;
} gpio_listen_evt_msg_t;

typedef struct gpio_unlisten_req_msg {
    struct cfw_message header;
    /** index of GPIO to monitor */
    uint8_t index;
    /** interrupt mode */
} gpio_unlisten_req_msg_t;

typedef struct gpio_unlisten_rsp_msg {
    struct cfw_rsp_message rsp_header;
    /** index of GPIO to monitor */
    uint8_t index;
} gpio_unlisten_rsp_msg_t;

typedef struct gpio_get_req_msg {
    struct cfw_message header;
} gpio_get_req_msg_t;

typedef struct gpio_get_rsp_msg {
    struct cfw_rsp_message rsp_header;
    /** state of the gpio port */
    uint32_t state;
} gpio_get_rsp_msg_t;

/**
 * \brief configure gpio_line
 *
 * Configures a GPIO line.
 *
 * \param svc_handle the service connection handle
 * \param index the GPIO index in the port to configure
 * \param mode the mode of the GPIO line
 *        0 - input
 *        1 - output
 * \param priv the private data passed back in the
 *             response message.
 */
int gpio_configure(svc_client_handle_t * svc_handle, uint8_t index,
        uint8_t mode, void * priv);

/**
 * \brief Set the state of a GPIO line.
 *
 * \param svc_handle the service connection handle
 * \param index the GPIO index in the port to configure
 * \param value the state to set the GPIO line
 *        0 - low level
 *        1 - high level
 * \param priv the private data passed back in the
 *             response message.
 */
int gpio_set_state(svc_client_handle_t * svc_handle, uint8_t index,
        uint8_t value, void * priv);

/**
 * \brief get state of a GPIO line
 *
 * \param svc_handle the service connection handle
 * \param priv the private data passed back in the
 *             response message.
 *
 * GPIO state will be available in the \ref gpio_get_state_rsp_msg_t
 */
int gpio_get_state(svc_client_handle_t * svc_handle, void * priv);

/**
 * \brief register to gpio state change.
 *
 * \param svc_handle:  the service connection handle
 * \param pin       :  the GPIO index in the port to configure
 * \param mode      :  interrupt mode (RISING_EDGE, FALLING_EDGE)
 * \param debounce  :  debounce config (DEB_OFF, DEB_ON)
 * \param priv      :  the private data passed back in the response message.
 *
 * GPIO state and changed mask are available in the \ref gpio_listen_rsp_msg_t message.
 */
int gpio_listen(svc_client_handle_t * h, uint8_t pin, gpio_service_isr_mode_t mode, uint8_t debounce, void *priv);

/**
 * \brief unregister to gpio state change.
 *
 * \param svc_handle:  the service connection handle
 * \param pin       :  the GPIO index in the port to configure
 * \param priv      :  the private data passed back in the response message.
 *
 * GPIO state and changed mask are available in the \ref gpio_unlisten_rsp_msg_t message.
 */
int gpio_unlisten(svc_client_handle_t * h, uint8_t pin, void *priv);

#endif

/**@} @} @}*/
