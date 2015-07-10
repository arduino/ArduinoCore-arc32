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

#include "services/gpio_service.h"
#include "cfw/cfw_client.h"

/****************************************************************************************
 *********************** SERVICE API IMPLEMENATION **************************************
 ****************************************************************************************/

int gpio_configure(svc_client_handle_t * h, uint8_t index, uint8_t mode, void * priv)
{
    struct cfw_message * msg = cfw_alloc_message_for_service(h, MSG_ID_GPIO_CONFIGURE_REQ,
            sizeof(gpio_configure_req_msg_t), priv);
    gpio_configure_req_msg_t * req = (gpio_configure_req_msg_t*) msg;
    req->mode = mode;
    req->index = index;
    cfw_send_message(msg);
    return 0;
}

int gpio_set_state(svc_client_handle_t * h, uint8_t index, uint8_t val, void *priv)
{
    struct cfw_message * msg = cfw_alloc_message_for_service(h, MSG_ID_GPIO_SET_REQ,
            sizeof(gpio_set_req_msg_t), priv);
    gpio_set_req_msg_t * req = (gpio_set_req_msg_t*) msg;
    req->state = val;
    req->index = index;
    cfw_send_message(msg);
    return 0;
}

int gpio_get_state(svc_client_handle_t * h, void *priv)
{
    struct cfw_message * msg = cfw_alloc_message_for_service(h, MSG_ID_GPIO_GET_REQ, sizeof(*msg), priv);
    cfw_send_message(msg);
    return 0;
}

int gpio_listen(svc_client_handle_t * h, uint8_t pin, gpio_service_isr_mode_t mode, uint8_t debounce, void *priv)
{
    struct cfw_message * msg = cfw_alloc_message_for_service(h, MSG_ID_GPIO_LISTEN_REQ,
            sizeof(gpio_listen_req_msg_t), priv);
    gpio_listen_req_msg_t * req = (gpio_listen_req_msg_t*) msg;
    req->index = pin;
    req->mode = mode;
    req->debounce = debounce;
    cfw_send_message(msg);
    return 0;
}

int gpio_unlisten(svc_client_handle_t * h, uint8_t pin, void *priv)
{
    struct cfw_message * msg = cfw_alloc_message_for_service(h, MSG_ID_GPIO_UNLISTEN_REQ,
            sizeof(gpio_unlisten_req_msg_t), priv);
    gpio_unlisten_req_msg_t * req = (gpio_unlisten_req_msg_t*) msg;
    req->index = pin;
    cfw_send_message(msg);
    return 0;
}
