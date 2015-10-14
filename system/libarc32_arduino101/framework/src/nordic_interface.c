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

#include <string.h>
#include "os/os.h"
#include "cfw/cfw.h"
#include "cfw/cfw_debug.h"
#include "cfw/cfw_internal.h"
#include "cfw/cfw_messages.h"
#include "infra/ipc_uart.h"
#include "infra/ipc_requests.h"
#include "infra/log.h"
#include "infra/message.h"
#include "infra/time.h"
#include "drivers/soc_gpio.h"
#include "platform.h"

#include "portable.h"

/* Macro definition for reset pin */
#define BLE_SW_CLK_PIN  27
#define BLE_SWDIO_PIN   6
#define RESET_PIN       BLE_SWDIO_PIN
#define ATP_BLE_INT     5

static T_QUEUE service_mgr_queue;

/* When we set val to 1 it will wake up the Nordic, setting it to 0
 * will set it back to sleep. */
static int nordic_wake_assert(bool val)
{
	uint8_t ret = soc_gpio_write(SOC_GPIO_32_ID, ATP_BLE_INT, val);
	if (ret != DRV_RC_OK)
		pr_error(LOG_MODULE_IPC, "Error setting ATP_BLE_INT %d", val);
	return ret;
}

void nordic_wake_deassert(void* ignored)
{
	nordic_wake_assert(0);
}

int send_message_ipc_uart(struct cfw_message * message) {
	int ret = uart_ipc_send_message(uart_ipc_get_channel(),
			CFW_MESSAGE_LEN(message), message);
	message_free((struct message *)message);
	return ret;
}

void free_message_ipc_uart(void * ptr) {
	bfree(ptr);
}

/**
 * IPC CFW message format is the following (little endian):
 * -------------------------------------
 * |  len: 2 bytes  | chan 1 byte: sender cpu id: 1 byte |
 * -------------------------------------
 * | REQUEST_ID   | payload             |
 * -------------------------------------
 *
 * For TYPE_MESSAGE request, the payload is the message copy.
 * For TYPE_FREE is not valid (this ipc is not shared mem based)
 */
void uart_ipc_message_cback(uint8_t cpu_id, int channel, int len, void * p_data)
{
	struct cfw_message * msg;
	unsigned int request = *(uint32_t*)p_data;

	switch (request) {
		case IPC_MSG_TYPE_MESSAGE:
		{
			OS_ERR_TYPE error = E_OS_OK;
			int size = len - sizeof(uint32_t);
			msg = (struct cfw_message *)message_alloc(size, &error);
			if (error != E_OS_OK)
				pr_error(LOG_MODULE_IPC, "NO MEM: error: %d size: %d", error, size);
			else {
				memcpy(msg, (uint8_t *)p_data + sizeof(uint32_t), size);
				handle_ipc_sync_request(cpu_id, request, 0, 0, msg);
			}
			break;
		}
		case IPC_REQUEST_ALLOC_PORT:
		{
			unsigned int ret;
			unsigned int error;
			uint16_t port_id = port_alloc(NULL);
			port_set_cpu_id(port_id, cpu_id);
			ret = port_id;
			pr_info(LOG_MODULE_IPC, "%s return port_id %d", __func__, ret);
			error = uart_ipc_send_sync_resp(channel, request, ret, 0, NULL);
			if (error)
				pr_error(LOG_MODULE_IPC, "%s returned error from ipc uart sync resp %d", __func__, error);
			break;
		}
		default:
		{
			unsigned int error;
			int32_t * p = ((int32_t *) p_data) + 1;
			int32_t param0 = *p++;
			int32_t param1 = *p++;
			void * ptr = (void *) *p;
			pr_info(LOG_MODULE_IPC, "%s request %xh, param1 %d, param2 %d", __func__, request, param0, param1);
			handle_ipc_sync_request(cpu_id, request, param0, param1, ptr);
			error = uart_ipc_send_sync_resp(channel, request, 0, 0, NULL);
			if (error)
				pr_error(LOG_MODULE_IPC, "%s returned error from ipc uart sync resp %d", __func__, error);
			break;
		}
	}
	bfree(p_data);

	/* Dequeue and process any new messages received */
	while(queue_process_message(service_mgr_queue) != 0);
}

/* Nordic reset is achieved by asserting low the SWDIO pin.
 * However, the Nordic chip can be in SWD debug mode, and NRF_POWER->RESET = 0 due to,
 * other constraints: therefore, this reset might not work everytime, especially after
 * flashing or debugging.
 */
static int nordic_reset(void)
{
	/* RESET_PIN depends on the board and the local configuration: check top of file */
	uint32_t delay_until;
	gpio_cfg_data_t pin_cfg = { .gpio_type = GPIO_OUTPUT };

	soc_gpio_set_config(SOC_GPIO_32_ID, RESET_PIN, &pin_cfg);

	/* Reset hold time is 0.2us (normal) or 100us (SWD debug) */
	soc_gpio_write(SOC_GPIO_32_ID, RESET_PIN, 0);
	/* Wait for ~1ms */
	delay_until = get_uptime_32k() + 32768;
	while (get_uptime_32k() < delay_until);
	/* De-assert the reset */
	soc_gpio_write(SOC_GPIO_32_ID, RESET_PIN, 1);

	/* Set back GPIO to input to avoid interfering with external debugger */
	pin_cfg.gpio_type = GPIO_INPUT;
	soc_gpio_set_config(SOC_GPIO_32_ID, RESET_PIN, &pin_cfg);

	return 0;
}

int nordic_interface_init(T_QUEUE queue)
{
	uint8_t ret;
	gpio_cfg_data_t config;

	service_mgr_queue = queue;

	config.gpio_type = GPIO_OUTPUT;
	ret = soc_gpio_set_config(SOC_GPIO_32_ID, ATP_BLE_INT, &config);
	if (ret != DRV_RC_OK)
		return -1;
	ret = nordic_wake_assert(1);
	if (ret != DRV_RC_OK)
		return -1;
	ret = nordic_reset();
	if (ret != DRV_RC_OK)
		return -1;

	return 0;
}
