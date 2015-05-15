/*
Copyright (c) 2015 Intel Corporation.  All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/
#include <stdarg.h>
#include <string.h>

#include "drivers/soc_gpio.h"
#include "drivers/arcv2_timer1.h"
#include "scss_registers.h"

#include "infra/ipc.h"
#include "cfw/cfw.h"
#include "cfw/cfw_debug.h"
#include "cfw/cfw_messages.h"
#include "cfw/cfw_internal.h"
#include "cfw/cfw_service.h"
#include "infra/log.h"
#include "infra/port.h"
#include "platform.h"
#include "services/gpio_service.h"

#define GPIO_2 2

#define PIN13 GPIO_2

#define OUTPUT 0
#define INPUT  1

#define LOW  0
#define HIGH 1


static void configure_soc_gpio(pin, mode)
{
    gpio_cfg_data_t cfg;
    uint8_t bit;

    memset(&cfg, 0, sizeof(cfg));

    cfg.gpio_type = (mode == OUTPUT) ? GPIO_OUTPUT : GPIO_INPUT;

    switch(pin) {
    case 13:
        bit = 2;
        break;
    default:
        // Invalid/not-supported
        return;
    };

    soc_gpio_set_config(SOC_GPIO_32, bit, &cfg);
}

void pinMode(pin, mode)
{
    switch(pin) {
    case 13:
        configure_soc_gpio(pin, mode);
        break;
    default:
        // Invalid/not-supported
        return;
    };
}

void digitalWrite(pin, state)
{
    uint8_t bit;

    switch(pin) {
    case 13:
        bit = 2;
        break;
    default:
        // Invalid/not-supported
        return;
    };

    soc_gpio_write(SOC_GPIO_32, bit, state);
}

static T_QUEUE service_mgr_queue;
static uint8_t gpio_txled_index = 12;
static uint8_t gpio_rxled_index = 26;
static svc_client_handle_t * gpio_service_handle = NULL;
static volatile boolean_t gpio_txled_init;
static volatile boolean_t gpio_rxled_init;

#if 0
#define TIMER1_TICK	1000
void timer1_user_isr(void)
{
    pin_state = !pin_state;
}
#endif

void setup(void)
{
    pinMode(13, OUTPUT);
//    timer1_driver_init(timer1_user_isr, TIMER1_TICK);
}

static void handle_gpio_message(struct cfw_message * msg, void * param)
{
    switch (CFW_MESSAGE_ID(msg)) {
    case MSG_ID_CFW_OPEN_SERVICE: {
        cfw_open_conn_rsp_msg_t * cnf = (cfw_open_conn_rsp_msg_t*)msg;
        gpio_service_handle = cnf->client_handle;
        gpio_configure(gpio_service_handle, gpio_txled_index, 1 /* output */,
                       (void *)(unsigned long)gpio_txled_index);
        gpio_configure(gpio_service_handle, gpio_rxled_index, 1 /* output */,
                       (void *)(unsigned long)gpio_rxled_index);
    }
        break;

    case MSG_ID_GPIO_CONFIGURE_RSP: {
        uint8_t index = (uint8_t)(unsigned long)(msg->priv);
        if (index == gpio_txled_index) {
            gpio_txled_init = true;
            gpio_set_state(gpio_service_handle, gpio_txled_index, 0, NULL);
        }
        if (index == gpio_rxled_index) {
            gpio_rxled_init = true;
            gpio_set_state(gpio_service_handle, gpio_rxled_index, 0, NULL);
        }
    }
        break;
    case MSG_ID_GPIO_SET_RSP:
    case MSG_ID_GPIO_GET_RSP:
    case MSG_ID_GPIO_LISTEN_RSP:
    case MSG_ID_GPIO_UNLISTEN_RSP:
    case MSG_ID_GPIO_EVT:
        /* TODO - add response message handling if needed */
        break;
    }
    cfw_msg_free(msg);
}

void gpio_client_init(T_QUEUE queue)
{
    cfw_handle_t *h = cfw_init(queue, handle_gpio_message, "client");
    cfw_open_service(h, SOC_GPIO_SERVICE_ID, "connect");
}

#define DELAY_CYCLES 100000
void loop(void)
{
    static uint32_t loop_count;
    static uint8_t pin_state;

    if (++loop_count > DELAY_CYCLES) {
        loop_count = 0;
        pin_state = !pin_state;
        if (gpio_txled_init)
            gpio_set_state(gpio_service_handle, gpio_txled_index, pin_state ? 0 : 1, NULL);
        if (gpio_rxled_init)
            gpio_set_state(gpio_service_handle, gpio_rxled_index, pin_state ? 0 : 1, NULL);
    }
}

int send_message_ipc(struct cfw_message * msg) {
    return ipc_request_sync_int(IPC_MSG_TYPE_MESSAGE, 0, 0, msg);
}

void free_message_ipc(void * msg) {
    ipc_request_sync_int(IPC_MSG_TYPE_FREE, 0, 0, msg);
}

#define TMP_SIZE 256
static char tmp[TMP_SIZE];

void cfw_log(char * fmt, ... ) {
    va_list args;
    uint32_t timeout;
    va_start(args, fmt);
    unsigned int tstamp = SCSS_REG_VAL(SCSS_AONC_CNT);
    int hours = tstamp / (32191 * 60 * 60);
    tstamp -= hours * (32191 * 60 * 60);
    int minutes = (tstamp) / (32191 * 60);
    tstamp -= minutes * (32191 * 60);
    int seconds = (tstamp) / 32191;
    tstamp -= seconds * 32191;
    int millis = (tstamp) / 32;
    int it = interrupt_lock();

    sprintf(tmp, "%03d:%02d:%02d.%03d  ", hours, minutes, seconds, millis);
    vsnprintf(tmp + strlen(tmp) - 1, TMP_SIZE, fmt, args);
    //pr_info(LOG_MODULE_MAIN, tmp);
    MBX_DAT0(4) = (unsigned int)tmp;
    MBX_DAT1(4) = 0;
    MBX_CTRL(4) = 0x80000000;
    timeout = get_timestamp() + 10000;
    while((MBX_STS(4) & 0x1) && (get_timestamp() < timeout))
        ;

    interrupt_unlock(it);
    va_end(args);
}

int main(void)
{
    /* CFW IPC initialisation */
    ipc_init(5, 0, 6, 1, CPU_ID_LMT);
    service_mgr_queue = queue_create(10, NULL);
    _cfw_init_proxy(service_mgr_queue, shared_data->ports,
                    shared_data->services, shared_data->service_mgr_port_id);
    set_cpu_id(CPU_ID_ARC);
    set_cpu_message_sender(0, send_message_ipc);
    set_cpu_free_handler(0, free_message_ipc);

    soc_gpio_enable(SOC_GPIO_32);
    SET_PIN_MODE(2, QRK_PMUX_SEL_MODEA);

    gpio_client_init(service_mgr_queue);

    setup();
    for(;;) {
        queue_process_message(service_mgr_queue);
        ipc_handle_message();
//	    __asm__("nop");
        loop();
    }

    __builtin_unreachable();
}
