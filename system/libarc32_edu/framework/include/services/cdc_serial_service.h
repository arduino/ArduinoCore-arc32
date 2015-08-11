/*
  Copyright (c) 2015 Intel Corporation.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  CDC-ACM class for Intel EDU - Aug 2015 <dave.hunt@emutex.com>

*/
#ifndef __CDC_SERIAL_SERVICE_H__
#define __CDC_SERIAL_SERVICE_H__
#include "cfw/cfw.h"
#include "cfw/cfw_client.h"
#include "platform.h"

#define SERIAL_BUFFER_SIZE 256

// Only goes from ARC to LMT. TX data out of CDC-ACM port
#define MSG_ID_CDC_SERIAL_TX    MSG_ID_CDC_SERIAL_BASE + 3

// Only goes from LMT to ARC. Ack in response to TX
#define MSG_ID_CDC_SERIAL_TX_ACK     MSG_ID_CDC_SERIAL_BASE + 0x83

// Only goes from LMT to ARC. Carries data RX'd from CDC-ACM port
#define MSG_ID_CDC_SERIAL_RX_EVT MSG_ID_CDC_SERIAL_BASE + 0x1001

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*cdc_recvbyte_cb_t)(uint8_t uc_data, void *arg);
typedef void (*cdc_sentbytes_cb_t)(uint32_t num, void *arg);

void cdc_register_byte_cb(cdc_recvbyte_cb_t cb, void *arg);
void cdc_register_sent_cb(cdc_sentbytes_cb_t cb, void *arg);


int cdc_serial_service_send(char * buff, int len, void * priv);
void cdc_serial_service_sent(struct cfw_message *msg);
void cdc_serial_service_receive(struct cfw_message *msg);

typedef struct cdc_serial_msg {
	struct cfw_message header;
	int len;
	char data[SERIAL_BUFFER_SIZE];
} cdc_serial_msg_t;

typedef struct cdc_serial_tx_ack_msg {
	struct cfw_rsp_message rsp_header;
	int num;
} cdc_serial_tx_ack_msg_t;

#ifdef __cplusplus
}
#endif

#endif
