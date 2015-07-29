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
#include "services/cdc_serial_service.h"
#include "cfw/cfw_client.h"

/****************************************************************************************
 *********************** SERVICE API IMPLEMENATION **************************************
 ****************************************************************************************/

static cdc_recvbyte_cb_t byte_cb = NULL;
static cdc_sentbytes_cb_t sent_cb = NULL;
static cdc_init_cb_t init_cb = NULL;

/*
 *  allows registering of a callback to populate rx buffer in CDCSerial class.
 */
void cdc_register_byte_cb(cdc_recvbyte_cb_t cb){
	byte_cb = cb;
}

void cdc_register_sent_cb(cdc_sentbytes_cb_t cb){
	sent_cb = cb;
}

void cdc_register_init_cb(cdc_sentbytes_cb_t cb){
	init_cb = cb;
}

extern svc_client_handle_t * cdc_serial_service_handle;

int cdc_serial_service_send(char * buff, int len, void *priv) {
	int i;
    struct cfw_message * msg = cfw_alloc_message_for_service(cdc_serial_service_handle, MSG_ID_CDC_SERIAL_TX,
            sizeof(cdc_serial_msg_t), priv);
    cdc_serial_msg_t *req = (cdc_serial_msg_t*) msg;
    req->len = len;
	for (i=0;i<len;i++) {
		req->data[i] = buff[i];
	}
    req->buff = buff;
    cfw_send_message(msg);
    return 0;
}

int cdc_serial_service_sent(struct cfw_message *msg) {
    cdc_serial_tx_ack_msg_t *req = (cdc_serial_tx_ack_msg_t*) msg;
	sent_cb(req->num);
    return 0;
}

int cdc_serial_service_receive_from_lmt(struct cfw_message *msg) {
	int i;
    cdc_serial_msg_t *req = (cdc_serial_msg_t*) msg;

	for (i=0;i<req->len;i++) {
		// call the callback to populate the rx buffer in the CDCSerial class.
		byte_cb(req->data[i]);
	}

    return 0;
}
int cdc_serial_service_receive_init_ack(struct cfw_message *msg) {

    cdc_serial_init_ack_msg_t *req = (cdc_serial_init_ack_msg_t*) msg;

	init_cb(req->acm_open);

    return 0;

}

void cdc_serial_service_init(uint32_t baud, uint8_t config) {
    struct cfw_message * msg = cfw_alloc_message_for_service(cdc_serial_service_handle, MSG_ID_CDC_SERIAL_INIT,
            sizeof(cdc_serial_init_msg_t), NULL);
    cdc_serial_init_msg_t *req = (cdc_serial_init_msg_t*) msg;
	req->baudrate = 115200;
	req->parity   = config;
	req->databits = config;
	req->stopbits = config;
    cfw_send_message(msg);
}




