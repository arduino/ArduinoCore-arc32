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
static void *byte_param = NULL;
static void *send_param = NULL;

extern volatile int cdc_service_available;

/*
 *  allows registering of a callback to populate rx buffer in CDCSerial class.
 */
void cdc_register_byte_cb(cdc_recvbyte_cb_t cb, void *arg){
	byte_cb = cb;
	byte_param = arg;
}

void cdc_register_sent_cb(cdc_sentbytes_cb_t cb, void *arg){
	sent_cb = cb;
	send_param = arg;
}

extern svc_client_handle_t * cdc_serial_service_handle;

int cdc_serial_service_send(char * buff, int len, void *priv) {
	int i;
	int ret=0;
	if (cdc_service_available) {
		struct cfw_message * msg = cfw_alloc_message_for_service(cdc_serial_service_handle, MSG_ID_CDC_SERIAL_TX,
				sizeof(cdc_serial_msg_t), priv);
		cdc_serial_msg_t *req = (cdc_serial_msg_t*) msg;
		req->len = len;
		for (i=0;i<len;i++) {
			req->data[i] = buff[i];
		}
		ret = cfw_send_message(msg);
	}
	return ret;
}

void cdc_serial_service_sent(struct cfw_message *msg) {
	cdc_serial_tx_ack_msg_t *req = (cdc_serial_tx_ack_msg_t*) msg;
	sent_cb(req->num, send_param);
}

void cdc_serial_service_receive(struct cfw_message *msg) {
	int i;
	cdc_serial_msg_t *req = (cdc_serial_msg_t*) msg;

	for (i=0;i<req->len;i++) {
		// call the callback to pass the data. 
		byte_cb(req->data[i], byte_param);
	}
}
