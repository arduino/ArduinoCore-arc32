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
#include "ble_protocol.h"
#include "services/ble/ble_service_gap_api.h"
#include "ble_service_utils.h"

uint8_t ble_sizeof_bt_uuid(const struct bt_uuid * p_uuid)
{
	return p_uuid->type + 1;
}

uint8_t * ble_encode_bt_uuid(const struct bt_uuid * p_uuid, uint8_t * p_data)
{
	// start with the type (length)
	UINT8_TO_LESTREAM(p_data, p_uuid->type);
	switch (p_uuid->type) {
	case BT_UUID16:
		UINT16_TO_LESTREAM(p_data, p_uuid->uuid16);
		break;
	case BT_UUID32:
		UINT32_TO_LESTREAM(p_data, p_uuid->uuid32);
		break;
	case BT_UUID128:
		memcpy(p_data, p_uuid->uuid128, MAX_UUID_SIZE);
		p_data += MAX_UUID_SIZE;
		break;
	}
	return p_data;
}

uint8_t ble_cpy_bt_uuid(uint8_t * p_uuid_dst,
		    const struct bt_uuid * p_uuid_src,
		    uint8_t type)
{
	uint8_t *p = p_uuid_dst;
	uint8_t res_type = (p_uuid_src->type >= type) ? type : 0;

	switch (res_type) {
	case BT_UUID16:
		switch (p_uuid_src->type) {
		case BT_UUID16:
			UINT16_TO_LESTREAM(p, p_uuid_src->uuid16);
			break;
		case BT_UUID32:
			UINT16_TO_LESTREAM(p, p_uuid_src->uuid16);
			break;
		case BT_UUID128: {
			uint16_t uuid16;
			p = (uint8_t *)&p_uuid_src->uuid128[BLE_BASE_UUID_OCTET_OFFSET];
			LESTREAM_TO_UINT16(p, uuid16);
			p = p_uuid_dst;
			UINT16_TO_LESTREAM(p, p_uuid_src->uuid16);
			break;
		}
		}
		break;
	case BT_UUID32:
		switch (p_uuid_src->type) {
		case BT_UUID32:
			UINT32_TO_LESTREAM(p, p_uuid_src->uuid32);
			break;
		case BT_UUID128: {
			uint32_t uuid32;
			p = (uint8_t *)&p_uuid_src->uuid128[BLE_BASE_UUID_OCTET_OFFSET];
			LESTREAM_TO_UINT32(p, uuid32);
			p = p_uuid_dst;
			UINT32_TO_LESTREAM(p, uuid32);
			break;
		}
		}
		break;
	case BT_UUID128:
		memcpy(p_uuid_dst, p_uuid_src->uuid128, MAX_UUID_SIZE);
		break;
	}
	return res_type;
}
