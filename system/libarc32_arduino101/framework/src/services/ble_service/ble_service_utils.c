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
#include "ble_service.h"
#include "ble_service_int.h"
#include "ble_service_utils.h"

void uint8_to_ascii(uint8_t in, uint8_t *p)
{
	uint8_t hi = (in & 0xF0) >> 4;
	uint8_t lo = in & 0x0F;

	if (hi < 0x0A)
		*p = '0' + hi;
	else
		*p = 'A' + (hi - 0x0A);

	p++;

	if (lo < 10)
		*p = '0' + lo;
	else
		*p = 'A' + (lo - 0x0A);
}

void uint8buf_to_ascii(uint8_t * dst, const uint8_t * src, int len)
{
	int i;
	for (i = 0; i < len; ++i) {
		uint8_to_ascii(src[i], dst);
		dst += 2;
	}
}

size_t adv_data_len(const struct bt_data *ad, size_t count)
{
	int i;
	size_t data_len = 0;

	if (ad == NULL)
		return 0;

	for (i = 0; i < count ; i++) {
		data_len += ad[i].data_len;
	}

	return data_len;
}
