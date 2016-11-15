/* uuid.c - Bluetooth UUID handling */

/*
 * Copyright (c) 2015 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//#include <nanokernel.h>
//#include <toolchain.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/byteorder.h>

#include <bluetooth/uuid.h>

#if defined(CONFIG_BLUETOOTH_DEBUG)
#include <stdio.h>
#endif /* CONFIG_BLUETOOTH_DEBUG */

#define UUID_16_BASE_OFFSET 12

/* TODO: Decide whether to continue using BLE format or switch to RFC 4122 */

/* Base UUID : 0000[0000]-0000-1000-8000-00805F9B34FB ->
 * { 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
 *   0x00, 0x10, 0x00, 0x00, [0x00, 0x00], 0x00, 0x00 }
 * 0x2800    : 0000[2800]-0000-1000-8000-00805F9B34FB ->
 * { 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
 *   0x00, 0x10, 0x00, 0x00, [0x00, 0x28], 0x00, 0x00 }
 *  little endian 0x2800 : [00 28] -> no swapping required
 *  big endian 0x2800    : [28 00] -> swapping required
 */
static const struct bt_uuid_128 uuid128_base = {
	.uuid.type = BT_UUID_TYPE_128,
	.val = { 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
		 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
};

static inline void u16_to_uuid128(void *dst, uint16_t u16)
{
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	u16 = bswap_16(u16);
#endif
	memcpy(dst, &u16, 2);
}

static void uuid_to_uuid128(const struct bt_uuid *src, struct bt_uuid_128 *dst)
{
	switch (src->type) {
	case BT_UUID_TYPE_16:
		*dst = uuid128_base;
		u16_to_uuid128(&dst->val[UUID_16_BASE_OFFSET],
			       BT_UUID_16(src)->val);
		return;
	case BT_UUID_TYPE_128:
		memcpy(dst, src, sizeof(*dst));
		return;
	}
}

static int uuid128_cmp(const struct bt_uuid *u1, const struct bt_uuid *u2)
{
	struct bt_uuid_128 uuid1, uuid2;

	uuid_to_uuid128(u1, &uuid1);
	uuid_to_uuid128(u2, &uuid2);

	return memcmp(uuid1.val, uuid2.val, 16);
}

int bt_uuid_cmp(const struct bt_uuid *u1, const struct bt_uuid *u2)
{
	/* Convert to 128 bit if types don't match */
	if (u1->type != u2->type)
		return uuid128_cmp(u1, u2);

	switch (u1->type) {
	case BT_UUID_TYPE_16:
		return memcmp(&BT_UUID_16(u1)->val, &BT_UUID_16(u2)->val, 2);//(int)BT_UUID_16(u1)->val - (int)BT_UUID_16(u2)->val;
	case BT_UUID_TYPE_128:
		return memcmp(BT_UUID_128(u1)->val, BT_UUID_128(u2)->val, 16);
	}

	return -EINVAL;
}

#if defined(CONFIG_BLUETOOTH_DEBUG)
void bt_uuid_to_str(const struct bt_uuid *uuid, char *str, size_t len)
{
	uint32_t tmp1, tmp5;
	uint16_t tmp0, tmp2, tmp3, tmp4;

	switch (uuid->type) {
	case BT_UUID_TYPE_16:
		snprintf(str, len, "%.4x", BT_UUID_16(uuid)->val);
		break;
	case BT_UUID_TYPE_128:
		memcpy(&tmp0, &BT_UUID_128(uuid)->val[0], sizeof(tmp0));
		memcpy(&tmp1, &BT_UUID_128(uuid)->val[2], sizeof(tmp1));
		memcpy(&tmp2, &BT_UUID_128(uuid)->val[6], sizeof(tmp2));
		memcpy(&tmp3, &BT_UUID_128(uuid)->val[8], sizeof(tmp3));
		memcpy(&tmp4, &BT_UUID_128(uuid)->val[10], sizeof(tmp4));
		memcpy(&tmp5, &BT_UUID_128(uuid)->val[12], sizeof(tmp5));

		snprintf(str, len, "%.8x-%.4x-%.4x-%.4x-%.8x%.4x",
			 tmp5, tmp4, tmp3, tmp2, tmp1, tmp0);
		break;
	default:
		memset(str, 0, len);
		return;
	}
}

const char *bt_uuid_str(const struct bt_uuid *uuid)
{
	static char str[37];

	bt_uuid_to_str(uuid, str, sizeof(str));

	return str;
}
#endif /* CONFIG_BLUETOOTH_DEBUG */
