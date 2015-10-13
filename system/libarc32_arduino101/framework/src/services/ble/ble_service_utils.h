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

#ifndef __BLE_SERVICE_UTILS_H__
#define __BLE_SERVICE_UTILS_H__

#include "services/ble/ble_service_gap_api.h"
#include "services/ble/ble_service_gatt.h"
//#include "ble_service_int.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0])) + __must_be_array(a)

#define IS_BIG_ENDIAN (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)

#define UINT8_TO_LESTREAM(p, i) \
    do { *(p)++ = (uint8_t)(i); } \
    while (sizeof(char[1 - 2 * !(__builtin_types_compatible_p(__typeof__(p), uint8_t *))]) - 1)
#define UINT16_TO_LESTREAM(p, i) UINT8_TO_LESTREAM(p, i); UINT8_TO_LESTREAM(p, (i)>>8)
#define UINT32_TO_LESTREAM(p, i) UINT16_TO_LESTREAM(p, i); UINT16_TO_LESTREAM(p, (i)>>16)

#define INT8_TO_LESTREAM(p, i) UINT8_TO_STREAM(p, (uint8_t)(i))
#define INT16_TO_LESTREAM(p, i) UINT16_TO_STREAM(p, (uint16_t)(i))
#define INT32_TO_LESTREAM(p, i) UINT32_TO_STREAM(p, (uint32_t)(i))

#define UINT8_TO_BESTREAM(p, i) UINT8_TO_LESTREAM(p, i)
#define UINT16_TO_BESTREAM(p, i) UINT8_TO_BESTREAM(p, (i) >> 8); UINT8_TO_BESTREAM(p, i)
#define UINT32_TO_BESTREAM(p, i) UINT16_TO_BESTREAM(p, (i) >> 16); UINT16_TO_BESTREAM(p, i)

#define INT8_TO_BESTREAM(p, i) UINT8_TO_BESTREAM(p, (uint8_t)(i))
#define INT16_TO_BESTREAM(p, i) UINT16_TO_BESTREAM(p, (uint16_t)(i))
#define INT32_TO_BESTREAM(p, i) UINT32_TO_BESTREAM(p, (uint32_t)(i))

#define LESTREAM_TO_UINT8(p, i) \
    do { i = *p; p++; } \
    while (sizeof(char[1 - 2 * !(__builtin_types_compatible_p(__typeof__(p), const uint8_t *) || \
                                 __builtin_types_compatible_p(__typeof__(p), uint8_t *))]) - 1)
#define LESTREAM_TO_UINT16(p, i) \
    do { uint16_t temp16; LESTREAM_TO_UINT8(p, i); LESTREAM_TO_UINT8(p, temp16); i |= (temp16 << 8); } \
    while (0)
#define LESTREAM_TO_UINT32(p, i) \
    do { uint32_t temp32; LESTREAM_TO_UINT16(p, i); LESTREAM_TO_UINT16(p, temp32); i |= (temp32 << 16); } \
    while (0)

#define LESTREAM_TO_INT8(p, i) \
    do {uint8_t __i; LESTREAM_TO_UINT8(p, __i); i = (int8_t)__i; } while (0)
#define LESTREAM_TO_INT16(p, i) \
    do {uint16_t __i; LESTREAM_TO_UINT16(p, __i); i = (int16_t)__i; } while (0)
#define LESTREAM_TO_INT32(p, i) \
    do {uint32_t __i; LESTREAM_TO_UINT32(p, __i); i = (int32_t)__i; } while (0)

#define BESTREAM_TO_UINT8(p, i) LESTREAM_TO_UINT8(p, i)
#define BESTREAM_TO_UINT16(p, i) \
    do { uint16_t temp16; BESTREAM_TO_UINT8(p, temp16); BESTREAM_TO_UINT8(p, i); i |= (temp16 << 8); } \
    while (0)
#define BESTREAM_TO_UINT32(p, i) \
    do { uint32_t temp32; BESTREAM_TO_UINT16(p, temp32); BESTREAM_TO_UINT16(p, i); i |= (temp32 << 16); } \
    while (0)

#define BESTREAM_TO_INT8(p, i) \
    do {uint8_t __i; BESTREAM_TO_UINT8(p, __i); i = (int8_t)__i; } while (0)
#define BESTREAM_TO_INT16(p, i) \
    do {uint16_t __i; BESTREAM_TO_UINT16(p, __i); i = (int16_t)__i; } while (0)
#define BESTREAM_TO_INT32(p, i) \
    do {uint32_t __i; BESTREAM_TO_UINT32(p, __i); i = (int32_t)__i; } while (0)

#define between(a, b, c) (((a) >= (b)) || ((a) <= (c)))

#define strict_between(a, b, c) (((a) > (b)) || ((a) < (c)))

/**
 * BLE helper functions.
 */
#define BLE_BASE_UUID_OCTET_OFFSET 12

/**
 * Compute the size required to encode a UUID in a buffer
 *
 * @param p_uuid Pointer to the UUID
 */
uint8_t ble_sizeof_bt_uuid(const struct bt_uuid * p_uuid);

/**
 * Encode a UUID in a buffer
 *
 * @param p_uuid Pointer to the UUID to encode
 * @param p_data Pointer to the buffer to store the encoded UUID
 *
 * @return The pointer to the buffer after the encoded UUID
 */
uint8_t * ble_encode_bt_uuid(const struct bt_uuid * p_uuid, uint8_t * p_data);

/** Copy BT/BLE address and eventually reduce size.
 *
 * Copies a uuid from src to dst. type may modify the resulting uuid type. only
 * going from bigger to small type is supported. typically 128 bits to 16 bits
 * If it is different from src type, a transformation is applied:
 * IN: BT_UUID128: dd97c415-fed9-4766-b18f-ba690d24a06a (stored in little endian)
 * OUT: BT_UUID16: c415
 * OUT: BT_UUID32: dd97c415
 * or
 * IN: BT_UUID32: dd97c415
 * OUT: BT_UUID16: c415
 *
 * @param p_uuid_src source uuid
 * @param[out] p_uuid_dst: destination to copy uuid to (excluding type). little endian
 * @param type output type
 *
 * \return size/type of copied uuid: BT_UUID16, BT_UUID32, BT_UUID128 or 0 for failure!
 *
 */
uint8_t ble_cpy_bt_uuid(uint8_t * p_uuid_dst,
                        const struct bt_uuid * p_uuid_src,
                        uint8_t type);

#endif
