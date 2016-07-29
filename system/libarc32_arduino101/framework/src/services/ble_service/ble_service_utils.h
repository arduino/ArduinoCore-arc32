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

#ifndef BLE_SERVICE_UTILS_H_
#define BLE_SERVICE_UTILS_H_

#include <stdint.h>

// For HAVE_SAME_TYPE
#include "compiler.h"

#define IS_BIG_ENDIAN (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)

#define UINT8_TO_LESTREAM(p, i) \
    do { *(p)++ = (uint8_t)(i); } \
    while (sizeof(char[1 - 2 * !(HAVE_SAME_TYPE(p, uint8_t *))]) - 1)
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
    while (sizeof(char[1 - 2 * !(HAVE_SAME_TYPE(p, const uint8_t *) || \
                                 HAVE_SAME_TYPE(p, uint8_t *))]) - 1)
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

/**
 * BLE helper functions.
 */

/**@brief Represent an unsigned 8 bit value in ASCII hexadecimal
 *
 * @param[in] in The value to represent
 * @param[inout] p Pointer to the buffer to fill
 *
 * @note The representation requires 2 bytes
 */
void uint8_to_ascii(uint8_t in, uint8_t * p);

/** Converts buffer from hex to ascii.
 *
 * @param dst buffer converted
 * @param src buffer which is converted
 * @param len length of src (dst shall be large enough)
 *
 * @return None
 */
void uint8buf_to_ascii(uint8_t * dst, const uint8_t * src, int len);

struct bt_data;
/**
 * Get advertisement data length.
 *
 * @param ad advertisement data
 * @param count array length
 * @return length of advertisement data
 *
 */
size_t adv_data_len(const struct bt_data *ad, size_t count);

#endif /* BLE_SERVICE_UTILS_H_ */
