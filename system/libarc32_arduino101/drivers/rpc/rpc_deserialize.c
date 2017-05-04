/*
 * Copyright (c) 2016 Intel Corporation
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

#include <stdlib.h>
#include <string.h>

#include "rpc.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

extern void panic(int err);

/* Include the functions offered */
#if defined(CONFIG_QUARK_SE_BLE_CORE)
#include "rpc_functions_to_ble_core.h"
#elif defined(CONFIG_SOC_QUARK_SE)
#include "rpc_functions_to_quark.h"
#elif defined(LINUX_HOST_RUNTIME)
// for the host compilation (to simulate connection to BLE controller)
#include "rpc_functions_to_ble_core.h"
#else
#error "File is compiled but should not"
#endif

/* Build the list of prototypes and check that list are made only of matching
 * signatures
 */
#define FN_SIG_NONE(__fn)	void __fn(void);
LIST_FN_SIG_NONE
#undef FN_SIG_NONE

#define FN_SIG_S(__fn, __s)	void __fn(__s p_s);
LIST_FN_SIG_S
#undef FN_SIG_S

#define FN_SIG_P(__fn, __type)	void __fn(__type priv);
LIST_FN_SIG_P
#undef FN_SIG_P

#define FN_SIG_S_B(__fn, __s, __type, __length)				\
	void __fn(__s p_s, __type buf, __length length);
LIST_FN_SIG_S_B
#undef FN_SIG_S_B

#define FN_SIG_S_P(__fn, __s, __type)	void __fn(__s p_s, __type priv);
LIST_FN_SIG_S_P
#undef FN_SIG_S_P

/* 1 - define the size check arrays */
#define FN_SIG_NONE(__fn)

#define FN_SIG_S(__fn, __s)				sizeof(*((__s)0)),

#define FN_SIG_P(__fn, __type)

#define FN_SIG_S_B(__fn, __s, __type, __length)		sizeof(*((__s)0)),

#define FN_SIG_S_P(__fn, __s, __type)			sizeof(*((__s)0)),

static uint8_t m_size_s[] = { LIST_FN_SIG_S };
static uint8_t m_size_s_b[] = { LIST_FN_SIG_S_B };
static uint8_t m_size_s_p[] = { LIST_FN_SIG_S_P };

#undef FN_SIG_NONE
#undef FN_SIG_S
#undef FN_SIG_P
#undef FN_SIG_S_B
#undef FN_SIG_S_P

/* 2- build the enumerations list */
#define FN_SIG_NONE(__fn)				fn_index_##__fn,
#define FN_SIG_S(__fn, __s)				FN_SIG_NONE(__fn)
#define FN_SIG_P(__fn, __type)				FN_SIG_NONE(__fn)
#define FN_SIG_S_B(__fn, __s, __type, __length)		FN_SIG_NONE(__fn)
#define FN_SIG_S_P(__fn, __s, __type)			FN_SIG_NONE(__fn)

/* Build the list of function indexes in the deserialization array */
enum { LIST_FN_SIG_NONE fn_none_index_max };
enum { LIST_FN_SIG_S fn_s_index_max };
enum { LIST_FN_SIG_P fn_p_index_max };
enum { LIST_FN_SIG_S_B fn_s_b_index_max };
enum { LIST_FN_SIG_S_P fn_s_p_index_max };

#undef FN_SIG_NONE
#undef FN_SIG_S
#undef FN_SIG_P
#undef FN_SIG_S_B
#undef FN_SIG_S_P

/* 3- build the array */
#define FN_SIG_NONE(__fn)			[fn_index_##__fn] =	\
								(void *)__fn,
#define FN_SIG_S(__fn, __s)			FN_SIG_NONE(__fn)
#define FN_SIG_P(__fn, __type)			FN_SIG_NONE(__fn)
#define FN_SIG_S_B(__fn, __s, __type, __length)				\
						FN_SIG_NONE(__fn)
#define FN_SIG_S_P(__fn, __s, __type)		FN_SIG_NONE(__fn)

static void (*m_fct_none[])(void) = { LIST_FN_SIG_NONE };
static void (*m_fct_s[])(void *structure) = { LIST_FN_SIG_S };
static void (*m_fct_p[])(void *pointer) = { LIST_FN_SIG_P };
static void (*m_fct_s_b[])(void *structure, void *buffer,
			   uint16_t length) = { LIST_FN_SIG_S_B };
static void (*m_fct_s_p[])(void *structure,
			   void *pointer) = { LIST_FN_SIG_S_P };

#undef FN_SIG_NONE
#undef FN_SIG_S
#undef FN_SIG_P
#undef FN_SIG_S_B
#undef FN_SIG_S_P

#define DJB2_HASH(__h, __v) ((__h << 5) + __h) + __v

#define FN_SIG_NONE(__fn) \
	hash = DJB2_HASH(hash, 1);

#define FN_SIG_S(__fn, __s) \
	hash = DJB2_HASH(hash, 2); \
	hash = DJB2_HASH(hash, sizeof(*((__s)0)));

#define FN_SIG_P(__fn, __type) \
	hash = DJB2_HASH(hash, 3);

#define FN_SIG_S_B(__fn, __s, __type, __length) \
	hash = DJB2_HASH(hash, 4); \
	hash = DJB2_HASH(hash, sizeof(*((__s)0)));

#define FN_SIG_S_P(__fn, __s, __type) \
	hash = DJB2_HASH(hash, 6); \
	hash = DJB2_HASH(hash, sizeof(*((__s)0)));

uint32_t rpc_deserialize_hash(void)
{
	uint32_t hash = 5381;

	LIST_FN_SIG_NONE;
	LIST_FN_SIG_S;
	LIST_FN_SIG_P;
	LIST_FN_SIG_S_B;
	LIST_FN_SIG_S_P;

	return hash;
}

static const uint8_t *deserialize_struct(const uint8_t *p,
					 const uint8_t **struct_ptr,
					 uint8_t *struct_length)
{
	*struct_length = *p++;
	*struct_ptr = p;

	return p + *struct_length;
}

static const uint8_t *deserialize_buf(const uint8_t *p, const uint8_t **buf_ptr,
				      uint16_t *buf_len)
{
	uint8_t b;

	/* Get the current byte */
	b = *p++;
	*buf_len = b & 0x7F;
	if (b & 0x80) {
		/* Get the current byte */
		b = *p++;
		*buf_len += (uint16_t)b << 7;
	}

	/* Return the values */
	*buf_ptr = p;
	p += *buf_len;
	return p;
}

static const uint8_t *deserialize_ptr(const uint8_t *p, uintptr_t *priv)
{
	*priv = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
	p += 4;
	return p;
}

static void deserialize_none(uint8_t fn_index, const uint8_t *buf,
			     uint16_t length)
{
	if (length != 0)
		panic(-1);

	rpc_free_cb(buf - 2);

	m_fct_none[fn_index]();
}

static void deserialize_s(uint8_t fn_index, const uint8_t *buf,
			  uint16_t length)
{
	const uint8_t *struct_ptr;
	uint8_t struct_length;
	const uint8_t *p = buf;

	p = deserialize_struct(p, &struct_ptr, &struct_length);

	if ((length != (p - buf)) || (struct_length != m_size_s[fn_index]))
		panic(-1);

	{
		/* Always align structures on word boundary */
		uintptr_t struct_data[(struct_length +
				(sizeof(uintptr_t) - 1))/(sizeof(uintptr_t))];

		memcpy(struct_data, struct_ptr, struct_length);

		rpc_free_cb(buf - 2);

		m_fct_s[fn_index](struct_data);
	}
}

static void deserialize_p(uint8_t fn_index, const uint8_t *buf,
			  uint16_t length)
{
	uintptr_t priv;
	const uint8_t *p = buf;

	p = deserialize_ptr(p, &priv);

	if (length != (p - buf))
		panic(-1);

	rpc_free_cb(buf - 2);

	m_fct_p[fn_index]((void *)priv);
}

static void deserialize_s_b(uint8_t fn_index, const uint8_t *buf,
			    uint16_t length)
{
	const uint8_t *p_struct_data;
	uint8_t struct_length;
	const uint8_t *p_vbuf;
	uint16_t vbuf_length;
	const uint8_t *p = buf;

	p = deserialize_struct(p, &p_struct_data, &struct_length);
	p = deserialize_buf(p, &p_vbuf, &vbuf_length);

	if ((length != (p - buf)) || (struct_length != m_size_s_b[fn_index]))
		panic(-1);

	{
		/* Always align structures on word boundary */
		uintptr_t struct_data[(struct_length +
				(sizeof(uintptr_t) - 1))/(sizeof(uintptr_t))];
		uintptr_t vbuf[(vbuf_length +
				(sizeof(uintptr_t) - 1))/(sizeof(uintptr_t))];

		memcpy(struct_data, p_struct_data, struct_length);

		if (vbuf_length) {
			memcpy(vbuf, p_vbuf, vbuf_length);
		}

		rpc_free_cb(buf - 2);

		m_fct_s_b[fn_index](struct_data, (void *)vbuf, vbuf_length);
	}
}

static void deserialize_s_p(uint8_t fn_index, const uint8_t *buf,
			    uint16_t length)
{
	const uint8_t *p_struct_data;
	uint8_t struct_length;
	uintptr_t priv;
	const uint8_t *p = buf;

	p = deserialize_struct(p, &p_struct_data, &struct_length);
	p = deserialize_ptr(p, &priv);

	if ((length != (p - buf)) || (struct_length != m_size_s_p[fn_index]))
		panic(-1);

	{
		/* Always align structures on word boundary */
		uintptr_t struct_data[(struct_length +
				(sizeof(uintptr_t) - 1))/(sizeof(uintptr_t))];

		memcpy(struct_data, p_struct_data, struct_length);

		rpc_free_cb(buf - 2);
		m_fct_s_p[fn_index](struct_data, (void *)priv);
	}
}

static void deserialize_control(uint8_t fn_index, const uint8_t *buf,
				uint16_t length)
{
	const uint8_t *p_struct_data;
	uint8_t struct_length;
	const uint8_t *p = buf;
	struct {
		uint32_t version;
		uint32_t ser_hash;
		uint32_t des_hash;
	} struct_data;

	switch(fn_index) {
	case 0:
		p = deserialize_struct(p, &p_struct_data, &struct_length);

		if ((length != (p - buf)) || (struct_length != sizeof(struct_data)))
			panic(-1);
		memcpy(&struct_data, p_struct_data, struct_length);

		rpc_free_cb(buf - 2);

		if (struct_data.ser_hash != rpc_deserialize_hash() ||
		    struct_data.des_hash != rpc_serialize_hash()) {
			rpc_init_cb(struct_data.version, false);
		} else {
			rpc_init_cb(struct_data.version, true);
		}
		break;
	}
}

void rpc_deserialize(const uint8_t *buf, uint16_t length)
{
	uint8_t fn_index;
	uint8_t sig_type;

	if (!buf) return;

	sig_type = buf[0];
	fn_index = buf[1];

	buf += 2;
	length -= 2;

	switch(sig_type) {
	case SIG_TYPE_NONE:
		if (sizeof(m_fct_none))
			deserialize_none(fn_index, buf, length);
		break;
	case SIG_TYPE_S:
		if (sizeof(m_fct_s))
			deserialize_s(fn_index, buf, length);
		break;
	case SIG_TYPE_P:
		if (sizeof(m_fct_p))
			deserialize_p(fn_index, buf, length);
		break;
	case SIG_TYPE_S_B:
		if (sizeof(m_fct_s_b))
			deserialize_s_b(fn_index, buf, length);
		break;
	case SIG_TYPE_S_P:
		if (sizeof(m_fct_s_p))
			deserialize_s_p(fn_index, buf, length);
		break;
	case SIG_TYPE_CONTROL:
		deserialize_control(fn_index, buf, length);
		break;
	default:
		panic(-1);
		break;
	}
}

__weak
void rpc_init_cb(uint32_t version, bool compatible)
{
}
