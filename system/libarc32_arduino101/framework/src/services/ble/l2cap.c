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

#include <errno.h>
#include <atomic.h>

#include <bluetooth/gatt.h>

#include "conn_internal.h"
#include "gatt_internal.h"
#include "smp.h"

/* #define BT_GATT_DEBUG 1 */

extern void on_nble_curie_log(char *fmt, ...);
extern void __assert_fail(void);
#ifdef BT_GATT_DEBUG
#define BT_DBG(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_ERR(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_WARN(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_INFO(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_ASSERT(cond) ((cond) ? (void)0 : __assert_fail())
#else
#define BT_DBG(fmt, ...) do {} while (0)
#define BT_ERR(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_WARN(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_INFO(fmt, ...) on_nble_curie_log(fmt, ##__VA_ARGS__)
#define BT_ASSERT(cond) ((cond) ? (void)0 : __assert_fail())
#endif

void bt_l2cap_connected(struct bt_conn *conn)
{
	/* TODO Add support for fixed channels on BR/EDR transport */
	if (conn->type != BT_CONN_TYPE_LE) {
		return;
	}
#if defined(CONFIG_BLUETOOTH_SMP)
	bt_smp_connected(conn);
#endif
	bt_gatt_connected(conn);
}

void bt_l2cap_disconnected(struct bt_conn *conn)
{
#if defined(CONFIG_BLUETOOTH_SMP)
	bt_smp_disconnected(conn);
#endif
	bt_gatt_disconnected(conn);
}
