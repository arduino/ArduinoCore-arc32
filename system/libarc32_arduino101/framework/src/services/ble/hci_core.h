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

/* State tracking for the local Bluetooth controller */
struct bt_dev {
	atomic_t flags[1];
};
extern struct bt_dev bt_dev;

#if defined(CONFIG_BLUETOOTH_SMP) || defined(CONFIG_BLUETOOTH_BREDR)
extern const struct bt_conn_auth_cb *bt_auth;
#endif /* CONFIG_BLUETOOTH_SMP || CONFIG_BLUETOOTH_BREDR */

static inline bool bt_addr_le_is_rpa(const bt_addr_le_t *addr)
{
	if (addr->type != BT_ADDR_LE_RANDOM)
		return false;

	if ((addr->val[5] & 0xc0) == 0x40)
	       return true;

	return false;
}

static inline bool bt_addr_le_is_identity(const bt_addr_le_t *addr)
{
	if (addr->type == BT_ADDR_LE_PUBLIC)
		return true;

	/* Check for Random Static address type */
	if ((addr->val[5] & 0xc0) == 0xc0)
		return true;

	return false;
}

static inline bool bt_le_conn_params_valid(uint16_t min, uint16_t max,
					uint16_t latency, uint16_t timeout)
{
	if (min > max || min < 6 || max > 3200) {
		return false;
	}

	/* Limits according to BT Core spec 4.2 [Vol 2, Part E, 7.8.12] */
	if (timeout < 10 || timeout > 3200) {
		return false;
	}

	/* Limits according to BT Core spec 4.2 [Vol 6, Part B, 4.5.1] */
	if (latency > 499 || ((latency + 1) * max) > (timeout * 4)) {
		return false;
	}

	return true;
}


