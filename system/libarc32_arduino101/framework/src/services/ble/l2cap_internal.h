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

#ifndef L2CAP_INTERNAL_H_
#define L2CAP_INTERNAL_H_

/* Notify L2CAP channels of a new connection */
void bt_l2cap_connected(struct bt_conn *conn);

/* Notify L2CAP channels of a disconnect event */
void bt_l2cap_disconnected(struct bt_conn *conn);


#endif /* L2CAP_INTERNAL_H_ */
