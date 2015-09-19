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

#ifndef __CURIE_FACTORY_DATA_H_
#define __CURIE_FACTORY_DATA_H_

/**
 * @defgroup infra_curie_factory_data Intel&reg; Curie&trade; Factory Data
 * Intel&reg; Curie&trade; Factory Data API only.
 * @ingroup infra
 * @{
 */

#define MAC_ADDRESS_SIZE 6
#define BT_DEVICE_NAME_SIZE 20

enum mac_address_type {
    MAC_ADDR_TYPE_PUBLIC,
    MAC_ADDR_TYPE_STATIC
};

/**
 * oem_data have a reserved project_data field for project specific use
 * For Intel&reg; Curie&trade;, project_data is used for store MAC address type and MAC address.
 */

struct curie_oem_data {
    /** Intel mac address type. 0: public, 1: static random*/
    uint8_t bt_mac_address_type;
    uint8_t bt_address[MAC_ADDRESS_SIZE];
    uint8_t ble_name[BT_DEVICE_NAME_SIZE]; /* name should include the NULL termination character */
} __packed;

/** @} */

#endif /** __CURIE_FACTORY_DATA_H_ */
