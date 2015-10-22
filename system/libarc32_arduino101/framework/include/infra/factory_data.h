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

#ifndef __FACTORY_DATA_H__
#define __FACTORY_DATA_H__

/**
 * @defgroup infra_factory_data Factory Data
 * Defines structures for OEM (Intel&reg;) and customer data (512 bytes each).
 * @ingroup infra
 * @{
 */

#include <stdint.h>

#ifndef __packed
#define __packed __attribute__((packed))
#endif

/** Magic string value mandatory in all factory_data struct. */
#define FACTORY_DATA_MAGIC "$FA!"

/** The version supported by this code. Increment each time the definition of
 * the factory_data struct is modified */
#define FACTORY_DATA_VERSION 0x01

/** Key Patterns in the OTP area */
#define PATTERN_KEY_START  0xA5A5A5A5
#define PATTERN_KEY_END    0x5A5A5A5A

enum hardware_type {
    EVT = 0x00,  /*!< Engineering hardware */
    DVT = 0x04,  /*!< Comes after EVT */
    PVT = 0x08,  /*!< Production-ready hardware  */
    PR = 0x09,   /*!< PRototype hardware */
    FF = 0x0c,   /*!< Form Factor hardware, like PR but with final product form factor */
    MAX_HARDWARE_TYPE_TYPE = 0x0F
};

/**
 * Describes an Intel hardware
 */
struct hardware_info {
    /** name of board */
    uint8_t hardware_name;
    /** type of board, used for debug and tracking */
    uint8_t hardware_type;
    /** revision number of board, used for debug and tracking */
    uint8_t hardware_revision;
    /** reserved for later use */
    uint8_t reserved;
    /** project-specific */
    uint32_t hardware_variant[3];
} __packed;

/**
 * Contains OEM, i.e. Intel data.
 * size: 512 bytes
 */
struct oem_data {
    /** Project-specific header. Keep to 0xFFFFFFFF by default */
    uint8_t header[4];

    /** Always equal to $FA! */
    uint8_t magic[4];

    /** OEM data format version */
    uint8_t version;

    /** OEM Production flag
     * factory mode = 0xFF, production mode = 0x01
     * shall be set to 0x01 at end of oem manufacturing process */
    uint8_t production_mode_oem;

    /** OEM Production flag
     * factory mode = 0xFF, production mode = 0x01
     * shall be set to 0x01 at end of customer manufacturing process */
    uint8_t production_mode_customer;

    /** reserved for later use */
    uint8_t reserved0;

    /** hardware info: 16 bytes */
    struct hardware_info hardware_info;

    /** UUID stored as binary data, usually displayed as hexadecimal.
     * It is used to generate debug token and is a fallback if factory_sn
     * is not unique */
    uint8_t uuid[16];

    /** factory serial number
     * Up to 32 ASCII characters. If its length is < 32, it is NULL terminated.*/
    char factory_sn[32];

    /** hardware identification number, stored as binary data
     * usually displayed as hexadecimal. Left padded with 0 if smaller than 32 bytes */
    uint8_t hardware_id[32];

    /** reserved for later use */
    uint8_t project_data[84];

    /** public keys and other infos used for secure boot and manufacturing
     * size: 64*4+128 bytes */
    uint8_t security[320];
} __packed;

/**
 * Contains Customer data.
 * size: 512 bytes
 */
struct customer_data {
    /** product serial number
     * Up to 16 ASCII characters. If its length is < 16, it is NULL terminated.*/
    uint8_t product_sn[16];

    /** product hardware version
     * hexadecimal value, left-padded */
    uint8_t product_hw_ver[4];

    uint8_t reserved[12];

    uint8_t board_name[32];

    uint8_t vendor_name[32];

    uint32_t product_sn_len;

    uint32_t board_name_len;

    uint32_t vendor_name_len;

    uint8_t reserved_1[388];

    uint32_t patternKeyStart;

    uint32_t blockVersionHi;

    uint32_t blockVersionLow;

    uint32_t patternKeyEnd;

} __packed;

/**
 * Defines the data provisioned at the factory for each device.
 *
 * Consist in 1 page of 1024 bytes to store OEM (Intel&reg;) and customer data:
 *  - 512 bytes for OEM
 *  - 512 bytes for customer
 */
struct factory_data {
    struct oem_data oem_data;
    struct customer_data customer_data;
} __packed;


/**
 * Pointer on the global factory_data instance. This instance is usually not part
 * of this binary but is instead flashed separately.
 */
extern const struct factory_data* global_factory_data;

/** @} */

#endif /* __FACTORY_DATA_H__ */
