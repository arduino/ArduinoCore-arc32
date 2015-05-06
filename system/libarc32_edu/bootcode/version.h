/*
 * INTEL CONFIDENTIAL Copyright 2015 Intel Corporation All Rights Reserved.
 *
 * The source code contained or described herein and all documents related to
 * the source code ("Material") are owned by Intel Corporation or its suppliers
 * or licensors.
 * Title to the Material remains with Intel Corporation or its suppliers and
 * licensors.
 * The Material contains trade secrets and proprietary and confidential information
 * of Intel or its suppliers and licensors. The Material is protected by worldwide
 * copyright and trade secret laws and treaty provisions.
 * No part of the Material may be used, copied, reproduced, modified, published,
 * uploaded, posted, transmitted, distributed, or disclosed in any way without
 * Intel's prior express written permission.
 *
 * No license under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or delivery
 * of the Materials, either expressly, by implication, inducement, estoppel or
 * otherwise.
 *
 * Any license under such intellectual property rights must be express and
 * approved by Intel in writing
 */

#ifndef __VERSION_H__
#define __VERSION_H__

#include <stdint.h>

/**
 * @addtogroup infra
 * @{
 * @defgroup infra_version Binary Version Header Definition
 * @}
 *
 * @addtogroup infra_version
 * @{
 */

/**
 * Define the content of a 48-bytes binary version header.
 *
 * The binary version header allows to uniquely identify the binary used. Note:
 *  - a device may include more than one binary, each having its own binary
 *    version header.
 *  - the position of this struct is usually defined in the linker script and
 *    its content is overwritten after the build (in a special post-build script).
 * It therefore doesn't need to be initialized at compile-time (excepted magic
 * and version), yet can be used in the code at runtime.
 *
 * The binary version header is usually localized at the beginning of
 * the payload, but in the case of e.g. a bootloader, it can also be stored
 * at the end of the payload, or even anywhere within the payload.
 *
 * Major, Minor, Patch are the following the usual definition, e.g. 1.0.0
 */
struct version_header {
    /** Always equal to $B!N */
    uint8_t magic[4];

    /** Header format version */
    uint8_t version;
    uint8_t major;
    uint8_t minor;
    uint8_t patch;

    /**
     * Human-friendly version string, free format (not NULL terminated)
     * Advised format is: PPPPXXXXXX-YYWWTBBBB
     *  - PPPP  : product code, e.g ATP1
     *  - XXXXXX: binary info. Usually contains information such as the
     *    binary type (bootloader, application), build variant (unit tests,
     *    debug, release), release/branch name
     *  - YY    : year last 2 digits
     *  - WW    : work week number
     *  - T     : build type, e.g. [W]eekly, [L]atest, [R]elease, [P]roduction,
     *    [F]actory, [C]ustom
     *  - BBBB  : build number, left padded with zeros
     * Examples:
     *  - ATP1BOOT01-1503W0234
     *  - CLRKAPP123-1502R0013
     */
    char version_string[20];

    /**
     * Micro-SHA1 (first 4 bytes of the SHA1) of the binary payload excluding
     * this header. It allows to uniquely identify the exact binary used.
     * In the case the header is located in the middle of the payload, the
     * SHA1 has to be computed from two disjoint buffers. */
    uint8_t hash[4];

    /** Position of the payload start relative to the address of this structure */
    int32_t offset;

    /** Filled with zeros, can be eventually used for 64 bits support */
    uint8_t reserved_1[4];

    /** Size of the payload in bytes, including this header */
    uint32_t size;

    /** Filled with zeros, can be eventually used for 64 bits support */
    uint8_t reserved_2[4];
} __packed;

/** The global version header struct */
extern struct version_header version_header;

#endif /* __VERSION_H__ */
