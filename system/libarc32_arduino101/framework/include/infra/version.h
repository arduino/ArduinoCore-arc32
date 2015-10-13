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

#ifndef __VERSION_H__
#define __VERSION_H__

#include <stdint.h>

/**
 * @defgroup infra_version Binary Version Header
 * @ingroup infra
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
} __attribute__((__packed__));

/** The global version header struct */
extern const struct version_header version_header;

/** @} */

#endif /* __VERSION_H__ */
