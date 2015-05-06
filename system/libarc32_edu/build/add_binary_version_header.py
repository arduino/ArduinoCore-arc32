#!/usr/bin/python
# -*- coding: utf-8 -*-

# INTEL CONFIDENTIAL Copyright 2015 Intel Corporation All Rights Reserved.
#
# The source code contained or described herein and all documents related to
# the source code ("Material") are owned by Intel Corporation or its suppliers
# or licensors.
# Title to the Material remains with Intel Corporation or its suppliers and
# licensors.
# The Material contains trade secrets and proprietary and confidential information
# of Intel or its suppliers and licensors. The Material is protected by worldwide
# copyright and trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified, published,
# uploaded, posted, transmitted, distributed, or disclosed in any way without
# Intel's prior express written permission.
#
# No license under any patent, copyright, trade secret or other intellectual
# property right is granted to or conferred upon you by disclosure or delivery
# of the Materials, either expressly, by implication, inducement, estoppel or
# otherwise.
#
# Any license under such intellectual property rights must be express and
# approved by Intel in writing

import ctypes
import sys
import argparse
import hashlib

class BinaryVersionHeader(ctypes.Structure):
    """Binary version header with $B!N magic, see declaration in
       infra/version.h """
    _pack_ = 1
    _fields_ = [
        # Always equal to $B!N
        ("magic", ctypes.c_char * 4),

        # Header format version
        ("version", ctypes.c_ubyte),
        ("major", ctypes.c_ubyte),
        ("minor", ctypes.c_ubyte),
        ("patch", ctypes.c_ubyte),

        # Human-friendly version string, free format (not NULL terminated)
        # Advised format is: PPPPXXXXXX-YYWWTBBBB
        #  - PPPP  : product code, e.g ATP1
        #  - XXXXXX: binary info. Usually contains information such as the
        #    binary type (bootloader, application), build variant (unit tests,
        #    debug, release), release/branch name
        #  - YY    : year last 2 digits
        #  - WW    : work week number
        #  - T     : build type, e.g. [Engineering], [W]eekly, [L]atest,
        #    [R]elease, [P]roduction, [F]actory, [C]ustom
        #  - BBBB  : build number, left padded with zeros
        # Examples:
        #  - ATP1BOOT01-1503W0234
        #  - CLRKAPP123-1502R0013
        ("version_string", ctypes.c_ubyte * 20),

        # Micro-SHA1 (first 4 bytes of the SHA1) of the binary payload excluding
        # this header. It allows to uniquely identify the exact binary used.
        ("hash", ctypes.c_ubyte * 4),

        # Position of the payload relative to the address of this structure
        ("offset", ctypes.c_int32),
        ("reserved_1", ctypes.c_ubyte * 4),

        # Size of the payload, i.e. the full binary on which the hash was
        # computed (excluding this header). The beginning of the payload
        # is assumed to start right after the last byte of this structure.
        ("size", ctypes.c_uint32),
        ("reserved_2", ctypes.c_ubyte * 4)
    ]

MAGIC = "$B!N"

def main(argv):
    parser = argparse.ArgumentParser(description="""Overwrite the binary
        version header in the passed binary file.""")
    parser.add_argument('--major', dest='major', type=int,
        help='major version number', required=True)
    parser.add_argument('--minor', dest='minor', type=int,
        help='minor version number', required=True)
    parser.add_argument('--patch', dest='patch', type=int,
        help='patch version number', required=True)
    parser.add_argument('--version_string', dest='version_string',
        help='human friendly version string, free format', required=True)
    parser.add_argument('--header_position_hint', dest='header_position_hint',
        type=str, help='one of start, end, unknown. Default to unknown.',
        default="unknown")
    parser.add_argument('input_file',
        help='the binary file to modify in place')
    args = parser.parse_args()

    arr = bytearray(open(args.input_file, "rb").read())

    if args.header_position_hint == 'start':
        header_pos = 0
    elif args.header_position_hint == 'end':
        header_pos = len(arr)-48
    elif args.header_position_hint == 'unknown':
        header_pos = arr.find(MAGIC)
        if header_pos == -1:
            raise Exception("Cannot find the magic string %s in the passed binary." % MAGIC)
        else:
            print "Found header magic at offset %x" % header_pos
    else:
        raise Exception("Invalid value for header_position_hint argument.")

    assert arr[header_pos:header_pos+4] == MAGIC
    assert arr[header_pos+4] == 1

    # Compute the hash. The header can be anywhere within the binary so we feed
    # the generator in 2 passes
    m = hashlib.sha1()
    m.update(arr[0:header_pos])
    m.update(arr[header_pos+48:])
    digest = bytearray(m.digest())

    # Create and initialize our header struct
    bh = BinaryVersionHeader(MAGIC, 0x01)
    assert len(bytearray(bh)) == 48
    bh.major = args.major
    bh.minor = args.minor
    bh.patch = args.patch
    for i in range(0, 4):
        bh.hash[i] = digest[i]
    if len(args.version_string) < 20:
        args.version_string += '\0' * (20-len(args.version_string))
    vs = bytearray(args.version_string)
    for i in range(0, 20):
        bh.version_string[i] = vs[i]
    bh.offset = -header_pos
    bh.size  = len(arr)

    # Over-write the header content
    arr[header_pos:header_pos+48] = bytearray(bh)

    # And save it
    out_file = open(args.input_file, "wb")
    out_file.write(arr)

if __name__ == "__main__":
    main(sys.argv)
