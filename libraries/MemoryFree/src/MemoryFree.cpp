/*
 * MemoryFree.cpp: taken from http://playground.arduino.cc/Code/AvailableMemory,
 * re-written for the Arduino 101 which uses a different malloc implementation.
 *
 * Arduino 101 malloc source:
 * https://github.com/foss-for-synopsys-dwc-arc-processors/glibc
 *
 * mallinfo() struct details:
 * http://man7.org/linux/man-pages/man3/mallinfo.3.html
 *
 * Copyright (c) 2017 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 */

#include <malloc.h>
#include "MemoryFree.h"

extern char __start_heap;
extern char __end_heap;
extern char __stack_size;
extern char __stack_start;

int freeStack() {
    int stack_end;
    int mark;

    stack_end = ((int)&__stack_start) - ((int)&__stack_size);
    return ((int)&mark) - stack_end;
}

int freeHeap (void) {
    int hsize;
    struct mallinfo mi;

    mi = mallinfo();
    hsize = (int)&__end_heap - (int)&__start_heap;
    return (hsize - mi.arena) + mi.fordblks;
}

int freeMemory (void) {
    int heap = freeHeap();
    int stack = freeStack();
    return (stack < 0) ? heap : stack + heap;
}
