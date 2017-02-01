/*
 * freeMemory.ino: This sketch demonstrates the use of the freeMemory()
 * function to measure the amount of free memory available in the system,
 * before and after using 'malloc' to allocate some memory.
 *
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

#include <MemoryFree.h>

void setup () {
    Serial.begin(9600);
    while(!Serial);
}

void loop() {
    char *p;

    Serial.println("Free memory: " + String(freeMemory()));
    Serial.println("Allocating 24 bytes ...");

    p = (char *)malloc(24);
    Serial.println("Free memory: " + String(freeMemory()));

    Serial.println("Freeing 24 bytes ...");
    free(p); 
    Serial.println("Free memory: " + String(freeMemory()));

    delay(2000);
}

/*
 * Copyright (c) 2017 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 */
