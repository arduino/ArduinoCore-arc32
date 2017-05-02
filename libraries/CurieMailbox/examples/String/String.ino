/*
 * CurieMailbox: String example
 *
 * Requires sample Zephyr application CurieMailbox_String running on x86 core
 * (Get it from https://github.com/01org/CODK-M-X86-Samples)
 *
 * This example demonstrates sending a short (< 16 chars) string from the x86
 * core to the ARC core. The sketch below enables mailbox channnel 0 for
 * receiving messages, and for each received message prints out the channel
 * payload as a string of ASCII characters.
 *
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

#include "CurieMailbox.h"

void setup (void) {
    Serial.begin(9600);

    /* Enable the mailbox */
    CurieMailbox.begin();

    /* Enable all channels for receiving messages */
    for (int i = 0; i < CurieMailbox.numChannels; ++i) {
        CurieMailbox.enableReceive(i);
    }
}

void printMessageAsString (CurieMailboxMsg msg)
{
    char *p = (char *)msg.data;
    Serial.print("Received message from channel " + String(msg.channel) + ": ");
    Serial.println(String(p));
}

void loop (void) {
    CurieMailboxMsg inMsg;  /* This will store the received message */

    while (CurieMailbox.available() > 0) {
        inMsg = CurieMailbox.get();
        printMessageAsString(inMsg);
    }
}

/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
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
 */
