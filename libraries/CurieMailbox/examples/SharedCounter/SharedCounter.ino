/*
 * CurieMailbox: Shared Counter example
 *
 * Requires sample Zephyr application CurieMailbox_SharedCounter running on x86
 * core (get it from https://github.com/01org/CODK-M-X86-Samples)
 *
 * This example demonstrates sharing a single integer value between the x86
 * and ARC core. This sketch (ARC core) starts by sending a value of 0 using
 * CurieMailbox.put(). This is received by the application running on the x86
 * core, which increments the value by 1 and sends it back via the mailbox.
 *
 * This sketch will read the reply, and again increment it by one and send
 * it back to the x86 core through the mailbox. In this way the count value
 * will increase indefinitely, with each core performing alternate increments
 * and using the mailbox to pass the value in between.
 *
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

#include "CurieMailbox.h"

uint32_t count = 0;

int sendChannel = 0;     /* We'll send mailbox messages on this channel */
int receiveChannel = 1;  /* And receive them on this channel */

void setup (void) {
    Serial.begin(9600);

    /* Enable the mailbox */
    CurieMailbox.begin();

    /* Enable channel for receiving messages */
    CurieMailbox.enableReceive(receiveChannel);
}

void loop (void) {
    CurieMailboxMsg outMsg; /* This will store the message we are sending */
    CurieMailboxMsg inMsg;  /* This will store the received message */

    delay(1000);

    outMsg.id = 0;                /* ID can be whatever you like */
    outMsg.data[0] = count;       /* Data is an array of 4 uint32_t types */
    outMsg.channel = sendChannel; /* Sending this message to sendChannel */

    CurieMailbox.put(outMsg);     /* Send the message */

    /* Wait for the response */
    while (CurieMailbox.available() == 0);

    /* Read the response */
    inMsg = CurieMailbox.get();

    Serial.print("Sent '" + String(outMsg.data[0]) + "' on channel ");
    Serial.print(String(outMsg.channel) + ", got reply '" + String(inMsg.data[0]));
    Serial.println("' on channel " + String(inMsg.channel));

    /* Update our count value with the value received from mailbox */
    count = inMsg.data[0] + 1;
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
