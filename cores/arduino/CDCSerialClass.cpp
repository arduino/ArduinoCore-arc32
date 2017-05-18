/*
  Copyright (c) 2011 Arduino.  All right reserved.
  Copyright (c) 2015 Intel Corporation.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  CDC-ACM class for Arduino 101 - Aug 2015 <dave.hunt@emutex.com>

*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include "portable.h"
#include "CDCSerialClass.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "variant.h"

#define CDC_MAILBOX_TX_CHANNEL  7
#define CDC_MAILBOX_RX_CHANNEL  6

extern void CDCSerial_Handler(void);
extern void serialEventRun1(void) __attribute__((weak));
extern void serialEvent1(void) __attribute__((weak));

static void cdc_mbox_isr(CurieMailboxMsg msg)
{
    char *rxbuffptr = (char*)msg.data;
    int new_head;
    for(int i = 0; i < MBOX_BYTES; i++)
    {
        if((uint8_t)(*(rxbuffptr+i)) != '\0')
        {
            new_head = (Serial._rx_buffer->head +1) % CDCACM_BUFFER_SIZE;
            if(new_head != Serial._rx_buffer->tail)
            {
                Serial._rx_buffer->data[Serial._rx_buffer->head] = *(rxbuffptr+i);
                Serial._rx_buffer->head = new_head;
            }
        }
        else
        {
            break;
        }
    }
}

// Constructors ////////////////////////////////////////////////////////////////

CDCSerialClass::CDCSerialClass(uart_init_info *info)
{
    this->info = info;
}

// Public Methods //////////////////////////////////////////////////////////////

void CDCSerialClass::setSharedData(struct cdc_acm_shared_data *cdc_acm_shared_data)
{
    _shared_data = cdc_acm_shared_data;
    _rx_buffer = cdc_acm_shared_data->rx_buffer;
    _tx_buffer = cdc_acm_shared_data->tx_buffer;
}

void CDCSerialClass::begin(const uint32_t dwBaudRate)
{
    begin(dwBaudRate, (uint8_t)SERIAL_8N1 );
}

void CDCSerialClass::begin(const uint32_t dwBaudRate, const uint8_t config)
{
    init(dwBaudRate, config );
}


void CDCSerialClass::init(const uint32_t dwBaudRate, const uint8_t modeReg)
{
    /* Set a per-byte write delay approximately equal to the time it would
     * take to clock out a byte on a standard UART at this baud rate */
    _writeDelayUsec = 8000000 / dwBaudRate;

    /* Make sure both ring buffers are initialized back to empty.
     * Empty the Rx buffer but don't touch Tx buffer: it is drained by the
     * LMT one way or another */
    _rx_buffer->tail = _rx_buffer->head;
    
    mailbox_register(CDC_MAILBOX_RX_CHANNEL, cdc_mbox_isr);
    mailbox_enable_receive(CDC_MAILBOX_RX_CHANNEL);
    _shared_data->device_open = true;
}

void CDCSerialClass::end( void )
{
    _shared_data->device_open = false;
}

int CDCSerialClass::available( void )
{
  if (!_shared_data->device_open)
    return (0);
  else
    return (int)(CDCACM_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) % CDCACM_BUFFER_SIZE;
}

int CDCSerialClass::availableForWrite(void)
{
    if (!_shared_data->device_open || !_shared_data->host_open)
        return(0);

    int head = _tx_buffer->head;
    int tail = _tx_buffer->tail;

    if (head >= tail)
        return CDCACM_BUFFER_SIZE - head + tail - 1;
    return tail - head - 1;
}

int CDCSerialClass::peek(void)
{
  if ((!_shared_data->device_open) || ( _rx_buffer->head == _rx_buffer->tail ))
    return -1;

  return _rx_buffer->data[_rx_buffer->tail];
}

int CDCSerialClass::read( void )
{
  if ((!_shared_data->device_open) || ( _rx_buffer->head == _rx_buffer->tail ))
    return -1;

  uint8_t uc = _rx_buffer->data[_rx_buffer->tail];
  _rx_buffer->tail = (_rx_buffer->tail + 1) % CDCACM_BUFFER_SIZE;
  return uc;
}

void CDCSerialClass::flush( void )
{
    while (_tx_buffer->tail != _tx_buffer->head) { /* This infinite loop is intentional
						      and requested by design */
	    delayMicroseconds(1);
    }
}

size_t CDCSerialClass::write(uint8_t uc_data )
{
    CurieMailboxMsg cdcacm_msg;

    if (!_shared_data->device_open || !_shared_data->host_open)
        return(0);

    cdcacm_msg.channel = CDC_MAILBOX_TX_CHANNEL;
    cdcacm_msg.data[0] = uc_data;
    mailbox_write(cdcacm_msg);
    delayMicroseconds(_writeDelayUsec);
    return 1;
}

size_t CDCSerialClass::write(const uint8_t *buffer, size_t size)
{
    CurieMailboxMsg cdcacm_msg;
     cdcacm_msg.channel = CDC_MAILBOX_TX_CHANNEL;
    if (!_shared_data->device_open || !_shared_data->host_open)
        return(0);
    
    int msg_len = size;
    
    for(int i = 0;msg_len > 0; msg_len -= MBOX_BYTES, i += MBOX_BYTES)
    {
        /* Copy data into mailbox message */
        memset(cdcacm_msg.data, 0, MBOX_BYTES);
        if(msg_len >= MBOX_BYTES)
        {
            memcpy(cdcacm_msg.data, buffer+i, MBOX_BYTES);
        }
        else
        {
            memcpy(cdcacm_msg.data, buffer+i, msg_len);
        }
        /* Write to mailbox*/
        mailbox_write(cdcacm_msg);
    }

    // Mimick the throughput of a typical UART by throttling the data
    // flow according to the configured baud rate  
    delayMicroseconds(_writeDelayUsec * size);
    
    return size;
}

size_t CDCSerialClass::write(const char *str)
{
    if (str == NULL) return 0;
    CurieMailboxMsg cdcacm_msg;
     cdcacm_msg.channel = CDC_MAILBOX_TX_CHANNEL;
    if (!_shared_data->device_open || !_shared_data->host_open)
        return(0);
    
    int msg_len = strlen(str);

    return write((const uint8_t *)str, msg_len);
}
