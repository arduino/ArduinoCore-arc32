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


extern void CDCSerial_Handler(void);
extern void serialEventRun1(void) __attribute__((weak));
extern void serialEvent1(void) __attribute__((weak));

// Constructors ////////////////////////////////////////////////////////////////

CDCSerialClass::CDCSerialClass(uart_init_info *info)
{
    this->info = info;
}

// Public Methods //////////////////////////////////////////////////////////////

void CDCSerialClass::setSharedData(struct cdc_acm_shared_data *cdc_acm_shared_data)
{
    this->_shared_data = cdc_acm_shared_data;
    this->_rx_buffer = cdc_acm_shared_data->rx_buffer;
    this->_tx_buffer = cdc_acm_shared_data->tx_buffer;
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

    _shared_data->device_open = true;
}

void CDCSerialClass::end( void )
{
    _shared_data->device_open = false;
}

int CDCSerialClass::available( void )
{
#define SBS	SERIAL_BUFFER_SIZE

  if (!_shared_data->device_open)
    return (0);
  else
    return (int)(SBS + _rx_buffer->head - _rx_buffer->tail) % SBS;
}

int CDCSerialClass::availableForWrite(void)
{
    if (!_shared_data->device_open || !_shared_data->host_open)
        return(0);

    int head = _tx_buffer->head;
    int tail = _tx_buffer->tail;

    if (head >= tail)
        return SERIAL_BUFFER_SIZE - head + tail - 1;
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
  _rx_buffer->tail = (_rx_buffer->tail + 1) % SERIAL_BUFFER_SIZE;
  return uc;
}

void CDCSerialClass::flush( void )
{
    while (_tx_buffer->tail != _tx_buffer->head) { /* This infinite loop is intentional
						      and requested by design */
	    delayMicroseconds(1);
    }
}

size_t CDCSerialClass::write( const uint8_t uc_data )
{
    uint32_t retries = 1;

    if (!_shared_data->device_open || !_shared_data->host_open)
        return(0);

    do {
        int i = (uint32_t)(_tx_buffer->head + 1) % SERIAL_BUFFER_SIZE;
        // if we should be storing the received character into the location
        // just before the tail (meaning that the head would advance to the
        // current location of the tail), we're about to overflow the buffer
        // and so we don't write the character or advance the head.
        if (i != _tx_buffer->tail) {
            _tx_buffer->data[_tx_buffer->head] = uc_data;
            _tx_buffer->head = i;

	    // Mimick the throughput of a typical UART by throttling the data
	    // flow according to the configured baud rate
	    delayMicroseconds(_writeDelayUsec);
            break;
        }
    } while (retries--);

    return 1;
}
