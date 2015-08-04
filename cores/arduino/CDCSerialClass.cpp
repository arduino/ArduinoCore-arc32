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

  CDC-ACM class for Intel EDU - Aug 2015 <dave.hunt@emutex.com>

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
#include "services/cdc_serial_service.h"

extern void CDCSerial_Handler(void);
extern void serialEventRun1(void) __attribute__((weak));
extern void serialEvent1(void) __attribute__((weak));

void CDCSerial_getByte(uint8_t uc_data, void *arg)
{
  ((CDCSerialClass *)arg)->getByte(uc_data);
}

void CDCSerial_bytes_sent(uint32_t num, void *arg)
{
  ((CDCSerialClass *)arg)->bytes_sent(num);
}

// Constructors ////////////////////////////////////////////////////////////////

CDCSerialClass::CDCSerialClass( uart_init_info *info, RingBuffer *pRx_buffer, RingBuffer *pTx_buffer )
{
	this->info = info;
	this->_rx_buffer = pRx_buffer;
	this->_tx_buffer = pTx_buffer;
	cdc_register_byte_cb(CDCSerial_getByte, this);
	cdc_register_sent_cb(CDCSerial_bytes_sent, this);
}

// Public Methods //////////////////////////////////////////////////////////////

void CDCSerialClass::begin(const uint32_t dwBaudRate)
{
  begin(dwBaudRate, (uint8_t)SERIAL_8N1 );
}

void CDCSerialClass::begin(const uint32_t dwBaudRate, const uint8_t config)
{
  init(dwBaudRate, config );
  opened = true;
}


void CDCSerialClass::init(const uint32_t dwBaudRate, const uint8_t modeReg)
{
	// Make sure both ring buffers are initialized back to empty.
	_rx_buffer->_iHead = _rx_buffer->_iTail = 0;
	_tx_buffer->_iHead = _tx_buffer->_iTail = 0;
	in_flight = false;
}

void CDCSerialClass::end( void )
{
  flush();
  opened = false;
  _rx_buffer->_iHead = _rx_buffer->_iTail;
}

void CDCSerialClass::setInterruptPriority(uint32_t priority)
{
}

uint32_t CDCSerialClass::getInterruptPriority()
{
  return 0;
}

int CDCSerialClass::available( void )
{
  return (int)(SERIAL_BUFFER_SIZE + _rx_buffer->_iHead - _rx_buffer->_iTail) % SERIAL_BUFFER_SIZE;
}

int CDCSerialClass::availableForWrite(void)
{
  if (!opened)
    return(0);
  int head = _tx_buffer->_iHead;
  int tail = _tx_buffer->_iTail;
  if (head >= tail) return SERIAL_BUFFER_SIZE - 1 - head + tail;
  return tail - head - 1;
}

int CDCSerialClass::peek(void)
{
  if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
    return -1;

  return _rx_buffer->_aucBuffer[_rx_buffer->_iTail];
}

int CDCSerialClass::read( void )
{
  if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
    return -1;

  uint8_t uc = _rx_buffer->_aucBuffer[_rx_buffer->_iTail];
  _rx_buffer->_iTail = (unsigned int)(_rx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE;
  return uc;
}

static char chars[SERIAL_BUFFER_SIZE];

void CDCSerialClass::flush( void )
{
	int i=0;
	while (_tx_buffer->_iTail != _tx_buffer->_iHead) {
		chars[i++] = _tx_buffer->_aucBuffer[_tx_buffer->_iTail];
		_tx_buffer->_iTail = (unsigned int)(_tx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE;
	}
	if (i>0)
	{
		chars[i] = 0;
		cdc_serial_service_send((char *)chars, i, (void*)"Flush");
	}
}


size_t CDCSerialClass::write( const uint8_t uc_data )
{
    int l;
	if (!opened)
		return(0);

	noInterrupts();
	if (in_flight)
	{
		l = (_tx_buffer->_iHead + 1) % SERIAL_BUFFER_SIZE;
		_tx_buffer->_aucBuffer[_tx_buffer->_iHead] = uc_data;
		_tx_buffer->_iHead = l;
		interrupts();
		return 1;
	}

	in_flight = true;
	interrupts();
	cdc_serial_service_send((char *)&uc_data, 1, (void*)"Flush");
	return 1;

}

void CDCSerialClass::getByte(uint8_t uc_data)
{
	_rx_buffer->store_char(uc_data);
	// While we're at it, we'll flush the output.
	flush();
}

void CDCSerialClass::bytes_sent(uint32_t len)
{
	in_flight = false;
	flush();
}
