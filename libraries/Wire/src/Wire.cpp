/*
 * TwoWire.h - TWI/I2C library for Linux Userspace
 * Copyright (c) 2013 Parav https://github.com/meanbot.
 * All rights reserved.
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Modifications to support Intel Arduino 101
 * Copyright (C) 2015 Intel Corporation
 */

extern "C" {
#include <string.h>
#include <i2c.h>
}

#include "Wire.h"
#include "variant.h"


TwoWire::TwoWire(void) : rxBufferIndex(0), rxBufferLength(0),
			 txBufferLength(0), init_status(-1)
{
	// Empty
}

void TwoWire::begin(void)
{
	init_status = i2c_openadapter();
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
	int ret;
	if (quantity > BUFFER_LENGTH)
		quantity = BUFFER_LENGTH;

	/* Set slave address via ioctl  */
	i2c_setslave(address);
	ret = i2c_readbytes(rxBuffer, quantity, !sendStop);
	if (ret < 0) {
		return 0;
	}
	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = quantity;

	return quantity;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity)
{
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop)
{
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) sendStop);
}

void TwoWire::beginTransmission(uint8_t address)
{
	if (init_status < 0)
		return;
	// set slave address
	i2c_setslave(address);
	// reset transmit buffer
	txBufferLength = 0;
}

void TwoWire::beginTransmission(int address)
{
	beginTransmission((uint8_t) address);
}

//
//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling endTransmission(false) allows a sketch to
//	perform a repeated start.
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to endTransmission(true) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
//
uint8_t TwoWire::endTransmission(uint8_t sendStop)
{
	int err;
		// transmit buffer (blocking)
		if (txBufferLength >= 1) {
			err = i2c_writebytes(txBuffer, txBufferLength, !sendStop);
		} else {
            //Workaround: I2C bus scan is currently implemented by sending an extra byte of value 0
            txBuffer[0] = 0;
            err = i2c_writebytes(txBuffer, 1, !sendStop);
		}
		// empty buffer
		txBufferLength = 0;
		if (err < 0) {
			return 2; // NACK on transmit of address
			/* NOTE: This implementation currently does not distinguish
			 * between NACK on transmit of adddress or data, or other errors
			 */
		}
		return 0; // success
}

//	This provides backwards compatibility with the original
//	definition, and expected behaviour, of endTransmission
//
uint8_t TwoWire::endTransmission(void)
{
	return endTransmission(true);
}

size_t TwoWire::write(uint8_t data)
{
	if (txBufferLength >= BUFFER_LENGTH)
		return 0;
	txBuffer[txBufferLength++] = data;
	return 1;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
	for (size_t i = 0; i < quantity; ++i) {
		if (txBufferLength >= BUFFER_LENGTH)
			return i;
		txBuffer[txBufferLength++] = data[i];
	}
	return quantity;
}

int TwoWire::available(void)
{
	return rxBufferLength - rxBufferIndex;
}

int TwoWire::read(void)
{
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex++];
	return -1;
}

int TwoWire::peek(void)
{
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex];
	return -1;
}

void TwoWire::flush(void)
{
	// Do nothing, use endTransmission(..) to force
	// data transfer.
}

// Preinstantiate Objects //////////////////////////////////////////////////////

TwoWire Wire = TwoWire();
