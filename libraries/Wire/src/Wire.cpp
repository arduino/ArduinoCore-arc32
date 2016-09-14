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
#include <i2c.h>
#include <string.h>
}

#include "Wire.h"
#include "variant.h"

TwoWire::TwoWire(I2C_CONTROLLER _controller_id)
    : rxBufferIndex(0), rxBufferLength(0), init_status(-1),
      controller_id(_controller_id)
{
    // Empty
}

void TwoWire::begin(void)
{
    init_status = i2c_openadapter(controller_id);
}

void TwoWire::begin(int i2c_speed)
{
    init_status = i2c_openadapter_speed(controller_id, i2c_speed);
}

void TwoWire::setClock(long speed)
{
    if (speed == 400000L) {
        init_status = i2c_openadapter_speed(controller_id, I2C_SPEED_FAST);
    } else if (speed == 100000L) {
        init_status = i2c_openadapter_speed(controller_id, I2C_SPEED_SLOW);
    } else if (speed == I2C_SPEED_FAST) {
        init_status = i2c_openadapter_speed(controller_id, I2C_SPEED_FAST);
    } else {
        init_status = i2c_openadapter(controller_id);
    }
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity,
                             uint8_t sendStop)
{
    int ret;
    if (quantity > BUFFER_LENGTH)
        quantity = BUFFER_LENGTH;

    /* Set slave address via ioctl  */
    i2c_setslave(controller_id, address);
    ret = i2c_readbytes(controller_id, rxBuffer, quantity, !sendStop);
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
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t) true);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop)
{
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

void TwoWire::beginTransmission(uint8_t address)
{
    if (init_status < 0)
        return;
    // set slave address
    i2c_setslave(controller_id, address);
    // reset transmit buffer
    txBufferLength = 0;
}

void TwoWire::beginTransmission(int address)
{
    beginTransmission((uint8_t)address);
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
        err =
            i2c_writebytes(controller_id, txBuffer, txBufferLength, !sendStop);
    } else {
        uint8_t temp = 0;
        // Workaround: I2C bus scan is currently implemented by reading,
        // so set the read length to 0 to inform the lower I2C driver that we are doing bus scan
        err = i2c_readbytes(controller_id, &temp, 0, 0);
    }
    // empty buffer
    txBufferLength = 0;
    if (err < 0) {
        return -err;
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

TwoWire Wire = TwoWire(I2C_SENSING_0);
