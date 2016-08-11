/*
 * TwoWire2.h - TWI/I2C library for Linux Userspace
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

#include "Wire2.h"
#include "variant.h"

uint8_t TwoWire2::rxBuffer[BUFFER_LENGTH];
uint8_t TwoWire2::rxBufferIndex = 0;
uint8_t TwoWire2::rxBufferLength = 0;

uint8_t TwoWire2::txAddress = 0;
uint8_t TwoWire2::txBuffer[BUFFER_LENGTH];
uint8_t TwoWire2::txBufferLength = 0;

void (*TwoWire2::onReceiveUserCallback)(int);
void (*TwoWire2::onRequestUserCallback)(void);

TwoWire2::TwoWire2(void) : init_status(-1)
{
    // Empty
}

void TwoWire2::begin(void)
{
    int i2c_speed = I2C_SPEED_FAST;
    int i2c_addr_mode = I2C_ADDR_7Bit;
    init_status = soc_i2c_openadapter(0, i2c_speed, i2c_addr_mode);
}

void TwoWire2::begin(uint8_t address, int i2c_speed, int i2c_addr_mode)
{
    if (address != 0) {
        init_status = soc_i2c_openadapter(address, i2c_speed, i2c_addr_mode);
        soc_i2c_slave_set_rx_user_buffer(rxBuffer, (uint8_t)sizeof(rxBuffer));
    } else
        init_status = soc_i2c_openadapter(0, i2c_speed, i2c_addr_mode);
}

void TwoWire2::begin(int address, int i2c_speed, int i2c_addr_mode)
{
    if (address != 0) {
        init_status = soc_i2c_openadapter(address, i2c_speed, i2c_addr_mode);
        soc_i2c_slave_set_rx_user_buffer(rxBuffer, (uint8_t)sizeof(rxBuffer));
    } else
        init_status = soc_i2c_openadapter(0, i2c_speed, i2c_addr_mode);
}

uint8_t TwoWire2::requestFrom(uint8_t address, uint8_t quantity,
                              uint8_t sendStop)
{
    int ret;
    if (quantity > BUFFER_LENGTH)
        quantity = BUFFER_LENGTH;

    /* Set slave address via ioctl  */
    soc_i2c_master_set_slave_address(address);
    ret = soc_i2c_master_readbytes(rxBuffer, quantity, !sendStop);
    if (ret < 0) {
        return 0;
    }
    // set rx buffer iterator vars
    rxBufferIndex = 0;
    rxBufferLength = quantity;

    return quantity;
}

uint8_t TwoWire2::requestFrom(uint8_t address, uint8_t quantity)
{
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t) true);
}

uint8_t TwoWire2::requestFrom(int address, int quantity)
{
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t) true);
}

uint8_t TwoWire2::requestFrom(int address, int quantity, int sendStop)
{
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

void TwoWire2::beginTransmission(uint8_t address)
{
    if (init_status < 0)
        return;
    // set slave address
    soc_i2c_master_set_slave_address(address);
    // reset transmit buffer
    txBufferLength = 0;
}

void TwoWire2::beginTransmission(int address)
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
uint8_t TwoWire2::endTransmission(uint8_t sendStop)
{
    int err;
    // transmit buffer (blocking)
    if (txBufferLength >= 1) {
        err = soc_i2c_master_witebytes(txBuffer, txBufferLength, !sendStop);
    } else {
        uint8_t temp = 0;
        // Workaround: I2C bus scan is currently implemented by reading,
        // so set the read length to 0 to inform the lower I2C driver that we
        // are doing bus scan
        err = soc_i2c_master_readbytes(&temp, 0, 0);
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
uint8_t TwoWire2::endTransmission(void)
{
    return endTransmission(true);
}

size_t TwoWire2::write(uint8_t data)
{
    if (txBufferLength >= BUFFER_LENGTH)
        return 0;
    txBuffer[txBufferLength++] = data;
    return 1;
}

size_t TwoWire2::write(const uint8_t *data, size_t quantity)
{
    for (size_t i = 0; i < quantity; ++i) {
        if (txBufferLength >= BUFFER_LENGTH)
            return i;
        txBuffer[txBufferLength++] = data[i];
    }
    return quantity;
}

int TwoWire2::available(void)
{
    return rxBufferLength - rxBufferIndex;
}

int TwoWire2::read(void)
{
    if (rxBufferIndex < rxBufferLength)
        return rxBuffer[rxBufferIndex++];
    return -1;
}

int TwoWire2::peek(void)
{
    if (rxBufferIndex < rxBufferLength)
        return rxBuffer[rxBufferIndex];
    return -1;
}

void TwoWire2::flush(void)
{
    // Do nothing, use endTransmission(..) to force
    // data transfer.
}

void TwoWire2::onReceiveCallback(int bytes)
{
    if (!onReceiveUserCallback) {
        return;
    }

    if (rxBufferIndex < rxBufferLength) {
        return;
    }

    rxBufferIndex = 0;
    rxBufferLength = bytes;

    onReceiveUserCallback(bytes);
}

void TwoWire2::onRequestCallback(void)
{
    if (!onRequestUserCallback) {
        return;
    }

    txBufferLength = 0;

    onRequestUserCallback();

    if (txBufferLength >= 1) {
        soc_i2c_slave_set_tx_user_buffer(txBuffer, txBufferLength);
    }
}

void TwoWire2::onReceive(void (*function)(int))
{
    onReceiveUserCallback = function;
    soc_i2c_slave_set_rx_user_callback(onReceiveCallback);
}

void TwoWire2::onRequest(void (*function)(void))
{
    onRequestUserCallback = function;
    soc_i2c_slave_set_tx_user_callback(onRequestCallback);
}

TwoWire2 Wire2 = TwoWire2();
