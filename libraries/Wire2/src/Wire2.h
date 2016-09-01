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
 */

#ifndef TwoWire2_h
#define TwoWire2_h

#include "Stream.h"
#include "variant.h"

#define BUFFER_LENGTH   32

#define I2C_SPEED_SLOW  1
#define I2C_SPEED_FAST  2
#define I2C_SPEED_HS    3

#define I2C_ADDR_7Bit   0
#define I2C_ADDR_10Bit  1

class TwoWire2 : public Stream
{
  public:
    TwoWire2(void);
    void begin();
    void begin(uint8_t, int i2c_speed = I2C_SPEED_FAST,
               int i2c_addr_mode = I2C_ADDR_7Bit);
    void begin(int, int i2c_speed = I2C_SPEED_FAST,
               int i2c_addr_mode = I2C_ADDR_7Bit);    
    void end();
    void setSpeed(uint32_t);
    void setAddressMode(uint32_t);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint32_t, uint8_t, uint8_t);
    uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void (*)(int));
    void onRequest(void (*)(void));

    inline size_t write(unsigned long n)
    {
        return write((uint8_t)n);
    }
    inline size_t write(long n)
    {
        return write((uint8_t)n);
    }
    inline size_t write(unsigned int n)
    {
        return write((uint8_t)n);
    }
    inline size_t write(int n)
    {
        return write((uint8_t)n);
    }
    using Print::write;

  private:
    static uint8_t rxBuffer[];
    static uint8_t rxBufferIndex;
    static uint8_t rxBufferLength;

    static uint8_t txAddress;
    static uint8_t txBuffer[];
    static uint8_t txBufferLength;

    static void (*onRequestUserCallback)(void);
    static void (*onReceiveUserCallback)(int);
    static void onReceiveCallback(int);
    static void onRequestCallback(void);

    int init_status;
};

extern TwoWire2 Wire2;

#endif
