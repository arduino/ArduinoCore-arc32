/*
 * TwoWire.h - TWI/I2C library for Linux Userspace
 * Copyright (c) 2013 Parav https://github.com/meanbot.
 * All rights reserved.
 */
#ifndef TwoWire_h
#define TwoWire_h

#include "Stream.h"
#include "variant.h"

#define BUFFER_LENGTH 32

class TwoWire : public Stream {
public:
	TwoWire(void);
	void begin();
	void beginTransmission(uint8_t);
	void beginTransmission(int);
	uint8_t endTransmission(void);
	uint8_t endTransmission(uint8_t);
	uint8_t requestFrom(uint8_t, uint8_t);
	uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
	uint8_t requestFrom(int, int);
	uint8_t requestFrom(int, int, int);
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *, size_t);
	virtual int available(void);
	virtual int read(void);
	virtual int peek(void);
	virtual void flush(void);

	inline size_t write(unsigned long n) { return write((uint8_t)n); }
	inline size_t write(long n) { return write((uint8_t)n); }
	inline size_t write(unsigned int n) { return write((uint8_t)n); }
	inline size_t write(int n) { return write((uint8_t)n); }
	using Print::write;

private:
	// RX Buffer
	uint8_t rxBuffer[BUFFER_LENGTH];
	uint8_t rxBufferIndex;
	uint8_t rxBufferLength;

	// TX Buffer
	uint8_t txAddress;
	uint8_t txBuffer[BUFFER_LENGTH];
	uint8_t txBufferLength;

	int init_status;
};

#if WIRE_INTERFACES_COUNT > 0
extern TwoWire Wire;
#endif

#endif

