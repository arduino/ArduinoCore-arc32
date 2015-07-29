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

#ifndef _CFWSERIAL_CLASS_
#define _CFWSERIAL_CLASS_

#include "HardwareSerial.h"
#include "RingBuffer.h"

#include <board.h>
#include <uart.h>

class CDCSerialClass : public HardwareSerial
{
  public:
    CDCSerialClass(uart_init_info *info, RingBuffer *pRx_buffer, RingBuffer *pTx_buffer );

    void begin(const uint32_t dwBaudRate);
    void begin(const uint32_t dwBaudRate, const uint8_t config);
    void end(void);
    int available(void);
    int availableForWrite(void);
    int peek(void);
    int read(void);
    void flush(void);
    size_t write(const uint8_t c);
    using Print::write; // pull in write(str) and write(buf, size) from Print
    void setInterruptPriority(uint32_t priority);
    uint32_t getInterruptPriority();

	void getByte(uint8_t uc_data);
	void bytes_sent(uint32_t num);
	void init_cb(uint32_t status);
	bool in_flight;
	int acm_open;
    void IrqHandler(void);

    operator bool() { return true; }; // CDCSerial always active

  protected:
    void init(const uint32_t dwBaudRate, const uint8_t config);

    RingBuffer *_rx_buffer;
    RingBuffer *_tx_buffer;

    uart_init_info *info;
    uint32_t _dwId;
    uint32_t opened;

};

#endif // _CDCSerial_CLASS_
