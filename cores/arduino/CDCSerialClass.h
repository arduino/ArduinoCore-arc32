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

#ifndef _CFWSERIAL_CLASS_
#define _CFWSERIAL_CLASS_

#include "HardwareSerial.h"
#include "platform.h"
#include "wiring.h"

#include <board.h>
#include <uart.h>

class CDCSerialClass : public HardwareSerial
{
  public:
    CDCSerialClass(uart_init_info *info);

    void setSharedData(struct cdc_acm_shared_data *cdc_acm_shared_data);

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

    operator bool() {
	/* In case bool() is called in a very tight while loop, give LMT space
	 * to set the variable */
	delay(1);
        return (_shared_data && _shared_data->host_open);
    };

  protected:
    void init(const uint32_t dwBaudRate, const uint8_t config);

    struct cdc_acm_shared_data *_shared_data;
    struct cdc_ring_buffer *_rx_buffer;
    struct cdc_ring_buffer *_tx_buffer;

    uart_init_info *info;
    uint32_t _writeDelayUsec;
    uint32_t _dwId;
};

#endif // _CDCSerial_CLASS_
