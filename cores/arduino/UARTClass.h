/*
  Copyright (c) 2011 Arduino.  All right reserved.

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
*/

#ifndef _UART_CLASS_
#define _UART_CLASS_

#include "HardwareSerial.h"
#include "RingBuffer.h"

#include <board.h>
#include <uart.h>

#define SERIAL_5N1      LCR_CS5 | LCR_PDIS | LCR_1_STB
#define SERIAL_6N1      LCR_CS6 | LCR_PDIS | LCR_1_STB
#define SERIAL_7N1      LCR_CS7 | LCR_PDIS | LCR_1_STB
#define SERIAL_8N1      LCR_CS8 | LCR_PDIS | LCR_1_STB
#define SERIAL_5N2      LCR_CS5 | LCR_PDIS | LCR_2_STB
#define SERIAL_6N2      LCR_CS6 | LCR_PDIS | LCR_2_STB
#define SERIAL_7N2      LCR_CS7 | LCR_PDIS | LCR_2_STB
#define SERIAL_8N2      LCR_CS8 | LCR_PDIS | LCR_2_STB
#define SERIAL_5E1      LCR_CS5 | LCR_PEN  | LCR_EPS | LCR_1_STB
#define SERIAL_6E1      LCR_CS6 | LCR_PEN  | LCR_EPS | LCR_1_STB
#define SERIAL_7E1      LCR_CS7 | LCR_PEN  | LCR_EPS | LCR_1_STB
#define SERIAL_8E1      LCR_CS8 | LCR_PEN  | LCR_EPS | LCR_1_STB
#define SERIAL_5E2      LCR_CS5 | LCR_PEN  | LCR_EPS | LCR_2_STB
#define SERIAL_6E2      LCR_CS6 | LCR_PEN  | LCR_EPS | LCR_2_STB
#define SERIAL_7E2      LCR_CS7 | LCR_PEN  | LCR_EPS | LCR_2_STB
#define SERIAL_8E2      LCR_CS8 | LCR_PEN  | LCR_EPS | LCR_2_STB
#define SERIAL_5O1      LCR_CS5 | LCR_PEN  | LCR_1_STB
#define SERIAL_6O1      LCR_CS6 | LCR_PEN  | LCR_1_STB
#define SERIAL_7O1      LCR_CS7 | LCR_PEN  | LCR_1_STB
#define SERIAL_8O1      LCR_CS8 | LCR_PEN  | LCR_1_STB
#define SERIAL_5O2      LCR_CS5 | LCR_PEN  | LCR_2_STB
#define SERIAL_6O2      LCR_CS6 | LCR_PEN  | LCR_2_STB
#define SERIAL_7O2      LCR_CS7 | LCR_PEN  | LCR_2_STB
#define SERIAL_8O2      LCR_CS8 | LCR_PEN  | LCR_2_STB

class UARTClass : public HardwareSerial
{
  public:
    //UARTClass(Uart* pUart, IRQn_Type dwIrq, uint32_t dwId, RingBuffer* pRx_buffer, RingBuffer* pTx_buffer);
    UARTClass(uart_init_info *info, RingBuffer *pRx_buffer, RingBuffer *pTx_buffer );

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

    void IrqHandler(void);

    operator bool() { return true; }; // UART always active

  protected:
    void init(const uint32_t dwBaudRate, const uint8_t config);

    RingBuffer *_rx_buffer;
    RingBuffer *_tx_buffer;

    uart_init_info *info;
    //Uart* _pUart;
    //IRQn_Type _dwIrq;
    uint32_t _dwId;
    uint32_t opened;

};

#endif // _UART_CLASS_
