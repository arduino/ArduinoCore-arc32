/*
SoftwareSerial.h

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

Implementation of SoftwareSerial Library for Arduino 101
*/

#ifndef SoftwareSerial_h
#define SoftwareSerial_h

#include <inttypes.h>
#include <Stream.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define _SS_MAX_RX_BUFF 64 // RX buffer size

class SoftwareSerial : public Stream
{
private:
  // per object data
  uint32_t _receivePin;
  uint32_t _receiveBitMask;
  volatile uint32_t *_receivePortRegister;
  uint32_t _transmitBitMask;
  //volatile uint32_t *_transmitPortRegister;
  volatile uint32_t *_pcint_maskreg;
  uint32_t _pcint_maskvalue;

  // Expressed as 4-cycle delays (must never be 0!)
  uint32_t _rx_delay_centering;
  uint32_t _rx_delay_intrabit;
  uint32_t _rx_delay_stopbit;
  uint32_t _tx_delay;

  uint32_t _buffer_overflow:1;
  uint32_t _inverse_logic:1;

  // static data
  static char _receive_buffer[_SS_MAX_RX_BUFF]; 
  static volatile uint8_t _receive_buffer_tail;
  static volatile uint8_t _receive_buffer_head;
  static SoftwareSerial *active_object;

  // private methods
  static void recv() __attribute__((__always_inline__));
  uint32_t rx_pin_read();
  void tx_pin_write(uint32_t pin_state) __attribute__((__always_inline__));
  void setTX(uint8_t transmitPin);
  void setRX(uint8_t receivePin);
  void setRxIntMsk(bool enable) __attribute__((__always_inline__));

  // Return num - sub, or 1 if the result would be < 1
  static uint16_t subtract_cap(uint16_t num, uint16_t sub);

  // private static method for timing
  static inline void tunedDelay(uint16_t delay);

public:
  // public methods
  SoftwareSerial(uint32_t receivePin, uint32_t transmitPin, bool inverse_logic = false);
  ~SoftwareSerial();
  void begin(long speed);
  bool listen();
  void end();
  bool isListening() { return this == active_object; }
  bool stopListening();
  bool overflow() { bool ret = _buffer_overflow; if (ret) _buffer_overflow = false; return ret; }
  int peek();

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual void flush();
  operator bool() { return true; }
  
  using Print::write;

  // public only for easy access by interrupt handlers
  static inline void handle_interrupt() __attribute__((__always_inline__));
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round

#endif
