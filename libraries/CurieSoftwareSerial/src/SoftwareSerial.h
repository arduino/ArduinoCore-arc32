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

CurieSoftwareSerial library is a work in progress
Rx is only functional up to 57600 bps
Rx does not work for pin 13
*/

#ifndef SoftwareSerial_h
#define SoftwareSerial_h

#include <inttypes.h>
#include <Stream.h>
#include <Arduino.h>
/******************************************************************************
* Definitions
******************************************************************************/

#define _SS_MAX_RX_BUFF 64 // RX buffer size

class SoftwareSerial : public Stream
{
private:
  // per object data
  uint32_t _receivePin;
  uint32_t _transmitPin;
  uint32_t _isSOCGpio;

  // delays
  int _bit_delay;
  int _rx_delay_centering;
  int _rx_delay_init_centering;
  int _rx_delay_init_intrabit;
  int _rx_delay_first_intrabit;
  int _rx_delay_intrabit;
  int _rx_delay_stopbit;
  int _tx_delay;

  uint32_t _buffer_overflow:1;
  bool _inverse_logic = false;

  // static data
  static char *_receive_buffer;
  static volatile uint8_t _receive_buffer_tail;
  static volatile uint8_t _receive_buffer_head;
  static SoftwareSerial *active_object;

  // private methods
  static void recv();
  uint32_t rx_pin_read();
  void tx_pin_write(uint32_t pin_state) __attribute__((__always_inline__));
  void setTX(uint8_t transmitPin);
  void setRX(uint8_t receivePin);

  // Return num - sub, or 1 if the result would be < 1
  static uint16_t subtract_cap(uint16_t num, uint16_t sub);

  // private static method for timing
  static inline void tunedDelay(uint16_t delay);

public:
  // public methods
  SoftwareSerial(uint32_t receivePin, uint32_t transmitPin, bool inverse_logic = false);
  virtual ~SoftwareSerial();
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

};

#endif
