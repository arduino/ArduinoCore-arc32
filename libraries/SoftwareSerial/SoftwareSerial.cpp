/*
SoftwareSerial.cpp

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

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 11
#define _DEBUG_PIN2 13
// 
// Includes
// 
#include <Arduino.h>
#include <SoftwareSerial.h>

//
// Statics
//
SoftwareSerial *SoftwareSerial::active_object = 0;
char SoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF]; 
volatile uint8_t SoftwareSerial::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSerial::_receive_buffer_head = 0;


//
// Globals
//
uint8_t txPin;
uint8_t rxPin;
uint16_t bitDelay;
uint16_t rxIntraBitDelay;
int rxCenteringDelay;
int initRxCenteringDelay;
bool firstStartBit = true;
bool bufferOverflow = true;

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
inline void DebugPulse(uint8_t pin, uint8_t count)
{
#if _DEBUG
  volatile uint32_t *pport = portOutputRegister(digitalPinToPort(pin));

  uint32_t val = *pport;
  while (count--)
  {
    *pport = val | digitalPinToBitMask(pin);
    *pport = val;
  }
#endif
}

//
// Private methods
//

/* static */ 
inline void SoftwareSerial::tunedDelay(uint16_t delay) { 
  //_delay_loop_2(delay);
  delayMicroseconds(delay);
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another 
bool SoftwareSerial::listen()
{
  if (active_object != this)
  {
    if (active_object)
      active_object->stopListening();

    bufferOverflow = false;
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;

    setRxIntMsk(true);
    return true;
  }

  return false;
}

// Stop listening. Returns true if we were actually listening.
bool SoftwareSerial::stopListening()
{
  if (active_object == this)
  {
    setRxIntMsk(false);
    active_object = NULL;
    return true;
  }
  return false;
}

//
// The receive routine called by the interrupt handler
//
void SoftwareSerial::recv()
{
  noInterrupts();
  uint8_t d = 0;
  /**
  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (_inverse_logic ? rx_pin_read() : !rx_pin_read())
  {
    // Disable further interrupts during reception, this prevents
    // triggering another interrupt directly after we return, which can
    // cause problems at higher baudrates.
    setRxIntMsk(false);

    // Wait approximately 1/2 of a bit width to "center" the sample
    tunedDelay(_rx_delay_centering);
    DebugPulse(_DEBUG_PIN2, 1);

    // Read each of the 8 bits
    for (uint8_t i=8; i > 0; --i)
    {
      tunedDelay(_rx_delay_intrabit);
      d >>= 1;
      DebugPulse(_DEBUG_PIN2, 1);
      if (rx_pin_read())
        d |= 0x80;
    }

    if (_inverse_logic)
      d = ~d;

    // if buffer full, set the overflow flag and return
    uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    if (next != _receive_buffer_head)
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = next;
    } 
    else 
    {
      DebugPulse(_DEBUG_PIN1, 1);
      _buffer_overflow = true;
    }

    // skip the stop bit
    tunedDelay(_rx_delay_stopbit);
    DebugPulse(_DEBUG_PIN1, 1);

    // Re-enable interrupts when we're sure to be inside the stop bit
    setRxIntMsk(true);

  }
  **/
  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if(!digitalRead(rxPin))
  {
    digitalWrite(13, HIGH);
    // The very first start bit the sketch receives takes about 5us longer
    if(firstStartBit)
    {
      tunedDelay(initRxCenteringDelay);
      firstStartBit = false;
    }
    else
    {
      tunedDelay(rxCenteringDelay);
    }

    for (uint8_t i=8; i > 0; --i)
    {
      tunedDelay(rxIntraBitDelay);
      d >>= 1;
      if (digitalRead(rxPin))  //rx_pin_read())
        d |= 0x80;
    }
    
    /**
    Serial.println("recv");
    Serial.print("int: ");
    Serial.println(d);
    Serial.print("char: ");
    Serial.println((char)d);
    Serial.print("Binary : ");
    Serial.println(d, BIN);
    **/
    
    //if (_inverse_logic)
    //  d = ~d;

    uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    if (next != _receive_buffer_head)
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = next;
    } 
    else 
    {
      //DebugPulse(_DEBUG_PIN1, 1);
      bufferOverflow = true;
    }

    // skip the stop bit
    tunedDelay(bitDelay * 3 / 4);
    //DebugPulse(_DEBUG_PIN1, 1);
  }
  interrupts();
}

uint32_t SoftwareSerial::rx_pin_read()
{
  return digitalRead(rxPin);
}

//
// Interrupt handling
//

/* static */
inline void SoftwareSerial::handle_interrupt()
{
  if (active_object)
  {
    active_object->recv();
  }
}

//
// Constructor
//
SoftwareSerial::SoftwareSerial(uint32_t receivePin, uint32_t transmitPin, bool inverse_logic /* = false */) : 
  _rx_delay_centering(0),
  _rx_delay_intrabit(0),
  _rx_delay_stopbit(0),
  _tx_delay(0),
  _buffer_overflow(false),
  _inverse_logic(inverse_logic)
{
  setTX(transmitPin);
  txPin = transmitPin;
  setRX(receivePin);
  rxPin = receivePin;
}

//
// Destructor
//
SoftwareSerial::~SoftwareSerial()
{
  end();
}

void SoftwareSerial::setTX(uint8_t tx)
{
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output hihg. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.
  digitalWrite(tx, _inverse_logic ? LOW : HIGH);
  pinMode(tx, OUTPUT);
}

void SoftwareSerial::setRX(uint8_t rx)
{
  pinMode(rx, INPUT);
  if (!_inverse_logic)
    pinMode(rx, INPUT_PULLUP);
    //digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin = rx;
  //_receiveBitMask = digitalPinToBitMask(rx);
  //uint32_t port = digitalPinToPort(rx);
  //_receivePortRegister = portInputRegister(port);
}

uint16_t SoftwareSerial::subtract_cap(uint16_t num, uint16_t sub) {
  if (num > sub)
    return num - sub;
  else
    return 1;
}

//
// Public methods
//

void SoftwareSerial::begin(long speed)
{
  _rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;

  //pre-calculate delays
  bitDelay = (1000000/speed + 0.5);


  //toggling a pin takes about 2us
  _tx_delay = bitDelay - 2;
  //reading a pin takes about 2us
  rxIntraBitDelay = bitDelay - 2;
  //it takes about 10uS from when the stop bit is sent to when the ISR is called
  rxCenteringDelay = (bitDelay / 2 - 10);
  if(rxCenteringDelay < 0)
      rxCenteringDelay = 0;
  //the first time the ISR is called is about 5 uS longer
  initRxCenteringDelay = rxCenteringDelay - 5;
  if(initRxCenteringDelay < 0)
      initRxCenteringDelay = 0;

#if _DEBUG
  pinMode(_DEBUG_PIN1, OUTPUT);
  pinMode(_DEBUG_PIN2, OUTPUT);
#endif
  attachInterrupt(rxPin, recv, FALLING);
  listen();
}

void SoftwareSerial::setRxIntMsk(bool enable)
{
    /**
    if (enable)
      *_pcint_maskreg |= _pcint_maskvalue;
    else
      *_pcint_maskreg &= ~_pcint_maskvalue;
    **/
}

void SoftwareSerial::end()
{
  stopListening();
}


// Read data from buffer
int SoftwareSerial::read()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int SoftwareSerial::available()
{
  if (!isListening())
    return -1;

  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(uint8_t b)
{
  if (_tx_delay == 0) {
    setWriteError();
    return 0;
  }
  
  // By declaring these as local variables, the compiler will put them
  // in registers _before_ disabling interrupts and entering the
  // critical timing sections below, which makes it a lot easier to
  // verify the cycle timings

  bool inv = _inverse_logic;
  uint16_t delay = _tx_delay;
  noInterrupts();
  if (inv)
    b = ~b;

  // Write the start bit
  if (inv)
    digitalWrite(txPin, HIGH);
  else
    digitalWrite(txPin, LOW);

  tunedDelay(delay);

  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i)
  {
    if (b & 1) // choose bit
      digitalWrite(txPin, HIGH);
    else
      digitalWrite(txPin, LOW);

    tunedDelay(delay);
    b >>= 1;
  }

  // restore pin to natural state
  if (inv)
    digitalWrite(txPin, LOW);
  else
    digitalWrite(txPin, HIGH);

  interrupts();
  tunedDelay(delay);
  
  return 1;
}

void SoftwareSerial::flush()
{
  if (!isListening())
    return;

  noInterrupts();
  _receive_buffer_head = _receive_buffer_tail = 0;
  interrupts();
}

int SoftwareSerial::peek()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}
