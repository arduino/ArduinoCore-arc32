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

CurieSoftwareSerial library is a work in progress
Rx is only functional up to 57600 bps
Rx does not work for pin 13
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
#include <SoftwareSerial.h>

//
// Statics
//
SoftwareSerial *SoftwareSerial::active_object = 0;
char *SoftwareSerial::_receive_buffer;
volatile uint8_t SoftwareSerial::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSerial::_receive_buffer_head = 0;

static uint8_t _rxPin;
static uint16_t bitDelay;
static uint16_t rxIntraBitDelay;
static int rxCenteringDelay;
static int initRxIntraBitDelay;
static int firstIntraBitDelay;
static int initRxCenteringDelay;
static bool firstStartBit = true;
static bool bufferOverflow = true;
static bool invertedLogic = false;
static bool isSOCGpio = false;

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
inline void DebugPulse(uint8_t pin, uint8_t count)
{
#if _DEBUG
  digitalWrite(pin, HIGH);
  digitalWrite(pin, LOW);
#endif
}

//
// Private methods
//

/* static */ 
inline void SoftwareSerial::tunedDelay(uint16_t delay) 
{
  //delay in tick counts
  delayTicks(delay);
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
    _rxPin = _receivePin;
    rxIntraBitDelay = _rx_delay_intrabit;
    rxCenteringDelay = _rx_delay_centering;
    initRxIntraBitDelay = _rx_delay_init_intrabit;
    firstIntraBitDelay = _rx_delay_first_intrabit;
    initRxCenteringDelay = _rx_delay_init_centering;
    invertedLogic = _inverse_logic;
    isSOCGpio = _isSOCGpio;
    if(invertedLogic)
    {
      attachInterrupt(_rxPin, recv, HIGH);
    }
    else
    {
      attachInterrupt(_rxPin, recv, LOW);
    }
    
    return true;
  }

  return false;
}

// Stop listening. Returns true if we were actually listening.
bool SoftwareSerial::stopListening()
{
  if (active_object == this)
  {
    active_object = NULL;
    detachInterrupt(_rxPin);
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
  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (invertedLogic ? digitalRead(_rxPin) : !digitalRead(_rxPin))
  {
    // The very first start bit the sketch receives takes about 5us longer
    if(firstStartBit && !isSOCGpio)
    {
      delayTicks(initRxCenteringDelay);
    }
    else
    {
      delayTicks(rxCenteringDelay);
    }

    for (uint8_t i=8; i > 0; --i)
    {
      // compensate for the centering delay if the ISR was too late and missed the center of the start bit.
      if(i == 8) 
      {
        if(firstStartBit && !isSOCGpio) 
        {
          delayTicks(initRxIntraBitDelay);
        }
        else
        {
          delayTicks(firstIntraBitDelay);
        }
      }
      else
      {
        delayTicks(rxIntraBitDelay);
      }
      d >>= 1;
      if (digitalRead(_rxPin))
        d |= 0x80;
      firstStartBit = false;
    }
    
    if (invertedLogic)
      d = ~d;

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
      bufferOverflow = true;
    }

    // wait until we see a stop bit/s or timeout;
    uint8_t loopTimeout = 32;
    if(invertedLogic)
    {
      while(digitalRead(_rxPin) && (loopTimeout >0))
      {
        delayTicks(bitDelay >> 4);
        loopTimeout--;
      }
    }
    else
    {
      while(!digitalRead(_rxPin) && (loopTimeout >0))
      {
        delayTicks(bitDelay >> 4);
        loopTimeout--;
      }
    }
    DebugPulse(_DEBUG_PIN1, 1);
  }
  interrupts();
}

uint32_t SoftwareSerial::rx_pin_read()
{
  return digitalRead(_rxPin);
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
  _inverse_logic = inverse_logic;
  setTX(transmitPin);
  _transmitPin = transmitPin;
  setRX(receivePin);
  _receivePin = receivePin;
  _receive_buffer = (char*)dccm_malloc(_SS_MAX_RX_BUFF);
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
  // output high. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.
  pinMode(tx, OUTPUT);
  digitalWrite(tx, invertedLogic ? LOW : HIGH);
}

void SoftwareSerial::setRX(uint8_t rx)
{
  pinMode(rx, INPUT);
  if (!invertedLogic)
    pinMode(rx, INPUT_PULLUP);
  _receivePin = rx;
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
  _bit_delay = (F_CPU/speed);
  PinDescription *p = &g_APinDescription[_rxPin];
  if (p->ulGPIOType == SOC_GPIO)
  {
    _isSOCGpio = true;
  }
  //toggling a pin takes about 68 ticks
  _tx_delay = _bit_delay - 68;
  //reading a pin takes about 70 ticks
  _rx_delay_intrabit = _bit_delay - 70;
  //it takes about 272 ticks from when the start bit is received to when the ISR is called
  _rx_delay_centering = (_bit_delay / 2 - 272 - 55);
  if(_rx_delay_centering < 0)
  {
    _rx_delay_first_intrabit = _rx_delay_intrabit + _rx_delay_centering;
    if(_rx_delay_first_intrabit < 0)
      _rx_delay_first_intrabit = 0;
    _rx_delay_centering = 0;
  }
  else
  {
    _rx_delay_first_intrabit = _rx_delay_intrabit;
  }
  //the first time the ISR is called is about 150 ticks longer
  _rx_delay_init_centering = _rx_delay_centering - 150;
  if(_rx_delay_init_centering < 0)
  {
    _rx_delay_init_intrabit = _rx_delay_intrabit + _rx_delay_init_centering;
      if(_rx_delay_init_intrabit < 0)
        _rx_delay_init_intrabit = 0;
    _rx_delay_init_centering = 0;
  }
  else
  {
    _rx_delay_init_intrabit = _rx_delay_first_intrabit;
  }
   
#if _DEBUG
  pinMode(_DEBUG_PIN1, OUTPUT);
  pinMode(_DEBUG_PIN2, OUTPUT);
#endif
  listen();
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

  uint16_t delay = _tx_delay;
  noInterrupts();
  if (invertedLogic)
    b = ~b;

  // Write the start bit
  if (invertedLogic)
    digitalWrite(_transmitPin, HIGH);
  else
    digitalWrite(_transmitPin, LOW);

  delayTicks(delay);

  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i)
  {
    if (b & 1) // choose bit
      digitalWrite(_transmitPin, HIGH);
    else
      digitalWrite(_transmitPin, LOW);

    delayTicks(delay);
    b >>= 1;
  }

  // restore pin to natural state
  if (invertedLogic)
    digitalWrite(_transmitPin, LOW);
  else
    digitalWrite(_transmitPin, HIGH);

  interrupts();
  delayTicks(delay);
  
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
