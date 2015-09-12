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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include "portable.h"
#include "UARTClass.h"
#include "wiring_constants.h"
#include "wiring_digital.h"

extern void UART_Handler(void);
extern void serialEventRun(void) __attribute__((weak));
extern void serialEvent(void) __attribute__((weak));

bool Serial0_available() {
  return Serial.available();
}

void serialEventRun(void)
{
  if (Serial0_available()) serialEvent();
}

// Constructors ////////////////////////////////////////////////////////////////

UARTClass::UARTClass( uart_init_info *info, RingBuffer *pRx_buffer, RingBuffer *pTx_buffer )
{
   this->info = info;
   this->_rx_buffer = pRx_buffer;
   this->_tx_buffer = pTx_buffer;
}

// Public Methods //////////////////////////////////////////////////////////////

void UARTClass::begin(const uint32_t dwBaudRate)
{
  begin(dwBaudRate, SERIAL_8N1);
}

void UARTClass::begin(const uint32_t dwBaudRate, const uint8_t config)
{
  init(dwBaudRate, config );
  opened = true;
}


void UARTClass::init(const uint32_t dwBaudRate, const uint8_t modeReg)
{
  uint8_t c;
  // Make sure both ring buffers are initialized back to empty.
  _rx_buffer->_iHead = _rx_buffer->_iTail = 0;
  _tx_buffer->_iHead = _tx_buffer->_iTail = 0;

  SET_PIN_MODE(17, UART_MUX_MODE); // Rdx SOC PIN (Arduino header pin 0)
  SET_PIN_MODE(16, UART_MUX_MODE); // Txd SOC PIN (Arduino header pin 1)

  info->options = 0;
  info->sys_clk_freq = SYSCLK_DEFAULT_IOSC_HZ;
  info->baud_rate = dwBaudRate;
  info->regs = CONFIG_UART_CONSOLE_REGS;
  info->irq = CONFIG_UART_CONSOLE_IRQ;
  info->int_pri = CONFIG_UART_CONSOLE_INT_PRI;
  info->async_format = modeReg;

  uart_init(CONFIG_UART_CONSOLE_INDEX, info);

  uart_irq_rx_disable(CONFIG_UART_CONSOLE_INDEX);
  uart_irq_tx_disable(CONFIG_UART_CONSOLE_INDEX);

  uart_int_connect(CONFIG_UART_CONSOLE_INDEX,           /* UART to which to connect */
                   UART_Handler, /* interrupt handler */
                   NULL,           /* argument to pass to handler */
                   NULL           /* ptr to interrupt stub code */
                   );

  while (uart_irq_rx_ready(CONFIG_UART_CONSOLE_INDEX))
      uart_fifo_read(CONFIG_UART_CONSOLE_INDEX, &c, 1);


  uart_irq_rx_enable(CONFIG_UART_CONSOLE_INDEX);

}

void UARTClass::end( void )
{
  int ret=0;
  uint8_t uc_data;
  // Wait for any outstanding data to be sent
  flush();
  uart_irq_rx_disable(CONFIG_UART_CONSOLE_INDEX);
  uart_irq_tx_disable(CONFIG_UART_CONSOLE_INDEX);
  while ( ret != -1 ) {
    ret = uart_poll_in(CONFIG_UART_CONSOLE_INDEX, &uc_data);
  }
  opened = false;
  // Clear any received data
  _rx_buffer->_iHead = _rx_buffer->_iTail;
}

void UARTClass::setInterruptPriority(uint32_t priority)
{
  //NVIC_SetPriority(_dwIrq, priority & 0x0F);
}

uint32_t UARTClass::getInterruptPriority()
{
  //return NVIC_GetPriority(_dwIrq);
  return 0;
}

int UARTClass::available( void )
{
  return (uint32_t)(SERIAL_BUFFER_SIZE + _rx_buffer->_iHead - _rx_buffer->_iTail) % SERIAL_BUFFER_SIZE;
}

int UARTClass::availableForWrite(void)
{
  if (!opened)
    return(0);
  int head = _tx_buffer->_iHead;
  int tail = _tx_buffer->_iTail;
  if (head >= tail) return SERIAL_BUFFER_SIZE - 1 - head + tail;
  return tail - head - 1;
}

int UARTClass::peek( void )
{
  if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
    return -1;

  return _rx_buffer->_aucBuffer[_rx_buffer->_iTail];
}

int UARTClass::read( void )
{
  // if the head isn't ahead of the tail, we don't have any characters
  if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
    return -1;

  uint8_t uc = _rx_buffer->_aucBuffer[_rx_buffer->_iTail];
  _rx_buffer->_iTail = (unsigned int)(_rx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE;
  return uc;
}

void UARTClass::flush( void )
{
  while (_tx_buffer->_iHead != _tx_buffer->_iTail); //wait for transmit data to be sent
  // Wait for transmission to complete
  while (uart_irq_tx_ready(CONFIG_UART_CONSOLE_INDEX));
}

size_t UARTClass::write( const uint8_t uc_data )
{
  if (!opened)
    return(0);

  // Is the hardware currently busy?
  if (_tx_buffer->_iTail != _tx_buffer->_iHead)
  {
    // If busy we buffer
    int l = (_tx_buffer->_iHead + 1) % SERIAL_BUFFER_SIZE;
    while (_tx_buffer->_iTail == l)
      ; // Spin locks if we're about to overwrite the buffer. This continues once the data is sent

    _tx_buffer->_aucBuffer[_tx_buffer->_iHead] = uc_data;
    _tx_buffer->_iHead = l;
    // Make sure TX interrupt is enabled
    uart_irq_tx_enable(CONFIG_UART_CONSOLE_INDEX);
  }
  else 
  {
     // Bypass buffering and send character directly
     uart_poll_out(CONFIG_UART_CONSOLE_INDEX, uc_data);
  }
  return 1;
}

void UARTClass::IrqHandler( void )
{
  uint8_t uc_data;
  int ret;
  ret = uart_poll_in(CONFIG_UART_CONSOLE_INDEX, &uc_data);
  
  while ( ret != -1 ) {
    _rx_buffer->store_char(uc_data);
    ret = uart_poll_in(CONFIG_UART_CONSOLE_INDEX, &uc_data);
  }

  // Do we need to keep sending data?
  if (!uart_irq_tx_ready(CONFIG_UART_CONSOLE_INDEX))
  {
    if (_tx_buffer->_iTail != _tx_buffer->_iHead) {
      uart_poll_out(CONFIG_UART_CONSOLE_INDEX, _tx_buffer->_aucBuffer[_tx_buffer->_iTail]);
      _tx_buffer->_iTail = (unsigned int)(_tx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE;
    }
    else
    {
      // Mask off transmit interrupt so we don't get it anymore
      uart_irq_tx_disable(CONFIG_UART_CONSOLE_INDEX);
    }
  }
}

