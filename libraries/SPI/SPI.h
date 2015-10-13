/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings AVR)
 * Copyright (c) 2014 by Andrew J. Kroll <xxxajk@gmail.com> (atomicity fixes)
 * Copyright (c) 2015 by Intel Corporation <dan@emutex.com> (Arduino 101 support)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Arduino.h>

#include "SPI_registers.h"

// SPI_HAS_TRANSACTION means SPI has beginTransaction(), endTransaction(),
// usingInterrupt(), and SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

// SPI_ATOMIC_VERSION means that SPI has atomicity fixes and what version.
// This way when there is a bug fix you can check this define to alert users
// of your code if it uses better version of this library.
// This also implies everything that SPI_HAS_TRANSACTION as documented above is
// available too.
#define SPI_ATOMIC_VERSION 1

// Uncomment this line to add detection of mismatched begin/end transactions.
// A mismatch occurs if other libraries fail to use SPI.endTransaction() for
// each SPI.beginTransaction().  Connect an LED to this pin.  The LED will turn
// on if any mismatch is ever detected.
//#define SPI_TRANSACTION_MISMATCH_LED 5

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

/* For Arduino Uno compatibility, divider values are doubled to provide equivalent clock rates
 * e.g. SPI_CLOCK_DIV4 will produce a 4MHz clock
 * The Intel Curie has a 32MHz base clock and a min divider of 2, so max SPI clock is 16MHz
 */
#define SPI_CLOCK_DIV4     8
#define SPI_CLOCK_DIV16   32
#define SPI_CLOCK_DIV64  128
#define SPI_CLOCK_DIV128 256
#define SPI_CLOCK_DIV2     4
#define SPI_CLOCK_DIV8    16
#define SPI_CLOCK_DIV32   64

#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03

class SPISettings {
public:
  SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
    /* Set frame size, bus mode and transfer mode */
    ctrl0 = (SPI_8_BIT << SPI_FSIZE_SHIFT) | ((dataMode << SPI_MODE_SHIFT) & SPI_MODE_MASK);
    /* Set SPI Clock Divider */
    baudr = (SPI_BASE_CLOCK / clock) & SPI_CLOCK_MASK;
    /* Only MSBFIRST supported in hardware, need to swizzle the bits if LSBFIRST selected */
    lsbFirst = (bitOrder == LSBFIRST) ? true : false;
  }
  SPISettings() {
    SPISettings(4000000, MSBFIRST, SPI_MODE0);
  }
private:
  uint32_t ctrl0;
  uint32_t baudr;
  bool lsbFirst;
  friend class SPIClass;
};


class SPIClass {
public:
  SPIClass(void) { initialized = 0; }

  // Initialize the SPI library
  void begin();

  // If SPI is used from within an interrupt, this function registers
  // that interrupt with the SPI library, so beginTransaction() can
  // prevent conflicts.  The input interruptNumber is the number used
  // with attachInterrupt.  If SPI is used from a different interrupt
  // (eg, a timer), interruptNumber should be 255.
  void usingInterrupt(uint8_t interruptNumber);
  // And this does the opposite.
  void notUsingInterrupt(uint8_t interruptNumber);
  // Note: the usingInterrupt and notUsingInterrupt functions should
  // not to be called from ISR context or inside a transaction.
  // For details see:
  // https://github.com/arduino/Arduino/pull/2381
  // https://github.com/arduino/Arduino/pull/2449

  // Before using SPI.transfer() or asserting chip select pins,
  // this function is used to gain exclusive access to the SPI bus
  // and configure the correct settings.
  inline void beginTransaction(SPISettings settings) {
      if (interruptMode > 0) {
          if (interruptMode < 8) {
              if (interruptMode & 1)
                  SET_ARC_MASK(SS_GPIO_8B0_BASE_ADDR + SS_GPIO_INTMASK, interruptMask[0]);
              if (interruptMode & 2)
                  SET_ARC_MASK(SS_GPIO_8B1_BASE_ADDR + SS_GPIO_INTMASK, interruptMask[1]);
              if (interruptMode & 4)
                  SET_MMIO_MASK(SOC_GPIO_BASE_ADDR + SOC_GPIO_INTMASK,  interruptMask[2]);
          } else {
              noInterrupts();
          }
      }

#ifdef SPI_TRANSACTION_MISMATCH_LED
      if (inTransactionFlag) {
          pinMode(SPI_TRANSACTION_MISMATCH_LED, OUTPUT);
          digitalWrite(SPI_TRANSACTION_MISMATCH_LED, HIGH);
      }
      inTransactionFlag = 1;
#endif

      /* disable controller */
      SPI1_M_REG_VAL(SPIEN) &= SPI_DISABLE;
      /* Configure clock divider, frame size and data mode */
      SPI1_M_REG_VAL(BAUDR) = settings.baudr;
      SPI1_M_REG_VAL(CTRL0) = settings.ctrl0;
      frameSize = SPI_8_BIT;
      lsbFirst = settings.lsbFirst;
      /* Enable controller */
      SPI1_M_REG_VAL(SPIEN) |= SPI_ENABLE;
  }

  // Write to the SPI bus (MOSI pin) and also receive (MISO pin)
  inline uint8_t transfer(uint8_t data) {
    setFrameSize(SPI_8_BIT);
    if (lsbFirst)
        return SPI_REVERSE_8(singleTransfer(SPI_REVERSE_8(data)));
    else
        return singleTransfer(data);
  }

  inline uint16_t transfer16(uint16_t data) {
    setFrameSize(SPI_16_BIT);
    if (lsbFirst)
        return SPI_REVERSE_16(singleTransfer(SPI_REVERSE_16(data)));
    else
        return singleTransfer(data);
  }

  inline uint32_t transfer24(uint32_t data) {
    setFrameSize(SPI_24_BIT);
    if (lsbFirst)
        return SPI_REVERSE_24(singleTransfer(SPI_REVERSE_24(data)));
    else
        return singleTransfer(data);
  }

  inline uint32_t transfer32(uint32_t data) {
    setFrameSize(SPI_32_BIT);
    if (lsbFirst)
        return SPI_REVERSE_32(singleTransfer(SPI_REVERSE_32(data)));
    else
        return singleTransfer(data);
  }

  inline void transfer(void *buf, size_t count) {
      setFrameSize(SPI_8_BIT);
      size_t remaining = count;
      uint8_t *p = (uint8_t *)buf;
      if (lsbFirst) {
          for (uint32_t i = 0; i < count; i++)
              p[i] = SPI_REVERSE_8(p[i]);
      }
      while (remaining > 0) {
          uint32_t transferSize = SPI_FIFO_DEPTH > remaining ? remaining : SPI_FIFO_DEPTH;
          /* Fill the TX FIFO */
          for (uint32_t i = 0; i < transferSize; i++)
              SPI1_M_REG_VAL(DR) = *(p + i);
          remaining -= transferSize;
          /* Wait for transfer to complete */
          while (SPI1_M_REG_VAL(SR) & SPI_STATUS_BUSY) ;
          do {
              uint32_t rxLevel = SPI1_M_REG_VAL(RXFL);
              /* Drain the RX FIFO */
              for (uint32_t i = 0; i < rxLevel; i++)
                  *(p + i) = SPI1_M_REG_VAL(DR);
              p += rxLevel;
              transferSize -= rxLevel;
          } while (transferSize);
      }
      if (lsbFirst) {
          p = (uint8_t *)buf;
          for (uint32_t i = 0; i < count; i++)
              p[i] = SPI_REVERSE_8(p[i]);
      }
  }

  // After performing a group of transfers and releasing the chip select
  // signal, this function allows others to access the SPI bus
  inline void endTransaction(void) {
#ifdef SPI_TRANSACTION_MISMATCH_LED
      if (!inTransactionFlag) {
          pinMode(SPI_TRANSACTION_MISMATCH_LED, OUTPUT);
          digitalWrite(SPI_TRANSACTION_MISMATCH_LED, HIGH);
      }
      inTransactionFlag = 0;
#endif

      if (interruptMode > 0) {
          if (interruptMode < 8) {
              if (interruptMode & 1)
                  CLEAR_ARC_MASK(SS_GPIO_8B0_BASE_ADDR + SS_GPIO_INTMASK, interruptMask[0]);
              if (interruptMode & 2)
                  CLEAR_ARC_MASK(SS_GPIO_8B1_BASE_ADDR + SS_GPIO_INTMASK, interruptMask[1]);
              if (interruptMode & 4)
                  CLEAR_MMIO_MASK(SOC_GPIO_BASE_ADDR + SOC_GPIO_INTMASK,  interruptMask[2]);
          } else {
              interrupts();
          }	
      }
  }

  // Disable the SPI bus
  void end();

  // This function is deprecated.  New applications should use
  // beginTransaction() to configure SPI settings.
  inline void setBitOrder(uint8_t bitOrder) {
    /* Only MSBFIRST supported in hardware, need to swizzle the bits if LSBFIRST selected */
    lsbFirst = (bitOrder == LSBFIRST) ? true : false;
  }
  // This function is deprecated.  New applications should use
  // beginTransaction() to configure SPI settings.
  void setDataMode(uint8_t dataMode);
  // This function is deprecated.  New applications should use
  // beginTransaction() to configure SPI settings.
  void setClockDivider(uint8_t clockDiv);

private:
  uint32_t initialized;
  uint32_t interruptMode;    // 0=none, 1-7=mask, 8=global
  uint32_t interruptMask[3]; // which interrupts to mask
  #ifdef SPI_TRANSACTION_MISMATCH_LED
  uint32_t inTransactionFlag;
  #endif
  bool lsbFirst;
  uint32_t frameSize;

  inline void setFrameSize(uint32_t size) {
    if (frameSize != size) {
      /* disable controller */
      SPI1_M_REG_VAL(SPIEN) &= SPI_DISABLE;
      /* Configure new frame size */
      frameSize = size;
      SPI1_M_REG_VAL(CTRL0) = (SPI1_M_REG_VAL(CTRL0) & ~(SPI_FSIZE_MASK)) | ((frameSize << SPI_FSIZE_SHIFT) & SPI_FSIZE_MASK);
      /* Enable controller */
      SPI1_M_REG_VAL(SPIEN) |= SPI_ENABLE;
    }
  }

  inline uint32_t singleTransfer(uint32_t data) {
      /* Write to TX FIFO */
      SPI1_M_REG_VAL(DR) = data;
      /* Wait for transfer to complete */
      while (SPI1_M_REG_VAL(SR) & SPI_STATUS_BUSY) ;
      while (SPI1_M_REG_VAL(RXFL) == 0) ;
      /* Read from RX FIFO */
      return SPI1_M_REG_VAL(DR);
  }
};

extern SPIClass SPI;

#endif
