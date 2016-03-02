/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#include "SPI.h"

SPIClass SPI;

void SPIClass::setClockDivider(uint8_t clockDiv)
{
    /* disable controller */
    SPI1_M_REG_VAL(SPIEN) &= SPI_DISABLE;

    /* Set SPI Clock Divider */
    SPI1_M_REG_VAL(BAUDR) = clockDiv & SPI_CLOCK_MASK;

    /* re-enable controller */
    SPI1_M_REG_VAL(SPIEN) |= SPI_ENABLE;
}

void SPIClass::setDataMode(uint8_t dataMode)
{
    /* disable controller */
    SPI1_M_REG_VAL(SPIEN) &= SPI_DISABLE;
    
    /* Set frame size, bus mode and transfer mode */
    SPI1_M_REG_VAL(CTRL0) = (SPI1_M_REG_VAL(CTRL0) & ~(SPI_MODE_MASK)) | ((dataMode << SPI_MODE_SHIFT) & SPI_MODE_MASK);

    /* re-enable controller */
    SPI1_M_REG_VAL(SPIEN) |= SPI_ENABLE;
}

void SPIClass::begin()
{
    uint32_t flags = interrupt_lock(); // Protect from a scheduler and prevent transactionBegin
    if (!initialized) {
        interruptMode = 0;
        interruptMask[0] = 0;
        interruptMask[1] = 0;
        interruptMask[2] = 0;
#ifdef SPI_TRANSACTION_MISMATCH_LED
        inTransactionFlag = 0;
#endif
        lsbFirst = false;
        frameSize = SPI_8_BIT;

        // Set SS to high so a connected chip will be "deselected" by default
        // TODO - confirm that data register is updated even if pin is set as input
        digitalWrite(SS, HIGH);

        // When the SS pin is set as OUTPUT, it can be used as
        // a general purpose output port (it doesn't influence
        // SPI operations).
        pinMode(SS, OUTPUT);

        /* disable controller */
        SPI1_M_REG_VAL(SPIEN) &= SPI_DISABLE;
		
		/* Enable clock to peripheral */
		MMIO_REG_VAL(PERIPH_CLK_GATE_CTRL) |= ENABLE_SPI_MASTER_1;
		
        /* Configure defaults for clock divider, frame size and data mode */
        SPI1_M_REG_VAL(BAUDR) = SPI_CLOCK_DIV4;
        SPI1_M_REG_VAL(CTRL0) = (frameSize << SPI_FSIZE_SHIFT) | (SPI_MODE0 << SPI_MODE_SHIFT);

        /* Disable interrupts */
        SPI1_M_REG_VAL(IMR) = SPI_DISABLE_INT;
        /* Enable at least one slave device (mandatory, though SS signals are unused) */
        SPI1_M_REG_VAL(SER) = 0x1;
        /* Enable controller */
        SPI1_M_REG_VAL(SPIEN) |= SPI_ENABLE;

        /* Set SoC pin mux configuration */
        SET_PIN_MODE(g_APinDescription[MOSI].ulSocPin, SPI_MUX_MODE);
        SET_PIN_MODE(g_APinDescription[MISO].ulSocPin, SPI_MUX_MODE);
        SET_PIN_MODE(g_APinDescription[SCK].ulSocPin,  SPI_MUX_MODE);
        g_APinDescription[MOSI].ulPinMode = SPI_MUX_MODE;
        g_APinDescription[MISO].ulPinMode = SPI_MUX_MODE;
        g_APinDescription[SCK].ulPinMode  = SPI_MUX_MODE;

    }
    initialized++; // reference count
    interrupt_unlock(flags);
}

void SPIClass::end() {
    uint32_t flags = interrupt_lock(); // Protect from a scheduler and prevent transactionBegin
    // Decrease the reference counter
    if (initialized)
        initialized--;
    // If there are no more references disable SPI
    if (!initialized) {
        SPI1_M_REG_VAL(SPIEN) &= SPI_DISABLE;
        MMIO_REG_VAL(PERIPH_CLK_GATE_CTRL) &= DISABLE_SPI_MASTER_1;
#ifdef SPI_TRANSACTION_MISMATCH_LED
        inTransactionFlag = 0;
#endif
    }
    interrupt_unlock(flags);
}

void SPIClass::usingInterrupt(uint8_t interruptNumber) {
    noInterrupts();
    if (interruptMode < 8) {
        if (interruptNumber >= NUM_DIGITAL_PINS) {
            interruptMode = 8;
        } else {
            uint32_t pin = interruptNumber;
            PinDescription *p = &g_APinDescription[pin];
            if (p->ulGPIOPort == SS_GPIO_8B0) {
                interruptMode |= 1;
                interruptMask[0] |= (1 << p->ulGPIOId);
            } else if (p->ulGPIOPort == SS_GPIO_8B1) {
                interruptMode |= 2;
                interruptMask[1] |= (1 << p->ulGPIOId);
            } else if (p->ulGPIOPort == SOC_GPIO_32) {
                interruptMode |= 4;
                interruptMask[2] |= (1 << p->ulGPIOId);
            } else {
                interruptMode = 8;
            }
        }
    }
    interrupts();
}

void SPIClass::notUsingInterrupt(uint8_t interruptNumber) {
    // Once in mode 8 we can't go back to 0 without a proper reference count
    if (interruptMode == 8)
        return;

    noInterrupts();
    if (interruptNumber < NUM_DIGITAL_PINS) {
        uint32_t pin = interruptNumber;
        PinDescription *p = &g_APinDescription[pin];
        if (p->ulGPIOPort == SS_GPIO_8B0) {
            interruptMask[0] &= ~(1 << p->ulGPIOId);
            if (!interruptMask[0])
                interruptMode &= ~1;
        } else if (p->ulGPIOPort == SS_GPIO_8B1) {
            interruptMask[1] &= ~(1 << p->ulGPIOId);
            if (!interruptMask[1])
                interruptMode &= ~2;
        } else if (p->ulGPIOPort == SOC_GPIO_32) {
            interruptMask[2] &= ~(1 << p->ulGPIOId);
            if (!interruptMask[2])
                interruptMode &= ~4;
        }
    }
    interrupts();
}
