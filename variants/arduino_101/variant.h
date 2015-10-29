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

#ifndef _VARIANT_ARDUINO_101_X_
#define _VARIANT_ARDUINO_101_X_

#include "gpio.h"
#include "ss_gpio_iface.h"
#include "soc_gpio.h"
#include "scss_registers.h"

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Arduino.h"
#include "wiring_digital.h"
#include "pins_arduino.h"
#ifdef __cplusplus
#include "UARTClass.h"
#include "CDCSerialClass.h"
//#include "USARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Platform Identifiers
 *----------------------------------------------------------------------------*/
#define PLATFORM_ID   0xAE
#define PLATFORM_NAME "Arduino101"

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

/*
 * LEDs
 */
#define PIN_LED_13           (13u)
#define PIN_LED              PIN_LED_13
#define LED_BUILTIN          13

/*
 * Other Pins
 */
#define PIN_ATN              (20ul)
 
/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1
#define I2C_MUX_MODE       QRK_PMUX_SEL_MODEA

/*
 * SPI
 */
#define SPI_MUX_MODE        QRK_PMUX_SEL_MODEB

/*
 * GPIO
 */
#define GPIO_MUX_MODE       QRK_PMUX_SEL_MODEA

#define SS_GPIO  1
#define SOC_GPIO 2
#define INPUT_MODE 1
#define OUTPUT_MODE 0
/*
 * PWM
 */
#define PWM_FREQUENCY		490
#define PWM_PERIOD_NS       2048000
#define PWM_MAX_DUTY_CYCLE	65535
#define PWM_MIN_DUTY_CYCLE	1
#define PWM_RESOLUTION		16
#define PWM_MUX_MODE        QRK_PMUX_SEL_MODEB
#define PWM_SCALE_490HZ     0 /* "Standard" Arduino PWM frequency is 490Hz */
#define PWM_SCALE_980HZ     1 /* Some pins on Arduino boards emit 980Hz PWM */

/*
 * UART
 */
#define UART_MUX_MODE       QRK_PMUX_SEL_MODEC

/*
 * ADC
 */

/* EAI ADC device registers */
#define ADC_SET                    (0x80015000)
#define ADC_DIVSEQSTAT             (0x80015001)
#define ADC_SEQ                    (0x80015002)
#define ADC_CTRL                   (0x80015003)
#define ADC_INTSTAT                (0x80015004)
#define ADC_SAMPLE                 (0x80015005)
#define AR_IO_CREG_MST0_CTRL       (0x80018000)
#define AR_IO_CREG_SLV0_OBSR       (0x80018080)

/* ADC Specific macros */
#define ADC_POP_SAMPLE             (0x80000000)
#define ADC_CLR_DATA_A             (1 << 16)
#define ADC_SEQ_TABLE_RST          (0x0040)
#define ADC_SEQ_PTR_RST            (0x0020)
#define ADC_SEQ_START              (0x0010)
#define ADC_INT_DATA_A             (0x1)
#define ADC_CLK_RATIO_MASK         (0x1fffff)
#define ADC_INT_DSB                (0x0F00)
#define ADC_CLK_ENABLE             (0x0004)
#define ADC_ENABLE                 (0x0002)
#define ADC_INT_ENABLE             (0x0000)
#define ADC_MUX_MODE               QRK_PMUX_SEL_MODEB
#define ADC_STANDBY                (0x02)
#define ADC_NORMAL_WO_CALIB        (0x04)
#define ADC_MODE_MASK              (0x07)
#define ADC_CLOCK_RATIO              32
/* Set sample width = 12 bits, input mode = single-ended, output mode = parallel & sequencer mode = single-shot. */
#define ADC_CONFIG_SETUP           (0x0B)
#define ADC_CONFIG_SEQ_TBL         (0x0A)
#define ADC_RESOLUTION               12
#define ADC_CLOCK_GATE             (1 << 31)

#define digitalPinToBitMask(P)     (1 << g_APinDescription[P].ulGPIOId)

//static uint8_t __unused_var_POR;
#define portOutputRegister(port)  (uint32_t*)port
#define portModeRegister(port)    (uint32_t*)port


static inline uint32_t digitalPinToPort(uint32_t pin) {
    uint32_t reg = 0;
    PinDescription *p = &g_APinDescription[pin];

    if (p->ulGPIOType == SS_GPIO)
    {
        reg = p->ulGPIOBase + SS_GPIO_SWPORTA_DR;
    }
    else if (p->ulGPIOType == SOC_GPIO)
    {
        reg = p->ulGPIOBase + SOC_GPIO_SWPORTA_DR;
    }
    return reg;
}

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

extern CDCSerialClass Serial;
extern UARTClass Serial1;

#endif

void CDCSerial_getByte(uint8_t uc_data);
void CDCSerial_bytes_sent(uint32_t num);

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE1       Serial1

extern uint32_t sizeof_g_APinDescription;

#endif /* _VARIANT_ARDUINO_101_X_ */

