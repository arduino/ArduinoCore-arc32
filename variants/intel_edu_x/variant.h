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

#ifndef _VARIANT_INTEL_EDU_X_
#define _VARIANT_INTEL_EDU_X_

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
//#include "USARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

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
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

/*
 * GPIO
 */
#define GPIO_MUX_MODE       QRK_PMUX_SEL_MODEA

#define SS_GPIO  1
#define SOC_GPIO 2

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

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

extern UARTClass Serial;
//extern USARTClass Serial1;

#endif

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
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE1       Serial1

extern uint32_t sizeof_g_APinDescription;

#endif /* _VARIANT_INTEL_EDU_X_ */

