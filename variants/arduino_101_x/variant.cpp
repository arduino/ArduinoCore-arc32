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

#include "variant.h"
#include "portable.h"

#include "cfw_platform.h"
#include "platform.h"


/*
 * Arduino 101
 *  Board pin      |     GPIO     | Label
 * ----------------+--------------+-------
 *   0             |  GPIO_SS[9]  | "RX0"
 *   1             |  GPIO_SS[8]  | "TX0"
 *   2             |  GPIO[18]    | ""
 *   3             |  GPIO_SS[10] | "PWM0"
 *   4             |  GPIO[19]    | ""
 *   5             |  GPIO_SS[11] | "PWM1"
 *   6             |  GPIO_SS[12] | "PWM2"
 *   7             |  GPIO[20]    | ""
 *   8             |  GPIO[16]    | ""
 *   9             |  GPIO_SS[13] | "PWM3"
 *  10             |  GPIO[11]    | ""
 *  11             |  GPIO[10]    | "MOSI"
 *  12             |  GPIO[9]     | "MISO"
 *  13             |  GPIO[8]     | "SCK"
 *  14             |  GPIO_SS[2]  | "A0"
 *  15             |  GPIO_SS[3]  | "A1"
 *  16             |  GPIO_SS[4]  | "A2"
 *  17             |  GPIO_SS[5]  | "A3"
 *  18             |  GPIO_SS[6]  | "A4"
 *  19             |  GPIO_SS[1]  | "A5"
 *  20             |  GPIO_SS[0]  | "ATN"
*/


#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pins descriptions
 */
PinDescription g_APinDescription[]=
{

//     gpio port          type       base                   soc pin mux mode       pwm chan pwm scale        adc channel pin mode
    {  1,   SS_GPIO_8B1,  SS_GPIO,   SS_GPIO_8B1_BASE_ADDR, 17,     GPIO_MUX_MODE, INVALID, INVALID,         INVALID, INPUT_MODE }, // Arduino IO0
    {  0,   SS_GPIO_8B1,  SS_GPIO,   SS_GPIO_8B1_BASE_ADDR, 16,     GPIO_MUX_MODE, INVALID, INVALID,         INVALID, INPUT_MODE }, // Arduino IO1
    { 18,   SOC_GPIO_32,  SOC_GPIO,  SOC_GPIO_BASE_ADDR,    52,     GPIO_MUX_MODE, INVALID, INVALID,         INVALID, INPUT_MODE }, // Arduino IO2
    {  2,   SS_GPIO_8B1,  SS_GPIO,   SS_GPIO_8B1_BASE_ADDR, 63,     GPIO_MUX_MODE, 0,       PWM_SCALE_490HZ, INVALID, INPUT_MODE }, // Arduino IO3
    { 19,   SOC_GPIO_32,  SOC_GPIO,  SOC_GPIO_BASE_ADDR,    53,     GPIO_MUX_MODE, INVALID, INVALID,         INVALID, INPUT_MODE }, // Arduino IO4
    {  3,   SS_GPIO_8B1,  SS_GPIO,   SS_GPIO_8B1_BASE_ADDR, 64,     GPIO_MUX_MODE, 1,       PWM_SCALE_980HZ, INVALID, INPUT_MODE }, // Arduino IO5
    {  4,   SS_GPIO_8B1,  SS_GPIO,   SS_GPIO_8B1_BASE_ADDR, 65,     GPIO_MUX_MODE, 2,       PWM_SCALE_980HZ, INVALID, INPUT_MODE }, // Arduino IO6
    { 20,   SOC_GPIO_32,  SOC_GPIO,  SOC_GPIO_BASE_ADDR,    54,     GPIO_MUX_MODE, INVALID, INVALID,         INVALID, INPUT_MODE }, // Arduino IO7
    { 16,   SOC_GPIO_32,  SOC_GPIO,  SOC_GPIO_BASE_ADDR,    50,     GPIO_MUX_MODE, INVALID, INVALID,         INVALID, INPUT_MODE }, // Arduino IO8
    {  5,   SS_GPIO_8B1,  SS_GPIO,   SS_GPIO_8B1_BASE_ADDR, 66,     GPIO_MUX_MODE, 3,       PWM_SCALE_490HZ, INVALID, INPUT_MODE }, // Arduino IO9
    { 11,   SOC_GPIO_32,  SOC_GPIO,  SOC_GPIO_BASE_ADDR,    45,     GPIO_MUX_MODE, INVALID, INVALID,         INVALID, INPUT_MODE }, // Arduino IO10
    { 10,   SOC_GPIO_32,  SOC_GPIO,  SOC_GPIO_BASE_ADDR,    44,     GPIO_MUX_MODE, INVALID, INVALID,         INVALID, INPUT_MODE }, // Arduino IO11
    {  9,   SOC_GPIO_32,  SOC_GPIO,  SOC_GPIO_BASE_ADDR,    43,     GPIO_MUX_MODE, INVALID, INVALID,         INVALID, INPUT_MODE }, // Arduino IO12
    {  8,   SOC_GPIO_32,  SOC_GPIO,  SOC_GPIO_BASE_ADDR,    42,     GPIO_MUX_MODE, INVALID, INVALID,         INVALID, INPUT_MODE }, // Arduino IO13
    {  2,   SS_GPIO_8B0,  SS_GPIO,   SS_GPIO_8B0_BASE_ADDR, 10,     GPIO_MUX_MODE, INVALID, INVALID,              10, INPUT_MODE }, // Arduino IO14
    {  3,   SS_GPIO_8B0,  SS_GPIO,   SS_GPIO_8B0_BASE_ADDR, 11,     GPIO_MUX_MODE, INVALID, INVALID,              11, INPUT_MODE }, // Arduino IO15
    {  4,   SS_GPIO_8B0,  SS_GPIO,   SS_GPIO_8B0_BASE_ADDR, 12,     GPIO_MUX_MODE, INVALID, INVALID,              12, INPUT_MODE }, // Arduino IO16
    {  5,   SS_GPIO_8B0,  SS_GPIO,   SS_GPIO_8B0_BASE_ADDR, 13,     GPIO_MUX_MODE, INVALID, INVALID,              13, INPUT_MODE }, // Arduino IO17
    {  6,   SS_GPIO_8B0,  SS_GPIO,   SS_GPIO_8B0_BASE_ADDR, 14,     GPIO_MUX_MODE, INVALID, INVALID,              14, INPUT_MODE }, // Arduino IO18
    {  1,   SS_GPIO_8B0,  SS_GPIO,   SS_GPIO_8B0_BASE_ADDR,  9,     GPIO_MUX_MODE, INVALID, INVALID,               9, INPUT_MODE }, // Arduino IO19
	{  0,   SS_GPIO_8B0,  SS_GPIO,   SS_GPIO_8B0_BASE_ADDR,  8,     GPIO_MUX_MODE, INVALID, INVALID,         INVALID, INPUT_MODE }, // Arduino IO20

} ;

#ifdef __cplusplus
}
#endif

uint32_t sizeof_g_APinDescription;


/*
 * UART objects
 */

// Serial - CDC-ACM port

uart_init_info info_cdc;

CDCSerialClass Serial(&info_cdc);

void serialEvent() __attribute__((weak));
void serialEvent() { }

// Serial1 - Arduino Header Pins 0 and 1

RingBuffer rx_buffer_uart;
RingBuffer tx_buffer_uart;
uart_init_info info_uart;

UARTClass Serial1(&info_uart, &rx_buffer_uart, &tx_buffer_uart);

void UART_Handler(void)
{
  Serial1.IrqHandler();
}

bool Serial1_available() {
  return Serial1.available();
}

void serialEvent1() __attribute__((weak));
void serialEvent1() { }

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
  if (Serial1.available()) serialEvent1();
}

void serialEventRun1(void)
{
  if (Serial1_available()) serialEvent1();
}
// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

void variantGpioInit(void)
{
#define GPIO_CLKENA_POS         (31)
#define GPIO_LS_SYNC_POS        (0)

    /* Enable SoC GPIO peripheral clock */
    SET_MMIO_BIT((SOC_GPIO_BASE_ADDR+SOC_GPIO_LS_SYNC), GPIO_CLKENA_POS);
    SET_MMIO_BIT((SOC_GPIO_BASE_ADDR+SOC_GPIO_LS_SYNC), GPIO_LS_SYNC_POS);
    /* Enable SS_GPIO port 0 peripheral clock */
    SET_ARC_BIT((SS_GPIO_8B0_BASE_ADDR+SS_GPIO_LS_SYNC), GPIO_CLKENA_POS);
    SET_ARC_BIT((SS_GPIO_8B0_BASE_ADDR+SS_GPIO_LS_SYNC), GPIO_LS_SYNC_POS);
    /* Enable SS_GPIO port 1 peripheral clock */
    SET_ARC_BIT((SS_GPIO_8B1_BASE_ADDR+SS_GPIO_LS_SYNC), GPIO_CLKENA_POS);
    SET_ARC_BIT((SS_GPIO_8B1_BASE_ADDR+SS_GPIO_LS_SYNC), GPIO_LS_SYNC_POS);

    for (uint8_t pin = 0; pin < NUM_DIGITAL_PINS; pin++) {
        PinDescription *p = &g_APinDescription[pin];
        SET_PIN_MODE(p->ulSocPin, p->ulPinMode);
    }
}

void variantPwmInit(void)
{
    /* Enable PWM peripheral clock */
    MMIO_REG_VAL(QRK_CLKGATE_CTRL) |= QRK_CLKGATE_CTRL_PWM_ENABLE;

    /* Select PWM mode, with interrupts masked */
    for (uint8_t i = 0; i < NUM_PWM; i++) {
        uint32_t offset = ((i * QRK_PWM_N_REGS_LEN) + QRK_PWM_N_CONTROL);
        MMIO_REG_VAL_FROM_BASE(QRK_PWM_BASE_ADDR, offset) = QRK_PWM_CONTROL_PWM_OUT | QRK_PWM_CONTROL_INT_MASK | QRK_PWM_CONTROL_MODE_PERIODIC;
    }
}

void variantAdcInit(void)
{
    uint32_t creg;
    uint32_t saved;

    /* read creg slave to get current Power Mode */
    creg = READ_ARC_REG(AR_IO_CREG_SLV0_OBSR);

    /* perform power up to "Normal mode w/o calibration" cycle if not already there */
    if( (creg & ADC_MODE_MASK) != ADC_NORMAL_WO_CALIB){

        /* Protect AR_IO_CREG_MST0_CTRL using lock and unlock of interruptions */
        saved = interrupt_lock();
        /* Read current CREG master */
        creg = READ_ARC_REG(AR_IO_CREG_MST0_CTRL);
        creg &= ~(ADC_MODE_MASK);
        /* request ADC to go to Standby mode */
        creg |= ADC_STANDBY | ADC_CLOCK_GATE;
        WRITE_ARC_REG(creg, AR_IO_CREG_MST0_CTRL);
        interrupt_unlock(saved);
        /* Poll CREG Slave 0 for Power Mode status = requested status */
        while ( (creg = READ_ARC_REG(AR_IO_CREG_SLV0_OBSR) & 0x8) == 0);
        /* Protect AR_IO_CREG_MST0_CTRL using lock and unlock of interruptions */
        saved = interrupt_lock();
        creg = READ_ARC_REG(AR_IO_CREG_MST0_CTRL);
        creg &= ~(ADC_MODE_MASK);
        /* request ADC to go to Normal mode w/o calibration */
        creg |= ADC_NORMAL_WO_CALIB | ADC_CLOCK_GATE;
        WRITE_ARC_REG(creg, AR_IO_CREG_MST0_CTRL);
        interrupt_unlock(saved);
        /* Poll CREG Slave 0 for Power Mode status = requested status */
        while ( ((creg = READ_ARC_REG(AR_IO_CREG_SLV0_OBSR)) & 0x8) == 0);
    }

    WRITE_ARC_REG(ADC_CLK_ENABLE | ADC_INT_DSB, ADC_CTRL);
    WRITE_ARC_REG(ADC_CONFIG_SETUP, ADC_SET);
    WRITE_ARC_REG(ADC_CLOCK_RATIO & ADC_CLK_RATIO_MASK, ADC_DIVSEQSTAT);

}


void initVariant( void )
{
    /* Initialise CDC-ACM shared buffers pointers, provided by LMT */
    Serial.setSharedData(shared_data->cdc_acm_buffers);

    /* For now, lets enable clocks for all interfaces we need
     * TODO - Consider only enabling as needed later to reduce power consumption
     */
    variantGpioInit();
    variantPwmInit();
    variantAdcInit();

    cfw_platform_init();
}

#ifdef __cplusplus
}
#endif
