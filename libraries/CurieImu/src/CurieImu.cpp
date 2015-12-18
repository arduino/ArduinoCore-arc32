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

#include "CurieImu.h"
#include "internal/ss_spi.h"
#include "interrupt.h"

#define CURIE_IMU_CHIP_ID 0xD1

#define BMI160_GPIN_AON_PIN 4

/******************************************************************************/

/** Power on and prepare for general usage.
 * This will prepare the SPI communication interface for accessing the BMI160
 * on the Curie module, before calling BMI160::initialize() to activate the
 * BMI160 accelerometer and gyroscpoe with default settings.
 */
bool CurieImuClass::begin()
{
    /* Configure pin-mux settings on the Intel Curie module to 
     * enable SPI mode usage */
    SET_PIN_MODE(35, QRK_PMUX_SEL_MODEA); // SPI1_SS_MISO 
    SET_PIN_MODE(36, QRK_PMUX_SEL_MODEA); // SPI1_SS_MOSI
    SET_PIN_MODE(37, QRK_PMUX_SEL_MODEA); // SPI1_SS_SCK
    SET_PIN_MODE(38, QRK_PMUX_SEL_MODEA); // SPI1_SS_CS_B[0]
 
    ss_spi_init();

    /* Perform a dummy read from 0x7f to switch to spi interface */
    uint8_t dummy_reg = 0x7F;
    serial_buffer_transfer(&dummy_reg, 1, 1);

    /* The SPI interface is ready - now invoke the base class initialization */
    BMI160Class::initialize();

    /** Verify the SPI connection.
     * MakgetGyroRatee sure the device is connected and responds as expected.
     * @return True if connection is valid, false otherwise
     */
    return (CURIE_IMU_CHIP_ID == getDeviceID());
}

int CurieImuClass::getGyroRate()
{
    return BMI160Class::getGyroRate();
}

void CurieImuClass::setGyroRate(int rate)
{
    BMI160Class::setGyroRate(rate);
}

int CurieImuClass::getAccelerometerRate()
{
    return getAccelRate();
}

void CurieImuClass::setAccelerometerRate(int rate)
{
    setAccelRate(rate);
}

int CurieImuClass::getGyroRange()
{
    return getFullScaleGyroRange();
}

void CurieImuClass::setGyroRange(int range)
{
    setFullScaleGyroRange(range);
}

int CurieImuClass::getAccelerometerRange()
{
    return getFullScaleAccelRange();
}

void CurieImuClass::setAccelerometerRange(int range)
{
    setFullScaleAccelRange(range);
}

void CurieImuClass::autoCalibrateGyroOffset()
{
    BMI160Class::autoCalibrateGyroOffset();
}

void CurieImuClass::autoCalibrateAccelerometerOffset(int axis, int target)
{
    switch (axis) {
        case X_AXIS:
            autoCalibrateXAccelOffset(target);
            break;

        case Y_AXIS:
            autoCalibrateYAccelOffset(target);
            break;

        case Z_AXIS:
            autoCalibrateZAccelOffset(target);
            break;

        default:
            break;
    }
}

void CurieImuClass::enableGyroOffset(bool state)
{
    setGyroOffsetEnabled(state);
}

void CurieImuClass::enableAccelerometerOffset(bool state)
{
    setAccelOffsetEnabled(state);
}

bool CurieImuClass::gyroOffsetEnabled()
{
    return getGyroOffsetEnabled();
}

bool CurieImuClass::accelerometerOffsetEnabled()
{
    return getAccelOffsetEnabled();
}

int CurieImuClass::getGyroOffset(int axis)
{
    switch (axis) {
        case X_AXIS:
            return getXGyroOffset();

        case Y_AXIS:
            return getYGyroOffset();

        case Z_AXIS:
            return getZGyroOffset();

        default:
            return -1;
    }
}

int CurieImuClass::getAccelerometerOffset(int axis)
{
    switch (axis) {
        case X_AXIS:
            return getXAccelOffset();

        case Y_AXIS:
            return getYAccelOffset();

        case Z_AXIS:
            return getZAccelOffset();

        default:
            return -1;
    }
}

void CurieImuClass::setGyroOffset(int axis, int offset)
{
    switch (axis) {
        case X_AXIS:
            setXGyroOffset(axis);
            break;

        case Y_AXIS:
            setYGyroOffset(axis);
            break;

        case Z_AXIS:
            setZGyroOffset(axis);
            break;

        default:
            break;
    }
}

void CurieImuClass::setAccelerometerOffset(int axis, int offset)
{
    switch (axis) {
        case X_AXIS:
            setXAccelOffset(axis);
            break;

        case Y_AXIS:
            setYAccelOffset(axis);
            break;

        case Z_AXIS:
            setZAccelOffset(axis);
            break;

        default:
            break;
    }
}

int CurieImuClass::getDetectionThreshold(int feature)
{
    switch (feature) {
        case CURIE_IMU_FREEFALL:
            return getFreefallDetectionThreshold();

        case CURIE_IMU_SHOCK:
            return getShockDetectionThreshold();

        case CURIE_IMU_MOTION:
            return getMotionDetectionThreshold();

        case CURIE_IMU_ZERO_MOTION:
            return getZeroMotionDetectionThreshold();

        case CURIE_IMU_TAP:
            return getTapDetectionThreshold();

        case CURIE_IMU_STEP:
        case CURIE_IMU_TAP_SHOCK:
        case CURIE_IMU_TAP_QUIET:
        case CURIE_IMU_DOUBLE_TAP:
        case CURIE_IMU_FIFO_FULL:
        case CURIE_IMU_DATA_READY:
        default:
            return -1;
    }
}

void CurieImuClass::setDetectionThreshold(int feature, int threshold)
{
    switch (feature) {
        case CURIE_IMU_FREEFALL:
            setFreefallDetectionThreshold(threshold);
            break;

        case CURIE_IMU_SHOCK:
            setShockDetectionThreshold(threshold);
            break;

        case CURIE_IMU_MOTION:
            setMotionDetectionThreshold(threshold);
            break;

        case CURIE_IMU_ZERO_MOTION:
            setZeroMotionDetectionThreshold(threshold);
            break;

        case CURIE_IMU_TAP:
            setTapDetectionThreshold(threshold);
            break;

        case CURIE_IMU_STEP:
        case CURIE_IMU_TAP_SHOCK:
        case CURIE_IMU_TAP_QUIET:
        case CURIE_IMU_DOUBLE_TAP:
        case CURIE_IMU_FIFO_FULL:
        case CURIE_IMU_DATA_READY:
        default:
            break;
    }
}

int CurieImuClass::getDetectionDuration(int feature)
{
    switch (feature) {
        case CURIE_IMU_FREEFALL:
            return getFreefallDetectionDuration();

        case CURIE_IMU_SHOCK:
            return getShockDetectionDuration();

        case CURIE_IMU_MOTION:
            return getMotionDetectionDuration();

        case CURIE_IMU_TAP_SHOCK:
            return getTapShockDuration();

        case CURIE_IMU_ZERO_MOTION:
            return getZeroMotionDetectionThreshold();

        case CURIE_IMU_TAP_QUIET:
            return getTapQuietDuration();

        case CURIE_IMU_DOUBLE_TAP:
            return getDoubleTapDetectionDuration();

        case CURIE_IMU_TAP:
        case CURIE_IMU_STEP:
        case CURIE_IMU_FIFO_FULL:
        case CURIE_IMU_DATA_READY:
        default:
            return -1;
    }
}

void CurieImuClass::setDetectionDuration(int feature, int value)
{
    switch (feature) {
        case CURIE_IMU_FREEFALL:
            setFreefallDetectionDuration(value);
            break;

        case CURIE_IMU_SHOCK:
            setShockDetectionDuration(value);
            break;

        case CURIE_IMU_MOTION:
            setMotionDetectionDuration(value);
            break;

        case CURIE_IMU_TAP_SHOCK:
            setTapShockDuration(value);
            break;

        case CURIE_IMU_ZERO_MOTION:
            setZeroMotionDetectionThreshold(value);
            break;

        case CURIE_IMU_TAP_QUIET:
            setTapQuietDuration(value);
            break;

        case CURIE_IMU_DOUBLE_TAP:
            setDoubleTapDetectionDuration(value);
            break;

        case CURIE_IMU_TAP:
        case CURIE_IMU_STEP:
        case CURIE_IMU_FIFO_FULL:
        case CURIE_IMU_DATA_READY:
        default:
            break;
    }
}

void CurieImuClass::enableInterrupt(int feature, bool enabled)
{
    switch (feature) {
        case CURIE_IMU_FREEFALL:
            setIntFreefallEnabled(enabled);
            break;

        case CURIE_IMU_SHOCK:
            setIntShockEnabled(enabled);
            break;

        case CURIE_IMU_STEP:
            setIntStepEnabled(enabled);
            break;

        case CURIE_IMU_MOTION:
            setIntMotionEnabled(enabled);
            break;

        case CURIE_IMU_ZERO_MOTION:
            setIntZeroMotionEnabled(enabled);
            break;

       case CURIE_IMU_TAP:
            setIntTapEnabled(enabled);
            break;

        case CURIE_IMU_DOUBLE_TAP:
            setIntDoubleTapEnabled(enabled);
            break;

        case CURIE_IMU_FIFO_FULL:
            setIntFIFOBufferFullEnabled(enabled);
            break;

        case CURIE_IMU_DATA_READY:
            setIntDataReadyEnabled(enabled);
            break;

        case CURIE_IMU_TAP_QUIET:
        case CURIE_IMU_TAP_SHOCK:
        default:
            break;
    }
}

bool CurieImuClass::interruptEnabled(int feature)
{
    switch (feature) {
        case CURIE_IMU_FREEFALL:
            return getIntFreefallEnabled();

        case CURIE_IMU_SHOCK:
            return getIntShockEnabled();

        case CURIE_IMU_STEP:
            return getIntStepEnabled();

        case CURIE_IMU_MOTION:
            return getIntMotionEnabled();

        case CURIE_IMU_ZERO_MOTION:
            return getIntZeroMotionEnabled();

       case CURIE_IMU_TAP:
            return getIntTapEnabled();

        case CURIE_IMU_DOUBLE_TAP:
            return getIntDoubleTapEnabled();

        case CURIE_IMU_FIFO_FULL:
            return getIntFIFOBufferFullEnabled();

        case CURIE_IMU_DATA_READY:
            return getIntDataReadyEnabled();

        case CURIE_IMU_TAP_QUIET:
        case CURIE_IMU_TAP_SHOCK:
        default:
            return -1;
    }
}

int CurieImuClass::getInterruptStatus(int feature)
{
    switch (feature) {
        case CURIE_IMU_FREEFALL:
            return getIntFreefallStatus();

        case CURIE_IMU_SHOCK:
            return getIntShockStatus();

        case CURIE_IMU_STEP:
            return getIntStepStatus();

        case CURIE_IMU_MOTION:
            return getIntMotionStatus();

        case CURIE_IMU_ZERO_MOTION:
            return getIntZeroMotionStatus();

        case CURIE_IMU_TAP:
            return getIntTapStatus();

        case CURIE_IMU_DOUBLE_TAP:
            return getIntDoubleTapStatus();

        case CURIE_IMU_FIFO_FULL:
            return getIntFIFOBufferFullStatus();

        case CURIE_IMU_DATA_READY:
            return getIntDataReadyStatus();

        case CURIE_IMU_TAP_QUIET:
        case CURIE_IMU_TAP_SHOCK:
        default:
            return -1;
    }
}

CurieIMUStepMode CurieImuClass::getStepDetectionMode()
{
    return (CurieIMUStepMode)BMI160Class::getStepDetectionMode();
}

void CurieImuClass::setStepDetectionMode(int mode)
{
    BMI160Class::setStepDetectionMode((BMI160StepMode)mode);
}

void CurieImuClass::readMotionSensor(short& ax, short& ay, short& az, short& gx, short& gy, short& gz)
{
    getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

void CurieImuClass::readAcceleration(short& x, short& y, short& z)
{
    getAcceleration(&x, &y, &z);
}

void CurieImuClass::readRotation(short& x, short& y, short& z)
{
    getRotation(&x, &y, &z);
}

short CurieImuClass::readAccelerometer(int axis)
{
    switch (axis) {
        case X_AXIS:
            return getAccelerationX();

        case Y_AXIS:
            return getAccelerationY();

        case Z_AXIS:
            return getAccelerationZ();

        default:
            return -1;
    }
}

short CurieImuClass::readGyro(int axis)
{
    switch (axis) {
        case X_AXIS:
            return getRotationX();

        case Y_AXIS:
            return getRotationY();

        case Z_AXIS:
            return getRotationZ();

        default:
            return -1;
    }
}

short CurieImuClass::readTemperature()
{
    return getTemperature();
}

bool CurieImuClass::shockDetected(int axis, int direction)
{
    if (direction == POSITIVE) {
        switch (axis) {
            case X_AXIS:
                return getXPosShockDetected();

            case Y_AXIS:
                return getYPosShockDetected();

            case Z_AXIS:
                return getZPosShockDetected();

            default:
                return -1;
        }
    } else if (direction == NEGATIVE) {
        switch (axis) {
            case X_AXIS:
                return getXNegShockDetected();

            case Y_AXIS:
                return getYNegShockDetected();

            case Z_AXIS:
                return getZNegShockDetected();

            default:
                return -1;
        }
    } else {
        return -1;
    }
}

bool CurieImuClass::motionDetected(int axis, int direction)
{
    if (direction == POSITIVE) {
        switch (axis) {
            case X_AXIS:
                return getXPosMotionDetected();

            case Y_AXIS:
                return getYPosMotionDetected();

            case Z_AXIS:
                return getZPosMotionDetected();

            default:
                return -1;
        }
    } else if (direction == NEGATIVE) {
        switch (axis) {
            case X_AXIS:
                return getXNegMotionDetected();

            case Y_AXIS:
                return getYNegMotionDetected();

            case Z_AXIS:
                return getZNegMotionDetected();

            default:
                return -1;
        }
    } else {
        return -1;
    }
}

bool CurieImuClass::tapDetected(int axis, int direction)
{
    if (direction == POSITIVE) {
        switch (axis) {
            case X_AXIS:
                return getXPosTapDetected();

            case Y_AXIS:
                return getYPosTapDetected();

            case Z_AXIS:
                return getZPosTapDetected();

            default:
                return -1;
        }
    } else if (direction == NEGATIVE) {
        switch (axis) {
            case X_AXIS:
                return getXNegTapDetected();

            case Y_AXIS:
                return getYNegTapDetected();

            case Z_AXIS:
                return getZNegTapDetected();

            default:
                return -1;
        }
    } else {
        return -1;
    }
}

bool CurieImuClass::stepsDetected()
{
    return getIntStepStatus();
}

/** Provides a serial buffer transfer implementation for the BMI160 base class
 *  to use for accessing device registers.  This implementation uses the SPI
 *  bus on the Intel Curie module to communicate with the BMI160.
 */
int CurieImuClass::serial_buffer_transfer(uint8_t *buf, unsigned tx_cnt, unsigned rx_cnt)
{
    int flags, status;

    if (rx_cnt) /* For read transfers, assume 1st byte contains register address */
        buf[0] |= (1 << BMI160_SPI_READ_BIT);

    /* Lock interrupts here to
     * - avoid concurrent access to the SPI bus
     * - avoid delays in SPI transfer due to unrelated interrupts
     */
    flags = interrupt_lock();
    status = ss_spi_xfer(buf, tx_cnt, rx_cnt);
    interrupt_unlock(flags);

    return status;
}

/** Interrupt handler for interrupts from PIN1 on the BMI160
 *  Calls a user callback if available.  The user callback is
 *  responsible for checking the source of the interrupt using
 *  the relevant API functions from the BMI160Class base class.
 */
void bmi160_pin1_isr(void)
{
    soc_gpio_mask_interrupt(SOC_GPIO_AON, BMI160_GPIN_AON_PIN);
    if (CurieIMU._user_callback)
        CurieIMU._user_callback();
    soc_gpio_unmask_interrupt(SOC_GPIO_AON, BMI160_GPIN_AON_PIN);
}

/** Stores a user callback, and enables PIN1 interrupts from the
 *  BMI160 module.
 */
void CurieImuClass::attachInterrupt(void (*callback)(void))
{
    gpio_cfg_data_t cfg;

    _user_callback = callback;

    memset(&cfg, 0, sizeof(gpio_cfg_data_t));
    cfg.gpio_type = GPIO_INTERRUPT;
    cfg.int_type = EDGE;
    cfg.int_polarity = ACTIVE_LOW;
    cfg.int_debounce = DEBOUNCE_ON;
    cfg.gpio_cb = bmi160_pin1_isr;
    soc_gpio_set_config(SOC_GPIO_AON, BMI160_GPIN_AON_PIN, &cfg);

    setInterruptMode(1);  // Active-Low
    setInterruptDrive(0); // Push-Pull
    setInterruptLatch(BMI160_LATCH_MODE_10_MS); // 10ms pulse
    setIntEnabled(true);
}

/** Disables PIN1 interrupts from the BMI160 module.
 */
void CurieImuClass::detachInterrupt(void)
{
    setIntEnabled(false);

    soc_gpio_deconfig(SOC_GPIO_AON, BMI160_GPIN_AON_PIN);
}

/* Pre-instantiated Object for this class */
CurieImuClass CurieIMU;
