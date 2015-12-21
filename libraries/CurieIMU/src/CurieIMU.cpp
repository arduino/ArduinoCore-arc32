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

#include "CurieIMU.h"
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
bool CurieIMUClass::begin()
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

int CurieIMUClass::getGyroRate()
{
    return BMI160Class::getGyroRate();
}

void CurieIMUClass::setGyroRate(int rate)
{
    BMI160Class::setGyroRate(rate);
}

int CurieIMUClass::getAccelerometerRate()
{
    return getAccelRate();
}

void CurieIMUClass::setAccelerometerRate(int rate)
{
    setAccelRate(rate);
}

int CurieIMUClass::getGyroRange()
{
    return getFullScaleGyroRange();
}

void CurieIMUClass::setGyroRange(int range)
{
    setFullScaleGyroRange(range);
}

int CurieIMUClass::getAccelerometerRange()
{
    return getFullScaleAccelRange();
}

void CurieIMUClass::setAccelerometerRange(int range)
{
    setFullScaleAccelRange(range);
}

void CurieIMUClass::autoCalibrateGyroOffset()
{
    BMI160Class::autoCalibrateGyroOffset();
}

void CurieIMUClass::autoCalibrateAccelerometerOffset(int axis, int target)
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

void CurieIMUClass::enableGyroOffset(bool state)
{
    setGyroOffsetEnabled(state);
}

void CurieIMUClass::enableAccelerometerOffset(bool state)
{
    setAccelOffsetEnabled(state);
}

bool CurieIMUClass::gyroOffsetEnabled()
{
    return getGyroOffsetEnabled();
}

bool CurieIMUClass::accelerometerOffsetEnabled()
{
    return getAccelOffsetEnabled();
}

int CurieIMUClass::getGyroOffset(int axis)
{
    if (axis == X_AXIS) {
        return getXGyroOffset();
    } else if (axis == Y_AXIS) {
        return getYGyroOffset();
    } else if (axis == Z_AXIS) {
        return getZGyroOffset();
    }

    return -1;
}

int CurieIMUClass::getAccelerometerOffset(int axis)
{
    if (axis == X_AXIS) {
        return getXAccelOffset();
    } else if (axis == Y_AXIS) {
        return getYAccelOffset();
    } else if (axis == Z_AXIS) {
        return getZAccelOffset();
    }

    return -1;
}

void CurieIMUClass::setGyroOffset(int axis, int offset)
{
    if (axis == X_AXIS) {
        setXGyroOffset(axis);
    } else if (axis == Y_AXIS) {
        setYGyroOffset(axis);
    } else if (axis == Z_AXIS) {
        setZGyroOffset(axis);
    }
}

void CurieIMUClass::setAccelerometerOffset(int axis, int offset)
{
    if (axis == X_AXIS) {
        setXAccelOffset(axis);
    } else if (axis == Y_AXIS) {
        setYAccelOffset(axis);
    } else if (axis == Z_AXIS) {
        setZAccelOffset(axis);
    }
}

int CurieIMUClass::getDetectionThreshold(int feature)
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

void CurieIMUClass::setDetectionThreshold(int feature, int threshold)
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

int CurieIMUClass::getDetectionDuration(int feature)
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

void CurieIMUClass::setDetectionDuration(int feature, int value)
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

void CurieIMUClass::enableInterrupt(int feature, bool enabled)
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

bool CurieIMUClass::interruptEnabled(int feature)
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
            return false;
    }
}

int CurieIMUClass::getInterruptStatus(int feature)
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
            return false;
    }
}

CurieIMUStepMode CurieIMUClass::getStepDetectionMode()
{
    return (CurieIMUStepMode)BMI160Class::getStepDetectionMode();
}

void CurieIMUClass::setStepDetectionMode(int mode)
{
    BMI160Class::setStepDetectionMode((BMI160StepMode)mode);
}

void CurieIMUClass::readMotionSensor(int& ax, int& ay, int& az, int& gx, int& gy, int& gz)
{
    short sax, say, saz, sgx, sgy, sgz;

    getMotion6(&sax, &say, &saz, &sgx, &sgy, &sgz);

    ax = sax;
    ay = say;
    az = saz;
    gx = sgx;
    gy = sgy;
    gz = sgz;
}

void CurieIMUClass::readAccelerometer(int& x, int& y, int& z)
{
    short sx, sy, sz;

    getAcceleration(&sx, &sy, &sz);

    x = sx;
    y = sy;
    z = sz;
}

void CurieIMUClass::readGyro(int& x, int& y, int& z)
{
    short sx, sy, sz;

    getRotation(&sx, &sy, &sz);

    x = sx;
    y = sy;
    z = sz;
}

int CurieIMUClass::readAccelerometer(int axis)
{
    if (axis == X_AXIS) {
        return getAccelerationX();
    } else if (axis == Y_AXIS) {
        return getAccelerationY();
    } else if (axis == Z_AXIS) {
        return getAccelerationZ();
    }

    return 0; 
}

int CurieIMUClass::readGyro(int axis)
{
    if (axis == X_AXIS) {
        return getRotationX();
    } else if (axis == Y_AXIS) {
        return getRotationY();
    } else if (axis == Z_AXIS) {
        return getRotationZ();
    }

    return 0;
}

int CurieIMUClass::readTemperature()
{
    return getTemperature();
}

bool CurieIMUClass::shockDetected(int axis, int direction)
{
    if (direction == POSITIVE) {
        if (axis == X_AXIS) {
            return getXPosShockDetected();
        } else if (axis == Y_AXIS) {
            return getYPosShockDetected();
        } else if (axis == Z_AXIS) {
            return getZPosShockDetected();
        }
    } else if (direction == NEGATIVE) {
        if (axis == X_AXIS) {
            return getXNegShockDetected();
        } else if (axis == Y_AXIS) {
            return getYNegShockDetected();
        } else if (axis == Z_AXIS) {
            return getZNegShockDetected();
        }
    }

    return false;
}

bool CurieIMUClass::motionDetected(int axis, int direction)
{
    if (direction == POSITIVE) {
        if (axis == X_AXIS) {
            return getXPosMotionDetected();
        } else if (axis == Y_AXIS) {
            return getYPosMotionDetected();
        } else if (axis == Z_AXIS) {
            return getZPosMotionDetected();
        }
    } else if (direction == NEGATIVE) {
        if (axis == X_AXIS) {
            return getXNegMotionDetected();
        } else if (axis == Y_AXIS) {
            return getYNegMotionDetected();
        } else if (axis == Z_AXIS) {
            return getZNegMotionDetected();
        }
    }

    return false;
}

bool CurieIMUClass::tapDetected(int axis, int direction)
{
    if (direction == POSITIVE) {
        if (axis == X_AXIS) {
            return getXPosTapDetected();
        } else if (axis == Y_AXIS) {
            return getYPosTapDetected();
        } else if (axis == Z_AXIS) {
            return getZPosTapDetected();
        }
    } else if (direction == NEGATIVE) {
        if (axis == X_AXIS) {
            return getXNegTapDetected();
        } else if (axis == Y_AXIS) {
            return getYNegTapDetected();
        } else if (axis == Z_AXIS) {
            return getZNegTapDetected();
        }
    }

    return false;
}

bool CurieIMUClass::stepsDetected()
{
    return getIntStepStatus();
}

/** Provides a serial buffer transfer implementation for the BMI160 base class
 *  to use for accessing device registers.  This implementation uses the SPI
 *  bus on the Intel Curie module to communicate with the BMI160.
 */
int CurieIMUClass::serial_buffer_transfer(uint8_t *buf, unsigned tx_cnt, unsigned rx_cnt)
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
void CurieIMUClass::attachInterrupt(void (*callback)(void))
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
void CurieIMUClass::detachInterrupt(void)
{
    setIntEnabled(false);

    soc_gpio_deconfig(SOC_GPIO_AON, BMI160_GPIN_AON_PIN);
}

/* Pre-instantiated Object for this class */
CurieIMUClass CurieIMU;
