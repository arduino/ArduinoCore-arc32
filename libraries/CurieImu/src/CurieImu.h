/*
 * BMI160 accelerometer/gyroscope library for Intel(R) Curie(TM) devices.
 *
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

#ifndef _CURIEIMU_H_
#define _CURIEIMU_H_

#include "BMI160.h"

/**
 * axis options
 *@see autoCalibrateAccelerometerOffset()
 *@see get/setGyroOffset()
 *@see get/setAccelerometerOffset()
 *@see readAcceleration()
 *@see readRotation()
 *@see shockDetected()
 *@see tapDetected()
 *@see motionDetected()
 */
typedef enum{
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS,
} CurieIMUAxis;

/**
 *direction options
 *@see shockDetected()
 *@see tapDetected()
 *@see motionDetected()
 */
typedef enum{
    POSITIVE,
    NEGATIVE,
} CurieIMUDirection;

 /**
 * Features for getThreshold(), getDuration() functions,
 */
typedef enum {
    CURIE_IMU_FREEFALL = 0,
    CURIE_IMU_SHOCK,
    CURIE_IMU_MOTION,
    CURIE_IMU_ZERO_MOTION,
    CURIE_IMU_STEP,
    CURIE_IMU_TAP,
    CURIE_IMU_TAP_SHOCK,
    CURIE_IMU_TAP_QUIET,
    CURIE_IMU_DOUBLE_TAP,
    CURIE_IMU_FIFO_FULL,
    CURIE_IMU_DATA_READY,
} CurieIMUFeature;

/**
 * Step Detection Mode options
 * @see setStepDetectionMode()
 */
typedef enum {
    CURIE_IMU_STEP_MODE_NORMAL = BMI160_STEP_MODE_NORMAL,
    CURIE_IMU_STEP_MODE_SENSITIVE = BMI160_STEP_MODE_SENSITIVE,
    CURIE_IMU_STEP_MODE_ROBUST = BMI160_STEP_MODE_ROBUST,
    CURIE_IMU_STEP_MODE_UNKNOWN = BMI160_STEP_MODE_UNKNOWN,
} CurieIMUStepMode;

/* Note that this CurieImuClass class inherits methods from the BMI160Class which
 * is defined in BMI160.h.  BMI160Class provides methods for configuring and
 * accessing features of the BMI160 IMU device.  This CurieImuClass extends that
 * class with implementation of details specific to the integration of the BMI160
 * device on the Intel Curie module, such as the serial communication interface
 * and interrupt signalling.
 *
 * Please refer to the respective .cpp files for documentation on each of the
 * methods provided by these classes.
 */
class CurieImuClass : public BMI160Class {
    friend void bmi160_pin1_isr(void);

    public:
        bool begin(void);

        int getGyroRange();
        void setGyroRange(int range);
        int getAccelerometerRange();
        void setAccelerometerRange(int range);

        void autoCalibrateGyroOffset();
        void autoCalibrateAccelerometerOffset(int axis, int target);

        void enableGyroOffset(bool state);
        void enableAccelerometerOffset(bool state);
        bool gyroOffsetEnabled();
        bool accelerometerOffsetEnabled();

        int getGyroOffset(int axis);
        int getAccelerometerOffset(int axis);

        void setGyroOffset(int axis, int offset);
        void setAccelerometerOffset(int axis, int offset);

        int getDetectionThreshold(int feature);
        void setDetectionThreshold(int feature, int threshold);

        int getDetectionDuration(int feature);
        void setDetectionDuration(int feature, int value); //value (bool) duration or samples

        void enableInterrupt(int feature, bool enabled);
        bool interruptEnabled(int feature);

        int getInterruptStatus(int feature);

        CurieIMUStepMode getStepDetectionMode();
        void setStepDetectionMode(int mode);

        void readMotionSensor(short& ax, short& ay, short& az, short& gx, short& gy, short& gz);
        void readAcceleration(short& x, short& y, short& z);
        void readRotation(short& x, short& y, short& z);

        short readAccelerometer(int axis);
        short readGyro(int axis);
        short readTemperature();

        bool shockDetected(int axis, int direction);
        bool motionDetected(int axis, int direction);
        bool tapDetected(int axis, int direction);
        bool stepsDetected();

        void attachInterrupt(void (*callback)(void));
        void detachInterrupt(void);

    private:
        int serial_buffer_transfer(uint8_t *buf, unsigned tx_cnt, unsigned rx_cnt);

        void (*_user_callback)(void);
};

extern CurieImuClass CurieIMU;

#endif /* _CURIEIMU_H_ */
