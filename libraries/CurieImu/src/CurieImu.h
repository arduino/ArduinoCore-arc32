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
 * Accelerometer Sensitivity Range options
 * @see setAccelerometerRange()
 */
typedef enum {
    CURIE_IMU_ACCELEROMETER_RANGE_2G = BMI160_ACCEL_RANGE_2G,
    CURIE_IMU_ACCELEROMETER_RANGE_4G = BMI160_ACCEL_RANGE_4G,
    CURIE_IMU_ACCELEROMETER_RANGE_8G = BMI160_ACCEL_RANGE_8G,
    CURIE_IMU_ACCEL_RANGE_16G = BMI160_ACCEL_RANGE_16G
} CurieIMUAccelerometerRange;

/**
 * Gyroscope Sensitivity Range options
 * @see setGyroRange()
 */
typedef enum {
    CURIE_IMU_GYRO_RANGE_2000 = BMI160_GYRO_RANGE_2000,
    CURIE_IMU_GYRO_RANGE_1000 = BMI160_GYRO_RANGE_1000,
    CURIE_IMU_GYRO_RANGE_500 = BMI160_GYRO_RANGE_500,
    CURIE_IMU_GYRO_RANGE_250 = BMI160_GYRO_RANGE_250,
    CURIE_IMU_GYRO_RANGE_125 = BMI160_GYRO_RANGE_125
} CurieIMUGyroRange;

/**
 * Accelerometer Output Data Rate options
 * @see setAccelerometerRate()
 */
typedef enum {
    CURIE_IMU_ACCELEROMETER_RATE_25_2HZ = BMI160_ACCEL_RATE_25_2HZ,
    CURIE_IMU_ACCELEROMETER_RATE_25HZ = BMI160_ACCEL_RATE_25HZ,
    CURIE_IMU_ACCELEROMETER_RATE_50HZ = BMI160_ACCEL_RATE_50HZ,
    CURIE_IMU_ACCELEROMETER_RATE_100HZ = BMI160_ACCEL_RATE_100HZ,
    CURIE_IMU_ACCELEROMETER_RATE_200HZ = BMI160_ACCEL_RATE_200HZ,
    CURIE_IMU_ACCELEROMETER_RATE_400HZ = BMI160_ACCEL_RATE_400HZ,
    CURIE_IMU_ACCELEROMETER_RATE_800HZ = BMI160_ACCEL_RATE_800HZ,
    CURIE_IMU_ACCELEROMETER_RATE_1600HZ = BMI160_ACCEL_RATE_1600HZ
} CurieIMUAccelRate;

/**
 * Gyroscope Output Data Rate options
 * @see setGyroRate()
 */
typedef enum {
    CURIE_IMU_GYRO_RATE_25HZ = BMI160_GYRO_RATE_25HZ,
    CURIE_IMU_GYRO_RATE_50HZ = BMI160_GYRO_RATE_50HZ,
    CURIE_IMU_GYRO_RATE_100HZ = BMI160_GYRO_RATE_100HZ,
    CURIE_IMU_GYRO_RATE_200HZ = BMI160_GYRO_RATE_200HZ,
    CURIE_IMU_GYRO_RATE_400HZ = BMI160_GYRO_RATE_400HZ,
    CURIE_IMU_GYRO_RATE_800HZ = BMI160_GYRO_RATE_800HZ,
    CURIE_IMU_GYRO_RATE_1600HZ = BMI160_GYRO_RATE_1600HZ,
    CURIE_IMU_GYRO_RATE_3200HZ = BMI160_GYRO_RATE_3200HZ
} CurieIMUGyroRate;

/**
 * Step Detection Mode options
 * @see setStepDetectionMode()
 */
typedef enum {
    CURIE_IMU_STEP_MODE_NORMAL = BMI160_STEP_MODE_NORMAL,
    CURIE_IMU_STEP_MODE_SENSITIVE = BMI160_STEP_MODE_SENSITIVE,
    CURIE_IMU_STEP_MODE_ROBUST = BMI160_STEP_MODE_ROBUST,
    CURIE_IMU_STEP_MODE_UNKNOWN = BMI160_STEP_MODE_UNKNOWN
} CurieIMUStepMode;

/**
 * Tap Detection Shock Duration options
 * @see setDetectionThreshold(CURIE_IMU_TAP, ...)
 */
typedef enum {
    CURIE_IMU_TAP_SHOCK_DURATION_50MS = BMI160_TAP_SHOCK_DURATION_50MS,
    CURIE_IMU_TAP_SHOCK_DURATION_75MS = BMI160_TAP_SHOCK_DURATION_75MS
} CurieIMUTapShockDuration;

/**
 * Tap Detection Quiet Duration options
 * @see setDetectionThreshold(CURIE_IMU_TAP_QUIET, ...)
 */
typedef enum {
    CURIE_IMU_TAP_QUIET_DURATION_30MS = BMI160_TAP_QUIET_DURATION_30MS,
    CURIE_IMU_TAP_QUIET_DURATION_20MS = BMI160_TAP_QUIET_DURATION_20MS
} CurieIMUTapQuietDuration;

/**
 * Double-Tap Detection Duration options
 * @see setDetectionThreshold(CURIE_IMU_DOUBLE_TAP, ...)
 */
typedef enum {
    CURIE_IMU_DOUBLE_TAP_DURATION_50MS = BMI160_DOUBLE_TAP_DURATION_50MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_100MS = BMI160_DOUBLE_TAP_DURATION_100MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_150MS = BMI160_DOUBLE_TAP_DURATION_150MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_200MS = BMI160_DOUBLE_TAP_DURATION_200MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_250MS = BMI160_DOUBLE_TAP_DURATION_250MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_375MS = BMI160_DOUBLE_TAP_DURATION_375MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_500MS = BMI160_DOUBLE_TAP_DURATION_500MS,
    CURIE_IMU_DOUBLE_TAP_DURATION_700MS = BMI160_DOUBLE_TAP_DURATION_700MS
} CurieIMUDoubleTapDuration;

/**
 * Zero-Motion Detection Duration options
 * @see setDetectionThreshold(CURIE_IMU_ZERO_MOTION, ...)
 */
typedef enum {
    CURIE_IMU_ZERO_MOTION_DURATION_1_28S = BMI160_ZERO_MOTION_DURATION_1_28S,
    CURIE_IMU_ZERO_MOTION_DURATION_2_56S = BMI160_ZERO_MOTION_DURATION_2_56S,
    CURIE_IMU_ZERO_MOTION_DURATION_3_84S = BMI160_ZERO_MOTION_DURATION_3_84S,
    CURIE_IMU_ZERO_MOTION_DURATION_5_12S = BMI160_ZERO_MOTION_DURATION_5_12S,
    CURIE_IMU_ZERO_MOTION_DURATION_6_40S = BMI160_ZERO_MOTION_DURATION_6_40S,
    CURIE_IMU_ZERO_MOTION_DURATION_7_68S = BMI160_ZERO_MOTION_DURATION_7_68S,
    CURIE_IMU_ZERO_MOTION_DURATION_8_96S = BMI160_ZERO_MOTION_DURATION_8_96S,
    CURIE_IMU_ZERO_MOTION_DURATION_10_24S = BMI160_ZERO_MOTION_DURATION_10_24S,
    CURIE_IMU_ZERO_MOTION_DURATION_11_52S = BMI160_ZERO_MOTION_DURATION_11_52S,
    CURIE_IMU_ZERO_MOTION_DURATION_12_80S = BMI160_ZERO_MOTION_DURATION_12_80S,
    CURIE_IMU_ZERO_MOTION_DURATION_14_08S = BMI160_ZERO_MOTION_DURATION_14_08S,
    CURIE_IMU_ZERO_MOTION_DURATION_15_36S = BMI160_ZERO_MOTION_DURATION_15_36S,
    CURIE_IMU_ZERO_MOTION_DURATION_16_64S = BMI160_ZERO_MOTION_DURATION_16_64S,
    CURIE_IMU_ZERO_MOTION_DURATION_17_92S = BMI160_ZERO_MOTION_DURATION_17_92S,
    CURIE_IMU_ZERO_MOTION_DURATION_19_20S = BMI160_ZERO_MOTION_DURATION_19_20S,
    CURIE_IMU_ZERO_MOTION_DURATION_20_48S = BMI160_ZERO_MOTION_DURATION_20_48S,
    CURIE_IMU_ZERO_MOTION_DURATION_25_60S = BMI160_ZERO_MOTION_DURATION_25_60S,
    CURIE_IMU_ZERO_MOTION_DURATION_30_72S = BMI160_ZERO_MOTION_DURATION_30_72S,
    CURIE_IMU_ZERO_MOTION_DURATION_35_84S = BMI160_ZERO_MOTION_DURATION_35_84S,
    CURIE_IMU_ZERO_MOTION_DURATION_40_96S = BMI160_ZERO_MOTION_DURATION_40_96S,
    CURIE_IMU_ZERO_MOTION_DURATION_46_08S = BMI160_ZERO_MOTION_DURATION_46_08S,
    CURIE_IMU_ZERO_MOTION_DURATION_51_20S = BMI160_ZERO_MOTION_DURATION_51_20S,
    CURIE_IMU_ZERO_MOTION_DURATION_56_32S = BMI160_ZERO_MOTION_DURATION_56_32S,
    CURIE_IMU_ZERO_MOTION_DURATION_61_44S = BMI160_ZERO_MOTION_DURATION_61_44S,
    CURIE_IMU_ZERO_MOTION_DURATION_66_56S = BMI160_ZERO_MOTION_DURATION_66_56S,
    CURIE_IMU_ZERO_MOTION_DURATION_71_68S = BMI160_ZERO_MOTION_DURATION_71_68S,
    CURIE_IMU_ZERO_MOTION_DURATION_76_80S = BMI160_ZERO_MOTION_DURATION_76_80S,
    CURIE_IMU_ZERO_MOTION_DURATION_81_92S = BMI160_ZERO_MOTION_DURATION_81_92S,
    CURIE_IMU_ZERO_MOTION_DURATION_87_04S = BMI160_ZERO_MOTION_DURATION_87_04S,
    CURIE_IMU_ZERO_MOTION_DURATION_92_16S = BMI160_ZERO_MOTION_DURATION_92_16S,
    CURIE_IMU_ZERO_MOTION_DURATION_97_28S = BMI160_ZERO_MOTION_DURATION_97_28S,
    CURIE_IMU_ZERO_MOTION_DURATION_102_40S = BMI160_ZERO_MOTION_DURATION_102_40S,
    CURIE_IMU_ZERO_MOTION_DURATION_112_64S = BMI160_ZERO_MOTION_DURATION_112_64S,
    CURIE_IMU_ZERO_MOTION_DURATION_122_88S = BMI160_ZERO_MOTION_DURATION_122_88S,
    CURIE_IMU_ZERO_MOTION_DURATION_133_12S = BMI160_ZERO_MOTION_DURATION_133_12S,
    CURIE_IMU_ZERO_MOTION_DURATION_143_36S = BMI160_ZERO_MOTION_DURATION_143_36S,
    CURIE_IMU_ZERO_MOTION_DURATION_153_60S = BMI160_ZERO_MOTION_DURATION_153_60S,
    CURIE_IMU_ZERO_MOTION_DURATION_163_84S = BMI160_ZERO_MOTION_DURATION_163_84S,
    CURIE_IMU_ZERO_MOTION_DURATION_174_08S = BMI160_ZERO_MOTION_DURATION_174_08S,
    CURIE_IMU_ZERO_MOTION_DURATION_184_32S = BMI160_ZERO_MOTION_DURATION_184_32S,
    CURIE_IMU_ZERO_MOTION_DURATION_194_56S = BMI160_ZERO_MOTION_DURATION_194_56S,
    CURIE_IMU_ZERO_MOTION_DURATION_204_80S = BMI160_ZERO_MOTION_DURATION_204_80S,
    CURIE_IMU_ZERO_MOTION_DURATION_215_04S = BMI160_ZERO_MOTION_DURATION_215_04S,
    CURIE_IMU_ZERO_MOTION_DURATION_225_28S = BMI160_ZERO_MOTION_DURATION_225_28S,
    CURIE_IMU_ZERO_MOTION_DURATION_235_52S = BMI160_ZERO_MOTION_DURATION_235_52S,
    CURIE_IMU_ZERO_MOTION_DURATION_245_76S = BMI160_ZERO_MOTION_DURATION_245_76S,
    CURIE_IMU_ZERO_MOTION_DURATION_256_00S = BMI160_ZERO_MOTION_DURATION_256_00S,
    CURIE_IMU_ZERO_MOTION_DURATION_266_24S = BMI160_ZERO_MOTION_DURATION_266_24S,
    CURIE_IMU_ZERO_MOTION_DURATION_276_48S = BMI160_ZERO_MOTION_DURATION_276_48S,
    CURIE_IMU_ZERO_MOTION_DURATION_286_72S = BMI160_ZERO_MOTION_DURATION_286_72S,
    CURIE_IMU_ZERO_MOTION_DURATION_296_96S = BMI160_ZERO_MOTION_DURATION_296_96S,
    CURIE_IMU_ZERO_MOTION_DURATION_307_20S = BMI160_ZERO_MOTION_DURATION_307_20S,
    CURIE_IMU_ZERO_MOTION_DURATION_317_44S = BMI160_ZERO_MOTION_DURATION_317_44S,
    CURIE_IMU_ZERO_MOTION_DURATION_327_68S = BMI160_ZERO_MOTION_DURATION_327_68S,
    CURIE_IMU_ZERO_MOTION_DURATION_337_92S = BMI160_ZERO_MOTION_DURATION_337_92S,
    CURIE_IMU_ZERO_MOTION_DURATION_348_16S = BMI160_ZERO_MOTION_DURATION_348_16S,
    CURIE_IMU_ZERO_MOTION_DURATION_358_40S = BMI160_ZERO_MOTION_DURATION_358_40S,
    CURIE_IMU_ZERO_MOTION_DURATION_368_64S = BMI160_ZERO_MOTION_DURATION_368_64S,
    CURIE_IMU_ZERO_MOTION_DURATION_378_88S = BMI160_ZERO_MOTION_DURATION_378_88S,
    CURIE_IMU_ZERO_MOTION_DURATION_389_12S = BMI160_ZERO_MOTION_DURATION_389_12S,
    CURIE_IMU_ZERO_MOTION_DURATION_399_36S = BMI160_ZERO_MOTION_DURATION_399_36S,
    CURIE_IMU_ZERO_MOTION_DURATION_409_60S = BMI160_ZERO_MOTION_DURATION_409_60S,
    CURIE_IMU_ZERO_MOTION_DURATION_419_84S = BMI160_ZERO_MOTION_DURATION_419_84S,
    CURIE_IMU_ZERO_MOTION_DURATION_430_08S = BMI160_ZERO_MOTION_DURATION_430_08S
} CurieImuZeroMotionDuration;

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

        int getGyroRate();
        void setGyroRate(int rate);

        int getAccelerometerRate();
        void setAccelerometerRate(int rate);

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
