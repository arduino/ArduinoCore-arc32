/*
===============================================
BMI160 accelerometer/gyroscope library for Intel(R) Curie(TM) devices.
Copyright (c) 2015 Intel Corporation.  All rights reserved.

Based on MPU6050 Arduino library provided by Jeff Rowberg as part of his
excellent I2Cdev device library: https://github.com/jrowberg/i2cdevlib

===============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _BMI160_H_
#define _BMI160_H_

#include "Arduino.h"

#define BMI160_SPI_READ_BIT         7

#define BMI160_RA_CHIP_ID           0x00

#define BMI160_ACC_PMU_STATUS_BIT   4
#define BMI160_ACC_PMU_STATUS_LEN   2
#define BMI160_GYR_PMU_STATUS_BIT   2
#define BMI160_GYR_PMU_STATUS_LEN   2

#define BMI160_RA_PMU_STATUS        0x03

#define BMI160_RA_GYRO_X_L          0x0C
#define BMI160_RA_GYRO_X_H          0x0D
#define BMI160_RA_GYRO_Y_L          0x0E
#define BMI160_RA_GYRO_Y_H          0x0F
#define BMI160_RA_GYRO_Z_L          0x10
#define BMI160_RA_GYRO_Z_H          0x11
#define BMI160_RA_ACCEL_X_L         0x12
#define BMI160_RA_ACCEL_X_H         0x13
#define BMI160_RA_ACCEL_Y_L         0x14
#define BMI160_RA_ACCEL_Y_H         0x15
#define BMI160_RA_ACCEL_Z_L         0x16
#define BMI160_RA_ACCEL_Z_H         0x17

#define BMI160_STATUS_FOC_RDY       3
#define BMI160_STATUS_NVM_RDY       4
#define BMI160_STATUS_DRDY_GYR      6
#define BMI160_STATUS_DRDY_ACC      7

#define BMI160_RA_STATUS            0x1B

#define BMI160_STEP_INT_BIT         0
#define BMI160_ANYMOTION_INT_BIT    2
#define BMI160_D_TAP_INT_BIT        4
#define BMI160_S_TAP_INT_BIT        5
#define BMI160_NOMOTION_INT_BIT     7
#define BMI160_FFULL_INT_BIT        5
#define BMI160_DRDY_INT_BIT         4
#define BMI160_LOW_G_INT_BIT        3
#define BMI160_HIGH_G_INT_BIT       2

#define BMI160_TAP_SIGN_BIT         7
#define BMI160_TAP_1ST_Z_BIT        6
#define BMI160_TAP_1ST_Y_BIT        5
#define BMI160_TAP_1ST_X_BIT        4

#define BMI160_ANYMOTION_SIGN_BIT   3
#define BMI160_ANYMOTION_1ST_Z_BIT  2
#define BMI160_ANYMOTION_1ST_Y_BIT  1
#define BMI160_ANYMOTION_1ST_X_BIT  0

#define BMI160_HIGH_G_SIGN_BIT      3
#define BMI160_HIGH_G_1ST_Z_BIT     2
#define BMI160_HIGH_G_1ST_Y_BIT     1
#define BMI160_HIGH_G_1ST_X_BIT     0

#define BMI160_RA_INT_STATUS_0      0x1C
#define BMI160_RA_INT_STATUS_1      0x1D
#define BMI160_RA_INT_STATUS_2      0x1E
#define BMI160_RA_INT_STATUS_3      0x1F

#define BMI160_RA_TEMP_L            0x20
#define BMI160_RA_TEMP_H            0x21

#define BMI160_RA_FIFO_LENGTH_0     0x22
#define BMI160_RA_FIFO_LENGTH_1     0x23

#define BMI160_FIFO_DATA_INVALID    0x80
#define BMI160_RA_FIFO_DATA         0x24

#define BMI160_ACCEL_RATE_SEL_BIT    0
#define BMI160_ACCEL_RATE_SEL_LEN    4

#define BMI160_RA_ACCEL_CONF        0X40
#define BMI160_RA_ACCEL_RANGE       0X41

#define BMI160_GYRO_RATE_SEL_BIT    0
#define BMI160_GYRO_RATE_SEL_LEN    4

#define BMI160_RA_GYRO_CONF         0X42
#define BMI160_RA_GYRO_RANGE        0X43

#define BMI160_FIFO_HEADER_EN_BIT   4
#define BMI160_FIFO_ACC_EN_BIT      6
#define BMI160_FIFO_GYR_EN_BIT      7

#define BMI160_RA_FIFO_CONFIG_0     0x46
#define BMI160_RA_FIFO_CONFIG_1     0x47

#define BMI160_ANYMOTION_EN_BIT     0
#define BMI160_ANYMOTION_EN_LEN     3
#define BMI160_D_TAP_EN_BIT         4
#define BMI160_S_TAP_EN_BIT         5
#define BMI160_NOMOTION_EN_BIT      0
#define BMI160_NOMOTION_EN_LEN      3
#define BMI160_LOW_G_EN_BIT         3
#define BMI160_LOW_G_EN_LEN         1
#define BMI160_HIGH_G_EN_BIT        0
#define BMI160_HIGH_G_EN_LEN        3

#define BMI160_STEP_EN_BIT          3
#define BMI160_DRDY_EN_BIT          4
#define BMI160_FFULL_EN_BIT         5

#define BMI160_RA_INT_EN_0          0x50
#define BMI160_RA_INT_EN_1          0x51
#define BMI160_RA_INT_EN_2          0x52

#define BMI160_INT1_EDGE_CTRL       0
#define BMI160_INT1_LVL             1
#define BMI160_INT1_OD              2
#define BMI160_INT1_OUTPUT_EN       3

#define BMI160_RA_INT_OUT_CTRL      0x53

#define BMI160_LATCH_MODE_BIT       0
#define BMI160_LATCH_MODE_LEN       4

#define BMI160_LATCH_MODE_NONE      0
#define BMI160_LATCH_MODE_312_5_US  1
#define BMI160_LATCH_MODE_625_US    2
#define BMI160_LATCH_MODE_1_25_MS   3
#define BMI160_LATCH_MODE_2_5_MS    4
#define BMI160_LATCH_MODE_5_MS      5
#define BMI160_LATCH_MODE_10_MS     6
#define BMI160_LATCH_MODE_20_MS     7
#define BMI160_LATCH_MODE_40_MS     8
#define BMI160_LATCH_MODE_80_MS     9
#define BMI160_LATCH_MODE_160_MS    10
#define BMI160_LATCH_MODE_320_MS    11
#define BMI160_LATCH_MODE_640_MS    12
#define BMI160_LATCH_MODE_1_28_S    13
#define BMI160_LATCH_MODE_2_56_S    14
#define BMI160_LATCH_MODE_LATCH     15

#define BMI160_RA_INT_LATCH         0x54
#define BMI160_RA_INT_MAP_0         0x55
#define BMI160_RA_INT_MAP_1         0x56
#define BMI160_RA_INT_MAP_2         0x57

#define BMI160_ANYMOTION_DUR_BIT    0
#define BMI160_ANYMOTION_DUR_LEN    2
#define BMI160_NOMOTION_DUR_BIT     2
#define BMI160_NOMOTION_DUR_LEN     6

#define BMI160_NOMOTION_SEL_BIT     0
#define BMI160_NOMOTION_SEL_LEN     1

#define BMI160_RA_INT_LOWHIGH_0     0x5A
#define BMI160_RA_INT_LOWHIGH_1     0x5B
#define BMI160_RA_INT_LOWHIGH_2     0x5C
#define BMI160_RA_INT_LOWHIGH_3     0x5D
#define BMI160_RA_INT_LOWHIGH_4     0x5E

#define BMI160_RA_INT_MOTION_0      0x5F
#define BMI160_RA_INT_MOTION_1      0x60
#define BMI160_RA_INT_MOTION_2      0x61
#define BMI160_RA_INT_MOTION_3      0x62

#define BMI160_TAP_DUR_BIT          0
#define BMI160_TAP_DUR_LEN          3
#define BMI160_TAP_SHOCK_BIT        6
#define BMI160_TAP_QUIET_BIT        7
#define BMI160_TAP_THRESH_BIT       0
#define BMI160_TAP_THRESH_LEN       5

#define BMI160_RA_INT_TAP_0         0x63
#define BMI160_RA_INT_TAP_1         0x64

#define BMI160_FOC_ACC_Z_BIT        0
#define BMI160_FOC_ACC_Z_LEN        2
#define BMI160_FOC_ACC_Y_BIT        2
#define BMI160_FOC_ACC_Y_LEN        2
#define BMI160_FOC_ACC_X_BIT        4
#define BMI160_FOC_ACC_X_LEN        2
#define BMI160_FOC_GYR_EN           6

#define BMI160_RA_FOC_CONF          0x69

#define BMI160_GYR_OFFSET_X_MSB_BIT 0
#define BMI160_GYR_OFFSET_X_MSB_LEN 2
#define BMI160_GYR_OFFSET_Y_MSB_BIT 2
#define BMI160_GYR_OFFSET_Y_MSB_LEN 2
#define BMI160_GYR_OFFSET_Z_MSB_BIT 4
#define BMI160_GYR_OFFSET_Z_MSB_LEN 2
#define BMI160_ACC_OFFSET_EN        6
#define BMI160_GYR_OFFSET_EN        7

#define BMI160_RA_OFFSET_0          0x71
#define BMI160_RA_OFFSET_1          0x72
#define BMI160_RA_OFFSET_2          0x73
#define BMI160_RA_OFFSET_3          0x74
#define BMI160_RA_OFFSET_4          0x75
#define BMI160_RA_OFFSET_5          0x76
#define BMI160_RA_OFFSET_6          0x77

#define BMI160_RA_STEP_CNT_L        0x78
#define BMI160_RA_STEP_CNT_H        0x79

#define BMI160_STEP_BUF_MIN_BIT     0
#define BMI160_STEP_BUF_MIN_LEN     3
#define BMI160_STEP_CNT_EN_BIT      3

#define BMI160_STEP_TIME_MIN_BIT    0
#define BMI160_STEP_TIME_MIN_LEN    3
#define BMI160_STEP_THRESH_MIN_BIT  3
#define BMI160_STEP_THRESH_MIN_LEN  2
#define BMI160_STEP_ALPHA_BIT       5
#define BMI160_STEP_ALPHA_LEN       3

#define BMI160_RA_STEP_CONF_0       0x7A
#define BMI160_RA_STEP_CONF_1       0x7B

#define BMI160_GYRO_RANGE_SEL_BIT   0
#define BMI160_GYRO_RANGE_SEL_LEN   3

/* Gyroscope Full-Scale Range Options */
#define BMI160_GYRO_RANGE_125       0x04
#define BMI160_GYRO_RANGE_250       0x03
#define BMI160_GYRO_RANGE_500       0x02
#define BMI160_GYRO_RANGE_1000      0x01
#define BMI160_GYRO_RANGE_2000      0x00

#define BMI160_GYRO_RATE_SEL_BIT    0
#define BMI160_GYRO_RATE_SEL_LEN    4

/* Gyroscope Output Data Rate Options */
#define BMI160_GYRO_RATE_25HZ       0x06
#define BMI160_GYRO_RATE_50HZ       0x07
#define BMI160_GYRO_RATE_100HZ      0x08
#define BMI160_GYRO_RATE_200HZ      0x09
#define BMI160_GYRO_RATE_400HZ      0x0A
#define BMI160_GYRO_RATE_800HZ      0x0B
#define BMI160_GYRO_RATE_1600HZ     0x0C
#define BMI160_GYRO_RATE_3200HZ     0x0D

#define BMI160_GYRO_DLPF_SEL_BIT    4
#define BMI160_GYRO_DLPF_SEL_LEN    2

#define BMI160_ACCEL_DLPF_SEL_BIT   4
#define BMI160_ACCEL_DLPF_SEL_LEN   3

/* Gyroscope Digital Low-Pass Filter Mode Options */
#define BMI160_DLPF_MODE_NORM       0x2
#define BMI160_DLPF_MODE_OSR2       0x1
#define BMI160_DLPF_MODE_OSR4       0x0

#define BMI160_ACCEL_RANGE_SEL_BIT  0
#define BMI160_ACCEL_RANGE_SEL_LEN  4

/* Accelerometer Range Options */
#define BMI160_ACCEL_RANGE_2G       0X03
#define BMI160_ACCEL_RANGE_4G       0X05
#define BMI160_ACCEL_RANGE_8G       0X08
#define BMI160_ACCEL_RANGE_16G      0X0C

/* Accelerometer Output Data Rate Options */
#define BMI160_ACCEL_RATE_25_2HZ    0x05
#define BMI160_ACCEL_RATE_25HZ      0x06
#define BMI160_ACCEL_RATE_50HZ      0x07
#define BMI160_ACCEL_RATE_100HZ     0x08
#define BMI160_ACCEL_RATE_200HZ     0x09
#define BMI160_ACCEL_RATE_400HZ     0x0A
#define BMI160_ACCEL_RATE_800HZ     0x0B
#define BMI160_ACCEL_RATE_1600HZ    0x0C
#define BMI160_ACCEL_RATE_3200HZ    0x0D

#define BMI160_CMD_START_FOC        0x03
#define BMI160_CMD_ACC_MODE_NORMAL  0x11
#define BMI160_CMD_GYR_MODE_NORMAL  0x15
#define BMI160_CMD_FIFO_FLUSH       0xB0
#define BMI160_CMD_INT_RESET        0xB1
#define BMI160_CMD_STEP_CNT_CLR     0xB2
#define BMI160_CMD_SOFT_RESET       0xB6

#define BMI160_RA_CMD               0x7E

typedef enum {
    BMI160_STEP_MODE_NORMAL = 0,
    BMI160_STEP_MODE_SENSITIVE,
    BMI160_STEP_MODE_ROBUST,
} BMI160StepMode;

typedef enum {
    BMI160_TAP_SHOCK_DURATION_50MS = 0,
    BMI160_TAP_SHOCK_DURATION_75MS,
} BMI160TapShockDuration;

typedef enum {
    BMI160_TAP_QUIET_DURATION_30MS = 0,
    BMI160_TAP_QUIET_DURATION_20MS,
} BMI160TapQuietDuration;

typedef enum {
    BMI160_DOUBLE_TAP_DURATION_50MS = 0,
    BMI160_DOUBLE_TAP_DURATION_100MS,
    BMI160_DOUBLE_TAP_DURATION_150MS,
    BMI160_DOUBLE_TAP_DURATION_200MS,
    BMI160_DOUBLE_TAP_DURATION_250MS,
    BMI160_DOUBLE_TAP_DURATION_375MS,
    BMI160_DOUBLE_TAP_DURATION_500MS,
    BMI160_DOUBLE_TAP_DURATION_700MS,
} BMI160DoubleTapDuration;

class BMI160Class {
    public:
        void initialize();
        bool testConnection();

        uint8_t getGyroRate();
        void setGyroRate(uint8_t rate);

        uint8_t getAccelRate();
        void setAccelRate(uint8_t rate);

        uint8_t getGyroDLPFMode();
        void setGyroDLPFMode(uint8_t bandwidth);

        uint8_t getAccelDLPFMode();
        void setAccelDLPFMode(uint8_t bandwidth);

        uint8_t getFullScaleGyroRange();
        void setFullScaleGyroRange(uint8_t range);
        uint8_t getFullScaleAccelRange();
        void setFullScaleAccelRange(uint8_t range);


#if 0
		// SELF_TEST registers
		uint8_t getAccelXSelfTestFactoryTrim();
		uint8_t getAccelYSelfTestFactoryTrim();
		uint8_t getAccelZSelfTestFactoryTrim();

		uint8_t getGyroXSelfTestFactoryTrim();
		uint8_t getGyroYSelfTestFactoryTrim();
		uint8_t getGyroZSelfTestFactoryTrim();
		
        // ACCEL_CONFIG register
        bool getAccelXSelfTest();
        void setAccelXSelfTest(bool enabled);
        bool getAccelYSelfTest();
        void setAccelYSelfTest(bool enabled);
        bool getAccelZSelfTest();
        void setAccelZSelfTest(bool enabled);
#endif

        void autoCalibrateGyroOffset();
        bool getGyroOffsetEnabled();
        void setGyroOffsetEnabled(bool enabled);

        int16_t getXGyroOffset();
        void setXGyroOffset(int16_t offset);
        int16_t getYGyroOffset();
        void setYGyroOffset(int16_t offset);
        int16_t getZGyroOffset();
        void setZGyroOffset(int16_t offset);

        void autoCalibrateXAccelOffset(int target);
        void autoCalibrateYAccelOffset(int target);
        void autoCalibrateZAccelOffset(int target);
        bool getAccelOffsetEnabled();
        void setAccelOffsetEnabled(bool enabled);

        int8_t getXAccelOffset();
        void setXAccelOffset(int8_t offset);
        int8_t getYAccelOffset();
        void setYAccelOffset(int8_t offset);
        int8_t getZAccelOffset();
        void setZAccelOffset(int8_t offset);

        uint8_t getFreefallDetectionThreshold();
        void setFreefallDetectionThreshold(uint8_t threshold);

        uint8_t getFreefallDetectionDuration();
        void setFreefallDetectionDuration(uint8_t duration);

        uint8_t getShockDetectionThreshold();
        void setShockDetectionThreshold(uint8_t threshold);

        uint8_t getShockDetectionDuration();
        void setShockDetectionDuration(uint8_t duration);

        uint8_t getMotionDetectionThreshold();
        void setMotionDetectionThreshold(uint8_t threshold);

        uint8_t getMotionDetectionDuration();
        void setMotionDetectionDuration(uint8_t duration);

        uint8_t getZeroMotionDetectionThreshold();
        void setZeroMotionDetectionThreshold(uint8_t threshold);

        unsigned getZeroMotionDetectionDuration();
        void setZeroMotionDetectionDuration(unsigned duration);

        uint8_t getTapDetectionThreshold();
        void setTapDetectionThreshold(uint8_t threshold);

        bool getTapShockDuration();
        void setTapShockDuration(bool duration);

        bool getTapQuietDuration();
        void setTapQuietDuration(bool duration);

        uint8_t getDoubleTapDetectionDuration();
        void setDoubleTapDetectionDuration(uint8_t duration);

        void setStepDetectionMode(BMI160StepMode mode);
        bool getStepCountEnabled();
        void setStepCountEnabled(bool enabled);
        uint16_t getStepCount();
        void resetStepCount();

        bool getIntFreefallEnabled();
        void setIntFreefallEnabled(bool enabled);
        bool getIntShockEnabled();
        void setIntShockEnabled(bool enabled);
        bool getIntStepEnabled();
        void setIntStepEnabled(bool enabled);
        bool getIntMotionEnabled();
        void setIntMotionEnabled(bool enabled);
        bool getIntZeroMotionEnabled();
        void setIntZeroMotionEnabled(bool enabled);
        bool getIntTapEnabled();
        void setIntTapEnabled(bool enabled);
        bool getIntDoubleTapEnabled();
        void setIntDoubleTapEnabled(bool enabled);

        bool getGyroFIFOEnabled();
        void setGyroFIFOEnabled(bool enabled);
        bool getAccelFIFOEnabled();
        void setAccelFIFOEnabled(bool enabled);

        bool getIntFIFOBufferFullEnabled();
        void setIntFIFOBufferFullEnabled(bool enabled);
        bool getIntDataReadyEnabled();
        void setIntDataReadyEnabled(bool enabled);

        uint8_t getIntStatus0();
        uint8_t getIntStatus1();
        uint8_t getIntStatus2();
        uint8_t getIntStatus3();
        bool getIntFreefallStatus();
        bool getIntShockStatus();
        bool getIntStepStatus();
        bool getIntMotionStatus();
        bool getIntZeroMotionStatus();
        bool getIntTapStatus();
        bool getIntDoubleTapStatus();
        bool getIntFIFOBufferFullStatus();
        bool getIntDataReadyStatus();

        void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
        void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
        int16_t getAccelerationX();
        int16_t getAccelerationY();
        int16_t getAccelerationZ();

        int16_t getTemperature();

        void getRotation(int16_t* x, int16_t* y, int16_t* z);
        int16_t getRotationX();
        int16_t getRotationY();
        int16_t getRotationZ();

        bool getXNegShockDetected();
        bool getXPosShockDetected();
        bool getYNegShockDetected();
        bool getYPosShockDetected();
        bool getZNegShockDetected();
        bool getZPosShockDetected();
        bool getZeroShockDetected();

        bool getXNegMotionDetected();
        bool getXPosMotionDetected();
        bool getYNegMotionDetected();
        bool getYPosMotionDetected();
        bool getZNegMotionDetected();
        bool getZPosMotionDetected();
        bool getZeroMotionDetected();

        bool getXNegTapDetected();
        bool getXPosTapDetected();
        bool getYNegTapDetected();
        bool getYPosTapDetected();
        bool getZNegTapDetected();
        bool getZPosTapDetected();

        bool getFIFOHeaderModeEnabled();
        void setFIFOHeaderModeEnabled(bool enabled);
        void resetFIFO();

        uint16_t getFIFOCount();
        void getFIFOBytes(uint8_t *data, uint16_t length);

        uint8_t getDeviceID();

        uint8_t getRegister(uint8_t reg);
        void setRegister(uint8_t reg, uint8_t data);

        /* TODO - consider making these protected - not sure if we want to allow use from a sketch
         * except perhaps for the latch control functions
         */
        bool getIntEnabled();
        void setIntEnabled(bool enabled);
        bool getInterruptMode();
        void setInterruptMode(bool mode);
        bool getInterruptDrive();
        void setInterruptDrive(bool drive);
        uint8_t getInterruptLatch();
        void setInterruptLatch(uint8_t latch);
        void resetInterrupt();

    protected:
        virtual int serial_buffer_transfer(uint8_t *buf, unsigned tx_cnt, unsigned rx_cnt);


    private:
        uint8_t reg_read (uint8_t reg);
        void reg_write(uint8_t reg, uint8_t data);
        void reg_write_bits(uint8_t reg, uint8_t data, unsigned pos, unsigned len);
        uint8_t reg_read_bits(uint8_t reg, unsigned pos, unsigned len);

        uint8_t buffer[12];
};

#endif /* _BMI160_H_ */
