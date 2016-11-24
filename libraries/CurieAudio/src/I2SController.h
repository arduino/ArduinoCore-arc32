//***************************************************************
//
// Copyright (c) 2016 Intel Corporation.  All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
//***************************************************************

#ifndef _I2S_CONTROLLER_H_
#define _I2S_CONTROLLER_H_

#include <Arduino.h>

// The following Arduino 101 pins are used for I2S Transmit (Tx) interface.
#define I2S_TXD     7        // Data
#define I2S_TWS     4        // Word Select
#define I2S_TSCK    2        // Clock

// The following Arduino 101 pins are used for I2S Receive (Rx) interface.
#define I2S_RXD     5
#define I2S_RWS     3
#define I2S_RSCK    8

// These modes are suppported by the Curie I2S:  I2S_MODE_PHILLIPS, 
// I2S_MODE_RJ (right justified), I2S_MODE_LJ (left jst), I2S_MODE_DSP.
#define I2S_MODE_SCK_POL    (0x01)
#define I2S_MODE_WS_POL     (0x02)
#define I2S_MODE_LR_ALIGN   (0x08)
#define I2S_MODE_SAMPLE_DEL (0x10)
#define I2S_MODE_WS_DSP     (0x20)

typedef enum {
  I2S_MODE_PHILLIPS = (I2S_MODE_LR_ALIGN),
  I2S_MODE_RJ = (I2S_MODE_WS_POL | I2S_MODE_SAMPLE_DEL),
  I2S_MODE_LJ = (I2S_MODE_WS_POL | I2S_MODE_LR_ALIGN | I2S_MODE_SAMPLE_DEL),
  I2S_MODE_DSP = (I2S_MODE_LR_ALIGN | I2S_MODE_WS_DSP)
} curieI2sMode;

// These are the supported sampling rate for the Curie I2s interface.
typedef enum {
  I2S_8KHZ = 0,
  I2S_12KHZ,
  I2S_22KHZ,
  I2S_24KHZ,
  I2S_44KHZ,
  I2S_48KHZ,
  MAX_I2S_RATE
} curieI2sSampleRate;

// These are the supported sample size for the Curie I2S interface,
typedef enum {
  I2S_16bit = 0,
  I2S_24bit,
  I2S_32bit,
  MAX_I2S_SIZE
} curieI2sSampleSize;

// This is a list of return error code.
typedef enum {
  SUCCESS = 0,
  I2S_INIT_FAIL,             // I2S controller initialization failure.
  I2S_WRITE_BUSY,            // Previous operation not completed when I2S write is called.
  I2S_WRITE_FAIL,            // I2S controller write operation failure.
  I2S_READ_BUSY,             // Previous operation not completed when I2S read is called.
  I2S_READ_FAIL,             // I2S controller read operation failure.
  I2S_MISC_ERROR
} i2sErrorCode;


class I2SController
{
    public:
        I2SController();

        //
        virtual i2sErrorCode begin(curieI2sSampleRate sample_rate,
				   curieI2sSampleSize resolution,
				   curieI2sMode mode,
				   uint8_t master) = 0;

	//
        virtual void end() = 0;

    protected:
        // mux/demux the i2s rx pins into i2s mode
        void muxRX(bool enable);
        
        // mux/demux the i2s tx pins into i2s mode
        void muxTX(bool enable);

	uint16_t *sampleRateMap;
	uint8_t *sampleSizeMap;
};


#endif
