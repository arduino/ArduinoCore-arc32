/*
  I2S Output API
  Copyright (c) 2016 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#ifndef ARDUINO_I2S_OUTPUT
#define ARDUINO_I2S_OUTPUT

#include "I2SController.h"

class I2SOutputClass : public I2SController
{
  friend void i2sTxDoneCB(void *x);
  friend void i2sPingPongTxCB(void *x);
  friend void i2sTxErrorCB(void *x);

  public:
    I2SOutputClass();

    i2sErrorCode begin(curieI2sSampleRate sample_rate,
		       curieI2sSampleSize resolution,
		       curieI2sMode mode,
		       uint8_t master = 1);
    void end();

    // Write one audio sample for each channel. Samples are temporarily stored in a buffer
    // until it is full, it will be dumped to the I2S output.  Setting the flush flag
    // will fill the unused space in the temperary buffer with 0 (mute) and cause it to
    // be dumped to I2S output.
    i2sErrorCode write(int32_t leftSample, int32_t rightSample,
		       uint16_t flush = 0);

    // This is an one time dumping of a buffer to the I2S output. The I2S bus is halted
    // upon completion.
    i2sErrorCode write(int32_t buffer[], int bufferLength,
		       uint32_t blocking = 1);

    inline uint8_t txDone(void) { return txdone_flag; }

    int txError(void);

  private:
    curieI2sSampleSize sampleSize;
    uint8_t txdone_flag;
    uint8_t txerror_flag;
    int txErrorCount;
};

extern I2SOutputClass I2SOutput;

#endif
