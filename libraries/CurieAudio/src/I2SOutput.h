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
  friend void txi2s_done(void* x);
  friend void txi2s_err(void* x);

  public:
    I2SOutputClass();

    i2sErrorCode begin(curieI2sSampleRate sample_rate,
		       curieI2sSampleSize resolution,
		       curieI2sMode mode,
		       uint8_t master = 1);
    void end();

    i2sErrorCode write(int32_t buffer[], uint32_t bufferLength,
		       uint32_t bytePerSample, uint32_t blocking = 1);

    void attachCallback(void (*userCallBack)(uint16_t result))
    {
      userCB = userCallBack;
    };

  private:
    curieI2sSampleSize sampleSize;
    uint8_t txdone_flag;
    uint8_t txerror_flag;
    void (*userCB)(uint16_t result);
};

extern I2SOutputClass I2SOutput;

#endif
