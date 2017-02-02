/*
  I2S Input API
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

#ifndef ARDUINO_I2S_INPUT
#define ARDUINO_I2S_INPUT

#include <I2SController.h>

class I2SInputClass : public I2SController
{
  friend void rxi2s_done(void* x);
  friend void rxi2s_err(void* x);

  public:
    I2SInputClass();

    i2sErrorCode begin(curieI2sSampleRate sample_rate,
		       curieI2sSampleSize resolution,
		       curieI2sMode mode,
		       uint8_t master = 1);
    void end();

    i2sErrorCode read(uint8_t buffer[], uint32_t length, uint32_t blocking = 1);
 
   void attachInterrupt(void (*userCallBack)(void))
    {
      userCB = userCallBack;
    };

  private:
    curieI2sSampleSize sampleSize;
    uint8_t rxdone_flag;
    uint8_t rxerror_flag;
    void (*userCB)(void);
};

extern I2SInputClass I2SInput;

#endif
