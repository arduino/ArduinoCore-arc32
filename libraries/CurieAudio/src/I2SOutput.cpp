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

#include "I2SOutput.h"
#include "soc_i2s.h"
#include "soc_dma.h"
#include "variant.h"
#include <interrupt.h>

I2SOutputClass I2SOutput;


void txi2s_done(void* x)
{
  I2SOutput.txdone_flag = 1;
  if (I2SOutput.userCB)
    I2SOutput.userCB(0);
}

void txi2s_err(void* x)
{
  I2SOutput.txerror_flag = 1;
  if (I2SOutput.userCB)
    I2SOutput.userCB(1);
}

I2SOutputClass::I2SOutputClass()
{
  txdone_flag = 1;
  txerror_flag = 0;
  userCB = NULL;
  //  frameDelay =  5;
}


i2sErrorCode I2SOutputClass::begin(curieI2sSampleRate sampleRate,
				   curieI2sSampleSize resolution,
				   curieI2sMode mode,
				   uint8_t master)
{
    struct soc_i2s_cfg txcfg;

    sampleSize = resolution;
    muxTX(1);
    soc_i2s_init(I2S_CHANNEL_TX);
    soc_dma_init();

    txcfg.clk_divider = sampleRateMap[(resolution * MAX_I2S_RATE) + sampleRate];
    txcfg.resolution = sampleSizeMap[resolution];
    txcfg.mode = (uint8_t)mode;
    txcfg.master = master;

    txcfg.cb_done = txi2s_done;
    txcfg.cb_err = txi2s_err;

#ifdef FOR_DEBUGGING
    Serial.print("Sample rate divider: ");
    Serial.println(txcfg.clk_divider);
    Serial.print("Sample resolution: ");
    Serial.println(txcfg.resolution);
    Serial.print("Mode: ");
    Serial.println(mode);
#endif

    if (soc_i2s_config(I2S_CHANNEL_TX, &txcfg) != DRV_RC_OK)
      return I2S_INIT_FAIL;

    return SUCCESS;
}


void I2SOutputClass::end()
{
    soc_i2s_stop_stream();
    muxTX(0);
}


i2sErrorCode I2SOutputClass::write(int32_t buffer[], uint32_t bufferLength,
				   uint32_t bytePerSample, uint32_t blocking)
{
  //    int count = 0;

    if (txdone_flag == 0)  return I2S_WRITE_BUSY;

    txdone_flag = 0;
    txerror_flag = 0;

    if (soc_i2s_stream((void *)buffer, bufferLength, bytePerSample, 0) != DRV_RC_OK)
    {
      txdone_flag = 1;
      return I2S_WRITE_FAIL;
    }

    if (blocking == 0)  return SUCCESS;

    while (1)  //  (count++ < 100000)  Need to bail sometime
    {
      if (txerror_flag == 1)  break;

      if (txdone_flag == 1)   return SUCCESS;
    }

    txerror_flag = 1;
    return I2S_WRITE_FAIL;
}



