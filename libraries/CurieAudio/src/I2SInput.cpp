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

#include "I2SInput.h"
#include "soc_i2s.h"
#include "soc_dma.h"
#include "variant.h"
#include <interrupt.h>

I2SInputClass I2SInput;


void rxi2s_done(void* x)
{
    I2SInput.rxdone_flag = 1;
    if (I2SInput.userCB)
      I2SInput.userCB();
}

void rxi2s_err(void* x)
{
    I2SInput.rxerror_flag = 1;
}


I2SInputClass::I2SInputClass()
{
    rxdone_flag = 1;
    rxerror_flag = 0;
    userCB = NULL;
}


i2sErrorCode I2SInputClass::begin(curieI2sSampleRate sampleRate,
				  curieI2sSampleSize resolution,
				  curieI2sMode mode,
				  uint8_t master)
{
    struct soc_i2s_cfg rxcfg ;

    sampleSize = resolution;
    muxRX(1);
    soc_i2s_init(I2S_CHANNEL_RX);
    soc_dma_init();

    rxcfg.clk_divider = sampleRateMap[(resolution * MAX_I2S_RATE) + sampleRate];
    rxcfg.resolution = sampleSizeMap[resolution];
    rxcfg.mode = (uint8_t)mode;
    rxcfg.master = master;

    rxcfg.cb_done = rxi2s_done;
    rxcfg.cb_err = rxi2s_err; 
    
    if (soc_i2s_config(I2S_CHANNEL_RX, &rxcfg) != DRV_RC_OK)
      return I2S_INIT_FAIL;

    return SUCCESS;
}


void I2SInputClass::end()
{
    soc_i2s_stop_listen();
    muxRX(0);
}


i2sErrorCode I2SInputClass::read(uint8_t buffer[], uint32_t length, uint32_t blocking)
{
    if (rxdone_flag != 0)  return I2S_READ_BUSY;

    rxdone_flag = 0;
    rxerror_flag = 0;

    uint32_t byteAlignment = (sampleSize > I2S_16bit) ? 4 : 2;

    if (soc_i2s_listen((void *)buffer, length, byteAlignment, 0) != DRV_RC_OK)
    {
      rxdone_flag = 1;
      return I2S_READ_FAIL;
    }

    if (blocking == 0)  return SUCCESS;

    while (1)    // use a count to bail.
    {
      if (rxerror_flag == 1)  break;

      if (rxdone_flag == 1)  return SUCCESS;
    }

    rxdone_flag = 1;
    return I2S_READ_FAIL;
}

