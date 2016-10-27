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

const int numPpBuffer = 2;
const int NUMBER_OF_SAMPLES = 256;

static struct pingpongstruct {
  int32_t buf[NUMBER_OF_SAMPLES];  // Left and right samples
  uint16_t empty;
} ppBuffer[numPpBuffer];

static int i2sRunning = 0;
static int sendIndex = 0;
static int fillIndex = 0;

I2SOutputClass I2SOutput;


void i2sTxDoneCB(void *x)
{
  I2SOutput.txdone_flag = 1;
  i2sRunning = 0;
}

void i2sPingPongTxCB(void *x)
{
  ppBuffer[sendIndex].empty = 1;
  sendIndex = (sendIndex + 1) & 0x01;

  if (ppBuffer[sendIndex].empty) {
    I2SOutput.txdone_flag = 1;
    soc_i2s_stop_transmit();
    i2sRunning = 0;
  }
}

void i2sTxErrorCB(void *x)
{
  I2SOutput.txErrorCount++;
  I2SOutput.txerror_flag = 1;
  i2sRunning = 0;
}


I2SOutputClass::I2SOutputClass()
{
  txdone_flag = 1;
  txErrorCount = 0;

  for (int i=0; i<numPpBuffer; i++) {
    ppBuffer[0].empty = ppBuffer[1].empty = 1;
  }
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

    txcfg.cb_done = i2sPingPongTxCB;
    txcfg.cb_err = i2sTxErrorCB;

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
    soc_i2s_stop_transmit();
    i2sRunning = 0;
    muxTX(0);
}


static i2sErrorCode kickOff(void)
{
  void *bufPtrArray[numPpBuffer];
  int i, j;

  for (i=0, j=sendIndex; i<numPpBuffer; i++) {
    bufPtrArray[i] = ppBuffer[j++].buf;
    if (j >= numPpBuffer)
      j = 0;
  }

  if (soc_i2s_transmit_loop(bufPtrArray, numPpBuffer, sizeof(ppBuffer[0].buf),
			    sizeof(int32_t)) != DRV_RC_OK) {
    return I2S_WRITE_DRIVER_FAIL;
  }
  i2sRunning = 1;
  return SUCCESS;
}


i2sErrorCode I2SOutputClass::write(int32_t leftSample, int32_t rightSample,
				   uint16_t flush)
{
  static int bufIndex = 0;
  int16_t nextBuffer;
  i2sErrorCode result = SUCCESS;

  ppBuffer[fillIndex].buf[bufIndex++] = leftSample;
  ppBuffer[fillIndex].buf[bufIndex++] = rightSample;

  if (flush) {
    while (bufIndex < NUMBER_OF_SAMPLES)
      ppBuffer[fillIndex].buf[bufIndex++] = 0;
  }

  if (bufIndex >= NUMBER_OF_SAMPLES) {
    // Buffer is filled
    ppBuffer[fillIndex].empty = 0;
    nextBuffer = (fillIndex + 1) & 0x01;

    while (bufIndex >= NUMBER_OF_SAMPLES) {
      if (!i2sRunning) {
	if ((result = kickOff()) != SUCCESS)
	  break;
      }

      if (ppBuffer[nextBuffer].empty == 1) {
	fillIndex = nextBuffer;
	bufIndex = 0;
      }
    }
  }
  return result;
}


i2sErrorCode I2SOutputClass::write(int32_t buffer[], int bufferLength,
				   uint32_t blocking)
{
  //    int count = 0;

    if (txdone_flag == 0)  return I2S_WRITE_BUSY;

    txerror_flag = txdone_flag = 0;

    if (soc_i2s_write((void *)buffer, bufferLength, sizeof(int32_t)) != DRV_RC_OK) {
      txdone_flag = 1;
      return I2S_WRITE_DRIVER_FAIL;
    }

    if (blocking == 0)  return SUCCESS;

    while (1)  //  (count++ < 100000)  Need to bail sometime
    {
      if (txerror_flag == 1)  return I2S_WRITE_FAIL;

      if (txdone_flag == 1)   return SUCCESS;
    }
}


int I2SOutputClass::txError(void)
{
  int tmp = txErrorCount;
  txErrorCount = 0;
  return tmp;
}




