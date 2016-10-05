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

#include "I2SController.h"
#include "variant.h"

uint16_t sampleRateLookupTable[MAX_I2S_SIZE][MAX_I2S_RATE] = {
  //  8K  12k  22k  24k  44k  48k
  { 125,  83,  45,  42,  23,  21},    // I2S_16bit
  {  83,  56,  30,  28,  15,  14},    // I2S_24bit
  {  63,  42,  23,  21,  11,  10}     // I2S_32bit
};

uint8_t sampleSizeLookupTable[MAX_I2S_SIZE] = {16, 24, 32};


I2SController::I2SController()
{
  sampleRateMap = (uint16_t *)sampleRateLookupTable;
  sampleSizeMap = sampleSizeLookupTable;
}


void I2SController::muxTX(bool enable)
{
    int mux_mode = GPIO_MUX_MODE;
    if(enable)
    {
        mux_mode = I2S_MUX_MODE;
    }
    
    /* Set SoC pin mux configuration */
    SET_PIN_MODE(g_APinDescription[I2S_TXD].ulSocPin, mux_mode);
    SET_PIN_MODE(g_APinDescription[I2S_TWS].ulSocPin, mux_mode);
    SET_PIN_MODE(g_APinDescription[I2S_TSCK].ulSocPin, mux_mode);

    // g_APinDescription is residing in ROM now, no update to the structure
    //    g_APinDescription[I2S_TXD].ulPinMode = mux_mode;
    //    g_APinDescription[I2S_TWS].ulPinMode = mux_mode;
    //    g_APinDescription[I2S_TSCK].ulPinMode  = mux_mode;
}


void I2SController::muxRX(bool enable)
{
    int mux_mode = GPIO_MUX_MODE;
    if(enable)
    {
        mux_mode = I2S_MUX_MODE;
    }
    
    /* Set SoC pin mux configuration */
    SET_PIN_MODE(49, mux_mode); //I2S_RXD
    SET_PIN_MODE(51, mux_mode); //I2S_RWS
    SET_PIN_MODE(50,  mux_mode); //I2S_RSCK

    // g_APinDescription is residing in ROM now, no update to the structure
    //    g_APinDescription[I2S_RXD].ulPinMode = mux_mode;
    //    g_APinDescription[I2S_RWS].ulPinMode = mux_mode;
    //    g_APinDescription[I2S_RSCK].ulPinMode  = mux_mode;
}


