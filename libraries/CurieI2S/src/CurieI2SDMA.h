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

//CurieI2SDMA.h

#ifndef _CURIEI2SDMA_H_
#define _CURIEI2SDMA_H_

#include <Arduino.h>

//I2S Arduino Pins
#define I2S_TXD     7
#define I2S_TWS     4
#define I2S_TSCK    2
#define I2S_RXD     5
#define I2S_RWS     3
#define I2S_RSCK    8

#define I2S_DMA_OK 0
#define I2S_DMA_FAIL 1
class Curie_I2SDMA
{
	private:
		// mux/demux the i2s rx pins into i2s mode
		void muxRX(bool enable);
        
		// mux/demux the i2s tx pins into i2s mode
		void muxTX(bool enable);
	
	public:
		Curie_I2SDMA();

		void lastFrameDelay();
    
		//
		int beginTX(uint16_t sample_rate,uint8_t resolution,uint8_t master,uint8_t mode);
		//
		int beginRX(uint16_t sample_rate,uint8_t resolution,uint8_t master,uint8_t mode);
        
		// initializes i2s interface
		int iniTX();
		//void transTX();
		int iniRX();

		// starts transmission of data to the tx channel
		int transTX(void* buf_TX,uint32_t len,uint32_t len_per_data);

		// starts listening to the rx channel
		int transRX(void* buf_RX,uint32_t len,uint32_t len_per_data);

		// merge data of left and right channel into one buffer
		int mergeData(void* buf_left,void* buf_right,void* buf_TX,uint32_t length_TX,uint32_t len_per_data);
        
		// seperate the data to left and right channl
		int separateData(void* buf_left,void* buf_right,void* buf_RX,uint32_t length_RX,uint32_t len_per_data);        
        	
		//
		void stopTX();
		//
        void stopRX();

};

extern Curie_I2SDMA CurieI2SDMA;

#endif
