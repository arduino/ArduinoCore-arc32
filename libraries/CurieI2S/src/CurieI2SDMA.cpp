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

//CurieI2SDMA.cpp

#include "CurieI2SDMA.h"
#include "soc_i2s.h"
#include "soc_dma.h"
#include "variant.h"
#include <interrupt.h>

static void txi2s_done(void* x);
static void rxi2s_done(void* x);
static void txi2s_err(void* x);
static void rxi2s_err(void* x);

volatile uint8_t txdone_flag = 0;
volatile uint8_t txerror_flag = 0;

volatile uint8_t rxdone_flag = 0;
volatile uint8_t rxerror_flag = 0;
uint8_t frameDelay = 0;

static void txi2s_done(void* x)
{
	CurieI2SDMA.lastFrameDelay();
	txdone_flag = 1;

	return;
}

static void rxi2s_done(void* x)
{
	rxdone_flag = 1;

	return;
}

static void txi2s_err(void* x)
{
	txerror_flag = 1;

	return;
}

static void rxi2s_err(void* x)
{
	rxerror_flag = 1;
    
	return;
}

struct soc_i2s_cfg txcfg ;

struct soc_i2s_cfg rxcfg ;

Curie_I2SDMA CurieI2SDMA;

Curie_I2SDMA::Curie_I2SDMA()
{
}

void Curie_I2SDMA::lastFrameDelay()
{
    delay(frameDelay);
}

int Curie_I2SDMA::iniTX()
{
	muxTX(1);
	soc_i2s_init();
	soc_dma_init();
	return I2S_DMA_OK; 
}

int Curie_I2SDMA::iniRX()
{
	muxRX(1);
	soc_i2s_init();
	soc_dma_init();
	return I2S_DMA_OK; 
}

void Curie_I2SDMA::muxTX(bool enable)
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
}

void Curie_I2SDMA::muxRX(bool enable)
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
}

int Curie_I2SDMA::beginTX(uint16_t sample_rate,uint8_t resolution,uint8_t master,uint8_t mode)
{
	switch(mode)
	{
		case 1:
			mode = I2S_MODE_PHILLIPS ;
			break;
		case 2:
			mode = I2S_MODE_RJ;
			break;
		case 3:
			mode = I2S_MODE_LJ;
			break;
		case 4:
			mode = I2S_MODE_DSP;
			break;
		default:
			break;
	}
    
	txcfg.sample_rate = sample_rate;
	txcfg.resolution = resolution;
	txcfg.mode = mode;
	txcfg.master = master;
	txcfg.cb_done = txi2s_done;
	txcfg.cb_err = txi2s_err;
	txdone_flag = 0;
	txerror_flag = 0;
	frameDelay =  5;
	soc_i2s_config(I2S_CHANNEL_TX, &txcfg);
	return I2S_DMA_OK;
}

int Curie_I2SDMA::beginRX(uint16_t sample_rate,uint8_t resolution,uint8_t master,uint8_t mode)
{
	switch(mode)
	{
		case 1:
			mode = I2S_MODE_PHILLIPS ;
			break;
		case 2:
			mode = I2S_MODE_RJ;
			break;
		case 3:
			mode = I2S_MODE_LJ;
			break;
		case 4:
			mode = I2S_MODE_DSP;
			break;
		default:
			break;
	}
    
	rxcfg.sample_rate = sample_rate;
	rxcfg.resolution = resolution;
	rxcfg.mode = mode;
	rxcfg.master = master;

	rxcfg.cb_done = rxi2s_done;
	rxcfg.cb_err = rxi2s_err; 
	
	rxdone_flag = 0;
	rxerror_flag = 0;

	soc_i2s_config(I2S_CHANNEL_RX, &rxcfg);

	return I2S_DMA_OK;
}

int Curie_I2SDMA::transTX(void* buf_TX,uint32_t len,uint32_t len_per_data)
{
	int status = soc_i2s_stream(buf_TX, len,len_per_data,0); 
	if(status)
		return I2S_DMA_FAIL;
	while (1) 
	{   
		// check the DMA and I2S status
		if(txdone_flag && !txerror_flag) 
		{  
			txdone_flag = 0;
			txerror_flag = 0;
			return I2S_DMA_OK;
		}
		if(txerror_flag)
		{
			return I2S_DMA_FAIL;
		}        
	}
}

int Curie_I2SDMA::transRX(void* buf_RX,uint32_t len,uint32_t len_per_data)
{
	int status = soc_i2s_listen(buf_RX, len ,len_per_data,0);
	if(status)
		return I2S_DMA_FAIL;

	while (1) 
	{
		// check the DMA and I2S status
		if(rxdone_flag && !rxerror_flag)  
		{
			rxdone_flag = 0;
			rxerror_flag = 0;
			return I2S_DMA_OK;
		}
		if(rxerror_flag)
		{
			return I2S_DMA_FAIL;
		}
	}
	return I2S_DMA_OK;
}

void Curie_I2SDMA::stopTX()
{
	soc_i2s_stop_stream();
	muxTX(0);
}

void Curie_I2SDMA::stopRX()
{
	soc_i2s_stop_listen();
	muxRX(0);
}

int Curie_I2SDMA::mergeData(void* buf_left,void* buf_right,void* buf_TX,uint32_t length_TX,uint32_t len_per_data)
{
	if(len_per_data == 1)
	{
		for(uint32_t i = 0; i < length_TX/2;++i)
		{
			*((uint8_t *)buf_TX+2*i) = *((uint8_t *)buf_left+i);
			*((uint8_t *)buf_TX+2*i+1) = *((uint8_t *)buf_right+i);
		}
	}
	else if(len_per_data == 2)
	{
		for(uint32_t i = 0; i < length_TX/2;++i)
		{
			*((uint16_t *)buf_TX+2*i) = *((uint16_t *)buf_left+i);
			*((uint16_t *)buf_TX+2*i+1) = *((uint16_t *)buf_right+i);
		}
	}
	else if(len_per_data == 4)
	{
		for(uint32_t i = 0; i < length_TX/2;++i)
		{
			*((uint32_t *)buf_TX+2*i) = *((uint32_t *)buf_left+i);
			*((uint32_t *)buf_TX+2*i+1) = *((uint32_t *)buf_right+i);
		}
	}
	else
		return I2S_DMA_FAIL;

	return I2S_DMA_OK;
}

int Curie_I2SDMA::separateData(void* buf_left,void* buf_right,void* buf_RX,uint32_t length_RX,uint32_t len_per_data)
{	 
	if(len_per_data == 1)
	{
		for(uint32_t i = 0; i < length_RX/2;++i)
		{
			*((uint8_t *)buf_left+i) = *((uint8_t *)buf_RX+2*i);
			*((uint8_t *)buf_right+i) = *((uint8_t *)buf_RX+2*i+1);
		}
	}
	else if(len_per_data == 2)
	{
		for(uint32_t i = 0; i < length_RX/2;++i)
		{
			*((uint16_t *)buf_left+i) = *((uint16_t *)buf_RX+2*i);
			*((uint16_t *)buf_right+i) = *((uint16_t *)buf_RX+2*i+1);
		}
	}
	else if(len_per_data == 4)
	{
		for(uint32_t i = 0; i < length_RX/2;++i)
		{
			*((uint32_t *)buf_left+i) = *((uint32_t *)buf_RX+2*i);
			*((uint32_t *)buf_right+i) = *((uint32_t *)buf_RX+2*i+1);
		}
	}
	else
		return I2S_DMA_FAIL;
  
	return I2S_DMA_OK;
}
