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

//CurieI2S.h

#ifndef __CurieI2S_H__
#define __CurieI2S_H__

#include <Arduino.h>

//I2S Modes
#define PHILIPS_MODE 0b001000001000
#define RIGHT_JST_MODE 0b010010010010
#define LEFT_JST_MODE 0b011010011010
#define DSP_MODE 0b101000101000

//Sample Rate I2S_SRR Values
#define I2S_48K     0x000A000A
#define I2S_44K     0x000F000F
#define I2S_24K     0x001C001C
#define I2S_22K     0x001E001E
#define I2S_12K     0x00530053
#define I2S_8K      0x007D007D

//Resolution I2S_SRR Values
#define I2S_32bit   0xF800F800
#define I2S_24bit   0xB800B800
#define I2S_16bit   0x78007800
 
//Registers
#define I2S_CTRL        (uint32_t *)0xB0003800             //I2S Control Register
#define I2S_STAT        (volatile uint32_t *)0xB0003804    //I2S Status Register
#define I2S_SRR         (uint32_t *)0xB0003808             //I2S Channels Sample Rate & Resolution Configuration Register
#define I2S_CID_CTRL    (uint32_t *)0xB000380C             //Clock, Interrupt and DMA Control Register
#define I2S_TFIFO_STAT  (volatile uint32_t *)0xB0003810    //Transmit FIFO Status Register
#define I2S_RFIFO_STAT  (volatile uint32_t *)0xB0003814    //Receive FIFO Status Register
#define I2S_TFIFO_CTRL  (uint32_t *)0xB0003818             //Transmit FIFO Control Register
#define I2S_RFIFO_CTRL  (uint32_t *)0xB000381C             //Receive FIFO Control Register
#define I2S_DEV_CONF    (uint32_t *)0xB0003820             //Device Configuration Register
#define I2S_DATA_REG    (volatile uint32_t *)0xB0003850    //Data Register
#define I2S_MASK_INT    (uint32_t *)0xB0800468             //Host Processor Interrupt Routing Mask 7 
#define CLK_GATE_CTL    (uint32_t *)0xB0800018            //CLK_GATE_CTL register

//DMA
//#define DMA_CTL_L0      (uint32_t *))0xB0700000
//#define DMA_BUFFER_SIZE 128


//Masks
#define I2S_SAMPLERATE_MASK 0xF800F800
#define I2S_RESOLUTION_MASK 0x07FF07FF


//I2S Arduino Pins
#define I2S_TXD     7
#define I2S_TWS     4
#define I2S_TSCK    2
#define I2S_RXD     5
#define I2S_RWS     3
#define I2S_RSCK    8

#define I2S_BUFFER_SIZE 256

//#define I2S_DEBUG

#ifdef I2S_DEBUG
#define I2S_DEBUG_PIN 12
#endif

struct i2s_ring_buffer
{
    volatile uint32_t data[I2S_BUFFER_SIZE];
    volatile int head = 0;
    volatile int tail= 0;
    volatile bool lock = false;
};

class Curie_I2S
{
    private:
        uint32_t clock;
        
        int frameDelay = 0;
        
        bool useDMA;
        
        // initializes i2s interface
        void init();
        
        // mux/demux the i2s rx pins into i2s mode
        void muxRX(bool enable);
        
        // mux/demux the i2s tx pins into i2s mode
        void muxTX(bool enable);
        
        // enables/disables the i2s rx channel
        void enableRXChannel(bool enable);
        
        // enables/disables the i2s tx channel
        void enableTXChannel(bool enable);
        
        // rx sync unit
        void syncTX(bool sync);
        
        // tx sync unit
        void syncRX(bool sync);
        
        // resets RX FIFO pointer
        void resetRXFIFO();
        
        // resets TX FIFO pointer
        void resetTXFIFO();

        //enables i2s interrupts
        void enableInterrupts();
        
        void (*i2s_rxCB)();
        
        void (*i2s_txCB)();
        
        void (*i2s_txEmptyCB)();
        
    public:
        Curie_I2S();
        
        //
        void begin(uint32_t sampleRate, uint32_t resolution);
        
        // enables rx channel
        void enableRX();
        
        // enables tx channel
        void enableTX();
        
        // starts listening to the rx channel
        void startRX();
        
        // starts transmission of data to the tx channel
        void startTX();
        
        // stops listening on rx channel
        void stopRX();
        
        // stops transmission on tx channel
        void stopTX();
        
        // sets the mode of operation for the i2s interface
        void setI2SMode(uint32_t mode);
        
        //  sets sample rate bor both i2s channels
        void setSampleRate(uint32_t dividerValues);
        
        // sets the bit resolution for both i2s cahnnels
        void setResolution(uint32_t resolution);
        
        // Initializes the i2s rx channel
        void initRX();
        
        // Initializes the i2s tx channel
        void initTX();
        
        void end();
        
        // Pushes a dword into the TX buffer
        int pushData(uint32_t data);
        
        // Pushes a dword into the TX FIFO
        void fastPushData(uint32_t data);
        
        void write();
        
        // Pulls a dword directly from the RX FIFO
        uint32_t pullData();
        
        // Pulls a dword from the tail of the rx buffer
        uint32_t read() {return requestdword(); };
        
        // Pulls a dword from the tail of the rx buffer
        uint32_t requestdword();
        
        // Returns number of dword avaialable in the rx buffer
        uint16_t available();
        
        // Returns the number of space available in the tx buffer;
        uint16_t availableTx();
        
        uint8_t getTxFIFOLength();
        
        uint8_t getRxFIFOLength();
        
        void lastFrameDelay();
        
        // Attach user callback that is triggered when there is data pushed into the rx buffer from the RX_FIFO
        void attachRxInterrupt(void (*userCallBack)());
        
        void detachRxInterrupt(void) { return attachTxInterrupt(NULL); };
    
        // Attach user callback that is triggered when that gets called when TX_FIFO is empty(transmission done);
        void attachTxEmptyInterrupt(void (*userCallBack)());
        
        void detachTxEmptyInterrupt(void) { return attachTxEmptyInterrupt(NULL); };
        
        // Attach user callback that is triggered when that gets called when TX_FIFO has available space;
        void attachTxInterrupt(void (*userCallBack)());
        
        void detachTxInterrupt(void) { return attachTxInterrupt(NULL); };
        
        //rx callback
        void i2s_rx_callback(void);
        
        //tx callback
        void i2s_tx_callback(void);
        
        //tx empty callback
        void i2s_tx_empty_callback(void);
};

extern Curie_I2S CurieI2S;

#endif
