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

//CurieI2S.cpp

#include "CurieI2S.h"
#include <interrupt.h>

Curie_I2S CurieI2S;

//static uint32_t _i2s_dma_Rx_Buffer[DMA_BUFFER_SIZE];
//static uint32_t _i2s_dma_Tx_Buffer[DMA_BUFFER_SIZE];

static struct i2s_ring_buffer _i2s_Rx_Buffer;
static struct i2s_ring_buffer _i2s_Tx_Buffer;

static struct i2s_ring_buffer *_i2s_Rx_BufferPtr = &_i2s_Rx_Buffer;
static struct i2s_ring_buffer *_i2s_Tx_BufferPtr = &_i2s_Tx_Buffer;

//static int _i2s_frame_delay = 960;

static void i2sInterruptHandler(void)
{
    //Serial.println("i2s int");
    uint32_t i2s_stat = *I2S_STAT;
    
    //RX FIFO almost full
    if((*I2S_STAT & 0x00008000) && (*I2S_CID_CTRL & 0x80000000))
    {
        //Serial.println("rx almost full");
        
        //disable RFIFO_AFULL interrupts
        //*I2S_CID_CTRL = *I2S_CID_CTRL & 0x7FFFFFFF;
        
        int index;
        int fifoDataLength = (*I2S_RFIFO_STAT & 0x0000000F);

        for(int i = 0; i < fifoDataLength; i++)
        {
            index = (uint32_t)(_i2s_Rx_BufferPtr->head +1) % I2S_BUFFER_SIZE;
            uint32_t data = *I2S_DATA_REG;
            if(index != _i2s_Rx_BufferPtr->tail)
            {
                _i2s_Rx_BufferPtr->data[_i2s_Rx_BufferPtr->head] = data;
                _i2s_Rx_BufferPtr->head = index;
            }
            #ifdef I2S_DEBUG
            digitalWrite(I2S_DEBUG_PIN, HIGH);
            digitalWrite(I2S_DEBUG_PIN, LOW);
            #endif

        }
            
        //enable RFIFO_EMPTY interrupt
        *I2S_CID_CTRL = *I2S_CID_CTRL | 0x10000000;
        
        //clear flags
        i2s_stat = i2s_stat & 0xFFFF0FFF;
        *I2S_STAT = i2s_stat | 0x00000010;
        
        //call rx callback
        CurieI2S.i2s_rx_callback();
        
        return;
    }

    //RX FIFO empty
    if((*I2S_STAT & 0x00001000) && (*I2S_CID_CTRL & 0x10000000))
    {
        //Serial.println("rx empty");

        //disable RFIFO_EMPTY interrupts
        *I2S_CID_CTRL = *I2S_CID_CTRL & 0xEFFFFFFF;
        
        #ifdef I2S_DEBUG
        digitalWrite(I2S_DEBUG_PIN, HIGH);
        digitalWrite(I2S_DEBUG_PIN, HIGH);
        digitalWrite(I2S_DEBUG_PIN, HIGH);
        digitalWrite(I2S_DEBUG_PIN, LOW);
        #endif
        
        //enable RFIFO_AFULL interrupts
        *I2S_CID_CTRL = *I2S_CID_CTRL | 0x80000000;
        
        //clear flags
        //*I2S_STAT = *I2S_STAT & 0xFFFFEFFF;
        i2s_stat = i2s_stat & 0xFFFF0FFF;
        *I2S_STAT = i2s_stat | 0x00000010;
        
        return;
    }
    
    //TX FIFO almost empty
    if((*I2S_STAT & 0x00000200) && (*I2S_CID_CTRL & 0x02000000))
    {
        //Serial.println("tx almost empty");
        
        //disable TFIFO_AEMPTY interrupts
        *I2S_CID_CTRL = *I2S_CID_CTRL & 0xFDFFFFFF;
        
        #ifdef I2S_DEBUG
        //digitalWrite(I2S_DEBUG_PIN, HIGH);
        //digitalWrite(I2S_DEBUG_PIN, LOW);
        //digitalWrite(I2S_DEBUG_PIN, HIGH);
        //digitalWrite(I2S_DEBUG_PIN, LOW);
        //digitalWrite(I2S_DEBUG_PIN, HIGH);
        //digitalWrite(I2S_DEBUG_PIN, LOW);
        #endif
        
        //call tx callback
        CurieI2S.i2s_tx_callback();
        
        if(_i2s_Tx_BufferPtr->head != _i2s_Tx_BufferPtr->tail)
        {
            int index = _i2s_Tx_BufferPtr->tail;
            int cnt = 0;
            int fifoSpace = (4 - (*I2S_TFIFO_STAT & 0x0000000F));
            for(cnt = 0; (index != (_i2s_Tx_BufferPtr->head)) && (cnt < fifoSpace); cnt++)
            {
                *I2S_DATA_REG  = _i2s_Tx_BufferPtr->data[index];
                index = (index+1)%I2S_BUFFER_SIZE;
            } 
            _i2s_Tx_BufferPtr->tail = (_i2s_Tx_BufferPtr->tail + cnt)%I2S_BUFFER_SIZE;
        }
        else
        {
            //Serial.println("buff empty");
        }
        //clear TX flags
        i2s_stat = i2s_stat & 0xFFFFFCFF;
        *I2S_STAT = i2s_stat | 0x00000001;
        
        //enable TFIFO_EMPTY and TFIFO_AEMPTY interrupt
        *I2S_CID_CTRL = *I2S_CID_CTRL | 0x03000000;
        
        return;
    }
    
    
    //TX FIFO full
    if((*I2S_STAT & 0x00000400) && (*I2S_CID_CTRL & 0x04000000))
    {
        #ifdef I2S_DEBUG
        //digitalWrite(I2S_DEBUG_PIN, HIGH);
        //digitalWrite(I2S_DEBUG_PIN, LOW);
        #endif
            
        //disable TFIFO_FULL interrupts
        *I2S_CID_CTRL = *I2S_CID_CTRL & 0xFBFFFFFF;
        
        //start transmit data in FIFO
        //*I2S_CTRL  = *I2S_CTRL | 0x02000000;
        
        //clear TX flags
        i2s_stat = i2s_stat & 0xFFFFFBFF;
        *I2S_STAT = i2s_stat | 0x00000001;
        
        //enable TFIFO_EMPTY and TFIFO_AEMPTY interrupt
        *I2S_CID_CTRL = *I2S_CID_CTRL | 0x03000000;
        
        return;
    }
    
    //TX FIFO almost full
    /**
    if((*I2S_STAT & 0x00000200) && (*I2S_CID_CTRL & 0x02000000))
    {
    }
    **/
    
    //TX FIFO empty
    if((*I2S_STAT & 0x00000100) && (*I2S_CID_CTRL & 0x01000000))
    {
        //Serial.println("tx fifo empty");
        int fifoLength = *I2S_TFIFO_STAT & 0x0000000F;
        if(!fifoLength)
        {
            //disable TFIFO_EMPTY interrupt
            *I2S_CID_CTRL = *I2S_CID_CTRL & 0xFEFFFFFF;
        
            #ifdef I2S_DEBUG
            digitalWrite(I2S_DEBUG_PIN, HIGH);
            digitalWrite(I2S_DEBUG_PIN, LOW);
            digitalWrite(I2S_DEBUG_PIN, HIGH);
            digitalWrite(I2S_DEBUG_PIN, LOW);
            #endif
            
            //make sure buffer there is no more data in buffer to put into fifo
            if(_i2s_Tx_BufferPtr->head != _i2s_Tx_BufferPtr->tail)
            {
                int index = _i2s_Tx_BufferPtr->tail;
                int cnt = 0;
                for(cnt = 0; (index != (_i2s_Tx_BufferPtr->head)) && (cnt < 4); cnt++)
                {
                    *I2S_DATA_REG  = _i2s_Tx_BufferPtr->data[index];
                    index = (index+1)%I2S_BUFFER_SIZE;
                }
                _i2s_Tx_BufferPtr->tail = (_i2s_Tx_BufferPtr->tail + cnt)%I2S_BUFFER_SIZE;

                //enable Tx interrrupts
                *I2S_CID_CTRL = *I2S_CID_CTRL | 0x07000000;
            }
            else
            {
                //TXFIFO and tx buffer empty
                
                //last frame delay
                //delayTicks(960);
                CurieI2S.lastFrameDelay();
                //stop transmission
                *I2S_CTRL = *I2S_CTRL & 0xFDFFFFFE;
                
                //clear flags
                i2s_stat = i2s_stat & 0xFFFFF0FF;
                *I2S_STAT = i2s_stat | 0x00000001;
                
                //call tx empty callback
                CurieI2S.i2s_tx_empty_callback();
                
                //enable TFIFO_AEMPTY and TFIFO_FULL interrupts
                *I2S_CID_CTRL = *I2S_CID_CTRL | 0x06000000;
                
                return;
            }
        }
        else
        {
            //not really empty
            //clear flags
            i2s_stat = i2s_stat & 0xFFFFFEFF;
            *I2S_STAT = i2s_stat | 0x00000001;
            
            //enable TFIFO_EMPTY and TFIFO_AEMPTY interrupts
            *I2S_CID_CTRL = *I2S_CID_CTRL | 0x03000000;
        }
        return;
    }
    
    /**
    Serial.println("unk int");
    Serial.print("I2S_STAT: ");
    Serial.println(*I2S_STAT, HEX);
    Serial.print("CID_CTRL: ");
    Serial.println(*I2S_CID_CTRL, HEX);
    **/
    
    //TODO: handle overrun and underrun
    
}
 
Curie_I2S::Curie_I2S()
{
    i2s_rxCB = NULL;
    i2s_txEmptyCB = NULL;
    i2s_txCB = NULL;
}

void Curie_I2S::begin(uint32_t sampleRate, uint32_t resolution)
{
    init();
    setSampleRate(sampleRate);
    setResolution(resolution);
}
void Curie_I2S::end()
{
    //disable I2S PCLK Clock Gate
    *CLK_GATE_CTL = *CLK_GATE_CTL & 0xFFDFFDFF;
    muxRX(0);
    enableRXChannel(0);
    syncRX(0);
    muxTX(0);
    enableTXChannel(0);
    syncTX(0);
}

void Curie_I2S::enableTX()
{
    enableTXChannel(1);
    syncTX(1);
}

void Curie_I2S::enableRX()
{
    enableRXChannel(1);
    syncRX(1);
}

void Curie_I2S::startRX()
{
    enableRXChannel(1);
    syncRX(1);
}

void Curie_I2S::startTX()
{
    //make sure fifo is empty
    #ifdef I2S_DEBUG
    digitalWrite(I2S_DEBUG_PIN, HIGH);
    digitalWrite(I2S_DEBUG_PIN, LOW);
    #endif
    resetTXFIFO();
    int cnt = 0;
    if(_i2s_Tx_BufferPtr->head != _i2s_Tx_BufferPtr->tail)
    {
        int index = _i2s_Tx_BufferPtr->tail;
        for(cnt = 0; (index != _i2s_Tx_BufferPtr->head) && (cnt < 4); cnt++)
        {
            *I2S_DATA_REG  = _i2s_Tx_BufferPtr->data[index];
            index = (index+1)%I2S_BUFFER_SIZE;
        } 
        _i2s_Tx_BufferPtr->tail = (_i2s_Tx_BufferPtr->tail + cnt)%I2S_BUFFER_SIZE;
        enableTX();
        //enable TFIFO_EMPTY and TFIFO_AEMPTY interrupt
        *I2S_CID_CTRL = *I2S_CID_CTRL | 0x02000000;
    }
}

void Curie_I2S::stopRX()
{
    syncRX(0);
    enableRXChannel(0);
}

void Curie_I2S::stopTX()
{
    syncTX(0);
}

void Curie_I2S::setI2SMode(uint32_t mode)
{
    *I2S_DEV_CONF = mode;
    uint32_t dev_conf = *I2S_DEV_CONF;
    dev_conf &= 0xFFFFF800;
    dev_conf |= mode;
    *I2S_DEV_CONF = dev_conf;
}

void Curie_I2S::setSampleRate(uint32_t dividerValue)
{
    uint32_t i2s_srr = *I2S_SRR;
    i2s_srr &= I2S_SAMPLERATE_MASK;
    i2s_srr |= dividerValue;
    *I2S_SRR = i2s_srr;
    
    //set frameDelay value.
    frameDelay = (dividerValue&0x000000FF)*32*2;
}

void Curie_I2S::setResolution(uint32_t resolution)
{
    switch(resolution)
    {
        case 32:
            resolution = I2S_32bit;
            break;
        case 24:
            resolution = I2S_24bit;
            break;
        case 16:
            resolution = I2S_16bit;
            break;
        default:
            break;
    }
    uint32_t i2s_srr = *I2S_SRR;
    i2s_srr &= I2S_RESOLUTION_MASK;
    i2s_srr |= resolution;
    *I2S_SRR = i2s_srr;
}

//*************************************************************************************//

void Curie_I2S::init()
{   
    //enable I2S PCLK Clock Gate and I2S Clock
    *CLK_GATE_CTL = *CLK_GATE_CTL | 0x00200200;
    
    //configure I2S_CTRL register and set TX as master and RX as slave
    uint32_t i2s_ctrl = *I2S_CTRL;
    i2s_ctrl &= 0xE0FFFCFC;
    i2s_ctrl |= 0x00B00100;
    *I2S_CTRL = i2s_ctrl;
    
    //set threshold for FIFOs
    *I2S_TFIFO_CTRL |= 0x00030002;
    *I2S_RFIFO_CTRL |= 0x00010002;
    
    //enable interrupts
    //ToDo: Use DMA instead of relying on interrupts
    enableInterrupts();
}

void Curie_I2S::muxRX(bool enable)
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

void Curie_I2S::muxTX(bool enable)
{
    int mux_mode = GPIO_MUX_MODE;
    if(enable)
    {
        mux_mode = I2S_MUX_MODE;
    }
    
    /* Set SoC pin mux configuration */
    SET_PIN_MODE(g_APinDescription[I2S_TXD].ulSocPin, mux_mode);
    SET_PIN_MODE(g_APinDescription[I2S_TWS].ulSocPin, mux_mode);
    SET_PIN_MODE(g_APinDescription[I2S_TSCK].ulSocPin,  mux_mode);
}

void Curie_I2S::initRX()
{
    muxRX(1);
    resetRXFIFO();
    //enableRXChannel(1);
    //enable RX interrupts
    *I2S_CID_CTRL = *I2S_CID_CTRL | 0x80000000;
}

void Curie_I2S::initTX()
{
    muxTX(1);
    resetTXFIFO();
    //enableTXChannel(1);
    //enable TX interrupts
    *I2S_CID_CTRL = *I2S_CID_CTRL | 0x06000000;
}

void Curie_I2S::enableRXChannel(bool enable)
{
    uint32_t i2s_ctrl = *I2S_CTRL;
    if(enable)
    {
        i2s_ctrl |= 0x00000002;
        *I2S_CTRL = i2s_ctrl;
    }
    else
    {
        i2s_ctrl &= 0xFFFFFFFD;
        *I2S_CTRL = i2s_ctrl;
    }
}

void Curie_I2S::enableTXChannel(bool enable)
{
    uint32_t i2s_ctrl = *I2S_CTRL;
    if(enable)
    {
        i2s_ctrl |= 0x00000001;
        *I2S_CTRL = i2s_ctrl;
    }
    else
    {
        i2s_ctrl &= 0xFFFFFFFE;
        *I2S_CTRL = i2s_ctrl;
    } 
}
        
void Curie_I2S::syncTX(bool sync)
{
    uint32_t i2s_ctrl = *I2S_CTRL;
    if(sync)
    {
        i2s_ctrl |= 0x02000000;
        *I2S_CTRL = i2s_ctrl;
    }
    else
    {
        i2s_ctrl &= 0xFDFFFFFF;
        *I2S_CTRL = i2s_ctrl;
    }    
}

void Curie_I2S::syncRX(bool sync)
{
    uint32_t i2s_ctrl = *I2S_CTRL;
    if(sync)
    {
        i2s_ctrl |= 0x04000000;
        *I2S_CTRL = i2s_ctrl;
    }
    else
    {
        i2s_ctrl &= 0xFBFFFFFF;
        *I2S_CTRL = i2s_ctrl;
    } 
}

void Curie_I2S::resetRXFIFO()
{
    uint32_t i2s_ctrl = *I2S_CTRL;
    i2s_ctrl &= 0xFEFFFFFF;
}

void Curie_I2S::resetTXFIFO()
{
    uint32_t i2s_ctrl = *I2S_CTRL;
    i2s_ctrl &= 0xFF7FFFFF;
}

void Curie_I2S::enableInterrupts()
{
    uint32_t int_i2s_mask = *I2S_MASK_INT;
    int_i2s_mask &= 0xFFFFFEFF;
    *I2S_MASK_INT = int_i2s_mask;
    
    uint32_t i2s_cid_ctrl = *I2S_CID_CTRL;
    //i2s_cid_ctrl |= 0x87008000;
    i2s_cid_ctrl |= 0x00008000;
    *I2S_CID_CTRL = i2s_cid_ctrl;
    
    interrupt_disable(IRQ_I2S_INTR);
    interrupt_connect(IRQ_I2S_INTR , &i2sInterruptHandler);
    interrupt_enable(IRQ_I2S_INTR); 
}

int Curie_I2S::pushData(uint32_t data)
{
    //disable TFIFO_AEMPTY interrupts
    *I2S_CID_CTRL = *I2S_CID_CTRL & 0xFDFFFFFF;

    int i = (uint32_t)(_i2s_Tx_BufferPtr->head +1) % I2S_BUFFER_SIZE;
    if(i != _i2s_Tx_BufferPtr->tail)
    {
        _i2s_Tx_BufferPtr->data[_i2s_Tx_BufferPtr->head] = data;
        _i2s_Tx_BufferPtr->head = i;
        //enable TFIFO_AEMPTY interrupts
        *I2S_CID_CTRL = *I2S_CID_CTRL | 0x02000000;
        return 1;
    }
    else
    {
        //enable TFIFO_AEMPTY interrupts
        *I2S_CID_CTRL = *I2S_CID_CTRL | 0x02000000;
        return 0;
    }
}

void Curie_I2S::fastPushData(uint32_t data)
{
    *I2S_DATA_REG = data;
}

uint32_t Curie_I2S::pullData()
{
    uint32_t data = *I2S_DATA_REG;
    return data;
}

uint32_t Curie_I2S::requestdword()
{
    if(_i2s_Rx_BufferPtr->head != _i2s_Rx_BufferPtr->tail)
    {
        uint32_t data = _i2s_Rx_BufferPtr->data[_i2s_Rx_BufferPtr->tail];
        _i2s_Rx_BufferPtr->tail = (_i2s_Rx_BufferPtr->tail + 1)%I2S_BUFFER_SIZE;
        return data;
    }
    else
    {
        //check if there is data in the FIFO
        if(*I2S_RFIFO_STAT & 0x0000000F)
        {
            int index = (uint32_t)(_i2s_Rx_BufferPtr->head +1) % I2S_BUFFER_SIZE;
            uint32_t data = *I2S_DATA_REG;
            if(index != _i2s_Rx_BufferPtr->tail)
            {
                _i2s_Rx_BufferPtr->data[_i2s_Rx_BufferPtr->head] = data;
                _i2s_Rx_BufferPtr->head = index;
            }
            _i2s_Rx_BufferPtr->tail = (_i2s_Rx_BufferPtr->tail + 1)%I2S_BUFFER_SIZE;
            return data;
        }
    }
    return 0;
}

uint16_t Curie_I2S::available()
{
    return (uint16_t)(_i2s_Rx_BufferPtr->head - _i2s_Rx_BufferPtr->tail)%I2S_BUFFER_SIZE;
}

uint16_t Curie_I2S::availableTx()
{   
    if(_i2s_Tx_BufferPtr->tail == _i2s_Tx_BufferPtr->head)
    {
        return I2S_BUFFER_SIZE;
    }
    else
    {
        return ((_i2s_Tx_BufferPtr->tail+I2S_BUFFER_SIZE) - _i2s_Tx_BufferPtr->head+1)%I2S_BUFFER_SIZE;
    }
}

uint8_t Curie_I2S::getTxFIFOLength()
{
    uint8_t fifolength = *I2S_TFIFO_STAT & 0x000000FF;
    return fifolength;
}

uint8_t Curie_I2S::getRxFIFOLength()
{
    uint8_t fifolength = *I2S_RFIFO_STAT & 0x000000FF;
    return fifolength;
}

void Curie_I2S::lastFrameDelay()
{
    delayTicks(frameDelay);
}

void Curie_I2S::attachRxInterrupt(void (*userCallBack)())
{
    i2s_rxCB = userCallBack;
}

void Curie_I2S::attachTxEmptyInterrupt(void (*userCallBack)())
{
    i2s_txEmptyCB = userCallBack;
}

void Curie_I2S::attachTxInterrupt(void (*userCallBack)())
{
    i2s_txCB = userCallBack;
}

inline void Curie_I2S::i2s_rx_callback(void)
{
    if(i2s_rxCB != NULL)
        i2s_rxCB();
}

inline void Curie_I2S::i2s_tx_empty_callback(void)
{
    if(i2s_txEmptyCB != NULL)
        i2s_txEmptyCB();
}

inline void Curie_I2S::i2s_tx_callback(void)
{
    if(i2s_txCB != NULL)
        i2s_txCB();
}
