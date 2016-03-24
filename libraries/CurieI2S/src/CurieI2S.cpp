//CurieI2S.cpp

#include "CurieI2S.h"
#include <interrupt.h>

Curie_I2S CurieI2S;

static uint32_t _i2s_dma_Rx_Buffer[DMA_BUFFER_SIZE];
static uint32_t _i2s_dma_Tx_Buffer[DMA_BUFFER_SIZE];

static struct i2s_ring_buffer _i2s_Rx_Buffer;
static struct i2s_ring_buffer _i2s_Tx_Buffer;

static struct i2s_ring_buffer *_i2s_Rx_BufferPtr = &_i2s_Rx_Buffer;
static struct i2s_ring_buffer *_i2s_Tx_BufferPtr = &_i2s_Tx_Buffer;

static void i2sInterruptHandler(void)
{
    //TODO: Make interrupt handling more atomic
    //noInterrupts();
    //uint32_t fifoL = *I2S_TFIFO_STAT;
    
    //tx FIFO almost empty
    if(*I2S_STAT & 0x00000200)
    {
        //Serial.println("tx almost empty");
        //Serial.print("I2S_STAT: ");
        //Serial.println(*I2S_STAT, HEX);
        //Serial.print("I2S_TFIFO_STAT: ");
        //Serial.println(fifoL, HEX);
        
        //disable TFIFO_AEMPTY interrupt
        //*I2S_CID_CTRL = *I2S_CID_CTRL & 0xFDFFFFFF;
        
        //fill FIFO from buffer
        if(_i2s_Tx_BufferPtr->head != _i2s_Tx_BufferPtr->tail)
        {
            int index = _i2s_Tx_BufferPtr->tail;
            int cnt = 0;
            for(cnt = 0; (index != _i2s_Tx_BufferPtr->head) && (cnt < 2); cnt++)
            {
                *I2S_DATA_REG  = _i2s_Tx_BufferPtr->data[index];
                index = (index+1)%I2S_BUFFER_SIZE;
            } 
            _i2s_Tx_BufferPtr->tail = (_i2s_Tx_BufferPtr->tail + cnt)%I2S_BUFFER_SIZE;
        }

        //clear flags
        *I2S_STAT = *I2S_CTRL & 0xFFFFF0FF;
        
        return;
    }
    
    //tx FIFO full
    if(*I2S_STAT & 0x00000400)
    {
        //Serial.println("tx full");
        //Serial.print("I2S_STAT: ");
        //Serial.println(*I2S_STAT, HEX);
        //Serial.print("I2S_TFIFO_STAT: ");
        //Serial.println(fifoL, HEX);
        //disable TFIFO_FULL interrupts
        *I2S_CID_CTRL = *I2S_CID_CTRL & 0xFBFFFFFF;
        
        //start transmit data in FIFO
        *I2S_CTRL  = *I2S_CTRL | 0x02000000;
        //delayTicks(3200);
    
        //clear flags
        *I2S_STAT = *I2S_CTRL & 0xFFFFF0FF;
        
        
        //enable TFIFO_EMPTY and TFIFO_AEMPTY interrupt
        *I2S_CID_CTRL = *I2S_CID_CTRL | 0x03000000;
        
        return;
    }
    
    //tx FIFO almost full
    //if(*I2S_STAT & 0x00000200)
    {
        //fill up FIFO from buffer if available;
    }
    
    //tx FIFO empty
    if(*I2S_STAT & 0x00000100)
    {
        //Serial.println("tx empty");
        //Serial.print("I2S_STAT: ");
        //Serial.println(*I2S_STAT, HEX);
        //Serial.print("I2S_TFIFO_STAT: ");
        //Serial.println(fifoL, HEX);
        //disable TFIFO_EMPTY interrupt
        *I2S_CID_CTRL = *I2S_CID_CTRL & 0xFEFFFFFF;
  
        delayTicks(960); //allow enough time to send last frame
        //stop transmission
        *I2S_CTRL = *I2S_CTRL & 0xFDFFFFFF;
        
        //clear flags
        *I2S_STAT = *I2S_CTRL & 0xFFFFF0FF;
        
        //enable TFIFO_FULL interrupt
        *I2S_CID_CTRL = *I2S_CID_CTRL | 0x04000000;
        
        
        //Serial.print("CID_CTRL: ");
        //Serial.println(*I2S_CID_CTRL, HEX);
        
        return;
    }
    
    
    
    //overrun and underrun
    
    //interrupts();
}
 
Curie_I2S::Curie_I2S()
{
    
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
}
void Curie_I2S::startRX()
{
    enableRXChannel(1);
    syncRX(1);
}

void Curie_I2S::startTX()
{
    syncTX(1);
    delayTicks(800); //add frame delay to prevent sending emptyFrame
    muxTX(1);
}

void Curie_I2S::stopRX()
{
    syncRX(0);
    enableRXChannel(0);
}

void Curie_I2S::stopTX()
{
    //delayTicks(160);
    //muxTX(0);
    delayTicks(320); 
    syncTX(0);
    //delayTicks(320);
    muxTX(0);
    //enableTXChannel(0);
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
    uint32_t clk_gate_ctl = *(uint32_t *)0xB0800018;
    clk_gate_ctl |= 0x200200;
    *(uint32_t *)0xB0800018 = clk_gate_ctl;
    
    //configure I2S_CTRL register and set TX as master and RX as slave
    uint32_t i2s_ctrl = *I2S_CTRL;
    i2s_ctrl &= 0xF9FFFCFC;
    i2s_ctrl |= 0x08B00100;
    *I2S_CTRL = i2s_ctrl;
    
    //set threshold for FIFOs
    *I2S_TFIFO_CTRL |= 0x00030002;
    *I2S_RFIFO_CTRL |= 0x00030002; 
    
    //enable interrupts
    //ToDo: Use DMA instead of relying on interrupts
    enableInterrupts();
}

void Curie_I2S::muxRX(bool enable)
{
    int mux_mode = GPIO_MUX_MODE;
    if(enable)
    {
        mux_mode = 1;
    }
    
    /* Set SoC pin mux configuration */
    SET_PIN_MODE(49, mux_mode); //I2S_RXD
    SET_PIN_MODE(51, mux_mode); //I2S_RWS
    SET_PIN_MODE(50,  mux_mode); //I2S_RSCK
    g_APinDescription[I2S_RXD].ulPinMode = mux_mode;
    g_APinDescription[I2S_RWS].ulPinMode = mux_mode;
    g_APinDescription[I2S_RSCK].ulPinMode  = mux_mode;
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
    g_APinDescription[I2S_TXD].ulPinMode = mux_mode;
    g_APinDescription[I2S_TWS].ulPinMode = mux_mode;
    g_APinDescription[I2S_TSCK].ulPinMode  = mux_mode;
}

void Curie_I2S::initRX()
{
    muxRX(1);
    resetRXFIFO();
}

void Curie_I2S::initTX()
{
    muxTX(1);
    resetTXFIFO();
    enableTXChannel(1);
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
    //Serial.println("enabling interrupts");
    uint32_t int_i2s_mask = *INT_I2S_MASK;
    int_i2s_mask &= 0xFFFFFEFF;
    *INT_I2S_MASK = int_i2s_mask;
    
    uint32_t i2s_cid_ctrl = *I2S_CID_CTRL;
    i2s_cid_ctrl |= 0x0F008000;
    *I2S_CID_CTRL = i2s_cid_ctrl;
    
    interrupt_disable(IRQ_I2S_INTR);
    interrupt_connect(IRQ_I2S_INTR , &i2sInterruptHandler);
    interrupt_enable(IRQ_I2S_INTR); 
}

void Curie_I2S::transmitFIFO()
{
    if(getTxFIFOLength())
    {
        startTX();
        while(getTxFIFOLength());
        stopTX();
    }
}

void Curie_I2S::transmitFrame()
{
    
}

void Curie_I2S::pushData(uint32_t data)
{
    int i = (uint32_t)(_i2s_Tx_BufferPtr->head +1) % I2S_BUFFER_SIZE;
    if(i != _i2s_Tx_BufferPtr->tail)
    {
        _i2s_Tx_BufferPtr->data[_i2s_Tx_BufferPtr->head] = data;
        _i2s_Tx_BufferPtr->head = i;
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

void Curie_I2S::pushDataFrame(uint32_t leftChData, uint32_t rightChData)
{
    //int fifoLength = getTxFIFOLength();
    //if(!(fifoLength%2) && (fifoLength < 4))
    {
        pushData(leftChData);
        pushData(rightChData);
    }
}

uint8_t Curie_I2S::getTxFIFOLength()
{
    uint8_t fifolength = *I2S_TFIFO_STAT & 0x000000FF;
    delayTicks(640);
    return fifolength;
}

uint8_t Curie_I2S::getRxFIFOLength()
{
    uint8_t fifolength = *I2S_RFIFO_STAT & 0x000000FF;
    delayTicks(640);
    return fifolength;
}
