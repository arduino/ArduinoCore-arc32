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
#define I2S_CTRL        (uint32_t *)0xB0003800    //I2S Control Register
#define I2S_STAT        (uint32_t *)0xB0003804    //I2S Status Register
#define I2S_SRR         (uint32_t *)0xB0003808    //I2S Channels Sample Rate & Resolution Configuration Register
#define I2S_CID_CTRL    (uint32_t *)0xB000380C    //Clock, Interrupt and DMA Control Register
#define I2S_TFIFO_STAT  (uint32_t *)0xB0003810    //Transmit FIFO Status Register
#define I2S_RFIFO_STAT  (uint32_t *)0xB0003814    //Receive FIFO Status Register
#define I2S_TFIFO_CTRL  (uint32_t *)0xB0003818    //Transmit FIFO Control Register
#define I2S_RFIFO_CTRL  (uint32_t *)0xB000381C    //Receive FIFO Control Register
#define I2S_DEV_CONF    (uint32_t *)0xB0003820    //Device Configuration Register
#define I2S_DATA_REG    (uint32_t *)0xB0003850    //Data Register
#define INT_I2S_MASK    (uint32_t *)0xB0800468

//DMA
//#define DMA_CTL_L0      (uint32_t *))0xB0700000
#define DMA_BUFFER_SIZE 128


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

struct i2s_ring_buffer
{
    uint32_t data[I2S_BUFFER_SIZE];
    int head = 0;
    int tail = 0;
};

class Curie_I2S
{
    private:
        uint32_t clock;
        bool useDMA;
        
        void init();
        void muxRX(bool enable);
        void muxTX(bool enable);
        
        void enableRXChannel(bool enable);
        void enableTXChannel(bool enable);
        
        void syncTX(bool sync);
        void syncRX(bool sync);
        
        void resetRXFIFO();
        void resetTXFIFO();

        void enableInterrupts();
        
    public:
        Curie_I2S();
        
        void begin(uint32_t sampleRate, uint32_t resolution);
        
        void startRX();
        
        void startTX();
        
        void stopRX();
        
        void stopTX();
        
        void setI2SMode(uint32_t mode);
        
        void setSampleRate(uint32_t dividerValues);
        
        void setResolution(uint32_t resolution);
        
        void initRX();
        
        void initTX();
        
        void end();
        
        void transmitFIFO();
        
        void transmitFrame();
        
        // Pushes a dword into the TX buffer
        void pushData(uint32_t data);
        
        // Pushes a dword into the TX FIFO without doing any checks
        void fastPushData(uint32_t data);
        
        // Pulls a dword from the rx FIFO
        uint32_t pullData();
        
        // Pushes a frame (two dwords) into the TX FIFO
        void pushDataFrame(uint32_t leftChData, uint32_t rightChData);
        
        uint8_t getTxFIFOLength();
        
        uint8_t getRxFIFOLength();
        
        void rsyncLoopback();
        
        void tsyncLoopback();
        
        inline void i2s_tx_callback(void);
        
        inline void i2s_rx_callback(void);
};

extern Curie_I2S CurieI2S;

#endif
