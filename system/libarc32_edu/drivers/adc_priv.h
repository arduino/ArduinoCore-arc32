/* ==========================================================================
* Synopsys DesignWare Sensor and Control IP Subsystem IO Software Driver and
* documentation (hereinafter, "Software") is an Unsupported proprietary work
* of Synopsys, Inc. unless otherwise expressly agreed to in writing between
* Synopsys and you.
*
* The Software IS NOT an item of Licensed Software or Licensed Product under
* any End User Software License Agreement or Agreement for Licensed Product
* with Synopsys or any supplement thereto. You are permitted to use and
* redistribute this Software in source and binary forms, with or without
* modification, provided that redistributions of source code must retain this
* notice. You may not view, use, disclose, copy or distribute this file or
* any information contained herein except pursuant to this license grant from
* Synopsys. If you do not agree with this notice, including the disclaimer
* below, then you are not authorized to use the Software.
*
* THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
* ========================================================================== */
#ifndef ADC_PRIV_H_
#define ADC_PRIV_H_


/* EAI ADC device registers */
#define     ADC_SET         (0x00)
#define     ADC_DIVSEQSTAT  (0x01)
#define     ADC_SEQ         (0x02)
#define     ADC_CTRL        (0x03)
#define     ADC_INTSTAT     (0x04)
#define     ADC_SAMPLE      (0x05)


/* ADC Specific macros */
#define     ADC_POP_SAMPLE              (0x80000000)
#define     ADC_FLUSH_RX                (0x40000000)
#define     ADC_FTL_SET_MASK            (0x00ffffff) /* FIFO threshold level */
#define     ADC_SEQ_SIZE_SET_MASK       (0x3fc0ffff)
#define     ADC_SEQ_MODE_SET_MASK       (0x3fffdfff)
#define     ADC_CONFIG_SET_MASK         (0x3fffe000)
#define     ADC_CLK_RATIO_MASK          (0x1fffff)
#define     ADC_CLR_UNDRFLOW            (1 << 18)
#define     ADC_CLR_OVERFLOW            (1 << 17)
#define     ADC_CLR_DATA_A              (1 << 16)
#define     ADC_SEQ_TABLE_RST           (0x0040)
#define     ADC_SEQ_PTR_RST             (0x0020)
#define     ADC_SEQ_START               (0x0010)
#define     ADC_SEQ_STOP_MASK           (0x078ec)
#define     ADC_INT_ENA_MASK            (0x001e)
#define     ADC_INT_DSB                 (0x0F00)
#define     ADC_INT_ENABLE              (0x0000)
#define     ADC_CLK_ENABLE              (0x0004)
#define     ADC_ENABLE                  (0x0002)
#define     ADC_DISABLE                 (0x0)
#define     ADC_RESET                   (0x1)
#define     ADC_INT_DATA_A              (0x1)
#define     ADC_INT_ERR                 (0x6)

#define ADC_STATE_CLOSED        0
#define ADC_STATE_DISABLED      1
#define ADC_STATE_IDLE          2
#define ADC_STATE_SAMPLING      3


#define BUFS_NUM               (2)


/* ADC control commands */
#define IO_ADC_SET_CLK_DIVIDER          (0x20)
#define IO_ADC_SET_CONFIG               (0x21)
#define IO_ADC_SET_SEQ_TABLE            (0x22)
#define IO_ADC_SET_SEQ_MODE             (0x23)
#define IO_ADC_SET_SEQ_STOP             (0x24)
#define IO_ADC_SET_RX_THRESHOLD         (0x25)

#define IO_ADC_INPUT_SINGLE_END       0
#define IO_ADC_INPUT_DIFF             1
#define IO_ADC_OUTPUT_PARAL           0
#define IO_ADC_OUTPUT_SERIAL          1
#define IO_ADC_CAPTURE_RISING_EDGE    0
#define IO_ADC_CAPTURE_FALLING_EDGE   1

#define IO_ADC_SEQ_MODE_SINGLESHOT    0
#define IO_ADC_SEQ_MODE_REPETITIVE    1


#define     REG_WRITE( reg, x )   _sr( (unsigned)(x), (unsigned)(dev->reg_base + reg) )
#define     REG_READ( reg )       _lr( (unsigned)(dev->reg_base + reg) )


typedef void (* ISR)();
typedef void (* IO_CB_FUNC)( uint32_t );


typedef struct adc_info_struct
{
    uint32_t        reg_base; /* base address of device register set */
    uint8_t         instID;
    uint8_t         state;
    uint8_t         seq_mode;
    uint8_t         index;
    uint32_t        seq_size;
    uint32_t        rx_len;
    uint32_t *      rx_buf[BUFS_NUM];
    uint32_t *      res_size[BUFS_NUM];
    /* Callbacks */
    IO_CB_FUNC      rx_cb;
    IO_CB_FUNC      err_cb;
    /* Interrupt numbers and handlers */
    uint8_t         rx_vector; /* ISR vectors */
    uint8_t         err_vector;
    ISR             rx_isr; /* ADC device ISRs */
    ISR             err_isr;
    uint16_t        fifo_tld;
    uint16_t        fifo_depth;
    /* SSS Interrupt Routing Mask Registers */
    uint32_t        adc_irq_mask;
    uint32_t        adc_err_mask;
    void *         priv;
} adc_info_t, *adc_info_pt;


#endif  /*  ADC_PRIV_H_ */
