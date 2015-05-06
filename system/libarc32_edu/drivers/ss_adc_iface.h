/** INTEL CONFIDENTIAL Copyright 2015 Intel Corporation All Rights Reserved.
  *
  * The source code contained or described herein and all documents related to
  * the source code ("Material") are owned by Intel Corporation or its suppliers
  * or licensors.
  * Title to the Material remains with Intel Corporation or its suppliers and
  * licensors.
  * The Material contains trade secrets and proprietary and confidential information
  * of Intel or its suppliers and licensors. The Material is protected by worldwide
  * copyright and trade secret laws and treaty provisions.
  * No part of the Material may be used, copied, reproduced, modified, published,
  * uploaded, posted, transmitted, distributed, or disclosed in any way without
  * Intel’s prior express written permission.
  *
  * No license under any patent, copyright, trade secret or other intellectual
  * property right is granted to or conferred upon you by disclosure or delivery
  * of the Materials, either expressly, by implication, inducement, estoppel or
  * otherwise.
  *
  * Any license under such intellectual property rights must be express and
  * approved by Intel in writing
  *
  ******************************************************************************/

/**
 * \defgroup arc_driver ARC Drivers
 * \brief Definition of the structure and functions used by ARC drivers implementation.
 *
 * It consists of:
 *  - ARC Drivers APIs
 *
 * @{
 */


/**
 * \addtogroup arc_driver
 * @{
 * \defgroup adc_arc_driver ADC: Analog/Digital Converter API
 * @{
 * \brief Definition of the structure and functions used by ADC ARC Driver implementation.
 */


#ifndef SS_ADC_IFACE_H_
#define SS_ADC_IFACE_H_


typedef enum {
    SINGLED_ENDED   = 0,
    DIFFERENTIAL    = 1
} INPUT_MODE;

typedef enum {
    PARALLEL    = 0,
    SERIAL      = 1
} OUTPUT_MODE;


typedef enum {
    RISING_EDGE     = 0,
    FALLING_EDGE    = 1
} CAPTURE_MODE;

typedef enum {
    WIDTH_6_BIT   = 0x0,
    WIDTH_8_BIT   = 0x1,
    WIDTH_10_BIT  = 0x2,
    WIDTH_12_BIT  = 0x3
} SAMPLE_WIDTH;


/*!
*  \brief  callback function signature
*/
typedef void (*adc_callback)( uint32_t );     /*!< callback function signature */

/* Structure representing AD converter configuration */
typedef struct
{
    INPUT_MODE      in_mode;        /*!< ADC input mode: single ended or differential */
    OUTPUT_MODE     out_mode;       /*!< ADC output mode: parallel or serial */
    uint8_t         serial_dly;     /*!< Number of adc_clk the first bit of serial output is delayed for after start of conversion */
    CAPTURE_MODE    capture_mode;   /*!< ADC controller capture mode: by rising or falling edge of adc_clk */
    SAMPLE_WIDTH    sample_width;   /*!< ADC sample width */
    uint32_t        clock_ratio;    /*!< TODO */
    adc_callback    cb_rx;          /*!< callback function for notification of data received and available to application, NULL if callback not required by application code */
    uint32_t        cb_rx_data;     /*!< this will be passed back by the callback routine - can be used as an controller identifier */
    adc_callback    cb_err;         /*!< callback function on transaction error, NULL if callback not required by application code */
    uint32_t        cb_err_data;    /*!< this will be passed back by the callback routine - can be used as an controller identifier */
} ss_adc_cfg_data_t;

/* simple macro to convert resolution to sample width to be used in the
 * configuration structure. It converts from 6,8,10,12 to 0x0,0x01,0x2,0x3
 */
#define ss_adc_res_to_sample_width(_res_) ((SAMPLE_WIDTH)(((_res_-6)/2) & 0x3))
#define ADC_VREF 3300 /* mV = 3.3V */
/* result in mV is given by converting raw data:
 * result = (data * ADC_VREF) / (2^resolution)
 */
#define ss_adc_data_to_mv(_data_, _resolution_) \
	((_data_ * ADC_VREF) / (1 << _resolution_))

/* Structure representing ADC sequence table entry. */
typedef struct
{
    uint32_t    sample_dly; /*!< delay to be introduced prior to start of conversion, in terms of adc clocks */
    uint8_t     channel_id; /*!< ADC input associated with the entry */
} io_adc_seq_entry_t;


/* Structure representing ADC sequence table. */
typedef struct
{
    io_adc_seq_entry_t  *entries;
    uint8_t             num_entries;
} io_adc_seq_table_t;


#ifdef __cplusplus
 extern "C" {
#endif

/*! \fn     void ss_adc_set_config(ss_adc_cfg_data_t *config)
*
*  \brief   Function to configure ADC controller
*           Configuration parameters must be valid or an error is returned - see return values below.
*
*  \param   config          : pointer to configuration structure
*
*/
void ss_adc_set_config(ss_adc_cfg_data_t *config);


/*! \fn     void ss_adc_enable(void)
*
*  \brief   Function to enable the ADC controller
*           Upon success, the ADC interface is no longer clock gated in hardware, it is now
*          TODO capable of transmitting and receiving on the ADC bus and of generating interrupts.
*
*/
void ss_adc_enable(void);


/*! \fn     void ss_adc_disable(void)
*
*  \brief   Function to disable the ADC controller.
*           Upon success, the ADC interface is clock gated in hardware,
*           it is no longer capable of generating interrupts.
*
*/
void ss_adc_disable(void);


/*! \fn     void ss_adc_read(io_adc_seq_table_t * seq_tbl, uint32_t *data,
                        uint32_t data_len, uint32_t *actual_data_len,
                        uint32_t *status_code);
*
*  \brief   Function to read samples from the ADC
*
*  \param   seq_tbl          : pointer to io_adc_seq_table
*  \param   data             : buffer for sampling output
*  \param   data_len         : size of data to be read
*  \param   actual_data_len  : size of data actually read
*  \param   status_code      : sampling status code
*
*  \return  DRV_RC_OK                       on success\n
*           DRV_RC_CONTROLLER_IN_USE,       if ADC is already sampling\n
*/

DRIVER_API_RC ss_adc_read(io_adc_seq_table_t * seq_tbl, uint32_t *data,
                        uint32_t data_len, uint32_t *actual_data_len,
                        uint32_t *status_code);

#ifdef __cplusplus
}
#endif

#endif  /* SS_ADC_IFACE_H_ */

/**@} @} @}*/
