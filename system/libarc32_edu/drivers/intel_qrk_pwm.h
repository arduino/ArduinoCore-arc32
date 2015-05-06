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

#ifndef INTEL_QRK_PWM_H_
#define INTEL_QRK_PWM_H_

/**
 * \addtogroup common_drivers
 * @{
 * \defgroup pwm PWM: Pulse Width Modulation API
 * @{
 * \brief Definition of the structure and functions used by PWM Driver implementation.
 */

#include "data_type.h"

/*  Number of available PWM channels available on the
    FST Quark platform */
#define QRK_PWM_NPWM    4

/* Max duty cycle at slowest PWM pulse frequency 100% at 1/60th of a hertz */
#define DUTY_CYCLE_MAX_NS               60000000000
/*  Slowest supported PWM pulse freqency - 1/60th of a hertz */
#define PERIOD_MAX_NS                   60000000000
/*  Max timer timeout - 60 seconds */
#define TIMER_TIMEOUT_MAX_NS            60000000000

/*
 * Mode of operation
 */
typedef enum{
    PWM_MODE = 0,
    TIMER_MODE
}soc_pwm_mode_t;

/*!
 *  \brief   PWM channel configuration
 *           The user instantiates one of these with given parameters for each PWM
 *           channel, configured using the "soc_pwm_set_config" function
 */
struct soc_pwm_channel_config{
    uint8_t channel_num;            /*!< Channel number 1-4 */
    soc_pwm_mode_t mode;      /*!< Operation Mode- PWM or timer mode */
    uint64_t pwm_period_ns;         /*!< PWM period in nanoseconds - PWM mode only  */
    uint64_t pwm_duty_cycle_ns;     /*!< PWM duty cycle in nanoseconds (time high) - PWM mode only */
    boolean_t pwm_enable_interrupts;/*!< Enable all edge interrupts - PWM mode only */
    uint64_t timer_timeout_ns;      /*!< Timer timeout in nanoseconds - timer mode only*/
    boolean_t timer_enable_oneshot; /*!< Enable one shot timer mode - timer mode only */
    void (*interrupt_fn) (void);    /*!< Pointer to function to call when a channel specific PWM interrupt fires or a timer expires */
};

#ifdef __cplusplus
 extern "C" {
#endif

/*! \fn     void soc_pwm_enable(void)
*
*  \brief   Function disable clock gating for the PWM device
*/
void soc_pwm_enable(void);

/*! \fn     void soc_pwm_disable(void)
*
*  \brief   Function to enable clock gating for the PWM device
*/
void soc_pwm_disable(void);

/*! \fn     DRIVER_API_RC soc_pwm_set_config(struct soc_pwm_channel_config *config)
*
*  \brief   Function to configure a specified PWM channel
*
*  \param   config   : pointer to a channel configuration structure
*
*  \return  RC_OK on success\n
*           RC_FAIL otherwise
*/
DRIVER_API_RC soc_pwm_set_config(struct soc_pwm_channel_config *config);

/*! \fn     void soc_pwm_start(int channel)
*
*  \brief   Function to start a pwm/timer channel
*
*  \param   channel   : Channel number
*/
void soc_pwm_start(uint8_t channel);

/*! \fn     void soc_pwm_stop(int channel)
*
*  \brief   Function to stop a pwm/timer channel
*
*  \param   channel   : Channel number
*/
void soc_pwm_stop(uint8_t channel);

/*! \fn     DRIVER_API_RC soc_pwm_block_init(void)
*
*  \brief   Function to initialise PWM controller. WARNING: This must be run before configuring a PWM channel.
*
*  \return  RC_OK on success\n
*           RC_FAIL otherwise
*/
DRIVER_API_RC soc_pwm_block_init(void);


/*! \fn     void pwm_isr(void*)
 *
 *  \brief   PWM ISR, if specified calls a user defined callback
 */
void pwm_isr(void*);

#ifdef __cplusplus
}
#endif

#endif /* INTEL_QRK_PWM_H_ */

/**@} @}*/
