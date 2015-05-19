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

#include "intel_qrk_pwm.h"
#include "portable.h"
#include "scss_registers.h"

#ifdef CONFIG_BOARD_ATLASPEAK_FPGA
    /* Clock is 16Mzh */
    #define GRANULARITY_X_100           6250
#else
    /* Clock is 32 Mhz */
    #define GRANULARITY_X_100           3125
#endif

/* One nanosecond in hertz */
#define NS_IN_HZ                        1000000000

/* TODO: DB - 64Bit calculation not working, causes issues setting slow speeds/high timers*/

#ifdef __cplusplus
 extern "C" {
#endif

static uint32_t soc_pwm_ioread(uint8_t channel, uint32_t offset);
static void soc_pwm_iowrite(uint8_t channel, uint32_t offset, uint32_t val);
static void soc_pwm_mask_interrupt(uint8_t channel);
static void soc_pwm_unmask_interrupt(uint8_t channel);
static DRIVER_API_RC soc_pwm_check_config(struct soc_pwm_channel_config *config);

boolean_t one_shot[QRK_PWM_NPWM];
void (*callback_fn[QRK_PWM_NPWM]) (void);

/*! \fn     void soc_pwm_enable(void)
*
*  \brief   Function disable clock gating for the PWM device
*/
/*  @TODO: clock gating is not supported on FPGA - test on silicon when available */
void soc_pwm_enable(void)
{
    MMIO_REG_VAL(QRK_CLKGATE_CTRL) |= QRK_CLKGATE_CTRL_PWM_ENABLE;
}

/*! \fn     void soc_pwm_disable(void)
*
*  \brief   Function to enable clock gating for the PWM device
*/
void soc_pwm_disable(void)
{
    MMIO_REG_VAL(QRK_CLKGATE_CTRL) &= ~QRK_CLKGATE_CTRL_PWM_ENABLE;
}


/*
 * The channel specific registers are grouped in  sequential blocks of size
 * 0x14. However, the 4 LCNT2 registers are grouped seperately.
 * Therefore, the address calculation changes accordingly for LCNT2.
 */
static uint32_t soc_pwm_ioread(uint8_t channel, uint32_t offset)
{
    int regs_len = 0;

    if (QRK_PWM_N_LOAD_COUNT2 == offset)
    {
        regs_len = QRK_PWM_N_LCNT2_LEN;
    }

    else
    {
        regs_len = QRK_PWM_N_REGS_LEN;
    }

    return MMIO_REG_VAL_FROM_BASE(QRK_PWM_BASE_ADDR, ((channel * regs_len)
                                    + offset));
}

/*
 * The channel specific registers are grouped in 4 sequential blocks of size
 * 0x14. However, the 4 LCNT2 registers are grouped seperately.
 * Therefore, the address calculation changes accordingly for LCNT2.
 */
static void soc_pwm_iowrite(uint8_t channel, uint32_t offset, uint32_t val)
{
    int regs_len = 0;

    if (QRK_PWM_N_LOAD_COUNT2 == offset)
    {
        regs_len = QRK_PWM_N_LCNT2_LEN;
    }

    else
    {
        regs_len = QRK_PWM_N_REGS_LEN;
    }

    MMIO_REG_VAL_FROM_BASE(QRK_PWM_BASE_ADDR, ((channel * regs_len)
                            + offset)) = val;
}

static DRIVER_API_RC soc_pwm_check_config(struct soc_pwm_channel_config *config)
{
    DRIVER_API_RC ret = DRV_RC_OK;

    if (config->channel_num > QRK_PWM_NPWM)
    {
        ret = DRV_RC_INVALID_CONFIG;
    }

    if (PWM_MODE == config->mode)
    {
        if (0 == config->pwm_period_ns || 0 == config->pwm_duty_cycle_ns ||
            config->pwm_duty_cycle_ns > config->pwm_period_ns ||
            config->pwm_duty_cycle_ns > DUTY_CYCLE_MAX_NS ||
            config->pwm_period_ns  > PERIOD_MAX_NS)
        {
             ret = DRV_RC_INVALID_CONFIG;
        }
    }

    else
    {
        if (0 == config->timer_timeout_ns || config->timer_timeout_ns > TIMER_TIMEOUT_MAX_NS)
        {
            ret = DRV_RC_INVALID_CONFIG;
        }
    }

    if (config->mode != PWM_MODE && config->mode != TIMER_MODE)
    {
        ret = DRV_RC_INVALID_CONFIG;
    }

    return ret;
}

/*! \fn     DRIVER_API_RC soc_pwm_set_config(struct soc_pwm_channel_config *config)
*
*  \brief   Function to configure a specified PWM channel
*
*  \param   config   : pointer to a channel configuration structure
*
*  \return  DRV_RC_OK on success\n
*           DRV_RC_FAIL otherwise
*/
DRIVER_API_RC soc_pwm_set_config(struct soc_pwm_channel_config *config)
{
    uint64_t duty_cycle = 0, period = 0;
    uint32_t hcnt = 0, lcnt = 0, val = 0;

    /* Dont go any further if config is bad */
    if(soc_pwm_check_config(config) != DRV_RC_OK)
    {
        return DRV_RC_INVALID_CONFIG;
    }

    /*  Manage differences between PWM and timer mode */
    val = soc_pwm_ioread(config->channel_num, QRK_PWM_N_CONTROL);

    if(PWM_MODE == config->mode)
    {
            duty_cycle = config->pwm_duty_cycle_ns;
            period = config->pwm_period_ns;

            /*  if PWM_MODE enable output */
            val |= QRK_PWM_CONTROL_PWM_OUT;
    }


    if(TIMER_MODE == config->mode)
    {
            duty_cycle = config->timer_timeout_ns;
            period = (config->timer_timeout_ns * 2);

            one_shot[config->channel_num] = config->timer_enable_oneshot;

            /*  if TIMER_MODE disable output */
            val &= ~QRK_PWM_CONTROL_PWM_OUT;
    }

    soc_pwm_iowrite(config->channel_num, QRK_PWM_N_CONTROL, val);


    /* Calculate value for count for LoadCount1 and LoadCount2 register */
    /* Values are multiplied by 100 to increase accuracy
    *  without the use of floats  */
    hcnt = (uint32_t) ((duty_cycle * 100)/GRANULARITY_X_100);
    lcnt = (uint32_t) (((period * 100) - (duty_cycle * 100))/GRANULARITY_X_100);


    /* A count of 0, equates to (1 * granularity), so adjust by -1 */
    if (hcnt > 0)
    {
        hcnt--;
    }

    if (lcnt > 0)
    {
        lcnt--;
    }

    /* Load counter value */
    soc_pwm_iowrite(config->channel_num, QRK_PWM_N_LOAD_COUNT2, hcnt);
    soc_pwm_iowrite(config->channel_num, QRK_PWM_N_LOAD_COUNT1, lcnt);

    /*  Interrupt masking/unmasking and channel specific interrupt functions */
    if (TRUE == config->pwm_enable_interrupts || TIMER_MODE == config->mode)
    {
        soc_pwm_unmask_interrupt(config->channel_num);
        callback_fn[config->channel_num] = config->interrupt_fn;
    }

    else
    {
        soc_pwm_mask_interrupt(config->channel_num);
    }

    return DRV_RC_OK;
}

/*! \fn     void soc_pwm_start(int channel)
*
*  \brief   Function to start a pwm/timer channel
*
*  \param   channel   : Channel number
*/

void soc_pwm_start(uint8_t channel)
{
    uint32_t val = 0;

    /* Read/Write protection */
    /* Protect QRK_PWM_N_CONTROL using lock and unlock of interruptions */
    uint32_t saved = interrupt_lock();

    val = soc_pwm_ioread(channel, QRK_PWM_N_CONTROL);
    val |= QRK_PWM_CONTROL_ENABLE;
    soc_pwm_iowrite(channel, QRK_PWM_N_CONTROL, val);
    interrupt_unlock(saved);
}

/*! \fn     void soc_pwm_stop(int channel)
*
*  \brief   Function to stop a pwm/timer channel
*
*  \param   channel   : Channel number
*/
void soc_pwm_stop(uint8_t channel)
{
    uint32_t val = 0;

    /* Read/Write protection */
    /* Protect QRK_PWM_N_CONTROL using lock and unlock of interruptions */
    uint32_t saved = interrupt_lock();

    val = soc_pwm_ioread(channel, QRK_PWM_N_CONTROL);
    val &= ~QRK_PWM_CONTROL_ENABLE;
    soc_pwm_iowrite(channel, QRK_PWM_N_CONTROL, val);
    interrupt_unlock(saved);
}

static void soc_pwm_mask_interrupt(uint8_t channel)
{
    uint32_t val = 0;

    /* Read/Write protection */
    /* Protect QRK_PWM_N_CONTROL using lock and unlock of interruptions */
    uint32_t saved = interrupt_lock();

    val = soc_pwm_ioread(channel, QRK_PWM_N_CONTROL);
    val |= QRK_PWM_CONTROL_INT_MASK;
    soc_pwm_iowrite(channel, QRK_PWM_N_CONTROL, val);
    interrupt_unlock(saved);
}

static void soc_pwm_unmask_interrupt(uint8_t channel)
{
    uint32_t val = 0;

    /* Read/Write protection */
    /* Protect QRK_PWM_N_CONTROL using lock and unlock of interruptions */
    uint32_t saved = interrupt_lock();

    val = soc_pwm_ioread(channel, QRK_PWM_N_CONTROL);
    val &= ~QRK_PWM_CONTROL_INT_MASK;
    soc_pwm_iowrite(channel, QRK_PWM_N_CONTROL, val);
    interrupt_unlock(saved);
}

/* ! \fn     void pwm_isr(void)
 *
 *  \brief   PWM ISR, if specified calls a user defined callback
 */
DECLARE_INTERRUPT_HANDLER void pwm_isr(void)
{
    uint32_t pending = 0, pwm = 0;

    /*   Which pin (if any) triggered the interrupt */
    while ((pending = MMIO_REG_VAL_FROM_BASE(QRK_PWM_BASE_ADDR, QRK_PWMS_INT_STATUS)))
    {

        do {
            if (pending & 0x01)
            {
                pwm = 0;
                pending &= ~0x01;
            }

            else if (pending & 0x02)
            {
                pwm = 1;
                pending &= ~0x02;
            }

            else if (pending & 0x04)
            {
                pwm = 2;
                pending &= ~0x04;
            }

            else if (pending & 0x08)
            {
                pwm = 3;
                pending &= ~0x08;
            }

            if (callback_fn[pwm])
            {
                (*callback_fn[pwm])();
            }

            if (TRUE == one_shot[pwm])
            {
                soc_pwm_stop(pwm);
            }

        } while (pending);

         /*  Clear the interrupt  */
        MMIO_REG_VAL_FROM_BASE(QRK_PWM_BASE_ADDR, QRK_PWMS_EOI);
    }
}

/*! \fn     DRIVER_API_RC soc_pwm_block_init(void)
*
*  \brief   Function to initialise PWM controller. WARNING: This must be run before configuring a PWM channel.
*
*  \return  DRV_RC_OK on success\n
*           DRV_RC_FAIL otherwise
*/
DRIVER_API_RC soc_pwm_block_init(void)
{
    int i = 0;
    uint32_t val = 0;
    /* Read/Write protection */
    /* Protect QRK_PWM_N_CONTROL using lock and unlock of interruptions */
    uint32_t saved = interrupt_lock();

    for (i = 0; i < QRK_PWM_NPWM; i++)
    {
        val = soc_pwm_ioread(i, QRK_PWM_N_CONTROL);

        /*  Mask interrupts */
        val |= QRK_PWM_CONTROL_INT_MASK;

        /* Set timer mode periodic-
        * EAS "Free-running timer mode is not supported
        * (i.e. TimerXControlReg.”Timer Mode” must be set to b1)*/
        val |= QRK_PWM_CONTROL_MODE_PERIODIC;
        soc_pwm_iowrite(i, QRK_PWM_N_CONTROL, val);
    }
    interrupt_unlock(saved);

    SET_INTERRUPT_HANDLER(SOC_PWM_INTERRUPT, pwm_isr);

    /* unmask pwm ints to arc core */
    SOC_UNMASK_INTERRUPTS(SCSS_INT_PWM_TIMER_MASK_OFFSET);

    return DRV_RC_OK;
}

#ifdef __cplusplus
}
#endif
