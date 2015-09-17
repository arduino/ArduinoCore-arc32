/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "soc_gpio.h"

#if defined(CONFIG_SOC_GPIO_32) || defined(CONFIG_SOC_GPIO_AON)

#include "soc_register.h"
#include "portable.h"

#define GPIO_CLKENA_POS         (31)
#define GPIO_LS_SYNC_POS        (0)

#ifdef __cplusplus
 extern "C" {
#endif

static void soc_gpio_ISR_proc( uint32_t dev_id );

#if defined(CONFIG_SOC_GPIO_32)
/*! \brief  Interrupt handler for GPIO port 0 (32 bits)
 */
DECLARE_INTERRUPT_HANDLER void gpio_isr()
{
    soc_gpio_ISR_proc(SOC_GPIO_32);
}
static gpio_callback_fn soc_gpio_32_cb[SOC_GPIO_32_BITS] = {NULL};
#endif

#if defined(CONFIG_SOC_GPIO_AON)
/*! \brief  Interrupt handler for GPIO port 1 (aon, 6 bits)
 */
DECLARE_INTERRUPT_HANDLER void gpio_aon_isr()
{
    soc_gpio_ISR_proc(SOC_GPIO_AON);
}
static gpio_callback_fn soc_gpio_aon_cb[SOC_GPIO_AON_BITS] = {NULL};
#endif

typedef void (*ISR) ();

/*! GPIO management structure */
typedef struct gpio_info_struct
{
    /* static settings */
    uint32_t           reg_base;       /*!< base address of device register set */
    uint8_t            no_bits;        /*!< no of gpio bits in this entity */
    uint8_t            vector;         /*!< GPIO ISR vector */
    ISR                gpio_isr;       /*!< GPIO ISR */
    uint32_t           gpio_int_mask;  /*!< SSS Interrupt Routing Mask Registers */
    gpio_callback_fn  *gpio_cb;        /*!< Array of user callback functions for user */
    uint8_t            is_init;        /*!< Init state of GPIO port */
} gpio_info_t, *gpio_info_pt;

static gpio_info_t gpio_ports_devs[] = {
#if defined(CONFIG_SOC_GPIO_32)
        { .is_init = 0,
          .reg_base = SOC_GPIO_BASE_ADDR,
          .no_bits = SOC_GPIO_32_BITS,
          .gpio_int_mask = INT_GPIO_MASK,
          .vector = SOC_GPIO_INTERRUPT,
          .gpio_cb = soc_gpio_32_cb,
          .gpio_isr = gpio_isr },
#endif
#if defined(CONFIG_SOC_GPIO_AON)
        { .is_init = 0,
          .reg_base = SOC_GPIO_AON_BASE_ADDR,
          .no_bits = SOC_GPIO_AON_BITS,
          .gpio_int_mask = INT_AON_GPIO_MASK,
          .vector = SOC_GPIO_AON_INTERRUPT,
          .gpio_cb = soc_gpio_aon_cb,
          .gpio_isr = gpio_aon_isr },
#endif
        };


DRIVER_API_RC soc_gpio_set_config(SOC_GPIO_PORT port_id, uint8_t bit, gpio_cfg_data_t *config)
{
    DRIVER_API_RC ret;
    uint32_t saved;

    // check port id
    if(port_id >= SOC_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }

    gpio_info_pt dev = &gpio_ports_devs[port_id];

    // Check pin index
    if (bit >= dev->no_bits) {
        return DRV_RC_CONTROLLER_NOT_ACCESSIBLE;
    }

    // Check if device is initialized
    if(dev->is_init == 0) {
        if((ret = soc_gpio_enable(port_id)) != DRV_RC_OK) {
            return ret;
        }
    }

    // Check if pin is already in use
    if((config->gpio_type == GPIO_INTERRUPT) && (dev->gpio_cb[bit] != NULL)) {
        // pin is already in use
        return DRV_RC_CONTROLLER_IN_USE;
    }

    // Validate config data
    if ((config->gpio_type != GPIO_INPUT) && (config->gpio_type != GPIO_OUTPUT) && (config->gpio_type != GPIO_INTERRUPT)) {
        return DRV_RC_INVALID_CONFIG;
    }
    if ((config->int_type != LEVEL) && (config->int_type != EDGE) && (config->int_type != DOUBLE_EDGE)) {
        return DRV_RC_INVALID_CONFIG;
    }

    /* Disable Interrupts from this bit */
    CLEAR_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_INTEN), (uint32_t)bit);

    // Set interrupt handler to NULL
    dev->gpio_cb[bit] = NULL;

    switch(config->gpio_type)
    {
    case GPIO_INPUT:
        /* configure as input */
        CLEAR_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_SWPORTA_DDR), (uint32_t)bit);
        break;
    case GPIO_OUTPUT:
        /* configure as output */
        SET_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_SWPORTA_DDR), (uint32_t)bit);
        break;
    case GPIO_INTERRUPT:
        saved = interrupt_lock();
        // Configure interrupt handler
        dev->gpio_cb[bit] = config->gpio_cb;

        /* Set as  input */
        MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_SWPORTA_DDR) &= ~(1 << bit);

        /* Set Level, Edge or Double Edge */
        if(config->int_type == LEVEL) {
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTTYPE_LEVEL) &= ~(1 << bit);
        } else if(config->int_type == EDGE) {
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTTYPE_LEVEL) |= (1 << bit);
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INT_BOTHEDGE) &= ~(1 << bit);
        } else { // DOUBLE_EDGE
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTTYPE_LEVEL) |= (1 << bit);
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INT_BOTHEDGE) |= (1 << bit);
        }

        /* Set Polarity - Active Low / High */
        if(ACTIVE_LOW == config->int_polarity) {
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTPOLARITY) &= ~(1 << bit);
        } else {
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTPOLARITY) |= (1 << bit);
        }

        /* Set Debounce - On / Off */
        if(config->int_debounce == DEBOUNCE_OFF) {
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_DEBOUNCE) &= ~(1 << bit);
        } else {
            MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_DEBOUNCE) |= (1 << bit);
        }

        /* Enable as Interrupt */
        MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTEN) |= (1 << bit);

        /* Unmask Interrupt */
        MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTMASK) &= ~(1 << bit);

        interrupt_unlock(saved);
        break;
    }

    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_set_port_config(SOC_GPIO_PORT port_id, gpio_port_cfg_data_t *config)
{
    DRIVER_API_RC ret;
    unsigned int i;

    // check port id
    if(port_id >= SOC_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }

    gpio_info_pt dev = &gpio_ports_devs[port_id];

    // Check if device is initialized
    if(dev->is_init == 0) {
        if((ret = soc_gpio_enable(port_id)) != DRV_RC_OK) {
            return ret;
        }
    }

    // Disable gpio interrupts
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTEN) = 0;

    for(i=0; i<dev->no_bits; i++) {
        dev->gpio_cb[i] = config->gpio_cb[i];
    }

    // Set gpio direction
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_SWPORTA_DDR) = config->gpio_type;
    // Set gpio interrupt settings
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTTYPE_LEVEL) = config->int_type;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTPOLARITY) = config->int_polarity;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_DEBOUNCE) = config->int_debounce;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_LS_SYNC) = config->int_ls_sync;
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INT_BOTHEDGE) = config->int_bothedge;

    // Clear interrupt flag
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_PORTA_EOI) = (1<<dev->no_bits)-1;
    // Enable gpio interrupt
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTMASK) = ~((~(config->gpio_type)) & (config->is_interrupt));
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTEN) = ((~(config->gpio_type)) & (config->is_interrupt));

    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_deconfig(SOC_GPIO_PORT port_id, uint8_t bit)
{
    gpio_info_pt dev;

    /* Check port id */
    if(port_id >= SOC_PORT_COUNT)
        return DRV_RC_INVALID_OPERATION;
    dev = &gpio_ports_devs[port_id];
    /* Check pin index */
    if (bit >= dev->no_bits)
        return DRV_RC_CONTROLLER_NOT_ACCESSIBLE;

    /* Disable interrupts from this pin */
    CLEAR_MMIO_BIT((volatile uint32_t *)(dev->reg_base + SOC_GPIO_INTEN),
		    (uint32_t)bit);
    /* De-configure the interrupt handler */
    dev->gpio_cb[bit] = NULL;
    /* Make sure the pin is left as input */
    CLEAR_MMIO_BIT((volatile uint32_t *)(dev->reg_base + SOC_GPIO_SWPORTA_DDR),
		    (uint32_t)bit);

    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_port_deconfig(SOC_GPIO_PORT port_id)
{
    unsigned int i;
    gpio_port_cfg_data_t config;

    // check port id
    if(port_id >= SOC_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }

    gpio_info_pt dev = &gpio_ports_devs[port_id];

    // Default configuration (input pin without interrupt)
    config.gpio_type = 0;
    config.is_interrupt = 0;
    config.int_type =  0;
    config.int_bothedge =  0;
    config.int_polarity = 0;
    config.int_debounce = 0;
    config.int_ls_sync =  0;

    // TODO: use memset
    for(i=0; i<dev->no_bits; i++) {
        config.gpio_cb[i] = NULL;
    }

    return soc_gpio_set_port_config(port_id, &config);
}

DRIVER_API_RC soc_gpio_enable(SOC_GPIO_PORT port_id)
{
    // check port id
    if(port_id >= SOC_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }
    gpio_info_pt dev = &gpio_ports_devs[port_id];

    /* enable peripheral clock */
    SET_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_LS_SYNC), (uint32_t)GPIO_CLKENA_POS);
    SET_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_LS_SYNC), (uint32_t)GPIO_LS_SYNC_POS);
    /* Clear any existing interrupts */
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_PORTA_EOI) = ~(0);
    /* enable interrupt for this GPIO block */
    SET_INTERRUPT_HANDLER(dev->vector, dev->gpio_isr);
    /* Enable GPIO  Interrupt into SSS  */
    SOC_UNMASK_INTERRUPTS(dev->gpio_int_mask);

    dev->is_init = 1;

    return DRV_RC_OK;
}

DRIVER_API_RC soc_an_gpio_disable(SOC_GPIO_PORT port_id)
{
    // check port id
    if(port_id >= SOC_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }
    gpio_info_pt dev = &gpio_ports_devs[port_id];

    /* Disable All Interrupts from port */
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTEN) = 0;
    /* disable peripheral clock */
    CLEAR_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_LS_SYNC), (uint32_t)GPIO_CLKENA_POS);
    CLEAR_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_LS_SYNC), (uint32_t)GPIO_LS_SYNC_POS);
    /* Disable GPIO  Interrupt into SSS */
    MMIO_REG_VAL(dev->gpio_int_mask) |= DISABLE_SSS_INTERRUPTS;

    dev->is_init = 0;
    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_write(SOC_GPIO_PORT port_id, uint8_t bit, boolean_t value)
{
    // check port id
    if(port_id >= SOC_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }
    gpio_info_pt dev = &gpio_ports_devs[port_id];

    // Check pin index
    if (bit >= dev->no_bits) {
        return DRV_RC_CONTROLLER_NOT_ACCESSIBLE;
    }
    /* read/modify/write bit */
    if (value) {
        SET_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_SWPORTA_DR), (uint32_t)bit);
    } else {
        CLEAR_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_SWPORTA_DR), (uint32_t)bit);
    }

    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_write_port(SOC_GPIO_PORT port_id, uint32_t value)
{
    // check port id
    if(port_id >= SOC_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }
    gpio_info_pt dev = &gpio_ports_devs[port_id];
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_SWPORTA_DR) = value;
    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_read(SOC_GPIO_PORT port_id, uint8_t bit, boolean_t *value)
{
    // check port id
    if(port_id >= SOC_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }

    gpio_info_pt dev = &gpio_ports_devs[port_id];
    // Check pin index
    if (bit >= dev->no_bits) {
        return DRV_RC_CONTROLLER_NOT_ACCESSIBLE;
    }
    if (MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_SWPORTA_DDR) & (1 << bit)) {
        return DRV_RC_INVALID_OPERATION;          /* not configured as input */
    }

    *value = !!(MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_EXT_PORTA) & (1 << bit));
    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_read_port(SOC_GPIO_PORT port_id, uint32_t *value)
{
    // check port id
    if(port_id >= SOC_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }
    gpio_info_pt dev = &gpio_ports_devs[port_id];

    *value = MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_EXT_PORTA);
    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_mask_interrupt(SOC_GPIO_PORT port_id, uint8_t bit)
{
    gpio_info_pt dev = &gpio_ports_devs[port_id];
    SET_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_INTMASK), (uint32_t)bit);
    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_unmask_interrupt(SOC_GPIO_PORT port_id, uint8_t bit)
{
    gpio_info_pt dev = &gpio_ports_devs[port_id];
    CLEAR_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_INTMASK), (uint32_t)bit);
    return DRV_RC_OK;
}

static void soc_gpio_ISR_proc( uint32_t dev_id )
{
    unsigned int i;
    gpio_info_pt dev = &gpio_ports_devs[dev_id];

    // Save interrupt status
    uint32_t status = MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTSTATUS);
    // Mask the pending interrupts
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTMASK) |= status;
    // Clear interrupt flag (write 1 to clear)
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_PORTA_EOI) = status;

    for (i=0; i<dev->no_bits; i++) {
        if ((status & (1 << i)) && (dev->gpio_cb[i]))
	    dev->gpio_cb[i]();
    }
    // Unmask the handled interrupts
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_INTMASK) &= ~status;
}

#ifdef __cplusplus
}
#endif

#endif
