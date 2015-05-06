/** INTEL CONFIDENTIAL Copyright 2014-2015 Intel Corporation All Rights Reserved.
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
  * Intel's prior express written permission.
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

#include <stddef.h>
#include <stdlib.h>

#include "data_type.h"
#include "ss_gpio_iface.h"
#include "soc_register.h"
#include "io_config.h"
#include "eiaextensions.h"
#include "portable.h"

/* EIA GPIO device registers  */
#define     SWPORTA_DR      (0x00)  /* GPIO Port A Data Register*/
#define     SWPORTA_DDR     (0x01)  /* GPIO Port A Data Direction Register */
#define     INTEN           (0x03)  /* GPIO Interrupt Enable Register */
#define     INTMASK         (0x04)  /* GPIO Interrupt Mask Register */
#define     INTTYPE_LEVEL   (0x05)  /* GPIO Interrupt Type Register */
#define     INT_POLARITY    (0x06)  /* GPIO Interrupt Polarity Register */
#define     INTSTATUS       (0x07)  /* GPIO Interrupt Status Register */
#define     DEBOUNCE        (0x08)  /* GPIO Debounce Enable Register */
#define     PORTA_EOI       (0x09)  /* GPIO Port A Clear Interrupt Register */
#define     EXT_PORTA       (0x0a)  /* GPIO External Port A Register */
#define     LS_SYNC         (0x0b)  /* GPIO Level-Sensitive Sync Enable and Clock Enable Register */

#define GPIO_CLKENA_POS         (31)
#define GPIO_CLKENA_MSK         (0x1 << GPIO_CLKENA_POS)
#define GPIO_LS_SYNC_POS        (0)
#define GPIO_LS_SYNC_MSK        (0x1 << GPIO_LS_SYNC_POS)

#define     REG_WRITE( reg, x )   _sr( (unsigned)(x), (unsigned)(dev->reg_base + reg) )
#define     REG_READ( reg )       _lr( (unsigned)(dev->reg_base + reg) )
#define     REG_WRITE_BITS( reg, x, y, len, pos )   REG_WRITE( reg, ( (((x)          & ~( (~(0xffffffff << len)) << pos ))  \
                                                                    | (((y) << pos)  &  ( (~(0xffffffff << len)) << pos ))) ))
#ifdef __cplusplus
 extern "C" {
#endif

static void ss_gpio_ISR_proc( uint32_t dev_id );

/*! \brief  Interrupt handler for GPIO port 0
 */
DECLARE_INTERRUPT_HANDLER static void ss_gpio_8b0_ISR()
{
    ss_gpio_ISR_proc(SS_GPIO_8B0);
}

/*! \brief  Interrupt handler for GPIO port 1
 */
DECLARE_INTERRUPT_HANDLER static void ss_gpio_8b1_ISR()
{
    ss_gpio_ISR_proc(SS_GPIO_8B1);
}

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
    void             **gpio_cb_arg;    /*!< Array of user priv data for callbacks */
    uint8_t            is_init;        /*!< Init state of GPIO port */
} gpio_info_t, *gpio_info_pt;

static gpio_callback_fn ss_gpio0_cb[SS_GPIO_8B0_BITS] = {NULL};
static gpio_callback_fn ss_gpio1_cb[SS_GPIO_8B1_BITS] = {NULL};

static void* ss_gpio0_arg[SS_GPIO_8B0_BITS] = {NULL};
static void* ss_gpio1_arg[SS_GPIO_8B1_BITS] = {NULL};

static gpio_info_t   gpio_ports_devs[] = {
        { .is_init = 0,
          .reg_base = AR_IO_GPIO_8B0_SWPORTA_DR,
          .no_bits = SS_GPIO_8B0_BITS,
          .gpio_int_mask = INT_SS_GPIO_0_INTR_MASK,
          .vector = IO_GPIO_8B0_INT_INTR_FLAG,
          .gpio_cb = ss_gpio0_cb,
          .gpio_cb_arg = ss_gpio0_arg,
          .gpio_isr = ss_gpio_8b0_ISR },
        { .is_init = 0,
          .reg_base = AR_IO_GPIO_8B1_SWPORTA_DR,
          .no_bits = SS_GPIO_8B1_BITS,
          .gpio_int_mask = INT_SS_GPIO_1_INTR_MASK,
          .vector = IO_GPIO_8B1_INT_INTR_FLAG,
          .gpio_cb = ss_gpio1_cb,
          .gpio_cb_arg = ss_gpio1_arg,
          .gpio_isr = ss_gpio_8b1_ISR }
        };

/* configuration for each GPIO */
static uint16_t gpio_cfg[SS_GPIO_8B0_BITS+SS_GPIO_8B1_BITS] = { 0 };

void* ss_gpio_get_callback_arg(SS_GPIO_PORT port_id, uint8_t pin)
{
    // check port id
    if(port_id >= SS_PORT_COUNT) {
        return NULL;
    }
    gpio_info_pt dev = &gpio_ports_devs[port_id];

    if (pin >= dev->no_bits) {
        return NULL;
    }

    return dev->gpio_cb_arg[pin];
}

DRIVER_API_RC ss_gpio_set_config(SS_GPIO_PORT port_id, uint8_t bit, gpio_cfg_data_t *config)
{
    uint8_t gpio_index = 0, i = 0;
    DRIVER_API_RC ret;
    uint32_t saved;

    // check port id
    if(port_id >= SS_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }

    gpio_info_pt dev = &gpio_ports_devs[port_id];

    // Check pin index
    if (bit >= dev->no_bits) {
        return DRV_RC_CONTROLLER_NOT_ACCESSIBLE;
    }

    // Check if device is initialized
    if(dev->is_init == 0) {
        if((ret = ss_gpio_enable(port_id)) != DRV_RC_OK) {
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
    if ((config->int_type != LEVEL) && (config->int_type != EDGE)) {
        return DRV_RC_INVALID_CONFIG;
    }

    /* calculate gpio_index */
    for(i = 0; i < port_id; i++){
        gpio_index += gpio_ports_devs[i].no_bits;
    }
    gpio_index += bit;
    gpio_cfg[gpio_index] = 0;

    /* store individual gpio config */
    gpio_cfg[gpio_index] |= (port_id << 9) | (bit << 4) | \
                            (config->int_type << 2) | (config->int_polarity << 1) | (config->int_debounce);


    /* Disable Interrupts from this bit */
    CLEAR_ARC_BIT((volatile uint32_t *)(dev->reg_base+INTEN), bit);

    // Configure interrupt handler
    dev->gpio_cb[bit] = config->gpio_cb;
    dev->gpio_cb_arg[bit] = config->gpio_cb_arg;

    switch(config->gpio_type)
    {
    case GPIO_INPUT:
        /* configure as input */
        CLEAR_ARC_BIT((volatile uint32_t *)(dev->reg_base+SWPORTA_DDR), bit);
        break;
    case GPIO_OUTPUT:
        /* configure as input */
        SET_ARC_BIT((volatile uint32_t *)(dev->reg_base+SWPORTA_DDR), bit);
        break;
    case GPIO_INTERRUPT:
        saved = interrupt_lock();
        /* Set as  input */
        WRITE_ARC_REG(READ_ARC_REG((volatile uint32_t)(dev->reg_base+SWPORTA_DDR)) & ~(1 << bit),
                                   (volatile uint32_t)(dev->reg_base+SWPORTA_DDR));

        /* Set Level or Edge */
        if(config->int_type == LEVEL) {
            WRITE_ARC_REG((READ_ARC_REG((volatile uint32_t)(dev->reg_base+INTTYPE_LEVEL))) & ~(1 << bit),
                                       (volatile uint32_t)(dev->reg_base+INTTYPE_LEVEL));
        } else {
            WRITE_ARC_REG((READ_ARC_REG((volatile uint32_t)(dev->reg_base+INTTYPE_LEVEL))) | (1 << bit),
                                       (volatile uint32_t)(dev->reg_base+INTTYPE_LEVEL));
        }

        /* Set Polarity - Active Low / High */
        if(ACTIVE_LOW == config->int_polarity) {
            WRITE_ARC_REG((READ_ARC_REG((volatile uint32_t)(dev->reg_base+INT_POLARITY))) & ~(1 << bit),
                                       (volatile uint32_t)(dev->reg_base+INT_POLARITY));
        } else {
            WRITE_ARC_REG((READ_ARC_REG((volatile uint32_t)(dev->reg_base+INT_POLARITY))) | (1 << bit),
                                       (volatile uint32_t)(dev->reg_base+INT_POLARITY));
        }

        /* Set Debounce - On / Off */
        if(config->int_debounce == DEBOUNCE_OFF) {
            WRITE_ARC_REG((READ_ARC_REG((volatile uint32_t)(dev->reg_base+DEBOUNCE))) & ~(1 << bit),
                                       (volatile uint32_t)(dev->reg_base+DEBOUNCE));
        } else {
            WRITE_ARC_REG((READ_ARC_REG((volatile uint32_t)(dev->reg_base+DEBOUNCE))) | (1 << bit),
                                       (volatile uint32_t)(dev->reg_base+DEBOUNCE));
        }

        /* Enable as Interrupt */
        WRITE_ARC_REG((READ_ARC_REG((volatile uint32_t)(dev->reg_base+INTEN))) | (1 << bit),
                                   (volatile uint32_t)(dev->reg_base+INTEN));

        /* Unmask Interrupt */
        WRITE_ARC_REG((READ_ARC_REG((volatile uint32_t)(dev->reg_base+INTMASK))) & ~(1 << bit),
                                   (volatile uint32_t)(dev->reg_base+INTMASK));

        interrupt_unlock(saved);

        break;
    }

    return DRV_RC_OK;
}

DRIVER_API_RC ss_gpio_enable(SS_GPIO_PORT port_id)
{
    // check port id
    if(port_id >= SS_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }
    gpio_info_pt dev = &gpio_ports_devs[port_id];

    /* enable peripheral clock */
    SET_ARC_BIT((volatile uint32_t *)(dev->reg_base+LS_SYNC), GPIO_CLKENA_POS);
    SET_ARC_BIT((volatile uint32_t *)(dev->reg_base+LS_SYNC), GPIO_LS_SYNC_POS);
    /* Clear any existing interrupts */
    REG_WRITE( PORTA_EOI, ~(0));
    /* enable interrupt for this GPIO block */
    SET_INTERRUPT_HANDLER( dev->vector, dev->gpio_isr);
    /* Enable GPIO  Interrupt into SSS  */
    SOC_UNMASK_INTERRUPTS(dev->gpio_int_mask);

    dev->is_init = 1;

    return DRV_RC_OK;
}

DRIVER_API_RC ss_gpio_disable(SS_GPIO_PORT port_id)
{
    // check port id
    if(port_id >= SS_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }
    gpio_info_pt dev = &gpio_ports_devs[port_id];

    /* Disable All Interrupts from port */
    REG_WRITE(INTEN, 0);
    /* disable peripheral clock */
    CLEAR_ARC_BIT((volatile uint32_t *)(dev->reg_base+LS_SYNC), GPIO_CLKENA_POS);
    /* Disable GPIO  Interrupt into SSS */
    MMIO_REG_VAL(dev->gpio_int_mask) |= DISABLE_SSS_INTERRUPTS;

    dev->is_init = 0;
    return DRV_RC_OK;
}

DRIVER_API_RC ss_gpio_write(SS_GPIO_PORT port_id, uint8_t bit, boolean_t value)
{
    // check port id
    if(port_id >= SS_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }
    gpio_info_pt dev = &gpio_ports_devs[port_id];

    // Check pin index
    if (bit >= dev->no_bits) {
        return DRV_RC_CONTROLLER_NOT_ACCESSIBLE;
    }
    /* read/modify/write bit */
    if (value) {
        SET_ARC_BIT((volatile uint32_t *)(dev->reg_base+SWPORTA_DR), bit);
    } else {
        CLEAR_ARC_BIT((volatile uint32_t *)(dev->reg_base+SWPORTA_DR), bit);
    }

    return DRV_RC_OK;
}

DRIVER_API_RC ss_gpio_write_port(SS_GPIO_PORT port_id, uint32_t value)
{
    // check port id
    if(port_id >= SS_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }
    gpio_info_pt dev = &gpio_ports_devs[port_id];
    REG_WRITE( SWPORTA_DR, value );
    return DRV_RC_OK;
}

DRIVER_API_RC ss_gpio_read(SS_GPIO_PORT port_id, uint8_t bit, boolean_t *value)
{
    // check port id
    if(port_id >= SS_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }
    gpio_info_pt dev = &gpio_ports_devs[port_id];
    // Check pin index
    if (bit >= dev->no_bits) {
        return DRV_RC_CONTROLLER_NOT_ACCESSIBLE;
    }
    if (REG_READ(SWPORTA_DDR) & (1 << bit)) {
        return DRV_RC_INVALID_OPERATION;          /* not configured as input */
    }
    *value = !!(REG_READ(EXT_PORTA) & (1 << bit));
    return DRV_RC_OK;
}

DRIVER_API_RC ss_gpio_read_port(SS_GPIO_PORT port_id, uint32_t *value)
{
    // check port id
    if(port_id >= SS_PORT_COUNT) {
        return DRV_RC_INVALID_OPERATION;
    }
    gpio_info_pt dev = &gpio_ports_devs[port_id];

    *value = REG_READ(EXT_PORTA);
    return DRV_RC_OK;
}

static void ss_gpio_ISR_proc( uint32_t dev_id )
{
    unsigned int i;
    gpio_info_pt dev = &gpio_ports_devs[dev_id];
    
    // Save interrupt status
    uint32_t status = REG_READ( INTSTATUS );
    // Clear interrupt flag (write 1 to clear)
    REG_WRITE( PORTA_EOI, status );

    for (i=0; i<dev->no_bits; i++) {
        if ((status & (1 << i)) && (dev->gpio_cb[i])) {
            (dev->gpio_cb[i])(status, dev->gpio_cb_arg[i]);
        }
    }
}

#ifdef __cplusplus
}
#endif
