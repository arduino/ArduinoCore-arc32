/*******************************************************************************
 *
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
 *
 ******************************************************************************/

/*******************************************************************************
 *
 * Modifications Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 ******************************************************************************/

#include <stdint.h>

#include "common_spi.h"
#include "eiaextensions.h"
#include "portable.h"
#include "soc_gpio.h"
#include "spi_priv.h"
#include "clk_system.h"

#include "ss_spi.h"

#define SPI_MAX_CNT (2)

/**
 *  Clock speed into SPI peripheral
 */
#define FREQ_SPI_CLOCK_IN                                                      \
    (CLOCK_SPEED * 1000 * 1000) /* CONFIG_CLOCK_SPEED in MHz */

static spi_info_t ss_spi_master_devs[SPI_MAX_CNT] = {
    {.instID = 0,
     .reg_base = AR_IO_SPI_MST0_CTRL,
     .creg_spi_clk_ctrl = CREG_CLK_CTRL_SPI0,
     .clk_gate_info =
         &(struct clk_gate_info_s){
             .clk_gate_register = SS_PERIPH_CLK_GATE_CTL,
             .bits_mask = SS_SPI0_CLK_GATE_MASK,
         }},
    {.instID = 1,
     .reg_base = AR_IO_SPI_MST1_CTRL,
     .creg_spi_clk_ctrl = CREG_CLK_CTRL_SPI1,
     .clk_gate_info =
         &(struct clk_gate_info_s){
             .clk_gate_register = SS_PERIPH_CLK_GATE_CTL,
             .bits_mask = SS_SPI1_CLK_GATE_MASK,
         }},
};

void ss_spi_set_clock_divider(SPI_CONTROLLER controller_id, uint8_t clockDiv)
{
    uint32_t spien, timing;
    spi_info_pt dev = &ss_spi_master_devs[controller_id];

    spien = READ_ARC_REG(dev->reg_base + SPIEN);
    /* disable controller */
    WRITE_ARC_REG(SPI_DISABLE, dev->reg_base + SPIEN);

    /* Set SPI Clock Divider */
    timing = READ_ARC_REG(dev->reg_base + TIMING);
    timing &= SPI_SCKDEV_SET_MASK;
    timing |= clockDiv;
    WRITE_ARC_REG(timing, dev->reg_base + TIMING);

    /* re-enable controller */
    WRITE_ARC_REG(spien | SPI_ENABLE, dev->reg_base + SPIEN);
}

void ss_spi_set_data_mode(SPI_CONTROLLER controller_id, uint8_t dataMode)
{
    uint32_t spien, ctrl;
    spi_info_pt dev = &ss_spi_master_devs[controller_id];

    spien = READ_ARC_REG(dev->reg_base + SPIEN);
    /* disable controller */
    WRITE_ARC_REG(SPI_DISABLE, dev->reg_base + SPIEN);

    /* Set frame size, bus mode and transfer mode */
    ctrl = READ_ARC_REG(dev->reg_base + CTRL);
    ctrl &= (SPI_SCPL_SET_MASK & SPI_SCPH_SET_MASK);
    ctrl = (dataMode << 6);
    WRITE_ARC_REG(ctrl, dev->reg_base + CTRL);

    /* re-enable controller */
    WRITE_ARC_REG(spien | SPI_ENABLE, dev->reg_base + SPIEN);
}

void ss_spi_init(SPI_CONTROLLER controller_id, uint32_t speed,
                 SPI_BUS_MODE mode, SPI_DATA_FRAME_SIZE data_frame_size,
                 SPI_SLAVE_ENABLE slave)
{
    uint32_t reg = 0;
    uint32_t cs = 0;
    spi_info_pt dev = &ss_spi_master_devs[controller_id];

    dev->slave = slave;
    if (dev->slave == SPI_SE_1) {
        cs = 0;
    } else if (dev->slave == SPI_SE_2) {
        cs = 1;
    } else if (dev->slave == SPI_SE_3) {
        cs = 2;
    } else if (dev->slave == SPI_SE_4) {
        cs = 3;
    }

    /* Configure pin-mux settings on the Intel Curie module to
     * enable SPI mode usage */
    if (controller_id == SPI_SENSING_0) {
        SET_PIN_MODE(28, QRK_PMUX_SEL_MODEA);      // SPI0_SS_MISO
        SET_PIN_MODE(29, QRK_PMUX_SEL_MODEA);      // SPI0_SS_MOSI
        SET_PIN_MODE(30, QRK_PMUX_SEL_MODEA);      // SPI0_SS_SCK
        SET_PIN_MODE(31 + cs, QRK_PMUX_SEL_MODEA); // SPI0_SS_CS_B[0]
    } else if (controller_id == SPI_SENSING_1) {
        SET_PIN_MODE(35, QRK_PMUX_SEL_MODEA);      // SPI1_SS_MISO
        SET_PIN_MODE(36, QRK_PMUX_SEL_MODEA);      // SPI1_SS_MOSI
        SET_PIN_MODE(37, QRK_PMUX_SEL_MODEA);      // SPI1_SS_SCK
        SET_PIN_MODE(38 + cs, QRK_PMUX_SEL_MODEA); // SPI0_SS_CS_B[0]
    }

    /* enable clock to peripheral */
    set_clock_gate(dev->clk_gate_info, CLK_GATE_ON);
    SET_ARC_BIT(AR_IO_CREG_MST0_CTRL, dev->creg_spi_clk_ctrl);

    /* disable controller */
    WRITE_ARC_REG(SPI_DISABLE, dev->reg_base + SPIEN);
    /* enable clock to controller to allow reg writes */
    WRITE_ARC_REG(SPI_CLK_ENABLED, dev->reg_base + CTRL);

    /* Set frame size, bus mode and transfer mode */
    reg = SPI_CLK_ENABLED | data_frame_size | (mode << 6) | (SPI_TX_RX << 8);
    WRITE_ARC_REG(reg, dev->reg_base + CTRL);
    WRITE_ARC_REG((7 << 16) | 0, dev->reg_base + FTLR);

    reg = READ_ARC_REG(dev->reg_base + TIMING);
    reg &= SPI_SCKDEV_SET_MASK;
    reg |= FREQ_SPI_CLOCK_IN / (speed * BAUD_DIVISOR);
    WRITE_ARC_REG(reg, dev->reg_base + TIMING);

    /* Disable interrupts */
    WRITE_ARC_REG(SPI_DISABLE_INT, dev->reg_base + INTR_MASK);
}

void ss_spi_disable(SPI_CONTROLLER controller_id)
{
    spi_info_pt dev = &ss_spi_master_devs[controller_id];
    /* gate SPI controller clock */
    WRITE_ARC_REG(0, dev->reg_base + CTRL);
}

static inline void spi_transmit(spi_info_pt dev, uint8_t *buf, size_t count,
                                boolean_t waitCompletion)
{
    while (count--) {
        if (READ_ARC_REG(dev->reg_base + SR) & SPI_STATUS_TFNF) {
            WRITE_ARC_REG(SPI_PUSH_DATA | *buf++, dev->reg_base + DR);
        }
    }

    /* Wait for transfer to complete */
    if (waitCompletion)
        while (READ_ARC_REG(dev->reg_base + SR) & SPI_STATUS_BUSY)
            ;
}

static inline void spi_receive(spi_info_pt dev, uint8_t *buf, size_t count)
{
    while (count) {
        if (READ_ARC_REG(dev->reg_base + RXFLR) > 0) {
            WRITE_ARC_REG(SPI_POP_DATA, dev->reg_base + DR);
            *buf++ = READ_ARC_REG(dev->reg_base + DR);
            count--;
        }
        READ_ARC_REG(dev->reg_base +
                     RXFLR); /* Extra read of RXFLR, apparently necessary */
    }
}

/* Polling-based SPI transfer to allow use within an ISR */
int ss_spi_xfer(SPI_CONTROLLER controller_id, uint8_t *buf, unsigned tx_cnt,
                unsigned rx_cnt)
{
    uint32_t spien = 0;
    uint32_t ctrl = 0;
    spi_info_pt dev = &ss_spi_master_devs[controller_id];

    spien = READ_ARC_REG(dev->reg_base + SPIEN);
    spien &= SPI_ENB_SET_MASK;
    spien &= SPI_SER_SET_MASK;
    spien |= (dev->slave << 4);
    WRITE_ARC_REG(spien, dev->reg_base + SPIEN);

    ctrl = READ_ARC_REG(dev->reg_base + CTRL) & SPI_NDF_SET_MASK &
           SPI_TMOD_SET_MASK;
    ctrl |= (rx_cnt - 1) << 16;
    if (tx_cnt == 0) {
        ctrl |= (SPI_RX_ONLY << 8);
    } else if (rx_cnt == 0) {
        ctrl |= (SPI_TX_ONLY << 8);
    } else {
        ctrl |= (SPI_EPROM_RD << 8);
    }
    WRITE_ARC_REG(ctrl, dev->reg_base + CTRL);

    // Assert the slave-select and start the SPI transfer
    WRITE_ARC_REG(spien | SPI_ENABLE, dev->reg_base + SPIEN);

    if (tx_cnt)
        spi_transmit(dev, buf, tx_cnt,
                     !(rx_cnt)); // No wait for TX-RX transfers
    else
        WRITE_ARC_REG(SPI_PUSH_DATA | 0x56,
                      dev->reg_base + DR); // start rx-only transfert

    if (rx_cnt)
        spi_receive(dev, buf, rx_cnt);

    // De-assert the slave-select and end the SPI transfer
    WRITE_ARC_REG(0, dev->reg_base + SPIEN);

    return 0;
}
