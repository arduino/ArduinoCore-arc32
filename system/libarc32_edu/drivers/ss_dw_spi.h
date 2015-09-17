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

#ifndef SS_DW_SPI_H_
#define SS_DW_SPI_H_


/**
 * @defgroup spi_arc_driver Sensor Subsystem SPI
 * @ingroup arc_driver
 * @{
 */

/**
 * Function called when a data on RX buffer is ready to be read.
 * @param dev device that has triggered the ISR
 */
void spi_mst_rx_ISR_proc( spi_info_pt dev );
/**
 * Function called when an TX buffer is ready to start transmission.
 * @param dev device that has triggered the ISR
 */
void spi_mst_tx_ISR_proc( spi_info_pt dev );
/**
 * Function called when an error is detected on SPI. An interrupt on higher
 * level is triggered.
 * @param dev device that has triggered the ISR
 */
void spi_mst_err_ISR_proc( spi_info_pt dev );

/** @} */

#endif  // SS_DW_SPI_H
