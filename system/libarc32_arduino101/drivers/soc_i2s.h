/*
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
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

#ifndef SOC_I2s_H_
#define SOC_I2s_H_

#include "data_type.h"
#include "soc_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup common_drivers Common Drivers
 * Definition of the drivers APIs accessible from any processor.
 * @ingroup drivers
 */

/**
 * @defgroup soc_i2s Quark SE SOC I2S
 * Quark SE SOC I2S (Audio) drivers API.
 * @ingroup common_drivers
 * @{
 */

// I2S channels
#define I2S_CHANNEL_TX      0
#define I2S_CHANNEL_RX      1
#define I2S_NUM_CHANNELS    2

// I2S modes
#define I2S_MODE_SCK_POL    (0x01)
#define I2S_MODE_WS_POL     (0x02)
#define I2S_MODE_LR_ALIGN   (0x08)
#define I2S_MODE_SAMPLE_DEL (0x10)
#define I2S_MODE_WS_DSP     (0x20)

#define I2S_MODE_PHILLIPS   (I2S_MODE_SCK_POL | I2S_MODE_LR_ALIGN)
#define I2S_MODE_RJ         (I2S_MODE_SCK_POL | I2S_MODE_WS_POL | \
			     I2S_MODE_SAMPLE_DEL)
#define I2S_MODE_LJ         (I2S_MODE_SCK_POL | I2S_MODE_WS_POL | \
			     I2S_MODE_LR_ALIGN | I2S_MODE_SAMPLE_DEL)
#define I2S_MODE_DSP        (I2S_MODE_SCK_POL | I2S_MODE_LR_ALIGN | \
			     I2S_MODE_WS_DSP)

// I2S configuration object
struct soc_i2s_cfg {
	uint16_t sample_rate;     // in Hz
	uint8_t resolution;
	uint8_t mode;
	uint8_t master;

	void (*cb_done)(void *);
	void *cb_done_arg;

	void (*cb_err)(void *);
	void *cb_err_arg;
};

// Internal struct for use by the controller (and soc_config)
struct soc_i2s_info {
	uint32_t reg_base;
	uint32_t int_vector;
	uint32_t int_mask;

	uint32_t clk_speed;

	struct soc_i2s_cfg cfg[I2S_NUM_CHANNELS];
	uint8_t en[I2S_NUM_CHANNELS];
	uint8_t cfgd[I2S_NUM_CHANNELS];

	struct soc_dma_cfg dma_cfg[I2S_NUM_CHANNELS];
	struct soc_dma_channel dma_ch[I2S_NUM_CHANNELS];

	struct clk_gate_info_s *clk_gate_info;
};

extern struct driver soc_i2s_driver;

/**
 *  Function to configure specified I2S channel
 *
 *  Configuration parameters must be valid or an error is returned - see return values below.
 *
 *  @param   channel         : I2S channel number
 *  @param   cfg             : pointer to configuration structure
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_INVALID_CONFIG               - if any configuration parameters are not valid
 *           - DRV_RC_CONTROLLER_IN_USE,           - if controller is in use
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_i2s_config(uint8_t channel, struct soc_i2s_cfg *cfg);

/**
 *  Function to deconfigure specified I2S channel
 *
 *  @param   channel         : I2S channel number
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_CONTROLLER_IN_USE,           - if controller is in use
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_i2s_deconfig(uint8_t channel);

/**
 *  Function to transmit a block of audio data
 *
 *  @param   buf             : pointer to data to write
 *  @param   len             : length of data to write (in bytes)
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_i2s_write(void *buf, uint32_t len, uint32_t len_per_data);

/**
 *  Function to continuously transmit blocks of audio data
 *
 *  @param   buf             : pointer to buffer to read data from
 *  @param   len             : length of buffer (in bytes)
 *  @param   num_bufs        : number of parts into which to split the buffer; calling the callback
 *                             after each is sent
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_i2s_stream(void *buf, uint32_t len, uint32_t len_per_data, uint32_t num_bufs);

/**
 *  Function to stop a continuous audio data write
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_i2s_stop_stream(void);

/**
 *  Function to receive a block of audio data
 *
 *  @param   buf             : pointer to buffer to fill with data
 *  @param   len             : length of buffer (in bytes)
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_i2s_read(void *buf, uint32_t len, uint32_t len_per_data);

/**
 *  Function to continuously receive blocks of audio data
 *
 *  @param   buf             : pointer to buffer to fill with data
 *  @param   len             : length of buffer (in bytes)
 *  @param   num_bufs        : number of parts into which to split the buffer; calling the callback
 *                             after each is filled
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_i2s_listen(void *buf, uint32_t len,  uint32_t len_per_data, uint8_t num_bufs);

/**
 *  Function to stop a continuous audio data read
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_i2s_stop_listen(void);

DRIVER_API_RC soc_i2s_init();

/** @} */

#ifdef __cplusplus
}
#endif

#endif
