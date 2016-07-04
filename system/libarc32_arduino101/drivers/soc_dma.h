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

#ifndef SOC_DMA_H_
#define SOC_DMA_H_

#include "data_type.h"
#include "os/os.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SOC_DMA_NUM_CHANNELS         8

// Valid values for type
#define SOC_DMA_TYPE_MEM2MEM         0
#define SOC_DMA_TYPE_MEM2PER         1
#define SOC_DMA_TYPE_PER2MEM         2
#define SOC_DMA_TYPE_PER2PER         3

// Valid values for delta
#define SOC_DMA_DELTA_INCR           0
#define SOC_DMA_DELTA_DECR           1
#define SOC_DMA_DELTA_NONE           2

// Valid values for width
#define SOC_DMA_WIDTH_8              0
#define SOC_DMA_WIDTH_16             1
#define SOC_DMA_WIDTH_32             2

// Valid values for dest_interface or src_interface
#define SOC_DMA_INTERFACE_UART0_TX   0
#define SOC_DMA_INTERFACE_UART0_RX   1
#define SOC_DMA_INTERFACE_UART1_TX   2
#define SOC_DMA_INTERFACE_UART1_RX   3
#define SOC_DMA_INTERFACE_SPIM0_TX   4
#define SOC_DMA_INTERFACE_SPIM0_RX   5
#define SOC_DMA_INTERFACE_SPIM1_TX   6
#define SOC_DMA_INTERFACE_SPIM1_RX   7
#define SOC_DMA_INTERFACE_SPIS_TX    8
#define SOC_DMA_INTERFACE_SPIS_RX    9
#define SOC_DMA_INTERFACE_I2S_TX    10
#define SOC_DMA_INTERFACE_I2S_RX    11
#define SOC_DMA_INTERFACE_I2C0_TX   12
#define SOC_DMA_INTERFACE_I2C0_RX   13
#define SOC_DMA_INTERFACE_I2C1_TX   14
#define SOC_DMA_INTERFACE_I2C1_RX   15

// Part of the list item that is the same from destination and source
struct soc_dma_xfer_part {
	uint8_t delta;
	uint8_t width;
	void *addr;
};

// DMa transfer list item
struct soc_dma_xfer_item {
	struct soc_dma_xfer_part src;
	struct soc_dma_xfer_part dest;
	struct soc_dma_xfer_item *next;
	uint16_t size;
};

// DMA configuration object
struct soc_dma_cfg {
	uint8_t type;

	uint8_t dest_interface;
	uint8_t src_interface;
	uint8_t dest_step_count;
	uint8_t src_step_count;
	uint32_t dest_step_interval;
	uint32_t src_step_interval;

	struct soc_dma_xfer_item xfer;

	void (*cb_done)(void *);
	void (*cb_block)(void *);
	void (*cb_err)(void *);
	void *cb_done_arg;
	void *cb_block_arg;
	void *cb_err_arg;
};

// DMA channel object
struct soc_dma_channel {
	uint8_t id;
	uint8_t active;
	uint8_t cfgd;
	struct soc_dma_cfg cfg;
	void *ll;
	void *curr;
};

typedef void (*isr_func)(void);

// Internal struct for use by the controller (and soc_config)
struct soc_dma_info {
	uint32_t                  int_mask[SOC_DMA_NUM_CHANNELS];
	uint32_t                  int_vector[SOC_DMA_NUM_CHANNELS];
	isr_func                  int_handler[SOC_DMA_NUM_CHANNELS];
	uint32_t                  err_mask;
	uint32_t                  err_vector;

	struct soc_dma_channel*   channel[SOC_DMA_NUM_CHANNELS];

	uint32_t                  active;
};


/**
 *  Function to configure a DMA channel (and set it up for transfer)
 *
 *  @param   channel         : pointer to channel object
 *  @param   cfg             : pointer to configuration by which to configure the channel
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_INVALID_CONFIG               - if any configuration parameters are not valid
 *           - DRV_RC_CONTROLLER_IN_USE,           - if controller is in use
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_dma_config(struct soc_dma_channel *channel, struct soc_dma_cfg *cfg);

/**
 *  Function to deconfigure and disable a DMA channel
 *
 *  @param   channel         : pointer to channel object
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_CONTROLLER_IN_USE,           - if controller is in use
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_dma_deconfig(struct soc_dma_channel *channel);

/**
 *  Function to acquire a free DMA channel, must be called before any other DMA functions are used
 *
 *  @param   channel         : pointer to channel object
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_dma_acquire(struct soc_dma_channel *channel);

/**
 *  Function to release a held DMA channel so it can be used by others
 *
 *  @param   channel         : pointer to channel object
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_CONTROLLER_IN_USE,           - if controller is in use
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_dma_release(struct soc_dma_channel *channel);

/**
 *  Function to start a DMA transfer, the channel must be configured before this function is called
 *
 *  @param   channel         : pointer to channel object
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_dma_start_transfer(struct soc_dma_channel *channel);

/**
 *  Function to stop an ongoing DMA channel, does nothing if no transfer is in progress
 *
 *  @param   channel         : pointer to channel object
 *
 *  @return
 *           - DRV_RC_OK on success
 */
DRIVER_API_RC soc_dma_stop_transfer(struct soc_dma_channel *channel);

/**
 *  Function to create a new node for a DMA xfer list. If base is provided, the allocated item will
 *  inherit all the fields from base and the new item will be linked as the item after base
 *
 *  @param   ret             : pointer to pointer which will be assigned to the new list item
 *  @param   base            : pointer to list item to be used as a base
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_dma_alloc_list_item(struct soc_dma_xfer_item **ret, struct soc_dma_xfer_item *base);

/**
 *  Function to free a DMA xfer list (given in the configuration object), should only be used if the
 *  list was allocated with soc_dma_alloc_list_item
 *
 *  @param   cfg             : pointer to configuration object
 *
 *  @return
 *           - DRV_RC_OK on success
 */
DRIVER_API_RC soc_dma_free_list(struct soc_dma_cfg *cfg);

DRIVER_API_RC soc_dma_init();

#ifdef __cplusplus
}
#endif

#endif /* SOC_DMA_H_ */
