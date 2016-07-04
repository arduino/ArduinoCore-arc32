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

#include "scss_registers.h"
#include "portable.h"
#include "os/os.h"
#include "i2s_priv.h"
#include "data_type.h"
#include "clk_system.h"
#include "soc_i2s.h"

/* Info struct */
struct soc_i2s_info g_i2s_info = {
	.reg_base = SOC_I2S_BASE,
	.int_vector = SOC_I2S_INTERRUPT,
	.int_mask = INT_I2S_MASK,
	.clk_speed = 32000000,
	.clk_gate_info = &(struct clk_gate_info_s) 
	{
		.clk_gate_register = PERIPH_CLK_GATE_CTRL,
		.bits_mask = I2S_CLK_GATE_MASK,
	}
};

struct soc_i2s_info* i2s_info = &g_i2s_info;

/* Function Prototypes */
static void i2s_enable(uint8_t channel);
static void i2s_disable(uint8_t channel);
static void i2s_isr(void);
DRIVER_API_RC soc_i2s_config(uint8_t channel, struct soc_i2s_cfg *cfg);
DRIVER_API_RC soc_i2s_deconfig(uint8_t channel);
DRIVER_API_RC soc_i2s_read(void *buf, uint32_t len, uint32_t len_per_data);
DRIVER_API_RC soc_i2s_listen(void *buf, uint32_t len, uint32_t len_per_data, uint8_t num_bufs);
DRIVER_API_RC soc_i2s_stop_listen(void);
DRIVER_API_RC soc_i2s_write(void *buf, uint32_t len, uint32_t len_per_data);
DRIVER_API_RC soc_i2s_stream(void *buf, uint32_t len, uint32_t len_per_data, uint32_t num_bufs);
DRIVER_API_RC soc_i2s_stop_stream(void);
DRIVER_API_RC soc_i2s_init();

/* Internal functions */
static void i2s_enable(uint8_t channel)
{
	uint32_t reg;

	// Enable local clock and interrupts
	reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].cid_ctrl);
	reg &= ~(1 << (i2s_reg_map[channel].cid_ctrl_strobe));
	reg &= ~(1 << (i2s_reg_map[channel].cid_ctrl_strobe_sync));
	reg |= (1 << (i2s_reg_map[channel].cid_ctrl_mask));
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].cid_ctrl) = reg;

	// Clear all interrupts
	reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].stat);
	reg &= ~(i2s_reg_map[channel].stat_mask);
	reg |= (1 << SOC_I2S_STAT_TDATA_UNDERR);
	reg |= (1 << SOC_I2S_STAT_RDATA_OVRERR);
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].stat) = reg;

	// Set enabled flag for channel
	i2s_info->en[channel] = 1;

	return;
}

static void i2s_disable(uint8_t channel)
{
	uint32_t reg;
	uint32_t num_active;
	int i;

	// Release DMA resources
	if (i2s_info->en[channel]) 
	{
		soc_dma_stop_transfer(&(i2s_info->dma_ch[channel]));
		soc_dma_release(&(i2s_info->dma_ch[channel]));
		soc_dma_free_list(&(i2s_info->dma_cfg[channel]));
	}
 	// Clear enabled flag for channel
	i2s_info->en[channel] = 0;

	// Let the processor do whatever power down it wants
	num_active = 0;
	for (i = 0; i < I2S_NUM_CHANNELS; i++) 
	{
		if (i2s_info->en[i]) 
		{
			num_active++;
		}
	}
 
	// Disable channel and hold parts in reset
	reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].ctrl);
	reg &= ~(1 << (i2s_reg_map[channel].ctrl_fifo_rst));
	reg &= ~(1 << (i2s_reg_map[channel].ctrl_sync_rst));
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].ctrl) = reg;

	// Clear all interrupts
	reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].stat);
	reg &= ~(i2s_reg_map[channel].stat_mask);
	reg &= ~(1 << i2s_reg_map[channel].stat_err);
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].stat) = reg;

	// Disable local clock and interrupts
	reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].cid_ctrl);
	reg |= (1 << (i2s_reg_map[channel].cid_ctrl_strobe));
	reg |= (1 << (i2s_reg_map[channel].cid_ctrl_strobe_sync));
	reg &= ~(1 << (i2s_reg_map[channel].cid_ctrl_mask));
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].cid_ctrl) = reg;

	return;

}

/* ISR */
static void i2s_isr(void)
{
	uint32_t stat;

	// Determine interrupt source
	stat = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, SOC_I2S_STAT);

	// Check for errors
	if (stat & (1 << SOC_I2S_STAT_TDATA_UNDERR)) 
	{
		if (i2s_info->cfg[I2S_CHANNEL_TX].cb_err) 
		{
			i2s_info->cfg[I2S_CHANNEL_TX].cb_err_arg=(void *)stat;
			i2s_info->cfg[I2S_CHANNEL_TX].cb_err(i2s_info->cfg[I2S_CHANNEL_TX].cb_err_arg);
		}
		i2s_disable(I2S_CHANNEL_TX);
	}
	if (stat & (1 << SOC_I2S_STAT_RDATA_OVRERR)) 
	{
		if (i2s_info->cfg[I2S_CHANNEL_RX].cb_err) 
		{
			i2s_info->cfg[I2S_CHANNEL_RX].cb_err_arg=(void *)stat;
			i2s_info->cfg[I2S_CHANNEL_RX].cb_err(i2s_info->cfg[I2S_CHANNEL_RX].cb_err_arg);
		}
		i2s_disable(I2S_CHANNEL_RX);
	}

	return;
}

DECLARE_INTERRUPT_HANDLER static void i2s_interrupt_handler()
{
	i2s_isr();
}

/* DMA Callbacks */
static void i2s_dma_cb_err(void *num)
{
	uint8_t channel = (uint32_t)num;

	if (i2s_info->cfg[channel].cb_err) 
	{
		i2s_info->cfg[channel].cb_err(i2s_info->cfg[channel].cb_err_arg);
	}
	i2s_disable(channel);

	return;
}

static void i2s_dma_cb_done(void *num)
{
	uint8_t channel = (uint32_t)num;
 	uint32_t reg;
  
	if(channel == I2S_CHANNEL_TX)
	{
		do
		{
			reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].fifo_stat);
		} while(reg & 0x000000FF); 
	}
	
	if (i2s_info->cfg[channel].cb_done) 
	{
		i2s_info->cfg[channel].cb_done(i2s_info->cfg[channel].cb_done_arg);
	}
 
	i2s_disable(channel);
 
	return;
}

static void i2s_dma_cb_block(void *num)
{
	uint8_t channel = (uint32_t)num;

	if (i2s_info->cfg[channel].cb_done) 
	{
		i2s_info->cfg[channel].cb_done(i2s_info->cfg[channel].cb_done_arg);
	}

	return;
}

/* External API */
DRIVER_API_RC soc_i2s_config(uint8_t channel, struct soc_i2s_cfg *cfg)
{
	uint32_t reg;
	uint16_t sample_rate;

	// Check channel no in use
	if (channel >= I2S_NUM_CHANNELS) 
	{
		return DRV_RC_FAIL;
	} 
	else if (i2s_info->en[channel]) 
	{
		return DRV_RC_CONTROLLER_IN_USE;
	}

	// Set master/slave
	reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].ctrl);
	reg &= ~(1 << (i2s_reg_map[channel].ctrl_ms));
	reg |= (cfg->master & 0x1) << i2s_reg_map[channel].ctrl_ms;
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].ctrl) = reg;

	// Calculate sample_rate divider (note, acts as if resolution is always 32)
	sample_rate = i2s_info->clk_speed / (cfg->sample_rate * cfg->resolution * 2);

	// Setup resolution and sampling rate
	reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].srr);
	reg &= ~(i2s_reg_map[channel].srr_mask);
	reg |= (sample_rate & 0x7FF) << i2s_reg_map[channel].srr_sample_rate;
	reg |= ((cfg->resolution - 1) & 0x1F) << i2s_reg_map[channel].srr_resolution;
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].srr) = reg;

	// Setup mode
	reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].dev_conf);
	reg &= ~(i2s_reg_map[channel].dev_conf_mask);
	// Use sck_polar as shift amount as its the LSb of the DEV_CONF settings
	reg |= ((cfg->mode & 0x3F) << i2s_reg_map[channel].dev_conf_sck_polar);
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].dev_conf) = reg;

	// Complete configuration (and set flag)
	i2s_info->cfg[channel] = *cfg;
	i2s_info->cfgd[channel] = 1;

	return DRV_RC_OK;
}

DRIVER_API_RC soc_i2s_deconfig(uint8_t channel)
{
	// Check channel no in use
	if (channel >= I2S_NUM_CHANNELS) 
	{
		return DRV_RC_FAIL;
	} 
	else if (i2s_info->en[channel]) 
	{
		return DRV_RC_CONTROLLER_IN_USE;
	}

	i2s_info->cfgd[channel] = 0;

	return DRV_RC_OK;
}

DRIVER_API_RC soc_i2s_read(void *buf, uint32_t len, uint32_t len_per_data)
{
	// Calling listen with 0 buffers is the same as a onetime read of the whole buffer
	return soc_i2s_listen(buf, len, len_per_data, 0);
}

DRIVER_API_RC soc_i2s_listen(void *buf, uint32_t len, uint32_t len_per_data, uint8_t num_bufs)
{
	DRIVER_API_RC ret;
	uint8_t channel = I2S_CHANNEL_RX;
	uint32_t reg;
	uint32_t len_per_buf;
	int i;
	struct soc_dma_xfer_item *dma_list;

	// Check channel no in use and configured
	if (channel >= I2S_NUM_CHANNELS) 
	{
		return DRV_RC_FAIL;
	} 
	else if (i2s_info->en[channel] || !(i2s_info->cfgd[channel])) 
	{
		return DRV_RC_FAIL;
	}

	// Get a DMA channel
	ret = soc_dma_acquire(&(i2s_info->dma_ch[channel]));

	if (ret != DRV_RC_OK) 
	{
		return DRV_RC_FAIL;
	}

	// Enable the channel
	i2s_enable(channel);

	// Determine the length of a single buffer
	if (num_bufs == 0) 
	{
		len_per_buf = len;
	} 
	else 
	{
		len_per_buf = len / num_bufs;
	}

	// Prep some configuration
	i2s_info->dma_cfg[channel].type = SOC_DMA_TYPE_PER2MEM;
	i2s_info->dma_cfg[channel].src_interface = SOC_DMA_INTERFACE_I2S_RX;
	i2s_info->dma_cfg[channel].dest_step_count = 0;
	i2s_info->dma_cfg[channel].src_step_count = 0;

	i2s_info->dma_cfg[channel].xfer.dest.delta = SOC_DMA_DELTA_INCR;
	i2s_info->dma_cfg[channel].xfer.src.delta = SOC_DMA_DELTA_NONE;
	i2s_info->dma_cfg[channel].xfer.src.addr = (void *)(SOC_I2S_BASE + SOC_I2S_DATA_REG);
 
	if(len_per_data == 1)
	{
		i2s_info->dma_cfg[channel].xfer.dest.width = SOC_DMA_WIDTH_8;
		i2s_info->dma_cfg[channel].xfer.src.width = SOC_DMA_WIDTH_8;
	}
	else if(len_per_data == 2)
	{
		i2s_info->dma_cfg[channel].xfer.dest.width = SOC_DMA_WIDTH_16;
		i2s_info->dma_cfg[channel].xfer.src.width = SOC_DMA_WIDTH_16;
	}
	else  if(len_per_data == 4)
	{
		i2s_info->dma_cfg[channel].xfer.dest.width = SOC_DMA_WIDTH_32;
		i2s_info->dma_cfg[channel].xfer.src.width = SOC_DMA_WIDTH_32;
	}
	else
		return DRV_RC_FAIL;

	if (num_bufs == 0) 
	{
		i2s_info->dma_cfg[channel].cb_done = i2s_dma_cb_done;
		i2s_info->dma_cfg[channel].cb_done_arg = (void *)((uint32_t)channel);
	} 
	else 
	{
		i2s_info->dma_cfg[channel].cb_block = i2s_dma_cb_block;
		i2s_info->dma_cfg[channel].cb_block_arg = (void *)((uint32_t)channel);
	}

	i2s_info->dma_cfg[channel].cb_err = i2s_dma_cb_err;
	i2s_info->dma_cfg[channel].cb_err_arg = (void *)((uint32_t)channel);

	// Setup the linked list
	for (i = 0; i < ((num_bufs == 0) ? 1 : num_bufs); i++) 
	{
		if (i == 0) 
		{
			dma_list = &(i2s_info->dma_cfg[channel].xfer);
		} 
		else 
		{
			ret = soc_dma_alloc_list_item(&dma_list, dma_list);

			if (ret != DRV_RC_OK) 
			{
				goto fail;
			}
		}

		dma_list->dest.addr = (void *)(uint8_t *)(buf+i * len_per_buf );
		dma_list->size = len_per_buf / len_per_data;
	}

	// Create a circular list if we are doing circular buffering
	if (num_bufs != 0) 
	{
		dma_list->next = &(i2s_info->dma_cfg[channel].xfer);
	}

	// Setup and start the DMA engine
	ret = soc_dma_config(&(i2s_info->dma_ch[channel]), &(i2s_info->dma_cfg[channel]));

	if (ret != DRV_RC_OK) 
	{
		goto fail;
	}

	ret = soc_dma_start_transfer(&(i2s_info->dma_ch[channel]));

	if (ret != DRV_RC_OK) 
	{
		goto fail;
	}

	// Enable the channel and let it go!
	reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].ctrl);
	reg |= (1 << (i2s_reg_map[channel].ctrl_en));
	reg |= (1 << (i2s_reg_map[channel].ctrl_sync_rst));
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].ctrl) = reg;

	return DRV_RC_OK;

fail:
	i2s_disable(channel);
	soc_dma_release(&(i2s_info->dma_ch[channel]));
	return DRV_RC_FAIL;
}

DRIVER_API_RC soc_i2s_stop_listen(void)
{
	uint8_t channel = I2S_CHANNEL_RX;
	uint32_t save;

	if (channel >= I2S_NUM_CHANNELS) 
	{
		return DRV_RC_FAIL;
	} 
	else if (!(i2s_info->en[channel])) 
	{
		return DRV_RC_FAIL;
	}

	save = interrupt_lock();
	i2s_disable(channel);
	interrupt_unlock(save);

	return DRV_RC_OK;
}

DRIVER_API_RC soc_i2s_write(void *buf, uint32_t len, uint32_t len_per_data)
{
	// Calling stream with 0 buffers is the same as a onetime write of the whole buffer
	return soc_i2s_stream(buf, len, len_per_data, 0);
}

DRIVER_API_RC soc_i2s_stream(void *buf, uint32_t len, uint32_t len_per_data, uint32_t num_bufs)
{
	DRIVER_API_RC ret;
	uint8_t channel = I2S_CHANNEL_TX;
	uint32_t reg;
	uint32_t len_per_buf;
	int i;
	struct soc_dma_xfer_item *dma_list;

	// Check channel no in use and configured
	if (channel >= I2S_NUM_CHANNELS) 
	{
		return DRV_RC_FAIL;
	} 
	else if (i2s_info->en[channel] || !(i2s_info->cfgd[channel])) 
	{
		return DRV_RC_FAIL;
	}

	// Get a DMA channel
	ret = soc_dma_acquire(&(i2s_info->dma_ch[channel]));

	if (ret != DRV_RC_OK) 
	{
		return DRV_RC_FAIL;
	}

	// Enable the channel
	i2s_enable(channel);

	// Determine the length of a single buffer
	if (num_bufs == 0) 
	{
		len_per_buf = len;
	} 
	else 
	{
		len_per_buf = len / num_bufs;
	}

	// Prep some configuration
	i2s_info->dma_cfg[channel].type = SOC_DMA_TYPE_MEM2PER;
	i2s_info->dma_cfg[channel].dest_interface = SOC_DMA_INTERFACE_I2S_TX;
	i2s_info->dma_cfg[channel].dest_step_count = 0;
	i2s_info->dma_cfg[channel].src_step_count = 0;

	i2s_info->dma_cfg[channel].xfer.dest.delta = SOC_DMA_DELTA_NONE;
	i2s_info->dma_cfg[channel].xfer.dest.addr = (void *)(SOC_I2S_BASE + SOC_I2S_DATA_REG);
	i2s_info->dma_cfg[channel].xfer.src.delta = SOC_DMA_DELTA_INCR;

	if(len_per_data == 1)
	{
		i2s_info->dma_cfg[channel].xfer.dest.width = SOC_DMA_WIDTH_8;
		i2s_info->dma_cfg[channel].xfer.src.width = SOC_DMA_WIDTH_8;
	}
	else if(len_per_data == 2)
	{
		i2s_info->dma_cfg[channel].xfer.dest.width = SOC_DMA_WIDTH_16;
		i2s_info->dma_cfg[channel].xfer.src.width = SOC_DMA_WIDTH_16;
	}
	else if(len_per_data == 4)
	{
		i2s_info->dma_cfg[channel].xfer.dest.width = SOC_DMA_WIDTH_32;
		i2s_info->dma_cfg[channel].xfer.src.width = SOC_DMA_WIDTH_32;
	}
	else
		return DRV_RC_FAIL;;
  
	if (num_bufs == 0) 
	{
		i2s_info->dma_cfg[channel].cb_done = i2s_dma_cb_done;
		i2s_info->dma_cfg[channel].cb_done_arg = (void *)((uint32_t)channel);
	} 
	else 
	{
		i2s_info->dma_cfg[channel].cb_block = i2s_dma_cb_block;
		i2s_info->dma_cfg[channel].cb_block_arg = (void *)((uint32_t)channel);
	}

	i2s_info->dma_cfg[channel].cb_err = i2s_dma_cb_err;
	i2s_info->dma_cfg[channel].cb_err_arg = (void *)((uint32_t)channel);

	// Setup the linked list
	for (i = 0; i < ((num_bufs == 0) ? 1 : num_bufs); i++) 
	{
		if (i == 0) 
		{
			dma_list = &(i2s_info->dma_cfg[channel].xfer);
		} 
		else 
		{
			ret = soc_dma_alloc_list_item(&dma_list, dma_list);

			if (ret != DRV_RC_OK) 
			{
				goto fail;
			}
		}

		dma_list->src.addr = (void *)(uint8_t *)(buf+i * len_per_buf );
		dma_list->size = len_per_buf / len_per_data;
	}

	// Create a circular list if we are doing circular buffering
	if (num_bufs != 0) 
	{
		dma_list->next = &(i2s_info->dma_cfg[channel].xfer);
	}

	// Setup and start the DMA engine
	ret = soc_dma_config(&(i2s_info->dma_ch[channel]), &(i2s_info->dma_cfg[channel]));

	if (ret != DRV_RC_OK) 
	{
		goto fail;
	}

	ret = soc_dma_start_transfer(&(i2s_info->dma_ch[channel]));

	if (ret != DRV_RC_OK) 
	{
		goto fail;
	}

	// Enable the channel and let it go!
	reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].ctrl);
	reg |= (1 << (i2s_reg_map[channel].ctrl_en));
	reg |= (1 << (i2s_reg_map[channel].ctrl_sync_rst));
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, i2s_reg_map[channel].ctrl) = reg;

	return DRV_RC_OK;

fail:
	i2s_disable(channel);
	soc_dma_release(&(i2s_info->dma_ch[channel]));
	return DRV_RC_FAIL;
}


DRIVER_API_RC soc_i2s_stop_stream(void)
{
	uint8_t channel = I2S_CHANNEL_TX;
	uint32_t save;

	if (channel >= I2S_NUM_CHANNELS) 
	{
		return DRV_RC_FAIL;
	} 
	else if (!(i2s_info->en[channel])) 
	{
		return DRV_RC_FAIL;
	}

	save = interrupt_lock();
	i2s_disable(channel);
	interrupt_unlock(save);

	return DRV_RC_OK;
}

/* Driver API */
DRIVER_API_RC soc_i2s_init()
{
	int i;
	uint32_t reg;

	// Prep info struct
	for (i = 0; i < I2S_NUM_CHANNELS; i++) 
	{
		i2s_info->en[i] = 0;
		i2s_info->cfgd[i] = 0;
		i2s_info->cfg[i].cb_done = NULL;
		i2s_info->cfg[i].cb_err = NULL;
	}

	// Enable global clock, use local clock gating per channel instead
	set_clock_gate(i2s_info->clk_gate_info, CLK_GATE_ON);

	// Setup ISR (and enable)
	SET_INTERRUPT_HANDLER(i2s_info->int_vector, i2s_interrupt_handler);
	SOC_UNMASK_INTERRUPTS(i2s_info->int_mask);

	// Set up control register
	reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, SOC_I2S_CTRL);
	reg |= (1 << SOC_I2S_CTRL_TR_CFG_0);
 	reg &= ~(1 << SOC_I2S_CTRL_TSYNC_LOOP_BACK);
	reg &= ~(1 << SOC_I2S_CTRL_RSYNC_LOOP_BACK);
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, SOC_I2S_CTRL) = reg;

	// Set the watermark levels
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, SOC_I2S_TFIFO_CTRL) &= 0xFFFCFFFF;
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, SOC_I2S_TFIFO_CTRL) |= (I2S_TFIFO_THR << SOC_I2S_TFIFO_CTRL_TAFULL_THRS);

	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, SOC_I2S_RFIFO_CTRL) &= 0xFFFCFFFF;
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, SOC_I2S_RFIFO_CTRL) |= (I2S_RFIFO_THR << SOC_I2S_RFIFO_CTRL_RAFULL_THRS);

	// Enable global interrupt mask
	reg = MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, SOC_I2S_CID_CTRL);
	reg |= (1 << SOC_I2S_CID_CTRL_INTREQ_MASK);
	MMIO_REG_VAL_FROM_BASE(SOC_I2S_BASE, SOC_I2S_CID_CTRL) = reg;

	// Initially, have all channels disabled
	for (i = 0; i < I2S_NUM_CHANNELS; i++) 
	{
		i2s_disable(i);
	}

	return DRV_RC_OK;
}

