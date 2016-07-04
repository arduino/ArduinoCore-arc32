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
#include "data_type.h"
#include "clk_system.h"

#include "soc_dma.h"
#include "soc_dma_priv.h"

// Some useful helper macros for setting and clearing bits and values (NR: dont read the value first)
// x - address or variable
// f - bit field to set or clear
#define SETB(x, f) ((x) |= (1 << (f)))
#define CLRB(x, f) ((x) &= ~(1 << (f)))

#define SETB_NR(x, f) ((x) = (1 << (f)))
#define CLRB_NR(x, f) ((x) = ~(1 << (f)))

// x - address or variable
// f - starting bit for value
// l - length of value
// v - value to set
#define SETV(x, f, l, v) ((x) &= ~(((1 << (l)) - 1) << (f))); \
	((x) |= ((((1 << (l)) - 1) & (v)) << (f)))

#define SETV_NR(x, f, l, v) ((x) = ((((1 << (l)) - 1) & (v)) << (f)))

// Function Prototypes
static void dma_disable(struct soc_dma_channel *channel);
static void dma_enable(struct soc_dma_channel *channel);
static struct soc_dma_xfer_item *dma_find_list_end(struct soc_dma_xfer_item *head);
static struct dma_lli *dma_find_ll_end(struct dma_lli *head);
static DRIVER_API_RC dma_create_ll(struct soc_dma_channel *channel, struct soc_dma_cfg *cfg);
static void dma_interrupt_handler(void *num);
DRIVER_API_RC soc_dma_config(struct soc_dma_channel *channel, struct soc_dma_cfg *cfg);
DRIVER_API_RC soc_dma_deconfig(struct soc_dma_channel *channel);
DRIVER_API_RC soc_dma_acquire(struct soc_dma_channel *channel);
DRIVER_API_RC soc_dma_release(struct soc_dma_channel *channel);
DRIVER_API_RC soc_dma_start_transfer(struct soc_dma_channel *channel);
DRIVER_API_RC soc_dma_stop_transfer(struct soc_dma_channel *channel);
DRIVER_API_RC soc_dma_alloc_list_item(struct soc_dma_xfer_item **ret, struct soc_dma_xfer_item *base);
DRIVER_API_RC soc_dma_free_list(struct soc_dma_cfg *cfg);
DRIVER_API_RC dma_init();

DECLARE_INTERRUPT_HANDLER static void dma_ch0_interrupt_handler()
{
    dma_interrupt_handler((void *)0);
}

DECLARE_INTERRUPT_HANDLER static void dma_ch1_interrupt_handler()
{
    dma_interrupt_handler((void *)1);
}

struct soc_dma_info g_dma_info = {
	.int_mask = {
		INT_DMA_CHANNEL_0_MASK,
		INT_DMA_CHANNEL_1_MASK,
		INT_DMA_CHANNEL_2_MASK,
		INT_DMA_CHANNEL_3_MASK,
		INT_DMA_CHANNEL_4_MASK,
		INT_DMA_CHANNEL_5_MASK,
		INT_DMA_CHANNEL_6_MASK,
		INT_DMA_CHANNEL_7_MASK
	},
	.int_vector = {
		SOC_DMA_CHANNEL0_INTERRUPT,
		SOC_DMA_CHANNEL1_INTERRUPT,
		SOC_DMA_CHANNEL2_INTERRUPT,
		SOC_DMA_CHANNEL3_INTERRUPT,
		SOC_DMA_CHANNEL4_INTERRUPT,
		SOC_DMA_CHANNEL5_INTERRUPT,
		SOC_DMA_CHANNEL6_INTERRUPT,
		SOC_DMA_CHANNEL7_INTERRUPT
	},
	.int_handler = {
		dma_ch0_interrupt_handler,
		dma_ch1_interrupt_handler,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},
	.err_mask = INT_DMA_ERROR_MASK,
	.err_vector = SOC_DMA_ERR_INTERRUPT
};

static struct soc_dma_info* dma_info = &g_dma_info;

/* Internal Functions */
static void dma_disable(struct soc_dma_channel *channel)
{
	uint32_t reg;

	// Turn off the given channel
	reg = MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_CH_EN_REG);
	SETB(reg, SOC_DMA_CH_EN_REG_CH_EN_WE + (channel->id));
	CLRB(reg, SOC_DMA_CH_EN_REG_CH_EN + (channel->id));
	MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_CH_EN_REG) = reg;

	// Clear interrupts and disable them
	SETB_NR(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_CLEAR_TFR), SOC_DMA_CLEAR_CLEAR + (channel->id));
	SETB_NR(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_CLEAR_BLOCK), SOC_DMA_CLEAR_CLEAR + (channel->id));
	SETB_NR(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_CLEAR_ERR), SOC_DMA_CLEAR_CLEAR + (channel->id));

	reg = MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_MASK_TFR);
	SETB(reg, SOC_DMA_MASK_INT_MASK_WE + (channel->id));
	CLRB(reg, SOC_DMA_MASK_INT_MASK + (channel->id));
	MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_MASK_TFR) = reg;

	reg = MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_MASK_BLOCK);
	SETB(reg, SOC_DMA_MASK_INT_MASK_WE + (channel->id));
	CLRB(reg, SOC_DMA_MASK_INT_MASK + (channel->id));
	MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_MASK_BLOCK) = reg;

	reg = MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_MASK_ERR);
	SETB(reg, SOC_DMA_MASK_INT_MASK_WE + (channel->id));
	CLRB(reg, SOC_DMA_MASK_INT_MASK + (channel->id));
	MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_MASK_ERR) = reg;

	CLRB(dma_regs[channel->id].CTL_L, SOC_DMA_CTL_L_INT_EN);

	// Deactivate channel
	channel->active = 0;
	CLRB(dma_info->active, channel->id);

	return;
}

static void dma_enable(struct soc_dma_channel *channel)
{
	uint32_t reg;

	// Clear interrupts and enable them
	SETB_NR(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_CLEAR_TFR), SOC_DMA_CLEAR_CLEAR + (channel->id));
	SETB_NR(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_CLEAR_BLOCK), SOC_DMA_CLEAR_CLEAR + (channel->id));
	SETB_NR(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_CLEAR_ERR), SOC_DMA_CLEAR_CLEAR + (channel->id));

	reg = MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_MASK_TFR);
	SETB(reg, SOC_DMA_MASK_INT_MASK_WE + (channel->id));
	SETB(reg, SOC_DMA_MASK_INT_MASK + (channel->id));
	MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_MASK_TFR) = reg;

	reg = MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_MASK_BLOCK);
	SETB(reg, SOC_DMA_MASK_INT_MASK_WE + (channel->id));
	SETB(reg, SOC_DMA_MASK_INT_MASK + (channel->id));
	MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_MASK_BLOCK) = reg;

	reg = MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_MASK_ERR);
	SETB(reg, SOC_DMA_MASK_INT_MASK_WE + (channel->id));
	SETB(reg, SOC_DMA_MASK_INT_MASK + (channel->id));
	MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_MASK_ERR) = reg;

	SETB(dma_regs[channel->id].CTL_L, SOC_DMA_CTL_L_INT_EN);

	// Active channel
	channel->active = 1;
	SETB(dma_info->active, channel->id);

	// Turn on the given channel
	reg = MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_CH_EN_REG);
	SETB(reg, SOC_DMA_CH_EN_REG_CH_EN_WE + channel->id);
	SETB(reg, SOC_DMA_CH_EN_REG_CH_EN + channel->id);
	MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_CH_EN_REG) = reg;

	return;
}

static struct soc_dma_xfer_item *dma_find_list_end(struct soc_dma_xfer_item *head)
{
	struct soc_dma_xfer_item *t;
	struct soc_dma_xfer_item *h;

	// Get rid of easy cases
	if ((head == NULL) || (head->next == NULL) ||
	    (head->next->next == NULL)) 
	{
		return NULL;
	}

	// Tortoise and hare check for cyclic ll
	t = head->next;
	h = head->next->next;
	while (t != h) 
	{
		h = h->next;
		if (h == NULL) 
		{
			return NULL;
		}
		h = h->next;
		if (h == NULL) 
		{
			return NULL;
		}
		t = t->next;
	}

	// Find the meeting place since a cycle was detected
	t = head;
	while (t != h)
	{
		t = t->next;
		h = h->next;
	}

	return t;
}

static struct dma_lli *dma_find_ll_end(struct dma_lli *head)
{
	struct dma_lli *t;
	struct dma_lli *h;

	// Get rid of easy cases
	if ((head == NULL) || (((struct dma_lli *)(head->llp)) == NULL) ||
	    (((struct dma_lli *)(((struct dma_lli *)(head->llp))->llp)) == NULL)) 
	{
		return NULL;
	}

	// Tortoise and hare check for cyclic ll
	t = (struct dma_lli *)(head->llp);
	h = (struct dma_lli *)(((struct dma_lli *)(head->llp))->llp);
	while (t != h)
	{
		h = (struct dma_lli *)(h->llp);
		if (h == NULL)
		{
			return NULL;
		}
		h = (struct dma_lli *)(h->llp);
		if (h == NULL)
		{
			return NULL;
		}
		t = (struct dma_lli *)(t->llp);
	}

	// Find the meeting place since a cycle was detected
	t = head;
	while (t != h) 
	{
		t = (struct dma_lli *)(t->llp);
		h = (struct dma_lli *)(h->llp);
	}

	return t;
}

static DRIVER_API_RC dma_create_ll(struct soc_dma_channel *channel, struct soc_dma_cfg *cfg)
{
	OS_ERR_TYPE err;
	struct dma_lli *lli;
	struct dma_lli *last_lli;
	struct soc_dma_xfer_item *xfer;
	struct soc_dma_xfer_item *last_xfer;
	uint8_t list_done;
	uint16_t size_left;
	uint32_t curr_src;
	uint8_t src_delta_amount;
	uint32_t curr_dest;
	uint8_t dest_delta_amount;
	uint32_t reg;

	// Save the LL
	channel->ll = balloc(sizeof(struct dma_lli), &err);
	lli = (struct dma_lli *)(channel->ll);
	last_lli = NULL;

	if (channel->ll == NULL) 
	{
		return DRV_RC_FAIL;
	}

	xfer = &(cfg->xfer);
	last_xfer = dma_find_list_end(xfer);

	list_done = 0;

	// Set register to default CTL_L register value
	reg = 0;
	SETB(reg, SOC_DMA_CTL_L_LLP_SRC_EN);
	SETB(reg, SOC_DMA_CTL_L_LLP_DST_EN);
	SETV(reg, SOC_DMA_CTL_L_TT_FC, SOC_DMA_CTL_L_TT_FC_LEN, cfg->type);
	SETV(reg, SOC_DMA_CTL_L_SRC_TR_WIDTH, SOC_DMA_CTL_L_SRC_TR_WIDTH_LEN, SOC_DMA_WIDTH_32);
	SETV(reg, SOC_DMA_CTL_L_DST_TR_WIDTH, SOC_DMA_CTL_L_DST_TR_WIDTH_LEN, SOC_DMA_WIDTH_32);
	SETB(reg, SOC_DMA_CTL_L_INT_EN);

	while (!list_done)
	{
		size_left = xfer->size;
		curr_src = (uint32_t)(xfer->src.addr);
		curr_dest = (uint32_t)(xfer->dest.addr);

		if (xfer == last_xfer)
		{
			last_lli = lli;
		}

		while (size_left)
		{
			lli->sar = curr_src;
			lli->dar = curr_dest;

			// Set the control register for this block
			SETV(lli->ctl_u, SOC_DMA_CTL_U_BLOCK_TS, SOC_DMA_CTL_U_BLOCK_TS_LEN,
				(size_left > SOC_DMA_BLOCK_SIZE_MAX) ? SOC_DMA_BLOCK_SIZE_MAX : size_left);

			lli->ctl_l = reg;

			if (cfg->src_step_count) 
			{
				SETB(lli->ctl_l, SOC_DMA_CTL_L_SRC_GATHER_EN);

				// Scatter-Gather adds too much complexity for breaking up big blocks
				if (size_left > SOC_DMA_BLOCK_SIZE_MAX) 
				{
					soc_dma_deconfig(channel);
					return DRV_RC_INVALID_CONFIG;
				}
			}

			if (cfg->dest_step_count) 
			{
				SETB(lli->ctl_l, SOC_DMA_CTL_L_DST_SCATTER_EN);

				// Scatter-Gather adds too much complexity for breaking up big blocks
				if (size_left > SOC_DMA_BLOCK_SIZE_MAX) 
				{
					soc_dma_deconfig(channel);
					return DRV_RC_INVALID_CONFIG;
				}
			}

			SETV(lli->ctl_l, SOC_DMA_CTL_L_SINC, SOC_DMA_CTL_L_SINC_LEN, xfer->src.delta);
			SETV(lli->ctl_l, SOC_DMA_CTL_L_DINC, SOC_DMA_CTL_L_DINC_LEN, xfer->dest.delta);

			SETV(lli->ctl_l, SOC_DMA_CTL_L_SRC_TR_WIDTH, SOC_DMA_CTL_L_SRC_TR_WIDTH_LEN, xfer->src.width);
			SETV(lli->ctl_l, SOC_DMA_CTL_L_DST_TR_WIDTH, SOC_DMA_CTL_L_DST_TR_WIDTH_LEN, xfer->dest.width);

			if (size_left > SOC_DMA_BLOCK_SIZE_MAX) 
			{
				lli->end_group = 0;
				lli->llp = (uint32_t)balloc(sizeof(struct dma_lli), &err);

				if (lli->llp  == 0) 
				{
					soc_dma_deconfig(channel);
					return DRV_RC_FAIL;
				}

				// Calculate new addresses for next block
				size_left -= SOC_DMA_BLOCK_SIZE_MAX;

				src_delta_amount = ((xfer->src.width == SOC_DMA_WIDTH_8) ? 1 :
					((xfer->src.width == SOC_DMA_WIDTH_16) ? 2 : 4));
				dest_delta_amount = ((xfer->dest.width == SOC_DMA_WIDTH_8) ? 1 :
					((xfer->dest.width == SOC_DMA_WIDTH_16) ? 2 : 4));

				switch (xfer->src.delta) 
				{
					case SOC_DMA_DELTA_INCR:
						curr_src += (src_delta_amount * SOC_DMA_BLOCK_SIZE_MAX);
						break;
					case SOC_DMA_DELTA_DECR:
						curr_src -= (src_delta_amount * SOC_DMA_BLOCK_SIZE_MAX);
						break;
				}

				switch (xfer->dest.delta) 
				{
					case SOC_DMA_DELTA_INCR:
						curr_dest += (dest_delta_amount * SOC_DMA_BLOCK_SIZE_MAX);
					break;
					case SOC_DMA_DELTA_DECR:
						curr_dest -= (dest_delta_amount * SOC_DMA_BLOCK_SIZE_MAX);
					break;
				}

				lli = (struct dma_lli *)lli->llp;
			} 
			else 
			{
				// Terminate this group of blocks for this xfer item
				lli->end_group = 1;
				size_left = 0;
			}
		}

		// Are we on the last block? If so, complete the list and leave, else setup the next loop
		if ((xfer->next) == last_xfer) 
		{
			lli->llp = (uint32_t)last_lli;
			list_done = 1;

			if (lli->llp == 0) 
			{
				SETB(lli->ctl_u, SOC_DMA_CTL_U_DONE_BIT);
				CLRB(lli->ctl_l, SOC_DMA_CTL_L_LLP_SRC_EN);
				CLRB(lli->ctl_l, SOC_DMA_CTL_L_LLP_DST_EN);
			}
		} 
		else 
		{
			lli->llp = (uint32_t)balloc(sizeof(struct dma_lli), &err);

			if (lli->llp == 0) 
			{
				soc_dma_deconfig(channel);
				return DRV_RC_FAIL;
			}

			xfer = xfer->next;
			lli = (struct dma_lli *)lli->llp;
		}
	}

	return DRV_RC_OK;
}

/* ISR */
static void dma_interrupt_handler(void *num)
{
	uint32_t id = (uint32_t)num;
	struct dma_lli *curr = ((struct dma_lli *)(dma_info->channel[id]->curr));

	// Figure out whether this was a block or done interrupt
	if (MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_STATUS_TFR) & (1 << (SOC_DMA_STATUS_STATUS + id)))
	{
		if ((dma_info->channel[id]) && (dma_info->channel[id]->cfg.cb_done)) 
		{
			dma_info->channel[id]->cfg.cb_done(dma_info->channel[id]->cfg.cb_done_arg);
		}

		// The user's callback might have already released the channel
		if (dma_info->channel[id]) 
		{
			dma_disable(dma_info->channel[id]);
		}
	} 
	else if (MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_STATUS_BLOCK) & (1 << (SOC_DMA_STATUS_STATUS + id))) 
	{
		if ((dma_info->channel[id]) && (dma_info->channel[id]->cfg.cb_block) && (curr->end_group)) 
		{
			dma_info->channel[id]->cfg.cb_block(dma_info->channel[id]->cfg.cb_block_arg);
		}

		if (dma_info->channel[id]) 
		{
			dma_info->channel[id]->curr = ((void *)(curr->llp));
			SETB_NR(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_CLEAR_BLOCK), SOC_DMA_CLEAR_CLEAR + id);
		}
	}

	return;
}

DECLARE_INTERRUPT_HANDLER static void dma_interrupt_err_handler()
{
	uint32_t reg = MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_STATUS_ERR);

	// Loop through channels and see which have errors; calling callback and disabling
	for (int i = 0; i < SOC_DMA_NUM_CHANNELS; i++) 
	{
		if (reg & (1 << (SOC_DMA_STATUS_STATUS + i))) 
		{
			if ((dma_info->channel[i]) && (dma_info->channel[i]->cfg.cb_err)) 
			{
				dma_info->channel[i]->cfg.cb_err(dma_info->channel[i]->cfg.cb_err_arg);
			}

			if (dma_info->channel[i]) 
			{
				dma_disable(dma_info->channel[i]);
			}
		}
	}

	return;
}

/* External API */
DRIVER_API_RC soc_dma_config(struct soc_dma_channel *channel, struct soc_dma_cfg *cfg)
{
	DRIVER_API_RC ret;
	uint32_t id = channel->id;

	// Check channel to be sure its alright to configure
	if (channel == NULL) 
	{
		return DRV_RC_FAIL;
	} 
	else if (channel->active) 
	{
		return DRV_RC_CONTROLLER_IN_USE;
	}

	// Setup the config register
	CLRB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].CFG_L), SOC_DMA_CFG_L_RELOAD_DST);
	CLRB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].CFG_L), SOC_DMA_CFG_L_RELOAD_SRC);

	CLRB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].CFG_L), SOC_DMA_CFG_L_SRC_HS_POL);
	CLRB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].CFG_L), SOC_DMA_CFG_L_DST_HS_POL);

	CLRB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].CFG_L), SOC_DMA_CFG_L_HS_SEL_SRC);
	CLRB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].CFG_L), SOC_DMA_CFG_L_HS_SEL_DST);

	SETV(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].CFG_U),
		SOC_DMA_CFG_U_DEST_PER, SOC_DMA_CFG_U_DEST_PER_LEN, cfg->dest_interface);
	SETV(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].CFG_U),
		SOC_DMA_CFG_U_SRC_PER, SOC_DMA_CFG_U_SRC_PER_LEN, cfg->src_interface);

	CLRB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].CFG_U), SOC_DMA_CFG_U_SS_UPD_EN);
	CLRB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].CFG_U), SOC_DMA_CFG_U_DS_UPD_EN);

	SETB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].CFG_U), SOC_DMA_CFG_U_FIFO_MODE);
	CLRB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].CFG_U), SOC_DMA_CFG_U_FCMODE);

	// Create new LL for DMA transfer
	ret = dma_create_ll(channel, cfg);

	if (ret != DRV_RC_OK) 
	{
		return ret;
	}

	// Setup scatter-gather registers
	SETV(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].SGR),
		SOC_DMA_SGR_SGC, SOC_DMA_SGR_SGC_LEN, cfg->src_step_count);
	SETV(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].SGR),
		SOC_DMA_SGR_SGI, SOC_DMA_SGR_SGI_LEN, cfg->src_step_interval);

	SETV(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].DSR),
		SOC_DMA_DSR_DSC, SOC_DMA_DSR_DSC_LEN, cfg->dest_step_count);
	SETV(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[id].DSR),
		SOC_DMA_DSR_DSI, SOC_DMA_DSR_DSI_LEN, cfg->dest_step_interval);

	// Setup the channel structs cfg stuff
	channel->cfgd = 1;
	channel->cfg = *cfg;

	return DRV_RC_OK;
}

DRIVER_API_RC soc_dma_deconfig(struct soc_dma_channel *channel)
{
	struct dma_lli *lli;
	struct dma_lli *lli_next;
	struct dma_lli *last_lli;

	// Check channel to be sure its alright to deconfigure
	if (channel == NULL) 
	{
		return DRV_RC_FAIL;
	} 
	else if (channel->active) 
	{
		return DRV_RC_CONTROLLER_IN_USE;
	}

	// Free the link list
	lli = (struct dma_lli *)(channel->ll);
	last_lli = dma_find_ll_end(lli);

	while (lli) 
	{
		if (((struct dma_lli *)(lli->llp)) == last_lli) 
		{
			bfree(lli);
			lli = NULL;
		} 
		else 
		{
			lli_next = ((struct dma_lli *)(lli->llp));
			bfree(lli);
			lli = lli_next;
		}
	}

	channel->ll = NULL;

	channel->cfgd = 0;

	return DRV_RC_OK;
}

DRIVER_API_RC soc_dma_acquire(struct soc_dma_channel *channel)
{
	uint32_t save;

	// Get a channel from the semaphore if one is free
	save = interrupt_lock();

	// Find the first free channel and set it up in the given struct
	for (int i = 0; i < SOC_DMA_NUM_CHANNELS; i++) 
	{
		if (dma_info->channel[i] == NULL) 
		{
			dma_info->channel[i] = channel;
			channel->id = i;
			channel->active = 0;
			soc_dma_deconfig(channel);
			interrupt_unlock(save);
			return DRV_RC_OK;
		}
	}

	// Somehow, we got an item from the semaphore but found no free channels...
	interrupt_unlock(save);

	return DRV_RC_FAIL;
}

DRIVER_API_RC soc_dma_release(struct soc_dma_channel *channel)
{

	if (channel->active) 
	{
		return DRV_RC_CONTROLLER_IN_USE;
	}

	// Deconfig the channel if still configured
	if (channel->cfgd) 
	{
		soc_dma_deconfig(channel);
	}

	// Clear the channel
	dma_info->channel[channel->id] = NULL;

	return DRV_RC_OK;
}

DRIVER_API_RC soc_dma_start_transfer(struct soc_dma_channel *channel)
{
	uint32_t save;

	if (!channel->cfgd) 
	{
		return DRV_RC_FAIL;
	}

	// Setup the control register for first "transfer"
	SETB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[channel->id].CTL_L), SOC_DMA_CTL_L_LLP_SRC_EN);
	SETB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[channel->id].CTL_L), SOC_DMA_CTL_L_LLP_DST_EN);

	SETB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[channel->id].CTL_L), SOC_DMA_CTL_L_INT_EN);

	SETV(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[channel->id].CTL_L),
		SOC_DMA_CTL_L_TT_FC, SOC_DMA_CTL_L_TT_FC_LEN, channel->cfg.type);

	// Setup LLP register to point to first block
	SETV(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, dma_regs[channel->id].LLP),
		SOC_DMA_LLP_LOC, SOC_DMA_LLP_LOC_LEN, ((uint32_t)(channel->ll)) >> SOC_DMA_LLP_LOC);

	// Set the current LLI pointer to point at the head of the list
	channel->curr = channel->ll;

	save = interrupt_lock();
	dma_enable(channel);
	interrupt_unlock(save);

	return DRV_RC_OK;
}

DRIVER_API_RC soc_dma_stop_transfer(struct soc_dma_channel *channel)
{
	uint32_t save;

	save = interrupt_lock();
	dma_disable(channel);
	interrupt_unlock(save);

	return DRV_RC_OK;
}

DRIVER_API_RC soc_dma_alloc_list_item(struct soc_dma_xfer_item **ret, struct soc_dma_xfer_item *base)
{
	OS_ERR_TYPE err;

	*ret = balloc(sizeof(struct soc_dma_xfer_item), &err);

	if (*ret == NULL) 
	{
		*ret = NULL;
		return DRV_RC_FAIL;
	}

	// Copy and connect to the base if one is given
	if (base) 
	{
		**ret = *base;
		base->next = (*ret);
	}

	return DRV_RC_OK;
}

DRIVER_API_RC soc_dma_free_list(struct soc_dma_cfg *cfg)
{
	struct soc_dma_xfer_item *p;
	struct soc_dma_xfer_item *np;
	struct soc_dma_xfer_item *lp;

	p = cfg->xfer.next;
	lp = dma_find_list_end(&(cfg->xfer));

	while (p) 
	{
		if (p->next == lp) 
		{
			bfree(p);
			p = NULL;
		} 
		else 
		{
			np = p->next;
			bfree(p);
			p = np;
		}
	}

	cfg->xfer.next = NULL;

	return DRV_RC_OK;
}

/* Driver API */
DRIVER_API_RC soc_dma_init()
{
	struct soc_dma_channel ch;

	dma_info->active = 0;

	// Enable global clock
	SETB(MMIO_REG_VAL(MLAYER_AHB_CTL), 6);

	// Setup ISRs (and enable)
	for (int i = 0; i < SOC_DMA_NUM_CHANNELS; i++) 
	{
        	SET_INTERRUPT_HANDLER(dma_info->int_vector[i], dma_info->int_handler[i]);
		SOC_UNMASK_INTERRUPTS(dma_info->int_mask[i]);
	}
	SET_INTERRUPT_HANDLER(dma_info->err_vector, &dma_interrupt_err_handler);
	SOC_UNMASK_INTERRUPTS(dma_info->err_mask);

	// Setup global DMA registers settings
	SETB(MMIO_REG_VAL_FROM_BASE(SOC_DMA_BASE, SOC_DMA_DMA_CFG_REG), SOC_DMA_DMA_CFG_REG_DMA_EN);

	// Disable all channels for now and leave the channel array empty
	for (int i = 0; i < SOC_DMA_NUM_CHANNELS; i++) 
	{
		ch.id = i;
		dma_disable(&ch);
		dma_info->channel[i] = NULL;
	}

	return DRV_RC_OK;
}

