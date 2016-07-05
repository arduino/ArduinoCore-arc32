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

#ifndef SOC_DMA_PRIV_H_
#define SOC_DMA_PRIV_H_

#include "soc_dma.h"

// Timeout on acquiring the DMA semaphore
#define SOC_DMA_SEMAPHORE_DELAY           1000

// Largest size a single DMA transfer can be (limited by BLOCK_TS bits in CTL_H)
#define SOC_DMA_BLOCK_SIZE_MAX            4095

// DMA Register map and fields
#define SOC_DMA_SAR_0                     (0x000)
#define SOC_DMA_SAR_1                     (0x058)
#define SOC_DMA_SAR_2                     (0x0B0)
#define SOC_DMA_SAR_3                     (0x108)
#define SOC_DMA_SAR_4                     (0x160)
#define SOC_DMA_SAR_5                     (0x1B8)
#define SOC_DMA_SAR_6                     (0x210)
#define SOC_DMA_SAR_7                     (0x268)
#define SOC_DMA_SAR_SAR                   (0)
#define SOC_DMA_SAR_SAR_LEN               (32)

#define SOC_DMA_DAR_0                     (0x008)
#define SOC_DMA_DAR_1                     (0x060)
#define SOC_DMA_DAR_2                     (0x0B8)
#define SOC_DMA_DAR_3                     (0x110)
#define SOC_DMA_DAR_4                     (0x168)
#define SOC_DMA_DAR_5                     (0x1C0)
#define SOC_DMA_DAR_6                     (0x218)
#define SOC_DMA_DAR_7                     (0x270)
#define SOC_DMA_DAR_DAR                   (0)
#define SOC_DMA_DAR_DAR_LEN               (32)

#define SOC_DMA_LLP_0                     (0x010)
#define SOC_DMA_LLP_1                     (0x068)
#define SOC_DMA_LLP_2                     (0x0C0)
#define SOC_DMA_LLP_3                     (0x118)
#define SOC_DMA_LLP_4                     (0x170)
#define SOC_DMA_LLP_5                     (0x1C8)
#define SOC_DMA_LLP_6                     (0x220)
#define SOC_DMA_LLP_7                     (0x278)
#define SOC_DMA_LLP_LOC                   (2)
#define SOC_DMA_LLP_LOC_LEN               (30)

#define SOC_DMA_CTL_L_0                   (0x018)
#define SOC_DMA_CTL_L_1                   (0x070)
#define SOC_DMA_CTL_L_2                   (0x0C8)
#define SOC_DMA_CTL_L_3                   (0x120)
#define SOC_DMA_CTL_L_4                   (0x178)
#define SOC_DMA_CTL_L_5                   (0x1D0)
#define SOC_DMA_CTL_L_6                   (0x228)
#define SOC_DMA_CTL_L_7                   (0x280)
#define SOC_DMA_CTL_L_LLP_SRC_EN          (28)
#define SOC_DMA_CTL_L_LLP_DST_EN          (27)
#define SOC_DMA_CTL_L_TT_FC               (20)
#define SOC_DMA_CTL_L_TT_FC_LEN           (3)
#define SOC_DMA_CTL_L_DST_SCATTER_EN      (18)
#define SOC_DMA_CTL_L_SRC_GATHER_EN       (17)
#define SOC_DMA_CTL_L_SRC_MSIZE           (14)
#define SOC_DMA_CTL_L_SRC_MSIZE_LEN       (3)
#define SOC_DMA_CTL_L_DEST_MSIZE          (11)
#define SOC_DMA_CTL_L_DEST_MSIZE_LEN      (3)
#define SOC_DMA_CTL_L_SINC                (9)
#define SOC_DMA_CTL_L_SINC_LEN            (2)
#define SOC_DMA_CTL_L_DINC                (7)
#define SOC_DMA_CTL_L_DINC_LEN            (2)
#define SOC_DMA_CTL_L_SRC_TR_WIDTH        (4)
#define SOC_DMA_CTL_L_SRC_TR_WIDTH_LEN    (3)
#define SOC_DMA_CTL_L_DST_TR_WIDTH        (1)
#define SOC_DMA_CTL_L_DST_TR_WIDTH_LEN    (3)
#define SOC_DMA_CTL_L_INT_EN              (0)

#define SOC_DMA_CTL_U_0                   (0x01C)
#define SOC_DMA_CTL_U_1                   (0x074)
#define SOC_DMA_CTL_U_2                   (0x0CC)
#define SOC_DMA_CTL_U_3                   (0x124)
#define SOC_DMA_CTL_U_4                   (0x17C)
#define SOC_DMA_CTL_U_5                   (0x1D4)
#define SOC_DMA_CTL_U_6                   (0x22C)
#define SOC_DMA_CTL_U_7                   (0x284)
#define SOC_DMA_CTL_U_DONE_BIT            (12)
#define SOC_DMA_CTL_U_BLOCK_TS            (0)
#define SOC_DMA_CTL_U_BLOCK_TS_LEN        (12)

#define SOC_DMA_CFG_L_0                   (0x040)
#define SOC_DMA_CFG_L_1                   (0x098)
#define SOC_DMA_CFG_L_2                   (0x0F0)
#define SOC_DMA_CFG_L_3                   (0x148)
#define SOC_DMA_CFG_L_4                   (0x1A0)
#define SOC_DMA_CFG_L_5                   (0x1F8)
#define SOC_DMA_CFG_L_6                   (0x250)
#define SOC_DMA_CFG_L_7                   (0x2A8)
#define SOC_DMA_CFG_L_RELOAD_DST          (31)
#define SOC_DMA_CFG_L_RELOAD_SRC          (30)
#define SOC_DMA_CFG_L_SRC_HS_POL          (19)
#define SOC_DMA_CFG_L_DST_HS_POL          (18)
#define SOC_DMA_CFG_L_HS_SEL_SRC          (11)
#define SOC_DMA_CFG_L_HS_SEL_DST          (10)
#define SOC_DMA_CFG_L_FIFO_EMPTY          (9)
#define SOC_DMA_CFG_L_CH_SUSP             (8)
#define SOC_DMA_CFG_L_CH_PRIOR            (5)
#define SOC_DMA_CFG_L_CH_PRIOR_LEN        (3)

#define SOC_DMA_CFG_U_0                   (0x044)
#define SOC_DMA_CFG_U_1                   (0x09C)
#define SOC_DMA_CFG_U_2                   (0x0F4)
#define SOC_DMA_CFG_U_3                   (0x14C)
#define SOC_DMA_CFG_U_4                   (0x1A4)
#define SOC_DMA_CFG_U_5                   (0x1FC)
#define SOC_DMA_CFG_U_6                   (0x254)
#define SOC_DMA_CFG_U_7                   (0x2AC)
#define SOC_DMA_CFG_U_DEST_PER            (11)
#define SOC_DMA_CFG_U_DEST_PER_LEN        (4)
#define SOC_DMA_CFG_U_SRC_PER             (7)
#define SOC_DMA_CFG_U_SRC_PER_LEN         (4)
#define SOC_DMA_CFG_U_SS_UPD_EN           (6)
#define SOC_DMA_CFG_U_DS_UPD_EN           (5)
#define SOC_DMA_CFG_U_PROTCTL             (2)
#define SOC_DMA_CFG_U_PROTCTL_LEN         (3)
#define SOC_DMA_CFG_U_FIFO_MODE           (1)
#define SOC_DMA_CFG_U_FCMODE              (0)

#define SOC_DMA_SGR_0                     (0x048)
#define SOC_DMA_SGR_1                     (0x0A0)
#define SOC_DMA_SGR_2                     (0x0F8)
#define SOC_DMA_SGR_3                     (0x150)
#define SOC_DMA_SGR_4                     (0x1A8)
#define SOC_DMA_SGR_5                     (0x200)
#define SOC_DMA_SGR_6                     (0x258)
#define SOC_DMA_SGR_7                     (0x2B0)
#define SOC_DMA_SGR_SGC                   (20)
#define SOC_DMA_SGR_SGC_LEN               (5)
#define SOC_DMA_SGR_SGI                   (0)
#define SOC_DMA_SGR_SGI_LEN               (20)

#define SOC_DMA_DSR_0                     (0x050)
#define SOC_DMA_DSR_1                     (0x0A8)
#define SOC_DMA_DSR_2                     (0x100)
#define SOC_DMA_DSR_3                     (0x158)
#define SOC_DMA_DSR_4                     (0x1B0)
#define SOC_DMA_DSR_5                     (0x208)
#define SOC_DMA_DSR_6                     (0x260)
#define SOC_DMA_DSR_7                     (0x2B8)
#define SOC_DMA_DSR_DSC                   (20)
#define SOC_DMA_DSR_DSC_LEN               (5)
#define SOC_DMA_DSR_DSI                   (0)
#define SOC_DMA_DSR_DSI_LEN               (20)

#define SOC_DMA_RAW_TFR                   (0x2C0)
#define SOC_DMA_RAW_BLOCK                 (0x2C8)
#define SOC_DMA_RAW_SRC_TRAN              (0x2D0)
#define SOC_DMA_RAW_DST_TRAN              (0x2D8)
#define SOC_DMA_RAW_ERR                   (0x2E0)
#define SOC_DMA_RAW_RAW                   (0)

#define SOC_DMA_STATUS_TFR                (0x2E8)
#define SOC_DMA_STATUS_BLOCK              (0x2F0)
#define SOC_DMA_STATUS_SRC_TRAN           (0x2F8)
#define SOC_DMA_STATUS_DST_TRAN           (0x300)
#define SOC_DMA_STATUS_ERR                (0x308)
#define SOC_DMA_STATUS_STATUS             (0)

#define SOC_DMA_MASK_TFR                  (0x310)
#define SOC_DMA_MASK_BLOCK                (0x318)
#define SOC_DMA_MASK_SRC_TRAN             (0x320)
#define SOC_DMA_MASK_DST_TRAN             (0x328)
#define SOC_DMA_MASK_ERR                  (0x330)
#define SOC_DMA_MASK_INT_MASK_WE          (8)
#define SOC_DMA_MASK_INT_MASK             (0)

#define SOC_DMA_CLEAR_TFR                 (0x338)
#define SOC_DMA_CLEAR_BLOCK               (0x340)
#define SOC_DMA_CLEAR_SRC_TRAN            (0x348)
#define SOC_DMA_CLEAR_DST_TRAN            (0x350)
#define SOC_DMA_CLEAR_ERR                 (0x358)
#define SOC_DMA_CLEAR_CLEAR               (0)

#define SOC_DMA_DMA_CFG_REG               (0x398)
#define SOC_DMA_DMA_CFG_REG_DMA_EN        (0)

#define SOC_DMA_CH_EN_REG                 (0x3A0)
#define SOC_DMA_CH_EN_REG_CH_EN_WE        (8)
#define SOC_DMA_CH_EN_REG_CH_EN           (0)

// DMA Register map struct, used to refer to a channel's specific registers by base name and id number
struct dma_reg_map {
	uint16_t SAR;
	uint16_t DAR;

	uint16_t LLP;

	uint16_t CTL_L;

	uint16_t CTL_U;

	uint16_t CFG_L;

	uint16_t CFG_U;

	uint16_t SGR;

	uint16_t DSR;
};

struct dma_reg_map dma_regs[SOC_DMA_NUM_CHANNELS] = {
	{
		.SAR = SOC_DMA_SAR_0,
		.DAR = SOC_DMA_DAR_0,
		.LLP = SOC_DMA_LLP_0,
		.CTL_L = SOC_DMA_CTL_L_0,
		.CTL_U = SOC_DMA_CTL_U_0,
		.CFG_L = SOC_DMA_CFG_L_0,
		.CFG_U = SOC_DMA_CFG_U_0,
		.SGR = SOC_DMA_SGR_0,
		.DSR = SOC_DMA_DSR_0
	},
	{
		.SAR = SOC_DMA_SAR_1,
		.DAR = SOC_DMA_DAR_1,
		.LLP = SOC_DMA_LLP_1,
		.CTL_L = SOC_DMA_CTL_L_1,
		.CTL_U = SOC_DMA_CTL_U_1,
		.CFG_L = SOC_DMA_CFG_L_1,
		.CFG_U = SOC_DMA_CFG_U_1,
		.SGR = SOC_DMA_SGR_1,
		.DSR = SOC_DMA_DSR_1
	},
	{
		.SAR = SOC_DMA_SAR_2,
		.DAR = SOC_DMA_DAR_2,
		.LLP = SOC_DMA_LLP_2,
		.CTL_L = SOC_DMA_CTL_L_2,
		.CTL_U = SOC_DMA_CTL_U_2,
		.CFG_L = SOC_DMA_CFG_L_2,
		.CFG_U = SOC_DMA_CFG_U_2,
		.SGR = SOC_DMA_SGR_2,
		.DSR = SOC_DMA_DSR_2
	},
	{
		.SAR = SOC_DMA_SAR_3,
		.DAR = SOC_DMA_DAR_3,
		.LLP = SOC_DMA_LLP_3,
		.CTL_L = SOC_DMA_CTL_L_3,
		.CTL_U = SOC_DMA_CTL_U_3,
		.CFG_L = SOC_DMA_CFG_L_3,
		.CFG_U = SOC_DMA_CFG_U_3,
		.SGR = SOC_DMA_SGR_3,
		.DSR = SOC_DMA_DSR_3
	},
	{
		.SAR = SOC_DMA_SAR_4,
		.DAR = SOC_DMA_DAR_4,
		.LLP = SOC_DMA_LLP_4,
		.CTL_L = SOC_DMA_CTL_L_4,
		.CTL_U = SOC_DMA_CTL_U_4,
		.CFG_L = SOC_DMA_CFG_L_4,
		.CFG_U = SOC_DMA_CFG_U_4,
		.SGR = SOC_DMA_SGR_4,
		.DSR = SOC_DMA_DSR_4
	},
	{
		.SAR = SOC_DMA_SAR_5,
		.DAR = SOC_DMA_DAR_5,
		.LLP = SOC_DMA_LLP_5,
		.CTL_L = SOC_DMA_CTL_L_5,
		.CTL_U = SOC_DMA_CTL_U_5,
		.CFG_L = SOC_DMA_CFG_L_5,
		.CFG_U = SOC_DMA_CFG_U_5,
		.SGR = SOC_DMA_SGR_5,
		.DSR = SOC_DMA_DSR_5
	},
	{
		.SAR = SOC_DMA_SAR_6,
		.DAR = SOC_DMA_DAR_6,
		.LLP = SOC_DMA_LLP_6,
		.CTL_L = SOC_DMA_CTL_L_6,
		.CTL_U = SOC_DMA_CTL_U_6,
		.CFG_L = SOC_DMA_CFG_L_6,
		.CFG_U = SOC_DMA_CFG_U_6,
		.SGR = SOC_DMA_SGR_6,
		.DSR = SOC_DMA_DSR_6
	},
	{
		.SAR = SOC_DMA_SAR_7,
		.DAR = SOC_DMA_DAR_7,
		.LLP = SOC_DMA_LLP_7,
		.CTL_L = SOC_DMA_CTL_L_7,
		.CTL_U = SOC_DMA_CTL_U_7,
		.CFG_L = SOC_DMA_CFG_L_7,
		.CFG_U = SOC_DMA_CFG_U_7,
		.SGR = SOC_DMA_SGR_7,
		.DSR = SOC_DMA_DSR_7
	}
};

// DMA Link List Item as specified by the DW DMAC doc; end_group is special field used by controller
struct dma_lli {
	uint32_t sar;
	uint32_t dar;
	uint32_t llp;
	uint32_t ctl_l;
	uint32_t ctl_u;
	uint32_t dstat;
	uint32_t sstat;
	uint8_t end_group;
};

#endif /* SOC_DMA_PRIV_H_ */
