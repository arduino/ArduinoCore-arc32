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

#ifndef I2S_PRIV_H_
#define I2S_PRIV_H_

#include "soc_i2s.h"

#define I2S_TFIFO_SIZE 4
#define I2S_RFIFO_SIZE 4

#define I2S_RFIFO_THR 1
#define I2S_TFIFO_THR 1

// I2S Registers (from base register) and fields
#define SOC_I2S_CTRL                       (0x00)
#define SOC_I2S_CTRL_EN_0                  (0)
#define SOC_I2S_CTRL_EN_1                  (1)
#define SOC_I2S_CTRL_TR_CFG_0              (8)
#define SOC_I2S_CTRL_TR_CFG_1              (9)
#define SOC_I2S_CTRL_LOOP_BACK_0_1         (16)
#define SOC_I2S_CTRL_SFR_RST               (20)
#define SOC_I2S_CTRL_T_MS                  (21)
#define SOC_I2S_CTRL_R_MS                  (22)
#define SOC_I2S_CTRL_TFIFO_RST             (23)
#define SOC_I2S_CTRL_RFIFO_RST             (24)
#define SOC_I2S_CTRL_TSYNC_RST             (25)
#define SOC_I2S_CTRL_RSYNC_RST             (26)
#define SOC_I2S_CTRL_TSYNC_LOOP_BACK       (27)
#define SOC_I2S_CTRL_RSYNC_LOOP_BACK       (28)

#define SOC_I2S_STAT                       (0x04)
#define SOC_I2S_STAT_TDATA_UNDERR          (0)
#define SOC_I2S_STAT_UNDERR_CODE           (1)
#define SOC_I2S_STAT_RDATA_OVRERR          (4)
#define SOC_I2S_STAT_OVRERR_CODE           (5)
#define SOC_I2S_STAT_TFIFO_EMPTY           (8)
#define SOC_I2S_STAT_TFIFO_AEMPTY          (9)
#define SOC_I2S_STAT_TFIFO_FULL            (10)
#define SOC_I2S_STAT_TFIFO_AFULL           (11)
#define SOC_I2S_STAT_RFIFO_EMPTY           (12)
#define SOC_I2S_STAT_RFIFO_AEMPTY          (13)
#define SOC_I2S_STAT_RFIFO_FULL            (14)
#define SOC_I2S_STAT_RFIFO_AFULL           (15)

#define SOC_I2S_SRR                        (0x08)
#define SOC_I2S_SRR_TSAMPLE_RATE           (0)
#define SOC_I2S_SRR_TRESOLUTION            (11)
#define SOC_I2S_SRR_RSAMPLE_RATE           (16)
#define SOC_I2S_SRR_RRESOLUTION            (27)

#define SOC_I2S_CID_CTRL                   (0x0C)
#define SOC_I2S_CID_CTRL_STROBE_0          (0)
#define SOC_I2S_CID_CTRL_STROBE_1          (1)
#define SOC_I2S_CID_CTRL_STROBE_TS         (8)
#define SOC_I2S_CID_CTRL_STROBE_RS         (9)
#define SOC_I2S_CID_CTRL_INTREQ_MASK       (15)
#define SOC_I2S_CID_CTRL_MASK_0            (16)
#define SOC_I2S_CID_CTRL_MASK_1            (17)
#define SOC_I2S_CID_CTRL_TFIFO_EMPTY_MASK  (24)
#define SOC_I2S_CID_CTRL_TFIFO_AEMPTY_MASK (25)
#define SOC_I2S_CID_CTRL_TFIFO_FULL_MASK   (26)
#define SOC_I2S_CID_CTRL_TFIFO_AFULL_MASK  (27)
#define SOC_I2S_CID_CTRL_RFIFO_EMPTY_MASK  (28)
#define SOC_I2S_CID_CTRL_RFIFO_AEMPTY_MASK (29)
#define SOC_I2S_CID_CTRL_RFIFO_FULL_MASK   (30)
#define SOC_I2S_CID_CTRL_RFIFO_AFULL_MASK  (31)

#define SOC_I2S_TFIFO_STAT                 (0x10)

#define SOC_I2S_RFIFO_STAT                 (0x14)

#define SOC_I2S_TFIFO_CTRL                 (0x18)
#define SOC_I2S_TFIFO_CTRL_TAEMPTY_THRS    (0)
#define SOC_I2S_TFIFO_CTRL_TAFULL_THRS     (16)

#define SOC_I2S_RFIFO_CTRL                 (0x1C)
#define SOC_I2S_RFIFO_CTRL_RAEMPTY_THRS    (0)
#define SOC_I2S_RFIFO_CTRL_RAFULL_THRS     (16)

#define SOC_I2S_DEV_CONF                   (0x20)
#define SOC_I2S_DEV_CONF_TRAN_SCK_POLAR    (0)
#define SOC_I2S_DEV_CONF_TRAN_WS_POLAR     (1)
#define SOC_I2S_DEV_CONF_TRAN_ABP_ALIGN_LR (2)
#define SOC_I2S_DEV_CONF_TRAN_I2S_ALIGN_LR (3)
#define SOC_I2S_DEV_CONF_TRAN_DATA_WS_DEL  (4)
#define SOC_I2S_DEV_CONF_TRAN_WS_DSP_MODE  (5)
#define SOC_I2S_DEV_CONF_REC_SCK_POLAR     (6)
#define SOC_I2S_DEV_CONF_REC_WS_POLAR      (7)
#define SOC_I2S_DEV_CONF_REC_ABP_ALIGN_LR  (8)
#define SOC_I2S_DEV_CONF_REC_I2S_ALIGN_LR  (9)
#define SOC_I2S_DEV_CONF_REC_DATA_WS_DEL   (10)
#define SOC_I2S_DEV_CONF_REC_WS_DSP_MODE   (11)

#define SOC_I2S_DATA_REG                   (0x50)

struct i2s_channel_regs {
	// CRTL Register
	uint8_t ctrl;
	uint8_t ctrl_en;
	uint8_t ctrl_tr_cfg;
	uint8_t ctrl_ms;
	uint8_t ctrl_fifo_rst;
	uint8_t ctrl_sync_rst;
	uint8_t ctrl_sync_loopback;

	// STAT Register
	uint8_t stat;
	uint8_t stat_err;
	uint8_t stat_code;
	uint8_t stat_fifo_empty;
	uint8_t stat_fifo_aempty;
	uint8_t stat_fifo_full;
	uint8_t stat_fifo_afull;
	uint32_t stat_mask;

	// SRR Register
	uint8_t srr;
	uint8_t srr_sample_rate;
	uint8_t srr_resolution;
	uint32_t srr_mask;

	// CID CTRL Register
	uint8_t cid_ctrl;
	uint8_t cid_ctrl_strobe;
	uint8_t cid_ctrl_strobe_sync;
	uint8_t cid_ctrl_mask;
	uint8_t cid_ctrl_fifo_empty_mask;
	uint8_t cid_ctrl_fifo_aempty_mask;
	uint8_t cid_ctrl_fifo_full_mask;
	uint8_t cid_ctrl_fifo_afull_mask;

	// FIFO STAT Register
	uint8_t fifo_stat;

	// FIFO CTRL Register
	uint8_t fifo_ctrl;
	uint8_t fifo_ctrl_aempty_thr;
	uint8_t fifo_ctrl_afull_thr;

	// DEV CONF Register
	uint8_t dev_conf;
	uint8_t dev_conf_sck_polar;
	uint8_t dev_conf_ws_polar;
	uint8_t dev_conf_abp_align_lr;
	uint8_t dev_conf_i2s_align_lr;
	uint8_t dev_conf_data_ws_del;
	uint8_t dev_conf_ws_dsp_mode;
	uint32_t dev_conf_mask;

	// DATA Register
	uint8_t data_reg;
};

const struct i2s_channel_regs i2s_reg_map[I2S_NUM_CHANNELS] = {
	// TX Channel
	{
		// CRTL Register
		.ctrl = SOC_I2S_CTRL,
		.ctrl_en = SOC_I2S_CTRL_EN_0,
		.ctrl_tr_cfg = SOC_I2S_CTRL_TR_CFG_0,
		.ctrl_ms = SOC_I2S_CTRL_T_MS,
		.ctrl_fifo_rst = SOC_I2S_CTRL_TFIFO_RST,
		.ctrl_sync_rst = SOC_I2S_CTRL_TSYNC_RST,
		.ctrl_sync_loopback = SOC_I2S_CTRL_TSYNC_LOOP_BACK,

		// STAT Register
		.stat = SOC_I2S_STAT,
		.stat_err = SOC_I2S_STAT_TDATA_UNDERR,
		.stat_code = SOC_I2S_STAT_UNDERR_CODE,
		.stat_fifo_empty = SOC_I2S_STAT_TFIFO_EMPTY,
		.stat_fifo_aempty = SOC_I2S_STAT_TFIFO_AEMPTY,
		.stat_fifo_full = SOC_I2S_STAT_TFIFO_FULL,
		.stat_fifo_afull = SOC_I2S_STAT_TFIFO_AFULL,
		.stat_mask = 0x00000F0F,

		// SRR Register
		.srr = SOC_I2S_SRR,
		.srr_sample_rate = SOC_I2S_SRR_TSAMPLE_RATE,
		.srr_resolution = SOC_I2S_SRR_TRESOLUTION,
		.srr_mask = 0x0000FFFF,

		// CID CTRL Register
		.cid_ctrl = SOC_I2S_CID_CTRL,
		.cid_ctrl_strobe = SOC_I2S_CID_CTRL_STROBE_0,
		.cid_ctrl_strobe_sync = SOC_I2S_CID_CTRL_STROBE_TS,
		.cid_ctrl_mask = SOC_I2S_CID_CTRL_MASK_0,
		.cid_ctrl_fifo_empty_mask = SOC_I2S_CID_CTRL_TFIFO_EMPTY_MASK,
		.cid_ctrl_fifo_aempty_mask = SOC_I2S_CID_CTRL_TFIFO_AEMPTY_MASK,
		.cid_ctrl_fifo_full_mask = SOC_I2S_CID_CTRL_TFIFO_FULL_MASK,
		.cid_ctrl_fifo_afull_mask = SOC_I2S_CID_CTRL_TFIFO_AFULL_MASK,

		// FIFO STAT Register
		.fifo_stat = SOC_I2S_TFIFO_STAT,

		// FIFO CTRL Register
		.fifo_ctrl = SOC_I2S_TFIFO_CTRL,
		.fifo_ctrl_aempty_thr = SOC_I2S_TFIFO_CTRL_TAEMPTY_THRS,
		.fifo_ctrl_afull_thr = SOC_I2S_TFIFO_CTRL_TAFULL_THRS,

		// DEV CONF Register
		.dev_conf = SOC_I2S_DEV_CONF,
		.dev_conf_sck_polar = SOC_I2S_DEV_CONF_TRAN_SCK_POLAR,
		.dev_conf_ws_polar = SOC_I2S_DEV_CONF_TRAN_WS_POLAR,
		.dev_conf_abp_align_lr = SOC_I2S_DEV_CONF_TRAN_ABP_ALIGN_LR,
		.dev_conf_i2s_align_lr = SOC_I2S_DEV_CONF_TRAN_I2S_ALIGN_LR,
		.dev_conf_data_ws_del = SOC_I2S_DEV_CONF_TRAN_DATA_WS_DEL,
		.dev_conf_ws_dsp_mode = SOC_I2S_DEV_CONF_TRAN_WS_DSP_MODE,
		.dev_conf_mask = 0x0000003F,

		// DATA Register
		.data_reg = SOC_I2S_DATA_REG
	},

	// RX Channel
	{
		// CRTL Register
		.ctrl = SOC_I2S_CTRL,
		.ctrl_en = SOC_I2S_CTRL_EN_1,
		.ctrl_tr_cfg = SOC_I2S_CTRL_TR_CFG_1,
		.ctrl_ms = SOC_I2S_CTRL_R_MS,
		.ctrl_fifo_rst = SOC_I2S_CTRL_RFIFO_RST,
		.ctrl_sync_rst = SOC_I2S_CTRL_RSYNC_RST,
		.ctrl_sync_loopback = SOC_I2S_CTRL_RSYNC_LOOP_BACK,

		// STAT Register
		.stat = SOC_I2S_STAT,
		.stat_err = SOC_I2S_STAT_RDATA_OVRERR,
		.stat_code = SOC_I2S_STAT_OVRERR_CODE,
		.stat_fifo_empty = SOC_I2S_STAT_RFIFO_EMPTY,
		.stat_fifo_aempty = SOC_I2S_STAT_RFIFO_AEMPTY,
		.stat_fifo_full = SOC_I2S_STAT_RFIFO_FULL,
		.stat_fifo_afull = SOC_I2S_STAT_RFIFO_AFULL,
		.stat_mask = 0x0000F0F0,

		// SRR Register
		.srr = SOC_I2S_SRR,
		.srr_sample_rate = SOC_I2S_SRR_RSAMPLE_RATE,
		.srr_resolution = SOC_I2S_SRR_RRESOLUTION,
		.srr_mask = 0xFFFF0000,

		// CID CTRL Register
		.cid_ctrl = SOC_I2S_CID_CTRL,
		.cid_ctrl_strobe = SOC_I2S_CID_CTRL_STROBE_1,
		.cid_ctrl_strobe_sync = SOC_I2S_CID_CTRL_STROBE_RS,
		.cid_ctrl_mask = SOC_I2S_CID_CTRL_MASK_1,
		.cid_ctrl_fifo_empty_mask = SOC_I2S_CID_CTRL_RFIFO_EMPTY_MASK,
		.cid_ctrl_fifo_aempty_mask = SOC_I2S_CID_CTRL_RFIFO_AEMPTY_MASK,
		.cid_ctrl_fifo_full_mask = SOC_I2S_CID_CTRL_RFIFO_FULL_MASK,
		.cid_ctrl_fifo_afull_mask = SOC_I2S_CID_CTRL_RFIFO_AFULL_MASK,

		// FIFO STAT Register
		.fifo_stat = SOC_I2S_RFIFO_STAT,

		// FIFO CTRL Register
		.fifo_ctrl = SOC_I2S_RFIFO_CTRL,
		.fifo_ctrl_aempty_thr = SOC_I2S_RFIFO_CTRL_RAEMPTY_THRS,
		.fifo_ctrl_afull_thr = SOC_I2S_RFIFO_CTRL_RAFULL_THRS,

		// DEV CONF Register
		.dev_conf = SOC_I2S_DEV_CONF,
		.dev_conf_sck_polar = SOC_I2S_DEV_CONF_REC_SCK_POLAR,
		.dev_conf_ws_polar = SOC_I2S_DEV_CONF_REC_WS_POLAR,
		.dev_conf_abp_align_lr = SOC_I2S_DEV_CONF_REC_ABP_ALIGN_LR,
		.dev_conf_i2s_align_lr = SOC_I2S_DEV_CONF_REC_I2S_ALIGN_LR,
		.dev_conf_data_ws_del = SOC_I2S_DEV_CONF_REC_DATA_WS_DEL,
		.dev_conf_ws_dsp_mode = SOC_I2S_DEV_CONF_REC_WS_DSP_MODE,
		.dev_conf_mask = 0x00000FC0,

		// DATA Register
		.data_reg = SOC_I2S_DATA_REG
	}
};

#endif /* I2S_PRIV_H_ */
