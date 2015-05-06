/** INTEL CONFIDENTIAL Copyright 2015 Intel Corporation All Rights Reserved.
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
  * Intel’s prior express written permission.
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

#ifndef SOC_REGISTER_H_
#define SOC_REGISTER_H_

#include "scss_registers.h"

/*
 * CREG defines
 */
#define     CREG_CLK_CTRL_SPI0      (27)
#define     CREG_CLK_CTRL_SPI1      (28)
#define     CREG_CLK_CTRL_I2C0      (29)
#define     CREG_CLK_CTRL_I2C1      (30)
#define     CREG_CLK_CTRL_ADC       (31)

/* PVP */
#define PVP_REGISTER_BASE   (0xB0600000)

#define IO_REG_PVP_NCR              (PVP_REGISTER_BASE + 0x00)
#define IO_REG_PVP_COMP             (PVP_REGISTER_BASE + 0x04)
#define IO_REG_PVP_LCOMP            (PVP_REGISTER_BASE + 0x08)
#define IO_REG_PVP_IDX_DIST         (PVP_REGISTER_BASE + 0x0C)
#define IO_REG_PVP_CAT              (PVP_REGISTER_BASE + 0x10)
#define IO_REG_PVP_AIF              (PVP_REGISTER_BASE + 0x14)
#define IO_REG_PVP_MINIF            (PVP_REGISTER_BASE + 0x18)
#define IO_REG_PVP_MAXIF            (PVP_REGISTER_BASE + 0x1C)
#define IO_REG_PVP_TESTCOMP         (PVP_REGISTER_BASE + 0x20)
#define IO_REG_PVP_TESTCAT          (PVP_REGISTER_BASE + 0x24)
#define IO_REG_PVP_NID              (PVP_REGISTER_BASE + 0x28)
#define IO_REG_PVP_GCR              (PVP_REGISTER_BASE + 0x2C)
#define IO_REG_PVP_RSTCHAIN         (PVP_REGISTER_BASE + 0x30)
#define IO_REG_PVP_NSR              (PVP_REGISTER_BASE + 0x34)
#define IO_REG_PVP_FORGET_NCOUNT    (PVP_REGISTER_BASE + 0x3C)


#define IO_REG_PVP_TESTCOMP         (PVP_REGISTER_BASE + 0x20)
#define IO_REG_PVP_TESTCAT          (PVP_REGISTER_BASE + 0x24)
#define IO_REG_PVP_NID              (PVP_REGISTER_BASE + 0x28)
#define IO_REG_PVP_GCR              (PVP_REGISTER_BASE + 0x2C)
#define IO_REG_PVP_RSTCHAIN         (PVP_REGISTER_BASE + 0x30)
#define IO_REG_PVP_NSR              (PVP_REGISTER_BASE + 0x34)
#define IO_REG_PVP_FORGET_NCOUNT    (PVP_REGISTER_BASE + 0x3C)

/* DMAC Base address */
#define DMAC_REGISTER_BASE          (0xB0700000)

#define AUX_INTERRUPT_CAUSE             (0x40a)
#define AUX_EXCEPTION_CAUSE             (0x403)

#endif
