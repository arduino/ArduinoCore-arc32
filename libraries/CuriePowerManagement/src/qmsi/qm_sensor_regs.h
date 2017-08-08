/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SENSOR_REGISTERS_H__
#define __SENSOR_REGISTERS_H__

/**
 * Quark SE SoC Sensor Subsystem Registers.
 *
 * For detailed description please read the SOC datasheet.
 *
 * @defgroup groupSSSEREG SoC Registers (Sensor Subsystem)
 * @{
 */

#define BIT(x) (1U << (x))

/* Bitwise OR operation macro for registers in the auxiliary memory space. */
#define QM_SS_REG_AUX_OR(reg, mask)                                            \
	(__builtin_arc_sr(__builtin_arc_lr(reg) | (mask), reg))
/* Bitwise NAND operation macro for registers in the auxiliary memory space. */
#define QM_SS_REG_AUX_NAND(reg, mask)                                          \
	(__builtin_arc_sr(__builtin_arc_lr(reg) & (~(mask)), reg))

/* Sensor Subsystem status32 register. */
#define QM_SS_AUX_STATUS32 (0xA)
/** Interrupt priority threshold. */
#define QM_SS_STATUS32_E_MASK (0x1E)
/** Interrupt enable. */
#define QM_SS_STATUS32_IE_MASK BIT(31)
/* Sensor Subsystem control register. */
#define QM_SS_AUX_IC_CTRL (0x11)
/* Sensor Subsystem cache invalidate register. */
#define QM_SS_AUX_IC_IVIL (0x19)
/* Sensor Subsystem vector base register. */
#define QM_SS_AUX_INT_VECTOR_BASE (0x25)

/**
 * @name SS Interrupt
 * @{
 */

#define QM_SS_EXCEPTION_NUM (16)  /* Exceptions and traps in ARC EM core. */
#define QM_SS_INT_TIMER_NUM (2)   /* Internal interrupts in ARC EM core. */
#define QM_SS_IRQ_SENSOR_NUM (18) /* IRQ's from the Sensor Subsystem. */
#define QM_SS_IRQ_COMMON_NUM (32) /* IRQ's from the common SoC fabric. */
#define QM_SS_INT_VECTOR_NUM                                                   \
        (QM_SS_EXCEPTION_NUM + QM_SS_INT_TIMER_NUM + QM_SS_IRQ_SENSOR_NUM +    \
         QM_SS_IRQ_COMMON_NUM)
#define QM_SS_IRQ_NUM (QM_SS_IRQ_SENSOR_NUM + QM_SS_IRQ_COMMON_NUM)

/**
 * SS IRQ context type.
 *
 * Applications should not modify the content.
 * This structure is only intended to be used by
 * qm_irq_save_context and qm_irq_restore_context functions.
 */
typedef struct {
	uint32_t status32_irq_threshold; /**< STATUS32 Interrupt Threshold. */
	uint32_t status32_irq_enable;    /**< STATUS32 Interrupt Enable. */
	uint32_t irq_ctrl; /**< Interrupt Context Saving Control Register. */

	/**
	 * IRQ configuration:
	 * - IRQ Priority:BIT(6):BIT(2)
	 * - IRQ Trigger:BIT(1)
	 * - IRQ Enable:BIT(0)
	 */
	uint8_t irq_config[QM_SS_INT_VECTOR_NUM - 1];
} qm_irq_context_t;

/** General Purpose register map. */
typedef struct {
	volatile uint32_t gps0; /**< General Purpose Sticky Register 0 */
	volatile uint32_t gps1; /**< General Purpose Sticky Register 1 */
	volatile uint32_t gps2; /**< General Purpose Sticky Register 2 */
	volatile uint32_t gps3; /**< General Purpose Sticky Register 3 */
	volatile uint32_t reserved;
	volatile uint32_t gp0; /**< General Purpose Scratchpad Register 0 */
	volatile uint32_t gp1; /**< General Purpose Scratchpad Register 1 */
	volatile uint32_t gp2; /**< General Purpose Scratchpad Register 2 */
	volatile uint32_t gp3; /**< General Purpose Scratchpad Register 3 */
	volatile uint32_t reserved1;
	volatile uint32_t id;    /**< Identification Register */
	volatile uint32_t rev;   /**< Revision Register */
	volatile uint32_t wo_sp; /**< Write-One-to-Set Scratchpad Register */
	volatile uint32_t
	    wo_st; /**< Write-One-to-Set Sticky Scratchpad Register */
} qm_scss_gp_reg_t;

#define QM_SCSS_GP_BASE (0xB0800100)
#define QM_SCSS_GP ((qm_scss_gp_reg_t *)QM_SCSS_GP_BASE)

/* The GPS0 register usage. */
#define QM_GPS0_BIT_FM (0)	    /**< Start Firmware Manager. */
#define QM_GPS0_BIT_X86_WAKEUP (1)    /**< Lakemont core reset type. */
#define QM_GPS0_BIT_SENSOR_WAKEUP (2) /**< Sensor core reset type. */

/** System Core register map. */
typedef struct {
	volatile uint32_t osc0_cfg0;    /**< Hybrid Oscillator Configuration 0. */
	volatile uint32_t osc0_stat1;   /**< Hybrid Oscillator status 1. */
	volatile uint32_t osc0_cfg1;    /**< Hybrid Oscillator configuration 1. */
	volatile uint32_t osc1_stat0;   /**< RTC Oscillator status 0. */
	volatile uint32_t osc1_cfg0;    /**< RTC Oscillator Configuration 0. */
	volatile uint32_t usb_pll_cfg0; /**< USB Phase lock look configuration. */
	volatile uint32_t
	    ccu_periph_clk_gate_ctl; /**< Peripheral Clock Gate Control. */
	volatile uint32_t
	    ccu_periph_clk_div_ctl0; /**< Peripheral Clock Divider Control. 0 */
	volatile uint32_t
	    ccu_gpio_db_clk_ctl; /**< Peripheral Clock Divider Control 1. */
	volatile uint32_t
	    ccu_ext_clock_ctl; /**< External Clock Control Register. */
	/** Sensor Subsystem peripheral clock gate control. */
	volatile uint32_t ccu_ss_periph_clk_gate_ctl;
	volatile uint32_t ccu_lp_clk_ctl; /**< System Low Power Clock Control. */
	volatile uint32_t reserved;
	volatile uint32_t ccu_mlayer_ahb_ctl; /**< AHB Control Register. */
	volatile uint32_t ccu_sys_clk_ctl; /**< System Clock Control Register. */
	volatile uint32_t osc_lock_0;      /**< Clocks Lock Register. */
} qm_scss_ccu_reg_t;

#define QM_SCSS_CCU ((qm_scss_ccu_reg_t *)SCSS_REGISTER_BASE)

/** Power Management register map. */
typedef struct {
	volatile uint32_t p_lvl2; /**< Processor level 2 */
	volatile uint32_t reserved[4];
	volatile uint32_t pm1c; /**< Power management 1 control */
	volatile uint32_t reserved1[9];
	volatile uint32_t aon_vr;     /**< AON Voltage Regulator */
	volatile uint32_t plat3p3_vr; /**< Platform 3p3 voltage regulator */
	volatile uint32_t plat1p8_vr; /**< Platform 1p8 voltage regulator */
	volatile uint32_t host_vr;    /**< Host Voltage Regulator */
	volatile uint32_t slp_cfg;    /**< Sleeping Configuration */
	/** Power Management Network (PMNet) Control and Status */
	volatile uint32_t pmnetcs;
	volatile uint32_t pm_wait; /**< Power Management Wait */
	volatile uint32_t reserved2;
	volatile uint32_t p_sts; /**< Processor Status */
	volatile uint32_t reserved3[3];
	volatile uint32_t rstc; /**< Reset Control */
	volatile uint32_t rsts; /**< Reset Status */
	volatile uint32_t reserved4[6];
	volatile uint32_t vr_lock; /**< Voltage regulator lock */
	volatile uint32_t pm_lock; /**< Power Management Lock */
} qm_scss_pmu_reg_t;

#define QM_SCSS_PMU_BASE (0xB0800504)
#define QM_SCSS_PMU ((qm_scss_pmu_reg_t *)QM_SCSS_PMU_BASE)

#define QM_SS_CFG_ARC_RUN_REQ_A BIT(24)
#define QM_P_STS_HALT_INTERRUPT_REDIRECTION BIT(26)
#define QM_P_STS_ARC_HALT BIT(14)

#define QM_AON_VR_VSEL_MASK (0xFFE0)
#define QM_AON_VR_VSEL_1V2 (0x8)
#define QM_AON_VR_VSEL_1V35 (0xB)
#define QM_AON_VR_VSEL_1V8 (0x10)
#define QM_AON_VR_EN BIT(7)
#define QM_AON_VR_VSTRB BIT(5)

#define QM_SCSS_SLP_CFG_LPMODE_EN BIT(8)
#define QM_SCSS_SLP_CFG_RTC_DIS BIT(7)
#define QM_SCSS_PM1C_SLPEN BIT(13)
#define QM_SCSS_HOST_VR_EN BIT(7)
#define QM_SCSS_PLAT3P3_VR_EN BIT(7)
#define QM_SCSS_PLAT1P8_VR_EN BIT(7)
#define QM_SCSS_HOST_VR_VREG_SEL BIT(6)
#define QM_SCSS_PLAT3P3_VR_VREG_SEL BIT(6)
#define QM_SCSS_PLAT1P8_VR_VREG_SEL BIT(6)
#define QM_SCSS_VR_ROK BIT(10)
#define QM_SCSS_VR_EN BIT(7)
#define QM_SCSS_VR_VREG_SEL BIT(6)

#define QM_SCSS_CCU_SS_LPS_EN BIT(0)

typedef enum {
        QM_SS_IRQ_LEVEL_SENSITIVE = 0,
        QM_SS_IRQ_EDGE_SENSITIVE = 1
} qm_ss_irq_trigger_t;

#define QM_SS_AUX_IRQ_CTRL (0xE)
#define QM_SS_AUX_IRQ_HINT (0x201)
#define QM_SS_AUX_IRQ_PRIORITY (0x206)
#define QM_SS_AUX_IRQ_STATUS (0x406)
#define QM_SS_AUX_IRQ_SELECT (0x40B)
#define QM_SS_AUX_IRQ_ENABLE (0x40C)
#define QM_SS_AUX_IRQ_TRIGGER (0x40D)

/** Always-On Timer Interrupt. */
#define QM_IRQ_AONPT_0_INT 28
#define QM_IRQ_AONPT_0_INT_MASK_OFFSET 32
#define QM_IRQ_AONPT_0_INT_VECTOR 64

/** RTC Single Interrupt. */
#define QM_IRQ_RTC_0_INT 11
#define QM_IRQ_RTC_0_INT_MASK_OFFSET 12
#define QM_IRQ_RTC_0_INT_VECTOR 47

/** @} */

/**
 * @name SS Timer
 * @{
 */

typedef enum {
	QM_SS_TIMER_COUNT = 0,
	QM_SS_TIMER_CONTROL,
	QM_SS_TIMER_LIMIT
} qm_ss_timer_reg_t;

/**
 * Sensor Subsystem Timers.
 */
typedef enum { QM_SS_TIMER_0 = 0, QM_SS_TIMER_NUM } qm_ss_timer_t;

/*
 * SS TIMER context type.
 *
 * Application should not modify the content.
 * This structure is only intended to be used by the qm_ss_timer_save_context
 * and qm_ss_timer_restore_context functions.
 */
typedef struct {
	uint32_t timer_count;   /**< Timer count. */
	uint32_t timer_control; /**< Timer control. */
	uint32_t timer_limit;   /**< Timer limit. */
} qm_ss_timer_context_t;

#define QM_SS_TIMER_0_BASE (0x21)
#define QM_SS_TIMER_1_BASE (0x100)
#define QM_SS_TSC_BASE QM_SS_TIMER_1_BASE

#define QM_SS_TIMER_CONTROL_INT_EN_OFFSET (0)
#define QM_SS_TIMER_CONTROL_NON_HALTED_OFFSET (1)
#define QM_SS_TIMER_CONTROL_WATCHDOG_OFFSET (2)
#define QM_SS_TIMER_CONTROL_INT_PENDING_OFFSET (3)
/** @} */

/**
 * GPIO registers and definitions.
 *
 * @name SS GPIO
 * @{
 */

/** Sensor Subsystem GPIO register block type. */
typedef enum {
	QM_SS_GPIO_SWPORTA_DR = 0,
	QM_SS_GPIO_SWPORTA_DDR,
	QM_SS_GPIO_INTEN = 3,
	QM_SS_GPIO_INTMASK,
	QM_SS_GPIO_INTTYPE_LEVEL,
	QM_SS_GPIO_INT_POLARITY,
	QM_SS_GPIO_INTSTATUS,
	QM_SS_GPIO_DEBOUNCE,
	QM_SS_GPIO_PORTA_EOI,
	QM_SS_GPIO_EXT_PORTA,
	QM_SS_GPIO_LS_SYNC
} qm_ss_gpio_reg_t;

/**
 * SS GPIO context type.
 *
 * Application should not modify the content.
 * This structure is only intended to be used by the qm_ss_gpio_save_context and
 * qm_ss_gpio_restore_context functions.
 */
typedef struct {
	uint32_t gpio_swporta_dr;    /**< Port A Data. */
	uint32_t gpio_swporta_ddr;   /**< Port A Data Direction. */
	uint32_t gpio_inten;	 /**< Interrupt Enable. */
	uint32_t gpio_intmask;       /**< Interrupt Mask. */
	uint32_t gpio_inttype_level; /**< Interrupt Type. */
	uint32_t gpio_int_polarity;  /**< Interrupt Polarity. */
	uint32_t gpio_debounce;      /**< Debounce Enable. */
	uint32_t gpio_ls_sync;       /**< Synchronization Level. */
} qm_ss_gpio_context_t;

#define QM_SS_GPIO_NUM_PINS (16)
#define QM_SS_GPIO_LS_SYNC_CLK_EN BIT(31)
#define QM_SS_GPIO_LS_SYNC_SYNC_LVL BIT(0)

/** Sensor Subsystem GPIO. */
typedef enum { QM_SS_GPIO_0 = 0, QM_SS_GPIO_1, QM_SS_GPIO_NUM } qm_ss_gpio_t;

#define QM_SS_GPIO_0_BASE (0x80017800)
#define QM_SS_GPIO_1_BASE (0x80017900)

/** @} */

/**
 * I2C registers and definitions.
 *
 * @name SS I2C
 * @{
 */

/** Sensor Subsystem I2C register block type. */
typedef enum {
	QM_SS_I2C_CON = 0,
	QM_SS_I2C_DATA_CMD,
	QM_SS_I2C_SS_SCL_CNT,
	QM_SS_I2C_FS_SCL_CNT = 0x04,
	QM_SS_I2C_INTR_STAT = 0x06,
	QM_SS_I2C_INTR_MASK,
	QM_SS_I2C_TL,
	QM_SS_I2C_INTR_CLR = 0x0A,
	QM_SS_I2C_STATUS,
	QM_SS_I2C_TXFLR,
	QM_SS_I2C_RXFLR,
	QM_SS_I2C_SDA_CONFIG,
	QM_SS_I2C_TX_ABRT_SOURCE,
	QM_SS_I2C_ENABLE_STATUS = 0x11
} qm_ss_i2c_reg_t;

/**
 * SS I2C context type.
 *
 * Application should not modify the content.
 * This structure is only intended to be used by the qm_ss_gpio_save_context and
 * qm_ss_gpio_restore_context functions.
 */
typedef struct {
	uint32_t i2c_con;
	uint32_t i2c_ss_scl_cnt;
	uint32_t i2c_fs_scl_cnt;
} qm_ss_i2c_context_t;

#define QM_SS_I2C_CON_ENABLE BIT(0)
#define QM_SS_I2C_CON_ABORT BIT(1)
#define QM_SS_I2C_CON_SPEED_SS BIT(3)
#define QM_SS_I2C_CON_SPEED_FS BIT(4)
#define QM_SS_I2C_CON_SPEED_MASK (0x18)
#define QM_SS_I2C_CON_IC_10BITADDR BIT(5)
#define QM_SS_I2C_CON_IC_10BITADDR_OFFSET (5)
#define QM_SS_I2C_CON_IC_10BITADDR_MASK (5)
#define QM_SS_I2C_CON_RESTART_EN BIT(7)
#define QM_SS_I2C_CON_TAR_SAR_OFFSET (9)
#define QM_SS_I2C_CON_TAR_SAR_MASK (0x7FE00)
#define QM_SS_I2C_CON_TAR_SAR_10_BIT_MASK (0x3FF)
#define QM_SS_I2C_CON_SPKLEN_OFFSET (22)
#define QM_SS_I2C_CON_SPKLEN_MASK (0x3FC00000)
#define QM_SS_I2C_CON_CLK_ENA BIT(31)

#define QM_SS_I2C_DATA_CMD_CMD BIT(8)
#define QM_SS_I2C_DATA_CMD_STOP BIT(9)
#define QM_SS_I2C_DATA_CMD_PUSH (0xC0000000)
#define QM_SS_I2C_DATA_CMD_POP (0x80000000)

#define QM_SS_I2C_SS_FS_SCL_CNT_HCNT_OFFSET (16)
#define QM_SS_I2C_SS_FS_SCL_CNT_16BIT_MASK (0xFFFF)

#define QM_SS_I2C_INTR_STAT_RX_UNDER BIT(0)
#define QM_SS_I2C_INTR_STAT_RX_OVER BIT(1)
#define QM_SS_I2C_INTR_STAT_RX_FULL BIT(2)
#define QM_SS_I2C_INTR_STAT_TX_OVER BIT(3)
#define QM_SS_I2C_INTR_STAT_TX_EMPTY BIT(4)
#define QM_SS_I2C_INTR_STAT_TX_ABRT BIT(6)

#define QM_SS_I2C_INTR_MASK_ALL (0x0)
#define QM_SS_I2C_INTR_MASK_RX_UNDER BIT(0)
#define QM_SS_I2C_INTR_MASK_RX_OVER BIT(1)
#define QM_SS_I2C_INTR_MASK_RX_FULL BIT(2)
#define QM_SS_I2C_INTR_MASK_TX_OVER BIT(3)
#define QM_SS_I2C_INTR_MASK_TX_EMPTY BIT(4)
#define QM_SS_I2C_INTR_MASK_TX_ABRT BIT(6)

#define QM_SS_I2C_TL_TX_TL_OFFSET (16)
#define QM_SS_I2C_TL_RX_TL_MASK (0xFF)
#define QM_SS_I2C_TL_TX_TL_MASK (0xFF0000)

#define QM_SS_I2C_INTR_CLR_ALL (0xFF)
#define QM_SS_I2C_INTR_CLR_TX_ABRT BIT(6)

#define QM_SS_I2C_TX_ABRT_SOURCE_NAK_MASK (0x09)
#define QM_SS_I2C_TX_ABRT_SOURCE_ALL_MASK (0x1FFFF)
#define QM_SS_I2C_TX_ABRT_SBYTE_NORSTRT BIT(9)
#define QM_SS_I2C_TX_ABRT_SOURCE_ART_LOST BIT(12)

#define QM_SS_I2C_ENABLE_CONTROLLER_EN BIT(0)
#define QM_SS_I2C_ENABLE_STATUS_IC_EN BIT(0)

#define QM_SS_I2C_STATUS_BUSY_MASK (0x21)
#define QM_SS_I2C_STATUS_RFNE BIT(3)
#define QM_SS_I2C_STATUS_TFE BIT(2)
#define QM_SS_I2C_STATUS_TFNF BIT(1)

#define QM_SS_I2C_IC_LCNT_MAX (65525)
#define QM_SS_I2C_IC_LCNT_MIN (8)
#define QM_SS_I2C_IC_HCNT_MAX (65525)
#define QM_SS_I2C_IC_HCNT_MIN (6)

#define QM_SS_I2C_FIFO_SIZE (8)

/** Sensor Subsystem I2C */
typedef enum { QM_SS_I2C_0 = 0, QM_SS_I2C_1, QM_SS_I2C_NUM } qm_ss_i2c_t;

#define QM_SS_I2C_0_BASE (0x80012000)
#define QM_SS_I2C_1_BASE (0x80012100)

/** @} */
/** Sensor Subsystem ADC @{*/

/** Sensor Subsystem ADC registers */
typedef enum {
	QM_SS_ADC_SET = 0,    /**< ADC and sequencer settings register. */
	QM_SS_ADC_DIVSEQSTAT, /**< ADC clock and sequencer status register. */
	QM_SS_ADC_SEQ,	/**< ADC sequence entry register. */
	QM_SS_ADC_CTRL,       /**< ADC control register. */
	QM_SS_ADC_INTSTAT,    /**< ADC interrupt status register. */
	QM_SS_ADC_SAMPLE      /**< ADC sample register. */
} qm_ss_adc_reg_t;

/** Sensor Subsystem ADC */
typedef enum {
	QM_SS_ADC_0 = 0, /**< ADC first module. */
	QM_SS_ADC_NUM
} qm_ss_adc_t;

/**
 * SS ADC context type.
 *
 * The application should not modify the content of this structure.
 *
 * This structure is intented to be used by qm_ss_adc_save_context and
 * qm_ss_adc_restore_context functions only.
 */
typedef struct {
	uint32_t adc_set;	/**< ADC settings. */
	uint32_t adc_divseqstat; /**< ADC clock divider and sequencer status. */
	uint32_t adc_seq;	/**< ADC sequencer entry. */
	uint32_t adc_ctrl;       /**< ADC control. */
} qm_ss_adc_context_t;

/* SS ADC register base. */
#define QM_SS_ADC_BASE (0x80015000)

/* For 1MHz, the max divisor is 7. */
#define QM_SS_ADC_DIV_MAX (7)

#define QM_SS_ADC_FIFO_LEN (32)

#define QM_SS_ADC_SET_POP_RX BIT(31)
#define QM_SS_ADC_SET_FLUSH_RX BIT(30)
#define QM_SS_ADC_SET_THRESHOLD_MASK (0x3F000000)
#define QM_SS_ADC_SET_THRESHOLD_OFFSET (24)
#define QM_SS_ADC_SET_SEQ_ENTRIES_MASK (0x3F0000)
#define QM_SS_ADC_SET_SEQ_ENTRIES_OFFSET (16)
#define QM_SS_ADC_SET_SEQ_MODE BIT(13)
#define QM_SS_ADC_SET_SAMPLE_WIDTH_MASK (0x1F)

#define QM_SS_ADC_DIVSEQSTAT_CLK_RATIO_MASK (0x1FFFFF)

#define QM_SS_ADC_CTRL_CLR_SEQERROR BIT(19)
#define QM_SS_ADC_CTRL_CLR_UNDERFLOW BIT(18)
#define QM_SS_ADC_CTRL_CLR_OVERFLOW BIT(17)
#define QM_SS_ADC_CTRL_CLR_DATA_A BIT(16)
#define QM_SS_ADC_CTRL_MSK_SEQERROR BIT(11)
#define QM_SS_ADC_CTRL_MSK_UNDERFLOW BIT(10)
#define QM_SS_ADC_CTRL_MSK_OVERFLOW BIT(9)
#define QM_SS_ADC_CTRL_MSK_DATA_A BIT(8)
#define QM_SS_ADC_CTRL_SEQ_TABLE_RST BIT(6)
#define QM_SS_ADC_CTRL_SEQ_PTR_RST BIT(5)
#define QM_SS_ADC_CTRL_SEQ_START BIT(4)
#define QM_SS_ADC_CTRL_CLK_ENA BIT(2)
#define QM_SS_ADC_CTRL_ADC_ENA BIT(1)

#define QM_SS_ADC_CTRL_MSK_ALL_INT (0xF00)
#define QM_SS_ADC_CTRL_CLR_ALL_INT (0xF0000)

#define QM_SS_ADC_SEQ_DELAYODD_OFFSET (21)
#define QM_SS_ADC_SEQ_MUXODD_OFFSET (16)
#define QM_SS_ADC_SEQ_DELAYEVEN_OFFSET (5)

#define QM_SS_ADC_SEQ_DUMMY (0x480)

#define QM_SS_ADC_INTSTAT_SEQERROR BIT(3)
#define QM_SS_ADC_INTSTAT_UNDERFLOW BIT(2)
#define QM_SS_ADC_INTSTAT_OVERFLOW BIT(1)
#define QM_SS_ADC_INTSTAT_DATA_A BIT(0)

/** End of Sensor Subsystem ADC @}*/

/**
 * CREG Registers.
 *
 * @name SS CREG
 * @{
 */

/* Sensor Subsystem CREG */
typedef enum {
	QM_SS_IO_CREG_MST0_CTRL = 0x0,  /**< Master control register. */
	QM_SS_IO_CREG_SLV0_OBSR = 0x80, /**< Slave control register. */
	QM_SS_IO_CREG_SLV1_OBSR = 0x180 /**< Slave control register. */
} qm_ss_creg_reg_t;

/* MST0_CTRL fields */
#define QM_SS_IO_CREG_MST0_CTRL_ADC_PWR_MODE_OFFSET (1)
#define QM_SS_IO_CREG_MST0_CTRL_ADC_PWR_MODE_MASK (0x7)
#define QM_SS_IO_CREG_MST0_CTRL_ADC_DELAY_OFFSET (3)
#define QM_SS_IO_CREG_MST0_CTRL_ADC_DELAY_MASK (0xFFF8)
#define QM_SS_IO_CREG_MST0_CTRL_ADC_CAL_REQ BIT(16)
#define QM_SS_IO_CREG_MST0_CTRL_ADC_CAL_CMD_OFFSET (17)
#define QM_SS_IO_CREG_MST0_CTRL_ADC_CAL_CMD_MASK (0xE0000)
#define QM_SS_IO_CREG_MST0_CTRL_ADC_CAL_VAL_OFFSET (20)
#define QM_SS_IO_CREG_MST0_CTRL_ADC_CAL_VAL_MASK (0x7F00000)
#define QM_SS_IO_CREG_MST0_CTRL_ADC_CAL_VAL_MAX (0x7F)
#define QM_SS_IO_CREG_MST0_CTRL_SPI1_CLK_GATE BIT(27)
#define QM_SS_IO_CREG_MST0_CTRL_SPI0_CLK_GATE BIT(28)
#define QM_SS_IO_CREG_MST0_CTRL_I2C0_CLK_GATE BIT(29)
#define QM_SS_IO_CREG_MST0_CTRL_I2C1_CLK_GATE BIT(30)
#define QM_SS_IO_CREG_MST0_CTRL_ADC_CLK_GATE BIT(31)
/* SLV0_OBSR fields */
#define QM_SS_IO_CREG_SLV0_OBSR_ADC_CAL_VAL_OFFSET (5)
#define QM_SS_IO_CREG_SLV0_OBSR_ADC_CAL_VAL_MASK (0xFE0)
#define QM_SS_IO_CREG_SLV0_OBSR_ADC_CAL_ACK BIT(4)
#define QM_SS_IO_CREG_SLV0_OBSR_ADC_PWR_MODE_STS BIT(3)

#define SS_CLK_PERIPH_ALL_IN_CREG                                              \
	(SS_CLK_PERIPH_ADC | SS_CLK_PERIPH_I2C_1 | SS_CLK_PERIPH_I2C_0 |       \
	 SS_CLK_PERIPH_SPI_1 | SS_CLK_PERIPH_SPI_0)

/* SS CREG base. */
#define QM_SS_CREG_BASE (0x80018000)

/** @} */

/**
 * I2C registers and definitions.
 *
 * @name SS SPI
 * @{
 */

/** Sensor Subsystem SPI register map. */
typedef enum {
	QM_SS_SPI_CTRL = 0,   /**< SPI control register. */
	QM_SS_SPI_SPIEN = 2,  /**< SPI enable register. */
	QM_SS_SPI_TIMING = 4, /**< SPI serial clock divider value. */
	QM_SS_SPI_FTLR,       /**< Threshold value for TX/RX FIFO. */
	QM_SS_SPI_TXFLR = 7,  /**< Number of valid data entries in TX FIFO. */
	QM_SS_SPI_RXFLR,      /**< Number of valid data entries in RX FIFO. */
	QM_SS_SPI_SR,	 /**< SPI status register.	*/
	QM_SS_SPI_INTR_STAT,  /**< Interrupt status register. */
	QM_SS_SPI_INTR_MASK,  /**< Interrupt mask register. */
	QM_SS_SPI_CLR_INTR,   /**< Interrupt clear register. */
	QM_SS_SPI_DR,	 /**< RW buffer for FIFOs. */
} qm_ss_spi_reg_t;

/**
 * Sensor Subsystem SPI context type.
 *
 * Applications should not modify the content.
 * This structure is only intended to be used by
 * the qm_ss_spi_save_context and qm_ss_spi_restore_context functions.
 */
typedef struct {
	uint32_t spi_ctrl;   /**< Control Register. */
	uint32_t spi_spien;  /**< SPI Enable Register. */
	uint32_t spi_timing; /**< Timing Register. */
} qm_ss_spi_context_t;

/** Sensor Subsystem SPI modules. */
typedef enum {
	QM_SS_SPI_0 = 0, /**< SPI module 0 */
	QM_SS_SPI_1,     /**< SPI module 1 */
	QM_SS_SPI_NUM
} qm_ss_spi_t;

#define QM_SS_SPI_0_BASE (0x80010000)
#define QM_SS_SPI_1_BASE (0x80010100)

#define QM_SS_SPI_CTRL_DFS_OFFS (0)
#define QM_SS_SPI_CTRL_DFS_MASK (0x0000000F)
#define QM_SS_SPI_CTRL_BMOD_OFFS (6)
#define QM_SS_SPI_CTRL_BMOD_MASK (0x000000C0)
#define QM_SS_SPI_CTRL_SCPH BIT(6)
#define QM_SS_SPI_CTRL_SCPOL BIT(7)
#define QM_SS_SPI_CTRL_TMOD_OFFS (8)
#define QM_SS_SPI_CTRL_TMOD_MASK (0x00000300)
#define QM_SS_SPI_CTRL_SRL BIT(11)
#define QM_SS_SPI_CTRL_CLK_ENA BIT(15)
#define QM_SS_SPI_CTRL_NDF_OFFS (16)
#define QM_SS_SPI_CTRL_NDF_MASK (0xFFFF0000)

#define QM_SS_SPI_SPIEN_EN BIT(0)
#define QM_SS_SPI_SPIEN_SER_OFFS (4)
#define QM_SS_SPI_SPIEN_SER_MASK (0x000000F0)

#define QM_SS_SPI_TIMING_SCKDV_OFFS (0)
#define QM_SS_SPI_TIMING_SCKDV_MASK (0x0000FFFF)
#define QM_SS_SPI_TIMING_RSD_OFFS (16)
#define QM_SS_SPI_TIMING_RSD_MASK (0x00FF0000)

#define QM_SS_SPI_FTLR_RFT_OFFS (0)
#define QM_SS_SPI_FTLR_RFT_MASK (0x0000FFFF)
#define QM_SS_SPI_FTLR_TFT_OFFS (16)
#define QM_SS_SPI_FTLR_TFT_MASK (0xFFFF0000)

#define QM_SS_SPI_SR_BUSY BIT(0)
#define QM_SS_SPI_SR_TFNF BIT(1)
#define QM_SS_SPI_SR_TFE BIT(2)
#define QM_SS_SPI_SR_RFNE BIT(3)
#define QM_SS_SPI_SR_RFF BIT(4)

#define QM_SS_SPI_INTR_TXEI BIT(0)
#define QM_SS_SPI_INTR_TXOI BIT(1)
#define QM_SS_SPI_INTR_RXUI BIT(2)
#define QM_SS_SPI_INTR_RXOI BIT(3)
#define QM_SS_SPI_INTR_RXFI BIT(4)
#define QM_SS_SPI_INTR_ALL (0x0000001F)

#define QM_SS_SPI_INTR_STAT_TXEI QM_SS_SPI_INTR_TXEI
#define QM_SS_SPI_INTR_STAT_TXOI QM_SS_SPI_INTR_TXOI
#define QM_SS_SPI_INTR_STAT_RXUI QM_SS_SPI_INTR_RXUI
#define QM_SS_SPI_INTR_STAT_RXOI QM_SS_SPI_INTR_RXOI
#define QM_SS_SPI_INTR_STAT_RXFI QM_SS_SPI_INTR_RXFI

#define QM_SS_SPI_INTR_MASK_TXEI QM_SS_SPI_INTR_TXEI
#define QM_SS_SPI_INTR_MASK_TXOI QM_SS_SPI_INTR_TXOI
#define QM_SS_SPI_INTR_MASK_RXUI QM_SS_SPI_INTR_RXUI
#define QM_SS_SPI_INTR_MASK_RXOI QM_SS_SPI_INTR_RXOI
#define QM_SS_SPI_INTR_MASK_RXFI QM_SS_SPI_INTR_RXFI

#define QM_SS_SPI_CLR_INTR_TXEI QM_SS_SPI_INTR_TXEI
#define QM_SS_SPI_CLR_INTR_TXOI QM_SS_SPI_INTR_TXOI
#define QM_SS_SPI_CLR_INTR_RXUI QM_SS_SPI_INTR_RXUI
#define QM_SS_SPI_CLR_INTR_RXOI QM_SS_SPI_INTR_RXOI
#define QM_SS_SPI_CLR_INTR_RXFI QM_SS_SPI_INTR_RXFI

#define QM_SS_SPI_DR_DR_OFFS (0)
#define QM_SS_SPI_DR_DR_MASK (0x0000FFFF)
#define QM_SS_SPI_DR_WR BIT(30)
#define QM_SS_SPI_DR_STROBE BIT(31)
#define QM_SS_SPI_DR_W_MASK (0xc0000000)
#define QM_SS_SPI_DR_R_MASK (0x80000000)

/** @} */
/** @} */

#endif /* __SENSOR_REGISTERS_H__ */
