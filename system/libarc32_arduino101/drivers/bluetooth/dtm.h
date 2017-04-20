/*
 * INTEL CONFIDENTIAL
 *
 * Copyright (c) 2016, Intel Corporation. All rights reserved.
 *
 * The source code contained or described herein and all documents related to
 * the source code ("Material") are owned by Intel Corporation or its suppliers
 * or licensors.
 * Title to the Material remains with Intel Corporation or its suppliers and
 * licensors.
 * The Material contains trade secrets and proprietary and confidential
 * information of Intel or its suppliers and licensors. The Material is
 * protected by worldwide copyright and trade secret laws and treaty provisions.
 * No part of the Material may be used, copied, reproduced, modified, published,
 * uploaded, posted, transmitted, distributed, or disclosed in any way without
 * Intel's prior express written permission.
 *
 * No license under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or delivery
 * of the Materials, either expressly, by implication, inducement, estoppel or
 * otherwise.
 *
 * Any license under such intellectual property rights must be express and
 * approved by Intel in writing
 */

#ifndef DTM_H_
#define DTM_H_

#include <stdint.h>

struct dtm_cmd_params;

struct ble_dtm_rsp {
	int status;
	uint16_t nb;
};

typedef void (*dtm_rsp_func_t)(struct ble_dtm_rsp *rsp,
					   void *user_data);

enum {
	DTM_START_RX = 0,
	DTM_START_TX,
	DTM_SET_TXPOWER,
	DTM_START_TX_CARRIER,
	DTM_END,
};

struct dtm_rx_cmd {
	uint8_t freq;
};

struct dtm_tx_cmd {
	uint8_t freq;
	uint8_t len;
	uint8_t pattern;
};

struct dtm_set_txpower_cmd {
	int8_t dbm;
};

struct dtm_cmd_params {
	uint8_t cmd_type;
	union {
		struct dtm_rx_cmd rx;
		struct dtm_tx_cmd tx;
		struct dtm_set_txpower_cmd tx_pwr;
	};
	dtm_rsp_func_t func;
	void *user_data;
};

int ble_dtm_cmd(struct dtm_cmd_params *params);

#endif
