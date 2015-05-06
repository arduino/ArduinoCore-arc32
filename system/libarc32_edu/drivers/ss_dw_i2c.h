/*
*
* CONFIDENTIAL AND PROPRIETARY INFORMATION
*
* Copyright (c) 2013 Synopsys, Inc. All rights reserved.
* This software and documentation contain confidential and
* proprietary information that is the property of
* Synopsys, Inc. The software and documentation are
* furnished under a license agreement and may be used
* or copied only in accordance with the terms of the license
* agreement. No part of the software and documentation
* may be reproduced, transmitted, or translated, in any
* form or by any means, electronic, mechanical, manual,
* optical, or otherwise, without prior written permission
* of Synopsys, Inc., or as expressly provided by the license agreement.
* Reverse engineering is prohibited, and reproduction,
* disclosure or use without specific written authorization
* of Synopsys Inc. is strictly forbidden.
*/


#ifndef SS_DW_I2C_H_
#define SS_DW_I2C_H_

void i2c_mst_err_ISR_proc(i2c_info_pt dev);
void i2c_fill_fifo(i2c_info_pt dev);
void i2c_mst_rx_avail_ISR_proc(i2c_info_pt dev);
void i2c_mst_tx_req_ISR_proc(i2c_info_pt dev);
void i2c_mst_stop_detected_ISR_proc(i2c_info_pt dev);

#endif  // SS_DW_I2C_H
