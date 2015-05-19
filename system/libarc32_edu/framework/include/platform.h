/** INTEL CONFIDENTIAL Copyright 2014 Intel Corporation All Rights Reserved.
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

#ifndef __PLATFORM_H_
#define __PLATFORM_H_

#define CPU_ID_LMT  0
#define CPU_ID_ARC  1
#define CPU_ID_BLE  2
#define CPU_ID_HOST 3
#define NUM_CPU     4

struct platform_shared_block_ {
    void * arc_start;
    void * ports;
    void * services;
    int service_mgr_port_id;
};

#define RAM_START           0xA8000000
#define shared_data ((struct platform_shared_block_ *) RAM_START)

#define SS_GPIO_SERVICE_ID  0x1001
#define SOC_GPIO_SERVICE_ID 0x1002
#define SS_ADC_SERVICE_ID   0x1003
#define LL_STOR_SERVICE_ID  0x1004

#define MSG_ID_GPIO_BASE    0x2000

unsigned int get_timestamp(void);
#endif
