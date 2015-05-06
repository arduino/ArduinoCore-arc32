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
  * Intel's prior express written permission.
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
/*
 * Intel common I2C header
 *
 */

#ifndef COMMON_I2C_H_
#define COMMON_I2C_H_

/**
 * \addtogroup common_driver
 * @{
 * \defgroup common_driver I2C: Inter Integrated Communication bus API
 * @{
 * \brief Definition of the structure and functions used by I2C ARC and SOC Drivers implementation.
 */

/*!<
API types
*/
typedef enum {
    I2C_OK = 0,
    I2C_BUSY,
    I2C_TX_ABORT,
    I2C_TX_OVER,
    I2C_RX_OVER,
    I2C_RX_UNDER,
}DRIVER_I2C_STATUS_CODE;

/*!
 * I2C speeds
 */
typedef enum {
    I2C_SLOW = 1,            /*!< (1) 0-100 Khz - note: Starts at 1 to remove need to translate before hardware write */
    I2C_FAST = 2,            /*!< (2) 400 Khz */
    I2C_HS = 3               /*!< (3) 3400 kbit/s mode added support for HS mode available only in SoC block */
} I2C_SPEED;

/*!
 * I2C addressing modes
 */
typedef enum {
    I2C_7_Bit = 0,           /*!< (0) 7 bit  */
    I2C_10_Bit,              /*!< (1) 10 bit */
} I2C_ADDR_MODE;

/*!
 * I2C mode types
 */
typedef enum {
    I2C_MASTER = 0,
    I2C_SLAVE
} I2C_MODE_TYPE;

/*!
*  \brief  callback function signature
*/
typedef void (*i2c_callback)( uint32_t );     /*!< callback function signature */


/*!
*  \brief   I2C controller configuration.
*           Driver instantiates one of these with given parameters for each I2C
*           controller configured using the "ss_i2c_set_config" function
*/
typedef struct i2c_cfg_data{
    I2C_SPEED           speed;
    I2C_ADDR_MODE       addressing_mode;    /*!< 7 bit / 10 bit addressing */
    I2C_MODE_TYPE       mode_type;          /*!< Master or Slave */
    uint32_t            slave_adr;          /*!< I2C address if configured as a Slave */
    i2c_callback        cb_rx;              /*!< callback function for notification of data received and available to application, NULL if callback not required by application code */
    uint32_t            cb_rx_data;         /*!< this will be passed back by the callback routine - can be used as an controller identifier */
    i2c_callback        cb_tx               /*!< callback function for notification of transmit completion by the I2C controller,  NULL if callback not required by application code */;
    uint32_t            cb_tx_data;         /*!< this will be passed back by the callback routine - can be used as an controller identifier */
    i2c_callback        cb_err;             /*!< callback function on transaction error, NULL if callback not required by application code */
    uint32_t            cb_err_data;        /*!< this will be passed back by the callback routine - can be used as an controller identifier */
}i2c_cfg_data_t;

/**@} @}*/

#endif /* COMMON_I2C_H_ */