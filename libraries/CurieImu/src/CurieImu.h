/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef _CURIEIMU_H_
#define _CURIEIMU_H_

#include "BMI160.h"

class CurieImuClass : public BMI160Class {
    friend void bmi160_pin1_isr(void);

    public:
        void initialize(void);
        void attachInterrupt(void (*callback)(void));
        void detachInterrupt(void);

    private:
        int serial_buffer_transfer(uint8_t *buf, unsigned tx_cnt, unsigned rx_cnt);

        void (*_user_callback)(void);
};

extern CurieImuClass CurieImu;

#endif /* _CURIEIMU_H_ */
