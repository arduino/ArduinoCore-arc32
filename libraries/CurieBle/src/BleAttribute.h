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

#ifndef _BLE_ATTRIBUTE_H_INCLUDED
#define _BLE_ATTRIBUTE_H_INCLUDED

enum BleAttributeType {
  BleTypeService        = 0x2800,
  BleTypeCharacteristic = 0x2803,
  BleTypeDescriptor     = 0x2900
};

class BleAttribute {
public:
    BleAttribute(enum BleAttributeType type);

    enum BleAttributeType type() const;


protected:
    static unsigned char numAttributes();

private:
    enum BleAttributeType _type;
    static unsigned char  _numAttributes;
};

#endif // _BLE_ATTRIBUTE_H_INCLUDED