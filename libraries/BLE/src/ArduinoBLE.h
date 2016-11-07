/*
  BLE API
  Copyright (c) 2016 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#ifndef ARDUINO_BLE_H
#define ARDUINO_BLE_H

#define ARDUINO_BLE_API_VERSION 10000 // version 1.0.0

class BLEDevice;
class BLECharacteristic;
class BLEDescriptor;
class BLEService;
class BLECharacteristicImp;
class BLEDescriptorImp;

#include "BLECommon.h"

#include "BLEDevice.h"
#include "BLEAttributeWithValue.h"
#include "BLECharacteristic.h"
#include "BLEDescriptor.h"
#include "BLEService.h"

#include "BLETypedCharacteristics.h"

#include "BLECentral.h"
#include "BLEPeripheral.h"

extern BLEDevice BLE;

#endif
