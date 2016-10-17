/*
  BLE Central API (deprecated)
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

#include "CurieBLE.h"

BLECentral::BLECentral(BLEDevice& device) :
  _device(&device)
{

}

bool BLECentral::connected(void)
{
    return _device.connected();
}

const char* BLECentral::address(void) const
{
    return _device.address().c_str();
}

bool BLECentral::disconnect(void)
{
    return _device.disconnect();
}

void BLECentral::poll(void)
{
  _device.poll();
}

BLECentral::operator bool(void) const
{
  return _device;
}

bool BLECentral::operator==(const BLECentral& rhs) const
{
  return (_device == rhs._device);
}

bool BLECentral::operator!=(const BLECentral& rhs) const
{
  return (_device != rhs._device);
}
