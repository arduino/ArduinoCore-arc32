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
// The API in this file is in DEPRECATED MODE, please DO NOT use it for Sketch construction
#ifndef ARDUINO_CENTRAL_H
#define ARDUINO_CENTRAL_H

class BLECentral {
  public:
    bool connected(void); // is the central connected
    
    const char* address(void) const; // address of the Central in string form

    bool disconnect(void); // Disconnect the central if it is connected
    void poll(void); // Poll the central for events

    operator bool(void) const;
    bool operator==(const BLECentral& rhs) const;
    bool operator!=(const BLECentral& rhs) const;
protected:
    friend class BLECharacteristicImp;
    friend void bleBackCompatiblePeripheralConnectHandler(BLEDevice central);
    friend void bleBackCompatiblePeripheralDisconnectHandler(BLEDevice central);
    friend class BLEPeripheral;
    BLECentral(BLEDevice& device);
  private:

    BLEDevice _device;
};

#endif
