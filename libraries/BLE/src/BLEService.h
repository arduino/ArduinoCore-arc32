/*
  BLE Service API
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

#ifndef ARDUINO_BLE_SERVICE_H
#define ARDUINO_BLE_SERVICE_H

#include "ArduinoBLE.h"
#include "BLEDevice.h"

//class BLECharacteristic;
class BLEServiceImp;

class BLEService
{
public:
    BLEService();
    BLEService(const char* uuid);
    virtual ~BLEService();
    
    virtual operator bool() const;  // is the service valid

    const char* uuid() const;

    /**
     * @brief   Add a characteristic in service
     *
     * @param   characteristic  The characteristic want to be added to service
     *
     * @return  none
     *
     * @note  none
     */
    void addCharacteristic(BLECharacteristic& characteristic);

    /**
     * @brief   Get the number of characteristics the service has
     *
     * @param   none
     *
     * @return  none
     *
     * @note  none
     */
    int characteristicCount() const;
    
    /**
     * @brief   Does the service have a characteristic with the specified UUID
     *
     * @param   uuid    The UUID of the characteristic
     *
     * @return  bool    true - Yes.     false - No
     *
     * @note  none
     */
    bool hasCharacteristic(const char* uuid) const;
    
    /**
     * @brief   Does the service have an nth characteristic with the 
     *           specified UUID
     *
     * @param   uuid    The UUID of the characteristic
     *
     * @param   index   The index of characteristic
     *
     * @return  bool    true - Yes.     false - No
     *
     * @note  none
     */
    bool hasCharacteristic(const char* uuid, int index) const;
    
    /**
     * @brief   Return the nth characteristic of the service
     *
     * @param   index   The index of characteristic
     *
     * @return  BLECharacteristic   The characteristic
     *
     * @note  none
     */
    BLECharacteristic characteristic(int index) const;
    
    /**
     * @brief   Return the characteristic with the specified UUID
     *
     * @param   uuid    The UUID of the characteristic
     *
     * @return  BLECharacteristic   The characteristic
     *
     * @note  none
     */
    BLECharacteristic characteristic(const char * uuid) const;
    
    /**
     * @brief   return the nth characteristic with the specified UUID
     *
     * @param   uuid    The UUID of the characteristic
     *
     * @param   index   The index of characteristic
     *
     * @return  BLECharacteristic   The characteristic
     *
     * @note  none
     */
    BLECharacteristic characteristic(const char * uuid, int index) const;
    
protected:
    friend class BLEDevice;
    friend class BLEServiceImp;
    BLEService(BLEServiceImp* serviceImp, const BLEDevice* bledev);
    void setServiceImp(BLEServiceImp* serviceImp);
private:
    BLEServiceImp* getServiceImp();
    BLEServiceImp* getServiceImp() const;
    
private:
    BLEDevice _bledevice;
    BLEServiceImp* _service_imp;
    char    _uuid_cstr[37];
};

#endif
