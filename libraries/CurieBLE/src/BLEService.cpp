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
#include "BLEService.h"

#include "./internal/BLEProfileManager.h"
#include "./internal/BLECharacteristicImp.h"

#include "./internal/BLEUtils.h"

BLEService::BLEService():_bledevice(),
    _service_imp(NULL),
    _service_local_imp(NULL)
{
    memset(_uuid_cstr, 0, sizeof (_uuid_cstr));
}

BLEService::BLEService(const char* uuid):_bledevice(),
    _service_imp(NULL),
    _service_local_imp(NULL)
{
    bt_uuid_128_t uuid_tmp;
    memset(_uuid_cstr, 0, sizeof (_uuid_cstr));
    
    BLEUtils::uuidString2BT(uuid, (bt_uuid_t *)&uuid_tmp);
    BLEUtils::uuidBT2String((const bt_uuid_t *)&uuid_tmp, _uuid_cstr);
    
    _bledevice.setAddress(*BLEUtils::bleGetLoalAddress());
}

BLEService::BLEService(const bt_uuid_t* uuid):_bledevice(),
    _service_imp(NULL),
    _service_local_imp(NULL)
{
    memset(_uuid_cstr, 0, sizeof (_uuid_cstr));
    BLEUtils::uuidBT2String(uuid, _uuid_cstr);
}

BLEService::BLEService(BLEServiceImp* serviceImp, const BLEDevice* bledev):
    _bledevice(bledev),_service_imp(serviceImp),
    _service_local_imp(NULL)
{
    memset(_uuid_cstr, 0, sizeof (_uuid_cstr));
    
    BLEUtils::uuidBT2String(serviceImp->bt_uuid(), _uuid_cstr);
}

BLEService::~BLEService()
{
    if (NULL != _service_local_imp)
    {
        delete _service_local_imp;
        _service_local_imp = NULL;
    }
}

BLEService::operator bool() const
{
    return (strlen(_uuid_cstr) > 3);
}

BLEService& BLEService::operator= (const BLEService& service)
{
    memcpy(_uuid_cstr, service._uuid_cstr, sizeof(_uuid_cstr));
    _bledevice.setAddress(*service._bledevice.bt_le_address());
    _service_imp = service._service_imp;
    _service_local_imp = NULL; // Not copy
    return *this;
}

const char* BLEService::uuid() const
{
    return _uuid_cstr;
}

int BLEService::addCharacteristic(BLECharacteristic& characteristic)
{
    BLEServiceImp* serviceImp = getServiceImp();
    int retVar = BLE_STATUS_ERROR;
    
    if (NULL != serviceImp)
    {
        retVar = serviceImp->addCharacteristic(_bledevice, characteristic);
    }
    else if (BLEUtils::isLocalBLE(_bledevice) == true)
    {
        // Only support the GATT server that create the service in local device.
        _service_local_imp = new BLEServiceImp(*this);
        if (NULL == _service_local_imp)
        {
            return BLE_STATUS_NO_MEMORY;
        }
        retVar = _service_local_imp->addCharacteristic(_bledevice, characteristic);
    }
    return retVar;
}

BLEServiceImp* BLEService::getLocalServiceImp()
{
    return _service_local_imp;
}

BLEServiceImp* BLEService::fetchOutLocalServiceImp()
{
    BLEServiceImp* temp = _service_local_imp;
    _service_local_imp = NULL;
    return temp;
}

int BLEService::characteristicCount() const
{
    int count = 0;
    BLEServiceImp* serviceImp = getServiceImp();
    if (NULL != serviceImp)
    {
        count = serviceImp->getCharacteristicCount();
    }
    return count;
}

bool BLEService::hasCharacteristic(const char* uuid) const
{
    BLECharacteristicImp* characteristicImp = NULL;
    BLEServiceImp* serviceImp = getServiceImp();
    if (NULL != serviceImp)
    {
        characteristicImp = serviceImp->characteristic(uuid);
    }
    return (NULL != characteristicImp);
}

bool BLEService::hasCharacteristic(const char* uuid, int index) const
{
    BLECharacteristicImp* characteristicImp = NULL;
    BLEServiceImp* serviceImp = getServiceImp();
    if (NULL != serviceImp)
    {
        characteristicImp = serviceImp->characteristic(index);
        if (false == characteristicImp->compareUuid(uuid))
        {
            // UUID not align
            characteristicImp = NULL;
        }
    }
    return (NULL != characteristicImp);
}

BLECharacteristic BLEService::characteristic(int index) const
{
    BLECharacteristicImp* characteristicImp = NULL;
    BLEServiceImp* serviceImp = getServiceImp();
    if (NULL != serviceImp)
    {
        characteristicImp = serviceImp->characteristic(index);
    }
    if (NULL == characteristicImp)
    {
        BLECharacteristic temp;
        return temp;
    }
    else
    {
        BLECharacteristic temp(characteristicImp, &_bledevice);
        return temp;
    }
}

BLECharacteristic BLEService::characteristic(const char * uuid) const
{
    BLECharacteristicImp* characteristicImp = NULL;
    BLEServiceImp* serviceImp = getServiceImp();
    if (NULL != serviceImp)
    {
        characteristicImp = serviceImp->characteristic(uuid);
    }
    
    if (NULL == characteristicImp)
    {
        BLECharacteristic temp;
        return temp;
    }
    else
    {
        BLECharacteristic temp(characteristicImp, &_bledevice);
        return temp;
    }
}

BLECharacteristic BLEService::characteristic(const char * uuid, int index) const
{
    BLECharacteristicImp* characteristicImp = NULL;
    BLEServiceImp* serviceImp = getServiceImp();
    if (NULL != serviceImp)
    {
        characteristicImp = serviceImp->characteristic(index);
        if (false == characteristicImp->compareUuid(uuid))
        {
            // UUID not align
            characteristicImp = NULL;
        }
    }
    if (NULL == characteristicImp)
    {
        BLECharacteristic temp;
        return temp;
    }
    else
    {
        BLECharacteristic temp(characteristicImp, &_bledevice);
        return temp;
    }
}

BLEServiceImp*  BLEService::getServiceImp()
{
    if (NULL == _service_imp)
    {
        _service_imp = BLEProfileManager::instance()->service(_bledevice, uuid());
    }
    return _service_imp;
}

BLEServiceImp*  BLEService::getServiceImp() const
{
    return _service_imp;
}

void BLEService::setServiceImp(BLEServiceImp* serviceImp)
{
    _service_imp = serviceImp;
}
    
