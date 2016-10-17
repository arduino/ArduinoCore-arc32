/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
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

#include <errno.h>

#include "CurieBLE.h"

#include "./internal/BLEUtils.h"

#include "BLECharacteristic.h"
#include "./internal/BLEProfileManager.h"
#include "./internal/BLEDeviceManager.h"

#include "./internal/BLECharacteristicImp.h"

BLECharacteristic::BLECharacteristic():
    _bledev(), _internal(NULL), _chrc_local_imp(NULL), _broadcast(false),
    _properties(0), _value_size(0), _value(NULL)//,
    //_event_handlers(NULL)
{
    memset(_uuid_cstr, 0, sizeof(_uuid_cstr));
    memset(_event_handlers, 0, sizeof(_event_handlers));
    memset(_oldevent_handlers, 0, sizeof(_oldevent_handlers));
}

BLECharacteristic::BLECharacteristic(const char* uuid, 
                                     unsigned char properties, 
                                     unsigned short valueSize):
    _bledev(), _internal(NULL), _chrc_local_imp(NULL), _broadcast(false),
    _properties(properties), 
    _value(NULL)//,
    //_event_handlers(NULL)
{
    bt_uuid_128 bt_uuid_tmp;
    _value_size = valueSize > BLE_MAX_ATTR_LONGDATA_LEN ? BLE_MAX_ATTR_LONGDATA_LEN : valueSize;
    BLEUtils::uuidString2BT(uuid, (bt_uuid_t *)&bt_uuid_tmp);
    BLEUtils::uuidBT2String((const bt_uuid_t *)&bt_uuid_tmp, _uuid_cstr);
    _bledev.setAddress(*BLEUtils::bleGetLoalAddress());
    memset(_event_handlers, 0, sizeof(_event_handlers));
    memset(_oldevent_handlers, 0, sizeof(_oldevent_handlers));
}

BLECharacteristic::BLECharacteristic(const char* uuid, 
                                     unsigned char properties, 
                                     const char* value):
    BLECharacteristic(uuid, properties, strlen(value))
{
    _setValue((const uint8_t*)value, strlen(value));
}

BLECharacteristic::BLECharacteristic(BLECharacteristicImp *characteristicImp,
                                     const BLEDevice *bleDev):
    _bledev(bleDev), _internal(characteristicImp),  _chrc_local_imp(NULL), 
    _broadcast(false), _value(NULL)//,_event_handlers(NULL)
{
    BLEUtils::uuidBT2String(characteristicImp->bt_uuid(), _uuid_cstr);
    _properties = characteristicImp->properties();
    _value_size = characteristicImp->valueSize();
    memset(_event_handlers, 0, sizeof(_event_handlers));
    memset(_oldevent_handlers, 0, sizeof(_oldevent_handlers));
}

BLECharacteristic::BLECharacteristic(const BLECharacteristic& rhs):
    _value(NULL)//,
    //_event_handlers(NULL)
{
    _chrc_local_imp = NULL; // Not copy
    _value_size = rhs._value_size;
    _internal = rhs._internal;
    _bledev.setAddress(*rhs._bledev.bt_le_address());
    memcpy(_uuid_cstr, rhs._uuid_cstr, sizeof(_uuid_cstr));
    _properties = rhs._properties;
    
    if (rhs._internal == NULL)
    {
        if (rhs._value != NULL)
        {
            _value = (unsigned char*)malloc(rhs._value_size);
            if (NULL != _value)
            {
                memcpy(_value, rhs._value, rhs._value_size);
            }
            else
            {
                errno = ENOMEM;
            }
            
        }
        
        //if (rhs._event_handlers != NULL)
        {
            //_event_handlers = (BLECharacteristicEventHandler*)malloc(sizeof(BLECharacteristicEventHandler) * BLECharacteristicEventLast);
            
            //if (NULL != _event_handlers)
                memcpy(_event_handlers, rhs._event_handlers, (sizeof(BLECharacteristicEventHandler) * BLECharacteristicEventLast));
        }
        memcpy(_oldevent_handlers, rhs._oldevent_handlers, (sizeof(BLECharacteristicEventHandler) * BLECharacteristicEventLast));
    }
}

BLECharacteristic::~BLECharacteristic()
{
    if (_value) 
    {
        free(_value);
        _value = NULL;
    }
    
    if (_chrc_local_imp != NULL)
    {
        delete  _chrc_local_imp;
        _chrc_local_imp = NULL;
    }
}

const char* BLECharacteristic::uuid() const
{
    return _uuid_cstr;
}

unsigned char BLECharacteristic::properties() const
{
    unsigned char property = 0;
    BLECharacteristicImp *characteristicImp = getImplementation();
    if (NULL != characteristicImp)
    {
        property = characteristicImp->properties();
    }
    return property;
}

int BLECharacteristic::valueSize() const
{
    int valuesize = 0;
    BLECharacteristicImp *characteristicImp = getImplementation();
    if (NULL != characteristicImp)
    {
        valuesize = characteristicImp->valueSize();
    }
    return valuesize;
}

const byte* BLECharacteristic::value() const
{
    const byte* value_temp = NULL;
    BLECharacteristicImp *characteristicImp = getImplementation();
    if (NULL != characteristicImp)
    {
        value_temp = characteristicImp->value();
    }
    return value_temp;
}

int BLECharacteristic::valueLength() const
{
    int valueLength = 0;
    BLECharacteristicImp *characteristicImp = getImplementation();
    if (NULL != characteristicImp)
    {
        valueLength = characteristicImp->valueLength();
    }
    return valueLength;
}

BLECharacteristic::operator bool() const
{
    return (strlen(_uuid_cstr) > 3);
}

BLECharacteristic& BLECharacteristic::operator= (const BLECharacteristic& chrc)
{
    if (this != &chrc)
    {
        memcpy(_uuid_cstr, chrc._uuid_cstr, sizeof(_uuid_cstr));
        _bledev.setAddress(*chrc._bledev.bt_le_address());
        _internal = chrc._internal;
        _chrc_local_imp = NULL; // Not copy
        _properties = chrc._properties;
        
        if (_value_size < chrc._value_size)
        {
            _value_size = chrc._value_size;
            if (NULL != _value)
            {
                free(_value);
                _value = NULL;
            }
        }
        
        if (_internal == NULL)
        {
            if (chrc._value != NULL)
            {
                if (NULL == _value)
                    _value = (unsigned char*) malloc(_value_size);
                
                if (NULL != _value)
                    memcpy(_value, chrc._value, chrc._value_size);
                else {
	                _value_size = 0;
	            }
            }
            
            //if (chrc._event_handlers != NULL)
            {
                //if (NULL == _event_handlers)
                //    _event_handlers = (BLECharacteristicEventHandler*)malloc(sizeof(BLECharacteristicEventHandler) * BLECharacteristicEventLast);
                
                //if (NULL != _event_handlers)
                    memcpy(_event_handlers, chrc._event_handlers, (sizeof(BLECharacteristicEventHandler) * BLECharacteristicEventLast));
            }
            memcpy(_oldevent_handlers, chrc._oldevent_handlers, (sizeof(BLECharacteristicEventHandler) * BLECharacteristicEventLast));
        }
    }
    return *this;
}

byte BLECharacteristic::operator[] (int offset) const
{
    byte data = 0;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        data = (*characteristicImp)[offset];
    }
    return data;
}

bool BLECharacteristic::setValue(const unsigned char value[], unsigned short length)
{
    return writeValue(value, (int)length);
}

bool BLECharacteristic::writeValue(const byte value[], int length)
{
    return writeValue(value, length, 0);
}

bool BLECharacteristic::writeValue(const byte value[], int length, int offset)
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        if (BLEUtils::isLocalBLE(_bledev) == true)
        {
            retVar = characteristicImp->writeValue(value, length, offset);
        }
        else
        {
            retVar = characteristicImp->write(value, (uint16_t)length);
        }
        
        if (true == _broadcast && 
            true == BLEDeviceManager::instance()->advertising())
        {
            BLEDeviceManager::instance()->stopAdvertising();
            BLEDeviceManager::instance()->setAdvertisedServiceData(characteristicImp->bt_uuid(),
                                                                   characteristicImp->value(), 
                                                                   characteristicImp->valueLength());
            BLEDeviceManager::instance()->startAdvertising();
        }
    }
    return retVar;
}

bool BLECharacteristic::writeValue(const char* value)
{
    return writeValue((const byte*)value, strlen(value));
}

bool BLECharacteristic::broadcast()
{
    _broadcast = true;
    BLEDeviceManager::instance()->setConnectable(false);
    if (BLEDeviceManager::instance()->advertising())
    {
        BLEDeviceManager::instance()->stopAdvertising();
        BLEDeviceManager::instance()->startAdvertising();
    }
    return _broadcast;
}

bool BLECharacteristic::written()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->written();
    }
    return retVar;
}

bool BLECharacteristic::subscribed()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->subscribed();
    }
    return retVar;
}

bool BLECharacteristic::canNotify()
{
    return (_properties & BLENotify);
}

bool BLECharacteristic::canIndicate()
{
    return (_properties & BLEIndicate);
}

bool BLECharacteristic::canRead()
{
    return (_properties & BLERead);
}

bool BLECharacteristic::canWrite()
{
    return (_properties & BLEWrite);
}

bool BLECharacteristic::canSubscribe()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    if (_properties & (BLENotify | BLEIndicate) && 
        (NULL != characteristicImp))
    {
        retVar = !characteristicImp->subscribed();
    }
    return retVar;
}

bool BLECharacteristic::canUnsubscribe()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->subscribed();
    }
    return retVar;
}

bool BLECharacteristic::read()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->read();
    }
    return retVar;
}

bool BLECharacteristic::write(const unsigned char* value, int length)
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->write(value, (uint16_t)length);
    }
    return retVar;
}

bool BLECharacteristic::subscribe()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->subscribe();
    }
    return retVar;
}

bool BLECharacteristic::unsubscribe()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->unsubscribe();
    }
    return retVar;
}

bool BLECharacteristic::valueUpdated()
{
    bool retVar = false;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->valueUpdated();
    }
    return retVar;
}

int BLECharacteristic::addDescriptor(BLEDescriptor& descriptor)
{
    int retVar = BLE_STATUS_ERROR;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        retVar = characteristicImp->addDescriptor(descriptor);
    }
    else if (BLEUtils::isLocalBLE(_bledev) == true)
    {
        // Only support the GATT server that create the service in local device.
        _chrc_local_imp = new BLECharacteristicImp(*this, _bledev);
        if (NULL == _chrc_local_imp)
        {
            retVar = BLE_STATUS_NO_MEMORY;
        }
        else
        {
            retVar = _chrc_local_imp->addDescriptor(descriptor);
        }
    }
    return retVar;
}

BLECharacteristicImp* BLECharacteristic::fetchCharacteristicImp()
{
    BLECharacteristicImp* temp = _chrc_local_imp;
    _chrc_local_imp = NULL;
    return temp;
}

int BLECharacteristic::descriptorCount() const
{
    int count = 0;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        count = characteristicImp->descriptorCount();
    }
    return count;
}

bool BLECharacteristic::hasDescriptor(const char* uuid) const
{
    BLEDescriptorImp* descriptorImp = NULL;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        descriptorImp = characteristicImp->descrptor(uuid);
    }
    
    return (descriptorImp != NULL);
}

bool BLECharacteristic::hasDescriptor(const char* uuid, int index) const
{
    bool retVal = false;
    BLEDescriptorImp* descriptorImp = NULL;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        descriptorImp = characteristicImp->descrptor(index);
        if (NULL != descriptorImp)
        {
            retVal = descriptorImp->compareUuid(uuid);
        }
    }
    
    return retVal;
}

BLEDescriptor BLECharacteristic::descriptor(int index) const
{
    BLEDescriptorImp* descriptorImp = NULL;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        descriptorImp = characteristicImp->descrptor(index);
    }
    
    if (descriptorImp != NULL)
    {
        return BLEDescriptor(descriptorImp, &_bledev);
    }
    else
    {
        return BLEDescriptor();
    }
}
BLEDescriptor BLECharacteristic::descriptor(const char * uuid) const
{
    BLEDescriptorImp* descriptorImp = NULL;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        descriptorImp = characteristicImp->descrptor(uuid);
    }
    
    if (descriptorImp != NULL)
    {
        return BLEDescriptor(descriptorImp, &_bledev);
    }
    else
    {
        return BLEDescriptor();
    }
}

BLEDescriptor BLECharacteristic::descriptor(const char * uuid, int index) const
{
    bool retVal = false;
    BLEDescriptorImp* descriptorImp = NULL;
    BLECharacteristicImp *characteristicImp = getImplementation();
    
    if (NULL != characteristicImp)
    {
        descriptorImp = characteristicImp->descrptor(index);
        if (NULL != descriptorImp)
        {
            retVal = descriptorImp->compareUuid(uuid);
        }
    }
    
    if (descriptorImp != NULL && true == retVal)
    {
        return BLEDescriptor(descriptorImp, &_bledev);
    }
    else
    {
        return BLEDescriptor();
    }
}

void BLECharacteristic::setEventHandler(BLECharacteristicEvent event, 
                                        BLECharacteristicEventHandler eventHandler)
{
    BLECharacteristicImp *characteristicImp = getImplementation();
    if (event >= BLECharacteristicEventLast)
    {
        return;
    }
    
    if (NULL != characteristicImp)
    {
        characteristicImp->setEventHandler(event, eventHandler);
    }
    else
    {
        _event_handlers[event] = eventHandler;
    }
}

void BLECharacteristic::setEventHandler(BLECharacteristicEvent event, 
                                        BLECharacteristicEventHandlerOld eventHandler)
{
    BLECharacteristicImp *characteristicImp = getImplementation();
    if (event >= BLECharacteristicEventLast)
    {
        return;
    }
    
    if (NULL != characteristicImp)
    {
        characteristicImp->setEventHandler(event, eventHandler);
    }
    else
    {
        _oldevent_handlers[event] = eventHandler;
    }
}


void
BLECharacteristic::_setValue(const uint8_t value[], uint16_t length)
{
    if (length > _value_size) {
        length = _value_size;
    }
    
    if (NULL == _value)
    {
        // Allocate the buffer for characteristic
        _value = (unsigned char*)malloc(_value_size);
    }
    if (NULL == _value)
    {
        errno = ENOMEM;
        return;
    }
    memcpy(_value, value, length);
}

BLECharacteristicImp* BLECharacteristic::getImplementation() const
{
    BLECharacteristicImp* tmp = NULL;
    tmp = _internal;
    if (NULL == tmp)
    {
        tmp = BLEProfileManager::instance()->characteristic(_bledev, (const char*)_uuid_cstr);
    }
    return tmp;
}

void BLECharacteristic::setBLECharacteristicImp(BLECharacteristicImp *characteristicImp)
{
    _internal = characteristicImp;
}


