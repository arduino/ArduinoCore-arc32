/*
  BLE Peripheral API (deprecated)
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

#include "BLEPeripheral.h"

static BLEPeripheralEventHandler m_eventHandlers[BLEDeviceLastEvent];

void bleBackCompatiblePeripheralConnectHandler(BLEDevice central) 
{
    if (m_eventHandlers[BLEConnected])
    {
        BLECentral temp(central);
        m_eventHandlers[BLEConnected](temp);
    }
}

void bleBackCompatiblePeripheralDisconnectHandler(BLEDevice central) 
{
    if (m_eventHandlers[BLEDisconnected])
    {
        BLECentral temp(central);
        m_eventHandlers[BLEDisconnected](temp);
    }
}


BLEPeripheral::BLEPeripheral(void) :
  _initCalled(false),
  _lastService(NULL),
  _lastCharacteristic(NULL)
{
}

BLEPeripheral::~BLEPeripheral(void)
{
}

void BLEPeripheral::setAdvertisedServiceUuid(const char* advertisedServiceUuid)
{
    if (!_initCalled) {
        init();
    }

    BLE.setAdvertisedServiceUuid(advertisedServiceUuid);
}
void BLEPeripheral::setLocalName(const char* localName)
{
    if (!_initCalled) {
        init();
    }

    BLE.setLocalName(localName);
}


void BLEPeripheral::setDeviceName(const char *deviceName)
{
    if (!_initCalled) {
        init();
    }

    BLE.setDeviceName(deviceName);
}

void BLEPeripheral::setAppearance(const unsigned short appearance)
{
    if (!_initCalled) {
        init();
    }

    BLE.setAppearance(appearance);
}

void BLEPeripheral::setConnectionInterval(const unsigned short minConnInterval, const unsigned short maxConnInterval)
{
    if (!_initCalled) {
        init();
    }

    BLE.setConnectionInterval(minConnInterval, maxConnInterval);
}

void BLEPeripheral::addAttribute(BLEService& service)
{
    if (!_initCalled)
    {
        init();
    }
    
    BLE.addService(service);
    _lastService = &service;
}

void BLEPeripheral::addAttribute(BLECharacteristic& characteristic)
{
    if (!_initCalled)
    {
        init();
    }

    if (_lastService)
    {
        _lastService->addCharacteristic(characteristic);
        _lastCharacteristic = &characteristic;
    }
}

void BLEPeripheral::addAttribute(BLEDescriptor& descriptor)
{
    if (!_initCalled)
    {
        init();
    }

    if (_lastCharacteristic)
    {
        _lastCharacteristic->addDescriptor(descriptor);
    }
}

void BLEPeripheral::setEventHandler(BLEPeripheralEvent event, BLEPeripheralEventHandler callback)
{
    if (BLEConnected == event || BLEDisconnected == event)
    {
        m_eventHandlers[event] = callback;
    }
}

bool BLEPeripheral::begin(void)
{
    if (!_initCalled)
    {
        init();
    }
    
    BLE.setEventHandler(BLEDisconnected, bleBackCompatiblePeripheralDisconnectHandler);
    BLE.setEventHandler(BLEConnected, bleBackCompatiblePeripheralConnectHandler);

    BLE.advertise();
    return true;
}

void BLEPeripheral::poll(void)
{
    BLE.poll();
}

void BLEPeripheral::end(void)
{
    BLE.end();
}

bool BLEPeripheral::disconnect(void)
{
    return BLE.disconnect();
}

BLECentral BLEPeripheral::central(void)
{
    BLEDevice centralBle = BLE.central();
    return BLECentral(centralBle);
}

bool BLEPeripheral::connected(void)
{
    return BLE.connected();
}

void BLEPeripheral::init()
{
    if (!_initCalled)
    {
        BLE.begin();
        memset(m_eventHandlers, 0, sizeof(m_eventHandlers));
        _initCalled = true;
    }
}

