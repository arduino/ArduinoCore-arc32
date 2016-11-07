/*
  BLE Device API
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
#include "ArduinoBLE.h"
#include "BLEDevice.h"

#include "./internal/BLEUtils.h"

#include "./internal/BLEProfileManager.h"
#include "./internal/BLEDeviceManager.h"
#include "./internal/BLECharacteristicImp.h"

BLEDevice::BLEDevice()
{
    memset(&_bt_addr, 0, sizeof(_bt_addr));
    _conn_param.interval_max = BT_GAP_INIT_CONN_INT_MAX;
    _conn_param.interval_min = BT_GAP_INIT_CONN_INT_MIN;
    _conn_param.latency = 0;
    _conn_param.timeout = 400;
}

/*
BLEDevice::BLEDevice(String bleaddress)
{
    BLEUtils::macAddressString2BT(bleaddress.c_str(), _bt_addr);
}

BLEDevice::BLEDevice(const char* bleaddress)
{
    BLEUtils::macAddressString2BT(bleaddress, _bt_addr);
}

*/

BLEDevice::BLEDevice(const bt_addr_le_t* bleaddress):
    BLEDevice()
{
    memcpy(&_bt_addr, bleaddress, sizeof(bt_addr_le_t));
}

BLEDevice::BLEDevice(const BLEDevice* bledevice)
{
    memcpy(&_bt_addr, bledevice->bt_le_address(), sizeof(bt_addr_le_t));
    memcpy(&_conn_param, &bledevice->_conn_param, sizeof (ble_conn_param_t));
}

BLEDevice::~BLEDevice()
{
    //pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
}

bool BLEDevice::begin()
{
    return BLEDeviceManager::instance()->begin(this);
}

void BLEDevice::poll()
{}

void BLEDevice::end()
{}

bool BLEDevice::connected()
{
    return BLEDeviceManager::instance()->connected(this);
}

bool BLEDevice::disconnect()
{
    return BLEDeviceManager::instance()->disconnect(this);
}

String BLEDevice::address() const
{
    return BLEUtils::macAddressBT2String(_bt_addr);
}

void BLEDevice::setAddress(const bt_addr_le_t& addr)
{
    memcpy(&_bt_addr, &addr, sizeof(_bt_addr));
}

void BLEDevice::setAdvertisedServiceUuid(const char* advertisedServiceUuid)
{
    BLEDeviceManager::instance()->setAdvertisedServiceUuid(advertisedServiceUuid);
}

void BLEDevice::setAdvertisedService(const BLEService& service)
{
    setAdvertisedServiceUuid(service.uuid());
}

void BLEDevice::setServiceSolicitationUuid(const char* serviceSolicitationUuid)
{
    BLEDeviceManager::instance()->setServiceSolicitationUuid(serviceSolicitationUuid);
}

void BLEDevice::setManufacturerData(const unsigned char manufacturerData[], 
                                    unsigned char manufacturerDataLength)
{}

void BLEDevice::setLocalName(const char *localName)
{
    BLEDeviceManager::instance()->setLocalName(localName);
}

void BLEDevice::setAdvertisingInterval(float advertisingInterval)
{
    BLEDeviceManager::instance()->setAdvertisingInterval(advertisingInterval);
}

void BLEDevice::setConnectionInterval(int minimumConnectionInterval, 
                                      int maximumConnectionInterval,
                                      uint16_t latency, 
                                      uint16_t timeout)
{
    // TODO: Update the connection interval need more discussion
}

void BLEDevice::setConnectionInterval(int minimumConnectionInterval, 
                                      int maximumConnectionInterval)
{
    // TODO: Update the connection interval need more discussion

}

bool BLEDevice::setTxPower(int txPower)
{
    return BLEDeviceManager::instance()->setTxPower(txPower);
}

void BLEDevice::setConnectable(bool connectable)
{
    BLEDeviceManager::instance()->setConnectable(connectable);
}

void BLEDevice::setDeviceName(const char* deviceName)
{
    BLEDeviceManager::instance()->setDeviceName(deviceName);
}

void BLEDevice::setAppearance(unsigned short appearance)
{
    BLEDeviceManager::instance()->setAppearance(appearance);
}

int BLEDevice::addService(BLEService& attribute)
{
    return BLEProfileManager::instance()->addService(*this, attribute);
}

int BLEDevice::startAdvertising()
{
    preCheckProfile();
    return BLEDeviceManager::instance()->startAdvertising();
}

void BLEDevice::stopAdvertising()
{
    BLEDeviceManager::instance()->stopAdvertising();
}

BLEDevice BLEDevice::central()
{
    return BLEDeviceManager::instance()->central();
}

BLEDevice BLEDevice::peripheral()
{
    // TODO: How to get the target devices
    BLEDevice temp;
    return temp;
}

BLEDevice::operator bool() const
{
    return BLEUtils::macAddressValid(_bt_addr);
}

//BLEDevice& BLEDevice::operator=(const BLEDevice& device)
//{
//    if (*this != device)
//    {
//        memcpy(&(this->_bt_addr), &(device._bt_addr), sizeof (bt_addr_le_t));
//    }
//    return *this;
//}

bool BLEDevice::operator==(const BLEDevice& device) const
{
    return (memcmp(this->_bt_addr.val, device._bt_addr.val, 6) == 0);
}

bool BLEDevice::operator!=(const BLEDevice& device) const
{
    return (memcmp(this->_bt_addr.val, device._bt_addr.val, 6) != 0);
}


void BLEDevice::startScanning()
{
    preCheckProfile();
    BLEDeviceManager::instance()->clearAdvertiseCritical();
    BLEDeviceManager::instance()->startScanning();
}

void BLEDevice::startScanning(String name)
{
    preCheckProfile();
    BLEDeviceManager::instance()->setAdvertiseCritical(name);
    BLEDeviceManager::instance()->startScanning();
}

void BLEDevice::startScanning(BLEService& service)
{
    preCheckProfile();
    BLEDeviceManager::instance()->setAdvertiseCritical(service);
    BLEDeviceManager::instance()->startScanning();
}

void BLEDevice::startScanningWithDuplicates()
{
    // TODO
}

void BLEDevice::stopScanning()
{
    BLEDeviceManager::instance()->stopScanning();
}

BLEDevice BLEDevice::available()
{
    return BLEDeviceManager::instance()->available();
}

bool BLEDevice::hasLocalName() const
{
    return BLEDeviceManager::instance()->hasLocalName(this);
}

bool BLEDevice::hasAdvertisedServiceUuid() const
{
    return BLEDeviceManager::instance()->hasAdvertisedServiceUuid(this);
}

bool BLEDevice::hasAdvertisedServiceUuid(int index) const
{
    return BLEDeviceManager::instance()->hasAdvertisedServiceUuid(this, index);
}

int BLEDevice::advertisedServiceUuidCount() const
{
    return BLEDeviceManager::instance()->advertisedServiceUuidCount(this);
}

String BLEDevice::localName() const
{
    return BLEDeviceManager::instance()->localName(this);
}

String BLEDevice::advertisedServiceUuid() const
{
    return BLEDeviceManager::instance()->advertisedServiceUuid(this);
}

String BLEDevice::advertisedServiceUuid(int index) const
{
    return BLEDeviceManager::instance()->advertisedServiceUuid(this, index);
}

int BLEDevice::rssi() const
{
    return BLEDeviceManager::instance()->rssi(this);
}

bool BLEDevice::connect()
{
    return BLEDeviceManager::instance()->connect(*this);
}

bool BLEDevice::discoverAttributes()
{
    return BLEProfileManager::instance()->discoverAttributes(this);
}

String BLEDevice::deviceName()
{
    return BLEDeviceManager::instance()->deviceName();
}

int BLEDevice::appearance()
{
    return BLEDeviceManager::instance()->appearance();
}

// For GATT
int BLEDevice::serviceCount() const
{
    return BLEProfileManager::instance()->serviceCount(*this);
}

bool BLEDevice::hasService(const char* uuid) const
{
    BLEServiceImp* serviceImp = BLEProfileManager::instance()->service(*this, uuid);
    return (NULL != serviceImp);
}

bool BLEDevice::hasService(const char* uuid, int index) const
{
    BLEServiceImp* serviceImp = BLEProfileManager::instance()->service(*this, index);
    return serviceImp->compareUuid(uuid);
}

BLEService BLEDevice::service(int index) const
{
    BLEServiceImp* serviceImp = BLEProfileManager::instance()->service(*this, index);
    if (serviceImp != NULL)
    {
        BLEService temp(serviceImp, this);
        return temp;
    }
    BLEService temp;
    return temp;
}

BLEService BLEDevice::service(const char * uuid) const
{
    BLEServiceImp* serviceImp = BLEProfileManager::instance()->service(*this, uuid);
    if (serviceImp != NULL)
    {
        BLEService temp(serviceImp, this);
        return temp;
    }
    BLEService temp;
    return temp;
}

BLEService BLEDevice::service(const char * uuid, int index) const
{
    BLEServiceImp* serviceImp = BLEProfileManager::instance()->service(*this, index);
    if (serviceImp != NULL && serviceImp->compareUuid(uuid))
    {
        BLEService temp(serviceImp, this);
        return temp;
    }
    BLEService temp;
    return temp;
}

int BLEDevice::characteristicCount() const
{
    return BLEProfileManager::instance()->characteristicCount(*this);
}

bool BLEDevice::hasCharacteristic(const char* uuid) const
{
    BLECharacteristicImp* characteristicImp = BLEProfileManager::instance()->characteristic(*this, uuid);
    return (NULL != characteristicImp);
}

bool BLEDevice::hasCharacteristic(const char* uuid, int index) const
{
    BLECharacteristicImp* characteristicImp = BLEProfileManager::instance()->characteristic(*this, uuid, index);
    return (NULL != characteristicImp);
}

BLECharacteristic BLEDevice::characteristic(int index) const
{
    BLECharacteristicImp* characteristicImp = BLEProfileManager::instance()->characteristic(*this, index);
    
    if (NULL == characteristicImp)
    {
        BLECharacteristic temp;
        return temp;
    }
    BLECharacteristic temp(characteristicImp, this);
    return temp;
}

BLECharacteristic BLEDevice::characteristic(const char * uuid) const
{
    BLECharacteristicImp* characteristicImp = BLEProfileManager::instance()->characteristic(*this, uuid);
    
    if (NULL == characteristicImp)
    {
        BLECharacteristic temp;
        return temp;
    }
    BLECharacteristic temp(characteristicImp, this);
    return temp;
}

BLECharacteristic BLEDevice::characteristic(const char * uuid, int index) const
{
    BLECharacteristicImp* characteristicImp = BLEProfileManager::instance()->characteristic(*this, index);
    if (false == characteristicImp->compareUuid(uuid))
    {
        // UUID not matching
        characteristicImp = NULL;
    }
    
    if (NULL == characteristicImp)
    {
        BLECharacteristic temp;
        return temp;
    }
    BLECharacteristic temp(characteristicImp, this);
    return temp;
}

// event handler
void BLEDevice::setEventHandler(BLEDeviceEvent event, 
                                BLEDeviceEventHandler eventHandler)
{
    BLEDeviceManager::instance()->setEventHandler(event, eventHandler);
}

const bt_addr_le_t* BLEDevice::bt_le_address() const
{
    return &_bt_addr;
}
const bt_le_conn_param* BLEDevice::bt_conn_param() const
{
    return &_conn_param;
}

void BLEDevice::preCheckProfile()
{
    if (false == BLEProfileManager::instance()->hasRegisterProfile() &&
        BLEProfileManager::instance()->serviceCount(*this) > 0)
    {
        BLEProfileManager::instance()->registerProfile(*this);
        delay(8); 
    }
}
    
