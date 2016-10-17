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
#include "CurieBLE.h"
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
    memcpy(&_bt_addr, bleaddress, sizeof(_bt_addr));
}

BLEDevice::BLEDevice(const BLEDevice* bledevice)
{
    memcpy(&_bt_addr, bledevice->bt_le_address(), sizeof(_bt_addr));
    memcpy(&_conn_param, &bledevice->_conn_param, sizeof (_conn_param));
}

BLEDevice::BLEDevice(const BLEDevice& bledevice)
{
    memcpy(&_bt_addr, bledevice.bt_le_address(), sizeof(_bt_addr));
    memcpy(&_conn_param, &bledevice._conn_param, sizeof (_conn_param));
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
{
    BLEProfileManager::instance()->handleDisconnectedPutOffEvent();
    BLEDeviceManager::instance()->poll();
}

void BLEDevice::end()
{}

bool BLEDevice::connected() const
{
    bool link_exist = BLEDeviceManager::instance()->connected(this);
    {
    // If release the discoverd attributes, 
    //  the GATT client may has crash issue due to used release pointer
    BLEProfileManager::instance()->handleDisconnectedPutOffEvent();
    }
    return link_exist;
}

bool BLEDevice::disconnect()
{
    bool retval = BLEDeviceManager::instance()->disconnect(this);
    BLEProfileManager::instance()->handleDisconnectedPutOffEvent();
    return retval;
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
{
    BLEDeviceManager::instance()->setManufacturerData(manufacturerData, manufacturerDataLength);
}

void BLEDevice::setLocalName(const char *localName)
{
    BLEDeviceManager::instance()->setLocalName(localName);
}

void BLEDevice::setAdvertisingInterval(float advertisingInterval)
{
    BLEDeviceManager::instance()->setAdvertisingInterval(advertisingInterval);
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
    BLEServiceImp *service_imp = BLEProfileManager::instance()->addService(*this, attribute);
    if (NULL == service_imp)
    {
        return BLE_STATUS_NO_MEMORY;
    }
    return BLE_STATUS_SUCCESS;
}

int BLEDevice::advertise()
{
    preCheckProfile();
    return BLEDeviceManager::instance()->startAdvertising();
}

void BLEDevice::stopAdvertise()
{
    BLEDeviceManager::instance()->stopAdvertising();
}

BLEDevice BLEDevice::central()
{
    return BLEDeviceManager::instance()->central();
}

BLEDevice BLEDevice::peripheral()
{
    return BLEDeviceManager::instance()->peripheral();
}

BLEDevice::operator bool() const
{
    return BLEUtils::macAddressValid(_bt_addr);
}

BLEDevice& BLEDevice::operator=(const BLEDevice& device)
{
    if (this != &device)
    {
        memcpy(&(this->_bt_addr), &(device._bt_addr), sizeof (this->_bt_addr));
        memcpy(&this->_conn_param, &device._conn_param, sizeof (this->_conn_param));
    }
    return *this;
}

bool BLEDevice::operator==(const BLEDevice& device) const
{
    return (memcmp(this->_bt_addr.val, device._bt_addr.val, 6) == 0);
}

bool BLEDevice::operator!=(const BLEDevice& device) const
{
    return (memcmp(this->_bt_addr.val, device._bt_addr.val, 6) != 0);
}


bool BLEDevice::startScan(bool withDuplicates)
{
    preCheckProfile();
    if (withDuplicates)
    {
        return BLEDeviceManager::instance()->startScanningWithDuplicates();
    }
    else
    {
        return BLEDeviceManager::instance()->startScanning();
    }
}


void BLEDevice::scan()
{
    scan(false);
}

void BLEDevice::scan(bool withDuplicates)
{
    BLEDeviceManager::instance()->clearAdvertiseCritical();
    startScan(withDuplicates);
}

void BLEDevice::scanForName(String name)
{
    scanForName(name, false);
}

void BLEDevice::scanForName(String name, bool withDuplicates)
{
    BLEDeviceManager::instance()->setAdvertiseCritical(name);
    startScan(withDuplicates);
}

void BLEDevice::scanForUuid(String uuid)
{
    scanForUuid(uuid, false);
}

void BLEDevice::scanForUuid(String uuid, bool withDuplicates)
{
    BLEService service_temp(uuid.c_str());
    BLEDeviceManager::instance()->setAdvertiseCritical(service_temp);
    startScan(withDuplicates);
}

void BLEDevice::stopScan()
{
    BLEDeviceManager::instance()->stopScanning();
}

BLEDevice BLEDevice::available()
{
    BLEProfileManager::instance()->handleDisconnectedPutOffEvent();
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

bool BLEDevice::discoverAttributesByService(const char* svc_uuid)
{
    bt_uuid_128_t uuid;
    BLEUtils::uuidString2BT(svc_uuid, (bt_uuid_t *)&uuid);
    return BLEProfileManager::instance()->discoverAttributesByService(this, (const bt_uuid_t *)&uuid);
}


String BLEDevice::deviceName()
{
    return BLEDeviceManager::instance()->deviceName(this);
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
    
