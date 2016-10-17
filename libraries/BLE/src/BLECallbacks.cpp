

#include <errno.h>

#include "ArduinoBLE.h"

#include "BLEAttribute.h"
#include "BLECharacteristicImp.h"
#include "BLEDeviceManager.h"
#include "BLEProfileManager.h"

// GATT Server Only
ssize_t profile_read_process(bt_conn_t *conn,
                             const bt_gatt_attr_t *attr,
                             void *buf, uint16_t len,
                             uint16_t offset)
{
    const unsigned char *pvalue;
    BLEAttribute *bleattr = (BLEAttribute *)attr->user_data;
    BLEAttributeType type = bleattr->type();
    if (BLETypeCharacteristic == type)
    {
        BLECharacteristicImp* blecharacteritic = (BLECharacteristicImp*)bleattr;
        pvalue = blecharacteritic->value();
        return bt_gatt_attr_read(conn, attr, buf, len, offset, pvalue,
                                 blecharacteritic->valueLength());
    }
    else if (BLETypeDescriptor == type)
    {
        BLEDescriptorImp* bledescriptor = (BLEDescriptorImp*)bleattr;
        pvalue = bledescriptor->value();
        return bt_gatt_attr_read(conn, attr, buf, len, offset, pvalue, bledescriptor->valueLength());
    }
    return 0;
}

// GATT server only
ssize_t profile_write_process(bt_conn_t *conn,
                              const bt_gatt_attr_t *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset)
{
    pr_info(LOG_MODULE_BLE, "%s1", __FUNCTION__);
    BLEAttribute *bleattr = (BLEAttribute *)attr->user_data;
    BLECharacteristicImp* blecharacteritic;
    BLEAttributeType type = bleattr->type();
    if ((BLETypeCharacteristic != type) || 0 != offset)
    {
        return 0;
    }
    
    blecharacteritic = (BLECharacteristicImp*)bleattr;
    blecharacteritic->setValue((const uint8_t *) buf, len);
    return len;
}

ssize_t profile_longwrite_process(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset)
{
    BLECharacteristicImp *blecharacteritic = (BLECharacteristicImp*)attr->user_data;
    
    blecharacteritic->setBuffer((const uint8_t *) buf, len, offset);
    
    return len;
}

int profile_longflush_process(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr, 
                              uint8_t flags)
{
    BLECharacteristicImp *blecharacteritic = (BLECharacteristicImp*)attr->user_data;

    switch (flags)
    {
    case BT_GATT_FLUSH_DISCARD:
        /* Discard buffer reseting it back with data */
        blecharacteritic->discardBuffer();
        return 0;
    case BT_GATT_FLUSH_SYNC:
        /* Sync buffer to data */
        blecharacteritic->syncupBuffer2Value();
        return 0;
    }

    return -EINVAL;
}


// GATT client only
uint8_t profile_notify_process (bt_conn_t *conn,
                                bt_gatt_subscribe_params_t *params,
                                const void *data, uint16_t length)
{
    //BLEPeripheralHelper* peripheral = BLECentralRole::instance()->peripheral(conn);// Find peripheral by bt_conn
    //BLEAttribute* notifyatt = peripheral->attribute(params); // Find attribute by params
    BLECharacteristicImp* chrc = NULL;
    BLEDevice bleDevice(bt_conn_get_dst(conn));
    BLEProfileManager::instance()->characteristic(bleDevice, params->value_handle);
    
    //assert(notifyatt->type() == BLETypeCharacteristic);
    pr_debug(LOG_MODULE_APP, "%s1", __FUNCTION__);
    
    chrc->setValue((const unsigned char *)data, length);
    return BT_GATT_ITER_CONTINUE;
}

// GATT client only
uint8_t profile_discover_process(bt_conn_t *conn,
                                 const bt_gatt_attr_t *attr,
                                 bt_gatt_discover_params_t *params)
{
    pr_debug(LOG_MODULE_BLE, "%s-%d", __FUNCTION__, __LINE__);
    return BLEProfileManager::instance()->discoverResponseProc(conn, attr, params);
}

// GATT Client only
uint8_t profile_read_rsp_process(bt_conn_t *conn, 
                                 int err,
                                 bt_gatt_read_params_t *params,
                                 const void *data, 
                                 uint16_t length)
{
    if (NULL == data)
    {
        return BT_GATT_ITER_STOP;
    }
    BLECharacteristicImp *chrc = NULL;
    BLEDevice bleDevice(bt_conn_get_dst(conn));
    
    // Get characteristic by handle params->single.handle
    chrc = BLEProfileManager::instance()->characteristic(bleDevice, params->single.handle);
    
    chrc->setValue((const unsigned char *)data, length);
    return BT_GATT_ITER_STOP;
}



void bleConnectEventHandler(bt_conn_t *conn, 
                            uint8_t err, 
                            void *param)
{
    BLEDeviceManager* p = (BLEDeviceManager*)param;

    p->handleConnectEvent(conn, err);
}


void bleDisconnectEventHandler(bt_conn_t *conn, 
                                uint8_t reason, 
                                void *param)
{
    BLEDeviceManager* p = (BLEDeviceManager*)param;
    
    pr_info(LOG_MODULE_BLE, "Connect lost. Reason: %d", reason);

    p->handleDisconnectEvent(conn, reason);
}

void bleParamUpdatedEventHandler(bt_conn_t *conn, 
                                 uint16_t interval,
                                 uint16_t latency, 
                                 uint16_t timeout, 
                                 void *param)
{
    BLEDeviceManager* p = (BLEDeviceManager*)param;

    p->handleParamUpdated(conn, interval, latency, timeout);
}


void ble_central_device_found(const bt_addr_le_t *addr, 
                              int8_t rssi, 
                              uint8_t type,
                              const uint8_t *ad, 
                              uint8_t len)
{
    char dev[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(addr, dev, sizeof(dev));
    //pr_debug(LOG_MODULE_BLE, "[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
    //       dev, type, len, rssi);

    BLEDeviceManager::instance()->handleDeviceFound(addr, rssi, type,
                                                  ad, len);
}


