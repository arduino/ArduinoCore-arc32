/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 */
#ifndef _BLE_ATTRIBUTE_H_INCLUDED
#define _BLE_ATTRIBUTE_H_INCLUDED

#include "BLECommon.h"

enum BLEAttributeType {
    BLETypeService        = 0x2800,
    BLETypeCharacteristic = 0x2803,
    BLETypeDescriptor     = 0x2900
};

class BLEPeripheral;

class BLEAttribute {
public:
    
    /**
     * Get the string representation of the Attribute
     *
     * @return const char* string representation of the Attribute
     */
    const char* uuid(void) const;

protected:
    friend BLEPeripheral;

    BLEAttribute(const char* uuid, enum BLEAttributeType type);

    BLEAttributeType type(void) const;
    bt_uuid btUuid(void) const;
    uint16_t handle(void);
    void setHandle(uint16_t handle);

    static unsigned char numAttributes(void);

private:
    static unsigned char _numAttributes;

    const char* _uuid;
    enum BLEAttributeType _type;
    uint16_t _handle;
};

#endif // _BLE_ATTRIBUTE_H_INCLUDED
