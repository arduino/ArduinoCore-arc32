/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 */
#include "BLEAttribute.h"
#include "BLEUuid.h"

unsigned char BLEAttribute::_numAttributes = 0;

BLEAttribute::BLEAttribute(const char* uuid, enum BLEAttributeType type) :
    _uuid(uuid),
    _type(type),
    _handle(0)
{
    _numAttributes++;
}

const char*
BLEAttribute::uuid() const {
    return _uuid;
}

enum BLEAttributeType
BLEAttribute::type() const {
    return this->_type;
}

uint16_t
BLEAttribute::handle() {
    return _handle;
}

void
BLEAttribute::setHandle(uint16_t handle) {
    _handle = handle;
}


bt_uuid
BLEAttribute::btUuid() const {
    BLEUuid bleUuid = BLEUuid(uuid());
    
    return bleUuid.uuid();
}

unsigned char
BLEAttribute::numAttributes() {
    return _numAttributes;
}
