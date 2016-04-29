/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 */
#ifndef _BLE_TYPED_CHARACTERISTICS_H_INCLUDED
#define _BLE_TYPED_CHARACTERISTICS_H_INCLUDED

#include "BLETypedCharacteristic.h"

class BLEBoolCharacteristic : public BLETypedCharacteristic<bool> {
public:
    BLEBoolCharacteristic(const char* uuid, unsigned char properties);
};

class BLECharCharacteristic : public BLETypedCharacteristic<char> {
public:
    BLECharCharacteristic(const char* uuid, unsigned char properties);
};

class BLEUnsignedCharCharacteristic : public BLETypedCharacteristic<unsigned char> {
public:
    BLEUnsignedCharCharacteristic(const char* uuid, unsigned char properties);
};

class BLEShortCharacteristic : public BLETypedCharacteristic<short> {
public:
    BLEShortCharacteristic(const char* uuid, unsigned char properties);
};

class BLEUnsignedShortCharacteristic : public BLETypedCharacteristic<unsigned short> {
public:
    BLEUnsignedShortCharacteristic(const char* uuid, unsigned char properties);
};

class BLEIntCharacteristic : public BLETypedCharacteristic<int> {
public:
    BLEIntCharacteristic(const char* uuid, unsigned char properties);
};

class BLEUnsignedIntCharacteristic : public BLETypedCharacteristic<unsigned int> {
public:
    BLEUnsignedIntCharacteristic(const char* uuid, unsigned char properties);
};

class BLELongCharacteristic : public BLETypedCharacteristic<long> {
public:
    BLELongCharacteristic(const char* uuid, unsigned char properties);
};

class BLEUnsignedLongCharacteristic : public BLETypedCharacteristic<unsigned long> {
public:
    BLEUnsignedLongCharacteristic(const char* uuid, unsigned char properties);
};

class BLEFloatCharacteristic : public BLETypedCharacteristic<float> {
public:
    BLEFloatCharacteristic(const char* uuid, unsigned char properties);
};

class BLEDoubleCharacteristic : public BLETypedCharacteristic<double> {
public:
    BLEDoubleCharacteristic(const char* uuid, unsigned char properties);
};

#endif // _BLE_TYPED_CHARACTERISTICS_H_INCLUDED
