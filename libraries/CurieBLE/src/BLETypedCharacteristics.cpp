/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
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

#include "BLETypedCharacteristics.h"

BLECharCharacteristic::BLECharCharacteristic(const char* uuid, unsigned char properties) :
    BLETypedCharacteristic<char>(uuid, properties) {
}

BLEUnsignedCharCharacteristic::BLEUnsignedCharCharacteristic(const char* uuid, unsigned char properties) :
    BLETypedCharacteristic<unsigned char>(uuid, properties) {
}

BLEShortCharacteristic::BLEShortCharacteristic(const char* uuid, unsigned char properties) :
    BLETypedCharacteristic<short>(uuid, properties) {
}

BLEUnsignedShortCharacteristic::BLEUnsignedShortCharacteristic(const char* uuid, unsigned char properties) :
    BLETypedCharacteristic<unsigned short>(uuid, properties) {
}

BLEIntCharacteristic::BLEIntCharacteristic(const char* uuid, unsigned char properties) :
    BLETypedCharacteristic<int>(uuid, properties) {
}

BLEUnsignedIntCharacteristic::BLEUnsignedIntCharacteristic(const char* uuid, unsigned char properties) :
    BLETypedCharacteristic<unsigned int>(uuid, properties) {
}

BLELongCharacteristic::BLELongCharacteristic(const char* uuid, unsigned char properties) :
    BLETypedCharacteristic<long>(uuid, properties) {
}

BLEUnsignedLongCharacteristic::BLEUnsignedLongCharacteristic(const char* uuid, unsigned char properties) :
    BLETypedCharacteristic<unsigned long>(uuid, properties) {
}

BLEFloatCharacteristic::BLEFloatCharacteristic(const char* uuid, unsigned char properties) :
    BLETypedCharacteristic<float>(uuid, properties) {
}

BLEDoubleCharacteristic::BLEDoubleCharacteristic(const char* uuid, unsigned char properties) :
    BLETypedCharacteristic<double>(uuid, properties) {
}
