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

#include "BleTypedCharacteristics.h"

BleCharCharacteristic::BleCharCharacteristic(const char* uuid, unsigned char properties) :
    BleTypedCharacteristic<char>(uuid, properties) {
}

BleUnsignedCharCharacteristic::BleUnsignedCharCharacteristic(const char* uuid, unsigned char properties) :
    BleTypedCharacteristic<unsigned char>(uuid, properties) {
}

BleShortCharacteristic::BleShortCharacteristic(const char* uuid, unsigned char properties) :
    BleTypedCharacteristic<short>(uuid, properties) {
}

BleUnsignedShortCharacteristic::BleUnsignedShortCharacteristic(const char* uuid, unsigned char properties) :
    BleTypedCharacteristic<unsigned short>(uuid, properties) {
}

BleIntCharacteristic::BleIntCharacteristic(const char* uuid, unsigned char properties) :
    BleTypedCharacteristic<int>(uuid, properties) {
}

BleUnsignedIntCharacteristic::BleUnsignedIntCharacteristic(const char* uuid, unsigned char properties) :
    BleTypedCharacteristic<unsigned int>(uuid, properties) {
}

BleLongCharacteristic::BleLongCharacteristic(const char* uuid, unsigned char properties) :
    BleTypedCharacteristic<long>(uuid, properties) {
}

BleUnsignedLongCharacteristic::BleUnsignedLongCharacteristic(const char* uuid, unsigned char properties) :
    BleTypedCharacteristic<unsigned long>(uuid, properties) {
}

BleFloatCharacteristic::BleFloatCharacteristic(const char* uuid, unsigned char properties) :
    BleTypedCharacteristic<float>(uuid, properties) {
}

BleDoubleCharacteristic::BleDoubleCharacteristic(const char* uuid, unsigned char properties) :
    BleTypedCharacteristic<double>(uuid, properties) {
}
