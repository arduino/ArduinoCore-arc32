/*
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
#include <CurieBLE.h>

/*
  This sketch example works with IMUBleNotification.ino

  IMUBleNotification.ino will send notification to this central sketch.
  This sketch will receive the notifications and output the received data in the serial monitor.
  It also illustrates using a non-typed characteristic.
  Set the baud rate to 115200 on the serial monitor to accomodate the speed of constant data updates from IMU subsystem.
*/

#define LED_PIN   13
#define MAX_IMU_RECORD 1

// define a structure that will serve as buffer for holding IMU data

typedef struct {
    int index;
    unsigned int slot[3];
} imuFrameType;

imuFrameType imuBuf[MAX_IMU_RECORD];

void setup()
{
    // This is set to higher baud rate because accelerometer data changes very quickly
    Serial.begin(9600);    // initialize serial communication
    while (!Serial);
    pinMode(LED_PIN, OUTPUT);    // initialize the LED on pin 13 to indicate when a central is connected
    
    /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
    BLE.begin();
    Serial.println(BLE.address());
    
    BLE.scanForName("Imu");
}


void loop()
{
    BLEDevice peripheral = BLE.available();
    //pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    if (peripheral) 
    {
        Serial.println(peripheral.address());
        BLE.stopScan();
        delay (1000);
        // central connected to peripheral
        controlImu(peripheral);
        delay (4000);
        BLE.scanForName("Imu");
    }
}

void controlImu(BLEDevice peripheral)
{
    static bool discovered = false;
    // connect to the peripheral
    Serial.print("Connecting ... ");
    Serial.println(peripheral.address());

    if (peripheral.connect())
    {
        Serial.print("Connected: ");
        Serial.println(peripheral.address());
    }
    else
    {
        Serial.println("Failed to connect!");
        return;
    }
    
    peripheral.discoverAttributes();
    
    BLECharacteristic bleImuChar = peripheral.characteristic("F7580003-153E-D4F6-F26D-43D8D98EEB13");
    
    if (!bleImuChar)
    {
        peripheral.disconnect();
        Serial.println("Peripheral does not have IMU characteristic!");
        delay(5000);
        return;
    } 
      bleImuChar.subscribe();
      

    discovered = false;
    while (peripheral.connected())
    {
        if (bleImuChar.valueUpdated())
        {
            const unsigned char *cvalue = bleImuChar.value();
            const imuFrameType *value = (const imuFrameType *)cvalue;
            Serial.print("\r\nCharacteristic event, written: ");
            Serial.print(value->index);
            Serial.print("\t");
            Serial.print(value->slot[0]);
            Serial.print("\t");
            Serial.print(value->slot[1]);
            Serial.print("\t");
            Serial.println(value->slot[2]);
        }
    }
    Serial.print("Disconnected");
    Serial.println(peripheral.address());
}
