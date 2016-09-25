/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

#include <CurieBLE.h>
#include <CurieIMU.h>

/*
    This sketch example works with IMUBleCentral.ino.

    This sketch will read IMU data from sensor and send notifications to IMUBleCentral.ino.
    IMUBleCentral.ino will receive the notifications and output the received data.
*/

#define MAX_IMU_RECORD 1
#define IMU_BUF_INDEX 0

typedef struct {
    int index;
    unsigned int slot[3];
} imuFrameType;

// Buffer to hold IMU data
imuFrameType imuBuf[MAX_IMU_RECORD];

unsigned int seqNum = 0;

BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)

// create a new service with a custom 128-bit UUID
BLEService bleImuService("F7580001-153E-D4F6-F26D-43D8D98EEB13");

// standard 128-bit characteristic UUID with Read and Notify properties that allow
// remote clients to get notifications when this characteristic changes
BLECharacteristic bleImuChar("F7580003-153E-D4F6-F26D-43D8D98EEB13", 
                             BLERead | BLENotify, sizeof(imuBuf));

void setup()
{
    // initialize serial communication
    Serial.begin(9600);
    // wait for the serial port to connect. Open the Serial Monitor to continue executing the sketch
    // If you don't care to see text messages sent to the Serial Monitor during board initialization, 
    // remove or comment out the next line
    while(!Serial) ;
    // initialize the LED on pin 13 to indicate when a central is connected
    pinMode(LED_BUILTIN, OUTPUT);


    /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet */
    blePeripheral.setLocalName("ImuBle");
    // add the 128-bit UUID of bleImuService
    blePeripheral.setAdvertisedServiceUuid(bleImuService.uuid());
    // add the bleImuService 
    blePeripheral.addAttribute(bleImuService);
    // add the bleImuChar characteristic
    blePeripheral.addAttribute(bleImuChar);

    /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
    blePeripheral.begin();
    Serial.println("ImuBle Peripheral");
    // Start the IMU
    CurieIMU.begin();
}

void loop()
{
    // listen for BLE peripherals to connect:
    // since we are a peripheral we need a central object to connect to
    BLECentralHelper central = blePeripheral.central();

    // if a central is connected to peripheral:
    if (central)
    {
        Serial.print("Connected to central: ");
        // print the central's MAC address:
        Serial.println(central.address());

        Serial.print("IMU buffer size: ");
        Serial.println(sizeof(imuBuf));

        // turn the LED on to indicate that the BLE peripheral (Arduino/Genuino 101)
        // is connected to the central device
        digitalWrite(LED_BUILTIN, HIGH);

        long currentMillis, sentTime;

        // Send IMU data as long as the central is still connected
        currentMillis = sentTime = millis();
        while (central.connected())
        {
            // Take IMU data every 100 msec
            if ((millis() - sentTime) >= 100)
            {
                recordImuData(IMU_BUF_INDEX);
                sentTime = millis();
                bleImuChar.setValue((unsigned char *)&(imuBuf[IMU_BUF_INDEX]), sizeof(imuBuf));
            }
        } // end of while loop

        // when the central disconnects, turn off the LED:
        digitalWrite(LED_BUILTIN, LOW);
        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
    }
}

// This function records the IMU data that we send to the central 
void recordImuData(int index)
{
    /* Read IMU data.
    */
    int ax, ay, az;
    int gx, gy, gz;

    imuBuf[index].index = seqNum++;
    CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
    
    // Encode the data into the buffer
    imuBuf[index].slot[0] = (unsigned int)((ax << 16) | (ay & 0x0FFFF));
    imuBuf[index].slot[1] = (unsigned int)((az << 16) | (gx & 0x0FFFF));
    imuBuf[index].slot[2] = (unsigned int)((gy << 16) | (gz & 0x0FFFF));

}


/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
