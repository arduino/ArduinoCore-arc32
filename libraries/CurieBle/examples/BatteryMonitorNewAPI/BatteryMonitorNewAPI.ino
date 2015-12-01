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
 */
#include <CurieBle.h>

/*
 * This sketch example partially implements the standard Bluetooth Low-Energy "Battery" service.
 * For more information: https://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx
 */

/* BLE Peripheral Device (this Intel Curie device) */
BlePeripheral blePeripheral;

/* BLE Battery Service */
BleService battSvc("180F");

/* BLE Battery Level Characteristic */
BleUnsignedCharCharacteristic battLvlChar("2A19",     /* standard 16-bit characteristic UUID */
                              BleRead | BleNotify /* remote clients will be able to get notifications if this characteristic changes */
                              );  

/* Variable to keep track of last battery level reading from analog input */
uint8_t oldBattLvl = 0;
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(115200);

  pinMode(13, OUTPUT);

  /* Set a name for the BLE device
   * We give it an arbitrary name which will appear in advertising packets
   * and can be used by remote peers to identify this BLE device
   * The name can be changed but must not exceed 20 characters in length */
  blePeripheral.setLocalName("AE_BATTMON");
  blePeripheral.setAdvertisedServiceUuid(battSvc.uuid());

  /* Add the BLE Battery service, and include the UUID in BLE advertising data */
  blePeripheral.addAttribute(battSvc);

  /* This service will have just one characteristic that reflects the current
   * percentage-charge level of the "battery" */
  blePeripheral.addAttribute(battLvlChar);

  /* Set an initial value for this characteristic; refreshed later the loop() function */
  battLvlChar.setValue(oldBattLvl);

  /* Now activate the BLE device.  It will start continuously transmitting BLE
   * advertising packets and thus become visible to remote BLE central devices
   * (e.g smartphones) until it receives a new connection */
  blePeripheral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  BleCentral central = blePeripheral.central();

  if (central) {
    // central connected to peripheral
    Serial.print(F("Connected to central: "));
    Serial.println(central.address());

    digitalWrite(13, HIGH);

    while (central.connected()) {
      // central still connected to peripheral

      unsigned long currentMillis = millis();

      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateBatteryLevel();
      }
    }

    digitalWrite(13, LOW);
    
    // central disconnected
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());   
  }
}

void updateBatteryLevel() {
  /* Read the current voltage level on the A0 analog input pin.
   * This is used here to simulate the charge level of a "battery".
   * The following tutorial shows how a potentiometer could be used
   * to vary the voltage on an analog input pin:
   * https://www.arduino.cc/en/Tutorial/Potentiometer
   */
  uint8_t battLvl = map(analogRead(A0), 0, 1023, 0, 100);
  
  if (battLvl != oldBattLvl) {
    Serial.print("Battery Level % is now: ");
    Serial.println(battLvl);
  
    /* If the voltage level has changed, we update the value of the
     * Battery Level BLE characteristic.  Because we have enabled
     * notifications for this characteristic, the remote device can
     * receive automatic updates when this value is changed. */
    battLvlChar.setValue(battLvl);
    oldBattLvl = battLvl;
  }
}

