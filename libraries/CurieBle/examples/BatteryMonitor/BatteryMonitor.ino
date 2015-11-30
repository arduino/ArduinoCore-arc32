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

/* UUID for Battery service */
#define SERVICE_UUID_BATTERY    "180F"
/* UUID for Battery Level characteristic */
#define CHAR_UUID_BATTERY_LEVEL "2A19"

/* Serial port to use for printing informational messages to the user */
#define LOG_SERIAL Serial

/* For convenience, this macro will invoke a specified function call and will
 * check the status value returned to ensure it is successful.  If not, it will
 * print an error message to the serial port and will return from the current function
 */
#define CHECK_STATUS(op)                               \
  do {                                                 \
    BleStatus status = op;                             \
    if (BLE_STATUS_SUCCESS != status) {                \
      LOG_SERIAL.print(#op" returned error status: "); \
      LOG_SERIAL.println(status);                      \
      return;                                          \
    }                                                  \
  } while(0)

/* BLE Battery Service */
BleService battSvc(SERVICE_UUID_BATTERY);

/* BLE Battery Level Characteristic */
BleUnsignedCharCharacteristic battLvlChar(CHAR_UUID_BATTERY_LEVEL,     /* standard 16-bit characteristic UUID */
                              BleRead | BleNotify /* remote clients will be able to get notifications if this characteristic changes */
                              );  

/* Bluetooth MAC address for this device */
BleDeviceAddress localAddress;

/* Variable to keep track of last battery level reading from analog input */
unsigned char oldBattLvl = 0;

/* This function will be called when a BLE GAP event is detected by the
 * Intel Curie BLE device */
void blePeripheralConnectedEventCb(BleCentral &bleCentral)
{
  /* We've got a new connection.  Lets print the MAC address of the remote device */
  LOG_SERIAL.println("Got CONNECTED event");
  LOG_SERIAL.println(bleCentral.address());
}

void blePeripheralDisconnectedEventCb(BleCentral &bleCentral)
{
  LOG_SERIAL.println("Got DISCONNECTED event");
}

void blePeripheralAdvTimeoutEventCb(BleCentral &bleCentral)
{
  LOG_SERIAL.println("Got ADV_TIMEOUT event");
}

void blePeripheralConnTimeoutEventCb(BleCentral &bleCentral)
{
  LOG_SERIAL.println("Got CONN_TIMEOUT event");
}

void setup() {
  pinMode(13, OUTPUT);
  LOG_SERIAL.begin(115200);

  /* Set a name for the BLE device
   * We give it an arbitrary name which will appear in advertising packets
   * and can be used by remote peers to identify this BLE device
   * The name can be changed but must not exceed 20 characters in length */
  CHECK_STATUS(blePeripheral.setLocalName("AE_BATTMON"));

  /* Set a function to be called whenever a BLE GAP event occurs */
  blePeripheral.setEventCallback(BLE_PERIPH_EVENT_CONNECTED, blePeripheralConnectedEventCb);
  blePeripheral.setEventCallback(BLE_PERIPH_EVENT_DISCONNECTED, blePeripheralDisconnectedEventCb);
  blePeripheral.setEventCallback(BLE_PERIPH_EVENT_ADV_TIMEOUT, blePeripheralAdvTimeoutEventCb);
  blePeripheral.setEventCallback(BLE_PERIPH_EVENT_CONN_TIMEOUT, blePeripheralConnTimeoutEventCb);

  CHECK_STATUS(blePeripheral.setAdvertisedServiceUuid(battSvc.uuid()));

  /* Add the BLE Battery service, and include the UUID in BLE advertising data */
  CHECK_STATUS(blePeripheral.addAttribute(battSvc));

  /* This service will have just one characteristic that reflects the current
   * percentage-charge level of the "battery" */
  CHECK_STATUS(blePeripheral.addAttribute(battLvlChar));

  /* Set an initial value for this characteristic; refreshed later the loop() function */
  CHECK_STATUS(battLvlChar.setValue(oldBattLvl));

  /* Now activate the BLE device.  It will start continuously transmitting BLE
   * advertising packets and thus become visible to remote BLE central devices
   * (e.g smartphones) until it receives a new connection */
  CHECK_STATUS(blePeripheral.begin());
  LOG_SERIAL.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  static int ledState;

  blePeripheral.poll();

  /* Blink the on-board LED (just to show some activity) */
  digitalWrite(13, ledState ? HIGH : LOW);
  ledState = !ledState;

  /* Read the current voltage level on the A0 analog input pin.
   * This is used here to simulate the charge level of a "battery".
   * The following tutorial shows how a potentiometer could be used
   * to vary the voltage on an analog input pin:
   * https://www.arduino.cc/en/Tutorial/Potentiometer
   */
  unsigned char battLvl = map(analogRead(A0), 0, 1023, 0, 100);

  if (battLvl != oldBattLvl) {
    LOG_SERIAL.print("Battery Level % is now: ");
    LOG_SERIAL.println(battLvl);

    /* If the voltage level has changed, we update the value of the
     * Battery Level BLE characteristic.  Because we have enabled
     * notifications for this characteristic, the remote device can
     * receive automatic updates when this value is changed. */
    CHECK_STATUS(battLvlChar.setValue(battLvl));
    oldBattLvl = battLvl;
  }

  /* Repeat the loop every 200ms - can be changed if desired */
  delay(200);
}
