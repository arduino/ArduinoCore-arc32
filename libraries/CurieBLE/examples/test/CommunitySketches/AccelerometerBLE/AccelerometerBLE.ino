#include <CurieBLE.h>
#include "CurieIMU.h"


BLEPeripheral blePeripheral;  // BLE Peripheral Device (the board you're programming)
BLEService accelService("19B10010-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

// BLE accelerometer Characteristic - custom 128-bit UUID, read by central
BLEFloatCharacteristic accelX("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic accelY("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic accelZ("19B10013-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

long lastUpdate = 0;

void setup() {
  Serial.begin(9600);

  // set advertised local name and service UUID:
  blePeripheral.setLocalName("tigoeAcc");
  blePeripheral.setAdvertisedServiceUuid(accelService.uuid());

  // add service and characteristic:
  blePeripheral.addAttribute(accelService);
  blePeripheral.addAttribute(accelX);
  blePeripheral.addAttribute(accelY);
  blePeripheral.addAttribute(accelZ);

  CurieIMU.begin();

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // set the initial value for the characeristic:
  accelX.setValue(0);
  accelY.setValue(0);
  accelZ.setValue(0);

  // begin advertising BLE service:
  blePeripheral.begin();
  pinMode(13, OUTPUT);
  Serial.println("Starting");
}

void loop() {
  // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {
    digitalWrite(13, HIGH);
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      long now = millis();
      if (now - lastUpdate > 1000) {
        updateAccelerometer();
        lastUpdate = now;
      }
    }
    // when the central disconnects, print it out:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    digitalWrite(13, LOW);

  }
}

void updateAccelerometer() {
  int axRaw, ayRaw, azRaw;         // raw accelerometer values
  float ax, ay, az;

  // read raw accelerometer measurements from device
  CurieIMU.readAccelerometer(axRaw, ayRaw, azRaw);

  // convert the raw accelerometer data to G's
  ax = convertRawAcceleration(axRaw);
  ay = convertRawAcceleration(ayRaw);
  az = convertRawAcceleration(azRaw);

  accelX.setValue(ax);
  accelY.setValue(ay);
  accelZ.setValue(az);
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}
