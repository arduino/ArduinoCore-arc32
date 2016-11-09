
#include "ArduinoBLE.h"

// LED pin
#define LED_PIN   13

// create service
BLEService          ledService("19b10100e8f2537e4f6cd104768a1214");

BLECharacteristic   switchCharacteristic("19b10101e8f2537e4f6cd104768a1214", BLERead | BLEWrite | BLENotify, 1);

BLEDescriptor       switchDescriptor("2901", "switch");

char buf[BT_ADDR_STR_LEN];

void setup() {
    Serial.begin(9600);
    Serial.println("test---");

    // set LED pin to output mode
    pinMode(LED_PIN, OUTPUT);

    // begin initialization
    BLE.begin();
    BLE.address(buf);
    Serial.println(buf);

    // set advertised local name and service UUID
    BLE.setLocalName("LED");
    BLE.setAdvertisedServiceUuid(ledService.uuid());

    // add service and characteristic
    BLE.addService(ledService);
    ledService.addCharacteristic(switchCharacteristic);
    switchCharacteristic.addDescriptor(switchDescriptor);
    unsigned char test = 1;
    switchCharacteristic.writeValue(&test,1);
    BLE.startAdvertising();
}

void loop() {
    static int i = 0;
  BLEDevice central = BLE.central();
    bool temp = central;
i++;
  if (temp) {
    // central connected to peripheral
    central.address(buf);
    Serial.print(i);
    Serial.print(F("Connected to central: "));
    Serial.println(buf);

    Serial.print(temp);

    while (central.connected()) {
      // central still connected to peripheral
      if (switchCharacteristic.written()) {
        char ledValue = *switchCharacteristic.value();
        // central wrote new value to characteristic, update LED
        if (ledValue) {
          Serial.println(F("LED on"));
          digitalWrite(LED_PIN, HIGH);
        } else {
          Serial.println(F("LED off"));
          digitalWrite(LED_PIN, LOW);
        }
      }
    }

    // central disconnected
    Serial.print(F("Disconnected from central: "));
    Serial.println(buf);
  }
  //delay (1000);
}
