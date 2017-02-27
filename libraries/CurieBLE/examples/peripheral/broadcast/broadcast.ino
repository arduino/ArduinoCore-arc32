#include <CurieBLE.h>

uint8_t value = 0;

BLEPeripheral peripheral;
BLEService service = BLEService("EEE0");
BLEShortCharacteristic characteristic = BLEShortCharacteristic("EEE1", BLERead | BLENotify | BLEBroadcast);

void setup() {
  Serial.begin(9600);

  peripheral.setLocalName("BLEBroadcast");
  peripheral.setAdvertisedServiceUuid(service.uuid());

  peripheral.addAttribute(service);
  peripheral.addAttribute(characteristic);

  characteristic.setValue(value);

  peripheral.begin();
  characteristic.broadcast();

  Serial.println(F("BLE Broadcast Count"));
}

void loop() {
    peripheral.poll();
    characteristic.setValue(value);    
    delay(1000);
    value++;
}
