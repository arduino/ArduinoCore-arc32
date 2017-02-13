#include <CurieBLE.h>

uint8_t value = 0;

BLEService service = BLEService("EEE0");
BLEShortCharacteristic characteristic = BLEShortCharacteristic("EEE1", BLERead | BLENotify | BLEBroadcast);

void setup() {
  Serial.begin(9600);

  BLE.setLocalName("BLEBroadcast");
  BLE.setAdvertisedServiceUuid(service.uuid());

  BLE.addService(service);
  service.addCharacteristic(characteristic);

  characteristic.setValue(value);

  BLE.begin();
  characteristic.broadcast();

  Serial.println(F("BLE Broadcast Count"));
}

void loop() {
    BLE.poll();
    characteristic.setValue(value);    
    delay(1000);
    value++;
}
