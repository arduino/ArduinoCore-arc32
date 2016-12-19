#include <CurieBLE.h>
// change the next line based on what your smartphone is configured to advertise
const String deviceName  = "Nexus 5X";
const int rssiTrigger = -50;
const int ledPin = 13;

void setup() {
  Serial.begin(9600);
  BLE.begin();
  pinMode(ledPin, OUTPUT);
  BLE.setEventHandler(BLEDiscovered, bleCentralDiscoverHandler);
  while (!Serial) ;
  Serial.println("Bluetooth device active, start scanning...");
  BLE.scan(true);
}

void loop() {
  BLE.poll();
}

void bleCentralDiscoverHandler(BLEDevice peripheral) {
  if (peripheral.hasLocalName()) {
    Serial.println(peripheral.localName());
    if (peripheral.localName().indexOf(deviceName) != -1) {
      Serial.println(" found");
      Serial.print("Rssi: ");
      Serial.println(peripheral.rssi());
      if (peripheral.rssi() > rssiTrigger) {
        Serial.println("LED ON");
        digitalWrite(ledPin, HIGH);
      } else {
        Serial.println("LED OFF");
        digitalWrite(ledPin, LOW);
      }
    }
  }
}
