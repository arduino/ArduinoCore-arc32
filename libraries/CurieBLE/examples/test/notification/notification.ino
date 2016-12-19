
#include "CurieBLE.h"

// LED pin
#define LED_PIN   13

// create service
BLEService          ledService("19b10100e8f2537e4f6cd104768a1214");

BLECharacteristic   switchCharacteristic("19b10101e8f2537e4f6cd104768a1214", BLERead | BLEWrite | BLENotify, 1);

BLEDescriptor       switchDescriptor("2901", "switch");

void setup() {
    Serial.begin(9600);
    Serial.println("test---");
    
    // set LED pin to output mode
    pinMode(LED_PIN, OUTPUT);

    // begin initialization
    BLE.begin();
    Serial.println(BLE.address());
    
    // set advertised local name and service UUID
    BLE.setLocalName("LED");
    BLE.setAdvertisedServiceUuid(ledService.uuid());
    
    // add service and characteristic
    BLE.addService(ledService);
    ledService.addCharacteristic(switchCharacteristic);
    switchCharacteristic.addDescriptor(switchDescriptor);
    unsigned char test = 1;
    switchCharacteristic.writeValue(&test,1);
    BLE.advertise();
}

void loop() {
    static int i = 0;
  BLEDevice central = BLE.central();
    bool temp = central;
i++;
  if (temp) {
    // central connected to peripheral
    Serial.print(i);
    Serial.print(F("Connected to central: "));
    Serial.println(central.address());

    Serial.print(temp);
    
    unsigned char ledstate = 0;

    while (central.connected()) {
      // central still connected to peripheral
      if (switchCharacteristic.canNotify()) 
      {
        
        if (ledstate == 1)
        {
            ledstate = 0;
            digitalWrite(LED_PIN, LOW);
        }
        else
        {
            ledstate = 1;
            digitalWrite(LED_PIN, HIGH);
        }
        switchCharacteristic.writeValue(&ledstate, sizeof(ledstate));
        delay(5000);
      }
    }

    // central disconnected
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
  //delay (1000);
}
