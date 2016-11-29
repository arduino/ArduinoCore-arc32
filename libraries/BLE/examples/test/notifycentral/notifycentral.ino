
#include "ArduinoBLE.h"

// LED pin
#define LED_PIN   13

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

    BLE.startScanning("LED");
}

void controlLed(BLEDevice &peripheral)
{
    static bool discovered = false;
    // connect to the peripheral
    peripheral.address(buf);
    Serial.print("Connecting ... ");
    Serial.println(buf);

    if (peripheral.connect())
    {
        Serial.print("Connected: ");
        Serial.println(buf);
    }
    else
    {
        Serial.println("Failed to connect!");
        return;
    }

    peripheral.discoverAttributes();

    BLECharacteristic ledCharacteristic = peripheral.characteristic("19b10101-e8f2-537e-4f6c-d104768a1214");

    if (!ledCharacteristic)
    {
        peripheral.disconnect();
        Serial.println("Peripheral does not have LED characteristic!");
        delay(5000);
        return;
    }
      ledCharacteristic.subscribe();


    discovered = false;
    while (peripheral.connected())
    {
        if (ledCharacteristic.valueUpdated())
        {
            char ledValue = *ledCharacteristic.value();

            if (ledValue) {
              Serial.println(F("LED on"));
              digitalWrite(LED_PIN, HIGH);
            } else {
              Serial.println(F("LED off"));
              digitalWrite(LED_PIN, LOW);
            }
        }
    }
    Serial.print("Disconnected");
    Serial.println(buf);
}

void loop() {
    BLEDevice peripheral = BLE.available();

    if (peripheral)
    {
        peripheral.address(buf);
        Serial.println(buf);
        BLE.stopScanning();
        delay (1000);
        // central connected to peripheral
        controlLed(peripheral);
        delay (4000);
        BLE.startScanning("LED");
    }
}


