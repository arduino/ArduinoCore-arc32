
#include "CurieBLE.h"

// LED pin
#define LED_PIN   13

void setup() {
    Serial.begin(9600);
    Serial.println("test---");
    
    // set LED pin to output mode
    pinMode(LED_PIN, OUTPUT);

    // begin initialization
    BLE.begin();
    Serial.println(BLE.address());
    
    BLE.scanForName("LED");
}

void controlLed(BLEDevice &peripheral)
{
    static bool discovered = false;
    // connect to the peripheral
    Serial.print("Connecting ... ");
    Serial.println(peripheral.address());

    if (peripheral.connect())
    {
        Serial.print("Connected: ");
        Serial.println(peripheral.address());
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
    Serial.println(peripheral.address());
}

void loop() {
    //pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    BLEDevice peripheral = BLE.available();
    //pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    if (peripheral) 
    {
    Serial.println(peripheral.address());
        BLE.stopScan();
        delay (1000);
        // central connected to peripheral
        controlLed(peripheral);
        delay (4000);
        BLE.scanForName("LED");
    }
}


