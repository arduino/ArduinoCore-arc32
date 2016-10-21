
#include "ArduinoBLE.h"
#include "BLEAttribute.h"
#include "BLECharacteristicImp.h"
#include "BLEProfileManager.h"

// LED pin
#define LED_PIN   13

void setup() {
    Serial1.begin(115200);
    Serial1.println("test---");
    
    // set LED pin to output mode
    pinMode(LED_PIN, OUTPUT);

    // begin initialization
    BLE.begin();
    Serial1.println(BLE.address());
    
    BLE.startScanning("LED");
}

void controlLed(BLEDevice &peripheral)
{
    static bool discovered = false;
    // connect to the peripheral
    Serial1.print("Connecting ... ");
    Serial1.println(peripheral.address());

    if (peripheral.connect())
    {
        Serial1.print("Connected: ");
        Serial1.println(peripheral.address());
    }
    else
    {
        Serial1.println("Failed to connect!");
        return;
    }
    
    peripheral.discoverAttributes();
    
    BLECharacteristic ledCharacteristic = peripheral.characteristic("19b10101-e8f2-537e-4f6c-d104768a1214");
    
    if (!ledCharacteristic)
    {
        //peripheral.disconnect();
        while(1)
        {
        Serial1.println("Peripheral does not have LED characteristic!");
        delay(5000);
        }
        return;
    } 
      
      
      unsigned char ledstate = 0;

    discovered = false;
    while (peripheral.connected())
    {
        if (ledstate == 1)
        {
            ledstate = 0;
        }
        else
        {
            ledstate = 1;
        }
        ledCharacteristic.write(&ledstate, sizeof(ledstate));
        delay(5000);
    }
    Serial1.print("Disconnected");
    Serial1.println(peripheral.address());
}

void loop() {
    //pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    BLEDevice peripheral = BLE.available();
    //pr_debug(LOG_MODULE_BLE, "%s-%d",__FUNCTION__, __LINE__);
    if (peripheral) 
    {
    Serial1.println(peripheral.address());
        BLE.stopScanning();
        delay (1000);
        // central connected to peripheral
        controlLed(peripheral);
        delay (4000);
        BLE.startScanning("LED");
    }
}


