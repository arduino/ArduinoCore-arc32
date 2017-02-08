
#include "CurieBLE.h"
// LED pin
#define LED_PIN   13

void bleCentralConnectionParameterUpdateHandler(BLEDevice peripheral) {
int interval = peripheral.getConnectionInterval();
  Serial1.println("Updated connection parameter");
  Serial1.println("-----------------------");

  // print address
  Serial1.print("Address: ");
  Serial1.println(peripheral.address());

  Serial1.print("Interval: ");
  Serial1.println(peripheral.getConnectionInterval());
  Serial1.print("Timeout: ");
  Serial1.println(peripheral.getConnectionTimeout());
  Serial1.print("Latency: ");
  Serial1.println(peripheral.getConnectionLatency());

}

void setup() {
    Serial1.begin(115200);
    Serial1.println("test---");
    
    // set LED pin to output mode
    pinMode(LED_PIN, OUTPUT);
    
    //Register callbacks
    BLE.setEventHandler(BLEConParamUpdate, 
                        bleCentralConnectionParameterUpdateHandler);

    // begin initialization
    BLE.begin();
    Serial1.println(BLE.address());
    
    BLE.scanForName("LED");
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
    
    if (peripheral.discoverAttributes() == false)
    {
        Serial1.println("Discover failed, Disconnecting...");
        peripheral.disconnect();
        return;
    }
    
    BLECharacteristic ledCharacteristic = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");
    
    if (!ledCharacteristic)
    {
        peripheral.disconnect();
        Serial1.println("Peripheral does not have LED characteristic!");
        delay(5000);
        return;
    } 
    
    unsigned char ledstate = 0;

    discovered = false;
    while (peripheral.connected())
    {
        static int connection_interval = 10;
        static bool send_update = true;

        if (ledstate == 1)
        {
            ledstate = 0;
        }
        else
        {
            ledstate = 1;
            //connection_interval++;
        }
        
        ledCharacteristic.write(&ledstate, sizeof(ledstate));
        delay(5000);
        //ledCharacteristic.read();
        if (ledstate == 1 && send_update)
        {
            // Central send update request. Uncomment the below line
            //peripheral.setConnectionInterval(connection_interval, connection_interval);
            //send_update = false;
        }
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
        BLE.stopScan();
        delay (1000);
        // central connected to peripheral
        controlLed(peripheral);
        delay (4000);
        BLE.scanForName("LED");
    }
}



