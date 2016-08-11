// Wire Slave Send Sketch
// Send data as an I2C slave device
// This example code is in the public domain.

#include <Wire2.h>

void setup() {
    Serial.begin(115200);           // start serial for output
    while(Serial);
    Wire2.begin(0x8);                           // join i2c bus with address #8
    Wire2.onRequest(requestEvent);               // register event
}
void loop() {

    delay(2000);
}

static int count = 0;
static uint8_t buffer[20];
// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
    int i;
    count++;
    Serial.print("call requestEvent ");
    Serial.print(count);
    Serial.println(" times");
    for (i = 0; i < sizeof(buffer); i++) {
        buffer[i] = ((count & 0xf) << 4) | i;
    }
    Wire2.write(buffer, sizeof(buffer));
}
