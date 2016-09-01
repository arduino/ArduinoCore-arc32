// Wire Slave Receiver Sketch
// Receives data as an I2C slave device


#include <Wire2.h>

void setup() {
  Serial.begin(115200);           // start serial for output
  while(Serial);
  Wire2.begin(0x8);                // join i2c bus with address #8
  Wire2.onReceive(receiveEvent); // register event
}

void loop() {

    delay(2000);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int bytes) {
    int i;
    for (i = 0; i < bytes; i++)
    {
        int x = Wire2.read();    // receive byte as an integer
        Serial.println(x, HEX);         // print the integer
    }
}
