// Wire Slave Send Sketch
// Send data as an I2C slave device
// This example code is in the public domain.

#include <Wire2.h>

// BUFFER_SIZE: the buffer size of slave side should not be less than master side
// TX_RX_len: record the data size received from master 
// buffer[BUFFER_SIZE]: it's the data distination for slave reader and also the data source for slave writer
#define BUFFER_SIZE 100 
static int TX_RX_len;
static uint8_t buffer[BUFFER_SIZE];

void setup(){
  Serial.begin(115200); // start serial for output
  while (Serial)
    ;
  Wire2.begin(0x8);              // join i2c bus with address #8
  Wire2.onRequest(requestEvent); // register event
  Wire2.onReceive(receiveEvent); // register event
}


void loop() {

}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int bytes) {
    TX_RX_len = bytes;
    for (int i = 0; i < bytes; i++)
    {
        int x = Wire2.read();    // receive byte as an integer
        Serial.println(x, HEX);  // print the integer
        buffer[i]=x;
    }
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
    //Serial.println("call requestEvent ");
    Wire2.write(buffer, TX_RX_len);
}
