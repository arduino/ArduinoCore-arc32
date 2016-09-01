// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.

/**
 * the sent data(8 bits): the higher 4 bits are equal to count and count will increase by 1 in every loop
 * when master is as writer, data are got from buffer_sender
 * when master is as reader, the received data are stored in buffer_receiver
 * data checking is to verify whether the buffer_sender is equal to buffer_receiver 
 **/
#include <Wire.h>
#define BUFFER_SIZE 8

static int count = 0;  // recode the higher 4 bits of data
static uint8_t buffer_sender[BUFFER_SIZE]; // data source for master writer
static uint8_t buffer_receiver[BUFFER_SIZE]; // data distination for master reader

void setup() 
{
  Wire.begin();         // join i2c bus (address optional for master)
  Serial.begin(115200); // start serial for output
  while (Serial)
    ;
}

void loop() 
{
  count++;
  Wire.beginTransmission(8); // transmit to device #8
  for (int i = 0; i < BUFFER_SIZE; i++) 
  {
    buffer_sender[i] = ((count & 0xf) << 4) | i;
    Wire.write(buffer_sender[i]); 
  }
  Wire.endTransmission();    // stop transmitting

  Wire.requestFrom(8, BUFFER_SIZE,true);    // request BUFFER_SIZE bytes from slave device #8    
  int k = 0;
  while (Wire.available())   // slave may send less than requested
  {
    buffer_receiver[k] = Wire.read();
    k++;
  }

  // check data: the received data should be equal to the sent data 
  for(int i = 0; i < BUFFER_SIZE; i++)
  {
    if(buffer_sender[i] == buffer_receiver[i])
      Serial.println("OK");
    else
    {
      Serial.print(buffer_sender[i],HEX);
      Serial.print("  !=  ");
      Serial.println(buffer_receiver[i],HEX);     
    }    
  }
  
  Serial.println("+++++");
  delay(1000);  

}
