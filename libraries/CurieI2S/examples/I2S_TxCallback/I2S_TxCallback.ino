#include <CurieI2S.h>

//I2S_TX -> Pin 7
//I2S_TSK -> Pin 4
//I2S_TSCK -> pin 2

void setup() 
{
  Serial.begin(115200); // initialize Serial communication
  while(!Serial) ;      // wait for serial port to connect.
  Serial.println("CurieI2S Tx Callback");
  CurieI2S.begin(I2S_44K, I2S_32bit);
  CurieI2S.setI2SMode(PHILIPS_MODE);
  CurieI2S.attachTxEmptyInterrupt(doneTX);
  CurieI2S.attachTxInterrupt(fillTxBuffer);
  CurieI2S.initTX();
}

void loop() 
{
  Serial.println("+++");

  //start filling the tx buffer
  CurieI2S.pushData(0xFFFFFFFF);
  CurieI2S.pushData(0x00000000);
  CurieI2S.pushData(0xDEADFACE);
  CurieI2S.pushData(0x10101010);
  Serial.println("start transmitting");
  //Start Transmission
  CurieI2S.startTX(); 
  for(int i = 0; i < 40; i++)
  {
    //keep filling the buffer
    while(!CurieI2S.pushData(0xFFFFFFFF));
    while(!CurieI2S.pushData(0x00000000));
    while(!CurieI2S.pushData(0xDEADFACE));
    while(!CurieI2S.pushData(0x10101010));
  }
  //Tx is automatically stopped after the tx buffer is emptied
  
  delay(500);
  Serial.println("+++");
}

void doneTX()
{
  Serial.println("done transmitting");
}

void fillTxBuffer()
{
  //you can fill the tx buffer here if you want
  //CurieI2S.pushData(0xDEADDEAD);
  //CurieI2S.pushData(0xDEADFACE);
}

