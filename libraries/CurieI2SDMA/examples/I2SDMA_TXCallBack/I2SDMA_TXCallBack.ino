//I2S_TX -> Pin 7
//I2S_TSK -> Pin 4
//I2S_TSCK -> pin 2

#include <CurieI2SDMA.h>

#define BUFF_SIZE 128
boolean blinkState = true;          // state of the LED
uint32_t dataBuff[BUFF_SIZE];
uint32_t loop_count = 0;
void setup() 
{
  Serial.begin(115200);
  while(!Serial);
  Serial.println("CurieI2SDMA Tx Callback");

  CurieI2SDMA.iniTX();
  /*
   * CurieI2SDMA.beginTX(sample_rate, resolution, master,mode)
   * mode 1 : PHILIPS_MODE
   *      2 : RIGHT_JST_MODE
   *      3 : LEFT_JST_MODE
   *      4 : DSP_MODE
   */
  CurieI2SDMA.beginTX(44100, 32,1, 1);
  digitalWrite(13, blinkState);  
}

void loop() 
{
  for(uint32_t i = 0; i <BUFF_SIZE; ++i)
  {
    dataBuff[i] = i + 1 + (loop_count<<16);
  }
  loop_count++;
  int status = CurieI2SDMA.transTX(dataBuff,sizeof(dataBuff));
  if(status)
    return;
   
  blinkState = !blinkState;
  digitalWrite(13, blinkState);

  //when the TX set to be slave, the two lines below will introduce delay.
  //Please remove them.
  Serial.println("done transmitting");
  delay(1000);
  
}


