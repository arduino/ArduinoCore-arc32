/**
 * A simple sketch to test the rx channel of the i2s interface. 
 * A callback function is used to fill up a buffer whenever data is received
 * 
 * To test this sketch you will need a second Arduino/Genuino 101 board with the I2S_TxCallback sketch uploaded
 * 
 * Connection:
 * GND -> GND
 * I2S_RSCK(pin 8) -> I2S_TSCK(pin 2) 
 * I2S_RWS(pin 3) -> I2S_TWS(pin 4)
 * I2S_RXD(pin 5) -> I2S_TXD(pin 7)
 * 
**/
#include <CurieI2SDMA.h>

#define BUFF_SIZE 128
uint32_t dataBuff[BUFF_SIZE+2]; // extra 2 buffer is for the padding zero

uint32_t OFFSET = 0;
uint8_t start_flag = 0;
uint8_t done_flag = 0;
uint32_t loop_count = 0;

void setup() 
{
  Serial.begin(115200);
  while(!Serial);
  Serial.println("CurieI2SDMA Rx Callback");

  CurieI2SDMA.iniRX();
  /*
   * CurieI2SDMA.beginTX(sample_rate, resolution, master,mode)
   * mode 1 : PHILIPS_MODE
   *      2 : RIGHT_JST_MODE
   *      3 : LEFT_JST_MODE
   *      4 : DSP_MODE
   */
  CurieI2SDMA.beginRX(44100, 32,0,1);

}

void loop() 
{
  int status = CurieI2SDMA.transRX(dataBuff,sizeof(dataBuff));
  if(status)
    return;

  if(start_flag)
  {   
    if( (dataBuff[OFFSET]>>16) != loop_count+1)
      Serial.println("+++ loop_count jump +++");        
  }
  else
  {
    start_flag = 1;
    OFFSET =  (dataBuff[0] == 0) ? 2 : 0;
  }
  loop_count = (dataBuff[OFFSET] >> 16);

  done_flag = 1;
  for(uint32_t i = 0 ;i < BUFF_SIZE;++i)
  {
    //Serial.println(dataBuff[i+OFFSET],HEX);
    if ((dataBuff[i+OFFSET] & 0XFFFF0000) == (loop_count <<16)
      && (dataBuff[i+OFFSET] & 0XFFFF) == (i+1))
            ;
    else
    {
      done_flag = 0;
      Serial.println(dataBuff[i+OFFSET],HEX);
      Serial.println("ERROR");
      break;
    }
  } 
    
  if(done_flag)
    Serial.println("RX done");
  delay(100);  
}
