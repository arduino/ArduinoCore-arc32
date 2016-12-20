/**
 * A simple sketch to test the rx channel of the i2s interface. 
 * A callback function is used to fill up a buffer whenever data is received
 * 
 * To test this sketch you will need a second Arduino/Genuino 101 board with the I2S_TxCallback sketch uploaded
 * 
 * Connection:
 *   I2S_RSCK(pin 8) -> I2S_TSCK(pin 2) 
 *   I2S_RWS (pin 3) -> I2S_TWS (pin 4)
 *   I2S_RXD (pin 5) -> I2S_TXD (pin 7)
 *   Ground  (GND)   -> Ground  (GND)
 * Notes:
 *   Transmission is sensitive to noise. To reduce noise:
 *   - Power both boards with an external power supply. Usb power is not always clean.
 *   - Insure that both boards are sharing the same ground.
 *   - Use short wires to connect between the board or use shielded wire.
**/
#include <CurieI2S.h>

const int BUFF_SIZE=128;

uint32_t dataBuff[BUFF_SIZE];
volatile int count = 0;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200); // initialize Serial communication
  while(!Serial) ;      // wait for serial port to connect.
  Serial.println("CurieI2S Rx Callback Example");
  CurieI2S.begin(I2S_44K, I2S_32bit);
  CurieI2S.setI2SMode(PHILIPS_MODE);
  CurieI2S.attachRxInterrupt(rxDataReceived);
  CurieI2S.initRX();
  CurieI2S.startRX();

}

void loop() 
{
  if(count>0)
  {
    for(int i =0; i < count; i++)
    {
      Serial.print("data: ");
      Serial.println(dataBuff[i], HEX);
    }
    count = 0;
  }
  delay(500);
}

//This function is called inside an ISR so it is important to make this as atomic/fast as possible
void rxDataReceived()
{
  while(CurieI2S.available())
  {
    dataBuff[count++] =  CurieI2S.requestdword();   
    count %= BUFF_SIZE; //prevent buffer overflow and just write data in front of the buffer.
  }
}
