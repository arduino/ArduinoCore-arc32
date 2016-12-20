/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/**
 * A simple sketch to test the rx channel of the i2s interface. 
 * A callback function is used to fill up a buffer whenever data is received
 * 
 * To test this sketch you will need a second Arduino/Genuino 101 board with the I2SDMA_TxCallback sketch uploaded
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
#include <CurieI2SDMA.h>

const int BUFF_SIZE=64;
const int OFFSET=2;
uint32_t dataBuff[BUFF_SIZE+OFFSET]; // extra 2 buffers are for the padding zero

/** 
 * the received data: the higher 16 bits should be equal to loop_count and the lower 16 bits should be (0x01~0x80)
 * loop_count should increase by 1 in evry loop()
 * for example: if first time the received data is 0x00010001~0x00010080, then next time the data should be 0x00020001~0x00020080
**/
uint8_t start_flag = 0;
uint8_t done_flag = 0; // when done_flag is 1, the received data are correct
uint32_t loop_count = 0; // record the higher 16 bits of received data
uint32_t shift_count = 0; // the position of first non-zero
void setup() 
{
  Serial.begin(115200); // initialize Serial communication
  while(!Serial) ;      // wait for serial port to connect.
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
  int status = CurieI2SDMA.transRX(dataBuff,sizeof(dataBuff),sizeof(uint32_t));
  if(status)
    return;

  // find out first non-zero
  shift_count = 0;
  for(uint32_t i = 0;i <= OFFSET;++i)
  {
    if(dataBuff[i] == 0)
      shift_count++;
    else
      break;
  }
  if(shift_count > OFFSET)
    return;
  
  // record the higher 16 bits of received data  
  if(start_flag)
  {   
    if((dataBuff[shift_count]>>16) != loop_count+1)
      Serial.println("+++ loop_count jump +++");        
  }
  else
  {
    start_flag = 1;
  }
  loop_count = (dataBuff[shift_count] >> 16);
  
  // check data serial: the higher 16 bits should be equal to loop_count and the lower 16 bits should be (0x01~0x80)  
  done_flag = 1;
  for(uint32_t i = 0 ;i < BUFF_SIZE;++i)
  {
    //Serial.println(dataBuff[i+shift_count],HEX);
    if ((dataBuff[i+shift_count] & 0XFFFF0000) == (loop_count <<16)
      && (dataBuff[i+shift_count] & 0XFFFF) == (i+1))
            ;
    else
    {
      done_flag = 0;
      Serial.println(dataBuff[i+shift_count],HEX);
      Serial.println("ERROR");
      break;
    }
  } 
    
  if(done_flag)
    Serial.println("RX done");
  delay(100);  
}

/*
  Copyright (c) 2016 Intel Corporation. All rights reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-
  1301 USA
*/
