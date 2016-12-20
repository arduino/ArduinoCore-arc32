/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

//I2S_TX -> Pin 7
//I2S_TSK -> Pin 4
//I2S_TSCK -> pin 2

#include <CurieI2SDMA.h>

const int BUFF_SIZE=64;
boolean blinkState = true;          // state of the LED
uint32_t dataBuff[BUFF_SIZE];
uint32_t loop_count = 0;
void setup() 
{
  Serial.begin(115200); // initialize Serial communication
  while(!Serial) ;      // wait for serial port to connect.
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
  int status = CurieI2SDMA.transTX(dataBuff,sizeof(dataBuff),sizeof(uint32_t));
  if(status)
    return;
   
  blinkState = !blinkState;
  digitalWrite(13, blinkState);

  //when the TX set to be slave, the two lines below will introduce delay.
  //Please remove them.
  Serial.println("done transmitting");
  delay(1000);
  
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
