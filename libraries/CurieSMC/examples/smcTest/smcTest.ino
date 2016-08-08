/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
 * This sketch example demonstrates a simple way to use the Shared Memory Communication library between 
 * the ARC and Quark cores. 
 * It requires that the firmware be upgraded to one that supports the Shared Memory Communication library.
 * This sketch example will need to be used with the smc example built using the CODK-M tree.
 * 
 * This sketch sends values to the Quark core which then reads those values, doubles them and sends it back to the ARC core.
 * This sketch then reads those values and displays the doubled values.
 */

#include <CurieSMC.h>

void setup() 
{
  Serial.begin(384000);
  while(!Serial);
  SMC.begin();
}

void loop() 
{
  for(int i = 0; i < 32; i++)
  {
    Serial.print("writing : ");
    Serial.println(i);
    SMC.write(i);
  }
  while(SMC.availableForRead())
  {
    Serial.print("data: ");
    Serial.println(SMC.read());
  }
  
  delay(1000);
}

/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

