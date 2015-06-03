#include "Arduino.h"
#include "portable.h"
//At the moment copy paste
#ifdef __cplusplus
 extern "C" {
#endif


uint32_t shiftIn( uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder ) {

	uint32_t value = 0;
	uint32_t i;

	for (i = 0; i < 32 ; i++) {

		digitalWrite(ulClockPin, HIGH);
		if (ulBitOrder == LSBFIRST)
			value |= digitalRead(ulDataPin) << i;
		else
			value |= digitalRead(ulDataPin) << (32-i);

		digitalWrite(ulClockPin,LOW);
	}

	return value;

}


void shiftOut( uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder, uint32_t ulVal ) {

	int i;
	for(i = 0; i < 32; i++) {
	
		if(ulBitOrder == LSBFIRST)
			digitalWrite(ulDataPin, !!(ulVal & (1 << i)));
		else
			digitalWrite(ulDataPin, !!(ulVal & (1 << (32-i))));

		digitalWrite(ulClockPin, HIGH);
		digitalWrite(ulClockPin, LOW);
	}
}


#ifdef __cplusplus
}
#endif


