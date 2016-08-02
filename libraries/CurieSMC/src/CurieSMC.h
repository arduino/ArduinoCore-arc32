
#define QUARK_BUFF 	    shared_data->quark_to_ARC.data
#define QUARK_BUFF_HEAD	shared_data->quark_to_ARC.head
#define QUARK_BUFF_TAIL	shared_data->quark_to_ARC.tail
#define QUARK_BUFF_FLAG	shared_data->quark_to_ARC.flag
#define ARC_BUFF	    shared_data->ARC_to_quark.data
#define ARC_BUFF_HEAD	shared_data->ARC_to_quark.head
#define ARC_BUFF_TAIL	shared_data->ARC_to_quark.tail
#define ARC_BUFF_FLAG	shared_data->ARC_to_quark.flag

#include "Arduino.h"
#include "platform.h"

class CurieSMC 
{
public:
  CurieSMC()
  {
  }
  
  void begin();
  void end();
  int write(uint8_t data);
  uint8_t read();
  int availableForWrite();
  int availableForRead();

private:
};

extern CurieSMC SMC;