
#include "CurieSMC.h"

CurieSMC SMC;
void CurieSMC::begin()
{
  QUARK_BUFF_HEAD = 0;
  QUARK_BUFF_TAIL = 0;
  QUARK_BUFF_FLAG = 2;
  ARC_BUFF_HEAD = 0;
  ARC_BUFF_TAIL = 0;
  ARC_BUFF_FLAG = 2; 
}
int CurieSMC::write(uint8_t data)
{
  //check if buffer is available
  if(ARC_BUFF_FLAG == 1)
  {
    return 1;
  }
  
  //lock the buffer
  ARC_BUFF_FLAG = 0;
  
  int new_head = (int)(ARC_BUFF_HEAD+1)%SHARED_BUFFER_SIZE;
  if(new_head != ARC_BUFF_TAIL)
  {
    ARC_BUFF[ARC_BUFF_HEAD] = data;
    ARC_BUFF_HEAD = new_head;
  }
  else
  {
    return 2; //buffer is full
  }

  //unlock the buffer
  ARC_BUFF_FLAG = 2;

  return 0;
}

uint8_t CurieSMC::read()
{
  //check if buffer is available
  if(QUARK_BUFF_FLAG == 1)
    return 0;

  //lock the buffer
  QUARK_BUFF_FLAG = 0;
  
  if(availableForRead())
  {
    uint8_t data = QUARK_BUFF[QUARK_BUFF_TAIL];
    QUARK_BUFF_TAIL = (QUARK_BUFF_TAIL + 1) % SHARED_BUFFER_SIZE;
    QUARK_BUFF_FLAG = 2; //unlock the buffer
    return data;
  }
  else
  {
    QUARK_BUFF_FLAG = 2; //unlock the buffer
    return 0;
  }
}

int CurieSMC::availableForWrite()
{
  int head = ARC_BUFF_HEAD;
  int tail = ARC_BUFF_TAIL;
  
  if(head >= tail)
    return SHARED_BUFFER_SIZE - head + tail - 1;
  return tail - head - 1;
}

int CurieSMC::availableForRead()
{
  return (int)(SHARED_BUFFER_SIZE + QUARK_BUFF_HEAD-QUARK_BUFF_TAIL) % SHARED_BUFFER_SIZE;
}
