#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(CANBUS_SUPPORT)
#ifndef _BASICCAN_H_
#define _BASICCAN_H_

class BasicCan
{
public:
  BasicCan(){}
  void Init(void);
  void SetFunctionValue(uint16_t FuncID, uint8_t *pBuff, uint8_t Len);
  void SetFunctionValue(uint16_t FuncID, uint16_t u16Value);
public:
  

private:
  
};

extern BasicCan BasicCanPort;

#endif //def _CANMODULE_H_
#endif //ENABLE CANBUS_SUPPORT