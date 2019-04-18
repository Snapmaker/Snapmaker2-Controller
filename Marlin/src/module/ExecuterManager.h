#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(EXECUTER_MANAGER_SUPPORT)
#ifndef _EXECUTER_MANAGER_H_
#define _EXECUTER_MANAGER_H_

#if ENABLED(EXECUTER_CANBUS)
#include "../../HAL/HAL_GD32F1/HAL_can_STM32F1.h"
#endif

class ExecuterManager
{
public:
  ExecuterManager(){};
  void Init();
  bool Detecte();
  
private:
  #if ENABLED(EXECUTER_CANBUS)
  uint8_t GetMachineTypeFromCAN(void);
  #endif
  #if ENABLED(EXECUTER_TEMPERATURE)
  uint8_t GetMachineTypeFromTemperature(void);
  #endif

public:
  uint8_t MachineType;
};

extern ExecuterManager ExecuterHead;

#endif //def _EXECUTER_MANAGER_H_
#endif //ENABLE EXECUTER_CANBUS