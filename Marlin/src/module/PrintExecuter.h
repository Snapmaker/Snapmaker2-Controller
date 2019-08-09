#pragma once

#include "../inc/MarlinConfig.h"


class PrintExecuter
{
public:
  PrintExecuter(){};
  void Init();
  bool HeatedBedSelfCheck(void);
  bool SetPID(uint8_t Index, float Value);

private:
  void EStepperInit();
};

