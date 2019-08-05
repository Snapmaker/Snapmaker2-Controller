#pragma once

#include "../inc/MarlinConfig.h"


class PrintExecuter
{
public:
  PrintExecuter(){};
  void Init();
  bool HeatedBedSelfCheck(void);

private:
  void EStepperInit();
};

