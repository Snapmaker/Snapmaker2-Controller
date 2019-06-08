#pragma once

#include "../inc/MarlinConfig.h"


class PrintExecuter 
{
public:
  PrintExecuter(){};
  void Init();

private:
  void EStepperInit();
};

