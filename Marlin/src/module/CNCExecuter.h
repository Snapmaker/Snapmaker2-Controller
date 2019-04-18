#pragma once

#include "../inc/MarlinConfig.h"


class CNCExecuter 
{
public:
  CNCExecuter(){};
  void Init();

  void SetCNCRPM(uint16_t RPMValue);
  void SetCNCPower(uint8_t Percent);

public:
  uint8_t Percent;

};

extern CNCExecuter CNC;
