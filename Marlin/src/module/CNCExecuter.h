#pragma once

#include "../inc/MarlinConfig.h"


class CNCExecuter 
{
public:
  CNCExecuter(){};
  void Init();
  void SetCNCRPM(uint16_t RPMValue);
  void SetCNCPower(float Percent);

public:
  uint8_t Percent;
  uint16_t RPM;

};

