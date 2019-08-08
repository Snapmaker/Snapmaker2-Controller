#pragma once

#include "../inc/MarlinConfig.h"


class CNCExecuter 
{
public:
  CNCExecuter(){};
  void Init();
  void SetCNCRPM(uint16_t RPMValue);
  void SetCNCPower(float Percent);
  void On();
  void Off();

public:
  float percent;
  uint16_t RPM;

};

