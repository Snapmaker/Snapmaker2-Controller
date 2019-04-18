#pragma once

#include "../inc/MarlinConfig.h"

void Tim1PwmInit();
void Tim1SetCCR2(uint16_t Value);

class LaserExecuter 
{
public:
  LaserExecuter(){};
  void Init();

  void SetLaserPower(uint8_t Percent);
  void SetLaserPower(uint16_t PwmValue);
  void SetLaserLowPower();

public:
  uint8_t LastPercent;

};

extern LaserExecuter Laser;
