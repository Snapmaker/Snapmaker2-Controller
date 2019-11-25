#pragma once

#include "../inc/MarlinConfig.h"


class CNCExecuter 
{
public:
  CNCExecuter(){};
  void Init();
  void SetPower(float Percent);
  float GetPower() { return percent; };
  void UpdateWorkingRPM(uint16_t NewRPM);
  void On();
  void Off();
  uint16_t GetRPM() { return RPM; }

  void ChangePower(float Percent);

private:
  float percent;
  uint16_t RPM;

};

