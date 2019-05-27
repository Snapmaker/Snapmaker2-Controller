#pragma once

#include "../inc/MarlinConfig.h"

void Tim1PwmInit();
void Tim1SetCCR2(uint16_t Value);

class LaserExecuter 
{
public:
  LaserExecuter(){};
  void Init();

  void SetLaserPower(float Percent);
  void SetLaserPower(uint16_t PwmValue);
  void SetLaserLowPower();
  void LaserOff();
  void LaserOn();
  void SavePlatformHeight(float height);
  void LoadPlatformHeight();
  #if ENABLED(EXECUTER_CANBUS_SUPPORT)
    void SaveFocusHeight(uint8_t index, float height);
    void SaveFocusHeight();
    bool LoadFocusHeight();
  #else
    static void SaveFocusHeight(uint8_t index, float height) {}
    static void SaveFocusHeight() {}
    static bool LoadFocusHeight() { return false; }
  #endif

public:
  float LastPercent;
  float FocusHeight;
  float PlatformHeight;
  
private:
  uint8_t LastSetIndex;
};

