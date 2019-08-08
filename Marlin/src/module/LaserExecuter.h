#pragma once

#include "../inc/MarlinConfig.h"

void Tim1PwmInit();
void Tim1SetCCR1(uint16_t Value);
void Tim1SetCCR2(uint16_t Value);
void Tim1SetCCR3(uint16_t Value);
void Tim1SetCCR4(uint16_t Value);


class LaserExecuter 
{
public:
  LaserExecuter(){};
  void Init();

  void SetLaserPower(float Percent);
  void SetLaserPower(uint16_t PwmValue);
  void SetLaserLowPower();
  void Off();
  void On();
  void SavePlatformHeight(float height);
  void LoadPlatformHeight();
  #if ENABLED(EXECUTER_CANBUS_SUPPORT)
    void SaveFocusHeight(float height);
    void SaveFocusHeight();
    bool LoadFocusHeight();
  #else
    static void SaveFocusHeight(uint8_t index, float height) {}
    static void SaveFocusHeight() {}
    static bool LoadFocusHeight() { return false; }
  #endif
  char ReadWifiStatus(char *SSID, char *Password, char *IP);
  char SetWifiParameter(char *SSID, char *Password);

private:
  void PackedProtocal(uint8_t *pData, uint16_t len);
  char GetReply(uint8_t *Buff, millis_t Timeout);

public:
  float LastPercent;
  float FocusHeight;
  float PlatformHeight;
  
private:
  uint8_t LastSetIndex;
  uint8_t tmpBuff[128];
};

