#pragma once

#include "../inc/MarlinConfig.h"

void Tim1PwmInit();
void Tim1SetCCR1(uint16_t Value);
void Tim1SetCCR2(uint16_t Value);
void Tim1SetCCR3(uint16_t Value);
void Tim1SetCCR4(uint16_t Value);
uint16_t Tim1GetCCR4(void);

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
  uint32_t GetPower() { return (uint32_t)(last_percent * 1000.0f); };
  float GetPowerPercent() { return last_percent; }
  uint16_t GetTimPwm();
  void RestorePower(float percent, uint16_t pwm);

  void UpdateLaserPower(float NewPower);
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
  void CheckFan();

public:
  float FocusHeight;
  
private:
  float last_percent;
  uint8_t tmpBuff[128];
};

