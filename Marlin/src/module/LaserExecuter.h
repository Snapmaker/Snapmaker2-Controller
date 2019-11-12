#pragma once

#include "../inc/MarlinConfig.h"

#define LASER_POWER_NORMAL_LIMIT   (100)
#define LASER_POWER_SAFE_LIMIT      (0.5)

void Tim1PwmInit();
void Tim1SetCCR1(uint16_t Value);
void Tim1SetCCR2(uint16_t Value);
void Tim1SetCCR3(uint16_t Value);
void Tim1SetCCR4(uint16_t Value);
uint16_t Tim1GetCCR4(void);

enum LAESR_FAN_STATE : uint8_t {
  LAESR_FAN_STA_OPEN,
  LAESR_FAN_STA_TO_BE_CLOSED,
  LAESR_FAN_STA_CLOSED,

  LAESR_FAN_STA_INVALID
};


class LaserExecuter
{
public:
  LaserExecuter(){};
  void Init();

  void SetLaserPower(float Percent);
  void SetLaserLowPower();
  void Off();
  void On();
  uint32_t GetPower() { return (uint32_t)(last_percent * 1000.0f); };
  float GetPowerPercent() { return last_percent; }
  uint16_t GetTimPwm();
  void ChangePower(float percent);

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
  char SetBluetoothName(char *Name);
  char ReadBluetoothName(char *Name);
  char ReadBluetoothMac(uint8_t *Mac);

  void TryCloseFan();

  void ChangePowerLimit(float limit);

private:
  void PackedProtocal(uint8_t *pData, uint16_t len);
  char GetReply(uint8_t *Buff, millis_t Timeout);
  void CheckFan(uint16_t p);

public:
  float FocusHeight;

private:
  float     last_percent;
  uint16_t  last_pwm;
  uint8_t   tmpBuff[128];

  uint8_t   fan_state_;
  millis_t  fan_tick_;

  float     power_limit_;
};

