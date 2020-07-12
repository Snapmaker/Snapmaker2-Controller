#pragma once

#include "../inc/MarlinConfig.h"
#include "../snap_module/event_handler.h"

#define LASER_POWER_NORMAL_LIMIT   (100)
#define LASER_POWER_SAFE_LIMIT      (0.5)

#define LASER_CAMERA_FOCAL_LENGTH_MAX 65000

enum LaserCameraCommand {
  M_REPORT_VERSIONS = 0x1,
  S_REPORT_VERSIONS,
  M_CAMERA_GET_AWB = 0x3,
  S_CAMERA_GET_AWB_ACK,
  M_CAMERA_SET_AWB = 0x5,
  S_CAMERA_SET_AWB_ACK,
  M_CAMERA_SET_ACE = 0x7,
  S_CAMERA_SET_ACE_ACK,
  M_CAMERA_SET_IMG_SIZE = 0x9,
  S_CAMERA_SET_IMG_SIZE_ACK,
  M_CAMERA_SET_QUALITY = 0xb,
  S_CAMERA_SET_QUALITY_ACK,
  M_CAMERA_GET_IMG = 0xd,
  S_CAMERA_IMG_ACK,
  M_UPDATE_MOUDLE = 0xf,
  S_UPDATRE_ACK,
  M_SET_BT_NAME = 0x11,
  S_SET_BT_NAME_ACK,
  M_REPORT_BT_NAME = 0x13,
  S_REPORT_BT_NAME_ACK,
  M_REPORT_BT_MAC = 0x15,
  S_REPORT_BT_MAC_ACK,
  M_SET_CAMERA_LIGHT = 0x17,
  S_SET_CAMERA_LIGHT_ACK,
  M_REPORT_CAMERA_LIGHT = 0x19,
  S_REPORT_CAMERA_LIGHT_ACK,
  M_REPORT_CAMERA_STATU = 0x1b,
  S_REPORT_CAMERA_STATU_ACK,

  S_CAMERA_INIT_FAIL = 0xfd,
  S_RECV_FAIL = 0xff,
};

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
  void SetLaserPwm(uint16_t pwm);

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
  char SetBluetoothName(char *Name);
  char ReadBluetoothName(char *Name);
  char ReadBluetoothMac(uint8_t *Mac);

  void TryCloseFan();

  void ChangePowerLimit(float limit);

  ErrCode GetFocalLength(Event_t &event);
  ErrCode SetFocalLength(Event_t &event);
  ErrCode DoManualFocusing(Event_t &event);
  ErrCode DoAutoFocusing(Event_t &event);

  ErrCode SetCameraBtName(Event_t &event);
  ErrCode GetCameraBtName(Event_t &event);
  ErrCode GetCameraBtMAC(Event_t &event);

private:
  Host serial_;
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
  uint16_t  fan_tick_;

  float     power_limit_;
};

