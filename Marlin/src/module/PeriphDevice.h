#pragma once

#include "../inc/MarlinConfig.h"
#include "../snap_module/error.h"
#include "../snap_module/host.h"

//IO Switch Bits position
#define PERIPH_IOSW_DOOR     0  // bit[0]

#define PERIPH_FAN_COUNT 1

enum ChamberState : uint8_t {
  CHAMBER_STA_NONE,
  CHAMBER_STA_OPEN,
  CHAMBER_STA_OPEN_HANDLED,
  CHAMBER_STA_CLOSED,
  CHAMBER_STA_INVALID
};

class PeriphDevice
{
public:
  PeriphDevice(){};
  void Init();
  #if ENABLED(DOOR_SWITCH)
    void SetDoorCheck(bool Enable);
    FORCE_INLINE bool GetDoorCheckFlag() { return TEST(IOSwitch, PERIPH_IOSW_DOOR); }
    bool IsDoorOpened();
    uint8_t GetEnclosureLightPower();
    uint8_t GetEnclosureFanSpeed();
  #else
    void SetDoorCheck(bool Enable) {}
    FORCE_INLINE bool GetDoorCheckFlag() { return false; }
    FORCE_INLINE bool IsDoorOpened() { return true; }
  #endif

  #if ENABLED(CAN_FAN)
    void SetEnclosureFanSpeed(uint8_t s_value);
    void SetEnclosureLightPower(uint8_t s_value);
  #endif

  void SetUartLock(bool f);
  bool FORCE_INLINE GetHoldUart() { return TEST(IOSwitch, PERIPH_IOSW_DOOR) && lock_uart_; }

  void ReportStatus();

  void CheckStatus();

  void TriggerDoorEvent(bool open);
  bool IsOnline(uint8_t periph_mask) { return TEST(online_, periph_mask); }

  // callback for HMI events
  ErrCode ReportEnclosureStatus(Event_t &event);
  ErrCode SetEnclosureFan(Event_t &event);
  ErrCode SetEnclosureLight(Event_t &event);
  ErrCode SetEnclosureDetection(Event_t &event);

private:
  void CheckChamberDoor();
  void TellUartState();
  void OpenDoorTrigger();
  void CloseDoorTrigger();

private:
  uint8_t FanSpeed[PERIPH_FAN_COUNT];
  uint8_t enclosure_light_power_ = 0;
  uint8_t enclosure_fan_speed_ = 0;
  ChamberState cb_state_;
  bool      lock_uart_;
  millis_t  next_ms_;
  uint8_t   online_;

public:
  uint8_t IOSwitch;
};

extern PeriphDevice Periph;

