#pragma once

#include "../inc/MarlinConfig.h"

//IO Switch Bits
#define PERIPH_IOSW_DOOR     1

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
    ChamberState LatestEnclosureEvent() { return cb_state_; }
    void LatestEnclosureEvent(ChamberState sta);
  #else
    void SetDoorCheck(bool Enable) {}
    FORCE_INLINE bool GetDoorCheckFlag() { return false; }
    FORCE_INLINE bool IsDoorOpened() { return true; }
  #endif

  uint8_t GetFanSpeed(uint8_t index) { return FanSpeed[index]; }

  #if ENABLED(CAN_FAN)
    void SetFanSpeed(uint8_t index, uint8_t DelayTime, uint8_t s_value);
    void SetEnclosureFanSpeed(uint8_t s_value);
  #endif
  
  void Process();

private:
  void CheckChamberDoor();

private:
  uint8_t FanSpeed[PERIPH_FAN_COUNT];
  ChamberState cb_state_;

public:
  uint8_t IOSwitch;
};

extern PeriphDevice Periph;

