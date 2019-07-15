#pragma once

#include "../inc/MarlinConfig.h"

//IO Switch Bits
#define PERIPH_IOSW_DOOR     (1<<0)
#define PERIPH_IOSW_PROBE    (1<<1)

//Periph IO Bits
#define PERIPH_IOL_DOOR   (1<<26)
#define PERIPH_IOL_PROBE  (1<<25)

#define PERIPH_FAN_COUNT 1

class PeriphDevice 
{
public:
  PeriphDevice(){};
  void Init();
  #if ENABLED(DOOR_SWITCH)
    void DoorSwitchInit();
    void SetDoorCheck(bool Enable);
    void StartDoorCheck();
    void StopDoorCheck();
    FORCE_INLINE bool GetDoorCheckFlag() { return (IOSwitch & PERIPH_IOSW_DOOR) != 0; } 
    FORCE_INLINE bool IsDoorOpened() { return (IOLevel & PERIPH_IOL_DOOR) != 0; }
  #else
    void DoorSwitchInit() {}
    void SetDoorCheck(bool Enable) {}
    void StartDoorCheck() {}
    void StopDoorCheck() {}
    FORCE_INLINE bool GetDoorCheckFlag() { return false; } 
    FORCE_INLINE bool IsDoorOpened() { return true; }
  #endif

  uint8_t GetFanSpeed(uint8_t index) { return FanSpeed[index]; }

  #if ENABLED(CAN_FAN)
    void SetFanSpeed(uint8_t index, uint8_t DelayTime, uint8_t s_value);
  #endif

  
private:
  uint8_t FanSpeed[PERIPH_FAN_COUNT];

public:
  uint8_t IOSwitch;
};

extern PeriphDevice Periph;

