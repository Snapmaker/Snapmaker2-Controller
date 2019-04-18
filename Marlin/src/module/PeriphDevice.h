#pragma once

#include "../inc/MarlinConfig.h"

//IO Switch Bits
#define PERIPH_IOW_DOOR     1
#define PERIPH_IOW_FILAMENT 2


class PeriphDevice 
{
public:
  PeriphDevice(){};
  void Init();
  #if ENABLED(DOOR_SENSOR)
  void SetDoorCheck(bool Enable);
  void StartDoorCheck();
  void StopDoorCheck();
  #else
  void SetDoorCheck(bool Enable){}
  void StartDoorCheck(){}
  void StopDoorCheck(){}
  #endif

  #if ENABLED(FILAMENT_SENSOR)
  void SetFilamentCheck(bool Enable);
  void StartFilamentCheck();
  void StopFilamentCheck();
  #else
  void SetFilamentCheck(bool Enable){}
  void StartFilamentCheck(){}
  void StopFilamentCheck(){}
  #endif
  FORCE_INLINE bool GetDoorCheckFlag() { return (IOSwitch & PERIPH_IOW_DOOR) != 0; } 

public:
  uint8_t IOSwitch;
};

extern PeriphDevice Periph;
