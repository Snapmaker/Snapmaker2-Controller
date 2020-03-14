#pragma once

#include "../inc/MarlinConfig.h"

#ifndef _EXECUTER_MANAGER_H_
#define _EXECUTER_MANAGER_H_

#if ENABLED(EXECUTER_CANBUS_SUPPORT)
#include "CanBus.h"
#include "CanModule.h"
#endif
#include "CNCExecuter.h"
#include "LaserExecuter.h"
#include "PrintExecuter.h"
#include "../snap_module/heartbeat_watch.h"

#define EXECUTER_FAN_COUNT  4

class ExecuterManager
{
public:
  ExecuterManager(){};
  void Init();
  bool Detecte();
  void Process();

  void CheckAlive();

  // callbacks to handle door events
  void CallbackOpenDoor();
  void CallbackCloseDoor();

  #if ENABLED(EXECUTER_CANBUS_SUPPORT)
    void SetTemperature(uint8_t index, uint16_t temperature);
    void SetFanDelayOff(uint8_t index, uint8_t time);
    void SetFan(uint8_t index, uint8_t s_value);
    uint8_t GetFanSpeed(uint8_t i) { if (i < EXECUTER_FAN_COUNT) return FanSpeed[i]; else return 0; }
    float GetTemp(uint8_t hotendindex) { return temp_hotend[hotendindex]; }
  #endif

private:
  #if DISABLED(EXECUTER_CANBUS_SUPPORT)
    uint8_t GetMachineTypeFromTemperature(void);
  #endif

  uint8_t keep_alive_;
  bool dead_;

public:
  HeartbeatWatch watch;
  LaserExecuter Laser;
  CNCExecuter CNC;
  PrintExecuter Print3D;
  uint8_t MachineType;
  bool CanTempMeasReady;
  float temp_hotend[HOTENDS];
  uint8_t FanSpeed[EXECUTER_FAN_COUNT];

};

extern ExecuterManager ExecuterHead;

#endif //def _EXECUTER_MANAGER_H_