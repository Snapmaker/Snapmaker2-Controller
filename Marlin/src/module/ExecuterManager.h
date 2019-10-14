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

#define EXECUTER_FAN_COUNT  4

#define FAN1_MASK 0x01
#define FAN2_MASK 0x02
#define FAN3_MASK 0x04
#define FAN4_MASK 0x08
#define FAN_ALL_MASK 0x0F

class ExecuterManager
{
public:
  ExecuterManager(){};
  void Init();
  bool Detecte();
  bool fan_state(uint8_t mask) { return (bool)(fan_state_ & mask); }

  #if ENABLED(EXECUTER_CANBUS_SUPPORT)
    void SetTemperature(uint8_t index, uint16_t temperature);
    void SetFanDelayOff(uint8_t index, uint8_t time, uint8_t s_value);
    void SetFan(uint8_t index, uint8_t s_value);
    float GetTemp(uint8_t hotendindex) { return temp_hotend[hotendindex]; }
  #endif
  
private:
  #if DISABLED(EXECUTER_CANBUS_SUPPORT)
    uint8_t GetMachineTypeFromTemperature(void);
  #endif
    uint8_t fan_state_;

    void SetFanState(uint8_t speed, uint8_t idx);

public:
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