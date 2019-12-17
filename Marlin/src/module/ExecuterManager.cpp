#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "temperature.h"
#include "configuration_store.h"
#include "ExecuterManager.h"
#include "CanBus.h"
#include "CanDefines.h"
#include "StatusControl.h"
#include "PeriphDevice.h"
#include "../snap_module/quickstop.h"

ExecuterManager ExecuterHead;

/**
 * Executer Init
 */
void ExecuterManager::Init()
{
  MachineType = MACHINE_TYPE_UNDEFINE;
  keep_alive_ = 0xff;
  dead_ = false;

  watch.Init(4, 1000);
}

/**
 * Executer Detect
 * return   true:Raack have received, false:No reack.result will be saved in member "MachineType"
 */
bool ExecuterManager::Detecte()
{
  #if DISABLED(EXECUTER_CANBUS_SUPPORT)
    MachineType = GetMachineTypeFromTemperature();
  #endif
  return (MachineType != MACHINE_TYPE_UNDEFINE);
}

void ExecuterManager::Process() {

  Laser.TryCloseFan();

}

void ExecuterManager::CheckAlive() {
  static millis_t next_second = millis() + 1000;
  millis_t now = millis();

  if (ELAPSED(now, next_second)) {
    // because laser deosn't have heartbeat packet, so need to read it manually
    // when it return focusheight, ISR will update the keep_alive_
    if (MACHINE_TYPE_LASER == MachineType) {
      CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_REPORT_LASER_FOCUS, NULL, 0);
    }

    next_second = millis() + 1000;
  }

  if (watch.CheckAlive() == HB_STA_JUST_DEAD) {
    SystemStatus.ThrowException(EHOST_EXECUTOR, ETYPE_LOST_HOST);
    // shutdown the power, to avoid heater keep heating up while toolhead malfunction.
    disable_power_domain(POWER_DOMAIN_1 | POWER_DOMAIN_2);
  }

  if (watch.CheckAlive() == HB_STA_JUST_ALIVE) {
    SystemStatus.ClearException(EHOST_EXECUTOR, ETYPE_LOST_HOST);
    enable_power_domain(POWER_DOMAIN_1 | POWER_DOMAIN_2);
  }
}

void ExecuterManager::CallbackOpenDoor() {
  switch (MachineType) {
  case MACHINE_TYPE_LASER:
    // just handle working with PC
    if (SystemStatus.GetWorkingPort() != WORKING_PORT_SC)
      Laser.ChangePowerLimit(LASER_POWER_SAFE_LIMIT);
    break;

  case MACHINE_TYPE_CNC:
    // just handle working with PC
    if (CNC.GetRPM() > 0 && SystemStatus.GetWorkingPort() != WORKING_PORT_SC) {
      quickstop.Trigger(QS_EVENT_STOP);
      Periph.SetUartLock(true);
    }
    break;

  default:
    break;
  }
}

void ExecuterManager::CallbackCloseDoor() {
  switch (MachineType) {
  case MACHINE_TYPE_LASER:
    Laser.ChangePowerLimit(LASER_POWER_NORMAL_LIMIT);
    break;

  case MACHINE_TYPE_CNC:
    Periph.SetUartLock(false);
    break;

  default:
    break;
  }
}


#if ENABLED(EXECUTER_CANBUS_SUPPORT)
  /**
   * SetTemperature:Set temperature
   * para index:executer index
   * temperature:
   */
  void ExecuterManager::SetTemperature(uint8_t index, uint16_t temperature)
  {
    uint8_t Data[2];

    Data[0] = (uint8_t)(temperature >> 8);
    Data[1] = (uint8_t)temperature;
    CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_TEMPEARTURE, Data, 2);
    if(temperature > 60) {
      SetFan(1, 255);
    }

    // because when leave control page of screen, it won't turn off FAN
    // So need to close FAN here
    if (temperature == 0) {
      if (thermalManager.degHotend(0) > 150) {
        SetFanDelayOff(1, 120);
      }
      else if (thermalManager.degHotend(0) > 60) {
        // delay 60s to turn off FAN
        SetFanDelayOff(1, 60);
      }
      else {
        SetFan(1, 0);
      }
    }
  }

  /**
   * SetFanDelayOff:
   * para index:executer index
   * para time:time to delay in second
   * percent:fan speed in percent
   */
  void ExecuterManager::SetFanDelayOff(uint8_t index, uint8_t time)
  {
    uint8_t Data[8];

    if (index > (EXECUTER_FAN_COUNT-1))
      return;

    if (MachineType != MACHINE_TYPE_3DPRINT)
      return;

    Data[0] = time;
    Data[1] = 0;

    FanSpeed[index] = 0;

    if(index == 0) {
      CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_FAN, Data, 3);
    }
    else if(index == 1) {
      CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_FAN2, Data, 3);
    }
  }
  /**
   * SetFan:Set fan
   * para index:executer index
   * percent:fan speed in percent
   */
  void ExecuterManager::SetFan(uint8_t index, uint8_t s_value)
  {
    uint8_t Data[8];

    if (index > (EXECUTER_FAN_COUNT-1))
      return;

    if (MachineType != MACHINE_TYPE_3DPRINT)
      return;

    Data[0] = 0;
    Data[1] = s_value;

    FanSpeed[index] = s_value;

    if(index == 0) {
      CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_FAN, Data, 2);
    }
    else if(index == 1) {
      CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_FAN2, Data, 2);
    }
  }
#else

  /**
   * GetMachineTypeFromTemperature:Get machine type from Temerature
   * return   One of MACHINE_TYPE_CNC,MACHINE_TYPE_LASER,MACHINE_TYPE_3DPRINT
   */
  uint8_t ExecuterManager::GetMachineTypeFromTemperature(void)
  {
    uint8_t type;
    millis_t tmpTick;
    tmpTick = millis();
    type = MACHINE_TYPE_UNDEFINE;
    while((millis() - tmpTick) < 1000L)
      thermalManager.manage_heater();
    while(type == MACHINE_TYPE_UNDEFINE) {
      thermalManager.manage_heater();
      if(thermalManager.temp_hotend[0].current > 400) {
        type = MACHINE_TYPE_CNC;
      }
      else if(thermalManager.temp_hotend[0].current > 260) {
        type = MACHINE_TYPE_LASER;
        SET_INPUT(HEATER_0_PIN);
        SET_INPUT(HEATER_BED_PIN);
      }
      else {
        type = MACHINE_TYPE_3DPRINT;
        SET_INPUT(HEATER_0_PIN);
        SET_INPUT(HEATER_BED_PIN);
      }
    }
    return type;
  }

#endif //ENABLED(EXECUTER_CANBUS_SUPPORT)
