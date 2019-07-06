#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "temperature.h"
#include "configuration_store.h"
#include "ExecuterManager.h"
#include "CanBus.h"
#include "CanDefines.h"

ExecuterManager ExecuterHead;

/**
 * Executer Init
 */
void ExecuterManager::Init()
{
  MachineType = MACHINE_TYPE_UNDEFINE;
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


#if ENABLED(EXECUTER_CANBUS_SUPPORT)
  /**
   * SetTemperature:Set temperature 
   * para index:executer index
   * temperature:
   */
  void ExecuterManager::SetTemperature(uint8_t index, uint16_t temperature)
  {
    uint8_t Data[3];

    Data[0] = (uint8_t)(temperature >> 8);
    Data[1] = (uint8_t)temperature;
    
    SERIAL_ECHOLN("Set Tamp");
    CanModules.SetFunctionValue(2, FUNC_SET_TEMPEARTURE, Data);
  }

  /**
   * SetFan:Set fan 
   * para index:executer index
   * percent:fan speed in percent
   */
  void ExecuterManager::SetFan(uint8_t index, uint8_t s_value)
  {
    uint8_t Data[8];

    Data[0] = 0;
    Data[1] = s_value;
    FanSpeed[index] = s_value;
    if(index == 0) CanModules.SetFunctionValue(2, FUNC_SET_FAN, Data);
    else if(index == 1) CanModules.SetFunctionValue(2, FUNC_SET_FAN2, Data);
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
