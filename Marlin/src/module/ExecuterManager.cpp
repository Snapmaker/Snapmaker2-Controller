#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "temperature.h"
#include "configuration_store.h"
#include "ExecuterManager.h"
#include "CanBus.h"

ExecuterManager ExecuterHead;

/**
 * Executer Init
 */
void ExecuterManager::Init()
{
  #if ENABLED(EXECUTER_CANBUS_SUPPORT)
  CanBusControlor.Init();
  #endif
}

/**
 * Executer Detect
 * return   true:Raack have received, false:No reack.result will be saved in member "MachineType"
 */
bool ExecuterManager::Detecte()
{
  #if ENABLED(EXECUTER_CANBUS_SUPPORT)
    MachineType = GetMachineTypeFromCAN();
  #else
    MachineType = GetMachineTypeFromTemperature();
  #endif
  return (MachineType != MACHINE_TYPE_UNDEFINE);
}


#if ENABLED(EXECUTER_CANBUS_SUPPORT)
  /**
   * Get machine type from Can Bus
   * return   One of MACHINE_TYPE_CNC,MACHINE_TYPE_LASER,MACHINE_TYPE_3DPRINT
   */
  uint8_t ExecuterManager::GetMachineTypeFromCAN(void)
  {
    uint32_t Err;
    uint8_t retry;
    millis_t tmpTick;
    MachineType = MACHINE_TYPE_UNDEFINE;
    uint8_t Buff[4];
    retry = 5;
    tmpTick = millis();
    Buff[0] = 0;
    while(MachineType == MACHINE_TYPE_UNDEFINE) {
      if((millis() - tmpTick) > 500L) {
        tmpTick = millis();
        if(retry-- == 0)
          return MACHINE_TYPE_UNDEFINE;
        if(CanBusControlor.SendData(2, CAN_IDS_BC, Buff, 1, &Err) == false)
          return MACHINE_TYPE_UNDEFINE;
      }
    }
    return MachineType;
  }

  /**
   * SetTemperature:Set temperature 
   * para index:executer index
   * temperature:
   */
  void ExecuterManager::SetTemperature(uint8_t index, uint16_t temperature)
  {
    uint8_t Data[3];

    Data[0] = index & 0x3F;
    Data[1] = (uint8_t)(temperature >> 8);
    Data[2] = (uint8_t)temperature;
    CanBusControlor.SendData(2, CAN_IDS_TEMP_CONTROL, Data, 3);
    SERIAL_ECHOLN("Set Tamp");
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
    Data[1] = index;
    Data[2] = 0;
    Data[3] = s_value;
    FanSpeed[index] = s_value;
    CanBusControlor.SendData(2, CAN_IDS_FAN, Data, 4);
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
