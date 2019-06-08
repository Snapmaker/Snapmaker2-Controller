#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "periphdevice.h"
#include "../HAL/HAL_GD32F1/HAL_exti_STM32F1.h"
#include "CanBus.h"

PeriphDevice Periph;

/**
 * Init
 */
void PeriphDevice::Init()
{
  IOLevel = 0xffffffff;
}

#if ENABLED(CAN_FAN)
/**
 * SetFanSpeed:Set Fan Speed
 * para DelayTime:
 * para index:
 * para percent:
 */
void PeriphDevice::SetFanSpeed(uint8_t index, uint8_t DelayTime, uint8_t s_value)
{
  uint8_t Data[3];

  Data[0] = index;
  Data[1] = DelayTime;
  Data[2] = s_value;
  FanSpeed[index] = s_value;
  CanBusControlor.SendData(1, CAN_IDS_FAN, Data, 3);
}
#endif

#if ENABLED(DOOR_SWITCH)

/**
 * DoorSwitchInit:Initialze the door switch IO
 * para Enable:true enable ,false disable
 */
void PeriphDevice::DoorSwitchInit()
{
  #if DISABLED(PERIPH_CANBUS_SUPPORT)
  #endif
}

/**
 * SetDoorCheck:enable or disable Door Sensor
 * para Enable:true enable ,false disable
 */
void PeriphDevice::SetDoorCheck(bool Enable)
{
  #if DISABLED(PERIPH_CANBUS_SUPPORT)
  #endif
}

/**
 * StartDoorCheck:Start Door Sensor working
 * para percent:
 */
void PeriphDevice::StartDoorCheck()
{
  #if DISABLED(PERIPH_CANBUS_SUPPORT)

  #endif
}

/**
 * StopDoorCheck:Stop Door Sensor working
 * para percent:
 */
void PeriphDevice::StopDoorCheck()
{
  #if DISABLED(PERIPH_CANBUS_SUPPORT)
  #endif
}
#endif //ENABLED(DOOR_SWITCH)

#if ENABLED(FILAMENT_SENSOR)
/**
 * SetFilamentCheck:enable or disable Filament Sensor
 * para Enable:true enable ,false disable
 */
void PeriphDevice::SetFilamentCheck(bool Enable)
{
  #if ENABLED(PERIPH_CANBUS_SUPPORT)

  #endif
}

/**
 * StartFilamentCheck:Start Filamnent Sensor working
 * para percent:
 */
void PeriphDevice::StartFilamentCheck()
{
  #if ENABLED(PERIPH_CANBUS_SUPPORT)

  #endif
}

/**
 * StopFilamentCheck:Stop Filamnent Sensor working
 * para percent:
 */
void PeriphDevice::StopFilamentCheck()
{
  #if ENABLED(PERIPH_CANBUS_SUPPORT)

  #endif
}
#endif // ENABLED(FILAMENT_SENSOR)

#if ENABLED(CAN_ENCLOSE_LED)

/**
 * EncloseLedOn:Turn on the led of enclose
 */
void PeriphDevice::EncloseLedOn()
{
  uint8_t Data[6];
  Data[0] = 0;
  Data[1] = 1;
  CanBusControlor.SendData(1, CAN_IDS_LIGHT, Data, 6);
}

/**
 * EncloseLedOff:Turn off the led of enclose
 */
void PeriphDevice::EncloseLedOff()
{
  uint8_t Data[6];
  Data[0] = 0;
  Data[1] = 0;
  Data[2] = 0;
  CanBusControlor.SendData(1, CAN_IDS_LIGHT, Data, 6);
}
#endif

/**
 * LevelingSensorValid:Check the bed leveling sensor valid
 * return: true for valid
 */
bool PeriphDevice::LevelingSensorValid()
{
  if(LevelingSensorExisted() == false)
    return false;
  if(IOSwitch & PERIPH_IOSW_BLS)
    return true;
  else
    return false;
}

/**
 * LevelingSensorValid:Check the bed leveling sensor valid
 * return: true for valid
 */
bool PeriphDevice::LevelingSensorExisted()
{
  #if ENABLED(BED_LEVELING_SENSOR)
    return true;
  #else
    return false;
  #endif
}

