#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "periphdevice.h"
#include "../HAL/HAL_GD32F1/HAL_exti_STM32F1.h"
#include "CanBus.h"
#include "CanModule.h"

PeriphDevice Periph;

/**
 * Init
 */
void PeriphDevice::Init()
{
  
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
