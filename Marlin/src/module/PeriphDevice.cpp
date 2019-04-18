#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "periphdevice.h"
#include "../HAL/HAL_GD32F1/HAL_exti_STM32F1.h"


PeriphDevice Periph;

#if ENABLED(PERIPH_CANBUS)
/**
 * Init
 */
void PeriphDevice::Init()
{
  CanInit(PERIPH_EXECUTER_CAN_PORT);
}

/**
 * SetCNCPower:Set laser power
 * para percent:
 */

#else
/**
 * Init
 */
void PeriphDevice::Init()
{
  #if ENABLED(DOOR_SENSOR)
    ExtiInit(PA, 0, 0);
  #endif
}

#if ENABLED(DOOR_SENSOR)
/**
 * SetDoorCheck:enable or disable Door Sensor
 * para Enable:true enable ,false disable
 */
void PeriphDevice::SetDoorCheck(bool Enable)
{
  
}

/**
 * StartDoorCheck:Start Door Sensor working
 * para percent:
 */
void PeriphDevice::StartDoorCheck()
{
}

/**
 * StopDoorCheck:Stop Door Sensor working
 * para percent:
 */
void PeriphDevice::StopDoorCheck()
{
}
#endif //ENABLED(DOOR_SENSOR)


#if ENABLED(FILAMENT_SENSOR)
/**
 * SetFilamentCheck:enable or disable Filament Sensor
 * para Enable:true enable ,false disable
 */
void PeriphDevice::SetFilamentCheck(bool Enable)
{
  
    
  
}

/**
 * StartFilamentCheck:Start Filamnent Sensor working
 * para percent:
 */
void PeriphDevice::StartFilamentCheck()
{
}

/**
 * StopFilamentCheck:Stop Filamnent Sensor working
 * para percent:
 */
void PeriphDevice::StopFilamentCheck()
{
}
#endif // ENABLED(FILAMENT_SENSOR)


#endif
