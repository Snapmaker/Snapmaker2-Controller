#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "CNCExecuter.h"
#include "CanBus.h"


#if ENABLED(EXECUTER_CANBUS_SUPPORT)

/**
 * Init
 */
void CNCExecuter::Init()
{
  
}

/**
 * SetCNCPower:Set CNC RPM
 * para RPMValue:Real RPM Value,between 6000 ~ 12000
 */
void CNCExecuter::SetCNCRPM(uint16_t RPMValue)
{
  
}

/**
 * SetCNCPower:Set CNC power
 * para percent:
 */
void CNCExecuter::SetCNCPower(float Percent)
{
  uint8_t Data[3];

  Data[0] = 0;
  Data[1] = Percent;
  
}

#else

/**
 * Init
 */
void CNCExecuter::Init()
{
  OUT_WRITE(CNC_PIN, 0);
}

/**
 * SetCNCRPM:Set CNC RPM
 * para RPMValue:above 0 ，start
 */
void CNCExecuter::SetCNCRPM(uint16_t RPMValue)
{
  if(RPMValue > 0)
    WRITE(CNC_PIN, true);
  else
    WRITE(CNC_PIN, false);
}

/**
 * SetCNCPower:Set CNC power
 * para percent:50-100
 */
void CNCExecuter::SetCNCPower(float Percent)
{
  if(Percent > 50)
    WRITE(CNC_PIN, true);
  else
    WRITE(CNC_PIN, false);
}

#endif // ENABLED EXECUTER_CANBUS_SUPPORT