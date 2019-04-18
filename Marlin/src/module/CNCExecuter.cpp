#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "CNCExecuter.h"


CNCExecuter CNC;

#if ENABLED(EXECUTER_CANBUS)
/**
 * Init
 */
void CNCExecuter::Init()
{
  CanInit(EXECUTER_CAN_PORT);
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
void CNCExecuter::SetCNCRPM(uint8_t Percent)
{
  
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
void CNCExecuter::SetCNCPower(uint8_t Percent)
{
  if(Percent > 50)
    WRITE(CNC_PIN, true);
  else
    WRITE(CNC_PIN, false);
}


#endif
