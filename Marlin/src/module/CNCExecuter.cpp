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
void CNCExecuter::Init() {
  
}

/**
 * SetCNCPower:Set CNC power
 * para percent:Only Support 50-100 int
 */
void CNCExecuter::SetPower(float Percent) {
  uint8_t Data[2];
  if(Percent < 50)
    Percent = 0;
  percent = Percent;
  Data[0] = Percent;
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_MOTOR_SPEED, Data, 1);
}

/**
 * UpdateRPM:Update the RPM
 * para NewRPM:New rpm got
 */
void CNCExecuter::UpdateWorkingRPM(uint16_t NewRPM)
{
  RPM = NewRPM;
}

/**
 * On:Turn on the CNC and set to the last percent power
 */
void CNCExecuter::On() {
  uint8_t Data[2];
  Data[0] = percent;
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_MOTOR_SPEED, Data, 1);
}

/**
 * Off:Turn off the CNC
 */
void CNCExecuter::Off() {
  uint8_t Data[2];
  Data[0] = 0;
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_MOTOR_SPEED, Data, 1);
} 

/**
 * change power but not set to module
 */
void CNCExecuter::ChangePower(float Percent) {
  if(Percent < 50)
    Percent = 0;
  percent = Percent;
}

#else

/**
 * Init
 */
void CNCExecuter::Init() {
  OUT_WRITE(CNC_PIN, 0);
}

/**
 * SetCNCRPM:Set CNC RPM
 * para RPMValue:above 0 ，start
 */
void CNCExecuter::SetCNCRPM(uint16_t RPMValue) {
  if(RPMValue > 0) WRITE(CNC_PIN, true);
  else WRITE(CNC_PIN, false);
}

/**
 * SetCNCPower:Set CNC power
 * para percent:50-100
 */
void CNCExecuter::SetCNCPower(float Percent) {
  if(Percent > 50) WRITE(CNC_PIN, true);
  else WRITE(CNC_PIN, false);
}

#endif // ENABLED EXECUTER_CANBUS_SUPPORT