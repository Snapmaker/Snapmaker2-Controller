#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "CNCExecuter.h"
#include "StatusControl.h"
#include "CanBus.h"

/**
 * EStepperInit:E stepper control io initialze
 */
void PrintExecuter::EStepperInit()
{
  #if HAS_E0_DIR
    SET_OUTPUT(E0_DIR_PIN);
  #endif
  #if HAS_E1_DIR
    SET_OUTPUT(E1_DIR_PIN);
  #endif
  #if HAS_E2_DIR
    SET_OUTPUT(E2_DIR_PIN);
  #endif
  #if HAS_E3_DIR
    SET_OUTPUT(E3_DIR_PIN);
  #endif
  #if HAS_E4_DIR
    SET_OUTPUT(E4_DIR_PIN);
  #endif
  #if HAS_E5_DIR
    SET_OUTPUT(E5_DIR_PIN);
  #endif

  #if HAS_E0_ENABLE
    SET_OUTPUT(E0_ENABLE_PIN);
    if (!E_ENABLE_ON) WRITE(E0_ENABLE_PIN, HIGH);
  #endif
  #if HAS_E1_ENABLE
    SET_OUTPUT(E1_ENABLE_PIN);
    if (!E_ENABLE_ON) WRITE(E1_ENABLE_PIN, HIGH);
  #endif
  #if HAS_E2_ENABLE
    SET_OUTPUT(E2_ENABLE_PIN);
    if (!E_ENABLE_ON) WRITE(E2_ENABLE_PIN, HIGH);
  #endif
  #if HAS_E3_ENABLE
    SET_OUTPUT(E3_ENABLE_PIN);
    if (!E_ENABLE_ON) WRITE(E3_ENABLE_PIN, HIGH);
  #endif
  #if HAS_E4_ENABLE
    SET_OUTPUT(E4_ENABLE_PIN);
    if (!E_ENABLE_ON) WRITE(E4_ENABLE_PIN, HIGH);
  #endif
  #if HAS_E5_ENABLE
    SET_OUTPUT(E5_ENABLE_PIN);
    if (!E_ENABLE_ON) WRITE(E5_ENABLE_PIN, HIGH);
  #endif

  SET_OUTPUT(E0_STEP_PIN);
}

/**
 * HeatedBedSelfCheck:Check if the heatedbed is OK
 * return : true if OK, or else false
 */
bool PrintExecuter::HeatedBedSelfCheck(void) {
  millis_t tmptick;

  enable_power_domain(POWER_DOMAIN_BED);
  // disable heated bed firstly
  OUT_WRITE(HEATER_BED_PIN, LOW);
  // and set input for the detect pin
  SET_INPUT_PULLUP(HEATEDBED_ON_PIN);
  tmptick = millis() + 10;
  while(tmptick > millis());
  // if we get LOW, indicate the NMOS is breakdown
  // we need to disable its power supply immediately
  if(READ(HEATEDBED_ON_PIN) == LOW) {
    disable_power_domain(POWER_DOMAIN_BED);
    enable_power_ban(POWER_DOMAIN_BED);
    SystemStatus.ThrowException(EHOST_MC, ETYPE_PORT_BAD);
  }
}

/**
 * HeatedBedSelfCheck:Check if the heatedbed is OK
 * return : true if OK, or else false
 */
bool PrintExecuter::SetPID(uint8_t Index, float Value) {
  uint32_t value_multiple1000;
  uint8_t buff[8];
  buff[0] = Index;
  value_multiple1000 = (uint32_t)(Value * 1000.0f);
  buff[1] = (uint8_t)(value_multiple1000 >> 24);
  buff[2] = (uint8_t)(value_multiple1000 >> 16);
  buff[3] = (uint8_t)(value_multiple1000 >> 8);
  buff[4] = (uint8_t)(value_multiple1000);
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_PID, buff, 5);
}

#if ENABLED(EXECUTER_CANBUS_SUPPORT)

/**
 * Init
 */
void PrintExecuter::Init()
{
  EStepperInit();
}

#else

/**
 * Init
 */
void PrintExecuter::Init()
{
  EStepperInit();

  #if HAS_HEATER_0
    OUT_WRITE(HEATER_0_PIN, HEATER_0_INVERTING);
  #endif
  #if HAS_HEATER_1
    OUT_WRITE(HEATER_1_PIN, HEATER_1_INVERTING);
  #endif
  #if HAS_HEATER_2
    OUT_WRITE(HEATER_2_PIN, HEATER_2_INVERTING);
  #endif
  #if HAS_HEATER_3
    OUT_WRITE(HEATER_3_PIN, HEATER_3_INVERTING);
  #endif
  #if HAS_HEATER_4
    OUT_WRITE(HEATER_4_PIN, HEATER_4_INVERTING);
  #endif
  #if HAS_HEATER_5
    OUT_WRITE(HEATER_5_PIN, HEATER_5_INVERTING);
  #endif
  #if HAS_FAN0
    INIT_FAN_PIN(FAN_PIN);
  #endif
  #if HAS_FAN1
    INIT_FAN_PIN(FAN1_PIN);
  #endif
  #if HAS_FAN2
    INIT_FAN_PIN(FAN2_PIN);
  #endif
  #if ENABLED(USE_CONTROLLER_FAN)
    INIT_FAN_PIN(CONTROLLER_FAN_PIN);
  #endif
}

#endif // ENABLED EXECUTER_CANBUS_SUPPORT