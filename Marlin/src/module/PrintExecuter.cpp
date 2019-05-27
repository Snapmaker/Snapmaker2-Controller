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
void PrintExecuter::Init()
{
  
}

#else

/**
 * Init
 */
void PrintExecuter::Init()
{
  #if DISABLED(EXECUTER_CANBUS_SUPPORT)
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

    OUT_WRITE(CNC_PIN, false);
  #endif // DISABLED(EXECUTER_CANBUS_SUPPORT)
}

#endif // ENABLED EXECUTER_CANBUS_SUPPORT