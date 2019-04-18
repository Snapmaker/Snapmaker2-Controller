#include "../inc/MarlinConfig.h"

#if ENABLED(EXECUTER_MANAGER_SUPPORT)


#include "../Marlin.h"
#include "../module/temperature.h"
#include "../module/configuration_store.h"
#include "ExecuterManager.h"

ExecuterManager ExecuterHead;


#define CAN_ID_BC   0x500
#define CAN_ID_SWITCH   0x501
#define CAN_ID_TEMP   0x510
#define CAN_ID_FAN    0x511
#define CAN_ID_DCMOTOR  0x512

/**
 * Executer Init
 */
void ExecuterManager::Init()
{
  #if ENABLED(EXECUTER_CANBUS)
    #if (EXECUTER_CAN_PORT == 1)
      CanInit(1);
    #elif (EXECUTER_CAN_PORT == 2)
      CanInit(2);
    #endif
  #endif
}

/**
 * Executer Detect
 * return   true:Raack have received, false:No reack.result will be saved in member "MachineType"
 */
bool ExecuterManager::Detecte()
{
  #if ENABLED(EXECUTER_CANBUS)
    MachineType = GetMachineTypeFromCAN();
  #endif

  #if ENABLED(EXECUTER_TEMPERATURE)
    MachineType = GetMachineTypeFromTemperature();
  #endif
  return (MachineType != MACHINE_TYPE_UNDEFINE);
}


#if ENABLED(EXECUTER_CANBUS)
  /**
   * Get machine type from Can Bus
   * return   One of MACHINE_TYPE_CNC,MACHINE_TYPE_LASER,MACHINE_TYPE_3DPRINT
   */
  uint8_t ExecuterManager::GetMachineTypeFromCAN(void)
  {
    uint8_t type;
    uitn8_t retry;
    millis_t tmpTick;
    type = MACHINE_TYPE_UNDEFINE;
    uint8_t Buff[8] = "SEEK";
    retry = 5;
    while(type == MACHINE_TYPE_UNDEFINE) {
      if((millis() - tmpTick) > 500L) {
        tmpTick = millis();
        if(retry-- == 0)
          return MACHINE_TYPE_UNDEFINE;
        if(CanSendMessage(CAN_ID_BC, 1, FRAME_DATA, 8, Buff) == false)
          return MACHINE_TYPE_UNDEFINE;
      }
    }
    return type;
  }
#endif // ENABLED(EXECUTER_CANBUS)


#if ENABLED(EXECUTER_TEMPERATURE)

  /**
   * Get machine type from Temerature
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
#endif //ENABLED(EXECUTER_TEMPERATURE)

extern "C"
{
#if (EXECUTER_CAN_PORT == 1)
void __irq_can1_tx(void)
{
}

void __irq_can1_rx0(void)
{
}

void __irq_can1_rx1(void)
{
}

void __irq_can1_sce(void)
{
}

#elif (EXECUTER_CAN_PORT == 2)
void __irq_can2_tx(void)
{
}

void __irq_can2_rx0(void)
{
}

void __irq_can2_rx1(void)
{
}

void __irq_can2_sce(void)
{
}
#endif
}

#endif // ENABLED EXECUTER_CANBUS
