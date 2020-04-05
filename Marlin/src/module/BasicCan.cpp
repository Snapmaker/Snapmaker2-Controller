#include "../inc/MarlinConfig.h"

#if ENABLED(CANBUS_SUPPORT)

#include "../../HAL/HAL_GD32F1/HAL_can_STM32F1.h"
#include "../Marlin.h"
#include "temperature.h"
#include "configuration_store.h"
#include "ExecuterManager.h"
#include "PeriphDevice.h"
#include "CanModule.h"
#include "BasicCan.h"
#include <EEPROM.h>
#include "../libs/GenerialFunctions.h"

BasicCan BasicCanPort;

/**
 *Init:Initialize
 */
void BasicCan::Init(void) {
  
}

void BasicCan::SetFunctionValue(uint16_t FuncID, uint8_t *pBuff, uint8_t Len) {
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FuncID, pBuff, Len);
}

void BasicCan::SetFunctionValue(uint16_t FuncID, uint16_t u16Value) {
  uint8_t pBuff[8];
  UInt16ToBytes(u16Value, pBuff);
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FuncID, pBuff, 2);
}


#endif // ENABLED CANBUS_SUPPORT
