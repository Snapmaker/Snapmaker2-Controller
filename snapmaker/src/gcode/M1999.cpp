#include "../common/config.h"

#include "src/inc/MarlinConfig.h"
#include "src/gcode/gcode.h"
#include HAL_PATH(src/HAL, HAL_watchdog_STM32F1.h)


void GcodeSuite::M1999() {
  SERIAL_ECHOLN("will reboot machine");
  WatchDogInit();
}
