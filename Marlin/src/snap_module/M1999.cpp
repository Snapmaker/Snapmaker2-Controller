#include "../inc/MarlinConfig.h"
#include HAL_PATH(../HAL, HAL_watchdog_STM32F1.h)
#include "../gcode/gcode.h"

void GcodeSuite::M1999() {
  SERIAL_ECHOLN("will reboot machine");
  WatchDogInit();
}
