#include "../common/config.h"

#include MARLIN_SRC(inc/MarlinConfig.h)
#include MARLIN_SRC(gcode/gcode.h)
#include MARLIN_HAL(HAL_watchdog_STM32F1.h)


void GcodeSuite::M1999() {
  SERIAL_ECHOLN("will reboot machine");
  WatchDogInit();
}
