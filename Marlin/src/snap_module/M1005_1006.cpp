#include "../gcode/gcode.h"
#include "../module/ExecuterManager.h"
#include "../module/CanModule.h"

void GcodeSuite::M1005() {
  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_CHAR(' ');
  SERIAL_ECHOLNPGM(SHORT_BUILD_VERSION);
  SERIAL_ECHO_MSG("Compiled: " __DATE__ ", " __TIME__);
  
  SERIAL_ECHO("Machine Size: ");
  switch (CanModules.GetMachineSizeType()) {
  case MACHINE_SIZE_S:
    SERIAL_ECHOLN("S");
    break;

  case MACHINE_SIZE_M:
    SERIAL_ECHOLN("M");
    break;

  case MACHINE_SIZE_L:
    SERIAL_ECHOLN("L");
    break;

  default:
    SERIAL_ECHOLN("U");
    break;
  }
  SERIAL_EOL();
}


void GcodeSuite::M1006() {
  SERIAL_ECHO("Tool Head: ");
  switch (ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    SERIAL_ECHOLN("3DP");
    break;

  case  MACHINE_TYPE_LASER:
    SERIAL_ECHOLN("LASER");
    break;

  case  MACHINE_TYPE_CNC:
    SERIAL_ECHOLN("CNC");
    break;

  default:
    SERIAL_ECHOLN("UNKNOWN");
    break;
  }
}
