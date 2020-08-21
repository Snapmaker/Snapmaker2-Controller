#include "../gcode/gcode.h"
#include "../module/ExecuterManager.h"
#include "../module/CanModule.h"
#include "../module/motion.h"
#include "../snap_module/upgrade_service.h"
#include "../libs/hex_print_routines.h"

void GcodeSuite::M1005() {
  char buffer[VERSION_STRING_SIZE];
  int i;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_CHAR(' ');
  SERIAL_ECHOLNPGM(SHORT_BUILD_VERSION);
  SERIAL_ECHO_MSG("Compiled: " __DATE__ ", " __TIME__);
  
    // screen show version
  char Version[33] = {0};
  memcpy(Version, (char*)(FLASH_BOOT_PARA + 2048), 30);
  SERIAL_ECHOPGM(MSG_MARLIN_PACK);
  SERIAL_CHAR(' ');
  SERIAL_ECHOPGM(Version);
  SERIAL_EOL();

  SERIAL_ECHOLN("Module Ver:");
  for (int i = 0; i < CanBusControlor.ModuleCount; i++) {
    if (CanModules.GetFirmwareVersion(BASIC_CAN_NUM, CanBusControlor.ModuleMacList[i], (char *)buffer)) {
      SERIAL_ECHO("0x");
      print_hex_word(CanBusControlor.ModuleMacList[i]>>16);
      print_hex_word(CanBusControlor.ModuleMacList[i]);
      SERIAL_ECHOLNPAIR(": ", buffer);
    }
  }

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
}


void GcodeSuite::M1006() {
  SERIAL_ECHO("Tool Head: ");
  switch (ExecuterHead.MachineType) {
  case MACHINE_TYPE_3DPRINT:
    SERIAL_ECHOLN("3DP");
    break;

  case  MACHINE_TYPE_LASER:
    SERIAL_ECHOLN("LASER");
    SERIAL_ECHO("Current Status: ");
    SERIAL_ECHOLN(ExecuterHead.Laser.GetTimPwm()? "ON" : "OFF");
    SERIAL_ECHOLNPAIR("Current Power: ", ExecuterHead.Laser.GetPowerPercent());
    SERIAL_ECHOLNPAIR("Focus Height: ", ExecuterHead.Laser.FocusHeight);
    break;

  case  MACHINE_TYPE_CNC:
    SERIAL_ECHOLN("CNC");
    SERIAL_ECHOLNPAIR("Current Power: ", ExecuterHead.CNC.GetPower());
    SERIAL_ECHOLNPAIR("RPM: ", ExecuterHead.CNC.GetRPM());
    break;

  default:
    SERIAL_ECHOLN("UNKNOWN");
    break;
  }
}


void GcodeSuite::M1007() {
  SERIAL_ECHO("Homed: ");
  SERIAL_ECHOLN(all_axes_homed()? "YES" : "NO");

  SERIAL_ECHOLNPAIR("Selected origin num: ", active_coordinate_system + 1);

  SERIAL_ECHO("Selected == Current: ");

  if (active_coordinate_system < 0) {
    SERIAL_ECHOLN("YES");
  }
  else if ((position_shift[X_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][X_AXIS]) &&
        (position_shift[Y_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][Y_AXIS]) &&
        (position_shift[Z_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][Z_AXIS])) {
    SERIAL_ECHOLN("YES");
  }
  else {
    SERIAL_ECHOLN("NO");
  }

  if (active_coordinate_system < 0) {
    SERIAL_ECHOLNPAIR("Origin offset X: ", position_shift[X_AXIS]);
    SERIAL_ECHOLNPAIR("Origin offset Y: ", position_shift[Y_AXIS]);
    SERIAL_ECHOLNPAIR("Origin offset Z: ", position_shift[Z_AXIS]);
  }
  else {
    SERIAL_ECHOLNPAIR("Origin offset X: ", gcode.coordinate_system[gcode.active_coordinate_system][X_AXIS]);
    SERIAL_ECHOLNPAIR("Origin offset Y: ", gcode.coordinate_system[gcode.active_coordinate_system][Y_AXIS]);
    SERIAL_ECHOLNPAIR("Origin offset Z: ", gcode.coordinate_system[gcode.active_coordinate_system][Z_AXIS]);
  }
}
