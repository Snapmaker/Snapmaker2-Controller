#include "../common/config.h"
#include "../common/debug.h"

#include "../service/upgrade.h"
#include "../module/can_host.h"
#include "../module/module_base.h"
#include "../module/linear.h"
#include "../module/toolhead_3dp.h"
#include "../module/toolhead_cnc.h"
#include "../module/toolhead_laser.h"

// marlin headers
#include "src/gcode/gcode.h"
#include "src/module/motion.h"
#include "src/libs/hex_print_routines.h"

void GcodeSuite::M1005() {
  CanExtCmd_t cmd;
  MAC_t       mac;

  char buffer[VERSION_STRING_SIZE + 4];
  int  i;

  // version in code
  LOG_I("%s: %s\n", MSG_MARLIN, SHORT_BUILD_VERSION);
  LOG_I("Compiled: %s, %s\n", __DATE__, __TIME__);

  // version in package
  memcpy(buffer, (char*)(FLASH_BOOT_PARA + 2048), 30);
  LOG_I("%s: %s\n", MSG_MARLIN_PACK, buffer);

  // version of modules
  LOG_I("Module Ver:\n");
  cmd.data = (uint8_t *)buffer;
  for (i = 0; i < MODULE_SUPPORT_CONNECTED_MAX; i++) {
    mac.val = canhost.mac(i);

    if (mac.val == MODULE_MAC_ID_INVALID)
      break;

    cmd.mac     = mac;
    cmd.data[0] = MODULE_EXT_CMD_VERSION_REQ;
    cmd.length  = 1;

    if (canhost.SendExtCmdSync(cmd, 500) != E_SUCCESS) {
      continue;
    }

    buffer[cmd.length + 2] = 0;
    LOG_I("0x%08X: %s\n", mac.val, buffer+2);
  }


  SERIAL_ECHO("Machine Size: ");
  switch (linear.machine_size()) {
  case MACHINE_SIZE_A150:
    SERIAL_ECHOLN("S");
    break;

  case MACHINE_SIZE_A250:
    SERIAL_ECHOLN("M");
    break;

  case MACHINE_SIZE_A350:
    SERIAL_ECHOLN("L");
    break;

  default:
    SERIAL_ECHOLN("U");
    break;
  }
}


void GcodeSuite::M1006() {
  int type = 0;
  SERIAL_ECHO("Tool Head: ");
  if (printer.IsOnline())
    type = 1;
  if (laser.IsOnline())
    type = 2;
  if (cnc.IsOnline())
    type = 3;

  switch (type) {
  case 1:
    SERIAL_ECHOLN("3DP");
    break;

  case  2:
    SERIAL_ECHOLN("LASER");
    SERIAL_ECHO("Current Status: ");
    SERIAL_ECHOLN((laser.state() == TOOLHEAD_LASER_STATE_ON)? "ON" : "OFF");
    SERIAL_ECHOLNPAIR("Current Power: ", laser.power());
    SERIAL_ECHOLNPAIR("Focus Height: ", laser.focus());
    break;

  case  3:
    SERIAL_ECHOLN("CNC");
    SERIAL_ECHOLNPAIR("Current Power: ", cnc.power());
    SERIAL_ECHOLNPAIR("RPM: ", cnc.rpm());
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
