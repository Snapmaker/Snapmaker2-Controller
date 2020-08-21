
#include "../common/config.h"

#include "../module/enclosure.h"

#include MARLIN_SRC(gcode/gcode.h)
#include MARLIN_SRC(gcode/queue.h)
#include MARLIN_SRC(core/macros.h)

/*
* Disable /Enable chamber door event
* S0: disable door event
* S1: enable door event
* S2: release hold of UART for receiving other PC gcodes
* S3: set light power, P 0-100%, eg.M1010 S3 P100
* S4: set fan power, P 0-100%, eg.M1010 S4 P100
*/


void GcodeSuite::M1010() {
  uint8_t s, p;

  if (!parser.seen('S')) {
    enclosure.ReportStatus();
    return;
  }

  // if chamber is not exist, not allow other operation
  if (!enclosure.IsOnline()) {
    SERIAL_ECHOLN("Enclosure is not online.");
    return;
  }

  s = parser.byteval('S', 12);
  p = parser.byteval('P', 0);
  switch (s)
  {
  case 0:
    enclosure.Disable();
    break;

  case 1:
    enclosure.Enable();
    break;

  case 2:
    ModuleBase::UnlockMarlinUart();
    break;

  case 3:
    enclosure.SetLightBar(p);
    break;

  case 4:
    enclosure.SetFanSpeed(p);
    break;

  case 10:
    enclosure.door_state(ENCLOSURE_DOOR_STATE_OPEN);
    SERIAL_ECHOLN("triggered door open");
    break;

  case 11:
    enclosure.door_state(ENCLOSURE_DOOR_STATE_CLOSED);
    SERIAL_ECHOLN("triggered door close");
    break;

  case 12:
    enclosure.PollDoorState();
    break;

  default:
    break;
  }
}
