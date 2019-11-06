#include "../gcode/gcode.h"
#include "../gcode/queue.h"
#include "../core/macros.h"
#include "../module/PeriphDevice.h"


/*
* Disable /Enable chamber door event
* S0: disable door event
* S1: enable door event
* S2: release hold of UART for receiving other PC gcodes
*/


void GcodeSuite::M1120() {
  uint8_t s;

  if (!parser.seen('S')) {
    SERIAL_ECHO("Chamber Door event is ");
    SERIAL_ECHOLN(Periph.GetDoorCheckFlag()? "Enabled" : "Disabled");
    Periph.ReportStatus();
    return;
  }

  s = parser.byteval('S', 2);

  switch (s)
  {
  case 0:
    Periph.SetDoorCheck(false);
    break;

  case 1:
    Periph.SetDoorCheck(true);
    break;

  case 2:
    Periph.SetUartLock(false);
    break;

  case 10:
    Periph.TriggerDoorEvent(true);
    SERIAL_ECHOLN("triggered door open");
    break;

  case 11:
    Periph.TriggerDoorEvent(false);
    SERIAL_ECHOLN("triggered door close");
    break;

  default:
    break;
  }
}