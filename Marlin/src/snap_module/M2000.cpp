
#include "snap_dbg.h"
#include "../gcode/gcode.h"
#include "../gcode/queue.h"
#include "../core/macros.h"

void GcodeSuite::M2000() {
  uint8_t l;
  uint8_t s = (uint8_t)parser.byteval('S', (uint8_t)10);

  if (!WITHIN(s, 0, 1)) {
    SERIAL_ECHOLNPGM("S out of range (0-1).");
    return;
  }

  switch (s) {
  case 0:
    // show current snapmaker info
    ShowDebugInfo();
    break;

  case 1:
    // set debug level
    l = (uint8_t)parser.byteval('L', (uint8_t)10);
    if (!WITHIN(l, 0, (int)SNAP_DEBUG_LEVEL_MAX)) {
      SnapDbg(SNAP_ERROR, "L out of range (0-%d)\n", (int)SNAP_DEBUG_LEVEL_MAX);
      return;
    }
    SetDbgLevel((SnapDbgLevel)l);
    break;
  }

}