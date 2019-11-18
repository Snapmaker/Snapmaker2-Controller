
#include "snap_dbg.h"
#include "../gcode/gcode.h"
#include "../gcode/queue.h"
#include "../core/macros.h"
#include "../module/StatusControl.h"

#if HAS_POSITION_SHIFT
  // The distance that XYZ has been offset by G92. Reset by G28.
  extern float position_shift[XYZ];
#endif
#if HAS_HOME_OFFSET
  // This offset is added to the configured home position.
  // Set by M206, M428, or menu item. Saved to EEPROM.
  extern float home_offset[XYZ];
#endif
#if HAS_HOME_OFFSET && HAS_POSITION_SHIFT
  // The above two are combined to save on computes
  extern float workspace_offset[XYZ];
#endif

extern float current_position[XYZE];

void GcodeSuite::M2000() {
  uint8_t l;
  uint8_t s = (uint8_t)parser.byteval('S', (uint8_t)0);

  switch (s) {
  case 0:
    // show current snapmaker info
    SNAP_DEBUG_SHOW_INFO();
    LOG_I("position_shift:\n");
    LOG_I("X: %f, Y:%f, Z:%f\n", position_shift[X_AXIS], position_shift[Y_AXIS], position_shift[Z_AXIS]);
    LOG_I("home_offset:\n");
    LOG_I("X: %f, Y:%f, Z:%f\n", home_offset[X_AXIS], home_offset[Y_AXIS], home_offset[Z_AXIS]);
    LOG_I("workspace_offset:\n");
    LOG_I("X: %f, Y:%f, Z:%f\n", workspace_offset[X_AXIS], workspace_offset[Y_AXIS], workspace_offset[Z_AXIS]);
    LOG_I("cur position:\n");
    LOG_I("X: %f, Y:%f, Z:%f\n", current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
    break;

  case 1:
    // set PC log level
    l = (uint8_t)parser.byteval('L', (uint8_t)10);
    if (!WITHIN(l, 0, (int)SNAP_DEBUG_LEVEL_MAX)) {
      LOG_E("L out of range (0-%d)\n", (int)SNAP_DEBUG_LEVEL_MAX);
      return;
    }
    SNAP_DEBUG_SET_LEVEL(0, (SnapDebugLevel)l);
    break;

  case 2:
    // set SC log level
    l = (uint8_t)parser.byteval('L', (uint8_t)10);
    if (!WITHIN(l, 0, (int)SNAP_DEBUG_LEVEL_MAX)) {
      LOG_E("L out of range (0-%d)\n", (int)SNAP_DEBUG_LEVEL_MAX);
      return;
    }
    SNAP_DEBUG_SET_LEVEL(1, (SnapDebugLevel)l);
    break;

  case 3:
    SNAP_DEBUG_SHOW_EXCEPTION();
    break;

  case 4:
    l = (uint8_t)parser.byteval('L', (uint8_t)0);
    if (!WITHIN(l, 1, 32)) {
      LOG_E("L is out of range (1-32)\n");
      return;
    }
    SystemStatus.ClearExceptionByFaultFlag(1<<(l-1));
    break;
  }

}