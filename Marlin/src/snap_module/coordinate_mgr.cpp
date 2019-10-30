#include "coordinate_mgr.h"

#include "snap_cmd.h"
#include "snap_dbg.h"
#include "../gcode/gcode.h"
#include "../SnapScreen/HMISC20.h"
#include "../module/motion.h"

#if ENABLED(CNC_COORDINATE_SYSTEMS)

extern HMI_SC20 SC20HMI;

#define BITS32_TO_BYTES(u32bit, buff, index) do { \
    buff[index++] = (uint8_t)(u32bit >> 24); \
    buff[index++] = (uint8_t)(u32bit >> 16); \
    buff[index++] = (uint8_t)(u32bit >> 8); \
    buff[index++] = (uint8_t)(u32bit); \
    }while(0)

void CoordinateMgrReportStatus(uint8_t eventid, uint8_t opcode) {
  uint8_t buff[20] = {0};
  uint8_t i = 0;
  int32_t pos_shift[XYZ];

  buff[i++] = eventid + 1;
  buff[i++] = opcode;

  if (all_axes_homed()) {
    buff[i++] = 0;
  }
  else {
    buff[i++] = 1;
  }

  if (gcode.active_coordinate_system < 0) {
    // number
    buff[i++] = 0;
    // state
    buff[i++] = 0;
    pos_shift[X_AXIS] = (int32_t)(position_shift[X_AXIS] * 1000);
    pos_shift[Y_AXIS] = (int32_t)(position_shift[Y_AXIS] * 1000);
    pos_shift[Z_AXIS] = (int32_t)(position_shift[Z_AXIS] * 1000);
  }
  else {
    buff[i++] = gcode.active_coordinate_system + 1;
    // check state
    if ((position_shift[X_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][X_AXIS]) &&
        (position_shift[Y_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][Y_AXIS]) &&
        (position_shift[Z_AXIS] == gcode.coordinate_system[gcode.active_coordinate_system][Z_AXIS])) {
      buff[i++] = 0;
    }
    else {
      buff[i++] = 1;
    }
    pos_shift[X_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][X_AXIS] * 1000);
    pos_shift[Y_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][Y_AXIS] * 1000);
    pos_shift[Z_AXIS] = (int32_t)(gcode.coordinate_system[gcode.active_coordinate_system][Z_AXIS] * 1000);
  }

  BITS32_TO_BYTES(pos_shift[X_AXIS], buff, i);
  BITS32_TO_BYTES(pos_shift[Y_AXIS], buff, i);
  BITS32_TO_BYTES(pos_shift[Z_AXIS], buff, i);

  SC20HMI.PackedProtocal((char *)buff, i);
}


ErrCode CoordinateMgrReportData(uint8_t start, uint8_t tot) {
  if (!tot) {
    LOG_E("must specified how many coordinates data you wanted!");
    return E_PARAM;
  }

  uint8_t buff[13 * MAX_COORDINATE_SYSTEMS + 3];
  uint8_t i;

  for (i = 0; i < MAX_COORDINATE_SYSTEMS; i++, start++) {
    buff[i * 13 + 3] = start;
    memcpy(&buff[i * 13 + 4], gcode.coordinate_system[start], 12);
    if (--tot == 0)
      break;
  }

  buff[0] = EID_STATUS_RESP;
  buff[1] = 0xF;
  buff[2] = i;

  SC20HMI.PackedProtocal((char *)buff, 13 * i + 3);

  return E_SUCCESS;
}

#endif
