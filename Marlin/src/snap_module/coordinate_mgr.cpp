#include "coordinate_mgr.h"

#include "snap_cmd.h"
#include "snap_dbg.h"
#include "../gcode/gcode.h"
#include "../SnapScreen/HMISC20.h"

extern HMI_SC20 SC20HMI;

ErrCode CoordinateMgrReport(uint8_t start, uint8_t tot) {
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


