#ifndef COORDINATE_MGR_H_
#define COORDINATE_MGR_H_

#include "error.h"

void CoordinateMgrReportStatus(uint8_t eventid, uint8_t opcode);
ErrCode CoordinateMgrReportData(uint8_t start, uint8_t tot);

#endif
