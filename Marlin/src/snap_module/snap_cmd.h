#ifndef SNAP_CMD_H_
#define SNAP_CMD_H_

#include <stdio.h>

// event IDs
// gcode from PC
#define EID_GCODE_REQ         1
#define EID_GCODE_RESP        2
// gcode from file
#define EID_FILE_GCODE_REQ    3
#define EID_FILE_GCODE_RESP   4
// file operation
#define EID_FILE_OP_REQ       5
#define EID_FILE_OP_RESP      6
// status query
#define EID_STATUS_REQ        7
#define EID_STATUS_RESP       8
// settings operation
#define EID_SETTING_REQ       9
#define EID_SETTING_RESP      0xa
// movement command
#define EID_MOVEMENT_REQ      0xb
#define EID_MOVEMENT_RESP     0xc
// laser&camera opration
#define EID_LAS_CAM_OP_REQ    0xd
#define EID_LAS_CAM_OP_RESP   0xe
// upgrade command
#define EID_UPGRADE_REQ       0xa9
#define EID_UPGRADE_RESP      0xaa
// Add-on operation
#define EID_ADDON_OP_REQ      0x11
#define EID_ADDON_OP_RESP     0x12
// laser calibration
#define EID_LASER_CALIBRATE_REQ   0x13
#define EID_LASER_CALIBRATE_RESP  0x14


// index of each field in command header
#define IDX_DATA_LEN  2
#define IDX_VERSION   4
#define IDX_LEN_CHK   5
#define IDX_CHKSUM    6
#define IDX_EVENT_ID  8
#define IDX_OP_CODE   9
#define IDX_DATA0     10

// common commands for add-ons
#define CMD_ADDON_CHK_ONLINE        0

// commands for light bar, LB = light bar
#define CMD_LB_QUERY_STATE          1
#define CMD_LB_SWITCH               2
#define CMD_LB_SET_MODE_BRIGHTNESS  3
// commands for enclosure fan
#define CMD_ENCLOSURE_FAN_SWITCH    4

#endif
