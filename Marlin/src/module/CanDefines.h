#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(CANBUS_SUPPORT)
#ifndef _CANDEFINES_H_
#define _CANDEFINES_H_

typedef struct
{
  uint32_t ID;
  uint8_t IDType;
  uint8_t FrameType;
  uint8_t Data[8];
}strCanData;

//Can prepare
enum
{
  CMD_M_CONFIG = 0,
  CMD_S_CONFIG_REACK,
  CMD_M_REQUEST_FUNCID,
  CMD_S_REPORT_FUNCID,
  CMD_M_CONFIG_FUNCID,
  CMD_S_CONFIG_FUNCID_REACK,
  CMD_M_UPDATE_REQUEST, //6
  CMD_S_UPDATE_REQUEST_REACK,
  CMD_M_UPDATE_PACKDATA,
  CMD_S_UPDATE_PACK_REQUEST,
  CMD_M_UPDATE_END,
  CMD_M_VERSIONS_REQUEST,
  CMD_S_VERSIONS_REACK,
  CMD_M_SET_RANDOM,  // 设置随机数
  CMD_S_SET_RANDOM_REACK,
  CMD_M_SET_LINEAR_LEN,  // 设置模组长度
  CMD_S_SET_LINEAR_LEN_REACK,
  CMD_M_SET_LINEAR_LEAD,  // 设置导程
  CMD_S_SET_LINEAR_LEAD_REACK,
  CMD_M_SET_LINEAR_LIMIT,  // 设置限位开关位置
  CMD_S_SET_LINEAR_LIMIT_REACK,
  CMD_M_UPDATE_STATUS_REQUEST,
  CMD_S_UPDATE_STATUS_REACK,
  CMD_M_UPDATE_START,
};

// FuncID defines
typedef enum {
  FUNC_REPORT_LIMIT          ,  // 0
  FUNC_REPORT_PROBE          ,  // 1
  FUNC_REPORT_CUT            ,  // 2
  FUNC_SET_STEP_CTRL         ,  // 3
  FUNC_SET_MOTOR_SPEED       ,  // 4
  FUNC_REPORT_MOTOR_SPEED    ,  // 5
  FUNC_REPORT_TEMPEARTURE    ,  // 6
  FUNC_SET_TEMPEARTURE       ,  // 7
  FUNC_SET_FAN               ,  // 8
  FUNC_SET_FAN2              ,  // 9
  FUNC_SET_PID               ,  // 10
  FUNC_SET_CAMERA_POWER      ,  // 11
  FUNC_SET_LASER_FOCUS       ,  // 12
  FUNC_REPORT_LASER_FOCUS    ,  // 13
  FUNC_SET_LIGHT_COLOR       ,  // 14
  FUNC_REPORT_ENCLOSURE      ,  // 15
  FUNC_REPORT_TEMP_PID       ,  // 16
  FUNC_PROOFREAD_KNIFE       ,  // 17
  FUNC_SET_ENCLOSURE_LIGHT   ,  // 18
  FUNC_SET_FAN_MODULE        ,  // 19
}FUNC_ID_E;


const uint16_t PriorityTable[][2] = {
// FUNCID                         Priority(0-15)
  {FUNC_REPORT_LIMIT,             0},
  {FUNC_REPORT_CUT,               1},
  {FUNC_REPORT_PROBE,             1},
  {FUNC_REPORT_TEMPEARTURE,       2},
  {FUNC_REPORT_MOTOR_SPEED,       2},
  {FUNC_REPORT_CUT,               2},
  {FUNC_SET_TEMPEARTURE,          15},
  {FUNC_SET_MOTOR_SPEED,          15},
  {FUNC_SET_PID,                  15},
  {FUNC_SET_FAN,                  0},
  {FUNC_SET_FAN2,                 15},
  {FUNC_SET_CAMERA_POWER,         0},
  {FUNC_SET_LASER_FOCUS,          10},
  {FUNC_REPORT_LASER_FOCUS,       7},
  {FUNC_SET_LIGHT_COLOR,          15},
  {FUNC_REPORT_ENCLOSURE,         15},
  {FUNC_SET_FAN_MODULE,           15},
  {FUNC_SET_ENCLOSURE_LIGHT,      15},
};


enum PeriphSwitchEnum : uint32_t {
  CAN_IO_ENCLOSURE = 0,
};

#define MODULE_MASK_BITS  0x1ff00000
#define MODULE_EXECUTER_PRINT   0
#define MODULE_EXECUTER_CNC     1
#define MODULE_EXECUTER_LASER   2
#define MODULE_LINEAR           3
#define MODULE_LIGHT            4
#define MODULE_ENCLOSER         5
#define MODULE_ROTATE           6
#define MODULE_AIRCONDITIONER   7

#define MAKE_ID(MID)  ((MID << 20) & MODULE_MASK_BITS)

#endif
#endif //ENABLE CANBUS_SUPPORT
