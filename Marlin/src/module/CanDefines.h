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
  CMD_T_CONFIG = 0,
  CMD_R_CONFIG_REACK,
  CMD_T_REQUEST_FUNCID,
  CMD_R_REPORT_FUNCID,
  CMD_T_CONFIG_FUNCID,
  CMD_R_CONFIG_FUNCID_REACK,
  CMD_T_UPDATE_REQUEST, //6
  CMD_R_UPDATE_REQUEST_REACK,
  CMD_T_UPDATE_PACKDATA,
  CMD_R_UPDATE_PACK_REQUEST,
  CMD_T_UPDATE_END,
};

// FuncID defines
typedef enum {
  FUNC_REPORT_LIMIT          ,
  FUNC_REPORT_PROBE          ,
  FUNC_REPORT_CUT            ,
  FUNC_SET_STEP_CTRL         ,
  FUNC_SET_MOTOR_SPEED       ,
  FUNC_REPORT_MOTOR_SPEED    ,
  FUNC_REPORT_TEMPEARTURE    ,
  FUNC_SET_TEMPEARTURE       ,
  FUNC_SET_FAN               ,
  FUNC_SET_FAN2              ,
  FUNC_SET_PID               ,
  FUNC_SET_CAMERA_POWER      ,
  FUNC_SET_LASER_FOCUS       ,
  FUNC_REPORT_LASER_FOCUS    ,    
  FUNC_SET_LIGHT_COLOR       ,
  FUNC_REPORT_ENCLOSUER      ,
}FUNC_ID_E;


//Priority Table for attributing the priority of the FuncID
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
  {FUNC_REPORT_ENCLOSUER,         15},
};

#endif
#endif //ENABLE CANBUS_SUPPORT
