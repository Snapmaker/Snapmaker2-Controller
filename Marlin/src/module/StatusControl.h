#pragma once

#include "../inc/MarlinConfig.h"
#include "../snap_module/error.h"

#ifndef _STATUS_CONTROL_H_
#define _STATUS_CONTROL_H_

#define FAULT_FLAG_NO_EXECUTOR        1       // lost executor
#define FAULT_FLAG_NO_LINEAR          (1<<1)  // no any linear module detect
#define FAULT_FLAG_BED_PORT           (1<<2)  // NMOS for controlling bed is breakdown
#define FAULT_FLAG_FILAMENT	          (1<<3)
#define FAULT_FLAG_LOST_SETTING	      (1<<4)
#define FAULT_FLAG_LOST_EXECUTOR      (1<<5)
#define FAULT_FLAG_POWER_LOSS	        (1<<6)
#define FAULT_FLAG_HOTEND_HEATFAIL    (1<<7)  /* invalid power panic data */
#define FAULT_FLAG_BED_HEATFAIL       (1<<8)  /* EEPROM settings lost*/
#define FAULT_FLAG_HOTEND_RUNWAWY     (1<<9) // temperature of hotend is runaway
#define FAULT_FLAG_BED_RUNAWAY        (1<<10) // temperature of bed is runaway
#define FAULT_FLAG_HOTEND_SENSOR_BAD  (1<<11) // temperature sensor of hotend is abnormal
#define FAULT_FLAG_BED_SENSOR_BAD     (1<<12) // temperature sensor of bed is abnormal
#define FAULT_FLAG_LOST_LINEAR        (1<<13)
#define FAULT_FLAG_HOTEND_MAXTEMP     (1<<14)
#define FAULT_FLAG_BED_MAXTEMP        (1<<15)
#define FAULT_FLAG_HOTEND_SHORTCIRCUIT    (1<<16)
#define FAULT_FLAG_BED_SHORTCIRCUIT       (1<<17)
#define FAULT_FLAG_HOTEND_SENSOR_COMEOFF  (1<<18)
#define FAULT_FLAG_BED_SENSOR_COMEOFF     (1<<19)
#define FAULT_FLAG_UNKNOW_MODEL       (1<<20)
#define FAULT_FLAG_DOOR_OPENED        (1<<21)
#define FAULT_FLAG_UNKNOW             (1<<31)

// this macro mask the bits which are allow to be cleared by screen
#define FAULT_FLAG_SC_CLEAR_MASK      (FAULT_FLAG_POWER_LOSS | FAULT_FLAG_FILAMENT | FAULT_FLAG_LOST_SETTING \
                                      | FAULT_FLAG_HOTEND_HEATFAIL | FAULT_FLAG_BED_HEATFAIL | FAULT_FLAG_HOTEND_RUNWAWY \
                                      | FAULT_FLAG_BED_RUNAWAY | FAULT_FLAG_HOTEND_MAXTEMP | FAULT_FLAG_BED_MAXTEMP \
                                      | FAULT_FLAG_UNKNOW | FAULT_FLAG_DOOR_OPENED)

// exception actions
#define EACTION_NONE                0
#define EACTION_PAUSE_WORKING       0x1
#define EACTION_STOP_WORKING        (0x1<<1)
#define EACTION_STOP_HEATING_BED    (0x1<<2)
#define EACTION_STOP_HEATING_HOTEND (0x1<<3)

#define EXCEPTION_ISR_BUFFSER_SIZE  4

enum ExceptionHost : int8_t {
  EHOST_CHAMBER = -2,  /* -2 ~ 6 just are to match Thermal Manage of Marlin */
  EHOST_BED = -1,
  EHOST_HOTEND0 = 0,
  EHOST_HOTEND1 = 1,
  EHOST_HOTEND2 = 2,
  EHOST_EXECUTOR = 7,  /* all executors, 0-6 is reserved for extruders */
  EHOST_LINEAR,        /* linear modules */
  EHOST_MC,            /* main control */

  EHOST_INVALID
};

enum ExceptionType : uint8_t {
  ETYPE_NO_HOST = 0,
  ETYPE_LOST_HOST,
  ETYPE_RUNOUT,
  ETYPE_LOST_CFG,
  ETYPE_PORT_BAD,
  ETYPE_POWER_LOSS,
  ETYPE_HEAT_FAIL,
  ETYPE_TEMP_RUNAWAY,
  ETYPE_TEMP_REDUNDANCY,
  ETYPE_SENSOR_BAD,
  ETYPE_BELOW_MINTEMP,
  ETYPE_OVERRUN_MAXTEMP,
  ETYPE_OVERRUN_MAXTEMP_AGAIN,
  ETYPE_ABRUPT_TEMP_DROP,
  ETYPE_SENSOR_COME_OFF,

  ETYPE_INVALID
};


enum TriggerSource : uint8_t {
  TRIGGER_SOURCE_NONE,
  TRIGGER_SOURCE_SC,              // trigger by screen
  TRIGGER_SOURCE_PC,              // trigger by PC
  TRIGGER_SOURCE_RUNOUT,          // trigger by filament runout
  TRIGGER_SOURCE_DOOR_OPEN,       // trigger by door opened
  TRIGGER_SOURCE_DOOR_CLOSE,      // trigger by door closed
  TRIGGER_SOURCE_STOP_BUTTON,     // trigger by emergency button
  TRIGGER_SOURCE_FINISH,          // trigger by job finished
  TRIGGER_SOURCE_EXCEPTION,       // trigger by exception

  TRIGGER_SOURCE_INVALID
};

// status of system
enum SysStatus: uint8_t {
  SYSTAT_INIT = 0,

  SYSTAT_IDLE,            // generally, we are in this status after initialization

  SYSTAT_WORK,            // working

  SYSTAT_PAUSE_TRIG,      // pause is triggered by some events
  SYSTAT_PAUSE_STOPPED,   // quick stop is handled
  SYSTAT_PAUSE_FINISH,    // the continuous status for pause

  SYSTAT_RESUME_TRIG,     // resume op is triggered
  SYSTAT_RESUME_MOVING,   // moving to the position when pause
  SYSTAT_RESUME_WAITING,  // waiting the Gcode

  SYSTAT_END_TRIG,
  SYSTAT_END_FINISH,
  SYSTAT_INVALID
};

// stage of system, some one just focus on stage
// but some one focus on detail status
enum SysStage : uint8_t {
  SYSTAGE_INIT,
  SYSTAGE_IDLE,
  SYSTAGE_WORK,
  SYSTAGE_PAUSE,
  SYSTAGE_RESUMING,
  SYSTAGE_END,
  SYSTAGE_INVALID
};

enum WorkingPort : uint8_t {
  WORKING_PORT_NONE,  // no work started from any port
  WORKING_PORT_PC,    // started a work from PC port
  WORKING_PORT_SC,    // started a work from Screen port
  WORKING_PORT_INVALID
};

enum RuntimeEnvType : uint8_t {
  RENV_TYPE_FEEDRATE,
  RENV_TYPE_HOTEND_TEMP,
  RENV_TYPE_BED_TEMP,
  RENV_TYPE_LASER_POWER,
  RENV_TYPE_ZOFFSET,

  RENV_TYPE_INVALID
};

class StatusControl
{
public:
  StatusControl(){};
  void Init();
  uint32_t GetSystemFault();
  uint8_t GetPeriphDeviceStatus();

  void ClearSystemFaultBit(uint32_t BitsToClear);
  void SetSystemFaultBit(uint32_t BitsToSet);

  void CheckException();
  ErrCode ThrowException(ExceptionHost h, ExceptionType t);
  ErrCode ThrowExceptionISR(ExceptionHost h, ExceptionType t);
  ErrCode ClearException(ExceptionHost h, ExceptionType t);
  ErrCode ClearExceptionByFaultFlag(uint32_t flag);

  ErrCode PauseTrigger(TriggerSource type);
  ErrCode StopTrigger(TriggerSource type);
  ErrCode ResumeTrigger(TriggerSource s);
  ErrCode ResumeOver();
  ErrCode StartWork(TriggerSource s);
  void Process();

  uint8_t MapCurrentStatusForSC();

  SysStatus FORCE_INLINE GetCurrentStatus() {
    return cur_status_;
  }

  ErrCode FORCE_INLINE SetCurrentStatus(SysStatus s) {
    if (s < SYSTAT_INVALID) {
      cur_status_ = s;
      return E_SUCCESS;
    }

    return E_PARAM;
  }

  WorkingPort FORCE_INLINE GetWorkingPort() { return work_port_;  }
  ErrCode FORCE_INLINE SetWorkingPort(WorkingPort p) {
    if (p < WORKING_PORT_INVALID) {
      work_port_ = p;
      return E_SUCCESS;
    }

    return E_PARAM;
  }

  SysStage GetCurrentStage();
  uint32_t GetFaultFlag() { return fault_flag_; }

  void CallbackOpenDoor();
  void CallbackCloseDoor();

  ErrCode ChangeRuntimeEnv(uint8_t param_type, float param);

private:
  void inline resume_3dp(void);
  void inline resume_cnc(void);
  void inline resume_laser(void);
  void inline RestoreXYZ(void);
  void PauseProcess();
  void StopProcess();
  void ResumeProcess();
  ErrCode PauseResume();
  void MapFaultFlagToException(uint32_t flag, ExceptionHost &host, ExceptionType &type);

public:
  uint32_t PeriphDeviceStatus;

private:
  uint8_t TriggleStat;
  TriggerSource pause_source_;  // record latest pause source
  TriggerSource stop_source_;
  uint32_t fault_flag_;

  int8_t isr_exception[EXCEPTION_ISR_BUFFSER_SIZE][2];
  uint8_t isr_e_rindex_;
  uint8_t isr_e_windex_;
  uint8_t isr_e_len_;

  SysStatus cur_status_;
  WorkingPort work_port_;    // indicates we are handling Gcode from which UART
};

extern StatusControl SystemStatus;

#endif //def _STATUS_CONTROL_H_
