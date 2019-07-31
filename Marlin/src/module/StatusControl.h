#pragma once

#include "../inc/MarlinConfig.h"
#include "../snap_module/error.h"

#ifndef _STATUS_CONTROL_H_
#define _STATUS_CONTROL_H_

#define FAULT_FLAG_HEATER0	          1
#define FAULT_FLAG_BED		            (1<<1)
#define FAULT_FLAG_LOAD		            (1<<2)
#define FAULT_FLAG_FILAMENT	          (1<<3)
#define FAULT_FLAG_HEATFAIL	          (1<<4)
#define FAULT_FLAG_FILAMENT_SENSOR	  (1<<5)
#define FAULT_FLAG_POWERPANIC	        (1<<6)
#define FAULT_FLAG_LASER_EEPROM	      (1<<7)
#define FAULT_FLAG_INVALID_PPD        (1<<8)    /* invalid power panic data */
#define FAULT_FLAG_SETTING            (1<<9)    /* EEPROM settings lost*/

// source for stopping
enum StopSource : uint8_t {
	STOP_SOURCE_SC = 0,   // stopped by Screen port
  STOP_SOURCE_PC,       // stopped by PC port
	STOP_SOURCE_FINISH,   // finish printing
  STOP_SOURCE_BUTTON,   // stop by quickstop button
	STOP_SOURCE_INVALID  
};

// source for pause
enum PauseSource : uint8_t {
  PAUSE_SOURCE_SC,            // paused by screen port
  PAUSE_SOURCE_PC,            // paused by PC port
  PAUSE_SOURCE_RUNOUT,        // paused by filament runout
  PAUSE_SOURCE_DOOR_OPEN,     // paused by door opened
  PAUSE_SOURCE_LOST_EXECUTOR, // paused by losting executor
  PAUSE_SOURCE_INVALID
};

enum ResumeSource : uint8_t {
  RESUME_SOURCE_PC,         // resume by PC port
  RESUME_SOURCE_SC,         // resume by screen port
  RESUME_SOURCE_DOOR_CLOSE, // resume door closed
  RESUME_SOURCE_INVALID
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

class StatusControl
{
public:
  StatusControl(){};
  void Init();
  uint32_t GetSystemFault();
  void CheckFatalError();
  void SetCurrentPrinterStatus(uint8_t newstatus);
  uint8_t GetPeriphDeviceStatus();

  void ClearSystemFaultBit(uint32_t BitsToClear);
  void SetSystemFaultBit(uint32_t BitsToSet);
  ErrCode PauseTrigger(PauseSource type);
  PauseSource GetPauseSource() { return pause_source_; }
  ErrCode StopTrigger(StopSource type);
  ErrCode ResumeTrigger(ResumeSource s);
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

  SysStage GetCurrentStage();

private:
  void InterruptAllCommand();
  void inline resume_3dp(void);
  void inline resume_cnc(void);
  void inline resume_laser(void);
  void PauseProcess();
  void StopProcess();
  void ResumeProcess();
  ErrCode PauseResume();

public:
  uint32_t PeriphDeviceStatus;

private:
  uint8_t TriggleStat;
  PauseSource pause_source_;  // record latest pause source
  StopSource stop_type_;
  uint32_t fault_flag_;

  SysStatus cur_status_;
  WorkingPort work_port_;    // indicates we are handling Gcode from which UART
};

extern StatusControl SystemStatus;

#endif //def _STATUS_CONTROL_H_
