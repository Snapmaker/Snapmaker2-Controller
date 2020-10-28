#ifndef SNAPMAKER_H_
#define SNAPMAKER_H_

#include "MapleFreeRTOS1030.h"
#include "common/config.h"
#include "common/error.h"
#include "common//debug.h"
#include "gcode/M1028.h"
#include "hmi/event_handler.h"
#include "service/bed_level.h"
#include "service/quick_stop.h"
#include "service/power_loss_recovery.h"

struct SnapTasks {
  TaskHandle_t marlin;
  TaskHandle_t hmi;
  TaskHandle_t heartbeat;
  MessageBufferHandle_t event_queue;
};
typedef struct SnapTasks* SnapTasks_t;

extern SnapTasks_t snap_tasks;

#define HMI_NOTIFY_WAITFOR_HEATING 0X00000001

void SnapmakerSetupEarly();

void SnapmakerSetupPost();



#endif  // #ifndef SNAPMAKER_H_
