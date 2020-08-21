#ifndef SNAPMAKER_H_
#define SNAPMAKER_H_

#include "MapleFreeRTOS1030.h"
#include "common/config.h"
#include "common/error.h"

#include "hmi/event_handler.h"

struct SnapTasks {
  TaskHandle_t marlin;
  TaskHandle_t hmi;
  TaskHandle_t heartbeat;
  MessageBufferHandle_t event_queue;
};
typedef struct SnapTasks* SnapTasks_t;

extern SnapTasks_t snap_tasks;

#define HMI_NOTIFY_WAITFOR_HEATING 0X00000001

#define MARLIN_LOOP_TASK_PRIO (configMAX_PRIORITIES - 1)
#define MARLIN_LOOP_STACK_DEPTH 1024

#define HMI_TASK_PRIO (configMAX_PRIORITIES - 1)
#define HMI_TASK_STACK_DEPTH 512

// task parameters for heartbeat task
#define HB_TASK_PRIO (configMAX_PRIORITIES - 1)
#define HB_TASK_STACK_DEPTH 512

#define EXECUTOR_SERIAL_IRQ_PRIORITY 7
#define HMI_SERIAL_IRQ_PRIORITY 8
#define MARLIN_SERIAL_IRQ_PRIORITY 9

ErrCode SnapmakerSetupEarly();

ErrCode SnapmakerSetupPost();



#endif  // #ifndef SNAPMAKER_H_
