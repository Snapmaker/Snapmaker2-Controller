#ifndef SNAPMAKER_CONFIG_H_
#define SNAPMAKER_CONFIG_H_

// task parameters for can event handler task
#define CAN_EVENT_HANDLER_PRIORITY    (2)
#define CAN_EVENT_HANDLER_STACK_DEPTH (512)

// task parameters for can receive handler task
#define CAN_RECEIVE_HANDLER_PRIORITY    (2)
#define CAN_RECEIVE_HANDLER_STACK_DEPTH (512)

// task parameters for marlin loop task
#define MARLIN_LOOP_TASK_PRIO (3)
#define MARLIN_LOOP_STACK_DEPTH 1024

// task parameters for hmi task
#define HMI_TASK_PRIO (3)
#define HMI_TASK_STACK_DEPTH 512

// task parameters for heartbeat task
#define HB_TASK_PRIO (2)
#define HB_TASK_STACK_DEPTH 512

// priority for UARTs
#define EXECUTOR_SERIAL_IRQ_PRIORITY 7
#define HMI_SERIAL_IRQ_PRIORITY 8
#define MARLIN_SERIAL_IRQ_PRIORITY 9

#endif  // #ifndef SNAPMAKER_CONFIG_H_
