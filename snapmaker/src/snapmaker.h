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


#define ACTION_BAN_NONE               (0)
#define ACTION_BAN_NO_WORKING         (0x1)
#define ACTION_BAN_NO_MOVING          (0x1<<1)
#define ACTION_BAN_NO_HEATING_BED     (0x1<<2)
#define ACTION_BAN_NO_HEATING_HOTEND  (0x1<<3)

extern uint8_t action_ban;
void enable_action_ban(uint8_t ab);
void disable_action_ban(uint8_t ab);

#define POWER_DOMAIN_NONE     (0)
#define POWER_DOMAIN_0        (0x01)       /* just for screen */
#define POWER_DOMAIN_1        (0x01<<1)    /* for all executors and all linear modules */
#define POWER_DOMAIN_2        (0x01<<2)    /* for bed and addon */
#define POWER_DOMAIN_ALL      0xFF

#define POWER_DOMAIN_SCREEN   POWER_DOMAIN_0
#define POWER_DOMAIN_LINEAR   POWER_DOMAIN_1
#define POWER_DOMAIN_EXECUTOR POWER_DOMAIN_1
#define POWER_DOMAIN_BED      POWER_DOMAIN_2
#define POWER_DOMAIN_ADDON    POWER_DOMAIN_2
#define POWER_DOMAIN_HOTEND   POWER_DOMAIN_1

extern uint8_t power_ban;
void enable_power_ban(uint8_t pd);
void disable_power_ban(uint8_t pd);
void enable_power_domain(uint8_t pd);
void disable_power_domain(uint8_t pd);

void SnapmakerSetupEarly();

void SnapmakerSetupPost();



#endif  // #ifndef SNAPMAKER_H_
