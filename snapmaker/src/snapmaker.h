/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
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
#include "module/module_base.h"

struct SnapmakerHandle {
  TaskHandle_t marlin;
  TaskHandle_t hmi;
  TaskHandle_t heartbeat;
  TaskHandle_t can_recv;
  TaskHandle_t can_event;

  MessageBufferHandle_t event_queue;
  EventGroupHandle_t    event_group;
};
typedef struct SnapmakerHandle* SnapmakerHandle_t;

extern SnapmakerHandle_t sm2_handle;


#define EVENT_GROUP_MODULE_READY      (0x00000001)
#define EVENT_GROUP_WAIT_FOR_HEATING  (0X00000002)


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
