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
#ifndef SNAPMAKER_CONFIG_H_
#define SNAPMAKER_CONFIG_H_

// task parameters for can event handler task
#define CAN_EVENT_HANDLER_PRIORITY    (3)
#define CAN_EVENT_HANDLER_STACK_DEPTH (512)

// task parameters for can receive handler task
#define CAN_RECEIVE_HANDLER_PRIORITY    (3)
#define CAN_RECEIVE_HANDLER_STACK_DEPTH (512)

// task parameters for marlin loop task
#define MARLIN_LOOP_TASK_PRIO (3)
#define MARLIN_LOOP_STACK_DEPTH 1024

// task parameters for hmi task
#define HMI_TASK_PRIO (3)
#define HMI_TASK_STACK_DEPTH 512

// task parameters for heartbeat task
#define HB_TASK_PRIO (3)
#define HB_TASK_STACK_DEPTH 512

// priority for UARTs
#define EXECUTOR_SERIAL_IRQ_PRIORITY 7
#define HMI_SERIAL_IRQ_PRIORITY 8
#define MARLIN_SERIAL_IRQ_PRIORITY 9

#endif  // #ifndef SNAPMAKER_CONFIG_H_
