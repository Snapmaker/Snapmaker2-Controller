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
#include "snapmaker.h"

#include "common/debug.h"
#include "hmi/event_handler.h"
#include "module/can_host.h"
#include "module/linear.h"
#include "service/system.h"
#include "service/upgrade.h"
#include "service/power_loss_recovery.h"

// marlin headers
#include "src/module/endstops.h"
#include "src/feature/runout.h"
#include "flash_stm32.h"
#include "hmi/gcode_result_handler.h"

SnapmakerHandle_t sm2_handle;

extern void enqueue_hmi_to_marlin();


uint8_t action_ban = 0;
void enable_action_ban(uint8_t ab) {
  action_ban |= ab;
}

void disable_action_ban(uint8_t ab) {
  action_ban &= (~ab);
}

// default all power domain is available
uint8_t power_ban = 0;
void enable_power_ban(uint8_t pd) {
  power_ban |= pd;
}

void disable_power_ban(uint8_t pd) {
  power_ban &= (~pd);
}

void enable_power_domain(uint8_t pd) {
  pd &= ~power_ban;
  #if PIN_EXISTS(POWER0_SUPPLY)
    if (pd & POWER_DOMAIN_0) WRITE(POWER0_SUPPLY_PIN, POWER0_SUPPLY_ON);
  #endif

  #if PIN_EXISTS(POWER1_SUPPLY)
    if (pd & POWER_DOMAIN_1) WRITE(POWER1_SUPPLY_PIN, POWER1_SUPPLY_ON);
  #endif

  #if PIN_EXISTS(POWER2_SUPPLY)
    if (pd & POWER_DOMAIN_2) WRITE(POWER2_SUPPLY_PIN, POWER2_SUPPLY_ON);
  #endif
}

void disable_power_domain(uint8_t pd) {
  #if PIN_EXISTS(POWER0_SUPPLY)
    if (pd & POWER_DOMAIN_0) WRITE(POWER0_SUPPLY_PIN, POWER0_SUPPLY_OFF);
  #endif

  #if PIN_EXISTS(POWER1_SUPPLY)
    if (pd & POWER_DOMAIN_1) WRITE(POWER1_SUPPLY_PIN, POWER1_SUPPLY_OFF);
  #endif

  #if PIN_EXISTS(POWER2_SUPPLY)
    if (pd & POWER_DOMAIN_2) WRITE(POWER2_SUPPLY_PIN, POWER2_SUPPLY_OFF);
  #endif
}


void HeatedBedSelfCheck(void) {
  enable_power_domain(POWER_DOMAIN_BED);
  // disable heated bed firstly
  OUT_WRITE(HEATER_BED_PIN, LOW);
  // and set input for the detect pin
  SET_INPUT_PULLUP(HEATEDBED_ON_PIN);
  vTaskDelay(pdMS_TO_TICKS(10));
  // if we get LOW, indicate the NMOS is breakdown
  // we need to disable its power supply immediately
  if(READ(HEATEDBED_ON_PIN) == LOW) {
    disable_power_domain(POWER_DOMAIN_BED);
    enable_power_ban(POWER_DOMAIN_BED);
    systemservice.ThrowException(EHOST_MC, ETYPE_PORT_BAD);
  }
}


/**
 * Check Update Flag
 */
void CheckUpdateFlag(void)
{
  uint32_t Address;
  uint32_t Flag;
  Address = FLASH_UPDATE_CONTENT_INFO;
  Flag = *((uint32_t*)Address);
  if(Flag != 0xffffffff)
  {
    FLASH_Unlock();
    FLASH_ErasePage(Address);
    FLASH_Lock();
  }
}


/**
 * main task
 */
static void main_loop(void *param) {
  struct DispatcherParam dispather_param;

  millis_t cur_mills;

  dispather_param.owner = TASK_OWN_MARLIN;

  dispather_param.event_buff = (uint8_t *)pvPortMalloc(SSTP_RECV_BUFFER_SIZE);
  configASSERT(dispather_param.event_buff);

  dispather_param.event_queue = ((SnapmakerHandle_t)param)->event_queue;

  HeatedBedSelfCheck();

  systemservice.SetCurrentStatus(SYSTAT_IDLE);

  // waiting for initializing modules
  xEventGroupWaitBits(((SnapmakerHandle_t)param)->event_group, EVENT_GROUP_MODULE_READY, pdFALSE, pdTRUE, pdMS_TO_TICKS(10000));

  // init power-loss recovery after initializing modules
  // because we need to check if current toolhead is same with previous
  pl_recovery.Init();

  SERIAL_ECHOLN("Finish init\n");

  cur_mills = millis() - 3000;

  for (;;) {

    // receive and execute one command, or push Gcode into Marlin queue
    DispatchEvent(&dispather_param);

    enqueue_hmi_to_marlin();
    if (commands_in_queue < BUFSIZE) get_available_commands();

    advance_command_queue();
    quickstop.Process();
    endstops.event_handler();
    idle();

    // avoid module proactive reply failure, loop query
    // case 1: unexpected faliment runout trigger if we startup withou toolhead loaded.
    // case 2: Z axis hit boundary when we run G28.
    // case 3: Z_MIN_Probe error, when we do z probe, the triggered message didn't arrive main controller

    if (cur_mills + 2500 <  millis()) {
      cur_mills = millis();
      // TODO: poll filament sensor state
    }
  }
}


static void hmi_task(void *param) {
  SnapmakerHandle_t    task_param;
  struct DispatcherParam dispather_param;

  ErrCode ret = E_FAILURE;

  uint8_t  count = 0;

  configASSERT(param);
  task_param = (SnapmakerHandle_t)param;

  dispather_param.owner = TASK_OWN_HMI;

  dispather_param.event_buff = (uint8_t *)pvPortMalloc(SSTP_RECV_BUFFER_SIZE);
  configASSERT(dispather_param.event_buff);

  dispather_param.event_queue = task_param->event_queue;

  for (;;) {
    if(READ(SCREEN_DET_PIN)) {
      xTaskNotifyStateClear(task_param->heartbeat);

      if (systemservice.GetCurrentStatus() == SYSTAT_WORK && count) {
        // if we lost screen in working for 100ms, stop current work
        systemservice.StopTrigger(TRIGGER_SOURCE_SC_LOST);
        LOG_E("stop cur work because screen lost!\n");
      }

      if (++count)
        count = 0;

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    else
      count = 0;

    ret = hmi.CheckoutCmd(dispather_param.event_buff, dispather_param.size);

    systemservice.CheckIfSendWaitEvent();

    if (ret != E_SUCCESS) {
      // no command, sleep 10ms for next command
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // execute or send out one command
    DispatchEvent(&dispather_param);

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}


static void heartbeat_task(void *param) {
  //SSTP_Event_t   event = {EID_SYS_CTRL_ACK, SYSCTL_OPC_GET_STATUES};

  int counter = 0;

  for (;;) {
    // do following every 10ms without being blocked
    #if HAS_FILAMENT_SENSOR
      runout.run();
    #endif

    systemservice.CheckException();

    if (++counter > 100) {
      counter = 0;

      // do following every 1s
      upgrade.Check();
      canhost.SendHeartbeat();
    }

    // sleep for 10ms
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void SnapmakerSetupEarly() {

  systemservice.Init();
  // init serial for HMI
  hmi.Init(&MSerial2, HMI_SERIAL_IRQ_PRIORITY);
}


/**
 * Check App Valid Flag
 */
void CheckAppValidFlag(void)
{
  uint32_t Value;
  uint32_t Address;
  Address = FLASH_BOOT_PARA;
  Value = *((uint32_t*)Address);
  if(Value != 0xaa55ee11) {
    FLASH_Unlock();
    FLASH_ErasePage(Address);
    FLASH_ProgramWord(Address, 0xaa55ee11);
    FLASH_Lock();
  }
}


static void TaskEventHandler(void *p) {
  canhost.EventHandler(p);
}


static void TaskReceiveHandler(void *p) {
  canhost.ReceiveHandler(p);
}


void SnapmakerSetupPost() {
  // init the power supply pins
  OUT_WRITE(POWER0_SUPPLY_PIN, POWER0_SUPPLY_ON);
  OUT_WRITE(POWER1_SUPPLY_PIN, POWER1_SUPPLY_OFF);
  OUT_WRITE(POWER2_SUPPLY_PIN, POWER2_SUPPLY_OFF);

  SET_INPUT_PULLUP(SCREEN_DET_PIN);

  if(READ(SCREEN_DET_PIN)) {
    disable_power_domain(POWER_DOMAIN_SCREEN);
    SERIAL_ECHOLN("Screen doesn't exist!\n");
  }
  else {
    enable_power_domain(POWER_DOMAIN_SCREEN);
    SERIAL_ECHOLN("Screen exists!\n");
  }

  // power on the modules by default
  enable_all_steppers();

  BreathLightInit();

  CheckAppValidFlag();

  CheckUpdateFlag();

  // to disable heartbeat if module need to be upgraded
  upgrade.CheckIfUpgradeModule();

  canhost.Init();

  enable_power_domain(POWER_DOMAIN_LINEAR);
  enable_power_domain(POWER_DOMAIN_ADDON);

  sm2_handle = (SnapmakerHandle_t)pvPortMalloc(sizeof(struct SnapmakerHandle));
  sm2_handle->event_queue = xMessageBufferCreate(1024);
  configASSERT(sm2_handle->event_queue);

  sm2_handle->event_group = xEventGroupCreate();
  configASSERT(sm2_handle->event_group);

  // create marlin task
  BaseType_t ret;
  ret = xTaskCreate((TaskFunction_t)main_loop, "Marlin_task", MARLIN_LOOP_STACK_DEPTH,
        (void *)sm2_handle, MARLIN_LOOP_TASK_PRIO, &sm2_handle->marlin);
  if (ret != pdPASS) {
    LOG_E("Failed to create marlin task!\n");
    while(1);
  }
  else {
    LOG_I("Created marlin task!\n");
  }

  ret = xTaskCreate((TaskFunction_t)hmi_task, "HMI_task", HMI_TASK_STACK_DEPTH,
        (void *)sm2_handle, HMI_TASK_PRIO, &sm2_handle->hmi);
  if (ret != pdPASS) {
    LOG_E("Failed to create HMI task!\n");
    while(1);
  }
  else {
    LOG_I("Created HMI task!\n");
  }


  ret = xTaskCreate((TaskFunction_t)heartbeat_task, "HB_task", HB_TASK_STACK_DEPTH,
        (void *)sm2_handle, HB_TASK_PRIO, &sm2_handle->heartbeat);
  if (ret != pdPASS) {
    LOG_E("Failed to create heartbeat task!\n");
    while(1);
  }
  else {
    LOG_I("Created heartbeat task!\n");
  }

  ret = xTaskCreate((TaskFunction_t)TaskReceiveHandler, "can_recv_handler", CAN_RECEIVE_HANDLER_STACK_DEPTH,
        (void *)sm2_handle, CAN_RECEIVE_HANDLER_PRIORITY,  &sm2_handle->can_recv);
  if (ret != pdPASS) {
    LOG_E("Failed to create receiver task!\n");
    while(1);
  }
  else {
    LOG_I("Created can receiver task!\n");
  }

  ret = xTaskCreate((TaskFunction_t)TaskEventHandler, "can_event_handler", CAN_EVENT_HANDLER_STACK_DEPTH,
        (void *)sm2_handle, CAN_EVENT_HANDLER_PRIORITY, &sm2_handle->can_event);
  if (ret != pdPASS) {
    LOG_E("Failed to create can event task!\n");
    while(1);
  }
  else {
    LOG_I("Created can event task!\n");
  }

  vTaskStartScheduler();
}


extern "C" {

void vApplicationMallocFailedHook( void ) {
  LOG_E("RTOS malloc failed");
}

}
