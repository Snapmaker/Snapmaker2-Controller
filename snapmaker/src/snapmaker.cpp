#include "snapmaker.h"

#include "common/debug.h"
#include "hmi/event_handler.h"
#include "module/can_host.h"
#include "service/system.h"
#include "service/upgrade.h"
#include "service/power_loss_recovery.h"

// marlin headers
#include "src/module/endstops.h"
#include "src/feature/runout.h"

#include "flash_stm32.h"


extern void enqueue_hmi_to_marlin();

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
  SnapTasks_t task_param;
  struct DispatcherParam dispather_param;

  millis_t cur_mills;

  configASSERT(param);
  task_param = (SnapTasks_t)param;

  dispather_param.owner = TASK_OWN_MARLIN;

  dispather_param.event_buff = (uint8_t *)pvPortMalloc(SSTP_RECV_BUFFER_SIZE);
  configASSERT(dispather_param.event_buff);

  dispather_param.event_queue = task_param->event_queue;

  systemservice.SetCurrentStatus(SYSTAT_IDLE);

  SERIAL_ECHOLN("Finish init\n");

  cur_mills = millis() - 3000;

  for (;;) {

    #if(0)
    #if ENABLED(SDSUPPORT)
      card.checkautostart();

      if (card.flag.abort_sd_printing) {
        card.stopSDPrint(
          #if SD_RESORT
            true
          #endif
        );
        clear_command_queue();
        quickstop_stepper();
        print_job_timer.stop();
        thermalManager.disable_all_heaters();
        thermalManager.zero_fan_speeds();
        wait_for_heatup = false;
        #if ENABLED(POWER_LOSS_RECOVERY)
          card.removeJobRecoveryFile();
        #endif
      }
    #endif // SDSUPPORT
    #endif

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
  SnapTasks_t    task_param;
  struct DispatcherParam dispather_param;

  ErrCode ret = E_FAILURE;

  uint8_t count = 0;

  configASSERT(param);
  task_param = (SnapTasks_t)param;

  dispather_param.owner = TASK_OWN_HMI;

  dispather_param.event_buff = (uint8_t *)pvPortMalloc(SSTP_RECV_BUFFER_SIZE);
  configASSERT(dispather_param.event_buff);

  dispather_param.event_queue = task_param->event_queue;

  for (;;) {
    if(READ(SCREEN_DET_PIN)) {
      xTaskNotifyStateClear(task_param->heartbeat);

      if (systemservice.GetCurrentStatus() == SYSTAT_WORK && count == 100) {
        // if we lost screen in working for 10s, stop current work
        systemservice.StopTrigger(TRIGGER_SOURCE_SC_LOST);
        LOG_E("stop cur work because screen lost!\n");
      }

      if (++count > 100)
        count = 0;

      vTaskDelay(portTICK_PERIOD_MS * 100);
      continue;
    }
    else
      count = 0;

    ret = hmi.CheckoutCmd(dispather_param.event_buff, dispather_param.size);
    if (ret != E_SUCCESS) {
      // no command, sleep 10ms for next command
      vTaskDelay(portTICK_PERIOD_MS * 10);
      continue;
    }

    // execute or send out one command
    DispatchEvent(&dispather_param);

    vTaskDelay(portTICK_PERIOD_MS * 5);
  }
}


static void heartbeat_task(void *param) {
  uint32_t  notificaiton = 0;
  SSTP_Event_t   event = {EID_SYS_CTRL_ACK, SYSCTL_OPC_GET_STATUES};

  int counter = 0;

  for (;;) {
    // do following every 10ms without being blocked
    #if HAS_FILAMENT_SENSOR
      runout.run();
    #endif

    systemservice.CheckException();

    for (int i = 0; static_modules[i] != NULL; i++)
      static_modules[i]->Process();

    if (++counter > 100) {
      counter = 0;

      // do following every 1s
      upgrade.Check();

      // comment temporarily
      // notificaiton = ulTaskNotifyTake(false, 0);
      // if (notificaiton && upgrade.GetState() != UPGRADE_STA_UPGRADING_EM)
      //   systemservice.SendStatus(event);
    }

    // sleep for 10ms
    vTaskDelay(portTICK_PERIOD_MS * 10);
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


void SnapmakerSetupPost() {
  millis_t ms = millis();

  LOG_E("ms: %u", ms);
  // init the power supply pins
  OUT_WRITE(POWER0_SUPPLY_PIN, POWER0_SUPPLY_ON);
  OUT_WRITE(POWER1_SUPPLY_PIN, POWER1_SUPPLY_ON);
  OUT_WRITE(POWER2_SUPPLY_PIN, POWER2_SUPPLY_ON);

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

  // init power panic handler and load data from flash
  pl_recovery.Init();

  canhost.Init();

  snap_tasks = (SnapTasks_t)pvPortMalloc(sizeof(struct SnapTasks));
  snap_tasks->event_queue = xMessageBufferCreate(1024);

  // create marlin task
  BaseType_t ret;
  ret = xTaskCreate((TaskFunction_t)main_loop, "Marlin_task", MARLIN_LOOP_STACK_DEPTH,
        (void *)snap_tasks, MARLIN_LOOP_TASK_PRIO, &snap_tasks->marlin);
  if (ret != pdPASS) {
    LOG_E("failt to create marlin task!\n");
    while(1);
  }
  else {
    LOG_I("success to create marlin task!\n");
  }

  ret = xTaskCreate((TaskFunction_t)hmi_task, "HMI_task", HMI_TASK_STACK_DEPTH,
        (void *)snap_tasks, HMI_TASK_PRIO, &snap_tasks->hmi);
  if (ret != pdPASS) {
    LOG_E("failt to create HMI task!\n");
    while(1);
  }
  else {
    LOG_I("success to create HMI task!\n");
  }


  ret = xTaskCreate((TaskFunction_t)heartbeat_task, "HB_task", HB_TASK_STACK_DEPTH,
        (void *)snap_tasks, HB_TASK_STACK_DEPTH, &snap_tasks->heartbeat);
  if (ret != pdPASS) {
    LOG_E("failt to create heartbeat task!\n");
    while(1);
  }
  else {
    LOG_I("success to create heartbeat task!\n");
  }

  vTaskStartScheduler();
}


void vApplicationMallocFailedHook( void ) {
  LOG_E("RTOS malloc failed");
}



