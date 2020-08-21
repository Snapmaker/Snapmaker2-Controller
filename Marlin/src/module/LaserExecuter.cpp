#include "../inc/MarlinConfig.h"

#include "../Marlin.h"
#include "temperature.h"
#include "configuration_store.h"
#include "motion.h"
#include "planner.h"
#include "LaserExecuter.h"
#include "CanBus.h"
#include "CanDefines.h"
#include "ExecuterManager.h"
#include "StatusControl.h"

#include "../snap_module/snap_dbg.h"
#include "../snap_module/M1028.h"

// time to delay close fan, 120s
#define TIME_TO_CLOSE_FAN 120

static const uint16_t LaserPowerTable[]=
{
  0,
  20,22,24,26,28,30,31,33,35,37,39,41,43,45,47,49,51,53,54,56,58,60,63,65,67,69,71,73,75,77,79,82,84,86,88,90,93,95,97,
  100,102,103,106,109,111,113,116,119,121,123,125,128,130,133,135,138,140,143,145,148,150,153,156,158,161,164,166,169,
  171,174,177,179,182,185,187,190,192,196,198,200,203,205,208,210,211,214,217,218,221,224,226,228,231,234,236,240,242,
  247,251,255
};

#define LASERSerial MSerial3

#define TimSetPwm(n)  Tim1SetCCR4(n)

/**
 * Init
 */
void LaserExecuter::Init()
{
  Tim1PwmInit();
  LoadFocusHeight();

  serial_.Init(&LASERSerial, EXECUTOR_SERIAL_IRQ_PRIORITY);

  power_limit_ = LASER_POWER_NORMAL_LIMIT;

  fan_state_ = LAESR_FAN_STA_CLOSED;
  fan_tick_ = 0;
}

/**
 * check if FAN is open, if not, will open it
 */
void LaserExecuter::CheckFan(uint16_t p) {
  uint8_t data[2];

  switch (fan_state_) {
  case LAESR_FAN_STA_OPEN:
    if (p == 0) {
      fan_state_ = LAESR_FAN_STA_TO_BE_CLOSED;
      fan_tick_ = 0;
    }
    break;

  case LAESR_FAN_STA_TO_BE_CLOSED:
    if (p > 0) {
      fan_state_ = LAESR_FAN_STA_OPEN;
    }
    break;

  case LAESR_FAN_STA_CLOSED:
    if (p > 0) {
      data[0] = 0;
      data[1] = 255;
      CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_FAN, data, 2);
      fan_state_ = LAESR_FAN_STA_OPEN;
    }
    break;
  }
}

/**
 * try to close fan if needed
 */
void LaserExecuter::TryCloseFan() {
  uint8_t data[2];

  if (ExecuterHead.MachineType != MACHINE_TYPE_LASER)
    return;

  if (fan_state_ == LAESR_FAN_STA_TO_BE_CLOSED) {
    if (++fan_tick_ > TIME_TO_CLOSE_FAN) {
      data[0] = 0;
      data[1] = 0;
      CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_FAN, data, 2);
      fan_state_ = LAESR_FAN_STA_CLOSED;
      fan_tick_ = 0;
    }
  }
}

void LaserExecuter::SetLaserPwm(uint16_t pwmvalue) {
  CheckFan(pwmvalue);
  TimSetPwm(pwmvalue);
}

/**
 * SetLaserLowPower:Set laser power
 * para percent:
 */
void LaserExecuter::SetLaserPower(float Percent) {
  SysStatus cur_stat = SystemStatus.GetCurrentStatus();

  if (cur_stat == SYSTAT_PAUSE_TRIG || cur_stat == SYSTAT_END_TRIG)
    return;

  ChangePower(Percent);
  SetLaserPwm(last_pwm);
}

/**
 * Off:Laser off without changing the power
 */
void LaserExecuter::Off()
{
  SetLaserPwm(0);
}

/**
 * On:Laser on and use the last power
 */
void LaserExecuter::On() {
  SysStatus cur_stat = SystemStatus.GetCurrentStatus();

  if (cur_stat == SYSTAT_PAUSE_TRIG || cur_stat == SYSTAT_END_TRIG) {
    LOG_W("cannot open laser during pause/stop triggered!\n");
    return;
  }
  SetLaserPower(last_percent);
}

/**
 * SetLaserLowPower:Set laser power
 * para percent:
 */
void LaserExecuter::SetLaserLowPower()
{
  SetLaserPower(0.5f);
}

#if ENABLED(EXECUTER_CANBUS_SUPPORT)
/**
 * SaveFocusHeight:Save the focus height
 * para height:the height of focus
 */
void LaserExecuter::SaveFocusHeight(float height)
{
  uint8_t Data[8];
  uint32_t intheight;
  intheight = height * 1000;
  if(FocusHeight > 65000)
    FocusHeight = 65000;

  Data[0] = (uint8_t)(intheight >> 8);
  Data[1] = (uint8_t)(intheight);
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_LASER_FOCUS, Data, 2);
}

/**
 * SaveFocusHeight:Save the focus height
 */
void LaserExecuter::SaveFocusHeight()
{
  uint8_t Data[8];
  uint32_t intheight;
  if(FocusHeight > 65000)
    FocusHeight = 65000;
  intheight = FocusHeight * 1000;

  Data[0] = (uint8_t)(intheight >> 8);
  Data[1] = (uint8_t)(intheight);
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_LASER_FOCUS, Data, 2);
}

/**
 * LoadFocusHeight:Load focus height from laser executer
 * return :true if load success or else false
 */
bool LaserExecuter::LoadFocusHeight()
{
  millis_t tmptick;
  uint8_t Data[3];

  Data[0] = 0;
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_REPORT_LASER_FOCUS, Data, 1);
  // Delay for update
  vTaskDelay(50 * portTICK_PERIOD_MS);

  return 0;
}
#endif // ENABLED(EXECUTER_CANBUS_SUPPORT)

/**
 * UpdateLaserPower:Update the laser power without output to the laser
 *para NewPower:New power the update
 */
void LaserExecuter::UpdateLaserPower(float NewPower) {
  last_percent = NewPower;
}


/**
 * SetBluetoothName:Set BT name
 * para Name:The name of the BT
 * ret  None
 */
char LaserExecuter::SetBluetoothName(char *name)
{
  Event_t  event = {M_SET_BT_NAME, 0};
  uint8_t  buffer[72];
  uint16_t size;
  ErrCode  ret;

  for(size = 0; size < 32; size++) {
    if(name[size] == 0)
      break;

    buffer[size] = name[size];
  }
  buffer[size++] = 0;

  event.length = size;
  event.data = buffer;

  serial_.Send(event);
  serial_.Flush();
  vTaskDelay(200);

  ret = serial_.CheckoutCmd(buffer, &size);
  if (ret != E_SUCCESS) {
    LOG_E("failed to set BT name: %u!\n", ret);
    return -1;
  }

  if((buffer[0] == S_SET_BT_NAME_ACK) && (buffer[1] == 0))
    return 0;

  return -1;
}

/**
 * ReadBlueToothName:Read BT Name
 * para Name:The pointer to the Name buffer
 * return:0 for read success, 1 for unname, 2 for timeout
 */
char LaserExecuter::ReadBluetoothName(char *name) {
  Event_t  event = {M_REPORT_BT_NAME, 0, 0, NULL};
  uint8_t  buff[72];
  uint16_t size;
  uint16_t i;
  ErrCode ret = E_SUCCESS;

  serial_.Send(event);
  serial_.Flush();
  vTaskDelay(200);

  ret = serial_.CheckoutCmd(buff, &size);
  if (ret != E_SUCCESS) {
    LOG_E("failed to read BT name - %u!\n", ret);
    return 2;
  }

  if (size < 2) {
    LOG_E("failed to read BT name - %u!\n", 112);
    return 2;
  }

  if (buff[0] != S_REPORT_BT_NAME_ACK || buff[1] != 0) {
    LOG_E("failed to read BT name - %u!\n", 113);
    return 1;
  }

  for (i = 2; i < size; i++) {
    if (!buff[i]) {
      break;
    }

    name[i-2] = buff[i];
  }

  name[i-2] = 0;

  return 0;
}

/**
 * ReadBluetoothMac:Read BlueTooth's MAC
 * para Name:The pointer to the Name buffer
 * return:0 for read success,  2 for timeout
 */
char LaserExecuter::ReadBluetoothMac(uint8_t *mac) {
  Event_t  event = {M_REPORT_BT_MAC, 0, 0, NULL};
  uint16_t size;
  uint16_t i = 0;
  uint8_t buff[32];
  ErrCode ret = E_SUCCESS;

  do {
    serial_.Send(event);
    serial_.Flush();
    vTaskDelay(200);

    ret = serial_.CheckoutCmd(buff, &size);
  } while (ret != E_SUCCESS && ++i < 3);

  if (ret != E_SUCCESS) {
    LOG_E("failed to read BT mac - %u!\n", ret);
    return 2;
  }

  if (size < 8) {
    LOG_E("failed to read BT mac - %u!\n", 112);
    return 2;
  }

  if (buff[0] != S_REPORT_BT_MAC_ACK || buff[1] != 0) {
    LOG_E("failed to read BT mac - %u!\n", 113);
    return 2;
  }

  for (i = 0; i < 6; i++) {
    mac[i] = buff[i + 2];
  }

  return 0;
}


uint16_t LaserExecuter::GetTimPwm() {
  return Tim1GetCCR4();
}

/**
 * change power limit, will be call when open / close chamber door
*/
void LaserExecuter::ChangePowerLimit(float limit) {
  float percent = last_percent;
  if (limit > LASER_POWER_NORMAL_LIMIT)
    limit = LASER_POWER_NORMAL_LIMIT;
  power_limit_ = limit;
  // if previous limit is larger than now, need to check need to lower current output
  ChangePower(last_percent);
  last_percent = percent;  // recover the value of the normal output

  if (GetTimPwm() > 0) {
    // If there is current output, it is equal to the limit output
    SetLaserPwm(last_pwm);
  }
}

/**
 * change power value, but not change output power
*/
void LaserExecuter::ChangePower(float percent) {
  int integer;
  float decimal;
  last_percent = percent;
  if (percent > power_limit_)
    percent = power_limit_;

  integer = percent;
  decimal = percent - integer;
  last_pwm = LaserPowerTable[integer] + (LaserPowerTable[integer + 1] - LaserPowerTable[integer]) * decimal;
}


ErrCode LaserExecuter::GetFocalLength(Event_t &event) {
  uint8_t buff[5];
  uint32_t focal_length;

  LoadFocusHeight();

  LOG_I("SC get focal length: %.3f\n", FocusHeight);

  buff[0] = 0;

  focal_length = (uint32_t)(FocusHeight * 1000);

  WORD_TO_PDU_BYTES(buff + 1, focal_length);

  event.length = 5;
  event.data = buff;

  return hmi.Send(event);
}


ErrCode LaserExecuter::SetFocalLength(Event_t &event) {
  ErrCode err = E_FAILURE;

  uint8_t buff[2];
  int     focal_length;

  if (event.length < 4) {
    LOG_E("Must specify focal length!\n");
    event.length = 1;
    event.data = &err;
    return hmi.Send(event);
  }

  PDU_TO_LOCAL_WORD(focal_length, event.data);

  LOG_I("SC set focal length: %d\n", focal_length);

  // length and data is picked up, can be changed
  event.length = 1;
  event.data = &err;

  if (focal_length > LASER_CAMERA_FOCAL_LENGTH_MAX) {
    LOG_E("new focal length[%d] is out of limit!\n", focal_length);
    return hmi.Send(event);
  }

  buff[0] = (uint8_t)(focal_length >> 8);
  buff[1] = (uint8_t)(focal_length);
  CanModules.SetFunctionValue(BASIC_CAN_NUM, FUNC_SET_LASER_FOCUS, buff, 2);

  LoadFocusHeight();

  err = E_SUCCESS;
  return hmi.Send(event);
}


ErrCode LaserExecuter::DoManualFocusing(Event_t &event) {
  ErrCode err = E_FAILURE;

  float pos[XYZ];

  float max_z_speed;

  LOG_I("SC req manual focusing\n");

  if (!all_axes_homed()) {
    LOG_E("Machine is not be homed!\n");
    goto out;
  }

  if (MACHINE_TYPE_LASER != ExecuterHead.MachineType) {
    LOG_E("Laser is offline!\n");
    goto out;
  }

  if (event.length < 12) {
    LOG_E("need to specify position!\n");
    goto out;
  }

  planner.synchronize();

  max_z_speed = planner.settings.max_feedrate_mm_s[Z_AXIS];

  PDU_TO_LOCAL_WORD(pos[X_AXIS], event.data);
  PDU_TO_LOCAL_WORD(pos[Y_AXIS], event.data+4);
  PDU_TO_LOCAL_WORD(pos[Z_AXIS], event.data+8);
  LOG_I("Laser will move to (%.2f, %.2f, %.2f)\n", pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS]);

  planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

  // Move to the Certain point
  do_blocking_move_to_logical_xy(pos[X_AXIS], pos[Y_AXIS], speed_in_calibration[X_AXIS]);

  // Move to the Z
  do_blocking_move_to_logical_z( pos[Z_AXIS], speed_in_calibration[Z_AXIS]);

  planner.synchronize();

  planner.settings.max_feedrate_mm_s[Z_AXIS] = max_z_speed;

out:
  event.length = 1;
  event.data = &err;
  return hmi.Send(event);
}


ErrCode LaserExecuter::DoAutoFocusing(Event_t &event) {
  ErrCode err = E_FAILURE;

  uint8_t Count = 21;
  float z_interval = 0.5;

  float start_pos[XYZ];

  int i = 0;
  float next_x, next_y, next_z;
  float line_space = 2;
  float line_len_short = 5;
  float line_len_long = 10;

  LOG_I("SC req auto focusing\n");

  if (!all_axes_homed()) {
    LOG_E("Machine is not be homed!\n");
    goto out;
  }

  if (MACHINE_TYPE_LASER != ExecuterHead.MachineType) {
    LOG_E("Laser is offline!\n");
    goto out;
  }

  if (event.length == 4) {
    PDU_TO_LOCAL_WORD(z_interval, event.data);
    z_interval /= 1000;
    LOG_I("SC specify Z interval: %.2f\n", z_interval);
  }

  planner.synchronize();

  LOOP_XYZ(i) {
    start_pos[i] = current_position[i];
  }

  next_x = start_pos[X_AXIS] - (int)(Count / 2) * 2;
  next_y = start_pos[Y_AXIS];
  next_z = start_pos[Z_AXIS] - ((float)(Count - 1) / 2.0 * z_interval);

  // too low
  if(next_z <= 5) {
    LOG_W("start Z height is too low: %.2f\n", next_z);
  }

  // Move to next Z
  move_to_limited_z(next_z, 20.0f);

  // Draw 10 Line
  do {
    // Move to the start point
    move_to_limited_xy(next_x, next_y, speed_in_calibration[X_AXIS]);
    planner.synchronize();

    // Laser on
    ExecuterHead.Laser.SetLaserPower(laser_pwr_in_cali);

    // Draw Line
    if((i % 5) == 0)
      move_to_limited_xy(next_x, next_y + line_len_long, speed_in_draw_ruler);
    else
      move_to_limited_xy(next_x, next_y + line_len_short, speed_in_draw_ruler);

    planner.synchronize();

    // Laser off
    ExecuterHead.Laser.SetLaserPower(0.0f);

    // Move up Z increase
    if(i != (Count - 1))
      move_to_limited_z(current_position[Z_AXIS] + z_interval, 20.0f);

    next_x = next_x + line_space;
    i++;
  } while(i < Count);

  planner.synchronize();

  // Move to beginning
  move_to_limited_z(start_pos[Z_AXIS], 20.0f);
  move_to_limited_xy(start_pos[X_AXIS], start_pos[Y_AXIS], 20.0f);
  planner.synchronize();

  err = E_SUCCESS;

out:
  event.data = &err;
  event.length = 1;
  return hmi.Send(event);
}


ErrCode LaserExecuter::SetCameraBtName(Event_t &event) {
  ErrCode err = E_FAILURE;

  LOG_I("SC set BT Name: %s\n", event.data);

  if(SetBluetoothName((char *)event.data) == 0)
    err = E_SUCCESS;

  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode LaserExecuter::GetCameraBtName(Event_t &event) {
  uint8_t buffer[34] = {0};
  int i;

  buffer[0] = ExecuterHead.Laser.ReadBluetoothName((char *)(buffer + 1));

  LOG_I("SC req BT Name: %s\n", buffer + 1);

  event.data = buffer;

  if (buffer[0] != 0) {
    event.length = 1;
    event.data = buffer;
  }
  else {
    for (i = 1; i < 34; i++)
      if (buffer[i] == 0) {
        event.length = i + 1;
        break;
      }
  }

  return hmi.Send(event);
}


ErrCode LaserExecuter::GetCameraBtMAC(Event_t &event) {
  uint8_t buffer[8] = {0};

  LOG_I("SC get BT MAC\n");

  buffer[0] = ReadBluetoothMac(buffer+1);
  event.data = buffer;

  // if buffer[0] is not 0, it indicates errors happened in ReadBluetoothMac()
  // just need to send the return code to screen
  if (buffer[0] != 0) {
    event.length = 1;
  }
  // otherwise send all data to screen 
  else {
    event.length = 7;
  }

  return hmi.Send(event);
}
