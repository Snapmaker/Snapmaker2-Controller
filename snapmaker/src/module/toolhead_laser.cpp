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
#include "toolhead_laser.h"

#include "../common/config.h"
#include "../common/debug.h"
#include "../gcode/M1028.h"

#include "../snapmaker.h"

// marlin headers
#include "src/core/macros.h"
#include "src/core/boards.h"
#include "Configuration.h"
#include "src/pins/pins.h"
#include "src/module/motion.h"
#include "src/module/planner.h"
#include "rotary_module.h"
#include "src/module/stepper.h"


#define LASER_CLOSE_FAN_DELAY     (120)

#define TimSetPwm(n)  Tim1SetCCR4(n)
#define TimGetPwm()  Tim1GetCCR4()

ToolHeadLaser laser;

extern void Tim1SetCCR4(uint16_t pwm);
extern uint16_t Tim1GetCCR4();
extern void Tim1PwmInit();

static __attribute__((section(".data"))) uint8_t power_table[]= {
  0,
  20,22,24,26,28,30,31,33,35,37,39,41,43,45,47,49,51,53,54,56,58,60,63,65,67,69,71,73,75,77,79,82,84,86,88,90,93,95,97,
  100,102,103,106,109,111,113,116,119,121,123,125,128,130,133,135,138,140,143,145,148,150,153,156,158,161,164,166,169,
  171,174,177,179,182,185,187,190,192,196,198,200,203,205,208,210,211,214,217,218,221,224,226,228,231,234,236,240,242,
  247,251,255
};

static void CallbackAckLaserFocus(CanStdDataFrame_t &cmd) {
  laser.focus(cmd.data[0]<<8 | cmd.data[1]);
}


ErrCode ToolHeadLaser::Init(MAC_t &mac, uint8_t mac_index) {
  ErrCode ret;

  CanExtCmd_t cmd;
  uint8_t     func_buffer[16];

  Function_t    function;
  message_id_t  message_id[4];

  if (axis_to_port[E_AXIS] != PORT_8PIN_1) {
    LOG_E("toolhead Laser failed: Please use the <M1029 E1> set E port\n");
    return E_HARDWARE;
  }

  ret = ModuleBase::InitModule8p(mac, E0_DIR_PIN, 0);
  if (ret != E_SUCCESS)
    return ret;

  LOG_I("\tGot toolhead Laser!\n");

  // we have configured Laser in same port
  if (mac_index_ != MODULE_MAC_INDEX_INVALID)
    return E_SAME_STATE;

  cmd.mac    = mac;
  cmd.data   = func_buffer;
  cmd.length = 1;

  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_GET_FUNCID_REQ;

  // try to get function ids from module
  if (canhost.SendExtCmdSync(cmd, 500, 2) != E_SUCCESS)
    return E_FAILURE;

  function.channel   = mac.bits.channel;
  function.mac_index = mac_index;
  function.sub_index = 0;
  function.priority  = MODULE_FUNC_PRIORITY_DEFAULT;

  // register function ids to can host, it will assign message id
  for (int i = 0; i < cmd.data[MODULE_EXT_CMD_INDEX_DATA]; i++) {
    function.id = (cmd.data[i*2 + 2]<<8 | cmd.data[i*2 + 3]);
    if (function.id == MODULE_FUNC_GET_LASER_FOCUS) {
      message_id[i]     = canhost.RegisterFunction(function, CallbackAckLaserFocus);
      msg_id_get_focus_ = message_id[i];
    }
    else
      message_id[i] = canhost.RegisterFunction(function, NULL);

    if (function.id == MODULE_FUNC_SET_FAN1)
      msg_id_set_fan_ = message_id[i];
  }

  ret = canhost.BindMessageID(cmd, message_id);

  Tim1PwmInit();
  esp32_.Init(&MSerial3, EXECUTOR_SERIAL_IRQ_PRIORITY);

  mac_index_ = mac_index;
  state_     = TOOLHEAD_LASER_STATE_OFF;

  // set toolhead
  SetToolhead(MODULE_TOOLHEAD_LASER);

  return E_SUCCESS;
}

uint16_t ToolHeadLaser::tim_pwm() {
  return TimGetPwm();
}
void ToolHeadLaser::tim_pwm(uint16_t pwm) {
  TimSetPwm(pwm);
}

void ToolHeadLaser::TurnOn() {
  if (state_ == TOOLHEAD_LASER_STATE_OFFLINE)
    return;

  state_ = TOOLHEAD_LASER_STATE_ON;
  CheckFan(power_pwm_);
  tim_pwm(power_pwm_);
}


void ToolHeadLaser::TurnOff() {
  if (state_ == TOOLHEAD_LASER_STATE_OFFLINE)
    return;

  state_ = TOOLHEAD_LASER_STATE_OFF;
  CheckFan(0);
  tim_pwm(0);
}


void ToolHeadLaser::SetOutput(float power) {
  SetPower(power);
  TurnOn();
}


void ToolHeadLaser::SetPower(float power) {
  int   integer;
  float decimal;

  if (state_ == TOOLHEAD_LASER_STATE_OFFLINE)
    return;

  power_val_ = power;

  if (power > power_limit_)
    power = power_limit_;

  integer = (int)power;
  decimal = power - integer;

  power_pwm_ = (uint16_t)(power_table[integer] + (power_table[integer + 1] - power_table[integer]) * decimal);
}


void ToolHeadLaser::SetPowerLimit(float limit) {
  float cur_power = power_val_;

  if (limit > TOOLHEAD_LASER_POWER_NORMAL_LIMIT)
    limit = TOOLHEAD_LASER_POWER_NORMAL_LIMIT;

  power_limit_ = limit;

  SetPower(power_val_);
  power_val_ = cur_power;

  // check if we need to change current output
  if (state_ == TOOLHEAD_LASER_STATE_ON)
    TurnOn();
}

void ToolHeadLaser::SetFanPower(uint8_t power) {
  CanStdMesgCmd_t cmd;
  uint8_t         buffer[2];
  buffer[0]  = 0;
  buffer[1]  = power;
  cmd.id     = msg_id_set_fan_;
  cmd.data   = buffer;
  cmd.length = 2;

  canhost.SendStdCmd(cmd);
}

void ToolHeadLaser::CheckFan(uint16_t pwm) {
  switch (fan_state_) {
  case TOOLHEAD_LASER_FAN_STATE_OPEN:
    if (pwm == 0) {
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_TO_BE_CLOSED;
      fan_tick_  = 0;
    }
    break;

  case TOOLHEAD_LASER_FAN_STATE_TO_BE_CLOSED:
    if (pwm > 0) {
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_OPEN;
      fan_tick_  = 0;
    }
    break;

  case TOOLHEAD_LASER_FAN_STATE_CLOSED:
    if (pwm > 0) {
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_OPEN;
      SetFanPower(255);
    }
    break;
  }
}

void ToolHeadLaser::TryCloseFan() {
  if (state_ == TOOLHEAD_LASER_STATE_OFFLINE)
    return;

  if (fan_state_ == TOOLHEAD_LASER_FAN_STATE_TO_BE_CLOSED) {
    if (++fan_tick_ > LASER_CLOSE_FAN_DELAY) {
      fan_tick_  = 0;
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_CLOSED;
      SetFanPower(0);
    }
  }
}


ErrCode ToolHeadLaser::LoadFocus() {
  CanStdMesgCmd_t cmd;
  uint8_t data[1];
  if (rotaryModule.status() != ROTATE_OFFLINE) {
    data[0] = 1;
  } else {
    data[0] = 0;
  }
  cmd.id     = msg_id_get_focus_;
  cmd.length = 1;
  cmd.data = data;
  return canhost.SendStdCmd(cmd);
}


ErrCode ToolHeadLaser::GetFocus(SSTP_Event_t &event) {
  uint8_t  buff[5];

  LoadFocus();
  vTaskDelay(pdMS_TO_TICKS(20));

  LOG_I("SC get Focus: %.2f mm\n", focus_);
  uint16_t focus = focus_ * 1000;

  buff[0] = 0;

  buff[1] = 0;
  buff[2] = 0;
  buff[3] = (uint8_t)(focus>>8);
  buff[4] = (uint8_t)focus;

  event.length = 5;
  event.data   = buff;

  return hmi.Send(event);
}


ErrCode ToolHeadLaser::SetFocus(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  CanStdFuncCmd_t cmd;

  uint8_t  buff[3];
  uint32_t focus;

  if (event.length < 4) {
    LOG_E("Must specify Focus!\n");
    event.length = 1;
    event.data = &err;
    return hmi.Send(event);
  }

  PDU_TO_LOCAL_WORD(focus, event.data);

  LOG_I("SC set Focus: %.2f\n", focus / 1000.0f);

  // length and data is picked up, can be changed
  event.length = 1;
  event.data = &err;

  if (focus > TOOLHEAD_LASER_CAMERA_FOCUS_MAX) {
    LOG_E("new focus[%u] is out of range!\n", focus);
    return hmi.Send(event);
  }

  buff[0] = (uint8_t)(focus >> 8);
  buff[1] = (uint8_t)(focus);
  if (rotaryModule.status() != ROTATE_OFFLINE) {
    buff[2] = 1;
  } else {
    buff[2] = 0;
  }

  cmd.id     = MODULE_FUNC_SET_LASER_FOCUS;
  cmd.data   = buff;
  cmd.length = 3;

  err = canhost.SendStdCmd(cmd, 0);

  if (err == E_SUCCESS)
    LoadFocus();
  LOG_I("SC new focal length: %.3f\n", focus_);

  return hmi.Send(event);
}


ErrCode ToolHeadLaser::DoManualFocusing(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  float pos[XYZ];

  float max_z_speed;

  LOG_I("SC req manual focusing\n");

  if (!all_axes_homed()) {
    LOG_E("Machine is not be homed!\n");
    goto out;
  }

  if (!IsOnline()) {
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
  event.data   = &err;
  return hmi.Send(event);
}


ErrCode ToolHeadLaser::DoAutoFocusing(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  uint8_t Count = 21;
  float z_interval = 0.5;

  float start_pos[XYZ];

  int   i = 0;
  float next_x, next_y, next_z;
  float line_space = 2;
  float line_len_short = 5;
  float line_len_long = 10;

  LOG_I("SC req auto focusing\n");

  if (!all_axes_homed()) {
    LOG_E("Machine is not be homed!\n");
    goto out;
  }

  if (!IsOnline()) {
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
  if(next_z <= 10) {
    LOG_E("start Z height is too low: %.2f\n", next_z);
    goto out;
  }

  // Move to next Z
  move_to_limited_z(next_z, 20.0f);

  // Draw 10 Line
  do {
    // Move to the start point
    move_to_limited_xy(next_x, next_y, speed_in_calibration[X_AXIS]);
    planner.synchronize();

    // Laser on
    SetOutput(laser_pwr_in_cali);

    // Draw Line
    if((i % 5) == 0)
      move_to_limited_xy(next_x, next_y + line_len_long, speed_in_draw_ruler);
    else
      move_to_limited_xy(next_x, next_y + line_len_short, speed_in_draw_ruler);

    planner.synchronize();

    // Laser off
    SetOutput(0);

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
  event.data   = &err;
  event.length = 1;
  return hmi.Send(event);
}



/**
 * ReadBlueToothName:Read BT Name
 * para Name:The pointer to the Name buffer
 * return:0 for read success, 1 for unname, 2 for timeout
 */
ErrCode ToolHeadLaser::ReadBluetoothInfo(LaserCameraCommand cmd, uint8_t *out, uint16_t &length) {
  SSTP_Event_t  event = {cmd, 0, 0, NULL};

  ErrCode  ret = E_SUCCESS;


  for (int i = 1; i < 4; i++) {
    esp32_.FlushInput();

    esp32_.Send(event);
    esp32_.FlushOutput();
    vTaskDelay(pdMS_TO_TICKS(200 * i));

    ret = esp32_.CheckoutCmd(out, length);
    if (ret != E_SUCCESS) {
      LOG_E("failed to read BT info - %u!\n", ret);
    }
    else {
      goto out;
    }
  }

  return ret;

out:
  if (length < 2) {
    LOG_E("failed to read BT info - %u!\n", 110);
    return E_NO_RESRC;
  }

  if (out[0] != (cmd + 1)) {
    LOG_E("failed to read BT info - %u!\n", 111);
    return E_INVALID_DATA;
  }

  if (out[1] != 0) {
    LOG_E("failed to read BT info - %u!\n", 112);
    return E_INVALID_DATA;
  }

  return E_SUCCESS;
}


/**
 * SetBluetoothName:Set BT name
 * para Name:The name of the BT
 * ret  None
 */
ErrCode ToolHeadLaser::SetBluetoothInfo(LaserCameraCommand cmd, uint8_t *info, uint16_t length) {
  SSTP_Event_t  event = {cmd, 0};

  uint8_t  buffer[72];
  ErrCode  ret;

  event.length = length;
  event.data   = info;

  for (int i = 1; i < 4; i++) {
    esp32_.FlushInput();

    esp32_.Send(event);
    esp32_.FlushOutput();
    vTaskDelay(pdMS_TO_TICKS(200 * i));

    ret = esp32_.CheckoutCmd(buffer, length);
    if (ret != E_SUCCESS) {
      LOG_E("failed to set BT info - %u!\n", ret);
    }
    else {
      goto out;
    }
  }

  return ret;

out:
  if (buffer[0] != (cmd + 1)) {
    LOG_E("failed to read BT info - %u!\n", 111);
    return E_INVALID_CMD;
  }

  if (buffer[1] != 0) {
    LOG_E("failed to read BT info - %u!\n", 112);
    return E_INVALID_DATA;
  }

  return E_SUCCESS;
}


ErrCode ToolHeadLaser::SetCameraBtName(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  LOG_I("SC set BT Name: %s\n", event.data);

  err = SetBluetoothInfo(M_SET_BT_NAME, event.data, event.length);

  event.data   = &err;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode ToolHeadLaser::GetCameraBtName(SSTP_Event_t &event) {
  uint8_t buffer[40] = {0};
  ErrCode ret;

  ret = ReadBluetoothInfo(M_REPORT_BT_NAME, buffer, event.length);

  LOG_I("SC req BT Name: %s\n", buffer + 2);


  if (ret != E_SUCCESS) {
    event.length = 1;
    event.data   = &ret;
  }
  else {
    /* when ReadBluetoothInfo() save content to buffer[],
     * buffer[0] is ACK command id, buffer[1] is ACK code from module,
     * buffer[2: end] is actual information we need.
     * Because we also need to return ACK to to HMI, so
     * event.data = buffer + 1; and length increase one
     */
    event.data   = buffer + 1;
    event.length -= 1;
  }

  return hmi.Send(event);
}


ErrCode ToolHeadLaser::GetCameraBtMAC(SSTP_Event_t &event) {
  uint8_t buffer[16] = {0};
  ErrCode ret;

  LOG_I("SC get BT MAC\n");

  ret = ReadBluetoothInfo(M_REPORT_BT_MAC, buffer, event.length);

  if (ret != E_SUCCESS) {
    event.data   = &ret;
    event.length = 1;
  }
  else {
    /* when ReadBluetoothInfo() save content to buffer[],
     * buffer[0] is ACK command id, buffer[1] is ACK code from module,
     * buffer[2: 2+length] is actual information we need.
     * Because we also need to return ACK to to HMI, so
     * event.data = buffer + 1; and length increase one
     */
    event.data   = buffer + 1;
    event.length -= 1;

    LOG_I("\t0x%08X, 0x%08X\n", *(uint32_t *)(buffer), *(uint32_t *)(buffer + 4));
  }

  return hmi.Send(event);
}

/**
 * ReadBlueToothName:Read BT versions
 * return:0 for read success, 1 for unname, 2 for timeout
 */
ErrCode ToolHeadLaser::ReadBluetoothVer() {
  uint8_t  buff[72];
  uint16_t size;
  ErrCode ret = E_SUCCESS;

  ret = ReadBluetoothInfo(M_REPORT_VERSIONS, buff, size);

  if (ret != E_SUCCESS) {
    LOG_E("failed to read BT version - %u!\n", ret);
    return 2;
  }

  LOG_I("BT version: %s\n", buff + 2);
  return 0;
}

/**
 * SetCameraLight:set camera light status
 * para state:1-open 0-close
 */
void ToolHeadLaser::SetCameraLight(uint8_t state) {
  SSTP_Event_t  event = {M_SET_CAMERA_LIGHT, 0, 0, NULL};
  uint8_t buff[1];
  buff[0] = !state;
  event.data = buff;
  event.length = 1;
  SetBluetoothInfo(M_SET_CAMERA_LIGHT, event.data, event.length);
  LOG_I("set Laser Camera light:%d!\n", state);
}

void ToolHeadLaser::Process() {
  if (++timer_in_process_ < 100) return;

  timer_in_process_ = 0;

  TryCloseFan();
}
