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
#include "linear.h"

#include "../common/config.h"
#include "../common/debug.h"
#include "../service/system.h"

// marlin headers
#include "src/inc/MarlinConfig.h"
#include "src/module/endstops.h"

Linear linear(MODULE_DEVICE_ID_LINEAR);
Linear linear_tmc(MODULE_DEVICE_ID_LINEAR_TMC);

Linear *linear_p = &linear;

LinearAxisType Linear::DetectAxis(MAC_t &mac, uint8_t &endstop) {
  CanExtCmd_t cmd;
  uint8_t     buffer[16];

  int i;
  int pins[3] = {X_DIR_PIN, Y_DIR_PIN, Z_DIR_PIN};

  cmd.mac    = mac;
  cmd.data   = buffer;

  WRITE(X_DIR_PIN, LOW);
  WRITE(Y_DIR_PIN, LOW);
  WRITE(Z_DIR_PIN, LOW);

  for (i = LINEAR_AXIS_X1; i <= LINEAR_AXIS_Z1; i++)  {
    WRITE(pins[i], HIGH);

    vTaskDelay(pdMS_TO_TICKS(10));

    cmd.data[MODULE_EXT_CMD_INDEX_ID]   = MODULE_EXT_CMD_CONFIG_REQ;
    cmd.data[MODULE_EXT_CMD_INDEX_DATA] = i;
    cmd.length = 2;

    if (canhost.SendExtCmdSync(cmd, 500) == E_SUCCESS) {
      if (cmd.data[MODULE_EXT_CMD_INDEX_DATA] == 1) {
        endstop = cmd.data[MODULE_EXT_CMD_INDEX_DATA + 3];
        break;
      }
    }

    WRITE(pins[i], LOW);
  }

  if (i > LINEAR_AXIS_Z1) {
    // if nobody tell us it detected low level from dir signal
    // we cannot recognize what kind of axis it is
    i = LINEAR_AXIS_UNKNOWN;
  }
  else {
    WRITE(pins[i], LOW);
  }

  return (LinearAxisType)i;
}


static void LinearCallbackEndstopX1(CanStdDataFrame_t &cmd) {
  switch (linear_p->machine_size())
  {
  case MACHINE_SIZE_A250:
  case MACHINE_SIZE_A350:
    linear_p->SetEndstopBit(X_MIN, cmd.data[0]);
    break;

  case MACHINE_SIZE_A150:
  default:
    linear_p->SetEndstopBit(X_MAX, cmd.data[0]);
    break;
  }
}


static void LinearCallbackEndstopY1(CanStdDataFrame_t &cmd) {
  linear_p->SetEndstopBit(Y_MAX, cmd.data[0]);
}

static void LinearCallbackEndstopY2(CanStdDataFrame_t &cmd) {
  linear_p->SetEndstopBit(Y_MAX, cmd.data[0]);
}

static void LinearCallbackEndstopZ1(CanStdDataFrame_t &cmd) {
  linear_p->SetEndstopBit(Z_MAX, cmd.data[0]);
}

static void LinearCallbackEndstopZ2(CanStdDataFrame_t &cmd) {
  linear_p->SetEndstopBit(Z_MAX, cmd.data[0]);
}


ErrCode Linear::Init(MAC_t &mac, uint8_t mac_index) {
  uint8_t   type;
  uint8_t   endstop;

  CanExtCmd_t cmd;
  uint8_t     func_buffer[16];

  Function_t    function;
  message_id_t  message_id[4];

  CanStdCmdCallback_t cb = NULL;
  int i;

  // need to check what kind of axis it is before we register function id
  type = DetectAxis(mac, endstop);
  if (type >= LINEAR_AXIS_MAX) {
    LOG_E("Unknown axis!\n\n", mac.val);
    return E_FAILURE;
  }

  LOG_I("\tGot axis %c, endstop: %u\n", axis_codes[type], endstop);

  // check if X/Y/Z-1 is exist
  if (mac_index_[type] != 0xFF) {
    // check if X/Y/Z-2 is exist
    if (mac_index_[type + 3] != 0xFF) {
      // because now we didn't support X/Y/Z-3, so just return unknown
      return E_FAILURE;
    }
    // it is one of X/Y/Z-2
    type += 3;
  }

  mac_index_[type] = mac_index;

  cmd.mac    = mac;
  cmd.data   = func_buffer;

  // try to get linear length
  cmd.data[MODULE_EXT_CMD_INDEX_ID]   = MODULE_EXT_CMD_LINEAR_LENGTH_REQ;
  cmd.data[MODULE_EXT_CMD_INDEX_DATA] = 0;
  cmd.length = 2;
  if (canhost.SendExtCmdSync(cmd, 500, 2) != E_SUCCESS)
    return E_FAILURE;
  length_[type] = (uint16_t)((cmd.data[2]<<24 | cmd.data[3]<<16 | cmd.data[4]<<8 | cmd.data[5]) / 1000);

  // try to get linear lead
  cmd.data[MODULE_EXT_CMD_INDEX_ID]   = MODULE_EXT_CMD_LINEAR_LEAD_REQ;
  cmd.data[MODULE_EXT_CMD_INDEX_DATA] = 0;
  cmd.length = 2;
  if (canhost.SendExtCmdSync(cmd, 500, 2) != E_SUCCESS)
    return E_FAILURE;
  lead_[type] = (uint16_t)((cmd.data[2]<<24 | cmd.data[3]<<16 | cmd.data[4]<<8 | cmd.data[5]) / 1000);

  LOG_I("\tlength: %u mm, lead: %u mm\n", length_[type], (200 * 16 / lead_[type]));

  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_GET_FUNCID_REQ;
  cmd.length = 1;

  // try to get function ids from module
  if (canhost.SendExtCmdSync(cmd, 500, 2) != E_SUCCESS)
    return E_FAILURE;

  switch (type) {
  case LINEAR_AXIS_X1:
  case LINEAR_AXIS_X2:
    cb = LinearCallbackEndstopX1;
    break;

  case LINEAR_AXIS_Y1:
    cb = LinearCallbackEndstopY1;
    break;
  case LINEAR_AXIS_Y2:
    cb = LinearCallbackEndstopY2;
    break;

  case LINEAR_AXIS_Z1:
    cb = LinearCallbackEndstopZ1;
    break;
  case LINEAR_AXIS_Z2:
    cb = LinearCallbackEndstopZ2;
    break;

  default:
    break;
  }

  function.channel   = cmd.mac.bits.channel;
  function.sub_index = type;
  function.mac_index = mac_index;
  function.priority  = MODULE_FUNC_PRIORITY_DEFAULT;

  if (cmd.data[MODULE_EXT_CMD_INDEX_DATA] > MODULE_FUNCTION_MAX_IN_ONE)
    cmd.data[MODULE_EXT_CMD_INDEX_DATA] = MODULE_FUNCTION_MAX_IN_ONE;

  // register function ids to can host, it will assign message id
  for (i = 0; i < cmd.data[MODULE_EXT_CMD_INDEX_DATA]; i++) {
    function.id = (cmd.data[i*2 + 2]<<8 | cmd.data[i*2 + 3]);
    if (function.id == MODULE_FUNC_ENDSTOP_STATE) {
      // just register callback for endstop
      // cache the message id for endstop for inquiring status of endstop later
      endstop_msg_[type] = canhost.RegisterFunction(function, cb);
      message_id[i] = endstop_msg_[type];
    }
    else {
      // for other functions in linear module, no callback for them
      message_id[i] = canhost.RegisterFunction(function, NULL);
    }
  }

  linear_p = this;

  return canhost.BindMessageID(cmd, message_id);
}

void Linear::reset_axis_steps_per_unit(void) {
  LOOP_X_TO_EN(i) {
    planner.settings.axis_steps_per_mm[i] = axis_steps_per_unit[i];
  }
}

ErrCode Linear::CheckModuleType() {
  int32_t i;
  uint32_t device_id = 0xffffffff;
  uint32_t id;
  MAC_t mac_t;
  // Initialize variables before detecting errors to avoid M999 movement errors
  if ((mac_index_[LINEAR_AXIS_X1] != 0xff) || (mac_index_[LINEAR_AXIS_X2] != 0xff)) {
    axis_steps_per_unit[X_AXIS] = mac_index_[LINEAR_AXIS_X1] != 0xff ? lead_[LINEAR_AXIS_X1] : lead_[LINEAR_AXIS_X2];
  }

  if ((mac_index_[LINEAR_AXIS_Y1] != 0xff) || (mac_index_[LINEAR_AXIS_Y2] != 0xff)) {
    axis_steps_per_unit[Y_AXIS] = mac_index_[LINEAR_AXIS_Y1] != 0xff ? lead_[LINEAR_AXIS_Y1] : lead_[LINEAR_AXIS_Y2];
  }

  if ((mac_index_[LINEAR_AXIS_Z1] != 0xff) || (mac_index_[LINEAR_AXIS_Z2] != 0xff) || (mac_index_[LINEAR_AXIS_Z3] != 0xff)) {
    axis_steps_per_unit[Z_AXIS] = mac_index_[LINEAR_AXIS_Z1] != 0xff ? lead_[LINEAR_AXIS_Z1] : mac_index_[LINEAR_AXIS_Z2] != 0xff ? lead_[LINEAR_AXIS_Z2] : lead_[LINEAR_AXIS_Z3];
  } else {
    axis_steps_per_unit[Z_AXIS] = MODULE_LINEAR_PITCH_20;
  }

  LOOP_XYZ(i) {
    if (!planner.is_user_set_lead) {
      planner.settings.axis_steps_per_mm[i] = axis_steps_per_unit[i];
    }
    SERIAL_ECHOLNPAIR("axis index:", i, "  pitch:", planner.settings.axis_steps_per_mm[i]);
  }

  planner.refresh_positioning();

  // check if all linear modules are the same generation
  for (i = LINEAR_AXIS_X1; i < LINEAR_AXIS_MAX; i++) {
    if (mac_index_[i] == 0xff) continue;

    mac_t.val = canhost.mac(mac_index_[i]);
    id = MODULE_GET_DEVICE_ID(mac_t.val);
    if (device_id == 0xffffffff) {
      device_id = id;
    }
    else if (device_id != id) {
      LOG_I("device id error\n");
      systemservice.ThrowException(EHOST_LINEAR, ETYPE_LINEAR_MODULE_DIFF_DRIVER);
      return E_FAILURE;
    }
  }

  // if using 2.5 generation linear modules, check if the linear module lead is correct for each axis
  if (device_id == MODULE_DEVICE_ID_LINEAR_TMC) {
    for (i = LINEAR_AXIS_X1; i < LINEAR_AXIS_MAX; i++) {
      if (mac_index_[i] == 0xff) continue;

      switch (i) {
        case LINEAR_AXIS_X1:
        case LINEAR_AXIS_X2:
        case LINEAR_AXIS_Y1:
        case LINEAR_AXIS_Y2:
          if (lead_[i] != MODULE_LINEAR_PITCH_20) {
            systemservice.ThrowException(EHOST_LINEAR, ETYPE_LINEAR_MODULE_LEAD_ERROR);
            return E_FAILURE;
          }
          break;
        case LINEAR_AXIS_Z1:
        case LINEAR_AXIS_Z2:
        case LINEAR_AXIS_Z3:
          if (lead_[i] != MODULE_LINEAR_PITCH_8) {
            systemservice.ThrowException(EHOST_LINEAR, ETYPE_LINEAR_MODULE_LEAD_ERROR);
            return E_FAILURE;
          }
          break;
        default:
          break;
      }
    }
  }

  return E_SUCCESS;
}


ErrCode Linear::PollEndstop(LinearAxisType axis) {
  CanStdMesgCmd_t message;

  // no data field in this message
  message.length = 0;

  if (axis < LINEAR_AXIS_MAX) {
    message.id = endstop_msg_[axis];
    return canhost.SendStdCmd(message);
  }

  for (int i = 0; i < LINEAR_AXIS_MAX; i++) {
    if (endstop_msg_[i] == MODULE_MESSAGE_ID_INVALID)
      continue;

    message.id = endstop_msg_[i];
    canhost.SendStdCmd(message);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  return E_SUCCESS;
}


MachineSize Linear::UpdateMachineSize() {
  bool is_err = false;
  if (length_[LINEAR_AXIS_X1] < 200) {
    // A150 judges only 3 axes,
    if (length_[LINEAR_AXIS_X1] == 0 ||
        length_[LINEAR_AXIS_X1] != length_[LINEAR_AXIS_Y1] ||
        length_[LINEAR_AXIS_Y1] != length_[LINEAR_AXIS_Z1]) {
      is_err = true;
    }
  } else {
    if (length_[LINEAR_AXIS_X1] != length_[LINEAR_AXIS_Y1] ||
        length_[LINEAR_AXIS_Y1] != length_[LINEAR_AXIS_Y2] ||
        length_[LINEAR_AXIS_Y2] != length_[LINEAR_AXIS_Z1] ||
        length_[LINEAR_AXIS_Z1] != length_[LINEAR_AXIS_Z2]) {
      is_err = true;
    }
  }

  if (CheckModuleType() != E_SUCCESS || is_err) {
    LOG_I("Model: unknow\n");
    if (length_[LINEAR_AXIS_X1] < 200 &&
        length_[LINEAR_AXIS_Y1] < 200 &&
        length_[LINEAR_AXIS_Z1] < 200) {
      X_HOME_DIR = 1;
      X_DIR = false;
      Y_HOME_DIR = 1;
      Y_DIR = false;
      Z_HOME_DIR = 1;
      Z_DIR = false;
    } else {
      X_HOME_DIR = -1;
      X_DIR = true;
      Y_HOME_DIR = 1;
      Y_DIR = false;
      Z_HOME_DIR = 1;
      Z_DIR = false;
    }
    X_MAX_POS = length_[LINEAR_AXIS_X1];
    Y_MAX_POS = length_[LINEAR_AXIS_Y1];
    Z_MAX_POS = length_[LINEAR_AXIS_Z1];
    UpdateMachineDefines();
    systemservice.ThrowException(EHOST_LINEAR, ETYPE_NO_HOST);
    return (machine_size_ = MACHINE_SIZE_UNKNOWN);
  } else if (length_[LINEAR_AXIS_X1] < 200) {
    LOG_I("Model: A150\n");
    X_MAX_POS = 167;
    Y_MAX_POS = 165;
    Z_MAX_POS = 150;
    X_HOME_DIR = 1;
    X_DIR = false;
    Y_HOME_DIR = 1;
    Y_DIR = false;
    Z_HOME_DIR = 1;
    Z_DIR = false;

    LOOP_XN(i) home_offset[i] = s_home_offset[i];

    X_DEF_SIZE = 160;
    Y_DEF_SIZE = 160;
    Z_DEF_SIZE = 145;

    MAGNET_X_SPAN = 114;
    MAGNET_Y_SPAN = 114;

    machine_size_ = MACHINE_SIZE_A150;

  } else if (length_[LINEAR_AXIS_X1] < 300) {
    LOG_I("Model: A250\n");
    X_MAX_POS = 252;
    Y_MAX_POS = 260;
    Z_MAX_POS = 235;
    X_HOME_DIR = -1;
    X_DIR = true;
    Y_HOME_DIR = 1;
    Y_DIR = false;
    Z_HOME_DIR = 1;
    Z_DIR = false;

    LOOP_XN(i) home_offset[i] = m_home_offset[i];

    X_DEF_SIZE = 230;
    Y_DEF_SIZE = 250;
    Z_DEF_SIZE = 235;

    MAGNET_X_SPAN = 184;
    MAGNET_Y_SPAN = 204;

    machine_size_ = MACHINE_SIZE_A250;
  } else if (length_[LINEAR_AXIS_X1] < 400) {
    LOG_I("Model: A350\n");
    X_MAX_POS = 345;
    Y_MAX_POS = 357;
    Z_MAX_POS = 334;
    X_HOME_DIR = -1;
    X_DIR = true;
    Y_HOME_DIR = 1;
    Y_DIR = false;
    Z_HOME_DIR = 1;
    Z_DIR = false;

    LOOP_XN(i) home_offset[i] = l_home_offset[i];

    X_DEF_SIZE = 320;
    Y_DEF_SIZE = 340;
    Z_DEF_SIZE = 330; // unused & spec is lager than actual size.  334 - 6 = 328?

    MAGNET_X_SPAN = 274;
    MAGNET_Y_SPAN = 304;

    machine_size_ = MACHINE_SIZE_A350;
  }

  UpdateMachineDefines();
  endstops.reinit_hit_status();
  PollEndstop(LINEAR_AXIS_ALL);
  systemservice.ClearException(EHOST_MC, ETYPE_NO_HOST);
  return machine_size_;
}


ErrCode Linear::SetLengthOrLead(SSTP_Event_t &event, uint8_t ext_cmd) {
  CanExtCmd_t cmd;
  uint8_t     buffer[8];

  int      i;
  uint32_t target_mac;

  PDU_TO_LOCAL_WORD(target_mac, event.data);

  cmd.data    = buffer;
  cmd.data[0] = ext_cmd;
  cmd.data[1] = 1;
  cmd.data[2] = event.data[4];
  cmd.data[3] = event.data[5];
  cmd.data[4] = event.data[6];
  cmd.data[5] = event.data[7];

  // error code to HMI
  event.data[0] = E_FAILURE;
  event.length = 1;

  target_mac &= MODULE_MAC_ID_MASK;
  for (i = 0; i < LINEAR_AXIS_MAX; i++) {
    if  (target_mac == (canhost.mac(mac_index_[i]) & MODULE_MAC_ID_MASK))
      goto out;
  }

  goto error;

out:

  cmd.mac.val = canhost.mac(mac_index_[i]);
  event.data[0] = canhost.SendExtCmdSync(cmd, 500);

error:
  return hmi.Send(event);
}


ErrCode Linear::GetLengthOrLead(SSTP_Event_t &event, uint8_t ext_cmd) {
  int i, j = 0;

  uint32_t    mac;
  CanExtCmd_t cmd;
  uint8_t     can_buffer[8];

  uint8_t buffer[8 * LINEAR_AXIS_MAX];


  cmd.data = can_buffer;

  for(i = 0; i < LINEAR_AXIS_MAX; i++) {
    if (mac_index_[i] == MODULE_MAC_INDEX_INVALID)
      continue;

    cmd.data[0] = ext_cmd;
    cmd.data[1] = 0;
    cmd.length  = 2;
    cmd.mac.val = canhost.mac(mac_index_[i]);

    if (canhost.SendExtCmdSync(cmd, 500) != E_SUCCESS)
      continue;

    mac = cmd.mac.val;
    WORD_TO_PDU_BYTES_INDEX_MOVE(buffer, mac, j);

    // length of this linear
    buffer[j++] = cmd.data[2];
    buffer[j++] = cmd.data[3];
    buffer[j++] = cmd.data[4];
    buffer[j++] = cmd.data[5];
  }

  event.data   = buffer;
  event.length = (uint16_t)j;

  return hmi.Send(event);
}
