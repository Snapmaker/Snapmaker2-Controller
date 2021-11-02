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

#define LINEAR_MODULE_MAC_INDEX_INVALID {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
#define LINEAR_MODULE_MESSAGE_ID_INVALID {0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff}

MachineSize Linear::machine_size_ = MACHINE_SIZE_UNKNOWN;
uint8_t Linear::mac_index_[] = LINEAR_MODULE_MAC_INDEX_INVALID;
uint16_t Linear::length_[] = {0};
uint16_t Linear::lead_[] = {0};
message_id_t Linear::endstop_msg_[] = LINEAR_MODULE_MESSAGE_ID_INVALID;
uint32_t Linear::endstop_ = 0xFFFFFFFF;
 
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
    linear_p->SetEndstopBit(X_MAX, cmd.data[0]);
    break;

  default:
    if (linear_p->length(LINEAR_AXIS_X1) < 200) {
      linear_p->SetEndstopBit(X_MAX, cmd.data[0]);
    } else {
      linear_p->SetEndstopBit(X_MIN, cmd.data[0]);
    } 
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

ErrCode Linear::UpdateMachineLead() {

  uint16_t type_id[LINEAR_AXIS_MAX];
  float axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;

  for (uint8_t i = LINEAR_AXIS_X1; i < LINEAR_AXIS_MAX; i++) {
    type_id[i] = CheckModuleType(mac_index_[i]);
  }
  // check if the coaxial types match
  if (type_id[LINEAR_AXIS_X1] && type_id[LINEAR_AXIS_X2]) {
    if (type_id[LINEAR_AXIS_X1] != type_id[LINEAR_AXIS_X2]) {
      goto type_err;
    }
  } 

  if (type_id[LINEAR_AXIS_Y1] && type_id[LINEAR_AXIS_Y2]) {
    if (type_id[LINEAR_AXIS_Y1] != type_id[LINEAR_AXIS_Y2]) {
      goto type_err;
    }
  }

  if (type_id[LINEAR_AXIS_Z1] && type_id[LINEAR_AXIS_Z2]) {
    if (type_id[LINEAR_AXIS_Z1] != type_id[LINEAR_AXIS_Z2]) {
      goto type_err;
    } else if (type_id[LINEAR_AXIS_Z3]) {
      if (type_id[LINEAR_AXIS_Z2] != type_id[LINEAR_AXIS_Z3]) {
        goto type_err;
      }
    }
  }
  // Whether the coaxial leads are equal
  if ((mac_index_[LINEAR_AXIS_X1] != 0xff) && (mac_index_[LINEAR_AXIS_X2] != 0xff)) {
    if (lead_[LINEAR_AXIS_X1] == lead_[LINEAR_AXIS_X2]) {
      axis_steps_per_unit[X_AXIS] = lead_[LINEAR_AXIS_X1];
    } else {
       goto lead_err;
    }  
  } else if ((mac_index_[LINEAR_AXIS_X1] != 0xff) || (mac_index_[LINEAR_AXIS_X2] != 0xff)) {
    axis_steps_per_unit[X_AXIS] = mac_index_[LINEAR_AXIS_X1] != 0xff ? lead_[LINEAR_AXIS_X1] : lead_[LINEAR_AXIS_X2];
  } 

  if ((mac_index_[LINEAR_AXIS_Y1] != 0xff) && (mac_index_[LINEAR_AXIS_Y2] != 0xff)) {
    if (lead_[LINEAR_AXIS_Y1] == lead_[LINEAR_AXIS_Y2]) {
       axis_steps_per_unit[Y_AXIS] = lead_[LINEAR_AXIS_Y1];                                
    } else {
       goto lead_err;
    }
  } else if ((mac_index_[LINEAR_AXIS_Y1] != 0xff) || (mac_index_[LINEAR_AXIS_Y2] != 0xff)) {
    axis_steps_per_unit[Y_AXIS] = mac_index_[LINEAR_AXIS_Y1] != 0xff ? lead_[LINEAR_AXIS_Y1] : lead_[LINEAR_AXIS_Y2];
  }

  if ((mac_index_[LINEAR_AXIS_Z1] != 0xff) && (mac_index_[LINEAR_AXIS_Z2] != 0xff)) {
    if (lead_[LINEAR_AXIS_Z1] == lead_[LINEAR_AXIS_Z2]) {
      axis_steps_per_unit[Z_AXIS] = lead_[LINEAR_AXIS_Z1];     
    } else {
       goto lead_err;
    }
  } else if ((mac_index_[LINEAR_AXIS_Z1] != 0xff) || (mac_index_[LINEAR_AXIS_Z2] != 0xff)) {
    axis_steps_per_unit[Z_AXIS] = mac_index_[LINEAR_AXIS_Z1] != 0xff ? lead_[LINEAR_AXIS_Z1] : lead_[LINEAR_AXIS_Z2];
  }
  if ((mac_index_[LINEAR_AXIS_Z3] != 0xff) && (lead_[LINEAR_AXIS_Z3] != axis_steps_per_unit[Z_AXIS])) {
    goto lead_err;
  }

  LOOP_XYZ(i) {
    if (!planner.is_user_set_lead) {
      planner.settings.axis_steps_per_mm[i] = axis_steps_per_unit[i];
    }
    SERIAL_ECHOLNPAIR("axis index:", i, "  pitch:", planner.settings.axis_steps_per_mm[i]);
  }

  planner.refresh_positioning();
  return E_SUCCESS;

type_err:
  SERIAL_ECHOLNPAIR("coaxial linear module type match error\n");
  systemservice.ThrowException(EHOST_LINEAR, ETYPE_LINEAR_MODULE_DIFF_DRIVER);
  return E_FAILURE;

lead_err:
  SERIAL_ECHOLNPAIR("coaxial linear module lead not equal\n");
  systemservice.ThrowException(EHOST_LINEAR, ETYPE_LINEAR_MODULE_LEAD_ERROR);
  return E_FAILURE;
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
  uint16_t axis_length[3] = {0, 0, 0};

  if (UpdateMachineLead() != E_SUCCESS) {
    X_MAX_POS = 0;
    Y_MAX_POS = 0;
    Z_MAX_POS = 0;
    systemservice.ThrowException(EHOST_MC, ETYPE_NO_HOST);
    return (machine_size_ = MACHINE_SIZE_UNKNOWN);
  }

  // Whether the coaxial lengths are equal
  if ((mac_index_[LINEAR_AXIS_X1] != 0xff) && (mac_index_[LINEAR_AXIS_X2] != 0xff)) {
    if (length_[LINEAR_AXIS_X1] == length_[LINEAR_AXIS_X2]) {
      axis_length[X_AXIS] = length_[LINEAR_AXIS_X1];
    } else {
      goto length_err;
    }  
  } else if ((mac_index_[LINEAR_AXIS_X1] != 0xff) || (mac_index_[LINEAR_AXIS_X2] != 0xff)) {
    axis_length[X_AXIS] = mac_index_[LINEAR_AXIS_X1] != 0xff ? length_[LINEAR_AXIS_X1] : length_[LINEAR_AXIS_X2];
  } 

  if ((mac_index_[LINEAR_AXIS_Y1] != 0xff) && (mac_index_[LINEAR_AXIS_Y2] != 0xff)) {
    if (length_[LINEAR_AXIS_Y1] == length_[LINEAR_AXIS_Y2]) {
       axis_length[Y_AXIS] = length_[LINEAR_AXIS_Y1];                                
    } else {
      goto length_err;
    }
  } else if ((mac_index_[LINEAR_AXIS_Y1] != 0xff) || (mac_index_[LINEAR_AXIS_Y2] != 0xff)) {
    axis_length[Y_AXIS] = mac_index_[LINEAR_AXIS_Y1] != 0xff ? length_[LINEAR_AXIS_Y1] : length_[LINEAR_AXIS_Y2];
  }

  if ((mac_index_[LINEAR_AXIS_Z1] != 0xff) && (mac_index_[LINEAR_AXIS_Z2] != 0xff)) {
    if (length_[LINEAR_AXIS_Z1] == length_[LINEAR_AXIS_Z2]) {
      axis_length[Z_AXIS] = length_[LINEAR_AXIS_Z1];      
    } else {
      goto length_err;
    }
  } else if ((mac_index_[LINEAR_AXIS_Z1] != 0xff) || (mac_index_[LINEAR_AXIS_Z2] != 0xff)) {
    axis_length[Z_AXIS] = mac_index_[LINEAR_AXIS_Z1] != 0xff ? length_[LINEAR_AXIS_Z1] : length_[LINEAR_AXIS_Z2];
  }
  if ((mac_index_[LINEAR_AXIS_Z3] != 0xff) && (length_[LINEAR_AXIS_Z3] != axis_length[Z_AXIS])) {
    goto length_err;
  }

  if (axis_length[X_AXIS] < 200) {
    X_MAX_POS = 167;
    X_DEF_SIZE = 160;
    MAGNET_X_SPAN = 114;
    X_HOME_DIR = 1;
    X_DIR = false;
    home_offset[X_AXIS] = s_home_offset[X_AXIS];
  } else if (axis_length[X_AXIS] < 300) {
    X_MAX_POS = 252;
    X_DEF_SIZE = 230;
    MAGNET_X_SPAN = 184;
    X_HOME_DIR = -1;
    X_DIR = true;
    home_offset[X_AXIS] = m_home_offset[X_AXIS];
  } else if (axis_length[X_AXIS] < 400) {
    X_MAX_POS = 345;
    X_DEF_SIZE = 320;
    MAGNET_X_SPAN = 274;
    X_HOME_DIR = -1;
    X_DIR = true;
    home_offset[X_AXIS] = l_home_offset[X_AXIS];
  } else {
    goto out;
  }

  if (axis_length[Y_AXIS] < 200) {
    Y_MAX_POS = 165;
    Y_DEF_SIZE = 160;
    MAGNET_Y_SPAN = 114;
    Y_HOME_DIR = 1;
    Y_DIR = false;
    home_offset[Y_AXIS] = s_home_offset[Y_AXIS];
  } else if (axis_length[Y_AXIS] < 300) {
    Y_MAX_POS = 260;
    Y_DEF_SIZE = 250;
    MAGNET_Y_SPAN = 204;
    Y_HOME_DIR = 1;
    Y_DIR = false;
    home_offset[Y_AXIS] = m_home_offset[Y_AXIS];
  } else if (axis_length[Y_AXIS] < 400) {
    Y_MAX_POS = 357;
    Y_DEF_SIZE = 340;
    MAGNET_Y_SPAN = 304;
    Y_HOME_DIR = 1;
    Y_DIR = false;
    home_offset[Y_AXIS] = l_home_offset[Y_AXIS];
  } else {
    goto out;
  }

if (axis_length[Z_AXIS] < 200) {
    Z_MAX_POS = 150;
    Z_DEF_SIZE = 145;
    Z_HOME_DIR = 1;
    Z_DIR = false;
    home_offset[Z_AXIS] = s_home_offset[Z_AXIS];
  } else if (axis_length[Z_AXIS] < 300) {
    Z_MAX_POS = 235;
    Z_HOME_DIR = 1;
    Z_DIR = false;
    Z_DEF_SIZE = 235;
    home_offset[Z_AXIS] = m_home_offset[Z_AXIS];
  } else if (axis_length[Z_AXIS] < 400) {
    Z_MAX_POS = 334;
    Z_HOME_DIR = 1;
    Z_DIR = false;
    Z_DEF_SIZE = 330; 
    home_offset[Z_AXIS] = l_home_offset[Z_AXIS];
  } else {
    goto out;
  }

  UpdateMachineDefines();
  endstops.reinit_hit_status();
  PollEndstop(LINEAR_AXIS_ALL);
  systemservice.ClearException(EHOST_MC, ETYPE_NO_HOST);
  
  for (uint8_t i = 0; i < LINEAR_AXIS_MAX; i++) {
    if (mac_index_[0] == 0xff || (mac_index_[i] != 0xff && length_[i] != length_[LINEAR_AXIS_X1])) {
      goto out;
    }
  }

  if (length_[LINEAR_AXIS_X1] == 0) {
    machine_size_ = MACHINE_SIZE_UNKNOWN;
  } else if (length_[LINEAR_AXIS_X1] < 200) {
    machine_size_ = MACHINE_SIZE_A150;
  } else if (length_[LINEAR_AXIS_X1] < 300) {
    machine_size_ = MACHINE_SIZE_A250;
  } else if (length_[LINEAR_AXIS_X1] < 400) {
    machine_size_ = MACHINE_SIZE_A350;
  } else {
    goto out;
  }
  
  return machine_size_;

length_err:
  SERIAL_ECHOLNPAIR("coaxial linear module length not equal\n");
  systemservice.ThrowException(EHOST_MC, ETYPE_NO_HOST);
  return (machine_size_ = MACHINE_SIZE_UNKNOWN);

out:
  machine_size_ = MACHINE_SIZE_UNKNOWN;
  SERIAL_ECHOLNPAIR("unknown machine size!\n");
  //systemservice.ThrowException(EHOST_MC, ETYPE_NO_HOST);
  return (machine_size_ = MACHINE_SIZE_UNKNOWN);
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
