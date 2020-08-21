#include "../common/config.h"
#include "linear.h"

#include "../service/system.h"

#include MARLIN_SRC(inc/MarlinConfig.h)
#include MARLIN_SRC(module/endstops.h)


Linear linear;


LinearAxisType Linear::DetectAxis(MAC_t &mac, uint16_t &length, uint8_t &endstop) {
  CanExtCmd_t cmd = {mac};
  uint8_t     buffer[16];

  int i;
  int pins[3] = {X_DIR_PIN, Y_DIR_PIN, Z_DIR_PIN};

  cmd.data   = buffer;
  cmd.length = 1;

  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_CONFIG_REQ;

  WRITE(X_DIR_PIN, LOW);
  WRITE(Y_DIR_PIN, LOW);
  WRITE(Z_DIR_PIN, LOW);

  for (i = LINEAR_AXIS_X1; i <= LINEAR_AXIS_Z1; i++)  {
    WRITE(pins[i], HIGH);

    vTaskDelay(portTICK_PERIOD_MS * 10);

    cmd.data[MODULE_EXT_CMD_INDEX_DATA] = i;

    if (canhost.SendExtCmdSync(cmd, 500) == E_SUCCESS) {
      if (cmd.data[MODULE_EXT_CMD_INDEX_DATA] == 1) {
        length = (cmd.data[MODULE_EXT_CMD_INDEX_DATA + 1]<<8 | cmd.data[MODULE_EXT_CMD_INDEX_DATA + 2]);
        endstop = cmd.data[MODULE_EXT_CMD_INDEX_DATA + 3];
        goto out;
      }
    }
  }

  // if nobody tell us it detected low level from dir signal
  // we cannot recognize what kind of axis it is
  i = LINEAR_AXIS_UNKNOWN;

out:
  WRITE(X_DIR_PIN, LOW);
  WRITE(Y_DIR_PIN, LOW);
  WRITE(Z_DIR_PIN, LOW);

  return (LinearAxisType)i;
}


static void LinearCallbackEndstopX1(uint8_t *cmd, uint8_t length) {
  switch (linear.machine_size())
  {
  case MACHINE_SIZE_A250:
  case MACHINE_SIZE_A350:
    linear.SetEndstop(X_MIN, cmd[0]);
    break;

  case MACHINE_SIZE_A150:
  default:
    linear.SetEndstop(X_MAX, cmd[0]);
    break;
  }
}


static void LinearCallbackEndstopY1(uint8_t *cmd, uint8_t length) {
  linear.SetEndstop(Y_MAX, cmd[0]);
}

static void LinearCallbackEndstopY2(uint8_t *cmd, uint8_t length) {
  linear.SetEndstop(Y_MAX, cmd[0]);
}

static void LinearCallbackEndstopZ1(uint8_t *cmd, uint8_t length) {
  linear.SetEndstop(Z_MAX, cmd[0]);
}

static void LinearCallbackEndstopZ2(uint8_t *cmd, uint8_t length) {
  linear.SetEndstop(Z_MAX, cmd[0]);
}


ErrCode Linear::Init(MAC_t &mac, uint8_t mac_index) {
  uint8_t   type;
  uint8_t   endstop;
  uint16_t  length;

  CanExtCmd_t cmd;
  uint8_t     func_buffer[16];

  Function_t    function;
  message_id_t  message_id[4];

  CanStdCmdCallback_t cb;
  int i;

  // need to check what kind of axis it is before we register function id
  type = DetectAxis(mac, length, endstop);
  if (type >= LINEAR_AXIS_MAX)
    return E_FAILURE;

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
  length_[type]    = mac_index;

  cmd.mac    = mac;
  cmd.data   = func_buffer;
  cmd.length = 1;

  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_GET_FUNCID_REQ;

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

  return canhost.BindMessageID(func_buffer, message_id);
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

    message.id = endstop_msg_[axis];
    canhost.SendStdCmd(message);
    vTaskDelay(portTICK_PERIOD_MS * 10);
  }

  return E_SUCCESS;
}


MachineSize Linear::UpdateMachineSize() {
  if (length_[LINEAR_AXIS_X1] != length_[LINEAR_AXIS_Y1] ||
      length_[LINEAR_AXIS_Y1] != length_[LINEAR_AXIS_Z1]) {

    systemservice.ThrowException(EHOST_MC, ETYPE_NO_HOST);
    return (machine_size_ = MACHINE_SIZE_UNKNOWN);
  }

  systemservice.ClearException(EHOST_MC, ETYPE_NO_HOST);

  if (length_[LINEAR_AXIS_X1] < 200) {
    X_MAX_POS = 167;
    Y_MAX_POS = 165;
    Z_MAX_POS = 150;
    X_HOME_DIR = 1;
    X_DIR = false;
    Y_HOME_DIR = 1;
    Y_DIR = false;
    Z_HOME_DIR = 1;
    Z_DIR = false;

    LOOP_XYZ(i) home_offset[i] = s_home_offset[i];

    X_DEF_SIZE = 160;
    Y_DEF_SIZE = 160;
    Z_DEF_SIZE = 145;

    MAGNET_X_SPAN = 114;
    MAGNET_Y_SPAN = 114;

    return (machine_size_ = MACHINE_SIZE_A150);
  }

  if (length_[LINEAR_AXIS_X1] < 300) {
    X_MAX_POS = 252;
    Y_MAX_POS = 260;
    Z_MAX_POS = 235;
    X_HOME_DIR = -1;
    X_DIR = true;
    Y_HOME_DIR = 1;
    Y_DIR = false;
    Z_HOME_DIR = 1;
    Z_DIR = false;

    LOOP_XYZ(i) home_offset[i] = m_home_offset[i];

    X_DEF_SIZE = 230;
    Y_DEF_SIZE = 250;
    Z_DEF_SIZE = 235;

    MAGNET_X_SPAN = 184;
    MAGNET_Y_SPAN = 204;

    return (machine_size_ = MACHINE_SIZE_A250);
  }

  if (length_[LINEAR_AXIS_X1] < 400) {
    X_MAX_POS = 345;
    Y_MAX_POS = 357;
    Z_MAX_POS = 334;
    X_HOME_DIR = -1;
    X_DIR = true;
    Y_HOME_DIR = 1;
    Y_DIR = false;
    Z_HOME_DIR = 1;
    Z_DIR = false;

    LOOP_XYZ(i) home_offset[i] = l_home_offset[i];

    X_DEF_SIZE = 320;
    Y_DEF_SIZE = 340;
    Z_DEF_SIZE = 330; // unused & spec is lager than actual size.  334 - 6 = 328?

    MAGNET_X_SPAN = 274;
    MAGNET_Y_SPAN = 304;

    return (machine_size_ = MACHINE_SIZE_A350);
  }
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


