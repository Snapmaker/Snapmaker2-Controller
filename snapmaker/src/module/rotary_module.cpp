#include "rotary_module.h"
#include "src/inc/MarlinConfig.h"

RotaryModule rotaryModule;

/**
 * Initialize rotary module.
 */
ErrCode RotaryModule::Init(MAC_t &mac, uint8_t mac_index) {
  CanExtCmd_t cmd;
  uint8_t     buffer[16];
  cmd.mac    = mac;
  cmd.data   = buffer;

  OUT_WRITE(E1_DIR_PIN, LOW);
  WRITE(B_DIR_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));

  cmd.data[MODULE_EXT_CMD_INDEX_ID]   = MODULE_EXT_CMD_CONFIG_REQ;
  cmd.data[MODULE_EXT_CMD_INDEX_DATA] = B_AXIS;
  cmd.length = 2;
  if (canhost.SendExtCmdSync(cmd, 500) == E_SUCCESS) {
    if (cmd.data[MODULE_EXT_CMD_INDEX_DATA] == 1) {
      status(ROTATE_ONLINE);
      SERIAL_ECHOLN("Rotary module B detected.");
    } else {
      status(ROTATE_UNUSABLE);
      SERIAL_ECHOLN("Rotary module B unusable.");
    }
  } else {
    status(ROTATE_OFFLINE);
    SERIAL_ECHOLN("Rotary module B not found.");
  }

  WRITE(B_DIR_PIN, LOW);
  return E_SUCCESS;
}

