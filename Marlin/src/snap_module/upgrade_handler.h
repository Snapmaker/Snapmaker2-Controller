#ifndef UPGRADE_HANDLER_H_
#define UPGRADE_HANDLER_H_

#include "event_handler.h"

enum UpgradeStatus: uint8_t {
  UPGRADE_STA_IDLE = 0,
  UPGRADE_STA_IS_UPGRADING,

  UPGRADE_STA_INVALID
};

class UpgradeHandler {
  public:
    ErrCode StartUpgrade(Event_t &event);
    ErrCode ReceiveFW(Event_t &event);
    ErrCode EndUpgarde(Event_t &event);
    ErrCode GetMainControllerVer(Event_t &event);
    ErrCode CompareMCVer(Event_t &event);
    ErrCode GetUpgradeStatus(Event_t &event);
    ErrCode GetModuleVer(Event_t &event);

  private:
    ErrCode RequestNextPacket();

  private:
    static const uint16_t max_packet_ = MARLIN_CODE_SIZE / 512;

    UpgradeStatus upgrade_status_ = UPGRADE_STA_IDLE;
    uint16_t req_pkt_counter_ = 0;
    uint32_t received_fw_size_ = 0;
};


extern UpgradeHandler upgrade;

#endif  //#ifndef UPGRADE_HANDLER_H_
