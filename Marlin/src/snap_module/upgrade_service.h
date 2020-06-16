#ifndef UPGRADE_HANDLER_H_
#define UPGRADE_HANDLER_H_

#include "event_handler.h"
#include "../core/macros.h"

#define VERSION_STRING_SIZE 32

enum UpgradeStatus: uint8_t {
  UPGRADE_STA_IDLE = 0,
  UPGRADE_STA_RECV_FW,
  UPGRADE_STA_REBOOTING,
  UPGRADE_STA_UPGRADING_EM,
  UPGRADE_STA_INVALID
};

class UpgradeService {
  public:
    ErrCode StartUpgrade(Event_t &event);
    ErrCode ReceiveFW(Event_t &event);
    ErrCode EndUpgarde(Event_t &event);
    ErrCode GetMainControllerVer(Event_t &event);
    ErrCode CompareMCVer(Event_t &event);
    ErrCode GetUpgradeStatus(Event_t &event);
    ErrCode GetModuleVer(Event_t &event);

    ErrCode SendModuleUpgradeStatus(uint8_t sta);
    ErrCode SendModuleVer(uint32_t mac, char ver[VERSION_STRING_SIZE]);

    void CheckIfUpgradeModule();

    void Check(void);

    UpgradeStatus GetState() { return upgrade_state_; }
    void SetState(UpgradeStatus sta) {
      if (sta < UPGRADE_STA_INVALID)
        upgrade_state_ = sta;
    }

  private:
    ErrCode RequestNextPacket();

  private:
    static const uint16_t max_packet_ = MARLIN_CODE_SIZE / 512;

    UpgradeStatus upgrade_state_ = UPGRADE_STA_IDLE;
    uint16_t  timeout_ = 0;
    uint16_t req_pkt_counter_ = 0;
    uint16_t pre_pkt_counter_ = 0;
    uint32_t received_fw_size_ = 0;
};


extern UpgradeService upgrade;

#endif  //#ifndef UPGRADE_HANDLER_H_
