#ifndef SNAPMAKER_UPGRADE_H_
#define SNAPMAKER_UPGRADE_H_

#include "../common/config.h"

#include MARLIN_SRC(core/macros.h)

#include "../hmi/event_handler.h"

#define VERSION_STRING_SIZE 32

#define UPGRADE_FW_OFFSET_FW_TYPE         ((uint32_t)(0))
#define UPGRADE_FW_OFFSET_START_MODULE_ID ((uint32_t)(1))
#define UPGRADE_FW_OFFSET_END_MODULE_ID   ((uint32_t)(3))
#define UPGRADE_FW_OFFSET_VERSION         ((uint32_t)(5))
#define UPGRADE_FW_OFFSET_FW_SIZE         ((uint32_t)(40))
#define UPGRADE_FW_OFFSET_CHECKSUM        ((uint32_t)(44))
#define UPGRADE_FW_OFFSET_FLAG            ((uint32_t)(48))
#define UPGRADE_FW_OFFSET_FW_CONTENT      ((uint32_t)(2048))

enum UpgradeStatus: uint8_t {
  UPGRADE_STA_IDLE = 0,
  UPGRADE_STA_RECV_FW,
  UPGRADE_STA_REBOOTING,
  UPGRADE_STA_UPGRADING_EM,
  UPGRADE_STA_INVALID
};

enum UpgradeTarget {
  UPGRADE_TARGET_MAIN_CONTROLLER,
  UPGRADE_TARGET_INTERNAL_MODULE,

  UPGRADE_TARGET_UNKNOWN
};

class UpgradeService {
  public:
    ErrCode StartUpgrade(SSTP_Event_t &event);
    ErrCode ReceiveFW(SSTP_Event_t &event);
    ErrCode EndUpgarde(SSTP_Event_t &event);
    ErrCode GetMainControllerVer(SSTP_Event_t &event);
    ErrCode CompareMCVer(SSTP_Event_t &event);
    ErrCode GetUpgradeStatus(SSTP_Event_t &event);
    ErrCode GetModuleVer(SSTP_Event_t &event);

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
    UpgradeTarget target_ = UPGRADE_TARGET_UNKNOWN;

    uint16_t timeout_ = 0;
    uint16_t req_pkt_counter_ = 0;
    uint16_t pre_pkt_counter_ = 0;
    uint32_t received_fw_size_ = 0;
};


extern UpgradeService upgrade;

#endif  // #ifndef SNAPMAKER_UPGRADE_H_
