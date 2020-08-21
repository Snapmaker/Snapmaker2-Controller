#ifndef SNAPMAKER_ENCLOSURE_H_
#define SNAPMAKER_ENCLOSURE_H_

#include "module_base.h"
#include "can_host.h"


#define ENCLOSURE_LIGHT_POWER_DEFAULT (255)
#define ENCLOSURE_FAN_SPEED_DEFAULT   (255)

 #define  ENCLOSURE_DOOR_STATE_OPEN   (1)
 #define  ENCLOSURE_DOOR_STATE_CLOSED (0)


enum EnclosureEventState {
  ENCLOSURE_EVENT_STATE_IDLE,
  ENCLOSURE_EVENT_STATE_OPENED,
  ENCLOSURE_EVENT_STATE_HANDLED_OPEN,
  ENCLOSURE_EVENT_STATE_CLOSED,

  ENCLOSURE_EVENT_STATE_INVALID
};


class Enclosure: public ModuleBase {
  public:
    Enclosure(): ModuleBase(MODULE_DEVICE_ID_ENCLOSURE) {
      door_state_  = ENCLOSURE_DOOR_STATE_CLOSED;
      mac_index_   = MODULE_MAC_INDEX_INVALID;
      enabled_     = false;
      fan_speed_   = 0;
      brightness_  = 0;
      event_state_ = ENCLOSURE_EVENT_STATE_IDLE;
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);
    bool    IsOnline(uint8_t sub_index = 0) { return (mac_index_ != MODULE_MAC_INDEX_INVALID); }

    ErrCode SetLightBar(uint8_t brightness);
    ErrCode SetFanSpeed(uint8_t speed);

    void Disable();
    void Enable();
    void ReportStatus();
    void Process();

    void PollDoorState();

    uint32_t mac(uint8_t sub_index = 0) { return canhost.mac(mac_index_); }

    uint8_t door_state() { return door_state_; };
    void    door_state(uint8_t state) { door_state_ = state; }

  private:
    void HandleDoorOpened();
    void HandleDoorClosed();


  private:
    uint8_t mac_index_;

    uint8_t fan_speed_;
    uint8_t brightness_;
    uint8_t door_state_;

    uint8_t event_state_;
    bool    enabled_;
};


extern Enclosure enclosure;

#endif
