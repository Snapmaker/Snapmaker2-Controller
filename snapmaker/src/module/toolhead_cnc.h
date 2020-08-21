#ifndef SNAPMAKER_TOOLHEAD_CNC_H_
#define SNAPMAKER_TOOLHEAD_CNC_H_

#include "module_base.h"
#include "can_host.h"

class ToolHeadCNC: public ModuleBase {
  public:
		ToolHeadCNC(): ModuleBase(MODULE_DEVICE_ID_CNC) {}

    ErrCode Init(MAC_t &mac, uint8_t mac_index);

    ErrCode ChangePowerImmediately(uint8_t power);

    ErrCode TurnOn();
    ErrCode TurnOff();

    bool IsOnline(uint8_t sub_index = 0);

    uint32_t mac(uint8_t sub_index = 0) {
      if (sub_index > 0)
        return MODULE_MAC_ID_INVALID;

      return canhost.mac(mac_index_);
    }

    uint16_t rpm() { return rpm_; }
    void rpm(uint16_t rpm) { rpm_ = rpm; }

    float power() { return power_; }
    void power(float power) { power_ = power; }

  private:
    uint8_t  mac_index_;
    uint8_t  power_;
    uint16_t rpm_;

    message_id_t msg_id_set_speed_;
};


extern ToolHeadCNC cnc;

#endif  // #ifndef TOOLHEAD_LASER_H_