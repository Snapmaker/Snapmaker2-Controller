#ifndef SNAPMAKER_TOOLHEAD_3DP_H_
#define SNAPMAKER_TOOLHEAD_3DP_H_

#include "module_base.h"
#include "can_host.h"

#include "../common/config.h"


#define TOOLHEAD_3DP_FAN_MAX    (2)

#define EXTRUDERS 1

class ToolHead3DP: public ModuleBase {
  public:
		ToolHead3DP(): ModuleBase(MODULE_DEVICE_ID_3DP) {
      for (int i = 0; i < EXTRUDERS; i++) {
        cur_temp_[i]  = 0;
        filament_state_[i] = 0;
      }

      for (int i = 0; i < TOOLHEAD_3DP_FAN_MAX; i++) {
        fan_speed_[i] = 0;
      }
      mac_index_      = MODULE_MAC_INDEX_INVALID;
      probe_state_    = 0;

      timer_in_process_ = 0;
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);

    ErrCode SetFan(uint8_t fan_index, uint8_t speed, uint8_t delay_time=0);
    ErrCode SetPID(uint8_t index, float value, uint8_t extrude_index=0);
    ErrCode SetHeater(uint16_t target_temp, uint8_t extrude_index=0);

    void Process();

    bool IsOnline(uint8_t head_index=0) { return mac_index_ != MODULE_MAC_INDEX_INVALID; };

    uint32_t mac(uint8_t head_index=0) {
      return canhost.mac(mac_index_);
    }

    uint8_t fan_speed(uint8_t fan_index) {
      if (fan_index >= TOOLHEAD_3DP_FAN_MAX)
        return 0;

      return fan_speed_[fan_index];
    }


    void probe_state(uint8_t state) {
      probe_state_ = state;
    }
    uint8_t probe_state() {
      return probe_state_;
    }

    void filament_state(uint8_t state, uint8_t extrude_index=0) {
      if (extrude_index >= EXTRUDERS)
        return;

      filament_state_[extrude_index] = state;
    }
    uint8_t filament_state(uint8_t extrude_index=0) {
      if (extrude_index >= EXTRUDERS)
        return 0;

      return filament_state_[extrude_index];
    }

    void SetTemp(uint16_t temp, uint8_t extrude_index=0) {
      if (extrude_index >= EXTRUDERS)
        return;

      cur_temp_[extrude_index] = temp;
    }
    uint16_t GetTemp(uint8_t extrude_index=0) {
      if (extrude_index >= EXTRUDERS)
        return 0;

      return cur_temp_[extrude_index];
    }

  private:
    uint8_t mac_index_;

    uint16_t timer_in_process_;

    uint16_t cur_temp_[EXTRUDERS];
    uint8_t  fan_speed_[TOOLHEAD_3DP_FAN_MAX];

    uint8_t probe_state_;
    uint8_t filament_state_[EXTRUDERS];
};


extern ToolHead3DP printer;

#endif  // #ifndef TOOLHEAD_3DP_H_
