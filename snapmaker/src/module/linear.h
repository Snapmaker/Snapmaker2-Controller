#ifndef SNAPMAKER_LINEAR_H_
#define SNAPMAKER_LINEAR_H_

#include "module_base.h"
#include "can_host.h"

// to be compact with EndstopEnum
enum LinearAxisType{
  LINEAR_AXIS_X1,
  LINEAR_AXIS_Y1,
  LINEAR_AXIS_Z1,
  LINEAR_AXIS_X2,
  LINEAR_AXIS_Y2,
  LINEAR_AXIS_Z2,
  LINEAR_AXIS_Z3,

  LINEAR_AXIS_MAX,
  LINEAR_AXIS_ALL = LINEAR_AXIS_MAX,
  LINEAR_AXIS_UNKNOWN = LINEAR_AXIS_MAX
};


enum MachineSize {
  MACHINE_SIZE_UNKNOWN,

  MACHINE_SIZE_A150,
  MACHINE_SIZE_A250,
  MACHINE_SIZE_A350
};

class Linear: public ModuleBase  {
  public:
    Linear (): ModuleBase(MODULE_DEVICE_ID_LINEAR) {
      for (int i = 0; i < LINEAR_AXIS_MAX; i++) {
        mac_index_[i]   = MODULE_MAC_INDEX_INVALID;
        endstop_msg_[i] = MODULE_MESSAGE_ID_INVALID;
        length_[i]      = 0;
      }
      machine_size_ = MACHINE_SIZE_UNKNOWN;
      endstop_      = 0xFFFFFFFF;
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);

    ErrCode PollEndstop(LinearAxisType axis);

    MachineSize UpdateMachineSize();

    ErrCode SetLength(SSTP_Event_t &event) { return SetLengthOrLead(event, MODULE_EXT_CMD_LINEAR_LENGTH_REQ); }
    ErrCode GetLength(SSTP_Event_t &event) { return GetLengthOrLead(event, MODULE_EXT_CMD_LINEAR_LENGTH_REQ); }
    ErrCode SetLead(SSTP_Event_t &event) { return SetLengthOrLead(event, MODULE_EXT_CMD_LINEAR_LEAD_REQ); }
    ErrCode GetLead(SSTP_Event_t &event) { return GetLengthOrLead(event, MODULE_EXT_CMD_LINEAR_LEAD_REQ); }

    uint16_t length(LinearAxisType axis) {
      if (axis < LINEAR_AXIS_MAX)
        return length_[axis];
      else
        return 0;
    }


    uint32_t mac(uint8_t sub_index = 0) {
      return canhost.mac(mac_index_[sub_index]);
    }


    void SetEndstopBit(uint8_t index, uint8_t state) {
      if (state) {
        endstop_ |= 1<<index;
      }
      else {
        endstop_ &= ~(1<<index);
      }
    }
    bool GetEndstopBit(uint8_t index) { return (endstop_>>index & 0x1);}

    uint32_t endstop() { return endstop_; }

    MachineSize machine_size() { return machine_size_; }

  private:
    LinearAxisType DetectAxis(MAC_t &mac, uint8_t &endstop);

    ErrCode SetLengthOrLead(SSTP_Event_t &event, uint8_t ext_cmd);
    ErrCode GetLengthOrLead(SSTP_Event_t &event, uint8_t ext_cmd);

  private:
    uint8_t       mac_index_[LINEAR_AXIS_MAX];
    uint16_t      length_[LINEAR_AXIS_MAX];
    uint16_t      lead_[LINEAR_AXIS_MAX];

    message_id_t  endstop_msg_[LINEAR_AXIS_MAX];
    uint32_t      endstop_;

    MachineSize   machine_size_;
};


extern Linear linear;

#endif  // #ifndef SNAPMAKER_LINEAR_H_
