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
    Linear (ModuleDeviceID id): ModuleBase(id) {
      for (int i = 0; i < LINEAR_AXIS_MAX; i++) {
        mac_index_[i]   = MODULE_MAC_INDEX_INVALID;
        endstop_msg_[i] = MODULE_MESSAGE_ID_INVALID;
        length_[i]      = 0;
      }
      machine_size_ = MACHINE_SIZE_UNKNOWN;
      endstop_      = 0xFFFFFFFF;
    }

    ErrCode Init(MAC_t &mac, uint8_t mac_index);

    ErrCode CheckModuleType();

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
    void reset_axis_steps_per_unit(void);

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
    float axis_steps_per_unit[5] = DEFAULT_TMC_AXIS_STEPS_PER_UNIT;
};

extern Linear linear;
extern Linear linear_tmc;
extern Linear *linear_p;

#endif  // #ifndef SNAPMAKER_LINEAR_H_
