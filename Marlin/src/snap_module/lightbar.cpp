#include "lightbar.h"
#include "../module/CanModule.h"
#include "../module/CanDefines.h"
#include "../SnapScreen/HMISC20.h"

LightBar lightbar;

/*
 * set the light bar color
 * param:
 *  r - red
 *  g - green
 *  b - blue
 *       0 indicates turn off, 100 indicates max
 * return:
 *  0 - error code
 */
ErrCode LightBar::set_led(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t br;
  uint8_t Buff[8];

  if (mode_ == LB_MODE_LIGHTING)
    br = br_light_;
  else
    br = br_status_;

  r = (uint8_t)((r * br) / 100);
  g = (uint8_t)((g * br) / 100);
  b = (uint8_t)((b * br) / 100);

  // TODO: need to send r,g,b to lightbar module by CAN
  Buff[0] = 1;
  Buff[1] = r;
  Buff[2] = g;
  Buff[3] = b;
  CanModules.SetFunctionValue(EXTEND_CAN_NUM, FUNC_SET_LIGHT_COLOR, Buff, 4);
  return E_SUCCESS;
}

/*
 * check whether the light bar is online
 * return:
 *    0     - online
 *    other - offline
 */
uint8_t LightBar::check_online(void) {
  if(CanModules.SearchModule(MODULE_LIGHT) > 0)
    return 0;
  else
    return 1;
}

/*
 * initialize the member var
 */
void LightBar::init() {
  state_ = LB_STATE_STANDBY;
  mode_ = LB_MODE_STATUS;
  door_sta_ = LB_DOORSTA_CLOSE;

  br_light_ = 100;
  br_status_ = DEFAULT_STATUS_BRIGHTNESS;

  online_ = check_online();
}

/*
 * turn on the light bar
 * return:
 *    see error code defination
 */
ErrCode LightBar::turn_on() {
  if (switch_sta_ == LB_SWITCH_ON)
    return E_SUCCESS;

  switch_sta_ = LB_SWITCH_ON;

  mode_ = LB_MODE_STATUS;
  return set_led_per_state();
}

/*
 * turn off the light bar
 * return:
 *    see error code defination
 */
ErrCode LightBar::turn_off() {
  if (switch_sta_ == LB_SWITCH_OFF)
    return E_SUCCESS;

  switch_sta_ = LB_SWITCH_OFF;

  return set_led(COLOR_TURNOFF);
}

/*
 * set led color per the state
 * return:
 *    see error code defination
 */
ErrCode LightBar::set_led_per_state() {
  uint8_t ret = E_SUCCESS;
  switch (state_) {
  case LB_STATE_ERROR:
    ret = set_led(COLOR_ERROR);
    break;

  case LB_STATE_STANDBY:
    ret = set_led(COLOR_STANBY);
    break;

  case LB_STATE_FINISH:
    ret = set_led(COLOR_FINISH);
    break;

  case LB_STATE_WORKING:
    ret = set_led(COLOR_WORKING);
    break;

  default:
    break;
  }

  return ret;
}

/*
 * set mode of light, only the screen can change the mode
 * param:
 *    m - the mode will be setup
 * return:
 *    see error code defination
 */
ErrCode LightBar::set_mode(LightBarMode m, bool sync) {
  uint8_t ret = E_SUCCESS;

  if (!online_)
    return E_NO_RESRC;

  if (m >= LB_MODE_INVALID)
    return E_PARAM;

  // current mode is same as new mode
  if (m == mode_)
    return E_SUCCESS;

  mode_ = m;

  switch (m)
  {
  case LB_MODE_STATUS:
    ret = set_led_per_state();
    break;

  case LB_MODE_LIGHTING:
    ret = set_led(COLOR_LIGHTING);
    break;

  default:
    break;
  }

  return ret;
}

/* set state of light, when working state is changed, for example:
 * standy  -> working
 * working -> pause(standby)
 * working -> finish
 * need to call this API to sync with light
 * param:
 *    s - the new state
 * return:
 *    see error code defination
 */
ErrCode LightBar::set_state(LightBarState s) {
  uint8_t ret;

  if (!online_)
    return E_NO_RESRC;

  if (s > LB_STATE_INVALID)
    return E_PARAM;

  /* even though current mode is LB_MODE_LBING, if new error happened,
   * we need to set the mode to LB_MODE_STATUS, and show error color to users.
   * otherwise we just save state for other new system state
   */
  if (mode_ == LB_MODE_LIGHTING) {
    state_ = s;

    if (s == LB_STATE_ERROR) {
      mode_ = LB_MODE_STATUS;

      ret = set_led(COLOR_ERROR);
      if (ret  != E_SUCCESS)
        return ret;

      // we changed the mode, need to tell screen
      sync2host();
    }

    return E_SUCCESS;
  }

  // arrive here, current mode is LB_MODE_STATUS
  if (s == state_)
    return E_SUCCESS;

  state_ = s;

  sync2host();

  return set_led_per_state();
}

/* when door state is changed, need to call this API to sync with light
 * param:
 *    ds - the new door state
 * return:
 *    see error code defination
 */
ErrCode LightBar::set_door_sta(LightBarDoorSta ds) {
  uint8_t ret;

  if (!online_)
    return E_NO_RESRC;

  if (ds > LB_DOORSTA_INVALID)
    return E_PARAM;

  if (ds == door_sta_)
    return E_SUCCESS;

  door_sta_ = ds;

  if (ds == LB_DOORSTA_OPEN) {
    /* here need to check current state. if state = error,
     * cannot change the color; but we can change color to white for other state
     */
    if ((mode_ = LB_MODE_LIGHTING) || (state_ == LB_STATE_ERROR))
      return E_SUCCESS;

    mode_ = LB_MODE_LIGHTING;

    ret = set_led(COLOR_LIGHTING);
    if (ret  != E_SUCCESS)
      return ret;
  }
  else {
    /* close door */
    if ((mode_ = LB_MODE_STATUS) || (state_ == LB_STATE_ERROR))
      return E_SUCCESS;

    mode_ = LB_MODE_STATUS;

    ret = set_led_per_state();
    if (ret  != E_SUCCESS)
      return ret;
  }

  sync2host();

  return E_SUCCESS;
}

/* set the brightness of lighting
 * param:
 *    br   - new brightness
 *    mode - which mode of brightness will be set
 * return:
 *    see error code defination
 */
ErrCode LightBar::set_brightness(uint8_t br) {
  if (br > MAX_BRIGHTNESS || !br)
    return E_PARAM;

  br_light_ = br;

  if (mode_ == LB_MODE_LIGHTING)
    set_led(COLOR_LIGHTING);

  return E_SUCCESS;
}

/* sync our state to host(screen)
 * for now we push mode, state, and brightness to host
 */
extern HMI_SC20 SC20HMI;
void LightBar::sync2host(void) {
  char ret[] = {EID_ADDON_OP_RESP, CMD_LB_QUERY_STATE, 0, 0, 0, 0};

  ret[2] = (char)online_;
  ret[3] = (char)switch_sta_;
  ret[4] = (char)mode_;
  ret[5] = (char)br_light_;

  SC20HMI.PackedProtocal(ret, 6);
}
