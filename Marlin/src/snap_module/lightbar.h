#ifndef LIGHTBAR_H_
#define LIGHTBAR_H_

#include <stdio.h>
#include <stdlib.h>

#include "../core/macros.h"

#include "error.h"
#include "snap_cmd.h"

// define some colors are in common use
// it is in R G B sequence
// Brightness is 100%
#define WHITE       255, 255, 255
#define RED         255, 0, 0
#define YELLOW      255, 255, 0
#define GREEN       0, 255, 0
#define ORANGE      255, 128, 0
#define BLACK       0,0,0

// default brightness for status is 50%
#define DEFAULT_STATUS_BRIGHTNESS   (50)

#define MAX_BRIGHTNESS  (100)

#define COLOR_STANBY    YELLOW
#define COLOR_ERROR     RED
#define COLOR_WORKING   GREEN
#define COLOR_FINISH    ORANGE
#define COLOR_LIGHTING  WHITE
#define COLOR_TURNOFF   BLACK

// corresponding to system state
enum LightBarState: uint8_t {
  LB_STATE_STANDBY,
  LB_STATE_WORKING,
  LB_STATE_ERROR,
  LB_STATE_FINISH,

  LB_STATE_INVALID
};

enum LightBarMode: uint8_t  {
  LB_MODE_STATUS,
  LB_MODE_LIGHTING,

  LB_MODE_INVALID
};

enum LightBarDoorSta: uint8_t  {
  LB_DOORSTA_CLOSE,
  LB_DOORSTA_OPEN,

  LB_DOORSTA_INVALID
};

enum LightSwitchSta: uint8_t {
  LB_SWITCH_OFF,
  LB_SWITCH_ON,
  LB_SWITCH_INVALID
};

class LightBar {
private:
  LightBarState   state_;
  LightBarMode    mode_;
  LightBarDoorSta door_sta_;
  LightSwitchSta  switch_sta_;

  // indicating if light bar is online, 1 -> offling, 0 -> online
  uint8_t online_;

  uint8_t br_light_;
  uint8_t br_status_;
  // change color per new state
  ErrCode FORCE_INLINE set_led_per_state();

  ErrCode set_led(uint8_t r, uint8_t g, uint8_t b);

public:
  void init(void);

  void sync2host(void);

  uint8_t check_online(void);
  uint8_t is_online(void) { return online_; }

  // turn on / off the light bar, only called by screen
  ErrCode turn_on(void);
  ErrCode turn_off(void);
  // check current is on or off
  LightSwitchSta switch_sta(void) { return switch_sta_; }

  // set working mode of light bar, maybe called by screen or door checking
  ErrCode set_mode(LightBarMode m, bool sync = false);
  LightBarMode get_mode(void) { return mode_; }

  // set current state of light bar
  ErrCode set_state(LightBarState s);
  LightBarState get_state(void) { return state_; }

  ErrCode set_door_sta(LightBarDoorSta ds);

  // set brightness, now only support lighting mode
  ErrCode set_brightness(uint8_t br);
  uint8_t get_brightness(void) { return br_light_; }
};

extern LightBar lightbar;

#endif // #ifndef LIGHTBAR_H_
