#ifndef LIGHTBAR_H_
#define LIGHTBAR_H_

#include <stdio.h>
#include <stdlib.h>

#include "../core/macros.h"

// define some colors are in common use
// it is in R G B sequence
// Brightness is 100%
#define WHITE       255, 255, 255
#define RED         255, 0, 0
#define YELLOW      255, 255, 0
#define GREEN       0, 255, 0
#define ORANGE      255, 128, 0

// default brightness for status is 50%
#define DEFAULT_STATUS_BRIGHTNESS   (50)

#define MAX_BRIGHTNESS  (100)

#define COLOR_STANBY    YELLOW
#define COLOR_ERROR     RED
#define COLOR_WORKING   GREEN
#define COLOR_FINISH    ORANGE
#define COLOR_LIGHTING  WHITE

// corresponding to system state
enum LightBarState {
  LB_STATE_STANDBY,
  LB_STATE_WORKING,
  LB_STATE_ERROR,
  LB_STATE_FINISH,

  LB_STATE_INVALID
};

enum LightBarMode {
  LB_MODE_STATUS,
  LB_MODE_LIGHTING,

  LB_MODE_INVALID
};

enum LightBarDoorSta {
  LB_DOORSTA_OPEN,
  LB_DOORSTA_CLOSE,

  LB_DOORSTA_INVALID
};

class LightBar {
private:
  uint8_t state_;
  uint8_t mode_;
  uint8_t door_sta_;
  uint8_t online_;

  uint8_t br_light_;
  uint8_t br_status_;
  // change color per new state
  uint8_t FORCE_INLINE set_led_per_state();

  uint8_t set_led(uint8_t r, uint8_t g, uint8_t b);

public:
  LightBar() {};

  void init();

  uint8_t check_online(void);

  uint8_t set_mode(uint8_t m);

  uint8_t get_mode() { return mode_; }

  uint8_t set_state(uint8_t s);

  uint8_t get_state() { return state_; }

  uint8_t set_door_sta(uint8_t ds);

  uint8_t cmd_handle(char *cmd);

  uint8_t set_brightness(uint8_t br, uint8_t mode);
};

extern LightBar lightbar;

#endif // #ifndef LIGHTBAR_H_
