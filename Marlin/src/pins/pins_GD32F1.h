/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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
 *
 */


/**
 * 21017 Victor Perez Marlin for stm32f1 test
 */

#define DEFAULT_MACHINE_NAME "GD32F305VGT6"
#define BOARD_NAME "Snapmaker V2.0"

// Ignore temp readings during develpment.
#define BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE

#define SM2_8PIN_PORT_COUNT 6

enum {
  PORT_8PIN_1,
  PORT_8PIN_2,
  PORT_8PIN_3,
  PORT_8PIN_4,
  PORT_8PIN_5,
  PORT_8PIN_6,
  PORT_8PIN_INVALID
};

// 8 PIN port from top to bottom
#define PORT_TO_STEP_PIN { /*STEP  DIR   ENABLE*/ \
  /* PORT_8PIN_1 */         {PE14, PB10, PB11}, \
  /* PORT_8PIN_2 */         {PE13, PC10, PC11}, \
  /* PORT_8PIN_3 */         {PC6 , PD15, PA3 }, \
  /* PORT_8PIN_4 */         {PB4 , PB3,  PB5 }, \
  /* PORT_8PIN_5 */         {PB7 , PB6,  PA2 }, \
  /* PORT_8PIN_6 */         {PA1 , PC12, PD2 }, \
  }

//                             X            Y            Z            B               E
#define DEFAULT_AXIS_TO_PORT {PORT_8PIN_3, PORT_8PIN_4, PORT_8PIN_5, PORT_8PIN_6, PORT_8PIN_1}

//
// Limit Switches
//
#define U_MIN_PIN         -1
#define V_MIN_PIN         -1
#define W_MIN_PIN         -1
#define X_STOP_PIN        PE12
#define Y_STOP_PIN        PE12
#define Z_STOP_PIN        PE12
#define Z_MIN_PROBE_PIN   -1 

//
// Steppers
//
extern uint8_t x_step_pin, x_dir_pin, x_enable_pin;
extern uint8_t y_step_pin, y_dir_pin, y_enable_pin;
extern uint8_t z_step_pin, z_dir_pin, z_enable_pin;
extern uint8_t b_step_pin, b_dir_pin, b_enable_pin;
extern uint8_t e0_step_pin, e0_dir_pin, e0_enable_pin;
#define X_STEP_PIN         x_step_pin
#define X_DIR_PIN          x_dir_pin
#define X_ENABLE_PIN       x_enable_pin

#define Y_STEP_PIN         y_step_pin
#define Y_DIR_PIN          y_dir_pin
#define Y_ENABLE_PIN       y_enable_pin

#define Z_STEP_PIN         z_step_pin
#define Z_DIR_PIN          z_dir_pin
#define Z_ENABLE_PIN       z_enable_pin

#define B_STEP_PIN         b_step_pin    // P6 - Add-on 2
#define B_DIR_PIN          b_dir_pin
#define B_ENABLE_PIN       b_enable_pin

#define E0_STEP_PIN        e0_step_pin
#define E0_DIR_PIN         e0_dir_pin
#define E0_ENABLE_PIN      e0_enable_pin

/**
 * TODO: Currently using same Enable pin to all steppers.
 */

#define E1_STEP_PIN        PE13
#define E1_DIR_PIN         PC10
#define E1_ENABLE_PIN      PD11

#define E2_STEP_PIN        -1
#define E2_DIR_PIN         -1
#define E2_ENABLE_PIN      -1

//
// Misc. Functions
//
#define SDSS               -1
#define LED_PIN            -1

//
// Heaters / Fans
//
#define HEATER_0_PIN       PA15   // EXTRUDER 1

#define HEATER_BED_PIN     PA7   // BED

#define LASER_PIN      PB1
#define SPINDLE_LASER_ENABLE_PIN       PB10

#define CNC_PIN        PB1

//
// Temperature Sensors
//
#define TEMP_BED_PIN       PC4   // ANALOG NUMBERING
#define TEMP_0_PIN         PC5   // ANALOG NUMBERING
#define TEMP_1_PIN         -1   // ANALOG NUMBERING

//
// Filament Switch
//
#define FIL_RUNOUT_PIN  PD0

//
// Power 0 supply, for HMI
//
#define POWER0_SUPPLY_PIN PB0
#define POWER0_SUPPLY_OFF HIGH
#define POWER0_SUPPLY_ON  LOW

//
// Power1 supply, for motor and extruder
//
#define POWER1_SUPPLY_PIN PB1
#define POWER1_SUPPLY_OFF LOW
#define POWER1_SUPPLY_ON  HIGH

//
// Power2 supply, for bed and add-on
//
#define POWER2_SUPPLY_PIN PB2

#define SCREEN_DET_PIN PD14

#define POWER2_SUPPLY_OFF  LOW
#define POWER2_SUPPLY_ON   HIGH

//
// to detect power loss
//
#define POWER_DETECT_PIN PE0
#define POWER_LOSS_STATE LOW
#define POWER_NORMAL_STATE  HIGH

//
// Heatedbed error detect pin
//
#define HEATEDBED_ON_PIN  PD13

//
// LCD Pins
//
#if ENABLED(ULTRA_LCD)

  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
    #define LCD_PINS_RS         49   // CS chip select /SS chip slave select
    #define LCD_PINS_ENABLE     51   // SID (MOSI)
    #define LCD_PINS_D4         52   // SCK (CLK) clock
  #elif BOTH(NEWPANEL, PANEL_ONE)
    #define LCD_PINS_RS         PF8//PB8
    #define LCD_PINS_ENABLE     PD2
    #define LCD_PINS_D4         PB12
    #define LCD_PINS_D5         PB13
    #define LCD_PINS_D6         PB14
    #define LCD_PINS_D7         PB15
  #else
    #define LCD_PINS_RS         PF8
    #define LCD_PINS_ENABLE     PD2
    #define LCD_PINS_D4         PB12
    #define LCD_PINS_D5         PB13
    #define LCD_PINS_D6         PB14
    #define LCD_PINS_D7         PB15
    #if DISABLED(NEWPANEL)
      #define BEEPER_PIN        33
      // Buttons are attached to a shift register
      // Not wired yet
      //#define SHIFT_CLK 38
      //#define SHIFT_LD 42
      //#define SHIFT_OUT 40
      //#define SHIFT_EN 17
    #endif
  #endif

  #if ENABLED(NEWPANEL)

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)

      #define BEEPER_PIN        37

      #define BTN_EN1           31
      #define BTN_EN2           33
      #define BTN_ENC           35

      #define SD_DETECT_PIN     49
      #define KILL_PIN          41

      #if ENABLED(BQ_LCD_SMART_CONTROLLER)
        #define LCD_BACKLIGHT_PIN 39
      #endif

    #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

      #define BTN_EN1           64
      #define BTN_EN2           59
      #define BTN_ENC           63
      #define SD_DETECT_PIN     42

    #elif ENABLED(LCD_I2C_PANELOLU2)

      #define BTN_EN1           47
      #define BTN_EN2           43
      #define BTN_ENC           32
      #define LCD_SDSS          53
      #define SD_DETECT_PIN     -1
      #define KILL_PIN          41

    #elif ENABLED(LCD_I2C_VIKI)

      #define BTN_EN1           22   // http://files.panucatt.com/datasheets/viki_wiring_diagram.pdf explains 40/42.
      #define BTN_EN2            7   // 22/7 are unused on RAMPS_14. 22 is unused and 7 the SERVO0_PIN on RAMPS_13.

      #define BTN_ENC           -1
      #define LCD_SDSS          53
      #define SD_DETECT_PIN     49

    #elif ANY(VIKI2, miniVIKI)

      #define BEEPER_PIN        33

      // Pins for DOGM SPI LCD Support
      #define DOGLCD_A0         44
      #define DOGLCD_CS         45
      #define LCD_SCREEN_ROT_180

      #define BTN_EN1           22
      #define BTN_EN2            7
      #define BTN_ENC           39

      #define SDSS              53
      #define SD_DETECT_PIN     -1   // Pin 49 for display sd interface, 72 for easy adapter board

      #define KILL_PIN          31

      #define STAT_LED_RED_PIN  32
      #define STAT_LED_BLUE_PIN 35

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
      #define BTN_EN1           35
      #define BTN_EN2           37
      #define BTN_ENC           31
      #define SD_DETECT_PIN     49
      #define LCD_SDSS          53
      #define KILL_PIN          41
      #define BEEPER_PIN        23
      #define DOGLCD_CS         29
      #define DOGLCD_A0         27
      #define LCD_BACKLIGHT_PIN 33

    #elif ENABLED(MINIPANEL)

      #define BEEPER_PIN        42
      // Pins for DOGM SPI LCD Support
      #define DOGLCD_A0         44
      #define DOGLCD_CS         66
      #define LCD_BACKLIGHT_PIN 65   // backlight LED on A11/D65
      #define SDSS              53

      #define KILL_PIN          64
      // GLCD features
      // Uncomment screen orientation
      //#define LCD_SCREEN_ROT_90
      //#define LCD_SCREEN_ROT_180
      //#define LCD_SCREEN_ROT_270
      // The encoder and click button
      #define BTN_EN1           40
      #define BTN_EN2           63
      #define BTN_ENC           59
      // not connected to a pin
      #define SD_DETECT_PIN     49

    #else

      // Beeper on AUX-4
      #define BEEPER_PIN        33

      // buttons are directly attached using AUX-2
      #if ENABLED(REPRAPWORLD_KEYPAD)
        #define BTN_EN1         64
        #define BTN_EN2         59
        #define BTN_ENC         63
        #define SHIFT_OUT       40
        #define SHIFT_CLK       44
        #define SHIFT_LD        42
      #elif ENABLED(PANEL_ONE)
        #define BTN_EN1         59   // AUX2 PIN 3
        #define BTN_EN2         63   // AUX2 PIN 4
        #define BTN_ENC         49   // AUX3 PIN 7
      #else
        #define BTN_EN1         37
        #define BTN_EN2         35
        #define BTN_ENC         31
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN   49
        #define KILL_PIN        41
      #else
        //#define SD_DETECT_PIN -1   // Ramps doesn't use this
      #endif

    #endif
  #endif // NEWPANEL

#endif // ULTRA_LCD

