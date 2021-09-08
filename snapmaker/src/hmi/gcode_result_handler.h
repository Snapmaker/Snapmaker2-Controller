
#include <stdio.h>
#include <HardwareSerial.h>


#define BUFFER_SIZE 256
typedef struct {
  uint8_t buf[BUFFER_SIZE];
  uint16_t index;
}screen_gcode_result_mgt_t;

class GcodeResultHandler {
  public:
    GcodeResultHandler() {
      gcode_result_mgt_.index = 0;
    }

    void GcodeResultCheck(unsigned char ch);
    void GcodeResultFlush();
  private:
    screen_gcode_result_mgt_t gcode_result_mgt_;
};

extern GcodeResultHandler gcode_result_handler;
