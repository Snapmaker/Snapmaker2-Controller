#include <wirish_time.h>
#include "gcode_result_handler.h"
#include "event_handler.h"
#include "../../Marlin/src/gcode/queue.h"

GcodeResultHandler gcode_result_handler;

void GcodeResultHandler::GcodeResultCheck(unsigned char ch) {
  if (Screen_send_ok[cmd_queue_index_r]) {

    gcode_result_mgt_.buf[gcode_result_mgt_.index] = ch;

    gcode_result_mgt_.index++;
    if (gcode_result_mgt_.index == BUFFER_SIZE) {
      SSTP_Event_t event = {EID_GCODE_RESULT_ACK, GCODE_RESULT_OPC_DATA};
      event.length = gcode_result_mgt_.index;
      event.data   = gcode_result_mgt_.buf;

      hmi.Send(event);

      gcode_result_mgt_.index = 0;
    }
  }
}

void GcodeResultHandler::GcodeResultFlush() {
  SSTP_Event_t event = {EID_GCODE_RESULT_ACK};

  if (Screen_send_ok[cmd_queue_index_r]) {
    if (gcode_result_mgt_.index > 0) {
      event.op_code = GCODE_RESULT_OPC_DATA;
      event.length  = gcode_result_mgt_.index;
      event.data    = gcode_result_mgt_.buf;
      hmi.Send(event);

      gcode_result_mgt_.index = 0;
    }

    event.op_code = GCODE_RESULT_OPC_EOF;
    event.length  = 0;
    event.data    = NULL;

    hmi.Send(event);
  }
}
