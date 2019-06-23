#include "../inc/MarlinConfig.h"

#if ENABLED(HMISUPPORT)

#include "../Marlin.h"
#include "Screen.h"

#if ENABLED(HMI_LONG)
#include "HMILong.h"
HMILong LongHMI;
#elif ENABLED(HMI_SC20W)
#include "HMISC20.h"
HMI_SC20 SC20HMI;
#endif
/**
 * Init:Initial the hmi communication port
 */
void HMIScreen::Init(void)
{
  HMISERIAL.begin(115200);
}

/**
 * CommandProcess:process the data from hmi
 */
void HMIScreen::CommandProcess(void)
{
  #if ENABLED(HMI_LONG)
    LongHMI.PollingCommand();
    LongHMI.Show();
  #elif ENABLED(HMI_SC20W)
    SC20HMI.PollingCommand();
  #endif
}

#if ENABLED(HMI_LONG)
/**
 * ChangePage:Change page
 * para Page:Page name,see HMILongDefines.h
 */
void HMIScreen::ChangePage(uint8_t Page)
{
  #if ENABLED(HMI_LONG)
    LongHMI.ChangePage(Page);
  #endif
}

/**
 * Show:Show print status
 */
void HMIScreen::Show(void)
{
  #if ENABLED(HMI_LONG)
    LongHMI.Show();
  #endif
}

#endif


#if ENABLED(HMI_SC20W)
/**
 * SendGcode:Send packed gcode to HMI
 * para Gcode:the Gcode STRINGIFY
 * para EventID:the Gcode Eventid
 */
void HMIScreen::SendGcode(char *GCode, uint8_t EventID) {
  SC20HMI.SendGcode(GCode, EventID);
}

void HMIScreen::SendMachineFaultFlag() {
  SC20HMI.SendMachineStatus();
}

void HMIScreen::SendMachineStatusChange(uint8_t Status, uint8_t Result) {
  SC20HMI.SendMachineStatusChange(Status, Result);
}

void HMIScreen::SendUpdateComplete(uint8_t Type) {
  SC20HMI.SendUpdateComplete(Type);
}
#endif


#endif //HMISUPPORT
