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
void HMIScreen::CommandProcess(bool nested)
{
  #if ENABLED(HMI_LONG)
    LongHMI.PollingCommand();
    LongHMI.Show();
  #elif ENABLED(HMI_SC20W)
    SC20HMI.PollingCommand(nested);
  #endif
}

void HMIScreen::SendModuleVersion(uint32_t MacID, char *pVersion) {
  #if ENABLED(HMI_SC20W)
    SC20HMI.ReportModuleFirmwareVersion(MacID, pVersion);
  #endif
}

#if ENABLED(HMI_LONG)
/**
 * ChangePage:Change page
 * para Page:Page name,see HMILongDefines.h
 */
void HMIScreen::ChangePage(uint8_t Page)
{
  LongHMI.ChangePage(Page);
}

/**
 * Show:Show print status
 */
void HMIScreen::Show(void)
{
  LongHMI.Show();
}

uint8_t HMIScreen::GetRequestStatus() {
  return LongHMI.HmiRequestStatus;
}

void HMIScreen::ClearRequestStatus() {
  LongHMI.HmiRequestStatus = STAT_IDLE;
}

#endif


#if ENABLED(HMI_SC20W)
/**
 * SendGcode:Send packed gcode to HMI
 * para Gcode:the Gcode STRINGIFY
 * para EventID:the Gcode Eventid
 */
void HMIScreen::SendEvent(uint8_t EventID, char *buffer, uint16_t len) {
  SC20HMI.SendEvent(EventID, buffer, len);
}

void HMIScreen::SendMachineFaultFlag(uint32_t flag) {
  SC20HMI.SendMachineFaultFlag(flag);
}

void HMIScreen::SendMachineStatusChange(uint8_t Status, uint8_t Result) {
  SC20HMI.SendMachineStatusChange(Status, Result);
}

void HMIScreen::SendUpdateComplete(uint8_t Type) {
  SC20HMI.SendUpdateComplete(Type);
}

uint8_t HMIScreen::GetRequestStatus() {
  return SC20HMI.RequestStatus;
}

void HMIScreen::ClearRequestStatus() {
  SC20HMI.RequestStatus = HMI_REQ_NONE;
}

void HMIScreen::SendHalfCalibratePoint(uint8_t op_code, uint8_t index) {
  SC20HMI.SendHalfCalibratePoint(op_code, index);
}

void HMIScreen::SetFeedrate(float f) {
  SC20HMI.SetFeedrate(f);
}
#endif


#endif //HMISUPPORT
