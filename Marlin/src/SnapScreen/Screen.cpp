#include "../inc/MarlinConfig.h"

#if ENABLED(HMISUPPORT)

#include "../Marlin.h"
#include "Screen.h"

#if ENABLED(HMI_LONG)
#include "HMILong.h"
HMILong LongHMI;
#elif ENABLED(HMI_SC20)
#include "HMISC20.h"
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

#endif


#if ENABLED(HMI_SC20)
/**
 * SendGcode:Send packed gcode to HMI
 * para Gcode:the Gcode STRINGIFY
 * para EventID:the Gcode Eventid
 */
void HMIScreen::SendGcode(char *GCode, uint8_t EventID)
{
}
#endif


#endif //HMISUPPORT
