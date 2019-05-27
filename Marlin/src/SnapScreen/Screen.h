#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(HMISUPPORT)
#ifndef _SCREEN_H_
#define _SCREEN_H_

class HMIScreen
{
    public:
    HMIScreen(){}
    void Init(void);
    void CommandProcess(void);    
    #if ENABLED(HMI_SC20W)  
      void SendMachineStatusChange(uint8_t, uint8_t);
      void SendMachineFaultFlag();
      void SendGcode(char *GCode, uint8_t EventID);
    #else
      FORCE_INLINE static void SendMachineStatusChange(uint8_t, uint8_t){}
      FORCE_INLINE static void SendMachineFaultFlag() {}
      FORCE_INLINE static void SendGcode(char *GCode, uint8_t EventID) {}
    #endif

    #if ENABLED(HMI_LONG)
      void Show(void);
      void ChangePage(uint8_t Page);
    #elif ENABLED(HMI_SC20W)
      FORCE_INLINE static void Show(void){}
      FORCE_INLINE static void ChangePage(uint8_t){}
    #endif

    private:

    public:
    uint32_t ScreenLockTick;
    bool ScreenLock;
    uint8_t CalibrateMethod;
    uint8_t RequestStatus;
    
};

extern HMIScreen HMI;

#endif
#endif //HMISUPPORT

