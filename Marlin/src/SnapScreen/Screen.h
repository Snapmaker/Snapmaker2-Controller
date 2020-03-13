#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(HMISUPPORT)
#ifndef _SCREEN_H_
#define _SCREEN_H_

enum HMIReq : uint8_t {
  HMI_REQ_NONE = 0,
  HMI_REQ_PAUSE = 0x4,
  HMI_REQ_RESUME = 0x5,
  HMI_REQ_STOP = 0x6,
  HMI_REQ_FINISH = 0x7,
  HMI_REQ_INVALID
};

class HMIScreen
{
    public:
    HMIScreen(){}
    void Init(void);
    void CommandProcess(bool nested);
    uint8_t GetRequestStatus(void);
    void ClearRequestStatus();
    void SendModuleVersion(uint32_t MacID, char *pVersion);
    #if ENABLED(HMI_SC20W)
      void SendMachineStatusChange(uint8_t, uint8_t);
      void SendMachineFaultFlag(uint32_t flag = 0);
      void SendEvent(uint8_t EventID, char *buffer, uint16_t len);
      void SendUpdateComplete(uint8_t Type);
      void SendHalfCalibratePoint(uint8_t Opcode, uint8_t Index);
      void SetFeedrate(float f);
    #else
      FORCE_INLINE static void SendMachineStatusChange(uint8_t, uint8_t){}
      FORCE_INLINE static void SendMachineFaultFlag() {}
      FORCE_INLINE static void SendGcode(char *GCode, uint8_t EventID) {}
      FORCE_INLINE static void SendUpdateComplete(uint8_t Type) {}
    #endif

    #if ENABLED(HMI_LONG)
      void Show(void);
      void ChangePage(uint8_t Page);
    #elif ENABLED(HMI_SC20W)
      FORCE_INLINE static void Show(void){}
      FORCE_INLINE static void ChangePage(uint8_t){}
    #endif

    public:
    uint32_t ScreenLockTick;
    bool ScreenLock;
    uint8_t CalibrateMethod;

};

extern HMIScreen HMI;

#endif
#endif //HMISUPPORT

