#pragma once

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#define FRAME_DATA  0
#define FRAME_REMOTE  1

#define IDTYPE_STDID  0
#define IDTYPE_EXTID  1

void CanInit();
bool CanSendPacked(uint32_t ID, uint8_t IDType, uint8_t PortNum, uint8_t FrameType, uint8_t DataLen, uint8_t *pData);
bool CanSendPacked2(uint32_t ID, uint8_t PortNum, uint8_t FrameType, uint8_t DataLen, uint8_t *pData, uint32_t *RegStatusValue);

uint8_t Canbus1ParseData(uint32_t *ID, uint8_t *IDType, uint8_t *FrameType, uint8_t *pData, uint8_t *Len, uint8_t FIFONum);
uint8_t Canbus2ParseData(uint32_t *ID, uint8_t *IDType, uint8_t *FrameType, uint8_t *pData, uint8_t *Len, uint8_t FIFONum);


