#pragma once

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#define   FRAME_DATA  0
#define   FRAME_REMOTE  1

void CanInit(uint8_t PortNum);
bool CanSendShortPacked(uint32_t ID, uint8_t PortNum, uint8_t FrameType, uint8_t DataLen, uint8_t *pData);
uint8_t Canbus1ParseData(uint32_t *ID, uint8_t *FrameType, uint8_t *pData, uint8_t *Len);
uint8_t Canbus2ParseData(uint32_t *ID, uint8_t *FrameType, uint8_t *pData, uint8_t *Len);


