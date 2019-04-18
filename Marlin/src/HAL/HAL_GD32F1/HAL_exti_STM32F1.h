#pragma once

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#define PA 0
#define PB 1
#define PC 2
#define PD 3
#define PE 4
#define PF 5
#define PG 6

/**
* ExtiInit:Exti Interrup INIT_AUTO_FAN_PIN
* para PortIndex:GPIOA-GPIOI
* para PinIndex:0-15
* para RisingFallingEdge:0-3,0:Falling, 1:Rising, 2:Rising and falling
*/
void ExtiInit(uint8_t PortIndex, uint8_t PinIndex, uint8_t RisingFallingEdge) ;

