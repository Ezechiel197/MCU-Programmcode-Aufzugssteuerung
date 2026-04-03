// This file contains a tiny (Manu) OS Layer for our Project.

#include "stm32l4xx_hal.h"

//EventList:
#define NO_EVENT 		0x00000000
#define MODBUS_EVENT	0x00000001
#define SIMUL_EVENT		0x00000002
#define DEBUG_EVENT		0x00000004
#define APP_EVENT		0x00000008

//Summe aller events:
#define ALL_EVENTS		0x0000000F

//Delay with reduced Power wait (saves some energy...)
//I don't link shitty running CPUs...
// @param  Delay is delaytime in ms (ms = systick time)
// @return ongoing Event or zero if delay complete
uint32_t LowPower_Delay(uint32_t Delay);

// Set/Reset an event.
// @param  Event is the event to Set (Bit coded)
// @return current event list
uint32_t SetEvent(uint32_t Event);
uint32_t ResetEvent(uint32_t Event);

// @return current event list
uint32_t GetEvent(void);
