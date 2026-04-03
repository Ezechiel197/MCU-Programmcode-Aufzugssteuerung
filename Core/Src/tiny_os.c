// This file contains a tiny (Manu) OS Layer for our Project.

#include "tiny_os.h"

//EventListe.
//Events:
//	0x00000001 = Modbus IO Work-Event
//	0x00000002 = Simulation IO Work-Event
//	0x00000004 = Debug IO Work-Event
//	0x00000008 = Application IO Work-Event
static volatile uint32_t eventlist = 0;

//Delay with reduced Power wait (saves some energy...)
//I don't link shitty running CPUs...
// @param  Delay is delaytime in ms (ms = systick time)
// @return ongoing Event or zero if delay complete
uint32_t LowPower_Delay(uint32_t Delay)
{
    uint32_t tickstart = HAL_GetTick();
    while ((HAL_GetTick() - tickstart) < Delay) {
        __WFI(); //CPU in low power state bis zum nächsten (irgend einen!) IRQ.
        if (eventlist) return eventlist; //Event? dann raus und event abarbeiten!
    }

    return 0; //Delay without event
}

// Set an event.
// @param  Event is the event to Set (Bit coded)
// @return current event list
uint32_t SetEvent(uint32_t Event)
{
	__disable_irq();
	eventlist |= Event;
	__enable_irq();
	return eventlist;
}

// Reset an event.
// @param  Event is the event to ReSet (Bit coded)
// @return current event list
uint32_t ResetEvent(uint32_t Event)
{
	__disable_irq();
	eventlist &= ~Event;
	__enable_irq();
	return eventlist;
}

// @return current event list
uint32_t GetEvent(void)
{
	return eventlist;
}
