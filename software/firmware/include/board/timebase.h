#ifndef BOARD_TIMEBASE_H
#define BOARD_TIMEBASE_H

#include <stdbool.h>
#include <stdint.h>

/* Start the 1 ms SysTick timebase. Call before any timed bus operation. */
bool timebase_initialize(void);

/* Milliseconds since reset. Subtract timestamps using unsigned arithmetic. */
uint32_t timebase_millis(void);

/* Sleep the CPU until an interrupt, usually the next SysTick. */
void timebase_idle(void);

#endif
