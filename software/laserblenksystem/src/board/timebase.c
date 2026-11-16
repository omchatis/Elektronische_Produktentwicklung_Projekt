#include "board/timebase.h"

#include "stm32g4xx.h"

enum
{
    SYSTICK_RATE_HZ = 1000
};

/* SysTick is the only writer. Readers take a single aligned 32-bit sample. */
static volatile uint32_t tick_ms;

void SysTick_Handler(void)
{
    ++tick_ms;
}

bool timebase_initialize(void)
{
    /* The CMSIS startup leaves the G431 on its HSI16 reset clock. */
    SystemCoreClockUpdate();
    uint32_t ticks_per_millisecond = SystemCoreClock / SYSTICK_RATE_HZ;
    uint32_t result = SysTick_Config(ticks_per_millisecond);
    return result == 0U;
}

uint32_t timebase_millis(void)
{
    return tick_ms;
}

void timebase_idle(void)
{
    __WFI();
}
