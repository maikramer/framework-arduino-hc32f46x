#include "systick.h"
#include <hc32_ddl.h>
#include "bsp_timer.h"

extern "C" void SysTick_IrqHandler(void)
{
    SysTick_IncTick();
}

void systick_init(){
    SysTick_Init(TICKS_PER_SECOND);
}

uint32_t systick_millis()
{
    return SysTick_GetTick();
}

uint32_t systick_micros()
{
    return SysTick_GetTick() * 1000;
}