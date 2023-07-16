#pragma once
#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>

#include "hc32_ddl.h"
#include "hc32f460_clk.h"
#include "hc32f460_efm.h"
#include "hc32f460_utility.h"
#include "hc32f460_sram.h"
#include "hc32f460_interrupts.h"
#include "hc32f460_pwc.h"

#define TICKS_PER_SECOND 1000ul

void systick_init();
uint32_t systick_millis();
uint32_t systick_micros();

#ifdef __cplusplus
}
#endif
