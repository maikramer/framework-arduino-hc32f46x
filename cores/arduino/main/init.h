#pragma once
#include <stdint.h>

#define LD_FLASH_START 0x8000u
// get flash start address from ddl framework
#ifndef LD_FLASH_START
#warning "LD_FLASH_START not defined, fallback to 0x0"
#define LD_FLASH_START 0x0
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

/**
 * @brief initialize the HC32F460 SoC 
 */
void core_init();
