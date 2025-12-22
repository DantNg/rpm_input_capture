#ifndef __NON_VOLATILE_STORAGE_H
#define __NON_VOLATILE_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

// Low-level primitives only (erase/program/read). No app-specific layouts.
HAL_StatusTypeDef NVS_ErasePage(uint32_t pageAddress);
HAL_StatusTypeDef NVS_WriteWords(uint32_t pageAddress, const uint32_t *words, uint32_t wordCount);
void              NVS_ReadWords(uint32_t pageAddress, uint32_t *words, uint32_t wordCount);
HAL_StatusTypeDef NVS_WriteWord(uint32_t address, uint32_t data);
uint32_t          NVS_ReadWord(uint32_t address);

#ifdef __cplusplus
}
#endif

#endif // __NON_VOLATILE_STORAGE_H
