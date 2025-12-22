#ifndef __MY_FLASH_H
#define __MY_FLASH_H

#include "stm32f1xx_hal.h"
#include "storage/nonVolatileStorage.h"

// === Page layout (STM32F103C8 default) ===
// Adjust these addresses if you choose different pages.
#define MYFLASH_PAGE_UART     0x0801EC00U  // consolidated UART params
#define MYFLASH_PAGE_ENCODER  0x0801F800U  // DIA + PPR stored together
#define MYFLASH_PAGE_LENGTH   0x0801FC00U  // single length value
#define MYFLASH_PAGE_MODE     0x0801F400U  // measurement mode
#define MYFLASH_PAGE_MODBUS   0x0801F000U  // Modbus configuration

// === Data structures ===
typedef struct {
	uint32_t baudRate;        // e.g., 9600, 115200
	uint32_t parity;          // 0=None, 1=Odd, 2=Even
	uint32_t stopBits;        // 1 or 2
	uint32_t frameTimeoutMs;  // previously "TIME"
} myUARTParams;

typedef struct {
	uint32_t diameter;        // DIA
	uint32_t pulsesPerRev;    // PPR
} myEncoderParams;

typedef struct {
	uint8_t slave_id;         // Modbus slave ID (1-247)
	uint8_t enabled;          // 1=enabled, 0=disabled
	uint16_t reserved;        // padding for 4-byte alignment
} myModbusConfig;

// === High-level helpers built on NVS ===
HAL_StatusTypeDef myFlash_SaveUARTParams(const myUARTParams *params);
void               myFlash_LoadUARTParams(myUARTParams *out);

HAL_StatusTypeDef myFlash_SaveEncoderParams(const myEncoderParams *params);
void               myFlash_LoadEncoderParams(myEncoderParams *out);

HAL_StatusTypeDef myFlash_SaveLength(uint32_t length);
uint32_t           myFlash_LoadLength(void);

HAL_StatusTypeDef myFlash_SaveMeasurementMode(uint32_t mode);
uint32_t           myFlash_LoadMeasurementMode(void);

HAL_StatusTypeDef myFlash_SaveModbusConfig(const myModbusConfig *config);
void               myFlash_LoadModbusConfig(myModbusConfig *out);

// === Low-level backward-compatible aliases ===
#define myFlash_Write(addr, data)      NVS_WriteWord((addr), (data))
#define myFlash_Read(addr)             NVS_ReadWord((addr))
#define myFlash_ErasePage(page_addr)   (void)NVS_ErasePage((page_addr))

#endif
