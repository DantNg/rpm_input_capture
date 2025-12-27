#include "myFlash.h"
#include <stdint.h>
#include<stdio.h>
HAL_StatusTypeDef myFlash_SaveUARTParams(const myUARTParams *params)
{
    uint32_t buffer[4];
    buffer[0] = params->baudRate;
    buffer[1] = params->parity;
    buffer[2] = params->stopBits;
    buffer[3] = params->frameTimeoutMs;
    return NVS_WriteWords(MYFLASH_PAGE_UART, buffer, 4U);
}

void myFlash_LoadUARTParams(myUARTParams *out)
{
    uint32_t buffer[4];
    NVS_ReadWords(MYFLASH_PAGE_UART, buffer, 4U);
    if(buffer[0]>0 && buffer[0]<10000000U) {
        // Valid baud rate range
    } else {
        buffer[0] = 115200U; // Default baud rate
    }
    if (buffer[1] <= 2U) {
        // Valid parity
    } else {
        buffer[1] = 0U; // Default to None
    }
    if (buffer[2] == 1U || buffer[2] == 2U) {
        // Valid stop bits
    } else {
        buffer[2] = 1U; // Default to 1 stop bit
    }
    if (buffer[3] >= 10U && buffer[3] <= 60000U) {
        // Valid timeout
    } else {
        buffer[3] = 1000U; // Default to 1000 ms
    }
    out->baudRate       = buffer[0];
    out->parity         = buffer[1];
    out->stopBits       = buffer[2];
    out->frameTimeoutMs = buffer[3];
}

HAL_StatusTypeDef myFlash_SaveEncoderParams(const myEncoderParams *params)
{
    uint32_t buffer[4];
    buffer[0] = params->diameter;
    buffer[1] = params->pulsesPerRev;
    buffer[2] = params->timeout;
    buffer[3] = params->sampleTimeMs;

    return NVS_WriteWords(MYFLASH_PAGE_ENCODER, buffer, 4U);
}

void myFlash_LoadEncoderParams(myEncoderParams *out)
{
    uint32_t buffer[4];
    NVS_ReadWords(MYFLASH_PAGE_ENCODER, buffer, 4U);
    if (buffer[0] == 0U || buffer[0] > 100000U) {
        buffer[0] = 1000U; // Default diameter in mm
    }
    if (buffer[1] == 0U || buffer[1] > 10000U) {
        buffer[1] = 1U; // Default PPR 1
    }
    if (buffer[2] > 60000U) {
        buffer[2] = 10000U; // Default 10s timeout in ms
    }
    if (buffer[3] < 10U || buffer[3] > 10000U) {
        buffer[3] = 100U; // Default sample time in 100 ms
    }
    out->diameter     = buffer[0];
    out->pulsesPerRev = buffer[1];
    out->timeout = buffer[2];
    out->sampleTimeMs = buffer[3];
}

HAL_StatusTypeDef myFlash_SaveLength(uint32_t length)
{
    return NVS_WriteWords(MYFLASH_PAGE_LENGTH, &length, 1U);
}

uint32_t myFlash_LoadLength(void)
{
    return NVS_ReadWord(MYFLASH_PAGE_LENGTH);
}

HAL_StatusTypeDef myFlash_SaveMeasurementMode(uint32_t mode)
{
    return NVS_WriteWords(MYFLASH_PAGE_MODE, &mode, 1U);
}

uint32_t myFlash_LoadMeasurementMode(void)
{
    return NVS_ReadWord(MYFLASH_PAGE_MODE);
}

HAL_StatusTypeDef myFlash_SaveModbusConfig(const myModbusConfig *config)
{
    uint32_t buffer[1];
    // Pack config into single 32-bit word: [reserved:16][enabled:8][slave_id:8]
    buffer[0] = ((uint32_t)config->reserved << 16) | ((uint32_t)config->enabled << 8) | (uint32_t)config->slave_id;
    return NVS_WriteWords(MYFLASH_PAGE_MODBUS, buffer, 1U);
}

void myFlash_LoadModbusConfig(myModbusConfig *out)
{
    uint32_t data = NVS_ReadWord(MYFLASH_PAGE_MODBUS);
    out->slave_id = (uint8_t)(data & 0xFF);
    out->enabled = (uint8_t)((data >> 8) & 0xFF);
    out->reserved = (uint16_t)((data >> 16) & 0xFFFF);
}

HAL_StatusTypeDef myFlash_SaveSpeedUnitConfig(const mySpeedUnitConfig *config)
{
    uint32_t buffer[1];
    // Pack config into single 32-bit word: [reserved:24][speed_unit:8]
    buffer[0] = ((uint32_t)config->reserved[2] << 24) | ((uint32_t)config->reserved[1] << 16) | 
                ((uint32_t)config->reserved[0] << 8) | (uint32_t)config->speed_unit;
    return NVS_WriteWords(MYFLASH_PAGE_SPEED_UNIT, buffer, 1U);
}

void myFlash_LoadSpeedUnitConfig(mySpeedUnitConfig *out)
{
    uint32_t data = NVS_ReadWord(MYFLASH_PAGE_SPEED_UNIT);
    out->speed_unit = (uint8_t)(data & 0xFF);
    out->reserved[0] = (uint8_t)((data >> 8) & 0xFF);
    out->reserved[1] = (uint8_t)((data >> 16) & 0xFF);
    out->reserved[2] = (uint8_t)((data >> 24) & 0xFF);
}

HAL_StatusTypeDef myFlash_SaveHysteresisTable(const myHysteresisTable *table)
{
    // Calculate required size: 1 word for header + 10 entries * 1 word each
    uint32_t buffer[11];
    
    // Pack header: [reserved:24][entry_count:8]
    buffer[0] = ((uint32_t)table->reserved[2] << 24) | ((uint32_t)table->reserved[1] << 16) | 
                ((uint32_t)table->reserved[0] << 8) | (uint32_t)table->entry_count;
    
    // Pack entries: [hysteresis:16][rpm_threshold:16] per word
    for (int i = 0; i < 10; i++) {
        buffer[i + 1] = ((uint32_t)table->entries[i].hysteresis << 16) | 
                        (uint32_t)table->entries[i].rpm_threshold;
    }
    
    return NVS_WriteWords(MYFLASH_PAGE_HYSTERESIS, buffer, 11U);
}

void myFlash_LoadHysteresisTable(myHysteresisTable *out)
{
    uint32_t buffer[11];
    NVS_ReadWords(MYFLASH_PAGE_HYSTERESIS, buffer, 11U);
    
    // Unpack header
    out->entry_count = (uint8_t)(buffer[0] & 0xFF);
    out->reserved[0] = (uint8_t)((buffer[0] >> 8) & 0xFF);
    out->reserved[1] = (uint8_t)((buffer[0] >> 16) & 0xFF);
    out->reserved[2] = (uint8_t)((buffer[0] >> 24) & 0xFF);
    
    // Unpack entries
    for (int i = 0; i < 10; i++) {
        out->entries[i].rpm_threshold = (uint16_t)(buffer[i + 1] & 0xFFFF);
        out->entries[i].hysteresis = (uint16_t)((buffer[i + 1] >> 16) & 0xFFFF);
    }
}

HAL_StatusTypeDef myFlash_SaveModbusUARTParams(const myModbusUARTParams *params)
{
    uint32_t buffer[4];
    buffer[0] = params->baudRate;
    buffer[1] = params->parity;
    buffer[2] = params->stopBits;
    buffer[3] = params->frameTimeoutMs;
    return NVS_WriteWords(MYFLASH_PAGE_MODBUS_UART, buffer, 4U);
}

void myFlash_LoadModbusUARTParams(myModbusUARTParams *out)
{
    uint32_t buffer[4];
    NVS_ReadWords(MYFLASH_PAGE_MODBUS_UART, buffer, 4U);

    // Backward-compatible fallback (older builds stored at an unaligned offset).
    // If the new page looks erased, try reading the legacy location.
    // if (buffer[0] == 0xFFFFFFFFU && buffer[1] == 0xFFFFFFFFU && buffer[2] == 0xFFFFFFFFU && buffer[3] == 0xFFFFFFFFU) {
    //     NVS_ReadWords(MYFLASH_PAGE_MODBUS + 0x100U, buffer, 4U);
    // }
    
    if(buffer[0]>0 && buffer[0]<10000000U) {
        // Valid baud rate range
    } else {
        buffer[0] = 9600U; // Default baud rate
    }
    if (buffer[1] <= 2U) {
        // Valid parity
    } else {
        buffer[1] = 0U; // Default to None
    }
    if (buffer[2] == 1U || buffer[2] == 2U) {
        // Valid stop bits
    } else {
        buffer[2] = 1U; // Default to 1 stop bit
    }
    if (buffer[3] >= 10U && buffer[3] <= 60000U) {
        // Valid timeout
    } else {
        buffer[3] = 1000U; // Default to 1000 ms
    }
    out->baudRate       = buffer[0];
    out->parity         = buffer[1];
    out->stopBits       = buffer[2];
    out->frameTimeoutMs = buffer[3];
}


HAL_StatusTypeDef myFlash_SaveDebugConfig(const myDebugConfig *params)
{
    uint32_t data;
    // Pack params into data: [enabled:8][reserved:8][interval:16]
    data = ((uint32_t)params->enabled << 16) | ((uint32_t)params->interval & 0xFFFF);
    return NVS_WriteWords(MYFLASH_PAGE_DEBUG, &data, 1U);
}

void myFlash_LoadDebugConfig(myDebugConfig *out)
{
    uint32_t data;
    NVS_ReadWords(MYFLASH_PAGE_DEBUG, &data, 1U);
    if(data)
    out->enabled = (uint8_t)((data >> 16) & 0xFF);
    out->interval = (uint16_t)(data & 0xFFFF);
}
