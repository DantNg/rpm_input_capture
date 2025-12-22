#include "myFlash.h"

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
    out->baudRate       = buffer[0];
    out->parity         = buffer[1];
    out->stopBits       = buffer[2];
    out->frameTimeoutMs = buffer[3];
}

HAL_StatusTypeDef myFlash_SaveEncoderParams(const myEncoderParams *params)
{
    uint32_t buffer[2];
    buffer[0] = params->diameter;
    buffer[1] = params->pulsesPerRev;
    return NVS_WriteWords(MYFLASH_PAGE_ENCODER, buffer, 2U);
}

void myFlash_LoadEncoderParams(myEncoderParams *out)
{
    uint32_t buffer[2];
    NVS_ReadWords(MYFLASH_PAGE_ENCODER, buffer, 2U);
    out->diameter     = buffer[0];
    out->pulsesPerRev = buffer[1];
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
