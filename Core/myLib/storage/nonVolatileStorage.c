// Cleaned to expose each function once; removed duplicate definitions.
#include "nonVolatileStorage.h"

HAL_StatusTypeDef NVS_ErasePage(uint32_t pageAddress)
{
    HAL_StatusTypeDef status;
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0U;

    eraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = pageAddress;
    eraseInitStruct.NbPages     = 1U;

    status = HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
    HAL_FLASH_Lock();
    return status;
}

HAL_StatusTypeDef NVS_WriteWords(uint32_t pageAddress, const uint32_t *words, uint32_t wordCount)
{
    HAL_StatusTypeDef status;
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0U;

    eraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = pageAddress;
    eraseInitStruct.NbPages     = 1U;

    status = HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    uint32_t addr = pageAddress;
    for (uint32_t i = 0U; i < wordCount; ++i) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, words[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        addr += 4U;
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

void NVS_ReadWords(uint32_t pageAddress, uint32_t *words, uint32_t wordCount)
{
    uint32_t addr = pageAddress;
    for (uint32_t i = 0U; i < wordCount; ++i) {
        words[i] = *(volatile uint32_t*)addr;
        addr += 4U;
    }
}

HAL_StatusTypeDef NVS_WriteWord(uint32_t address, uint32_t data)
{
    HAL_StatusTypeDef status;
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    uint32_t pageError = 0U;

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = address;   // Caller should pass page start if required
    erase.NbPages     = 1U;

    status = HAL_FLASHEx_Erase(&erase, &pageError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);
    HAL_FLASH_Lock();
    return status;
}

uint32_t NVS_ReadWord(uint32_t address)
{
    return *(volatile uint32_t*)address;
}
