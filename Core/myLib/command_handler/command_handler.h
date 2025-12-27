#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "stm32f1xx_hal.h"
#include "../myEncoder/measurement_mode.h"
#include <stdbool.h>
#include <stdint.h>

// Forward declaration for UART3 DMA restart function
void Restart_UART3_DMA(void);

#ifdef __cplusplus
extern "C" {
#endif

#define CMDH_BUFFER_SIZE 64

// Command handler initialization structure
typedef struct {
    UART_HandleTypeDef *huart;
    // Callback function pointers for external dependencies
    void (*reinit_uarts)(void);
    void (*apply_parity_config)(uint32_t parity_mode);
    void (*encoder_init)(void* enc, void* htim, uint32_t ppr, float dia, uint32_t timeout, uint32_t time);
    
    // Flash save function pointers
    HAL_StatusTypeDef (*save_uart_params)(const void* params);
    HAL_StatusTypeDef (*save_encoder_params)(const void* params);
    HAL_StatusTypeDef (*save_length)(uint32_t length_mm);
    HAL_StatusTypeDef (*save_measurement_mode)(uint32_t mode);
    HAL_StatusTypeDef (*save_modbus_config)(const void* config);
    uint32_t (*load_measurement_mode)(void);
    
    // System data pointers
    uint32_t *ppr;
    float *dia;
    uint32_t *time;
    uint32_t *timeout;
    // float *length;  // Now handled by encoder library
    uint32_t *parity;
    MeasurementMode_t *measurement_mode;
    UART_HandleTypeDef *huart1;
    UART_HandleTypeDef *huart3;
    void *encoder;
    void *htim;
    uint8_t slave_id;
} CommandHandler_Config_t;

// Command handler instance
typedef struct {
    CommandHandler_Config_t config;
    char cmd_buffer[32];
    uint8_t cmd_index;
    bool show_prompt;
} CommandHandler_t;

// Function prototypes
void CommandHandler_Init(CommandHandler_t *handler, CommandHandler_Config_t *config);
void CommandHandler_Process(CommandHandler_t *handler);
void CommandHandler_Reset(CommandHandler_t *handler);
SpeedDisplayUnit_t CommandHandler_GetSpeedDisplayUnit(void);
uint8_t CommandHandler_GetModbusSlaveId(void);
bool CommandHandler_IsModbusEnabled(void);
void CommandHandler_SetModbusConfig(uint8_t slave_id, bool enabled);
void CommandHandler_InitModbusFromFlash(void);
void CommandHandler_InitSpeedUnitFromFlash(void);
void CommandHandler_SetDebugConfig(bool enabled, uint32_t interval_ms);
void CommandHandler_GetDebugConfig(bool *enabled, uint32_t *interval_ms);
void CommandHandler_InitDebugConfigFromFlash(void);
#ifdef __cplusplus
}
#endif

#endif // COMMAND_HANDLER_H
