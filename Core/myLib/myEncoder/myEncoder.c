#include "myEncoder.h"

// Global encoder instance that can be accessed by interrupt handlers
static Encoder_t* g_encoder_instance = NULL;

// Register encoder instance for interrupt handling
void Encoder_RegisterInstance(Encoder_t* enc) {
    g_encoder_instance = enc;
}

// Timer overflow interrupt handler - call this from stm32f1xx_it.c
void Encoder_HandleTimerOverflow(void) {
    if (g_encoder_instance && g_encoder_instance->htim) {
        if (__HAL_TIM_GET_FLAG(g_encoder_instance->htim, TIM_FLAG_UPDATE)) {
            __HAL_TIM_CLEAR_IT(g_encoder_instance->htim, TIM_IT_UPDATE);
            
            // Handle overflow based on direction
            uint16_t counter = __HAL_TIM_GET_COUNTER(g_encoder_instance->htim);
            if (counter < 32768) {
                // Forward overflow
                g_encoder_instance->total_pulse += 65536;
            } else {
                // Reverse overflow
                g_encoder_instance->total_pulse -= 65536;
            }
        }
    }
}

// Timer initialization function for encoder mode
HAL_StatusTypeDef Encoder_InitTimer(TIM_HandleTypeDef* htim) {
    if (!htim || !htim->Instance) return HAL_ERROR;
    
    TIM_Encoder_InitTypeDef sConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 65535;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    
    if (HAL_TIM_Encoder_Init(htim, &sConfig) != HAL_OK) {
        return HAL_ERROR;
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

// Full encoder initialization with timer configuration
HAL_StatusTypeDef Encoder_InitFull(Encoder_t* enc, TIM_HandleTypeDef* htim, uint16_t ppr, float diameter_m, uint16_t update_ms) {
    if (!enc || !htim) return HAL_ERROR;
    
    // Initialize timer first
    if (Encoder_InitTimer(htim) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // Initialize encoder struct and start encoder
    Encoder_Init(enc, htim, ppr, diameter_m, update_ms);
    
    return HAL_OK;
}