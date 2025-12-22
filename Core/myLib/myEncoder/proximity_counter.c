/**
 ******************************************************************************
 * @file    proximity_counter.c
 * @brief   Proximity sensor RPM counter library implementation
 * @author  Auto-generated
 * @date    December 2025
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "proximity_counter.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Apply adaptive hysteresis filter to RPM value using configurable table
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param new_rpm: New RPM value to filter
 * @param prev_rpm: Previous RPM value  
 * @param stability_counter: Pointer to stability counter
 * @retval Filtered RPM value
 */
int ProximityCounter_ApplyHysteresisFilter(ProximityCounter_t *prox_counter, 
                                         int new_rpm, 
                                         int prev_rpm, 
                                         volatile uint8_t *stability_counter) {
    if (!prox_counter) {
        return new_rpm;
    }
    
    int threshold = 50; // Default threshold
    
    // Find appropriate threshold from table
    for (int i = prox_counter->hysteresis_table_size - 1; i >= 0; i--) {
        if (new_rpm >= prox_counter->hysteresis_table[i].rpm_threshold) {
            threshold = prox_counter->hysteresis_table[i].hysteresis;
            break;
        }
    }

    // Calculate absolute difference
    int diff = new_rpm - prev_rpm;
    if (diff < 0) diff = -diff;
    
    if (diff <= threshold) {
        (*stability_counter)++;
        if (*stability_counter >= 10) {
            return prev_rpm + (new_rpm - prev_rpm) / 4;  // Smooth update
        }
        return prev_rpm;
    } else {
        *stability_counter = 0;
        return new_rpm;
    }
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize proximity counter
 */
void ProximityCounter_Init(ProximityCounter_t *prox_counter, 
                          const ProximityCounterConfig_t *config,
                          TIM_HandleTypeDef *htim) {
    if (!prox_counter || !config || !htim) {
        return;
    }
    
    // Clear the structure
    memset(prox_counter, 0, sizeof(ProximityCounter_t));
    
    // Set configuration parameters
    prox_counter->ppr = config->ppr > 0 ? config->ppr : 1;
    prox_counter->diameter = config->diameter > 0.0f ? config->diameter : 0.25f;
    prox_counter->timeout_ms = config->timeout_ms > 0 ? config->timeout_ms : PROXIMITY_NO_PULSE_TIMEOUT_MS;
    prox_counter->averaging_samples = config->averaging_samples > 0 ? config->averaging_samples : 3;
    prox_counter->speed_unit = PROXIMITY_SPEED_UNIT_RPM; // Default to RPM
    
    // Set timer handle
    prox_counter->htim = htim;
    
    // Initialize default hysteresis table
    ProximityCounter_InitDefaultHysteresis(prox_counter);
    
    // Initialize state variables
    prox_counter->first_measurement = 1;
    prox_counter->last_capture_time = HAL_GetTick();
}

/**
 * @brief Start proximity counter measurement
 */
void ProximityCounter_Start(ProximityCounter_t *prox_counter) {
    if (!prox_counter || !prox_counter->htim) {
        return;
    }
    
    // Reset all measurement states
    ProximityCounter_Reset(prox_counter);
    
    // Start input capture interrupt
    HAL_TIM_IC_Start_IT(prox_counter->htim, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(prox_counter->htim);  // Enable overflow interrupt
}

/**
 * @brief Stop proximity counter measurement
 */
void ProximityCounter_Stop(ProximityCounter_t *prox_counter) {
    if (!prox_counter || !prox_counter->htim) {
        return;
    }
    
    // Stop input capture interrupt
    HAL_TIM_IC_Stop_IT(prox_counter->htim, TIM_CHANNEL_1);
    HAL_TIM_Base_Stop_IT(prox_counter->htim);
}

/**
 * @brief Process new capture data and calculate RPM
 */
void ProximityCounter_ProcessCapture(ProximityCounter_t *prox_counter) {
    if (!prox_counter || !prox_counter->new_capture_ready) {
        return;
    }
    
    prox_counter->new_capture_ready = 0;  // Clear flag
    
    // Calculate RPM from captured difference
    if (prox_counter->difference > 0) {
        float frequency = (float)PROXIMITY_COUNTER_HZ / (float)prox_counter->difference;
        int rpm_raw = (int)(frequency * 60.0f / prox_counter->ppr);  // Account for PPR
        
        // Apply adaptive hysteresis filter
        prox_counter->rpm = (float)ProximityCounter_ApplyHysteresisFilter(
            prox_counter,
            rpm_raw, 
            prox_counter->rpm_previous, 
            &prox_counter->stability_counter
        );
        prox_counter->rpm_previous = (int)prox_counter->rpm;
    }
}

/**
 * @brief Check for timeout and reset if no pulses detected
 */
void ProximityCounter_CheckTimeout(ProximityCounter_t *prox_counter) {
    if (!prox_counter) {
        return;
    }
    
    uint32_t now = HAL_GetTick();
    if ((now - prox_counter->last_capture_time) > prox_counter->timeout_ms) {
        prox_counter->rpm = 0;
        
        // Reset averaging state
        prox_counter->first_measurement = 1;
        prox_counter->period_sum = 0;
        prox_counter->period_count = 0;
        prox_counter->is_first_captured = 0;
        
        // Reset hysteresis state
        prox_counter->rpm_previous = 0;
        prox_counter->stability_counter = 0;
    }
}

/**
 * @brief Get current RPM value
 */
float ProximityCounter_GetRPM(const ProximityCounter_t *prox_counter) {
    if (!prox_counter) {
        return 0.0f;
    }
    return prox_counter->rpm;
}

/**
 * @brief Get current frequency in Hz
 */
float ProximityCounter_GetFrequency(const ProximityCounter_t *prox_counter) {
    if (!prox_counter) {
        return 0.0f;
    }
    return prox_counter->rpm / 60.0f * prox_counter->ppr;
}

/**
 * @brief Set PPR (Pulses Per Revolution)
 */
void ProximityCounter_SetPPR(ProximityCounter_t *prox_counter, uint32_t ppr) {
    if (!prox_counter || ppr == 0) {
        return;
    }
    prox_counter->ppr = ppr;
}

/**
 * @brief Set diameter
 */
void ProximityCounter_SetDiameter(ProximityCounter_t *prox_counter, float diameter) {
    if (!prox_counter || diameter <= 0.0f) {
        return;
    }
    prox_counter->diameter = diameter;
}

/**
 * @brief Set timeout value
 */
void ProximityCounter_SetTimeout(ProximityCounter_t *prox_counter, uint32_t timeout_ms) {
    if (!prox_counter || timeout_ms == 0) {
        return;
    }
    prox_counter->timeout_ms = timeout_ms;
}

/**
 * @brief Reset all measurement states
 */
void ProximityCounter_Reset(ProximityCounter_t *prox_counter) {
    if (!prox_counter) {
        return;
    }
    
    // Reset measurement variables
    prox_counter->rpm = 0.0f;
    prox_counter->ic_val1 = 0;
    prox_counter->ic_val2 = 0;
    prox_counter->difference = 0;
    prox_counter->is_first_captured = 0;
    prox_counter->overflow_count = 0;
    prox_counter->new_capture_ready = 0;
    
    // Reset averaging variables
    prox_counter->period_sum = 0;
    prox_counter->period_count = 0;
    prox_counter->first_measurement = 1;
    
    // Reset hysteresis variables
    prox_counter->rpm_previous = 0;
    prox_counter->stability_counter = 0;
    
    // Update timestamp
    prox_counter->last_capture_time = HAL_GetTick();
}

/**
 * @brief Update proximity counter configuration
 */
void ProximityCounter_UpdateConfig(ProximityCounter_t *prox_counter, uint32_t ppr, float diameter) {
    if (!prox_counter) {
        return;
    }
    
    prox_counter->ppr = ppr > 0 ? ppr : 1;
    prox_counter->diameter = diameter > 0.0f ? diameter : 0.25f;
    
    // Reset measurement to apply new config
    ProximityCounter_Reset(prox_counter);
}

/**
 * @brief Handle input capture callback - call this from HAL_TIM_IC_CaptureCallback
 */
void ProximityCounter_HandleCapture(ProximityCounter_t *prox_counter, TIM_HandleTypeDef *htim) {
    if (!prox_counter || !htim || htim != prox_counter->htim) {
        return;
    }
    
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        if (prox_counter->is_first_captured == 0) {
            // First rising edge - store initial value
            prox_counter->ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            prox_counter->is_first_captured = 1;
            prox_counter->overflow_count = 0;
            prox_counter->last_capture_time = HAL_GetTick();
        } else {
            // Second rising edge - calculate period
            prox_counter->ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            
            // Calculate difference accounting for overflows
            prox_counter->difference = (prox_counter->overflow_count * 65536UL) + 
                                      prox_counter->ic_val2 - prox_counter->ic_val1;
            
            if (prox_counter->first_measurement) {
                // First measurement: use single period
                prox_counter->new_capture_ready = 1;
                prox_counter->first_measurement = 0;
            } else {
                // Subsequent measurements: collect periods for averaging
                prox_counter->period_sum += prox_counter->difference;
                prox_counter->period_count++;
                
                if (prox_counter->period_count >= prox_counter->averaging_samples) {
                    // Calculate average period
                    prox_counter->difference = prox_counter->period_sum / prox_counter->period_count;
                    prox_counter->new_capture_ready = 1;
                    
                    // Reset for next averaging cycle
                    prox_counter->period_sum = 0;
                    prox_counter->period_count = 0;
                }
            }
            
            prox_counter->last_capture_time = HAL_GetTick();
            prox_counter->ic_val1 = prox_counter->ic_val2;
            prox_counter->overflow_count = 0;
        }
    }
}

/**
 * @brief Handle timer overflow callback - call this from HAL_TIM_PeriodElapsedCallback
 */
void ProximityCounter_HandleOverflow(ProximityCounter_t *prox_counter, TIM_HandleTypeDef *htim) {
    if (!prox_counter || !htim || htim != prox_counter->htim) {
        return;
    }
    
    if (htim->Instance == prox_counter->htim->Instance && prox_counter->is_first_captured == 1) {
        prox_counter->overflow_count++;
    }
}

/**
 * @brief Get current speed in specified unit
 */
float ProximityCounter_GetSpeed(const ProximityCounter_t *prox_counter, ProximitySpeedUnit_t unit) {
    if (!prox_counter) {
        return 0.0f;
    }
    
    if (unit == PROXIMITY_SPEED_UNIT_RPM) {
        return prox_counter->rpm;
    } else {
        // Convert RPM to m/min: RPM * circumference
        float circumference = 3.14159f * prox_counter->diameter;  // π * diameter
        return prox_counter->rpm * circumference;
    }
}

/**
 * @brief Set speed display unit
 */
void ProximityCounter_SetSpeedUnit(ProximityCounter_t *prox_counter, ProximitySpeedUnit_t unit) {
    if (!prox_counter) {
        return;
    }
    prox_counter->speed_unit = unit;
}

/**
 * @brief Get current speed display unit
 */
ProximitySpeedUnit_t ProximityCounter_GetSpeedUnit(const ProximityCounter_t *prox_counter) {
    if (!prox_counter) {
        return PROXIMITY_SPEED_UNIT_RPM;
    }
    return prox_counter->speed_unit;
}

/**
 * @brief Set hysteresis threshold entry
 */
void ProximityCounter_SetHysteresisEntry(ProximityCounter_t *prox_counter, 
                                        uint8_t index, 
                                        uint16_t rpm_threshold, 
                                        uint16_t hysteresis) {
    if (!prox_counter || index >= PROXIMITY_HYSTERESIS_TABLE_SIZE) {
        return;
    }
    
    prox_counter->hysteresis_table[index].rpm_threshold = rpm_threshold;
    prox_counter->hysteresis_table[index].hysteresis = hysteresis;
    
    // Update table size if needed
    if (index >= prox_counter->hysteresis_table_size) {
        prox_counter->hysteresis_table_size = index + 1;
    }
    
    // Sort table by rpm_threshold (insertion sort)
    for (int i = 1; i < prox_counter->hysteresis_table_size; i++) {
        ProximityHysteresisEntry_t key = prox_counter->hysteresis_table[i];
        int j = i - 1;
        
        while (j >= 0 && prox_counter->hysteresis_table[j].rpm_threshold > key.rpm_threshold) {
            prox_counter->hysteresis_table[j + 1] = prox_counter->hysteresis_table[j];
            j--;
        }
        prox_counter->hysteresis_table[j + 1] = key;
    }
}

/**
 * @brief Get hysteresis threshold entry
 */
bool ProximityCounter_GetHysteresisEntry(const ProximityCounter_t *prox_counter, 
                                        uint8_t index, 
                                        ProximityHysteresisEntry_t *entry) {
    if (!prox_counter || !entry || index >= prox_counter->hysteresis_table_size) {
        return false;
    }
    
    *entry = prox_counter->hysteresis_table[index];
    return true;
}

/**
 * @brief Initialize default hysteresis table
 */
void ProximityCounter_InitDefaultHysteresis(ProximityCounter_t *prox_counter) {
    if (!prox_counter) {
        return;
    }
    
    // Default hysteresis table - same as original hard-coded values
    prox_counter->hysteresis_table[0] = (ProximityHysteresisEntry_t){0, 5};     // < 100 RPM: threshold 5
    prox_counter->hysteresis_table[1] = (ProximityHysteresisEntry_t){100, 10};  // 100-499 RPM: threshold 10  
    prox_counter->hysteresis_table[2] = (ProximityHysteresisEntry_t){500, 20};  // 500-799 RPM: threshold 20
    prox_counter->hysteresis_table[3] = (ProximityHysteresisEntry_t){800, 30};  // 800-1099 RPM: threshold 30
    prox_counter->hysteresis_table[4] = (ProximityHysteresisEntry_t){1100, 50}; // >= 1100 RPM: threshold 50
    
    prox_counter->hysteresis_table_size = 5;
}