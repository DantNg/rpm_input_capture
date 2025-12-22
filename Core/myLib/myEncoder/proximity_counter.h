/**
 ******************************************************************************
 * @file    proximity_counter.h
 * @brief   Proximity sensor RPM counter library header
 * @author  Auto-generated
 * @date    December 2025
 ******************************************************************************
 */

#ifndef __PROXIMITY_COUNTER_H
#define __PROXIMITY_COUNTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define PROXIMITY_MAX_RPM_ALLOWED   10000.0f
#define PROXIMITY_NO_PULSE_TIMEOUT_MS 10000UL  // 10 seconds
#define PROXIMITY_TIMCLOCK   72000000UL
#define PROXIMITY_PRESCALAR  72UL
#define PROXIMITY_COUNTER_HZ (PROXIMITY_TIMCLOCK / PROXIMITY_PRESCALAR)
#define PROXIMITY_HYSTERESIS_TABLE_SIZE 10

/* Exported types ------------------------------------------------------------*/
typedef enum {
    PROXIMITY_SPEED_UNIT_RPM = 0,   // Rotations per minute
    PROXIMITY_SPEED_UNIT_M_MIN = 1  // Meters per minute
} ProximitySpeedUnit_t;

typedef struct {
    uint16_t rpm_threshold;  // RPM threshold for this entry
    uint16_t hysteresis;     // Hysteresis value for this threshold
} ProximityHysteresisEntry_t;
typedef struct {
    // Configuration parameters
    uint32_t ppr;                    // Pulses per revolution (default: 1)
    float diameter;                  // Wheel/object diameter in meters
    uint32_t timeout_ms;             // No pulse timeout in milliseconds
    uint32_t averaging_samples;      // Number of samples for averaging (default: 3)
    ProximitySpeedUnit_t speed_unit; // Speed display unit
    
    // Hysteresis configuration table
    ProximityHysteresisEntry_t hysteresis_table[PROXIMITY_HYSTERESIS_TABLE_SIZE];
    uint8_t hysteresis_table_size;   // Number of valid entries in table
    
    // Runtime state variables
    volatile float rpm;              // Current RPM value
    volatile uint32_t last_capture_time;
    volatile uint16_t ic_val1;
    volatile uint16_t ic_val2;
    volatile uint32_t difference;
    volatile int is_first_captured;
    volatile uint32_t overflow_count;
    volatile uint8_t new_capture_ready;
    
    // Averaging variables
    volatile uint32_t period_sum;
    volatile uint8_t period_count;
    volatile uint8_t first_measurement;
    
    // Hysteresis filter variables
    volatile int rpm_previous;
    volatile uint8_t stability_counter;
    
    // Timer handle pointer
    TIM_HandleTypeDef *htim;
} ProximityCounter_t;

typedef struct {
    uint32_t ppr;
    float diameter;
    uint32_t timeout_ms;
    uint32_t averaging_samples;
} ProximityCounterConfig_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Initialize proximity counter
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param config: Pointer to configuration structure
 * @param htim: Pointer to timer handle used for input capture
 * @retval None
 */
void ProximityCounter_Init(ProximityCounter_t *prox_counter, 
                          const ProximityCounterConfig_t *config,
                          TIM_HandleTypeDef *htim);

/**
 * @brief Start proximity counter measurement
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @retval None
 */
void ProximityCounter_Start(ProximityCounter_t *prox_counter);

/**
 * @brief Stop proximity counter measurement
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @retval None
 */
void ProximityCounter_Stop(ProximityCounter_t *prox_counter);

/**
 * @brief Process new capture data and calculate RPM
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @retval None
 * @note Call this function in main loop when new_capture_ready flag is set
 */
void ProximityCounter_ProcessCapture(ProximityCounter_t *prox_counter);

/**
 * @brief Check for timeout and reset if no pulses detected
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @retval None
 * @note Call this function periodically in main loop
 */
void ProximityCounter_CheckTimeout(ProximityCounter_t *prox_counter);

/**
 * @brief Get current RPM value
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @retval Current RPM as float
 */
float ProximityCounter_GetRPM(const ProximityCounter_t *prox_counter);

/**
 * @brief Get current frequency in Hz
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @retval Current frequency as float
 */
float ProximityCounter_GetFrequency(const ProximityCounter_t *prox_counter);

/**
 * @brief Set PPR (Pulses Per Revolution)
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param ppr: New PPR value
 * @retval None
 */
void ProximityCounter_SetPPR(ProximityCounter_t *prox_counter, uint32_t ppr);

/**
 * @brief Set diameter
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param diameter: New diameter in meters
 * @retval None
 */
void ProximityCounter_SetDiameter(ProximityCounter_t *prox_counter, float diameter);

/**
 * @brief Set timeout value
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param timeout_ms: New timeout in milliseconds
 * @retval None
 */
void ProximityCounter_SetTimeout(ProximityCounter_t *prox_counter, uint32_t timeout_ms);

/**
 * @brief Get current speed in specified unit
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param unit: Speed unit (RPM or m/min)
 * @retval Current speed as float
 */
float ProximityCounter_GetSpeed(const ProximityCounter_t *prox_counter, ProximitySpeedUnit_t unit);

/**
 * @brief Set speed display unit
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param unit: New speed unit
 * @retval None
 */
void ProximityCounter_SetSpeedUnit(ProximityCounter_t *prox_counter, ProximitySpeedUnit_t unit);

/**
 * @brief Get current speed display unit
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @retval Current speed unit
 */
ProximitySpeedUnit_t ProximityCounter_GetSpeedUnit(const ProximityCounter_t *prox_counter);

/**
 * @brief Set hysteresis threshold entry
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param index: Index in table (0-9)
 * @param rpm_threshold: RPM threshold
 * @param hysteresis: Hysteresis value
 * @retval None
 */
void ProximityCounter_SetHysteresisEntry(ProximityCounter_t *prox_counter, 
                                        uint8_t index, 
                                        uint16_t rpm_threshold, 
                                        uint16_t hysteresis);

/**
 * @brief Get hysteresis threshold entry
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param index: Index in table (0-9)
 * @param entry: Pointer to store the entry
 * @retval true if valid entry, false otherwise
 */
bool ProximityCounter_GetHysteresisEntry(const ProximityCounter_t *prox_counter, 
                                        uint8_t index, 
                                        ProximityHysteresisEntry_t *entry);

/**
 * @brief Initialize default hysteresis table
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @retval None
 */
void ProximityCounter_InitDefaultHysteresis(ProximityCounter_t *prox_counter);

/**
 * @brief Reset all measurement states
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @retval None
 */
void ProximityCounter_Reset(ProximityCounter_t *prox_counter);

/**
 * @brief Update proximity counter configuration
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param ppr: New PPR value
 * @param diameter: New diameter in meters
 * @retval None
 */
void ProximityCounter_UpdateConfig(ProximityCounter_t *prox_counter, uint32_t ppr, float diameter);

/**
 * @brief Handle input capture callback - call this from HAL_TIM_IC_CaptureCallback
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param htim: Timer handle that triggered the interrupt
 * @retval None
 */
void ProximityCounter_HandleCapture(ProximityCounter_t *prox_counter, TIM_HandleTypeDef *htim);

/**
 * @brief Handle timer overflow callback - call this from HAL_TIM_PeriodElapsedCallback
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param htim: Timer handle that triggered the interrupt
 * @retval None
 */
void ProximityCounter_HandleOverflow(ProximityCounter_t *prox_counter, TIM_HandleTypeDef *htim);

/**
 * @brief Apply hysteresis filter to RPM value using configurable table
 * @param prox_counter: Pointer to ProximityCounter_t structure
 * @param new_rpm: New RPM value to filter
 * @param prev_rpm: Previous RPM value
 * @param stability_counter: Pointer to stability counter
 * @retval Filtered RPM value
 */
int ProximityCounter_ApplyHysteresisFilter(ProximityCounter_t *prox_counter, 
                                         int new_rpm, 
                                         int prev_rpm, 
                                         volatile uint8_t *stability_counter);

#ifdef __cplusplus
}
#endif

#endif /* __PROXIMITY_COUNTER_H */