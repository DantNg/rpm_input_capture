#ifndef ENCODER_INTERRUPT_H
#define ENCODER_INTERRUPT_H

#include "stm32f1xx_hal.h"
#include "measurement_mode.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define M_PI 3.14159265f

typedef struct {
    TIM_HandleTypeDef* htim;
    int16_t ppr;                 // Pulse per revolution (đã nhân 4 nếu dùng encoder x4)
    float wheel_diameter_m;     // Đường kính bánh xe (m)
    uint16_t update_ms;         // Chu kỳ lấy mẫu tính RPM (ms)
    int64_t total_pulse;        // Tổng số xung encoder (gồm cả phần tràn)
    uint32_t last_time_ms;      // Lần đo trước đó
    int64_t last_total_pulse;   // Tổng xung lần đo trước
    
    // Cached values for performance
    float current_length;       // Current length in meters
    float current_rpm;          // Current RPM value
    // uint32_t last_rpm_update;   // No longer needed - timing handled in GetRPM
} Encoder_t;

// Timer initialization function - call this before Encoder_Init
HAL_StatusTypeDef Encoder_InitTimer(TIM_HandleTypeDef* htim);

// Encoder initialization with timer already configured
static inline void Encoder_Init(Encoder_t* enc, TIM_HandleTypeDef* htim, uint16_t ppr, float diameter_m, uint16_t update_ms) {
    if (!enc || !htim || !htim->Instance) return;  // Safety check
    
    enc->htim = htim;
    enc->ppr = ppr * 4;  // dùng chế độ encoder x4
    enc->wheel_diameter_m = diameter_m;
    enc->update_ms = update_ms;
    enc->total_pulse = 0;
    enc->last_total_pulse = 0;
    enc->last_time_ms = HAL_GetTick();
    enc->current_length = 0.0f;
    enc->current_rpm = 0.0f;
    // enc->last_rpm_update = 0; // No longer needed

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(htim, 0);

    // Kích hoạt ngắt tràn timer
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);

    if (htim->Instance == TIM2) {
        HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    } else if (htim->Instance == TIM3) {
        HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
}

// Full encoder initialization with timer configuration
HAL_StatusTypeDef Encoder_InitFull(Encoder_t* enc, TIM_HandleTypeDef* htim, uint16_t ppr, float diameter_m, uint16_t update_ms);

// Gọi hàm này định kỳ để lấy RPM - FIXED VERSION
static inline float Encoder_GetRPM(Encoder_t* enc) {
    if (!enc || !enc->htim) return 0.0f;
    
    uint32_t now = HAL_GetTick();
    uint32_t dt = now - enc->last_time_ms;
    
    // Chỉ tính RPM khi đủ thời gian để có độ chính xác
    if (dt < enc->update_ms) {
        return enc->current_rpm; // Trả về giá trị đã cache
    }

    int64_t pulse_now = enc->total_pulse + (int64_t)__HAL_TIM_GET_COUNTER(enc->htim);
    int64_t delta = pulse_now - enc->last_total_pulse;

    enc->last_total_pulse = pulse_now;
    enc->last_time_ms = now;

    // Tính RPM với thời gian thực tế đã đo
    float rev = (float)delta / (float)enc->ppr;
    float minutes = (float)dt / 60000.0f;
    
    if (minutes > 0) {
        float new_rpm = rev / minutes;
        
        // Simple moving average filter để làm mượt giá trị
        enc->current_rpm = (enc->current_rpm * 0.7f) + (new_rpm * 0.3f);
        return enc->current_rpm;
    }
    
    return enc->current_rpm;
}


// Tính quãng đường đã đi (mét)
static inline float Encoder_GetLengthMeter(Encoder_t* enc) {
    if (!enc || !enc->htim) return 0.0f;
    int64_t pulse_now = enc->total_pulse + (int64_t)__HAL_TIM_GET_COUNTER(enc->htim);
    float rev = (float)pulse_now / (float)enc->ppr;
    float circumference = enc->wheel_diameter_m * M_PI;
    return rev * circumference;
}

// Lấy tổng xung encoder hiện tại
static inline int64_t Encoder_GetPulse(Encoder_t* enc) {
    if (!enc || !enc->htim) return 0;
    return enc->total_pulse + (int64_t)__HAL_TIM_GET_COUNTER(enc->htim);
}

// Convert RPM to linear speed (m/min)
static inline float Encoder_ConvertRPMToMetersPerMin(Encoder_t* enc, float rpm) {
    if (!enc) return 0.0f;
    float circumference = enc->wheel_diameter_m * M_PI;
    return rpm * circumference;
}

// Get current speed in selected display unit
static inline float Encoder_GetCurrentSpeed(Encoder_t* enc, SpeedDisplayUnit_t display_unit) {
    if (!enc) return 0.0f;
    
    float rpm = enc->current_rpm;
    if (display_unit == SPEED_UNIT_RPM) {
        return rpm;
    } else {
        return Encoder_ConvertRPMToMetersPerMin(enc, rpm);
    }
}

// Get speed unit string for display
static inline const char* Encoder_GetSpeedUnitString(SpeedDisplayUnit_t display_unit) {
    return (display_unit == SPEED_UNIT_RPM) ? "RPM" : "m/min";
}

// Process encoder measurements and update holding registers
static inline void Encoder_ProcessMeasurements(Encoder_t* enc, uint16_t* holding_regs, MeasurementMode_t measurement_mode) {
    if (!enc || !holding_regs) return;
    
    uint32_t now = HAL_GetTick();
    
    // Always update length immediately
    enc->current_length = Encoder_GetLengthMeter(enc);
    
    // Update RPM - GetRPM now handles timing internally
    enc->current_rpm = Encoder_GetRPM(enc);
    
    // Update holding registers based on measurement mode
    if (measurement_mode == MEASUREMENT_MODE_RPM) {
        // Get current speed display unit from command handler
        extern SpeedDisplayUnit_t CommandHandler_GetSpeedDisplayUnit(void);
        SpeedDisplayUnit_t speed_unit = CommandHandler_GetSpeedDisplayUnit();
        float display_speed = Encoder_GetCurrentSpeed(enc, speed_unit);
        
        // Pack float speed into two 16-bit registers
        uint32_t speed_bits = 0;
        memcpy(&speed_bits, &display_speed, sizeof(display_speed));
        holding_regs[5] = (uint16_t)(speed_bits >> 16);
        holding_regs[6] = (uint16_t)(speed_bits & 0xFFFF);
        
        // Debug: Print speed values with unit for troubleshooting
        static uint32_t last_debug_time = 0;
        if (now - last_debug_time >= 1000) { // Print every 1 second
            printf("🔄 Speed: %.2f %s | RPM: %.2f | Reg[5]: 0x%04X | Reg[6]: 0x%04X\r\n", 
                   (double)display_speed, Encoder_GetSpeedUnitString(speed_unit),
                   (double)enc->current_rpm, holding_regs[5], holding_regs[6]);
            last_debug_time = now;
        }
    } else {
        // LENGTH mode: Pack float length into two 16-bit registers
        uint32_t len_bits = 0;
        memcpy(&len_bits, &enc->current_length, sizeof(enc->current_length));
        holding_regs[7] = (uint16_t)(len_bits >> 16);  // length high
        holding_regs[8] = (uint16_t)(len_bits & 0xFFFF);  // length low
    }
}

// Get current length value (cached for performance)
static inline float Encoder_GetCurrentLength(Encoder_t* enc) {
    if (!enc) return 0.0f;
    return enc->current_length;
}

// Get current RPM value (cached for performance)
static inline float Encoder_GetCurrentRPM(Encoder_t* enc) {
    if (!enc) return 0.0f;
    return enc->current_rpm;
}

// Function declarations for encoder.c
void Encoder_RegisterInstance(Encoder_t* enc);
void Encoder_HandleTimerOverflow(void);

#endif // ENCODER_INTERRUPT_H
