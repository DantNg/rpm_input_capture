 #include "command_handler.h"
#include "../myEncoder/myEncoder.h"
#include "../myFlash/myFlash.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// UART Port definitions
#define SERIAL_PORT1  1  // UART1 for debug/command
#define SERIAL_PORT2  3  // UART3 for Modbus

// Forward declarations for internal functions
static void Process_BasicCommands(CommandHandler_t *handler, const char* cmd);
static void Process_UARTCommands(CommandHandler_t *handler, const char* cmd);
static void Process_EncoderCommands(CommandHandler_t *handler, const char* cmd);
static void Process_LengthCommands(CommandHandler_t *handler, const char* cmd);
static void Process_ModeCommands(CommandHandler_t *handler, const char* cmd);
static void Process_SpeedUnitCommands(CommandHandler_t *handler, const char* cmd);
static void Process_ModbusCommands(CommandHandler_t *handler, const char* cmd);
static void Process_ProximityCommands(CommandHandler_t *handler, const char* cmd);
static void Show_Help(void);

// Global speed display unit variable
static SpeedDisplayUnit_t current_speed_unit = SPEED_UNIT_RPM;

// Global Modbus control variables
static uint8_t current_slave_id = 0x01;  // Default slave ID
static bool modbus_enabled = true;       // Default enabled

// External functions for proximity counter control (defined in main.c)
extern void SetProximitySpeedUnit(int unit);
extern void SetProximityHysteresis(int index, int rpm_threshold, int hysteresis);
extern void ShowProximityHysteresis(void);
extern void ClearProximityHysteresis(void);
extern void SaveProximityHysteresis(void);
extern void LoadProximityHysteresis(void);

void CommandHandler_Init(CommandHandler_t *handler, CommandHandler_Config_t *config) {
    if (!handler || !config) return;
    
    handler->config = *config;
    handler->cmd_index = 0;
    handler->show_prompt = true;
    memset(handler->cmd_buffer, 0, sizeof(handler->cmd_buffer));
}

void CommandHandler_Process(CommandHandler_t *handler) {
    if (!handler || !handler->config.huart) return;
    
    if (handler->show_prompt) {
        printf("\r\nCmd> ");
        handler->show_prompt = false;
    }
    
    // Check for single character input (non-blocking)
    if (__HAL_UART_GET_FLAG(handler->config.huart, UART_FLAG_RXNE)) {
        uint8_t ch;
        if (HAL_UART_Receive(handler->config.huart, &ch, 1, 0) == HAL_OK) {
            if (ch == '\r' || ch == '\n') {
                // Enter pressed - process command
                if (handler->cmd_index > 0) {
                    handler->cmd_buffer[handler->cmd_index] = '\0';
                    printf("\r\n");
                    
                    // Process commands by category
                    bool command_found = false;
                    
                    // Basic commands
                    if (strcmp(handler->cmd_buffer, "led_on") == 0 || strcmp(handler->cmd_buffer, "led_off") == 0 ||
                        strcmp(handler->cmd_buffer, "status") == 0 || strcmp(handler->cmd_buffer, "help") == 0 ||
                        strcmp(handler->cmd_buffer, "reset") == 0) {
                        Process_BasicCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    // Mode commands
                    else if (strncmp(handler->cmd_buffer, "mode ", 5) == 0) {
                        Process_ModeCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    // Speed unit commands
                    else if (strncmp(handler->cmd_buffer, "speed_unit ", 11) == 0 || strcmp(handler->cmd_buffer, "speed_unit") == 0) {
                        Process_SpeedUnitCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    // Modbus commands
                    else if (strncmp(handler->cmd_buffer, "modbus ", 7) == 0 || strcmp(handler->cmd_buffer, "modbus") == 0) {
                        Process_ModbusCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    // UART commands with port selection
                    else if (strncmp(handler->cmd_buffer, "uart1 ", 6) == 0 || 
                             strncmp(handler->cmd_buffer, "uart3 ", 6) == 0) {
                        Process_UARTCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    // Hysteresis/Proximity commands
                    else if (strncmp(handler->cmd_buffer, "hyst ", 5) == 0 || strcmp(handler->cmd_buffer, "hyst") == 0) {
                        Process_ProximityCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    // Encoder commands
                    else if (strncmp(handler->cmd_buffer, "ppr ", 4) == 0 || 
                             strncmp(handler->cmd_buffer, "dia ", 4) == 0 ||
                             strncmp(handler->cmd_buffer, "time ", 5) == 0) {
                        Process_EncoderCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    // Length commands
                    else if (strcmp(handler->cmd_buffer, "len_reset") == 0 || 
                             strncmp(handler->cmd_buffer, "len_set ", 8) == 0 ||
                             strcmp(handler->cmd_buffer, "len_save") == 0) {
                        Process_LengthCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    
                    if (!command_found) {
                        printf("❌ Unknown: %s (type 'help' for commands)\r\n", handler->cmd_buffer);
                    }
                }
                
                // Reset for next command
                handler->cmd_index = 0;
                memset(handler->cmd_buffer, 0, sizeof(handler->cmd_buffer));
                handler->show_prompt = true;
                
            } else if (ch == '\b' || ch == 127) {
                // Backspace
                if (handler->cmd_index > 0) {
                    handler->cmd_index--;
                    printf("\b \b"); // Erase character on terminal
                }
            } else if (ch >= 32 && ch < 127 && handler->cmd_index < (sizeof(handler->cmd_buffer) - 1)) {
                // Printable character
                handler->cmd_buffer[handler->cmd_index++] = ch;
                printf("%c", ch); // Echo character
            }
        }
    }
}

void CommandHandler_Reset(CommandHandler_t *handler) {
    if (!handler) return;
    
    handler->cmd_index = 0;
    handler->show_prompt = true;
    memset(handler->cmd_buffer, 0, sizeof(handler->cmd_buffer));
}

SpeedDisplayUnit_t CommandHandler_GetSpeedDisplayUnit(void) {
    return current_speed_unit;
}

uint8_t CommandHandler_GetModbusSlaveId(void) {
    return current_slave_id;
}

bool CommandHandler_IsModbusEnabled(void) {
    return modbus_enabled;
}

void CommandHandler_SetModbusConfig(uint8_t slave_id, bool enabled) {
    current_slave_id = slave_id;
    modbus_enabled = enabled;
}

void CommandHandler_InitModbusFromFlash(void) {
    myModbusConfig config;
    myFlash_LoadModbusConfig(&config);
    
    // Check if config is valid (not 0xFF from erased flash)
    if (config.slave_id != 0xFF && config.slave_id >= 1 && config.slave_id <= 247) {
        current_slave_id = config.slave_id;
        modbus_enabled = (config.enabled != 0);
    } else {
        // Use defaults for uninitialized flash
        current_slave_id = 0x01;
        modbus_enabled = true;
    }
}

// Command processing functions implementation
static void Process_BasicCommands(CommandHandler_t *handler, const char* cmd) {
    if (strcmp(cmd, "status") == 0) {
        printf("=== SYSTEM STATUS ===\r\n");
        Encoder_t* enc = (Encoder_t*)handler->config.encoder;
        printf("ENCODER: PPR=%lu DIA=%.3f LEN=%.3fm\r\n", 
               *handler->config.ppr, (double)*handler->config.dia, 
               (double)Encoder_GetCurrentLength(enc));
        printf("UART: BAUD=%lu PARITY=%lu STOP=%lu\r\n",
               (unsigned long)handler->config.huart1->Init.BaudRate, 
               (unsigned long)*handler->config.parity,
               (unsigned long)(handler->config.huart1->Init.StopBits == UART_STOPBITS_2 ? 2 : 1));
        printf("MODBUS: SLAVE_ID=0x%02X TIMEOUT=%lums\r\n", current_slave_id, (unsigned long)*handler->config.time);
        printf("MODBUS STATUS: %s\r\n", modbus_enabled ? "ENABLED" : "DISABLED");
        if (*handler->config.measurement_mode == MEASUREMENT_MODE_LENGTH) {
            printf("MODE: LENGTH measurement (meters)\r\n");
        } else {
            printf("MODE: RPM measurement (rotations/min)\r\n");
            if (current_speed_unit == SPEED_UNIT_RPM) {
                printf("SPEED UNIT: RPM (rotations per minute)\r\n");
            } else {
                printf("SPEED UNIT: m/min (meters per minute)\r\n");
            }
            
            // Show current speed in selected unit
            Encoder_t* enc = (Encoder_t*)handler->config.encoder;
            if (enc) {
                float current_speed = Encoder_GetCurrentSpeed(enc, current_speed_unit);
                printf("CURRENT SPEED: %.2f %s\r\n", 
                       (double)current_speed, 
                       Encoder_GetSpeedUnitString(current_speed_unit));
            }
        }
    } else if (strcmp(cmd, "help") == 0) {
        Show_Help();
    } else if (strcmp(cmd, "reset") == 0) {
        printf("System resetting...\r\n");
        HAL_Delay(100);
        HAL_NVIC_SystemReset();
    }
}

static void Process_UARTCommands(CommandHandler_t *handler, const char* cmd) {
    uint8_t port = 0;
    const char* param_start = NULL;
    
    // Determine which UART port
    if (strncmp(cmd, "uart1 ", 6) == 0) {
        port = SERIAL_PORT1;
        param_start = cmd + 6;
    } else if (strncmp(cmd, "uart3 ", 6) == 0) {
        port = SERIAL_PORT2;
        param_start = cmd + 6;
    } else {
        printf("❌ Invalid UART command format. Use: uart1/uart3 <command>\r\n");
        return;
    }
    
    UART_HandleTypeDef* target_uart = (port == SERIAL_PORT1) ? handler->config.huart1 : handler->config.huart3;
    
    if (strncmp(param_start, "baud ", 5) == 0) {
        uint32_t baud = atoi(param_start + 5);
        if (baud >= 2400 && baud <= 921600) {
            target_uart->Init.BaudRate = baud;
            if (handler->config.reinit_uarts) {
                handler->config.reinit_uarts();
            }
            
            // Save to Flash only if both UARTs have valid settings
            typedef struct {
                uint32_t baudRate;
                uint32_t parity;
                uint32_t stopBits;
                uint32_t frameTimeoutMs;
            } myUARTParams;
            
            myUARTParams p = {
                .baudRate = baud,
                .parity = *handler->config.parity,
                .stopBits = (target_uart->Init.StopBits == UART_STOPBITS_2) ? 2U : 1U,
                .frameTimeoutMs = *handler->config.time
            };
            if (handler->config.save_uart_params && handler->config.save_uart_params(&p) == HAL_OK) {
                printf("✅ UART%d BAUD set to %lu and saved\r\n", port, (unsigned long)baud);
            } else {
                printf("✅ UART%d BAUD set to %lu (save failed)\r\n", port, (unsigned long)baud);
            }
        } else {
            printf("❌ Invalid baud rate (2400-921600)\r\n");
        }
    } else if (strncmp(param_start, "parity ", 7) == 0) {
        uint32_t par = atoi(param_start + 7);
        if (par <= 2) {
            // Parity affects both UARTs globally
            *handler->config.parity = par;
            if (handler->config.apply_parity_config) {
                handler->config.apply_parity_config(*handler->config.parity);
            }
            if (handler->config.reinit_uarts) {
                handler->config.reinit_uarts();
            }
            printf("✅ PARITY set to %lu (0=None,1=Odd,2=Even) for all UARTs\r\n", (unsigned long)*handler->config.parity);
        } else {
            printf("❌ Invalid parity (0=None, 1=Odd, 2=Even)\r\n");
        }
    } else if (strncmp(param_start, "stop ", 5) == 0) {
        uint32_t stop = atoi(param_start + 5);
        if (stop == 1 || stop == 2) {
            target_uart->Init.StopBits = (stop == 2) ? UART_STOPBITS_2 : UART_STOPBITS_1;
            if (handler->config.reinit_uarts) {
                handler->config.reinit_uarts();
            }
            printf("✅ UART%d STOP BITS set to %lu\r\n", port, (unsigned long)stop);
        } else {
            printf("❌ Invalid stop bits (1 or 2)\r\n");
        }
    } else {
        printf("❌ Unknown UART command. Available: baud, parity, stop\r\n");
    }
}

static void Process_EncoderCommands(CommandHandler_t *handler, const char* cmd) {
    if (strncmp(cmd, "ppr ", 4) == 0) {
        uint32_t new_ppr = atoi(cmd + 4);
        if (new_ppr > 0 && new_ppr <= 10000) {
            *handler->config.ppr = new_ppr;
            if (handler->config.encoder_init) {
                handler->config.encoder_init(handler->config.encoder, handler->config.htim, 
                                           *handler->config.ppr, *handler->config.dia, *handler->config.time);
            }
            
            // Save to Flash - need to define myEncoderParams structure
            typedef struct {
                uint32_t diameter;
                uint32_t pulsesPerRev;
            } myEncoderParams;
            
            myEncoderParams enc_params = {
                .diameter = (uint32_t)(*handler->config.dia * 1000),
                .pulsesPerRev = *handler->config.ppr
            };
            if (handler->config.save_encoder_params && 
                handler->config.save_encoder_params(&enc_params) == HAL_OK) {
                printf("✅ PPR set to %lu and saved\r\n", (unsigned long)*handler->config.ppr);
            }
        } else {
            printf("❌ Invalid PPR (1-10000)\r\n");
        }
    } else if (strncmp(cmd, "dia ", 4) == 0) {
        float new_dia = atof(cmd + 4);
        if (new_dia > 0.001 && new_dia < 10.0) {
            *handler->config.dia = new_dia;
            if (handler->config.encoder_init) {
                handler->config.encoder_init(handler->config.encoder, handler->config.htim, 
                                           *handler->config.ppr, *handler->config.dia, *handler->config.time);
            }
            printf("✅ DIA set to %.3f\r\n", (double)*handler->config.dia);
        } else {
            printf("❌ Invalid diameter (0.001-10.0 meters)\r\n");
        }
    } else if (strncmp(cmd, "time ", 5) == 0) {
        uint32_t new_time = atoi(cmd + 5);
        if (new_time >= 10 && new_time <= 10000) {
            *handler->config.time = new_time;
            if (handler->config.encoder_init) {
                handler->config.encoder_init(handler->config.encoder, handler->config.htim, 
                                           *handler->config.ppr, *handler->config.dia, *handler->config.time);
            }
            printf("✅ SAMPLE TIME set to %lums\r\n", (unsigned long)*handler->config.time);
        } else {
            printf("❌ Invalid time (10-10000ms)\r\n");
        }
    }
}

static void Process_LengthCommands(CommandHandler_t *handler, const char* cmd) {
    if (strcmp(cmd, "len_reset") == 0) {
        // Reset encoder internal length counter
        Encoder_t* enc = (Encoder_t*)handler->config.encoder;
        if (enc) {
            enc->total_pulse = 0;
            enc->current_length = 0.0f;
            __HAL_TIM_SET_COUNTER(enc->htim, 0);
        }
        if (handler->config.save_length && handler->config.save_length(0) == HAL_OK) {
            printf("✅ Length reset to 0 and saved\r\n");
        }
    } else if (strncmp(cmd, "len_set ", 8) == 0) {
        printf("❌ len_set command deprecated. Use len_reset to reset length counter.\r\n");
        printf("   Length is now measured continuously by encoder.\r\n");
    } else if (strcmp(cmd, "len_save") == 0) {
        Encoder_t* enc = (Encoder_t*)handler->config.encoder;
        float current_length = enc ? Encoder_GetCurrentLength(enc) : 0.0f;
        uint32_t length_mm = (uint32_t)(current_length * 1000);
        if (handler->config.save_length && handler->config.save_length(length_mm) == HAL_OK) {
            printf("✅ Current length %.3fm saved to Flash\r\n", (double)current_length);
        } else {
            printf("❌ Failed to save length\r\n");
        }
    }
}

static void Process_ModeCommands(CommandHandler_t *handler, const char* cmd) {
    if (strncmp(cmd, "mode length", 11) == 0) {
        *handler->config.measurement_mode = MEASUREMENT_MODE_LENGTH;
        if (handler->config.save_measurement_mode && 
            handler->config.save_measurement_mode(MEASUREMENT_MODE_LENGTH) == HAL_OK) {
            printf("✅ Switched to LENGTH measurement mode and saved\r\n");
        } else {
            printf("✅ Switched to LENGTH measurement mode (save failed)\r\n");
        }
        printf("📏 System now measuring length in meters\r\n");
        // Reset encoder length when switching to length mode
        Encoder_t* enc = (Encoder_t*)handler->config.encoder;
        if (enc) {
            enc->total_pulse = 0;
            enc->current_length = 0.0f;
            __HAL_TIM_SET_COUNTER(enc->htim, 0);
        }
        if (handler->config.encoder_init) {
            handler->config.encoder_init(handler->config.encoder, handler->config.htim,
                                       *handler->config.ppr, *handler->config.dia, *handler->config.time);
        }
    } else if (strncmp(cmd, "mode rpm", 8) == 0) {
        *handler->config.measurement_mode = MEASUREMENT_MODE_RPM;
        if (handler->config.save_measurement_mode && 
            handler->config.save_measurement_mode(MEASUREMENT_MODE_RPM) == HAL_OK) {
            printf("✅ Switched to RPM measurement mode and saved\r\n");
        } else {
            printf("✅ Switched to RPM measurement mode (save failed)\r\n");
        }
        printf("🔄 System now measuring rotational speed\r\n");
        if (handler->config.encoder_init) {
            handler->config.encoder_init(handler->config.encoder, handler->config.htim,
                                       *handler->config.ppr, *handler->config.dia, *handler->config.time);
        }
    } else if (strcmp(cmd, "mode") == 0) {
        printf("=== CURRENT MODE ===\r\n");
        if (*handler->config.measurement_mode == MEASUREMENT_MODE_LENGTH) {
            printf("📏 Current mode: LENGTH measurement\r\n");
            printf("📊 Measuring: Distance in meters\r\n");
        } else {
            printf("🔄 Current mode: RPM measurement\r\n");
            printf("📊 Measuring: Rotational speed\r\n");
        }
    } else {
        printf("❌ Invalid mode. Available: length, rpm\r\n");
    }
}

static void Process_ModbusCommands(CommandHandler_t *handler, const char* cmd) {
    if (strcmp(cmd, "modbus") == 0) {
        printf("=== MODBUS STATUS ===\r\n");
        printf("🔗 SLAVE ID: 0x%02X (%d)\r\n", current_slave_id, current_slave_id);
        printf("📡 STATUS: %s\r\n", modbus_enabled ? "ENABLED" : "DISABLED");
        printf("⏱️  TIMEOUT: %lums\r\n", (unsigned long)*handler->config.time);
        if (!modbus_enabled) {
            printf("⚠️  Note: Modbus communication is currently disabled\r\n");
        }
    } else if (strncmp(cmd, "modbus id ", 10) == 0) {
        // Parse slave ID (support both hex and decimal)
        const char* id_str = cmd + 10;
        uint32_t new_id = 0;
        
        if (strncmp(id_str, "0x", 2) == 0 || strncmp(id_str, "0X", 2) == 0) {
            // Hex format
            new_id = strtol(id_str, NULL, 16);
        } else {
            // Decimal format
            new_id = atoi(id_str);
        }
        
        if (new_id >= 1 && new_id <= 247) {  // Valid Modbus slave ID range
            current_slave_id = (uint8_t)new_id;
            
            // Save to Flash
            myModbusConfig config = {
                .slave_id = current_slave_id,
                .enabled = (uint8_t)modbus_enabled,
                .reserved = 0
            };
            
            if (myFlash_SaveModbusConfig(&config) == HAL_OK) {
                printf("✅ Modbus SLAVE ID set to 0x%02X (%d) and saved\r\n", current_slave_id, current_slave_id);
            } else {
                printf("✅ Modbus SLAVE ID set to 0x%02X (%d) (save failed)\r\n", current_slave_id, current_slave_id);
            }
            
            printf("⚠️  Note: System restart required for ID change to take effect\r\n");
        } else {
            printf("❌ Invalid SLAVE ID. Valid range: 1-247 (0x01-0xF7)\r\n");
            printf("💡 Use: modbus id 1 or modbus id 0x01\r\n");
        }
    } else if (strncmp(cmd, "modbus enable", 13) == 0) {
        modbus_enabled = true;
        printf("✅ Modbus communication ENABLED\r\n");
        printf("📡 System will process Modbus frames\r\n");
        
        // Save to Flash
        myModbusConfig config = {
            .slave_id = current_slave_id,
            .enabled = (uint8_t)modbus_enabled,
            .reserved = 0
        };
        myFlash_SaveModbusConfig(&config);
    } else if (strncmp(cmd, "modbus disable", 14) == 0) {
        modbus_enabled = false;
        printf("⚠️  Modbus communication DISABLED\r\n");
        printf("🚫 System will ignore Modbus frames\r\n");
        
        // Save to Flash
        myModbusConfig config = {
            .slave_id = current_slave_id,
            .enabled = (uint8_t)modbus_enabled,
            .reserved = 0
        };
        myFlash_SaveModbusConfig(&config);
    } else {
        printf("❌ Invalid Modbus command. Available: id, enable, disable\r\n");
        printf("💡 Use 'modbus' to show current status\r\n");
    }
}

static void Process_SpeedUnitCommands(CommandHandler_t *handler, const char* cmd) {
    if (strcmp(cmd, "speed_unit") == 0) {
        printf("=== CURRENT SPEED DISPLAY UNIT ===\r\n");
        if (current_speed_unit == SPEED_UNIT_RPM) {
            printf("🔄 Current unit: RPM (rotations per minute)\r\n");
        } else {
            printf("📏 Current unit: m/min (meters per minute)\r\n");
        }
        printf("💡 Note: Speed unit applies to RPM measurement mode\r\n");
    } else if (strncmp(cmd, "speed_unit rpm", 14) == 0) {
        current_speed_unit = SPEED_UNIT_RPM;
        SetProximitySpeedUnit(0); // 0 = RPM
        printf("✅ Speed display unit set to RPM (rotations per minute)\r\n");
        printf("🔄 Speed will be displayed as rotational speed\r\n");
    } else if (strncmp(cmd, "speed_unit m/min", 16) == 0) {
        current_speed_unit = SPEED_UNIT_M_MIN;
        SetProximitySpeedUnit(1); // 1 = m/min
        printf("✅ Speed display unit set to m/min (meters per minute)\r\n");
        printf("📏 Speed will be displayed as linear speed\r\n");
    } else {
        printf("❌ Invalid speed unit. Available: rpm, m/min\r\n");
        printf("💡 Use 'speed_unit' to show current setting\r\n");
    }
}

static void Show_Help(void) {
    printf("=== COMMAND HELP ===\r\n");
    printf("BASIC:\r\n");
    printf("  help         - Show this help\r\n");
    printf("  status       - Show system status\r\n");
    printf("  reset        - System reset\r\n");
    printf("MODE:\r\n");
    printf("  mode         - Show current measurement mode\r\n");
    printf("  mode length  - Switch to length measurement mode\r\n");
    printf("  mode rpm     - Switch to RPM measurement mode\r\n");
    printf("UART CONFIG:\r\n");
    printf("  uart1 baud <rate>  - Set UART1 baud rate (2400-921600)\r\n");
    printf("  uart3 baud <rate>  - Set UART3 baud rate (2400-921600)\r\n");
    printf("  uart1 parity <n>   - Set parity (0=None,1=Odd,2=Even) [global]\r\n");
    printf("  uart1 stop <n>     - Set UART1 stop bits (1 or 2)\r\n");
    printf("  uart3 stop <n>     - Set UART3 stop bits (1 or 2)\r\n");
    printf("ENCODER CONFIG:\r\n");
    printf("  ppr <n>      - Set pulses per revolution (1-10000)\r\n");
    printf("  dia <f>      - Set diameter in meters (0.001-10.0)\r\n");
    printf("  time <ms>    - Set sample time (10-10000ms)\r\n");
    printf("LENGTH:\r\n");
    printf("  len_reset    - Reset length to 0\r\n");
    printf("  len_set <f>  - Set length in meters (0-10000)\r\n");
    printf("  len_save     - Save current length to Flash\r\n");
    printf("SPEED DISPLAY:\r\n");
    printf("  speed_unit       - Show current speed display unit\r\n");
    printf("  speed_unit rpm   - Set speed display to RPM\r\n");
    printf("  speed_unit m/min - Set speed display to m/min\r\n");
    printf("MODBUS CONFIG:\r\n");
    printf("  modbus           - Show Modbus status\r\n");
    printf("  modbus id <n>    - Set Modbus SLAVE ID (0x01-0xF7)\r\n");
    printf("  modbus enable    - Enable Modbus communication\r\n");
    printf("  modbus disable   - Disable Modbus communication\r\n");
    printf("HYSTERESIS CONFIG:\r\n");
    printf("  hyst             - Show hysteresis table\r\n");
    printf("  hyst set <i> <rpm> <h> - Set hysteresis entry\r\n");
    printf("  hyst add <rpm> <h>     - Add hysteresis entry (auto index)\r\n");
    printf("  hyst clear       - Clear all entries\r\n");
    printf("  hyst default     - Restore default table\r\n");
    printf("  hyst save/load   - Save/Load to Flash\r\n");
}

/**
 * @brief Process proximity counter/hysteresis commands
 */
static void Process_ProximityCommands(CommandHandler_t *handler, const char* cmd) {
    if (strcmp(cmd, "hyst") == 0 || strcmp(cmd, "hyst show") == 0) {
        ShowProximityHysteresis();
        
    } else if (strncmp(cmd, "hyst set ", 9) == 0) {
        // Parse: hyst set <index> <rpm_threshold> <hysteresis>
        const char* params = cmd + 9;
        int index, rpm_threshold, hysteresis;
        
        if (sscanf(params, "%d %d %d", &index, &rpm_threshold, &hysteresis) == 3) {
            if (index >= 0 && index < 10 && rpm_threshold >= 0 && hysteresis > 0 && hysteresis <= 1000) {
                SetProximityHysteresis(index, rpm_threshold, hysteresis);
            } else {
                printf("❌ Invalid parameters:\r\n");
                printf("   Index: 0-9\r\n");
                printf("   RPM threshold: >= 0\r\n");
                printf("   Hysteresis: 1-1000\r\n");
            }
        } else {
            printf("❌ Invalid format. Use: hyst set <index> <rpm_threshold> <hysteresis>\r\n");
            printf("💡 Example: hyst set 2 500 15\r\n");
        }
        
    } else if (strcmp(cmd, "hyst clear") == 0) {
        ClearProximityHysteresis();
        
    } else if (strcmp(cmd, "hyst save") == 0) {
        SaveProximityHysteresis();
        
    } else if (strcmp(cmd, "hyst load") == 0) {
        LoadProximityHysteresis();
        
    } else if (strcmp(cmd, "hyst default") == 0) {
        printf("🔄 Restoring default hysteresis table...\r\n");
        ClearProximityHysteresis();
        // Set default values
        SetProximityHysteresis(0, 0, 5);
        SetProximityHysteresis(1, 100, 10);
        SetProximityHysteresis(2, 500, 20);
        SetProximityHysteresis(3, 800, 30);
        SetProximityHysteresis(4, 1100, 50);
        printf("✅ Default hysteresis table restored\r\n");
        
    } else if (strncmp(cmd, "hyst add ", 9) == 0) {
        // Parse: hyst add <rpm_threshold> <hysteresis> - automatically find next free index
        const char* params = cmd + 9;
        int rpm_threshold, hysteresis;
        
        if (sscanf(params, "%d %d", &rpm_threshold, &hysteresis) == 2) {
            if (rpm_threshold >= 0 && hysteresis > 0 && hysteresis <= 1000) {
                // Find next available index (simple implementation)
                static int next_index = 0;
                if (next_index < 10) {
                    SetProximityHysteresis(next_index, rpm_threshold, hysteresis);
                    next_index++;
                } else {
                    printf("❌ Hysteresis table full (max 10 entries)\r\n");
                    printf("💡 Use 'hyst clear' or 'hyst set <index>' to modify existing entries\r\n");
                }
            } else {
                printf("❌ Invalid parameters: RPM >= 0, Hysteresis 1-1000\r\n");
            }
        } else {
            printf("❌ Invalid format. Use: hyst add <rpm_threshold> <hysteresis>\r\n");
            printf("💡 Example: hyst add 1500 60\r\n");
        }
        
    } else {
        printf("❌ Unknown hysteresis command\r\n");
        printf("📚 Available commands:\r\n");
        printf("  hyst              - Show current hysteresis table\r\n");
        printf("  hyst show         - Show current hysteresis table\r\n");
        printf("  hyst set <i> <rpm> <h> - Set entry at index i\r\n");
        printf("  hyst add <rpm> <h>     - Add new entry (auto index)\r\n");
        printf("  hyst clear        - Clear all entries\r\n");
        printf("  hyst default      - Restore default table\r\n");
        printf("  hyst save         - Save table to Flash\r\n");
        printf("  hyst load         - Load table from Flash\r\n");
        printf("💡 Parameters: i=index(0-9), rpm=threshold(>=0), h=hysteresis(1-1000)\r\n");
    }
}
