 #include "command_handler.h"
#include "../myEncoder/myEncoder.h"
#include "../myEncoder/proximity_counter.h"
#include "../myFlash/myFlash.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// UART Port definitions
#define SERIAL_PORT1  1  // UART1 for debug/command
#define SERIAL_PORT2  3  // UART3 for Modbus

// Forward declarations for internal functions
static void Process_BasicCommands(CommandHandler_t *handler, const char* cmd);
static void Process_DebugCommands(CommandHandler_t *handler, const char* cmd);
static void Process_UARTCommands(CommandHandler_t *handler, const char* cmd);
static void Process_ModbusUARTCommands(CommandHandler_t *handler, const char* cmd);
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
static bool debug_enabled = true;
static uint32_t debug_interval_ms = 1000;
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
                    else if (strncmp(handler->cmd_buffer, "debug ", 6) == 0 || strcmp(handler->cmd_buffer, "debug") == 0) {
                        Process_DebugCommands(handler, handler->cmd_buffer);
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
                    // Modbus commands (check modbus uart first to avoid conflict)
                    else if (strncmp(handler->cmd_buffer, "modbus uart ", 12) == 0) {
                        Process_ModbusUARTCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    else if (strncmp(handler->cmd_buffer, "modbus ", 7) == 0 || strcmp(handler->cmd_buffer, "modbus") == 0) {
                        Process_ModbusCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    // UART1 commands only
                    else if (strncmp(handler->cmd_buffer, "uart1 ", 6) == 0) {
                        Process_UARTCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    // Hysteresis/Proximity commands
                    else if (strncmp(handler->cmd_buffer, "hyst ", 5) == 0 || strcmp(handler->cmd_buffer, "hyst") == 0) {
                        Process_ProximityCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    // Proximity status command
                    else if (strcmp(handler->cmd_buffer, "proximity_setting") == 0) {
                        Process_ProximityCommands(handler, handler->cmd_buffer);
                        command_found = true;
                    }
                    // Encoder commands
                    else if (strncmp(handler->cmd_buffer, "ppr ", 4) == 0 || 
                             strncmp(handler->cmd_buffer, "dia ", 4) == 0 ||
                             strncmp(handler->cmd_buffer, "sampletime ", 11) == 0||
                             strncmp(handler->cmd_buffer, "timeout ", 8) == 0) {
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

void CommandHandler_InitSpeedUnitFromFlash(void) {
    mySpeedUnitConfig config;
    myFlash_LoadSpeedUnitConfig(&config);
    
    // Check if config is valid (not 0xFF from erased flash)
    if (config.speed_unit == 0 || config.speed_unit == 1) {
        current_speed_unit = (config.speed_unit == 0) ? SPEED_UNIT_RPM : SPEED_UNIT_M_MIN;
    } else {
        // Use default for uninitialized flash
        current_speed_unit = SPEED_UNIT_RPM;
    }
}

void CommandHandler_SetDebugConfig(bool enabled, uint32_t interval_ms) {
    debug_enabled = enabled;
    debug_interval_ms = interval_ms;
}

void CommandHandler_GetDebugConfig(bool *enabled, uint32_t *interval_ms) {
    if (enabled) {
        *enabled = debug_enabled;
    }
    if (interval_ms) {
        *interval_ms = debug_interval_ms;
    }
}

void CommandHandler_InitDebugConfigFromFlash(void) {
    // Load debug config from flash
    myDebugConfig config;
    myFlash_LoadDebugConfig(&config);
    
    debug_enabled = (config.enabled != 0);
    if( config.interval > 0 && config.interval <10000 ) {
        debug_interval_ms = config.interval;
    } else {
        debug_interval_ms = 1000; // Default interval
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
        
        // Display UART1 (Debug/Command port) configuration details
        UART_HandleTypeDef* debug_uart = handler->config.huart1;
        if (debug_uart) {
            printf("=== UART1 (DEBUG/COMMAND) CONFIG ===\r\n");
            printf("🔌 BAUD RATE: %lu bps\r\n", (unsigned long)debug_uart->Init.BaudRate);
            
            // Parity information
            const char* parity_str;
            switch (debug_uart->Init.Parity) {
                case UART_PARITY_NONE: parity_str = "NONE"; break;
                case UART_PARITY_ODD:  parity_str = "ODD";  break;
                case UART_PARITY_EVEN: parity_str = "EVEN"; break;
                default: parity_str = "UNKNOWN"; break;
            }
            printf("🔧 PARITY: %s\r\n", parity_str);
            
            // Stop bits information
            const char* stopbits_str;
            uint8_t stopbits_num;
            if (debug_uart->Init.StopBits == UART_STOPBITS_1) {
                stopbits_str = "1 bit";
                stopbits_num = 1;
            } else if (debug_uart->Init.StopBits == UART_STOPBITS_2) {
                stopbits_str = "2 bits";
                stopbits_num = 2;
            } else {
                stopbits_str = "UNKNOWN";
                stopbits_num = 0;
            }
            printf("🛑 STOP BITS: %s (%d)\r\n", stopbits_str, stopbits_num);
            
            // Word length information
            const char* wordlen_str;
            uint8_t wordlen_num;
            if (debug_uart->Init.WordLength == UART_WORDLENGTH_8B) {
                wordlen_str = "8 bits";
                wordlen_num = 8;
            } else if (debug_uart->Init.WordLength == UART_WORDLENGTH_9B) {
                wordlen_str = "9 bits";
                wordlen_num = 9;
            } else {
                wordlen_str = "UNKNOWN";
                wordlen_num = 0;
            }
            printf("📏 WORD LENGTH: %s (%d)\r\n", wordlen_str, wordlen_num);
            
            // Hardware flow control
            const char* flow_str;
            switch (debug_uart->Init.HwFlowCtl) {
                case UART_HWCONTROL_NONE:    flow_str = "NONE"; break;
                case UART_HWCONTROL_RTS:     flow_str = "RTS"; break;
                case UART_HWCONTROL_CTS:     flow_str = "CTS"; break;
                case UART_HWCONTROL_RTS_CTS: flow_str = "RTS_CTS"; break;
                default: flow_str = "UNKNOWN"; break;
            }
            printf("🌊 FLOW CONTROL: %s\r\n", flow_str);
            
            // Mode (TX/RX)
            const char* mode_str;
            switch (debug_uart->Init.Mode) {
                case UART_MODE_RX:    mode_str = "RX only"; break;
                case UART_MODE_TX:    mode_str = "TX only"; break;
                case UART_MODE_TX_RX: mode_str = "TX_RX"; break;
                default: mode_str = "UNKNOWN"; break;
            }
            printf("↔️  MODE: %s\r\n", mode_str);
        }
        
        // Display UART3 (Modbus port) configuration details
        UART_HandleTypeDef* modbus_uart = handler->config.huart3;
        if (modbus_uart) {
            printf("=== UART3 (MODBUS) CONFIG ===\r\n");
            printf("🔌 BAUD RATE: %lu bps\r\n", (unsigned long)modbus_uart->Init.BaudRate);
            
            // Parity information
            const char* parity_str;
            switch (modbus_uart->Init.Parity) {
                case UART_PARITY_NONE: parity_str = "NONE"; break;
                case UART_PARITY_ODD:  parity_str = "ODD";  break;
                case UART_PARITY_EVEN: parity_str = "EVEN"; break;
                default: parity_str = "UNKNOWN"; break;
            }
            printf("🔧 PARITY: %s\r\n", parity_str);
            
            // Stop bits information
            const char* stopbits_str;
            uint8_t stopbits_num;
            if (modbus_uart->Init.StopBits == UART_STOPBITS_1) {
                stopbits_str = "1 bit";
                stopbits_num = 1;
            } else if (modbus_uart->Init.StopBits == UART_STOPBITS_2) {
                stopbits_str = "2 bits";
                stopbits_num = 2;
            } else {
                stopbits_str = "UNKNOWN";
                stopbits_num = 0;
            }
            printf("🛑 STOP BITS: %s (%d)\r\n", stopbits_str, stopbits_num);
            
            // Word length information
            const char* wordlen_str;
            uint8_t wordlen_num;
            if (modbus_uart->Init.WordLength == UART_WORDLENGTH_8B) {
                wordlen_str = "8 bits";
                wordlen_num = 8;
            } else if (modbus_uart->Init.WordLength == UART_WORDLENGTH_9B) {
                wordlen_str = "9 bits";
                wordlen_num = 9;
            } else {
                wordlen_str = "UNKNOWN";
                wordlen_num = 0;
            }
            printf("📏 WORD LENGTH: %s (%d)\r\n", wordlen_str, wordlen_num);
            
            // Hardware flow control
            const char* flow_str;
            switch (modbus_uart->Init.HwFlowCtl) {
                case UART_HWCONTROL_NONE:    flow_str = "NONE"; break;
                case UART_HWCONTROL_RTS:     flow_str = "RTS"; break;
                case UART_HWCONTROL_CTS:     flow_str = "CTS"; break;
                case UART_HWCONTROL_RTS_CTS: flow_str = "RTS_CTS"; break;
                default: flow_str = "UNKNOWN"; break;
            }
            printf("🌊 FLOW CONTROL: %s\r\n", flow_str);
            
            // Mode (TX/RX)
            const char* mode_str;
            switch (modbus_uart->Init.Mode) {
                case UART_MODE_RX:    mode_str = "RX only"; break;
                case UART_MODE_TX:    mode_str = "TX only"; break;
                case UART_MODE_TX_RX: mode_str = "TX_RX"; break;
                default: mode_str = "UNKNOWN"; break;
            }
            printf("↔️  MODE: %s\r\n", mode_str);
        }
        
        printf("=== MODBUS STATUS ===\r\n");
        printf("MODBUS: SLAVE_ID=0x%02X \r\n", current_slave_id);
        printf("MODBUS STATUS: %s\r\n", modbus_enabled ? "ENABLED" : "DISABLED");
        
        if (*handler->config.measurement_mode == MEASUREMENT_MODE_LENGTH) {
            printf("=== MEASUREMENT MODE ===\r\n");
            printf("MODE: LENGTH measurement (meters)\r\n");
        } else {
            printf("=== MEASUREMENT MODE ===\r\n");
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

static void Process_DebugCommands(CommandHandler_t *handler, const char* cmd) {
    myDebugConfig debug_config;
    myFlash_LoadDebugConfig(&debug_config);
    
    if (strcmp(cmd, "debug") == 0) {
        printf("=== DEBUG CONFIGURATION ===\r\n");
        printf("DEBUG MESSAGES: %s\r\n", debug_config.enabled ? "ENABLED" : "DISABLED");
        printf("DEBUG INTERVAL: %lu ms\r\n", (unsigned long)debug_config.interval);
    } else if (strcmp(cmd, "debug on") == 0) {
        debug_config.enabled = 1;
        if (myFlash_SaveDebugConfig(&debug_config) == HAL_OK) {
            printf("✅ Debug messages ENABLED and saved\r\n Please restart the system to apply changes.\r\n");
            // CommandHandler_SetDebugConfig(true, debug_config.interval);
        } else {
            printf("⚠️ Debug messages ENABLED but save failed\r\n");
        }
    } else if (strcmp(cmd, "debug off") == 0) {
        debug_config.enabled = 0;
        if (myFlash_SaveDebugConfig(&debug_config) == HAL_OK) {
            printf("✅ Debug messages DISABLED and saved\r\n Please restart the system to apply changes.\r\n");
            // CommandHandler_SetDebugConfig(false, debug_config.interval);
        } else {
            printf("⚠️ Debug messages DISABLED but save failed\r\n");
        }
    } else if (strncmp(cmd, "debug interval ", 15) == 0) {
        uint32_t interval = atoi(cmd + 15);
        if (interval >= 100 && interval <= 10000) {
            debug_config.interval = interval;
            if (myFlash_SaveDebugConfig(&debug_config) == HAL_OK) {
                printf("✅ Debug interval set to %lu ms and saved\r\n Please restart the system to apply changes.\r\n", (unsigned long)interval);
                // CommandHandler_SetDebugConfig(debug_config.enabled != 0, interval);
            } else {
                printf("⚠️ Debug interval set to %lu ms but save failed\r\n", (unsigned long)interval);
            }
        } else {
            printf("❌ Invalid interval. Must be between 100 and 10000 ms\r\n");
        }
    } else {
        printf("❌ Unknown debug command. Available: on, off, interval <ms>\r\n");
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

static void Process_ModbusUARTCommands(CommandHandler_t *handler, const char* cmd) {
    if (strncmp(cmd, "modbus uart ", 12) == 0) {
        const char* param_start = cmd + 12;
        
        if (strncmp(param_start, "baud ", 5) == 0) {
            uint32_t baud = atoi(param_start + 5);
            if (baud >= 2400 && baud <= 921600) {
                // Save Modbus UART params independently
                myModbusUARTParams modbus_params;
                myFlash_LoadModbusUARTParams(&modbus_params);
                modbus_params.baudRate = baud;
                modbus_params.parity = *handler->config.parity;
                modbus_params.stopBits = (handler->config.huart3->Init.StopBits == UART_STOPBITS_2) ? 2U : 1U;
                modbus_params.frameTimeoutMs = *handler->config.time;
                // Apply baud rate to UART3 only
                handler->config.huart3->Init.BaudRate = baud;
                HAL_UART_DeInit(handler->config.huart3);
                HAL_UART_Init(handler->config.huart3);
                // Restart DMA for UART3
                Restart_UART3_DMA();
                
                if (myFlash_SaveModbusUARTParams(&modbus_params) == HAL_OK) {
                    printf("✅ Modbus UART BAUD set to %lu and saved\r\n", (unsigned long)baud);
                } else {
                    printf("⚠️ Modbus UART BAUD set to %lu but save failed\r\n", (unsigned long)baud);
                }
            } else {
                printf("❌ Invalid baud rate. Range: 2400-921600\r\n");
            }
        } else if (strncmp(param_start, "parity ", 7) == 0) {
            uint32_t par = atoi(param_start + 7);
            if (par <= 2) {
                myModbusUARTParams modbus_params;
                myFlash_LoadModbusUARTParams(&modbus_params);
                modbus_params.parity = par;
                modbus_params.baudRate = handler->config.huart3->Init.BaudRate;
                modbus_params.stopBits = (handler->config.huart3->Init.StopBits == UART_STOPBITS_2) ? 2U : 1U;
                modbus_params.frameTimeoutMs = *handler->config.time;
                // Apply parity to UART3 only
                switch (par) {
                    case 0: // NONE
                        handler->config.huart3->Init.Parity = UART_PARITY_NONE;
                        handler->config.huart3->Init.WordLength = UART_WORDLENGTH_8B;
                        break;
                    case 1: // ODD
                        handler->config.huart3->Init.Parity = UART_PARITY_ODD;
                        handler->config.huart3->Init.WordLength = UART_WORDLENGTH_9B;
                        break;
                    case 2: // EVEN
                        handler->config.huart3->Init.Parity = UART_PARITY_EVEN;
                        handler->config.huart3->Init.WordLength = UART_WORDLENGTH_9B;
                        break;
                }
                HAL_UART_DeInit(handler->config.huart3);
                HAL_UART_Init(handler->config.huart3);
                Restart_UART3_DMA();
                
                if (myFlash_SaveModbusUARTParams(&modbus_params) == HAL_OK) {
                    printf("✅ Modbus UART PARITY set to %lu (0=None,1=Odd,2=Even) and saved\r\n", (unsigned long)par);
                } else {
                    printf("⚠️ Modbus UART PARITY set to %lu but save failed\r\n", (unsigned long)par);
                }
            } else {
                printf("❌ Invalid parity. Use: 0=None, 1=Odd, 2=Even\r\n");
            }
        } else if (strncmp(param_start, "stop ", 5) == 0) {
            uint32_t stop = atoi(param_start + 5);
            if (stop == 1 || stop == 2) {
                myModbusUARTParams modbus_params;
                myFlash_LoadModbusUARTParams(&modbus_params);
                modbus_params.stopBits = stop;
                modbus_params.baudRate = handler->config.huart3->Init.BaudRate;
                modbus_params.parity = *handler->config.parity;
                modbus_params.frameTimeoutMs = *handler->config.time;
                // Apply stop bits to UART3 only
                if (stop == 2) {
                    handler->config.huart3->Init.StopBits = UART_STOPBITS_2;
                } else {
                    handler->config.huart3->Init.StopBits = UART_STOPBITS_1;
                }
                HAL_UART_DeInit(handler->config.huart3);
                HAL_UART_Init(handler->config.huart3);
                Restart_UART3_DMA();
                
                if (myFlash_SaveModbusUARTParams(&modbus_params) == HAL_OK) {
                    printf("✅ Modbus UART STOP BITS set to %lu and saved\r\n", (unsigned long)stop);
                } else {
                    printf("⚠️ Modbus UART STOP BITS set to %lu but save failed\r\n", (unsigned long)stop);
                }
            } else {
                printf("❌ Invalid stop bits. Use: 1 or 2\r\n");
            }
        } else if (strncmp(param_start, "timeout ", 8) == 0) {
            uint32_t timeout = atoi(param_start + 8);
            if (timeout >= 10 && timeout <= 10000) {
                myModbusUARTParams modbus_params;
                myFlash_LoadModbusUARTParams(&modbus_params);
                modbus_params.frameTimeoutMs = timeout;
                modbus_params.baudRate = handler->config.huart3->Init.BaudRate;
                modbus_params.parity = *handler->config.parity;
                modbus_params.stopBits = (handler->config.huart3->Init.StopBits == UART_STOPBITS_2) ? 2U : 1U;
                // Apply timeout to global config
                if (myFlash_SaveModbusUARTParams(&modbus_params) == HAL_OK) {
                    printf("✅ Modbus FRAME TIMEOUT set to %lu ms and saved\r\n", (unsigned long)timeout);
                } else {
                    printf("⚠️ Modbus FRAME TIMEOUT set to %lu ms but save failed\r\n", (unsigned long)timeout);
                }
            } else {
                printf("❌ Invalid timeout. Range: 10-10000 ms\r\n");
            }
        } else {
            printf("❌ Unknown Modbus UART command. Available: baud, parity, stop, timeout\r\n");
        }
    }
}

static void Process_EncoderCommands(CommandHandler_t *handler, const char* cmd) {
    if (strncmp(cmd, "ppr ", 4) == 0) {
        uint32_t new_ppr = atoi(cmd + 4);
        if (new_ppr > 0 && new_ppr <= 10000) {
            *handler->config.ppr = new_ppr;
            
            // Update proximity counter configuration (same as Modbus handler)
            extern ProximityCounter_t proximity_counter;
            ProximityCounter_UpdateConfig(&proximity_counter, *handler->config.ppr, *handler->config.dia);
            
            if (handler->config.encoder_init) {
                handler->config.encoder_init(handler->config.encoder, handler->config.htim, 
                                           *handler->config.ppr, *handler->config.dia, *handler->config.timeout, *handler->config.time);
            }

			myEncoderParams enc_params = {
				.diameter = (uint32_t)(*handler->config.dia * 1000),
				.pulsesPerRev = *handler->config.ppr,
                .timeout = *handler->config.timeout,
				.sampleTimeMs = *handler->config.time,
			};
			if (handler->config.save_encoder_params &&
				handler->config.save_encoder_params(&enc_params) == HAL_OK) {
				printf("✅ PPR set to %lu and saved\r\n", (unsigned long)*handler->config.ppr);
			} else {
				printf("⚠️ PPR set to %lu but save failed\r\n", (unsigned long)*handler->config.ppr);
			}
        } else {
            printf("❌ Invalid PPR (1-10000)\r\n");
        }
    } else if (strncmp(cmd, "dia ", 4) == 0) {
        float new_dia = atof(cmd + 4);
        if (new_dia > 0.001 && new_dia < 10.0) {
            *handler->config.dia = new_dia;
            
            // Update proximity counter configuration (same as Modbus handler)
            extern ProximityCounter_t proximity_counter;
            ProximityCounter_UpdateConfig(&proximity_counter, *handler->config.ppr, *handler->config.dia);
            
            if (handler->config.encoder_init) {
                handler->config.encoder_init(handler->config.encoder, handler->config.htim, 
                                           *handler->config.ppr, *handler->config.dia, *handler->config.timeout, *handler->config.time);
            }

			myEncoderParams enc_params = {
				.diameter = (uint32_t)(*handler->config.dia * 1000),
				.pulsesPerRev = *handler->config.ppr,
                .timeout = *handler->config.timeout,
				.sampleTimeMs = *handler->config.time,
			};
			if (handler->config.save_encoder_params &&
				handler->config.save_encoder_params(&enc_params) == HAL_OK) {
				printf("✅ DIA set to %.3f and saved\r\n", (double)*handler->config.dia);
			} else {
				printf("⚠️ DIA set to %.3f but save failed\r\n", (double)*handler->config.dia);
			}
        } else {
            printf("❌ Invalid diameter (0.001-10.0 meters)\r\n");
        }
    } else if (strncmp(cmd, "sampletime ", 11) == 0){
        uint32_t new_time = atoi(cmd + 11);
        if (new_time >= 10 && new_time <= 10000) {
            *handler->config.time = new_time;
           
            if (handler->config.encoder_init) {
                handler->config.encoder_init(handler->config.encoder, handler->config.htim, 
                                           *handler->config.ppr, *handler->config.dia,*handler->config.timeout, *handler->config.time);
            }

            myEncoderParams enc_params = {
                .diameter = (uint32_t)(*handler->config.dia * 1000),
                .pulsesPerRev = *handler->config.ppr,
                .timeout = *handler->config.timeout,
                .sampleTimeMs = *handler->config.time,
            };
            if (handler->config.save_encoder_params &&
                handler->config.save_encoder_params(&enc_params) == HAL_OK) {
                printf("✅ SAMPLE TIME set to %lums and saved\r\n", (unsigned long)*handler->config.time);
            } else {
                printf("⚠️ SAMPLE TIME set to %lums but save failed\r\n", (unsigned long)*handler->config.time);
            }
        } else {
            printf("❌ Invalid time (10-10000ms)\r\n");
        }
    }else if (strncmp(cmd, "timeout ", 8) == 0) {
        uint32_t new_timeout = atoi(cmd + 8);
        if (new_timeout >= 10 && new_timeout <= 10000) {
            *handler->config.timeout = new_timeout;
            
            // Update proximity counter timeout
            extern ProximityCounter_t proximity_counter;
            ProximityCounter_SetTimeout(&proximity_counter, new_timeout * 10);
            
            if (handler->config.encoder_init) {
                handler->config.encoder_init(handler->config.encoder, handler->config.htim, 
                                           *handler->config.ppr, *handler->config.dia,*handler->config.timeout, *handler->config.time);
            }

            myEncoderParams enc_params = {
                .diameter = (uint32_t)(*handler->config.dia * 1000),
                .pulsesPerRev = *handler->config.ppr,
                .timeout = *handler->config.timeout,
                .sampleTimeMs = *handler->config.time,
            };
            if (handler->config.save_encoder_params &&
                handler->config.save_encoder_params(&enc_params) == HAL_OK) {
                printf("✅ TIMEOUT set to %lums and saved\r\n", (unsigned long)*handler->config.timeout);
            } else {
                printf("⚠️ TIMEOUT set to %lums but save failed\r\n", (unsigned long)*handler->config.timeout);
            }
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
        float new_len_m = (float)atof(cmd + 8);
        if (new_len_m < 0.0f || new_len_m > 10000.0f) {
            printf("❌ Invalid length (0-10000 meters)\r\n");
            return;
        }

        uint32_t length_mm = (uint32_t)(new_len_m * 1000.0f);
        Encoder_t* enc = (Encoder_t*)handler->config.encoder;
        if (enc) {
            enc->current_length = new_len_m;
        }

        if (handler->config.save_length && handler->config.save_length(length_mm) == HAL_OK) {
            printf("✅ Length set to %.3fm and saved\r\n", (double)new_len_m);
        } else {
            printf("⚠️ Length set to %.3fm but save failed\r\n", (double)new_len_m);
        }
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
                                       *handler->config.ppr, *handler->config.dia, *handler->config.timeout, *handler->config.time);
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
                                       *handler->config.ppr, *handler->config.dia, *handler->config.timeout, *handler->config.time);
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
        
        // Display UART3 (Modbus port) configuration details
        UART_HandleTypeDef* modbus_uart = handler->config.huart3;
        if (modbus_uart) {
            printf("=== MODBUS UART3 CONFIG ===\r\n");
            printf("🔌 BAUD RATE: %lu bps\r\n", (unsigned long)modbus_uart->Init.BaudRate);
            
            // Parity information
            const char* parity_str;
            switch (modbus_uart->Init.Parity) {
                case UART_PARITY_NONE: parity_str = "NONE"; break;
                case UART_PARITY_ODD:  parity_str = "ODD";  break;
                case UART_PARITY_EVEN: parity_str = "EVEN"; break;
                default: parity_str = "UNKNOWN"; break;
            }
            printf("🔧 PARITY: %s\r\n", parity_str);
            
            // Stop bits information
            const char* stopbits_str;
            uint8_t stopbits_num;
            if (modbus_uart->Init.StopBits == UART_STOPBITS_1) {
                stopbits_str = "1 bit";
                stopbits_num = 1;
            } else if (modbus_uart->Init.StopBits == UART_STOPBITS_2) {
                stopbits_str = "2 bits";
                stopbits_num = 2;
            } else {
                stopbits_str = "UNKNOWN";
                stopbits_num = 0;
            }
            printf("🛑 STOP BITS: %s (%d)\r\n", stopbits_str, stopbits_num);
            
            // Word length information
            const char* wordlen_str;
            uint8_t wordlen_num;
            if (modbus_uart->Init.WordLength == UART_WORDLENGTH_8B) {
                wordlen_str = "8 bits";
                wordlen_num = 8;
            } else if (modbus_uart->Init.WordLength == UART_WORDLENGTH_9B) {
                wordlen_str = "9 bits";
                wordlen_num = 9;
            } else {
                wordlen_str = "UNKNOWN";
                wordlen_num = 0;
            }
            printf("📏 WORD LENGTH: %s (%d)\r\n", wordlen_str, wordlen_num);
            
            // Hardware flow control
            const char* flow_str;
            switch (modbus_uart->Init.HwFlowCtl) {
                case UART_HWCONTROL_NONE:    flow_str = "NONE"; break;
                case UART_HWCONTROL_RTS:     flow_str = "RTS"; break;
                case UART_HWCONTROL_CTS:     flow_str = "CTS"; break;
                case UART_HWCONTROL_RTS_CTS: flow_str = "RTS_CTS"; break;
                default: flow_str = "UNKNOWN"; break;
            }
            printf("🌊 FLOW CONTROL: %s\r\n", flow_str);
            
            // Mode (TX/RX)
            const char* mode_str;
            switch (modbus_uart->Init.Mode) {
                case UART_MODE_RX:    mode_str = "RX only"; break;
                case UART_MODE_TX:    mode_str = "TX only"; break;
                case UART_MODE_TX_RX: mode_str = "TX_RX"; break;
                default: mode_str = "UNKNOWN"; break;
            }
            printf("↔️  MODE: %s\r\n", mode_str);
            
            printf("💡 Standard Modbus RTU: 8N1 or 8E1 or 8O1\r\n");
        }
        
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
        
        // Save to Flash
        mySpeedUnitConfig config = {
            .speed_unit = 0,  // 0 = RPM
            .reserved = {0, 0, 0}
        };
        if (myFlash_SaveSpeedUnitConfig(&config) == HAL_OK) {
            printf("✅ Speed display unit set to RPM and saved\r\n");
        } else {
            printf("✅ Speed display unit set to RPM (save failed)\r\n");
        }
        printf("🔄 Speed will be displayed as rotational speed\r\n");
    } else if (strncmp(cmd, "speed_unit m/min", 16) == 0) {
        current_speed_unit = SPEED_UNIT_M_MIN;
        SetProximitySpeedUnit(1); // 1 = m/min
        
        // Save to Flash
        mySpeedUnitConfig config = {
            .speed_unit = 1,  // 1 = m/min
            .reserved = {0, 0, 0}
        };
        if (myFlash_SaveSpeedUnitConfig(&config) == HAL_OK) {
            printf("✅ Speed display unit set to m/min and saved\r\n");
        } else {
            printf("✅ Speed display unit set to m/min (save failed)\r\n");
        }
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
    printf("DEBUG CONFIG:\r\n");
    printf("  debug            - Show debug settings\r\n");
    printf("  debug on         - Enable debug messages\r\n");
    printf("  debug off        - Disable debug messages\r\n");
    printf("  debug interval <ms> - Set debug message interval (100-10000ms)\r\n");
    printf("UART1 CONFIG (Command Port):\r\n");
    printf("  uart1 baud <rate>    - Set UART1 baud rate (2400-921600)\r\n");
    printf("  uart1 parity <n>     - Set UART1 parity (0=None,1=Odd,2=Even)\r\n");
    printf("  uart1 stop <n>       - Set UART1 stop bits (1 or 2)\r\n");
    printf("MODBUS UART CONFIG (UART3):\r\n");
    printf("  modbus uart baud <rate>    - Set Modbus UART baud rate (2400-921600)\r\n");
    printf("  modbus uart parity <n>     - Set Modbus UART parity (0=None,1=Odd,2=Even)\r\n");
    printf("  modbus uart stop <n>       - Set Modbus UART stop bits (1 or 2)\r\n");
    printf("  modbus uart timeout <ms>   - Set Modbus frame timeout (10-10000ms)\r\n");
    printf("ENCODER CONFIG:\r\n");
    printf("  ppr <n>      - Set pulses per revolution (1-10000)\r\n");
    printf("  dia <f>      - Set diameter in meters (0.001-10.0)\r\n");
    printf("  sampletime <ms>    - Set sample time (10-10000ms)\r\n");
    printf("  timeout <ms> - Set encoder timeout (10-10000ms)\r\n");
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
    printf("  hyst set <i> <rpm> <h> - Set/modify hysteresis entry\r\n");
    printf("  hyst clear       - Clear all entries\r\n");
    printf("  hyst default     - Restore default table\r\n");
    printf("  hyst save/load   - Save/Load to Flash\r\n");
    printf("PROXIMITY STATUS:\r\n");
    printf("  proximity_setting - Show proximity counter configuration\r\n");
}

/**
 * @brief Process proximity counter/hysteresis commands
 */
static void Process_ProximityCommands(CommandHandler_t *handler, const char* cmd) {
    if (strcmp(cmd, "hyst") == 0 || strcmp(cmd, "hyst show") == 0) {
        ShowProximityHysteresis();
        
    } else if (strcmp(cmd, "proximity_setting") == 0) {
        printf("=== PROXIMITY COUNTER SETTING ===\r\n");
        printf("🔄 PPR=%lu DIA=%.3f TIME=%lums\r\n", 
               (unsigned long)*handler->config.ppr, 
               (double)*handler->config.dia, 
               (unsigned long)*handler->config.time);
      
        
    } else if (strncmp(cmd, "hyst set ", 9) == 0) {
        // Parse: hyst set <index> <rpm_threshold> <hysteresis>
        const char* params = cmd + 9;
        int index, rpm_threshold, hysteresis;
        
        if (sscanf(params, "%d %d %d", &index, &rpm_threshold, &hysteresis) == 3) {
            if (index >= 0 && index < 10 && rpm_threshold >= 0 && hysteresis > 0 && hysteresis <= 1000) {
                SetProximityHysteresis(index, rpm_threshold, hysteresis);
                printf("✅ Hysteresis entry at index %d set to: RPM=%d, Hysteresis=%d\r\n", 
                       index, rpm_threshold, hysteresis);
                printf("💡 Use 'hyst show' to view updated table\r\n");
                printf("💾 Remember to use 'hyst save' to persist changes to flash\r\n");
            } else {
                printf("❌ Invalid parameters:\r\n");
                printf("   Index: 0-9 (overwrites existing or adds new)\r\n");
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
        printf("💾 Saving hysteresis table...\r\n");
        SaveProximityHysteresis();
        printf("✅ Save completed. Use 'hyst load' to test or reset to verify persistence.\r\n");
        
    } else if (strcmp(cmd, "hyst load") == 0) {
        printf("📖 Loading hysteresis table from Flash...\r\n");
        LoadProximityHysteresis();
        printf("✅ Load completed. Use 'hyst show' to view loaded table.\r\n");
        
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
        printf("💾 Remember to use 'hyst save' to save changes to flash\r\n");
        
    } else if (strncmp(cmd, "hyst add ", 9) == 0) {
        // Parse: hyst add <rpm_threshold> <hysteresis> - automatically find next free index
        const char* params = cmd + 9;
        int rpm_threshold, hysteresis;
        
        if (sscanf(params, "%d %d", &rpm_threshold, &hysteresis) == 2) {
            if (rpm_threshold >= 0 && hysteresis > 0 && hysteresis <= 1000) {
                printf("⚠️  Note: 'hyst add' command is deprecated to avoid overwriting existing entries\r\n");
                printf("💡 Please use 'hyst set <index> %d %d' to specify the exact index\r\n", rpm_threshold, hysteresis);
                printf("📊 Use 'hyst show' to see current table and available indices\r\n");
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
        printf("  hyst set <i> <rpm> <h> - Set/modify entry at index i (0-9)\r\n");
        printf("  hyst clear        - Clear all entries\r\n");
        printf("  hyst default      - Restore default table\r\n");
        printf("  hyst save         - Save table to Flash\r\n");
        printf("  hyst load         - Load table from Flash\r\n");
        printf("  hyst debug        - Show debug info (internal state)\r\n");
        printf("💡 Parameters: i=index(0-9), rpm=threshold(>=0), h=hysteresis(1-1000)\r\n");
        printf("💾 Note: 'hyst set' can overwrite existing entries or add new ones\r\n");
        printf("💾 Always use 'hyst save' after making changes to persist them\r\n");
    }
}
