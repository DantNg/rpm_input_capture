#ifndef MODBUS_UNIFIED_MODBUS_H_
#define MODBUS_UNIFIED_MODBUS_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

// Modbus RS485 DE (Driver Enable) Pin Configuration
#define MODBUS_DE_GPIO_PORT    GPIOB
#define MODBUS_DE_GPIO_PIN     GPIO_PIN_12
#define MODBUS_DE_TX_STATE     GPIO_PIN_SET      // DE=HIGH for transmit
#define MODBUS_DE_RX_STATE     GPIO_PIN_RESET    // DE=LOW for receive

// Macro for DE pin control
#define MODBUS_SET_DE_TX()     HAL_GPIO_WritePin(MODBUS_DE_GPIO_PORT, MODBUS_DE_GPIO_PIN, MODBUS_DE_TX_STATE)
#define MODBUS_SET_DE_RX()     HAL_GPIO_WritePin(MODBUS_DE_GPIO_PORT, MODBUS_DE_GPIO_PIN, MODBUS_DE_RX_STATE)

// Function prototypes for DE control
void modbus_set_de_transmit(void);
void modbus_set_de_receive(void);

typedef enum {
    MODBUS_MODE_RTU = 0,
    MODBUS_MODE_TCP = 1
} ModbusMode;

typedef enum {
    MODBUS_ROLE_MASTER = 0,
    MODBUS_ROLE_SLAVE = 1
} ModbusRole;

// Function codes (master-side definitions)
typedef enum {
    MODBUS_FC_READ_COILS = 0x01,
    MODBUS_FC_READ_DISCRETE_INPUTS = 0x02,
    MODBUS_FC_READ_HOLDING_REGISTERS = 0x03,
    MODBUS_FC_READ_INPUT_REGISTERS = 0x04,
    MODBUS_FC_WRITE_SINGLE_COIL = 0x05,
    MODBUS_FC_WRITE_SINGLE_REGISTER = 0x06,
    MODBUS_FC_WRITE_MULTIPLE_COILS = 0x0F,
    MODBUS_FC_WRITE_MULTIPLE_REGISTERS = 0x10,
} ModbusFunctionCode;

typedef struct {
    uint8_t slave_id;                // RTU: address, TCP: unit id
    uint16_t transaction_id;         // TCP only (ignored if 0 in RTU); auto-increment if 0
    ModbusFunctionCode func_code;
    uint16_t addr;
    uint16_t quantity;               // number of registers/coils to read/write
    const uint16_t *write_data;      // pointer to data for write functions
} ModbusRequest_t;

typedef struct {
    uint16_t transaction_id;         // only meaningful in TCP mode
    uint8_t slave_id;                // RTU: address, TCP: unit id
    ModbusFunctionCode func_code;
    bool is_exception;
    uint8_t exception_code;

    union {
        struct {
            uint8_t byte_count;
            uint16_t registers[64]; // max 64 words
            uint16_t quantity;
        } read;

        struct {
            uint16_t addr;
            uint16_t value;
        } write_ack;
    };
} ModbusParsedResponse_t;

typedef void (*ModbusResponseCallback)(uint8_t *data, uint16_t len);

// Slave configuration (unified)
typedef struct {
    uint8_t id;
    uint8_t  *coils;              // 0x01, 0x05, 0x0F
    uint16_t coil_count;

    uint8_t  *discrete_inputs;    // 0x02
    uint16_t discrete_input_count;

    uint16_t *holding_registers;  // 0x03, 0x06, 0x10
    uint16_t holding_register_count;

    uint16_t *input_registers;    // 0x04
    uint16_t input_register_count;

    // Callbacks for function codes
    void (*on_read_coils)(uint16_t addr, uint16_t quantity);
    void (*on_read_discrete_inputs)(uint16_t addr, uint16_t quantity);
    void (*on_read_holding_registers)(uint16_t addr, uint16_t quantity);
    void (*on_read_input_registers)(uint16_t addr, uint16_t quantity);
    void (*on_write_single_coil)(uint16_t addr, bool value);
    void (*on_write_single_register)(uint16_t addr, uint16_t value);
    void (*on_write_multiple_coils)(uint16_t addr, const uint8_t *values, uint16_t quantity);
    void (*on_write_multiple_registers)(uint16_t addr, const uint16_t *values, uint16_t quantity);
} ModbusSlaveConfig;

// Unified initialization and routing API
void modbus_init_master(UART_HandleTypeDef *huart, ModbusMode mode);
void modbus_init_slave(UART_HandleTypeDef *huart, ModbusSlaveConfig *cfg, ModbusMode mode);

// Master operations
bool modbus_send_request(ModbusRequest_t *req);
void modbus_handle_response(uint8_t *data, uint16_t len);
bool modbus_parse_response(uint8_t *data, uint16_t len, ModbusParsedResponse_t *out);
extern ModbusResponseCallback modbus_user_on_response;

// Slave operations
void modbus_handle_frame(const uint8_t *frame, uint16_t len);

// Current role/mode getters
ModbusRole modbus_get_role(void);
ModbusMode modbus_get_mode(void);

#endif // MODBUS_UNIFIED_MODBUS_H_