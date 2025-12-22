#include "modbus.h"
#include "modbus/modbus_master/modbus_master.h"
#include "modbus/modbus_slave/modbus_slave.h"

// Modbus DE pin control functions
void modbus_set_de_transmit(void) {
    MODBUS_SET_DE_TX();
}

void modbus_set_de_receive(void) {
    MODBUS_SET_DE_RX();
}

static ModbusRole s_role = MODBUS_ROLE_MASTER;
static ModbusMode s_mode = MODBUS_MODE_RTU;
static UART_HandleTypeDef *s_huart = NULL;

void modbus_init_master(UART_HandleTypeDef *huart, ModbusMode mode)
{
    s_role = MODBUS_ROLE_MASTER;
    s_mode = mode;
    s_huart = huart;
    modbus_master_init_ex(huart, (mode == MODBUS_MODE_RTU) ? MODBUS_MASTER_MODE_RTU : MODBUS_MASTER_MODE_TCP);
}

void modbus_init_slave(UART_HandleTypeDef *huart, ModbusSlaveConfig *cfg, ModbusMode mode)
{
    s_role = MODBUS_ROLE_SLAVE;
    s_mode = mode;
    s_huart = huart;

    // Adapt unified config to slave-specific type
    modbus_slave_config_t slave_cfg = {
        .id = cfg->id,
        .coils = cfg->coils,
        .coil_count = cfg->coil_count,
        .discrete_inputs = cfg->discrete_inputs,
        .discrete_input_count = cfg->discrete_input_count,
        .holding_registers = cfg->holding_registers,
        .holding_register_count = cfg->holding_register_count,
        .input_registers = cfg->input_registers,
        .input_register_count = cfg->input_register_count,
        .on_read_coils = cfg->on_read_coils,
        .on_read_discrete_inputs = cfg->on_read_discrete_inputs,
        .on_read_holding_registers = cfg->on_read_holding_registers,
        .on_read_input_registers = cfg->on_read_input_registers,
        .on_write_single_coil = cfg->on_write_single_coil,
        .on_write_single_register = cfg->on_write_single_register,
        .on_write_multiple_coils = cfg->on_write_multiple_coils,
        .on_write_multiple_registers = cfg->on_write_multiple_registers,
    };

    modbus_slave_init_ex(huart, &slave_cfg, (mode == MODBUS_MODE_RTU) ? MODBUS_SLAVE_MODE_RTU : MODBUS_SLAVE_MODE_TCP);
}

// Master operations
bool modbus_send_request(ModbusRequest_t *req)
{
    if (s_role != MODBUS_ROLE_MASTER) {
        return false;
    }
    return modbus_master_send_request(req);
}

void modbus_handle_response(uint8_t *data, uint16_t len)
{
    if (s_role != MODBUS_ROLE_MASTER) {
        return;
    }
    modbus_master_handle_response(data, len);
}

// Parsing is provided by the master module; no wrapper to avoid name clash.

// Slave operations
void modbus_handle_frame(const uint8_t *frame, uint16_t len)
{
    if (s_role != MODBUS_ROLE_SLAVE) {
        return;
    }
    modbus_slave_handle_frame(frame, len);
}

ModbusRole modbus_get_role(void)
{
    return s_role;
}

ModbusMode modbus_get_mode(void)
{
    return s_mode;
}
