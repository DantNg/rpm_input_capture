/*
 * modbus_master.h
 *
 *  Created on: Jul 16, 2025
 *      Author: victus16
 */

#ifndef MODBUS_MODBUS_MASTER_MODBUS_MASTER_H_
#define MODBUS_MODBUS_MASTER_MODBUS_MASTER_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
// Use unified Modbus types
#include "../modbus.h"

#define MODBUS_MAX_FRAME_SIZE 256

typedef enum {
    MODBUS_MASTER_MODE_RTU = 0,
    MODBUS_MASTER_MODE_TCP = 1
} ModbusMasterMode;

// Function codes and shared request/response structures are defined in modbus.h

// Use ModbusRequest_t from modbus.h

// Use ModbusParsedResponse_t from modbus.h

// ModbusResponseCallback is defined in modbus.h

void modbus_master_init(UART_HandleTypeDef *huart); // default RTU
void modbus_master_init_ex(UART_HandleTypeDef *huart, ModbusMasterMode mode);
bool modbus_master_send_request(ModbusRequest_t *req);
void modbus_master_handle_response(uint8_t *data, uint16_t len);

// Parsing API available via modbus.h; declare here for convenience
bool modbus_parse_response(uint8_t *data, uint16_t len, ModbusParsedResponse_t *out);
ModbusMasterMode modbus_master_get_mode(void);
extern ModbusResponseCallback modbus_user_on_response;
#endif /* MODBUS_MODBUS_MASTER_MODBUS_MASTER_H_ */
