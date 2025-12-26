/*
 * modbus_master.c
 *
 *  Created on: Jul 16, 2025
 *      Author: victus16
 */
#include "modbus_master.h"
#include "../modbus.h"
#include "queue/queue.h"
#include "modbus/crc16/crc16.h"
#include <string.h>

static UART_HandleTypeDef *modbus_uart = NULL;
static ModbusMasterMode modbus_mode = MODBUS_MASTER_MODE_RTU;
static uint16_t tcp_tid_counter = 1; // auto increment if transaction_id == 0
ModbusResponseCallback modbus_user_on_response = NULL;

void modbus_master_init(UART_HandleTypeDef *huart) {
    modbus_uart = huart;
    modbus_mode = MODBUS_MASTER_MODE_RTU;
}

void modbus_master_init_ex(UART_HandleTypeDef *huart, ModbusMasterMode mode) {
    modbus_uart = huart;
    modbus_mode = mode;
}

ModbusMasterMode modbus_master_get_mode(void) {
    return modbus_mode;
}

static uint16_t build_pdu(uint8_t *pdu, const ModbusRequest_t *req) {
    uint16_t len = 0;
    pdu[len++] = req->func_code;
    pdu[len++] = req->addr >> 8;
    pdu[len++] = req->addr & 0xFF;

    switch (req->func_code) {
        case MODBUS_FC_READ_COILS:
        case MODBUS_FC_READ_DISCRETE_INPUTS:
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_READ_INPUT_REGISTERS:
            pdu[len++] = req->quantity >> 8;
            pdu[len++] = req->quantity & 0xFF;
            break;
        case MODBUS_FC_WRITE_SINGLE_COIL:
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
            pdu[len++] = req->write_data[0] >> 8;
            pdu[len++] = req->write_data[0] & 0xFF;
            break;
        case MODBUS_FC_WRITE_MULTIPLE_COILS: {
            uint16_t byte_count = (req->quantity + 7) / 8;
            pdu[len++] = req->quantity >> 8;
            pdu[len++] = req->quantity & 0xFF;
            pdu[len++] = byte_count;
            for (uint16_t i = 0; i < byte_count; i++) {
                uint8_t byte = 0;
                for (uint8_t b = 0; b < 8; b++) {
                    uint16_t bit_index = i * 8 + b;
                    if (bit_index < req->quantity && req->write_data[bit_index])
                        byte |= (1 << b);
                }
                pdu[len++] = byte;
            }
            break;
        }
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            pdu[len++] = req->quantity >> 8;
            pdu[len++] = req->quantity & 0xFF;
            pdu[len++] = req->quantity * 2;
            for (uint16_t i = 0; i < req->quantity; i++) {
                pdu[len++] = req->write_data[i] >> 8;
                pdu[len++] = req->write_data[i] & 0xFF;
            }
            break;
        default:
            return 0;
    }
    return len;
}

bool modbus_master_send_request(ModbusRequest_t *req) {
    if (!modbus_uart || !req) return false;

    uint8_t frame[MODBUS_MAX_FRAME_SIZE];
    uint16_t len = 0;

    if (modbus_mode == MODBUS_MASTER_MODE_RTU) {
        frame[len++] = req->slave_id; // address
        uint16_t pdu_len = build_pdu(&frame[len], req);
        if (pdu_len == 0) return false;
        len += pdu_len;
        uint16_t crc = modbus_crc16(frame, len);
        frame[len++] = crc & 0xFF;
        frame[len++] = crc >> 8;
    } else { // TCP
        uint16_t tid = req->transaction_id;
        if (tid == 0) {
            tid = tcp_tid_counter++;
        }
        uint8_t pdu[MODBUS_MAX_FRAME_SIZE];
        uint16_t pdu_len = build_pdu(pdu, req);
        if (pdu_len == 0) return false;
        // MBAP header
        frame[len++] = (uint8_t)(tid >> 8);
        frame[len++] = (uint8_t)(tid & 0xFF);
        frame[len++] = 0x00; // Protocol ID hi
        frame[len++] = 0x00; // Protocol ID lo
        uint16_t length_field = (uint16_t)(1 + pdu_len); // UnitID + PDU
        frame[len++] = (uint8_t)(length_field >> 8);
        frame[len++] = (uint8_t)(length_field & 0xFF);
        frame[len++] = req->slave_id; // Unit ID
        // Copy PDU
        memcpy(&frame[len], pdu, pdu_len);
        len += pdu_len;
    }

    // Set DE=HIGH trước khi gửi (chế độ transmit RS485)
    MODBUS_SET_DE_TX();
    HAL_Delay(10); // cho tín hiệu ổn định
    // Debug: print transmission frame
    // printf("📤 TX frame (%d bytes): ", len);
    // for(uint16_t i = 0; i < len; i++) {
    //     printf("0x%02X ", frame[i]);
    // }
    // printf("\r\n");
    HAL_StatusTypeDef ret = HAL_UART_Transmit(modbus_uart, frame, len, HAL_MAX_DELAY);
  HAL_Delay(10); // cho tín hiệu ổn định
    MODBUS_SET_DE_RX();
    return ret == HAL_OK;
}

void modbus_master_handle_response(uint8_t *data, uint16_t len) {
    printf("🔍 Handle response: len=%d\r\n", len);
    if (!data || len == 0) {
        // printf("❌ Invalid data or length\r\n");
        return;
    }
    if (modbus_mode == MODBUS_MASTER_MODE_RTU) {
        printf("🔧 RTU mode validation\r\n");
        if (len < 5) {
            // printf("❌ Length too short: %d < 5\r\n", len);
            return;
        }
        uint16_t crc_calc = modbus_crc16(data, len - 2);
        uint16_t crc_recv = data[len - 2] | (data[len - 1] << 8);
        printf("🔒 CRC check: calc=0x%04X, recv=0x%04X\r\n", crc_calc, crc_recv);
        if (crc_calc != crc_recv) {
            // printf("❌ CRC mismatch!\r\n");
            return;
        }
        // printf("✅ CRC OK, calling callback\r\n");
        if (modbus_user_on_response) modbus_user_on_response(data, len);
    } else { // TCP
        printf("🔧 TCP mode validation\r\n");
        if (len < 8) return; // MBAP (7) + at least FC(1)
        uint16_t pid = (uint16_t)(data[2] << 8 | data[3]);
        if (pid != 0x0000) return; // only standard Modbus
        uint16_t length_field = (uint16_t)(data[4] << 8 | data[5]);
        if (len < (uint16_t)(6 + length_field)) return; // incomplete frame
        if (modbus_user_on_response) modbus_user_on_response(data, len);
    }
}
bool modbus_parse_response(uint8_t *data, uint16_t len, ModbusParsedResponse_t *out) {
    if (!data || !out) return false;
    memset(out, 0, sizeof(ModbusParsedResponse_t));

    uint8_t *p = data;
    uint16_t available = len;
    if (modbus_mode == MODBUS_MASTER_MODE_TCP) {
        if (len < 8) return false;
        out->transaction_id = (uint16_t)(data[0] << 8 | data[1]);
        uint16_t pid = (uint16_t)(data[2] << 8 | data[3]);
        if (pid != 0x0000) return false;
        uint16_t length_field = (uint16_t)(data[4] << 8 | data[5]);
        if (len < (uint16_t)(6 + length_field)) return false;
        out->slave_id = data[6];
        p = &data[7];
        available = (uint16_t)(length_field - 1); // exclude unit id
    } else { // RTU
        if (len < 5) return false;
        out->slave_id = data[0];
        // optional CRC already checked by handler; adjust available excluding CRC
        if (available >= 2) available = (uint16_t)(available - 2);
        p = &data[1];
    }

    if (available == 0) return false;
    uint8_t func = p[0];

    if (func & 0x80) {
        out->is_exception = true;
        out->func_code = (ModbusFunctionCode)(func & 0x7F);
        out->exception_code = p[1];
        return true;
    }
    out->func_code = (ModbusFunctionCode)func;

    switch (func) {
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_READ_INPUT_REGISTERS: {
            if (available < 2) return false;
            uint8_t byte_count = p[1];
            if (byte_count > sizeof(out->read.registers) * 2) return false;
            if (available < (uint16_t)(2 + byte_count)) return false;
            out->read.byte_count = byte_count;
            out->read.quantity = (uint16_t)(byte_count / 2);
            for (uint16_t i = 0; i < out->read.quantity; i++) {
                out->read.registers[i] = (p[2 + i * 2] << 8) | p[3 + i * 2];
            }
            return true;
        }
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
        case MODBUS_FC_WRITE_SINGLE_COIL:
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
        case MODBUS_FC_WRITE_MULTIPLE_COILS: {
            if (available < 5) return false;
            out->write_ack.addr = (p[1] << 8) | p[2];
            out->write_ack.value = (p[3] << 8) | p[4];
            return true;
        }
        default:
            return false;
    }
}
