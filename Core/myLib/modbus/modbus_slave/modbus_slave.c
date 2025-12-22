// modbus_slave.c
#include "modbus_slave.h"
#include "../modbus.h"
#include "modbus/crc16/crc16.h"
#include <string.h>
#include <stdio.h>
static modbus_slave_config_t slave_cfg;
static UART_HandleTypeDef *modbus_uart;
static ModbusSlaveMode slave_mode = MODBUS_SLAVE_MODE_RTU;

void modbus_slave_init(UART_HandleTypeDef *huart, modbus_slave_config_t *cfg) {
	modbus_uart = huart;
	slave_cfg = *cfg;
}

void modbus_slave_init_ex(UART_HandleTypeDef *huart, modbus_slave_config_t *cfg, ModbusSlaveMode mode) {
	modbus_uart = huart;
	slave_cfg = *cfg;
	slave_mode = mode;
}

ModbusSlaveMode modbus_slave_get_mode(void) {
	return slave_mode;
}

void send_response(uint8_t *data, uint16_t len) {
	// Set DE=HIGH trước khi gửi response
	MODBUS_SET_DE_TX();
	HAL_UART_Transmit_DMA(modbus_uart, data, len);
	// TxCpltCallback sẽ tự động set DE=LOW khi gửi xong
}

static void send_tcp_response(uint16_t tid, uint8_t uid, const uint8_t *pdu, uint16_t pdu_len) {
	uint8_t buf[256];
	uint16_t idx = 0;
	// MBAP
	buf[idx++] = (uint8_t)(tid >> 8);
	buf[idx++] = (uint8_t)(tid & 0xFF);
	buf[idx++] = 0x00; // PID
	buf[idx++] = 0x00;
	uint16_t length_field = (uint16_t)(1 + pdu_len);
	buf[idx++] = (uint8_t)(length_field >> 8);
	buf[idx++] = (uint8_t)(length_field & 0xFF);
	buf[idx++] = uid;
	// PDU
	memcpy(&buf[idx], pdu, pdu_len);
	idx = (uint16_t)(idx + pdu_len);
	MODBUS_SET_DE_TX();
	HAL_UART_Transmit_DMA(modbus_uart, buf, idx);
}

static uint16_t build_exception_pdu(uint8_t *out, uint8_t fn, uint8_t ex) {
	out[0] = (uint8_t)(fn | 0x80);
	out[1] = ex;
	return 2;
}

void modbus_slave_handle_frame(const uint8_t *frame, uint16_t len) {
	if (!frame || len == 0) return;
	if (slave_mode == MODBUS_SLAVE_MODE_TCP) {
		if (len < 8) return; // MBAP + FC
		uint16_t tid = (uint16_t)(frame[0] << 8 | frame[1]);
		uint16_t pid = (uint16_t)(frame[2] << 8 | frame[3]);
		if (pid != 0x0000) return;
		uint16_t l = (uint16_t)(frame[4] << 8 | frame[5]);
		if (len < (uint16_t)(6 + l)) return;
		uint8_t uid = frame[6];
		if (uid != slave_cfg.id) return;
		const uint8_t *pdu = &frame[7];
		uint16_t pdu_len = (uint16_t)(l - 1);
		if (pdu_len < 1) return;

		uint8_t fn = pdu[0];
		uint8_t resp_pdu[256];
		uint16_t resp_pdu_len = 0;

		switch (fn) {
			case MODBUS_FUNC_READ_HOLDING_REGISTERS: {
				if (pdu_len < 5) { // fn + addr(2) + qty(2)
					resp_pdu_len = build_exception_pdu(resp_pdu, fn, 0x03);
					break;
				}
				uint16_t addr = (uint16_t)(pdu[1] << 8 | pdu[2]);
				uint16_t count = (uint16_t)(pdu[3] << 8 | pdu[4]);
				if (count == 0 || (uint32_t)addr + count > slave_cfg.holding_register_count) {
					resp_pdu_len = build_exception_pdu(resp_pdu, fn, 0x02);
					break;
				}
				if (slave_cfg.on_read_holding_registers) slave_cfg.on_read_holding_registers(addr, count);
				resp_pdu[0] = fn;
				resp_pdu[1] = (uint8_t)(count * 2);
				for (uint16_t i = 0; i < count; i++) {
					uint16_t v = slave_cfg.holding_registers[addr + i];
					resp_pdu[2 + i * 2] = (uint8_t)(v >> 8);
					resp_pdu[3 + i * 2] = (uint8_t)(v & 0xFF);
				}
				resp_pdu_len = (uint16_t)(2 + count * 2);
				break;
			}
			case MODBUS_FUNC_WRITE_SINGLE_REGISTER: {
				if (pdu_len < 5) { resp_pdu_len = build_exception_pdu(resp_pdu, fn, 0x03); break; }
				uint16_t addr = (uint16_t)(pdu[1] << 8 | pdu[2]);
				uint16_t val  = (uint16_t)(pdu[3] << 8 | pdu[4]);
				if (addr >= slave_cfg.holding_register_count) {
					resp_pdu_len = build_exception_pdu(resp_pdu, fn, 0x02); break;
				}
				slave_cfg.holding_registers[addr] = val;
				if (slave_cfg.on_write_single_register) slave_cfg.on_write_single_register(addr, val);
				resp_pdu[0] = fn;
				resp_pdu[1] = pdu[1];
				resp_pdu[2] = pdu[2];
				resp_pdu[3] = pdu[3];
				resp_pdu[4] = pdu[4];
				resp_pdu_len = 5;
				break;
			}
			case MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS: {
				if (pdu_len < 6) { resp_pdu_len = build_exception_pdu(resp_pdu, fn, 0x03); break; }
				uint16_t addr = (uint16_t)(pdu[1] << 8 | pdu[2]);
				uint16_t count = (uint16_t)(pdu[3] << 8 | pdu[4]);
				uint8_t byte_count = pdu[5];
				if (byte_count != count * 2 || pdu_len != (uint16_t)(6 + byte_count) || (uint32_t)addr + count > slave_cfg.holding_register_count) {
					resp_pdu_len = build_exception_pdu(resp_pdu, fn, 0x03); break;
				}
				for (uint16_t i = 0; i < count; i++) {
					uint16_t v = (uint16_t)(pdu[6 + i * 2] << 8 | pdu[7 + i * 2]);
					slave_cfg.holding_registers[addr + i] = v;
				}
				if (slave_cfg.on_write_multiple_registers) {
					slave_cfg.on_write_multiple_registers(addr, &slave_cfg.holding_registers[addr], count);
				}
				resp_pdu[0] = fn;
				resp_pdu[1] = pdu[1];
				resp_pdu[2] = pdu[2];
				resp_pdu[3] = pdu[3];
				resp_pdu[4] = pdu[4];
				resp_pdu_len = 5;
				break;
			}
			case MODBUS_FUNC_READ_INPUT_REGISTERS: {
				if (pdu_len < 5) { resp_pdu_len = build_exception_pdu(resp_pdu, fn, 0x03); break; }
				uint16_t addr = (uint16_t)(pdu[1] << 8 | pdu[2]);
				uint16_t count = (uint16_t)(pdu[3] << 8 | pdu[4]);
				if (count == 0 || (uint32_t)addr + count > slave_cfg.input_register_count) {
					resp_pdu_len = build_exception_pdu(resp_pdu, fn, 0x02); break;
				}
				if (slave_cfg.on_read_input_registers) slave_cfg.on_read_input_registers(addr, count);
				resp_pdu[0] = fn;
				resp_pdu[1] = (uint8_t)(count * 2);
				for (uint16_t i = 0; i < count; i++) {
					uint16_t v = slave_cfg.input_registers[addr + i];
					resp_pdu[2 + i * 2] = (uint8_t)(v >> 8);
					resp_pdu[3 + i * 2] = (uint8_t)(v & 0xFF);
				}
				resp_pdu_len = (uint16_t)(2 + count * 2);
				break;
			}
			default: {
				resp_pdu_len = build_exception_pdu(resp_pdu, fn, 0x01);
				break;
			}
		}
		if (resp_pdu_len > 0) {
			send_tcp_response(tid, uid, resp_pdu, resp_pdu_len);
		}
		return;
	}

	// RTU flow (existing)
	if (len < 5)
		return;

	if (frame[0] != slave_cfg.id)
		return;

	uint16_t crc_recv = frame[len - 2] | (frame[len - 1] << 8);
	uint16_t crc_calc = modbus_crc16(frame, len - 2);
	if (crc_recv != crc_calc)
		return;

	modbus_function_code_t func = (modbus_function_code_t) frame[1];
	uint16_t addr = 0;
	uint16_t quantity = 0;
	uint8_t byte_count = 0;
	uint8_t response[256];
	// printf("Request to %d with Function Code 0x%02X\n",frame[0],func);
	switch (func) {
	case MODBUS_FUNC_READ_COILS: // Read Coils
	{

		addr = (frame[2] << 8) | frame[3];
		quantity = (frame[4] << 8) | frame[5];
		if ((addr + quantity) > slave_cfg.coil_count)
			return;

		byte_count = (quantity + 7) / 8;
		response[0] = slave_cfg.id;
		response[1] = func;
		response[2] = byte_count;

		for (uint16_t i = 0; i < quantity; ++i) {
			if (slave_cfg.coils[addr + i])
				response[3 + i / 8] |= (1 << (i % 8));
			else
				response[3 + i / 8] &= ~(1 << (i % 8));
		}

		if (slave_cfg.on_read_coils)
			slave_cfg.on_read_coils(addr, quantity);

		uint16_t crc = modbus_crc16(response, 3 + byte_count);
		response[3 + byte_count] = crc & 0xFF;
		response[4 + byte_count] = crc >> 8;

		send_response(response, 5 + byte_count);

	}
		break;
	case MODBUS_FUNC_READ_DISCRETE_INPUTS: // Read Discrete Inputs
	{
		addr = (frame[2] << 8) | frame[3];
		quantity = (frame[4] << 8) | frame[5];
		if ((addr + quantity) > slave_cfg.discrete_input_count)
			return;

		byte_count = (quantity + 7) / 8;
		response[0] = slave_cfg.id;
		response[1] = func;
		response[2] = byte_count;

		for (uint16_t i = 0; i < quantity; ++i) {
			if (slave_cfg.discrete_inputs[addr + i])
				response[3 + i / 8] |= (1 << (i % 8));
			else
				response[3 + i / 8] &= ~(1 << (i % 8));
		}

		if (slave_cfg.on_read_discrete_inputs)
			slave_cfg.on_read_discrete_inputs(addr, quantity);

		uint16_t crc = modbus_crc16(response, 3 + byte_count);
		response[3 + byte_count] = crc & 0xFF;
		response[4 + byte_count] = crc >> 8;

		send_response(response, 5 + byte_count);
	}
		break;
	case MODBUS_FUNC_READ_HOLDING_REGISTERS: { // Read Holding Registers
		if (len != 8)
			return;
		uint16_t addr = (frame[2] << 8) | frame[3];
		uint16_t count = (frame[4] << 8) | frame[5];
		if (addr + count > slave_cfg.holding_register_count || count == 0)
			return;
		if (slave_cfg.on_read_holding_registers) {
			slave_cfg.on_read_holding_registers(addr, count);
		}
		uint8_t resp[256];
		resp[0] = slave_cfg.id;
		resp[1] = func;
		resp[2] = count * 2;
		for (uint16_t i = 0; i < count; i++) {
			resp[3 + i * 2] = slave_cfg.holding_registers[addr + i] >> 8;
			resp[4 + i * 2] = slave_cfg.holding_registers[addr + i] & 0xFF;
		}

		uint16_t crc = modbus_crc16(resp, 3 + count * 2);
		resp[3 + count * 2] = crc & 0xFF;
		resp[4 + count * 2] = crc >> 8;

		send_response(resp, 5 + count * 2);
		break;
	}
	case MODBUS_FUNC_READ_INPUT_REGISTERS: // Read Input Registers
	{
		addr = (frame[2] << 8) | frame[3];
		quantity = (frame[4] << 8) | frame[5];
		if ((addr + quantity) > slave_cfg.input_register_count)
			return;

		byte_count = quantity * 2;
		response[0] = slave_cfg.id;
		response[1] = func;
		response[2] = byte_count;

		for (uint16_t i = 0; i < quantity; ++i) {
			response[3 + 2 * i] = slave_cfg.input_registers[addr + i] >> 8;
			response[3 + 2 * i + 1] = slave_cfg.input_registers[addr + i]
					& 0xFF;
		}

		if (slave_cfg.on_read_input_registers)
			slave_cfg.on_read_input_registers(addr, quantity);

		uint16_t crc = modbus_crc16(response, 3 + byte_count);
		response[3 + byte_count] = crc & 0xFF;
		response[4 + byte_count] = crc >> 8;

		send_response(response, 5 + byte_count);
	}
		break;
	case MODBUS_FUNC_WRITE_SINGLE_COIL: // Write Single Coil
	{
		addr = (frame[2] << 8) | frame[3];
		uint16_t value = (frame[4] << 8) | frame[5];
		if (addr >= slave_cfg.coil_count)
			return;

		slave_cfg.coils[addr] = (value == 0xFF00) ? 1 : 0;

		if (slave_cfg.on_write_single_coil)
			slave_cfg.on_write_single_coil(addr, slave_cfg.coils[addr]);
		memcpy(response, frame, 6);  // Echo request
		uint16_t crc = modbus_crc16(response, 6);
		response[3 + byte_count] = crc & 0xFF;
		response[4 + byte_count] = crc >> 8;

		send_response(response, 8);
	}
		break;

	case MODBUS_FUNC_WRITE_SINGLE_REGISTER: { // Write Single Register
		if (len != 8)
			return;
		uint16_t addr = (frame[2] << 8) | frame[3];
		uint16_t val = (frame[4] << 8) | frame[5];
		if (addr >= slave_cfg.holding_register_count)
			return;

		slave_cfg.holding_registers[addr] = val;
		if (slave_cfg.on_write_single_register) {
			slave_cfg.on_write_single_register(addr, val);
		}

		// Echo original frame
		send_response((uint8_t*) frame, len);
		break;
	}
	case MODBUS_FUNC_WRITE_MULTIPLE_COILS: // Write Multiple Coils
	{
		addr = (frame[2] << 8) | frame[3];
		quantity = (frame[4] << 8) | frame[5];
		byte_count = frame[6];
		if ((addr + quantity) > slave_cfg.coil_count)
			return;

		for (uint16_t i = 0; i < quantity; ++i) {
			uint8_t bit = (frame[7 + i / 8] >> (i % 8)) & 0x01;
			slave_cfg.coils[addr + i] = bit;
		}

		if (slave_cfg.on_write_multiple_coils)
			slave_cfg.on_write_multiple_coils(addr, &frame[7], quantity);

		response[0] = slave_cfg.id;
		response[1] = 0x0F;
		response[2] = frame[2];
		response[3] = frame[3];
		response[4] = frame[4];
		response[5] = frame[5];

		uint16_t crc = modbus_crc16(response, 6);
		response[3 + byte_count] = crc & 0xFF;
		response[4 + byte_count] = crc >> 8;

		send_response(response, 8);

	}
		break;
	case MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS: { // Write Multiple Registers
		if (len < 9)
			return;
		uint16_t addr = (frame[2] << 8) | frame[3];
		uint16_t count = (frame[4] << 8) | frame[5];
		uint8_t byte_count = frame[6];

		if (addr + count > slave_cfg.holding_register_count)
			return;
		if (byte_count != count * 2 || len != 9 + byte_count)
			return;

		for (uint16_t i = 0; i < count; i++) {
			uint16_t val = (frame[7 + i * 2] << 8) | frame[8 + i * 2];
			slave_cfg.holding_registers[addr + i] = val;
		}

		if (slave_cfg.on_write_multiple_registers) {
			slave_cfg.on_write_multiple_registers(addr,
					&slave_cfg.holding_registers[addr], count);
		}

		uint8_t resp[8];
		memcpy(resp, frame, 6);
		uint16_t crc = modbus_crc16(resp, 6);
		resp[6] = crc & 0xFF;
		resp[7] = crc >> 8;

		send_response(resp, 8);
		break;
	}
	default:
		// unsupported function code
		break;
	}
}
