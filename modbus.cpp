#include "modbus.hpp"

#include <hardware/irq.h>
#include <hardware/uart.h>
#include <pico/stdlib.h>
#include <stdio.h>

#include <string.h>

// global dummy variable
uint16_t state              = 0;
uint16_t error_code         = 10;
uint16_t busy_code          = 20;

uint16_t command            = 100;
uint16_t command_param[3]   = {0}; 

uint16_t sensor_0           = 1000;
uint16_t sensor_1           = 1001;
uint16_t sensor_2           = 1002;

typedef enum { MB_DATA_READY, MB_DATA_INCOMPLETE, MB_INVALID_SLAVE_ADDRESS, MB_INVALID_FUNCTION } mb_state_t;

uint8_t mb_slave_address = 0;
uint8_t mb_request_buf[MB_RX_BUF_SIZE];
uint8_t mb_response_buf[MB_TX_BUF_SIZE];
int mb_request_buf_pos = 0;
int mb_response_buf_pos = 0;
uint32_t mb_timeout;



__attribute__((weak)) uint8_t mb_read_coil_status(uint16_t start, uint16_t count) {
  return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

__attribute__((weak)) uint8_t mb_read_input_status(uint16_t start, uint16_t count) {
  return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

// __attribute__((weak)) uint8_t mb_read_holding_registers(uint16_t start, uint16_t count) {
//   return MB_ERROR_ILLEGAL_DATA_ADDRESS;
// }

__attribute__((weak)) uint8_t mb_read_input_registers(uint16_t start, uint16_t count) {
  return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

__attribute__((weak)) uint8_t mb_write_single_coil(uint16_t start, uint16_t value) {
  return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

// __attribute__((weak)) uint8_t mb_write_single_register(uint16_t start, uint16_t value) {
//   return MB_ERROR_ILLEGAL_DATA_ADDRESS;
// }

__attribute__((weak)) uint8_t mb_write_multiple_coils(uint16_t start, uint8_t* values, uint16_t len) {
  return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

__attribute__((weak)) uint8_t mb_write_multiple_registers(uint16_t start, uint16_t* values, uint16_t len) {
  return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

static uint16_t mb_calc_crc16(const uint8_t* buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  uint8_t i, j = 0;
  while (j < len) {
    crc ^= buf[j];
    for (i = 0; i < 8; i++) {
      if (crc & 0x01) {
        crc >>= 1;
        crc ^= 0xA001;
      } else
        crc >>= 1;
    }
    j++;
  }
  return crc;
}

static mb_state_t mb_check_buf() {
  if (mb_request_buf_pos > 4) {
    if (mb_request_buf[0] != mb_slave_address || mb_slave_address == 0) {
      return MB_INVALID_SLAVE_ADDRESS;
    }

    if (mb_request_buf[1] >= 0x01 && mb_request_buf[1] <= 0x06) {
      if (mb_request_buf_pos == 8) {
        return MB_DATA_READY;
      }
    } else if (mb_request_buf[1] == 0x10 || mb_request_buf[1] == 0x0F) {
      if (mb_request_buf_pos == mb_request_buf[6] + 9) {
        return MB_DATA_READY;
      }
    } else {
      return MB_INVALID_FUNCTION;
    }
  }

  return MB_DATA_INCOMPLETE;
}

static void mb_reset_buf() {
  mb_request_buf_pos = 0;
  memset(mb_request_buf, 0, sizeof(mb_request_buf));
}

static void mb_response_tx() {
  // Calculate CRC
  uint16_t crc = mb_calc_crc16(mb_response_buf, mb_response_buf_pos);
  mb_response_buf[mb_response_buf_pos++] = crc & 0xFF;
  mb_response_buf[mb_response_buf_pos++] = (crc & 0xFF00) >> 8;

  // Send RTU packet
  mb_tx(mb_response_buf, mb_response_buf_pos);
}

static void mb_error(uint8_t err) {
  mb_response_buf_pos = 0;
  mb_response_buf[mb_response_buf_pos++] = mb_slave_address;
  mb_response_buf[mb_response_buf_pos++] = mb_request_buf[1] | 0x80;
  mb_response_buf[mb_response_buf_pos++] = err;
  mb_response_tx();
}

//тут поменял местами запись в регистры
void mb_response_add(uint16_t value) {
  mb_response_buf[2] += 2;
  mb_response_buf[mb_response_buf_pos++] = (value & 0xFF00) >> 8;
  mb_response_buf[mb_response_buf_pos++] = value & 0xFF;

}

// для write single register
void mb_response_add_single_register(uint16_t value) {
  mb_response_buf_pos--; // не надо передавать длинну сбщ
  mb_response_buf[mb_response_buf_pos++] = (value & 0xFF00) >> 8;
  mb_response_buf[mb_response_buf_pos++] = value & 0xFF;

}

void mb_response_reset(uint8_t fn) {
  mb_response_buf_pos = 0;
  mb_response_buf[mb_response_buf_pos++] = mb_slave_address;
  mb_response_buf[mb_response_buf_pos++] = fn;
  mb_response_buf[mb_response_buf_pos++] = 0;
}

static void mb_rx_rtu() {
  uint8_t res;

  // Check CRC
  uint16_t crc = mb_calc_crc16(mb_request_buf, mb_request_buf_pos - 2);
  if (memcmp(&crc, &mb_request_buf[mb_request_buf_pos - 2], sizeof(crc)) != 0) {
    mb_reset_buf();
    return;
  }

  mb_response_reset(mb_request_buf[1]);
  switch (mb_request_buf[1]) {
    case MB_READ_COIL_STATUS:
      res = mb_read_coil_status((mb_request_buf[2] << 8) + mb_request_buf[3],
                                (mb_request_buf[4] << 8) + mb_request_buf[5]);
      break;
    case MB_READ_INPUT_STATUS:
      res = mb_read_input_status((mb_request_buf[2] << 8) + mb_request_buf[3],
                                 (mb_request_buf[4] << 8) + mb_request_buf[5]);
      break;
    case MB_READ_HOLDING_REGISTERS:
      res = mb_read_holding_registers((mb_request_buf[2] << 8) + mb_request_buf[3],
                                      (mb_request_buf[4] << 8) + mb_request_buf[5]);
      break;
    case MB_READ_INPUT_REGISTERS:
      res = mb_read_input_registers((mb_request_buf[2] << 8) + mb_request_buf[3],
                                    (mb_request_buf[4] << 8) + mb_request_buf[5]);
      break;
    case MB_WRITE_SINGLE_COIL:
      res = mb_write_single_coil((mb_request_buf[2] << 8) + mb_request_buf[3],
                                 (mb_request_buf[4] << 8) + mb_request_buf[5]);
      break;
    case MB_WRITE_SINGLE_REGISTER:
      res = mb_write_single_register((mb_request_buf[2] << 8) + mb_request_buf[3],
                                     (mb_request_buf[4] << 8) + mb_request_buf[5]);
      break;
    case MB_WRITE_MULTIPLE_COILS:
      res = mb_write_multiple_coils((mb_request_buf[2] << 8) + mb_request_buf[3], &mb_request_buf[6],
                                    (mb_request_buf[4] << 8) + mb_request_buf[5]);
      break;
    case MB_WRITE_MULTIPLE_REGISTERS:
      res = mb_write_multiple_registers((mb_request_buf[2] << 8) + mb_request_buf[3], (uint16_t*)&mb_request_buf[6],
                                        (mb_request_buf[4] << 8) + mb_request_buf[5]);
      break;
    default:
      res = MB_ERROR_ILLEGAL_FUNCTION;
      break;
  }

  if (res == MB_NO_ERROR) {
    mb_response_tx();
  } else {
    mb_error(res);
  }

  mb_reset_buf();
}

void mb_init(uint8_t slave_address) {
  mb_slave_address = slave_address;
  mb_reset_buf();
}

void mb_rx(uint8_t b) {
  if (mb_get_tick_ms() - mb_timeout > MB_TIMEOUT) {
    mb_reset_buf();
  }
  mb_timeout = mb_get_tick_ms();
  if (mb_request_buf_pos < (sizeof(mb_request_buf) - 1)) {
    mb_request_buf[mb_request_buf_pos++] = b;
  }
}

void mb_process() {
  mb_state_t mb_state = mb_check_buf();
  switch (mb_state) {
    case MB_INVALID_FUNCTION:
      mb_error(MB_ERROR_ILLEGAL_FUNCTION);
    case MB_INVALID_SLAVE_ADDRESS:
      mb_reset_buf();
      break;
    case MB_DATA_READY:
      mb_rx_rtu();
    default:
    case MB_DATA_INCOMPLETE:
      break;
  }
}

uint8_t mb_write_single_register(uint16_t start, uint16_t value)
{
    uint16_t val;
    uint16_t addr = start;

    switch (addr) 
    {
    // case MB_VOLTAGE_L2_N:
    //     registers.voltage_l2 = value;
    //     break;
    // case MB_STATE_REGISTER:
    //     state = value;
    //     break;
    // case MB_ERROR_CODE_REGISTER:
    //     error_code = value;
    //     break;
    // case MB_BUSY_CODE_REGISTER:
    //     busy_code = value;
    //     break;
    case MB_COMMAND_REGISTER:
        command = value;
        break;
    case MB_COMMAND_PARAM_0_REGISTER:
        command_param[0] = value;
        break;
    case MB_COMMAND_PARAM_1_REGISTER:
        command_param[1] = value;
        break;
    case MB_COMMAND_PARAM_2_REGISTER:
        command_param[2] = value;
        break;
    // case MB_SENSOR_0_REGISTER:
    //     sensor_0 = value;
    //     break;
    default:
        return MB_ERROR_ILLEGAL_DATA_ADDRESS;
    }

    mb_response_add_single_register(start);
    mb_response_add_single_register(value);
    
    return MB_NO_ERROR;
}

static uint8_t mb_read_holding_register(uint16_t addr, uint16_t* reg) 
{
    switch (addr) 
    {
    case MB_STATE_REGISTER:
        *reg = state;
        break;
    case MB_ERROR_CODE_REGISTER:
        *reg = error_code;
        break;
    case MB_BUSY_CODE_REGISTER:
        *reg = busy_code;
        break;

    case MB_COMMAND_REGISTER:
        *reg = command;
        break;

    case MB_SENSOR_0_REGISTER:
        *reg = sensor_0;
        break;
    case MB_SENSOR_1_REGISTER:
        *reg = sensor_1;
        break;
    case MB_SENSOR_2_REGISTER:
        *reg = sensor_2;
        break;
    default:
        return MB_ERROR_ILLEGAL_DATA_ADDRESS;
    }
    return MB_NO_ERROR;
}

uint8_t mb_read_holding_registers(uint16_t start, uint16_t count) 
{
  uint16_t val;
  for (int i = 0; i < count; i++) {
    if (mb_read_holding_register(start + i, &val) == MB_NO_ERROR) 
    {
      mb_response_add(val);
    } 
    else 
    {
      return MB_ERROR_ILLEGAL_DATA_ADDRESS;
    }
  }
  return MB_NO_ERROR;
}


uint32_t mb_get_tick_ms(void) {
  return time_us_64() / 1000;
}