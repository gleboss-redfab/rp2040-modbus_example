#include "modbus.hpp"

#include <stdio.h>
#include <string.h>

#include <hardware/irq.h>
#include <hardware/uart.h>
#include <pico/stdlib.h>

// Modbus errors
#define MB_NO_ERROR                     0x00
#define MB_ERROR_ILLEGAL_FUNCTION       0x01
#define MB_ERROR_ILLEGAL_DATA_ADDRESS   0x02
#define MB_ERROR_ILLEGAL_DATA_VALUE     0x03
#define MB_ERROR_SLAVE_DEVICE_FAILURE   0x04

// Commands
#define MB_READ_COIL_STATUS             0x01
#define MB_READ_INPUT_STATUS            0x02
#define MB_READ_HOLDING_REGISTERS       0x03
#define MB_READ_INPUT_REGISTERS         0x04
#define MB_WRITE_SINGLE_COIL            0x05
#define MB_WRITE_SINGLE_REGISTER        0x06
#define MB_WRITE_MULTIPLE_COILS         0x10
#define MB_WRITE_MULTIPLE_REGISTERS     0x0F

// Registers adresses
#define MB_STATE_REGISTER               0
#define MB_ERROR_CODE_REGISTER          10
#define MB_BUSY_CODE_REGISTER           20

#define MB_COMMAND_REGISTER             100
#define MB_COMMAND_PARAM_0_REGISTER     110
#define MB_COMMAND_PARAM_1_REGISTER     111
#define MB_COMMAND_PARAM_2_REGISTER     112

#define MB_SENSOR_0_REGISTER            1000
#define MB_SENSOR_1_REGISTER            1001
#define MB_SENSOR_2_REGISTER            1002

__attribute__((weak)) 
uint8_t ModbusManager::mb_read_coil_status(uint16_t start, uint16_t count) {
  return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

__attribute__((weak)) 
uint8_t ModbusManager::mb_read_input_status(uint16_t start, uint16_t count) {
  return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

// РАЛИЗОВАННО
// __attribute__((weak)) uint8_t mb_read_holding_registers(uint16_t start, uint16_t count) {
//   return MB_ERROR_ILLEGAL_DATA_ADDRESS;
// }

__attribute__((weak))
uint8_t ModbusManager::mb_read_input_registers(uint16_t start, uint16_t count) {
  return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

__attribute__((weak))
uint8_t ModbusManager::mb_write_single_coil(uint16_t start, uint16_t value) {
  return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

// РАЛИЗОВАННО
// __attribute__((weak)) uint8_t mb_write_single_register(uint16_t start, uint16_t value) {
//   return MB_ERROR_ILLEGAL_DATA_ADDRESS;
// }

__attribute__((weak)) 
uint8_t ModbusManager::mb_write_multiple_coils(uint16_t start, uint8_t* values, uint16_t len) {
  return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}

__attribute__((weak)) 
uint8_t ModbusManager::mb_write_multiple_registers(uint16_t start, uint16_t* values, uint16_t len) {
  return MB_ERROR_ILLEGAL_DATA_ADDRESS;
}


uint16_t ModbusManager::mb_calc_crc16(const uint8_t* buf, uint8_t len) 
{
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


mb_state_t ModbusManager::mb_check_buf() 
{
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


 void ModbusManager::mb_reset_buf() 
{
  mb_request_buf_pos = 0;
  memset(mb_request_buf, 0, sizeof(mb_request_buf));
}


void ModbusManager::mb_response_tx() 
{
  // Calculate CRC
  uint16_t crc = mb_calc_crc16(mb_response_buf, mb_response_buf_pos);
  mb_response_buf[mb_response_buf_pos++] = crc & 0xFF;
  mb_response_buf[mb_response_buf_pos++] = (crc & 0xFF00) >> 8;

  // Send RTU packet
  mb_tx(mb_response_buf, mb_response_buf_pos);
}


void ModbusManager::mb_error(uint8_t err) 
{
  mb_response_buf_pos = 0;
  mb_response_buf[mb_response_buf_pos++] = mb_slave_address;
  mb_response_buf[mb_response_buf_pos++] = mb_request_buf[1] | 0x80;
  mb_response_buf[mb_response_buf_pos++] = err;
  mb_response_tx();
}


//тут поменял местами запись в регистры
void ModbusManager::mb_response_add(uint16_t value) 
{
  mb_response_buf[2] += 2;
  mb_response_buf[mb_response_buf_pos++] = (value & 0xFF00) >> 8;
  mb_response_buf[mb_response_buf_pos++] = value & 0xFF;

}


// для write single register
void ModbusManager::mb_response_add_single_register(uint16_t value) 
{
  mb_response_buf_pos--; // не надо передавать длинну сбщ
  mb_response_buf[mb_response_buf_pos++] = (value & 0xFF00) >> 8;
  mb_response_buf[mb_response_buf_pos++] = value & 0xFF;

}


void ModbusManager::mb_response_reset(uint8_t fn) 
{
  mb_response_buf_pos = 0;
  mb_response_buf[mb_response_buf_pos++] = mb_slave_address;
  mb_response_buf[mb_response_buf_pos++] = fn;
  mb_response_buf[mb_response_buf_pos++] = 0;
}


void ModbusManager::mb_rx_rtu() 
{
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

  if (res == MB_NO_ERROR) 
  {
    mb_response_tx();
  } 
  else 
  {
    mb_error(res);
  }

  mb_reset_buf();
}

void ModbusManager::mb_init(uint8_t slave_address, uint8_t uart_num, 
                                uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uart_parity_t parity, 
                                uint8_t rx_pin, uint8_t tx_pin, uint8_t de_pin)
{
  mb_slave_address = slave_address;

  // DE pin init
  gpio_init(de_pin);
  gpio_set_dir(de_pin, GPIO_OUT);
  gpio_put(de_pin, 0);
  this->de_pin = de_pin;

  // UART init
  gpio_set_function(tx_pin, GPIO_FUNC_UART);
  gpio_set_function(rx_pin, GPIO_FUNC_UART);

  if(uart_num == 0)
  {
    uart_number = 0;
    uart_init(uart0, baudrate);      
    uart_set_format(uart0, data_bits, stop_bits, parity);

    uart_set_fifo_enabled(uart0, false);

    irq_set_exclusive_handler(UART0_IRQ, on_mb_rx);
 
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);
  }
  else if(uart_num == 1)
  {
    uart_number = 1;
    uart_init(uart1, baudrate);      
    uart_set_format(uart1, data_bits, stop_bits, parity);

    uart_set_fifo_enabled(uart1, false);

    irq_set_exclusive_handler(UART1_IRQ, on_mb_rx);

    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, true, false);
  }

  mb_reset_buf();
}

void ModbusManager::mb_rx(uint8_t data) 
{
  if (mb_get_tick_ms() - mb_timeout > MB_TIMEOUT) 
    mb_reset_buf();

  mb_timeout = mb_get_tick_ms();

  if (mb_request_buf_pos < (sizeof(mb_request_buf) - 1)) 
    mb_request_buf[mb_request_buf_pos++] = data;
}


void ModbusManager::mb_process() 
{
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


// Задаются только writable регистры
// чтобы добавить новые, сделать по образцу существующих case
uint8_t ModbusManager::mb_write_single_register(uint16_t start, uint16_t value)
{
    uint16_t val;
    uint16_t addr = start;

    switch (addr) 
    {
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

    default:
        return MB_ERROR_ILLEGAL_DATA_ADDRESS;
    }

    mb_response_add_single_register(start);
    mb_response_add_single_register(value);
    
    return MB_NO_ERROR;
}


// readable регистры
// чтобы добавить новые, сделать по образцу существующих case
uint8_t ModbusManager::mb_read_holding_register(uint16_t addr, uint16_t* reg) 
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


uint8_t ModbusManager::mb_read_holding_registers(uint16_t start, uint16_t count) 
{
  uint16_t val;
  for (int i = 0; i < count; i++) 
  {
    if (mb_read_holding_register(start + i, &val) == MB_NO_ERROR) 
      mb_response_add(val);
    else 
      return MB_ERROR_ILLEGAL_DATA_ADDRESS;
  }
  return MB_NO_ERROR;
}


uint32_t ModbusManager::mb_get_tick_ms(void) 
{
  return time_us_64() / 1000;
}

void ModbusManager::mb_tx(uint8_t* data, uint32_t size) 
{
  gpio_put(de_pin, 1);

  if (uart_number == 0)
    uart_write_blocking(uart0, data, size);
  else
    uart_write_blocking(uart1, data, size);

  // Wait until fifo is drained so we now when to turn off the driver enable pin.
  if (uart_number == 0)
    uart_tx_wait_blocking(uart0);
  else
    uart_tx_wait_blocking(uart1);

  gpio_put(de_pin, 0);
}
