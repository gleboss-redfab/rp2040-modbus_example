#pragma once

#include <stdint.h>

// Modbus errors
#define MB_NO_ERROR                     0x00
#define MB_ERROR_ILLEGAL_FUNCTION       0x01
#define MB_ERROR_ILLEGAL_DATA_ADDRESS   0x02
#define MB_ERROR_ILLEGAL_DATA_VALUE     0x03
#define MB_ERROR_SLAVE_DEVICE_FAILURE   0x04

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

// Commands
#define MB_READ_COIL_STATUS         0x01
#define MB_READ_INPUT_STATUS        0x02
#define MB_READ_HOLDING_REGISTERS   0x03
#define MB_READ_INPUT_REGISTERS     0x04
#define MB_WRITE_SINGLE_COIL        0x05
#define MB_WRITE_SINGLE_REGISTER    0x06
#define MB_WRITE_MULTIPLE_COILS     0x10
#define MB_WRITE_MULTIPLE_REGISTERS 0x0F
#define MB_RX_BUF_SIZE              64
#define MB_TX_BUF_SIZE              64
#define MB_TIMEOUT                  100


void mb_init(uint8_t slave_address);
void mb_rx(uint8_t b);
void mb_response_add(uint16_t value);
void mb_response_add_single_register(uint16_t value);
void mb_response_reset(uint8_t fn);
void mb_process();

uint32_t mb_get_tick_ms(void);
extern void mb_tx(uint8_t* data, uint32_t len);

uint8_t mb_read_coil_status(uint16_t start, uint16_t count);
uint8_t mb_read_input_status(uint16_t start, uint16_t count);
uint8_t mb_read_holding_registers(uint16_t start, uint16_t count);
uint8_t mb_read_input_registers(uint16_t start, uint16_t count);
uint8_t mb_write_single_coil(uint16_t start, uint16_t value);
uint8_t mb_write_single_register(uint16_t start, uint16_t value);
uint8_t mb_write_multiple_coils(uint16_t start, uint8_t* values, uint16_t len);
uint8_t mb_write_multiple_registers(uint16_t start, uint16_t* values, uint16_t len);
