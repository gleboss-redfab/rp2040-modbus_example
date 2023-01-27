#pragma once

#include <stdint.h>
#include <hardware/uart.h>

// Constatnts
#define MB_RX_BUF_SIZE                  64
#define MB_TX_BUF_SIZE                  64
#define MB_TIMEOUT                      100


// определено в main
// КОСТЫЛЬ
extern void on_mb_rx();

// хз почему внутри класса не работает
// КОСТЫЛЬ
typedef enum { MB_DATA_READY, MB_DATA_INCOMPLETE, MB_INVALID_SLAVE_ADDRESS, MB_INVALID_FUNCTION } mb_state_t;


class ModbusManager
{
public:
    void mb_init(uint8_t slave_address, uint8_t uart_num, 
                uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uart_parity_t parity, 
                uint8_t rx_pin, uint8_t tx_pin, uint8_t de_pin);

    void mb_rx(uint8_t data);
    void mb_process();

    uint8_t uart_number;

    //dummy data stored here
    uint16_t state              = 0;
    uint16_t error_code         = 10;
    uint16_t busy_code          = 20;

    uint16_t command            = 100;
    uint16_t command_param[3]   = {0}; 

    uint16_t sensor_0           = 1000;
    uint16_t sensor_1           = 1001;
    uint16_t sensor_2           = 1002;

private:
    void mb_response_add(uint16_t value);
    void mb_response_add_single_register(uint16_t value);
    void mb_response_reset(uint8_t fn);

    void mb_tx(uint8_t* data, uint32_t size);

    uint32_t mb_get_tick_ms(void);

    uint8_t mb_read_coil_status(uint16_t start, uint16_t count);
    uint8_t mb_read_input_status(uint16_t start, uint16_t count);
    uint8_t mb_read_holding_registers(uint16_t start, uint16_t count);
    uint8_t mb_read_input_registers(uint16_t start, uint16_t count);
    uint8_t mb_write_single_coil(uint16_t start, uint16_t value);
    uint8_t mb_write_single_register(uint16_t start, uint16_t value);
    uint8_t mb_write_multiple_coils(uint16_t start, uint8_t* values, uint16_t len);
    uint8_t mb_write_multiple_registers(uint16_t start, uint16_t* values, uint16_t len);

    uint8_t mb_read_holding_register(uint16_t addr, uint16_t* reg);
    uint16_t mb_calc_crc16(const uint8_t* buf, uint8_t len);
    mb_state_t mb_check_buf();
    
    void mb_reset_buf(); 
    void mb_error(uint8_t err);
    void mb_rx_rtu();
    void mb_response_tx();

    uint8_t de_pin;
    uint8_t mb_slave_address = 0;

    uint8_t mb_request_buf[MB_RX_BUF_SIZE];
    uint8_t mb_response_buf[MB_TX_BUF_SIZE];

    int mb_request_buf_pos = 0;
    int mb_response_buf_pos = 0;

    uint32_t mb_timeout;
};
