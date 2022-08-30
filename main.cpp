#include <hardware/irq.h>
#include <hardware/uart.h>
#include <pico/stdlib.h>
#include <stdio.h>

#include "modbus.hpp"

#define MB_SLAVE_ADDRESS 1

#define LED_PIN          PICO_DEFAULT_LED_PIN

#define MB_UART          uart0
#define MB_UART_IRQ      UART0_IRQ

#define MB_DE_PIN        7
#define MB_TX_PIN        0
#define MB_RX_PIN        1

extern uint16_t state;            
extern uint16_t error_code;       
extern uint16_t busy_code;      

extern uint16_t command;          
extern uint16_t command_param[3];

extern uint16_t sensor_0;         
extern uint16_t sensor_1;         
extern uint16_t sensor_2;         

void mb_tx(uint8_t* data, uint32_t size) {
  gpio_put(MB_DE_PIN, 1);

  uart_write_blocking(MB_UART, data, size);

  // Wait until fifo is drained so we now when to turn off the driver enable pin.
  uart_tx_wait_blocking(MB_UART);
  gpio_put(MB_DE_PIN, 0);
}


void on_mb_rx() {
  mb_rx(uart_getc(MB_UART));
}


void setup(void) {
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  gpio_init(MB_DE_PIN);
  gpio_set_dir(MB_DE_PIN, GPIO_OUT);
  gpio_put(MB_DE_PIN, 0);

  uart_init(MB_UART, 115200);      // Modbus
  uart_set_format(MB_UART, 8, 1, UART_PARITY_NONE);

  gpio_set_function(MB_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(MB_RX_PIN, GPIO_FUNC_UART);

  uart_set_fifo_enabled(MB_UART, false);

  irq_set_exclusive_handler(MB_UART_IRQ, on_mb_rx);

  irq_set_enabled(MB_UART_IRQ, true);

  uart_set_irq_enables(MB_UART, true, false);

  mb_init(MB_SLAVE_ADDRESS);
}

int main(void)
{
    stdio_init_all();

    printf("Modbus demo firmware start\r\n");


    setup();

    for (;;) 
    {
        mb_process();

        uint16_t time_sec = (uint16_t) (time_us_64()/(1000*1000));

        sensor_0 = time_sec*10;
        sensor_1 = time_sec*20;
        sensor_2 = time_sec*30;
    }
}

