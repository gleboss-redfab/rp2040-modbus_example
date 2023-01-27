#include <hardware/irq.h>
#include <hardware/uart.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include "pico/multicore.h"

#include "modbus.hpp"

//////////////////////////////////////////////////////////////////
// Modbus parameters
#define MB_SLAVE_ADDRESS 1
#define MB_BAUDRATE      115200
#define MB_DATA_BITS     8
#define MB_STOP_BITS     1
#define MB_PARITY        UART_PARITY_NONE

#define MB_UART_NUMBER   1
#define MB_TX_PIN        4
#define MB_RX_PIN        5

#define MB_DE_PIN        7

ModbusManager modbus;
//////////////////////////////////////////////////////////////////


// костыль, тк irq_handler не модет быть функция - метод класса
// внутри .hpp файла реализовать не получилось, тк надо вызывать функцию - метод класса
// работает и норм -_-
void on_mb_rx() 
{
  if (modbus.uart_number == 0)
    modbus.mb_rx(uart_getc(uart0));
  else
    modbus.mb_rx(uart_getc(uart1));
}

void modbus_process_on_core_1()
{
  while(true)
  {
    modbus.mb_process();

    uint16_t time_sec = (uint16_t) (time_us_64()/(1000*1000));

    modbus.sensor_0 = time_sec*10;
    modbus.sensor_1 = time_sec*20;
    modbus.sensor_2 = time_sec*30;
  }
}


int main(void)
{
  stdio_init_all();
  printf("Modbus demo firmware start\r\n");

  modbus.mb_init(MB_SLAVE_ADDRESS, 
                 MB_UART_NUMBER, 
                 MB_BAUDRATE, 
                 MB_DATA_BITS, 
                 MB_STOP_BITS, 
                 MB_PARITY, 
                 MB_RX_PIN, 
                 MB_TX_PIN, 
                 MB_DE_PIN);

  multicore_launch_core1(modbus_process_on_core_1);

  while(true)
  {
    printf("core 0 in sleep...");
    sleep_ms(1000);
  }
}

