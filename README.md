# Modbus RTU Slave library for Raspberry Pi Pico (RP2040) based on the C++ SDK
This repository contain **working** example Modbus RTU Slave implementation for rp2040 (Raspberry Pi Pico) with C++ SDK

## Features:
- Modbus RTU Slave
- **RS-485** support
- C++ style class 
- Registers layout
- Separate setup read-only and read-write registers
- UART 0 and 1 support
- it works out of box and tested


## How to use the example:
- Tune VS Code for Raspberry Pico development ([instruction](https://www.youtube.com/watch?v=B5rQSoOmR5w))
- You can use 2nd Raspberry Pico as a bootloader and debuger ([instruction](https://www.youtube.com/watch?v=jnC5LrTx470))
- Modify launch.json and setting.json files in .vscode according to your environment
- Start the debugging session 
- Use Modbus Master emulator for connection (ModBus Poll for example)
- Change UART parameters in main.cpp and registers layout in modbus.cpp for your purpose
- Change stored data inside Modbus Manager class
- Change Switch-case optins according to your data and registers in *mb_read_holding_register()* and *mb_write_single_register()*

## TODO:
- incapsulation for class data
- register layout definition throw init
- register checking throw std::dict instead of switch-case
- move on_mb_rx() to class logic
- mb_write_multiple_registers()
- mb_read_input_registers()
- 

