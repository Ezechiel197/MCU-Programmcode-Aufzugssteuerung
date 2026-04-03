MCU Elevator Control System (Master)
This repository contains the firmware for the central control unit (Master) of an elevator simulation. 
The system is based on an STM32 microcontroller and coordinates floor requests, motor control, and communication with peripheral devices.

Key Features

Central Control Logic: Real-time processing of floor calls and management of elevator status, including position, direction, and door states.
Modbus-RTU Communication: Implementation of a robust Modbus Master to poll sensors and control actuators via a serial interface.
Simulator Connectivity: A dedicated module for communicating with a PC-based simulation environment via UART for visualization and debugging.
Event-Driven System: Utilizes an event loop for efficient task handling without blocking the processor.
Terminal Interface: Integrated command interpreter for real-time configuration of parameters, such as the maximum number of floors.

Technical Stack

Hardware: STM32 Microcontroller
Language: C (Embedded C)
Protocols: Modbus RTU, UART (RS232/RS485)
Development Environment: STM32CubeIDE / HAL Library.

