# Filament Dryer/ Heated plate controller
This project is still in development
## Introduction
The problem: remove 3d printer filament moisture

It's a simple task to complete, but in the middle of the development, I found another aplication.
Using a precisely control of a heat source this project can also be converted to a heated plate solder machine, since both use a heat source and a thermistor to measure the temperature.

## Hardware
 - I2C LCD with the HD44780 generic controller
 - ESP32 dev board
 - Rotary encoder EC11
 - 3950 NTC Thermistor with 100k ohms
 - LED's
 - Resistors

*(schematics still not available)* Check the schematics for the resistors values

## References
The code in the lcd component was extracted from the link [I2C in ESP32 || ESP-IDF || LCD 1602](https://controllerstech.com/i2c-in-esp32-esp-idf-lcd-1602/)