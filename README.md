# CAN-IO
CAN (Controller Area Network) Controlled Analog/Digital Input/Output Module

> :warning: Over voltage/over current protection for analog and digital inputs/outputs must be implemented as required!

> :warning: Implement voltage dividers and filter capacitors for analog inputs as required.

| Program              | MCU | Board                        | Supply Voltage            | Analog Inputs | Analog Outputs | Digital Inputs | Digital Outputs |
| -------------------- | --- | ---------------------------- | ------------------------- | ------------- | -------------- | -------------- | --------------- |
| [ESP32_MCP2515_CAN_IO](https://github.com/matthias-bs/CAN-IO/tree/main/src/ESP32_MCP2515_CAN_IO) | ESP32 | generic<br> (e.g. Joy-It SBC-NodeMCU-ESP32) | 5V<sup>(1)</sup> | 2<sup>(2)</sup> | 2<sup>(3)</sup> (0...5V)    | 6<sup>(2)</sup> | 2<sup>(2)</sup> |


(1) Supply voltage level and quality have direct impact on the DACs' output signals

(2) ESP32 allows great flexibility of utilizing I/O pins - the numbers provided here are only for the default configuration. The actual number of I/O pins varies with different boards.

(3) More DAC channels possible with MCP4725 devices with suitable ISC addresses
