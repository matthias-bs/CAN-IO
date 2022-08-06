# CAN-IO
CAN (Controller Area Network) Controlled Analog/Digital Input/Output Module

> ⚠️ Check supply and I/O voltages carefully and build up your circuit accordingly!
> 
> :warning: Over voltage/over current protection for analog and digital inputs/outputs must be implemented as required!
>
> :warning: Implement voltage dividers and filter capacitors for analog inputs as required.



| Program              | MCU | Board                        | Devel.-Env. | Supply Voltage<br>Board/MCU | Analog Inputs | Analog Outputs | Digital Inputs | Digital Outputs |
| -------------------- | --- | ---------------------------- | -------------------------- |--------------------------- | ------------- | -------------- | -------------- | --------------- |
| [ESP32_MCP2515_CAN_IO](src/ESP32_MCP2515_CAN_IO) | ESP32 | generic<br> (e.g. Joy-It SBC-NodeMCU-ESP32) | Arduino | 5V<sup>(1)</sup> / 3.3V | 2<sup>(2)</sup> | 2<sup>(3)</sup> (0...5V)    | 6<sup>(2)</sup> | 2<sup>(2)</sup> |
| [ESP32_IntegratedSJA1000_CAN_IO](src/ESP32_IntegratedSJA1000_CAN_IO) | ESP32 | generic<br> (e.g. Joy-It SBC-NodeMCU-ESP32) | Arduino | 5V<sup>(1)</sup> / 3.3V | 2<sup>(2)</sup> | 2<sup>(3)</sup> (0...5V)    | 6<sup>(2)</sup> | 2<sup>(2)</sup> |
| AT90CAN128_CAN_IO<br>(coming soon)                             | AT90CAN128 | Crumb128-CAN | AVR-GCC | 5V<sup>(1)</sup> / 5V | 4<sup>(4)</sup>| 2<sup>(3)</sup> | 8<sup>(5)</sup> | 8<sup>(5)</sup> |

(1) Supply voltage level and quality have direct impact on the DACs' output signals. For instance, you won't get a 5V analog output from a USB power supply (due to the typical schottky diode between VDDUSB and VDD5V). 

(2) ESP32 allows great flexibility of utilizing I/O pins - the numbers provided here are only for the default configuration. The actual number of I/O pins varies with different boards.

(3) More DAC channels possible with MCP4725 devices with suitable I2C addresses

(4) More ADC channels are available if required

(5) More I/O ports are available if required

## Timing
Currently no effort is made to synchronize the various inputs and outputs -
* CAN_ID_DOUT message reception -> update digital outputs
* CAN_ID_AOUT0/1 message reception -> start DAC for analog output 0/1, respectively
* read analog inputs (sequentially) -> transmit CAN_ID_AIN message
* read digital inputs -> transmit CAN_ID_DIN message

Please take the conversion time (and transmission time, if applicable) of your DACs/ADCs into account.
