# CAN-IO
CAN (Controller Area Network) Controlled Analog/Digital Input/Output Module

* Control analog outputs by CAN messages
* Control digital outputs by CAN message
* Monitor analog inputs via CAN message
* Monitor digital inputs via CAN message   


> ⚠️ Check supply and I/O voltages carefully and build up your circuit accordingly!
> 
> :warning: Over voltage/over current protection for analog and digital inputs/outputs must be implemented as required!
>
> :warning: Implement voltage dividers and filter capacitors for analog inputs as required.



| Program              | MCU | Board                        | Devel.-Env. | Supply Voltage<br>Board/MCU | Analog Inputs | Analog Outputs | Digital Inputs | Digital Outputs |
| -------------------- | --- | ---------------------------- | -------------------------- |--------------------------- | ------------- | -------------- | -------------- | --------------- |
| [ESP32_MCP2515_CAN_IO](src/ESP32_MCP2515_CAN_IO) | ESP32 | generic<br> (e.g. Joy-It SBC-NodeMCU-ESP32) | Arduino | 5V<sup>(1)</sup> / 3.3V | 2<sup>(2)</sup> (0...3.3V) | 2<sup>(3)</sup> (external; 0...5V)    | 6<sup>(2)</sup> | 2<sup>(2)</sup> |
| [ESP32_IntegratedSJA1000<br>_CAN_IO](src/ESP32_IntegratedSJA1000_CAN_IO) | ESP32 | generic<br> (e.g. Joy-It SBC-NodeMCU-ESP32) | Arduino | 5V<sup>(1)</sup> / 3.3V | 2<sup>(2)</sup> (0...3.3V) | 2<sup>(3)</sup> (external; 0...5V)    | 6<sup>(2)</sup> | 2<sup>(2)</sup> |
| [ATSAME51_CAN_IO](src/ATSAME51_CAN_IO) | ATSAME51 | [Adafruit Feather M4 CAN Express](https://www.adafruit.com/product/4759) | Arduino | 5V<sup>(1)</sup> / 3.3V<sup>(1)</sup> | 4 (0...3.3V) | 2 (internal; 0..3.3V) /<br> 2<sup>(3)</sup> (external; 0..5V) | 6 | 2 |
| [AT90CAN128_CAN_IO](src/AT90CAN128_CAN_IO)       | AT90CAN128 | [Crumb128-CAN](https://www.chip45.com/products/crumb128-can-5.0_avr_atmega_module_board_at90can128_usb_rs232_can.php?en) | AVR-GCC | 5V<sup>(1)</sup> / 5V | 4<sup>(4)</sup> (0...5V) | 2<sup>(3)</sup> (external; 0...5V) | 8<sup>(5)</sup> | 8<sup>(5)</sup> |

(1) Supply voltage level and quality have direct impact on the DACs' output signals. For instance, you won't get a 5V analog output from a USB power supply (due to the typical schottky diode between VDDUSB and VDD5V). 

(2) ESP32 allows great flexibility of utilizing I/O pins - the numbers provided here are only for the default configuration. The actual number of I/O pins varies with different boards.

(3) More DAC channels possible with MCP4725 devices with suitable I2C addresses

(4) More ADC channels are available if required

(5) More I/O ports are available if required

| Program                                           | MCU        | Board                        | CAN Controller | CAN Transceiver |
| ------------------------------------------------- | ---------- | ---------------------------- | -------------- |-----------------|
| [ESP32_MCP2515_CAN_IO](src/ESP32_MCP2515_CAN_IO)  | ESP32 | generic<br> (e.g. Joy-It SBC-NodeMCU-ESP32) | external (MCP2515)<sup>(1)</sup> | external<sup>(1)</sup> |
| [ESP32_IntegratedSJA1000<br>_CAN_IO](src/ESP32_IntegratedSJA1000_CAN_IO) | ESP32 | generic<br> (e.g. Joy-It SBC-NodeMCU-ESP32) | internal | external | 
| [ATSAME51_CAN_IO](src/ATSAME51_CAN_IO) | ATSAME51 | [Adafruit Feather M4 CAN Express](https://www.adafruit.com/product/4759) | internal | Adafruit Feather M4 CAN Express: on board |
| [AT90CAN128_CAN_IO](src/AT90CAN128_CAN_IO)        | AT90CAN128 | [Crumb128-CAN](https://www.chip45.com/products/crumb128-can-5.0_avr_atmega_module_board_at90can128_usb_rs232_can.php?en) | internal | Crumb128-CAN: on board |

(1) Tested with [AZDelivery MCP2515 CAN Bus Module](https://www.az-delivery.de/en/products/mcp2515-can-bus-modul)

## Timing
Currently no effort is made to synchronize the various inputs and outputs -
* CAN_ID_DOUT message reception -> update digital outputs
* CAN_ID_AOUT0/1 message reception -> start DAC for analog output 0/1, respectively
* read analog inputs (sequentially) -> transmit CAN_ID_AIN message
* read digital inputs -> transmit CAN_ID_DIN message

Please take the conversion time (and transmission time, if applicable) of your DACs/ADCs into account.

## AT90CAN128 / Crumb128-CAN

* Configure fuses on a fresh MCU - see [AVR Fuse Calculator](http://eleccelerator.com/fusecalc/fusecalc.php?chip=at90can128)
    * Switch off "Divide clock by 8 internally":
    * Switch to external clock oscillator >= 8 MHz (here: with a very conservative startup time...)

    `avrdude -c <programmer> -P <port> -p at90can128 -U lfuse:w:0xFF:m`

* The SW is configured for a 16 MHz crystal
* Set J6 "USBpowered" if desired
* Set J8 "HighSpeed CAN" if you don't have other requirements
