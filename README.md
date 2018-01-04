# AVR Poolmon

This application is targeted for the AVR ATtiny84A, as part of the [esp32-poolmon](https://github.com/DavidAntliff/esp32-poolmon) application.

It monitors four physical switches on PA0, PA1, PA2 and PA3, and uses the state of these switches to determine the output of two Solid State Relays (SSRs).

SSR1 drives the Circulation Pump (CP), and SSR2 drives the Purge Pump (PP).

Switch 1 (labeled "CP Mode") is used to select between Auto and Manual modes for the Circulation Pump.

Switch 2 (labeled "CP Man") is used to select between Circulation Pump On or Off, only when switch 1 is in Manual mode.

Switch 3 (labeled "PP Mode") is used to select between Auto and Manual modes for the Purge Pump.

Switch 4 (labeled "CP Man") is used to select between Purge Pump On or Off, only when switch 3 is in Manual mode.

For each pump, in Auto mode the ESP32 can send commands (via I2C register write) to control the SSR1 state. In Manual mode, only the corresponding "Man" switch can control the associated pump. This provides a way for manual control of either or both pumps regardless of the ESP32 state.

In addition, an I2C register write can be used to enable or disable an audible buzzer (as an alarm).

Status reporting is available via I2C register read operation.

## Registers

See [registers.h](https://github.com/DavidAntliff/avr-poolmon/blob/master/registers.h) for detailed description of the supported I2C registers.

Due to limitations with the I2C Slave library, register access must be performed in the following manner, using SMBus Send/Receive Byte protocols:

### To set the value of a register:

Send two bytes - the register address followed by the register value.

### To read the value of a register:

Send one byte - the register address, then receive a single byte.

SMBus read/write bus/word protocols are not supported.

## License

[MIT](https://github.com/DavidAntliff/avr-poolmon/blob/master/LICENSE)

## Acknowledgements

Incorporates [eriksl/usitwislave](https://github.com/eriksl/usitwislave.git).


