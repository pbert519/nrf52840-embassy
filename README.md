# [Adafruit Feather NRF52840 Sense Demo](https://learn.adafruit.com/adafruit-feather-sense)

This project aims to make use of all features of the development board using rust.

A additonal OLED display is connected.

Used features:
- [x]  OLED display by I2C using embedded_graphics
- [x]  PDM microphon is sampled and a fft is caluclated
- [x]  Battery voltage is measured
- [x]  LED D13 blinks periodically red
- [x]  LED 2 can be switched on/off by the onbard button
- [x]  SHT30 measures temperature and humidity
- [x]  BMP280 measures temperature and pressure
- [x]  Neopixel led shows different color gradients
- [x]  The external flash on the board is used as data storage
    - currently only dummy values
- [x]  BLE is enabled and a multiple charateristics can be read 
    - currently only dummy values
- [ ] LSM6DS33 + LIS3MDL
- [ ] APDS9960
- [ ] USB

Currently all features the measurements are only printed on the console. 

However a goal is that all measurements can be read and the LED controled over BLE.

Additonaly and all sensor data should be stored in the external flash.

## BLE Softdevice
Nordic softdevice is a binary provided by Nordic Semicondutor which implements bluetooth low energy.

The softdevice S140 v7.3.0 is required by this project and need be flashed before hand.

Install the softdevice:
- Download SoftDevice S140 from Nordic's website [here](https://www.nordicsemi.com/Products/Development-software/s140/download).  Supported versions are 7.x.x
- Unzip
- Erase the flash with ``probe-rs-cli erase --chip nrf52840``
- Flash the SoftDevice with ``probe-rs-cli download --chip nrf52840 --format hex s140_nrf52_7.X.X_softdevice.hex``







