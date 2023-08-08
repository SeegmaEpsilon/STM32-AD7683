# STM32-AD7683

This repository contains a small driver for working with ADCs that use a **non-standard SPI protocol** like the AD7683.

2 ways of working with ADC are shown: using **hardware SPI** and **bit-banging**, which can be implemented on any GPIO.

There is also a working example on STM32F401RE microcontroller (Nucleo Board).

The library used is **HAL**, but you can replace it with any other library you like by replacing the HAL_SPI_TransmitReceive(...) function.
