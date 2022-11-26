# Driver_Development
 Device drivers implementation for STM32F407x-based processors.

* The project is developed in STM32Cube IDE.
* The code and examples are developed for the STM32F407G-DISC1 board.
* Drivers are developed for GPIO, SPI, I2C, and USART peripherals.


# Description
General description of developed source and header files

* Drivers-related header and source files are located in the 'drivers' folder.
  <br>  ` stm32f407xx_drivers > drivers` </br>
* The 'drivers' folder contains two sub-folders: *Inc* and *Src*.
* The *Inc* folder contains the following:
    1. `stm32f407xx.h` (Device-specific header file)
    2. `stm32f4o7xx_gpio_drivers.h` (for GPIO-specific APIs and structures)
    3. `stm32f4o7xx_spi_drivers.h` (for SPI-specific APIs and structures)
    4. `stm32f4o7xx_i2c_drivers.h` (for I2C-specific APIs and structures)
* The *Src* folder contains the following:
    1. `stm32f4o7xx_gpio_drivers.c` (GPIP drivers implementation)
    2. `stm32f4o7xx_spi_drivers.c` (SPI drivers implementation)
    3. `stm32f4o7xx_i2c_drivers.c` (I2C drivers implementation)

* Example applications for each peripheral are located in the 'Src' folder
<br>  ` stm32f407xx_drivers > Src` </br>
