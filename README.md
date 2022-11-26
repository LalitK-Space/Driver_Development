# Driver_Development
 Device drivers implementation for STM32F407x-based processors.

* The project is developed in STM32Cube IDE.
* The code and examples are developed for the STM32F407G-DISC1 board.
* Drivers are developed for GPIO, SPI, I2C, and USART peripherals.


# Description
General description of developed source and header files

* Drivers-related header and source files are located in the 'drivers' folder.
  <br>  ` STM32F407xx_Drivers > drivers` </br>
* The 'drivers' folder contains two sub-folders: *Inc* and *Src*.
* The *Inc* folder contains the following:
    1. `stm32f407xx.h` (Device-specific header file)
    2. `stm32f4o7xx_gpio_drivers.h` (for GPIO-specific APIs and structures)
* The *Src* folder contains the following:
    1. `stm32f4o7xx_gpio_drivers.c` (GPIP drivers implementation)
