################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f407xx_gpio_drivers.c \
../drivers/Src/stm32f407xx_i2c_drivers.c \
../drivers/Src/stm32f407xx_spi_drivers.c 

OBJS += \
./drivers/Src/stm32f407xx_gpio_drivers.o \
./drivers/Src/stm32f407xx_i2c_drivers.o \
./drivers/Src/stm32f407xx_spi_drivers.o 

C_DEPS += \
./drivers/Src/stm32f407xx_gpio_drivers.d \
./drivers/Src/stm32f407xx_i2c_drivers.d \
./drivers/Src/stm32f407xx_spi_drivers.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/Resources/Embedded_Systems/3_Device_Drivers/Driver_Development/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f407xx_gpio_drivers.d ./drivers/Src/stm32f407xx_gpio_drivers.o ./drivers/Src/stm32f407xx_gpio_drivers.su ./drivers/Src/stm32f407xx_i2c_drivers.d ./drivers/Src/stm32f407xx_i2c_drivers.o ./drivers/Src/stm32f407xx_i2c_drivers.su ./drivers/Src/stm32f407xx_spi_drivers.d ./drivers/Src/stm32f407xx_spi_drivers.o ./drivers/Src/stm32f407xx_spi_drivers.su

.PHONY: clean-drivers-2f-Src

