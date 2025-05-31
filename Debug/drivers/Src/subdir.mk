################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f302x8_gpio_drivers.c \
../drivers/Src/stm32f302x8_i2c_drivers.c \
../drivers/Src/stm32f302x8_spi_drivers.c 

OBJS += \
./drivers/Src/stm32f302x8_gpio_drivers.o \
./drivers/Src/stm32f302x8_i2c_drivers.o \
./drivers/Src/stm32f302x8_spi_drivers.o 

C_DEPS += \
./drivers/Src/stm32f302x8_gpio_drivers.d \
./drivers/Src/stm32f302x8_i2c_drivers.d \
./drivers/Src/stm32f302x8_spi_drivers.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F302R8Tx -DSTM32F3 -DNUCLEO_F302R8 -c -I../Inc -I"C:/Users/Hyunbin/STM32CubeIDE/workspace_1.16.1/target_3/stm32f302x8_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f302x8_gpio_drivers.cyclo ./drivers/Src/stm32f302x8_gpio_drivers.d ./drivers/Src/stm32f302x8_gpio_drivers.o ./drivers/Src/stm32f302x8_gpio_drivers.su ./drivers/Src/stm32f302x8_i2c_drivers.cyclo ./drivers/Src/stm32f302x8_i2c_drivers.d ./drivers/Src/stm32f302x8_i2c_drivers.o ./drivers/Src/stm32f302x8_i2c_drivers.su ./drivers/Src/stm32f302x8_spi_drivers.cyclo ./drivers/Src/stm32f302x8_spi_drivers.d ./drivers/Src/stm32f302x8_spi_drivers.o ./drivers/Src/stm32f302x8_spi_drivers.su

.PHONY: clean-drivers-2f-Src

