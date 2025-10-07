################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers_Devices/Src/stm32f407xx.c \
../Drivers_Devices/Src/stm32f407xx_adc.c \
../Drivers_Devices/Src/stm32f407xx_gpio.c \
../Drivers_Devices/Src/stm32f407xx_i2c.c \
../Drivers_Devices/Src/stm32f407xx_spi.c \
../Drivers_Devices/Src/stm32f407xx_timer.c \
../Drivers_Devices/Src/stm32f407xx_usart.c 

OBJS += \
./Drivers_Devices/Src/stm32f407xx.o \
./Drivers_Devices/Src/stm32f407xx_adc.o \
./Drivers_Devices/Src/stm32f407xx_gpio.o \
./Drivers_Devices/Src/stm32f407xx_i2c.o \
./Drivers_Devices/Src/stm32f407xx_spi.o \
./Drivers_Devices/Src/stm32f407xx_timer.o \
./Drivers_Devices/Src/stm32f407xx_usart.o 

C_DEPS += \
./Drivers_Devices/Src/stm32f407xx.d \
./Drivers_Devices/Src/stm32f407xx_adc.d \
./Drivers_Devices/Src/stm32f407xx_gpio.d \
./Drivers_Devices/Src/stm32f407xx_i2c.d \
./Drivers_Devices/Src/stm32f407xx_spi.d \
./Drivers_Devices/Src/stm32f407xx_timer.d \
./Drivers_Devices/Src/stm32f407xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers_Devices/Src/%.o Drivers_Devices/Src/%.su Drivers_Devices/Src/%.cyclo: ../Drivers_Devices/Src/%.c Drivers_Devices/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"E:/Device_Drivers/Custom_drivers/Drivers_Devices/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers_Devices-2f-Src

clean-Drivers_Devices-2f-Src:
	-$(RM) ./Drivers_Devices/Src/stm32f407xx.cyclo ./Drivers_Devices/Src/stm32f407xx.d ./Drivers_Devices/Src/stm32f407xx.o ./Drivers_Devices/Src/stm32f407xx.su ./Drivers_Devices/Src/stm32f407xx_adc.cyclo ./Drivers_Devices/Src/stm32f407xx_adc.d ./Drivers_Devices/Src/stm32f407xx_adc.o ./Drivers_Devices/Src/stm32f407xx_adc.su ./Drivers_Devices/Src/stm32f407xx_gpio.cyclo ./Drivers_Devices/Src/stm32f407xx_gpio.d ./Drivers_Devices/Src/stm32f407xx_gpio.o ./Drivers_Devices/Src/stm32f407xx_gpio.su ./Drivers_Devices/Src/stm32f407xx_i2c.cyclo ./Drivers_Devices/Src/stm32f407xx_i2c.d ./Drivers_Devices/Src/stm32f407xx_i2c.o ./Drivers_Devices/Src/stm32f407xx_i2c.su ./Drivers_Devices/Src/stm32f407xx_spi.cyclo ./Drivers_Devices/Src/stm32f407xx_spi.d ./Drivers_Devices/Src/stm32f407xx_spi.o ./Drivers_Devices/Src/stm32f407xx_spi.su ./Drivers_Devices/Src/stm32f407xx_timer.cyclo ./Drivers_Devices/Src/stm32f407xx_timer.d ./Drivers_Devices/Src/stm32f407xx_timer.o ./Drivers_Devices/Src/stm32f407xx_timer.su ./Drivers_Devices/Src/stm32f407xx_usart.cyclo ./Drivers_Devices/Src/stm32f407xx_usart.d ./Drivers_Devices/Src/stm32f407xx_usart.o ./Drivers_Devices/Src/stm32f407xx_usart.su

.PHONY: clean-Drivers_Devices-2f-Src

