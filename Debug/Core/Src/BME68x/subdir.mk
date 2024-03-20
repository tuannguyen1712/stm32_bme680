################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BME68x/bme68x.c \
../Core/Src/BME68x/common.c 

OBJS += \
./Core/Src/BME68x/bme68x.o \
./Core/Src/BME68x/common.o 

C_DEPS += \
./Core/Src/BME68x/bme68x.d \
./Core/Src/BME68x/common.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/BME68x/%.o Core/Src/BME68x/%.su Core/Src/BME68x/%.cyclo: ../Core/Src/BME68x/%.c Core/Src/BME68x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-BME68x

clean-Core-2f-Src-2f-BME68x:
	-$(RM) ./Core/Src/BME68x/bme68x.cyclo ./Core/Src/BME68x/bme68x.d ./Core/Src/BME68x/bme68x.o ./Core/Src/BME68x/bme68x.su ./Core/Src/BME68x/common.cyclo ./Core/Src/BME68x/common.d ./Core/Src/BME68x/common.o ./Core/Src/BME68x/common.su

.PHONY: clean-Core-2f-Src-2f-BME68x

