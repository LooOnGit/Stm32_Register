################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Custom_Driver/Src/clock.c 

OBJS += \
./Drivers/Custom_Driver/Src/clock.o 

C_DEPS += \
./Drivers/Custom_Driver/Src/clock.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Custom_Driver/Src/clock.o: ../Drivers/Custom_Driver/Src/clock.c Drivers/Custom_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/HOC_ARM/Thanh_ghi/STM32_workspace/RCC/Drivers/Custom_Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Custom_Driver/Src/clock.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

