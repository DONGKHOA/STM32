################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SX_1278/sx_1278.c \
../SX_1278/sx_1278_hw.c 

OBJS += \
./SX_1278/sx_1278.o \
./SX_1278/sx_1278_hw.o 

C_DEPS += \
./SX_1278/sx_1278.d \
./SX_1278/sx_1278_hw.d 


# Each subdirectory must supply rules for building sources it contributes
SX_1278/%.o SX_1278/%.su SX_1278/%.cyclo: ../SX_1278/%.c SX_1278/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/thanh/STM32CubeIDE/workspace_1.12.1/Lora/SX_1278" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-SX_1278

clean-SX_1278:
	-$(RM) ./SX_1278/sx_1278.cyclo ./SX_1278/sx_1278.d ./SX_1278/sx_1278.o ./SX_1278/sx_1278.su ./SX_1278/sx_1278_hw.cyclo ./SX_1278/sx_1278_hw.d ./SX_1278/sx_1278_hw.o ./SX_1278/sx_1278_hw.su

.PHONY: clean-SX_1278

