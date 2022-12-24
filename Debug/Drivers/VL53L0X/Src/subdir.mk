################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L0X/Src/VL53L0X.c 

OBJS += \
./Drivers/VL53L0X/Src/VL53L0X.o 

C_DEPS += \
./Drivers/VL53L0X/Src/VL53L0X.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L0X/Src/VL53L0X.o: ../Drivers/VL53L0X/Src/VL53L0X.c Drivers/VL53L0X/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I"C:/Users/amine/STM32CubeIDE/New folder/TOF_3/Drivers/VL53L0X/Inc" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L0X-2f-Src

clean-Drivers-2f-VL53L0X-2f-Src:
	-$(RM) ./Drivers/VL53L0X/Src/VL53L0X.d ./Drivers/VL53L0X/Src/VL53L0X.o ./Drivers/VL53L0X/Src/VL53L0X.su

.PHONY: clean-Drivers-2f-VL53L0X-2f-Src

