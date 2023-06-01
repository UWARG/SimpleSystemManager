################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Los_Driver/Common/Src/CircularBuffer.cpp \
../Core/Los_Driver/Common/Src/callback_organize.cpp \
../Core/Los_Driver/Common/Src/driver_config.cpp 

OBJS += \
./Core/Los_Driver/Common/Src/CircularBuffer.o \
./Core/Los_Driver/Common/Src/callback_organize.o \
./Core/Los_Driver/Common/Src/driver_config.o 

CPP_DEPS += \
./Core/Los_Driver/Common/Src/CircularBuffer.d \
./Core/Los_Driver/Common/Src/callback_organize.d \
./Core/Los_Driver/Common/Src/driver_config.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Los_Driver/Common/Src/%.o Core/Los_Driver/Common/Src/%.su: ../Core/Los_Driver/Common/Src/%.cpp Core/Los_Driver/Common/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Los_Driver-2f-Common-2f-Src

clean-Core-2f-Los_Driver-2f-Common-2f-Src:
	-$(RM) ./Core/Los_Driver/Common/Src/CircularBuffer.d ./Core/Los_Driver/Common/Src/CircularBuffer.o ./Core/Los_Driver/Common/Src/CircularBuffer.su ./Core/Los_Driver/Common/Src/callback_organize.d ./Core/Los_Driver/Common/Src/callback_organize.o ./Core/Los_Driver/Common/Src/callback_organize.su ./Core/Los_Driver/Common/Src/driver_config.d ./Core/Los_Driver/Common/Src/driver_config.o ./Core/Los_Driver/Common/Src/driver_config.su

.PHONY: clean-Core-2f-Los_Driver-2f-Common-2f-Src

