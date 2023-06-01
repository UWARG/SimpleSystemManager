################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Los_Driver/SBUS/Src/LOS_D_SBUSReceiver.cpp \
../Core/Los_Driver/SBUS/Src/LOS_D_SBUSSender.cpp 

OBJS += \
./Core/Los_Driver/SBUS/Src/LOS_D_SBUSReceiver.o \
./Core/Los_Driver/SBUS/Src/LOS_D_SBUSSender.o 

CPP_DEPS += \
./Core/Los_Driver/SBUS/Src/LOS_D_SBUSReceiver.d \
./Core/Los_Driver/SBUS/Src/LOS_D_SBUSSender.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Los_Driver/SBUS/Src/%.o Core/Los_Driver/SBUS/Src/%.su: ../Core/Los_Driver/SBUS/Src/%.cpp Core/Los_Driver/SBUS/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Los_Driver-2f-SBUS-2f-Src

clean-Core-2f-Los_Driver-2f-SBUS-2f-Src:
	-$(RM) ./Core/Los_Driver/SBUS/Src/LOS_D_SBUSReceiver.d ./Core/Los_Driver/SBUS/Src/LOS_D_SBUSReceiver.o ./Core/Los_Driver/SBUS/Src/LOS_D_SBUSReceiver.su ./Core/Los_Driver/SBUS/Src/LOS_D_SBUSSender.d ./Core/Los_Driver/SBUS/Src/LOS_D_SBUSSender.o ./Core/Los_Driver/SBUS/Src/LOS_D_SBUSSender.su

.PHONY: clean-Core-2f-Los_Driver-2f-SBUS-2f-Src

