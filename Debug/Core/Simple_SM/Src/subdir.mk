################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Simple_SM/Src/Simple_System_Manager.cpp 

OBJS += \
./Core/Simple_SM/Src/Simple_System_Manager.o 

CPP_DEPS += \
./Core/Simple_SM/Src/Simple_System_Manager.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Simple_SM/Src/%.o Core/Simple_SM/Src/%.su: ../Core/Simple_SM/Src/%.cpp Core/Simple_SM/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Simple_SM-2f-Src

clean-Core-2f-Simple_SM-2f-Src:
	-$(RM) ./Core/Simple_SM/Src/Simple_System_Manager.d ./Core/Simple_SM/Src/Simple_System_Manager.o ./Core/Simple_SM/Src/Simple_System_Manager.su

.PHONY: clean-Core-2f-Simple_SM-2f-Src

