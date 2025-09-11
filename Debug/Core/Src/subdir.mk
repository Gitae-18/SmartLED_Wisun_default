################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc_user.c \
../Core/Src/cbor_format.c \
../Core/Src/fft.c \
../Core/Src/main.c \
../Core/Src/memory.c \
../Core/Src/node_main.c \
../Core/Src/stm32h5xx_hal_msp.c \
../Core/Src/stm32h5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h5xx.c \
../Core/Src/wisun_frame.c \
../Core/Src/wisun_transport.c 

OBJS += \
./Core/Src/adc_user.o \
./Core/Src/cbor_format.o \
./Core/Src/fft.o \
./Core/Src/main.o \
./Core/Src/memory.o \
./Core/Src/node_main.o \
./Core/Src/stm32h5xx_hal_msp.o \
./Core/Src/stm32h5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h5xx.o \
./Core/Src/wisun_frame.o \
./Core/Src/wisun_transport.o 

C_DEPS += \
./Core/Src/adc_user.d \
./Core/Src/cbor_format.d \
./Core/Src/fft.d \
./Core/Src/main.d \
./Core/Src/memory.d \
./Core/Src/node_main.d \
./Core/Src/stm32h5xx_hal_msp.d \
./Core/Src/stm32h5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h5xx.d \
./Core/Src/wisun_frame.d \
./Core/Src/wisun_transport.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H562xx -c -I../Core/Inc -IC:/Users/nicek/Desktop/CMSIS-DSP/Include -I"C:/Users/nicek/Desktop/CMSIS-DSP/Include" -I"C:/Users/nicek/Desktop/CMSIS-DSP/Include/CMSIS/Core/Include" -I"C:/Users/nicek/Desktop/CMSIS-DSP/Include/dsp" -IC:/Users/nicek/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-ALGOBUILD/1.4.0/Middlewares/Third_Party/ARM/DSP/Inc -ID:/Smart_OnDevice/SmartDevice/smart_device/X-CUBE-AI/App -I"D:/Smart_OnDevice/SmartDevice/smart_device/Middlewares/ST/AI/Inc" -I"D:/Smart_OnDevice/SmartDevice/smart_device/X-CUBE-AI/Target" -I"D:/Smart_OnDevice/SmartDevice/smart_device/X-CUBE-AI" -I"D:/Smart_OnDevice/SmartDevice/smart_device/X-CUBE-AI" -I"D:/Smart_OnDevice/SmartDevice/smart_device/X-CUBE-AI/Target" -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Inc -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/CMSIS/Device/ST/STM32H5xx/Include -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/CMSIS/Include -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/STM32H5xx_HAL_Driver/Inc -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/CMSIS/Device/ST/STM32H5xx/Include -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/fft.o: ../Core/Src/fft.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H562xx -c -I../Core/Inc -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/STM32H5xx_HAL_Driver/Inc -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/CMSIS/Device/ST/STM32H5xx/Include -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/CMSIS/Include -I"C:/Users/nicek/Desktop/CMSIS-DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/main.o: ../Core/Src/main.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H562xx -c -I../Core/Inc -IC:/Users/nicek/Desktop/CMSIS-DSP/Include -I"C:/Users/nicek/Desktop/CMSIS-DSP/Include" -I"C:/Users/nicek/Desktop/CMSIS-DSP/Include/CMSIS/Core/Include" -I"C:/Users/nicek/Desktop/CMSIS-DSP/Include/dsp" -IC:/Users/nicek/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-ALGOBUILD/1.4.0/Middlewares/Third_Party/ARM/DSP/Inc -ID:/Smart_OnDevice/SmartDevice/smart_device/X-CUBE-AI/App -I"D:/Smart_OnDevice/SmartDevice/smart_device/Middlewares/ST/AI/Inc" -I"D:/Smart_OnDevice/SmartDevice/smart_device/X-CUBE-AI/Target" -I"D:/Smart_OnDevice/SmartDevice/smart_device/X-CUBE-AI" -I"D:/Smart_OnDevice/SmartDevice/smart_device/X-CUBE-AI" -I"D:/Smart_OnDevice/SmartDevice/smart_device/X-CUBE-AI/Target" -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Inc -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/CMSIS/Device/ST/STM32H5xx/Include -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/CMSIS/Include -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/STM32H5xx_HAL_Driver/Inc -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/CMSIS/Device/ST/STM32H5xx/Include -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc_user.cyclo ./Core/Src/adc_user.d ./Core/Src/adc_user.o ./Core/Src/adc_user.su ./Core/Src/cbor_format.cyclo ./Core/Src/cbor_format.d ./Core/Src/cbor_format.o ./Core/Src/cbor_format.su ./Core/Src/fft.cyclo ./Core/Src/fft.d ./Core/Src/fft.o ./Core/Src/fft.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/memory.cyclo ./Core/Src/memory.d ./Core/Src/memory.o ./Core/Src/memory.su ./Core/Src/node_main.cyclo ./Core/Src/node_main.d ./Core/Src/node_main.o ./Core/Src/node_main.su ./Core/Src/stm32h5xx_hal_msp.cyclo ./Core/Src/stm32h5xx_hal_msp.d ./Core/Src/stm32h5xx_hal_msp.o ./Core/Src/stm32h5xx_hal_msp.su ./Core/Src/stm32h5xx_it.cyclo ./Core/Src/stm32h5xx_it.d ./Core/Src/stm32h5xx_it.o ./Core/Src/stm32h5xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h5xx.cyclo ./Core/Src/system_stm32h5xx.d ./Core/Src/system_stm32h5xx.o ./Core/Src/system_stm32h5xx.su ./Core/Src/wisun_frame.cyclo ./Core/Src/wisun_frame.d ./Core/Src/wisun_frame.o ./Core/Src/wisun_frame.su ./Core/Src/wisun_transport.cyclo ./Core/Src/wisun_transport.d ./Core/Src/wisun_transport.o ./Core/Src/wisun_transport.su

.PHONY: clean-Core-2f-Src

