################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/Source/WindowFunctions/WindowFunctions.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f64.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f32.c \
../Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f64.c 

OBJS += \
./Drivers/CMSIS/DSP/Source/WindowFunctions/WindowFunctions.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f64.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f32.o \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f64.o 

C_DEPS += \
./Drivers/CMSIS/DSP/Source/WindowFunctions/WindowFunctions.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f64.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f32.d \
./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f64.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/Source/WindowFunctions/%.o Drivers/CMSIS/DSP/Source/WindowFunctions/%.su Drivers/CMSIS/DSP/Source/WindowFunctions/%.cyclo: ../Drivers/CMSIS/DSP/Source/WindowFunctions/%.c Drivers/CMSIS/DSP/Source/WindowFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H562xx -c -I../Core/Inc -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/STM32H5xx_HAL_Driver/Inc -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/CMSIS/Device/ST/STM32H5xx/Include -IC:/Users/nicek/STM32Cube/Repository/STM32Cube_FW_H5_V1.2.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-Source-2f-WindowFunctions

clean-Drivers-2f-CMSIS-2f-DSP-2f-Source-2f-WindowFunctions:
	-$(RM) ./Drivers/CMSIS/DSP/Source/WindowFunctions/WindowFunctions.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/WindowFunctions.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/WindowFunctions.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/WindowFunctions.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_bartlett_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hamming_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hanning_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft116d_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft144d_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft169d_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft196d_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft223d_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft248d_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft90d_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f32.d
	-$(RM) ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_hft95_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3a_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall3b_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4a_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4b_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_nuttall4c_f64.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f32.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f32.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f32.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f32.su ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f64.cyclo ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f64.d ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f64.o ./Drivers/CMSIS/DSP/Source/WindowFunctions/arm_welch_f64.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-Source-2f-WindowFunctions

