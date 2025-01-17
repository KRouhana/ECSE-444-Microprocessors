################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/freertos.c \
../Core/Src/hts221.c \
../Core/Src/lis3mdl.c \
../Core/Src/lps22hb.c \
../Core/Src/lsm6dsl.c \
../Core/Src/main.c \
../Core/Src/stm32l4s5i_iot01.c \
../Core/Src/stm32l4s5i_iot01_accelero.c \
../Core/Src/stm32l4s5i_iot01_gyro.c \
../Core/Src/stm32l4s5i_iot01_hsensor.c \
../Core/Src/stm32l4s5i_iot01_magneto.c \
../Core/Src/stm32l4s5i_iot01_psensor.c \
../Core/Src/stm32l4s5i_iot01_qspi.c \
../Core/Src/stm32l4s5i_iot01_tsensor.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_hal_timebase_tim.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/freertos.o \
./Core/Src/hts221.o \
./Core/Src/lis3mdl.o \
./Core/Src/lps22hb.o \
./Core/Src/lsm6dsl.o \
./Core/Src/main.o \
./Core/Src/stm32l4s5i_iot01.o \
./Core/Src/stm32l4s5i_iot01_accelero.o \
./Core/Src/stm32l4s5i_iot01_gyro.o \
./Core/Src/stm32l4s5i_iot01_hsensor.o \
./Core/Src/stm32l4s5i_iot01_magneto.o \
./Core/Src/stm32l4s5i_iot01_psensor.o \
./Core/Src/stm32l4s5i_iot01_qspi.o \
./Core/Src/stm32l4s5i_iot01_tsensor.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_hal_timebase_tim.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/freertos.d \
./Core/Src/hts221.d \
./Core/Src/lis3mdl.d \
./Core/Src/lps22hb.d \
./Core/Src/lsm6dsl.d \
./Core/Src/main.d \
./Core/Src/stm32l4s5i_iot01.d \
./Core/Src/stm32l4s5i_iot01_accelero.d \
./Core/Src/stm32l4s5i_iot01_gyro.d \
./Core/Src/stm32l4s5i_iot01_hsensor.d \
./Core/Src/stm32l4s5i_iot01_magneto.d \
./Core/Src/stm32l4s5i_iot01_psensor.d \
./Core/Src/stm32l4s5i_iot01_qspi.d \
./Core/Src/stm32l4s5i_iot01_tsensor.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_hal_timebase_tim.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I"/Users/karl/STM32CubeIDE/ECSE-444/Lab 4/Drivers/BSP/" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/hts221.cyclo ./Core/Src/hts221.d ./Core/Src/hts221.o ./Core/Src/hts221.su ./Core/Src/lis3mdl.cyclo ./Core/Src/lis3mdl.d ./Core/Src/lis3mdl.o ./Core/Src/lis3mdl.su ./Core/Src/lps22hb.cyclo ./Core/Src/lps22hb.d ./Core/Src/lps22hb.o ./Core/Src/lps22hb.su ./Core/Src/lsm6dsl.cyclo ./Core/Src/lsm6dsl.d ./Core/Src/lsm6dsl.o ./Core/Src/lsm6dsl.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32l4s5i_iot01.cyclo ./Core/Src/stm32l4s5i_iot01.d ./Core/Src/stm32l4s5i_iot01.o ./Core/Src/stm32l4s5i_iot01.su ./Core/Src/stm32l4s5i_iot01_accelero.cyclo ./Core/Src/stm32l4s5i_iot01_accelero.d ./Core/Src/stm32l4s5i_iot01_accelero.o ./Core/Src/stm32l4s5i_iot01_accelero.su ./Core/Src/stm32l4s5i_iot01_gyro.cyclo ./Core/Src/stm32l4s5i_iot01_gyro.d ./Core/Src/stm32l4s5i_iot01_gyro.o ./Core/Src/stm32l4s5i_iot01_gyro.su ./Core/Src/stm32l4s5i_iot01_hsensor.cyclo ./Core/Src/stm32l4s5i_iot01_hsensor.d ./Core/Src/stm32l4s5i_iot01_hsensor.o ./Core/Src/stm32l4s5i_iot01_hsensor.su ./Core/Src/stm32l4s5i_iot01_magneto.cyclo ./Core/Src/stm32l4s5i_iot01_magneto.d ./Core/Src/stm32l4s5i_iot01_magneto.o ./Core/Src/stm32l4s5i_iot01_magneto.su ./Core/Src/stm32l4s5i_iot01_psensor.cyclo ./Core/Src/stm32l4s5i_iot01_psensor.d ./Core/Src/stm32l4s5i_iot01_psensor.o ./Core/Src/stm32l4s5i_iot01_psensor.su ./Core/Src/stm32l4s5i_iot01_qspi.cyclo ./Core/Src/stm32l4s5i_iot01_qspi.d ./Core/Src/stm32l4s5i_iot01_qspi.o ./Core/Src/stm32l4s5i_iot01_qspi.su ./Core/Src/stm32l4s5i_iot01_tsensor.cyclo ./Core/Src/stm32l4s5i_iot01_tsensor.d ./Core/Src/stm32l4s5i_iot01_tsensor.o ./Core/Src/stm32l4s5i_iot01_tsensor.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_hal_timebase_tim.cyclo ./Core/Src/stm32l4xx_hal_timebase_tim.d ./Core/Src/stm32l4xx_hal_timebase_tim.o ./Core/Src/stm32l4xx_hal_timebase_tim.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su

.PHONY: clean-Core-2f-Src

