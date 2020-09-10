################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/SDK_2.3.1_MKL03Z32xxx4/devices/MKL03Z4/utilities/fsl_assert.c \
E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/SDK_2.3.1_MKL03Z32xxx4/devices/MKL03Z4/utilities/fsl_debug_console.c \
E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/SDK_2.3.1_MKL03Z32xxx4/devices/MKL03Z4/utilities/io/fsl_io.c \
E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/SDK_2.3.1_MKL03Z32xxx4/devices/MKL03Z4/utilities/log/fsl_log.c \
E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/SDK_2.3.1_MKL03Z32xxx4/devices/MKL03Z4/utilities/str/fsl_str.c 

OBJS += \
./utilities/fsl_assert.o \
./utilities/fsl_debug_console.o \
./utilities/fsl_io.o \
./utilities/fsl_log.o \
./utilities/fsl_str.o 

C_DEPS += \
./utilities/fsl_assert.d \
./utilities/fsl_debug_console.d \
./utilities/fsl_io.d \
./utilities/fsl_log.d \
./utilities/fsl_str.d 


# Each subdirectory must supply rules for building sources it contributes
utilities/fsl_assert.o: E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/SDK_2.3.1_MKL03Z32xxx4/devices/MKL03Z4/utilities/fsl_assert.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../../CMSIS/Include -I../../../../../../../devices -I../.. -I../../../../.. -I../../../../../../../devices/MKL03Z4/drivers -I../../../../../../../devices/MKL03Z4 -I../../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

utilities/fsl_debug_console.o: E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/SDK_2.3.1_MKL03Z32xxx4/devices/MKL03Z4/utilities/fsl_debug_console.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../../CMSIS/Include -I../../../../../../../devices -I../.. -I../../../../.. -I../../../../../../../devices/MKL03Z4/drivers -I../../../../../../../devices/MKL03Z4 -I../../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

utilities/fsl_io.o: E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/SDK_2.3.1_MKL03Z32xxx4/devices/MKL03Z4/utilities/io/fsl_io.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../../CMSIS/Include -I../../../../../../../devices -I../.. -I../../../../.. -I../../../../../../../devices/MKL03Z4/drivers -I../../../../../../../devices/MKL03Z4 -I../../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

utilities/fsl_log.o: E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/SDK_2.3.1_MKL03Z32xxx4/devices/MKL03Z4/utilities/log/fsl_log.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../../CMSIS/Include -I../../../../../../../devices -I../.. -I../../../../.. -I../../../../../../../devices/MKL03Z4/drivers -I../../../../../../../devices/MKL03Z4 -I../../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

utilities/fsl_str.o: E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/SDK_2.3.1_MKL03Z32xxx4/devices/MKL03Z4/utilities/str/fsl_str.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../../CMSIS/Include -I../../../../../../../devices -I../.. -I../../../../.. -I../../../../../../../devices/MKL03Z4/drivers -I../../../../../../../devices/MKL03Z4 -I../../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


