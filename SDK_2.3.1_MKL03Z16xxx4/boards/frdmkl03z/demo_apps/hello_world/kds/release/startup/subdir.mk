################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/system_MKL03Z4.c 

S_UPPER_SRCS += \
E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/gcc/startup_MKL03Z4.S 

OBJS += \
./startup/startup_MKL03Z4.o \
./startup/system_MKL03Z4.o 

C_DEPS += \
./startup/system_MKL03Z4.d 

S_UPPER_DEPS += \
./startup/startup_MKL03Z4.d 


# Each subdirectory must supply rules for building sources it contributes
startup/startup_MKL03Z4.o: E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/gcc/startup_MKL03Z4.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall -x assembler-with-cpp -D__STARTUP_CLEAR_BSS -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup/system_MKL03Z4.o: E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/system_MKL03Z4.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall -DNDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -I../../../../../../CMSIS/Include -I../../../../../../devices -I../.. -I../../../.. -I../../../../../../devices/MKL03Z4/drivers -I../../../../../../devices/MKL03Z4 -I../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


