################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/SDK_2.3.1_MKL03Z32xxx4/boards/frdmkl03z/driver_examples/i2c/interrupt_b2b_transfer/master/i2c_interrupt_b2b_transfer_master.c 

OBJS += \
./source/i2c_interrupt_b2b_transfer_master.o 

C_DEPS += \
./source/i2c_interrupt_b2b_transfer_master.d 


# Each subdirectory must supply rules for building sources it contributes
source/i2c_interrupt_b2b_transfer_master.o: E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/SDK_2.3.1_MKL03Z32xxx4/boards/frdmkl03z/driver_examples/i2c/interrupt_b2b_transfer/master/i2c_interrupt_b2b_transfer_master.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../../../CMSIS/Include -I../../../../../../../../devices -I../.. -I../../../../../.. -I../../../../../../../../devices/MKL03Z4/drivers -I../../../../../../../../devices/MKL03Z4 -I../../../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

