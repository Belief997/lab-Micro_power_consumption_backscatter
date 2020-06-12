################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/KinetisSDK/examples/frdmkl03z/user_apps/Prj_test16/main.c 

OBJS += \
./sources/main.o 

C_DEPS += \
./sources/main.d 


# Each subdirectory must supply rules for building sources it contributes
sources/main.o: E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/KinetisSDK/examples/frdmkl03z/user_apps/Prj_test16/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MKL03Z16VFG4 -I../../../../../../platform/osa/inc -I../../../../../../platform/utilities/inc -I../../../../../../platform/CMSIS/Include -I../../../../../../platform/devices -I../../../../../../platform/devices/MKL03Z4/include -I../../../../../../platform/devices/MKL03Z4/startup -I../../../../../../platform/hal/inc -I../../../../../../platform/drivers/inc -I../../../../../../platform/system/inc -I../../ -I../../../../../../platform/drivers/src/adc16 -I../../../../../../platform/drivers/src/cmp -I../../../../../../platform/drivers/src/gpio -I../../../../../../platform/drivers/src/i2c -I../../../../../../platform/drivers/src/lpsci -I../../../../../../platform/drivers/src/lptmr -I../../../../../../platform/drivers/src/spi -I../../../../../../platform/drivers/src/tpm -I../../../../../../platform/drivers/src/cop -std=gnu99 -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


