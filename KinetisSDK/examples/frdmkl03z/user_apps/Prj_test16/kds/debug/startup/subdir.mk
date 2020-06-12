################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/KinetisSDK/platform/devices/startup.c \
E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/KinetisSDK/platform/devices/MKL03Z4/startup/system_MKL03Z4.c 

S_UPPER_SRCS += \
E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/KinetisSDK/platform/devices/MKL03Z4/startup/gcc/startup_MKL03Z4.S 

OBJS += \
./startup/startup.o \
./startup/startup_MKL03Z4.o \
./startup/system_MKL03Z4.o 

C_DEPS += \
./startup/startup.d \
./startup/system_MKL03Z4.d 

S_UPPER_DEPS += \
./startup/startup_MKL03Z4.d 


# Each subdirectory must supply rules for building sources it contributes
startup/startup.o: E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/KinetisSDK/platform/devices/startup.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MKL03Z16VFG4 -I../../../../../../platform/osa/inc -I../../../../../../platform/utilities/inc -I../../../../../../platform/CMSIS/Include -I../../../../../../platform/devices -I../../../../../../platform/devices/MKL03Z4/include -I../../../../../../platform/devices/MKL03Z4/startup -I../../../../../../platform/hal/inc -I../../../../../../platform/drivers/inc -I../../../../../../platform/system/inc -I../../ -I../../../../../../platform/drivers/src/adc16 -I../../../../../../platform/drivers/src/cmp -I../../../../../../platform/drivers/src/gpio -I../../../../../../platform/drivers/src/i2c -I../../../../../../platform/drivers/src/lpsci -I../../../../../../platform/drivers/src/lptmr -I../../../../../../platform/drivers/src/spi -I../../../../../../platform/drivers/src/tpm -I../../../../../../platform/drivers/src/cop -std=gnu99 -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup/startup_MKL03Z4.o: E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/KinetisSDK/platform/devices/MKL03Z4/startup/gcc/startup_MKL03Z4.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -x assembler-with-cpp -DDEBUG -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup/system_MKL03Z4.o: E:/Project/IAR/MPC_backscatter/lab-Micro_power_consumption_backscatter/KinetisSDK/platform/devices/MKL03Z4/startup/system_MKL03Z4.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g -DDEBUG -DCPU_MKL03Z16VFG4 -I../../../../../../platform/osa/inc -I../../../../../../platform/utilities/inc -I../../../../../../platform/CMSIS/Include -I../../../../../../platform/devices -I../../../../../../platform/devices/MKL03Z4/include -I../../../../../../platform/devices/MKL03Z4/startup -I../../../../../../platform/hal/inc -I../../../../../../platform/drivers/inc -I../../../../../../platform/system/inc -I../../ -I../../../../../../platform/drivers/src/adc16 -I../../../../../../platform/drivers/src/cmp -I../../../../../../platform/drivers/src/gpio -I../../../../../../platform/drivers/src/i2c -I../../../../../../platform/drivers/src/lpsci -I../../../../../../platform/drivers/src/lptmr -I../../../../../../platform/drivers/src/spi -I../../../../../../platform/drivers/src/tpm -I../../../../../../platform/drivers/src/cop -std=gnu99 -fno-common  -ffreestanding  -fno-builtin  -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


