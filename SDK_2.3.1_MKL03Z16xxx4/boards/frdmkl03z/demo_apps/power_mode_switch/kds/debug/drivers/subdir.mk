################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_clock.c \
E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_common.c \
E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_flash.c \
E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_gpio.c \
E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_llwu.c \
E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_lptmr.c \
E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_lpuart.c \
E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_pmc.c \
E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_rcm.c \
E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_smc.c 

OBJS += \
./drivers/fsl_clock.o \
./drivers/fsl_common.o \
./drivers/fsl_flash.o \
./drivers/fsl_gpio.o \
./drivers/fsl_llwu.o \
./drivers/fsl_lptmr.o \
./drivers/fsl_lpuart.o \
./drivers/fsl_pmc.o \
./drivers/fsl_rcm.o \
./drivers/fsl_smc.o 

C_DEPS += \
./drivers/fsl_clock.d \
./drivers/fsl_common.d \
./drivers/fsl_flash.d \
./drivers/fsl_gpio.d \
./drivers/fsl_llwu.d \
./drivers/fsl_lptmr.d \
./drivers/fsl_lpuart.d \
./drivers/fsl_pmc.d \
./drivers/fsl_rcm.d \
./drivers/fsl_smc.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/fsl_clock.o: E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_clock.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../.. -I../../../.. -I../../../../../../devices/MKL03Z4/drivers -I../../../../../../devices/MKL03Z4 -I../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_common.o: E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_common.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../.. -I../../../.. -I../../../../../../devices/MKL03Z4/drivers -I../../../../../../devices/MKL03Z4 -I../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_flash.o: E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_flash.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../.. -I../../../.. -I../../../../../../devices/MKL03Z4/drivers -I../../../../../../devices/MKL03Z4 -I../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_gpio.o: E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_gpio.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../.. -I../../../.. -I../../../../../../devices/MKL03Z4/drivers -I../../../../../../devices/MKL03Z4 -I../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_llwu.o: E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_llwu.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../.. -I../../../.. -I../../../../../../devices/MKL03Z4/drivers -I../../../../../../devices/MKL03Z4 -I../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_lptmr.o: E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_lptmr.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../.. -I../../../.. -I../../../../../../devices/MKL03Z4/drivers -I../../../../../../devices/MKL03Z4 -I../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_lpuart.o: E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_lpuart.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../.. -I../../../.. -I../../../../../../devices/MKL03Z4/drivers -I../../../../../../devices/MKL03Z4 -I../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_pmc.o: E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_pmc.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../.. -I../../../.. -I../../../../../../devices/MKL03Z4/drivers -I../../../../../../devices/MKL03Z4 -I../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_rcm.o: E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_rcm.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../.. -I../../../.. -I../../../../../../devices/MKL03Z4/drivers -I../../../../../../devices/MKL03Z4 -I../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

drivers/fsl_smc.o: E:/Project/IAR/MPC_backscatter/SDK_2.3.1_MKL03Z16xxx4/devices/MKL03Z4/drivers/fsl_smc.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -ffreestanding -fno-builtin -Wall  -g -DDEBUG -DCPU_MKL03Z32VFK4 -DFRDM_KL03Z -DFREEDOM -I../../../../../../CMSIS/Include -I../../../../../../devices -I../.. -I../../../.. -I../../../../../../devices/MKL03Z4/drivers -I../../../../../../devices/MKL03Z4 -I../../../../../../devices/MKL03Z4/utilities/io -I../../../../../../devices/MKL03Z4/utilities/str -I../../../../../../devices/MKL03Z4/utilities/log -I../../../../../../devices/MKL03Z4/utilities -std=gnu99 -mapcs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


