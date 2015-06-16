################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../example/src/blinky.c \
../example/src/cr_startup_lpc5410x.c \
../example/src/crp.c \
../example/src/sysinit.c 

OBJS += \
./example/src/blinky.o \
./example/src/cr_startup_lpc5410x.o \
./example/src/crp.o \
./example/src/sysinit.o 

C_DEPS += \
./example/src/blinky.d \
./example/src/cr_startup_lpc5410x.d \
./example/src/crp.d \
./example/src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
example/src/%.o: ../example/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__MULTICORE_NONE -DDEBUG -D__CODE_RED -D__USE_LPCOPEN -D__REDLIB__ -DCORE_M4 -I"C:\Users\kbahar\Documents\LPCXpresso_7.8.0_426\workspace\lpc_chip_5410x\inc" -I"C:\Users\kbahar\Documents\LPCXpresso_7.8.0_426\workspace\lpc_board_lpcxpresso_54102\inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fsingle-precision-constant -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


