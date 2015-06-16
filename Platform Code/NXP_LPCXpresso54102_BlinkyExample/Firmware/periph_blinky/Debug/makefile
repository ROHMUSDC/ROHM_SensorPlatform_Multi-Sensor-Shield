################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include example/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: periph_blinky.axf

# Tool invocations
periph_blinky.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\kbahar\Documents\LPCXpresso_7.8.0_426\workspace\lpc_chip_5410x\Debug" -L"C:\Users\kbahar\Documents\LPCXpresso_7.8.0_426\workspace\lpc_board_lpcxpresso_54102\Debug" -L"C:\Users\kbahar\Documents\LPCXpresso_7.8.0_426\workspace\lpc_chip_5410x\libs" -Xlinker -Map="periph_blinky.map" -Xlinker --gc-sections -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -T "periph_blinky_Debug.ld" -o "periph_blinky.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) periph_blinky.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "periph_blinky.axf"; # arm-none-eabi-objcopy -v -O binary "periph_blinky.axf" "periph_blinky.bin" ; # checksum -p LPC54102J512 -d "periph_blinky.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
