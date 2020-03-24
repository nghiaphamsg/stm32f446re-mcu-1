################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include drivers/Src/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Stm32f446xx_driver.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Stm32f446xx_driver.list \

OBJCOPY_BIN += \
Stm32f446xx_driver.bin \


# All Target
all: Stm32f446xx_driver.elf secondary-outputs

# Tool invocations
Stm32f446xx_driver.elf: $(OBJS) $(USER_OBJS) C:\Users\AD\Desktop\MCU-1_Course\MCU1\Stm32f446xx_driver\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "Stm32f446xx_driver.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\AD\Desktop\MCU-1_Course\MCU1\Stm32f446xx_driver\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Stm32f446xx_driver.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Stm32f446xx_driver.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Stm32f446xx_driver.list"
	@echo 'Finished building: $@'
	@echo ' '

Stm32f446xx_driver.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Stm32f446xx_driver.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
