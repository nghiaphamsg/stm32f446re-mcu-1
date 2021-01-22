################################################################################
# Automatically-generated file.
################################################################################

# Every subdirectory with source files must be described here
TARGET_OBJ += \
Startup/ \
01_Button_GPIO/ \
../drivers/Src/

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Stm32f446xx_driver.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Stm32f446xx_driver.list \

OBJCOPY_BIN += \
Stm32f446xx_driver.bin \

# Every subdirectory with source files must be described here

# Eviroment 
CC = /home/neko/gcc-arm-none-eabi-8/bin/arm-none-eabi-gcc
CC_SIZE = /home/neko/gcc-arm-none-eabi-8/bin/arm-none-eabi-size
OBJ_DUMP = /home/neko/gcc-arm-none-eabi-8/bin/arm-none-eabi-objdump
OBJ_CP = /home/neko/gcc-arm-none-eabi-8/bin/arm-none-eabi-objcopy

STLINK= /home/neko/Projects/MCU_STM32_Flash/stlink-1.6.1
FLASH_PATH = /home/neko/Projects/STM32F446RE_Phase_1/Stm32f446xx_driver/app/STM32F446RETX_FLASH.ld
CFLAGS = --specs=nosys.specs -Wl,-Map="Stm32f446xx_driver.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group

# All Target
all: Stm32f446xx_driver.elf secondary-outputs

# Tool invocations
Stm32f446xx_driver.elf: $(FLASH_PATH)
	for dir in $(TARGET_OBJ); do \
		cd $$dir; make; \
		cd -; \
	done
	$(CC) -o "Stm32f446xx_driver.elf" @"objects.list" -mcpu=cortex-m4 -T$(FLASH_PATH) $(CFLAGS)
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	$(CC_SIZE)  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Stm32f446xx_driver.list: $(EXECUTABLES)
	$(OBJ_DUMP) -h -S $(EXECUTABLES) > "Stm32f446xx_driver.list"
	@echo 'Finished building: $@'
	@echo ' '

Stm32f446xx_driver.bin: $(EXECUTABLES)
	$(OBJ_CP)  -O binary $(EXECUTABLES) "Stm32f446xx_driver.bin"
	@echo 'Finished building: $@'
	@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

clean:
	for dir in $(TARGET_OBJ); do \
		cd $$dir; \
		rm -f *.o *.d *.su;\
		cd -; \
	done
	@rm -f *.bin *.elf *.map Stm32f446xx_driver.list

burn: 
	$(STLINK)/st-flash write Stm32f446xx_driver.bin 0x8000000