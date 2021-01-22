# Add inputs and outputs from these tool invocations to the build variables 
CC = /home/neko/gcc-arm-none-eabi-8/bin/arm-none-eabi-gcc
PATH = /home/neko/Projects/STM32F446RE_Phase_1/Stm32f446xx_driver/drivers/Inc
CFLAGS  = -mcpu=cortex-m4 -std=gnu11 -g3 -c -I$(PATH) -O0 
CFLAGS += -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP

# Each subdirectory must supply rules for building sources it contributes
buttonLED.o: ./buttonLED.c
	$(CC) "$<" $(CFLAGS) -MF"./buttonLED.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

