# Add inputs and outputs from these tool invocations to the build variables 
all: \
stm32f446xx_gpio_driver.o \
stm32f446xx_i2c_driver.o \
stm32f446xx_rcc_driver.o \
stm32f446xx_spi_driver.o \
stm32f446xx_usart_driver.o \

PATH = /home/neko/Projects/STM32F446RE_Phase_1/Stm32f446xx_driver/drivers/Inc
CC = /home/neko/gcc-arm-none-eabi-8/bin/arm-none-eabi-gcc

CFLAGS  = -mcpu=cortex-m4 -std=gnu11 -g3 -c -I$(PATH) -O0 
CFLAGS += -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP
CFLAGS += --specs=nano.specs -mfloat-abi=soft -mthumb

# Each subdirectory must supply rules for building sources it contributes
stm32f446xx_gpio_driver.o: stm32f446xx_gpio_driver.c
	${CC} "$<" $(CFLAGS) -MT"$@" -o "$@"
stm32f446xx_i2c_driver.o: stm32f446xx_i2c_driver.c
	$(CC) "$<" $(CFLAGS) -MT"$@" -o "$@"
stm32f446xx_rcc_driver.o: stm32f446xx_rcc_driver.c
	$(CC) "$<" $(CFLAGS) -MT"$@" -o "$@"
stm32f446xx_spi_driver.o: stm32f446xx_spi_driver.c
	$(CC) "$<" $(CFLAGS) -MT"$@" -o "$@"
stm32f446xx_usart_driver.o: stm32f446xx_usart_driver.c
	$(CC) "$<" $(CFLAGS) -MT"$@" -o "$@"
