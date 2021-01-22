# Add inputs and outputs from these tool invocations to the build variables 
CC = /home/neko/gcc-arm-none-eabi-8/bin/arm-none-eabi-gcc
CFLAGS  = -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp --specs=nano.specs -mfloat-abi=soft -mthumb

# Each subdirectory must supply rules for building sources it contributes
startup_stm32f446retx.o: startup_stm32f446retx.s
	$(CC) $(CFLAGS) "$<" -o "$@"