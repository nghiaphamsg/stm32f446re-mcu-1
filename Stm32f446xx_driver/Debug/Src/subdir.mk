################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/usartTransmitterToArduinoSlave.c 

OBJS += \
./Src/usartTransmitterToArduinoSlave.o 

C_DEPS += \
./Src/usartTransmitterToArduinoSlave.d 


# Each subdirectory must supply rules for building sources it contributes
Src/usartTransmitterToArduinoSlave.o: ../Src/usartTransmitterToArduinoSlave.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I../Inc -I"C:/Users/AD/Desktop/MCU-1_Course/MCU1/Stm32f446xx_driver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/usartTransmitterToArduinoSlave.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

