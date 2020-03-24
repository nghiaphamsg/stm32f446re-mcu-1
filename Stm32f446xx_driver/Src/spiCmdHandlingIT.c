/*
 * spiCmdHandlingIT.c
 *
 *  Created on: Mar 11, 2020
 *      Author: AD
 */


#include <string.h>

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"

//extern void initialise_monitor_handles(void);
/*
 * Command codes
 */
#define COMMAND_LED_CTRL					0x50					//CMD_LED_CTRL		<pin no(1)>	<value(1)>
#define COMMAND_SENSOR_READ					0x51					//CMD_SENOSR_READ   <analog pin number(1)>
#define COMMAND_LED_READ      				0x52					//CMD_LED_READ 	 	<pin no(1)>
#define COMMAND_PRINT      					0x53					//CMD_PRINT 		<len(2)>  <message(length)>
#define COMMAND_ID_READ      				0x54					//CMD_ID_READ

#define LED_ON     							1
#define LED_OFF    							0

/*
 * Arduino analog pins
 */
#define ANALOG_PIN0 						0
#define ANALOG_PIN1 						1
#define ANALOG_PIN2 						2
#define ANALOG_PIN3 						3
#define ANALOG_PIN4 						4

/*
 * Arduino led
 */
#define LED_PIN  							9

SPI_Handle_t SPI2Handle;

void delay250ms(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}

uint8_t SPI_VerityRespone(uint8_t ackByteRespone) {
	if (ackByteRespone == (uint8_t)0xF5) {
		return 1;
	}
	return 0;
}

void SPI2_IRQHandler(void) {

	SPI_IRQHandling(&SPI2Handle);
}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GPIOBtn;
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	GPIOBtn.pGPIOx = GPIOC;

	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;


	GPIO_Init(&GPIOBtn);
}

void SPI2_GPIOInits(void) {
	GPIO_Handle_t SPIPins;
	memset(&SPIPins, 0, sizeof(SPIPins));

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode 	= 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;

	SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_13;					//SPI2_SCK
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_15;					//SPI2_MOSI
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_14;					//SPI2_MISO
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_12;					//SPI2_NSS
	GPIO_Init(&SPIPins);
}


void SPI2_Inits(void) {
	SPI_Handle_t SPI2Handle;
	memset(&SPI2Handle, 0, sizeof(SPI2Handle));

	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_DeviceMode 		= SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig 			= SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_SclkSpeed 			= SPI_SCLK_SPEED_DIV8;					//Generates SCLK 2MHz
	SPI2Handle.SPIConfig.SPI_DFF 				= SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL			 	= SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA 				= SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM 				= SPI_SSM_DI;

	SPI_Init(&SPI2Handle);
}

int main(void) {
	uint8_t dummyWrite = 0xff;
	uint8_t dummyRead;

	GPIO_ButtonInit();

	//This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/* making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * i.e when SPE=1 , NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2,ENABLE);

	/* SPI2 IRQ configurations */
	SPI_IRQInterruptConfig(IRQ_NO_SPI2,ENABLE);

	while(1) {
		/* Wait till button is pressed */
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//printf("SPI communication started!\n");

		/* 200ms delay */
		delay250ms();

		/* Enable SPI2 peripheral */
		SPI_PeripheralControl(SPI2, ENABLE);

		/* Send SPI data: CMD LED Control */
		uint8_t commandCode = COMMAND_LED_CTRL;

		/* Send command */
		while(SPI_TransmitDataIT(&SPI2Handle, &commandCode, 1) != SPI_READY);

		/* Dummy read to clear off the RXNE */
		while(SPI_ReceiveDataIT(&SPI2Handle, &dummyRead, 1) != SPI_READY);

		/* Send dummy bits (byte) to fetch the response from slave */
		while(SPI_TransmitDataIT(&SPI2Handle, &dummyWrite, 1) != SPI_READY);

		/* Receive Acknowledgment byte */
		uint8_t ackByte;
		uint8_t args[2];
		while(SPI_ReceiveDataIT(&SPI2Handle, &ackByte, 1) != SPI_READY);

		/* Verify response from SPI slave */
		if(SPI_VerityRespone(ackByte)) {
			/* Send arguments */
			args[0] = LED_PIN;
			args[1] = LED_ON;
			while(SPI_TransmitDataIT(&SPI2Handle, args, 2) != SPI_READY);
			//printf("Command LED executed!\n");
		}
		/* End of CMD LED Control */

		/* Send SPI data: CMD Sensor Read */
		commandCode = COMMAND_SENSOR_READ;

		/* Send command */
		while(SPI_TransmitDataIT(&SPI2Handle, &commandCode, 1) != SPI_READY);

		/* Dummy read to clear off the RXNE */
		while(SPI_ReceiveDataIT(&SPI2Handle, &dummyRead, 1) != SPI_READY);

		/* Send dummy bits (byte) to fetch the response from slave */
		while(SPI_TransmitDataIT(&SPI2Handle, &dummyWrite, 1) != SPI_READY);

		/* Receive Acknowledgment byte */
		while(SPI_ReceiveDataIT(&SPI2Handle, &ackByte, 1) != SPI_READY);

		/* Verify response from SPI slave */
		if(SPI_VerityRespone(ackByte)) {
			/* Send arguments */
			args[0] = ANALOG_PIN0;
			while(SPI_TransmitDataIT(&SPI2Handle, args, 1) != SPI_READY);
			//printf("Command Sensor Read executed!\n");

			/* Dummy read to clear off the RXNE */
			while(SPI_ReceiveDataIT(&SPI2Handle, &dummyRead, 1) != SPI_READY);

			/* Send dummy bits (byte) to fetch the response from slave */
			while(SPI_TransmitDataIT(&SPI2Handle, &dummyWrite, 1) != SPI_READY);

			/* Wait for Slave to be ready with data */
			delay250ms();

			/* Receive Sensor Data from Slave */
			uint8_t analogRead;
			while(SPI_ReceiveDataIT(&SPI2Handle, &analogRead, 1) != SPI_READY);
			//printf("Analog read value: %d\n",analogRead);
		}
		/* End of CMD Sensor Read */

		/* Confirm SPI2 not busy */
		SPI_GetFlagStatus(SPI2, SPI_SR_BSY);

		/* Disable SPI2 peripheral */
		SPI_PeripheralControl(SPI2, DISABLE);

		//TODO: Implement rest of commands to test if needed
		//printf("SPI communication closed!\n");
	}

	return 0;
}

void SPI2_IRQHandler(void)
{

	SPI_IRQHandling(&SPI2handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv) {
	if (AppEv == SPI_EVENT_TX_CMPLT) {
		//printf("Tx is complete!\n");
	}
	else if(AppEv == SPI_EVENT_RX_CMPLT) {
		//printf("Rx is complete!\n");
	}
	else if(AppEv == SPI_EVENT_OVR_ERR) {
		//printf("OVR Error triggered!\n");
	}
}
