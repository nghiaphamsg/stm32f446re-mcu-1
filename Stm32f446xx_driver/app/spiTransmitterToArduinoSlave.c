/*
 * spiTransmitterToArduinoSlave.c
 *
 *  Created on: Feb 23, 2020
 *      Author: AD
 */

/*********************************************************************************************************************
 ************************************************** PIN CONFIGURE ****************************************************
 *********************************************************************************************************************
 * User Button 		PC13
 *
 * Master (STM32F446xx)
 * PB12		>>		SPI2_NSS				PA4		>>		SPI1_NSS
 * PB13		>>		SPI2_SCLK				PA5		>>		SPI1_SCLK
 * PB14		>>		SPI2_MISO				PA6		>>		SPI1_MISO
 * PB15		>>		SPI2_MOSI				PA77	>>		SPI1_MOSI
 * Alternate function: 5
 *
 * Slave (Arduino Uno R3)
 * Pin 13	>>		SCLK
 * Pin 12	>>		MISO
 * Pin 11	>>		MOSI
 * Pin 10	>>		SS
 * *******************************************************************************************************************
 * *******************************************************************************************************************
 */
#include <string.h>

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"

void delay250ms(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
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

//	SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_14;					//SPI2_MISO
//	GPIO_Init(&SPIPins);

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
	char dataTranmitter[] = "Hello every body i'm neko bot";

	GPIO_ButtonInit();
	SPI2_GPIOInits();																	//Initialize GPIO pins behave as SPI2
	SPI2_Inits();																		//Initialize SPI2 with parameters
	SPI_SSOEConfig(SPI2, ENABLE);														//SSOE Enable
	while (1) {
		while ( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));							//When push the button then Master transmitter data to Slave
		delay250ms();

		SPI_PeripheralControl(SPI2, ENABLE);												//SPI Enable

		uint8_t dataLen = strlen(dataTranmitter);
		SPI_TransmitData(SPI2, &dataLen, 1);

		SPI_TransmitData(SPI2, (uint8_t*)dataTranmitter, strlen(dataTranmitter));			//Send data function

		while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));										//Lets confirm SPI is not busy
		SPI_PeripheralControl(SPI2,DISABLE);												//Disable the SPI2 peripheral
	}

	return 0;
}

