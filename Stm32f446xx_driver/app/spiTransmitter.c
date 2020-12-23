/*
 * spiTransmitter.c
 *
 *  Created on: Feb 22, 2020
 *      Author: AD
 *
 *********************************************************************************************************************
 ************************************************** PIN CONFIGURE ****************************************************
 *********************************************************************************************************************
 * PB12		>>		SPI2_NSS
 * PB13		>>		SPI2_SCK
 * PB14		>>		SPI2_MISO
 * PB15		>>		SPI2_MOSI
 * Alternate function: 5
 *
 */
#include <string.h>

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"

void SPI2_GPIOInits(void) {
	GPIO_Handle_t SPIPins;
	memset(&SPIPins, 0, sizeof(SPIPins));

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode 	= 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_FAST;


	SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_13;					//SPI2_SCK
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_15;					//SPI2_MOSI
	GPIO_Init(&SPIPins);

	//SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_14;					//SPI2_MISO
	//GPIO_Init(&SPIPins);

	//SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_12;					//SPI2_NSS
	//GPIO_Init(&SPIPins);
}

void SPI2_Inits(void) {
	SPI_Handle_t SPI2Handle;
	memset(&SPI2Handle, 0, sizeof(SPI2Handle));

	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_DeviceMode 		= SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig 			= SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_SclkSpeed 			= SPI_SCLK_SPEED_DIV8;					//Generates SCLK 16/8 = 2MHz
	SPI2Handle.SPIConfig.SPI_DFF 				= SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL			 	= SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA 				= SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM 				= SPI_SSM_EN;

	SPI_Init(&SPI2Handle);
}


int main(void) {
	char dataTranmitter[] = "Hello world";
	SPI2_GPIOInits();																	//Initialize GPIO pins behave as SPI2
	SPI2_Inits();																		//Initialize SPI2 with parameters
	SPI_SSIConfig(SPI2, ENABLE);														//Enable SSI
	SPI_PeripheralControl(SPI2, ENABLE);												//SPI Enable
	SPI_TransmitData(SPI2, (uint8_t*)dataTranmitter, strlen(dataTranmitter));			//Send data function
	while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );										//Lets confirm SPI is not busy
	SPI_PeripheralControl(SPI2,DISABLE);												//Disable the SPI2 peripheral

	while (1);
	return 0;
}
