/*
 * spiCmdHandling.c
 *
 *  Created on: Feb 24, 2020
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
 * Pin 10	>>		SSp;[ m
 * Pin 9	>>		LED
 * *******************************************************************************************************************
 * *******************************************************************************************************************
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

	SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_14;					//SPI2_MISO
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_12;					//SPI2_NSS
	GPIO_Init(&SPIPins);
}

uint8_t SPI_VerityRespone(uint8_t ackByteRespone) {
	if (ackByteRespone == (uint8_t)0xF5) {
		return 1;
	}
	return 0;
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

void commonCmd(uint8_t cmdCode, uint8_t portSlave, uint8_t cmdMess) {
//	printf("debug_0001\n");
	uint8_t dummyByteWrite = 0xff;
	uint8_t dummyByteRead;

	uint8_t ackByte, args[2];

	SPI_TransmitData(SPI2, &cmdCode, 1);
	SPI_ReceiveData(SPI2, &dummyByteRead, 1);

	//Send some dummy bits to fetch the response from the slave
	SPI_TransmitData(SPI2, &dummyByteWrite, 1);

	//Read the ACK byte received
	SPI_ReceiveData(SPI2, &ackByte, 1);

	//Check ACK and send messages command to slave
	if (SPI_VerityRespone(ackByte)) {
		switch (cmdCode) {
			case COMMAND_LED_CTRL:
				if (cmdMess == LED_ON){
					args[0] = portSlave;
					args[1] = LED_ON;
					SPI_TransmitData(SPI2, args, 2);
//					printf("COMMAND_LED_CTRL executed pin %d turn on\n", portSlave);
				}
				else {
					args[0] = portSlave;
					args[1] = LED_OFF;
					SPI_TransmitData(SPI2, args, 2);
//					printf("COMMAND_LED_CTRL executed pin turn off %d\n", portSlave);
				}
				break;
			case COMMAND_SENSOR_READ:
				switch (portSlave) {
					uint8_t analogRead;
					case ANALOG_PIN0:
						args[0] = portSlave;
						SPI_TransmitData(SPI2, args, 1);
						SPI_ReceiveData(SPI2, &dummyByteRead, 1);
						delay250ms();
						SPI_TransmitData(SPI2, &dummyByteWrite, 1);
						SPI_ReceiveData(SPI2, &analogRead, 1);
//						printf("COMMAND_SENSOR_READ %d\n",analogRead);
						break;
					case ANALOG_PIN1:
						args[0] = portSlave;
						SPI_TransmitData(SPI2, args, 1);
						SPI_ReceiveData(SPI2, &dummyByteRead, 1);
						delay250ms();
						SPI_TransmitData(SPI2, &dummyByteWrite, 1);
						SPI_ReceiveData(SPI2, &analogRead, 1);
//						printf("COMMAND_SENSOR_READ %d\n",analogRead);
						break;
					case ANALOG_PIN2:
						args[0] = portSlave;
						SPI_TransmitData(SPI2, args, 1);
						SPI_ReceiveData(SPI2, &dummyByteRead, 1);
						delay250ms();
						SPI_TransmitData(SPI2, &dummyByteWrite, 1);
						SPI_ReceiveData(SPI2, &analogRead, 1);
//						printf("COMMAND_SENSOR_READ %d\n",analogRead);
						break;
					case ANALOG_PIN3:
						args[0] = portSlave;
						SPI_TransmitData(SPI2, args, 1);
						SPI_ReceiveData(SPI2, &dummyByteRead, 1);
						delay250ms();
						SPI_TransmitData(SPI2, &dummyByteWrite, 1);
						SPI_ReceiveData(SPI2, &analogRead, 1);
//						printf("COMMAND_SENSOR_READ %d\n",analogRead);
						break;
					case ANALOG_PIN4:
						args[0] = portSlave;
						SPI_TransmitData(SPI2, args, 1);
						SPI_ReceiveData(SPI2, &dummyByteRead, 1);
						delay250ms();
						SPI_TransmitData(SPI2, &dummyByteWrite, 1);
						SPI_ReceiveData(SPI2, &analogRead, 1);
//						printf("COMMAND_SENSOR_READ %d\n",analogRead);
						break;
					default:
						break;
				}
			case COMMAND_LED_READ:
				args[0] = portSlave;
				SPI_TransmitData(SPI2, args, 1);
				SPI_ReceiveData(SPI2, &dummyByteRead, 1);
				delay250ms();
				SPI_TransmitData(SPI2, &dummyByteWrite, 1);
				uint8_t ledStatus;
				SPI_ReceiveData(SPI2, &ledStatus, 1);
//				printf("COMMAND_READ_LED %d\n",ledStatus);
				break;
			default:
				break;
		}
	}
}

int main(void) {
//	initialise_monitor_handles();
	GPIO_ButtonInit();
	SPI2_GPIOInits();																	//Initialize GPIO pins behave as SPI2
	SPI2_Inits();																		//Initialize SPI2 with parameters
	SPI_SSOEConfig(SPI2, ENABLE);														//SSOE Enable
	while (1) {
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));							//When push the button then Master transmitter data to Slave
		delay250ms();

		SPI_PeripheralControl(SPI2, ENABLE);											//SPI Enable

		//1.Command for control led of arduino
		commonCmd((uint8_t)COMMAND_LED_CTRL, (uint8_t)LED_PIN, (uint8_t)LED_ON);

		//2.Command for read analog value of arduino
		//commonCmd((uint8_t)COMMAND_SENSOR_READ, (uint8_t)ANALOG_PIN0, 0);

		//3.Command for read status pin of arduino
		//commonCmd((uint8_t)COMMAND_LED_READ, (uint8_t)LED_PIN, 0);


		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );										//Lets confirm SPI is not busy
		SPI_PeripheralControl(SPI2,DISABLE);												//Disable the SPI2 peripheral
//		printf("SPI Communication Closed\n");
	}

	return 0;
}

