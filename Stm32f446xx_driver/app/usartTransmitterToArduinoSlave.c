/*
 * usartTransmitterToArduinoSlave.c
 *
 *  Created on: Mar 15, 2020
 *      Author: AD
 */
/*********************************************************************************************************************
 ************************************************** PIN CONFIGURE ****************************************************
 *********************************************************************************************************************
 * User Button 		PC13
 *
 * Master (STM32F446xx)
 * PA2-9/PB10		>>		USART_Tx
 * PA3-10/PB11		>>		USART_Rx
 * Alternate function: 7
 *
 * Slave (Arduino Uno R3)
 * Pin A0	>>		Rx
 * Pin A1	>>		Tx
 * *******************************************************************************************************************
 * *******************************************************************************************************************
 */
#include "string.h"
#include "stm32f446xx.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_gpio_driver.h"

USART_Handle_t USART2Handle;
char msg[1024] = "UART Tx testing...\n\r";

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

void USART2_GPIOInits(void) {
	GPIO_Handle_t USART2Pins;
	memset(&USART2Pins, 0, sizeof(USART2Pins));

	USART2Pins.pGPIOx = GPIOA;
	USART2Pins.GPIO_PinConfig.GPIO_PinMode 			= GPIO_MODE_ALTFN;
	USART2Pins.GPIO_PinConfig.GPIO_PinAltFunMode 	= 7;
	USART2Pins.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_PP;
	USART2Pins.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PIN_PU;
	USART2Pins.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;

	USART2Pins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_9;					//USART_Tx
	GPIO_Init(&USART2Pins);

	USART2Pins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_10;					//USART_Rx
	GPIO_Init(&USART2Pins);
}

void USART2_Inits(void) {

//	memset(&USART2Handle, 0, sizeof(USART2Handle));

	USART2Handle.pUSARTx = USART1;
	USART2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&USART2Handle);
}

int main(void) {
	GPIO_ButtonInit();
	USART2_GPIOInits();
	USART2_Inits();
	USART_PeripheralControl(USART1, ENABLE);

	while(1) {
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay250ms();
		USART_SendData(&USART2Handle, (uint8_t*)msg, strlen(msg));
	}
}
