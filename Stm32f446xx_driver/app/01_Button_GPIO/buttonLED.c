/*
 * buttonLED.c
 *
 *  Created on: Feb 12, 2020
 *      Author: AD
 */
/*********************************************************************************************************************
 ************************************************** PIN CONFIGURE ****************************************************
 *********************************************************************************************************************
 * User Button 		PC13
 * GPIO LED			PA5
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include <string.h>

#define HIGH			1
#define BTN_PRESSED 	HIGH

void delay(void)
{
	for( uint32_t i = 0; i < 500000/2 ; i++);
}

int main(void)
{
	GPIO_Handle_t GPIOLed, GPIOBtn;
	memset(&GPIOLed, 0, sizeof(GPIOLed));
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	//Configuration enable GPIO LED
	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_5;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIOLed);

	//Configuration enable GPIO Button
	GPIOBtn.pGPIOx = GPIOC;

	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GPIOBtn);

	while(1)
	{
		//No press = 1; press = 0
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != BTN_PRESSED)
		{
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, ENABLE);
		}else {
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, DISABLE);
		}
	}
	return 0;
}
