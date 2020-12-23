/*
 * buttonInterrupt.c
 *
 *  Created on: Feb 15, 2020 Author: $NghiaPham$
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

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

	//Configuration enable GPIO Button with trigger an interrupt whenever button
	//press
	GPIOBtn.pGPIOx = GPIOC;

	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PIN_PU;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GPIOBtn);

	//IRQ Config
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);

}

void EXTI15_10_IRQHandler(void) {

	GPIO_ISRHandling(GPIO_PIN_NO_13);
	while(1) {
		delay();
		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, ENABLE);
		delay();
		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, DISABLE);
	}
}
