/*
 * stm32f446_rcc_driver.c
 *
 *  Created on: Mar 15, 2020
 *      Author: AD
 */
#include "stm32f446xx.h"
#include "stm32f446xx_rcc_driver.h"

uint16_t AHB_PreScaler[8]	= {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4]	= {2, 4, 8, 16};

uint32_t RCC_Get2PCLKValue(void) {
	uint32_t pclk1, systemClk;
	uint8_t clkSrc, ahbPrescaler, ahbClk, apbPrescaler, apbClk;

	//Configure System clock switch status
	clkSrc = ((RCC->CFGR >> 2) & 0x3);

	if (clkSrc == 0) {
		systemClk = 16000000;					// HSI oscillator used as the system clock
	}
	else if (clkSrc == 1) {
		systemClk = 8000000;					//HSE oscillator used as the system clock
	}
	else if (clkSrc == 2) {
		//PLL used as the system clock
	}
	else if (clkSrc == 3) {
		//PLL_R used as the system clock
	}

	//Configure AHB Pre-scaler
	ahbPrescaler = ((RCC->CFGR >> 4) & 0xF);

	if (ahbPrescaler < 8) {
		ahbClk = 1;
	}
	else {
		ahbClk = AHB_PreScaler[ahbPrescaler-8];
	}

	//Configure APB Low speed Pre-scaler (APB1)
	apbPrescaler = ((RCC->CFGR >> 10) & 0x7);

	if (apbPrescaler < 4) {
		apbClk = 1;
	}
	else {
		apbClk = APB1_PreScaler[apbPrescaler-4];
	}

	pclk1 = (systemClk/ahbClk)/apbClk;

	return pclk1;
}
