/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Feb 11, 2020
 *      Author: $NghiaPham$
 */


#include "stm32f446xx_gpio_driver.h"

/*
 * Peripheral clock setup
 */
/*********************************************************************************
 * @fn				- GPIO_PeriClockControl
 * @brief			- This function disable or enable peripheral clock for the give GPIO port
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @return			- None
 * @note			- None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE){
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}
	}else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		}
	}
}

/*
 * Initialize and De-initialize
 */
/*********************************************************************************
 * @fn				- GPIO_Init
 * @brief			- This function initialize the give GPIO port and give GPIO pin
 * @param[in]		- Handle structure for a GPIO pin
 * @return			- None
 * @note			- Configure the mode of GPIO Pin >> Configure the speed >> Configure the pull-up pull-down >> Configure the output type >> Configure the alt functionality
 */
 void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
 {
	 uint32_t temp = 0;
	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	 //Configure the mode of GPIO Pin
	 if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		 //Non-interrupt mode
		 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2) );
		 pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2) );
		 pGPIOHandle->pGPIOx->MODER |= temp;
	 }else {
		 //Interrupt mode
		 if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			 //Configure the FTSR
			 EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			 //Clear the corresponding RTSR
			 EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		 }
		 else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			 //Configure the RTSR
			 EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			 //Clear the corresponding FTSR
			 EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		 }
		 else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			 //Configure the FTSR & RTSR
			 EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			 EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		 }

		//Configure the GPIO port selection in SYSCFG_EXTICR
		 uint8_t temp1, temp2, portcode;
		 temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		 temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		 portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		 SYSCFG_PCLK_EN();
		 SYSCFG->EXTICR[temp1] |= (portcode << (4 * temp2));

		//Enable EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

	 }

	 //2. Configure the speed
	 temp = 0;
	 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber *2) );
	 pGPIOHandle->pGPIOx->OSPEEDER &= ~( 0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2) );
	 pGPIOHandle->pGPIOx->OSPEEDER |= temp;

	 //3. Configure the pull-up pull-down
	 temp = 0;
	 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2) );
	 pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2) );
	 pGPIOHandle->pGPIOx->PUPDR |= temp;

	 //4. Configure the output type
	 temp = 0;
	 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	 pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	 pGPIOHandle->pGPIOx->OTYPER |= temp;

	 temp = 0;
	 //5. Configure the alt functionality
	 if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		 uint8_t temp1, temp2;
		 temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		 temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		 pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		 pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	 }
 }

 /*********************************************************************************
  * @fn				- GPIO_DeInit
  * @brief			- This function de-initialize the give GPIO port and give GPIO pin
  * @param[in]		- Base address of the GPIO peripheral
  * @return			- None
  * @note			- Use RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
  */
 void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
 {
	 if (pGPIOx == GPIOA) {
		 GPIOA_REG_RESET();
	 }
	 else if (pGPIOx == GPIOB) {
		 GPIOB_REG_RESET();
	 }
	 else if (pGPIOx == GPIOC) {
		 GPIOC_REG_RESET();
	 }
	 else if (pGPIOx == GPIOD) {
		 GPIOD_REG_RESET();
	 }
 }

 /*
  * Data Read and Write
  */
 /*********************************************************************************
  * @fn				- GPIO_ReadFromInputPin
  * @brief			- Read the corresponding bit position in the input data register
  * @param[in]		- Base address of the GPIO peripheral
  * @param[in]		- Pin number which need read
  * @return			- Pin of value 0 or 1
  * @note			- None
  */
 uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
 {
	 uint8_t ReadInputPinValue;
	 ReadInputPinValue = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
	 return ReadInputPinValue;
 }

 /*********************************************************************************
  * @fn				- GPIO_ReadFromInputPort
  * @brief			- Read the corresponding bit position in the input port data register
  * @param[in]		- Base address of the GPIO peripheral
  * @return			- Port of value
  * @note			- None
  */
 uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
 {
	 uint16_t ReadInputPortValue;
	 ReadInputPortValue = (uint16_t)pGPIOx->IDR;
	 return ReadInputPortValue;
 }

 /*********************************************************************************
   * @fn			- GPIO_WriteToOutputPin
   * @brief			-
   * @param[in]		- Base address of the GPIO peripheral
   * @param[in]		- Pin number which need write
   * @param[in]		- Value to write
   * @return		- None
   * @note			- None
   */
 void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
 {
	 if(Value == GPIO_PIN_SET) {
		 pGPIOx->ODR |= (1 << PinNumber );
	 } else{
		 pGPIOx->ODR &= ~(1 << PinNumber );
	 }
 }

 /*********************************************************************************
   * @fn			- GPIO_WriteToOutputPort
   * @brief			-
   * @param[in]		- Base address of the GPIO peripheral
   * @param[in]		- Value to write
   * @return		- None
   * @note			- None
   */
 void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
 {
	 pGPIOx->ODR = Value;
 }

 /*********************************************************************************
   * @fn			- GPIO_ToggleOutputPin
   * @brief			-
   * @param[in]		- Base address of the GPIO peripheral
   * @param[in]		- Pin number which need write
   * @return		- None
   * @note			- None
   */
 void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
 {
	 pGPIOx->ODR ^= ( 1 << PinNumber );
 }

 /*
  * IRQ Configuration and ISR handling
  */
 /*********************************************************************************
   * @fn			- GPIO_IRQConfig
   * @brief			-
   * @param[in]		- IRQ number
   * @param[in]		- IRQ priority
   * @param[in]		- ENABLE or DISABLE macros
   * @return		- None
   * @note			- None
   */
 void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
 {
	 if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ISER1 |= ( 1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= ( 1 << IRQNumber % 64);
		}
	} else {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 |= ( 1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= ( 1 << IRQNumber % 64);
		}
	}
 }

 /*********************************************************************************
    * @fn			- GPIO_IRQPriorityConfig
    * @brief			-
    * @param[in]		- IRQ number
    * @param[in]		- IRQ priority
    * @return		- None
    * @note			- None
    */
 void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	 uint8_t iprx, iprxSection, shiftAmount;
	 iprx = IRQNumber / 4;
	 iprxSection = IRQNumber % 4;
	 shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);
	 *(NVIC_PR_BASE_ADDR + iprx ) = (IRQPriority << shiftAmount);
 }

 /*********************************************************************************
   * @fn			- GPIO_ISRHandling
   * @brief			-
   * @param[in]		- Pin number which need write
   * @return		- None
   * @note			- None
   */
 void GPIO_ISRHandling(uint8_t PinNumber)
 {
	//clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}
 }
