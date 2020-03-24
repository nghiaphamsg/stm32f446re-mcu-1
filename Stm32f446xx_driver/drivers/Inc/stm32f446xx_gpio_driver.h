/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Feb 11, 2020
 *      Author: $NghiaPham$
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/									BIT POSITION DEFINITIONS OF GPIO PERIPHERAL									_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*
 * @GPIO_PIN_NUMBER
 * GPIO Pin Numbers
 */
#define GPIO_PIN_NO_0				  		0
#define GPIO_PIN_NO_1				  		1
#define GPIO_PIN_NO_2						2
#define GPIO_PIN_NO_3				  		3
#define GPIO_PIN_NO_4				  		4
#define GPIO_PIN_NO_5				  		5
#define GPIO_PIN_NO_6				  		6
#define GPIO_PIN_NO_7				  		7
#define GPIO_PIN_NO_8				  		8
#define GPIO_PIN_NO_9						9
#define GPIO_PIN_NO_10						10
#define GPIO_PIN_NO_11						11
#define GPIO_PIN_NO_12						12
#define GPIO_PIN_NO_13						13
#define GPIO_PIN_NO_14						14
#define GPIO_PIN_NO_15						15
#define GPIO_PIN_NO_16						16

/*
 * @GPIO_PIN_MODE
 * GPIO Pin Possible Modes
 */
#define GPIO_MODE_IN				  		0				/*! GPIO Pin as General Purpose Input 		*/
#define GPIO_MODE_OUT				  		1				/*! GPIO Pin as General Purpose Output 		*/
#define GPIO_MODE_ALTFN						2				/*! GPIO Pin as Alternate Function 			*/
#define GPIO_MODE_ANALOG					3				/*! GPIO Pin as Analog 						*/
#define GPIO_MODE_IT_FT						4				/*! GPIO Pin as Falling Edge				*/
#define GPIO_MODE_IT_RT						5				/*! GPIO Pin as Rising Edge					*/
#define GPIO_MODE_IT_RFT					6				/*! GPIO Pin as Rising Falling Edge			*/

/*
 * @GPIO_PIN_OPT
 * GPIO Pin Possible Output Types
 */
#define GPIO_OP_TYPE_PP						0				/*! GPIO Output Type Push-Pull				*/
#define GPIO_OP_TYPE_OD						1				/*! GPIO Output Type Open-Drain 			*/

/*
 * @GPIO_PIN_SPEED
 * GPIO Pin Possible Output Speed
 */
#define GPIO_SPEED_LOW						0				/*! GPIO Speed Low 							*/
#define GPIO_SPEED_MEDIUM					1				/*! GPIO Speed Medium 						*/
#define GPIO_SPEED_FAST						2				/*! GPIO Speed Fast 						*/
#define GPIO_SPEED_HIGH						3				/*! GPIO Speed High 						*/

/*
 * @GPIO_PIN_PUPD
 * GPIO Pin Pull-up And Pull-down configuration macros
 */
#define GPIO_NO_PUPD						0				/*! No Pull-up Pull-down resistor enabled 	*/
#define GPIO_PIN_PU							1				/*! Pull up resistor enabled 				*/
#define GPIO_PIN_PD							2				/*! Pull down resistor enabled 				*/

/*
 * Configuration structure for GPIOx peripheral
 */
typedef struct
{
	uint8_t GPIO_PinNumber;									/*! Possible value from @GPIO_PIN_NUMBER  	*/
	uint8_t GPIO_PinMode;									/*! Possible value from @GPIO_PIN_MODE  	*/
	uint8_t GPIO_PinSpeed;									/*! Possible value from @GPIO_PIN_SPEED  	*/
	uint8_t GPIO_PinPuPdControl;							/*! Possible value from @GPIO_PIN_PUPD  	*/
	uint8_t GPIO_PinOPType;									/*! Possible value from @GPIO_PIN_OPT	  	*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t 		*pGPIOx;
	GPIO_PinConfig_t 	GPIO_PinConfig;
}GPIO_Handle_t;


//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/												APIs SUPPORT BY THIS DRIVERS									_/
//_/							FOR MORE INFORMATION ABOUT THE APIs CHECK THE FUCTION DEFINITIONs					_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

//------------------------------------------------------------------
// PERIPHERAL CLOCK SETUP
//------------------------------------------------------------------
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//------------------------------------------------------------------
// INITIALIZE AND DEINITIALIZE
//------------------------------------------------------------------
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//------------------------------------------------------------------
// DATA READ AND WRITE
//------------------------------------------------------------------
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//------------------------------------------------------------------
// IRQ CONFIGUARATION AND ISR HANDLING
//------------------------------------------------------------------
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_ISRHandling(uint8_t PinNumber);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
